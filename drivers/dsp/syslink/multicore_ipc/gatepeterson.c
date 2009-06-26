/*
 *  gatepeterson.h
 *
 *  The Gate Peterson Algorithm for mutual exclusion of shared memory.
 *  Current implementation works for 2 processors.
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 *  This package is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <gt.h>
#include <multiproc.h>
#include <nameserver.h>
#include <gatepeterson.h>

/* IPC stubs */

/*
 *  Name of the reserved NameServer used for GatePeterson
 */
#define GATEPETERSON_NAMESERVER  	"GatePeterson"
#define GATEPETERSON_BUSY       	 1
#define GATEPETERSON_FREE       	 0
#define GATEPETERSON_VERSION    	 1
#define GATEPETERSON_CREATED     	 0x08201997 /* Stamp to indicate GP
							was created here */
#define MAX_GATEPETERSON_NAME_LEN 	 32

/*
 *  structure for GatePeterson module state
 */
struct gatepeterson_moduleobject {
	bool is_init;
	void *nshandle;
	struct list_head obj_list;
	struct mutex *list_lock; /* Lock for obj list */
	struct gatepeterson_config cfg;
	struct gatepeterson_config default_cfg;
};

/*
 *  Structure defining attribute parameters for the Gate Peterson module
 */
struct gatepeterson_attrs {
	u32 version;
	u32 status;
	u16 creator_proc_id;
	u16 opener_proc_id;
};

/*
 *  Structure defining internal object for the Gate Peterson
 */
struct gatepeterson_obj {
	struct list_head elem;
	volatile struct gatepeterson_attrs *attrs; /* Instance attr */
	volatile void *flag[2]; /* Falgs for processors */
	volatile void *turn; /* Indicates whoes turn it is now? */
	u32 nested; /* Counter to track nesting */
	u8 self_id; /* Self identifier */
	u8 other_id; /* Other's identifier */
	void *local_gate; /* Local lock handle */
	struct gatepeterson_params params;
	u32 key; /* Value returned by local lock */
	void *top;  /* Pointer to the top Object */
	u32 ref_count; /* Local reference count */
};

/*
 *  Structure defining object for the Gate Peterson
 */
struct gatepeterson_object {
	void *(*lock_get_knl_handle)(void **handle); /* Pointer to
					Kernl object will be returned */
	u32 (*enter)(void *handle); /* Function to enter GP */
	void (*leave)(void *handle, u32 key); /* Function to leave GP */
	struct gatepeterson_obj *obj; /* Pointer to GP internal object */
};

/*
 *  Variable for holding state of the GatePeterson module
 */
struct gatepeterson_moduleobject gatepeterson_state = {
	.obj_list     = LIST_HEAD_INIT(gatepeterson_state.obj_list),
	.default_cfg.max_name_len        = MAX_GATEPETERSON_NAME_LEN,
	.default_cfg.default_protection  = GATEPETERSON_PROTECT_PROCESS,
	.default_cfg.use_nameserver      = false,
	.default_cfg.max_runtime_entries = ~(0),
	.default_cfg.name_table_gate     = NULL,
};

/*
 * ======== gatepeterson_get_config ========
 *  Purpose:
 *  This will get the default configuration parameters for gatepeterson
 *  module
 */
int gatepeterson_get_config(struct gatepeterson_config *config)
{
	s32 retval = 0;

	if (WARN_ON(config == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if (gatepeterson_state.is_init != true)
		memcpy(config, &gatepeterson_state.default_cfg,
					sizeof(struct gatepeterson_config));
	else
		memcpy(config, &gatepeterson_state.cfg,
					sizeof(struct gatepeterson_config));
	return 0;

exit:
	return retval;
}
EXPORT_SYMBOL(gatepeterson_get_config);

/*
 * ======== gatepeterson_setup ========
 *  Purpose:
 *  This will setup the GatePeterson module
 */
int gatepeterson_setup(const struct gatepeterson_config *config)
{
	struct nameserver_params params;
	void *nshandle = NULL;
	s32 retval = 0;
	s32 ret;

	BUG_ON(config == NULL);
	if (WARN_ON(config->max_name_len == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	if (likely((config->use_nameserver == true))) {
		retval = nameserver_get_params(NULL, &params);
		params.max_value_len = sizeof(u32);
		params.max_name_len = config->max_name_len;
		params.gate_handle = config->name_table_gate; /* FIX ME */
		params.max_runtime_entries = config->max_runtime_entries;
		/* Create the nameserver for modules */
		nshandle = nameserver_create(GATEPETERSON_NAMESERVER, &params);
		if (nshandle == NULL)
			goto exit;

		gatepeterson_state.nshandle = nshandle;
	}

	memcpy(&gatepeterson_state.cfg, config,
					sizeof(struct gatepeterson_config));
	gatepeterson_state.list_lock = kmalloc(sizeof(struct mutex),
								GFP_KERNEL);
	if (gatepeterson_state.list_lock == NULL) {
		if ((likely(config->use_nameserver == true)))
			ret = nameserver_delete(&gatepeterson_state.nshandle);

		retval = -ENOMEM;
		goto exit;
	}

	mutex_init(gatepeterson_state.list_lock);
	gatepeterson_state.is_init = true;
	return 0;

exit:
	printk(KERN_ERR	"gatepeterson_setup failed status: %x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_setup);

/*
 * ======== gatepeterson_destroy ========
 *  Purpose:
 *  This will destroy the GatePeterson module
 */
int gatepeterson_destroy(void)

{
	struct mutex *lock = NULL;
	s32 retval = 0;

	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -ENODEV;
		goto exit;
	}

	/* If  an entry exist, do not proceed  */
	if (!list_empty(&gatepeterson_state.obj_list)) {
		retval = -EBUSY;
		goto exit;
	}

	retval = mutex_lock_interruptible(gatepeterson_state.list_lock);
	if (retval != 0)
		goto exit;

	if (likely(gatepeterson_state.cfg.use_nameserver == true)) {
		retval = nameserver_delete(&gatepeterson_state.nshandle);
		if (unlikely(retval != 0))
			goto exit;
	}

	lock = gatepeterson_state.list_lock;
	gatepeterson_state.list_lock = NULL;
	memset(&gatepeterson_state.cfg, 0, sizeof(struct gatepeterson_config));
	gatepeterson_state.is_init = false;
	mutex_unlock(lock);
	kfree(lock);
	return 0;

exit:;
	printk(KERN_ERR "gatepeterson_destroy failed status:%x\n", retval);
	return retval;

}
EXPORT_SYMBOL(gatepeterson_destroy);

/*
 * ======== gatepeterson_params_init ========
 *  Purpose:
 *  This will  Initialize this config-params structure with
 *  supplier-specified defaults before instance creation
 */
int gatepeterson_params_init(struct gatepeterson_params *params)
{
	BUG_ON(params == NULL);

	params->shared_addr = 0;
	params->shared_addr_size = 0;
	params->name = NULL;
	params->local_protection = GATEPETERSON_PROTECT_PROCESS;
	params->opener_proc_id = MULTIPROC_INVALIDID;
	return 0;
}
EXPORT_SYMBOL(gatepeterson_params_init);

/*
 * ======== gatepeterson_create ========
 *  Purpose:
 *  This will creates a new instance of GatePeterson module
 */
void *gatepeterson_create(const struct gatepeterson_params *params)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	s32 retval = 0;
	u32 shaddrsize;
	u32 len;
	s32 status;
	void *entry = NULL;

	BUG_ON(params == NULL);
	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -ENODEV;
		goto exit;
	}

	shaddrsize = gatepeterson_shared_memreq(params);
	if (WARN_ON(params->shared_addr == NULL ||
		params->shared_addr_size < shaddrsize)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = kmalloc(sizeof(struct gatepeterson_object), GFP_KERNEL);
	if (handle == NULL) {
		retval = -ENOMEM;
		goto handle_alloc_fail;
	}

	obj = kmalloc(sizeof(struct gatepeterson_obj), GFP_KERNEL);
	if (obj == NULL) {
		retval = -ENOMEM;
		goto obj_alloc_fail;
	}

	if (likely(gatepeterson_state.cfg.use_nameserver == true &&
						params->name != NULL)) {
		len = strlen(params->name) + 1;
		obj->params.name = kmalloc(len, GFP_KERNEL);
		if (obj->params.name == NULL) {
			retval = -ENOMEM;
			goto name_alloc_fail;
		}

		strncpy(obj->params.name, params->name, len);
		entry = nameserver_add_uint32(gatepeterson_state.nshandle,
						params->name,
						(u32)(params->shared_addr));
		if (entry == NULL)
			goto ns_add32_fail;
	}

	handle->obj = obj;
	handle->enter = gatepeterson_enter;
	handle->leave = gatepeterson_leave;
	handle->lock_get_knl_handle = gatepeterson_get_knl_handle;

	/* assign the memory with proper cache line padding */
	obj->attrs = (struct gatepeterson_attrs *)params->shared_addr;
	obj->flag[0] = ((void *)(((u32)obj->attrs) + 128));
	obj->flag[1] = ((void *)(((u32)obj->flag[0]) + 128));
	obj->turn    = ((void *)(((u32)obj->flag[1]) + 128)); /* TBD: Fixme */
	obj->self_id                = 0; /* Creator has selfid set to 0 */
	obj->other_id               = 1;
	obj->nested                 = 0;
	obj->ref_count              = 0;
	obj->attrs->creator_proc_id = multiproc_get_id(NULL);
	obj->attrs->opener_proc_id  = params->opener_proc_id;
	obj->attrs->status          = GATEPETERSON_CREATED;
	obj->attrs->version         = GATEPETERSON_VERSION;
	obj->top                    = handle;

	/* Create the local lock if not provided */
	if (likely(params->local_protection == GATEPETERSON_PROTECT_DEFAULT))
		obj->params.local_protection =
				gatepeterson_state.cfg.default_protection;
	else
		obj->params.local_protection = params->local_protection;

	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:      /* Fall through */
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		obj->local_gate = kmalloc(sizeof(struct mutex),	GFP_KERNEL);
		if (obj->local_gate == NULL) {
			retval = -ENOMEM;
			goto gate_create_fail;
		}

		mutex_init(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied, FIXME */
		break;
	}

	/* Populate the params member */
	memcpy(&obj->params, params, sizeof(struct gatepeterson_params));
	/* Put in the local list */
	retval = mutex_lock_interruptible(gatepeterson_state.list_lock);
	if (retval)
		goto list_lock_fail;

	list_add_tail(&obj->elem, &gatepeterson_state.obj_list);
	mutex_unlock(gatepeterson_state.list_lock);
	return (void *)handle;

list_lock_fail:
	kfree(obj->local_gate);

gate_create_fail:
	status = nameserver_remove(gatepeterson_state.nshandle, params->name);

ns_add32_fail:
	kfree(obj->params.name);

name_alloc_fail:
	kfree(obj);

obj_alloc_fail:
	kfree(handle);

handle_alloc_fail: /* Fall through */
exit:
	printk(KERN_ERR "gatepeterson_create failed status: %x\n", retval);
	return NULL;
}
EXPORT_SYMBOL(gatepeterson_create);

/*
 * ======== gatepeterson_delete ========
 *  Purpose:
 *  This will deletes an instance of GatePeterson module
 */
int gatepeterson_delete(void **gphandle)

{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	struct gatepeterson_params *params = NULL;
	s32 retval;

	BUG_ON(gphandle == NULL);
	BUG_ON(*gphandle == NULL);
	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -ENODEV;
		goto exit;
	}

	handle = (struct gatepeterson_object *)(*gphandle);
	obj = (struct gatepeterson_obj *)handle->obj;
	/* Check if we have created the GP or not */
	if (obj->attrs->creator_proc_id != multiproc_get_id(NULL)) {
		retval = -EACCES;
		goto exit;
	}

	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	if (obj->ref_count != 0) {
		obj->ref_count--;
		retval = -EBUSY;
		goto error_handle;
	}

	retval = mutex_lock_interruptible(gatepeterson_state.list_lock);
	if (retval)
		goto error_handle;

	list_del(&obj->elem); /* Remove the GP instance from the GP list */
	mutex_unlock(gatepeterson_state.list_lock);
	params = &obj->params;
	/* Remove from the name server */
	if (likely(gatepeterson_state.cfg.use_nameserver) &&
						params->name != NULL) {
		retval = nameserver_remove(gatepeterson_state.nshandle,
								params->name);
		if (unlikely(retval != 0))
			goto error_handle;
		kfree(params->name);
	}

	mutex_unlock(obj->local_gate);
	/* If the lock handle was created internally */
	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:      /* Fall through */
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		kfree(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied, FIXME */
		break;
	}

	kfree(obj);
	kfree(handle);
	*gphandle = NULL;
	return 0;

error_handle:
	mutex_unlock(obj->local_gate);

exit:
	printk(KERN_ERR "gatepeterson_create failed status: %x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_delete);

/*
 * ======== gatepeterson_inc_refcount ========
 *  Purpose:
 *  This will increment the reference count while opening
 *  a GP instance if it is already opened from local processor
 */
static bool gatepeterson_inc_refcount(const struct gatepeterson_params *params,
					void **handle)
{
	struct gatepeterson_obj *obj = NULL;
	s32 retval  = 0;
	bool done = false;

	list_for_each_entry(obj, &gatepeterson_state.obj_list, elem) {
		if (params->shared_addr != NULL) {
			if (obj->params.shared_addr == params->shared_addr) {
				retval = mutex_lock_interruptible(
						gatepeterson_state.list_lock);
				if (retval)
					break;

				obj->ref_count++;
				*handle = obj->top;
				mutex_unlock(gatepeterson_state.list_lock);
				done = true;
				break;
			}
		} else if (params->name != NULL) {
			if (strcmp(obj->params.name, params->name) == 0) {
				retval = mutex_lock_interruptible(
						gatepeterson_state.list_lock);
				if (retval)
					break;

				obj->ref_count++;
				*handle = obj->top;
				mutex_unlock(gatepeterson_state.list_lock);
				done = true;
				break;
			}
		}
	}

	return done;
}

/*
 * ======== gatepeterson_open ========
 *  Purpose:
 *  This will opens a created instance of GatePeterson
 *  module.
 */
int gatepeterson_open(void **gphandle,
			const struct gatepeterson_params *params)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	void *temp = NULL;
	s32 retval = 0;
	s32 status;
	u32 sharedaddr;
	u32 len;
	void *entry = NULL;

	BUG_ON(params == NULL);
	BUG_ON(gphandle == NULL);
	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -EINVAL;
		goto exit;
	}

	if (gatepeterson_state.cfg.use_nameserver == false &&
			params->shared_addr == NULL) {
		retval = -EINVAL;
		goto exit;
	}

	if (gatepeterson_state.cfg.use_nameserver == true &&
		params->shared_addr == NULL && params->name == NULL) {
		retval = -EINVAL;
		goto exit;
	}
	if (params->shared_addr != NULL && params->shared_addr_size <
			gatepeterson_shared_memreq(params)) {
		retval = -EINVAL;
		goto exit;
	}

	if (gatepeterson_inc_refcount(params, &temp))
		goto exit; /* It's already opened from local processor */

	if (unlikely(params->shared_addr == NULL)) {
		if (likely(gatepeterson_state.cfg.use_nameserver == true &&
						params->name != NULL)) {
			/* Find in name server */
			retval = nameserver_get(gatepeterson_state.nshandle,
						params->name, &sharedaddr,
						sizeof(u32), NULL);
			if (retval != 0)
				goto noentry_fail; /* Entry not found */
		}
	} else
		sharedaddr = (u32)params->shared_addr;

	if (unlikely(((struct gatepeterson_attrs *)sharedaddr)->status !=
						GATEPETERSON_CREATED)) {
		retval = -ENXIO; /* Not created */
		goto noentry_fail;
	}

	handle = kmalloc(sizeof(struct gatepeterson_object), GFP_KERNEL);
	if (handle == NULL) {
		retval = -ENOMEM;
		goto handle_alloc_fail;
	}

	obj = kmalloc(sizeof(struct gatepeterson_obj), GFP_KERNEL);
	if (obj == NULL) {
		retval = -ENOMEM; /* Not created */
		goto obj_alloc_fail;
	}

	if (likely(gatepeterson_state.cfg.use_nameserver == true &&
						params->name != NULL)) {
		len = strlen(params->name) + 1;
		obj->params.name = kmalloc(len, GFP_KERNEL);
		if (obj->params.name == NULL) {
			retval = -ENOMEM;
			goto name_alloc_fail;
		}

		strncpy(obj->params.name, params->name, len);
		entry = nameserver_add_uint32(gatepeterson_state.nshandle,
						params->name, sharedaddr);
		if (entry == NULL)
			goto ns_add32_fail;
	}

	handle->obj = obj;
	handle->enter = gatepeterson_enter;
	handle->leave = gatepeterson_leave;
	handle->lock_get_knl_handle = gatepeterson_get_knl_handle;
	/* assign the memory with proper cache line padding */
	obj->attrs   = (struct gatepeterson_attrs *)sharedaddr;
	obj->flag[0] = ((void *)(((u32)obj->attrs) + 128));
	obj->flag[1] = ((void *)(((u32) obj->flag[0]) + 128));
	obj->turn    = ((void *)(((u32) obj->flag[1]) + 128)); /* TBD: Fixme */
	/* Creator always has selfid set to 0 */
	obj->self_id              = 1;
	obj->other_id             = 0;
	obj->nested               = 0;
	obj->ref_count            = 0;
	obj->attrs->opener_proc_id = multiproc_get_id(NULL);
	obj->top                   = handle;

	/* Create the local lock if not provided */
	if (likely(params->local_protection ==
						GATEPETERSON_PROTECT_DEFAULT))
		obj->params.local_protection =
				gatepeterson_state.cfg.default_protection;
	else
		obj->params.local_protection = params->local_protection;

	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:      /* Fall through */
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		obj->local_gate = kmalloc(sizeof(struct mutex),	GFP_KERNEL);
		if (obj->local_gate == NULL) {
			retval = -ENOMEM;
			goto gate_create_fail;
		}

		mutex_init(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied, FIXME */
		break;
	}

	/* Populate the params member */
	memcpy(&obj->params, params, sizeof(struct gatepeterson_params));
	/* Put in the local list */
	retval = mutex_lock_interruptible(gatepeterson_state.list_lock);
	if (retval)
		goto list_lock_fail;

	list_add_tail(&obj->elem, &gatepeterson_state.obj_list);
	mutex_unlock(gatepeterson_state.list_lock);
	*gphandle = handle;
	return 0;

list_lock_fail:
	kfree(obj->local_gate);

gate_create_fail:
	status = nameserver_remove(gatepeterson_state.nshandle, params->name);

ns_add32_fail:
	kfree(obj->params.name);

name_alloc_fail:
	kfree(obj);

obj_alloc_fail:
	kfree(handle);

handle_alloc_fail: /* Fall through */
noentry_fail: /* Fall through */
exit:
	printk(KERN_ERR "gatepeterson_open failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_open);

/*
 * ======== gatepeterson_close ========
 *  Purpose:
 *  This will closes previously opened/created instance
 * of GatePeterson module
 */
int gatepeterson_close(void **gphandle)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	struct gatepeterson_params *params = NULL;
	s32 retval = 0;

	BUG_ON(gphandle == NULL);
	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -EINVAL;
		goto exit;
	}

	if (WARN_ON(*gphandle == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct gatepeterson_object *)(*gphandle);
	obj = (struct gatepeterson_obj *) handle->obj;
	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	if (obj->ref_count > 1) {
		obj->ref_count--;
		mutex_unlock(obj->local_gate);
		goto exit;
	}

	retval = mutex_lock_interruptible(gatepeterson_state.list_lock);
	if (retval)
		goto error_handle;

	list_del(&obj->elem);
	mutex_unlock(gatepeterson_state.list_lock);
	params = &obj->params;
	/* remove from the name server */
	if (likely(gatepeterson_state.cfg.use_nameserver == true &&
						params->name != NULL)) {
		retval = nameserver_remove(gatepeterson_state.nshandle,
							params->name);
		if (unlikely(retval != 0))
			goto error_handle;

		kfree(params->name);
	}

	mutex_unlock(obj->local_gate);
	/* If the lock handle was created internally */
	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:      /* Fall through */
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		kfree(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied, FIXME */
		break;
	}

	kfree(obj);
	kfree(handle);
	*gphandle = NULL;
	return 0;

error_handle:
	mutex_unlock(obj->local_gate);

exit:
	printk(KERN_ERR "gatepeterson_close failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_close);

/*
 * ======== gatepeterson_enter ========
 *  Purpose:
 *  This will enters the GatePeterson instance
 */
u32 gatepeterson_enter(void *gphandle)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	s32 retval = 0;

	BUG_ON(gphandle == NULL);
	if (WARN_ON(gatepeterson_state.is_init != true)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct gatepeterson_object *)gphandle;
	obj = (struct gatepeterson_obj *) handle->obj;
	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	obj->nested++;
	if (obj->nested == 1) {
		/* indicate, needs to use the resource. */
		*((u32 *)obj->flag[obj->self_id]) = GATEPETERSON_BUSY ;
		/* Give away the turn. */
		*((u32 *)(obj->turn)) = obj->other_id;
		/* Wait while other processor is using the resource and has
		 *  the turn
		 */
		while ((*((u32 *) obj->flag[obj->other_id])
			== GATEPETERSON_BUSY) &&
			(*((u32  *)obj->turn) == obj->other_id))
			; /* Empty body loop */
	}

	return 0;

exit:
	return retval;
}
EXPORT_SYMBOL(gatepeterson_enter);

/*
 * ======== gatepeterson_leave ========
 *  Purpose:
 *  This will leaves the GatePeterson instance
 */
void gatepeterson_leave(void *gphandle, u32 flag)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj    = NULL;

	BUG_ON(gatepeterson_state.is_init != true);
	BUG_ON(gphandle == NULL);

	handle = (struct gatepeterson_object *)gphandle;
	(void) flag;
	obj = (struct gatepeterson_obj *)handle->obj;
	obj->nested--;
	if (obj->nested == 0)
		*((u32 *) obj->flag[obj->self_id]) = GATEPETERSON_FREE ;

	mutex_unlock(obj->local_gate);
	return;
}
EXPORT_SYMBOL(gatepeterson_leave);

/*
 * ======== gatepeterson_get_knl_handle ========
 *  Purpose:
 *  This will gatepeterson kernel object pointer
 */
void *gatepeterson_get_knl_handle(void **gphandle)
{
	BUG_ON(gphandle == NULL);
	return gphandle;
}
EXPORT_SYMBOL(gatepeterson_get_knl_handle);

/*
 * ======== gatepeterson_shared_memreq ========
 *  Purpose:
 *  This will give the amount of shared memory required
 *  for creation of each instance
 */
u32 gatepeterson_shared_memreq(const struct gatepeterson_params *params)
{
	u32 retval = 0;

	if (params != NULL)
		retval = 128 * 4;
	else
		retval = 4 * sizeof(u32);

	return retval;
}
EXPORT_SYMBOL(gatepeterson_shared_memreq);

