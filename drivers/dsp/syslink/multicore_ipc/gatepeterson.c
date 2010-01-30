/*
 *  gatepeterson.c
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

#include <syslink/atomic_linux.h>
#include <multiproc.h>
#include <nameserver.h>
#include <sharedregion.h>
#include <gatepeterson.h>


/* IPC stubs */

/*
 *  Name of the reserved NameServer used for gatepeterson
 */
#define GATEPETERSON_NAMESERVER  	"GatePeterson"
#define GATEPETERSON_BUSY	   	 1
#define GATEPETERSON_FREE	   	 0
#define GATEPETERSON_VERSION		 1
#define GATEPETERSON_CREATED	 	 0x08201997 /* Stamp to indicate GP
							was created here */
#define MAX_GATEPETERSON_NAME_LEN 	 32

/* Cache line size */
#define GATEPETERSON_CACHESIZE   	128

/* Macro to make a correct module magic number with ref_count */
#define GATEPETERSON_MAKE_MAGICSTAMP(x) ((GATEPETERSON_MODULEID << 12) | (x))

/*
 *  structure for gatepeterson module state
 */
struct gatepeterson_moduleobject {
	atomic_t ref_count; /* Reference count */
	void *nshandle;
	struct list_head obj_list;
	struct mutex *mod_lock; /* Lock for obj list */
	struct gatepeterson_config cfg;
	struct gatepeterson_config default_cfg;
	struct gatepeterson_params def_inst_params; /* default instance
							paramters */
};

/*
 *  Structure defining attribute parameters for the Gate Peterson module
 */
struct gatepeterson_attrs {
	VOLATILE u32 version;
	VOLATILE u32 status;
	VOLATILE u16 creator_proc_id;
	VOLATILE  u16 opener_proc_id;
};

/*
 *  Structure defining internal object for the Gate Peterson
 */
struct gatepeterson_obj {
	struct list_head elem;
	VOLATILE struct gatepeterson_attrs *attrs; /* Instance attr */
	VOLATILE u32 *flag[2]; /* Falgs for processors */
	VOLATILE u32 *turn; /* Indicates whoes turn it is now? */
	u8 self_id; /* Self identifier */
	u8 other_id; /* Other's identifier */
	u32 nested; /* Counter to track nesting */
	void *local_gate; /* Local lock handle */
	void *ns_key; /* NameServer key received in create */
	enum gatepeterson_protect local_protection; /* Type of local protection
								to be used */
	struct gatepeterson_params params;
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
 *  Variable for holding state of the gatepeterson module
 */
struct gatepeterson_moduleobject gatepeterson_state = {
	.obj_list	 = LIST_HEAD_INIT(gatepeterson_state.obj_list),
	.default_cfg.max_name_len		= MAX_GATEPETERSON_NAME_LEN,
	.default_cfg.default_protection  = GATEPETERSON_PROTECT_PROCESS,
	.default_cfg.use_nameserver	  = true,
	.def_inst_params.shared_addr	 = 0x0,
	.def_inst_params.shared_addr_size = 0x0,
	.def_inst_params.name 			 = NULL,
	.def_inst_params.local_protection = GATEPETERSON_PROTECT_DEFAULT

};

static void *_gatepeterson_create(const struct gatepeterson_params *params,
							bool create_flag);

/*
 * ======== gatepeterson_get_config ========
 *  Purpose:
 *  This will get the default configuration parameters for gatepeterson
 *  module
 */
void gatepeterson_get_config(struct gatepeterson_config *config)
{
	if (WARN_ON(config == NULL))
		goto exit;

	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true)
		memcpy(config, &gatepeterson_state.default_cfg,
					sizeof(struct gatepeterson_config));
	else
		memcpy(config, &gatepeterson_state.cfg,
					sizeof(struct gatepeterson_config));

exit:
	return;
}
EXPORT_SYMBOL(gatepeterson_get_config);

/*
 * ======== gatepeterson_setup ========
 *  Purpose:
 *  This will setup the gatepeterson module
 */
int gatepeterson_setup(const struct gatepeterson_config *config)
{
	struct nameserver_params params;
	struct gatepeterson_config tmp_cfg;
	void *nshandle = NULL;
	s32 retval = 0;
	s32 ret;

	/* This sets the ref_count variable  not initialized, upper 16 bits is
	* written with module Id to ensure correctness of ref_count variable
	*/
	atomic_cmpmask_and_set(&gatepeterson_state.ref_count,
					GATEPETERSON_MAKE_MAGICSTAMP(0),
					GATEPETERSON_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&gatepeterson_state.ref_count)
					!= GATEPETERSON_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (config == NULL) {
		gatepeterson_get_config(&tmp_cfg);
		config = &tmp_cfg;
	}

	if (WARN_ON(config->max_name_len == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	if (likely((config->use_nameserver == true))) {
		retval = nameserver_params_init(&params);
		params.max_value_len = sizeof(u32);
		params.max_name_len = config->max_name_len;
		/* Create the nameserver for modules */
		nshandle = nameserver_create(GATEPETERSON_NAMESERVER, &params);
		if (nshandle == NULL)
			goto exit;

		gatepeterson_state.nshandle = nshandle;
	}

	memcpy(&gatepeterson_state.cfg, config,
					sizeof(struct gatepeterson_config));
	gatepeterson_state.mod_lock = kmalloc(sizeof(struct mutex),
								GFP_KERNEL);
	if (gatepeterson_state.mod_lock == NULL) {
		retval = -ENOMEM;
		goto lock_create_fail;
	}

	mutex_init(gatepeterson_state.mod_lock);
	return 0;

lock_create_fail:
	if ((likely(config->use_nameserver == true)))
		ret = nameserver_delete(&gatepeterson_state.nshandle);

exit:
	atomic_set(&gatepeterson_state.ref_count,
				GATEPETERSON_MAKE_MAGICSTAMP(0));

	printk(KERN_ERR	"gatepeterson_setup failed status: %x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_setup);

/*
 * ======== gatepeterson_destroy ========
 *  Purpose:
 *  This will destroy the gatepeterson module
 */
int gatepeterson_destroy(void)

{
	struct gatepeterson_obj *obj = NULL;
	struct mutex *lock = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&gatepeterson_state.ref_count)
			== GATEPETERSON_MAKE_MAGICSTAMP(0))) {
		retval = 1;
		goto exit;
	}

	/* Temporarily increment ref_count here. */
	atomic_set(&gatepeterson_state.ref_count,
				GATEPETERSON_MAKE_MAGICSTAMP(1));
	/* Check if any gatepeterson instances have not been
	*  ideleted/closed so far, if there any, delete or close them
	*/
	list_for_each_entry(obj, &gatepeterson_state.obj_list, elem) {
		if (obj->attrs->creator_proc_id ==
					multiproc_get_id(NULL))
			gatepeterson_delete(&obj->top);
		else
			gatepeterson_close(&obj->top);

		if (list_empty(&gatepeterson_state.obj_list))
			break;
	}

	/* Again reset ref_count. */
	atomic_set(&gatepeterson_state.ref_count,
				GATEPETERSON_MAKE_MAGICSTAMP(0));

	retval = mutex_lock_interruptible(gatepeterson_state.mod_lock);
	if (retval != 0)
		goto exit;

	if (likely(gatepeterson_state.cfg.use_nameserver == true)) {
		retval = nameserver_delete(&gatepeterson_state.nshandle);
		if (unlikely(retval != 0))
			goto exit;
	}

	lock = gatepeterson_state.mod_lock;
	gatepeterson_state.mod_lock = NULL;
	memset(&gatepeterson_state.cfg, 0, sizeof(struct gatepeterson_config));
	mutex_unlock(lock);
	kfree(lock);
	/* Decrease the ref_count */
	atomic_set(&gatepeterson_state.ref_count,
			GATEPETERSON_MAKE_MAGICSTAMP(0));
	return 0;

exit:;
	if (retval < 0) {
		printk(KERN_ERR "gatepeterson_destroy failed status:%x\n",
			retval);
	}
	return retval;

}
EXPORT_SYMBOL(gatepeterson_destroy);

/*
 * ======== gatepeterson_params_init ========
 *  Purpose:
 *  This will  Initialize this config-params structure with
 *  supplier-specified defaults before instance creation
 */
void gatepeterson_params_init(void *handle,
				struct gatepeterson_params *params)
{
	if (WARN_ON(atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(params == NULL))
		goto exit;

	if (handle == NULL)
		memcpy(params, &(gatepeterson_state.def_inst_params),
				sizeof(struct gatepeterson_params));
	else {
		struct gatepeterson_obj *obj =
					(struct gatepeterson_obj *)handle;
		/* Return updated gatepeterson instance specific parameters. */
		memcpy(params, &(obj->params),
				sizeof(struct gatepeterson_params));
	}

exit:
	return;
}
EXPORT_SYMBOL(gatepeterson_params_init);

/*
 * ======== gatepeterson_create ========
 *  Purpose:
 *  This will creates a new instance of gatepeterson module
 */
void *gatepeterson_create(const struct gatepeterson_params *params)
{
	void *handle = NULL;
	s32 retval = 0;
	u32 shaddrsize;

	BUG_ON(params == NULL);
	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
					GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	shaddrsize = gatepeterson_shared_memreq(params);
	if (WARN_ON(params->shared_addr == NULL ||
		params->shared_addr_size < shaddrsize)) {
		retval = -EINVAL;
		goto exit;
	}

	if (params->local_protection >= GATEPETERSON_PROTECT_END_VALUE) {
		retval = -EINVAL;
		goto exit;
	}

	handle = _gatepeterson_create(params, true);
	return handle;

exit:
	return NULL;
}
EXPORT_SYMBOL(gatepeterson_create);

/*
 * ======== gatepeterson_delete ========
 *  Purpose:
 *  This will deletes an instance of gatepeterson module
 */
int gatepeterson_delete(void **gphandle)

{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	struct gatepeterson_params *params = NULL;
	s32 retval;

	BUG_ON(gphandle == NULL);
	BUG_ON(*gphandle == NULL);
	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	handle = (struct gatepeterson_object *)(*gphandle);
	obj = (struct gatepeterson_obj *)handle->obj;
	if (unlikely(obj == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if (unlikely(obj->attrs == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	/* Check if we have created the GP or not */
	if (unlikely(obj->attrs->creator_proc_id != multiproc_get_id(NULL))) {
		retval = -EACCES;
		goto exit;
	}

	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	if (obj->ref_count != 0) {
		retval = -EBUSY;
		goto error_handle;
	}

	obj->attrs->status = !GATEPETERSON_CREATED;
	retval = mutex_lock_interruptible(gatepeterson_state.mod_lock);
	if (retval)
		goto exit;

	list_del(&obj->elem); /* Remove the GP instance from the GP list */
	mutex_unlock(gatepeterson_state.mod_lock);
	params = &obj->params;
	/* Remove from the name server */
	if (likely(gatepeterson_state.cfg.use_nameserver) &&
						params->name != NULL) {
		retval = nameserver_remove_entry(gatepeterson_state.nshandle,
								obj->ns_key);
		if (unlikely(retval != 0))
			goto error_handle;
		kfree(params->name);
		obj->ns_key = NULL;
	}

	mutex_unlock(obj->local_gate);
	/* If the lock handle was created internally */
	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:	  /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		/* FIXME: Add a spinlock protection */
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
						gatepeterson_state.mod_lock);
				if (retval)
					break;

				obj->ref_count++;
				*handle = obj->top;
				mutex_unlock(gatepeterson_state.mod_lock);
				done = true;
				break;
			}
		} else if (params->name != NULL && obj->params.name != NULL) {
			if (strcmp(obj->params.name, params->name) == 0) {
				retval = mutex_lock_interruptible(
						gatepeterson_state.mod_lock);
				if (retval)
					break;

				obj->ref_count++;
				*handle = obj->top;
				mutex_unlock(gatepeterson_state.mod_lock);
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
 *  This will opens a created instance of gatepeterson
 *  module.
 */
int gatepeterson_open(void **gphandle,
			struct gatepeterson_params *params)
{
	void *temp = NULL;
	s32 retval = 0;
	u32 sharedaddr;

	BUG_ON(params == NULL);
	BUG_ON(gphandle == NULL);
	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
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

	if (gatepeterson_inc_refcount(params, &temp)) {
		retval = -EBUSY;
		goto exit; /* It's already opened from local processor */
	}

	if (unlikely(params->shared_addr == NULL)) {
		if (likely(gatepeterson_state.cfg.use_nameserver == true &&
						params->name != NULL)) {
			/* Find in name server */
			retval = nameserver_get(gatepeterson_state.nshandle,
						params->name, &sharedaddr,
						sizeof(u32), NULL);
			if (retval < 0)
				goto noentry_fail; /* Entry not found */

			params->shared_addr = sharedregion_get_ptr(
							(u32 *)sharedaddr);
			if (params->shared_addr == NULL)
				goto noentry_fail;
		}
	} else
		sharedaddr = (u32) params->shared_addr;

	if (unlikely(((struct gatepeterson_attrs *)sharedaddr)->status !=
						GATEPETERSON_CREATED)) {
		retval = -ENXIO; /* Not created */
		goto exit;
	}

	if (unlikely(((struct gatepeterson_attrs *)sharedaddr)->version !=
						GATEPETERSON_VERSION)) {
		retval = -ENXIO; /* FIXME Version mismatch,
					need to change retval */
		goto exit;
	}

	*gphandle = _gatepeterson_create(params, false);
	return 0;

noentry_fail: /* Fall through */
	retval = -ENOENT;
exit:
	printk(KERN_ERR "gatepeterson_open failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(gatepeterson_open);

/*
 * ======== gatepeterson_close ========
 *  Purpose:
 *  This will closes previously opened/created instance
 * of gatepeterson module
 */
int gatepeterson_close(void **gphandle)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	struct gatepeterson_params *params = NULL;
	s32 retval = 0;

	BUG_ON(gphandle == NULL);
	if (atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(*gphandle == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct gatepeterson_object *)(*gphandle);
	obj = (struct gatepeterson_obj *) handle->obj;
	if (unlikely(obj == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	if (obj->ref_count > 1) {
		obj->ref_count--;
		mutex_unlock(obj->local_gate);
		goto exit;
	}

	retval = mutex_lock_interruptible(gatepeterson_state.mod_lock);
	if (retval)
		goto error_handle;

	list_del(&obj->elem);
	mutex_unlock(gatepeterson_state.mod_lock);
	params = &obj->params;
	if (likely(params->name != NULL))
		kfree(params->name);

	mutex_unlock(obj->local_gate);
	/* If the lock handle was created internally */
	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:	  /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		/* FIXME: Add a spinlock protection */
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		kfree(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied */
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
 *  This will enters the gatepeterson instance
 */
u32 gatepeterson_enter(void *gphandle)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	s32 retval = 0;

	BUG_ON(gphandle == NULL);
	if (WARN_ON(atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}


	handle = (struct gatepeterson_object *)gphandle;
	obj = (struct gatepeterson_obj *) handle->obj;
	if (obj->local_gate != NULL)
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
		while ((*((VOLATILE u32 *) obj->flag[obj->other_id])
			== GATEPETERSON_BUSY) &&
			(*((VOLATILE u32  *)obj->turn) == obj->other_id))
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
 *  This will leaves the gatepeterson instance
 */
void gatepeterson_leave(void *gphandle, u32 flag)
{
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj	= NULL;

	if (WARN_ON(atomic_cmpmask_and_lt(&(gatepeterson_state.ref_count),
				GATEPETERSON_MAKE_MAGICSTAMP(0),
				GATEPETERSON_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	BUG_ON(gphandle == NULL);

	handle = (struct gatepeterson_object *)gphandle;
	(void) flag;
	obj = (struct gatepeterson_obj *)handle->obj;
	obj->nested--;
	if (obj->nested == 0)
		*((VOLATILE u32 *)obj->flag[obj->self_id]) = GATEPETERSON_FREE;

	if (obj->local_gate != NULL)
		mutex_unlock(obj->local_gate);

exit:
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

    retval = (GATEPETERSON_CACHESIZE * 4) ;
	return retval;
}
EXPORT_SYMBOL(gatepeterson_shared_memreq);

/*
 * ======== gatepeterson_create ========
 *  Purpose:
 *  Creates a new instance of gatepeterson module.
 *  This is an internal function because both
 *  gatepeterson_create and gatepeterson_open
 *  call use the same functionality.
 */
static void *_gatepeterson_create(const struct gatepeterson_params *params,
							bool create_flag)
{
	int status = 0;
	struct gatepeterson_object *handle = NULL;
	struct gatepeterson_obj *obj = NULL;
	u32 len;
	u32	shm_index;
	u32 shared_shm_base;
	s32 retval = 0;


	handle = kmalloc(sizeof(struct gatepeterson_object), GFP_KERNEL);
	if (handle == NULL) {
		retval = -ENOMEM;
		goto exit;
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

		if (create_flag == true) {
			shm_index = sharedregion_get_index(
							params->shared_addr);
			shared_shm_base = (u32)sharedregion_get_srptr(
						(void *)params->shared_addr,
						shm_index);
			obj->ns_key = nameserver_add_uint32(
						gatepeterson_state.nshandle,
						params->name,
						(u32) (shared_shm_base));
			if (obj->ns_key == NULL) {
				status = -ENOMEM; /* FIXME */
				goto ns_add32_fail;
			}
		}

	}

	handle->obj = obj;
	handle->enter = &gatepeterson_enter;
	handle->leave = &gatepeterson_leave;
	handle->lock_get_knl_handle = &gatepeterson_get_knl_handle;
	/* assign the memory with proper cache line padding */
	obj->attrs   = (struct gatepeterson_attrs *) params->shared_addr;
	obj->flag[0] = ((void *)(((u32) obj->attrs) +
						GATEPETERSON_CACHESIZE));
	obj->flag[1] = ((void *)(((u32) obj->flag[0]) +
						GATEPETERSON_CACHESIZE));
	obj->turn	 = ((void *)(((u32) obj->flag[1])
				  + GATEPETERSON_CACHESIZE)); /* TBD: Fixme */

	/* Creator always has selfid set to 0 */
	if (create_flag == true) {
		obj->self_id		    = 0;
		obj->other_id		    = 1;
		obj->attrs->creator_proc_id = multiproc_get_id(NULL);
		obj->attrs->opener_proc_id  = MULTIPROC_INVALIDID;
		obj->attrs->status	    = GATEPETERSON_CREATED;
		obj->attrs->version	    = GATEPETERSON_VERSION;

		/* Set up shared memory */
		*(obj->turn)	   = 0x0;
		*(obj->flag[0])    = 0x0;
		*(obj->flag[1])	   = 0x0;
		obj->ref_count	   = 0;
	} else {
		obj->self_id	   = 1;
		obj->other_id	   = 0;
		obj->attrs->opener_proc_id  = multiproc_get_id(NULL);
		obj->ref_count	   = 1;
	}
	obj->nested		   = 0;
	obj->top		   = handle;

	/* Populate the params member */
	memcpy(&obj->params, params, sizeof(struct gatepeterson_params));

	/* Create the local lock if not provided */
	if (likely(params->local_protection == GATEPETERSON_PROTECT_DEFAULT))
		obj->params.local_protection =
				gatepeterson_state.cfg.default_protection;
	else
		obj->params.local_protection = params->local_protection;

	switch (obj->params.local_protection) {
	case  GATEPETERSON_PROTECT_NONE:	  /* Fall through */
		obj->local_gate = NULL; /* TBD: Fixme */
		break;
	/* In syslink ; for interrupt protect gatespinlock is used, that
	*   internally uses the mutex. So we added mutex for interrupt
	*   protection here also
	*/
	case  GATEPETERSON_PROTECT_INTERRUPT: /* Fall through */
		/* FIXME: Add a spinlock protection */
	case  GATEPETERSON_PROTECT_TASKLET: /* Fall through */
	case  GATEPETERSON_PROTECT_THREAD:  /* Fall through */
	case  GATEPETERSON_PROTECT_PROCESS:
		obj->local_gate = kmalloc(sizeof(struct mutex), GFP_KERNEL);
		if (obj->local_gate == NULL) {
			retval = -ENOMEM;
			goto gate_create_fail;
		}

		mutex_init(obj->local_gate);
		break;
	default:
		/* An invalid protection level was supplied, FIXME */
		obj->local_gate = NULL;
		break;
	}

	/* Put in the local list */
	retval = mutex_lock_interruptible(gatepeterson_state.mod_lock);
	if (retval)
		goto mod_lock_fail;

	list_add_tail(&obj->elem, &gatepeterson_state.obj_list);
	mutex_unlock(gatepeterson_state.mod_lock);
	return (void *)handle;

mod_lock_fail:
	kfree(obj->local_gate);

gate_create_fail:
	status = nameserver_remove_entry(gatepeterson_state.nshandle,
					obj->ns_key);

ns_add32_fail:
	kfree(obj->params.name);

name_alloc_fail:
	kfree(obj);

obj_alloc_fail:
	kfree(handle);
	handle = NULL;

exit:
	if (create_flag == true)
		printk(KERN_ERR "_gatepeterson_create (create) failed status: %x\n",
				retval);
	else
		printk(KERN_ERR "_gatepeterson_create (open) failed status: %x\n",
				retval);

	return NULL;
}
