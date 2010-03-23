/*
 *  heapbuf.c
 *
 *  Heap module manages fixed size buffers that can be used
 *  in a multiprocessor system with shared memory.
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

#include <atomic_linux.h>
#include <multiproc.h>
#include <nameserver.h>
#include <sharedregion.h>
#include <gatepeterson.h>
#include <heapbuf.h>
#include <listmp.h>
#include <listmp_sharedmemory.h>

/*
 *  Name of the reserved nameserver used for heapbuf.
 */
#define HEAPBUF_NAMESERVER		"HeapBuf"
#define HEAPBUF_MAX_NAME_LEN		32
#define HEAPBUF_CACHESIZE		128
/* brief Macro to make a correct module magic number with refCount */
#define HEAPBUF_MAKE_MAGICSTAMP(x)	((HEAPBUF_MODULEID << 12) | (x))


/*
 *  Structure defining attribute parameters for the heapbuf module
 */
struct heapbuf_attrs {
	VOLATILE u32 version;
	VOLATILE u32 status;
	VOLATILE u32 num_free_blocks;
	VOLATILE u32 min_free_blocks;
	VOLATILE u32 block_size;
	VOLATILE u32 align;
	VOLATILE u32 num_blocks;
	VOLATILE u32 buf_size;
	VOLATILE char *buf;
};

/*
 *  Structure defining processor related information for the
 *  heapbuf module
 */
struct heapbuf_proc_attrs {
	bool creator; /* Creator or opener */
	u16 proc_id; /* Processor identifier */
	u32 open_count; /* open count in a processor */
};

/*
 *  Structure for heapbuf module state
 */
struct heapbuf_module_object {
	atomic_t ref_count; /* Reference count */
	void *ns_handle;
	struct list_head obj_list; /* List holding created objects */
	struct mutex *local_lock; /* lock for protecting obj_list */
	struct heapbuf_config cfg;
	struct heapbuf_config default_cfg; /* Default config values */
	struct heapbuf_params default_inst_params; /* Default instance
						creation parameters */
};

struct heapbuf_module_object heapbuf_state = {
	.obj_list = LIST_HEAD_INIT(heapbuf_state.obj_list),
	.default_cfg.max_name_len = HEAPBUF_MAX_NAME_LEN,
	.default_cfg.use_nameserver = true,
	.default_cfg.track_max_allocs = false,
	.default_inst_params.gate = NULL,
	.default_inst_params.exact = false,
	.default_inst_params.name = NULL,
	.default_inst_params.resource_id = 0,
	.default_inst_params.cache_flag = false,
	.default_inst_params.align = 1,
	.default_inst_params.num_blocks = 0,
	.default_inst_params.block_size = 0,
	.default_inst_params.shared_addr = NULL,
	.default_inst_params.shared_addr_size = 0,
	.default_inst_params.shared_buf = NULL,
	.default_inst_params.shared_buf_size = 0
};

/*
 *  Structure for the handle for the heapbuf
 */
struct heapbuf_obj {
	struct list_head list_elem; /* Used for creating a linked list */
	struct heapbuf_params params; /* The creation parameter structure */
	struct heapbuf_attrs *attrs; /* The shared attributes structure */
	void *free_list; /* List of free buffers */
	struct mutex *gate; /* Lock used for critical region management */
	void *ns_key; /* nameserver key required for remove */
	struct heapbuf_proc_attrs owner; /* owner processor info */
	void *top; /* Pointer to the top object */
	bool cacheFlag; /* added for future use */
};

/*
 * ======== heapbuf_get_config ========
 *  Purpose:
 *  This will get default configuration for the
 *  heapbuf module
 */
int heapbuf_get_config(struct heapbuf_config *cfgparams)
{
	BUG_ON(cfgparams == NULL);

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true)
		memcpy(cfgparams, &heapbuf_state.default_cfg,
					sizeof(struct heapbuf_config));
	else
		memcpy(cfgparams, &heapbuf_state.cfg,
					sizeof(struct heapbuf_config));
	return 0;
}
EXPORT_SYMBOL(heapbuf_get_config);

/*
 * ======== heapbuf_setup ========
 *  Purpose:
 *  This will setup the heapbuf module
 *
 *  This function sets up the HeapBuf module. This function
 *  must be called before any other instance-level APIs can be
 *  invoked.
 *  Module-level configuration needs to be provided to this
 *  function. If the user wishes to change some specific config
 *  parameters, then heapbuf_getconfig can be called to get
 *  the configuration filled with the default values. After this,
 *  only the required configuration values can be changed. If the
 *  user does not wish to make any change in the default parameters,
 *  the application can simply call HeapBuf_setup with NULL
 *  parameters. The default parameters would get automatically used.
 */
int heapbuf_setup(const struct heapbuf_config *cfg)
{
	struct nameserver_params params;
	struct heapbuf_config tmp_cfg;
	void *ns_handle = NULL;
	s32 retval  = 0;

	/* This sets the ref_count variable  not initialized, upper 16 bits is
	* written with module Id to ensure correctness of ref_count variable
	*/
	atomic_cmpmask_and_set(&heapbuf_state.ref_count,
					HEAPBUF_MAKE_MAGICSTAMP(0),
					HEAPBUF_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&heapbuf_state.ref_count)
					!= HEAPBUF_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (cfg == NULL) {
		heapbuf_get_config(&tmp_cfg);
		cfg = &tmp_cfg;
	}

	if (cfg->max_name_len == 0 ||
		cfg->max_name_len > HEAPBUF_MAX_NAME_LEN) {
		retval = -EINVAL;
		goto error;
	}

	heapbuf_state.local_lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (heapbuf_state.local_lock == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	if (likely((cfg->use_nameserver == true))) {
		retval = nameserver_get_params(NULL, &params);
		params.max_value_len = sizeof(u32);
		params.max_name_len = cfg->max_name_len;
		ns_handle  = nameserver_create(HEAPBUF_NAMESERVER, &params);
		if (ns_handle == NULL) {
			retval = -EFAULT;
			goto ns_create_fail;
		}
		heapbuf_state.ns_handle = ns_handle;
	}

	memcpy(&heapbuf_state.cfg, cfg, sizeof(struct heapbuf_config));
	mutex_init(heapbuf_state.local_lock);
	return 0;

ns_create_fail:
	kfree(heapbuf_state.local_lock);

error:
	atomic_set(&heapbuf_state.ref_count,
				HEAPBUF_MAKE_MAGICSTAMP(0));

	printk(KERN_ERR "heapbuf_setup failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_setup);

/*
 * ======== heapbuf_destroy ========
 *  Purpose:
 *  This will destroy the heapbuf module
 */
int heapbuf_destroy(void)
{
	s32 retval  = 0;
	struct mutex *lock = NULL;
	struct heapbuf_obj *obj = NULL;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (atomic_dec_return(&heapbuf_state.ref_count)
				== HEAPBUF_MAKE_MAGICSTAMP(0)) {
		/* Temporarily increment ref_count here. */
		atomic_set(&heapbuf_state.ref_count,
					HEAPBUF_MAKE_MAGICSTAMP(1));

		/*  Check if any heapbuf instances have not been deleted/closed
		 *   so far. if there any, delete or close them
		 */
		list_for_each_entry(obj, &heapbuf_state.obj_list, list_elem) {
			if (obj->owner.proc_id == multiproc_get_id(NULL))
				heapbuf_delete(&obj->top);
			else
				heapbuf_close(obj->top);

			if (list_empty(&heapbuf_state.obj_list))
				break;
		}

		/* Again reset ref_count. */
		atomic_set(&heapbuf_state.ref_count,
					HEAPBUF_MAKE_MAGICSTAMP(0));

		if (likely(heapbuf_state.cfg.use_nameserver == true)) {
			retval = nameserver_delete(&heapbuf_state.ns_handle);
			if (unlikely(retval != 0))
				goto error;
		}

		retval = mutex_lock_interruptible(heapbuf_state.local_lock);
		if (retval)
			goto error;

		lock = heapbuf_state.local_lock;
		heapbuf_state.local_lock = NULL;
		mutex_unlock(lock);
		kfree(lock);
		memset(&heapbuf_state.cfg, 0, sizeof(struct heap_config));

		atomic_set(&heapbuf_state.ref_count,
					HEAPBUF_MAKE_MAGICSTAMP(0));
	}

	return 0;

error:
	printk(KERN_ERR "heapbuf_destroy failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_destroy);

/*
 * ======== heapbuf_params_init ========
 *  Purpose:
 *  This will get the intialization prams for a heapbuf
 *  module instance
 */
void heapbuf_params_init(void *handle,
				struct heapbuf_params *params)
{
	struct heapbuf_obj *obj = NULL;
	s32 retval  = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	BUG_ON(params == NULL);

	if (handle == NULL)
		memcpy(params, &heapbuf_state.default_inst_params,
					sizeof(struct heapbuf_params));
	else {
		obj = (struct heapbuf_obj *)handle;
		memcpy(params, (void *)&obj->params,
				sizeof(struct heapbuf_params));
	}
	return;
error:
	printk(KERN_ERR "heapbuf_params_init failed status: %x\n", retval);
}
EXPORT_SYMBOL(heapbuf_params_init);

/*
 * ======== _heapbuf_create ========
 *  Purpose:
 *  This will create a new instance of heapbuf module
 *  This is an internal function as both heapbuf_create
 *  and heapbuf_open use the functionality
 *
 *  NOTE: The lock to protect the shared memory area
 *  used by heapbuf is provided by the consumer of
 *  heapbuf module
 */
int _heapbuf_create(void **handle_ptr, const struct heapbuf_params *params,
				u32 create_flag)
{
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	char *buf = NULL;
	listmp_sharedmemory_params listmp_params;
	s32 retval  = 0;
	u32 i;
	s32 align;
	s32 shm_index;
	u32 shared_shm_base;
	void *entry = NULL;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	BUG_ON(handle_ptr == NULL);

	BUG_ON(params == NULL);

	/* No need for parameter checks, since this is an internal function. */

	/* Initialize return parameter. */
	*handle_ptr = NULL;

	handle = kmalloc(sizeof(struct heap_object), GFP_KERNEL);
	if (handle == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	obj =  kmalloc(sizeof(struct heapbuf_obj), GFP_KERNEL);
	if (obj == NULL) {
		retval = -ENOMEM;
		goto obj_alloc_error;
	}

	handle->obj = (struct heapbuf_obj *)obj;
	handle->alloc = &heapbuf_alloc;
	handle->free  = &heapbuf_free;
	handle->get_stats = &heapbuf_get_stats;
	/* FIXME: handle->is_blocking = &heapbuf_isblocking; */
	/* Create the shared list */
	listmp_sharedmemory_params_init(NULL, &listmp_params);
	listmp_params.shared_addr = (u32 *)((u32) (params->shared_addr)
					+ ((sizeof(struct heapbuf_attrs)
					+ (HEAPBUF_CACHESIZE - 1))
					& ~(HEAPBUF_CACHESIZE - 1)));
	listmp_params.shared_addr_size =
			listmp_sharedmemory_shared_memreq(&listmp_params);
	listmp_params.gate = NULL;
	/* Assign the memory with proper cache line padding */
	obj->attrs = (struct heapbuf_attrs *) params->shared_addr;

	if (create_flag == false)
		listmp_sharedmemory_open(&obj->free_list, &listmp_params);
	else {
		obj->free_list = listmp_sharedmemory_create(&listmp_params);

		if (obj->free_list == NULL) {
			retval = -ENOMEM;
			goto listmp_error;
		}

		obj->attrs->version = HEAPBUF_VERSION;
		obj->attrs->num_free_blocks = params->num_blocks;
		obj->attrs->min_free_blocks = params->num_blocks;
		obj->attrs->block_size = params->block_size;
		obj->attrs->align  = params->align;
		obj->attrs->num_blocks = params->num_blocks;
		obj->attrs->buf_size = params->shared_buf_size;
		buf = params->shared_buf;
		align = obj->attrs->align;
		buf = (char *)(((u32)buf + (align - 1)) & ~(align - 1));
		obj->attrs->buf	= buf;

		/*
		*  Split the buffer into blocks that are length
		*  block_size" and add into the free_list Queue
		*/
		for (i = 0; i < obj->attrs->num_blocks; i++) {
			listmp_put_tail((struct listmp_object *)
						obj->free_list,
						(struct listmp_elem *)buf);
			buf += obj->attrs->block_size;
		}
	}

	obj->gate = params->gate;

	/* Populate the params member */
	memcpy(&obj->params, params, sizeof(struct heapbuf_params));
	if (params->name != NULL) {
		obj->params.name = kmalloc(strlen(params->name) + 1,
								GFP_KERNEL);
		if (obj->params.name == NULL) {
			retval  = -ENOMEM;
			goto name_alloc_error;
		}
		strncpy(obj->params.name, params->name,
					strlen(params->name) + 1);
	}

	if (create_flag == true) {
		obj->owner.creator = true;
		obj->owner.open_count = 1;
		obj->owner.proc_id = multiproc_get_id(NULL);
		obj->top = handle;
		obj->attrs->status = HEAPBUF_CREATED;
	} else {
		obj->owner.creator = false;
		obj->owner.open_count = 0;
		obj->owner.proc_id = MULTIPROC_INVALIDID;
		obj->top = handle;
	}

	retval = mutex_lock_interruptible(heapbuf_state.local_lock);
	if (retval)
		goto lock_error;

	INIT_LIST_HEAD(&obj->list_elem);
	list_add_tail(&obj->list_elem, &heapbuf_state.obj_list);
	mutex_unlock(heapbuf_state.local_lock);

	if ((likely(heapbuf_state.cfg.use_nameserver == true))
			&& (create_flag == true)) {
		/* We will store a shared pointer in the nameserver */
		shm_index = sharedregion_get_index(params->shared_addr);
		shared_shm_base = (u32)sharedregion_get_srptr(
					params->shared_addr, shm_index);
		if (obj->params.name != NULL) {
			entry = nameserver_add_uint32(heapbuf_state.ns_handle,
							params->name,
							(u32)(shared_shm_base));
			if (entry == NULL) {
				retval = -EFAULT;
				goto ns_add_error;
			}
		}
	}

	*handle_ptr = (void *)handle;
	return retval;

ns_add_error:
	retval = mutex_lock_interruptible(heapbuf_state.local_lock);
	list_del(&obj->list_elem);
	mutex_unlock(heapbuf_state.local_lock);

lock_error:
	if (obj->params.name != NULL) {
		if (obj->ns_key != NULL) {
			nameserver_remove_entry(heapbuf_state.ns_handle,
						obj->ns_key);
			obj->ns_key = NULL;
		}
		kfree(obj->params.name);
	}

name_alloc_error: /* Fall through */
	if (create_flag == true)
		listmp_sharedmemory_delete((listmp_sharedmemory_handle *)
					   &obj->free_list);
	else
		listmp_sharedmemory_close((listmp_sharedmemory_handle *)
					   &obj->free_list);

listmp_error:
	kfree(obj);

obj_alloc_error:
	kfree(handle);

error:
	printk(KERN_ERR "_heapbuf_create failed status: %x\n", retval);
	return retval;
}

/*
 * ======== heapbuf_create ========
 *  Purpose:
 *  This will create a new instance of heapbuf module
 */
void *heapbuf_create(const struct heapbuf_params *params)
{
	s32 retval = 0;
	void *handle = NULL;
	u32 buf_size;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	BUG_ON(params == NULL);

	if ((params->shared_addr) == NULL ||
		params->shared_buf == NULL) {
		retval = -EINVAL;
		goto error;
	}

	if ((params->shared_addr_size)
		< heapbuf_shared_memreq(params, &buf_size)) {
		/* if Shared memory size is less than required */
		retval = -EINVAL;
		goto error;
	}

	if (params->shared_buf_size < buf_size) {
		/* if shared memory size is less than required */
		retval = -EINVAL;
		goto error;
	}

	retval = _heapbuf_create((void **)&handle, params, true);
	if (retval < 0)
		goto error;

	return (void *)handle;

error:
	printk(KERN_ERR "heapbuf_create failed status: %x\n", retval);
	return (void *)handle;
}
EXPORT_SYMBOL(heapbuf_create);

/*
 * ======== heapbuf_delete ========
 *  Purpose:
 *  This will delete an instance of heapbuf module
 */
int heapbuf_delete(void **handle_ptr)
{
	int status = 0;
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	struct heapbuf_params *params = NULL;
	s32 retval = 0;
	u16 myproc_id;

	BUG_ON(handle_ptr == NULL);
	handle = (struct heap_object *)(*handle_ptr);
	if (WARN_ON(handle == NULL)) {
		retval = -EINVAL;
		goto error;
	}

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	obj = (struct heapbuf_obj *)handle->obj;
	if (obj == NULL) {
		retval = -EINVAL;
		goto error;
	}

	myproc_id =  multiproc_get_id(NULL);

	if (obj->owner.proc_id != myproc_id) {
		retval  = -EPERM;
		goto error;
	}

	if (likely(obj->gate != NULL)) {
		status = gatepeterson_enter(obj->gate);
		if (status < 0) {
			retval = -EINVAL;
			goto gate_error;
		}
	}

	if (obj->owner.open_count > 1) {
		retval  = -EBUSY;
		goto device_busy_error;;
	}

	if (obj->owner.open_count != 1) {
		retval  = -EBUSY;
		goto device_busy_error;;
	}

	retval = mutex_lock_interruptible(heapbuf_state.local_lock);
	if (retval)
		goto local_lock_error;

	list_del(&obj->list_elem);
	mutex_unlock(heapbuf_state.local_lock);
	params = (struct heapbuf_params *) &obj->params;
	if (likely(params->name != NULL)) {
		if (likely(heapbuf_state.cfg.use_nameserver == true)) {
			retval = nameserver_remove(heapbuf_state.ns_handle,
							params->name);
			if (retval != 0)
				goto ns_remove_error;
			obj->ns_key = NULL;
		}
		kfree(params->name);
	}

	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);
	retval = listmp_sharedmemory_delete(&obj->free_list);
	kfree(obj);
	kfree(handle);
	*handle_ptr = NULL;
	return 0;

ns_remove_error: /* Fall through */
gate_error: /* Fall through */
local_lock_error: /* Fall through */
device_busy_error:
	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);

error:
	printk(KERN_ERR "heapbuf_delete failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_delete);

/*
 * ======== heapbuf_open  ========
 *  Purpose:
 *  This will opens a created instance of heapbuf
 *  module
 */
int heapbuf_open(void **handle_ptr,
				struct heapbuf_params *params)
{
	struct heapbuf_obj *obj = NULL;
	bool found = false;
	s32 retval = 0;
	u16 myproc_id;
	u32 shared_shm_base;
	struct heapbuf_attrs *attrs;

	BUG_ON(handle_ptr == NULL);
	BUG_ON(params == NULL);

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if ((heapbuf_state.cfg.use_nameserver == false)
			&& (params->shared_addr == (u32)NULL)) {
		retval = -EINVAL;
		goto error;
	}

	if ((heapbuf_state.cfg.use_nameserver == true)
			&& (params->shared_addr == (u32)NULL)
			&& (params->name == NULL)) {
		retval = -EINVAL;
		goto error;
	}

	myproc_id = multiproc_get_id(NULL);
	list_for_each_entry(obj, &heapbuf_state.obj_list, list_elem) {
		if (obj->params.shared_addr == params->shared_addr)
			found = true;
		else if (params->name != NULL) {
			if (strcmp(obj->params.name, params->name) == 0)
				found = true;
		}

		if (found == true) {
			retval = mutex_lock_interruptible(
						heapbuf_state.local_lock);
			if (retval)
				goto error;
			if (obj->owner.proc_id == myproc_id)
				obj->owner.open_count++;
			*handle_ptr = obj->top;
			mutex_unlock(heapbuf_state.local_lock);
		}
	}

	if (likely(found == false)) {
		if (unlikely(params->shared_addr == NULL)) {
			if (likely(heapbuf_state.cfg.use_nameserver == true)) {
				/* Find in name server */
				retval = nameserver_get(heapbuf_state.ns_handle,
							params->name,
							&shared_shm_base,
							sizeof(u32),
							NULL);
				if (retval < 0)
					goto error;

				/*
				 * Convert from shared region pointer
				 * to local address
				 */
				params->shared_addr = sharedregion_get_ptr
							(&shared_shm_base);
				if (params->shared_addr == NULL) {
					retval = -EINVAL;
					goto error;
				}
			}
		}

		attrs = (struct heapbuf_attrs *)(params->shared_addr);
		if (unlikely(attrs->status != (HEAPBUF_CREATED)))
			retval = -ENXIO; /* Not created */
		else if (unlikely(attrs->version != (HEAPBUF_VERSION))) {
			retval = -EINVAL;
			goto error;
		}

		retval = _heapbuf_create((void **)handle_ptr, params, false);
		if (retval < 0)
			goto error;

	}

	return 0;

error:
	printk(KERN_ERR "heapbuf_open failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_open);

/*
 * ======== heapbuf_close  ========
 *  Purpose:
 *  This will closes previously opened/created instance
 *  of heapbuf module
 */
int heapbuf_close(void *handle_ptr)
{
	int status = 0;
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	struct heapbuf_params *params = NULL;
	s32 retval = 0;
	u16 myproc_id = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (WARN_ON(handle_ptr == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	handle = (struct heap_object *)(handle_ptr);
	obj = (struct heapbuf_obj *)handle->obj;

	if (obj != NULL) {
		retval = mutex_lock_interruptible(heapbuf_state.local_lock);
		if (retval)
			goto error;

		myproc_id = multiproc_get_id(NULL);
		/* opening an instance created locally */
		if (obj->owner.proc_id == myproc_id) {
			if (obj->owner.open_count > 1)
				obj->owner.open_count--;
		}

		/* Check if HeapBuf is opened on same processor*/
		if ((((struct heapbuf_obj *)obj)->owner.creator == false)
				&&  (obj->owner.open_count == 0)) {
			list_del(&obj->list_elem);

			/* Take the local lock */
			if (likely(obj->gate != NULL)) {
				status = gatepeterson_enter(obj->gate);
				if (status < 0) {
					retval = -EINVAL;
					goto error;
				}
			}

			params = (struct heapbuf_params *)&obj->params;
			if (likely((params->name) != NULL))
				kfree(params->name); /* Free memory */

			/* Release the local lock */
			if (likely(obj->gate != NULL))
				gatepeterson_leave(obj->gate, 0);

			/* Delete the list */
			listmp_sharedmemory_close((listmp_sharedmemory_handle *)
				obj->free_list);
			kfree(obj);
			kfree(handle);
			handle = NULL;
		}
		mutex_unlock(heapbuf_state.local_lock);
	}
	return 0;

error:
	printk(KERN_ERR "heapbuf_close failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_close);

/*
 * ======== heapbuf_alloc  ========
 *  Purpose:
 *  This will allocs a block of memory
 */
void *heapbuf_alloc(void *hphandle, u32 size, u32 align)
{
	int status = 0;
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	char *block = NULL;
	s32 retval  = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(size == 0)) {
		retval  = -EINVAL;
		goto error;
	}

	handle = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)handle->obj;

	if ((obj->params.exact == true)
			&& (size != obj->attrs->block_size)) {
		retval  = -EINVAL;
		goto error;
	}

	if (size > obj->attrs->block_size) {
		retval  = -EINVAL;
		goto error;
	}

	if (likely(obj->gate != NULL)) {
		status = gatepeterson_enter(obj->gate);
		if (status < 0) {
			retval = -EINVAL;
			goto error;
		}
	}

	block = listmp_get_head((void *)obj->free_list);
	if (block == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	obj->attrs->num_free_blocks--;
	/*
	*  Keep track of the min number of free for this heapbuf, if user
	*  has set the config variable trackMaxAllocs to true.
	*
	*  The min number of free blocks, 'min_free_blocks', will be used to
	*  compute the "all time" maximum number of allocated blocks in
	*  getExtendedStats().
	*/
	if (heapbuf_state.cfg.track_max_allocs) {
		if (obj->attrs->num_free_blocks < obj->attrs->min_free_blocks)
			/* save the new minimum */
			obj->attrs->min_free_blocks =
						obj->attrs->num_free_blocks;
	}

	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);
	return block;
error:
	printk(KERN_ERR "heapbuf_alloc failed status: %x\n", retval);
	return NULL;
}
EXPORT_SYMBOL(heapbuf_alloc);

/*
 * ======== heapbuf_free  ========
 *  Purpose:
 *  This will free a block of memory
 */
int heapbuf_free(void *hphandle, void *block, u32 size)
{
	int status = 0;
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	s32 retval  = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(block == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	handle = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)handle->obj;
	if (likely(obj->gate != NULL)) {
		status = gatepeterson_enter(obj->gate);
		if (status < 0) {
			retval = -EINVAL;
			goto error;
		}
	}

	retval = listmp_put_tail((void *)obj->free_list, block);
	obj->attrs->num_free_blocks++;
	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);
	return 0;

error:
	printk(KERN_ERR "heapbuf_free failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_free);

/*
 * ======== heapbuf_get_stats  ========
 *  Purpose:
 *  This will get memory statistics
 */
int heapbuf_get_stats(void *hphandle, struct memory_stats *stats)
{
	int status = 0;
	struct heap_object *object = NULL;
	struct heapbuf_obj *obj = NULL;
	u32 block_size;
	s32 retval  = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	BUG_ON(stats == NULL);

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	object = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)object->obj;

	if (likely(obj->gate != NULL)) {
		status = gatepeterson_enter(obj->gate);
		if (status < 0) {
			retval = -EINVAL;
			goto error;
		}
	}

	block_size = obj->attrs->block_size;
	stats->total_size = (u32 *)(block_size * obj->attrs->num_blocks);
	stats->total_free_size = (u32 *)(block_size *
						obj->attrs->num_free_blocks);
	if (obj->attrs->num_free_blocks)
		stats->largest_free_size =  (u32 *)block_size;
	else
		stats->largest_free_size =  (u32 *)0;

	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);
	return 0;

error:
	printk(KERN_ERR "heapbuf_get_stats failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_get_stats);

/*
 * ======== heapbuf_isblocking  ========
 *  Purpose:
 *  Indicate whether the heap may block during an alloc or free call
 */
bool heapbuf_isblocking(void *handle)
{
	bool isblocking = false;
	s32 retval  = 0;

	if (WARN_ON(handle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	/* TBD: Figure out how to determine whether the gate is blocking */
	isblocking = true;

	/* retval true  Heap blocks during alloc/free calls */
	/* retval false Heap does not block during alloc/free calls */
	return isblocking;

error:
	printk(KERN_ERR "heapbuf_isblocking status: %x\n", retval);
	return isblocking;
}
EXPORT_SYMBOL(heapbuf_isblocking);

/*
 * ======== heapbuf_get_extended_stats  ========
 *  Purpose:
 *  This will get extended statistics
 */
int heapbuf_get_extended_stats(void *hphandle,
				struct heapbuf_extended_stats *stats)
{
	int status = 0;
	struct heap_object *object = NULL;
	struct heapbuf_obj *obj = NULL;
	s32 retval  = 0;

	if (atomic_cmpmask_and_lt(&(heapbuf_state.ref_count),
				HEAPBUF_MAKE_MAGICSTAMP(0),
				HEAPBUF_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	BUG_ON(stats == NULL);
	if (WARN_ON(heapbuf_state.ns_handle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	object = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)object->obj;
	if (likely(obj->gate != NULL)) {
		status = gatepeterson_enter(obj->gate);
		if (status < 0) {
			retval = -EINVAL;
			goto error;
		}
	}

	/*
	*  The maximum number of allocations for this heapbuf (for any given
	*  instance of time during its liftime) is computed as follows:
	*
	*  max_allocated_blocks = obj->num_blocks - obj->min_free_blocks
	*
	*  Note that max_allocated_blocks is *not* the maximum allocation count,
	*  but rather the maximum allocations seen at any snapshot of time in
	*  the heapbuf instance.
	*/
	/* if nothing has been alloc'ed yet, return 0 */
	if ((u32)(obj->attrs->min_free_blocks) == -1) /* FIX THIS */
		stats->max_allocated_blocks = 0;
	else
		stats->max_allocated_blocks = obj->attrs->num_blocks
					- obj->attrs->min_free_blocks;
	/* current number of alloc'ed blocks is computed using curr # free
	*   blocks
	*/
	stats->num_allocated_blocks = obj->attrs->num_blocks
				-  obj->attrs->num_free_blocks;
	if (likely(obj->gate != NULL))
		gatepeterson_leave(obj->gate, 0);

error:
	printk(KERN_ERR "heapbuf_get_extended_stats status: %x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_get_extended_stats);

/*
 * ======== heapbuf_shared_memreq ========
 *  Purpose:
 *  This will get amount of shared memory required for
 *  creation of each instance
 */
int heapbuf_shared_memreq(const struct heapbuf_params *params, u32 *buf_size)
{
	int state_size = 0;
	listmp_sharedmemory_params listmp_params;

	BUG_ON(params == NULL);

	/* Size for attrs */
	state_size = (sizeof(struct heapbuf_attrs) + (HEAPBUF_CACHESIZE - 1))
				& ~(HEAPBUF_CACHESIZE - 1);

	listmp_params_init(NULL, &listmp_params);
	listmp_params.resource_id = params->resource_id;
	state_size += listmp_shared_memreq(&listmp_params);

	/* Determine size for the buffer */
	*buf_size = params->num_blocks * params->block_size;

	return state_size;
}
EXPORT_SYMBOL(heapbuf_shared_memreq);
