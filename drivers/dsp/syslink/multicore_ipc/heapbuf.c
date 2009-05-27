/*
 *  heapbuf.h
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

#include <gt.h>
#include <multiproc.h>
#include <nameserver.h>
#include <sharedregion.h>
#include <heapbuf.h>
#include <listmp.h>
#include <listmp_sharedmemory.h>


/*
 *  Name of the reserved nameserver used for heapbuf.
 */
#define HEAPBUF_NAMESERVER  "HeapBuf"
#define HEAPBUF_MAX_NAME_LEN   32
#define HEAPBUF_CACHESIZE              128

/*
 *  Structure defining attribute parameters for the heapbuf module
 */
struct heapbuf_attrs {
	u32 version;
	u32 status;
	u32 num_free_blocks;
	u32 min_free_blocks;
	u32 block_size;
	u32 align;
	u32 num_blocks;
	char *buf;
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
	struct list_head obj_list; /* List holding created objects */
	void *nshandle;
	struct mutex *list_lock; /* lock for protecting obj_list */
	struct heap_config cfg;
	struct heap_config defaultcfg; /* Default config values */
	struct heapbuf_params default_inst_params; /* Default instance
						creation parameters */
};

struct heapbuf_module_object heapbuf_state = {
	.obj_list = LIST_HEAD_INIT(heapbuf_state.obj_list),
	.defaultcfg.max_name_len = HEAPBUF_MAX_NAME_LEN,
	.defaultcfg.track_max_allocs = false,
	.default_inst_params.lock_handle = NULL,
	.default_inst_params.exact = false,
	.default_inst_params.name = NULL,
	.default_inst_params.align = 1,
	.default_inst_params.num_blocks = 0,
	.default_inst_params.block_size = 0,
	.default_inst_params.shared_addr = NULL,
	.default_inst_params.shared_addr_size = 0
};


/*
 *  Structure for the handle for the heapbuf
 */
struct heapbuf_obj {
	struct list_head list_elem; /* Used for creating a linked list */
	struct heapbuf_params params; /* The creation parameter structure */
	struct heapbuf_attrs *attrs; /* The shared attributes structure */
	void *free_list; /* List of free buffers */
	struct mutex *lock_handle; /* Lock used for list */
	void *nskey; /*! nameserver key required for remove */
	struct heapbuf_proc_attrs *owner; /* owner processor info */
	struct heapbuf_proc_attrs *remote; /* Remote processor info */
	void *top;
};

/*
 * ======== heapbuf_get_config ========
 *  Purpose:
 *  This will get default configuration for the
 *  heapbuf module
 */
int heapbuf_get_config(struct heap_config *cfgparams)
{
	gt_1trace(heapbuf_debugmask, GT_ENTER, "heapbuf_get_config:\n"
		"cfgparams: %x \n", cfgparams);
	BUG_ON(cfgparams == NULL);

	if (heapbuf_state.nshandle == NULL)
		memcpy(cfgparams, &heapbuf_state.defaultcfg,
					sizeof(struct heap_config));
	else
		memcpy(cfgparams, &heapbuf_state.cfg,
					sizeof(struct heap_config));
	return 0;
}
EXPORT_SYMBOL(heapbuf_get_config);

/*
 * ======== heapbuf_setup ========
 *  Purpose:
 *  This will setup the heapbuf module
 */
int heapbuf_setup(const struct heap_config *config)
{
	struct nameserver_params params;
	void *nshandle = NULL;
	s32 retval  = 0;

	gt_1trace(heapbuf_debugmask, GT_ENTER, "heapbuf_setup:\n"
		"config: %x \n", config);
	BUG_ON(config == NULL);

	if (config->max_name_len == 0 ||
		config->max_name_len > HEAPBUF_MAX_NAME_LEN) {
		retval = -EINVAL;
		goto error;
	}

	heapbuf_state.list_lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (heapbuf_state.list_lock == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	nameserver_params_init(&params);
	params.max_value_len = sizeof(u32);
	params.max_name_len = config->max_name_len;
	nshandle  = nameserver_create(HEAPBUF_NAMESERVER, &params);
	if (nshandle == NULL)
		goto ns_create_fail;

	heapbuf_state.nshandle = nshandle;
	memcpy(&heapbuf_state.cfg, config, sizeof(struct heap_config));
	mutex_init(heapbuf_state.list_lock);
	return 0;

ns_create_fail:
	kfree(heapbuf_state.list_lock);

error:
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

	gt_0trace(heapbuf_debugmask, GT_ENTER, "heapbuf_destroy:\n");

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval = -ENODEV;
		goto error;
	}

#if 0
	/*  Check if any heapbuf instances have not been deleted so far.
	  *  if not, delete them IS THIS IS SYSLINK IMP ?
	  */
	List_for_each(elem, &heapbuf_state.obj_list) {
		heapbuf_delete(((struct heapbuf_obj *)(elem))->top);
	}
#endif

	/* If a heapbuf instance exist, do not proceed IS THIS OK  */
	if (!list_empty(&heapbuf_state.obj_list)) {
		retval = -EBUSY;
		goto error;
	}

	retval = nameserver_delete(&heapbuf_state.nshandle);
	if (unlikely(retval != 0))
		goto error;

	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

	lock = heapbuf_state.list_lock;
	heapbuf_state.list_lock = NULL;
	mutex_unlock(lock);
	kfree(lock);
	memset(&heapbuf_state.cfg, 0, sizeof(struct heap_config));
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
	struct heap_object *object = NULL;

	gt_2trace(heap_debugmask, GT_6CLASS, "heapbuf_params_init:\n"
		"handle: %x, params: %x \n", handle, params);
	BUG_ON(params == NULL);

	if (handle == NULL)
		memcpy(params, &heapbuf_state.default_inst_params,
					sizeof(struct heapbuf_params));
	else {
		object = (struct heap_object *)handle;
		obj = (struct heapbuf_obj *)object->obj;
		memcpy(&obj->params, params, sizeof(struct heapbuf_params));
	}
}
EXPORT_SYMBOL(heapbuf_params_init);

/*
 * ======== _heapbuf_create ========
 *  Purpose:
 *  This will create a new instance of heapbuf module
 *  This is an internal function as both heapbuf_create
 *  and heapbuf_open use the functionality
 */
static void *_heapbuf_create(const struct heapbuf_params *params,
				u32 createflag)
{
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	char *buf = NULL;
	listmp_sharedmemory_params listmp_params;
	s32 retval  = 0;
	u32 i;
	s32 align;
	s32 shmindex;
	u32 shared_shmbase;
	void *entry = NULL;

	gt_2trace(heap_debugmask, GT_6CLASS, "_heapbuf_create:\n"
		"params: %x, createflag: %x \n", params, createflag);
	BUG_ON(params == NULL);

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
	handle->get_extended_stats = &heapbuf_get_extended_stats;
	/* Create the shared list */
	listmp_sharedmemory_params_init(NULL, &listmp_params);
	listmp_params.shared_addr = (u32 *)((u32) (params->shared_addr)
						+ HEAPBUF_CACHESIZE);
	listmp_params.shared_addr_size =
			listmp_sharedmemory_shared_memreq(&listmp_params);
	listmp_params.lock_handle = params->lock_handle;
	if (createflag == true)
		obj->free_list = listmp_sharedmemory_create(&listmp_params);
	else
		listmp_sharedmemory_open(&obj->free_list, &listmp_params);

	if (obj->free_list == NULL) {
		retval = -ENOMEM;
		goto listmp_error;
	}
	obj->lock_handle = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (obj->lock_handle == NULL) {
		retval = -ENOMEM;
		goto gate_create_error;
	}

	/* Assign the memory with proper cache line padding */
	obj->attrs = (struct heapbuf_attrs *) params->shared_addr;
	obj->attrs->version = HEAPBUF_VERSION;
	obj->attrs->num_free_blocks = params->num_blocks;
	obj->attrs->min_free_blocks = params->num_blocks;
	obj->attrs->block_size = params->block_size;
	obj->attrs->align  = params->align;
	obj->attrs->num_blocks = params->num_blocks;
	obj->attrs->buf = (char *)((u32 *)(obj->attrs) +
			HEAPBUF_CACHESIZE + listmp_params.shared_addr_size);
	buf = obj->attrs->buf;
	align = obj->attrs->align;
	buf = (char *)(((u32)buf + (align - 1)) & ~(align - 1));
	obj->attrs->buf	= buf;
	/*
	*  Split the buffer into blocks that are length
	* block_size" and add into the free_list Queue
	*/
	for (i = 0; i < obj->attrs->num_blocks; i++) {
		listmp_put_tail((void *)obj->free_list,
					(struct listmp_elem *)buf);
		buf += obj->attrs->block_size;
	}

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

	/* Update processor information */
	obj->owner   =  kmalloc(sizeof(struct heapbuf_proc_attrs), GFP_KERNEL);
	obj->remote  =  kmalloc(sizeof(struct heapbuf_proc_attrs), GFP_KERNEL);
	if (obj->owner == NULL || obj->remote == NULL) {
		retval = -ENOMEM;
		goto proc_attr_alloc_error;
	}

	if (createflag == true) {
		obj->remote->creator    = false;
		obj->remote->open_count = 0;
		obj->remote->proc_id    = MULTIPROC_INVALIDID;
		obj->owner->creator     = true;
		obj->owner->open_count  = 1;
		obj->owner->proc_id     = multiproc_get_id(NULL);
		obj->top		= handle;
		obj->attrs->status      = HEAPBUF_CREATED;
	} else {
		obj->remote->creator    = true;
		obj->remote->open_count = 1;
		obj->remote->proc_id    = MULTIPROC_INVALIDID;
		obj->owner->creator     = false;
		obj->owner->open_count  = 0;
		obj->owner->proc_id     = multiproc_get_id("SysM3");
		obj->top		= handle;
	}

	/* We will store a shared pointer in the nameserver */
	shmindex = sharedregion_get_index(params->shared_addr);
	shared_shmbase = (u32)sharedregion_get_srptr(params->shared_addr,
								shmindex);
	if (obj->params.name != NULL) {
		entry = nameserver_add_uint32(heapbuf_state.nshandle,
						params->name,
						(u32)(params->shared_addr));
		if (entry == NULL)
			goto ns_add_error;
	}

	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto lock_error;

	INIT_LIST_HEAD(&obj->list_elem);
	list_add_tail(&obj->list_elem, &heapbuf_state.obj_list);
	mutex_unlock(heapbuf_state.list_lock);
	return (void *)handle;

ns_add_error:  /* Fall through */
lock_error: /* Fall through */
proc_attr_alloc_error:
	kfree(obj->owner);
	kfree(obj->remote);

name_alloc_error: /* Fall through */
	listmp_sharedmemory_delete((listmp_sharedmemory_handle *)
					&obj->free_list);
gate_create_error: /* Fall through */
listmp_error:
	kfree(obj);

obj_alloc_error:
	kfree(handle);

error:
	printk(KERN_ERR "_heapbuf_create failed status: %x\n", retval);
	return NULL;
}

/*
 * ======== heapbuf_create ========
 *  Purpose:
 *  This will create a new instance of heapbuf module
 */
void *heapbuf_create(const struct heapbuf_params *params)
{
	s32 retval  = 0;
	u32 size;
	void *handle = NULL;

	gt_1trace(heap_debugmask, GT_6CLASS, "heapbuf_create:\n"
		"params: %x \n", params);
	BUG_ON(params == NULL);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval = -ENODEV;
		goto error;
	}

	if (params->shared_addr == NULL) {
		retval = -EINVAL;
		goto error;
	}

	size = heapbuf_shared_memreq(params);
	if (params->shared_addr_size < size) {
		retval = -EINVAL;
		goto error;
	}

	handle = _heapbuf_create(params, true);
	return handle;

error:
	printk(KERN_ERR "heapbuf_create failed status: %x\n", retval);
	return handle;
}
EXPORT_SYMBOL(heapbuf_create);

/*
 * ======== heapbuf_delete ========
 *  Purpose:
 *  This will delete an instance of heapbuf module
 */
int heapbuf_delete(void **hphandle)
{
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	struct heapbuf_params *params = NULL;
	s32 retval  = 0;
	u16 myproc_id;
	struct mutex *lock = NULL;

	gt_1trace(heap_debugmask, GT_6CLASS, "heapbuf_delete:\n"
		"params: %x \n", params);
	BUG_ON(hphandle == NULL);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval = -ENODEV;
		goto error;
	}

	handle = (struct heap_object *)(*hphandle);
	if (WARN_ON(handle == NULL)) {
		retval = -EINVAL;
		goto error;
	}

	obj = (struct heapbuf_obj *)handle->obj;
	myproc_id =  multiproc_get_id(NULL);
	if (obj->owner->proc_id != myproc_id) {
		retval  = -EPERM;
		goto error;
	}

	retval = mutex_lock_interruptible(obj->lock_handle);
	if (retval)
		goto error;

	if (obj->owner->open_count != 1 || obj->remote->open_count != 0) {
		retval  = -EBUSY;
		goto device_busy_error;;
	}

	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto list_lock_error;

	list_del(&obj->list_elem);
	mutex_unlock(heapbuf_state.list_lock);
	params = (struct heapbuf_params *) &obj->params;
	if (params->name != NULL) {
		retval = nameserver_remove(heapbuf_state.nshandle,
						params->name);
		if (retval != 0)
			goto ns_remove_error;

		kfree(params->name);
	}

	lock = obj->lock_handle;
	obj->lock_handle = NULL;
	mutex_unlock(lock);
	kfree(lock);
	retval = listmp_sharedmemory_delete(&obj->free_list);
	kfree(obj->owner);
	kfree(obj->remote);
	kfree(obj);
	kfree(handle);
	*hphandle = NULL;
	return 0;

ns_remove_error: /* Fall through */
list_lock_error: /* Fall through */
device_busy_error:
	mutex_unlock(obj->lock_handle);

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
int heapbuf_open(void **hphandle,
			const struct heapbuf_params *params)
{
	struct heapbuf_obj *obj = NULL;
	bool found = false;
	u32 size;
	s32 retval = 0;
	u16 myproc_id;

	gt_1trace(heap_debugmask, GT_ENTER,
		"heapbuf_open:\n params: %x\n", params);
	BUG_ON(hphandle == NULL);
	BUG_ON(params == NULL);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval  = -ENODEV;
		goto error;
	}

	if (*hphandle == NULL) {
		retval  = -EINVAL;
		goto error;
	}

	if ((params->name == NULL) && (params->shared_addr == (u32)NULL)) {
		retval = -EINVAL;
		goto error;
	}

	size = heapbuf_shared_memreq(params);
	if (params->shared_addr != NULL && params->shared_addr_size < size) {
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
						heapbuf_state.list_lock);
			if (obj->owner->proc_id == myproc_id)
				obj->owner->open_count++;
			else
				obj->remote->open_count++;
			*hphandle = obj->top;
			mutex_unlock(heapbuf_state.list_lock);
		}
	}

	if (found == false)
		*hphandle = (void *)_heapbuf_create(params, false);

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
int heapbuf_close(void *hphandle)
{
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	struct heapbuf_params *params = NULL;
	s32 retval = 0;
	u16 myproc_id = 0;
	struct mutex *lock = NULL;

	gt_1trace(heapbuf_debugmask, GT_ENTER,
	    "heapbuf_close:\n hphandle: %x \n", hphandle);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	handle = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)handle->obj;
	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

	myproc_id = multiproc_get_id(NULL);
	if ((obj->remote->proc_id == myproc_id)
			&& (obj->remote->open_count == 0)) {
		list_del(&obj->list_elem);
		retval = mutex_lock_interruptible(obj->lock_handle);
		if (retval)
			goto error;

		params = (struct heapbuf_params *)&obj->params;
		if (params->name != NULL) {
			retval = nameserver_remove(heapbuf_state.nshandle,
							params->name);
			if (unlikely(retval != 0))
				goto error;
		}

		kfree(params->name);
	}

	lock = obj->lock_handle;
	obj->lock_handle = NULL;
	mutex_unlock(lock);
	kfree(lock);
	listmp_sharedmemory_delete((listmp_sharedmemory_handle *)
				&obj->free_list);
	kfree(obj->owner);
	kfree(obj->remote);
	kfree(obj);
	kfree(handle);
	handle = NULL;
	mutex_unlock(heapbuf_state.list_lock);
	return 0;

error:
	printk(KERN_ERR "heapbuf_close failed status: %x\n", retval);
	return retval;

}
EXPORT_SYMBOL(heapbuf_close);

/*
 * ======== heapbuf_free  ========
 *  Purpose:
 *  This will allocs a block of memory
 */
void *heapbuf_alloc(void *hphandle, u32 size, u32 align)
{
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	char *block = NULL;
	s32 retval  = 0;

	gt_3trace(heapbuf_debugmask, GT_ENTER,
		"heapbuf_free:\n"
		"hphandle: %x, size: %x, align: %x\n",
		hphandle, size, align);

	if (WARN_ON(heapbuf_state.nshandle == NULL))
		goto error;

	if (WARN_ON(hphandle == NULL))
		goto error;

	if (WARN_ON(size == 0))
		goto error;


	handle = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)handle->obj;
	if (size > obj->attrs->block_size)
		goto error;

	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

	block = listmp_get_head((void *)obj->free_list);
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

	mutex_unlock(heapbuf_state.list_lock);
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
	struct heap_object *handle = NULL;
	struct heapbuf_obj *obj = NULL;
	s32 retval  = 0;

	gt_3trace(heapbuf_debugmask, GT_ENTER,
		"heapbuf_free:\n"
		"hphandle: %x, block: %x, size: %x\n",
		hphandle, block, size);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval  = -EINVAL;
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
	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

	retval = listmp_put_tail((void *)obj->free_list, block);
	obj->attrs->num_free_blocks++;
	mutex_unlock(heapbuf_state.list_lock);
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
	struct heap_object *object = NULL;
	struct heapbuf_obj *obj = NULL;
	u32 block_size;
	s32 retval  = 0;

	gt_2trace(heapbuf_debugmask, GT_ENTER,
		"heapbuf_get_stats:\n"
		"hphandle: %x, stats: %x\n", hphandle, stats);
	BUG_ON(stats == NULL);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	object = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)object->obj;
	block_size = obj->attrs->block_size;
	stats->total_size = (u32 *)(block_size * obj->attrs->num_blocks);
	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

	stats->total_free_size = (u32 *)(block_size *
						obj->attrs->num_free_blocks);
	if (obj->attrs->num_free_blocks)
		stats->largest_free_size =  (u32 *)block_size;
	else
		stats->largest_free_size =  (u32 *)0;

	mutex_unlock(heapbuf_state.list_lock);
	return 0;

error:
	printk(KERN_ERR "heapbuf_get_stats failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(heapbuf_get_stats);

/*
 * ======== heapbuf_get_extended_stats  ========
 *  Purpose:
 *  This will get extended statistics
 */
int heapbuf_get_extended_stats(void *hphandle,
				struct heap_extended_stats *stats)
{
	struct heap_object *object = NULL;
	struct heapbuf_obj *obj = NULL;
	s32 retval  = 0;

	gt_2trace(heapbuf_debugmask, GT_ENTER,
		"heapbuf_get_extended_stats:\n"
		"hphandle: %x, stats: %x\n", hphandle, stats);
	BUG_ON(stats == NULL);

	if (WARN_ON(heapbuf_state.nshandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	if (WARN_ON(hphandle == NULL)) {
		retval  = -EINVAL;
		goto error;
	}

	object = (struct heap_object *)(hphandle);
	obj = (struct heapbuf_obj *)object->obj;
	retval = mutex_lock_interruptible(heapbuf_state.list_lock);
	if (retval)
		goto error;

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
	mutex_unlock(heapbuf_state.list_lock);

error:
	printk(KERN_ERR "heapbuf_get_extended_stats failed status: %x\n",
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
int heapbuf_shared_memreq(const struct heapbuf_params *params)
{
	s32 retval = 0;
	listmp_sharedmemory_params listmp_params;

	gt_1trace(heapbuf_debugmask, GT_ENTER,
			"heapbuf_sharedmemreq", params);
	BUG_ON(params == NULL);

	retval = params->num_blocks * params->block_size;
	listmp_sharedmemory_params_init(NULL, &listmp_params);
	retval += listmp_sharedmemory_shared_memreq(&listmp_params);
	retval += HEAPBUF_CACHESIZE; /* Add in attrs */
	return retval;
}
EXPORT_SYMBOL(heapbuf_shared_memreq);

