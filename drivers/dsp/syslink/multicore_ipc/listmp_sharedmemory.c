/*
 *  listmp_sharedmemory.c
 *
 *  The listmp_sharedmemory is a linked-list based module designed to be
 *  used in a multi-processor environment.  It is designed to
 *  provide a means of communication between different processors.
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

/*!
 *  This module is instance based. Each instance requires a small
 *  piece of shared memory. This is specified via the #shared_addr
 *  parameter to the create. The proper #shared_addr_size parameter
 *  can be determined via the #shared_memreq call. Note: the
 *  parameters to this function must be the same that will used to
 *  create or open the instance.
 *  The listmp_sharedmemory module uses a #NameServer instance
 *  to store instance information when an instance is created and
 *  the name parameter is non-NULL. If a name is supplied, it must
 *  be unique for all listmp_sharedmemory instances.
 *  The #create also initializes the shared memory as needed. The
 *  shared memory must be initialized to 0 or all ones
 *  (e.g. 0xFFFFFFFFF) before the listmp_sharedmemory instance
 *  is created.
 *  Once an instance is created, an open can be performed. The #open
 *  is used to gain access to the same listmp_sharedmemory instance.
 *  Generally an instance is created on one processor and opened
 *  on the other processor.
 *  The open returns a listmp_sharedmemory instance handle like the
 *  create, however the open does not modify the shared memory.
 *  Generally an instance is created on one processor and opened
 *  on the other processor.
 *  There are two options when opening the instance:
 *  @li Supply the same name as specified in the create. The
 *  listmp_sharedmemory module queries the NameServer to get the
 *  needed information.
 *  @li Supply the same #shared_addr value as specified in the
 *  create.
 *  If the open is called before the instance is created, open
 *  returns NULL.
 *  There is currently a list of restrictions for the module:
 *  @li Both processors must have the same endianness. Endianness
 *  conversion may supported in a future version of
 *  listmp_sharedmemory.
 *  @li The module will be made a gated module
 */


/* Standard headers */
#include <linux/types.h>
#include <linux/module.h>

/* Utilities headers */
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/string.h>

/* Syslink headers */
#include <syslink/atomic_linux.h>

/* Module level headers */
#include <nameserver.h>
#include <sharedregion.h>
#include <multiproc.h>
#include "_listmp.h"
#include <listmp.h>
#include <gatepeterson.h>
#include <listmp_sharedmemory.h>


/* =============================================================================
 * Globals
 * =============================================================================
 */
/*! @brief Macro to make a correct module magic number with refCount */
#define LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(x)                                  \
				((LISTMPSHAREDMEMORY_MODULEID << 12u) | (x))

/*!
 *  @brief  Name of the reserved NameServer used for listmp_sharedmemory.
 */
#define LISTMP_SHAREDMEMORY_NAMESERVER	"ListMPSharedMemory"

/*!
 *  @brief  Cache size
 */
#define LISTMP_SHAREDMEMORY_CACHESIZE	128



/* =============================================================================
 * Structures and Enums
 * =============================================================================
 */
/*! @brief structure for listmp_sharedmemory module state */
struct listmp_sharedmemory_module_object {
	atomic_t              ref_count;
	/*!< Reference count */
	void *ns_handle;
	/*!< Handle to the local NameServer used for storing GP objects */
	struct list_head obj_list;
	/*!< List holding created listmp_sharedmemory objects */
	struct mutex *local_gate;
	/*!< Handle to lock for protecting obj_list */
	struct listmp_config cfg;
	/*!< Current config values */
	struct listmp_config default_cfg;
	/*!< Default config values */
	listmp_sharedmemory_params default_inst_params;
	/*!< Default instance creation parameters */
};

/*!
 *  @var	listmp_sharedmemory_nameServer
 *
 *  @brief  Name of the reserved NameServer used for listmp_sharedmemory.
 */
static
struct listmp_sharedmemory_module_object listmp_sharedmemory_state = {
			.default_cfg.max_name_len = 32,
			.default_cfg.use_name_server = true,
			.default_inst_params.shared_addr = 0,
			.default_inst_params.shared_addr_size = 0,
			.default_inst_params.name = NULL,
			.default_inst_params.gate = NULL,
			.default_inst_params.list_type = listmp_type_SHARED};

/*!
 *  @brief  Structure for the internal Handle for the listmp_sharedmemory.
 */
struct listmp_sharedmemory_obj{
	struct list_head list_elem;
	/*!< Used for creating a linked list */
	VOLATILE struct listmp_elem *listmp_elem;
	/*!< Used for storing listmp_sharedmemory element */
	struct listmp_proc_attrs *owner;
	/*!< Creator's attributes associated with an instance */
	listmp_sharedmemory_params params;
	/*!< the parameter structure */
	void *ns_key;
	u32 index;
	/*!< the index for SrPtr */
	VOLATILE struct listmp_attrs *attrs;
	/*!< Shared memory attributes */
	void *top;
	/*!< Pointer to the top Object */
};


/* =============================================================================
 * Function definitions
 * =============================================================================
 */
/*
 * ======== _listmp_sharedmemory_create ========
 *  Purpose:
 *  Creates a new instance of listmp_sharedmemory module. This is an internal
 *  function because both listmp_sharedmemory_create and
 *  listmp_sharedmemory_open call use the same functionality.
 */
static listmp_sharedmemory_handle
_listmp_sharedmemory_create(listmp_sharedmemory_params *params,
				u32 create_flag);


/* =============================================================================
 * Function API's
 * =============================================================================
 */
/*
 * ======== listmp_sharedmemory_get_config ========
 *  Purpose:
 *  Function to get configuration parameters to setup the
 *  the listmp_sharedmemory module.
 */
int listmp_sharedmemory_get_config(struct listmp_config *cfgParams)
{
	int status = 0;

	if (WARN_ON(cfgParams == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true)
		/* If setup has not yet been called) */
		memcpy(cfgParams, &listmp_sharedmemory_state.default_cfg,
			sizeof(struct listmp_config));
	else
		memcpy(cfgParams, &listmp_sharedmemory_state.cfg,
			sizeof(struct listmp_config));
	return 0;

exit:
	printk(KERN_ERR "listmp_sharedmemory_get_config failed: "
		"status = 0x%x\n", status);
	return status;
}

/*
 * ======== listmp_sharedmemory_setup ========
 *  Purpose:
 *  Function to setup the listmp_sharedmemory module.
 */
int listmp_sharedmemory_setup(struct listmp_config *config)
{
	int status = 0;
	int status1 = 0;
	void *nshandle = NULL;
	struct nameserver_params params;
	struct listmp_config tmp_cfg;

	/* This sets the refCount variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
	atomic_cmpmask_and_set(&listmp_sharedmemory_state.ref_count,
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&listmp_sharedmemory_state.ref_count)
				!= LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (config == NULL) {
		listmp_sharedmemory_get_config(&tmp_cfg);
		config = &tmp_cfg;
	}

	if (WARN_ON(config->max_name_len == 0)) {
		status = -EINVAL;
		goto exit;
	}

	if (likely((config->use_name_server == true))) {
		/* Initialize the parameters */
		nameserver_params_init(&params);
		params.max_value_len = 4;
		params.max_name_len = config->max_name_len;
		/* Create the nameserver for modules */
		nshandle = nameserver_create(
				LISTMP_SHAREDMEMORY_NAMESERVER, &params);
		if (unlikely(nshandle == NULL)) {
			status = LISTMPSHAREDMEMORY_E_FAIL;
			goto exit;
		}
	}

	listmp_sharedmemory_state.ns_handle = nshandle;
	/* Construct the list object */
	INIT_LIST_HEAD(&listmp_sharedmemory_state.obj_list);
	/* Create a lock for protecting list object */
	listmp_sharedmemory_state.local_gate = \
		kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (listmp_sharedmemory_state.local_gate == NULL) {
		status = -ENOMEM;
		goto clean_nameserver;
	}

	mutex_init(listmp_sharedmemory_state.local_gate);
	/* Copy the cfg */
	memcpy(&listmp_sharedmemory_state.cfg, config,
			sizeof(struct listmp_config));
	return 0;

clean_nameserver:
	printk(KERN_ERR "listmp_sharedmemory_setup: Failed to create the local "
		"gate! status = 0x%x\n", status);
	atomic_set(&listmp_sharedmemory_state.ref_count,
					LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0));
	status1 = nameserver_delete
		(&(listmp_sharedmemory_state.ns_handle));
	BUG_ON(status1 < 0);

exit:
	printk(KERN_ERR "listmp_sharedmemory_setup failed! status = 0x%x\n",
		status);
	return status;
}

/*
 * ======== listmp_sharedmemory_destroy ========
 *  Purpose:
 *  Function to destroy the listmp_sharedmemory module.
 */
int listmp_sharedmemory_destroy(void)
{
	int status = 0;
	int status1 = 0;
	struct list_head *elem = NULL;
	struct list_head *head = &listmp_sharedmemory_state.obj_list;
	struct list_head *next;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (atomic_dec_return(&listmp_sharedmemory_state.ref_count)
			== LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0)) {
		/* Temporarily increment refCount here. */
		atomic_set(&listmp_sharedmemory_state.ref_count,
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1));
		/* Check if any listmp_sharedmemory instances have not been
		 * deleted so far. If not, delete them. */
		for (elem = (head)->next; elem != (head); elem = next) {
			/* Retain the next pointer so it doesn't
			   get overwritten */
			next = elem->next;
			if (((struct listmp_sharedmemory_obj *) elem)->owner
				->proc_id == multiproc_get_id(NULL)) {
				status1 = listmp_sharedmemory_delete(
				(listmp_sharedmemory_handle *)
				&(((struct listmp_sharedmemory_obj *) \
							elem)->top));
				WARN_ON(status1 < 0);
			} else {
				status1 = listmp_sharedmemory_close(
				(listmp_sharedmemory_handle)
				(((struct listmp_sharedmemory_obj *) \
							elem)->top));
				WARN_ON(status1 < 0);
			}
		}

		/* Again reset refCount. */
		atomic_set(&listmp_sharedmemory_state.ref_count,
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0));
		if (likely(listmp_sharedmemory_state.cfg.use_name_server
								== true)) {
			/* Delete the nameserver for modules */
			status = nameserver_delete(
				&(listmp_sharedmemory_state.ns_handle));
			BUG_ON(status < 0);
		}

		/* Destruct the list object */
		list_del(&listmp_sharedmemory_state.obj_list);
		/* Delete the list lock */
		kfree(listmp_sharedmemory_state.local_gate);
		listmp_sharedmemory_state.local_gate = NULL;
		memset(&listmp_sharedmemory_state.cfg, 0,
			sizeof(struct listmp_config));
		/* Decrease the refCount */
		atomic_set(&listmp_sharedmemory_state.ref_count,
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0));
	}

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_destroy failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_params_init ========
 *  Purpose:
 *  Function to initialize the config-params structure with supplier-specified
 *  defaults before instance creation.
 */
void listmp_sharedmemory_params_init(listmp_sharedmemory_handle handle,
				listmp_sharedmemory_params *params)
{
	s32 status = 0;
	struct listmp_sharedmemory_obj *obj = NULL;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (handle == NULL) {
		memcpy(params,
		&(listmp_sharedmemory_state.default_inst_params),
		sizeof(listmp_sharedmemory_params));
	} else {
		obj = (struct listmp_sharedmemory_obj *)
				(((listmp_sharedmemory_object *) handle)->obj);

			memcpy((void *)&obj->params,
			(void *)params,
			 sizeof(listmp_sharedmemory_params));
	}
	return;

exit:
	printk(KERN_ERR "listmp_sharedmemory_params_init failed! "
		"status = 0x%x\n", status);
	return;
}

/*
 * ======== listmp_sharedmemory_create ========
 *  Purpose:
 *  Creates a new instance of listmp_sharedmemory module.
 */
listmp_sharedmemory_handle listmp_sharedmemory_create(
				listmp_sharedmemory_params *params)
{
	s32 status = 0;
	listmp_sharedmemory_object *handle = NULL;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (WARN_ON((params->shared_addr == (u32)NULL)
		&& (params->list_type != listmp_type_FAST))) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params->shared_addr_size
		< listmp_sharedmemory_shared_memreq(params))) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)
		_listmp_sharedmemory_create(params, true);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_create failed! "
			"status = 0x%x\n", status);
	}
	return (listmp_sharedmemory_handle) handle;
}

/*
 * ======== listmp_sharedmemory_delete ========
 *  Purpose:
 *  Deletes a instance of listmp_sharedmemory instance object.
 */
int listmp_sharedmemory_delete(listmp_sharedmemory_handle *listmp_handleptr)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	listmp_sharedmemory_params *params = NULL;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(listmp_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(*listmp_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *) (*listmp_handleptr);
	obj = (struct listmp_sharedmemory_obj *) handle->obj;
	params = (listmp_sharedmemory_params *) &obj->params;

	if (obj->owner->proc_id != multiproc_get_id(NULL)) {
		status = -EPERM;
		goto exit;
	}

	if (obj->owner->open_count > 1) {
		status = -EBUSY;
		goto exit;
	}

	if (obj->owner->open_count != 1) {
		status = -EBUSY;
		goto exit;
	}

	/* Remove from  the local list */
	status = mutex_lock_interruptible(
				listmp_sharedmemory_state.local_gate);
	if (status)
		goto exit;
	list_del(&obj->list_elem);
	mutex_unlock(listmp_sharedmemory_state.local_gate);

	if (likely(params->name != NULL)) {
		/* Free memory for the name */
		kfree(params->name);
		/* Remove from the name server */
		if (likely(listmp_sharedmemory_state.cfg.use_name_server)) {
			if (obj->ns_key != NULL) {
				nameserver_remove_entry(
					listmp_sharedmemory_state.ns_handle,
					obj->ns_key);
				obj->ns_key = NULL;
			}
		}
	}

	/* Free memory for the processor info's */
	if (obj)
		kfree(obj->owner);
	/* Now free the handle */
	kfree(obj);
	obj = NULL;

	/* Now free the handle */
	kfree(handle);
	handle = NULL;
	*listmp_handleptr = NULL;

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_delete failed! "
			"status = 0x%x\n", status);
	}
	return status;

}


/*
 * ======== listmp_sharedmemory_shared_memreq ========
 *  Purpose:
 *  Function to return the amount of shared memory required for creation of
 *  each instance.
 */
int listmp_sharedmemory_shared_memreq(listmp_sharedmemory_params *params)
{
	int retval = 0;

	if (WARN_ON(params == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if (params->list_type == listmp_type_SHARED)
		retval = (LISTMP_SHAREDMEMORY_CACHESIZE * 2);

exit:
	if (retval < 0) {
		printk(KERN_ERR "listmp_sharedmemory_shared_memreq failed! "
			"retval = 0x%x\n", retval);
	}
	/*! @retval	((1 * sizeof(struct listmp_elem))
	*!		+  1 * sizeof(struct listmp_attrs)) if params is null */
	/*! @retval (2 * cacheSize) if params is provided */
	/*! @retval (0) if hardware queue */
	return retval;
}

/*
 * ======== listmp_sharedmemory_open ========
 *  Purpose:
 *  Function to open a listmp_sharedmemory instance
 */
int listmp_sharedmemory_open(listmp_sharedmemory_handle *listmp_handleptr,
				listmp_sharedmemory_params *params)
{
	int status = 0;
	bool done_flag = false;
	struct list_head *elem;
	u32 shared_shm_base;
	struct listmp_attrs *attrs;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(listmp_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (WARN_ON((listmp_sharedmemory_state.cfg.use_name_server == false)
				&& (params->shared_addr == (u32)NULL))) {
		status = -EINVAL;
		goto exit;
	}

	if (WARN_ON((listmp_sharedmemory_state.cfg.use_name_server == true)
					&& (params->shared_addr == (u32)NULL))
						&& (params->name == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	/* First check in the local list */
	list_for_each(elem, &listmp_sharedmemory_state.obj_list) {
		if (((struct listmp_sharedmemory_obj *)elem)->params.shared_addr
						== params->shared_addr) {
			status = mutex_lock_interruptible(
				listmp_sharedmemory_state.local_gate);
			if (status)
				goto exit;
			if (((struct listmp_sharedmemory_obj *)elem)
				->owner->proc_id
				== multiproc_get_id(NULL))
				((struct listmp_sharedmemory_obj *)elem)
					->owner->open_count++;
			mutex_unlock(
				listmp_sharedmemory_state.local_gate);
			*listmp_handleptr = \
				(((struct listmp_sharedmemory_obj *)
				elem)->top);
			done_flag = true;
			break;
		} else if ((params->name != NULL)
			&& (((struct listmp_sharedmemory_obj *)elem) \
						->params.name != NULL)){
			if (strcmp(((struct listmp_sharedmemory_obj *)elem)
				->params.name, params->name) == 0) {
				status = mutex_lock_interruptible(
					listmp_sharedmemory_state.local_gate);
				if (status)
					goto exit;
				if (((struct listmp_sharedmemory_obj *)elem)
					->owner->proc_id
					== multiproc_get_id(NULL))
					((struct listmp_sharedmemory_obj *)elem)
						->owner->open_count++;
				mutex_unlock(
					listmp_sharedmemory_state.local_gate);
				*listmp_handleptr = \
					(((struct listmp_sharedmemory_obj *)
					elem)->top);
				done_flag = true;
				break;
			}
		}
	}

	if (likely(done_flag == false)) {
		if (unlikely(params->shared_addr == NULL)) {
			if (likely(listmp_sharedmemory_state.cfg.use_name_server
								 == true)){
				/* Find in name server */
				status =
				nameserver_get(
					listmp_sharedmemory_state.ns_handle,
					params->name,
					&shared_shm_base,
					sizeof(u32),
					NULL);
				if (status >= 0)
					params->shared_addr =
						sharedregion_get_ptr(
						(u32 *)shared_shm_base);
				if (params->shared_addr == NULL) {
					status =
					LISTMPSHAREDMEMORY_E_NOTCREATED;
					goto exit;
				}
			}
		}
	}

	if (status >= 0) {
		attrs = (struct listmp_attrs *) (params->shared_addr);
		if (unlikely(attrs->status != (LISTMP_SHAREDMEMORY_CREATED)))
				status = LISTMPSHAREDMEMORY_E_NOTCREATED;
		else if (unlikely(attrs->version !=
					(LISTMP_SHAREDMEMORY_VERSION)))
				status = LISTMPSHAREDMEMORY_E_VERSION;
	}

	if (likely(status >= 0))
		*listmp_handleptr = (listmp_sharedmemory_handle)
				_listmp_sharedmemory_create(params, false);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_open failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_close ========
 *  Purpose:
 *  Function to close a previously opened instance
 */
int listmp_sharedmemory_close(listmp_sharedmemory_handle  listmp_handle)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	listmp_sharedmemory_params *params = NULL;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;
	if (obj == NULL) {
		status = -EINVAL;
		goto exit;
	}

	status = mutex_lock_interruptible(
		listmp_sharedmemory_state.local_gate);
	if (status)
		goto exit;
	if (obj->owner->proc_id == multiproc_get_id(NULL))
		(obj)->owner->open_count--;

	params = (listmp_sharedmemory_params *) &obj->params;
	/* Check if ListMP is opened on same processor*/
	if (((struct listmp_sharedmemory_obj *)obj)->owner->creator == false) {
		list_del(&obj->list_elem);
		/* remove from the name server */
		if (params->name != NULL)
			/* Free memory for the name */
			kfree(params->name);

		kfree(obj->owner);
		kfree(obj);
		obj = NULL;
		kfree(handle);
		handle = NULL;
	}

	mutex_unlock(listmp_sharedmemory_state.local_gate);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_close failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_empty ========
 *  Purpose:
 *  Function to check if the shared memory list is empty
 */
bool listmp_sharedmemory_empty(listmp_sharedmemory_handle listmp_handle)
{

	int status = 0;
	bool is_empty = false;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	s32 retval = 0;
	struct listmp_elem *sharedHead;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0) {
			status = -EINVAL;
			goto exit;
		}
	}

	/*! @retval true if list is empty */
	sharedHead = (struct listmp_elem *)(sharedregion_get_srptr(
				(void *)obj->listmp_elem, obj->index));
	dsb();
	if (obj->listmp_elem->next == sharedHead)
		is_empty = true;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	return is_empty;
}

/*
 * ======== listmp_sharedmemory_get_head ========
 *  Purpose:
 *  Function to get head element from a shared memory list
 */
void *listmp_sharedmemory_get_head(listmp_sharedmemory_handle listmp_handle)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *elem = NULL;
	struct listmp_elem *localHeadNext = NULL;
	struct listmp_elem *localNext = NULL;
	struct listmp_elem *sharedHead = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true)
		goto exit;

	if (WARN_ON(listmp_handle == NULL)) {
		/*! @retval  NULL if listmp_handle passed is NULL */
		elem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0)
			goto exit;
	}

	localHeadNext = sharedregion_get_ptr((u32 *)obj->listmp_elem->next);
	dsb();
	/* See if the listmp_sharedmemory_object was empty */
	if (localHeadNext == (struct listmp_elem *)obj->listmp_elem) {
		/*! @retval NULL if list is empty */
		elem = NULL ;
	} else {
		/* Elem to return */
		elem = localHeadNext;
		dsb();
		localNext = sharedregion_get_ptr((u32 *)elem->next);
		sharedHead = (struct listmp_elem *) sharedregion_get_srptr(
				(void *)obj->listmp_elem, obj->index);

		/* Fix the head of the list next pointer */
		obj->listmp_elem->next = elem->next;
		dsb();
		/* Fix the prev pointer of the new first elem on the
		list */
		localNext->prev = sharedHead;
	}

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	return elem;
}

/*
 * ======== listmp_sharedmemory_get_tail ========
 *  Purpose:
 *  Function to get tail element from a shared memory list
 */
void *listmp_sharedmemory_get_tail(listmp_sharedmemory_handle listmp_handle)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *elem = NULL;
	struct listmp_elem *localHeadNext = NULL;
	struct listmp_elem *localHeadPrev = NULL;
	struct listmp_elem *localPrev = NULL;
	struct listmp_elem *sharedHead = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true)
		goto exit;

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		/*! @retval NULL if Module was not initialized */
		elem = NULL;
		goto exit;
	}
	if (WARN_ON(listmp_handle == NULL)) {
		/*! @retval NULL if listmp_handle passed is NULL */
		elem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0)
			goto exit;
	}

	localHeadNext = sharedregion_get_ptr((u32 *)obj->listmp_elem->next);
	localHeadPrev = sharedregion_get_ptr((u32 *)obj->listmp_elem->prev);

	/* See if the listmp_sharedmemory_object was empty */
	if (localHeadNext == (struct listmp_elem *)obj->listmp_elem) {
		/* Empty, return NULL */
		elem = NULL ;
	} else {
		/* Elem to return */
		elem = localHeadPrev;
		localPrev = sharedregion_get_ptr((u32 *)elem->prev);
		sharedHead = (struct listmp_elem *) sharedregion_get_srptr(
					(void *)obj->listmp_elem, obj->index);
		obj->listmp_elem->prev = elem->prev;
		localPrev->next = sharedHead;
	}

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	return elem;
}

/*
 * ======== listmp_sharedmemory_put_head ========
 *  Purpose:
 *  Function to put head element into a shared memory list
 */
int listmp_sharedmemory_put_head(listmp_sharedmemory_handle listmp_handle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localNextElem = NULL;
	struct listmp_elem *sharedElem = NULL;
	struct listmp_elem *sharedHead = NULL;
	s32 retval = 0;
	u32 index;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;
	dsb();
	index = sharedregion_get_index(elem);
	sharedElem = (struct listmp_elem *) sharedregion_get_srptr(elem, index);
	sharedHead = (struct listmp_elem *)sharedregion_get_srptr(
					(void *)obj->listmp_elem, obj->index);
	dsb();
	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0) {
			status = -EINVAL;
			goto exit;
		}
	}
	/* Add the new elem into the list */
	elem->next = obj->listmp_elem->next;
	elem->prev = sharedHead;
	dsb();
	localNextElem = sharedregion_get_ptr((u32 *)elem->next);
	localNextElem->prev = sharedElem;
	obj->listmp_elem->next = sharedElem;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_put_head failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_put_tail ========
 *  Purpose:
 *  Function to put tail element into a shared memory list
 */
int listmp_sharedmemory_put_tail(listmp_sharedmemory_handle listmp_handle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *sharedElem = NULL;
	struct listmp_elem *sharedHead = NULL;
	s32 retval = 0;
	u32 index;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;
	dsb();
	/* Safe to do outside the gate */
	index = sharedregion_get_index(elem);
	sharedElem = (struct listmp_elem *)
			sharedregion_get_srptr(elem, index);
	sharedHead = (struct listmp_elem *)sharedregion_get_srptr
				((void *)obj->listmp_elem,
				obj->index);

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0) {
			status = -EINVAL;
			goto exit;
		}
	}
	/* Add the new elem into the list */
	elem->next = sharedHead;
	elem->prev = obj->listmp_elem->prev;
	dsb();
	localPrevElem = sharedregion_get_ptr((u32 *)elem->prev);
	dsb();
	localPrevElem->next = sharedElem;
	obj->listmp_elem->prev = sharedElem;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_put_tail failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_insert ========
 *  Purpose:
 *  Function to insert an element into a shared memory list
 */
int listmp_sharedmemory_insert(listmp_sharedmemory_handle  listmp_handle,
				struct listmp_elem *new_elem,
				struct listmp_elem *cur_elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *sharedNewElem = NULL;
	struct listmp_elem *sharedCurElem = NULL;
	s32 retval = 0;
	u32 index;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(new_elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0) {
			status = -EINVAL;
			goto exit;
		}
	}

	/* If NULL change cur_elem to the obj */
	if (cur_elem == NULL)
		cur_elem = (struct listmp_elem *)obj->listmp_elem->next;

	/* Get SRPtr for new_elem */
	index = sharedregion_get_index(new_elem);
	sharedNewElem = (struct listmp_elem *)
				sharedregion_get_srptr(new_elem, index);
	dsb();
	/* Get SRPtr for cur_elem */
	index = sharedregion_get_index(cur_elem);
	sharedCurElem = (struct listmp_elem *)
				sharedregion_get_srptr(cur_elem, index);

	/* Get SRPtr for cur_elem->prev */
	localPrevElem = sharedregion_get_ptr((u32 *)cur_elem->prev);
	dsb();
	new_elem->next = sharedCurElem;
	new_elem->prev = cur_elem->prev;
	localPrevElem->next = sharedNewElem;
	dsb();
	cur_elem->prev = sharedNewElem;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_insert failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_remove ========
 *  Purpose:
 *  Function to remove a element from a shared memory list
 */
int listmp_sharedmemory_remove(listmp_sharedmemory_handle  listmp_handle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj	*obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *localNextElem = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
			LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0) {
			status = -EINVAL;
			goto exit;
		}
	}

	localPrevElem = sharedregion_get_ptr((u32 *)elem->prev);
	localNextElem = sharedregion_get_ptr((u32 *)elem->next);
	dsb();
	localPrevElem->next = elem->next;
	localNextElem->prev = elem->prev;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_sharedmemory_remove failed! "
			"status = 0x%x\n", status);
	}
	return status;
}

/*
 * ======== listmp_sharedmemory_next ========
 *  Purpose:
 *  Function to traverse to next element in shared memory list
 */
void *listmp_sharedmemory_next(listmp_sharedmemory_handle listmp_handle,
				struct listmp_elem *elem)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *retElem = NULL;
	struct listmp_elem *sharedNextElem = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true)
		goto exit;

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0)
			goto exit;
	}

	/* If element is NULL start at head */
	if (elem == NULL)
		sharedNextElem = (struct listmp_elem *) obj->listmp_elem->next;
	else
		sharedNextElem = (struct listmp_elem *)elem->next;

	retElem = sharedregion_get_ptr((u32 *)sharedNextElem);

	/*! @retval NULL if list is empty */
	if (retElem == (struct listmp_elem *)obj->listmp_elem)
		retElem = NULL;
	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	return retElem;
}

/*
 * ======== listmp_sharedmemory_prev ========
 *  Purpose:
 *  Function to traverse to prev element in shared memory list
 */
void *listmp_sharedmemory_prev(listmp_sharedmemory_handle listmp_handle,
				struct listmp_elem *elem)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *retElem = NULL;
	struct listmp_elem *sharedPrevElem = NULL;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(listmp_sharedmemory_state.ref_count),
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(0),
				LISTMPSHAREDMEMORY_MAKE_MAGICSTAMP(1)) == true)
		goto exit;

	handle = (listmp_sharedmemory_object *)listmp_handle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.gate != NULL) {
		retval = gatepeterson_enter(obj->params.gate);
		if (retval < 0)
			goto exit;
	}

	/* If elem is NULL use head */
	if (elem == NULL)
		sharedPrevElem = (struct listmp_elem *)
						obj->listmp_elem->prev;
	else
		sharedPrevElem = (struct listmp_elem *)elem->prev;

	retElem = sharedregion_get_ptr((u32 *)sharedPrevElem);

	/*! @retval NULL if list is empty */
	if (retElem == (struct listmp_elem *)(obj->listmp_elem))
		retElem = NULL;

	if (obj->params.gate != NULL)
		gatepeterson_leave(obj->params.gate, 0);

exit:
	return retElem;
}

/*
 * ======== _listmp_sharedmemory_create ========
 *  Purpose:
 *  Creates a new instance of listmp_sharedmemory module. This is an internal
 *  function because both listmp_sharedmemory_create and
 *  listmp_sharedmemory_open call use the same functionality.
 */
listmp_sharedmemory_handle _listmp_sharedmemory_create(
		listmp_sharedmemory_params *params, u32 create_flag)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	u32 key;
	u32 shm_index;
	u32 *shared_shm_base;

	BUG_ON(params == NULL);

	/* Allow local lock not being provided. Don't do protection if local
	* lock is not provided.
	*/
	/* Create the handle */
	handle = kzalloc(sizeof(listmp_sharedmemory_object), GFP_KERNEL);
	if (handle == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	obj = kzalloc(sizeof(struct listmp_sharedmemory_obj),
			GFP_KERNEL);
	if (obj == NULL) {
		/*! @retval NULL if Memory allocation failed for
		* internal object "Memory allocation failed for handle"
		* "of type struct listmp_sharedmemory_obj"); */
		status = -ENOMEM;
		goto exit;
	}

	handle->obj = (struct listmp_sharedmemory_obj *)obj ;
	handle->empty = &listmp_sharedmemory_empty;
	handle->get_head = &listmp_sharedmemory_get_head;
	handle->get_tail = &listmp_sharedmemory_get_tail;
	handle->put_head = &listmp_sharedmemory_put_head;
	handle->put_tail = &listmp_sharedmemory_put_tail;
	handle->insert = &listmp_sharedmemory_insert;
	handle->remove = &listmp_sharedmemory_remove;
	handle->next = &listmp_sharedmemory_next;
	handle->prev = &listmp_sharedmemory_prev;

	/* Update attrs */
	obj->attrs = (struct listmp_attrs *) params->shared_addr;
	/* Assign the memory with proper cache line padding */
	obj->listmp_elem = (void *) ((u32)obj->attrs + \
			LISTMP_SHAREDMEMORY_CACHESIZE);
	obj->index = sharedregion_get_index(params->shared_addr);

	if (create_flag == true) {
		obj->attrs->shared_addr_size = params->shared_addr_size;
		obj->attrs->version = LISTMP_SHAREDMEMORY_VERSION;
		obj->listmp_elem->next = obj->listmp_elem->prev =
			(struct listmp_elem *)
			(sharedregion_get_srptr((void *)obj->listmp_elem,
							obj->index));
	}

	/* Populate the params member */
	memcpy((void *)&obj->params, (void *)params,
			sizeof(listmp_sharedmemory_params));

	if (likely(listmp_sharedmemory_state.cfg.use_name_server
							== true)) {
		if (obj->params.name != NULL) {
			/* Copy the name */
			obj->params.name = kmalloc(strlen(params->name) + 1,
								GFP_KERNEL);

			if (obj->params.name == NULL) {
				/*! @retval NULL if Memory allocation failed for
				name */
				status = -ENOMEM;
			} else {
				strncpy(obj->params.name, params->name,
						strlen(params->name) + 1);
			}
		}
	}

	/* Update processor information */
	obj->owner = kmalloc(sizeof(struct listmp_proc_attrs),
							GFP_KERNEL);
	if (obj->owner == NULL) {
		printk(KERN_ERR "_listmp_sharedmemory_create: Memory "
			"allocation failed for processor information!\n");
		status = -ENOMEM;
	} else {
		/* Update owner and opener details */
		if (create_flag == true) {
			obj->owner->creator = true;
			obj->owner->open_count = 1;
			obj->owner->proc_id = multiproc_get_id(NULL);
			obj->top = handle;
			obj->attrs->status = \
				LISTMP_SHAREDMEMORY_CREATED;
		} else {
			obj->owner->creator = false;
			obj->owner->open_count = 0;
			obj->owner->proc_id = MULTIPROC_INVALIDID;
			obj->top = handle;
		}

		/* Put in the local list */
		key = mutex_lock_interruptible(
			listmp_sharedmemory_state.local_gate);
		INIT_LIST_HEAD(&obj->list_elem);
		list_add_tail((&obj->list_elem),
			&listmp_sharedmemory_state.obj_list);
		mutex_unlock(listmp_sharedmemory_state.local_gate);

		if (create_flag == true) {

			/* We will store a shared pointer in the
			 * NameServer
			 */
			shm_index = sharedregion_get_index(params->shared_addr);
			shared_shm_base = sharedregion_get_srptr(
						params->shared_addr, shm_index);
			/* Add list instance to name server */
			if (obj->params.name != NULL) {
				obj->ns_key = nameserver_add_uint32(
					listmp_sharedmemory_state.ns_handle,
					params->name,
					(u32) (shared_shm_base));
				if (obj->ns_key == NULL)
					status = -EFAULT;
			}
		}
	}

	if (status < 0) {
		/* Remove from  the local list */
		key = mutex_lock_interruptible(
					listmp_sharedmemory_state.local_gate);
		list_del(&obj->list_elem);
		mutex_unlock(listmp_sharedmemory_state.local_gate);

		if (likely(listmp_sharedmemory_state.cfg.use_name_server
								== true)) {
			if ((obj->params.name != NULL) && (status != -EFAULT))
				nameserver_remove_entry(
					listmp_sharedmemory_state.ns_handle,
					obj->ns_key);
		}
		if (obj->owner != NULL)
			kfree(obj->owner);

		if (obj->params.name != NULL)
			kfree(obj->params.name);

		if (obj != NULL) {
			kfree(obj);
			obj = NULL;
		}
		if (handle != NULL) {
			kfree(handle);
			handle = NULL;
		}
	}

exit:
	if (status < 0) {
		printk(KERN_ERR "_listmp_sharedmemory_create failed! "
			"status = 0x%x\n", status);
	}
	return (listmp_sharedmemory_handle) handle;
}
