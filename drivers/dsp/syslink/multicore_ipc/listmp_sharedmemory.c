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
#include <gt.h>

/* Module level headers */
#include <nameserver.h>
#include <sharedregion.h>
#include <multiproc.h>
#include "_listmp.h"
#include <listmp.h>
#include <listmp_sharedmemory.h>


/* =============================================================================
 * Globals
 * =============================================================================
 */
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
	void *ns_handle;
	/*!< Handle to the local NameServer used for storing GP objects */
	struct list_head obj_list;
	/*!< List holding created listmp_sharedmemory objects */
	struct mutex *lock_handle;
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
			.default_inst_params.shared_addr = 0,
			.default_inst_params.shared_addr_size = 0,
			.default_inst_params.name = NULL,
			.default_inst_params.lock_handle = NULL,
			.default_inst_params.heap_handle = NULL,
			.default_inst_params.list_type = listmp_type_SHARED};

/*!
 *  @brief  Structure for the internal Handle for the listmp_sharedmemory.
 */
struct listmp_sharedmemory_obj{
	struct list_head list_elem;
	/*!< Used for creating a linked list */
	struct listmp_elem *listmp_elem;
	/*!< Used for storing listmp_sharedmemory element */
	struct listmp_proc_attrs *owner;
	/*!< Creator's attributes associated with an instance */
	struct listmp_proc_attrs *remote;
	/*!< Peer's attributes assoicated with an instance */
	listmp_sharedmemory_params params;
	/*!< the parameter structure */
	u32 index;
	/*!< the index for SrPtr */
	struct listmp_attrs *attrs;
	/*!< Shared memory attributes */
	void *top;
	/*!< Pointer to the top Object */
};

#if GT_TRACE
/* GT trace variable */
static struct GT_Mask listmpshm_debugmask = { NULL, NULL };
EXPORT_SYMBOL(listmpshm_debugmask);
#endif

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

	gt_1trace(listmpshm_debugmask, GT_ENTER,
		"listmp_sharedmemory_get_config", cfgParams);

	BUG_ON(cfgParams == NULL);
	if (WARN_ON(cfgParams == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (listmp_sharedmemory_state.ns_handle == NULL)
		/* If setup has not yet been called) */
		memcpy(cfgParams, &listmp_sharedmemory_state.default_cfg,
			sizeof(struct listmp_config));
	else
		memcpy(cfgParams, &listmp_sharedmemory_state.cfg,
			sizeof(struct listmp_config));

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE,
		"listmp_sharedmemory_get_config", status);
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

	gt_1trace(listmpshm_debugmask, GT_ENTER, "ListMPSharedmemsetup",
			config);

	if (WARN_ON(config == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(config->max_name_len == 0)) {
		status = -EINVAL;
		goto exit;
	}

	/* Initialize the parameters */
	nameserver_params_init(&params);
	params.max_value_len = 4;
	params.max_name_len = config->max_name_len;
	/* Create the nameserver for modules */
	nshandle = nameserver_create(LISTMP_SHAREDMEMORY_NAMESERVER, &params);
	if (unlikely(nshandle == NULL))
		goto exit;

	listmp_sharedmemory_state.ns_handle = nshandle;
	/* Construct the list object */
	INIT_LIST_HEAD(&listmp_sharedmemory_state.obj_list);
	/* Create a lock for protecting list object */
	listmp_sharedmemory_state.lock_handle = \
		kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (listmp_sharedmemory_state.lock_handle == NULL) {
		gt_2trace(listmpshm_debugmask,
				GT_4CLASS,
				"ListMPSharedmemsetup",
				LISTMPSHAREDMEMORY_E_FAIL,
				"Failed to create the lock_handle!");
		status = -ENOMEM;
		goto clean_nameserver;
	}

	mutex_init(listmp_sharedmemory_state.lock_handle);
	memset(&listmp_sharedmemory_state.cfg, 0, sizeof(struct listmp_config));
	goto exit;

clean_nameserver:
	status1 = nameserver_delete
		(&(listmp_sharedmemory_state.ns_handle));
	BUG_ON(status1 < 0);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "ListMPSharedmemsetup",
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

	gt_0trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_destroy");

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}

	/* Check if any listmp_sharedmemory instances have not been
	 * deleted so far. If not, delete them. */
	list_for_each(elem, &listmp_sharedmemory_state.obj_list) {
		if (((struct listmp_sharedmemory_obj *) elem)->owner
			->proc_id == multiproc_get_id(NULL)) {
			status1 = listmp_sharedmemory_delete(
			(listmp_sharedmemory_handle *)
			&(((struct listmp_sharedmemory_obj *) elem)->top));
			WARN_ON(status1 < 0);
		} else {
			status1 = listmp_sharedmemory_close(
			(listmp_sharedmemory_handle)
			(((struct listmp_sharedmemory_obj *) elem)->top));
			WARN_ON(status1 < 0);
		}
	}

	/* Delete the nameserver for modules */
	status = nameserver_delete(
			&(listmp_sharedmemory_state.ns_handle));
	BUG_ON(status < 0);

	/* Destruct the list object */
	list_del(&listmp_sharedmemory_state.obj_list);
	/* Delete the list lock */
	kfree(listmp_sharedmemory_state.lock_handle);
	listmp_sharedmemory_state.lock_handle = NULL;
	memset(&listmp_sharedmemory_state.cfg, 0, sizeof(struct listmp_config));

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_destroy",
		status);
	return status;
}

/*!
 *  @brief	Initialize this config-params structure with supplier-specified
 *		defaults before instance creation.
 *
 *  @param	handle Instance specific handle.
 *  @param	params Instance creation structure.
 *
 *  @sa	 listmp_sharedmemory_create
 */
void listmp_sharedmemory_params_init(listmp_sharedmemory_handle handle,
				listmp_sharedmemory_params *params)
{
	int status = 0;
	listmp_sharedmemory_object *object = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;

	gt_2trace(listmpshm_debugmask, GT_ENTER,
		"listmp_sharedmemory_params_init", handle, params);

	BUG_ON(params == NULL);
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (handle == NULL) {
		memcpy(params,
		&(listmp_sharedmemory_state.default_inst_params),
		sizeof(listmp_sharedmemory_params));
	} else {
		object = (listmp_sharedmemory_object *) handle;
		obj = (struct listmp_sharedmemory_obj *) object->obj;
		memcpy((void *)&obj->params,
			(void *)params,
			 sizeof(listmp_sharedmemory_params));
	}

exit:
	gt_0trace(listmpshm_debugmask, GT_LEAVE,
		"listmp_sharedmemory_params_init");
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
	struct listmp_sharedmemory_obj	*obj = NULL;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_create",
		params);

	BUG_ON(params == NULL);
	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON((params->name == NULL) && (params->shared_addr == NULL))) {
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

	obj = (struct listmp_sharedmemory_obj *)handle->obj;
	obj->listmp_elem->next = (struct listmp_elem *) (sharedregion_get_srptr(
				(void *)obj->listmp_elem, obj->index));
	obj->listmp_elem->prev = (struct listmp_elem *) (sharedregion_get_srptr(
				(void *)obj->listmp_elem, obj->index));

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_create",
		handle);
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
	u32	  key;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_delete",
		listmp_handleptr);

	BUG_ON(listmp_handleptr == NULL);
	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
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

	if (obj->owner->proc_id == multiproc_get_id(NULL)) {
		/* Remove from  the local list */
		key = mutex_lock_interruptible(
				listmp_sharedmemory_state.lock_handle);
		list_del(&obj->list_elem);
		mutex_unlock(listmp_sharedmemory_state.lock_handle);

		if ((obj->owner->open_count == 1)
		&& (obj->remote->open_count == 0)) {
			/* Remove from the name server */
			if (params->name != NULL) {
				nameserver_remove(
					listmp_sharedmemory_state.ns_handle,
					params->name);

				/* Free memory for the name */
				kfree(params->name);
			}

			/* Free memory for the processor info's */
			kfree(obj->owner);
			kfree(obj->remote);

			/* Now free the handle */
			kfree(obj);
			obj = NULL;

			/* Now free the handle */
			kfree(handle);
			handle = NULL;
		} else if (obj->owner->open_count > 1) {
			gt_2trace(listmpshm_debugmask,
					GT_4CLASS,
					"ListMP_delete",
					status,
					"Unmatched open/close calls!");
			status = -EBUSY;
		} else {
			gt_2trace(listmpshm_debugmask,
					GT_4CLASS,
					"ListMP_delete",
					status,
					"Opener of the instance has not closed"
					" the instance!");
			status = -EBUSY;
		}
	} else {
		/*! @retval LISTMPSHAREDMEMORY_E_NOTONWER
		* Instance was not created on this processor
		*/
		status = LISTMPSHAREDMEMORY_E_NOTOWNER;
		gt_2trace(listmpshm_debugmask,
			GT_4CLASS,
			"ListMP_delete",
			status,
			"Instance was not created on this processor!");
	}

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE,
		"listmp_sharedmemory_delete", status);
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

	gt_1trace(listmpshm_debugmask, GT_ENTER,
			"listmp_sharedmemory_sharedMemReq", params);

	BUG_ON(params == NULL);
	if (WARN_ON(params == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if (params->list_type == listmp_type_SHARED) {
		if (params != NULL)
			retval = (LISTMP_SHAREDMEMORY_CACHESIZE * 2);
	}

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE,
			"listmp_sharedmemory_sharedMemReq", retVal);
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
	u32 key;

	gt_1trace(listmpshm_debugmask, GT_ENTER,
			"listmp_sharedmemory_open", params);

	BUG_ON(listmp_handleptr == NULL);
	BUG_ON(params == NULL);
	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
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
	if (WARN_ON((params->name == NULL)
		&& (params->shared_addr == (u32)NULL))) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params->shared_addr_size
		 < listmp_sharedmemory_shared_memreq(params))) {
		status = -EINVAL;
		goto exit;
	}

	/* First check in the local list */
	list_for_each(elem, &listmp_sharedmemory_state.obj_list) {
		if (params->name != NULL) {
			if (strcmp(((struct listmp_sharedmemory_obj *)elem)
				->params.name, params->name) == 0) {
				key = mutex_lock_interruptible(
					listmp_sharedmemory_state.lock_handle);
				if (((struct listmp_sharedmemory_obj *)elem)
					->owner->proc_id
					== multiproc_get_id(NULL))
					((struct listmp_sharedmemory_obj *)elem)
						->owner->open_count++;
				else
					((struct listmp_sharedmemory_obj *)elem)
					->remote->open_count++;
				mutex_unlock(
					listmp_sharedmemory_state.lock_handle);
				*listmp_handleptr = \
					(((struct listmp_sharedmemory_obj *)
					elem)->top);
				done_flag = true;
				break;
			}
		} else {
			if (((struct listmp_sharedmemory_obj *)elem)
				->params.shared_addr == params->shared_addr) {
				key = mutex_lock_interruptible(
					listmp_sharedmemory_state.lock_handle);
				if (((struct listmp_sharedmemory_obj *)elem)
					->owner->proc_id == \
					multiproc_get_id(NULL))
					((struct listmp_sharedmemory_obj *)elem)
						->owner->open_count++;
				else
					((struct listmp_sharedmemory_obj *)elem)
						->remote->open_count++;

				mutex_unlock(
					listmp_sharedmemory_state.lock_handle);
				*listmp_handleptr = (
					((struct listmp_sharedmemory_obj *)elem)
					->top);
				done_flag = true;
				break;
			}
		}
	}

	/* Not found in local list */
	if (done_flag == false) {
		*listmp_handleptr = (listmp_sharedmemory_handle)
				_listmp_sharedmemory_create(params, false);
	}

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_open",
		status);
	/*! @retval LISTMPSHAREDMEMORY_SUCCESS Operation Successful*/
	return status;
}

/*
 * ======== listmp_sharedmemory_close ========
 *  Purpose:
 *  Function to close a previously opened instance
 */
int listmp_sharedmemory_close(listmp_sharedmemory_handle  listMPHandle)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	listmp_sharedmemory_params *params = NULL;
	u32		key;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_close",
			listMPHandle);

	BUG_ON(listMPHandle == NULL);
	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (listMPHandle == NULL) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	key = mutex_lock_interruptible(
		listmp_sharedmemory_state.lock_handle);
	if (obj->owner->proc_id == multiproc_get_id(NULL))
		(obj)->owner->open_count--;
	else
		(obj)->remote->open_count--;
	mutex_unlock(listmp_sharedmemory_state.lock_handle);

	params = (listmp_sharedmemory_params *) &obj->params;

	key = mutex_lock_interruptible(
		listmp_sharedmemory_state.lock_handle);
	if ((((struct listmp_sharedmemory_obj *)obj)->remote->proc_id == \
		multiproc_get_id(NULL)) && \
		(((struct listmp_sharedmemory_obj *)obj)
			->remote->open_count == 0)) {
		list_del(&obj->list_elem);

		/* remove from the name server */
		if (params->name != NULL)
			nameserver_remove(
				listmp_sharedmemory_state.ns_handle,
				params->name);

		mutex_unlock(listmp_sharedmemory_state.lock_handle);

		/* remove from the name server */
		if (params->name != NULL)
			/* Free memory for the name */
			kfree(params->name);

		/* Now free the obj */
		kfree(obj->params.name);
		kfree(obj->owner);
		kfree(obj->remote);
		kfree(obj);
		obj = NULL;
		kfree(handle);
		handle = NULL;
	} else {
		mutex_unlock(listmp_sharedmemory_state.lock_handle);
	}

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_close",
		status);
	return status;
}

/*
 * ======== listmp_sharedmemory_empty ========
 *  Purpose:
 *  Function to check if the shared memory list is empty
 */
bool listmp_sharedmemory_empty(listmp_sharedmemory_handle listMPHandle)
{

	int status = 0;
	bool isEmpty = false;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	u32 key = 0;
	struct listmp_elem *sharedHead;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_empty",
			listMPHandle);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(listMPHandle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	/*! @retval true if list is empty */
	sharedHead = (struct listmp_elem *)(sharedregion_get_srptr(
				(void *)obj->listmp_elem, obj->index));

	if (obj->listmp_elem->next == sharedHead)
		isEmpty = true;

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_empty",
			isEmpty);
	return isEmpty;
}

/*
 * ======== listmp_sharedmemory_get_head ========
 *  Purpose:
 *  Function to get head element from a shared memory list
 */
void *listmp_sharedmemory_get_head(listmp_sharedmemory_handle listMPHandle)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *elem = NULL;
	struct listmp_elem *localHeadNext = NULL;
	struct listmp_elem *localNext = NULL;
	struct listmp_elem *sharedHead = NULL;
	u32 key = 0;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_get_head",
			listMPHandle);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		/*! @retval NULL if Module was not initialized */
		elem = NULL;
		goto exit;
	}
	if (WARN_ON(listMPHandle == NULL)) {
		/*! @retval  NULL if listMPHandle passed is NULL */
		elem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	localHeadNext = sharedregion_get_ptr((u32 *)obj->listmp_elem->next);
	/* See if the listmp_sharedmemory_object was empty */
	if (localHeadNext == (struct listmp_elem *)obj->listmp_elem) {
		/*! @retval NULL if list is empty */
		elem = NULL ;
	} else {
		/* Elem to return */
		elem = localHeadNext;

		localNext = sharedregion_get_ptr((u32 *)elem->next);
		sharedHead = (struct listmp_elem *) sharedregion_get_srptr(
				obj->listmp_elem, obj->index);

		/* Fix the head of the list next pointer */
		obj->listmp_elem->next = elem->next;

		/* Fix the prev pointer of the new first elem on the
		list */
		localNext->prev = sharedHead;
	}

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_get_head",
			elem);
	return elem;
}

/*
 * ======== listmp_sharedmemory_get_tail ========
 *  Purpose:
 *  Function to get tail element from a shared memory list
 */
void *listmp_sharedmemory_get_tail(listmp_sharedmemory_handle listMPHandle)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *elem = NULL;
	struct listmp_elem *localHeadNext = NULL;
	struct listmp_elem *localHeadPrev = NULL;
	struct listmp_elem *localPrev = NULL;
	struct listmp_elem *sharedHead = NULL;
	u32 key = 0;

	gt_1trace(listmpshm_debugmask, GT_ENTER,
			"listmp_sharedmemory_get_tail", listMPHandle);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		/*! @retval NULL if Module was not initialized */
		elem = NULL;
		goto exit;
	}
	if (WARN_ON(listMPHandle == NULL)) {
		/*! @retval NULL if listMPHandle passed is NULL */
		elem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

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
					obj->listmp_elem, 	obj->index);
		obj->listmp_elem->prev = elem->prev;
		localPrev->next = sharedHead;
	}

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_get_tail",
		elem);
	return elem;
}

/*
 * ======== listmp_sharedmemory_put_head ========
 *  Purpose:
 *  Function to put head element into a shared memory list
 */
int listmp_sharedmemory_put_head(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localNextElem = NULL;
	struct listmp_elem *sharedElem = NULL;
	struct listmp_elem *sharedHead = NULL;
	u32 key = 0;
	u32 index;

	gt_1trace(listmpshm_debugmask, GT_ENTER,
			"listmp_sharedmemory_put_head", listMPHandle);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(listMPHandle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	index = sharedregion_get_index(elem);
	sharedElem = (struct listmp_elem *) sharedregion_get_srptr(elem, index);
	sharedHead = (struct listmp_elem *)sharedregion_get_srptr(
					(void *)obj->listmp_elem, obj->index);

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	/* Add the new elem into the list */
	elem->next = obj->listmp_elem->next;
	elem->prev = sharedHead;
	localNextElem = sharedregion_get_ptr((u32 *)elem->next);
	localNextElem->prev = sharedElem;
	obj->listmp_elem->next = sharedElem;

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_put_head",
			status);
	/*! @retval LISTMPSHAREDMEMORY_SUCCESS Operation Successful*/
	return status;
}

/*
 * ======== listmp_sharedmemory_put_tail ========
 *  Purpose:
 *  Function to put tail element into a shared memory list
 */
int listmp_sharedmemory_put_tail(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *sharedElem = NULL;
	struct listmp_elem *sharedHead = NULL;
	u32 key = 0;
	u32 index;

	gt_2trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_put_tail",
					listMPHandle, elem);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	/* Safe to do outside the gate */
	index = sharedregion_get_index(elem);
	sharedElem = (struct listmp_elem *)
			sharedregion_get_srptr(elem, index);
	sharedHead = (struct listmp_elem *)sharedregion_get_srptr
				((void *)obj->listmp_elem,
				obj->index);
	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	/* Add the new elem into the list */
	elem->next = sharedHead;
	elem->prev = obj->listmp_elem->prev;
	localPrevElem = sharedregion_get_ptr((u32 *)elem->prev);
	localPrevElem->next = sharedElem;
	obj->listmp_elem->prev = sharedElem;

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_put_tail",
		status);
	return status;
}

/*
 * ======== listmp_sharedmemory_insert ========
 *  Purpose:
 *  Function to insert an element into a shared memory list
 */
int listmp_sharedmemory_insert(listmp_sharedmemory_handle  listMPHandle,
				struct listmp_elem *new_elem,
				struct listmp_elem *cur_elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *sharedNewElem = NULL;
	struct listmp_elem *sharedCurElem = NULL;
	u32 key = 0;
	u32 index;

	gt_3trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_insert",
				listMPHandle, new_elem, cur_elem);


	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(new_elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	/* If NULL change cur_elem to the obj */
	if (cur_elem == NULL)
		cur_elem = (struct listmp_elem *)obj->listmp_elem->next;

	/* Get SRPtr for new_elem */
	index = sharedregion_get_index(new_elem);
	sharedNewElem = (struct listmp_elem *)
				sharedregion_get_srptr(new_elem, index);

	/* Get SRPtr for cur_elem */
	index = sharedregion_get_index(cur_elem);
	sharedCurElem = (struct listmp_elem *)
				sharedregion_get_srptr(cur_elem, index);

	/* Get SRPtr for cur_elem->prev */
	localPrevElem = sharedregion_get_ptr((u32 *)cur_elem->prev);

	new_elem->next = sharedCurElem;
	new_elem->prev = cur_elem->prev;
	localPrevElem->next = sharedNewElem;
	cur_elem->prev = sharedNewElem;

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_insert",
		status);
	return status;
}

/*
 * ======== listmp_sharedmemory_remove ========
 *  Purpose:
 *  Function to remove a element from a shared memory list
 */
int listmp_sharedmemory_remove(listmp_sharedmemory_handle  listMPHandle,
				struct listmp_elem *elem)
{
	int status = 0;
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj	*obj = NULL;
	struct listmp_elem *localPrevElem = NULL;
	struct listmp_elem *localNextElem = NULL;
	u32 key = 0;

	gt_2trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_remove",
					listMPHandle,
					elem);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	localPrevElem = sharedregion_get_ptr((u32 *)elem->prev);
	localNextElem = sharedregion_get_ptr((u32 *)elem->next);

	localPrevElem->next = elem->next;
	localNextElem->prev = elem->prev;

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_remove",
		status);
	return status;
}

/*
 * ======== listmp_sharedmemory_next ========
 *  Purpose:
 *  Function to traverse to next element in shared memory list
 */
void *listmp_sharedmemory_next(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj *obj = NULL;
	struct listmp_elem *retElem = NULL;
	struct listmp_elem *sharedNextElem = NULL;
	u32 key = 0;


	gt_2trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_next",
				   listMPHandle,
				   elem);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		/*! @retval NULL if Module was not initialized */
		retElem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

	/* If element is NULL start at head */
	if (elem == NULL)
		sharedNextElem = (struct listmp_elem *) obj->listmp_elem->next;
	else
		sharedNextElem = (struct listmp_elem *)elem->next;

	retElem = sharedregion_get_ptr((u32 *)sharedNextElem);

	/*! @retval NULL if list is empty */
	if (retElem == (struct listmp_elem *)obj->listmp_elem)
		retElem = NULL;
	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_next",
			retElem);
	return retElem;
}

/*
 * ======== listmp_sharedmemory_prev ========
 *  Purpose:
 *  Function to traverse to prev element in shared memory list
 */
void *listmp_sharedmemory_prev(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem)
{
	listmp_sharedmemory_object *handle = NULL;
	struct listmp_sharedmemory_obj	*obj = NULL;
	struct listmp_elem *retElem = NULL;
	struct listmp_elem *sharedPrevElem = NULL;
	u32 key = 0;

	gt_2trace(listmpshm_debugmask, GT_ENTER, "listmp_sharedmemory_prev",
				   listMPHandle,
				   elem);

	if (WARN_ON(listmp_sharedmemory_state.ns_handle == NULL)) {
		/*! @retval NULL if Module was not initialized */
		retElem = NULL;
		goto exit;
	}

	handle = (listmp_sharedmemory_object *)listMPHandle;
	obj = (struct listmp_sharedmemory_obj *) handle->obj;

	if (obj->params.lock_handle != NULL)
		key = mutex_lock_interruptible(obj->params.lock_handle);

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

	if (obj->params.lock_handle != NULL)
		mutex_unlock(obj->params.lock_handle);

exit:
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "listmp_sharedmemory_prev",
			retElem);
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
	u32 shmIndex;
	u32 *sharedShmBase;

	gt_1trace(listmpshm_debugmask, GT_ENTER, "_listmp_sharedmemory_create",
			params);

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
	obj->attrs = (struct listmp_attrs *)
					params->shared_addr;
	obj->attrs->shared_addr_size = params->shared_addr_size;
	obj->attrs->version = LISTMP_SHAREDMEMORY_VERSION;
	obj->index = sharedregion_get_index(params->shared_addr);
	/* Assign the memory with proper cache line padding */
	obj->listmp_elem = (void *) ((u32)obj->attrs + \
			LISTMP_SHAREDMEMORY_CACHESIZE);

	/* Populate the params member */
	memcpy((void *)&obj->params,
		(void *)params,
		 sizeof(listmp_sharedmemory_params));

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

	if (params->heap_handle != NULL)
		obj->params.heap_handle = params->heap_handle;

	/* Update processor information */
	obj->owner = kmalloc(sizeof(struct listmp_proc_attrs),
							GFP_KERNEL);
	obj->remote = kmalloc(sizeof(struct listmp_proc_attrs),
							GFP_KERNEL);

	if ((obj->owner == NULL) || (obj->remote == NULL)) {
		gt_2trace(listmpshm_debugmask,
					GT_4CLASS,
					"_listmp_sharedmemory_create",
					status,
					"Memory allocation failed"
					"for processor information!");
		status = -ENOMEM;
	} else {

		/* Update owner and opener details */
		if (create_flag == true) {
			obj->remote->creator = false;
			obj->remote->open_count = 0;
			obj->remote->proc_id = MULTIPROC_INVALIDID;
			obj->owner->creator = true;
			obj->owner->open_count = 1;
			obj->owner->proc_id = multiproc_get_id(NULL);
			obj->top = handle;
			obj->attrs->status = \
				LISTMP_SHAREDMEMORY_CREATED;
		} else {
			obj->remote->creator = true;
			obj->remote->open_count = 1;
			obj->remote->proc_id = MULTIPROC_INVALIDID;
			obj->owner->creator = false;
			obj->owner->open_count = 0;
			obj->owner->proc_id = multiproc_get_id("DSP");
			/* TBD */
			obj->top = handle;
		}

		/* Put in the local list */
		key = mutex_lock_interruptible(
			listmp_sharedmemory_state.lock_handle);
		INIT_LIST_HEAD(&obj->list_elem);
		list_add_tail((&obj->list_elem),
			&listmp_sharedmemory_state.obj_list);
		mutex_unlock(listmp_sharedmemory_state.lock_handle);

		/* We will store a shared pointer in the
		 * NameServer
		 */
		shmIndex = sharedregion_get_index(params->shared_addr);
		sharedShmBase = sharedregion_get_srptr(
					params->shared_addr, shmIndex);
		/* Add list instance to name server */
		if (obj->params.name != NULL) {
			nameserver_add_uint32(
				listmp_sharedmemory_state.ns_handle,
				params->name,
				(u32) (params->shared_addr));
		}
	}

	if (status < 0) {
		/* Remove from  the local list */
		key = mutex_lock_interruptible(
					listmp_sharedmemory_state.lock_handle);
		list_del(&obj->list_elem);
		mutex_unlock(listmp_sharedmemory_state.lock_handle);

		if (obj->params.name != NULL)
			nameserver_remove(listmp_sharedmemory_state.ns_handle,
				params->name);
		if (obj->owner != NULL)
			kfree(obj->owner);
		if (obj->remote != NULL)
			kfree(obj->remote);
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
	gt_1trace(listmpshm_debugmask, GT_LEAVE, "_listmp_sharedmemory_create",
		handle);
	return (listmp_sharedmemory_handle) handle;
}
