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
 *
 *  This module implements the ListMP.
 *  This module is instance based. Each instance requires a small
 *  piece of shared memory. This is specified via the #shared_addr
 *  parameter to the create. The proper #shared_addr_size parameter
 *  can be determined via the #sharedMemReq call. Note: the
 *  parameters to this function must be the same that will used to
 *  create or open the instance.
 *  The ListMP module uses a #ti.sdo.utils.NameServer instance
 *  to store instance information when an instance is created and
 *  the name parameter is non-NULL. If a name is supplied, it must
 *  be unique for all ListMP instances.
 *  The #create also initializes the shared memory as needed. The
 *  shared memory must be initialized to 0 or all ones
 *  (e.g. 0xFFFFFFFFF) before the ListMP instance is created.
 *  Once an instance is created, an open can be performed. The #open
 *  is used to gain access to the same ListMP instance.
 *  Generally an instance is created on one processor and opened
 *  on the other processor.
 *  The open returns a ListMP instance handle like the create,
 *  however the open does not modify the shared memory. Generally an
 *  instance is created on one processor and opened on the other
 *  processor.
 *  There are two options when opening the instance:
 *  @li Supply the same name as specified in the create. The
 *  ListMP module queries the NameServer to get the needed
 *  information.
 *  @li Supply the same #shared_addr value as specified in the
 *  create. If the open is called before the instance is created,
 *  open returns NULL.
 *  There is currently a list of restrictions for the module:
 *  @li Both processors must have the same endianness. Endianness
 *  conversion may supported in a future version of ListMP.
 *  @li The module will be made a gated module
 */


/* Standard headers */
#include <linux/types.h>
#include <linux/module.h>

/* Utilities headers */
#include <linux/string.h>
#include <linux/list.h>

/* Module level headers */
#include <nameserver.h>
#include <listmp.h>
#include <listmp_sharedmemory.h>


/*
 * ======== listmp_params_init ========
 *  Purpose:
 *  Function initializes listmp parameters
 */
void listmp_params_init(void *listmp_handle, struct listmp_params *params)
{
	BUG_ON(params == NULL);
	listmp_sharedmemory_params_init(listmp_handle, params);
}

/*
 * ======== listmp_shared_memreq ========
 *  Purpose:
 *  Function to get shared memory requirement for the module
 */
int listmp_shared_memreq(struct listmp_params *params)
{
	int shared_memreq = 0;

	if (WARN_ON(params == NULL))
		goto exit;

	shared_memreq = listmp_sharedmemory_shared_memreq(params);
exit:
	return shared_memreq;
}

/*
 * ======== listmp_create ========
 *  Purpose:
 *  Creates a new instance of listmp_sharedmemory module.
 */
void *listmp_create(struct listmp_params *params)
{
	struct listmp_object *handle = NULL;

	if (WARN_ON(params == NULL))
		goto exit;

	if (params->list_type == listmp_type_SHARED)
		handle = (struct listmp_object *)
				listmp_sharedmemory_create(params);
	return (void *)handle;

exit:
	printk(KERN_ERR "listmp_create: listmp_params passed are NULL!\n");
	return NULL;
}

/*
 * ======== listmp_delete ========
 *  Purpose:
 *  Deletes an instance of listmp_sharedmemory module.
 */
int listmp_delete(void **listmp_handle_ptr)
{
	int status = 0;

	if (WARN_ON(*listmp_handle_ptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (((struct listmp_object *)(*listmp_handle_ptr))->list_type == \
		listmp_type_SHARED)
		status = listmp_sharedmemory_delete(listmp_handle_ptr);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_delete failed! status = 0x%x\n",
			status);
	}
	return status;
}

/*
 * ======== listmp_open ========
 *  Purpose:
 *  Opens an instance ofa previously created listmp_sharedmemory module.
 */
int listmp_open(void **listmp_handle_ptr, struct listmp_params *params)
{
	int  status = 0;

	if (WARN_ON(listmp_handle_ptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (params->list_type == listmp_type_SHARED)
		status = listmp_sharedmemory_open(listmp_handle_ptr, params);

exit:
	if (status < 0)
		printk(KERN_ERR "listmp_open failed! status = 0x%x\n", status);
	return status;
}

/*
 * ======== listmp_close ========
 *  Purpose:
 *  Closes an instance of a previously opened listmp_sharedmemory module.
 */
int listmp_close(void *listmp_handle)
{
	int status = 0;

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (((struct listmp_object *)listmp_handle)->list_type == \
		listmp_type_SHARED)
		status = listmp_sharedmemory_close(listmp_handle);

exit:
	if (status < 0)
		printk(KERN_ERR "listmp_close failed! status = 0x%x\n", status);
	return status;
}

/*
 * ======== listmp_empty ========
 *  Purpose:
 *  Function to check if list is empty
 */
bool listmp_empty(void *listmp_handle)
{
	bool isEmpty = false;
	struct listmp_object *handle = NULL;

	if (WARN_ON(listmp_handle == NULL))
		goto exit;

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->empty == NULL);
	isEmpty = handle->empty(listmp_handle);

exit:
	return isEmpty;
}

/*
 * ======== listmp_get_head ========
 *  Purpose:
 *  Function to get head element from a listmp_sharedmemory list
 */
void *listmp_get_head(void *listmp_handle)
{
	struct listmp_object *handle = NULL;
	struct listmp_elem *elem = NULL;

	if (WARN_ON(listmp_handle == NULL))
		goto exit;

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->get_head == NULL);
	elem = handle->get_head(listmp_handle);

exit:
	return elem;
}

/*
 * ======== listmp_get_tail =====
 *  Purpose:
 *  Function to get tailement from a listmp_sharedmemory list
 */
void *listmp_get_tail(void *listmp_handle)
{
	struct listmp_object *handle = NULL;
	struct listmp_elem *elem = NULL;

	if (WARN_ON(listmp_handle == NULL))
		goto exit;

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->get_tail == NULL);
	elem = handle->get_tail(listmp_handle);

exit:
	return elem;
}

/*
 * ======== listmp_put_head ========
 *  Purpose:
 *  Function to put head element into a listmp_sharedmemory list
 */
int listmp_put_head(void *listmp_handle, struct listmp_elem *elem)
{
	int status = 0;
	struct listmp_object *handle = NULL;

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (struct listmp_object *) listmp_handle;

	BUG_ON(handle->put_head == NULL);
	status = handle->put_head(listmp_handle, elem);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_put_head failed! status = 0x%x\n",
			status);
	}
	return status;
}

/*
 * ======== listmp_put_tail ========
 *  Purpose:
 *  Function to put tail element into a listmp_sharedmemory list
 */
int listmp_put_tail(void *listmp_handle, struct listmp_elem *elem)
{
	int status = 0;
	struct listmp_object *handle = NULL;

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->put_tail == NULL);
	status = handle->put_tail(listmp_handle, elem);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_put_tail failed! status = 0x%x\n",
			status);
	}
	return status;
}

/*
 * ======== listmp_insert ========
 *  Purpose:
 *  Function to insert element into a listmp_sharedmemory list
 */
int listmp_insert(void *listmp_handle, struct listmp_elem *elem,
			struct listmp_elem *curElem)
{
	int status = 0;
	struct listmp_object *handle = NULL;

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(curElem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->insert == NULL);
	status = handle->insert(listmp_handle, elem, curElem);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_insert failed! status = 0x%x\n",
			status);
	}
	return status;
}

/*
 * ======== listmp_remove ========
 *  Purpose:
 *  Function to remove an element from a listmp_sharedmemory list
 */
int listmp_remove(void *listmp_handle, struct listmp_elem *elem)
{
	int status = LISTMP_SUCCESS;
	struct listmp_object *handle = NULL;

	if (WARN_ON(listmp_handle == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(elem == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->remove == NULL);
	status = handle->remove(listmp_handle, elem);

exit:
	if (status < 0) {
		printk(KERN_ERR "listmp_remove failed! status = 0x%x\n",
			status);
	}
	return status ;
}

/*
 * ======== listmp_next ========
 *  Purpose:
 *  Function to return the next element from a listmp_sharedmemory list
 */
void *listmp_next(void *listmp_handle, struct listmp_elem *elem)
{
	struct listmp_object *handle = NULL;
	struct listmp_elem *nextElem = NULL;

	if (WARN_ON(listmp_handle == NULL))
		goto exit;

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->next == NULL);
	nextElem = handle->next(listmp_handle, elem);

exit:
	return nextElem;
}

/*
 * ======== listmp_next ========
 *  Purpose:
 *  Function to return the prev element from a listmp_sharedmemory list
 */
void *listmp_prev(void *listmp_handle, struct listmp_elem *elem)
{
	struct listmp_object *handle = NULL;
	struct listmp_elem *prevElem = NULL;

	if (WARN_ON(listmp_handle == NULL))
		goto exit;

	handle = (struct listmp_object *)listmp_handle;

	BUG_ON(handle->prev == NULL);
	prevElem = handle->prev(listmp_handle, elem);

exit:
	return prevElem;
}
