/*
 *  listmp_sharedmemory_ioctl.c
 *
 *  This file implements all the ioctl operations required on the
 *  listmp_sharedmemory module.
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
/* Standard headers */
#include <linux/types.h>

/* Linux headers */
#include <linux/uaccess.h>
#include <linux/bug.h>
#include <linux/fs.h>
#include <linux/mm.h>

/* Module Headers */
#include <listmp.h>
#include <listmp_sharedmemory.h>
#include <listmp_sharedmemory_ioctl.h>
#include <sharedregion.h>

/*
 * ======== listmp_sharedmemory_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_get_config function
 */
static inline int listmp_sharedmemory_ioctl_get_config(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	struct listmp_config config;

	status = listmp_sharedmemory_get_config(&config);
	if (unlikely(status))
		goto exit;

	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct listmp_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_setup function
 */
static inline int listmp_sharedmemory_ioctl_setup(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	struct listmp_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
				sizeof(struct listmp_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	status = listmp_sharedmemory_setup(&config);
	cargs->api_status = status;
	if (unlikely(status))
		goto exit;
exit:
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_destroy function
 */
static inline int listmp_sharedmemory_ioctl_destroy(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	cargs->api_status = listmp_sharedmemory_destroy();
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_params_init function
 */
static inline int listmp_sharedmemory_ioctl_params_init(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	listmp_sharedmemory_params params;

	size = copy_from_user(&params,
				cargs->args.params_init.params,
				sizeof(listmp_sharedmemory_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	listmp_sharedmemory_params_init(
		cargs->args.params_init.listmp_handle, &params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(listmp_sharedmemory_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

exit:
	cargs->api_status = status;
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_create function
 */
static inline int listmp_sharedmemory_ioctl_create(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	listmp_sharedmemory_params params;

	size = copy_from_user(&params, cargs->args.create.params,
					sizeof(listmp_sharedmemory_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	/* Allocate memory for the name */
	if (cargs->args.create.name_len > 0) {
		params.name = kmalloc(cargs->args.create.name_len, GFP_KERNEL);
		if (params.name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		/* Copy the name */
		size = copy_from_user(params.name,
					cargs->args.create.params->name,
					cargs->args.create.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.create.shared_addr_srptr);
	if (unlikely(params.shared_addr == NULL))
		goto free_name;

	/* Update gate in params. */
	params.gate = cargs->args.create.knl_gate;
	cargs->args.create.listmp_handle = listmp_sharedmemory_create(&params);

	size = copy_to_user(cargs->args.create.params, &params,
				sizeof(listmp_sharedmemory_params));
	if (!size)
		goto free_name;

	/* Error copying, so delete the handle */
	retval = -EFAULT;
	if (cargs->args.create.listmp_handle)
		listmp_sharedmemory_delete(&cargs->args.create.listmp_handle);

free_name:
	if (cargs->args.create.name_len > 0)
		kfree(params.name);

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_delete function
 */
static inline int listmp_sharedmemory_ioctl_delete(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	cargs->api_status = listmp_sharedmemory_delete(
		&(cargs->args.delete_listmp.listmp_handle));
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_open function
 */
static inline int listmp_sharedmemory_ioctl_open(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	listmp_sharedmemory_params params;
	void *listmp_handle = NULL;

	if (unlikely(cargs->args.open.params == NULL)) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->args.open.params,
				sizeof(listmp_sharedmemory_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if ((cargs->args.open.name_len > 0) &&
			(cargs->args.open.params->name != NULL)) {
		params.name = kmalloc(cargs->args.open.name_len, GFP_KERNEL);
		if (params.name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		/* Copy the name */
		size = copy_from_user(params.name,
					cargs->args.open.params->name,
					cargs->args.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	/* For open by name, the shared_add_srptr may be invalid */
	if (cargs->args.open.shared_addr_srptr != \
		(u32)SHAREDREGION_INVALIDSRPTR) {
		params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.open.shared_addr_srptr);
	}
	if (unlikely(params.shared_addr == NULL))
		goto free_name;

	/* Update gate in params. */
	params.gate = cargs->args.open.knl_gate;
	status = listmp_sharedmemory_open(&listmp_handle, &params);
	if (status < 0)
		goto free_name;
	cargs->args.open.listmp_handle = listmp_handle;

	size = copy_to_user(cargs->args.open.params, &params,
				sizeof(listmp_sharedmemory_params));
	if (!size)
		goto free_name;

	/* Error copying, so close the handle */
	retval = -EFAULT;
	if (cargs->args.open.listmp_handle) {
		listmp_sharedmemory_close(cargs->args.open.listmp_handle);
		cargs->args.open.listmp_handle = NULL;
	}

free_name:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

	cargs->api_status = status;
exit:
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_close function
 */
static inline int listmp_sharedmemory_ioctl_close(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	cargs->api_status = \
		listmp_sharedmemory_close(cargs->args.close.listmp_handle);
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_is_empty ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_empty function
 */
static inline int listmp_sharedmemory_ioctl_empty(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	cargs->args.is_empty.is_empty = \
		listmp_sharedmemory_empty(cargs->args.is_empty.listmp_handle);
	cargs->api_status = 0;
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_get_head ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_get_head function
 */
static inline int listmp_sharedmemory_ioctl_get_head(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem;
	u32 *elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	cargs->api_status = LISTMPSHAREDMEMORY_E_FAIL;

	elem = listmp_sharedmemory_get_head(cargs->args.get_head.listmp_handle);
	if (unlikely(elem == NULL))
		goto exit;

	index = sharedregion_get_index(elem);
	if (unlikely(index < 0))
		goto exit;

	elem_srptr = sharedregion_get_srptr((void *)elem, index);

	cargs->api_status = 0;
exit:
	cargs->args.get_head.elem_srptr = elem_srptr;
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_get_tail ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_get_tail function
 */
static inline int listmp_sharedmemory_ioctl_get_tail(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem;
	u32 *elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	cargs->api_status = LISTMPSHAREDMEMORY_E_FAIL;

	elem = listmp_sharedmemory_get_tail(cargs->args.get_tail.listmp_handle);
	if (unlikely(elem == NULL))
		goto exit;

	index = sharedregion_get_index(elem);
	if (unlikely(index < 0))
		goto exit;

	elem_srptr = sharedregion_get_srptr((void *)elem, index);

	cargs->api_status = 0;
exit:
	cargs->args.get_tail.elem_srptr = elem_srptr;
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_put_head ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_put_head function
 */
static inline int listmp_sharedmemory_ioctl_put_head(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem;

	cargs->api_status = LISTMPSHAREDMEMORY_E_FAIL;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.put_head.elem_srptr);
	if (unlikely(elem == NULL))
		goto exit;

	cargs->api_status = listmp_sharedmemory_put_head(
		cargs->args.put_head.listmp_handle, elem);
exit:
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_put_tail ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_put_tail function
 */
static inline int listmp_sharedmemory_ioctl_put_tail(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem;

	cargs->api_status = LISTMPSHAREDMEMORY_E_FAIL;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.put_tail.elem_srptr);
	if (unlikely(elem == NULL))
		goto exit;

	cargs->api_status = listmp_sharedmemory_put_tail(
		cargs->args.put_head.listmp_handle, elem);
exit:
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_insert ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_insert function
 */
static inline int listmp_sharedmemory_ioctl_insert(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *new_elem;
	struct listmp_elem *cur_elem;

	new_elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.insert.new_elem_srptr);
	if (unlikely(new_elem == NULL))
		goto exit;

	cur_elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.insert.cur_elem_srptr);
	if (unlikely(cur_elem == NULL))
		goto exit;

	cargs->api_status = listmp_sharedmemory_insert(
		cargs->args.insert.listmp_handle, new_elem, cur_elem);
exit:
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_remove ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_remove function
 */
static inline int listmp_sharedmemory_ioctl_remove(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.remove.elem_srptr);
	if (unlikely(elem == NULL))
		goto exit;

	cargs->api_status = listmp_sharedmemory_remove(
				cargs->args.get_head.listmp_handle, elem);

exit:
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_next ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_next function
 */
static inline int listmp_sharedmemory_ioctl_next(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem = NULL;
	struct listmp_elem *ret_elem = NULL;
	u32 *next_elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	if (cargs->args.next.elem_srptr != NULL) {
		elem = (struct listmp_elem *) sharedregion_get_ptr(
						cargs->args.next.elem_srptr);
	}
	ret_elem = (struct listmp_elem *) listmp_sharedmemory_next(
					cargs->args.next.listmp_handle, elem);
	if (unlikely(ret_elem == NULL))
		goto exit;

	index = sharedregion_get_index(ret_elem);
	if (unlikely(index < 0))
		goto exit;

	next_elem_srptr = sharedregion_get_srptr((void *)ret_elem, index);

	cargs->api_status = 0;
exit:
	cargs->args.next.next_elem_srptr = next_elem_srptr;
	return 0;
}

/*
 * ======== listmp_sharedmemory_ioctl_prev ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_prev function
 */
static inline int listmp_sharedmemory_ioctl_prev(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	struct listmp_elem *elem = NULL;
	struct listmp_elem *ret_elem = NULL;
	u32 *prev_elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	if (cargs->args.next.elem_srptr != NULL) {
		elem = (struct listmp_elem *) sharedregion_get_ptr(
						cargs->args.prev.elem_srptr);
	}
	ret_elem = (struct listmp_elem *) listmp_sharedmemory_prev(
					cargs->args.prev.listmp_handle, elem);
	if (unlikely(ret_elem == NULL))
		goto exit;

	index = sharedregion_get_index(ret_elem);
	if (unlikely(index < 0))
		goto exit;

	prev_elem_srptr = sharedregion_get_srptr((void *)ret_elem, index);

	cargs->api_status = 0;
exit:
	cargs->args.prev.prev_elem_srptr = prev_elem_srptr;
	return 0;

}

/*
 * ======== listmp_sharedmemory_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to listmp_sharedmemory_shared_memreq function
 */
static inline int listmp_sharedmemory_ioctl_shared_memreq(
				struct listmp_sharedmemory_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	listmp_sharedmemory_params params;

	size = copy_from_user(&params, cargs->args.shared_memreq.params,
				sizeof(listmp_sharedmemory_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->args.shared_memreq.bytes =
		listmp_sharedmemory_shared_memreq(&params);

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== listmp_sharedmemory_ioctl ========
 *  Purpose:
 *  ioctl interface function for listmp_sharedmemory module
 */
int listmp_sharedmemory_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct listmp_sharedmemory_cmd_args __user *uarg =
			(struct listmp_sharedmemory_cmd_args __user *)args;
	struct listmp_sharedmemory_cmd_args cargs;
	unsigned long size;

	if (_IOC_DIR(cmd) & _IOC_READ)
		os_status = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		os_status = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));
	if (os_status) {
		os_status = -EFAULT;
		goto exit;
	}

	/* Copy the full args from user-side */
	size = copy_from_user(&cargs, uarg,
				sizeof(struct listmp_sharedmemory_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_LISTMP_SHAREDMEMORY_GETCONFIG:
		os_status = listmp_sharedmemory_ioctl_get_config(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_SETUP:
		os_status = listmp_sharedmemory_ioctl_setup(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_DESTROY:
		os_status = listmp_sharedmemory_ioctl_destroy(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_PARAMS_INIT:
		os_status = listmp_sharedmemory_ioctl_params_init(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_CREATE:
		os_status = listmp_sharedmemory_ioctl_create(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_DELETE:
		os_status = listmp_sharedmemory_ioctl_delete(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_OPEN:
		os_status = listmp_sharedmemory_ioctl_open(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_CLOSE:
		os_status = listmp_sharedmemory_ioctl_close(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_ISEMPTY:
		os_status = listmp_sharedmemory_ioctl_empty(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_GETHEAD:
		os_status = listmp_sharedmemory_ioctl_get_head(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_GETTAIL:
		os_status = listmp_sharedmemory_ioctl_get_tail(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_PUTHEAD:
		os_status = listmp_sharedmemory_ioctl_put_head(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_PUTTAIL:
		os_status = listmp_sharedmemory_ioctl_put_tail(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_INSERT:
		os_status = listmp_sharedmemory_ioctl_insert(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_REMOVE:
		os_status = listmp_sharedmemory_ioctl_remove(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_NEXT:
		os_status = listmp_sharedmemory_ioctl_next(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_PREV:
		os_status = listmp_sharedmemory_ioctl_prev(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMORY_SHAREDMEMREQ:
		os_status = listmp_sharedmemory_ioctl_shared_memreq(&cargs);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
	}
	if (os_status < 0)
		goto exit;

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs,
				sizeof(struct listmp_sharedmemory_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}
	return os_status;

exit:
	printk(KERN_ERR "listmp_sharedmemory_ioctl failed: status = 0x%x\n",
		os_status);
	return os_status;
}
