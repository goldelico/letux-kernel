/*
 *  listmp_ioctl.c
 *
 *  This file implements all the ioctl operations required on the
 *  listmp module.
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
#include <_listmp.h>
#include <listmp_ioctl.h>
#include <sharedregion.h>

/* ioctl interface to listmp_get_config function */
static inline int listmp_ioctl_get_config(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct listmp_config config;

	listmp_get_config(&config);
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

/* ioctl interface to listmp_setup function */
static inline int listmp_ioctl_setup(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct listmp_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
				sizeof(struct listmp_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = listmp_setup(&config);

exit:
	return retval;
}

/* ioctl interface to listmp_destroy function */
static inline int listmp_ioctl_destroy(struct listmp_cmd_args *cargs)
{
	cargs->api_status = listmp_destroy();
	return 0;
}

/* ioctl interface to listmp_params_init function */
static inline int listmp_ioctl_params_init(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct listmp_params params;

	size = copy_from_user(&params,
				cargs->args.params_init.params,
				sizeof(struct listmp_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	listmp_params_init(&params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct listmp_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = 0;

exit:
	return retval;
}

/* ioctl interface to listmp_create function */
static inline int listmp_ioctl_create(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct listmp_params params;

	size = copy_from_user(&params, cargs->args.create.params,
					sizeof(struct listmp_params));
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

	/* Update gate in params. */
	params.gatemp_handle = cargs->args.create.knl_gate;
	cargs->args.create.listmp_handle = listmp_create(&params);

	size = copy_to_user(cargs->args.create.params, &params,
				sizeof(struct listmp_params));
	if (!size)
		goto free_name;

	/* Error copying, so delete the handle */
	retval = -EFAULT;
	if (cargs->args.create.listmp_handle)
		listmp_delete(&cargs->args.create.listmp_handle);

free_name:
	if (cargs->args.create.name_len > 0)
		kfree(params.name);

	cargs->api_status = 0;
exit:
	return retval;
}

/* ioctl interface to listmp_delete function */
static inline int listmp_ioctl_delete(struct listmp_cmd_args *cargs)
{
	cargs->api_status = listmp_delete(
		&(cargs->args.delete_instance.listmp_handle));
	return 0;
}

/* ioctl interface to listmp_open function */
static inline int listmp_ioctl_open(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	char *name = NULL;
	void *listmp_handle = NULL;

	if (cargs->args.open.name_len > 0) {
		name = kmalloc(cargs->args.open.name_len, GFP_KERNEL);
		if (name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		/* Copy the name */
		size = copy_from_user(name, cargs->args.open.name,
					cargs->args.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	/* Update gate in params. */
	cargs->api_status = listmp_open(name, &listmp_handle);
	cargs->args.open.listmp_handle = listmp_handle;

free_name:
	if (cargs->args.open.name_len > 0)
		kfree(name);
exit:
	return retval;
}

/* ioctl interface to listmp_open_by_addr function */
static inline int listmp_ioctl_open_by_addr(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	void *listmp_handle = NULL;
	void *shared_addr = NULL;

	/* For open_by_addr, the shared_add_srptr may be invalid */
	if (cargs->args.open_by_addr.shared_addr_srptr != \
		(u32)SHAREDREGION_INVALIDSRPTR) {
		shared_addr = sharedregion_get_ptr((u32 *)
				cargs->args.open_by_addr.shared_addr_srptr);
	}

	/* Update gate in params. */
	cargs->api_status = listmp_open_by_addr(shared_addr, &listmp_handle);
	cargs->args.open_by_addr.listmp_handle = listmp_handle;

	return retval;
}

/* ioctl interface to listmp_close function */
static inline int listmp_ioctl_close(struct listmp_cmd_args *cargs)
{
	cargs->api_status = listmp_close(&cargs->args.close.listmp_handle);
	return 0;
}

/* ioctl interface to listmp_empty function */
static inline int listmp_ioctl_isempty(struct listmp_cmd_args *cargs)
{
	cargs->args.is_empty.is_empty = \
		listmp_empty(cargs->args.is_empty.listmp_handle);
	cargs->api_status = 0;
	return 0;
}

/* ioctl interface to listmp_get_head function */
static inline int listmp_ioctl_get_head(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem;
	u32 *elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	cargs->api_status = LISTMP_E_FAIL;

	elem = listmp_get_head(cargs->args.get_head.listmp_handle);
	if (unlikely(elem == NULL))
		goto exit;

	index = sharedregion_get_id(elem);
	if (unlikely(index < 0))
		goto exit;

	elem_srptr = sharedregion_get_srptr((void *)elem, index);
	cargs->api_status = 0;

exit:
	cargs->args.get_head.elem_srptr = elem_srptr;
	return 0;
}

/* ioctl interface to listmp_get_tail function */
static inline int listmp_ioctl_get_tail(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem;
	u32 *elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	cargs->api_status = LISTMP_E_FAIL;

	elem = listmp_get_tail(cargs->args.get_tail.listmp_handle);
	if (unlikely(elem == NULL))
		goto exit;

	index = sharedregion_get_id(elem);
	if (unlikely(index < 0))
		goto exit;

	elem_srptr = sharedregion_get_srptr((void *)elem, index);
	cargs->api_status = 0;

exit:
	cargs->args.get_tail.elem_srptr = elem_srptr;
	return 0;
}

/* ioctl interface to listmp_put_head function */
static inline int listmp_ioctl_put_head(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.put_head.elem_srptr);
	cargs->api_status = listmp_put_head(
		cargs->args.put_head.listmp_handle, elem);

	return 0;
}

/* ioctl interface to listmp_put_tail function */
static inline int listmp_ioctl_put_tail(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.put_tail.elem_srptr);
	cargs->api_status = listmp_put_tail(
		cargs->args.put_head.listmp_handle, elem);

	return 0;
}

/* ioctl interface to listmp_insert function */
static inline int listmp_ioctl_insert(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *new_elem;
	struct listmp_elem *cur_elem;
	int status = -1;

	new_elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.insert.new_elem_srptr);
	if (unlikely(new_elem == NULL))
		goto exit;

	cur_elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.insert.cur_elem_srptr);
	if (unlikely(cur_elem == NULL))
		goto exit;

	status = listmp_insert(cargs->args.insert.listmp_handle, new_elem,
				cur_elem);
exit:
	cargs->api_status = status;
	return 0;
}

/* ioctl interface to listmp_remove function */
static inline int listmp_ioctl_remove(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem;

	elem = (struct listmp_elem *) sharedregion_get_ptr(
					cargs->args.remove.elem_srptr);
	cargs->api_status = listmp_remove(
				cargs->args.get_head.listmp_handle, elem);

	return 0;
}

/* ioctl interface to listmp_next function */
static inline int listmp_ioctl_next(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem = NULL;
	struct listmp_elem *ret_elem = NULL;
	u32 *next_elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	if (cargs->args.next.elem_srptr != NULL) {
		elem = (struct listmp_elem *) sharedregion_get_ptr(
						cargs->args.next.elem_srptr);
	}
	ret_elem = (struct listmp_elem *) listmp_next(
					cargs->args.next.listmp_handle, elem);
	if (unlikely(ret_elem == NULL))
		goto exit;

	index = sharedregion_get_id(ret_elem);
	if (unlikely(index < 0))
		goto exit;

	next_elem_srptr = sharedregion_get_srptr((void *)ret_elem, index);
	cargs->api_status = 0;

exit:
	cargs->args.next.next_elem_srptr = next_elem_srptr;
	return 0;
}

/* ioctl interface to listmp_prev function */
static inline int listmp_ioctl_prev(struct listmp_cmd_args *cargs)
{
	struct listmp_elem *elem = NULL;
	struct listmp_elem *ret_elem = NULL;
	u32 *prev_elem_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	if (cargs->args.next.elem_srptr != NULL) {
		elem = (struct listmp_elem *) sharedregion_get_ptr(
						cargs->args.prev.elem_srptr);
	}
	ret_elem = (struct listmp_elem *) listmp_prev(
					cargs->args.prev.listmp_handle, elem);
	if (unlikely(ret_elem == NULL))
		goto exit;

	index = sharedregion_get_id(ret_elem);
	if (unlikely(index < 0))
		goto exit;

	prev_elem_srptr = sharedregion_get_srptr((void *)ret_elem, index);
	cargs->api_status = 0;

exit:
	cargs->args.prev.prev_elem_srptr = prev_elem_srptr;
	return 0;

}

/* ioctl interface to listmp_shared_mem_req function */
static inline int listmp_ioctl_shared_mem_req(struct listmp_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct listmp_params params;

	size = copy_from_user(&params, cargs->args.shared_mem_req.params,
				sizeof(struct listmp_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	params.shared_addr = sharedregion_get_ptr(
			cargs->args.shared_mem_req.shared_addr_srptr);
	cargs->args.shared_mem_req.bytes = listmp_shared_mem_req(&params);
	cargs->api_status = 0;

exit:
	return retval;
}

/* ioctl interface function for listmp module */
int listmp_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct listmp_cmd_args __user *uarg =
			(struct listmp_cmd_args __user *)args;
	struct listmp_cmd_args cargs;
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
	size = copy_from_user(&cargs, uarg, sizeof(struct listmp_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_LISTMP_GETCONFIG:
		os_status = listmp_ioctl_get_config(&cargs);
		break;

	case CMD_LISTMP_SETUP:
		os_status = listmp_ioctl_setup(&cargs);
		break;

	case CMD_LISTMP_DESTROY:
		os_status = listmp_ioctl_destroy(&cargs);
		break;

	case CMD_LISTMP_PARAMS_INIT:
		os_status = listmp_ioctl_params_init(&cargs);
		break;

	case CMD_LISTMP_CREATE:
		os_status = listmp_ioctl_create(&cargs);
		break;

	case CMD_LISTMP_DELETE:
		os_status = listmp_ioctl_delete(&cargs);
		break;

	case CMD_LISTMP_OPEN:
		os_status = listmp_ioctl_open(&cargs);
		break;

	case CMD_LISTMP_CLOSE:
		os_status = listmp_ioctl_close(&cargs);
		break;

	case CMD_LISTMP_ISEMPTY:
		os_status = listmp_ioctl_isempty(&cargs);
		break;

	case CMD_LISTMP_GETHEAD:
		os_status = listmp_ioctl_get_head(&cargs);
		break;

	case CMD_LISTMP_GETTAIL:
		os_status = listmp_ioctl_get_tail(&cargs);
		break;

	case CMD_LISTMP_PUTHEAD:
		os_status = listmp_ioctl_put_head(&cargs);
		break;

	case CMD_LISTMP_PUTTAIL:
		os_status = listmp_ioctl_put_tail(&cargs);
		break;

	case CMD_LISTMP_INSERT:
		os_status = listmp_ioctl_insert(&cargs);
		break;

	case CMD_LISTMP_REMOVE:
		os_status = listmp_ioctl_remove(&cargs);
		break;

	case CMD_LISTMP_NEXT:
		os_status = listmp_ioctl_next(&cargs);
		break;

	case CMD_LISTMP_PREV:
		os_status = listmp_ioctl_prev(&cargs);
		break;

	case CMD_LISTMP_SHAREDMEMREQ:
		os_status = listmp_ioctl_shared_mem_req(&cargs);
		break;

	case CMD_LISTMP_OPENBYADDR:
		os_status = listmp_ioctl_open_by_addr(&cargs);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
	}
	if (os_status < 0)
		goto exit;

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs, sizeof(struct listmp_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}
	return os_status;

exit:
	printk(KERN_ERR "listmp_ioctl failed: status = 0x%x\n", os_status);
	return os_status;
}
