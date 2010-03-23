/*
 *  messageq_ioctl.c
 *
 *  This file implements all the ioctl operations required on the messageq
 *  module.
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
#include <messageq.h>
#include <messageq_ioctl.h>
#include <sharedregion.h>

/*
 * ======== messageq_ioctl_put ========
 *  Purpose:
 *  This ioctl interface to messageq_put function
 */
static inline int messageq_ioctl_put(struct messageq_cmd_args *cargs)
{
	int status = 0;
	messageq_msg msg;

	msg = (messageq_msg) sharedregion_get_ptr(cargs->args.put.msg_srptr);
	if (unlikely(msg == NULL))
		goto exit;

	status = messageq_put(cargs->args.put.queue_id, msg);

	cargs->api_status = status;
exit:
	return 0;
}

/*
 * ======== messageq_ioctl_get ========
 *  Purpose:
 *  This ioctl interface to messageq_get function
 */
static inline int messageq_ioctl_get(struct messageq_cmd_args *cargs)
{
	messageq_msg msg = NULL;
	u32 *msg_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	cargs->api_status = messageq_get(cargs->args.get.messageq_handle,
					&msg,
					cargs->args.get.timeout);
	if (unlikely(cargs->api_status < 0))
		goto exit;

	index = sharedregion_get_index(msg);
	if (unlikely(index < 0)) {
		cargs->api_status = index;
		goto exit;
	}

	msg_srptr = sharedregion_get_srptr(msg, index);

exit:
	cargs->args.get.msg_srptr = msg_srptr;
	return 0;
}

/*
 * ======== messageq_ioctl_count ========
 *  Purpose:
 *  This ioctl interface to messageq_count function
 */
static inline int messageq_ioctl_count(struct messageq_cmd_args *cargs)
{
	int result = messageq_count(cargs->args.count.messageq_handle);
	if (result < 0)
		cargs->api_status = result;
	else
		cargs->args.count.count = result;

	return 0;
}

/*
 * ======== messageq_ioctl_alloc ========
 *  Purpose:
 *  This ioctl interface to messageq_alloc function
 */
static inline int messageq_ioctl_alloc(struct messageq_cmd_args *cargs)
{
	messageq_msg msg;
	u32 *msg_srptr = SHAREDREGION_INVALIDSRPTR;
	int index;

	msg = messageq_alloc(cargs->args.alloc.heap_id, cargs->args.alloc.size);
	if (unlikely(msg == NULL))
		goto exit;

	index = sharedregion_get_index(msg);
	if (unlikely(index < 0))
		goto exit;

	msg_srptr = sharedregion_get_srptr(msg, index);

	cargs->api_status = 0;
exit:
	cargs->args.alloc.msg_srptr = msg_srptr;
	return 0;
}

/*
 * ======== messageq_ioctl_free ========
 *  Purpose:
 *  This ioctl interface to messageq_free function
 */
static inline int messageq_ioctl_free(struct messageq_cmd_args *cargs)
{
	int status = 0;
	messageq_msg msg;

	msg = sharedregion_get_ptr(cargs->args.free.msg_srptr);
	if (unlikely(msg == NULL))
		goto exit;
	status = messageq_free(msg);

	cargs->api_status = status;
exit:
	return 0;
}

/*
 * ======== messageq_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to messageq_params_init function
 */
static inline int messageq_ioctl_params_init(struct messageq_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	struct messageq_params params;

	messageq_params_init(cargs->args.params_init.messageq_handle,
						&params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct messageq_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = status;
exit:
	return retval;
}

/*
 * ======== messageq_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to messageq_create function
 */
static inline int messageq_ioctl_create(struct messageq_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	struct messageq_params params;
	char *name = NULL;

	size = copy_from_user(&params, cargs->args.create.params,
					sizeof(struct messageq_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	/* Allocate memory for the name */
	if (cargs->args.create.name_len > 0) {
		name = kmalloc(cargs->args.create.name_len, GFP_KERNEL);
		if (name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		size = copy_from_user(name, cargs->args.create.name,
						cargs->args.create.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	cargs->args.create.messageq_handle = messageq_create(name, &params);
	if (cargs->args.create.messageq_handle != NULL) {
		cargs->args.create.queue_id = messageq_get_queue_id(
					 cargs->args.create.messageq_handle);
	}

free_name:
	if (cargs->args.create.name_len > 0)
		kfree(name);

	cargs->api_status = status;
exit:
	return retval;
}

/*
 * ======== messageq_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to messageq_delete function
 */
static inline int messageq_ioctl_delete(struct messageq_cmd_args *cargs)
{
	cargs->api_status =
		messageq_delete(&(cargs->args.delete_messageq.messageq_handle));
	return 0;
}

/*
 * ======== messageq_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to messageq_open function
 */
static inline int messageq_ioctl_open(struct messageq_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	char *name = NULL;
	u32 queue_id = MESSAGEQ_INVALIDMESSAGEQ;

	/* Allocate memory for the name */
	if (cargs->args.open.name_len > 0) {
		name = kmalloc(cargs->args.open.name_len, GFP_KERNEL);
		if (name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}
		size = copy_from_user(name, cargs->args.open.name,
						cargs->args.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	status = messageq_open(name, &queue_id);
	cargs->args.open.queue_id = queue_id;

free_name:
	if (cargs->args.open.name_len > 0)
		kfree(name);

	cargs->api_status = status;
exit:
	return retval;
}

/*
 * ======== messageq_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to messageq_close function
 */
static inline int messageq_ioctl_close(struct messageq_cmd_args *cargs)
{
	u32 queue_id = cargs->args.close.queue_id;
	messageq_close(&queue_id);
	cargs->args.close.queue_id = queue_id;

	cargs->api_status = 0;
	return 0;
}

/*
 * ======== messageq_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to messageq_get_config function
 */
static inline int messageq_ioctl_get_config(struct messageq_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_config config;

	messageq_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct messageq_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== messageq_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to messageq_setup function
 */
static inline int messageq_ioctl_setup(struct messageq_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
					sizeof(struct messageq_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status =  messageq_setup(&config);

exit:
	return retval;
}

/*
 * ======== messageq_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to messageq_destroy function
 */
static inline int messageq_ioctl_destroy(struct messageq_cmd_args *cargs)
{
	cargs->api_status = messageq_destroy();
	return 0;
}


/*
 * ======== messageq_ioctl_register_heap ========
 *  Purpose:
 *  This ioctl interface to messageq_register_heap function
 */
static inline int messageq_ioctl_register_heap(struct messageq_cmd_args *cargs)
{
	cargs->api_status = \
		messageq_register_heap(cargs->args.register_heap.heap_handle,
				cargs->args.register_heap.heap_id);
	return 0;
}

/*
 * ======== messageq_ioctl_unregister_heap ========
 *  Purpose:
 *  This ioctl interface to messageq_unregister_heap function
 */
static inline int messageq_ioctl_unregister_heap(
					struct messageq_cmd_args *cargs)
{
	cargs->api_status =  messageq_unregister_heap(
				cargs->args.unregister_heap.heap_id);
	return 0;
}

/*
 * ======== messageq_ioctl ========
 *  Purpose:
 *  ioctl interface function for messageq module
 */
int messageq_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct messageq_cmd_args __user *uarg =
				(struct messageq_cmd_args __user *)args;
	struct messageq_cmd_args cargs;
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
	size = copy_from_user(&cargs, uarg, sizeof(struct messageq_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_MESSAGEQ_PUT:
		os_status = messageq_ioctl_put(&cargs);
		break;

	case CMD_MESSAGEQ_GET:
		os_status = messageq_ioctl_get(&cargs);
		break;

	case CMD_MESSAGEQ_COUNT:
		os_status = messageq_ioctl_count(&cargs);
		break;

	case CMD_MESSAGEQ_ALLOC:
		os_status = messageq_ioctl_alloc(&cargs);
		break;

	case CMD_MESSAGEQ_FREE:
		os_status = messageq_ioctl_free(&cargs);
		break;

	case CMD_MESSAGEQ_PARAMS_INIT:
		os_status = messageq_ioctl_params_init(&cargs);
		break;

	case CMD_MESSAGEQ_CREATE:
		os_status = messageq_ioctl_create(&cargs);
		break;

	case CMD_MESSAGEQ_DELETE:
		os_status = messageq_ioctl_delete(&cargs);
		break;

	case CMD_MESSAGEQ_OPEN:
		os_status = messageq_ioctl_open(&cargs);
		break;

	case CMD_MESSAGEQ_CLOSE:
		os_status = messageq_ioctl_close(&cargs);
		break;

	case CMD_MESSAGEQ_GETCONFIG:
		os_status = messageq_ioctl_get_config(&cargs);
		break;

	case CMD_MESSAGEQ_SETUP:
		os_status = messageq_ioctl_setup(&cargs);
		break;

	case CMD_MESSAGEQ_DESTROY:
		os_status = messageq_ioctl_destroy(&cargs);
		break;

	case CMD_MESSAGEQ_REGISTERHEAP:
		os_status = messageq_ioctl_register_heap(&cargs);
		break;

	case CMD_MESSAGEQ_UNREGISTERHEAP:
		os_status = messageq_ioctl_unregister_heap(&cargs);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
	}

	if ((cargs.api_status == -ERESTARTSYS) || (cargs.api_status == -EINTR))
		os_status = -ERESTARTSYS;

	if (os_status < 0)
		goto exit;

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs, sizeof(struct messageq_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}
	return os_status;

exit:
	printk(KERN_ERR "messageq_ioctl failed: status = 0x%x\n", os_status);
	return os_status;
}
