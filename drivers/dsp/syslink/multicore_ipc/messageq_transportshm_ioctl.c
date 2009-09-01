/*
 *  messageq_transportshm_ioctl.c
 *
 *  This file implements all the ioctl operations required on the
 *  messageq_transportshm module.
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

/* Module Headers */
#include <messageq.h>
#include <messageq_transportshm.h>
#include <messageq_transportshm_ioctl.h>
#include <sharedregion.h>

/*
 * ======== messageq_transportshm_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_get_config function
 */
static inline int messageq_transportshm_ioctl_get_config(
				struct messageq_transportshm_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_transportshm_config config;

	messageq_transportshm_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct messageq_transportshm_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== messageq_transportshm_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_setup function
 */
static inline int messageq_transportshm_ioctl_setup(
				struct messageq_transportshm_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_transportshm_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
				sizeof(struct messageq_transportshm_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = messageq_transportshm_setup(&config);

exit:
	return retval;
}

/*
 * ======== messageq_transportshm_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_destroy function
 */
static inline int messageq_transportshm_ioctl_destroy(
				struct messageq_transportshm_cmd_args *cargs)
{
	cargs->api_status = messageq_transportshm_destroy();
	return 0;
}

/*
 * ======== messageq_transportshm_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_params_init function
 */
static inline int messageq_transportshm_ioctl_params_init(
				struct messageq_transportshm_cmd_args *cargs)
{
	s32 retval = 0;
	int status = 0;
	unsigned long size;
	struct messageq_transportshm_params params;

	messageq_transportshm_params_init(
		cargs->args.params_init.messageq_transportshm_handle, &params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct messageq_transportshm_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = status;
exit:
	return retval;
}

/*
 * ======== messageq_transportshm_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_create function
 */
static inline int messageq_transportshm_ioctl_create(
				struct messageq_transportshm_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_transportshm_params params;

	size = copy_from_user(&params, cargs->args.create.params,
				sizeof(struct messageq_transportshm_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.create.shared_addr_srptr);
	if (unlikely(params.shared_addr == NULL))
		goto exit;

	params.gate = cargs->args.create.knl_lock_handle;
	params.notify_driver = cargs->args.create.knl_notify_driver;
	cargs->args.create.messageq_transportshm_handle = \
		messageq_transportshm_create(cargs->args.create.proc_id,
						&params);

	/*
	 * Here we are not validating the return from the module.
	 * Even it is NULL, we pass it to user and user has to pass
	 * proper return to application
	*/
	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== messageq_transportshm_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_ioctl_delete function
 */
static inline int messageq_transportshm_ioctl_delete(
				struct messageq_transportshm_cmd_args *cargs)
{
	cargs->api_status = messageq_transportshm_delete(
		&(cargs->args.delete_transport.messageq_transportshm_handle));
	return 0;
}

/*
 * ======== messageq_transportshm_ioctl_put ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_put function
 */
static inline int messageq_transportshm_ioctl_put(
				struct messageq_transportshm_cmd_args *cargs)
{
	int status = 0;
	messageq_msg msg;

	msg = (messageq_msg) sharedregion_get_ptr(cargs->args.put.msg_srptr);
	if (unlikely(msg == NULL))
		goto exit;

	status = messageq_transportshm_put(
			cargs->args.put.messageq_transportshm_handle, msg);

	cargs->api_status = status;
exit:
	return 0;
}

/*
 * ======== messageq_transportshm_ioctl_leave ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_leave function
 */
static inline int messageq_transportshm_ioctl_get_status(
				struct messageq_transportshm_cmd_args *cargs)
{
	cargs->args.get_status.status = \
		messageq_transportshm_get_status(
			cargs->args.get_status.messageq_transportshm_handle);
	cargs->api_status = 0;
	return 0;
}

/*
 * ======== messageq_transportshm_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to messageq_transportshm_shared_memreq function
 */
static inline int messageq_transportshm_ioctl_shared_memreq(
				struct messageq_transportshm_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct messageq_transportshm_params params;

	size = copy_from_user(&params, cargs->args.shared_memreq.params,
				sizeof(struct messageq_transportshm_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->args.shared_memreq.bytes =
		messageq_transportshm_shared_mem_req(&params);

	cargs->api_status = 0;
exit:
	return retval;
}

/*
 * ======== messageq_transportshm_ioctl ========
 *  Purpose:
 *  ioctl interface function for messageq_transportshm
 */
int messageq_transportshm_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct messageq_transportshm_cmd_args __user *uarg =
			(struct messageq_transportshm_cmd_args __user *)args;
	s32 retval = 0;
	struct messageq_transportshm_cmd_args cargs;
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
				sizeof(struct messageq_transportshm_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_MESSAGEQ_TRANSPORTSHM_GETCONFIG:
		os_status = messageq_transportshm_ioctl_get_config(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_SETUP:
		os_status = messageq_transportshm_ioctl_setup(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_DESTROY:
		os_status = messageq_transportshm_ioctl_destroy(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_PARAMS_INIT:
		retval  = messageq_transportshm_ioctl_params_init(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_CREATE:
		os_status = messageq_transportshm_ioctl_create(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_DELETE:
		os_status = messageq_transportshm_ioctl_delete(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_PUT:
		os_status = messageq_transportshm_ioctl_put(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_GETSTATUS:
		os_status = messageq_transportshm_ioctl_get_status(&cargs);
		break;

	case CMD_MESSAGEQ_TRANSPORTSHM_SHAREDMEMREQ:
		os_status = messageq_transportshm_ioctl_shared_memreq(&cargs);
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
				sizeof(struct messageq_transportshm_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}
	return os_status;

exit:
	printk(KERN_ERR "messageq_transportshm_ioctl failed: status = 0x%x\n",
		os_status);
	return os_status;
}
