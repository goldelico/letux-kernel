/*
 *  sysipc_ioctl.c
 *
 *  This file implements all the ioctl operations required on the sysmgr
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
#include <ipc.h>
#include <sysipc_ioctl.h>
/*#include <platform.h>*/


/*
 * ioctl interface to ipc_setup function
 */
static inline int sysipc_ioctl_setup(struct sysipc_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct ipc_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
					sizeof(struct ipc_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = ipc_setup(&config);

exit:
	return retval;
}

/*
 * ioctl interface to ipc_control function
 */
static inline int sysipc_ioctl_control(struct sysipc_cmd_args *cargs)
{
	cargs->api_status = ipc_control(cargs->args.control.proc_id,
					cargs->args.control.cmd_id,
					cargs->args.control.arg);
	return 0;
}

/*
 * ioctl interface to ipc_read_config function
 */
static inline int sysipc_ioctl_read_config(struct sysipc_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	void *cfg = NULL;

	cfg = kzalloc(cargs->args.read_config.size, GFP_KERNEL);
	if (cfg == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	cargs->api_status = ipc_read_config(
					cargs->args.read_config.remote_proc_id,
					cargs->args.read_config.tag, cfg,
					cargs->args.read_config.size);

	size = copy_to_user(cargs->args.read_config.cfg, cfg,
					cargs->args.read_config.size);
	if (size)
		retval = -EFAULT;

	kfree(cfg);

exit:
	return retval;
}

/*
 * ioctl interface to ipc_write_config function
 */
static inline int sysipc_ioctl_write_config(struct sysipc_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	void *cfg = NULL;

	cfg = kzalloc(cargs->args.write_config.size, GFP_KERNEL);
	if (cfg == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(cfg, cargs->args.write_config.cfg,
					cargs->args.write_config.size);
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = ipc_write_config(
					cargs->args.write_config.remote_proc_id,
					cargs->args.write_config.tag, cfg,
					cargs->args.write_config.size);

	kfree(cfg);

exit:
	return retval;
}

/*
 * ioctl interface to sysmgr_destroy function
 */
static inline int sysipc_ioctl_destroy(struct sysipc_cmd_args *cargs)
{
	cargs->api_status = ipc_destroy();
	return 0;
}

/*
 * ioctl interface function for sysmgr module
 */
int sysipc_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct sysipc_cmd_args __user *uarg =
				(struct sysipc_cmd_args __user *)args;
	struct sysipc_cmd_args cargs;
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
	size = copy_from_user(&cargs, uarg, sizeof(struct sysipc_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_IPC_SETUP:
		os_status = sysipc_ioctl_setup(&cargs);
		break;

	case CMD_IPC_DESTROY:
		os_status = sysipc_ioctl_destroy(&cargs);
		break;

	case CMD_IPC_CONTROL:
		os_status = sysipc_ioctl_control(&cargs);
		break;

	case CMD_IPC_READCONFIG:
		os_status = sysipc_ioctl_read_config(&cargs);
		break;

	case CMD_IPC_WRITECONFIG:
		os_status = sysipc_ioctl_write_config(&cargs);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
	}
	if (os_status < 0)
		goto exit;

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs, sizeof(struct sysipc_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

exit:
	return os_status;
}
