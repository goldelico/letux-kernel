/*
 *  sysmgr_ioctl.c
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
#include <sysmgr.h>
#include <sysmgr_ioctl.h>
#include <platform.h>
/*
 * ======== sysmgr_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to sysmgr_setup function
 */
static inline int sysmgr_ioctl_setup(struct sysmgr_cmd_args *cargs)
{
	s32 retval = 0;
	unsigned long size;
	struct sysmgr_config config;

	size = copy_from_user(&config, cargs->args.setup.config,
					sizeof(struct sysmgr_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->api_status = sysmgr_setup(&config);

exit:
	return retval;
}

/*
 * ======== sysmgr_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to sysmgr_destroy function
 */
static inline int sysmgr_ioctl_destroy(struct sysmgr_cmd_args *cargs)
{
	cargs->api_status = sysmgr_destroy();
	return 0;
}

/*
 * ======== sysmgr_ioctl ========
 *  Purpose:
 *  ioctl interface function for sysmgr module
 */
int sysmgr_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	int os_status = 0;
	struct sysmgr_cmd_args __user *uarg =
				(struct sysmgr_cmd_args __user *)args;
	struct sysmgr_cmd_args cargs;
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
	size = copy_from_user(&cargs, uarg, sizeof(struct sysmgr_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_SYSMGR_SETUP:
		os_status = sysmgr_ioctl_setup(&cargs);
		break;

	case CMD_SYSMGR_DESTROY:
		os_status = sysmgr_ioctl_destroy(&cargs);
		break;

	case CMD_SYSMGR_LOADCALLBACK:
#if defined CONFIG_SYSLINK_USE_SYSMGR
		platform_load_callback((void *)(cargs.args.proc_id));
		cargs.api_status = 0;
#endif
		break;

	case CMD_SYSMGR_STARTCALLBACK:
#if defined CONFIG_SYSLINK_USE_SYSMGR
		platform_start_callback((void *)(cargs.args.proc_id));
		cargs.api_status = 0;
#endif
		break;

	case CMD_SYSMGR_STOPCALLBACK:
#if defined CONFIG_SYSLINK_USE_SYSMGR
		platform_stop_callback((void *)(cargs.args.proc_id));
		cargs.api_status = 0;
#endif
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
	size = copy_to_user(uarg, &cargs, sizeof(struct sysmgr_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

exit:
	return os_status;
}
