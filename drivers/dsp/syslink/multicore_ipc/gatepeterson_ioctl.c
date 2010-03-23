/*
 *  gatepeterson_ioctl.c
 *
 *  The Gate Peterson Algorithm for mutual exclusion of shared memory.
 *  Current implementation works for 2 processors.
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

#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <gatepeterson.h>
#include <gatepeterson_ioctl.h>
#include <sharedregion.h>

/*
 * ======== gatepeterson_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_get_config function
 */
static int gatepeterson_ioctl_get_config(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_config config;
	s32 status = 0;
	s32 size;

	gatepeterson_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct gatepeterson_config));
	if (size)
		status = -EFAULT;

	cargs->api_status = 0;
	return status;
}

/*
 * ======== gatepeterson_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_setup function
 */
static int gatepeterson_ioctl_setup(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_config config;
	s32 status = 0;
	s32 size;

	size = copy_from_user(&config, cargs->args.setup.config,
				sizeof(struct gatepeterson_config));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	cargs->api_status = gatepeterson_setup(&config);

exit:
	return status;
}

/*
 * ======== gatepeterson_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_destroy function
 */
static int gatepeterson_ioctl_destroy(
				struct gatepeterson_cmd_args *cargs)
{
	cargs->api_status = gatepeterson_destroy();
	return 0;
}

/*
 * ======== gatepeterson_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_params_init function
 */
static int gatepeterson_ioctl_params_init(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_params params;
	s32 status = 0;
	s32 size;

	gatepeterson_params_init(cargs->args.params_init.handle,
							&params);
	size = copy_to_user(cargs->args.params_init.params, &params,
					sizeof(struct gatepeterson_params));
	if (size)
		status = -EFAULT;

	cargs->api_status = 0;
	return status;
}

/*
 * ======== gatepeterson_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_create function
 */
static int gatepeterson_ioctl_create(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_params params;
	void *handle = NULL;
	s32 status = 0;
	s32 size;

	size = copy_from_user(&params, cargs->args.create.params,
					sizeof(struct gatepeterson_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	if (cargs->args.create.name_len > 0) {
		params.name = kmalloc(cargs->args.create.name_len + 1,
								GFP_KERNEL);
		if (params.name == NULL) {
			status = -ENOMEM;
			goto exit;
		}

		params.name[cargs->args.create.name_len] = '\0';
		size = copy_from_user(params.name,
					cargs->args.create.params->name,
					cargs->args.create.name_len);
		if (size) {
			status = -EFAULT;
			goto name_from_usr_error;
		}

	}

	params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.create.shared_addr_srptr);
	handle =  gatepeterson_create(&params);
	/* Here we are not validating the return from the module.
	Even it is nul, we pass it to user and user has to pass
	proper return to application
	*/
	cargs->args.create.handle = handle;
	cargs->api_status = 0;

name_from_usr_error:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

exit:
	return status;
}

/*
 * ======== gatepeterson_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_ioctl_delete function
 */
static int gatepeterson_ioctl_delete(struct gatepeterson_cmd_args *cargs)

{
	cargs->api_status = gatepeterson_delete(&cargs->args.delete.handle);
	return 0;
}

/*
 * ======== gatepeterson_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_open function
 */
static int gatepeterson_ioctl_open(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_params params;
	void *handle = NULL;
	s32 status = 0;
	s32 size;

	size = copy_from_user(&params, cargs->args.open.params,
				sizeof(struct gatepeterson_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	if (cargs->args.open.name_len > 0) {
		params.name = kmalloc(cargs->args.open.name_len + 1,
							GFP_KERNEL);
		if (params.name == NULL) {
			status = -ENOMEM;
			goto exit;
		}

		params.name[cargs->args.open.name_len] = '\0';
		size = copy_from_user(params.name,
					cargs->args.open.params->name,
					cargs->args.open.name_len);
		if (size) {
			status = -EFAULT;
			goto name_from_usr_error;
		}
	}

	/* For open by name, the shared_add_srptr may be invalid */
	if (cargs->args.open.shared_addr_srptr != \
					(u32) SHAREDREGION_INVALIDSRPTR) {
		params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.open.shared_addr_srptr);
	}
	cargs->api_status = gatepeterson_open(&handle, &params);
	cargs->args.open.handle = handle;

name_from_usr_error:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

exit:
	return status;
}

/*
 * ======== gatepeterson_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_close function
 */
static int gatepeterson_ioctl_close(struct gatepeterson_cmd_args *cargs)
{
	cargs->api_status = gatepeterson_close(&cargs->args.close.handle);
	return 0;
}

/*
 * ======== gatepeterson_ioctl_enter ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_enter function
 */
static int gatepeterson_ioctl_enter(struct gatepeterson_cmd_args *cargs)
{
	cargs->api_status = gatepeterson_enter(cargs->args.enter.handle);
	return 0;
}

/*
 * ======== gatepeterson_ioctl_leave ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_leave function
 */
static int gatepeterson_ioctl_leave(struct gatepeterson_cmd_args *cargs)
{
	gatepeterson_leave(cargs->args.enter.handle,
						cargs->args.enter.flags);
	cargs->api_status = 0;
	return 0;
}

/*
 * ======== gatepeterson_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_shared_memreq function
 */
static int gatepeterson_ioctl_shared_memreq(struct gatepeterson_cmd_args *cargs)
{
	struct gatepeterson_params params;
	s32 status = 0;
	s32 size;


	size = copy_from_user(&params, cargs->args.shared_memreq.params,
					sizeof(struct gatepeterson_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	cargs->args.shared_memreq.bytes =
		gatepeterson_shared_memreq(cargs->args.shared_memreq.params);
	cargs->api_status = 0;

exit:
	return status;
}

/*
 * ======== gatepeterson_ioctl ========
 *  Purpose:
 *  This ioctl interface for gatepeterson module
 */
int gatepeterson_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	s32 status = 0;
	s32 size = 0;
	struct gatepeterson_cmd_args __user *uarg =
				(struct gatepeterson_cmd_args __user *)args;
	struct gatepeterson_cmd_args cargs;


	if (_IOC_DIR(cmd) & _IOC_READ)
		status = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		status = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));

	if (status) {
		status = -EFAULT;
		goto exit;
	}

	/* Copy the full args from user-side */
	size = copy_from_user(&cargs, uarg,
					sizeof(struct gatepeterson_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_GATEPETERSON_GETCONFIG:
		status = gatepeterson_ioctl_get_config(&cargs);
		break;

	case CMD_GATEPETERSON_SETUP:
		status = gatepeterson_ioctl_setup(&cargs);
		break;

	case CMD_GATEPETERSON_DESTROY:
		status = gatepeterson_ioctl_destroy(&cargs);
		break;

	case CMD_GATEPETERSON_PARAMS_INIT:
		status  = gatepeterson_ioctl_params_init(&cargs);
		break;

	case CMD_GATEPETERSON_CREATE:
		status = gatepeterson_ioctl_create(&cargs);
		break;

	case CMD_GATEPETERSON_DELETE:
		status = gatepeterson_ioctl_delete(&cargs);
		break;

	case CMD_GATEPETERSON_OPEN:
		status = gatepeterson_ioctl_open(&cargs);
		break;

	case CMD_GATEPETERSON_CLOSE:
		status = gatepeterson_ioctl_close(&cargs);
		break;

	case CMD_GATEPETERSON_ENTER:
		status = gatepeterson_ioctl_enter(&cargs);
		break;

	case CMD_GATEPETERSON_LEAVE:
		status = gatepeterson_ioctl_leave(&cargs);
		break;

	case CMD_GATEPETERSON_SHAREDMEMREQ:
		status = gatepeterson_ioctl_shared_memreq(&cargs);
		break;

	default:
		WARN_ON(cmd);
		status = -ENOTTY;
		break;
	}

	if ((cargs.api_status == -ERESTARTSYS) || (cargs.api_status == -EINTR))
		status = -ERESTARTSYS;

	if (status < 0)
		goto exit;

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs,
			sizeof(struct gatepeterson_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

exit:
	return status;
}

