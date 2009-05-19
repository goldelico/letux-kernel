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

#include <gt.h>
#include <gatepeterson.h>
#include <gatepeterson_ioctl.h>
#include <sharedregion.h>

/*
 * ======== gatepeterson_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_get_config function
 */
static int gatepeterson_ioctl_get_config(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_config config;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval =  gatepeterson_get_config(&config);
	if (unlikely(retval))
		goto exit;

	size = copy_to_user(cargs->cmd_arg.get_config.config, &config,
				sizeof(struct gatepeterson_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_setup function
 */
static int gatepeterson_ioctl_setup(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_config config;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&config, cargs->cmd_arg.setup.config,
				sizeof(struct gatepeterson_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval =  gatepeterson_setup(&config);
	if (unlikely(retval))
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_destroy function
 */
static int gatepeterson_ioctl_destroy()
{
	s32 retval = 0;
	retval = gatepeterson_destroy();
	return retval;
}

/*
 * ======== gatepeterson_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_params_init function
 */
static int gatepeterson_ioctl_params_init(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_params params;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval =  gatepeterson_params_init(&params);
	size = copy_to_user(cargs->cmd_arg.params_init.params, &params,
				sizeof(struct gatepeterson_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_create function
 */
static int gatepeterson_ioctl_create(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_params params;
	void *handle = NULL;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.create.params,
					sizeof(struct gatepeterson_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->cmd_arg.create.name_len > 0) {
		params.name = kmalloc(cargs->cmd_arg.create.name_len,
								GFP_KERNEL);
		if (params.name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}

		size = copy_from_user(params.name,
					cargs->cmd_arg.create.params->name,
					cargs->cmd_arg.create.name_len);
		if (size) {
			retval = -EFAULT;
			goto name_from_usr_error;
		}

	}

	params.shared_addr = sharedregion_get_ptr((u32 *)params.shared_addr);
	handle =  gatepeterson_create(&params);
	/* Here we are not validating the return from the module.
	Even it is nul, we pass it to user and user has to pass
	proper return to application
	*/
	size = copy_to_user(&args->cmd_arg.create.handle, &handle,
							sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto handle_to_usr_error;
	}

	if (cargs->cmd_arg.create.name_len > 0)
		kfree(params.name);

	return 0;

handle_to_usr_error:
	if (handle)
		gatepeterson_delete(&handle);

name_from_usr_error:
	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_ioctl_delete function
 */
static int gatepeterson_ioctl_delete(struct gatepeterson_cmd_args *args)

{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = gatepeterson_delete(&cargs->cmd_arg.delete.handle);
	if (retval)
		goto exit;

	/* Clear user side memory that stored handle */
	size = copy_to_user(&args->cmd_arg.delete.handle,
			&cargs->cmd_arg.delete.handle, sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_open function
 */
static int gatepeterson_ioctl_open(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_params params;
	void *handle = NULL;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.open.params,
				sizeof(struct gatepeterson_params));

	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->cmd_arg.open.name_len > 0) {
		params.name = kmalloc(cargs->cmd_arg.open.name_len, GFP_KERNEL);
		if (params.name != NULL) {
			retval = -ENOMEM;
			goto exit;
		}

		size = copy_from_user(params.name,
					cargs->cmd_arg.open.params->name,
					cargs->cmd_arg.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto name_from_usr_error;
		}
	}

	params.shared_addr = sharedregion_get_ptr((u32 *)params.shared_addr);
	retval = gatepeterson_open(&handle, &params);
	if (retval)
		goto exit;

	size = copy_to_user(&args->cmd_arg.open.handle, &handle,
						sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto handle_to_usr_error;
	}


	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

	return 0;

handle_to_usr_error:
	if (handle)
		gatepeterson_delete(&handle);

name_from_usr_error:
	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_close function
 */
static int gatepeterson_ioctl_close(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = gatepeterson_close(&cargs->cmd_arg.close.handle);
	if (retval)
		goto exit;

	size = copy_to_user(&args->cmd_arg.close.handle,
				&cargs->cmd_arg.close.handle, sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}


exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_enter ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_enter function
 */
static int gatepeterson_ioctl_enter(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = gatepeterson_enter(cargs->cmd_arg.enter.handle);

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl_leave ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_leave function
 */
static int gatepeterson_ioctl_leave(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
			sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	gatepeterson_leave(cargs->cmd_arg.enter.handle,
				cargs->cmd_arg.enter.flags);

exit:
	return 0;
}

/*
 * ======== gatepeterson_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to gatepeterson_shared_memreq function
 */
static int gatepeterson_ioctl_shared_memreq(struct gatepeterson_cmd_args *args)
{
	struct gatepeterson_cmd_args uarg;
	struct gatepeterson_cmd_args *cargs = &uarg;
	struct gatepeterson_params params;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
			sizeof(struct gatepeterson_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.shared_memreq.params,
				sizeof(struct gatepeterson_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	cargs->cmd_arg.shared_memreq.bytes =
		gatepeterson_shared_memreq(cargs->cmd_arg.shared_memreq.params);

	size = copy_to_user(&args->cmd_arg.shared_memreq.bytes,
			&cargs->cmd_arg.shared_memreq.bytes, sizeof(u32));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== gatepeterson_ioctl ========
 *  Purpose:
 *  This ioctl interface for gatepeterson module
 */
int gatepeterson_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	struct gatepeterson_cmd_args __user *uarg =
				(struct gatepeterson_cmd_args __user *)args;
	s32 retval = 0;

	gt_4trace(curTrace, GT_ENTER, "gatepeterson_ioctl"
		"inode: %x, filp: %x,\n cmd: %x, args: %x",
		inode, filp, cmd, args);

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));

	if (retval) {
		retval = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_GATEPETERSON_GETCONFIG:
		retval = gatepeterson_ioctl_get_config(uarg);
		break;

	case CMD_GATEPETERSON_SETUP:
		retval = gatepeterson_ioctl_setup(uarg);
		break;

	case CMD_GATEPETERSON_DESTROY:
		retval = gatepeterson_ioctl_destroy();
		break;

	case CMD_GATEPETERSON_PARAMS_INIT:
		retval  = gatepeterson_ioctl_params_init(uarg);
		break;

	case CMD_GATEPETERSON_CREATE:
		retval = gatepeterson_ioctl_create(uarg);
		break;

	case CMD_GATEPETERSON_DELETE:
		retval = gatepeterson_ioctl_delete(uarg);
		break;

	case CMD_GATEPETERSON_OPEN:
		retval = gatepeterson_ioctl_open(uarg);
		break;

	case CMD_GATEPETERSON_CLOSE:
		retval = gatepeterson_ioctl_close(uarg);
		break;

	case CMD_GATEPETERSON_ENTER:
		retval = gatepeterson_ioctl_enter(uarg);
		break;

	case CMD_GATEPETERSON_LEAVE:
		retval = gatepeterson_ioctl_leave(uarg);
		break;

	case CMD_GATEPETERSON_SHAREDMEMREQ:
		retval = gatepeterson_ioctl_shared_memreq(uarg);
		break;

	default:
		WARN_ON(cmd);
		retval = -ENOTTY;
		break;
	}

exit:
	return retval;
}

