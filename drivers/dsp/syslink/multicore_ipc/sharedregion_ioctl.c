/*
 *  sharedregion_ioctl.c
 *
 *  The sharedregion module is designed to be used in a
 *  multi-processor environment where there are memory regions
 *  that are shared and accessed across different processors
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
#include <sharedregion.h>
#include <sharedregion_ioctl.h>


/*
 * ======== sharedregion_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_config function
 */
static int sharedregion_ioctl_get_config(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	struct sharedregion_config config;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_get_config(&config);
	size = copy_to_user(cargs->cmd_arg.get_config.config, &config,
				sizeof(struct sharedregion_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}


/*
 * ======== sharedregion_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to sharedregion_setup function
 */
static int sharedregion_ioctl_setup(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	struct sharedregion_config config;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&config, cargs->cmd_arg.setup.config,
				sizeof(struct sharedregion_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_setup(&config);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl_destroy========
 *  Purpose:
 *  This ioctl interface to sharedregion_destroy function
 */
static int sharedregion_ioctl_destroy()
{
	s32 retval = 0;
	retval = sharedregion_destroy();
	return retval;
}

/*
 * ======== sharedregion_ioctl_add ========
 *  Purpose:
 *  This ioctl interface to sharedregion_add function
 */
static int sharedregion_ioctl_add(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval =  sharedregion_add(cargs->cmd_arg.add.index,
					cargs->cmd_arg.add.base,
					cargs->cmd_arg.add.len);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}



/*
 * ======== sharedregion_ioctl_get_index ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_index function
 */
static int sharedregion_ioctl_get_index(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;
	s32 index = 0;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	index = sharedregion_get_index(cargs->cmd_arg.get_index.addr);
	if (index < 0) {
		retval = index;
		goto exit;
	}

	size = copy_to_user(&args->cmd_arg.get_index.index, &index,
							sizeof(s32));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl_get_ptr ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_ptr function
 */
static int sharedregion_ioctl_get_ptr(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;
	void *addr = NULL;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	addr = sharedregion_get_ptr(cargs->cmd_arg.get_ptr.srptr);
	/* We are not checking the return from the module, its user
	responsibilty to pass proper value to application
	*/
	size = copy_to_user(&args->cmd_arg.get_ptr.addr, &addr,
						sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl_get_srptr ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_srptr function
 */
static int sharedregion_ioctl_get_srptr(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;
	u32 *srptr = NULL;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	srptr = sharedregion_get_srptr(cargs->cmd_arg.get_srptr.addr,
					cargs->cmd_arg.get_srptr.index);
	/* We are not checking the return from the module, its user
	responsibilty to pass proper value to application
	*/
	size = copy_to_user(&args->cmd_arg.get_srptr.srptr, &srptr,
							sizeof(u32 *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl_get_table_info ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_table_info function
 */
static int sharedregion_ioctl_get_table_info(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	struct sharedregion_info info;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_get_table_info(
				cargs->cmd_arg.get_table_info.index,
				cargs->cmd_arg.get_table_info.proc_id, &info);
	if (retval)
		goto exit;

	size = copy_to_user(cargs->cmd_arg.get_table_info.info, &info,
				sizeof(struct sharedregion_info));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}


/*
 * ======== sharedregion_ioctl_remove ========
 *  Purpose:
 *  This ioctl interface to sharedregion_remove function
 */
static int sharedregion_ioctl_remove(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_remove(cargs->cmd_arg.remove.index);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl_set_table_info ========
 *  Purpose:
 *  This ioctl interface to sharedregion_set_table_info function
 */
static int sharedregion_ioctl_set_table_info(struct sharedregion_cmd_args *args)
{
	struct sharedregion_cmd_args uarg;
	struct sharedregion_cmd_args *cargs = &uarg;
	struct sharedregion_info info;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct sharedregion_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&info, cargs->cmd_arg.set_table_info.info,
				sizeof(struct sharedregion_info));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_set_table_info(
					cargs->cmd_arg.set_table_info.index,
				cargs->cmd_arg.set_table_info.proc_id, &info);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== sharedregion_ioctl ========
 *  Purpose:
 *  This ioctl interface for sharedregion module
 */
int sharedregion_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	struct sharedregion_cmd_args __user *uarg =
				(struct sharedregion_cmd_args __user *)args;
	s32 retval = 0;

	gt_4trace(curTrace, GT_ENTER, "sharedregion_ioctl"
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
	case CMD_SHAREDREGION_GETCONFIG:
		retval = sharedregion_ioctl_get_config(uarg);
		break;

	case CMD_SHAREDREGION_SETUP:
		retval = sharedregion_ioctl_setup(uarg);
		break;

	case CMD_SHAREDREGION_DESTROY:
		retval = sharedregion_ioctl_destroy();
		break;

	case CMD_SHAREDREGION_ADD:
		retval = sharedregion_ioctl_add(uarg);
		break;

	case CMD_SHAREDREGION_GETINDEX:
		retval = sharedregion_ioctl_get_index(uarg);
		break;

	case CMD_SHAREDREGION_GETPTR:
		retval = sharedregion_ioctl_get_ptr(uarg);
		break;

	case CMD_SHAREDREGION_GETSRPTR:
		retval = sharedregion_ioctl_get_srptr(uarg);
		break;

	case CMD_SHAREDREGION_GETTABLEINFO:
		retval = sharedregion_ioctl_get_table_info(uarg);
		break;

	case CMD_SHAREDREGION_REMOVE:
		retval = sharedregion_ioctl_remove(uarg);
		break;

	case CMD_SHAREDREGION_SETTABLEINFO:
		retval = sharedregion_ioctl_set_table_info(uarg);
		break;

	default:
		WARN_ON(cmd);
		retval = -ENOTTY;
		break;
    }

exit:
	return retval;
}

