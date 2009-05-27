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
static int sharedregion_ioctl_get_config(struct sharedregion_cmd_args *cargs)
{

	struct sharedregion_config config;
	s32 retval = 0;
	s32 size;

	retval = sharedregion_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct sharedregion_config));
	if (size)
		retval = -EFAULT;

	return retval;
}


/*
 * ======== sharedregion_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to sharedregion_setup function
 */
static int sharedregion_ioctl_setup(struct sharedregion_cmd_args *cargs)
{
	struct sharedregion_config config;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(&config, cargs->args.setup.config,
				sizeof(struct sharedregion_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_setup(&config);

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
static int sharedregion_ioctl_add(struct sharedregion_cmd_args *cargs)
{
	s32 retval = 0;

	retval =  sharedregion_add(cargs->args.add.index,
				cargs->args.add.base,
				cargs->args.add.len);
	return retval;
}



/*
 * ======== sharedregion_ioctl_get_index ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_index function
 */
static int sharedregion_ioctl_get_index(struct sharedregion_cmd_args *cargs)
{
	s32 retval = 0;
	s32 index = 0;

	index = sharedregion_get_index(cargs->args.get_index.addr);
	cargs->args.get_index.index = index;
	if (index < 0)
		retval = index;

	return retval;
}

/*
 * ======== sharedregion_ioctl_get_ptr ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_ptr function
 */
static int sharedregion_ioctl_get_ptr(struct sharedregion_cmd_args *cargs)
{
	s32 retval = 0;
	void *addr = NULL;

	addr = sharedregion_get_ptr(cargs->args.get_ptr.srptr);
	/* We are not checking the return from the module, its user
	responsibilty to pass proper value to application
	*/
	cargs->args.get_ptr.addr = addr;
	return retval;
}

/*
 * ======== sharedregion_ioctl_get_srptr ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_srptr function
 */
static int sharedregion_ioctl_get_srptr(struct sharedregion_cmd_args *cargs)
{
	s32 retval = 0;
	u32 *srptr = NULL;

	srptr = sharedregion_get_srptr(cargs->args.get_srptr.addr,
					cargs->args.get_srptr.index);
	/* We are not checking the return from the module, its user
	responsibilty to pass proper value to application
	*/
	cargs->args.get_srptr.srptr = srptr;
	return retval;
}

/*
 * ======== sharedregion_ioctl_get_table_info ========
 *  Purpose:
 *  This ioctl interface to sharedregion_get_table_info function
 */
static int sharedregion_ioctl_get_table_info(
				struct sharedregion_cmd_args *cargs)
{
	struct sharedregion_info info;
	s32 retval = 0;
	s32 size;

	retval = sharedregion_get_table_info(
				cargs->args.get_table_info.index,
				cargs->args.get_table_info.proc_id, &info);
	if (retval)
		goto exit;

	size = copy_to_user(cargs->args.get_table_info.info, &info,
				sizeof(struct sharedregion_info));
	if (size)
		retval = -EFAULT;

exit:
	return retval;
}


/*
 * ======== sharedregion_ioctl_remove ========
 *  Purpose:
 *  This ioctl interface to sharedregion_remove function
 */
static int sharedregion_ioctl_remove(struct sharedregion_cmd_args *cargs)
{
	s32 retval = 0;

	retval = sharedregion_remove(cargs->args.remove.index);
	return retval;
}

/*
 * ======== sharedregion_ioctl_set_table_info ========
 *  Purpose:
 *  This ioctl interface to sharedregion_set_table_info function
 */
static int sharedregion_ioctl_set_table_info(
			struct sharedregion_cmd_args *cargs)
{
	struct sharedregion_info info;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(&info, cargs->args.set_table_info.info,
				sizeof(struct sharedregion_info));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = sharedregion_set_table_info(
				cargs->args.set_table_info.index,
				cargs->args.set_table_info.proc_id, &info);

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
	s32 os_status = 0;
	s32 size = 0;
	struct sharedregion_cmd_args __user *uarg =
				(struct sharedregion_cmd_args __user *)args;
	struct sharedregion_cmd_args cargs;

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
					sizeof(struct sharedregion_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_SHAREDREGION_GETCONFIG:
		os_status = sharedregion_ioctl_get_config(&cargs);
		break;

	case CMD_SHAREDREGION_SETUP:
		os_status = sharedregion_ioctl_setup(&cargs);
		break;

	case CMD_SHAREDREGION_DESTROY:
		os_status = sharedregion_ioctl_destroy();
		break;

	case CMD_SHAREDREGION_ADD:
		os_status = sharedregion_ioctl_add(&cargs);
		break;

	case CMD_SHAREDREGION_GETINDEX:
		os_status = sharedregion_ioctl_get_index(&cargs);
		break;

	case CMD_SHAREDREGION_GETPTR:
		os_status = sharedregion_ioctl_get_ptr(&cargs);
		break;

	case CMD_SHAREDREGION_GETSRPTR:
		os_status = sharedregion_ioctl_get_srptr(&cargs);
		break;

	case CMD_SHAREDREGION_GETTABLEINFO:
		os_status = sharedregion_ioctl_get_table_info(&cargs);
		break;

	case CMD_SHAREDREGION_REMOVE:
		os_status = sharedregion_ioctl_remove(&cargs);
		break;

	case CMD_SHAREDREGION_SETTABLEINFO:
		os_status = sharedregion_ioctl_set_table_info(&cargs);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
    }


	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs, sizeof(struct sharedregion_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

exit:
	return os_status;
}

