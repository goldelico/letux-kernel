/*
*  multiproc_ioctl.c
*
*  This provides the ioctl interface for multiproc module
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
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>

#include <gt.h>
#include <multiproc.h>
#include <multiproc_ioctl.h>


/*
 * ======== mproc_ioctl_get_id ========
 *  Purpose:
 *  This wrapper function will call the multproc function
 *  to get proccesor Id from proccessor name
 */
static int mproc_ioctl_get_id(struct multiproc_cmd_args *args)
{
	struct multiproc_cmd_args uarg;
	struct multiproc_cmd_args *cargs = &uarg;
	char *name = NULL;
	s32 retval = 0;
	ulong size = 0;
	u16 proc_id;

	size = copy_from_user(cargs, args,
				sizeof(struct multiproc_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.get_id.name_len, GFP_KERNEL);
	if (name == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.get_id.name,
				cargs->cmd_arg.get_id.name_len);
	/* Handle partial copy */
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}
	proc_id = multiproc_get_id(name);
	size = copy_to_user(cargs->cmd_arg.get_id.proc_id,
						&proc_id, sizeof(u16));
	if (size) {
		retval = -EFAULT;
		goto proc_id_to_usr_error;
	}

	kfree(name);
	return 0;

proc_id_to_usr_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== mproc_ioctl_get_name ========
 *  Purpose:
 *  This wrapper function will call the multproc function
 *  to get get name from processor id
 */
static int mproc_ioctl_get_name(struct multiproc_cmd_args *args)
{
	struct multiproc_cmd_args uarg;
	struct multiproc_cmd_args *cargs = &uarg;
	s32 retval = 0;
	char *name = NULL;
	ulong size = 0;

	size = copy_from_user(cargs, args,
				sizeof(struct multiproc_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = multiproc_get_name(cargs->cmd_arg.get_name.proc_id);

	size = copy_to_user(cargs->cmd_arg.get_name.name, name,
						strlen(name));
	/* Handle partial copy */
	if (size) {
		retval = -EFAULT;
		goto exit;
	}
	return 0;

exit:
	return retval;

}

/* ======== mproc_ioctl_get_max_processors ========
 *  Purpose:
 *  This wrapper function will call the multproc function
 *  to get maximum proc Id in the system.
 */
static int mproc_ioctl_get_max_processors(struct multiproc_cmd_args *args)
{
	struct multiproc_cmd_args uarg;
	struct multiproc_cmd_args *cargs = &uarg;
	u16 max_id;
	s32 size = 0;
	s32 retval = 0;

	size = copy_from_user(cargs, args,
				sizeof(struct multiproc_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	max_id = multiproc_get_max_processors();
	size = copy_to_user(cargs->cmd_arg.get_max_id.max_id,
						&max_id, sizeof(u16));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

exit:
	return retval;
}


/*
 * ======== multiproc_ioctl ========
 *  Purpose:
 *  This ioctl interface for multiproc module
 */
int multiproc_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	struct multiproc_cmd_args __user *uarg =
				(struct multiproc_cmd_args __user *)args;
	s32 retval = 0;

	gt_4trace(curTrace, GT_ENTER, "multiproc_ioctl"
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
	case CMD_MULTIPROC_GETID:
		retval = mproc_ioctl_get_id(uarg);
		break;

	case CMD_MULTIPROC_GETNAME:
		retval = mproc_ioctl_get_name(uarg);
		break;

	case CMD_MULTIPROC_GETMAXID:
		retval = mproc_ioctl_get_max_processors(uarg);
		break;

	default:
		WARN_ON(cmd);
		retval = -ENOTTY;
		break;
	}

exit:
	return retval;
}

