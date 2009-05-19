/*
* nameserver_ioctl.c
*
* This provides the ioctl interface for nameserver module
*
* Copyright (C) 2008-2009 Texas Instruments, Inc.
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
* PURPOSE.
*/
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/fs.h>

#include <gt.h>
#include <nameserver.h>
#include <nameserver_ioctl.h>

/*
 *  FUNCTIONS NEED TO BE REVIEWED OPTIMIZED!
 */

/*
 * ======== nameserver_ioctl_setup ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  setup nameserver module
 */
static int nameserver_ioctl_setup(void)
{
	s32 retval = 0;
	retval = nameserver_setup();
	return retval;
}

/*
 * ======== nameserver_ioctl_destroy ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  destroy nameserver module
 */
static int nameserver_ioctl_destroy(void)
{
	s32 retval = 0;
	retval = nameserver_destroy();
	return retval;
}

/*
 * ======== nameserver_ioctl_params_init ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  get the default configuration of a nameserver instance
 */
static int nameserver_ioctl_params_init(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	struct nameserver_params params;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = nameserver_params_init(&params);
	size = copy_to_user(cargs->cmd_arg.params_init.params, &params,
				sizeof(struct nameserver_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_get_handle ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  get the handle of a nameserver instance from name
 */
static int nameserver_ioctl_get_handle(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	void *handle = NULL;
	char *name = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.get_handle.name_len, GFP_KERNEL);
	if (name == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.get_handle.name,
				cargs->cmd_arg.get_handle.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	handle = nameserver_get_handle(name);
	size = copy_to_user(&args->cmd_arg.get_handle.handle, &handle,
							sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto handle_to_usr_error;
	}

	kfree(name);
	return 0;

handle_to_usr_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_create ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  create a name server instance
 */
static int nameserver_ioctl_create(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	struct nameserver_params params;
	void *handle = NULL;
	char *name = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.create.name_len, GFP_KERNEL);
	if (name == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.create.name,
				cargs->cmd_arg.create.name_len);
	if (size) {
		retval = -EFAULT;
		goto copy_from_usr_error;
	}

	size = copy_from_user(&params, cargs->cmd_arg.create.params,
				sizeof(struct nameserver_params));
	if (size) {
		retval = -EFAULT;
		goto copy_from_usr_error;
	}

	handle = nameserver_create(name, &params);
	size = copy_to_user(&args->cmd_arg.create.handle, &handle,
							sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto copy_to_usr_error;
	}

	kfree(name);
	return 0;

copy_to_usr_error:
	if (handle)
		nameserver_delete(handle);

copy_from_usr_error:
	kfree(name);

exit:
	return retval;
}


/*
 * ======== nameserver_ioctl_delete ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  delete a name server instance
 */
static int nameserver_ioctl_delete(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = nameserver_delete(&cargs->cmd_arg.delete_instance.handle);
	if (retval)
		goto exit;

	size = copy_to_user(&args->cmd_arg.delete_instance.handle,
			&cargs->cmd_arg.delete_instance.handle,	sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_add ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  add an entry into a nameserver instance
 */
static int nameserver_ioctl_add(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	char *buf = NULL;
	void *entry;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.add.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.add.name,
				cargs->cmd_arg.add.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	buf = kmalloc(cargs->cmd_arg.add.len, GFP_KERNEL);
	if (buf == NULL) {
		retval = -ENOMEM;
		goto buf_alloc_error;
	}

	size = copy_from_user(buf, cargs->cmd_arg.add.buf,
					cargs->cmd_arg.add.len);
	if (size) {
		retval = -EFAULT;
		goto buf_from_usr_error;
	}

	entry = nameserver_add(cargs->cmd_arg.add.handle, name, buf,
						cargs->cmd_arg.add.len);
	size = copy_to_user(cargs->cmd_arg.add.entry, &entry, sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto entry_to_usr_error;
	}

	kfree(name);
	kfree(buf);
	return 0;

entry_to_usr_error:
	if (entry)
		nameserver_remove(entry, name);

buf_from_usr_error:
	kfree(buf);

buf_alloc_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return retval;
}


/*
 * ======== nameserver_ioctl_add_uint32 ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  add a Uint32 entry into a nameserver instance
 */
static int nameserver_ioctl_add_uint32(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	void *entry;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.addu32.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.addu32.name,
				cargs->cmd_arg.addu32.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	entry = nameserver_add_uint32(cargs->cmd_arg.addu32.handle, name,
						cargs->cmd_arg.addu32.value);
	size = copy_to_user(cargs->cmd_arg.addu32.entry, &entry,
						sizeof(void *));
	if (size) {
		retval = -EFAULT;
		goto entry_to_usr_error;
	}

	kfree(name);
	return 0;

entry_to_usr_error:
	if (entry)
		nameserver_remove(entry, name);

name_from_usr_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_match ========
 *  Purpose:
 *  This wrapper function will call the nameserver function
 *  to retrieve the value portion of a name/value
 *  pair from local table
 */
static int nameserver_ioctl_match(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	u32 buf;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.match.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.match.name,
				cargs->cmd_arg.match.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	retval = nameserver_match(cargs->cmd_arg.match.handle, name, &buf);
	size = copy_to_user(cargs->cmd_arg.match.value, &buf, sizeof(u32 *));
	if (size) {
		retval = -EFAULT;
		goto buf_to_usr_error;
	}

	kfree(name);
	return 0;

buf_to_usr_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_remove ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  remove a name/value pair from a name server
 */
static int nameserver_ioctl_remove(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.remove.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	size = copy_from_user(name, cargs->cmd_arg.remove.name,
				cargs->cmd_arg.remove.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	retval = nameserver_remove(cargs->cmd_arg.remove.handle, name);
	kfree(name);
	return 0;

name_from_usr_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_remove_entry ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  remove an entry from a name server
 */
static int nameserver_ioctl_remove_entry(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = nameserver_remove_entry(cargs->cmd_arg.remove_entry.handle,
					cargs->cmd_arg.remove_entry.entry);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_get_local ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  retrieve the value portion of a name/value pair from local table
 */
static int nameserver_ioctl_get_local(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	char *buf = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.get_local.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	buf = kmalloc(cargs->cmd_arg.get_local.len, GFP_KERNEL);
	if (buf) {
		retval = -ENOMEM;
		goto buf_alloc_error;
	}

	size = copy_from_user(name, cargs->cmd_arg.get_local.name,
				cargs->cmd_arg.get_local.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	retval = nameserver_get_local(cargs->cmd_arg.get_local.handle, name,
					buf, cargs->cmd_arg.get_local.len);
	size = copy_to_user(cargs->cmd_arg.get_local.buf, buf,
				cargs->cmd_arg.get_local.len);
	if (size) {
		retval = -EFAULT;
		goto buf_to_usr_error;
	}

	kfree(name);
	kfree(buf);
	return 0;

buf_to_usr_error: /* Fall through */
name_from_usr_error:
	kfree(buf);

buf_alloc_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl_get ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  retrieve the value portion of a name/value pair from table
 */
static int nameserver_ioctl_get(struct nameserver_cmd_args *args)
{
	struct nameserver_cmd_args uarg;
	struct nameserver_cmd_args *cargs = &uarg;
	char *name = NULL;
	char *buf = NULL;
	u16 *proc_id = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct nameserver_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	name = kmalloc(cargs->cmd_arg.get.name_len, GFP_KERNEL);
	if (name) {
		retval = -ENOMEM;
		goto exit;
	}

	buf = kmalloc(cargs->cmd_arg.get.len, GFP_KERNEL);
	if (buf) {
		retval = -ENOMEM;
		goto buf_alloc_error;
	}

	proc_id = kmalloc(cargs->cmd_arg.get.proc_len, GFP_KERNEL);
	if (proc_id) {
		retval = -ENOMEM;
		goto proc_alloc_error;
	}

	size = copy_from_user(name, cargs->cmd_arg.get.name,
				cargs->cmd_arg.get.name_len);
	if (size) {
		retval = -EFAULT;
		goto name_from_usr_error;
	}

	retval = copy_from_user(proc_id, cargs->cmd_arg.get.proc_id,
					cargs->cmd_arg.get.proc_len);
	if (size) {
		retval = -EFAULT;
		goto proc_from_usr_error;
	}

	retval = nameserver_get(cargs->cmd_arg.get.handle, name, buf,
					cargs->cmd_arg.get.len, proc_id);
	size = copy_to_user(cargs->cmd_arg.get.buf, buf,
				cargs->cmd_arg.get.len);
	if (size) {
		retval = -EFAULT;
		goto buf_to_usr_error;
	}

	kfree(name);
	kfree(buf);
	kfree(proc_id);
	return 0;

buf_to_usr_error: /* Fall through */
proc_from_usr_error: /* Fall through */
name_from_usr_error:
	kfree(proc_id);

proc_alloc_error:
	kfree(buf);

buf_alloc_error:
	kfree(name);

exit:
	return retval;
}

/*
 * ======== nameserver_ioctl ========
 *  Purpose:
 *  This ioctl interface for nameserver module
 */
int nameserver_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	struct nameserver_cmd_args __user *uarg =
				(struct nameserver_cmd_args __user *)args;
	s32 retval = 0;

	gt_4trace(curTrace, GT_ENTER, "nameserver_ioctl"
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
	case CMD_NAMESERVER_ADD:
		retval = nameserver_ioctl_add(uarg);
		break;

	case CMD_NAMESERVER_ADDUINT32:
		retval = nameserver_ioctl_add_uint32(uarg);
		break;

	case CMD_NAMESERVER_GET:
		retval = nameserver_ioctl_get(uarg);
		break;

	case CMD_NAMESERVER_GETLOCAL:
		retval = nameserver_ioctl_get_local(uarg);
		break;

	case CMD_NAMESERVER_MATCH:
		retval = nameserver_ioctl_match(uarg);
		break;

	case CMD_NAMESERVER_REMOVE:
		retval = nameserver_ioctl_remove(uarg);
		break;

	case CMD_NAMESERVER_REMOVEENTRY:
		retval = nameserver_ioctl_remove_entry(uarg);
		break;

	case CMD_NAMESERVER_PARAMS_INIT:
		retval = nameserver_ioctl_params_init(uarg);
		break;

	case CMD_NAMESERVER_CREATE:
		retval = nameserver_ioctl_create(uarg);
		break;

	case CMD_NAMESERVER_DELETE:
		retval = nameserver_ioctl_delete(uarg);
		break;

	case CMD_NAMESERVER_GETHANDLE:
		retval = nameserver_ioctl_get_handle(uarg);
		break;

	case CMD_NAMESERVER_SETUP:
		retval = nameserver_ioctl_setup();
		break;

	case CMD_NAMESERVER_DESTROY:
		retval = nameserver_ioctl_destroy();
		break;

	default:
		WARN_ON(cmd);
		retval = -ENOTTY;
		break;
	}

exit:
	return retval;
}

