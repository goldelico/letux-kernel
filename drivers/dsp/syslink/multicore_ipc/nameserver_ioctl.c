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
#include <linux/mm.h>
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
static int nameserver_ioctl_setup(
			struct nameserver_cmd_args *cargs)
{
	cargs->api_status = nameserver_setup();
	return 0;
}

/*
 * ======== nameserver_ioctl_destroy ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  destroy nameserver module
 */
static int nameserver_ioctl_destroy(
			struct nameserver_cmd_args *cargs)
{
	cargs->api_status = nameserver_destroy();
	return 0;
}

/*
 * ======== nameserver_ioctl_params_init ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  get the default configuration of a nameserver instance
 */
static int nameserver_ioctl_params_init(struct nameserver_cmd_args *cargs)
{
	struct nameserver_params params;
	s32 status = 0;
	ulong size;

	cargs->api_status = nameserver_params_init(&params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct nameserver_params));
	if (size)
		status = -EFAULT;

	return status;
}

/*
 * ======== nameserver_ioctl_get_handle ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  get the handle of a nameserver instance from name
 */
static int nameserver_ioctl_get_handle(struct nameserver_cmd_args *cargs)
{
	void *handle = NULL;
	char *name = NULL;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.get_handle.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.get_handle.name,
				cargs->args.get_handle.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	handle = nameserver_get_handle(name);
	cargs->args.get_handle.handle = handle;
	cargs->api_status = 0;

name_from_usr_error:
	kfree(name);

exit:
	return status;
}

/*
 * ======== nameserver_ioctl_create ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  create a name server instance
 */
static int nameserver_ioctl_create(struct nameserver_cmd_args *cargs)
{
	struct nameserver_params params;
	void *handle = NULL;
	char *name = NULL;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.create.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.create.name,
				cargs->args.create.name_len);
	if (size) {
		status = -EFAULT;
		goto copy_from_usr_error;
	}

	size = copy_from_user(&params, cargs->args.create.params,
				sizeof(struct nameserver_params));
	if (size) {
		status = -EFAULT;
		goto copy_from_usr_error;
	}

	handle = nameserver_create(name, &params);
	cargs->args.create.handle = handle;
	cargs->api_status = 0;

copy_from_usr_error:
	kfree(name);

exit:
	return status;
}


/*
 * ======== nameserver_ioctl_delete ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  delete a name server instance
 */
static int nameserver_ioctl_delete(struct nameserver_cmd_args *cargs)
{
	cargs->api_status =
		nameserver_delete(&cargs->args.delete_instance.handle);
	return 0;
}

/*
 * ======== nameserver_ioctl_add ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  add an entry into a nameserver instance
 */
static int nameserver_ioctl_add(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	char *buf = NULL;
	void *entry;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.add.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.add.name,
				cargs->args.add.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	buf = kmalloc(cargs->args.add.len, GFP_KERNEL);
	if (buf == NULL) {
		status = -ENOMEM;
		goto buf_alloc_error;
	}

	size = copy_from_user(buf, cargs->args.add.buf,
					cargs->args.add.len);
	if (size) {
		status = -EFAULT;
		goto buf_from_usr_error;
	}

	entry = nameserver_add(cargs->args.add.handle, name, buf,
						cargs->args.add.len);
	cargs->args.add.entry = entry;
	cargs->args.add.node = entry;
	cargs->api_status = 0;

buf_from_usr_error:
	kfree(buf);

buf_alloc_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return status;
}


/*
 * ======== nameserver_ioctl_add_uint32 ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  add a Uint32 entry into a nameserver instance
 */
static int nameserver_ioctl_add_uint32(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	void *entry;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.addu32.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.addu32.name,
				cargs->args.addu32.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	entry = nameserver_add_uint32(cargs->args.addu32.handle, name,
						cargs->args.addu32.value);
	cargs->args.addu32.entry = entry;
	cargs->api_status = 0;

name_from_usr_error:
	kfree(name);

exit:
	return status;
}


/*
 * ======== nameserver_ioctl_match ========
 *  Purpose:
 *  This wrapper function will call the nameserver function
 *  to retrieve the value portion of a name/value
 *  pair from local table
 */
static int nameserver_ioctl_match(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	u32 buf;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.match.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.match.name,
				cargs->args.match.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	cargs->api_status =
		nameserver_match(cargs->args.match.handle, name, &buf);
	size = copy_to_user(cargs->args.match.value, &buf, sizeof(u32 *));
	if (size) {
		status = -EFAULT;
		goto buf_to_usr_error;
	}

buf_to_usr_error: /* Fall through */
name_from_usr_error:
	kfree(name);

exit:
	return status;
}

/*
 * ======== nameserver_ioctl_remove ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  remove a name/value pair from a name server
 */
static int nameserver_ioctl_remove(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.remove.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	size = copy_from_user(name, cargs->args.remove.name,
				cargs->args.remove.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	cargs->api_status =
		nameserver_remove(cargs->args.remove.handle, name);

name_from_usr_error:
	kfree(name);

exit:
	return status;
}


/*
 * ======== nameserver_ioctl_remove_entry ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  remove an entry from a name server
 */
static int nameserver_ioctl_remove_entry(struct nameserver_cmd_args *cargs)
{
	cargs->api_status = nameserver_remove_entry(
						cargs->args.remove_entry.handle,
						cargs->args.remove_entry.entry);
	return 0;
}

/*
 * ======== nameserver_ioctl_get_local ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  retrieve the value portion of a name/value pair from local table
 */
static int nameserver_ioctl_get_local(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	char *buf = NULL;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.get_local.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	buf = kmalloc(cargs->args.get_local.len, GFP_KERNEL);
	if (buf == NULL) {
		status = -ENOMEM;
		goto buf_alloc_error;
	}

	size = copy_from_user(name, cargs->args.get_local.name,
				cargs->args.get_local.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	cargs->api_status = nameserver_get_local(
					cargs->args.get_local.handle, name,
					buf, cargs->args.get_local.len);
	size = copy_to_user(cargs->args.get_local.buf, buf,
				cargs->args.get_local.len);
	if (size)
		status = -EFAULT;

name_from_usr_error:
	kfree(buf);

buf_alloc_error:
	kfree(name);

exit:
	return status;
}


/*
 * ======== nameserver_ioctl_get ========
 *  Purpose:
 *  This wrapper function will call the nameserver function to
 *  retrieve the value portion of a name/value pair from table
 */
static int nameserver_ioctl_get(struct nameserver_cmd_args *cargs)
{
	char *name = NULL;
	char *buf = NULL;
	u16 *proc_id = NULL;
	s32 status = 0;
	ulong size;

	name = kmalloc(cargs->args.get.name_len + 1, GFP_KERNEL);
	if (name == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	name[cargs->args.get_handle.name_len] = '\0';
	buf = kmalloc(cargs->args.get.len, GFP_KERNEL);
	if (buf == NULL) {
		status = -ENOMEM;
		goto buf_alloc_error;
	}

	proc_id = kmalloc(cargs->args.get.proc_len, GFP_KERNEL);
	if (proc_id == NULL) {
		status = -ENOMEM;
		goto proc_alloc_error;
	}

	size = copy_from_user(name, cargs->args.get.name,
				cargs->args.get.name_len);
	if (size) {
		status = -EFAULT;
		goto name_from_usr_error;
	}

	status = copy_from_user(proc_id, cargs->args.get.proc_id,
					cargs->args.get.proc_len);
	if (size) {
		status = -EFAULT;
		goto proc_from_usr_error;
	}

	cargs->api_status = nameserver_get(cargs->args.get.handle, name, buf,
					cargs->args.get.len, proc_id);
	size = copy_to_user(cargs->args.get.buf, buf,
				cargs->args.get.len);
	if (size)
		status = -EFAULT;


proc_from_usr_error: /* Fall through */
name_from_usr_error:
	kfree(proc_id);

proc_alloc_error:
	kfree(buf);

buf_alloc_error:
	kfree(name);

exit:
	return status;
}

/*
 * ======== nameserver_ioctl ========
 *  Purpose:
 *  This ioctl interface for nameserver module
 */
int nameserver_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	s32 status = 0;
	s32 size = 0;
	struct nameserver_cmd_args __user *uarg =
				(struct nameserver_cmd_args __user *)args;
	struct nameserver_cmd_args cargs;


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
					sizeof(struct nameserver_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_NAMESERVER_ADD:
		status = nameserver_ioctl_add(&cargs);
		break;

	case CMD_NAMESERVER_ADDUINT32:
		status = nameserver_ioctl_add_uint32(&cargs);
		break;

	case CMD_NAMESERVER_GET:
		status = nameserver_ioctl_get(&cargs);
		break;

	case CMD_NAMESERVER_GETLOCAL:
		status = nameserver_ioctl_get_local(&cargs);
		break;

	case CMD_NAMESERVER_MATCH:
		status = nameserver_ioctl_match(&cargs);
		break;

	case CMD_NAMESERVER_REMOVE:
		status = nameserver_ioctl_remove(&cargs);
		break;

	case CMD_NAMESERVER_REMOVEENTRY:
		status = nameserver_ioctl_remove_entry(&cargs);
		break;

	case CMD_NAMESERVER_PARAMS_INIT:
		status = nameserver_ioctl_params_init(&cargs);
		break;

	case CMD_NAMESERVER_CREATE:
		status = nameserver_ioctl_create(&cargs);
		break;

	case CMD_NAMESERVER_DELETE:
		status = nameserver_ioctl_delete(&cargs);
		break;

	case CMD_NAMESERVER_GETHANDLE:
		status = nameserver_ioctl_get_handle(&cargs);
		break;

	case CMD_NAMESERVER_SETUP:
		status = nameserver_ioctl_setup(&cargs);
		break;

	case CMD_NAMESERVER_DESTROY:
		status = nameserver_ioctl_destroy(&cargs);
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
	size = copy_to_user(uarg, &cargs, sizeof(struct nameserver_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

exit:
	return status;
}

