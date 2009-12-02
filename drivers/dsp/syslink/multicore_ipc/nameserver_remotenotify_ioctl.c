/*
 *  nameserver_remotenotify_ioctl.h
 *
 *  The nameserver_remotenotify module provides functionality to get name
 *  value pair from a remote nameserver.
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
#include <sharedregion.h>
#include <nameserver_remotenotify_ioctl.h>

/*
 * ======== nameserver_remotenotify_ioctl_get ======
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_get function
 */
static int nameserver_remotenotify_ioctl_get(
		struct nameserver_remotenotify_cmd_args *cargs)
{
	s32 status = 0;
	ulong size;
	char *instance_name = NULL;
	char *name = NULL;
	u8 *value = NULL;

	BUG_ON(cargs->args.get.instance_name_len == 0);
	if (cargs->args.get.instance_name_len) {
		instance_name = kmalloc(cargs->args.get.instance_name_len + 1,
								GFP_KERNEL);
		if (instance_name == NULL) {
			status = ENOMEM;
			goto exit;
		}

		instance_name[cargs->args.get.instance_name_len] = '\0';
		size = copy_from_user(instance_name,
					cargs->args.get.instance_name,
					cargs->args.get.instance_name_len);
		if (size) {
			status = -ENOMEM;
			goto exit;
		}
	}

	if (cargs->args.get.name_len) {
		name = kmalloc(cargs->args.get.name_len + 1,
						GFP_KERNEL);
		if (name == NULL) {
			status = ENOMEM;
			goto exit;
		}

		instance_name[cargs->args.get.instance_name_len] = '\0';
		size = copy_from_user(name, cargs->args.get.name,
						cargs->args.get.name_len);
		if (size) {
			status = -ENOMEM;
			goto exit;
		}
	}

	/* Allocate memory for the value */
	if (cargs->args.get.value_len >= 0) {
		value = kmalloc(cargs->args.get.value_len, GFP_KERNEL);
		size = copy_from_user(value, cargs->args.get.value,
					cargs->args.get.value_len);
		if (size) {
			status = -ENOMEM;
			goto exit;
		}
	}

	cargs->args.get.len = nameserver_remotenotify_get(
						cargs->args.get.handle,
						instance_name,
						name,
						value,
						cargs->args.get.value_len,
						cargs->args.get.reserved);
	cargs->api_status = 0;

exit:
	kfree(value);
	kfree(name);
	kfree(instance_name);
	return status;
}

/*
 * ======== nameserver_remotenotify_ioctl_shared_memreq ======
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_shared_memreq function
 */
static int nameserver_remotenotify_ioctl_shared_memreq(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	struct nameserver_remotenotify_params params;
	s32 status = 0;
	ulong size;

    /* params may be NULL. */
    if (cargs->args.shared_memreq.params != NULL) {
		size = copy_from_user(&params,
			      cargs->args.shared_memreq.params,
			      sizeof(struct nameserver_remotenotify_params));
		if (size) {
			status = -EFAULT;
			goto exit;
		}
    }

	cargs->args.shared_memreq.shared_mem_size =
			nameserver_remotenotify_shared_memreq(&params);
	cargs->api_status = 0;

exit:
	return status;
}

/*
 * ======== nameserver_remotenotify_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_params_init function
 */
static int nameserver_remotenotify_ioctl_params_init(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	struct nameserver_remotenotify_params params;
	s32 status = 0;
	ulong size;

	nameserver_remotenotify_params_init(cargs->args.params_init.handle,
					&params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct nameserver_remotenotify_params));
	if (size)
		status = -EFAULT;

	cargs->api_status = 0;
	return status;
}

/*
 * ======== nameserver_remotenotify_ioctl_create========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_create function
 */
static int nameserver_remotenotify_ioctl_create(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	struct nameserver_remotenotify_params params;
	s32 status = 0;
	ulong size;
	size = copy_from_user(&params, cargs->args.create.params,
			sizeof(struct nameserver_remotenotify_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	params.shared_addr = sharedregion_get_ptr((u32 *)
				cargs->args.create.params->shared_addr);
	cargs->args.create.handle = nameserver_remotenotify_create(
						cargs->args.create.proc_id,
						&params);
	cargs->api_status = 0;
exit:
	return status;
}

/*
 * ======== nameserver_remotenotify_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_delete function
 */
static int nameserver_remotenotify_ioctl_delete(
		struct nameserver_remotenotify_cmd_args *cargs)
{
	cargs->api_status = nameserver_remotenotify_delete(
					&cargs->args.delete_instance.handle);
	return 0;
}

/*
 * ======== nameserver_remotenotify_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_get_config function
 */
static int nameserver_remotenotify_ioctl_get_config(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	s32 status = 0;
	ulong size;
	struct nameserver_remotenotify_config config;

	nameserver_remotenotify_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
				sizeof(struct nameserver_remotenotify_config));
	if (size)
		status  = -EFAULT;

	cargs->api_status = 0;
	return status;
}

/*
 * ======== nameserver_remotenotify_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_setup function
 */
static int nameserver_remotenotify_ioctl_setup(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	struct nameserver_remotenotify_config config;
	s32 status = 0;
	ulong size;

	size = copy_from_user(&config, cargs->args.setup.config,
			sizeof(struct nameserver_remotenotify_config));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	cargs->api_status = nameserver_remotenotify_setup(&config);
exit:
	return status;
}


/*
 * ======== nameserver_remotenotify_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to nameserver_remotenotify_destroy function
 */
static int nameserver_remotenotify_ioctl_destroy(
			struct nameserver_remotenotify_cmd_args *cargs)
{
	cargs->api_status = nameserver_remotenotify_destroy();
	return 0;
}

/*
 * ======== nameserver_remotenotify_ioctl ========
 *  Purpose:
 *  This ioctl interface for nameserver_remotenotify module
 */
int nameserver_remotenotify_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args)
{
	s32 status = 0;
	s32 size = 0;
	struct nameserver_remotenotify_cmd_args __user *uarg =
			(struct nameserver_remotenotify_cmd_args __user *)args;
	struct nameserver_remotenotify_cmd_args cargs;

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
			sizeof(struct nameserver_remotenotify_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_NAMESERVERREMOTENOTIFY_GET:
		status = nameserver_remotenotify_ioctl_get(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_SHAREDMEMREQ:
		status = nameserver_remotenotify_ioctl_shared_memreq(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_PARAMS_INIT:
		status = nameserver_remotenotify_ioctl_params_init(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_CREATE:
		status = nameserver_remotenotify_ioctl_create(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_DELETE:
		status = nameserver_remotenotify_ioctl_delete(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_GETCONFIG:
		status = nameserver_remotenotify_ioctl_get_config(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_SETUP:
		status = nameserver_remotenotify_ioctl_setup(&cargs);
		break;

	case CMD_NAMESERVERREMOTENOTIFY_DESTROY:
		status = nameserver_remotenotify_ioctl_destroy(&cargs);
		break;

	default:
		WARN_ON(cmd);
		status = -ENOTTY;
		break;
	}

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs,
			sizeof(struct nameserver_remotenotify_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

exit:
	return status;
}

