/*
 *  heapbuf_ioctl.c
 *
 *  Heap module manages fixed size buffers that can be used
 *  in a multiprocessor system with shared memory.
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
#include <heap.h>
#include <heapbuf.h>
#include <heapbuf_ioctl.h>
#include <sharedregion.h>

/*
 * ======== heapbuf_ioctl_alloc ========
 *  Purpose:
 *  This ioctl interface to heapbuf_alloc function
 */
static int heapbuf_ioctl_alloc(struct heapbuf_cmd_args *cargs)
{
	u32 *block_srptr = SHAREDREGION_INVALIDSRPTR;
	void *block;
	s32 index;
	s32 status = 0;

	block = heapbuf_alloc(cargs->args.alloc.handle,
				cargs->args.alloc.size,
				cargs->args.alloc.align);
	if (block != NULL) {
		index = sharedregion_get_index(block);
		block_srptr = sharedregion_get_srptr(block, index);
	}
	/* The error on above fn will be a null ptr. We are not
	checking that condition here. We are passing whatever
	we are getting from the heapbuf module. So IOCTL will succed,
	but the actual fn might be failed inside heapbuf
	*/
	cargs->args.alloc.block_srptr = block_srptr;
	cargs->api_status = 0;
	return status;
}

/*
 * ======== heapbuf_ioctl_free ========
 *  Purpose:
 *  This ioctl interface to heapbuf_free function
 */
static int heapbuf_ioctl_free(struct heapbuf_cmd_args *cargs)
{
	char *block;

	block = sharedregion_get_ptr(cargs->args.free.block_srptr);
	cargs->api_status  = heapbuf_free(cargs->args.free.handle, block,
					cargs->args.free.size);
	return 0;
}

/*
 * ======== heapbuf_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to heapbuf_params_init function
 */
static int heapbuf_ioctl_params_init(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 status = 0;
	u32 size;

	heapbuf_params_init(cargs->args.params_init.handle, &params);
	cargs->api_status = 0;
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct heapbuf_params));
	if (size)
		status = -EFAULT;

	return status;
}

/*
 * ======== heapbuf_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to heapbuf_create function
 */
static int heapbuf_ioctl_create(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 status = 0;
	u32 size;
	void *handle = NULL;

	size = copy_from_user(&params, cargs->args.create.params,
				sizeof(struct heapbuf_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	if (cargs->args.create.name_len > 0) {
		params.name = kmalloc(cargs->args.create.name_len, GFP_KERNEL);
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

	params.shared_addr = sharedregion_get_ptr((u32 *)
				cargs->args.create.shared_addr_srptr);
	params.shared_buf = sharedregion_get_ptr((u32 *)
				cargs->args.create.shared_buf_srptr);
	params.gate = cargs->args.create.knl_gate;
	handle = heapbuf_create(&params);
	cargs->args.create.handle = handle;
	cargs->api_status  = 0;

name_from_usr_error:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

exit:
	return status;
}


/*
 * ======== heapbuf_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to heapbuf_delete function
 */
static int heapbuf_ioctl_delete(struct heapbuf_cmd_args *cargs)
{
	cargs->api_status = heapbuf_delete(&cargs->args.delete.handle);
	return 0;
}

/*
 * ======== heapbuf_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to heapbuf_open function
 */
static int heapbuf_ioctl_open(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	void *handle = NULL;
	s32 status = 0;
	ulong size;

	size = copy_from_user(&params, cargs->args.open.params,
						sizeof(struct heapbuf_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	if (cargs->args.open.name_len > 0) {
		params.name = kmalloc(cargs->args.open.name_len, GFP_KERNEL);
		if (params.name == NULL) {
			status = -ENOMEM;
			goto exit;
		}

		params.name[cargs->args.create.name_len] = '\0';
		size = copy_from_user(params.name,
					cargs->args.open.params->name,
					cargs->args.open.name_len);
		if (size) {
			status = -EFAULT;
			goto free_name;
		}
	}

	/* For open by name, the shared_add_srptr may be invalid */
	if (cargs->args.open.shared_addr_srptr != SHAREDREGION_INVALIDSRPTR) {
		params.shared_addr = sharedregion_get_ptr(
				(u32 *)cargs->args.open.shared_addr_srptr);
	}
	params.gate = cargs->args.open.knl_gate;

	cargs->api_status = heapbuf_open(&handle, &params);
	if (cargs->api_status < 0)
		goto free_name;

	cargs->args.open.handle = handle;
	size = copy_to_user(cargs->args.open.params, &params,
				sizeof(struct heapbuf_params));
	if (size) {
		status = -EFAULT;
		goto copy_to_usr_error;
	}

	goto free_name;

copy_to_usr_error:
	if (handle) {
		heapbuf_close(handle);
		cargs->args.open.handle = NULL;
	}

free_name:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

exit:
	return status;
}


/*
 * ======== heapbuf_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to heapbuf_close function
 */
static int heapbuf_ioctl_close(struct heapbuf_cmd_args *cargs)
{
	cargs->api_status = heapbuf_close(cargs->args.close.handle);
	return 0;
}

/*
 * ======== heapbuf_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to heapbuf_shared_memreq function
 */
static int heapbuf_ioctl_shared_memreq(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 status = 0;
	ulong size;
	u32 bytes;

	size = copy_from_user(&params, cargs->args.shared_memreq.params,
						sizeof(struct heapbuf_params));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	bytes = heapbuf_shared_memreq(&params,
					&cargs->args.shared_memreq.buf_size);
	cargs->args.shared_memreq.bytes = bytes;
	cargs->api_status = 0;

exit:
	return status;
}


/*
 * ======== heapbuf_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_config function
 */
static int heapbuf_ioctl_get_config(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_config config;
	s32 status = 0;
	ulong size;

	cargs->api_status = heapbuf_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
						sizeof(struct heapbuf_config));
	if (size)
		status = -EFAULT;

	return status;
}

/*
 * ======== heapbuf_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to heapbuf_setup function
 */
static int heapbuf_ioctl_setup(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_config config;
	s32 status = 0;
	ulong size;

	size = copy_from_user(&config, cargs->args.setup.config,
						sizeof(struct heapbuf_config));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	cargs->api_status = heapbuf_setup(&config);

exit:
	return status;
}
/*
 * ======== heapbuf_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to heapbuf_destroy function
 */
static int heapbuf_ioctl_destroy(struct heapbuf_cmd_args *cargs)
{
	cargs->api_status = heapbuf_destroy();
	return 0;
}


/*
 * ======== heapbuf_ioctl_get_stats ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_stats function
 */
static int heapbuf_ioctl_get_stats(struct heapbuf_cmd_args *cargs)
{
	struct memory_stats stats;
	s32 status = 0;
	ulong size;

	cargs->api_status = heapbuf_get_stats(cargs->args.get_stats.handle,
						&stats);
	if (status)
		goto exit;

	size = copy_to_user(cargs->args.get_stats.stats, &stats,
					sizeof(struct memory_stats));
	if (size)
		status = -EFAULT;

exit:
	return status;
}

/*
 * ======== heapbuf_ioctl_get_extended_stats ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_extended_stats function
 */
static int heapbuf_ioctl_get_extended_stats(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_extended_stats stats;
	s32 status = 0;
	ulong size;

	cargs->api_status = heapbuf_get_extended_stats(
			cargs->args.get_extended_stats.handle, &stats);
	if (cargs->api_status != 0)
		goto exit;

	size = copy_to_user(cargs->args.get_extended_stats.stats, &stats,
				sizeof(struct heapbuf_extended_stats));
	if (size)
		status = -EFAULT;

exit:
	return status;
}

/*
 * ======== heapbuf_ioctl ========
 *  Purpose:
 *  This ioctl interface for heapbuf module
 */
int heapbuf_ioctl(struct inode *pinode, struct file *filp,
			unsigned int cmd, unsigned long  args)
{
	s32 status = 0;
	s32 size = 0;
	struct heapbuf_cmd_args __user *uarg =
				(struct heapbuf_cmd_args __user *)args;
	struct heapbuf_cmd_args cargs;

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
					sizeof(struct heapbuf_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_HEAPBUF_ALLOC:
		status = heapbuf_ioctl_alloc(&cargs);
		break;

	case CMD_HEAPBUF_FREE:
		status = heapbuf_ioctl_free(&cargs);
		break;

	case CMD_HEAPBUF_PARAMS_INIT:
		status = heapbuf_ioctl_params_init(&cargs);
		break;

	case CMD_HEAPBUF_CREATE:
		status = heapbuf_ioctl_create(&cargs);
		break;

	case CMD_HEAPBUF_DELETE:
		status  = heapbuf_ioctl_delete(&cargs);
		break;

	case CMD_HEAPBUF_OPEN:
		status  = heapbuf_ioctl_open(&cargs);
		break;

	case CMD_HEAPBUF_CLOSE:
		status = heapbuf_ioctl_close(&cargs);
		break;

	case CMD_HEAPBUF_SHAREDMEMREQ:
		status = heapbuf_ioctl_shared_memreq(&cargs);
		break;

	case CMD_HEAPBUF_GETCONFIG:
		status = heapbuf_ioctl_get_config(&cargs);
		break;

	case CMD_HEAPBUF_SETUP:
		status = heapbuf_ioctl_setup(&cargs);
		break;

	case CMD_HEAPBUF_DESTROY:
		status = heapbuf_ioctl_destroy(&cargs);
		break;

	case CMD_HEAPBUF_GETSTATS:
		status = heapbuf_ioctl_get_stats(&cargs);
		break;

	case CMD_HEAPBUF_GETEXTENDEDSTATS:
		status = heapbuf_ioctl_get_extended_stats(&cargs);
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
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		status = -EFAULT;
		goto exit;
	}

exit:
	return status;

}

