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

#include <gt.h>
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
	s32 retval = 0;

	block = heapbuf_alloc(cargs->args.alloc.handle,
				cargs->args.alloc.size,
				cargs->args.alloc.align);
	if (block != NULL) {
		index = sharedregion_get_index(block);
		if (index < 0) {
			retval = index;
			goto exit;
		}

		block_srptr = sharedregion_get_srptr(block, index);
	}
	/* The error on above fn will be a null ptr. We are not
	checking that condition here. We are passing whatever
	we are getting from the heapbuf module. So IOCTL will succed,
	but the actual fn might be failed inside heapbuf
	*/
	cargs->args.alloc.block_srptr = block_srptr;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_free ========
 *  Purpose:
 *  This ioctl interface to heapbuf_free function
 */
static int heapbuf_ioctl_free(struct heapbuf_cmd_args *cargs)
{
	char *block;
	s32 retval = 0;

	block = sharedregion_get_ptr(cargs->args.free.block_srptr);
	retval = heapbuf_free(cargs->args.free.handle, block,
					cargs->args.free.size);
	return retval;
}

/*
 * ======== heapbuf_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to heapbuf_params_init function
 */
static int heapbuf_ioctl_params_init(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 retval = 0;
	u32 size;

	heapbuf_params_init(cargs->args.params_init.handle, &params);
	size = copy_to_user(cargs->args.params_init.params, &params,
				sizeof(struct heapbuf_params));
	if (size)
		retval = -EFAULT;

	return retval;
}

/*
 * ======== heapbuf_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to heapbuf_create function
 */
static int heapbuf_ioctl_create(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 retval = 0;
	u32 size;
	void *handle = NULL;

	size = copy_from_user(&params, cargs->args.create.params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->args.create.name_len >= 0) {
		params.name = kmalloc(cargs->args.create.name_len + 1,
							GFP_KERNEL);
		if (params.name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}

		params.name[cargs->args.create.name_len] = '\0';
		size = copy_from_user(params.name,
					cargs->args.create.params->name,
					cargs->args.create.name_len);
		if (size) {
			retval = -EFAULT;
			goto name_from_usr_error;
		}
	}

	params.shared_addr = sharedregion_get_ptr((u32 *)
				cargs->args.create.params->shared_addr);
	handle = heapbuf_create(&params);
	cargs->args.create.handle = handle;

name_from_usr_error:
	if (cargs->args.open.name_len > 0)
		kfree(params.name);

exit:
	return retval;
}


/*
 * ======== heapbuf_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to heapbuf_delete function
 */
static int heapbuf_ioctl_delete(struct heapbuf_cmd_args *cargs)
{
	s32 retval = 0;

	retval = heapbuf_delete(cargs->args.delete.handle);
	return retval;
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
	s32 retval = 0;
	ulong size;

	size = copy_from_user(&params, cargs->args.open.params,
						sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->args.open.name_len >= 0) {
		params.name = kmalloc(cargs->args.open.name_len + 1,
							GFP_KERNEL);
		if (params.name == NULL) {
			retval = -ENOMEM;
			goto exit;
		}

		params.name[cargs->args.create.name_len] = '\0';
		size = copy_from_user(params.name,
					cargs->args.open.params->name,
					cargs->args.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto free_name;
		}
	}

	params.shared_addr = sharedregion_get_ptr((u32 *)
					cargs->args.open.params->shared_addr);
	retval = heapbuf_open(&handle, &params);
	if (retval)
		goto free_name;

	cargs->args.open.handle = handle;
	size = copy_to_user(cargs->args.open.params, &params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
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
	return retval;
}


/*
 * ======== heapbuf_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to heapbuf_close function
 */
static int heapbuf_ioctl_close(struct heapbuf_cmd_args *cargs)
{
	s32 retval = 0;

	retval = heapbuf_close(cargs->args.close.handle);
	return retval;
}

/*
 * ======== heapbuf_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to heapbuf_shared_memreq function
 */
static int heapbuf_ioctl_shared_memreq(struct heapbuf_cmd_args *cargs)
{
	struct heapbuf_params params;
	s32 retval = 0;
	ulong size;
	u32 bytes;

	size = copy_from_user(&params, cargs->args.shared_memreq.params,
						sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	bytes = heapbuf_shared_memreq(&params);
	cargs->args.shared_memreq.bytes = bytes;

exit:
	return retval;
}


/*
 * ======== heapbuf_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_config function
 */
static int heapbuf_ioctl_get_config(struct heapbuf_cmd_args *cargs)
{
	struct heap_config config;
	s32 retval = 0;
	ulong size;

	retval = heapbuf_get_config(&config);
	size = copy_to_user(cargs->args.get_config.config, &config,
						sizeof(struct heap_config));
	if (size)
		retval = -EFAULT;

	return retval;
}

/*
 * ======== heapbuf_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to heapbuf_setup function
 */
static int heapbuf_ioctl_setup(struct heapbuf_cmd_args *cargs)
{
	struct heap_config config;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(&config, cargs->args.setup.config,
						sizeof(struct heap_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_setup(&config);

exit:
	return retval;
}
/*
 * ======== heapbuf_ioctl_destroy ========
 *  Purpose:
 *  This ioctl interface to heapbuf_destroy function
 */
static int heapbuf_ioctl_destroy()
{
	s32 retval = 0;
	retval = heapbuf_destroy();
	return retval;
}


/*
 * ======== heapbuf_ioctl_get_stats ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_stats function
 */
static int heapbuf_ioctl_get_stats(struct heapbuf_cmd_args *cargs)
{
	struct memory_stats stats;
	s32 retval = 0;
	ulong size;


	retval = heapbuf_get_stats(cargs->args.get_stats.handle, &stats);
	if (retval)
		goto exit;

	size = copy_to_user(cargs->args.get_stats.stats, &stats,
					sizeof(struct memory_stats));
	if (size)
		retval = -EFAULT;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_get_extended_stats ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_extended_stats function
 */
static int heapbuf_ioctl_get_extended_stats(struct heapbuf_cmd_args *cargs)
{
	struct heap_extended_stats stats;
	s32 retval = 0;
	ulong size;

	retval = heapbuf_get_extended_stats(
			cargs->args.get_extended_stats.handle, &stats);
	if (retval)
		goto exit;

	size = copy_to_user(cargs->args.get_extended_stats.stats, &stats,
				sizeof(struct heap_extended_stats));
	if (size)
		retval = -EFAULT;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl ========
 *  Purpose:
 *  This ioctl interface for heapbuf module
 */
int heapbuf_ioctl(struct inode *pinode, struct file *filp,
			unsigned int cmd, unsigned long  args)
{
	s32 os_status = 0;
	s32 size = 0;
	struct heapbuf_cmd_args __user *uarg =
				(struct heapbuf_cmd_args __user *)args;
	struct heapbuf_cmd_args cargs;

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
					sizeof(struct heapbuf_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CMD_HEAPBUF_ALLOC:
		os_status = heapbuf_ioctl_alloc(uarg);
		break;

	case CMD_HEAPBUF_FREE:
		os_status = heapbuf_ioctl_free(uarg);
		break;

	case CMD_HEAPBUF_PARAMS_INIT:
		os_status = heapbuf_ioctl_params_init(uarg);
		break;

	case CMD_HEAPBUF_CREATE:
		os_status = heapbuf_ioctl_create(uarg);
		break;

	case CMD_HEAPBUF_DELETE:
		os_status  = heapbuf_ioctl_delete(uarg);
		break;

	case CMD_HEAPBUF_OPEN:
		os_status  = heapbuf_ioctl_open(uarg);
		break;

	case CMD_HEAPBUF_CLOSE:
		os_status = heapbuf_ioctl_close(uarg);
		break;

	case CMD_HEAPBUF_SHAREDMEMREQ:
		os_status = heapbuf_ioctl_shared_memreq(uarg);
		break;

	case CMD_HEAPBUF_GETCONFIG:
		os_status = heapbuf_ioctl_get_config(uarg);
		break;

	case CMD_HEAPBUF_SETUP:
		os_status = heapbuf_ioctl_setup(uarg);
		break;

	case CMD_HEAPBUF_DESTROY:
		os_status = heapbuf_ioctl_destroy();
		break;

	case CMD_HEAPBUF_GETSTATS:
		os_status = heapbuf_ioctl_get_stats(uarg);
		break;

	case CMD_HEAPBUF_GETEXTENDEDSTATS:
		os_status = heapbuf_ioctl_get_extended_stats(uarg);
		break;

	default:
		WARN_ON(cmd);
		os_status = -ENOTTY;
		break;
	}

	/* Copy the full args to the user-side. */
	size = copy_to_user(uarg, &cargs,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		os_status = -EFAULT;
		goto exit;
	}

exit:
	return os_status;

}

