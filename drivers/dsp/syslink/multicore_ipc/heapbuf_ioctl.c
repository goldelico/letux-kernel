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
static int heapbuf_ioctl_alloc(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	u32 *block_srptr = SHAREDREGION_INVALIDSRPTR;
	void *block;
	s32 index;
	s32 retval = 0;
	u32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	block = heapbuf_alloc(cargs->cmd_arg.alloc.handle,
				cargs->cmd_arg.alloc.size,
				cargs->cmd_arg.alloc.align);
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
	size = copy_to_user(&args->cmd_arg.alloc.block_srptr,
				&block_srptr, sizeof(u32 *));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;
exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_free ========
 *  Purpose:
 *  This ioctl interface to heapbuf_free function
 */
static int heapbuf_ioctl_free(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	char *block;
	s32 retval = 0;
	s32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	block = sharedregion_get_ptr(cargs->cmd_arg.free.block_srptr);
	retval = heapbuf_free(cargs->cmd_arg.free.handle, block,
					cargs->cmd_arg.free.size);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_params_init ========
 *  Purpose:
 *  This ioctl interface to heapbuf_params_init function
 */
static int heapbuf_ioctl_params_init(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heapbuf_params params;
	s32 retval = 0;
	u32 size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	heapbuf_params_init(cargs->cmd_arg.params_init.handle, &params);
	size = copy_to_user(cargs->cmd_arg.params_init.params, &params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;
exit:
	return retval;

}

/*
 * ======== heapbuf_ioctl_create ========
 *  Purpose:
 *  This ioctl interface to heapbuf_create function
 */
static int heapbuf_ioctl_create(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heapbuf_params params;
	s32 retval = 0;
	u32 size;
	void *handle = NULL;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.create.params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->cmd_arg.create.name_len >= 0) {
		params.name = kmalloc(cargs->cmd_arg.create.name_len,
							GFP_KERNEL);
		if (params.name) {
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

	params.shared_addr = sharedregion_get_ptr((u32 *)
				cargs->cmd_arg.create.params->shared_addr);
	handle = heapbuf_create(&params);
	size = copy_to_user(&args->cmd_arg.create.handle,
				&handle, sizeof(u32 *));
	if (size) {
		retval = -EFAULT;
		goto handle_to_usr_error;
	}

	if (cargs->cmd_arg.create.name_len >= 0)
		kfree(params.name);

	return 0;

handle_to_usr_error:
	if (handle)
		heapbuf_delete(&handle);

name_from_usr_error:
	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

exit:
	return retval;
}


/*
 * ======== heapbuf_ioctl_delete ========
 *  Purpose:
 *  This ioctl interface to heapbuf_delete function
 */
static int heapbuf_ioctl_delete(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_delete(cargs->cmd_arg.delete.handle);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_open ========
 *  Purpose:
 *  This ioctl interface to heapbuf_open function
 */
static int heapbuf_ioctl_open(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heapbuf_params params;
	void *handle = NULL;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.open.params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	if (cargs->cmd_arg.open.name_len >= 0) {
		params.name = kmalloc(cargs->cmd_arg.open.name_len, GFP_KERNEL);
		if (params.name) {
			retval = -ENOMEM;
			goto exit;
		}

		size = copy_from_user(params.name,
					cargs->cmd_arg.open.params->name,
					cargs->cmd_arg.open.name_len);
		if (size) {
			retval = -EFAULT;
			goto copy_from_usr_error;
		}
	}

	params.shared_addr = sharedregion_get_ptr((u32 *)
				cargs->cmd_arg.open.params->shared_addr);
	retval = heapbuf_open(&handle, &params);
	if (retval)
		goto heap_open_error;

	size = copy_to_user(cargs->cmd_arg.open.params, &params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto copy_to_usr_error;
	}

	size = copy_to_user(&args->cmd_arg.open.handle, &handle,
						sizeof(void *));

	if (size) {
		retval = -EFAULT;
		goto copy_to_usr_error;
	}

	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

	return 0;

copy_to_usr_error:
	heapbuf_close(handle);

heap_open_error: /* Fall through */
copy_from_usr_error:
	if (cargs->cmd_arg.open.name_len > 0)
		kfree(params.name);

exit:
	return retval;
}


/*
 * ======== heapbuf_ioctl_close ========
 *  Purpose:
 *  This ioctl interface to heapbuf_close function
 */
static int heapbuf_ioctl_close(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_close(cargs->cmd_arg.close.handle);
	if (retval)
		goto exit;

	return 0;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_shared_memreq ========
 *  Purpose:
 *  This ioctl interface to heapbuf_shared_memreq function
 */
static int heapbuf_ioctl_shared_memreq(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heapbuf_params params;
	s32 retval = 0;
	ulong size;
	u32 bytes;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&params, cargs->cmd_arg.shared_memreq.params,
				sizeof(struct heapbuf_params));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	bytes = heapbuf_shared_memreq(&params);
	size = copy_to_user(&args->cmd_arg.shared_memreq.bytes,
				&bytes, sizeof(u32));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}


/*
 * ======== heapbuf_ioctl_get_config ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_config function
 */
static int heapbuf_ioctl_get_config(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heap_config config;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_get_config(&config);
	size = copy_to_user(cargs->cmd_arg.get_config.config, &config,
					sizeof(struct heap_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_setup ========
 *  Purpose:
 *  This ioctl interface to heapbuf_setup function
 */
static int heapbuf_ioctl_setup(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heap_config config;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	size = copy_from_user(&config, cargs->cmd_arg.setup.config,
					sizeof(struct heap_config));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_setup(&config);
	if (retval)
		goto exit;

	return 0;

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
static int heapbuf_ioctl_get_stats(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct memory_stats stats;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_get_stats(cargs->cmd_arg.get_stats.handle, &stats);
	if (retval)
		goto exit;

	size = copy_to_user(cargs->cmd_arg.get_stats.stats, &stats,
				sizeof(struct memory_stats));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return retval;
}

/*
 * ======== heapbuf_ioctl_get_extended_stats ========
 *  Purpose:
 *  This ioctl interface to heapbuf_get_extended_stats function
 */
static int heapbuf_ioctl_get_extended_stats(struct heapbuf_cmd_args *args)
{
	struct heapbuf_cmd_args uarg;
	struct heapbuf_cmd_args *cargs = &uarg;
	struct heap_extended_stats stats;
	s32 retval = 0;
	ulong size;

	size = copy_from_user(cargs, args,
				sizeof(struct heapbuf_cmd_args));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	retval = heapbuf_get_extended_stats(
			cargs->cmd_arg.get_extended_stats.handle, &stats);
	if (retval)
		goto exit;
	size = copy_to_user(cargs->cmd_arg.get_extended_stats.stats, &stats,
				sizeof(struct heap_extended_stats));
	if (size) {
		retval = -EFAULT;
		goto exit;
	}

	return 0;

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
	struct heapbuf_cmd_args __user *uarg =
				(struct heapbuf_cmd_args __user *)args;
	s32 retval = 0;

	gt_4trace(curTrace, GT_ENTER, "heapbuf_ioctl"
		"pinode: %x, filp: %x,\n cmd: %x, args: %x",
		pinode, filp, cmd, args);

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));

	if (retval) {
		retval = -EFAULT;
		goto exit;
	}


	switch (cmd) {
	case CMD_HEAPBUF_ALLOC:
		retval = heapbuf_ioctl_alloc(uarg);
		break;

	case CMD_HEAPBUF_FREE:
		retval = heapbuf_ioctl_free(uarg);
		break;

	case CMD_HEAPBUF_PARAMS_INIT:
		retval = heapbuf_ioctl_params_init(uarg);
		break;

	case CMD_HEAPBUF_CREATE:
		retval = heapbuf_ioctl_create(uarg);
		break;

	case CMD_HEAPBUF_DELETE:
		retval  = heapbuf_ioctl_delete(uarg);
		break;

	case CMD_HEAPBUF_OPEN:
		retval  = heapbuf_ioctl_open(uarg);
		break;

	case CMD_HEAPBUF_CLOSE:
		retval = heapbuf_ioctl_close(uarg);
		break;

	case CMD_HEAPBUF_SHAREDMEMREQ:
		retval = heapbuf_ioctl_shared_memreq(uarg);
		break;

	case CMD_HEAPBUF_GETCONFIG:
		retval = heapbuf_ioctl_get_config(uarg);
		break;

	case CMD_HEAPBUF_SETUP:
		retval = heapbuf_ioctl_setup(uarg);
		break;

	case CMD_HEAPBUF_DESTROY:
		retval = heapbuf_ioctl_destroy();
		break;

	case CMD_HEAPBUF_GETSTATS:
		retval = heapbuf_ioctl_get_stats(uarg);
		break;

	case CMD_HEAPBUF_GETEXTENDEDSTATS:
		retval = heapbuf_ioctl_get_extended_stats(uarg);
		break;

	default:
		WARN_ON(cmd);
		retval = -ENOTTY;
	}

exit:
	return retval;
}

