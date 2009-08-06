/*
 * procmgr_drv.c
 *
 * Syslink driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

/* Module headers */
#include <procmgr.h>
#include "procmgr_drvdefs.h"

#define PROCMGR_NAME "syslink-procmgr"

static char *driver_name = PROCMGR_NAME;

static s32 driver_major;

static s32 driver_minor;

struct procmgr_dev {
	struct cdev cdev;
};

static struct procmgr_dev *procmgr_device;

static struct class *proc_mgr_class;


/** ====================================
 *  Forward declarations of internal functions
 *  ====================================
 */
/* Linux driver function to open the driver object. */
static int proc_mgr_drv_open(struct inode *inode, struct file *filp);

/* Linux driver function to close the driver object. */
static int proc_mgr_drv_release(struct inode *inode, struct file *filp);

/* Linux driver function to invoke the APIs through ioctl. */
static int proc_mgr_drv_ioctl(struct inode *inode,
				 struct file *filp,
				 unsigned int cmd,
				 unsigned long args);

/* Linux driver function to map memory regions to user space. */
static int proc_mgr_drv_mmap(struct file *filp, struct vm_area_struct *vma);

/* Module initialization function for Linux driver. */
static int __init proc_mgr_drv_initialize_module(void);

/* Module finalization  function for Linux driver. */
static void  __exit proc_mgr_drv_finalize_module(void);

/*
 * name   DriverOps
 *
 * desc   Function to invoke the APIs through ioctl
 *
 */
static const struct file_operations procmgr_fops = {
	.open  =  proc_mgr_drv_open,
	.ioctl =  proc_mgr_drv_ioctl,
	.release = proc_mgr_drv_release,
	.mmap = proc_mgr_drv_mmap,
} ;

static struct platform_driver procmgr_driver_ldm = {
	.driver = {
		.owner = THIS_MODULE,
		.name = PROCMGR_NAME,
	},
	.probe = NULL,
	.shutdown = NULL,
	.remove = NULL,
};

/*
* brief  Linux specific function to open the driver.
 */
static int proc_mgr_drv_open(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
* brief  Linux driver function to close the driver object.
 */
static int proc_mgr_drv_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
* Linux driver function to invoke the APIs through ioctl.
 */
static int proc_mgr_drv_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args)
{
	int retval = 0;
	struct proc_mgr_cmd_args *cmd_args = (struct proc_mgr_cmd_args *)args;
	struct proc_mgr_cmd_args command_args;

	switch (cmd) {
	case CMD_PROCMGR_GETCONFIG:
	{
		struct proc_mgr_cmd_args_get_config *src_args =
			(struct proc_mgr_cmd_args_get_config *)args;
		struct proc_mgr_config cfg;

		/* copy_from_user is not needed for proc_mgr_get_config,
		* since the user's config is not used.
		*/
		proc_mgr_get_config(&cfg);

		retval = copy_to_user((void *)(src_args->cfg),
				(const void *)&cfg,
				sizeof(struct proc_mgr_config));

		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_SETUP:
	{
		struct proc_mgr_cmd_args_setup *src_args =
			(struct proc_mgr_cmd_args_setup *)args;
		struct proc_mgr_config cfg;

		retval = copy_from_user((void *)&cfg,
				(const void *)(src_args->cfg),
				sizeof(struct proc_mgr_config));

		/* This check is needed at run-time also since it
		* depends on run environment.
		* It must not be optimized out.
		*/
		if (WARN_ON(retval != 0))
			goto func_exit;

		 retval = proc_mgr_setup(&cfg);
	}
	break;

	case CMD_PROCMGR_DESTROY:
	{
		retval = proc_mgr_destroy();
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_PARAMS_INIT:
	{
		struct proc_mgr_cmd_args_params_init src_args;
		struct proc_mgr_params params;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_params_init));

		if (WARN_ON(retval != 0))
			goto func_exit;

			proc_mgr_params_init(src_args.handle, &params);

		/* Copy only the params to user-side */
		retval = copy_to_user((void *)(src_args.params),
				(const void *)&params,
				sizeof(struct proc_mgr_params));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_CREATE:
	{
		struct proc_mgr_cmd_args_create src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_create));

		src_args.handle = proc_mgr_create(src_args.proc_id,
					 &(src_args.params));
		if (src_args.handle == NULL) {
			retval = -EFAULT;
			goto func_exit;
		}
		retval = copy_to_user((void *)(args),
			(const void *)&src_args,
			sizeof(struct proc_mgr_cmd_args_create));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_DELETE:
	{
		struct proc_mgr_cmd_args_delete src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_delete));
		if (WARN_ON(retval != 0))
			goto func_exit;

		retval = proc_mgr_delete(&(src_args.handle));
	}
	break;

	case CMD_PROCMGR_OPEN:
	{
		struct proc_mgr_cmd_args_open src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_open));

		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_open(&(src_args.handle),
						src_args.proc_id);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = proc_mgr_get_proc_info(src_args.handle,
				&(src_args.proc_info));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args), (const void *)&src_args,
				sizeof(struct proc_mgr_cmd_args_open));
		WARN_ON(retval);
	}
	break;

	case CMD_PROCMGR_CLOSE:
	{
		struct proc_mgr_cmd_args_close src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_close));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_close(&(src_args.handle));
	}
	break;

	case CMD_PROCMGR_GETATTACHPARAMS:
	{
		struct proc_mgr_cmd_args_get_attach_params src_args;
		struct proc_mgr_attach_params params;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_get_attach_params));
		if (WARN_ON(retval != 0))
			goto func_exit;
		proc_mgr_get_attach_params(src_args.handle, &params);
		retval = copy_to_user((void *)(src_args.params),
				(const void *)&params,
				sizeof(struct proc_mgr_attach_params));
		WARN_ON(retval);
	}
	break;

	case CMD_PROCMGR_ATTACH:
	{
		struct proc_mgr_cmd_args_attach src_args;
		struct proc_mgr_attach_params  params;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_attach));
		if (WARN_ON(retval != 0))
			goto func_exit;
		/* Copy params from user-side. */
		retval = copy_from_user((void *)&params,
			(const void *)(src_args.params),
			sizeof(struct proc_mgr_attach_params));
		retval = proc_mgr_attach(src_args.handle, &params);
		if (WARN_ON(retval < 0))
			goto func_exit;
		/* Get memory information. */
		retval = proc_mgr_get_proc_info(src_args.handle,
					&(src_args.proc_info));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args),
			(const void *)&src_args,
			sizeof(struct proc_mgr_cmd_args_attach));
	}
	break;

	case CMD_PROCMGR_DETACH:
	{
		struct proc_mgr_cmd_args_detach src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_detach));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_detach(src_args.handle);
		if (WARN_ON(retval < 0))
			goto func_exit;
	}
	break;

	case CMD_PROCMGR_GETSTARTPARAMS:
	{
		struct proc_mgr_cmd_args_get_start_params src_args;
		struct proc_mgr_start_params params;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_get_start_params));
		if (WARN_ON(retval != 0))
			goto func_exit;
		proc_mgr_get_start_params(src_args.handle, &params);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(src_args.params),
				(const void *)&params,
				sizeof(struct proc_mgr_start_params));
		WARN_ON(retval);
	}
	break;

	case CMD_PROCMGR_START:
	{
		struct proc_mgr_cmd_args_start  src_args;
		struct proc_mgr_start_params   params;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_start));
		/* Copy params from user-side. */
		retval = copy_from_user((void *)&params,
			(const void *)(src_args.params),
			sizeof(struct proc_mgr_start_params));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_start(src_args.handle,
				src_args.entry_point, &params);

		WARN_ON(retval);
	}
	break;

	case CMD_PROCMGR_STOP:
	{
		struct proc_mgr_cmd_args_stop src_args;

		struct proc_mgr_stop_params   params;
		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_stop));
		/* Copy params from user-side. */
		retval = copy_from_user((void *)&params,
			(const void *)(src_args.params),
			sizeof(struct proc_mgr_stop_params));

		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_stop(src_args.handle, &params);
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_GETSTATE:
	{
		struct proc_mgr_cmd_args_get_state src_args;
		enum proc_mgr_state procmgrstate;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_get_state));
		if (WARN_ON(retval != 0))
			goto func_exit;
		procmgrstate = proc_mgr_get_state(src_args.handle);
		src_args.proc_mgr_state = procmgrstate;
		retval = copy_to_user((void *)(args), (const void *)&src_args,
			sizeof(struct proc_mgr_cmd_args_get_state));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_READ:
	{
		struct proc_mgr_cmd_args_read src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_read));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_read(src_args.handle,
			src_args.proc_addr, &(src_args.num_bytes),
			src_args.buffer);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args),
				(const void *)&src_args,
				sizeof(struct proc_mgr_cmd_args_read));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_WRITE:
	{
		struct proc_mgr_cmd_args_write src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_write));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_write(src_args.handle,
			src_args.proc_addr, &(src_args.num_bytes),
			src_args.buffer);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args),
			(const void *)&src_args,
			sizeof(struct proc_mgr_cmd_args_write));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_CONTROL:
	{
		struct proc_mgr_cmd_args_control src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc_mgr_cmd_args_control));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_control(src_args.handle,
			src_args.cmd, src_args.arg);
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_TRANSLATEADDR:
	{
		struct proc_mgr_cmd_args_translate_addr src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_translate_addr));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_translate_addr(src_args.handle,
			&(src_args.dst_addr), src_args.dst_addr_type,
			src_args.src_addr, src_args.src_addr_type);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args),
			(const void *)&src_args, sizeof
			(struct proc_mgr_cmd_args_translate_addr));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_MAP:
	{
		struct proc_mgr_cmd_args_map src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_map));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_map(src_args.handle,
				src_args.proc_addr, src_args.size,
				&(src_args.mapped_addr),
				&(src_args.mapped_size),
				src_args.map_attribs);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(args),
				(const void *)&src_args,
				sizeof(struct proc_mgr_cmd_args_map));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_UNMAP:
	{
		struct proc_mgr_cmd_args_unmap src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_unmap));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_unmap(src_args.handle,
						(src_args.mapped_addr));
		WARN_ON(retval < 0);
	}

	case CMD_PROCMGR_REGISTERNOTIFY:
	{
		struct proc_mgr_cmd_args_register_notify src_args;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_register_notify));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_register_notify(src_args.handle,
				src_args.callback_fxn,
				src_args.args, src_args.state);
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROCMGR_GETPROCINFO:
	{
		struct proc_mgr_cmd_args_get_proc_info src_args;
		struct proc_mgr_proc_info proc_info;

		 /* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
		(const void *)(args),
		sizeof(struct proc_mgr_cmd_args_get_proc_info));
		if (WARN_ON(retval != 0))
			goto func_exit;
		retval = proc_mgr_get_proc_info
			(src_args.handle, &proc_info);
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_to_user((void *)(src_args.proc_info),
				(const void *) &proc_info,
				sizeof(struct proc_mgr_proc_info));
		WARN_ON(retval < 0);
	}
	break;

	default:
		BUG_ON(1);
	break;
	}
func_exit:
	/* Set the retval and copy the common args to user-side. */
	command_args.api_status = retval;
	retval = copy_to_user((void *)cmd_args,
		(const void *)&command_args, sizeof(struct proc_mgr_cmd_args));

	WARN_ON(retval < 0);
	return retval;
}


/*
  Driver function to map memory regions to user space.
 */
static int proc_mgr_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma,
			 vma->vm_start,
			 vma->vm_pgoff,
			 vma->vm_end - vma->vm_start,
			 vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

/*
* Module initialization  function for Linux driver.
 */
static int __init proc_mgr_drv_initialize_module(void)
{
	dev_t dev = 0 ;
	int retval;

	/* Display the version info and created date/time */
	printk(KERN_INFO "proc_mgr_drv_initialize_module\n");

	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		retval = register_chrdev_region(dev, 1, driver_name);
	} else {
		retval = alloc_chrdev_region(&dev, driver_minor, 1,
				 driver_name);
		driver_major = MAJOR(dev);
	}

	procmgr_device = kmalloc(sizeof(struct procmgr_dev), GFP_KERNEL);
	if (!procmgr_device) {
		retval = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		goto exit;
	}
	memset(procmgr_device, 0, sizeof(struct procmgr_dev));
	cdev_init(&procmgr_device->cdev, &procmgr_fops);
	procmgr_device->cdev.owner = THIS_MODULE;
	procmgr_device->cdev.ops = &procmgr_fops;

	retval = cdev_add(&procmgr_device->cdev, dev, 1);

	if (retval) {
		printk(KERN_ERR "Failed to add the syslink procmgr device \n");
		goto exit;
	}

	/* udev support */
	proc_mgr_class = class_create(THIS_MODULE, "syslink-procmgr");

	if (IS_ERR(proc_mgr_class)) {
		printk(KERN_ERR "Error creating bridge class \n");
		goto exit;
	}
	device_create(proc_mgr_class, NULL, MKDEV(driver_major, driver_minor),
			NULL, PROCMGR_NAME);

	retval = platform_driver_register(&procmgr_driver_ldm);
exit:
	return retval;
}


/*
* driver function to finalize the driver module.
 */
static void __exit proc_mgr_drv_finalize_module(void)
{
	dev_t devno = 0;

	platform_driver_unregister(&procmgr_driver_ldm);
	devno = MKDEV(driver_major, driver_minor);
	if (procmgr_device) {
		cdev_del(&procmgr_device->cdev);
		kfree(procmgr_device);
	}
	unregister_chrdev_region(devno, 1);
	if (proc_mgr_class) {
		/* remove the device from sysfs */
		device_destroy(proc_mgr_class, MKDEV(driver_major,
						driver_minor));
		class_destroy(proc_mgr_class);
	}
}

/*
* brief  Macro calls that indicate initialization and finalization functions
 *	to the kernel.
 */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mugdha Kamoolkar");
module_init(proc_mgr_drv_initialize_module);
module_exit(proc_mgr_drv_finalize_module);
