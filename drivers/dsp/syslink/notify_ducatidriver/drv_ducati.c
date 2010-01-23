/*
 * drv_ducatidrv.c
 *
 * Syslink driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*  ----------------------------------- OS Specific Headers         */
#include <generated/autoconf.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>

#include <linux/io.h>
#include <asm/pgtable.h>
#include <syslink/notifydefs.h>
#include <syslink/notify_ducatidriver.h>
#include <syslink/notify_ducatidriver_defs.h>
#include <syslink/GlobalTypes.h>


/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
#define NOTIFYDUCATI_NAME "notifyducatidrv"

static char *driver_name =  NOTIFYDUCATI_NAME;

static s32 driver_major;

static s32 driver_minor;

struct notifyducati_dev {
	struct cdev cdev;
};

static struct notifyducati_dev *notifyducati_device;

static struct class *notifyducati_class;





/* driver function to open the notify mailbox driver object. */
static int drvducati_open(struct inode *inode, struct file *filp);
static int drvducati_release(struct inode *inode, struct file *filp);
static int drvducati_ioctl(struct inode *inode,
			   struct file *filp,
			   unsigned int cmd,
			   unsigned long args);


/* notify mailbox driver initialization function. */
static int __init drvducati_initialize_module(void) ;

/* notify mailbox driver cleanup function. */
static void  __exit drvducati_finalize_module(void) ;


/* Function to invoke the APIs through ioctl. */
static const struct file_operations driver_ops = {
	.open = drvducati_open,
	.release = drvducati_release,
	.ioctl = drvducati_ioctl,

};

/* Initialization function */
static int __init drvducati_initialize_module(void)
{
	int result = 0 ;
	dev_t dev;

	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		result = register_chrdev_region(dev, 1, driver_name);
	} else {
		result = alloc_chrdev_region(&dev, driver_minor, 1,
				 driver_name);
		driver_major = MAJOR(dev);
	}

	notifyducati_device = kmalloc(sizeof(struct notifyducati_dev),
					GFP_KERNEL);
	if (!notifyducati_device) {
		result = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		goto func_end;
	}
	memset(notifyducati_device, 0, sizeof(struct notifyducati_dev));
	cdev_init(&notifyducati_device->cdev, &driver_ops);
	notifyducati_device->cdev.owner = THIS_MODULE;
	notifyducati_device->cdev.ops = &driver_ops;

	result = cdev_add(&notifyducati_device->cdev, dev, 1);

	if (result) {
		printk(KERN_ERR "Failed to add the syslink notify ducati device \n");
		goto func_end;
	}

	/* udev support */
	notifyducati_class = class_create(THIS_MODULE, "syslink-notifyducati");

	if (IS_ERR(notifyducati_class)) {
		printk(KERN_ERR "Error creating notifyducati class \n");
		goto func_end;
	}
	device_create(notifyducati_class, NULL,
			MKDEV(driver_major, driver_minor), NULL,
			NOTIFYDUCATI_NAME);

func_end:
	return result ;
}

/* Finalization function */
static void __exit drvducati_finalize_module(void)
{
	dev_t dev_no;

	dev_no = MKDEV(driver_major, driver_minor);
	if (notifyducati_device) {
		cdev_del(&notifyducati_device->cdev);
		kfree(notifyducati_device);
	}
	unregister_chrdev_region(dev_no, 1);
	if (notifyducati_class) {
		/* remove the device from sysfs */
		device_destroy(notifyducati_class, MKDEV(driver_major,
						driver_minor));
		class_destroy(notifyducati_class);
	}

}
/* driver open*/
static int drvducati_open(struct inode *inode, struct file *filp)
{
	return 0 ;
}

/*drivr close*/
static int drvducati_release(struct inode *inode, struct file *filp)
{
	return 0;
}



/*
 *  brief  linux driver function to invoke the APIs through ioctl.
 *
 */
static int drvducati_ioctl(struct inode *inode,
			   struct file *filp,
			   unsigned int cmd,
			   unsigned long args)
{
	int  status = 0;
	int retval = 0;
	struct notify_ducatidrv_cmdargs *cmd_args =
		(struct notify_ducatidrv_cmdargs *) args;
	struct notify_ducatidrv_cmdargs common_args;

	switch (cmd) {
	case CMD_NOTIFYDRIVERSHM_GETCONFIG: {
		struct notify_ducatidrv_cmdargs_getconfig *src_args =
			(struct notify_ducatidrv_cmdargs_getconfig *) args;
		struct notify_ducatidrv_config cfg;
		notify_ducatidrv_getconfig(&cfg);

		retval = copy_to_user((void *)(src_args->cfg),
				      (const void *) &cfg,
				      sizeof(struct notify_ducatidrv_config));

		if (WARN_ON(retval != 0))
			goto func_end;

	}
	break;

	case CMD_NOTIFYDRIVERSHM_SETUP: {
		struct notify_ducatidrv_cmdargs_setup *src_args =
			(struct notify_ducatidrv_cmdargs_setup *) args;
		struct notify_ducatidrv_config cfg;
		retval = copy_from_user((void *) &cfg,
					(const void *)(src_args->cfg),
					sizeof(struct notify_ducatidrv_config));
		if (WARN_ON(retval != 0))
			goto func_end;

		status = notify_ducatidrv_setup(&cfg);
		if (status < 0)
			printk(KERN_ERR "FAIL: notify_ducatidrv_setup\n");
	}
	break;

	case CMD_NOTIFYDRIVERSHM_DESTROY: {
		status = notify_ducatidrv_destroy();

		if (status < 0)
			printk(KERN_ERR "FAIL: notify_ducatidrv_destroy\n");
	}
	break;

	case CMD_NOTIFYDRIVERSHM_PARAMS_INIT: {
		struct notify_ducatidrv_cmdargs_paramsinit src_args;
		struct notify_ducatidrv_params params;
		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(
				struct notify_ducatidrv_cmdargs_paramsinit));
		if (WARN_ON(retval != 0))
			goto func_end;
		notify_ducatidrv_params_init(src_args.handle, &params);

		retval = copy_to_user((void *)(src_args.params),
			      (const void *) &params,
			      sizeof(struct notify_ducatidrv_params));
		if (WARN_ON(retval != 0))
			goto func_end;
	}
	break;

	case CMD_NOTIFYDRIVERSHM_CREATE: {
		struct notify_ducatidrv_cmdargs_create src_args;
		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_ducatidrv_cmdargs_create));
		if (WARN_ON(retval != 0))
			goto func_end;
		src_args.handle = notify_ducatidrv_create(
					  src_args.driverName,
					  &(src_args.params));
		if (src_args.handle == NULL) {
			status = -EFAULT;
			printk(KERN_ERR "drvducati_ioctl:status 0x%x,"
			       "NotifyDriverShm_create failed",
			       status);
		}
		retval = copy_to_user((void *)(args),
			      (const void *) &src_args,
			      sizeof(struct notify_ducatidrv_cmdargs_create));
		if (WARN_ON(retval != 0))
			goto func_end;
	}
	break;

	case CMD_NOTIFYDRIVERSHM_DELETE: {
		struct notify_ducatidrv_cmdargs_delete src_args;
		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_ducatidrv_cmdargs_delete));
		if (WARN_ON(retval != 0))
			goto func_end;

		status = notify_ducatidrv_delete(&(src_args.handle));
		if (status < 0)
			printk(KERN_ERR "drvducati_ioctl:"
			       " notify_ducatidrv_delete failed"
			       " status = %d\n", status);
	}
	break;

	case CMD_NOTIFYDRIVERSHM_OPEN: {
		struct notify_ducatidrv_cmdargs_open src_args;

		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_ducatidrv_cmdargs_open));
		if (WARN_ON(retval != 0))
			goto func_end;

		status = notify_ducatidrv_open(src_args.driverName,
					       &(src_args.handle));
		if (status < 0)
			printk(KERN_ERR "drvducati_ioctl:"
			       " notify_ducatidrv_open failed"
			       " status = %d\n", status);
		retval = copy_to_user((void *)(args),
			      (const void *) &src_args,
			      sizeof(struct notify_ducatidrv_cmdargs_open));
		if (WARN_ON(retval != 0))
			goto func_end;
	}
	break;

	case CMD_NOTIFYDRIVERSHM_CLOSE: {
		struct notify_ducatidrv_cmdargs_close src_args;

		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_ducatidrv_cmdargs_close));
		if (WARN_ON(retval != 0))
			goto func_end;

		status = notify_ducatidrv_close(&(src_args.handle));
		if (status < 0)
			printk(KERN_ERR "drvducati_ioctl:"
			       " notify_ducatidrv_close"
			       " failed status = %d\n", status);
	}
	break;

	default: {
		status = -EINVAL;
		printk(KERN_ERR "drivducati_ioctl:Unsupported"
		       " ioctl command specified");
	}
	break;
	}

func_end:
	/* Set the status and copy the common args to user-side. */
	common_args.api_status = status;
	status = copy_to_user((void *) cmd_args,
		      (const void *) &common_args,
		      sizeof(struct notify_ducatidrv_cmdargs));

	if (status < 0)
		retval = -EFAULT;

	return retval;
}




MODULE_LICENSE("GPL");
module_init(drvducati_initialize_module);
module_exit(drvducati_finalize_module);
