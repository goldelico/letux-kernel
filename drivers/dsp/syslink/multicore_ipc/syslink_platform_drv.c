/*
 *  syslink_platform_drv.c
 *
 *  SYSLINK_PLATFORM driver module.
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <multiproc_ioctl.h>

#define SYSLINK_PLATFORM_NAME		"syslink_platform"
#define SYSLINK_PLATFORM_MAJOR		0
#define SYSLINK_PLATFORM_MINOR		0
#define SYSLINK_PLATFORM_DEVICES		1

struct syslink_platform_device {
	struct cdev cdev;
};

struct syslink_platform_device *syslink_platform_device;
static struct class *syslink_platform_class;

s32 syslink_platform_major = SYSLINK_PLATFORM_MAJOR;
s32 syslink_platform_minor = SYSLINK_PLATFORM_MINOR;
char *syslink_platform_name = SYSLINK_PLATFORM_NAME;

module_param(syslink_platform_name, charp, 0);
MODULE_PARM_DESC(syslink_platform_name,
		"Device name, default = syslink_platform");

module_param(syslink_platform_major, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(syslink_platform_major,
		"Major device number, default = 0 (auto)");

module_param(syslink_platform_minor, int, 0);	/* Driver's minor number */
MODULE_PARM_DESC(syslink_platform_minor,
		"Minor device number, default = 0 (auto)");

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL v2");

/*
 * ======== syslink_platform_open ========
 *  This function is invoked when an application
 *  opens handle to the syslink_platform driver
 */
int syslink_platform_open(struct inode *inode, struct file *filp)
{
	s32 retval = 0;
	struct syslink_platform_device *dev;
	dev = container_of(inode->i_cdev, struct syslink_platform_device, cdev);
	filp->private_data = dev;
	return retval;
}

/*
 * ======== syslink_platform_release ========
 *  This function is invoked when an application
 *  closes handle to the syslink_platform driver
 */
int syslink_platform_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * ======== syslink_platform_ioctl ========
 *  This function  provides IO interface  to the
 *  syslink_platform driver
 */
int syslink_platform_ioctl(struct inode *ip,
				struct file *filp,
				u32 cmd,
				ulong arg)
{
	s32 retval = 0;
	void __user *argp = (void __user *)arg;
	u32 ioc_nr;

	/* Verify the memory and ensure that it is not is kernel
	     address space
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, argp, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, argp, _IOC_SIZE(cmd));

	if (retval) {
		retval = -EFAULT;
		goto exit;
	}

	ioc_nr = _IOC_NR(cmd);
	if (ioc_nr >= MULTIPROC_BASE_CMD && ioc_nr <= MULTIPROC_END_CMD)
		retval = multiproc_ioctl(NULL, NULL, cmd, arg);
	return retval;

exit:
	return retval;
}

const struct file_operations syslink_platform_fops = {
	.open = syslink_platform_open,
	.release = syslink_platform_release,
	.ioctl = syslink_platform_ioctl,
};

/*
 * ======== syslink_platform_init ========
 *  Initialization routine. Executed when the driver is
 *  loaded (as a kernel module), or when the system
 *  is booted (when included as part of the kernel
 *  image).
 */
static int __init syslink_platform_init(void)
{
	dev_t dev ;
	s32 retval = 0;

	retval = alloc_chrdev_region(&dev, syslink_platform_minor,
					SYSLINK_PLATFORM_DEVICES,
					syslink_platform_name);
	syslink_platform_major = MAJOR(dev);

	if (retval < 0) {
		printk(KERN_ERR "syslink_platform_init: can't get major %x\n",
			syslink_platform_major);
		goto exit;
	}

	syslink_platform_device =
		kmalloc(sizeof(struct syslink_platform_device), GFP_KERNEL);
	if (!syslink_platform_device) {
		printk(KERN_ERR \
			"syslink_platform_init: memory allocation failed for "
			"syslink_platform_device\n");
		retval = -ENOMEM;
		goto unreg_exit;
	}

	memset(syslink_platform_device, 0,
		sizeof(struct syslink_platform_device));

	syslink_platform_class = class_create(THIS_MODULE,
						"syslink_syslink_platform");
	if (IS_ERR(syslink_platform_class)) {
		printk(KERN_ERR "syslink_platform_init: "
			"error creating class\n");
		retval = PTR_ERR(syslink_platform_class);
		goto unreg_exit;
	}

	device_create(syslink_platform_class, NULL,
			MKDEV(syslink_platform_major, syslink_platform_minor),
			NULL, syslink_platform_name);
	cdev_init(&syslink_platform_device->cdev, &syslink_platform_fops);
	syslink_platform_device->cdev.owner = THIS_MODULE;
	retval = cdev_add(&syslink_platform_device->cdev, dev,
				SYSLINK_PLATFORM_DEVICES);
	if (retval) {
		printk(KERN_ERR "syslink_platform_init: "
			"failed to add the device\n");
		goto class_exit;
	}
	return retval;

class_exit:
	class_destroy(syslink_platform_class);

unreg_exit:
	unregister_chrdev_region(dev, SYSLINK_PLATFORM_DEVICES);
	kfree(syslink_platform_device);

exit:
	return retval;
}

/*
 * ======== syslink_platform_exit ========
 *  This function is invoked during unlinking of syslink_platform
 *  module from the kernel. syslink_platform resources are
 *  freed in this function.
 */
static void __exit syslink_platform_exit(void)
{
	dev_t devno;

	devno = MKDEV(syslink_platform_major, syslink_platform_minor);
	if (syslink_platform_device) {
		cdev_del(&syslink_platform_device->cdev);
		kfree(syslink_platform_device);
	}
	unregister_chrdev_region(devno, SYSLINK_PLATFORM_DEVICES);
	if (syslink_platform_class) {
		/* remove the device from sysfs */
		device_destroy(syslink_platform_class,
				MKDEV(syslink_platform_major,
				syslink_platform_minor));
		class_destroy(syslink_platform_class);
	}
}

/*
 *  syslink_platform driver initialization and de-initialization functions
 */
module_init(syslink_platform_init);
module_exit(syslink_platform_exit);
