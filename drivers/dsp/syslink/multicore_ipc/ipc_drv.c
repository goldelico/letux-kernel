/*
 *  ipc_drv.c
 *
 *  IPC driver module.
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

/*#ifdef MODULE*/
#include <linux/module.h>
/*#endif*/
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <ipc_ioctl.h>
#include <nameserver.h>

#define IPC_NAME  		"syslink_ipc"
#define IPC_MAJOR		0
#define IPC_MINOR		0
#define IPC_DEVICES 		1

struct ipc_device {
	struct cdev cdev;
};

struct ipc_device *ipc_device;
static struct class *ipc_class;

s32 ipc_major = IPC_MAJOR;
s32 ipc_minor = IPC_MINOR;
char *ipc_name = IPC_NAME;

module_param(ipc_name, charp, 0);
MODULE_PARM_DESC(ipc_name, "Device name, default = syslink_ipc");

module_param(ipc_major, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(ipc_major, "Major device number, default = 0 (auto)");

module_param(ipc_minor, int, 0);	/* Driver's minor number */
MODULE_PARM_DESC(ipc_minor, "Minor device number, default = 0 (auto)");

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

/*
 * ======== ipc_open ========
 *  This function is invoked when an application
 *  opens handle to the ipc driver
 */
int ipc_open(struct inode *inode, struct file *filp)
{
	s32 retval = 0;
	struct ipc_device *dev;

	dev = container_of(inode->i_cdev, struct ipc_device, cdev);
	filp->private_data = dev;
	return retval;
}

/*
 * ======== ipc_release ========
 *  This function is invoked when an application
 *  closes handle to the ipc driver
 */
int ipc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * ======== ipc_ioctl ========
 *  This function  provides IO interface  to the
 *  ipc driver
 */
int ipc_ioctl(struct inode *ip, struct file *filp, u32 cmd, ulong arg)
{
	s32 retval = 0;
	void __user *argp = (void __user *)arg;

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

	retval = ipc_ioc_router(cmd, (ulong)argp);
	return retval;

exit:
	return retval;
}

const struct file_operations ipc_fops = {
open: ipc_open,
release : ipc_release,
ioctl : ipc_ioctl,
};

/*
 * ======== ipc_modules_Init ========
 *  IPC Initialization routine. will initialize various
 *  sub components (modules) of IPC.
 */
static int ipc_modules_init(void)
{
	return 0;
}

/*
 * ======== ipc_modules_Init ========
 *  IPC cleanup routine. will cleanup of various
 *  sub components (modules) of IPC.
 */
static void ipc_modules_exit(void)
{

}

/*
 * ======== ipc_init ========
 *  Initialization routine. Executed when the driver is
 *  loaded (as a kernel module), or when the system
 *  is booted (when included as part of the kernel
 *  image).
 */
static int __init ipc_init(void)
{
	dev_t dev ;
	s32 retval = 0;

	/*  2.6 device model */
	if (ipc_major) {
		dev = MKDEV(ipc_major, ipc_minor);
		retval = register_chrdev_region(dev, IPC_DEVICES, ipc_name);
	} else {
		retval = alloc_chrdev_region(&dev, ipc_minor, IPC_DEVICES,
								ipc_name);
		ipc_major = MAJOR(dev);
	}

	if (retval < 0) {
		printk(KERN_ERR "ipc_init: can't get major %x \n", ipc_major);
		goto exit;
	}

	ipc_device = kmalloc(sizeof(struct ipc_device), GFP_KERNEL);
	if (!ipc_device) {
		printk(KERN_ERR "ipc_init: memory allocation failed for "
			"ipc_device \n");
		retval = -ENOMEM;
		goto unreg_exit;
	}

	memset(ipc_device, 0, sizeof(struct ipc_device));
	retval = ipc_modules_init();
	if (retval) {
		printk(KERN_ERR "ipc_init: ipc initialization failed \n");
		goto unreg_exit;

	}
	/* TO DO : NEED TO LOOK IN TO THIS */
	ipc_class = class_create(THIS_MODULE, "syslink_ipc");
	if (IS_ERR(ipc_class)) {
		printk(KERN_ERR "ipc_init: error creating ipc class \n");
		retval = PTR_ERR(ipc_class);
		goto unreg_exit;
	}

	device_create(ipc_class, NULL, MKDEV(ipc_major, ipc_minor), NULL,
								ipc_name);
	cdev_init(&ipc_device->cdev, &ipc_fops);
	ipc_device->cdev.owner = THIS_MODULE;
	retval = cdev_add(&ipc_device->cdev, dev, IPC_DEVICES);
	if (retval) {
		printk(KERN_ERR "ipc_init: failed to add the ipc device \n");
		goto class_exit;
	}
	return retval;

class_exit:
	class_destroy(ipc_class);

unreg_exit:
	unregister_chrdev_region(dev, IPC_DEVICES);
	kfree(ipc_device);

exit:
	return retval;
}

/*
 * ======== ipc_init ========
 *  This function is invoked during unlinking of ipc
 *  module from the kernel. ipc resources are
 *  freed in this function.
 */
static void __exit ipc_exit(void)
{
	dev_t devno;

	ipc_modules_exit();
	devno = MKDEV(ipc_major, ipc_minor);
	if (ipc_device) {
		cdev_del(&ipc_device->cdev);
		kfree(ipc_device);
	}
	unregister_chrdev_region(devno, IPC_DEVICES);
	if (ipc_class) {
		/* remove the device from sysfs */
		device_destroy(ipc_class, MKDEV(ipc_major, ipc_minor));
		class_destroy(ipc_class);
	}
}

/*
 *  ipc driver initialization and de-initialization functions
 */
module_init(ipc_init);
module_exit(ipc_exit);
