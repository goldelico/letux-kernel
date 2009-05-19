/*
 * drv_notifytesla.c
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
#include <linux/autoconf.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <linux/io.h>
#include <asm/pgtable.h>
#include <syslink/notify_tesladriver.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/GlobalTypes.h>


/* Major number of driver */
static signed long int major = 233 ;


/* driver function to open the notify mailbox driver object. */
static int drvtesla_open(struct inode *inode, struct file *filp);

/* notify mailbox driver initialization function. */
static int __init drvtesla_initialize_module(void) ;

/* notify mailbox driver cleanup function. */
static void  __exit drvtesla_finalize_module(void) ;


/* Function to invoke the APIs through ioctl. */
static struct file_operations driver_ops = {
	.open = drvtesla_open,
};

/* Initialization function */
static int __init drvtesla_initialize_module(void)
{
	int result = 0;

	result = register_chrdev(major, "notifytesladrv", &driver_ops);
	if (result < 0)
		pr_err("Notify tesla driver initialization file\n");

	return result ;
}

/* Finalization function */
static void __exit drvtesla_finalize_module(void)
{
	unregister_chrdev(major, "notifytesladrv");
}

static int drvtesla_open(struct inode *inode, struct file *filp)
{
	return 0 ;
}

MODULE_LICENSE("GPL");
module_init(drvtesla_initialize_module);
module_exit(drvtesla_finalize_module);
