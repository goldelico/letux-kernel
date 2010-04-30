/*
 * proc4430_drv.c
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
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

/* PwrMgmt Initialization */
#include <syslink/notify.h>
#include <syslink/notify_driver.h>
#include <syslink/notifydefs.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/notify_ducatidriver.h>

/* Module headers */
#include "proc4430.h"
#include "proc4430_drvdefs.h"

/* Power Management headers */
#ifdef CONFIG_SYSLINK_DUCATI_PM
#include <linux/platform_device.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/dma.h>
#include <plat/dmtimer.h>
#include <linux/gpio.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#endif


/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
#define PROC4430_NAME "syslink-proc4430"

static char *driver_name = PROC4430_NAME;

static s32 driver_major;

static s32 driver_minor;

struct proc_4430_dev {
	struct cdev cdev;
};

static struct proc_4430_dev *proc_4430_device;

static struct class *proc_4430_class;



/** ============================================================================
 *  Forward declarations of internal functions
 *  ============================================================================
 */
/* Linux driver function to open the driver object. */
static int proc4430_drv_open(struct inode *inode, struct file *filp);

/* Linux driver function to close the driver object. */
static int proc4430_drv_release(struct inode *inode, struct file *filp);

/* Linux driver function to invoke the APIs through ioctl. */
static int proc4430_drv_ioctl(struct inode *inode,
			struct file *filp, unsigned int cmd,
			unsigned long args);

/* Linux driver function to map memory regions to user space. */
static int proc4430_drv_mmap(struct file *filp,
			struct vm_area_struct *vma);

/* Module initialization function for Linux driver. */
static int __init proc4430_drv_initializeModule(void);

/* Module finalization  function for Linux driver. */
static void  __exit proc4430_drv_finalizeModule(void);



/** ============================================================================
 *  Globals
 *  ============================================================================
 */

/*
  File operations table for PROC4430.
 */
static const struct file_operations proc_4430_fops = {
	.open = proc4430_drv_open,
	.release = proc4430_drv_release,
	.ioctl = proc4430_drv_ioctl,
	.mmap = proc4430_drv_mmap,
};

static int proc4430_drv_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int proc4430_drv_release(struct inode *inode, struct file *filp)
{
	return 0;
}

#ifdef CONFIG_SYSLINK_DUCATI_PM
#define SYS_M3 2
#define APP_M3 3

union message_slicer rcb_payload;

struct omap_dm_timer *p_gpt;
int timeout = 1000;

int pm_action_type;
int pm_resource_type;
int pm_notify_response;

int pm_gptimer_num;
int pm_gptimer_counter;

int pm_gpio_num;
int pm_gpio_counter;

int pm_sdmachan_num;
int pm_sdmachan_counter;
int pm_sdmachan_dummy;

int ch, ch_aux;
int return_val;
int gptimer_list[GP_TIMER_MAX] = {
	GP_TIMER_3,
	GP_TIMER_4,
	GP_TIMER_9,
	GP_TIMER_11};
#endif

/*
  Linux driver function to invoke the APIs through ioctl.
 *
 */
static int proc4430_drv_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long args)
{
	int retval = 0;
	struct proc_mgr_cmd_args *cmd_args = (struct proc_mgr_cmd_args *)args;
	struct proc_mgr_cmd_args command_args;

	switch (cmd) {
	case CMD_PROC4430_GETCONFIG:
	{
		struct proc4430_cmd_args_get_config *src_args =
			(struct proc4430_cmd_args_get_config *)args;
		struct proc4430_config cfg;

		/* copy_from_useris not needed for
		* proc4430_get_config, since the
		* user's config is not used.
		*/
		proc4430_get_config(&cfg);

		retval = copy_to_user((void *)(src_args->cfg),
				(const void *)&cfg,
				sizeof(struct proc4430_config));
		if (WARN_ON(retval < 0))
			goto func_exit;
	}
	break;

	case CMD_PROC4430_SETUP:
	{
		struct proc4430_cmd_args_setup *src_args =
			(struct proc4430_cmd_args_setup *)args;
		struct proc4430_config cfg;

		retval = copy_from_user((void *)&cfg,
			(const void *)(src_args->cfg),
			sizeof(struct proc4430_config));
		if (WARN_ON(retval < 0))
			goto func_exit;
		proc4430_setup(&cfg);
	}
	break;

	case CMD_PROC4430_DESTROY:
	{
		proc4430_destroy();
	}
	break;

	case CMD_PROC4430_PARAMS_INIT:
	{
		struct proc4430_cmd_args_params_init src_args;
		struct proc4430_params params;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc4430_cmd_args_params_init));
		if (WARN_ON(retval < 0))
			goto func_exit;
		proc4430_params_init(src_args.handle, &params);
		retval = copy_to_user((void *)(src_args.params),
			(const void *) &params,
			sizeof(struct proc4430_params));
		if (WARN_ON(retval < 0))
			goto func_exit;
	}
	break;

	case CMD_PROC4430_CREATE:
	{
		struct proc4430_cmd_args_create   src_args;
		struct proc4430_params params;
		struct proc4430_mem_entry *entries = NULL;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
				(const void *)(args),
				sizeof(struct proc4430_cmd_args_create));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = copy_from_user((void *) &params,
			(const void *)(src_args.params),
			sizeof(struct proc4430_params));
		if (WARN_ON(retval < 0))
			goto func_exit;
		/* Copy the contents of mem_entries from user-side */
		if (params.num_mem_entries) {
			entries = vmalloc(params.num_mem_entries * \
				sizeof(struct proc4430_mem_entry));
			if (WARN_ON(!entries))
				goto func_exit;
			retval = copy_from_user((void *) (entries),
				(const void *)(params.mem_entries),
				params.num_mem_entries * \
				sizeof(struct proc4430_mem_entry));
			if (WARN_ON(retval < 0)) {
				vfree(entries);
				goto func_exit;
			}
			params.mem_entries = entries;
		}
		src_args.handle = proc4430_create(src_args.proc_id,
							&params);
		if (WARN_ON(src_args.handle == NULL))
			goto func_exit;
		retval = copy_to_user((void *)(args),
				(const void *)&src_args,
				sizeof(struct proc4430_cmd_args_create));
		/* Free the memory created */
		if (params.num_mem_entries)
			vfree(entries);
	}
	break;

	case CMD_PROC4430_DELETE:
	{
		struct proc4430_cmd_args_delete src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc4430_cmd_args_delete));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = proc4430_delete(&(src_args.handle));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROC4430_OPEN:
	{
		struct proc4430_cmd_args_open src_args;

		/*Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
				(const void *)(args),
				sizeof(struct proc4430_cmd_args_open));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = proc4430_open(&(src_args.handle),
					src_args.proc_id);
		retval = copy_to_user((void *)(args),
				(const void *)&src_args,
			sizeof(struct proc4430_cmd_args_open));
		WARN_ON(retval < 0);
	}
	break;

	case CMD_PROC4430_CLOSE:
	{
		struct proc4430_cmd_args_close src_args;

		/*Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
			(const void *)(args),
			sizeof(struct proc4430_cmd_args_close));
		if (WARN_ON(retval < 0))
			goto func_exit;
		retval = proc4430_close(src_args.handle);
		WARN_ON(retval < 0);
	}
	break;

	default:
	{
		printk(KERN_ERR "unsupported ioctl\n");
	}
	break;
	}
func_exit:
	/* Set the status and copy the common args to user-side. */
	command_args.api_status = retval;
	retval = copy_to_user((void *) cmd_args,
		(const void *) &command_args,
		 sizeof(struct proc_mgr_cmd_args));
	WARN_ON(retval < 0);
	return retval;
}


/*
  Linux driver function to map memory regions to user space.
 *
 */
static int proc4430_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);

	if (remap_pfn_range(vma,
			 vma->vm_start,
			 vma->vm_pgoff,
			 vma->vm_end - vma->vm_start,
			 vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

#ifdef CONFIG_SYSLINK_DUCATI_PM
/*
  Function for PM resources Callback
 *
 */
void proc4430_drv_pm_callback(short int procId,
							int eventNo,
							unsigned long args,
							int payload)
{
	/* Get the payload */
	rcb_payload.whole = payload;

	/* Get the type of resource and the actions required */
	pm_action_type = rcb_struct->rcb[rcb_payload.fields.rcb_num].msg_type;
	pm_resource_type = rcb_struct->rcb[rcb_payload.fields.rcb_num].sub_type;

	/* Request the resource to PRCM */
	switch (pm_resource_type) {
	case SDMA:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			return_val =
				proc4430_drv_pm_get_sdma_chan(procId,
					rcb_payload.fields.rcb_num);
			if (return_val < 0) {
				printk(KERN_INFO "Error Requesting SDMA\n");
				/* Update payload */
				rcb_payload.fields.msg_type =
				PM_REQUEST_FAIL;
				rcb_payload.fields.parm =
				PM_INSUFFICIENT_CHANNELS;
				break;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE) {
			pm_sdmachan_num =
			rcb_struct->rcb
			[rcb_payload.fields.rcb_num]
			.num_chan;
			proc4430_drv_pm_rel_sdma_chan
				(rcb_payload.fields.rcb_num);
		}
		break;
	case GP_TIMER:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			/* GP Timers 3,4,9 or 11 for Ducati M3 */
			pm_gptimer_num =
				proc4430_drv_pm_get_gptimer
				(rcb_payload.fields.rcb_num);
			if (pm_gptimer_num < 0) {
				/* GPtimer for Ducati unavailable */
				printk(KERN_ERR "GPTIMER error while allocating\n");
				/* Update the payload with the failure msg */
				rcb_payload.fields.msg_type = PM_REQUEST_FAIL;
				rcb_payload.fields.parm = PM_NO_GPTIMER;
				break;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE)
			proc4430_drv_pm_rel_gptimer(rcb_payload.fields.rcb_num);
		break;
	case GP_IO:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			pm_gpio_num =
				rcb_struct->rcb
				[rcb_payload.fields.rcb_num]
				.fill9;
			if (!gpio_request(pm_gpio_num , "ducati-ss"))
				pm_gpio_counter++;
			else {
				printk(KERN_ERR " GPIO request error\n");
				/* Update the payload with the failure msg */
				rcb_payload.fields.msg_type = PM_REQUEST_FAIL;
				rcb_payload.fields.parm = PM_NO_GPIO;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE) {
			pm_gpio_num =
				rcb_struct->rcb
				[rcb_payload.fields.rcb_num]
				.fill9;
			gpio_free(pm_gpio_num);
			printk(KERN_ERR " GPIO release successfully\n");
			pm_gpio_counter--;
		}
		break;
	case DUCATI:
	case IVA_HD:
	case ISS:
	case I2C:
	default:
		printk(KERN_INFO "Unsupported resource\n");
		break;
	}

	/* Update the payload with the reply msg */
	rcb_payload.fields.reply_flag = true;

	/* Update the payload before send */
	payload = rcb_payload.whole;

	/* send the ACK to DUCATI*/
	return_val = notify_sendevent(
				platform_notifydrv_handle,
				SYS_M3,/*DUCATI_PROC*/
				PM_RESOURCE,/*PWR_MGMT_EVENT*/
				payload,
				false);
	if (return_val < 0)
		printk(KERN_INFO "*****ERROR SENDING PM EVENT\n");
}
EXPORT_SYMBOL(proc4430_drv_pm_callback);

/*
  Function for PM notifications Callback
 *
 */
void proc4430_drv_pm_notify_callback(short int procId,
							int eventNo,
							unsigned long args,
							int payload)
{
	/**
	 * Post semaphore based in eventType (payload);
	 */
	/* Get the payload */
	rcb_payload.whole = payload;
	switch (rcb_payload.fields.msg_subtype) {
	case PM_SUSPEND:
		up(pm_event[PM_SUSPEND].sem_handle);
		break;
	case PM_RESUME:
		up(pm_event[PM_RESUME].sem_handle);
		break;
	case PM_OTHER:
		up(pm_event[PM_OTHER].sem_handle);
		break;
	}
}
EXPORT_SYMBOL(proc4430_drv_pm_notify_callback);

/*
  Function for send PM Notifications
 *
 */
int proc4430_drv_pm_notifications(enum pm_event_type eventType)
{
	/**
	 * Function called by linux driver
	 * Recieves evenType: Suspend, Resume, others...
	 * Send event to Ducati;
	 * Pend semaphore based in eventType (payload);
	 * Return ACK to caller;
	 */
	int pm_ack = true;
	switch (eventType) {
	case PM_SUSPEND:
		rcb_payload.fields.msg_type = PM_NOTIFICATIONS;
		rcb_payload.fields.msg_subtype = PM_SUSPEND;
		rcb_payload.fields.parm = PM_SUCCESS;
		/* send the ACK to DUCATI*/
		return_val = notify_sendevent(
				platform_notifydrv_handle,
				SYS_M3,/*DUCATI_PROC*/
				PM_NOTIFICATION,/*PWR_MGMT_EVENT*/
				(unsigned int)rcb_payload.whole,
				false);
		if (return_val < 0)
			printk(KERN_INFO "*****ERROR SENDING PM EVENT\n");
		return_val = down_timeout(pm_event[PM_SUSPEND].sem_handle,
			msecs_to_jiffies(timeout));
		if (return_val < 0) {
			printk(KERN_INFO "Error Suspend\n");
			pm_ack = PM_FAILURE;
		} else
			pm_ack = rcb_payload.fields.parm;
		break;
	case PM_RESUME:
		rcb_payload.fields.msg_type = PM_NOTIFICATIONS;
		rcb_payload.fields.msg_subtype = PM_RESUME;
		rcb_payload.fields.parm = PM_SUCCESS;
		/* send the ACK to DUCATI*/
		return_val = notify_sendevent(
				platform_notifydrv_handle,
				SYS_M3,/*DUCATI_PROC*/
				PM_NOTIFICATION,/*PWR_MGMT_EVENT*/
				(unsigned int)rcb_payload.whole,
				false);
		if (return_val < 0)
			printk(KERN_INFO "*****ERROR SENDING PM EVENT\n");
		return_val = down_timeout(pm_event[PM_RESUME].sem_handle,
			msecs_to_jiffies(timeout));
		if (return_val < 0) {
			printk(KERN_INFO "Error Suspend\n");
			pm_ack = PM_FAILURE;
		} else
			pm_ack = rcb_payload.fields.parm;
		break;
	case PM_OTHER:
		rcb_payload.fields.msg_type = PM_NOTIFICATIONS;
		rcb_payload.fields.msg_subtype = PM_OTHER;
		rcb_payload.fields.parm = PM_SUCCESS;
		/* send the ACK to DUCATI*/
		return_val = notify_sendevent(
				platform_notifydrv_handle,
				SYS_M3,/*DUCATI_PROC*/
				PM_NOTIFICATION,/*PWR_MGMT_EVENT*/
				(unsigned int)rcb_payload.whole,
				false);
		if (return_val < 0)
			printk(KERN_INFO "*****ERROR SENDING PM EVENT\n");
		return_val = down_timeout(pm_event[PM_OTHER].sem_handle,
			msecs_to_jiffies(timeout));
		if (return_val < 0) {
			printk(KERN_INFO "Error Suspend\n");
			pm_ack = PM_FAILURE;
		} else
			pm_ack = rcb_payload.fields.parm;
		break;
	}

	return pm_ack;
}
EXPORT_SYMBOL(proc4430_drv_pm_notifications);

/*
  Function for get sdma channels from PRCM
 *
 */
inline int proc4430_drv_pm_get_sdma_chan(int proc_id, unsigned rcb_num)
{
	/* Get numchannels from RCB */
	pm_sdmachan_num =
		rcb_struct->rcb[rcb_num]
		.num_chan;
	if (WARN_ON(pm_sdmachan_num < 0))
		return PM_FAILURE;
	if (WARN_ON(pm_sdmachan_num > SDMA_CHANNELS_MAX))
		return PM_FAILURE;

	for (ch = 0; ch < pm_sdmachan_num; ch++) {
		return_val = omap_request_dma(proc_id,
			"ducati-ss",
			NULL,
			NULL,
			&pm_sdmachan_dummy);
		if (return_val == 0) {
			pm_sdmachan_counter++;
			rcb_struct->rcb
			[rcb_num]
			.channels[ch] =
			(unsigned char)pm_sdmachan_dummy;
		} else
			goto clean_sdma;
	}
	return PM_SUCCESS;
clean_sdma:
	printk(KERN_INFO "Error Requesting SDMA\n");
	/*failure, need to free the chanels*/
	for (ch_aux = 0;
			ch_aux < ch;
			ch_aux++) {
		pm_sdmachan_dummy =
		(int)rcb_struct->rcb
		[rcb_num]
		.channels[ch_aux];
		omap_free_dma
			(pm_sdmachan_dummy);
		pm_sdmachan_counter--;
	}
	return PM_FAILURE;
}

/*
  Function for get gptimers from PRCM
 *
 */
inline int proc4430_drv_pm_get_gptimer(unsigned rcb_num)
{
	int pm_gp_num;
	if (WARN_ON(rcb_num < 1))
		return PM_FAILURE;
	if (WARN_ON(rcb_num > RCB_MAX))
		return PM_FAILURE;
	for (pm_gp_num = 0; pm_gp_num < GP_TIMER_MAX; pm_gp_num++) {
		p_gpt = omap_dm_timer_request_specific(gptimer_list[pm_gp_num]);
		if (p_gpt != NULL)
			break;
	}
	if (p_gpt == NULL)
		return PM_FAILURE;
	else {
		/* Store the gptimer number and base address */
		rcb_struct->rcb[rcb_num]
		.fill9 = gptimer_list[pm_gp_num];
		rcb_struct->rcb[rcb_num]
		.mod_base_addr = (unsigned)p_gpt;
		pm_gptimer_counter++;
		return PM_SUCCESS;
	}
}

/*
  Function for release sdma channels to PRCM
 *
 */
inline void proc4430_drv_pm_rel_sdma_chan(unsigned rcb_num)
{
	if (WARN_ON(rcb_num < 1)) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	if (WARN_ON(rcb_num > RCB_MAX)) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	pm_sdmachan_num =
		rcb_struct->rcb
		[rcb_num]
		.num_chan;
	for (ch = 0; ch < pm_sdmachan_num; ch++) {
		pm_sdmachan_dummy =
		(int)rcb_struct->rcb
		[rcb_num]
		.channels[ch];
		omap_free_dma(pm_sdmachan_dummy);
		pm_sdmachan_counter--;
	}
}

/*
  Function for release gptimer to PRCM
 *
 */
inline void proc4430_drv_pm_rel_gptimer(unsigned rcb_num)
{
	if (WARN_ON(rcb_num < 1)) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	if (WARN_ON(rcb_num > RCB_MAX)) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	p_gpt = (struct omap_dm_timer *)rcb_struct->rcb[rcb_num]
			.mod_base_addr;
	if (p_gpt != NULL)
		omap_dm_timer_free(p_gpt);
	rcb_struct->rcb[rcb_num]
	.mod_base_addr = 0;
	pm_gptimer_counter--;
}

#endif

/** ==================================================================
 *  Functions required for multiple .ko modules configuration
 *  ==================================================================
 */
/*
  Module initialization  function for Linux driver.
 */
static int __init proc4430_drv_initializeModule(void)
{
	dev_t dev = 0 ;
	int retval;

	/* Display the version info and created date/time */
	printk(KERN_INFO "proc4430_drv_initializeModule\n");

	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		retval = register_chrdev_region(dev, 1, driver_name);
	} else {
		retval = alloc_chrdev_region(&dev, driver_minor, 1,
				 driver_name);
		driver_major = MAJOR(dev);
	}

	proc_4430_device = kmalloc(sizeof(struct proc_4430_dev), GFP_KERNEL);
	if (!proc_4430_device) {
		retval = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		goto exit;
	}
	memset(proc_4430_device, 0, sizeof(struct proc_4430_dev));
	cdev_init(&proc_4430_device->cdev, &proc_4430_fops);
	proc_4430_device->cdev.owner = THIS_MODULE;
	proc_4430_device->cdev.ops = &proc_4430_fops;

	retval = cdev_add(&proc_4430_device->cdev, dev, 1);

	if (retval) {
		printk(KERN_ERR "Failed to add the syslink proc_4430 device \n");
		goto exit;
	}

	/* udev support */
	proc_4430_class = class_create(THIS_MODULE, "syslink-proc4430");

	if (IS_ERR(proc_4430_class)) {
		printk(KERN_ERR "Error creating bridge class \n");
		goto exit;
	}
	device_create(proc_4430_class, NULL, MKDEV(driver_major, driver_minor),
			NULL, PROC4430_NAME);
exit:
	return 0;
}

/*
 function to finalize the driver module.
 */
static void __exit proc4430_drv_finalizeModule(void)
{
	dev_t devno = 0;

	/* FIX ME: THIS MIGHT NOT BE THE RIGHT PLACE TO CALL THE SETUP */
	proc4430_destroy();

	devno = MKDEV(driver_major, driver_minor);
	if (proc_4430_device) {
		cdev_del(&proc_4430_device->cdev);
		kfree(proc_4430_device);
	}
	unregister_chrdev_region(devno, 1);
	if (proc_4430_class) {
		/* remove the device from sysfs */
		device_destroy(proc_4430_class, MKDEV(driver_major,
						driver_minor));
		class_destroy(proc_4430_class);
	}
	return;
}

/*
  Macro calls that indicate initialization and finalization functions
 *		  to the kernel.
 */
MODULE_LICENSE("GPL v2");
module_init(proc4430_drv_initializeModule);
module_exit(proc4430_drv_finalizeModule);
