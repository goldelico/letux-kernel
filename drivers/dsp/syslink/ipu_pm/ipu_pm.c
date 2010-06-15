/*
 * ipu_pm.c
 *
 * IPU Power Management support functions for TI OMAP processors.
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
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <syslink/notify.h>
#include <syslink/notify_driver.h>
#include <syslink/notifydefs.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/notify_ducatidriver.h>

/* Power Management headers */
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/dma.h>
#include <plat/dmtimer.h>
#include <plat/clock.h>
#include <plat/i2c.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
/* Module headers */
#include "ipu_pm.h"

/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
#define A9 3
#define SYS_M3 2
#define APP_M3 1
#define TESLA 0

#define LINE_ID 0
/* SYSM3 and APPM3 */
#define NUM_REMOTE_PROC 2

/** ============================================================================
 *  Forward declarations of internal functions
 *  ============================================================================
 */

/* Function to get sdma channels from PRCM */
inline int ipu_pm_get_sdma_chan(int proc_id, unsigned rcb_num);

/* Function to get gptimers from PRCM */
inline int ipu_pm_get_gptimer(int proc_id, unsigned rcb_num);

/* Function to release sdma channels to PRCM */
inline void ipu_pm_rel_sdma_chan(int proc_id, unsigned rcb_num);

/* Function to release gptimer to PRCM */
inline void ipu_pm_rel_gptimer(int proc_id, unsigned rcb_num);

/* Function to get an i2c bus */
inline int ipu_pm_get_i2c_bus(int proc_id, unsigned rcb_num);

/* Function to release an i2c bus */
inline int ipu_pm_rel_i2c_bus(int proc_id, unsigned rcb_num);

/* Function to get ipu pm object */
inline struct ipu_pm_object *ipu_pm_get_handle(int proc_id);

/** ============================================================================
 *  Globals
 *  ============================================================================
 */

union message_slicer pm_msg;

int pm_action_type;
int pm_resource_type;
int pm_notify_response;
int pm_gptimer_num;
int pm_gpio_num;
int pm_i2c_bus_num;
int pm_sdmachan_num;
int pm_sdmachan_dummy;
int ch, ch_aux;
int return_val;

int ipu_timer_list[NUM_IPU_TIMERS] = {
	GP_TIMER_3,
	GP_TIMER_4,
	GP_TIMER_9,
	GP_TIMER_11};

struct ipu_pm_object *pm_handle_appm3;
struct ipu_pm_object *pm_handle_sysm3;

static struct ipu_pm_module_object ipu_pm_state = {
	.def_cfg.reserved = 1,
	.gate_handle = NULL
} ;

static struct ipu_pm_params pm_params = {
	.pm_gpio_counter = 0,
	.pm_gptimer_counter = 0,
	.pm_i2c_bus_counter = 0,
	.pm_sdmachan_counter = 0,
	.shared_addr = (void *)PM_SHM_BASE_ADDR,
	.timeout = 10000,
	.pm_num_events = NUMBER_PM_EVENTS,
	.pm_resource_event = PM_RESOURCE,
	.pm_notification_event = PM_NOTIFICATION,
	.proc_id = A9,
	.remote_proc_id = -1,
	.line_id = 0
} ;

/*
  Function for PM resources Callback
 *
 */
void ipu_pm_callback(u16 proc_id, u16 line_id, u32 event_id,
					uint *arg, u32 payload)
{
	struct rcb_block *rcb_p;
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	int status;

	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get the payload */
	pm_msg.whole = payload;
	/* Get pointer to the proper RCB */
	rcb_p = (struct rcb_block *)
		&handle->rcb_table->rcb[pm_msg.fields.rcb_num];

	/* Get the type of resource and the actions required */
	pm_action_type = rcb_p->msg_type;
	pm_resource_type = rcb_p->sub_type;

	/* Request the resource to PRCM */
	switch (pm_resource_type) {
	case SDMA:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			return_val =
				ipu_pm_get_sdma_chan(proc_id,
					pm_msg.fields.rcb_num);
			if (return_val < 0) {
				printk(KERN_INFO "Error Requesting SDMA\n");
				/* Update payload with the failure msg */
				pm_msg.fields.msg_type = PM_REQUEST_FAIL;
				pm_msg.fields.parm = PM_INSUFFICIENT_CHANNELS;
				break;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE)
			ipu_pm_rel_sdma_chan(proc_id, pm_msg.fields.rcb_num);
		break;
	case GP_TIMER:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			/* GP Timers 3,4,9 or 11 for Ducati M3 */
			return_val = ipu_pm_get_gptimer(proc_id,
							pm_msg.fields.rcb_num);
			if (return_val < 0) {
				/* GPtimer for Ducati unavailable */
				printk(KERN_ERR "GPTIMER error allocating\n");
				/* Update the payload with the failure msg */
				pm_msg.fields.msg_type = PM_REQUEST_FAIL;
				pm_msg.fields.parm = PM_NO_GPTIMER;
				break;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE)
			ipu_pm_rel_gptimer(proc_id, pm_msg.fields.rcb_num);
		break;
	case GP_IO:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			status = gpio_request(pm_gpio_num , "ducati-ss");
			if (status == 0)
				params->pm_gpio_counter++;
			else {
				/* Update the payload with the failure msg */
				pm_msg.fields.msg_type = PM_REQUEST_FAIL;
				pm_msg.fields.parm = PM_NO_GPIO;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE) {
			pm_gpio_num = rcb_p->fill9;
			gpio_free(pm_gpio_num);
			printk(KERN_ERR " GPIO release successfully\n");
			params->pm_gpio_counter--;
		}
		break;
	case I2C:
		if (pm_action_type == PM_REQUEST_RESOURCE) {
			return_val = ipu_pm_get_i2c_bus(proc_id,
						pm_msg.fields.rcb_num);
			if (return_val < 0) {
				/* i2c bus/clock for Ducati unavailable */
				printk(KERN_ERR "i2c error\n");
				/* Update the payload with the failure msg */
				pm_msg.fields.msg_type = PM_REQUEST_FAIL;
				pm_msg.fields.parm = PM_NO_I2C;
				break;
			}
		}
		if (pm_action_type == PM_RELEASE_RESOURCE)
			ipu_pm_rel_i2c_bus(proc_id, pm_msg.fields.rcb_num);
		break;
	case DUCATI:
	case IVA_HD:
	case ISS:
	default:
		printk(KERN_INFO "Unsupported resource\n");
		break;
	}

	/* Update the payload with the reply msg */
	pm_msg.fields.reply_flag = true;

	/* Update the payload before send */
	payload = pm_msg.whole;

	/* send the ACK to DUCATI*/
	return_val = notify_send_event(
				params->remote_proc_id,/*DUCATI_PROC*/
				params->line_id,
				params->pm_resource_event | \
					(NOTIFY_SYSTEMKEY << 16),
				payload,
				false);
	if (return_val < 0)
		printk(KERN_INFO "ERROR SENDING PM EVENT\n");
}
EXPORT_SYMBOL(ipu_pm_callback);

/*
  Function for PM notifications Callback
 *
 */
void ipu_pm_notify_callback(u16 proc_id, u16 line_id, u32 event_id,
					uint *arg, u32 payload)
{
#ifdef CONFIG_SYSLINK_DUCATI_PM
	/**
	 * Post semaphore based in eventType (payload);
	 * IPU has alreay finished the process for the
	 * notification
	 */
	/* Get the payload */
	struct ipu_pm_object *handle;
	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);

	pm_msg.whole = payload;
	switch (pm_msg.fields.msg_subtype) {
	case PM_SUSPEND:
		up(handle->pm_event[PM_SUSPEND].sem_handle);
		break;
	case PM_RESUME:
		up(handle->pm_event[PM_RESUME].sem_handle);
		break;
	case PM_OTHER:
		up(handle->pm_event[PM_OTHER].sem_handle);
		break;
	}
#endif
}
EXPORT_SYMBOL(ipu_pm_notify_callback);

/*
  Function for send PM Notifications
 *
 */
int ipu_pm_notifications(enum pm_event_type event_type)
{
	/**
	 * Function called by linux driver
	 * Recieves evenType: Suspend, Resume, others...
	 * Send event to Ducati
	 * Pend semaphore based in event_type (payload)
	 * Return ACK to caller
	 */

	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	int pm_ack = 0;
	int i;
	int proc_id;

#ifdef CONFIG_SYSLINK_DUCATI_PM
	/*get the handle to proper ipu pm object */
	for (i = 0; i < NUM_REMOTE_PROC; i++) {
		proc_id = i + 1;
		handle = ipu_pm_get_handle(proc_id);
		if (handle == NULL)
			continue;
		params = handle->params;
		switch (event_type) {
		case PM_SUSPEND:
			pm_msg.fields.msg_type = PM_NOTIFICATIONS;
			pm_msg.fields.msg_subtype = PM_SUSPEND;
			pm_msg.fields.parm = PM_SUCCESS;
			/* send the request to IPU*/
			return_val = notify_send_event(
					params->remote_proc_id,
					params->line_id,
					params->pm_notification_event | \
						(NOTIFY_SYSTEMKEY << 16),
					(unsigned int)pm_msg.whole,
					false);
			if (return_val < 0)
				printk(KERN_INFO "ERROR SENDING PM EVENT\n");
			/* wait until event from IPU (ipu_pm_notify_callback)*/
			return_val = down_timeout
					(handle->pm_event[PM_SUSPEND]
					.sem_handle,
					msecs_to_jiffies(params->timeout));
			if (WARN_ON((return_val < 0) ||
					(pm_msg.fields.parm ==
						PM_NOTIFICATIONS_FAIL))) {
				printk(KERN_INFO "Error Suspend\n");
				pm_ack = EBUSY;
			}
			break;
		case PM_RESUME:
			pm_msg.fields.msg_type = PM_NOTIFICATIONS;
			pm_msg.fields.msg_subtype = PM_RESUME;
			pm_msg.fields.parm = PM_SUCCESS;
			/* send the request to IPU*/
			return_val = notify_send_event(
					params->remote_proc_id,
					params->line_id,
					params->pm_notification_event | \
						(NOTIFY_SYSTEMKEY << 16),
					(unsigned int)pm_msg.whole,
					false);
			if (return_val < 0)
				printk(KERN_INFO "ERROR SENDING PM EVENT\n");
			/* wait until event from IPU (ipu_pm_notify_callback)*/
			return_val = down_timeout
					(handle->pm_event[PM_RESUME].sem_handle,
					msecs_to_jiffies(params->timeout));
			if (WARN_ON((return_val < 0) ||
					(pm_msg.fields.parm ==
						PM_NOTIFICATIONS_FAIL))) {
				printk(KERN_INFO "Error Resume\n");
				pm_ack = EBUSY;
			}
			break;
		case PM_OTHER:
			pm_msg.fields.msg_type = PM_NOTIFICATIONS;
			pm_msg.fields.msg_subtype = PM_OTHER;
			pm_msg.fields.parm = PM_SUCCESS;
			/* send the request to IPU*/
			return_val = notify_send_event(
					params->remote_proc_id,
					params->line_id,
					params->pm_notification_event | \
						(NOTIFY_SYSTEMKEY << 16),
					(unsigned int)pm_msg.whole,
					false);
			if (return_val < 0)
				printk(KERN_INFO "ERROR SENDING PM EVENT\n");
			/* wait until event from IPU (ipu_pm_notify_callback)*/
			return_val = down_timeout
					(handle->pm_event[PM_OTHER].sem_handle,
					msecs_to_jiffies(params->timeout));
			if (WARN_ON((return_val < 0) ||
					(pm_msg.fields.parm ==
						PM_NOTIFICATIONS_FAIL))) {
				printk(KERN_INFO "Error Other\n");
				pm_ack = EBUSY;
			}
			break;
		}
	}
#endif
	return pm_ack;
}
EXPORT_SYMBOL(ipu_pm_notifications);

/*
  Function to get sdma channels from PRCM
 *
 */
inline int ipu_pm_get_sdma_chan(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;

	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX)))
		return PM_FAILURE;
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];
	/* Get number of channels from RCB */
	pm_sdmachan_num = rcb_p->num_chan;
	if (WARN_ON((pm_sdmachan_num <= 0) ||
			(pm_sdmachan_num > SDMA_CHANNELS_MAX)))
		return PM_FAILURE;

	/* Request resource using PRCM API */
	for (ch = 0; ch < pm_sdmachan_num; ch++) {
		return_val = omap_request_dma(proc_id,
			"ducati-ss",
			NULL,
			NULL,
			&pm_sdmachan_dummy);
		if (return_val == 0) {
			params->pm_sdmachan_counter++;
			rcb_p->channels[ch] = (unsigned char)pm_sdmachan_dummy;
		} else
			goto clean_sdma;
	}
	return PM_SUCCESS;
clean_sdma:
	printk(KERN_INFO "Error Requesting SDMA\n");
	/*failure, need to free the chanels*/
	for (ch_aux = 0; ch_aux < ch; ch_aux++) {
		pm_sdmachan_dummy = (int)rcb_p->channels[ch_aux];
		omap_free_dma(pm_sdmachan_dummy);
		params->pm_sdmachan_counter--;
	}
	return PM_FAILURE;
}

/*
  Function to get gptimers from PRCM
 *
 */
inline int ipu_pm_get_gptimer(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;
	struct omap_dm_timer *p_gpt;
	int pm_gp_num;

	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX)))
		return PM_FAILURE;
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];
	/* Request resource using PRCM API */
	for (pm_gp_num = 0; pm_gp_num < NUM_IPU_TIMERS; pm_gp_num++) {
		p_gpt = omap_dm_timer_request_specific
			(ipu_timer_list[pm_gp_num]);
		if (p_gpt != NULL)
			break;
	}
	if (p_gpt == NULL)
		return PM_FAILURE;
	else {
		/* Store the gptimer number and base address */
		rcb_p->fill9 = ipu_timer_list[pm_gp_num];
		rcb_p->mod_base_addr = (unsigned)p_gpt;
		params->pm_gptimer_counter++;
		return PM_SUCCESS;
	}
}

/*
  Function to get an i2c bus
 *
 */
inline int ipu_pm_get_i2c_bus(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;
	struct clk *p_i2c_clk;
	int i2c_clk_status;
	char i2c_name[10];

	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX)))
		return PM_FAILURE;
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];

	pm_i2c_bus_num = rcb_p->fill9;
	if (WARN_ON((pm_i2c_bus_num <= I2C_BUS_MIN) ||
			(pm_i2c_bus_num > I2C_BUS_MAX)))
		return PM_FAILURE;

	/* building the name for i2c_clk */
	sprintf(i2c_name, "i2c%d_fck", pm_i2c_bus_num);

	/* Request resource using PRCM API */
	p_i2c_clk = omap_clk_get_by_name(i2c_name);
	if (p_i2c_clk == 0)
		return PM_FAILURE;
	i2c_clk_status = clk_enable(p_i2c_clk);
	if (i2c_clk_status != 0)
		return PM_FAILURE;
	rcb_p->mod_base_addr = (unsigned)p_i2c_clk;
	params->pm_i2c_bus_counter++;

	return PM_SUCCESS;
}

/*
  Function to release sdma channels to PRCM
 *
 */
inline void ipu_pm_rel_sdma_chan(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;

	/*get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX))) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];

	/* Release resource using PRCM API */
	pm_sdmachan_num = rcb_p->num_chan;
	for (ch = 0; ch < pm_sdmachan_num; ch++) {
		pm_sdmachan_dummy = (int)rcb_p->channels[ch];
		omap_free_dma(pm_sdmachan_dummy);
		params->pm_sdmachan_counter--;
	}
}

/*
  Function to release gptimer to PRCM
 *
 */
inline void ipu_pm_rel_gptimer(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;
	struct omap_dm_timer *p_gpt;

	/* get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX))) {
		printk(KERN_INFO "Invalid RCB number\n");
		return;
	}
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];

	p_gpt = (struct omap_dm_timer *)rcb_p->mod_base_addr;
	/* Release resource using PRCM API */
	if (p_gpt != NULL)
		omap_dm_timer_free(p_gpt);
	rcb_p->mod_base_addr = 0;
	params->pm_gptimer_counter--;
}

/*
  Function to release an i2c bus
 *
 */
inline int ipu_pm_rel_i2c_bus(int proc_id, unsigned rcb_num)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	struct rcb_block *rcb_p;
	struct clk *p_i2c_clk;

	/* get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(proc_id);
	params = handle->params;

	/* Get pointer to the proper RCB */
	if (WARN_ON((rcb_num < RCB_MIN) || (rcb_num > RCB_MAX))) {
		printk(KERN_INFO "Invalid RCB number\n");
		return PM_FAILURE;
	}
	rcb_p = (struct rcb_block *)&handle->rcb_table->rcb[rcb_num];
	p_i2c_clk = (struct clk *)rcb_p->mod_base_addr;

	/* Release resource using PRCM API */
	/* TODO */
	clk_disable(p_i2c_clk);
	rcb_p->mod_base_addr = 0;
	params->pm_i2c_bus_counter--;

	return PM_SUCCESS;
}

/*
  Function to set init parameters
 *
 */
void ipu_pm_params_init(struct ipu_pm_params *params)
{
	s32 retval = 0;

	if (WARN_ON(unlikely(params == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	memcpy(params, &(pm_params), sizeof(struct ipu_pm_params));
	return;
exit:
	printk(KERN_ERR "ipu_pm_params_init failed status(0x%x)", retval);
}
EXPORT_SYMBOL(ipu_pm_params_init);

/*
  Function to calculate ipu_pm mem required
 *
 */
int ipu_pm_mem_req(const struct ipu_pm_params *params)
{
	/* Memory required for ipu pm module */
	/* FIXME: Maybe more than this is needed */
	return sizeof(struct sms);
}
EXPORT_SYMBOL(ipu_pm_mem_req);

/*
  Function to register events
  This function will register the events needed for ipu_pm
  the events reserved for power management are 2 and 3
  both sysm3 and appm3 will use the same events.
 */
int ipu_pm_init_transport(struct ipu_pm_object *obj)
{
	s32 status = 0;
	struct ipu_pm_params *params;

	if (WARN_ON(unlikely(obj == NULL))) {
		status = -EINVAL;
		goto pm_register_fail;
	}

	params = obj->params;

	status = notify_register_event(
		params->remote_proc_id,
		params->line_id,
		params->pm_resource_event | \
				(NOTIFY_SYSTEMKEY << 16),
		(notify_fn_notify_cbck)ipu_pm_callback,
		(void *)NULL);
	if (status < 0)
		goto pm_register_fail;

	status = notify_register_event(
		params->remote_proc_id,
		params->line_id,
		params->pm_notification_event | \
				(NOTIFY_SYSTEMKEY << 16),
		(notify_fn_notify_cbck)ipu_pm_notify_callback,
		(void *)NULL);

	if (status < 0) {
		status = notify_unregister_event(
		params->remote_proc_id,
		params->line_id,
		params->pm_resource_event | \
				(NOTIFY_SYSTEMKEY << 16),
		(notify_fn_notify_cbck)ipu_pm_callback,
		(void *)NULL);
		if (status < 0)
			printk(KERN_INFO "ERROR UNREGISTERING PM EVENT\n");
		goto pm_register_fail;
	}

	return status;

pm_register_fail:
	printk(KERN_ERR "pm register events failed status(0x%x)", status);
	return status;

}

/*
  Function to create ipu pm object
 *
 */
struct ipu_pm_object *ipu_pm_create(const struct ipu_pm_params *params)
{
	int i;
	s32 retval = 0;

	if (WARN_ON(unlikely(params == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	if (params->remote_proc_id == SYS_M3) {
		pm_handle_sysm3 = kmalloc(sizeof(struct ipu_pm_object),
						GFP_ATOMIC);

		if (WARN_ON(unlikely(pm_handle_sysm3 == NULL))) {
			retval = -EINVAL;
			goto exit;
		}

		pm_handle_sysm3->rcb_table = (struct sms *)params->shared_addr;

#ifdef CONFIG_SYSLINK_DUCATI_PM
		pm_handle_sysm3->pm_event = kzalloc(sizeof(struct pm_event)
					* params->pm_num_events, GFP_KERNEL);

		if (WARN_ON(unlikely(pm_handle_sysm3->pm_event == NULL))) {
			retval = -EINVAL;
			kfree(pm_handle_sysm3);
			goto exit;
		}

		/* Each event has it own sem */
		for (i = 0; i < params->pm_num_events; i++) {
			pm_handle_sysm3->pm_event[i].sem_handle =
					kzalloc(sizeof(struct semaphore),
						GFP_KERNEL);
			if (WARN_ON(unlikely(pm_handle_sysm3->pm_event[i]
						.sem_handle == NULL))) {
				retval = -EINVAL;
				kfree(pm_handle_sysm3->pm_event);
				kfree(pm_handle_sysm3);
				goto exit;
			}
			sema_init(pm_handle_sysm3->pm_event[i].sem_handle, 0);
			pm_handle_sysm3->pm_event[i].event_type = i;
		}
#endif

		pm_handle_sysm3->params = kzalloc(sizeof(struct ipu_pm_params)
							, GFP_KERNEL);

		if (WARN_ON(unlikely(pm_handle_sysm3->params == NULL))) {
			retval = -EINVAL;
			kfree(pm_handle_sysm3);
			goto exit;
		}

		memcpy(pm_handle_sysm3->params, params,
			sizeof(struct ipu_pm_params));

		return pm_handle_sysm3;
	} else {/* remote_proc_id == APP_M3 */
		pm_handle_appm3 = kmalloc(sizeof(struct ipu_pm_object),
						GFP_ATOMIC);

		if (WARN_ON(unlikely(pm_handle_appm3 == NULL))) {
			retval = -EINVAL;
			goto exit;
		}

		pm_handle_appm3->rcb_table = (struct sms *)params->shared_addr;

#ifdef CONFIG_SYSLINK_DUCATI_PM
		pm_handle_appm3->pm_event = kzalloc(sizeof(struct pm_event)
					* params->pm_num_events, GFP_KERNEL);

		if (WARN_ON(unlikely(pm_handle_appm3->pm_event == NULL))) {
			retval = -EINVAL;
			kfree(pm_handle_appm3);
			goto exit;
		}

		/* Each event has it own sem */
		for (i = 0; i < params->pm_num_events; i++) {
			pm_handle_appm3->pm_event[i].sem_handle =
					kzalloc(sizeof(struct semaphore),
						GFP_KERNEL);
			if (WARN_ON(unlikely(pm_handle_appm3->pm_event[i]
						.sem_handle == NULL))) {
				retval = -EINVAL;
				kfree(pm_handle_appm3->pm_event);
				kfree(pm_handle_appm3);
				goto exit;
			}
			sema_init(pm_handle_appm3->pm_event[i].sem_handle, 0);
			pm_handle_appm3->pm_event[i].event_type = i;
		}
#endif
		pm_handle_appm3->params = kzalloc(sizeof(struct ipu_pm_params)
						, GFP_KERNEL);

		if (WARN_ON(unlikely(pm_handle_appm3->params == NULL))) {
			retval = -EINVAL;
			kfree(pm_handle_appm3);
			goto exit;
		}

		memcpy(pm_handle_appm3->params, params,
			sizeof(struct ipu_pm_params));

		return pm_handle_appm3;
	}

exit:
	printk(KERN_ERR "ipu_pm_create failed! "
		"status = 0x%x\n", retval);
	return NULL;
}

/*
  Function to delete ipu pm object
 *
 */
void ipu_pm_delete(struct ipu_pm_object *handle)
{
	u32 i = 0;
	s32 retval = 0;
	struct ipu_pm_params *params;

	if (WARN_ON(unlikely(handle == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	params = handle->params;
	/* Release the shared RCB */
	handle->rcb_table = NULL;
#ifdef CONFIG_SYSLINK_DUCATI_PM
	for (i = 0; i < params->pm_num_events; i++) {
		kfree(handle->pm_event[i].sem_handle);
		handle->pm_event[i].event_type = 0;
	}
	kfree(handle->pm_event);
	if (params->remote_proc_id == SYS_M3)
		pm_handle_sysm3 = NULL;
	else
		pm_handle_appm3 = NULL;
	kfree(handle->params);
#else
	if (params->remote_proc_id == SYS_M3)
		pm_handle_sysm3 = NULL;
	else
		pm_handle_appm3 = NULL;
	kfree(handle->params);
#endif
	kfree(handle);
	return;
exit:
	printk(KERN_ERR "ipu_pm_delete is already NULL "
		"status = 0x%x\n", retval);
}

/*
  Function to get ipu pm object
 *
 */
inline struct ipu_pm_object *ipu_pm_get_handle(int proc_id)
{
	if (proc_id == SYS_M3)
		return pm_handle_sysm3;
	else if (proc_id == APP_M3)
		return pm_handle_appm3;
	else
		return NULL;
}

/*
  Get the default configuration for the ipu_pm module.
  needed in ipu_pm_setup.
 */
void ipu_pm_get_config(struct ipu_pm_config *cfg)
{
	s32 retval = 0;

	if (WARN_ON(unlikely(cfg == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	if (atomic_cmpmask_and_lt(&(ipu_pm_state.ref_count),
			IPU_PM_MAKE_MAGICSTAMP(0),
			IPU_PM_MAKE_MAGICSTAMP(1)) == true)
		memcpy(cfg, &ipu_pm_state.def_cfg,
			sizeof(struct ipu_pm_config));
	else
		memcpy(cfg, &ipu_pm_state.cfg, sizeof(struct ipu_pm_config));
	return;

exit:
	if (retval < 0) {
		printk(KERN_ERR "ipu_pm_get_config failed! status = 0x%x",
			retval);
	}
	return;
}
EXPORT_SYMBOL(ipu_pm_get_config);

/*
  Function to setup ipu pm object
  This function is called in platform_setup()
  TODO
  This function will load the default configuration for ipu_pm
  in this function we can decide what is going to be controled
  by ipu_pm (DVFS, NOTIFICATIONS, ...) this configuration can
  can be changed on run-time.
 */
int ipu_pm_setup(struct ipu_pm_config *cfg)
{
	struct ipu_pm_config tmp_cfg;
	s32 retval = 0;
	struct mutex *lock = NULL;

	/* This sets the ref_count variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
	atomic_cmpmask_and_set(&ipu_pm_state.ref_count,
				IPU_PM_MAKE_MAGICSTAMP(0),
				IPU_PM_MAKE_MAGICSTAMP(0));
	if (atomic_inc_return(&ipu_pm_state.ref_count)
				!= IPU_PM_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (cfg == NULL) {
		ipu_pm_get_config(&tmp_cfg);
		cfg = &tmp_cfg;
	}

	/* Create a default gate handle for local module protection */
	lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL) {
		retval = -ENOMEM;
		goto exit;
	}
	mutex_init(lock);
	ipu_pm_state.gate_handle = lock;

	/* No proc attached yet */
	pm_handle_appm3 = NULL;
	pm_handle_sysm3 = NULL;

	memcpy(&ipu_pm_state.cfg, cfg, sizeof(struct ipu_pm_config));
	ipu_pm_state.is_setup = true;
	return retval;

exit:
	printk(KERN_ERR "ipu_pm_setup failed! retval = 0x%x", retval);
	return retval;
}
EXPORT_SYMBOL(ipu_pm_setup);

/*
  Function to attach ipu pm object
  This function is called in ipc_attach()
  TODO
  This function will create the object based on the remoteproc id
  and save the handle.
  It is also recieving the shared address pointer to use in rcb
 */
int ipu_pm_attach(u16 remote_proc_id, void *shared_addr)
{
	struct ipu_pm_params params;
	struct ipu_pm_object *ipu_pm_handle;
	s32 retval = 0;

	ipu_pm_params_init(&params);
	params.remote_proc_id = remote_proc_id;
	params.shared_addr = (void *)shared_addr;
	params.line_id = LINE_ID;
	params.shared_addr_size = ipu_pm_mem_req(NULL);

	ipu_pm_handle = ipu_pm_create(&params);

	if (WARN_ON(unlikely(ipu_pm_handle == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	ipu_pm_init_transport(ipu_pm_handle);

	return retval;
exit:
	printk(KERN_ERR "ipu_pm_attach failed! retval = 0x%x", retval);
	return retval;
}
EXPORT_SYMBOL(ipu_pm_attach);

/*
  Function to deattach ipu pm object
  This function is called in ipc_deattach()
  TODO
  This function will delete the object based on the remoteproc id
  and save the handle.
 */
int ipu_pm_detach(u16 remote_proc_id)
{
	struct ipu_pm_object *handle;
	struct ipu_pm_params *params;
	s32 retval = 0;

	/* get the handle to proper ipu pm object */
	handle = ipu_pm_get_handle(remote_proc_id);
	params = handle->params;

	if (WARN_ON(unlikely(handle == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	/* unregister the events used for ipu_pm */
	retval = notify_unregister_event(
		params->remote_proc_id,
		params->line_id,
		params->pm_resource_event | (NOTIFY_SYSTEMKEY << 16),
		(notify_fn_notify_cbck)ipu_pm_callback,
		(void *)NULL);
	if (retval < 0) {
		printk(KERN_INFO "ERROR UNREGISTERING PM EVENT\n");
		goto exit;
	}
	retval = notify_unregister_event(
		params->remote_proc_id,
		params->line_id,
		params->pm_notification_event | (NOTIFY_SYSTEMKEY << 16),
		(notify_fn_notify_cbck)ipu_pm_notify_callback,
		(void *)NULL);
	if (retval < 0) {
		printk(KERN_INFO "ERROR UNREGISTERING PM EVENT\n");
		goto exit;
	}

	/* Deleting the handle based on remote_proc_id */
	ipu_pm_delete(handle);
	return retval;
exit:
	printk(KERN_ERR "ipu_pm_detach failed handle null retval 0x%x", retval);
	return retval;
}
EXPORT_SYMBOL(ipu_pm_detach);

/*
  Function to destroy ipu_pm module
  this function will destroy the shared region 1(?)
  an all the other structs created to set the configuration
 */
int ipu_pm_destroy(void)
{
	s32 retval = 0;
	struct mutex *lock = NULL;

	if (WARN_ON(unlikely(atomic_cmpmask_and_lt(
				&ipu_pm_state.ref_count,
				IPU_PM_MAKE_MAGICSTAMP(0),
				IPU_PM_MAKE_MAGICSTAMP(1)) == true))) {
		retval = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&ipu_pm_state.ref_count)
					== IPU_PM_MAKE_MAGICSTAMP(0))) {
		retval = 1;
		goto exit;
	}

	if (WARN_ON(ipu_pm_state.gate_handle == NULL)) {
		retval = -ENODEV;
		goto exit;
	}

	retval = mutex_lock_interruptible(ipu_pm_state.gate_handle);
	if (retval)
		goto exit;

	lock = ipu_pm_state.gate_handle;
	ipu_pm_state.gate_handle = NULL;
	mutex_unlock(lock);
	kfree(lock);
	return 0;

exit:
	if (retval < 0) {
		printk(KERN_ERR "ipu_pm_destroy failed, retval: %x\n",
			retval);
	}
	return retval;
}
EXPORT_SYMBOL(ipu_pm_destroy);
