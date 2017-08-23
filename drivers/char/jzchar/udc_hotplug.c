/*
 * linux/drivers/char/jzchar/udc_hotplug.c
 *
 * UDC hotplug driver.
 *
 * Copyright (C) 2007 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include <asm/jzsoc.h>

#include "jzchars.h"

#ifndef GPIO_UDC_HOTPLUG
#define GPIO_UDC_HOTPLUG 86
#endif

#define UDC_HOTPLUG_PIN   GPIO_UDC_HOTPLUG
#define UDC_HOTPLUG_IRQ   (IRQ_GPIO_0 + UDC_HOTPLUG_PIN)

int jz_udc_active = 0; /* 0: Have no actions; 1: Have actions */

static int udc_pin_level;
static int udc_old_state;

static struct timer_list udc_timer;

/* Kernel thread to deliver event to user space */
static struct task_struct *kudcd_task;

static void run_sbin_udc_hotplug(int insert, int old_state)
{
        int i;
        char *argv[3], *envp[8];
 
        if (!uevent_helper[0])
                return;

	argv[0] = uevent_helper;
	argv[1] = "udc";
	argv[2] = NULL;

        /* minimal command environment */
	i = 0;
        envp[i++] = "HOME=/";
        envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";

        if (insert)
                envp[i++] = "ACTION=add";
        else {
		if (old_state)
                	envp[i++] = "ACTION=remove-cable";
		else
                	envp[i++] = "ACTION=remove-power";	
	}
        envp[i] = 0;

	call_usermodehelper (argv[0], argv, envp, 0);
}

static int udc_thread(void *unused)
{
	printk(KERN_NOTICE "UDC starting monitor thread\n");

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		/* Deliver this event to user space */
		run_sbin_udc_hotplug(udc_pin_level, udc_old_state);
	}
}

static void udc_timer_routine(unsigned long data)
{
	udc_old_state = jz_udc_active; /* jz_udc_active was updated in the udc driver */

	udc_pin_level = __gpio_get_pin(UDC_HOTPLUG_PIN);

	/* Setup irq for next event */
	if (udc_pin_level) {
		/* Cable has connected, wait for disconnection. */
		__gpio_as_irq_fall_edge(UDC_HOTPLUG_PIN);
	}
	else {
		/* update udc state */
		jz_udc_active = 0; /* Have no actions */

		/* Cable has disconnected, wait for connection. */
		__gpio_as_irq_rise_edge(UDC_HOTPLUG_PIN);
	}

	wake_up_process(kudcd_task);
}

static irqreturn_t udc_hotplug_irq(int irq, void *dev_id)
{
        __gpio_ack_irq(UDC_HOTPLUG_PIN); /* clear interrupt pending status */

	del_timer(&udc_timer);
	init_timer(&udc_timer);
        udc_timer.function = udc_timer_routine;
	udc_timer.expires = jiffies + HZ/2; /* about 500 ms */
	add_timer(&udc_timer);

	return IRQ_HANDLED;
}

/*
 * Module init and exit
 */
static int __init udc_hotplug_init(void)
{
        int retval;

	kudcd_task = kthread_run(udc_thread, NULL, "kudcd");
	if (IS_ERR(kudcd_task)) {
		printk(KERN_ERR "jz_udc_hotplug: Failed to create system monitor thread.\n");
		return PTR_ERR(kudcd_task);
	}

        retval = request_irq(UDC_HOTPLUG_IRQ, udc_hotplug_irq,
                             IRQF_DISABLED, "udc_hotplug", NULL);
        if (retval) {
                printk("Could not get udc hotplug irq %d\n", UDC_HOTPLUG_IRQ);
		return retval;
        }

        /* get current pin level */
	__gpio_disable_pull(UDC_HOTPLUG_PIN);
        __gpio_as_input(UDC_HOTPLUG_PIN);
	udelay(1);
        udc_pin_level = __gpio_get_pin(UDC_HOTPLUG_PIN);

        if (udc_pin_level) {
		/* Cable has connected, wait for disconnection. */
		__gpio_as_irq_fall_edge(UDC_HOTPLUG_PIN);
        }
        else {
		/* Cable has disconnected, wait for connection. */
		__gpio_as_irq_rise_edge(UDC_HOTPLUG_PIN);
        }

	printk("JZ UDC hotplug driver registered\n");

	return 0;
}

static void __exit udc_hotplug_exit(void)
{
	free_irq(UDC_HOTPLUG_IRQ, NULL);
}

module_init(udc_hotplug_init);
module_exit(udc_hotplug_exit);

EXPORT_SYMBOL(jz_udc_active);

MODULE_AUTHOR("Peter <jlwei@ingenic.cn>");
MODULE_DESCRIPTION("JzSOC OnChip udc hotplug driver");
MODULE_LICENSE("GPL");
