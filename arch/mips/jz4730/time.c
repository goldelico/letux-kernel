/*
 *  linux/arch/mips/jz4730/time.c
 *
 *  Setting up the clock on the JZ4730 boards.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/clockchips.h>

#include <asm/time.h>
#include <asm/jzsoc.h>

#define JZ_TIMER_CHAN  0
#define JZ_TIMER_IRQ   IRQ_OST0

static unsigned int timer_latch;

void (*jz_timer_callback)(void);

static void jz_set_mode(enum clock_event_mode mode,
			struct clock_event_device *evt)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
                break;
        case CLOCK_EVT_MODE_ONESHOT:
        case CLOCK_EVT_MODE_UNUSED:
        case CLOCK_EVT_MODE_SHUTDOWN:
                break;
        case CLOCK_EVT_MODE_RESUME:
                break;
        }
}

static struct clock_event_device jz_clockevent_device = {
	.name		= "jz-timer",
	.features	= CLOCK_EVT_FEAT_PERIODIC,

	/* .mult, .shift, .max_delta_ns and .min_delta_ns left uninitialized */

	.rating		= 300,
	.irq		= JZ_TIMER_IRQ,
	.set_mode	= jz_set_mode,
};

static irqreturn_t jz_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;

	__ost_clear_uf(JZ_TIMER_CHAN); /* ACK timer */

	if (jz_timer_callback)
		jz_timer_callback();

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static struct irqaction jz_irqaction = {
	.handler	= jz_timer_interrupt,
	.flags		= IRQF_DISABLED | IRQF_PERCPU,
	.name		= "jz-timer",
};

static void __init jz_timer_setup(void)
{
	struct clock_event_device *cd = &jz_clockevent_device;
	struct irqaction *action = &jz_irqaction;
	unsigned int cpu = smp_processor_id();

	cd->cpumask = cpumask_of_cpu(cpu);
	clockevents_register_device(cd);
	action->dev_id = cd;
	setup_irq(JZ_TIMER_IRQ, &jz_irqaction);
}

void __init plat_time_init(void)
{
	/* Init timer */
	timer_latch = (JZ_EXTAL + (HZ>>1)) / HZ;
	__ost_set_mode(JZ_TIMER_CHAN, OST_TCSR_UIE | OST_TCSR_CKS_EXTAL);
	__ost_set_reload(JZ_TIMER_CHAN, timer_latch);
	__ost_set_count(JZ_TIMER_CHAN, timer_latch);
	__ost_enable_channel(JZ_TIMER_CHAN);

	jz_timer_setup();
}
