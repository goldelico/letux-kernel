/*
 * linux/arch/mips/jz4740/time.c
 * 
 * Setting up the clock on the JZ4740 boards.
 * 
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/clockchips.h>

#include <asm/time.h>
#include <asm/jzsoc.h>

/* This is for machines which generate the exact clock. */

#define JZ_TIMER_CHAN  0
#define JZ_TIMER_IRQ  IRQ_TCU0

static unsigned int latch;

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

	REG_TCU_TFCR = 1 << JZ_TIMER_CHAN; /* ACK timer */

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
	latch = ( (JZ_EXTAL>>4)  + (HZ>>1)) / HZ;

	REG_TCU_TCSR(JZ_TIMER_CHAN) = TCU_TCSR_PRESCALE16 | TCU_TCSR_EXT_EN;
	REG_TCU_TCNT(JZ_TIMER_CHAN) = 0;
	REG_TCU_TDHR(JZ_TIMER_CHAN) = 0;
	REG_TCU_TDFR(JZ_TIMER_CHAN) = latch;

	REG_TCU_TMSR = (1 << (JZ_TIMER_CHAN + 16)); /* mask half irq */
	REG_TCU_TMCR = (1 << JZ_TIMER_CHAN); /* unmask full irq */
	REG_TCU_TSCR = (1 << JZ_TIMER_CHAN); /* enable timer clock */
	REG_TCU_TESR = (1 << JZ_TIMER_CHAN); /* start counting up */

	jz_timer_setup();
}
