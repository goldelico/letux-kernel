// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform timer support
 *  Copyright (C) 2017, 2019 Paul Boddie <paul@boddie.org.uk>
 *  JZ4730 customisations
 */

#include <linux/export.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/mach-jz4740/base.h>

#ifdef CONFIG_MACH_JZ4730
#include <asm/mach-jz4740/jz4730_timer.h>
#else
#include <asm/mach-jz4740/timer.h>
#endif

void __iomem *jz4740_timer_base;
EXPORT_SYMBOL_GPL(jz4740_timer_base);

void jz4740_timer_enable_watchdog(void)
{
#ifdef CONFIG_MACH_JZ4730
	/* Watchdog not controlled via the timers in the JZ4730. */
#else
	writel(BIT(16), jz4740_timer_base + JZ_REG_TIMER_STOP_CLEAR);
#endif
}
EXPORT_SYMBOL_GPL(jz4740_timer_enable_watchdog);

void jz4740_timer_disable_watchdog(void)
{
#ifdef CONFIG_MACH_JZ4730
	/* Watchdog not controlled via the timers in the JZ4730. */
#else
	writel(BIT(16), jz4740_timer_base + JZ_REG_TIMER_STOP_SET);
#endif
}
EXPORT_SYMBOL_GPL(jz4740_timer_disable_watchdog);

#ifdef CONFIG_MACH_JZ4730
void __init jz4740_timer_init(void)
{
	jz4740_timer_base = ioremap(JZ4730_OST_BASE_ADDR, 0x100);

	if (!jz4740_timer_base)
		panic("Failed to ioremap timer registers");

	/* Disable all timer clocks except for those used as system timers */
	writel(0x00000001, jz4740_timer_base + JZ_REG_TIMER_ENABLE);

	/* Timer irqs are unmasked by default, mask them */
	jz4740_timer_irq_full_disable(0);
	jz4740_timer_irq_full_disable(1);
	jz4740_timer_irq_full_disable(2);
}
#else
void __init jz4740_timer_init(void)
{
	jz4740_timer_base = ioremap(JZ4740_TCU_BASE_ADDR, 0x100);

	if (!jz4740_timer_base)
		panic("Failed to ioremap timer registers");

	/* Disable all timer clocks except for those used as system timers */
	writel(0x000100fc, jz4740_timer_base + JZ_REG_TIMER_STOP_SET);

	/* Timer irqs are unmasked by default, mask them */
	writel(0x00ff00ff, jz4740_timer_base + JZ_REG_TIMER_MASK_SET);
}
#endif
