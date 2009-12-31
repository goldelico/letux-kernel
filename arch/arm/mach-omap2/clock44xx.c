/*
 * OMAP4-specific clock framework functions
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Rajendra Nayak (rnayak@ti.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/cpufreq.h>
#include "clock.h"
#include "opp4xxx.h"

#ifdef CONFIG_CPU_FREQ

extern struct opp_table omap4_vdd1_table[];

static struct cpufreq_frequency_table freq_table[NO_OF_VDD1_OPP + 1];

void omap2_clk_init_cpufreq_table(struct cpufreq_frequency_table **table)
{
	int i = 0;

	for (i = 0; i < NO_OF_VDD1_OPP; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = omap4_vdd1_table[i].freq / 1000;
	}

	if (i == 0) {
		printk(KERN_WARNING "%s: failed to initialize frequency table\n",
								__func__);
		return;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;
	*table = &freq_table[0];
}
#endif

struct clk_functions omap2_clk_functions = {
	.clk_enable		= omap2_clk_enable,
	.clk_disable		= omap2_clk_disable,
	.clk_round_rate		= omap2_clk_round_rate,
	.clk_set_rate		= omap2_clk_set_rate,
	.clk_set_parent		= omap2_clk_set_parent,
	.clk_disable_unused	= omap2_clk_disable_unused,
#ifdef CONFIG_CPU_FREQ
	.clk_init_cpufreq_table	= omap2_clk_init_cpufreq_table,
#endif
};

const struct clkops clkops_noncore_dpll_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
};

void omap2_clk_prepare_for_reboot(void)
{
	return;
}
