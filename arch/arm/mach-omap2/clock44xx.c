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
#include <linux/io.h>
#include <linux/delay.h>
#include <plat/common.h>
#include "clock.h"
#include "opp4xxx.h"
#include "cm.h"
#include "cm-regbits-44xx.h"

#define MAX_FREQ_UPDATE_WAIT  100000
#define MAX_DPLL_LOCK_WAIT    100000

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

/**
 * omap4_core_dpll_m2_set_rate - set CORE DPLL M2 divider
 * @clk: struct clk * of DPLL to set
 * @rate: rounded target rate
 *
 * Programs the CM shadow registers to update CORE DPLL M2
 * divider. Also updates EMIF parameters.
 * Returns -EINVAL/-1 on error and 0 on success.
 */
int omap4_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate)
{
	int i = 0;
	u32 validrate = 0, shadow_freq_cfg1 = 0, new_div = 0;

	if (!clk || !rate)
		return -EINVAL;

	validrate = omap2_clksel_round_rate_div(clk, rate, &new_div);
	if (validrate != rate)
		return -EINVAL;

	/* Use Shadow registers and Freq update method */

	/* reset the dll after freq change */
	shadow_freq_cfg1 |= 1 << OMAP4430_DLL_RESET_SHIFT;
	/* Set the DLL override */
	shadow_freq_cfg1 |= 1 << OMAP4430_DLL_OVERRIDE_SHIFT;
	/* set the new M2 value */
	shadow_freq_cfg1 |= new_div << OMAP4430_DPLL_CORE_M2_DIV_SHIFT;
	/* lock the dpll after freq change */
	shadow_freq_cfg1 |= DPLL_LOCKED << OMAP4430_DPLL_CORE_DPLL_EN_SHIFT;
	/* Specify to h/w a new config is available */
	shadow_freq_cfg1 |= 1 << OMAP4430_FREQ_UPDATE_SHIFT;
	__raw_writel(shadow_freq_cfg1, OMAP4430_CM_SHADOW_FREQ_CONFIG1);

	/* wait for the configuration to be applied */
	omap_test_timeout(((__raw_readl(OMAP4430_CM_SHADOW_FREQ_CONFIG1)
				& OMAP4430_FREQ_UPDATE_MASK) == 0),
				MAX_FREQ_UPDATE_WAIT, i);

	if (i == MAX_FREQ_UPDATE_WAIT) {
		pr_err("%s: Frequency update for CORE DPLL M2 change failed\n",
				__func__);
		return -1;
	}

	i = 0;

	/* wait for the DPLL to lock */
	omap_test_timeout(((__raw_readl(OMAP4430_CM_CLKMODE_DPLL_CORE)
			& OMAP4430_DPLL_EN_MASK) == DPLL_LOCKED),
			MAX_DPLL_LOCK_WAIT, i);

	if (i == MAX_DPLL_LOCK_WAIT) {
		pr_err("%s: Unable to lock CORE DPLL post frequency update\n",
				__func__);
		return -1;
	}
	return 0;
}
