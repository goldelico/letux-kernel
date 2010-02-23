/*
 *  linux/arch/arm/mach-omap2/clock.c
 *
 *  Copyright (C) 2005-2008 Texas Instruments, Inc.
 *  Copyright (C) 2004-2008 Nokia Corporation
 *
 *  Contacts:
 *  Richard Woodruff <r-woodruff2@ti.com>
 *  Paul Walmsley
 *
 *  Based on earlier work by Tuukka Tikkanen, Tony Lindgren,
 *  Gordon McNutt and RidgeRun, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/bitops.h>

#include <plat/clock.h>
#include <plat/sram.h>
#include <plat/prcm.h>
#include <plat/clkdev_omap.h>
#include <asm/div64.h>
#include <asm/clkdev.h>

#include <plat/sdrc.h>
#include "clock.h"
#include "clock2xxx.h"
#include "opp2xxx.h"
#include "prm.h"
#include "prm-regbits-24xx.h"
#include "cm.h"
#include "cm-regbits-24xx.h"

struct clk *vclk, *sclk, *dclk;

/*-------------------------------------------------------------------------
 * Omap24xx specific clock functions
 *-------------------------------------------------------------------------*/

/*
 * Set clocks for bypass mode for reboot to work.
 */
void omap2xxx_clk_prepare_for_reboot(void)
{
	u32 rate;

	if (vclk == NULL || sclk == NULL)
		return;

	rate = clk_get_rate(sclk);
	clk_set_rate(vclk, rate);
}

/*
 * Switch the MPU rate if specified on cmdline.  We cannot do this
 * early until cmdline is parsed.  XXX This should be removed from the
 * clock code and handled by the OPP layer code in the near future.
 */
static int __init omap2xxx_clk_arch_init(void)
{
	int ret;

	if (!cpu_is_omap24xx())
		return 0;

	ret = omap2_clk_switch_mpurate_at_boot("virt_prcm_set");
	if (!ret)
		omap2_clk_print_new_rates("sys_ck", "dpll_ck", "mpu_ck");

	return ret;
}

arch_initcall(omap2xxx_clk_arch_init);


