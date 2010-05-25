/*
 * OMAP4 Power Management Routines
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <plat/control.h>
#include <plat/clockdomain.h>
#include <plat/powerdomain.h>
#include "prm.h"
#include "pm.h"
#include "cm.h"
#include "cm-regbits-44xx.h"
#include "clock.h"

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);

static struct powerdomain *cpu0_pwrdm, *cpu1_pwrdm, *mpu_pwrdm;

/* This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled. */
int set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}

#ifdef CONFIG_SUSPEND
static int omap4_pm_prepare(void)
{
	disable_hlt();
	return 0;
}

static int omap4_pm_suspend(void)
{
	u32 scu_pwr_st;
	struct power_state *pwrst;
	int state;

	/* Program the CPU to hit RET */
	/* Program the SCU power state register */
	scu_pwr_st = omap_readl(0x48240008);
	scu_pwr_st |= 0x2;
	omap_writel(scu_pwr_st, 0x48240008);
	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node)
		if (strcmp(pwrst->pwrdm->name, "cpu1_pwrdm"))
			if (set_pwrdm_state(pwrst->pwrdm, PWRDM_POWER_RET))
				goto restore;

	isb();
	wmb();
	asm volatile("wfi\n"
		:
		:
		: "memory", "cc");
restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_pwrst(pwrst->pwrdm);
		printk(KERN_INFO "Powerdomain (%s) entered state %d\n",
			pwrst->pwrdm->name, state);
		if (strcmp(pwrst->pwrdm->name, "cpu1_pwrdm"))
			set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	scu_pwr_st = omap_readl(0x48240008);
	scu_pwr_st &= ~0x2;
	omap_writel(scu_pwr_st, 0x48240008);
	return 0;
}

static int omap4_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap4_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap4_pm_finish(void)
{
	enable_hlt();
	return;
}

static int omap4_pm_begin(suspend_state_t state)
{
	return 0;
}

static void omap4_pm_end(void)
{
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap4_pm_begin,
	.end		= omap4_pm_end,
	.prepare	= omap4_pm_prepare,
	.enter		= omap4_pm_enter,
	.finish		= omap4_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */

static int __attribute__ ((unused)) __init
		pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	if (pwrdm_has_hdwr_sar(pwrdm))
		pwrdm_enable_hdwr_sar(pwrdm);

	return set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __attribute__ ((unused)) __init
		clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		omap2_clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		atomic_read(&clkdm->usecount) == 0)
		omap2_clkdm_sleep(clkdm);
	return 0;
}

void  __raw_rmw_reg_bits(u32 mask, u32 bits, const volatile void __iomem *addr)
{
	u32 v;

	v = __raw_readl(addr);
	v &= ~mask;
	v |= bits;
	__raw_writel(v, addr);

}

static void __init prcm_setup_regs(void)
{
	struct clk *dpll_abe_ck, *dpll_core_ck, *dpll_iva_ck;
	struct clk *dpll_mpu_ck, *dpll_per_ck, *dpll_usb_ck;
	struct clk *dpll_unipro_ck;

	/*Enable all the DPLL autoidle */
	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");
	omap3_dpll_allow_idle(dpll_abe_ck);
	dpll_core_ck = clk_get(NULL, "dpll_core_ck");
	omap3_dpll_allow_idle(dpll_core_ck);
	dpll_iva_ck = clk_get(NULL, "dpll_iva_ck");
	omap3_dpll_allow_idle(dpll_iva_ck);
	dpll_mpu_ck = clk_get(NULL, "dpll_mpu_ck");
	omap3_dpll_allow_idle(dpll_mpu_ck);
	dpll_per_ck = clk_get(NULL, "dpll_per_ck");
	omap3_dpll_allow_idle(dpll_per_ck);
	dpll_usb_ck = clk_get(NULL, "dpll_usb_ck");
	omap3_dpll_allow_idle(dpll_usb_ck);
	dpll_unipro_ck = clk_get(NULL, "dpll_unipro_ck");
	omap3_dpll_allow_idle(dpll_unipro_ck);

	/* Enable autogating for all DPLL post dividers */
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_MPU);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M4_DPLL_IVA);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M5_DPLL_IVA);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M3_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M4_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M5_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M6_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M7_DPLL_CORE);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M3_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M4_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M5_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M6_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M7_DPLL_PER);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_ABE);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_ABE);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M3_DPLL_ABE);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_USB);
	__raw_rmw_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
			OMAP4430_CM_DIV_M2_DPLL_UNIPRO);
}

static int __init omap4_pm_init(void)
{
	int ret = 0;

	if (!cpu_is_omap44xx())
		return -ENODEV;

	printk(KERN_INFO "Power Management for TI OMAP4.\n");

#ifdef CONFIG_PM
#if 0
	/* HACK:
	 * This Breaks the Ducati Camera use case
	 * In anycase PM is not enabled on ES1.0 so disable setup
	 */
	prcm_setup_regs();
#endif
	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		printk(KERN_ERR "Failed to setup powerdomains\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);
#endif

	cpu0_pwrdm = pwrdm_lookup("cpu0_pwrdm");
	cpu1_pwrdm = pwrdm_lookup("cpu1_pwrdm");
	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (!cpu0_pwrdm || !cpu1_pwrdm || !mpu_pwrdm) {
		printk(KERN_ERR "Failed to get lookup for MPU pwrdm's\n");
		goto err2;
	}

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	omap4_idle_init();

err2:
	return ret;
}
late_initcall(omap4_pm_init);
