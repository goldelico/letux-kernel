/*
 * OMAP4 voltage scaling functions
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Rajendra Nayak (rnayak@ti.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <plat/common.h>
#include <linux/cpufreq.h>

#include "prm.h"
#include "prm-regbits-44xx.h"
#include "voltage.h"
#include "opp4xxx.h"

#define V_MPU	0x1
#define V_IVA	0x2
#define V_CORE	0x3

/* TODO:This is board specific */
#define PMIC_SLAVE_ID		0x12
#define V_MPU_CFG_FORCE		0x55
#define V_MPU_CFG_VOLT		0x56
#define V_IVA_CFG_FORCE		0x5B
#define V_IVA_CFG_VOLT		0x5C
#define V_CORE_CFG_FORCE	0x61
#define V_CORE_CFG_VOLT		0x62

#define MAX_ACK_WAIT	1000000


static struct vc_config omap4_vc_cfg = {
	.hsmcode	= 0x0,
	/* TODO: These are actualy sysclk dependent */
	.scll		= 0x60,
	.sclh		= 0x26,
	.hsscll		= 0x0a,
	.hssclh		= 0x05,
};

#define CONFIG_VOLTAGE_SCALE_VCBYPASS 1

#ifdef CONFIG_VOLTAGE_SCALE_VCBYPASS
int omap4_voltage_scale(u8 v_domain, u32 vsel)
{
	u32 vc_val_bypass = 0;
	u8 force_reg = 0, volt_reg = 0;
	int i = 0;

	switch (v_domain) {
	case V_MPU:
		force_reg = V_MPU_CFG_FORCE;
		volt_reg = V_MPU_CFG_VOLT;
		break;
	case V_IVA:
		force_reg = V_IVA_CFG_FORCE;
		volt_reg = V_IVA_CFG_VOLT;
		break;
	case V_CORE:
		force_reg = V_CORE_CFG_FORCE;
		volt_reg = V_CORE_CFG_VOLT;
		break;
	default:
		pr_err("Unknow voltage domain\n");
		break;
	}

	/* Set the force register */
	vc_val_bypass |= vsel << OMAP4430_DATA_SHIFT;
	vc_val_bypass |= force_reg << OMAP4430_REGADDR_SHIFT;
	vc_val_bypass |= PMIC_SLAVE_ID << OMAP4430_SLAVEADDR_SHIFT;
	vc_val_bypass |= 1 << OMAP4430_VALID_SHIFT;
	__raw_writel(vc_val_bypass, OMAP4430_PRM_VC_VAL_BYPASS);

	omap_test_timeout(((__raw_readl(OMAP4430_PRM_VC_VAL_BYPASS)
				& OMAP4430_VALID_MASK) == 0),
				MAX_ACK_WAIT, i);

	if (i == MAX_ACK_WAIT)
		pr_err("Failed to scale voltage on VDD%d to %dmv\n",
					v_domain, (600 + (125*vsel/10)));

	/* FIXME:Clear the MPU IRQSTATUS */
	__raw_writel(__raw_readl(OMAP4430_PRM_IRQSTATUS_MPU),
					OMAP4430_PRM_IRQSTATUS_MPU);

	/* Set the voltage register */
	vc_val_bypass &= ~OMAP4430_REGADDR_MASK;
	vc_val_bypass |= volt_reg << OMAP4430_REGADDR_SHIFT;
	__raw_writel(vc_val_bypass, OMAP4430_PRM_VC_VAL_BYPASS);

	omap_test_timeout(((__raw_readl(OMAP4430_PRM_VC_VAL_BYPASS)
				& OMAP4430_VALID_MASK) == 0),
				MAX_ACK_WAIT, i);

	if (i == MAX_ACK_WAIT)
		pr_err("Failed to scale voltage on VDD%d to %dmv\n",
					v_domain, (600 + (125*vsel/10)));

	/* FIXME:Clear the MPU IRQSTATUS */
	__raw_writel(__raw_readl(OMAP4430_PRM_IRQSTATUS_MPU),
					OMAP4430_PRM_IRQSTATUS_MPU);

	return 0;
}
#endif

int omap4_configure_vc(void)
{
	u32 prm_vc_cfg_i2c_mode = 0;
	u32 prm_vc_cfg_i2c_clk = 0;

	/* Configure I2C CLK */
	if (omap4_vc_cfg.hsmcode) {
		prm_vc_cfg_i2c_mode |= omap4_vc_cfg.hsmcode <<
						OMAP4430_HSMCODE_SHIFT;
		prm_vc_cfg_i2c_clk |= omap4_vc_cfg.hsscll <<
						OMAP4430_HSSCLL_SHIFT;
		prm_vc_cfg_i2c_clk |= omap4_vc_cfg.hssclh <<
						OMAP4430_HSSCLH_SHIFT;
	} else {
		/* Disable High speed mode */
		prm_vc_cfg_i2c_mode &= ~(1 << OMAP4430_HSMODEEN_SHIFT);
		/* Disable repeated start operation mode */
		prm_vc_cfg_i2c_mode &= ~(1 << OMAP4430_SRMODEEN_SHIFT);
		prm_vc_cfg_i2c_clk |= omap4_vc_cfg.scll << OMAP4430_SCLL_SHIFT;
		prm_vc_cfg_i2c_clk |= omap4_vc_cfg.sclh << OMAP4430_SCLH_SHIFT;
	}
	__raw_writel(prm_vc_cfg_i2c_mode, OMAP4430_PRM_VC_CFG_I2C_MODE);
	__raw_writel(prm_vc_cfg_i2c_clk, OMAP4430_PRM_VC_CFG_I2C_CLK);

	return 0;
}

void omap4_configure_vp(void)
{}

void omap4_configure_sr(void)
{}

#ifdef CONFIG_CPU_FREQ
static int omap4_vdd1_scale(struct notifier_block *nb,
			unsigned long state, struct cpufreq_freqs *freqs)
{
	int vsel;

	vsel = get_vdd1_voltage(freqs->new * 1000);
	if (vsel == -EINVAL)
		return NOTIFY_DONE;

	switch (state) {
	case CPUFREQ_POSTCHANGE:
		/* Scale voltage if going down */
		if (freqs->old > freqs->new)
			omap4_voltage_scale(V_MPU, vsel);
		break;
	case CPUFREQ_PRECHANGE:
		/* Scale voltage if going up */
		if (freqs->old < freqs->new)
			omap4_voltage_scale(V_MPU, vsel);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block vdd1_nb = {
	.notifier_call = omap4_vdd1_scale,
};
#endif

static int __init omap4_voltage_init(void)
{
	omap4_configure_vc();
	omap4_configure_vp();
	omap4_configure_sr();
#ifdef CONFIG_CPU_FREQ
	/* Register for a Freq scale callback */
	cpufreq_register_notifier(&vdd1_nb, CPUFREQ_TRANSITION_NOTIFIER);
#endif
	return 0;
}

arch_initcall(omap4_voltage_init);
