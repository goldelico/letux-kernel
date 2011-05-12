/**
 * OMAP and TPS PMIC specific intializations.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated.
 * Vishwanath BS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/i2c/twl.h>
#include <linux/delay.h>

#include <plat/voltage.h>

#include "pm.h"
#include "prm44xx.h"
#include "prm-regbits-44xx.h"

#define OMAP4_SRI2C_SLAVE_ADDR		0x60
#define TPS62361_RAMP_ADDR		0x6
#define OMAP4_VDD_MPU_SR_VOLT_REG	0x01
#define OMAP4_VP_CONFIG_ERROROFFSET	0x00
#define OMAP4_VP_VSTEPMIN_VSTEPMIN	0x01
#define OMAP4_VP_VSTEPMAX_VSTEPMAX	0x0A
#define OMAP4_VP_VLIMITTO_TIMEOUT_US	200
#define OMAP4_VP_MPU_VLIMITTO_VDDMIN	0x0D
#define OMAP4_VP_MPU_VLIMITTO_VDDMAX	0x3F

#define SYSEN_CFG_GRP			0x06
#define APE_GRP					BIT(0)
#define	TPS62361_VDCDC1_MIN		500000	/* 0.5V		*/
#define	TPS62361_VDCDC1_STEP	10000	/* 10mV	*/

static u8 omap_tps_onforce_cmd(const u8 vsel)
{
	return vsel;
}

static u8 omap_tps_on_cmd(const u8 vsel)
{
	return vsel;
}

static u8 omap_tps_sleepforce_cmd(const u8 vsel)
{
	return vsel;
}

static u8 omap_tps_sleep_cmd(const u8 vsel)
{
	return vsel;
}

static unsigned long tps6261_vsel_to_uv(const u8 vsel)
{
	return (TPS62361_VDCDC1_MIN + (TPS62361_VDCDC1_STEP * vsel));
}

static u8 tps6261_uv_to_vsel(unsigned long uv)
{
	return DIV_ROUND_UP(uv - TPS62361_VDCDC1_MIN, TPS62361_VDCDC1_STEP);
}

static struct omap_volt_pmic_info omap4_mpu_volt_info = {
	.name = "tps",
	.slew_rate = 8000,
	.step_size = TPS62361_VDCDC1_STEP,
	.i2c_addr = OMAP4_SRI2C_SLAVE_ADDR,
	.i2c_vreg = OMAP4_VDD_MPU_SR_VOLT_REG,
	.vsel_to_uv = tps6261_vsel_to_uv,
	.uv_to_vsel = tps6261_uv_to_vsel,
	.onforce_cmd = omap_tps_onforce_cmd,
	.on_cmd = omap_tps_on_cmd,
	.sleepforce_cmd = omap_tps_sleepforce_cmd,
	.sleep_cmd = omap_tps_sleep_cmd,
	.vp_config_erroroffset = OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin_vstepmin = OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax_vstepmax = OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vlimitto_timeout_us = OMAP4_VP_VLIMITTO_TIMEOUT_US,
	.vp_vlimitto_vddmin = OMAP4_VP_MPU_VLIMITTO_VDDMIN,
	.vp_vlimitto_vddmax = OMAP4_VP_MPU_VLIMITTO_VDDMAX,
};

/**
 * omap4_enable_tps() - Enable tps chip
 *
 * This function enables TPS chip by associating SYSEN signal
 * to APE resource group of TWL6030.
 *
 * Returns 0 on sucess, error is returned if I2C read/write fails.
 */
int __init omap4_tps62361_enable(void)
{
	u8 temp;
	int ret;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &temp,
					SYSEN_CFG_GRP);
	if (ret)
		goto end;
	temp |= APE_GRP;
	ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, temp,
				SYSEN_CFG_GRP);

end:
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
	return ret;
}

int __init omap4_tps62361_init(void)
{
	int ret = 0;
	u16 loop_cnt = 0, retries_cnt = 0;
	u32 vc_bypass_value;

	/* TODO: Due to a sequencing issue with TWL init, commenting this
	 * out as of now as the default association of SYSEN is
	 * enabled by default.
	 */
#if 0
	/* Associate SYSEN to APE resource group (TWL6030 group 1) */
	ret = omap4_tps62361_enable();
#endif

	if (!ret)
		omap_voltage_register_pmic(&omap4_mpu_volt_info, "mpu");

	/* XXX Hack: COnfgirue RMP register for 8mV/uS ramp using vc bypass method */
	vc_bypass_value = (2 << OMAP4430_DATA_SHIFT) | (TPS62361_RAMP_ADDR << OMAP4430_REGADDR_SHIFT) |
		(OMAP4_SRI2C_SLAVE_ADDR << OMAP4430_SLAVEADDR_SHIFT);
	__raw_writel(vc_bypass_value, OMAP4430_PRM_VC_VAL_BYPASS);
	__raw_writel(vc_bypass_value | OMAP4430_VALID_MASK, OMAP4430_PRM_VC_VAL_BYPASS);

	vc_bypass_value = __raw_readl(OMAP4430_PRM_VC_VAL_BYPASS);
	while (vc_bypass_value & OMAP4430_VALID_MASK) {
		loop_cnt++;

		if (retries_cnt > 10) {
			pr_warning("%s: Retry count exceeded\n", __func__);
				return -ETIMEDOUT;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
	}

	return ret;
}
