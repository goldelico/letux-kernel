/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/i2c/twl4030.h>
#include <linux/io.h>

#include <mach/omap34xx.h>
#include <mach/control.h>
#include <mach/clock.h>
#include <mach/omap-pm.h>

#include "prm.h"
#include "smartreflex.h"
#include "prm-regbits-34xx.h"

/* The timeout values have been measured using 32Khz timer
 * for rover codebase by Nishant Menon. It is found to be
 * working for Zoom2 also. Should get proper timeout values
 * from h/w team. The SR disbale time out value 3472 was found
 * to be insufficient on zoom2 where we get occasional timeouts.
 * Increasing it to a safer value.
*/
#define SR_DISABLE_TIMEOUT	10000
#define VP_TRANXDONE_TIMEOUT	62
#define VP_IDLE_TIMEOUT		3472

#define INTC_MIR0               0x48200084
#define INTC_MIR_CLEAR0         0x48200088
#define INTC_MIR_SET0           0x4820008C
#define INTC_SR1		(0x1 << 18)
#define INTC_SR2		(0x1 << 19)
#define ERRCONFIG_STATUS_MASK	(ERRCONFIG_VPBOUNDINTST | \
		 ERRCONFIG_MCUBOUNDINTST | ERRCONFIG_MCUDISACKINTST)



struct omap_sr {
	int		srid;
	int		is_sr_reset;
	int		is_autocomp_active;
	struct clk	*clk;
	u32		clk_length;
	u32		req_opp_no;
	u32		opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue;
	u32		opp5_nvalue, opp6_nvalue;
	u32		senp_mod, senn_mod;
	void __iomem	*srbase_addr;
	void __iomem	*vpbase_addr;
	u32		starting_ret_volt;
};

#define SR_REGADDR(offs)	(sr->srbase_addr + offset)

static struct clk *dpll1_ck, *l3_ick;

static omap3_voltagescale_vcbypass_t omap3_volscale_vcbypass_fun;

static inline void sr_write_reg(struct omap_sr *sr, unsigned offset, u32 value)
{
	__raw_writel(value, SR_REGADDR(offset));
}

static inline void sr_modify_reg(struct omap_sr *sr, unsigned offset, u32 mask,
					u32 value)
{
	u32 reg_val;
	if (offset == ERRCONFIG)
		mask |= ERRCONFIG_STATUS_MASK;

	reg_val = __raw_readl(SR_REGADDR(offset));
	reg_val &= ~mask;
	reg_val |= value;

	__raw_writel(reg_val, SR_REGADDR(offset));
}

static inline u32 sr_read_reg(struct omap_sr *sr, unsigned offset)
{
	return __raw_readl(SR_REGADDR(offset));
}

static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->clk) != 0) {
		pr_err("Could not enable %s\n", sr->clk->name);
		return -1;
	}

	if (cpu_is_omap3630())
		sr_modify_reg(sr, ERRCONFIG, SR_IDLEMODE_MASK,
				SR_SMART_IDLE);
	else
	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
		      SR_CLKACTIVITY_IOFF_FON);

	return 0;
}

static void sr_clk_disable(struct omap_sr *sr)
{
	if (cpu_is_omap3630())
		sr_modify_reg(sr, ERRCONFIG, SR_IDLEMODE_MASK,
		SR_FORCE_IDLE);
	else
	/* set fclk, iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
		      SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr->clk);
	sr->is_sr_reset = 1;
}

static struct omap_sr sr1 = {
	.srid			= SR1,
	.is_sr_reset		= 1,
	.is_autocomp_active	= 0,
	.clk_length		= 0,
	.srbase_addr		= OMAP2_IO_ADDRESS(OMAP34XX_SR1_BASE),
};

static struct omap_sr sr2 = {
	.srid			= SR2,
	.is_sr_reset		= 1,
	.is_autocomp_active	= 0,
	.clk_length		= 0,
	.srbase_addr		= OMAP2_IO_ADDRESS(OMAP34XX_SR2_BASE),
};

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}

static u32 cal_test_nvalue(u32 sennval, u32 senpval)
{
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;

	/* Calculating the gain and reciprocal of the SenN and SenP values */
	cal_reciprocal(senpval, &senpgain, &rnsenp);
	cal_reciprocal(sennval, &senngain, &rnsenn);

	return ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));
}

void sr_calculate_rg(u32 rfuse, u32 gain_fuse, u32 delta_nt,
					u32 *rnsen, u32 *sengain)
{
	u32 nadj;
	nadj = ((1 << (gain_fuse + 8)) / rfuse) + delta_nt;
	cal_reciprocal(nadj, sengain, rnsen);
}

/* extrapolate OPP6  nvalues from OPP5 value */
static u32 calculate_opp_nvalue(u32 opp5_nvalue, u32 delta_p, u32 delta_n)
{
	u32 sen_pgain_fuse, sen_ngain_fuse, sen_prn_fuse, sen_nrn_fuse;
	u32 sen_nrn, sen_ngain, sen_prn, sen_pgain;
	sen_pgain_fuse = (opp5_nvalue & 0x00F0000) >> 0x14;
	sen_ngain_fuse = (opp5_nvalue & 0x000F0000) >> 0x10;
	sen_prn_fuse = (opp5_nvalue & 0x0000FF00) >> 0x08;
	sen_nrn_fuse = (opp5_nvalue & 0x000000FF);
	sr_calculate_rg(sen_nrn_fuse, sen_ngain_fuse, delta_n, &sen_nrn,
					&sen_ngain);
	sr_calculate_rg(sen_prn_fuse, sen_pgain_fuse, delta_p, &sen_prn,
					&sen_pgain);
	return  (sen_pgain << 0x14) | (sen_ngain << 0x10)
			| (sen_prn << 0x08) | (sen_nrn);
}

/* determine the current OPP from the frequency
 * we need to give this function last element of OPP rate table
 * and the frequency
 */
static u16 get_opp(struct omap_opp *opp_freq_table,
					unsigned long freq)
{
	struct omap_opp *prcm_config;

	prcm_config = opp_freq_table;

	if (prcm_config->rate <= freq)
		return prcm_config->opp_id; /* Return the Highest OPP */
	for (; prcm_config->rate; prcm_config--)
		if (prcm_config->rate < freq)
			return (prcm_config+1)->opp_id;
		else if (prcm_config->rate == freq)
			return prcm_config->opp_id;
	/* Return the least OPP */
	return (prcm_config+1)->opp_id;
}

static inline u16 get_vdd1_opp(void)
{
	return get_opp(mpu_opps + MAX_VDD1_OPP, dpll1_ck->rate);
}

static inline u16 get_vdd2_opp(void)
{
	return get_opp(l3_opps + MAX_VDD2_OPP, l3_ick->rate);
}


static void sr_set_clk_length(struct omap_sr *sr)
{
	struct clk *sys_ck;
	u32 sys_clk_speed;

	sys_ck = clk_get(NULL, "sys_ck");
	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);

	switch (sys_clk_speed) {
	case 12000000:
		sr->clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000000:
		sr->clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200000:
		sr->clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000000:
		sr->clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400000:
		sr->clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default :
		pr_err("Invalid sysclk value: %d\n", sys_clk_speed);
		break;
	}
}

static void sr_set_efuse_nvalues(struct omap_sr *sr)
{
	if (sr->srid == SR1) {
		sr->senn_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENNENABLE_MASK) >>
					OMAP343X_SR1_SENNENABLE_SHIFT;
		sr->senp_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENPENABLE_MASK) >>
					OMAP343X_SR1_SENPENABLE_SHIFT;

		sr->opp5_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP5_VDD1);
		sr->opp4_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP4_VDD1);
		sr->opp3_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD1);
		sr->opp2_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD1);
		sr->opp1_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD1);
		if (sr->opp5_nvalue) {
			sr->opp6_nvalue = calculate_opp_nvalue(sr->opp5_nvalue,
			227, 379);
		}
	} else if (sr->srid == SR2) {
		sr->senn_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENNENABLE_MASK) >>
					OMAP343X_SR2_SENNENABLE_SHIFT;

		sr->senp_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENPENABLE_MASK) >>
					OMAP343X_SR2_SENPENABLE_SHIFT;

		sr->opp3_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD2);
		sr->opp2_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD2);
		sr->opp1_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD2);
	}
}

/* Hard coded nvalues for testing purposes, may cause device to hang! */
static void sr_set_testing_nvalues(struct omap_sr *sr)
{
	if (sr->srid == SR1) {
		if (cpu_is_omap3630()) {
			sr->senp_mod = 0x1;
			sr->senn_mod = 0x1;

			/* calculate nvalues for each opp */
			sr->opp1_nvalue = cal_test_nvalue(557, 470);
			sr->opp2_nvalue = cal_test_nvalue(997, 844);
			sr->opp3_nvalue = cal_test_nvalue(1556, 1330);
			sr->opp4_nvalue = cal_test_nvalue(1728, 1480);
		} else {
		sr->senp_mod = 0x03;	/* SenN-M5 enabled */
		sr->senn_mod = 0x03;

		/* calculate nvalues for each opp */
			sr->opp1_nvalue = cal_test_nvalue(0x373 + 0x100,
							0x28c + 0x100);
			sr->opp2_nvalue = cal_test_nvalue(0x506 + 0x1a0,
							0x3be + 0x1a0);
			sr->opp3_nvalue = cal_test_nvalue(0x85b + 0x200,
							0x655 + 0x200);
			sr->opp4_nvalue = cal_test_nvalue(0x964 + 0x2a0,
							0x727 + 0x2a0);
			sr->opp5_nvalue = cal_test_nvalue(0xacd + 0x330,
							0x848 + 0x330);
		}
		if (sr->opp5_nvalue) {
			sr->opp6_nvalue = calculate_opp_nvalue(sr->opp5_nvalue,
			227, 379);
		}
	} else if (sr->srid == SR2) {
		if (cpu_is_omap3630()) {
			sr->senp_mod = 0x1;
			sr->senn_mod = 0x1;

			sr->opp1_nvalue = cal_test_nvalue(556, 468);
			sr->opp2_nvalue = cal_test_nvalue(1099, 933);

		} else {
			sr->senp_mod = 0x03;
			sr->senn_mod = 0x03;

			sr->opp1_nvalue = cal_test_nvalue(0x359, 0x25d);
			sr->opp2_nvalue = cal_test_nvalue(0x4f5 + 0x1c0,
							0x390 + 0x1c0);
			sr->opp3_nvalue = cal_test_nvalue(0x76f + 0x200,
							0x579 + 0x200);
		}

	}

}

static void sr_set_nvalues(struct omap_sr *sr)
{
	if (SR_TESTING_NVALUES)
		sr_set_testing_nvalues(sr);
	else
		sr_set_efuse_nvalues(sr);
}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;
	u32 vsel;
	u32 target_opp_no;

	if (srid == SR1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no) {
			/* Assume Nominal OPP as current OPP unknown */
			vsel = mpu_opps[VDD1_OPP3].vsel;
			target_opp_no = VDD1_OPP3;
		}
		else
			vsel = mpu_opps[target_opp_no].vsel;

		vpconfig = PRM_VP1_CONFIG_ERROROFFSET |
			OMAP3430_TIMEOUTEN |
			vsel << OMAP3430_INITVOLTAGE_SHIFT |
			((target_opp_no < VDD1_OPP3)
					? PRM_VP1_CONFIG_ERRORGAIN_OPPLOW
					: PRM_VP1_CONFIG_ERRORGAIN_OPPHIGH);
		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP1_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP1_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP1_VLIMITTO_VDDMAX |
					PRM_VP1_VLIMITTO_VDDMIN |
					PRM_VP1_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);

		/* Clear initVDD copy trigger bit */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);

		/* Force update of voltage */
		prm_set_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* Clear force bit */
		prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);

	} else if (srid == SR2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no) {
			/* Assume Nominal OPP */
			vsel = l3_opps[VDD2_OPP3].vsel;
			target_opp_no = VDD2_OPP3;
		}
		else
			vsel = l3_opps[target_opp_no].vsel;

		vpconfig = PRM_VP2_CONFIG_ERROROFFSET |
			OMAP3430_TIMEOUTEN |
			vsel << OMAP3430_INITVOLTAGE_SHIFT |
			((target_opp_no < VDD2_OPP3)
					? PRM_VP2_CONFIG_ERRORGAIN_OPPLOW
					: PRM_VP2_CONFIG_ERRORGAIN_OPPHIGH);
		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP2_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP2_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP2_VLIMITTO_VDDMAX |
					PRM_VP2_VLIMITTO_VDDMIN |
					PRM_VP2_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);

		/* Clear initVDD copy trigger bit */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);

		/* Force update of voltage */
		prm_set_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* Clear force bit */
		prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);

	}
}

static void sr_configure(struct omap_sr *sr, u32 target_opp)
{
	u32 sr_config;
	u32 senp_en , senn_en;

	if (sr->clk_length == 0)
		sr_set_clk_length(sr);

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;
	if (sr->srid == SR1) {
		if (cpu_is_omap3630()) {
			sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT_36XX) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT_36XX);

			sr_write_reg(sr, SRCONFIG, sr_config);
			sr_write_reg(sr, AVGWEIGHT,
				SR1_AVGWEIGHT_SENPAVGWEIGHT |
				SR1_AVGWEIGHT_SENNAVGWEIGHT);

			sr_modify_reg(sr, ERRCONFIG_36XX, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT |
			((target_opp < VDD1_OPP3) ? SR1_ERRMINLIMIT_OPPLOW
			: SR1_ERRMINLIMIT_OPPHIGH)));
		} else {
			sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

			sr_write_reg(sr, SRCONFIG, sr_config);
			sr_write_reg(sr, AVGWEIGHT,
					SR1_AVGWEIGHT_SENPAVGWEIGHT |
					SR1_AVGWEIGHT_SENNAVGWEIGHT);

			sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT |
			((target_opp < VDD1_OPP3) ? SR1_ERRMINLIMIT_OPPLOW
			: SR1_ERRMINLIMIT_OPPHIGH)));
		}
	} else if (sr->srid == SR2) {
		if (cpu_is_omap3630()) {
			sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT_36XX) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT_36XX);

			sr_write_reg(sr, SRCONFIG, sr_config);
			sr_write_reg(sr, AVGWEIGHT,
					SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);
			sr_modify_reg(sr, ERRCONFIG_36XX, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT |
			((target_opp < VDD2_OPP3) ? SR2_ERRMINLIMIT_OPPLOW
			: SR2_ERRMINLIMIT_OPPHIGH)));

		} else {
			sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

			sr_write_reg(sr, SRCONFIG, sr_config);
			sr_write_reg(sr, AVGWEIGHT,
					SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);
			sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT |
			((target_opp < VDD2_OPP3) ? SR2_ERRMINLIMIT_OPPLOW
			: SR2_ERRMINLIMIT_OPPHIGH)));
		}
	}
	sr->is_sr_reset = 0;
}

static int sr_reset_voltage(int srid)
{
	u32 target_opp_no, vsel = 0;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_bypass_value;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;
	u32 prm_vp1_voltage, prm_vp2_voltage;

	if (srid == SR1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no) {
			pr_info("Current OPP unknown: Cannot reset voltage\n");
			return 1;
		}
		vsel = mpu_opps[target_opp_no].vsel;
		reg_addr = R_VDD1_SR_CONTROL;
		prm_vp1_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP1_VOLTAGE_OFFSET);
		t2_smps_steps = abs(vsel - prm_vp1_voltage);
	} else if (srid == SR2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no) {
			pr_info("Current OPP unknown: Cannot reset voltage\n");
			return 1;
		}
		vsel = l3_opps[target_opp_no].vsel;
		reg_addr = R_VDD2_SR_CONTROL;
		prm_vp2_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP2_VOLTAGE_OFFSET);
		t2_smps_steps = abs(vsel - prm_vp2_voltage);
	}

	vc_bypass_value = (vsel << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_info("Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	/*
	 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
	 *  2us added as buffer.
	 */
	t2_smps_delay = ((t2_smps_steps * 125) / 40) + 2;
	udelay(t2_smps_delay);

	return 0;
}

static int sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, v;

	if (!(mpu_opps && l3_opps)) {
		pr_notice("VSEL values not found\n");
		return false;
	}

	sr->req_opp_no = target_opp_no;

	if (sr->srid == SR1) {
		switch (target_opp_no) {
		case 6:
			nvalue_reciprocal = sr->opp6_nvalue;
			break;
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	} else {
		switch (target_opp_no) {
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	}

	if (nvalue_reciprocal == 0) {
		pr_notice("OPP%d doesn't support SmartReflex \n",
								target_opp_no);
		return false;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);
	if (cpu_is_omap3630()) {
		/* Enable the interrupt */
		sr_modify_reg(sr, ERRCONFIG_36XX,
				(ERRCONFIG_VPBOUNDINTEN_36XX |
				ERRCONFIG_VPBOUNDINTST_36XX),
				(ERRCONFIG_VPBOUNDINTEN_36XX |
				ERRCONFIG_VPBOUNDINTST_36XX));
	} else {
	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST));
	}
	if (sr->srid == SR1) {
		/* set/latch init voltage */
		v = prm_read_mod_reg(OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		v &= ~(OMAP3430_INITVOLTAGE_MASK | OMAP3430_INITVDD);
		v |= mpu_opps[target_opp_no].vsel <<
			OMAP3430_INITVOLTAGE_SHIFT;
		prm_write_mod_reg(v, OMAP3430_GR_MOD,
				  OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* write1 to latch */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* write2 clear */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* Enable VP1 */
		prm_set_mod_reg_bits(OMAP3430_VPENABLE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
	} else if (sr->srid == SR2) {
		/* set/latch init voltage */
		v = prm_read_mod_reg(OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		v &= ~(OMAP3430_INITVOLTAGE_MASK | OMAP3430_INITVDD);
		v |= l3_opps[target_opp_no].vsel <<
			OMAP3430_INITVOLTAGE_SHIFT;
		prm_write_mod_reg(v, OMAP3430_GR_MOD,
				  OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* write1 to latch */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* write2 clear */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* Enable VP2 */
		prm_set_mod_reg_bits(OMAP3430_VPENABLE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);
	return true;
}

static int vp_disable(struct omap_sr *sr)
{
	u32 vp_config_offs, vp_status_offs, vp_tranxdone_st;
	int timeout = 0;

	if (sr->srid == SR1) {
		vp_config_offs = OMAP3_PRM_VP1_CONFIG_OFFSET;
		vp_status_offs = OMAP3_PRM_VP1_STATUS_OFFSET;
		vp_tranxdone_st = OMAP3430_VP1_TRANXDONE_ST;
	} else if (sr->srid == SR2) {
		vp_config_offs = OMAP3_PRM_VP2_CONFIG_OFFSET;
		vp_status_offs = OMAP3_PRM_VP2_STATUS_OFFSET;
		vp_tranxdone_st = OMAP3430_VP2_TRANXDONE_ST;
	} else {
		pr_warning("Wrong SR id. SR %d does not exost\n", sr->srid);
		return -1;
	}

	while (timeout < VP_TRANXDONE_TIMEOUT) {
		prm_write_mod_reg(vp_tranxdone_st, OCP_MOD,
				  OMAP2_PRM_IRQSTATUS_MPU_OFFSET);
		if (!(prm_read_mod_reg(OCP_MOD, OMAP2_PRM_IRQSTATUS_MPU_OFFSET)
						& vp_tranxdone_st))
			break;

		udelay(1);
		timeout++;
	}

	if (timeout == VP_TRANXDONE_TIMEOUT)
		pr_warning("VP:TRANXDONE timeout exceeded still going ahead\
					with disabling VP%d\n", sr->srid);

	/* Disable VP */
	prm_clear_mod_reg_bits(OMAP3430_VPENABLE, OMAP3430_GR_MOD,
			       vp_config_offs);

	/* Wait for VP to be in IDLE - typical latency < 1 microsecond */
	timeout = 0;
	while (timeout < VP_IDLE_TIMEOUT &&
	       !(prm_read_mod_reg(OMAP3430_GR_MOD, vp_status_offs) &
		 OMAP3430_VPINIDLE)) {
		udelay(1);
		timeout++;
	}
	if (timeout == VP_IDLE_TIMEOUT)
		pr_warning("VP%d not idle timedout\n", sr->srid);
	return 0;
}


static int sr_disable(struct omap_sr *sr)
{
	int c = 0;
	u32 sr_int_status = 0;

	/*Disable VP before disabling SR */
	if (vp_disable(sr))
		return -1;

	sr->is_sr_reset = 1;
	/* Check to see if SR is already disabled.
	 * If so do nothing and return
	 */
	if (!(sr_read_reg(sr, SRCONFIG) & SRCONFIG_SRENABLE))
		return 0;

	/*Check if SR interrupt is enabled at INTC level */
	if (sr->srid == SR1)
		sr_int_status = omap_readl(INTC_MIR0) & INTC_SR1;
	else if (sr->srid == SR2)
		sr_int_status = omap_readl(INTC_MIR0) & INTC_SR2;

	/* Mask the SR1 or SR2 interrupt at INTC level if unmasked
	 * so that the isr is not called. If this step is
	 * is not done random spurious SR1/SR2 interrupts or
	 * system suspend not working due to spurious interrupts
	 * can be ovbserved.
	 */
	if (!sr_int_status) {
		if (sr->srid == SR1)
			omap_writel(INTC_SR1, INTC_MIR_SET0);
		if (sr->srid == SR2)
			omap_writel(INTC_SR2, INTC_MIR_SET0);
		/* DSB is essential as none of the memory is
		 * Strongly ordered
		 */
		dsb();
	}
	/* Enable MCUDisableAcknowledge interrupt, Disable VPBOUND interrupt
	 * and Clean VPBOUNT interrupt status
	 */
	if (cpu_is_omap3430())
		sr_modify_reg(sr, ERRCONFIG,
			ERRCONFIG_MCUDISACKINTEN | ERRCONFIG_VPBOUNDINTEN,
			ERRCONFIG_MCUDISACKINTEN | ERRCONFIG_VPBOUNDINTST);

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, ~SRCONFIG_SRENABLE);

	/* Wait for SR to be disabled.
	 * wait until ERRCONFIG.MCUDISACKINTST = 1
	 * Typical latency is < 1us.
	 */
	if (cpu_is_omap3430()) {
		while ((c < SR_DISABLE_TIMEOUT) &&
			(!(sr_read_reg(sr, ERRCONFIG) &
				ERRCONFIG_MCUDISACKINTST))) {
			udelay(1);
			c++;
		}
		if (c == SR_DISABLE_TIMEOUT)
			pr_warning("SR%d not disabled\n", sr->srid);


		/* Disable MCUDisableAcknowledge interrupt &
		 * clear pending interrupt Also enable VPBOUND interrrupt
		 */
		sr_modify_reg(sr, ERRCONFIG, ERRCONFIG_MCUDISACKINTEN,
			ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_MCUDISACKINTST);
	}

	/* DSB is essential as none of the memory is Strongly ordered */
	dsb();
	/* Wait till ERRCONFIG_MCUDISACKINTST is cleared before unmasking
	 * SR interrupts. Else we can have spurious interrupts
	 */
	if (cpu_is_omap3430())
		while (sr_read_reg(sr, ERRCONFIG) & ERRCONFIG_MCUDISACKINTST)
			;

	if (!sr_int_status) {
		/* Unmask the SR interrrupts if previously unmasked*/
		if (sr->srid == SR1)
			omap_writel(INTC_SR1, INTC_MIR_CLEAR0);
		if (sr->srid == SR2)
			omap_writel(INTC_SR2, INTC_MIR_CLEAR0);
	}
	return 0;
}

static void change_ret_volt(struct omap_sr *sr, u32 val)
{
	u32 prm_vc_cmd_val = 0;
	if (sr == &sr1) {
		prm_vc_cmd_val = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_CMD_VAL_0_OFFSET);
		prm_write_mod_reg((prm_vc_cmd_val &
					~OMAP3430_VC_CMD_RET_MASK) |
				(val << OMAP3430_VC_CMD_RET_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_0_OFFSET);
	}

	if (sr == &sr2) {
		prm_vc_cmd_val = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_CMD_VAL_1_OFFSET);
		prm_write_mod_reg((prm_vc_cmd_val &
					~OMAP3430_VC_CMD_RET_MASK) |
				(val << OMAP3430_VC_CMD_RET_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_1_OFFSET);
	}

	return;
}

static void try_change_ret_volt(struct omap_sr *sr, u32 *cmd_volt)
{
	u32 vp_volt = 0;

	if (sr != &sr1 && sr != &sr2)
		return;

	if (sr == &sr1 && get_vdd1_opp() == 1)
		vp_volt = prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VP1_VOLTAGE_OFFSET);

	if (sr == &sr2 && get_vdd2_opp() == 2)
		vp_volt = prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VP2_VOLTAGE_OFFSET);

	if (*cmd_volt != vp_volt) {
		change_ret_volt(sr, vp_volt);
		*cmd_volt = prm_read_mod_reg(OMAP3430_GR_MOD,
				(sr == &sr1) ? OMAP3_PRM_VP1_VOLTAGE_OFFSET :
					OMAP3_PRM_VP2_VOLTAGE_OFFSET);
	}

	return;
}

static u32 get_ret_volt(struct omap_sr *sr)
{
	if (sr == &sr1)
		return (prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_0_OFFSET) &
				OMAP3430_VC_CMD_RET_MASK) >>
			OMAP3430_VC_CMD_RET_SHIFT;

	if (sr == &sr2)
		return (prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_1_OFFSET) &
				OMAP3430_VC_CMD_RET_MASK) >>
			OMAP3430_VC_CMD_RET_SHIFT;

	return -EINVAL;
}


void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;
	else
		return;

	if (sr->is_sr_reset == 1) {
		if (sr_clk_enable(sr))
			return;
		sr_configure(sr, target_opp_no);
	}

	if (sr->is_autocomp_active == 1) {
		pr_warning("SR%d: VDD autocomp is already active\n",
									srid);
		return;
	}

	sr->is_autocomp_active = 1;
	if (!sr_enable(sr, target_opp_no)) {
		pr_warning("SR%d: VDD autocomp not activated\n", srid);
		sr->is_autocomp_active = 0;
		if (sr->is_sr_reset == 1)
			sr_clk_disable(sr);
	}
}
EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;
	else
		return -EINVAL;

	if (sr->is_autocomp_active == 1) {
		if (sr_disable(sr))
			pr_warning("Problems in SR Disable!!!!\n");
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		/* Reset the volatage for current OPP */
		sr_reset_voltage(srid);
		change_ret_volt(sr, sr->starting_ret_volt);
		return true;
	}
	return false;
}
EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;
	else
		return;

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			/* Enable SR clks */
			if (sr_clk_enable(sr))
				return;

			if (srid == SR1)
				target_opp_no = get_vdd1_opp();
			else if (srid == SR2)
				target_opp_no = get_vdd2_opp();

			if (!target_opp_no) {
				pr_info("Current OPP unknown \
						 Cannot configure SR\n");
			}

			sr_configure(sr, target_opp_no);

			if (!sr_enable(sr, target_opp_no))
				sr_clk_disable(sr);
		}
	}
}

void disable_smartreflex(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;
	else
		return;

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 0) {
			if (sr_disable(sr))
				pr_warning("Problems in SR Disable!!!!\n");
			 /* Disable SR clk */
			sr_clk_disable(sr);
			/* Reset the volatage for current OPP */
			sr_reset_voltage(srid);
		}
	}
}

void omap3_voltagescale_vcbypass_setup(omap3_voltagescale_vcbypass_t fun)
{
	omap3_volscale_vcbypass_fun = fun;
}

/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u32 current_opp,
					u8 target_vsel, u8 current_vsel)
{
	u32 vdd, target_opp_no, current_opp_no;
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;
	u32 error_gain;

	if (omap3_volscale_vcbypass_fun)
		return omap3_volscale_vcbypass_fun(target_opp, current_opp,
						target_vsel, current_vsel);

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);
	current_opp_no = get_opp_no(current_opp);

	if (vdd == VDD1_OPP) {
		t2_smps_steps = abs(target_vsel - current_vsel);
		error_gain = ((target_opp_no < VDD1_OPP3)
				? PRM_VP1_CONFIG_ERRORGAIN_OPPLOW
				: PRM_VP1_CONFIG_ERRORGAIN_OPPHIGH);
		prm_rmw_mod_reg_bits(OMAP3430_ERRORGAIN_MASK, error_gain,
				OMAP3430_GR_MOD, OMAP3_PRM_VP1_CONFIG_OFFSET);
		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
				(target_vsel << OMAP3430_VC_CMD_ON_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_0_OFFSET);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == VDD2_OPP) {
		t2_smps_steps =  abs(target_vsel - current_vsel);
		error_gain = ((target_opp_no < VDD2_OPP3)
				? PRM_VP2_CONFIG_ERRORGAIN_OPPLOW
				: PRM_VP2_CONFIG_ERRORGAIN_OPPHIGH);
		prm_rmw_mod_reg_bits(OMAP3430_ERRORGAIN_MASK, error_gain,
				OMAP3430_GR_MOD, OMAP3_PRM_VP2_CONFIG_OFFSET);
		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
				(target_vsel << OMAP3430_VC_CMD_ON_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_1_OFFSET);
		reg_addr = R_VDD2_SR_CONTROL;
	}

	vc_bypass_value = (target_vsel << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_info("Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	/*
	 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
	 *  2us added as buffer.
	 */
	t2_smps_delay = ((t2_smps_steps * 125) / 40) + 2;
	udelay(t2_smps_delay);

	return 0;
}

/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t omap_sr_vdd1_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sr1.is_autocomp_active);
}

static ssize_t omap_sr_vdd1_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		pr_err("sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (value == 0) {
		sr_stop_vddautocomap(SR1);
	} else {
		u32 current_vdd1opp_no = get_vdd1_opp();
		if (!current_vdd1opp_no) {
			pr_err("sr_vdd1_autocomp: Current VDD1 opp unknown\n");
			return -EINVAL;
		}
		sr_start_vddautocomap(SR1, current_vdd1opp_no);
	}
	return n;
}

static struct kobj_attribute sr_vdd1_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd1_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd1_autocomp_show,
	.store = omap_sr_vdd1_autocomp_store,
};

/* Sysfs interface to select SR VDD2 auto compensation */
static ssize_t omap_sr_vdd2_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sr2.is_autocomp_active);
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		pr_err("sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (value == 0) {
		sr_stop_vddautocomap(SR2);
	} else {
		u32 current_vdd2opp_no = get_vdd2_opp();
		if (!current_vdd2opp_no) {
			pr_err("sr_vdd2_autocomp: Current VDD2 opp unknown\n");
			return -EINVAL;
		}
		sr_start_vddautocomap(SR2, current_vdd2opp_no);
	}
	return n;
}

static struct kobj_attribute sr_vdd2_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd2_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd2_autocomp_show,
	.store = omap_sr_vdd2_autocomp_store,
};

static irqreturn_t sr_omap_irq(int irq, void *dev_id)
{
	static u32 vp1_volt;
	static u32 vp2_volt;

	if (dev_id == &sr1)
		try_change_ret_volt(dev_id, &vp1_volt);

	if (dev_id == &sr2)
		try_change_ret_volt(dev_id, &vp2_volt);

	if (cpu_is_omap34xx()) {
		sr_modify_reg(dev_id, ERRCONFIG, ERRCONFIG_MCUBOUNDINTST,
				ERRCONFIG_MCUBOUNDINTST);

		/* Flush posted writes to avoid spurious IRQ */
		sr_read_reg(dev_id, ERRCONFIG);
	} else if (cpu_is_omap3630())
		/* Flush posted writes to avoid spurious IRQ */
		sr_read_reg(dev_id, ERRCONFIG_36XX);

	return IRQ_HANDLED;
}

static int __init omap3_sr_init(void)
{
	int ret = 0;
	u8 RdReg;
	u32 current_opp_no;

	/* Exit if OPP tables are not defined */
        if (!(mpu_opps && l3_opps)) {
                pr_err("SR: OPP rate tables not defined for platform, not enabling SmartReflex\n");
		return -ENODEV;
        }

	dpll1_ck = clk_get(NULL, "dpll1_ck");
	if (dpll1_ck == NULL || IS_ERR(dpll1_ck))
		return -ENODEV;

	l3_ick = clk_get(NULL, "l3_ick");
	if (l3_ick == NULL || IS_ERR(l3_ick))
		return -ENODEV;

	ret = get_ret_volt(&sr1);
	if (ret == -EINVAL)
		return -ENODEV;
	sr1.starting_ret_volt = ret;

	ret = get_ret_volt(&sr2);
	if (ret == -EINVAL)
		return -ENODEV;
	sr2.starting_ret_volt = ret;


#ifdef CONFIG_TWL4030_CORE
	/* Enable SR on T2 */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &RdReg,
					R_DCDC_GLOBAL_CFG);

	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, RdReg,
					R_DCDC_GLOBAL_CFG);
#endif

	if (cpu_is_omap34xx()) {
		sr1.clk = clk_get(NULL, "sr1_fck");
		sr2.clk = clk_get(NULL, "sr2_fck");
	}
	sr_set_clk_length(&sr1);
	sr_set_clk_length(&sr2);

	/* Call the VPConfig, VCConfig, set N Values. */
	sr_set_nvalues(&sr1);
	sr_configure_vp(SR1);

	sr_set_nvalues(&sr2);
	sr_configure_vp(SR2);

	ret = request_irq(SR1_IRQ, sr_omap_irq, IRQF_DISABLED, "sr1", &sr1);
	if (ret)
		goto out1;

	ret = request_irq(SR2_IRQ, sr_omap_irq, IRQF_DISABLED, "sr2", &sr2);
	if (ret)
		goto out2;

	pr_info("SmartReflex driver initialized\n");

	ret = sysfs_create_file(power_kobj, &sr_vdd1_autocomp.attr);
	if (ret)
		pr_err("sysfs_create_file failed: %d\n", ret);

	ret = sysfs_create_file(power_kobj, &sr_vdd2_autocomp.attr);
	if (ret)
		pr_err("sysfs_create_file failed: %d\n", ret);

	/* Enable SR only for 3430 for now */
	if (!cpu_is_omap3630()) {
		if (sr1.opp3_nvalue) {
			current_opp_no = get_vdd1_opp();
			if (!current_opp_no) {
				pr_err("omap3_sr_init: Current VDD1 opp unknown\n");
				return -EINVAL;
			}
			sr_start_vddautocomap(SR1, current_opp_no);
		}
		if (sr2.opp3_nvalue) {
			current_opp_no = get_vdd2_opp();
			if (!current_opp_no) {
				pr_err("omap3_sr_init: Current VDD2 opp unknown\n");
				return -EINVAL;
			}
			sr_start_vddautocomap(SR2, current_opp_no);
			pr_info("SmartReflex: enabling autocompensation\n");
		}
	}

	return 0;

	/*
	 * TODO: Proper cleanup routines necessary like disabling SR on T2,
	 * etc.
	 */
out1:

out2:
	free_irq(SR1_IRQ, &sr1);

	return ret;
}

late_initcall(omap3_sr_init);
