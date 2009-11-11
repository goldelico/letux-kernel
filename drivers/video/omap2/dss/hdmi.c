/*
 * linux/drivers/video/omap2/dss/hdmi.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * HDMI settings from TI's DSS driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <mach/display.h>
#include <mach/cpu.h>
#include <mach/hdmi_lib.h>
#include <mach/gpio.h>

#include "dss.h"

#define HDMI_PLLCTRL		0x58006200
#define HDMI_PHY		0x58006300

/* PLL */
#define PLLCTRL_PLL_CONTROL				0x0ul
#define PLLCTRL_PLL_STATUS				0x4ul
#define PLLCTRL_PLL_GO					0x8ul
#define PLLCTRL_CFG1					0xCul
#define PLLCTRL_CFG2					0x10ul
#define PLLCTRL_CFG3					0x14ul
#define PLLCTRL_CFG4					0x20ul

/* HDMI PHY */
#define HDMI_TXPHY_TX_CTRL				0x0ul
#define HDMI_TXPHY_DIGITAL_CTRL			0x4ul
#define HDMI_TXPHY_POWER_CTRL			0x8ul

/* CEA861D_CODE1 */
const struct omap_video_timings omap_dss_hdmi_timings3 = {
	.x_res = 640,
	.y_res = 480,
	.pixel_clock = 25200,
	.hsw = 96,
	.hfp = 16,
	.hbp = 48,
	.vsw = 2,
	.vfp = 10,
	.vbp = 33,
};
/* CEA861D_CODE4 */
const struct omap_video_timings omap_dss_hdmi_timings2 = {
	.x_res = 1280,
	.y_res = 720,
	.pixel_clock = 74250,
	.hsw = 40,
	.hfp = 110,
	.hbp = 220,
	.vsw = 5,
	.vfp = 5,
	.vbp = 20,
};
/* CEA861D_CODE16 */
const struct omap_video_timings omap_dss_hdmi_timings = {
	.x_res = 1920,
	.y_res = 1080,
	.pixel_clock = 148500,
	.hsw = 44,
	.hfp = 88,
	.hbp = 148,
	.vsw = 5,
	.vfp = 4,
	.vbp = 36,
};

static struct {
	void __iomem *base_phy;
	void __iomem *base_pll;
	struct mutex lock;
} hdmi;

static inline void hdmi_write_reg(u32 base, u16 idx, u32 val)
{
	void __iomem *b;

	switch (base) {
	case HDMI_PHY:
	  b = hdmi.base_phy;
	  break;
	case HDMI_PLLCTRL:
	  b = hdmi.base_pll;
	  break;
	default:
	  BUG();
	}
	__raw_writel(val, b + idx);
	/* DBG("write = 0x%x idx =0x%x\r\n", val, idx); */
}

static inline u32 hdmi_read_reg(u32 base, u16 idx)
{
	void __iomem *b;
	u32 l;

	switch (base) {
	case HDMI_PHY:
	 b = hdmi.base_phy;
	 break;
	case HDMI_PLLCTRL:
	 b = hdmi.base_pll;
	 break;
	default:
	 BUG();
	}
	l = __raw_readl(b + idx);

	/* DBG("addr = 0x%p rd = 0x%x idx = 0x%x\r\n", (b+idx), l, idx); */
	return l;
}

#define FLD_MASK(start, end)	(((1 << (start - end + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << end) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define REG_FLD_MOD(b, i, v, s, e) \
	hdmi_write_reg(b, i, FLD_MOD(hdmi_read_reg(b, i), v, s, e))

/*
 * refclk = (sys_clk/(highfreq+1))/(n+1)
 * so refclk = 38.4/2/(n+1) = 19.2/(n+1)
 * choose n = 15, makes refclk = 1.2
 *
 * m = tclk/cpf*refclk = tclk/2*1.2
 *
 *	for clkin = 38.2/2 = 192
 *	    phy = 2520
 *
 *	m = 2520*16/2* 192 = 105;
 *
 *	for clkin = 38.4
 *	    phy = 2520
 *
 */

#define CPF			2

typedef struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm4; /* M4_CLOCK_DIV */
} hdmi_pll_info;

/* HDMI TRM Page 53 */
static const hdmi_pll_info coef_hdmi[3] = {
	{15, 105, 0, 4},	/*  CEA861D_CODE1 */
	{15, 309, 98304, 7},	/*  CEA861D_CODE4 */
	{15, 618, 196608, 14},	/*  CEA861D_CODE16 */
};

static void compute_pll(int clkin, int phy,
	int n, int *m)
{
	int hf = 0;
	int refclk;

	if (clkin > 3200) /* 32 mHz */
		hf = 1;

	if (hf == 0)
		refclk = clkin / (n + 1);
	else
		refclk = clkin / (2 * (n + 1));

	/* HDMIPHY(MHz) = (CPF * regm / regn) * (clkin / (highfreq + 1)) */
	/* phy = CPF * regm * refclk */

	*m = phy / (CPF * refclk);

}

static int hdmi_pll_init(int refsel, int dcofreq, struct hdmi_pll_info *fmt)
{
	u32 r;
	unsigned t = 500000;
	u32 pll = HDMI_PLLCTRL;

	DSSDBG("0x%x %d %d %d %d\n", pll, refsel,
		fmt->regm, fmt->regn, dcofreq);

	/* PLL start always use manual mode */
	REG_FLD_MOD(pll, PLLCTRL_PLL_CONTROL, 0x0, 0, 0);

	r = hdmi_read_reg(pll, PLLCTRL_CFG1);
	r = FLD_MOD(r, fmt->regm, 20, 9); /* CFG1__PLL_REGM */
	r = FLD_MOD(r, fmt->regn, 8, 1);  /* CFG1__PLL_REGN */
	r = FLD_MOD(r, fmt->regm4, 25, 21); /* M4_CLOCK_DIV */

	hdmi_write_reg(pll, PLLCTRL_CFG1, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG2);

	/* SYS w/o divide by 2 [22:21] = donot care  [11:11] = 0x0 */
	/* SYS divide by 2     [22:21] = 0x3 [11:11] = 0x1 */
	/* PCLK, REF1 or REF2  [22:21] = 0x0, 0x 1 or 0x2 [11:11] = 0x1 */
	r = FLD_MOD(r, 0x0, 11, 11); /* PLL_CLKSEL 1: PLL 0: SYS*/
	r = FLD_MOD(r, 0x0, 12, 12); /* PLL_HIGHFREQ divide by 2 */
	r = FLD_MOD(r, 0x1, 13, 13); /* PLL_REFEN */
	r = FLD_MOD(r, 0x1, 14, 14); /* PHY_CLKINEN de-assert during locking */
	r = FLD_MOD(r, 0x0, 20, 20); /* HSDIVBYPASS assert during locking */
	r = FLD_MOD(r, refsel, 22, 21); /* REFSEL */
	/* DPLL3  used by DISPC or HDMI itself*/
	r = FLD_MOD(r, 0x0, 17, 17); /* M4_CLOCK_PWDN */
	r = FLD_MOD(r, 0x1, 16, 16); /* M4_CLOCK_EN */

	if (dcofreq)
		r = FLD_MOD(r, 0x4, 3, 1); /* 1000MHz and 2000MHz */
	else
		r = FLD_MOD(r, 0x2, 3, 1); /* 500MHz and 1000MHz */

	hdmi_write_reg(pll, PLLCTRL_CFG2, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG4);
	r = FLD_MOD(r, 0, 24, 18); /* todo: M2 */
	r = FLD_MOD(r, fmt->regmf, 17, 0);

	/* go now */
	REG_FLD_MOD(pll, PLLCTRL_PLL_GO, 0x1ul, 0, 0);

	/* wait for bit change */
	while (FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_GO), 0, 0))

	/* Wait till the lock bit is set */
	/* read PLL status */
	DSSDBG("status 0x%x\r\n", hdmi_read_reg(pll, PLLCTRL_PLL_STATUS));

	DSSDBG("CFG1 0x%x\r\n", hdmi_read_reg(pll, PLLCTRL_CFG1));
	DSSDBG("CFG2 0x%x\r\n", hdmi_read_reg(pll, PLLCTRL_CFG2));
	DSSDBG("CFG4 0x%x\r\n", hdmi_read_reg(pll, PLLCTRL_CFG4));

	while (0 == FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_STATUS), 1, 1)) {
		udelay(1);
		if (!--t) {
			ERR("cannot lock PLL\n");
			return -EIO;
		}
	}

	DSSDBG("PLL locked!\n");

	r = hdmi_read_reg(pll, PLLCTRL_CFG2);
	r = FLD_MOD(r, 0, 0, 0);	/* PLL_IDLE */
	r = FLD_MOD(r, 0, 5, 5);	/* PLL_PLLLPMODE */
	r = FLD_MOD(r, 0, 6, 6);	/* PLL_LOWCURRSTBY */
	r = FLD_MOD(r, 0, 8, 8);	/* PLL_DRIFTGUARDEN */
	r = FLD_MOD(r, 0, 10, 9);	/* PLL_LOCKSEL */
	r = FLD_MOD(r, 1, 13, 13);	/* PLL_REFEN */
	r = FLD_MOD(r, 1, 14, 14);	/* PHY_CLKINEN */
	r = FLD_MOD(r, 0, 15, 15);	/* BYPASSEN */
	r = FLD_MOD(r, 0, 20, 20);	/* HSDIVBYPASS */
	hdmi_write_reg(pll, PLLCTRL_CFG2, r);

	return 0;
}

static int hdmi_pll_reset(void)
{
	int t = 0;

	/* SYSREEST  controled by power FSM*/
	REG_FLD_MOD(HDMI_PLLCTRL, PLLCTRL_PLL_CONTROL, 0x0, 3, 3);

	/* READ 0x0 reset is in progress */
	while (!FLD_GET(hdmi_read_reg(HDMI_PLLCTRL,
			PLLCTRL_PLL_STATUS), 0, 0)) {
		udelay(1);
		if (t++ > 1000) {
			ERR("Failed to sysrest PLL\n");
			return -ENODEV;
		}
	}
	return 0;
}

int hdmi_pll_program(struct hdmi_pll_info *fmt)
{
	u32 r;
	int refsel, range;
	int pclk;

	HDMI_PllPwr_t PllPwrWaitParam;

	/* wait for wrapper rest */
	HDMI_W1_SetWaitSoftReset();

	/* power off PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_ALLOFF;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP,
				PllPwrWaitParam);
	if (r)
		return r;

	/* power on PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_BOTHON_ALLCLKS;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP,
				PllPwrWaitParam);
	if (r)
		return r;

	hdmi_pll_reset();

	pclk = (fmt->regm * 384)/((fmt->regn + 1) * 10);
	DSSDBG("pclk = %d\n", pclk);

	if (pclk > 1000)
		range = 1;
	else
		range = 0;

	refsel = 0x3; /* select SYSCLK reference */

	r = hdmi_pll_init(refsel, range, fmt);

	return r;

}

/* double check the order */
static int hdmi_phy_init(u32 w1,
		u32 phy)
{
	u32 count;
	int r;

	/* wait till PHY_PWR_STATUS=LDOON */
	/* HDMI_PHYPWRCMD_LDOON = 1 */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 1);
	if (r)
		return r;

	/* wait till PHY_PWR_STATUS=TXON */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 2);
	if (r)
		return r;

	/* read address 0 in order to get the SCPreset done completed */
	/* Dummy access performed to solve resetdone issue */
	hdmi_read_reg(phy, HDMI_TXPHY_TX_CTRL);

	/* write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(phy, HDMI_TXPHY_DIGITAL_CTRL,
				0xF0000000);

	/* setup max LDO voltage */
	REG_FLD_MOD(phy, HDMI_TXPHY_POWER_CTRL, 0xB, 3, 0);

	/* use HFBITCLK write HDMI_TXPHY_TX_CONTROL__FREQOUT field */
	REG_FLD_MOD(phy, HDMI_TXPHY_TX_CTRL, 0x1, 31, 30);

	count = 0;
	while (count++ < 1000)

	return 0;
}

static int hdmi_phy_off(u32 name)
{
	int r = 0;
	u32 count;

	/* wait till PHY_PWR_STATUS=OFF */
	/* HDMI_PHYPWRCMD_OFF = 0 */
	r = HDMI_W1_SetWaitPhyPwrState(name, 0);
	if (r)
		return r;

	count = 0;
	while (count++ < 200)

	return 0;
}

/* driver */
static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_panel_probe()\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = omap_dss_hdmi_timings;

	return 0;
}

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
}

static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	return 0;
}

static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
}

static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	return 0;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	return 0;
}

static void hdmi_enable_clocks(int enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
}

static struct omap_dss_driver hdmi_driver = {
	.probe		= hdmi_panel_probe,
	.remove		= hdmi_panel_remove,

	.enable		= hdmi_panel_enable,
	.disable	= hdmi_panel_disable,
	.suspend	= hdmi_panel_suspend,
	.resume		= hdmi_panel_resume,

	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};
/* driver end */

int hdmi_init(struct platform_device *pdev)
{
	hdmi_pll_info *pllptr;

	DSSDBG("Enter hdmi_init()\n");

	mutex_init(&hdmi.lock);

	hdmi.base_pll = ioremap(HDMI_PLLCTRL, 64);
	if (!hdmi.base_pll) {
		ERR("can't ioremap pll\n");
		return -ENOMEM;
	}
	hdmi.base_phy = ioremap(HDMI_PHY, 64);

	if (!hdmi.base_phy) {
		ERR("can't ioremap phy\n");
		return -ENOMEM;
	}

	hdmi_enable_clocks(1);

	hdmi_lib_init();

	hdmi_enable_clocks(0);
	return omap_dss_register_driver(&hdmi_driver);

}

void hdmi_exit(void)
{
	hdmi_lib_exit();

	iounmap(hdmi.base_pll);
	iounmap(hdmi.base_phy);
}


static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int format, pll_idx;
	int r;
	hdmi_pll_info *ptr;

	/* use EDID in the future */
	if (1920 == dssdev->panel.timings.x_res) {
		format = 16;
		pll_idx = 2;
	} else if (1280 == dssdev->panel.timings.x_res) {
		format = 4;
		pll_idx = 1;
	} else {
		format = 1;
		pll_idx = 0;
	}

	hdmi_enable_clocks(1);

	HDMI_W1_StopVideoFrame(HDMI_WP);

	dispc_enable_digit_out(0);

	/* while(dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT))
		msleep(40); */

	dispc_go(OMAP_DSS_CHANNEL_DIGIT);

	/* config the PLL and PHY first */
	ptr = &coef_hdmi[pll_idx];
	r = hdmi_pll_program(ptr);

	if (r)
		DSSERR("Failed to lock PLL\n");

	r = hdmi_phy_init(HDMI_WP, HDMI_PHY);
	if (r)
		DSSERR("Failed to start PHY\n");

	DSS_HDMI_CONFIG(HDMI_CORE_SYS, HDMI_CORE_AV,
		HDMI_WP, HDMI_PHY, HDMI_PLLCTRL, format);

	/* these settings are independent of overlays */
	dss_switch_tv_hdmi(1);

	/* bypass TV gamma table*/
	dispc_enable_gamma_table(0);

	/* do not fall into any sort of idle */
	dispc_set_idle_mode();

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	/* dss_dump_regs(NULL); */
	/* dispc_dump_regs(NULL); */

	HDMI_W1_StartVideoFrame(HDMI_WP);

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	dispc_enable_digit_out(1);

	/* while(dispc_go_busy(OMAP_DSS_CHANNEL_DIGIT)) */
	/*	msleep(40); */
	dispc_go(OMAP_DSS_CHANNEL_DIGIT);

	return 0;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	/* todo: find out what needs to be done for power off */
	dispc_enable_digit_out(0);

	hdmi_phy_off(HDMI_WP);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	hdmi_enable_clocks(0);
}

static int hdmi_enable_display(struct omap_dss_device *dssdev)
{
	int r = 0;
	DSSDBG("ENTER hdmi_enable_display()\n");

	mutex_lock(&hdmi.lock);

	/* the tv overlay manager is shared*/
	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err;
	}

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	hdmi_power_on(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&hdmi.lock);
	return r;

}

static void hdmi_disable_display(struct omap_dss_device *dssdev)
{
	DSSDBG("Enter hdmi_disable_display()\n");

	mutex_lock(&hdmi.lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
		goto end;

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		/* suspended is the same as disabled with venc */
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		goto end;
	}

	hdmi_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
end:
	omap_dss_stop_device(dssdev);
	mutex_unlock(&hdmi.lock);
}

static int hdmi_display_suspend(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSDBG("hdmi_display_suspend\n");
	return r;
}

static int hdmi_display_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSDBG("hdmi_display_resume\n");

	return r;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_set_timings\n");

	dssdev->panel.timings = *timings;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		/* turn the hdmi off and on to get new timings to use */
		hdmi_disable_display(dssdev);
		hdmi_enable_display(dssdev);
	}
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_check_timings\n");

	if (memcmp(&omap_dss_hdmi_timings, timings, sizeof(*timings)) == 0)
		return 0;

	return -EINVAL;
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	dssdev->enable = hdmi_enable_display;
	dssdev->disable = hdmi_disable_display;
	dssdev->suspend = hdmi_display_suspend;
	dssdev->resume = hdmi_display_resume;
	dssdev->get_timings = hdmi_get_timings;
	dssdev->set_timings = hdmi_set_timings;
	dssdev->check_timings = hdmi_check_timings;

	return 0;
}


#define HDMI_EDID_MAX_LENGTH 256

void hdmi_read_edid(void)
{
	u32 edid[HDMI_EDID_MAX_LENGTH];

	memset(edid, 0, HDMI_EDID_MAX_LENGTH);
	/* HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, edid); */
}
