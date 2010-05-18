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
#include <plat/display.h>
#include <plat/cpu.h>
#include <plat/hdmi_lib.h>
#include <plat/gpio.h>

#include "dss.h"
#include "hdmi.h"

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
#define HDMI_TXPHY_PAD_CFG_CTRL			0xCul

static int hdmi_read_edid(struct omap_video_timings *);
static int get_edid_timing_data(u8 *edid, u16 *pixel_clk, u16 *horizontal_res,
			  u16 *vertical_res);
/* CEA-861-D Codes */
const struct omap_video_timings cea861d1 = \
	{640, 480, 25200, 96, 16, 48, 2, 10, 33};
const struct omap_video_timings cea861d4 = \
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20};
const struct omap_video_timings cea861d16 = \
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36};
const struct omap_video_timings cea861d17 = \
	{720, 576, 27000, 64, 12, 68, 5, 5, 39};
const struct omap_video_timings cea861d18 = \
	{720, 576, 27000, 64, 12, 68, 5, 5, 39};
const struct omap_video_timings cea861d29 = \
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39};
const struct omap_video_timings cea861d30 = \
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39};
const struct omap_video_timings cea861d31 = \
	{1920, 1080, 148500, 44, 528, 148, 5, 4, 36};
const struct omap_video_timings cea861d35 = \
	{2880, 480, 108000, 248, 64, 240, 6, 9, 30};
const struct omap_video_timings cea861d36 = \
	{2880, 480, 108000, 248, 64, 240, 6, 9, 30};
const struct omap_video_timings vesad10 = \
	{1024, 768, 65000, 136, 24, 160, 6, 3, 29};

static struct {
	void __iomem *base_phy;
	void __iomem *base_pll;
	struct mutex lock;
	int code;
	HDMI_Timing_t ti;
} hdmi;
struct omap_video_timings edid_timings;

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

struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm4; /* M4_CLOCK_DIV */
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};

static void compute_pll(int clkin, int phy,
	int n, struct hdmi_pll_info *pi)
{
	int refclk;
	u32 temp, mf;

	if (clkin > 3200) /* 32 mHz */
		refclk = clkin / (2 * (n + 1));
	else
		refclk = clkin / (n + 1);

	temp = phy * 100/(CPF * refclk);

	pi->regn = n;
	pi->regm = temp/100;
	pi->regm2 = 1;

	mf = (phy - pi->regm * CPF * refclk) * 262144;
	pi->regmf = mf/(CPF * refclk);

	if (phy > 1000 * 100) {
		pi->regm4 = phy / 10000;
		pi->dcofreq = 1;
		pi->regsd = ((pi->regm * 384)/((n + 1) * 250) + 5)/10;
	} else {
		pi->regm4 = 1;
		pi->dcofreq = 0;
		pi->regsd = 0;
	}

	DSSDBG("M = %d Mf = %d, m4= %d\n", pi->regm, pi->regmf, pi->regm4);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static int hdmi_pll_init(int refsel, int dcofreq, struct hdmi_pll_info *fmt, u16 sd)
{
	u32 r;
	unsigned t = 500000;
	u32 pll = HDMI_PLLCTRL;

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
	r = FLD_MOD(r, 0x0, 14, 14); /* PHY_CLKINEN de-assert during locking */
	r = FLD_MOD(r, 0x1, 20, 20); /* HSDIVBYPASS assert during locking */
	r = FLD_MOD(r, refsel, 22, 21); /* REFSEL */
	/* DPLL3  used by DISPC or HDMI itself*/
	r = FLD_MOD(r, 0x0, 17, 17); /* M4_CLOCK_PWDN */
	r = FLD_MOD(r, 0x1, 16, 16); /* M4_CLOCK_EN */

	if (dcofreq) {
		/* divider programming for 1080p */
		REG_FLD_MOD(pll, PLLCTRL_CFG3, sd, 17, 10);
		r = FLD_MOD(r, 0x4, 3, 1); /* 1000MHz and 2000MHz */
	} else
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
	while (0 == FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_STATUS), 1, 1)) {
		udelay(1);
		if (!--t) {
			printk(KERN_WARNING "HDMI: cannot lock PLL\n");
			DSSDBG("CFG1 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG1));
			DSSDBG("CFG2 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG2));
			DSSDBG("CFG4 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG4));
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
	int refsel;

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

	refsel = 0x3; /* select SYSCLK reference */

	r = hdmi_pll_init(refsel, fmt->dcofreq, fmt, fmt->regsd);

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

	/* write to phy address 0 to configure the clock */
	/* use HFBITCLK write HDMI_TXPHY_TX_CONTROL__FREQOUT field */
	REG_FLD_MOD(phy, HDMI_TXPHY_TX_CTRL, 0x1, 31, 30);

	/* write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(phy, HDMI_TXPHY_DIGITAL_CTRL,
				0xF0000000);

	/* setup max LDO voltage */
	REG_FLD_MOD(phy, HDMI_TXPHY_POWER_CTRL, 0xB, 3, 0);
	/*  write to phy address 3 to change the polarity control  */
	REG_FLD_MOD(phy, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 27, 27);

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
	switch (hdmi.code) {
	case 1:
	case 11:
		dssdev->panel.timings = cea861d1;
		break;
	case 4:
		dssdev->panel.timings = cea861d4;
		break;
	case 10:
		dssdev->panel.timings = vesad10;
		break;
	case 30:
		dssdev->panel.timings = cea861d30;
		break;
	case 16:
	default:
		dssdev->panel.timings = cea861d16;
	}

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

int hdmi_init(struct platform_device *pdev, int code)
{
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
	hdmi.code = code;
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
	int r = 0;
	int mode = 1;
	struct omap_video_timings *p;
	struct hdmi_pll_info pll_data;

	int clkin, n, phy;

	switch (hdmi.code) {
	case 1:
	case 11:
		dssdev->panel.timings = cea861d1;
		break;
	case 4:
		dssdev->panel.timings = cea861d4;
		break;
	case 10:
		dssdev->panel.timings = vesad10;
		break;
	case 30:
		dssdev->panel.timings = cea861d30;
		break;
	case 16:
	default:
		dssdev->panel.timings = cea861d16;
	}

	if (hdmi.code == 1 || hdmi.code == 10)
		mode = 0; /* DVI mode */

	hdmi_enable_clocks(1);

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	p = &dssdev->panel.timings;

	r = hdmi_read_edid(p);
	if (r) {
		r = -EIO;
		goto err;
	}

	clkin = 3840; /* 38.4 mHz */
	n = 15; /* this is a constant for our math */
	phy = p->pixel_clock;
	compute_pll(clkin, phy, n, &pll_data);

	HDMI_W1_StopVideoFrame(HDMI_WP);

	dispc_enable_digit_out(0);

	/* config the PLL and PHY first */
	r = hdmi_pll_program(&pll_data);
	if (r) {
		DSSERR("Failed to lock PLL\n");
		r = -EIO;
		goto err;
	}

	r = hdmi_phy_init(HDMI_WP, HDMI_PHY);
	if (r) {
		DSSERR("Failed to start PHY\n");
		r = -EIO;
		goto err;
	}

	DSS_HDMI_CONFIG(hdmi.ti, hdmi.code, mode);

	/* these settings are independent of overlays */
	dss_switch_tv_hdmi(1);

	/* bypass TV gamma table*/
	dispc_enable_gamma_table(0);

	/* do not fall into any sort of idle */
	dispc_set_idle_mode();

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	HDMI_W1_StartVideoFrame(HDMI_WP);

	dispc_enable_digit_out(1);

err:
	return r;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	HDMI_W1_StopVideoFrame(HDMI_WP);

	dispc_enable_digit_out(0);

	hdmi_phy_off(HDMI_WP);

	HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	hdmi_enable_clocks(0);

	/* reset to default */
	hdmi.code = 16;
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

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	r = hdmi_power_on(dssdev);
	if (r) {
		DSSERR("failed to power on device\n");
		goto err;
	}

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

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	omap_dss_stop_device(dssdev);

	hdmi_power_off(dssdev);
end:
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

	if (memcmp(&dssdev->panel.timings, timings, sizeof(*timings)) == 0)
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

static int hdmi_read_edid(struct omap_video_timings *dp)
{
	int r = 0;
	u8		edid[HDMI_EDID_MAX_LENGTH];
	u16		horizontal_res;
	u16		vertical_res;
	u16		pixel_clk;
	struct omap_video_timings *tp;

	memset(edid, 0, HDMI_EDID_MAX_LENGTH);
	tp = dp;

	if (HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, edid) != 0) {
		printk(KERN_WARNING "HDMI failed to read E-EDID\n");
		r = -EIO;
		goto err;
	} else {
		edid_timings.pixel_clock = dp->pixel_clock;
		edid_timings.x_res = dp->x_res;
		edid_timings.y_res = dp->y_res;
		/* search for timings of default resolution */
		if (get_edid_timing_data(edid, &pixel_clk,
				&horizontal_res, &vertical_res)) {
			dp->pixel_clock = pixel_clk * 10; /* be careful */
			tp = &edid_timings;
		} else {
				edid_timings.pixel_clock =
							cea861d4.pixel_clock;
				edid_timings.x_res = cea861d4.x_res;
				edid_timings.y_res = cea861d4.y_res;
				if (get_edid_timing_data(edid,
					&pixel_clk, &horizontal_res,
							&vertical_res)) {
					dp->pixel_clock = pixel_clk * 10;
					dp->x_res = horizontal_res;
					dp->y_res = vertical_res;
					tp = &edid_timings;
					if (tp->hfp == 440)
						hdmi.code = 19; /*720p 50hZ*/
					else /* 720p 60hZ */
						hdmi.code = 4;
		}
	}

	if (tp->x_res == cea861d16.x_res)
		switch (tp->hfp) {
		case 628: /* 1080p 24Hz */
			hdmi.code = 32;
			break;
		case 528: /* 1080p 50Hz */
			hdmi.code = 31;
			break;
		case 88: /* 1080p 60Hz */
		default:
			hdmi.code = 16;
		}
	}
	hdmi.ti.pixelPerLine = tp->x_res;
	hdmi.ti.linePerPanel = tp->y_res;
	hdmi.ti.horizontalBackPorch = tp->hbp;
	hdmi.ti.horizontalFrontPorch = tp->hfp;
	hdmi.ti.horizontalSyncPulse = tp->hsw;
	hdmi.ti.verticalBackPorch = tp->vbp;
	hdmi.ti.verticalFrontPorch = tp->vfp;
	hdmi.ti.verticalSyncPulse = tp->vsw;

err:
	return r;
}

u16 current_descriptor_addrs;

void get_horz_vert_timing_info(u8 *edid)
{
	/*HORIZONTAL FRONT PORCH */
	edid_timings.hfp = edid[current_descriptor_addrs + 8];
	/*HORIZONTAL SYNC WIDTH */
	edid_timings.hsw = edid[current_descriptor_addrs + 9];
	/*HORIZONTAL BACK PORCH */
	edid_timings.hbp = (((edid[current_descriptor_addrs + 4]
					  & 0x0F) << 8) |
					edid[current_descriptor_addrs + 3]) -
		(edid_timings.hfp + edid_timings.hsw);
	/*VERTICAL FRONT PORCH */
	edid_timings.vfp = ((edid[current_descriptor_addrs + 10] &
				       0xF0) >> 4);
	/*VERTICAL SYNC WIDTH */
	edid_timings.vsw = (edid[current_descriptor_addrs + 10] &
				      0x0F);
	/*VERTICAL BACK PORCH */
	edid_timings.vbp = (((edid[current_descriptor_addrs + 7] &
					0x0F) << 8) |
				      edid[current_descriptor_addrs + 6]) -
		(edid_timings.vfp + edid_timings.vsw);

	printk(KERN_INFO "hfp			= %d\n"
			"hsw			= %d\n"
			"hbp			= %d\n"
			"vfp			= %d\n"
			"vsw			= %d\n"
			"vbp			= %d\n",
		 edid_timings.hfp,
		 edid_timings.hsw,
		 edid_timings.hbp,
		 edid_timings.vfp,
		 edid_timings.vsw,
		 edid_timings.vbp);

}

/*------------------------------------------------------------------------------
 | Function    : get_edid_timing_data
 +------------------------------------------------------------------------------
 | Description : This function gets the resolution information from EDID
 |
 | Parameters  : void
 |
 | Returns     : void
 +----------------------------------------------------------------------------*/
static int get_edid_timing_data(u8 *edid, u16 *pixel_clk, u16 *horizontal_res,
			  u16 *vertical_res)
{
	u8 offset, effective_addrs;
	u8 count;
	u8 flag = false;
	/* Seach block 0, there are 4 DTDs arranged in priority order */
	for (count = 0; count < EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR; count++) {
		current_descriptor_addrs =
			EDID_DESCRIPTOR_BLOCK0_ADDRESS +
			count * EDID_TIMING_DESCRIPTOR_SIZE;
		*horizontal_res =
			(((edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 4 +
			   count * EDID_TIMING_DESCRIPTOR_SIZE] & 0xF0) << 4) |
			 edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 2 +
			 count * EDID_TIMING_DESCRIPTOR_SIZE]);
		*vertical_res =
			(((edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 7 +
			   count * EDID_TIMING_DESCRIPTOR_SIZE] & 0xF0) << 4) |
			 edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 5 +
			 count * EDID_TIMING_DESCRIPTOR_SIZE]);
		DSSDBG("***Block-0-Timing-descriptor[%d]***\n", count);
#ifdef EDID_DEBUG
		for (i = current_descriptor_addrs;
		      i <
		      (current_descriptor_addrs+EDID_TIMING_DESCRIPTOR_SIZE);
		      i++)
			DSSDBG("%d ==>		%x\n", i, edid[i]);

			DSSDBG("E-EDID Buffer Index	= 0x%x\n"
				 "horizontal_res       	= %d\n"
				 "vertical_res		= %d\n",
				 current_descriptor_addrs,
				 *horizontal_res,
				 *vertical_res
				 );
#endif
		if (*horizontal_res == edid_timings.x_res &&
		    *vertical_res == edid_timings.y_res) {
			DSSDBG("Found EDID Data for %d x %dp\n",
					*horizontal_res, *vertical_res);
			flag = true;
			break;
			}
	}

	/*check for the 1080p in extended block CEA DTDs*/
	if (flag != true) {
		offset = edid[EDID_DESCRIPTOR_BLOCK1_ADDRESS + 2];
		if (offset != 0) {
			effective_addrs = EDID_DESCRIPTOR_BLOCK1_ADDRESS
				+ offset;
			/*to determine the number of descriptor blocks */
			for (count = 0;
			      count < EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR;
			      count++) {
				current_descriptor_addrs = effective_addrs +
					count * EDID_TIMING_DESCRIPTOR_SIZE;
				*horizontal_res =
					(((edid[effective_addrs + 4 +
					   count*EDID_TIMING_DESCRIPTOR_SIZE] &
					   0xF0) << 4) |
					 edid[effective_addrs + 2 +
					 count * EDID_TIMING_DESCRIPTOR_SIZE]);
				*vertical_res =
					(((edid[effective_addrs + 7 +
					   count*EDID_TIMING_DESCRIPTOR_SIZE] &
					   0xF0) << 4) |
					 edid[effective_addrs + 5 +
					 count * EDID_TIMING_DESCRIPTOR_SIZE]);

				DSSDBG("Block1-Timing-descriptor[%d]\n", count);
#ifdef EDID_DEBUG
				for (i = current_descriptor_addrs;
				      i < (current_descriptor_addrs+
					   EDID_TIMING_DESCRIPTOR_SIZE); i++)
					DSSDBG("%x ==> 	%x\n",
						   i, edid[i]);

				DSSDBG("current_descriptor	= 0x%x\n"
						"horizontal_res		= %d\n"
						"vertical_res 		= %d\n",
					 current_descriptor_addrs,
					 *horizontal_res, *vertical_res);
#endif
				if (*horizontal_res == edid_timings.x_res &&
				    *vertical_res == edid_timings.y_res) {
					DSSDBG("Found EDID Data for "
						 "%d x %dp\n",
						 *horizontal_res,
						 *vertical_res
						 );
					flag = true;
					break;
					}
			}
		}
	}

	if (flag == true) {
		*pixel_clk = ((edid[current_descriptor_addrs + 1] << 8) |
					edid[current_descriptor_addrs]);

		edid_timings.x_res = *horizontal_res;
		edid_timings.y_res = *vertical_res;
		edid_timings.pixel_clock = *pixel_clk*10;
		printk(KERN_INFO "EDID TIMING DATA FOUND\n"
			 "EDID DTD block address	= 0x%x\n"
			 "pixel_clk 			= %d\n"
			 "horizontal res		= %d\n"
			 "vertical res			= %d\n",
			 current_descriptor_addrs,
			 edid_timings.pixel_clock,
			 edid_timings.x_res,
			 edid_timings.y_res
			 );

		get_horz_vert_timing_info(edid);
	} else {

		printk(
			 "EDID TIMING DATA supported NOT FOUND\n"
			 "setting default timing values\n"
			 "pixel_clk 		= %d\n"
			 "horizontal res	= %d\n"
			 "vertical res		= %d\n",
			 edid_timings.pixel_clock,
			 edid_timings.x_res,
			 edid_timings.y_res
			 );

		*pixel_clk = edid_timings.pixel_clock;
		*horizontal_res = edid_timings.x_res;
		*vertical_res = edid_timings.y_res;
	}

	return flag;
}

void hdmi_dump_regs(struct seq_file *s)
{
	DSSDBG("0x4a100060 x%x\n", omap_readl(0x4A100060));
	DSSDBG("0x4A100088 x%x\n", omap_readl(0x4A100088));
	DSSDBG("0x48055134 x%x\n", omap_readl(0x48055134));
	DSSDBG("0x48055194 x%x\n", omap_readl(0x48055194));
}
