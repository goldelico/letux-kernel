/*
 * linux/drivers/video/omap2/dss/dpi.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#define DSS_SUBSYS_NAME "DPI"

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c/twl.h>

#include <linux/io.h>
#include <plat/board.h>
#include <plat/display.h>
#include <plat/cpu.h>

#include "dss.h"

/* for enabling VPLL2*/
#define ENABLE_VPLL2_DEDICATED		0x05
#define ENABLE_VPLL2_DEV_GRP		0xE0
#define TWL4030_VPLL2_DEV_GRP		0x33
#define TWL4030_VPLL2_DEDICATED		0x36

#define DPI2_BASE		0x58005000
void __iomem  *dpi2_base;


static struct {
	int update_enabled;
} dpi;

static void dpi_set_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings);

/*TODO: OMAP4: check the clock divisor mechanism? */
static int enable_vpll2_power(int enable)
{
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			 (enable) ? ENABLE_VPLL2_DEDICATED : 0,
			 TWL4030_VPLL2_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			 (enable) ? ENABLE_VPLL2_DEV_GRP : 0,
			 TWL4030_VPLL2_DEV_GRP);
	return 0;
}

static int dpi_set_dsi_clk(int lcd_channel_ix, bool is_tft,
		unsigned long pck_req,
		unsigned long *fck, int *lck_div, int *pck_div)
{
	struct dsi_clock_info dsi_cinfo;
	struct dispc_clock_info dispc_cinfo;
	int r;
	printk(KERN_INFO "DPI set dsi clk");
	if (!cpu_is_omap44xx()) {
		r = dsi_pll_calc_clock_div_pck(lcd_channel_ix, is_tft,
			pck_req, &dsi_cinfo, &dispc_cinfo);
		if (r)
			return r;
	} else {
		dispc_cinfo.lck_div = 1;
		dispc_cinfo.pck_div = 4;
		dsi_cinfo.regn = 19;
		dsi_cinfo.regm = 150;
		dsi_cinfo.regm3 = 4;
		dsi_cinfo.regm4 = 4;
		dsi_cinfo.use_dss2_fck = true;
		dsi_cinfo.highfreq = 0;
		dsi_calc_clock_rates(&dsi_cinfo);
	}
	r = dsi_pll_set_clock_div(lcd_channel_ix, &dsi_cinfo);
	if (r)
		return r;

	if (cpu_is_omap44xx())
		dss_select_clk_source_dsi(lcd_channel_ix, 1, 1);
	else
		dss_select_clk_source(0, 1);

	r = dispc_set_clock_div(lcd_channel_ix, &dispc_cinfo);
	if (r)
		return r;

	*fck = dsi_cinfo.dsi1_pll_fclk;
	*lck_div = dispc_cinfo.lck_div;
	*pck_div = dispc_cinfo.pck_div;

	return 0;
}

static int dpi_set_dispc_clk(int lcd_channel_ix,
	bool is_tft, unsigned long pck_req,
	unsigned long *fck, int *lck_div, int *pck_div)
{
	struct dss_clock_info dss_cinfo;
	struct dispc_clock_info dispc_cinfo;
	int r;

	if (cpu_is_omap44xx()) {
		printk(KERN_INFO "dpi set dispc clk");
	/*OMAP4: check this later?*/
		return 0;
	}

	r = dss_calc_clock_div(is_tft, pck_req, &dss_cinfo, &dispc_cinfo);
	if (r)
		return r;

	r = dss_set_clock_div(&dss_cinfo);
	if (r)
		return r;

	r = dispc_set_clock_div(lcd_channel_ix, &dispc_cinfo);
	if (r)
		return r;

	*fck = dss_cinfo.fck;
	*lck_div = dispc_cinfo.lck_div;
	*pck_div = dispc_cinfo.pck_div;

	return 0;
}

static int dpi_set_mode(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *t = &dssdev->panel.timings;
	int lck_div = 0, pck_div = 0;
	unsigned long fck = 0;
	unsigned long pck = 0;
	bool is_tft;
	int r = 0, lcd_channel_ix = 0;
	int use_dsi_for_hdmi = 0;

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		lcd_channel_ix = 1;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_set_pol_freq(OMAP_DSS_CHANNEL_LCD2, dssdev->panel.config,
				dssdev->panel.acbi, dssdev->panel.acb);
	else
		dispc_set_pol_freq(OMAP_DSS_CHANNEL_LCD, dssdev->panel.config,
				dssdev->panel.acbi, dssdev->panel.acb);

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

	if (use_dsi_for_hdmi)
		r = dpi_set_dsi_clk(lcd_channel_ix, is_tft,
				    t->pixel_clock * 1000,
				    &fck, &lck_div, &pck_div);
	else
		r = dpi_set_dispc_clk(lcd_channel_ix, is_tft,
				      t->pixel_clock * 1000,
				      &fck, &lck_div, &pck_div);
	if (r)
		goto err0;

	if (!cpu_is_omap44xx())
		pck = fck / lck_div / pck_div / 1000;
	else
		pck = 0;

	if (pck != t->pixel_clock) {
		DSSWARN("Could not find exact pixel clock. "
				"Requested %d kHz, got %lu kHz\n",
				t->pixel_clock, pck);

		t->pixel_clock = pck;
	}

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_set_lcd_timings(OMAP_DSS_CHANNEL_LCD2, t);
	else
		dispc_set_lcd_timings(OMAP_DSS_CHANNEL_LCD, t);


err0:
	if (cpu_is_omap44xx() && use_dsi_for_hdmi) {
		dss_select_clk_source_dsi(lcd_channel_ix, false, false);
		dsi_pll_uninit(lcd_channel_ix);
	}

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	return r;
}

static int dpi_basic_init(struct omap_dss_device *dssdev)
{
	bool is_tft;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

	dispc_set_parallel_interface_mode(dssdev->channel,
		OMAP_DSS_PARALLELMODE_BYPASS);
	dispc_set_lcd_display_type(dssdev->channel,
		is_tft ? OMAP_DSS_LCD_DISPLAY_TFT : OMAP_DSS_LCD_DISPLAY_STN);
	return dispc_set_tft_data_lines(dssdev->channel,
		dssdev->phy.dpi.data_lines);
}

/*This one needs to be to set the ovl info to dirty*/
static void dpi_start_auto_update(struct omap_dss_device *dssdev)

{
	int i;
	DSSDBG("starting auto update\n");
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		struct omap_overlay *ovl;
		ovl = omap_dss_get_overlay(i);
		if (!ovl){
			WARN_ON(1);
			continue;
		}
		if (ovl->manager == dssdev->manager)
			ovl->info_dirty = true;
			printk(KERN_ERR "ovl[%d]->manager = %s", i,
				ovl->manager->name);
	}
	if (dssdev->manager)
		dssdev->manager->apply(dssdev->manager);
}

static int dpi_display_enable(struct omap_dss_device *dssdev)
{
	int r;
	int lcd_channel_ix = 1;
	int use_dsi_for_hdmi = 0;

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) {
		DSSINFO("Lcd channel index 1");
		dpi2_base = ioremap(DPI2_BASE, 2000);
		lcd_channel_ix = 1;
	} else
		lcd_channel_ix = 0;

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		DSSERR("display already enabled\n");
		r = -EINVAL;
		goto err1;
	}

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	r = dpi_basic_init(dssdev);
	if (r)
		goto err2;
	if (use_dsi_for_hdmi) {
		dss_clk_enable(DSS_CLK_FCK2);
		enable_vpll2_power(1);

		if (cpu_is_omap3630())
			r = dsi_pll_init(lcd_channel_ix, dssdev, 1, 1);
		else
			/*check param 2*/
			r = dsi_pll_init(lcd_channel_ix, dssdev, 0, 1);
		if (r)
			goto err3;
	}
	r = dpi_set_mode(dssdev);
	if (r)
		goto err4;

	mdelay(2);

	if (cpu_is_omap44xx())
		dpi_start_auto_update(dssdev);

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 1);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 1);


	r = dssdev->driver->enable(dssdev);
	if (r)
		goto err5;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	/* This is done specifically for HDMI panel
	 * Default HDMI panel timings may not work for all monitors
	 * Reset HDMI panel timings after enabling HDMI.
	 */
	if (use_dsi_for_hdmi)
		dpi_set_timings(dssdev, &dssdev->panel.timings);

	return 0;

err5:
	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 0);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 0);
err4:
	if (use_dsi_for_hdmi) {
		dsi_pll_uninit(lcd_channel_ix);
		enable_vpll2_power(0);
err3:
		dss_clk_disable(DSS_CLK_FCK2);
	}
err2:
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
err1:
	omap_dss_stop_device(dssdev);
err0:
	return r;
}

static int dpi_display_resume(struct omap_dss_device *dssdev);

static void dpi_display_disable(struct omap_dss_device *dssdev)
{
	int lcd_channel_ix = 0;
	int use_dsi_for_hdmi = 0;

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		lcd_channel_ix = 1;

	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
		return;

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		dpi_display_resume(dssdev);

	dssdev->driver->disable(dssdev);

	if (use_dsi_for_hdmi) {
		dss_select_clk_source(0, 0);

		dispc_go(OMAP_DSS_CHANNEL_LCD);
		while (dispc_go_busy(OMAP_DSS_CHANNEL_LCD))
			;
		dsi_pll_uninit(lcd_channel_ix);
		enable_vpll2_power(0);
		dss_clk_disable(DSS_CLK_FCK2);
	}

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 0);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 0);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	omap_dss_stop_device(dssdev);
}

static int dpi_display_suspend(struct omap_dss_device *dssdev)
{
	int use_dsi_for_hdmi = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	DSSDBG("dpi_display_suspend\n");

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->driver->suspend)
		dssdev->driver->suspend(dssdev);

	if (use_dsi_for_hdmi) {
		dss_select_clk_source(0, 0);

		dispc_go(OMAP_DSS_CHANNEL_LCD);
		while (dispc_go_busy(OMAP_DSS_CHANNEL_LCD))
			;

		dsi_pll_uninit(dsi1);
		enable_vpll2_power(0);
		dss_clk_disable(DSS_CLK_FCK2);
	}

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 0);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 0);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int dpi_display_resume(struct omap_dss_device *dssdev)
{
	int r = 0;
	int lcd_channel_ix = 1;
	int use_dsi_for_hdmi = 0;

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) {
		DSSINFO("Lcd channel index 1");
		dpi2_base = ioremap(DPI2_BASE, 2000);
		lcd_channel_ix = 1;
	} else
		lcd_channel_ix = 0;

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	DSSDBG("dpi_display_resume\n");

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	if (use_dsi_for_hdmi) {
		dss_clk_enable(DSS_CLK_FCK2);
		enable_vpll2_power(1);

		if (cpu_is_omap3630())
			r = dsi_pll_init(lcd_channel_ix, dssdev, 1, 1);
		else
			r = dsi_pll_init(lcd_channel_ix, dssdev, 0, 1);
		if (r)
			goto err0;

		r = dpi_set_mode(dssdev);
		if (r)
			goto err1;
	}

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 1);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 1);

	if (dssdev->driver->resume) {
		r = dssdev->driver->resume(dssdev);
		if (r)
			goto err2;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	/* This is done specifically for HDMI panel
	 * Default HDMI panel timings may not work for all monitors
	 * Reset HDMI panel timings after enabling HDMI.
	 */
	if (use_dsi_for_hdmi)
		dpi_set_timings(dssdev, &dssdev->panel.timings);

	return 0;
err2:
	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 0);
	else
		dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 0);

	if (use_dsi_for_hdmi) {
err1:
		DSSERR("<%s!!> err0: failed to init DSI_PLL = %d\n",
		       __func__, r);
		dss_select_clk_source(0, 0);

		dispc_go(OMAP_DSS_CHANNEL_LCD);
		while (dispc_go_busy(OMAP_DSS_CHANNEL_LCD))
			;

		dsi_pll_uninit(dsi1);
		enable_vpll2_power(0);
err0:
		dss_clk_disable(DSS_CLK_FCK2);
	}

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	return r;
}

static void dpi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("dpi_set_timings\n");
	dssdev->panel.timings = *timings;
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dpi_set_mode(dssdev);
		if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
			dispc_go(OMAP_DSS_CHANNEL_LCD2);
		else {
			dispc_go(OMAP_DSS_CHANNEL_LCD);
			while (dispc_go_busy(OMAP_DSS_CHANNEL_LCD))
				;
		}

	}
}

static int dpi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	bool is_tft;
	int r = 0, lcd_channel_ix = 0;
	int lck_div = 0, pck_div = 0;
	unsigned long fck = 0;
	unsigned long pck = 0;
	int use_dsi_for_hdmi = 0;

	if (strncmp("hdmi", dssdev->name, 4) == 0)
		use_dsi_for_hdmi = 1;

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
		lcd_channel_ix = 1;

	if (!dispc_lcd_timings_ok(timings))
		return -EINVAL;

	if (timings->pixel_clock == 0)
		return -EINVAL;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

/*TODO: OMAP4: check the clock divisor mechanism? */
	if (use_dsi_for_hdmi) {
		struct dsi_clock_info dsi_cinfo;
		struct dispc_clock_info dispc_cinfo;
		r = dsi_pll_calc_clock_div_pck(lcd_channel_ix, is_tft,
				timings->pixel_clock * 1000,
				&dsi_cinfo, &dispc_cinfo);

		if (r)
			return r;

		fck = dsi_cinfo.dsi1_pll_fclk;
		lck_div = dispc_cinfo.lck_div;
		pck_div = dispc_cinfo.pck_div;
	} else {
		struct dss_clock_info dss_cinfo;
		struct dispc_clock_info dispc_cinfo;
		r = dss_calc_clock_div(is_tft, timings->pixel_clock * 1000,
				&dss_cinfo, &dispc_cinfo);

		if (r)
			return r;

		fck = dss_cinfo.fck;
		lck_div = dispc_cinfo.lck_div;
		pck_div = dispc_cinfo.pck_div;
	}

	pck = fck / lck_div / pck_div / 1000;

	timings->pixel_clock = pck;

	return 0;
}

static void dpi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int dpi_display_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	if (mode == OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	if (mode == OMAP_DSS_UPDATE_DISABLED) {
		if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
			dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 0);
		else
			dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 0);

		dpi.update_enabled = 0;
	} else {

		if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2)
			dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD2, 1);
		else
			dispc_enable_lcd_out(OMAP_DSS_CHANNEL_LCD, 1);

		dpi.update_enabled = 1;
	}

	return 0;
}

static enum omap_dss_update_mode dpi_display_get_update_mode(
		struct omap_dss_device *dssdev)
{
	return dpi.update_enabled ? OMAP_DSS_UPDATE_AUTO :
		OMAP_DSS_UPDATE_DISABLED;
}

static int dpi_set_data_lines(struct omap_dss_device *dssdev, int data_lines)
{
	int r = 0;

	if (dssdev->channel != OMAP_DSS_CHANNEL_LCD &&
		dssdev->channel != OMAP_DSS_CHANNEL_LCD2)
		return -EINVAL;

	dispc_enable_lcd_out(dssdev->channel, 0);

	r = dispc_set_tft_data_lines(dssdev->channel, data_lines);

	if (!r)
		dssdev->phy.dpi.data_lines = data_lines;

	dispc_enable_lcd_out(dssdev->channel, 1);

	return r;
}

static int dpi_get_data_lines(struct omap_dss_device *dssdev)
{
	return dssdev->phy.dpi.data_lines;
}

int dpi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	dssdev->enable = dpi_display_enable;
	dssdev->disable = dpi_display_disable;
	dssdev->suspend = dpi_display_suspend;
	dssdev->resume = dpi_display_resume;
	dssdev->set_timings = dpi_set_timings;
	dssdev->check_timings = dpi_check_timings;
	dssdev->get_timings = dpi_get_timings;
	dssdev->set_update_mode = dpi_display_set_update_mode;
	dssdev->get_update_mode = dpi_display_get_update_mode;
	dssdev->set_data_lines = dpi_set_data_lines;
	dssdev->get_data_lines = dpi_get_data_lines;

	return 0;
}

int dpi_init(void)
{
	return 0;
}

void dpi_exit(void)
{
}

