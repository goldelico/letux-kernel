/*
 * hdmi.c
 *
 * HDMI interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Yong Zhi
 *	Mythri pk <mythripk@ti.com>
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
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/omapfb.h>
#include <video/omapdss.h>
#include <linux/gpio.h>

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/slab.h>
#include "ti_hdmi_4xxx_ip.h"
#endif

#include "ti_hdmi.h"
#include "dss.h"
#include "dss_features.h"

#define HDMI_WP			0x0
#define HDMI_PLLCTRL		0x200
#define HDMI_PHY		0x300

#define HDMI_CORE_AV		0x900

/* HDMI EDID Length move this */
#define HDMI_EDID_MAX_LENGTH			256
#define EDID_TIMING_DESCRIPTOR_SIZE		0x12
#define EDID_DESCRIPTOR_BLOCK0_ADDRESS		0x36
#define EDID_DESCRIPTOR_BLOCK1_ADDRESS		0x80
#define EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR	4
#define EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR	4

#define OMAP_HDMI_TIMINGS_NB			37

#define HDMI_DEFAULT_REGN 16
#define HDMI_DEFAULT_REGM2 1

static struct {
	struct mutex lock;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
	struct hdmi_ip_data ip_data;
	struct omap_dss_device *dssdev;
	int code;
	int mode;
	u8 edid[HDMI_EDID_MAX_LENGTH];
	u8 edid_set;
	bool custom_set;
	int hdmi_irq;
	bool hpd;
	bool can_do_hdmi;

	struct clk *sys_clk;
	struct clk *phy_clk;
	struct regulator *vdds_hdmi;
	int enabled;
	bool force_timings;
} hdmi;

static const u8 edid_header[8] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0};

static int hdmi_runtime_get(void)
{
	int r;

	DSSDBG("hdmi_runtime_get\n");

	r = pm_runtime_get_sync(&hdmi.pdev->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

static void hdmi_runtime_put(void)
{
	int r;

	DSSDBG("hdmi_runtime_put\n");

	r = pm_runtime_put_sync(&hdmi.pdev->dev);
	WARN_ON(r < 0);
}

#ifdef CONFIG_OMAP5_DSS_HDMI
static int omapdss_hdmi_io_configure(void)
{
	int r;

	printk(KERN_DEBUG "configure TPD\n");

	r = pio_a_read_byte(0xC);
	r &= 0xFC;
	pio_a_i2c_write(0xC, r);
	r = pio_a_read_byte(0x4);
	r |= 0x3;
	pio_a_i2c_write(0x4, r);

	return 0;
}
#endif

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSDBG("init_display\n");

	dss_init_hdmi_ip_ops(&hdmi.ip_data);
#ifdef CONFIG_OMAP5_DSS_HDMI
	r = pio_a_init();
	if (!r)
		omapdss_hdmi_io_configure();
#endif
	return r;
}

static int relaxed_fb_mode_is_equal(const struct fb_videomode *mode1,
				    const struct fb_videomode *mode2)
{
	u32 ratio1 = mode1->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);
	u32 ratio2 = mode2->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);
	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->pixclock     <= mode2->pixclock * 201 / 200 &&
		mode1->pixclock     >= mode2->pixclock * 200 / 201 &&
		mode1->hsync_len + mode1->left_margin + mode1->right_margin ==
		mode2->hsync_len + mode2->left_margin + mode2->right_margin &&
		mode1->vsync_len + mode1->upper_margin + mode1->lower_margin ==
		mode2->vsync_len + mode2->upper_margin + mode2->lower_margin &&
		(!ratio1 || !ratio2 || ratio1 == ratio2) &&
		(mode1->vmode & FB_VMODE_INTERLACED) ==
		(mode2->vmode & FB_VMODE_INTERLACED));
}

static int hdmi_set_timings(struct fb_videomode *vm, bool check_only)
{
	int i = 0;
	DSSDBG("hdmi_set_timings\n");

	if (!vm->xres || !vm->yres || !vm->pixclock)
		goto fail;

	for (i = 0; i < CEA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(cea_modes + i, vm)) {
			*vm = cea_modes[i];
			if (check_only)
				return 1;
			hdmi.ip_data.cfg.cm.code = i;
			hdmi.ip_data.cfg.cm.mode = HDMI_HDMI;
			hdmi.ip_data.cfg.timings =
					cea_modes[hdmi.ip_data.cfg.cm.code];
			goto done;
		}
	}

	for (i = 0; i < VESA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(vesa_modes + i, vm)) {
			*vm = vesa_modes[i];
			if (check_only)
				return 1;
			hdmi.ip_data.cfg.cm.code = i;
			hdmi.ip_data.cfg.cm.mode = HDMI_DVI;
			hdmi.ip_data.cfg.timings =
					vesa_modes[hdmi.ip_data.cfg.cm.code];
			goto done;
		}
	}
fail:
	if (check_only)
		return 0;
	hdmi.ip_data.cfg.cm.code = 1;
	hdmi.ip_data.cfg.cm.mode = HDMI_HDMI;
	hdmi.ip_data.cfg.timings = cea_modes[hdmi.ip_data.cfg.cm.code];
	i = -1;
done:
	DSSDBG("%s-%d\n", hdmi.ip_data.cfg.cm.mode ? "CEA" : "VESA",
						hdmi.ip_data.cfg.cm.code);
	return i >= 0;
}

void hdmi_get_monspecs(struct fb_monspecs *specs)
{
	int i, j;
	char *edid = (char *) hdmi.edid;
	u32 fclk = dispc_fclk_rate() / 1000;
	u32 max_pclk = hdmi.dssdev->clocks.hdmi.max_pixclk_khz;

	if (max_pclk && max_pclk < fclk)
		fclk = max_pclk;

	memset(specs, 0x0, sizeof(*specs));
	if (!hdmi.edid_set)
		return;
	fb_edid_to_monspecs(edid, specs);
	if (specs->modedb == NULL)
		return;

	for (i = 1; i <= edid[0x7e] && i * 128 < HDMI_EDID_MAX_LENGTH; i++) {
		if (edid[i * 128] == 0x2)
			fb_edid_add_monspecs(edid + i * 128, specs);
	}

	hdmi.can_do_hdmi = specs->misc & FB_MISC_HDMI;

	/* filter out resolutions we don't support */
	if (hdmi.force_timings) {
		for (i = 0; i < specs->modedb_len; i++) {
			specs->modedb[i++] = hdmi.ip_data.cfg.timings;
			break;
		}
		specs->modedb_len = i;
		hdmi.force_timings = false;
		return;
	}

	for (i = j = 0; i < specs->modedb_len; i++) {
		if (!hdmi_set_timings(&specs->modedb[i], true))
			continue;
		if (fclk < PICOS2KHZ(specs->modedb[i].pixclock))
			continue;
		if (specs->modedb[i].flag & FB_FLAG_PIXEL_REPEAT)
			continue;
		specs->modedb[j++] = specs->modedb[i];
	}
	specs->modedb_len = j;
}

u8 *hdmi_read_valid_edid(void)
{
	int ret, i;

	if (hdmi.edid_set)
		return hdmi.edid;

	memset(hdmi.edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = hdmi.ip_data.ops->read_edid(&hdmi.ip_data, hdmi.edid,
						  HDMI_EDID_MAX_LENGTH);

	if (cpu_is_omap54xx()) {
		unsigned char edid_980[] = {
			0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x44,
			0x89, 0xd4, 0x03, 0x15, 0xcd, 0x5b, 0x07, 0x1d, 0x14,
			0x01, 0x03, 0x80, 0x50, 0x2d, 0x78, 0x0a, 0x0d, 0xc9,
			0xa0, 0x57, 0x47, 0x98, 0x27, 0x12, 0x48, 0x4c, 0x20,
			0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58,
			0x2c, 0x45, 0x00, 0x20, 0xc2, 0x31, 0x00, 0x00, 0x1e,
			0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58,
			0x2c, 0x25, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x9e,
			0x00, 0x00, 0x00, 0xfc, 0x00, 0x48, 0x44, 0x4d, 0x49,
			0x20, 0x41, 0x6e, 0x61, 0x6c, 0x79, 0x7a, 0x65, 0x72,
			0x00, 0x00, 0x00, 0xfd, 0x00, 0x17, 0xf1, 0x08, 0x8c,
			0x17, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
			0x01, 0xab, 0x02, 0x03, 0x60, 0x71, 0x5f, 0x90, 0x1f,
			0x20, 0x05, 0x14, 0x04, 0x13, 0x03, 0x02, 0x12, 0x11,
			0x07, 0x06, 0x16, 0x15, 0x01, 0x0f, 0x0e, 0x1e, 0x1d,
			0x0d, 0x0c, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x0a,
			0x0b, 0x09, 0x5c, 0x08, 0x21, 0x22, 0x23, 0x24, 0x25,
			0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e,
			0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
			0x38, 0x39, 0x3a, 0x3b, 0x32, 0x0f, 0x7f, 0x07, 0x17,
			0x7f, 0x50, 0x3f, 0x7f, 0xc0, 0x57, 0x7f, 0x00, 0x5f,
			0x7f, 0x01, 0x67, 0x7f, 0x00, 0x83, 0x4f, 0x00, 0x00,
			0x67, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x38, 0x2d, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x82
		};
		DSSWARN("forcing EDID on OMAP5\n");
		memcpy(hdmi.edid, edid_980, sizeof(edid_980));
		ret = 0;
	}

	for (i = 0; i < HDMI_EDID_MAX_LENGTH; i += 16)
		pr_info("edid[%03x] = %02x %02x %02x %02x %02x %02x %02x %02x "
			"%02x %02x %02x %02x %02x %02x %02x %02x\n", i,
			hdmi.edid[i], hdmi.edid[i + 1], hdmi.edid[i + 2],
			hdmi.edid[i + 3], hdmi.edid[i + 4], hdmi.edid[i + 5],
			hdmi.edid[i + 6], hdmi.edid[i + 7], hdmi.edid[i + 8],
			hdmi.edid[i + 9], hdmi.edid[i + 10], hdmi.edid[i + 11],
			hdmi.edid[i + 12], hdmi.edid[i + 13], hdmi.edid[i + 14],
			hdmi.edid[i + 15]);

	if (ret) {
		DSSWARN("failed to read E-EDID\n");
		return NULL;
	}
	if (memcmp(hdmi.edid, edid_header, sizeof(edid_header))) {
		DSSWARN("failed to read E-EDID: wrong header\n");
		return NULL;
	}
	hdmi.edid_set = true;
	return hdmi.edid;
}

unsigned long hdmi_get_pixel_clock(void)
{
	/* HDMI Pixel Clock in Mhz */
	return PICOS2KHZ(hdmi.ip_data.cfg.timings.pixclock) * 1000;
}

static void hdmi_compute_pll(struct omap_dss_device *dssdev, int phy,
		struct hdmi_pll_info *pi)
{
	unsigned long clkin, refclk;
	u32 mf;

	clkin = clk_get_rate(hdmi.sys_clk) / 10000;
	/*
	 * Input clock is predivided by N + 1
	 * out put of which is reference clk
	 */
	if (dssdev->clocks.hdmi.regn == 0)
		pi->regn = HDMI_DEFAULT_REGN;
	else
		pi->regn = dssdev->clocks.hdmi.regn;

	refclk = clkin / pi->regn;

	if (dssdev->clocks.hdmi.regm2 == 0) {
		if (cpu_is_omap44xx()) {
			pi->regm2 = HDMI_DEFAULT_REGM2;
		} else if (cpu_is_omap54xx()) {
			if (phy <= 50000)
				pi->regm2 = 2;
			else
				pi->regm2 = 1;
		}
	} else {
		pi->regm2 = dssdev->clocks.hdmi.regm2;
	}

	/*
	 * multiplier is pixel_clk/ref_clk
	 * Multiplying by 100 to avoid fractional part removal
	 */

	pi->regm = (phy * 100 * pi->regm2 / refclk) / 100;
	/*
	 * fractional multiplier is remainder of the difference between
	 * multiplier and actual phy(required pixel clock thus should be
	 * multiplied by 2^18(262144) divided by the reference clock
	 */
	mf = (phy - pi->regm / pi->regm2 * refclk) * 262144;
	pi->regmf = pi->regm2 * mf / refclk;

	/*
	 * Dcofreq should be set to 1 if required pixel clock
	 * is greater than 1000MHz
	 */
	pi->dcofreq = phy > 1000 * 100;
	pi->regsd = ((pi->regm * clkin / 10) / (pi->regn * 250) + 5) / 10;

	/* Set the reference clock to sysclk reference */
	pi->refsel = HDMI_REFSEL_SYSCLK;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int r;
	struct omap_video_timings *p;
	unsigned long phy;

	r = hdmi_runtime_get();
	if (r)
		return r;

	r = regulator_enable(hdmi.vdds_hdmi);
	if (r)
		goto err;

	hdmi.ip_data.ops->video_enable(&hdmi.ip_data, 0);

	dispc_mgr_enable(OMAP_DSS_CHANNEL_DIGIT, 0);

	p = &dssdev->panel.timings;

	DSSDBG("hdmi_power_on x_res= %d y_res = %d\n",
		dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);

	if (!hdmi.custom_set) {
		struct fb_videomode vesa_vga = vesa_modes[4];
		hdmi_set_timings(&vesa_vga, false);
	}

	omapfb_fb2dss_timings(&hdmi.ip_data.cfg.timings,
						&dssdev->panel.timings);

	switch (hdmi.ip_data.cfg.deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		phy = (p->pixel_clock * 125) / 100 ;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		if (p->pixel_clock >= 148500) {
			DSSERR("36 bit deep color not supported for the"
				" pixel clock %d\n", p->pixel_clock);
			goto err;
		}

		phy = (p->pixel_clock * 150) / 100;
		break;
	case HDMI_DEEP_COLOR_24BIT:
	default:
		phy = p->pixel_clock;
		break;
	}

	hdmi_compute_pll(dssdev, phy, &hdmi.ip_data.pll_data);

	/* config the PLL and PHY hdmi_set_pll_pwrfirst */
	r = hdmi.ip_data.ops->pll_enable(&hdmi.ip_data);
	if (r) {
		DSSDBG("Failed to lock PLL\n");
		goto err;
	}

	if (gpio_get_value(hdmi.dssdev->hpd_gpio)) {
		/*
		 * If TPD is enabled before power on First interrupt is missed
		 * so check for current HPD state
		 */
		hdmi.hpd = 1;
		hdmi.ip_data.ops->notify_hpd(&hdmi.ip_data, hdmi.hpd);
	} else {
		r = hdmi.ip_data.ops->phy_enable(&hdmi.ip_data);
		if (r) {
			DSSDBG("Failed to start PHY\n");
#if 0
			/* Ignore Phy transition error for now*/
			goto err;
#endif
		}
	}

	hdmi.ip_data.cfg.cm.mode = hdmi.can_do_hdmi ? hdmi.mode : HDMI_DVI;
	hdmi.ip_data.cfg.cm.code = hdmi.code;

	if ((hdmi.mode)) {
		switch (hdmi.code) {
		case 20:
		case 5:
		case 6:
		case 21:
			hdmi.ip_data.cfg.interlace = 1;
			break;
		default:
			hdmi.ip_data.cfg.interlace = 0;
			break;
		}
	}
	hdmi.ip_data.ops->video_configure(&hdmi.ip_data);

	/* Make selection of HDMI in DSS */
	dss_select_hdmi_venc_clk_source(DSS_HDMI_M_PCLK);

	/* Select the dispc clock source as PRCM clock, to ensure that it is not
	 * DSI PLL source as the clock selected by DSI PLL might not be
	 * sufficient for the resolution selected / that can be changed
	 * dynamically by user. This can be moved to single location , say
	 * Boardfile.
	 */
	dss_select_dispc_clk_source(dssdev->clocks.dispc.dispc_fclk_src);

	/* bypass TV gamma table */
	dispc_enable_gamma_table(0);

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	hdmi.ip_data.ops->video_enable(&hdmi.ip_data, 1);

	dispc_mgr_enable(OMAP_DSS_CHANNEL_DIGIT, 1);

	return 0;
err:
	hdmi_runtime_put();
	return -EIO;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	hdmi.ip_data.ops->video_enable(&hdmi.ip_data, 0);

	dispc_mgr_enable(OMAP_DSS_CHANNEL_DIGIT, 0);

	hdmi.ip_data.ops->phy_disable(&hdmi.ip_data);
	hdmi.ip_data.ops->pll_disable(&hdmi.ip_data);

	regulator_disable(hdmi.vdds_hdmi);
	hdmi_runtime_put();

	hdmi.ip_data.cfg.deep_color = HDMI_DEEP_COLOR_24BIT;
	hdmi.hpd = 0;
}

int omapdss_hdmi_set_deepcolor(struct omap_dss_device *dssdev, int val,
		bool hdmi_restart)
{
	int r;

	if (!hdmi_restart) {
		hdmi.ip_data.cfg.deep_color = val;
		return 0;
	}

	dssdev->driver->disable(dssdev);
	hdmi.custom_set = true;
	hdmi.ip_data.cfg.deep_color = val;
	r = dssdev->driver->enable(dssdev);
	if (r)
		return r;

	return 0;
}

int omapdss_hdmi_get_deepcolor(void)
{
	return hdmi.ip_data.cfg.deep_color;
}

int omapdss_hdmi_set_range(int range)
{
	int r = 0;
	enum hdmi_range old_range;

	old_range = hdmi.ip_data.cfg.range;
	hdmi.ip_data.cfg.range = range;

	/* HDMI 1.3 section 6.6 VGA (640x480) format requires Full Range */
	if ((range == 0) &&
		((hdmi.ip_data.cfg.cm.code == 4 &&
		hdmi.ip_data.cfg.cm.mode == HDMI_DVI) ||
		(hdmi.ip_data.cfg.cm.code == 1 &&
		hdmi.ip_data.cfg.cm.mode == HDMI_HDMI)))
			return -EINVAL;

	r = hdmi.ip_data.ops->configure_range(&hdmi.ip_data);
	if (r)
		hdmi.ip_data.cfg.range = old_range;

	return r;
}

int omapdss_hdmi_get_range(void)
{
	return hdmi.ip_data.cfg.range;
}

int omapdss_hdmi_display_check_timing(struct omap_dss_device *dssdev,
					struct omap_video_timings *timings)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(timings, &t);

	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}
	if (!hdmi_set_timings(&t, true))
		return -EINVAL;

	return 0;
}

int omapdss_hdmi_display_set_mode2(struct omap_dss_device *dssdev,
				   struct fb_videomode *vm,
				   int code, int mode)
{
	/* turn the hdmi off and on to get new timings to use */
	dssdev->driver->disable(dssdev);
	hdmi.ip_data.cfg.timings = *vm;
	hdmi.custom_set = 1;
	hdmi.code = code;
	hdmi.mode = mode;
	return dssdev->driver->enable(dssdev);
}

int omapdss_hdmi_display_set_mode(struct omap_dss_device *dssdev,
				  struct fb_videomode *vm)
{
	int r1, r2;
	/* turn the hdmi off and on to get new timings to use */
	dssdev->driver->disable(dssdev);
	r1 = hdmi_set_timings(vm, false) ? 0 : -EINVAL;
	hdmi.custom_set = true;
	hdmi.code = hdmi.ip_data.cfg.cm.code;
	hdmi.mode = hdmi.ip_data.cfg.cm.mode;
	r2 = dssdev->driver->enable(dssdev);
	return r1 ? : r2;
}

int omapdss_hdmi_display_3d_enable(struct omap_dss_device *dssdev,
					struct s3d_disp_info *info, int code)
{
	int r = 0;

	DSSDBG("ENTER hdmi_display_3d_enable\n");

	mutex_lock(&hdmi.lock);

	if (dssdev->manager == NULL) {
		DSSERR("failed to enable display: no manager\n");
		r = -ENODEV;
		goto err0;
	}

	if (hdmi.enabled)
		goto err0;

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			DSSERR("failed to enable GPIO's\n");
			goto err1;
		}
	}

	/* hdmi.s3d_enabled will be updated when powering display up */
	/* if there's no S3D support it will be reset to false */
	switch (info->type) {
	case S3D_DISP_OVERUNDER:
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_NONE) {
			dssdev->panel.s3d_info = *info;
			hdmi.ip_data.cfg.s3d_info.frame_struct = HDMI_S3D_FRAME_PACKING;
			hdmi.ip_data.cfg.s3d_info.subsamp = false;
			hdmi.ip_data.cfg.s3d_info.subsamp_pos = 0;
			hdmi.ip_data.cfg.s3d_enabled = true;
			hdmi.ip_data.cfg.s3d_info.vsi_enabled = true;
		} else {
			goto err2;
		}
		break;
	case S3D_DISP_SIDEBYSIDE:
		dssdev->panel.s3d_info = *info;
		if (info->sub_samp == S3D_DISP_SUB_SAMPLE_NONE) {
			hdmi.ip_data.cfg.s3d_info.frame_struct = HDMI_S3D_SIDE_BY_SIDE_FULL;
			hdmi.ip_data.cfg.s3d_info.subsamp = true;
			hdmi.ip_data.cfg.s3d_info.subsamp_pos = HDMI_S3D_HOR_EL_ER;
			hdmi.ip_data.cfg.s3d_enabled = true;
			hdmi.ip_data.cfg.s3d_info.vsi_enabled = true;
		} else if (info->sub_samp == S3D_DISP_SUB_SAMPLE_H) {
			hdmi.ip_data.cfg.s3d_info.frame_struct = HDMI_S3D_SIDE_BY_SIDE_HALF;
			hdmi.ip_data.cfg.s3d_info.subsamp = true;
			hdmi.ip_data.cfg.s3d_info.subsamp_pos = HDMI_S3D_HOR_EL_ER;
			hdmi.ip_data.cfg.s3d_info.vsi_enabled = true;
		} else {
			goto err2;
		}
		break;
	default:
		goto err2;
	}
	if (hdmi.ip_data.cfg.s3d_enabled) {
		hdmi.custom_set = 1;
		hdmi.code = code;
		hdmi.mode = 1;
	}

	r = hdmi_power_on(dssdev);
	if (r) {
		DSSERR("failed to power on device\n");
		goto err2;
	}

	hdmi.custom_set = 0;

	mutex_unlock(&hdmi.lock);
	return 0;

err2:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
err1:
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&hdmi.lock);
	return r;
}

int omapdss_hdmi_display_set_display_interface(struct omap_dss_device *dssdev,
					union omap_display_interface_data data)
{
	int  r;
	DSSINFO("Enter omapdss_hdmi_display_set_display_interface\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		r = omapdss_hdmi_set_deepcolor(dssdev,
						data.hdmi.deep_color, false);
	else
		r = omapdss_hdmi_set_deepcolor(dssdev,
						data.hdmi.deep_color, true);

	return r;
}

void omapdss_hdmi_display_set_timing(struct omap_dss_device *dssdev)
{
	struct fb_videomode t;
	omapfb_dss2fb_timings(&dssdev->panel.timings, &t);
	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}
	omapdss_hdmi_display_set_mode(dssdev, &t);
}

void hdmi_dump_regs(struct seq_file *s)
{
	mutex_lock(&hdmi.lock);

	if (hdmi_runtime_get())
		return;

	hdmi.ip_data.ops->dump_wrapper(&hdmi.ip_data, s);
	hdmi.ip_data.ops->dump_pll(&hdmi.ip_data, s);
	hdmi.ip_data.ops->dump_phy(&hdmi.ip_data, s);
	hdmi.ip_data.ops->dump_core(&hdmi.ip_data, s);

	hdmi_runtime_put();
	mutex_unlock(&hdmi.lock);
}

int hdmi_get_current_hpd(void)
{
	return gpio_get_value(hdmi.dssdev->hpd_gpio);
}

static irqreturn_t hpd_enable_handler(int irq, void *ptr)
{
	static int hpd_prev;
	int hpd = hdmi_get_current_hpd();
	pr_info("hpd %d\n", hpd);

	if (hpd_prev != hpd) {
		hdmi_panel_hpd_handler(hpd);
		hpd_prev = hpd;
	}

	hdmi_runtime_get();
	hdmi.ip_data.ops->notify_hpd(&hdmi.ip_data, hdmi.hpd);
	hdmi_runtime_put();

	return IRQ_HANDLED;
}

static irqreturn_t hpd_irq_handler(int irq, void *ptr)
{
	hdmi.hpd = hdmi_get_current_hpd();
	return IRQ_WAKE_THREAD;
}

int omapdss_hdmi_read_edid(u8 *buf, int len)
{
	int r;

	mutex_lock(&hdmi.lock);

	r = hdmi_runtime_get();
	BUG_ON(r);

	r = hdmi.ip_data.ops->read_edid(&hdmi.ip_data, buf, len);

	hdmi_runtime_put();
	mutex_unlock(&hdmi.lock);

	return r;
}

bool omapdss_hdmi_detect(void)
{
	int r;

	mutex_lock(&hdmi.lock);

	r = hdmi_runtime_get();
	BUG_ON(r);

	r = hdmi.ip_data.ops->detect(&hdmi.ip_data);

	hdmi_runtime_put();
	mutex_unlock(&hdmi.lock);

	return r == 1;
}

static ssize_t hdmi_timings_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_videomode *t = &hdmi.ip_data.cfg.timings;
	return snprintf(buf, PAGE_SIZE,
			"%u,%u/%u/%u/%u,%u/%u/%u/%u,%c/%c,%s-%u\n",
			t->pixclock ? (u32)PICOS2KHZ(t->pixclock) : 0,
			t->xres, t->right_margin, t->left_margin, t->hsync_len,
			t->yres, t->lower_margin, t->upper_margin, t->vsync_len,
			(t->sync & FB_SYNC_HOR_HIGH_ACT) ? '+' : '-',
			(t->sync & FB_SYNC_VERT_HIGH_ACT) ? '+' : '-',
			hdmi.ip_data.cfg.cm.mode == HDMI_HDMI ? "CEA" : "VESA",
			hdmi.ip_data.cfg.cm.code);
}

static ssize_t hdmi_timings_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct fb_videomode t = { .pixclock = 0 }, c;
	u32 code, x, y, old_rate, new_rate = 0;
	int mode = -1, pos = 0, pos2 = 0;
	char hsync, vsync, ilace;
	int hpd;

	/* check for timings */
	if (sscanf(buf, "%u,%u/%u/%u/%u,%u/%u/%u/%u,%c/%c%n",
		   &t.pixclock,
		   &t.xres, &t.right_margin, &t.left_margin, &t.hsync_len,
		   &t.yres, &t.lower_margin, &t.upper_margin, &t.vsync_len,
		   &hsync, &vsync, &pos) >= 11 &&
	    (hsync == '+' || hsync == '-') &&
	    (vsync == '+' || vsync == '-') &&
	    t.pixclock) {
		t.sync = (hsync == '+' ? FB_SYNC_HOR_HIGH_ACT : 0) |
			(vsync == '+' ? FB_SYNC_VERT_HIGH_ACT : 0);
		t.pixclock = KHZ2PICOS(t.pixclock);
		buf += pos;
		if (*buf == ',')
			buf++;
	} else {
		t.pixclock = 0;
	}

	/* check for CEA/VESA code/mode */
	pos = 0;
	if (sscanf(buf, "CEA-%u%n", &code, &pos) >= 1 &&
	    code < CEA_MODEDB_SIZE) {
		mode = HDMI_HDMI;
		if (t.pixclock)
			t.flag = cea_modes[code].flag;
		else
			t = cea_modes[code];
	} else if (sscanf(buf, "VESA-%u%n", &code, &pos) >= 1 &&
		   code < VESA_MODEDB_SIZE) {
		mode = HDMI_DVI;
		if (!t.pixclock)
			t = vesa_modes[code];
	} else if (!t.pixclock &&
		   sscanf(buf, "%u*%u%c,%uHz%n",
			  &t.xres, &t.yres, &ilace, &t.refresh, &pos) >= 4 &&
		   (ilace == 'p' || ilace == 'i')) {

		/* optional aspect ratio (defaults to 16:9) for 720p */
		if (sscanf(buf + pos, ",%u:%u%n", &x, &y, &pos2) >= 2 &&
		      (x * 9 == y * 16 || x * 3 == y * 4) && x) {
			pos += pos2;
		} else {
			x = t.yres >= 720 ? 16 : 4;
			y = t.yres >= 720 ? 9 : 3;
		}

		pr_err("looking for %u*%u%c,%uHz,%u:%u\n",
		       t.xres, t.yres, ilace, t.refresh, x, y);
		/* CEA shorthand */
#define RATE(x) ((x) + ((x) % 6 == 5))
		t.flag = (x * 9 == y * 16) ? FB_FLAG_RATIO_16_9 :
							FB_FLAG_RATIO_4_3;
		t.vmode = (ilace == 'i') ? FB_VMODE_INTERLACED :
							FB_VMODE_NONINTERLACED;
		for (code = 0; code < CEA_MODEDB_SIZE; code++) {
			c = cea_modes[code];
			if (t.xres == c.xres &&
			    t.yres == c.yres &&
			    RATE(t.refresh) == RATE(c.refresh) &&
			    t.vmode == (c.vmode & FB_VMODE_MASK) &&
			    t.flag == (c.flag &
				(FB_FLAG_RATIO_16_9 | FB_FLAG_RATIO_4_3)))
				break;
		}
		if (code >= CEA_MODEDB_SIZE)
			return -EINVAL;
		mode = HDMI_HDMI;
		if (t.refresh != c.refresh)
			new_rate = t.refresh;
		t = c;
	} else {
		mode = HDMI_DVI;
		code = 0;
	}

	if (!t.pixclock)
		return -EINVAL;

	pos2 = 0;
	if (new_rate || sscanf(buf + pos, ",%uHz%n", &new_rate, &pos2) == 1) {
		u64 temp;
		pos += pos2;
		new_rate = RATE(new_rate) * 1000000 /
					(1000 + ((new_rate % 6) == 5));
		old_rate = RATE(t.refresh) * 1000000 /
					(1000 + ((t.refresh % 6) == 5));
		pr_err("%u mHz => %u mHz (%u", old_rate, new_rate, t.pixclock);
		temp = (u64) t.pixclock * old_rate;
		do_div(temp, new_rate);
		t.pixclock = temp;
		pr_err("=>%u)\n", t.pixclock);
	}

	pr_err("setting %u,%u/%u/%u/%u,%u/%u/%u/%u,%c/%c,%s-%u\n",
			t.pixclock ? (u32) PICOS2KHZ(t.pixclock) : 0,
			t.xres, t.right_margin, t.left_margin, t.hsync_len,
			t.yres, t.lower_margin, t.upper_margin, t.vsync_len,
			(t.sync & FB_SYNC_HOR_HIGH_ACT) ? '+' : '-',
			(t.sync & FB_SYNC_VERT_HIGH_ACT) ? '+' : '-',
			mode == HDMI_HDMI ? "CEA" : "VESA",
			code);

	hpd = !strncmp(buf + pos, "+hpd", 4);
	if (hpd) {
		hdmi.force_timings = true;
		hdmi_panel_hpd_handler(0);
		msleep(500);
		omapdss_hdmi_display_set_mode2(hdmi.dssdev, &t, code, mode);
		hdmi_panel_hpd_handler(1);
	} else {
		size = omapdss_hdmi_display_set_mode2(hdmi.dssdev, &t,
							code, mode) ? : size;
	}
	return size;
}

DEVICE_ATTR(hdmi_timings, S_IRUGO | S_IWUSR, hdmi_timings_show,
							hdmi_timings_store);

int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSDBG("ENTER hdmi_display_enable\n");

	mutex_lock(&hdmi.lock);

	if (dssdev->manager == NULL) {
		DSSERR("failed to enable display: no manager\n");
		r = -ENODEV;
		goto err0;
	}

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			DSSERR("failed to enable GPIO's\n");
			goto err1;
		}
	}

	r = hdmi_power_on(dssdev);
	if (r) {
		DSSERR("failed to power on device\n");
		goto err2;
	}

	hdmi.enabled = true;

	mutex_unlock(&hdmi.lock);
	return 0;

err2:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
err1:
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&hdmi.lock);
	return r;
}

void omapdss_hdmi_display_disable(struct omap_dss_device *dssdev)
{
	DSSDBG("Enter hdmi_display_disable\n");

	mutex_lock(&hdmi.lock);

	if (!hdmi.enabled)
		goto done;

	hdmi.enabled = false;

	if (!dssdev->sync_lost_error
		&& (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)) {
		/* clear EDID and mode on disable only */
		hdmi.edid_set = false;
		hdmi.custom_set = false;
		pr_info("hdmi: clearing EDID info\n");
	}

	hdmi_power_off(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omap_dss_stop_device(dssdev);
done:
	mutex_unlock(&hdmi.lock);
}

static irqreturn_t hdmi_irq_handler(int irq, void *arg)
{
	int r = 0;

	r = hdmi.ip_data.ops->irq_handler(&hdmi.ip_data);
	DSSDBG("Received HDMI IRQ = %08x\n", r);
	r = hdmi.ip_data.ops->irq_process(&hdmi.ip_data);
	return IRQ_HANDLED;
}

#if defined(CONFIG_SND_OMAP_SOC_OMAP5_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP5_HDMI_MODULE)
int omapdss_hdmi_get_hdmi_mode(void)
{
	return hdmi.mode;
}
#endif

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)

static int hdmi_audio_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct hdmi_ip_data *ip_data = snd_soc_codec_get_drvdata(codec);
	struct hdmi_audio_format audio_format;
	struct hdmi_audio_dma audio_dma;
	struct hdmi_core_audio_config core_cfg;
	struct hdmi_core_infoframe_audio aud_if_cfg;
	int err, n, cts;
	u32 pclk = hdmi.ip_data.cfg.timings.timings.pixel_clock;
	enum hdmi_core_audio_sample_freq sample_freq;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		core_cfg.i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_20BITS;
		core_cfg.i2s_cfg.word_length = HDMI_AUDIO_I2S_CHST_WORD_16_BITS;
		core_cfg.i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_16;
		core_cfg.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_dma.transfer_size = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		core_cfg.i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_24BITS;
		core_cfg.i2s_cfg.word_length = HDMI_AUDIO_I2S_CHST_WORD_24_BITS;
		core_cfg.i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_24;
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		core_cfg.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		audio_dma.transfer_size = 0x20;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 32000:
		sample_freq = HDMI_AUDIO_FS_32000;
		break;
	case 44100:
		sample_freq = HDMI_AUDIO_FS_44100;
		break;
	case 48000:
		sample_freq = HDMI_AUDIO_FS_48000;
		break;
	default:
		return -EINVAL;
	}

	err = hdmi_config_audio_acr(pclk, params_rate(params), &n, &cts);
	if (err < 0)
		return err;

	/* Audio wrapper config */
	audio_format.stereo_channels = HDMI_AUDIO_STEREO_ONECHANNEL;
	audio_format.active_chnnls_msk = 0x03;
	audio_format.type = HDMI_AUDIO_TYPE_LPCM;
	audio_format.sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;
	/* Disable start/stop signals of IEC 60958 blocks */
	audio_format.en_sig_blk_strt_end = HDMI_AUDIO_BLOCK_SIG_STARTEND_OFF;

	audio_dma.block_size = 0xC0;
	audio_dma.mode = HDMI_AUDIO_TRANSF_DMA;
	audio_dma.fifo_threshold = 0x20; /* in number of samples */

	hdmi_wp_audio_config_dma(ip_data, &audio_dma);
	hdmi_wp_audio_config_format(ip_data, &audio_format);

	/*
	 * I2S config
	 */
	core_cfg.i2s_cfg.en_high_bitrate_aud = false;
	/* Only used with high bitrate audio */
	core_cfg.i2s_cfg.cbit_order = false;
	/* Serial data and word select should change on sck rising edge */
	core_cfg.i2s_cfg.sck_edge_mode = HDMI_AUDIO_I2S_SCK_EDGE_RISING;
	core_cfg.i2s_cfg.vbit = HDMI_AUDIO_I2S_VBIT_FOR_PCM;
	/* Set I2S word select polarity */
	core_cfg.i2s_cfg.ws_polarity = HDMI_AUDIO_I2S_WS_POLARITY_LOW_IS_LEFT;
	core_cfg.i2s_cfg.direction = HDMI_AUDIO_I2S_MSB_SHIFTED_FIRST;
	/* Set serial data to word select shift. See Phillips spec. */
	core_cfg.i2s_cfg.shift = HDMI_AUDIO_I2S_FIRST_BIT_SHIFT;
	/* Enable one of the four available serial data channels */
	core_cfg.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN;

	/* Core audio config */
	core_cfg.freq_sample = sample_freq;
	core_cfg.n = n;
	core_cfg.cts = cts;
	if (dss_has_feature(FEAT_HDMI_CTS_SWMODE)) {
		core_cfg.aud_par_busclk = 0;
		core_cfg.cts_mode = HDMI_AUDIO_CTS_MODE_SW;
		core_cfg.use_mclk = false;
	} else {
		core_cfg.aud_par_busclk = (((128 * 31) - 1) << 8);
		core_cfg.cts_mode = HDMI_AUDIO_CTS_MODE_HW;
		core_cfg.use_mclk = true;
		core_cfg.mclk_mode = HDMI_AUDIO_MCLK_128FS;
	}
	core_cfg.layout = HDMI_AUDIO_LAYOUT_2CH;
	core_cfg.en_spdif = false;
	/* Use sample frequency from channel status word */
	core_cfg.fs_override = true;
	/* Enable ACR packets */
	core_cfg.en_acr_pkt = true;
	/* Disable direct streaming digital audio */
	core_cfg.en_dsd_audio = false;
	/* Use parallel audio interface */
	core_cfg.en_parallel_aud_input = true;

	hdmi_core_audio_config(ip_data, &core_cfg);

	/*
	 * Configure packet
	 * info frame audio see doc CEA861-D page 74
	 */
	aud_if_cfg.db1_coding_type = HDMI_INFOFRAME_AUDIO_DB1CT_FROM_STREAM;
	aud_if_cfg.db1_channel_count = 2;
	aud_if_cfg.db2_sample_freq = HDMI_INFOFRAME_AUDIO_DB2SF_FROM_STREAM;
	aud_if_cfg.db2_sample_size = HDMI_INFOFRAME_AUDIO_DB2SS_FROM_STREAM;
	aud_if_cfg.db4_channel_alloc = 0x00;
	aud_if_cfg.db5_downmix_inh = false;
	aud_if_cfg.db5_lsv = 0;

	hdmi_core_audio_infoframe_config(ip_data, &aud_if_cfg);
	return 0;
}

static int hdmi_audio_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	if (!hdmi.mode) {
		pr_err("Current video settings do not support audio.\n");
		return -EIO;
	}
	return 0;
}

static int hdmi_audio_drv_probe(struct snd_soc_codec *codec)
{
	struct hdmi_ip_data *priv;

	priv = kzalloc(sizeof(struct hdmi_ip_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->base_wp = hdmi.ip_data.base_wp;
	priv->core_av_offset = HDMI_CORE_AV;

	snd_soc_codec_set_drvdata(codec, priv);
	return 0;
}

static int hdmi_audio_drv_remove(struct snd_soc_codec *codec)
{
	struct hdmi_ip_data *priv = snd_soc_codec_get_drvdata(codec);
	kfree(priv);
	return 0;
}

static struct snd_soc_codec_driver hdmi_audio_codec_drv = {
	.probe = hdmi_audio_drv_probe,
	.remove = hdmi_audio_drv_remove,
};

static struct snd_soc_dai_ops hdmi_audio_codec_ops = {
	.hw_params = hdmi_audio_hw_params,
	.trigger = hdmi_audio_trigger,
	.startup = hdmi_audio_startup,
};

static struct snd_soc_dai_driver hdmi_codec_dai_drv = {
		.name = "hdmi-audio-codec",
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &hdmi_audio_codec_ops,
};
#endif

static int hdmi_get_clocks(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "sys_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.sys_clk = clk;

	clk = clk_get(&pdev->dev, "hdmi_phy_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.phy_clk = clk;

	return 0;
}

static void hdmi_put_clocks(void)
{
	if (hdmi.sys_clk)
		clk_put(hdmi.sys_clk);
	if (hdmi.phy_clk)
		clk_put(hdmi.phy_clk);
}

/* HDMI HW IP initialisation */
static int omapdss_hdmihw_probe(struct platform_device *pdev)
{
	struct resource *hdmi_mem;
	struct omap_dss_board_info *board_data;
	struct regulator *vdds_hdmi;
	int r;

	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;

	mutex_init(&hdmi.lock);

	/* save reference to HDMI device */
	board_data = hdmi.pdata->board_data;
	for (r = 0; r < board_data->num_devices; r++) {
		if (board_data->devices[r]->type == OMAP_DISPLAY_TYPE_HDMI)
			hdmi.dssdev = board_data->devices[r];
	}
	if (!hdmi.dssdev) {
		DSSERR("can't get HDMI device\n");
		return -EINVAL;
	}

	hdmi_mem = platform_get_resource(hdmi.pdev, IORESOURCE_MEM, 0);
	if (!hdmi_mem) {
		DSSERR("can't get IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	/* Base address taken from platform */
	hdmi.ip_data.base_wp = ioremap(hdmi_mem->start,
						resource_size(hdmi_mem));
	if (!hdmi.ip_data.base_wp) {
		DSSERR("can't ioremap WP\n");
		return -ENOMEM;
	}

	r = hdmi_get_clocks(pdev);
	if (r) {
		iounmap(hdmi.ip_data.base_wp);
		return r;
	}

	pm_runtime_enable(&pdev->dev);

	r = request_threaded_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio),
			hpd_irq_handler, hpd_enable_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING, "hpd", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %d failed\n",
			gpio_to_irq(hdmi.dssdev->hpd_gpio));
		return -EINVAL;
	}

	hdmi.hdmi_irq = platform_get_irq(pdev, 0);
	r = request_irq(hdmi.hdmi_irq, hdmi_irq_handler, 0, "OMAP HDMI", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %s failed\n", pdev->name);
		return -EINVAL;
	}

	/* Request for regulator supply required by HDMI PHY */
	vdds_hdmi = regulator_get(&pdev->dev, "vdds_hdmi");
	if (IS_ERR(vdds_hdmi)) {
		DSSERR("can't get VDDS_HDMI regulator\n");
		return PTR_ERR(vdds_hdmi);
	}

	hdmi.vdds_hdmi = vdds_hdmi;

	hdmi.ip_data.core_sys_offset = dss_feat_get_hdmi_core_sys_offset();
	hdmi.ip_data.core_av_offset = HDMI_CORE_AV;
	hdmi.ip_data.pll_offset = HDMI_PLLCTRL;
	hdmi.ip_data.phy_offset = HDMI_PHY;

	hdmi_panel_init();

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)

	/* Register ASoC codec DAI */
	r = snd_soc_register_codec(&pdev->dev, &hdmi_audio_codec_drv,
					&hdmi_codec_dai_drv, 1);
	if (r) {
		DSSERR("can't register ASoC HDMI audio codec\n");
		return r;
	}
#endif
	return 0;
}

static int omapdss_hdmihw_remove(struct platform_device *pdev)
{
	hdmi_panel_exit();

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)
	snd_soc_unregister_codec(&pdev->dev);
#endif

	free_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler);

	hdmi.dssdev = NULL;

	pm_runtime_disable(&pdev->dev);

	hdmi_put_clocks();

	regulator_put(hdmi.vdds_hdmi);
	hdmi.vdds_hdmi = NULL;

	iounmap(hdmi.ip_data.base_wp);

	return 0;
}

static int hdmi_runtime_suspend(struct device *dev)
{
	clk_disable(hdmi.phy_clk);
	clk_disable(hdmi.sys_clk);

	dispc_runtime_put();
	dss_runtime_put();

	return 0;
}

static int hdmi_runtime_resume(struct device *dev)
{
	int r;

	r = dss_runtime_get();
	if (r < 0)
		goto err_get_dss;

	r = dispc_runtime_get();
	if (r < 0)
		goto err_get_dispc;

	clk_enable(hdmi.sys_clk);
	clk_enable(hdmi.phy_clk);

	return 0;

err_get_dispc:
	dss_runtime_put();
err_get_dss:
	return r;
}

static const struct dev_pm_ops hdmi_pm_ops = {
	.runtime_suspend = hdmi_runtime_suspend,
	.runtime_resume = hdmi_runtime_resume,
};

static struct platform_driver omapdss_hdmihw_driver = {
	.probe          = omapdss_hdmihw_probe,
	.remove         = omapdss_hdmihw_remove,
	.driver         = {
		.name   = "omapdss_hdmi",
		.owner  = THIS_MODULE,
		.pm	= &hdmi_pm_ops,
	},
};

int hdmi_init_platform_driver(void)
{
	return platform_driver_register(&omapdss_hdmihw_driver);
}

void hdmi_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omapdss_hdmihw_driver);
}
