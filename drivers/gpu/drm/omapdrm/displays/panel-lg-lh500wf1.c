/*
 * Driver for panels with Renesas R63311
 * http://www.rsp.renesas.com/en/news/2012/news20120418.htm
 *
 * and specifically the LH500WF1-SD01
 *
 * Similar LG panels are e.g. http://www.panelook.com/LH550WF1-SD01_LG%20Display_5.5_LCM_parameter_19410.html
 * This chip also said to be the one used in the Nexus 7 (2nd Gen)
 * and a Sharp LS050T3SX01 or LS059T1SX01 is said to use the same controller and be used in the Xperia Z
 * But beware: they may need a different set of initialization parameters!!!
 *
 * Copyright 2011 Texas Instruments, Inc.
 * Author: Archit Taneja <archit@ti.com>
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * based on lg4591 panel driver by H. Nikolaus Schaller <hns@goldelico.com>
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

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "../dss/omapdss.h"
#include <video/omap-panel-data.h>
#include <video/mipi_display.h>

#include <linux/err.h>
#include <linux/regulator/consumer.h>

/* extended DCS commands (not defined in mipi_display.h) */
#define DCS_READ_DDB_START		0x02
#define DCS_READ_NUM_ERRORS		0x05
#define DCS_BRIGHTNESS			0x51	// write brightness
#define DCS_READ_BRIGHTNESS		0x52	// read brightness
#define DCS_CTRL_DISPLAY		0x53	// enable backlight etc.
#define DCS_READ_CTRL_DISPLAY		0x53	// read control
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
#define MCS_READID1			0xda
#define MCS_READID2			0xdb
#define MCS_READID3			0xdc

/* manufacturer specific commands */
#define MCS_MANUFPROT	0xb0
#define MCS_SETDEEPSTBY	0xb1
#define MCS_IFACESET	0xb3
#define MCS_MIPISPEED	0xb6
#define MCS_DISPLSET1	0xc1
#define MCS_DISPLSET2	0xc2
#define MCS_VSYNCEN	0xc3
#define MCS_SRCTIMING	0xc4
#define MCS_LPTSTIMING	0xc6
#define MCS_GAMMA_A	0xc7
#define MCS_GAMMA_B	0xc8
#define MCS_GAMMA_C	0xc9
#define MCS_PANELIFACE	0xcc
#define MCS_CHARGEPUMP	0xd0
#define MCS_POWERSET	0xd3
#define MCS_VCOMSET	0xd5

#define IS_MCS(CMD) (((CMD) >= 0xb0 && (CMD) <= 0xff) && !((CMD) == 0xda || (CMD) == 0xdb || (CMD) == 0xdc))

/* horizontal * vertical * refresh */
#define R63311_W				(1080)
#define R63311_H				(1920)
#define R63311_WIDTH			(R63311_W+120)
#define R63311_HEIGHT			(R63311_H+80)
#define R63311_FPS				(60ll)
#define R63311_PIXELCLOCK		(R63311_WIDTH * R63311_HEIGHT * R63311_FPS)	// Full HD * 60 fps
/* panel has 16.7M colors = RGB888 = 3*8 bit per pixel */
#define R63311_PIXELFORMAT		OMAP_DSS_DSI_FMT_RGB888	// 16.7M color = RGB888
#define R63311_BIT_PER_PIXEL	(3*8)
/* the panel can handle 4 lanes */
#define R63311_LANES			4
/* high speed clock is running at double data rate, i.e. half speed
 * (take care of integer overflows!)
 * hsck =  bit/pixel * 110% * pixel clock / lanes / 2 clock edges
 * real clock rate may be rounded up or down depending on divisors
 */
#define R63311_HS_CLOCK			(R63311_BIT_PER_PIXEL * (R63311_PIXELCLOCK / (R63311_LANES * 2)))
/* low power clock is quite arbitrarily choosen to be roughly 10 MHz */
#define R63311_LP_CLOCK			10000000	// low power clock

static struct videomode r63311_timings = {
	.hactive		= R63311_W,
	.vactive		= R63311_H,
	.pixelclock	= R63311_PIXELCLOCK,
	// the following values are choosen arbitrarily since there is no spec in the data sheet
	// they are choosen to round up to 1200x2000 pixels giving a pixel clock of 144 MHz
	.hfront_porch		= 10,
	.hsync_len		= 10,
	.hback_porch		= 100,
	.vfront_porch		= 62,
	.vsync_len		= 10,
	.vback_porch		= 8,
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct videomode timings;

	struct platform_device *pdev;

	struct mutex lock;

	struct backlight_device *bldev;
	int bl;

	int	reset_gpio;
	int	regulator_gpio;

	struct omap_dsi_pin_config pin_config;

	bool enabled;

	int config_channel;
	int pixel_channel;

};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

struct r63311_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

static struct r63311_reg init_seq[] = {
	/* see Table 3.9.2 Display Initial Set */
	/* we should define some macros... */
#if 1
	{ {	MIPI_DCS_NOP }, 1 },
	{	{
			MCS_MANUFPROT,
			0x00	// remove access protect (except NVM)
		}, 2
	},
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{	{
			MCS_IFACESET,
			0x14, 0x00, 0x00, 0x00, 0x00,
			0x00
		}, 7
	},
	{	{
			MCS_MIPISPEED,
			0x3A, 0xD3
		}, 3
	},
	{	{
			MCS_DISPLSET1,
			0x84, 0x60, 0x40, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x0C,
			0x01, 0x58, 0x73, 0xAE, 0x31,
			0x20, 0x06, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x10, 0x10,
			0x10, 0x10, 0x00, 0x00, 0x00,
			0x22, 0x02, 0x02, 0x00
		}, 35
	},
	{	{
			MCS_DISPLSET2,
			0x31, 0xF7, 0x80, 0x0A, 0x08,
			0x00, 0x00
		}, 8
	},
	{	{
			MCS_VSYNCEN,
			0x01, 0x00, 0x00,
		}, 4
	},
	{	{
			MCS_SRCTIMING,
			0x70, 0x00, 0x00, 0x00, 0x07,
			0x05, 0x05, 0x09, 0x09, 0x0c,
			0x06, 0x00, 0x00, 0x00, 0x00,
			0x07, 0x05, 0x05, 0x09, 0x09,
			0x0c, 0x06
		}, 23
	},
	{	{
			MCS_LPTSTIMING,
			0x00, 0x69, 0x00, 0x69, 0x00,
			0x69, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x69, 0x00, 0x69, 0x00,
			0x69, 0x10, 0x19, 0x07, 0x00,
			0x01, 0x00, 0x69, 0x00, 0x69,
			0x00, 0x69, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x69, 0x00, 0x69,
			0x00, 0x69, 0x10, 0x19, 0x07
		}, 41
	},
	{	{
			MCS_GAMMA_A,
			0x00, 0x09, 0x14, 0x26, 0x31,
			0x48, 0x3B, 0x52, 0x5F, 0x67,
			0x6B, 0x70, 0x00, 0x09, 0x14,
			0x26, 0x31, 0x48, 0x3B, 0x52,
			0x5F, 0x67, 0x6B, 0x70
		}, 25
	},
	{	{
			MCS_GAMMA_B,
			0x00, 0x09, 0x14, 0x26, 0x31,
			0x48, 0x3B, 0x52, 0x5F, 0x67,
			0x6B, 0x70, 0x00, 0x09, 0x14,
			0x26, 0x31, 0x48, 0x3B, 0x52,
			0x5F, 0x67, 0x6B, 0x70
		}, 25
	},
	{	{
			MCS_GAMMA_C,
			0x00, 0x09, 0x14, 0x26, 0x31,
			0x48, 0x3B, 0x52, 0x5F, 0x67,
			0x6B, 0x70, 0x00, 0x09, 0x14,
			0x26, 0x31, 0x48, 0x3B, 0x52,
			0x5F, 0x67, 0x6B, 0x70
		}, 25
	},
	{	{
			MCS_PANELIFACE,
			0x09
		}, 2
	},
#endif
#if 1
	{	{
			MCS_CHARGEPUMP,
			0x00, 0x00, 0x19, 0x18, 0x99,
			0x99, 0x19, 0x01, 0x89, 0x00,
			0x55, 0x19, 0x99, 0x01,
		}, 15
	},
#endif
#if 1
	{	{
			MCS_POWERSET,
			0x1B, 0x33, 0xBB, 0xCC, 0xC4,
			0x33, 0x33, 0x33, 0x00, 0x01,
			0x00, 0xA0, 0xD8, 0xA0, 0x0D,
			0x37, 0x33, 0x44, 0x22, 0x70,
			0x02, 0x37, 0x03, 0x3D, 0xBF,
			0x00
		}, 27
	},
	{	{
			MCS_VCOMSET,
			0x06, 0x00, 0x00, 0x01, 0x39,
			0x01, 0x39
		}, 8
	},
	{	{
			MCS_VCOMSET,	/* second time */
			0x06, 0x00, 0x00, 0x01, 0x39,
			0x01, 0x39
		}, 8
	},
#endif
#if 1
	{ {	0xce,
		0x00, 0x01, 0x40, 0xc1, 0x00,
		0x00, 0x00
	}, 8 },
	{ { DCS_CTRL_DISPLAY, 0x24}, 2 },	// LEDPWM ON
	{ { DCS_WRITE_CABC, 0x00}, 2 },		// CABC off
#endif
#if 0	/* switch panel to test mode */
	{	{
			MCS_IFACESET,
			// test pattern needs internal oscillator
			0x04, 0x00, 0x00, 0x00, 0x00,
			0x00
		}, 7
	},
	{	{
			0xde,
			0x01, 0xff, 0x07, 0x10, 0x00,
			0x77
		}, 7
	},
	{ {	MIPI_DCS_NOP }, 1 },
	{ { MIPI_DCS_SET_DISPLAY_ON, }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ { MIPI_DCS_EXIT_SLEEP_MODE, }, 1 },
#endif
};

static struct r63311_reg test_image[] = {
	/* switch panel to test mode */
	{ { MIPI_DCS_ENTER_SLEEP_MODE, }, 1 },
	{ { MIPI_DCS_SET_DISPLAY_OFF, }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_IFACESET,
		// test pattern needs internal oscillator
		0x04, 0x00, 0x00, 0x00, 0x00,
		0x00
	}, 7 },
	{ {	0xde,
		0x01, 0xff, 0x07, 0x10, 0x00,
		0x77
	}, 7 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ { MIPI_DCS_SET_DISPLAY_ON, }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ { MIPI_DCS_EXIT_SLEEP_MODE, }, 1 },
};

static struct r63311_reg sleep_out[] = {
	{ { MIPI_DCS_SET_DISPLAY_ON, }, 1 },
	{ { MIPI_DCS_EXIT_SLEEP_MODE, }, 1 },
};

static struct r63311_reg display_on[] = {
	{	{
			MCS_VCOMSET,
			0x06, 0x00, 0x00, 0x01, 0x2c,
			0x01, 0x2c
		}, 8
	},
	{	{
			MCS_VCOMSET,	/* second time */
			0x06, 0x00, 0x00, 0x01, 0x2c,
			0x01, 0x2c
		}, 8
	},
};

/* not used - needs also MCS_VCOMSET and some delays */

static struct r63311_reg display_off[] = {
	{ { MIPI_DCS_SET_DISPLAY_OFF, }, 1},
};

static struct r63311_reg sleep_in[] = {
	{ { MIPI_DCS_ENTER_SLEEP_MODE, }, 1},
};

static int r63311_write(struct omap_dss_device *dssdev, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->dssdev.src;
	int r;
	int i;

	printk("dsi: r63311_write("); for (i = 0; i < len; i++) printk("%02x%s", buf[i], i + 1 == len ? ")\n" : " ");

	if (IS_MCS(buf[0])) {
		// this is a "manufacturer command" that must be sent as a "generic read command"
		r = in->ops->dsi.gen_write(in, ddata->config_channel, buf, len);
	} else {
		// this is a "user command" that must be sent as "DCS command"
		r = in->ops->dsi.dcs_write_nosync(in, ddata->config_channel, buf, len);
	}

	if (r)
		dev_err(&ddata->pdev->dev, "write cmd/reg(%x) failed: %d\n",
			buf[0], r);

	return r;
}

static int r63311_read(struct omap_dss_device *dssdev, u8 dcs_cmd, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->dssdev.src;
	int r;
	int i;

	r = in->ops->dsi.set_max_rx_packet_size(in, ddata->config_channel, len);	// tell panel how much we expect
	if (r) {
		dev_err(&ddata->pdev->dev, "can't set max rx packet size\n");
		return -EIO;
	}

	if (IS_MCS(dcs_cmd)) {
		// this is a "manufacturer command" that must be sent as a "generic read command"
		r = in->ops->dsi.gen_read(in, ddata->config_channel, &dcs_cmd, 1, buf, len);
	} else {
		// this is a "user command" that must be sent as "DCS command"
		r = in->ops->dsi.dcs_read(in, ddata->config_channel, dcs_cmd, buf, len);
	}

	if (r)
		dev_err(&ddata->pdev->dev, "read cmd/reg(%02x, %d) failed: %d\n",
			dcs_cmd, len, r);
	printk("dsi: r63311_read(%02x,", dcs_cmd); for (i = 0; i < len; i++) printk(" %02x", buf[i]);
	printk(") -> %d\n", r);
	return r;
}

static int r63311_write_sequence(struct omap_dss_device *dssdev,
				 struct r63311_reg *seq, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r, i;

	for (i = 0; i < len; i++) {
		r = r63311_write(dssdev, seq[i].data, seq[i].len);
		if (r) {
			dev_err(&ddata->pdev->dev, "sequence failed: %d\n", i);
			return -EINVAL;
		}

		/* TODO: Figure out if and why this is needed for OMAP5 */
		msleep(1);
	}

	return 0;
}

static int r63311_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops->connect(in, dssdev);
	if (r) {
		dev_err(dev, "Failed to connect to video source\n");
		return r;
	}

	/* channel0 used for video packets */
	r = in->ops->dsi.request_vc(ddata->dssdev.src, &ddata->pixel_channel);
	if (r) {
		dev_err(dev, "failed to get virtual channel\n");
		goto err_req_vc0;
	}

	r = in->ops->dsi.set_vc_id(ddata->dssdev.src, ddata->pixel_channel, 0);
	if (r) {
		dev_err(dev, "failed to set VC_ID\n");
		goto err_vc_id0;
	}

	/* channel1 used for registers access in LP mode */
	r = in->ops->dsi.request_vc(ddata->dssdev.src, &ddata->config_channel);
	if (r) {
		dev_err(dev, "failed to get virtual channel\n");
		goto err_req_vc1;
	}

	r = in->ops->dsi.set_vc_id(ddata->dssdev.src, ddata->config_channel, 0);
	if (r) {
		dev_err(dev, "failed to set VC_ID\n");
		goto err_vc_id1;
	}

	return 0;

err_vc_id1:
	in->ops->dsi.release_vc(ddata->dssdev.src, ddata->config_channel);
err_req_vc1:
err_vc_id0:
	in->ops->dsi.release_vc(ddata->dssdev.src, ddata->pixel_channel);
err_req_vc0:
	in->ops->disconnect(in, dssdev);
	return r;
}

static void r63311_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops->dsi.release_vc(in, ddata->pixel_channel);
	in->ops->dsi.release_vc(in, ddata->config_channel);
	in->ops->disconnect(in, dssdev);
}


static void r63311_get_timings(struct omap_dss_device *dssdev,
			       struct videomode *timings)
{
	*timings = dssdev->panel.vm;
}

static void r63311_set_timings(struct omap_dss_device *dssdev,
			       struct videomode *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	ddata->vm.hactive = timings->hactive;
	ddata->vm.vactive = timings->vactive;
	ddata->vm.pixelclock = timings->pixelclock;
	ddata->vm.hsync_len = timings->hsync_len;
	ddata->vm.hfront_porch = timings->hfront_porch;
	ddata->vm.hback_porch = timings->hback_porch;
	ddata->vm.vsync_len = timings->vsync_len;
	ddata->vm.vfront_porch = timings->vfront_porch;
	ddata->vm.vback_porch = timings->vback_porch;
}

static int r63311_check_timings(struct omap_dss_device *dssdev,
				struct videomode *timings)
{
	return 0;
}

#if 0
static void r63311_get_resolution(struct omap_dss_device *dssdev,
				  u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.vm.hactive;
	*yres = dssdev->panel.vm.vactive;
}
#endif

static int r63311_reset(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: r63311_reset(%d)\n", state);
	if (gpio_is_valid(ddata->reset_gpio))
		gpio_set_value(ddata->reset_gpio, state);
	return 0;
}

static int r63311_regulator(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: r63311_regulator(%d)\n", state);
	if (gpio_is_valid(ddata->regulator_gpio))
		gpio_set_value(ddata->regulator_gpio, state);	// switch regulator
	return 0;
}

static int r63311_update_brightness(struct omap_dss_device *dssdev, int level)
{
	int r;
#if 1
	u8 buf[2];
	buf[0] = DCS_BRIGHTNESS;
	buf[1] = level;
#else
	u8 buf[3];
	buf[0] = DCS_BRIGHTNESS;
	buf[1] = level >> 4;	// 12 bit mode
	buf[2] = buf[1] + ((level & 0x0f) << 4);
#endif
	r = r63311_write(dssdev, buf, sizeof(buf));
	if (r)
		return r;
	return 0;
}

static int r63311_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int bl = bd->props.brightness;
	int r = 0;
	printk("dsi: r63311_set_brightness(%d)\n", bl);

	if (bl == ddata->bl)
		return 0;

	mutex_lock(&ddata->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		struct omap_dss_device *in = ddata->dssdev.src;
		in->ops->dsi.bus_lock(in);

		r = r63311_update_brightness(dssdev, bl);
		if (!r)
			ddata->bl = bl;

		in->ops->dsi.bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	return r;
}

static int r63311_get_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	u8 data[16];
	u16 brightness = 0;
	int r = 0;
	printk("dsi: r63311_get_brightness()\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk("dsi: display is not active\n");
		return 0;
	}

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		struct omap_dss_device *in = ddata->dssdev.src;
		in->ops->dsi.bus_lock(in);
		r = r63311_read(dssdev, DCS_READ_BRIGHTNESS, data, 2);
		brightness = (data[0]<<4) + (data[1]>>4);

		in->ops->dsi.bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	if(r < 0) {
		printk("dsi: read error\n");
		return bd->props.brightness;
	}
	printk("dsi: read %d\n", brightness);
	return brightness>>4;	// get to range 0..255
}

static const struct backlight_ops r63311_backlight_ops  = {
	.get_brightness = r63311_get_brightness,
	.update_status = r63311_set_brightness,
};

/* sysfs callbacks */

static int r63311_start(struct omap_dss_device *dssdev);
static void r63311_stop(struct omap_dss_device *dssdev);

static ssize_t set_dcs(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int r = 0;
	u8 data[24];
	u8 d = 0;
	int argc = 0;
	int second = 0;
	int read = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;

	const char *p;

	if(strncmp(buf, "start", 5) == 0)
		{
		int r = r63311_start(dssdev);
		return r < 0 ? r : count;
		}
	if(strncmp(buf, "stop", 4) == 0)
		{
		r63311_stop(dssdev);
		return count;
		}
	if(strncmp(buf, "reset", 5) == 0)
		{
		r63311_reset(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "noreset", 7) == 0)
		{
		r63311_reset(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "power", 5) == 0)
		{
		r63311_regulator(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "nopower", 7) == 0)
		{
		r63311_regulator(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "status", 6) == 0) {
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			struct omap_dss_device *in = ddata->dssdev.src;
			in->ops->dsi.bus_lock(in);
			r = r63311_read(dssdev, 0xbf, data, 5);	// R63311 chip ID
			r = r63311_read(dssdev, 0xb0, data, 1);	// MCS access protection
			r = r63311_read(dssdev, 0xb5, data, 3);	// checksum and ECC errors
			r = r63311_read(dssdev, 0x04, data, 16);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			r = r63311_read(dssdev, DCS_READ_NUM_ERRORS, data, 1);	// dsi errors
			//		r = r63311_read(dssdev, 0x06, data, 1);	// red
			//		r = r63311_read(dssdev, 0x07, data, 1);	// green
			//		r = r63311_read(dssdev, 0x08, data, 1);	// blue
			r = r63311_read(dssdev, 0x0a, data, 1);	// power mode 0x10=sleep off; 0x04=display on
			//		r = r63311_read(dssdev, 0x0b, data, 1);	// address mode
			r = r63311_read(dssdev, MIPI_DCS_GET_PIXEL_FORMAT, data, 1);	// pixel format 0x70 = RGB888
			r = r63311_read(dssdev, 0x0d, data, 1);	// display mode	0x80 = command 0x34/0x35
			r = r63311_read(dssdev, 0x0e, data, 1);	// signal mode
			r = r63311_read(dssdev, MIPI_DCS_GET_DIAGNOSTIC_RESULT, data, 1);	// diagnostic 0x40 = functional
			r = r63311_read(dssdev, 0x45, data, 2);	// get scanline
			r = r63311_read(dssdev, 0x52, data, 2);	// brightness
			r = r63311_read(dssdev, 0x56, data, 1);	// adaptive brightness
			r = r63311_read(dssdev, 0x5f, data, 2);	// CABC minimum
			r = r63311_read(dssdev, 0x68, data, 1);	// auto brightness
			r = r63311_read(dssdev, 0xa1, data, 16);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			r = r63311_read(dssdev, 0xda, data, 1);	// ID1
			r = r63311_read(dssdev, 0xdb, data, 1);	// ID2
			r = r63311_read(dssdev, 0xdc, data, 1);	// ID3
			in->ops->dsi.bus_unlock(in);
		}
		mutex_unlock(&ddata->lock);
		return r < 0 ? r : count;
	}
	if(strncmp(buf, "test", 4) == 0)
		{
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			struct omap_dss_device *in = ddata->dssdev.src;
			in->ops->dsi.bus_lock(in);
			r = r63311_write_sequence(dssdev, test_image, ARRAY_SIZE(test_image));
			in->ops->dsi.bus_unlock(in);
		}
		mutex_unlock(&ddata->lock);
		return r < 0 ? r : count;
		}

	for(p = buf; p < buf + count && argc < sizeof(data); p++)
		{

//		printk("  2nd:%d argc:%d read:%d %c\n", second, argc, read, *p);

		if(!second && (*p == ' ' || *p == '\t' || *p == '\n'))
			continue;
		if(!second && argc >= 1 && *p == 'r')	// r must follow the address (or another r)
			{
			data[argc++]=0;
			read = 1;
			continue;
			}
		if(read && argc > 1)
			return -EIO;	// no hex digits after the first r
		if(*p >= '0' && *p <= '9')
			d=(d<<4) + (*p-'0');
		else if(*p >= 'a' && *p <= 'f')
			d=(d<<4) + (*p-'a') + 10;
		else if(*p >= 'A' && *p <= 'F')
			d=(d<<4) + (*p-'A') + 10;
		else
			return -EIO;
		if(second)
			data[argc++]=d;	// store every second digit
		second ^= 1;
		}

//	printk("  2nd:%d argc:%d ---\n", second, argc);

	if(second)
		return -EIO;	// not an even number of digits

	if(argc == 0)
		return -EIO;	// missing address

	mutex_lock(&ddata->lock);
	if (ddata->enabled) {
		struct omap_dss_device *in = ddata->dssdev.src;
		in->ops->dsi.bus_lock(in);

		if(read)
			r = r63311_read(dssdev, data[0], &data[1], argc-1);
		else
			r = r63311_write(dssdev, data, argc);

		in->ops->dsi.bus_unlock(in);
	} else
		r=-EIO;	// not enabled
	mutex_unlock(&ddata->lock);

	return r < 0 ? r : count;
}

static ssize_t show_dcs(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "XX XX ... | XX r ... | test | status | start | stop | reset | noreset | regulator | noregulator\n");
}

static DEVICE_ATTR(dcs, S_IWUSR | S_IRUGO,
				   show_dcs, set_dcs);

static struct attribute *r63311_attributes[] = {
	&dev_attr_dcs.attr,
	NULL
};

static const struct attribute_group r63311_attr_group = {
	.attrs = r63311_attributes,
};

static int r63311_power_on(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->pdev->dev;
	struct omap_dss_device *in = ddata->dssdev.src;
	int r;
	struct omap_dss_dsi_config r63311_dsi_config = {
		.mode = OMAP_DSS_DSI_VIDEO_MODE,
		.pixel_format = R63311_PIXELFORMAT,
		.vm = &ddata->vm,
		.hs_clk_min = 125000000 /*R63311_HS_CLOCK*/,
		.hs_clk_max = 450000000 /*(12*R63311_HS_CLOCK)/10*/,
		.lp_clk_min = (7*R63311_LP_CLOCK)/10,
		.lp_clk_max = R63311_LP_CLOCK,
		.ddr_clk_always_on = true,
		.trans_mode = OMAP_DSS_DSI_BURST_MODE,
	};
//	printk("hs_clk_min=%lu\n", r63311_dsi_config.hs_clk_min);
	printk("dsi: r63311_power_on()\n");

	r63311_reset(dssdev, 0);	// activate reset

#if 0
	if (ddata->pin_config.num_pins > 0) {
		r = in->ops->dsi.configure_pins(in, &ddata->pin_config);
		if (r) {
			dev_err(&ddata->pdev->dev,
					"failed to configure DSI pins\n");
			goto err0;
		}
	}
#endif

	r = in->ops->dsi.set_config(in, &r63311_dsi_config);
	if (r) {
		dev_err(dev, "failed to configure DSI\n");
		goto err0;
	}

	r = in->ops->enable(in);
	if (r) {
		dev_err(dev, "failed to enable DSI\n");
		goto err0;
	}


	r63311_regulator(dssdev, 1);	// switch power on
	msleep(50);

	r63311_reset(dssdev, 1);	// release reset
	msleep(10);


	in->ops->dsi.enable_hs(in, ddata->pixel_channel, true);

	r = r63311_write_sequence(dssdev, init_seq, ARRAY_SIZE(init_seq));
	if (r) {
		dev_err(dev, "failed to configure panel\n");
		goto err;
	}

	msleep(20);

	r = r63311_update_brightness(dssdev, 255);
	if (r)
		goto err;

	r = r63311_write_sequence(dssdev, sleep_out, ARRAY_SIZE(sleep_out));
	if (r)
		goto err;


	r = in->ops->dsi.enable_video_output(in, ddata->pixel_channel);
	if (r)
		goto err;

	msleep(120);

#if 0	// this is recommended by the latest data sheet
	r = r63311_write_sequence(dssdev, display_on, ARRAY_SIZE(display_on));
	if (r)
		goto err;
#endif
	ddata->enabled = true;
	printk("dsi: powered on()\n");

	return r;
err:
	printk("dsi: power on error\n");
	dev_err(dev, "error while enabling panel, issuing HW reset\n");

	in->ops->dsi.disable(in, false, false);
	mdelay(10);
	r63311_reset(dssdev, 0);	// activate reset
	r63311_regulator(dssdev, 0);	// switch power off
	mdelay(20);

err0:
	return r;
}

// we don't have a sophisticated power management (sending the panel to power off)
// we simply stop the video stream and assert the RESET
// please note that we don't/can't switch off the VCCIO

static void r63311_power_off(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->dssdev.src;

	printk("dsi: r63311_power_off()\n");

	ddata->enabled = false;
	in->ops->dsi.disable_video_output(in, ddata->pixel_channel);
	in->ops->dsi.disable(in, false, false);
	mdelay(10);
	r63311_reset(dssdev, 0);	// activate reset
	r63311_regulator(dssdev, 0);	// switch power off - after stopping video stream
	mdelay(20);
	/* here we can also power off IOVCC */
}

static int r63311_start(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->dssdev.src;
	int r = 0;

	printk("dsi: r63311_start()\n");
	mutex_lock(&ddata->lock);

	in->ops->dsi.bus_lock(in);

	r = r63311_power_on(dssdev);

	in->ops->dsi.bus_unlock(in);

	if (r)
		dev_err(&ddata->pdev->dev, "enable failed\n");
	else
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ddata->lock);

	return r;
}

static void r63311_stop(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->dssdev.src;

	printk("dsi: r63311_stop()\n");
	mutex_lock(&ddata->lock);

	in->ops->dsi.bus_lock(in);

	r63311_power_off(dssdev);

	in->ops->dsi.bus_unlock(in);

	mutex_unlock(&ddata->lock);
}

static void r63311_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: r63311_disable()\n");
	dev_dbg(&ddata->pdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		r63311_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int r63311_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: r63311_enable()\n");
	dev_dbg(&ddata->pdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return r63311_start(dssdev);
}

static struct omap_dss_driver r63311_ops = {
	.connect	= r63311_connect,
	.disconnect	= r63311_disconnect,

	.enable		= r63311_enable,
	.disable	= r63311_disable,

#if 0
	.get_resolution	= r63311_get_resolution,
#endif

	.check_timings	= r63311_check_timings,
	.set_timings	= r63311_set_timings,
	.get_timings	= r63311_get_timings,
};

static int r63311_probe_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *in;
	int gpio;

	printk("dsi: r63311_probe_of()\n");

	gpio = of_get_gpio(node, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse reset gpio (err=%d)\n", gpio);
		return gpio;
	}
	ddata->reset_gpio = gpio;

	gpio = of_get_gpio(node, 1);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse regulator gpio (err=%d)\n", gpio);
		return gpio;
	}
	ddata->regulator_gpio = gpio;

	in = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(in)) {
		dev_err(&pdev->dev, "failed to find video source (err=%ld)\n", PTR_ERR(in));
		return PTR_ERR(in);
	}

	ddata->in = in;

	return 0;
}

static int r63311_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev = NULL;
	struct panel_drv_data *ddata;
	struct device *dev = &pdev->dev;
	struct omap_dss_device *dssdev;
	int r;

	printk("dsi: r63311_probe()\n");
	dev_dbg(dev, "r63311_probe\n");

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);
	ddata->pdev = pdev;

	if (dev_get_platdata(dev)) {
		r = -EINVAL /*r63311_probe_pdata(pdev)*/;
		if (r)
			return r;
	} else if (pdev->dev.of_node) {
		r = r63311_probe_of(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	ddata->timings = r63311_timings;

	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &r63311_ops;
	dssdev->panel.vm = r63311_timings;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;

	dssdev->panel.dsi_pix_fmt = R63311_PIXELFORMAT;

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(dev, "Failed to register panel\n");
		goto err_reg;
	}

	mutex_init(&ddata->lock);

	if (gpio_is_valid(ddata->reset_gpio)) {
		r = devm_gpio_request_one(&pdev->dev, ddata->reset_gpio,
								  GPIOF_DIR_OUT, "lcd reset");
		if (r) {
			dev_err(dev, "failed to request reset gpio (%d err=%d)\n", ddata->reset_gpio, r);
			return r;
		}
	}

	if (gpio_is_valid(ddata->regulator_gpio)) {
		r = devm_gpio_request_one(dev, ddata->regulator_gpio,
								  GPIOF_DIR_OUT, "lcd DC/DC regulator");
		if (r) {
			dev_err(dev, "failed to request regulator gpio (%d err=%d)\n", ddata->regulator_gpio, r);
			return r;
		}
	}

	/* Register sysfs hooks */
	r = sysfs_create_group(&dev->kobj, &r63311_attr_group);
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_sysfs_create;
	}

	printk("r63311_probe ok\n");

	return 0;

err_sysfs_create:
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_bl:
//	destroy_workqueue(ddata->workqueue);
err_reg:
	return r;
}

static int __exit r63311_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct backlight_device *bldev;

	printk("dsi: r63311_remove()\n");

	omapdss_unregister_display(dssdev);

	r63311_disable(dssdev);
	r63311_disconnect(dssdev);

	sysfs_remove_group(&pdev->dev.kobj, &r63311_attr_group);

	omap_dss_put_device(ddata->in);

	mutex_destroy(&ddata->lock);

	return 0;
}

static const struct of_device_id r63311_of_match[] = {
	{
		.compatible = "omapdss,lg,lh500wf1",
	},
	{},
};

MODULE_DEVICE_TABLE(of, r63311_of_match);

static struct platform_driver r63311_driver = {
	.probe = r63311_probe,
	.remove = __exit_p(r63311_remove),
	.driver = {
		.name = "r63311",
		.owner = THIS_MODULE,
		.of_match_table = r63311_of_match,
	},
};

module_platform_driver(r63311_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("r63311 driver");
MODULE_LICENSE("GPL");
