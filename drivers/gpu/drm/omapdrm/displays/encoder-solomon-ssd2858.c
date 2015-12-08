/*
 * Driver for panels through Solomon Systech SSD2858 rotator chip
 *
 * Device Tree FIXME:
 *   this chip is defined with a pair "port {}" elements
 *   to keep the panel separated from the DSI interface
 *   so it is an "encoder" with an input and an output port
 *   Device Tree could config bypass, rotation and some other paramters
 *   that are used to initialize the chip.
 *   The current implementation is still a mix of SSD2858 plus Panel specific
 *   code and needs a lot of work to be finialised.
 *
 *
 * Copyright 2011 Texas Instruments, Inc.
 * Author: Archit Taneja <archit@ti.com>
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
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

#include <video/omapdss.h>
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
#define DCS_READ_CTRL_DISPLAY	0x53	// read control
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
#define MCS_READID1		0xda
#define MCS_READID2		0xdb
#define MCS_READID3		0xdc

/* manufacturer specific commands */
#define MCS_MANUFPROT	0xb0
#define MCS_SETDEEPSTBY	0xb1
#define MCS_IFACESET	0xb3
#define MCS_MIPISPEED	0xb6
#define MCS_DISPLSET1	0xc1
#define MCS_DISPLSET2	0xc2
#define MCS_VSYNCEN		0xc3
#define MCS_SRCTIMING	0xc4
#define MCS_LPTSTIMING	0xc6
#define MCS_GAMMA_A		0xc7
#define MCS_GAMMA_B		0xc8
#define MCS_GAMMA_C		0xc9
#define MCS_PANELIFACE	0xcc
#define MCS_CHARGEPUMP	0xd0
#define MCS_POWERSET	0xd3
#define MCS_VCOMSET		0xd5

#define IS_MCS(CMD) (((CMD) >= 0xb0 && (CMD) <= 0xff) && !((CMD) == 0xda || (CMD) == 0xdb || (CMD) == 0xdc))

/* horizontal * vertical * refresh */
#define SSD2858_W				(720)
#define SSD2858_H				(1280)
#define SSD2858_WIDTH			(SSD2858_W+280)
#define SSD2858_HEIGHT			(SSD2858_H+160)
#define SSD2858_FPS				(60ll)
#define SSD2858_PIXELCLOCK		(SSD2858_WIDTH * SSD2858_HEIGHT * SSD2858_FPS)	// Full HD * 60 fps
/* panel has 16.7M colors = RGB888 = 3*8 bit per pixel */
#define SSD2858_PIXELFORMAT		OMAP_DSS_DSI_FMT_RGB888	// 16.7M color = RGB888
#define SSD2858_BIT_PER_PIXEL	(3*8)
/* the panel can handle 4 lanes */
#define SSD2858_LANES			4
/* high speed clock is running at double data rate, i.e. half speed
 * (take care of integer overflows!)
 * hsck =  bit/pixel * 110% * pixel clock / lanes / 2 clock edges
 * real clock rate may be rounded up or down depending on divisors
 */
#define SSD2858_HS_CLOCK			(SSD2858_BIT_PER_PIXEL * (SSD2858_PIXELCLOCK / (SSD2858_LANES * 2)))
/* low power clock is quite arbitrarily choosen to be roughly 10 MHz */
#define SSD2858_LP_CLOCK			10000000	// low power clock

static struct omap_video_timings ssd2858_timings = {
	.x_res		= SSD2858_W,
	.y_res		= SSD2858_H,
	.pixelclock	= SSD2858_PIXELCLOCK,
	// they are choosen to round up to XTOTALxYTOTAL pixels giving a pixel clock of 86.400 MHz
	.hfp		= 20,
	.hsw		= 20,
	.hbp		= 240,	// sum must match XTOTAL-XRES
	.vfp		= 50,
	.vsw		= 60,
	.vbp		= 50,	// sum must match YTOTAL-YRES
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;
	struct omap_dss_device *out;	/* should be the real panel connected to the SSD */

	struct omap_video_timings timings;
	
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

struct ssd2858_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

static struct ssd2858_reg init_seq[] = {
	/* see Table 3.9.2 Display Initial Set */
	/* we should define some macros... */
#if 0
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_MANUFPROT,
		0x00	// remove access protect (except NVM)
	}, 2 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_IFACESET,
		0x14, 0x00, 0x00, 0x00, 0x00,
		0x00
	}, 7 },
	{ {	MCS_MIPISPEED,
		0x3A, 0xD3 }, 3 },
	{ {	MCS_DISPLSET1,
		0x84, 0x60, 0x40, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0C,
		0x01, 0x58, 0x73, 0xAE, 0x31,
		0x20, 0x06, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x10, 0x10,
		0x10, 0x10, 0x00, 0x00, 0x00,
		0x22, 0x02, 0x02, 0x00
	}, 35 },
	{ {	MCS_DISPLSET2,
		0x31, 0xF7, 0x80, 0x0A, 0x08,
		0x00, 0x00
	}, 8 },
	{ {	MCS_VSYNCEN,
		0x01, 0x00, 0x00,
	}, 4 },
	{ {	MCS_SRCTIMING,
		0x70, 0x00, 0x00, 0x00, 0x07,
		0x05, 0x05, 0x09, 0x09, 0x0c,
		0x06, 0x00, 0x00, 0x00, 0x00,
		0x07, 0x05, 0x05, 0x09, 0x09,
		0x0c, 0x06
	}, 23 },
	{ {	MCS_LPTSTIMING,
		0x00, 0x69, 0x00, 0x69, 0x00,
		0x69, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x69, 0x00, 0x69, 0x00,
		0x69, 0x10, 0x19, 0x07, 0x00,
		0x01, 0x00, 0x69, 0x00, 0x69,
		0x00, 0x69, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x69, 0x00, 0x69,
		0x00, 0x69, 0x10, 0x19, 0x07
	}, 41 },
	{ {	MCS_GAMMA_A,
		0x00, 0x09, 0x14, 0x26, 0x31,
		0x48, 0x3B, 0x52, 0x5F, 0x67,
		0x6B, 0x70, 0x00, 0x09, 0x14,
		0x26, 0x31, 0x48, 0x3B, 0x52,
		0x5F, 0x67, 0x6B, 0x70
	}, 25 },
	{ {	MCS_GAMMA_B,
		0x00, 0x09, 0x14, 0x26, 0x31,
		0x48, 0x3B, 0x52, 0x5F, 0x67,
		0x6B, 0x70, 0x00, 0x09, 0x14,
		0x26, 0x31, 0x48, 0x3B, 0x52,
		0x5F, 0x67, 0x6B, 0x70
	}, 25 },
	{ {	MCS_GAMMA_C,
		0x00, 0x09, 0x14, 0x26, 0x31,
		0x48, 0x3B, 0x52, 0x5F, 0x67,
		0x6B, 0x70, 0x00, 0x09, 0x14,
		0x26, 0x31, 0x48, 0x3B, 0x52,
		0x5F, 0x67, 0x6B, 0x70
	}, 25 },
	{ {	MCS_PANELIFACE,
		0x09 }, 2 },
#endif
#if 0
	{ {	MCS_CHARGEPUMP,
		0x00, 0x00, 0x19, 0x18, 0x99,
		0x99, 0x19, 0x01, 0x89, 0x00,
		0x55, 0x19, 0x99, 0x01,
	}, 15 },
#endif
#if 0
	{ {	MCS_POWERSET,
		0x1B, 0x33, 0xBB, 0xCC, 0xC4,
		0x33, 0x33, 0x33, 0x00, 0x01,
		0x00, 0xA0, 0xD8, 0xA0, 0x0D,
		0x37, 0x33, 0x44, 0x22, 0x70,
		0x02, 0x37, 0x03, 0x3D, 0xBF,
		0x00
	}, 27 },
	{ {	MCS_VCOMSET,
		0x06, 0x00, 0x00, 0x01, 0x39,
		0x01, 0x39
	}, 8 },
	{ {	MCS_VCOMSET,	/* second time */
		0x06, 0x00, 0x00, 0x01, 0x39,
		0x01, 0x39
	}, 8 },
#endif
#if 0
	{ {	0xce,
		0x00, 0x01, 0x40, 0xc1, 0x00,
		0x00, 0x00
	}, 8 },
	{ { DCS_CTRL_DISPLAY, 0x24}, 2 },	// LEDPWM ON
	{ { DCS_WRITE_CABC, 0x00}, 2 },		// CABC off
#endif
};

static struct ssd2858_reg test_image[] = {
	/* switch panel to test mode */
#if 0
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
#endif
};

static struct ssd2858_reg sleep_out[] = {
	{ { MIPI_DCS_SET_DISPLAY_ON, }, 1 },
	{ { MIPI_DCS_EXIT_SLEEP_MODE, }, 1 },
};

static struct ssd2858_reg display_on[] = {
#if 0
	{ {	MCS_VCOMSET,
		0x06, 0x00, 0x00, 0x01, 0x2c,
		0x01, 0x2c
	}, 8 },
	{ {	MCS_VCOMSET,	/* second time */
		0x06, 0x00, 0x00, 0x01, 0x2c,
		0x01, 0x2c
	}, 8 },
#endif
};

/* not used - needs also MCS_VCOMSET and some delays */

static struct ssd2858_reg display_off[] = {
	{ { MIPI_DCS_SET_DISPLAY_OFF, }, 1},
};

static struct ssd2858_reg sleep_in[] = {
	{ { MIPI_DCS_ENTER_SLEEP_MODE, }, 1},
};

/* communication with the SSD chip */

static int mode=0;	// starts in non-bypass mode

static int ssd2858_write(struct omap_dss_device *dssdev, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	printk("dsi: ssd2858_write(%s", IS_MCS(buf[0])?"g":""); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");

	if(IS_MCS(buf[0]))
		{ // this is a "manufacturer command" that must be sent as a "generic write command"
			r = in->ops.dsi->gen_write(in, ddata->config_channel, buf, len);
		}
	else
		{ // this is a "user command" that must be sent as "DCS command"
			r = in->ops.dsi->dcs_write_nosync(in, ddata->config_channel, buf, len);
		}

	if (r)
		dev_err(&ddata->pdev->dev, "write cmd/reg(%x) failed: %d\n",
				buf[0], r);

	return r;
}

static int ssd2858_pass_to_panel(struct omap_dss_device *dssdev, int enable)
{ // choose destination of further commands: SSD chip or panel
	int r = 0;
	// if !enable and ssd is in reset => error
	if(mode != enable)
		{ // write a "special control packet" to switch pass through modes
			u8 buf[2];
			buf[0] = 0xff;
			buf[1] = (mode = enable);
			r = ssd2858_write(dssdev, buf, sizeof(buf));
		}
	return r;
}

static int ssd2858_read(struct omap_dss_device *dssdev, u8 dcs_cmd, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	r = in->ops.dsi->set_max_rx_packet_size(in, ddata->config_channel, len);	// tell panel how much we expect
	if (r) {
		dev_err(&ddata->pdev->dev, "can't set max rx packet size\n");
		return -EIO;
	}

	if(IS_MCS(dcs_cmd))
		{ // this is a "manufacturer command" that must be sent as a "generic read command"
			r = in->ops.dsi->gen_read(in, ddata->config_channel, &dcs_cmd, 1, buf, len);
		}
	else
		{ // this is a "user command" that must be sent as "DCS command"
			r = in->ops.dsi->dcs_read(in, ddata->config_channel, dcs_cmd, buf, len);
		}

	if (r)
		dev_err(&ddata->pdev->dev, "read cmd/reg(%02x, %d) failed: %d\n",
				dcs_cmd, len, r);
	printk("dsi: ssd2858_read(%02x,", dcs_cmd); for(i=0; i<len; i++) printk(" %02x", buf[i]);
	printk(") -> %d\n", r);
	return r;
}

static int ssd2858_write_sequence(struct omap_dss_device *dssdev,
		struct ssd2858_reg *seq, int len)
{
	int r, i;
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	ssd2858_pass_to_panel(dssdev, false);	// initialization for the SSD

	for (i = 0; i < len; i++) {
		r = ssd2858_write(dssdev, seq[i].data, seq[i].len);
		if (r) {
			dev_err(&ddata->pdev->dev, "sequence failed: %d\n", i);
			return -EINVAL;
		}

		/* TODO: Figure out why this is needed for OMAP5 */
		msleep(1);
	}

	return 0;
}

static int ssd2858_write_reg(struct omap_dss_device *dssdev, unsigned short address, unsigned long data)
{
	int r;
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	u8 buf[6];
	buf[0] = address >> 8;
	buf[1] = address >> 0;
	buf[2] = data >> 24;
	buf[3] = data >> 16;
	buf[4] = data >> 8;
	buf[5] = data >> 0;

	ssd2858_pass_to_panel(dssdev, false);	// initialization for the SSD

	r = in->ops.dsi->gen_write(in, ddata->config_channel, buf, 6);

#if 1
	printk("ssd2858_write_reg: %04x <- %08lx (r=%d)\n", address, data, r);
#endif
	return r;
}

static int ssd2858_read_reg(struct omap_dss_device *dssdev, unsigned short address, unsigned long *data)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	u8 buf[6];

	ssd2858_pass_to_panel(dssdev, false);	// initialization for the SSD

	r = in->ops.dsi->set_max_rx_packet_size(in, ddata->config_channel, 4);	// tell panel how much we expect
	if (r) {
		dev_err(&ddata->pdev->dev, "can't set max rx packet size\n");
		return -EIO;
	}

	buf[0] = address >> 8;
	buf[1] = address >> 0;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 0;
	buf[5] = 0;
	r = in->ops.dsi->gen_read(in, ddata->config_channel, buf, 2, &buf[2], 4);
	*data = buf[2] << 24;
	*data|= buf[3] << 16;
	*data|= buf[4] << 8;
	*data|= buf[5] << 0;
#if 1
	printk("ssd2858_read_reg: %04x -> %08lx (r=%d)\n", address, *data, r);
#endif
	return r;
}

static int ssd2858_write_reg_sequence(struct omap_dss_device *dssdev, void *seq, int len)
{
	// tbd.
	return -EINVAL;
}

/* callbacks from the DSI driver */

static int ssd2858_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;
	
	if (omapdss_device_is_connected(dssdev))
		return 0;
	
	r = in->ops.dsi->connect(in, dssdev);
	if (r) {
		dev_err(dev, "Failed to connect to video source\n");
		return r;
	}
	
	/* channel0 used for video packets */
	r = in->ops.dsi->request_vc(ddata->in, &ddata->pixel_channel);
	if (r) {
		dev_err(dev, "failed to get virtual channel\n");
		goto err_req_vc0;
	}
	
	r = in->ops.dsi->set_vc_id(ddata->in, ddata->pixel_channel, 0);
	if (r) {
		dev_err(dev, "failed to set VC_ID\n");
		goto err_vc_id0;
	}
	
	/* channel1 used for registers access in LP mode */
	r = in->ops.dsi->request_vc(ddata->in, &ddata->config_channel);
	if (r) {
		dev_err(dev, "failed to get virtual channel\n");
		goto err_req_vc1;
	}
	
	r = in->ops.dsi->set_vc_id(ddata->in, ddata->config_channel, 0);
	if (r) {
		dev_err(dev, "failed to set VC_ID\n");
		goto err_vc_id1;
	}
	// FIXME: forward connect to panel

	return 0;
	
err_vc_id1:
	in->ops.dsi->release_vc(ddata->in, ddata->config_channel);
err_req_vc1:
err_vc_id0:
	in->ops.dsi->release_vc(ddata->in, ddata->pixel_channel);
err_req_vc0:
	in->ops.dsi->disconnect(in, dssdev);
	return r;
}

static void ssd2858_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	
	if (!omapdss_device_is_connected(dssdev))
		return;
	// FIXME: forward disconnect to panel

	in->ops.dsi->release_vc(in, ddata->pixel_channel);
	in->ops.dsi->release_vc(in, ddata->config_channel);
	in->ops.dsi->disconnect(in, dssdev);
}


static void ssd2858_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void ssd2858_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixelclock = timings->pixelclock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
	// FIXME: forward to panel
}

static int ssd2858_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return 0;
}

static void ssd2858_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
	// FIXME: forward to panel?
}

/* hardware control */

static int ssd2858_reset(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_reset(%d)\n", state);
	if (gpio_is_valid(ddata->reset_gpio))
		gpio_set_value(ddata->reset_gpio, state);
	// FIXME: forward to panel
	return 0;
}

static int ssd2858_regulator(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_regulator(%d)\n", state);
	if (gpio_is_valid(ddata->regulator_gpio))
		gpio_set_value(ddata->regulator_gpio, state);	// switch regulator
	// FIXME: forward to panel regulator
	return 0;
}

/* backlight control */

static int ssd2858_update_brightness(struct omap_dss_device *dssdev, int level)
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
	r = ssd2858_write(dssdev, buf, sizeof(buf));
	if (r)
		return r;
	return 0;
}

static int ssd2858_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
//	struct omap_dss_device *in = ddata->in;
	int bl = bd->props.brightness;
	int r = 0;
	printk("dsi: ssd2858_set_brightness(%d)\n", bl);
	if (bl == ddata->bl)
		return 0;

#if 0

	mutex_lock(&ddata->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		in->ops.dsi->bus_lock(in);

		r = ssd2858_update_brightness(dssdev, bl);
		if (!r)
			ddata->bl = bl;

		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);
#endif

	return r;
}

static int ssd2858_get_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	u8 data[16];
	u16 brightness = 0;
	int r = 0;
	printk("dsi: ssd2858_get_brightness()\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk("dsi: display is not active\n");
		return 0;
	}

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);
		r = ssd2858_read(dssdev, DCS_READ_BRIGHTNESS, data, 2);
		brightness = (data[0]<<4) + (data[1]>>4);

		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	if(r < 0) {
		printk("dsi: read error\n");
		return bd->props.brightness;
	}
	printk("dsi: read %d\n", brightness);
	return brightness>>4;	// get to range 0..255
}

static const struct backlight_ops ssd2858_backlight_ops  = {
	.get_brightness = ssd2858_get_brightness,
	.update_status = ssd2858_set_brightness,
};

/* sysfs callbacks */

static int ssd2858_start(struct omap_dss_device *dssdev);
static void ssd2858_stop(struct omap_dss_device *dssdev);

static ssize_t set_control(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int r = 0;
	// decode some commands like setting the rotation etc.
	return r < 0 ? r : count;
}

static ssize_t show_control(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "control the SSD chip\n");
}

static DEVICE_ATTR(control, S_IWUSR | S_IRUGO,
				   show_control, set_control);

static struct attribute *ssd2858_attributes[] = {
	&dev_attr_control.attr,
	NULL
};

static const struct attribute_group ssd2858_attr_group = {
	.attrs = ssd2858_attributes,
};

/* calculate timings */

#if 0	// we need to define the structs to make this code work

/* some of these values are defined by the panel driver */

struct paneltiming {
PANEL_WIDTH=720
PANEL_HEIGHT=1280
PANEL_FPS=60	// frames per second
PANEL_BPP=24	// bits per lane
PANEL_LANES=4	// lanes
PANEL_HFP=10	// front porch
PANEL_HSA=10	// sync active
PANEL_HBP=100	// back porch
// NOTE: we must set this so the sum of V* is < ~70 to get a slightly higher pixel and DDR rate or the panel wouldn't sync properly
// warning: some VBP values are causing horizontal misalignemt:
// 12..15, 18..23, 26..?, 34..40, ...?
PANEL_VFP=10	// top porch
PANEL_VSA=2	// sync active
PANEL_VBP=48	// bottom porch
PANEL_LPCLOCK=8000000	// maximum is 9.2 MHz
PANEL_MAX_DDR=250000000	// maximum DDR clock (4..25ns) that can be processed by controller
}

static void calculate_timings(struct omap_dss_device *dssdev, struct paneltiming *panel, bool rotate)
{
/* panel derived parameters */
unsigned int PANEL_FRAME_WIDTH = PANEL_WIDTH + PANEL_HFP + PANEL_HSA + PANEL_HBP	// some margin for sync and retrace
unsigned int PANEL_FRAME_HEIGHT = PANEL_HEIGHT + PANEL_VFP + PANEL_VSA + PANEL_VBP	// some margin for sync and retrace
unsigned long PANEL_PCLK = PANEL_FRAME_WIDTH * PANEL_FRAME_HEIGHT * PANEL_FPS	// required pixel clock
unsigned long PANEL_MIN_DDR = PANEL_PCLK / 2 / PANEL_LANES * PANEL_BPP	// min is defined by required data bandwidth

// SSD2858 mode parameters
int SPLIT_MEM = 1, TE_SEL = 1, VBP = 0;
int CKE = 0;
int VB_MODE = 0;
int VBE = 0;
int ROT90 = 0;

// SSD2858 clock divider parameters
unsigned int XTAL = 24000000;	// we have 24 MHz XTAL
unsigned int PCLK_NUM = 1, PCLK_DEN = 4; // must be 1:4 for proper video processing

unsigned int SYS_CLK_DIV = 5; // this is something to play with until all parameters fit the constraints
// we might also loop over all potential values until we find the lowest value that fulfills the constraints

// OMAP MIPI parameters
/* no specifc requirements checked or considered */

// SSD divider calculations

// VTCM
unsigned long TARGET_PIXEL_CLK = PANEL_PCLK / 2;
unsigned long TARGET_SYS_CLK = PCLK_DEN * TARGET_PIXEL_CLK / PCLK_NUM;
unsigned long TARGET_MAIN_CLK = 2 * SYS_CLK_DIV * TARGET_SYS_CLK;
unsigned int TARGET_DIV = 1500000000 / TARGET_MAIN_CLK;	// total required divisor between PLL and MAIN_CLK - rounded down (running the PLL at least at required frequency)
unsigned long TARGET_PLL = TARGET_DIV * TARGET_MAIN_CLK;	// what we expect to see as PLL frequency
// PLL
unsigned int PLL_MULT= (TARGET_PLL + XTAL - 1) / XTAL;	// required PLL multiplier from XTAL (rounded up)
unsigned long PLL = XTAL * PLL_MULT;	// real PLL frequency
unsigned int PLL_POST_DIV = PLL / TARGET_MAIN_CLK;	// PLL_POST_DIV to get MAIN_CLOCK - should be >= TARGET_DIV
unsigned long MAIN_CLK = PLL / PLL_POST_DIV;	// real MAIN clock
unsigned long SYS_CLK = MAIN_CLK / 2 / SYS_CLK_DIV;	// real SYS clock
unsigned long PIXEL_CLK = (SYS_CLK * PCLK_NUM) / PCLK_DEN;	// real VTCM pixel clock
// MIPITX
unsigned int MTX_CLK_DIV = MAIN_CLK / ( 2 * PANEL_MIN_DDR );	// try to run at least with PANEL_MIN_DDR speed
unsigned long MIPITX_BIT_CLK = MAIN_CLK / MTX_CLK_DIV;
unsigned long MIPITX_DDR_CLK = MIPITX_BIT_CLK / 2;	// going to panel
unsigned long MIPITX_BYTE_CLK = MIPITX_BIT_CLK / 8;
unsigned ing LP_CLK_DIV = (MIPITX_BYTE_CLK + PANEL_LPCLOCK - 1) / PANEL_LPCLOCK;	// divider
unsigned int SSD_LPCLK = MIPITX_BYTE_CLK / $LP_CLK_DIV;	// real LP clock output
// LOCKCNT - at least 30us - this is the number of LPCLOCK (XTAL / 2); we add 40% safety margin
unsigned int LOCKCNT=(unsigned int) (1.4 * 30e-6 * (XTAL / 2);

// calculate OMAP parameters
unsigned int OMAP_LPCLK = XTAL / 2;	// we must drive SSD with this LP clock frequency
unsigned long OMAP_PCLK = 2 * $PIXEL_CLK;	// feed pixels in speed defined by SSD2858
unsigned int OMAP_WIDTH = PANEL_WIDTH
unsigned int OMAP_HEIGHT= PANEL_HEIGHT
unsigned int OMAP_HFP = PANEL_HFP
unsigned int OMAP_HSA = PANEL_HSA
unsigned int OMAP_HBP = PANEL_HBP
unsigned int OMAP_VFP = PANEL_VFP
unsigned int OMAP_VSA = PANEL_VSA
unsigned int OMAP_VBP = PANEL_VBP
unsigned int OMAP_DDR = MIPITX_DDR_CLK	// I hope the OMAP DSS calculates what it needs

// default flags

if (rotate) { /* swap width and height */
	OMAP_WIDTH = PANEL_HEIGHT;
	OMAP_HEIGHT = PANEL_WIDTH;
	ROT90 = 1;
}

// FIXME: convert this into printk()
// do all constraint checks afterwards so that we can loop over SYS_CLK_DIV
// break; if no constraint is violated

echo "Panel MIPI:"
echo "  Dimensions: ${PANEL_WIDTH}x${PANEL_HEIGHT} in ${PANEL_FRAME_WIDTH}x${PANEL_FRAME_HEIGHT}"
echo "  Pixel CLK: $PANEL_PCLK"
echo "  DDR CLK: $PANEL_MIN_DDR"
echo "  LPCLK target: $PANEL_LPCLOCK"

echo "SSD2858:"
echo "  XTAL CLK: $XTAL"
[ $XTAL -ge 20000000 -a $XTAL -le 300000000 ] || echo "XTAL frequency problem: $XTAL (20 - 30 MHz)"
echo "  LPCLK in: $(($XTAL / 2))"
echo "  target VTCM PIXEL CLK: $TARGET_PIXEL_CLK"
echo "  target SYS_CLK: $TARGET_SYS_CLK"
echo "  target MAIN_CLK: $TARGET_MAIN_CLK"
echo "  target PLL: $TARGET_PLL"
echo "  PLL_MULT / PLL_POST_DIV: $PLL_MULT / $PLL_POST_DIV"
[ $PLL_MULT -ge 1 -a $PLL_MULT -le 128 ] || echo "PLL_MULT problem: $PLL_MULT (1 .. 127)"
[ $PLL_POST_DIV -ge 1 -a $PLL_POST_DIV -le 64 ] || echo "PLL_POST_DIV problem: $PLL_POST_DIV (1 .. 63)"
echo "  real PLL: $PLL"
[ $PLL -ge 1000000000 -a $PLL -le 1500000000 ] || echo "PLL frequency problem: $PLL (1.000 .. 1.500 GHz)"
echo "  MAIN_CLK: $MAIN_CLK"
echo "  SYS_CLK_DIV: $SYS_CLK_DIV"
[ $SYS_CLK_DIV -ge 1 -a $SYS_CLK_DIV -le 16 ] || echo "SYS_CLK_DIV problem: $SYS_CLK_DIV (1 .. 15)"
echo "  SYS_CLK: $SYS_CLK"
[ $SYS_CLK -le 150000000 ] || echo "SYS_CLK problem: $SYS_CLK ( ... 150 MHz)"
[ $PCLK_NUM -ge 1 -a $PCLK_NUM -le 128 ] || echo "PCLK_NUM problem: $PCLK_NUM (1 .. 127)"
[ $PCLK_DEN -ge 1 -a $PCLK_DEN -le 256 ] || echo "PCLK_DEN problem: $PCLK_DEN (1 .. 255)"
echo "  PCLK_NUM / PCLK_DEN: $PCLK_NUM / $PCLK_DEN"
echo "  VTCM PIXEL_CLK: $PIXEL_CLK"
echo "  Panel PIXEL_CLK: $(( 2 * $PIXEL_CLK ))"
echo "  MTX_CLK_DIV: $MTX_CLK_DIV"
[ $MTX_CLK_DIV -ge 1 -a $MTX_CLK_DIV -le 16 ] || echo "MTX_CLK_DIV problem: $MTX_CLK_DIV (1 .. 15)"
[ $MTX_CLK_DIV -eq 1 -o $((MTX_CLK_DIV % 2)) -eq 0 ] || echo "MTX_CLK_DIV problem: $MTX_CLK_DIV (1 or even divider) - ignored"
echo "  MIPITX_BIT_CLK: $MIPITX_BIT_CLK"
echo "  MIPITX_DDR_CLK: $MIPITX_DDR_CLK"
[ $MIPITX_DDR_CLK -ge $PANEL_MIN_DDR ] || echo "MIPITX_DDR_CLK vs. PANEL_MIN_DDR problem: $MIPITX_DDR_CLK < $PANEL_MIN_DDR"
[ $MIPITX_DDR_CLK -le $PANEL_MAX_DDR ] || echo "MIPITX_DDR_CLK vs. PANEL_MAX_DDR problem: $MIPITX_DDR_CLK < $PANEL_MAX_DDR"
echo "  MIPITX_BYTE_CLK: $MIPITX_BYTE_CLK"
echo "  LPCLK out: $SSD_LPCLK"
[ $LP_CLK_DIV -gt 0 -a $LP_CLK_DIV -le 64 ] || echo "LP_CLK_DIV problem: $LP_CLK_DIV (1 .. 63)"
echo "  LOCKCNT: $LOCKCNT"
[ $LOCKCNT -gt 0 -a $LOCKCNT -le 65535 ] || echo "LOCKCNT problem: $LOCKCNT (1 .. 65535)"

echo "OMAP MIPI:"
echo "  Dimensions: ${OMAP_WIDTH}x${OMAP_HEIGHT} in ${PANEL_FRAME_WIDTH}x${PANEL_FRAME_HEIGHT}"
echo "  Pixel CLK: $OMAP_PCLK"
echo "  DDR CLK: $OMAP_DDR"
echo "  LPCLK out: $(($XTAL / 2))"

/* now initialize OMAP MIPI Interface */

// FIXME: initialize the omap interface - but we can't do a fbset here!!!

fbset -g $OMAP_WIDTH $OMAP_HEIGHT $OMAP_WIDTH $OMAP_HEIGHT 32

echo start x_res=$OMAP_WIDTH y_res=$OMAP_HEIGHT lpclock=$OMAP_LPCLK pixelclock=$OMAP_PCLK \
	hfp=$OMAP_HFP hsw=$OMAP_HSA hbp=$OMAP_HBP \
	vfp=$OMAP_VFP vsw=$OMAP_VSA vbp=$OMAP_VBP >dcs	// start MIPI interface timing

sleep .5

// FIXME: write to ssd registers

/* replace e.g.

	echo g0024 00003000 >dcs

   by

	ssd2858_write_reg(dssdev, 0x0024, 0x00003000);

*/

// prepare for getting access to the ssd
echo 28 >dcs
echo 10 >dcs
echo ff00 >dcs
echo 28 >dcs
echo 10 >dcs

// start programming the SSD2858 with the SCM

echo g0008 $(printf "%08x" $(( ($LOCKCNT << 16) | (0 << 15) | (0 << 15) | (($PLL_POST_DIV-1) << 8) | $PLL_MULT << 0)) ) >dcs
echo g000c $(printf "%08x" $(( (($MTX_CLK_DIV-1) << 4) | (($SYS_CLK_DIV-1) << 0) )) ) >dcs
echo g0014 $(printf "%08x" $(( 0x0C37800F )) ) >dcs	// SCM_MISC2 (0C77800F): MRXEN = enabled
echo g0020 $(printf "%08x" $(( 0x1592567D | (1 << 22) )) ) >dcs	// SCM_ANACTRL1 (1592567D): CPEN = enabled
echo g0024 00003000 >dcs	// SCM_ANACTRL2 (00003300): CPPER=24

// some DCS
echo 11 >dcs
sleep 0.001
echo 2a $(printf "%08x" $(( $OMAP_WIDTH - 1 )) ) >dcs
echo 2b $(printf "%08x" $(( $OMAP_HEIGHT - 1 )) ) >dcs

// MIPIRX
echo g1008 01200445 >dcs	// MIPIRX_DCR (01200245): HST=4

// VCTM
echo g200c $(printf "%08x" $(( ($SPLIT_MEM << 9) | ($ROT90 << 8) | ($VBP << 2) | ($TE_SEL << 1) )) ) >dcs	// VCTM_CFGR (00000000)
echo g2010 $(printf "%08x" $(( ($PCLK_DEN << 16) | ($PCLK_NUM << 0) )) ) >dcs	// VCTM_PCFRR (00010001)
echo g2014 $(printf "%08x" $(( ($PANEL_FRAME_WIDTH << 16) | $PANEL_HBP )) ) >dcs	// HDCFGR
echo g2018 $(printf "%08x" $(( ($PANEL_FRAME_HEIGHT << 16) | $PANEL_VBP )) ) >dcs	// VDCFGR
echo g201c $(printf "%08x" $(( ($OMAP_HEIGHT << 16) | $OMAP_WIDTH )) ) >dcs	// MSZR
echo g2020 $(printf "%08x" $(( ($PANEL_HEIGHT << 16) | $PANEL_WIDTH )) ) >dcs		// DSZR
echo g2024 $(printf "%08x" $(( ($PANEL_HEIGHT << 16) | $PANEL_WIDTH )) ) >dcs		// PSZR
echo g203c $(printf "%08x" $(( ($PANEL_HEIGHT << 16) | $PANEL_WIDTH )) ) >dcs		// ISZR
echo g2034 00000000 >dcs	// VCTM_POSR (00000000)
echo g2038 $(printf "%08x" $(( (($PANEL_HEIGHT - 1) << 16) | ($PANEL_WIDTH - 1) )) ) >dcs >dcs	// POER
echo g2030 00000015 >dcs	// URAM refresh period
echo g20a0 00000050 >dcs	// VTCM_QFBCCTRLR (00004151) - no padding, no pixswap, no fbc

// some DCS
echo 35 02 >dcs		// tear on
echo 44 0500 >dcs	// tear scan line

// MIPITX
echo g6008 $(printf "%08x" $(( 0x00000008 | (($PANEL_LANES - 1) << 22) | (($LP_CLK_DIV-1) << 16) | ($CKE << 0) )) ) >dcs	// MIPITX_CTLR (00030008)
echo g600c $(printf "%08x" $(( ($PANEL_VBP << 24) | ($PANEL_HBP << 16) | ($PANEL_VSA << 8) | ($PANEL_HSA << 0) )) ) >dcs	// MIPITX_VTC1R (0214020A)
echo g6010 $(printf "%08x" $(( ($PANEL_HEIGHT << 16) | ($PANEL_VFP << 8) | ($PANEL_HFP << 0) )) ) >dcs	// MIPITX_VTC2R (0438020A)
echo g6014 $(printf "%08x" $(( 0x01000102 | ($VB_MODE << 13) | ($VBE << 30) )) ) >dcs	// MIPITX_VCFR (01000101): VM=burst mode
echo g6084 $(printf "%08x" $(( $PANEL_WIDTH << 0 )) ) >dcs	// MIPITX_DSI0VR (00000400)
}

#endif

/* out ops available to be called by attached panel */

/* a panel driver may call these operations and we must either
 * - translate them into SSD2858 register settings or
 * - forward to in->ops i.e. the video source
 */

/* here is a sketch how DCS and generic packet commands requested by the panel driver should be handled */

/* FIXME: if we have a "hardware-bypass-available"
 * we can directly write to dcs while the ssd is in reset state
 * if the ssd is in reset and we have no bypass, report an error
 */
 
static int from_panel_gen_write(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ // panel driver wants us to send a generic packet to the panel through the SSD
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	ssd2858_pass_to_panel(dssdev, true);	// switch SSD2858 to pass-through mode
	if(buf[0] == 0xff) {
		u8 nbuf[5];
		if(len > 4)
			return -EIO;	// can't forward longer packets
		memcpy(&nbuf[1], buf, len);
		nbuf[0]=0xff;	// we need to prefix with 0xff
		return in->ops.dsi->gen_write(in, channel, nbuf, len+1);
	} else
		return in->ops.dsi->gen_write(in, channel, buf, len);
}

static int from_panel_dcs_write_nosync(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ // panel driver wants us to send a DCS packet to the panel through the SSD
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	ssd2858_pass_to_panel(dssdev, true);	// switch SSD2858 to pass-through mode
	return in->ops.dsi->dcs_write_nosync(in, channel, buf, len);
}

/* this are the ops the panel driver can call */

static struct omapdss_dsi_ops ssd2858_out_ops = {
	/* we must support at least (used in BOE driver)
	 *
	 * disable_video_output
	 * disable
	 * enable_hs
	 * connect
	 * disconnect
	 * request_vc
	 * set_vc_id
	 * release_vc
	 * bus_lock
	 * bus_unlock
	 * gen_write
	 * dcs_write_nosync
	 * set_max_rx_packet_size
	 * gen_read
	 * dcs_read
	 * set_config
	 ? configure_pins

	 */

	/*
	.connect	= from_panel_connect,
	.disconnect	= from_panel_disconnect,

	.enable		= from_panel_enable,
	.disable	= from_panel_disable,
	 */
	.dcs_write_nosync	= from_panel_dcs_write_nosync,
	.gen_write		= from_panel_gen_write,
};

/* in ops available to be called by dss video source */

static int ssd2858_power_on(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->pdev->dev;
	struct omap_dss_device *in = ddata->in;
	struct omap_dss_device *out = ddata->out;
	int r;
	struct omap_dss_dsi_config ssd2858_dsi_config = {
		.mode = OMAP_DSS_DSI_VIDEO_MODE,
		.pixel_format = SSD2858_PIXELFORMAT,
		.timings = &ddata->timings,
		.hs_clk_min = 125000000 /*SSD2858_HS_CLOCK*/,
		.hs_clk_max = 450000000 /*(12*SSD2858_HS_CLOCK)/10*/,
		.lp_clk_min = (7*SSD2858_LP_CLOCK)/10,
		.lp_clk_max = SSD2858_LP_CLOCK,
		.ddr_clk_always_on = true,
		.trans_mode = OMAP_DSS_DSI_BURST_MODE,
	};
//	printk("hs_clk_min=%lu\n", ssd2858_dsi_config.hs_clk_min);
	printk("dsi: ssd2858_power_on()\n");
	
	ssd2858_reset(dssdev, 0);	// activate reset
	
#if 0
	if (ddata->pin_config.num_pins > 0) {
		r = in->ops.dsi->configure_pins(in, &ddata->pin_config);
		if (r) {
			dev_err(&ddata->pdev->dev,
					"failed to configure DSI pins\n");
			goto err0;
		}
	}
#endif
	
	r = in->ops.dsi->set_config(in, &ssd2858_dsi_config);
	if (r) {
		dev_err(dev, "failed to configure DSI\n");
		goto err0;
	}
	
	r = in->ops.dsi->enable(in);
	if (r) {
		dev_err(dev, "failed to enable DSI\n");
		goto err0;
	}
	
	
	ssd2858_regulator(dssdev, 1);	// switch power on
	msleep(50);
	
	ssd2858_reset(dssdev, 1);	// release reset
	msleep(10);


	in->ops.dsi->enable_hs(in, ddata->pixel_channel, true);

#if 0
	r = ssd2858_write_sequence(dssdev, init_seq, ARRAY_SIZE(init_seq));
	if (r) {
		dev_err(dev, "failed to configure panel\n");
		goto err;
	}
	
	msleep(20);
	
	r = ssd2858_update_brightness(dssdev, 255);
	if (r)
		goto err;
	
	r = ssd2858_write_sequence(dssdev, sleep_out, ARRAY_SIZE(sleep_out));
	if (r)
		goto err;
	
	
	r = in->ops.dsi->enable_video_output(in, ddata->pixel_channel);
	if (r)
		goto err;
	
	msleep(120);
#endif	
#if 0	// this is recommended by the latest data sheet
	r = ssd2858_write_sequence(dssdev, display_on, ARRAY_SIZE(display_on));
	if (r)
		goto err;
#endif
	ddata->enabled = true;
	printk("dsi: powered on()\n");

	out->ops.dsi->enable(out);	// power on the panel (will call some out ops)

	return r;
err:
	printk("dsi: power on error\n");
	dev_err(dev, "error while enabling panel, issuing HW reset\n");
	
	in->ops.dsi->disable(in, false, false);
	mdelay(10);
//	ssd2858_reset(dssdev, 0);	// activate reset
	ssd2858_regulator(dssdev, 0);	// switch power off
	mdelay(20);
	
err0:
	return r;
}

// we don't have a sophisticated power management (sending the panel to power off)
// we simply stop the video stream and assert the RESET
// please note that we don't/can't switch off the VCCIO

static void ssd2858_power_off(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: ssd2858_power_off()\n");

	ddata->enabled = 0;
	in->ops.dsi->disable_video_output(in, ddata->pixel_channel);
	in->ops.dsi->disable(in, false, false);
	mdelay(10);
	ssd2858_reset(dssdev, 0);	// activate reset
	ssd2858_regulator(dssdev, 0);	// switch power off - after stopping video stream
	mdelay(20);
	/* here we can also power off IOVCC */
}

static int ssd2858_start(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r = 0;

	printk("dsi: ssd2858_start()\n");
	mutex_lock(&ddata->lock);

	in->ops.dsi->bus_lock(in);

	r = ssd2858_power_on(dssdev);

	in->ops.dsi->bus_unlock(in);

	if (r)
		dev_err(&ddata->pdev->dev, "enable failed\n");
	else
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ddata->lock);

	return r;
}

static void ssd2858_stop(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: ssd2858_stop()\n");
	mutex_lock(&ddata->lock);

	in->ops.dsi->bus_lock(in);

	ssd2858_power_off(dssdev);

	in->ops.dsi->bus_unlock(in);

	mutex_unlock(&ddata->lock);
}

static void ssd2858_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_disable()\n");
	dev_dbg(&ddata->pdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		ssd2858_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int ssd2858_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_enable()\n");
	dev_dbg(&ddata->pdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return ssd2858_start(dssdev);
}

static struct omap_dss_driver ssd2858_in_ops = {
	.connect	= ssd2858_connect,
	.disconnect	= ssd2858_disconnect,
	
	.enable		= ssd2858_enable,
	.disable	= ssd2858_disable,
	
	.get_resolution	= ssd2858_get_resolution,
	
	.check_timings	= ssd2858_check_timings,
	.set_timings	= ssd2858_set_timings,
	.get_timings	= ssd2858_get_timings,
	/*
	 * can we handle get_rotate and set_rotate?
	 */
};

static int ssd2858_probe_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *ep;
	int gpio;
	
	printk("dsi: ssd2858_probe_of()\n");
	
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
	
	ep = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(ep)) {
		dev_err(&pdev->dev, "failed to find video source (err=%ld)\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}
	
	ddata->in = ep;
	
	ep = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(ep)) {
		dev_err(&pdev->dev, "failed to find video sink (err=%ld)\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}
	
	ddata->out = ep;

	/*
	 * parse additional dt properties like
	 * rotation angle (0, 90, 180, 270 degrees)
	 * type of compression (e.g. optimal, lossy)
	 * those are to be reflected by different register settings of the SSD2858 chip
	 */

	return 0;
}

static int ssd2858_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev = NULL;
	struct panel_drv_data *ddata;
	struct device *dev = &pdev->dev;
	struct omap_dss_device *dssdev;
	int r;
	
	printk("dsi: ssd2858_probe()\n");
	dev_dbg(dev, "ssd2858_probe\n");
	
	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;
	
	platform_set_drvdata(pdev, ddata);
	ddata->pdev = pdev;
	
	if (dev_get_platdata(dev)) {
		r = -EINVAL /* could call ssd2858_probe_pdata(pdev) */;
		if (r)
			return r;
	} else if (pdev->dev.of_node) {
		r = ssd2858_probe_of(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}
	
	ddata->timings = ssd2858_timings;
	
	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &ssd2858_in_ops;
	dssdev->panel.timings = ssd2858_timings;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;
	
	dssdev->panel.dsi_pix_fmt = SSD2858_PIXELFORMAT;
	
	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(dev, "Failed to register controller\n");
		goto err_reg;
	}
	
	mutex_init(&ddata->lock);
	
	if (gpio_is_valid(ddata->reset_gpio)) {
		r = devm_gpio_request_one(&pdev->dev, ddata->reset_gpio,
								  GPIOF_DIR_OUT, "rotator reset");
		if (r) {
			dev_err(dev, "failed to request reset gpio (%d err=%d)\n", ddata->reset_gpio, r);
			return r;
		}
	}
	
	if (gpio_is_valid(ddata->regulator_gpio)) {
		r = devm_gpio_request_one(dev, ddata->regulator_gpio,
								  GPIOF_DIR_OUT, "rotator DC/DC regulator");
		if (r) {
			dev_err(dev, "failed to request regulator gpio (%d err=%d)\n", ddata->regulator_gpio, r);
			return r;
		}
	}
	
	/* Register sysfs hooks */
	r = sysfs_create_group(&dev->kobj, &ssd2858_attr_group);
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_sysfs_create;
	}
	
	printk("ssd2858_probe ok\n");
	
	return 0;
	
err_sysfs_create:
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_bl:
	//	destroy_workqueue(ddata->workqueue);
err_reg:
	return r;
}


static int __exit ssd2858_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct backlight_device *bldev;
	
	printk("dsi: ssd2858_remove()\n");
	
	omapdss_unregister_display(dssdev);
	
	ssd2858_disable(dssdev);
	ssd2858_disconnect(dssdev);
	
	sysfs_remove_group(&pdev->dev.kobj, &ssd2858_attr_group);
	
	omap_dss_put_device(ddata->in);
	
	mutex_destroy(&ddata->lock);
	
	return 0;
}

static const struct of_device_id ssd2858_of_match[] = {
	{ .compatible = "omapdss,solomon-systech,ssd2858", },
	{},
};

MODULE_DEVICE_TABLE(of, ssd2858_of_match);

static struct platform_driver ssd2858_driver = {
	.probe = ssd2858_probe,
	.remove = ssd2858_remove,
	.driver = {
		.name = "ssd2858",
		.owner = THIS_MODULE,
		.of_match_table = ssd2858_of_match,
	},
};

module_platform_driver(ssd2858_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("ssd2858 driver");
MODULE_LICENSE("GPL");
