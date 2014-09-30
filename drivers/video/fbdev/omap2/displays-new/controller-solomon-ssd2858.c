/*
 * Driver for panels through Solomon Systech SSD2858 rotator chip
 *
 * Device Tree FIXME:
 *   this chip should be defined with some "port {}" elements
 *   to keep the panel separated from the DSI interface
 *   so it is not really a "panel" driver but should be treated as
 *   an "transcoder"
 *   Device Tree could config bypass, rotation and some other paramters
 *   that are used to initialize the chip.
 *   The current implementation is a mix of SSD2858 plus Panel specific
 *   code.
 *
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

// #define IS_MCS(CMD) (((CMD) >= 0xb0 && (CMD) <= 0xff) && !((CMD) == 0xda || (CMD) == 0xdb || (CMD) == 0xdc))
#define IS_MCS(CMD) (0)

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
#if FUTURE
	struct omap_dss_device *out;	/* should be the real panel connected to the SSD */
#endif

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

static int mode=0;	// starts in non-bypass mode

static int ssd2858_pass_to_panel(struct omap_dss_device *dssdev, int enable)
{ // choose destination of further commands: SSD chip or panel
	int r = 0;
	if(mode != enable)
		{ // write a "special packet"
			u8 buf[2];
			struct panel_drv_data *ddata = to_panel_data(dssdev);
			struct omap_dss_device *in = ddata->in;
			int i;
			int len=sizeof(buf);
			buf[0]=0xff;
			buf[1]=mode=enable;
			printk("dsi: ssd2858_pass_to_panel("); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");
			r = in->ops.dsi->gen_write(in, ddata->config_channel, buf, len);
			if (r)
				dev_err(&ddata->pdev->dev, "write cmd/reg(%x %x) failed: %d\n",
						buf[0], buf[1], r);
		}
	return r;
}

static int ssd2858_write(struct omap_dss_device *dssdev, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	printk("dsi: ssd2858_write("); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");

	if(mode == 1 && len > 0 && buf[0] == 0xff)
		{
		/*
		 * FIXME: if a generic packet goes to the panel
		 * and starts with 0xff we have to send a 0xff,0xff,...
		 * packet instead (without switching)
		 */
		return -EINVAL;
		}
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
#if 1
	r = in->ops.dsi->gen_write(in, ddata->config_channel, buf, 6);
#else
	r = 0;
#endif
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

static int ssd2858_write_reg_sequence(struct omap_dss_device *dssdev,
									  void *seq, int len)
{
	// tbd.
	return -EINVAL;
}

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
}

static int ssd2858_reset(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_reset(%d)\n", state);
	if (gpio_is_valid(ddata->reset_gpio))
		gpio_set_value(ddata->reset_gpio, state);
	return 0;
}

static int ssd2858_regulator(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: ssd2858_regulator(%d)\n", state);
	if (gpio_is_valid(ddata->regulator_gpio))
		gpio_set_value(ddata->regulator_gpio, state);	// switch regulator
	return 0;
}

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

static ssize_t set_dcs(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int r = 0;
	u8 data[24];
	u8 d = 0;
	int topanel=0;
	int argc = 0;
	int second = 0;
	int read = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;
	const char *p;

	if(strncmp(buf, "start", 5) == 0)
		{
		int r = ssd2858_start(dssdev);
		return r < 0 ? r : count;
		}
	if(strncmp(buf, "stop", 4) == 0)
		{
		ssd2858_stop(dssdev);
		return count;
		}
	if(strncmp(buf, "stream", 6) == 0)
		{
		in->ops.dsi->bus_lock(in);
		in->ops.dsi->enable_video_output(in, ddata->pixel_channel);
		in->ops.dsi->bus_unlock(in);
		return count;
		}
	if(strncmp(buf, "nostream", 8) == 0)
		{
		in->ops.dsi->bus_lock(in);
		in->ops.dsi->disable_video_output(in, ddata->pixel_channel);
		in->ops.dsi->bus_unlock(in);
		return count;
		}
	if(strncmp(buf, "reset", 5) == 0)
		{
		ssd2858_reset(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "noreset", 7) == 0)
		{
		ssd2858_reset(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "power", 5) == 0)
		{
		ssd2858_regulator(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "nopower", 7) == 0)
		{
		ssd2858_regulator(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "status", 6) == 0) {
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			in->ops.dsi->bus_lock(in);
			ssd2858_pass_to_panel(dssdev, 0);	// send to ssd2858
//			r = ssd2858_read(dssdev, 0x0a, data, 1);	// power mode 0x10=sleep off; 0x04=display on
			r = ssd2858_read(dssdev, 0x0b, data, 1);	// address mode
			r = ssd2858_read(dssdev, MIPI_DCS_GET_PIXEL_FORMAT, data, 1);	// pixel format 0x70 = RGB888
//			r = ssd2858_read(dssdev, 0x0d, data, 1);	// display mode	0x80 = command 0x34/0x35
//			r = ssd2858_read(dssdev, 0x0e, data, 1);	// signal mode
//			r = ssd2858_read(dssdev, MIPI_DCS_GET_DIAGNOSTIC_RESULT, data, 1);	// diagnostic 0x40 = functional
			r = ssd2858_read(dssdev, 0x45, data, 2);	// get scanline
			in->ops.dsi->bus_unlock(in);
		}
		mutex_unlock(&ddata->lock);
		return r < 0 ? r : count;
	}
	if(strncmp(buf, "pstatus", 6) == 0) {
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			in->ops.dsi->bus_lock(in);
			ssd2858_pass_to_panel(dssdev, 1);	// send to panel
			//			r = ssd2858_read(dssdev, 0xbf, data, 5);	// SSD2858 chip ID
			//			r = ssd2858_read(dssdev, 0xb0, data, 1);	// MCS access protection
			//			r = ssd2858_read(dssdev, 0xb5, data, 3);	// checksum and ECC errors
			//			r = ssd2858_read(dssdev, 0x04, data, 16);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			//			r = ssd2858_read(dssdev, DCS_READ_NUM_ERRORS, data, 1);	// dsi errors
			//		r = ssd2858_read(dssdev, 0x06, data, 1);	// red
			//		r = ssd2858_read(dssdev, 0x07, data, 1);	// green
			//		r = ssd2858_read(dssdev, 0x08, data, 1);	// blue
			//			r = ssd2858_read(dssdev, 0x0a, data, 1);	// power mode 0x10=sleep off; 0x04=display on
			r = ssd2858_read(dssdev, 0x0b, data, 1);	// address mode
			r = ssd2858_read(dssdev, MIPI_DCS_GET_PIXEL_FORMAT, data, 1);	// pixel format 0x70 = RGB888
			//			r = ssd2858_read(dssdev, 0x0d, data, 1);	// display mode	0x80 = command 0x34/0x35
			//			r = ssd2858_read(dssdev, 0x0e, data, 1);	// signal mode
			//			r = ssd2858_read(dssdev, MIPI_DCS_GET_DIAGNOSTIC_RESULT, data, 1);	// diagnostic 0x40 = functional
			r = ssd2858_read(dssdev, 0x45, data, 2);	// get scanline
			//			r = ssd2858_read(dssdev, 0x52, data, 2);	// brightness
			//			r = ssd2858_read(dssdev, 0x56, data, 1);	// adaptive brightness
			//			r = ssd2858_read(dssdev, 0x5f, data, 2);	// CABC minimum
			//			r = ssd2858_read(dssdev, 0x68, data, 1);	// auto brightness
			//			r = ssd2858_read(dssdev, 0xa1, data, 16);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			//			r = ssd2858_read(dssdev, 0xda, data, 1);	// ID1
			//			r = ssd2858_read(dssdev, 0xdb, data, 1);	// ID2
			//			r = ssd2858_read(dssdev, 0xdc, data, 1);	// ID3
			in->ops.dsi->bus_unlock(in);
		}
		mutex_unlock(&ddata->lock);
		return r < 0 ? r : count;
	}
	if(strncmp(buf, "test", 4) == 0)
		{
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			in->ops.dsi->bus_lock(in);
			r = ssd2858_write_sequence(dssdev, test_image, ARRAY_SIZE(test_image));
			in->ops.dsi->bus_unlock(in);
		}
		mutex_unlock(&ddata->lock);
		return r < 0 ? r : count;
		}

	for(p = buf; p < buf + count && argc < sizeof(data); p++)
		{

//		printk("  2nd:%d argc:%d read:%d %c\n", second, argc, read, *p);

		if(!second && (*p == ' ' || *p == '\t' || *p == '\n'))
			continue;
		if(argc == 0 && !second && *p == 'p')
		   { // 'panel' prefix for address
		   topanel=1;
		   continue;
		   }
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
		in->ops.dsi->bus_lock(in);

		ssd2858_pass_to_panel(dssdev, topanel);	// send to ssd or panel
		if(read)
			r = ssd2858_read(dssdev, data[0], &data[1], argc-1);
		else
			r = ssd2858_write(dssdev, data, argc);

		in->ops.dsi->bus_unlock(in);
	} else
		r=-EIO;	// not enabled
	mutex_unlock(&ddata->lock);

	return r < 0 ? r : count;
}

static ssize_t show_dcs(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "[p]aa dd dd ... | [p]aa r ... | test | [p]status | start | stop | reset | noreset | power | nopower | stream | nostream\n");
}

static DEVICE_ATTR(dcs, S_IWUSR | S_IRUGO,
				   show_dcs, set_dcs);

static ssize_t set_reg(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int r = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;
	const char *p;
	unsigned short address=0;	// 16 bit
	unsigned long data=0;		// 32 bit
	int ad=0;
	int read=0;

	for(p = buf; p < buf + count; p++)
		{
		int d;
		if((*p == ' ' || *p == '\t' || *p == '\n'))
			{
			ad=1;
			continue;	// skip whitespace and switch to data
			}
		if(*p == 'r')	// r must follow the address
			{
			read = 1;
			continue;
			}
		if(read)
			return -EIO;	// no more digits after the first r
		if(*p >= '0' && *p <= '9')
			d=(*p-'0');
		else if(*p >= 'a' && *p <= 'f')
			d=(*p-'a') + 10;
		else if(*p >= 'A' && *p <= 'F')
			d=(*p-'A') + 10;
		else
			return -EIO;
		if(p == buf + 4)
			ad=1;	// switch to data
		if(ad)
			data=(data<<4) + d;
		else
			address=(address<<4) + d;
		}

	mutex_lock(&ddata->lock);
	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);

		ssd2858_pass_to_panel(dssdev, 0);	// send to SSD2858 directly
		if(read)
			r = ssd2858_read_reg(dssdev, address, &data);
		else
			r = ssd2858_write_reg(dssdev, address, data);

		in->ops.dsi->bus_unlock(in);
	} else
		r=-EIO;	// not enabled
	mutex_unlock(&ddata->lock);

	return r < 0 ? r : count;
}

static ssize_t show_reg(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "aaaa dddddddddddd | aaaa r\n");
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
				   show_reg, set_reg);

static struct attribute *ssd2858_attributes[] = {
	&dev_attr_dcs.attr,
	&dev_attr_reg.attr,
	NULL
};

static const struct attribute_group ssd2858_attr_group = {
	.attrs = ssd2858_attributes,
};


static int ssd2858_power_on(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->pdev->dev;
	struct omap_dss_device *in = ddata->in;
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
	printk("hs_clk_min=%lu\n", ssd2858_dsi_config.hs_clk_min);
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
	
#if 0	// this is recommended by the latest data sheet
	r = ssd2858_write_sequence(dssdev, display_on, ARRAY_SIZE(display_on));
	if (r)
		goto err;
#endif
	ddata->enabled = true;
	printk("dsi: powered on()\n");
	
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

static struct omap_dss_driver ssd2858_ops = {
	.connect	= ssd2858_connect,
	.disconnect	= ssd2858_disconnect,
	
	.enable		= ssd2858_enable,
	.disable	= ssd2858_disable,
	
	.get_resolution	= ssd2858_get_resolution,
	
	.check_timings	= ssd2858_check_timings,
	.set_timings	= ssd2858_set_timings,
	.get_timings	= ssd2858_get_timings,
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
	
#if FUTURE
	ep = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(ep)) {
		dev_err(&pdev->dev, "failed to find video sink (err=%ld)\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}
	
	ddata->out = ep;
#endif

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
		r = -EINVAL /*ssd2858_probe_pdata(pdev)*/;
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
	dssdev->driver = &ssd2858_ops;
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
	/* these are fake until we can control a panel on the out-endpoint */
	{ .compatible = "omapdss,success,s90451-di050hd", },
	{ .compatible = "omapdss,boe,btl507212-w677l", },
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
