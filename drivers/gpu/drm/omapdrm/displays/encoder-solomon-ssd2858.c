/*
 * Driver for panels through Solomon Systech SSD2858 rotator chip
 *
 * Device Tree:
 *   this chip is defined with a pair "port {}" elements
 *   to keep the panel separated from the DSI interface
 *   so it is an "encoder" with an input and an output port
 *   Device Tree can config bypass, rotation and some other paramters
 *   that are used to initialize the chip.
 *   The current implementation is still a mix of SSD2858 plus Panel specific
 *   code and needs a lot of work to be finialised.
 *
 * External Bypass:
 *   the ssd2858 can send DCS(MCS) commands to the panel through a special
 *   mode. There is one exception that DCS(MCS) commands starting with 0xff
 *   need to be escaped and are limited in length. If the panel happens
 *   to need a command that is too long (like the boe-wl677) we need an
 *   external switch that bypasses the lane1 of the ssd2858 while the ssd2858
 *   is in reset state. This is assumed to exist and controlled by the same
 *   gpio as the ssd2858 reset.
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

#define BACKLIGHT 1
#define REGULATOR 0
#define SYSFS 1
#define LOG 1
/* can be removed later */
#define EDIT_HINT 0
#define OLD 0

#if BACKLIGHT
#include <linux/backlight.h>
#endif
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
#define DCS_READ_CTRL_DISPLAY	0x53	// read control
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
#define MCS_READID1		0xda
#define MCS_READID2		0xdb
#define MCS_READID3		0xdc

#define MIPI_DCS_SET_ADDRESS_MODE_HFLIP 0x40
#define MIPI_DCS_SET_ADDRESS_MODE_VFLIP 0x80

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

#define SSD2858_PIXELFORMAT		OMAP_DSS_DSI_FMT_RGB888	// 16.7M color = RGB888

static struct videomode ssd2858_timings = {
	// all values will be calculated! */
};

struct panel_drv_data {
	struct omap_dss_device dssdev;	/* the output port (where a panel connects) */
	struct omap_dss_device *in;	/* the input port (OMAP to SSD) */

	struct videomode vm;

	struct platform_device *pdev;

	struct mutex lock;

#if BACKLIGHT
	struct backlight_device *bldev;
#endif
	int bl;

	int	reset_gpio;
#if REGULATOR
	int	regulator_gpio;
#endif

	struct omap_video_timings panel_timings;
	struct omap_dss_dsi_config panel_dsi_config;

	bool reset;	/* ssd is in reset state */
	bool bypass;	/* in bypass more */
	bool enabled;	/* video is enabled */

	int xtal;	/* xtal clock frequency */
	bool rotate;	/* rotate by 90 degrees */
	int flip;	/* flip code */

	int config_channel;
	int pixel_channel;
};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

struct ssd2858_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

#if OLD
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

#endif

/* communication with the SSD chip */

static int ssd2858_write(struct omap_dss_device *dssdev, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

#if LOG
	printk("dsi: ssd2858_write(%s", IS_MCS(buf[0])?"g":""); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");
#endif

	if (IS_MCS(buf[0]))
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

static inline int ssd2858_write_cmd4(struct omap_dss_device *dssdev,
			     u8 dcs,
			     u8 p1, u8 p2, u8 p3, u8 p4)
{ /* send command with 4 parameter bytes */
	u8 buf[5];
	buf[0] = dcs;
	buf[1] = p1;
	buf[2] = p2;
	buf[3] = p3;
	buf[4] = p4;
	return ssd2858_write(dssdev, buf, sizeof(buf));
}

static inline int ssd2858_write_cmd3(struct omap_dss_device *dssdev,
			     u8 dcs,
			     u8 p1, u8 p2, u8 p3)
{ /* send command with 3 parameter bytes */
	u8 buf[3];
	buf[0] = dcs;
	buf[1] = p1;
	buf[2] = p2;
	buf[3] = p3;
	return ssd2858_write(dssdev, buf, sizeof(buf));
}

static inline int ssd2858_write_cmd2(struct omap_dss_device *dssdev,
			     u8 dcs,
			     u8 p1, u8 p2)
{ /* send command with 2 parameter bytes */
	u8 buf[3];
	buf[0] = dcs;
	buf[1] = p1;
	buf[2] = p2;
	return ssd2858_write(dssdev, buf, sizeof(buf));
}

static inline int ssd2858_write_cmd1(struct omap_dss_device *dssdev,
			     u8 dcs,
			     u8 p1)
{ /* send command with 1 parameter byte */
	u8 buf[2];
	buf[0] = dcs;
	buf[1] = p1;
	return ssd2858_write(dssdev, buf, sizeof(buf));
}

static inline int ssd2858_write_cmd0(struct omap_dss_device *dssdev,
			     u8 dcs)
{ /* send command with no parameter byte */
	return ssd2858_write(dssdev, &dcs, 1);
}

static int ssd2858_pass_to_panel(struct omap_dss_device *dssdev, bool enable)
{ // choose destination of further commands: SSD chip or panel
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r = 0;

	if (ddata->reset) { /* assume hardware bypass is on */
		if (!enable)
			dev_err(&ddata->pdev->dev, "can't send commands to ssd while in reset\n");
	} else if (ddata->bypass != enable)
		{ /* write a "special control packet" prefix to switch pass through modes */
			ddata->bypass = enable;
			ssd2858_write_cmd1(dssdev, 0xff, enable);
		}
	return r;
}

static int ssd2858_read(struct omap_dss_device *dssdev, u8 dcs_cmd, u8 *buf, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

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
#if LOG
	printk("dsi: ssd2858_read(%02x,", dcs_cmd); for(i=0; i<len; i++) printk(" %02x", buf[i]);
	printk(") -> %d\n", r);
#endif
	return r;
}

#if OLD
static int ssd2858_write_sequence(struct omap_dss_device *dssdev,
		struct ssd2858_reg *seq, int len)
{
	int r, i;
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

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
#endif

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

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

	r = in->ops.dsi->gen_write(in, ddata->config_channel, buf, 6);

#if LOG
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
	unsigned long val;

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

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
	val  = (buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | (buf[5] << 0);
#if LOG
	printk("ssd2858_read_reg: %04x -> %08lx (r=%d)\n", address, val, r);
#endif
	if (data)
		*data = val;
	return r;
}

#if OLD
static int ssd2858_write_reg_sequence(struct omap_dss_device *dssdev, void *seq, int len)
{
	// tbd.
	return -EINVAL;
}
#endif

/* callbacks from the DSI driver */

static int ssd2858_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;

	printk("ssd2858_connect\n");
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

	printk("ssd2858_disconnect\n");
	if (!omapdss_device_is_connected(dssdev))
		return;
	// FIXME: forward disconnect to panel

	in->ops.dsi->release_vc(in, ddata->pixel_channel);
	in->ops.dsi->release_vc(in, ddata->config_channel);
	in->ops.dsi->disconnect(in, dssdev);
}

static void ssd2858_get_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("ssd2858_get_timings\n");
	*timings = dssdev->panel.vm;
}

static void ssd2858_set_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("ssd2858_set_timings\n");
	dssdev->panel.vm.hactive = timings->hactive;
	dssdev->panel.vm.vactive = timings->vactive;
	dssdev->panel.vm.pixelclock = timings->pixelclock;
	dssdev->panel.vm.hsync_len = timings->hsync_len;
	dssdev->panel.vm.hfront_porch = timings->hfront_porch;
	dssdev->panel.vm.hback_porch = timings->hback_porch;
	dssdev->panel.vm.vsync_len = timings->vsync_len;
	dssdev->panel.vm.vfront_porch = timings->vfront_porch;
	dssdev->panel.vm.vback_porch = timings->vback_porch;
	// FIXME: forward to panel
}

static int ssd2858_check_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("ssd2858_check_timings\n");
	return 0;
}

static void ssd2858_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	printk("ssd2858_get_resolution\n");
	*xres = dssdev->panel.vm.hactive;
	*yres = dssdev->panel.vm.vactive;
	// FIXME: forward to panel?
}

/* hardware control */

static int ssd2858_reset(struct omap_dss_device *dssdev, bool state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

#if LOG
	printk("dsi: ssd2858_reset(%d)\n", state);
#endif

	if (gpio_is_valid(ddata->reset_gpio))
		gpio_set_value(ddata->reset_gpio, !state);	/* assume active low */
	ddata->reset = !state;

	// forward to panel driver?

	return 0;
}

#if REGULATOR
static int ssd2858_regulator(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

#if LOG
	printk("dsi: ssd2858_regulator(%d)\n", state);
#endif

	if (gpio_is_valid(ddata->regulator_gpio))
		gpio_set_value(ddata->regulator_gpio, state);	// switch regulator

	return 0;
}
#endif

#if BACKLIGHT
/* backlight control */

static int ssd2858_update_brightness(struct omap_dss_device *dssdev, int level)
{
#if 1
	return ssd2858_write_cmd1(dssdev, DCS_BRIGHTNESS, level);
#else
	return ssd2858_write_cmd2(dssdev, DCS_BRIGHTNESS, level>>4, (level&0xf) << 4);
#endif
}

static int ssd2858_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
//	struct omap_dss_device *in = ddata->in;
	int bl = bd->props.brightness;
	int r = 0;
#if LOG
	printk("dsi: ssd2858_set_brightness(%d)\n", bl);
#endif
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
#if LOG
	printk("dsi: ssd2858_get_brightness()\n");
#endif
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		dev_err(&ddata->pdev->dev, "dsi: display is not active\n");
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
		dev_err(&ddata->pdev->dev, "dsi: read error\n");
		return bd->props.brightness;
	}

#if LOG
	printk("dsi: read %d\n", brightness);
#endif
	return brightness>>4;	// get to range 0..255
}

static const struct backlight_ops ssd2858_backlight_ops  = {
	.get_brightness = ssd2858_get_brightness,
	.update_status = ssd2858_set_brightness,
};
#endif

#if SYSFS

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

#endif

/* out ops available to be called by attached panel */

/* a panel driver may call these operations and we must either
 * - translate them into SSD2858 register settings or
 * - forward to in->ops i.e. the video source
 */

/* here is a sketch how DCS and generic packet commands requested by the panel driver should be handled */

static int from_panel_gen_write(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ /* panel driver wants us to send a generic packet to the panel through the SSD/bypass */
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("from_panel_gen_write\n");
#endif

	ssd2858_pass_to_panel(dssdev, true);	// switch SSD2858 to pass-through mode

	if (!ddata->reset && buf[0] == 0xff) {
		u8 nbuf[5];
		if(len > 4) {
			dev_err(&ddata->pdev->dev, "packet too long for forwarding mechanism: %d\n", len);
			return -EIO;
		}
		memcpy(&nbuf[1], buf, len);
		nbuf[0]=0xff;	// we need to prefix with 0xff
		return in->ops.dsi->gen_write(in, channel, nbuf, len+1);
	} else
		return in->ops.dsi->gen_write(in, channel, buf, len);
}

static int from_panel_dcs_write_nosync(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ /* panel driver wants us to send a DCS packet to the panel through the SSD/bypass */
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
#if LOG
	printk("from_panel_dcs_write_nosync\n");
#endif
	ssd2858_pass_to_panel(dssdev, true);	// switch SSD2858 to pass-through mode

	return in->ops.dsi->dcs_write_nosync(in, channel, buf, len);
}

/* this are the out ops the panel driver can call */
/* FIXME: make them known to the panel driver */

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
	int r;
	struct omap_dss_dsi_config ssd2858_dsi_config = {
		.mode = OMAP_DSS_DSI_VIDEO_MODE,
		.pixel_format = SSD2858_PIXELFORMAT,
		.vm = &ddata->vm,
		.ddr_clk_always_on = true,
		.trans_mode = OMAP_DSS_DSI_BURST_MODE,
		// others will be calculated
	};
#if LOG
	printk("dsi: ssd2858_power_on()\n");
#endif
	ssd2858_reset(dssdev, true);	// activate reset

	; /* should we enable the panel here? */

	/* initialize panel timing parameters */

#if 1
	// initial Panel parameters (should be provided by the panel driver!)
	ddata->panel_timings.x_res = 720;
	ddata->panel_timings.y_res = 1280;
	u16 PANEL_FPS = 60;	// frames per second
	u8 PANEL_BPP = 24;	// bits per lane
	u8 PANEL_LANES = 4;	// lanes
	ddata->panel_timings.hfp = 10;	// front porch
	ddata->panel_timings.hsw = 10;	// sync active
	ddata->panel_timings.hbp = 100;	// back porch
	// NOTE: we must set this so the sum of V* is < ~70 to get a slightly higher pixel and DDR rate or the panel wouldn't sync properly
	// warning: some VBP values are causing horizontal misalignemt:
	// 12..15, 18..23, 26..?, 34..40, ...?
	ddata->panel_timings.hfp = 10;	// top porch
	ddata->panel_timings.hsw = 2;	// sync active
	ddata->panel_timings.hbp = 48;	// bottom porch
	ddata->panel_dsi_config.lp_clk_max = 8000000;	// maximum is 9.2 MHz
	u32 PANEL_MAX_DDR=250000000;	// maximum DDR clock (4..25ns) that can be processed by controller
#endif

	/* calculate timings */

	/* derived panel parameters */
	u16 PANEL_FRAME_WIDTH = ddata->panel_timings.x_res + ddata->panel_timings.hfp + ddata->panel_timings.hsw + ddata->panel_timings.hbp;	// some margin for sync and retrace
	u16 PANEL_FRAME_HEIGHT = ddata->panel_timings.y_res + ddata->panel_timings.vfp + ddata->panel_timings.vsw + ddata->panel_timings.vbp;	// some margin for sync and retrace
	u32 PANEL_PCLK = PANEL_FRAME_WIDTH * PANEL_FRAME_HEIGHT * PANEL_FPS;	// required pixel clock
	u32 PANEL_MIN_DDR = (PANEL_PCLK / 2 / PANEL_LANES) * PANEL_BPP;	// min is defined by required data bandwidth

	/* SSD2858 mode parameters */

	bool SPLIT_MEM = 1, TE_SEL = 1, VBP = 0;
	bool CKE = 0;
	bool VB_MODE = 0;
	bool VBE = 0;

	/* SSD2858 clock divider parameters */

	u32 XTAL = ddata->xtal;

	bool PCLK_NUM = 1, PCLK_DEN = 4; // must be 1:4 for proper video processing

	u8 SYS_CLK_DIV; // this is something to play with until all parameters fit the constraints
	u16 LOCKCNT;
	u16 PLL_POST_DIV;
	u16 PLL_MULT;
	u16 MTX_CLK_DIV;
	u16 LP_CLK_DIV;

	for(SYS_CLK_DIV = 5; SYS_CLK_DIV <= 5; SYS_CLK_DIV++) {

		/* SSD divider calculations */

		/* 1. VCTM */
		u32 TARGET_PIXEL_CLK = PANEL_PCLK / 2;
		u32 TARGET_SYS_CLK = PCLK_DEN * TARGET_PIXEL_CLK / PCLK_NUM;
		u32 TARGET_MAIN_CLK = 2 * SYS_CLK_DIV * TARGET_SYS_CLK;
		u16 TARGET_DIV = 1500000000 / TARGET_MAIN_CLK;	// total required divisor between PLL and MAIN_CLK - rounded down (running the PLL at least at required frequency)
		u32 TARGET_PLL = TARGET_DIV * TARGET_MAIN_CLK;	// what we expect to see as PLL frequency
		/* 2. PLL */
		PLL_MULT = (TARGET_PLL + XTAL - 1) / XTAL;	// required PLL multiplier from XTAL (rounded up)
		u32 PLL = XTAL * PLL_MULT;	// real PLL frequency
		PLL_POST_DIV = PLL / TARGET_MAIN_CLK;	// PLL_POST_DIV to get MAIN_CLOCK - should be >= TARGET_DIV
		u32 MAIN_CLK = PLL / PLL_POST_DIV;	// real MAIN clock
		u32 SYS_CLK = MAIN_CLK / 2 / SYS_CLK_DIV;	// real SYS clock
		u32 PIXEL_CLK = (SYS_CLK * PCLK_NUM) / PCLK_DEN;	// real VTCM pixel clock
		/* 3. MIPITX */
		MTX_CLK_DIV = MAIN_CLK / ( 2 * PANEL_MIN_DDR );	// try to run at least with PANEL_MIN_DDR speed
		u32 MIPITX_BIT_CLK = MAIN_CLK / MTX_CLK_DIV;
		u32 MIPITX_DDR_CLK = MIPITX_BIT_CLK / 2;	// going to panel
		u32 MIPITX_BYTE_CLK = MIPITX_BIT_CLK / 8;
		LP_CLK_DIV = (MIPITX_BYTE_CLK + ddata->panel_dsi_config.lp_clk_max - 1) / ddata->panel_dsi_config.lp_clk_max;	// divider
		u32 SSD_LPCLK = MIPITX_BYTE_CLK / LP_CLK_DIV;	// real LP clock output
		/* LOCKCNT - at least 30us - this is the number of LPCLOCK (XTAL / 2); we add 40% safety margin */
		LOCKCNT=((((XTAL / 2 ) * 30) / 1000000) * 140) / 100;

		/* calculate OMAP parameters */

		ssd2858_dsi_config.lp_clk_min = ssd2858_dsi_config.lp_clk_max = XTAL / 2;	// we must drive SSD with this LP clock frequency
		u32 OMAP_PCLK = 2 * PIXEL_CLK;	// feed pixels in speed defined by SSD2858
		ssd2858_timings = ddata->panel_timings;
		if (ddata->rotate) {
			ssd2858_timings.x_res = ddata->panel_timings.y_res;
			ssd2858_timings.y_res = ddata->panel_timings.x_res;
		}

		ssd2858_dsi_config.hs_clk_min = ssd2858_dsi_config.hs_clk_max = MIPITX_DDR_CLK;	// I hope the OMAP DSS calculates what it needs

#if LOG
		printk("Panel MIPI:\n");
		printk("  Dimensions: {%dx%d} in {%dx%d}\n", ddata->panel_timings.x_res, ddata->panel_timings.y_res, PANEL_FRAME_WIDTH, PANEL_FRAME_HEIGHT);
		printk("  Pixel CLK: %d\n", PANEL_PCLK);
		printk("  DDR CLK: %d\n", PANEL_MIN_DDR);
		printk("  LPCLK target: %ld..%ld\n", ddata->panel_dsi_config.lp_clk_max, ddata->panel_dsi_config.lp_clk_max);

		printk("SSD2858:\n");
		printk("  XTAL CLK: %d\n", XTAL);
		printk("  LPCLK in: %d\n", XTAL / 2);
		printk("  target VTCM PIXEL CLK: %d\n", TARGET_PIXEL_CLK);
		printk("  target SYS_CLK: %d\n", TARGET_SYS_CLK);
		printk("  target MAIN_CLK: %d\n", TARGET_MAIN_CLK);
		printk("  target PLL: %d\n", TARGET_PLL);
		printk("  PLL_MULT / PLL_POST_DIV: %d / %d\n", PLL_MULT, PLL_POST_DIV);
		printk("  real PLL: %d\n", PLL);
		printk("  MAIN_CLK: %d\n", MAIN_CLK);
		printk("  SYS_CLK_DIV: %d\n", SYS_CLK_DIV);
		printk("  SYS_CLK: %d\n", SYS_CLK);
		printk("  PCLK_NUM / PCLK_DEN: %d / %d\n", PCLK_NUM , PCLK_DEN);
		printk("  VTCM PIXEL_CLK: %d\n", PIXEL_CLK);
		printk("  Panel PIXEL_CLK: %d\n", 2 * PIXEL_CLK);
		printk("  MTX_CLK_DIV: %d\n", MTX_CLK_DIV);
		printk("  MIPITX_BIT_CLK: %d\n", MIPITX_BIT_CLK);
		printk("  MIPITX_DDR_CLK: %d\n", MIPITX_DDR_CLK);
		printk("  MIPITX_BYTE_CLK: %d\n", MIPITX_BYTE_CLK);
		printk("  LPCLK out: %d\n", SSD_LPCLK);
		printk("  LOCKCNT: %d\n", LOCKCNT);

		printk("OMAP MIPI:\n");
		printk("  Dimensions: %dx%d} in {%dx%d}\n", ssd2858_timings.x_res, ssd2858_timings.y_res, PANEL_FRAME_WIDTH, PANEL_FRAME_HEIGHT);
		printk("  Pixel CLK: %d\n", OMAP_PCLK);
		printk("  DDR CLK: %ld..%ld\n", ssd2858_dsi_config.hs_clk_min, ssd2858_dsi_config.hs_clk_max);
		printk("  LPCLK out: %ld..%ld\n", ssd2858_dsi_config.lp_clk_min, ssd2858_dsi_config.lp_clk_max);

#endif

#if 0
	/* check parameters */

// these are independent of the SYS_CLK_DIV

[ $XTAL -ge 20000000 -a $XTAL -le 300000000 ] || echo "XTAL frequency problem: $XTAL (20 - 30 MHz)"

	// we could simply print optional warning message and continue;

[ $PLL_MULT -ge 1 -a $PLL_MULT -le 128 ] || echo "PLL_MULT problem: $PLL_MULT (1 .. 127)"
[ $PLL_POST_DIV -ge 1 -a $PLL_POST_DIV -le 64 ] || echo "PLL_POST_DIV problem: $PLL_POST_DIV (1 .. 63)"
[ $PLL -ge 1000000000 -a $PLL -le 1500000000 ] || echo "PLL frequency problem: $PLL (1.000 .. 1.500 GHz)"
[ $SYS_CLK_DIV -ge 1 -a $SYS_CLK_DIV -le 16 ] || echo "SYS_CLK_DIV problem: $SYS_CLK_DIV (1 .. 15)"
[ $SYS_CLK -le 150000000 ] || echo "SYS_CLK problem: $SYS_CLK ( ... 150 MHz)"
[ $PCLK_NUM -ge 1 -a $PCLK_NUM -le 128 ] || echo "PCLK_NUM problem: $PCLK_NUM (1 .. 127)"
[ $PCLK_DEN -ge 1 -a $PCLK_DEN -le 256 ] || echo "PCLK_DEN problem: $PCLK_DEN (1 .. 255)"
[ $MTX_CLK_DIV -ge 1 -a $MTX_CLK_DIV -le 16 ] || echo "MTX_CLK_DIV problem: $MTX_CLK_DIV (1 .. 15)"
[ $MTX_CLK_DIV -eq 1 -o $((MTX_CLK_DIV % 2)) -eq 0 ] || echo "MTX_CLK_DIV problem: $MTX_CLK_DIV (1 or even divider) - ignored"
[ $MIPITX_DDR_CLK -ge $PANEL_MIN_DDR ] || echo "MIPITX_DDR_CLK vs. PANEL_MIN_DDR problem: $MIPITX_DDR_CLK < $PANEL_MIN_DDR"
[ $MIPITX_DDR_CLK -le $PANEL_MAX_DDR ] || echo "MIPITX_DDR_CLK vs. PANEL_MAX_DDR problem: $MIPITX_DDR_CLK < $PANEL_MAX_DDR"
[ $LP_CLK_DIV -gt 0 -a $LP_CLK_DIV -le 64 ] || echo "LP_CLK_DIV problem: $LP_CLK_DIV (1 .. 63)"
[ $LOCKCNT -gt 0 -a $LOCKCNT -le 65535 ] || echo "LOCKCNT problem: $LOCKCNT (1 .. 65535)"
#endif

	}

	// if no sufficiently matching SYS_CLK_DIV found => return -EINVAL

	/* now initialize OMAP MIPI Interface */

#if EDIT_HINT
// FIXME: initialize the omap interface - but we can't do a fbset here!!!

	fbset -g $OMAP_WIDTH $OMAP_HEIGHT $OMAP_WIDTH $OMAP_HEIGHT 32

	echo start x_res=$OMAP_WIDTH y_res=$OMAP_HEIGHT lpclock=$OMAP_LPCLK pixelclock=$OMAP_PCLK \
		hfp=$OMAP_HFP hsw=$OMAP_HSA hbp=$OMAP_HBP \
		vfp=$OMAP_VFP vsw=$OMAP_VSA vbp=$OMAP_VBP >dcs	// start MIPI interface timing


#endif

	msleep(500);

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

#if REGULATOR
	ssd2858_regulator(dssdev, 1);	// switch power on
	msleep(50);
#endif

	in->ops.dsi->enable_hs(in, ddata->pixel_channel, true);

	/* send setup */

	ssd2858_pass_to_panel(dssdev, true);	/* communicate with the panel */

	// may read some registers / status

	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_OFF);
	ssd2858_write_cmd0(dssdev, MIPI_DCS_ENTER_SLEEP_MODE);

	ssd2858_reset(dssdev, false);	// release reset
	msleep(10);

	/* setup the panel here */

	/* prepare for getting access to the ssd */

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */
	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_OFF);
	ssd2858_write_cmd0(dssdev, MIPI_DCS_ENTER_SLEEP_MODE);

#if LOG
	ssd2858_read_reg(dssdev, 0x0008, NULL);
	ssd2858_read_reg(dssdev, 0x000c, NULL);
	ssd2858_read_reg(dssdev, 0x0014, NULL);
	ssd2858_read_reg(dssdev, 0x0020, NULL);
	ssd2858_read_reg(dssdev, 0x0024, NULL);
#endif

	/* start with programming the SCM of the SSD2858 */

	ssd2858_write_reg(dssdev, 0x0008, (LOCKCNT << 16) | (0 << 15) | (0 << 15) | ((PLL_POST_DIV-1) << 8) | PLL_MULT << 0);
	ssd2858_write_reg(dssdev, 0x000c, ((MTX_CLK_DIV-1) << 4) | ((SYS_CLK_DIV-1) << 0));
	ssd2858_write_reg(dssdev, 0x0014, 0x0C37800F);	// SCM_MISC2 (0C77800F): MRXEN = enabled
	ssd2858_write_reg(dssdev, 0x0020, 0x1592567D);	// SCM_ANACTRL1 (1592567D): CPEN = enabled
	ssd2858_write_reg(dssdev, 0x0024, 0x00003000);

#if LOG
	ssd2858_read_reg(dssdev, 0x0008, NULL);
	ssd2858_read_reg(dssdev, 0x000c, NULL);
	ssd2858_read_reg(dssdev, 0x0014, NULL);
	ssd2858_read_reg(dssdev, 0x0020, NULL);
	ssd2858_read_reg(dssdev, 0x0024, NULL);
#endif

	/* some DCS */

	ssd2858_write_cmd0(dssdev, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(1);

	ssd2858_write_cmd4(dssdev, MIPI_DCS_SET_COLUMN_ADDRESS,
			  (ssd2858_timings.x_res-1) >> 24,
			  (ssd2858_timings.x_res-1) >> 16,
			  (ssd2858_timings.x_res-1) >> 8,
			  (ssd2858_timings.x_res-1) >> 0);
	ssd2858_write_cmd4(dssdev, MIPI_DCS_SET_PAGE_ADDRESS,
			  (ssd2858_timings.y_res-1) >> 24,
			  (ssd2858_timings.y_res-1) >> 16,
			  (ssd2858_timings.y_res-1) >> 8,
			  (ssd2858_timings.y_res-1) >> 0);

	/* MIPIRX */

	ssd2858_write_reg(dssdev, 0x1008, 0x01200445);	// MIPIRX_DCR (01200245): HST=4

	/* VCTM */
	ssd2858_write_reg(dssdev, 0x200c, (SPLIT_MEM << 9) | (ddata->rotate << 8) | (VBP << 2) | (TE_SEL << 1));// VCTM_CFGR (00000000)
	ssd2858_write_reg(dssdev, 0x2010, (PCLK_DEN << 16) | (PCLK_NUM << 0));	// VCTM_PCFRR (00010001)
	ssd2858_write_reg(dssdev, 0x2014, (PANEL_FRAME_WIDTH << 16) | ddata->panel_timings.hbp);	// HDCFGR
	ssd2858_write_reg(dssdev, 0x2018, (PANEL_FRAME_HEIGHT << 16) | ddata->panel_timings.vbp);	// VDCFGR
	ssd2858_write_reg(dssdev, 0x201c, (ssd2858_timings.y_res << 16) | ssd2858_timings.x_res);	// MSZR
	ssd2858_write_reg(dssdev, 0x2020, (ddata->panel_timings.y_res << 16) | ddata->panel_timings.x_res);	// DSZR
	ssd2858_write_reg(dssdev, 0x2024, (ddata->panel_timings.y_res << 16) | ddata->panel_timings.x_res);	// PSZR
	ssd2858_write_reg(dssdev, 0x203c, (ddata->panel_timings.y_res << 16) | ddata->panel_timings.x_res);	// ISZR
	ssd2858_write_reg(dssdev, 0x2034, 0x00000000);	// VCTM_POSR (00000000)
	ssd2858_write_reg(dssdev, 0x2038, ((ddata->panel_timings.y_res - 1) << 16) | (ddata->panel_timings.x_res - 1));	// POER
	ssd2858_write_reg(dssdev, 0x2030, 0x00000015);	// URAM refresh period
	ssd2858_write_reg(dssdev, 0x20a0, 0x00000050);	// VTCM_QFBCCTRLR (00004151) - no padding, no pixswap, no fbc

	/* some more DCS */

	ssd2858_write_cmd1(dssdev, MIPI_DCS_SET_TEAR_ON, 0x02);

	ssd2858_write_cmd2(dssdev, MIPI_DCS_SET_TEAR_SCANLINE, ssd2858_timings.y_res >> 8, ssd2858_timings.y_res >> 0);

	if (ddata->flip)
		ssd2858_write_cmd1(dssdev, MIPI_DCS_SET_ADDRESS_MODE, ddata->flip);

	ssd2858_write_reg(dssdev, 0x6008, 0x00000008 | ((PANEL_LANES - 1) << 22) | ((LP_CLK_DIV-1) << 16) | (CKE << 0));	// MIPITX_CTLR (00030008)
	ssd2858_write_reg(dssdev, 0x600c, (ddata->panel_timings.vbp << 24) | (ddata->panel_timings.hbp << 16) | (ddata->panel_timings.vsw << 8) | (ddata->panel_timings.hsw << 0));	// MIPITX_VTC1R (0214020A)
	ssd2858_write_reg(dssdev, 0x6010, (ddata->panel_timings.y_res << 16) | (ddata->panel_timings.vfp << 8) | (ddata->panel_timings.hfp << 0));	// MIPITX_VTC2R (0438020A)
	ssd2858_write_reg(dssdev, 0x6014, 0x01000102 | (VB_MODE << 13) | (VBE << 30));	// MIPITX_VCFR (01000101): VM=burst mode
	ssd2858_write_reg(dssdev, 0x6084, ddata->panel_timings.x_res << 0);	// MIPITX_DSI0VR (00000400)

#if LOG
	ssd2858_read_reg(dssdev, 0x6010, NULL);
#endif

// this sould come from the panel driver...

	ssd2858_pass_to_panel(dssdev, true);	/* communicate with the panel */

#if LOG
	u8 data[8];
	ssd2858_read(dssdev, 0x0b, data, 1);
	ssd2858_read(dssdev, 0x0c, data, 1);
	ssd2858_read(dssdev, 0x45, data, 1);
#endif

	ssd2858_write_cmd0(dssdev, MIPI_DCS_EXIT_SLEEP_MODE);
	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_ON);

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

	r = in->ops.dsi->enable_video_output(in, ddata->pixel_channel);
	if (r)
		goto err;

	// do we need this delay?

	msleep(120);

	ddata->enabled = true;

	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_ON);

#if 0

	r = ssd2858_update_brightness(dssdev, 255);
	if (r)
		goto err;

	r = ssd2858_write_sequence(dssdev, sleep_out, ARRAY_SIZE(sleep_out));
	if (r)
		goto err;

#endif

	printk("dsi: powered on()\n");

	/* FIXME: isn't this far too late? */
//	out->ops.dsi->enable(out);	// power on the panel (will this call some out->ops?)

	return r;
err:
	printk("dsi: power on error\n");
	dev_err(dev, "error powering on ssd2858 - activating reset\n");

	in->ops.dsi->disable(in, false, false);
	mdelay(10);
	ssd2858_reset(dssdev, true);	// activate reset
#if REGULATOR
	ssd2858_regulator(dssdev, 0);	// switch power off
#endif
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

#if LOG
	printk("dsi: ssd2858_power_off()\n");
#endif

	ssd2858_pass_to_panel(dssdev, true);	/* communicate with the panel */

	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_OFF);
	ssd2858_write_cmd0(dssdev, MIPI_DCS_ENTER_SLEEP_MODE);

	ddata->enabled = 0;
	in->ops.dsi->disable_video_output(in, ddata->pixel_channel);

	ssd2858_pass_to_panel(dssdev, false);	/* communicate with the SSD */

	ssd2858_write_cmd0(dssdev, MIPI_DCS_SET_DISPLAY_OFF);
	ssd2858_write_cmd0(dssdev, MIPI_DCS_ENTER_SLEEP_MODE);

	in->ops.dsi->disable(in, false, false);
	mdelay(10);
	ssd2858_reset(dssdev, true);	// activate reset
#if REGULATOR
	ssd2858_regulator(dssdev, 0);	// switch power off - after stopping video stream
#endif
	mdelay(20);
	/* here we can also power off IOVCC */
}

static int ssd2858_start(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r = 0;
#if LOG
	printk("dsi: ssd2858_start()\n");
#endif
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
#if LOG
	printk("dsi: ssd2858_stop()\n");
#endif
	mutex_lock(&ddata->lock);

	in->ops.dsi->bus_lock(in);

	ssd2858_power_off(dssdev);

	in->ops.dsi->bus_unlock(in);

	mutex_unlock(&ddata->lock);
}

static void ssd2858_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
#if LOG
	printk("dsi: ssd2858_disable()\n");
#endif
	dev_dbg(&ddata->pdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		ssd2858_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int ssd2858_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
#if LOG
	printk("dsi: ssd2858_enable()\n");
#endif
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
	 * future improvement:
	 * handle get_rotate and set_rotate - but that significantly changes timing!
	 */
};

#if OLD
static struct device_node *omapdss_of_get_remote_port(const struct device_node *node)
{
	struct device_node *np;

	np = of_parse_phandle(node, "remote-endpoint", 0);
	if (!np)
		return NULL;

	np = of_get_next_parent(np);

	return np;
}

static struct device_node *
omapdss_of_get_nth_endpoint(const struct device_node *parent, int portnum)
{
	struct device_node *port = NULL, *ep;

	do {
		port = omapdss_of_get_next_port(parent, port);

		if (!port)
			return NULL;
	} while (portnum-- > 0);

	ep = omapdss_of_get_next_endpoint(port, NULL);

	of_node_put(port);

	return ep;
}

static struct omap_dss_device *
omapdss_of_find_source_for_nth_ep(struct device_node *node, int portnum)
{
	struct device_node *ep;
	struct device_node *src_port;
	struct omap_dss_device *src;

	ep = omapdss_of_get_nth_endpoint(node, portnum);
	if (!ep)
		return ERR_PTR(-EINVAL);

	src_port = omapdss_of_get_remote_port(ep);
	if (!src_port) {
		of_node_put(ep);
		return ERR_PTR(-EINVAL);
	}

	of_node_put(ep);

	src = omap_dss_find_output_by_port_node(src_port);

	of_node_put(src_port);

	return src ? src : ERR_PTR(-EPROBE_DEFER);
}
#endif

static int ssd2858_probe_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *ep;
	int gpio;
	u32 val32;
#if LOG
	printk("dsi: ssd2858_probe_of(%s)\n", node->name);
#endif
	gpio = of_get_gpio(node, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse reset gpio (err=%d)\n", gpio);
		return gpio;
	}
	ddata->reset_gpio = gpio;

#if REGULATOR
	gpio = of_get_gpio(node, 1);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse regulator gpio (err=%d)\n", gpio);
		return gpio;
	}
	ddata->regulator_gpio = gpio;
#endif

	if (of_property_read_u32(node, "xtal-freq-mhz", &val32)) {
		dev_err(&pdev->dev, "failed to find xtal frequency\n");
		return -EINVAL;
	}
	ddata->xtal = val32;

	ddata->rotate = false;
	ddata->flip = 0;

	val32 = 0;
	of_property_read_u32(node, "rotate", &val32);

	switch (val32) {
	case 0:
		break;
	case 90:
		ddata->rotate = true;
		break;
	case 180:
		ddata->flip = MIPI_DCS_SET_ADDRESS_MODE_HFLIP | MIPI_DCS_SET_ADDRESS_MODE_VFLIP;
		break;
	case 270:
		ddata->rotate = true;
		ddata->flip = MIPI_DCS_SET_ADDRESS_MODE_HFLIP | MIPI_DCS_SET_ADDRESS_MODE_VFLIP;
		break;
	default:
		dev_err(&pdev->dev, "rotate property must be 0, 90, 180 or 270 but is %d\n", val32);
		return -EINVAL;
	}

	if (of_property_read_bool(node, "flip-x"))
		ddata->flip ^= MIPI_DCS_SET_ADDRESS_MODE_HFLIP;
	if (of_property_read_bool(node, "flip-y"))
		ddata->flip ^= MIPI_DCS_SET_ADDRESS_MODE_VFLIP;

#if LOG
	printk("rotate = %d flip = 0x%02x\n", ddata->rotate, ddata->flip);
#endif
	/*
	 * parse additional dt properties like
	 * - type of compression (e.g. optimal, lossy)
	 * - video-pass-through
	 * - test mode
	 * - channel definitions
	 * - charge pump vs. external power
	 * those are to be reflected by different register settings of the SSD2858 chip
	 */

	ep = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(ep)) {
		dev_err(&pdev->dev, "failed to find video source (err=%ld)\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}

	ddata->in = ep;

#if OLD
	ep = omapdss_of_find_source_for_second_ep(node, 1);
	if (IS_ERR(ep)) {
		dev_err(&pdev->dev, "failed to find video sink (err=%ld)\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}

	printk("out = %p\n", ep);
	ddata->out = ep;
#endif

	/*
	 * Room for enhancement:
	 * The SSD has a second MIPITX and can also set the output channel if two
	 * panels are connected in parallel.
	 * So we could handle more output ports or multiple endpoints for the second
	 * port here.
	 */

	return 0;
}

static int ssd2858_probe(struct platform_device *pdev)
{
#if BACKLIGHT
	// struct backlight_properties props;
	struct backlight_device *bldev = NULL;
#endif
	struct panel_drv_data *ddata;
	struct device *dev = &pdev->dev;
	struct omap_dss_device *dssdev;
	int r;

#if LOG
	printk("dsi: ssd2858_probe()\n");
#endif
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

	mutex_init(&ddata->lock);

	if (gpio_is_valid(ddata->reset_gpio)) {
		r = devm_gpio_request_one(dev, ddata->reset_gpio,
					  GPIOF_DIR_OUT, "rotator reset");
		if (r) {
			dev_err(dev, "failed to request reset gpio (%d err=%d)\n", ddata->reset_gpio, r);
			return r;
		}
	}

#if REGULATOR
	if (gpio_is_valid(ddata->regulator_gpio)) {
		r = devm_gpio_request_one(dev, ddata->regulator_gpio,
					 GPIOF_DIR_OUT, "rotator DC/DC regulator");
		if (r) {
			dev_err(dev, "failed to request regulator gpio (%d err=%d)\n", ddata->regulator_gpio, r);
			return r;
		}
	}
#endif

//	ddata->timings = ssd2858_timings;

	dssdev = &ddata->dssdev;
	dssdev->driver = &ssd2858_in_ops;
	dssdev->ops.dsi = &ssd2858_out_ops;
	dssdev->dev = dev;

	/* FIXME: don't these depend on calculation of connected panel timings??? */

//	dssdev->panel.timings = ssd2858_timings;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->output_type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;

	dssdev->panel.dsi_pix_fmt = SSD2858_PIXELFORMAT;
	dssdev->id = OMAP_DSS_OUTPUT_DSI1;

	dssdev->output_type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->name = "dsi.0";
	dssdev->dispc_channel = 1;

	/* register dsi output so that a panel can connect */

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(dev, "Failed to register controller\n");
		goto err_reg;
	}

	/* future: if we want to use the second MIPITX, we should
	 * register another output here
	 */

#if SYSFS
	/* Register sysfs hooks */
	r = sysfs_create_group(&dev->kobj, &ssd2858_attr_group);
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_sysfs_create;
	}
#endif

#if LOG
	printk("ssd2858_probe ok\n");
#endif

	return 0;

#if SYSFS
err_sysfs_create:
#endif
#if BACKLIGHT
	if (bldev != NULL)
		backlight_device_unregister(bldev);
#endif
err_bl:
	//	destroy_workqueue(ddata->workqueue);
err_reg:
	return r;
}


static int __exit ssd2858_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;

#if LOG
	printk("dsi: ssd2858_remove()\n");
#endif
	omapdss_unregister_output(&ddata->dssdev);

	ssd2858_disable(dssdev);
	ssd2858_disconnect(dssdev);

#if SYSFS
	sysfs_remove_group(&pdev->dev.kobj, &ssd2858_attr_group);
#endif
#if BACKLIGHT
	if (ddata->bldev != NULL)
		backlight_device_unregister(ddata->bldev);
#endif

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
