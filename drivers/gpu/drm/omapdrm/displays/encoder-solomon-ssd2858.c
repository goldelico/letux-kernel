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
 * Copyright (C) 2014 Golden Delicious Computers
 * Author: H. Nikolaus Schaller <hns@goldelico.com>
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
#define SYSFS 0
#define LOG 1
/* can be removed later */
#define EDIT_HINT 0
#define REMOVEME 0

#if BACKLIGHT
#include <linux/backlight.h>
#endif
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "../dss/omapdss.h"
#include <video/omap-panel-data.h>
#include <video/mipi_display.h>

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

	u16 tearline;
	bool tear;	/* enable tearing impulse */

	int config_channel;
	int pixel_channel;
};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

struct ssd2858_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

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
	printk("dsi: ssd2858_write_reg: %04x <- %08lx (r=%d)\n", address, data, r);
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
	printk("dsi: ssd2858_read_reg: %04x -> %08lx (r=%d)\n", address, val, r);
#endif
	if (data)
		*data = val;
	return r;
}

#if REMOVEME

/* callbacks from the DSI driver */

static int old_ssd2858_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;

	printk("dsi: ssd2858_connect\n");
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

static void old_ssd2858_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: ssd2858_disconnect\n");
	if (!omapdss_device_is_connected(dssdev))
		return;
	// FIXME: forward disconnect to panel

	in->ops.dsi->release_vc(in, ddata->pixel_channel);
	in->ops.dsi->release_vc(in, ddata->config_channel);
	in->ops.dsi->disconnect(in, dssdev);
}

static void old_ssd2858_get_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("dsi: ssd2858_get_timings\n");
	*timings = dssdev->panel.vm;
}

static void old_ssd2858_set_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("dsi: ssd2858_set_timings\n");
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

static int old_ssd2858_check_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	printk("dsi: ssd2858_check_timings\n");
	return 0;
}

static void old_ssd2858_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	printk("dsi: ssd2858_get_resolution\n");
	*xres = dssdev->panel.vm.hactive;
	*yres = dssdev->panel.vm.vactive;
	// FIXME: forward to panel?
}
#endif

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

static int ssd2858_gen_write(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ /* panel driver wants us to send a generic packet to the panel through the SSD/bypass */
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_gen_write\n");
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

static int ssd2858_dcs_write_nosync(struct omap_dss_device *dssdev, int channel, u8 *buf, int len)
{ /* panel driver wants us to send a DCS packet to the panel through the SSD/bypass */
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
#if LOG
	printk("dsi: ssd2858: ssd2858_dcs_write_nosync\n");
#endif
	ssd2858_pass_to_panel(dssdev, true);	// switch SSD2858 to pass-through mode

	return in->ops.dsi->dcs_write_nosync(in, channel, buf, len);
}

static int ssd2858_gen_read(struct omap_dss_device *dssdev, int channel,
				u8 *reqdata, int reqlen,
				u8 *data, int len)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_gen_read\n");
#endif
	return -EINVAL;
}

static int ssd2858_dcs_read(struct omap_dss_device *dssdev, int channel, u8 dcs_cmd,
				u8 *data, int len)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_dcs_read\n");
#endif
	return -EINVAL;
}

static int ssd2858_set_max_rx_packet_size(struct omap_dss_device *dssdev, int channel, u16 plen)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_set_max_rx_packet_size\n");
#endif
	return -EINVAL;
}

static int ssd2858_connect(struct omap_dss_device *dssdev, struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

#if LOG
	printk("dsi: ssd2858: ssd2858_connect\n");
#endif
	dev_dbg(dssdev->dev, "connect\n");

	if (omapdss_device_is_connected(dssdev))
		return -EBUSY;

	r = in->ops.dsi->connect(in, dssdev);
	if (r)
		return r;

	dst->src = dssdev;
	dssdev->dst = dst;

	return 0;
}

static void ssd2858_disconnect(struct omap_dss_device *dssdev, struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_disconnect\n");
#endif
	dev_dbg(dssdev->dev, "disconnect\n");

	WARN_ON(!omapdss_device_is_connected(dssdev));
	if (!omapdss_device_is_connected(dssdev))
		return;

	WARN_ON(dst != dssdev->dst);
	if (dst != dssdev->dst)
		return;

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.dsi->disconnect(in, &ddata->dssdev);
}

static int ssd2858_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

#if LOG
	printk("dsi: ssd2858: ssd2858_enable\n");
#endif
	dev_dbg(dssdev->dev, "enable\n");

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

// can we calculate timings here?
// there is no dsi->set_timings - is driver->set_timings ok?
	in->driver->set_timings(in, &ddata->timings);

	r = in->ops.dsi->enable(in);
	if (r)
		return r;

// power on and program the sdd here

/*
	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 1);
*/

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void ssd2858_disable(struct omap_dss_device *dssdev, bool disconnect_lanes, bool enter_ulps)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_disable\n");
#endif
	dev_dbg(dssdev->dev, "disable\n");

	if (!omapdss_device_is_enabled(dssdev))
		return;

/* power off the ssd
	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 0);
*/

	in->ops.dsi->disable(in, disconnect_lanes, enter_ulps);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

#if NOT_YET_IMPLEMENTED

static void ssd2858_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	dev_dbg(dssdev->dev, "set_timings\n");

	ddata->timings = *timings;
	dssdev->panel.timings = *timings;

	in->ops.dsi->set_timings(in, timings);
}

static void ssd2858_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	dev_dbg(dssdev->dev, "get_timings\n");

	*timings = ddata->timings;
}

static int ssd2858_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	dev_dbg(dssdev->dev, "check_timings\n");

	return in->ops.dsi->check_timings(in, timings);
}

#endif

static unsigned long channels;

static int ssd2858_request_vc(struct omap_dss_device *dssdev, int *channel)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_request_vc\n");
#endif
	if (channels == 0xffffffff)
		return -ENOSPC;
	*channel = ffz(channels);
	printk("dsi: ssd2858: ssd2858_request_vc assigned channel %d\n", *channel);
	channels |= (1 << *channel);	// reserve this channel
	return 0;
}

static void ssd2858_release_vc(struct omap_dss_device *dssdev, int channel)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_release_vc\n");
#endif
	channels &= ~(1 << channel);	// release channel bit
	return;
}

static void ssd2858_enable_hs(struct omap_dss_device *dssdev, int channel, bool enable)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_enable_hs\n");
#endif
	in->ops.dsi->enable_hs(in, channel, enable);
}

static int ssd2858_enable_video_output(struct omap_dss_device *dssdev, int channel)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_enable_video_output\n");
#endif

	return in->ops.dsi->enable_video_output(in, channel);
}

static void ssd2858_disable_video_output(struct omap_dss_device *dssdev, int channel)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858: ssd2858_disable_video_output\n");
#endif

	return in->ops.dsi->disable_video_output(in, channel);
}

static int ssd2858_set_vc_id(struct omap_dss_device *dssdev, int channel, int vc_id)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_set_vc_id\n");
#endif
	// we use default id values
	return 0;
}

static void ssd2858_bus_lock(struct omap_dss_device *dssdev)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_bus_lock\n");
#endif
	// no locking implemented - would need for multiple panels on single mipitx
	return;
}

static void ssd2858_bus_unlock(struct omap_dss_device *dssdev)
{
#if LOG
	printk("dsi: ssd2858: ssd2858_bus_unlock\n");
#endif
	// no locking
	return;
}

static struct omapdss_dsi_ops ssd2858_dsi_ops = {
	/* we must support at least (used in BOE driver) */

	.connect		= ssd2858_connect,
	.disconnect		= ssd2858_disconnect,

	.enable			= ssd2858_enable,
	.disable		= ssd2858_disable,
	.enable_hs		= ssd2858_enable_hs,
	.enable_video_output	= ssd2858_enable_video_output,
	.disable_video_output	= ssd2858_disable_video_output,

	.gen_write		= ssd2858_gen_write,
	.dcs_write_nosync	= ssd2858_dcs_write_nosync,
	.gen_read		= ssd2858_gen_read,
	.dcs_read		= ssd2858_dcs_read,
	.set_max_rx_packet_size	= ssd2858_set_max_rx_packet_size,

	.request_vc		= ssd2858_request_vc,
	.release_vc		= ssd2858_release_vc,
	.set_vc_id		= ssd2858_set_vc_id,

	.bus_lock		= ssd2858_bus_lock,
	.bus_unlock		= ssd2858_bus_unlock,

#if NOT_YET_IMPLEMENTED
	.check_timings		= ssd2858_check_timings,
	.get_resolution		= ssd2858_get_resolution,
	.set_timings		= ssd2858_set_timings,
	.get_timings		= ssd2858_get_timings,
#endif

	/*
	?  .configure_pins	= ssd2858_configure_pins,
	*/
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

	if (ddata->tear) {
		ssd2858_write_cmd1(dssdev, MIPI_DCS_SET_TEAR_ON, 0x02);
		ssd2858_write_cmd2(dssdev, MIPI_DCS_SET_TEAR_SCANLINE, ddata->tearline >> 8, ddata->tearline >> 0);
	}

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

#if REMOVEME

static void old_ssd2858_disable(struct omap_dss_device *dssdev)
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

static int old_ssd2858_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r = 0;

#if LOG
	printk("dsi: ssd2858_enable()\n");
#endif
	dev_dbg(&ddata->pdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

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
	if (node == NULL) {
		dev_err(&pdev->dev, "Unable to find device tree\n");
		return -EINVAL;
	}

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

	if (of_property_read_u32(node, "te-scanline", &val32)) {
		ddata->tearline = val32;
		ddata->tear = true;
	}

#if LOG
	printk("dsi: ssd2858: rotate = %d flip = 0x%02x\n", ddata->rotate, ddata->flip);
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

	r = ssd2858_probe_of(pdev);
	if (r)
		return r;

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

	dssdev = &ddata->dssdev;
	dssdev->ops.dsi = &ssd2858_dsi_ops;
	dssdev->dev = dev;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->output_type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;

	dssdev->panel.dsi_pix_fmt = SSD2858_PIXELFORMAT;
	dssdev->id = OMAP_DSS_OUTPUT_DSI1;
	dssdev->name = "dsi.0";
	dssdev->dispc_channel = 1;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(dev, "Failed to register output\n");
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
	printk("dsi: ssd2858_probe ok\n");
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
	omap_dss_put_device(ddata->in);
	return r;
}


static int __exit ssd2858_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

#if LOG
	printk("dsi: ssd2858_remove()\n");
#endif
	omapdss_unregister_output(&ddata->dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		ssd2858_disable(dssdev, false, false);	// check parameters!

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		ssd2858_disconnect(dssdev, 0);	// check parameters!

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
MODULE_DESCRIPTION("ssd2858 video encoder driver");
MODULE_LICENSE("GPL");
