/*
 * Driver for MIPI DCS and Generic debugging
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

/* horizontal * vertical * refresh */
#define mipi_debugW				(720)
#define mipi_debugH				(1280)
#define mipi_debugWIDTH			(mipi_debugW+80+88)
#define mipi_debugHEIGHT			(mipi_debugH+160)
#define mipi_debugFPS				(60ll)
#define mipi_debugPIXELCLOCK		(mipi_debugWIDTH * mipi_debugHEIGHT * mipi_debugFPS)	// Half HD * 60 fps
/* panel has 16.7M colors = RGB888 = 3*8 bit per pixel */
#define mipi_debugPIXELFORMAT		OMAP_DSS_DSI_FMT_RGB888	// 16.7M color = RGB888
#define mipi_debugBIT_PER_PIXEL	(3*8)
/* the panel can handle 4 lanes */
#define mipi_debugLANES			4
/* high speed clock is running at double data rate, i.e. half speed
 * (take care of integer overflows!)
 * hsck =  bit/pixel * 110% * pixel clock / lanes / 2 clock edges
 * real clock rate may be rounded up or down depending on divisors
 */
#define mipi_debugHS_CLOCK			(mipi_debugBIT_PER_PIXEL * (mipi_debugPIXELCLOCK / (mipi_debugLANES * 2)))
/* low power clock is quite arbitrarily choosen to be roughly 10 MHz */
#define mipi_debugLP_CLOCK			10000000	// low power clock

static const struct videomode mipi_default_timings = {
	.hactive		= mipi_debugW,
	.vactive		= mipi_debugH,
	.pixelclock	= mipi_debugPIXELCLOCK,
	.hfront_porch		= 5,
	.hsync_len		= 5,
	.hback_porch		= mipi_debugWIDTH-mipi_debugW-5-5,
	.vfront_porch		= 50,
	.vsync_len		= mipi_debugHEIGHT-mipi_debugH-50-50,
	.vback_porch		= 50,
};

static struct omap_dss_dsi_config mipi_dsi_config = {
	.mode = OMAP_DSS_DSI_VIDEO_MODE,
	.pixel_format = mipi_debugPIXELFORMAT,
	.vm = NULL,
	.hs_clk_min = 125000000 /*mipi_debugHS_CLOCK*/,
	.hs_clk_max = 450000000 /*(12*mipi_debugHS_CLOCK)/10*/,
	.lp_clk_min = (7*mipi_debugLP_CLOCK)/10,
	.lp_clk_max = mipi_debugLP_CLOCK,
	.ddr_clk_always_on = true,
	.trans_mode = OMAP_DSS_DSI_BURST_MODE,
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct platform_device *pdev;

	struct mutex lock;

	struct backlight_device *bldev;
	int bl;

	int reset_gpio;
	int regulator_gpio;

	struct omap_dsi_pin_config pin_config;

	bool enabled;

	int config_channel;
	int pixel_channel;

	int max_rx_packet_size;

	char response[256];

};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

struct mipi_debug_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

static int mipi_debug_write(struct omap_dss_device *dssdev, u8 *buf, int len, int generic)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	printk("dsi: mipi_debug_write(%s", generic?"g,":""); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");

	if(generic)
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

static int mipi_debug_read(struct omap_dss_device *dssdev, u8 *dcs_cmd, int cmdlen, u8 *buf, int len, int generic)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int i;

	if(len != ddata->max_rx_packet_size) { /* has changed */
		r = in->ops.dsi->set_max_rx_packet_size(in, ddata->config_channel, len);	// tell panel how much we expect
		if (r) {
			dev_err(&ddata->pdev->dev, "can't set max rx packet size\n");
			return -EIO;
		}
		printk("dsi: mipi_debug_read() rx packet size := %d\n", len);
		ddata->max_rx_packet_size = len;
	}
	if(generic)
		{ // this is a "manufacturer command" that must be sent as a "generic read command"
			r = in->ops.dsi->gen_read(in, ddata->config_channel, dcs_cmd, cmdlen, buf, len);
		}
	else
		{ // this is a "user command" that must be sent as "DCS command"
			if(cmdlen != 1)
				{
				printk("dsi: mipi_debug_read() must specify 1 byte for DCS read commands!");
				return -EIO;
				}
			r = in->ops.dsi->dcs_read(in, ddata->config_channel, dcs_cmd[0], buf, len);
		}

	printk("dsi: mipi_debug_read(%s", generic?"g,":"");
	for(i=0; i<cmdlen; i++) printk(" %02x", dcs_cmd[i]);
	printk(") -> ");
	for(i=0; i<len; i++) printk(" %02x", buf[i]);
	if (r)
		printk(" failed: %d", r);
	printk("\n");
	return r;
}

static int mipi_debug_read_dcs(struct omap_dss_device *dssdev, u8 dcs_cmd, u8 *buf, int len)
{
	return mipi_debug_read(dssdev, &dcs_cmd, 1, buf, len, 0);
}

static int mipi_debug_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;

	printk("dsi: mipi_debug_connect()\n");

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

	printk("dsi: mipi_debug_connect() connected\n");

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

static void mipi_debug_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: mipi_debug_disconnect()\n");

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dsi->release_vc(in, ddata->pixel_channel);
	in->ops.dsi->release_vc(in, ddata->config_channel);
	in->ops.dsi->disconnect(in, dssdev);

	printk("dsi: mipi_debug_disconnect() disconnected\n");
}


static void mipi_debug_get_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	/* if we are connected to the ssd2858 driver in->driver provides get_timings() */

	if (in->driver && in->driver->get_timings) {
		printk("dsi: mipi_debug_get_timings() get from source\n");
		in->driver->get_timings(in, timings);
	} else
		*timings = dssdev->panel.vm;
}

static void mipi_debug_set_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	dssdev->panel.vm.hactive = timings->hactive;
	dssdev->panel.vm.vactive = timings->vactive;
	dssdev->panel.vm.pixelclock = timings->pixelclock;
	dssdev->panel.vm.hsync_len = timings->hsync_len;
	dssdev->panel.vm.hfront_porch = timings->hfront_porch;
	dssdev->panel.vm.hback_porch = timings->hback_porch;
	dssdev->panel.vm.vsync_len = timings->vsync_len;
	dssdev->panel.vm.vfront_porch = timings->vfront_porch;
	dssdev->panel.vm.vback_porch = timings->vback_porch;
}

static int mipi_debug_check_timings(struct omap_dss_device *dssdev,
		struct videomode *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	/* if we are connected to the ssd2858 driver in->driver provides check_timings() */

	if (in->driver && in->driver->check_timings) {
		printk("dsi: mipi_debug_check_timings() check with source\n");
		return in->driver->check_timings(in, timings);
	}
	return 0;
}

#if 0
static void mipi_debug_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.vm.hactive;
	*yres = dssdev->panel.vm.vactive;
}
#endif

static int mipi_debug_reset(struct omap_dss_device *dssdev, bool activate)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: mipi_debug_reset(%s)\n", activate?"active":"inactive");
	if (gpio_is_valid(ddata->reset_gpio))
		gpio_set_value(ddata->reset_gpio, !activate);
	return 0;
}

static int mipi_debug_regulator(struct omap_dss_device *dssdev, int state)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	printk("dsi: mipi_debug_regulator(%d)\n", state);
	if (gpio_is_valid(ddata->regulator_gpio))
		gpio_set_value(ddata->regulator_gpio, state);	// switch regulator
	return 0;
}

#if 0
static int mipi_debug_update_brightness(struct omap_dss_device *dssdev, int level)
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
	r = mipi_debug_write(dssdev, buf, sizeof(buf), 0);
	if (r)
		return r;
	return 0;
}
#endif

static int mipi_debug_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
//	struct omap_dss_device *in = ddata->in;
	int bl = bd->props.brightness;
	int r = 0;
	printk("dsi: mipi_debug_set_brightness(%d)\n", bl);
	if (bl == ddata->bl)
		return 0;

#if 0

	mutex_lock(&ddata->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		in->ops.dsi->bus_lock(in);

		r = mipi_debug_update_brightness(dssdev, bl);
		if (!r)
			ddata->bl = bl;

		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);
#endif

	return r;
}

static int mipi_debug_get_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	u8 data[16];
	u16 brightness = 0;
	int r = 0;
	printk("dsi: mipi_debug_get_brightness()\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk("dsi: mipi_debug_get_brightness(): display is not active\n");
		return 0;
	}

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);
		r = mipi_debug_read_dcs(dssdev, DCS_READ_BRIGHTNESS, data, 2);
		brightness = (data[0]<<4) + (data[1]>>4);

		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	if(r < 0) {
		printk("dsi: mipi_debug_get_brightness(): read error\n");
		return bd->props.brightness;
	}
	printk("dsi: mipi_debug_get_brightness() read %d\n", brightness);
	return brightness>>4;	// get to range 0..255
}

static const struct backlight_ops mipi_debug_backlight_ops  = {
	.get_brightness = mipi_debug_get_brightness,
	.update_status = mipi_debug_set_brightness,
};

/* sysfs callbacks */

static int mipi_debug_start(struct omap_dss_device *dssdev);
static void mipi_debug_stop(struct omap_dss_device *dssdev);

static ssize_t set_dcs(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int r = 0;
	u8 data[24];
	u8 ret[24];
	u8 d = 0;
	int argc = 0;
	int second = 0;
	int read = 0;
	int generic = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;
	const char *p;

	if(strncmp(buf, "start", 5) == 0) {
		int r;
		p = buf+5;
		while(p < buf + count) {
			if(!(*p == ' ' || *p == '\t' || *p == '\n')) {
				// parse "start var=value" sequences
				const char *arg=p;
				unsigned long val=0;
				int len;
				while(p < buf + count) {
					if(*p == '=')
						break;
					p++;
					}
				if(!(p < buf + count)) {
					printk("missing '='\n");
					return -EIO;
				}
				len=p++ - arg;	// skip =
				while(p < buf + count) {
					// collect digits
					if(*p == ' ' || *p == '\t' || *p == '\n')
						break;	// end of this value
					if(*p >= '0' && *p <= '9')
						val=10*val + (*p-'0');
					else {
						printk("invalid digit for parameter %.*s: %c\n", len, arg, *p);
						return -EIO;
					}
					p++;
				}
				/* let's hope that the video modes are really changed... */
				if(len == 5 && strncmp(arg, "x_res", len) == 0)
					dssdev->panel.vm.hactive=val;
				else if(len == 5 && strncmp(arg, "y_res", len) == 0)
					dssdev->panel.vm.vactive=val;
				else if(len == 10 && strncmp(arg, "pixelclock", len) == 0)
					dssdev->panel.vm.pixelclock=val;
				else if(len == 3 && strncmp(arg, "hfp", len) == 0)
					dssdev->panel.vm.hfront_porch=val;
				else if(len == 3 && strncmp(arg, "hsw", len) == 0)
					dssdev->panel.vm.hsync_len=val;
				else if(len == 3 && strncmp(arg, "hbp", len) == 0)
					dssdev->panel.vm.hback_porch=val;
				else if(len == 3 && strncmp(arg, "vfp", len) == 0)
					dssdev->panel.vm.vfront_porch=val;
				else if(len == 3 && strncmp(arg, "vsw", len) == 0)
					dssdev->panel.vm.vsync_len=val;
				else if(len == 3 && strncmp(arg, "vbp", len) == 0)
					dssdev->panel.vm.vback_porch=val;
				/* mipi_dsi_config evaluated during mipi_debug_start() */
				else if(len == 7 && strncmp(arg, "lpclock", len) == 0)
					mipi_dsi_config.lp_clk_max=val;
// add others here
				else {
					printk("unknown parameter: %.*s=%lu\n", len, arg, val);
					return -EIO;	// invalid parameter name
				}
				printk("  %.*s=%lu\n", len, arg, val);
			}
			p++;
		}
		r = mipi_debug_start(dssdev);
		return r < 0 ? r : count;
	}
	if(strncmp(buf, "stop", 4) == 0)
		{
		mipi_debug_stop(dssdev);
		return count;
		}
	if(strncmp(buf, "stream", 6) == 0)
		{
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			{
			in->ops.dsi->bus_lock(in);
			in->ops.dsi->enable_video_output(in, ddata->pixel_channel);
			in->ops.dsi->bus_unlock(in);
			}
		return count;
		}
	if(strncmp(buf, "nostream", 8) == 0)
		{
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			{
			in->ops.dsi->bus_lock(in);
			in->ops.dsi->disable_video_output(in, ddata->pixel_channel);
			in->ops.dsi->bus_unlock(in);
			}
		return count;
		}
	if(strncmp(buf, "reset", 5) == 0)
		{
		mipi_debug_reset(dssdev, true);
		return count;
		}
	if(strncmp(buf, "noreset", 7) == 0)
		{
		mipi_debug_reset(dssdev, false);
		return count;
		}
	if(strncmp(buf, "power", 5) == 0)
		{
		mipi_debug_regulator(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "nopower", 7) == 0)
		{
		mipi_debug_regulator(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "status", 6) == 0) {
		mutex_lock(&ddata->lock);
		if (ddata->enabled) {
			in->ops.dsi->bus_lock(in);
			/* read some registers through DCS commands */
			r = mipi_debug_read_dcs(dssdev, 0x05, ret, 1);
			r = mipi_debug_read_dcs(dssdev, 0x0a, ret, 1);	// power mode 0x10=sleep off; 0x04=display on
			r = mipi_debug_read_dcs(dssdev, 0x0b, ret, 1);	// address mode
			r = mipi_debug_read_dcs(dssdev, MIPI_DCS_GET_PIXEL_FORMAT, ret, 1);	// pixel format 0x70 = RGB888
			r = mipi_debug_read_dcs(dssdev, 0x0d, ret, 1);	// display mode	0x80 = command 0x34/0x35
			r = mipi_debug_read_dcs(dssdev, 0x0e, ret, 1);	// signal mode
			r = mipi_debug_read_dcs(dssdev, MIPI_DCS_GET_DIAGNOSTIC_RESULT, ret, 1);	// diagnostic 0x40 = functional
			r = mipi_debug_read_dcs(dssdev, 0x45, ret, 2);	// get scanline
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
		if(!generic && argc == 0 && *p == 'g')
			{ // must be first
			generic=1;
			continue;
			}
		if(!second && argc >= 1 && *p == 'r')	// r must follow the address (or another r)
			{ // count number of r
			if(read >= sizeof(ret))
				return -EIO;
			read++;
			continue;
			}
		if(read > 0)
			return -EIO;	// no hex digits after the first r
		if(argc == sizeof(data))
			return -EIO;
		if(*p >= '0' && *p <= '9')
			d=(d<<4) + (*p-'0');
		else if(*p >= 'a' && *p <= 'f')
			d=(d<<4) + (*p-'a') + 10;
		else if(*p >= 'A' && *p <= 'F')
			d=(d<<4) + (*p-'A') + 10;
		else
			return -EIO;
		if(second)
			data[argc++]=d;	// store after every second digit
		second ^= 1;
		}

//	printk("  2nd:%d argc:%d ---\n", second, argc);

	if(second)
		return -EIO;	// was not an even number of digits

	if(argc == 0)
		return -EIO;	// missing data[0]

	mutex_lock(&ddata->lock);
	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);

		// handle DCS vs. Generic
		if(read)
			{
			r = mipi_debug_read(dssdev, data, argc, ret, read, generic);
			if(r >= 0)
				{ /* provide response bytes to next cat dcs */
				char *p=ddata->response;
				int i;
				for(i=0; i<read; i++)
					p+=sprintf(p, "%s%02x", i==0?"":" ", ret[i]);
				}
			else
				sprintf(ddata->response, "error: %d", r);	// failure
			}
		else
			r = mipi_debug_write(dssdev, data, argc, generic);

		in->ops.dsi->bus_unlock(in);
	} else
		r=-EIO;	// not enabled
	mutex_unlock(&ddata->lock);

	return r < 0 ? r : count;
}

static ssize_t show_dcs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	if (ddata->response[0])
		{ /* return hex bytes of last read command */
		int l=sprintf(buf, "%s\n", ddata->response);
		ddata->response[0]=0;	/* but return exactly once */
		return l;
		}
	return sprintf(buf, "usage: [g]aa dd dd ... | [g]aa r ... | status | start [ x_res=# | y_res=# | pixelclock=# | lpclock=# ] | stop | reset | noreset | power | nopower | stream | nostream\n");
}

static DEVICE_ATTR(dcs, S_IWUSR | S_IRUGO,
				   show_dcs, set_dcs);

static struct attribute *mipi_debug_attributes[] = {
	&dev_attr_dcs.attr,
	NULL
};

static const struct attribute_group mipi_debugattr_group = {
	.attrs = mipi_debug_attributes,
};


static int mipi_debug_power_on(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct device *dev = &ddata->pdev->dev;
	struct omap_dss_device *in = ddata->in;
	int r;
	printk("dsi: mipi_debug_power_on()\n");
//	printk("hs_clk_min=%lu\n", mipi_dsi_config.hs_clk_min);

	if(ddata->enabled)
		return 0;	// already enabled

//	mipi_debug_reset(dssdev, true);	// activate reset
//	mipi_debug_regulator(dssdev, 0);	// switch power off

#if 0
	if (ddata->pin_config.num_pins > 0) {
		r = in->ops.dsi->configure_pins(in, &ddata->pin_config);
		if (r) {
			dev_err(&ddata->pdev->dev,
					"failed to configure DSI pins\n");
			return r;
		}
	}
#endif

	mipi_dsi_config.vm = &dssdev->panel.vm;

	r = in->ops.dsi->set_config(in, &mipi_dsi_config);
	if (r) {
		dev_err(dev, "failed to configure DSI\n");
		return r;
	}

	r = in->ops.dsi->enable(in);
	if (r) {
		dev_err(dev, "failed to enable DSI\n");
		return r;
	}

#if 0	// user space must apply "power" + "noreset"
	mipi_debug_regulator(dssdev, 1);	// switch power on
	msleep(50);

	mipi_debug_reset(dssdev, false);	// release reset
	msleep(10);
#endif

	in->ops.dsi->enable_hs(in, ddata->pixel_channel, true);

	/* don't initialize the panel here - user space has to do */

	ddata->enabled = true;
	printk("dsi: mipi_debug_power_on: enabled()\n");

	return r;
}

// we don't have a sophisticated power management (sending the panel to power off)
// we simply stop the video stream and assert the RESET
// please note that we don't/can't switch off the VCCIO

static void mipi_debug_power_off(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: mipi_debug_power_off()\n");
	if(!ddata->enabled)
		return;	// already disabled
	ddata->enabled = 0;
	in->ops.dsi->disable_video_output(in, ddata->pixel_channel);
	in->ops.dsi->disable(in, false, false);
#if 0
	mdelay(10);
	mipi_debug_reset(dssdev, true);	// activate reset
	mipi_debug_regulator(dssdev, 0);	// switch power off - after stopping video stream
	mdelay(20);
	/* here we can also power off IOVCC */
#endif
}

static int mipi_debug_start(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r = 0;

	ddata->max_rx_packet_size = -1;

	printk("dsi: mipi_debug_start()\n");

	printk("  Dimensions: {%ux%u} in {%ux%u}\n", dssdev->panel.vm.vactive,
		/* FIXME: */  dssdev->panel.vm.hactive, 0, 0);
	printk("  Pixel CLK: %lu\n", dssdev->panel.vm.pixelclock);
	printk("  HSYNC: %u %u %u\n", dssdev->panel.vm.hfront_porch, dssdev->panel.vm.hsync_len, dssdev->panel.vm.hback_porch);
	printk("  VSYNC: %u %u %u\n", dssdev->panel.vm.vfront_porch, dssdev->panel.vm.vsync_len, dssdev->panel.vm.vback_porch);
	printk("  DDR CLK: %lu..%lu\n", mipi_dsi_config.hs_clk_min, mipi_dsi_config.hs_clk_max);
	printk("  LPCLK out: %lu..%lu\n", mipi_dsi_config.lp_clk_min, mipi_dsi_config.lp_clk_max);
	printk("  MODE: %u\n", mipi_dsi_config.mode);
	printk("  FMT: %u\n", mipi_dsi_config.pixel_format);
	printk("  Always On: %u\n", mipi_dsi_config.ddr_clk_always_on);
	printk("  Trans Mode: %u\n", mipi_dsi_config.trans_mode);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	mutex_lock(&ddata->lock);

	in->ops.dsi->bus_lock(in);

	r = mipi_debug_power_on(dssdev);

	in->ops.dsi->bus_unlock(in);

	if (r)
		dev_err(&ddata->pdev->dev, "enable failed\n");
	else
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ddata->lock);

	return r;
}

static void mipi_debug_stop(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	printk("dsi: mipi_debug_stop()\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	mutex_lock(&ddata->lock);

	in->ops.dsi->bus_lock(in);

	mipi_debug_power_off(dssdev);

	in->ops.dsi->bus_unlock(in);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&ddata->lock);
}

/* enable/disable do nothing */

static void mipi_debug_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	printk("dsi: mipi_debug_disable()\n");
	dev_dbg(&ddata->pdev->dev, "disable\n");

//	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
//		mipi_debug_stop(dssdev);

//	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int mipi_debug_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	printk("dsi: mipi_debug_enable()\n");
	dev_dbg(&ddata->pdev->dev, "enable\n");

//	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
//		return -EINVAL;
	printk("dsi: mipi_debug_enable() done\n");

	return 0;
}

static struct omap_dss_driver mipi_debugops = {
	.connect	= mipi_debug_connect,
	.disconnect	= mipi_debug_disconnect,

	.enable		= mipi_debug_enable,
	.disable	= mipi_debug_disable,

#if 0
	.get_resolution	= mipi_debug_get_resolution,
#endif

	.check_timings	= mipi_debug_check_timings,
	.set_timings	= mipi_debug_set_timings,
	.get_timings	= mipi_debug_get_timings,
};

static int mipi_debug_probe_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *ep;
	int gpio;

	printk("dsi: mipi_debug_probe_of()\n");

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

	return 0;
}

static int mipi_debug_probe(struct platform_device *pdev)
{
	// struct backlight_properties props;
	struct backlight_device *bldev = NULL;
	struct panel_drv_data *ddata;
	struct device *dev = &pdev->dev;
	struct omap_dss_device *dssdev;
	int r;

	printk("dsi: mipi_debug_probe()\n");
	dev_dbg(dev, "mipi_debug_probe\n");

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);
	ddata->pdev = pdev;

	if (dev_get_platdata(dev)) {
		r = -EINVAL /*mipi_debug_probe_pdata(pdev)*/;
		if (r)
			return r;
	} else if (pdev->dev.of_node) {
		r = mipi_debug_probe_of(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &mipi_debugops;
	dssdev->panel.vm = mipi_default_timings;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;

	dssdev->panel.dsi_pix_fmt = mipi_debugPIXELFORMAT;

// can we postpone this so that user space can modify the mipi timings before registration?

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

	mipi_debug_reset(dssdev, true);	// start with active reset

	if (gpio_is_valid(ddata->regulator_gpio)) {
		r = devm_gpio_request_one(dev, ddata->regulator_gpio,
								  GPIOF_DIR_OUT, "rotator DC/DC regulator");
		if (r) {
			dev_err(dev, "failed to request regulator gpio (%d err=%d)\n", ddata->regulator_gpio, r);
			return r;
		}
	}

	mipi_debug_regulator(dssdev, 0);	// start with power off

	/* Register sysfs hooks */
	r = sysfs_create_group(&dev->kobj, &mipi_debugattr_group);
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_sysfs_create;
	}

	printk("dsi: mipi_debug_probe ok\n");

	return 0;

err_sysfs_create:
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_reg:
	return r;
}


static int __exit mipi_debug_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	// struct backlight_device *bldev;

	printk("dsi: mipi_debug_remove()\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		{
		ddata->in->ops.dsi->bus_lock(ddata->in);
		ddata->in->ops.dsi->disable_video_output(ddata->in, ddata->pixel_channel);
		ddata->in->ops.dsi->bus_unlock(ddata->in);
		}

	mipi_debug_stop(dssdev);

	mipi_debug_regulator(dssdev, 0);	// power off

	omapdss_unregister_display(dssdev);

	mipi_debug_disable(dssdev);
	mipi_debug_disconnect(dssdev);

	omap_dss_put_device(ddata->in);

	mutex_destroy(&ddata->lock);

	sysfs_remove_group(&pdev->dev.kobj, &mipi_debugattr_group);

	return 0;
}

static const struct of_device_id mipi_debug_of_match[] = {
	{ .compatible = "omapdss,mipi,debug", },
	{},
};

MODULE_DEVICE_TABLE(of, mipi_debug_of_match);

static struct platform_driver mipi_debug_driver = {
	.probe = mipi_debug_probe,
	.remove = mipi_debug_remove,
	.driver = {
		.name = "mipi-debug",
		.owner = THIS_MODULE,
		.of_match_table = mipi_debug_of_match,
	},
};

module_platform_driver(mipi_debug_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("mipi-debug driver");
MODULE_LICENSE("GPL");
