/*
 * Driver for panels with Samsung S6D2AA0X01
 *
 * and specifically the LS052K3SY54S
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/device.h>

#include <video/omapdss.h>
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

/* manufacturer specific commands */
#define MCS_PASSWD1			0xf0
#define MCS_BUS_DELAY_ON	0xe4
#define MCS_INIT_H_L		0xf2


#define IS_MCS(CMD) (((CMD) >= 0xb0 && (CMD) <= 0xff) && !((CMD) == 0xda || (CMD) == 0xdb || (CMD) == 0xdc))

/* horizontal * vertical * refresh */
#define S6D2AA0X01_PIXELCLOCK		((720+226) * (1280+16) * 60)	// HD * 60 fps
/* panel has 16.7M colors = RGB888 = 3*8 bit per pixel */
#define S6D2AA0X01_PIXELFORMAT		OMAP_DSS_DSI_FMT_RGB888	// 16.7M color = RGB888
#define S6D2AA0X01_BIT_PER_PIXEL	(3*8)
/* the panel can handle 4 lanes */
#define S6D2AA0X01_LANES			4
/* high speed clock is running at double data rate, i.e. half speed
 * (take care of integer overflows!)
 * hsck =  bit/pixel * 110% * pixel clock / lanes / 2 clock edges
 * real clock rate may be rounded up or down depending on divisors
 */
#define S6D2AA0X01_HS_CLOCK			(S6D2AA0X01_BIT_PER_PIXEL * (S6D2AA0X01_PIXELCLOCK / (S6D2AA0X01_LANES * 2)))
/* low power clock is quite arbitrarily choosen to be roughly 10 MHz */
#define S6D2AA0X01_LP_CLOCK			10000000	// low power clock

static struct omap_video_timings s6d2aa0x01_timings = {
	.x_res		= 720,
	.y_res		= 1280,
	.pixel_clock	= S6D2AA0X01_PIXELCLOCK/1000,	// specified in kHz
	// the following values are choosen arbitrarily since there is no spec in the data sheet
	// they are choosen to round up to 1200x2000 pixels giving a pixel clock of 144 MHz
	.hfp		= 30,
	.hsw		= 4,
	.hbp		= 192,
	.vfp		= 8,
	.vsw		= 2,
	.vbp		= 6,
};

/*
 * The DSI port needs its own hsync and vsync timing (to insert the sync
 * packets at the right moments and protect the link from congestions or drain)
 * or we will see loss of SYNC interrupts
 *
 * There is a timing calculator spreadsheet
 * <http://e2e.ti.com/cfs-file.ashx/__key/communityserver-discussions-components-files/849/2555.Demistify-DSI-IF-_2D00_-Video-mode-registers-settings.xlsx, http://e2e.ti.com/support/omap/f/849/p/289189/1013573.aspx>
 * linked by this  * discussion: <http://e2e.ti.com/support/omap/f/849/p/289189/1013573.aspx>
 *
 * It must be used to verify the clock settings (compare with cat /sys/kernel/debug/omapdss/clk)
 * and to adjust the .h?? and .v?? values so that DISPC and DSI run at the same speed
 */

static struct omap_dss_dsi_videomode_timings vm_data = {
	.hsa			= 0,	// ignored unless OMAP_DSS_DSI_PULSE_MODE
	.hfp			= 12,	// this is calculated in byte clocks and not in pixels - they are ca. 75%
	.hbp			= 75,	// this should be shorter by ca. 25% than for the DISPC
	.vsa			= 10,
	.vfp			= 62,
	.vbp			= 8,

	.blanking_mode		= 0,
	.hsa_blanking_mode	= 1,
	.hfp_blanking_mode	= 1,
	.hbp_blanking_mode	= 1,

	.ddr_clk_always_on	= true,

	.window_sync		= 4,
};

struct s6d2aa0x01_data {
	struct mutex lock;

	int	reset_gpio;
	int	regulator_gpio;
	struct omap_dsi_pin_config pin_config;

	struct omap_dss_device *dssdev;
	struct backlight_device *bldev;
	bool enabled;
	int bl;

	int config_channel;
	int pixel_channel;

	struct omap_video_timings *timings;
};

struct s6d2aa0x01_reg {
	/* Address and register value */
	u8 data[50];
	int len;
};

static struct s6d2aa0x01_reg init_seq[] = {
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_PASSWD1,
		0x5A, 0x5A
	}, 3 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_BUS_DELAY_ON,
		0x00, 0x04,
	}, 3 },
	{ {	MCS_PASSWD2,
		0xA5, 0xA5
	}, 3 },

#if 1
	{ { DCS_CTRL_DISPLAY, 0x24}, 2 },	// LEDPWM ON
	{ { DCS_WRITE_CABC, 0x00}, 2 },		// CABC off
#endif
};

static struct s6d2aa0x01_reg sleep_out[] = {
	{ { MIPI_DCS_SET_DISPLAY_ON, }, 1 },
};

static struct r63311_reg display_on[] = {
	{ { MIPI_DCS_EXIT_SLEEP_MODE, }, 1 },
};

/* not used - needs also MCS_VCOMSET and some delays */

static struct s6d2aa0x01_reg power_off_seq[] = {

	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_PASSWD1,
		0x5A, 0x5A
	}, 3 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MIPI_DCS_NOP }, 1 },
	{ {	MCS_INIT_H_L,
		0x10, 0x00, 0x00, 0x00, 0x00
		0x12, 0x01, 0x0A, 0x36, 0x0E
		0x0A, 0x99, 0x4C
	}, 14 },
	{ {	MCS_PASSWD2,
		0xA5, 0xA5
	}, 3 },
};

static struct s6d2aa0x01_reg display_off[] = {
	{ { MIPI_DCS_SET_DISPLAY_OFF, }, 1},
};

static struct s6d2aa0x01_reg sleep_in[] = {
	{ { MIPI_DCS_ENTER_SLEEP_MODE, }, 1},
};

static int s6d2aa0x01_write(struct omap_dss_device *dssdev, u8 *buf, int len)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r;
	int i;

	printk("dsi: s6d2aa0x01_write("); for(i=0; i<len; i++) printk("%02x%s", buf[i], i+1 == len?")\n":" ");

	if(IS_MCS(buf[0]))
		{ // this is a "manufacturer command" that must be sent as a "generic read command"
			r = dsi_vc_generic_write(dssdev, lg_d->config_channel, buf, len);
		}
	else
		{ // this is a "user command" that must be sent as "DCS command"
			r = dsi_vc_dcs_write_nosync(dssdev, lg_d->config_channel, buf, len);
		}

	if (r)
		dev_err(&dssdev->dev, "write cmd/reg(%x) failed: %d\n",
				buf[0], r);

	return r;
}

static int s6d2aa0x01_read(struct omap_dss_device *dssdev, u8 dcs_cmd, u8 *buf, int len)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r;
	int i;

	r = dsi_vc_set_max_rx_packet_size(dssdev, lg_d->config_channel, len);	// tell panel how much we expect
	if (r) {
		dev_err(&dssdev->dev, "can't set max rx packet size\n");
		return -EIO;
	}

	if(IS_MCS(dcs_cmd))
		{ // this is a "manufacturer command" that must be sent as a "generic read command"
			r = dsi_vc_generic_read_1(dssdev, lg_d->config_channel, dcs_cmd, buf, len);
		}
	else
		{ // this is a "user command" that must be sent as "DCS command"
			r = dsi_vc_dcs_read(dssdev, lg_d->config_channel, dcs_cmd, buf, len);
		}

	if (r)
		dev_err(&dssdev->dev, "read cmd/reg(%02x, %d) failed: %d\n",
				dcs_cmd, len, r);
	printk("dsi: s6d2aa0x01_read(%02x,", dcs_cmd); for(i=0; i<len; i++) printk(" %02x", buf[i]);
	printk(") -> %d\n", r);
	return r;
}

static int s6d2aa0x01_write_sequence(struct omap_dss_device *dssdev,
		struct s6d2aa0x01_reg *seq, int len)
{
	int r, i;

	for (i = 0; i < len; i++) {
		r = s6d2aa0x01_write(dssdev, seq[i].data, seq[i].len);
		if (r) {
			dev_err(&dssdev->dev, "sequence failed: %d\n", i);
			return -EINVAL;
		}

		/* TODO: Figure out why this is needed for OMAP5 */
		msleep(1);
	}

	return 0;
}

static void s6d2aa0x01_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void s6d2aa0x01_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int s6d2aa0x01_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return 0;
}

static void s6d2aa0x01_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int s6d2aa0x01_reset(struct omap_dss_device *dssdev, int state)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	printk("dsi: s6d2aa0x01_reset(%d)\n", state);
	gpio_set_value(lg_d->reset_gpio, state);
	return 0;
}

static int s6d2aa0x01_regulator(struct omap_dss_device *dssdev, int state)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	printk("dsi: s6d2aa0x01_regulator(%d)\n", state);
	gpio_set_value(lg_d->regulator_gpio, state);	// switch regulator
	return 0;
}

static int s6d2aa0x01_update_brightness(struct omap_dss_device *dssdev, int level)
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
	r = s6d2aa0x01_write(dssdev, buf, sizeof(buf));
	if (r)
		return r;
	return 0;
}

static int s6d2aa0x01_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int bl = bd->props.brightness;
	int r = 0;
	printk("dsi: s6d2aa0x01_set_brightness(%d)\n", bl);

	if (bl == lg_d->bl)
		return 0;

	mutex_lock(&lg_d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);

		r = s6d2aa0x01_update_brightness(dssdev, bl);
		if (!r)
			lg_d->bl = bl;

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&lg_d->lock);

	return r;
}

static int s6d2aa0x01_get_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	u8 data[16];
	u16 brightness = 0;
	int r = 0;
	printk("dsi: s6d2aa0x01_get_brightness()\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk("dsi: display is not active\n");
		return 0;
	}

	mutex_lock(&lg_d->lock);

	if (lg_d->enabled) {
		dsi_bus_lock(dssdev);
		r = s6d2aa0x01_read(dssdev, DCS_READ_BRIGHTNESS, data, 2);
		brightness = (data[0]<<4) + (data[1]>>4);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&lg_d->lock);

	if(r < 0) {
		printk("dsi: read error\n");
		return bd->props.brightness;
	}
	printk("dsi: read %d\n", brightness);
	return brightness>>4;	// get to range 0..255
}

static const struct backlight_ops s6d2aa0x01_backlight_ops  = {
	.get_brightness = s6d2aa0x01_get_brightness,
	.update_status = s6d2aa0x01_set_brightness,
};

/* sysfs callbacks */

static int s6d2aa0x01_start(struct omap_dss_device *dssdev);
static void s6d2aa0x01_stop(struct omap_dss_device *dssdev);

static ssize_t set_dcs(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u8 data[24];
	u8 d = 0;
	int argc = 0;
	int second = 0;
	int read = 0;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r = 0;
	const char *p;

	if(strncmp(buf, "start", 5) == 0)
		{
		int r = s6d2aa0x01_start(dssdev);
		return r < 0 ? r : count;
		}
	if(strncmp(buf, "stop", 4) == 0)
		{
		s6d2aa0x01_stop(dssdev);
		return count;
		}
	if(strncmp(buf, "reset", 5) == 0)
		{
		s6d2aa0x01_reset(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "noreset", 7) == 0)
		{
		s6d2aa0x01_reset(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "power", 5) == 0)
		{
		s6d2aa0x01_regulator(dssdev, 1);
		return count;
		}
	if(strncmp(buf, "nopower", 7) == 0)
		{
		s6d2aa0x01_regulator(dssdev, 0);
		return count;
		}
	if(strncmp(buf, "status", 6) == 0) {
		mutex_lock(&lg_d->lock);
		if (lg_d->enabled) {
			dsi_bus_lock(dssdev);
			r = s6d2aa0x01_read(dssdev, 0x04, data, 3);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			r = s6d2aa0x01_read(dssdev, DCS_READ_NUM_ERRORS, data, 1);	// dsi errors
			//		r = s6d2aa0x01_read(dssdev, 0x06, data, 1);	// red
			//		r = s6d2aa0x01_read(dssdev, 0x07, data, 1);	// green
			//		r = s6d2aa0x01_read(dssdev, 0x08, data, 1);	// blue
			r = s6d2aa0x01_read(dssdev, 0x0a, data, 1);	// power mode 0x10=sleep off; 0x04=display on
			//		r = s6d2aa0x01_read(dssdev, 0x0b, data, 1);	// address mode
			r = s6d2aa0x01_read(dssdev, MIPI_DCS_GET_PIXEL_FORMAT, data, 1);	// pixel format 0x70 = RGB888
			r = s6d2aa0x01_read(dssdev, 0x0d, data, 1);	// display mode	0x80 = command 0x34/0x35
			r = s6d2aa0x01_read(dssdev, 0x0e, data, 1);	// signal mode
			r = s6d2aa0x01_read(dssdev, MIPI_DCS_GET_DIAGNOSTIC_RESULT, data, 1);	// diagnostic 0x40 = functional
			r = s6d2aa0x01_read(dssdev, 0x52, data, 1);	// brightness
			r = s6d2aa0x01_read(dssdev, 0x56, data, 1);	// adaptive brightness
			r = s6d2aa0x01_read(dssdev, 0x5f, data, 1);	// CABC minimum
			r = s6d2aa0x01_read(dssdev, 0xa1, data, 5);	// should end in 0xff and be Supplier ID and Effective Data (i.e. some serial number)
			r = s6d2aa0x01_read(dssdev, 0xda, data, 1);	// ID1
			r = s6d2aa0x01_read(dssdev, 0xdb, data, 1);	// ID2
			r = s6d2aa0x01_read(dssdev, 0xdc, data, 1);	// ID3
			dsi_bus_unlock(dssdev);
		}
		mutex_unlock(&lg_d->lock);
		return r < 0 ? r : count;
	}
	if(strncmp(buf, "test", 4) == 0)
		{
		mutex_lock(&lg_d->lock);
		if (lg_d->enabled) {
			dsi_bus_lock(dssdev);
			r = s6d2aa0x01_write_sequence(dssdev, test_image, ARRAY_SIZE(test_image));
			dsi_bus_unlock(dssdev);
		}
		mutex_unlock(&lg_d->lock);
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

	mutex_lock(&lg_d->lock);
	if (lg_d->enabled) {
		dsi_bus_lock(dssdev);

		if(read)
			r = s6d2aa0x01_read(dssdev, data[0], &data[1], argc-1);
		else
			r = s6d2aa0x01_write(dssdev, data, argc);

		dsi_bus_unlock(dssdev);
	} else
		r=-EIO;	// not enabled
	mutex_unlock(&lg_d->lock);

	return r < 0 ? r : count;
}

static ssize_t show_dcs(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "XX XX ... | XX r ... | test | status | start | stop | reset | noreset | regulator | noregulator\n");
}

static DEVICE_ATTR(dcs, S_IWUSR | S_IRUGO,
				   show_dcs, set_dcs);

static struct attribute *s6d2aa0x01_attributes[] = {
	&dev_attr_dcs.attr,
	NULL
};

static const struct attribute_group s6d2aa0x01_attr_group = {
	.attrs = s6d2aa0x01_attributes,
};


static int s6d2aa0x01_probe_of(struct omap_dss_device *dssdev,
		struct s6d2aa0x01_data *lg_d)
{
	struct device_node *node = dssdev->dev.of_node;
	struct property *prop;
	u32 lane_arr[10];
	int len, i, num_pins, r;
	printk("dsi: s6d2aa0x01_probe_of()\n");
	r = of_get_gpio(node, 0);
	if (!gpio_is_valid(r)) {
		dev_err(&dssdev->dev, "failed to parse reset gpio\n");
		return r;
	}
	lg_d->reset_gpio = r;

	r = of_get_gpio(node, 1);
	if (!gpio_is_valid(r)) {
		dev_err(&dssdev->dev, "failed to parse regulator gpio\n");
		return r;
	}
	lg_d->regulator_gpio = r;

	prop = of_find_property(node, "lanes", &len);
	if (prop == NULL) {
		dev_err(&dssdev->dev, "failed to find lane data\n");
		return -EINVAL;
	}

	num_pins = len / sizeof(u32);

	if (num_pins < 4 || num_pins % 2 != 0
			|| num_pins > ARRAY_SIZE(lane_arr)) {
		dev_err(&dssdev->dev, "bad number of lanes\n");
		return -EINVAL;
	}

	r = of_property_read_u32_array(node, "lanes", lane_arr, num_pins);
	if (r) {
		dev_err(&dssdev->dev, "failed to read lane data\n");
		return r;
	}

	lg_d->pin_config.num_pins = num_pins;
	for (i = 0; i < num_pins; ++i)
		lg_d->pin_config.pins[i] = (int)lane_arr[i];

	return 0;
}

static int s6d2aa0x01_probe(struct omap_dss_device *dssdev)
{
	struct device_node *node = dssdev->dev.of_node;
	struct backlight_properties props;
	struct s6d2aa0x01_data *lg_d;
	int r;

	printk("dsi: s6d2aa0x01_probe()\n");
	dev_dbg(&dssdev->dev, "s6d2aa0x01_probe\n");

	if (node == NULL) {
		dev_err(&dssdev->dev, "no device tree data!\n");
		return -EINVAL;
	}

	lg_d = devm_kzalloc(&dssdev->dev, sizeof(*lg_d), GFP_KERNEL);
	if (!lg_d)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, lg_d);
	lg_d->dssdev = dssdev;

	r = s6d2aa0x01_probe_of(dssdev, lg_d);
	if (r)
		return r;

	dssdev->caps = 0;
	dssdev->panel.timings = s6d2aa0x01_timings;
	dssdev->panel.dsi_pix_fmt = OMAP_DSS_DSI_FMT_RGB888;

	mutex_init(&lg_d->lock);

	r = devm_gpio_request_one(&dssdev->dev, lg_d->reset_gpio,
			GPIOF_DIR_OUT, "lcd reset");
	if (r) {
		dev_err(&dssdev->dev, "failed to request reset gpio\n");
		return r;
	}

	r = devm_gpio_request_one(&dssdev->dev, lg_d->regulator_gpio,
							  GPIOF_DIR_OUT, "lcd DC/DC regulator");
	if (r) {
		dev_err(&dssdev->dev, "failed to request regulator gpio\n");
		return r;
	}

	/* Register DSI backlight control */
	memset(&props, 0, sizeof(struct backlight_properties));

	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	props.brightness = 255;

	lg_d->bl = props.brightness;

	lg_d->bldev = backlight_device_register("s6d2aa0x01", &dssdev->dev, dssdev,
			&s6d2aa0x01_backlight_ops, &props);
	if (IS_ERR(lg_d->bldev))
		return PTR_ERR(lg_d->bldev);

	r = omap_dsi_request_vc(dssdev, &lg_d->pixel_channel);
	if (r) {
		dev_err(&dssdev->dev,
				"failed to request pixel update channel\n");
		goto err_req_pix_vc;
	}

	r = omap_dsi_set_vc_id(dssdev, lg_d->pixel_channel, 0);
	if (r) {
		dev_err(&dssdev->dev,
				"failed to set VC_ID for pixel data virtual channel\n");
		goto err_set_pix_vc_id;
	}

	r = omap_dsi_request_vc(dssdev, &lg_d->config_channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to request config channel\n");
		goto err_req_cfg_vc;
	}

	r = omap_dsi_set_vc_id(dssdev, lg_d->config_channel, 0);
	if (r) {
		dev_err(&dssdev->dev,
				"failed to set VC_ID for config virtual channel\n");
		goto err_set_cfg_vc_id;
	}

	/* Register sysfs hooks */
	r = sysfs_create_group(&dssdev->dev.kobj, &s6d2aa0x01_attr_group);
	if (r)
		goto err_set_cfg_vc_id;

	return 0;

err_set_cfg_vc_id:
	omap_dsi_release_vc(dssdev, lg_d->config_channel);
err_req_cfg_vc:
err_set_pix_vc_id:
	omap_dsi_release_vc(dssdev, lg_d->pixel_channel);
err_req_pix_vc:
	backlight_device_unregister(lg_d->bldev);

	return r;
}

static void s6d2aa0x01_remove(struct omap_dss_device *dssdev)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);

	printk("dsi: s6d2aa0x01_remove()\n");
	omap_dsi_release_vc(dssdev, lg_d->config_channel);
	omap_dsi_release_vc(dssdev, lg_d->pixel_channel);

	sysfs_remove_group(&dssdev->dev.kobj, &s6d2aa0x01_attr_group);

	backlight_device_unregister(lg_d->bldev);
}

static int s6d2aa0x01_power_on(struct omap_dss_device *dssdev)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r;
	printk("dsi: s6d2aa0x01_power_on()\n");

	s6d2aa0x01_reset(dssdev, 0);	// activate reset

	r = omapdss_dsi_configure_pins(dssdev, &lg_d->pin_config);
	if (r) {
		dev_err(&dssdev->dev, "failed to configure DSI pins\n");
		goto err0;
	}

	omapdss_dsi_set_pixel_format(dssdev, S6D2AA0X01_PIXELFORMAT);
	omapdss_dsi_set_timings(dssdev, &s6d2aa0x01_timings);
	omapdss_dsi_set_videomode_timings(dssdev, &vm_data);
	omapdss_dsi_set_operation_mode(dssdev, OMAP_DSS_DSI_VIDEO_MODE);

	printk("dsi:    hs clock = %d Hz\n", S6D2AA0X01_HS_CLOCK);
	printk("dsi:    lp clock = %d Hz\n", S6D2AA0X01_LP_CLOCK);
	printk("dsi: pixel clock = %d Hz\n", 1000*s6d2aa0x01_timings.pixel_clock);
	r = omapdss_dsi_set_clocks(dssdev, S6D2AA0X01_HS_CLOCK, S6D2AA0X01_LP_CLOCK);
	if (r) {
		dev_err(&dssdev->dev, "failed to set HS and LP clocks\n");
		goto err0;
	}

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	s6d2aa0x01_regulator(dssdev, 1);	// switch power on
	msleep(50);

	s6d2aa0x01_reset(dssdev, 1);	// release reset
	msleep(10);

#if 1
	omapdss_dsi_vc_enable_hs(dssdev, lg_d->pixel_channel, true);
#endif
#if 1
	r = s6d2aa0x01_write_sequence(dssdev, init_seq, ARRAY_SIZE(init_seq));
	if (r) {
		dev_err(&dssdev->dev, "failed to configure panel\n");
		goto err;
	}
#endif
	msleep(20);

#if 1
	r = s6d2aa0x01_update_brightness(dssdev, lg_d->bl);
	if (r)
		goto err;
#endif
#if 1
	r = s6d2aa0x01_write_sequence(dssdev, sleep_out, ARRAY_SIZE(sleep_out));
	if (r)
		goto err;
#endif
#if 1
	dsi_enable_video_output(dssdev, lg_d->pixel_channel);

	msleep(120);
#endif
#if 0
	r = s6d2aa0x01_write_sequence(dssdev, display_on, ARRAY_SIZE(display_on));
	if (r)
		goto err;
#endif
	lg_d->enabled = true;
	printk("dsi: powered on()\n");

	return r;
err:
	printk("dsi: power on error\n");
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	omapdss_dsi_display_disable(dssdev, false, false);
	mdelay(10);
	s6d2aa0x01_reset(dssdev, 0);	// activate reset
	s6d2aa0x01_regulator(dssdev, 0);	// switch power off
	mdelay(20);

err0:
	return r;
}

// we don't have a sophisticated power management (sending the panel to power off)
// we simply stop the video stream and assert the RESET
// please note that we don't/can't switch off the VCCIO

static void s6d2aa0x01_power_off(struct omap_dss_device *dssdev)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r;

	printk("dsi: s6d2aa0x01_power_off()\n");

	r = s6d2aa0x01_write_sequence(dssdev, display_off, ARRAY_SIZE(display_off));
	msleep(40);
	
	r = s6d2aa0x01_write_sequence(dssdev, power_off_seq, ARRAY_SIZE(power_off_seq));

	r = s6d2aa0x01_write_sequence(dssdev, sleep_in, ARRAY_SIZE(sleep_in));
	msleep(120);
	
	lg_d->enabled = 0;
	dsi_disable_video_output(dssdev, lg_d->pixel_channel);
	omapdss_dsi_display_disable(dssdev, false, false);
	mdelay(10);
	s6d2aa0x01_reset(dssdev, 0);	// activate reset
	s6d2aa0x01_regulator(dssdev, 0);	// switch power off - after stopping video stream
	mdelay(20);
	/* here we can also power off IOVCC */
}

// this driver API has been simplified in later (Linux 3.12ff) DSS implementations
// so there is no point to improve this e.g. to send the display to sleep mode

static int s6d2aa0x01_start(struct omap_dss_device *dssdev)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	printk("dsi: s6d2aa0x01_start()\n");
	mutex_lock(&lg_d->lock);

	dsi_bus_lock(dssdev);

	r = s6d2aa0x01_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		dev_err(&dssdev->dev, "enable failed\n");
	else
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&lg_d->lock);

	return r;
}

static void s6d2aa0x01_stop(struct omap_dss_device *dssdev)
{
	struct s6d2aa0x01_data *lg_d = dev_get_drvdata(&dssdev->dev);

	printk("dsi: s6d2aa0x01_stop()\n");
	mutex_lock(&lg_d->lock);

	dsi_bus_lock(dssdev);

	s6d2aa0x01_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	mutex_unlock(&lg_d->lock);
}

static void s6d2aa0x01_disable(struct omap_dss_device *dssdev)
{
	printk("dsi: s6d2aa0x01_disable()\n");
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		s6d2aa0x01_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int s6d2aa0x01_enable(struct omap_dss_device *dssdev)
{
	printk("dsi: s6d2aa0x01_enable()\n");
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return s6d2aa0x01_start(dssdev);
}

#if defined(CONFIG_OF)
static const struct of_device_id s6d2aa0x01_of_match[] = {
	{
		.compatible = "lg,lh500wf1",
	},
	{},
};

MODULE_DEVICE_TABLE(of, s6d2aa0x01_of_match);
#else
#define dss_of_match NULL
#endif

static struct omap_dss_driver s6d2aa0x01_driver = {
	.probe = s6d2aa0x01_probe,
	.remove = s6d2aa0x01_remove,

	.enable = s6d2aa0x01_enable,
	.disable = s6d2aa0x01_disable,

	.get_resolution = s6d2aa0x01_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings = s6d2aa0x01_get_timings,
	.set_timings = s6d2aa0x01_set_timings,
	.check_timings = s6d2aa0x01_check_timings,

	.driver = {
		.name = "s6d2aa0x01",
		.owner = THIS_MODULE,
		.of_match_table = s6d2aa0x01_of_match,
	},
};

static int __init s6d2aa0x01_init(void)
{
	omap_dss_register_driver(&s6d2aa0x01_driver);
	return 0;
}

static void __exit s6d2aa0x01_exit(void)
{
	omap_dss_unregister_driver(&s6d2aa0x01_driver);
}

module_init(s6d2aa0x01_init);
module_exit(s6d2aa0x01_exit);

MODULE_DESCRIPTION("s6d2aa0x01 driver");
MODULE_LICENSE("GPL");