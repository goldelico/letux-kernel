/*
 * A V4L2 driver for OmniVision sc2232h cameras.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>

#include <isp-sensor.h>

#define SC2232H_CHIP_ID_H       (0xcb)
#define SC2232H_CHIP_ID_M       (0x07)
#define SC2232H_CHIP_ID_L       (0x01)
#define SC2232H_REG_END		0xffff
#define SC2232H_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct sc2232h_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct sc2232h_gpio {
	int pin;
	int active_level;
};

struct sc2232h_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct sc2232h_win_size *win;
	struct sc2232h_gpio reset;

};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut sc2232h_again_lut[] = {

	{0x10, 0},
	{0x11, 5731},
	{0x12, 11136},
	{0x13, 16248},
	{0x14, 21097},
	{0x15, 25710},
	{0x16, 30109},
	{0x17, 34312},
	{0x18, 38336},
	{0x19, 42195},
	{0x1a, 45904},
	{0x1b, 49472},
	{0x1c, 52910},
	{0x1d, 56228},
	{0x1e, 59433},
	{0x1f, 62534},
	{0x110,	65536},
	{0x111,	71267},
	{0x112,	76672},
	{0x113,	81784},
	{0x114,	86633},
	{0x115,	91246},
	{0x116,	95645},
	{0x117,	99848},
	{0x118, 103872},
	{0x119,	107731},
	{0x11a,	111440},
	{0x11b,	115008},
	{0x11c,	118446},
	{0x11d,	121764},
	{0x11e,	124969},
	{0x11f,	128070},
	{0x310, 131072},
	{0x311,	136803},
	{0x312,	142208},
	{0x313,	147320},
	{0x314,	152169},
	{0x315,	156782},
	{0x316,	161181},
	{0x317,	165384},
	{0x318,	169408},
	{0x319,	173267},
	{0x31a,	176976},
	{0x31b,	180544},
	{0x31c,	183982},
	{0x31d,	187300},
	{0x31e,	190505},
	{0x31f,	193606},
	{0x710, 196608},
	{0x711,	202339},
	{0x712,	207744},
	{0x713,	212856},
	{0x714,	217705},
	{0x715,	222318},
	{0x716,	226717},
	{0x717,	230920},
	{0x718,	234944},
	{0x719,	238803},
	{0x71a,	242512},
	{0x71b,	246080},
	{0x71c,	249518},
	{0x71d,	252836},
	{0x71e,	256041},
	/* {0x71f, 259142}, */
};

static inline struct sc2232h_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc2232h_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sc2232h_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list sc2232h_init_regs_1920_1080_25fps_mipi[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3034, 0x81},
	{0x3039, 0xd4},
	{0x3624, 0x08},
	{0x337f, 0x03},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3366, 0x7c},
	{0x3302, 0x1f},
	{0x3907, 0x00},
	{0x3908, 0x11},
	{0x335e, 0x01},
	{0x335f, 0x03},
	{0x337c, 0x04},
	{0x337d, 0x06},
	{0x33a0, 0x05},
	{0x3633, 0x4f},
	{0x3622, 0x06},
	{0x3631, 0x84},
	{0x366e, 0x08},
	{0x3326, 0x00},
	{0x3303, 0x20},
	{0x3638, 0x1f},
	{0x3636, 0x25},
	{0x3625, 0x02},
	{0x331b, 0x83},
	{0x3333, 0x30},
	{0x3635, 0xa0},
	{0x363c, 0x05},
	{0x3038, 0xff},
	{0x3639, 0x09},
	{0x3621, 0x28},
	{0x3211, 0x0c},
	{0x3320, 0x01},
	{0x331e, 0x19},
	{0x3620, 0x28},
	{0x3309, 0x60},
	{0x331f, 0x59},
	{0x3308, 0x10},
	{0x3f00, 0x07},
	{0x33aa, 0x10},
	{0x3677, 0x86},
	{0x3678, 0x88},
	{0x3679, 0x88},
	{0x367e, 0x08},
	{0x367f, 0x28},
	{0x3670, 0x0c},
	{0x3690, 0x33},
	{0x3691, 0x11},
	{0x3692, 0x43},
	{0x369c, 0x08},
	{0x369d, 0x28},
	{0x360f, 0x01},
	{0x3671, 0xc6},
	{0x3672, 0x06},
	{0x3673, 0x16},
	{0x367a, 0x28},
	{0x367b, 0x3f},
	{0x3221, 0x80},
	{0x3222, 0x29},
	{0x3901, 0x02},
	{0x3905, 0x98},
	{0x3e1e, 0x34},
	{0x3900, 0x19},
	{0x391d, 0x04},
	{0x391e, 0x00},
	{0x3641, 0x01},
	{0x3213, 0x04},
	{0x3614, 0x80},
	{0x363a, 0x9f},
	{0x3630, 0x9c},
	{0x3306, 0x48},
	{0x330b, 0xcd},
	{0x3018, 0x33},
	{0x3031, 0x0a},
	{0x3037, 0x20},
	{0x3001, 0xfe},
	{0x4603, 0x00},
	{0x4827, 0x48},
	{0x301c, 0x78},
	{0x4809, 0x01},
	{0x3314, 0x04},
	{0x3933, 0x0a},
	{0x3934, 0x18},
	{0x3940, 0x60},
	{0x3942, 0x02},
	{0x3943, 0x1f},
	{0x3960, 0xba},
	{0x3961, 0xae},
	{0x3966, 0xba},
	{0x3980, 0xa0},
	{0x3981, 0x40},
	{0x3982, 0x18},
	{0x3903, 0x08},
	{0x3984, 0x08},
	{0x3985, 0x20},
	{0x3986, 0x50},
	{0x3987, 0xb0},
	{0x3988, 0x08},
	{0x3989, 0x10},
	{0x398a, 0x20},
	{0x398b, 0x30},
	{0x398c, 0x60},
	{0x398d, 0x20},
	{0x398e, 0x10},
	{0x398f, 0x08},
	{0x3990, 0x60},
	{0x3991, 0x24},
	{0x3992, 0x15},
	{0x3993, 0x08},
	{0x3994, 0x0a},
	{0x3995, 0x20},
	{0x3996, 0x38},
	{0x3997, 0xa0},
	{0x3998, 0x08},
	{0x3999, 0x10},
	{0x399a, 0x18},
	{0x399b, 0x30},
	{0x399c, 0x30},
	{0x399d, 0x18},
	{0x399e, 0x10},
	{0x399f, 0x08},
	{0x3637, 0x55},
	{0x366f, 0x2c},
	{0x5000, 0x06},
	{0x5780, 0x7f},
	{0x5781, 0x04},
	{0x5782, 0x03},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x18},
	{0x5786, 0x10},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x57a0, 0x00},
	{0x57a1, 0x71},
	{0x57a2, 0x01},
	{0x57a3, 0xf1},
	{0x395e, 0xc0},
	{0x3962, 0x88},
	{0x303a, 0xb3},
	{0x303c, 0x0e},
	{0x3035, 0x9b},
	{0x320c, 0x08},
	{0x320d, 0x20},
	{0x320e, 0x05},
	{0x320f, 0xdc},/*25fps*/
	{0x3f04, 0x03},
	{0x3f05, 0xec},
	{0x3235, 0x09},
	{0x3236, 0xc2},
	{0x4837, 0x33},
	{0x3802, 0x00},
	{0x3963, 0x80},
	{0x303b, 0x16},
	{0x301f, 0x05},
	{0x363b, 0x26},
	{0x3902, 0xc5},
	{0x3e00, 0x00},
	{0x3e01, 0x9c},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x10},
	{0x3301, 0x12},
	{0x3632, 0x08},
	{0x3034, 0x01},
	{0x3039, 0x24},

	{SC2232H_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2232h_init_regs_1920_1080_25fps_dvp[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3034, 0x81},
	{0x3039, 0xd4},
	{0x3018, 0x1f},
	{0x3019, 0xff},
	{0x301c, 0xb4},
	{0x3624, 0x08},
	{0x337f, 0x03},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3366, 0x7c},
	{0x3302, 0x1f},
	{0x303f, 0x81},
	{0x3907, 0x00},
	{0x3908, 0x11},
	{0x335e, 0x01},
	{0x335f, 0x03},
	{0x337c, 0x04},
	{0x337d, 0x06},
	{0x33a0, 0x05},
	{0x3633, 0x4f},
	{0x3622, 0x06},
	{0x3631, 0x84},
	{0x366e, 0x08},
	{0x3326, 0x00},
	{0x3303, 0x20},
	{0x3638, 0x1f},
	{0x3636, 0x25},
	{0x3625, 0x02},
	{0x331b, 0x83},
	{0x3333, 0x30},
	{0x3635, 0xa0},
	{0x363c, 0x05},
	{0x3038, 0xff},
	{0x3639, 0x09},
	{0x3621, 0x28},
	{0x3211, 0x0c},
	{0x3320, 0x01},
	{0x331e, 0x19},
	{0x3620, 0x28},
	{0x3309, 0x60},
	{0x331f, 0x59},
	{0x3308, 0x10},
	{0x3f00, 0x07},
	{0x33aa, 0x10},
	{0x3677, 0x86},
	{0x3678, 0x88},
	{0x3679, 0x88},
	{0x367e, 0x08},
	{0x367f, 0x28},
	{0x3670, 0x0c},
	{0x3690, 0x33},
	{0x3691, 0x11},
	{0x3692, 0x43},
	{0x369c, 0x08},
	{0x369d, 0x28},
	{0x360f, 0x01},
	{0x3671, 0xc6},
	{0x3672, 0x06},
	{0x3673, 0x16},
	{0x367a, 0x28},
	{0x367b, 0x3f},
	{0x3221, 0x80},
	{0x3222, 0x29},
	{0x3901, 0x02},
	{0x3905, 0x98},
	{0x3e1e, 0x34},
	{0x3900, 0x19},
	{0x391d, 0x04},
	{0x391e, 0x00},
	{0x3641, 0x01},
	{0x3213, 0x04},
	{0x3614, 0x80},
	{0x363a, 0x9f},
	{0x3630, 0x9c},
	{0x3306, 0x48},
	{0x330b, 0xcd},
	{0x3314, 0x04},
	{0x3933, 0x0a},
	{0x3934, 0x18},
	{0x3940, 0x60},
	{0x3942, 0x02},
	{0x3943, 0x1f},
	{0x3960, 0xba},
	{0x3961, 0xae},
	{0x3966, 0xba},
	{0x3980, 0xa0},
	{0x3981, 0x40},
	{0x3982, 0x18},
	{0x3903, 0x08},
	{0x3984, 0x08},
	{0x3985, 0x20},
	{0x3986, 0x50},
	{0x3987, 0xb0},
	{0x3988, 0x08},
	{0x3989, 0x10},
	{0x398a, 0x20},
	{0x398b, 0x30},
	{0x398c, 0x60},
	{0x398d, 0x20},
	{0x398e, 0x10},
	{0x398f, 0x08},
	{0x3990, 0x60},
	{0x3991, 0x24},
	{0x3992, 0x15},
	{0x3993, 0x08},
	{0x3994, 0x0a},
	{0x3995, 0x20},
	{0x3996, 0x38},
	{0x3997, 0xa0},
	{0x3998, 0x08},
	{0x3999, 0x10},
	{0x399a, 0x18},
	{0x399b, 0x30},
	{0x399c, 0x30},
	{0x399d, 0x18},
	{0x399e, 0x10},
	{0x399f, 0x08},
	{0x3637, 0x55},
	{0x366f, 0x2c},
	{0x5000, 0x06},
	{0x5780, 0x7f},
	{0x5781, 0x04},
	{0x5782, 0x03},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x18},
	{0x5786, 0x10},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x57a0, 0x00},
	{0x57a1, 0x71},
	{0x57a2, 0x01},
	{0x57a3, 0xf1},
	{0x395e, 0xc0},
	{0x3962, 0x88},
	{0x303a, 0xb3},
	{0x303c, 0x0e},
	{0x3035, 0x9b},
	{0x320c, 0x08},
	{0x320d, 0x20},
	{0x320e, 0x05},
	{0x320f, 0xd0},/*vts for 25fps*/
	{0x3f04, 0x03},
	{0x3f05, 0xec},
	{0x3235, 0x09},
	{0x3236, 0xc2},
	{0x3802, 0x00},
	{0x3963, 0x80},
	{0x303b, 0x16},
	{0x301f, 0x03},
	{0x363b, 0x26},
	{0x3902, 0xc5},
	{0x3e00, 0x00},
//	{0x3e01, 0x9c},
	{0x3e01, 0xbc},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x10},
	{0x3d08, 0x00},//PCLK polarity
	{0x3301, 0x12},
	{0x3632, 0x08},
	{0x3034, 0x01},
	{0x3039, 0x24},

	{SC2232H_REG_END, 0x00},	/* END MARKER */
};
/*
 * the part of driver was fixed.
 */

static struct regval_list sc2232h_stream_on[] = {
	{0x0100, 0x01},
	{SC2232H_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2232h_stream_off[] = {
	{0x0100, 0x00},
	{SC2232H_REG_END, 0x00},	/* END MARKER */
};

static int sc2232h_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct sc2232h_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {(reg>>8)&0xff, reg&0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int sc2232h_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct sc2232h_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3] = {(reg>>8)&0xff, reg&0xff, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int sc2232h_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != SC2232H_REG_END) {
		if (vals->reg_num == SC2232H_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc2232h_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int sc2232h_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != SC2232H_REG_END) {
		if (vals->reg_num == SC2232H_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc2232h_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static int sc2232h_reset(struct v4l2_subdev *sd, u32 val)
{
	struct sc2232h_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}


static int sc2232h_init(struct v4l2_subdev *sd, u32 val)
{
	struct sc2232h_info *info = to_state(sd);
	int ret = 0;

	ret = sc2232h_write_array(sd, info->win->regs);

	return ret;
}



static int sc2232h_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = sc2232h_read(sd, 0x3107, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC2232H_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = sc2232h_read(sd, 0x3108, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC2232H_CHIP_ID_M)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	ret = sc2232h_read(sd, 0x3109, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC2232H_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;
	return 0;
}


static struct sc2232h_win_size sc2232h_win_sizes[] = {
	{
		.width		= 1920,
		.height		= 1080,
		.sensor_info.fps		= 25 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2232h_init_regs_1920_1080_25fps_dvp,
	},
};

static int sc2232h_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_SC2232H_FMTS)
		return -EINVAL;

	code->code = sc2232h_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int sc2232h_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct sc2232h_format_struct *ovfmt;
	struct sc2232h_win_size *wsize;
	struct sc2232h_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int sc2232h_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct sc2232h_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if(!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = info->win->width;
	fmt->height = info->win->height;
	fmt->code = info->win->mbus_code;
	fmt->colorspace = info->win->colorspace;
	*(unsigned int *)fmt->reserved = &info->win->sensor_info;


//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int sc2232h_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2232h_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2232h_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2232h_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int sc2232h_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int sc2232h_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc2232h_again_lut); i++) {
		lut = &sc2232h_again_lut[i];

		if(gain <= lut->gain) {
			return lut->value;
		}
	}

	/*last value.*/
	return lut->value;
}

static int regval_to_again(unsigned int regval)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc2232h_again_lut); i++) {
		lut = &sc2232h_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return 0;
}

static int sc2232h_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct sc2232h_info *info = to_state(sd);
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret += sc2232h_read(sd, 0x3509, &v);

	reg_val |= v;
	ret += sc2232h_read(sd, 0x3508, &v);
	reg_val |= v << 8;


	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int sc2232h_s_again(struct v4l2_subdev *sd, int value)
{
	struct sc2232h_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	int i;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}
	/* denoise logic */
	if (reg_value < 0x110) {
		sc2232h_write(sd, 0x3812, 0x00);
		sc2232h_write(sd, 0x3301, 0x12);
		sc2232h_write(sd, 0x3632, 0x08);
		sc2232h_write(sd, 0x3812, 0x30);
	} else if (reg_value >= 0x110 && reg_value < 0x310) {
		sc2232h_write(sd, 0x3812, 0x00);
		sc2232h_write(sd, 0x3301, 0x20);
		sc2232h_write(sd, 0x3632, 0x08);
		sc2232h_write(sd, 0x3812, 0x30);
	} else if(reg_value >= 0x310&&reg_value < 0x710) {
		sc2232h_write(sd, 0x3812, 0x00);
		sc2232h_write(sd, 0x3301, 0x28);
		sc2232h_write(sd, 0x3632, 0x08);
		sc2232h_write(sd, 0x3812, 0x30);
	} else if(reg_value >= 0x710&&reg_value <= 0x71e) {
		sc2232h_write(sd, 0x3812, 0x00);
		sc2232h_write(sd, 0x3301, 0x64);
		sc2232h_write(sd, 0x3632, 0x08);
		sc2232h_write(sd, 0x3812, 0x30);
	} else { //may be flick
		sc2232h_write(sd, 0x3812, 0x00);
		sc2232h_write(sd, 0x3301, 0x64);
		sc2232h_write(sd, 0x3632, 0x48);
		sc2232h_write(sd, 0x3812, 0x30);
	}
	ret = sc2232h_write(sd, 0x3e09, (unsigned char)(reg_value & 0xff));
	ret += sc2232h_write(sd, 0x3e08, (unsigned char)((reg_value >> 8 << 2) | 0x03));
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Tweak autogain.
 */
static int sc2232h_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2232h_s_exp(struct v4l2_subdev *sd, int value)
{
	struct sc2232h_info *info = to_state(sd);
	int ret = 0;

	value *= 2;
	ret = sc2232h_write(sd, 0x3e00, (unsigned char)((value >> 12) & 0x0f));
	ret += sc2232h_write(sd, 0x3e01, (unsigned char)((value >> 4) & 0xff));
	ret += sc2232h_write(sd, 0x3e02, (unsigned char)((value & 0x0f) << 4));

	if (value < 250) {
		ret += sc2232h_write(sd, 0x3314, 0x14);
	}
	else if(value > 450){
		ret += sc2232h_write(sd, 0x3314, 0x04);
	}

	if (ret < 0)
		return ret;

	return ret;
}

static int sc2232h_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc2232h_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return sc2232h_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc2232h_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int sc2232h_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc2232h_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sc2232h_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return sc2232h_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return sc2232h_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return sc2232h_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* sc2232h_s_gain turns off auto gain */
			return sc2232h_s_gain(sd, info->gain->val);
		}
		return sc2232h_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return sc2232h_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc2232h_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sc2232h_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc2232h_ctrl_ops = {
	.s_ctrl = sc2232h_s_ctrl,
	.g_volatile_ctrl = sc2232h_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc2232h_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = sc2232h_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int sc2232h_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	sc2232h_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int sc2232h_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc2232h_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ret = sc2232h_write_array(sd, sc2232h_stream_on);
		pr_debug("sc2232h stream on\n");

	}
	else {
		ret = sc2232h_write_array(sd, sc2232h_stream_off);
		pr_debug("sc2232h stream off\n");
	}
	return ret;
}

int sc2232h_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct sc2232h_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sc2232h_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sc2232h_g_register,
	.s_register = sc2232h_s_register,
#endif

};

static const struct v4l2_subdev_video_ops sc2232h_video_ops = {
	.s_stream = sc2232h_s_stream,
	.g_frame_interval = sc2232h_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc2232h_pad_ops = {
	//.enum_frame_interval = sc2232h_enum_frame_interval,
	//.num_frame_size = sc2232h_enum_frame_size,
	//.enum_mbus_code = sc2232h_enum_mbus_code,
	.set_fmt = sc2232h_set_fmt,
	.get_fmt = sc2232h_get_fmt,
};

static const struct v4l2_subdev_ops sc2232h_ops = {
	.core = &sc2232h_core_ops,
	.video = &sc2232h_video_ops,
	.pad = &sc2232h_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int sc2232h_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct sc2232h_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	v4l2_i2c_subdev_init(sd, client, &sc2232h_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sc2232h_reset(sd, 1);

#if 1
	/* Make sure it's an sc2232h */
	ret = sc2232h_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an sc2232h chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 261773, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &sc2232h_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &sc2232h_win_sizes[0];
	sc2232h_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "sc2232h Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	return ret;
}


static int sc2232h_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2232h_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	return 0;
}

static const struct i2c_device_id sc2232h_id[] = {
	{ "sc2232h", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2232h_id);

static const struct of_device_id sc2232h_of_match[] = {
	{.compatible = "ingenic,sc2232h", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);


static struct i2c_driver sc2232h_driver = {
	.driver = {
		.name	= "sc2232h",
		.of_match_table = of_match_ptr(sc2232h_of_match),
	},
	.probe		= sc2232h_probe,
	.remove		= sc2232h_remove,
	.id_table	= sc2232h_id,
};

module_i2c_driver(sc2232h_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision sc2232h sensors");
MODULE_LICENSE("GPL");
