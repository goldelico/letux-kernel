/*
 * A V4L2 driver for Himax HM2140 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
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
#include <media/v4l2-clk.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>

#include <isp-sensor.h>

#define HM2140_CHIP_ID_H	(0x21)
#define HM2140_CHIP_ID_L	(0x40)
#define HM2140_REG_END		0xff
#define HM2140_REG_DELAY	0x00
#define HM2140_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct hm2140_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct hm2140_gpio {
	int pin;
	int active_level;
};

struct hm2140_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct hm2140_win_size *win;

	struct hm2140_gpio pwdn;
	struct hm2140_gpio sync;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};

struct again_lut hm2140_again_lut[] ={
	{0x00, 0},
	{0x01, 4096},
	{0x02, 8192},
	{0x03, 12288},
	{0x04, 16384},
	{0x05, 20480},
	{0x06, 24576},
	{0x07, 28672},
	{0x08, 32768},
	{0x09, 36864},
	{0x0A, 40960},
	{0x0B, 45056},
	{0x0C, 49152},
	{0x0D, 53248},
	{0x0E, 57344},
	{0x0F, 61440},
	{0x10, 65536},
	{0x12, 69632},
	{0x14, 73728},
	{0x16, 77824},
	{0x18, 81920},
	{0x1A, 86016},
	{0x1C, 90112},
	{0x1E, 94208},
	{0x20, 98304},
	{0x22, 102400},
	{0x24, 106496},
	{0x26, 110592},
	{0x28, 114688},
	{0x2A, 118784},
	{0x2C, 122880},
	{0x2E, 126976},
	{0x30, 131072},
	{0x34, 135168},
	{0x38, 139264},
	{0x3C, 143360},
	{0x40, 147456},
	{0x44, 151552},
	{0x48, 155648},
	{0x4C, 159744},
	{0x50, 163840},
	{0x54, 167936},
	{0x58, 172032},
	{0x5C, 176128},
	{0x60, 180224},
	{0x64, 184320},
	{0x68, 188416},
	{0x6C, 192512},
	{0x70, 196608},
};

static inline struct hm2140_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hm2140_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct hm2140_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list hm2140_init_regs_1920_1080_30fps_MIPI[] = {
    {0x0103, 0x00},
    {HM2140_REG_DELAY, 5},
    {0x5227, 0x74},
    {0x030B, 0x11},
    {HM2140_REG_DELAY, 100},
    {0x0307, 0x00},
    {0x0309, 0x00},
    {0x030A, 0x0A},
    {0x030D, 0x03},
    {0x030F, 0x2B},
    {0x5235, 0xA4},
    {0x5236, 0x92},
    {0x5237, 0x23},
    {0x5238, 0xDC},
    {0x5239, 0x01},
    {0x0100, 0x02},
    {0x4001, 0x00},
    {0x4002, 0x2B},
    {0x0101, 0x00},
    {0x4026, 0x3B},
    {0x0202, 0x04},
    {0x0203, 0x92},
    {0x0340, 0x04},
    {0x0341, 0x94},
    {0x0342, 0x08},
    {0x0343, 0x98},
    {0x034c, 0x07},
    {0x034d, 0x80},
    {0x034e, 0x04},
    {0x034f, 0x38},
    {0x0350, 0x73},
    {0x5015, 0x33},
    {0x50DD, 0x01},
    {0x50CB, 0xA3},
    {0x5004, 0x40},
    {0x5005, 0x28},
    {0x504D, 0x7F},
    {0x504E, 0x00},
    {0x5040, 0x07},
    {0x5011, 0x00},
    {0x501D, 0x4C},
    {0x5011, 0x0B},
    {0x5012, 0x01},
    {0x5013, 0x03},
    {0x4131, 0x01},
    {0x5282, 0xFF},
    {0x5283, 0x07},
    {0x5010, 0x20},
    {0x4132, 0x20},
    {0x50D5, 0xE0},
    {0x50D7, 0x12},
    {0x50BB, 0x14},
    {0x50B7, 0x00},
    {0x50B8, 0x35},
    {0x50B9, 0x11},
    {0x50BA, 0x30},
    {0x50B3, 0x24},
    {0x50B4, 0x00},
    {0x50FA, 0x02},
    {0x509B, 0x01},
    {0x50AA, 0xFF},
    {0x50AB, 0x45},
    {0x509C, 0x00},
    {0x50AD, 0x0C},
    {0x5096, 0x00},
    {0x50A1, 0x12},
    {0x50AF, 0x21},
    {0x50A0, 0x11},
    {0x50A2, 0x21},
    {0x509D, 0x20},
    {0x50AC, 0x55},
    {0x50AE, 0x26},
    {0x509E, 0x03},
    {0x509F, 0x01},
    {0x5097, 0x12},
    {0x5099, 0x00},
    {0x50B5, 0x00},
    {0x50B6, 0x10},
    {0x5094, 0x08},
    {0x5200, 0x43},
    {0x5201, 0xC0},
    {0x5202, 0x00},
    {0x5203, 0x00},
    {0x5204, 0x00},
    {0x5205, 0x05},
    {0x5206, 0xA1},
    {0x5207, 0x01},
    {0x5208, 0x0A},
    {0x5209, 0x0C},
    {0x520A, 0x00},
    {0x520B, 0x45},
    {0x520C, 0x15},
    {0x520D, 0x40},
    {0x520E, 0x50},
    {0x520F, 0x10},
    {0x5214, 0x40},
    {0x5215, 0x14},
    {0x5216, 0x00},
    {0x5217, 0x02},
    {0x5218, 0x07},
    {0x521C, 0x00},
    {0x521E, 0x00},
    {0x522A, 0x3F},
    {0x522C, 0x00},
    {0x5230, 0x00},
    {0x5232, 0x05},
    {0x523A, 0x20},
    {0x523B, 0x34},
    {0x523C, 0x03},
    {0x523D, 0x00},
    {0x523E, 0x00},
    {0x523F, 0x70},
    {0x50E8, 0x16},
    {0x50E9, 0x00},
    {0x50EB, 0x0F},
    {0x4B11, 0x0F},
    {0x4B12, 0x0F},
    {0x4B31, 0x04},
    {0x4B3B, 0x02},
    {0x4B44, 0x80},
    {0x4B45, 0x00},
    {0x4B47, 0x00},
    {0x4B4E, 0x30},
    {0x4020, 0x20},
    {0x5100, 0x13},
    {0x5101, 0x2B},
    {0x5102, 0x3B},
    {0x5103, 0x4B},
    {0x5104, 0x5F},
    {0x5105, 0x6F},
    {0x5106, 0x7F},
    {0x5108, 0x00},
    {0x5109, 0x00},
    {0x510A, 0x00},
    {0x510B, 0x00},
    {0x510C, 0x00},
    {0x510D, 0x00},
    {0x510E, 0x00},
    {0x5110, 0x0E},
    {0x5111, 0x0E},
    {0x5112, 0x0E},
    {0x5113, 0x0E},
    {0x5114, 0x0E},
    {0x5115, 0x0E},
    {0x5116, 0x0E},
    {0x5118, 0x09},
    {0x5119, 0x09},
    {0x511A, 0x09},
    {0x511B, 0x09},
    {0x511C, 0x09},
    {0x511D, 0x09},
    {0x511E, 0x09},
    {0x5120, 0xEA},
    {0x5121, 0x6A},
    {0x5122, 0x6A},
    {0x5123, 0x6A},
    {0x5124, 0x6A},
    {0x5125, 0x6A},
    {0x5126, 0x6A},
    {0x5140, 0x0B},
    {0x5141, 0x1B},
    {0x5142, 0x2B},
    {0x5143, 0x3B},
    {0x5144, 0x4B},
    {0x5145, 0x5B},
    {0x5146, 0x6B},
    {0x5148, 0x02},
    {0x5149, 0x02},
    {0x514A, 0x02},
    {0x514B, 0x02},
    {0x514C, 0x02},
    {0x514D, 0x02},
    {0x514E, 0x02},
    {0x5150, 0x08},
    {0x5151, 0x08},
    {0x5152, 0x08},
    {0x5153, 0x08},
    {0x5154, 0x08},
    {0x5155, 0x08},
    {0x5156, 0x08},
    {0x5158, 0x02},
    {0x5159, 0x02},
    {0x515A, 0x02},
    {0x515B, 0x02},
    {0x515C, 0x02},
    {0x515D, 0x02},
    {0x515E, 0x02},
    {0x5160, 0x66},
    {0x5161, 0x66},
    {0x5162, 0x66},
    {0x5163, 0x66},
    {0x5164, 0x66},
    {0x5165, 0x66},
    {0x5166, 0x66},
    {0x5180, 0x00},
    {0x5189, 0x00},
    {0x5192, 0x00},
    {0x519B, 0x00},
    {0x51A4, 0x00},
    {0x51AD, 0x00},
    {0x51B6, 0x00},
    {0x51C0, 0x00},
    {0x5181, 0x00},
    {0x518A, 0x00},
    {0x5193, 0x00},
    {0x519C, 0x00},
    {0x51A5, 0x00},
    {0x51AE, 0x00},
    {0x51B7, 0x00},
    {0x51C1, 0x00},
    {0x5182, 0x85},
    {0x518B, 0x85},
    {0x5194, 0x85},
    {0x519D, 0x85},
    {0x51A6, 0x85},
    {0x51AF, 0x85},
    {0x51B8, 0x85},
    {0x51C2, 0x85},
    {0x5183, 0x52},
    {0x518C, 0x52},
    {0x5195, 0x52},
    {0x519E, 0x52},
    {0x51A7, 0x52},
    {0x51B0, 0x52},
    {0x51B9, 0x52},
    {0x51C3, 0x52},
    {0x5184, 0x00},
    {0x518D, 0x00},
    {0x5196, 0x08},
    {0x519F, 0x08},
    {0x51A8, 0x08},
    {0x51B1, 0x08},
    {0x51BA, 0x08},
    {0x51C4, 0x08},
    {0x5185, 0x73},
    {0x518E, 0x73},
    {0x5197, 0x73},
    {0x51A0, 0x73},
    {0x51A9, 0x73},
    {0x51B2, 0x73},
    {0x51BB, 0x73},
    {0x51C5, 0x73},
    {0x5186, 0x34},
    {0x518F, 0xA4},
    {0x5198, 0x34},
    {0x51A1, 0x34},
    {0x51AA, 0x34},
    {0x51B3, 0x3F},
    {0x51BC, 0x3F},
    {0x51C6, 0x3F},
    {0x5187, 0x40},
    {0x5190, 0x38},
    {0x5199, 0x20},
    {0x51A2, 0x08},
    {0x51AB, 0x04},
    {0x51B4, 0x04},
    {0x51BD, 0x02},
    {0x51C7, 0x01},
    {0x5188, 0x20},
    {0x5191, 0x40},
    {0x519A, 0x40},
    {0x51A3, 0x40},
    {0x51AC, 0x40},
    {0x51B5, 0x78},
    {0x51BE, 0x78},
    {0x51C8, 0x78},
    {0x51E1, 0x07},
    {0x51E3, 0x07},
    {0x51E5, 0x07},
    {0x51ED, 0x00},
    {0x51EE, 0x00},
    {0x4002, 0x2B},
    {0x3132, 0x00},
    {0x4024, 0x40},
    {0x5229, 0xFC},
    {0x4002, 0x2B},
    {0x3110, 0x03},
    {0x373D, 0x12},
    {0xBAA2, 0xC0},
    {0xBAA2, 0x40},
    {0xBA90, 0x01},
    {0xBA93, 0x02},
    {0x350D, 0x01},
    {0x3514, 0x00},
    {0x350C, 0x01},
    {0x3519, 0x00},
    {0x351A, 0x01},
    {0x351B, 0x1E},
    {0x351C, 0x90},
    {0x351E, 0x05},
    {0x351D, 0x05},
    {0x4B20, 0x8E},
    {0x4B18, 0x00},
    {0x4B3E, 0x00},
    {0x4B0E, 0x21},
#if 0 // test pattern
    {0x4026, 0x3a},
    {0x0601, 0x02},
    {0x3110, 0x01},
    {0x3735, 0x00},
    {0x5013, 0x00},
    {0x501d, 0x04},
    {0x4131, 0x00},
#endif
    {0x0104, 0x01},
    {0x0104, 0x00},
    {0x0100, 0x00},
    {HM2140_REG_END, 0x00}, /* END MARKER */
};


static struct regval_list hm2140_stream_on[] = {
	{0x0100, 0x01},
	{HM2140_REG_END, 0x00},
};

static struct regval_list hm2140_stream_off[] = {
	{0x0100, 0x00},
	{HM2140_REG_END, 0x00},
};

/* read a register */
static int hm2140_read(struct v4l2_subdev *sd, uint16_t reg, uint8_t *val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {(uint8_t)((reg & 0xFF00) >> 8), (uint8_t)((reg & 0x00FF))};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = buf,
		},
		[1] = {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = val,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a register */
static int hm2140_write(struct v4l2_subdev *sd, uint16_t reg, uint8_t val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3];
	struct i2c_msg msg;

	buf[0] = (uint8_t)((reg & 0xFF00) >> 8);
	buf[1] = (uint8_t)((reg & 0x00FF));
	buf[2] = val;

	msg.addr        = client->addr;
	msg.flags       = 0;
	msg.len         = 3;
	msg.buf         = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
} 

static int hm2140_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != HM2140_REG_END) {
		if (vals->reg_num == HM2140_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = hm2140_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
			if (vals->reg_num == HM2140_PAGE_REG){
				val &= 0xf8;
				val |= (vals->value & 0x07);
				ret = hm2140_write(sd, vals->reg_num, val);
				ret = hm2140_read(sd, vals->reg_num, &val);
			}
		}
		vals++;
	}
	return 0;
}
static int hm2140_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != HM2140_REG_END) {
		if (vals->reg_num == HM2140_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = hm2140_write(sd, vals->reg_num, vals->value);
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
static int hm2140_reset(struct v4l2_subdev *sd, u32 val)
{
	struct hm2140_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->pwdn.pin, info->pwdn.active_level);
		msleep(10);
		gpio_direction_output(info->pwdn.pin, !info->pwdn.active_level);
		msleep(10);
	}
	return 0;
}


static int hm2140_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	ret = hm2140_read(sd, 0x0000, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != HM2140_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = hm2140_read(sd, 0x0001, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != HM2140_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}


static struct hm2140_win_size hm2140_win_sizes[] = {
	/* 1920*1080 */
	{
		.width		= 1920,
		.height		= 1080,
		.sensor_info.fps		= 30 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= hm2140_init_regs_1920_1080_30fps_MIPI,
	}
};

static int hm2140_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_HM2140_FMTS)
		return -EINVAL;

	code->code = hm2140_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int hm2140_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct hm2140_format_struct *ovfmt;
	struct hm2140_win_size *wsize;
	struct hm2140_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int hm2140_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct hm2140_info *info = to_state(sd);
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

static int hm2140_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int hm2140_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int hm2140_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int hm2140_s_vflip(struct v4l2_subdev *sd, int value)
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
static int hm2140_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int hm2140_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(hm2140_again_lut); i++) {
		lut = &hm2140_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(hm2140_again_lut); i++) {
		lut = &hm2140_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int hm2140_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct hm2140_info *info = to_state(sd);
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret += hm2140_read(sd, 0x0205, &v);
	reg_val |= v;

	*value = regval_to_again(reg_val);
	return ret;
}

static int hm2140_s_again(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
	struct hm2140_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int i;

	printk("[%s %d ]````\n", __func__, __LINE__);
	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += hm2140_write(sd, 0x0205, (unsigned char)(reg_value & 0xff));
	ret += hm2140_write(sd, 0x0104, 0x01);
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Tweak autogain.
 */
static int hm2140_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int hm2140_s_exp(struct v4l2_subdev *sd, int value)
{
	struct hm2140_info *info = to_state(sd);
	int ret = 0;
	ret += hm2140_write(sd, 0x0203, ((unsigned char)(value & 0xff)));
	ret += hm2140_write(sd, 0x0202, ((unsigned char)(value >> 8)) & 0xff);
	ret += hm2140_write(sd, 0x0104, 0x01);
	return 0;
}

static int hm2140_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct hm2140_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return -EINVAL; //hm2140_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return hm2140_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int hm2140_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct hm2140_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return hm2140_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return hm2140_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return hm2140_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return hm2140_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* hm2140_s_gain turns off auto gain */
			return hm2140_s_gain(sd, info->gain->val);
		}
		return hm2140_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return hm2140_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return hm2140_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return hm2140_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops hm2140_ctrl_ops = {
	.s_ctrl = hm2140_s_ctrl,
	.g_volatile_ctrl = hm2140_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int hm2140_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = hm2140_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int hm2140_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	hm2140_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int hm2140_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	if (enable) {
		ret = hm2140_write_array(sd, hm2140_stream_on);
		printk("hm2140 stream on\n");
	}
	else {
		ret = hm2140_write_array(sd, hm2140_stream_off);
		printk("hm2140 stream off\n");
	}
	return ret;
}


int hm2140_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct hm2140_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops hm2140_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = hm2140_g_register,
	.s_register = hm2140_s_register,
#endif

};

static const struct v4l2_subdev_video_ops hm2140_video_ops = {
	.s_stream = hm2140_s_stream,
	.g_frame_interval = hm2140_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops hm2140_pad_ops = {
	//.enum_frame_interval = hm2140_enum_frame_interval,
	//.num_frame_size = hm2140_enum_frame_size,
	//.enum_mbus_code = hm2140_enum_mbus_code,
	.set_fmt = hm2140_set_fmt,
	.get_fmt = hm2140_get_fmt,
};

static const struct v4l2_subdev_ops hm2140_ops = {
	.core = &hm2140_core_ops,
	.video = &hm2140_video_ops,
	.pad = &hm2140_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int hm2140_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct hm2140_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	printk("------------%s, %d\n", __func__, __LINE__);

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwdn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,sync-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->sync.pin = gpio;
		info->sync.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	v4l2_i2c_subdev_init(sd, client, &hm2140_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
	info->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}

	ret = v4l2_clk_set_rate(info->clk, 24000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");

	info->win = &hm2140_win_sizes[0];

	hm2140_reset(sd, 1);
	/* Make sure it's an hm2140 */
	ret = hm2140_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an hm2140 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	ret = hm2140_write_array(sd, info->win->regs);
	if (ret) {
		v4l_err(client,
			"hm2140 init failed!\n");
		return ret;
	}

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 262144, 1, 1);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &hm2140_ctrl_ops,
			V4L2_CID_EXPOSURE, 1, 1080, 1, 0x0108);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "hm2140 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int hm2140_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2140_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id hm2140_id[] = {
	{ "hm2140", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, hm2140_id);

static const struct of_device_id hm2140_of_match[] = {
	{.compatible = "himax,hm2140", },
	{},
};
MODULE_DEVICE_TABLE(of, hm2140_of_match);


static struct i2c_driver hm2140_driver = {
	.driver = {
		.name	= "hm2140",
		.of_match_table = of_match_ptr(hm2140_of_match),
	},
	.probe		= hm2140_probe,
	.remove		= hm2140_remove,
	.id_table	= hm2140_id,
};

module_i2c_driver(hm2140_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for Himax hm2140 sensors");
MODULE_LICENSE("GPL");
