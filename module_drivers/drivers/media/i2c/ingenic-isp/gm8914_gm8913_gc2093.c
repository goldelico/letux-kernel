/*
 * A V4L2 driver for OmniVision gc2093 cameras.
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
#define DEBUG
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


#define GC2093_CHIP_ID_H	(0x20)
#define GC2093_CHIP_ID_L	(0x93)
#define GC2093_REG_CHIP_ID_HIGH         0x03f0
#define GC2093_REG_CHIP_ID_LOW          0x03f1


#define LHAN 0


#define GC2093_REG_END		0xffff
#define GC2093_REG_DELAY	0x00
#define GC2093_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct gc2093_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int mbus_code;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct gc2093_gpio {
	int pin;
	int active_level;
};

struct gc2093_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct gc2093_win_size *win;

	struct gc2093_gpio pwr;

	struct gc2093_gpio reset;
	struct gc2093_gpio cimen;
	struct gc2093_gpio ircutp;
	struct gc2093_gpio ircutn;
};



static unsigned int  regValTable_t30[25][8] = {
       // {0xb3,0xb8,0xb9,0x155,0xc2,0xcf,0xd9},
	{0x00,0x01,0x00,0x08,0x10,0x08,0x0a,0},
	{0x10,0x01,0x0c,0x08,0x10,0x08,0x0a,14997},
	{0x20,0x01,0x1b,0x08,0x11,0x08,0x0c,32233},
	{0x30,0x01,0x2c,0x08,0x12,0x08,0x0e,46808},
	{0x40,0x01,0x3f,0x08,0x14,0x08,0x12,60995},
	{0x50,0x02,0x16,0x08,0x15,0x08,0x14,75348},
	{0x60,0x02,0x35,0x08,0x17,0x08,0x18,90681},
	{0x70,0x03,0x16,0x08,0x18,0x08,0x1a,104361},
	{0x80,0x04,0x02,0x08,0x1a,0x08,0x1e,118021},
	{0x90,0x04,0x31,0x08,0x1b,0x08,0x20,131438},
	{0xa0,0x05,0x32,0x08,0x1d,0x08,0x24,145749},
	{0xb0,0x06,0x35,0x08,0x1e,0x08,0x26,159553},
	{0xc0,0x08,0x04,0x08,0x20,0x08,0x2a,172553},
	{0x5a,0x09,0x19,0x08,0x1e,0x08,0x2a,183132},
	{0x83,0x0b,0x0f,0x08,0x1f,0x08,0x2a,198614},
	{0x93,0x0d,0x12,0x08,0x21,0x08,0x2e,212697},
	{0x84,0x10,0x00,0x0b,0x22,0x08,0x30,226175},
	{0x94,0x12,0x3a,0x0b,0x24,0x08,0x34,240788},
	{0x5d,0x1a,0x02,0x0b,0x26,0x08,0x34,271536},
	{0x9b,0x1b,0x20,0x0b,0x26,0x08,0x34,272451},
	{0x8c,0x20,0x0f,0x0b,0x26,0x08,0x34,287144},
	{0x9c,0x26,0x07,0x12,0x26,0x08,0x34,302425},
	{0xB6,0x36,0x21,0x12,0x26,0x08,0x34,334228},
	{0xad,0x37,0x3a,0x12,0x26,0x08,0x34,351574},
	{0xbd,0x3d,0x02,0x12,0x26,0x08,0x34,367506},
};
unsigned int  gainLevelTable[26] = {
	64,
	76,
	91,
	107,
	125,
	147,
	177,
	211,
	248,
	297,
	356,
	425,
	504,
	599,
	709,
	836,
	978,
	1153,
	1647,
	1651,
	1935,
	2292,
	3239,
	3959,
	4686,
	0xffffffff,
};

/***********************************/


struct again_lut {
	unsigned int index;
	unsigned char regb0;
	unsigned char regb1;
	unsigned int  regb2;
	unsigned char regb3;
	unsigned char regb4;
	unsigned char regb5;
	unsigned char regb6;
	unsigned char regb7;

	unsigned int gain;
};


static inline struct gc2093_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc2093_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gc2093_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list gc2093_init_regs_1920_1080_30fps_mipi[] = {
/*
mclk=27mhz
pclk:74.25Mhz
Linelength:2200
row time:29.629us
frame rate:30fps
window height:1125 
size:1920*1080
*/

/****system****/
{0x03fe,0x80},
{0x03fe,0x80},
{0x03fe,0x80},
{0x03fe,0x00},
{0x03f2,0x00}, 
{0x03f3,0x0f}, 
{0x03f4,0x36}, 
{0x03f5,0xc0}, 
{0x03f6,0x0A}, 
{0x03f7,0x01}, 
{0x03f8,0x2C}, 
{0x03f9,0x10}, 
{0x03fc,0x8e},
/****CISCTL & ANALOG****/
{0x0087,0x18},
{0x00ee,0x30},
{0x00d0,0xb7},
{0x01a0,0x00},
{0x01a4,0x40},
{0x01a5,0x40},
{0x01a6,0x40},
{0x01af,0x09},
{0x0001,0x00},
{0x0002,0x02},
{0x0003,0x00},
{0x0004,0x02},
{0x0005,0x04},
{0x0006,0x4c},
{0x0007,0x00},
{0x0008,0x11},
{0x0009,0x00},
{0x000a,0x02},
{0x000b,0x00},
{0x000c,0x04},
{0x000d,0x04},
{0x000e,0x40},
{0x000f,0x07},
{0x0010,0x8c},
{0x0013,0x15},
{0x0019,0x0c},
{0x0041,0x04},
{0x0042,0x65},
{0x0053,0x60},
{0x008d,0x92},
{0x0090,0x00},
{0x0053,0x60},
{0x00c7,0xe1},
{0x001b,0x73},
{0x0028,0x0d},
{0x0029,0x24},
{0x002b,0x04},
{0x002e,0x23},
{0x0037,0x03},
{0x0043,0x04},
{0x0044,0x20},
{0x0046,0x0b},
{0x004a,0x01},
{0x004b,0x20},
{0x0055,0x20},
{0x0068,0x20},
{0x0069,0x20},
{0x0077,0x00},
{0x0078,0x04},
{0x007c,0x91},
{0x00ce,0x7c},
{0x00d3,0xdc},
{0x00e6,0x50},
/*gain*/
{0x00b6,0xc0},
{0x00b0,0x60},
/*isp*/
{0x0102,0x89},
{0x0104,0x01},
/*blk*/
{0x0026,0x20},
{0x0142,0x00},
{0x0149,0x1e},
{0x014a,0x07},
{0x014b,0x80},
{0x0155,0x07},
{0x0414,0x7e},
{0x0415,0x7e},
{0x0416,0x7e},
{0x0417,0x7e},
/*window*/
{0x0192,0x02},
{0x0194,0x03},
{0x0195,0x04},
{0x0196,0x38},
{0x0197,0x07},
{0x0198,0x80},
/****DVP & MIPI****/
{0x019a,0x06},
//{0x007b,0x2a},
{0x0023,0x2d},
{0x0201,0x20},
{0x0202,0x56},
{0x0203,0xb2},
{0x0212,0x80},
{0x0213,0x07},
{0x003e,0x40},
	
	{GC2093_REG_END, 0x00},
};

static struct regval_list gc2093_init_regs_1920_1080_60fps_mipi[] = {
//mclk=27mhz
//pclk:148.5Mhz
//Linelength:2200
//row time:14.814us
//frame rate:60fps
//window height:1125
//size:1920*1080
/****system****/
	{0x03fe, 0x80},
	{0x03fe, 0x80},
	{0x03fe, 0x80},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x0f},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x42},
	{0x03f7, 0x01},
	{0x03f8, 0x2c},
	{0x03f9, 0x82}, // I2C transfer error, ABORT interrupt
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xb7},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x00},
	{0x0004, 0x02},
	{0x0005, 0x02},
	{0x0006, 0x26},
	{0x0007, 0x00},
	{0x0008, 0x11},
	{0x0009, 0x00},
	{0x000a, 0x02},
	{0x000b, 0x00},
	{0x000c, 0x04},
	{0x000d, 0x04},
	{0x000e, 0x40},
	{0x000f, 0x07},
	{0x0010, 0x8c},
	{0x0013, 0x15},
	{0x0019, 0x0c},
	{0x0041, 0x04},
	{0x0042, 0x65},
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x0053, 0x60},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x20},
	{0x0046, 0x0b},
	{0x004a, 0x01},
	{0x004b, 0x20},
	{0x0055, 0x20},
	{0x0068, 0x20},
	{0x0069, 0x20},
	{0x0077, 0x00},
	{0x0078, 0x04},
	{0x007c, 0x91},
	{0x00ce, 0x7c},
	{0x00d3, 0xdc},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},
	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},
	/*blk*/
	{0x0026, 0x20},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x07},
	{0x0414, 0x7e},
	{0x0415, 0x7e},
	{0x0416, 0x7e},
	{0x0417, 0x7e},
	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x04},
	{0x0196, 0x38},
	{0x0197, 0x07},
	{0x0198, 0x80},
	/****DVP & MIPI****/
	{0x019a, 0x06}, 
	//{0x007b, 0x2a}, // I2C transfer error, ABORT interrupt
	{0x0023, 0x2d},
	{0x0201, 0x20},
	{0x0202, 0x56},
	{0x0203, 0xb2},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x003e, 0x40},
	{GC2093_REG_END, 0x00},
};

static struct regval_list gc2093_stream_on[] = {
	{GC2093_REG_END, 0x00},
};

static struct regval_list gc2093_stream_off[] = {
	{GC2093_REG_END, 0x00},
};


int gc2093_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2]={(reg>>8)&0xff,reg&0xff};
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
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int gc2093_write(struct v4l2_subdev *sd, unsigned short reg,
			unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3]={(reg>>8)&0xff,reg&0xff,value};
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

static int gc2093_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != GC2093_REG_END) {
		if (vals->reg_num == GC2093_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = gc2093_read(sd, vals->reg_num, &val);
			printk("%s() 0x%02x, 0x%02x\n", __func__, vals->reg_num, val);
			if (ret < 0)
			{
				return ret;
			}
		}
		vals++;
	}
	return 0;
}
static int gc2093_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != GC2093_REG_END) {
		if (vals->reg_num == GC2093_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = gc2093_write(sd, vals->reg_num, vals->value);
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
static int gc2093_reset(struct v4l2_subdev *sd, u32 val)
{
	struct gc2093_info *info = to_state(sd);
	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(20);
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(20);
	}
	return 0;
}

static int gc2093_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct gc2093_info *info = to_state(sd);
return 0;
	if(val) {
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
	} else {
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, !info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
	}
	return 0;
}


static int gc2093_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;


	ret = gc2093_read(sd, GC2093_REG_CHIP_ID_HIGH, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC2093_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = gc2093_read(sd, GC2093_REG_CHIP_ID_LOW, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC2093_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	printk(" gc2093 id %x\n", *ident);
	return 0;
}

static int gm8914_i2c_read(unsigned int reg, unsigned int *data);
static int gm8914_i2c_write(unsigned int reg, unsigned int data);
static int gm8913_i2c_read(unsigned int reg, unsigned int *data);
static int gm8913_i2c_write(unsigned int reg, unsigned int data);


static int I2C_write(int addr, int reg, int data)
{
	switch(addr) {
	case 0xC0:
		gm8914_i2c_write(reg, data);
		break;
	case 0x32:
		gm8913_i2c_write(reg, data);
		break;
		/*
	case 0x52:
		gc2093_i2c_write(reg, data);
		break;*/
	default:
		printk("%s() error slave i2c addr: 0x%x\n", __func__, addr);
	}

	return;
}

static int gm8914_gm8913_init(void)
{
	I2C_write(0xC0, 0x50, 0x32);
	I2C_write(0xC0, 0x54, 0x02);
	I2C_write(0xC0, 0x04, 0x07);
	mdelay(10);

	I2C_write(0xC0, 0x07, 0x32);
	mdelay(10);
	I2C_write(0x32, 0x11, 0x31); // i2c 100kHz
	I2C_write(0x32, 0x12, 0x31); // i2c 100kHz
	I2C_write(0x32, 0x53, 0x3f);
	//I2C_write(0x32, 0x53, 0x0f);
	mdelay(10);
	I2C_write(0xC0, 0x08, 0x6e); // sensor id: 0x6e/6f,
	I2C_write(0xC0, 0x10, 0x52);
	mdelay(10);
}


static int gc2093_init(struct v4l2_subdev *sd, u32 val)
{
	struct gc2093_info *info = to_state(sd);
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	ret = gc2093_write_array(sd, info->win->regs);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Error\n", __func__);
		return ret;
	}
#if 1
	ret = gc2093_read_array(sd, info->win->regs);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Error\n", __func__);
		return ret;
	}
#endif
	return ret;
}


static struct gc2093_win_size gc2093_win_sizes[] = {
	{
		.width		= 1920,
		.height		= 1080,
		.fps		= 30 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= gc2093_init_regs_1920_1080_30fps_mipi,
		//.regs 		= gc2093_init_regs_1920_1080_60fps_mipi,
	},

};
static const struct gc2093_win_size *gc2093_select_win(u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(gc2093_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(gc2093_win_sizes); i++) {
		if ((*width >= gc2093_win_sizes[i].width) &&
				(*height >= gc2093_win_sizes[i].height)) {
			*width = gc2093_win_sizes[i].width;
			*height = gc2093_win_sizes[i].height;
			return &gc2093_win_sizes[i];
		}
	}

	*width = gc2093_win_sizes[default_size].width;
	*height = gc2093_win_sizes[default_size].height;
	printk("------w=%d,h=%d--default_size=%d---->line=%d,func=%s\n",*width,*height,default_size,__LINE__,__func__);
	return &gc2093_win_sizes[default_size];
}

static int gc2093_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_gc2093_FMTS)
		return -EINVAL;

	code->code = gc2093_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int gc2093_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct gc2093_format_struct *ovfmt;
	struct gc2093_win_size *wsize;
	struct gc2093_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

//	info->win = gc2093_select_win(&format->format.width, &format->format.height);


	return 0;
}

static int gc2093_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct gc2093_info *info = to_state(sd);
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


	printk("----%s, %d, width: %d, height: %d, code: %x\n",
			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int gc2093_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2093_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2093_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2093_s_vflip(struct v4l2_subdev *sd, int value)
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
static int gc2093_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int gc2093_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

#if  0
static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(gc2093_again_lut); i++) {
		lut = &gc2093_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(gc2093_again_lut); i++) {
		lut = &gc2093_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}
#endif

static int gc2093_g_again(struct v4l2_subdev *sd, __s32 *value)
{
//	printk("~~~g_again=%d\n",*value);
#if 1
    return 0;
#else
    char v=0;
    unsigned int reg_val=0;
    int ret=0;
    int i=0;
    int total;
    total = sizeof(gainLevelTable) / sizeof(unsigned int);

    ret=gc2093_read(sd,0x00b1,&v);
    reg_val|=(v<<6);
    ret+=gc2093_read(sd,0x00b2,&v);
    reg_val|=((v>>2)|0x3f);

    for (i = 0; i < total; i++)
    {
        if ((gainLevelTable[i] <= reg_val)&&(reg_val < gainLevelTable[i+1]))
	{
		*value=reg_val;
		break;
	}
    }
    return ret;
#endif
}
/*set analog gain db value, map value to sensor register.*/
static int gc2093_s_again(struct v4l2_subdev *sd, int value)
{
//	printk("~~~s_again=%d\n",value);
	int index;
	//return 0;

#if 1
    int i,ret=0;
    int total;
    unsigned int tol_dig_gain = 0;

    total = sizeof(gainLevelTable) / sizeof(unsigned int);

    for (i = 0; i < total; i++)
    {
        //if ((regValTable[i][8] <= value)&&(value < regValTable[i+1][8]))
        if (value <= regValTable_t30[i][7])
            break;
    }
//    printk("~~i=%d,~~~gainLevelTable[i]=%d\n",i,gainLevelTable[i]);
//    printk("~~~regValTable[i][7]=%d\n",regValTable[i][7]);
    tol_dig_gain = value*64/regValTable_t30[i][7];

    ret += gc2093_write(sd,0x00b3,regValTable_t30[i][0]);
    ret += gc2093_write(sd,0x00b8,regValTable_t30[i][1]);
    ret += gc2093_write(sd,0x00b9,regValTable_t30[i][2]);
    ret += gc2093_write(sd,0x0155,regValTable_t30[i][3]);
    gc2093_write(sd,0x031d,0x2d);
   // ret += gc2093_write(sd,0x00c2,regValTable_t30[i][4]);
   // ret += gc2093_write(sd,0x00cf,regValTable_t30[i][5]);
   // ret += gc2093_write(sd,0x00d9,regValTable_t30[i][6]);
    gc2093_write(sd,0x031d,0x28);

   ret +=gc2093_write(sd,0x00b1,(tol_dig_gain>>6));
   ret +=gc2093_write(sd,0x00b2,((tol_dig_gain&0x3f)<<2));
    if (ret < 0) {
	    printk("gc2093_write error  %d" ,__LINE__ );
	    return ret;
    }
#endif
    return 0;
}

/*
 * Tweak autogain.
 */
static int gc2093_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2093_s_exp(struct v4l2_subdev *sd, int intt)
{
	int line_frame_total=1200;
	int frame_length;
	if(intt>line_frame_total)
		frame_length=intt+1;
	else
		frame_length=line_frame_total;
	gc2093_write(sd,0x0003,(intt>>8)&0x3f);
	gc2093_write(sd,0x0004,intt & 0xff);
	return 0;
}

static int gc2093_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2093_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return gc2093_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc2093_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int gc2093_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2093_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return gc2093_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return gc2093_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return gc2093_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return gc2093_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* gc2093_s_gain turns off auto gain */
			return gc2093_s_gain(sd, info->gain->val);
		}
		return gc2093_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return gc2093_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc2093_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return gc2093_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops gc2093_ctrl_ops = {
	.s_ctrl = gc2093_s_ctrl,
	.g_volatile_ctrl = gc2093_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc2093_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = gc2093_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int gc2093_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	gc2093_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int gc2093_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc2093_info *info = to_state(sd);
	int ret = 0;
	int val=0;
	printk("------------>line=%d,func=%s\n",__LINE__,__func__);
	if (enable) {
	//	ret = gc2093_write_array(sd, info->win->regs);
	//	if(ret<0)
	//		return ret;
		ret = gc2093_write_array(sd, gc2093_stream_on);

	//	ret = gc2093_read(sd, 0x0199, &val);

		pr_debug("gc2093 stream on\n");
	printk("----on-------->line=%d,func=%s\n",__LINE__,__func__);

	}
	else {
		ret = gc2093_write_array(sd, gc2093_stream_off);
		pr_debug("gc2093 stream off\n");
	printk("----off-------->line=%d,func=%s\n",__LINE__,__func__);
	}
	return ret;
}

int gc2093_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct gc2093_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops gc2093_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc2093_g_register,
	.s_register = gc2093_s_register,
#endif

};

static const struct v4l2_subdev_video_ops gc2093_video_ops = {
	.s_stream = gc2093_s_stream,
	.g_frame_interval = gc2093_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc2093_pad_ops = {
	//.enum_frame_interval = gc2093_enum_frame_interval,
	//.num_frame_size = gc2093_enum_frame_size,
	//.enum_mbus_code = gc2093_enum_mbus_code,
	.set_fmt = gc2093_set_fmt,
	.get_fmt = gc2093_get_fmt,
};

static const struct v4l2_subdev_ops gc2093_ops = {
	.core = &gc2093_core_ops,
	.video = &gc2093_video_ops,
	.pad = &gc2093_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int gc2093_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct gc2093_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	printk(" gc2093 probe \n\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;


	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	v4l2_i2c_subdev_init(sd, client, &gc2093_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
	info->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}

	ret = v4l2_clk_set_rate(info->clk, 27000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");

	gc2093_reset(sd, 1);

	gm8914_gm8913_init();

	/* Make sure it's an gc2093 */
	ret = gc2093_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an gc2093 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
//	gc2093_ircut(sd, 0);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
		//	V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);
			V4L2_CID_ANALOGUE_GAIN, 0, 405939, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &gc2093_win_sizes[0];//720p
//	info->win = &gc2093_win_sizes[1];//1080*360
//	info->win = &gc2093_win_sizes[2];//720*360

	gc2093_init(sd, 1);

	gc2093_init(sd, 1);	/* try 2 time init. */

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	}
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "gc2093 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int gc2093_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2093_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id gc2093_id[] = {
	{ "gc2093", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc2093_id);

static const struct of_device_id gc2093_of_match[] = {
	{.compatible = "galaxyc,gc2093", },
	{},
};
MODULE_DEVICE_TABLE(of, gc2093_of_match);


static struct i2c_driver gc2093_driver = {
	.driver = {
		.name	= "gc2093",
		.of_match_table = of_match_ptr(gc2093_of_match),
	},
	.probe		= gc2093_probe,
	.remove		= gc2093_remove,
	.id_table	= gc2093_id,
};

module_i2c_driver(gc2093_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for galaxyc gc2093 sensors");
MODULE_LICENSE("GPL");


/* ================================= GM8913 ===================================== */

struct gm8913_info {
	struct i2c_client *i2c;
	struct i2c_device_id *id;
};

static struct gm8913_info *gm8913info;

static int gm8913_read(struct i2c_client *i2c, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = i2c;
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int gm8913_write(struct i2c_client *i2c, unsigned char reg,
			unsigned char value)
{
	struct i2c_client *client = i2c;
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int gm8913_i2c_read(unsigned int reg, unsigned int *data)
{
	int ret;
	if (gm8913info==NULL)
		return -1;

	ret = gm8913_read(gm8913info->i2c, reg, (unsigned char *)data);
	printk("%s() gm8913info=%p, reg=%02x, data=%02x\n", __func__, gm8913info, reg, *data);
	if (ret < 0)
		return ret;

	return 0;
}

static int gm8913_i2c_write(unsigned int reg, unsigned int data)
{
	int ret;
	printk("%s() gm8913info=%p, reg=%02x, data=%02x\n", __func__, gm8913info, reg, data);
	if (gm8913info==NULL)
		return -1;

	ret = gm8913_write(gm8913info->i2c, reg, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int gm8913_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gm8913_info *info;

	printk("===========func=%s,line=%d\n\n",__func__,__LINE__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	gm8913info = info;
	info->i2c = client;

	return 0;
}

static int gm8913_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id gm8913_id[] = {
	{ "gm8913", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, gm8913_id);

static const struct of_device_id gm8913_of_match[] = {
	{.compatible = "zhenxin,gm8913", },
	{},
};

MODULE_DEVICE_TABLE(of, gm8913_of_match);


static struct i2c_driver gm8913_driver = {
	.driver = {
		.name	= "gm8913",
		.of_match_table = of_match_ptr(gm8913_of_match),
	},
	.probe		= gm8913_probe,
	.remove		= gm8913_remove,
	.id_table	= gm8913_id,
};

module_i2c_driver(gm8913_driver);

/* ================================= GM8913 ===================================== */

/* ================================= GM8914 ===================================== */

struct gm8914_info {
	struct i2c_client *i2c;
	struct i2c_device_id *id;
};

static struct gm8914_info *gm8914info;

static int gm8914_read(struct i2c_client *i2c, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = i2c;
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int gm8914_write(struct i2c_client *i2c, unsigned char reg,
			unsigned char value)
{
	struct i2c_client *client = i2c;
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int gm8914_i2c_read(unsigned int reg, unsigned int *data)
{
	int ret;
	if (gm8914info==NULL)
		return -1;

	ret = gm8914_read(gm8914info->i2c, reg, (unsigned char *)data);
	printk("%s() gm8914info=%p, reg=%02x, data=%02x\n", __func__, gm8914info, reg, *data);
	if (ret < 0)
		return ret;

	return 0;
}

static int gm8914_i2c_write(unsigned int reg, unsigned int data)
{
	int ret;
	printk("%s() gm8914info=%p, reg=%02x, data=%02x\n", __func__, gm8914info, reg, data);
	if (gm8914info==NULL)
		return -1;

	ret = gm8914_write(gm8914info->i2c, reg, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int gm8914_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gm8914_info *info;

	printk("===========func=%s,line=%d\n\n",__func__,__LINE__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	gm8914info = info;
	info->i2c = client;

	return 0;
}

static int gm8914_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id gm8914_id[] = {
	{ "gm8914", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, gm8914_id);

static const struct of_device_id gm8914_of_match[] = {
	{.compatible = "zhenxin,gm8914", },
	{},
};

MODULE_DEVICE_TABLE(of, gm8914_of_match);


static struct i2c_driver gm8914_driver = {
	.driver = {
		.name	= "gm8914",
		.of_match_table = of_match_ptr(gm8914_of_match),
	},
	.probe		= gm8914_probe,
	.remove		= gm8914_remove,
	.id_table	= gm8914_id,
};

module_i2c_driver(gm8914_driver);

/* ================================= GM8914 ===================================== */

/* ================================= board dts ===================================== */
#if 0
&i2c3 {
	status = "okay";
	clock-frequency = <100000>;
	timeout = <1000>;
	pinctrl-names = "default";
	pinctrl-0 = <&i2c3_pa>;

	gm8914:gm8914@0x60 {
		status = "ok";
		compatible = "zhenxin,gm8914";
		reg = <0x60>;
	};
	gm8913:gm8913@0x19 {
		status = "ok";
		compatible = "zhenxin,gm8913";
		reg = <0x19>;
	};

/* gc8914 sensor slave id :0x37*/

	gc2093:gc2093@0x29 {
		status = "ok";
		compatible = "galaxyc,gc2093";
		reg = <0x29>;
		pinctrl-names = "default";
		pinctrl-0 = <&vic_pa_low_10bit>;

		port {
			gc2093_ep0:endpoint {
				remote-endpoint = <&isp0_ep>;
				bus-width = <10>;	/* Used data lines */
				data-shift = <0>;	/* Lines 9:0 are used */

				/* If hsync-active/vsync-active are missing,
				   embedded BT.656 sync is used */
				hsync-active = <1>;	/* Active high */
				vsync-active = <1>;	/* Active high */
				data-active = <1>;	/* Active high */
				pclk-sample = <1>;	/* Rising */
			};
		};

	};

};

&isp0_ep {
	remote-endpoint = <&gc2093_ep0>;
	bus-width = <10>;	/* Used data lines */
	data-shift = <0>;	/* Lines 9:0 are used */

	/* If hsync-active/vsync-active are missing,
	   embedded BT.656 sync is used */
	hsync-active = <1>;	/* Active high */
	vsync-active = <1>;	/* Active high */

	data-active = <1>;	/* Active high */
	pclk-sample = <1>;	/* Rising */
};
#endif
/* ================================= board dts ===================================== */
