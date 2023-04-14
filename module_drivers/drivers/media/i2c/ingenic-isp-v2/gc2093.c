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
#include <linux/clk.h>

#include <isp-sensor.h>

#define GC2093_CHIP_ID_H	(0x20)
#define GC2093_CHIP_ID_L	(0x93)
#define GC2093_REG_CHIP_ID_HIGH         0x03f0
#define GC2093_REG_CHIP_ID_LOW          0x03f1


#define GC2093_REG_END		0xffff
#define GC2093_REG_DELAY	0x00
#define GC2093_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");


struct gc2093_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
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
	struct v4l2_ctrl *again_short;

	struct v4l2_clk *clk;
	/* struct clk *sclka; */

	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *exposure_short;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct gc2093_win_size *win;

	struct gc2093_gpio reset;
	struct gc2093_gpio pwdn;
	struct gc2093_gpio pwen;
};

/*
 * the part of driver maybe modify about different sensor and different board.
 */
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

struct again_lut gc2093_again_lut[] = {
	{0x0,0x00,0x00,0x01,0x00,0x68,0x07,0x00,0xf8,0},
	{0x1,0x00,0x10,0x01,0x0c,0x68,0x07,0x00,0xf8,14997},
	{0x2,0x00,0x20,0x01,0x1b,0x6c,0x07,0x00,0xf8,32233},
	{0x3,0x00,0x30,0xf01,0x2c,0x6c,0x07,0x00,0xf8,46808},
	{0x4,0x00,0x40,0x01,0x3f,0x7c,0x07,0x00,0xf8,60995},
	{0x5,0x00,0x50,0x02,0x16,0x7c,0x07,0x00,0xf8,75348},
	{0x6,0x00,0x60,0x02,0x35,0x7c,0x08,0x00,0xf9,90681},
	{0x7,0x00,0x70,0x03,0x16,0x7c,0x0b,0x00,0xfc,104361},
	{0x8,0x00,0x80,0x04,0x02,0x7c,0x0d,0x00,0xfe,118021},
	{0x9,0x00,0x90,0x04,0x31,0x7c,0x0f,0x08,0x00,131438},
	{0xA,0x00,0xa0,0x05,0x32,0x7c,0x11,0x08,0x02,145749},
	{0xB,0x00,0xb0,0x06,0x35,0x7c,0x14,0x08,0x05,159553},
	{0xC,0x00,0xc0,0x08,0x04,0x7c,0x16,0x08,0x07,172553},
	{0xD,0x00,0x5a,0x09,0x19,0x7c,0x18,0x08,0x09,183132},
	{0xE,0x00,0x83,0x0b,0x0f,0x7c,0x1b,0x08,0x0c,198614},
	{0xF,0x00,0x93,0x0d,0x12,0x7c,0x1e,0x08,0x0f,212697},
	{0x10,0x00,0x84,0x10,0x00,0x7c,0x22,0x08,0x13,226175},
	{0x11,0x00,0x94,0x12,0x3a,0x7c,0x26,0x08,0x17,240788},
	{0x12,0x00,0x5d,0x1a,0x02,0x7c,0x30,0x08,0x21,271536},
	{0x13,0x00,0x9b,0x1b,0x20,0x7c,0x30,0x08,0x21,272451},
	{0x14,0x00,0x8c,0x20,0x0f,0x7c,0x35,0x08,0x26,287144},
	{0x15,0x00,0x9c,0x26,0x07,0x7c,0x3b,0x08,0x2c,302425},
	{0x16,0x00,0xB6,0x36,0x21,0x7c,0x43,0x08,0x34,334228},
	{0x17,0x00,0xad,0x37,0x3a,0x7c,0x43,0x08,0x34,351574},
	{0x18,0x00,0xbd,0x3d,0x02,0x7c,0x43,0x08,0x34,367506},
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

static struct regval_list gc2093_init_regs_1920_1080_60fps_mipi[] = {
       /****system****/
       {0x03fe, 0xf0},
       {0x03fe, 0xf0},
       {0x03fe, 0xf0},
       {0x03fe, 0x00},
       {0x03f2, 0x00},
       {0x03f3, 0x00},
       {0x03f4, 0x36},
       {0x03f5, 0xc0},
       {0x03f6, 0x0B},
       {0x03f7, 0x01},
      // {0x03f8, 0x58}, //24Mhz
       {0x03f8, 0x58},  //27Mhz
       {0x03f9, 0x40},
       {0x03fc, 0x8e},
       /****CISCTL & NALOG****/
       {0x0087, 0x18},
       {0x00ee, 0x30},
       {0x00d0, 0xbf},
       {0x01a0, 0x00},
       {0x01a4, 0x40},
       {0x01a5, 0x40},
       {0x01a6, 0x40},
       {0x01af, 0x09},
       {0x0001, 0x00},
       {0x0002, 0x02},
       {0x0003, 0x04},
       {0x0004, 0x02},
       {0x0005, 0x02},
       {0x0006, 0x94},
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
	{0x0042, 0xE2},   //frame length 0x04e2=1250

      // {0x0041, 0x05},    //or dorp
      // {0x0042, 0xE6},    //frame length 0x05E6=1350

       {0x0053, 0x60},
       {0x008d, 0x92},
       {0x0090, 0x00},
       {0x00c7, 0xe1},
       {0x001b, 0x73},
       {0x0028, 0x0d},
       {0x0029, 0x24},
       {0x002b, 0x04},
       {0x002e, 0x23},
       {0x0037, 0x03},
       {0x0043, 0x04},
       {0x0044, 0x28},
       {0x004a, 0x01},
       {0x004b, 0x20},
       {0x0055, 0x28},
       {0x0066, 0x3f},
       {0x0068, 0x3f},
       {0x006b, 0x44},
       {0x0077, 0x00},
       {0x0078, 0x20},
       {0x007c, 0xa1},
       {0x00ce, 0x7c},
       {0x00d3, 0xd4},
       {0x00e6, 0x50},
       /****gain****/
       {0x00b6, 0xc0},
       {0x00b0, 0x68},    //0x60

       {0x00b3,0x00},
       {0x00b8,0x01},
       {0x00b9,0x00},
       {0x00b1,0x01},
       {0x00b2,0x00},

       /*****isp****/
       //{0x0101, 0x0c},
       {0x0102, 0x89},
       {0x0104, 0x01},
       {0x010f, 0x00},
       {0x0158, 0x00},
       {/*dark sun*/},
       {0x0123, 0x08},
       {0x0123, 0x00},
       {0x0120, 0x01},
       {0x0121, 0x04},
       {0x0122, 0xd8},
       {0x0124, 0x03},
       {0x0125, 0xff},
       {0x001a, 0x8c},
       {0x00c6, 0xe0},
       /****blk****/
       {0x0026, 0x30},
       {0x0142, 0x00},
       {0x0149, 0x1e},
       {0x014a, 0x0f},
       {0x014b, 0x00},
       {0x0155, 0x07},
       {0x0414, 0x78},
       {0x0415, 0x78},
       {0x0416, 0x78},
       {0x0417, 0x78},
       {0x0454, 0x78},
       {0x0455, 0x78},
       {0x0456, 0x78},
       {0x0457, 0x78},
       {0x04e0, 0x18},
       /****window****/
       {0x0192, 0x02},
       {0x0194, 0x03},
       {0x0195, 0x04},
       {0x0196, 0x38},
       {0x0197, 0x07},
       {0x0198, 0x80},
       /****DVP & MIP*****/
       {0x019a, 0x06},
       {0x007b, 0x2a},
       {0x0023, 0x2d},
       {0x0201, 0x27},
       {0x0202, 0x56},
       //{0x0203, 0xb6},     //try 0xce or 0x8e  or drop
       {0x0203, 0x8e},     //try 0xce or 0x8e  or drop
       {0x0212, 0x80},
       {0x0213, 0x07},
       {0x0215, 0x10},
       {0x003e, 0x91},
       {GC2093_REG_END, 0x00},	/* END MARKER */
};

#if 1
static struct regval_list gc2093_init_regs_1920_1080_30fps_mipi_lin[] = {
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0A},
	{0x03f7, 0x01},
	{0x03f8, 0x24},
	{0x03f9, 0x10},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x04},
	{0x0004, 0x00},
	{0x0005, 0x02},
	{0x0006, 0xd5},
	{0x0007, 0x00},
	{0x0008, 0x8e},//vb=142
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
//	{0x0041, 0x04},//1250
//	{0x0042, 0xe2},//1250
	{0x0041, 0x05},//1350 jz
	{0x0042, 0x46},//1350 jz
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x38},
	{0x004a, 0x01},
	{0x004b, 0x28},
	{0x0055, 0x38},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},
	{0x00b3, 0x00},
	{0x00b8, 0x01},
	{0x00b9, 0x00},
	{0x00b1, 0x01},
	{0x00b2, 0x00},
	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x010e, 0x00},//jz wdr -> lin
	{0x010f, 0x00},
	{0x0158, 0x00},
	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x00},
	{0x0121, 0x00},
	{0x0122, 0x0f},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x0126, 0x3c},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x00},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x04},
	{0x0196, 0x38},
	{0x0197, 0x07},
	{0x0198, 0x80},
	{0x01b0, 0x01},
	{0x01b1, 0x00},
	{0x01b2, 0x20},
	{0x01b3, 0x00},
	{0x01b4, 0xf0},
	{0x01b5, 0x80},
	{0x01b6, 0x05},
	{0x01b8, 0x01},
	{0x01b9, 0xe0},
	{0x01ba, 0x01},
	{0x01bb, 0x80},
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x56},
//	{0x0203, 0xb6},
	{0x0203, 0x8e},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x0215, 0x12},
	/****HDR EN****/
	{0x0027, 0x70},//jz
	{0x0215, 0x12},//jz
	{0x024d, 0x00},//jz
	{0x003e, 0x91},
	{GC2093_REG_END, 0x00}, /* END MARKER */
};
#endif

static struct regval_list gc2093_init_regs_1920_1080_15fps_mipi_wdr[] = {
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03f2, 0x00},
	{0x03f3, 0x00},
	{0x03f4, 0x36},
	{0x03f5, 0xc0},
	{0x03f6, 0x0A},
	{0x03f7, 0x01},
	{0x03f8, 0x24},
	{0x03f9, 0x10},
	{0x03fc, 0x8e},
	/****CISCTL & ANALOG****/
	{0x0087, 0x18},
	{0x00ee, 0x30},
	{0x00d0, 0xbf},
	{0x01a0, 0x00},
	{0x01a4, 0x40},
	{0x01a5, 0x40},
	{0x01a6, 0x40},
	{0x01af, 0x09},
	{0x0001, 0x00},
	{0x0002, 0x02},
	{0x0003, 0x04},
	{0x0004, 0x00},
	{0x0005, 0x02},
	{0x0006, 0xd5},
	{0x0007, 0x00},
	{0x0008, 0x8e},//vb=142
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
//	{0x0041, 0x04},//1250
//	{0x0042, 0xe2},//1250
	{0x0041, 0x05},//1350 jz
	{0x0042, 0x46},//1350 jz
	{0x0053, 0x60},
	{0x008d, 0x92},
	{0x0090, 0x00},
	{0x00c7, 0xe1},
	{0x001b, 0x73},
	{0x0028, 0x0d},
	{0x0029, 0x24},
	{0x002b, 0x04},
	{0x002e, 0x23},
	{0x0037, 0x03},
	{0x0043, 0x04},
	{0x0044, 0x38},
	{0x004a, 0x01},
	{0x004b, 0x28},
	{0x0055, 0x38},
	{0x006b, 0x44},
	{0x0077, 0x00},
	{0x0078, 0x20},
	{0x007c, 0xa1},
	{0x00ce, 0x7c},
	{0x00d3, 0xd4},
	{0x00e6, 0x50},
	/*gain*/
	{0x00b6, 0xc0},
	{0x00b0, 0x60},
	{0x00b3, 0x00},
	{0x00b8, 0x01},
	{0x00b9, 0x00},
	{0x00b1, 0x01},
	{0x00b2, 0x00},
	/*isp*/
	{0x0102, 0x89},
	{0x0104, 0x01},
	{0x010e, 0x01},
	{0x010f, 0x00},
	{0x0158, 0x00},
	/*dark sun*/
	{0x0123, 0x08},
	{0x0123, 0x00},
	{0x0120, 0x00},
	{0x0121, 0x00},
	{0x0122, 0x0f},
	{0x0124, 0x03},
	{0x0125, 0xff},
	{0x0126, 0x3c},
	{0x001a, 0x8c},
	{0x00c6, 0xe0},
	/*blk*/
	{0x0026, 0x30},
	{0x0142, 0x00},
	{0x0149, 0x1e},
	{0x014a, 0x07},
	{0x014b, 0x80},
	{0x0155, 0x00},
	{0x0414, 0x78},
	{0x0415, 0x78},
	{0x0416, 0x78},
	{0x0417, 0x78},
	{0x04e0, 0x18},
	/*window*/
	{0x0192, 0x02},
	{0x0194, 0x03},
	{0x0195, 0x04},
	{0x0196, 0x38},
	{0x0197, 0x07},
	{0x0198, 0x80},
	{0x01b0, 0x01},
	{0x01b1, 0x00},
	{0x01b2, 0x20},
	{0x01b3, 0x00},
	{0x01b4, 0xf0},
	{0x01b5, 0x80},
	{0x01b6, 0x05},
	{0x01b8, 0x01},
	{0x01b9, 0xe0},
	{0x01ba, 0x01},
	{0x01bb, 0x80},
	/****DVP & MIPI****/
	{0x019a, 0x06},
	{0x007b, 0x2a},
	{0x0023, 0x2d},
	{0x0201, 0x27},
	{0x0202, 0x56},
//	{0x0203, 0xb6},
	{0x0203, 0x8e},
	{0x0212, 0x80},
	{0x0213, 0x07},
	{0x0215, 0x12},
	/****HDR EN****/
	{0x0027, 0x71},
	{0x0215, 0x92},
	{0x024d, 0x01},
	{0x003e, 0x91},
	{GC2093_REG_END, 0x00}, /* END MARKER */
};

/*
 * the part of driver was fixed.
 */

static struct regval_list gc2093_stream_on_mipi[] = {
	{GC2093_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list gc2093_stream_off_mipi[] = {
	{GC2093_REG_END, 0x00},	/* END MARKER */
};

static int gc2093_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct gc2093_info *info = to_state(sd);
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

	printk("i2c device addr : 0x%x~~~~~~~~~~~~~\n",client->addr);
	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int gc2093_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct gc2093_info *info = to_state(sd);
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

static int gc2093_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != GC2093_REG_END) {
		if (vals->reg_num == GC2093_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = gc2093_read(sd, vals->reg_num, &val);
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
	return 0;
}

static int gc2093_ircut(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

static int gc2093_init(struct v4l2_subdev *sd, u32 val)
{
	struct gc2093_info *info = to_state(sd);
	int ret = 0;
	printk("%s:~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n",__func__);
	ret = gc2093_write_array(sd, info->win->regs);

	return ret;
}



static int gc2093_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = gc2093_read(sd, 0x03f0, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC2093_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = gc2093_read(sd, 0x03f1, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC2093_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}

static struct gc2093_win_size gc2093_win_sizes[] = {
	{
		.sensor_info.mipi_cfg.clk			= 650,	//data rate = 650Mbps/lane, clk is about 650Mbps.
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_mode		= SENSOR_MIPI_OTHER_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start1x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start1y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3y	= 0,
		.sensor_info.mipi_cfg.hcrop_diff_en		= 0,
		.sensor_info.mipi_cfg.line_sync_mode	= 0,
		.sensor_info.mipi_cfg.work_start_flag	= 0,
		.sensor_info.mipi_cfg.data_type_en		= 0,
		.sensor_info.mipi_cfg.data_type_value	= 0,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt	= TX_SENSOR_RAW10,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.wdr_en				= 0,

		.width				= 1920,
		.height				= 1080,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		//.regs 		= gc2093_init_regs_1920_1080_30fps_mipi_lin,

		.regs 		= gc2093_init_regs_1920_1080_60fps_mipi,
	},
#if 0
	{
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_mode		= SENSOR_MIPI_OTHER_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start1x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start1y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3y	= 0,
		.sensor_info.mipi_cfg.hcrop_diff_en		= 0,
		.sensor_info.mipi_cfg.line_sync_mode	= 0,
		.sensor_info.mipi_cfg.work_start_flag	= 0,
		.sensor_info.mipi_cfg.data_type_en		= 0,
		.sensor_info.mipi_cfg.data_type_value	= 0,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_WDR_2_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_VC_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt	= TX_SENSOR_RAW10,
                .sensor_info
		.width				= 1920,
		.height				= 1080,
		.fps				= 15 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= gc2093_init_regs_1920_1080_15fps_mipi_wdr,
	},
#endif
};

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

	return 0;
}

static int gc2093_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct gc2093_info *info = to_state(sd);
	struct gc2093_win_size *wsize = info->win;
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if(!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = wsize->width;
	fmt->height = wsize->height;
	fmt->code = wsize->mbus_code;
	fmt->colorspace = wsize->colorspace;
	*(unsigned int *)fmt->reserved = &wsize->sensor_info;

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

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(gc2093_again_lut); i++) {
		lut = &gc2093_again_lut[i];

		if(gain <= lut->gain) {
			return lut->index;
		}
	}

	/*last value.*/
	return lut->index;
}

static int gc2093_g_again(struct v4l2_subdev *sd, __s32 *value)
{
    return 0;
}

/*set analog gain db value, map value to sensor register.*/
static int gc2093_s_again(struct v4l2_subdev *sd, int again)
{
	struct gc2093_info *info = to_state(sd);
	struct again_lut *val_lut = gc2093_again_lut;
	unsigned int reg_value = 0;
	int ret = 0;
	if(again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	}

	ret += gc2093_write(sd,0xb4,val_lut[reg_value].regb0);
	ret += gc2093_write(sd,0xb3,val_lut[reg_value].regb1);
	ret += gc2093_write(sd,0xb8,val_lut[reg_value].regb2);
	ret += gc2093_write(sd,0xb9,val_lut[reg_value].regb3);
	ret += gc2093_write(sd,0xce,val_lut[reg_value].regb4);
	ret += gc2093_write(sd,0xc2,val_lut[reg_value].regb5);
	ret += gc2093_write(sd,0xcf,val_lut[reg_value].regb6);
	ret += gc2093_write(sd,0xd9,val_lut[reg_value].regb7);
	return ret;
}

/*
 * Tweak autogain.
 */
static int gc2093_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2093_s_exp(struct v4l2_subdev *sd, int value)
{
	struct gc2093_info *info = to_state(sd);
	int ret = 0;

	ret = gc2093_write(sd, 0x04, value & 0xff);
	ret += gc2093_write(sd, 0x03, (value>>8)&0x3f);

	return ret;
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
	printk("%s:~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n",__func__);
	if (enable) {
		ret = gc2093_write_array(sd, gc2093_stream_on_mipi);
		printk("gc2093 stream on\n");

	}
	else {
		ret = gc2093_write_array(sd, gc2093_stream_off_mipi);
		printk("gc2093 stream off\n");
	}
	return ret;
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

void gc2093_power_init_seq(struct gc2093_info *info)
{
	*(volatile unsigned int*)0xb0010100 = 0x1;
	gpio_direction_output(4,0);
	udelay(50);
	gpio_direction_output(info->pwen.pin,info->pwen.active_level);
	udelay(50);

	gpio_direction_output(info->reset.pin,!info->reset.active_level);
	msleep(10);
	gpio_direction_output(info->reset.pin,info->reset.active_level);
	msleep(20);
	gpio_direction_output(info->reset.pin,!info->reset.active_level);
	msleep(50);

	gpio_direction_output(info->pwdn.pin,info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin,!info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin,info->pwdn.active_level);
	msleep(10);
}
/* ----------------------------------------------------------------------- */

static int gc2093_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	printk("~~~%s:%d~~~=====================================\n", __func__, __LINE__);
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct gc2093_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned long rate;
	unsigned char value;
	printk("~~~%s:%d~~~\n", __func__, __LINE__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwen-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwen.pin = gpio;
		info->pwen.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwdn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	printk("gpio set ok ~~~%s:%d~~~\n", __func__, __LINE__);

	gc2093_power_init_seq(info);
	v4l2_i2c_subdev_init(sd, client, &gc2093_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
#if 1
	info->clk = v4l2_clk_get(&client->dev, "div_cim0");
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
#endif

#if 1
	/* Make sure it's an gc2093 */
	ret = gc2093_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an gc2093 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif
#if 1
	/*IRCUT ctl 0:off 1:on*/
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
			V4L2_CID_ANALOGUE_GAIN, 0, 367506, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &gc2093_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1350 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;
		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &gc2093_win_sizes[0];

	gc2093_init(sd, 1);
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	/* test image   */
//	gc2093_write(sd,0x018c,0x01);
	dev_info(&client->dev, "gc2093 Probed\n");
	return 0;
#endif
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
	{ "gc2093b", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc2093_id);

static const struct of_device_id gc2093_of_match[] = {
	{.compatible = "gc2093b", },
	{},
};
MODULE_DEVICE_TABLE(of, gc2093_of_match);


static struct i2c_driver gc2093_driver = {
	.driver = {
		.name	= "gc2093b",
		.of_match_table = of_match_ptr(gc2093_of_match),
	},
	.probe		= gc2093_probe,
	.remove		= gc2093_remove,
	.id_table	= gc2093_id,
};

module_i2c_driver(gc2093_driver);
MODULE_DESCRIPTION("A low-level driver for gc2093 sensors");
MODULE_LICENSE("GPL");
