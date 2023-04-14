/*
 * ov5645 Camera Driver
 *
 * Copyright (C) 2010 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 *
 * Based on ov772x, ov9640 drivers and previous non merged implementations.
 *
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2006, OmniVision
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-device.h>
#include <ingenic_camera.h>

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define REG_CHIP_ID_HIGH	0x300a
#define REG_CHIP_ID_LOW		0x300b
#define CHIP_ID_HIGH		0x56
#define CHIP_ID_LOW		0x45

#define  OV5645_DEFAULT_WIDTH    640
#define  OV5645_DEFAULT_HEIGHT   480

/* Private v4l2 controls */
#define V4L2_CID_PRIVATE_BALANCE  (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_EFFECT  (V4L2_CID_PRIVATE_BASE + 1)

#define REG_TC_VFLIP			0x3820
#define REG_TC_MIRROR			0x3821
#define OV5645_FLIP_VAL			((unsigned char)0x04)
#define OV5645_FLIP_MASK		((unsigned char)0x04)

/* whether sensor support high resolution (> vga) preview or not */
#define SUPPORT_HIGH_RESOLUTION_PRE		1

/*
 * Struct
 */
struct regval_list {
	u16 reg_num;
	u16 value;
};

struct mode_list {
	u16 index;
	const struct regval_list *mode_regs;
};

/* Supported resolutions */
enum ov5645_width {
	W_VGA	= OV5645_DEFAULT_WIDTH,
	W_720P	= 1280,
	W_1080P	= 1920,
};

enum ov5645_height {
	H_VGA	= OV5645_DEFAULT_HEIGHT,
	H_720P	= 720,
	H_1080P	= 1080,
};

struct ov5645_format {
	u32 code;
	const struct regval_list *regs;
};

struct ov5645_win_size {
	char *name;
	enum ov5645_width width;
	enum ov5645_height height;
	const struct regval_list *regs;
};

struct ov5645_gpio {
	int pin;
	int active_level;
};

struct ov5645_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct v4l2_clk			*clk;
	const struct ov5645_win_size	*win;

	int				model;
	u16				balance_value;
	u16				effect_value;
	u16				flag_vflip:1;
	u16				flag_hflip:1;

	struct v4l2_subdev_platform_data *sd_pdata;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *vcc_en_gpio;

	int (*power)(struct i2c_client *client, int on);
	int (*reset)(struct i2c_client *client);

	struct regulator *reg;
	struct regulator *reg1;
};

static inline int sensor_i2c_master_send(struct i2c_client *client,
		const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client,
		char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}


unsigned char ov5645_read_reg(struct i2c_client *client, u16 reg)
{
	int ret;
	unsigned char retval;
	unsigned short r = cpu_to_be16(reg);

	ret = sensor_i2c_master_send(client,(u8 *)&r,2);

	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;

	ret = sensor_i2c_master_recv(client, &retval, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;
	return retval;
}

int ov5645_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	unsigned char msg[3];
	int ret;

	reg = cpu_to_be16(reg);

	memcpy(&msg[0], &reg, 2);
	memcpy(&msg[2], &val, 1);

	ret = sensor_i2c_master_send(client, msg, 3);

	if (ret < 0)
	{
		printk("RET<0\n");
		return ret;
	}
	if (ret < 3)
	{
		printk("RET<3\n");
		return -EIO;
	}

	return 0;
}

/*
 * Registers settings
 */

#define ENDMARKER { 0xffff, 0x00 }
#define OV5645_REG_DELAY	0xfffe

static const struct regval_list ov5645_init_regs[] = {
	{0x3103, 0x11}, // software standby
	{0x3008, 0x82}, // software reset
	//delay(5ms)
	{OV5645_REG_DELAY, 3},
	{0x4202, 0x0f}, // analog control
	{0x4202, 0x0f}, // analog control

	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3503, 0x07},
	{0x3002, 0x1c},
	{0x3006, 0xc3},
	{0x300e, 0x45},
	{0x3017, 0x40},
	{0x3018, 0x00},
	{0x302e, 0x0b},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3611, 0x06},
	{0x3612, 0xab},
	{0x3614, 0x50},
	{0x3618, 0x00},
	{0x3034, 0x18},
	{0x3035, 0x21},
	{0x3036, 0x70},
	{0x3500, 0x00},
	{0x3501, 0x01},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x3f},
	{0x3600, 0x09},
	{0x3601, 0x43},
	{0x3620, 0x33},
	{0x3621, 0xe0},
	{0x3622, 0x01},
	{0x3630, 0x2d},
	{0x3631, 0x00},
	{0x3632, 0x32},
	{0x3633, 0x52},
	{0x3634, 0x70},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3702, 0x6e},
	{0x3703, 0x52},
	{0x3704, 0xa0},
	{0x3705, 0x33},
	{0x3708, 0x66},
	{0x3709, 0x12},
	{0x370b, 0x61},
	{0x370c, 0xc3},
	{0x370f, 0x10},
	{0x3715, 0x08},
	{0x3717, 0x01},
	{0x371b, 0x20},
	{0x3731, 0x22},
	{0x3739, 0x70},
	{0x3901, 0x0a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3719, 0x86},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x06},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9d},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x03},
	{0x380b, 0xc0},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x47},
	{0x3821, 0x07},
	{0x3824, 0x01},
	{0x3826, 0x03},
	{0x3828, 0x08},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0xf8},
	{0x3a0a, 0x01},
	{0x3a0b, 0xa4},
	{0x3a0e, 0x02},
	{0x3a0d, 0x02},
	{0x3a14, 0x03}, // 50Hz max exposure = 984
	{0x3a15, 0xd8}, // 50Hz max exposure
	{0x3a18, 0x01}, // gain ceiling = 31.5x
	{0x3a19, 0xf8}, // gain ceiling
	// 50/0xauto, 0xde,tect
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c07, 0x07},
	{0x3c09, 0xc2},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3c01, 0x34},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4005, 0x18},
	{0x4300, 0x32}, /* YUYV */
	{0x4514, 0x00},
	{0x4520, 0xb0},
	{0x460b, 0x37},
	{0x460c, 0x20},
	// MIPI timing
	{0x4818, 0x01},
	{0x481d, 0xf0},
	{0x481f, 0x50},
	{0x4823, 0x70},
	{0x4831, 0x14},
	{0x4837, 0x10},
	// BLC start line
	// B0xline, 0xnu,mber
	// BLC update by gain change
	// YUV 422, YUYV
	// global timing
	{0x5000, 0xa7}, // Lenc on, raw gamma on, BPC on, WPC on, color interpolation on
	{0x5001, 0x83}, // SDE on, scale off, UV adjust off, color matrix on, AWB on
	{0x501d, 0x00},
	{0x501f, 0x00}, // select ISP YUV 422
	{0x503d, 0x00},
	{0x505c, 0x30},
	// AWB control
	{0x5181, 0x59},
	{0x5183, 0x00},
	{0x5191, 0xf0},
	{0x5192, 0x03},
	// AVG control
	{0x5684, 0x10},
	{0x5685, 0xa0},
	{0x5686, 0x0c},
	{0x5687, 0x78},
	{0x5a00, 0x08},
	{0x5a21, 0x00},
	{0x5a24, 0x00},
	{0x3008, 0x02}, // wake 0xfrom, 0xso,ftware standby
	{0x3503, 0x00}, // AGC auto, AEC auto
	// AWB control
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x0a},
	{0x5189, 0x75},
	{0x518a, 0x52},
	{0x518b, 0xea},
	{0x518c, 0xa8},
	{0x518d, 0x42},
	{0x518e, 0x38},
	{0x518f, 0x56},
	{0x5190, 0x42},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	// matrix
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0b},
	{0x5385, 0x84},
	{0x5386, 0x8f},
	{0x5387, 0x82},
	{0x5388, 0x71},
	{0x5389, 0x11},
	{0x538a, 0x01},
	{0x538b, 0x98},
	// CIP
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	// Gamma
	{0x5480, 0x01},
	{0x5481, 0x0e},
	{0x5482, 0x18},
	{0x5483, 0x2b},
	{0x5484, 0x52},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x06},
	{0x5583, 0x40},
	{0x5584, 0x30},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	// LENC
	{0x5800, 0x3f},
	{0x5801, 0x16},
	{0x5802, 0x0e},
	{0x5803, 0x0d},
	{0x5804, 0x17},
	{0x5805, 0x3f},
	{0x5806, 0x0b},
	{0x5807, 0x06},
	{0x5808, 0x04},
	{0x5809, 0x04},
	{0x580a, 0x06},
	{0x580b, 0x0b},
	{0x580c, 0x09},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x08},
	{0x5812, 0x0a},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x04},
	{0x5817, 0x09},
	{0x5818, 0x0f},
	{0x5819, 0x08},
	{0x581a, 0x06},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0c},
	{0x581e, 0x3f},
	{0x581f, 0x1e},
	{0x5820, 0x12},
	{0x5821, 0x13},
	{0x5822, 0x21},
	{0x5823, 0x3f},
	{0x5824, 0x68},
	{0x5825, 0x28},
	{0x5826, 0x2c},
	{0x5827, 0x28},
	{0x5828, 0x08},
	{0x5829, 0x48},
	{0x582a, 0x64},
	{0x582b, 0x62},
	{0x582c, 0x64},
	{0x582d, 0x28},
	{0x582e, 0x46},
	{0x582f, 0x62},
	{0x5830, 0x60},
	{0x5831, 0x62},
	{0x5832, 0x26},
	{0x5833, 0x48},
	{0x5834, 0x66},
	{0x5835, 0x44},
	{0x5836, 0x64},
	{0x5837, 0x28},
	{0x5838, 0x66},
	{0x5839, 0x48},
	{0x583a, 0x2c},
	{0x583b, 0x28},
	{0x583c, 0x26},
	{0x583d, 0xae},
	{0x5025, 0x00}, // AEC in H
	{0x3a0f, 0x38}, // AEC in L
	{0x3a10, 0x30}, // AEC out H
	{0x3a1b, 0x38}, // AEC out L
	{0x3a1e, 0x30}, // control zone H
	{0x3a11, 0x70}, // control zone L
	{0x3a1f, 0x18}, // wake up from standby
	{0x3008, 0x02},
	// DPC Setting //
	{0x5780, 0xfc}, // default value
	{0x5781, 0x13}, // default value
	{0x5782, 0x03}, // default 8
	{0x5786, 0x20}, // default 10 -- add
	{0x5787, 0x40}, // default 10
	{0x5788, 0x08},
	// white pixel threshold
	{0x5789, 0x08}, // 1x
	{0x578a, 0x02}, // 8x – set in 5783
	{0x578b, 0x01}, // 12x – set in 5784
	{0x578c, 0x01}, // 128x
	// black pixel threshold
	{0x578d, 0x0c}, // 1x
	{0x578e, 0x02}, // 8x – set in 5783
	{0x578f, 0x01}, // 12x – set in 5784
	ENDMARKER,
};

static const struct regval_list ov5645_wb_auto_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_wb_incandescence_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_wb_daylight_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_wb_fluorescent_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_wb_cloud_regs[] = {
	ENDMARKER,
};

static const struct mode_list ov5645_balance[] = {
	{0, ov5645_wb_auto_regs}, {1, ov5645_wb_incandescence_regs},
	{2, ov5645_wb_daylight_regs}, {3, ov5645_wb_fluorescent_regs},
	{4, ov5645_wb_cloud_regs},
};


static const struct regval_list ov5645_effect_normal_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_effect_grayscale_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_effect_sepia_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_effect_colorinv_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_effect_sepiabluel_regs[] = {
	ENDMARKER,
};

static const struct mode_list ov5645_effect[] = {
	{0, ov5645_effect_normal_regs}, {1, ov5645_effect_grayscale_regs},
	{2, ov5645_effect_sepia_regs}, {3, ov5645_effect_colorinv_regs},
	{4, ov5645_effect_sepiabluel_regs},
};

static const struct regval_list ov5645_vga_regs[] = {
	//Sysclk = 42Mhz, MIPI 2 lane 168MBps
	//0x3612, 0xa9,
	{0x3618, 0x00},
	{0x3035, 0x21},
	{0x3036, 0x54},
	{0x3600, 0x09},
	{0x3601, 0x43},
	{0x3708, 0x66},
	{0x370c, 0xc3},
	{0x3803, 0xfa},// VS L
	{0x3806, 0x06},// VH = 1705
	{0x3807, 0xa9},// VH
	{0x3808, 0x02},// DVPHO = 640
	{0x3809, 0x80},// DVPHO
	{0x380a, 0x01},// DVPVO = 480
	{0x380b, 0xe0},// DVPVO
	{0x380c, 0x07},// HTS = 1892
	{0x380d, 0x64},// HTS
	{0x380e, 0x02},// VTS = 740
	{0x380f, 0xe4},// VTS
	{0x3814, 0x31},// X INC
	{0x3815, 0x31},// X INC
	{0x3820, 0x41},// flip off, V bin on
	{0x3821, 0x07},// mirror on, H bin on
	{0x4514, 0x00},

	{0x3a02, 0x02},// night mode ceiling = 740
	{0x3a03, 0xe4},// night mode ceiling
	{0x3a08, 0x00},// B50 = 222
	{0x3a09, 0xde},// B50
	{0x3a0a, 0x00},// B60 = 185
	{0x3a0b, 0xb9},// B60
	{0x3a0e, 0x03},// max 50
	{0x3a0d, 0x04},// max 60
	{0x3a14, 0x02},//max 50hz exposure = 3/100
	{0x3a15, 0x9a},// max 50hz exposure
	{0x3a18, 0x01},// max gain = 31.5x
	{0x3a19, 0xf8},// max gain
	{0x4004, 0x02},// BLC line number
	{0x4005, 0x18},// BLC update by gain change
	{0x4837, 0x16},// MIPI global timing
	{0x3503, 0x00},// AGC/AEC on

	{0x4300, 0x30},//YUYV
	{0x501f, 0x00},//ISP YUV
//	{0x503d ,0x80},

	ENDMARKER,
};

static const struct regval_list ov5645_720p_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov5645_1080p_regs[] = {
	{0x3618 ,0x04},
	{0x3035 ,0x61},
	{0x3036 ,0x30},
	{0x3600 ,0x08},
	{0x3601 ,0x33},
	{0x3708 ,0x63},
	{0x370c ,0xc0},
	{0x3800 ,0x01},
	{0x3801 ,0x50},
	{0x3802 ,0x01},
	{0x3803 ,0xb2},
	{0x3804 ,0x08},
	{0x3805 ,0xef},
	{0x3806 ,0x05},
	{0x3807 ,0xf1},
	{0x3808 ,0x07},//width[11:8]1920
	{0x3809 ,0x80},//width[7:0]
	{0x380a ,0x04},//height[11:8]1080
	{0x380b ,0x38},//height[7:0]
	{0x380c ,0x09},
	{0x380d ,0xc4},
	{0x380e ,0x04},
	{0x380f ,0x60},
	{0x3813 ,0x04},
	{0x3814 ,0x11},
	{0x3815 ,0x11},
	{0x3820 ,0x40},
	{0x3821 ,0x06},
	{0x3a02 ,0x04},
	{0x3a03 ,0x60},
	{0x3a09 ,0x50},
	{0x3a0b ,0x18},
	{0x3a0e ,0x03},
	{0x3a0d ,0x04},
	{0x3a14 ,0x04},
	{0x3a15 ,0x60},
	{0x3a18 ,0x00},
	{0x4004 ,0x06},
	{0x4050 ,0x6e},
	{0x4051 ,0x8f},
	{0x4300 ,0x30},
	{0x4837 ,0x0b},
	{0x5301 ,0x1e},
	{0x5305 ,0x1e},
	{0x530a ,0x1e},
	{0x5580 ,0x02},
	{0x5780 ,0xfc},
	{0x5781 ,0x13},
	{0x5782 ,0x03},
	{0x5786 ,0x20},
	{0x5787 ,0x40},
	{0x5788 ,0x08},
	{0x5789 ,0x08},
	{0x578a ,0x02},
	{0x578b ,0x01},
	{0x578c ,0x01},
	{0x578d ,0x0c},
	{0x578e ,0x02},
	{0x578f ,0x01},
	{0x5790 ,0x01},
	{0x3a11 ,0x70},
	{0x3a1b ,0x38},
	{0x3a1e ,0x30},
	{0x3a1f ,0x18},
	{0x4300 ,0x00},
	{0x501f ,0x03},
	{0x3008 ,0x02},
//	{0x503d ,0x80},

	ENDMARKER,
};


#define OV5645_SIZE(n, w, h, r) \
	{.name = n, .width = w , .height = h, .regs = r }

static struct ov5645_win_size ov5645_supported_win_sizes[] = {
	OV5645_SIZE("VGA", W_VGA, H_VGA, ov5645_vga_regs),
	OV5645_SIZE("720P", W_720P, H_720P, ov5645_720p_regs),
	OV5645_SIZE("1080P", W_1080P, H_1080P, ov5645_1080p_regs),
};

#define N_WIN_SIZES (ARRAY_SIZE(ov5645_supported_win_sizes))

struct regval_list ov5645_BGR565_Setting[] = {
	{0x4300, 0x60},//BGR565
	{0x501f, 0x01},//ISP RGB
	ENDMARKER,
};

struct regval_list ov5645_RGB565_Setting[] = {
	/* {0x4300, 0x61},//RGB565 */
	{0x4300, 0x6F},//RGB565
	{0x501f, 0x01},//ISP RGB */
	ENDMARKER,
};

struct regval_list ov5645_GRB565_Setting[] = {
	{0x4300, 0x62},//GRB565
	{0x501f, 0x01},//ISP RGB
	ENDMARKER,
};

struct regval_list ov5645_BRG565_Setting[] = {
	{0x4300, 0x63},//BRG565
	{0x501f, 0x01},//ISP RGB
	ENDMARKER,
};

struct regval_list ov5645_GBR565_Setting[] = {
	{0x4300, 0x64},//GBR565
	{0x501f, 0x01},//ISP RGB
	ENDMARKER,
};

struct regval_list ov5645_RBG565_Setting[] = {
	{0x4300, 0x65},//GBR565
	{0x501f, 0x01},//ISP RGB
	ENDMARKER,
};

struct regval_list ov5645_YUV422_YUYV_Setting[] = {
	{0x4300, 0x30},//YUYV
	{0x501f, 0x00},//ISP YUV
	ENDMARKER,
};

struct regval_list ov5645_YUV422_YVYU_Setting[] = {
	{0x4300, 0x31},//YVYU
	{0x501f, 0x00},//ISP YUV
	ENDMARKER,
};

struct regval_list ov5645_YUV422_UYVY_Setting[] = {
	{0x4300, 0x32},//UYVY
	{0x501f, 0x00},//ISP YUV
	ENDMARKER,
};

struct regval_list ov5645_YUV422_VYUY_Setting[] = {
	{0x4300, 0x33},//VYUY
	{0x501f, 0x00},//ISP YUV
	ENDMARKER,
};

struct regval_list ov5645_SBGGR8_Setting[] = {
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x10},
	{0x3813, 0x04},
	ENDMARKER,
};

struct regval_list ov5645_SGBRG8_Setting[] = {
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x11},
	{0x3813, 0x04},
	ENDMARKER,
};

struct regval_list ov5645_SGRBG8_Setting[] = {
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x10},
	{0x3813, 0x05},
	ENDMARKER,
};

struct regval_list ov5645_SRGGB8_Setting[] = {
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x11},
	{0x3813, 0x05},
	ENDMARKER,
};


struct regval_list ov5645_SBGGR10_Setting[] = {
	{0x3034, 0x1a},
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x10},
	{0x3813, 0x04},
	ENDMARKER,
};

struct regval_list ov5645_SGBRG10_Setting[] = {
	{0x3034, 0x1a},
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x11},
	{0x3813, 0x04},
	ENDMARKER,
};

struct regval_list ov5645_SGRBG10_Setting[] = {
	{0x3034, 0x1a},
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x10},
	{0x3813, 0x05},
	ENDMARKER,
};

struct regval_list ov5645_SRGGB10_Setting[] = {
	{0x3034, 0x1a},
	{0x4300, 0xf8},//raw
	{0x501f, 0x03},
	{0x3811, 0x11},
	{0x3813, 0x05},
	ENDMARKER,
};


struct ov5645_format ov5645_fmt[] = {
	{MEDIA_BUS_FMT_RGB565_2X8_LE, ov5645_RGB565_Setting},
	{INGENIC_MEDIA_BUS_FMT_RBG565_2X8_LE, ov5645_RBG565_Setting},
	{MEDIA_BUS_FMT_BGR565_2X8_LE, ov5645_BGR565_Setting},
	{INGENIC_MEDIA_BUS_FMT_BRG565_2X8_LE, ov5645_BRG565_Setting},
	{INGENIC_MEDIA_BUS_FMT_GRB565_2X8_LE, ov5645_GRB565_Setting},
	{INGENIC_MEDIA_BUS_FMT_GBR565_2X8_LE, ov5645_GBR565_Setting},
	{MEDIA_BUS_FMT_YUYV8_2X8, ov5645_YUV422_YUYV_Setting},
	{MEDIA_BUS_FMT_YVYU8_2X8, ov5645_YUV422_YVYU_Setting},
	{MEDIA_BUS_FMT_UYVY8_2X8, ov5645_YUV422_UYVY_Setting},
	{MEDIA_BUS_FMT_VYUY8_2X8, ov5645_YUV422_VYUY_Setting},
	{MEDIA_BUS_FMT_SBGGR8_1X8, ov5645_SBGGR8_Setting},
	{MEDIA_BUS_FMT_SGBRG8_1X8, ov5645_SGBRG8_Setting},
	{MEDIA_BUS_FMT_SGRBG8_1X8, ov5645_SGRBG8_Setting},
	{MEDIA_BUS_FMT_SRGGB8_1X8, ov5645_SRGGB8_Setting},
	{MEDIA_BUS_FMT_SBGGR10_1X10, ov5645_SBGGR10_Setting},
	{MEDIA_BUS_FMT_SGBRG10_1X10, ov5645_SGBRG10_Setting},
	{MEDIA_BUS_FMT_SGRBG10_1X10, ov5645_SGRBG10_Setting},
	{MEDIA_BUS_FMT_SRGGB10_1X10, ov5645_SRGGB10_Setting},
};

#define N_FMT_SIZES (ARRAY_SIZE(ov5645_fmt))

/*
 * Supported balance menus
 */
static const struct v4l2_querymenu ov5645_balance_menus[] = {
	{
		.id		= V4L2_CID_PRIVATE_BALANCE,
		.index		= 0,
		.name		= "auto",
	}, {
		.id		= V4L2_CID_PRIVATE_BALANCE,
		.index		= 1,
		.name		= "incandescent",
	}, {
		.id		= V4L2_CID_PRIVATE_BALANCE,
		.index		= 2,
		.name		= "fluorescent",
	},  {
		.id		= V4L2_CID_PRIVATE_BALANCE,
		.index		= 3,
		.name		= "daylight",
	},  {
		.id		= V4L2_CID_PRIVATE_BALANCE,
		.index		= 4,
		.name		= "cloudy-daylight",
	},

};

/*
 * Supported effect menus
 */
static const struct v4l2_querymenu ov5645_effect_menus[] = {
	{
		.id		= V4L2_CID_PRIVATE_EFFECT,
		.index		= 0,
		.name		= "none",
	}, {
		.id		= V4L2_CID_PRIVATE_EFFECT,
		.index		= 1,
		.name		= "mono",
	}, {
		.id		= V4L2_CID_PRIVATE_EFFECT,
		.index		= 2,
		.name		= "sepia",
	},  {
		.id		= V4L2_CID_PRIVATE_EFFECT,
		.index		= 3,
		.name		= "negative",
	}, {
		.id		= V4L2_CID_PRIVATE_EFFECT,
		.index		= 4,
		.name		= "aqua",
	},
};


/*
 * General functions
 */
static struct ov5645_priv *to_ov5645(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov5645_priv,
			    subdev);
}

static int ov5645_write_array(struct i2c_client *client,
			      const struct regval_list *vals)
{
	int ret;

	while (vals->reg_num != 0xffff) {
		ret = ov5645_write_reg(client, vals->reg_num, vals->value);
		dev_vdbg(&client->dev, "array: 0x%02x, 0x%02x",
			 vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

static int ov5645_mask_set(struct i2c_client *client,
			   u16  reg, u16  mask, u16  set)
{
	s32 val = ov5645_read_reg(client, reg);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return ov5645_write_reg(client, reg, val);
}


/* OF probe functions */
static int ov5645_hw_power(struct i2c_client *client, int on)
{
	struct ov5645_priv *priv = to_ov5645(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
			__func__, on ? "ENABLE" : "DISABLE");

	/* thses gpio should be set according to the active level in dt defines */
	if(priv->vcc_en_gpio) {
		gpiod_direction_output(priv->vcc_en_gpio, on);
	}

	if (priv->pwdn_gpio) {
		gpiod_direction_output(priv->pwdn_gpio, !on);
	}

	msleep(10);
	return 0;
}

static int ov5645_reset(struct i2c_client *client)
{
	struct ov5645_priv *priv = to_ov5645(client);

	if(priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}

	return 0;
}

static int ov5645_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov5645_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv *priv = to_ov5645(client);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ctrl->val = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->val = priv->flag_hflip;
		break;
	case V4L2_CID_PRIVATE_BALANCE:
		ctrl->val = priv->balance_value;
		break;
	case V4L2_CID_PRIVATE_EFFECT:
		ctrl->val = priv->effect_value;
		break;
	default:
		break;
	}
	return 0;
}

static int ov5645_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov5645_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv *priv = to_ov5645(client);
	int ret = 0;
	int i = 0;
	u16 value;

	int balance_count = ARRAY_SIZE(ov5645_balance);
	int effect_count = ARRAY_SIZE(ov5645_effect);

	switch (ctrl->id) {
	case V4L2_CID_PRIVATE_BALANCE:
		if(ctrl->val > balance_count)
			return -EINVAL;

		for(i = 0; i < balance_count; i++) {
			if(ctrl->val == ov5645_balance[i].index) {
				ret = ov5645_write_array(client,
						ov5645_balance[ctrl->val].mode_regs);
				priv->balance_value = ctrl->val;
				break;
			}
		}
		break;

	case V4L2_CID_PRIVATE_EFFECT:
		if(ctrl->val > effect_count)
			return -EINVAL;

		for(i = 0; i < effect_count; i++) {
			if(ctrl->val == ov5645_effect[i].index) {
				ret = ov5645_write_array(client,
						ov5645_effect[ctrl->val].mode_regs);
				priv->effect_value = ctrl->val;
				break;
			}
		}
		break;

	case V4L2_CID_VFLIP:
		value = ctrl->val ? OV5645_FLIP_VAL : 0x00;
		priv->flag_vflip = ctrl->val ? 1 : 0;
		ret = ov5645_mask_set(client, REG_TC_VFLIP, OV5645_FLIP_MASK, value);
		break;

	case V4L2_CID_HFLIP:
		value = ctrl->val ? OV5645_FLIP_VAL : 0x00;
		priv->flag_hflip = ctrl->val ? 1 : 0;
		ret = ov5645_mask_set(client, REG_TC_MIRROR, OV5645_FLIP_MASK, value);
		break;

	default:
		dev_err(&client->dev, "no V4L2 CID: 0x%x ", ctrl->id);
		return -EINVAL;
	}
	return ret;
}

static const struct v4l2_ctrl_ops ov5645_ctrl_ops = {
	.s_ctrl = ov5645_s_ctrl,
	.g_volatile_ctrl = ov5645_g_ctrl,
};

static int ov5645_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv *priv = to_ov5645(client);

	if(priv->sd_pdata) {
		if(on)
			return regulator_bulk_enable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
		else
			return regulator_bulk_disable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
	} else if(priv->power){
		return priv->power(client, on);
	} else {
		dev_err(&client->dev, "ov5640_s_power failde");
		return -EINVAL;
	}
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5645_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = ov5645_read_reg(client, reg->reg);
	if (ret < 0)
		return ret;

	reg->val = ret;

	return 0;
}

static int ov5645_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff)
		return -EINVAL;

	return ov5645_write_reg(client, reg->reg, reg->val);
}
#endif

/*
 * soc_camera_ops functions
 */
static int ov5645_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);

	if (!enable ) {
		dev_info(&client->dev, "stream down\n");
		ov5645_write_reg(client, 0x4202, 0x0f);
		return 0;
	}

	dev_info(&client->dev, "stream on\n");
	ov5645_write_reg(client, 0x4202, 0x00);

	return 0;
}

/* Select the nearest higher resolution for capture */
static const struct ov5645_win_size *ov5645_select_win(u32 *width, u32 *height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5645_supported_win_sizes); i++) {
		if ((*width <= ov5645_supported_win_sizes[i].width) &&
		    (*height <= ov5645_supported_win_sizes[i].height)) {
			*width = ov5645_supported_win_sizes[i].width;
			*height = ov5645_supported_win_sizes[i].height;
			return &ov5645_supported_win_sizes[i];
		}
	}
	return NULL;
}

static int ov5645_get_selection(struct v4l2_subdev *sd,
					     struct v4l2_subdev_pad_config *cfg,
					     struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv *priv = to_ov5645(client);

	if(!priv->win)
		return -EINVAL;

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int ov5645_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv *priv = to_ov5645(client);

	if (format->pad)
		return -EINVAL;

	if(priv->win) {
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = OV5645_DEFAULT_WIDTH;
		mf->height = OV5645_DEFAULT_HEIGHT;
	}

	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov5645_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct ov5645_priv       *priv = to_ov5645(client);
	int ret;

	int bala_index = priv->balance_value;
	int effe_index = priv->effect_value;
	int i;

	/* select win */
	priv->win = ov5645_select_win(width, height);

	/* select format */
	priv->cfmt_code = 0;

	/* reset hardware */
	ov5645_reset(client);

	/* initialize the sensor with default data */
	dev_dbg(&client->dev, "%s: Init default", __func__);
	ret = ov5645_write_array(client, ov5645_init_regs);
	if (ret < 0)
		goto err;

	/* set balance */
	ret = ov5645_write_array(client, ov5645_balance[bala_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set effect */
	ret = ov5645_write_array(client, ov5645_effect[effe_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set size win */
	ret = ov5645_write_array(client, priv->win->regs);
	if (ret < 0)
		goto err;

	for (i = 0; i < N_FMT_SIZES; i++) {
		if (ov5645_fmt[i].code == code) {
			ret = ov5645_write_array(client, ov5645_fmt[i].regs);
			if (ret < 0)
				goto err;
		}
	}

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	ov5645_reset(client);
	priv->win = NULL;

	return ret;
}

static int ov5645_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5645_priv       *priv = to_ov5645(client);

	if (format->pad)
		return -EINVAL;

	/*
	 * select suitable win, but don't store it
	 */
	priv->win = ov5645_select_win(&mf->width, &mf->height);
	if(!priv->win)
		return -EINVAL;

	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ov5645_set_params(client, &mf->width,
					 &mf->height, mf->code);
	if(cfg)
		cfg->try_fmt = *mf;
	return 0;
}

static int ov5645_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_FMT_SIZES)
		return -EINVAL;

	code->code = ov5645_fmt[code->index].code;
	return 0;
}

static int ov5645_enum_frame_size(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j;
	int num_valid = -1;
	__u32 index = fse->index;

	if(index >= N_WIN_SIZES)
		return -EINVAL;

	j = N_FMT_SIZES;
	while(--j)
		if(fse->code == ov5645_fmt[j].code)
			break;

	for (i = 0; i < N_WIN_SIZES; i++) {
		if (index == ++num_valid) {
			fse->code = ov5645_fmt[j].code;
			fse->min_width = ov5645_supported_win_sizes[index].width;
			fse->max_width = fse->min_width;
			fse->min_height = ov5645_supported_win_sizes[index].height;
			fse->max_height = fse->min_height;
			return 0;
		}
	}

	return -EINVAL;
}

static int ov5645_hw_reset(struct i2c_client *client)
{
	struct ov5645_priv *priv = to_ov5645(client);

	if(priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		usleep_range(3000, 5000);
		gpiod_direction_output(priv->resetb_gpio, 0);
	}
	return 0;
}

static int ov5645_video_probe(struct i2c_client *client)
{
	unsigned char retval = 0, retval_high = 0, retval_low = 0;
	struct ov5645_priv *priv = to_ov5645(client);
	int ret;

	ret = ov5645_s_power(&priv->subdev, 1);
	if (ret < 0)
		return ret;
	/*
	 * check and show product ID and manufacturer ID
	 */

	retval = ov5645_write_reg(client, 0x3008, 0x82);
	if(retval) {
		dev_err(&client->dev, "i2c write failed!\n");
		ret = -EINVAL;
		goto done;
	}

	retval_high = ov5645_read_reg(client, REG_CHIP_ID_HIGH);
	if (retval_high != CHIP_ID_HIGH) {
		dev_err(&client->dev, "read sensor %s chip_id high %x is error\n",
				client->name, retval_high);
		ret = -EINVAL;
		goto done;
	}


	retval_low = ov5645_read_reg(client, REG_CHIP_ID_LOW);
	if (retval_low != CHIP_ID_LOW) {
		dev_err(&client->dev, "read sensor %s chip_id low %x is error\n",
				client->name, retval_low);
		ret = -EINVAL;
		goto done;
	}

	dev_info(&client->dev, "read sensor %s id high:0x%x,low:%x successed!\n",
			client->name, retval_high, retval_low);

done:
	return ret;
}

static int ov5645_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_config *cfg)
{
	/*int dts*/
	return 0;
}

static struct v4l2_subdev_core_ops ov5645_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov5645_g_register,
	.s_register	= ov5645_s_register,
#endif
	.s_power	= ov5645_s_power,
};

static struct v4l2_subdev_video_ops ov5645_subdev_video_ops = {
	.s_stream	= ov5645_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5645_subdev_pad_ops = {
	.enum_mbus_code = ov5645_enum_mbus_code,
	.enum_frame_size = ov5645_enum_frame_size,
	.get_fmt	= ov5645_get_fmt,
	.set_fmt	= ov5645_set_fmt,
	.get_mbus_config = ov5645_g_mbus_config,
	.get_selection	= ov5645_get_selection,
};

static struct v4l2_subdev_ops ov5645_subdev_ops = {
	.core	= &ov5645_subdev_core_ops,
	.video	= &ov5645_subdev_video_ops,
	.pad	= &ov5645_subdev_pad_ops,
};

static int ov5645_probe_dt(struct i2c_client *client,
		struct ov5645_priv *priv)
{
	struct v4l2_subdev_platform_data *sd_pdata = priv->sd_pdata;
	struct device_node *np = client->dev.of_node;
	int supplies = 0, index = 0;

	supplies = of_property_count_strings(np, "supplies-name");
	if(supplies > 0) {
		sd_pdata = devm_kzalloc(&client->dev, sizeof(struct v4l2_subdev_platform_data), GFP_KERNEL);
		sd_pdata->num_regulators = supplies;
		sd_pdata->regulators = devm_kzalloc(&client->dev, supplies * sizeof(struct regulator_bulk_data), GFP_KERNEL);
		if(!sd_pdata->regulators) {
			dev_err(&client->dev, "Failed to allocate regulators.!\n");
			devm_kfree(&client->dev,sd_pdata);
			return -ENOMEM;
		}

		for(index = 0; index < sd_pdata->num_regulators; index ++) {
			of_property_read_string_index(np, "supplies-name", index,
					&(sd_pdata->regulators[index].supply));

			dev_dbg(&client->dev, "sd_pdata->regulators[%d].supply: %s\n",
					index, sd_pdata->regulators[index].supply);
		}

		devm_regulator_bulk_get(&client->dev, sd_pdata->num_regulators, sd_pdata->regulators);
	} else {
		/* Request the reset GPIO deasserted */
		priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
				GPIOD_OUT_LOW);
		if (!priv->resetb_gpio)
			dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
		else if (IS_ERR(priv->resetb_gpio))
			return PTR_ERR(priv->resetb_gpio);

		/* Request the power down GPIO asserted */
		priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
				GPIOD_OUT_HIGH);
		if (!priv->pwdn_gpio)
			dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
		else if (IS_ERR(priv->pwdn_gpio))
			return PTR_ERR(priv->pwdn_gpio);

		/* Request the power down GPIO asserted */
		priv->vcc_en_gpio = devm_gpiod_get_optional(&client->dev, "vcc-en",
				GPIOD_OUT_HIGH);
		if (!priv->vcc_en_gpio)
			dev_dbg(&client->dev, "vcc_en gpio is not assigned!\n");
		else if (IS_ERR(priv->vcc_en_gpio))
			return PTR_ERR(priv->vcc_en_gpio);

		/* Initialize the soc_camera_subdev_desc */
		priv->power = ov5645_hw_power;
		priv->reset = ov5645_hw_reset;
	}
	return 0;
}

#include <linux/regulator/consumer.h>
/*
 * i2c_driver functions
 */
static int ov5645_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov5645_priv	*priv;
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int	ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"ov5645: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ov5645_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(priv->clk))
		return -EPROBE_DEFER;

	v4l2_clk_set_rate(priv->clk, 24000000);

	/*mclk gate close*/
//	*(volatile unsigned int *)0xb0000024 &= ~(1 << 5);

	ret = ov5645_probe_dt(client, priv);
	if (ret)
		goto err_probe_dt;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5645_subdev_ops);

	/* add handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov5645_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov5645_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}

	ret = ov5645_video_probe(client);
	if (ret < 0)
		goto err_detect;

	ret = v4l2_ctrl_handler_setup(&priv->hdl);

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto err_async_register_subdev;

	dev_info(&adapter->dev, "ov5645 Probed\n");

	return 0;

err_async_register_subdev:
err_detect:
	v4l2_ctrl_handler_free(&priv->hdl);
err_hdl:
	v4l2_device_unregister_subdev(&priv->subdev);
err_probe_dt:
	v4l2_clk_put(priv->clk);
	devm_kfree(&client->dev, priv);
	return ret;
}

static int ov5645_remove(struct i2c_client *client)
{
	struct ov5645_priv       *priv = to_ov5645(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	devm_kfree(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id ov5645_id[] = {
	{ "ov5645",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5645_id);
static const struct of_device_id ov5645_of_match[] = {
	{.compatible = "ovti,ov5645", },
	{},
};
MODULE_DEVICE_TABLE(of, ov5645_of_match);
static struct i2c_driver ov5645_i2c_driver = {
	.driver = {
		.name = "ov5645",
		.of_match_table = of_match_ptr(ov5645_of_match),
	},
	.probe    = ov5645_probe,
	.remove   = ov5645_remove,
	.id_table = ov5645_id,
};
module_i2c_driver(ov5645_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omni Vision ov5645 sensor");
MODULE_AUTHOR("Alberto Panizzo");
MODULE_LICENSE("GPL v2");
