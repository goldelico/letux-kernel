/*
 * A V4L2 driver for SmartSens sc2310 cameras.
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

#define SC2310_CHIP_ID_H	(0x23)
#define SC2310_CHIP_ID_L	(0x11)
#define SC2310_REG_END		0xffff
#define SC2310_REG_DELAY	0xfffe

#define AGAIN_MAX_DB 0x64
#define DGAIN_MAX_DB 0x64
#define LOG2_GAIN_SHIFT 16

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

//#define SC2310_WDR_EN

struct sc2310_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;
};

struct sc2310_gpio {
	int pin;
	int active_level;
};

struct sc2310_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *again_short;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *exposure_short;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct sc2310_win_size *win;

	struct sc2310_gpio reset;
	struct sc2310_gpio ircutp;
	struct sc2310_gpio ircutn;
	struct sc2310_gpio pwen;
	struct sc2310_gpio pwdn;
};

void sc2310_power_init_seq(struct sc2310_info *info);
static inline struct sc2310_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc2310_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sc2310_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;
	unsigned int gain;
};

struct again_lut sc2310_again_lut[] = {
	{0x340, 0},
	{0x342, 2886},
	{0x344, 5776},
	{0x346, 8494},
	{0x348, 11136},
	{0x34a, 13706},
	{0x34c, 16287},
	{0x34e, 18723},
	{0x350, 21097},
	{0x352, 23414},
	{0x354, 25746},
	{0x356, 27953},
	{0x358, 30109},
	{0x35a, 32217},
	{0x35c, 34345},
	{0x35e, 36361},
	{0x360, 38336},
	{0x362, 40270},
	{0x364, 42226},
	{0x366, 44082},
	{0x368, 45904},
	{0x36a, 47690},
	{0x36c, 49500},
	{0x36e, 51220},
	{0x370, 52910},
	{0x372, 54571},
	{0x374, 56254},
	{0x376, 57857},
	{0x378, 59433},
	{0x37a, 60984},
	{0x37c, 62558},
	{0x37e, 64059},
	{0x740, 65536},
	{0x742, 68468},
	{0x744, 71267},
	{0x746, 74030},
	{0x748, 76672},
	{0x74a, 79283},
	{0x74c, 81784},
	{0x74e, 84259},
	{0x750, 86633},
	{0x752, 88986},
	{0x754, 91246},
	{0x756, 93489},
	{0x2341, 96091},
	{0x2343, 98956},
	{0x2345, 101736},
	{0x2347, 104437},
	{0x2349, 107063},
	{0x234b, 109618},
	{0x234d, 112106},
	{0x234f, 114530},
	{0x2351, 116894},
	{0x2353, 119200},
	{0x2355, 121451},
	{0x2357, 123649},
	{0x2359, 125798},
	{0x235b, 127899},
	{0x235d, 129954},
	{0x235f, 131965},
	{0x2361, 133935},
	{0x2363, 135864},
	{0x2365, 137755},
	{0x2367, 139609},
	{0x2369, 141427},
	{0x236b, 143211},
	{0x236d, 144962},
	{0x236f, 146681},
	{0x2371, 148369},
	{0x2373, 150027},
	{0x2375, 151657},
	{0x2377, 153260},
	{0x2379, 154836},
	{0x237b, 156385},
	{0x237d, 157910},
	{0x237f, 159411},
	{0x2741, 161610},
	{0x2743, 164475},
	{0x2745, 167256},
	{0x2747, 169958},
	{0x2749, 172584},
	{0x274b, 175140},
	{0x274d, 177628},
	{0x274f, 180052},
	{0x2751, 182416},
	{0x2753, 184722},
	{0x2755, 186974},
	{0x2757, 189172},
	{0x2759, 191321},
	{0x275b, 193423},
	{0x275d, 195478},
	{0x275f, 197490},
	{0x2761, 199460},
	{0x2763, 201389},
	{0x2765, 203280},
	{0x2767, 205134},
	{0x2769, 206953},
	{0x276b, 208736},
	{0x276d, 210487},
	{0x276f, 212207},
	{0x2771, 213895},
	{0x2773, 215554},
	{0x2775, 217184},
	{0x2777, 218786},
	{0x2779, 220362},
	{0x277b, 221912},
	{0x277d, 223437},
	{0x277f, 224938},
	{0x2f41, 227146},
	{0x2f43, 230011},
	{0x2f45, 232792},
	{0x2f47, 235494},
	{0x2f49, 238120},
	{0x2f4b, 240676},
	{0x2f4d, 243164},
	{0x2f4f, 245588},
	{0x2f51, 247952},
	{0x2f53, 250258},
	{0x2f55, 252510},
	{0x2f57, 254708},
	{0x2f59, 256857},
	{0x2f5b, 258959},
	{0x2f5d, 261014},
	{0x2f5f, 263026},
	{0x2f61, 264996},
	{0x2f63, 266925},
	{0x2f65, 268816},
	{0x2f67, 270670},
	{0x2f69, 272489},
	{0x2f6b, 274273},
	{0x2f6d, 276023},
	{0x2f6f, 277743},
	{0x2f71, 279431},
	{0x2f73, 281090},
	{0x2f75, 282720},
	{0x2f77, 284322},
	{0x2f79, 285898},
	{0x2f7b, 287448},
	{0x2f7d, 288973},
	{0x2f7f, 290474},
	{0x3f41, 292682},
	{0x3f43, 295547},
	{0x3f45, 298328},
	{0x3f47, 301030},
	{0x3f49, 303656},
	{0x3f4b, 306212},
	{0x3f4d, 308700},
	{0x3f4f, 311124},
	{0x3f51, 313488},
	{0x3f53, 315794},
	{0x3f55, 318046},
	{0x3f57, 320244},
	{0x3f59, 322393},
	{0x3f5b, 324495},
	{0x3f5d, 326550},
	{0x3f5f, 328562},
	{0x3f61, 330532},
	{0x3f63, 332461},
	{0x3f65, 334352},
	{0x3f67, 336206},
	{0x3f69, 338025},
	{0x3f6b, 339809},
	{0x3f6d, 341559},
	{0x3f6f, 343279},
	{0x3f71, 344967},
	{0x3f73, 346626},
	{0x3f75, 348256},
	{0x3f77, 349858},
	{0x3f79, 351434},
	{0x3f7b, 352984},
	{0x3f7d, 354509},
	{0x3f7f, 356010},
};

static struct regval_list sc2310_init_regs_1920_1080_25fps_mipi_dol[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0xa6},
	{0x36f9, 0x85},
	{0x3001, 0xfe},
	{0x3018, 0x33},
	{0x301c, 0x78},
	{0x3031, 0x0a},
	{0x3037, 0x24},
	{0x3038, 0x44},
	{0x303f, 0x01},
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x3f},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x320c, 0x04},
	{0x320d, 0x4c},
	{0x320e, 0x08},//0x08
	{0x320f, 0xca},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3220, 0x51},
	{0x3222, 0x29},
	{0x3301, 0x10},
	{0x3302, 0x10},
	{0x3303, 0x30},
	{0x3306, 0x54},
	{0x3308, 0x10},
	{0x3309, 0x48},
	{0x330a, 0x00},
	{0x330b, 0xb4},
	{0x330e, 0x30},
	{0x3314, 0x04},
	{0x331b, 0x83},
	{0x331e, 0x21},
	{0x331f, 0x39},
	{0x3320, 0x01},
	{0x3324, 0x02},
	{0x3325, 0x02},
	{0x3326, 0x00},
	{0x3333, 0x30},
	{0x3334, 0x40},
	{0x333d, 0x08},
	{0x3341, 0x07},
	{0x3343, 0x03},
	{0x3364, 0x1d},
	{0x3366, 0xc0},
	{0x3367, 0x08},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x336c, 0x42},
	{0x337f, 0x03},
	{0x3380, 0x1b},
	{0x33aa, 0x00},
	{0x33b6, 0x07},
	{0x33b7, 0x07},
	{0x33b8, 0x10},
	{0x33b9, 0x10},
	{0x33ba, 0x10},
	{0x33bb, 0x07},
	{0x33bc, 0x07},
	{0x33bd, 0x18},
	{0x33be, 0x18},
	{0x33bf, 0x18},
	{0x33c0, 0x05},
	{0x360f, 0x05},
	{0x3621, 0xac},
	{0x3622, 0xf6},
	{0x3623, 0x18},
	{0x3624, 0x47},
	{0x3625, 0x09},
	{0x3630, 0xc8},
	{0x3631, 0x88},
	{0x3632, 0x18},
	{0x3633, 0x22},
	{0x3634, 0x44},
	{0x3635, 0x20},
	{0x3636, 0x62},
	{0x3637, 0x0c},
	{0x3638, 0x24},
	{0x363a, 0x83},
	{0x363b, 0x08},
	{0x363c, 0x05},
	{0x363d, 0x05},
	{0x3640, 0x00},
	{0x366e, 0x04},
	{0x3670, 0x6a},
	{0x3671, 0xf6},
	{0x3672, 0x16},
	{0x3673, 0x16},
	{0x3674, 0xc8},
	{0x3675, 0x54},
	{0x3676, 0x18},
	{0x3677, 0x22},
	{0x3678, 0x53},
	{0x3679, 0x55},
	{0x367a, 0x40},
	{0x367b, 0x40},
	{0x367c, 0x40},
	{0x367d, 0x58},
	{0x367e, 0x40},
	{0x367f, 0x58},
	{0x3693, 0x20},
	{0x3694, 0x40},
	{0x3695, 0x40},
	{0x3696, 0x9f},
	{0x3697, 0x9f},
	{0x3698, 0x9f},
	{0x369e, 0x40},
	{0x369f, 0x40},
	{0x36a0, 0x58},
	{0x36a1, 0x78},
	{0x36ea, 0x35},
	{0x36eb, 0x0a},
	{0x36ec, 0x0e},
	{0x36fa, 0xa8},
	{0x3802, 0x00},
	{0x3901, 0x02},
	{0x3902, 0xc5},
	{0x3905, 0xd8},
	{0x3907, 0x01},
	{0x3908, 0x01},
	{0x391d, 0x21},
	{0x391e, 0x00},
	{0x391f, 0xc0},
	{0x3933, 0x28},
	{0x3934, 0x0a},
	{0x3940, 0x1b},
	{0x3941, 0x40},
	{0x3942, 0x08},
	{0x3943, 0x0e},
	{0x3e00, 0x01},
	{0x3e01, 0x07},
	{0x3e02, 0xa0},
	{0x3e03, 0x0b},
	{0x3e04, 0x10},
	{0x3e05, 0x80},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e0e, 0x66},
	{0x3e14, 0xb0},
	{0x3e1e, 0x35},
	{0x3e23, 0x00},
	{0x3e24, 0x26},
	{0x3e25, 0x03},
	{0x3e26, 0x40},
	{0x3f00, 0x0d},
	{0x3f04, 0x02},
	{0x3f05, 0x1e},
	{0x3f08, 0x04},
	{0x4500, 0x59},
	{0x4501, 0xa4},
	{0x4509, 0x10},
	{0x4602, 0x0f},
	{0x4603, 0x00},
	{0x4809, 0x01},
	{0x4816, 0x51},
	{0x4837, 0x1a},
	{0x5000, 0x06},
	{0x5780, 0x7f},
	{0x5781, 0x06},
	{0x5782, 0x04},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x16},
	{0x5786, 0x12},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x57a0, 0x00},
	{0x57a1, 0x74},
	{0x57a2, 0x01},
	{0x57a3, 0xf4},
	{0x57a4, 0xf0},
	{0x6000, 0x06},
	{0x6002, 0x06},
	{0x36e9, 0x26},
	{0x36f9, 0x05},
	{0x0100, 0x01},

	{SC2310_REG_DELAY, 10},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_init_regs_1920_1080_15fps_mipi[] = {
	/* {0x0103, 0x01}, */
	/* {0x0100, 0x00}, */
	/* {SC2310_REG_DELAY, 10}, */
	{0x36e9, 0xa3},//bypass pll1	20180830
	{0x36f9, 0x85},//bypass pll2	20180830
	{0x4509, 0x10},
	{0x4500, 0x39},
	{0x3907, 0x00},
	{0x3908, 0x44},
	{0x3633, 0x87},
	{0x3306, 0x7e},
	{0x330b, 0x00},
	{0x3635, 0x4c},
	{0x330e, 0x7a},
	{0x3302, 0x1f}, //3302 need be odd why????  3366	 3302	 3621
	{0x3e01, 0x8c},
	{0x3e02, 0x80},
	{0x3e09, 0x1f},
	{0x3e08, 0x3f},
	{0x3e06, 0x03},
	{0x337f, 0x03}, //new auto precharge  330e in 3372   [7:6] 11: close div_rst 00:open div_rst
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3320, 0x06}, // New ramp offset timing
	{0x3326, 0x00},
	{0x331e, 0x11},
	{0x331f, 0x21},
	{0x3303, 0x20},
	{0x3309, 0x30},
	{0x4501, 0xc4},
	{0x3e06, 0x00},
	{0x3e08, 0x03},
	{0x3e09, 0x10},
	{0x3366, 0x7c}, // div_rst gap
	{0x3622, 0x02},
	{0x3633, 0x63},
	{0x3038, 0x88},
	{0x3635, 0x45},
	{0x363b, 0x04},
	{0x363c, 0x06},
	{0x363d, 0x05},
	{0x3633, 0x23},
	{0x3301, 0x10},
	{0x3306, 0x58},
	{0x3622, 0x06},//blksun
	{0x3631, 0x88},
	{0x3630, 0x38},
	{0x3633, 0x22},
	{0x3018, 0x33},//[7:5] lane_num-1
	{0x3031, 0x0c},//[3:0] bitmode
	{0x3037, 0x40},//[6:5] bitsel  40:12bit
	{0x3001, 0xFE},//[0] c_y
	{0x4603, 0x00},//[0] data_fifo mipi mode
	{0x4837, 0x35},//[7:0] pclk period * 2
	{0x36e9, 0x83},
	{0x36eb, 0x0f},
	{0x36ec, 0x1f},
	{0x303f, 0x01},
	{0x330b, 0x20},
	{0x3640, 0x00},
	{0x3308, 0x10},
	{0x3637, 0x0a},
	{0x3e09, 0x20}, //3f for 2x fine gain
	{0x363b, 0x08},
	{0x3637, 0x09}, // ADC range: 14.8k fullwell  blc target : 0.9k	  output fullwell: 13.9k (5fps 27C  linear fullwell is 14.5K)
	{0x3638, 0x14},
	{0x3636, 0x65},
	{0x3907, 0x01},
	{0x3908, 0x01},
	{0x3320, 0x01}, //ramp
	{0x331e, 0x15},
	{0x331f, 0x25},
	{0x3366, 0x80},
	{0x3634, 0x34},
	{0x57a4, 0xf0}, //default c0,
	{0x3635, 0x41}, //fpn
	{0x3e02, 0x30}, //minimum exp 3? debug
	{0x3333, 0x30}, //col fpn  G >br	 ?
	{0x331b, 0x83},
	{0x3334, 0x40},
	{0x3306, 0x6c},
	{0x3638, 0x17},
	{0x330a, 0x01},
	{0x330b, 0x14},
	{0x3302, 0x10},
	{0x3308, 0x08},
	{0x3303, 0x18},
	{0x3309, 0x18},
	{0x331e, 0x11},
	{0x331f, 0x11},
	{0x3f00, 0x0d}, //[2]	hts/2-4
	{0x3f04, 0x02},
	{0x3f05, 0x22},
	{0x3622, 0xe6},
	{0x3633, 0x22},
	{0x3630, 0xc8},
	{0x3301, 0x10},
	{0x36e9, 0xa3},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x3638, 0x27},
	{0x33aa, 0x00}, //power save mode
	{0x3624, 0x02},
	{0x3621, 0xac},
	{0x4509, 0x40},
	{0x391e, 0x00},
	{0x391f, 0xc0},
	{0x3635, 0x45},
	{0x336c, 0x40},
	{0x3621, 0xae},
	{0x3623, 0x08},
	{0x36fa, 0xad}, //charge pump
	{0x3634, 0x44},
	{0x3621, 0xac}, //fifo delay
	{0x4500, 0x59},
	{0x3623, 0x18}, //for more grp rdout setup margin
	{0x3f08, 0x04},
	{0x3f00, 0x0d}, //[2]	hts/2-4-{3f08}
	{0x3f04, 0x02}, //0321
	{0x3f05, 0x1e}, //0321
	{0x336c, 0x42}, //recover read timing
	{0x5000, 0x06},//dpc enable
	{0x5780, 0x7f},//auto blc setting
	{0x57a0, 0x00},	//gain0 = 2x	0x0710ÖÁ0x071f
	{0x57a1, 0x71},
	{0x57a2, 0x01},	//gain1 = 8x	0x1f10ÖÁ0x1f1f
	{0x57a3, 0xf1},
	{0x5781, 0x06},	//white	1x
	{0x5782, 0x04},	//2x
	{0x5783, 0x02},	//8x
	{0x5784, 0x01},	//128x
	{0x5785, 0x16},	//black	1x
	{0x5786, 0x12},	//2x
	{0x5787, 0x08},	//8x
	{0x5788, 0x02},	//128x
	{0x3933, 0x28},
	{0x3934, 0x0a},
	{0x3940, 0x1b},
	{0x3941, 0x40},
	{0x3942, 0x08},
	{0x3943, 0x0e},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x08},
	{0x3213, 0x08},
	{0x36e9, 0xa3},
	{0x36ea, 0x77},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x36ed, 0x03},
	{0x36f9, 0x85},
	{0x36fa, 0x2d},
	{0x36fb, 0x10},
	{0x320c, 0x04},//hts=1125*2=2250
	{0x320d, 0x65},
	{0x320e, 0x12},//vts=2400
	{0x320f, 0x60},
	{0x3235, 0x12},//vts*2-0x02
	{0x3236, 0xbe},
	{0x3f04, 0x02},//{0x320c,0x320d}/2-0x3f08-0x04
	{0x3f05, 0x2a},
	{0x3802, 0x00},
	{0x3624, 0x47},
	{0x3621, 0xac},
	{0x36fa, 0x2f},
	{0x3637, 0x08},
	{0x3638, 0x25},
	{0x3635, 0x40},
	{0x363b, 0x08},
	{0x363c, 0x05},
	{0x363d, 0x05},
	{0x3303, 0x1c},
	{0x3309, 0x1c},
	{0x3324, 0x02}, //falling edge: ramp_offset_en cover ramp_integ_en
	{0x3325, 0x02},
	{0x333d, 0x08}, //count_div_rst_width
	{0x3314, 0x04},
	{0x36fa, 0x28},
	{0x3205, 0x93},
	{0x3e14, 0xb0}, //[7]:1 ana fine gain double 20~3f
	{0x3e1e, 0x35}, //[7]:1 open DCG function in 0x3e03=0x03 [6]:0 DCG >2	[2] 1: dig_fine_gain_en [1:0] max fine gain  01: 3f
	{0x3e0e, 0x66}, //[7:3] fine gain to compsensate 2.4x DCGgain  5 : 2.3125x  6:2.375x  [2]:1 DCG gain between sa1gain 2~4	 [1]:1 dcg gain in 0x3e08[5]
	{0x3364, 0x1d},//[4] fine gain op 1~20--3f 0~10--1f [4] ana dithring en
	{0x33b6, 0x07},//gain0 when dcg off
	{0x33b7, 0x07},//gain1 when dcg off
	{0x33b8, 0x10},//sel0 when dcg off
	{0x33b9, 0x10},//sel1 when dcg off
	{0x33ba, 0x10},//sel2 when dcg off
	{0x33bb, 0x07},//gain0 when dcg on
	{0x33bc, 0x07},//gain1 when dcg on
	{0x33bd, 0x14},//sel0 when dcg on
	{0x33be, 0x14},//sel1 when dcg on
	{0x33bf, 0x14},//sel2 when dcg on
	{0x360f, 0x05},//[0] 3622 auto en
	{0x367a, 0x40},//gain0
	{0x367b, 0x40},//gain1
	{0x3671, 0xf6},//sel0
	{0x3672, 0x16},//sel1
	{0x3673, 0x16},//sel2
	{0x366e, 0x04},//[2] fine gain op 1~20--3f 0~10--1f
	{0x3670, 0x4a},//[1] 3630 auto en, [3] 3633 auto en, [6] 363a auto en
	{0x367c, 0x40},//gain0	3e08[5:2] 1000
	{0x367d, 0x58},//gain1 3e08[5:2] 1011
	{0x3674, 0xc8},//sel0
	{0x3675, 0x54},//sel1
	{0x3676, 0x18},//sel2
	{0x367e, 0x40},//gain0	3e08[5:2] 1000
	{0x367f, 0x58},//gain1	3e08[5:2] 1011
	{0x3677, 0x22},//sel0
	{0x3678, 0x53},//sel1
	{0x3679, 0x55},//sel2
	{0x36a0, 0x58},//gain0	3e08[5:2] 1011
	{0x36a1, 0x78},//gain1	3e08[5:2] 1111
	{0x3696, 0x83},//sel0
	{0x3697, 0x87},//sel1
	{0x3698, 0x9f},//sel2
	{0x4837, 0x31},
	{0x6000, 0x00},
	{0x6002, 0x00},
	{0x301c, 0x78},//close dvp
	{0x3037, 0x44},//[3:0] pump div	range [10M,20M],sclk=81/2=40.5M,div=4-->sclk/4=10.125M,duty cycle-->even number
	{0x3038, 0x44},//[7:4]ppump & [3:0]npump
	{0x3632, 0x18},//[5:4]idac driver
	{0x5785, 0x40},//black	point 1x
	{0x4809, 0x01},//mipi first frame, lp status
	{0x3637, 0x10},
	{0x5000, 0x06},//dpc enable
	{0x5780, 0x7f},//auto blc setting
	{0x57a0, 0x00},	//gain0 = 2x	0x0740ÖÁ0x077f
	{0x57a1, 0x74},
	{0x57a2, 0x01},	//gain1 = 8x	0x1f40ÖÁ0x1f7f
	{0x57a3, 0xf4},
	{0x5781, 0x06},	//white	1x
	{0x5782, 0x04},	//2x
	{0x5783, 0x02},	//8x
	{0x5784, 0x01},	//128x
	{0x5785, 0x16},	//black	1x
	{0x5786, 0x12},	//2x
	{0x5787, 0x08},	//8x
	{0x5788, 0x02},	//128x
	{0x4501, 0xb4},//reduce bit
	{0x3637, 0x20},
	{0x4509, 0x20},//blc quantification	//20181206
	{0x3364, 0x1d},//[4] fine gain op 1~20--3f 0~10--1f [4] ana dithring en
	{0x33b6, 0x07},//gain0 when dcg off	gain<dcg
	{0x33b7, 0x07},//gain1 when dcg off
	{0x33b8, 0x10},//sel0 when dcg off
	{0x33b9, 0x10},//sel1 when dcg off
	{0x33ba, 0x10},//sel2 when dcg off
	{0x33bb, 0x07},//gain0 when dcg on		gain>=dcg
	{0x33bc, 0x07},//gain1 when dcg on
	{0x33bd, 0x20},//sel0 when dcg on
	{0x33be, 0x20},//sel1 when dcg on
	{0x33bf, 0x20},//sel2 when dcg on
	{0x360f, 0x05},//[0] 3622 auto en
	{0x367a, 0x40},//gain0
	{0x367b, 0x40},//gain1
	{0x3671, 0xf6},//sel0
	{0x3672, 0x16},//sel1
	{0x3673, 0x16},//sel2
	{0x366e, 0x04},//[2] fine gain op 1~20--3f 0~10--1f
	{0x3670, 0x4a},//[1] 3630 auto en, [3] 3633 auto en, [6] 363a auto en
	{0x367c, 0x40},//gain0	3e08[5:2] 1000
	{0x367d, 0x58},//gain1 3e08[5:2] 1011
	{0x3674, 0xc8},//sel0
	{0x3675, 0x54},//sel1
	{0x3676, 0x18},//sel2
	{0x367e, 0x40},//gain0	3e08[5:2] 1000
	{0x367f, 0x58},//gain1	3e08[5:2] 1011
	{0x3677, 0x22},//sel0
	{0x3678, 0x33},//sel1
	{0x3679, 0x44},//sel2
	{0x36a0, 0x58},//gain0	3e08[5:2] 1011
	{0x36a1, 0x78},//gain1	3e08[5:2] 1111
	{0x3696, 0x83},//sel0
	{0x3697, 0x87},//sel1
	{0x3698, 0x9f},//sel2
	{0x3637, 0x17},//fullwell 8.6k
	{0x331e, 0x11},
	{0x331f, 0x21},//1d
	{0x3303, 0x1c},	//[hl,to][1,1a,2e]
	{0x3309, 0x3c},	//[hl,to][1,32,46]
	{0x330a, 0x00},
	{0x330b, 0xc8},	//[bs,to][1,a8,ec]
	{0x3306, 0x68},	//[hl,bs][1,46,88]
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x00},
	{0x3203, 0x04},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3380, 0x1b},
	{0x3341, 0x07},//3318[3:0] + 2
	{0x3343, 0x03},//3318[3:0] -2
	{0x3e25, 0x03},//blc dithering(analog fine gain)
	{0x3e26, 0x40},
	{0x3366, 0x70},//[60,78]
	{0x3e00, 0x00},//max_exposure = vts*2-6;	min_exposure = 3;	20180712
	{0x3e01, 0x95},
	{0x3e02, 0xa0},
	{0x3e03, 0x0b},//gain map 0x0b mode	gain=1x
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3905, 0xd8},
	{0x36e9, 0x23},//enable pll1	20180830
	{0x36f9, 0x05},//enable pll2	20180830
	{0x0100, 0x01},

	{SC2310_REG_DELAY, 10},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_init_regs_1920_1080_15fps_mipi_dol[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0xa3},//bypass pll1	20180830
	{0x36f9, 0x85},//bypass pll2	20180830
	{0x4509, 0x10},
	{0x4500, 0x39},
	{0x3907, 0x00},
	{0x3908, 0x44},
	{0x3633, 0x87},
	{0x3306, 0x7e},
	{0x330b, 0x00},
	{0x3635, 0x4c},
	{0x330e, 0x7a},
	{0x3302, 0x1f}, //3302 need be odd why????  3366  3302   3621
	{0x3e01, 0x8c},
	{0x3e02, 0x80},
	{0x3e09, 0x1f},
	{0x3e08, 0x3f},
	{0x3e06, 0x03},
	{0x337f, 0x03}, //new auto precharge  330e in 3372   [7:6] 11: close div_rst 00:open div_rst
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3320, 0x06}, // New ramp offset timing
	{0x3326, 0x00},
	{0x331e, 0x11},
	{0x331f, 0x21},
	{0x3303, 0x20},
	{0x3309, 0x30},
	{0x4501, 0xc4},
	{0x3e06, 0x00},
	{0x3e08, 0x03},
	{0x3e09, 0x10},
	{0x3366, 0x7c}, // div_rst gap
	{0x3622, 0x02},
	{0x3633, 0x63},
	{0x3038, 0x88},
	{0x3635, 0x45},
	{0x363b, 0x04},
	{0x363c, 0x06},
	{0x363d, 0x05},
	{0x3633, 0x23},
	{0x3301, 0x10},
	{0x3306, 0x58},
	{0x3622, 0x06},//blksun
	{0x3631, 0x88},
	{0x3630, 0x38},
	{0x3633, 0x22},
	{0x3018, 0x33},//[7:5] lane_num-1
	{0x3031, 0x0c},//[3:0] bitmode
	{0x3037, 0x40},//[6:5] bitsel  40:12bit
	{0x3001, 0xFE},//[0] c_y
	{0x4603, 0x00},//[0] data_fifo mipi mode
	{0x4837, 0x35},//[7:0] pclk period * 2
	{0x36e9, 0x83},
	{0x36eb, 0x0f},
	{0x36ec, 0x1f},
	{0x303f, 0x01},
	{0x330b, 0x20},
	{0x3640, 0x00},
	{0x3308, 0x10},
	{0x3637, 0x0a},
	{0x3e09, 0x20}, //3f for 2x fine gain
	{0x363b, 0x08},
	{0x3637, 0x09}, // ADC range: 14.8k fullwell  blc target : 0.9k   output fullwell: 13.9k (5fps 27C  linear fullwell is 14.5K)
	{0x3638, 0x14},
	{0x3636, 0x65},
	{0x3907, 0x01},
	{0x3908, 0x01},
	{0x3320, 0x01}, //ramp
	{0x331e, 0x15},
	{0x331f, 0x25},
	{0x3366, 0x80},
	{0x3634, 0x34},
	{0x57a4, 0xf0}, //default c0,
	{0x3635, 0x41}, //fpn
	{0x3e02, 0x30}, //minimum exp 3? debug
	{0x3333, 0x30}, //col fpn  G >br  ?
	{0x331b, 0x83},
	{0x3334, 0x40},
	{0x3306, 0x6c},
	{0x3638, 0x17},
	{0x330a, 0x01},
	{0x330b, 0x14},
	{0x3302, 0x10},
	{0x3308, 0x08},
	{0x3303, 0x18},
	{0x3309, 0x18},
	{0x331e, 0x11},
	{0x331f, 0x11},
	{0x3f00, 0x0d}, //[2]   hts/2-4
	{0x3f04, 0x02},
	{0x3f05, 0x22},
	{0x3622, 0xe6},
	{0x3633, 0x22},
	{0x3630, 0xc8},
	{0x3301, 0x10},
	{0x36e9, 0xa3},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x3638, 0x27},
	{0x33aa, 0x00}, //power save mode
	{0x3624, 0x02},
	{0x3621, 0xac},
	{0x4509, 0x40},
	{0x391e, 0x00},
	{0x391f, 0xc0},
	{0x3635, 0x45},
	{0x336c, 0x40},
	{0x3621, 0xae},
	{0x3623, 0x08},
	{0x36fa, 0xad}, //charge pump
	{0x3634, 0x44},
	{0x3621, 0xac}, //fifo delay
	{0x4500, 0x59},
	{0x3623, 0x18}, //for more grp rdout setup margin
	{0x3f08, 0x04},
	{0x3f00, 0x0d}, //[2]   hts/2-4-{3f08}
	{0x3f04, 0x02}, //0321
	{0x3f05, 0x1e}, //0321
	{0x336c, 0x42}, //recover read timing
	{0x5000, 0x06},//dpc enable
	{0x5780, 0x7f},//auto blc setting
	{0x57a0, 0x00},	//gain0 = 2x	0x0710ÖÁ0x071f
	{0x57a1, 0x71},
	{0x57a2, 0x01},	//gain1 = 8x	0x1f10ÖÁ0x1f1f
	{0x57a3, 0xf1},
	{0x5781, 0x06},	//white	1x
	{0x5782, 0x04},	//2x
	{0x5783, 0x02},	//8x
	{0x5784, 0x01},	//128x
	{0x5785, 0x16},	//black	1x
	{0x5786, 0x12},	//2x
	{0x5787, 0x08},	//8x
	{0x5788, 0x02},	//128x
	{0x3933, 0x28},
	{0x3934, 0x0a},
	{0x3940, 0x1b},
	{0x3941, 0x40},
	{0x3942, 0x08},
	{0x3943, 0x0e},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x08},
	{0x3213, 0x08},
	{0x36e9, 0xa3},
	{0x36ea, 0x77},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x36ed, 0x03},
	{0x36f9, 0x85},
	{0x36fa, 0x2d},
	{0x36fb, 0x10},
	{0x320c, 0x04},//hts=1125*2=2250
	{0x320d, 0x65},
	{0x320e, 0x04},//vts=1200
	{0x320f, 0xb0},
	{0x3235, 0x12},//vts*2-0x02
	{0x3236, 0xbe},
	{0x3f04, 0x02},//{0x320c,0x320d}/2-0x3f08-0x04
	{0x3f05, 0x2a},
	{0x3802, 0x00},
	{0x3624, 0x47},
	{0x3621, 0xac},
	{0x36fa, 0x2f},
	{0x3637, 0x08},
	{0x3638, 0x25},
	{0x3635, 0x40},
	{0x363b, 0x08},
	{0x363c, 0x05},
	{0x363d, 0x05},
	{0x3303, 0x1c},
	{0x3309, 0x1c},
	{0x3324, 0x02}, //falling edge: ramp_offset_en cover ramp_integ_en
	{0x3325, 0x02},
	{0x333d, 0x08}, //count_div_rst_width
	{0x3314, 0x04},
	{0x36fa, 0x28},
	{0x3205, 0x93},
	{0x3e14, 0xb0}, //[7]:1 ana fine gain double 20~3f
	{0x3e1e, 0x35}, //[7]:1 open DCG function in 0x3e03=0x03 [6]:0 DCG >2   [2] 1: dig_fine_gain_en [1:0] max fine gain  01: 3f
	{0x3e0e, 0x66}, //[7:3] fine gain to compsensate 2.4x DCGgain  5 : 2.3125x  6:2.375x  [2]:1 DCG gain between sa1gain 2~4  [1]:1 dcg gain in 0x3e08[5]
	{0x3364, 0x1d},//[4] fine gain op 1~20--3f 0~10--1f [4] ana dithring en
	{0x33b6, 0x07},//gain0 when dcg off
	{0x33b7, 0x07},//gain1 when dcg off
	{0x33b8, 0x10},//sel0 when dcg off
	{0x33b9, 0x10},//sel1 when dcg off
	{0x33ba, 0x10},//sel2 when dcg off
	{0x33bb, 0x07},//gain0 when dcg on
	{0x33bc, 0x07},//gain1 when dcg on
	{0x33bd, 0x14},//sel0 when dcg on
	{0x33be, 0x14},//sel1 when dcg on
	{0x33bf, 0x14},//sel2 when dcg on
	{0x360f, 0x05},//[0] 3622 auto en
	{0x367a, 0x40},//gain0
	{0x367b, 0x40},//gain1
	{0x3671, 0xf6},//sel0
	{0x3672, 0x16},//sel1
	{0x3673, 0x16},//sel2
	{0x366e, 0x04},//[2] fine gain op 1~20--3f 0~10--1f
	{0x3670, 0x4a},//[1] 3630 auto en, [3] 3633 auto en, [6] 363a auto en
	{0x367c, 0x40},//gain0  3e08[5:2] 1000
	{0x367d, 0x58},//gain1 3e08[5:2] 1011
	{0x3674, 0xc8},//sel0
	{0x3675, 0x54},//sel1
	{0x3676, 0x18},//sel2
	{0x367e, 0x40},//gain0  3e08[5:2] 1000
	{0x367f, 0x58},//gain1  3e08[5:2] 1011
	{0x3677, 0x22},//sel0
	{0x3678, 0x53},//sel1
	{0x3679, 0x55},//sel2
	{0x36a0, 0x58},//gain0  3e08[5:2] 1011
	{0x36a1, 0x78},//gain1  3e08[5:2] 1111
	{0x3696, 0x83},//sel0
	{0x3697, 0x87},//sel1
	{0x3698, 0x9f},//sel2
	{0x4837, 0x31},
	{0x6000, 0x00},
	{0x6002, 0x00},
	{0x301c, 0x78},//close dvp
	{0x3037, 0x44},//[3:0] pump div	range [10M,20M],sclk=81/2=40.5M,div=4-->sclk/4=10.125M,duty cycle-->even number
	{0x3038, 0x44},//[7:4]ppump & [3:0]npump
	{0x3632, 0x18},//[5:4]idac driver
	{0x5785, 0x40},//black	point 1x
	{0x4809, 0x01},//mipi first frame, lp status
	{0x3637, 0x10},
	{0x5000, 0x06},//dpc enable
	{0x5780, 0x7f},//auto blc setting
	{0x57a0, 0x00},	//gain0 = 2x	0x0740ÖÁ0x077f
	{0x57a1, 0x74},
	{0x57a2, 0x01},	//gain1 = 8x	0x1f40ÖÁ0x1f7f
	{0x57a3, 0xf4},
	{0x5781, 0x06},	//white	1x
	{0x5782, 0x04},	//2x
	{0x5783, 0x02},	//8x
	{0x5784, 0x01},	//128x
	{0x5785, 0x16},	//black	1x
	{0x5786, 0x12},	//2x
	{0x5787, 0x08},	//8x
	{0x5788, 0x02},	//128x
	{0x4501, 0xb4},//reduce bit
	{0x3637, 0x20},
	{0x4509, 0x20},//blc quantification	//20181206
	{0x3364, 0x1d},//[4] fine gain op 1~20--3f 0~10--1f [4] ana dithring en
	{0x33b6, 0x07},//gain0 when dcg off	gain<dcg
	{0x33b7, 0x07},//gain1 when dcg off
	{0x33b8, 0x10},//sel0 when dcg off
	{0x33b9, 0x10},//sel1 when dcg off
	{0x33ba, 0x10},//sel2 when dcg off
	{0x33bb, 0x07},//gain0 when dcg on		gain>=dcg
	{0x33bc, 0x07},//gain1 when dcg on
	{0x33bd, 0x20},//sel0 when dcg on
	{0x33be, 0x20},//sel1 when dcg on
	{0x33bf, 0x20},//sel2 when dcg on
	{0x360f, 0x05},//[0] 3622 auto en
	{0x367a, 0x40},//gain0
	{0x367b, 0x40},//gain1
	{0x3671, 0xf6},//sel0
	{0x3672, 0x16},//sel1
	{0x3673, 0x16},//sel2
	{0x366e, 0x04},//[2] fine gain op 1~20--3f 0~10--1f
	{0x3670, 0x4a},//[1] 3630 auto en, [3] 3633 auto en, [6] 363a auto en
	{0x367c, 0x40},//gain0  3e08[5:2] 1000
	{0x367d, 0x58},//gain1 3e08[5:2] 1011
	{0x3674, 0xc8},//sel0
	{0x3675, 0x54},//sel1
	{0x3676, 0x18},//sel2
	{0x367e, 0x40},//gain0  3e08[5:2] 1000
	{0x367f, 0x58},//gain1  3e08[5:2] 1011
	{0x3677, 0x22},//sel0
	{0x3678, 0x33},//sel1
	{0x3679, 0x44},//sel2
	{0x36a0, 0x58},//gain0  3e08[5:2] 1011
	{0x36a1, 0x78},//gain1  3e08[5:2] 1111
	{0x3696, 0x9f},//sel0
	{0x3697, 0x9f},//sel1
	{0x3698, 0x9f},//sel2
	{0x3637, 0x17},//fullwell 8.6k
	{0x331e, 0x11},
	{0x331f, 0x21},//1d
	{0x3303, 0x1c},	//[hl,to][1,1a,2e]
	{0x3309, 0x3c},	//[hl,to][1,32,46]
	{0x330a, 0x00},
	{0x330b, 0xc8},	//[bs,to][1,a8,ec]
	{0x3306, 0x68},	//[hl,bs][1,46,88]
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x00},
	{0x3203, 0x04},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3380, 0x1b},
	{0x3341, 0x07},//3318[3:0] + 2
	{0x3343, 0x03},//3318[3:0] -2
	{0x3e25, 0x03},//blc dithering(analog fine gain)
	{0x3e26, 0x40},
	{0x3366, 0x70},//[60,78]
	{0x4816, 0x51},//bit[4]
	{0x3220, 0x51},//bit[6]
	{0x4602, 0x0f},//bit[3:0]
	{0x33c0, 0x05},//bit[2]
	{0x6000, 0x06},
	{0x6002, 0x06},
	/* {0x320e, 0x1a},//double vts */
	/* {0x320f, 0x60}, */
	{0x320e, 0x12},//double vts
	{0x320f, 0x60},
	{0x3202, 0x00},//x_start must be 0x00
	{0x3203, 0x00},
	{0x3206, 0x04},//1088	activeBoard=4
	{0x3207, 0x3f},
	{0x3e00, 0x01},//max long exposure = 0x103e
	{0x3e01, 0x19},
	{0x3e02, 0xe0},
	{0x3e04, 0x11},//max short exposure = 0x104
	{0x3e05, 0x80},
	{0x3e23, 0x01},//max long exp : max short exp <= 16:1
	{0x3e24, 0x1a},
	{0x3e03, 0x0b},//gain map 0x0b mode	gain=1x
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3905, 0xd8},
	{0x36e9, 0x23},//enable pll1	20180830
	{0x36f9, 0x05},//enable pll2	20180830
	{0x0100, 0x01},
	{SC2310_REG_DELAY, 10},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_init_regs_1920_1080_25fps_mipi[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0xa3},//bypass pll1
	{0x36f9, 0x85},//bypass pll2
	{0x337f, 0x03},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3326, 0x00},
	{0x3631, 0x88},
	{0x3018, 0x33},
	{0x3031, 0x0c},
	{0x3001, 0xfe},
	{0x4603, 0x00},
	{0x303f, 0x01},
	{0x3640, 0x00},
	{0x3636, 0x65},
	{0x3907, 0x01},
	{0x3908, 0x01},
	{0x3320, 0x01},
	{0x3366, 0x70},
	{0x57a4, 0xf0},
	{0x3333, 0x30},
	{0x331b, 0x83},
	{0x3334, 0x40},
	{0x3302, 0x10},
	{0x3308, 0x08},
	{0x3622, 0xe6},
	{0x3633, 0x22},
	{0x3630, 0xc8},
	{0x3301, 0x10},
	{0x33aa, 0x00},
	{0x391e, 0x00},
	{0x391f, 0xc0},
	{0x3634, 0x44},
	{0x4500, 0x59},
	{0x3623, 0x18},
	{0x3f08, 0x04},
	{0x3f00, 0x0d},
	{0x336c, 0x42},
	{0x3933, 0x28},
	{0x3934, 0x0a},
	{0x3940, 0x1b},
	{0x3941, 0x40},
	{0x3942, 0x08},
	{0x3943, 0x0e},
	{0x36ea, 0x77},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x36ed, 0x03},
	{0x36fb, 0x10},
	{0x320c, 0x04},//1125
	{0x320d, 0x65},
	{0x320e, 0x05},//1440
	{0x320f, 0xa0},
	{0x3235, 0x09},
	{0x3236, 0x5e},
	{0x3f04, 0x02},
	{0x3f05, 0x2a},
	{0x3802, 0x00},
	{0x3624, 0x47},
	{0x3621, 0xac},
	{0x3638, 0x25},
	{0x3635, 0x40},
	{0x363b, 0x08},
	{0x363c, 0x05},
	{0x363d, 0x05},
	{0x3324, 0x02},
	{0x3325, 0x02},
	{0x333d, 0x08},
	{0x3314, 0x04},
	{0x36fa, 0x28},
	{0x3e14, 0xb0},
	{0x3e1e, 0x35},
	{0x3e0e, 0x66},
	{0x4837, 0x31},
	{0x6000, 0x00},
	{0x6002, 0x00},
	{0x301c, 0x78},
	{0x3037, 0x44},
	{0x3038, 0x44},
	{0x3632, 0x18},
	{0x4809, 0x01},
	{0x5000, 0x06},
	{0x5780, 0x7f},
	{0x57a0, 0x00},
	{0x57a1, 0x74},
	{0x57a2, 0x01},
	{0x57a3, 0xf4},
	{0x5781, 0x06},
	{0x5782, 0x04},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x16},
	{0x5786, 0x12},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x4501, 0xb4},
	{0x4509, 0x20},
	{0x3364, 0x1d},
	{0x33b6, 0x07},
	{0x33b7, 0x07},
	{0x33b8, 0x10},
	{0x33b9, 0x10},
	{0x33ba, 0x10},
	{0x33bb, 0x07},
	{0x33bc, 0x07},
	{0x33bd, 0x20},
	{0x33be, 0x20},
	{0x33bf, 0x20},
	{0x360f, 0x05},
	{0x367a, 0x40},
	{0x367b, 0x40},
	{0x3671, 0xf6},
	{0x3672, 0x16},
	{0x3673, 0x16},
	{0x366e, 0x04},
	{0x3670, 0x4a},
	{0x367c, 0x40},
	{0x367d, 0x58},
	{0x3674, 0xc8},
	{0x3675, 0x54},
	{0x3676, 0x18},
	{0x367e, 0x40},
	{0x367f, 0x58},
	{0x3677, 0x22},
	{0x3678, 0x33},
	{0x3679, 0x44},
	{0x36a0, 0x58},
	{0x36a1, 0x78},
	{0x3696, 0x83},
	{0x3697, 0x87},
	{0x3698, 0x9f},
	{0x3637, 0x17},
	{0x331e, 0x11},
	{0x331f, 0x21},
	{0x3303, 0x1c},
	{0x3309, 0x3c},
	{0x330a, 0x00},
	{0x330b, 0xc8},
	{0x3306, 0x68},
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x00},
	{0x3203, 0x04},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3380, 0x1b},
	{0x3341, 0x07},
	{0x3343, 0x03},
	{0x3e25, 0x03},
	{0x3e26, 0x40},
	{0x3e00, 0x00},
	{0x3e01, 0x95},
	{0x3e02, 0xa0},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3905, 0xd8},
	{0x36e9, 0x23},
	{0x36f9, 0x05},
	{0x0100, 0x01},
	{SC2310_REG_DELAY, 10},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_init_regs_1920_1080_25fps_dvp[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0xa3},//bypass pll1
	{0x36f9, 0x85},//bypass pll2
	{0x337f, 0x03},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x08},
	{0x330e, 0x30},
	{0x3326, 0x00},
	{0x3631, 0x88},
	{0x3640, 0x00},
	{0x3636, 0x65},
	{0x3907, 0x01},
	{0x3908, 0x01},
	{0x3320, 0x01},
	{0x3366, 0x70},
	{0x57a4, 0xf0},
	{0x3333, 0x30},
	{0x331b, 0x83},
	{0x3334, 0x40},
	{0x3302, 0x10},
	{0x3308, 0x08},
	{0x3622, 0xe6},
	{0x3633, 0x22},
	{0x3630, 0xc8},
	{0x3301, 0x10},
	{0x33aa, 0x00},
	{0x391e, 0x00},
	{0x391f, 0xc0},
	{0x3634, 0x44},
	{0x4500, 0x59},
	{0x3623, 0x18},
	{0x3f08, 0x04},
	{0x3f00, 0x0d},
	{0x336c, 0x42},
	{0x3933, 0x28},
	{0x3934, 0x0a},
	{0x3940, 0x1b},
	{0x3941, 0x40},
	{0x3942, 0x08},
	{0x3943, 0x0e},
	{0x36ea, 0x77},
	{0x36eb, 0x0b},
	{0x36ec, 0x0f},
	{0x36ed, 0x03},
	{0x36fb, 0x10},
	{0x320c, 0x04},
	{0x320d, 0x65},
	{0x320e, 0x05},
	{0x320f, 0xa0},
	{0x3235, 0x09},
	{0x3236, 0x5e},
	{0x3f04, 0x02},
	{0x3f05, 0x2a},
	{0x3802, 0x00},
	{0x3624, 0x47},
	{0x3621, 0xac},
	{0x3019, 0xff},
	{0x301c, 0xb4},
	{0x3018, 0x1f},
	{0x3031, 0x0a},
	{0x3001, 0xff},
	{0x4603, 0x01},
	{0x4837, 0x36},
	{0x303f, 0x81},
	{0x3d08, 0x01},
	{0x3638, 0x25},
	{0x3635, 0x40},
	{0x363b, 0x08},
	{0x363c, 0x05},
	{0x363d, 0x05},
	{0x3641, 0x01},
	{0x3324, 0x02},
	{0x3325, 0x02},
	{0x333d, 0x08},
	{0x3314, 0x04},
	{0x36fa, 0x28},
	{0x3e14, 0xb0},
	{0x3e1e, 0x35},
	{0x3e0e, 0x66},
	{0x6000, 0x00},
	{0x6002, 0x00},
	{0x3037, 0x24},
	{0x3038, 0x44},
	{0x3632, 0x18},
	{0x4809, 0x01},
	{0x5000, 0x06},
	{0x5780, 0x7f},
	{0x57a0, 0x00},
	{0x57a1, 0x74},
	{0x57a2, 0x01},
	{0x57a3, 0xf4},
	{0x5781, 0x06},
	{0x5782, 0x04},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x16},
	{0x5786, 0x12},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x4501, 0xb4},
	{0x4509, 0x20},
	{0x3364, 0x1d},
	{0x33b6, 0x07},
	{0x33b7, 0x07},
	{0x33b8, 0x10},
	{0x33b9, 0x10},
	{0x33ba, 0x10},
	{0x33bb, 0x07},
	{0x33bc, 0x07},
	{0x33bd, 0x20},
	{0x33be, 0x20},
	{0x33bf, 0x20},
	{0x360f, 0x05},
	{0x367a, 0x40},
	{0x367b, 0x40},
	{0x3671, 0xf6},
	{0x3672, 0x16},
	{0x3673, 0x16},
	{0x366e, 0x04},
	{0x3670, 0x4a},
	{0x367c, 0x40},
	{0x367d, 0x58},
	{0x3674, 0xc8},
	{0x3675, 0x54},
	{0x3676, 0x18},
	{0x367e, 0x40},
	{0x367f, 0x58},
	{0x3677, 0x22},
	{0x3678, 0x33},
	{0x3679, 0x44},
	{0x36a0, 0x58},
	{0x36a1, 0x78},
	{0x3696, 0x83},
	{0x3697, 0x87},
	{0x3698, 0x9f},
	{0x3637, 0x17},
	{0x331e, 0x11},
	{0x331f, 0x21},
	{0x3303, 0x1c},
	{0x3309, 0x3c},
	{0x330a, 0x00},
	{0x330b, 0xc8},
	{0x3306, 0x68},
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x00},
	{0x3203, 0x04},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3380, 0x1b},
	{0x3341, 0x07},
	{0x3343, 0x03},
	{0x3e25, 0x03},
	{0x3e26, 0x40},
	{0x3e00, 0x00},
	{0x3e01, 0x95},
	{0x3e02, 0xa0},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3905, 0xd8},
	{0x36e9, 0x23},
	{0x36f9, 0x05},
	{0x0100, 0x01},
	{SC2310_REG_DELAY, 10},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};




/*
 * the part of driver was fixed.
 */

static struct regval_list sc2310_stream_on_dvp[] = {
	{0x0100, 0x01},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_stream_off_dvp[] = {
	{0x0100, 0x00},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_stream_on_mipi[] = {
	{0x0100,0x01},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc2310_stream_off_mipi[] = {
	{0x0100, 0x00},
	{SC2310_REG_END, 0x00},	/* END MARKER */
};

static int sc2310_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct sc2310_info *info = to_state(sd);
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

static int sc2310_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct sc2310_info *info = to_state(sd);
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
#if 0
static int sc2310_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != SC2310_REG_END) {
		if (vals->reg_num == SC2310_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc2310_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}
#endif

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int sc2310_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != SC2310_REG_END) {
		if (vals->reg_num == SC2310_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc2310_write(sd, vals->reg_num, vals->value);
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
static int sc2310_reset(struct v4l2_subdev *sd, u32 val)
{
	struct sc2310_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int sc2310_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct sc2310_info *info = to_state(sd);

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

static int sc2310_init(struct v4l2_subdev *sd, u32 val)
{
	struct sc2310_info *info = to_state(sd);
	int ret = 0;

	ret = sc2310_write_array(sd, info->win->regs);

	return ret;
}



static int sc2310_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = sc2310_read(sd, 0x3107, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC2310_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = sc2310_read(sd, 0x3108, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC2310_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}

static struct sc2310_win_size sc2310_win_sizes[] = {
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 15 << 16 | 1,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 400,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 1,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_WDR_2_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2310_init_regs_1920_1080_15fps_mipi,
	},//[0]
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 400,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2310_init_regs_1920_1080_25fps_mipi,
	},//[1]
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 400,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2310_init_regs_1920_1080_15fps_mipi_dol,
	},//[2]
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 400,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2310_init_regs_1920_1080_25fps_mipi_dol,
	},//[3]
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps		= 25 << 16 | 1,
		.sensor_info.wdr_en		= 1,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc2310_init_regs_1920_1080_25fps_dvp,
	},//[4]
};

static int sc2310_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

/*
 * Set a format.
 */
static int sc2310_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct sc2310_format_struct *ovfmt;
	struct sc2310_win_size *wsize;
	struct sc2310_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int sc2310_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct sc2310_info *info = to_state(sd);
	struct sc2310_win_size *wsize = info->win;
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

static int sc2310_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2310_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2310_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2310_s_vflip(struct v4l2_subdev *sd, int value)
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
static int sc2310_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int sc2310_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc2310_again_lut); i++) {
		lut = &sc2310_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(sc2310_again_lut); i++) {
		lut = &sc2310_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int sc2310_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct sc2310_info *info = to_state(sd);
	unsigned char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	return ret;
}

static int sc2310_g_again_short(struct v4l2_subdev *sd, __s32 *value)
{
	struct sc2310_info *info = to_state(sd);
	unsigned char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int sc2310_s_again(struct v4l2_subdev *sd, int again)
{
	struct sc2310_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	if(again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	} 

	ret = sc2310_write(sd, 0x3e09, (unsigned char)(reg_value & 0xff));
	ret += sc2310_write(sd, 0x3e08, (unsigned char)(reg_value >> 8 & 0x3f));

	return ret;
}

static int sc2310_s_again_short(struct v4l2_subdev *sd, int again)
{
	struct sc2310_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	if(again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	} 

	ret = sc2310_write(sd, 0x3e13, (unsigned char)(reg_value & 0xff));
	ret += sc2310_write(sd, 0x3e12, (unsigned char)(reg_value >> 8 & 0x3f));

	return ret;
}

/*
 * Tweak autogain.
 */
static int sc2310_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc2310_s_exp(struct v4l2_subdev *sd, int value)
{
	struct sc2310_info *info = to_state(sd);
	int ret = 0;
	unsigned int expo = 0;
#ifdef SC2310_WDR_EN
	expo = value*4;
	ret += sc2310_write(sd, 0x3e00, (unsigned char)((expo >> 12) & 0x0f));
	ret += sc2310_write(sd, 0x3e01, (unsigned char)((expo >> 4) & 0xff));
	ret += sc2310_write(sd, 0x3e02, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x14);
		ret += sc2310_write(sd, 0x3812, 0x30);
	} else if(value > 0xa0){
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x04);
		ret += sc2310_write(sd, 0x3812, 0x30);
	}
#else
	expo = value*2;
	ret += sc2310_write(sd, 0x3e00, (unsigned char)((expo >> 12) & 0x0f));
	ret += sc2310_write(sd, 0x3e01, (unsigned char)((expo >> 4) & 0xff));
	ret += sc2310_write(sd, 0x3e02, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x14);
		ret += sc2310_write(sd, 0x3812, 0x30);
	} else if(value > 0xa0){
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x04);
		ret += sc2310_write(sd, 0x3812, 0x30);
	}
#endif

	return ret;
}

static int sc2310_s_exp_short(struct v4l2_subdev *sd, int value)
{
	struct sc2310_info *info = to_state(sd);
	int ret = 0;
	unsigned int expo = 0;
	ret += sc2310_write(sd, 0x3e04, (unsigned char)((expo >> 4) & 0xff));
	ret += sc2310_write(sd, 0x3e05, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x14);
		ret += sc2310_write(sd, 0x3812, 0x30);
	}
	else if(value > 0xa0){
		ret += sc2310_write(sd, 0x3812, 0x00);
		ret += sc2310_write(sd, 0x3314, 0x04);
		ret += sc2310_write(sd, 0x3812, 0x30);
	}

	return ret;
}

static int sc2310_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc2310_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return sc2310_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc2310_g_again(sd, &info->again->val);
	case V4L2_CID_USER_ANALOG_GAIN_SHORT:
		return sc2310_g_again_short(sd, &info->again_short->val);
	}
	return -EINVAL;
}

static int sc2310_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc2310_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sc2310_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return sc2310_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return sc2310_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return sc2310_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* sc2310_s_gain turns off auto gain */
			return sc2310_s_gain(sd, info->gain->val);
		}
		return sc2310_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return sc2310_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc2310_s_again(sd, ctrl->val);
	case V4L2_CID_USER_ANALOG_GAIN_SHORT:
		return sc2310_s_again_short(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sc2310_s_exp(sd, ctrl->val);
	case V4L2_CID_USER_EXPOSURE_SHORT:
		return sc2310_s_exp_short(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc2310_ctrl_ops = {
	.s_ctrl = sc2310_s_ctrl,
	.g_volatile_ctrl = sc2310_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc2310_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = sc2310_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 2;
	return ret;
}

static int sc2310_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	sc2310_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int sc2310_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc2310_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		sc2310_power_init_seq(info);
		sc2310_init(sd, 1);
		ret = sc2310_write_array(sd, sc2310_stream_on_mipi);
		printk("--------------------->>>>>sc2310 stream on\n");

	}
	else {
		ret = sc2310_write_array(sd, sc2310_stream_off_mipi);
		printk("--------------------->>>>>>>>sc2310 stream off\n");
	}
	return ret;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sc2310_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sc2310_g_register,
	.s_register = sc2310_s_register,
#endif

};

static const struct v4l2_subdev_video_ops sc2310_video_ops = {
	.s_stream = sc2310_s_stream,
};

static const struct v4l2_subdev_pad_ops sc2310_pad_ops = {
	//.enum_frame_interval = sc2310_enum_frame_interval,
	//.num_frame_size = sc2310_enum_frame_size,
	//.enum_mbus_code = sc2310_enum_mbus_code,
	.set_fmt = sc2310_set_fmt,
	.get_fmt = sc2310_get_fmt,
};

static const struct v4l2_subdev_ops sc2310_ops = {
	.core = &sc2310_core_ops,
	.video = &sc2310_video_ops,
	.pad = &sc2310_pad_ops,
};

void sc2310_power_init_seq(struct sc2310_info *info)
{
	*(volatile unsigned int*)0xb0010100 = 0x1;  //set gpio power domain 1.8v
	gpio_direction_output(4,0);
	udelay(50);
	if(info->pwen.pin == 3){
		gpio_direction_output(info->pwen.pin,info->pwen.active_level);
		udelay(50);
	}
	gpio_direction_output(info->pwen.pin,info->pwen.active_level);
	udelay(50);

	gpio_direction_output(info->reset.pin,!info->reset.active_level);
	msleep(40);
	gpio_direction_output(info->reset.pin,info->reset.active_level);
	msleep(40);
	gpio_direction_output(info->reset.pin,!info->reset.active_level);
	msleep(40);

	gpio_direction_output(info->pwdn.pin,info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin,!info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin,info->pwdn.active_level);
	msleep(10);
}
/* ----------------------------------------------------------------------- */

static int sc2310_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct sc2310_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned long rate;
	struct v4l2_ctrl_config cfg = {0};
	int mclk_index = -1;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutp-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->ircutp.pin = gpio;
		info->ircutp.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->ircutn.pin = gpio;
		info->ircutn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwen-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwen.pin = gpio;
		info->pwen.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwdn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	sc2310_power_init_seq(info);
	v4l2_i2c_subdev_init(sd, client, &sc2310_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
#if 1

	char id_div[9];
	char id_mux[9];
	printk("%s:%d\n", __func__, __LINE__);
	of_property_read_u32(client->dev.of_node, "ingenic,mclk", &mclk_index);
	if(mclk_index == 0) {
		memcpy(id_div, "div_cim0", sizeof(id_div));
		memcpy(id_mux, "mux_cim0", sizeof(id_mux));
	} else if(mclk_index == 1) {
		memcpy(id_div, "div_cim1", sizeof(id_div));
		memcpy(id_mux, "mux_cim1", sizeof(id_mux));
	} else if(mclk_index == 2) {
		memcpy(id_div, "div_cim2", sizeof(id_div));
		memcpy(id_mux, "mux_cim2", sizeof(id_mux));
	} else
		printk("Unkonwn mclk index\n");


	info->clk = v4l2_clk_get(&client->dev, id_div);
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
#endif


	//sc2310_reset(sd, 1);
#if 1
	/* Make sure it's an sc2310 */
	ret = sc2310_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an sc2310 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	//sc2310_ircut(sd, 0);
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 356010, 1, 10000);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &sc2310_ctrl_ops,
			V4L2_CID_EXPOSURE, 2, 0x5a0 - 3, 1, 1000);

#if 1
	cfg.ops = &sc2310_ctrl_ops;
	cfg.id = V4L2_CID_USER_EXPOSURE_SHORT;
	cfg.name = "expo short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 5;
	cfg.max = 0x5a0 - 3;
	cfg.step = 1;
	cfg.def = 1000;
	info->exposure_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);

	memset(&cfg, 0, sizeof(cfg));
	cfg.ops = &sc2310_ctrl_ops;
	cfg.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	cfg.name = "analog gain short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 0;
	cfg.max = 356010;
	cfg.step = 1;
	cfg.def = 10000;
	info->again_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);
#endif

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &sc2310_win_sizes[1];

	sc2310_init(sd, 1);
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "sc2310 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int sc2310_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2310_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id sc2310_id[] = {
	{ "sc2310", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2310_id);

static const struct of_device_id sc2310_of_match[] = {
	{.compatible = "smartsens,sc2310", },
	{},
};
MODULE_DEVICE_TABLE(of,	sc2310_of_match);


static struct i2c_driver sc2310_driver = {
	.driver = {
		.name	= "sc2310",
		.of_match_table = of_match_ptr(sc2310_of_match),
	},
	.probe		= sc2310_probe,
	.remove		= sc2310_remove,
	.id_table	= sc2310_id,
};

module_i2c_driver(sc2310_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for SmartSens sc2310 sensors");
MODULE_LICENSE("GPL");
