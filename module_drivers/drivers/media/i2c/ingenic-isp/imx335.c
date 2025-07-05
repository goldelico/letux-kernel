/*
 * A V4L2 driver for Sony imx335 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 *
 * imx335 datasheet description:
 * Input frequency: 6 to 27 MHz / 37.125 MHz / 74.25 MHz
 * ◆ Built-in timing adjustment circuit, H/V driver and serial communication circuit
 * ◆ Number of recommended recording pixels: 2592 (H) × 1944 (V) approx. 5.04M pixel
 * uboot epll clock config:#define CONFIG_SYS_EPLL_FREQ           297000000
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
#include <linux/clk.h>

#include <isp-sensor.h>

#define IMX335_CHIP_ID_H	(0x08)
#define IMX335_CHIP_ID_L	(0x00)
#define IMX335_REG_CHIP_ID_HIGH         0x3112
#define IMX335_REG_CHIP_ID_LOW          0x3113
#define IMX335_REG_END		0xffff
#define IMX335_REG_DELAY	0x00
#define IMX335_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct imx335_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct imx335_gpio {
	int pin;
	int active_level;
};

struct imx335_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct imx335_win_size *win;

	struct imx335_gpio pwr;

	struct imx335_gpio reset;
	struct imx335_gpio pwdn;
	struct imx335_gpio ircutp;
	struct imx335_gpio ircutn;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;
	unsigned int gain;
};

static inline struct imx335_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx335_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx335_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};


struct again_lut imx335_again_lut[] = {
	/* analog gain */
	{0x0, 0},
	{0x1, 3265},
	{0x2, 6531},
	{0x3, 9796},
	{0x4, 13062},
	{0x5, 16327},
	{0x6, 19593},
	{0x7, 22859},
	{0x8, 26124},
	{0x9, 29390},
	{0xa, 32655},
	{0xb, 35921},
	{0xc, 39187},
	{0xd, 42452},
	{0xe, 45718},
	{0xf, 48983},
	{0x10, 52249},
	{0x11, 55514},
	{0x12, 58780},
	{0x13, 62046},
	{0x14, 65311},
	{0x15, 68577},
	{0x16, 71842},
	{0x17, 75108},
	{0x18, 78374},
	{0x19, 81639},
	{0x1a, 84905},
	{0x1b, 88170},
	{0x1c, 91436},
	{0x1d, 94702},
	{0x1e, 97967},
	{0x1f, 101233},
	{0x20, 104498},
	{0x21, 107764},
	{0x22, 111029},
	{0x23, 114295},
	{0x24, 117561},
	{0x25, 120826},
	{0x26, 124092},
	{0x27, 127357},
	{0x28, 130623},
	{0x29, 133889},
	{0x2a, 137154},
	{0x2b, 140420},
	{0x2c, 143685},
	{0x2d, 146951},
	{0x2e, 150217},
	{0x2f, 153482},
	{0x30, 156748},
	{0x31, 160013},
	{0x32, 163279},
	{0x33, 166544},
	{0x34, 169810},
	{0x35, 173076},
	{0x36, 176341},
	{0x37, 179607},
	{0x38, 182872},
	{0x39, 186138},
	{0x3a, 189404},
	{0x3b, 192669},
	{0x3c, 195935},
	{0x3d, 199200},
	{0x3e, 202466},
	{0x3f, 205732},
	{0x40, 208997},
	{0x41, 212263},
	{0x42, 215528},
	{0x43, 218794},
	{0x44, 222059},
	{0x45, 225325},
	{0x46, 228591},
	{0x47, 231856},
	{0x48, 235122},
	{0x49, 238387},
	{0x4a, 241653},
	{0x4b, 244919},
	{0x4c, 248184},
	{0x4d, 251450},
	{0x4e, 254715},
	{0x4f, 257981},
	{0x50, 261247},
	{0x51, 264512},
	{0x52, 267778},
	{0x53, 271043},
	{0x54, 274309},
	{0x55, 277574},
	{0x56, 280840},
	{0x57, 284106},
	{0x58, 287371},
	{0x59, 290637},
	{0x5a, 293902},
	{0x5b, 297168},
	{0x5c, 300434},
	{0x5d, 303699},
	{0x5e, 306965},
	{0x5f, 310230},
	{0x60, 313496},
	{0x61, 316762},
	{0x62, 320027},
	{0x63, 323293},
	{0x64, 326558},
	/* analog+digital */
	{0x65, 329824},
	{0x66, 333089},
	{0x67, 336355},
	{0x68, 339621},
	{0x69, 342886},
	{0x6a, 346152},
	{0x6b, 349417},
	{0x6c, 352683},
	{0x6d, 355949},
	{0x6e, 359214},
	{0x6f, 362480},
	{0x70, 365745},
	{0x71, 369011},
	{0x72, 372277},
	{0x73, 375542},
	{0x74, 378808},
	{0x75, 382073},
	{0x76, 385339},
	{0x77, 388604},
	{0x78, 391870},
	{0x79, 395136},
	{0x7a, 398401},
	{0x7b, 401667},
	{0x7c, 404932},
	{0x7d, 408198},
	{0x7e, 411464},
	{0x7f, 414729},
	{0x80, 417995},
	{0x81, 421260},
	{0x82, 424526},
	{0x83, 427792},
	{0x84, 431057},
	{0x85, 434323},
	{0x86, 437588},
	{0x87, 440854},
	{0x88, 444119},
	{0x89, 447385},
	{0x8a, 450651},
	{0x8b, 453916},
	{0x8c, 457182},
	{0x8d, 460447},
	{0x8e, 463713},
	{0x8f, 466979},
	{0x90, 470244},
	{0x91, 473510},
	{0x92, 476775},
	{0x93, 480041},
	{0x94, 483307},
	{0x95, 486572},
	{0x96, 489838},
	{0x97, 493103},
	{0x98, 496369},
	{0x99, 499634},
	{0x9a, 502900},/*204x*/
	{0x9b, 506166},
	{0x9c, 509431},
	{0x9d, 512697},
	{0x9e, 515962},
	{0x9f, 519228},
	{0xa0, 522494},
	{0xa1, 525759},
	{0xa2, 529025},
	{0xa3, 532290},
	{0xa4, 535556},
	{0xa5, 538822},
	{0xa6, 542087},
	{0xa7, 545353},
	{0xa8, 548618},
	{0xa9, 551884},
	{0xaa, 555149},
	{0xab, 558415},
	{0xac, 561681},
	{0xad, 564946},
	{0xae, 568212},
	{0xaf, 571477},
	{0xb0, 574743},
	{0xb1, 578009},
	{0xb2, 581274},
	{0xb3, 584540},
	{0xb4, 587805},
	{0xb5, 591071},
	{0xb6, 594337},
	{0xb7, 597602},
	{0xb8, 600868},
	{0xb9, 604133},
	{0xba, 607399},
	{0xbb, 610664},
	{0xbc, 613930},
	{0xbd, 617196},
	{0xbe, 620461},
	{0xbf, 623727},
	{0xc0, 626992},
	{0xc1, 630258},
	{0xc2, 633524},
	{0xc3, 636789},
	{0xc4, 640055},
	{0xc5, 643320},
	{0xc6, 646586},
	{0xc7, 649852},
	{0xc8, 653117},
	{0xc9, 656383},
	{0xca, 659648},
	{0xcb, 662914},
	{0xcc, 666179},
	{0xcd, 669445},
	{0xce, 672711},
	{0xcf, 675976},
	{0xd0, 679242},
	{0xd1, 682507},
	{0xd2, 685773},
	{0xd3, 689039},
	{0xd4, 692304},
	{0xd5, 695570},
	{0xd6, 698835},
	{0xd7, 702101},
	{0xd8, 705367},
	{0xd9, 708632},
	{0xda, 711898},
	{0xdb, 715163},
	{0xdc, 718429},
	{0xdd, 721694},
	{0xde, 724960},
	{0xdf, 728226},
	{0xe0, 731491},
	{0xe1, 734757},
	{0xe2, 738022},
	{0xe3, 741288},
	{0xe4, 744554},
	{0xe5, 747819},
	{0xe6, 751085},
	{0xe7, 754350},
	{0xe8, 757616},
	{0xe9, 760882},
	{0xea, 764147},
	{0xeb, 767413},
	{0xec, 770678},
	{0xed, 773944},
	{0xee, 777209},
	{0xef, 780475},
	{0xf0, 783741},
};

/***********************************/

static struct regval_list imx335_init_regs_2592_1944_15fps_mipi[] = {
	{0x3000, 0x01},
	{0x3001, 0x00},
	{0x3002, 0x01},
	{0x3004, 0x00},
	{0x300C, 0x5B},
	{0x300D, 0x40},
	{0x3018, 0x00},
	{0x302C, 0x30},
	{0x302D, 0x00},
	{0x302E, 0x38},
	{0x302F, 0x0A},
	{0x3030, 0x1d},
	{0x3031, 0x10},
	{0x3032, 0x00},
	{0x3034, 0xb0},
	{0x3035, 0x04},/* 20fps 0x384 */
	{0x3048, 0x00},
	{0x3049, 0x00},
	{0x304A, 0x03},
	{0x304B, 0x01},
	{0x304C, 0x14},
	{0x304E, 0x00},
	{0x304F, 0x00},/* vflip */
	{0x3050, 0x00},
	{0x3056, 0xAC},
	{0x3057, 0x07},
	{0x3058, 0x09},
	{0x3059, 0x00},
	{0x305A, 0x00},
	{0x305C, 0x12},
	{0x305D, 0x00},
	{0x305E, 0x00},
	{0x3060, 0xE8},
	{0x3061, 0x00},
	{0x3062, 0x00},
	{0x3064, 0x09},
	{0x3065, 0x00},
	{0x3066, 0x00},
	{0x3068, 0xCE},
	{0x3069, 0x00},
	{0x306A, 0x00},
	{0x306C, 0x68},
	{0x306D, 0x06},
	{0x306E, 0x00},
	{0x3072, 0x28},
	{0x3073, 0x00},
	{0x3074, 0xB0},
	{0x3075, 0x00},
	{0x3076, 0x58},
	{0x3077, 0x0F},
	{0x3078, 0x01},
	{0x3079, 0x02},
	{0x307A, 0xFF},
	{0x307B, 0x02},
	{0x307C, 0x00},
	{0x307D, 0x00},
	{0x307E, 0x00},
	{0x307F, 0x00},
	{0x3080, 0x01},
	{0x3081, 0x02},
	{0x3082, 0xFF},
	{0x3083, 0x02},
	{0x3084, 0x00},
	{0x3085, 0x00},
	{0x3086, 0x00},
	{0x3087, 0x00},
	{0x30A4, 0x33},
	{0x30A8, 0x10},
	{0x30A9, 0x04},
	{0x30AC, 0x00},
	{0x30AD, 0x00},
	{0x30B0, 0x10},
	{0x30B1, 0x08},
	{0x30B4, 0x00},
	{0x30B5, 0x00},
	{0x30B6, 0x00},
	{0x30B7, 0x00},
	{0x30C6, 0x00},
	{0x30C7, 0x00},
	{0x30CE, 0x00},
	{0x30CF, 0x00},
	{0x30D8, 0x4C},
	{0x30D9, 0x10},
	{0x30E8, 0x00},
	{0x30E9, 0x00},
	{0x30EA, 0x00},
	{0x30EB, 0x00},
	{0x30EC, 0x00},
	{0x30ED, 0x00},
	{0x30EE, 0x00},
	{0x30EF, 0x00},
	{0x3112, 0x08},
	{0x3113, 0x00},
	{0x3116, 0x08},
	{0x3117, 0x00},
	{0x314C, 0x80},
	{0x314D, 0x00},
	{0x315A, 0x06},
	{0x3167, 0x01},
	{0x3168, 0x68},
	{0x316A, 0x7E},
	{0x3199, 0x00},
	{0x319D, 0x00},
	{0x319E, 0x03},
	{0x31A0, 0x2A},
	{0x31A1, 0x00},
	{0x31D4, 0x00},
	{0x31D5, 0x00},
	{0x3288, 0x21},
	{0x328A, 0x02},
	{0x3300, 0x00},
	{0x3302, 0x32},
	{0x3303, 0x00},
	{0x3414, 0x05},
	{0x3416, 0x18},
	{0x341C, 0xff},//0xFF},
	{0x341D, 0x01},
	{0x3648, 0x01},
	{0x364A, 0x04},
	{0x364C, 0x04},
	{0x3678, 0x01},
	{0x367C, 0x31},
	{0x367E, 0x31},
	{0x3706, 0x10},
	{0x3708, 0x03},
	{0x3714, 0x02},
	{0x3715, 0x02},
	{0x3716, 0x01},
	{0x3717, 0x03},
	{0x371C, 0x3D},
	{0x371D, 0x3F},
	{0x372C, 0x00},
	{0x372D, 0x00},
	{0x372E, 0x46},
	{0x372F, 0x00},
	{0x3730, 0x89},
	{0x3731, 0x00},
	{0x3732, 0x08},
	{0x3733, 0x01},
	{0x3734, 0xFE},
	{0x3735, 0x05},
	{0x3740, 0x02},
	{0x375D, 0x00},
	{0x375E, 0x00},
	{0x375F, 0x11},
	{0x3760, 0x01},
	{0x3768, 0x1B},
	{0x3769, 0x1B},
	{0x376A, 0x1B},
	{0x376B, 0x1B},
	{0x376C, 0x1A},
	{0x376D, 0x17},
	{0x376E, 0x0F},
	{0x3776, 0x00},
	{0x3777, 0x00},
	{0x3778, 0x46},
	{0x3779, 0x00},
	{0x377A, 0x89},
	{0x377B, 0x00},
	{0x377C, 0x08},
	{0x377D, 0x01},
	{0x377E, 0x23},
	{0x377F, 0x02},
	{0x3780, 0xD9},
	{0x3781, 0x03},
	{0x3782, 0xF5},
	{0x3783, 0x06},
	{0x3784, 0xA5},
	{0x3788, 0x0F},
	{0x378A, 0xD9},
	{0x378B, 0x03},
	{0x378C, 0xEB},
	{0x378D, 0x05},
	{0x378E, 0x87},
	{0x378F, 0x06},
	{0x3790, 0xF5},
	{0x3792, 0x43},
	{0x3794, 0x7A},
	{0x3796, 0xA1},
	{0x37B0, 0x36},
	{0x3A01, 0x01},
	{0x3A04, 0x48},
	{0x3A05, 0x09},
	{0x3A18, 0x67},
	{0x3A19, 0x00},
	{0x3A1A, 0x27},
	{0x3A1B, 0x00},
	{0x3A1C, 0x27},
	{0x3A1D, 0x00},
	{0x3A1E, 0xB7},
	{0x3A1F, 0x00},
	{0x3A20, 0x2F},
	{0x3A21, 0x00},
	{0x3A22, 0x4F},
	{0x3A23, 0x00},
	{0x3A24, 0x2F},
	{0x3A25, 0x00},
	{0x3A26, 0x47},
	{0x3A27, 0x00},
	{0x3A28, 0x27},
	{0x3A29, 0x00},

	/*pattern generator*/
#ifdef IMX335_TESTPATTERN
	{0x3148, 0x10},
	{0x3280, 0x00},
	{0x329c, 0x01},
	{0x329e, 0x0b},
	{0x32a0, 0x11},/*tpg colorwidth*/
#endif
	{0x3002, 0x00},
	{0x3000, 0x00},

	{IMX335_REG_END, 0x00},/* END MARKER */
};

static struct regval_list imx335_init_regs_2592_1944_25fps_mipi[] = {
	{0x3000, 0x01},
	{0x3001, 0x00},
	{0x3002, 0x01},
	{0x3004, 0x00},
	{0x300c, 0x5b},
	{0x300d, 0x40},
	{0x3018, 0x00},
	{0x302c, 0x30},
	{0x302d, 0x00},
	{0x302e, 0x38},
	{0x302f, 0x0a},
	{0x3030, 0x18},
	{0x3031, 0x15},//1518
	{0x3032, 0x00},
	{0x3034, 0x26},
	{0x3035, 0x02},
	{0x3050, 0x00},
	{0x315a, 0x02},
	{0x316a, 0x7e},
	{0x319d, 0x00},
	{0x31a1, 0x00},
	{0x3288, 0x21},
	{0x328a, 0x02},
	{0x3414, 0x05},
	{0x3416, 0x18},
	{0x341c, 0xff},
	{0x341d, 0x01},
	{0x3648, 0x01},
	{0x364a, 0x04},
	{0x364c, 0x04},
	{0x3678, 0x01},
	{0x367c, 0x31},
	{0x367e, 0x31},
	{0x3706, 0x10},
	{0x3708, 0x03},
	{0x3714, 0x02},
	{0x3715, 0x02},
	{0x3716, 0x01},
	{0x3717, 0x03},
	{0x371c, 0x3d},
	{0x371d, 0x3f},
	{0x372c, 0x00},
	{0x372d, 0x00},
	{0x372e, 0x46},
	{0x372f, 0x00},
	{0x3730, 0x89},
	{0x3731, 0x00},
	{0x3732, 0x08},
	{0x3733, 0x01},
	{0x3734, 0xfe},
	{0x3735, 0x05},
	{0x3740, 0x02},
	{0x375d, 0x00},
	{0x375e, 0x00},
	{0x375f, 0x11},
	{0x3760, 0x01},
	{0x3768, 0x1b},
	{0x3769, 0x1b},
	{0x376a, 0x1b},
	{0x376b, 0x1b},
	{0x376c, 0x1a},
	{0x376d, 0x17},
	{0x376e, 0x0f},
	{0x3776, 0x00},
	{0x3777, 0x00},
	{0x3778, 0x46},
	{0x3779, 0x00},
	{0x377a, 0x89},
	{0x377b, 0x00},
	{0x377c, 0x08},
	{0x377d, 0x01},
	{0x377e, 0x23},
	{0x377f, 0x02},
	{0x3780, 0xd9},
	{0x3781, 0x03},
	{0x3782, 0xf5},
	{0x3783, 0x06},
	{0x3784, 0xa5},
	{0x3788, 0x0f},
	{0x378a, 0xd9},
	{0x378b, 0x03},
	{0x378c, 0xeb},
	{0x378d, 0x05},
	{0x378e, 0x87},
	{0x378f, 0x06},
	{0x3790, 0xf5},
	{0x3792, 0x43},
	{0x3794, 0x7a},
	{0x3796, 0xa1},
	{0x3a01, 0x01},
	/*pattern generator*/
#ifdef IMX335_TESTPATTERN
	{0x3148, 0x10},
	{0x3280, 0x00},
	{0x329c, 0x01},
	{0x329e, 0x0b},
	{0x32a0, 0x11},/*tpg colorwidth*/
#endif
	{0x3002, 0x00},
	{0x3000, 0x00},
	{IMX335_REG_END, 0x00},/* END MARKER */
};

/*
 * the part of driver was fixed.
 */
static struct regval_list imx335_stream_on_mipi[] = {
	{0x3000, 0x00},
	{IMX335_REG_END, 0x00},
};

static struct regval_list imx335_stream_off_mipi[] = {
	{0x3000, 0x01},
	{IMX335_REG_END, 0x00},
};

int imx335_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
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

static int imx335_write(struct v4l2_subdev *sd, unsigned short reg,
			unsigned char value)
{
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

static int imx335_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != IMX335_REG_END) {
		if (vals->reg_num == IMX335_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = imx335_read(sd, vals->reg_num, &val);
			if (ret < 0)
			{
				printk("array: 0x%02x, 0x%02x\n",vals->reg_num, val);
				return ret;
			}
		}
		vals++;
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int imx335_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != IMX335_REG_END) {
		if (vals->reg_num == IMX335_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = imx335_write(sd, vals->reg_num, vals->value);
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
static int imx335_reset(struct v4l2_subdev *sd, u32 val)
{
	struct imx335_info *info = to_state(sd);
		gpio_direction_output(info->pwdn.pin, info->pwdn.active_level);
//		printk("==========file=%s,func=%s,line=%d,pwdn=%d,level=%d,\n\n",__FILE__,__func__,__LINE__,info->pwdn.pin,info->pwdn.active_level);
	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int imx335_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct imx335_info *info = to_state(sd);

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

static int imx335_init(struct v4l2_subdev *sd, u32 val)
{
	struct imx335_info *info = to_state(sd);
	int ret = 0;

	ret = imx335_write_array(sd, info->win->regs);

	return ret;
}




static int imx335_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	ret = imx335_read(sd, IMX335_REG_CHIP_ID_HIGH, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != IMX335_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = imx335_read(sd, IMX335_REG_CHIP_ID_LOW, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != IMX335_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	printk(" imx335 id %x\n", *ident);
	return 0;
}


static struct imx335_win_size imx335_win_sizes[] = {
#if 1
	{
		.sensor_info.mipi_cfg.twidth		= 2616,
		.sensor_info.mipi_cfg.theight		= 1944,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 648, //hcrop(2592-1944)
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 34,  //Margin for color processing
		.sensor_info.fps			= 15 << 16 | 1,

		.width		= 1944,
		.height		= 1944,
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= imx335_init_regs_2592_1944_15fps_mipi,
	},
#endif
	{
		.sensor_info.mipi_cfg.twidth		= 2616,
		.sensor_info.mipi_cfg.theight		= 1944,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 648,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 34,
		.sensor_info.fps			= 25 << 16 | 1,

		.width		= 1944,
		.height		= 1944,
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= imx335_init_regs_2592_1944_25fps_mipi,
	},

};
static const struct imx335_win_size *imx335_select_win(u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(imx335_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(imx335_win_sizes); i++) {
		if ((*width >= imx335_win_sizes[i].width) &&
				(*height >= imx335_win_sizes[i].height)) {
			*width = imx335_win_sizes[i].width;
			*height = imx335_win_sizes[i].height;
			return &imx335_win_sizes[i];
		}
	}

	*width = imx335_win_sizes[default_size].width;
	*height = imx335_win_sizes[default_size].height;
	printk("------w=%d,h=%d--default_size=%d---->line=%d,func=%s\n",*width,*height,default_size,__LINE__,__func__);
	return &imx335_win_sizes[default_size];
}

static int imx335_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_imx335_FMTS)
		return -EINVAL;

	code->code = imx335_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int imx335_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct imx335_format_struct *ovfmt;
	struct imx335_win_size *wsize;
	struct imx335_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

//	info->win = imx335_select_win(&format->format.width, &format->format.height);


	return 0;
}

static int imx335_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct imx335_info *info = to_state(sd);
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

	printk("----%s, %d, width: %d, height: %d, code: %x\n",
			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int imx335_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx335_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx335_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx335_s_vflip(struct v4l2_subdev *sd, int value)
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
static int imx335_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int imx335_s_gain(struct v4l2_subdev *sd, int value)
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
	for(i = 0; i < ARRAY_SIZE(imx335_again_lut); i++) {
		lut = &imx335_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(imx335_again_lut); i++) {
		lut = &imx335_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}
#endif

static int imx335_g_again(struct v4l2_subdev *sd, __s32 *value)
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

    ret=imx335_read(sd,0x00b1,&v);
    reg_val|=(v<<6);
    ret+=imx335_read(sd,0x00b2,&v);
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
static int imx335_s_again(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
#if 0
	ret = imx335_write(sd, 0x30e8, (unsigned char)(value & 0xff));
	ret += imx335_write(sd, 0x30e9, (unsigned char)((value >> 8) & 0x07));
	if (ret < 0)
		return ret;
#endif
    return 0;
}

/*
 * Tweak autogain.
 */
static int imx335_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx335_s_exp(struct v4l2_subdev *sd, int value)
{
#if 1
	int ret = 0;

	unsigned short shr0 = 0;
	unsigned short vmax = 0;

	vmax = imx335_win_sizes[0].sensor_info.mipi_cfg.theight;
	printk("====================>>>>>>>>>>>>>>>vmax=%d\n",vmax);
	shr0 = vmax - value;
	printk("====================>>>>>>>>>>>>>>>shr0=%d\n",shr0);
	ret = imx335_write(sd, 0x3058, (unsigned char)(shr0 & 0xff));
	ret += imx335_write(sd, 0x3059, (unsigned char)((shr0 >> 8) & 0xff));
	ret += imx335_write(sd, 0x305a, (unsigned char)((shr0 >> 16) & 0x0f));
	if (0 != ret) {
		IS_ERR("err: imx335_write err\n");
		return ret;
	}
#endif
	return 0;
}

static int imx335_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct imx335_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return imx335_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return imx335_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int imx335_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct imx335_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return imx335_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return imx335_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return imx335_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return imx335_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* imx335_s_gain turns off auto gain */
			return imx335_s_gain(sd, info->gain->val);
		}
		return imx335_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return imx335_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return imx335_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return imx335_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops imx335_ctrl_ops = {
	.s_ctrl = imx335_s_ctrl,
	.g_volatile_ctrl = imx335_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int imx335_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = imx335_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 2;
	return ret;
}

static int imx335_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	imx335_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int imx335_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx335_info *info = to_state(sd);
	int ret = 0;
	int val=0;
	printk("------------>line=%d,func=%s\n",__LINE__,__func__);
	if (enable) {
		imx335_reset(sd,1);
		imx335_init(sd,1);
		ret = imx335_write_array(sd, imx335_stream_on_mipi);
		pr_debug("imx335 stream on\n");
	printk("----on-------->line=%d,func=%s\n",__LINE__,__func__);

	}
	else {
		ret = imx335_write_array(sd, imx335_stream_off_mipi);
		pr_debug("imx335 stream off\n");
	printk("----off-------->line=%d,func=%s\n",__LINE__,__func__);
	}
	return ret;
}

int imx335_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct imx335_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops imx335_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = imx335_g_register,
	.s_register = imx335_s_register,
#endif

};

static const struct v4l2_subdev_video_ops imx335_video_ops = {
	.s_stream = imx335_s_stream,
	.g_frame_interval = imx335_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx335_pad_ops = {
	//.enum_frame_interval = imx335_enum_frame_interval,
	//.num_frame_size = imx335_enum_frame_size,
	//.enum_mbus_code = imx335_enum_mbus_code,
	.set_fmt = imx335_set_fmt,
	.get_fmt = imx335_get_fmt,
};

static const struct v4l2_subdev_ops imx335_ops = {
	.core = &imx335_core_ops,
	.video = &imx335_video_ops,
	.pad = &imx335_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int imx335_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct imx335_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned long rate;

	printk(" imx335 probe \n\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

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

	v4l2_i2c_subdev_init(sd, client, &imx335_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
#if 1

	info->clk = v4l2_clk_get(&client->dev, "div_cim");
#if 1
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}
	info->sclka = devm_clk_get(&client->dev, "mux_cim");

	printk("sclka is %s\n", info->sclka?"NOT NULL":"NULL");
	rate = v4l2_clk_get_rate(info->clk);
	if (((rate / 1000) % 37125) != 0) {
		ret = clk_set_parent(info->sclka, clk_get(NULL, "epll"));
		info->sclka = devm_clk_get(&client->dev, "epll");
		if (IS_ERR(info->sclka)) {
			pr_err("get sclka failed\n");
		} else {
			rate = clk_get_rate(info->sclka);
			if (((rate / 1000) % 37125) != 0) {
				clk_set_rate(info->sclka, 891000000);
			}
		}
	}
#endif

	ret = v4l2_clk_set_rate(info->clk, 37125000);

	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");
#endif

	imx335_reset(sd, 1);

	/* Make sure it's an imx335 */
	ret = imx335_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an imx335 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
//	imx335_ircut(sd, 0);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
		//	V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);
			V4L2_CID_ANALOGUE_GAIN, 0, 405939, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &imx335_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &imx335_win_sizes[0];//720p
//	info->win = &imx335_win_sizes[1];//1080*360
//	info->win = &imx335_win_sizes[2];//720*360
	imx335_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "imx335 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int imx335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx335_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id imx335_id[] = {
	{ "imx335", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx335_id);

static const struct of_device_id imx335_of_match[] = {
	{.compatible = "sony,imx335", },
	{},
};
MODULE_DEVICE_TABLE(of, imx335_of_match);


static struct i2c_driver imx335_driver = {
	.driver = {
		.name	= "imx335",
		.of_match_table = of_match_ptr(imx335_of_match),
	},
	.probe		= imx335_probe,
	.remove		= imx335_remove,
	.id_table	= imx335_id,
};

module_i2c_driver(imx335_driver);
MODULE_DESCRIPTION("A low-level driver for Sony imx335 sensors");
MODULE_LICENSE("GPL");
