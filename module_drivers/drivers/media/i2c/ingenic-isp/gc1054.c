/*
 * A V4L2 driver for OmniVision gc1054 cameras.
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

#include <isp-sensor.h>

#define GC1054_CHIP_ID_H	(0x10)
#define GC1054_CHIP_ID_L	(0x54)
#define GC1054_REG_CHIP_ID_HIGH         0xF0
#define GC1054_REG_CHIP_ID_LOW          0xF1


#define GC1054_REG_END		0xff
#define GC1054_REG_DELAY	0x00
#define GC1054_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct gc1054_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct gc1054_gpio {
	int pin;
	int active_level;
};

struct gc1054_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct gc1054_win_size *win;

	struct gc1054_gpio pwr;

	struct gc1054_gpio reset;
	struct gc1054_gpio cimen;
	struct gc1054_gpio ircutp;
	struct gc1054_gpio ircutn;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};

struct again_lut gc1054_again_lut[] = {
//gc1054_mipi.c add//
    {0x00, 0},
    {0x01, 33220},
    {0x02, 64776},
    {0x03, 98789},
    {0x04, 131801},
    {0x05, 165630},
    {0x06, 197337},
    {0x07, 231174},
    {0x08, 262879},
    {0x09, 297665},
    { 0x0a, 329326},

};

static inline struct gc1054_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc1054_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gc1054_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list gc1054_init_regs_1280_720_30fps_dvp[] = {
//Mclk=24Mhz£¬PCLK=39Mhz
//HD=1726,VD=750,row_time=HD/PCLK=44.25us
//Actual_window_size=1280*720
	/////////////////////////////////////////////////////
	//////////////////////   SYS   //////////////////////
	/////////////////////////////////////////////////////
	{0xf2,0x00},
	{0xf6,0x00},
	{0xfc,0x04},
	{0xf7,0x01},
	{0xf8,0x0c},
	{0xf9,0x00}, // 06
	{0xfa,0x80},
	{0xfc,0x0e},
	/////////////////////////////////////////////////////
	////////////////   ANALOG & CISCTL   ////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x00},
	{0x03,0x02},
	{0x04,0xa6},
	{0x05,0x02}, //HB
	{0x06,0x07},
	{0x07,0x00}, //VB
	{0x08,0x0a},
	{0x09,0x00},

	{0x0a,0x04}, //row start
	{0x0b,0x00},
	{0x0c,0x00}, //col start

 	 {0x0d,0x02},
	 {0x0e,0xd4}, //height 724

	{0x0f,0x05},
	{0x10,0x08}, //width 1288


	{0x17,0xc0},
	{0x18,0x02},
	{0x19,0x08},
	{0x1a,0x18},
	{0x1d,0x12},
	{0x1e,0x50},
	{0x1f,0x80},
	{0x21,0x30},
	{0x23,0xf8},
	{0x25,0x10},
	{0x28,0x20},
	{0x34,0x08}, //data low
	{0x3c,0x10},
	{0x3d,0x0e},
	{0xcc,0x8e},
	{0xcd,0x9a},
	{0xcf,0x70},
	{0xd0,0xa9},
	{0xd1,0xc5},
	{0xd2,0xed}, //data high
	{0xd8,0x3c}, //dacin offset
	{0xd9,0x7a},
	{0xda,0x12},
	{0xdb,0x50},
	{0xde,0x0c},
	{0xe3,0x60},
	{0xe4,0x78},
	{0xfe,0x01},
	{0xe3,0x01},
	{0xe6,0x10}, //ramps offset
	/////////////////////////////////////////////////////
	//////////////////////   ISP   //////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x01},
	{0x80,0x50},
	{0x88,0x23},
	{0x89,0x03},
//	{0x8c,0x01},//bit0:test color bar.
	{0x90,0x01},
	{0x92,0x02}, //crop win 2<=y<=4
	{0x94,0x03}, //crop win 2<=x<=5
	{0x95,0x02}, //crop win height
	{0x96,0xd0},
	{0x97,0x05}, //crop win width
	{0x98,0x00},
	/////////////////////////////////////////////////////
	//////////////////////   BLK   //////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x01},
	{0x40,0x22},
	{0x43,0x03},
	{0x4e,0x3c},
	{0x4f,0x00},
	{0x60,0x00},
	{0x61,0x80},
	/////////////////////////////////////////////////////
	//////////////////////   GAIN   /////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x01},
	{0xb0,0x48},
	{0xb1,0x01},
	{0xb2,0x00},
	{0xb6,0x00},
	{0xfe,0x02},
	{0x01,0x00},
	{0x02,0x01},
	{0x03,0x02},
	{0x04,0x03},
	{0x05,0x04},
	{0x06,0x05},
	{0x07,0x06},
	{0x08,0x0e},
	{0x09,0x16},
	{0x0a,0x1e},
	{0x0b,0x36},
	{0x0c,0x3e},
	{0x0d,0x56},
	{0xfe,0x02},
	{0xb0,0x00}, //col_gain[11:8]
	{0xb1,0x00},
	{0xb2,0x00},
	{0xb3,0x11},
	{0xb4,0x22},
	{0xb5,0x54},
	{0xb6,0xb8},
	{0xb7,0x60},
	{0xb9,0x00}, //col_gain[12]
	{0xba,0xc0},
	{0xc0,0x20}, //col_gain[7:0]
	{0xc1,0x2d},
	{0xc2,0x40},
	{0xc3,0x5b},
	{0xc4,0x80},
	{0xc5,0xb5},
	{0xc6,0x00},
	{0xc7,0x6a},
	{0xc8,0x00},
	{0xc9,0xd4},
	{0xca,0x00},
	{0xcb,0xa8},
	{0xcc,0x00},
	{0xcd,0x50},
	{0xce,0x00},
	{0xcf,0xa1},
	/////////////////////////////////////////////////////
	////////////////////   DARKSUN   ////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x02},
	{0x54,0xf7},
	{0x55,0xf0},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x00},
	{0x5a,0x04},
	/////////////////////////////////////////////////////
	///////////////////////   DD   //////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x04},
	{0x81,0x8a},
	/////////////////////////////////////////////////////
	//////////////////////	 DVI	/////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x03},
	{0x01,0x00},
	{0x02,0x00},
	{0x03,0x00},
	{0x10,0x11},

	{0x15,0x00},
	{0x40,0x01},
	{0x41,0x00},
////////////////////////////////////////////
/////////////   pad enable   ///////////////
////////////////////////////////////////////
	{0xfe,0x00},
	{0xf2,0x0f},

	{GC1054_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list gc1054_init_regs_1080_360_60fps_dvp[] = {
//Mclk=24Mhz，PCLK=39Mhz
//HD=1626,VD=399,row_time=HD/PCLK=41.69us
//Actual_window_size=1280*720
/////////////////////////////////////////////////////
//////////////////////   SYS   //////////////////////
/////////////////////////////////////////////////////
	{0xf2,0x00},
	{0xf6,0x00},
	{0xfc,0x04},
	{0xf7,0x01},
//	{0xf8,0x0c},//pclk 39Mhz.
	{0xf8,0x11},//pclk 54Mhz.
	{0xf9,0x00},
	{0xfa,0x80},
	{0xfc,0x0e},
////////////////////////////////////////////
///////   ANALOG & CISCTL   ////////////////
////////////////////////////////////////////
	{0xfe,0x00},
//	{0x03,0x02},////// exposure 12bit~8bit
//	{0x04,0xa6},/////  exposure 7bit~0bit 720p default:678 ,1080*360<399
	///////////
	{0x03,0x01},
	{0x04,0x08}, //xtang debug :384
	/////////////
	{0x05,0x02}, //HB
	{0x06,0x07},
	{0x07,0x00}, //VB
	{0x08,0x13},//====ori
//	{0x08,0x0a},//====720p def
	/////////////
	{0x09,0x00},
//	{0x0a,0x04}, //row start
	{0x0a,0xb4}, //row start 180
	{0x0b,0x00},
//	{0x0c,0x00}, //col start
	{0x0c,0xc8}, //64 col start 100
	////////////
	{0x0d,0x01},
	{0x0e,0x78}, //height 376
	{0x0f,0x04},
	{0x10,0x48}, //width 1096
	/////////
//	{0x0d,0x02},
//	{0x0e,0xd4},//724
//	{0x0f,0x05},
//	{0x10,0x08}, //1288
	/////////
	{0x17,0xc0},
	{0x18,0x02},
	{0x19,0x08},
	{0x1a,0x18},
	{0x1d,0x12},
	{0x1e,0x50},
	{0x1f,0x80},
	{0x21,0x30},
	{0x23,0xf8},
	{0x25,0x10},
	{0x28,0x20},
	{0x34,0x08}, //data low
	{0x3c,0x10},
	{0x3d,0x0e},
	{0xcc,0x8e},
	{0xcd,0x9a},
	{0xcf,0x70},
	{0xd0,0xa9},
	{0xd1,0xc5},
	{0xd2,0xed}, //data high
	{0xd8,0x3c}, //dacin offset
	{0xd9,0x7a},
	{0xda,0x12},
	{0xdb,0x50},
	{0xde,0x0c},
	{0xe3,0x60},
	{0xe4,0x78},
	{0xfe,0x01},
	{0xe3,0x01},
	{0xe6,0x10}, //ramps offset
///////////////////,/////////////////////////
/////////////   ISP   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0x80,0x50},
	{0x88,0x23},
	{0x89,0x03},
//	{0x8c,0x01},//test color bar
	{0x90,0x01},
	{0x92,0x02}, //crop win 2<=y<=4
	{0x94,0x03}, //crop win 2<=x<=5
	{0x95,0x01}, //crop win height
	{0x96,0x68},
	{0x97,0x04}, //crop win width
	{0x98,0x38},
////////////////////////////////////////////
/////////////   BLK   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0x40,0x22},
	{0x43,0x03},
	{0x4e,0x3c},
	{0x4f,0x00},
	{0x60,0x00},
	{0x61,0x80},
////////////////////////////////////////////
/////////////   GAIN   /////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0xb0,0x48},
	{0xb1,0x01},
	{0xb2,0x00},
	{0xb6,0x00},
	{0xfe,0x02},
	{0x01,0x00},
	{0x02,0x01},
	{0x03,0x02},
	{0x04,0x03},
	{0x05,0x04},
	{0x06,0x05},
	{0x07,0x06},
	{0x08,0x0e},
	{0x09,0x16},
	{0x0a,0x1e},
	{0x0b,0x36},
	{0x0c,0x3e},
	{0x0d,0x56},
	{0xfe,0x02},
	{0xb0,0x00}, //col_gain[11:8]
	{0xb1,0x00},
	{0xb2,0x00},
	{0xb3,0x11},
	{0xb4,0x22},
	{0xb5,0x54},
	{0xb6,0xb8},
	{0xb7,0x60},
	{0xb9,0x00}, //col_gain[12]
	{0xba,0xc0},
	{0xc0,0x20}, //col_gain[7:0]
	{0xc1,0x2d},
	{0xc2,0x40},
	{0xc3,0x5b},
	{0xc4,0x80},
	{0xc5,0xb5},
	{0xc6,0x00},
	{0xc7,0x6a},
	{0xc8,0x00},
	{0xc9,0xd4},
	{0xca,0x00},
	{0xcb,0xa8},
	{0xcc,0x00},
	{0xcd,0x50},
	{0xce,0x00},
	{0xcf,0xa1},
////////////////////////////////////////////
///////////   DARKSUN   ////////////////////
////////////////////////////////////////////
	{0xfe,0x02}, 
	{0x54,0xf7},
	{0x55,0xf0},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x00},
	{0x5a,0x04},
////////////////////////////////////////////
//////////////   DD   //////////////////////
////////////////////////////////////////////
	{0xfe,0x04}, 
	{0x81,0x8a},
////////////////////////////////////////////
/////////////	 MIPI	/////////////////////
////////////////////////////////////////////
	{0xfe,0x03}, 
	{0x01,0x00},
	{0x02,0x00},
	{0x03,0x00},
	{0x10,0x11},
	{0x15,0x00},
	{0x40,0x01},
	{0x41,0x00},
	///////////
	{0x42,0x38},
	{0x43,0x04},// width 1080 (must set!)
////////////////////////////////////////////
/////////////   pad enable   ///////////////
////////////////////////////////////////////
	{0xfe,0x00},
	{0xf2,0x0f},
	{GC1054_REG_END, 0x00},	/* END MARKER */

};
static struct regval_list gc1054_init_regs_540_360_100fps_dvp[] = {
//Mclk=24Mhz，PCLK=39Mhz
//HD=1626,VD=398,row_time=HD/PCLK=25.1us
//Actual_window_size=540*360
/////////////////////////////////////////////////////
//////////////////////   SYS   //////////////////////
/////////////////////////////////////////////////////
	{0xf2,0x00},
	{0xf6,0x00},
	{0xfc,0x04},
	{0xf7,0x01},
//	{0xf8,0x11},// stage:3M,54MHZ.
	{0xf8,0x0c},// stage:3M,39MHZ.
	{0xf9,0x00},
	{0xfa,0x80},
	{0xfc,0x0e},
////////////////////////////////////////////
///////   ANALOG & CISCTL   ////////////////
////////////////////////////////////////////
	{0xfe,0x00},
//	{0x03,0x02},
//	{0x04,0xa6},
	////////
	{0x03,0x01},
//	{0x04,0x08}, //xtang debug:384
	{0x04,0x00}, //gc debug:256
	////////
	{0x05,0x02}, //HB
//	{0x06,0x07},
//	//////
	{0x06,0x06},// xtang debug  hb
	/////
	{0x07,0x00}, //VB
	{0x08,0x12},
	{0x09,0x00},
	{0x0a,0x04}, //row start
	{0x0b,0x00},
	{0x0c,0x00}, //col start
	{0x0d,0x01},
	{0x0e,0x78}, //height 724
//	{0x0f,0x04},
//	{0x10,0x48}, //width 1288
//	////
	{0x0f,0x02},
	{0x10,0x2c}, //debug width 556
//	////
	{0x17,0xc0},
	{0x18,0x02},
	{0x19,0x08},
	{0x1a,0x18},
	{0x1d,0x12},
	{0x1e,0x50},
	{0x1f,0x80},
	{0x21,0x30},
	{0x23,0xf8},
	{0x25,0x10},
	{0x28,0x20},
	{0x34,0x08}, //data low
	{0x3c,0x10},
	{0x3d,0x0e},
	{0xcc,0x8e},
	{0xcd,0x9a},
	{0xcf,0x70},
	{0xd0,0xa9},
	{0xd1,0xc5},
	{0xd2,0xed}, //data high
	{0xd8,0x3c}, //dacin offset
	{0xd9,0x7a},
	{0xda,0x12},
	{0xdb,0x50},
	{0xde,0x0c},
	{0xe3,0x60},
	{0xe4,0x78},
	{0xfe,0x01},
	{0xe3,0x01},
	{0xe6,0x10}, //ramps offset
////////////////////////////////////////////
/////////////   ISP   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01},
	{0x80,0x50},
	{0x88,0x23},
	{0x89,0x03},
	{0x90,0x01},
	{0x92,0x02}, //crop win 2<=y<=4
	{0x94,0x03}, //crop win 2<=x<=5
	{0x95,0x01}, //crop win height
	{0x96,0x68},
	{0x97,0x02}, //crop win width
	{0x98,0x1c},
////////////////////////////////////////////
/////////////   BLK   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01},
	{0x40,0x22},
	{0x43,0x03},
	{0x4e,0x3c},
	{0x4f,0x00},
	{0x60,0x00},
	{0x61,0x80},
////////////////////////////////////////////
/////////////   GAIN   /////////////////////
////////////////////////////////////////////
	{0xfe,0x01},
	{0xb0,0x48},
	{0xb1,0x01},
	{0xb2,0x00},
	{0xb6,0x00},
	{0xfe,0x02},
	{0x01,0x00},
	{0x02,0x01},
	{0x03,0x02},
	{0x04,0x03},
	{0x05,0x04},
	{0x06,0x05},
	{0x07,0x06},
	{0x08,0x0e},
	{0x09,0x16},
	{0x0a,0x1e},
	{0x0b,0x36},
	{0x0c,0x3e},
	{0x0d,0x56},
	{0xfe,0x02},
	{0xb0,0x00}, //col_gain[11:8]
	{0xb1,0x00},
	{0xb2,0x00},
	{0xb3,0x11},
	{0xb4,0x22},
	{0xb5,0x54},
	{0xb6,0xb8},
	{0xb7,0x60},
	{0xb9,0x00}, //col_gain[12]
	{0xba,0xc0},
	{0xc0,0x20}, //col_gain[7:0]
	{0xc1,0x2d},
	{0xc2,0x40},
	{0xc3,0x5b},
	{0xc4,0x80},
	{0xc5,0xb5},
	{0xc6,0x00},
	{0xc7,0x6a},
	{0xc8,0x00},
	{0xc9,0xd4},
	{0xca,0x00},
	{0xcb,0xa8},
	{0xcc,0x00},
	{0xcd,0x50},
	{0xce,0x00},
	{0xcf,0xa1},
////////////////////////////////////////////
///////////   DARKSUN   ////////////////////
////////////////////////////////////////////
	{0xfe,0x02},
	{0x54,0xf7},
	{0x55,0xf0},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x00},
	{0x5a,0x04},
////////////////////////////////////////////
//////////////   DD   //////////////////////
////////////////////////////////////////////
	{0xfe,0x04},
	{0x81,0x8a},
////////////////////////////////////////////
/////////////	 MIPI	/////////////////////
////////////////////////////////////////////
	{0xfe,0x03},
	{0x01,0x00},
	{0x02,0x00},
	{0x03,0x00},
	{0x10,0x11},
	{0x15,0x00},
	{0x40,0x01},
	{0x41,0x00},
	{0x42,0x1c},
	{0x43,0x02},
////////////////////////////////////////////
/////////////   pad enable   ///////////////
////////////////////////////////////////////
	{0xfe,0x00},
	{0xf2,0x0f},
	{GC1054_REG_END, 0x00},
};
static struct regval_list gc1054_init_regs_720_360_100fps_dvp[] = {
//Mclk=24Mhz，PCLK=39Mhz
//HD=1626,VD=398,row_time=HD/PCLK=25.1us
//Actual_window_size=720*360
/////////////////////////////////////////////////////
//////////////////////   SYS   //////////////////////
/////////////////////////////////////////////////////
	{0xf2,0x00}, 
	{0xf6,0x00},
	{0xfc,0x04},
	{0xf7,0x01},
	{0xf8,0x11},
	{0xf9,0x00},
	{0xfa,0x80},
	{0xfc,0x0e},
////////////////////////////////////////////
///////   ANALOG & CISCTL   ////////////////
////////////////////////////////////////////
	{0xfe,0x00}, 
	{0x03,0x01}, 
	{0x04,0x00}, 
	{0x05,0x02}, //HB
	{0x06,0x07},
	{0x07,0x00}, //VB
	{0x08,0x12}, 
	{0x09,0x00},
	{0x0a,0x04}, //row start
	{0x0b,0x00},
	{0x0c,0x02}, //col start
	{0x0d,0x01}, 
	{0x0e,0x90}, //height 724
	{0x0f,0x02},
	{0x10,0xe0},//width 1288
	{0x17,0xc0},
	{0x18,0x02},
	{0x19,0x08},
	{0x1a,0x18},
	{0x1d,0x12},
	{0x1e,0x50},
	{0x1f,0x80},
	{0x21,0x30},
	{0x23,0xf8},
	{0x25,0x10},
	{0x28,0x20},
	{0x34,0x08}, //data low
	{0x3c,0x10},
	{0x3d,0x0e},
	{0xcc,0x8e},
	{0xcd,0x9a},
	{0xcf,0x70},
	{0xd0,0xa9},
	{0xd1,0xc5},
	{0xd2,0xed}, //data high
	{0xd8,0x3c}, //dacin offset
	{0xd9,0x7a},
	{0xda,0x12},
	{0xdb,0x50},
	{0xde,0x0c},
	{0xe3,0x60},
	{0xe4,0x78},
	{0xfe,0x01},
	{0xe3,0x01},
	{0xe6,0x10}, //ramps offset
////////////////////////////////////////////
/////////////   ISP   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0x80,0x50},
	{0x88,0x23},
	{0x89,0x03},
	{0x90,0x01}, 
	{0x92,0x01}, //crop win 2<=y<=4
	{0x94,0x02}, //crop win 2<=x<=5
	{0x95,0x01}, //crop win height
	{0x96,0x68},
	{0x97,0x02}, //crop win width
	{0x98,0xd0},
////////////////////////////////////////////
/////////////   BLK   //////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0x40,0x22},
	{0x43,0x03},
	{0x4e,0x3c},
	{0x4f,0x00},
	{0x60,0x00},
	{0x61,0x80},
////////////////////////////////////////////
/////////////   GAIN   /////////////////////
////////////////////////////////////////////
	{0xfe,0x01}, 
	{0xb0,0x48},
	{0xb1,0x01}, 
	{0xb2,0x00}, 
	{0xb6,0x00}, 
	{0xfe,0x02},
	{0x01,0x00},
	{0x02,0x01},
	{0x03,0x02},
	{0x04,0x03},
	{0x05,0x04},
	{0x06,0x05},
	{0x07,0x06},
	{0x08,0x0e},
	{0x09,0x16},
	{0x0a,0x1e},
	{0x0b,0x36},
	{0x0c,0x3e},
	{0x0d,0x56},
	{0xfe,0x02},
	{0xb0,0x00}, //col_gain[11:8]
	{0xb1,0x00},
	{0xb2,0x00},
	{0xb3,0x11},
	{0xb4,0x22},
	{0xb5,0x54},
	{0xb6,0xb8},
	{0xb7,0x60},
	{0xb9,0x00}, //col_gain[12]
	{0xba,0xc0},
	{0xc0,0x20}, //col_gain[7:0]
	{0xc1,0x2d},
	{0xc2,0x40},
	{0xc3,0x5b},
	{0xc4,0x80},
	{0xc5,0xb5},
	{0xc6,0x00},
	{0xc7,0x6a},
	{0xc8,0x00},
	{0xc9,0xd4},
	{0xca,0x00},
	{0xcb,0xa8},
	{0xcc,0x00},
	{0xcd,0x50},
	{0xce,0x00},
	{0xcf,0xa1},
////////////////////////////////////////////
///////////   DARKSUN   ////////////////////
////////////////////////////////////////////
	{0xfe,0x02}, 
	{0x54,0xf7},
	{0x55,0xf0},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x00},
	{0x5a,0x04},
////////////////////////////////////////////
//////////////   DD   //////////////////////
////////////////////////////////////////////
	{0xfe,0x04}, 
	{0x81,0x8a},
////////////////////////////////////////////
/////////////	 MIPI	/////////////////////
////////////////////////////////////////////
	{0xfe,0x03}, 
	{0x01,0x00},
	{0x02,0x00},
	{0x03,0x00},
	{0x10,0x11},
	{0x15,0x00},
	{0x40,0x01},
	{0x41,0x00},
	{0x42,0xd0},
	{0x43,0x02},
////////////////////////////////////////////
/////////////   pad enable   ///////////////
////////////////////////////////////////////
	{0xfe,0x00}, 
	{0xf2,0x0f}, 
	{GC1054_REG_END, 0x00},

};

// reg: 0xf2
//「陌上桑: DVP用这个控制stream on」
//—————————
//「陌上桑: 0x0f写0x00就是关闭输出了」
//—————————
//
static struct regval_list gc1054_stream_on[] = {
	{0xfe, 0x03},
//	{0x10, 0x90},//mipi stream on
	{0x10, 0x11},//dvp fake stream on

	{GC1054_REG_END, 0x00},
};

static struct regval_list gc1054_stream_off[] = {
	{0xfe, 0x03},
	{0x10, 0x00},//fake stream off

	{GC1054_REG_END, 0x00},
};


int gc1054_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
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

static int gc1054_write(struct v4l2_subdev *sd, unsigned char reg,
			unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
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

static int gc1054_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != GC1054_REG_END) {
		if (vals->reg_num == GC1054_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = gc1054_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
//			dev_vdbg(&sd->dev, "array: 0x%02x, 0x%02x",
			pr_debug("array: 0x%02x, 0x%02x\n",
					         vals->reg_num, val);
		}
		vals++;
	}
	return 0;
}
static int gc1054_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != GC1054_REG_END) {
		if (vals->reg_num == GC1054_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = gc1054_write(sd, vals->reg_num, vals->value);
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
static int gc1054_reset(struct v4l2_subdev *sd, u32 val)
{
	struct gc1054_info *info = to_state(sd);
return 0;
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

static int gc1054_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct gc1054_info *info = to_state(sd);
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

static int gc1054_init(struct v4l2_subdev *sd, u32 val)
{
	struct gc1054_info *info = to_state(sd);
	int ret = 0;
//	struct regval_list*reg = info->win->regs;
//	ret = gc1054_write_array(sd, info->win->regs);
//#if 0
//	while(reg->reg_num != GC1054_REG_END)
//	{
//			printk("--->reg:0x%02x,value:[0x%02x]\n",reg->reg_num,reg->value);
//			reg ++;
//
//	}
//#else
//	ret = gc1054_read_array(sd, info->win->regs);
//	if (ret<0)
//#endif

	return ret;
}



static int gc1054_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;


	ret = gc1054_read(sd, GC1054_REG_CHIP_ID_HIGH, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC1054_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = gc1054_read(sd, GC1054_REG_CHIP_ID_LOW, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != GC1054_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	printk(" gc1054 id %x\n", *ident);
	return 0;
}


static struct gc1054_win_size gc1054_win_sizes[] = {
	/* 1280*720 */

	{
		.width		= 1280,
		.height		= 720,
		.sensor_info.fps		= 30 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= gc1054_init_regs_1280_720_30fps_dvp,
	},
	/* 1080*360 */

	{
		.width		= 1080,
		.height		= 360,
		.sensor_info.fps		= 60 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= gc1054_init_regs_1080_360_60fps_dvp,
	},
	
	/* 720*360 */

	{
		.width		= 720,
		.height		= 360,
		.sensor_info.fps		= 100 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= gc1054_init_regs_720_360_100fps_dvp,
	},

};
static const struct gc1054_win_size *gc1054_select_win(u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(gc1054_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(gc1054_win_sizes); i++) {
		if ((*width >= gc1054_win_sizes[i].width) &&
				(*height >= gc1054_win_sizes[i].height)) {
			*width = gc1054_win_sizes[i].width;
			*height = gc1054_win_sizes[i].height;
			return &gc1054_win_sizes[i];
		}
	}

	*width = gc1054_win_sizes[default_size].width;
	*height = gc1054_win_sizes[default_size].height;
//	printk("------w=%d,h=%d--default_size=%d---->line=%d,func=%s\n",*width,*height,default_size,__LINE__,__func__);
	return &gc1054_win_sizes[default_size];
}

static int gc1054_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_GC1054_FMTS)
		return -EINVAL;

	code->code = gc1054_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int gc1054_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct gc1054_format_struct *ovfmt;
	struct gc1054_win_size *wsize;
	struct gc1054_info *info = to_state(sd);
	int ret;

//	if (format->pad)
//		return -EINVAL;

	info->win = gc1054_select_win(&format->format.width, &format->format.height);


	return 0;
}

static int gc1054_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct gc1054_info *info = to_state(sd);
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

static int gc1054_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc1054_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc1054_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc1054_s_vflip(struct v4l2_subdev *sd, int value)
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
static int gc1054_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int gc1054_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(gc1054_again_lut); i++) {
		lut = &gc1054_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(gc1054_again_lut); i++) {
		lut = &gc1054_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int gc1054_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret = gc1054_read(sd, 0x23, &v);
	reg_val |= v ;

	*value = regval_to_again(reg_val);

	return ret;

}
/*set analog gain db value, map value to sensor register.*/
static int gc1054_s_again(struct v4l2_subdev *sd, int value)
{
	struct gc1054_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

//gc1054_mipi.c//
	ret = gc1054_write(sd, 0xfe, 0x01);
	ret += gc1054_write(sd, 0xb6, (unsigned char)reg_value);
	ret = gc1054_write(sd, 0xfe, 0x00);
//	printk("gc1054_write ok ,set analog gain from gc1054_again_lut array. %d\n" ,__LINE__ );
//gc1054_mipi.c//

	if (ret < 0){
		printk("gc1054_write error  %d\n" ,__LINE__ );
		return ret;
	}
	return 0;
}

/*
 * Tweak autogain.
 */
static int gc1054_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc1054_s_exp(struct v4l2_subdev *sd, int value)
{
	struct gc1054_info *info = to_state(sd);
	int ret = 0;

//	ret = gc1054_write(sd, 0xfd, 0x01);
//	ret += gc1054_write(sd, 0x4, (unsigned char)(value & 0xff));
//	ret += gc1054_write(sd, 0x3, (unsigned char)((value & 0xff00) >> 8));
//	ret += gc1054_write(sd, 0x01, 0x01);
//
//	if (ret < 0) {
//		printk("gc1054_write error  %d\n" ,__LINE__);
//		return ret;
//	}
	return ret;
}

static int gc1054_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc1054_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return gc1054_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc1054_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int gc1054_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc1054_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return gc1054_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return gc1054_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return gc1054_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return gc1054_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* gc1054_s_gain turns off auto gain */
			return gc1054_s_gain(sd, info->gain->val);
		}
		return gc1054_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return gc1054_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc1054_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return gc1054_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops gc1054_ctrl_ops = {
	.s_ctrl = gc1054_s_ctrl,
	.g_volatile_ctrl = gc1054_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc1054_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = gc1054_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int gc1054_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	gc1054_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int gc1054_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc1054_info *info = to_state(sd);
	int ret = 0;
	if (enable) {
		ret = gc1054_write_array(sd, info->win->regs);
		if(ret<0)
			return ret;
		ret = gc1054_write_array(sd, gc1054_stream_on);
		pr_debug("gc1054 stream on\n");

	}
	else {
		ret = gc1054_write_array(sd, gc1054_stream_off);
		pr_debug("gc1054 stream off\n");
	}
	return ret;
}

int gc1054_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct gc1054_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops gc1054_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc1054_g_register,
	.s_register = gc1054_s_register,
#endif

};

static const struct v4l2_subdev_video_ops gc1054_video_ops = {
	.s_stream = gc1054_s_stream,
	.g_frame_interval = gc1054_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc1054_pad_ops = {
	//.enum_frame_interval = gc1054_enum_frame_interval,
	//.num_frame_size = gc1054_enum_frame_size,
	//.enum_mbus_code = gc1054_enum_mbus_code,
	.set_fmt = gc1054_set_fmt,
	.get_fmt = gc1054_get_fmt,
};

static const struct v4l2_subdev_ops gc1054_ops = {
	.core = &gc1054_core_ops,
	.video = &gc1054_video_ops,
	.pad = &gc1054_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int gc1054_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct gc1054_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

printk(" gc1054 probe \n\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

//	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,cim-rst-gpio", 0, &flags);
//	if(gpio_is_valid(gpio)) {
//		info->reset.pin = gpio;
//		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
//		printk("~~~~~~~line=%d,reset gpio=%d,value=%d\n",__LINE__,info->reset.pin,info->reset.active_level);
//		ret = gpio_request_one(info->reset.pin, GPIOF_DIR_OUT, "rst");
//		if(ret < 0) {
//				dev_err(&client->dev, "Failed to request reset pin!\n");
//				return ret;
//		}
//	}



//init cim-en-gpio:3v3

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,cim-en-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->cimen.pin = gpio;
		info->cimen.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(info->cimen.pin, GPIOF_DIR_OUT, "cim-en");
		if(ret < 0) {
			dev_err(&client->dev, "Failed to request cimen pin!\n");
			return ret;
		}
		ret = gpio_direction_output(info->cimen.pin,info->cimen.active_level); //0
		if(ret < 0) {
			dev_err(&client->dev, "Failed to output cimen pin 0!\n");
			return ret;
		}

	}


	v4l2_i2c_subdev_init(sd, client, &gc1054_ops);
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

//	gc1054_reset(sd, 1);

#if 1
	/* Make sure it's an gc1054 */
		ret = gc1054_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an gc1054 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif
	/*IRCUT ctl 0:off 1:on*/
//	gc1054_ircut(sd, 0);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &gc1054_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

//	info->win = &gc1054_win_sizes[0];//720p
//	info->win = &gc1054_win_sizes[1];//1080*360
//	info->win = &gc1054_win_sizes[2];//720*360
	gc1054_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "gc1054 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int gc1054_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc1054_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id gc1054_id[] = {
	{ "gc1054", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc1054_id);

static const struct of_device_id gc1054_of_match[] = {
	{.compatible = "galaxyc,gc1054", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);


static struct i2c_driver gc1054_driver = {
	.driver = {
		.name	= "gc1054",
		.of_match_table = of_match_ptr(gc1054_of_match),
	},
	.probe		= gc1054_probe,
	.remove		= gc1054_remove,
	.id_table	= gc1054_id,
};

module_i2c_driver(gc1054_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision gc1054 sensors");
MODULE_LICENSE("GPL");
