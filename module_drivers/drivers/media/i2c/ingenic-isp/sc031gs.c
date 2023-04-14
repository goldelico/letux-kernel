/*
 * A V4L2 driver for OmniVision SC031GS cameras.
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

#define SC031GS_CHIP_ID_H       (0x00)
#define SC031GS_CHIP_ID_M       (0x31)
#define SC031GS_CHIP_ID_L       (0x01)
#define SC031GS_REG_END		0xffff
#define SC031GS_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct sc031gs_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct sc031gs_gpio {
	int pin;
	int active_level;
};

struct sc031gs_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct sc031gs_win_size *win;

	struct sc031gs_gpio reset;
	struct sc031gs_gpio vcc_en;
	struct sc031gs_gpio ircutp;
	struct sc031gs_gpio ircutn;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut sc031gs_again_lut[] = {

	{0x80, 0},
	{0x88, 5731},
	{0x90, 11136},
	{0x98, 16248},
	{0xa0, 21097},
	{0xa8, 25710},
	{0xb0, 30109},
	{0xb8, 34312},
	{0xc0, 38336},
	{0xc8, 42195},
	{0xd0, 45904},
	{0xd8, 49472},
	{0xe0, 52910},
	{0xe8, 56228},
	{0xf0, 59433},
	{0xf8, 62534},
	{0x178, 65536},
	{0x17c, 68445},
	{0x180, 71267},
	{0x184, 74008},
	{0x188, 76672},
	{0x18c, 79262},
	{0x190, 81784},
	{0x194, 84240},
	{0x198, 86633},
	{0x19c, 88968},
	{0x1a0, 91246},
	{0x1a4, 93471},
	{0x1a8, 95645},
	{0x1ac, 97770},
	{0x1b0, 99848},
	{0x1b4, 101881},
	{0x1b8, 103872},
	{0x1bc, 105821},
	{0x1c0, 107731},
	{0x1c4, 109604},
	{0x1c8, 111440},
	{0x1cc, 113241},
	{0x1d0, 115008},
	{0x1d4, 116743},
	{0x1d8, 118446},
	{0x1dc, 120120},
	{0x1e0, 121764},
	{0x1e4, 123380},
	{0x1e8, 124969},
	{0x1ec, 126532},
	{0x1f0, 128070},
	{0x1f4, 129583},
	{0x374, 131072},
	{0x376, 132537},
	{0x378, 133981},
	{0x37a, 135403},
	{0x37c, 136803},
	{0x37e, 138184},
	{0x380, 139544},
	{0x382, 140885},
	{0x384, 142208},
	{0x386, 143512},
	{0x388, 144798},
	{0x38a, 146067},
	{0x38c, 147320},
	{0x38e, 148556},
	{0x390, 149776},
	{0x392, 150980},
	{0x394, 152169},
	{0x396, 153344},
	{0x398, 154504},
	{0x39a, 155650},
	{0x39c, 156782},
	{0x39e, 157901},
	{0x3a0, 159007},
	{0x3a2, 160100},
	{0x3a4, 161181},
	{0x3a6, 162249},
	{0x3a8, 163306},
	{0x3aa, 164350},
	{0x3ac, 165384},
	{0x3ae, 166406},
	{0x3b0, 167417},
	{0x3b2, 168418},
	{0x3b4, 169408},
	{0x3b6, 170387},
	{0x3b8, 171357},
	{0x3ba, 172317},
	{0x3bc, 173267},
	{0x3be, 174208},
	{0x3c0, 175140},
	{0x3c2, 176062},
	{0x3c4, 176976},
	{0x3c6, 177880},
	{0x3c8, 178777},
	{0x3ca, 179664},
	{0x3cc, 180544},
	{0x3ce, 181415},
	{0x3d0, 182279},
	{0x3d2, 183134},
	{0x3d4, 183982},
	{0x3d6, 184823},
	{0x3d8, 185656},
	{0x3da, 186482},
	{0x3dc, 187300},
	{0x3de, 188112},
	{0x3e0, 188916},
	{0x3e2, 189714},
	{0x3e4, 190505},
	{0x3e6, 191290},
	{0x3e8, 192068},
	{0x3ea, 192840},
	{0x3ec, 193606},
	{0x3ee, 194365},
	{0x3f0, 195119},
	{0x3f2, 195866},
	{0x778, 196608},
	{0x779, 197343},
	{0x77a, 198073},
	{0x77b, 198798},
	{0x77c, 199517},
	{0x77d, 200230},
	{0x77e, 200939},
	{0x77f, 201642},
	{0x780, 202339},
	{0x781, 203032},
	{0x782, 203720},
	{0x783, 204402},
	{0x784, 205080},
	{0x785, 205753},
	{0x786, 206421},
	{0x787, 207085},
	{0x788, 207744},
	{0x789, 208398},
	{0x78a, 209048},
	{0x78b, 209693},
	{0x78c, 210334},
	{0x78d, 210971},
	{0x78e, 211603},
	{0x78f, 212232},
	{0x790, 212856},
	{0x791, 213476},
	{0x792, 214092},
	{0x793, 214704},
	{0x794, 215312},
	{0x795, 215916},
	{0x796, 216516},
	{0x797, 217113},
	{0x798, 217705},
	{0x799, 218294},
	{0x79a, 218880},
	{0x79b, 219462},
	{0x79c, 220040},
	{0x79d, 220615},
	{0x79e, 221186},
	{0x79f, 221754},
	{0x7a0, 222318},
	{0x7a1, 222880},
	{0x7a2, 223437},
	{0x7a3, 223992},
	{0x7a4, 224543},
	{0x7a5, 225091},
	{0x7a6, 225636},
	{0x7a7, 226178},
	{0x7a8, 226717},
	{0x7a9, 227253},
	{0x7aa, 227785},
	{0x7ab, 228315},
	{0x7ac, 228842},
	{0x7ad, 229365},
	{0x7ae, 229886},
	{0x7af, 230404},
	{0x7b0, 230920},
	{0x7b1, 231432},
	{0x7b2, 231942},
	{0x7b3, 232449},
	{0x7b4, 232953},
	{0x7b5, 233455},
	{0x7b6, 233954},
	{0x7b7, 234450},
	{0x7b8, 234944},
	{0x7b9, 235435},
	{0x7ba, 235923},
	{0x7bb, 236410},
	{0x7bc, 236893},
	{0x7bd, 237374},
	{0x7be, 237853},
	{0x7bf, 238329},
	{0x7c0, 238803},
	{0x7c1, 239275},
	{0x7c2, 239744},
	{0x7c3, 240211},
	{0x7c4, 240676},
	{0x7c5, 241138},
	{0x7c6, 241598},
	{0x7c7, 242056},
	{0x7c8, 242512},
	{0x7c9, 242965},
	{0x7ca, 243416},
	{0x7cb, 243865},
	{0x7cc, 244313},
	{0x7cd, 244757},
	{0x7ce, 245200},
	{0x7cf, 245641},
	{0x7d0, 246080},
	{0x7d1, 246517},
	{0x7d2, 246951},
	{0x7d3, 247384},
	{0x7d4, 247815},
	{0x7d5, 248243},
	{0x7d6, 248670},
	{0x7d7, 249095},
	{0x7d8, 249518},
	{0x7d9, 249939},
	{0x7da, 250359},
	{0x7db, 250776},
	{0x7dc, 251192},
	{0x7dd, 251606},
	{0x7de, 252018},
	{0x7df, 252428},
	{0x7e0, 252836},
	{0x7e1, 253243},
	{0x7e2, 253648},
	{0x7e3, 254051},
	{0x7e4, 254452},
	{0x7e5, 254852},
	{0x7e6, 255250},
	{0x7e7, 255647},
	{0x7e8, 256041},
	{0x7e9, 256435},
	{0x7ea, 256826},
	{0x7eb, 257216},
	{0x7ec, 257604},
	{0x7ed, 257991},
	{0x7ee, 258376},
	{0x7ef, 258760},
	{0x7f0, 259142},
	{0x7f1, 259522},
	{0x7f2, 259901},
	{0x7f3, 260279},
	{0x7f4, 260655},
	{0x7f5, 261029},
	{0x7f6, 261402},
	{0x7f7, 261773},
};

static inline struct sc031gs_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc031gs_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sc031gs_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list sc031gs_init_regs_640_480_60fps_mipi[] = {
	//[ParaList]
	{0x0103,0x01},
	{0x0100,0x00},


	//close mipi
	//{0x3018,0x1f}, 
	//{0x3019,0xff},
	//{0x301c,0xb4},



	//for mipi
	{0x4603,0x00},//[0] fifo dvp read en
	{0x3000,0x00},//
	{0x3001,0x00},//pad dvp en
	{0x303f,0x01},//[7] pll pclk dis

	{0x3018,0x13},//[7:5] lane num = [7:5] + 1 , [3:2] pd_mipi

	{0x3031,0x0a},//[3:0] mipi bit num
	{0x3037,0x20},//[6:5] phy bit num 00~8bit 01~10bit 10~12bit
	{0x301c,0x78},//[7] pclk_dvp_en [6] pclk_mipi_en [3] rst_dvp [2] rst_mipi
	{0x3019,0xfe},//[3:0] lane_disable


	{0x363c,0x08},
	{0x3630,0x82},


	{0x3638,0x0f}, //ramp offset
	{0x3639,0x08}, //ramp_vref
	{0x335b,0x80}, //pix_samp all high
	//{0x3323,0x80}, //[7] srs all1  col fpn /with vln all 1 pix fpn

	{0x3636,0x25},
	{0x3640,0x02},
	{0x3306,0x38},

	{0x3304,0x48},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x68},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},


	//0101
	{0x3366,0x04}, //sb over count up

	{0x335f,0x80}, //avdd samp all high
	{0x363a,0x00}, // psr

	{0x3622,0x01},
	{0x3633,0x62},

	//bsi
	{0x36f9,0x20},

	//{0x3250,0xf0},
	{0x3637,0x80},
	{0x363d,0x04},
	{0x3e06,0x00},
	{0x363c,0x48},


	{0x320c,0x03},
	{0x320e,0x0e},
	{0x320f,0xa8},
	{0x3306,0x38},
	{0x330b,0xb6},


	//{0x3621,0x90},
	{0x36f9,0x24},


	//0104
	{0x363b,0x4a},
	{0x3366,0x02},

	//precharge
	{0x3316,0x78},
	{0x3344,0x74},
	{0x3335,0x74},
	{0x332f,0x70},
	{0x332d,0x6c},
	{0x3329,0x6c},

	//0105
	//{0x3250,0xff},

	//1.24
	{0x363c,0x08},
	{0x3630,0x81},

	//{0x3314,0x28}, //for offset cancel
	//{0x3317,0x23},
	//{0x3366,0x04},

	{0x3366,0x06},

	{0x3314,0x3a},
	{0x3317,0x28},

	//0130
	{0x3622,0x05}, //blksun
	{0x363d,0x00},
	{0x3637,0x86},

	{0x3e01,0x62},
	{0x3633,0x52},
	{0x3630,0x86},
	{0x3306,0x4c},
	{0x330b,0xa0},

	//0131
	{0x3631,0x48},

	//0201
	{0x33b1,0x03}, // chis position
	{0x33b2,0x06}, //[3:2]=chis_length [1:0] +1=trans_length
	{0x320c,0x02},
	{0x320e,0x02},
	{0x320f,0x0d},
	{0x3e01,0x20},
	{0x3e02,0x20},

	//precharge
	{0x3316,0x48},
	{0x3344,0x44},
	{0x3335,0x44},
	{0x332f,0x40},
	{0x332d,0x3c},
	{0x3329,0x3c},

	{0x3310,0x10}, //tx width

	{0x3637,0x87},
	{0x363e,0xf8},


	//power save mode  0201B

	{0x3254,0x02},
	{0x3255,0x07},  //{3204,3205}={3260,3261}+1

	{0x3252,0x02},
	{0x3253,0xa6},  // {3252,3253}={320e,320f}-5

	{0x3250,0xc0},  //[5:4]sa1 [3:2] opbuf  [1:0] ramp 0724
	{0x3251,0x02},  //[1] opbuf  [0] ramp

	//0322
	{0x330f,0x50},
	{0x3630,0x46},

	//0324
	{0x3621,0xa2}, //aa?

	//0329
	{0x3621,0xa0},
	{0x4500,0x59}, //fifo delay

	//0404
	{0x3637,0x88}, //blksun margin
	{0x3908,0x81}, //for digital gain

	{0x3640,0x00},
	{0x3641,0x02},

	{0x363c,0x05}, //only for vt++ com1 dvp sample
	{0x363b,0x4c},

	//72MPCLK 183fps

	{0x36e9,0x40},
	{0x36ea,0x36},
	{0x36ed,0x13},

	{0x36f9,0x04},
	{0x36fa,0x38},
	{0x330b,0x80},


	//{0x3304,0x40},
	//{0x3389,0x01}, //offset en
	//{0x3385,0x31}, //integ en
	//{0x330c,0x10}, //0109
	//{0x3315,0x30},
	//{0x3309,0x60},
	//{0x3387,0x51}, //integ_en
	//{0x3306,0x50},

	{0x3640,0x00},
	{0x3641,0x01},

	//0502
	{0x3d08,0x00},
	{0x3306,0x48},
	{0x3621,0xa4},
	{0x300f,0x0f},


	//0612
	{0x4837, 0x1b},  //0x4837=2000/PCLK
	{0x4809, 0x01}, //clock lane lp initialized to 11

	//0622  24M input 72MPCLK 120fps
	{0x363b,0x48},
	{0x363c,0x06},
	{0x36e9,0x00},
	{0x36ea,0x3b},
	{0x36eb,0x0e},
	{0x36ec,0x0e},
	{0x36ed,0x33},
	{0x36f9,0x00},
	{0x36fa,0x3a},
	{0x36fc,0x01},
#if 0
	{0x320c,0x03}, //for high temp more margin
	{0x320d,0x6e},
	{0x320e,0x02},
	{0x320f,0xab}, //120fps
#else
	{0x320c,0x04}, //for high temp more margin
	{0x320d,0x8e},
	{0x320e,0x03},
	{0x320f,0xeb}, //60fps
#endif


#if 0
	{0x3208,0x00},
	{0x3209,0x80},
	{0x320a,0x00},
	{0x320b,0x60}, //128*96
#endif

	{0x330b,0x80},
	{0x330f,0x50},

	{0x3637,0x89}, //blksun margin

	{0x3641,0x01}, //0x01 for 2.8V dovdd 0x02 for 1.8V DOVDD

	//0625 for 70C 4ms 16X gain
	{0x4501,0xc4},
	{0x5011,0x01},
	{0x3908,0x21},
	{0x3e01,0x01},
	{0x3e02,0xa0},
	{0x3306,0x38},
	{0x330b,0xe0}, //high temp [a0,f0]
	{0x330f,0x20},

	//0626
	{0x3d08,0x01},

	//0724
	{0x5011,0x00},
	{0x3e06,0x0c},
	{0x3908,0x91},

	//{0x3314,0x3a}, 
	//{0x3317,0x20},
	{0x3314,0x1e},
	{0x3317,0x10}, //gain

	//st light
	{0x3635,0x18},

	//  reduce minimum precharge
	{0x3316,0x40},
	{0x3344,0x3c},
	{0x3335,0x3c},
	{0x332f,0x38},
	{0x332d,0x34},
	{0x3329,0x34},
	//20180829
	{0x3624,0x47},

	//20181101
	{0x3220,0x10},//[1:0] vflip blc����

	//20181106
	{0x3635,0x18},

	{0x0100,0x01}, //72M sysclk 576M cntclk



	{0x4418,0x08},
	{0x4419,0x8e}, //20190222 cover avdd 2.6V-3.0V

//	{0x0100,0x01}, //72M sysclk 576M cntclk


	/*
	//[gain<2]
	{0x3314,0x1e},//0222
	{0x3317,0x10},
	//[4>gain>=2]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625
	//[gain>=4]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625

	//[half count_clk test]
	{0x36f9,0x20},
	{0x3304,0x40},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x60},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},

	//[noise_test]
	{0x3324,0x10},
	{0x3907,0x04},
	{0x3e06,0x03},

	//[dark current test]
	{0x3e01,0xea},
	{0x3e02,0x10},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0c},
	{0x3e08,0x1f},
	{0x3622,0x01},

	//[pos offset]
	{0x3314,0x34},
	{0x3317,0x29},

	//[precharge test]
	{0x3e09,0x1f},
	{0x3e08,0x1f},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0d},
	{0x390a,0x80},


	//[pls test] //AF=2.6
	{0x4419,0x8e},
	{0x3630,0x66},
	{0x3902,0x05},
	{0x3909,0x0f},
	{0x390a,0x80},
	{0x3314,0x4a},
	{0x3317,0x0e},
	{0x320e,0x10},
	//{0x3907,0x04},
	{0x3e01,0x00},
	{0x3e02,0x20},
	{0x320c,0x12},
	{0x320d,0xee},
	*/
//	{0x4501,0xac},
//	{0x3902,0x05},
//	{0x3e06,0x03},
	{SC031GS_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc031gs_init_regs_320_240_60fps_mipi[] = {
	//[ParaList]
	{0x0103,0x01},
	{0x0100,0x00},


	//close mipi
	//{0x3018,0x1f}, 
	//{0x3019,0xff},
	//{0x301c,0xb4},



	//for mipi
	{0x4603,0x00},//[0] fifo dvp read en
	{0x3000,0x00},//
	{0x3001,0x00},//pad dvp en
	{0x303f,0x01},//[7] pll pclk dis

	{0x3018,0x13},//[7:5] lane num = [7:5] + 1 , [3:2] pd_mipi

	{0x3031,0x0a},//[3:0] mipi bit num
	{0x3037,0x20},//[6:5] phy bit num 00~8bit 01~10bit 10~12bit
	{0x301c,0x78},//[7] pclk_dvp_en [6] pclk_mipi_en [3] rst_dvp [2] rst_mipi
	{0x3019,0xfe},//[3:0] lane_disable


	{0x363c,0x08},
	{0x3630,0x82},


	{0x3638,0x0f}, //ramp offset
	{0x3639,0x08}, //ramp_vref
	{0x335b,0x80}, //pix_samp all high
	//{0x3323,0x80}, //[7] srs all1  col fpn /with vln all 1 pix fpn

	{0x3636,0x25},
	{0x3640,0x02},
	{0x3306,0x38},

	{0x3304,0x48},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x68},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},


	//0101
	{0x3366,0x04}, //sb over count up

	{0x335f,0x80}, //avdd samp all high
	{0x363a,0x00}, // psr

	{0x3622,0x01},
	{0x3633,0x62},

	//bsi
	{0x36f9,0x20},

	//{0x3250,0xf0},
	{0x3637,0x80},
	{0x363d,0x04},
	{0x3e06,0x00},
	{0x363c,0x48},


	{0x320c,0x03},
	{0x320e,0x0e},
	{0x320f,0xa8},
	{0x3306,0x38},
	{0x330b,0xb6},


	//{0x3621,0x90},
	{0x36f9,0x24},


	//0104
	{0x363b,0x4a},
	{0x3366,0x02},

	//precharge
	{0x3316,0x78},
	{0x3344,0x74},
	{0x3335,0x74},
	{0x332f,0x70},
	{0x332d,0x6c},
	{0x3329,0x6c},

	//0105
	//{0x3250,0xff},

	//1.24
	{0x363c,0x08},
	{0x3630,0x81},

	//{0x3314,0x28}, //for offset cancel
	//{0x3317,0x23},
	//{0x3366,0x04},

	{0x3366,0x06},

	{0x3314,0x3a},
	{0x3317,0x28},

	//0130
	{0x3622,0x05}, //blksun
	{0x363d,0x00},
	{0x3637,0x86},

	{0x3e01,0x62},
	{0x3633,0x52},
	{0x3630,0x86},
	{0x3306,0x4c},
	{0x330b,0xa0},

	//0131
	{0x3631,0x48},

	//0201
	{0x33b1,0x03}, // chis position
	{0x33b2,0x06}, //[3:2]=chis_length [1:0] +1=trans_length
	{0x320c,0x02},
	{0x320e,0x02},
	{0x320f,0x0d},
	{0x3e01,0x20},
	{0x3e02,0x20},

	//precharge
	{0x3316,0x48},
	{0x3344,0x44},
	{0x3335,0x44},
	{0x332f,0x40},
	{0x332d,0x3c},
	{0x3329,0x3c},

	{0x3310,0x10}, //tx width

	{0x3637,0x87},
	{0x363e,0xf8},


	//power save mode  0201B

	{0x3254,0x02},
	{0x3255,0x07},  //{3204,3205}={3260,3261}+1

	{0x3252,0x02},
	{0x3253,0xa6},  // {3252,3253}={320e,320f}-5

	{0x3250,0xc0},  //[5:4]sa1 [3:2] opbuf  [1:0] ramp 0724
	{0x3251,0x02},  //[1] opbuf  [0] ramp

	//0322
	{0x330f,0x50},
	{0x3630,0x46},

	//0324
	{0x3621,0xa2}, //aa?

	//0329
	{0x3621,0xa0},
	{0x4500,0x59}, //fifo delay

	//0404
	{0x3637,0x88}, //blksun margin
	{0x3908,0x81}, //for digital gain

	{0x3640,0x00},
	{0x3641,0x02},

	{0x363c,0x05}, //only for vt++ com1 dvp sample
	{0x363b,0x4c},

	//72MPCLK 183fps

	{0x36e9,0x40},
	{0x36ea,0x36},
	{0x36ed,0x13},

	{0x36f9,0x04},
	{0x36fa,0x38},
	{0x330b,0x80},


	//{0x3304,0x40},
	//{0x3389,0x01}, //offset en
	//{0x3385,0x31}, //integ en
	//{0x330c,0x10}, //0109
	//{0x3315,0x30},
	//{0x3309,0x60},
	//{0x3387,0x51}, //integ_en
	//{0x3306,0x50},

	{0x3640,0x00},
	{0x3641,0x01},

	//0502
	{0x3d08,0x00},
	{0x3306,0x48},
	{0x3621,0xa4},
	{0x300f,0x0f},


	//0612
	{0x4837, 0x1b},  //0x4837=2000/PCLK
	{0x4809, 0x01}, //clock lane lp initialized to 11

	//0622  24M input 72MPCLK 120fps
	{0x363b,0x48},
	{0x363c,0x06},
	{0x36e9,0x00},
	{0x36ea,0x3b},
	{0x36eb,0x0e},
	{0x36ec,0x0e},
	{0x36ed,0x33},
	{0x36f9,0x00},
	{0x36fa,0x3a},
	{0x36fc,0x01},
#if 0
	{0x320c,0x03}, //for high temp more margin
	{0x320d,0x6e},
	{0x320e,0x02},
	{0x320f,0xab}, //120fps
#else
	{0x320c,0x04}, //for high temp more margin
	{0x320d,0x8e},
	{0x320e,0x03},
	{0x320f,0xeb}, //60fps
#endif


#if 1
	{0x3208,0x01},
	{0x3209,0x40},
	{0x320a,0x00},
	{0x320b,0xf0}, //320*240

	{0x3210, 0x00},
	{0x3211, 0xA0},
	{0x3212, 0x00},
	{0x3213, 0x78},
#endif

	{0x330b,0x80},
	{0x330f,0x50},

	{0x3637,0x89}, //blksun margin

	{0x3641,0x01}, //0x01 for 2.8V dovdd 0x02 for 1.8V DOVDD

	//0625 for 70C 4ms 16X gain
	{0x4501,0xc4},
	{0x5011,0x01},
	{0x3908,0x21},
	{0x3e01,0x0a},
	{0x3e02,0x80}, //expo time
	{0x3306,0x38},
	{0x330b,0xe0}, //high temp [a0,f0]
	{0x330f,0x20},

	//0626
	{0x3d08,0x01},

	//0724
	{0x5011,0x00},
	{0x3e06,0x0c},
	{0x3908,0x91},

	//{0x3314,0x3a}, 
	//{0x3317,0x20},
	{0x3314,0x1e},
	{0x3317,0x10}, //gain

	//st light
	{0x3635,0x18},

	//  reduce minimum precharge
	{0x3316,0x40},
	{0x3344,0x3c},
	{0x3335,0x3c},
	{0x332f,0x38},
	{0x332d,0x34},
	{0x3329,0x34},
	//20180829
	{0x3624,0x47},

	//20181101
	{0x3220,0x10},//[1:0] vflip blc����

	//20181106
	{0x3635,0x18},

	{0x0100,0x01}, //72M sysclk 576M cntclk



	{0x4418,0x08},
	{0x4419,0x8e}, //20190222 cover avdd 2.6V-3.0V

//	{0x0100,0x01}, //72M sysclk 576M cntclk


	/*
	//[gain<2]
	{0x3314,0x1e},//0222
	{0x3317,0x10},
	//[4>gain>=2]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625
	//[gain>=4]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625

	//[half count_clk test]
	{0x36f9,0x20},
	{0x3304,0x40},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x60},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},

	//[noise_test]
	{0x3324,0x10},
	{0x3907,0x04},
	{0x3e06,0x03},

	//[dark current test]
	{0x3e01,0xea},
	{0x3e02,0x10},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0c},
	{0x3e08,0x1f},
	{0x3622,0x01},

	//[pos offset]
	{0x3314,0x34},
	{0x3317,0x29},

	//[precharge test]
	{0x3e09,0x1f},
	{0x3e08,0x1f},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0d},
	{0x390a,0x80},


	//[pls test] //AF=2.6
	{0x4419,0x8e},
	{0x3630,0x66},
	{0x3902,0x05},
	{0x3909,0x0f},
	{0x390a,0x80},
	{0x3314,0x4a},
	{0x3317,0x0e},
	{0x320e,0x10},
	//{0x3907,0x04},
	{0x3e01,0x00},
	{0x3e02,0x20},
	{0x320c,0x12},
	{0x320d,0xee},
	*/
//	{0x4501,0xac},
//	{0x3902,0x05},
//	{0x3e06,0x03},
	{SC031GS_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc031gs_init_regs_128_96_60fps_mipi[] = {
	//[ParaList]
	{0x0103,0x01},
	{0x0100,0x00},


	//close mipi
	//{0x3018,0x1f},
	//{0x3019,0xff},
	//{0x301c,0xb4},



	//for mipi
	{0x4603,0x00},//[0] fifo dvp read en
	{0x3000,0x00},//
	{0x3001,0x00},//pad dvp en
	{0x303f,0x01},//[7] pll pclk dis

	{0x3018,0x13},//[7:5] lane num = [7:5] + 1 , [3:2] pd_mipi

	{0x3031,0x0a},//[3:0] mipi bit num
	{0x3037,0x20},//[6:5] phy bit num 00~8bit 01~10bit 10~12bit
	{0x301c,0x78},//[7] pclk_dvp_en [6] pclk_mipi_en [3] rst_dvp [2] rst_mipi
	{0x3019,0xfe},//[3:0] lane_disable


	{0x363c,0x08},
	{0x3630,0x82},


	{0x3638,0x0f}, //ramp offset
	{0x3639,0x08}, //ramp_vref
	{0x335b,0x80}, //pix_samp all high
	//{0x3323,0x80}, //[7] srs all1  col fpn /with vln all 1 pix fpn

	{0x3636,0x25},
	{0x3640,0x02},
	{0x3306,0x38},

	{0x3304,0x48},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x68},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},


	//0101
	{0x3366,0x04}, //sb over count up

	{0x335f,0x80}, //avdd samp all high
	{0x363a,0x00}, // psr

	{0x3622,0x01},
	{0x3633,0x62},

	//bsi
	{0x36f9,0x20},

	//{0x3250,0xf0},
	{0x3637,0x80},
	{0x363d,0x04},
	{0x3e06,0x00},
	{0x363c,0x48},


	{0x320c,0x03},
	{0x320e,0x0e},
	{0x320f,0xa8},
	{0x3306,0x38},
	{0x330b,0xb6},


	//{0x3621,0x90},
	{0x36f9,0x24},


	//0104
	{0x363b,0x4a},
	{0x3366,0x02},

	//precharge
	{0x3316,0x78},
	{0x3344,0x74},
	{0x3335,0x74},
	{0x332f,0x70},
	{0x332d,0x6c},
	{0x3329,0x6c},

	//0105
	//{0x3250,0xff},

	//1.24
	{0x363c,0x08},
	{0x3630,0x81},

	//{0x3314,0x28}, //for offset cancel
	//{0x3317,0x23},
	//{0x3366,0x04},

	{0x3366,0x06},

	{0x3314,0x3a},
	{0x3317,0x28},

	//0130
	{0x3622,0x05}, //blksun
	{0x363d,0x00},
	{0x3637,0x86},

	{0x3e01,0x62},
	{0x3633,0x52},
	{0x3630,0x86},
	{0x3306,0x4c},
	{0x330b,0xa0},

	//0131
	{0x3631,0x48},

	//0201
	{0x33b1,0x03}, // chis position
	{0x33b2,0x06}, //[3:2]=chis_length [1:0] +1=trans_length
	{0x320c,0x02},
	{0x320e,0x02},
	{0x320f,0x0d},
	{0x3e01,0x20},
	{0x3e02,0x20},

	//precharge
	{0x3316,0x48},
	{0x3344,0x44},
	{0x3335,0x44},
	{0x332f,0x40},
	{0x332d,0x3c},
	{0x3329,0x3c},

	{0x3310,0x10}, //tx width

	{0x3637,0x87},
	{0x363e,0xf8},


	//power save mode  0201B

	{0x3254,0x02},
	{0x3255,0x07},  //{3204,3205}={3260,3261}+1

	{0x3252,0x02},
	{0x3253,0xa6},  // {3252,3253}={320e,320f}-5

	{0x3250,0xc0},  //[5:4]sa1 [3:2] opbuf  [1:0] ramp 0724
	{0x3251,0x02},  //[1] opbuf  [0] ramp

	//0322
	{0x330f,0x50},
	{0x3630,0x46},

	//0324
	{0x3621,0xa2}, //aa?

	//0329
	{0x3621,0xa0},
	{0x4500,0x59}, //fifo delay

	//0404
	{0x3637,0x88}, //blksun margin
	{0x3908,0x81}, //for digital gain

	{0x3640,0x00},
	{0x3641,0x02},

	{0x363c,0x05}, //only for vt++ com1 dvp sample
	{0x363b,0x4c},

	//72MPCLK 183fps

	{0x36e9,0x40},
	{0x36ea,0x36},
	{0x36ed,0x13},

	{0x36f9,0x04},
	{0x36fa,0x38},
	{0x330b,0x80},


	//{0x3304,0x40},
	//{0x3389,0x01}, //offset en
	//{0x3385,0x31}, //integ en
	//{0x330c,0x10}, //0109
	//{0x3315,0x30},
	//{0x3309,0x60},
	//{0x3387,0x51}, //integ_en
	//{0x3306,0x50},

	{0x3640,0x00},
	{0x3641,0x01},

	//0502
	{0x3d08,0x00},
	{0x3306,0x48},
	{0x3621,0xa4},
	{0x300f,0x0f},


	//0612
	{0x4837, 0x1b},  //0x4837=2000/PCLK
	{0x4809, 0x01}, //clock lane lp initialized to 11

	//0622  24M input 72MPCLK 120fps
	{0x363b,0x48},
	{0x363c,0x06},
	{0x36e9,0x00},
	{0x36ea,0x3b},
	{0x36eb,0x0e},
	{0x36ec,0x0e},
	{0x36ed,0x33},
	{0x36f9,0x00},
	{0x36fa,0x3a},
	{0x36fc,0x01},
#if 0
	{0x320c,0x03}, //for high temp more margin
	{0x320d,0x6e},
	{0x320e,0x02},
	{0x320f,0xab}, //120fps
#else
	{0x320c,0x04}, //for high temp more margin
	{0x320d,0x8e},
	{0x320e,0x03},
	{0x320f,0xeb}, //60fps
#endif


#if 1
	{0x3208,0x00},
	{0x3209,0x80},
	{0x320a,0x00},
	{0x320b,0x60}, //128*96
#endif

	{0x330b,0x80},
	{0x330f,0x50},

	{0x3637,0x89}, //blksun margin

	{0x3641,0x01}, //0x01 for 2.8V dovdd 0x02 for 1.8V DOVDD

	//0625 for 70C 4ms 16X gain
	{0x4501,0xc4},
	{0x5011,0x01},
	{0x3908,0x21},
	{0x3e01,0x0a},
	{0x3e02,0x80},
	{0x3306,0x38},
	{0x330b,0xe0}, //high temp [a0,f0]
	{0x330f,0x20},

	//0626
	{0x3d08,0x01},

	//0724
	{0x5011,0x00},
	{0x3e06,0x0c},
	{0x3908,0x91},

	//{0x3314,0x3a}, 
	//{0x3317,0x20},
	{0x3314,0x1e},
	{0x3317,0x10}, //gain

	//st light
	{0x3635,0x18},

	//  reduce minimum precharge
	{0x3316,0x40},
	{0x3344,0x3c},
	{0x3335,0x3c},
	{0x332f,0x38},
	{0x332d,0x34},
	{0x3329,0x34},
	//20180829
	{0x3624,0x47},

	//20181101
	{0x3220,0x10},//[1:0] vflip blc����

	//20181106
	{0x3635,0x18},

	{0x0100,0x01}, //72M sysclk 576M cntclk



	{0x4418,0x08},
	{0x4419,0x8e}, //20190222 cover avdd 2.6V-3.0V

//	{0x0100,0x01}, //72M sysclk 576M cntclk


	/*
	//[gain<2]
	{0x3314,0x1e},//0222
	{0x3317,0x10},
	//[4>gain>=2]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625
	//[gain>=4]
	{0x3314,0x4f}, //0222
	{0x3317,0x0f}, //0625

	//[half count_clk test]
	{0x36f9,0x20},
	{0x3304,0x40},
	{0x3389,0x01}, //offset en
	{0x3385,0x31}, //integ en
	{0x330c,0x18}, //0109
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x60},
	{0x3387,0x51}, //integ_en
	{0x3306,0x48},

	//[noise_test]
	{0x3324,0x10},
	{0x3907,0x04},
	{0x3e06,0x03},

	//[dark current test]
	{0x3e01,0xea},
	{0x3e02,0x10},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0c},
	{0x3e08,0x1f},
	{0x3622,0x01},

	//[pos offset]
	{0x3314,0x34},
	{0x3317,0x29},

	//[precharge test]
	{0x3e09,0x1f},
	{0x3e08,0x1f},
	{0x3e06,0x03},
	{0x3902,0x05},
	{0x3909,0x0d},
	{0x390a,0x80},


	//[pls test] //AF=2.6
	{0x4419,0x8e},
	{0x3630,0x66},
	{0x3902,0x05},
	{0x3909,0x0f},
	{0x390a,0x80},
	{0x3314,0x4a},
	{0x3317,0x0e},
	{0x320e,0x10},
	//{0x3907,0x04},
	{0x3e01,0x00},
	{0x3e02,0x20},
	{0x320c,0x12},
	{0x320d,0xee},
	*/
//	{0x4501,0xac},
//	{0x3902,0x05},
//	{0x3e06,0x03},
	{SC031GS_REG_END, 0x00},	/* END MARKER */
};


/*
 * the part of driver was fixed.
 */

static struct regval_list sc031gs_stream_on_mipi[] = {
	{0x0100, 0x01},
	{SC031GS_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc031gs_stream_off_mipi[] = {
	//close mipi
//	{0x3018,0x1f}, 
//	{0x3019,0xff},
//	{0x301c,0xb4},
	{SC031GS_REG_END, 0x00},	/* END MARKER */
};

static int sc031gs_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct sc031gs_info *info = to_state(sd);
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

static int sc031gs_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct sc031gs_info *info = to_state(sd);
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

static int sc031gs_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != SC031GS_REG_END) {
		if (vals->reg_num == SC031GS_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc031gs_read(sd, vals->reg_num, &val);
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
static int sc031gs_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != SC031GS_REG_END) {
		if (vals->reg_num == SC031GS_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc031gs_write(sd, vals->reg_num, vals->value);
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
static int sc031gs_power(struct v4l2_subdev *sd, u32 val)
{
	struct sc031gs_info *info = to_state(sd);

	if(val){
		*(volatile unsigned int *)0xb0010230 &= ~(1 << 28);
		*(volatile unsigned int *)0xb0010240 |= (1 << 28);
//		gpio_direction_output(info->reset.pin, info->reset.active_level);
	}else{
		*(volatile unsigned int *)0xb0010230 |= (1 << 28);
		*(volatile unsigned int *)0xb0010240 &= ~(1 << 28);
//		gpio_direction_output(info->reset.pin, !info->reset.active_level);
	}
	return 0;
}

static int sc031gs_reset(struct v4l2_subdev *sd, u32 val)
{
	struct sc031gs_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int sc031gs_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct sc031gs_info *info = to_state(sd);

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

static int sc031gs_init(struct v4l2_subdev *sd, u32 val)
{
	struct sc031gs_info *info = to_state(sd);
	int ret = 0;

	ret = sc031gs_write_array(sd, info->win->regs);

	return ret;
}



static int sc031gs_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v = 0;
	int ret;

	ret = sc031gs_read(sd, 0x3107, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC031GS_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = sc031gs_read(sd, 0x3108, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC031GS_CHIP_ID_M)
		return -ENODEV;
	*ident = v;

	ret = sc031gs_read(sd, 0x3109, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC031GS_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}


static struct sc031gs_win_size sc031gs_win_sizes[] = {
	{
		.sensor_info.mipi_cfg.twidth		= 640,
		.sensor_info.mipi_cfg.theight		= 480,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 60 << 16 | 1,

		.width		= 640,
		.height		= 480,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc031gs_init_regs_640_480_60fps_mipi,
	},
	{
		.sensor_info.mipi_cfg.twidth		= 320,
		.sensor_info.mipi_cfg.theight		= 240,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 60 << 16 | 1,

		.width		= 320,
		.height		= 240,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc031gs_init_regs_320_240_60fps_mipi,
	},
	{
		.sensor_info.mipi_cfg.twidth		= 128,
		.sensor_info.mipi_cfg.theight		= 96,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 60 << 16 | 1,

		.width		= 128,
		.height		= 96,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc031gs_init_regs_128_96_60fps_mipi,
	},
};

static int sc031gs_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_SC031GS_FMTS)
		return -EINVAL;

	code->code = sc031gs_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int sc031gs_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct sc031gs_format_struct *ovfmt;
	struct sc031gs_win_size *wsize;
	struct sc031gs_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int sc031gs_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct sc031gs_info *info = to_state(sd);
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

static int sc031gs_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc031gs_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc031gs_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc031gs_s_vflip(struct v4l2_subdev *sd, int value)
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
static int sc031gs_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int sc031gs_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc031gs_again_lut); i++) {
		lut = &sc031gs_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(sc031gs_again_lut); i++) {
		lut = &sc031gs_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
}

static int sc031gs_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	/*
	   struct sc031gs_info *info = to_state(sd);
	   char v = 0;
	   unsigned int reg_val = 0;


	   ret += sc031gs_read(sd, 0x3509, &v);

	   reg_val |= v;
	   ret += sc031gs_read(sd, 0x3508, &v);
	   reg_val |= v << 8;


	 *value = regval_to_again(reg_val);
	 */

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int sc031gs_s_again(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
#if 0
	struct sc031gs_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int i;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += sc031gs_write(sd, 0x3509, (unsigned char)(reg_value & 0xff));
	ret += sc031gs_write(sd, 0x3508, (unsigned char)((reg_value>>8) & 0xff));
	if (ret < 0)
		return ret;

#endif
	return ret;
}

/*
 * Tweak autogain.
 */
static int sc031gs_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc031gs_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
	/*
	   struct sc031gs_info *info = to_state(sd);

	   ret += sc031gs_write(sd, 0x3502, ((unsigned char)(value & 0xf)) << 4);
	   ret += sc031gs_write(sd, 0x3501, (unsigned char)((value >> 4) & 0xff));
	   ret += sc031gs_write(sd, 0x3500, (unsigned char)((value >> 12) & 0xf));

*/
	return ret;
}

static int sc031gs_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc031gs_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_AUTOGAIN:
			return sc031gs_g_gain(sd, &info->gain->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return sc031gs_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int sc031gs_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc031gs_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return sc031gs_s_brightness(sd, ctrl->val);
		case V4L2_CID_CONTRAST:
			return sc031gs_s_contrast(sd, ctrl->val);
		case V4L2_CID_VFLIP:
			return sc031gs_s_vflip(sd, ctrl->val);
		case V4L2_CID_HFLIP:
			return sc031gs_s_hflip(sd, ctrl->val);
		case V4L2_CID_AUTOGAIN:
			/* Only set manual gain if auto gain is not explicitly
			   turned on. */
			if (!ctrl->val) {
				/* sc031gs_s_gain turns off auto gain */
				return sc031gs_s_gain(sd, info->gain->val);
			}
			return sc031gs_s_autogain(sd, ctrl->val);
		case V4L2_CID_GAIN:
			return sc031gs_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc031gs_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sc031gs_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc031gs_ctrl_ops = {
	.s_ctrl = sc031gs_s_ctrl,
	.g_volatile_ctrl = sc031gs_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc031gs_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = sc031gs_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int sc031gs_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	sc031gs_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int sc031gs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc031gs_info *info = to_state(sd);
	int ret = 0;

	if (enable) {

		sc031gs_write_array(sd, info->win->regs);
		sc031gs_s_again(sd, 100000);
		ret = sc031gs_write_array(sd, sc031gs_stream_on_mipi);
		pr_debug("sc031gs stream on\n");

	}
	else {
		ret = sc031gs_write_array(sd, sc031gs_stream_off_mipi);
		pr_debug("sc031gs stream off\n");
	}
	return ret;
}

int sc031gs_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct sc031gs_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sc031gs_core_ops = {
	.reset = sc031gs_reset,
	.init = sc031gs_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sc031gs_g_register,
	.s_register = sc031gs_s_register,
#endif

};

static const struct v4l2_subdev_video_ops sc031gs_video_ops = {
	.s_stream = sc031gs_s_stream,
	.g_frame_interval = sc031gs_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc031gs_pad_ops = {
	//.enum_frame_interval = sc031gs_enum_frame_interval,
	//.num_frame_size = sc031gs_enum_frame_size,
	//.enum_mbus_code = sc031gs_enum_mbus_code,
	.set_fmt = sc031gs_set_fmt,
	.get_fmt = sc031gs_get_fmt,
};

static const struct v4l2_subdev_ops sc031gs_ops = {
	.core = &sc031gs_core_ops,
	.video = &sc031gs_video_ops,
	.pad = &sc031gs_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int sc031gs_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct sc031gs_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	int tmp;

	dev_info(&client->dev, "sc031gs probing\n");

	/*
	*(volatile unsigned int *)0xb0010340 &= ~(3 << 30);

	*(volatile unsigned int *)0xb0010010 &= ~(3 << 16);
	*(volatile unsigned int *)0xb0010020 &= ~(3 << 16);
	*(volatile unsigned int *)0xb0010030 &= ~(3 << 16);
	*(volatile unsigned int *)0xb0010040 &= ~(3 << 16);

//	*(volatile unsigned int *)0xb00100a0 |= (3 << 16);
//	*(volatile unsigned int *)0xb00100b0 |= (3 << 16);
//	*(volatile unsigned int *)0xb00100c0 |= (3 << 16);
	*/

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	printk("[%d] gpio=%d\n", __LINE__, gpio);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,vcc-en-gpio", 0, &flags);
	printk("[%d] gpio=%d\n", __LINE__, gpio);
	if(gpio_is_valid(gpio)) {
		info->vcc_en.pin = gpio;
		info->vcc_en.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutp-gpio", 0, &flags);
	printk("[%d] gpio=%d\n", __LINE__, gpio);
	if(gpio_is_valid(gpio)) {
		info->ircutp.pin = gpio;
		info->ircutp.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutn-gpio", 0, &flags);
	printk("[%d] gpio=%d\n", __LINE__, gpio);
	if(gpio_is_valid(gpio)) {
		info->ircutn.pin = gpio;
		info->ircutn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	v4l2_i2c_subdev_init(sd, client, &sc031gs_ops);
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

	sc031gs_power(sd, 1);
	msleep(10);
	sc031gs_reset(sd, 1);
#if 1
	/* Make sure it's an sc031gs */
	ret = sc031gs_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an sc031gs chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 261773, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &sc031gs_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &sc031gs_win_sizes[0];

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "sc031gs Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int sc031gs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc031gs_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id sc031gs_id[] = {
	{ "sc031gs", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc031gs_id);

static const struct of_device_id sc031gs_of_match[] = {
	{.compatible = "ovti,sc031gs", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);


static struct i2c_driver sc031gs_driver = {
	.driver = {
		.name	= "sc031gs",
		.of_match_table = of_match_ptr(sc031gs_of_match),
	},
	.probe		= sc031gs_probe,
	.remove		= sc031gs_remove,
	.id_table	= sc031gs_id,
};

module_i2c_driver(sc031gs_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision sc031gs sensors");
MODULE_LICENSE("GPL");
