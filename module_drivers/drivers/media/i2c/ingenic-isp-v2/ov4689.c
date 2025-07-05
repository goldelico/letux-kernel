/*
 * A V4L2 driver for OmniVision OV4689 cameras.
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

#define OV4689_CHIP_ID_H	(0x46)
#define OV4689_CHIP_ID_L	(0x88)
#define OV4689_REG_END		0xffff
#define OV4689_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ov4689_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct ov4689_gpio {
	int pin;
	int active_level;
};

struct ov4689_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct ov4689_win_size *win;

	struct ov4689_gpio reset;
	struct ov4689_gpio ircutp;
	struct ov4689_gpio ircutn;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut ov4689_again_lut[] = {

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

static inline struct ov4689_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov4689_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov4689_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list ov4689_init_regs_1920_1080_30fps_mipi[] = {
	//1080p@30fps 2 or 4 lane
	/* @@ 0 35 RES_1920x1080_30fps_816Mbps  2Lane */
	/* ;SCLK = 80MHz MIPI CLK = 816Mbps/lane Mipi PCLK = 102MHz */
	/* ;HTS = 2294 VTS = 1162 */
	{0x0103, 0x01},
	{0x3638, 0x00},
	{0x0300, 0x00},
	{0x0302, 0x23},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x030b, 0x00},
	{0x030d, 0x1e},
	{0x030e, 0x04},
	{0x030f, 0x02},
	{0x0312, 0x01},
	{0x031e, 0x00},
	{0x3000, 0x20},
	{0x3002, 0x00},
//	{0x3018, 0x72}, //4lan
	{0x3018, 0x32}, //2lan
	{0x3020, 0x93},
	{0x3021, 0x03},
	{0x3022, 0x01},
	{0x3031, 0x0a},
	{0x303f, 0x0c},
	{0x3305, 0xf1},
	{0x3307, 0x04},
	{0x3309, 0x29},
	{0x3500, 0x00},
	{0x3501, 0x4c},
	{0x3502, 0x00},
	{0x3503, 0x04},
	{0x3504, 0x00},
	{0x3505, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x00},
	{0x3508, 0x00},
	{0x3509, 0x80},
	{0x350a, 0x00},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x350e, 0x00},
	{0x350f, 0x80},
	{0x3510, 0x00},
	{0x3511, 0x00},
	{0x3512, 0x00},
	{0x3513, 0x00},
	{0x3514, 0x00},
	{0x3515, 0x80},
	{0x3516, 0x00},
	{0x3517, 0x00},
	{0x3518, 0x00},
	{0x3519, 0x00},
	{0x351a, 0x00},
	{0x351b, 0x80},
	{0x351c, 0x00},
	{0x351d, 0x00},
	{0x351e, 0x00},
	{0x351f, 0x00},
	{0x3520, 0x00},
	{0x3521, 0x80},
	{0x3522, 0x08},
	{0x3524, 0x08},
	{0x3526, 0x08},
	{0x3528, 0x08},
	{0x352a, 0x08},
	{0x3602, 0x00},
	{0x3603, 0x40},
	{0x3604, 0x02},
	{0x3605, 0x00},
	{0x3606, 0x00},
	{0x3607, 0x00},
	{0x3609, 0x12},
	{0x360a, 0x40},
	{0x360c, 0x08},
	{0x360f, 0xe5},
	{0x3608, 0x8f},
	{0x3611, 0x00},
	{0x3613, 0xf7},
	{0x3616, 0x58},
	{0x3619, 0x99},
	{0x361b, 0x60},
	{0x361c, 0x7a},
	{0x361e, 0x79},
	{0x361f, 0x02},
	{0x3632, 0x00},
	{0x3633, 0x10},
	{0x3634, 0x10},
	{0x3635, 0x10},
	{0x3636, 0x15},
	{0x3646, 0x86},
	{0x364a, 0x0b},
	{0x3700, 0x17},
	{0x3701, 0x22},
	{0x3703, 0x10},
	{0x370a, 0x37},
	{0x3705, 0x00},
	{0x3706, 0x63},
	{0x3709, 0x3c},
	{0x370b, 0x01},
	{0x370c, 0x30},
	{0x3710, 0x24},
	{0x3711, 0x0c},
	{0x3716, 0x00},
	{0x3720, 0x28},
	{0x3729, 0x7b},
	{0x372a, 0x84},
	{0x372b, 0xbd},
	{0x372c, 0xbc},
	{0x372e, 0x52},
	{0x373c, 0x0e},
	{0x373e, 0x33},
	{0x3743, 0x10},
	{0x3744, 0x88},
	{0x3745, 0xc0},
	{0x374a, 0x43},
	{0x374c, 0x00},
	{0x374e, 0x23},
	{0x3751, 0x7b},
	{0x3752, 0x84},
	{0x3753, 0xbd},
	{0x3754, 0xbc},
	{0x3756, 0x52},
	{0x375c, 0x00},
	{0x3760, 0x00},
	{0x3761, 0x00},
	{0x3762, 0x00},
	{0x3763, 0x00},
	{0x3764, 0x00},
	{0x3767, 0x04},
	{0x3768, 0x04},
	{0x3769, 0x08},
	{0x376a, 0x08},
	{0x376b, 0x20},
	{0x376c, 0x00},
	{0x376d, 0x00},
	{0x376e, 0x00},
	{0x3773, 0x00},
	{0x3774, 0x51},
	{0x3776, 0xbd},
	{0x3777, 0xbd},
	{0x3781, 0x18},
	{0x3783, 0x25},
	{0x3798, 0x1b},
	{0x3800, 0x01},
	{0x3801, 0x88},
	{0x3802, 0x00},
	{0x3803, 0xe0},
	{0x3804, 0x09},
	{0x3805, 0x17},
	{0x3806, 0x05},
	{0x3807, 0x1f},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x08},
	{0x380d, 0xf6},
	{0x380e, 0x04},
	{0x380f, 0x8A},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3819, 0x01},
	{0x3820, 0x00},
	{0x3821, 0x06},
	{0x3829, 0x00},
	{0x382a, 0x01},
	{0x382b, 0x01},
	{0x382d, 0x7f},
	{0x3830, 0x04},
	{0x3836, 0x01},
	{0x3837, 0x00},
	{0x3841, 0x02},
	{0x3846, 0x08},
	{0x3847, 0x07},
	{0x3d85, 0x36},
	{0x3d8c, 0x71},
	{0x3d8d, 0xcb},
	{0x3f0a, 0x00},
	{0x4000, 0xf1},
	{0x4001, 0x40},
	{0x4002, 0x04},
	{0x4003, 0x14},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x401a, 0x00},
	{0x401b, 0x00},
	{0x401c, 0x00},
	{0x401d, 0x00},
	{0x401f, 0x00},
	{0x4020, 0x00},
	{0x4021, 0x10},
	{0x4022, 0x06},
	{0x4023, 0x13},
	{0x4024, 0x07},
	{0x4025, 0x40},
	{0x4026, 0x07},
	{0x4027, 0x50},
	{0x4028, 0x00},
	{0x4029, 0x02},
	{0x402a, 0x06},
	{0x402b, 0x04},
	{0x402c, 0x02},
	{0x402d, 0x02},
	{0x402e, 0x0e},
	{0x402f, 0x04},
	{0x4302, 0xff},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4306, 0x00},
	{0x4308, 0x02},
	{0x4500, 0x6c},
	{0x4501, 0xc4},
	{0x4502, 0x40},
	{0x4503, 0x01},
	{0x4601, 0x77},
	{0x4800, 0x04},
	{0x4813, 0x08},
	{0x481f, 0x40},
	{0x4829, 0x78},
//	{0x4837, 0x14}, //for zebra
	{0x4837, 0x80}, //for halley5
	{0x4b00, 0x2a},
	{0x4b0d, 0x00},
	{0x4d00, 0x04},
	{0x4d01, 0x42},
	{0x4d02, 0xd1},
	{0x4d03, 0x93},
	{0x4d04, 0xf5},
	{0x4d05, 0xc1},
	{0x5000, 0xf3},
	{0x5001, 0x11},
	{0x5004, 0x00},
	{0x500a, 0x00},
	{0x500b, 0x00},
	{0x5032, 0x00},
	{0x5040, 0x00},
	{0x5050, 0x0c},
	{0x5500, 0x00},
	{0x5501, 0x10},
	{0x5502, 0x01},
	{0x5503, 0x0f},
	{0x8000, 0x00},
	{0x8001, 0x00},
	{0x8002, 0x00},
	{0x8003, 0x00},
	{0x8004, 0x00},
	{0x8005, 0x00},
	{0x8006, 0x00},
	{0x8007, 0x00},
	{0x8008, 0x00},
	{0x3638, 0x00},
	{0x0100, 0x00},
	{OV4689_REG_END, 0x00},/* END MARKER */
};

static struct regval_list ov4689_init_regs_2592_1520_30fps_mipi[] = {
	{0x0103, 0x01},
	{0x3638, 0x00},
	{0x0300, 0x00},
	{0x0302, 0x19}, //0x19
	{0x0303, 0x00},
	{0x0304, 0x03}, //0x03
	{0x030b, 0x00},
	{0x030d, 0x1e},
	{0x030e, 0x04},
	{0x030f, 0x01},
	{0x0311, 0x00},
	{0x0312, 0x01},
	{0x031e, 0x00},
	{0x3000, 0x20},
	{0x3002, 0x00},
	{0x3018, 0x32},
	{0x3019, 0x0c},
	{0x3020, 0x93},
	{0x3021, 0x03},
	{0x3022, 0x01},
	{0x3031, 0x0a},
	{0x303f, 0x0c},
	{0x3305, 0xf1},
	{0x3307, 0x04},
	{0x3309, 0x29},
	{0x3500, 0x00},
	{0x3501, 0x60},
	{0x3502, 0x00},
	{0x3503, 0x04},
	{0x3504, 0x00},
	{0x3505, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x00},
	{0x3508, 0x00},
	{0x3509, 0x80},
	{0x350a, 0x00},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x350e, 0x00},
	{0x350f, 0x80},
	{0x3510, 0x00},
	{0x3511, 0x00},
	{0x3512, 0x00},
	{0x3513, 0x00},
	{0x3514, 0x00},
	{0x3515, 0x80},
	{0x3516, 0x00},
	{0x3517, 0x00},
	{0x3518, 0x00},
	{0x3519, 0x00},
	{0x351a, 0x00},
	{0x351b, 0x80},
	{0x351c, 0x00},
	{0x351d, 0x00},
	{0x351e, 0x00},
	{0x351f, 0x00},
	{0x3520, 0x00},
	{0x3521, 0x80},
	{0x3522, 0x08},
	{0x3524, 0x08},
	{0x3526, 0x08},
	{0x3528, 0x08},
	{0x352a, 0x08},
	{0x3602, 0x00},
	{0x3603, 0x40},
	{0x3604, 0x02},
	{0x3605, 0x00},
	{0x3606, 0x00},
	{0x3607, 0x00},
	{0x3609, 0x12},
	{0x360a, 0x40},
	{0x360c, 0x08},
	{0x360f, 0xe5},
	{0x3608, 0x8f},
	{0x3611, 0x00},
	{0x3613, 0xf7},
	{0x3616, 0x58},
	{0x3619, 0x99},
	{0x361b, 0x60},
	{0x361c, 0x7a},
	{0x361e, 0x79},
	{0x361f, 0x02},
	{0x3632, 0x00},
	{0x3633, 0x10},
	{0x3634, 0x10},
	{0x3635, 0x10},
	{0x3636, 0x15},
	{0x3646, 0x86},
	{0x364a, 0x0b},
	{0x3700, 0x17},
	{0x3701, 0x22},
	{0x3703, 0x10},
	{0x370a, 0x37},
	{0x3705, 0x00},
	{0x3706, 0x63},
	{0x3709, 0x3c},
	{0x370b, 0x01},
	{0x370c, 0x30},
	{0x3710, 0x24},
	{0x3711, 0x0c},
	{0x3716, 0x00},
	{0x3720, 0x28},
	{0x3729, 0x7b},
	{0x372a, 0x84},
	{0x372b, 0xbd},
	{0x372c, 0xbc},
	{0x372e, 0x52},
	{0x373c, 0x0e},
	{0x373e, 0x33},
	{0x3743, 0x10},
	{0x3744, 0x88},
	{0x3745, 0xc0},
	{0x374a, 0x43},
	{0x374c, 0x00},
	{0x374e, 0x23},
	{0x3751, 0x7b},
	{0x3752, 0x84},
	{0x3753, 0xbd},
	{0x3754, 0xbc},
	{0x3756, 0x52},
	{0x375c, 0x00},
	{0x3760, 0x00},
	{0x3761, 0x00},
	{0x3762, 0x00},
	{0x3763, 0x00},
	{0x3764, 0x00},
	{0x3767, 0x04},
	{0x3768, 0x04},
	{0x3769, 0x08},
	{0x376a, 0x08},
	{0x376b, 0x20},
	{0x376c, 0x00},
	{0x376d, 0x00},
	{0x376e, 0x00},
	{0x3773, 0x00},
	{0x3774, 0x51},
	{0x3776, 0xbd},
	{0x3777, 0xbd},
	{0x3781, 0x18},
	{0x3783, 0x25},
	{0x3798, 0x1b},
	{0x3800, 0x00},
	{0x3801, 0x38},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x67},
	{0x3806, 0x05},
	{0x3807, 0xfb},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x05},
	{0x380b, 0xf0},
	{0x380c, 0x0A},
	{0x380d, 0x82},//0x18
	{0x380e, 0x06},
	{0x380f, 0xfb},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3819, 0x01},
	{0x3820, 0x00},
	{0x3821, 0x06},
	{0x3829, 0x00},
	{0x382a, 0x01},
	{0x382b, 0x01},
	{0x382d, 0x7f},
	{0x3830, 0x04},
	{0x3836, 0x01},
	{0x3837, 0x00},
	{0x3841, 0x02},
	{0x3846, 0x08},
	{0x3847, 0x07},
	{0x3d85, 0x36},
	{0x3d8c, 0x71},
	{0x3d8d, 0xcb},
	{0x3f0a, 0x00},
	{0x4000, 0xf1},
	{0x4001, 0x40},
	{0x4002, 0x04},
	{0x4003, 0x14},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x401a, 0x00},
	{0x401b, 0x00},
	{0x401c, 0x00},
	{0x401d, 0x00},
	{0x401f, 0x00},
	{0x4020, 0x00},
	{0x4021, 0x10},
	{0x4022, 0x08},
	{0x4023, 0xb3},
	{0x4024, 0x09},
	{0x4025, 0xe0},
	{0x4026, 0x09},
	{0x4027, 0xf0},//
	{0x4028, 0x00},
	{0x4029, 0x02},
	{0x402a, 0x06},
	{0x402b, 0x04},
	{0x402c, 0x02},
	{0x402d, 0x02},
	{0x402e, 0x0e},
	{0x402f, 0x04},
	{0x4302, 0xff},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4306, 0x00},
	{0x4308, 0x02},
	{0x4500, 0x6c},
	{0x4501, 0xc4},
	{0x4502, 0x40},
	{0x4503, 0x01},
	{0x4601, 0xA1},//
	{0x4800, 0x04},
	{0x4813, 0x08},
	{0x481f, 0x40},
	{0x4829, 0x78},
	{0x4837, 0x10},
	{0x4b00, 0x2a},
	{0x4b0d, 0x00},
	{0x4d00, 0x04},
	{0x4d01, 0x42},
	{0x4d02, 0xd1},
	{0x4d03, 0x93},
	{0x4d04, 0xf5},
	{0x4d05, 0xc1},
	{0x5000, 0xf3},
	{0x5001, 0x11},
	{0x5004, 0x00},
	{0x500a, 0x00},
	{0x500b, 0x00},
	{0x5032, 0x00},
	{0x5040, 0x00},
	{0x5050, 0x0c},
	{0x5500, 0x00},
	{0x5501, 0x10},
	{0x5502, 0x01},
	{0x5503, 0x0f},
	{0x8000, 0x00},
	{0x8001, 0x00},
	{0x8002, 0x00},
	{0x8003, 0x00},
	{0x8004, 0x00},
	{0x8005, 0x00},
	{0x8006, 0x00},
	{0x8007, 0x00},
	{0x8008, 0x00},
	{0x3638, 0x00},
	{0x0100, 0x00},
	{OV4689_REG_END, 0x00},/* END MARKER */
};




static struct regval_list ov4689_stream_on[] = {
	{0x0100,0x01},
	{OV4689_REG_END, 0x00},
};

static struct regval_list ov4689_stream_off[] = {
	{0x0100,0x00},
	{OV4689_REG_END, 0x00},
};


int ov4689_read(struct v4l2_subdev *sd, unsigned short reg,
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
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ov4689_write(struct v4l2_subdev *sd, unsigned short reg,
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
	unsigned int timeout  = 100;
	while(timeout--){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == -EAGAIN){
			msleep(100);
			continue;
		}
		else
			break;
	}
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ov4689_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != OV4689_REG_END) {
		if (vals->reg_num == OV4689_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov4689_read(sd, vals->reg_num, &val);
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
static int ov4689_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != OV4689_REG_END) {
		if (vals->reg_num == OV4689_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov4689_write(sd, vals->reg_num, vals->value);
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
static int ov4689_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ov4689_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int ov4689_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct ov4689_info *info = to_state(sd);

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


static int ov4689_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = ov4689_read(sd, 0x300a, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV4689_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = ov4689_read(sd, 0x300b, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV4689_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;
	return 0;

}

static struct ov4689_win_size ov4689_win_sizes[] = {
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 30 << 16 | 1,
		.sensor_info.total_width			= 2294,
		.sensor_info.total_height			= 1162,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 80,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW10,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW10,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov4689_init_regs_1920_1080_30fps_mipi,
	},
	{
		.width				= 2592,
		.height				= 1520,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.total_width			= 2690,
		.sensor_info.total_height			= 1787,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 80,
		.sensor_info.mipi_cfg.twidth		= 2592,
		.sensor_info.mipi_cfg.theight		= 1520,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW10,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW10,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov4689_init_regs_2592_1520_30fps_mipi,
	},
};


static int ov4689_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_OV4689_FMTS)
		return -EINVAL;

	code->code = ov4689_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int ov4689_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ov4689_format_struct *ovfmt;
	struct ov4689_win_size *wsize;
	struct ov4689_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int ov4689_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct ov4689_info *info = to_state(sd);
	struct ov4689_win_size *wsize = info->win;
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
	*(unsigned int *)fmt->reserved = &wsize->sensor_info; /*reserved[0] reserved[1]*/

//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int ov4689_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov4689_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov4689_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov4689_s_vflip(struct v4l2_subdev *sd, int value)
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
static int ov4689_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ov4689_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

//	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ov4689_again_lut); i++) {
		lut = &ov4689_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(ov4689_again_lut); i++) {
		lut = &ov4689_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return 0;
}

static int ov4689_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct ov4689_info *info = to_state(sd);
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret += ov4689_read(sd, 0x3509, &v);

	reg_val |= v;
	ret += ov4689_read(sd, 0x3508, &v);
	reg_val |= v << 8;


	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int ov4689_s_again(struct v4l2_subdev *sd, int value)
{
	struct ov4689_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	int i;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += ov4689_write(sd, 0x3509, (unsigned char)(reg_value & 0xff));
	ret += ov4689_write(sd, 0x3508, (unsigned char)((reg_value>>8) & 0xff));
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Tweak autogain.
 */
static int ov4689_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov4689_s_exp(struct v4l2_subdev *sd, int value)
{
	struct ov4689_info *info = to_state(sd);
	int ret = 0;

	ret += ov4689_write(sd, 0x3502, ((unsigned char)(value & 0xf)) << 4);
	ret += ov4689_write(sd, 0x3501, (unsigned char)((value >> 4) & 0xff));
	ret += ov4689_write(sd, 0x3500, (unsigned char)((value >> 12) & 0xf));

	return ret;
}

static int ov4689_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov4689_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ov4689_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov4689_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ov4689_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov4689_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov4689_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov4689_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ov4689_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov4689_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ov4689_s_gain turns off auto gain */
			return ov4689_s_gain(sd, info->gain->val);
		}
		return ov4689_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ov4689_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov4689_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov4689_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov4689_ctrl_ops = {
	.s_ctrl = ov4689_s_ctrl,
	.g_volatile_ctrl = ov4689_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov4689_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ov4689_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov4689_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ov4689_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

static int ov4689_core_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov4689_info *info = to_state(sd);
	int ret = 0;

	ret = ov4689_write_array(sd, info->win->regs);

	return ret;
}


int ov4689_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov4689_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ret = ov4689_write_array(sd, ov4689_stream_on);
		printk("ov4689 stream on\n");

	}
	else {
		ret = ov4689_write_array(sd, ov4689_stream_off);
		printk("ov4689 stream off\n");
	}
	return ret;
}


int ov4689_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ov4689_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov4689_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov4689_g_register,
	.s_register = ov4689_s_register,
#endif
	.init = ov4689_core_init,
};

static const struct v4l2_subdev_video_ops ov4689_video_ops = {
	.s_stream = ov4689_s_stream,
	.g_frame_interval	= ov4689_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov4689_pad_ops = {
	//.enum_frame_interval = ov4689_enum_frame_interval,
	//.num_frame_size = ov4689_enum_frame_size,
	//.enum_mbus_code = ov4689_enum_mbus_code,
	.set_fmt = ov4689_set_fmt,
	.get_fmt = ov4689_get_fmt,
};

static const struct v4l2_subdev_ops ov4689_ops = {
	.core = &ov4689_core_ops,
	.video = &ov4689_video_ops,
	.pad = &ov4689_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ov4689_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ov4689_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned long rate;
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

	v4l2_i2c_subdev_init(sd, client, &ov4689_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
	char id_div[9];
	char id_mux[9];
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
	info->sclka = devm_clk_get(&client->dev, id_mux);

	rate = v4l2_clk_get_rate(info->clk);
	if (((rate / 1000) % 24000) != 0) {
		ret = clk_set_parent(info->sclka, clk_get(NULL, "epll"));
		info->sclka = devm_clk_get(&client->dev, "epll");
		if (IS_ERR(info->sclka)) {
			pr_err("get sclka failed\n");
		} else {
			rate = clk_get_rate(info->sclka);
			if (((rate / 1000) % 27000) != 0) {
				clk_set_rate(info->sclka, 891000000);
			}
		}
	}

	ret = v4l2_clk_set_rate(info->clk, 24000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");

	ov4689_reset(sd, 1);
#if 1
	/* Make sure it's an ov4689 */
	ret = ov4689_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an ov4689 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	ov4689_ircut(sd, 0);
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 261773, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ov4689_ctrl_ops,
			V4L2_CID_EXPOSURE, 0, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &ov4689_win_sizes[1];

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "ov4689 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int ov4689_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov4689_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id ov4689_id[] = {
	{ "ov4689", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov4689_id);

static const struct of_device_id ov4689_of_match[] = {
	{.compatible = "ovit,ov4689", },
	{},
};
MODULE_DEVICE_TABLE(of, ov4689_of_match);


static struct i2c_driver ov4689_driver = {
	.driver = {
		.name	= "ov4689",
		.of_match_table = of_match_ptr(ov4689_of_match),
	},
	.probe		= ov4689_probe,
	.remove		= ov4689_remove,
	.id_table	= ov4689_id,
};

module_i2c_driver(ov4689_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov4689 sensors");
MODULE_LICENSE("GPL");
