/*
 * A V4L2 driver for OmniVision OV6710 cameras.
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

#define OV6710_CHIP_ID_H	(0x67)
#define OV6710_CHIP_ID_L	(0x10)
#define OV6710_REG_END		0xffff
#define OV6710_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ov6710_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct ov6710_gpio {
	int pin;
	int active_level;
};

struct ov6710_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct ov6710_win_size *win;

	struct ov6710_gpio reset;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut ov6710_again_lut[] = {

	{0x0,0},
	{0x2,2909},
	{0x5,5731},
	{0x7,8472},
	{0x10,11136},
	{0x12,13726},
	{0x15,16248},
	{0x17,18704},
	{0x20,21097},
	{0x22,23432},
	{0x25,25710},
	{0x27,27935},
	{0x30,30109},
	{0x32,32234},
	{0x35,34312},
	{0x37,36345},
	{0x40,38336},
	{0x42,40285},
	{0x45,42195},
	{0x47,44068},
	{0x50,45904},
	{0x52,47704},
	{0x55,49472},
	{0x57,51207},
	{0x60,52910},
	{0x62,54584},
	{0x65,56228},
	{0x67,57844},
	{0x70,59433},
	{0x72,60996},
	{0x75,62534},
	{0x77,64047},
	{0x80,65536},
	{0x82,67001},
	{0x85,68445},
	{0x87,69867},
	{0x90,71267},
	{0x92,72648},
	{0x95,74008},
	{0x97,75349},
	{0xa0,76672},
	{0xa2,77976},
	{0xa5,79262},
	{0xa7,80531},
	{0xb0,81784},
	{0xb2,83020},
	{0xb5,84240},
	{0xb7,85444},
	{0xc0,86633},
	{0xc2,87808},
	{0xc5,88968},
	{0xc7,90114},
	{0xd0,91246},
	{0xd2,92365},
	{0xd5,93471},
	{0xd7,94564},
	{0xe0,95645},
	{0xe2,96713},
	{0xe5,97770},
	{0xe7,98814},
	{0xf0,99848},
	{0xf2,100870},
	{0xf5,101881},
	{0xf7,102882},
	{0x100,103872},
	{0x102,104851},
	{0x105,105821},
	{0x107,106781},
	{0x110,107731},
	{0x112,108672},
	{0x115,109604},
	{0x117,110526},
	{0x120,111440},
	{0x122,112344},
	{0x125,113240},
	{0x127,114128},
	{0x130,115008},
	{0x132,115879},
	{0x135,116743},
	{0x137,117598},
	{0x140,118446},
	{0x142,119287},
	{0x145,120120},
	{0x147,120946},
	{0x150,121764},
	{0x152,122576},
	{0x155,123380},
	{0x157,124178},
	{0x160,124969},
	{0x162,125754},
	{0x165,126532},
	{0x167,127304},
	{0x170,128070},
	{0x172,128829},
	{0x175,129583},
	{0x177,130330},
	{0x180,131072},
	{0x182,131807},
	{0x185,132537},
	{0x187,133262},
	{0x190,133981},
	{0x192,134694},
	{0x195,135403},
	{0x197,136106},
	{0x1a0,136803},
	{0x1a2,137496},
	{0x1a5,138184},
	{0x1a7,138866},
	{0x1b0,139544},
	{0x1b2,140217},
	{0x1b5,140885},
	{0x1b7,141549},
	{0x1c0,142208},
	{0x1c2,142862},
	{0x1c5,143512},
	{0x1c7,144157},
	{0x1d0,144798},
	{0x1d2,145435},
	{0x1d5,146067},
	{0x1d7,146696},
	{0x1e0,147320},
	{0x1e2,147940},
	{0x1e5,148556},
	{0x1e7,149168},
	{0x1f0,149776},
	{0x1f2,150380},
	{0x1f5,150980},
	{0x1f7,151577},
	{0x200,152169},
	{0x202,152758},
	{0x205,153344},
	{0x207,153926},
	{0x210,154504},
	{0x212,155079},
	{0x215,155650},
	{0x217,156218},
	{0x220,156782},
	{0x222,157344},
	{0x225,157901},
	{0x227,158456},
	{0x230,159007},
	{0x232,159555},
	{0x235,160100},
	{0x237,160642},
	{0x240,161181},
	{0x242,161716},
	{0x245,162249},
	{0x247,162779},
	{0x250,163306},
	{0x252,163829},
	{0x255,164350},
	{0x257,164868},
	{0x260,165384},
	{0x262,165896},
	{0x265,166406},
	{0x267,166913},
	{0x270,167417},
	{0x272,167919},
	{0x275,168418},
	{0x277,168914},
	{0x280,169408},
	{0x282,169899},
	{0x285,170387},
	{0x287,170873},
	{0x290,171357},
	{0x292,171838},
	{0x295,172317},
	{0x297,172793},
	{0x2a0,173267},
	{0x2a2,173739},
	{0x2a5,174208},
	{0x2a7,174675},
	{0x2b0,175140},
	{0x2b2,175602},
	{0x2b5,176062},
	{0x2b7,176520},
	{0x2c0,176976},
	{0x2c2,177429},
	{0x2c5,177880},
	{0x2c7,178329},
	{0x2d0,178776},
	{0x2d2,179221},
	{0x2d5,179664},
	{0x2d7,180105},
	{0x2e0,180544},
	{0x2e2,180981},
	{0x2e5,181415},
	{0x2e7,181848},
	{0x2f0,182279},
	{0x2f2,182707},
	{0x2f5,183134},
	{0x2f7,183559},
	{0x300,183982},
	{0x302,184403},
	{0x305,184823},
	{0x307,185240},
	{0x310,185656},
	{0x312,186070},
	{0x315,186482},
	{0x317,186892},
	{0x320,187300},
	{0x322,187707},
	{0x325,188112},
	{0x327,188515},
	{0x330,188916},
	{0x332,189316},
	{0x335,189714},
	{0x337,190111},
	{0x340,190505},
	{0x342,190899},
	{0x345,191290},
	{0x347,191680},
	{0x350,192068},
	{0x352,192455},
	{0x355,192840},
	{0x357,193224},
	{0x360,193606},
	{0x362,193986},
	{0x365,194365},
	{0x367,194743},
	{0x370,195119},
	{0x372,195493},
	{0x375,195866},
	{0x377,196237},
	{0x380,196608},
	{0x382,196976},
	{0x385,197343},
	{0x387,197709},
	{0x390,198073},
	{0x392,198436},
	{0x395,198798},
	{0x397,199158},
	{0x3a0,199517},
	{0x3a2,199874},
	{0x3a5,200230},
	{0x3a7,200585},
	{0x3b0,200939},
	{0x3b2,201291},
	{0x3b5,201642},
	{0x3b7,201991},
	{0x3c0,202339},
	{0x3c2,202686},
	{0x3c5,203032},
	{0x3c7,203377},
	{0x3d0,203720},
	{0x3d2,204062},
	{0x3d5,204402},
	{0x3d7,204742},
	{0x3e0,205080},
	{0x3e2,205417},
	{0x3e5,205753},
	{0x3e7,206088},
	{0x3f0,206421},
	{0x3f2,206754},
	{0x3f5,207085},
	{0x3f7,207415},
};

static inline struct ov6710_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov6710_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov6710_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list ov6710_init_regs_400_400_120fps_mipi[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x00},
	{0x3013, 0x12},
	{0x3014, 0x04},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x00},
	{0x3037, 0xf0},
	{0x3080, 0x01},
	{0x3081, 0x00},
	{0x3082, 0x01},
	{0x3098, 0x04},
	{0x3099, 0x28},
	{0x309a, 0x06},
	{0x309b, 0x04},
	{0x309c, 0x00},
	{0x309d, 0x00},
	{0x309e, 0x01},
	{0x309f, 0x00},
	{0x30b0, 0x0a},
	{0x30b1, 0x02},
	{0x30b2, 0x00},
	{0x30b3, 0x32},
	{0x30b4, 0x02},
	{0x30b5, 0x05},
	{0x3106, 0xd9},
	{0x3500, 0x00},
	{0x3501, 0x73},
	{0x3502, 0x20},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350a, 0x00},
	{0x350b, 0x10},
	{0x3600, 0xfc},
	{0x3620, 0xb7},
	{0x3621, 0x05},
	{0x3626, 0x31},
	{0x3627, 0x40},
	{0x3632, 0xa3},
	{0x3633, 0x34},
	{0x3634, 0x40},
	{0x3636, 0x00},
	{0x3660, 0x80},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x366a, 0x10},
	{0x366b, 0x06},
	{0x3680, 0xf4},
	{0x3681, 0x50},
	{0x3682, 0x00},
	{0x3708, 0x20},
	{0x3709, 0x40},
	{0x370d, 0x03},
	{0x373b, 0x02},
	{0x373c, 0x08},
	{0x3742, 0x00},
	{0x3744, 0x16},
	{0x3745, 0x08},
	{0x3781, 0xfc},
	{0x3788, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x04},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x01},
	{0x3805, 0x9b},
	{0x3806, 0x01},
	{0x3807, 0x9b},
	{0x3808, 0x01},
	{0x3809, 0x90},
	{0x380a, 0x01},
	{0x380b, 0x90},
	{0x380c, 0x05},
	{0x380d, 0xf2},
	{0x380e, 0x08},
	{0x380f, 0x36},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x382b, 0xfa},
	{0x382f, 0x04},
	{0x3832, 0x00},
	{0x3833, 0x05},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3882, 0x04},
	{0x3883, 0x00},
	{0x38a4, 0x10},
	{0x38a5, 0x00},
	{0x38b1, 0x03},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0xf0},
	{0x4004, 0x04},
	{0x404e, 0x01},
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4837, 0x43},
	{0x5a01, 0x00},
	{0x5a03, 0x00},
	{0x5a04, 0x10},
	{0x5a05, 0xa0},
	{0x5a06, 0x0c},
	{0x5a07, 0x78},
	{0x5a08, 0x00},
//	{0x0100, 0x01},
	{OV6710_REG_END, 0x00},/* END MARKER */
};

static struct regval_list ov6710_init_regs_200_200_120fps_mipi_bin[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x00},
	{0x3013, 0x12},
	{0x3014, 0x04},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x00},
	{0x3037, 0xf0},
	{0x3080, 0x01},
	{0x3081, 0x00},
	{0x3082, 0x01},
	{0x3098, 0x04},
	{0x3099, 0x14},
	{0x309a, 0x06},
	{0x309b, 0x02},
	{0x309c, 0x00},
	{0x309d, 0x00},
	{0x309e, 0x01},
	{0x309f, 0x00},
	{0x30b0, 0x0a},
	{0x30b1, 0x04},
	{0x30b2, 0x00},
	{0x30b3, 0x32},
	{0x30b4, 0x02},
	{0x30b5, 0x05},
	{0x3106, 0xd9},
	{0x3500, 0x00},
	{0x3501, 0x0e},
	{0x3502, 0x80},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3620, 0xb7},
	{0x3621, 0x05},
	{0x3626, 0x31},
	{0x3627, 0x40},
	{0x3632, 0xa3},
	{0x3633, 0x34},
	{0x3634, 0x40},
	{0x3636, 0x00},
	{0x3660, 0x80},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x366a, 0x00},
	{0x366b, 0x50},
	{0x3680, 0xf4},
	{0x3681, 0x50},
	{0x3682, 0x00},
	{0x3708, 0x20},
	{0x3709, 0x40},
	{0x370d, 0x03},
	{0x373b, 0x02},
	{0x373c, 0xe8},
	{0x3742, 0x00},
	{0x3744, 0x16},
	{0x3745, 0x08},
	{0x3781, 0xfc},
	{0x3788, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x01},
	{0x3805, 0x9f},
	{0x3806, 0x01},
	{0x3807, 0x9f},
	{0x3808, 0x00},
	{0x3809, 0xc8},
	{0x380a, 0x00},
	{0x380b, 0xc8},
	{0x380c, 0x05},
	{0x380d, 0x78},
	{0x380e, 0x00},
	{0x380f, 0xee},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x02},
	{0x3821, 0x01},
	{0x382b, 0xfa},
	{0x382f, 0x04},
	{0x3832, 0x00},
	{0x3833, 0x05},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3882, 0x04},
	{0x3883, 0x00},
	{0x38a4, 0x10},
	{0x38a5, 0x00},
	{0x38b1, 0x03},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0xf0},
	{0x4004, 0x02},
	{0x404e, 0x01},
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4837, 0x85},
	{0x0100, 0x01},
	{OV6710_REG_END, 0x00},/* END MARKER */
};

static struct regval_list ov6710_init_regs_200_200_120fps_mipi_skip[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x00},
	{0x3013, 0x12},
	{0x3014, 0x04},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x00},
	{0x3037, 0xf0},
	{0x3080, 0x01},
	{0x3081, 0x00},
	{0x3082, 0x01},
	{0x3098, 0x04},
	{0x3099, 0x14},
	{0x309a, 0x06},
	{0x309b, 0x02},
	{0x309c, 0x00},
	{0x309d, 0x00},
	{0x309e, 0x01},
	{0x309f, 0x00},
	{0x30b0, 0x0a},
	{0x30b1, 0x04},
	{0x30b2, 0x00},
	{0x30b3, 0x32},
	{0x30b4, 0x02},
	{0x30b5, 0x05},
	{0x3106, 0xd9},
	{0x3500, 0x00},
	{0x3501, 0x0e},
	{0x3502, 0x80},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3620, 0xb7},
	{0x3621, 0x05},
	{0x3626, 0x31},
	{0x3627, 0x40},
	{0x3632, 0xa3},
	{0x3633, 0x34},
	{0x3634, 0x40},
	{0x3636, 0x00},
	{0x3660, 0x80},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x366a, 0x00},
	{0x366b, 0x50},
	{0x3680, 0xf4},
	{0x3681, 0x50},
	{0x3682, 0x00},
	{0x3708, 0x20},
	{0x3709, 0x40},
	{0x370d, 0x03},
	{0x373b, 0x02},
	{0x373c, 0x08},
	{0x3742, 0xe0},
	{0x3744, 0x16},
	{0x3745, 0x08},
	{0x3781, 0xfc},
	{0x3788, 0x21},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x01},
	{0x3805, 0x9f},
	{0x3806, 0x01},
	{0x3807, 0x9f},
	{0x3808, 0x00},
	{0x3809, 0xc8},
	{0x380a, 0x00},
	{0x380b, 0xc8},
	{0x380c, 0x05},
	{0x380d, 0x78},
	{0x380e, 0x00},
	{0x380f, 0xee},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x31},
	{0x3815, 0x22},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x382b, 0xfa},
	{0x382f, 0x04},
	{0x3832, 0x00},
	{0x3833, 0x05},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3882, 0x04},
	{0x3883, 0x00},
	{0x38a4, 0x10},
	{0x38a5, 0x00},
	{0x38b1, 0x03},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0xf0},
	{0x4004, 0x02},
	{0x404e, 0x01},
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4837, 0x53},
	{0x0100, 0x01},
	{OV6710_REG_END, 0x00},/* END MARKER */
};

static struct regval_list ov6710_init_regs_100_100_120fps_mipi[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x00},
	{0x3013, 0x12},
	{0x3014, 0x04},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x00},
	{0x3037, 0xf0},
	{0x3080, 0x01},
	{0x3081, 0x00},
	{0x3082, 0x01},
	{0x3098, 0x04},
	{0x3099, 0x14},
	{0x309a, 0x08},
	{0x309b, 0x02},
	{0x309c, 0x00},
	{0x309d, 0x00},
	{0x309e, 0x01},
	{0x309f, 0x00},
	{0x30b0, 0x0a},
	{0x30b1, 0x06},
	{0x30b2, 0x00},
	{0x30b3, 0x32},
	{0x30b4, 0x02},
	{0x30b5, 0x05},
	{0x3106, 0xd9},
	{0x3500, 0x00},
	{0x3501, 0x07},
	{0x3502, 0x80},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3620, 0xb7},
	{0x3621, 0x05},
	{0x3626, 0x31},
	{0x3627, 0x40},
	{0x3632, 0xa3},
	{0x3633, 0x34},
	{0x3634, 0x40},
	{0x3636, 0x00},
	{0x3660, 0x80},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x366a, 0x00},
	{0x366b, 0x50},
	{0x3680, 0xf4},
	{0x3681, 0x50},
	{0x3682, 0x00},
	{0x3708, 0x20},
	{0x3709, 0x40},
	{0x370d, 0x03},
	{0x373b, 0x02},
	{0x373c, 0x08},
	{0x3742, 0xe0},
	{0x3744, 0x16},
	{0x3745, 0x08},
	{0x3781, 0xfc},
	{0x3788, 0x61},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x01},
	{0x3805, 0x9f},
	{0x3806, 0x01},
	{0x3807, 0x9f},
	{0x3808, 0x00},
	{0x3809, 0x64},
	{0x380a, 0x00},
	{0x380b, 0x64},
	{0x380c, 0x05},
	{0x380d, 0x00},
	{0x380e, 0x00},
	{0x380f, 0xc4},
	{0x3810, 0x00},
	{0x3811, 0x02},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x71},
	{0x3815, 0x44},
	{0x3820, 0x00},
	{0x3821, 0x01},
	{0x382b, 0xfa},
	{0x382f, 0x04},
	{0x3832, 0x00},
	{0x3833, 0x05},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3882, 0x04},
	{0x3883, 0x00},
	{0x38a4, 0x10},
	{0x38a5, 0x00},
	{0x38b1, 0x03},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0xf0},
	{0x4004, 0x02},
	{0x404e, 0x01},
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4837, 0xc8},
	{0x0100, 0x01},
	{OV6710_REG_END, 0x00},/* END MARKER */
};

static struct regval_list ov6710_stream_on_mipi[] = {
	{0x0100, 0x01},
	{OV6710_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list ov6710_stream_off_mipi[] = {
	{0x0100, 0x00},
	{OV6710_REG_END, 0x00},	/* END MARKER */
};

static int ov6710_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct ov6710_info *info = to_state(sd);
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

static int ov6710_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct ov6710_info *info = to_state(sd);
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

static int ov6710_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != OV6710_REG_END) {
		if (vals->reg_num == OV6710_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov6710_read(sd, vals->reg_num, &val);
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
static int ov6710_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != OV6710_REG_END) {
		if (vals->reg_num == OV6710_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov6710_write(sd, vals->reg_num, vals->value);
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
static int ov6710_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ov6710_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}


static int ov6710_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov6710_info *info = to_state(sd);
	int ret = 0;

	ret = ov6710_write_array(sd, info->win->regs);

	return ret;
}



static int ov6710_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = ov6710_read(sd, 0x300a, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV6710_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = ov6710_read(sd, 0x300b, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV6710_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}


static struct ov6710_win_size ov6710_win_sizes[] = {
	{

		.sensor_info.mipi_cfg.twidth		= 400,
		.sensor_info.mipi_cfg.theight		= 400,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 120 << 16 | 1,


		.width		= 400,
		.height		= 400,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov6710_init_regs_400_400_120fps_mipi,
	},
	{
		.sensor_info.mipi_cfg.twidth		= 200,
		.sensor_info.mipi_cfg.theight		= 200,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 120 << 16 | 1,

		.width		= 200,
		.height		= 200,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov6710_init_regs_200_200_120fps_mipi_skip,
	},
	{
		.sensor_info.mipi_cfg.twidth		= 100,
		.sensor_info.mipi_cfg.theight		= 100,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 120 << 16 | 1,

		.width		= 100,
		.height		= 100,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov6710_init_regs_100_100_120fps_mipi,
	},
};

static int ov6710_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_OV6710_FMTS)
		return -EINVAL;

	code->code = ov6710_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int ov6710_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ov6710_format_struct *ovfmt;
	struct ov6710_win_size *wsize;
	struct ov6710_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int ov6710_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct ov6710_info *info = to_state(sd);
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

static int ov6710_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov6710_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov6710_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov6710_s_vflip(struct v4l2_subdev *sd, int value)
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
static int ov6710_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ov6710_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ov6710_again_lut); i++) {
		lut = &ov6710_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(ov6710_again_lut); i++) {
		lut = &ov6710_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return 0;
}

static int ov6710_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct ov6710_info *info = to_state(sd);
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret += ov6710_read(sd, 0x350a, &v);

	reg_val |= v;
	ret += ov6710_read(sd, 0x350b, &v);
	reg_val |= v << 8;


	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int ov6710_s_again(struct v4l2_subdev *sd, int value)
{
	struct ov6710_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	int i;


	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += ov6710_write(sd, 0x350a, (unsigned char)((reg_value>>8) & 0x3));
	ret += ov6710_write(sd, 0x350b, (unsigned char)(reg_value & 0xff));

	return ret;
}

/*
 * Tweak autogain.
 */
static int ov6710_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov6710_s_exp(struct v4l2_subdev *sd, int value)
{
	struct ov6710_info *info = to_state(sd);
	int ret = 0;
	unsigned char v;
	int frame_length = 0;

	ret = ov6710_read(sd, 0x380e, &v);
	frame_length = v << 8;

	ret += ov6710_read(sd, 0x380f, &v);
	frame_length |= v;

	if(value < frame_length){
		ret += ov6710_write(sd, 0x3502, ((unsigned char)(value & 0xf)) << 4);
		ret += ov6710_write(sd, 0x3501, (unsigned char)((value >> 4) & 0xff));
		ret += ov6710_write(sd, 0x3500, (unsigned char)((value >> 12) & 0xf));
	}
	return ret;
}

static int ov6710_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov6710_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ov6710_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov6710_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ov6710_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov6710_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov6710_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov6710_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ov6710_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov6710_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ov6710_s_gain turns off auto gain */
			return ov6710_s_gain(sd, info->gain->val);
		}
		return ov6710_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ov6710_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov6710_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov6710_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov6710_ctrl_ops = {
	.s_ctrl = ov6710_s_ctrl,
	.g_volatile_ctrl = ov6710_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov6710_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ov6710_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov6710_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ov6710_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int ov6710_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov6710_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ov6710_write_array(sd, info->win->regs);
		ov6710_s_again(sd, 100000);
		ret = ov6710_write_array(sd, ov6710_stream_on_mipi);
		pr_debug("ov6710 stream on\n");

	}
	else {
		ret = ov6710_write_array(sd, ov6710_stream_off_mipi);
		pr_debug("ov6710 stream off\n");
	}
	return ret;
}

int ov6710_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ov6710_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov6710_core_ops = {
	//.reset = ov6710_reset,
	.init = ov6710_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov6710_g_register,
	.s_register = ov6710_s_register,
#endif

};

static const struct v4l2_subdev_video_ops ov6710_video_ops = {
	.s_stream = ov6710_s_stream,
	.g_frame_interval = ov6710_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov6710_pad_ops = {
	//.enum_frame_interval = ov6710_enum_frame_interval,
	//.num_frame_size = ov6710_enum_frame_size,
	//.enum_mbus_code = ov6710_enum_mbus_code,
	.set_fmt = ov6710_set_fmt,
	.get_fmt = ov6710_get_fmt,
};

static const struct v4l2_subdev_ops ov6710_ops = {
	.core = &ov6710_core_ops,
	.video = &ov6710_video_ops,
	.pad = &ov6710_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ov6710_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ov6710_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
    unsigned int iovcc_1v8;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	iovcc_1v8 = of_property_read_bool(client->dev.of_node, "ingenic,iovcc_1v8");
	if(iovcc_1v8) {
		/*X2000 spec*/
		*(volatile unsigned int *)0xb00000e0 |= (1<<30);
	}
	v4l2_i2c_subdev_init(sd, client, &ov6710_ops);
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

	ov6710_reset(sd, 1);
#if 1
	/* Make sure it's an ov6710 */
	ret = ov6710_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an ov6710 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 261773, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ov6710_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &ov6710_win_sizes[0];

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "ov6710 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int ov6710_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6710_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id ov6710_id[] = {
	{ "ov6710", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov6710_id);

static const struct of_device_id ov6710_of_match[] = {
	{.compatible = "ovti,ov6710", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);


static struct i2c_driver ov6710_driver = {
	.driver = {
		.name	= "ov6710",
		.of_match_table = of_match_ptr(ov6710_of_match),
	},
	.probe		= ov6710_probe,
	.remove		= ov6710_remove,
	.id_table	= ov6710_id,
};

module_i2c_driver(ov6710_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov6710 sensors");
MODULE_LICENSE("GPL");
