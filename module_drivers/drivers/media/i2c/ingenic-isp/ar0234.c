/*
 * A V4L2 driver for OmniVision AR0234 cameras.
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
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>

#include <isp-sensor.h>

#define AR0234_CHIP_ID_H	(0x0a)
#define AR0234_CHIP_ID_L	(0x56)
#define AR0234_CHIP_ID          (0x0356)
#define AR0234_CHIP_ID_REG      (0x3000)

#define AR0234_REG_END		0xffff
#define AR0234_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ar0234_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct ar0234_gpio {
	int pin;
	int active_level;
};

struct ar0234_supply{
	struct regulator *pwen;
	struct regulator *shutdown;
};

struct ar0234_info {
	struct ar0234_supply supply;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct ar0234_win_size *win;

	struct ar0234_gpio reset;
	struct ar0234_gpio expo;
	struct ar0234_gpio oe;
	struct ar0234_gpio pwen;
	struct ar0234_gpio shutdown;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut ar0234_again_lut[] = {
    { 0x0, 0 },
    { 0x1, 2794 },
    { 0x2, 6397 },
    { 0x3, 9011 },
    { 0x4, 12388 },
    { 0x5, 16447 },
    { 0x6, 19572 },
    { 0x7, 23340 },
    { 0x8, 26963 },
    { 0x9, 31135 },
    { 0xa, 35780 },
    { 0xb, 39588 },
    { 0xc, 44438 },
    { 0xd, 49051 },
    { 0xe, 54517 },
    { 0xf, 59685 },
    { 0x10, 65536 },
    { 0x12, 71490 },
    { 0x14, 78338 },
    { 0x16, 85108 },
    { 0x18, 92854 },
    { 0x1a, 100992 },
    { 0x1c, 109974 },
    { 0x1e, 120053 },
    { 0x20, 131072 },
    { 0x22, 137247 },
    { 0x24, 143667 },
    { 0x26, 150644 },
    { 0x28, 158212 },
    { 0x2a, 166528 },
    { 0x2c, 175510 },
    { 0x2e, 185457 },
    { 0x30, 196608 },
    { 0x32, 202783 },
    { 0x34, 209203 },
    { 0x36, 216276 },
    { 0x38, 223748 },
    { 0x3a, 232064 },
    { 0x3c, 241046 },
    { 0x3e, 250993 },
    { 0x40, 262144 },
};

static inline struct ar0234_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0234_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar0234_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned short value;
};

static struct regval_list ar0234_init_regs_1080p_60fps_mipi[] = {
	/*
	   [AR0234_MIPI_2Lane_1920x1080_10bits_24MCLK_891Mbps_60fps]
	   ;load = seq
	   load = PLL_settings
	   load = Timing_settings
	   [seq]
	   */
	{ AR0234_REG_DELAY, 200},
	{ 0x301A, 0x00D9}, // RESET_REGISTER
	{ AR0234_REG_DELAY, 200},
	{ 0x3F4C, 0x121F}, // PIX_DEF_1D_DDC_LO_DEF
	{ 0x3F4E, 0x121F}, // PIX_DEF_1D_DDC_HI_DEF
	{ 0x3F50, 0x0B81}, // PIX_DEF_1D_DDC_EDGE
	{ 0x31E0, 0x0003}, // PIX_DEF_ID
	{ 0x31E0, 0x0003}, // PIX_DEF_ID
	{ 0x30B0, 0x0028}, // DIGITAL_TEST
	{ AR0234_REG_DELAY, 200},
	{ 0x3088, 0x8000}, // SEQ_CTRL_PORT
	{ 0x3086, 0xC1AE}, // SEQ_DATA_PORT
	{ 0x3086, 0x327F}, // SEQ_DATA_PORT
	{ 0x3086, 0x5780}, // SEQ_DATA_PORT
	{ 0x3086, 0x272F}, // SEQ_DATA_PORT
	{ 0x3086, 0x7416}, // SEQ_DATA_PORT
	{ 0x3086, 0x7E13}, // SEQ_DATA_PORT
	{ 0x3086, 0x8000}, // SEQ_DATA_PORT
	{ 0x3086, 0x307E}, // SEQ_DATA_PORT
	{ 0x3086, 0xFF80}, // SEQ_DATA_PORT
	{ 0x3086, 0x20C3}, // SEQ_DATA_PORT
	{ 0x3086, 0xB00E}, // SEQ_DATA_PORT
	{ 0x3086, 0x8190}, // SEQ_DATA_PORT
	{ 0x3086, 0x1643}, // SEQ_DATA_PORT
	{ 0x3086, 0x1651}, // SEQ_DATA_PORT
	{ 0x3086, 0x9D3E}, // SEQ_DATA_PORT
	{ 0x3086, 0x9545}, // SEQ_DATA_PORT
	{ 0x3086, 0x2209}, // SEQ_DATA_PORT
	{ 0x3086, 0x3781}, // SEQ_DATA_PORT
	{ 0x3086, 0x9016}, // SEQ_DATA_PORT
	{ 0x3086, 0x4316}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F90}, // SEQ_DATA_PORT
	{ 0x3086, 0x8000}, // SEQ_DATA_PORT
	{ 0x3086, 0x387F}, // SEQ_DATA_PORT
	{ 0x3086, 0x1380}, // SEQ_DATA_PORT
	{ 0x3086, 0x233B}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F93}, // SEQ_DATA_PORT
	{ 0x3086, 0x4502}, // SEQ_DATA_PORT
	{ 0x3086, 0x8000}, // SEQ_DATA_PORT
	{ 0x3086, 0x7FB0}, // SEQ_DATA_PORT
	{ 0x3086, 0x8D66}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F90}, // SEQ_DATA_PORT
	{ 0x3086, 0x8192}, // SEQ_DATA_PORT
	{ 0x3086, 0x3C16}, // SEQ_DATA_PORT
	{ 0x3086, 0x357F}, // SEQ_DATA_PORT
	{ 0x3086, 0x9345}, // SEQ_DATA_PORT
	{ 0x3086, 0x0280}, // SEQ_DATA_PORT
	{ 0x3086, 0x007F}, // SEQ_DATA_PORT
	{ 0x3086, 0xB08D}, // SEQ_DATA_PORT
	{ 0x3086, 0x667F}, // SEQ_DATA_PORT
	{ 0x3086, 0x9081}, // SEQ_DATA_PORT
	{ 0x3086, 0x8237}, // SEQ_DATA_PORT
	{ 0x3086, 0x4502}, // SEQ_DATA_PORT
	{ 0x3086, 0x3681}, // SEQ_DATA_PORT
	{ 0x3086, 0x8044}, // SEQ_DATA_PORT
	{ 0x3086, 0x1631}, // SEQ_DATA_PORT
	{ 0x3086, 0x4374}, // SEQ_DATA_PORT
	{ 0x3086, 0x1678}, // SEQ_DATA_PORT
	{ 0x3086, 0x7B7D}, // SEQ_DATA_PORT
	{ 0x3086, 0x4502}, // SEQ_DATA_PORT
	{ 0x3086, 0x450A}, // SEQ_DATA_PORT
	{ 0x3086, 0x7E12}, // SEQ_DATA_PORT
	{ 0x3086, 0x8180}, // SEQ_DATA_PORT
	{ 0x3086, 0x377F}, // SEQ_DATA_PORT
	{ 0x3086, 0x1045}, // SEQ_DATA_PORT
	{ 0x3086, 0x0A0E}, // SEQ_DATA_PORT
	{ 0x3086, 0x7FD4}, // SEQ_DATA_PORT
	{ 0x3086, 0x8024}, // SEQ_DATA_PORT
	{ 0x3086, 0x0E82}, // SEQ_DATA_PORT
	{ 0x3086, 0x9CC2}, // SEQ_DATA_PORT
	{ 0x3086, 0xAFA8}, // SEQ_DATA_PORT
	{ 0x3086, 0xAA03}, // SEQ_DATA_PORT
	{ 0x3086, 0x430D}, // SEQ_DATA_PORT
	{ 0x3086, 0x2D46}, // SEQ_DATA_PORT
	{ 0x3086, 0x4316}, // SEQ_DATA_PORT
	{ 0x3086, 0x5F16}, // SEQ_DATA_PORT
	{ 0x3086, 0x530D}, // SEQ_DATA_PORT
	{ 0x3086, 0x1660}, // SEQ_DATA_PORT
	{ 0x3086, 0x401E}, // SEQ_DATA_PORT
	{ 0x3086, 0x2904}, // SEQ_DATA_PORT
	{ 0x3086, 0x2984}, // SEQ_DATA_PORT
	{ 0x3086, 0x81E7}, // SEQ_DATA_PORT
	{ 0x3086, 0x816F}, // SEQ_DATA_PORT
	{ 0x3086, 0x1706}, // SEQ_DATA_PORT
	{ 0x3086, 0x81E7}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F81}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C0D}, // SEQ_DATA_PORT
	{ 0x3086, 0x5754}, // SEQ_DATA_PORT
	{ 0x3086, 0x495F}, // SEQ_DATA_PORT
	{ 0x3086, 0x5305}, // SEQ_DATA_PORT
	{ 0x3086, 0x5307}, // SEQ_DATA_PORT
	{ 0x3086, 0x4D2B}, // SEQ_DATA_PORT
	{ 0x3086, 0xF810}, // SEQ_DATA_PORT
	{ 0x3086, 0x164C}, // SEQ_DATA_PORT
	{ 0x3086, 0x0755}, // SEQ_DATA_PORT
	{ 0x3086, 0x562B}, // SEQ_DATA_PORT
	{ 0x3086, 0xB82B}, // SEQ_DATA_PORT
	{ 0x3086, 0x984E}, // SEQ_DATA_PORT
	{ 0x3086, 0x1129}, // SEQ_DATA_PORT
	{ 0x3086, 0x9460}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C09}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C1B}, // SEQ_DATA_PORT
	{ 0x3086, 0x4002}, // SEQ_DATA_PORT
	{ 0x3086, 0x4500}, // SEQ_DATA_PORT
	{ 0x3086, 0x4580}, // SEQ_DATA_PORT
	{ 0x3086, 0x29B6}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F80}, // SEQ_DATA_PORT
	{ 0x3086, 0x4004}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F88}, // SEQ_DATA_PORT
	{ 0x3086, 0x4109}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C0B}, // SEQ_DATA_PORT
	{ 0x3086, 0x29B2}, // SEQ_DATA_PORT
	{ 0x3086, 0x4115}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C03}, // SEQ_DATA_PORT
	{ 0x3086, 0x4105}, // SEQ_DATA_PORT
	{ 0x3086, 0x5F2B}, // SEQ_DATA_PORT
	{ 0x3086, 0x902B}, // SEQ_DATA_PORT
	{ 0x3086, 0x8081}, // SEQ_DATA_PORT
	{ 0x3086, 0x6F40}, // SEQ_DATA_PORT
	{ 0x3086, 0x1041}, // SEQ_DATA_PORT
	{ 0x3086, 0x0160}, // SEQ_DATA_PORT
	{ 0x3086, 0x29A2}, // SEQ_DATA_PORT
	{ 0x3086, 0x29A3}, // SEQ_DATA_PORT
	{ 0x3086, 0x5F4D}, // SEQ_DATA_PORT
	{ 0x3086, 0x1C17}, // SEQ_DATA_PORT
	{ 0x3086, 0x0281}, // SEQ_DATA_PORT
	{ 0x3086, 0xE729}, // SEQ_DATA_PORT
	{ 0x3086, 0x8345}, // SEQ_DATA_PORT
	{ 0x3086, 0x8840}, // SEQ_DATA_PORT
	{ 0x3086, 0x0F7F}, // SEQ_DATA_PORT
	{ 0x3086, 0x8A40}, // SEQ_DATA_PORT
	{ 0x3086, 0x2345}, // SEQ_DATA_PORT
	{ 0x3086, 0x8024}, // SEQ_DATA_PORT
	{ 0x3086, 0x4008}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F88}, // SEQ_DATA_PORT
	{ 0x3086, 0x5D29}, // SEQ_DATA_PORT
	{ 0x3086, 0x9288}, // SEQ_DATA_PORT
	{ 0x3086, 0x102B}, // SEQ_DATA_PORT
	{ 0x3086, 0x0489}, // SEQ_DATA_PORT
	{ 0x3086, 0x165C}, // SEQ_DATA_PORT
	{ 0x3086, 0x4386}, // SEQ_DATA_PORT
	{ 0x3086, 0x170B}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C03}, // SEQ_DATA_PORT
	{ 0x3086, 0x8A48}, // SEQ_DATA_PORT
	{ 0x3086, 0x4D4E}, // SEQ_DATA_PORT
	{ 0x3086, 0x2B80}, // SEQ_DATA_PORT
	{ 0x3086, 0x4C09}, // SEQ_DATA_PORT
	{ 0x3086, 0x4119}, // SEQ_DATA_PORT
	{ 0x3086, 0x816F}, // SEQ_DATA_PORT
	{ 0x3086, 0x4110}, // SEQ_DATA_PORT
	{ 0x3086, 0x4001}, // SEQ_DATA_PORT
	{ 0x3086, 0x6029}, // SEQ_DATA_PORT
	{ 0x3086, 0x8229}, // SEQ_DATA_PORT
	{ 0x3086, 0x8329}, // SEQ_DATA_PORT
	{ 0x3086, 0x435C}, // SEQ_DATA_PORT
	{ 0x3086, 0x055F}, // SEQ_DATA_PORT
	{ 0x3086, 0x4D1C}, // SEQ_DATA_PORT
	{ 0x3086, 0x81E7}, // SEQ_DATA_PORT
	{ 0x3086, 0x4502}, // SEQ_DATA_PORT
	{ 0x3086, 0x8180}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F80}, // SEQ_DATA_PORT
	{ 0x3086, 0x410A}, // SEQ_DATA_PORT
	{ 0x3086, 0x9144}, // SEQ_DATA_PORT
	{ 0x3086, 0x1609}, // SEQ_DATA_PORT
	{ 0x3086, 0x2FC3}, // SEQ_DATA_PORT
	{ 0x3086, 0xB130}, // SEQ_DATA_PORT
	{ 0x3086, 0xC3B1}, // SEQ_DATA_PORT
	{ 0x3086, 0x0343}, // SEQ_DATA_PORT
	{ 0x3086, 0x164A}, // SEQ_DATA_PORT
	{ 0x3086, 0x0A43}, // SEQ_DATA_PORT
	{ 0x3086, 0x160B}, // SEQ_DATA_PORT
	{ 0x3086, 0x4316}, // SEQ_DATA_PORT
	{ 0x3086, 0x8F43}, // SEQ_DATA_PORT
	{ 0x3086, 0x1690}, // SEQ_DATA_PORT
	{ 0x3086, 0x4316}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F81}, // SEQ_DATA_PORT
	{ 0x3086, 0x450A}, // SEQ_DATA_PORT
	{ 0x3086, 0x410F}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F83}, // SEQ_DATA_PORT
	{ 0x3086, 0x5D29}, // SEQ_DATA_PORT
	{ 0x3086, 0x4488}, // SEQ_DATA_PORT
	{ 0x3086, 0x102B}, // SEQ_DATA_PORT
	{ 0x3086, 0x0453}, // SEQ_DATA_PORT
	{ 0x3086, 0x0D40}, // SEQ_DATA_PORT
	{ 0x3086, 0x2345}, // SEQ_DATA_PORT
	{ 0x3086, 0x0240}, // SEQ_DATA_PORT
	{ 0x3086, 0x087F}, // SEQ_DATA_PORT
	{ 0x3086, 0x8053}, // SEQ_DATA_PORT
	{ 0x3086, 0x0D89}, // SEQ_DATA_PORT
	{ 0x3086, 0x165C}, // SEQ_DATA_PORT
	{ 0x3086, 0x4586}, // SEQ_DATA_PORT
	{ 0x3086, 0x170B}, // SEQ_DATA_PORT
	{ 0x3086, 0x5C05}, // SEQ_DATA_PORT
	{ 0x3086, 0x8A60}, // SEQ_DATA_PORT
	{ 0x3086, 0x4B91}, // SEQ_DATA_PORT
	{ 0x3086, 0x4416}, // SEQ_DATA_PORT
	{ 0x3086, 0x09C1}, // SEQ_DATA_PORT
	{ 0x3086, 0x2CA9}, // SEQ_DATA_PORT
	{ 0x3086, 0xAB30}, // SEQ_DATA_PORT
	{ 0x3086, 0x51B3}, // SEQ_DATA_PORT
	{ 0x3086, 0x3D5A}, // SEQ_DATA_PORT
	{ 0x3086, 0x7E3D}, // SEQ_DATA_PORT
	{ 0x3086, 0x7E19}, // SEQ_DATA_PORT
	{ 0x3086, 0x8000}, // SEQ_DATA_PORT
	{ 0x3086, 0x8B1F}, // SEQ_DATA_PORT
	{ 0x3086, 0x2A1F}, // SEQ_DATA_PORT
	{ 0x3086, 0x83A2}, // SEQ_DATA_PORT
	{ 0x3086, 0x7516}, // SEQ_DATA_PORT
	{ 0x3086, 0xAD33}, // SEQ_DATA_PORT
	{ 0x3086, 0x450A}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F53}, // SEQ_DATA_PORT
	{ 0x3086, 0x8023}, // SEQ_DATA_PORT
	{ 0x3086, 0x8C66}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F13}, // SEQ_DATA_PORT
	{ 0x3086, 0x8184}, // SEQ_DATA_PORT
	{ 0x3086, 0x1481}, // SEQ_DATA_PORT
	{ 0x3086, 0x8031}, // SEQ_DATA_PORT
	{ 0x3086, 0x3D64}, // SEQ_DATA_PORT
	{ 0x3086, 0x452A}, // SEQ_DATA_PORT
	{ 0x3086, 0x9451}, // SEQ_DATA_PORT
	{ 0x3086, 0x9E96}, // SEQ_DATA_PORT
	{ 0x3086, 0x3D2B}, // SEQ_DATA_PORT
	{ 0x3086, 0x3D1B}, // SEQ_DATA_PORT
	{ 0x3086, 0x529F}, // SEQ_DATA_PORT
	{ 0x3086, 0x0E3D}, // SEQ_DATA_PORT
	{ 0x3086, 0x083D}, // SEQ_DATA_PORT
	{ 0x3086, 0x167E}, // SEQ_DATA_PORT
	{ 0x3086, 0x307E}, // SEQ_DATA_PORT
	{ 0x3086, 0x1175}, // SEQ_DATA_PORT
	{ 0x3086, 0x163E}, // SEQ_DATA_PORT
	{ 0x3086, 0x970E}, // SEQ_DATA_PORT
	{ 0x3086, 0x82B2}, // SEQ_DATA_PORT
	{ 0x3086, 0x3D7F}, // SEQ_DATA_PORT
	{ 0x3086, 0xAC3E}, // SEQ_DATA_PORT
	{ 0x3086, 0x4502}, // SEQ_DATA_PORT
	{ 0x3086, 0x7E11}, // SEQ_DATA_PORT
	{ 0x3086, 0x7FD0}, // SEQ_DATA_PORT
	{ 0x3086, 0x8000}, // SEQ_DATA_PORT
	{ 0x3086, 0x8C66}, // SEQ_DATA_PORT
	{ 0x3086, 0x7F90}, // SEQ_DATA_PORT
	{ 0x3086, 0x8194}, // SEQ_DATA_PORT
	{ 0x3086, 0x3F44}, // SEQ_DATA_PORT
	{ 0x3086, 0x1681}, // SEQ_DATA_PORT
	{ 0x3086, 0x8416}, // SEQ_DATA_PORT
	{ 0x3086, 0x2C2C}, // SEQ_DATA_PORT
	{ 0x3086, 0x2C2C}, // SEQ_DATA_PORT
	//	[PLL_settings]
	{ 0x302A, 0x0005}, 	//VT_PIX_CLK_DIV = 5
	{ 0x302C, 0x0001}, 	//VT_SYS_CLK_DIV = 1
	{ 0x302E, 0x0007}, 	//PRE_PLL_CLK_DIV = 7
	{ 0x3030, 0x0082}, 	//PLL_MULTIPLIER = 130
	{ 0x3036, 0x000A}, 	//OP_PIX_CLK_DIV = 10
	{ 0x3038, 0x0001}, 	//OP_SYS_CLK_DIV = 1
	//	BITFIELD = 0x30B0, 0x4000, 0x0000	//DIGITAL_TEST, bits 0x4000 = 0
	{ 0x31B0, 0x0082}, 	//FRAME_PREAMBLE = 130
	{ 0x31B2, 0x005C}, 	//LINE_PREAMBLE = 92
	{ 0x31B4, 0x4248}, 	//MIPI_TIMING_0 = 16968
	{ 0x31B6, 0x3258}, 	//MIPI_TIMING_1 = 12888
	{ 0x31B8, 0x804B}, 	//MIPI_TIMING_2 = 32843
	{ 0x31BA, 0x030B}, 	//MIPI_TIMING_3 = 779
	{ 0x31BC, 0x8E09}, 	//MIPI_TIMING_4 = 36361
	{ 0x3354, 0x002B}, 	//MIPI_CNTRL = 43


	//	[Timing_settings]
	{ 0x301A, 0x2058}, 	//RESET_REGISTER = 8280
	{ 0x31AE, 0x0202}, 	//SERIAL_FORMAT = 514
	{ 0x3002, 0x0044}, 	//Y_ADDR_START = 68
	{ 0x3004, 0x0008}, 	//X_ADDR_START = 8
	{ 0x3006, 0x047B}, 	//Y_ADDR_END = 1147
	{ 0x3008, 0x0787}, 	//X_ADDR_END = 1927
	{ 0x300A, 0x04B8}, 	//FRAME_LENGTH_LINES = 1208
	{ 0x300C, 0x0264}, 	//LINE_LENGTH_PCK = 612
	{ 0x3012, 0x048D}, 	//COARSE_INTEGRATION_TIME = 1165
	{ 0x31AC, 0x0A0A}, 	//DATA_FORMAT_BITS = 2570
	{ 0x3354, 0x002B},        //MIPI DATATYPE 10BITS
	{ 0x306E, 0x9010}, 	//DATAPATH_SELECT = 36880
	{ 0x30A2, 0x0001}, 	//X_ODD_INC = 1
	{ 0x30A6, 0x0001}, 	//Y_ODD_INC = 1
	{ 0x3082, 0x0003}, 	//OPERATION_MODE_CTRL = 3
	{ 0x3040, 0x0000}, 	//READ_MODE = 0
	{ 0x31D0, 0x0000}, 	//COMPANDING = 0
//	{ 0x301A, 0x205C}, 	//RESET_REGISTER = 8284
	{AR0234_REG_END, 0x00},/* END MARKER */
};

/*
 * the part of driver was fixed.
 */

static struct regval_list ar0234_stream_on[] = {
//	{0x301A, 0x19CC},
	{0x301A, 0x205C},
	{AR0234_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list ar0234_stream_off[] = {
//	{0x301A, 0x19C8},
	{0x301A, 0x2058},
	{AR0234_REG_END, 0x00},	/* END MARKER */
};

static int ar0234_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct ar0234_info *info = to_state(sd);
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
			.len	= 2,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int ar0234_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned short value)
{
	struct ar0234_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[4] = {reg >> 8, reg & 0xff, value >> 8, value & 0xff};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 4,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ar0234_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val[2];
	uint16_t value;
	while (vals->reg_num != AR0234_REG_END) {
		if (vals->reg_num == AR0234_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ar0234_read(sd, vals->reg_num, val);
			if (ret < 0)
				return ret;
		}
		vals->value=(val[0]<<8)|val[1];
		vals++;
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ar0234_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != AR0234_REG_END) {
		if (vals->reg_num == AR0234_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ar0234_write(sd, vals->reg_num, vals->value);
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
static int ar0234_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ar0234_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int ar0234_shutdown(struct v4l2_subdev *sd, u32 on)
{
	struct ar0234_info *info = to_state(sd);
	int ret = 0;

	if(on)
		ret = gpio_direction_output(info->shutdown.pin, info->shutdown.active_level);
	else
		ret = gpio_direction_output(info->shutdown.pin, !info->shutdown.active_level);
	return ret;
}

static int ar0234_init(struct v4l2_subdev *sd, u32 val)
{
	struct ar0234_info *info = to_state(sd);
	int ret = 0;

	ret = ar0234_write_array(sd, info->win->regs);

	return ret;
}



static int ar0234_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v[2] = {0};
	int ret;
	ret = ar0234_read(sd, 0x3000, v);

	if (ret < 0)
		return ret;

	printk("%x %x \n", v[0], v[1]);
	if (v[0] != AR0234_CHIP_ID_H)
		return -ENODEV;

	if (v[1] != AR0234_CHIP_ID_L)
		return -ENODEV;

	return 0;
}


static struct ar0234_win_size ar0234_win_sizes[] = {
	{
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 60 << 16 | 1,

		.width		= 1920,
		.height		= 1080,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ar0234_init_regs_1080p_60fps_mipi,
	},
};

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_AR0234_FMTS)
		return -EINVAL;

	code->code = ar0234_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int ar0234_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ar0234_format_struct *ovfmt;
	struct ar0234_win_size *wsize;
	struct ar0234_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int ar0234_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct ar0234_info *info = to_state(sd);
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

static int ar0234_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0234_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0234_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0234_s_vflip(struct v4l2_subdev *sd, int value)
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
static int ar0234_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ar0234_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ar0234_again_lut); i++) {
		lut = &ar0234_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(ar0234_again_lut); i++) {
		lut = &ar0234_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return 0;
}

static int ar0234_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct ar0234_info *info = to_state(sd);
	unsigned int reg_val = 0;
	int ret = 0;


	ret = ar0234_read(sd, 0x3509, &reg_val);

	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int ar0234_s_again(struct v4l2_subdev *sd, int value)
{
	struct ar0234_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	int i;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += ar0234_write(sd, 0x3060, reg_value);
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Tweak autogain.
 */
static int ar0234_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0234_s_exp(struct v4l2_subdev *sd, int value)
{
	struct ar0234_info *info = to_state(sd);
	int ret = 0;

	ret = ar0234_write(sd, 0x3012, value);

	return ret;
}

static int ar0234_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ar0234_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ar0234_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ar0234_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ar0234_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ar0234_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ar0234_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ar0234_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ar0234_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ar0234_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ar0234_s_gain turns off auto gain */
			return ar0234_s_gain(sd, info->gain->val);
		}
		return ar0234_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ar0234_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ar0234_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ar0234_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.s_ctrl = ar0234_s_ctrl,
	.g_volatile_ctrl = ar0234_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0234_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ar0234_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ar0234_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ar0234_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif


int ar0234_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0234_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	info->supply.pwen  = devm_regulator_get_optional(&client->dev, "pwen");
	info->supply.shutdown  = devm_regulator_get_optional(&client->dev, "shutdown");

	if (IS_ERR(info->supply.pwen)||IS_ERR(info->supply.shutdown)) {
		if ((PTR_ERR(info->supply.pwen) == -EPROBE_DEFER)||(PTR_ERR(info->supply.shutdown) == -EPROBE_DEFER))
			return -EPROBE_DEFER;
		printk("No ar0234 vdd regulator found\n");
	}

	if ((!IS_ERR(info->supply.pwen))&&(!IS_ERR(info->supply.shutdown))){
		if(on){
			ret = regulator_enable(info->supply.pwen);
			ret = regulator_enable(info->supply.shutdown);
			if (ret)
			   dev_err(&client->dev, "ar0234 vdd supply disable failed\n");
		}else{
			ret = regulator_disable(info->supply.pwen);
			ret = regulator_disable(info->supply.shutdown);
			if (ret)
			    dev_err(&client->dev, "ar0234 vdd supply disable failed\n");
		}
	}else{
		dev_err(&client->dev, "ar0234 vdd supply IS_ERR failed\n");
	}
	return ret;
}

int ar0234_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0234_info *info = to_state(sd);
	int ret = 0;
	struct regval_list *vals;

	if (enable) {
		ret = ar0234_write_array(sd, ar0234_stream_on);
		pr_debug("ar0234 stream on\n");
	}
	else {
		ret = ar0234_write_array(sd, ar0234_stream_off);
		pr_debug("ar0234 stream off\n");
	}
	return ret;
}


int ar0234_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ar0234_info *info = to_state(sd);
	if(info->win->fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ar0234_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ar0234_g_register,
	.s_register = ar0234_s_register,
#endif

};

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = ar0234_s_stream,
	.g_frame_interval = ar0234_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	//.enum_frame_interval = ar0234_enum_frame_interval,
	//.num_frame_size = ar0234_enum_frame_size,
	//.enum_mbus_code = ar0234_enum_mbus_code,
	.set_fmt = ar0234_set_fmt,
	.get_fmt = ar0234_get_fmt,
};

static const struct v4l2_subdev_ops ar0234_ops = {
	.core = &ar0234_core_ops,
	.video = &ar0234_video_ops,
	.pad = &ar0234_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ar0234_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ar0234_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,expo-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->expo.pin = gpio;
		info->expo.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,oe-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->oe.pin = gpio;
		info->oe.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwen-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwen.pin = gpio;
		info->pwen.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,shutdown-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->shutdown.pin = gpio;
		info->shutdown.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}


	v4l2_i2c_subdev_init(sd, client, &ar0234_ops);
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

//	ar0234_shutdown(sd, 0);
	ar0234_s_power(sd, 1);
	ar0234_reset(sd, 1);
#if 1
	/* Make sure it's an ar0234 */
	ret = ar0234_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an ar0234 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 1, 262144, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ar0234_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &ar0234_win_sizes[0];
	ar0234_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "ar0234 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	ar0234_s_power(sd, 0);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id ar0234_id[] = {
	{ "ar0234", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0234_id);

static const struct of_device_id ar0234_of_match[] = {
	{.compatible = "onsemi,ar0234", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);

static int ar0234_suspend(struct device *dev)
{
        struct i2c_client *client = container_of(dev, struct i2c_client, dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct ar0234_info *info = to_state(sd);
	int ret=0;

	ar0234_s_power(sd,0);
	v4l2_clk_disable(info->clk);
        return 0;
}

static int ar0234_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234_info *info = to_state(sd);
	int ret=0;

	ar0234_s_power(sd,1);
	v4l2_clk_enable(info->clk);
	ar0234_reset(sd, 1);
	ar0234_init(sd, 1);
	return 0;
}

const struct dev_pm_ops ar0234_pm =
{
        .suspend = ar0234_suspend,
        .resume  = ar0234_resume,
};


static struct i2c_driver ar0234_driver = {
	.driver = {
		.name	= "ar0234",
		.of_match_table = of_match_ptr(ar0234_of_match),
		.pm	=  &ar0234_pm,
	},
	.probe		= ar0234_probe,
	.remove		= ar0234_remove,
	.id_table	= ar0234_id,
};

module_i2c_driver(ar0234_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for onsemi ar0234 sensors");
MODULE_LICENSE("GPL");
