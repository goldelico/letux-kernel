/*
 * drivers/media/video/ov9655.c
 *
 * Based on mt9v113 decoder driver
 *
 * by H. N. Schaller <hns@computer.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/io.h>

#include <media/v4l2-int-device.h>
#include <media/ov9655.h>

#include "ov9655_regs.h"

/* Module Name */
#define OV9655_MODULE_NAME		"ov9655"

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)

/* Debug functions */
static int debug = 1;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static int ov9655_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int ov9655_write(struct i2c_client *client, const u8 reg,
						const u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int ov9655_set(struct i2c_client *client, const u8 reg,
					  const u8 value, const u8 mask)
{
	int ret;
	
	ret = ov9655_read(client, reg);
	if (ret < 0)
		return ret;
	return ov9655_write(client, reg, (ret & ~mask) | (value & mask));
}

static int ov9655_write_regs(struct i2c_client *client,
							 const struct ov9655_reg *regs, const int n)
{
	int i, ret;
	
	for (i = 0; i < n; i++) {
		printk("%02x := %02x\n", regs->addr, regs->value);
		ret = ov9655_write(client, regs->addr, regs->value);
		if (ret < 0)
			return ret;
		regs++;
	}
	
	return 0;
}

/*
 * enum ov9655_state - enum for different decoder states
 */
enum ov9655_state {
	STATE_NOT_DETECTED,
	STATE_DETECTED
};

/*
 * enum ov9655_std - enum for supported standards
 */
enum ov9655_std {
	OV9655_STD_VGA = 0,
	OV9655_STD_QVGA,
	OV9655_STD_SXGA,
	OV9655_STD_CIF,
	OV9655_STD_INVALID
};

/*
 * struct ov9655_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct ov9655_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

/*
 * struct ov9655_decoded - decoder object
 * @v4l2_int_device: Slave handle
 * @pdata: Board specific
 * @client: I2C client data
 * @id: Entry from I2C table
 * @ver: Chip version
 * @state: decoder state - detected or not-detected
 * @pix: Current pixel format
 * @num_fmts: Number of formats
 * @fmt_list: Format list
 * @current_std: Current standard
 * @num_stds: Number of standards
 * @std_list: Standards list
 */
struct ov9655_decoder {
	struct v4l2_int_device *v4l2_int_device;
	const struct ov9655_platform_data *pdata;
	struct i2c_client *client;

	struct i2c_device_id *id;

	int ver;
	enum ov9655_state state;

	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;

	enum ov9655_std current_std;
	int num_stds;
	struct ov9655_std_info *std_list;
};


/* Number of pixels and number of lines per frame for different standards */
#define VGA_NUM_ACTIVE_PIXELS		(4*160)	/* 4:3 */
#define VGA_NUM_ACTIVE_LINES		(3*160)
#define QVGA_NUM_ACTIVE_PIXELS		(VGA_NUM_ACTIVE_PIXELS/2)	/* 4:3 */
#define QVGA_NUM_ACTIVE_LINES		(VGA_NUM_ACTIVE_LINES/2)
#define SXGA_NUM_ACTIVE_PIXELS		(5*256)	/* 5:4 */
#define SXGA_NUM_ACTIVE_LINES		(4*256)
#define CIF_NUM_ACTIVE_PIXELS		(11*32)	/* 11:9 ~ 5:4 */
#define CIF_NUM_ACTIVE_LINES		(9*32)

/*
 * some assumptions for the interface settings
 * 1. we are connected to a TI OMAP3530 CAM interface at 1.8V through short wires
 * 2. XCLKA up to 48 MHz so that we can reach 15 fps @ SXGA
 * 3. the interface expects VSYNC and HSYNC at positive impulses (not HREF)
 * 4. PCLK is XCLK scaled up by PLL (DBLV=1x,4x,6x,8x) and divided down by CLKRC
 */

static const struct ov9655_reg ov9655_init_hardware[] = {
	/* here we write only registers that must match
	   our hardware interface */
//	{ OV9655_COM7, 0x82, },	/* reset to chip defaults - don't do that since we should wait 1ms */
	{ OV9655_COM2, 0x01 },	/* drive outputs at 2x; disable soft sleep */
//	{ OV9655_COM10, OV9655_COM10_HREF2HSYNC },	/* define pin polarity and functions as default (VSYNC, HSYNC as positive pulses) */
	{ OV9655_COM10, OV9655_COM10_HREF2HSYNC | OV9655_COM10_HSYNC_NEG },	/* define pin polarity and functions as default (VSYNC, HSYNC as positive pulses) */
	{ OV9655_CLKRC, 1 },	/* compensate for PLL_4X */
	// check if this works with 48 MHz - and what PLL_8X means
	{ OV9655_DBLV, OV9655_DBLV_PLL_4X | OV9655_DBLV_BANDGAP },
	{ OV9655_TSLB, 0x0c },	// pixel clock delay and UYVY byte order
};

static const struct ov9655_reg ov9655_init_regs[] = {
	/* here we do some common settings (bias, gain, agc etc.) */
	{ OV9655_COM3, 0xc0 },	// color bar for testing
//	{ OV9655_COM1, 0x03 },	// AEC low bits
//	{ OV9655_COM5, 0x61 },	// slam mode & exposure
	{ OV9655_COM6, 0x40 },	/* manually update window size and timing */
//	{ OV9655_COM7, 0x02 },	// default
//	{ OV9655_COM8, 0xe0 | OV9655_COM8_AGC | OV9655_COM8_AWB | OV9655_COM8_AEC },
//	{ OV9655_COM9, 0x2a },	// agc
//	{ OV9655_REG16, 0x24 },
//	{ OV9655_MVFP, 0 },	// mirror&flip - depends on GTA04 board
//	{ OV9655_AEW, 0x3c },
//	{ OV9655_AEB, 0x36 },
//	{ OV9655_VPT, 0x72 },
//	{ OV9655_BBIAS, 0x08 },
//	{ OV9655_GBBIAS, 0x08 },
//	{ OV9655_PREGAIN, 0x15 },
//	{ OV9655_EXHCH, 0x00 },
//	{ OV9655_EXHCL, 0x00 },
//	{ OV9655_RBIAS, 0x08 },
//	{ OV9655_CHLF, 0x00 },
//	{ OV9655_AREF2, 0x00 },
//	{ OV9655_ADC2, 0x72 },
//	{ OV9655_AREF4, 0x57 },
	{ OV9655_COM11, 0x05 }, // no night mode
//	{ OV9655_COM13, 0x99 },
//	{ OV9655_EDGE, 0x02 },	// edge enhancement factor
//	{ OV9655_COM15, 0xc0 },	// full scale output range
//	{ OV9655_COM17, 0xc1 },	// denoise, edge enhancement, 50 Hz banding filter
	
//	{ OV9655_DNSTH, 0x21 },	// denoise threshold

	/* gamma correction
	{ OV9655_SLOP, 0x12 },
	{ OV9655_GAM1, 0x08 },
	{ OV9655_GAM2, 0x16 },
	{ OV9655_GAM3, 0x30 },
	{ OV9655_GAM4, 0x5e },
	{ OV9655_GAM5, 0x72 },
	{ OV9655_GAM6, 0x82 },
	{ OV9655_GAM7, 0x8e },
	{ OV9655_GAM8, 0x9a },
	{ OV9655_GAM9, 0xa4 },
	{ OV9655_GAM10, 0xac },
	{ OV9655_GAM11, 0xb8 },
	{ OV9655_GAM12, 0xc3 },
	{ OV9655_GAM13, 0xd6 },
	{ OV9655_GAM14, 0xe6 },
	{ OV9655_GAM15, 0xf2 },

	 { OV9655_AWBOP1, 0x85 },
	 { OV9655_AWBOP2, 0xa9 },
	 { OV9655_AWBOP3, 0x64 },
	 { OV9655_AWBOP4, 0x84 },
	 { OV9655_AWBOP5, 0x53 },
	 { OV9655_AWBOP6, 0x0e },
	 { OV9655_BLMT, 0xf0 },
	 { OV9655_RLMT, 0xf0 },
	 { OV9655_GLMT, 0xf0 },
	 { OV9655_LCC1, 0x00 },
	 { OV9655_LCC2, 0x00 },
	 { OV9655_LCC3, 0x02 },
	 { OV9655_LCC6, 0x03 },

	 { OV9655_AECH, 0x40 },

	 */
	
	{ OV9655_COM19, 0x0c },	// UV
//	{ OV9655_COM21, 0x50 },	// digital gain

	/* other values that can be changed
	{ OV9655_GREEN, 0x4a },
	{ OV9655_REFA8, 0xc1 },
	{ OV9655_REFA9, 0xef },
	{ OV9655_BLC1, 0x80 },
	{ OV9655_BLC2, 0x80 },
	{ OV9655_BLC3, 0x80 },
	{ OV9655_BLC4, 0x80 },
	{ OV9655_BLC7, 0xf2 },
	{ OV9655_BLC8, 0x20 },
	{ OV9655_CTRLB4, 0x20 },
	{ OV9655_ADBOFF, 0x7f },
	{ OV9655_ADROFF, 0x7f },
	{ OV9655_ADGBOFF, 0x7f },
	{ OV9655_ADGROFF, 0x7f },
	{ OV9655_AREF1, 0x3d },
	{ OV9655_AREF3, 0x34 },
	{ OV9655_LCC4, 0x16 },
	{ OV9655_LCC5, 0x01 },
	{ OV9655_COM20, 0x03 },	// color bar test mode
	{ OV9655_LCC7, 0x04 },
	{ OV9655_BD50MAX, 0x05 },
	{ OV9655_BD50, 0x9d },
	{ OV9655_BD60, 0x83 },
	{ OV9655_BD60MAX, 0x07 },
	 */
};

/* Register values for SXGA format */
static const struct ov9655_reg ov9655_sxga[] = {
	/* COM7 is set through code since it is shared with color encoding format */
//	{ OV9655_COM7, OV9655_COM7_SXGA | OV9655_COM7_YUV }, 
	// fixme: define macros that split a 11 bit row/col position into higher bytes and HREF/VREF
	{ OV9655_HSTART, 0x1d },
	{ OV9655_HSTOP, 0xbd },
	{ OV9655_HREF, 0xff },
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x81 },
	{ OV9655_VREF, 0x1b },	// vertical frame control
	{ OV9655_COM14, 0x0c },	// zoom
	{ OV9655_COM16, 0x00 },	// scaling
	{ OV9655_POIDX, 0x00 },	// skip lines
	{ OV9655_PCKDV, 0x00 },	// pixel clock divisor (48 MHz)
	{ OV9655_XINDX, 0x3a },	// scale down
	{ OV9655_YINDX, 0x35 },	// scale down
	{ OV9655_COM24, 0x80 },	// pixel clock frequency	
};

/* Register values for VGA format */
static const struct ov9655_reg ov9655_vga[] = {
//	{ OV9655_COM7, OV9655_COM7_VGA | OV9655_COM7_YUV },
	{ OV9655_HSTART, 0x16 },
	{ OV9655_HSTOP, 0x02 },
	{ OV9655_HREF, 0xff },
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x3d },
	{ OV9655_VREF, 0x12 },	// vertical frame control - mixed with AGC
	{ OV9655_COM14, 0x0c },	// pixel correction and zoom
	{ OV9655_COM16, 0x00 },	// no scaling
	{ OV9655_POIDX, 0x00 },	// normal output
	{ OV9655_PCKDV, 0x00 },	// pixel clock divisor (48 MHz / (2^(1)) -> 24 MHz)
	{ OV9655_XINDX, 0x3a },	// scale down
	{ OV9655_YINDX, 0x35 },	// scale down
	{ OV9655_COM24, 0x80 },	// pixel clock frequency
};

/* Register values for QVGA format */
static const struct ov9655_reg ov9655_qvga[] = {
//	{ OV9655_COM7, OV9655_COM7_VGA | OV9655_COM7_YUV },
	{ OV9655_HSTART, 0x18 },
	{ OV9655_HSTOP, 0x04 },
	{ OV9655_HREF, 0x2a },	// lower bits of hstart/stop
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x81 },
	{ OV9655_VREF, 0x02 },	// vertical frame control - mixed with AGC
	{ OV9655_COM14, 0x0e },	// pixel correction and zoom
	{ OV9655_COM16, 0x01 },	// enable scaling
	{ OV9655_POIDX, 0x11 },	// 1 line every 2 px
	{ OV9655_PCKDV, 0x02 },	// pixel clock divisor
	{ OV9655_XINDX, 0x10 },	// scale down
	{ OV9655_YINDX, 0x10 },	// scale down
	{ OV9655_COM24, 0x81 },	// pixel clock frequency
};

/* Register values for CIF format */
static const struct ov9655_reg ov9655_cif[] = {
	/* fixme: this is QQVGA and not CIF */
//	{ OV9655_COM7, OV9655_COM7_VGA | OV9655_COM7_YUV },
	{ OV9655_HSTART, 0x18 },
	{ OV9655_HSTOP, 0x04 },
	{ OV9655_HREF, 0xa4 },
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x81 },
	{ OV9655_VREF, 0x02 },	// vertical frame control - mixed with AGC
	{ OV9655_COM14, 0x0e },	// pixel correction and zoom
	{ OV9655_COM16, 0x01 },	// enable scaling
	{ OV9655_POIDX, 0x22 },	// 1 line every 4 px
	{ OV9655_PCKDV, 0x03 },	// pixel clock divisor
	{ OV9655_XINDX, 0x10 },	// scale down
	{ OV9655_YINDX, 0x10 },	// scale down
	{ OV9655_COM24, 0x82 },	// pixel clock frequency
};

#if UNUSED
/* Register values for YUV format */
static const struct ov9655_reg ov9655_yuv_regs[] = {
	/* merge value with VGA/SXGA setting! */
//	{ OV9655_COM7, OV9655_COM7_YUV },  // select SXGA vs. VGA
	{ OV9655_MTX1, 0x80 },
	{ OV9655_MTX2, 0x80 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x22 },
	{ OV9655_MTX5, 0x5e },
	{ OV9655_MTX6, 0x80 },
	{ OV9655_MTXS, 0x1e },
};

/* Register values for RGB format */
static const struct ov9655_reg ov9655_rgb_regs[] = {
//	{ OV9655_MTX1, 0x98 },
	{ OV9655_MTX2, 0x98 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x28,},
	{ OV9655_MTX5, 0x70 },
	{ OV9655_MTX6, 0x98 },
	{ OV9655_MTXS, 0x1a },
};
#endif

/* List of image formats supported by ov9655
 */
static const struct v4l2_fmtdesc ov9655_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	// checkme: do we really support this?
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
#if 0	// FIXME: we need to control the chip to send these formats...
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "16-bit RGB 5:6:5 Format",
	 .pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "16-bit RGB 5:5:5 Format",
	 .pixelformat = V4L2_PIX_FMT_RGB555/* X */,
	},
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit Y/CbCr 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_NV16,
	},
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "10-bit Bayer10 (GrR/BGb) Format",
	 .pixelformat = V4L2_PIX_FMT_SGRBG10,	/* is this wat the camera provides? */
	},
	// finally a RGB4:2:2 format that appears to be n/a in L4V2
#endif
};

/*
 * Supported standards
 *
 */
/*
 * enum ov9655_std - enum for supported standards
 */

// is some duplicate with enum ov9655_std

#define OV9655_IMAGE_STD_VGA			(0)
#define OV9655_IMAGE_STD_QVGA			(1)
#define OV9655_IMAGE_STD_SXGA			(2)
#define OV9655_IMAGE_STD_CIF			(3)
#define OV9655_IMAGE_STD_INVALID		(-1)

#define FRAME_PERIOD_NTSC {1001, 30000}
#define FRAME_PERIOD_PAL {1, 25}

static struct ov9655_std_info ov9655_std_list[] = {
	[OV9655_STD_SXGA] = {
	 .width = SXGA_NUM_ACTIVE_PIXELS,
	 .height = SXGA_NUM_ACTIVE_LINES,
	 .video_std = OV9655_IMAGE_STD_SXGA,
	 .standard = {
			 .index = 0,
			 .id = OV9655_STD_SXGA,
			 .name = "SXGA",
			 .frameperiod = FRAME_PERIOD_NTSC,
			 .framelines = SXGA_NUM_ACTIVE_LINES
			},
	},
	[OV9655_STD_VGA] = {
	 .width = VGA_NUM_ACTIVE_PIXELS,
	 .height = VGA_NUM_ACTIVE_LINES,
	 .video_std = OV9655_IMAGE_STD_VGA,
	 .standard = {
		      .index = 0,
		      .id = OV9655_STD_VGA,
		      .name = "VGA",
			  .frameperiod = FRAME_PERIOD_NTSC,		 // what does this mean??? see http://v4l2spec.bytesex.org/spec/r9288.htm Table 4
		      .framelines = VGA_NUM_ACTIVE_LINES
		     },
	},
	[OV9655_STD_QVGA] = {
	 .width = QVGA_NUM_ACTIVE_PIXELS,
	 .height = QVGA_NUM_ACTIVE_LINES,
	 .video_std = OV9655_IMAGE_STD_QVGA,
	 .standard = {
		      .index = 1,
		      .id = OV9655_STD_QVGA,
		      .name = "QVGA",
		      .frameperiod = FRAME_PERIOD_NTSC,
		      .framelines = QVGA_NUM_ACTIVE_LINES
		     },
	},
	[OV9655_STD_CIF] = {
		.width = CIF_NUM_ACTIVE_PIXELS,
		.height = CIF_NUM_ACTIVE_LINES,
		.video_std = OV9655_IMAGE_STD_CIF,
		.standard = {
			 .index = 0,
			 .id = OV9655_STD_CIF,
			 .name = "CIF",
			 .frameperiod = FRAME_PERIOD_NTSC,
			 .framelines = CIF_NUM_ACTIVE_LINES
			},
	},
};


/*
 * Control structure for Auto Gain
 *     This is temporary data, will get replaced once
 *     v4l2_ctrl_query_fill supports it.
 */
static const struct v4l2_queryctrl ov9655_autogain_ctrl = {
	.id = V4L2_CID_AUTOGAIN,
	.name = "Gain, Automatic",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.minimum = 0,
	.maximum = 1,
	.step = 1,
	.default_value = 1,
};

const struct v4l2_fract ov9655_frameintervals[] = {
	{  .numerator = 1, .denominator = 10 }
};

/*
 * ov9655_get_current_std:
 * Returns the current standard
 */
static enum ov9655_std ov9655_get_current_std(struct ov9655_decoder
						*decoder)
{
	enum ov9655_std current_std;
	unsigned short com7;

	com7 = ov9655_read(decoder->client, OV9655_COM7);

	if((com7 & OV9655_COM7_VGA) == OV9655_COM7_VGA)
		// FIXME: check for scaling to detect QVGA and CIF
		current_std = OV9655_STD_VGA;
	else
		current_std = OV9655_STD_SXGA;
	printk("current standard: %d (%s)\n", current_std, decoder->std_list[current_std].standard.name);
	return current_std;
}

/*
 * Configure the OV9655 to given resolution
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov9655_set_current_std(struct ov9655_decoder *decoder, enum ov9655_std std)
{
	int err = -EINVAL;
	printk("set standard: %d (%s)\n", std, decoder->std_list[std].standard.name);

	switch(std) {
		case OV9655_IMAGE_STD_SXGA:
			ov9655_set(decoder->client, OV9655_COM7, OV9655_COM7_RES_MASK, OV9655_COM7_SXGA);
			err = ov9655_write_regs(decoder->client, ov9655_sxga, ARRAY_SIZE(ov9655_sxga));
			break;
		case OV9655_IMAGE_STD_VGA:
			ov9655_set(decoder->client, OV9655_COM7, OV9655_COM7_RES_MASK, OV9655_COM7_VGA);
			err = ov9655_write_regs(decoder->client, ov9655_vga, ARRAY_SIZE(ov9655_vga));
			break;
		case OV9655_IMAGE_STD_QVGA:
			ov9655_set(decoder->client, OV9655_COM7, OV9655_COM7_RES_MASK, OV9655_COM7_VGA);
			err = ov9655_write_regs(decoder->client, ov9655_qvga, ARRAY_SIZE(ov9655_qvga));
			break;
		case OV9655_IMAGE_STD_CIF:
			ov9655_set(decoder->client, OV9655_COM7, OV9655_COM7_RES_MASK, OV9655_COM7_VGA);
			err = ov9655_write_regs(decoder->client, ov9655_cif, ARRAY_SIZE(ov9655_cif));
			break;
		default:
			break;
	}
	if (err)
		return err;

	return 0;
}

/*
 * Configure the ov9655 with the current register settings
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov9655_configure(struct ov9655_decoder *decoder)
{
	int err;
	// write 0x80 to register 0x12 and msleep(100)
	/* common register initialization */
	err =
	    ov9655_write_regs(decoder->client, ov9655_init_hardware, ARRAY_SIZE(ov9655_init_hardware));
	if (err)
		return err;
	err =
	ov9655_write_regs(decoder->client, ov9655_init_regs, ARRAY_SIZE(ov9655_init_regs));
	if (err)
		return err;
	/* and configure as default to VGA mode */
	err =ov9655_set_current_std(decoder, OV9655_STD_VGA);
	if (err)
		return err;
	
	return 0;
}

/*
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (frms->pixel_format == decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == decoder->num_fmts)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= decoder->num_stds)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = decoder->std_list[frms->index].width;
	frms->discrete.height = decoder->std_list[frms->index].height;

	return 0;

}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;

	if (frmi->index >= 1)
		return -EINVAL;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (frmi->pixel_format == decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == decoder->num_fmts)
		return -EINVAL;

	if (frmi->index >= ARRAY_SIZE(ov9655_frameintervals))
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
		ov9655_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
		ov9655_frameintervals[frmi->index].denominator;
	return 0;
}


/*
 * Detect if an ov9655 is present, and if so which revision.
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int ov9655_detect(struct ov9655_decoder *decoder)
{
	unsigned short val=0;
	printk("ov9655_detect\n");

	
	/* Read chip manufacturer register */
	val = (ov9655_read(decoder->client, OV9655_MIDH) << 8) + ov9655_read(decoder->client, OV9655_MIDL);

	if (val < 0) {
		v4l_dbg(1, debug, decoder->client, "Strange error reading sensor"
				" manufacturer\n");
		return -ENODEV;
	}
	v4l_dbg(1, debug, decoder->client, "chip manufacturer detected 0x%x\n", val);
	
	if (0x7fa2 != val) {
		v4l_dbg(1, debug, decoder->client, "No OmniVision sensor detected, "
				" manufacturer register read 0x%x\n", val);
		return -ENODEV;
	}
	
	val = ov9655_read(decoder->client, OV9655_PID);
	v4l_dbg(1, debug, decoder->client, "chip pid detected 0x%x\n", val);

	if (OV9655_CHIP_PID != val) {
		/* We didn't read the values we expected, so this must not be
		 * OV9655.
		 */
		v4l_err(decoder->client,
				"chip pid mismatch read 0x%x, expecting 0x%x\n", val,
				OV9655_CHIP_PID);
		return -ENODEV;
	}

	val = ov9655_read(decoder->client, OV9655_VER);
	v4l_dbg(1, debug, decoder->client, "chip ver detected 0x%x\n", val);

	if (OV9655_CHIP_VER4 == val)
		decoder->ver = 4;
	else if (OV9655_CHIP_VER5 == val)
		decoder->ver = 5;
	else 		{
		/* We didn't read the values we expected, so this must not be
		 * OV9655.
		 */
		v4l_err(decoder->client,
			"chip id mismatch read 0x%x, expecting 0x%x or 0x%x\n", val,
				OV9655_CHIP_VER4, OV9655_CHIP_VER5);
		return -ENODEV;
	}

	decoder->state = STATE_DETECTED;

	v4l_info(decoder->client,
			"%s found at 0x%x (%s)\n", decoder->client->name,
			decoder->client->addr << 1,
			decoder->client->adapter->name);

	return 0;
}

/*
 * Following are decoder interface functions implemented by
 * ov9655 decoder driver.
 */

/*
 * ioctl_querystd - V4L2 decoder interface handler for VIDIOC_QUERYSTD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by ov9655. If no active input is
 * detected, returns -EINVAL
 */
static int ioctl_querystd(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct ov9655_decoder *decoder = s->priv;
	enum ov9655_std current_std;

	if (std_id == NULL)
		return -EINVAL;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_IMAGE_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	*std_id = decoder->std_list[current_std].standard.id;

	v4l_dbg(1, debug, decoder->client, "Current STD: %s\n",
			decoder->std_list[current_std].standard.name);
	return 0;
}

/*
 * ioctl_s_std - V4L2 decoder interface handler for VIDIOC_S_STD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 v4l2_std_id ioctl enum
 *
 * If std_id is supported, sets the requested standard. Otherwise, returns
 * -EINVAL
 */
static int ioctl_s_std(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct ov9655_decoder *decoder = s->priv;
	int err, i;

	if (std_id == NULL)
		return -EINVAL;

	for (i = 0; i < decoder->num_stds; i++)
		if (*std_id == decoder->std_list[i].standard.id)
			break;

	if ((i == decoder->num_stds))	/* not found */
		return -EINVAL;
	
	err =ov9655_set_current_std(decoder, i);

	decoder->current_std = i;

	v4l_dbg(1, debug, decoder->client, "Standard set to: %s\n",
			decoder->std_list[i].standard.name);
	return 0;
}

/*
 * ioctl_s_routing - V4L2 decoder interface handler for VIDIOC_S_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: number of the input
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int ioctl_s_routing(struct v4l2_int_device *s,
				struct v4l2_routing *route)
{
	return 0;
}

/*
 * ioctl_queryctrl - V4L2 decoder interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qctrl: standard V4L2 v4l2_queryctrl structure
 *
 * If the requested control is supported, returns the control information.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = -EINVAL;

	if (qctrl == NULL)
		return err;
	switch (qctrl->id) {
#if NOT_YET_FIXED
			// FIXME: adapt to OV9655 controls
	case V4L2_CID_BRIGHTNESS:
		/* Brightness supported is same as standard one (0-255),
		 * so make use of standard API provided.
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
		/* Saturation and Contrast supported is -
		 *	Contrast: 0 - 255 (Default - 128)
		 *	Saturation: 0 - 255 (Default - 128)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_HUE:
		/* Hue Supported is -
		 *	Hue - -180 - +180 (Default - 0, Step - +180)
		 */
		err = v4l2_ctrl_query_fill(qctrl, -180, 180, 180, 0);
		break;
	case V4L2_CID_AUTOGAIN:
		/* Autogain is either 0 or 1*/
		memcpy(qctrl, &ov9655_autogain_ctrl,
				sizeof(struct v4l2_queryctrl));
		err = 0;
		break;
#endif
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", qctrl->id);
		return err;
	}
	v4l_dbg(1, debug, decoder->client,
			"Query Control: %s : Min - %d, Max - %d, Def - %d\n",
			qctrl->name,
			qctrl->minimum,
			qctrl->maximum,
			qctrl->default_value);

	return err;
}

/*
 * ioctl_g_ctrl - V4L2 decoder interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, returns the control's current
 * value from the decoder. Otherwise, returns -EINVAL if the control is not
 * supported.
 */
static int
ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct ov9655_decoder *decoder = s->priv;

	if (ctrl == NULL)
		return -EINVAL;
	switch (ctrl->id) {
#if NOT_YET_FIXED
			// FIXME: adapt to OV9655 controls
	case V4L2_CID_BRIGHTNESS:
			// FIXME: write OV9655_BRTN
		ctrl->value = ov9655_reg_list[REG_BRIGHTNESS].val;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = ov9655_reg_list[REG_CONTRAST].val;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = ov9655_reg_list[REG_SATURATION].val;
		break;
	case V4L2_CID_HUE:
		ctrl->value = ov9655_reg_list[REG_HUE].val;
		if (ctrl->value == 0x7F)
			ctrl->value = 180;
		else if (ctrl->value == 0x80)
			ctrl->value = -180;
		else
			ctrl->value = 0;

		break;
	case V4L2_CID_AUTOGAIN:
		ctrl->value = ov9655_reg_list[REG_AFE_GAIN_CTRL].val;
		if ((ctrl->value & 0x3) == 3)
			ctrl->value = 1;
		else
			ctrl->value = 0;

		break;
#endif
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", ctrl->id);
		return -EINVAL;
	}
	v4l_dbg(1, debug, decoder->client,
			"Get Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);
	return 0;
}

/*
 * ioctl_s_ctrl - V4L2 decoder interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW. Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = -EINVAL, value;

	if (ctrl == NULL)
		return err;

	value = (__s32) ctrl->value;
#if NOT_YET_FIXED
	// FIXME: adapt to OV9655 controls
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid brightness setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_BRIGHTNESS,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_BRIGHTNESS].val = value;
		break;
	case V4L2_CID_CONTRAST:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid contrast setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_CONTRAST,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_CONTRAST].val = value;
		break;
	case V4L2_CID_SATURATION:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid saturation setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_SATURATION,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_SATURATION].val = value;
		break;
	case V4L2_CID_HUE:
		if (value == 180)
			value = 0x7F;
		else if (value == -180)
			value = 0x80;
		else if (value == 0)
			value = 0;
		else {
			v4l_err(decoder->client,
					"invalid hue setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_HUE,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_HUE].val = value;
		break;
	case V4L2_CID_AUTOGAIN:
		if (value == 1)
			value = 0x0F;
		else if (value == 0)
			value = 0x0C;
		else {
			v4l_err(decoder->client,
					"invalid auto gain setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_AFE_GAIN_CTRL,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_AFE_GAIN_CTRL].val = value;
		break;
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", ctrl->id);
		return err;
	}
#endif
	v4l_dbg(1, debug, decoder->client,
			"Set Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);

	return err;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl to enumerate supported formats
 */
static int
ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	struct ov9655_decoder *decoder = s->priv;
	int index;

	if (fmt == NULL)
		return -EINVAL;

	index = fmt->index;
	if ((index >= decoder->num_fmts) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memcpy(fmt, &decoder->fmt_list[index],
		sizeof(struct v4l2_fmtdesc));

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: index - %d (%s)\n",
			decoder->fmt_list[index].index,
			decoder->fmt_list[index].description);
	return 0;
}

/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int
ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;
	struct v4l2_pix_format *pix;
	enum ov9655_std current_std;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix = &f->fmt.pix;

	/* Calculate height and width based on current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (pix->pixelformat ==
			decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	if (ifmt == decoder->num_fmts)
		ifmt = 0;	/* None of the format matched, select default */
	pix->pixelformat = decoder->fmt_list[ifmt].pixelformat;

	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	v4l_dbg(1, debug, decoder->client,
			"Try FMT: pixelformat - %s, bytesperline - %d, "
			"Width - %d, Height - %d\n",
			decoder->fmt_list[ifmt].description, pix->bytesperline,
			pix->width, pix->height);
	return 0;
}

/*
 * ioctl_s_fmt_cap - V4L2 decoder interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int
ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_pix_format *pix;
	int rval;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	pix = &f->fmt.pix;
	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;
	
	decoder->pix = *pix;

	/* FIXME: set the hardware registers */

	return rval;
}

/*
 * ioctl_g_fmt_cap - V4L2 decoder interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the decoder's current pixel format in the v4l2_format
 * parameter.
 */
static int
ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	f->fmt.pix = decoder->pix;

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: bytesperline - %d, "
			"Width - %d, Height - %d\n",
			decoder->pix.bytesperline,
			decoder->pix.width, decoder->pix.height);
	return 0;
}

/*
 * ioctl_g_parm - V4L2 decoder interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_captureparm *cparm;
	enum ov9655_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/*
 * ioctl_s_parm - V4L2 decoder interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_fract *timeperframe;
	enum ov9655_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/*
 * ioctl_g_ifparm - V4L2 decoder interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p. This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct ov9655_decoder *decoder = s->priv;
	int rval;
	enum ov9655_std current_std;

	if (p == NULL)
		return -EINVAL;

	if (NULL == decoder->pdata->ifparm)
		return -EINVAL;

	rval = decoder->pdata->ifparm(p);
	if (rval) {
		v4l_err(decoder->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;
	
	decoder->current_std = current_std;
	
	/*
	 * this defines the expected XCLK frequency (MCLK is divided by the ISP)
	 *   XCLK = fps * width * height * bytespersample * prescaler
	 */
	
	switch(current_std) {
		case OV9655_IMAGE_STD_SXGA:
			p->u.bt656.clock_curr = 30*1520*1050*2*1;
			break;
		case OV9655_IMAGE_STD_VGA:
			p->u.bt656.clock_curr = 30*800*500*2*1;
			break;
		case OV9655_IMAGE_STD_QVGA:
			p->u.bt656.clock_curr = 30*800*500*2*1;	// scaling is done within camera
			break;
		case OV9655_IMAGE_STD_CIF:
			p->u.bt656.clock_curr = 30*800*500*2*1;	// scaling is done within camera
			break;
		default:
			return -EINVAL;
	}
	printk("xclk = %u\n", p->u.bt656.clock_curr);
	return 0;
}

/*
 * ioctl_g_priv - V4L2 decoder interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold decoder's private data address
 *
 * Returns device's (decoder's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov9655_decoder *decoder = s->priv;

	if (NULL == decoder->pdata->priv_data_set)
		return -EINVAL;

	return decoder->pdata->priv_data_set(p);
}

/*
 * ioctl_s_power - V4L2 decoder interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = 0;

	/* FIXME: this appears to be broken */
	/* the sensor still responds on I2C and sends SYNC */
	/* does this mean hardware power off or sleep */
	
	switch (on) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		/* TODO: FIXME: implement proper OFF and Standby code here */
#if 0
		err = ov9655_write_reg(decoder->client, REG_OPERATION_MODE,
				0x01);
#endif
		if (decoder->pdata->power_set)
			err |= decoder->pdata->power_set(s, on);	/* power off LDO */
		decoder->state = STATE_NOT_DETECTED;
		break;

	case V4L2_POWER_STANDBY:
	
//		err = ov9655_set(decoder->client, OV9655_COM2,
//						 OV9655_COM2_SLEEP, OV9655_COM2_SLEEP);	// set sleep flag

		if (decoder->pdata->power_set)
			err = decoder->pdata->power_set(s, on);
		break;

	case V4L2_POWER_ON:
		if (decoder->state == STATE_NOT_DETECTED) {

			if (decoder->pdata->power_set)
				err = decoder->pdata->power_set(s, on);

			/* Detect the sensor is not already detected */
			err |= ov9655_detect(decoder);
			if (err) {
				v4l_err(decoder->client,
						"Unable to detect decoder\n");
				return err;
			}
		}
		err = ov9655_configure(decoder);
		break;

	default:
		err = -ENODEV;
		break;
	}

	return err;
}

/*
 * ioctl_init - V4L2 decoder interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the decoder device (calls ov9655_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = 0;

	err = ov9655_configure(decoder);

	return err;
}

/*
 * ioctl_dev_exit - V4L2 decoder interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach. The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*
 * ioctl_dev_init - V4L2 decoder interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master. Returns 0 if
 * ov9655 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov9655_decoder *decoder = s->priv;
	int err;

	err = ov9655_detect(decoder);
	if (err < 0) {
		v4l_err(decoder->client,
			"Unable to detect decoder\n");
		return err;
	}

	v4l_info(decoder->client,
		 "chip version 0x%.2x detected\n", decoder->ver);

	err = ov9655_configure(decoder);

	return 0;
}

static struct v4l2_int_ioctl_desc ov9655_ioctl_desc[] = {
 {vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func*) ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_priv_num, (v4l2_int_ioctl_func*) ioctl_g_priv},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	 (v4l2_int_ioctl_func *) ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_querystd_num, (v4l2_int_ioctl_func *) ioctl_querystd},
	{vidioc_int_s_std_num, (v4l2_int_ioctl_func *) ioctl_s_std},
	{vidioc_int_s_video_routing_num,
		(v4l2_int_ioctl_func *) ioctl_s_routing},
	{vidioc_int_enum_framesizes_num,
		(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
		(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
};

static struct v4l2_int_slave ov9655_slave = {
	.ioctls = ov9655_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov9655_ioctl_desc),
};

static struct ov9655_decoder ov9655_dev = {
	.state = STATE_NOT_DETECTED,

	.fmt_list = ov9655_fmt_list,
	.num_fmts = ARRAY_SIZE(ov9655_fmt_list),

#if 0	/* initialized during setup */
	.pix = {		/* Default to 8-bit YUV 422 */
		.width = VGA_NUM_ACTIVE_PIXELS,
		.height = VGA_NUM_ACTIVE_LINES,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_NONE,
		.bytesperline = VGA_NUM_ACTIVE_PIXELS * 2,
		.sizeimage =
			VGA_NUM_ACTIVE_PIXELS * 2 * VGA_NUM_ACTIVE_LINES,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},

	.current_std = OV9655_STD_VGA,
#endif
	.std_list = ov9655_std_list,
	.num_stds = ARRAY_SIZE(ov9655_std_list),
	.current_std = OV9655_STD_INVALID,

};

static struct v4l2_int_device ov9655_int_device = {
	.module = THIS_MODULE,
	.name = OV9655_MODULE_NAME,
	.priv = &ov9655_dev,
	.type = v4l2_int_type_slave,
	.u = {
	      .slave = &ov9655_slave,
	      },
};

/*
 * ov9655_probe - decoder driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register decoder as an i2c client device and V4L2
 * device.
 */
static int
ov9655_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov9655_decoder *decoder = &ov9655_dev;
	int err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	decoder->pdata = client->dev.platform_data;
	if (!decoder->pdata) {
		v4l_err(client, "No platform data!!\n");
		return -ENODEV;
	}
	/*
	 * Save the id data, required for power up sequence
	 */
	decoder->id = (struct i2c_device_id *)id;
	/* Attach to Master */
	strcpy(ov9655_int_device.u.slave->attach_to, decoder->pdata->master);
	decoder->v4l2_int_device = &ov9655_int_device;
	decoder->client = client;
	i2c_set_clientdata(client, decoder);

	/* Register with V4L2 layer as slave device */
	err = v4l2_int_device_register(decoder->v4l2_int_device);
	if (err) {
		i2c_set_clientdata(client, NULL);
		v4l_err(client,
			"Unable to register to v4l2. Err[%d]\n", err);

	} else
		v4l_info(client, "Registered to v4l2 master %s!!\n",
				decoder->pdata->master);

	return 0;
}

/*
 * ov9655_remove - decoder driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister decoder as an i2c client device and V4L2
 * device. Complement of ov9655_probe().
 */
static int __exit ov9655_remove(struct i2c_client *client)
{
	struct ov9655_decoder *decoder = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(decoder->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

/*
 * I2C Device Table -
 *
 * name - Name of the actual device/chip.
 * driver_data - private Driver data
 */
static const struct i2c_device_id ov9655_id[] = {
	{"ov9655", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9655_id);

static struct i2c_driver ov9655_i2c_driver = {
	.driver = {
		   .name = OV9655_MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = ov9655_probe,
	.remove = __exit_p(ov9655_remove),
	.id_table = ov9655_id,
};

/*
 * ov9655_init
 *
 * Module init function
 */
static int __init ov9655_init(void)
{
	return i2c_add_driver(&ov9655_i2c_driver);
}

/*
 * ov9655_cleanup
 *
 * Module exit function
 */
static void __exit ov9655_cleanup(void)
{
	i2c_del_driver(&ov9655_i2c_driver);
}

module_init(ov9655_init);
module_exit(ov9655_cleanup);

MODULE_DESCRIPTION("OmniVision OV9655 Camera driver");
MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_LICENSE("GPL");
