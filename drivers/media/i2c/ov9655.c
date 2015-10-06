/*
 * Driver for OV9655 CMOS Image Sensor from OmniVision
 * 
 * H. N. Schaller <hns@goldelico.com>
 *
 * Based on Driver for MT9P031 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2011, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2011, Javier Martin <javier.martin@vista-silicon.com>
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on the MT9V032 driver and Bastian Hecht's code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* NOTE
 * THIS IS A DRIVER UNDER HEAVY DEVELOPMENT AND NEEDS A LOT OF CLEANUP
 *
 * therefore it is currently a mix of dead code fragments and working code
 * all MT9P031 is coming from the old MT9P031 driver
 * symbols starting with ov9655 are new
 *
 * currently it just initialized the OV9655 camera but does not configure
 * it specific to any user's settings
 *
 * power and reset are controlled fine
 *
 * and the XCLK is hard coded to be ~24 MHz
 *
 * what needs to be done:
 * - add functions to switch camera resolution (and frame rate)
 * - correctly set width, height, frame, crop, rotation
 * - correctly set gamma, AGC etc.
 * - add flash strobe trigger
 */
 
#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/ov9655.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#ifndef FIXED
/* these constants have disappeared in 3.19-rc1 - so we have to find a replacement */

#define V4L2_MBUS_FMT_UYVY8_2X8 0
#define V4L2_MBUS_FMT_RGB565_2X8_BE 1
#define V4L2_MBUS_FMT_SGRBG12_1X12 2
#endif

#if 1	// still needed since we have not yet updated the code to control an OV9655

// #include "aptina-pll.h"

#define MT9P031_PIXEL_ARRAY_WIDTH			2752
#define MT9P031_PIXEL_ARRAY_HEIGHT			2004

#define MT9P031_CHIP_VERSION				0x00
#define		MT9P031_CHIP_VERSION_VALUE		0x1801
#define MT9P031_ROW_START				0x01
#define		MT9P031_ROW_START_MIN			0
#define		MT9P031_ROW_START_MAX			2004
#define		MT9P031_ROW_START_DEF			54
#define MT9P031_COLUMN_START				0x02
#define		MT9P031_COLUMN_START_MIN		0
#define		MT9P031_COLUMN_START_MAX		2750
#define		MT9P031_COLUMN_START_DEF		16
#define MT9P031_WINDOW_HEIGHT				0x03
#define		MT9P031_WINDOW_HEIGHT_MIN		2
#define		MT9P031_WINDOW_HEIGHT_MAX		2006
#define		MT9P031_WINDOW_HEIGHT_DEF		1944
#define MT9P031_WINDOW_WIDTH				0x04
#define		MT9P031_WINDOW_WIDTH_MIN		2
#define		MT9P031_WINDOW_WIDTH_MAX		2752
#define		MT9P031_WINDOW_WIDTH_DEF		2592
#define MT9P031_HORIZONTAL_BLANK			0x05
#define		MT9P031_HORIZONTAL_BLANK_MIN		0
#define		MT9P031_HORIZONTAL_BLANK_MAX		4095
#define MT9P031_VERTICAL_BLANK				0x06
#define		MT9P031_VERTICAL_BLANK_MIN		1
#define		MT9P031_VERTICAL_BLANK_MAX		4096
#define		MT9P031_VERTICAL_BLANK_DEF		26
#define MT9P031_OUTPUT_CONTROL				0x07
#define		MT9P031_OUTPUT_CONTROL_CEN		2
#define		MT9P031_OUTPUT_CONTROL_SYN		1
#define		MT9P031_OUTPUT_CONTROL_DEF		0x1f82
#define MT9P031_SHUTTER_WIDTH_UPPER			0x08
#define MT9P031_SHUTTER_WIDTH_LOWER			0x09
#define		MT9P031_SHUTTER_WIDTH_MIN		1
#define		MT9P031_SHUTTER_WIDTH_MAX		1048575
#define		MT9P031_SHUTTER_WIDTH_DEF		1943
#define	MT9P031_PLL_CONTROL				0x10
#define		MT9P031_PLL_CONTROL_PWROFF		0x0050
#define		MT9P031_PLL_CONTROL_PWRON		0x0051
#define		MT9P031_PLL_CONTROL_USEPLL		0x0052
#define	MT9P031_PLL_CONFIG_1				0x11
#define	MT9P031_PLL_CONFIG_2				0x12
#define MT9P031_PIXEL_CLOCK_CONTROL			0x0a
#define MT9P031_FRAME_RESTART				0x0b
#define MT9P031_SHUTTER_DELAY				0x0c
#define MT9P031_RST					0x0d
#define		MT9P031_RST_ENABLE			1
#define		MT9P031_RST_DISABLE			0
#define MT9P031_READ_MODE_1				0x1e
#define MT9P031_READ_MODE_2				0x20
#define		MT9P031_READ_MODE_2_ROW_MIR		(1 << 15)
#define		MT9P031_READ_MODE_2_COL_MIR		(1 << 14)
#define		MT9P031_READ_MODE_2_ROW_BLC		(1 << 6)
#define MT9P031_ROW_ADDRESS_MODE			0x22
#define MT9P031_COLUMN_ADDRESS_MODE			0x23
#define MT9P031_GLOBAL_GAIN				0x35
#define		MT9P031_GLOBAL_GAIN_MIN			8
#define		MT9P031_GLOBAL_GAIN_MAX			1024
#define		MT9P031_GLOBAL_GAIN_DEF			8
#define		MT9P031_GLOBAL_GAIN_MULT		(1 << 6)
#define MT9P031_ROW_BLACK_TARGET			0x49
#define MT9P031_ROW_BLACK_DEF_OFFSET			0x4b
#define MT9P031_GREEN1_OFFSET				0x60
#define MT9P031_GREEN2_OFFSET				0x61
#define MT9P031_BLACK_LEVEL_CALIBRATION			0x62
#define		MT9P031_BLC_MANUAL_BLC			(1 << 0)
#define MT9P031_RED_OFFSET				0x63
#define MT9P031_BLUE_OFFSET				0x64
#define MT9P031_TEST_PATTERN				0xa0
#define		MT9P031_TEST_PATTERN_SHIFT		3
#define		MT9P031_TEST_PATTERN_ENABLE		(1 << 0)
#define		MT9P031_TEST_PATTERN_DISABLE		(0 << 0)
#define MT9P031_TEST_PATTERN_GREEN			0xa1
#define MT9P031_TEST_PATTERN_RED			0xa2
#define MT9P031_TEST_PATTERN_BLUE			0xa3

enum ov9655_model {
	MT9P031_MODEL_COLOR,
	MT9P031_MODEL_MONOCHROME,
};

#endif

/* ov9655 register addresses */
#define OV9655_GAIN			0x00
#define OV9655_BLUE			0x01
#define OV9655_RED			0x02
#define OV9655_VREF			0x03
#define OV9655_COM1			0x04
#define OV9655_BAVE			0x05
#define OV9655_GBAVE			0x06
#define OV9655_GRAVE			0x07
#define OV9655_RAVE			0x08
#define OV9655_COM2			0x09
#define OV9655_COM2_SLEEP	0x10
#define OV9655_PID			0x0A
#define OV9655_CHIP_PID			0x96
#define OV9655_VER			0x0B
#define OV9655_CHIP_VER4		0x56
#define OV9655_CHIP_VER5		0x57
#define OV9655_COM3			0x0C
#define OV9655_COM3_SWAP	0x40
#define OV9655_COM4			0x0D
#define OV9655_COM5			0x0E
#define OV9655_COM6			0x0F
#define OV9655_COM6_TIMING	0x02	
#define OV9655_COM6_WINDOW	0x04
#define OV9655_AEC			0x10
#define OV9655_CLKRC			0x11
#define OV9655_CLKRC_EXT	0x40
#define OV9655_CLKRC_SCALAR	0x3f
#define OV9655_COM7			0x12
#define OV9655_COM7_FMT_MASK	0x07
#define OV9655_COM7_RAW		0x00
#define OV9655_COM7_RAW_INT	0x01
#define OV9655_COM7_YUV		0x02
#define OV9655_COM7_RGB		0x03
#define OV9655_COM7_RGB5X5	0x07
#define OV9655_COM7_RES_MASK	0x70
#define OV9655_COM7_SXGA	0x00
#define OV9655_COM7_VGA		0x60
#define OV9655_COM8			0x13
#define OV9655_COM8_AGC		0x04
#define OV9655_COM8_AWB		0x02
#define OV9655_COM8_AEC		0x01
#define OV9655_COM9			0x14
#define OV9655_COM10			0x15
#define OV9655_COM10_HSYNC_NEG	0x01
#define OV9655_COM10_VSYNC_NEG	0x02
#define OV9655_COM10_RESET_END	0x04
#define OV9655_COM10_HREF_REV	0x08
#define OV9655_COM10_PCLK_REV	0x10
#define OV9655_COM10_PCLK_GATE	0x20
#define OV9655_COM10_HREF2HSYNC	0x40
#define OV9655_COM10_SLAVE_MODE	0x80
#define OV9655_REG16			0x16
#define OV9655_HSTART			0x17
#define OV9655_HSTOP			0x18
#define OV9655_VSTART			0x19
#define OV9655_VSTOP			0x1A
#define OV9655_PSHFT			0x1B
#define OV9655_MIDH			0x1C
#define OV9655_MIDL			0x1D
#define OV9655_CHIP_MID		0x7fa2
#define OV9655_MVFP			0x1E
#define OV9655_MVFP_VFLIP	0x10
#define OV9655_MVFP_MIRROR	0x20
#define OV9655_LAEC			0x1F
#define OV9655_BOS			0x20
#define OV9655_GBOS			0x21
#define OV9655_GROS			0x22
#define OV9655_ROS			0x23
#define OV9655_AEW			0x24
#define OV9655_AEB			0x25
#define OV9655_VPT			0x26
#define OV9655_BBIAS			0x27
#define OV9655_GBBIAS			0x28
#define OV9655_PREGAIN			0x29
#define OV9655_EXHCH			0x2A
#define OV9655_EXHCL			0x2B
#define OV9655_RBIAS			0x2C
#define OV9655_ADVFL			0x2D
#define OV9655_ADVFH			0x2E
#define OV9655_YAVE			0x2F
#define OV9655_HSYST			0x30
#define OV9655_HSYEN			0x31
#define OV9655_HREF			0x32
#define OV9655_CHLF			0x33
#define OV9655_AREF1			0x34
#define OV9655_AREF2			0x35
#define OV9655_AREF3			0x36
#define OV9655_ADC1			0x37
#define OV9655_ADC2			0x38
#define OV9655_AREF4			0x39
#define OV9655_TSLB			0x3A
#define OV9655_TSLB_YUV_MASK	0x0C
#define OV9655_TSLB_YUYV	0x00
#define OV9655_TSLB_YVYU	0x04
#define OV9655_TSLB_VYUY	0x08
#define OV9655_TSLB_UYVY	0x0C
#define OV9655_COM11			0x3B
#define OV9655_COM12			0x3C
#define OV9655_COM13			0x3D
#define OV9655_COM14			0x3E
#define OV9655_COM14_ZOOM	0x02
#define OV9655_EDGE			0x3F
#define OV9655_COM15			0x40
#define OV9655_COM15_RGB_MASK	0x30
#define OV9655_COM15_RGB	0x00
#define OV9655_COM15_RGB565	0x10
#define OV9655_COM15_RGB555	0x30
#define OV9655_COM16			0x41
#define OV9655_COM16_SCALING	0x01
#define OV9655_COM17			0x42
#define OV9655_MTX1			0x4F
#define OV9655_MTX2			0x50
#define OV9655_MTX3			0x51
#define OV9655_MTX4			0x52
#define OV9655_MTX5			0x53
#define OV9655_MTX6			0x54
#define OV9655_BRTN			0x55
#define OV9655_CNST1			0x56
#define OV9655_CNST2			0x57
#define OV9655_MTXS			0x58
#define OV9655_AWBOP1			0x59
#define OV9655_AWBOP2			0x5A
#define OV9655_AWBOP3			0x5B
#define OV9655_AWBOP4			0x5C
#define OV9655_AWBOP5			0x5D
#define OV9655_AWBOP6			0x5E
#define OV9655_BLMT			0x5F
#define OV9655_RLMT			0x60
#define OV9655_GLMT			0x61
#define OV9655_LCC1			0x62
#define OV9655_LCC2			0x63
#define OV9655_LCC3			0x64
#define OV9655_LCC4			0x65
#define OV9655_LCC5			0x66
#define OV9655_MANU			0x67
#define OV9655_MANV			0x68
#define OV9655_BD50MAX			0x6A
#define OV9655_DBLV			0x6B
#define OV9655_DBLV_BANDGAP		0x0a	/* default value */
#define OV9655_DBLV_LDO_BYPASS	0x10
#define OV9655_DBLV_PLL_BYPASS	0x00
#define OV9655_DBLV_PLL_4X		0x40
#define OV9655_DBLV_PLL_6X		0x80
#define OV9655_DBLV_PLL_8X		0xc0
#define OV9655_DNSTH			0x70
#define OV9655_POIDX			0x72
#define OV9655_POIDX_VDROP	0x40
#define OV9655_PCKDV			0x73
#define OV9655_XINDX			0x74
#define OV9655_YINDX			0x75
#define OV9655_SLOP			0x7A
#define OV9655_GAM1			0x7B
#define OV9655_GAM2			0x7C
#define OV9655_GAM3			0x7D
#define OV9655_GAM4			0x7E
#define OV9655_GAM5			0x7F
#define OV9655_GAM6			0x80
#define OV9655_GAM7			0x81
#define OV9655_GAM8			0x82
#define OV9655_GAM9			0x83
#define OV9655_GAM10			0x84
#define OV9655_GAM11			0x85
#define OV9655_GAM12			0x86
#define OV9655_GAM13			0x87
#define OV9655_GAM14			0x88
#define OV9655_GAM15			0x89
#define OV9655_COM18			0x8B
#define OV9655_COM19			0x8C
#define OV9655_COM20			0x8D
#define OV9655_DMLNL			0x92
#define OV9655_DMNLH			0x93
#define OV9655_LCC6			0x9D
#define OV9655_LCC7			0x9E
#define OV9655_AECH			0xA1
#define OV9655_BD50			0xA2
#define OV9655_BD60			0xA3
#define OV9655_COM21			0xA4
#define OV9655_GREEN			0xA6
#define OV9655_VZST			0xA7
#define OV9655_REFA8			0xA8
#define OV9655_REFA9			0xA9
#define OV9655_BLC1			0xAC
#define OV9655_BLC2			0xAD
#define OV9655_BLC3			0xAE
#define OV9655_BLC4			0xAF
#define OV9655_BLC5			0xB0
#define OV9655_BLC6			0xB1
#define OV9655_BLC7			0xB2
#define OV9655_BLC8			0xB3
#define OV9655_CTRLB4			0xB4
#define OV9655_FRSTL			0xB7
#define OV9655_FRSTH			0xB8
#define OV9655_ADBOFF			0xBC
#define OV9655_ADROFF			0xBD
#define OV9655_ADGBOFF			0xBE
#define OV9655_ADGROFF			0xBF
#define OV9655_COM23			0xC4
#define OV9655_BD60MAX			0xC5
#define OV9655_COM24			0xC7

#define CAMERA_TARGET_FREQ	48000000	/* pixel clock frequency for 15 fps SXGA (2 clocks per pixel for byte multiplexing) */
#define CAMERA_EXT_FREQ		24000000	/* must be between 10 and 48 MHz or I2C does not work */

/* do we need these constants? */
#define OV9655_MAX_WIDTH		1280
#define OV9655_MIN_WIDTH		2
#define OV9655_MAX_HEIGHT		1024
#define OV9655_MIN_HEIGHT		2
#define OV9655_COLUMN_SKIP		237
#define OV9655_ROW_SKIP			11
#define OV9655_LEFT_SKIP		3
#define OV9655_TOP_SKIP			1

#define OV9655_COLUMS			1520
#define OV9655_ROWS			1050

/* Number of pixels and number of lines per frame for different standards */
#define VGA_NUM_ACTIVE_PIXELS           (4*160) /* 4:3 */
#define VGA_NUM_ACTIVE_LINES            (3*160)
#define QVGA_NUM_ACTIVE_PIXELS          (VGA_NUM_ACTIVE_PIXELS/2)       /* 4:3 */
#define QVGA_NUM_ACTIVE_LINES           (VGA_NUM_ACTIVE_LINES/2)
#define SXGA_NUM_ACTIVE_PIXELS          (5*256) /* 5:4 */
#define SXGA_NUM_ACTIVE_LINES           (4*256)
#define CIF_NUM_ACTIVE_PIXELS           (11*32) /* 11:9 ~ 5:4 */
#define CIF_NUM_ACTIVE_LINES            (9*32)

#define WH(WIDTH, HEIGHT) ((((u32) HEIGHT)<<16)+((u32) WIDTH))

#define VGA		WH(VGA_NUM_ACTIVE_PIXELS, VGA_NUM_ACTIVE_LINES)
#define QVGA	WH(QVGA_NUM_ACTIVE_PIXELS, QVGA_NUM_ACTIVE_LINES)
#define SXGA	WH(SXGA_NUM_ACTIVE_PIXELS, SXGA_NUM_ACTIVE_LINES)
#define CIF		WH(CIF_NUM_ACTIVE_PIXELS, CIF_NUM_ACTIVE_LINES)

/* this is to compile the code to handle crop (no idea if and for what we need it) */

#define OV9655_PIXEL_ARRAY_WIDTH	OV9655_MAX_WIDTH
#define OV9655_PIXEL_ARRAY_HEIGHT	OV9655_MAX_HEIGHT
#define	OV9655_WINDOW_HEIGHT_MIN	2
#define	OV9655_WINDOW_HEIGHT_MAX	OV9655_MAX_HEIGHT
#define	OV9655_WINDOW_HEIGHT_DEF	OV9655_MAX_HEIGHT
#define	OV9655_WINDOW_WIDTH_MIN		2
#define	OV9655_WINDOW_WIDTH_MAX		OV9655_MAX_WIDTH
#define	OV9655_WINDOW_WIDTH_DEF		OV9655_MAX_WIDTH
#define	OV9655_ROW_START_MIN		0
#define	OV9655_ROW_START_MAX		OV9655_MAX_HEIGHT
#define	OV9655_ROW_START_DEF		0
#define	OV9655_COLUMN_START_MIN		0
#define	OV9655_COLUMN_START_MAX		OV9655_MAX_WIDTH
#define	OV9655_COLUMN_START_DEF		0

/* make this match the camera format settings in the registers */
// see http://lxr.free-electrons.com/source/include/uapi/linux/v4l2-mediabus.h#L37

/* media-ctl only knows these:
 { "Y8", V4L2_MBUS_FMT_Y8_1X8},
 { "Y10", V4L2_MBUS_FMT_Y10_1X10 },
 { "Y12", V4L2_MBUS_FMT_Y12_1X12 },
 { "YUYV", V4L2_MBUS_FMT_YUYV8_1X16 },
 { "YUYV1_5X8", V4L2_MBUS_FMT_YUYV8_1_5X8 },
 { "YUYV2X8", V4L2_MBUS_FMT_YUYV8_2X8 },
 { "UYVY", V4L2_MBUS_FMT_UYVY8_1X16 },
 { "UYVY1_5X8", V4L2_MBUS_FMT_UYVY8_1_5X8 },
 { "UYVY2X8", V4L2_MBUS_FMT_UYVY8_2X8 },
 { "SBGGR8", V4L2_MBUS_FMT_SBGGR8_1X8 },
 { "SGBRG8", V4L2_MBUS_FMT_SGBRG8_1X8 },
 { "SGRBG8", V4L2_MBUS_FMT_SGRBG8_1X8 },
 { "SRGGB8", V4L2_MBUS_FMT_SRGGB8_1X8 },
 { "SBGGR10", V4L2_MBUS_FMT_SBGGR10_1X10 },
 { "SGBRG10", V4L2_MBUS_FMT_SGBRG10_1X10 },
 { "SGRBG10", V4L2_MBUS_FMT_SGRBG10_1X10 },
 { "SRGGB10", V4L2_MBUS_FMT_SRGGB10_1X10 },
 { "SBGGR10_DPCM8", V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8 },
 { "SGBRG10_DPCM8", V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8 },
 { "SGRBG10_DPCM8", V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8 },
 { "SRGGB10_DPCM8", V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8 },
 { "SBGGR12", V4L2_MBUS_FMT_SBGGR12_1X12 },
 { "SGBRG12", V4L2_MBUS_FMT_SGBRG12_1X12 },
 { "SGRBG12", V4L2_MBUS_FMT_SGRBG12_1X12 },
 { "SRGGB12", V4L2_MBUS_FMT_SRGGB12_1X12 },
*/ 

// #define OV9655_FORMAT 	 V4L2_MBUS_FMT_SGRBG12_1X12	// this was ok for MT9P031
// #define OV9655_FORMAT 	V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE
#define OV9655_FORMAT 	V4L2_MBUS_FMT_UYVY8_2X8
// #define OV9655_FORMAT 	V4L2_MBUS_FMT_YUYV8_2X8
// #define OV9655_FORMAT	V4L2_MBUS_FMT_UYVY8_1X16
// #define OV9655_FORMAT 	 V4L2_MBUS_FMT_RGB565_2X8_BE


/* to bulk program camera registers */

struct ov9655_reg {
	u8 addr;
	u8 value;
	u8 clear;	/* mask all bits with this value before setting the (new) value: newbit = (oldbit & clear) | value; if clear != 0 this is a read-modify-write command, otherwise (i.e. not explicitly initialized) a direct write */
};

/* we assume that the camera is operated as follows:
 * XCLK is 24 MHz (required to be between 10 MHz and 48 MHz to operate I2C)
 * PCLK is 48 MHz for SXGA 15 fps and VGA 30 fps, no delay
 * HSYNC and VSYNC are positive impulses
 * HSYNC is HSYNC and not HREF
 * data polarity is not inverted
 * please make sure that the capture interface is configured accordingly
 * data format is YUV
 * SXGA/VGA/QVGA/CIF is asjusted by the format settings
 */

static const struct ov9655_reg ov9655_init_hardware[] = {
	/* here we write only registers for the hardware interface - must be matched by the hardware interface in the board file */
	{ OV9655_COM2, 0x01 },	/* drive outputs at 2x; disable soft sleep */
	{ OV9655_COM10, OV9655_COM10_HREF2HSYNC /* | OV9655_COM10_HSYNC_NEG */ },	/* define pin polarity and functions as default (VSYNC, HSYNC as positive pulses) */
	{ OV9655_CLKRC, 0 },	/* compensate for PLL_4X (note this means: PCLK = XCLK x 4) */
	{ OV9655_DBLV, OV9655_DBLV_PLL_4X | OV9655_DBLV_BANDGAP },
};

static const struct ov9655_reg ov9655_init_regs[] = {
	/* here we do some common settings (bias, gain, agc etc.) */
#if 0	// color bar test mode
	{ OV9655_COM3, 0x80 },	// color bar for testing
	{ OV9655_COM20, 0x10 },	// color bar for testing
#else
	{ OV9655_COM3, 0x00, ~OV9655_COM3_SWAP },	// don't swap MSB and LSB
#endif
	{ OV9655_COM6, 0x40 },	/* manually update window size and timing */
	{ OV9655_COM7, OV9655_COM7_YUV, ~OV9655_COM7_FMT_MASK },	/* choose YUV */
	{ OV9655_COM11, 0x05 }, // no night mode	
	{ OV9655_COM15, 0xc0 },	// full scale output range and RGB555
	
	//	{ OV9655_COM19, 0x0c },	// UV

	//	{ OV9655_COM1, 0x03 },	// AEC low bits
	//	{ OV9655_COM5, 0x61 },	// slam mode & exposure
	//	{ OV9655_COM8, 0xe0 | OV9655_COM8_AGC | OV9655_COM8_AWB | OV9655_COM8_AEC },
	//	{ OV9655_COM9, 0x2a },	// agc
	//	{ OV9655_REG16, 0x24 },
	//	{ OV9655_MVFP, 0 },	// mirror&flip
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
	//	{ OV9655_COM13, 0x99 },
	//	{ OV9655_EDGE, 0x02 },	// edge enhancement factor
	
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
	{ OV9655_COM7, OV9655_COM7_SXGA, ~OV9655_COM7_RES_MASK }, 
	{ OV9655_HSYEN, 0x50 }	/* adjust sync */
#if 0	// stuff from other driver without knowing if that is good or bad
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
#endif
};

/* Register values for VGA format */
static const struct ov9655_reg ov9655_vga[] = {
	{ OV9655_COM7, OV9655_COM7_VGA, ~OV9655_COM7_RES_MASK },
#if 0	// stuff from other driver without knowing if that is good or bad
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
#endif
};

/* Register values for QVGA format */
static const struct ov9655_reg ov9655_qvga[] = {
	{ OV9655_COM7, OV9655_COM7_VGA, ~OV9655_COM7_RES_MASK },
#if 0	// stuff from other driver without knowing if that is good or bad
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
#endif
};

/* Register values for CIF format */
static const struct ov9655_reg ov9655_cif[] = {
	/* fixme: this is QQVGA and not CIF */
	{ OV9655_COM7, OV9655_COM7_VGA, ~OV9655_COM7_RES_MASK },
#if 0	// stuff from other driver without knowing if that is good or bad
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
#endif
};

/* Register values for YUV format */
static const struct ov9655_reg ov9655_uyvy_regs[] = {
	/* merge value with VGA/SXGA setting! */
	{ OV9655_COM7, OV9655_COM7_YUV, ~OV9655_COM7_FMT_MASK },	/* choose YUV */
	{ OV9655_TSLB, OV9655_TSLB_VYUY, ~OV9655_TSLB_YUV_MASK },	/* VYUY byte order */
#if 0	// stuff from other driver without knowing if that is good or bad
	{ OV9655_MTX1, 0x80 },
	{ OV9655_MTX2, 0x80 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x22 },
	{ OV9655_MTX5, 0x5e },
	{ OV9655_MTX6, 0x80 },
	{ OV9655_MTXS, 0x1e },
#endif
};

/* Register values for RGB format */
static const struct ov9655_reg ov9655_rgb_regs[] = {
	{ OV9655_COM7, OV9655_COM7_RGB, OV9655_COM7_FMT_MASK },	/* choose RGB */
	{ OV9655_COM15, 0xc0 | OV9655_COM15_RGB555 },	// full scale output range and RGB555
#if 0	// stuff from other driver without knowing if that is good or bad
	//	{ OV9655_MTX1, 0x98 },
	{ OV9655_MTX2, 0x98 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x28,},
	{ OV9655_MTX5, 0x70 },
	{ OV9655_MTX6, 0x98 },
	{ OV9655_MTXS, 0x1a },
#endif
};

/* Register values for RGB-RAW format */
static const struct ov9655_reg ov9655_raw_regs[] = {
	{ OV9655_COM7, OV9655_COM7_RAW, OV9655_COM7_FMT_MASK },	/* choose RGB */
	{ OV9655_COM15, 0xc0 | OV9655_COM15_RGB555 },	// full scale output range and RGB555
#if 0	// stuff from other driver without knowing if that is good or bad
	//	{ OV9655_MTX1, 0x98 },
	{ OV9655_MTX2, 0x98 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x28,},
	{ OV9655_MTX5, 0x70 },
	{ OV9655_MTX6, 0x98 },
	{ OV9655_MTXS, 0x1a },
#endif
};

struct ov9655 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_rect crop;  /* Sensor window */
	struct v4l2_mbus_framefmt format;
	struct ov9655_platform_data *pdata;
	struct mutex power_lock; /* lock to protect power_count */

	int power_count;
	int reset;	/* reset GPIO number */

	struct clk *clk;
	struct regulator *vdd;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *blc_auto;
	struct v4l2_ctrl *blc_offset;
};

static struct ov9655 *to_ov9655(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9655, subdev);
}

static int  ov9655_read(struct i2c_client *client, u8 reg)
{
	dev_info(&client->dev, "OV9655 read register %02x\n", reg);
	return i2c_smbus_read_byte_data(client, reg);
}

static int  ov9655_write(struct i2c_client *client, u8 reg, u8 data)
{
	dev_info(&client->dev, "OV9655 write %02x to register %02x\n", data, reg);
	return i2c_smbus_write_byte_data(client, reg, data);
}

static int ov9655_write_regs(struct i2c_client *client,
							 const struct ov9655_reg *regs, const int n)
{
	int i, ret;
	for (i = 0; i < n; i++) {
		u8 val = regs[i].value;
		if(regs[i].clear != 0) { /* modify only some bits */
			ret = ov9655_read(client, regs[i].addr);
			if(ret < 0)
				return ret;
			val |= (ret & regs[i].clear);
			if(val == (u8) ret)
				continue;	/* no need to write */
		}
		ret = ov9655_write(client, regs[i].addr, val);
		if (ret < 0)
			return ret;
	}
	return 0;
}

#if 0	// stubs to fulfill old driver code

static int  mt9p031_read(struct i2c_client *client, u8 reg)
{
	// disable reading anything
	return 0;
	return i2c_smbus_read_byte_data(client, reg);
}

#endif

#if 1

static int  mt9p031_write(struct i2c_client *client, u8 reg, u16 data)
{
	// disable writing to anything
	return 0;	
	return i2c_smbus_write_word_swapped(client, reg, data);
}
#endif

#if 1	// remove since the OV9655 has different registers for that purpose

static int ov9655_set_output_control(struct ov9655 *ov9655, u16 clear,	// appears to control enable/disable
				      u16 set)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	u16 value = (ov9655->output_control & ~clear) | set;
	int ret;

//	printk("ov9655_set_output_control\n");

	ret =  mt9p031_write(client, MT9P031_OUTPUT_CONTROL, value);
	if (ret < 0)
		return ret;

	ov9655->output_control = value;
#endif
	return 0;
}

static int ov9655_set_mode2(struct ov9655 *ov9655, u16 clear, u16 set)	// appears to control black level compensation and mirroring
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	u16 value = (ov9655->mode2 & ~clear) | set;
	int ret;

//	printk("ov9655_set_mode2\n");

	ret =  mt9p031_write(client, MT9P031_READ_MODE_2, value);
	if (ret < 0)
		return ret;

	ov9655->mode2 = value;
#endif
	return 0;
}

#endif	// remove

static int ov9655_reset(struct ov9655 *ov9655)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	int ret;

	printk("ov9655_reset\n");
	dev_info(&client->dev, "ov9655_reset\n");

#if 0
	ret =  ov9655_write(client, OV9655_COM7, 0x82);	/* reset to chip defaults */
	if (ret < 0)
		{
		printk("write failed err=%d\n", ret);
		dev_info(&client->dev, "write failed err=%d\n", ret);
		return ret;
		}
#endif
	usleep_range(1000, 2000);
	
	return ov9655_write_regs(client, ov9655_init_hardware, ARRAY_SIZE(ov9655_init_hardware));
}

// FIXME: adjust timing to OV9655 recommendations

static int ov9655_power_on(struct ov9655 *ov9655)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	dev_info(&client->dev, "OV9655 power on\n");
	/* Ensure RESET_BAR is low */
	if (ov9655->reset != -1) {
		gpio_set_value(ov9655->reset, 0);
		usleep_range(1000, 2000);
	}

	/* Bring up the supplies */
    ret = regulator_enable(ov9655->vdd);
	if (ret < 0) {
		dev_info(&client->dev, "regulator_enable failed err=%d\n", ret);
		return ret;
	}

	/* Enable clock */
	
	if (ov9655->clk)
		clk_prepare_enable(ov9655->clk);

	/* Now RESET_BAR must be high */
	if (ov9655->reset != -1) {
		gpio_set_value(ov9655->reset, 1);
		usleep_range(1000, 2000);
	}

	return 0;
}

static void ov9655_power_off(struct ov9655 *ov9655)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	dev_info(&client->dev, "OV9655 power off\n");
	if (ov9655->reset != -1) {
		gpio_set_value(ov9655->reset, 0);
		usleep_range(1000, 2000);
	}

	regulator_disable(ov9655->vdd);

	if (ov9655->clk)
		clk_disable_unprepare(ov9655->clk);
}

static int __ov9655_set_power(struct ov9655 *ov9655, bool on)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	int ret;

	if (!on) {
		ov9655_power_off(ov9655);
		return 0;
	}

	ret = ov9655_power_on(ov9655);
	if (ret < 0)
		return ret;

	ret = ov9655_reset(ov9655);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to reset the camera\n");
		return ret;
	}

	return v4l2_ctrl_handler_setup(&ov9655->ctrls);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

#if 1	// our private development testing code because dev_info can't be controlled that easily

void printfmt(struct v4l2_format *format)
{
	printk("fmt=%d h=%u w=%u bpl=%u size=%u\n", format->fmt.pix.pixelformat,
		   format->fmt.pix.height,
		   format->fmt.pix.width,
		   format->fmt.pix.bytesperline,
		   format->fmt.pix.sizeimage);
}

void printmbusfmt(struct v4l2_mbus_framefmt *format)
{
	printk("h=%u w=%u code=%u field=%u csp=%u\n", format->height,
		   format->width,
		   format->code,
		   format->field,
		   format->colorspace);
}

#endif

static int ov9655_set_params(struct ov9655 *ov9655)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	struct v4l2_mbus_framefmt *format = &ov9655->format;
	const struct v4l2_rect *crop = &ov9655->crop;
	unsigned int hblank;
	unsigned int vblank;
	unsigned int xskip;
	unsigned int yskip;
	unsigned int xbin;
	unsigned int ybin;
	int ret = 0;

	dev_info(&client->dev, "ov9655_set_params\n");
	
	printk("ov9655_set_params\n");
	printmbusfmt(format);

	/* generic initialization */
	ov9655_write_regs(client, ov9655_init_regs, ARRAY_SIZE(ov9655_init_regs));

	/* format specific initializations */
	
	switch(WH(format->width, format->height)) {

			// FIXME: should we also set/change the pixel clock here?

		case SXGA:
			printk("SXGA\n");
			ov9655_write_regs(client, ov9655_sxga, ARRAY_SIZE(ov9655_sxga));
			break;
		case VGA:
			printk("VGA\n");
			ov9655_write_regs(client, ov9655_vga, ARRAY_SIZE(ov9655_vga));
		case QVGA:
			printk("QVGA\n");
			ov9655_write_regs(client, ov9655_qvga, ARRAY_SIZE(ov9655_qvga));
			break;
		case CIF:
			printk("CIF\n");
			ov9655_write_regs(client, ov9655_cif, ARRAY_SIZE(ov9655_cif));
			break;
	}

	printk("format->code=%08x\n", format->code);

	switch(format->code) {
			// handle format->code == V4L2_MBUS_FMT_BGR565_2X8_BE etc.
			// to set COM3[2], COM7[1:0], TSLB[4:2], COM15[5:4], 			
		case V4L2_MBUS_FMT_UYVY8_2X8:
			ov9655_write_regs(client, ov9655_uyvy_regs, ARRAY_SIZE(ov9655_uyvy_regs));
			break;
		case V4L2_MBUS_FMT_RGB565_2X8_BE:
			ov9655_write_regs(client, ov9655_rgb_regs, ARRAY_SIZE(ov9655_rgb_regs));
			break;
		case V4L2_MBUS_FMT_SGRBG12_1X12:
			ov9655_write_regs(client, ov9655_raw_regs, ARRAY_SIZE(ov9655_raw_regs));
			break;
		default:
			// should have been rejected in ov9655_set_format()
			break;
	}

	printk("format->field=%08x\n", format->field);

	// format->field could ask for some interlacing
	
	// we should also handle the crop rect
	
#if 0
	/* Windows position and size.
	 *
	 * TODO: Make sure the start coordinates and window size match the
	 * skipping, binning and mirroring (see description of registers 2 and 4
	 * in table 13, and Binning section on page 41).
	 */
	ret =  mt9p031_write(client, MT9P031_COLUMN_START, crop->left);
	if (ret < 0)
		return ret;
	ret =  mt9p031_write(client, MT9P031_ROW_START, crop->top);
	if (ret < 0)
		return ret;
	ret =  mt9p031_write(client, MT9P031_WINDOW_WIDTH, crop->width - 1);
	if (ret < 0)
		return ret;
	ret =  mt9p031_write(client, MT9P031_WINDOW_HEIGHT, crop->height - 1);
	if (ret < 0)
		return ret;

	/* Row and column binning and skipping. Use the maximum binning value
	 * compatible with the skipping settings.
	 */
	xskip = DIV_ROUND_CLOSEST(crop->width, format->width);
	yskip = DIV_ROUND_CLOSEST(crop->height, format->height);
	xbin = 1 << (ffs(xskip) - 1);
	ybin = 1 << (ffs(yskip) - 1);

	ret =  mt9p031_write(client, MT9P031_COLUMN_ADDRESS_MODE,
			    ((xbin - 1) << 4) | (xskip - 1));
	if (ret < 0)
		return ret;
	ret =  mt9p031_write(client, MT9P031_ROW_ADDRESS_MODE,
			    ((ybin - 1) << 4) | (yskip - 1));
	if (ret < 0)
		return ret;

	/* Blanking - use minimum value for horizontal blanking and default
	 * value for vertical blanking.
	 */
	hblank = 346 * ybin + 64 + (80 >> min_t(unsigned int, xbin, 3));
	vblank = MT9P031_VERTICAL_BLANK_DEF;

	ret =  mt9p031_write(client, MT9P031_HORIZONTAL_BLANK, hblank - 1);
	if (ret < 0)
		return ret;
	ret =  mt9p031_write(client, MT9P031_VERTICAL_BLANK, vblank - 1);
	if (ret < 0)
		return ret;
#endif
	return ret;
}

static int ov9655_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	int ret;

	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	printk("ov9655_s_stream %d\n", enable);
	dev_info(&client->dev, "ov9655_s_stream(%d)\n", enable);

	if (!enable) {
		/* Stop sensor readout */
#if 0
		ret = ov9655_set_output_control(ov9655,
						 MT9P031_OUTPUT_CONTROL_CEN, 0);
		if (ret < 0)
			return ret;
#endif
		return 0;
	}

	ret = ov9655_set_params(ov9655);
	if (ret < 0)
		return ret;

#if 0
	/* Switch to master "normal" mode */
	ret = ov9655_set_output_control(ov9655, 0,
					 MT9P031_OUTPUT_CONTROL_CEN);
	if (ret < 0)
		return ret;
#endif
	return 0;
}

static int ov9655_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);

	printk("ov9655_enum_mbus_code\n");
	if (code->pad || code->index)
		return -EINVAL;

	code->code = ov9655->format.code;
	return 0;
}

static int ov9655_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);

	printk("ov9655_enum_frame_size\n");
	if (fse->index >= 8 || fse->code != ov9655->format.code)
		return -EINVAL;

	fse->min_width = OV9655_WINDOW_WIDTH_DEF
		       / min_t(unsigned int, 7, fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = OV9655_WINDOW_HEIGHT_DEF / (fse->index + 1);
	fse->max_height = fse->min_height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov9655_get_pad_format(struct ov9655 *ov9655, struct v4l2_subdev_fh *fh,
			 unsigned int pad, u32 which)
{
	printk("__ov9655_get_pad_format pad=%u which=%u %s\n", pad, which, which==V4L2_SUBDEV_FORMAT_TRY?"V4L2_SUBDEV_FORMAT_TRY":"V4L2_SUBDEV_FORMAT_ACTIVE");
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		// FIXME:
		return v4l2_subdev_get_try_format(fh, NULL, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9655->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__ov9655_get_pad_crop(struct ov9655 *ov9655, struct v4l2_subdev_fh *fh,
		     unsigned int pad, u32 which)
{
	printk("__ov9655_get_pad_crop pad=%u which=%u %s\n", pad, which, which==V4L2_SUBDEV_FORMAT_TRY?"V4L2_SUBDEV_FORMAT_TRY":"V4L2_SUBDEV_FORMAT_ACTIVE");
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		// FIXME:
		return v4l2_subdev_get_try_crop(fh, NULL, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9655->crop;
	default:
		return NULL;
	}
}

static int ov9655_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *fmt)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);

	printk("ov9655_get_format\n");
	fmt->format = *__ov9655_get_pad_format(ov9655, fh, fmt->pad,
						fmt->which);
	printmbusfmt(&fmt->format);
	return 0;
}

static int ov9655_set_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	unsigned int width;
	unsigned int height;
	unsigned int hratio;
	unsigned int vratio;

	printk("ov9655_set_format\n");
	printmbusfmt(&format->format);

	__crop = __ov9655_get_pad_crop(ov9655, fh, format->pad,
					format->which);

	/* Clamp the width and height to avoid dividing by zero. */
	width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
			max(__crop->width / 7, OV9655_WINDOW_WIDTH_MIN),
			__crop->width);
	height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
			max(__crop->height / 8, OV9655_WINDOW_HEIGHT_MIN),
			__crop->height);

	hratio = DIV_ROUND_CLOSEST(__crop->width, width);
	vratio = DIV_ROUND_CLOSEST(__crop->height, height);

	/* take what has been defined for the pad
	 * by user space command e.g.
	 *    media-ctl -V '"ov9655 2-0030":0 [UYVY2x8 1024x1024]'
	 */

	__format = __ov9655_get_pad_format(ov9655, fh, format->pad,
					    format->which);
	printmbusfmt(__format);
	__format->width = __crop->width / hratio;
	__format->height = __crop->height / vratio;
	printmbusfmt(__format);

	format->format = *__format;
	printmbusfmt(&format->format);
	
	switch(WH(__format->width, __format->height)) {
		case SXGA:
		case VGA:
		case QVGA:
		case CIF:
			break;	/* ok */
		default:
			printk("ov9655_set_format unknown format width=%u height=%u\n", __format->width, __format->height);
			return -EINVAL;	// unknown format
	}

	switch(__format->code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
		case V4L2_MBUS_FMT_RGB565_2X8_BE:
		case V4L2_MBUS_FMT_SGRBG12_1X12:
			break;
		default:
			printk("ov9655_set_format unknown format code=%08x\n", __format->code);
			return -EINVAL;	// unknown format
	}

	return 0;
}

// replace by ov9655_get_selection
static int ov9655_get_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);

	printk("ov9655_get_crop\n");
	crop->rect = *__ov9655_get_pad_crop(ov9655, fh, crop->pad,
					     crop->which);
	return 0;
}

// replace by ov9655_set_selection

static int ov9655_set_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;

	printk("ov9655_set_crop\n");
	/* Clamp the crop rectangle boundaries and align them to a multiple of 2
	 * pixels to ensure a GRBG Bayer pattern.
	 */
	rect.left = clamp(ALIGN(crop->rect.left, 2), OV9655_COLUMN_START_MIN,
			  OV9655_COLUMN_START_MAX);
	rect.top = clamp(ALIGN(crop->rect.top, 2), OV9655_ROW_START_MIN,
			 OV9655_ROW_START_MAX);
	rect.width = clamp(ALIGN(crop->rect.width, 2),
			   OV9655_WINDOW_WIDTH_MIN,
			   OV9655_WINDOW_WIDTH_MAX);
	rect.height = clamp(ALIGN(crop->rect.height, 2),
			    OV9655_WINDOW_HEIGHT_MIN,
			    OV9655_WINDOW_HEIGHT_MAX);

	rect.width = min(rect.width, OV9655_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min(rect.height, OV9655_PIXEL_ARRAY_HEIGHT - rect.top);

	__crop = __ov9655_get_pad_crop(ov9655, fh, crop->pad, crop->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/* Reset the output image size if the crop rectangle size has
		 * been modified.
		 */
		__format = __ov9655_get_pad_format(ov9655, fh, crop->pad,
						    crop->which);
		__format->width = rect.width;
		__format->height = rect.height;
	}

	*__crop = rect;
	crop->rect = rect;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */

#define V4L2_CID_BLC_AUTO		(V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_BLC_TARGET_LEVEL	(V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_BLC_ANALOG_OFFSET	(V4L2_CID_USER_BASE | 0x1004)
#define V4L2_CID_BLC_DIGITAL_OFFSET	(V4L2_CID_USER_BASE | 0x1005)

static int ov9655_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9655 *ov9655 =
			container_of(ctrl->handler, struct ov9655, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	u16 data;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
			printk("ov9655_s_ctrl V4L2_CID_EXPOSURE %08x\n", ctrl->val);
		ret =  mt9p031_write(client, MT9P031_SHUTTER_WIDTH_UPPER,
				    (ctrl->val >> 16) & 0xffff);
		if (ret < 0)
			return ret;

		return  mt9p031_write(client, MT9P031_SHUTTER_WIDTH_LOWER,
				     ctrl->val & 0xffff);

	case V4L2_CID_GAIN:
			printk("ov9655_s_ctrl V4L2_CID_GAIN %u\n", ctrl->val);
		/* Gain is controlled by 2 analog stages and a digital stage.
		 * Valid values for the 3 stages are
		 *
		 * Stage                Min     Max     Step
		 * ------------------------------------------
		 * First analog stage   x1      x2      1
		 * Second analog stage  x1      x4      0.125
		 * Digital stage        x1      x16     0.125
		 *
		 * To minimize noise, the gain stages should be used in the
		 * second analog stage, first analog stage, digital stage order.
		 * Gain from a previous stage should be pushed to its maximum
		 * value before the next stage is used.
		 */
		if (ctrl->val <= 32) {
			data = ctrl->val;
		} else if (ctrl->val <= 64) {
			ctrl->val &= ~1;
			data = (1 << 6) | (ctrl->val >> 1);
		} else {
			ctrl->val &= ~7;
			data = ((ctrl->val - 64) << 5) | (1 << 6) | 32;
		}

		return  mt9p031_write(client, MT9P031_GLOBAL_GAIN, data);

	case V4L2_CID_HFLIP:
			printk("ov9655_s_ctrl V4L2_CID_HFLIP %u\n", ctrl->val);
		if (ctrl->val)
			return ov9655_set_mode2(ov9655,
					0, MT9P031_READ_MODE_2_COL_MIR);
		else
			return ov9655_set_mode2(ov9655,
					MT9P031_READ_MODE_2_COL_MIR, 0);

	case V4L2_CID_VFLIP:
			printk("ov9655_s_ctrl V4L2_CID_VFLIP %u\n", ctrl->val);
		if (ctrl->val)
			return ov9655_set_mode2(ov9655,
					0, MT9P031_READ_MODE_2_ROW_MIR);
		else
			return ov9655_set_mode2(ov9655,
					MT9P031_READ_MODE_2_ROW_MIR, 0);

	case V4L2_CID_TEST_PATTERN:
			printk("ov9655_s_ctrl V4L2_CID_TEST_PATTERN %u\n", ctrl->val);
		if (!ctrl->val) {
			/* Restore the black level compensation settings. */
			if (ov9655->blc_auto->cur.val != 0) {
				ret = ov9655_s_ctrl(ov9655->blc_auto);
				if (ret < 0)
					return ret;
			}
			if (ov9655->blc_offset->cur.val != 0) {
				ret = ov9655_s_ctrl(ov9655->blc_offset);
				if (ret < 0)
					return ret;
			}
			return  mt9p031_write(client, MT9P031_TEST_PATTERN,
					     MT9P031_TEST_PATTERN_DISABLE);
		}

		ret =  mt9p031_write(client, MT9P031_TEST_PATTERN_GREEN, 0x05a0);
		if (ret < 0)
			return ret;
		ret =  mt9p031_write(client, MT9P031_TEST_PATTERN_RED, 0x0a50);
		if (ret < 0)
			return ret;
		ret =  mt9p031_write(client, MT9P031_TEST_PATTERN_BLUE, 0x0aa0);
		if (ret < 0)
			return ret;

		/* Disable digital black level compensation when using a test
		 * pattern.
		 */
		ret = ov9655_set_mode2(ov9655, MT9P031_READ_MODE_2_ROW_BLC,
					0);
		if (ret < 0)
			return ret;

		ret =  mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET, 0);
		if (ret < 0)
			return ret;

		return  mt9p031_write(client, MT9P031_TEST_PATTERN,
				((ctrl->val - 1) << MT9P031_TEST_PATTERN_SHIFT)
				| MT9P031_TEST_PATTERN_ENABLE);

	case V4L2_CID_BLC_AUTO:
			printk("ov9655_s_ctrl V4L2_CID_BLC_AUTO %d\n", ctrl->val);
		ret = ov9655_set_mode2(ov9655,
				ctrl->val ? 0 : MT9P031_READ_MODE_2_ROW_BLC,
				ctrl->val ? MT9P031_READ_MODE_2_ROW_BLC : 0);
		if (ret < 0)
			return ret;

		return  mt9p031_write(client, MT9P031_BLACK_LEVEL_CALIBRATION,
				     ctrl->val ? 0 : MT9P031_BLC_MANUAL_BLC);

	case V4L2_CID_BLC_TARGET_LEVEL:
			printk("ov9655_s_ctrl V4L2_CID_BLC_TARGET_LEVEL %08x\n", ctrl->val);
		return  mt9p031_write(client, MT9P031_ROW_BLACK_TARGET,
				     ctrl->val);

	case V4L2_CID_BLC_ANALOG_OFFSET:
			printk("ov9655_s_ctrl V4L2_CID_BLC_ANALOG_OFFSET %08x\n", ctrl->val);
		data = ctrl->val & ((1 << 9) - 1);

		ret =  mt9p031_write(client, MT9P031_GREEN1_OFFSET, data);
		if (ret < 0)
			return ret;
		ret =  mt9p031_write(client, MT9P031_GREEN2_OFFSET, data);
		if (ret < 0)
			return ret;
		ret =  mt9p031_write(client, MT9P031_RED_OFFSET, data);
		if (ret < 0)
			return ret;
		return  mt9p031_write(client, MT9P031_BLUE_OFFSET, data);

	case V4L2_CID_BLC_DIGITAL_OFFSET:
			printk("ov9655_s_ctrl V4L2_CID_BLC_DIGITAL_OFFSET %08x\n", ctrl->val);
		return  mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET,
				     ctrl->val & ((1 << 12) - 1));
	}

	return 0;
}

static struct v4l2_ctrl_ops ov9655_ctrl_ops = {
	.s_ctrl = ov9655_s_ctrl,
};

static const char * const ov9655_test_pattern_menu[] = {
	"Disabled",
	"Color Field",
	"Horizontal Gradient",
	"Vertical Gradient",
	"Diagonal Gradient",
	"Classic Test Pattern",
	"Walking 1s",
	"Monochrome Horizontal Bars",
	"Monochrome Vertical Bars",
	"Vertical Color Bars",
};

static const struct v4l2_ctrl_config ov9655_ctrls[] = {
	{
		.ops		= &ov9655_ctrl_ops,
		.id		= V4L2_CID_BLC_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "BLC, Auto",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
		.flags		= 0,
	}, {
		.ops		= &ov9655_ctrl_ops,
		.id		= V4L2_CID_BLC_TARGET_LEVEL,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "BLC Target Level",
		.min		= 0,
		.max		= 4095,
		.step		= 1,
		.def		= 168,
		.flags		= 0,
	}, {
		.ops		= &ov9655_ctrl_ops,
		.id		= V4L2_CID_BLC_ANALOG_OFFSET,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "BLC Analog Offset",
		.min		= -255,
		.max		= 255,
		.step		= 1,
		.def		= 32,
		.flags		= 0,
	}, {
		.ops		= &ov9655_ctrl_ops,
		.id		= V4L2_CID_BLC_DIGITAL_OFFSET,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "BLC Digital Offset",
		.min		= -2048,
		.max		= 2047,
		.step		= 1,
		.def		= 40,
		.flags		= 0,
	}
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int ov9655_set_power(struct v4l2_subdev *subdev, int on)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	int ret = 0;

	mutex_lock(&ov9655->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (ov9655->power_count == !on) {
		ret = __ov9655_set_power(ov9655, !!on);
		if (ret < 0)
			goto out;
	}

	/* Update the power count. */
	ov9655->power_count += on ? 1 : -1;
	WARN_ON(ov9655->power_count < 0);

out:
	mutex_unlock(&ov9655->power_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ov9655_registered(struct v4l2_subdev *subdev)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct ov9655 *ov9655 = to_ov9655(subdev);
	s32 data;
	int ret;

	ret = ov9655_power_on(ov9655);
	if (ret < 0) {
		dev_err(&client->dev, "OV9655 power up failed\n");
		return ret;
	}

	/* Read chip manufacturer register */
	data = ( ov9655_read(client, OV9655_MIDH) << 8) +  ov9655_read(client, OV9655_MIDL);
	
	if (data < 0) {
		dev_err(&client->dev, "OV9655 not detected, can't read manufacturer id\n");
		return -ENODEV;
	}
	
	if (data != OV9655_CHIP_MID) {
		dev_err(&client->dev, "OV9655 not detected, wrong manufacturer "
				"0x%04x\n", (unsigned) data);
		return -ENODEV;
	}
	
	data =  ov9655_read(client, OV9655_PID);
	if (data != OV9655_CHIP_PID) {
		dev_err(&client->dev, "OV9655 not detected, wrong part "
				"0x%02x\n", (unsigned) data);
		return -ENODEV;
	}
	
	data =  ov9655_read(client, OV9655_VER);
	if (data != OV9655_CHIP_VER4 && data != OV9655_CHIP_VER5) {
		dev_err(&client->dev, "OV9655 not detected, wrong version "
				"0x%02x\n", (unsigned) data);
		return -ENODEV;
	}

	ov9655_power_off(ov9655);

	dev_info(&client->dev, "OV9655 detected at address 0x%02x\n",
		 client->addr);

	return ret;
}

static int ov9655_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	printk("ov9655_open\n");
	
	// v4l2_subdev_get_try_crop
	crop = v4l2_subdev_get_try_crop(fh, NULL, 0);
	crop->left = OV9655_COLUMN_START_DEF;
	crop->top = OV9655_ROW_START_DEF;
	crop->width = OV9655_WINDOW_WIDTH_DEF;
	crop->height = OV9655_WINDOW_HEIGHT_DEF;

	// FIXME
	format = v4l2_subdev_get_try_format(fh, NULL, 0);

	format->code = OV9655_FORMAT;

	format->width = OV9655_WINDOW_WIDTH_DEF;
	format->height = OV9655_WINDOW_HEIGHT_DEF;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return ov9655_set_power(subdev, 1);
}

static int ov9655_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	printk("ov9655_close\n");
	return ov9655_set_power(subdev, 0);
}

static struct v4l2_subdev_core_ops ov9655_subdev_core_ops = {
	.s_power        = ov9655_set_power,
};

static struct v4l2_subdev_video_ops ov9655_subdev_video_ops = {
	.s_stream       = ov9655_s_stream,
};

static struct v4l2_subdev_pad_ops ov9655_subdev_pad_ops = {
	.enum_mbus_code = ov9655_enum_mbus_code,
	.enum_frame_size = ov9655_enum_frame_size,
	.get_fmt = ov9655_get_format,
	.set_fmt = ov9655_set_format,
// should become get_selection/set_selection
// as of commit 05b9cc3eeffcb2b0d6720930fc144058d4ace1aa
//	.get_crop = ov9655_get_crop,
//	.set_crop = ov9655_set_crop,
};

static struct v4l2_subdev_ops ov9655_subdev_ops = {
	.core   = &ov9655_subdev_core_ops,
	.video  = &ov9655_subdev_video_ops,
	.pad    = &ov9655_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov9655_subdev_internal_ops = {
	.registered = ov9655_registered,
	.open = ov9655_open,
	.close = ov9655_close,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static int ov9655_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov9655_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ov9655 *ov9655;
	unsigned int i;
	int ret;

	if (pdata == NULL) {
		// parse DT here
		dev_err(&client->dev, "No platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev,
			"I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov9655 = kzalloc(sizeof(*ov9655), GFP_KERNEL);
	if (ov9655 == NULL)
		return -ENOMEM;

	ov9655->pdata = pdata;
//	ov9655->output_control	= MT9P031_OUTPUT_CONTROL_DEF;
//	ov9655->mode2 = MT9P031_READ_MODE_2_ROW_BLC;
//	ov9655->model = did->driver_data;	// second paramter from ov9655_id[]
	ov9655->reset = -1;

    ov9655->vdd = devm_regulator_get(&client->dev, "vaux3");

	if (IS_ERR(ov9655->vdd)) {
		dev_err(&client->dev, "Unable to get regulator\n");
		return -ENODEV;
	}

	v4l2_ctrl_handler_init(&ov9655->ctrls, ARRAY_SIZE(ov9655_ctrls) + 6);

	/* register custom controls */
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_EXPOSURE, MT9P031_SHUTTER_WIDTH_MIN,
			  MT9P031_SHUTTER_WIDTH_MAX, 1,
			  MT9P031_SHUTTER_WIDTH_DEF);
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_GAIN, MT9P031_GLOBAL_GAIN_MIN,
			  MT9P031_GLOBAL_GAIN_MAX, 1, MT9P031_GLOBAL_GAIN_DEF);
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_PIXEL_RATE, CAMERA_TARGET_FREQ,
			  CAMERA_TARGET_FREQ, 1, CAMERA_TARGET_FREQ);
	v4l2_ctrl_new_std_menu_items(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_TEST_PATTERN,
			  ARRAY_SIZE(ov9655_test_pattern_menu) - 1, 0,
			  0, ov9655_test_pattern_menu);

	for (i = 0; i < ARRAY_SIZE(ov9655_ctrls); ++i)
		v4l2_ctrl_new_custom(&ov9655->ctrls, &ov9655_ctrls[i], NULL);

	ov9655->subdev.ctrl_handler = &ov9655->ctrls;

	if (ov9655->ctrls.error) {
		printk(KERN_INFO "%s: control initialization error %d\n",
		       __func__, ov9655->ctrls.error);
		ret = ov9655->ctrls.error;
		goto done;
	}

	ov9655->blc_auto = v4l2_ctrl_find(&ov9655->ctrls, V4L2_CID_BLC_AUTO);
	ov9655->blc_offset = v4l2_ctrl_find(&ov9655->ctrls,
					     V4L2_CID_BLC_DIGITAL_OFFSET);

	mutex_init(&ov9655->power_lock);
	v4l2_i2c_subdev_init(&ov9655->subdev, client, &ov9655_subdev_ops);
	ov9655->subdev.internal_ops = &ov9655_subdev_internal_ops;

	ov9655->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ov9655->subdev.entity, 1, &ov9655->pad, 0);
	if (ret < 0)
		goto done;

	ov9655->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ov9655->crop.width = OV9655_WINDOW_WIDTH_DEF;
	ov9655->crop.height = OV9655_WINDOW_HEIGHT_DEF;
	ov9655->crop.left = OV9655_COLUMN_START_DEF;
	ov9655->crop.top = OV9655_ROW_START_DEF;

	ov9655->format.code = OV9655_FORMAT;

	ov9655->format.width = OV9655_WINDOW_WIDTH_DEF;
	ov9655->format.height = OV9655_WINDOW_HEIGHT_DEF;
	ov9655->format.field = V4L2_FIELD_NONE;
	ov9655->format.colorspace = V4L2_COLORSPACE_SRGB;

	if (pdata->reset != -1) {
		ret = gpio_request_one(pdata->reset, GPIOF_OUT_INIT_LOW,
				       "ov9655_rst");
		if (ret < 0)
			goto done;

		ov9655->reset = pdata->reset;
	}

	ov9655->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(ov9655->clk))
		return PTR_ERR(ov9655->clk);

	clk_set_rate(ov9655->clk, CAMERA_EXT_FREQ /* pdata->ext_freq */);

	ret = 0;

done:
	if (ret < 0) {
		if (ov9655->reset != -1)
			gpio_free(ov9655->reset);

		v4l2_ctrl_handler_free(&ov9655->ctrls);
		media_entity_cleanup(&ov9655->subdev.entity);
		kfree(ov9655);
	}

	return ret;
}

static int ov9655_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ov9655 *ov9655 = to_ov9655(subdev);

	v4l2_ctrl_handler_free(&ov9655->ctrls);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	if (ov9655->reset != -1)
		gpio_free(ov9655->reset);
	kfree(ov9655);

	return 0;
}

static const struct i2c_device_id ov9655_id[] = {
	{ "ov9655", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9655_id);

static const struct of_device_id of_ov9655_match_tbl[] = {
	{ .compatible = "omnivision,ov9655", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_ov9655_match_tbl);

static struct i2c_driver ov9655_i2c_driver = {
	.driver = {
		.name = "ov9655",
		.of_match_table = of_ov9655_match_tbl,
	},
	.probe          = ov9655_probe,
	.remove         = ov9655_remove,
	.id_table       = ov9655_id,
};

module_i2c_driver(ov9655_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV9655 Camera driver");
MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_LICENSE("GPL v2");
