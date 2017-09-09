/*
 * Driver for OV9655 CMOS Image Sensor from OmniVision
 *
 * Copyright (C) 2017, H. N. Schaller <hns@goldelico.com>
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

#define DEBUG 1
#define USEI2C 0

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

struct ov9655 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_rect crop;  /* Sensor window */
	struct v4l2_mbus_framefmt format;
	struct mutex power_lock; /* lock to protect power_count */

	int power_count;
	struct gpio_desc *reset;	/* reset GPIO */
	struct gpio_desc *powerdown;	/* powerdown GPIO */

	unsigned long xclk_frequency;
	struct clk *clk;
	struct regulator_bulk_data regulators[3];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *blc_auto;
	struct v4l2_ctrl *blc_offset;

	u32 bus_width;
	u32 hsync_active;
	u32 vsync_active;
	u32 data_active;
	u32 pclk_sample;
	u32 pclk_delay;
	u32 output_drive;
	bool clock_noncontinuous;
	bool slave_mode;
};

/* ov9655 register addresses */
#define OV9655_GAIN			0x00
#define OV9655_BLUE			0x01
#define OV9655_RED			0x02
#define   OV9655_VREF_END		0x38
#define   OV9655_VREF_START		0x03
#define OV9655_VREF			0x03
#define OV9655_COM1			0x04
#define   OV9655_COM1_AEC		0x03
#define OV9655_BAVE			0x05
#define OV9655_GBAVE			0x06
#define OV9655_GRAVE			0x07
#define OV9655_RAVE			0x08
#define OV9655_COM2			0x09
#define   OV9655_COM2_STRENGTH		0x03
#define   OV9655_COM2_SLEEP		0x10
#define OV9655_PID			0x0A
#define   OV9655_CHIP_PID		0x96
#define OV9655_REV			0x0B
#define   OV9655_CHIP_REV4		0x56
#define   OV9655_CHIP_REV5		0x57
#define OV9655_COM3			0x0C
#define   OV9655_COM3_RGB565		0x04
#define   OV9655_COM3_SWAP		0x40
#define   OV9655_COM3_CBAR		0x80
#define OV9655_COM4			0x0D
#define OV9655_COM5			0x0E
#define OV9655_COM6			0x0F
#define   OV9655_COM6_TIMING		0x02
#define   OV9655_COM6_WINDOW		0x04
#define OV9655_AEC			0x10
#define OV9655_CLKRC			0x11
#define   OV9655_CLKRC_EXT		0x40
#define   OV9655_CLKRC_SCALAR		0x3f
#define OV9655_COM7			0x12
#define   OV9655_COM7_FMT_MASK		0x07
#define     OV9655_COM7_RAW		0x00
#define     OV9655_COM7_RAW_INT	0x01
#define     OV9655_COM7_YUV		0x02
#define     OV9655_COM7_RGB		0x03
#define     OV9655_COM7_RGB5X5		0x07
#define   OV9655_COM7_RES_MASK		0x70
#define     OV9655_COM7_SXGA		0x00
#define     OV9655_COM7_VGA		0x60
#define OV9655_COM8			0x13
#define   OV9655_COM8_AGC		0x04
#define   OV9655_COM8_AWB		0x02
#define   OV9655_COM8_AEC		0x01
#define OV9655_COM9			0x14
#define OV9655_COM10			0x15
#define   OV9655_COM10_HSYNC_NEG	0x01
#define   OV9655_COM10_VSYNC_NEG	0x02
#define   OV9655_COM10_RESET_END	0x04
#define   OV9655_COM10_HREF_REV	0x08
#define   OV9655_COM10_PCLK_REV	0x10
#define   OV9655_COM10_PCLK_GATE	0x20
#define   OV9655_COM10_HREF2HSYNC	0x40
#define   OV9655_COM10_SLAVE_MODE	0x80
#define OV9655_REG16			0x16
#define OV9655_HSTART			0x17
#define OV9655_HSTOP			0x18
#define OV9655_VSTART			0x19
#define OV9655_VSTOP			0x1A
#define OV9655_PSHFT			0x1B
#define OV9655_MIDH			0x1C
#define OV9655_MIDL			0x1D
#define   OV9655_CHIP_MID		0x7fa2
#define OV9655_MVFP			0x1E
#define   OV9655_MVFP_VFLIP		0x10
#define   OV9655_MVFP_MIRROR		0x20
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
#define   OV9655_HREF_END		0x38
#define   OV9655_HREF_START		0x03
#define OV9655_CHLF			0x33
#define OV9655_AREF1			0x34
#define OV9655_AREF2			0x35
#define OV9655_AREF3			0x36
#define OV9655_ADC1			0x37
#define OV9655_ADC2			0x38
#define OV9655_AREF4			0x39
#define OV9655_TSLB			0x3A
#define   OV9655_TSLB_PCLK_MASK	0xC0
#define   OV9655_TSLB_PCLK_OFFSET	6
#define   OV9655_TSLB_YUV_MASK		0x0C
#define   OV9655_TSLB_YUYV		0x00
#define   OV9655_TSLB_YVYU		0x04
#define   OV9655_TSLB_VYUY		0x08
#define   OV9655_TSLB_UYVY		0x0C
#define OV9655_COM11			0x3B
#define OV9655_COM12			0x3C
#define OV9655_COM13			0x3D
#define OV9655_COM14			0x3E
#define   OV9655_COM14_ZOOM		0x02
#define OV9655_EDGE			0x3F
#define OV9655_COM15			0x40
#define   OV9655_COM15_RGB_MASK	0x30
#define     OV9655_COM15_RGB		0x00
#define     OV9655_COM15_RGB565	0x10
#define     OV9655_COM15_RGB555	0x30
#define OV9655_COM16			0x41
#define   OV9655_COM16_SCALING		0x01
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
#define OV9655_69			0x69	/* undocumented but must be changed for VGA */
#define   OV9655_69_SXGA		0x02
#define   OV9655_69_VGA		0x0a
#define OV9655_BD50MAX			0x6A
#define OV9655_DBLV			0x6B
#define   OV9655_DBLV_BANDGAP_MASK	0x0F
#define     OV9655_DBLV_BANDGAP	0x0a	/* default value */
#define   OV9655_DBLV_LDO_BYPASS	0x10
#define   OV9655_DBLV_PLL_MASK		0xC0
#define     OV9655_DBLV_PLL_BYPASS	0x00
#define     OV9655_DBLV_PLL_4X		0x40
#define     OV9655_DBLV_PLL_6X		0x80
#define     OV9655_DBLV_PLL_8X		0xc0
#define OV9655_DNSTH			0x70
#define OV9655_POIDX			0x72
#define   OV9655_POIDX_VDROP		0x40
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
#define   OV9655_COM20_CBAR		0x10
#define OV9655_DMLNL			0x92
#define OV9655_DMNLH			0x93
#define OV9655_LCC6			0x9D
#define OV9655_LCC7			0x9E
#define OV9655_AECH			0xA1
#define   OV9655_AECH_AEC		0x3f
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

/* dummy value for V4L2_CID_PIXEL_RATE */
#define CAMERA_TARGET_FREQ	48000000

#define OV9655_SHUTTER_WIDTH_MIN	0
#define OV9655_SHUTTER_WIDTH_MAX	65535
#define OV9655_SHUTTER_WIDTH_DEF	(OV9655_SHUTTER_WIDTH_MAX / 2 + 1)

/* do we need these constants? */
#define OV9655_MAX_WIDTH		1280
#define OV9655_MIN_WIDTH		2
#define OV9655_MAX_HEIGHT		1024
#define OV9655_MIN_HEIGHT		2
#define OV9655_COLUMN_SKIP		237
#define OV9655_ROW_SKIP		11
#define OV9655_LEFT_SKIP		3
#define OV9655_TOP_SKIP		1

/* total sensor dimensions */
#define OV9655_COLUMS			1520
#define OV9655_ROWS			1050

/* Number of pixels and number of lines per frame for different standards */
#define SXGA_NUM_ACTIVE_PIXELS          (5 * 256) /* 5:4 */
#define SXGA_NUM_ACTIVE_LINES           (4 * 256)
#define VGA_NUM_ACTIVE_PIXELS           (4 * 160) /* 4:3 */
#define VGA_NUM_ACTIVE_LINES            (3 * 160)
#define QVGA_NUM_ACTIVE_PIXELS          (VGA_NUM_ACTIVE_PIXELS / 2)       /* 4:3 */
#define QVGA_NUM_ACTIVE_LINES           (VGA_NUM_ACTIVE_LINES / 2)
#define CIF_NUM_ACTIVE_PIXELS           (11 * 32) /* 11:9 ~ 5:4 */
#define CIF_NUM_ACTIVE_LINES            (9 * 32)

#define WH(WIDTH, HEIGHT) ((((u32) HEIGHT) << 16) + ((u32) WIDTH))

#define SXGA	WH(SXGA_NUM_ACTIVE_PIXELS, SXGA_NUM_ACTIVE_LINES)
#define VGA	WH(VGA_NUM_ACTIVE_PIXELS, VGA_NUM_ACTIVE_LINES)
#define QVGA	WH(QVGA_NUM_ACTIVE_PIXELS, QVGA_NUM_ACTIVE_LINES)
#define QQVGA	WH(VGA_NUM_ACTIVE_PIXELS/4, VGA_NUM_ACTIVE_LINES/4)
#define CIF	WH(CIF_NUM_ACTIVE_PIXELS, CIF_NUM_ACTIVE_LINES)

/* this is to compile the code to handle crop and probably needs to be FIXED */

#define OV9655_PIXEL_ARRAY_WIDTH	OV9655_MAX_WIDTH
#define OV9655_PIXEL_ARRAY_HEIGHT	OV9655_MAX_HEIGHT
#define OV9655_WINDOW_HEIGHT_MIN	2
#define OV9655_WINDOW_HEIGHT_MAX	OV9655_MAX_HEIGHT
#define OV9655_WINDOW_HEIGHT_DEF	OV9655_MAX_HEIGHT
#define OV9655_WINDOW_WIDTH_MIN	2
#define OV9655_WINDOW_WIDTH_MAX	OV9655_MAX_WIDTH
#define OV9655_WINDOW_WIDTH_DEF	OV9655_MAX_WIDTH
#define OV9655_ROW_START_MIN		0
#define OV9655_ROW_START_MAX		OV9655_MAX_HEIGHT
#define OV9655_ROW_START_DEF		0
#define OV9655_COLUMN_START_MIN	0
#define OV9655_COLUMN_START_MAX	OV9655_MAX_WIDTH
#define OV9655_COLUMN_START_DEF	0

struct ov9655_pixfmt {
	u32 code;
	u32 colorspace;
};

struct ov9655_framesize {
	u16 width;
	u16 height;
	u16 fps;
	u16 hstart;
	u16 hend;
	u16 vstart;
	u16 vend;
	u8 resolution;
	u8 clkrc;
	u8 scaling;
};

static const struct ov9655_framesize ov9655_framesizes[] = {
	{ /* SXGA */
		.width		= SXGA_NUM_ACTIVE_PIXELS,
		.height		= SXGA_NUM_ACTIVE_LINES,
		.fps		= 15,
		.hstart		= 200,
		.hend		= 200+SXGA_NUM_ACTIVE_PIXELS,
		.vstart		= 0,
		.vend		= 0+SXGA_NUM_ACTIVE_LINES,
		.resolution	= OV9655_COM7_SXGA,
		.clkrc		= 0,	/* compensate for PLL_4X (note this means: PCLK = XCLK x 4) */
		.scaling	= 0,	/* disable scaling */
	}, { /* VGA */
		.width		= VGA_NUM_ACTIVE_PIXELS,
		.height		= VGA_NUM_ACTIVE_LINES,
		.fps		= 30,
		.hstart		= 160,
		.hend		= 160+VGA_NUM_ACTIVE_PIXELS,
		.vstart		= 8,
		.vend		= 8+VGA_NUM_ACTIVE_LINES,
		.resolution	= OV9655_COM7_VGA,
		.clkrc		= 1,	/* VGA needs 1/4 of of SGXA pixel rate but has 30fps */
		.scaling	= 0,	/* disable scaling */
	}, { /* QVGA */
		.width		= QVGA_NUM_ACTIVE_PIXELS,
		.height		= QVGA_NUM_ACTIVE_LINES,
		.fps		= 30,
		/* TODO: fix me */
		.hstart		= 154,
		.hend		= 750,
		.vstart		= 10,
		.vend		= 550,
		.resolution	= OV9655_COM7_VGA,
		.clkrc		= 3,	/* VGA needs 1/4 of of SGXA pixel rate but has 30fps */
		.scaling	= 1,	/* enable scaling */
	}, { /* QQVGA */
		.width		= QVGA_NUM_ACTIVE_PIXELS/2,
		.height		= QVGA_NUM_ACTIVE_PIXELS/2,
		.fps		= 30,
		/* TODO: fix me */
		.hstart		= 154,
		.hend		= 750,
		.vstart		= 10,
		.vend		= 550,
		.resolution	= OV9655_COM7_VGA,
		.clkrc		= 7,	/* VGA needs 1/4 of of SGXA pixel rate but has 30fps */
		.scaling	= 1,	/* enable scaling */
	}, { /* CIF */
		.width		= CIF_NUM_ACTIVE_PIXELS,
		.height		= CIF_NUM_ACTIVE_LINES,
		.fps		= 30,
		/* TODO: fix me */
		.hstart		= 154,
		.hend		= 750,
		.vstart		= 10,
		.vend		= 550,
		.resolution	= OV9655_COM7_VGA,
		.clkrc		= 3,	/* VGA needs 1/4 of of SGXA pixel rate but has 30fps */
		.scaling	= 1,	/* enable scaling */
	},
};

/*
 * choose formats that are handled by media-ctl
 *  see http://lxr.free-electrons.com/source/include/uapi/linux/media-bus-format.h#L37
 *  and https://git.linuxtv.org/v4l-utils.git/tree/utils/media-ctl/libv4l2subdev.c#n853
 */

static const struct ov9655_pixfmt ov9655_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_VYUY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YVYU8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE, V4L2_COLORSPACE_SRGB, },
//	{ MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_COLORSPACE_SRGB, },
};

static struct ov9655 *to_ov9655(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9655, subdev);
}

static int ov9655_read(struct i2c_client *client, u8 reg)
{
#if !USEI2C	// SMBUS
	int val;

	val = i2c_smbus_read_byte_data(client, reg);
	dev_info(&client->dev, "OV9655 read register %02x : %02x\n", reg, val);
	return val;
#else
	u8 buf = reg;
	u8 val;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &buf
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1) {
		msg.flags = I2C_M_RD;
		ret = i2c_transfer(client->adapter, &msg, 1);

		if (ret == 1)
			val = buf;
	}

	v4l2_dbg(2, true, client, "%s: 0x%02x @ 0x%02x. (%d)\n",
		__func__, val, reg, ret);

	return ret == 1 ? val : -EIO;
#endif
}

static int ov9655_write(struct i2c_client *client, u8 reg, u8 data)
{
#if !USEI2C
	dev_info(&client->dev, "OV9655 write register %02x : %02x\n", reg, data);
	return i2c_smbus_write_byte_data(client, reg, data);
#else
	u8 buf[2] = { reg, data };

	int ret = i2c_master_send(client, buf, 2);

	dev_info(&client->dev, "OV9655 write register %02x : %02x\n", reg, data);
	v4l2_dbg(2, true, client, "%s: 0x%02x @ 0x%02X (%d)\n",
		__func__, reg, data, ret);

	return ret == 2 ? 0 : ret;
#endif
}

static int ov9655_update_bits(struct i2c_client *client, u8 reg, unsigned int mask, unsigned int val)
{
	if (mask != 0xff) { /* modify not all bits */
		int ret, tmp;

		ret = ov9655_read(client, reg);
		if (ret < 0)
			return ret;

		tmp = ret & ~mask;
		tmp |= val & mask;

		if (tmp == ret)
			return 0;	/* no need to change */
		val = tmp;
	}

	return ov9655_write(client, reg, val);
}

static int ov9655_reset(struct ov9655 *ov9655)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	int ret;

	dev_info(&client->dev, "%s\n", __func__);

#if 0
	ret =  ov9655_write(client, OV9655_COM7, 0x82);	/* reset to chip defaults */
	if (ret < 0) {
		dev_err(&client->dev, "%s: write failed err=%d\n", __func__, ret);
		return ret;
		}
	usleep_range(1000, 2000);
#endif

	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_HREF2HSYNC, OV9655_COM10_HREF2HSYNC);
	if (ret < 0) {
		dev_err(&client->dev, "%s: write failed err=%d\n", __func__, ret);
		return ret;
		}

	/* assume next writes succeed */
	ret = ov9655_update_bits(client, OV9655_COM12, 0x80, 0x80);	/* set "always has href" */

	dev_info(&client->dev, "%s: output_drive %d\n", __func__, ov9655->output_drive);
	ret = ov9655_update_bits(client, OV9655_COM2, OV9655_COM2_STRENGTH,
		ov9655->output_drive&OV9655_COM2_STRENGTH);
	dev_info(&client->dev, "%s: pclk_sample %d\n", __func__, ov9655->pclk_sample);
	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_PCLK_REV,
		ov9655->pclk_sample ? 0: OV9655_COM10_PCLK_REV);
	dev_info(&client->dev, "%s: vsync_active %d\n", __func__, ov9655->vsync_active);
	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_VSYNC_NEG,
		ov9655->vsync_active ? 0 : OV9655_COM10_VSYNC_NEG);
	dev_info(&client->dev, "%s: hsync_active %d\n", __func__, ov9655->hsync_active);
	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_HSYNC_NEG,
		ov9655->hsync_active ? OV9655_COM10_HSYNC_NEG : 0);
	dev_info(&client->dev, "%s: pclk_delay %d\n", __func__, ov9655->pclk_delay);
	ret = ov9655_update_bits(client, OV9655_TSLB, OV9655_TSLB_PCLK_MASK,
		ov9655->pclk_delay << OV9655_TSLB_PCLK_OFFSET);

	dev_info(&client->dev, "%s: clock_noncontinuous %d\n", __func__, ov9655->clock_noncontinuous);
	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_PCLK_GATE,
		ov9655->clock_noncontinuous ? OV9655_COM10_PCLK_GATE : 0);
	dev_info(&client->dev, "%s: slave_mode %d\n", __func__, ov9655->slave_mode);
	ret = ov9655_update_bits(client, OV9655_COM10, OV9655_COM10_SLAVE_MODE,
		ov9655->slave_mode ? OV9655_COM10_SLAVE_MODE : 0);

	dev_info(&client->dev, "%s: data_active %d\n", __func__, ov9655->data_active);
	dev_info(&client->dev, "%s: bus_width %d\n", __func__, ov9655->bus_width);

	return ret;
}

// we maybe should insert code like static int mt9p031_clk_setup(struct mt9p031 *mt9p031)

static int __ov9655_set_power(struct ov9655 *ov9655, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	int ret;

	dev_info(&client->dev, "%s on=%d\n", __func__, on);

	if (on) {
		/* Bring up the power supply */
		ret = regulator_bulk_enable(ARRAY_SIZE(ov9655->regulators),
					   ov9655->regulators);
		if (ret < 0) {
			dev_err(&client->dev, "regulator_enable failed err=%d\n", ret);
			return ret;
		}
		msleep(25);

		/* Enable clock */
		ret = clk_prepare_enable(ov9655->clk);
		if (ret)
			dev_warn(&client->dev,
				 "Failed to enable clock (%d)\n", ret);
		msleep(1);

		gpiod_set_value_cansleep(ov9655->powerdown, 0);
		gpiod_set_value_cansleep(ov9655->reset, 0);

		msleep(1);

	} else {
		gpiod_set_value_cansleep(ov9655->reset, 1);
		/* FIXME: if we have a dedicated DOVDD regulator we should not keep powerdown asserted */
		gpiod_set_value_cansleep(ov9655->powerdown, 1);

		clk_disable_unprepare(ov9655->clk);
		regulator_bulk_disable(ARRAY_SIZE(ov9655->regulators),
			       ov9655->regulators);

	}
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

#if 1	// our private development testing code because dev_info can't be controlled that easily

void printfmt(struct i2c_client *client, struct v4l2_format *format)
{
	dev_info(&client->dev, "fmt: pix=%d h=%u w=%u bpl=%u size=%u\n", format->fmt.pix.pixelformat,
		   format->fmt.pix.height,
		   format->fmt.pix.width,
		   format->fmt.pix.bytesperline,
		   format->fmt.pix.sizeimage);
}

void printmbusfmt(struct i2c_client *client, struct v4l2_mbus_framefmt *format)
{
	dev_info(&client->dev, "busfmt: h=%u w=%u code=%u field=%u csp=%u\n", format->height,
		   format->width,
		   format->code,
		   format->field,
		   format->colorspace);
}

#endif

static int ov9655_set_params(struct ov9655 *ov9655)
{ /* called by ov9655_s_stream() */
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	struct v4l2_mbus_framefmt *format = &ov9655->format;
	const struct v4l2_rect *crop = &ov9655->crop;
	int i;
	int val, ret = 0;
	int is_sxga;

	dev_info(&client->dev, "%s\n", __func__);

	/* general settings */

	ret = ov9655_update_bits(client, OV9655_COM6, 0xff, 0x40);	/* manually update window size and timing and use optical BLC */
	if (ret < 0)
		return ret;
	ret = ov9655_update_bits(client, OV9655_COM11, 0xff, 0x05);	// no night mode
	if (ret < 0)
		return ret;
	ret = ov9655_update_bits(client, OV9655_COM15, 0xff, 0xc0);	// full scale output range and RGB555
	if (ret < 0)
		return ret;

	printmbusfmt(client, format);

	/* set clock PLL */

	// FIXME: should depend on format and desired FPS

	/*
	 * basically we should calculate:
	 * check if bit 6 of CKRC is set -> then external clock: pclk = xclk
	 *
	 * target-pixel-clock = fps * (horiz-res + sync) * (vert-res + sync)
	 * pll-clock = xclk * DBLV (x1, x4, x6, x8)
	 * CLKRC divisor = (pll-clock / target-pixel-clock) - 1
	 *
	 * note: we need twice the pixel clock because we have 2 pclk per pixel (2x8 bit)
	 *
	 * example: xclk = 24 MHz
	 * PLL_4X gives pll-clock = 48 MHz
	 * SXGA @ 15fps needs 48 MHz
	 * CLKRC = 0 gives :1
	 *
	 * example: xclk = 24 MHz
	 * PLL_4X gives pll-clock = 48 MHz
	 * VGA @ 30fps needs 24 MHz
	 * CLKRC = 1 gives :2
	 * probably we can also run with external clock
	 */

	/* set resolution */

	i = ARRAY_SIZE(ov9655_framesizes);

	while (--i)
		/* width is unique enough */
		if (format->width == ov9655_framesizes[i].width)
			break;

	ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_RES_MASK, ov9655_framesizes[i].resolution);

	is_sxga = (ov9655_framesizes[i].resolution == OV9655_COM7_SXGA);

	if (format->code == MEDIA_BUS_FMT_SGRBG12_1X12)
		;	/* adjust pixel clock for 12 bit wide (raw rgb) interface */

	ret = ov9655_update_bits(client, OV9655_DBLV, OV9655_DBLV_PLL_MASK, OV9655_DBLV_PLL_4X);

	ret = ov9655_write(client, OV9655_CLKRC, ov9655_framesizes[i].clkrc);
	ret = ov9655_write(client, OV9655_AREF3, is_sxga ? 0xf9 : 0xfa);
	ret = ov9655_write(client, OV9655_69, is_sxga ? OV9655_69_SXGA : OV9655_69_VGA);
	ret = ov9655_update_bits(client, OV9655_COM19, 0x80, is_sxga ? 0 : 0x80);
	ret = ov9655_write(client, OV9655_REFA9, is_sxga ? 0x8d : 0xef);
	ret = ov9655_update_bits(client, OV9655_COM16, OV9655_COM16_SCALING, ov9655_framesizes[i].scaling);
	if (ov9655_framesizes[i].scaling) {
		// FIXME: set scaling factors and clock dividers like
		// POIDX, XINDX, YINDX and increase PCKDV and COM24
	}
	val = ov9655_framesizes[i].hstart;
	ret = ov9655_write(client, OV9655_HSTART, val >> 3);	/* image size upper 8 bit */
	ret = ov9655_update_bits(client, OV9655_HREF, OV9655_VREF_START, val);	/* image size lower 3 bit */
// CHECKME: isn't this always hstart + width?
	val = ov9655_framesizes[i].hend;
	ret = ov9655_write(client, OV9655_HSTOP, val >> 3);	/* image size upper 8 bit */
	ret = ov9655_update_bits(client, OV9655_HREF, OV9655_VREF_END, val << 3);		/* image size lower 3 bit */
	val = ov9655_framesizes[i].vstart;
	ret = ov9655_write(client, OV9655_VSTART, val >> 3);	/* image size upper 8 bit */
	ret = ov9655_update_bits(client, OV9655_VREF, OV9655_VREF_START, val);	/* image size lower 3 bit */
	val = ov9655_framesizes[i].vend;
	ret = ov9655_write(client, OV9655_VSTOP, val >> 3);	/* image size upper 8 bit */
	ret = ov9655_update_bits(client, OV9655_VREF, OV9655_VREF_END, val << 3);		/* image size lower 3 bit */

	/* set data format */

	dev_info(&client->dev, "format->code=%08x\n", format->code);

	switch (format->code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_YUV);	/* choose YUV */
		ret = ov9655_update_bits(client, OV9655_TSLB, OV9655_TSLB_YUV_MASK, OV9655_TSLB_UYVY);	/* UYV8 byte order */
		break;
	case MEDIA_BUS_FMT_VYUY8_2X8:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_YUV);	/* choose YUV */
		ret = ov9655_update_bits(client, OV9655_TSLB, OV9655_TSLB_YUV_MASK, OV9655_TSLB_VYUY);	/* VYUY byte order */
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_YUV);	/* choose YUV */
		ret = ov9655_update_bits(client, OV9655_TSLB, OV9655_TSLB_YUV_MASK, OV9655_TSLB_YUYV);	/* YUYV byte order */
		break;
	case MEDIA_BUS_FMT_YVYU8_2X8:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_YUV);	/* choose YUV */
		ret = ov9655_update_bits(client, OV9655_TSLB, OV9655_TSLB_YUV_MASK, OV9655_TSLB_YVYU);	/* YVYU byte order */
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_RGB);	/* choose RGB */
		ret = ov9655_update_bits(client, OV9655_COM15, OV9655_COM15_RGB_MASK, OV9655_COM15_RGB565);	// RGB565
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, OV9655_COM3_SWAP);	/* swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_RGB);	/* choose RGB */
		ret = ov9655_update_bits(client, OV9655_COM15, OV9655_COM15_RGB_MASK, OV9655_COM15_RGB565);	// RGB565
		break;
#if 0
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_SWAP, 0x00);	/* no swap */
		ret = ov9655_update_bits(client, OV9655_COM7, OV9655_COM7_FMT_MASK, OV9655_COM7_RAW);	/* choose raw RGB */
		ret = ov9655_update_bits(client, OV9655_COM15, OV9655_COM15_RGB_MASK, OV9655_COM15_RGB555);	// RGB555
		// FIXME: we must probably adjust pixel clock by factor 2
		break;
#endif
	default:
		// no update. Should have been rejected in ov9655_set_format()
		break;
	}

	dev_info(&client->dev, "format->field=%08x\n", format->field);

	// format->field could ask for some interlacing

	/* set crop */

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

#if 0	// other values that can be changed from reset state

	{ OV9655_COM19, 0x0c },	// UV
	{ OV9655_COM1, 0x03 },	// AEC low bits
	{ OV9655_COM5, 0x61 },	// slam mode & exposure
	{ OV9655_COM8, 0xe0 | OV9655_COM8_AGC | OV9655_COM8_AWB | OV9655_COM8_AEC },
	{ OV9655_COM9, 0x2a },	// agc
	{ OV9655_REG16, 0x24 },
	{ OV9655_MVFP, 0 },	// mirror&flip
	{ OV9655_AEW, 0x3c },
	{ OV9655_AEB, 0x36 },
	{ OV9655_VPT, 0x72 },
	{ OV9655_BBIAS, 0x08 },
	{ OV9655_GBBIAS, 0x08 },
	{ OV9655_PREGAIN, 0x15 },
	{ OV9655_EXHCH, 0x00 },
	{ OV9655_EXHCL, 0x00 },
	{ OV9655_RBIAS, 0x08 },
	{ OV9655_CHLF, 0x00 },
	{ OV9655_AREF2, 0x00 },
	{ OV9655_ADC2, 0x72 },
	{ OV9655_AREF4, 0x57 },
	{ OV9655_COM13, 0x99 },
	{ OV9655_EDGE, 0x02 },	// edge enhancement factor

	{ OV9655_COM17, 0xc1 },	// denoise, edge enhancement, 50 Hz banding filter
	{ OV9655_DNSTH, 0x21 },	// denoise threshold

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

	{ OV9655_COM21, 0x50 },	// digital gain

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
	{ OV9655_LCC7, 0x04 },
	{ OV9655_BD50MAX, 0x05 },
	{ OV9655_BD50, 0x9d },
	{ OV9655_BD60, 0x83 },
	{ OV9655_BD60MAX, 0x07 },
#endif

	return ret;
}

static int ov9655_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	int ret;

	dev_info(&client->dev, "%s(%d)\n", __func__, enable);

	if (!enable)
		/* stop sensor readout */
		return ov9655_update_bits(client, OV9655_COM2,
			OV9655_COM2_SLEEP, OV9655_COM2_SLEEP);

	ret = ov9655_set_params(ov9655);
	if (ret < 0)
		return ret;

	/* take out of soft sleep */
	return ov9655_update_bits(client, OV9655_COM2,
				OV9655_COM2_SLEEP, 0x00);
}

static int ov9655_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov9655_formats))
		return -EINVAL;

	code->code = ov9655_formats[code->index].code;
	return 0;
}

static int ov9655_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	int i = ARRAY_SIZE(ov9655_formats);

	if (fse->index >= ARRAY_SIZE(ov9655_framesizes))
		return -EINVAL;

	while (--i)
		if (fse->code == ov9655_formats[i].code)
			break;

	fse->code = ov9655_formats[i].code;

	fse->min_width  = ov9655_framesizes[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = ov9655_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov9655_get_pad_format(struct ov9655 *ov9655, struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, u32 which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	dev_info(&client->dev, "%s: pad=%u which=%u %s\n", __func__, pad, which,
		which == V4L2_SUBDEV_FORMAT_TRY ?
			"V4L2_SUBDEV_FORMAT_TRY" : "V4L2_SUBDEV_FORMAT_ACTIVE");

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ov9655->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9655->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__ov9655_get_pad_crop(struct ov9655 *ov9655, struct v4l2_subdev_pad_config *cfg,
		     unsigned int pad, u32 which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	dev_info(&client->dev, "%s: pad=%u which=%u %s\n", __func__, pad, which,
		which == V4L2_SUBDEV_FORMAT_TRY ?
			"V4L2_SUBDEV_FORMAT_TRY" : "V4L2_SUBDEV_FORMAT_ACTIVE");

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ov9655->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9655->crop;
	default:
		return NULL;
	}
}

static void ov9655_get_default_format(struct v4l2_mbus_framefmt *mf)
{
	mf->width = ov9655_framesizes[0].width;
	mf->height = ov9655_framesizes[0].height;
	mf->colorspace = ov9655_formats[0].colorspace;
	mf->code = ov9655_formats[0].code;
	mf->field = V4L2_FIELD_NONE;
}

static int ov9655_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *fmt)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	dev_info(&client->dev, "%s\n", __func__);

	fmt->format = *__ov9655_get_pad_format(ov9655, cfg, fmt->pad,
						fmt->which);
	printmbusfmt(client, &fmt->format);

	return 0;
}

static int ov9655_set_format(struct v4l2_subdev *subdev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	unsigned int index = ARRAY_SIZE(ov9655_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	while (--index)
		if (ov9655_formats[index].code == mf->code)
			break;
	// FIXME: error handling if format not found

	dev_info(&client->dev, "%s %d\n", __func__, index);

	mf->colorspace	= ov9655_formats[index].colorspace;
	mf->code	= ov9655_formats[index].code;
	mf->field	= V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		if (!cfg) {
			mf = v4l2_subdev_get_try_format(subdev, cfg, fmt->pad);
			*mf = fmt->format;
		}
	} else
		ov9655->format = fmt->format;

	printmbusfmt(client, &ov9655->format);
	printmbusfmt(client, mf);

	return 0;
}

static int ov9655_get_selection(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	dev_info(&client->dev, "%s\n", __func__);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ov9655_get_pad_crop(ov9655, cfg, sel->pad, sel->which);
	return 0;
}

static int ov9655_set_selection(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);

	dev_info(&client->dev, "%s\n", __func__);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	/* Clamp the crop rectangle boundaries and align them to a multiple of 2
	 * pixels to ensure a GRBG Bayer pattern.
	 */
	rect.left = clamp(ALIGN(sel->r.left, 2), OV9655_COLUMN_START_MIN,
			  OV9655_COLUMN_START_MAX);
	rect.top = clamp(ALIGN(sel->r.top, 2), OV9655_ROW_START_MIN,
			 OV9655_ROW_START_MAX);
	rect.width = clamp_t(unsigned int, ALIGN(sel->r.width, 2),
			   OV9655_WINDOW_WIDTH_MIN,
			   OV9655_WINDOW_WIDTH_MAX);
	rect.height = clamp_t(unsigned int, ALIGN(sel->r.height, 2),
			    OV9655_WINDOW_HEIGHT_MIN,
			    OV9655_WINDOW_HEIGHT_MAX);

	rect.width = min_t(unsigned int, rect.width,
			OV9655_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min_t(unsigned int, rect.height,
			OV9655_PIXEL_ARRAY_HEIGHT - rect.top);

	__crop = __ov9655_get_pad_crop(ov9655, cfg, sel->pad, sel->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/* Reset the output image size if the crop rectangle size has
		 * been modified.
		 */
		__format = __ov9655_get_pad_format(ov9655, cfg, sel->pad,
						    sel->which);
		__format->width = rect.width;
		__format->height = rect.height;
	}

	*__crop = rect;
	sel->r = rect;

	return 0;
}

/*
 * V4L2 subdev control operations
 */

/* private extensions to the MT9P031 driver? */

#define V4L2_CID_BLC_AUTO		(V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_BLC_TARGET_LEVEL	(V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_BLC_ANALOG_OFFSET	(V4L2_CID_USER_BASE | 0x1004)
#define V4L2_CID_BLC_DIGITAL_OFFSET	(V4L2_CID_USER_BASE | 0x1005)

/*
 * NOTE:
 * interesting features of ov9655 for controls:
 *
 * night mode (bit7 of COM11) + Exposure
 * 50/60 Hz filter
 * gamma curve adjustment
 * AGC automatic / manual gain (separate for RGB channels?)
 * AEC automatic / manual exposure
 * BLC automatic / manual black
 * AWB automatic / manual white balance
 * RGB bias?
 * denoise, edge enhancement (COM17)
 * brightness adjust (BRTN)
 * lens correction - may depend not on sensor chip but module (?)
 */

static int ov9655_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9655 *ov9655 =
			container_of(ctrl->handler, struct ov9655, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&ov9655->subdev);
	u16 data;
	int ret;

	dev_info(&client->dev, "%s %08x\n", __func__, ctrl->id);

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	/*
	 * NOTEs:
	 * AEC has a slow and a fast mode (see OV9655_COM5 bit 0) and COM8
	 * AGC has a ceiling register (COM9)
	 * registers 0x20 .. 0x29 control gain, agc, awb etc.
	 * registers 27, 28, 2c are bias values (default 0x80)
	 */

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev, "%s: V4L2_CID_EXPOSURE %08x\n", __func__, ctrl->val);

		ret = ov9655_update_bits(client, OV9655_AECH, OV9655_AECH_AEC, ctrl->val >> 10);	// upper 6 bits
		if (ret < 0)
			return ret;
		ret = ov9655_write(client, OV9655_COM6, ctrl->val >> 2);	// middle 8 bits
		if (ret < 0)
			return ret;
		return ov9655_update_bits(client, OV9655_COM1, OV9655_COM1_AEC, ctrl->val);	// lower 2 bits

	case V4L2_CID_GAIN:
		dev_info(&client->dev, "%s: V4L2_CID_GAIN %u\n", __func__, ctrl->val);

#if 0
		/*
		 * Gain is controlled by 2 analog stages and a digital stage.
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

		/*
		 * for ov9655 it is  similar
		 * OV9655_GAIN + 2 bits in OV9655_VREF
		 * OV9655_BLUE
		 * OV9655_RED
		 * OV9655_GREEN
		 * OV9655_PREGAIN
		 * OV9655_ADC1		analog range adjustment
		 * OV9655_COM15		data output full range
		 * OV9655_COM21		digital gain value
		 */

#endif

		return -EINVAL;

	case V4L2_CID_HFLIP:
		dev_info(&client->dev, "%s: V4L2_CID_HFLIP %u\n", __func__, ctrl->val);

		return ov9655_update_bits(client, OV9655_MVFP, OV9655_MVFP_MIRROR, ctrl->val?OV9655_MVFP_MIRROR:0);

	case V4L2_CID_VFLIP:
		dev_info(&client->dev, "%s: V4L2_CID_VFLIP %u\n", __func__, ctrl->val);

		return ov9655_update_bits(client, OV9655_MVFP, OV9655_MVFP_MIRROR, ctrl->val?OV9655_MVFP_VFLIP:0);

	case V4L2_CID_TEST_PATTERN:
		dev_info(&client->dev, "%s: V4L2_CID_TEST_PATTERN %u\n", __func__, ctrl->val);

		ret = ov9655_update_bits(client, OV9655_COM3, OV9655_COM3_CBAR, ctrl->val?OV9655_COM3_CBAR:0);
		if (ret < 0)
			return ret;

		return ov9655_update_bits(client, OV9655_COM20, OV9655_COM20_CBAR, ctrl->val?OV9655_COM20_CBAR:0);

/* private extensions to the MT9P031 driver? */

#if 0
	case V4L2_CID_BLC_AUTO:
		dev_info(&client->dev, "%s: V4L2_CID_BLC_AUTO %d\n", __func__, ctrl->val);

		ret = ov9655_set_mode2(ov9655,
				ctrl->val ? 0 : MT9P031_READ_MODE_2_ROW_BLC,
				ctrl->val ? MT9P031_READ_MODE_2_ROW_BLC : 0);
		if (ret < 0)
			return ret;

		return  mt9p031_write(client, MT9P031_BLACK_LEVEL_CALIBRATION,
				     ctrl->val ? 0 : MT9P031_BLC_MANUAL_BLC);

	case V4L2_CID_BLC_TARGET_LEVEL:
		dev_info(&client->dev, "%s: V4L2_CID_BLC_TARGET_LEVEL %08x\n", __func__, ctrl->val);

		return  mt9p031_write(client, MT9P031_ROW_BLACK_TARGET,
				     ctrl->val);

	case V4L2_CID_BLC_ANALOG_OFFSET:
		dev_info(&client->dev, "%s: V4L2_CID_BLC_ANALOG_OFFSET %08x\n", __func__, ctrl->val);

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
		dev_info(&client->dev, "%s: V4L2_CID_BLC_DIGITAL_OFFSET %08x\n", __func__, ctrl->val);

		return  mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET,
				     ctrl->val & ((1 << 12) - 1));
#endif

#if 0	// more interesting controls
	case V4L2_CID_AUTO_WHITE_BALANCE:
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_GAMMA:
#endif
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov9655_ctrl_ops = {
	.s_ctrl = ov9655_s_ctrl,
};

static const char * const ov9655_test_pattern_menu[] = {
	"Disabled",
	"Color bars",
};

/* private extensions */

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

/*
 * V4L2 subdev core operations
 */

static int ov9655_set_power(struct v4l2_subdev *subdev, int on)
{
	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	int ret = 0;

	dev_info(&client->dev, "%s on=%d\n", __func__, on);

	mutex_lock(&ov9655->power_lock);

	/*
	 * If the power count is modified from 0 to 1 or from 1 to 0,
	 * update the power state.
	 */
	if (ov9655->power_count == !on) {
		ret = __ov9655_set_power(ov9655, !!on);
		if (ret < 0)
			goto out;
		if (on) {
			ret = ov9655_reset(ov9655);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to reset the camera\n");
				goto out;
				}
			ret = v4l2_ctrl_handler_setup(&ov9655->ctrls);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to choose defaults\n");
				goto out;
				}
		}
	}

	/* Update the power count. */
	ov9655->power_count += on ? 1 : -1;
	WARN_ON(ov9655->power_count < 0);

out:
	mutex_unlock(&ov9655->power_lock);
	return ret;
}

/*
 * V4L2 subdev internal operations
 */

static int ov9655_registered(struct v4l2_subdev *subdev)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct ov9655 *ov9655 = to_ov9655(subdev);
	s32 data;
	int ret;

	ret = __ov9655_set_power(ov9655, 1);
	if (ret < 0) {
		dev_err(&client->dev, "OV9655 power up failed\n");
		return ret;
	}

	/* Read chip manufacturer register */
	data = (ov9655_read(client, OV9655_MIDH) << 8) +
		ov9655_read(client, OV9655_MIDL);

	if (data < 0) {
		dev_err(&client->dev, "OV9655 not detected, can't read manufacturer id\n");
		return -ENODEV;
	}

	if (data != OV9655_CHIP_MID) {
		dev_err(&client->dev,
			"OV9655 not detected, wrong manufacturer 0x%04x\n",
			(unsigned int) data);
		return -ENODEV;
	}

	data =  ov9655_read(client, OV9655_PID);
	if (data != OV9655_CHIP_PID) {
		dev_err(&client->dev,
			"OV9655 not detected, wrong part 0x%02x\n",
			(unsigned int) data);
		return -ENODEV;
	}

	data =  ov9655_read(client, OV9655_REV);
	if (data != OV9655_CHIP_REV4 && data != OV9655_CHIP_REV5) {
		dev_err(&client->dev,
			"OV9655 not detected, wrong version 0x%02x\n",
			(unsigned int) data);
		return -ENODEV;
	}

	__ov9655_set_power(ov9655, 0);

	dev_info(&client->dev, "OV9655 %s detected at address 0x%02x\n",
		 (data == OV9655_CHIP_REV5) ? "REV5" : "REV4",
		 client->addr);

	return ret;
}

static int ov9655_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
//	struct ov9655 *ov9655 = to_ov9655(subdev);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	dev_info(&client->dev, "%s\n", __func__);

	// v4l2_subdev_get_try_crop
	crop = v4l2_subdev_get_try_crop(subdev, fh->pad, 0);
	crop->left = OV9655_COLUMN_START_DEF;
	crop->top = OV9655_ROW_START_DEF;
	crop->width = OV9655_WINDOW_WIDTH_DEF;
	crop->height = OV9655_WINDOW_HEIGHT_DEF;

	// is this overwritten by ov9655_get_default_format ?)
	format = v4l2_subdev_get_try_format(subdev, fh->pad, 0);

	ov9655_get_default_format(format);

	return 0;
}

static struct v4l2_subdev_core_ops ov9655_subdev_core_ops = {
	.s_power		= ov9655_set_power,
	.log_status		= v4l2_ctrl_subdev_log_status,
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_video_ops ov9655_subdev_video_ops = {
	.s_stream       = ov9655_s_stream,
};

static struct v4l2_subdev_pad_ops ov9655_subdev_pad_ops = {
	.enum_mbus_code		= ov9655_enum_mbus_code,
	.enum_frame_size	= ov9655_enum_frame_size,
	.get_fmt		= ov9655_get_format,
	.set_fmt		= ov9655_set_format,
	.get_selection		= ov9655_get_selection,
	.set_selection		= ov9655_set_selection,
};

static struct v4l2_subdev_ops ov9655_subdev_ops = {
	.core   = &ov9655_subdev_core_ops,
	.video  = &ov9655_subdev_video_ops,
	.pad    = &ov9655_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov9655_subdev_internal_ops = {
	.registered	= ov9655_registered,
	.open		= ov9655_open,
};

/*
 * Driver initialization and probing
 */

static int ov9655_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct device_node *np;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ov9655 *ov9655;
	unsigned int i;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);

#if !USEI2C	// SMBUS
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev,
			"I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}
#endif

	ov9655 = devm_kzalloc(&client->dev, sizeof(*ov9655), GFP_KERNEL);
	if (!ov9655)
		return -ENOMEM;

	np = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if (!np)
		return -EINVAL;

	/* CHECKME: does not appear to find endpoint properties */

	if (of_property_read_u32(np, "bus-width", &ov9655->bus_width))
		ov9655->bus_width = 10;
	if (of_property_read_u32(np, "hsync-active", &ov9655->hsync_active))
		ov9655->hsync_active = 0;
	if (of_property_read_u32(np, "vsync-active", &ov9655->vsync_active))
		ov9655->hsync_active = 0;
	if (of_property_read_u32(np, "data-active", &ov9655->data_active))
		ov9655->data_active = 1;
	if (of_property_read_u32(np, "pclk-sample", &ov9655->pclk_sample))
		ov9655->pclk_sample = 0;
	if (of_property_read_u32(np, "pclk-delay", &ov9655->pclk_delay))
		ov9655->pclk_delay = 2;
	if (of_property_read_u32(np, "output-drive", &ov9655->output_drive))
		ov9655->output_drive = 1;

	ov9655->clock_noncontinuous = of_property_read_bool(np, "clock-noncontinuous");
	/* FIXME: allow slave_mode only for REV5 */
	ov9655->slave_mode = of_property_read_bool(np, "slave-mode");

	of_node_put(np);

//	ov9655->model = did->driver_data;	// second parameter from ov9655_id[]

	ov9655->regulators[0].supply = "dvdd";
	ov9655->regulators[1].supply = "dovdd";
	ov9655->regulators[2].supply = "avdd";

	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(ov9655->regulators),
					ov9655->regulators);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to get regulators\n");
		return ret;
	}

	mutex_init(&ov9655->power_lock);

	v4l2_ctrl_handler_init(&ov9655->ctrls, ARRAY_SIZE(ov9655_ctrls) + 6);

	/* register custom controls */

	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_EXPOSURE,
			  OV9655_SHUTTER_WIDTH_MIN,
			  OV9655_SHUTTER_WIDTH_MAX, 1,
			  OV9655_SHUTTER_WIDTH_DEF);
#if 0
	v4l2_ctrl_new_std(&ov9655->ctrls, &ov9655_ctrl_ops,
			  V4L2_CID_GAIN, MT9P031_GLOBAL_GAIN_MIN,
			  MT9P031_GLOBAL_GAIN_MAX, 1, MT9P031_GLOBAL_GAIN_DEF);
#endif

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
		dev_err(&client->dev, "%s: control initialization error %d\n",
		       __func__, ov9655->ctrls.error);
		ret = ov9655->ctrls.error;
		goto err_1;
	}

	ov9655->blc_auto = v4l2_ctrl_find(&ov9655->ctrls, V4L2_CID_BLC_AUTO);
	ov9655->blc_offset = v4l2_ctrl_find(&ov9655->ctrls,
					     V4L2_CID_BLC_DIGITAL_OFFSET);

	v4l2_i2c_subdev_init(&ov9655->subdev, client, &ov9655_subdev_ops);
	ov9655->subdev.internal_ops = &ov9655_subdev_internal_ops;

	ov9655->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov9655->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&ov9655->subdev.entity, 1, &ov9655->pad);
	if (ret < 0)
		goto err_2;

	ov9655->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ov9655->crop.width = OV9655_WINDOW_WIDTH_DEF;
	ov9655->crop.height = OV9655_WINDOW_HEIGHT_DEF;
	ov9655->crop.left = OV9655_COLUMN_START_DEF;
	ov9655->crop.top = OV9655_ROW_START_DEF;

	ov9655->reset = devm_gpiod_get_optional(&client->dev, "reset",
						 GPIOD_OUT_HIGH);
	ov9655->powerdown = devm_gpiod_get_optional(&client->dev, "powerdown",
						 GPIOD_OUT_LOW);

	ov9655->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(ov9655->clk)) {
		dev_err(&client->dev, "Could not get clock\n");
		ret = PTR_ERR(ov9655->clk);
		goto err_3;
	}
	ov9655->xclk_frequency = clk_get_rate(ov9655->clk);
	if (ov9655->xclk_frequency < 10000000 || ov9655->xclk_frequency > 48000000) {
		dev_err(&client->dev, "Clock frequency %lu Hz not in valid range\n",
					ov9655->xclk_frequency);
		ret = -EINVAL;
		goto err_3;
	}

	ret = v4l2_async_register_subdev(&ov9655->subdev);

	if (!ret)
		return 0;

err_3:
	media_entity_cleanup(&ov9655->subdev.entity);
err_2:
	v4l2_ctrl_handler_free(&ov9655->ctrls);
err_1:
	mutex_destroy(&ov9655->power_lock);

	return ret;
}

static int ov9655_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ov9655 *ov9655 = to_ov9655(subdev);

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	v4l2_ctrl_handler_free(&ov9655->ctrls);
	mutex_destroy(&ov9655->power_lock);

	return 0;
}

static const struct i2c_device_id ov9655_id[] = {
	{ "ov9655", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9655_id);

static const struct of_device_id of_ov9655_match_tbl[] = {
	{ .compatible = "ovti,ov9155", },	/* B&W variant - could define different colorspace */
	{ .compatible = "ovti,ov9655", },
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
