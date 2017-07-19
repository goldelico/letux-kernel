/*
 * Omnivision OV9655 CMOS Image Sensor driver
 *
 * Copyright (C) STMicroelectronics SA 2017
 * Author: Hugues Fruchet <hugues.fruchet@st.com> for STMicroelectronics.
 *
 * This driver is forked from OV9650/52 driver (ov9650.c):
 *   Copyright (C) 2013, Sylwester Nawrocki <sylvester.nawrocki@gmail.com>
 *   Copyright (C) 2010, Vladimir Fonov
 *
 * OV9655 initial support is based on a driver written by H. Nikolaus Schaller:
 *   http://git.goldelico.com/?p=gta04-kernel.git;a=shortlog;h=refs/heads/work/hns/video/ov9655
 * OV9655 registers sequence from STM32CubeF7 embedded software, see:
 *   https://developer.mbed.org/teams/ST/code/BSP_DISCO_F746NG/file/e1d9da7fe856/Drivers/BSP/Components/ov9655/ov9655.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/ratelimit.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

#define DRIVER_NAME "ov9655"

/*
 * OV9655 register definitions
 */
#define REG_GAIN		0x00	/* Gain control, AGC[7:0] */
#define REG_BLUE		0x01	/* AWB - Blue chanel gain */
#define REG_RED			0x02	/* AWB - Red chanel gain */
#define REG_VREF		0x03	/* [7:6] - AGC[9:8], [5:3]/[2:0] */
#define  VREF_GAIN_MASK		0xc0	/* - VREF end/start low 3 bits */
#define REG_COM1		0x04
#define  COM1_CCIR656		0x40
#define REG_B_AVE		0x05
#define REG_GB_AVE		0x06
#define REG_GR_AVE		0x07
#define REG_R_AVE		0x08
#define REG_COM2		0x09
#define REG_PID			0x0a	/* Product ID MSB */
#define REG_VER			0x0b	/* Product ID LSB */
#define REG_COM3		0x0c
#define  COM3_COLORBAR		0x80
#define  COM3_SWAP		0x40	/* Doesn't work in RGB */
#define  COM3_RESETB		0x08
#define  COM3_VARIOPIXEL1	0x04
#define  COM3_RGB565		0x00
#define  COM3_SINGLEFRAME	0x01
#define REG_COM5		0x0e	/* System clock options */
#define  COM5_SLAVE_MODE	0x10
#define  COM5_EXPOSURESTEP	0x01
#define REG_COM6		0x0f	/* HREF & ADBLC options */
#define  COM6_BLC_OPTICAL	0x40	/* Optical black */
#define REG_AECH		0x10	/* Exposure value, AEC[9:2] */
#define REG_CLKRC		0x11	/* Clock control */
#define  CLK_EXT		0x40	/* Use external clock directly */
#define  CLK_SCALE		0x3f	/* Mask for internal clock scale */
#define REG_COM7		0x12	/* SCCB reset, output format */
#define  COM7_RESET		0x80
#define  COM7_VGA		0x60
#define  COM7_RAWRGB		0x00	/* different format encoding */
#define  COM7_RAWRGBINT		0x01
#define  COM7_YUV		0x02
#define  COM7_RGB		0x03
#define REG_COM8		0x13	/* AGC/AEC options */
#define  COM8_FASTAEC		0x80	/* Enable fast AGC/AEC */
#define  COM8_AECSTEP		0x40	/* Unlimited AEC step size */
#define  COM8_BFILT		0x20	/* Band filter enable */
#define  COM8_AGC		0x04	/* Auto gain enable */
#define  COM8_AWB		0x02	/* White balance enable */
#define  COM8_AEC		0x01	/* Auto exposure enable */
#define REG_COM9		0x14	/* Gain ceiling */
#define  COM9_GAIN_CEIL_MASK	0x70
#define  COM9_GAIN_CEIL_16X	0x30
#define  COM9_EXPTIMING		0x08
#define  COM9_VSYNCDROP		0x04
#define  COM9_AECDROP		0x02
#define REG_COM10		0x15	/* PCLK, HREF, HSYNC signals polarity */
#define  COM10_SLAVE_PIN	0x80	/* SLHS/SLVS instead of RESETB/PWDN */
#define  COM10_HSYNC		0x40	/* HSYNC instead of HREF */
#define  COM10_PCLK_HB		0x20	/* Suppress PCLK on horiz blank */
#define  COM10_PCLK_REV		0x10	/* PCLK reverse */
#define  COM10_HREF_REV		0x08	/* Reverse HREF */
#define  COM10_RESET_OPTION	0x04	/* Reset signal end point */
#define  COM10_VS_NEG		0x02	/* VSYNC negative */
#define  COM10_HS_NEG		0x01	/* HSYNC negative */
#define REG16			0x16	/* dummy frame and blanking */
#define   REG16_DUMMY_8		0x20	/* dummy frame when gain > 8 */
#define REG_HSTART		0x17	/* Horiz start high bits */
#define REG_HSTOP		0x18	/* Horiz stop high bits */
#define REG_VSTART		0x19	/* Vert start high bits */
#define REG_VSTOP		0x1a	/* Vert stop high bits */
#define REG_PSHFT		0x1b	/* Pixel delay after HREF */
#define REG_MIDH		0x1c	/* Manufacturer ID MSB */
#define REG_MIDL		0x1d	/* Manufufacturer ID LSB */
#define REG_MVFP		0x1e	/* Image mirror/flip */
#define  MVFP_MIRROR		0x20	/* Mirror image */
#define  MVFP_FLIP		0x10	/* Vertical flip */
#define REG_BOS			0x20	/* B channel Offset */
#define REG_GBOS		0x21	/* Gb channel Offset */
#define REG_GROS		0x22	/* Gr channel Offset */
#define REG_ROS			0x23	/* R channel Offset */
#define REG_AEW			0x24	/* AGC upper limit */
#define REG_AEB			0x25	/* AGC lower limit */
#define REG_VPT			0x26	/* AGC/AEC fast mode op region */
#define REG_BBIAS		0x27	/* B channel output bias */
#define REG_GBBIAS		0x28	/* Gb channel output bias */
#define REG_PREGAIN		0x29
#define REG_EXHCH		0x2a	/* Dummy pixel insert MSB */
#define REG_EXHCL		0x2b	/* Dummy pixel insert LSB */
#define REG_RBIAS		0x2c	/* R channel output bias */
#define REG_ADVFL		0x2d	/* LSB of dummy line insert */
#define REG_ADVFH		0x2e	/* MSB of dummy line insert */
#define REG_YAVE		0x2f	/* Y/G channel average value */
#define REG_HSYST		0x30	/* HSYNC rising edge delay LSB*/
#define REG_HSYEN		0x31	/* HSYNC falling edge delay LSB*/
#define REG_HREF		0x32	/* HREF pieces */
#define REG_CHLF		0x33	/* Array current control */
#define REG_AREF1		0x34	/* Array reference control */
#define REG_AREF2		0x35	/* Array reference control */
#define REG_AREF3		0x36	/* Array reference control */
#define REG_ADC1		0x37	/* ADC Control 1 (Range adjustment) */
#define REG_ADC2		0x38	/* ADC Control 2 (Range adjustment) */
#define REG_AREF4		0x39	/* Array reference control */
#define REG_TSLB		0x3a	/* YUVU format */
#define  TSLB_PCLKDELAY2NS	0x40
#define  TSLB_PCLKDELAY4NS	0x80
#define  TSLB_PCLKDELAY6NS	0xc0
#define  TSLB_OUTREVERSE	0x20
#define  TSLB_FIXEDUV		0x10
#define  TSLB_YUYV_MASK		0x0c	/* UYVY or VYUY - see com13 */
#define  TSLB_YUYV		0x00
#define  TSLB_YVYU		0x04
#define  TSLB_VYUY		0x08
#define  TSLB_UYVY		0x0c
#define  TSLB_BANDINGAUTO	0x02
#define REG_COM11		0x3b	/* Night mode, banding filter enable */
#define  COM11_NIGHT		0x80	/* Night mode enable */
#define  COM11_NMFR		0x60	/* Two bit NM frame rate */
#define  COM11_BANDING		0x01	/* Banding filter */
#define  COM11_AEC_REF_MASK	0x18	/* AEC reference area selection */
#define REG_COM12		0x3c	/* HREF option, UV average */
#define  COM12_HREF		0x80	/* HREF always */
#define REG_COM13		0x3d	/* Gamma selection, Color matrix en. */
#define  COM13_GAMMA		0x80	/* Gamma enable */
#define  COM13_UVSAT		0x40	/* UV saturation auto adjustment */
#define  COM13_Y_DELAY		0x08	/* Delay Y channel */
#define  COM13_UVSWAP		0x01	/* V before U - w/TSLB */
#define REG_COM14		0x3e	/* pixel correction/zoom ON/OFF sel. */
#define  COM14_BLACK_PIX	0x08	/* Black pixel correction */
#define  COM14_WHITE_PIX	0x04	/* White pixel correction */
#define  COM14_ZOOM		0x02	/* Zoom function ON */
#define REG_EDGE		0x3f	/* Edge enhancement factor */
#define  EDGE_FACTOR_MASK	0x0f
#define REG_COM15		0x40	/* Output range, RGB 555/565 */
#define  COM15_R10F0		0x00	/* Data range 10 to F0 */
#define  COM15_R01FE		0x80	/* 01 to FE */
#define  COM15_R00FF		0xc0	/* 00 to FF */
#define  COM15_RGB565		0x10	/* RGB565 output */
#define  COM15_RGB555		0x30	/* RGB555 output */
#define  COM15_SWAPRB		0x04	/* Swap R&B */
#define REG_COM16		0x41	/* Color matrix coeff options */
#define REG_COM17		0x42	/* Denoise, edge, auto gain, ... */
#define   COM17_EDGE_AUTO	0x40	/* Edge auto */
#define   COM17_DENOISE_AUTO	0x80	/* Denoise auto */
#define REG_RSVD(__n)	(0x43 + (__n) - 1) /* reserved but used... */
/* n = 1...9, 0x4f..0x57 */
#define REG_MTX(__n)		(0x4f + (__n) - 1)
#define REG_MTXS		0x58
#define REG_AWBOP(__n)		(0x59 + (__n) - 1) /* AWB control options */
#define REG_BLMT		0x5F	/* AWB Blue Component Gain Limit */
#define REG_RLMT		0x60	/* AWB Red Component Gain Limit */
#define REG_GLMT		0x61	/* AWB Green Component Gain Limit */
/* Lens Correction Option 1...5, __n = 0...5 */
#define REG_LCC(__n)		(0x62 + (__n) - 1)
#define  LCC5_LCC_ENABLE	0x01	/* LCC5, enable lens correction */
#define  LCC5_LCC_COLOR		0x04
#define REG_MANU		0x67	/* Manual U value */
#define REG_MANV		0x68	/* Manual V value */
#define REG_HV			0x69	/* Manual banding filter MSB */
#define REG_MBD			0x6a	/* Manual banding filter value */
#define REG_DBLV		0x6b	/* PLL, DVDD regu bypass, bandgap */
#define  DBLV_BANDGAP		0x0a	/* default value */
#define  DBLV_LDO_BYPASS	0x10
#define  DBLV_PLL_BYPASS	0x00
#define  DBLV_PLL_4X		0x40
#define  DBLV_PLL_6X		0x80
#define  DBLV_PLL_8X		0xc0
#define REG_GSP			0x6c	/* Gamma curve */
#define  GSP_LEN		15
#define REG_DNSTH		0x70	/* De-noise Function Threshold Adj. */
#define REG_POIDX		0x72	/* Pixel output index */
#define REG_PCKDV		0x73	/* Pixel Clock Output Selection */
#define REG_XINDX		0x74	/* Horizontal Scaling Down Coeff. */
#define REG_YINDX		0x75	/* Vertical Scaling Down Coeff. */
#define REG_SLOP		0x7A	/* Gamma Curve Highest Segment Slope */
#define REG_GAM(__n)	(0x7B + (__n) - 1)	/* Gamma curve */
#define REG_GST			0x7c	/* Gamma curve */
#define  GST_LEN		15
#define REG_COM18		0x8b	/* Zoom mode in VGA */
#define REG_COM19		0x8c	/* UV adjustment */
#define REG_COM20		0x8d
#define  COM20_TEST_MODE	0x10
#define REG_DM_LNL		0x92	/* Dummy line low 8 bits */
#define REG_DM_LNH		0x93	/* Dummy line high 8 bits */
#define REG_LCCFB		0x9d	/* Lens Correction B channel */
#define REG_LCCFR		0x9e	/* Lens Correction R channel */
#define REG_DBLC_GB		0x9f	/* Digital BLC GB chan offset */
#define REG_DBLC_GR		0xa0	/* Digital BLC GR chan offset */
#define REG_AECHM		0xa1	/* Exposure value - bits AEC[15:10] */
#define REG_BD50ST		0xa2	/* Banding filter value for 50Hz */
#define REG_BD60ST		0xa3	/* Banding filter value for 60Hz */
#define REG_COM21		0xa4	/* Digital gain */
#define REG_AWB_GREEN		0xa6	/* AWB green */
#define REG_REF_A8		0xa8	/* Analog Reference Control */
#define REG_REF_A9		0xa9	/* Analog Reference Control */
#define REG_BLC(__n)	(0xac + (__n) - 1) /* Black Level Control */
#define REG_CTRLB4		0xb4	/* UV adjustment */
#define REG_ADBOFF		0xbc	/* ADC B channel offset setting */
#define REG_ADROFF		0xbd	/* ADC R channel offset setting */
#define REG_ADGBOFF		0xbe	/* ADC Gb channel offset setting */
#define REG_ADGEOFF		0xbf	/* ADC Gr channel offset setting */
#define REG_COM24		0xc7	/* Pixel clock frequency selection */
#define REG_NULL		0xff	/* Array end token */

#define DEF_CLKRC		0x80

#define OV9655_ID(_msb, _lsb)	((_msb) << 8 | (_lsb))
#define OV9655V4_ID		0x9656
#define OV9655V5_ID		0x9657

struct ov9655_ctrls {
	struct v4l2_ctrl_handler handler;
	u8 update;
};

struct i2c_rv {
	u8 addr;
	u8 value;
};

struct ov9655_framesize {
	u16 width;
	u16 height;
	u16 max_exp_lines;
	const struct i2c_rv *regs;
};

struct ov9655_interval {
	struct v4l2_fract interval;
	/* Maximum resolution for this interval */
	struct v4l2_frmsize_discrete size;
	u8 clkrc_div;
};

struct ov9655 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	enum v4l2_mbus_type bus_type;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_powerdown;
	struct clk *clk;
	struct regulator *avdd;

	/* Protects the struct fields below */
	struct mutex lock;

	struct i2c_client *client;

	/* Exposure row interval in us */
	unsigned int exp_row_interval;

	unsigned short id;
	const struct ov9655_framesize *frame_size;
	/* YUYV sequence (pixel format) control register */
	u8 tslb_reg;
	struct v4l2_mbus_framefmt format;

	struct ov9655_ctrls ctrls;
	/* Pointer to frame rate control data structure */
	const struct ov9655_interval *fiv;

	int streaming;
	int power;

	u8 apply_frame_fmt;
};

struct ov9655_pixfmt {
	u32 code;
	u32 colorspace;
	/* REG_TSLB value, only bits [3:2] may be set. */
	u8 tslb_reg;
};

/*
 * OV9655 init sequence
 */
static const struct i2c_rv ov9655_init_regs[] = {
	{ REG_GAIN, 0x00 },
	{ REG_BLUE, 0x80 },
	{ REG_RED, 0x80 },
	{ REG_VREF, 0x02 },
	{ REG_COM1, 0x03 },
	{ REG_COM2, 0x01 },/* Output drive x2 */
	{ REG_COM3, COM3_RGB565 },/* Output drive x2, RGB565 */
	{ REG_COM5, 0x60 | COM5_EXPOSURESTEP },/* 0x60 ? */
	{ REG_COM6, COM6_BLC_OPTICAL },
	{ REG_CLKRC, 0x01 },/* F(internal clk) = F(input clk) / 2 */
	{ REG_COM7, COM7_VGA | COM7_YUV },
	{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP |
			COM8_AGC | COM8_AWB | COM8_AEC },
	{ REG_COM9, COM9_GAIN_CEIL_16X | COM9_EXPTIMING |
			COM9_AECDROP },
	{ REG16, REG16_DUMMY_8 | 0x4 },
	{ REG_HSTART, 0x18 },
	{ REG_HSTOP, 0x04 },
	{ REG_VSTART, 0x01 },
	{ REG_VSTOP, 0x81 },
	{ REG_MVFP, 0x00 },/* No mirror/flip */
	{ REG_AEW, 0x3c },
	{ REG_AEB, 0x36 },
	{ REG_VPT, 0x72 },
	{ REG_BBIAS, 0x08 },
	{ REG_GBBIAS, 0x08 },
	{ REG_PREGAIN, 0x15 },
	{ REG_EXHCH, 0x00 },
	{ REG_EXHCL, 0x00 },
	{ REG_RBIAS, 0x08 },
	{ REG_HREF, 0x12 },/* QVGA */
	{ REG_CHLF, 0x00 },
	{ REG_AREF1, 0x3f },
	{ REG_AREF2, 0x00 },
	{ REG_AREF3, 0x3a },
	{ REG_ADC2, 0x72 },
	{ REG_AREF4, 0x57 },
	{ REG_TSLB, TSLB_PCLKDELAY6NS | TSLB_UYVY },
	{ REG_COM11, 0x04 },/* 0x04 ? */
	{ REG_COM13, COM13_GAMMA | 0x10 |
			COM13_Y_DELAY | COM13_UVSWAP },/* 0x10 ? */
	{REG_COM14, COM14_ZOOM }, /* QVGA */
	{ REG_EDGE, 0xc1 },
	{ REG_COM15, COM15_R00FF },/* Full range output */
	{ REG_COM16, 0x41 },/* 0x41 ? */
	{ REG_COM17, COM17_EDGE_AUTO |
			COM17_DENOISE_AUTO },
	{ REG_RSVD(1), 0x0a },
	{ REG_RSVD(2), 0xf0 },
	{ REG_RSVD(3), 0x46 },
	{ REG_RSVD(4), 0x62 },
	{ REG_RSVD(5), 0x2a },
	{ REG_RSVD(6), 0x3c },
	{ REG_RSVD(7), 0xfc },
	{ REG_RSVD(8), 0xfc },
	{ REG_RSVD(9), 0x7f },
	{ REG_RSVD(10), 0x7f },
	{ REG_RSVD(11), 0x7f },
	{ REG_MTX(1), 0x98 },
	{ REG_MTX(2), 0x98 },
	{ REG_MTX(3), 0x00 },
	{ REG_MTX(4), 0x28 },
	{ REG_MTX(5), 0x70 },
	{ REG_MTX(6), 0x98 },
	{ REG_MTXS, 0x1a },
	{ REG_AWBOP(1), 0x85 },
	{ REG_AWBOP(2), 0xa9 },
	{ REG_AWBOP(3), 0x64 },
	{ REG_AWBOP(4), 0x84 },
	{ REG_AWBOP(5), 0x53 },
	{ REG_AWBOP(6), 0x0e },
	{ REG_BLMT, 0xf0 },
	{ REG_RLMT, 0xf0 },
	{ REG_GLMT, 0xf0 },
	{ REG_LCC(1), 0x00 },
	{ REG_LCC(2), 0x00 },
	{ REG_LCC(3), 0x02 },
	{ REG_LCC(4), 0x20 },
	{ REG_LCC(5), 0x00 },
	{ 0x69, 0x0a },/* Reserved... */
	{ REG_DBLV, DBLV_PLL_4X | DBLV_LDO_BYPASS |
			DBLV_BANDGAP },
	{ 0x6c, 0x04 },/* Reserved... */
	{ 0x6d, 0x55 },/* Reserved... */
	{ 0x6e, 0x00 },/* Reserved... */
	{ 0x6f, 0x9d },/* Reserved... */
	{ REG_DNSTH, 0x21 },
	{ 0x71, 0x78 },/* Reserved... */
	{ REG_POIDX, 0x11 },/* QVGA */
	{ REG_PCKDV, 0x01 },/* QVGA */
	{ REG_XINDX, 0x10 },
	{ REG_YINDX, 0x10 },
	{ 0x76, 0x01 },/* Reserved... */
	{ 0x77, 0x02 },/* Reserved... */
	{ 0x7A, 0x12 },/* Reserved... */
	{ REG_GAM(1), 0x08 },
	{ REG_GAM(2), 0x16 },
	{ REG_GAM(3), 0x30 },
	{ REG_GAM(4), 0x5e },
	{ REG_GAM(5), 0x72 },
	{ REG_GAM(6), 0x82 },
	{ REG_GAM(7), 0x8e },
	{ REG_GAM(8), 0x9a },
	{ REG_GAM(9), 0xa4 },
	{ REG_GAM(10), 0xac },
	{ REG_GAM(11), 0xb8 },
	{ REG_GAM(12), 0xc3 },
	{ REG_GAM(13), 0xd6 },
	{ REG_GAM(14), 0xe6 },
	{ REG_GAM(15), 0xf2 },
	{ 0x8a, 0x24 },/* Reserved... */
	{ REG_COM19, 0x80 },
	{ 0x90, 0x7d },/* Reserved... */
	{ 0x91, 0x7b },/* Reserved... */
	{ REG_LCCFB, 0x02 },
	{ REG_LCCFR, 0x02 },
	{ REG_DBLC_GB, 0x7a },
	{ REG_DBLC_GR, 0x79 },
	{ REG_AECHM, 0x40 },
	{ REG_COM21, 0x50 },
	{ 0xa5, 0x68 },/* Reserved... */
	{ REG_AWB_GREEN, 0x4a },
	{ REG_REF_A8, 0xc1 },
	{ REG_REF_A9, 0xef },
	{ 0xaa, 0x92 },/* Reserved... */
	{ 0xab, 0x04 },/* Reserved... */
	{ REG_BLC(1), 0x80 },
	{ REG_BLC(2), 0x80 },
	{ REG_BLC(3), 0x80 },
	{ REG_BLC(4), 0x80 },
	{ REG_BLC(7), 0xf2 },
	{ REG_BLC(8), 0x20 },
	{ REG_CTRLB4, 0x20 },
	{ 0xb5, 0x00 },/* Reserved... */
	{ 0xb6, 0xaf },/* Reserved... */
	{ 0xb6, 0xaf },/* Reserved... */
	{ 0xbb, 0xae },/* Reserved... */
	{ REG_ADBOFF, 0x7f },
	{ REG_ADROFF, 0x7f },
	{ REG_ADGBOFF, 0x7f },
	{ REG_ADGEOFF, 0x7f },
	{ REG_ADGEOFF, 0x7f },
	{ 0xc0, 0xaa },/* Reserved... */
	{ 0xc1, 0xc0 },/* Reserved... */
	{ 0xc2, 0x01 },/* Reserved... */
	{ 0xc3, 0x4e },/* Reserved... */
	{ 0xc6, 0x05 },/* Reserved... */
	{ REG_COM24, 0x81 },/* QVGA */
	{ 0xc9, 0xe0 },/* Reserved... */
	{ 0xca, 0xe8 },/* Reserved... */
	{ 0xcb, 0xf0 },/* Reserved... */
	{ 0xcc, 0xd8 },/* Reserved... */
	{ 0xcd, 0x93 },/* Reserved... */
	{ REG_COM7, COM7_VGA | COM7_RGB },
	{ REG_COM15, COM15_RGB565 },
	{ REG_NULL, 0}
};

static const struct i2c_rv ov9655_qvga_regs[] = {
	{ REG_HREF, 0x12 },
	{ REG_COM14, COM14_ZOOM },
	{ REG_POIDX, 0x11 },
	{ REG_PCKDV, 0x01 },
	{ REG_COM24, 0x81 },
	{ REG_NULL, 0}
};

static const struct i2c_rv ov9655_qqvga_regs[] = {
	{ REG_HREF, 0xa4 },
	{ REG_COM14, COM14_BLACK_PIX | COM14_WHITE_PIX |
			COM14_ZOOM },
	{ REG_POIDX, 0x22 },
	{ REG_PCKDV, 0x02 },
	{ REG_COM24, 0x82 },
	{ REG_NULL, 0}
};

static const struct i2c_rv ov9655_vga_regs[] = {
	{ REG_GAIN, 0x11 },
	{ REG_VREF, 0x12 },
	{ REG_B_AVE, 0x2e },
	{ REG_GB_AVE, 0x2e },
	{ REG_GR_AVE, 0x2e },
	{ REG_R_AVE, 0x2e },
	{ REG_COM6, 0x48 },
	{ REG_AECH, 0x7b },
	{ REG_CLKRC, 0x03 },
	{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT |
			COM8_AGC | COM8_AWB | COM8_AEC },
	{ REG_HSTART, 0x16 },
	{ REG_HSTOP, 0x02 },
	{ REG_VSTART, 0x01 },
	{ REG_VSTOP, 0x3d },
	{ REG_MVFP, 0x04 },
	{ REG_YAVE, 0x2e },
	{ REG_HREF, 0xff },
	{ REG_AREF1, 0x3d },
	{ REG_AREF3, 0xfa },
	{ REG_TSLB, 0xcc },
	{ REG_COM11, 0xcc },
	{ REG_COM14, 0x0c },
	{ REG_EDGE, 0x82 },
	{ REG_COM15, COM15_R00FF | COM15_RGB565 },/* full range */
	{ REG_COM16, 0x40 },
	{ REG_RSVD(1), 0x14 },
	{ REG_RSVD(2), 0xf0 },
	{ REG_RSVD(3), 0x46 },
	{ REG_RSVD(4), 0x62 },
	{ REG_RSVD(5), 0x2a },
	{ REG_RSVD(6), 0x3c },
	{ REG_RSVD(8), 0xe9 },
	{ REG_RSVD(9), 0xdd },
	{ REG_RSVD(10), 0xdd },
	{ REG_RSVD(11), 0xdd },
	{ REG_RSVD(12), 0xdd },
	{ REG_LCC(1), 0x00 },
	{ REG_LCC(2), 0x00 },
	{ REG_LCC(3), 0x02 },
	{ REG_LCC(4), 0x20 },
	{ REG_LCC(5), 0x01 },
	{ REG_GSP, 0x0c },
	{ 0x6f, 0x9e },/* Reserved... */
	{ REG_DNSTH, 0x06 },
	{ REG_POIDX, 0x00 },
	{ REG_PCKDV, 0x00 },
	{ REG_XINDX, 0x3a },
	{ REG_YINDX, 0x35 },
	{ REG_SLOP, 0x20 },
	{ REG_GAM(1), 0x1c },
	{ REG_GAM(2), 0x28 },
	{ REG_GAM(3), 0x3c },
	{ REG_GAM(4), 0x5a },
	{ REG_GAM(5), 0x68 },
	{ REG_GAM(6), 0x76 },
	{ REG_GAM(7), 0x80 },
	{ REG_GAM(8), 0x88 },
	{ REG_GAM(9), 0x8f },
	{ REG_GAM(10), 0x96 },
	{ REG_GAM(11), 0xa3 },
	{ REG_GAM(12), 0xaf },
	{ REG_GAM(13), 0xc4 },
	{ REG_GAM(14), 0xd7 },
	{ REG_GAM(15), 0xe8 },
	{ 0x8a, 0x23 },/* Reserved... */
	{ REG_COM19, 0x8d },
	{ 0x90, 0x92 },/* Reserved... */
	{ 0x91, 0x92 },/* Reserved... */
	{ REG_DBLC_GB, 0x90 },
	{ REG_DBLC_GR, 0x90 },
	{ REG_AWB_GREEN, 0x40 },
	{ REG_ADBOFF, 0x02 },
	{ REG_ADROFF, 0x01 },
	{ REG_ADGBOFF, 0x02 },
	{ REG_ADGEOFF, 0x01 },
	{ 0xc1, 0xc8 },/* Reserved... */
	{ 0xc6, 0x85 },/* Reserved... */
	{ REG_COM24, 0x80 },
	{ REG_NULL, 0}
};

static const struct ov9655_framesize ov9655_framesizes[] = {
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs		= ov9655_vga_regs,
		.max_exp_lines	= 498,
	}, {
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.regs		= ov9655_qvga_regs,
		.max_exp_lines	= 248,
	},
	{
		.width		= QQVGA_WIDTH,
		.height		= QQVGA_HEIGHT,
		.regs		= ov9655_qqvga_regs,
		.max_exp_lines	= 124,
	},
};

static const struct ov9655_pixfmt ov9655_formats[] = {
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, 0x08},
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov9655, ctrls.handler)->sd;
}

static inline struct ov9655 *to_ov9655(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9655, sd);
}

static int ov9655_read(struct i2c_client *client, u8 addr, u8 *val)
{
	u8 buf = addr;
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
			*val = buf;
	}

	v4l2_dbg(2, debug, client, "%s: 0x%02x @ 0x%02x. (%d)\n",
		 __func__, *val, addr, ret);

	return ret == 1 ? 0 : ret;
}

static int ov9655_write(struct i2c_client *client, u8 addr, u8 val)
{
	u8 buf[2] = { addr, val };

	int ret = i2c_master_send(client, buf, 2);

	v4l2_dbg(2, debug, client, "%s: 0x%02x @ 0x%02X (%d)\n",
		 __func__, val, addr, ret);

	return ret == 2 ? 0 : ret;
}

static int ov9655_write_array(struct i2c_client *client,
			      const struct i2c_rv *regs)
{
	int i, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov9655_write(client, regs[i].addr, regs[i].value);

	return ret;
}

static void __ov9655_set_power(struct ov9655 *ov9655, int on)
{
	int ret;

	if (on) {
		/* Bring up the power supply */
		ret = regulator_enable(ov9655->avdd);
		if (ret)
			dev_warn(&ov9655->client->dev,
				 "Failed to enable analog power (%d)\n", ret);
		msleep(25);

		/* Enable clock */
		ret = clk_prepare_enable(ov9655->clk);
		if (ret)
			dev_warn(&ov9655->client->dev,
				 "Failed to enable clock (%d)\n", ret);
		msleep(25);

		gpiod_set_value_cansleep(ov9655->gpio_powerdown, 0);
		gpiod_set_value_cansleep(ov9655->gpio_reset, 0);
		msleep(25);
	} else {
		gpiod_set_value_cansleep(ov9655->gpio_reset, 1);
		gpiod_set_value_cansleep(ov9655->gpio_powerdown, 1);

		clk_disable_unprepare(ov9655->clk);
		regulator_disable(ov9655->avdd);
	}

	ov9655->streaming = 0;
}

static int ov9655_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov9655 *ov9655 = to_ov9655(sd);
	struct i2c_client *client = ov9655->client;
	int ret = 0;

	v4l2_dbg(1, debug, client, "%s: on: %d\n", __func__, on);

	mutex_lock(&ov9655->lock);
	if (ov9655->power == !on) {
		__ov9655_set_power(ov9655, on);
		if (on) {
			ret = ov9655_write_array(client,
						 ov9655_init_regs);
			ov9655->apply_frame_fmt = 1;
			ov9655->ctrls.update = 1;
		}
	}
	if (!ret)
		ov9655->power += on ? 1 : -1;

	WARN_ON(ov9655->power < 0);
	mutex_unlock(&ov9655->lock);
	return ret;
}

/*
 * V4L2 controls
 */
static int ov9655_set_test_pattern(struct ov9655 *ov9655, int value)
{
	int ret;
	u8 reg;
	u8 addr = REG_COM3;
	u8 mask = COM3_COLORBAR;

	ret = ov9655_read(ov9655->client, addr, &reg);
	if (ret < 0)
		return ret;
	reg = value ? reg | mask : reg & ~mask;
	return ov9655_write(ov9655->client, addr, reg);
}

static int ov9655_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov9655 *ov9655 = to_ov9655(sd);
	int ret = -EINVAL;

	v4l2_dbg(1, debug, sd, "s_ctrl: %s, value: %d. power: %d\n",
		 ctrl->name, ctrl->val, ov9655->power);

	mutex_lock(&ov9655->lock);
	/*
	 * If the device is not powered up now postpone applying control's
	 * value to the hardware, until it is ready to accept commands.
	 */
	if (ov9655->power == 0) {
		mutex_unlock(&ov9655->lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ret = ov9655_set_test_pattern(ov9655, ctrl->val);
		break;
	}

	mutex_unlock(&ov9655->lock);
	return ret;
}

static const struct v4l2_ctrl_ops ov9655_ctrl_ops = {
	.s_ctrl	= ov9655_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Color bars",
};

static int ov9655_initialize_controls(struct ov9655 *ov9655)
{
	const struct v4l2_ctrl_ops *ops = &ov9655_ctrl_ops;
	struct ov9655_ctrls *ctrls = &ov9655->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0)
		return ret;

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(test_pattern_menu) - 1, 0, 0,
				     test_pattern_menu);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ov9655->sd.ctrl_handler = hdl;
	return 0;
}

/*
 * V4L2 subdev video and pad level operations
 */
static void ov9655_get_default_format(struct v4l2_mbus_framefmt *mf)
{
	mf->width = ov9655_framesizes[0].width;
	mf->height = ov9655_framesizes[0].height;
	mf->colorspace = ov9655_formats[0].colorspace;
	mf->code = ov9655_formats[0].code;
	mf->field = V4L2_FIELD_NONE;
}

static int ov9655_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov9655_formats))
		return -EINVAL;

	code->code = ov9655_formats[code->index].code;
	return 0;
}

static int ov9655_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int ov9655_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov9655 *ov9655 = to_ov9655(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		fmt->format = *mf;
		return 0;
	}

	mutex_lock(&ov9655->lock);
	fmt->format = ov9655->format;
	mutex_unlock(&ov9655->lock);

	return 0;
}

static void __ov9655_try_frame_size(struct v4l2_mbus_framefmt *mf,
				    const struct ov9655_framesize **size)
{
	const struct ov9655_framesize *fsize = &ov9655_framesizes[0],
		*match = NULL;
	int i = ARRAY_SIZE(ov9655_framesizes);
	unsigned int min_err = UINT_MAX;
//FIXME change to upper bounds
	while (i--) {
		int w_err = (fsize->width - mf->width);
		int h_err = (fsize->height - mf->height);
		int err = w_err + h_err;

		if ((w_err >= 0) && (h_err >= 0) && (err < min_err)) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}
	if (!match)
		match = &ov9655_framesizes[0];
	mf->width  = match->width;
	mf->height = match->height;
	if (size)
		*size = match;
}

static int ov9655_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	unsigned int index = ARRAY_SIZE(ov9655_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct ov9655 *ov9655 = to_ov9655(sd);
	const struct ov9655_framesize *size = NULL;
	int ret = 0;

	__ov9655_try_frame_size(mf, &size);

	while (--index)
		if (ov9655_formats[index].code == mf->code)
			break;

	mf->colorspace	= ov9655_formats[index].colorspace;
	mf->code	= ov9655_formats[index].code;
	mf->field	= V4L2_FIELD_NONE;

	mutex_lock(&ov9655->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		if (cfg != NULL) {
			mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
			*mf = fmt->format;
		}
	} else {
		if (ov9655->streaming) {
			ret = -EBUSY;
		} else {
			ov9655->frame_size = size;
			ov9655->format = fmt->format;
			ov9655->tslb_reg = ov9655_formats[index].tslb_reg;
			ov9655->apply_frame_fmt = 1;
		}
	}
	mutex_unlock(&ov9655->lock);

	return ret;
}

static int ov9655_set_frame_size(struct ov9655 *ov9655)
{
	v4l2_dbg(1, debug, ov9655->client, "%s\n", __func__);

	return ov9655_write_array(ov9655->client,
				  ov9655->frame_size->regs);
}

static int ov9655_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9655 *ov9655 = to_ov9655(sd);
	struct ov9655_ctrls *ctrls = &ov9655->ctrls;
	int ret = 0;

	v4l2_dbg(1, debug, client, "%s: on: %d\n", __func__, on);

	mutex_lock(&ov9655->lock);
	if (ov9655->streaming == !on) {
		if (on)
			ret = ov9655_set_frame_size(ov9655);

		if (!ret && ctrls->update) {
			/*
			 * ov9655_s_ctrl callback takes the mutex
			 * so it needs to be released here.
			 */
			mutex_unlock(&ov9655->lock);
			ret = v4l2_ctrl_handler_setup(&ctrls->handler);

			mutex_lock(&ov9655->lock);
			if (!ret)
				ctrls->update = 0;
		}
		if (!ret)
			ret = ov9655_write(client, REG_COM2, on ? 0x01 : 0x11);
	}
	if (!ret)
		ov9655->streaming += on ? 1 : -1;

	WARN_ON(ov9655->streaming < 0);
	mutex_unlock(&ov9655->lock);

	return ret;
}

/*
 * V4L2 subdev internal operations
 */
static int ov9655_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf = v4l2_subdev_get_try_format(sd, fh->pad, 0);

	ov9655_get_default_format(mf);
	return 0;
}

static const struct v4l2_subdev_pad_ops ov9655_pad_ops = {
	.enum_mbus_code = ov9655_enum_mbus_code,
	.enum_frame_size = ov9655_enum_frame_sizes,
	.get_fmt = ov9655_get_fmt,
	.set_fmt = ov9655_set_fmt,
};

static const struct v4l2_subdev_video_ops ov9655_video_ops = {
	.s_stream = ov9655_s_stream,
};

static const struct v4l2_subdev_internal_ops ov9655_sd_internal_ops = {
	.open = ov9655_open,
};

static const struct v4l2_subdev_core_ops ov9655_core_ops = {
	.s_power = ov9655_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ov9655_subdev_ops = {
	.core = &ov9655_core_ops,
	.pad = &ov9655_pad_ops,
	.video = &ov9655_video_ops,
};

static int ov9655_detect_sensor(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9655 *ov9655 = to_ov9655(sd);
	u8 pid, ver;
	int ret;

	mutex_lock(&ov9655->lock);
	__ov9655_set_power(ov9655, 1);
	msleep(25);

	/* Check sensor revision */
	ret = ov9655_read(client, REG_PID, &pid);
	if (!ret)
		ret = ov9655_read(client, REG_VER, &ver);

	__ov9655_set_power(ov9655, 0);

	if (!ret) {
		ov9655->id = OV9655_ID(pid, ver);
		switch (ov9655->id) {
		case OV9655V4_ID:
			v4l2_info(sd, "Found OV9655 REV4 sensor, this revision is not supported\n");
			ret = -ENODEV;
			break;
		case OV9655V5_ID:
			v4l2_info(sd, "Found OV9655 REV5 sensor\n");
			break;
		default:
			v4l2_err(sd, "Sensor detection failed (%04X, %d)\n",
				 ov9655->id, ret);
			ret = -ENODEV;
		}
	}
	mutex_unlock(&ov9655->lock);

	return ret;
}

static int ov9655_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct ov9655 *ov9655;
	int ret;

	ov9655 = devm_kzalloc(&client->dev, sizeof(*ov9655), GFP_KERNEL);
	if (!ov9655)
		return -ENOMEM;

	ov9655->client = client;

	ov9655->gpio_reset =
		devm_gpiod_get_optional(&client->dev, "reset", 1);
	ov9655->gpio_powerdown =
		devm_gpiod_get_optional(&client->dev, "powerdown", 1);

	ov9655->avdd = devm_regulator_get(&client->dev, "avdd");
	if (IS_ERR(ov9655->avdd)) {
		dev_err(&client->dev, "Could not get analog regulator\n");
		return PTR_ERR(ov9655->avdd);
	}

	ov9655->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(ov9655->clk)) {
		dev_err(&client->dev, "Could not get clock\n");
		return PTR_ERR(ov9655->clk);
	}

	sd = &ov9655->sd;
	v4l2_i2c_subdev_init(sd, client, &ov9655_subdev_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));

	sd->internal_ops = &ov9655_sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	ov9655->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov9655->pad);
	if (ret < 0)
		return ret;

	mutex_init(&ov9655->lock);

	ret = ov9655_initialize_controls(ov9655);
	if (ret < 0)
		goto err_me;

	ov9655_get_default_format(&ov9655->format);

	ret = ov9655_detect_sensor(sd);
	if (ret < 0)
		goto err_ctrls;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		goto err_ctrls;

	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(sd->ctrl_handler);
err_me:
	mutex_destroy(&ov9655->lock);
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int ov9655_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9655 *ov9655 = to_ov9655(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	mutex_destroy(&ov9655->lock);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct i2c_device_id ov9655_id[] = {
	{ "ov9655", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ov9655_id);

static const struct of_device_id ov9655_of_match[] = {
	{ .compatible = "ovti,ov9655", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov9655_of_match);

static struct i2c_driver ov9655_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = ov9655_of_match,
	},
	.probe		= ov9655_probe,
	.remove		= ov9655_remove,
	.id_table	= ov9655_id,
};

module_i2c_driver(ov9655_i2c_driver);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@st.com>");
MODULE_DESCRIPTION("OV9655 CMOS Image Sensor driver");
MODULE_LICENSE("GPL");
