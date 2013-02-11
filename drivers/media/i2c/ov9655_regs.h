/*
 * drivers/media/video/ov9655_regs.h
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Sivaraj R <sivaraj@ti.com>
 *     Brijesh R Jadav <brijesh.j@ti.com>
 *     Hardik Shah <hardik.shah@ti.com>
 *     Manjunath Hadli <mrh@ti.com>
 *     Karicheri Muralidharan <m-karicheri2@ti.com>
 *
 * merged with material from
 *
 * Copyright (C) 2008 - 2009
 * Heinz Nixdorf Institute - University of Paderborn
 * Department of System and Circuit Technology
 * Stefan Herbrechtsmeier <hbmeier@hni.uni-paderborn.de>
 *
 * Based on rj54n1cbc, mt9t031 and soc_camera_platform driver
 *
 * Copyright (C) 2008, Guennadi Liakhovetski, DENX Software Engineering <lg@denx.de>
 * Copyright (C) 2009, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
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

#ifndef _OV9655_REGS_H
#define _OV9655_REGS_H

/* this register list is taken from the OV9655 soc_camera driver */

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
#define OV9655_VER			0x0B
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

/**
 * struct ov9655_reg - Structure for TVP5146/47 register initialization values
 * @token - Token: TOK_WRITE, TOK_TERM etc..
 * @reg - Register offset
 * @val - Register Value for TOK_WRITE or delay in ms for TOK_DELAY
 */
struct ov9655_reg {
//	unsigned short token;
	unsigned short addr;
	unsigned short value;
};

/**
 * struct ov9655_init_seq - Structure for TVP5146/47/46M2/47M1 power up
 *		Sequence.
 * @ no_regs - Number of registers to write for power up sequence.
 * @ init_reg_seq - Array of registers and respective value to write.
 */
struct ov9655_init_seq {
	unsigned int no_regs;
	const struct ov9655_reg *init_reg_seq;
};

#define OV9655_CHIP_PID			(0x96)
#define OV9655_CHIP_VER4		(0x56)
#define OV9655_CHIP_VER5		(0x57)

#endif				/* ifndef _OV9655_REGS_H */
