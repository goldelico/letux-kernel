/*
 * ALSA SoC TWL6030 codec driver
 *
 * Author:      Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TWL6030_H__
#define __TWL6030_H__

#define TWL6030_REG_ASICID		0x01
#define TWL6030_REG_ASICREV		0x02
#define TWL6030_REG_INTID		0x03
#define TWL6030_REG_INTMR		0x04
#define TWL6030_REG_NCPCTL		0x05
#define TWL6030_REG_LDOCTL		0x06
#define TWL6030_REG_HPPLLCTL		0x07
#define TWL6030_REG_LPPLLCTL		0x08
#define TWL6030_REG_LPPLLDIV		0x09
#define TWL6030_REG_AMICBCTL		0x0A
#define TWL6030_REG_DMICBCTL		0x0B
#define TWL6030_REG_MICLCTL		0x0C
#define TWL6030_REG_MICRCTL		0x0D
#define TWL6030_REG_MICGAIN		0x0E
#define TWL6030_REG_LINEGAIN		0x0F
#define TWL6030_REG_HSLCTL		0x10
#define TWL6030_REG_HSRCTL		0x11
#define TWL6030_REG_HSGAIN		0x12
#define TWL6030_REG_EARCTL		0x13
#define TWL6030_REG_HFLCTL		0x14
#define TWL6030_REG_HFLGAIN		0x15
#define TWL6030_REG_HFRCTL		0x16
#define TWL6030_REG_HFRGAIN		0x17
#define TWL6030_REG_VIBCTLL		0x18
#define TWL6030_REG_VIBDATL		0x19
#define TWL6030_REG_VIBCTLR		0x1A
#define TWL6030_REG_VIBDATR		0x1B
#define TWL6030_REG_HKCTL1		0x1C
#define TWL6030_REG_HKCTL2		0x1D
#define TWL6030_REG_GPOCTL		0x1E
#define TWL6030_REG_ALB			0x1F
#define TWL6030_REG_DLB			0x20
#define TWL6030_REG_TRIM1		0x28
#define TWL6030_REG_TRIM2		0x29
#define TWL6030_REG_TRIM3		0x2A
#define TWL6030_REG_HSOTRIM		0x2B
#define TWL6030_REG_HFOTRIM		0x2C
#define TWL6030_REG_ACCCTL		0x2D
#define TWL6030_REG_STATUS		0x2E

#define TWL6030_CACHEREGNUM		(TWL6030_REG_STATUS + 1)

#define TWL6030_VIOREGNUM		18
#define TWL6030_VDDREGNUM		21

/* NCPCTL (0x05) fields */

#define TWL6030_NCPENA			0x01
#define TWL6030_NCPOPEN			0x40

/* LDOCTL (0x06) fields */

#define TWL6030_LSLDOENA		0x01
#define TWL6030_HSLDOENA		0x04
#define TWL6030_REFENA			0x40
#define TWL6030_OSCENA			0x80

/* HPPLLCTL (0x07) fields */

#define TWL6030_HPLLENA			0x01
#define TWL6030_HPLLRST			0x02
#define TWL6030_HPLLBP			0x04
#define TWL6030_HPLLSQRENA		0x08
#define TWL6030_HPLLSQRBP		0x10
#define TWL6030_MCLK_12000KHZ		(0 << 5)
#define TWL6030_MCLK_19200KHZ		(1 << 5)
#define TWL6030_MCLK_26000KHZ		(2 << 5)
#define TWL6030_MCLK_38400KHZ		(3 << 5)
#define TWL6030_MCLK_MSK		0x60

/* LPPLLCTL (0x08) fields */

#define TWL6030_LPLLENA			0x01
#define TWL6030_LPLLRST			0x02
#define TWL6030_LPLLSEL			0x04
#define TWL6030_LPLLFIN			0x08
#define TWL6030_HPLLSEL			0x10

/* ACCCTL (0x2D) fields */

#define TWL6030_RESETSPLIT		0x04

extern struct snd_soc_dai twl6030_dai;
extern struct snd_soc_codec_device soc_codec_dev_twl6030;

#endif /* End of __TWL6030_H__ */
