// SPDX-License-Identifier: GPL-2.0-or-later
//
// KMS driver for SLCD controller of Ingenic X1000 series SoCs - Registers definitions
//
// Copyright (C) 2023, Reimu NotMoe <reimu@sudomaker.com>

#ifndef DRIVERS_GPU_DRM_INGENIC_X1000_SLCDC_DRM_H
#define DRIVERS_GPU_DRM_INGENIC_X1000_SLCDC_DRM_H

#include <linux/bitops.h>
#include <linux/types.h>

#define JZ_REG_LCD_CFG				0x00
#define JZ_REG_LCD_DAH				0x10
#define JZ_REG_LCD_DAV				0x14
#define JZ_REG_LCD_PS				0x18
#define JZ_REG_LCD_CLS				0x1C
#define JZ_REG_LCD_SPL				0x20
#define JZ_REG_LCD_REV				0x24
#define JZ_REG_LCD_CTRL				0x30
#define JZ_REG_LCD_STATE			0x34
#define JZ_REG_LCD_IID				0x38
#define JZ_REG_LCD_DA0				0x40
#define JZ_REG_LCD_SA0				0x44
#define JZ_REG_LCD_FID0				0x48
#define JZ_REG_LCD_CMD0				0x4C

#define JZ_REG_SLCD_MCFG			0xA0
#define JZ_REG_SLCD_MCTRL			0xA4
#define JZ_REG_SLCD_MSTATE			0xA8
#define JZ_REG_SLCD_MDATA			0xAC
#define JZ_REG_SLCD_WTIME			0xB0
#define JZ_REG_SLCD_TASH			0xB4
#define JZ_REG_SLCD_MCFG_NEW			0xB8
#define JZ_REG_SLCD_SMWT			0xBC

#define JZ_LCD_CFG_INV					BIT(17)

#define JZ_LCD_DAH_HDE_OFFSET				0

#define JZ_LCD_DAV_VDE_OFFSET				0

#define JZ_LCD_CTRL_BURST_4				(0x0 << 28)
#define JZ_LCD_CTRL_BURST_8				(0x1 << 28)
#define JZ_LCD_CTRL_BURST_16				(0x2 << 28)
#define JZ_LCD_CTRL_BURST_32				(0x3 << 28)
#define JZ_LCD_CTRL_BURST_64				(0x4 << 28)
#define JZ_LCD_CTRL_BURST_MASK				(0x7 << 28)
#define JZ_LCD_CTRL_EOF_IRQ				BIT(13)
#define JZ_LCD_CTRL_SOF_IRQ				BIT(12)
#define JZ_LCD_CTRL_IFU0_IRQ				BIT(10)
#define JZ_LCD_CTRL_QDD_IRQ				BIT(7)
#define JZ_LCD_CTRL_REVERSE_ENDIAN			BIT(6)
#define JZ_LCD_CTRL_LSB_FISRT				BIT(5)
#define JZ_LCD_CTRL_ENABLE				BIT(3)
#define JZ_LCD_CTRL_BPP_15_16				0x4
#define JZ_LCD_CTRL_BPP_18_24				0x5
#define JZ_LCD_CTRL_BPP_24_COMP				0x6
#define JZ_LCD_CTRL_BPP_30				0x7
#define JZ_LCD_CTRL_BPP_MASK				(JZ_LCD_CTRL_RGB555 | 0x7)

#define JZ_LCD_CMD_SOF_IRQ				BIT(31)
#define JZ_LCD_CMD_EOF_IRQ				BIT(30)
#define JZ_LCD_CMD_IS_CMD				BIT(29)
#define JZ_LCD_CMD_FRM_ENABLE				BIT(26)

#define JZ_LCD_STATE_QD					BIT(7)
#define JZ_LCD_STATE_EOF_IRQ				BIT(5)
#define JZ_LCD_STATE_SOF_IRQ				BIT(4)
#define JZ_LCD_STATE_IFU0				BIT(2)

#define JZ_LCD_XYP01_XPOS_LSB				0
#define JZ_LCD_XYP01_YPOS_LSB				16

#define JZ_LCD_SIZE01_WIDTH_LSB				0
#define JZ_LCD_SIZE01_HEIGHT_LSB			16

#define JZ_LCD_DESSIZE_ALPHA_OFFSET			24
#define JZ_LCD_DESSIZE_HEIGHT_MASK			GENMASK(23, 12)
#define JZ_LCD_DESSIZE_WIDTH_MASK			GENMASK(11, 0)

#define JZ_LCD_CPOS_BPP_15_16				(4 << 27)
#define JZ_LCD_CPOS_BPP_18_24				(5 << 27)
#define JZ_LCD_CPOS_BPP_30				(7 << 27)
#define JZ_LCD_CPOS_RGB555				BIT(30)

#define JZ_LCD_CPOS_YPOS0_OFFSET			12
#define JZ_LCD_CPOS_XPOS0_OFFSET			0

#define JZ_SLCD_MCFG_CWIDTH_MASK			GENMASK(9, 8)
#define JZ_SLCD_MCFG_CWIDTH_16_9			(0 << 8)
#define JZ_SLCD_MCFG_CWIDTH_8				(1 << 8)
#define JZ_SLCD_MCFG_CWIDTH_18				(2 << 8)
#define JZ_SLCD_MCFG_CWIDTH_24				(3 << 8)

#define JZ_SLCD_MCFGNEW_DWIDTH_MASK			GENMASK(15, 13)
#define JZ_SLCD_MCFGNEW_DWIDTH_8			(0 << 13)
#define JZ_SLCD_MCFGNEW_DWIDTH_9			(1 << 13)
#define JZ_SLCD_MCFGNEW_DWIDTH_16			(2 << 13)
#define JZ_SLCD_MCFGNEW_DWIDTH_18			(3 << 13)
#define JZ_SLCD_MCFGNEW_DWIDTH_24			(4 << 13)
#define JZ_SLCD_MCFGNEW_6800_MD				BIT(11)
#define JZ_SLCD_MCFGNEW_CMD_9BIT			BIT(10)
#define JZ_SLCD_MCFGNEW_DTIMES_MASK			GENMASK(9, 8)
#define JZ_SLCD_MCFGNEW_DTIMES_1			(0 << 8)
#define JZ_SLCD_MCFGNEW_DTIMES_2			(1 << 8)
#define JZ_SLCD_MCFGNEW_DTIMES_3			(2 << 8)
#define JZ_SLCD_MCFGNEW_CSPOL				BIT(5)
#define JZ_SLCD_MCFGNEW_RSPOL				BIT(4)
#define JZ_SLCD_MCFGNEW_CLKPOL				BIT(3)
#define JZ_SLCD_MCFGNEW_DTYPE				BIT(2)
#define JZ_SLCD_MCFGNEW_CTYPE				BIT(1)
#define JZ_SLCD_MCFGNEW_FMT_CONV			BIT(0)

#define JZ_SLCD_MCTRL_NARROW_TE				BIT(10)
#define JZ_SLCD_MCTRL_TE_INV				BIT(9)
#define JZ_SLCD_MCTRL_NOT_USE_TE			BIT(8)
#define JZ_SLCD_MCTRL_DCSI_SEL				BIT(7)
#define JZ_SLCD_MCTRL_MIPI_SLCD				BIT(6)
#define JZ_SLCD_MCTRL_FAST_MODE				BIT(4)
#define JZ_SLCD_MCTRL_GATE_MASK				BIT(3)
#define JZ_SLCD_MCTRL_DMAMODE				BIT(2)
#define JZ_SLCD_MCTRL_DMASTART				BIT(1)
#define JZ_SLCD_MCTRL_DMATXEN				BIT(0)

#define JZ_SLCD_MSTATE_BUSY				BIT(0)

struct device;
struct drm_plane;
struct drm_plane_state;
struct platform_driver;

void ingenic_drm_plane_config(struct device *dev,
			      struct drm_plane *plane, u32 fourcc);
void ingenic_drm_plane_disable(struct device *dev, struct drm_plane *plane);
bool ingenic_drm_map_noncoherent(const struct device *dev);

extern struct platform_driver *ingenic_ipu_driver_ptr;

#endif /* DRIVERS_GPU_DRM_INGENIC_X1000_SLCDC_DRM_H */
