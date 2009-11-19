/*
 * isp_csi.c
 *
 * Driver Library for ISP CSI1/CCP2 Control module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <media/v4l2-common.h>
#include <linux/delay.h>

#include "isp.h"

#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

/**
 * isp_csi_enable - Enable CSI1/CCP2 interface.
 * @isp_csi: Pointer to ISP CSI/CCP2 device.
 * @enable: Enable flag.
 **/
void isp_csi_enable(struct isp_csi_device *isp_csi, u8 enable)
{
	struct device *dev = to_device(isp_csi);

	isp_reg_and_or(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL,
		       ~(ISPCSI1_CTRL_IF_EN |
			 ISPCSI1_CTRL_MODE_CCP2),
		       enable ?
			(ISPCSI1_CTRL_IF_EN |
			 ISPCSI1_CTRL_MODE_CCP2)
			: 0);

	isp_csi->if_enabled = enable ? true : false;
}

/**
 * isp_csi_reset - Reset CSI1/CCP2 interface.
 * @isp_csi: Pointer to ISP CSI/CCP2 device.
 * @enable: Enable flag.
 **/
static void isp_csi_reset(struct isp_csi_device *isp_csi)
{
	struct device *dev = to_device(isp_csi);
	u32 i = 0;

	isp_reg_or(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSCONFIG,
		   ISPCSI1_SYSCONFIG_SOFT_RESET);
	while (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
		 ISPCSI1_SYSSTATUS_RESET_DONE)) {
		udelay(10);
		if (i++ > 10)
			break;
	}
	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
	      ISPCSI1_SYSSTATUS_RESET_DONE)) {
		dev_warn(dev,
		       "timeout waiting for csi reset\n");
	}
}

/**
 * isp_csi_set_vp_cfg - Configure CSI1/CCP2 Videport.
 * @isp_csi: Pointer to ISP CSI/CCP2 device.
 * @vp_cfg: Pointer to videoport configuration structure.
 *
 * Returns 0 if successful, -EBUSY if the interface is enabled,
 * and -EINVAL if wrong parameters are passed.
 **/
static int isp_csi_set_vp_cfg(struct isp_csi_device *isp_csi,
			      struct isp_csi_vp_cfg *vp_cfg)
{
	struct device *dev = to_device(isp_csi);
	u32 val;

	if (isp_csi->if_enabled) {
		dev_err(dev, "CSI1/CCP2: VP cannot be configured when"
			     " interface is enabled.");
		return -EBUSY;
	}

	if (vp_cfg->write_polarity > 1) {
		dev_err(dev, "CSI1/CCP2: Wrong VP clock polarity value."
			     " Must be 0 or 1");
		return -EINVAL;
	}

	if (vp_cfg->divider > 3) {
		dev_err(dev, "CSI1/CCP2: Wrong divisor value. Must be"
			     " between 0 and 3");
		return -EINVAL;
	}

	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	/* Mask VP related fields */
	val &= ~(ISPCSI1_CTRL_VP_ONLY_EN |
		 (ISPCSI1_CTRL_VP_OUT_CTRL_MASK <<
		  ISPCSI1_CTRL_VP_OUT_CTRL_SHIFT) |
		 ISPCSI1_CTRL_VP_CLK_POL);

	if (vp_cfg->no_ocp)
		val |= ISPCSI1_CTRL_VP_ONLY_EN;

	val |= (vp_cfg->divider << ISPCSI1_CTRL_VP_OUT_CTRL_SHIFT);

	if (vp_cfg->write_polarity)
		val |= ISPCSI1_CTRL_VP_CLK_POL;

	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	return 0;
}

/**
 * isp_csi_configure_interface - Initialize CSI1/CCP2 interface.
 * @isp_csi: Pointer to ISP CSI/CCP2 device.
 * @config: Pointer to ISP CSI/CCP2 interface config structure.
 *
 * This will analize the parameters passed by the interface config
 * structure, and configure the respective registers for proper CSI1/CCP2
 * config.
 *
 * Returns -EINVAL if wrong format, -EIO if strobe is choosen in CSI1 mode, or
 * 0 on success.
 **/
int isp_csi_configure_interface(struct isp_csi_device *isp_csi,
				struct isp_csi_interface_cfg *config)
{
	struct device *dev = to_device(isp_csi);
	struct isp_csi_vp_cfg new_vp_cfg;
	u32 val, reg;
	int format;

	switch (config->format) {
	case V4L2_PIX_FMT_SGRBG10:
		format = 0x16;		/* RAW10+VP */
		break;
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		format = 0x12;		/* RAW8+DPCM10+VP */
		break;
	default:
		dev_err(dev, "isp_csi_configure_interface: bad csi format\n");
		return -EINVAL;
	}

	/* Reset the CSI and wait for reset to complete */
	isp_csi_reset(isp_csi);

	/* Configure Videoport */
	new_vp_cfg.no_ocp = false;
	new_vp_cfg.divider = config->vpclk;
	new_vp_cfg.write_polarity = 1;
	isp_csi_set_vp_cfg(isp_csi, &new_vp_cfg);

	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	/* I/O cell output is parallel
	 * (no effect, but errata says should be enabled for class 1/2)
	 */
	val |= ISPCSI1_CTRL_IO_OUT_SEL;

	/* Data/strobe physical layer */
	val &= ~(ISPCSI1_CTRL_PHY_SEL | ISPCSI1_CTRL_INV);
	if (config->signalling)
		val |= ISPCSI1_CTRL_PHY_SEL;
	if (config->strobe_clock_inv)
		val |= ISPCSI1_CTRL_INV;
	val |= ISPCSI1_CTRL_MODE_CCP2;
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	/* Logical channel #0 - Set format and generic configuration */
	reg = ISPCSI1_LCx_CTRL(0);
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, ISPCSI1_LCx_CTRL_FORMAT_SHIFT,
		ISPCSI1_LCx_CTRL_FORMAT_MASK, format);
	/* Enable setting of frame regions of interest */
	val |= ISPCSI1_LCx_CTRL_REGION_EN;
	val &= ~(ISPCSI1_LCx_CTRL_CRC_EN);
	if (config->crc)
		val |= ISPCSI1_LCx_CTRL_CRC_EN;
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* Logical channel #0 - Set pixel data region */
	reg = ISPCSI1_LCx_DAT_START(0);
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, ISPCSI1_LCx_DAT_START_VERT_SHIFT,
		ISPCSI1_LCx_DAT_START_VERT_MASK, config->data_start);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	reg = ISPCSI1_LCx_DAT_SIZE(0);
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, ISPCSI1_LCx_DAT_SIZE_VERT_SHIFT,
		ISPCSI1_LCx_DAT_SIZE_VERT_MASK, config->data_size);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* Clear status bits for logical channel #0 */
	val = ISPCSI1_LC01_IRQSTATUS_LC0_FIFO_OVF_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_CRC_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FSP_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FW_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FSC_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_SSC_IRQ;

	/* Clear IRQ status bits for logical channel #0 */
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_LC01_IRQSTATUS);

	/* Enable IRQs for logical channel #0 */
	isp_reg_or(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_LC01_IRQENABLE, val);

	/* Enable CSI1 */
	isp_csi_enable(isp_csi, 1);

	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL) &
	      ISPCSI1_CTRL_MODE_CCP2)) {
		dev_warn(dev, "OMAP3 CSI1 bus not available\n");
		if (config->signalling) {
			/* Strobe mode requires CCP2 */
			return -EIO;
		}
	}

	return 0;
}

/**
 * isp_csi_init - Routine for module driver init
 **/
int __init isp_csi_init(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_csi_device *isp_csi = &isp->isp_csi;

	isp_csi->if_enabled = false;
	isp_csi->vp_cfg.no_ocp = false;
	isp_csi->vp_cfg.divider = 1;
	isp_csi->vp_cfg.write_polarity = 0;

	return 0;
}
