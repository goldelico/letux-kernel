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
		       ~(BIT(0) | BIT(4)),
		       enable ? (BIT(0) | BIT(4)) : 0);

	isp_csi->if_enabled = enable ? true : false;
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
	u32 i = 0, val, reg;
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
	isp_reg_writel(dev, isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_SYSCONFIG) | BIT(1),
		       OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSCONFIG);
	while (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
		 BIT(0))) {
		udelay(10);
		if (i++ > 10)
			break;
	}
	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
	      BIT(0))) {
		dev_warn(dev,
		       "omap3_isp: timeout waiting for csi reset\n");
	}

	/* ISPCSI1_CTRL */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	val &= ~BIT(11);	/* Enable VP only off ->
				   extract embedded data to interconnect */
	BIT_SET(val, 8, 0x3, config->vpclk);	/* Video port clock */
/*	val |= BIT(3);	*/	/* Wait for FEC before disabling interface */
	val |= BIT(2);		/* I/O cell output is parallel
				   (no effect, but errata says should be enabled
				   for class 1/2) */
	val |= BIT(12);		/* VP clock polarity to falling edge
				   (needed or bad picture!) */

	/* Data/strobe physical layer */
	BIT_SET(val, 1, 1, config->signalling);
	BIT_SET(val, 10, 1, config->strobe_clock_inv);
	val |= BIT(4);		/* Magic bit to enable CSI1 and strobe mode */
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	/* ISPCSI1_LCx_CTRL logical channel #0 */
	reg = ISPCSI1_LCx_CTRL(0);	/* reg = ISPCSI1_CTRL1; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	/* Format = RAW10+VP or RAW8+DPCM10+VP*/
	BIT_SET(val, 3, 0x1f, format);
	/* Enable setting of frame regions of interest */
	BIT_SET(val, 1, 1, 1);
	BIT_SET(val, 2, 1, config->crc);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_START for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_START(0);		/* reg = ISPCSI1_DAT_START; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->data_start);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_SIZE for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_SIZE(0);		/* reg = ISPCSI1_DAT_SIZE; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->data_size);
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

	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2,
			    ISPCSI1_CTRL) & BIT(4))) {
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

	return 0;
}
