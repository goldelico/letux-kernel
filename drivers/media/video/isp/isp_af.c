/*
 * isp_af.c
 *
 * AF module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Linux specific include files */
#include <asm/cacheflush.h>

#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <asm/atomic.h>

#include "isp.h"
#include "ispreg.h"
#include "isph3a.h"
#include "isp_af.h"

#define IS_OUT_OF_BOUNDS(value, min, max)		\
	(((value) < (min)) || ((value) > (max)))

void isp_af_set_address(struct isp_af_device *isp_af, unsigned long address)
{
	isp_reg_writel(isp_af->dev, address, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AFBUFST);
}

/* Function to check paxel parameters */
int isp_af_check_paxel(struct isp_af_device *isp_af)
{
	struct af_paxel *paxel_cfg = &isp_af->config.paxel_config;
	struct af_iir *iir_cfg = &isp_af->config.iir_config;

	/* Check horizontal Count */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->hz_cnt, AF_PAXEL_HORIZONTAL_COUNT_MIN,
			     AF_PAXEL_HORIZONTAL_COUNT_MAX)) {
		DPRINTK_ISP_AF("Error : Horizontal Count is incorrect");
		return -AF_ERR_HZ_COUNT;
	}

	/*Check Vertical Count */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->vt_cnt, AF_PAXEL_VERTICAL_COUNT_MIN,
			     AF_PAXEL_VERTICAL_COUNT_MAX)) {
		DPRINTK_ISP_AF("Error : Vertical Count is incorrect");
		return -AF_ERR_VT_COUNT;
	}

	/*Check Height */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->height, AF_PAXEL_HEIGHT_MIN,
			     AF_PAXEL_HEIGHT_MAX)) {
		DPRINTK_ISP_AF("Error : Height is incorrect");
		return -AF_ERR_HEIGHT;
	}

	/*Check width */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->width, AF_PAXEL_WIDTH_MIN,
			     AF_PAXEL_WIDTH_MAX)) {
		DPRINTK_ISP_AF("Error : Width is incorrect");
		return -AF_ERR_WIDTH;
	}

	/*Check Line Increment */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->line_incr, AF_PAXEL_INCREMENT_MIN,
			     AF_PAXEL_INCREMENT_MAX)) {
		DPRINTK_ISP_AF("Error : Line Increment is incorrect");
		return -AF_ERR_INCR;
	}

	/*Check Horizontal Start */
	if ((paxel_cfg->hz_start % 2 != 0) ||
	    (paxel_cfg->hz_start < (iir_cfg->hz_start_pos + 2)) ||
	    IS_OUT_OF_BOUNDS(paxel_cfg->hz_start,
			     AF_PAXEL_HZSTART_MIN, AF_PAXEL_HZSTART_MAX)) {
		DPRINTK_ISP_AF("Error : Horizontal Start is incorrect");
		return -AF_ERR_HZ_START;
	}

	/*Check Vertical Start */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->vt_start, AF_PAXEL_VTSTART_MIN,
			     AF_PAXEL_VTSTART_MAX)) {
		DPRINTK_ISP_AF("Error : Vertical Start is incorrect");
		return -AF_ERR_VT_START;
	}
	return 0;
}

/**
 * isp_af_check_iir - Function to check IIR Coefficient.
 **/
int isp_af_check_iir(struct isp_af_device *isp_af)
{
	struct af_iir *iir_cfg = &isp_af->config.iir_config;
	int index;

	for (index = 0; index < AF_NUMBER_OF_COEF; index++) {
		if ((iir_cfg->coeff_set0[index]) > AF_COEF_MAX) {
			DPRINTK_ISP_AF("Error : Coefficient for set 0 is "
				       "incorrect");
			return -AF_ERR_IIR_COEF;
		}

		if ((iir_cfg->coeff_set1[index]) > AF_COEF_MAX) {
			DPRINTK_ISP_AF("Error : Coefficient for set 1 is "
				       "incorrect");
			return -AF_ERR_IIR_COEF;
		}
	}

	if (IS_OUT_OF_BOUNDS(iir_cfg->hz_start_pos, AF_IIRSH_MIN,
			     AF_IIRSH_MAX)) {
		DPRINTK_ISP_AF("Error : IIRSH is incorrect");
		return -AF_ERR_IIRSH;
	}

	return 0;
}

/* Function to perform hardware set up */
int isp_af_configure(struct isp_af_device *isp_af,
		     struct af_configuration *afconfig)
{
	int result;
	int buf_size;
	unsigned int busyaf;
	struct af_configuration *af_curr_cfg = &isp_af->config;
	struct ispstat_buffer *buf;

	if (NULL == afconfig) {
		dev_err(isp_af->dev, "af: Null argument in configuration. \n");
		return -EINVAL;
	}

	memcpy(af_curr_cfg, afconfig, sizeof(struct af_configuration));
	/* Get the value of PCR register */
	busyaf = isp_reg_readl(isp_af->dev, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);

	if ((busyaf & AF_BUSYAF) == AF_BUSYAF) {
		DPRINTK_ISP_AF("AF_register_setup_ERROR : Engine Busy");
		DPRINTK_ISP_AF("\n Configuration cannot be done ");
		return -AF_ERR_ENGINE_BUSY;
	}

	/* Check IIR Coefficient and start Values */
	result = isp_af_check_iir(isp_af);
	if (result < 0)
		return result;

	/* Check Paxel Values */
	result = isp_af_check_paxel(isp_af);
	if (result < 0)
		return result;

	/* Check HMF Threshold Values */
	if (af_curr_cfg->hmf_config.threshold > AF_THRESHOLD_MAX) {
		DPRINTK_ISP_AF("Error : HMF Threshold is incorrect");
		return -AF_ERR_THRESHOLD;
	}

	/* Compute buffer size */
	buf_size = (af_curr_cfg->paxel_config.hz_cnt + 1) *
		(af_curr_cfg->paxel_config.vt_cnt + 1) * AF_PAXEL_SIZE;

	result = ispstat_bufs_alloc(&isp_af->stat, buf_size);
	if (result)
		return result;

	result = isp_af_register_setup(isp_af);
	if (result < 0)
		return result;

	isp_af->size_paxel = buf_size;

	buf = ispstat_buf_next(&isp_af->stat);
	isp_af_set_address(isp_af, buf->iommu_addr);

	/* Set configuration flag to indicate HW setup done */
	if (af_curr_cfg->af_config)
		isp_af_enable(isp_af, 1);
	else
		isp_af_enable(isp_af, 0);

	/* Success */
	return 0;
}
EXPORT_SYMBOL(isp_af_configure);

int isp_af_register_setup(struct isp_af_device *isp_af)
{
	unsigned int pcr = 0, pax1 = 0, pax2 = 0, paxstart = 0;
	unsigned int coef = 0;
	unsigned int base_coef_set0 = 0;
	unsigned int base_coef_set1 = 0;
	int index;

	/* Configure Hardware Registers */
	/* Read PCR Register */
	pcr = isp_reg_readl(isp_af->dev, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);

	/* Set Accumulator Mode */
	if (isp_af->config.mode == ACCUMULATOR_PEAK)
		pcr |= FVMODE;
	else
		pcr &= ~FVMODE;

	/* Set A-law */
	if (isp_af->config.alaw_enable == H3A_AF_ALAW_ENABLE)
		pcr |= AF_ALAW_EN;
	else
		pcr &= ~AF_ALAW_EN;

	/* Set RGB Position */
	pcr &= ~RGBPOS;
	pcr |= isp_af->config.rgb_pos << AF_RGBPOS_SHIFT;

	/* HMF Configurations */
	if (isp_af->config.hmf_config.enable == H3A_AF_HMF_ENABLE) {
		pcr &= ~AF_MED_EN;
		/* Enable HMF */
		pcr |= AF_MED_EN;

		/* Set Median Threshold */
		pcr &= ~MED_TH;
		pcr |= isp_af->config.hmf_config.threshold << AF_MED_TH_SHIFT;
	} else
		pcr &= ~AF_MED_EN;

	/* Set PCR Register */
	isp_reg_writel(isp_af->dev, pcr, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);

	pax1 &= ~PAXW;
	pax1 |= isp_af->config.paxel_config.width << AF_PAXW_SHIFT;

	/* Set height in AFPAX1 */
	pax1 &= ~PAXH;
	pax1 |= isp_af->config.paxel_config.height;

	isp_reg_writel(isp_af->dev, pax1, OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX1);

	/* Configure AFPAX2 Register */
	/* Set Line Increment in AFPAX2 Register */
	pax2 &= ~AFINCV;
	pax2 |= isp_af->config.paxel_config.line_incr << AF_LINE_INCR_SHIFT;
	/* Set Vertical Count */
	pax2 &= ~PAXVC;
	pax2 |= isp_af->config.paxel_config.vt_cnt << AF_VT_COUNT_SHIFT;
	/* Set Horizontal Count */
	pax2 &= ~PAXHC;
	pax2 |= isp_af->config.paxel_config.hz_cnt;
	isp_reg_writel(isp_af->dev, pax2, OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX2);

	/* Configure PAXSTART Register */
	/*Configure Horizontal Start */
	paxstart &= ~PAXSH;
	paxstart |= isp_af->config.paxel_config.hz_start << AF_HZ_START_SHIFT;
	/* Configure Vertical Start */
	paxstart &= ~PAXSV;
	paxstart |= isp_af->config.paxel_config.vt_start;
	isp_reg_writel(isp_af->dev, paxstart, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AFPAXSTART);

	/*SetIIRSH Register */
	isp_reg_writel(isp_af->dev, isp_af->config.iir_config.hz_start_pos,
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFIIRSH);

	/*Set IIR Filter0 Coefficients */
	base_coef_set0 = ISPH3A_AFCOEF010;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= isp_af->config.iir_config.coeff_set0[index];
		coef &= ~COEF_MASK1;
		coef |= isp_af->config.iir_config.coeff_set0[index + 1] <<
			AF_COEF_SHIFT;
		isp_reg_writel(isp_af->dev, coef, OMAP3_ISP_IOMEM_H3A,
			       base_coef_set0);
		base_coef_set0 = base_coef_set0 + AFCOEF_OFFSET;
	}

	/* set AFCOEF0010 Register */
	isp_reg_writel(isp_af->dev, isp_af->config.iir_config.coeff_set0[10],
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF010);

	/*Set IIR Filter1 Coefficients */

	base_coef_set1 = ISPH3A_AFCOEF110;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= isp_af->config.iir_config.coeff_set1[index];
		coef &= ~COEF_MASK1;
		coef |= isp_af->config.iir_config.coeff_set1[index + 1] <<
			AF_COEF_SHIFT;
		isp_reg_writel(isp_af->dev, coef, OMAP3_ISP_IOMEM_H3A,
			       base_coef_set1);

		base_coef_set1 = base_coef_set1 + AFCOEF_OFFSET;
	}
	isp_reg_writel(isp_af->dev, isp_af->config.iir_config.coeff_set1[10],
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF1010);

	return 0;
}

/*
 * This API allows the user to update White Balance gains, as well as
 * exposure time and analog gain. It is also used to request frame
 * statistics.
 */
int isp_af_request_statistics(struct isp_af_device *isp_af,
			      struct isp_af_data *afdata)
{
	struct ispstat_buffer *buf;

	if (!isp_af->config.af_config) {
		dev_err(isp_af->dev, "af: engine not enabled\n");
		return -EINVAL;
	}

	if (afdata->update & REQUEST_STATISTICS) {
		buf = ispstat_buf_get(&isp_af->stat,
			      (void *)afdata->af_statistics_buf,
			      afdata->frame_number);
		if (IS_ERR(buf))
			return PTR_ERR(buf);

		afdata->config_counter = buf->config_counter;
		afdata->frame_number = buf->frame_number;

		ispstat_buf_release(&isp_af->stat);
	} else
		afdata->af_statistics_buf = NULL;
	afdata->curr_frame = isp_af->stat.frame_number;

	return 0;
}
EXPORT_SYMBOL(isp_af_request_statistics);

/* This function will handle the H3A interrupt. */
void isp_af_isr(struct isp_af_device *isp_af)
{
	struct ispstat_buffer *buf;

	/* Exchange buffers */
	buf = ispstat_buf_next(&isp_af->stat);
	isp_af_set_address(isp_af, buf->iommu_addr);
}

static int __isp_af_enable(struct isp_af_device *isp_af, int enable)
{
	unsigned int pcr;

	pcr = isp_reg_readl(isp_af->dev, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);

	/* Set AF_EN bit in PCR Register */
	if (enable)
		pcr |= AF_EN;
	else
		pcr &= ~AF_EN;

	isp_reg_writel(isp_af->dev, pcr, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);
	return 0;
}

/* Function to Enable/Disable AF Engine */
int isp_af_enable(struct isp_af_device *isp_af, int enable)
{
	int rval;

	rval = __isp_af_enable(isp_af, enable);

	if (!rval)
		isp_af->pm_state = enable;

	return rval;
}

/* Function to Suspend AF Engine */
void isp_af_suspend(struct isp_af_device *isp_af)
{
	if (isp_af->pm_state)
		__isp_af_enable(isp_af, 0);
}

/* Function to Resume AF Engine */
void isp_af_resume(struct isp_af_device *isp_af)
{
	if (isp_af->pm_state)
		__isp_af_enable(isp_af, 1);
}

int isp_af_busy(struct isp_af_device *isp_af)
{
	return isp_reg_readl(isp_af->dev, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR)
		& ISPH3A_PCR_BUSYAF;
}

/* Function to register the AF character device driver. */
int __init isp_af_init(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_af_device *isp_af = &isp->isp_af;

	isp_af->dev = dev;
	ispstat_init(dev, &isp_af->stat, H3A_MAX_BUFF, MAX_FRAME_COUNT);

	return 0;
}

void isp_af_exit(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	/* Free buffers */
	ispstat_free(&isp->isp_af.stat);
}
