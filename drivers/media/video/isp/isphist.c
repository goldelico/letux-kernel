/*
 * isphist.c
 *
 * HISTOGRAM module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	David Cohen <david.cohen@nokia.com>
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

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include "isp.h"
#include "ispreg.h"
#include "isphist.h"

/* Structure for saving/restoring histogram module registers */
struct isp_reg isphist_reg_list[] = {
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_WB_GAIN, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_ADDR, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD_OFF, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_H_V_INFO, 0},
	{0, ISP_TOK_TERM, 0}
};

static void isp_hist_print_status(struct isp_hist_device *isp_hist);

static void __isp_hist_enable(struct isp_hist_device *isp_hist, u8 enable)
{
	struct device *dev = to_device(isp_hist);
	unsigned int pcr;

	pcr = isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR);

	/* Set AF_EN bit in PCR Register */
	if (enable) {
		dev_dbg(dev, "hist: enabled\n");
		pcr |= ISPHIST_PCR_EN;
	} else {
		dev_dbg(dev, "hist: disabled\n");
		pcr &= ~ISPHIST_PCR_EN;
	}

	isp_reg_writel(dev, pcr, OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR);
}

/**
 * isp_hist_enable - Enables ISP Histogram submodule operation.
 * @enable: 1 - Enables the histogram submodule.
 *
 * Client should configure all the Histogram registers before calling this
 * function.
 **/
void isp_hist_enable(struct isp_hist_device *isp_hist, u8 enable)
{
	unsigned long irqflags;

	spin_lock_irqsave(&isp_hist->lock, irqflags);

	if (!isp_hist->config.enable) {
		spin_unlock_irqrestore(&isp_hist->lock, irqflags);
		return;
	}

	__isp_hist_enable(isp_hist, enable);
	isp_hist->enabled = enable;

	spin_unlock_irqrestore(&isp_hist->lock, irqflags);
}

/**
 * isp_hist_suspend - Suspend ISP Histogram submodule.
 **/
void isp_hist_suspend(struct isp_hist_device *isp_hist)
{
	unsigned long irqflags;

	spin_lock_irqsave(&isp_hist->lock, irqflags);
	if (isp_hist->enabled)
		__isp_hist_enable(isp_hist, 0);
	spin_unlock_irqrestore(&isp_hist->lock, irqflags);
}

void isp_hist_try_enable(struct isp_hist_device *isp_hist)
{
	unsigned long irqflags;

	spin_lock_irqsave(&isp_hist->lock, irqflags);
	if (unlikely(!isp_hist->enabled && isp_hist->config.enable)) {
		isp_hist->update = 1;
		spin_unlock_irqrestore(&isp_hist->lock, irqflags);
		isp_hist_config_registers(isp_hist);
		isp_hist_enable(isp_hist, 1);
	} else
		spin_unlock_irqrestore(&isp_hist->lock, irqflags);
}

/**
 * isp_hist_resume - Resume ISP Histogram submodule.
 **/
void isp_hist_resume(struct isp_hist_device *isp_hist)
{
	unsigned long irqflags;

	spin_lock_irqsave(&isp_hist->lock, irqflags);
	if (isp_hist->enabled)
		__isp_hist_enable(isp_hist, 1);
	spin_unlock_irqrestore(&isp_hist->lock, irqflags);
}

int isp_hist_busy(struct isp_hist_device *isp_hist)
{
	struct device *dev = to_device(isp_hist);

	return isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR)
			     & ISPHIST_PCR_BUSY;
}

/**
 * isp_hist_reset_mem - clear Histogram memory before start stats engine.
 **/
static void isp_hist_reset_mem(struct isp_hist_device *isp_hist)
{
	struct device *dev = to_device(isp_hist);
	int i = HIST_MEM_SIZE / 4;;

	isp_reg_writel(dev, 0, OMAP3_ISP_IOMEM_HIST, ISPHIST_ADDR);
	isp_reg_or(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, ISPHIST_CNT_CLR_EN);
	do {
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
	} while (likely(--i));
	isp_reg_and(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT,
		    ~ISPHIST_CNT_CLR_EN);
}

/**
 * isp_hist_set_regs - Helper function to update Histogram registers.
 **/
void isp_hist_config_registers(struct isp_hist_device *isp_hist)
{
	struct device *dev = to_device(isp_hist);
	unsigned long irqflags;

	if (!isp_hist->update || !isp_hist->config.enable)
		return;

	spin_lock_irqsave(&isp_hist->lock, irqflags);
	isp_hist->num_acc_frames = isp_hist->config.num_acc_frames;
	isp_hist_reset_mem(isp_hist);

	isp_reg_writel(dev, isp_hist->regs.cnt, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_CNT);
	isp_reg_writel(dev, isp_hist->regs.wb_gain,
		       OMAP3_ISP_IOMEM_HIST, ISPHIST_WB_GAIN);
	isp_reg_writel(dev, isp_hist->regs.reg_hor[0], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R0_HORZ);
	isp_reg_writel(dev, isp_hist->regs.reg_ver[0], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R0_VERT);
	isp_reg_writel(dev, isp_hist->regs.reg_hor[1], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R1_HORZ);
	isp_reg_writel(dev, isp_hist->regs.reg_ver[1], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R1_VERT);
	isp_reg_writel(dev, isp_hist->regs.reg_hor[2], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R2_HORZ);
	isp_reg_writel(dev, isp_hist->regs.reg_ver[2], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R2_VERT);
	isp_reg_writel(dev, isp_hist->regs.reg_hor[3], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R3_HORZ);
	isp_reg_writel(dev, isp_hist->regs.reg_ver[3], OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R3_VERT);
	isp_reg_writel(dev, isp_hist->regs.hist_radd,
		       OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD);
	isp_reg_writel(dev, isp_hist->regs.hist_radd_off,
		       OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD_OFF);
	isp_reg_writel(dev, isp_hist->regs.h_v_info,
		       OMAP3_ISP_IOMEM_HIST, ISPHIST_H_V_INFO);

	isp_hist->update = 0;
	spin_unlock_irqrestore(&isp_hist->lock, irqflags);

	isp_hist_print_status(isp_hist);
}

/**
 * isp_hist_isr - Callback from ISP driver for HIST interrupt.
 **/
void isp_hist_buf_process(struct isp_hist_device *isp_hist)
{
	struct device *dev = to_device(isp_hist);
	unsigned long irqflags;
	u32 *buf = isp_hist->active_buf->virt_addr;
	/* We'll read 4 times a 4-bytes-word at each iteration
	 * We know buf_size is divisible by 16 */
	unsigned int i = isp_hist->buf_size / 16;

	spin_lock_irqsave(&isp_hist->lock, irqflags);

	if (--isp_hist->num_acc_frames) {
		/* We need to accumulate more frames to save the hist frame */
		spin_unlock_irqrestore(&isp_hist->lock, irqflags);
		return;
	}

	isp_reg_writel(dev, 0, OMAP3_ISP_IOMEM_HIST, ISPHIST_ADDR);
	isp_reg_or(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT,
		   ISPHIST_CNT_CLR_EN);
	do {
		*buf++ = isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		*buf++ = isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		*buf++ = isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		*buf++ = isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
	} while (likely(--i));
	isp_reg_and(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT,
		    ~ISPHIST_CNT_CLR_EN);

	isp_hist->active_buf = ispstat_buf_next(&isp_hist->stat);
	isp_hist->num_acc_frames = isp_hist->config.num_acc_frames;

	spin_unlock_irqrestore(&isp_hist->lock, irqflags);
}

/**
 * isp_hist_validate_params - Helper function to check user given params.
 * @user_cfg: Pointer to user configuration structure.
 *
 * Returns 0 on success configuration.
 **/
static int isp_hist_validate_params(struct isp_hist_config *user_cfg)
{
	int c;

	if (user_cfg->source > HIST_SOURCE_MEM)
		return -EINVAL;

	if (user_cfg->source == HIST_SOURCE_MEM) {
		if ((user_cfg->input_bit_width < HIST_MIN_BIT_WIDTH) ||
			(user_cfg->input_bit_width > HIST_MAX_BIT_WIDTH))
			return -EINVAL;

		/* Should be in 32 byte boundary if source is mem */
		if ((user_cfg->hist_radd & ~ISP_32B_BOUNDARY_BUF) ||
		    (user_cfg->hist_radd_off & ~ISP_32B_BOUNDARY_OFFSET))
			return -EINVAL;
	} else if (user_cfg->input_bit_width != 10) /* CCDC must be 10bits */
		return -EINVAL;

	if (user_cfg->cfa > HIST_CFA_FOVEONX3)
		return -EINVAL;

	/* Regions size and position */

	if ((user_cfg->num_regions < HIST_MIN_REGIONS) ||
	    (user_cfg->num_regions > HIST_MAX_REGIONS))
		return -EINVAL;

	/* Regions */
	for (c = 0; c < user_cfg->num_regions; c++) {
		if (user_cfg->reg_hor[c] & ~ISPHIST_REGHORIZ_HEND_MASK &
		    ~ISPHIST_REGHORIZ_HSTART_MASK)
			return -EINVAL;
		if ((user_cfg->reg_hor[c] & ISPHIST_REGHORIZ_HEND_MASK) <=
		    ((user_cfg->reg_hor[c] & ISPHIST_REGHORIZ_HSTART_MASK) >>
		     ISPHIST_REGHORIZ_HSTART_SHIFT))
			return -EINVAL;
		if (user_cfg->reg_ver[c] & ~ISPHIST_REGVERT_VEND_MASK &
		    ~ISPHIST_REGVERT_VSTART_MASK)
			return -EINVAL;
		if ((user_cfg->reg_ver[c] & ISPHIST_REGVERT_VEND_MASK) <=
		    ((user_cfg->reg_ver[c] & ISPHIST_REGVERT_VSTART_MASK) >>
		     ISPHIST_REGVERT_VSTART_SHIFT))
			return -EINVAL;
	}

	switch (user_cfg->num_regions) {
	case 1:
		if (user_cfg->hist_bins > HIST_BINS_256)
			return -EINVAL;
		break;
	case 2:
		if (user_cfg->hist_bins > HIST_BINS_128)
			return -EINVAL;
		break;
	default: /* 3 or 4 */
		if (user_cfg->hist_bins > HIST_BINS_64)
			return -EINVAL;
		break;
	}

	return 0;
}

static int isp_hist_comp_params(struct isp_hist_device *isp_hist,
				struct isp_hist_config *user_cfg)
{
	struct isp_hist_config *cur_cfg = &isp_hist->config;
	int c;

	if ((cur_cfg->source && !user_cfg->source) ||
	    (!cur_cfg->source && user_cfg->source))
		return 1;

	if (cur_cfg->input_bit_width != user_cfg->input_bit_width)
		return 1;

	if (user_cfg->source) {
		if (cur_cfg->hist_h_v_info != user_cfg->hist_h_v_info)
			return 1;
		if (cur_cfg->hist_radd != user_cfg->hist_radd)
			return 1;
		if (cur_cfg->hist_radd_off != user_cfg->hist_radd_off)
			return 1;
	}

	if (cur_cfg->cfa != user_cfg->cfa)
		return 1;

	if (cur_cfg->num_acc_frames != user_cfg->num_acc_frames)
		return 1;

	if (cur_cfg->hist_bins != user_cfg->hist_bins)
		return 1;

	for (c = 0; c < HIST_MAX_WG; c++) {
		if (c == 3 && user_cfg->cfa == HIST_CFA_FOVEONX3)
			break;
		else if (cur_cfg->wg[c] != user_cfg->wg[c])
			return 1;
	}

	if (cur_cfg->num_regions != user_cfg->num_regions)
		return 1;

	/* Regions */
	for (c = 0; c < user_cfg->num_regions; c++) {
		if (cur_cfg->reg_hor[c] != user_cfg->reg_hor[c])
			return 1;
		if (cur_cfg->reg_ver[c] != user_cfg->reg_ver[c])
			return 1;
	}

	return 0;
}

/**
 * isp_hist_update_params - Helper function to check and store user given params.
 * @user_cfg: Pointer to user configuration structure.
 *
 * Returns 0 on success configuration.
 **/
static void isp_hist_update_params(struct isp_hist_device *isp_hist,
				   struct isp_hist_config *user_cfg)
{
	int bit_shift;
	int c;

	if (!isp_hist_comp_params(isp_hist, user_cfg)) {
		isp_hist->config.enable = user_cfg->enable;
		return;
	}

	memcpy(&isp_hist->config, user_cfg, sizeof(*user_cfg));

	if (user_cfg->input_bit_width > HIST_MIN_BIT_WIDTH)
		WRITE_DATA_SIZE(isp_hist->regs.cnt, 0);
	else
		WRITE_DATA_SIZE(isp_hist->regs.cnt, 1);

	WRITE_SOURCE(isp_hist->regs.cnt, user_cfg->source);

	if (user_cfg->source == HIST_SOURCE_MEM) {
		WRITE_HV_INFO(isp_hist->regs.h_v_info, user_cfg->hist_h_v_info);
		WRITE_RADD(isp_hist->regs.hist_radd, user_cfg->hist_radd);
		WRITE_RADD_OFF(isp_hist->regs.hist_radd_off,
			       user_cfg->hist_radd_off);
	}

	WRITE_CFA(isp_hist->regs.cnt, user_cfg->cfa);

	WRITE_WG0(isp_hist->regs.wb_gain, user_cfg->wg[0]);
	WRITE_WG1(isp_hist->regs.wb_gain, user_cfg->wg[1]);
	WRITE_WG2(isp_hist->regs.wb_gain, user_cfg->wg[2]);
	if (user_cfg->cfa == HIST_CFA_BAYER)
		WRITE_WG3(isp_hist->regs.wb_gain, user_cfg->wg[3]);

	/* Regions size and position */
	for (c = 0; c < HIST_MAX_REGIONS; c++) {
		if (c < user_cfg->num_regions) {
			WRITE_REG_HORIZ(isp_hist->regs.reg_hor[c],
					user_cfg->reg_hor[c]);
			WRITE_REG_VERT(isp_hist->regs.reg_ver[c],
				       user_cfg->reg_ver[c]);
		} else {
			isp_hist->regs.reg_hor[c] = 0;
			isp_hist->regs.reg_ver[c] = 0;
		}
	}

	WRITE_NUM_BINS(isp_hist->regs.cnt, user_cfg->hist_bins);
	switch (user_cfg->hist_bins) {
	case HIST_BINS_256:
		bit_shift = user_cfg->input_bit_width - 8;
		break;
	case HIST_BINS_128:
		bit_shift = user_cfg->input_bit_width - 7;
		break;
	case HIST_BINS_64:
		bit_shift = user_cfg->input_bit_width - 6;
		break;
	default: /* HIST_BINS_32 */
		bit_shift = user_cfg->input_bit_width - 5;
		break;
	}
	WRITE_BIT_SHIFT(isp_hist->regs.cnt, bit_shift);

	isp_hist->update = 1;
}

/**
 * isp_hist_configure - API to configure HIST registers.
 * @histcfg: Pointer to user configuration structure.
 *
 * Returns 0 on success configuration.
 **/
int omap34xx_isp_hist_config(struct isp_hist_device *isp_hist,
			     struct isp_hist_config *histcfg)
{
	struct device *dev = to_device(isp_hist);
	struct isp_device *isp = to_isp_device(isp_hist);
	unsigned long irqflags;
	int ret = 0;
	unsigned int size;
	const unsigned int size_bins[] =
			{ HIST_MEM_SIZE_BINS(32), HIST_MEM_SIZE_BINS(64),
			  HIST_MEM_SIZE_BINS(128), HIST_MEM_SIZE_BINS(256) };

	if (!histcfg) {
		dev_err(dev,
			"hist: Null argument in configuration.\n");
		return -EINVAL;
	}

	/* Check Parameters */
	ret = isp_hist_validate_params(histcfg);
	if (ret) {
		dev_dbg(dev, "hist: wrong configure params "
				       "received.\n");
		return ret;
	}

	size = size_bins[histcfg->hist_bins] * histcfg->num_regions;

	/* Alloc buffers */
	ret = ispstat_bufs_alloc(&isp_hist->stat, size);
	if (ret) {
		dev_dbg(dev, "hist: unable to alloc buffers.\n");
		return ret;
	}

	if (!(isp->running == ISP_RUNNING && isp_hist->config.enable))
		isp_hist->active_buf = ispstat_buf_next(&isp_hist->stat);

	spin_lock_irqsave(&isp_hist->lock, irqflags);
	isp_hist->buf_size = size;
	isp_hist_update_params(isp_hist, histcfg);
	spin_unlock_irqrestore(&isp_hist->lock, irqflags);

	return 0;
}

/**
 * omap34xx_isp_hist_request_statistics - Request statistics in Histogram.
 * @histdata: Pointer to data structure.
 *
 * This API allows the user to request for histogram statistics.
 *
 * Returns 0 on successful request.
 **/
int omap34xx_isp_hist_request_statistics(struct isp_hist_device *isp_hist,
					 struct isp_hist_data *histdata)
{
	struct device *dev = to_device(isp_hist);
	struct ispstat_buffer *buf;

	if (!isp_hist->config.enable) {
		dev_err(dev, "hist: statistics requested while engine is not "
			     "configured\n");
		return -EINVAL;
	}

	if (histdata->update & REQUEST_STATISTICS) {
		buf = ispstat_buf_get(&isp_hist->stat,
				      (void *)histdata->hist_statistics_buf,
				      histdata->frame_number);
		if (IS_ERR(buf))
			return PTR_ERR(buf);

		histdata->ts = buf->ts;
		histdata->config_counter = buf->config_counter;
		histdata->frame_number = buf->frame_number;

		ispstat_buf_release(&isp_hist->stat);
	}

	histdata->curr_frame = isp_hist->stat.frame_number;

	return 0;
}

/**
 * isp_hist_init - Module Initialization.
 *
 * Returns 0 if successful.
 **/
int __init isp_hist_init(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_hist_device *isp_hist = &isp->isp_hist;

	spin_lock_init(&isp_hist->lock);
	return ispstat_init(dev, "HIST", &isp_hist->stat, HIST_MAX_BUFF,
			    MAX_FRAME_COUNT);
}

/**
 * isp_hist_cleanup - Module cleanup.
 **/
void isp_hist_cleanup(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	ispstat_free(&isp->isp_hist.stat);
}

/**
 * isphist_save_context - Saves the values of the histogram module registers.
 **/
void isphist_save_context(struct device *dev)
{
	dev_dbg(dev, "hist: Saving context\n");
	isp_save_context(dev, isphist_reg_list);
}

/**
 * isphist_restore_context - Restores the values of the histogram module regs.
 **/
void isphist_restore_context(struct device *dev)
{
	dev_dbg(dev, "hist: Restoring context\n");
	isp_restore_context(dev, isphist_reg_list);
}

/**
 * isp_hist_print_status - Debug print
 **/
static void isp_hist_print_status(struct isp_hist_device *isp_hist)
{
	struct device *dev = to_device(isp_hist);

	dev_dbg(dev, "hist: ISPHIST_PCR = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR));
	dev_dbg(dev, "hist: ISPHIST_CNT = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT));
	dev_dbg(dev, "hist: ISPHIST_WB_GAIN = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_WB_GAIN));
	dev_dbg(dev, "hist: ISPHIST_R0_HORZ = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_HORZ));
	dev_dbg(dev, "hist: ISPHIST_R0_VERT = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_VERT));
	dev_dbg(dev, "hist: ISPHIST_R1_HORZ = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_HORZ));
	dev_dbg(dev, "hist: ISPHIST_R1_VERT = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_VERT));
	dev_dbg(dev, "hist: ISPHIST_R2_HORZ = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_HORZ));
	dev_dbg(dev, "hist: ISPHIST_R2_VERT = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_VERT));
	dev_dbg(dev, "hist: ISPHIST_R3_HORZ = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_HORZ));
	dev_dbg(dev, "hist: ISPHIST_R3_VERT = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_VERT));
	dev_dbg(dev, "hist: ISPHIST_RADD = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD));
	dev_dbg(dev, "hist: ISPHIST_RADD_OFF = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD_OFF));
	dev_dbg(dev, "hist: ISPHIST_H_V_INFO = 0x%08x\n",
		isp_reg_readl(dev, OMAP3_ISP_IOMEM_HIST, ISPHIST_H_V_INFO));
}
