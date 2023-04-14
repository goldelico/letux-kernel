/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <asm/dma.h>
#include <asm/page.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/ingenic_video_nr.h>
#include "ingenic_camera.h"
#include "mipi_csi.h"
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <asm/mach-generic/spaces.h>



int max_buffer_num = 3;
module_param(max_buffer_num, int, 0644);
MODULE_PARM_DESC(max_buffer_num, "cim max buffer number");

static int frame_size_check_flag = 0;
static int showFPS = 0;

struct cim_video_format cim_formats[] = {
	/*MONO */
	{
		.name     = "Y8, GREY",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
	},
	{
		.name     = "Y10, Y10 ",
		.fourcc   = V4L2_PIX_FMT_Y10,
		.depth    = {10},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
		.colorspace = V4L2_COLORSPACE_RAW,
	},
	{
		.name     = "Y12, Y12 ",
		.fourcc   = V4L2_PIX_FMT_Y12,
		.depth    = {12},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_Y12_1X12,
		.colorspace = V4L2_COLORSPACE_RAW,
	},
	/*yuv422 */
	{
		.name     = "YUV422, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "YUV422, YVYU",
		.fourcc   = V4L2_PIX_FMT_YVYU,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "YUV422, UYVY",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "YUV422, VYUY",
		.fourcc   = V4L2_PIX_FMT_VYUY,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	/*RGB*/
	{
		.name     = "RGB565, RGBP",
		.fourcc   = V4L2_PIX_FMT_RGB565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "RBG565, RBGP",
		.fourcc   = INGENIC_V4L2_PIX_FMT_RBG565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = INGENIC_MEDIA_BUS_FMT_RBG565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "BGR565, BGRP",
		.fourcc   = INGENIC_V4L2_PIX_FMT_BGR565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_BGR565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "BRG565, BRGP",
		.fourcc   = INGENIC_V4L2_PIX_FMT_BRG565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = INGENIC_MEDIA_BUS_FMT_BRG565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "GRB565, GRBP",
		.fourcc   = INGENIC_V4L2_PIX_FMT_GRB565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = INGENIC_MEDIA_BUS_FMT_GRB565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "GBR565, GBRP",
		.fourcc   = INGENIC_V4L2_PIX_FMT_GBR565,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = INGENIC_MEDIA_BUS_FMT_GBR565_2X8_LE,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	{
		.name     = "GBR888, GBR3",
		.fourcc   = V4L2_PIX_FMT_RGB24,
		.depth    = {24},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
		.colorspace = V4L2_COLORSPACE_DEFAULT,
	},
	/* RAW8 */
	{
		.name     = "RAW8, 8  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR8,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG8,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG8,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB8,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	/* RAW10 */
	{
		.name     = "RAW10, 10  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR10,
		.depth    = {10},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG10,
		.depth    = {10},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG10,
		.depth    = {10},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB10,
		.depth    = {10},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	/* RAW12 */
	{
		.name     = "RAW12, 12  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR12,
		.depth    = {12},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG12,
		.depth    = {12},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG12,
		.depth    = {12},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB12,
		.depth    = {12},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
};


static struct ingenic_camera_dev *v4l2_dev_to_pcdev(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct ingenic_camera_dev, v4l2_dev);
}

static struct cim_video_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct cim_video_ctx, hdl);
}

static int get_asd_index(struct v4l2_async_subdev *asd)
{
	struct cim_async_subdev *casd = container_of(asd, struct cim_async_subdev, asd);
	return casd->index;
}

static int is_cim_disabled(struct ingenic_camera_dev *pcdev)
{
	return !(cim_readl(pcdev, CIM_ST) & CIM_ST_WORKING);
}

static int ingenic_cim_get_video_nr(int index)
{
	if(index == 0)
		return INGENIC_CIM_VIDEO_NR_0;
	else if(index == 1)
		return INGENIC_CIM_VIDEO_NR_1;
	else
		return 0;
}

void cim_dump_reg(struct ingenic_camera_dev *pcdev)
{
	if(!pcdev) {
		printk("===>>%s,%d pcdev is NULL!\n",__func__,__LINE__);
		return;
	}
	printk("\n***************************************\n");
	printk("GLB_CFG			0x%08x\n", cim_readl(pcdev, GLB_CFG));
	printk("FRM_SIZE		0x%08x\n", cim_readl(pcdev, FRM_SIZE));
	printk("CROP_SITE		0x%08x\n", cim_readl(pcdev, CROP_SITE));
	printk("RESIZE_CFG		0x%08x\n", cim_readl(pcdev, RESIZE_CFG));
	printk("RESIZE_COEF_X		0x%08x\n", cim_readl(pcdev, RESIZE_COEF_X));
	printk("RESIZE_COEF_Y		0x%08x\n", cim_readl(pcdev, RESIZE_COEF_Y));
	printk("SCAN_CFG		0x%08x\n", cim_readl(pcdev, SCAN_CFG));
	printk("QOS_CTRL		0x%08x\n", cim_readl(pcdev, QOS_CTRL));
	printk("QOS_CFG			0x%08x\n", cim_readl(pcdev, QOS_CFG));
	printk("DLY_CFG			0x%08x\n", cim_readl(pcdev, DLY_CFG));
	printk("DES_ADDR		0x%08x\n", cim_readl(pcdev, DES_ADDR));
	printk("CIM_ST			0x%08x\n", cim_readl(pcdev, CIM_ST));
	printk("CIM_CLR_ST		0x%08x\n", cim_readl(pcdev, CIM_CLR_ST));
	printk("CIM_INTC		0x%08x\n", cim_readl(pcdev, CIM_INTC));
	printk("INT_FLAG		0x%08x\n", cim_readl(pcdev, INT_FLAG));
	printk("FRAME_ID		0x%08x\n", cim_readl(pcdev, FRAME_ID));
	printk("ACT_SIZE		0x%08x\n", cim_readl(pcdev, ACT_SIZE));
	printk("DBG_CGC			0x%08x\n", cim_readl(pcdev, DBG_CGC));
	printk("***************************************\n\n");
}

void cim_dump_reg_des(struct ingenic_camera_dev *pcdev)
{
	int version = pcdev->cim_priv->version_num;
	printk("\n***************************************\n");
	printk("DBG_DES next		0x%08x\n", cim_readl(pcdev, DBG_DES));
	printk("DBG_DES WRBK_ADDR:	0x%08x\n", cim_readl(pcdev, DBG_DES));
	printk("DBG_DES WRBK_STRD:	0x%08x\n", cim_readl(pcdev, DBG_DES));
	printk("DBG_DES DES_INTC_t:	0x%08x\n", cim_readl(pcdev, DBG_DES));
	printk("DBG_DES DES_CFG_t:	0x%08x\n", cim_readl(pcdev, DBG_DES));
	if (version == 2) {
		printk("DES_HIST_CFG_t:		0x%08x\n", cim_readl(pcdev, DBG_DES));
		printk("HIST_WRBK_ADDR:		0x%08x\n", cim_readl(pcdev, DBG_DES));
	} else {
		printk("DES_HIST_CFG_t:		0x%08x\n", cim_readl(pcdev, DBG_DES));
		printk("HIST_WRBK_ADDR:		0x%08x\n", cim_readl(pcdev, DBG_DES));
	}
	printk("SF_WRBK_ADDR:		0x%08x\n", cim_readl(pcdev, DBG_DES));
	printk("DBG_DMA:		0x%08x\n", cim_readl(pcdev, DBG_DMA));
	printk("***************************************\n\n");
}

void cim_dump_des(struct ingenic_camera_dev *pcdev)
{
	struct ingenic_camera_desc_v1 *desc_v1 = NULL;
	struct ingenic_camera_desc_v2 *desc_v2 = NULL;
	int i;
	int version = pcdev->cim_priv->version_num;
	if (version == 1) {
		desc_v1 = (struct ingenic_camera_desc_v1 *)pcdev->desc_vaddr;
		printk("\n***************************************\n");
		for(i = 0; i < pcdev->buf_cnt; i++) {
			printk("print des %d\n",i);
			printk("next:			0x%08x\n", desc_v1[i].next);
			printk("WRBK_ADDR:		0x%08x\n", desc_v1[i].WRBK_ADDR);
			printk("WRBK_STRD:		0x%08x\n", desc_v1[i].WRBK_STRD);
			printk("DES_INTC_t:		0x%08x\n", desc_v1[i].DES_INTC_t.d32);
			printk("DES_CFG_t:		0x%08x\n", desc_v1[i].DES_CFG_t.d32);
			printk("DES_HIST_CFG_t:		0x%08x\n", desc_v1[i].DES_HIST_CFG_t.d32);
			printk("HIST_WRBK_ADDR:		0x%08x\n", desc_v1[i].HIST_WRBK_ADDR);
			printk("SF_WRBK_ADDR:		0x%08x\n", desc_v1[i].SF_WRBK_ADDR);
		}
		printk("***************************************\n\n");
	} else {
		desc_v2 = (struct ingenic_camera_desc_v2 *)pcdev->desc_vaddr;
		printk("\n***************************************\n");
		for(i = 0; i < pcdev->buf_cnt; i++) {
			printk("print des %d\n",i);
			printk("next:			0x%08x\n", desc_v2[i].next);
			printk("WRBK_ADDR:		0x%08x\n", desc_v2[i].WRBK_ADDR);
			printk("WRBK_STRD:		0x%08x\n", desc_v2[i].WRBK_STRD);
			printk("DES_INTC_t:		0x%08x\n", desc_v2[i].DES_INTC_t.d32);
			printk("DES_CFG_t:		0x%08x\n", desc_v2[i].DES_CFG_t.d32);
			printk("DES_LUMI_CFG_t:		0x%08x\n", desc_v2[i].DES_LUMI_CFG_t.d32);
			printk("LUMI_WRBK_ADDR:		0x%08x\n", desc_v2[i].LUMI_WRBK_ADDR);
			printk("SF_WRBK_ADDR:		0x%08x\n", desc_v2[i].SF_WRBK_ADDR);
		}
		printk("***************************************\n\n");
	}
}

static void calculate_frame_rate(void)
{
	static ktime_t time_now, time_last;
	unsigned int interval_in_us;
	static unsigned int fpsCount = 0;

	switch(showFPS){
	case 1:
		fpsCount++;
		time_now = ktime_get();
		interval_in_us =time_now - time_last;
		if ( interval_in_us > (NSEC_PER_SEC) ) { /* 1 second = 1000000 us. */
			printk(" CIM FPS: %d\n",fpsCount);
			fpsCount = 0;
			time_last = time_now;
		}
		break;
	case 2:
		time_now = ktime_get();
		interval_in_us =time_now - time_last;
		printk(" CIM tow frame interval in ns: %d\n",interval_in_us);
		time_last = time_now;
		break;
	default:
		if (showFPS > 2) {
			int d, f;
			fpsCount++;
			time_now = ktime_get();
			interval_in_us =time_now - time_last;
			if (interval_in_us > NSEC_PER_SEC * showFPS ) { /* 1 second = 1000000 us. */
				d = fpsCount / showFPS;
				f = (fpsCount * 10) / showFPS - d * 10;
				printk(" CIM FPS: %d.%01d\n", d, f);
				fpsCount = 0;
				time_last = time_now;
			}
		}
		break;
	}
}



static unsigned int fourcc_to_mbus_code(unsigned int fourcc){
	const struct cim_video_format *fmt = NULL;
	int i;
	for(i=0; i<ARRAY_SIZE(cim_formats); i++)
	{
		fmt = &cim_formats[i];
		if(fourcc ==fmt->fourcc)
			return fmt->mbus_code;
	}
	return 0;
}

static unsigned int fourcc_to_mbus_depth(unsigned int fourcc){
	const struct cim_video_format *fmt = NULL;
	int i;
	for(i=0; i<ARRAY_SIZE(cim_formats); i++)
	{
		fmt = &cim_formats[i];
		if(fourcc ==fmt->fourcc)
			return fmt->depth[0];
	}
	return 0;
}


static int ingenic_camera_alloc_desc(struct ingenic_camera_dev *pcdev, unsigned int count)
{

	int desc_size = 0;
	pcdev->buf_cnt = count;
	if (pcdev->cim_priv->version_num == 2) {
		desc_size = sizeof(struct ingenic_camera_desc_v2);
		pcdev->desc_vaddr = dma_alloc_coherent(pcdev->v4l2_dev.dev,
				desc_size * pcdev->buf_cnt,
				(dma_addr_t *)&pcdev->desc_v2_paddr, GFP_KERNEL);
		if (!pcdev->desc_v2_paddr)
			return -ENOMEM;
	} else {
		desc_size = sizeof(struct ingenic_camera_desc_v1);
		pcdev->desc_vaddr = dma_alloc_coherent(pcdev->v4l2_dev.dev,
				desc_size * pcdev->buf_cnt,
				(dma_addr_t *)&pcdev->desc_v1_paddr, GFP_KERNEL);
		if (!pcdev->desc_v1_paddr)
			return -ENOMEM;
	}

	return 0;
}

static void ingenic_camera_free_desc(struct ingenic_camera_dev *pcdev)
{
	int desc_size = 0;
	if (pcdev->cim_priv->version_num == 2) {
		desc_size = sizeof(struct ingenic_camera_desc_v2);

		if(pcdev && pcdev->desc_vaddr) {
			dma_free_coherent(pcdev->v4l2_dev.dev,
					desc_size * pcdev->buf_cnt,
					pcdev->desc_vaddr, (dma_addr_t )pcdev->desc_v2_paddr);

			pcdev->desc_vaddr = NULL;
		}
	} else {
		desc_size = sizeof(struct ingenic_camera_desc_v1);
		if (pcdev && pcdev->desc_vaddr) {
			dma_free_coherent(pcdev->v4l2_dev.dev,
					desc_size * pcdev->buf_cnt,
					pcdev->desc_vaddr, (dma_addr_t )pcdev->desc_v1_paddr);

			pcdev->desc_vaddr = NULL;
		}
	}
}

static int ingenic_init_desc(struct vb2_buffer *vb2) {
	struct cim_video_ctx *ctx = vb2_get_drv_priv(vb2->vb2_queue);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	struct ingenic_camera_dev *pcdev = ctx->cimvideo.pcdev;
	struct ingenic_camera_desc_v1 *desc_v1;
	struct ingenic_camera_desc_v2 *desc_v2;
	dma_addr_t dma_address;
	u32 index = vb2->index;
	u32 pixfmt = user_fmt->pixelformat;
	u32 sizeimage = user_fmt->sizeimage;
	u32 user_width = user_fmt->width;

	dma_address = *(dma_addr_t *)vb2_plane_cookie(vb2, 0);
	if(!dma_address) {
		dev_err(ctx->cimvideo.dev, "Failed to setup DMA address\n");
		return -ENOMEM;
	}

	if (pcdev->cim_priv->version_num == 2) {
		desc_v2 = (struct ingenic_camera_desc_v2 *) pcdev->desc_vaddr;
		desc_v2[index].DES_CFG_t.data.ID = index;
		desc_v2[index].WRBK_ADDR = dma_address;
	} else {
		desc_v1 = (struct ingenic_camera_desc_v1 *) pcdev->desc_vaddr;
		desc_v1[index].DES_CFG_t.data.ID = index;
		desc_v1[index].WRBK_ADDR = dma_address;
	}

	switch (pixfmt){
		case V4L2_PIX_FMT_RGB565:
		case INGENIC_V4L2_PIX_FMT_RBG565:
		case INGENIC_V4L2_PIX_FMT_BGR565:
		case INGENIC_V4L2_PIX_FMT_BRG565:
		case INGENIC_V4L2_PIX_FMT_GRB565:
		case INGENIC_V4L2_PIX_FMT_GBR565:
			if (pcdev->cim_priv->version_num == 2) {
				desc_v2[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_RGB565;
			} else {
				desc_v1[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_RGB565;
			}
			break;
		case V4L2_PIX_FMT_RGB32:
			if (pcdev->cim_priv->version_num == 2) {
				desc_v2[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_RGB888;
			} else {
				desc_v1[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_RGB888;
			}
			break;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
			if (pcdev->cim_priv->version_num == 2) {
				desc_v2[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_YUV422;
			} else {
				desc_v1[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_YUV422;
			}
			break;
		case V4L2_PIX_FMT_GREY:
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SGBRG8:
		case V4L2_PIX_FMT_SGRBG8:
		case V4L2_PIX_FMT_SRGGB8:
		case V4L2_PIX_FMT_SBGGR10:
		case V4L2_PIX_FMT_SGBRG10:
		case V4L2_PIX_FMT_SGRBG10:
		case V4L2_PIX_FMT_SRGGB10:
			if (pcdev->cim_priv->version_num == 2) {
				desc_v2[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_MONO;
			} else {
				desc_v1[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_MONO;
			}
			break;
		default:
			dev_err(ctx->cimvideo.dev, "No support format!\n");
			return -EINVAL;
	}

	if(pcdev->cim_priv->version_num == 2 && ctx->output_y == 1)
		desc_v2[index].DES_CFG_t.data.WRBK_FMT = WRBK_FMT_Y;


	if(pcdev->cim_priv->version_num == 1 && !pcdev->desc_v1_head &&
			!pcdev->desc_v1_tail) {
		pcdev->desc_v1_head = &desc_v1[index];
		pcdev->desc_v1_tail = &desc_v1[index];
		desc_v1[index].DES_INTC_t.data.EOF_MSK = 0;
		desc_v1[index].DES_CFG_t.data.DES_END = 1;
		desc_v1[index].next = 0;
	} else if(pcdev->cim_priv->version_num == 1 && pcdev->desc_v1_head != NULL &&
			pcdev->desc_v1_tail != NULL) {
		pcdev->desc_v1_tail->next = (dma_addr_t)(&pcdev->desc_v1_paddr[index]);
		v4l2_async_notifier_init(&pcdev->notifier);
		pcdev->desc_v1_tail->DES_INTC_t.data.EOF_MSK = 1;
		pcdev->desc_v1_tail->DES_CFG_t.data.DES_END = 0;
		pcdev->desc_v1_tail = &desc_v1[index];

		desc_v1[index].DES_INTC_t.data.EOF_MSK = 0;
		desc_v1[index].DES_CFG_t.data.DES_END = 1;
		desc_v1[index].next = 0;
	} else if (pcdev->cim_priv->version_num == 2 && !pcdev->desc_v2_head &&
			!pcdev->desc_v2_tail) {
		pcdev->desc_v2_head = &desc_v2[index];
		pcdev->desc_v2_tail = &desc_v2[index];
		desc_v2[index].DES_INTC_t.data.EOF_MSK = 0;
		desc_v2[index].DES_CFG_t.data.DES_END = 1;
		desc_v2[index].next = 0;
	} else if(pcdev->cim_priv->version_num == 2 && pcdev->desc_v2_head != NULL &&
			pcdev->desc_v2_tail != NULL) {
		pcdev->desc_v2_tail->next = (dma_addr_t)(&pcdev->desc_v2_paddr[index]);
		pcdev->desc_v2_tail->DES_INTC_t.data.EOF_MSK = 1;
		pcdev->desc_v2_tail->DES_CFG_t.data.DES_END = 0;
		pcdev->desc_v2_tail = &desc_v2[index];

		desc_v2[index].DES_INTC_t.data.EOF_MSK = 0;
		desc_v2[index].DES_CFG_t.data.DES_END = 1;
		desc_v2[index].next = 0;
	}

	if(pcdev->cim_priv->version_num == 2) {
		if(!ctx->lumi_en) {
			desc_v2[index].DES_LUMI_CFG_t.d32 = 0;
			desc_v2[index].LUMI_WRBK_ADDR = 0;
		} else {
			desc_v2[index].DES_LUMI_CFG_t.LUMI_EN = 1;
			desc_v2[index].LUMI_WRBK_ADDR
				= desc_v2[index].WRBK_ADDR + sizeimage;
		}

		if(!ctx->interlace) {
			desc_v2[index].WRBK_STRD = user_width;
			desc_v2[index].SF_WRBK_ADDR = 0;
		} else {
			desc_v2[index].WRBK_STRD = user_width * 2;
			desc_v2[index].SF_WRBK_ADDR =
				desc_v2[index].WRBK_ADDR + user_width * 2;
		}
	} else {
		if(!ctx->hist_en) {
			desc_v1[index].DES_HIST_CFG_t.d32 = 0;
			desc_v1[index].HIST_WRBK_ADDR = 0;
		} else {
			if(pixfmt != V4L2_PIX_FMT_YUYV &&
					pixfmt != V4L2_PIX_FMT_GREY) {
				dev_err(ctx->cimvideo.dev,"fmt not support hist!\n");
				return -EINVAL;
			}
			desc_v1[index].DES_HIST_CFG_t.HIST_EN = 1;
			desc_v1[index].DES_HIST_CFG_t.GAIN_MUL = ctx->hist_gain_mul;
			desc_v1[index].DES_HIST_CFG_t.GAIN_ADD = ctx->hist_gain_add;
			desc_v1[index].HIST_WRBK_ADDR
				= desc_v1[index].WRBK_ADDR + sizeimage;
		}

		if(!ctx->interlace) {
			desc_v1[index].WRBK_STRD = user_width;
			desc_v1[index].SF_WRBK_ADDR = 0;
		} else {
			desc_v1[index].WRBK_STRD = user_width * 2;
			desc_v1[index].SF_WRBK_ADDR =
				desc_v1[index].WRBK_ADDR + user_width * 2;
		}
	}

	return 0;
}

static int ingenic_camera_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cim_video_ctx *ctx = ctrl_to_ctx(ctrl);
	struct ingenic_camera_dev *pcdev = ctx->cimvideo.pcdev;
	struct ingenic_camera_desc_v1 *desc_v1;
	unsigned int tmp;
	int i;
	int expo_width = 0x08;
	int dly_num = 0x80;

	switch (ctrl->id) {
		case INGENIC_CID_CROP_WAY:
		//	img_sz->crop_way = ctrl->val;
			break;
		case INGENIC_CID_LUMI_EN:
			ctx->lumi_en = 1;
			break;
		case INGENIC_CID_OUTPUT_Y:
			ctx->output_y = 1;
			break;
		case INGENIC_CID_POINT1_X:
			tmp = cim_readl(pcdev, LUMI_POINT1);
			tmp &= ~0x7ff;
			tmp |= ctrl->val;
			cim_writel(pcdev, tmp, LUMI_POINT1);
			break;
		case INGENIC_CID_POINT1_Y:
			tmp = cim_readl(pcdev, LUMI_POINT1);
			tmp &= ~(0x7ff << 16);
			tmp |= ctrl->val << 16;
			cim_writel(pcdev, tmp, LUMI_POINT1);
			break;
		case INGENIC_CID_POINT2_X:
			tmp = cim_readl(pcdev, LUMI_POINT2);
			tmp &= ~0x7ff;
			tmp |= ctrl->val;
			cim_writel(pcdev, tmp, LUMI_POINT2);
			break;
		case INGENIC_CID_POINT2_Y:
			tmp = cim_readl(pcdev, LUMI_POINT2);
			tmp &= ~(0x7ff << 16);
			tmp |= ctrl->val << 16;
			cim_writel(pcdev, tmp, LUMI_POINT2);
			break;
		case INGENIC_CID_SNAPSHOT_EN:
			if(!is_cim_disabled(pcdev)){
				printk("too late to set snapshot mode\n");
				return 0;
			}
			ctx->snapshot = 1;
			tmp = cim_readl(pcdev, GLB_CFG);
			tmp |= GLB_CFG_DAT_MODE;
			tmp &= ~GLB_CFG_EXPO_WIDTH_MASK_V2;
			tmp |= (expo_width << GLB_CFG_EXPO_WIDTH_LBIT);
			cim_writel(pcdev, tmp, GLB_CFG);
			tmp = dly_num;
			tmp |= CIM_DLY_EN;
			tmp |= CIM_DLY_MD;
			cim_writel(pcdev, tmp, DLY_CFG);
			break;
		case INGENIC_CID_EXP_PULSE_W:
			if(!is_cim_disabled(pcdev)){
				printk("too late to set exposure width!\n");
				return 0;
			}
			if(!ctx->snapshot)
				return 0;
			tmp = cim_readl(pcdev, GLB_CFG);
			tmp &= ~GLB_CFG_EXPO_WIDTH_MASK_V2;
			tmp |= ((ctrl->val & 0x1f) << GLB_CFG_EXPO_WIDTH_LBIT);
			cim_writel(pcdev, tmp, GLB_CFG);
			expo_width = ctrl->val;
			break;
		case INGENIC_CID_SNAPSHOT_DELAY_T:
			if(!is_cim_disabled(pcdev)){
				printk("too late to set delay number!\n");
				return 0;
			}
			if(!ctx->snapshot)
				return 0;
			tmp = ctrl->val;
			tmp |= CIM_DLY_EN;
			tmp |= CIM_DLY_MD;
			cim_writel(pcdev, tmp, DLY_CFG);
			dly_num = ctrl->val;
			break;
		case INGENIC_CID_SCALE_SHARP:
			tmp = cim_readl(pcdev,RESIZE_CFG);
			tmp &= ~CIM_SHARPL_MASK;
			tmp |= ctrl->val << CIM_SHARPL_LBIT;
			cim_writel(pcdev, tmp, RESIZE_CFG);
			break;
		case INGENIC_CID_HIST_EN:
			ctx->hist_en = 1;
			break;
		case INGENIC_CID_HIST_GAIN_ADD:
			ctx->hist_gain_add = ctrl->val;
			desc_v1 = (struct ingenic_camera_desc_v1 *) pcdev->desc_vaddr;
			if(!ctx->hist_en || !desc_v1) {
				dev_dbg(pcdev->dev, "setting hist_add are not satisfied!\n");
				return 0;
			}
			for(i = 0; i < pcdev->buf_cnt; i++)
				desc_v1[i].DES_HIST_CFG_t.GAIN_ADD = ctx->hist_gain_add;
			break;
		case INGENIC_CID_HIST_GAIN_MUL:
			ctx->hist_gain_mul = ctrl->val;
			desc_v1 = (struct ingenic_camera_desc_v1 *) pcdev->desc_vaddr;
			if(!ctx->hist_en || !desc_v1) {
				dev_dbg(pcdev->dev, "setting hist_mul are not satisfied!\n");
				return 0;
			}
			for(i = 0; i < pcdev->buf_cnt; i++)
				desc_v1[i].DES_HIST_CFG_t.GAIN_MUL = ctx->hist_gain_mul;
			break;
		default:
			dev_err(pcdev->dev, "ctrl id not supported!\n");
			break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops ingenic_camera_ctrl_ops = {
	.s_ctrl = ingenic_camera_s_ctrl,
};

static const struct v4l2_ctrl_config img_crop_way = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_CROP_WAY,
	.name	= "crop way",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= CROP_SPECIFY_ZONE,
	.max	= CROP_BASE_MAX_PIC,
	.min	= CROP_SPECIFY_ZONE,
	.max	= CROP_BASE_MAX_PIC,
	.step	= 1,
};

static const struct v4l2_ctrl_config lumi_en = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_LUMI_EN,
	.type	= V4L2_CTRL_TYPE_BUTTON,
	.name	= "lumi enable",
};

static const struct v4l2_ctrl_config output_y = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_OUTPUT_Y,
	.type	= V4L2_CTRL_TYPE_BUTTON,
	.name	= "output Y enable",
};

static const struct v4l2_ctrl_config point1_x = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_POINT1_X,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "point1 x-offset",
	.def	= 0x78,
	.min	= 0,
	.max	= 0x7ff,
	.step	= 1,
};

static const struct v4l2_ctrl_config point1_y = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_POINT1_Y,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "point1 y-offset",
	.def	= 0x78,
	.min	= 0,
	.max	= 0x7ff,
	.step	= 1,
};

static const struct v4l2_ctrl_config point2_x = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_POINT2_X,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "point2 x-offset",
	.def	= 0xf0,
	.min	= 0,
	.max	= 0x7ff,
	.step	= 1,
};

static const struct v4l2_ctrl_config point2_y = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_POINT2_Y,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "point2 y-offset",
	.def	= 0xf0,
	.min	= 0,
	.max	= 0x7ff,
	.step	= 1,
};
static const struct v4l2_ctrl_config snapshot_en = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_SNAPSHOT_EN,
	.type	= V4L2_CTRL_TYPE_BUTTON,
	.name	= "snapshot enable",
};

static const struct v4l2_ctrl_config exp_pulse_w= {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_EXP_PULSE_W,
	.name	= "pulse width",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= 0xf,
	.min	= 0x01,
	.max	= 0x1f,
	.step	= 1,
};

static const struct v4l2_ctrl_config delay_t= {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_SNAPSHOT_DELAY_T,
	.name	= "delay number",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= 0x00,
	.min	= 0x00,
	.max	= 0x800000,
	.step	= 1,
};

static const struct v4l2_ctrl_config img_scale_sharp = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_SCALE_SHARP,
	.name	= "scale sharp",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= 2,
	.min	= 0,
	.max	= 3,
	.step	= 1,
};

static const struct v4l2_ctrl_config hist_en = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_HIST_EN,
	.type	= V4L2_CTRL_TYPE_BUTTON,
	.name	= "histgram enable",
};

static const struct v4l2_ctrl_config hist_gain_add = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_HIST_GAIN_ADD,
	.name	= "hist gain add",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= 0,
	.min	= 0,
	.max	= 0xff,
	.step	= 1,
};

static const struct v4l2_ctrl_config hist_gain_mul = {
	.ops	= &ingenic_camera_ctrl_ops,
	.id	= INGENIC_CID_HIST_GAIN_MUL,
	.name	= "hist gain mul",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.def	= 0x20,
	.min	= 0,
	.max	= 0xff,
	.step	= 1,
};


static int ingenic_camera_wakeup(struct ingenic_camera_dev *pcdev) {
	struct ingenic_buffer *buf = pcdev->active;
	struct vb2_v4l2_buffer *vbuf = &pcdev->active->vb;
	int id = cim_readl(pcdev, FRAME_ID);

	if(!buf || vbuf->vb2_buf.index != id) {
		dev_err(pcdev->dev, "cim buf synchronous problems!\n");
		return -EINVAL;
	}

	list_del_init(&buf->list);
	vbuf->vb2_buf.timestamp = ktime_get_ns();
	vbuf->sequence = pcdev->sequence++;
	vb2_buffer_done(&vbuf->vb2_buf, VB2_BUF_STATE_DONE);

	return 0;
}

static irqreturn_t ingenic_camera_irq_handler(int irq, void *data) {
	struct ingenic_camera_dev *pcdev = (struct ingenic_camera_dev *)data;
	struct ingenic_camera_desc_v1 *desc_v1_p;
	struct ingenic_camera_desc_v2 *desc_v2_p;
	unsigned long status = 0;
	unsigned long flags = 0;
	unsigned long regval = 0;

	spin_lock_irqsave(&pcdev->lock, flags);

	/* read interrupt flag register */
	status = cim_readl(pcdev,INT_FLAG);
	if (!status) {
		dev_err(pcdev->dev, "cim irq_flag is NULL! \n");
		goto out;
	}

	if(status & CIM_INT_FLAG_EOW) {
		cim_writel(pcdev, CIM_INT_FLAG_EOW, CIM_CLR_ST);

		if(status & CIM_INT_FLAG_EOF) {
			cim_writel(pcdev, CIM_INT_FLAG_EOF, CIM_CLR_ST);
		}

		if(ingenic_camera_wakeup(pcdev))
			goto out;

		if (list_empty(&pcdev->video_buffer_list)) {
			pcdev->active = NULL;
			goto out;
		}

		/* start next dma frame. */
		if (pcdev->cim_priv->version_num == 2) {
			desc_v2_p = pcdev->desc_v2_paddr;
			regval = (unsigned long)(pcdev->desc_v2_head->next);
			pcdev->desc_v2_head =
				(struct ingenic_camera_desc_v2 *)((regval)|0xa0000000);
		} else {
			desc_v1_p = pcdev->desc_v1_paddr;
			regval = (unsigned long)(pcdev->desc_v1_head->next);
			pcdev->desc_v1_head =
				(struct ingenic_camera_desc_v1 *)(regval|0xa0000000);
		}

		cim_writel(pcdev, regval, DES_ADDR);
		cim_writel(pcdev, CIM_CTRL_START, CIM_CTRL);

		pcdev->active =
			list_entry(pcdev->video_buffer_list.next, struct ingenic_buffer, list);
		goto out;
	}

	if(status & CIM_INT_FLAG_EOF) {
		cim_writel(pcdev, CIM_INT_FLAG_EOF, CIM_CLR_ST);

		if(ingenic_camera_wakeup(pcdev))
			goto out;

		pcdev->active =
			list_entry(pcdev->video_buffer_list.next, struct ingenic_buffer, list);
		if (pcdev->cim_priv->version_num == 2) {
			pcdev->desc_v2_head =
				(struct ingenic_camera_desc_v2 *)((pcdev->desc_v2_head->next)|0xa0000000);
		} else
			pcdev->desc_v1_head =
				(struct ingenic_camera_desc_v1 *)((pcdev->desc_v1_head->next)|0xa0000000);

		goto out;
	}

	if (status & CIM_INT_FLAG_SZ_ERR) {
		unsigned int val;
		val = cim_readl(pcdev, ACT_SIZE);
		cim_writel(pcdev, CIM_CLR_SZ_ERR, CIM_CLR_ST);
		dev_err(pcdev->dev, "cim size err! w = %d h = %d\n", val&0x3ff, val>>16&0x3ff);
		goto out;
	}

	if (status & CIM_INT_FLAG_OVER) {
		cim_writel(pcdev, CIM_INT_FLAG_OVER, CIM_CLR_ST);
		dev_err(pcdev->dev, "cim overflow! \n");
		goto out;
	}

	if(status & CIM_INT_FLAG_GSA) {
		cim_writel(pcdev, CIM_INT_FLAG_GSA, CIM_CLR_ST);
		printk("cim general stop! \n");
		goto out;
	}

	if(status & CIM_INT_FLAG_SOF) {
		cim_writel(pcdev, CIM_INT_FLAG_SOF, CIM_CLR_ST);
		goto out;
	}

out:
	spin_unlock_irqrestore(&pcdev->lock, flags);
	return IRQ_HANDLED;
}

/*ioctl_ops*/
static int ingenic_camera_querycap(struct file *file, void *fh,
			       struct v4l2_capability *cap)
{
	struct cim_video_ctx *ctx = video_drvdata(file);

	strlcpy(cap->driver, "ingenic-camera", sizeof(cap->driver));
	cap->version = VERSION_CODE;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int ingenic_camera_enum_fmt_vid_cap(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	const struct cim_video_format *fmt = NULL;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int i, ret = 0;

	code.index = f->index;
	ret = v4l2_subdev_call(cimvideo->subdev, pad, enum_mbus_code, NULL, &code);
	if (ret < 0)
		return -EINVAL;

	for(i=0; i < ARRAY_SIZE(cim_formats); i++) {
		fmt = &cim_formats[i];
		if(fmt->mbus_code == code.code)
		{
			strlcpy(f->description, fmt->name, sizeof(f->description));
			f->pixelformat = fmt->fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int ingenic_camera_set_bus_param(struct cim_video_ctx *ctx) {
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	struct cim_async_subdev *casd = &pcdev->casd[cimvideo->index];
	struct ingenic_image_sz *img_sz = &ctx->img_sz;
	int version = pcdev->cim_priv->version_num;
	unsigned long common_flags;
	unsigned long glb_cfg = 0;
	unsigned long tmp;
	unsigned long intc = 0;

	glb_cfg = cim_readl(pcdev,GLB_CFG);
	if(casd->bus_type == V4L2_MBUS_CSI2_DPHY) {
		/***Csi2 interface requirements***/
		common_flags = V4L2_MBUS_MASTER |
			V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			V4L2_MBUS_PCLK_SAMPLE_RISING;
		glb_cfg |= GLB_CFG_DAT_IF_SEL;

	} else if(casd->bus_type == V4L2_MBUS_PARALLEL) {
		common_flags = casd->parallel.flags;
		glb_cfg &= ~GLB_CFG_DAT_IF_SEL;
	} else if(casd->bus_type == V4L2_MBUS_BT656) {
		common_flags = casd->parallel.flags;
		if(common_flags & V4L2_MBUS_SLAVE) {
			dev_err(cimvideo->dev, "bt656_mbus slave mode not supported!\n");
			return -EINVAL;
		}
		glb_cfg &= ~GLB_CFG_DAT_IF_SEL;
	} else {
		dev_err(cimvideo->dev, "cim interface not supported!\n");
		return -EINVAL;
	}

	/*PCLK Polarity Set*/
	glb_cfg = (common_flags & V4L2_MBUS_PCLK_SAMPLE_FALLING) ?
		(glb_cfg | GLB_CFG_DE_PCLK) : (glb_cfg & (~GLB_CFG_DE_PCLK));

	/*VSYNC Polarity Set*/
	glb_cfg = (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW) ?
		(glb_cfg | GLB_CFG_DL_VSYNC) : (glb_cfg & (~GLB_CFG_DL_VSYNC));

	/*HSYNC Polarity Set*/
	glb_cfg = (common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW) ?
		(glb_cfg | GLB_CFG_DL_HSYNC) : (glb_cfg & (~GLB_CFG_DL_HSYNC));

	glb_cfg &= (version == 1) ? (~GLB_CFG_C_ORDER_MASK_V1) : (~GLB_CFG_C_ORDER_MASK_V2);
	glb_cfg &= (~GLB_CFG_C_ORDER_MASK_V1);

	if(casd->bus_type != V4L2_MBUS_BT656) {
		switch(fourcc_to_mbus_code(user_fmt->pixelformat)) {
			case MEDIA_BUS_FMT_RGB565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_RGB);
				break;
			case INGENIC_MEDIA_BUS_FMT_RBG565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_RBG);
				break;
			case MEDIA_BUS_FMT_BGR565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_BGR);
				break;
			case INGENIC_MEDIA_BUS_FMT_BRG565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_BRG);
				break;
			case INGENIC_MEDIA_BUS_FMT_GRB565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_GRB);
				break;
			case INGENIC_MEDIA_BUS_FMT_GBR565_2X8_LE:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB565 | GLB_CFG_C_ORDER_GBR);
				break;
			case MEDIA_BUS_FMT_UYVY8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_YUYV422 | GLB_CFG_C_ORDER_UYVY);
				break;
			case MEDIA_BUS_FMT_VYUY8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_YUYV422 | GLB_CFG_C_ORDER_VYUY);
				break;
			case MEDIA_BUS_FMT_YUYV8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_YUYV422 | GLB_CFG_C_ORDER_YUYV);
				break;
			case MEDIA_BUS_FMT_YVYU8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_YUYV422 | GLB_CFG_C_ORDER_YVYU);
				break;
			case MEDIA_BUS_FMT_RGB888_1X24:
				glb_cfg |= (GLB_CFG_ORG_FMT_RGB888 | GLB_CFG_C_ORDER_RGB);
				break;
			case MEDIA_BUS_FMT_Y8_1X8:
				glb_cfg |= GLB_CFG_ORG_FMT_MONO;
				break;
			case MEDIA_BUS_FMT_Y10_1X10:
				glb_cfg |= GLB_CFG_ORG_FMT_10BIT_MONO;
				break;
			case MEDIA_BUS_FMT_Y12_1X12:
				glb_cfg |= GLB_CFG_ORG_FMT_12BIT_MONO;
				break;
			case MEDIA_BUS_FMT_SBGGR8_1X8:
				glb_cfg |= (GLB_CFG_C_ORDER_BGGR | GLB_CFG_ORG_FMT_8BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGBRG8_1X8:
				glb_cfg |= (GLB_CFG_C_ORDER_GBRG | GLB_CFG_ORG_FMT_8BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGRBG8_1X8:
				glb_cfg |= (GLB_CFG_C_ORDER_GRBG | GLB_CFG_ORG_FMT_8BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SRGGB8_1X8:
				glb_cfg |= (GLB_CFG_C_ORDER_RGGB | GLB_CFG_ORG_FMT_8BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SBGGR10_1X10:
				glb_cfg |= (GLB_CFG_C_ORDER_BGGR | GLB_CFG_ORG_FMT_10BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGBRG10_1X10:
				glb_cfg |= (GLB_CFG_C_ORDER_GBRG | GLB_CFG_ORG_FMT_10BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGRBG10_1X10:
				glb_cfg |= (GLB_CFG_C_ORDER_GRBG | GLB_CFG_ORG_FMT_10BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SRGGB10_1X10:
				glb_cfg |= (GLB_CFG_C_ORDER_RGGB | GLB_CFG_ORG_FMT_10BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SBGGR12_1X12:
				glb_cfg |= (GLB_CFG_C_ORDER_BGGR | GLB_CFG_ORG_FMT_12BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGBRG12_1X12:
				glb_cfg |= (GLB_CFG_C_ORDER_GBRG | GLB_CFG_ORG_FMT_12BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SGRBG12_1X12:
				glb_cfg |= (GLB_CFG_C_ORDER_GRBG | GLB_CFG_ORG_FMT_12BIT_RAW);
				break;
			case MEDIA_BUS_FMT_SRGGB12_1X12:
				glb_cfg |= (GLB_CFG_C_ORDER_RGGB | GLB_CFG_ORG_FMT_12BIT_RAW);
				break;
			default:
				dev_err(cimvideo->dev, "Providing code %x not support\n", fourcc_to_mbus_code(user_fmt->pixelformat));
				return -EINVAL;
		}
	} else {
		unsigned int sf_cfg = 0;
		switch(fourcc_to_mbus_code(user_fmt->pixelformat)) {
			case MEDIA_BUS_FMT_YUYV8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_ITU656 | GLB_CFG_C_ORDER_YUYV);
				break;
			case MEDIA_BUS_FMT_YVYU8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_ITU656 | GLB_CFG_C_ORDER_YVYU);
				break;
			case MEDIA_BUS_FMT_UYVY8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_ITU656 | GLB_CFG_C_ORDER_UYVY);
				break;
			case MEDIA_BUS_FMT_VYUY8_2X8:
				glb_cfg |= (GLB_CFG_ORG_FMT_ITU656 | GLB_CFG_C_ORDER_VYUY);
				break;
			default:
				dev_err(cimvideo->dev, "Providing code %d not support\n",
					fourcc_to_mbus_code(user_fmt->pixelformat));
				break;
		}

		if(user_fmt->field == V4L2_FIELD_NONE) {
			ctx->interlace = 0;
		} else {
			switch(user_fmt->field) {
			case V4L2_FIELD_INTERLACED_TB:
			case V4L2_FIELD_INTERLACED:
				sf_cfg &= ~CIM_F_ORDER;
				break;
			case V4L2_FIELD_INTERLACED_BT:
				sf_cfg |= CIM_F_ORDER;
				break;
			default:
				dev_err(cimvideo->dev,
					"itu656 unsupported field");
				break;
			}
			sf_cfg |= CIM_SCAN_MD;
			sf_cfg &= ~CIM_SF_HEIGHT_MASK;
			sf_cfg |= ((img_sz->src_h / 2) & 0xfff);
			ctx->interlace = 1;
		}
		cim_writel(pcdev, sf_cfg, SCAN_CFG);
	}

	glb_cfg |= GLB_CFG_BURST_LEN_32;
	glb_cfg |= GLB_CFG_AUTO_RECOVERY;
	if(frame_size_check_flag == 1)
		glb_cfg |= GLB_CFG_SIZE_CHK;
	else
		glb_cfg &= ~GLB_CFG_SIZE_CHK;

#ifdef CONFIG_CAMERA_USE_SNAPSHOT
	tmp = CONFIG_SNAPSHOT_PULSE_WIDTH;
	if(!tmp) {
		tmp = 0x08;
	}
	glb_cfg |= GLB_CFG_DAT_MODE;
	if (version == 1) {
		glb_cfg &= ~GLB_CFG_EXPO_WIDTH_MASK_V1;
		glb_cfg |= (((tmp-1) & 0x7) << GLB_CFG_EXPO_WIDTH_LBIT);
	} else {
		glb_cfg &= ~GLB_CFG_EXPO_WIDTH_MASK_V2;
		glb_cfg |= ((tmp & 0x1f) << GLB_CFG_EXPO_WIDTH_LBIT);
	}
	tmp = CONFIG_SNAPSHOT_FRAME_DELAY & 0xffffff;
	tmp |= CIM_DLY_EN;
	tmp |= CIM_DLY_MD;
	if (version == 2)
		tmp |= CIM_EXPO_WIDTH_EN;
	cim_writel(pcdev, tmp, DLY_CFG);
#else
	cim_writel(pcdev, 0, DLY_CFG);
#endif

	if(ctx->interlace) {
		img_sz->src_h = img_sz->src_h / 2;
	}
	tmp = (img_sz->src_w << CIM_CROP_WIDTH_LBIT) |
		(img_sz->src_h << CIM_CROP_HEIGHT_LBIT);
	cim_writel(pcdev, tmp, FRM_SIZE);
	tmp = cim_readl(pcdev, FRM_SIZE);
	cim_writel(pcdev, 0, CROP_SITE);

	/* enable stop of frame interrupt. */
	intc |= CIM_INTC_MSK_EOW;

	/* enable end of frame interrupt,
	 * Work together with DES_INTC.EOF_MSK,
	 * only when both are 1 will generate interrupt. */
	intc |= CIM_INTC_MSK_EOF;

	/* enable general stop of frame interrupt. */
	intc |= CIM_INTC_MSK_GSA;

	/* enable rxfifo overflow interrupt */
	intc |= CIM_INTC_MSK_OVER;

	/* enable size check err */
	if(frame_size_check_flag == 1)
		intc |= CIM_INTC_MSK_SZ_ERR;
	else
		intc &= ~CIM_INTC_MSK_SZ_ERR;

	cim_writel(pcdev, 0, DBG_CGC);
	cim_writel(pcdev, glb_cfg, GLB_CFG);
	cim_writel(pcdev, intc, CIM_INTC);
	cim_writel(pcdev, 0, QOS_CTRL);

	return 0;
}

static int __ingenic_camera_try_fmt(struct cim_video_ctx *ctx, struct v4l2_format *f)
{
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_subdev *sd = cimvideo->subdev;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	struct v4l2_subdev_pad_config cfg;
	struct v4l2_mbus_framefmt *mf = &format.format;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct ingenic_image_sz *img_sz = &ctx->img_sz;
	int depth = 0;
	int ret = 0;

	v4l_bound_align_image(&pix->width, 100, 2046,
			(pix->pixelformat == V4L2_PIX_FMT_YUYV ? 1 : 0),
			&pix->height, 100, 2046, 0, 0);

	mf->width	= pix->width;
	mf->height	= pix->height;
	mf->field	= pix->field;
	mf->colorspace	= pix->colorspace;
	mf->code	= fourcc_to_mbus_code(pix->pixelformat);

	ret = v4l2_subdev_call(sd, pad, set_fmt, &cfg, &format);
	if (ret < 0){
		return ret;
	}

	pix->width = mf->width;
	pix->height = mf->height;
	pix->field = mf->field;
	pix->colorspace = mf->colorspace;
	depth = fourcc_to_mbus_depth(pix->pixelformat);
	pix->bytesperline = pix->width * depth / 8;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int ingenic_camera_try_fmt_vid_cap(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	return __ingenic_camera_try_fmt(ctx, f);
}

static int ingenic_camera_s_fmt_vid_cap(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_subdev *sd = cimvideo->subdev;
	struct ingenic_image_sz *img_sz = &ctx->img_sz;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_mbus_framefmt *mf = &format.format;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	unsigned int pad = 0;
	int ret = 0;

	printk("%s:%d\n",__func__, __LINE__);
	ret = __ingenic_camera_try_fmt(ctx, f);
	if(ret)
		return ret;

	mf->width	= pix->width;
	mf->height	= pix->height;
	mf->field	= pix->field;
	mf->colorspace	= pix->colorspace;
	mf->code	= fourcc_to_mbus_code(pix->pixelformat);

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &format);
	if (ret < 0){
		return ret;
	}

	img_sz->src_w = mf->width;
	img_sz->src_h = mf->height;

	if(unlikely(img_sz->rsz_flag == true)) {
		dev_err(cimvideo->dev, "Set_crop should be behind set_fmt!\n");
		return -EINVAL;
	}

	user_fmt->width = pix->width;
	user_fmt->height = pix->height;
	user_fmt->bytesperline = pix->bytesperline;
	user_fmt->sizeimage = pix->sizeimage;
	user_fmt->pixelformat = pix->pixelformat;
	user_fmt->colorspace = pix->colorspace;
	user_fmt->field = pix->field;

	return ingenic_camera_set_bus_param(ctx);
}

static int ingenic_camera_g_fmt_vid_cap(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;

	printk("%s:%d\n",__func__, __LINE__);
	memcpy(&f->fmt.pix, user_fmt, sizeof(struct v4l2_pix_format));

	return 0;
}

static int ingenic_camera_g_selection(struct file *file, void *fh,
				  struct v4l2_selection *s)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_subdev *sd = cimvideo->subdev;
	struct ingenic_image_sz *img_sz = &ctx->img_sz;
	struct v4l2_subdev_selection sel;
	struct v4l2_subdev_pad_config cfg;
	int ret = 0;

	switch (s->target) {
		case V4L2_SEL_TGT_CROP:
			printk("%s:%d\n",__func__, __LINE__);
		case V4L2_SEL_TGT_CROP_DEFAULT:
			printk("%s:%d\n",__func__, __LINE__);
			s->r.left= img_sz->c_left;
			s->r.top= img_sz->c_top;
			s->r.width= img_sz->c_w;
			s->r.height= img_sz->c_h;
			return 0;
		case V4L2_SEL_TGT_CROP_BOUNDS:
			printk("%s:%d\n",__func__, __LINE__);
			ret = v4l2_subdev_call(sd, pad, get_selection, &cfg, &sel);
			if(!ret)
				s->r =sel.r;
			else
				memset(&s->r, sizeof(struct v4l2_rect), 0);
			return 0;
	}
	printk("%s:%d\n",__func__, __LINE__);
	return -EINVAL;
}

static int ingenic_camera_check_crop(struct cim_video_ctx *ctx, struct v4l2_rect *r)
{
	struct v4l2_rect rect_w = *r;
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	u32 pixfmt = user_fmt->pixelformat;
	unsigned int walign;
	unsigned int halign;

	if(user_fmt->field != V4L2_FIELD_NONE) {
		dev_err(ctx->cimvideo.dev, "field not supported by crop!\n");
		return -EINVAL;
	}

	if((rect_w.width + rect_w.left > 2046) ||
		(rect_w.height + rect_w.top > 2046)) {
		dev_err(ctx->cimvideo.dev, "crop bounds check failed!\n");
		return -EINVAL;
	}

	switch(pixfmt){
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_UYVY:
		walign = 1;
		halign = 0;
		break;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		walign = 1;
		halign = 1;
		break;
	default:
		walign = 0;
		halign = 0;
		break;
	}

	v4l_bound_align_image(&rect_w.width, 100, 2046, walign,
			&rect_w.height, 100, 2046, halign, 0);

	if((rect_w.width != r->width) ||
		(rect_w.height != r->height)) {
		dev_err(ctx->cimvideo.dev, "crop sizes check failed!\n");
		return -EINVAL;
	}

	if((pixfmt == V4L2_PIX_FMT_YUYV)
		&& (rect_w.left % 2)) {
		dev_err(ctx->cimvideo.dev, "crop align check failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int ingenic_camera_s_selection(struct file *file, void *fh,
				  struct v4l2_selection *s)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_subdev *sd = cimvideo->subdev;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	struct v4l2_rect rect_w = s->r;
	struct v4l2_subdev_selection sel = {
		.r = rect_w,
	};
	struct ingenic_image_sz *img_sz = &ctx->img_sz;
	unsigned int framesize = 0;
	unsigned int cropsite = 0;
	int depth = 0;
	int width, height;
	int ret;

	ret = v4l2_subdev_call(sd, pad, set_selection, NULL ,&sel);
	if(!ret){
		/*crop by sensor*/
		dev_dbg(cimvideo->dev, "Sensor cropped %dx%d\n",
				rect_w.width, rect_w.height);

		img_sz->src_w		= rect_w.width;
		img_sz->src_h		= rect_w.height;

		img_sz->c_left		= sel.r.left;
		img_sz->c_top		= sel.r.top;
		img_sz->c_w		= sel.r.width;
		img_sz->c_h		= sel.r.height;

		framesize = (img_sz->src_w << CIM_CROP_WIDTH_LBIT) |
			(img_sz->src_h << CIM_CROP_HEIGHT_LBIT);
		cropsite = 0;
	} else {
		/*crop by cim*/
		ret = ingenic_camera_check_crop(ctx, &rect_w);
		if (ret)
			return ret;

		width = img_sz->src_w;
		height = img_sz->src_h;

		if(((rect_w.left + rect_w.width) > width) ||
				((rect_w.top + rect_w.height) > height)) {
			dev_err(cimvideo->dev, "please enum appropriate framesizes!\n");
			return -EINVAL;
		}

		img_sz->c_left		= rect_w.left;
		img_sz->c_top		= rect_w.top;
		img_sz->c_w		= rect_w.width;
		img_sz->c_h		= rect_w.height;


		framesize = (img_sz->c_w << CIM_CROP_WIDTH_LBIT) |
			(img_sz->c_h << CIM_CROP_HEIGHT_LBIT);
		cropsite = (img_sz->c_left << CIM_CROP_X_LBIT) |
			(img_sz->c_top << CIM_CROP_Y_LBIT);

	}
	user_fmt->width = rect_w.width;
	user_fmt->height = rect_w.height;
	depth = fourcc_to_mbus_depth(user_fmt->pixelformat);
	user_fmt->bytesperline = user_fmt->width * depth / 8;
	user_fmt->sizeimage = user_fmt->bytesperline * user_fmt->height;

	img_sz->rsz_flag		= true;

	cim_writel(pcdev, framesize, FRM_SIZE);
	cim_writel(pcdev, cropsite, CROP_SITE);

	printk("%s:%d\n",__func__, __LINE__);
	return 0;
}

static int ingenic_camera_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct v4l2_subdev *sd = cimvideo->subdev;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	ret = v4l2_subdev_call(sd, pad, enum_frame_size, NULL, &fse);
	if (ret < 0)
		return ret;

	if (fse.min_width == fse.max_width &&
	    fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
		return 0;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = fse.min_width;
	fsize->stepwise.max_width = fse.max_width;
	fsize->stepwise.min_height = fse.min_height;
	fsize->stepwise.max_height = fse.max_height;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int ingenic_camera_s_input(struct file *file, void *fh, unsigned int i)
{
	return 0;
}

static const struct v4l2_ioctl_ops cim_video_ioctl_ops = {
	.vidioc_querycap                = ingenic_camera_querycap,
	.vidioc_enum_fmt_vid_cap        = ingenic_camera_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = ingenic_camera_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = ingenic_camera_s_fmt_vid_cap,
	.vidioc_g_selection		= ingenic_camera_g_selection,
	.vidioc_s_selection		= ingenic_camera_s_selection,
	.vidioc_try_fmt_vid_cap         = ingenic_camera_try_fmt_vid_cap,
	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_querybuf                = vb2_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_streamon                = vb2_ioctl_streamon,
	.vidioc_streamoff               = vb2_ioctl_streamoff,
	.vidioc_enum_framesizes         = ingenic_camera_enum_framesizes,
	.vidioc_s_input			= ingenic_camera_s_input,
//	.vidioc_g_parm                  = isp_video_g_parm,
//	.vidioc_s_ctrl                  = isp_video_s_ctrl,
//	.vidioc_g_ctrl                  = isp_video_g_ctrl,
};


/*video_queue_ops*/
static int ingenic_videobuf_init(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);
	struct ingenic_buffer *buf = container_of(vbuf, struct ingenic_buffer, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}


static int ingenic_videobuf_setup(struct vb2_queue *q,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct cim_video_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	int max_video_mem = pcdev->cim_priv->max_video_mem;
	int size;

	*num_planes = 1;
	size = user_fmt->sizeimage;
	if(ctx->hist_en)
		size += CIM_HIST_SIZE;
	else if(ctx->lumi_en){
		size += CIM_LUMI_SIZE;
	}

	if (!*num_buffers || *num_buffers > max_buffer_num)
		*num_buffers = max_buffer_num;

	if (size * *num_buffers > max_video_mem)
		*num_buffers = max_video_mem / size;

	sizes[0] = size;
	alloc_devs[0] = cimvideo->pcdev->dev;

	pcdev->start_streaming_called = 0;
	pcdev->sequence = 0;
	pcdev->active = NULL;

	if(ingenic_camera_alloc_desc(pcdev, *num_buffers))
		return -ENOMEM;
	dev_info(cimvideo->dev, "%s, count=%d, size=%d\n", __func__,
		*num_buffers, size);

	return 0;
}

static	int ingenic_buffer_prepare(struct vb2_buffer *vb)
{
	struct cim_video_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format *user_fmt = &ctx->format.fmt.pix;
	int ret = 0;

	dev_vdbg(ctx->cimvideo.dev, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	vb2_set_plane_payload(vb, 0, user_fmt->sizeimage);
	if (vb2_plane_vaddr(vb, 0) &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		ret = -EINVAL;
		return ret;
	}

	return 0;

}


static void ingenic_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cim_video_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ingenic_camera_dev *pcdev = ctx->cimvideo.pcdev;
	struct ingenic_buffer *buf = container_of(vbuf, struct ingenic_buffer, vb);
	struct ingenic_camera_desc_v1 *desc_v1_v;
	struct ingenic_camera_desc_v1 *desc_v1_p;
	struct ingenic_camera_desc_v2 *desc_v2_v;
	struct ingenic_camera_desc_v2 *desc_v2_p;
	unsigned long flags;
	unsigned int regval;
	int index = vb->index;

	spin_lock_irqsave(&pcdev->lock, flags);

	list_add_tail(&buf->list, &pcdev->video_buffer_list);
	if (!pcdev->active) {
		pcdev->active = buf;
	}

	if(!pcdev->start_streaming_called) {
		goto out;
	}

	if((index > pcdev->buf_cnt) || (index < 0)) {
		dev_err(ctx->cimvideo.dev,"Warning: index %d out of range!\n", index);
		goto out;
	}

	if(pcdev->cim_priv->version_num == 2) {
		desc_v2_v = (struct ingenic_camera_desc_v2 *) pcdev->desc_vaddr;
		desc_v2_p = pcdev->desc_v2_paddr;

		if(is_cim_disabled(pcdev) && list_is_singular(&pcdev->video_buffer_list)) {
			desc_v2_v[index].DES_CFG_t.data.DES_END = 1;
			desc_v2_v[index].DES_INTC_t.data.EOF_MSK = 0;

			pcdev->desc_v2_head = &desc_v2_v[index];
			pcdev->desc_v2_tail = &desc_v2_v[index];

			regval = (dma_addr_t)(&desc_v2_p[index]);
			cim_writel(pcdev, regval, DES_ADDR);
			cim_writel(pcdev, CIM_CTRL_START, CIM_CTRL);
		} else {
			pcdev->desc_v2_tail->next = (dma_addr_t)(&desc_v2_p[index]);
			pcdev->desc_v2_tail->DES_INTC_t.data.EOF_MSK = 1;
			pcdev->desc_v2_tail->DES_CFG_t.data.DES_END = 0;
			pcdev->desc_v2_tail = &desc_v2_v[index];

			desc_v2_v[index].DES_CFG_t.data.DES_END = 1;
			desc_v2_v[index].DES_INTC_t.data.EOF_MSK = 0;
		}
	} else {
		desc_v1_v = (struct ingenic_camera_desc_v1 *) pcdev->desc_vaddr;
		desc_v1_p = pcdev->desc_v1_paddr;

		if(is_cim_disabled(pcdev) && list_is_singular(&pcdev->video_buffer_list)) {
			desc_v1_v[index].DES_CFG_t.data.DES_END = 1;
			desc_v1_v[index].DES_INTC_t.data.EOF_MSK = 0;

			pcdev->desc_v1_head = &desc_v1_v[index];
			pcdev->desc_v1_tail = &desc_v1_v[index];

			regval = (dma_addr_t)(&desc_v1_p[index]);
			cim_writel(pcdev, regval, DES_ADDR);
			cim_writel(pcdev, CIM_CTRL_START, CIM_CTRL);
		} else {
			pcdev->desc_v1_tail->next = (dma_addr_t)(&desc_v1_p[index]);
			pcdev->desc_v1_tail->DES_INTC_t.data.EOF_MSK = 1;
			pcdev->desc_v1_tail->DES_CFG_t.data.DES_END = 0;
			pcdev->desc_v1_tail = &desc_v1_v[index];

			desc_v1_v[index].DES_CFG_t.data.DES_END = 1;
		}
	}

out:
	spin_unlock_irqrestore(&pcdev->lock, flags);

	if(showFPS){
		calculate_frame_rate();
	}

	return;
}

static void ingenic_cim_start(struct ingenic_camera_dev *pcdev)
{
	struct vb2_buffer *vb2 = &pcdev->active->vb.vb2_buf;
	unsigned regval = 0;
	int index;

	BUG_ON(!pcdev->active);

	/* clear status register */
	cim_writel(pcdev, CIM_CLR_ALL, CIM_CLR_ST);

	/* configure dma desc addr*/
	index = vb2->index;
	if (pcdev->cim_priv->version_num == 2)
		regval = (dma_addr_t)(&(pcdev->desc_v2_paddr[index]));
	else
		regval = (dma_addr_t)(&(pcdev->desc_v1_paddr[index]));

	cim_writel(pcdev, regval, DES_ADDR);

	cim_writel(pcdev, CIM_CTRL_START, CIM_CTRL);
}

static void ingenic_cim_stop(struct ingenic_camera_dev *pcdev)
{
	cim_writel(pcdev, CIM_CTRL_QCK_STOP, CIM_CTRL);
	cim_writel(pcdev, CIM_CLR_ALL, CIM_CLR_ST);
}

static int ingenic_cim_soft_reset(struct cim_video_ctx *ctx)
{
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	struct v4l2_subdev *sd = cimvideo->subdev;
	int count = 4000;

	v4l2_subdev_call(sd, video, s_stream, 1);

	cim_writel(pcdev, CIM_CTRL_SOFT_RST, CIM_CTRL);
	while((!(cim_readl(pcdev,CIM_ST) & CIM_ST_SRST)) && count--) {
		udelay(1000);
	}

	if (count < 0) {
		v4l2_subdev_call(sd, video, s_stream, 0);
		dev_err(pcdev->dev, "cim soft reset failed!\n");
		return -ENODEV;
	}
	cim_writel(pcdev,CIM_CLR_SRST,CIM_CLR_ST);

	v4l2_subdev_call(sd, video, s_stream, 0);

	return 0;
}

static int ingenic_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct cim_video_ctx *ctx = vb2_get_drv_priv(q);
	struct ingenic_camera_dev *pcdev = ctx->cimvideo.pcdev;
	struct v4l2_subdev *sd = ctx->cimvideo.subdev;
	struct ingenic_buffer *buf, *node;
	int version = pcdev->cim_priv->version_num;
	int index = ctx->cimvideo.index;
	struct cim_async_subdev *casd = &pcdev->casd[index];
	unsigned long flags;
	int ret = 0;
	unsigned int tmp = 0;

	list_for_each_entry_safe(buf, node, &pcdev->video_buffer_list, list) {
		ret = ingenic_init_desc(&buf->vb.vb2_buf);
		if(ret) {
			dev_err(ctx->cimvideo.dev, "%s:Desc initialization failed\n",
					__func__);
			return ret;
		}
	}

	if(casd->bus_type == V4L2_MBUS_CSI2_DPHY) {
		ret = csi_phy_start(version, casd->mipi_csi2.num_data_lanes);
		if(ret < 0) {
			dev_err(ctx->cimvideo.dev, "csi starting failed!\n");
			return ret;
		}
	}

	ret = ingenic_cim_soft_reset(ctx);
	if(ret)
		goto err;

	v4l2_subdev_call(sd, video, s_stream, 1);

	if(ret)
		goto err;

	spin_lock_irqsave(&pcdev->lock, flags);
	ingenic_cim_start(pcdev);
	pcdev->start_streaming_called = 1;
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
err:
	if(casd->bus_type == V4L2_MBUS_CSI2_DPHY)
		csi_phy_stop(version);

	if(ctx->snapshot){
		tmp = cim_readl(pcdev, GLB_CFG);
		tmp &= ~GLB_CFG_DAT_MODE;
		tmp &= ~GLB_CFG_EXPO_WIDTH_MASK_V2;
		cim_writel(pcdev, tmp, GLB_CFG);
		cim_writel(pcdev, 0, DLY_CFG);
	}

	list_for_each_entry_safe(buf, node, &pcdev->video_buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}

	pcdev->desc_v1_head = NULL;
	pcdev->desc_v1_tail = NULL;
	pcdev->desc_v2_head = NULL;
	pcdev->desc_v2_tail = NULL;

	return ret;
}

static void ingenic_stop_streaming(struct vb2_queue *q)
{
	struct cim_video_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_subdev *sd = ctx->cimvideo.subdev;
	struct ingenic_camera_dev *pcdev = ctx->cimvideo.pcdev;
	struct ingenic_buffer *buf, *node;
	int version = pcdev->cim_priv->version_num;
	int index = ctx->cimvideo.index;
	struct cim_async_subdev *casd = &pcdev->casd[index];
	unsigned long flags;
	unsigned int tmp = 0;

	spin_lock_irqsave(&pcdev->lock, flags);
	ingenic_cim_stop(pcdev);
//	printk("[%d]~~~~~~~~%x\n", __LINE__, *(volatile unsigned int *)0xb3062004);

	/* Release all active buffers */
	list_for_each_entry_safe(buf, node, &pcdev->video_buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	pcdev->start_streaming_called = 0;
	pcdev->desc_v1_head = NULL;
	pcdev->desc_v1_tail = NULL;
	pcdev->desc_v2_head = NULL;
	pcdev->desc_v2_tail = NULL;
	pcdev->active = NULL;

	if(ctx->snapshot){
		tmp = cim_readl(pcdev, GLB_CFG);
		tmp &= ~GLB_CFG_DAT_MODE;
		tmp &= ~GLB_CFG_EXPO_WIDTH_MASK_V2;
		cim_writel(pcdev, tmp, GLB_CFG);
		cim_writel(pcdev, 0, DLY_CFG);
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);

	if(casd->bus_type == V4L2_MBUS_CSI2_DPHY)
		csi_phy_stop(version);
	v4l2_subdev_call(sd, video, s_stream, 0);
}


static struct vb2_ops cim_video_queue_ops = {
	.buf_init		= ingenic_videobuf_init,
	.queue_setup		= ingenic_videobuf_setup,
	.buf_prepare		= ingenic_buffer_prepare,
	.buf_queue		= ingenic_buffer_queue,
	.start_streaming	= ingenic_start_streaming,
	.stop_streaming		= ingenic_stop_streaming,
};

static void ingenic_camera_activate(struct ingenic_camera_dev *pcdev) {
	int ret = -1;

	if(pcdev->clk) {
		ret = clk_prepare_enable(pcdev->clk);
	}
	if(pcdev->mipi_clk) {
		ret = clk_prepare_enable(pcdev->mipi_clk);
	}
}

static void ingenic_camera_deactivate(struct ingenic_camera_dev *pcdev) {
	if( pcdev->mipi_clk) {
		clk_disable_unprepare(pcdev->mipi_clk);
	}
	if(pcdev->clk) {
		clk_disable_unprepare(pcdev->clk);
	}
}


static int cim_video_open(struct file *file)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	struct video_device *video = &cimvideo->video;
	struct vb2_queue *queue = NULL;
	int ret = 0;

	mutex_lock(&pcdev->mutex);

	if(pcdev->active_ctx){
		if(cimvideo->index != pcdev->active_ctx->cimvideo.index)
			dev_info(cimvideo->dev, "Only one cim video device node can be operated at the same time");
		ret = -EBUSY;
		goto err_busy;
	}

	v4l2_fh_init(&ctx->fh, &cimvideo->video);
	v4l2_fh_add(&ctx->fh);

	queue = &ctx->queue;
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP |VB2_USERPTR;
	queue->drv_priv = ctx;
	queue->ops = &cim_video_queue_ops;
	queue->mem_ops = &ingenic_vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct ingenic_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &cimvideo->queue_lock;

	ret = vb2_queue_init(queue);
	if(ret < 0) {
		goto err_vb2_queue_init;
	}

	video->queue = queue;

	file->private_data = &ctx->fh;
	pcdev->active_ctx = ctx;
	ingenic_camera_activate(pcdev);

	mutex_unlock(&pcdev->mutex);

	return 0;

err_vb2_queue_init:
	v4l2_fh_del(&ctx->fh);
err_busy:
	mutex_unlock(&pcdev->mutex);
	return ret;
}

static int cim_video_release(struct file *file)
{
	struct cim_video_ctx *ctx = video_drvdata(file);
	struct cim_video_device *cimvideo = &ctx->cimvideo;
	struct ingenic_camera_dev *pcdev = cimvideo->pcdev;
	struct video_device *video = &cimvideo->video;

	/*lock*/
	mutex_lock(&pcdev->mutex);

	vb2_queue_release(video->queue);

	ingenic_camera_deactivate(pcdev);
	ingenic_camera_free_desc(pcdev);

	pcdev->active_ctx = NULL;
	file->private_data = NULL;
	video->queue = NULL;

	memset(&ctx->img_sz, 0, sizeof(struct ingenic_image_sz));
	memset(&ctx->format, 0, sizeof(struct v4l2_format));
	ctx->interlace = 0;
	ctx->uv_offset = 0;
	ctx->payload_size = 0;

	ctx->hist_en = 0;
	ctx->hist_gain_add = 0;
	ctx->hist_gain_mul = 0;
	ctx->lumi_en = 0;
	ctx->snapshot = 0;
	ctx->output_y = 0;
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	/*unlock*/
	mutex_unlock(&pcdev->mutex);

	return 0;

}


static struct v4l2_file_operations cim_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = cim_video_open,
	.release = cim_video_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

int ingenic_cim_asd_notifier_bound(struct v4l2_async_notifier *notifier,
		struct v4l2_subdev *subdev,
		struct v4l2_async_subdev *asd)
{
	struct v4l2_device *v4l2_dev = notifier->v4l2_dev;
	struct ingenic_camera_dev *pcdev = v4l2_dev_to_pcdev(v4l2_dev);
	struct cim_video_device *cimvideo = NULL;
	struct video_device *video = NULL;
	int index = get_asd_index(asd);
	struct cim_video_ctx *ctx = pcdev->ctx[index];
	int ret = 0;

	ctx = kzalloc(sizeof(struct cim_video_ctx), GFP_KERNEL);
	if(IS_ERR_OR_NULL(ctx)) {
		dev_err(cimvideo->dev, "Failed to alloc ctx for cim_video_ctx\n");
		ret = -ENOMEM;
		goto err_ctx_alloc;
	}

	cimvideo = &ctx->cimvideo;
	video = &cimvideo->video;

	cimvideo->pcdev = pcdev;
	cimvideo->index = index;
	cimvideo->alloc_ctx = ingenic_vb2_dma_contig_init_ctx(cimvideo->dev);
	if(IS_ERR_OR_NULL(cimvideo->alloc_ctx)) {
		ret = -ENOMEM;
		goto err_dma_alloc_ctx;
	}

	mutex_init(&cimvideo->queue_lock);
	mutex_init(&cimvideo->stream_lock);

	/*ctrl_handler*/
	v4l2_ctrl_handler_init(&ctx->hdl, 10);
	v4l2_ctrl_new_custom(&ctx->hdl, &img_crop_way, NULL);
	if (pcdev->cim_priv->version_num == 2) {
		v4l2_ctrl_new_custom(&ctx->hdl, &lumi_en, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &output_y, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &point1_x, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &point1_y, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &point2_x, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &point2_y, NULL);
	} else {
		v4l2_ctrl_new_custom(&ctx->hdl, &img_scale_sharp, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &hist_en, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &hist_gain_add, NULL);
		v4l2_ctrl_new_custom(&ctx->hdl, &hist_gain_mul, NULL);
	}
	v4l2_ctrl_new_custom(&ctx->hdl, &snapshot_en, NULL);
	v4l2_ctrl_new_custom(&ctx->hdl, &exp_pulse_w, NULL);
	v4l2_ctrl_new_custom(&ctx->hdl, &delay_t, NULL);

	v4l2_ctrl_handler_setup(&ctx->hdl);

	/*video register device*/
	snprintf(video->name, sizeof(video->name), "%s-%d", v4l2_dev->name, index);
	video->fops = &cim_video_fops;
	video->ioctl_ops = &cim_video_ioctl_ops;
	video->release = video_device_release_empty;
	video->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video->v4l2_dev = v4l2_dev;
	video->ctrl_handler = &ctx->hdl;
	video_set_drvdata(video, ctx);

	cimvideo->dev = pcdev->dev;
	cimvideo->subdev = subdev;

	ret = video_register_device(video, VFL_TYPE_VIDEO, ingenic_cim_get_video_nr(index));
	if(ret < 0) {
		goto err_video_register_device;
	}

	dev_info(pcdev->dev, "register video device %s @ /dev/video%d ok\n", video->name, video->num);

	return 0;

err_video_register_device:
	ingenic_vb2_dma_contig_cleanup_ctx(cimvideo->alloc_ctx);
err_dma_alloc_ctx:
	kfree(ctx);
err_ctx_alloc:
	return ret;
}

int ingenic_cim_asd_notifier_complete(struct v4l2_async_notifier *notifier){

	return 0;
}

void ingenic_cim_asd_notifier_unbind(struct v4l2_async_notifier *notifier,
		struct v4l2_subdev *subdev,
		struct v4l2_async_subdev *asd)
{
	struct v4l2_device *v4l2_dev = notifier->v4l2_dev;
	struct ingenic_camera_dev *pcdev = v4l2_dev_to_pcdev(v4l2_dev);
	int index = get_asd_index(asd);
	struct cim_video_ctx *ctx = pcdev->ctx[index];
	struct cim_video_device *cimvideo = &ctx->cimvideo;

	v4l2_ctrl_handler_free(&ctx->hdl);
	video_unregister_device(&cimvideo->video);
	ingenic_vb2_dma_contig_cleanup_ctx(cimvideo->alloc_ctx);
	kfree(ctx);
}

struct v4l2_async_notifier_operations ingenic_cim_async_ops = {
	.bound = ingenic_cim_asd_notifier_bound,
	.complete = ingenic_cim_asd_notifier_complete,
	.unbind = ingenic_cim_asd_notifier_unbind,
};


static int ingenic_cim_phrase_remote_endpoint(struct ingenic_camera_dev *pcdev)
{
	struct device_node *node = NULL;
	struct v4l2_async_subdev *asd = NULL;
	int ret = 0;

	v4l2_async_notifier_init(&pcdev->notifier);
	while((node = of_graph_get_next_endpoint(pcdev->dev->of_node, node)) && pcdev->asd_num < MAX_ASYNC_SUBDEVS)
	{
		pcdev->casd[pcdev->asd_num].index = pcdev->asd_num;
		ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &pcdev->vep[pcdev->asd_num]);
		if(ret) {
			of_node_put(node);
			return ret;
		}
		pcdev->casd[pcdev->asd_num].bus_type = pcdev->vep[pcdev->asd_num].bus_type;
		pcdev->casd[pcdev->asd_num].parallel = pcdev->vep[pcdev->asd_num].bus.parallel;
		pcdev->casd[pcdev->asd_num].mipi_csi2 = pcdev->vep[pcdev->asd_num].bus.mipi_csi2;
		pcdev->casd[pcdev->asd_num].index = pcdev->asd_num;

		asd = &pcdev->casd[pcdev->asd_num].asd;
		asd->match.fwnode = fwnode_graph_get_remote_port_parent(of_fwnode_handle(node));
		asd->match_type = V4L2_ASYNC_MATCH_FWNODE;

		of_node_put(node);

		ret = v4l2_async_notifier_add_subdev(&pcdev->notifier, asd);
		if (ret) {
			fwnode_handle_put(asd->match.fwnode);
			return ret;
		}

		fwnode_handle_put(asd->match.fwnode);

		pcdev->asd_num++;
	}
	return ret;
}

/**********************cim_debug***************************/
static ssize_t frame_size_check_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	snprintf(buf, 100, " 0: disable frame size check.\n 1: enable frame size check.\n");
	return 100;
}

static ssize_t frame_size_check_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        if (count != 2)
                return -EINVAL;

        if (*buf == '0') {
                frame_size_check_flag = 0;
        } else if (*buf == '1') {
                frame_size_check_flag = 1;
        } else {
                return -EINVAL;
        }

        return count;
}

static ssize_t fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("\n-----you can choice print way:\n");
	printk("Example: echo NUM > show_fps\n");
	printk("NUM = 0: close fps statistics\n");
	printk("NUM = 1: print recently fps\n");
	printk("NUM = 2: print interval between last and this queue buffers\n");
	printk("NUM > 2: print fps after NUM second\n");
	return 0;
}

static ssize_t fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int num = 0;
	num = simple_strtoul(buf, NULL, 0);
	if(num < 0){
		printk("\n--please 'cat show_fps' to view using the method\n\n");
		return n;
	}
	showFPS = num;
	return n;
}

static DEVICE_ATTR(frame_size_check, S_IRUGO|S_IWUSR, frame_size_check_r, frame_size_check_w);
static DEVICE_ATTR(show_fps, S_IRUGO|S_IWUSR, fps_show, fps_store);

static struct attribute *cim_debug_attrs[] = {
	&dev_attr_frame_size_check.attr,
	&dev_attr_show_fps.attr,
	NULL,
};
const char cim_group_name[] = "debug";
static struct attribute_group cim_debug_attr_group = {
	.name	= cim_group_name,
	.attrs	= cim_debug_attrs,
};

/***************************prob*******************************/
static struct cim_private version1_pri = {
	.max_video_mem = VIDEO_MEM + CIM_HIST_SIZE,
	.version_num = 1,

};

static struct cim_private version2_pri = {
	.max_video_mem = VIDEO_MEM + CIM_LUMI_SIZE,
	.version_num = 2,
};

static const struct of_device_id ingenic_camera_of_match[] = {
	{ .compatible = "ingenic,m300-cim", .data = (struct cim_private *)&version1_pri },
	{ .compatible = "ingenic,x2000-cim", .data = (struct cim_private *)&version1_pri },
	{ .compatible = "ingenic,x1600-cim", .data = (struct cim_private *)&version2_pri },
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_camera_of_match);

static int ingenic_camera_probe(struct platform_device *pdev) {
	int ret = 0;
	unsigned int irq;
	struct resource *res;
	void __iomem *base;
	struct ingenic_camera_dev *pcdev;

	/* malloc */
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	/* resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(&pdev->dev, "Could not get resource!\n");
		ret = -ENODEV;
		goto err_get_resource;
	}

	/* irq */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "Could not get irq!\n");
		ret = -ENODEV;
		goto err_get_irq;
	}

	pcdev->priv = of_match_node(ingenic_camera_of_match, pdev->dev.of_node);
	pcdev->cim_priv = (struct cim_private *)pcdev->priv->data;

	printk("cim current version: %d\n", pcdev->cim_priv->version_num);

	/*get cim clk*/
	pcdev->clk = of_clk_get(pdev->dev.of_node, 1);
	if (pcdev->clk == NULL) {
		ret = PTR_ERR(pcdev->clk);
		dev_err(&pdev->dev, "%s:can't get clk %s\n", __func__, "cim");
		goto err_clk_get_cim;
	}

	/*get mipi clk*/
	pcdev->mipi_clk = of_clk_get(pdev->dev.of_node, 2);
	if (pcdev->mipi_clk == NULL) {
		ret = PTR_ERR(pcdev->clk);
		dev_err(&pdev->dev, "%s:can't get clk %s\n", __func__, "cim");
		goto err_clk_get_mipi;
	}

	/* Request the regions. */
	if (!request_mem_region(res->start, resource_size(res), DRIVER_NAME)) {
		ret = -EBUSY;
		goto err_request_mem_region;
	}
	base = ioremap(res->start, resource_size(res));
	if (!base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	spin_lock_init(&pcdev->lock);
	INIT_LIST_HEAD(&pcdev->video_buffer_list);
	mutex_init(&pcdev->mutex);

	pcdev->dev = &pdev->dev;
	pcdev->res = res;
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->active = NULL;
	pcdev->desc_v1_head = NULL;
	pcdev->desc_v1_tail = NULL;
	pcdev->desc_v2_head = NULL;
	pcdev->desc_v2_tail = NULL;

	/* request irq */
	ret = devm_request_irq(&pdev->dev, pcdev->irq, ingenic_camera_irq_handler, 0,
			dev_name(&pdev->dev), pcdev);
	if(ret) {
		dev_err(&pdev->dev, "request irq failed!\n");
		goto err_request_irq;
	}

	ret = v4l2_device_register(&pdev->dev, &pcdev->v4l2_dev);
	if (ret)
		goto err_v4l2_dev_register;

	ret = sysfs_create_group(&pcdev->dev->kobj, &cim_debug_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_free_file;
	}

	/*asd*/
	ret = ingenic_cim_phrase_remote_endpoint(pcdev);
	if(ret)
		goto err_phrase_remote_endpoint;

	pcdev->notifier.ops = &ingenic_cim_async_ops;
	ret = v4l2_async_notifier_register(&pcdev->v4l2_dev, &pcdev->notifier);
	if(ret){
		printk( "v4l2_async_notifier_register failed %d\n", ret);
		goto err_regist_notifier;
	}

	ret = of_reserved_mem_device_init(pcdev->dev);
	if(ret)
		dev_warn(pcdev->dev, "failed to init reserved mem\n");

	dev_dbg(&pdev->dev, "ingenic Camera driver loaded!\n");

	platform_set_drvdata(pdev, pcdev);
	return 0;

err_regist_notifier:
err_phrase_remote_endpoint:
	sysfs_remove_group(&pcdev->dev->kobj, &cim_debug_attr_group);
err_free_file:
	v4l2_device_unregister(&pcdev->v4l2_dev);
err_v4l2_dev_register:
	free_irq(pcdev->irq, pcdev);
err_request_irq:
	iounmap(pcdev->base);
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_request_mem_region:
	devm_clk_put(&pdev->dev, pcdev->mipi_clk);
err_clk_get_mipi:
	devm_clk_put(&pdev->dev, pcdev->clk);
err_clk_get_cim:
err_get_irq:
err_get_resource:
	kfree(pcdev);
//	cimvideo->ops = video_ops;
err_kzalloc:
	return ret;

}

static int ingenic_camera_remove(struct platform_device *pdev)
{
	struct ingenic_camera_dev *pcdev = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&pcdev->notifier);
	sysfs_remove_group(&pcdev->dev->kobj, &cim_debug_attr_group);
	v4l2_device_unregister(&pcdev->v4l2_dev);
	free_irq(pcdev->irq, pcdev);
	iounmap(pcdev->base);
	release_mem_region(pcdev->res->start, resource_size(pcdev->res));
	devm_clk_put(&pdev->dev, pcdev->mipi_clk);
	devm_clk_put(&pdev->dev, pcdev->clk);
	kfree(pcdev);
	dev_dbg(&pdev->dev, "ingenic Camera driver unloaded\n");
	return 0;
}

static struct platform_driver ingenic_camera_driver = {
	.probe		= ingenic_camera_probe,
	.remove		= ingenic_camera_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ingenic_camera_of_match,
	},
};

module_platform_driver(ingenic_camera_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("anan <an.an@ingenic.cn>");
MODULE_DESCRIPTION("ingenic Soc Camera Host Driver");
MODULE_ALIAS("a ingenic-cim platform");
