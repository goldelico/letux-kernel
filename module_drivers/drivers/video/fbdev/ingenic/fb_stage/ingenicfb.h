/* drivers/video/fbdev/ingenic/x2000_v12/ingenicfb.h
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * Author:clwang<chunlei.wang@ingenic.com>
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */
#ifndef __JZ_FB_H__
#define	__JZ_FB_H__
#include <linux/fb.h>
#include <libdmmu.h>

#include "jz_dsim.h"

#include "uapi_ingenicfb.h"
#include "hw_composer.h"
#include "dpu_ctrl.h"
#include "sysfs.h"

#define FRAME_CTRL_DEFAULT_SET  (0x04)

#define FRAME_CFG_ALL_UPDATE (0xFF)

enum {
	RDMA_CHAIN_CFG_COLOR_RGB = 0,
	RDMA_CHAIN_CFG_COLOR_RBG = 1,
	RDMA_CHAIN_CFG_COLOR_GRB = 2,
	RDMA_CHAIN_CFG_COLOR_GBR = 3,
	RDMA_CHAIN_CFG_COLOR_BRG = 4,
	RDMA_CHAIN_CFG_COLOR_BGR = 5,
};
enum {
	RDMA_CHAIN_CFG_FORMAT_555 = 0,
	RDMA_CHAIN_CFG_FORMAT_ARGB1555 = 1,
	RDMA_CHAIN_CFG_FORMAT_565 = 2,
	RDMA_CHAIN_CFG_FORMAT_888 = 4,
};

struct ingenicfb_timestamp {
	#define TIMESTAMP_CAP	16
	volatile int wp; /* write position */
	int rp;	/* read position */
	u64 value[TIMESTAMP_CAP];
};


struct ingenicfb_device {
	int open_cnt;
	int is_lcd_en;		/* 0, disable  1, enable */

	struct fb_videomode *active_video_mode;
	struct dpu_ctrl dctrl;

	struct hw_composer_master *comp_master;
	struct hw_composer_ctx *comp_ctx;

	struct hw_compfb_device *compfb;
	void *comp_v4l2;

	struct ingenicfb_colormode *color_mode; //当前rdma所用的color_mode;
	struct rdma_setup_info rdma_info;

	struct fb_info *fb;
	struct device *dev;
	struct lcd_panel *panel;
	int disable_rdma_fb;				//是否将rdma导出为fb节点.dts指定.默认为1, dts可以设置为0.

	struct dpu_sysfs sysfs;

	size_t vidmem_size;

	void *vidmem[CONFIG_FB_INGENIC_NR_FRAMES];
	dma_addr_t vidmem_phys[CONFIG_FB_INGENIC_NR_FRAMES];

	size_t frm_size;

	wait_queue_head_t vsync_wq;
	unsigned int vsync_skip_map;	/* 10 bits width */
	int vsync_skip_ratio;

	struct ingenicfb_timestamp timestamp;

	struct mutex lock;
	struct mutex suspend_lock;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int is_suspend;
	unsigned int pan_display_count;
	int blank;
	unsigned int pseudo_palette[16];
	int slcd_continua;

	unsigned int user_addr;
	unsigned int tlba;
	unsigned int tlb_err_cnt;
	struct dmmu_mm_notifier dmn;

};

static inline struct ingenicfb_device *sysfs_get_fbdev(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static inline struct dpu_ctrl *sysfs_get_dctrl(struct device *dev)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	return &fbdev->dctrl;
}

static inline struct hw_compfb_device *sysfs_get_compfb(struct device *dev)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	return fbdev->compfb;
}


struct dpu_dmmu_map_info {
	unsigned int addr;
	unsigned int len;
};

struct smash_mode {
    unsigned long vaddr;
    int mode;
};

void ingenicfb_clk_enable(struct ingenicfb_device *ingenicfb);
void ingenicfb_clk_disable(struct ingenicfb_device *fbdev);
int ingenicfb_release_vidmem(struct ingenicfb_device *fbdev);
int pan_init_logo(struct fb_info *fb);


extern int ingenicfb_register_panel(struct lcd_panel *panel);
#endif
