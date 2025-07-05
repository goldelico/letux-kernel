/*
 * drivers/video/fbdev/ingenic/x2000_v12/ingenicfb.c
 *
 * Copyright (c) 2020 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>
#include "dpu_reg.h"
#include "ingenicfb.h"
#include "hw_composer_fb.h"
#include "hw_composer_v4l2.h"
#include <libdmmu.h>

#define dpu_debug 1
#define print_dbg(f, arg...) if(dpu_debug) printk(KERN_INFO "dpu: %s, %d: " f "\n", __func__, __LINE__, ## arg)

#define COMPOSER_DIRECT_OUT_EN

#include "./jz_mipi_dsi/jz_mipi_dsih_hal.h"
#include "./jz_mipi_dsi/jz_mipi_dsi_regs.h"
#include "./jz_mipi_dsi/jz_mipi_dsi_lowlevel.h"
extern struct dsi_device * jzdsi_init(struct jzdsi_data *pdata);
extern void jzdsi_remove(struct dsi_device *dsi);
extern void dump_dsi_reg(struct dsi_device *dsi);
int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power);
static const struct of_device_id ingenicfb_of_match[];
#ifdef CONFIG_TRUE_COLOR_LOGO
#include <video/ingenic_logo.h>
extern unsigned char logo_buf_initdata[] __initdata; /* obj/drivers/video/logo-ingenic/logo.c */
extern void show_logo(struct fb_info *info);
static unsigned char * copyed_logo_buf = NULL;
#endif

struct lcd_panel *fbdev_panel = NULL;
struct platform_device *fbdev_pdev = NULL;
static unsigned int cmp_gen_sop = 0;
static int uboot_inited;
static int showFPS = 0;
static int over_cnt = 0;
static bool panel_mipi_init_ok = false;
static struct ingenicfb_device *fbdev;

#ifdef CONFIG_FB_VSYNC_SKIP_DISABLE
static unsigned int old_desc_addr = 0;
#endif

/* #define TEST_IRQ */

const struct fb_fix_screeninfo ingenicfb_fix  = {
	.id = "ingenicfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 1,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

struct ingenicfb_colormode {
	uint32_t mode;
	const char *name;
	uint32_t color;
	uint32_t bits_per_pixel;
	uint32_t nonstd;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

struct ingenic_priv {
	unsigned int dsi_iobase;
	unsigned int dsi_phy_iobase;
	bool support_comp;
};

/* 这个实际是comp支持的, 应该放在compfb中。这里应该只记录rdma支持的格式.*/
static struct ingenicfb_colormode ingenicfb_colormodes[] = {
	{
		.mode = RDMA_CHAIN_CFG_FORMAT_888,
		.name = "rgb888",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 32,
		.nonstd = 0,
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.red	= { .length = 8, .offset = 16, .msb_right = 0 },
		.green	= { .length = 8, .offset = 8, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_555,
		.name = "rgb555",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red	= { .length = 5, .offset = 10, .msb_right = 0 },
		.green	= { .length = 5, .offset = 5, .msb_right = 0 },
		.blue	= { .length = 5, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_ARGB1555,
		.name = "argb1555",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red	= { .length = 5, .offset = 10, .msb_right = 0 },
		.green	= { .length = 5, .offset = 5, .msb_right = 0 },
		.blue	= { .length = 5, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 1, .offset = 15, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_565,
		.name = "rgb565",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red	= { .length = 5, .offset = 11, .msb_right = 0 },
		.green	= { .length = 6, .offset = 5, .msb_right = 0 },
		.blue	= { .length = 5, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	},
};

static int ingenicfb_dmmu_mm_release(void *data)
{
	struct ingenicfb_device *fbdev = (struct ingenicfb_device *)data;

	hw_composer_lock(fbdev->comp_ctx);
	hw_composer_stop(fbdev->comp_ctx);
	hw_composer_unlock(fbdev->comp_ctx);

	return 0;
}


static struct dmmu_mm_ops ingenicfb_dmmu_mm_ops = {
	.mm_release = ingenicfb_dmmu_mm_release,
};

	static void
ingenicfb_videomode_to_var(struct fb_var_screeninfo *var,
		const struct fb_videomode *mode)
{
	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = mode->xres;
	var->yres_virtual = mode->yres * CONFIG_FB_INGENIC_NR_FRAMES;
	var->xoffset = 0;
	var->yoffset = 0;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = mode->vmode & FB_VMODE_MASK;
	var->pixclock = mode->pixclock;
}

static struct fb_videomode *ingenicfb_get_mode(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	size_t i;
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode = fbdev->panel->modes;

	for (i = 0; i < fbdev->panel->num_modes; ++i, ++mode) {
		if (mode->xres == var->xres && mode->yres == var->yres
				&& mode->vmode == var->vmode
				&& mode->right_margin == var->right_margin) {
			if (fbdev->panel->lcd_type != LCD_TYPE_SLCD) {
				if (mode->pixclock == var->pixclock)
					return mode;
			} else {
				return mode;
			}
		}
	}

	return NULL;
}

static void ingenicfb_colormode_to_var(struct fb_var_screeninfo *var,
		struct ingenicfb_colormode *color)
{
	var->bits_per_pixel = color->bits_per_pixel;
	var->nonstd = color->nonstd;
	var->red = color->red;
	var->green = color->green;
	var->blue = color->blue;
	var->transp = color->transp;
}

static bool cmp_var_to_colormode(struct fb_var_screeninfo *var,
		struct ingenicfb_colormode *color)
{
	bool cmp_component(struct fb_bitfield *f1, struct fb_bitfield *f2)
	{
		return f1->length == f2->length &&
			f1->offset == f2->offset &&
			f1->msb_right == f2->msb_right;
	}

	if (var->bits_per_pixel == 0 ||
			var->red.length == 0 ||
			var->blue.length == 0 ||
			var->green.length == 0)
		return 0;

	return var->bits_per_pixel == color->bits_per_pixel &&
		cmp_component(&var->red, &color->red) &&
		cmp_component(&var->green, &color->green) &&
		cmp_component(&var->blue, &color->blue) &&
		cmp_component(&var->transp, &color->transp);
}

static struct ingenicfb_colormode *ingenicfb_check_colormode(struct fb_var_screeninfo *var)
{
	int i;

	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
			struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
			if (var->nonstd == m->nonstd) {
				ingenicfb_colormode_to_var(var, m);
				return m;
			}
		}

		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
		struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
		if (cmp_var_to_colormode(var, m)) {
			ingenicfb_colormode_to_var(var, m);
			return m;
		}
	}
	/* To support user libraries that only support RGB format */
	for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
		struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
		if (var->bits_per_pixel == m->bits_per_pixel) {
			ingenicfb_colormode_to_var(var, m);
			return m;
		}
	}

	return NULL;
}

static int ingenicfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode;
	struct ingenicfb_colormode *color_mode;
	int ret;

	mode = ingenicfb_get_mode(var, info);
	if (mode == NULL) {
		dev_err(info->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}

	ingenicfb_videomode_to_var(var, mode);

	/*TODO:不应该修改var.*/
	color_mode = ingenicfb_check_colormode(var);
	if(color_mode == NULL) {
		dev_err(info->dev,"Check colormode failed!\n");
		return  -EINVAL;
	}

	fbdev->color_mode = color_mode;

	return 0;
}

#define DPU_WAIT_IRQ_TIME 8000
static int ingenicfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct comp_setup_info comp_info;
	struct ingenicfb_device *fbdev = info->par;
	int ret;
	csc_mode_t csc_mode;
	int value;
	int tmp;
	int i;

	switch (cmd) {
		case JZFB_SET_VSYNCINT:
			if (unlikely(copy_from_user(&value, argp, sizeof(int))))
				return -EFAULT;
			break;
		case FBIO_WAITFORVSYNC:
			if(fbdev->dctrl.chan == DATA_CH_COMP)
				break;
#ifndef CONFIG_FB_VSYNC_SKIP_DISABLE
			unlock_fb_info(info);
			ret = wait_event_interruptible_timeout(fbdev->vsync_wq,
					fbdev->timestamp.wp != fbdev->timestamp.rp,
					msecs_to_jiffies(DPU_WAIT_IRQ_TIME));
			lock_fb_info(info);
			if(ret == 0) {
				dev_err(info->dev, "DPU wait vsync timeout!\n");
				return -EFAULT;
			}

			ret = copy_to_user(argp, fbdev->timestamp.value + fbdev->timestamp.rp,
					sizeof(u64));
			fbdev->timestamp.rp = (fbdev->timestamp.rp + 1) % TIMESTAMP_CAP;

			if (unlikely(ret))
				return -EFAULT;
#else
			{
				unlock_fb_info(info);
				fbdev->timestamp.rp = fbdev->timestamp.wp;
				ret = wait_event_interruptible_timeout(fbdev->vsync_wq,
						fbdev->timestamp.wp != fbdev->timestamp.rp,
						msecs_to_jiffies(DPU_WAIT_IRQ_TIME));
				lock_fb_info(info);
				if(ret == 0) {
					dev_err(info->dev, "DPU wait vsync timeout!\n");
					return -EFAULT;
				}
			}
#endif
			break;
		case JZFB_PUT_FRM_CFG:
			copy_from_user(&comp_info.frm_cfg, (void *)argp, sizeof(struct ingenicfb_frm_cfg));


			comp_info.nframes = 1;
			//comp_info.out_mode = COMP_WRITE_BACK;

			/*设置composer帧信息，并且启动一次comp设备进行合成.*/
			hw_composer_lock(fbdev->comp_ctx);

			hw_composer_setup(fbdev->comp_ctx, &comp_info);

			hw_composer_start(fbdev->comp_ctx);
			hw_composer_unlock(fbdev->comp_ctx);

			break;
		case JZFB_GET_FRM_CFG:
			copy_to_user((void *)argp,
				      &fbdev->comp_ctx->comp_info.frm_cfg,
				      sizeof(struct ingenicfb_frm_cfg));
			break;
		case JZFB_GET_LAYERS_NUM:
			{
				unsigned int layers_num = CONFIG_FB_INGENIC_NR_LAYERS;
				copy_to_user((void *)argp,
					      &layers_num,
					      sizeof(unsigned int));
			}
			break;
		case JZFB_SET_CSC_MODE:
			if (unlikely(copy_from_user(&csc_mode, argp, sizeof(csc_mode_t))))
				return -EFAULT;

			//TODO:
			//csc_mode_set(fbdev, csc_mode);
			break;
		case JZFB_USE_TLB:
			if((unsigned int)arg != 0){
				fbdev->dctrl.tlba = (unsigned int)arg;

				dmmu_register_mm_notifier(&fbdev->dmn);
			} else
				printk("tlb err!!!\n");
			break;
		case JZFB_DMMU_MEM_SMASH:
			{
				struct smash_mode sm;
				if (copy_from_user(&sm, (void *)arg, sizeof(sm))) {
					ret = -EFAULT;
					break;
				}
				return dmmu_memory_smash(sm.vaddr, sm.mode);
				break;
			}
		case JZFB_DMMU_DUMP_MAP:
			{
				unsigned long vaddr;
				if (copy_from_user(&vaddr, (void *)arg, sizeof(unsigned long))) {
					ret = -EFAULT;
					break;
				}
				return dmmu_dump_map(vaddr);
				break;
			}
		case JZFB_DMMU_MAP:
			{
				struct dpu_dmmu_map_info di;
				if (copy_from_user(&di, (void *)arg, sizeof(di))) {
					ret = -EFAULT;
					break;
				}
				fbdev->user_addr = di.addr;
				return dmmu_map(info->dev,di.addr,di.len);
				break;
			}
		case JZFB_DMMU_UNMAP:
			{
				struct dpu_dmmu_map_info di;
				if (copy_from_user(&di, (void *)arg, sizeof(di))) {
					ret = -EFAULT;
					break;
				}
				return dmmu_unmap(info->dev,di.addr,di.len);
				break;
			}
		case JZFB_DMMU_UNMAP_ALL:
			dmmu_unmap_all(info->dev);
			break;
		case JZFB_DMMU_FLUSH_CACHE:
			{
				struct dpu_dmmu_map_info di;
				if (copy_from_user(&di, (void *)arg, sizeof(di))) {
					ret = -EFAULT;
					break;
				}
				return dmmu_flush_cache(di.addr,di.len);
				break;
			}
		default:
			printk("Command:%x Error!\n",cmd);
			break;
	}
	return 0;
}

int ingenicfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;

	start = info->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	/* Write-Acceleration */
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_WA;

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		return -EAGAIN;
	}

	return 0;
}

static void ingenicfb_set_vsync_value(void *data)
{
	struct ingenicfb_device *fbdev = (struct ingenicfb_device *)data;

	fbdev->vsync_skip_map = (fbdev->vsync_skip_map >> 1 |
				fbdev->vsync_skip_map << 9) & 0x3ff;
	if(likely(fbdev->vsync_skip_map & 0x1)) {
		fbdev->timestamp.value[fbdev->timestamp.wp] =
			ktime_to_ns(ktime_get());
		fbdev->timestamp.wp = (fbdev->timestamp.wp + 1) % TIMESTAMP_CAP;
		wake_up_interruptible(&fbdev->vsync_wq);
#ifdef CONFIG_FB_VSYNC_SKIP_DISABLE
	} else {
		fbdev->timestamp.wp = fbdev->timestamp.rp + 1;
		wake_up_interruptible(&fbdev->vsync_wq);
#endif
	}
}

static inline uint32_t convert_color_to_hw(unsigned val, struct fb_bitfield *bf)
{
	return (((val << bf->length) + 0x7FFF - val) >> 16) << bf->offset;
}

static int ingenicfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *fb)
{
	if (regno >= 16)
		return -EINVAL;

	((uint32_t *)(fb->pseudo_palette))[regno] =
		convert_color_to_hw(red, &fb->var.red) |
		convert_color_to_hw(green, &fb->var.green) |
		convert_color_to_hw(blue, &fb->var.blue) |
		convert_color_to_hw(transp, &fb->var.transp);

	return 0;
}

static int calc_refresh_ratio(struct lcd_panel *panel)
{
	struct smart_config *smart_config = panel->smart_config;

	switch(smart_config->smart_type){
	case SMART_LCD_TYPE_8080:
	case SMART_LCD_TYPE_6800:
		break;
	case SMART_LCD_TYPE_SPI_3:
		return 9;
	case SMART_LCD_TYPE_SPI_4:
		return 8;
	default:
		printk("%s %d err!\n",__func__,__LINE__);
		break;
	}

	switch(smart_config->pix_fmt) {
	case SMART_LCD_FORMAT_888:
		if(smart_config->dwidth == SMART_LCD_DWIDTH_8_BIT)
			return 3;
		if(smart_config->dwidth == SMART_LCD_DWIDTH_24_BIT)
			return 1;
	case SMART_LCD_FORMAT_565:
		if(smart_config->dwidth == SMART_LCD_DWIDTH_8_BIT)
			return 2;
		if(smart_config->dwidth == SMART_LCD_DWIDTH_16_BIT)
			return 1;
	default:
		printk("%s %d err!\n",__func__,__LINE__);
		break;
	}

	return 1;

}

static int refresh_pixclock_auto_adapt(struct fb_info *info, struct fb_videomode *mode)
{
	struct ingenicfb_device *fbdev = info->par;
	struct lcd_panel *panel = fbdev->panel;
	struct fb_var_screeninfo *var = &info->var;
	uint16_t hds, vds;
	uint16_t hde, vde;
	uint16_t ht, vt;
	unsigned long rate;
	unsigned int refresh_ratio = 1;

	if (mode == NULL) {
		dev_err(fbdev->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}

	if (fbdev->panel->lcd_type == LCD_TYPE_SLCD) {
		refresh_ratio = calc_refresh_ratio(panel);
	}

	hds = mode->hsync_len + mode->left_margin;
	hde = hds + mode->xres;
	ht = hde + mode->right_margin;

	vds = mode->vsync_len + mode->upper_margin;
	vde = vds + mode->yres;
	vt = vde + mode->lower_margin;

	if(mode->refresh){
		rate = mode->refresh * vt * ht * refresh_ratio;

		mode->pixclock = KHZ2PICOS(round_up(rate, 1000)/1000);
		var->pixclock = mode->pixclock;
	}else if(mode->pixclock){
		rate = PICOS2KHZ(mode->pixclock) * 1000;
		mode->refresh = rate / vt / ht / refresh_ratio;
	}else{
		dev_err(fbdev->dev,"%s error:lcd important config info is absenced\n",__func__);
		return -EINVAL;
	}

	dev_info(fbdev->dev, "mode->refresh: %d, mode->pixclock: %d, rate: %ld, modex->pixclock: %d\n", mode->refresh, mode->pixclock, rate, mode->pixclock);

	return 0;
}


static int ingenicfb_set_par(struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	struct fb_videomode *mode;
	struct ingenicfb_colormode *color_mode;
	unsigned long flags;
	int ret;

	/*根据var xres,yres信息，查找panel支持的mode*/
	mode = ingenicfb_get_mode(var, info);
	if (mode == NULL) {
		dev_err(info->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}
	info->mode = mode;

	/*根据var信息, 查找到colormode.*/
	color_mode = ingenicfb_check_colormode(var);
	if(color_mode == NULL) {
		dev_err(info->dev,"Check colormode failed!\n");
		return  -EINVAL;
	}

	/*修改当前指向的color_mode? 那check_var需要修改吗?*/
	fbdev->color_mode = color_mode;

	/*构建rdma信息，设置并启动.*/
	struct rdma_setup_info *rdma_info = &fbdev->rdma_info;
	rdma_info->nframes = CONFIG_FB_INGENIC_NR_FRAMES;	//TODO:
	rdma_info->format = color_mode->mode;
	rdma_info->color = color_mode->color;
	rdma_info->stride = var->xres;
	rdma_info->continuous = 1; //TODO:
	rdma_info->vidmem = (unsigned char**)&fbdev->vidmem;
	rdma_info->vidmem_phys = (unsigned int**)&fbdev->vidmem_phys;

	dpu_ctrl_rdma_stop(dctrl, QCK_STOP);

	dpu_ctrl_rdma_setup(dctrl, rdma_info);

	dpu_ctrl_rdma_start(dctrl);

	return 0;
}

static inline int timeval_sub_to_us(struct timespec64 lhs,
						struct timespec64 rhs)
{
	int sec, nsec;
	sec = lhs.tv_sec - rhs.tv_sec;
	nsec = lhs.tv_nsec - rhs.tv_nsec;

	return (sec*1000000 + nsec/1000);
}

static inline int time_us2ms(int us)
{
	return (us/1000);
}

static void calculate_frame_rate(void)
{
	static struct timespec64 time_now, time_last;
	unsigned int interval_in_us;
	unsigned int interval_in_ms;
	static unsigned int fpsCount = 0;

	switch(showFPS){
	case 1:
		fpsCount++;
		ktime_get_real_ts64(&time_now);
		interval_in_us = timeval_sub_to_us(time_now, time_last);
		if ( interval_in_us > (USEC_PER_SEC) ) { /* 1 second = 1000000 us. */
			printk(" Pan display FPS: %d\n",fpsCount);
			fpsCount = 0;
			time_last = time_now;
		}
		break;
	case 2:
		ktime_get_real_ts64(&time_now);
		interval_in_us = timeval_sub_to_us(time_now, time_last);
		interval_in_ms = time_us2ms(interval_in_us);
		printk(" Pan display interval ms: %d\n",interval_in_ms);
		time_last = time_now;
		break;
	default:
		if (showFPS > 3) {
			int d, f;
			fpsCount++;
			ktime_get_real_ts64(&time_now);
			interval_in_us = timeval_sub_to_us(time_now, time_last);
			if (interval_in_us > USEC_PER_SEC * showFPS ) { /* 1 second = 1000000 us. */
				d = fpsCount / showFPS;
				f = (fpsCount * 10) / showFPS - d * 10;
				printk(" Pan display FPS: %d.%01d\n", d, f);
				fpsCount = 0;
				time_last = time_now;
			}
		}
		break;
	}
}

static int ingenicfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int next_frm;
	int ret = 0;

	if (var->xoffset - info->var.xoffset) {
		dev_err(info->dev, "No support for X panning for now\n");
		return -EINVAL;
	}

	fbdev->pan_display_count++;
	if(showFPS){
		calculate_frame_rate();
	}

	next_frm = var->yoffset / var->yres;

	dpu_ctrl_rdma_change(dctrl, next_frm);

	return 0;
}

static int ingenicfb_do_resume(struct ingenicfb_device *fbdev)
{
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int ret = 0;

	ret = dpu_ctrl_resume(dctrl);

	return ret;
}

static int ingenicfb_do_suspend(struct ingenicfb_device *fbdev)
{
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int ret = 0;

	ret = dpu_ctrl_suspend(dctrl);

	return ret;
}

static int ingenicfb_blank(int blank_mode, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	int ret = 0;

	if (blank_mode == FB_BLANK_UNBLANK) {
		ret = ingenicfb_do_resume(fbdev);
	} else {
		ret = ingenicfb_do_suspend(fbdev);
	}

	return ret;
}

static int ingenicfb_open(struct fb_info *info, int user)
{
	struct ingenicfb_device *fbdev = info->par;
	int ret;
#if 0 //TODO:

	if (!fbdev->is_lcd_en && fbdev->vidmem_phys) {
		fbdev->timestamp.rp = 0;
		fbdev->timestamp.wp = 0;
		ingenicfb_set_fix_par(info);
		ret = ingenicfb_set_par(info);
		if(ret) {
			dev_err(info->dev, "Set par failed!\n");
			return ret;
		}
		memset(fbdev->vidmem[fbdev->current_frm_desc][0], 0, fbdev->frm_size);
		ingenicfb_enable(info);
	}
#endif
	dev_dbg(info->dev, "####open count : %d\n", ++fbdev->open_cnt);

	return 0;
}

static int ingenicfb_release(struct fb_info *info, int user)
{
	struct ingenicfb_device *fbdev = info->par;

	dev_dbg(info->dev, "####close count : %d\n", fbdev->open_cnt--);
	if(!fbdev->open_cnt) {
//		fbdev->timestamp.rp = 0;
//		fbdev->timestamp.wp = 0;
	}
	return 0;
}

ssize_t ingenicfb_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct ingenicfb_device *fbdev = info->par;
	u8 *buffer, *src;
	u8 __iomem *dst;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	unsigned long p = *ppos;
	int screen_base_offset = 0;
	int next_frm = 0;

	next_frm = info->var.yoffset / info->var.yres;

	screen_base_offset = next_frm * info->screen_size;

	total_size = info->screen_size;

	if (total_size == 0)
		total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
			 GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	dst = (u8 __iomem *) (info->screen_base + p + screen_base_offset);

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		src = buffer;

		if (copy_from_user(src, buf, c)) {
			err = -EFAULT;
			break;
		}

		fb_memcpy_tofb(dst, src, c);
		dst += c;
		src += c;
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);
end:
	return (cnt) ? cnt : err;
}

static ssize_t
ingenicfb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u8 *buffer, *dst;
	u8 __iomem *src;
	int c, cnt = 0, err = 0;
	unsigned long total_size;

	int screen_base_offset = 0;
	int next_frm = 0;

	next_frm = info->var.yoffset / info->var.yres;

	if (!info || ! info->screen_base)
		return -ENODEV;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->screen_size;

	if (total_size == 0)
		total_size = info->fix.smem_len;

	if (p >= total_size)
		return 0;

	if (count >= total_size)
		count = total_size;

	if (count + p > total_size)
		count = total_size - p;

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
			 GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	src = (u8 __iomem *) (info->screen_base + p + screen_base_offset);

	while (count) {
		c  = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		dst = buffer;
		fb_memcpy_fromfb(dst, src, c);
		dst += c;
		src += c;

		if (copy_to_user(buf, buffer, c)) {
			err = -EFAULT;
			break;
		}
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (err) ? err : cnt;


}

static struct fb_ops ingenicfb_ops = {
	.owner 		= THIS_MODULE,
	.fb_open 	= ingenicfb_open,
	.fb_release 	= ingenicfb_release,
	.fb_write 	= ingenicfb_write,
	.fb_read 	= ingenicfb_read,
	.fb_check_var 	= ingenicfb_check_var,
	.fb_set_par 	= ingenicfb_set_par,
	.fb_setcolreg 	= ingenicfb_setcolreg,
	.fb_blank 	= ingenicfb_blank,
	.fb_pan_display = ingenicfb_pan_display,
	.fb_fillrect 	= cfb_fillrect,
	.fb_copyarea 	= cfb_copyarea,
	.fb_imageblit 	= cfb_imageblit,
	.fb_ioctl 	= ingenicfb_ioctl,
	.fb_mmap 	= ingenicfb_mmap,
};


static int vsync_skip_set(struct ingenicfb_device *fbdev, int vsync_skip)
{
#if 0
	unsigned int map_wide10 = 0;
	int rate, i, p, n;
	int fake_float_1k;

	if (vsync_skip < 0 || vsync_skip > 9)
		return -EINVAL;

	rate = vsync_skip + 1;
	fake_float_1k = 10000 / rate;   /* 10.0 / rate */

	p = 1;
	n = (fake_float_1k * p + 500) / 1000;   /* +0.5 to int */

	for (i = 1; i <= 10; i++) {
		map_wide10 = map_wide10 << 1;
		if (i == n) {
			map_wide10++;
			p++;
			n = (fake_float_1k * p + 500) / 1000;
		}
	}
	mutex_lock(&fbdev->lock);
	fbdev->vsync_skip_map = map_wide10;
	fbdev->vsync_skip_ratio = rate - 1;     /* 0 ~ 9 */
	mutex_unlock(&fbdev->lock);

	printk("vsync_skip_ratio = %d\n", fbdev->vsync_skip_ratio);
	printk("vsync_skip_map = 0x%08x\n", fbdev->vsync_skip_map);
#endif
	return 0;
}


static int ingenicfb_copy_logo(struct fb_info *info)
{

#if 0
	unsigned long src_addr = 0;	/* u-boot logo buffer address */
	unsigned long dst_addr = 0;	/* kernel frame buffer address */
	struct ingenicfb_device *fbdev = info->par;
	unsigned long size;
	unsigned int ctrl;
	unsigned read_times;
	lay_cfg_en_t lay_cfg_en;

	/* Sure the uboot SLCD using the continuous mode, Close irq */
	if (!(reg_read(fbdev, DC_ST) & DC_WORKING)) {
		dev_err(fbdev->dev, "uboot is not display logo!\n");
#ifdef CONFIG_TRUE_COLOR_LOGO
        fbdev->current_frm_desc = 0;
        ingenicfb_set_fix_par(fbdev->fb);
        ingenicfb_set_par(fbdev->fb);
        ingenicfb_enable(fbdev->fb);
        show_logo(fbdev->fb);
        fb_blank(fbdev->fb, FB_BLANK_UNBLANK);
#endif
		return -ENOEXEC;
	}

	/*fbdev->is_lcd_en = 1;*/

	/* Reading Desc from regisger need reset */
	ctrl = reg_read(fbdev, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(fbdev, DC_CTRL, ctrl);

	/* For geting LayCfgEn need read  DC_FRM_DES 10 times */
	read_times = 10 - 1;
	while(read_times--) {
		reg_read(fbdev, DC_FRM_DES);
	}
	lay_cfg_en.d32 = reg_read(fbdev, DC_FRM_DES);
	if(!lay_cfg_en.b.lay0_en) {
		dev_err(fbdev->dev, "Uboot initialization is not using layer0!\n");
		return -ENOEXEC;
	}

	/* For geting LayerBufferAddr need read  DC_LAY0_DES 3 times */
	read_times = 3 - 1;
	/* get buffer physical address */
	while(read_times--) {
		reg_read(fbdev, DC_LAY0_DES);
	}
	src_addr = (unsigned long)reg_read(fbdev, DC_LAY0_DES);

	if (src_addr) {
		size = info->fix.line_length * info->var.yres;
		src_addr = (unsigned long)phys_to_virt(src_addr);
		dst_addr = (unsigned long)fbdev->vidmem[0][0];
		memcpy((void *)dst_addr, (void *)src_addr, size);
	}
#endif
	return 0;
}

static int ingenicfb_alloc_vidmem(struct ingenicfb_device *fbdev, unsigned int x_res, unsigned int y_res, unsigned int bits_per_pixel)
{
	unsigned int alloc_size = 0;
	int i = 0;

	fbdev->vidmem_size = 0;

	alloc_size = x_res * y_res;
	fbdev->frm_size = alloc_size * bits_per_pixel >> 3;
	alloc_size *= MAX_BITS_PER_PIX >> 3;

	alloc_size = alloc_size * CONFIG_FB_INGENIC_NR_FRAMES;
	fbdev->vidmem[0] = dma_alloc_coherent(fbdev->dev, alloc_size, &fbdev->vidmem_phys[0], GFP_KERNEL);

	if(IS_ERR_OR_NULL(fbdev->vidmem[0])) {
		return -ENOMEM;
	}
	fbdev->vidmem_size = alloc_size;

	for(i = 1; i < CONFIG_FB_INGENIC_NR_FRAMES; i++) {
		fbdev->vidmem[i] = fbdev->vidmem[0] + i * fbdev->frm_size;
		fbdev->vidmem_phys[i] = fbdev->vidmem_phys[0] + i * fbdev->frm_size;
	}

	dev_info(fbdev->dev, "vidmem @ 0x%x size %d\n", fbdev->vidmem[0], fbdev->vidmem_size);

	return 0;
}

int ingenicfb_release_vidmem(struct ingenicfb_device *fbdev)
{
	if(fbdev->vidmem_size) {
		dma_free_coherent(fbdev->dev, fbdev->vidmem_size, fbdev->vidmem[0], fbdev->vidmem_phys[0]);
		fbdev->vidmem_size = 0;
	}

	return 0;
}

int pan_init_logo(struct fb_info *fb)
{

#ifdef CONFIG_TRUE_COLOR_LOGO
	if(!copyed_logo_buf || !fb) {
		return 0;
	}

	logo_info.p8 = copyed_logo_buf;
	show_logo(fb);
	/* free logo mem */
//	kfree(copyed_logo_buf);
//	copyed_logo_buf = NULL;
#endif

	return 0;
}
static int ingenicfb_do_probe(struct platform_device *pdev, struct lcd_panel *panel)
{
	struct fb_videomode *video_mode;
	struct fb_info *fb;
	struct dpu_ctrl *dctrl;
	int ret = 0;

	const struct of_device_id *match;
	struct ingenic_priv *ingenic_priv;

	fb = framebuffer_alloc(sizeof(struct ingenicfb_device), &pdev->dev);
	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		return -ENOMEM;
	}


	fbdev = fb->par;
	fbdev->fb = fb;
	fbdev->dev = &pdev->dev;

	fbdev->dmn.dev = fbdev->dev;
	fbdev->dmn.data = fbdev;
	fbdev->dmn.ops = &ingenicfb_dmmu_mm_ops;

	of_property_read_u32(fbdev->dev->of_node, "ingenic,disable-rdma-fb", &fbdev->disable_rdma_fb);

	match = of_match_node(ingenicfb_of_match, pdev->dev.of_node);
	if (!match)
		        return -ENODEV;
	ingenic_priv = (struct ingenic_priv *)match->data;
	if(panel->dsi_pdata){
		panel->dsi_pdata->dsi_iobase = ingenic_priv->dsi_iobase;
		panel->dsi_pdata->dsi_phy_iobase = ingenic_priv->dsi_phy_iobase;
	}
	fbdev->panel = panel;

	/*一个屏幕可能由多个modes, 将所有的modes转换成modelist.*/
	fb_videomode_to_modelist(panel->modes, panel->num_modes, &fb->modelist);

	/*同一时刻应该只有一个video_mode, 所以要找到active_video_mode.*/
	fbdev->active_video_mode = &panel->modes[0];
	ingenicfb_videomode_to_var(&fb->var, fbdev->active_video_mode);

	ret = refresh_pixclock_auto_adapt(fb, fbdev->active_video_mode);
	if(ret) {
		goto err_calc_pixclk;
	}

	dctrl = &fbdev->dctrl;

	dctrl->dev = &pdev->dev;
	dctrl->pdev = pdev;
	dctrl->active_video_mode = fbdev->active_video_mode;

	dctrl->set_vsync_value = ingenicfb_set_vsync_value;
	dctrl->vsync_data = fbdev;
	dctrl->support_comp = ingenic_priv->support_comp;

	ret = dpu_ctrl_init(dctrl, panel);
	if(ret) {
		goto err_dctrl_init;
	}
	panel_mipi_init_ok = true;

	if(ingenic_priv->support_comp) {
		// composer master
		fbdev->comp_master = hw_composer_init(dctrl);
		if(IS_ERR_OR_NULL(fbdev->comp_master)) {
			goto err_comp_init;
		}

		// one composer instance for fbdev.
		// using by ioctl procedure.
		fbdev->comp_ctx = hw_composer_create(fbdev);
		if(IS_ERR_OR_NULL(fbdev->comp_ctx)) {
			goto err_comp_ctx;
		}
	}

	/*width只是物理尺寸，以mm为单位。可以用于计算PPI*/
	fb->fbops = &ingenicfb_ops;
	fb->flags = FBINFO_DEFAULT;
	fb->var.width = panel->width;
	fb->var.height = panel->height;

	/*默认使用第一个colormode. */
	fbdev->color_mode = &ingenicfb_colormodes[0];
	ingenicfb_colormode_to_var(&fb->var, fbdev->color_mode);

	ret = ingenicfb_check_var(&fb->var, fb);
	if (ret) {
		goto err_check_var;
	}

	platform_set_drvdata(pdev, fbdev);

	vsync_skip_set(fbdev, CONFIG_FB_VSYNC_SKIP);
	init_waitqueue_head(&fbdev->vsync_wq);


	fbdev->open_cnt = 0;
	fbdev->is_lcd_en = 0;
	fbdev->timestamp.rp = 0;
	fbdev->timestamp.wp = 0;


	if(!fbdev->disable_rdma_fb) {

		ret = ingenicfb_alloc_vidmem(fbdev, fb->var.xres, fb->var.yres, fb->var.bits_per_pixel);
		if (ret) {
			dev_err(&pdev->dev, "Failed to allocate video memory\n");
			goto err_alloc_vidmem;
		}

		fb->fix = ingenicfb_fix;
		fb->fix.line_length = (fb->var.bits_per_pixel * fb->var.xres) >> 3;
		fb->fix.smem_start = fbdev->vidmem_phys[0];
		fb->fix.smem_len = fbdev->vidmem_size;
		fb->screen_size = fb->fix.line_length * fb->var.yres;
		fb->screen_base = fbdev->vidmem[0];
		fb->pseudo_palette = fbdev->pseudo_palette;

		ret = register_framebuffer(fb);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register framebuffer: %d\n",
					ret);
			goto err_register_framebuffer;
		}


	}

	/*1. Common setup. */
	dctrl->chan = DATA_CH_RDMA;
	dpu_ctrl_setup(dctrl);


	if(!fbdev->disable_rdma_fb) {
		struct rdma_setup_info *rdma_info = &fbdev->rdma_info;
		rdma_info->nframes = CONFIG_FB_INGENIC_NR_FRAMES;	//TODO:
		rdma_info->format = fbdev->color_mode->mode;
		rdma_info->color = fbdev->color_mode->color;
		rdma_info->stride = fb->var.xres;
		rdma_info->continuous = 1; //TODO:
		rdma_info->vidmem = (unsigned char**)&fbdev->vidmem;
		rdma_info->vidmem_phys = (unsigned int**)&fbdev->vidmem_phys;

		dpu_ctrl_rdma_setup(dctrl, rdma_info);

		dpu_ctrl_rdma_start(dctrl);

		pan_init_logo(fbdev->fb);
	}


	if(ingenic_priv->support_comp) {
		/*2. compfb init and export.*/
		fbdev->compfb = hw_compfb_init(fbdev);
		if(IS_ERR_OR_NULL(fbdev->compfb)) {
			dev_err(&pdev->dev, "Failed to init compfb!\n");
			goto err_compfb_init;
		}

		/*3. export composer.*/
		fbdev->compfb->active_video_mode = fbdev->active_video_mode;
		fbdev->compfb->dev = &pdev->dev;
		fbdev->compfb->panel = panel;
	}

	{
		fbdev->comp_v4l2 = hw_comp_v4l2_init(&pdev->dev);
	}

	ret = dpu_sysfs_init(&fbdev->sysfs, dctrl, fbdev->compfb);
	if(ret < 0) {
		dev_err(fbdev->dev, "failed to init sysfs!\n");
	}


	return 0;

err_compfb_init:
err_register_framebuffer:
err_alloc_vidmem:
err_check_var:
err_comp_ctx:
	hw_composer_exit(fbdev->comp_master);
err_comp_init:
	dpu_ctrl_exit(dctrl);
err_dctrl_init:
	fb_delete_videomode(panel->modes, &fb->modelist);
	fb_destroy_modelist(&fb->modelist);
err_calc_pixclk:
	framebuffer_release(fb);
	return ret;
}

int ingenicfb_register_panel(struct lcd_panel *panel)
{
	WARN_ON(fbdev_panel != NULL);

	if(fbdev_pdev != NULL) {
		if(panel_mipi_init_ok == false)
			return ingenicfb_do_probe(fbdev_pdev, panel);
		else
			return -EBUSY;
	}

	fbdev_panel = panel;

	return 0;
}
EXPORT_SYMBOL_GPL(ingenicfb_register_panel);

static int ingenicfb_probe(struct platform_device *pdev)
{
	WARN_ON(fbdev_pdev != NULL);

	fbdev_pdev = pdev;

	if(fbdev_panel != NULL) {
		return ingenicfb_do_probe(fbdev_pdev, fbdev_panel);
	}

	return 0;
}

static int ingenicfb_remove(struct platform_device *pdev)
{
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	// TODO:
	// release comp_ctx, compfb, comp_v4l2.

	hw_compfb_exit(fbdev->compfb);
	hw_comp_v4l2_exit(fbdev->comp_v4l2);
	hw_composer_destroy(fbdev->comp_ctx);
	hw_composer_exit(fbdev->comp_master);

	dpu_ctrl_exit(&fbdev->dctrl);

	platform_set_drvdata(pdev, NULL);

	dpu_sysfs_exit(&fbdev->sysfs);

	ingenicfb_release_vidmem(fbdev);
	framebuffer_release(fbdev->fb);

	return 0;
}

static void ingenicfb_shutdown(struct platform_device *pdev)
{
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	if(!IS_ERR_OR_NULL(fbdev))
		fb_blank(fbdev->fb, FB_BLANK_POWERDOWN);
};

#ifdef CONFIG_PM

static int ingenicfb_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	if(!IS_ERR_OR_NULL(fbdev))
		fb_blank(fbdev->fb, FB_BLANK_POWERDOWN);

	return 0;
}

static int ingenicfb_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	if(!IS_ERR_OR_NULL(fbdev))
		fb_blank(fbdev->fb, FB_BLANK_UNBLANK);

	return 0;
}

static const struct dev_pm_ops ingenicfb_pm_ops = {
	.suspend = ingenicfb_suspend,
	.resume = ingenicfb_resume,
};
#endif

static struct ingenic_priv x2000_m300_priv = {
	.dsi_iobase = 0x10075000,
	.dsi_phy_iobase = 0x10077000,
	.support_comp = true,
};

static struct ingenic_priv x2500_priv = {
	.dsi_iobase = 0x10003000,
	.dsi_phy_iobase = 0x10004000,
	.support_comp = true,
};

static struct ingenic_priv x1600_priv = {
	.support_comp = false,
};

static const struct of_device_id ingenicfb_of_match[] = {
	{ .compatible = "ingenic,x2000-dpu",.data = (void*)&x2000_m300_priv},
	{ .compatible = "ingenic,x2500-dpu",.data = (void*)&x2500_priv},
	{ .compatible = "ingenic,m300-dpu",.data = (void*)&x2000_m300_priv},
	{ .compatible = "ingenic,x1600-dpu",.data = (void*)&x1600_priv},
	{},
};

static struct platform_driver ingenicfb_driver = {
	.probe = ingenicfb_probe,
	.remove = ingenicfb_remove,
	.shutdown = ingenicfb_shutdown,
	.driver = {
		.name = "ingenic-fb",
		.of_match_table = ingenicfb_of_match,
#ifdef CONFIG_PM
		.pm = &ingenicfb_pm_ops,
#endif

	},
};

static int __init ingenicfb_init(void)
{
#ifdef CONFIG_TRUE_COLOR_LOGO
	/* copy logo_buf from .init.data section */
	int size;
	size = logo_info.width*logo_info.height*(logo_info.bpp/8);
	printk(KERN_INFO "copy logo_buf from .init.data section, size=%d\n", size);
	copyed_logo_buf = kmalloc(size, GFP_KERNEL);
	memcpy(copyed_logo_buf, &logo_buf_initdata[0], size);
#endif
	platform_driver_register(&ingenicfb_driver);
	return 0;
}

static void __exit ingenicfb_cleanup(void)
{
	platform_driver_unregister(&ingenicfb_driver);
}


#ifdef CONFIG_EARLY_INIT_RUN
rootfs_initcall(ingenicfb_init);
#else
module_init(ingenicfb_init);
#endif

module_exit(ingenicfb_cleanup);

MODULE_DESCRIPTION("JZ LCD Controller driver");
MODULE_AUTHOR("qipenzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL");
