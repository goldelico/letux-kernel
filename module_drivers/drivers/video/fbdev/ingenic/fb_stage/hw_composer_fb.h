#ifndef __HW_COMPFB_DEVICE_H__
#define __HW_COMPFB_DEVICE_H__


struct compfb_colormode {
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

struct export_info {
	int layer_exported[DPU_SUPPORT_MAX_LAYERS];
	int layer_frames[DPU_SUPPORT_MAX_LAYERS];	/*how many framebuffers for layer.*/
	int layer_frame_size_w[DPU_SUPPORT_MAX_LAYERS];	/*存储当前layer最大frame宽高, 设置的src_size必须小于frame_size.*/
	int layer_frame_size_h[DPU_SUPPORT_MAX_LAYERS];
	int layer_color_mode[DPU_SUPPORT_MAX_LAYERS];		/*存储当前layer的格式信息.*/
	int wback_exported;

	struct ingenicfb_frm_cfg frm_cfg;
};

struct hw_compfb_device {

	struct fb_info *fbs[DPU_SUPPORT_MAX_LAYERS];
	struct fb_info *wbfb;
	struct hw_composer_ctx *comp_ctx;
	struct hw_composer_master *comp_master;
	struct device *dev;
	struct lcd_panel *panel;

	int logo_pan_layer;	//init logo pan layer.


	struct export_info export_info;

	/*每个Layer的显存.*/
	unsigned char *vidmem[DPU_SUPPORT_MAX_LAYERS][CONFIG_FB_INGENIC_NR_FRAMES];
	dma_addr_t vidmem_phys[DPU_SUPPORT_MAX_LAYERS][CONFIG_FB_INGENIC_NR_FRAMES];
	unsigned int vidmem_size[DPU_SUPPORT_MAX_LAYERS];
	unsigned int frm_size[DPU_SUPPORT_MAX_LAYERS];

	/*writeback memory*/
	dma_addr_t wback_phys;
	unsigned int wback_vidmem;

	int exported;		/*state of export*/
	unsigned int pseudo_palette[16];

	struct fb_videomode *active_video_mode;

	struct comp_setup_info comp_info;	//实例化direct out 配置.

	struct compfb_colormode *color_modes[DPU_SUPPORT_MAX_LAYERS];//当前各层使用的colormode.
	struct compfb_colormode *wb_color_mode;//当前各层使用的colormode.
};



struct hw_compfb_device *hw_compfb_init(void *data);

int hw_compfb_exit(struct hw_compfb_device *compfb);

int hw_compfb_export(struct hw_compfb_device *compfb, unsigned int width, unsigned int height, int wback);

/* export composer layer and writback mem to fb.*/
int hw_compfb_export2(struct hw_compfb_device *compfb, struct export_info *info);

void hw_compfb_unexport(struct hw_compfb_device *compfb);

/* update composer configurations after export or modify layer configs. */
int hw_compfb_update(struct hw_compfb_device *compfb);

extern int ingenicfb_mmap(struct fb_info *info, struct vm_area_struct *vma);
extern ssize_t ingenicfb_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos);

extern const struct fb_fix_screeninfo ingenicfb_fix;
#endif
