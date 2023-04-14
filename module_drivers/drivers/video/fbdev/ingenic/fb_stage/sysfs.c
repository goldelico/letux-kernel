#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/fb.h>

#include "sysfs.h"
#include "ingenicfb.h"
#include "hw_composer_fb.h"
#include "uapi_ingenicfb.h"

static ssize_t
dump_lcd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dpu_ctrl *dctrl = sysfs_get_dctrl(dev);

	dump_lcdc_registers(dctrl);

	dump_comp_desc(dctrl);

	dump_rdma_desc(dctrl);

	return 0;
}

	static ssize_t
dump_irqcnts(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dpu_ctrl *dctrl = sysfs_get_dctrl(dev);
	struct dbg_irqcnt *dbg = &dctrl->dbg_irqcnt;
	char *p = buf;

	p += sprintf(p, "comp_fps	: %lld\n", dctrl->comp_fps);
	p += sprintf(p, "---------------------\n");
	p += sprintf(p, "irqcnt		: %lld\n", dbg->irqcnt);
	p += sprintf(p, "cmp_start	: %lld\n", dbg->cmp_start);
	p += sprintf(p, "stop_disp_ack	: %lld\n", dbg->stop_disp_ack);
	p += sprintf(p, "disp_end	: %lld\n", dbg->disp_end);
	p += sprintf(p, "tft_under	: %lld\n", dbg->tft_under);
	p += sprintf(p, "wdma_over	: %lld\n", dbg->wdma_over);
	p += sprintf(p, "wdma_end	: %lld\n", dbg->wdma_end);
	p += sprintf(p, "layer3_end	: %lld\n", dbg->layer3_end);
	p += sprintf(p, "layer2_end	: %lld\n", dbg->layer2_end);
	p += sprintf(p, "layer1_end	: %lld\n", dbg->layer1_end);
	p += sprintf(p, "layer0_end	: %lld\n", dbg->layer0_end);
	p += sprintf(p, "clr_cmp_end	: %lld\n", dbg->clr_cmp_end);
	p += sprintf(p, "stop_wrbk_ack	: %lld\n", dbg->stop_wrbk_ack);
	p += sprintf(p, "srd_start	: %lld\n", dbg->srd_start);
	p += sprintf(p, "srd_end	: %lld\n", dbg->srd_end);
	p += sprintf(p, "cmp_w_slow	: %lld\n", dbg->cmp_w_slow);

	return p - buf;
}


static DEVICE_ATTR(dump_lcd, S_IRUGO|S_IWUSR, dump_lcd, NULL);
static DEVICE_ATTR(dump_irqcnts, (S_IRUGO|S_IWUGO) & (~S_IWOTH), dump_irqcnts, NULL);

/*Debug sysfs*/
static struct attribute *lcd_debug_attrs[] = {
	&dev_attr_dump_lcd.attr,
	&dev_attr_dump_irqcnts.attr,
        NULL,
};

const char lcd_group_name[] = "debug";
static struct attribute_group lcd_debug_attr_group = {
        .name   = lcd_group_name,
        .attrs  = lcd_debug_attrs,
};

/* ===========================Composer=============================*/

static ssize_t store_frame_size(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *p;
	char *s = (char *)buf;

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}

	if(info->layer_exported[layer] == 1) {
		return -EBUSY;
	}

	info->layer_frame_size_w[layer] = simple_strtoul(p, NULL, 0);
	info->layer_frame_size_h[layer] = simple_strtoul(s, NULL, 0);

	return n;
}


static ssize_t store_src_size(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *p;
	char *s = (char *)buf;
	unsigned int src_w = 0;
	unsigned int src_h = 0;

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}

	src_w = simple_strtoul(p, NULL, 0);
	src_h = simple_strtoul(s, NULL, 0);

	if((src_w * src_h) > (info->layer_frame_size_w[layer] * info->layer_frame_size_h[layer])) {
		dev_err(dev, "src_size(%xx%x) should smaller than frame_size(%xx%x)",
				src_w, src_h,
				info->layer_frame_size_w[layer], info->layer_frame_size_h[layer]);
		return -EINVAL;
	}

	lay_cfg->source_w = src_w;
	lay_cfg->source_h = src_h;

	return n;
}

static ssize_t store_stride(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *s = (char *)buf;
	unsigned int stride = 0;

	stride = simple_strtoul(s, NULL, 0);

	lay_cfg->stride = stride;

	return n;
}

static ssize_t store_uv_stride(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *s = (char *)buf;
	unsigned int stride = 0;

	stride = simple_strtoul(s, NULL, 0);

	lay_cfg->uv_stride = stride;

	return n;
}

static ssize_t store_z_order(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *s = (char *)buf;
	unsigned int z_order = 0;

	z_order = simple_strtoul(s, NULL, 0);

	lay_cfg->lay_z_order = z_order;

	return n;
}

static ssize_t store_src_fmt(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	int format = simple_strtoul(buf, NULL, 0);

	info->layer_color_mode[layer] = format;

	return n;

}

static ssize_t store_target_size(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *p;
	char *s = (char *)buf;

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}


	lay_cfg->scale_w = simple_strtoul(p, NULL, 0);
	lay_cfg->scale_h = simple_strtoul(s, NULL, 0);

	return n;
}

static ssize_t store_target_pos(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *p;
	char *s = (char *)buf;

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}


	lay_cfg->disp_pos_x = simple_strtoul(p, NULL, 0);
	lay_cfg->disp_pos_y = simple_strtoul(s, NULL, 0);

	return n;
}

static ssize_t store_nframes(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;

	if(info->layer_exported[layer] == 1) {
		return -EBUSY;
	}

	info->layer_frames[layer] = simple_strtoul(buf, NULL, 0);

	return n;
}

/*tlb_en:buf_addr:uv_addr*/
static ssize_t store_buf_addr(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	char *p;
	char *s = (char *)buf;

	p = strsep(&s, ":");
	if(!s) {
		return -EINVAL;
	}

	lay_cfg->addr[0] = simple_strtoul(p, NULL, 16);
	lay_cfg->uv_addr[0] = simple_strtoul(s, NULL, 16);

	return n;
}

static ssize_t store_enable(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	if(lay_cfg->scale_w == 0 || lay_cfg->scale_h == 0) {
		dev_err(compfb->dev, "invalid scale_w and scale_h for layer: %d\n", layer);
	}

	lay_cfg->lay_en = simple_strtoul(buf, NULL, 0);

	return n;
}

static ssize_t show_exportfb(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	int layer = to_layer_attr(attr)->id;

	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];
	int ret = 0;
	char *p = buf;

	p += sprintf(p, "==== info ====\n");
	p += sprintf(p, "export:%d\n", info->layer_exported[layer]);
	p += sprintf(p, "src:%dx%d\n", lay_cfg->source_w, lay_cfg->source_h);
	p += sprintf(p, "stride:%d\n", lay_cfg->stride);
	p += sprintf(p, "uv_stride:%d\n", lay_cfg->uv_stride);
	p += sprintf(p, "z_order:%d\n", lay_cfg->lay_z_order);
	p += sprintf(p, "fmt:%d \n", info->layer_color_mode[layer]);
	p += sprintf(p, "target:%dx%d\n", lay_cfg->scale_w, lay_cfg->scale_h);
	p += sprintf(p, "pos:%dx%d\n", lay_cfg->disp_pos_x, lay_cfg->disp_pos_y);
	p += sprintf(p, "nframes:%d\n", info->layer_frames[layer]);
	p += sprintf(p, "addr:%d\n", lay_cfg->addr[0]);
	p += sprintf(p, "uv_buf_addr:%d\n", lay_cfg->uv_addr[0]);
	p += sprintf(p, "framesize:%dx%d\n", info->layer_frame_size_w[layer], info->layer_frame_size_h[layer]);
	p += sprintf(p, "enable:%d\n", lay_cfg->lay_en);

	return p - buf;
}

static ssize_t store_exportfb(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	struct export_info *info = &compfb->export_info;
	int layer = to_layer_attr(attr)->id;

	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	int export = simple_strtoul(buf, NULL, 0);

	if(export) {
		/*TODO:add Lock.*/
		if(info->layer_exported[layer] == 1) {
			return -EBUSY;
		}

		if(lay_cfg->source_w == 0 || lay_cfg->source_h == 0) {
			dev_err(compfb->dev, "Invalid layer src_size %dx%d\n", lay_cfg->source_w, lay_cfg->source_h);
			return -EINVAL;
		}

		if(info->layer_frame_size_w[layer] == 0 || info->layer_frame_size_h[layer] == 0) {
			dev_err(compfb->dev, "Invalid layer_framesize %dx%d, wrap to lay_cfg src_size: %dx%d\n",
					info->layer_frame_size_w[layer], info->layer_frame_size_h[layer],
					lay_cfg->source_w, lay_cfg->source_h);
			info->layer_frame_size_w[layer] = lay_cfg->source_w;
			info->layer_frame_size_h[layer] = lay_cfg->source_h;
		}

		info->layer_exported[layer] = 1;
	} else {
		/*TODO:add Lock.*/
		if(info->layer_exported[layer] == 0) {
			return n;
		}

		/*Do unexport.*/
		info->layer_exported[layer] = 0;
	}

	return  n;
}

LAYER_DEVICE_ATTR(frame_size, S_IRUGO|S_IWUSR, NULL, store_frame_size);
LAYER_DEVICE_ATTR(src_size, S_IRUGO|S_IWUSR, NULL, store_src_size);
LAYER_DEVICE_ATTR(stride, S_IRUGO|S_IWUSR, NULL, store_stride);
LAYER_DEVICE_ATTR(uv_stride, S_IRUGO|S_IWUSR, NULL, store_uv_stride);
LAYER_DEVICE_ATTR(z_order, S_IRUGO|S_IWUSR, NULL, store_z_order);
LAYER_DEVICE_ATTR(src_fmt, S_IRUGO|S_IWUSR, NULL, store_src_fmt);
LAYER_DEVICE_ATTR(target_size, S_IRUGO|S_IWUSR, NULL, store_target_size);
LAYER_DEVICE_ATTR(target_pos, S_IRUGO|S_IWUSR, NULL, store_target_pos);
LAYER_DEVICE_ATTR(nframes, S_IRUGO|S_IWUSR, NULL, store_nframes);
LAYER_DEVICE_ATTR(buf_addr, S_IRUGO|S_IWUSR, NULL, store_buf_addr);
LAYER_DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, NULL, store_enable);
LAYER_DEVICE_ATTR(exportfb, S_IRUGO|S_IWUSR, show_exportfb, store_exportfb);


#define LAYER_ATTRIBUTE_GROUP(name)                                     \
        static struct attribute *comp_##name##_attrs[] = {               \
		&dev_attr_frame_size##name.attr.attr,			\
                &dev_attr_src_size##name.attr.attr,                     \
                &dev_attr_stride##name.attr.attr,                     \
                &dev_attr_uv_stride##name.attr.attr,                     \
                &dev_attr_z_order##name.attr.attr,                     \
                &dev_attr_src_fmt##name.attr.attr,                      \
                &dev_attr_target_size##name.attr.attr,                  \
                &dev_attr_target_pos##name.attr.attr,                   \
                &dev_attr_nframes##name.attr.attr,                       \
                &dev_attr_buf_addr##name.attr.attr,                       \
                &dev_attr_enable##name.attr.attr,                       \
                &dev_attr_exportfb##name.attr.attr,                       \
                NULL,                                                   \
        };


LAYER_ATTRIBUTE_GROUP(layer0);
LAYER_ATTRIBUTE_GROUP(layer1);
LAYER_ATTRIBUTE_GROUP(layer2);
LAYER_ATTRIBUTE_GROUP(layer3);

static struct attribute_group comp_layer0_group = {
                .name = "layer0",
                .attrs = comp_layer0_attrs,
};
static struct attribute_group comp_layer1_group = {
                .name = "layer1",
                .attrs = comp_layer1_attrs,
};
static struct attribute_group comp_layer2_group = {
                .name = "layer2",
                .attrs = comp_layer2_attrs,
};
static struct attribute_group comp_layer3_group = {
                .name = "layer3",
                .attrs = comp_layer3_attrs,
};

static struct attribute *comp_wback_attrs[] = {
#if 0
	&dev_attr_wback_framesize.attr,
	&dev_attr_wback_fmt.attr,
	&dev_attr_wback_enable.attr,
	&dev_attr_wback_export.attr,
#endif
        NULL,
};

static struct attribute_group comp_wback_group = {
	.name = "wback",
	.attrs = comp_wback_attrs,
};

static const struct attribute_group *comp_groups[] = {
        &comp_layer0_group,
        &comp_layer1_group,
        &comp_layer2_group,
        &comp_layer3_group,
	&comp_wback_group,
        NULL,
};



static ssize_t show_comp_exportfbs(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_comp_exportfbs(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	struct export_info *info = &compfb->export_info;
	int ret = 0;

	int export = simple_strtoul(buf, NULL, 0);

	if(export == 1) {
		ret = hw_compfb_export2(compfb, info);
		if(ret < 0) {
			return ret;
		}
	} else {
		hw_compfb_unexport(compfb);
	}

	return n;
}

static ssize_t store_comp_update(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	struct export_info *info = &compfb->export_info;
	int ret = 0;

	ret = hw_compfb_update(compfb);

	return n;
}


static ssize_t store_rdma_destroy(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = sysfs_get_fbdev(dev);
	struct dpu_ctrl *dctrl = sysfs_get_dctrl(dev);
	struct hw_compfb_device *compfb = sysfs_get_compfb(dev);
	struct export_info *info = &compfb->export_info;
	int ret = 0;
	int value = simple_strtoul(buf, NULL, 0);

	if(value != 1) {
		return -EINVAL;
	}

	ret = dpu_ctrl_try_rdma_release(dctrl);
	if(ret < 0) {
		return ret;
	}

#if 0
	unregister_framebuffer(fbdev->fb);
	framebuffer_release(fbdev->fb);
	fbdev->fb = NULL;
#endif
	ingenicfb_release_vidmem(fbdev);
	return n;
}

static DEVICE_ATTR(comp_exportfbs, S_IRUGO|S_IWUSR, show_comp_exportfbs, store_comp_exportfbs);
static DEVICE_ATTR(comp_update, S_IRUGO|S_IWUSR, show_comp_exportfbs, store_comp_update);
static DEVICE_ATTR(rdma_destroy, S_IWUSR, NULL, store_rdma_destroy);

static int of_sysfs_parse_export_info(struct hw_compfb_device *compfb)
{
	struct device_node *np = compfb->dev->of_node;
	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	unsigned int value[8];
	int ret = 0;
	unsigned int comp_fb = 0;
	int i = 0;

	if(of_property_read_u32_array(np, "ingenic,layer-exported", info->layer_exported, 4) < 0) {
		info->layer_exported[0] = 0;
		info->layer_exported[1] = 0;
		info->layer_exported[2] = 0;
		info->layer_exported[3] = 0;
	}

	if(of_property_read_u32_array(np, "ingenic,layer-frames", info->layer_frames, 4) < 0) {
		info->layer_frames[0] = 1;
		info->layer_frames[1] = 1;
		info->layer_frames[2] = 1;
		info->layer_frames[3] = 1;
	}

	if(of_property_read_u32_array(np, "layer,color_mode", info->layer_color_mode, 4) < 0) {
		info->layer_color_mode[0] = 0;
		info->layer_color_mode[1] = 0;
		info->layer_color_mode[2] = 0;
		info->layer_color_mode[3] = 0;
	}


	ret = of_property_read_u32_array(np, "layer,src-size", value, 8);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->source_w = value[i*2 + 0];
			lay_cfg->source_h = value[i*2 + 1];
		}
	}

	ret = of_property_read_u32_array(np, "layer,stride", value, 4);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->stride = value[i];
		}
	} else {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];
			lay_cfg->stride = lay_cfg->source_w;
		}
	}

	ret = of_property_read_u32_array(np, "layer,uv_stride", value, 4);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->uv_stride = value[i];
		}
	} else {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];
			lay_cfg->uv_stride = lay_cfg->source_w;
		}
	}

	ret = of_property_read_u32_array(np, "layer,z_order", value, 4);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->lay_z_order = value[i];
		}
	} else {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];
			lay_cfg->lay_z_order = LAYER_Z_ORDER_3 - i;
		}
	}

	ret = of_property_read_u32_array(np, "ingenic,layer-framesize", value, 8);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			info->layer_frame_size_w[i] = value[i*2 + 0];
			info->layer_frame_size_h[i] = value[i*2 + 1];
		}
	} else {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];
			info->layer_frame_size_w[i] = lay_cfg->source_w;
			info->layer_frame_size_h[i] = lay_cfg->source_h;
		}

	}

	ret = of_property_read_u32_array(np, "layer,target-size", value, 8);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->scale_w = value[i*2 + 0];
			lay_cfg->scale_h = value[i*2 + 1];
		}
	}

	ret = of_property_read_u32_array(np, "layer,target-pos", value, 8);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];

			lay_cfg->disp_pos_x = value[i*2 + 0];
			lay_cfg->disp_pos_y = value[i*2 + 1];
		}
	}

	ret = of_property_read_u32_array(np, "layer,enable", value, 4);
	if(ret == 0) {
		for(i = 0; i < 4; i++) {
			struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[i];
			lay_cfg->lay_en = info->layer_exported[i] ? value[i] : 0;
		}
	}



	of_property_read_u32(np, "ingenic,logo-pan-layer", &compfb->logo_pan_layer);

	return 0;
}

int dpu_sysfs_init(struct dpu_sysfs *sysfs, struct dpu_ctrl *dctrl, struct hw_compfb_device *compfb)
{
	int ret = 0;
	struct export_info *info = &compfb->export_info;
	int export = 0;
	int i = 0;

	sysfs->dctrl = dctrl;
	sysfs->compfb = compfb;

	/*1. create debug group.*/
	ret = sysfs_create_group(&dctrl->dev->kobj, &lcd_debug_attr_group);
	if (ret) {
		dev_err(dctrl->dev, "device create sysfs group failed\n");
		ret = -EINVAL;
	}
	if(dctrl->support_comp) {
		/*2. create comp group.*/
		ret = sysfs_create_groups(&dctrl->dev->kobj, comp_groups);
		if(ret) {
			dev_err(dctrl->dev, "Failed to create sysfs groups\n");
		}

		/*3. comp controller*/
		ret = device_create_file(dctrl->dev, &dev_attr_comp_exportfbs);
		if(ret) {
			dev_err(dctrl->dev, "Failed to create comp_exportfbs\n");
		}

		ret = device_create_file(dctrl->dev, &dev_attr_comp_update);
		if(ret) {
			dev_err(dctrl->dev, "Failed to create comp_update\n");
		}
	}
	ret = device_create_file(dctrl->dev, &dev_attr_rdma_destroy);
	if(ret) {
		dev_err(dctrl->dev, "Failed to create rdma_destroy\n");
	}

	if(dctrl->support_comp) {
		of_sysfs_parse_export_info(compfb);
		for(i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
			if(info->layer_exported[i] == 1) {
				export = 1;
				break;
			}
		}
	}

	/*Try to export compfb at init stage if dts has set export values.*/
	if(export) {
		ret = hw_compfb_export2(compfb, info);
		if(ret < 0) {
			dev_err(compfb->dev, "Failed to export composer fbs.\n");
			goto err_export;
		}

		pan_init_logo(compfb->fbs[compfb->logo_pan_layer]);

#if 1
		pan_init_logo(compfb->fbs[1]);
		pan_init_logo(compfb->fbs[2]);
		pan_init_logo(compfb->fbs[3]);
#endif
		ret = hw_compfb_update(compfb);
		if(ret < 0) {
			dev_err(compfb->dev, "failed to update composer!\n");
		}
	}

	return ret;
err_export:
	return ret;
}


int dpu_sysfs_exit(struct dpu_sysfs *sysfs)
{
	struct dpu_ctrl *dctrl = sysfs->dctrl;
	struct hw_compfb_device *compfb = sysfs->compfb;
	int ret = 0;

	if(dctrl->support_comp)
		hw_compfb_unexport(compfb);

	device_remove_file(dctrl->dev, &dev_attr_rdma_destroy);

	if(dctrl->support_comp) {
		device_remove_file(dctrl->dev, &dev_attr_comp_update);
		device_remove_file(dctrl->dev, &dev_attr_comp_exportfbs);
		sysfs_remove_groups(&dctrl->dev->kobj, comp_groups);
	}

	sysfs_remove_group(&dctrl->dev->kobj, &lcd_debug_attr_group);

	return ret;
}
