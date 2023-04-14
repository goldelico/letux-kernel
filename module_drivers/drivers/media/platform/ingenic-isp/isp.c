#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>



#include "isp-drv.h"
#include "isp-regs.h"
#include "tiziano_core.h"

int isp_clk = 300000000;
module_param(isp_clk, int, S_IRUGO);
MODULE_PARM_DESC(isp_clk, "isp core clock");

DEFINE_SPINLOCK(isp_cpm_lock);

static inline unsigned int isp_reg_readl(struct isp_device *isp, unsigned int reg)
{
	return readl(isp->iobase + reg);
}

static inline void isp_reg_writel(struct isp_device *isp, unsigned int reg, unsigned int val)
{
	writel(val, isp->iobase + reg);
}



/* interface used by isp-core.*/
int system_reg_write(void *isp, unsigned int reg, unsigned int value)
{
	isp_reg_writel(isp, reg, value);
	return 0;
}

unsigned int  system_reg_read(void *isp, unsigned int reg)
{
	return isp_reg_readl(isp, reg);
}

int system_irq_func_set(void *hdl, int irq, void *func, void *data)
{
	struct isp_device *core = hdl;

	core->irq_func_cb[irq] = func;
	core->irq_func_data[irq] = data;
	return 0;
}

static int isp_cpm_stop(struct isp_device *isp)
{
	int timeout = 0xffffff;
	unsigned int value = 0;


	spin_lock(&isp_cpm_lock);
	/*stop request*/
	value = readl(isp->cpm_reset);
	value |= 1 << isp->bit_stp;
	writel(value, isp->cpm_reset);

	/*stop ack*/
	while(!(readl(isp->cpm_reset) & (1 << isp->bit_ack)) && --timeout);

	if(timeout == 0) {
		spin_unlock(&isp_cpm_lock);
		dev_err(isp->dev, "isp wait stop timeout\n");
		return -ETIMEDOUT;
	}

	spin_unlock(&isp_cpm_lock);
	return 0;
}

static int isp_cpm_reset(struct isp_device *isp)
{
	int timeout = 0xffffff;
	unsigned int value = 0;
	int ret = 0;


	spin_lock(&isp_cpm_lock);
	/*stop request*/
	value = readl(isp->cpm_reset);
	value |= 1 << isp->bit_stp;
	writel(value, isp->cpm_reset);

	/*stop ack*/
	while(!(readl(isp->cpm_reset) & (1 << isp->bit_ack)) && --timeout);

	if(timeout == 0) {
		dev_err(isp->dev, "isp wait stop timeout\n");
		ret = -ETIMEDOUT;
		goto reset_timeout;
	}

	/* activate reset */
	value = readl(isp->cpm_reset);
	value &= ~(1 << isp->bit_stp);
	value |= 1 << isp->bit_sr;
	writel(value, isp->cpm_reset);

	/* deactive reset */
	value = readl(isp->cpm_reset);
	value &= ~(1 << isp->bit_sr);
	writel(value, isp->cpm_reset);

	spin_unlock(&isp_cpm_lock);
	return 0;

reset_timeout:
	spin_unlock(&isp_cpm_lock);
	return ret;
}

static int ingenic_isp_parse_dt(struct isp_device * isp)
{
	struct device *dev = isp->dev;
	unsigned int cpm_reset = 0;
	int ret = 0;

	of_property_read_u32(dev->of_node, "ingenic,cpm_reset", &cpm_reset);
	of_property_read_u32(dev->of_node, "ingenic,bit_sr", &isp->bit_sr);
	of_property_read_u32(dev->of_node, "ingenic,bit_stp", &isp->bit_stp);
	of_property_read_u32(dev->of_node, "ingenic,bit_ack", &isp->bit_ack);

	isp->cpm_reset = (void __iomem *)cpm_reset;

	return ret;
}


/* interface should be removed. */
static int isp_subdev_init(struct v4l2_subdev *sd, u32 val)
{

	return 0;
}

/* interface should be removed. */
static int isp_subdev_reset(struct v4l2_subdev *sd, u32 val)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	int ret = 0;

	ret = isp_cpm_reset(isp);

	return ret;
}

static const struct v4l2_subdev_core_ops isp_subdev_core_ops = {
	.init	= isp_subdev_init,
	.reset = isp_subdev_reset,
        .log_status = v4l2_ctrl_subdev_log_status,
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static int isp_subdev_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt;
	int ret = 0;

	remote = media_entity_remote_pad(&isp->pads[ISP_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	/*获取源当前格式，复制到输出格式.*/
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if(ret < 0) {
		dev_err(isp->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	/*本身ISP时没有格式区别的， 这里必须从VIC获取，即ISP的SINK_PAD.*/
	if(format->pad == ISP_PAD_SOURCE) {
		memcpy(&format->format, &remote_subdev_fmt.format, sizeof(format->format));
	} else {
		dev_warn(isp->dev, "ISP_PAD_SOURCE should be set!\n");
	}

	isp->formats[ISP_PAD_SINK] = isp->formats[ISP_PAD_SOURCE] = *format;
	isp->sensor_info = *(unsigned int *)format->format.reserved;

	//printk("----%s, %d, format->pad: %d\n", __func__, __LINE__, format->pad);
	return 0;
}

static int isp_subdev_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	int ret = 0;

	remote = media_entity_remote_pad(&isp->pads[ISP_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	ret = v4l2_subdev_call(remote_sd, pad, set_fmt, NULL, format);
	if(ret < 0) {
		dev_dbg(isp->dev, "Failed to set_fmt from remote pad\n");
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops isp_subdev_pad_ops = {
        .set_fmt                = isp_subdev_set_fmt,
        .get_fmt                = isp_subdev_get_fmt,
	/*
        .init_cfg               = isp_subdev_init_cfg,
        .enum_mbus_code         = isp_subdev_enum_mbus_code,
        .enum_frame_size        = isp_subdev_enum_frame_size,
	*/
};

enum isp_bayer_type {
	TIZIANO_BAYER_TYPE_RGGB = 0,
	TIZIANO_BAYER_TYPE_BGGR,
	TIZIANO_BAYER_TYPE_GRBG,
	TIZIANO_BAYER_TYPE_GBRG
};


static int isp_fw_process(void *data)
{
        struct isp_device *isp = (struct isp_device *)data;

        while(!kthread_should_stop()){
                tisp_fw_process(&isp->core);
        }
        return 0;
}


static int isp_load_params(struct isp_device *isp)
{
	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];
	struct ispcam_device *ispcam = isp->ispcam;
	unsigned int ret = 0;
	struct file *file = NULL;
	struct inode *inode = NULL;
	mm_segment_t old_fs;
	loff_t fsize = 0;
	loff_t *pos = NULL;

	char file_name[64] = {0};

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	snprintf(file_name, sizeof(file_name), "/etc/sensor/%s-%s-%d-%d.bin", ispcam->isd[0].sd->dev->driver->name, ispcam->chip_name, input_fmt->format.width, input_fmt->format.height);
	file = filp_open(file_name, O_RDONLY, 0);
	if (file < 0 || IS_ERR(file)) {
		dev_dbg(isp->dev, "ISP: open %s file for isp calibrate read failed\n", file_name);

		snprintf(file_name, sizeof(file_name), "/etc/sensor/%s-%s.bin", ispcam->isd[0].sd->dev->driver->name, ispcam->chip_name);
		file = filp_open(file_name, O_RDONLY, 0);
		if (file < 0 || IS_ERR(file)) {
			dev_dbg(isp->dev, "ISP: open %s file for isp calibrate read failed\n", file_name);
			ret = -1;
			isp->core.tuned_params = NULL;
			goto failed_open_file;
		}
	}

	/* read file */
	inode = file->f_inode;
	fsize = inode->i_size;
	pos = &(file->f_pos);

	if(isp->core.tuned_params == NULL){
		isp->core.tuned_params = kzalloc(fsize, GFP_KERNEL);
		if(isp->core.tuned_params == NULL){
			dev_err(isp->dev, "%s[%d]: Failed to alloc %lld KB buffer!\n",__func__,__LINE__, fsize >> 10);
			ret = -1;
			goto failed_malloc_data;
		}
		isp->core.tuned_params_size = fsize;
	}

	vfs_read(file, isp->core.tuned_params, fsize, pos);

failed_malloc_data:
	filp_close(file, NULL);
	set_fs(old_fs);
failed_open_file:
	return ret;
}

static int isp_release_params(struct isp_device *isp)
{
	if(isp->core.tuned_params) {
		kfree(isp->core.tuned_params);
		isp->core.tuned_params = NULL;
		isp->core.tuned_params_size = 0;
	}

	return 0;
}

static int isp_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct ispcam_device *ispcam = isp->ispcam;
	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];

	tisp_init_param_t iparam;

	if(enable) {

		if(isp->enabled++ > 0) {
			return 0;
		}

		iparam.width = input_fmt->format.width;
		iparam.height = input_fmt->format.height;


		switch(input_fmt->format.code) {
			case MEDIA_BUS_FMT_SBGGR8_1X8:
			case MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8:
			case MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE:
			case MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE:
			case MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE:
			case MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE:
			case MEDIA_BUS_FMT_SBGGR10_1X10:
			case MEDIA_BUS_FMT_SBGGR12_1X12:
				//				printk("BGGR\n");
				iparam.bayer = 2;
				break;
			case MEDIA_BUS_FMT_SGBRG8_1X8:
			case MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8:
			case MEDIA_BUS_FMT_SGBRG10_1X10:
			case MEDIA_BUS_FMT_SGBRG12_1X12:
				//				printk("GBRG\n");
				iparam.bayer = 3;
				break;
			case MEDIA_BUS_FMT_SGRBG8_1X8:
			case MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8:
			case MEDIA_BUS_FMT_SGRBG10_1X10:
			case MEDIA_BUS_FMT_SGRBG12_1X12:
				iparam.bayer = 1;
				//				printk("GRBG\n");
				break;
			case MEDIA_BUS_FMT_SRGGB8_1X8:
			case MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8:
			case MEDIA_BUS_FMT_SRGGB10_1X10:
			case MEDIA_BUS_FMT_SRGGB12_1X12:
				iparam.bayer = 0;
				//				printk("RGGB\n");
				break;
			default:
				dev_err(isp->dev, "%s[%d] the format(0x%08x) of input couldn't be handled!\n",
						__func__,__LINE__, input_fmt->format.code);
				return -EINVAL;
				break;
		}


		strncpy(iparam.sensor, ispcam->isd[0].sd->dev->driver->name, sizeof(iparam.sensor));
		iparam.sensor_info.fps = isp->sensor_info->fps;
		iparam.sensor_info.total_width = isp->sensor_info->total_width;
		iparam.sensor_info.total_height = isp->sensor_info->total_height;


		isp_load_params(isp);
		tisp_core_init(&isp->core, &iparam, isp);
		isp->core_inited = 1;
		isp_core_tuning_param_sync(&isp->core);

		isp_release_params(isp);

		/*For event engine.*/
		isp->process_thread = kthread_run(isp_fw_process, isp, "isp_fw_process");
		if(IS_ERR_OR_NULL(isp->process_thread)){
			dev_err(isp->dev, "%s[%d] kthread_run was failed!\n",__func__,__LINE__);
			return -EINVAL;
		}


		isp_reg_writel(isp, TOP_CTRL_ADDR_INT_EN, 0x283f03ff);
#if 0
		iparam.sensor_info.max_again = core->vin.attr->max_again;       //the format is .16
		iparam.sensor_info.max_dgain = core->vin.attr->max_dgain;       //the format is .16
		iparam.sensor_info.again = core->vin.attr->again;
		iparam.sensor_info.dgain = core->vin.attr->dgain;
		iparam.sensor_info.fps = core->vin.fps;
		iparam.sensor_info.min_integration_time = core->vin.attr->min_integration_time;
		iparam.sensor_info.min_integration_time_native = core->vin.attr->min_integration_time_native;
		iparam.sensor_info.max_integration_time_native = core->vin.attr->max_integration_time_native;
		iparam.sensor_info.integration_time_limit = core->vin.attr->integration_time_limit;
		iparam.sensor_info.integration_time = core->vin.attr->integration_time;
		iparam.sensor_info.total_width = core->vin.attr->total_width;
		iparam.sensor_info.total_height = core->vin.attr->total_height;
		iparam.sensor_info.max_integration_time = core->vin.attr->max_integration_time;
		iparam.sensor_info.integration_time_apply_delay = core->vin.attr->integration_time_apply_delay;
		iparam.sensor_info.again_apply_delay = core->vin.attr->again_apply_delay;
		iparam.sensor_info.dgain_apply_delay = core->vin.attr->dgain_apply_delay;
		iparam.sensor_info.one_line_expr_in_us = core->vin.attr->one_line_expr_in_us;
#endif

	} else {

		if(--isp->enabled > 0) {
			return 0;
		}
		/*disable irq.*/
		isp_reg_writel(isp, TOP_CTRL_ADDR_INT_EN, 0);
		/* isp tirger */
		isp_reg_writel(isp, INPUT_CTRL_ADDR_IP_TRIG, 0x0);

		kthread_stop(isp->process_thread);

		tisp_core_deinit(&isp->core);
		isp->core_inited = 0;

	}

	return 0;
}


static const struct v4l2_subdev_video_ops isp_subdev_video_ops = {
        .s_stream = isp_subdev_s_stream,
};


static const struct v4l2_subdev_ops isp_subdev_ops = {
        .core = &isp_subdev_core_ops,
        .pad = &isp_subdev_pad_ops,
        .video = &isp_subdev_video_ops,
};
static ssize_t
dump_isp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "TOP_CTRL_ADDR_VERSION            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_VERSION            ));
	p += sprintf(p, "TOP_CTRL_ADDR_FM_SIZE            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_FM_SIZE            ));
	p += sprintf(p, "TOP_CTRL_ADDR_BAYER_TYPE         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_BAYER_TYPE         ));
	p += sprintf(p, "TOP_CTRL_ADDR_BYPASS_CON         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_BYPASS_CON         ));
	p += sprintf(p, "TOP_CTRL_ADDR_TOP_CON            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TOP_CON            ));
	p += sprintf(p, "TOP_CTRL_ADDR_TOP_STATE          :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TOP_STATE          ));
	p += sprintf(p, "TOP_CTRL_ADDR_LINE_SPACE         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_LINE_SPACE         ));
	p += sprintf(p, "TOP_CTRL_ADDR_REG_CON            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_REG_CON            ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_TRIG        :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_TRIG        ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_CON         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_CON         ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_ADDR        :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_ADDR        ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_STATE       :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_STATE       ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_APB_WR_DATA :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_APB_WR_DATA ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RC_APB_WR_ADDR :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RC_APB_WR_ADDR ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RD_CON         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RD_CON         ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_WR_CON         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_WR_CON         ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_FR_WR_CON      :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_FR_WR_CON      ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_STA_WR_CON     :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_STA_WR_CON     ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_RD_DEBUG       :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_RD_DEBUG       ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_WR_DEBUG       :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_WR_DEBUG       ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_FR_WR_DEBUG    :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_FR_WR_DEBUG    ));
	p += sprintf(p, "TOP_CTRL_ADDR_DMA_STA_WR_DEBUG   :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_DMA_STA_WR_DEBUG   ));
	p += sprintf(p, "TOP_CTRL_ADDR_INT_EN             :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_INT_EN             ));
	p += sprintf(p, "TOP_CTRL_ADDR_INT_REG            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_INT_REG            ));
	p += sprintf(p, "TOP_CTRL_ADDR_INT_CLR            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_INT_CLR            ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_FREERUN         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_FREERUN         ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_CON             :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_CON             ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_SIZE            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_SIZE            ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_FONT            :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_FONT            ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_FLICK           :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_FLICK           ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_CS_TYPE         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_CS_TYPE         ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_CS_FCLO         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_CS_FCLO         ));
	p += sprintf(p, "TOP_CTRL_ADDR_TP_CS_BCLO         :0x%08x\n", isp_reg_readl(isp, TOP_CTRL_ADDR_TP_CS_BCLO         ));

	return p - buf;
}


static DEVICE_ATTR(dump_isp, S_IRUGO|S_IWUSR, dump_isp, NULL);

static struct attribute *isp_debug_attrs[] = {
        &dev_attr_dump_isp.attr,
	NULL,
};

static struct attribute_group isp_debug_attr_group = {
        .name   = "debug",
        .attrs  = isp_debug_attrs,
};


static int isp_comp_bind(struct device *comp, struct device *master,
                              void *master_data)
{
	struct isp_device *isp = dev_get_drvdata(comp);
	struct ispcam_device *ispcam = (struct ispcam_device *)master_data;
	struct v4l2_device *v4l2_dev = &ispcam->v4l2_dev;
	struct v4l2_subdev *sd = &isp->sd;
	int ret = 0;

	//dev_info(comp, "----dev_name(comp): %s----%s, %d \n", dev_name(comp), __func__, __LINE__);

	/* link subdev to master.*/
	isp->ispcam = (void *)ispcam;
	ispcam->isp = isp;

	/*1. register supported subdev ctrls.*/


	/*2. init v4l2_subdev*/

	v4l2_subdev_init(sd, &isp_subdev_ops);

	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(comp), sizeof(sd->name));
	v4l2_set_subdevdata(sd, isp);


	/* init isp pads. */
	isp->pads = kzalloc(sizeof(struct media_pad) * ISP_NUM_PADS, GFP_KERNEL);
	if(!isp->pads) {
		ret = -ENOMEM;
		goto err_alloc_pads;
	}
	isp->pads[0].index = ISP_PAD_SINK;
	isp->pads[0].flags = MEDIA_PAD_FL_SINK;
	isp->pads[1].index = ISP_PAD_SOURCE;
	isp->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, ISP_NUM_PADS, isp->pads);

	/*3. register v4l2_subdev*/
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_COMPOSER;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if(ret < 0) {
		dev_err(comp, "Failed to register v4l2_subdev for isp\n");
		goto err_subdev_register;
	}

	return 0;
err_subdev_register:
err_alloc_pads:
	return ret;
}


static void isp_comp_unbind(struct device *comp, struct device *master,
                                 void *master_data)
{
        struct isp_device *isp = dev_get_drvdata(comp);

	dev_info(comp, "---TODO:--%p---%s, %d \n", isp,  __func__, __LINE__);

}

static const struct component_ops isp_comp_ops = {
        .bind = isp_comp_bind,
        .unbind = isp_comp_unbind,
};

static irqreturn_t isp_irq_handler(int irq, void *data)
{
	struct isp_device *isp = (struct isp_device *)data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct mscaler_device *mscaler = ispcam->mscaler;
	struct v4l2_subdev *sd = &mscaler->sd;
	bool handled = 0;
	unsigned int status;
	int ret = 0;
	int i = 0;

	/*read irq_flags*/
	status = isp_reg_readl(isp, TOP_CTRL_ADDR_INT_REG);


	/*1. process irq by subdev.*/
	ret = v4l2_subdev_call(sd, core, interrupt_service_routine, status, &handled);
	if(ret < 0) {

	}

	/*2. isp-core irq callbacks */
	for(i = 0; i < 32; i++) {
		if(status & (1 << i) && isp->irq_func_cb[i]) {
			ret = isp->irq_func_cb[i](isp->irq_func_data[i]);
			if(ret < 0) {

			}
		}
	}
	/*clear irq_flags*/
	isp_reg_writel(isp, TOP_CTRL_ADDR_INT_CLR, status);


	return IRQ_HANDLED;
}

static int ingenic_isp_probe(struct platform_device *pdev)
{

	struct isp_device *isp = NULL;
	int ret = 0;
	struct resource *regs = NULL;

	isp = kzalloc(sizeof(struct isp_device), GFP_KERNEL);
	if(!isp) {
		pr_err("Failed to alloc isp dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	isp->dev = &pdev->dev;
	platform_set_drvdata(pdev, isp);


	ingenic_isp_parse_dt(isp);

	isp->irq = platform_get_irq(pdev, 0);
	if(isp->irq < 0) {
		dev_warn(&pdev->dev, "No ISP IRQ specified\n");
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!regs) {
		dev_err(&pdev->dev, "No iomem resource!\n");
		goto err_get_resource;
	}

	isp->iobase = devm_ioremap_resource(&pdev->dev, regs);
	if(!isp->iobase) {
		goto err_ioremap;
	}

	ret = devm_request_irq(isp->dev, isp->irq, isp_irq_handler, 0,
			dev_name(isp->dev), isp);
	if(ret) {
		dev_err(isp->dev, "request irq failed!\n");
		goto err_request_irq;
	}

	isp->div_clk = of_clk_get(isp->dev->of_node, 0);
	if(!isp->div_clk) {
		dev_err(isp->dev, "failed to get isp div_clk\n");
		goto err_div_clk;
	}

	clk_set_rate(isp->div_clk, isp_clk);

	clk_prepare_enable(isp->div_clk);

	isp->gate_clk = of_clk_get(isp->dev->of_node, 1);
	if(!isp->gate_clk) {
		dev_err(isp->dev, "failed to get isp gate_clk\n");
		goto err_gate_clk;
	}

	clk_prepare_enable(isp->gate_clk);

	isp->power_clk = of_clk_get(isp->dev->of_node, 2);
	if(!isp->power_clk) {
		dev_err(isp->dev, "failed to get isp power clk!\n");
		goto err_power_clk;
	}
	clk_prepare_enable(isp->power_clk);

	ret = sysfs_create_group(&isp->dev->kobj, &isp_debug_attr_group);
	if (ret) {
		dev_err(isp->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_sys_group;
	}


	ret = component_add(isp->dev, &isp_comp_ops);
	if(ret < 0) {
		dev_err(isp->dev, "Failed to add component isp!\n");
	}

	return 0;
err_sys_group:
err_power_clk:
err_gate_clk:
err_div_clk:
err_request_irq:
err_ioremap:
err_get_resource:
	return ret;
}



static int ingenic_isp_remove(struct platform_device *pdev)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(isp->power_clk);
	clk_disable_unprepare(isp->gate_clk);
	clk_disable_unprepare(isp->div_clk);
	return 0;
}



static const struct of_device_id ingenic_isp_dt_match[] = {
        { .compatible = "ingenic,x2000-isp" },
        { .compatible = "ingenic,m300-isp" },
        { }
};

MODULE_DEVICE_TABLE(of, ingenic_isp_dt_match);

static int __maybe_unused ingenic_isp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	if(isp->enabled){
		dev_err(isp->dev, "faild to suspend, isp is streaming on\n");
		return -EBUSY;
	}

	isp_cpm_stop(isp);
	clk_disable_unprepare(isp->power_clk);
	clk_disable_unprepare(isp->gate_clk);
	clk_disable_unprepare(isp->div_clk);

        return 0;
}

static int __maybe_unused ingenic_isp_resume(struct platform_device *pdev)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	clk_prepare_enable(isp->div_clk);
	clk_prepare_enable(isp->gate_clk);
	clk_prepare_enable(isp->power_clk);
	isp_cpm_reset(isp);

        return 0;
}

static struct platform_driver ingenic_isp_driver = {
        .probe = ingenic_isp_probe,
        .remove = ingenic_isp_remove,
	.suspend = ingenic_isp_suspend,
	.resume = ingenic_isp_resume,
        .driver = {
                .name = "ingenic-isp",
                .of_match_table = ingenic_isp_dt_match,
        },
};

module_platform_driver(ingenic_isp_driver);

MODULE_ALIAS("platform:ingenic-isp");
MODULE_DESCRIPTION("ingenic isp subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");

