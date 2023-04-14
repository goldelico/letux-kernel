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
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/ingenic_video_nr.h>

#include "isp-drv.h"
#include "vic-regs.h"

#define VIC_STOP_BUFFER_COUNT 2
#define VIC_RESTART_BUFFER_COUNT 3

int count_period = 256;
module_param(count_period, int, S_IRUGO);
MODULE_PARM_DESC(count_period, "vic counter period");

int vic_stop_counter = 0;
module_param(vic_stop_counter, int, S_IRUGO);
MODULE_PARM_DESC(vic_stop_counter, "vic0 bypass mode stop counter");

int dma_debug_en = 0;
module_param(dma_debug_en, int, 0664);
MODULE_PARM_DESC(dma_debug_en, "vic dma debug mode enable");

struct isp_video_format vic_mipi_formats[] = {
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
	/* dvp: Y8_1X8 */
	{
		.name     = "Y8, GREY",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_RAW,
	},
	/* RAW8 */
	{
		.name     = "RAW8, 8  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR8,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG8,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG8,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW8, 8  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB8,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	/* RAW10 */
	{
		.name     = "RAW10, 10  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR10,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG10,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG10,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW10, 10  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB10,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	/* RAW12 */
	{
		.name     = "RAW12, 12  BGBG.. GRGR..",
		.fourcc   = V4L2_PIX_FMT_SBGGR12,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  GBGB.. RGRG..",
		.fourcc   = V4L2_PIX_FMT_SGBRG12,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  GRGR.. BGBG..",
		.fourcc   = V4L2_PIX_FMT_SGRBG12,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "RAW12, 12  RGRG.. GBGB..",
		.fourcc   = V4L2_PIX_FMT_SRGGB12,
		.depth    = {16},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
};

struct isp_video_format vic_dvp_formats = {

};


static inline void vic_reg_writel(struct vic_device *vic, unsigned int reg, unsigned int val)
{
	writel(val, vic->iobase + reg);
}

static inline unsigned int vic_reg_readl(struct vic_device *vic, unsigned int reg)
{
	return readl(vic->iobase + reg);
}


static int ingenic_vic_parse_dt(struct vic_device * vic)
{
	struct device *dev = vic->dev;
	int ret = 0;

	return ret;
}

unsigned int mux_out;
static int vic_subdev_core_init(struct v4l2_subdev *sd, u32 ispcam_enabled)
{
	struct vic_device *vic = v4l2_get_subdevdata(sd);
	if((ispcam_enabled == 0 && vic->iobase == 0xb3380000) || (ispcam_enabled == 1 && vic->iobase == 0xb3390000))
		mux_out = 0;
	else
		mux_out = INPUT_MXU_OUT;
	return 0;
}

static const struct v4l2_subdev_core_ops vic_subdev_core_ops = {
		.init = vic_subdev_core_init,
        .log_status = v4l2_ctrl_subdev_log_status,
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static int vic_subdev_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct vic_device *vic = v4l2_get_subdevdata(sd);
	struct isp_async_device *isd = &vic->ispcam->isd[0];	/*TODO: 默认支持一个asd.*/
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt;
	int ret = 0;

	remote = media_entity_remote_pad(&vic->pads[VIC_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	/*获取源当前格式，复制到输出格式.*/
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if(ret < 0) {
		dev_err(vic->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	memcpy(&format->format, &remote_subdev_fmt.format, sizeof(format->format));

	vic->formats[VIC_PAD_SINK] = vic->formats[VIC_PAD_SOURCE] = *format;

	//printk("----%s, %d, format->pad: %d\n", __func__, __LINE__, format->pad);
	return 0;
}

static int vic_subdev_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct vic_device *vic = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt;
	int ret = 0;

	dev_dbg(vic->dev, "%s, %d, width: %d, height: %d, mbus_code: %x\n", __func__, __LINE__, format->format.width, format->format.height, format->format.code);

	vic->formats[VIC_PAD_SOURCE] = *format;

	remote = media_entity_remote_pad(&vic->pads[VIC_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	ret = v4l2_subdev_call(remote_sd, pad, set_fmt, NULL, format);
	if(ret < 0) {
		dev_dbg(vic->dev, "Failed to set_fmt from remote pad\n");
	}

	/*获取源当前格式，复制到输出格式.*/
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if(ret < 0) {
		dev_err(vic->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	memcpy(&vic->formats[VIC_PAD_SINK], &remote_subdev_fmt, sizeof(struct v4l2_subdev_format));
	vic->sensor_info = *(unsigned int *)remote_subdev_fmt.format.reserved;

	return 0;
}


static const struct v4l2_subdev_pad_ops vic_subdev_pad_ops = {
	.get_fmt		= vic_subdev_get_fmt,
	.set_fmt		= vic_subdev_set_fmt,
};


static int vic_cfg_stream_mipi(struct vic_device *vic)
{
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	struct v4l2_fwnode_bus_mipi_csi2 *mipi_csi2 = ispcam_get_bus_mipi_csi2(vic->ispcam);
	struct mipi_cfg *mipi_cfg = &vic->sensor_info->mipi_cfg;
	//unsigned int timeout = 10000;
	unsigned int mipi_sensor_ctl = 0;

	dev_dbg(vic->dev, "%s, %d, input_fmt->format.width: %d, intput_fmt->format.height: %d\n",
			__func__, __LINE__, input_fmt->format.width, input_fmt->format.height);

	if(mipi_cfg->mipi_mode == SENSOR_MIPI_SONY_MODE) {
		dev_info(vic->dev, "[ %s:%d ] sensor type is SONY_MIPI!\n", __func__, __LINE__);
		/*imx307 nomal*/
		vic_reg_writel(vic, VIC_IN_DVP, (0x0 << 31 | 0x0 <<24 | 0x1 << 17 | 0x0 << 11 | 0x0 << 4));
		vic_reg_writel(vic, VIC_CONTROL_CONTROL, 0x00100010);
	} else {
		dev_info(vic->dev, "[ %s:%d ] sensor type is OTHER_MIPI!\n", __func__, __LINE__);
		vic_reg_writel(vic, VIC_CONTROL_DELAY, (10 << 16) | (10));
	}

	vic_reg_writel(vic, VIC_INTF_TYPE, INTF_TYPE_MIPI);

	switch(input_fmt->format.code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			vic_reg_writel(vic, VIC_IN_CSI_FMT,MIPI_RAW8);//RAW8
			vic_reg_writel(vic, MIPI_ALL_WIDTH_4BYTE, (mipi_cfg->twidth * 8 % 32) ? (mipi_cfg->twidth * 8 / 32)+ 1 : (mipi_cfg->twidth * 8 / 32));
			break;
		case    MEDIA_BUS_FMT_SBGGR10_1X10:
		case    MEDIA_BUS_FMT_SGBRG10_1X10:
		case    MEDIA_BUS_FMT_SGRBG10_1X10:
		case    MEDIA_BUS_FMT_SRGGB10_1X10:
			vic_reg_writel(vic, VIC_IN_CSI_FMT,MIPI_RAW10);//RAW10
			vic_reg_writel(vic, MIPI_ALL_WIDTH_4BYTE, (mipi_cfg->twidth * 10 % 32) ? (mipi_cfg->twidth * 10 / 32)+ 1 : (mipi_cfg->twidth * 10 / 32));
			break;
		case    MEDIA_BUS_FMT_SBGGR12_1X12:
		case    MEDIA_BUS_FMT_SGBRG12_1X12:
		case    MEDIA_BUS_FMT_SGRBG12_1X12:
		case    MEDIA_BUS_FMT_SRGGB12_1X12:
			vic_reg_writel(vic, VIC_IN_CSI_FMT,MIPI_RAW12);//RAW12
			vic_reg_writel(vic, MIPI_ALL_WIDTH_4BYTE, (mipi_cfg->twidth * 12 % 32) ? (mipi_cfg->twidth * 12 / 32)+ 1 : (mipi_cfg->twidth * 12 / 32));
			break;
		case MEDIA_BUS_FMT_Y8_1X8 :
			vic_reg_writel(vic, VIC_IN_CSI_FMT, MIPI_YUV422);//YUV422
			vic_reg_writel(vic, MIPI_ALL_WIDTH_4BYTE, (mipi_cfg->twidth*2 * 8 % 32) ? (mipi_cfg->twidth*2 * 8 / 32)+ 1 : (mipi_cfg->twidth*2 * 8 / 32));
			break;
		case MEDIA_BUS_FMT_YUYV8_2X8 :
		case MEDIA_BUS_FMT_YVYU8_2X8 :
		case MEDIA_BUS_FMT_UYVY8_2X8 :
		case MEDIA_BUS_FMT_VYUY8_2X8 :
			vic_reg_writel(vic, VIC_IN_CSI_FMT, MIPI_YUV422);//YUV422
			vic_reg_writel(vic, MIPI_ALL_WIDTH_4BYTE, (mipi_cfg->twidth * 8 % 32) ? (mipi_cfg->twidth * 8 / 32)+ 1 : (mipi_cfg->twidth * 8 / 32));
			break;
		default:
			dev_err(vic->dev, "unsupported mbus fmt: %x\n", input_fmt->format.code);
			return -EINVAL;
	}

	vic_reg_writel(vic, VIC_CONTROL_CONTROL, mipi_cfg->sensor_mode << SENSOR_MODE | mipi_cfg->sensor_frame_mode << SENSOR_FRM_NUM);

	mipi_sensor_ctl = mipi_cfg->hcrop_diff_en << HCROP_DIFF_EN | mipi_cfg->mipi_vcomp_en << VCOMP_EN | mipi_cfg->mipi_hcomp_en << HCOMP_EN |mipi_cfg->line_sync_mode << LINE_SYNC_MODE | mipi_cfg->work_start_flag << WORK_START_FLAG | mipi_cfg->data_type_en << DATA_TYPE_EN | mipi_cfg->data_type_value << DATA_TYPE_VALUE | mipi_cfg->del_start << DEL_START | mipi_cfg->sensor_frame_mode << SENSOR_FRM_NUM | mipi_cfg->sensor_fid_mode << SENSOR_FID_MODE | mipi_cfg->sensor_mode << SENSOR_MODE;
	vic_reg_writel(vic, MIPI_SENSOR_CONTROL, mipi_sensor_ctl);

	vic_reg_writel(vic, MIPI_HCROP_CH0, (mipi_cfg->twidth << 16) | mipi_cfg->mipi_crop_start0x);
	vic_reg_writel(vic, MIPI_HCROP_CH1, mipi_cfg->mipi_crop_start1x);
	vic_reg_writel(vic, MIPI_HCROP_CH2, mipi_cfg->mipi_crop_start2x);
	vic_reg_writel(vic, MIPI_HCROP_CH3, mipi_cfg->mipi_crop_start3x);

	vic_reg_writel(vic, MIPI_VCROP_DEL01, mipi_cfg->mipi_crop_start1y << 16 | mipi_cfg->mipi_crop_start0y);
	vic_reg_writel(vic, MIPI_VCROP_DEL23, mipi_cfg->mipi_crop_start3y << 16 | mipi_cfg->mipi_crop_start2y);

	return 0;
}

static int vic_cfg_stream_dvp(struct vic_device *vic)
{
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	struct v4l2_fwnode_bus_parallel *parallel = ispcam_get_bus_parallel(vic->ispcam);
	struct dvp_cfg *dvp_cfg = &vic->sensor_info->dvp_cfg;
	unsigned int input_cfg = 0;

	vic->frame_capture_counter = 0; /* reset counter */
	vic->global_reset = 0;

	dev_dbg(vic->dev, "parallel->flags: 0x%x\n", parallel->flags);
	dev_dbg(vic->dev, "parallel->bus_width: %d\n", parallel->bus_width);
	dev_dbg(vic->dev, "parallel->data_shift: %d\n", parallel->data_shift);
	vic_reg_writel(vic, VIC_INTF_TYPE, INTF_TYPE_DVP);
	switch(input_fmt->format.code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			if(parallel->data_shift) {
				input_cfg = DVP_RAW8 | DVP_RAW_ALIG;
			} else {
				input_cfg = DVP_RAW8;
			}
			break;
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			if(parallel->data_shift) {
				input_cfg = DVP_RAW10 | DVP_RAW_ALIG;
			} else {
				input_cfg = DVP_RAW10;
			}
			break;
		case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			input_cfg = DVP_RAW12;
			break;
		case MEDIA_BUS_FMT_UYVY8_1_5X8 :
		case MEDIA_BUS_FMT_VYUY8_1_5X8 :
		case MEDIA_BUS_FMT_YUYV8_1_5X8 :
		case MEDIA_BUS_FMT_YVYU8_1_5X8 :
		case MEDIA_BUS_FMT_YUYV8_1X16 :
		case MEDIA_BUS_FMT_YUYV8_2X8 :
		case MEDIA_BUS_FMT_YVYU8_2X8 :
		case MEDIA_BUS_FMT_UYVY8_2X8 :
		case MEDIA_BUS_FMT_VYUY8_2X8 :
		case MEDIA_BUS_FMT_Y8_1X8 :
			input_cfg = DVP_YUV422_8BIT;
			break;

		default:
			dev_err(vic->dev, "unsupported input formats\n");
			return -EINVAL;
	}
	if(parallel->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW) {
		input_cfg |= HSYN_POLAR;
	}
	if(parallel->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW) {
		input_cfg |= VSYN_POLAR;
	}
	vic_reg_writel(vic, VIC_IN_DVP, input_cfg);
	/*dev_info(vic->dev, "VIC_IN_DVP	input_cfg=0x%x\n", input_cfg);
	if (input_fmt->format.width>2048 && input_fmt->format.height == 1) {
		vic_reg_writel(vic, VIC_IN_DVP, 0x6<<17 | 1<<1 | 1<<0);    //cisadc, 0x6<<17 | 1<<1 | 1<<0;
	}*/
	/*TODO: hblank and vblank*/

	if(input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_2X8)
		vic_reg_writel(vic, VIC_IN_HOR_PARA0, input_fmt->format.width * 2 + 0 /*hblank*/);
	else
		vic_reg_writel(vic, VIC_IN_HOR_PARA0, input_fmt->format.width + 0 /*hblank*/);
	// vc control
	vic_reg_writel(vic, VIC_CONTROL_DELAY, 0x10 << 16 | 0x10);
	vic_reg_writel(vic, VIC_CONTROL_DELEY_BLK, 0x10);
	vic_reg_writel(vic, VIC_CONTROL_LIMIT, 0);

	return 0;
}

static int vic_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vic_device *vic = v4l2_get_subdevdata(sd);
	struct isp_async_device *isd = &vic->ispcam->isd[0];	/*TODO: 默认支持一个asd.*/
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	struct isp_video_buffer *isp_buffer = NULL;
	struct isp_video_buffer *tmp = NULL;
	int ret = 0;
	unsigned long flags;
	int width;
	unsigned int regval;

	spin_lock_irqsave(&vic->lock, flags);
	/*hw configure.*/
	if(enable) {

		if(vic->enabled++ > 0) {
			goto finish;
		}

		/* set width */
		switch(input_fmt->format.code) {
		case MEDIA_BUS_FMT_Y8_1X8 :
			input_fmt->format.width /= 2; /*  */
			width = input_fmt->format.width;
			break;
			/* TODO: NV21/NV12 */
		default:
			width = input_fmt->format.width;
		}

#if 0
		/* enable timestamp .*/
		vic_reg_writel(vic, VIC_TS_COUNTER, count_period - 1);
		vic_reg_writel(vic, VIC_TS_DMA_OFFSET, 0);
		vic_reg_writel(vic, VIC_TS_MS_CH0_OFFSET, 0);
		vic_reg_writel(vic, VIC_TS_MS_CH1_OFFSET, 0);
		vic_reg_writel(vic, VIC_TS_MS_CH2_OFFSET, 0);
		vic_reg_writel(vic, VIC_TS_ENABLE, TS_COUNTER_EN |
				TS_VIC_DMA_EN | TS_MS_CH0_EN |
				TS_MS_CH1_EN | TS_MS_CH2_EN);
#endif

		/*confgure hw and start*/
		if(isd->bus_type == V4L2_MBUS_CSI2_DPHY) {
			ret = vic_cfg_stream_mipi(vic);
			mux_out |= INPUT_MXU_MIPI_EN;
		} else {
			ret = vic_cfg_stream_dvp(vic);
			mux_out |= INPUT_MXU_DVP_EN;
		}

		vic_reg_writel(vic, VIC_RESOLUTION, (width << 16) |
				input_fmt->format.height);

		if(ispvideo->bypass == 1) {
			if(dma_debug_en) {
				printk("please disable vic dma debug mode\ntry \"echo 0 > /sys/module/vic/parameters/dma_debug_en\"\n");
				goto finish;
			}
			/*DMA config*/
			regval = 0;
			/*TODO: fmts.*/
			switch(input_fmt->format.code) {
			case MEDIA_BUS_FMT_UYVY8_1_5X8 :
			case MEDIA_BUS_FMT_VYUY8_1_5X8 :
			case MEDIA_BUS_FMT_YUYV8_1_5X8 :
			case MEDIA_BUS_FMT_YVYU8_1_5X8 :
			case MEDIA_BUS_FMT_YUYV8_1X16 :
			case MEDIA_BUS_FMT_YUYV8_2X8 :
				regval |= 1<<8; /* Y1UY2V */
				regval |= 3<<0; /* mode 3: YUV422 */
				break;
			case MEDIA_BUS_FMT_YVYU8_2X8 :
				regval |= 0<<8; /* Y2VY1U */
				regval |= 3<<0; /* mode 3: YUV422 */
				break;
			case MEDIA_BUS_FMT_UYVY8_2X8 :
				regval |= 3<<8; /* UY1VY2 */
				regval |= 3<<0; /* mode 3: YUV422 */
				break;
			case MEDIA_BUS_FMT_VYUY8_2X8 :
				regval |= 2<<8; /* VY2UY1 */
				regval |= 3<<0; /* mode 3: YUV422 */
				break;
			case MEDIA_BUS_FMT_Y8_1X8 :
				regval |= 3<<0; /* mode 3: YUV422 */
				break;
				/* TODO: NV21/NV12 */
			case MEDIA_BUS_FMT_SBGGR8_1X8:
			case MEDIA_BUS_FMT_SGBRG8_1X8:
			case MEDIA_BUS_FMT_SGRBG8_1X8:
			case MEDIA_BUS_FMT_SRGGB8_1X8:
			case MEDIA_BUS_FMT_SBGGR10_1X10:
			case MEDIA_BUS_FMT_SGBRG10_1X10:
			case MEDIA_BUS_FMT_SGRBG10_1X10:
			case MEDIA_BUS_FMT_SRGGB10_1X10:
			case MEDIA_BUS_FMT_SBGGR12_1X12:
			case MEDIA_BUS_FMT_SGBRG12_1X12:
			case MEDIA_BUS_FMT_SGRBG12_1X12:
			case MEDIA_BUS_FMT_SRGGB12_1X12:
				regval |= 0;
				break;
			default:
				dev_info(vic->dev, "%s(), TODO: DMA config, input_fmt->format.code=0x%x\n",
					 __func__, input_fmt->format.code);
				return -EINVAL;
			}
			regval |= 1 << 31; /* enable dma */
			regval |= (ispvideo->max_buffer_num - 1) << 3;
			regval |= 8 << 16; /*max outout num*/
			vic_reg_writel(vic, VIC_DMA_CONFIG, regval);

			vic_reg_writel(vic, VIC_DMA_RESOLUTION, (width << 16) |
				       input_fmt->format.height);
			vic_reg_writel(vic, VIC_DMA_Y_STRID, width*2);
			vic_reg_writel(vic, VIC_DMA_UV_STRID, width*2);

			vic_reg_writel(vic, VIC_CONTROL_TIZIANO_ROUTE, 0);
			if(isd->bus_type == V4L2_MBUS_CSI2_DPHY)
				vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0x4440);
			else
				vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0x4210);
			vic_reg_writel(vic, VIC_INT_MASK, 0x0);
			vic_reg_writel(vic, VIC_INT_MASK2, 0x0);
			/*
			if(input_fmt->format.width > 2048) {
				if(0 && input_fmt->format.height > 1)
					vic_reg_writel(vic, VIC_CONTROL_LIMIT, (1 << 31)|((input_fmt->format.width / 8 * 3) << 16));
				else
					vic_reg_writel(vic, VIC_CONTROL_LIMIT, (1 << 31)|(512<<16)); // cisadc, height==1, width > 2048
			}
			*/
			vic->framenum = 0;

		} else {
			if(vic->sensor_info->wdr_en)
				vic_reg_writel(vic, VIC_CONTROL_TIZIANO_ROUTE, 0x4410);
			else
				vic_reg_writel(vic, VIC_CONTROL_TIZIANO_ROUTE, 0x4404);
			if(dma_debug_en)
				if(vic->sensor_info->wdr_en)
					vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0x4410);
				else{
					if(isd->bus_type == V4L2_MBUS_CSI2_DPHY)
						vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0x4440);
					else
						vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0x4210);
				}
			else
				vic_reg_writel(vic, VIC_CONTROL_DMA_ROUTE, 0);
			vic_reg_writel(vic, VIC_INT_MASK, 0);
			vic_reg_writel(vic, VIC_INT_MASK2, 0);

			vic_reg_writel(vic, VIC_CONTROL, GLB_RST);
		}

		vic_reg_writel(vic, VIC_CONTROL_INPUT_MUX_OUTROUTE, mux_out);
		if(vic->iobase == 0xb3390000 && *(volatile unsigned int *)0xb33801c0 == 0)
			*(volatile unsigned int *)0xb33801c0 = 0x02;

		/*
		vic_reg_writel(vic, VIC_CONTROL, REG_ENABLE);
		unsigned int timeout  = 0xfffff;
		while(vic_reg_readl(vic,VIC_CONTROL) && --timeout);
		if(!timeout) {
			dev_err(vic->dev, "wait vic init timeout!\n");
		}
		*/
		vic_reg_writel(vic, VIC_CONTROL, VIC_START);


	} else {
		/*reset flags*/
		vic->buffer_index = 0;
		vic->stop_flag = 0;
		vic_stop_counter = 0;

		if(--vic->enabled) {
			goto finish;
		}

		/* disable timestamp */
		vic_reg_writel(vic, VIC_TS_COUNTER, 0);
		vic_reg_writel(vic, VIC_TS_ENABLE, 0);

		if(ispvideo->bypass) {
			/*disable dma*/
			vic_reg_writel(vic, VIC_DMA_CONFIG, 0);
			/*disable contro limit*/
			if(input_fmt->format.width > 2048)
				vic_reg_writel(vic, VIC_CONTROL_LIMIT, 0);
		}

		/* keep reset*/
		vic_reg_writel(vic, VIC_CONTROL, GLB_RST);

		/* stop vic. */
		list_for_each_entry_safe(isp_buffer, tmp, &vic->dma_queued_list, list_entry) {
			struct vb2_buffer *vb2_buf = &isp_buffer->vb2.vb2_buf;
			vb2_buffer_done(vb2_buf, VB2_BUF_STATE_ERROR);
			list_del(&isp_buffer->list_entry);
			vic->buffer_count--;
		}

	}

finish:
	spin_unlock_irqrestore(&vic->lock, flags);

	return ret;
}


static const struct v4l2_subdev_video_ops vic_subdev_video_ops = {
        .s_stream = vic_subdev_s_stream,
};
static const struct v4l2_subdev_ops vic_subdev_ops = {
        .core = &vic_subdev_core_ops,
        .pad = &vic_subdev_pad_ops,
        .video = &vic_subdev_video_ops,
};

struct isp_video_format *vic_find_format(const u32 *pixelformat, const u32 *mbus_code, int index)
{
	int i;
	struct isp_video_format *fmt = NULL;

	for(i = 0; i < ARRAY_SIZE(vic_mipi_formats); i++) {
		fmt = &vic_mipi_formats[i];

		if(pixelformat && fmt->fourcc == *pixelformat) {
			return fmt;
		}
		if(mbus_code && fmt->mbus_code == *mbus_code) {
			return fmt;
		}

		if(index == i) {
			return fmt;
		}
	}

	return NULL;
}

static int vic_video_qbuf(struct isp_video_device *ispvideo, struct isp_video_buffer *isp_buffer)
{
	struct vic_device *vic = ispvideo->ispcam->vic;
	struct vb2_buffer *vb2_buf = NULL;
	struct isp_video_buffer *tmp = NULL;
	unsigned long flags = 0;
	dma_addr_t y_addr = 0;
	dma_addr_t uv_addr = 0;
	unsigned int regval;

	spin_lock_irqsave(&vic->lock, flags);

	vb2_buf = &isp_buffer->vb2.vb2_buf;

	list_add_tail(&isp_buffer->list_entry, &vic->dma_queued_list);
	vic->buffer_count++;


	/*Y*/
	y_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
	vic_reg_writel(vic, VIC_DMA_Y_CH0_BUF0 + vic->buffer_index * 4, y_addr);

	uv_addr = y_addr + isp_buffer->uv_offset;
	/*UV*/
	if(vb2_buf->num_planes == 2) {
		uv_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
	}
	vic_reg_writel(vic, VIC_DMA_UV_CH0_BUF0 + vic->buffer_index * 4, uv_addr);

	vic->buffer_index++;
	vic->buffer_index %= ispvideo->max_buffer_num;

	if(vic->buffer_count >= VIC_RESTART_BUFFER_COUNT && vic->stop_flag)
	{
		/*reset flags*/
		vic->stop_flag = 0;
		vic->buffer_index = 0;

		/*buffers in the list refill in register*/
		list_for_each_entry_safe(isp_buffer, tmp, &vic->dma_queued_list, list_entry) {

			vb2_buf = &isp_buffer->vb2.vb2_buf;
			/*Y*/
			y_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
			vic_reg_writel(vic, VIC_DMA_Y_CH0_BUF0 + vic->buffer_index * 4, y_addr);

			uv_addr = y_addr + isp_buffer->uv_offset;
			/*UV*/
			if(vb2_buf->num_planes == 2) {
				uv_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
			}
			vic_reg_writel(vic, VIC_DMA_UV_CH0_BUF0 + vic->buffer_index * 4, uv_addr);

			vic->buffer_index++;
			vic->buffer_index %= ispvideo->max_buffer_num;

		}
		/*restart*/
		regval = vic_reg_readl(vic, VIC_DMA_CONFIG);
		vic_reg_writel(vic, VIC_DMA_RESET, 1);
		vic_reg_writel(vic, VIC_DMA_RESET, 0);
		regval |= 1 << 31; /* enable dma */
		regval &= ~(0xf<<3); /* reset dma buffer num */
		regval |= (ispvideo->max_buffer_num - 1) << 3;
		vic_reg_writel(vic, VIC_DMA_CONFIG, regval);
		/* lgwang, re-start vic */
		if (vic->global_reset) {
			vic_reg_writel(vic, VIC_CONTROL, 0x1);
			vic->global_reset = 0;
		}
		goto done;
	}

done:
	spin_unlock_irqrestore(&vic->lock, flags);

	return 0;
}

static const struct isp_video_ops vic_video_ops = {
	.find_format 	= vic_find_format,
	.qbuf		= vic_video_qbuf,
};

static ssize_t
dump_vic(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vic_device *vic = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "VIC_CONTROL			:0x%08x\n", vic_reg_readl(vic, VIC_CONTROL));
	p += sprintf(p, "VIC_RESOLUTION                 :0x%08x\n", vic_reg_readl(vic, VIC_RESOLUTION));
	p += sprintf(p, "VIC_FRM_ECC                    :0x%08x\n", vic_reg_readl(vic, VIC_FRM_ECC));
	p += sprintf(p, "VIC_INTF_TYPE                  :0x%08x\n", vic_reg_readl(vic, VIC_INTF_TYPE));
	p += sprintf(p, "VIC_IN_DVP                     :0x%08x\n", vic_reg_readl(vic, VIC_IN_DVP));
	p += sprintf(p, "VIC_IN_CSI_FMT                 :0x%08x\n", vic_reg_readl(vic, VIC_IN_CSI_FMT));
	p += sprintf(p, "VIC_IN_HOR_PARA0               :0x%08x\n", vic_reg_readl(vic, VIC_IN_HOR_PARA0));
	p += sprintf(p, "VIC_IN_HOR_PARA1               :0x%08x\n", vic_reg_readl(vic, VIC_IN_HOR_PARA1));
	p += sprintf(p, "VIC_IN_VER_PARA0               :0x%08x\n", vic_reg_readl(vic, VIC_IN_VER_PARA0));
	p += sprintf(p, "VIC_IN_VER_PARA1               :0x%08x\n", vic_reg_readl(vic, VIC_IN_VER_PARA1));
	p += sprintf(p, "VIC_IN_VER_PARA2               :0x%08x\n", vic_reg_readl(vic, VIC_IN_VER_PARA2));
	p += sprintf(p, "VIC_IN_VER_PARA3               :0x%08x\n", vic_reg_readl(vic, VIC_IN_VER_PARA3));
	p += sprintf(p, "VIC_VLD_LINE_SAV               :0x%08x\n", vic_reg_readl(vic, VIC_VLD_LINE_SAV));
	p += sprintf(p, "VIC_VLD_LINE_EAV               :0x%08x\n", vic_reg_readl(vic, VIC_VLD_LINE_EAV));
	p += sprintf(p, "VIC_VLD_FRM_SAV                :0x%08x\n", vic_reg_readl(vic, VIC_VLD_FRM_SAV));
	p += sprintf(p, "VIC_VLD_FRM_EAV                :0x%08x\n", vic_reg_readl(vic, VIC_VLD_FRM_EAV));
	p += sprintf(p, "VIC_VC_CONTROL                 :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL));
	p += sprintf(p, "VIC_VC_CONTROL_CH0_PIX         :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH0_PIX));
	p += sprintf(p, "VIC_VC_CONTROL_CH1_PIX         :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH1_PIX));
	p += sprintf(p, "VIC_VC_CONTROL_CH2_PIX         :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH2_PIX));
	p += sprintf(p, "VIC_VC_CONTROL_CH3_PIX         :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH3_PIX));
	p += sprintf(p, "VIC_VC_CONTROL_CH0_LINE        :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH0_LINE));
	p += sprintf(p, "VIC_VC_CONTROL_CH1_LINE        :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH1_LINE));
	p += sprintf(p, "VIC_VC_CONTROL_CH2_LINE        :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH2_LINE));
	p += sprintf(p, "VIC_VC_CONTROL_CH3_LINE        :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_CH3_LINE));
	p += sprintf(p, "VIC_VC_CONTROL_FIFO_USE        :0x%08x\n", vic_reg_readl(vic, VIC_VC_CONTROL_FIFO_USE));
	p += sprintf(p, "MIPI_ALL_WIDTH_4BYTE           :0x%08x\n", vic_reg_readl(vic, MIPI_ALL_WIDTH_4BYTE));
	p += sprintf(p, "MIPI_VCROP_DEL01               :0x%08x\n", vic_reg_readl(vic, MIPI_VCROP_DEL01));
	p += sprintf(p, "MIPI_SENSOR_CONTROL            :0x%08x\n", vic_reg_readl(vic, MIPI_SENSOR_CONTROL));
	p += sprintf(p, "MIPI_HCROP_CH0                 :0x%08x\n", vic_reg_readl(vic, MIPI_HCROP_CH0));
	p += sprintf(p, "VIC_CONTROL_LIMIT              :0x%08x\n", vic_reg_readl(vic, VIC_CONTROL_LIMIT));
	p += sprintf(p, "VIC_CONTROL_DELAY              :0x%08x\n", vic_reg_readl(vic, VIC_CONTROL_DELAY));
	p += sprintf(p, "VIC_CONTROL_TIZIANO_ROUTE      :0x%08x\n", vic_reg_readl(vic, VIC_CONTROL_TIZIANO_ROUTE));
	p += sprintf(p, "VIC_CONTROL_DMA_ROUTE          :0x%08x\n", vic_reg_readl(vic, VIC_CONTROL_DMA_ROUTE));
	p += sprintf(p, "VIC_INT_STA                    :0x%08x\n", vic_reg_readl(vic, VIC_INT_STA));
	p += sprintf(p, "VIC_INT_MASK                   :0x%08x\n", vic_reg_readl(vic, VIC_INT_MASK));
	p += sprintf(p, "VIC_INT_CLR                    :0x%08x\n", vic_reg_readl(vic, VIC_INT_CLR));
	p += sprintf(p, "VIC_DMA_CONFIG        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_CONFIG));
	p += sprintf(p, "VIC_DMA_RESOLUTION    :0x%08x\n", vic_reg_readl(vic, VIC_DMA_RESOLUTION));
	p += sprintf(p, "VIC_DMA_RESET         :0x%08x\n", vic_reg_readl(vic, VIC_DMA_RESET));
	p += sprintf(p, "VIC_DMA_Y_STRID       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_STRID));
	p += sprintf(p, "VIC_DMA_Y_CH0_BUF0        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_CH0_BUF0));
	p += sprintf(p, "VIC_DMA_Y_CH0_BUF1        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_CH0_BUF1));
	p += sprintf(p, "VIC_DMA_Y_CH0_BUF2        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_CH0_BUF2));
	p += sprintf(p, "VIC_DMA_Y_CH0_BUF3        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_CH0_BUF3));
	p += sprintf(p, "VIC_DMA_Y_CH0_BUF4        :0x%08x\n", vic_reg_readl(vic, VIC_DMA_Y_CH0_BUF4));
	p += sprintf(p, "VIC_DMA_UV_STRID      :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_STRID));
	p += sprintf(p, "VIC_DMA_UV_CH0_BUF0       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_CH0_BUF0));
	p += sprintf(p, "VIC_DMA_UV_CH0_BUF1       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_CH0_BUF1));
	p += sprintf(p, "VIC_DMA_UV_CH0_BUF2       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_CH0_BUF2));
	p += sprintf(p, "VIC_DMA_UV_CH0_BUF3       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_CH0_BUF3));
	p += sprintf(p, "VIC_DMA_UV_CH0_BUF4       :0x%08x\n", vic_reg_readl(vic, VIC_DMA_UV_CH0_BUF4));



	return p - buf;
}

ssize_t vic_dma_debug(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct vic_device *vic = dev_get_drvdata(dev);
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int imagesize = 0;
	unsigned int lineoffset = 0;
	unsigned int num = 0;
	int loop = 100;
	struct file *fd = NULL;
	mm_segment_t old_fs;
	loff_t *pos;
	int ret = 0;

	if (!strncmp(buf, "snapraw", sizeof("snapraw")-1)) {
		if(!dma_debug_en) {
			printk("please enable vic dma debug mode\ntry \"echo 1 > /sys/module/vic/parameters/dma_debug_en\"\n");
			return -EINVAL;
		}

		lineoffset = input_fmt->format.width * 2;
		imagesize = lineoffset * input_fmt->format.height;
		dev_info(vic->dev, "width is %d,height is %d,imagesize is %d\n",input_fmt->format.width,input_fmt->format.height,imagesize);
		if(input_fmt->format.width > VIC_DMA_OUTPUT_MAX_WIDTH){
			return -EINVAL;
		}
		if(vic->snap_paddr){
			return -EBUSY;
		}

		if(vic->sensor_info->wdr_en)
			num = 2;
		else
			num = 1;
		vic->snap_vaddr = kmalloc(imagesize * num, GFP_KERNEL);

		if(vic->snap_vaddr){
			vic->snap_paddr = virt_to_phys((void *)vic->snap_vaddr);
			vic_reg_writel(vic, VIC_DMA_RESET, 0x01);
			vic_reg_writel(vic, VIC_DMA_RESOLUTION, input_fmt->format.width << 16 | input_fmt->format.height);
			vic_reg_writel(vic, VIC_DMA_Y_STRID, lineoffset);
			vic_reg_writel(vic, VIC_DMA_Y_CH0_BUF0, vic->snap_paddr);
			if(vic->sensor_info->wdr_en)
				vic_reg_writel(vic, VIC_DMA_Y_CH1_BUF0, vic->snap_paddr + imagesize);
			vic_reg_writel(vic, VIC_DMA_CONFIG,
			(1	<< 31) // enable dma
			| (num	<< 16) // get > max
			| (0	<<  3) // use buffer num
			| (0	<<  0) // raw
			);
			while(loop){
				ret = wait_for_completion_interruptible(&vic->snap_comp);
				if (ret >= 0)
					break;
				loop--;
			}
			if(!loop){
				dev_err(vic->dev, "snapraw timeout!\n");
				goto exit;
			}

			vic->dma_complete = 0;
			/* save raw */
			fd = filp_open("/tmp/snap.raw", O_CREAT | O_WRONLY | O_TRUNC, 00766);
			if (fd < 0) {
				dev_err(vic->dev, "Failed to open /tmp/snap.raw\n");
				goto exit;
			}

			/* write file */
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			pos = &(fd->f_pos);
			vfs_write(fd, vic->snap_vaddr, imagesize, pos);
			if(vic->sensor_info->wdr_en)
				vfs_write(fd, vic->snap_vaddr + imagesize, imagesize, pos);
			filp_close(fd, NULL);
			set_fs(old_fs);
		}
	}

exit:
	kfree(vic->snap_vaddr);
	vic->snap_paddr = 0;

	return count;
}

static DEVICE_ATTR(dump_vic, S_IRUGO|S_IWUSR, dump_vic, NULL);
static DEVICE_ATTR(vic_dma_debug, S_IRUGO|S_IWUSR, NULL, vic_dma_debug);

static struct attribute *vic_debug_attrs[] = {
        &dev_attr_dump_vic.attr,
#ifdef CONFIG_VIC_DMA_ROUTE
	&dev_attr_vic_dma_debug.attr,
#endif
	NULL,
};

static struct attribute_group vic_debug_attr_group = {
        .name   = "debug",
        .attrs  = vic_debug_attrs,
};

int bypass_video_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device 	*ispcam = ispvideo->ispcam;
	struct isp_device 	*isp	= ispcam->isp;
	struct sensor_device *sensor = &isp->ispcam->sensor;
	int ret = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, a);
	if(ret < 0) {
		dev_err(isp->dev, "failed to get ID:%d!\n", a->id);
	}
	return ret;
}

int bypass_video_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device 	*ispcam = ispvideo->ispcam;
	struct isp_device 	*isp	= ispcam->isp;
	struct sensor_device *sensor = &isp->ispcam->sensor;
	int ret = 0;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, a);
	if(ret < 0) {
		dev_err(isp->dev, "failed to set ID:%d!\n", a->id);
	}
	return ret;
}

int vic_video_nr_map[3] = {
	INGENIC_VIC0_VIDEO_NR,
	INGENIC_VIC1_VIDEO_NR,
	INGENIC_VIC2_VIDEO_NR,
};

static int vic_comp_bind(struct device *comp, struct device *master,
                              void *master_data)
{
	struct vic_device *vic = dev_get_drvdata(comp);
	struct ispcam_device *ispcam = (struct ispcam_device *)master_data;
	struct v4l2_device *v4l2_dev = &ispcam->v4l2_dev;
	struct v4l2_subdev *sd = &vic->sd;
	int ret = 0;
	int nr = vic_video_nr_map[ispcam->dev_nr];

	//dev_info(comp, "--------%s, %d \n", __func__, __LINE__);
	/* link subdev to master.*/
	vic->ispcam = (void *)ispcam;
	ispcam->vic = vic;

	v4l2_subdev_init(sd, &vic_subdev_ops);


	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(comp), sizeof(sd->name));
	v4l2_set_subdevdata(sd, vic);

	/* init vic pads. */
	vic->pads = kzalloc(sizeof(struct media_pad) * VIC_NUM_PADS, GFP_KERNEL);
	if(!vic->pads) {
		ret = -ENOMEM;
		goto err_alloc_pads;
	}
	vic->pads[0].index = VIC_PAD_SINK;
	vic->pads[0].flags = MEDIA_PAD_FL_SINK;	/* CSI->VIC, MIPI Interface*/
	vic->pads[1].index = VIC_PAD_SOURCE;
	vic->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, VIC_NUM_PADS, vic->pads);

	/*3. register v4l2_subdev*/
	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if(ret < 0) {
		dev_err(comp, "Failed to register v4l2_subdev for vic\n");
		goto err_subdev_register;
	}

#ifdef CONFIG_VIC_DMA_ROUTE
	init_completion(&vic->snap_comp);

	struct isp_video_device *ispvideo = &vic->ispvideo;
	ispvideo->ispcam = ispcam;
	ispvideo->bypass = 1;
	char name[32];
	sprintf(name, "%s", dev_name(vic->dev));
	ret = isp_video_init(ispvideo, name, &vic_video_ops);
	if(ret < 0) {
		/*TODO:*/

	}

	ret = isp_video_register(ispvideo, &ispcam->v4l2_dev, nr);
	if(ret < 0) {
		/*TODO:*/
	};
#endif

err_subdev_register:
err_alloc_pads:


	return ret;
}


static void vic_comp_unbind(struct device *comp, struct device *master,
                                 void *master_data)
{
        struct vic_device *vic = dev_get_drvdata(comp);

	dev_info(comp, "---TODO: %p-----%s, %d \n", vic, __func__, __LINE__);

}

static const struct component_ops vic_comp_ops = {
        .bind = vic_comp_bind,
        .unbind = vic_comp_unbind,
};

static int vic_err[13] = {0};

static irqreturn_t vic_irq_handler(int irq, void *data)
{
	struct vic_device *vic = (struct vic_device *)data;
	unsigned int status = 0;
	unsigned int status2 = 0;
	struct isp_video_buffer *isp_buffer = NULL;
	unsigned int regval = 0;
	unsigned int complete_num = 0;

	spin_lock(&vic->lock);
	status = vic_reg_readl(vic, VIC_INT_STA);
	status2 = vic_reg_readl(vic, VIC_INT_STA2);
//	printk("VIC_INT_STA=%x\n", status);
//	printk("VIC_INT_STA2=%x\n", status2);
	/* DMA FRAME DONE */
	if(status2 & (1 << 0)) {
		vic->frame_capture_counter++;
		//printk(KERN_DEBUG "%d: vic->buffer_count=0x%x\n", vic->frame_capture_counter, vic->buffer_count);

#ifdef CONFIG_VIC_DMA_ROUTE
		if(dma_debug_en){
			vic->dma_complete++;
		} else {
			isp_buffer = list_first_entry_or_null(&vic->dma_queued_list, struct isp_video_buffer, list_entry);
			if(!isp_buffer) {
				dev_err(vic->dev, "[warning] no isp_buffer found in dma_queued_list when interrupt happend!\n");
				goto done;
			}

			list_del(&isp_buffer->list_entry);
			vic->buffer_count--;

			isp_buffer->vb2.vb2_buf.timestamp = ktime_get_ns();
			isp_buffer->vb2.sequence = vic->framenum++;

			vb2_buffer_done(&isp_buffer->vb2.vb2_buf, VB2_BUF_STATE_DONE);

			if(vic->buffer_count <= VIC_STOP_BUFFER_COUNT)
			{
				vic->stop_flag = 1;
				regval = vic_reg_readl(vic, VIC_DMA_CONFIG);
				regval &= ~(1 << 31);
				vic_reg_writel(vic, VIC_DMA_CONFIG, regval);
				vic_stop_counter++;
				goto done;
			}
		}
#endif
	}

	if(status2 & (1 << 1))
		if(dma_debug_en)
			vic->dma_complete++;

#if 0
	if(status & (1 << 9)) {
		//dev_err(vic->dev,
		printk(KERN_DEBUG "vic->frame_capture_counter=%d, Image output limit too small! VIC_INT_STA=%x\n", vic->frame_capture_counter, status);
		/* lgwang -- reset vic */
		vic->global_reset = 1;
		vic_reg_writel(vic, VIC_CONTROL, 0x4);
		vic->stop_flag = 1;
		vic_stop_counter++;
	}
#endif
	if((1 << 9) & status) {
		vic_err[11]++;
		dev_err(vic->dev, "Err [VIC_INT] : frame asfifo ovf!!!!!\n");
	}
	if((1 << 10) & status) {
		vic_err[0]++;
		dev_err(vic->dev, "Err [VIC_INT] : hor err ch0 !!!!!\n");
	}
	if((1 << 11) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : hor err ch1 !!!!!\n");
	}
	if((1 << 12) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : hor err ch2 !!!!!\n");
	}
	if((1 << 13) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : hor err ch3 !!!!!\n");
	}
	if((1 << 14) & status) {
		vic_err[1]++;
		dev_err(vic->dev, "Err [VIC_INT] : ver err ch0 !!!!!\n");
	}
	if((1 << 15) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : ver err ch1 !!!!!\n");
	}
	if((1 << 16) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : ver err ch2 !!!!!\n");
	}
	if((1 << 17) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : ver err ch3 !!!!!\n");
	}
	if((1 << 18) & status) {
		vic_err[2]++;
		dev_err(vic->dev, "Err [VIC_INT] : hvf err !!!!!\n");
	}
	if((1 << 19) & status) {
		vic_err[3]++;
		dev_err(vic->dev, "Err [VIC_INT] : dvp hcomp err!!!!\n");
	}
	if((1 << 20) & status) {
		vic_err[4]++;
		dev_err(vic->dev, "Err [VIC_INT] : dma syfifo ovf!!!\n");
	}
	if((1 << 21) & status) {
		vic_err[5]++;
		dev_err(vic->dev, "Err [VIC_INT] : control limit err!!!\n");
	}
	if((1 << 22) & status) {
		vic_err[6]++;
		dev_err(vic->dev, "Err [VIC_INT] : image syfifo ovf !!!\n");
	}
	if((1 << 23) & status) {
		vic_err[7]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi fid asfifo ovf!!!\n");
	}
	if((1 << 24) & status) {
		vic_err[8]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch0 hcomp err !!!\n");
	}
	if((1 << 25) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch1 hcomp err !!!\n");
	}
	if((1 << 26) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch2 hcomp err !!!\n");
	}
	if((1 << 27) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch3 hcomp err !!!\n");
	}
	if((1 << 28) & status) {
		vic_err[9]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch0 vcomp err !!!\n");
	}
	if((1 << 29) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch1 vcomp err !!!\n");
	}
	if((1 << 30) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch2 vcomp err !!!\n");
	}
	if((1 << 31) & status) {
		vic_err[12]++;
		dev_err(vic->dev, "Err [VIC_INT] : mipi ch3 vcomp err !!!\n");
	}

	if(dma_debug_en)
	{
		if(vic->sensor_info->wdr_en)
			complete_num = 2;
		else
			complete_num = 1;

		if(vic->dma_complete == complete_num)
		{
			vic_reg_writel(vic, VIC_DMA_CONFIG, 0);
			complete(&vic->snap_comp);
		}
	}
done:
	vic_reg_writel(vic, VIC_INT_CLR, status);
	vic_reg_writel(vic, VIC_INT_CLR2, status2);
	spin_unlock(&vic->lock);
	return IRQ_HANDLED;

}

static int ingenic_vic_probe(struct platform_device *pdev)
{

	struct vic_device *vic = NULL;
	struct resource *regs = NULL;
	int ret = 0;

	vic = kzalloc(sizeof(struct vic_device), GFP_KERNEL);
	if(!vic) {
		pr_err("Failed to alloc vic dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	vic->dev = &pdev->dev;
	platform_set_drvdata(pdev, vic);


	ingenic_vic_parse_dt(vic);

	vic->irq = platform_get_irq(pdev, 0);
	if(vic->irq < 0) {
		dev_warn(&pdev->dev, "No CSI IRQ specified\n");
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!regs) {
		dev_err(&pdev->dev, "No iomem resource!\n");
		goto err_get_resource;
	}

	vic->iobase = devm_ioremap_resource(&pdev->dev, regs);
	if(!vic->iobase) {
		goto err_ioremap;
	}

	spin_lock_init(&vic->lock);
	INIT_LIST_HEAD(&vic->dma_queued_list);

	ret = devm_request_irq(vic->dev, vic->irq, vic_irq_handler, 0,
			dev_name(vic->dev), vic);
	if(ret) {
		dev_err(vic->dev, "request irq failed!\n");
		goto err_request_irq;
	}

	ret = sysfs_create_group(&vic->dev->kobj, &vic_debug_attr_group);
	if (ret) {
		dev_err(vic->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_sys_group;
	}

	ret = component_add(vic->dev, &vic_comp_ops);
	if(ret < 0) {
		dev_err(vic->dev, "Failed to add component vic!\n");
		goto err_component;
	}

	return 0;
err_component:
err_sys_group:
err_request_irq:
err_ioremap:
err_get_resource:
	return ret;
}



static int ingenic_vic_remove(struct platform_device *pdev)
{

	return 0;
}



static const struct of_device_id ingenic_vic_dt_match[] = {
        { .compatible = "ingenic,x2500-vic" },
        { }
};

MODULE_DEVICE_TABLE(of, ingenic_vic_dt_match);

static int __maybe_unused ingenic_vic_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct vic_device *vic = dev_get_drvdata(&pdev->dev);

	if(vic->enabled){
		dev_err(vic->dev, "faild to suspend, vic is streaming on\n");
		return -EBUSY;
	}

        return 0;
}

static int __maybe_unused ingenic_vic_resume(struct platform_device *pdev)
{
        return 0;
}

static struct platform_driver ingenic_vic_driver = {
        .probe = ingenic_vic_probe,
        .remove = ingenic_vic_remove,
	.suspend = ingenic_vic_suspend,
	.resume = ingenic_vic_resume,
        .driver = {
                .name = "ingenic-vic",
                .of_match_table = ingenic_vic_dt_match,
        },
};

module_platform_driver(ingenic_vic_driver);

MODULE_ALIAS("platform:ingenic-vic");
MODULE_DESCRIPTION("ingenic vic subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");

