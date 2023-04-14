#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>

#include "isp-drv.h"


#define to_isp_device(c) 	(struct isp_device *)c->priv_data
#define to_sensor_device(isp) 	&isp->ispcam->sensor

static void sensor_hw_reset_enable(sensor_control_t *ctrl)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_hw_reset_disable(sensor_control_t *ctrl)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static int32_t sensor_alloc_analog_gain(sensor_control_t *ctrl, int32_t gain, sensor_context_t *p_ctx)
{
	/* printk("result gain is 0x%x\n",again); */
	return gain;
}

static int32_t sensor_alloc_digital_gain(sensor_control_t *ctrl, int32_t gain, sensor_context_t *p_ctx)
{
	printk("----%s, %d, gain: %d\n", __func__, __LINE__, gain);

	/* printk("result gain is 0x%x\n",p_ctx->dgain); */
	return gain;
}

static void sensor_alloc_integration_time(sensor_control_t *ctrl, uint16_t *int_time, sensor_context_t *p_ctx)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_set_integration_time(sensor_control_t *ctrl, uint16_t int_time, sensor_param_t* param)
{
	struct isp_device *isp = to_isp_device(ctrl);
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_control exposure;
	int ret = 0;

	exposure.id = V4L2_CID_EXPOSURE;
	exposure.value = int_time;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &exposure);
	if(ret < 0) {
		dev_err(isp->dev, "failed to set exposure!\n");
	}
}

static void sensor_set_analog_gain(sensor_control_t *ctrl, uint32_t again_reg_val, sensor_context_t *p_ctx)
{
	struct isp_device *isp = to_isp_device(ctrl);
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_control analog;
	int ret = 0;

	analog.id = V4L2_CID_ANALOGUE_GAIN;
	analog.value = again_reg_val;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}
}

static int sensor_get_analog_gain(sensor_control_t *ctrl, uint32_t *again_reg_val, sensor_context_t *p_ctx)
{
	struct isp_device *isp = to_isp_device(ctrl);
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_control analog;
	int ret = 0;

	analog.id = V4L2_CID_ANALOGUE_GAIN;
	analog.value = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}


	*again_reg_val = analog.value;

	return ret;
}


static void sensor_set_digital_gain(sensor_control_t *ctrl, uint32_t dgain_reg_val, sensor_context_t *p_ctx)
{
	struct isp_device *isp = to_isp_device(ctrl);
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_control control;
	int ret = 0;

	control.id = V4L2_CID_GAIN;
	control.value = dgain_reg_val;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &control);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}
}

static int sensor_get_digital_gain(sensor_control_t *ctrl, uint32_t *dgain_reg_val, sensor_context_t *p_ctx)
{
	struct isp_device *isp = to_isp_device(ctrl);
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_control control;
	int ret = 0;

	control.id = V4L2_CID_GAIN;
	control.value = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, &control);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}

	*dgain_reg_val = control.value;

	return ret;
}


static uint16_t sensor_get_normal_fps(sensor_control_t *ctrl, sensor_param_t* param)
{
	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static uint16_t sensor_read_black_pedestal(sensor_control_t *ctrl, int i,uint32_t gain)
{
	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static void sensor_set_mode(sensor_control_t *ctrl, uint8_t mode, sensor_param_t* param)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_start_changes(sensor_control_t *ctrl, sensor_context_t *p_ctx)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_end_changes(sensor_control_t *ctrl, sensor_context_t *p_ctx)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static uint16_t sensor_get_id(sensor_control_t *ctrl)
{
	printk("----%s, %d\n", __func__, __LINE__);

	return 0;
}

static void sensor_set_wdr_mode(sensor_control_t *ctrl, uint8_t mode, sensor_param_t* param)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static uint32_t sensor_fps_control(sensor_control_t *ctrl, uint8_t fps, sensor_param_t* param)
{

	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static void sensor_disable_isp(sensor_control_t *ctrl)
{
	printk("----%s, %d\n", __func__, __LINE__);
}

static uint32_t sensor_get_lines_per_second(sensor_control_t *ctrl, sensor_param_t* param)
{
	uint32_t lines_per_second=0;
	printk("----%s, %d\n", __func__, __LINE__);
	return lines_per_second;
}

struct sensor_control_ops sensor_ctrl_ops = {
	.hw_reset_disable 	= sensor_hw_reset_disable,
	.hw_reset_enable 	= sensor_hw_reset_enable,
	.alloc_analog_gain 	= sensor_alloc_analog_gain,
	.alloc_digital_gain 	= sensor_alloc_digital_gain,
	.alloc_integration_time = sensor_alloc_integration_time,
	.set_integration_time 	= sensor_set_integration_time,
	.start_changes 		= sensor_start_changes,
	.end_changes 		= sensor_end_changes,
	.set_analog_gain 	= sensor_set_analog_gain,
	.get_analog_gain 	= sensor_get_analog_gain,
	.set_digital_gain 	= sensor_set_digital_gain,
	.get_digital_gain 	= sensor_get_digital_gain,
	.get_normal_fps 	= sensor_get_normal_fps,
	.read_black_pedestal 	= sensor_read_black_pedestal,
	.set_mode 		= sensor_set_mode,
	.set_wdr_mode 		= sensor_set_wdr_mode,
	.fps_control 		= sensor_fps_control,
	.get_id			= sensor_get_id,
	.disable_isp		= sensor_disable_isp,
	.get_lines_per_second	= sensor_get_lines_per_second
};

void sensor_init(sensor_control_t *ctrl, void *priv_data)
{
	tisp_core_t *core = priv_data;
	struct isp_device *isp = core->priv_data;
	struct sensor_device *sensor = to_sensor_device(isp);
	struct v4l2_queryctrl query;
	tisp_init_param_t *sensor_info = &core->sensor_info;
	int ret = 0;

	query.id = V4L2_CID_EXPOSURE;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_EXPOSURE failed\n");
	}

	ctrl->param.integration_time_min = query.minimum;
	ctrl->param.integration_time_max = query.maximum;

	query.id = V4L2_CID_ANALOGUE_GAIN;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_ANALOGUE_GAIN failed\n");
	}
	ctrl->param.again_log2_max = query.maximum;


	query.id = V4L2_CID_GAIN;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "queyr sensor V4L2_CID_GAIN failed\n");
	}
	ctrl->param.dgain_log2_max = query.maximum;

#if 0
	struct tx_isp_video_in *vin = &core->vin;
	struct tx_isp_sensor_attribute *attr = vin->attr;

	ctrl->param.again_log2_max = 		attr->max_again;
	ctrl->param.dgain_log2_max= 		attr->max_dgain;
	ctrl->param.integration_time_apply_delay = attr->integration_time_apply_delay;
	ctrl->param.analog_gain_apply_delay = 	attr->again_apply_delay;
	ctrl->param.digital_gain_apply_delay = 	attr->dgain_apply_delay;
	ctrl->param.integration_time_min = 	attr->min_integration_time;
	ctrl->param.integration_time_max = 	attr->max_integration_time;
#endif
	ctrl->priv_data = isp;
	isp->core.sensor_ctrl_ops = &sensor_ctrl_ops;

	/*fill sensor_info*/
	sensor_info->sensor_info.max_again = ctrl->param.again_log2_max;       //the format is .16
	sensor_info->sensor_info.max_dgain = ctrl->param.dgain_log2_max;      //the format is .16
// 	sensor_info->sensor_info.again = core->vin.attr->again;
// 	sensor_info->sensor_info.dgain = core->vin.attr->dgain;
// 	sensor_info->sensor_info.fps = core->vin.fps;
	sensor_info->sensor_info.min_integration_time = ctrl->param.integration_time_min;
	sensor_info->sensor_info.max_integration_time = ctrl->param.integration_time_max;
// 	sensor_info->sensor_info.min_integration_time_native = core->vin.attr->min_integration_time_native;
// 	sensor_info->sensor_info.max_integration_time_native = core->vin.attr->max_integration_time_native;
// 	sensor_info->sensor_info.integration_time_limit = core->vin.attr->integration_time_limit;
// 	sensor_info->sensor_info.integration_time = core->vin.attr->integration_time;
// 	sensor_info->sensor_info.total_width = core->vin.attr->total_width;
// 	sensor_info->sensor_info.total_height = core->vin.attr->total_height;
// 	sensor_info->sensor_info.integration_time_apply_delay = core->vin.attr->integration_time_apply_delay;
// 	sensor_info->sensor_info.again_apply_delay = core->vin.attr->again_apply_delay;
// 	sensor_info->sensor_info.dgain_apply_delay = core->vin.attr->dgain_apply_delay;
//	sensor_info->sensor_info.one_line_expr_in_us = core->vin.attr->one_line_expr_in_us;

}



int sensor_device_probe(struct sensor_device *sensor, struct ispcam_device *ispcam)
{
	int ret = 0;
	sensor->ispcam = ispcam;
	sensor->isp = ispcam->isp;
	sensor->isd = &ispcam->isd[0];

	return ret;
}
EXPORT_SYMBOL_GPL(sensor_device_probe);
