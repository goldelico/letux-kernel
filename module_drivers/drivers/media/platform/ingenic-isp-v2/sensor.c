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

#define to_sensor_device(isp) 	&isp->ispcam->sensor

extern unsigned int MAIN_ISP_INDEX;
extern unsigned int SEC_ISP_INDEX;

extern struct isp_device *g_isp_device[2];
struct isp_device *index_to_isp_device(int isp_index)
{
	if(isp_index == 0)
		return g_isp_device[0];
	else if(isp_index == 1)
		return g_isp_device[1];
	else{
		printk("unsupported isp index!\n");
		return NULL;
	}
}

#if 1
static void sensor_hw_reset_enable(void)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_hw_reset_disable(void)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static int32_t sensor_alloc_analog_gain(int32_t gain, sensor_context_t *p_ctx)
{
	/* printk("result gain is 0x%x\n",again); */
	return gain;
}

static int32_t sensor_alloc_analog_gain_short(int32_t gain, sensor_context_t *p_ctx)
{
	/* printk("result gain is 0x%x\n",again); */
	return gain;
}

static int32_t sensor_alloc_digital_gain(int32_t gain, sensor_context_t *p_ctx)
{
//	printk("----%s, %d, gain: %d\n", __func__, __LINE__, gain);

	/* printk("result gain is 0x%x\n",p_ctx->dgain); */
	return gain;
}

static uint32_t sensor_alloc_integration_time(uint32_t int_time, sensor_context_t *p_ctx)
{
	p_ctx->it = int_time;
	return int_time;
}

static uint32_t sensor_alloc_integration_time_short(uint32_t int_time, sensor_context_t *p_ctx)
{
	p_ctx->it_short = int_time;
	return int_time;
}

static void sensor_set_integration_time(uint16_t int_time, sensor_param_t* param)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control exposure;
	int ret = 0;

	if(param->sensor_ctx.sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(param->sensor_ctx.sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	exposure.id = V4L2_CID_EXPOSURE;
	exposure.value = int_time;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &exposure);
	if(ret < 0) {
		dev_err(isp->dev, "failed to set exposure!\n");
	}
}

static void sensor_set_integration_time_short(uint16_t int_time, sensor_param_t* param)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control exposure;
	int ret = 0;

	if(param->sensor_ctx.sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(param->sensor_ctx.sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	exposure.id = V4L2_CID_USER_EXPOSURE_SHORT;
	exposure.value = int_time;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &exposure);
	if(ret < 0) {
		dev_err(isp->dev, "failed to set exposure!\n");
	}
}

static void sensor_set_analog_gain(uint32_t again_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control analog;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	analog.id = V4L2_CID_ANALOGUE_GAIN;
	analog.value = again_reg_val;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}
}

static void sensor_get_analog_gain(uint32_t *again_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control analog;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	analog.id = V4L2_CID_ANALOGUE_GAIN;
	analog.value = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}


	*again_reg_val = analog.value;
}

static void sensor_set_analog_gain_short(uint32_t again_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control analog;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	analog.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	analog.value = again_reg_val;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}
}

static void sensor_get_analog_gain_short(uint32_t *again_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control analog;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	analog.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	analog.value = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, &analog);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}


	*again_reg_val = analog.value;
}

static void sensor_set_digital_gain(uint32_t dgain_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control control;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	control.id = V4L2_CID_GAIN;
	control.value = dgain_reg_val;

	ret = v4l2_s_ctrl(NULL, sensor->isd->sd->ctrl_handler, &control);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}
}

static int sensor_get_digital_gain(uint32_t *dgain_reg_val, sensor_context_t *p_ctx)
{
	int isp_index = -1;
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_control control;
	int ret = 0;

	if(p_ctx->sensor_id == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(p_ctx->sensor_id == 1)
		isp_index = SEC_ISP_INDEX;
	isp = index_to_isp_device(isp_index);
	sensor = to_sensor_device(isp);

	control.id = V4L2_CID_GAIN;
	control.value = 0;

	ret = v4l2_g_ctrl(sensor->isd->sd->ctrl_handler, &control);
	if(ret < 0) {
		printk("error s_ctrl, ret: %d\n", ret);
	}

	*dgain_reg_val = control.value;

	return ret;
}
#endif

#if 1
static uint16_t sensor_get_normal_fps(sensor_param_t* param)
{
//	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static uint16_t sensor_read_black_pedestal(int i,uint32_t gain)
{
//	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static void sensor_set_mode(uint8_t mode, sensor_param_t* param)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_start_changes(sensor_context_t *p_ctx)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static void sensor_end_changes(sensor_context_t *p_ctx)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}


static uint16_t sensor_get_id(sensor_context_t *p_ctx)
{
//	printk("----%s, %d\n", __func__, __LINE__);

	return 0;
}

static void sensor_set_wdr_mode(uint8_t mode, sensor_param_t* param)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static uint32_t sensor_fps_control(uint8_t fps, sensor_param_t* param)
{

//	printk("----%s, %d\n", __func__, __LINE__);
	return 0;
}

static void sensor_disable_isp(sensor_context_t *p_ctx)
{
//	printk("----%s, %d\n", __func__, __LINE__);
}

static uint32_t sensor_get_lines_per_second(sensor_param_t* param)
{
	uint32_t lines_per_second=0;
//	printk("----%s, %d\n", __func__, __LINE__);
	return lines_per_second;
}
#endif


extern tisp_init_par_t tisp_par_info;
void sensor_init(sensor_control_t *ctrl, int vinum)
{
	struct isp_device *isp = NULL;
	struct sensor_device *sensor = NULL;
	struct v4l2_queryctrl query;
	tisp_init_param_t *sensor_info = &tisp_par_info.sensor[vinum];
	int ret = 0;
	int isp_index = -1;

	if(vinum == 0)
		isp_index = MAIN_ISP_INDEX;
	else if(vinum == 1)
		isp_index = SEC_ISP_INDEX;

	isp = g_isp_device[isp_index];
	sensor = to_sensor_device(isp);

	query.id = V4L2_CID_EXPOSURE;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_EXPOSURE failed\n");
	}

	ctrl->param[vinum].integration_time_min = query.minimum;
	ctrl->param[vinum].integration_time_max = query.maximum;

	query.id = V4L2_CID_USER_EXPOSURE_SHORT;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_USER_EXPOSURE_SHORT failed\n");
	}

	ctrl->param[vinum].integration_time_min_short = query.minimum;
	ctrl->param[vinum].integration_time_max_short = query.maximum;

	query.id = V4L2_CID_ANALOGUE_GAIN;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_ANALOGUE_GAIN failed\n");
	}
	ctrl->param[vinum].again_log2_max = query.maximum;

	query.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "query sensor V4L2_CID_USER_ANALOG_GAIN failed\n");
	}
	ctrl->param[vinum].again_log2_max_short = query.maximum;


	query.id = V4L2_CID_GAIN;
	query.type = V4L2_CTRL_TYPE_INTEGER;
	ret = v4l2_queryctrl(sensor->isd->sd->ctrl_handler, &query);
	if(ret < 0) {
		dev_err(isp->dev, "queyr sensor V4L2_CID_GAIN failed\n");
	}
	ctrl->param[vinum].dgain_log2_max = query.maximum;

#if 1
	/*fill sensor_info*/
	sensor_info->sensor_info.max_again = ctrl->param[vinum].again_log2_max;       //the format is .16
	sensor_info->sensor_info.max_again_short = ctrl->param[vinum].again_log2_max_short;       //the format is .16
	sensor_info->sensor_info.max_dgain = ctrl->param[vinum].dgain_log2_max;      //the format is .16
// 	sensor_info->sensor_info.again = core->vin.attr->again;
// 	sensor_info->sensor_info.dgain = core->vin.attr->dgain;
// 	sensor_info->sensor_info.fps = core->vin.fps;
	sensor_info->sensor_info.min_integration_time = ctrl->param[vinum].integration_time_min;
	sensor_info->sensor_info.max_integration_time = ctrl->param[vinum].integration_time_max;
	sensor_info->sensor_info.min_integration_time_short = ctrl->param[vinum].integration_time_min_short;
	sensor_info->sensor_info.max_integration_time_short = ctrl->param[vinum].integration_time_max_short;
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
#endif

	ctrl->hw_reset_disable = sensor_hw_reset_disable;
	ctrl->hw_reset_enable = sensor_hw_reset_enable;
	ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
	ctrl->alloc_analog_gain_short = sensor_alloc_analog_gain_short;
	ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
	ctrl->alloc_integration_time = sensor_alloc_integration_time;
	ctrl->alloc_integration_time_short = sensor_alloc_integration_time_short;
	ctrl->set_integration_time = sensor_set_integration_time;
	ctrl->set_integration_time_short = sensor_set_integration_time_short;
	ctrl->start_changes = sensor_start_changes;
	ctrl->end_changes = sensor_end_changes;
	ctrl->set_analog_gain = sensor_set_analog_gain;
	ctrl->get_analog_gain = sensor_get_analog_gain;
	ctrl->set_analog_gain_short = sensor_set_analog_gain_short;
	ctrl->get_analog_gain_short = sensor_get_analog_gain_short;
	ctrl->set_digital_gain = sensor_set_digital_gain;
	ctrl->get_normal_fps = sensor_get_normal_fps;
	ctrl->read_black_pedestal = sensor_read_black_pedestal;
	ctrl->set_mode = sensor_set_mode;
	ctrl->set_wdr_mode = sensor_set_wdr_mode;
	ctrl->fps_control = sensor_fps_control;
	ctrl->get_id = sensor_get_id;
	ctrl->disable_isp = sensor_disable_isp;
	ctrl->get_lines_per_second = sensor_get_lines_per_second;
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
