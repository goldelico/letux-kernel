#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "isp-drv.h"

static inline int apical_isp_hflip_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;
	unsigned int value = 0;
	value = isp->ctrls.hflip;

	tisp_mirror_enable(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_hflip_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;
	return ret;
}

static inline int apical_isp_vflip_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;
	unsigned int value = 0;
	value = isp->ctrls.vflip;

	tisp_flip_enable(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_vflip_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;
	return ret;
}

static inline int apical_isp_sat_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = isp->ctrls.sat & 0xff;
	tisp_set_saturation(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_sat_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if(input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16){
		return ret;
	}
	/* the original value */
	value = tisp_get_saturation(&core->core_tuning);
	isp->ctrls.contrast = value;

	return ret;
}

static inline int apical_isp_bright_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = isp->ctrls.bright & 0xff;
	tisp_set_brightness(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_bright_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = tisp_get_brightness(&core->core_tuning);
	isp->ctrls.bright = value;

	return ret;
}

static inline int apical_isp_contrast_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = isp->ctrls.contrast & 0xff;
	tisp_set_contrast(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_contrast_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = tisp_get_contrast(&core->core_tuning);
	isp->ctrls.contrast = value;

	return ret;
}

static inline int apical_isp_sharp_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if(input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16){
		return ret;
	}
	/* the original value */
	value = isp->ctrls.sharp & 0xff;
	tisp_set_sharpness(&core->core_tuning, value);

	return ret;
}

static inline int apical_isp_sharp_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 128;
	int ret = 0;

	if (input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16) {
		return ret;
	}
	/* the original value */
	value = tisp_get_sharpness(&core->core_tuning);
	isp->ctrls.sharp = value;

	return ret;
}


static int ispcore_exp_auto_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;
	/*
	 * control->value.
	 *	0: 自动曝光.
	 *	1: 手动曝光.
	 *	2. 快门优先.
	 *	3. 光圈优先.
	 * */

	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);
	if (isp->ctrls.exp_auto == V4L2_EXPOSURE_AUTO) {
		ae_attr.manual_it = 0;
	} else if(isp->ctrls.exp_auto == V4L2_EXPOSURE_MANUAL) {
		ae_attr.manual_it = 1;
	}

	ret = tisp_s_ae_attr(&core->core_tuning, ae_attr);

	return ret;
}

/* 获取当前曝光方式.*/
static int ispcore_exp_auto_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;

	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	isp->ctrls.exp_auto = ae_attr.manual_it == 0 ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;

	return ret;
}

/* 设置曝光等级. eg: [-4, 4]
 *
 *  相对曝光时间. 从 [-4, 4] -> [inte_min, inte_max]
 * */
static int ispcore_exp_s_control(tisp_core_t *core)
{

	return 0;
}

static int ispcore_exp_g_control(tisp_core_t *core)
{

	return 0;
}

/* 设置曝光时间. */
static int ispcore_exp_abs_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;

	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);
	if (ae_attr.manual_it) {
		ae_attr.integration_time = isp->ctrls.exp_abs;
		ret = tisp_s_ae_attr(&core->core_tuning, ae_attr);
	} else {
		dev_warn(isp->dev, "set exp_abs while in exposure [auto] mode.\n");
	}

	return ret;
}

static int ispcore_exp_abs_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;

	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	isp->ctrls.exp_abs = ae_attr.integration_time;

	return ret;
}

/*设置自动增益控制方式
 * 0: 自动增益控制
 * 1: 手动增益控制.
 * */
static inline int ispcore_auto_gain_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;
	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	if (isp->ctrls.auto_gain == 0 ) {
		ae_attr.manual_ag = 0;
	} else if (isp->ctrls.auto_gain == 1 ){
		ae_attr.manual_ag = 1;
	}

	ret = tisp_s_ae_attr_ag(&core->core_tuning, ae_attr);

	return ret;
}

/*获取自动增益控制方式.*/
static inline int ispcore_auto_gain_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;
	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	isp->ctrls.auto_gain = ae_attr.manual_ag;

	return ret;
}

/*手动增益控制模式下，设置当前增益.*/
static inline int ispcore_gain_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;
	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	if (ae_attr.manual_ag) {
		ae_attr.again = isp->ctrls.gain;
		ret = tisp_s_ae_attr_ag(&core->core_tuning, ae_attr);
	} else {
		dev_warn(isp->dev, "set gain_s warning while in gain mode.\n");
	}

	return ret;
}

static inline int ispcore_gain_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_ev_attr_t ae_attr;
	int ret = 0;
	ret = tisp_g_ev_attr(&core->core_tuning, &ae_attr);

	isp->ctrls.gain = ae_attr.again;

	return ret;
}

static inline int ispcore_auto_wb_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t *wb_attr = &isp->ctrls.wb_attr;
	int ret = 0;

	ret = tisp_s_wb_frz_attr(&core->core_tuning, 0);
	switch(isp->ctrls.auto_wb) {
	case 1:
		wb_attr->tisp_wb_manual = 0;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case 0:
		wb_attr->tisp_wb_manual = 1;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		ret = tisp_s_wb_frz_attr(&core->core_tuning, 1);
		break;
	default:
		break;
	}
	return ret;
}


static inline int ispcore_auto_n_wb_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t *wb_attr = &isp->ctrls.wb_attr;
	int ret = 0;

	ret = tisp_s_wb_frz_attr(&core->core_tuning, 0);
	switch(isp->ctrls.wb) {
	case V4L2_WHITE_BALANCE_AUTO:
		wb_attr->tisp_wb_manual = 0;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_MANUAL:
		wb_attr->tisp_wb_manual = 1;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		ret = tisp_s_wb_frz_attr(&core->core_tuning, 1);
		break;
	case V4L2_WHITE_BALANCE_FLASH:
		wb_attr->tisp_wb_manual = 2;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT:
		wb_attr->tisp_wb_manual = 3;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_CLOUDY:
		wb_attr->tisp_wb_manual = 4;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_HORIZON:
		wb_attr->tisp_wb_manual = 5;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_DAYLIGHT:
		wb_attr->tisp_wb_manual = 6;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT_H:
		wb_attr->tisp_wb_manual = 7;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_INCANDESCENT:
		wb_attr->tisp_wb_manual = 8;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		break;
	case V4L2_WHITE_BALANCE_SHADE:
		wb_attr->tisp_wb_manual = 9;
		ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
		ret = tisp_s_wb_frz_attr(&core->core_tuning, 1);
		break;
	default:
		break;
	}
	return ret;
}


static inline int ispcore_red_wb_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t *wb_attr = &isp->ctrls.wb_attr;
	int ret = 0;

	wb_attr->tisp_wb_rg = isp->ctrls.red_wb;
	return ret;
}


static inline int ispcore_blue_wb_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t *wb_attr = &isp->ctrls.wb_attr;
	int ret = 0;

	wb_attr->tisp_wb_bg = isp->ctrls.blue_wb;
	return ret;
}


static inline int ispcore_do_wb_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t *wb_attr = &isp->ctrls.wb_attr;
	int ret = 0;

	ret = tisp_s_wb_frz_attr(&core->core_tuning, 0);
	ret = tisp_s_wb_attr(&core->core_tuning, *wb_attr);
	return ret;
}


static inline int ispcore_auto_wb_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t wb_attr;
	int ret = 0;

	ret = tisp_g_wb_attr(&core->core_tuning, &wb_attr);
	switch( wb_attr.tisp_wb_manual ) {
	case 0:
		isp->ctrls.auto_wb = V4L2_WHITE_BALANCE_AUTO;
		break;
	case 1:
		isp->ctrls.auto_wb = V4L2_WHITE_BALANCE_MANUAL;
		break;
	default:
		break;
	}
	return ret;
}


static inline int ispcore_auto_n_wb_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t wb_attr;
	int ret = 0;

	ret = tisp_g_wb_attr(&core->core_tuning, &wb_attr);
	switch( wb_attr.tisp_wb_manual ) {
	case 0:
	        isp->ctrls.wb = V4L2_WHITE_BALANCE_AUTO;
		break;
	case 1:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_MANUAL;
		break;
	case 2:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_FLASH;
		break;
	case 3:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_FLUORESCENT;
		break;
	case 4:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_CLOUDY;
		break;
	case 5:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_HORIZON;
		break;
	case 6:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_DAYLIGHT;
		break;
	case 7:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_FLUORESCENT_H;
		break;
	case 8:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_INCANDESCENT;
		break;
	case 9:
		isp->ctrls.wb = V4L2_WHITE_BALANCE_SHADE;
		break;
	default:
		break;
	}
	return ret;
}

static inline int ispcore_red_wb_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t wb_attr;
	int ret = 0;

	ret = tisp_g_wb_attr(&core->core_tuning, &wb_attr);
	if (wb_attr.tisp_wb_manual == 0) {
		isp->ctrls.red_wb = 65536 / wb_attr.tisp_wb_rg_sta_weight;
	} else {
		isp->ctrls.red_wb = wb_attr.tisp_wb_rg;
	}
	return ret;
}

static inline int ispcore_blue_wb_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	tisp_wb_attr_t wb_attr;
	int ret = 0;

	ret = tisp_g_wb_attr(&core->core_tuning, &wb_attr);
	if (wb_attr.tisp_wb_manual == 0) {
		isp->ctrls.blue_wb = 65536 / wb_attr.tisp_wb_bg_sta_weight;
	} else {
		isp->ctrls.blue_wb = wb_attr.tisp_wb_bg;
	}
	return ret;
}


static inline int apical_isp_hilightdepress_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	unsigned int value = 0;
	int ret = 0;

	if(input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16){
		return ret;
	}
	value = isp->ctrls.hill;
	ret = tisp_s_Hilightdepress(&core->core_tuning, value);

	return ret;
}


static inline int apical_isp_hilightdepress_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	struct ispcam_device *ispcam = isp->ispcam;
	struct vic_device *vic = ispcam->vic;
	struct v4l2_subdev_format *input_fmt = &vic->formats[VIC_PAD_SINK];
	int ret=0;

	if(input_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16){
		return -1;
	}
	ret = tisp_g_Hilightdepress(&core->core_tuning, &isp->ctrls.hill);

	return 0;
}


static inline int flicker_value_v4l2_to_tuning(int val)
{
	int ret = 0;
	switch(val){
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		ret = 0;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		ret = 50;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		ret = 60;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static inline int apical_isp_flicker_s_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;

	ret = tisp_s_antiflick(&core->core_tuning, flicker_value_v4l2_to_tuning(isp->ctrls.flicker));
	if(ret != 0)
		dev_err(isp->dev, "set flicker failed!!!\n");

	return ret;
}

static inline int apical_isp_flicker_g_control(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;

	return ret;
}

static int apical_isp_module_s_attr(tisp_core_t *core)
{
	tisp_module_control_t module;
	struct isp_device *isp = core->priv_data;
	module.key = isp->ctrls.module;

	tisp_s_module_control(&core->core_tuning, module);

	return 0;
}

static int apical_isp_module_g_attr(tisp_core_t *core)
{
	tisp_module_control_t module;
	struct isp_device *isp = core->priv_data;
	int ret = 0;

	tisp_g_module_control(&core->core_tuning, &module);
	isp->ctrls.module = module.key;

	return ret;
}

static inline int apical_isp_day_or_night_s_ctrl(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	int ret = 0;
	TISP_MODE_DN_E dn = isp->ctrls.daynight;

	tisp_day_or_night_s_ctrl(&core->core_tuning, dn);

	return ret;
}

static inline int apical_isp_day_or_night_g_ctrl(tisp_core_t *core)
{
	struct isp_device *isp = core->priv_data;
	TISP_MODE_DN_E dn ;

	dn = tisp_day_or_night_g_ctrl(&core->core_tuning);
	isp->ctrls.daynight = dn;

	return 0;
}


static int apical_isp_ae_luma_g_ctrl(tisp_core_t *core)
{
	unsigned char luma;
	struct isp_device *isp = core->priv_data;
	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];
	int width = input_fmt->format.width;
	int height = input_fmt->format.height;

	tisp_g_ae_luma(&core->core_tuning, &luma, width, height);
	isp->ctrls.luma = luma;

	return 0;
}

int isp_core_tuning_param_sync(tisp_core_t *core)
{
	int ret = 0;
	struct isp_device *isp = core->priv_data;
	struct v4l2_control *control;
	control = kzalloc(sizeof(struct v4l2_control), GFP_KERNEL);

	if (isp->ctrls.hflip_pending) {
		ret = apical_isp_hflip_s_control(core);
		isp->ctrls.hflip_pending = 0;
	}
	if (isp->ctrls.vflip_pending) {
		ret = apical_isp_vflip_s_control(core);
		isp->ctrls.vflip_pending = 0;
	}
	if (isp->ctrls.sat_pending) {
		ret = apical_isp_sat_s_control(core);
		isp->ctrls.sat_pending = 0;
	}
	if (isp->ctrls.bright_pending) {
		ret = apical_isp_bright_s_control(core);
		isp->ctrls.bright_pending = 0;
	}
	if (isp->ctrls.contrast_pending) {
		ret = apical_isp_contrast_s_control(core);
		isp->ctrls.contrast_pending = 0;
	}
	if (isp->ctrls.sharp_pending) {
		ret = apical_isp_sharp_s_control(core);
		isp->ctrls.sharp_pending = 0;
	}
	if (isp->ctrls.exp_auto_pending) {
		ret = ispcore_exp_auto_s_control(core);
		isp->ctrls.exp_auto_pending = 0;
	}
	if (isp->ctrls.exp_abs_pending) {
		ret = ispcore_exp_abs_s_control(core);
		isp->ctrls.exp_abs_pending = 0;
	}
	if (isp->ctrls.auto_gain_pending) {
		ret = ispcore_auto_gain_s_control(core);
		isp->ctrls.auto_gain_pending = 0;
	}
	if (isp->ctrls.gain_pending) {
		ret = ispcore_gain_s_control(core);
		isp->ctrls.gain_pending = 0;
	}
	if (isp->ctrls.wb_pending) {
		ret = ispcore_auto_n_wb_s_control(core);
		isp->ctrls.wb_pending = 0;
	}
	if (isp->ctrls.auto_wb_pending) {
		ret = ispcore_auto_wb_s_control(core);
		isp->ctrls.auto_wb_pending = 0;
	}
	if (isp->ctrls.red_wb_pending) {
		ret = ispcore_red_wb_s_control(core);
		isp->ctrls.red_wb_pending = 0;
	}
	if (isp->ctrls.blue_wb_pending) {
		ret = ispcore_blue_wb_s_control(core);
		isp->ctrls.blue_wb_pending = 0;
	}
	if (isp->ctrls.do_wb_pending) {
		ret = ispcore_do_wb_s_control(core);
		isp->ctrls.do_wb_pending = 0;
	}
	if (isp->ctrls.hill_pending) {
		ret = apical_isp_hilightdepress_s_control(core);
		isp->ctrls.hill_pending = 0;
	}
	if (isp->ctrls.flicker_pending) {
		ret = apical_isp_flicker_s_control(core);
		isp->ctrls.flicker_pending = 0;
	}
	if (isp->ctrls.module_pending) {
		ret = apical_isp_module_s_attr(core);
		isp->ctrls.module_pending = 0;
	}
	if (isp->ctrls.daynight_pending) {
		ret = apical_isp_day_or_night_s_ctrl(core);
		isp->ctrls.daynight_pending = 0;
	}
	return ret;
}


static int isp_video_g_ctrl_inited(struct isp_device *isp, struct v4l2_control *a)
{
	tisp_core_t *core = &isp->core;
	int ret = 0;

	switch (a->id) {
	case V4L2_CID_HFLIP:
		ret = apical_isp_hflip_g_control(core);
		a->value = isp->ctrls.hflip;
		break;
	case V4L2_CID_VFLIP:
		ret = apical_isp_vflip_g_control(core);
		a->value = isp->ctrls.vflip;
		break;
	case V4L2_CID_SATURATION:
		ret = apical_isp_sat_g_control(core);
		a->value = isp->ctrls.sat;
		break;
	case V4L2_CID_BRIGHTNESS:
		ret = apical_isp_bright_g_control(core);
		a->value = isp->ctrls.bright;
		break;
	case V4L2_CID_CONTRAST:
		ret = apical_isp_contrast_g_control(core);
		a->value = isp->ctrls.contrast;
		break;
	case V4L2_CID_SHARPNESS:
		ret = apical_isp_sharp_g_control(core);
		a->value = isp->ctrls.sharp;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ispcore_exp_auto_g_control(core);
		a->value = isp->ctrls.exp_auto;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		ret = ispcore_exp_abs_g_control(core);
		a->value = isp->ctrls.exp_abs;
		break;
	case V4L2_CID_EXPOSURE:
		ret = ispcore_exp_g_control(core);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = ispcore_auto_gain_g_control(core);
		a->value = isp->ctrls.auto_gain;
		break;
	case V4L2_CID_GAIN:
		ret = ispcore_gain_g_control(core);
		a->value = isp->ctrls.gain;
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ret = ispcore_auto_n_wb_g_control(core);
		a->value = isp->ctrls.wb;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ispcore_auto_wb_g_control(core);
		a->value = isp->ctrls.auto_wb;
		break;
	case V4L2_CID_RED_BALANCE:
		ret = ispcore_red_wb_g_control(core);
		a->value = isp->ctrls.red_wb;
		break;
	case V4L2_CID_BLUE_BALANCE:
		ret = ispcore_blue_wb_g_control(core);
		a->value = isp->ctrls.blue_wb;
		break;
	case IMAGE_TUNING_CID_HILIGHTDEPRESS:
		ret = apical_isp_hilightdepress_g_control(core);
		a->value = isp->ctrls.hill;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = apical_isp_flicker_g_control(core);
		a->value = isp->ctrls.flicker;
		break;
	case IMAGE_TUNING_CID_MODULE_CONTROL:
		ret = apical_isp_module_g_attr(core);
		a->value = isp->ctrls.module;
		break;
	case IMAGE_TUNING_CID_DAY_OR_NIGHT:
		ret = apical_isp_day_or_night_g_ctrl(core);
		a->value = isp->ctrls.daynight;
		break;
	case IMAGE_TUNING_CID_AE_LUMA:
		ret = apical_isp_ae_luma_g_ctrl(core);
		a->value = isp->ctrls.luma;
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}


static int isp_video_g_ctrl_noinited(struct isp_device *isp, struct v4l2_control *a)
{
	int ret = 0;

	switch (a->id) {
	case V4L2_CID_HFLIP:
		a->value = isp->ctrls.hflip;
		break;
	case V4L2_CID_VFLIP:
		a->value = isp->ctrls.vflip;
		break;
	case V4L2_CID_SATURATION:
		a->value = isp->ctrls.sat;
		break;
	case V4L2_CID_BRIGHTNESS:
		a->value = isp->ctrls.bright;
		break;
	case V4L2_CID_CONTRAST:
		a->value = isp->ctrls.contrast;
		break;
	case V4L2_CID_SHARPNESS:
		a->value = isp->ctrls.sharp;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		a->value = isp->ctrls.exp_auto;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		a->value = isp->ctrls.exp_abs;
		break;
	case V4L2_CID_AUTOGAIN:
		a->value = isp->ctrls.auto_gain;
		break;
	case V4L2_CID_GAIN:
		a->value = isp->ctrls.gain;
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		a->value = isp->ctrls.wb;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		a->value = isp->ctrls.auto_wb;
		break;
	case V4L2_CID_RED_BALANCE:
		a->value = isp->ctrls.red_wb;
		break;
	case V4L2_CID_BLUE_BALANCE:
		a->value = isp->ctrls.blue_wb;
		break;
	case IMAGE_TUNING_CID_HILIGHTDEPRESS:
		a->value = isp->ctrls.hill;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		a->value = isp->ctrls.flicker;
		break;
	case IMAGE_TUNING_CID_MODULE_CONTROL:
		a->value = isp->ctrls.module;
		break;
	case IMAGE_TUNING_CID_DAY_OR_NIGHT:
		a->value = isp->ctrls.daynight;
		break;
	case IMAGE_TUNING_CID_AE_LUMA:
		a->value = isp->ctrls.luma;
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}


int isp_video_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device 	*ispcam = ispvideo->ispcam;
	struct isp_device 	*isp	= ispcam->isp;
	int ret = 0;

	if (isp->core_inited) {
		ret = isp_video_g_ctrl_inited(isp, a);
	} else {
		ret = isp_video_g_ctrl_noinited(isp, a);
	}

	return ret;
}

static int isp_video_s_ctrl_inited(struct isp_device *isp, struct v4l2_control *a)
{
	tisp_core_t *core = &isp->core;
	int ret = 0;

	switch (a->id) {
	case V4L2_CID_HFLIP:
		isp->ctrls.hflip = a->value;
		ret = apical_isp_hflip_s_control(core);
		break;
	case V4L2_CID_VFLIP:
		isp->ctrls.vflip = a->value;
		ret = apical_isp_vflip_s_control(core);
		break;
	case V4L2_CID_SATURATION:
		isp->ctrls.sat = a->value;
		ret = apical_isp_sat_s_control(core);
		break;
	case V4L2_CID_BRIGHTNESS:
		isp->ctrls.bright = a->value;
		ret = apical_isp_bright_s_control(core);
		break;
	case V4L2_CID_CONTRAST:
		isp->ctrls.contrast = a->value;
		ret = apical_isp_contrast_s_control(core);
		break;
	case V4L2_CID_SHARPNESS:
		isp->ctrls.sharp = a->value;
		ret = apical_isp_sharp_s_control(core);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		isp->ctrls.exp_auto = a->value;
		ret = ispcore_exp_auto_s_control(core);
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		isp->ctrls.exp_abs = a->value;
		ret = ispcore_exp_abs_s_control(core);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ispcore_exp_s_control(core);
		break;
	case V4L2_CID_AUTOGAIN:
		isp->ctrls.auto_gain = a->value;
		ret = ispcore_auto_gain_s_control(core);
		break;
	case V4L2_CID_GAIN:
		isp->ctrls.gain = a->value;
		ret = ispcore_gain_s_control(core);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		isp->ctrls.wb    = a->value;
		ret = ispcore_auto_n_wb_s_control(core);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		isp->ctrls.auto_wb    = a->value;
		ret = ispcore_auto_wb_s_control(core);
		break;
	case V4L2_CID_RED_BALANCE:
		isp->ctrls.red_wb    = a->value;
		ret = ispcore_red_wb_s_control(core);
		break;
	case V4L2_CID_BLUE_BALANCE:
		isp->ctrls.blue_wb    = a->value;
		ret = ispcore_blue_wb_s_control(core);
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		ret = ispcore_do_wb_s_control(core);
		break;
	case IMAGE_TUNING_CID_HILIGHTDEPRESS:
		isp->ctrls.hill = a->value;
		ret = apical_isp_hilightdepress_s_control(core);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		isp->ctrls.flicker = a->value;
		ret = apical_isp_flicker_s_control(core);
		break;
	case IMAGE_TUNING_CID_MODULE_CONTROL:
		isp->ctrls.module = a->value;
		ret = apical_isp_module_s_attr(core);
		break;
	case IMAGE_TUNING_CID_DAY_OR_NIGHT:
		isp->ctrls.daynight = a->value;
		ret = apical_isp_day_or_night_s_ctrl(core);
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}

static int isp_video_s_ctrl_noinited(struct isp_device *isp, struct v4l2_control *a)
{
	int ret = 0;

	switch (a->id) {
	case V4L2_CID_HFLIP:
		isp->ctrls.hflip = a->value;
		isp->ctrls.hflip_pending = 1;
		break;
	case V4L2_CID_VFLIP:
		isp->ctrls.vflip = a->value;
		isp->ctrls.vflip_pending = 1;
		break;
	case V4L2_CID_SATURATION:
		isp->ctrls.sat = a->value;
		isp->ctrls.sat_pending = 1;
		break;
	case V4L2_CID_BRIGHTNESS:
		isp->ctrls.bright = a->value;
		isp->ctrls.bright_pending = 1;
		break;
	case V4L2_CID_CONTRAST:
		isp->ctrls.contrast = a->value;
		isp->ctrls.contrast_pending = 1;
		break;
	case V4L2_CID_SHARPNESS:
		isp->ctrls.sharp = a->value;
		isp->ctrls.sharp_pending = 1;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		isp->ctrls.exp_auto = a->value;
		isp->ctrls.exp_auto_pending = 1;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		isp->ctrls.exp_abs = a->value;
		isp->ctrls.exp_abs_pending = 1;
		break;
	case V4L2_CID_AUTOGAIN:
		isp->ctrls.auto_gain = a->value;
		isp->ctrls.auto_gain_pending = 1;
		break;
	case V4L2_CID_GAIN:
		isp->ctrls.gain = a->value;
		isp->ctrls.gain_pending = 1;
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		isp->ctrls.wb    = a->value;
		isp->ctrls.wb_pending = 1;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		isp->ctrls.auto_wb    = a->value;
		isp->ctrls.auto_wb_pending = 1;
		break;
	case V4L2_CID_RED_BALANCE:
		isp->ctrls.red_wb    = a->value;
		isp->ctrls.red_wb_pending = 1;
		break;
	case V4L2_CID_BLUE_BALANCE:
		isp->ctrls.blue_wb    = a->value;
		isp->ctrls.blue_wb_pending = 1;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		isp->ctrls.do_wb_pending = 1;
		break;
	case IMAGE_TUNING_CID_HILIGHTDEPRESS:
		isp->ctrls.hill = a->value;
		isp->ctrls.hill_pending = 1;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		isp->ctrls.flicker = a->value;
		isp->ctrls.flicker_pending = 1;
		break;
	case IMAGE_TUNING_CID_MODULE_CONTROL:
		isp->ctrls.module = a->value;
		isp->ctrls.module_pending = 1;
		break;
	case IMAGE_TUNING_CID_DAY_OR_NIGHT:
		isp->ctrls.daynight = a->value;
		isp->ctrls.daynight_pending = 1;
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}

int isp_video_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device 	*ispcam = ispvideo->ispcam;
	struct isp_device 	*isp	= ispcam->isp;
	int ret = 0;

	if (isp->core_inited) {
		ret = isp_video_s_ctrl_inited(isp, a);
	} else {
		ret = isp_video_s_ctrl_noinited(isp, a);
	}
	return ret;
}

