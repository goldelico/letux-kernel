/*
 * A V4L2 driver for OmniVision IMX327 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include <isp-sensor.h>

#define IMX327_CHIP_ID_H	(0xb2)
#define IMX327_CHIP_ID_L	(0x01)
#define IMX327_REG_END		0xffff
#define IMX327_REG_DELAY	0xfffe

#define AGAIN_MAX_DB 0x64
#define DGAIN_MAX_DB 0x64
#define LOG2_GAIN_SHIFT 16

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");


struct imx327_win_size {
	unsigned int width;
	unsigned int height;
	struct sensor_info sensor_info;
	unsigned int mbus_code;
	enum v4l2_colorspace colorspace;
	void *regs;
};

struct imx327_gpio {
	int pin;
	int active_level;
};

struct imx327_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *again_short;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *exposure_short;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct imx327_win_size *win;

	struct imx327_gpio reset;
	struct imx327_gpio ircutp;
	struct imx327_gpio ircutn;
};


static inline struct imx327_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx327_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx327_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};


static struct regval_list imx327_init_regs_1920_1080_30fps_mipi[] = {
	{0x3000, 0x01},//standy
	{0x3001, 0x01},
	{0x3002, 0x01},//Master stop
//	{IMX327_REG_DELAY, 0x18},
	{0x3005, 0x01},
	{0x3007, 0x00},
	{0x3009, 0x02},
	{0x300A, 0xF0},
	{0x300B, 0x00},
	{0x3011, 0x0A},
	{0x3012, 0x64},
	{0x3014, 0x00},
	{0x3018, 0x65},// 1125
	{0x3019, 0x04},
	{0x301A, 0x00},
	{0x301C, 0xa0},//0x1130:30fps 0x14a0:25fps
	{0x301D, 0x14},
	{0x3022, 0x00},
	{0x3046, 0x01},
	{0x3048, 0x00},
	{0x3049, 0x08},
	{0x304B, 0x0A},
	{0x305C, 0x18},
	{0x305D, 0x03},
	{0x305E, 0x20},
	{0x305F, 0x01},
	{0x309E, 0x4A},
	{0x309F, 0x4A},
	{0x30D2, 0x19},
	{0x30D7, 0x03},
	{0x3129, 0x00},
	{0x313B, 0x61},
	{0x315E, 0x1A},
	{0x3164, 0x1A},
	{0x317C, 0x00},
	{0x31EC, 0x0E},
	{0x3405, 0x10},
	{0x3407, 0x01},
	{0x3414, 0x0A},
	{0x3418, 0x49},
	{0x3419, 0x04},
	{0x3441, 0x0C},
	{0x3442, 0x0C},
	{0x3443, 0x01},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x37},
	{0x3449, 0x00},
	{0x344A, 0x2F},//1f
	{0x344B, 0x00},
	{0x344C, 0x1F},
	{0x344D, 0x00},
	{0x344E, 0x1F},
	{0x344F, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},
	{0x3452, 0x1F},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
	{0x346a, 0x9c},//EBD
	{0x346b, 0x07},
	{0x3472, 0x9C},
	{0x3473, 0x07},
	{0x3480, 0x49},
//	{IMX327_REG_DELAY, 0x18},
	{0x3001, 0x00},//standy cancel
	{0x3002, 0x00},//Master start
	{0x3000, 0x01},//standy cancel
	{IMX327_REG_END, 0x00},/* END MARKER */
};


static struct regval_list imx327_init_regs_1920_1080_30fps_mipi_2dol_lcg[] = {
//add
	{0x3000, 0x01},//standy
	{0x3001, 0x01},
	{0x3002, 0x01},//Master stop
//	{IMX327_REG_DELAY, 0x18},
//	{0x3002, 0x00},
	{0x3005, 0x01},//ADBIT 01:12bit
	{0x3007, 0x40},//WINMODE[6:4]
	{0x3009, 0x01},//FRSEL[1:0]
	{0x300a, 0xf0},
	{0x300c, 0x11},//WDMODE[0] WDSEL[5:4]
	{0x3010, 0x61},//FPGC gain for each gain
	{0x30f0, 0x64},//FPGC1 gain for each gain
	{0x3011, 0x02},
	{0x3018, 0x6E},//Vmax FSC=vmax*2
	{0x3019, 0x05},//0x486
	{0x301c, 0x58},//Hmax
	{0x301d, 0x08},
#if 0
	{0x3018, 0x86},//Vmax FSC=vmax*2
	{0x3019, 0x04},//0x486  1158  60.04fps
	{0x301c, 0x05},//Hmax   2136 0x858
	{0x301d, 0x0a},
#endif
#if 1
	{0x3020, 0x02},//SHS1 S
	{0x3021, 0x00},
#else
//	{0x3014, 0x70},
	{0x3020, 0x02},//SHS1 S
	{0x3021, 0x00},
	{0x3022, 0x00},
#endif
	{0x3024, 0x73},//SHS2 L
	{0x3025, 0x04},
	{0x3028, 0x00},//SHS3
	{0x3029, 0x00},
	{0x302a, 0x00},
	{0x3030, 0x65},//RHS1
	{0x3031, 0x00},
	{0x3032, 0x00},
	{0x3034, 0x00},//RHS2
	{0x3035, 0x00},
	{0x3036, 0x00},
	{0x303c, 0x04},//WINPV[7:0]
	{0x303d, 0x00},//WINPV[2:0]
	{0x303e, 0x41},//WINWV[7:0]
	{0x303f, 0x04},//WINWV[2:0]
	{0x3045, 0x05},
	{0x3046, 0x01},//ODBIT[1:0]
	{0x304b, 0x0a},
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x309e, 0x4a},
	{0x309f, 0x4a},
	{0x30d2, 0x19},
	{0x30d7, 0x03},
	{0x3106, 0x11},
	{0x3129, 0x00},
	{0x313b, 0x61},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
#if 0
	{0x31a0, 0xbc},
	{0x31a1, 0x00},

#endif
	{0x317c, 0x00},
	{0x31ec, 0x0e},
	{0x3204, 0x4a},
	{0x3209, 0xf0},
	{0x320a, 0x22},
	{0x3344, 0x38},
	{0x3405, 0x00},
	{0x3407, 0x01},
	{0x3414, 0x00},//OPB_SIZE_V[5:0]
	{0x3415, 0x00},//NULL0_SIZE_V[5:0]
	{0x3418, 0x7a},//Y_OUT_SIZE[7:0]
	{0x3419, 0x09},//Y_OUT_SIZE[4:0]
	{0x3441, 0x0c},
	{0x3442, 0x0c},
	{0x3443, 0x01},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x77},
	{0x3447, 0x00},
	{0x3448, 0x67},
	{0x3449, 0x00},
	{0x344a, 0x47},
	{0x344b, 0x00},
	{0x344c, 0x37},
	{0x344d, 0x00},
	{0x344e, 0x3f},
	{0x344f, 0x00},
	{0x3450, 0xff},
	{0x3451, 0x00},
	{0x3452, 0x3f},
	{0x3453, 0x00},
	{0x3454, 0x37},
	{0x3455, 0x00},
	{0x346a, 0x9c},//EBD
	{0x346b, 0x07},
	{0x3472, 0xa0},
	{0x3473, 0x07},
	{0x347b, 0x23},
	{0x3480, 0x49},
//	{IMX327_REG_DELAY, 0x18},
	{0x3001, 0x00},//standy cancel
	{0x3002, 0x00},//Master start
	{0x3000, 0x01},//standy cancel
	{IMX327_REG_END, 0x00},/* END MARKER */

};



/*
 * the part of driver was fixed.
 */

static struct regval_list imx327_stream_on_mipi[] = {
	{0x3000, 0x00},
	{IMX327_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list imx327_stream_off_mipi[] = {
	{0x3000, 0x01},
	{IMX327_REG_END, 0x00},	/* END MARKER */
};

static int imx327_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct imx327_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {(reg>>8)&0xff, reg&0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int imx327_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char value)
{
	struct imx327_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3] = {(reg>>8)&0xff, reg&0xff, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int imx327_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != IMX327_REG_END) {
		if (vals->reg_num == IMX327_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = imx327_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int imx327_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != IMX327_REG_END) {
		if (vals->reg_num == IMX327_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = imx327_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}


static int imx327_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct imx327_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
	} else {
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, !info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
	}
	return 0;
}

static int imx327_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = imx327_read(sd, 0x301e, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != IMX327_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = imx327_read(sd, 0x301f, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != IMX327_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}

static struct imx327_win_size imx327_win_sizes[] = {
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.total_width			= 2136,
		.sensor_info.total_height			= 2781,
		.sensor_info.wdr_en				= 1,
		.sensor_info.mipi_cfg.clk			= 891,
		.sensor_info.mipi_cfg.twidth		= 1952,
		.sensor_info.mipi_cfg.theight		= 1109,
		.sensor_info.mipi_cfg.mipi_mode		= SENSOR_MIPI_SONY_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en		= 1,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 16,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 12,
		.sensor_info.mipi_cfg.mipi_crop_start1x	= 16,
		.sensor_info.mipi_cfg.mipi_crop_start1y	= 62,
		.sensor_info.mipi_cfg.mipi_crop_start2x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3y	= 0,
		.sensor_info.mipi_cfg.hcrop_diff_en		= 0,
		.sensor_info.mipi_cfg.line_sync_mode	= 0,
		.sensor_info.mipi_cfg.work_start_flag	= 0,
		.sensor_info.mipi_cfg.data_type_en		= 0,
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 1,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_WDR_2_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_NOT_VC_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= imx327_init_regs_1920_1080_30fps_mipi_2dol_lcg,
	},
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 25 << 16 | 1,
		.sensor_info.total_width			= 5280,
		.sensor_info.total_height			= 1125,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 445,
		.sensor_info.mipi_cfg.twidth		= 1948,
		.sensor_info.mipi_cfg.theight		= 1109,
		.sensor_info.mipi_cfg.mipi_mode		= SENSOR_MIPI_SONY_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 12,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 20,
		.sensor_info.mipi_cfg.mipi_crop_start1x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start1y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start2y	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start3y	= 0,
		.sensor_info.mipi_cfg.hcrop_diff_en		= 0,
		.sensor_info.mipi_cfg.line_sync_mode	= 0,
		.sensor_info.mipi_cfg.work_start_flag	= 0,
		.sensor_info.mipi_cfg.data_type_en		= 0,
		.sensor_info.mipi_cfg.data_type_value	= RAW12,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW12,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= imx327_init_regs_1920_1080_30fps_mipi,
	},
};

static int imx327_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_IMX327_FMTS)
		return -EINVAL;

	code->code = imx327_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int imx327_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct imx327_format_struct *ovfmt;
	struct imx327_win_size *wsize;
	struct imx327_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int imx327_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct imx327_info *info = to_state(sd);
	struct imx327_win_size *wsize = info->win;
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if(!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = wsize->width;
	fmt->height = wsize->height;
	fmt->code = wsize->mbus_code;
	fmt->colorspace = wsize->colorspace;
	*(unsigned int *)fmt->reserved = &wsize->sensor_info;

//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int imx327_s_wdr(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
	struct imx327_info *info = to_state(sd);
	printk("%s:%d\n", __func__, value);
	if(value)
		info->win = &imx327_win_sizes[0];
	else
		info->win = &imx327_win_sizes[1];

	return ret;
}

static int imx327_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx327_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx327_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx327_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int imx327_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int imx327_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	unsigned int again=(gain*20)>>LOG2_GAIN_SHIFT;
	// Limit Max gain
	if(again>AGAIN_MAX_DB+DGAIN_MAX_DB) again=AGAIN_MAX_DB+DGAIN_MAX_DB;
	return again;
}

static int regval_to_again(unsigned int regval)
{
	return (((int32_t)regval)<<LOG2_GAIN_SHIFT)/20;
}

static int imx327_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct imx327_info *info = to_state(sd);
	unsigned char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret += imx327_read(sd, 0x3014, &v);
	reg_val = v;

	*value = regval_to_again(reg_val);

	return ret;
}

static int imx327_g_again_short(struct v4l2_subdev *sd, __s32 *value)
{
	struct imx327_info *info = to_state(sd);
	unsigned char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;

	ret += imx327_read(sd, 0x30f2, &v);
	reg_val = v;

	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int imx327_s_again(struct v4l2_subdev *sd, int again)
{
	struct imx327_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

//	printk("again = %d\n", again);
	reg_value = again_to_regval(again);
	ret += imx327_write(sd, 0x3014, (unsigned char)(reg_value & 0xff));

	return ret;
}

static int imx327_s_again_short(struct v4l2_subdev *sd, int again)
{
	struct imx327_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	reg_value = again_to_regval(again);
	ret += imx327_write(sd, 0x30f2, (unsigned char)(reg_value & 0xff));


	return ret;
}

/*
 * Tweak autogain.
 */
static int imx327_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int imx327_s_exp(struct v4l2_subdev *sd, int value)
{
	struct imx327_info *info = to_state(sd);
	int ret = 0;
	int shs = 0;
	unsigned char val = 0;
	int total_height = 0;

//	printk("exp = %d\n", value);
	ret = imx327_read(sd, 0x3019, &val);
	total_height |= val << 8;
	ret = imx327_read(sd, 0x3018, &val);
	total_height |= val;

	if(info->win->sensor_info.wdr_en) {
		total_height *= 2;
		shs = total_height - value - 1;
		ret = imx327_write(sd, 0x3024, (unsigned char)(shs & 0xff));
		ret += imx327_write(sd, 0x3025, (unsigned char)((shs >> 8) & 0xff));
		ret += imx327_write(sd, 0x3026, (unsigned char)((shs >> 16) & 0x3));
	} else {
		shs = total_height - value - 1;
		ret = imx327_write(sd, 0x3020, (unsigned char)(shs & 0xff));
		ret += imx327_write(sd, 0x3021, (unsigned char)((shs >> 8) & 0xff));
		ret += imx327_write(sd, 0x3022, (unsigned char)((shs >> 16) & 0x3));
	}

	return ret;
}

static int imx327_s_exp_short(struct v4l2_subdev *sd, int value)
{
	struct imx327_info *info = to_state(sd);
	int ret = 0;

	int shs = 101 - value - 1;
	ret = imx327_write(sd, 0x3020, (unsigned char)(shs & 0xff));
	ret += imx327_write(sd, 0x3021, (unsigned char)((shs >> 8) & 0xff));
	ret += imx327_write(sd, 0x3022, (unsigned char)((shs >> 16) & 0x3));


	return ret;
}

static int imx327_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct imx327_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return imx327_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return imx327_g_again(sd, &info->again->val);
	case V4L2_CID_USER_ANALOG_GAIN_SHORT:
		return imx327_g_again_short(sd, &info->again_short->val);
	}
	return -EINVAL;
}

static int imx327_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct imx327_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return imx327_s_brightness(sd, ctrl->val);
		case V4L2_CID_CONTRAST:
			return imx327_s_contrast(sd, ctrl->val);
		case V4L2_CID_VFLIP:
			return imx327_s_vflip(sd, ctrl->val);
		case V4L2_CID_HFLIP:
			return imx327_s_hflip(sd, ctrl->val);
		case V4L2_CID_AUTOGAIN:
			/* Only set manual gain if auto gain is not explicitly
			   turned on. */
			if (!ctrl->val) {
				/* imx327_s_gain turns off auto gain */
				return imx327_s_gain(sd, info->gain->val);
			}
			return imx327_s_autogain(sd, ctrl->val);
		case V4L2_CID_GAIN:
			return imx327_s_gain(sd, ctrl->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return imx327_s_again(sd, ctrl->val);
		case V4L2_CID_USER_ANALOG_GAIN_SHORT:
			return imx327_s_again_short(sd, ctrl->val);
		case V4L2_CID_EXPOSURE:
			return imx327_s_exp(sd, ctrl->val);
		case V4L2_CID_USER_EXPOSURE_SHORT:
			return imx327_s_exp_short(sd, ctrl->val);
		case V4L2_CID_WIDE_DYNAMIC_RANGE:
			return imx327_s_wdr(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops imx327_ctrl_ops = {
	.s_ctrl = imx327_s_ctrl,
	.g_volatile_ctrl = imx327_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int imx327_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = imx327_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int imx327_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	imx327_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

static int imx327_core_init(struct v4l2_subdev *sd, u32 val)
{
	struct imx327_info *info = to_state(sd);
	int ret = 0;
	ret = imx327_write_array(sd, info->win->regs);
	return ret;
}

static int imx327_core_reset(struct v4l2_subdev *sd, u32 val)
{
	struct imx327_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}


int imx327_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx327_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ret = imx327_write_array(sd, imx327_stream_on_mipi);
		pr_debug("imx327 stream on\n");

	}
	else {
		ret = imx327_write_array(sd, imx327_stream_off_mipi);
		pr_debug("imx327 stream off\n");
	}
	return ret;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops imx327_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = imx327_g_register,
	.s_register = imx327_s_register,
#endif
	.init = imx327_core_init,
	.reset = imx327_core_reset,

};

static const struct v4l2_subdev_video_ops imx327_video_ops = {
	.s_stream = imx327_s_stream,
};

static const struct v4l2_subdev_pad_ops imx327_pad_ops = {
	//.enum_frame_interval = imx327_enum_frame_interval,
	//.num_frame_size = imx327_enum_frame_size,
	//.enum_mbus_code = imx327_enum_mbus_code,
	.set_fmt = imx327_set_fmt,
	.get_fmt = imx327_get_fmt,
};

static const struct v4l2_subdev_ops imx327_ops = {
	.core = &imx327_core_ops,
	.video = &imx327_video_ops,
	.pad = &imx327_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int imx327_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct imx327_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned long rate;
	struct v4l2_ctrl_config cfg = {0};
	int mclk_index = -1;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutp-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->ircutp.pin = gpio;
		info->ircutp.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,ircutn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->ircutn.pin = gpio;
		info->ircutn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}

	v4l2_i2c_subdev_init(sd, client, &imx327_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
#if 1

	char id_div[9];
	char id_mux[9];
	printk("%s:%d\n", __func__, __LINE__);
	of_property_read_u32(client->dev.of_node, "ingenic,mclk", &mclk_index);
	if(mclk_index == 0) {
		memcpy(id_div, "div_cim0", sizeof(id_div));
		memcpy(id_mux, "mux_cim0", sizeof(id_mux));
	} else if(mclk_index == 1) {
		memcpy(id_div, "div_cim1", sizeof(id_div));
		memcpy(id_mux, "mux_cim1", sizeof(id_mux));
	} else if(mclk_index == 2) {
		memcpy(id_div, "div_cim2", sizeof(id_div));
		memcpy(id_mux, "mux_cim2", sizeof(id_mux));
	} else
		printk("Unkonwn mclk index\n");


	info->clk = v4l2_clk_get(&client->dev, id_div);
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}
	info->sclka = devm_clk_get(&client->dev, id_mux);

	printk("sclka is %s\n", info->sclka?"NOT NULL":"NULL");
	rate = v4l2_clk_get_rate(info->clk);
	if (((rate / 1000) % 37125) != 0) {
		ret = clk_set_parent(info->sclka, clk_get(NULL, "epll"));
		info->sclka = devm_clk_get(&client->dev, "epll");
		if (IS_ERR(info->sclka)) {
			pr_err("get sclka failed\n");
		} else {
			rate = clk_get_rate(info->sclka);
			if (((rate / 1000) % 37125) != 0) {
				clk_set_rate(info->sclka, 891000000);
			}
		}
	}

	ret = v4l2_clk_set_rate(info->clk, 37125000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");
#endif


	imx327_core_reset(sd, 1);
#if 1
	/* Make sure it's an imx327 */
	ret = imx327_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an imx327 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	imx327_ircut(sd, 0);
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 10);
	v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 589824, 1, 10000);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &imx327_ctrl_ops,
			V4L2_CID_EXPOSURE, 1, 1176, 1, 1000);

	cfg.ops = &imx327_ctrl_ops;
	cfg.id = V4L2_CID_USER_EXPOSURE_SHORT;
	cfg.name = "expo short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 1;
	cfg.max = 98;
	cfg.step = 1;
	cfg.def = 50;
	info->exposure_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);

	memset(&cfg, 0, sizeof(cfg));
	cfg.ops = &imx327_ctrl_ops;
	cfg.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	cfg.name = "analog gain short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 0;
	cfg.max = 589824;
	cfg.step = 1;
	cfg.def = 10000;
	info->again_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "imx327 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int imx327_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx327_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id imx327_id[] = {
	{ "imx327", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx327_id);

static const struct of_device_id imx327_of_match[] = {
	{.compatible = "ovti,imx327", },
	{},
};
MODULE_DEVICE_TABLE(of, imx327_of_match);


static struct i2c_driver imx327_driver = {
	.driver = {
		.name	= "imx327",
		.of_match_table = of_match_ptr(imx327_of_match),
	},
	.probe		= imx327_probe,
	.remove		= imx327_remove,
	.id_table	= imx327_id,
};

module_i2c_driver(imx327_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision imx327 sensors");
MODULE_LICENSE("GPL");
