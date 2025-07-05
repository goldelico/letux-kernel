/*
 * A V4L2 driver for OmniVision OV2735a cameras.
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


#define OV2735A_CHIP_ID_H	(0x27)
#define OV2735A_CHIP_ID_L	(0x35)
#define OV2735A_REG_END		0xff
#define OV2735A_REG_DELAY	0x00
#define OV2735A_PAGE_REG	    0xfd


static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ov2735a_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct ov2735a_gpio {
	int pin;
	int active_level;
};

struct ov2735a_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct ov2735a_win_size *win;

	struct ov2735a_gpio reset;
	struct ov2735a_gpio ircutp;
	struct ov2735a_gpio ircutn;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};

struct again_lut ov2735a_again_lut[] = {
	{0x10,	0},
	{0x11,	5731},
	{0x12,	11136},
	{0x13,	16248},
	{0x14,	21097},
	{0x15,	25710},
	{0x16,	30109},
	{0x17,	34312},
	{0x18,	38336},
	{0x19,	42195},
	{0x1a,	45904},
	{0x1b,	49472},
	{0x1c,	52910},
	{0x1d,	56228},
	{0x1e,	59433},
	{0x1f,	62534},
	{0x20,	65536},
	{0x22,	71267},
	{0x24,	76672},
	{0x26,	81784},
	{0x28,	86633},
	{0x2a,	91246},
	{0x2c,	95645},
	{0x2e,	99848},
	{0x30,	103872},
	{0x32,	107731},
	{0x34,	111440},
	{0x36,	115008},
	{0x38,	118446},
	{0x3a,	121764},
	{0x3c,	124969},
	{0x3e,	128070},
	{0x40,	131072},
	{0x44,	136803},
	{0x48,	142208},
	{0x4c,	147320},
	{0x50,	152169},
	{0x54,	156782},
	{0x58,	161181},
	{0x5c,	165384},
	{0x60,	169408},
	{0x64,	173267},
	{0x68,	176976},
	{0x6c,	180544},
	{0x70,	183982},
	{0x74,	187300},
	{0x78,	190505},
	{0x7c,	193606},
	{0x80,	196608},
	{0x88,	202339},
	{0x90,	207744},
	{0x98,	212856},
	{0xa0,	217705},
	{0xa8,	222318},
	{0xb0,	226717},
	{0xb8,	230920},
	{0xc0,	234944},
	{0xc8,	238803},
	{0xd0,	242512},
	{0xd8,	246080},
	{0xe0,	249518},
	{0xe8,	252836},
	{0xf0,	256041},
	{0xf8,	259142},
};

static inline struct ov2735a_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2735a_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov2735a_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list ov2735a_init_regs_1920_1080_30fps_MIPI[] = {
	{0xfd, 0x00},
	{OV2735A_REG_DELAY, 0x05},
	{0xfd, 0x00},
	{0x2f, 0x10},
	{0x34, 0x00},
	{0x30, 0x15},
	{0x33, 0x01},
	{0x35, 0x20},
	{0xfd, 0x01},
	{0x0d, 0x00},
	{0x30, 0x00},
	{0x03, 0x01},
	{0x04, 0x8f},
	{0x01, 0x01},
	{0x09, 0x00},
	{0x0a, 0x20},
	{0x06, 0x0a},
	{0x24, 0x10},
	{0x01, 0x01},
	{0xfb, 0x73},
	{0x01, 0x01},
	{0xfd, 0x01},
	{0x1a, 0x6b},
	{0x1c, 0xea},
	{0x16, 0x0c},
	{0x21, 0x00},
	{0x11, 0xe8},//;63	;fr
	{0x19, 0xc3},
	{0x26, 0xda},
	{0x29, 0x01},
	{0x33, 0x6f},
	{0x2a, 0xd2},
	{0x2c, 0x40},
	{0xd0, 0x02},
	{0xd1, 0x01},
	{0xd2, 0x20},
	{0xd3, 0x03},//;04	;
	{0xd4, 0xa4},//;2a	;
	{0x50, 0x00},
	{0x51, 0x2c},
	{0x52, 0x29},
	{0x53, 0x00},
	{0x55, 0x44},
	{0x58, 0x29},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5d, 0x00},
	{0x64, 0x2f},
	{0x66, 0x62},
	{0x68, 0x5b},
	{0x75, 0x46},
	{0x76, 0xf0},
	{0x77, 0x4f},
	{0x78, 0xef},
	{0x72, 0xcf},
	{0x73, 0x36},
	{0x7d, 0x0d},
	{0x7e, 0x0d},
	{0x8a, 0x77},
	{0x8b, 0x77},
	{0xfd, 0x01},
	{0xb1, 0x83},//;DPHY enable 8b

//	{0xb2, 0x40},//hs mode

	{0xb3, 0x0b},//;0b;09;1d
	{0xb4, 0x14},//;MIPI PLL enable;14;35;36
	{0x9d, 0x40},//;mipi hs dc level 40/03/55
	{0xa1, 0x05},//;speed/03
	{0x94, 0x44},//;dphy time
	{0x95, 0x33},//;dphy time
	{0x96, 0x1f},//;dphy time
	{0x98, 0x45},//;dphy time
	{0x9c, 0x10},//;dphy time
	{0xb5, 0x70},//;30
	{0xa0, 0x01},//;mipi enable
	{0x25, 0xe0},
	{0x20, 0x7b},
	{0x8f, 0x88},
	{0x91, 0x40},

	{0xfd, 0x02},
	{0x5e, 0x03},
	{0xa1, 0x04},
	{0xa3, 0x40},
	{0xa5, 0x02},
	{0xa7, 0xc4},
	{0xfd, 0x01},
	{0x86, 0x77},
	{0x89, 0x77},
	{0x87, 0x74},
	{0x88, 0x74},
	{0xfc, 0xe0},
	{0xfe, 0xe0},
	{0xf0, 0x40},
	{0xf1, 0x40},
	{0xf2, 0x40},
	{0xf3, 0x40},

#if 1
	{0xfd, 0x02},//;;crop to 1920x1080
	{0xa0, 0x00},//;Image vertical start MSB3bits
	{0xa1, 0x08},//;Image vertical start LSB8bits
	{0xa2, 0x04},//;image vertical size  MSB8bits
	{0xa3, 0x38},//;image vertical size  LSB8bits
	{0xa4, 0x00},
	{0xa5, 0x04},//;H start 8Lsb, keep center
	{0xa6, 0x03},
	{0xa7, 0xc0},//;Half H size Lsb8bits
	{0xfd, 0x01},
	{0x8e, 0x07},
	{0x8f, 0x80},//;MIPI column number
	{0x90, 0x04},//;MIPI row number
	{0x91, 0x38},

	{0xfd, 0x03},
	{0xc0, 0x01},//;enable transfer OTP BP information
	{0xfd, 0x04},
	{0x21, 0x14},
	{0x22, 0x14},
	{0x23, 0x14},//;enhance normal and dummy BPC

	{0xfd, 0x01},
	{0x06, 0xe0},//;insert dummy line , the frame rate is 30.01.
//	{0x01, 0x01},//;
//	{0xa0, 0x01},//;MIPI enable, stream on
#else
	{0xfd, 0x02},//;;crop to 1920x1080
	{0xa0, 0x00},//;Image vertical start MSB3bits
	{0xa1, 0x08},//;Image vertical start LSB8bits
	{0xa2, 0x01},//;image vertical size  MSB8bits
	{0xa3, 0xe0},//;image vertical size  LSB8bits
	{0xa4, 0x00},
	{0xa5, 0x04},//;H start 8Lsb, keep center
	{0xa6, 0x01},
	{0xa7, 0x40},//;Half H size Lsb8bits
	{0xfd, 0x01},
	{0x8e, 0x02},
	{0x8f, 0x80},//;MIPI column number
	{0x90, 0x01},//;MIPI row number
	{0x91, 0xe0},

	{0xfd, 0x03},
	{0xc0, 0x01},//;enable transfer OTP BP information
	{0xfd, 0x04},
	{0x21, 0x14},
	{0x22, 0x14},
	{0x23, 0x14},//;enhance normal and dummy BPC

	{0xfd, 0x01},
	{0x06, 0xe0},//;insert dummy line , the frame rate is 30.01.
	{0x01, 0x01},//;
	{0xa0, 0x01},//;MIPI enable, stream on
#endif

#if 0
	{0xfd, 0x01},//@@ 1 11 Mirror_On_Flip_Off

	{0x3f, 0x01},//03
	{0xf8, 0x00},//02
	{0x01, 0x01},
	{0xfd, 0x02},
	{0x62, 0x01},//;full size mode flip off row start for OTP BPC
	{0x63, 0x00},
	{0xfd, 0x01},

#endif
#if 0
	{0xfd, 0x01},//@@ 1 12 Mirror_On_Flip_On

	{0x3f, 0x03},// 03

	{0xf8, 0x02},//02

	{0x01, 0x01},
	{0xfd, 0x02},
	{0x62, 0x48},
	{0x63, 0x04},//;full size flip, row start:1096(0x448)
	{0xfd, 0x01},
#endif

#if 0
	{0xfd, 0x01},//@@ 1 13 Mirror_Off_Flip_Off

	{0x3f, 0x00},//03
	{0xf8, 0x00},// 02

	{0x01, 0x01},
	{0xfd, 0x02},
	{0x62, 0x01},//;full size mode flip off row start for OTP BPC
	{0x63, 0x00},
	{0xfd, 0x01},
#endif

#if 1
	{0xfd, 0x01},//@@ 1 14 Mirror_Off_Flip_On

	{0x3f, 0x02},//03
	{0xf8, 0x02},//02
	{0x01, 0x01},

	{0xfd, 0x02},
	{0x62, 0x48},
	{0x63, 0x04},//;full size flip, row start:1096(0x448)
	{0xfd, 0x01},
#endif

	{0x01, 0x01},//;
	{0xa0, 0x01},//;MIPI enable, stream on
	{OV2735A_REG_END, 0x00},	/* END MARKER */
};


static struct regval_list ov2735a_stream_on[] = {
	{0xfd, 0x00},
	{0x36, 0x00},
	{0x37, 0x00},//fake stream on
	{OV2735A_REG_END, 0x00},
};

static struct regval_list ov2735a_stream_off[] = {
	{0xfd, 0x00},
	{0x36, 0x01},
	{0x37, 0x01},//fake stream off
	{OV2735A_REG_END, 0x00},
};


int ov2735a_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ov2735a_write(struct v4l2_subdev *sd, unsigned char reg,
			unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= buf,
	};
	int ret;
	unsigned int timeout  = 100;
	while(timeout--){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == -EAGAIN){
			msleep(100);
			continue;
		}
		else
			break;
	}
	if(ret > 0)
		ret = 0;
	return ret;
}

static int ov2735a_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != OV2735A_REG_END) {
		if (vals->reg_num == OV2735A_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = ov2735a_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
			if (vals->reg_num == OV2735A_PAGE_REG){
				val &= 0xf8;
				val |= (vals->value & 0x07);
				ret = ov2735a_write(sd, vals->reg_num, val);
				ret = ov2735a_read(sd, vals->reg_num, &val);
			}
		}
		vals++;
	}
	return 0;
}
static int ov2735a_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != OV2735A_REG_END) {
		if (vals->reg_num == OV2735A_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = ov2735a_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static int ov2735a_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735a_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int ov2735a_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735a_info *info = to_state(sd);

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

static int ov2735a_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735a_info *info = to_state(sd);
	int ret = 0;

	ret = ov2735a_write_array(sd, info->win->regs);

	return ret;
}



static int ov2735a_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	ret = ov2735a_read(sd, 0x02, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV2735A_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = ov2735a_read(sd, 0x03, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV2735A_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	ret = ov2735a_read(sd, 0x04, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != 0x05 && v != 0x06 && v != 0x07)
		return -ENODEV;


	return 0;

}

static struct ov2735a_win_size ov2735a_win_sizes[] = {
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 30 << 16 | 1,
		.sensor_info.total_width			= 1936,
		.sensor_info.total_height			= 1096,
		.sensor_info.wdr_en				= 0,
		.sensor_info.mipi_cfg.clk			= 600,
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_mode		= SENSOR_MIPI_OTHER_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en		= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
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
		.sensor_info.mipi_cfg.data_type_value	= RAW10,
		.sensor_info.mipi_cfg.del_start		= 0,
		.sensor_info.mipi_cfg.sensor_frame_mode	= TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode	= 0,
		.sensor_info.mipi_cfg.sensor_mode		= TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt		= TX_SENSOR_RAW10,
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ov2735a_init_regs_1920_1080_30fps_MIPI,
	},
};

static int ov2735a_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_OV2735a_FMTS)
		return -EINVAL;

	code->code = ov2735a_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int ov2735a_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ov2735a_format_struct *ovfmt;
	struct ov2735a_win_size *wsize;
	struct ov2735a_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int ov2735a_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct ov2735a_info *info = to_state(sd);
	struct ov2735a_win_size *wsize = info->win;
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
	*(unsigned int *)fmt->reserved = &wsize->sensor_info; /*reserved[0] reserved[1]*/

//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int ov2735a_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735a_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735a_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735a_s_vflip(struct v4l2_subdev *sd, int value)
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
static int ov2735a_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ov2735a_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

//	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ov2735a_again_lut); i++) {
		lut = &ov2735a_again_lut[i];

		if(gain <= lut->gain) {
			return lut->value;
		}
	}

	/*last value.*/
	return lut->value;
}

static int regval_to_again(unsigned int regval)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ov2735a_again_lut); i++) {
		lut = &ov2735a_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int ov2735a_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;


	ret = ov2735a_read(sd, 0x23, &v);
	reg_val |= v ;

	*value = regval_to_again(reg_val);

	return ret;

}
/*set analog gain db value, map value to sensor register.*/
static int ov2735a_s_again(struct v4l2_subdev *sd, int value)
{
	struct ov2735a_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret = ov2735a_write(sd, 0xfd, 0x01);
	ret += ov2735a_write(sd, 0x24, (unsigned char)reg_value);
	ret += ov2735a_write(sd, 0x01, 0x01);
	if (ret < 0){
		printk("ov2735a_write error  %d\n" ,__LINE__ );
		return ret;
	}
	return 0;
}

/*
 * Tweak autogain.
 */
static int ov2735a_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735a_s_exp(struct v4l2_subdev *sd, int value)
{
	struct ov2735a_info *info = to_state(sd);
	int ret = 0;

	ret = ov2735a_write(sd, 0xfd, 0x01);
	ret += ov2735a_write(sd, 0x4, (unsigned char)(value & 0xff));
	ret += ov2735a_write(sd, 0x3, (unsigned char)((value & 0xff00) >> 8));
	ret += ov2735a_write(sd, 0x01, 0x01);

	if (ret < 0) {
		printk("ov2735a_write error  %d\n" ,__LINE__);
		return ret;
	}
	return ret;
}

static int ov2735a_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov2735a_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ov2735a_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov2735a_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ov2735a_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov2735a_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov2735a_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov2735a_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ov2735a_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov2735a_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ov2735a_s_gain turns off auto gain */
			return ov2735a_s_gain(sd, info->gain->val);
		}
		return ov2735a_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ov2735a_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov2735a_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov2735a_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov2735a_ctrl_ops = {
	.s_ctrl = ov2735a_s_ctrl,
	.g_volatile_ctrl = ov2735a_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2735a_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ov2735a_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov2735a_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ov2735a_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int ov2735a_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2735a_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ov2735a_reset(sd, 1);
		ov2735a_write_array(sd, info->win->regs);
		ret = ov2735a_write_array(sd, ov2735a_stream_on);
		printk("ov2735a stream on\n");

	}
	else {
		ret = ov2735a_write_array(sd, ov2735a_stream_off);
		printk("ov2735a stream off\n");
	}
	return ret;
}


int ov2735a_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ov2735a_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov2735a_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2735a_g_register,
	.s_register = ov2735a_s_register,
#endif

};

static const struct v4l2_subdev_video_ops ov2735a_video_ops = {
	.s_stream = ov2735a_s_stream,
	.g_frame_interval	= ov2735a_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov2735a_pad_ops = {
	//.enum_frame_interval = ov2735a_enum_frame_interval,
	//.num_frame_size = ov2735a_enum_frame_size,
	//.enum_mbus_code = ov2735a_enum_mbus_code,
	.set_fmt = ov2735a_set_fmt,
	.get_fmt = ov2735a_get_fmt,
};

static const struct v4l2_subdev_ops ov2735a_ops = {
	.core = &ov2735a_core_ops,
	.video = &ov2735a_video_ops,
	.pad = &ov2735a_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ov2735a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ov2735a_info *info;
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

	v4l2_i2c_subdev_init(sd, client, &ov2735a_ops);
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
	if (((rate / 1000) % 24000) != 0) {
		ret = clk_set_parent(info->sclka, clk_get(NULL, "epll"));
		info->sclka = devm_clk_get(&client->dev, "epll");
		if (IS_ERR(info->sclka)) {
			pr_err("get sclka failed\n");
		} else {
			rate = clk_get_rate(info->sclka);
			if (((rate / 1000) % 24000) != 0) {
				clk_set_rate(info->sclka, 120000000);
			}
		}
	}

	ret = v4l2_clk_set_rate(info->clk, 24000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");
#endif

	info->win = &ov2735a_win_sizes[0];
	ov2735a_reset(sd, 1);
#if 1
	/* Make sure it's an ov2735a */
	ret = ov2735a_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an ov2735a chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif
	/*IRCUT ctl 0:off 1:on*/
	ov2735a_ircut(sd, 0);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ov2735a_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

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

	dev_info(&client->dev, "ov2735a Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int ov2735a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2735a_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id ov2735a_id[] = {
	{ "ov2735a", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2735a_id);

static const struct of_device_id ov2735a_of_match[] = {
	{.compatible = "ovti,ov2735a", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2735a_of_match);


static struct i2c_driver ov2735a_driver = {
	.driver = {
		.name	= "ov2735a",
		.of_match_table = of_match_ptr(ov2735a_of_match),
	},
	.probe		= ov2735a_probe,
	.remove		= ov2735a_remove,
	.id_table	= ov2735a_id,
};

module_i2c_driver(ov2735a_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov2735a sensors");
MODULE_LICENSE("GPL");
