/*
 * ov2735b Camera Driver
 *
 * Copyright (C) 2010 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 *
 * Based on ov772x, ov9640 drivers and previous non merged implementations.
 *
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2006, OmniVision
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-device.h>


#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define OV2735B_CHIP_ID_H       (0x27)
#define OV2735B_CHIP_ID_L       (0x35)
#define OV2735B_REG_END         0xff
#define OV2735B_REG_DELAY       0x00
#define OV2735B_PAGE_REG            0xfd

#define  ov2735b_DEFAULT_WIDTH    640
#define  ov2735b_DEFAULT_HEIGHT   480

/* Private v4l2 controls */
#define V4L2_CID_PRIVATE_BALANCE  (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_EFFECT  (V4L2_CID_PRIVATE_BASE + 1)

#define ov2735b_FLIP_VAL			((unsigned char)0x04)
#define ov2735b_FLIP_MASK		((unsigned char)0x04)

/* whether sensor support high resolution (> vga) preview or not */
#define SUPPORT_HIGH_RESOLUTION_PRE		1

/*
 * Struct
 */
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

struct mode_list {
	u16 index;
	const struct regval_list *mode_regs;
};

struct ov2735b_win_size {
	char *name;
	int width;
	int height;
	const struct regval_list *regs;
	unsigned int mbus_code;
};


struct ov2735b_gpio {
	int pin;
	int active_level;
};

struct ov2735b_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct v4l2_clk			*clk;
	const struct ov2735b_win_size	*win;

	int				model;
	u16				balance_value;
	u16				effect_value;
	u16				flag_vflip:1;
	u16				flag_hflip:1;

//	struct soc_camera_subdev_desc	ssdd_dt;
	struct v4l2_subdev_platform_data *sd_pdata;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *vcc_en_gpio;
	int (*power)(struct i2c_client *client, int on);
	int (*reset)(struct i2c_client *client);

	struct regulator *reg;
	struct regulator *reg1;
};

unsigned short ov2735b_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
	struct i2c_msg msg[2] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = &reg,
		},
		[1] = {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

int ov2735b_write_reg(struct i2c_client *client, unsigned char reg, unsigned short value)
{
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 2,
		.buf    = buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;

}

static struct regval_list ov2735b_1080p_regs[] = {
	{0xfd, 0x00},
	{OV2735B_REG_DELAY, 0x05},
	{0xfd, 0x00},
	{0x2f, 0x10},
	{0x34, 0x00},
	{0x30, 0x15},
	{0x33, 0x01},
	{0x35, 0x20},
	{0x1d, 0xa5},
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
	{0x11, 0x63},
	{0x19, 0xc3},
	{0x26, 0xda},
	{0x29, 0x01},
	{0x33, 0x6f},
	{0x2a, 0xd2},
	{0x2c, 0x40},
	{0xd0, 0x02},
	{0xd1, 0x01},
	{0xd2, 0x20},
	{0xd3, 0x04},
	{0xd4, 0x2a},
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
	{0xb1, 0x82},
	{0xb3, 0x0b},
	{0xb4, 0x14},
	{0x9d, 0x40},
	{0xa1, 0x05},
	{0x94, 0x44},
	{0x95, 0x33},
	{0x96, 0x1f},
	{0x98, 0x45},
	{0x9c, 0x10},
	{0xb5, 0x70},
	{0xa0, 0x00},
	{0x25, 0xe0},
	{0x20, 0x7b},
	{0x8f, 0x88},
	{0x91, 0x40},
	{0xfd, 0x01},
	{0xfd, 0x02},
	{0x5e, 0x03},
	{0xfd, 0x02},
	{0x60, 0xf0},
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
	{0xfd, 0x02},
	{0x36, 0x08},
	{0xa0, 0x00},
	{0xa1, 0x08},
	{0xa2, 0x04},
	{0xa3, 0x38},
	{0xa4, 0x00},
	{0xa5, 0x04},
	{0xa6, 0x03},
	{0xa7, 0xc0},
	{0xfd, 0x03},
	{0xc0, 0x01},//OTP transf
	{0xfd, 0x04},
	{0x22, 0x14},
	{0x23, 0x14},
	{0xfd, 0x01},
	{0x06, 0xe0},
	{0x01, 0x01},
	{0xfd, 0x00},
	{0x1b, 0x00},
	{0xfd, 0x01},
	{0x0d, 0x10},
	{0x0e, 0x05},
	{0x0f, 0x2b},
	{0x01, 0x01},

	{OV2735B_REG_END, 0x00},        /* END MARKER */
};

#if 0

static const struct regval_list ov2735b_wb_auto_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_wb_incandescence_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_wb_daylight_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_wb_fluorescent_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_wb_cloud_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct mode_list ov2735b_balance[] = {
	{0, ov2735b_wb_auto_regs}, {1, ar0144_wb_incandescence_regs},
	{2, ov2735b_wb_daylight_regs}, {3, ar0144_wb_fluorescent_regs},
	{4, ov2735b_wb_cloud_regs},
};


static const struct regval_list ov2735b_effect_normal_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_effect_grayscale_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_effect_sepia_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_effect_colorinv_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ov2735b_effect_sepiabluel_regs[] = {
	{OV2735B_REG_END, 0x00},/* END MARKER */
};

static const struct mode_list ov2735b_effect[] = {
	{0, ov2735b_effect_normal_regs}, {1, ar0144_effect_grayscale_regs},
	{2, ov2735b_effect_sepia_regs}, {3, ar0144_effect_colorinv_regs},
	{4, ov2735b_effect_sepiabluel_regs},
};

#endif

static struct ov2735b_win_size ov2735b_supported_win_sizes[] = {
	{
		.name = "1080P",
		.width = 1920,
		.height = 1080,
		.regs = ov2735b_1080p_regs,
		//.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov2735b_supported_win_sizes))

/*
 * General functions
 */

static struct ov2735b_priv *to_ov2735b(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov2735b_priv,
			    subdev);
}

static int ov2735b_write_array(struct i2c_client *client,
			      const struct regval_list *vals)
{
	int ret;

	while (vals->reg_num != OV2735B_REG_END) {
		if (vals->reg_num == OV2735B_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov2735b_write_reg(client, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}
#if 0
static int ov2735b_mask_set(struct i2c_client *client,
			   u16  reg, u16  mask, u16  set)
{
	int ret = 0;
	unsigned char value[2] = {0};
	ret = ov2735b_read_reg(client, reg, value);
	if (ret < 0)
		return ret;

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return ov2735b_write_reg(client, reg, val);
}
#endif

/*
 * soc_camera_ops functions
 */


/*hw ops*/
static int ov2735b_hw_power(struct i2c_client *client, int on)
{
	struct ov2735b_priv *priv = to_ov2735b(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
			__func__, on ? "ENABLE" : "DISABLE");

	/* thses gpio should be set according to the active level in dt defines */
	if(priv->vcc_en_gpio) {
		gpiod_direction_output(priv->vcc_en_gpio, on);
	}

	if (priv->pwdn_gpio) {
		gpiod_direction_output(priv->pwdn_gpio, on);
	}

	msleep(10);
	return 0;
}

static int ov2735b_reset(struct i2c_client *client)
{
	struct ov2735b_priv *priv = to_ov2735b(client);

	if(priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}
	return 0;
}



/*ctrl_ops*/
static int ov2735b_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov2735b_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv *priv = to_ov2735b(client);

	switch (ctrl->id) {

	case V4L2_CID_VFLIP:
		ctrl->val = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->val = priv->flag_hflip;
		break;
	default:
		break;
	}
	return 0;
}


static int ov2735b_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
#if 0
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov2735b_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv *priv = to_ov2735b(client);
	int i = 0;
	u16 value;

	int balance_count = ARRAY_SIZE(ov2735b_balance);
	int effect_count = ARRAY_SIZE(ov2735b_effect);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		value = ctrl->val ? ov2735b_FLIP_VAL : 0x00;
		priv->flag_vflip = ctrl->val ? 1 : 0;
//		ret = ov2735b_mask_set(client, 0x301D, ar0144_FLIP_MASK, value);
		break;

	case V4L2_CID_HFLIP:
		value = ctrl->val ? ov2735b_FLIP_VAL : 0x00;
		priv->flag_hflip = ctrl->val ? 1 : 0;
//		ret = ov2735b_mask_set(client, 0x301D, ar0144_FLIP_MASK, value);
		break;

	default:
		dev_err(&client->dev, "no V4L2 CID: 0x%x ", ctrl->id);
		return -EINVAL;
	}

#endif
	return ret;
}

static const struct v4l2_ctrl_ops ov2735b_ctrl_ops = {
	.s_ctrl = ov2735b_s_ctrl,
	.g_volatile_ctrl = ov2735b_g_ctrl,
};

/*core_ops*/
static int ov2735b_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv *priv = to_ov2735b(client);

	if(priv->sd_pdata) {
		if(on)
			return regulator_bulk_enable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
		else
			return regulator_bulk_disable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
	} else if(priv->power){
		return priv->power(client, on);
	} else {
		dev_err(&client->dev, "ov2735b_s_power failde");
		return -EINVAL;
	}
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2735b_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = ov2735b_read_reg(client, reg->reg, reg->val);

	return 0;
}

static int ov2735b_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff)
		return -EINVAL;

	return ov2735b_write_reg(client, reg->reg, reg->val);
}
#endif

static struct regval_list ov2735b_stream_on[] = {
	{0xfd, 0x00},
	{0x36, 0x00},
	{0x37, 0x00},//fake stream on

	{OV2735B_REG_END, 0x00},
};

static struct regval_list ov2735b_stream_off[] = {
	{0xfd, 0x00},
	{0x36, 0x01},
	{0x37, 0x01},//fake stream off

	{OV2735B_REG_END, 0x00},
};

static int ov2735b_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);

	if (!enable ) {
		dev_info(&client->dev, "stream down\n");
		ov2735b_write_array(client, ov2735b_stream_off);
		return 0;
	}

	dev_info(&client->dev, "stream on\n");
	ov2735b_write_array(client, ov2735b_stream_on);

	return 0;
}

/* Select the nearest higher resolution for capture */
static const struct ov2735b_win_size *ov2735b_select_win(u32 *code, u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(ov2735b_supported_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(ov2735b_supported_win_sizes); i++) {
		if ((*width >= ov2735b_supported_win_sizes[i].width) &&
		    (*height >= ov2735b_supported_win_sizes[i].height) &&
		    (*code == ov2735b_supported_win_sizes[i].mbus_code)) {
			*width = ov2735b_supported_win_sizes[i].width;
			*height = ov2735b_supported_win_sizes[i].height;
			return &ov2735b_supported_win_sizes[i];
		}
	}

	return NULL;
}

static int ov2735b_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv *priv = to_ov2735b(client);

	if(!priv->win)
		return -EINVAL;

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int ov2735b_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv *priv = to_ov2735b(client);

	if (format->pad)
		return -EINVAL;

	if(priv->win){
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = ov2735b_DEFAULT_WIDTH;
		mf->height = ov2735b_DEFAULT_HEIGHT;
	}
	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov2735b_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct ov2735b_priv       *priv = to_ov2735b(client);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int ret;

	int bala_index = priv->balance_value;
	int effe_index = priv->effect_value;

	/* reset hardware */
	ov2735b_reset(client);

	/* initialize the sensor with default data */
	dev_dbg(&client->dev, "%s: Init default", __func__);
#if 0
	ret = ov2735b_write_array(client, ov2735b_init_regs);
	if (ret < 0)
		goto err;

	/* set balance */
	ret = ov2735b_write_array(client, ar0144_balance[bala_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set effect */
	ret = ov2735b_write_array(client, ar0144_effect[effe_index].mode_regs);
	if (ret < 0)
		goto err;
#endif
	/* set size win */
	dev_dbg(&client->dev, "priv->win.width = %d\n", priv->win->width);
	dev_dbg(&client->dev, "priv->win.height = %d\n", priv->win->height);

	ret = ov2735b_write_array(client, priv->win->regs);
	if (ret < 0)
		goto err;

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	ov2735b_reset(client);
	priv->win = NULL;

	return ret;
}

static int ov2735b_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2735b_priv       *priv = to_ov2735b(client);

	if (format->pad)
		return -EINVAL;

	priv->win = ov2735b_select_win(&mf->code ,&mf->width, &mf->height);
	if(!priv->win)
		return -EINVAL;

	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ov2735b_set_params(client, &mf->width,
					 &mf->height, mf->code);
	if(cfg)
		cfg->try_fmt = *mf;
	return 0;
}

static int ov2735b_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov2735b_supported_win_sizes))
		return -EINVAL;

	code->code = ov2735b_supported_win_sizes[code->index].mbus_code;
	return 0;
}

static int ov2735b_enum_frame_size(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j;
	int num_valid = -1;
	__u32 index = fse->index;

	if(index >= N_WIN_SIZES)
		return -EINVAL;

	fse->code = ov2735b_supported_win_sizes[index].mbus_code;
	fse->min_width = ov2735b_supported_win_sizes[index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov2735b_supported_win_sizes[index].height;
	fse->max_height = fse->min_height;
	return 0;

}

static int ov2735b_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_config *cfg)
{
	/*int dts*/
	return 0;
}

static struct v4l2_subdev_core_ops ov2735b_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov2735b_g_register,
	.s_register	= ov2735b_s_register,
#endif
	.s_power	= ov2735b_s_power,
};

static struct v4l2_subdev_video_ops ov2735b_subdev_video_ops = {
	.s_stream	= ov2735b_s_stream,
};

static const struct v4l2_subdev_pad_ops ov2735b_subdev_pad_ops = {
	.enum_mbus_code = ov2735b_enum_mbus_code,
	.enum_frame_size = ov2735b_enum_frame_size,
	.get_fmt	= ov2735b_get_fmt,
	.set_fmt	= ov2735b_set_fmt,
	.get_mbus_config	= ov2735b_g_mbus_config,
	.get_selection	= ov2735b_get_selection,
};

static struct v4l2_subdev_ops ov2735b_subdev_ops = {
	.core	= &ov2735b_subdev_core_ops,
	.video	= &ov2735b_subdev_video_ops,
	.pad	= &ov2735b_subdev_pad_ops,
};


static int ov2735b_detect(struct i2c_client *client, unsigned int *ident)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	unsigned char v;
	int ret;

	ov2735b_s_power(sd, 1);
	ret = ov2735b_read_reg(client, 0x02, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV2735B_CHIP_ID_H)
		return -ENODEV;

	ret = ov2735b_read_reg(client, 0x03, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != OV2735B_CHIP_ID_L)
		return -ENODEV;

	ret = ov2735b_read_reg(client, 0x04, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != 0x05 && v != 0x06 && v != 0x07)
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

//	ov2735b_s_power(sd, 0);
	return 0;

}


static int ov2735b_probe_dt(struct i2c_client *client,
		struct ov2735b_priv *priv)
{
	struct v4l2_subdev_platform_data *sd_pdata = priv->sd_pdata;
	struct device_node *np = client->dev.of_node;
	int supplies = 0, index = 0;

	supplies = of_property_count_strings(np, "supplies-name");
	if(supplies > 0) {

		sd_pdata = devm_kzalloc(&client->dev, sizeof(struct v4l2_subdev_platform_data), GFP_KERNEL);
		sd_pdata->num_regulators = supplies;
		sd_pdata->regulators = devm_kzalloc(&client->dev, supplies * sizeof(struct regulator_bulk_data), GFP_KERNEL);
		if(!sd_pdata->regulators) {
			dev_err(&client->dev, "Failed to allocate regulators.!\n");
			devm_kfree(&client->dev,sd_pdata);
			return -ENOMEM;
		}

		for(index = 0; index < sd_pdata->num_regulators; index ++) {
			of_property_read_string_index(np, "supplies-name", index,
					&(sd_pdata->regulators[index].supply));

			dev_dbg(&client->dev, "sd_pdata->regulators[%d].supply: %s\n",
					index, sd_pdata->regulators[index].supply);
		}

		devm_regulator_bulk_get(&client->dev, sd_pdata->num_regulators, sd_pdata->regulators);

		//if(!client->dev.platform_data) /*TODO*/
		//	client->dev.platform_data = &priv->sd_pdata;
	} else {


		/* Request the power down GPIO asserted */
		priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
				GPIOD_OUT_HIGH);
		if (!priv->pwdn_gpio)
			dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
		else if (IS_ERR(priv->pwdn_gpio))
			return PTR_ERR(priv->pwdn_gpio);

		/* Request the power vcc-en GPIO asserted */
		priv->vcc_en_gpio = devm_gpiod_get_optional(&client->dev, "vcc-en",
				GPIOD_OUT_HIGH);
		if (!priv->vcc_en_gpio)
			dev_dbg(&client->dev, "vcc_en gpio is not assigned!\n");
		else if (IS_ERR(priv->vcc_en_gpio))
			return PTR_ERR(priv->vcc_en_gpio);

		/* Request the power resetb GPIO asserted */
		priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
				GPIOD_OUT_LOW);
		if (!priv->resetb_gpio)
			dev_dbg(&client->dev, "vcc_en gpio is not assigned!\n");
		else if (IS_ERR(priv->resetb_gpio))
			return PTR_ERR(priv->resetb_gpio);

		/* Initialize the soc_camera_subdev_desc */
		priv->power = ov2735b_hw_power;
	//	priv->reset = ov2735b_hw_reset; /*TODO*/
	}
	return 0;
}

static int ov2735b_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov2735b_priv	*priv;
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int	ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"ov2735b: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ov2735b_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(priv->clk))
		return -EPROBE_DEFER;

	v4l2_clk_set_rate(priv->clk, 24000000);

	/*mclk gate close*/
//	*(volatile unsigned int *)0xb0000024 &= ~(1 << 5); 

	ret = ov2735b_probe_dt(client, priv);
	if (ret)
		goto err_probe_dt;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov2735b_subdev_ops);

	/* add handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);

	v4l2_ctrl_new_std(&priv->hdl, &ov2735b_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov2735b_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);

	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}

#if 0
	ret = ov2735b_detect(client, NULL);
	if(ret < 0){
		v4l_err(client,"chip found @ 0x%x (%s) is not an ov2735b chip.\n",
				client->addr << 1, client->adapter->name);
		goto err_detect;
	}
#endif

	ret = v4l2_ctrl_handler_setup(&priv->hdl);

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto err_async_register_subdev;

	dev_info(&adapter->dev, "ov2735b Probed\n");

	return 0;

err_async_register_subdev:
err_detect:
	v4l2_ctrl_handler_free(&priv->hdl);
err_hdl:
	v4l2_device_unregister_subdev(&priv->subdev);
err_probe_dt:
	v4l2_clk_put(priv->clk);
	devm_kfree(&client->dev, priv);
	return ret;
}

static int ov2735b_remove(struct i2c_client *client)
{
	struct ov2735b_priv       *priv = to_ov2735b(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	devm_kfree(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id ov2735b_id[] = {
	{ "ov2735b",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2735b_id);
static const struct of_device_id ov2735b_of_match[] = {
	{.compatible = "ovit,ov2735b", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2735b_of_match);
static struct i2c_driver ov2735b_i2c_driver = {
	.driver = {
		.name = "ov2735b",
		.of_match_table = of_match_ptr(ov2735b_of_match),
	},
	.probe    = ov2735b_probe,
	.remove   = ov2735b_remove,
	.id_table = ov2735b_id,
};
module_i2c_driver(ov2735b_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for onsemi ov2735b sensor");
MODULE_LICENSE("GPL v2");
