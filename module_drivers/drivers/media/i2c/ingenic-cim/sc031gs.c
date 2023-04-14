/*
 * sc031gs Camera Driver
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
#include <ingenic_camera.h>

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define REG_CHIP_ID_HIGH	0x3107
#define REG_CHIP_ID_LOW		0x3108
#define CHIP_ID_HIGH		0x00
#define CHIP_ID_LOW		0x31

#define  SC031GS_DEFAULT_WIDTH    640
#define  SC031GS_DEFAULT_HEIGHT   480

/* Private v4l2 controls */
#define V4L2_CID_PRIVATE_BALANCE  (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_EFFECT  (V4L2_CID_PRIVATE_BASE + 1)

/*
 * Struct
 */
struct regval_list {
	u16 reg_num;
	u16 value;
};

struct mode_list {
	u16 index;
	const struct regval_list *mode_regs;
};

/* Supported resolutions */
enum sc031gs_width {
	W_VGA	= SC031GS_DEFAULT_WIDTH,
};

enum sc031gs_height {
	H_VGA	= SC031GS_DEFAULT_HEIGHT,
};

struct sc031gs_win_size {
	char *name;
	enum sc031gs_width width;
	enum sc031gs_height height;
	const struct regval_list *regs;
	unsigned int mbus_code;
};

struct sc031gs_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct v4l2_clk			*clk;
	const struct sc031gs_win_size	*win;

	int				model;
	u16				balance_value;
	u16				effect_value;
	u16				flag_vflip:1;
	u16				flag_hflip:1;

	struct v4l2_subdev_platform_data *sd_pdata;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *vcc_en_gpio;

	int (*power)(struct i2c_client *client, int on);
	int (*reset)(struct i2c_client *client);

	struct regulator *reg;
	struct regulator *reg1;
};

static inline int sensor_i2c_master_send(struct i2c_client *client,
		const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client,
		char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}


unsigned char sc031gs_read_reg(struct i2c_client *client, u16 reg)
{
	int ret;
	unsigned char retval;
	unsigned short r = cpu_to_be16(reg);

	ret = sensor_i2c_master_send(client,(u8 *)&r,2);

	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;

	ret = sensor_i2c_master_recv(client, &retval, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;
	return retval;
}

int sc031gs_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	unsigned char msg[3];
	int ret;

	reg = cpu_to_be16(reg);

	memcpy(&msg[0], &reg, 2);
	memcpy(&msg[2], &val, 1);

	ret = sensor_i2c_master_send(client, msg, 3);

	if (ret < 0)
	{
		printk("RET<0\n");
		return ret;
	}
	if (ret < 3)
	{
		printk("RET<3\n");
		return -EIO;
	}

	return 0;
}

/*
 * Registers settings
 */

#define ENDMARKER { 0xffff, 0xff }

static const struct regval_list sc031gs_init_regs[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x3018,0x1f},
	{0x3019,0xff},
	{0x301c,0xb4},
	{0x363c,0x08},
	{0x3630,0x82},
	{0x3638,0x0f},
	{0x3639,0x08},
	{0x335b,0x80},
	{0x3636,0x25},
	{0x3640,0x02},
	{0x3306,0x38},
	{0x3304,0x48},
	{0x3389,0x01},
	{0x3385,0x31},
	{0x330c,0x18},
	{0x3315,0x38},
	{0x3306,0x28},
	{0x3309,0x68},
	{0x3387,0x51},
	{0x3306,0x48},
	{0x3366,0x04},
	{0x335f,0x80},
	{0x363a,0x00},
	{0x3622,0x01},
	{0x3633,0x62},
	{0x36f9,0x20},
	{0x3637,0x80},
	{0x363d,0x04},
	{0x3e06,0x00},
	{0x363c,0x48},
	{0x320c,0x03},
	{0x320e,0x0e},
	{0x320f,0xa8},
	{0x3306,0x38},
	{0x330b,0xb6},
	{0x36f9,0x24},
	{0x363b,0x4a},
	{0x3366,0x02},
	{0x3316,0x78},
	{0x3344,0x74},
	{0x3335,0x74},
	{0x332f,0x70},
	{0x332d,0x6c},
	{0x3329,0x6c},
	{0x363c,0x08},
	{0x3630,0x81},
	{0x3366,0x06},
	{0x3314,0x3a},
	{0x3317,0x28},
	{0x3622,0x05},
	{0x363d,0x00},
	{0x3637,0x86},
	{0x3e01,0x62},
	{0x3633,0x52},
	{0x3630,0x86},
	{0x3306,0x4c},
	{0x330b,0xa0},
	{0x3631,0x48},
	{0x33b1,0x03},
	{0x33b2,0x06},
	{0x320c,0x02},
	{0x320e,0x02},
	{0x320f,0x0d},
	{0x3e01,0x20},
	{0x3e02,0x20},
	{0x3316,0x68},
	{0x3344,0x64},
	{0x3335,0x64},
	{0x332f,0x60},
	{0x332d,0x5c},
	{0x3329,0x5c},
	{0x3310,0x10},
	{0x3637,0x87},
	{0x363e,0xf8},
	{0x3254,0x02},
	{0x3255,0x07},
	{0x3252,0x02},
	{0x3253,0xa6},
	{0x3250,0xf0},
	{0x3251,0x02},
	{0x330f,0x50},
	{0x3630,0x46},
	{0x3621,0xa2},
	{0x3621,0xa0},
	{0x4500,0x59},
	{0x3637,0x88},
	{0x3908,0x81},
	{0x3640,0x00},
	{0x3641,0x02},
	{0x363c,0x05},
	{0x363b,0x4c},
	{0x36e9,0x40},
	{0x36ea,0x36},
	{0x36ed,0x13},
	{0x36f9,0x04},
	{0x36fa,0x38},
	{0x330b,0x80},
	{0x3640,0x00},
	{0x3641,0x01},
	{0x3d08,0x00},
	{0x3306,0x48},
	{0x3621,0xa4},
	{0x300f,0x0f},
	{0x4837,0x1b},
	{0x4809,0x01},
	{0x363b,0x48},
	{0x363c,0x06},
	{0x36e9,0x00},
	{0x36ea,0x3b},
	{0x36eb,0x1A},
	{0x36ec,0x0A},
	{0x36ed,0x33},
	{0x36f9,0x00},
	{0x36fa,0x3a},
	{0x36fc,0x01},
	{0x320c,0x03},
	{0x320d,0x6e},
	{0x320e,0x02},
	{0x320f,0xab},
	{0x330b,0x80},
	{0x330f,0x50},
	{0x3637,0x89},
	{0x3641,0x01},
	{0x4501,0xc4},
	{0x5011,0x01},
	{0x3908,0x21},
	{0x3e01,0x40},
	{0x3e02,0x80},
	{0x3306,0x38},
	{0x330b,0xe0},
	{0x330f,0x20},
	{0x3d08,0x01},
	{0x3314,0x65},
	{0x5011,0x00},
	{0x3e06,0x0c},
	{0x3908,0x91},
	{0x3624,0x47},
	{0x3220,0x10},
	{0x3635,0x18},
	{0x3223,0x50},
	{0x301f,0x01},
	{0x3028,0x82},
	{0x0100,0x01},

	//Delay 10ms
	{0x4418,0x08},
	{0x4419,0x80},
	{0x363d,0x10},
	{0x3630,0x48},
	//[gain<2]
	{0x3314,0x65},
	{0x3317,0x10},
	//[4>gain>=2]
	{0x3314,0x65},
	{0x3317,0x10},
	//[gain>=4]
	{0x3314,0x60},
	{0x3317,0x0e},

	ENDMARKER,
};

static const struct regval_list sc031gs_qvga_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_vga_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_720p_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_1080p_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_wb_auto_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_wb_incandescence_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_wb_daylight_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_wb_fluorescent_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_wb_cloud_regs[] = {
	ENDMARKER,
};

static const struct mode_list sc031gs_balance[] = {
	{0, sc031gs_wb_auto_regs}, {1, sc031gs_wb_incandescence_regs},
	{2, sc031gs_wb_daylight_regs}, {3, sc031gs_wb_fluorescent_regs},
	{4, sc031gs_wb_cloud_regs},
};


static const struct regval_list sc031gs_effect_normal_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_effect_grayscale_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_effect_sepia_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_effect_colorinv_regs[] = {
	ENDMARKER,
};

static const struct regval_list sc031gs_effect_sepiabluel_regs[] = {
	ENDMARKER,
};

static const struct mode_list sc031gs_effect[] = {
	{0, sc031gs_effect_normal_regs}, {1, sc031gs_effect_grayscale_regs},
	{2, sc031gs_effect_sepia_regs}, {3, sc031gs_effect_colorinv_regs},
	{4, sc031gs_effect_sepiabluel_regs},
};

#define SC031GS_SIZE(n, w, h, r, c) \
	{.name = n, .width = w , .height = h, .regs = r, .mbus_code = c }

static struct sc031gs_win_size sc031gs_supported_win_sizes[] = {
	SC031GS_SIZE("VGA", W_VGA, H_VGA, sc031gs_vga_regs, MEDIA_BUS_FMT_Y8_1X8),
};

#define N_WIN_SIZES (ARRAY_SIZE(sc031gs_supported_win_sizes))

/*
 * General functions
 */
static struct sc031gs_priv *to_sc031gs(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct sc031gs_priv,
			    subdev);
}

static int sc031gs_write_array(struct i2c_client *client,
			      const struct regval_list *vals)
{
	int ret;

	while ((vals->reg_num != 0xffff) || (vals->value != 0xff)) {
		ret = sc031gs_write_reg(client, vals->reg_num, vals->value);
		dev_vdbg(&client->dev, "array: 0x%02x, 0x%02x",
			 vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

/* OF probe functions */
static int sc031gs_hw_power(struct i2c_client *client, int on)
{
	struct sc031gs_priv *priv = to_sc031gs(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
			__func__, on ? "ENABLE" : "DISABLE");

	/* thses gpio should be set according to the active level in dt defines */
	if(priv->vcc_en_gpio) {
		gpiod_direction_output(priv->vcc_en_gpio, !on);
	}

	msleep(3);

	if (priv->pwdn_gpio) {
		gpiod_direction_output(priv->pwdn_gpio, on);
	}

	msleep(10);
	return 0;
}

static int sc031gs_reset(struct i2c_client *client)
{
	struct sc031gs_priv *priv = to_sc031gs(client);

	if(priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}

	return 0;
}

static int sc031gs_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sc031gs_priv *priv = to_sc031gs(client);

	if(priv->sd_pdata) {
		if(on)
			return regulator_bulk_enable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
		else
			return regulator_bulk_disable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
	} else if(priv->power) {
		return priv->power(client, on);
	} else {
		dev_err(&client->dev, "sc031gs_s_power failde");
		return -EINVAL;
	}

}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc031gs_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = sc031gs_read_reg(client, reg->reg);
	if (ret < 0)
		return ret;

	reg->val = ret;

	return 0;
}

static int sc031gs_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff)
		return -EINVAL;

	return sc031gs_write_reg(client, reg->reg, reg->val);
}
#endif

/*
 * soc_camera_ops functions
 */
static int sc031gs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);

	if (!enable ) {
		dev_info(&client->dev, "stream down\n");
		sc031gs_write_reg(client, 0x0100, 0x00);
		return 0;
	}

	dev_info(&client->dev, "stream on\n");
	sc031gs_write_reg(client, 0x0100, 0x01);

	return 0;
}

/* Select the nearest higher resolution for capture */
static const struct sc031gs_win_size *sc031gs_select_win(u32 code, u32 *width, u32 *height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sc031gs_supported_win_sizes); i++) {
		if ((*width >= sc031gs_supported_win_sizes[i].width) &&
		    (*height >= sc031gs_supported_win_sizes[i].height) &&
		    (code == sc031gs_supported_win_sizes[i].mbus_code)) {
			*width = sc031gs_supported_win_sizes[i].width;
			*height = sc031gs_supported_win_sizes[i].height;
			return &sc031gs_supported_win_sizes[i];
		}
	}
	return NULL;
}

static int sc031gs_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct sc031gs_priv *priv = to_sc031gs(client);

	if(!priv->win)
		return -EINVAL;

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int sc031gs_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct sc031gs_priv *priv = to_sc031gs(client);

	if (format->pad)
		return -EINVAL;
	if(priv->win) {
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = SC031GS_DEFAULT_WIDTH;
		mf->height = SC031GS_DEFAULT_HEIGHT;
	}
	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int sc031gs_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct sc031gs_priv       *priv = to_sc031gs(client);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int ret;

	int bala_index = priv->balance_value;
	int effe_index = priv->effect_value;

	/* select win */
	priv->win = sc031gs_select_win(code, width, height);

	/* select format */
	priv->cfmt_code = 0;

	/* reset hardware */
	sc031gs_reset(client);

	/* initialize the sensor with default data */
	dev_dbg(&client->dev, "%s: Init default", __func__);

	ret = sc031gs_write_array(client, sc031gs_init_regs);
	if (ret < 0)
		goto err;

	/* set balance */
	ret = sc031gs_write_array(client, sc031gs_balance[bala_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set effect */
	ret = sc031gs_write_array(client, sc031gs_effect[effe_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set size win */
	ret = sc031gs_write_array(client, priv->win->regs);
	if (ret < 0)
		goto err;

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	sc031gs_reset(client);
	priv->win = NULL;

	return ret;
}

static int sc031gs_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sc031gs_priv *priv = to_sc031gs(client);

	if (format->pad)
		return -EINVAL;

	/*
	 * select suitable win, but don't store it
	 */
	priv->win = sc031gs_select_win(mf->code, &mf->width, &mf->height);
	if(!priv->win)
		return -EINVAL;

	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		sc031gs_set_params(client, &mf->width, &mf->height, mf->code);
	if(cfg)
		cfg->try_fmt = *mf;
	return 0;
}

static int sc031gs_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(sc031gs_supported_win_sizes))
		return -EINVAL;

	code->code = sc031gs_supported_win_sizes[code->index].mbus_code;
	return 0;
}

static int sc031gs_enum_frame_size(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_frame_size_enum *fse)
{
	__u32 index = fse->index;

	if(index >= N_WIN_SIZES)
		return -EINVAL;

	fse->code = sc031gs_supported_win_sizes[index].mbus_code;
	fse->min_width = sc031gs_supported_win_sizes[index].width;
	fse->max_width = fse->min_width;
	fse->min_height = sc031gs_supported_win_sizes[index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int sc031gs_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_config *cfg)
{
	/* int dts*/
	return 0;
}

static struct v4l2_subdev_core_ops sc031gs_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= sc031gs_g_register,
	.s_register	= sc031gs_s_register,
#endif
	.s_power	= sc031gs_s_power,
};

static struct v4l2_subdev_video_ops sc031gs_subdev_video_ops = {
	.s_stream	= sc031gs_s_stream,
};

static const struct v4l2_subdev_pad_ops sc031gs_subdev_pad_ops = {
	.enum_mbus_code = sc031gs_enum_mbus_code,
	.enum_frame_size = sc031gs_enum_frame_size,
	.get_fmt	= sc031gs_get_fmt,
	.set_fmt	= sc031gs_set_fmt,
	.get_mbus_config = sc031gs_g_mbus_config,
	.get_selection	= sc031gs_get_selection,
};

static struct v4l2_subdev_ops sc031gs_subdev_ops = {
	.core	= &sc031gs_subdev_core_ops,
	.video	= &sc031gs_subdev_video_ops,
	.pad	= &sc031gs_subdev_pad_ops,
};

static int sc031gs_hw_reset(struct i2c_client *client)
{
	struct sc031gs_priv *priv = to_sc031gs(client);

	if (priv->resetb_gpio) {
		/* Active the resetb pin to perform a reset pulse */
		gpiod_direction_output(priv->resetb_gpio, 1);
		usleep_range(3000, 5000);
		gpiod_direction_output(priv->resetb_gpio, 0);
	}

	return 0;
}

static int sc031gs_video_probe(struct i2c_client *client)
{
	unsigned char retval = 0, retval_high = 0, retval_low = 0;
	struct sc031gs_priv *priv = to_sc031gs(client);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int ret;

	ret = sc031gs_s_power(&priv->subdev, 1);
	if (ret < 0)
		return ret;
	/*
	 * check and show product ID and manufacturer ID
	 */

	retval = sc031gs_write_reg(client, 0x0103, 0x01);
	if(retval) {
		dev_err(&client->dev, "i2c write failed!\n");
		ret = -EINVAL;
		goto done;
	}

	retval_high = sc031gs_read_reg(client, REG_CHIP_ID_HIGH);
	if (retval_high != CHIP_ID_HIGH) {
		dev_err(&client->dev, "read sensor %s chip_id high %x is error\n",
				client->name, retval_high);
		ret = -EINVAL;
		goto done;
	}


	retval_low = sc031gs_read_reg(client, REG_CHIP_ID_LOW);
	if (retval_low != CHIP_ID_LOW) {
		dev_err(&client->dev, "read sensor %s chip_id low %x is error\n",
				client->name, retval_low);
		ret = -EINVAL;
		goto done;
	}

	dev_info(&client->dev, "read sensor %s id high:0x%x,low:%x successed!\n",
			client->name, retval_high, retval_low);

done:
	return ret;
}

static int sc031gs_probe_dt(struct i2c_client *client,
		struct sc031gs_priv *priv)
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
	} else {
		/* Request the reset GPIO deasserted */
		priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
				GPIOD_OUT_LOW);
		if (!priv->resetb_gpio)
			dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
		else if (IS_ERR(priv->resetb_gpio))
			return PTR_ERR(priv->resetb_gpio);

		/* Request the power down GPIO asserted */
		priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
				GPIOD_OUT_LOW);
		if (!priv->pwdn_gpio)
			dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
		else if (IS_ERR(priv->pwdn_gpio))
			return PTR_ERR(priv->pwdn_gpio);

		/* Request the power down GPIO asserted */
		priv->vcc_en_gpio = devm_gpiod_get_optional(&client->dev, "vcc-en",
				GPIOD_OUT_HIGH);
		if (!priv->vcc_en_gpio)
			dev_dbg(&client->dev, "vcc_en gpio is not assigned!\n");
		else if (IS_ERR(priv->vcc_en_gpio))
			return PTR_ERR(priv->vcc_en_gpio);

		/* Initialize the soc_camera_subdev_desc */
		priv->power = sc031gs_hw_power;
		priv->reset = sc031gs_hw_reset;
	}
	return 0;
}

#include <linux/regulator/consumer.h>
/*
 * i2c_driver functions
 */
static int sc031gs_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct sc031gs_priv	*priv;
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int	ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"sc031gs: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct sc031gs_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(priv->clk))
		return -EPROBE_DEFER;

	v4l2_clk_set_rate(priv->clk, 24000000);

	ret = sc031gs_probe_dt(client, priv);
	if (ret)
		goto err_probe_dt;

	v4l2_i2c_subdev_init(&priv->subdev, client, &sc031gs_subdev_ops);

	ret = sc031gs_video_probe(client);
	if (ret < 0)
		goto err_detect;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto err_async_register_subdev;

	dev_info(&adapter->dev, "sc031gs Probed\n");

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

static int sc031gs_remove(struct i2c_client *client)
{
	struct sc031gs_priv       *priv = to_sc031gs(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	devm_kfree(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id sc031gs_id[] = {
	{ "sc031gs",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc031gs_id);
static const struct of_device_id sc031gs_of_match[] = {
	{.compatible = "sc031gs", },
	{},
};
MODULE_DEVICE_TABLE(of, sc031gs_of_match);
static struct i2c_driver sc031gs_i2c_driver = {
	.driver = {
		.name = "sc031gs",
		.of_match_table = of_match_ptr(sc031gs_of_match),
	},
	.probe    = sc031gs_probe,
	.remove   = sc031gs_remove,
	.id_table = sc031gs_id,
};

module_i2c_driver(sc031gs_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for smartsens SC031GS sensor");
MODULE_LICENSE("GPL v2");
