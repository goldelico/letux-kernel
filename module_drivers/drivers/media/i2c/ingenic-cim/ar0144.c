/*
 * ar0144 Camera Driver
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

#define AR0144_REG_END         0xffff
#define AR0144_REG_DELAY       0xfffe
#define AR0144_REG_CHIP_ID		0x3000
#define AR0144_CHIP_ID_HIGH		0x03
#define AR0144_CHIP_ID_LOW		0x56

#define  ar0144_DEFAULT_WIDTH    640
#define  ar0144_DEFAULT_HEIGHT   480

/* Private v4l2 controls */
#define V4L2_CID_PRIVATE_BALANCE  (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_EFFECT  (V4L2_CID_PRIVATE_BASE + 1)

#define ar0144_FLIP_VAL			((unsigned char)0x04)
#define ar0144_FLIP_MASK		((unsigned char)0x04)

/* whether sensor support high resolution (> vga) preview or not */
#define SUPPORT_HIGH_RESOLUTION_PRE		1

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

struct ar0144_win_size {
	char *name;
	int width;
	int height;
	const struct regval_list *regs;
	unsigned int mbus_code;
};


struct ar0144_gpio {
	int pin;
	int active_level;
};

struct ar0144_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct v4l2_clk			*clk;
	const struct ar0144_win_size	*win;

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

unsigned short ar0144_read_reg(struct i2c_client *client, u16 reg, unsigned char *value)
{
	int ret;
	unsigned short r = cpu_to_be16(reg);
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
			.len	= 2,
			.buf	= value,
		}
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

int ar0144_write_reg(struct i2c_client *client, unsigned short reg, unsigned short value)
{
	int ret;
	uint8_t buf[4] = {reg >> 8, reg & 0xff, value >> 8, value & 0xff};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 4,
		.buf	= buf,
	};
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static struct regval_list ar0144_720p_regs_mipi[] = {
	//mipibps 444
	//pclk 74.00
	//hts:0x5D0  VTS 0x338
	//again default 0x3060 = 0x0010
	//expo 0x3012 = 0x01F3
	//[ar0144_720P_60FPS]
//	{ AR0144_REG_DELAY, 200},
	{ 0x301A, 0x00D9 }, // RESET_REGISTER
	{ AR0144_REG_DELAY, 200},
	{ 0x301A, 0x3058 }, // RESET_REGISTER
	{ AR0144_REG_DELAY, 100},
	{ 0x3F4C, 0x003F }, // PIX_DEF_1D_DDC_LO_DEF
	{ 0x3F4E, 0x0018 }, // PIX_DEF_1D_DDC_HI_DEF
	{ 0x3F50, 0x17DF }, // PIX_DEF_1D_DDC_EDGE
	{ 0x30B0, 0x0028 }, // DIGITAL_TEST
	{ AR0144_REG_DELAY, 200},
	{ 0x3ED6, 0x3CB5 }, // DAC_LD_10_11
	{ 0x3ED8, 0x8765 }, // DAC_LD_12_13
	{ 0x3EDA, 0x8888 }, // DAC_LD_14_15
	{ 0x3EDC, 0x97FF }, // DAC_LD_16_17
	{ 0x3EF8, 0x6522 }, // DAC_LD_44_45
	{ 0x3EFA, 0x2222 }, // DAC_LD_46_47
	{ 0x3EFC, 0x6666 }, // DAC_LD_48_49
	{ 0x3F00, 0xAA05 }, // DAC_LD_52_53
	{ 0x3EE2, 0x180E }, // DAC_LD_22_23
	{ 0x3EE4, 0x0808 }, // DAC_LD_24_25
	{ 0x3EEA, 0x2A09 }, // DAC_LD_30_31
	{ 0x3060, 0x003C }, // ANALOG_GAIN
	{ 0x30FE, 0x00A8 }, // NOISE_PEDESTAL
	{ 0x3092, 0x00CF }, // ROW_NOISE_CONTROL
	{ 0x3268, 0x0030 }, // SEQUENCER_CONTROL
	{ 0x3786, 0x0006 }, // DIGITAL_CTRL_1
	{ 0x3F4A, 0x0F70 }, // DELTA_DK_PIX_THRES
	{ 0x306E, 0x4810 }, // DATAPATH_SELECT
	{ 0x3064, 0x1802 }, // SMIA_TEST
	{ 0x3EF6, 0x804D }, // DAC_LD_42_43
	{ 0x3180, 0xC08F }, // DELTA_DK_CONTROL
	{ 0x30BA, 0x7623 }, // DIGITAL_CTRL
	{ 0x3176, 0x0480 }, // DELTA_DK_ADJUST_GREENR
	{ 0x3178, 0x0480 }, // DELTA_DK_ADJUST_RED
	{ 0x317A, 0x0480 }, // DELTA_DK_ADJUST_BLUE
	{ 0x317C, 0x0480 }, // DELTA_DK_ADJUST_GREENB
	{ 0x302A, 0x0006 }, // VT_PIX_CLK_DIV
	{ 0x302C, 0x0001 }, // VT_SYS_CLK_DIV
	{ 0x302E, 0x0004 }, // PRE_PLL_CLK_DIV
	{ 0x3030, 0x004A }, // PLL_MULTIPLIER
	{ 0x3036, 0x000C }, // OP_PIX_CLK_DIV
	{ 0x3038, 0x0001 }, // OP_SYS_CLK_DIV
	{ 0x30B0, 0x0028 }, // DIGITAL_TEST
	{ AR0144_REG_DELAY, 100},
	{ 0x31AE, 0x0202 }, // SERIAL_FORMAT
	{ 0x31AC, 0x0C0C }, // DATA_FORMAT_BITS
	{ 0x31B0, 0x0042 }, // FRAME_PREAMBLE
	{ 0x31B2, 0x002E }, // LINE_PREAMBLE
	{ 0x31B4, 0x2633 }, // MIPI_TIMING_0
	{ 0x31B6, 0x210E }, // MIPI_TIMING_1
	{ 0x31B8, 0x20C7 }, // MIPI_TIMING_2
#if 0
	{ 0x31BA, 0x0105 }, // MIPI_TIMING_3
	{ 0x31BC, 0x0004 }, // MIPI_TIMING_4
#else
	{ 0x31BA, 0x01a5 }, // MIPI_TIMING_3
	{ 0x31BC, 0x8004 }, // MIPI_TIMING_4
#endif
	{ 0x3002, 0x0028 }, // Y_ADDR_START
	{ 0x3004, 0x0004 }, // X_ADDR_START
	{ 0x3006, 0x02F7 }, // Y_ADDR_END
	{ 0x3008, 0x0503 }, // X_ADDR_END
	{ 0x300A, 0x0338 }, // FRAME_LENGTH_LINES
	{ 0x300C, 0x05D0 }, // LINE_LENGTH_PCK
//	{ 0x3012, 0x0064 }, // COARSE_INTEGRATION_TIME
	{ 0x3012, 0x01F3 }, // COARSE_INTEGRATION_TIME
	{ 0x30A2, 0x0001 }, // X_ODD_INC
	{ 0x30A6, 0x0001 }, // Y_ODD_INC
	{ 0x3040, 0x0000 }, // READ_MODE
	{ 0x3064, 0x1882 }, // SMIA_TEST
	{ 0x3064, 0x1802 }, // SMIA_TEST
	{ 0x3028, 0x0010 }, // ROW_SPEED
//	{ 0x301D, 0x0003 }, // FLIP
#ifdef SNAPSHOT
	{ 0x301A, 0x0958 }, // RESET_REGISTER stream_off
	{ 0x30CE, 0x0120 },
#else
#endif
	{AR0144_REG_END, 0x00},/* END MARKER */
};


#if 0

static const struct regval_list ar0144_wb_auto_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_wb_incandescence_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_wb_daylight_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_wb_fluorescent_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_wb_cloud_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct mode_list ar0144_balance[] = {
	{0, ar0144_wb_auto_regs}, {1, ar0144_wb_incandescence_regs},
	{2, ar0144_wb_daylight_regs}, {3, ar0144_wb_fluorescent_regs},
	{4, ar0144_wb_cloud_regs},
};


static const struct regval_list ar0144_effect_normal_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_effect_grayscale_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_effect_sepia_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_effect_colorinv_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct regval_list ar0144_effect_sepiabluel_regs[] = {
	{AR0144_REG_END, 0x00},/* END MARKER */
};

static const struct mode_list ar0144_effect[] = {
	{0, ar0144_effect_normal_regs}, {1, ar0144_effect_grayscale_regs},
	{2, ar0144_effect_sepia_regs}, {3, ar0144_effect_colorinv_regs},
	{4, ar0144_effect_sepiabluel_regs},
};

#endif

static struct ar0144_win_size ar0144_supported_win_sizes[] = {
	{
		.name = "720P",
		.width = 1920 ,
		.height = 720,
		.regs = ar0144_720p_regs_mipi,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ar0144_supported_win_sizes))

/*
 * General functions
 */

static struct ar0144_priv *to_ov2735b(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ar0144_priv,
			    subdev);
}

static int ar0144_write_array(struct i2c_client *client,
			      const struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != AR0144_REG_END) {
		if (vals->reg_num == AR0144_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ar0144_write_reg(client, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}

#if 0
static int ar0144_mask_set(struct i2c_client *client,
			   u16  reg, u16  mask, u16  set)
{
	int ret = 0;
	unsigned char value[2] = {0};
	ret = ar0144_read_reg(client, reg, value);
	if (ret < 0)
		return ret;

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return ar0144_write_reg(client, reg, val);
}
#endif

/*
 * soc_camera_ops functions
 */


/*hw ops*/
static int ar0144_hw_power(struct i2c_client *client, int on)
{
	struct ar0144_priv *priv = to_ov2735b(client);

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

static int ar0144_reset(struct i2c_client *client)
{
	struct ar0144_priv *priv = to_ov2735b(client);

	if(priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}
	return 0;
}



/*ctrl_ops*/
static int ar0144_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ar0144_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv *priv = to_ov2735b(client);

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


static int ar0144_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
#if 0
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ar0144_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv *priv = to_ov2735b(client);
	int i = 0;
	u16 value;

	int balance_count = ARRAY_SIZE(ar0144_balance);
	int effect_count = ARRAY_SIZE(ar0144_effect);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		value = ctrl->val ? ar0144_FLIP_VAL : 0x00;
		priv->flag_vflip = ctrl->val ? 1 : 0;
//		ret = ar0144_mask_set(client, 0x301D, ar0144_FLIP_MASK, value);
		break;

	case V4L2_CID_HFLIP:
		value = ctrl->val ? ar0144_FLIP_VAL : 0x00;
		priv->flag_hflip = ctrl->val ? 1 : 0;
//		ret = ar0144_mask_set(client, 0x301D, ar0144_FLIP_MASK, value);
		break;

	default:
		dev_err(&client->dev, "no V4L2 CID: 0x%x ", ctrl->id);
		return -EINVAL;
	}

#endif
	return ret;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl = ar0144_s_ctrl,
	.g_volatile_ctrl = ar0144_g_ctrl,
};

/*core_ops*/
static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv *priv = to_ov2735b(client);

	if(priv->sd_pdata) {
		if(on)
			return regulator_bulk_enable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
		else
			return regulator_bulk_disable(priv->sd_pdata->num_regulators,priv->sd_pdata->regulators);
	} else if(priv->power){
		return priv->power(client, on);
	} else {
		dev_err(&client->dev, "ar0144_s_power failde");
		return -EINVAL;
	}
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0144_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = ar0144_read_reg(client, reg->reg, reg->val);

	return 0;
}

static int ar0144_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff)
		return -EINVAL;

	return ar0144_write_reg(client, reg->reg, reg->val);
}
#endif

static int ar0144_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);

	if (!enable ) {
		dev_info(&client->dev, "stream down\n");
#ifdef SNAPSHOT
		ar0144_write_reg(client, 0x301A, 0x0958);
#else
		ar0144_write_reg(client, 0x301A, 0x0058);
#endif
		return 0;
	}

	dev_info(&client->dev, "stream on\n");
#ifdef SNAPSHOT
	ar0144_write_reg(client, 0x301A, 0x095C);
#else
	ar0144_write_reg(client, 0x301A, 0x005C);
#endif

	return 0;
}

/* Select the nearest higher resolution for capture */
static const struct ar0144_win_size *ar0144_select_win(u32 *code, u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(ar0144_supported_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(ar0144_supported_win_sizes); i++) {
		if ((*width >= ar0144_supported_win_sizes[i].width) &&
		    (*height >= ar0144_supported_win_sizes[i].height) &&
		    (*code == ar0144_supported_win_sizes[i].mbus_code)) {
			*width = ar0144_supported_win_sizes[i].width;
			*height = ar0144_supported_win_sizes[i].height;
			return &ar0144_supported_win_sizes[i];
		}
	}
	return NULL;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv *priv = to_ov2735b(client);

	if(!priv->win)
		return -EINVAL;

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int ar0144_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv *priv = to_ov2735b(client);

	if (format->pad)
		return -EINVAL;

	if(priv->win){
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = ar0144_DEFAULT_WIDTH;//priv->win->width;
		mf->height = ar0144_DEFAULT_HEIGHT;//priv->win->height;
	}
	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ar0144_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct ar0144_priv       *priv = to_ov2735b(client);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int ret;

	int bala_index = priv->balance_value;
	int effe_index = priv->effect_value;

	/* reset hardware */
	ar0144_reset(client);

	/* initialize the sensor with default data */
#if 0
	dev_dbg(&client->dev, "%s: Init default", __func__);
	ret = ar0144_write_array(client, ov2735b_init_regs);
	if (ret < 0)
		goto err;

	/* set balance */
	ret = ar0144_write_array(client, ar0144_balance[bala_index].mode_regs);
	if (ret < 0)
		goto err;

	/* set effect */
	ret = ar0144_write_array(client, ar0144_effect[effe_index].mode_regs);
	if (ret < 0)
		goto err;
#endif
	/* set size win */
	dev_dbg(&client->dev, "priv->win.width = %d\n", priv->win->width);
	dev_dbg(&client->dev, "priv->win.height = %d\n", priv->win->height);

	ret = ar0144_write_array(client, priv->win->regs);
	if (ret < 0)
		goto err;

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	ar0144_reset(client);
	priv->win = NULL;

	return ret;
}

static int ar0144_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0144_priv       *priv = to_ov2735b(client);

	if (format->pad)
		return -EINVAL;

	priv->win = ar0144_select_win(&mf->code ,&mf->width, &mf->height);
	if(!priv->win)
		return -EINVAL;

	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ar0144_set_params(client, &mf->width,
					 &mf->height, mf->code);
	if(cfg)
		cfg->try_fmt = *mf;
	return 0;
}

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ar0144_supported_win_sizes))
		return -EINVAL;

	code->code = ar0144_supported_win_sizes[code->index].mbus_code;
	return 0;
}

static int ar0144_enum_frame_size(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j;
	int num_valid = -1;
	__u32 index = fse->index;

	if(index >= N_WIN_SIZES)
		return -EINVAL;

	fse->code = ar0144_supported_win_sizes[index].mbus_code;
	fse->min_width = ar0144_supported_win_sizes[index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ar0144_supported_win_sizes[index].height;
	fse->max_height = fse->min_height;
	return 0;

}

static int ar0144_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_config *cfg)
{
	/*in dts*/
	return 0;
}

static struct v4l2_subdev_core_ops ar0144_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ar0144_g_register,
	.s_register	= ar0144_s_register,
#endif
	.s_power	= ar0144_s_power,
};

static struct v4l2_subdev_video_ops ar0144_subdev_video_ops = {
	.s_stream	= ar0144_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_size,
	.get_fmt	= ar0144_get_fmt,
	.set_fmt	= ar0144_set_fmt,
	.get_mbus_config	= ar0144_g_mbus_config,
	.get_selection	= ar0144_get_selection,
};

static struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core	= &ar0144_subdev_core_ops,
	.video	= &ar0144_subdev_video_ops,
	.pad	= &ar0144_subdev_pad_ops,
};


static int ar0144_detect(struct i2c_client *client, unsigned int *ident)
{
	unsigned char value[2] = {0};
	int ret;

	ret = ar0144_read_reg(client, AR0144_REG_CHIP_ID, value);

	if (ret < 0)
		return ret;

	if (value[0] != AR0144_CHIP_ID_HIGH)
		return -ENODEV;

	if (value[1] != AR0144_CHIP_ID_LOW)
		return -ENODEV;

	return 0;
}


static int ar0144_probe_dt(struct i2c_client *client,
		struct ar0144_priv *priv)
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
		priv->power = ar0144_hw_power;
	//	priv->reset = ar0144_hw_reset; /*TODO*/
	}
	return 0;
}

static int ar0144_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ar0144_priv	*priv;
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int	ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"ar0144: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ar0144_priv), GFP_KERNEL);
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

	ret = ar0144_probe_dt(client, priv);
	if (ret)
		goto err_probe_dt;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ar0144_subdev_ops);

	/* add handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);

	v4l2_ctrl_new_std(&priv->hdl, &ar0144_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ar0144_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);

	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}

#if 1
	ret = ar0144_detect(client, NULL);
	if(ret < 0){
		v4l_err(client,"chip found @ 0x%x (%s) is not an ar0144 chip.\n",
				client->addr << 1, client->adapter->name);
		goto err_detect;
	}
#endif

	ret = v4l2_ctrl_handler_setup(&priv->hdl);

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto err_async_register_subdev;

	dev_info(&adapter->dev, "ar0144 Probed\n");

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

static int ar0144_remove(struct i2c_client *client)
{
	struct ar0144_priv       *priv = to_ov2735b(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	devm_kfree(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id ar0144_id[] = {
	{ "ar0144",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0144_id);
static const struct of_device_id ar0144_of_match[] = {
	{.compatible = "onsemi,ar0144", },
	{},
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);
static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.name = "ar0144",
		.of_match_table = of_match_ptr(ar0144_of_match),
	},
	.probe    = ar0144_probe,
	.remove   = ar0144_remove,
	.id_table = ar0144_id,
};
module_i2c_driver(ar0144_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for onsemi ar0144 sensor");
MODULE_LICENSE("GPL v2");
