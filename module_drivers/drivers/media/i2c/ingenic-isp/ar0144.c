/*
 * A V4L2 driver for OmniVision AR0144 cameras.
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
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>

#include <isp-sensor.h>

#define AR0144_CHIP_ID_H	(0x03)
#define AR0144_CHIP_ID_L	(0x56)
#define AR0144_CHIP_ID          (0x0356)
#define AR0144_CHIP_ID_REG      (0x3000)

#define AR0144_REG_END		0xffff
#define AR0144_REG_DELAY	0xfffe

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ar0144_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct ar0144_gpio {
	int pin;
	int active_level;
};


struct ar0144_supply{
	struct regulator *pwen;
	struct regulator *avdd;
	struct regulator *dvdd;
	struct regulator *dovdd;
};

struct ar0144_info {
	struct ar0144_supply supply;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct ar0144_win_size *win;

	struct ar0144_gpio reset;
	struct ar0144_gpio expo;
	struct ar0144_gpio oe;
	struct ar0144_gpio pwen;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};
struct again_lut ar0144_again_lut[] = {
    { 0x0, 0 },
    { 0x1, 2794 },
    { 0x2, 6397 },
    { 0x3, 9011 },
    { 0x4, 12388 },
    { 0x5, 16447 },
    { 0x6, 19572 },
    { 0x7, 23340 },
    { 0x8, 26963 },
    { 0x9, 31135 },
    { 0xa, 35780 },
    { 0xb, 39588 },
    { 0xc, 44438 },
    { 0xd, 49051 },
    { 0xe, 54517 },
    { 0xf, 59685 },
    { 0x10, 65536 },
    { 0x12, 71490 },
    { 0x14, 78338 },
    { 0x16, 85108 },
    { 0x18, 92854 },
    { 0x1a, 100992 },
    { 0x1c, 109974 },
    { 0x1e, 120053 },
    { 0x20, 131072 },
    { 0x22, 137247 },
    { 0x24, 143667 },
    { 0x26, 150644 },
    { 0x28, 158212 },
    { 0x2a, 166528 },
    { 0x2c, 175510 },
    { 0x2e, 185457 },
    { 0x30, 196608 },
    { 0x32, 202783 },
    { 0x34, 209203 },
    { 0x36, 216276 },
    { 0x38, 223748 },
    { 0x3a, 232064 },
    { 0x3c, 241046 },
    { 0x3e, 250993 },
    { 0x40, 262144 },
};

static inline struct ar0144_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar0144_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned short value;
};

static struct regval_list ar0144_init_regs_720p_60fps_mipi[] = {
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
	{ 0x3060, 0x0010 }, // ANALOG_GAIN
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
	{ 0x3006, 0x02F8 }, // Y_ADDR_END
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
//	{ 0x301A, 0x005C }, // RESET_REGISTER stream_on
	{AR0144_REG_END, 0x00},/* END MARKER */
};

/*
 * the part of driver was fixed.
 */

static struct regval_list ar0144_stream_on[] = {
//	{0x301A, 0x19CC},
	{0x301A, 0x005C},
	{AR0144_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list ar0144_stream_off[] = {
//	{0x301A, 0x19C8},
	{0x301A, 0x0058},
	{AR0144_REG_END, 0x00},	/* END MARKER */
};

static int ar0144_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct ar0144_info *info = to_state(sd);
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
			.len	= 2,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int ar0144_write(struct v4l2_subdev *sd, unsigned short reg,
		unsigned short value)
{
	struct ar0144_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[4] = {reg >> 8, reg & 0xff, value >> 8, value & 0xff};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 4,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ar0144_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val[2];
	uint16_t value;
	while (vals->reg_num != AR0144_REG_END) {
		if (vals->reg_num == AR0144_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ar0144_read(sd, vals->reg_num, val);
			if (ret < 0)
				return ret;
		}
		vals->value=(val[0]<<8)|val[1];
		vals++;
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ar0144_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != AR0144_REG_END) {
		if (vals->reg_num == AR0144_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ar0144_write(sd, vals->reg_num, vals->value);
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
static int ar0144_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ar0144_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(20);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(20);
	}
	return 0;
}

static int ar0144_init(struct v4l2_subdev *sd, u32 val)
{
	struct ar0144_info *info = to_state(sd);
	int ret = 0;

	ret = ar0144_write_array(sd, info->win->regs);

	return ret;
}



static int ar0144_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v[2] = {0};
	int ret;
	ret = ar0144_read(sd, 0x3000, v);

	if (ret < 0)
		return ret;

	if (v[0] != AR0144_CHIP_ID_H)
		return -ENODEV;

	if (v[1] != AR0144_CHIP_ID_L)
		return -ENODEV;

	return 0;
}


static struct ar0144_win_size ar0144_win_sizes[] = {
	{
		.sensor_info.mipi_cfg.twidth		= 1280,
		.sensor_info.mipi_cfg.theight		= 720,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.sensor_info.fps			= 60 << 16 | 1,

		.width		= 1280,
		.height		= 720,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= ar0144_init_regs_720p_60fps_mipi,
	},
};

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_AR0144_FMTS)
		return -EINVAL;

	code->code = ar0144_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int ar0144_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ar0144_format_struct *ovfmt;
	struct ar0144_win_size *wsize;
	struct ar0144_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int ar0144_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct ar0144_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if(!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = info->win->width;
	fmt->height = info->win->height;
	fmt->code = info->win->mbus_code;
	fmt->colorspace = info->win->colorspace;
	*(unsigned int *)fmt->reserved = &info->win->sensor_info;


//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int ar0144_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0144_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0144_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0144_s_vflip(struct v4l2_subdev *sd, int value)
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
static int ar0144_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ar0144_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(ar0144_again_lut); i++) {
		lut = &ar0144_again_lut[i];

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
	for(i = 0; i < ARRAY_SIZE(ar0144_again_lut); i++) {
		lut = &ar0144_again_lut[i];

		if(regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return 0;
}

static int ar0144_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	struct ar0144_info *info = to_state(sd);
	unsigned int reg_val = 0;
	int ret = 0;


	ret = ar0144_read(sd, 0x3509, &reg_val);

	*value = regval_to_again(reg_val);

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int ar0144_s_again(struct v4l2_subdev *sd, int value)
{
	struct ar0144_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	int i;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret += ar0144_write(sd, 0x3060, reg_value);
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Tweak autogain.
 */
static int ar0144_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ar0144_s_exp(struct v4l2_subdev *sd, int value)
{
	struct ar0144_info *info = to_state(sd);
	int ret = 0;

	ret = ar0144_write(sd, 0x3012, value);

	return ret;
}

static int ar0144_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ar0144_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ar0144_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ar0144_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ar0144_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ar0144_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ar0144_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ar0144_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ar0144_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ar0144_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ar0144_s_gain turns off auto gain */
			return ar0144_s_gain(sd, info->gain->val);
		}
		return ar0144_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ar0144_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ar0144_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ar0144_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl = ar0144_s_ctrl,
	.g_volatile_ctrl = ar0144_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0144_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ar0144_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ar0144_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ar0144_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int ar0144_power_t(struct v4l2_subdev *sd, int on)
{
	struct ar0144_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	info->supply.pwen  = devm_regulator_get_optional(&client->dev, "pwen");
	if (IS_ERR(info->supply.pwen)){
			if ((PTR_ERR(info->supply.pwen) == -EPROBE_DEFER))
				return -EPROBE_DEFER;
			printk("No ar0144 vdd regulator found\n");
	}

	if (!IS_ERR(info->supply.pwen)){
		if(on){
			ret = regulator_enable(info->supply.pwen);
			if (ret)
				dev_err(&client->dev, "ar0144 pwen supply disable failed\n");
		}else{
			ret = regulator_disable(info->supply.pwen);
			if (ret)
				dev_err(&client->dev, "ar0144 pwen supply disable failed\n");
		}
	}else{
		dev_err(&client->dev, "ar0144 vdd supply IS_ERR failed\n");
	}

	return ret;
}

int ar0144_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/*double sensor,eg halley5_camera v1.0*/
	ar0144_power_t(sd,on);

	info->supply.avdd  = devm_regulator_get_optional(&client->dev, "avdd");
	info->supply.dvdd  = devm_regulator_get_optional(&client->dev, "dvdd");
	info->supply.dovdd = devm_regulator_get_optional(&client->dev, "dovdd");

	if (IS_ERR(info->supply.avdd)||IS_ERR(info->supply.dvdd)
		||IS_ERR(info->supply.dovdd)) {
			if ((PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
				||(PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
					||(PTR_ERR(info->supply.avdd) == -EPROBE_DEFER))
				return -EPROBE_DEFER;
			printk("No ar0144 vdd regulator found\n");
	}

	if ((!IS_ERR(info->supply.avdd))&&(!IS_ERR(info->supply.avdd))
			&&(!IS_ERR(info->supply.avdd)))  {
		if(on){
			ret = regulator_enable(info->supply.avdd);
			ret = regulator_enable(info->supply.dvdd);
			ret = regulator_enable(info->supply.dovdd);
			if (ret)
			   dev_err(&client->dev, "ar0144 vdd supply disable failed\n");
		}
		else{
			ret = regulator_disable(info->supply.avdd);
			ret = regulator_disable(info->supply.dvdd);
			ret = regulator_disable(info->supply.dovdd);
			if (ret)
			    dev_err(&client->dev, "ar0144 vdd supply disable failed\n");
		}
	}else{
		dev_err(&client->dev, "ar0144 vdd supply IS_ERR failed\n");
	}

	return ret;
}

int ar0144_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0144_info *info = to_state(sd);
	int ret = 0;
	struct regval_list *vals;

	if (enable) {
		ret = ar0144_write_array(sd, ar0144_stream_on);
		pr_debug("ar0144 stream on\n");
	}
	else {
		ret = ar0144_write_array(sd, ar0144_stream_off);
		pr_debug("ar0144 stream off\n");
	}
	return ret;
}


int ar0144_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ar0144_info *info = to_state(sd);
	if(info->win->fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ar0144_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ar0144_g_register,
	.s_register = ar0144_s_register,
#endif

};

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
	.g_frame_interval = ar0144_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0144_pad_ops = {
	//.enum_frame_interval = ar0144_enum_frame_interval,
	//.num_frame_size = ar0144_enum_frame_size,
	//.enum_mbus_code = ar0144_enum_mbus_code,
	.set_fmt = ar0144_set_fmt,
	.get_fmt = ar0144_get_fmt,
};

static const struct v4l2_subdev_ops ar0144_ops = {
	.core = &ar0144_core_ops,
	.video = &ar0144_video_ops,
	.pad = &ar0144_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ar0144_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_fract tpf;
	struct v4l2_subdev *sd;
	struct ar0144_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,expo-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->expo.pin = gpio;
		info->expo.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,oe-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->oe.pin = gpio;
		info->oe.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwen-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwen.pin = gpio;
		info->pwen.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}


	v4l2_i2c_subdev_init(sd, client, &ar0144_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ar0144_power(sd, 1);
	msleep(30);

	/*clk*/
	info->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}

	ret = v4l2_clk_set_rate(info->clk, 24000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");

	ar0144_reset(sd, 1);
#if 1
	/* Make sure it's an ar0144 */
	ret = ar0144_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an ar0144 chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 1, 262144, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ar0144_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1520 - 4, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &ar0144_win_sizes[0];

	ar0144_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "ar0144 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int ar0144_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id ar0144_id[] = {
	{ "ar0144", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0144_id);

static const struct of_device_id ar0144_of_match[] = {
	{.compatible = "onsemi,ar0144", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2640_of_match);

int ar0144_suspend(struct device *dev)
{
        struct i2c_client *client = container_of(dev, struct i2c_client, dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct ar0144_info *info = to_state(sd);
	int ret=0;

	ar0144_power(sd, 0);
	v4l2_clk_disable(info->clk);
        return 0;
}

int ar0144_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144_info *info = to_state(sd);
	int ret=0;

	ar0144_power(sd, 1);
	v4l2_clk_enable(info->clk);
	ar0144_reset(sd, 1);
	ar0144_init(sd, 1);
	return 0;
}

const struct dev_pm_ops ar0144_pm =
{
        .suspend = ar0144_suspend,
        .resume = ar0144_resume,
};


static struct i2c_driver ar0144_driver = {
	.driver = {
		.name	= "ar0144",
		.of_match_table = of_match_ptr(ar0144_of_match),
		.pm     = &ar0144_pm,
	},
	.probe		= ar0144_probe,
	.remove		= ar0144_remove,
	.id_table	= ar0144_id,
};

module_i2c_driver(ar0144_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for onsemi ar0144 sensors");
MODULE_LICENSE("GPL");
