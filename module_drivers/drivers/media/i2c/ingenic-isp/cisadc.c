
#define DEBUG

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

#include <isp-sensor.h>

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct cisadc_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct cisadc_gpio {
	int pin;
	int active_level;
};

struct cisadc_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct cisadc_win_size *win;

	struct cisadc_gpio reset;
	struct cisadc_gpio ircutp;
	struct cisadc_gpio ircutn;
};


/* scanner cis 300dpi/600dpi/1200dpi */
static struct cisadc_win_size cisadc_win_sizes[] = {
	/* 300dpi */
	{
		.width		= 5376, // 300dpi, dma 32bytes aligned
		.height		= 1,
		.sensor_info.fps		= 30 << 16 | 1,
		/* .mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8, */
		/* .colorspace	= V4L2_COLORSPACE_SRGB, */
		/* .mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8, */
		/* .colorspace	= V4L2_COLORSPACE_DEFAULT, */
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.colorspace	= V4L2_COLORSPACE_RAW,
	},
	/* 600dpi */
	{
		.width		= 10556, // 600dpi, dma 32bytes aligned
		.height		= 1,
		.sensor_info.fps		= 30 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.colorspace	= V4L2_COLORSPACE_RAW,
	},
	/* 1200dpi */
	{
		.width		= 20920, // 1200dpi, dma 32bytes aligned
		.height		= 1,
		.sensor_info.fps		= 30 << 16 | 1,
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.colorspace	= V4L2_COLORSPACE_RAW,
	},
	/* user set xdpi, width/height */
	{
		.width		= 0, // 1200dpi, dma 32bytes aligned
		.height		= 0,
		.sensor_info.fps		= 30 << 16 | 1,
		/* .mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8, */
		/* .colorspace	= V4L2_COLORSPACE_SRGB, */
		/* .mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8, */
		/* .colorspace	= V4L2_COLORSPACE_DEFAULT, */
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.colorspace	= V4L2_COLORSPACE_RAW,
	},
};

static inline struct cisadc_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct cisadc_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct cisadc_info, hdl)->sd;
}

#if 0
/*
 * Stuff that knows about the sensor.
 */
static int cisadc_reset(struct v4l2_subdev *sd, u32 val)
{
	struct cisadc_info *info = to_state(sd);
#if 0
	if(val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
#endif
	return 0;
}

static int cisadc_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct cisadc_info *info = to_state(sd);
#if 0
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
#endif
	return 0;
}

static int cisadc_init(struct v4l2_subdev *sd, u32 val)
{
	struct cisadc_info *info = to_state(sd);
	int ret = 0;

	//ret = cisadc_write_array(sd, info->win->regs);

	return ret;
}



static int cisadc_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	pr_debug("-----%s: %d ret = %d, v = 0x%02x return 0;\n", __func__, __LINE__, ret,v);

	return 0;
}
#endif

static int cisadc_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_CISADC_FMTS)
		return -EINVAL;

	code->code = cisadc_formats[code->index].mbus_code;
#endif
	return 0;
}

static const struct cisadc_win_size *cisadc_select_win(u32 *width, u32 *height)
{
	int i, default_size, win_size;
	default_size = 0;
	win_size = ARRAY_SIZE(cisadc_win_sizes);
	//win_size -= 1;
	for (i = 0; i < win_size; i++) {
		if ((*width <= cisadc_win_sizes[i].width) &&
				(*height <= cisadc_win_sizes[i].height)) {
			*width = cisadc_win_sizes[i].width;
			*height = cisadc_win_sizes[i].height;
			return &cisadc_win_sizes[i];
		}
	}

	*width = cisadc_win_sizes[default_size].width;
	*height = cisadc_win_sizes[default_size].height;
	return &cisadc_win_sizes[default_size];
}

/*
 * Set a format.
 */
static int cisadc_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct cisadc_format_struct *ovfmt;
	struct cisadc_win_size *wsize;
	struct cisadc_info *info = to_state(sd);
	int ret;
	int win_size;

	win_size = ARRAY_SIZE(cisadc_win_sizes);
	dev_info(sd->dev, "format width=%d, height=%d, code=0x%x, format->pad=%p, win_size=%d\n", format->format.width, format->format.height, format->format.code, format->pad, win_size);

	win_size--;
	format->format.width = cisadc_win_sizes[win_size].width = format->format.width;
	format->format.height = cisadc_win_sizes[win_size].height = format->format.height;

	info->win = &cisadc_win_sizes[win_size];
	//info->win = cisadc_select_win(&format->format.width, &format->format.height);

	return 0;
}

static int cisadc_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct cisadc_info *info = to_state(sd);
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


	printk("----%s, %d, width: %d, height: %d, code: %x\n",
			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

#if 0
static int cisadc_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int cisadc_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int cisadc_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int cisadc_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int cisadc_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;

	return ret;

}
/*set analog gain db value, map value to sensor register.*/
static int cisadc_s_again(struct v4l2_subdev *sd, int value)
{
	struct cisadc_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	return 0;
}

/*
 * Tweak autogain.
 */
static int cisadc_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int cisadc_s_exp(struct v4l2_subdev *sd, int value)
{
	struct cisadc_info *info = to_state(sd);
	int ret = 0;
	if(info->exposure->val != value) {
		ret = cisadc_write(sd, 0xfd, 0x01);
		ret += cisadc_write(sd, 0x4, (unsigned char)(value & 0xff));
		ret += cisadc_write(sd, 0x3, (unsigned char)((value & 0xff00) >> 8));
		ret += cisadc_write(sd, 0x01, 0x01);
	}
	if (ret < 0) {
		printk("cisadc_write error  %d\n" ,__LINE__);
		return ret;
	}
	return ret;
}
#endif

#if 0
static int cisadc_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
#if 0
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct cisadc_info *info = to_state(sd);
	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return cisadc_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return cisadc_g_again(sd, &info->again->val);
	}
#endif
	return -EINVAL;
}

static int cisadc_s_ctrl(struct v4l2_ctrl *ctrl)
{
#if 0
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct cisadc_info *info = to_state(sd);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return cisadc_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return cisadc_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return cisadc_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return cisadc_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* cisadc_s_gain turns off auto gain */
			return cisadc_s_gain(sd, info->gain->val);
		}
		return cisadc_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return cisadc_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return cisadc_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return cisadc_s_exp(sd, ctrl->val);
	}
#endif
	return -EINVAL;
}

static const struct v4l2_ctrl_ops cisadc_ctrl_ops = {
	.s_ctrl = cisadc_s_ctrl,
	.g_volatile_ctrl = cisadc_g_volatile_ctrl,
};
#endif

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int cisadc_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	//ret = cisadc_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int cisadc_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	//cisadc_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int cisadc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct cisadc_info *info = to_state(sd);
	int ret = 0;

	return ret;
}

int cisadc_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct cisadc_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops cisadc_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = cisadc_g_register,
	.s_register = cisadc_s_register,
#endif
};

static const struct v4l2_subdev_video_ops cisadc_video_ops = {
	.s_stream = cisadc_s_stream,
	.g_frame_interval = cisadc_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops cisadc_pad_ops = {
	//.enum_frame_interval = cisadc_enum_frame_interval,
	//.num_frame_size = cisadc_enum_frame_size,
	//.enum_mbus_code = cisadc_enum_mbus_code,
	.set_fmt = cisadc_set_fmt,
	.get_fmt = cisadc_get_fmt,
};

static const struct v4l2_subdev_ops cisadc_ops = {
	.core = &cisadc_core_ops,
	.video = &cisadc_video_ops,
	.pad = &cisadc_pad_ops,
};

/* ----------------------------------------------------------------------- */
extern int isp_clk;

static int cisadc_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct cisadc_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;
	unsigned int ispcdr;

	/*
	 * after system boot up, #devmem 0x10000030 32 0x8000000f
	 * to set vic clock on.
	 */
	isp_clk = 4710000;	/* isp clk = cis-clk/2 = 9.3MHz/2 = 4.7MHz */
	ispcdr = *(unsigned int*)0xB0000030;
	v4l_info(client, "%s() isp_clk=%d, ispcdr=0x%08x\n",
		 __func__, isp_clk, ispcdr);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	v4l2_i2c_subdev_init(sd, client, &cisadc_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#if 0
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

	cisadc_reset(sd, 1);

	/* Make sure it's an cisadc */
	ret = cisadc_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an cisadc chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}

	/*IRCUT ctl 0:off 1:on*/
	cisadc_ircut(sd, 0);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &cisadc_ctrl_ops,
			V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);
#endif
	info->win = &cisadc_win_sizes[0];
	//cisadc_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "cisadc Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int cisadc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cisadc_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id cisadc_id[] = {
	{ "cisadc", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, cisadc_id);

static const struct of_device_id cisadc_of_match[] = {
	{.compatible = "ingenic,cisadc", },
	{},
};

static struct i2c_driver cisadc_driver = {
	.driver = {
		.name	= "cisadc",
		.of_match_table = of_match_ptr(cisadc_of_match),
	},
	.probe		= cisadc_probe,
	.remove		= cisadc_remove,
	.id_table	= cisadc_id,
};

module_i2c_driver(cisadc_driver);
MODULE_AUTHOR("Justin <linggang.wang@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for CIS ADC");
MODULE_LICENSE("GPLv2");
