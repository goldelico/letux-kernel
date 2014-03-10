#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include <linux/pm_runtime.h>

struct fpdlink_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

static const struct fpdlink_color_format fpdlink_cfmts[] = {
	{
		.code           = V4L2_MBUS_FMT_SBGGR8_1X8,
		.colorspace     = V4L2_COLORSPACE_SRGB,
	},
};

static int fpdlink_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(fpdlink_cfmts))
		return -EINVAL;

	*code = fpdlink_cfmts[0].code;

	return 0;
}

static int fpdlink_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	return 0;
}

static int fpdlink_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	return 0;
}

static int fpdlink_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	return 0;
}

static int fpdlink_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static struct v4l2_subdev_video_ops fpdlink_video_ops = {
	.enum_mbus_fmt	= fpdlink_enum_fmt,
	.g_mbus_fmt	= fpdlink_g_fmt,
	.try_mbus_fmt	= fpdlink_try_fmt,
	.s_mbus_fmt	= fpdlink_s_fmt,
	.s_stream	= fpdlink_s_stream,
};

static struct v4l2_subdev_ops fpdlink_subdev_ops = {
	.video		= &fpdlink_video_ops,
};

static const struct i2c_device_id fpdlink_id[] = {
	{ "ti,fpdlink", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fpdlink_id);

static const struct of_device_id fpdlink_dt_id[] = {
	{
	.compatible   = "ti,fpdlink"
	},
	{
	}
};

static int fpdlink_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct v4l2_subdev *sd;
	int ret = -1;

	sd = devm_kzalloc(&client->dev, sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	v4l2_i2c_subdev_init(sd, client, &fpdlink_subdev_ops);

	/* V4l2 asyn subdev register */
	sd->dev = &client->dev;
	ret = v4l2_async_register_subdev(sd);
	if (!ret) {
		i2c_set_clientdata(client, sd);
		v4l2_info(sd, "Camera sensor driver registered\n");
		pm_runtime_enable(&client->dev);
	}

	return ret;
}

static int fpdlink_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	if (sd)
		v4l2_async_unregister_subdev(sd);

	return 0;
}

static struct i2c_driver fpdlink_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "fpdlink",
		.of_match_table = fpdlink_dt_id,
	},
	.probe    = fpdlink_probe,
	.remove   = fpdlink_remove,
	.id_table = fpdlink_id,
};

module_i2c_driver(fpdlink_i2c_driver);

MODULE_DESCRIPTION("FPDLink display interface");
MODULE_AUTHOR("Nikhil Devshatwar");
MODULE_LICENSE("GPL v2");
