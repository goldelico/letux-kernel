/*
 * OPA362 analog video amplifier with output/power control
 *
 * Copyright (C) 2013 Golden Delicious Computers
 * Author: H. Nikolaus Schaller <hns@goldelico.com>
 *
 * based on encoder-tfp410
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	int enable_gpio;
	bool bypass;
	bool acbias;

	struct omap_video_timings timings;
};

#define to_panel_data(x) container_of(x, struct panel_drv_data, dssdev)

static int opa362_connect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return -EBUSY;

	r = in->ops.atv->connect(in, dssdev);
	if (r)
		return r;

	dst->src = dssdev;
	dssdev->dst = dst;

	return 0;
}

static void opa362_disconnect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	WARN_ON(!omapdss_device_is_connected(dssdev));
	if (!omapdss_device_is_connected(dssdev))
		return;

	WARN_ON(dst != dssdev->dst);
	if (dst != dssdev->dst)
		return;

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.atv->disconnect(in, &ddata->dssdev);
}

static int opa362_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.atv->set_timings(in, &ddata->timings);
	/* fixme: should we receive the invert from our consumer, i.e. the connector? */
	in->ops.atv->invert_vid_out_polarity(in, true);
	in->ops.atv->bypass_and_acbias(in, ddata->bypass, ddata->acbias);

	r = in->ops.atv->enable(in);
	if (r)
		return r;

	if (gpio_is_valid(ddata->enable_gpio))
		gpio_set_value_cansleep(ddata->enable_gpio, 1);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void opa362_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	if (gpio_is_valid(ddata->enable_gpio))
		gpio_set_value_cansleep(ddata->enable_gpio, 0);

	in->ops.atv->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void opa362_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->timings = *timings;
	dssdev->panel.timings = *timings;

	in->ops.atv->set_timings(in, timings);
}

static void opa362_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->timings;
}

static int opa362_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.atv->check_timings(in, timings);
}

static void opa362_set_type(struct omap_dss_device *dssdev,
		enum omap_dss_venc_type type)
{
	/* we can only drive a COMPOSITE output */
	WARN_ON(type != OMAP_DSS_VENC_TYPE_COMPOSITE);

}

static void opa362_invert_vid_out_polarity(struct omap_dss_device *dssdev,
		bool invert_polarity)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	/* OPA362 inverts the polarity */
	in->ops.atv->invert_vid_out_polarity(in, !invert_polarity);
}

static const struct omapdss_atv_ops opa362_atv_ops = {
	.connect	= opa362_connect,
	.disconnect	= opa362_disconnect,

	.enable		= opa362_enable,
	.disable	= opa362_disable,

	.check_timings	= opa362_check_timings,
	.set_timings	= opa362_set_timings,
	.get_timings	= opa362_get_timings,

	.set_type	= opa362_set_type,
	.invert_vid_out_polarity	= opa362_invert_vid_out_polarity,
};

static int opa362_probe_pdata(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct amplifier_opa362_platform_data *pdata;
	struct omap_dss_device *dssdev, *in;

	pdata = dev_get_platdata(&pdev->dev);

	ddata->enable_gpio = pdata->enable_gpio;
	ddata->bypass = pdata->bypass;
	ddata->acbias = pdata->acbias;

	in = omap_dss_find_output(pdata->source);
	if (in == NULL) {
		dev_err(&pdev->dev, "Failed to find video source\n");
		return -ENODEV;
	}

	ddata->in = in;

	dssdev = &ddata->dssdev;
	dssdev->name = pdata->name;

	return 0;
}

static int opa362_probe(struct platform_device *pdev)
{
	struct panel_drv_data *ddata;
	struct omap_dss_device *dssdev;
	int r;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);

	if (dev_get_platdata(&pdev->dev)) {
		r = opa362_probe_pdata(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	if (gpio_is_valid(ddata->enable_gpio)) {
		r = devm_gpio_request_one(&pdev->dev, ddata->enable_gpio,
				GPIOF_OUT_INIT_LOW, "opa362 enable");
		if (r) {
			dev_err(&pdev->dev, "Failed to request enable GPIO %d\n",
					ddata->enable_gpio);
			goto err_gpio;
		}
	}

	dssdev = &ddata->dssdev;
	dssdev->ops.atv = &opa362_atv_ops;
	dssdev->dev = &pdev->dev;
	dssdev->type = OMAP_DISPLAY_TYPE_VENC;
	dssdev->output_type = OMAP_DISPLAY_TYPE_VENC;
	dssdev->owner = THIS_MODULE;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to register output\n");
		goto err_reg;
	}

	return 0;
err_reg:
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}

static int __exit opa362_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	omapdss_unregister_output(&ddata->dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		opa362_disable(dssdev);

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		opa362_disconnect(dssdev, dssdev->dst);

	omap_dss_put_device(in);

	return 0;
}

static struct platform_driver opa362_driver = {
	.probe	= opa362_probe,
	.remove	= __exit_p(opa362_remove),
	.driver	= {
		.name	= "amplifier-opa362",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(opa362_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("OPA362 analog video amplifier with output/power control");
MODULE_LICENSE("GPL");
