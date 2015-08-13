/*
 * OMAP SoC COPROC  driver
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *		Carlos Hernandez <ceh@ti.com>
 *		Nishanth Menon <nm@ti.com>
 *		Ravikumar Kattekola <rk@ti.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/pm_opp.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/pm_voltage_domain.h>

/**
 * struct coproc_data - Co processor private data
 * @profile:   profile specific for this device
 * @dev:  device pointer
 * @:   for this device
 * @stat: my current statistics
 * @dev_clk: my device clk
 * @dpll_clk: my dpll clk
 * @nb:	my notifier block
 */
struct coproc_data {
	struct device *dev;
	struct clk *dev_clk;
	struct clk *dpll_clk;
	struct notifier_block *clk_nb;
};

/**
 * DOC:
 * clock-names should be defined in dts file, e.g.
 * coproc {
 *
 *  compatible = "ti,coproc";
 *  clocks = <&dpll_iva_m2_ck>, <&dpll_iva_ck>;
 *  clock-names = "fclk", "dpll";
 *  clock-target-frequency = <532000000>;
 *  operating-points = <
 *    388200  1055000
 *    500000  1150000
 *    532000  1250000
 *  >;
 *  coproc-voltdm = <&voltdm_ivahd>;
 *  voltage-tolerance = <1>;
 *
 * };
 */
#define DEV_CLK_NAME "fclk"
#define DPLL_CLK_NAME "dpll"

static int coproc_probe(struct platform_device *pdev)
{
	struct coproc_data *d;
	unsigned int voltage_latency;
	u32 target_freq = 0;
	int err = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = of_node_get(dev->of_node);
	bool noset_dpll_as_rate;

	dev_info(dev, "probe\n");

	of_property_read_u32(np, "clock-target-frequency", &target_freq);
	if (!target_freq) {
		dev_err(dev, "%s: Invalid/no target frequency found in dt.\n",
			__func__);
		return -EINVAL;
	}

	noset_dpll_as_rate = of_property_read_bool(np, "noset-dpll-rate");

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (d == NULL) {
		dev_err(dev, "%s: Cannot allocate memory.\n", __func__);
		err = -ENOMEM;
		goto out;
	}
	platform_set_drvdata(pdev, d);

	d->dpll_clk = devm_clk_get(dev, DPLL_CLK_NAME);
	if (IS_ERR(d->dpll_clk)) {
		err = PTR_ERR(d->dpll_clk);
		dev_err(dev, "%s: Cannot get dpll clk(%d).\n", __func__, err);
		d->dpll_clk = NULL;
	}

	d->dev_clk = devm_clk_get(dev, DEV_CLK_NAME);
	if (IS_ERR(d->dev_clk)) {
		err = PTR_ERR(d->dpll_clk);
		dev_err(dev, "%s: Cannot get func clk(%d).\n", __func__, err);
		goto out;
	}

	if (noset_dpll_as_rate)
		d->dpll_clk = NULL;

	err = of_init_opp_table(dev);
	if (err) {
		dev_err(dev, "%s: Cannot initialize opp table(%d).\n", __func__,
			err);
		goto free_opp;
	}
	d->dev = dev;

	/* Register voltage domain notifier */
	d->clk_nb = of_pm_voltdm_notifier_register(dev, np, d->dev_clk,
						   "coproc",
						   &voltage_latency);
	if (IS_ERR(d->clk_nb)) {
		err = PTR_ERR(d->clk_nb);
		/* defer probe if regulator is not yet registered */
		if (err == -EPROBE_DEFER) {
			dev_err(dev,
				"coproc clock notifier not ready, retry\n");
		} else {
			dev_err(dev,
				"Failed to register coproc clk notifier: %d\n",
				err);
		}
		goto free_opp;
	}

	if (target_freq) {
		if (d->dpll_clk) {
			err = clk_set_rate(d->dpll_clk, target_freq);
			if (err) {
				dev_err(dev, "%s: Cannot set dpll clock rate(%d).\n",
					__func__, err);
				goto free_vm;
			}
		}

		err = clk_set_rate(d->dev_clk, target_freq);
		if (err) {
			dev_err(dev, "%s: Cannot set func clock rate(%d).\n",
				__func__, err);
			goto free_vm;
		}
	}

	/* All good.. */
	goto out;

free_vm:
	of_pm_voltdm_notifier_unregister(d->clk_nb);
	d->clk_nb = NULL;
free_opp:
	of_free_opp_table(dev);
out:
	dev_info(dev, "%s result=%d", __func__, err);
	return err;
}

static int coproc_remove(struct platform_device *pdev)
{
	struct coproc_data *d = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, "remove\n");

	if (!IS_ERR_OR_NULL(d->clk_nb))
		of_pm_voltdm_notifier_unregister(d->clk_nb);

	of_free_opp_table(dev);
	dev_info(dev, "%s Removed\n", __func__);
	return 0;
}

/**
 * coproc_suspend() - dummy hook for suspend
 * @dev: device pointer
 *
 * Return: 0
 */
static int coproc_suspend(struct device *dev)
{
	dev_info(dev, "suspend\n");
	return 0;
}

/**
 * coproc_resume() - dummy hook for resume
 * @dev: device pointer
 *
 * Return: 0
 */
static int coproc_resume(struct device *dev)
{
	dev_info(dev, "resume\n");
	return 0;
}

/* Device power management hooks */
static SIMPLE_DEV_PM_OPS(coproc_pm,
			 coproc_suspend,
			 coproc_resume);

static struct of_device_id of_coproc_match[] = {
	{.compatible = "ti,coproc"},
	{},
};

MODULE_DEVICE_TABLE(of, of_coproc_match);

static struct platform_driver coproc_driver = {
	.probe = coproc_probe,
	.remove = coproc_remove,
	.driver = {
		   .name = "coproc",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(of_coproc_match),
		   .pm = &coproc_pm,
		   },
};
module_platform_driver(coproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Co-processor Driver");
MODULE_AUTHOR("Texas Instruments Inc.");
