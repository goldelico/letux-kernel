// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2014 Linaro Ltd
 *
 * Author: Ulf Hansson <ulf.hansson@linaro.org>
 *
 *  ESP32 SDIO power sequence management
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/reset.h>

#include <linux/mmc/host.h>

#include "pwrseq.h"

struct mmc_pwrseq_esp32 {
	struct mmc_pwrseq pwrseq;
	bool clk_enabled;
	u32 post_power_on_delay_ms;
	u32 power_off_delay_us;
	struct clk *ext_clk;
	struct gpio_descs *reset_gpios;
	struct gpio_desc *flashing_gpio;
	struct reset_control *reset_ctrl;
};

#define to_pwrseq_esp32(p) container_of(p, struct mmc_pwrseq_esp32, pwrseq)

static void mmc_pwrseq_esp32_set_gpios_value(struct mmc_pwrseq_esp32 *pwrseq,
					      int value)
{
	struct gpio_descs *reset_gpios = pwrseq->reset_gpios;

	if (!IS_ERR(reset_gpios)) {
		unsigned long *values;
		int nvalues = reset_gpios->ndescs;

		values = bitmap_alloc(nvalues, GFP_KERNEL);
		if (!values)
			return;

		if (value)
			bitmap_fill(values, nvalues);
		else
			bitmap_zero(values, nvalues);

		gpiod_multi_set_value_cansleep(reset_gpios, values);

		bitmap_free(values);
	}
}

static void mmc_pwrseq_esp32_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_esp32 *pwrseq = to_pwrseq_esp32(host->pwrseq);

	if (!IS_ERR(pwrseq->ext_clk) && !pwrseq->clk_enabled) {
		clk_prepare_enable(pwrseq->ext_clk);
		pwrseq->clk_enabled = true;
	}

	if (pwrseq->reset_ctrl) {
		reset_control_deassert(pwrseq->reset_ctrl);
		reset_control_assert(pwrseq->reset_ctrl);
	} else
		mmc_pwrseq_esp32_set_gpios_value(pwrseq, 1);
}

static void mmc_pwrseq_esp32_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_esp32 *pwrseq = to_pwrseq_esp32(host->pwrseq);

	if (pwrseq->reset_ctrl)
		reset_control_deassert(pwrseq->reset_ctrl);
	else
		mmc_pwrseq_esp32_set_gpios_value(pwrseq, 0);

	if (pwrseq->post_power_on_delay_ms)
		msleep(pwrseq->post_power_on_delay_ms);
}

static void mmc_pwrseq_esp32_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_esp32 *pwrseq = to_pwrseq_esp32(host->pwrseq);

	if (pwrseq->reset_ctrl)
		reset_control_assert(pwrseq->reset_ctrl);
	else
		mmc_pwrseq_esp32_set_gpios_value(pwrseq, 1);

	if (pwrseq->power_off_delay_us)
		usleep_range(pwrseq->power_off_delay_us,
			2 * pwrseq->power_off_delay_us);

	if (!IS_ERR(pwrseq->ext_clk) && pwrseq->clk_enabled) {
		clk_disable_unprepare(pwrseq->ext_clk);
		pwrseq->clk_enabled = false;
	}
}

static const struct mmc_pwrseq_ops mmc_pwrseq_esp32_ops = {
	.pre_power_on = mmc_pwrseq_esp32_pre_power_on,
	.post_power_on = mmc_pwrseq_esp32_post_power_on,
	.power_off = mmc_pwrseq_esp32_power_off,
};

static const struct of_device_id mmc_pwrseq_esp32_of_match[] = {
	{ .compatible = "mmc-pwrseq-esp32",},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_esp32_of_match);

static void mmc_delay(unsigned int ms)
{
	if (ms <= 20)
		usleep_range(ms * 1000, ms * 1250);
	else
		msleep(ms);
}

static ssize_t enable_flashing_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct mmc_pwrseq_esp32 *pwrseq = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", gpiod_get_value_cansleep(pwrseq->flashing_gpio));
}

static ssize_t enable_flashing_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{ /* here we can control the IO0 pin of the pwrseq directly for flashing purposes */
	struct mmc_pwrseq_esp32 *pwrseq = dev_get_drvdata(dev);
	unsigned int value = 0;
	int err;

	err = kstrtouint(buf, 10, &value);
	if (err < 0)
		return err;

	err = gpiod_set_value_cansleep(pwrseq->flashing_gpio, value);
	if (err < 0)
		return err;

	return count;
}
static DEVICE_ATTR_RW(enable_flashing);

static ssize_t enable_power_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return -EINVAL;
}

static ssize_t enable_power_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{ /* here we can control the RESET pin of the pwrseq directly for flashing purposes but circumvent mmc power management */
	struct mmc_pwrseq *pwrseq = dev_get_drvdata(dev);
	unsigned int value = 0;
	int err;
	struct mmc_host dummy = {
		.pwrseq = pwrseq,
		.ios.power_delay_ms = 10,
		.ios.power_mode = MMC_POWER_UNDEFINED
	};
	struct mmc_host *host = &dummy;

	err = kstrtouint(buf, 10, &value);
	if (err < 0)
		return err;

	if (value) { /* power on */
		if (pwrseq->ops->pre_power_on)
			pwrseq->ops->pre_power_on(host);
		mmc_delay(host->ios.power_delay_ms);
		if (pwrseq->ops->post_power_on)
			pwrseq->ops->post_power_on(host);
		mmc_delay(host->ios.power_delay_ms);
	} else {
		if (pwrseq->ops->power_off)
			pwrseq->ops->power_off(host);
		mmc_delay(1);
	}

	return count;
}
static DEVICE_ATTR_RW(enable_power);

static const struct attribute * esp32_attrs[] = {
	&dev_attr_enable_flashing.attr,
	&dev_attr_enable_power.attr,
	NULL
};


static int mmc_pwrseq_esp32_probe(struct platform_device *pdev)
{
	struct mmc_pwrseq_esp32 *pwrseq;
	struct device *dev = &pdev->dev;
	int ngpio;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	pwrseq->ext_clk = devm_clk_get(dev, "ext_clock");
	if (IS_ERR(pwrseq->ext_clk) && PTR_ERR(pwrseq->ext_clk) != -ENOENT)
		return dev_err_probe(dev, PTR_ERR(pwrseq->ext_clk), "external clock not ready\n");

	ngpio = of_count_phandle_with_args(dev->of_node, "reset-gpios", "#gpio-cells");
	if (ngpio == 1) {
		pwrseq->reset_ctrl = devm_reset_control_get_optional_shared(dev, NULL);
		if (IS_ERR(pwrseq->reset_ctrl))
			return dev_err_probe(dev, PTR_ERR(pwrseq->reset_ctrl),
					     "reset control not ready\n");
	}

	/*
	 * Fallback to GPIO based reset control in case of multiple reset lines
	 * are specified or the platform doesn't have support for RESET at all.
	 */
	if (!pwrseq->reset_ctrl) {
		pwrseq->reset_gpios = devm_gpiod_get_array(dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(pwrseq->reset_gpios) &&
		    PTR_ERR(pwrseq->reset_gpios) != -ENOENT &&
		    PTR_ERR(pwrseq->reset_gpios) != -ENOSYS) {
			return dev_err_probe(dev, PTR_ERR(pwrseq->reset_gpios),
					     "reset GPIOs not ready\n");
		}
	}

	pwrseq->flashing_gpio = devm_gpiod_get(dev, "flashing", GPIOD_OUT_LOW);	// initially deactivated
	if (IS_ERR(pwrseq-> flashing_gpio))
		return dev_err_probe(dev, PTR_ERR(pwrseq->flashing_gpio),
					     "flashing GPIO not ready\n");

	device_property_read_u32(dev, "post-power-on-delay-ms",
				 &pwrseq->post_power_on_delay_ms);
	device_property_read_u32(dev, "power-off-delay-us",
				 &pwrseq->power_off_delay_us);

	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_esp32_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	int ret = sysfs_create_files(&pdev->dev.kobj, esp32_attrs);
	if (ret)
		dev_err(&pdev->dev, "unable to create sysfs files\n");

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static void mmc_pwrseq_esp32_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_esp32 *pwrseq = platform_get_drvdata(pdev);

	sysfs_remove_files(&pdev->dev.kobj, esp32_attrs);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);
}

static struct platform_driver mmc_pwrseq_esp32_driver = {
	.probe = mmc_pwrseq_esp32_probe,
	.remove = mmc_pwrseq_esp32_remove,
	.driver = {
		.name = "pwrseq_esp32",
		.of_match_table = mmc_pwrseq_esp32_of_match,
	},
};

module_platform_driver(mmc_pwrseq_esp32_driver);
MODULE_DESCRIPTION("ESP32 power sequence management for SDIO");
MODULE_LICENSE("GPL v2");
