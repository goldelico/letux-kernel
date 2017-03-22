/*
 * txs02612.c
 * Driver for controlling the txs02612 sd level shifter and switch.
 *
 * currently, this is a very simple switch - it just provides a /sysfs node
 * to throw the switch.
 *
 * in the future this should become a MMC card driver (similar to e.g. WLAN
 * chips) and register as two new MMC ports to the system.
 *
 * switching should be done on the fly, i.e. block by block request, so that
 * both cards can be read or written in parallel and simply appear as two
 * separate drives (mmcblk) in user space.
 *
 * so we should probably convert to use sdio_register_driver()
 * maybe this is a hint: http://www.varsanofiev.com/inside/WritingLinuxSDIODrivers.htm
 * and of course http://lxr.free-electrons.com/source/drivers/mmc/core/
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#undef pr_debug
#define pr_debug printk

struct txs_data {
	int		control_gpio;	/* the control gpio number */
};

static ssize_t set_switch(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct txs_data *data = dev_get_drvdata(dev);
	unsigned long val;
// FIXME: we could decode the state as string (eMMC vs uSD)
	int err = kstrtoul(buf, 10, &val);

	pr_debug("%s() to %ld\n", __func__, val);

	if (err)
		return err;
	if (val > 1)
		return -EINVAL;

	gpio_set_value_cansleep(data->control_gpio, val);

// maybe: mmc_detect_change(host, delay);

	return count;
}

static ssize_t show_switch(struct device *dev,
			struct device_attribute *attr, char *buf)
	{
	struct txs_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", gpio_get_value(data->control_gpio)?"uSD":"eMMC");
	}

static DEVICE_ATTR(switch, S_IWUSR | S_IRUGO,
		show_switch, set_switch);

static struct attribute *txs_attributes[] = {
	&dev_attr_switch.attr,
	NULL
};

static const struct attribute_group txs_attr_group = {
	.attrs = txs_attributes,
};

static int txs_probe(struct platform_device *dev)
{
	struct txs_data *data;
	enum of_gpio_flags flags;
	int err;
	int initial;

	pr_debug("%s()\n", __func__);

	data = devm_kzalloc(&dev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		goto out;
	}

	err = data->control_gpio = of_get_named_gpio_flags(dev->dev.of_node,
						"select-gpio", 0,
							&flags);

//	if (err == -EPROBE_DEFER)
//		return -EPROBE_DEFER;
	if (err < 0)
		goto out;	/* defer until we have all gpios */

	err = devm_gpio_request(&dev->dev, data->control_gpio, "txs-control");
	if (err < 0)
		goto out;

	initial = of_property_read_bool(dev->dev.of_node, "switch-to-b");
	gpio_direction_output(data->control_gpio, initial);
	pr_debug("%s() initial = %d\n", __func__, initial);


	platform_set_drvdata(dev, data);

       /* Register sysfs hooks */
	err = sysfs_create_group(&dev->dev.kobj, &txs_attr_group);
	if (err)
		goto out;

	pr_debug("%s() probed\n", __func__);

	return 0;

out:
	pr_debug("%s() error %d\n", __func__, err);

	return err;
}

static int txs_remove(struct platform_device *dev)
{
	struct txs_data *data = platform_get_drvdata(dev);

	sysfs_remove_group(&dev->dev.kobj, &txs_attr_group);

	return 0;
}

static int txs_suspend(struct device *dev)
{
	struct txs_data *data = dev_get_drvdata(dev);

	return 0;
}

static int txs_resume(struct device *dev)
{
	struct txs_data *data = dev_get_drvdata(dev);

	return 0;
}

static const struct of_device_id txs_of_match[] = {
	{ .compatible = "ti,txs02612" },
	{},
};
MODULE_DEVICE_TABLE(of, txs_of_match);

SIMPLE_DEV_PM_OPS(txs_pm_ops, txs_suspend, txs_resume);

static struct platform_driver txs_driver = {
	.probe		= txs_probe,
	.remove		= txs_remove,
	.driver = {
		.name	= "txs02612",
		.owner	= THIS_MODULE,
		.pm	= &txs_pm_ops,
		.of_match_table = of_match_ptr(txs_of_match)
	},
};

module_platform_driver(txs_driver);

MODULE_ALIAS("platform:txs02612");

MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("txs02612 SD level shifter and switch");
MODULE_LICENSE("GPL v2");
