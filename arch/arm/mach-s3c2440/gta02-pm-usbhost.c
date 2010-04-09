/*
 * USBHOST Management code for the Openmoko Freerunner GSM Phone
 *
 * (C) 2007 by Openmoko Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>

#include <mach/gta02.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

static struct regulator *gta02_usbhost_regulator;

static ssize_t usbhost_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if (!strcmp(attr->attr.name, "power_on")) {
		if (regulator_is_enabled(gta02_usbhost_regulator))
			goto out_1;
	}

	return strlcpy(buf, "0\n", 3);
out_1:
	return strlcpy(buf, "1\n", 3);
}

static void usbhost_on_off(struct device *dev, int on)
{

	on = !!on;

	if (on == regulator_is_enabled(gta02_usbhost_regulator))
		return;

	if (!on) {
		regulator_disable(gta02_usbhost_regulator);
		return;
	}

	regulator_enable(gta02_usbhost_regulator);
}

static ssize_t usbhost_write(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		usbhost_on_off(dev, on);

		return count;
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, usbhost_read, usbhost_write);

#ifdef CONFIG_PM

static int gta02_usbhost_suspend(struct device *dev)
{
	return 0;
}

static int gta02_usbhost_suspend_late(struct device *dev)
{
	return 0;
}

static int gta02_usbhost_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops gta02_usbhost_pm_ops = {
	.suspend	= gta02_usbhost_suspend,
	.suspend_noirq	= gta02_usbhost_suspend_late,
	.resume		= gta02_usbhost_resume,
};

#define GTA02_USBHOST_PM_OPS (&gta02_usbhost_pm_ops)

#else
#define GTA02_USBHOST_PM_OPS NULL
#endif /* CONFIG_PM */

static struct attribute *gta02_usbhost_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	NULL
};

static struct attribute_group gta02_usbhost_attr_group = {
	.name	= NULL,
	.attrs	= gta02_usbhost_sysfs_entries,
};

static int __init gta02_usbhost_probe(struct platform_device *pdev)
{
	int ret;

	gta02_usbhost_regulator = regulator_get_exclusive(&pdev->dev, "USBHOST");

	if (IS_ERR(gta02_usbhost_regulator)) {
		ret = PTR_ERR(gta02_usbhost_regulator);
		dev_err(&pdev->dev, "Failed to get regulator: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &gta02_usbhost_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs entries: %d\n", ret);
		return ret;
	}

	return 0;
}

static int gta02_usbhost_remove(struct platform_device *pdev)
{
	usbhost_on_off(&pdev->dev, 0);

	sysfs_remove_group(&pdev->dev.kobj, &gta02_usbhost_attr_group);
	regulator_put(gta02_usbhost_regulator);

	return 0;
}

static struct platform_driver gta02_usbhost_driver = {
	.probe		= gta02_usbhost_probe,
	.remove		= gta02_usbhost_remove,
	.driver		= {
		.name	= "gta02-pm-usbhost",
		.pm	= GTA02_USBHOST_PM_OPS,
	},
};

static int __devinit gta02_usbhost_init(void)
{
	return platform_driver_register(&gta02_usbhost_driver);
}
module_init(gta02_usbhost_init);

static void gta02_usbhost_exit(void)
{
	platform_driver_unregister(&gta02_usbhost_driver);
}
module_exit(gta02_usbhost_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Openmoko Freerunner USBHOST Power Management");
