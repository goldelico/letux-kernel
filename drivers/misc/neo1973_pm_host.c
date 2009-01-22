/*
 * Bluetooth PM code for the FIC Neo1973 GSM Phone
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

#include <mach/hardware.h>
#include <asm/mach-types.h>

#ifdef CONFIG_MACH_NEO1973_GTA02
#include <mach/gta02.h>
#include <linux/pcf50633.h>

static ssize_t pm_host_read(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%d\n",
		       pcf50633_gpio_get(pcf50633_global, PCF50633_GPO));
}

static ssize_t pm_host_write(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	pcf50633_gpio_set(pcf50633_global, PCF50633_GPO, on);

	return count;
}

static DEVICE_ATTR(hostmode, 0644, pm_host_read, pm_host_write);

static struct attribute *neo1973_pm_host_sysfs_entries[] = {
	&dev_attr_hostmode.attr,
	NULL
};

static struct attribute_group neo1973_pm_host_attr_group = {
	.name	= NULL,
	.attrs	= neo1973_pm_host_sysfs_entries,
};

static int __init neo1973_pm_host_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "starting\n");

	switch (machine_arch_type) {
#ifdef CONFIG_MACH_NEO1973_GTA01
	case MACH_TYPE_NEO1973_GTA01:
		return -EINVAL;
#endif /* CONFIG_MACH_NEO1973_GTA01 */
	default:
		break;
	}

	return sysfs_create_group(&pdev->dev.kobj, &neo1973_pm_host_attr_group);
}

static int neo1973_pm_host_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &neo1973_pm_host_attr_group);
	return 0;
}

static struct platform_driver neo1973_pm_host_driver = {
	.probe		= neo1973_pm_host_probe,
	.remove		= neo1973_pm_host_remove,
	.driver		= {
		.name		= "neo1973-pm-host",
	},
};

static int __devinit neo1973_pm_host_init(void)
{
	return platform_driver_register(&neo1973_pm_host_driver);
}

static void neo1973_pm_host_exit(void)
{
	platform_driver_unregister(&neo1973_pm_host_driver);
}

module_init(neo1973_pm_host_init);
module_exit(neo1973_pm_host_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Neo1973 USB Host Power Management");
#endif
