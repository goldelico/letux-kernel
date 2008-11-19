/*
 * GTA02 WLAN power management
 *
 * (C) 2008 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
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
#include <asm/plat-s3c24xx/neo1973.h>

#include <mach/gta02.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

#include <linux/delay.h>


static void gta02_wlan_power(int on)
{
	if (!on) {
		s3c2410_gpio_setpin(GTA02_CHIP_PWD, 1);
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 0);
		return;
	}

	/* power up sequencing */

	s3c2410_gpio_setpin(GTA02_CHIP_PWD, 1);
	s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 0);
	msleep(100);
	s3c2410_gpio_setpin(GTA02_CHIP_PWD, 0);
	msleep(100);
	s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 1);

}

static ssize_t gta02_wlan_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (s3c2410_gpio_getpin(GTA02_CHIP_PWD))
		return strlcpy(buf, "0\n", 3);

	return strlcpy(buf, "1\n", 3);
}

static ssize_t gta02_wlan_write(struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10) & 1;

	gta02_wlan_power(on);
	return count;
}

static DEVICE_ATTR(power_on, 0644, gta02_wlan_read, gta02_wlan_write);

#ifdef CONFIG_PM
static int gta02_wlan_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, "suspending\n");

	return 0;
}

static int gta02_wlan_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "resuming\n");

	return 0;
}
#else
#define gta02_wlan_suspend	NULL
#define gta02_wlan_resume		NULL
#endif

static struct attribute *gta02_wlan_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	NULL
};

static struct attribute_group gta02_wlan_attr_group = {
	.name	= NULL,
	.attrs	= gta02_wlan_sysfs_entries,
};

static int __init gta02_wlan_probe(struct platform_device *pdev)
{
	/* default-on for now */
	const int default_state = 1;

	if (!machine_is_neo1973_gta02())
		return -EINVAL;

	dev_info(&pdev->dev, "starting\n");

	s3c2410_gpio_cfgpin(GTA02_CHIP_PWD, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(GTA02_GPIO_nWLAN_RESET, S3C2410_GPIO_OUTPUT);
	gta02_wlan_power(default_state);

	return sysfs_create_group(&pdev->dev.kobj, &gta02_wlan_attr_group);
}

static int gta02_wlan_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gta02_wlan_attr_group);

	return 0;
}

static struct platform_driver gta02_wlan_driver = {
	.probe		= gta02_wlan_probe,
	.remove		= gta02_wlan_remove,
	.suspend	= gta02_wlan_suspend,
	.resume		= gta02_wlan_resume,
	.driver		= {
		.name		= "gta02-pm-wlan",
	},
};

static int __devinit gta02_wlan_init(void)
{
	return platform_driver_register(&gta02_wlan_driver);
}

static void gta02_wlan_exit(void)
{
	platform_driver_unregister(&gta02_wlan_driver);
}

module_init(gta02_wlan_init);
module_exit(gta02_wlan_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Openmoko GTA02 WLAN power management");
