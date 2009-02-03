/*
 * Bluetooth PM code for the FIC Neo1973 GSM Phone
 *
 * (C) 2007 by OpenMoko Inc.
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

#include <asm/hardware.h>
#include <asm/mach-types.h>

#ifdef CONFIG_MACH_NEO1973_GTA02
#include <asm/arch/gta02.h>
#include <linux/pcf50633.h>
#endif

static ssize_t pm_usbhost_read(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	return sprintf(buf, "%d\n",
		       pcf50633_gpio_get(pcf50633_global, PCF50633_GPO));
}

static ssize_t pm_usbhost_write(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	pcf50633_gpio_set(pcf50633_global, PCF50633_GPO, on);

	return count;
}

static DEVICE_ATTR(hostmode, 0644, pm_usbhost_read, pm_usbhost_write);

#ifdef CONFIG_PM
static int neo1973_usbhost_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, "suspending\n");
	/* FIXME: The PMU should save the PMU status, and the GPIO code should
	 * preserve the GPIO level, so there shouldn't be anything left to do
	 * for us, should there? */

	return 0;
}

static int neo1973_usbhost_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "resuming\n");

	return 0;
}
#else
#define neo1973_usbhost_suspend	NULL
#define neo1973_usbhost_resume	NULL
#endif

static struct attribute *neo1973_usbhost_sysfs_entries[] = {
	&dev_attr_hostmode.attr,
	NULL
};

static struct attribute_group neo1973_usbhost_attr_group = {
	.name	= NULL,
	.attrs	= neo1973_usbhost_sysfs_entries,
};

static int __init neo1973_usbhost_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "starting\n");

	switch (machine_arch_type) {

#ifdef CONFIG_MACH_NEO1973_GTA01
	case MACH_TYPE_NEO1973_GTA01:
		return -EINVAL;
#endif /* CONFIG_MACH_NEO1973_GTA01 */

#ifdef CONFIG_MACH_NEO1973_GTA02
	case MACH_TYPE_NEO1973_GTA02:
/* race */
/*		pcf50633_gpio_set(pcf50633_global, PCF50633_GPO, 0); */
		break;
#endif /* CONFIG_MACH_NEO1973_GTA02 */
	}

	return sysfs_create_group(&pdev->dev.kobj, &neo1973_usbhost_attr_group);
}

static int neo1973_usbhost_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &neo1973_usbhost_attr_group);

	return 0;
}

static struct platform_driver neo1973_usbhost_driver = {
	.probe		= neo1973_usbhost_probe,
	.remove		= neo1973_usbhost_remove,
	.suspend	= neo1973_usbhost_suspend,
	.resume		= neo1973_usbhost_resume,
	.driver		= {
		.name		= "neo1973-pm-host",
	},
};

static int __devinit neo1973_usbhost_init(void)
{
	return platform_driver_register(&neo1973_usbhost_driver);
}

static void neo1973_usbhost_exit(void)
{
	platform_driver_unregister(&neo1973_usbhost_driver);
}

module_init(neo1973_usbhost_init);
module_exit(neo1973_usbhost_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Neo1973 USB Host Power Management");
