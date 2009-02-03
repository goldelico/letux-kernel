/*
 * Charging LED sysfs for the FIC Neo1973 GSM Phone
 * (currently only implemented in GTA02 but ready for GTA01 implementation)
 *
 * (C) 2008 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License charging_led 2 as
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

static enum neo1973_charging_led_modes charging_mode;

static char *charging_led_mode_names[] = {
	"Disabled",
	"Aux LED",
	"Power LED"
};

static ssize_t charging_led_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%03X\n", gta02_get_pcb_revision());
}

static ssize_t charging_led_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%03X\n", gta02_get_pcb_revision());
}


static DEVICE_ATTR(pcb, 0644, charging_led_read, charging_led_write);

static struct attribute *neo1973_charging_led_sysfs_entries[] = {
	&dev_attr_pcb.attr,
	NULL
};

static struct attribute_group neo1973_charging_led_attr_group = {
	.name	= NULL,
	.attrs	= neo1973_charging_led_sysfs_entries,
};

static int __init neo1973_charging_led_probe(struct platform_device *pdev)
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

	return sysfs_create_group(&pdev->dev.kobj,
					     &neo1973_charging_led_attr_group);
}

static int neo1973_charging_led_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &neo1973_charging_led_attr_group);
	return 0;
}

static struct platform_driver neo1973_charging_led_driver = {
	.probe		= neo1973_charging_led_probe,
	.remove		= neo1973_charging_led_remove,
	.driver		= {
		.name		= "neo1973-charging-led",
	},
};

static int __devinit neo1973_charging_led_init(void)
{
	return platform_driver_register(&neo1973_charging_led_driver);
}

static void neo1973_charging_led_exit(void)
{
	platform_driver_unregister(&neo1973_charging_led_driver);
}

module_init(neo1973_charging_led_init);
module_exit(neo1973_charging_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Neo1973 PCB charging_led");
#endif
