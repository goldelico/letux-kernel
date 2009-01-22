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
#include <linux/rfkill.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/plat-s3c24xx/neo1973.h>

/* For GTA01 */
#include <mach/gta01.h>
#include <linux/pcf50606.h>

/* For GTA02 */
#include <mach/gta02.h>
#include <linux/pcf50633.h>


#define DRVMSG "FIC Neo1973 Bluetooth Power Management"

static ssize_t bt_read(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int ret = 0;	

	if (!strcmp(attr->attr.name, "power_on")) {

		if (machine_is_neo1973_gta01()) {
			if (pcf50606_onoff_get(pcf50606_global,
						PCF50606_REGULATOR_D1REG) &&
			    pcf50606_voltage_get(pcf50606_global,
						 PCF50606_REGULATOR_D1REG) == 3100)
				ret = 1;
		} else if (machine_is_neo1973_gta02()) {
			if (s3c2410_gpio_getpin(GTA02_GPIO_BT_EN))
				ret = 1;
		}
	} else if (!strcmp(attr->attr.name, "reset")) {
		if (machine_is_neo1973_gta01()) {
			if (s3c2410_gpio_getpin(GTA01_GPIO_BT_EN) == 0)
				ret = 1;
		} else if (machine_is_neo1973_gta02()) {
			if (s3c2410_gpio_getpin(GTA02_GPIO_BT_EN) == 0)
				ret = 1;
		}
	}

	if (!ret) {
		return strlcpy(buf, "0\n", 3);
	} else {
		return strlcpy(buf, "1\n", 3);
	}
}

static int bt_rfkill_toggle_radio(void *data, enum rfkill_state state)
{
	struct device *dev = data;
	unsigned long on = (state == RFKILL_STATE_ON);
	unsigned int vol;

	if (machine_is_neo1973_gta01()) {
		/* if we are powering up, assert reset, then power,
		 * then release reset */
		if (on) {
			neo1973_gpb_setpin(GTA01_GPIO_BT_EN, 0);
			pcf50606_voltage_set(pcf50606_global,
					     PCF50606_REGULATOR_D1REG,
					     3100);
		}
		pcf50606_onoff_set(pcf50606_global,
				   PCF50606_REGULATOR_D1REG, on);
		neo1973_gpb_setpin(GTA01_GPIO_BT_EN, on);
	} else if (machine_is_neo1973_gta02()) {
		if (s3c2410_gpio_getpin(GTA02_GPIO_BT_EN) == on)
			return 0;
		neo1973_gpb_setpin(GTA02_GPIO_BT_EN, !on);
		pcf50633_voltage_set(pcf50633_global,
			PCF50633_REGULATOR_LDO4, on ? 3200 : 0);
		pcf50633_onoff_set(pcf50633_global,
			PCF50633_REGULATOR_LDO4, on);
		vol = pcf50633_voltage_get(pcf50633_global,
			PCF50633_REGULATOR_LDO4);
		dev_info(dev, "GTA02 Set PCF50633 LDO4 = %d\n", vol);
		neo1973_gpb_setpin(GTA02_GPIO_BT_EN, on);
	}

	return 0;
}

static ssize_t bt_write(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		struct rfkill *rfkill = dev_get_drvdata(dev);
		enum rfkill_state state = on ? RFKILL_STATE_ON : RFKILL_STATE_OFF;
		bt_rfkill_toggle_radio(dev, state);
		rfkill->state = state;
	} else if (!strcmp(attr->attr.name, "reset")) {
		/* reset is low-active, so we need to invert */
		if (machine_is_neo1973_gta01()) {
			neo1973_gpb_setpin(GTA01_GPIO_BT_EN, on ? 0 : 1);
		} else if (machine_is_neo1973_gta02()) {
			neo1973_gpb_setpin(GTA02_GPIO_BT_EN, on ? 0 : 1);
		}
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, bt_read, bt_write);
static DEVICE_ATTR(reset, 0644, bt_read, bt_write);

#ifdef CONFIG_PM
static int gta01_bt_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, DRVMSG ": suspending\n");
	/* FIXME: The PMU should save the PMU status, and the GPIO code should
	 * preserve the GPIO level, so there shouldn't be anything left to do
	 * for us, should there? */

	return 0;
}

static int gta01_bt_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, DRVMSG ": resuming\n");

	return 0;
}
#else
#define gta01_bt_suspend	NULL
#define gta01_bt_resume		NULL
#endif

static struct attribute *gta01_bt_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
	NULL
};

static struct attribute_group gta01_bt_attr_group = {
	.name	= NULL,
	.attrs	= gta01_bt_sysfs_entries,
};

static int __init gta01_bt_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;
	dev_info(&pdev->dev, DRVMSG ": starting\n");

	if (machine_is_neo1973_gta01()) {
		/* we make sure that the voltage is off */
		pcf50606_onoff_set(pcf50606_global,
				   PCF50606_REGULATOR_D1REG, 0);
		/* we pull reset to low to make sure that the chip doesn't
	 	 * drain power through the reset line */
		neo1973_gpb_setpin(GTA01_GPIO_BT_EN, 0);
	} else if (machine_is_neo1973_gta02()) {
		/* we make sure that the voltage is off */
		pcf50633_onoff_set(pcf50633_global,
				     PCF50633_REGULATOR_LDO4, 0);
		/* we pull reset to low to make sure that the chip doesn't
	 	 * drain power through the reset line */
		neo1973_gpb_setpin(GTA02_GPIO_BT_EN, 0);
	}

	rfkill = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);

	rfkill->name = pdev->name;
	rfkill->data = &pdev->dev;
	rfkill->state = RFKILL_STATE_OFF;
	rfkill->toggle_radio = bt_rfkill_toggle_radio;

	if (rfkill_register(rfkill) < 0) {
		/* We can live if it fails to register, but report it. */
		dev_dbg(&pdev->dev, DRVMSG ": RFKILL registration failed\n");
	}

	platform_set_drvdata(pdev, rfkill);

	return sysfs_create_group(&pdev->dev.kobj, &gta01_bt_attr_group);
}

static int gta01_bt_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &gta01_bt_attr_group);

	rfkill_unregister(rfkill);
	rfkill_free(rfkill);

	return 0;
}

static struct platform_driver gta01_bt_driver = {
	.probe		= gta01_bt_probe,
	.remove		= gta01_bt_remove,
	.suspend	= gta01_bt_suspend,
	.resume		= gta01_bt_resume,
	.driver		= {
		.name		= "neo1973-pm-bt",
	},
};

static int __devinit gta01_bt_init(void)
{
	return platform_driver_register(&gta01_bt_driver);
}

static void gta01_bt_exit(void)
{
	platform_driver_unregister(&gta01_bt_driver);
}

module_init(gta01_bt_init);
module_exit(gta01_bt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION(DRVMSG);
