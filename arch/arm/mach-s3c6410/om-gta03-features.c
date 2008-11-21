/*
 * Support for features of Openmoko GTA03
 *
 * (C) 2008 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
 * All rights reserved.
 *
 * Somewhat based on the GTA01 / 02 neo1973_pm_ stuff mainly by Harald Welte
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/om-gta03.h>
#include <asm/mach-types.h>

#include <linux/regulator/consumer.h>
#include <linux/mfd/pcf50633/core.h>

#include <plat/gpio-cfg.h>

struct om_gta03_features_data {
	int gps_power_was_on;
	struct regulator *gps_regulator;

	int wlan_bt_power_was_on;

	int gsm_power_was_on;

	int usbhost_power_was_on;

};

static struct om_gta03_features_data om_gta03_features;

/* GPS power */

static void om_gta03_gps_pwron_set(int on)
{
	if ((on) && (!om_gta03_features.gps_power_was_on)) {
		regulator_enable(om_gta03_features.gps_regulator);
		/* enable LNA */
		gpio_direction_output(GTA03_GPIO_GPS_LNA_EN, 1);
	}

	if ((!on) && (om_gta03_features.gps_power_was_on)) {
		/* disable LNA */
		gpio_direction_output(GTA03_GPIO_GPS_LNA_EN, 0);
		regulator_disable(om_gta03_features.gps_regulator);
	}
}

static ssize_t om_gta03_gps_power_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (regulator_is_enabled(om_gta03_features.gps_regulator))
		return strlcpy(buf, "1\n", 3);
	else
		return strlcpy(buf, "0\n", 3);
}

static ssize_t om_gta03_gps_power_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int on = !!simple_strtoul(buf, NULL, 10);

	om_gta03_gps_pwron_set(on);
	om_gta03_features.gps_power_was_on = on;

	return count;
}

static DEVICE_ATTR(gps_power, 0644, om_gta03_gps_power_read,
						      om_gta03_gps_power_write);

/* WLAN / BT power */

static void om_gta03_wlan_bt_pwron_set(int on)
{
	if (!on) {
		/* remove power from WLAN / BT module */
		gpio_direction_output(GTA03_GPIO_NWLAN_POWER, 1);
		s3c_gpio_setpull(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_SFN(1));

		return;
	}

	/* give power to WLAN / BT module */
	s3c_gpio_setpull(GTA03_GPIO_WLAN_RESET, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GTA03_GPIO_WLAN_RESET, S3C_GPIO_SFN(1));
	gpio_direction_output(GTA03_GPIO_WLAN_RESET, 0);

	gpio_direction_output(GTA03_GPIO_NWLAN_POWER, 0);
	s3c_gpio_setpull(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_SFN(1));
	msleep(1);
	gpio_direction_output(GTA03_GPIO_WLAN_RESET, 1);
}

static ssize_t om_gta03_wlan_bt_power_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (om_gta03_features.wlan_bt_power_was_on)
		return strlcpy(buf, "1\n", 3);
	else
		return strlcpy(buf, "0\n", 3);
}

static ssize_t om_gta03_wlan_bt_power_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	om_gta03_wlan_bt_pwron_set(simple_strtoul(buf, NULL, 10));

	return count;
}

static DEVICE_ATTR(wlan_bt_power, 0644, om_gta03_wlan_bt_power_read,
						  om_gta03_wlan_bt_power_write);


/* GSM Module on / off */

static void om_gta03_gsm_pwron_set(int on)
{
	if (!on) {
		/* remove power from WLAN / BT module */
		gpio_direction_output(GTA03_GPIO_MODEN_ON, 0);
		s3c_gpio_setpull(GTA03_GPIO_MODEN_ON, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_MODEN_ON, S3C_GPIO_SFN(1));

		return;
	}

	/* give power to GSM module */
	s3c_gpio_setpull(GTA03_GPIO_N_MODEM_RESET, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GTA03_GPIO_N_MODEM_RESET, S3C_GPIO_SFN(1));
	gpio_direction_output(GTA03_GPIO_N_MODEM_RESET, 0);

	gpio_direction_output(GTA03_GPIO_MODEN_ON, 1);
	s3c_gpio_setpull(GTA03_GPIO_MODEN_ON, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GTA03_GPIO_MODEN_ON, S3C_GPIO_SFN(1));
	msleep(1);
	gpio_direction_output(GTA03_GPIO_N_MODEM_RESET, 1);
}

static ssize_t om_gta03_gsm_power_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (om_gta03_features.gsm_power_was_on)
		return strlcpy(buf, "1\n", 3);
	else
		return strlcpy(buf, "0\n", 3);
}

static ssize_t om_gta03_gsm_power_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int on = !!simple_strtoul(buf, NULL, 10);

	om_gta03_gsm_pwron_set(on);
	om_gta03_features.gsm_power_was_on = on;

	return count;
}

static DEVICE_ATTR(gsm_power, 0644, om_gta03_gsm_power_read,
						  om_gta03_gsm_power_write);


/* USB host power on / off */

static void om_gta03_usbhost_pwron_set(int on)
{
	pcf50633_gpio_set(om_gta03_pcf_pdata.pcf, PCF50633_GPO, on);
}

static ssize_t om_gta03_usbhost_power_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (pcf50633_gpio_get(om_gta03_pcf_pdata.pcf, PCF50633_GPO))
		return strlcpy(buf, "1\n", 3);
	else
		return strlcpy(buf, "0\n", 3);
}

static ssize_t om_gta03_usbhost_power_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int on = !!simple_strtoul(buf, NULL, 10);

	om_gta03_usbhost_pwron_set(on);
	om_gta03_features.usbhost_power_was_on = on;

	return count;
}

static DEVICE_ATTR(usbhost_power, 0644, om_gta03_usbhost_power_read,
						  om_gta03_usbhost_power_write);




static struct attribute *om_gta03_features_sysfs_entries[] = {
	&dev_attr_gps_power.attr,
	&dev_attr_wlan_bt_power.attr,
	&dev_attr_gsm_power.attr,
	&dev_attr_usbhost_power.attr,
	NULL
};

static struct attribute_group om_gta03_features_attr_group = {
	.name	= NULL,
	.attrs	= om_gta03_features_sysfs_entries,
};

static int __init om_gta03_features_probe(struct platform_device *pdev)
{
	om_gta03_features.gps_regulator = regulator_get(&pdev->dev, "RF_3V");
	dev_info(&pdev->dev, "starting\n");

	om_gta03_features.wlan_bt_power_was_on = 1; /* defaults to on */

	return sysfs_create_group(&pdev->dev.kobj, &om_gta03_features_attr_group);
}

static int om_gta03_features_remove(struct platform_device *pdev)
{

	regulator_put(om_gta03_features.gps_regulator);
	sysfs_remove_group(&pdev->dev.kobj, &om_gta03_features_attr_group);

	return 0;
}


#ifdef CONFIG_PM
static int om_gta03_features_suspend(struct platform_device *pdev,
				     pm_message_t state)
{
	om_gta03_gps_pwron_set(0);
	om_gta03_wlan_bt_pwron_set(0);
	om_gta03_usbhost_pwron_set(0);

	return 0;
}

static int om_gta03_features_resume(struct platform_device *pdev)
{
	if (om_gta03_features.gps_power_was_on)
		om_gta03_gps_pwron_set(1);

	if (om_gta03_features.wlan_bt_power_was_on)
		om_gta03_wlan_bt_pwron_set(1);

	if (om_gta03_features.usbhost_power_was_on)
		om_gta03_usbhost_pwron_set(1);

	return 0;
}
#else
#define om_gta03_features_suspend	NULL
#define om_gta03_features_resume	NULL
#endif

static struct platform_driver om_gta03_features_driver = {
	.probe		= om_gta03_features_probe,
	.remove		= om_gta03_features_remove,
	.suspend	= om_gta03_features_suspend,
	.resume		= om_gta03_features_resume,
	.driver		= {
		.name		= "om-gta03",
	},
};

static int __devinit om_gta03_features_init(void)
{
	return platform_driver_register(&om_gta03_features_driver);
}

static void om_gta03_features_exit(void)
{
	platform_driver_unregister(&om_gta03_features_driver);
}

module_init(om_gta03_features_init);
module_exit(om_gta03_features_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Openmoko GTA03 Feature Driver");
