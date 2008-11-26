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
#include <linux/mfd/pcf50633/gpio.h>

#include <plat/gpio-cfg.h>

enum feature {
	OM_GTA03_GPS,		/* power to GPS section and LNA */
	OM_GTA03_WLAN_BT,	/* WLAN and BT Module */
	OM_GTA03_GSM,		/* GSM module */
	OM_GTA03_USBHOST,	/* USB Host power generation */
	OM_GTA03_VIB,		/* Vibrator */

	OM_GTA03_FEATURE_COUNT	/* always last */
};


struct om_gta03_feature_info {
	const char * name;
	int depower_on_suspend;
	int on;
};

static struct om_gta03_feature_info feature_info[OM_GTA03_FEATURE_COUNT] = {
	[OM_GTA03_GPS] =	{ "gps_power",		1, 0 },
	[OM_GTA03_WLAN_BT] =	{ "wlan_bt_power",	1, 1 },
	[OM_GTA03_GSM] =	{ "gsm_power",		0, 0 },
	[OM_GTA03_USBHOST] =	{ "usbhost_power",	1, 0 },
	[OM_GTA03_VIB] =	{ "vibrator_power",	1, 0 },
};

static struct regulator *gps_regulator;



static void om_gta03_features_pwron_set_on(enum feature feature)
{
	switch (feature) {
	case OM_GTA03_GPS:
		regulator_enable(gps_regulator);
		/* enable LNA */
		gpio_direction_output(GTA03_GPIO_GPS_LNA_EN, 1);
		break;
	case OM_GTA03_WLAN_BT:
		/* give power to WLAN / BT module */
		s3c_gpio_setpull(GTA03_GPIO_WLAN_RESET, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_WLAN_RESET, S3C_GPIO_SFN(1));
		gpio_direction_output(GTA03_GPIO_WLAN_RESET, 0);

		gpio_direction_output(GTA03_GPIO_NWLAN_POWER, 0);
		s3c_gpio_setpull(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_SFN(1));
		msleep(1);
		gpio_direction_output(GTA03_GPIO_WLAN_RESET, 1);
		break;
	case OM_GTA03_GSM:
		/* give power to GSM module */
		s3c_gpio_setpull(GTA03_GPIO_N_MODEM_RESET, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_N_MODEM_RESET, S3C_GPIO_SFN(1));
		gpio_direction_output(GTA03_GPIO_N_MODEM_RESET, 0);

		gpio_direction_output(GTA03_GPIO_MODEN_ON, 1);
		s3c_gpio_setpull(GTA03_GPIO_MODEN_ON, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_MODEN_ON, S3C_GPIO_SFN(1));
		msleep(1);
		gpio_direction_output(GTA03_GPIO_N_MODEM_RESET, 1);
		break;
	case OM_GTA03_USBHOST:
		pcf50633_gpio_set(om_gta03_pcf_pdata.pcf, PCF50633_GPO, 1);
		break;
	case OM_GTA03_VIB:
		gpio_direction_output(GTA03_GPIO_VIBRATOR_ON, 1);
		break;
	default:
		break;
	}
}

static void om_gta03_features_pwron_set_off(enum feature feature)
{
	switch (feature) {
	case OM_GTA03_GPS:
		/* disable LNA */
		gpio_direction_output(GTA03_GPIO_GPS_LNA_EN, 0);
		regulator_disable(gps_regulator);
		break;
	case OM_GTA03_WLAN_BT:
		/* remove power from WLAN / BT module */
		gpio_direction_output(GTA03_GPIO_NWLAN_POWER, 1);
		s3c_gpio_setpull(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_NWLAN_POWER, S3C_GPIO_SFN(1));
		break;
	case OM_GTA03_GSM:
		/* remove power from WLAN / BT module */
		gpio_direction_output(GTA03_GPIO_MODEN_ON, 0);
		s3c_gpio_setpull(GTA03_GPIO_MODEN_ON, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GTA03_GPIO_MODEN_ON, S3C_GPIO_SFN(1));
		break;
	case OM_GTA03_USBHOST:
		pcf50633_gpio_set(om_gta03_pcf_pdata.pcf, PCF50633_GPO, 0);
		break;
	case OM_GTA03_VIB:
		gpio_direction_output(GTA03_GPIO_VIBRATOR_ON, 0);
		break;
	default:
		break;
	}
}

static void om_gta03_features_pwron_set(enum feature feature, int on)
{
	if ((on) && (!feature_info[feature].on))
		om_gta03_features_pwron_set_on(feature);
	else
		if ((!on) && (feature_info[feature].on))
			om_gta03_features_pwron_set_off(feature);
}

static ssize_t om_gta03_feature_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int on;
	int feature = 0;
	int hit = 0;

	while (!hit && feature < OM_GTA03_FEATURE_COUNT) {
		if (!strcmp(attr->attr.name, feature_info[feature].name))
			hit = 1;
		else
			feature++;
	}

	if (!hit)
		return -EINVAL;

	switch (feature) {
	case OM_GTA03_GPS:
		on = regulator_is_enabled(gps_regulator);
		break;
	case OM_GTA03_USBHOST:
		on = pcf50633_gpio_get(om_gta03_pcf_pdata.pcf, PCF50633_GPO);
		break;
	default:
		on = feature_info[feature].on;
	}

	*buf++ = '0' + on;
	*buf++='\n';
	*buf = '\0';

	return 3;
}

static ssize_t om_gta03_feature_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int on = !!simple_strtoul(buf, NULL, 10);
	int feature = 0;
	int hit = 0;

	while (!hit && feature < OM_GTA03_FEATURE_COUNT) {
		if (!strcmp(attr->attr.name, feature_info[feature].name))
			hit = 1;
		else
			feature++;
	}

	if (!hit)
		return -EINVAL;

	om_gta03_features_pwron_set(feature, on);
	feature_info[feature].on = on;

	return count;
}


static DEVICE_ATTR(gps_power, 0644, om_gta03_feature_read,
							om_gta03_feature_write);

static DEVICE_ATTR(wlan_bt_power, 0644, om_gta03_feature_read,
							om_gta03_feature_write);

static DEVICE_ATTR(gsm_power, 0644, om_gta03_feature_read,
							om_gta03_feature_write);

static DEVICE_ATTR(usbhost_power, 0644, om_gta03_feature_read,
							om_gta03_feature_write);

static DEVICE_ATTR(vibrator_power, 0644, om_gta03_feature_read,
							om_gta03_feature_write);


static struct attribute *om_gta03_features_sysfs_entries[] = {
	&dev_attr_gps_power.attr,
	&dev_attr_wlan_bt_power.attr,
	&dev_attr_gsm_power.attr,
	&dev_attr_usbhost_power.attr,
	&dev_attr_vibrator_power.attr,
	NULL
};


static struct attribute_group om_gta03_features_attr_group = {
	.name	= NULL,
	.attrs	= om_gta03_features_sysfs_entries,
};

static int __init om_gta03_features_probe(struct platform_device *pdev)
{
	gps_regulator = regulator_get(&pdev->dev, "RF_3V");
	dev_info(&pdev->dev, "starting\n");

	return sysfs_create_group(&pdev->dev.kobj,
						 &om_gta03_features_attr_group);
}

static int om_gta03_features_remove(struct platform_device *pdev)
{

	regulator_put(gps_regulator);
	sysfs_remove_group(&pdev->dev.kobj, &om_gta03_features_attr_group);

	return 0;
}


#ifdef CONFIG_PM
static int om_gta03_features_suspend(struct platform_device *pdev,
				     pm_message_t state)
{
	int feature;

	for (feature = 0; feature < OM_GTA03_FEATURE_COUNT; feature++)
		if (feature_info[feature].depower_on_suspend)
			om_gta03_features_pwron_set_off(feature);

	return 0;
}

static int om_gta03_features_resume(struct platform_device *pdev)
{
	int feature;

	for (feature = 0; feature < OM_GTA03_FEATURE_COUNT; feature++)
		if (feature_info[feature].depower_on_suspend)
			if (feature_info[feature].on)
				om_gta03_features_pwron_set_on(feature);

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
