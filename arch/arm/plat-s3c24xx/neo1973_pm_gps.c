/*
 * GPS Power Management code for the FIC Neo1973 GSM Phone
 *
 * (C) 2007-2009 by Openmoko Inc.
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
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>

#include <asm/plat-s3c24xx/neo1973.h>

/* For GTA01 */
#include <mach/gta01.h>
#include <linux/mfd/pcf50606/core.h>
#include <linux/mfd/pcf50606/gpo.h>

/* For GTA02 */
#include <mach/gta02.h>
#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/pmic.h>

#include <linux/regulator/consumer.h>
#include <linux/err.h>

enum gta01_pm_gps_supplies {

	/* GTA01 */
	GTA01_GPS_REG_2V8,
	GTA01_GPS_REG_3V,
	GTA01_GPS_REG_3V3,
	GTA01_GPS_REG_1V5,
	GTA01_GPS_REG_2V5,

	/* GTA02 */
	GTA02_GPS_REG_RF_3V,

	/* Always last */
	GTA01_GPS_NUM_REG
};

#include <linux/err.h>

struct neo1973_pm_gps_data {
#ifdef CONFIG_PM
	int keep_on_in_suspend;
#endif
	int power_was_on; /* For GTA02 only */
	int regulator_state[GTA01_GPS_NUM_REG];
	struct regulator *regulator[GTA01_GPS_NUM_REG];
};

static struct neo1973_pm_gps_data neo1973_gps;

int neo1973_pm_gps_is_on(void)
{
	return neo1973_gps.power_was_on;
}
EXPORT_SYMBOL_GPL(neo1973_pm_gps_is_on);

#ifdef CONFIG_MACH_NEO1973_GTA01

/* This is the 2.8V supply for the RTC crystal, the mail clock crystal and
 * the input to VDD_RF */
static void gps_power_2v8_set(int on)
{
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_2V8];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
		if (on)
			regulator_enable(regulator);
		else
			regulator_disable(regulator);
		neo1973_gps.regulator_state[GTA01_GPS_REG_2V8] = on;
		break;
	case GTA01Bv2_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA01_GPIO_GPS_EN_2V8, on);
			break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		break;
	}
}

static int gps_power_2v8_get(void)
{
	int ret = 0;
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_2V8];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
			ret = regulator_is_enabled(regulator);
		break;
	case GTA01Bv2_SYSTEM_REV:
		if (s3c2410_gpio_getpin(GTA01_GPIO_GPS_EN_2V8))
			ret = 1;
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		break;
	}

	return ret;
}

/* This is the 3V supply (AVDD) for the external RF frontend (LNA bias) */
static void gps_power_3v_set(int on)
{
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_3V];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
		if (on)
			regulator_enable(regulator);
		else
			regulator_disable(regulator);
		neo1973_gps.regulator_state[GTA01_GPS_REG_3V3] = on;
		break;
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA01_GPIO_GPS_EN_3V, on);
		break;
	}
}

static int gps_power_3v_get(void)
{
	int ret = 0;
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_3V];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
		ret = regulator_is_enabled(regulator);
		break;
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		if (s3c2410_gpio_getpin(GTA01_GPIO_GPS_EN_3V))
			ret = 1;
		break;
	}

	return ret;
}

/* This is the 3.3V supply for VDD_IO and VDD_LPREG input */
static void gps_power_3v3_set(int on)
{
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_3V3];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		if (on)
			regulator_enable(regulator);
		else
			regulator_disable(regulator);
		neo1973_gps.regulator_state[GTA01_GPS_REG_3V3] = on;
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA01_GPIO_GPS_EN_3V3, on);
		break;
	}
}

static int gps_power_3v3_get(void)
{
	int ret = 0;
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_3V3];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		ret = regulator_is_enabled(regulator);
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		if (s3c2410_gpio_getpin(GTA01_GPIO_GPS_EN_3V3))
			ret = 1;
		break;
	}

	return ret;
}

/* This is the 2.5V supply for VDD_PLLREG and VDD_COREREG input */
static void gps_power_2v5_set(int on)
{
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_2V5];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		/* This is CORE_1V8 and cannot be disabled */
		break;
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		if (on)
			regulator_enable(regulator);
		else
			regulator_disable(regulator);
		neo1973_gps.regulator_state[GTA01_GPS_REG_2V5] = on;
		break;
	}
}

static int gps_power_2v5_get(void)
{
	int ret = 0;
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_2V5];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		/* This is CORE_1V8 and cannot be disabled */
		ret = 1;
		break;
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		ret = regulator_is_enabled(regulator);
		break;
	}

	return ret;
}

/* This is the 1.5V supply for VDD_CORE */
static void gps_power_1v5_set(int on)
{
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_1V5];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		/* This is switched via 2v5 */
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		if (on)
			regulator_enable(regulator);
		else
			regulator_disable(regulator);
		neo1973_gps.regulator_state[GTA01_GPS_REG_1V5] = on;
		break;
	}
}

static int gps_power_1v5_get(void)
{
	int ret = 0;
	struct regulator *regulator = neo1973_gps.regulator[GTA01_GPS_REG_1V5];

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		/* This is switched via 2v5 */
		ret = 1;
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		ret = regulator_is_enabled(regulator);
		break;
	}

	return ret;
}
#endif

/* This is the POWERON pin */
static void gps_pwron_set(int on)
{

	if (machine_is_neo1973_gta01())
		neo1973_gpb_setpin(GTA01_GPIO_GPS_PWRON, on);

	if (machine_is_neo1973_gta02()) {
		if (on) {
			/* return UART pins to being UART pins */
			s3c2410_gpio_cfgpin(S3C2410_GPH4, S3C2410_GPH4_TXD1);
			/* remove pulldown now it won't be floating any more */
			s3c2410_gpio_pullup(S3C2410_GPH5, 0);
		} else {
			/*
			 * take care not to power unpowered GPS from UART TX
			 * return them to GPIO and force low
			 */
			s3c2410_gpio_cfgpin(S3C2410_GPH4, S3C2410_GPH4_OUTP);
			s3c2410_gpio_setpin(S3C2410_GPH4, 0);
			/* don't let RX from unpowered GPS float */
			s3c2410_gpio_pullup(S3C2410_GPH5, 1);
		}
		if (on && !neo1973_gps.power_was_on)
			regulator_enable(neo1973_gps.regulator[GTA02_GPS_REG_RF_3V]);
		else if (!on && neo1973_gps.power_was_on)
			regulator_disable(neo1973_gps.regulator[GTA02_GPS_REG_RF_3V]);
	}
}

static int gps_pwron_get(void)
{
	if (machine_is_neo1973_gta01())
		return !!s3c2410_gpio_getpin(GTA01_GPIO_GPS_PWRON);

	if (machine_is_neo1973_gta02())
		return regulator_is_enabled(neo1973_gps.regulator[GTA02_GPS_REG_RF_3V]);
	return -1;
}


#ifdef CONFIG_MACH_NEO1973_GTA01
static void gps_rst_set(int on);
static int gps_rst_get(void);
#endif

#ifdef CONFIG_PM
/* This is the flag for keeping gps ON during suspend */
static void gps_keep_on_in_suspend_set(int on)
{
	neo1973_gps.keep_on_in_suspend = on;
}

static int gps_keep_on_in_suspend_get(void)
{
	return neo1973_gps.keep_on_in_suspend;
}
#endif

static ssize_t power_gps_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!strcmp(attr->attr.name, "power_on")) {
		ret = gps_pwron_get();
#ifdef CONFIG_PM
	} else if (!strcmp(attr->attr.name, "keep_on_in_suspend")) {
		ret = gps_keep_on_in_suspend_get();
#endif
#ifdef CONFIG_MACH_NEO1973_GTA01
	} else if (!strcmp(attr->attr.name, "power_avdd_3v")) {
		ret = gps_power_3v_get();
	} else if (!strcmp(attr->attr.name, "power_tcxo_2v8")) {
		ret = gps_power_2v8_get();
	} else if (!strcmp(attr->attr.name, "reset")) {
		ret = gps_rst_get();
	} else if (!strcmp(attr->attr.name, "power_lp_io_3v3")) {
		ret = gps_power_3v3_get();
	} else if (!strcmp(attr->attr.name, "power_pll_core_2v5")) {
		ret = gps_power_2v5_get();
	} else if (!strcmp(attr->attr.name, "power_core_1v5") ||
		   !strcmp(attr->attr.name, "power_vdd_core_1v5")) {
		ret = gps_power_1v5_get();
#endif
	}
	if (ret)
		return strlcpy(buf, "1\n", 3);
	else
		return strlcpy(buf, "0\n", 3);
}

static ssize_t power_gps_write(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		gps_pwron_set(on);
		neo1973_gps.power_was_on = !!on;
#if CONFIG_PM
	} else if (!strcmp(attr->attr.name, "keep_on_in_suspend")) {
		gps_keep_on_in_suspend_set(on);
#endif
#ifdef CONFIG_MACH_NEO1973_GTA01
	} else if (!strcmp(attr->attr.name, "power_avdd_3v")) {
		gps_power_3v_set(on);
	} else if (!strcmp(attr->attr.name, "power_tcxo_2v8")) {
		gps_power_2v8_set(on);
	} else if (!strcmp(attr->attr.name, "reset")) {
		gps_rst_set(on);
	} else if (!strcmp(attr->attr.name, "power_lp_io_3v3")) {
		gps_power_3v3_set(on);
	} else if (!strcmp(attr->attr.name, "power_pll_core_2v5")) {
		gps_power_2v5_set(on);
	} else if (!strcmp(attr->attr.name, "power_core_1v5") ||
		   !strcmp(attr->attr.name, "power_vdd_core_1v5")) {
		gps_power_1v5_set(on);
#endif
	}
	return count;
}


#ifdef CONFIG_MACH_NEO1973_GTA01

/* This is the nRESET pin */
static void gps_rst_set(int on)
{
	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		pcf50606_gpo_set_active(gta01_pcf, PCF50606_GPO1, on);
		break;
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA01_GPIO_GPS_RESET, on);
		break;
	}
}

static int gps_rst_get(void)
{
	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		return pcf50606_gpo_get_active(gta01_pcf, PCF50606_GPO1);
		break;
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		if (s3c2410_gpio_getpin(GTA01_GPIO_GPS_RESET))
			return 1;
		break;
	}

	return 0;
}


static void gps_power_sequence_up(void)
{
	/* According to PMB2520 Data Sheet, Rev. 2006-06-05,
	 * Chapter 4.2.2 */

	/* nRESET must be asserted low */
	gps_rst_set(0);

	/* POWERON must be de-asserted (low) */
	gps_pwron_set(0);

	/* Apply VDD_IO and VDD_LPREG_IN */
	gps_power_3v3_set(1);

	/* VDD_COREREG_IN, VDD_PLLREG_IN */
	gps_power_1v5_set(1);
	gps_power_2v5_set(1);

	/* and VDD_RF may be applied */
	gps_power_2v8_set(1);

	/* We need to enable AVDD, since in GTA01Bv3 it is
	 * shared with RFREG_IN */
	gps_power_3v_set(1);

	msleep(3); 	/* Is 3ms enough? */

	/* De-asert nRESET */
	gps_rst_set(1);

	/* Switch power on */
	gps_pwron_set(1);

}

static void gps_power_sequence_down(void)
{
	/* According to PMB2520 Data Sheet, Rev. 2006-06-05,
	 * Chapter 4.2.3.1 */
	gps_pwron_set(0);

	/* Don't disable AVDD before PWRON is cleared, since
	 * in GTA01Bv3, AVDD and RFREG_IN are shared */
	if (neo1973_gps.regulator_state[GTA01_GPS_REG_3V])
		gps_power_3v_set(0);

	/* Remove VDD_COREREG_IN, VDD_PLLREG_IN and VDD_REFREG_IN */
	if (neo1973_gps.regulator_state[GTA01_GPS_REG_1V5])
		gps_power_1v5_set(0);
	if (neo1973_gps.regulator_state[GTA01_GPS_REG_2V5])
		gps_power_2v5_set(0);
	if (neo1973_gps.regulator_state[GTA01_GPS_REG_2V8])
		gps_power_2v8_set(0);

	/* Remove VDD_LPREG_IN and VDD_IO */
	if (neo1973_gps.regulator_state[GTA01_GPS_REG_3V3])
		gps_power_3v3_set(0);

}

static ssize_t power_sequence_read(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return strlcpy(buf, "power_up power_down\n", PAGE_SIZE);
}

static ssize_t power_sequence_write(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	dev_dbg(dev, "wrote: '%s'\n", buf);

	if (!strncmp(buf, "power_up", 8))
		gps_power_sequence_up();
	else if (!strncmp(buf, "power_down", 10))
		gps_power_sequence_down();
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(power_tcxo_2v8, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_avdd_3v, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(reset, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_lp_io_3v3, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_pll_core_2v5, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_core_1v5, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_vdd_core_1v5, 0644, power_gps_read, power_gps_write);
static DEVICE_ATTR(power_sequence, 0644, power_sequence_read,
		   power_sequence_write);
#endif

#ifdef CONFIG_PM
static int gta01_pm_gps_suspend(struct platform_device *pdev,
				pm_message_t state)
{
#ifdef CONFIG_MACH_NEO1973_GTA01
	if (machine_is_neo1973_gta01())
		/* FIXME */
		gps_power_sequence_down();
#endif
	if (machine_is_neo1973_gta02()) {
		if (!neo1973_gps.keep_on_in_suspend ||
		    !neo1973_gps.power_was_on)
			gps_pwron_set(0);
		else
			dev_warn(&pdev->dev, "GTA02: keeping gps ON "
				 "during suspend\n");
	}

	return 0;
}

static int gta01_pm_gps_resume(struct platform_device *pdev)
{
#ifdef CONFIG_MACH_NEO1973_GTA01
	if (machine_is_neo1973_gta01())
		if (neo1973_gps.power_was_on)
			gps_power_sequence_up();
#endif
	if (machine_is_neo1973_gta02())
		if (!neo1973_gps.keep_on_in_suspend && neo1973_gps.power_was_on)
		    gps_pwron_set(1);

	return 0;
}

static DEVICE_ATTR(keep_on_in_suspend, 0644, power_gps_read, power_gps_write);
#else
#define gta01_pm_gps_suspend	NULL
#define gta01_pm_gps_resume	NULL
#endif

static DEVICE_ATTR(power_on, 0644, power_gps_read, power_gps_write);


static struct attribute *gta01_gps_sysfs_entries[] = {
	&dev_attr_power_on.attr,
#ifdef CONFIG_MACH_NEO1973_GTA01
	&dev_attr_power_avdd_3v.attr,
	&dev_attr_reset.attr,
	&dev_attr_power_lp_io_3v3.attr,
	&dev_attr_power_pll_core_2v5.attr,
	&dev_attr_power_sequence.attr,
	NULL,	/* power_core_1v5 */
	NULL,	/* power_vdd_core_1v5 */
#endif
	NULL    /* terminating entry */
};

static struct attribute_group gta01_gps_attr_group = {
	.name	= NULL,
	.attrs	= gta01_gps_sysfs_entries,
};

static struct attribute *gta02_gps_sysfs_entries[] = {
	&dev_attr_power_on.attr,
#ifdef CONFIG_PM
	&dev_attr_keep_on_in_suspend.attr,
#endif
	NULL
};

static struct attribute_group gta02_gps_attr_group = {
	.name	= NULL,
	.attrs	= gta02_gps_sysfs_entries,
};

static int __init gta01_pm_gps_probe(struct platform_device *pdev)
{
	if (machine_is_neo1973_gta01()) {
		s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_PWRON, S3C2410_GPIO_OUTPUT);

		switch (system_rev) {
		case GTA01v3_SYSTEM_REV:
			break;
		case GTA01v4_SYSTEM_REV:
			s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_RESET,
			    S3C2410_GPIO_OUTPUT);
			break;
		case GTA01Bv3_SYSTEM_REV:
		case GTA01Bv4_SYSTEM_REV:
			s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_EN_3V3,
			    S3C2410_GPIO_OUTPUT);
			/* fallthrough */
		case GTA01Bv2_SYSTEM_REV:
			s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_EN_2V8,
			    S3C2410_GPIO_OUTPUT);
			s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_EN_3V,
			    S3C2410_GPIO_OUTPUT);
			s3c2410_gpio_cfgpin(GTA01_GPIO_GPS_RESET,
			    S3C2410_GPIO_OUTPUT);
			break;
		default:
			dev_warn(&pdev->dev, "Unknown GTA01 Revision 0x%x, "
				"AGPS PM features not available!!!\n",
				system_rev);
			return -1;
			break;
		}

#ifdef CONFIG_MACH_NEO1973_GTA01
	
		neo1973_gps.regulator[GTA01_GPS_REG_2V8] =
		       			regulator_get(&pdev->dev, "GPS_2V8");
		neo1973_gps.regulator[GTA01_GPS_REG_3V] =
		       			regulator_get(&pdev->dev, "GPS_3V");
		neo1973_gps.regulator[GTA01_GPS_REG_3V3] =
		       			regulator_get(&pdev->dev, "GPS_3V3");
		neo1973_gps.regulator[GTA01_GPS_REG_1V5] =
		       			regulator_get(&pdev->dev, "GPS_1V5");
		neo1973_gps.regulator[GTA01_GPS_REG_2V5] =
		       			regulator_get(&pdev->dev, "GPS_2V5");

		gps_power_sequence_down();

		switch (system_rev) {
			int entries = ARRAY_SIZE(gta01_gps_sysfs_entries);
		case GTA01v3_SYSTEM_REV:
		case GTA01v4_SYSTEM_REV:
		case GTA01Bv2_SYSTEM_REV:
			gta01_gps_sysfs_entries[entries-3] =
			    &dev_attr_power_tcxo_2v8.attr;
			break;
		case GTA01Bv3_SYSTEM_REV:
		case GTA01Bv4_SYSTEM_REV:
			gta01_gps_sysfs_entries[entries-3] =
			    &dev_attr_power_core_1v5.attr;
			gta01_gps_sysfs_entries[entries-2] =
			    &dev_attr_power_vdd_core_1v5.attr;
			break;
		}
#endif
		return sysfs_create_group(&pdev->dev.kobj,
		    &gta01_gps_attr_group);
	}

	if (machine_is_neo1973_gta02()) {
		switch (system_rev) {
		case GTA02v2_SYSTEM_REV:
		case GTA02v3_SYSTEM_REV:
		case GTA02v4_SYSTEM_REV:
		case GTA02v5_SYSTEM_REV:
		case GTA02v6_SYSTEM_REV:
			neo1973_gps.regulator[GTA02_GPS_REG_RF_3V] = regulator_get(
							&pdev->dev, "RF_3V");
			if (IS_ERR(neo1973_gps.regulator)) {
				dev_err(&pdev->dev, "probe failed %ld\n",
				    PTR_ERR(neo1973_gps.regulator));
				return PTR_ERR(neo1973_gps.regulator);
			}

			dev_info(&pdev->dev, "FIC Neo1973 GPS Power Managerment:"
				 "starting\n");
			break;
		default:
			dev_warn(&pdev->dev, "Unknown GTA02 Revision 0x%x, "
				"AGPS PM features not available!!!\n",
				system_rev);
			return -1;
		}

		/*
		 * u-boot enables LDO5 (GPS), which doesn't make sense and
		 * causes confusion. We therefore disable the regulator here.
		 *
		 * We don't do this through the regulator API because we'd have
		 * to second-guess some of its internal logic and make it do
		 * something that isn't really part of its design.
		 */
		pcf50633_reg_write(gta02_pcf,
		    PCF50633_REG_LDO5ENA, 0);

		return sysfs_create_group(&pdev->dev.kobj,
		    &gta02_gps_attr_group);
	}
	return -1;
}

static int gta01_pm_gps_remove(struct platform_device *pdev)
{
	if (machine_is_neo1973_gta01()) {
#ifdef CONFIG_MACH_NEO1973_GTA01
		int i;

		gps_power_sequence_down();
		/* Now disable all regulators */
		for (i = 0; i < GTA01_GPS_NUM_REG; i++) {
			regulator_put(neo1973_gps.regulator[i]);
		}
#endif
		sysfs_remove_group(&pdev->dev.kobj, &gta01_gps_attr_group);
	}

	if (machine_is_neo1973_gta02()) {
		regulator_put(neo1973_gps.regulator[GTA02_GPS_REG_RF_3V]);
		sysfs_remove_group(&pdev->dev.kobj, &gta02_gps_attr_group);
	}
	return 0;
}

static struct platform_driver gta01_pm_gps_driver = {
	.probe		= gta01_pm_gps_probe,
	.remove		= gta01_pm_gps_remove,
	.suspend	= gta01_pm_gps_suspend,
	.resume		= gta01_pm_gps_resume,
	.driver		= {
		.name		= "neo1973-pm-gps",
	},
};

static int __devinit gta01_pm_gps_init(void)
{
	return platform_driver_register(&gta01_pm_gps_driver);
}

static void gta01_pm_gps_exit(void)
{
	platform_driver_unregister(&gta01_pm_gps_driver);
}

module_init(gta01_pm_gps_init);
module_exit(gta01_pm_gps_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("FIC Neo1973 GPS Power Management");
