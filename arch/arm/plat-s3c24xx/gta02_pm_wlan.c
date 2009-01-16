/*
 * GTA02 WLAN power management
 *
 * (C) 2008, 2009 by Openmoko Inc.
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
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/plat-s3c24xx/neo1973.h>

#include <mach/gta02.h>
#include <mach/gta02-pm-wlan.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

#include <linux/delay.h>
#include <linux/rfkill.h>


/* ----- Module hardware reset ("power") ----------------------------------- */


static void __gta02_wlan_power(int on)
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

void gta02_wlan_power(int on)
{
	static DEFINE_MUTEX(lock);
	static int is_on = -1; /* initial state is unknown */

	on = !!on; /* normalize */
	mutex_lock(&lock);
	if (on != is_on)
		__gta02_wlan_power(on);
	is_on = on;
	mutex_unlock(&lock);
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


/* ----- rfkill ------------------------------------------------------------ */

/*
 * S3C MCI handles suspend/resume through device removal/insertion. In order to
 * preserve rfkill state, as required in clause 7 of section 3.1 in rfkill.txt,
 * we therefore need to maintain rfkill state outside the driver.
 *
 * This platform driver is as good a place as any other.
 */

static int (*gta02_wlan_rfkill_cb)(void *user, int on);
static void *gta02_wlan_rfkill_user;
static DEFINE_MUTEX(gta02_wlan_rfkill_lock);
static int gta02_wlan_rfkill_on;


/*
 * gta02_wlan_query_rfkill_lock is used to obtain the rfkill state before the
 * driver is ready to process rfkill callbacks. To prevent the state from
 * changing until the driver has completed its initialization, we grab and hold
 * the rfkill lock.
 *
 * A call to gta02_wlan_query_rfkill_lock must be followed by either
 * - a call to gta02_wlan_set_rfkill_cb, to complete the setup, or
 * - a call to gta02_wlan_query_rfkill_unlock to abort the setup process.
 */

int gta02_wlan_query_rfkill_lock(void)
{
	mutex_lock(&gta02_wlan_rfkill_lock);
	return gta02_wlan_rfkill_on;
}
EXPORT_SYMBOL_GPL(gta02_wlan_query_rfkill_lock);

void gta02_wlan_query_rfkill_unlock(void)
{
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_query_rfkill_unlock);


void gta02_wlan_set_rfkill_cb(int (*cb)(void *user, int on), void *user)
{
	BUG_ON(!mutex_is_locked(&gta02_wlan_rfkill_lock));
	BUG_ON(gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_cb = cb;
	gta02_wlan_rfkill_user = user;
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_set_rfkill_cb);

void gta02_wlan_clear_rfkill_cb(void)
{
	mutex_lock(&gta02_wlan_rfkill_lock);
	BUG_ON(!gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_cb = NULL;
	mutex_unlock(&gta02_wlan_rfkill_lock);
}
EXPORT_SYMBOL_GPL(gta02_wlan_clear_rfkill_cb);

static int gta02_wlan_toggle_radio(void *data, enum rfkill_state state)
{
	struct device *dev = data;
	int on = state == RFKILL_STATE_UNBLOCKED;
	int res = 0;

	dev_dbg(dev, "gta02_wlan_toggle_radio: state %d (%p)\n",
	    state, gta02_wlan_rfkill_cb);
	mutex_lock(&gta02_wlan_rfkill_lock);
	if (gta02_wlan_rfkill_cb)
		res = gta02_wlan_rfkill_cb(gta02_wlan_rfkill_user, on);
	if (!res)
		gta02_wlan_rfkill_on = on;
	mutex_unlock(&gta02_wlan_rfkill_lock);
	return res;
}


/* ----- Initialization/removal -------------------------------------------- */


static int __init gta02_wlan_probe(struct platform_device *pdev)
{
	/* default-on for now */
	const int default_state = 1;
	struct rfkill *rfkill;
	int error;

	if (!machine_is_neo1973_gta02())
		return -EINVAL;

	dev_info(&pdev->dev, "starting\n");

	s3c2410_gpio_cfgpin(GTA02_CHIP_PWD, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(GTA02_GPIO_nWLAN_RESET, S3C2410_GPIO_OUTPUT);
	gta02_wlan_power(1);

	rfkill = rfkill_allocate(&pdev->dev, RFKILL_TYPE_WLAN);
	rfkill->name = "ar6000";
	rfkill->data = &pdev->dev;
	rfkill->state = default_state ? RFKILL_STATE_ON : RFKILL_STATE_OFF;
	/*
	 * If the WLAN driver somehow managed to get activated before we're
	 * ready, the driver is now in an unknown state, which isn't something
	 * we're prepared to handle. This can't happen, so just fail hard.
	 */
	BUG_ON(gta02_wlan_rfkill_cb);
	gta02_wlan_rfkill_on = default_state;
	rfkill->toggle_radio = gta02_wlan_toggle_radio;

	error = rfkill_register(rfkill);
	if (error) {
		rfkill_free(rfkill);
		return error;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gta02_wlan_attr_group);
	if (error) {
		rfkill_free(rfkill);
		return error;
	}

	dev_set_drvdata(&pdev->dev, rfkill);

	return 0;
}

static int gta02_wlan_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill = dev_get_drvdata(&pdev->dev);

	rfkill_unregister(rfkill);
	rfkill_free(rfkill);

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
