/*
 * Battery driver for the Openmoko GTA01 device, using the pcf50606 chip.
 *
 * This is nothing more than a write-thru interface to the real logic,
 * which is part of the pcf50606.c multifunction chip driver.
 *	Copyright Â© 2008  Mike Westerhof <mwester@dls.net>
 *
 *
 * Portions liberally borrowed from olpc_battery.c, copyright below:
 *	Copyright Â© 2006  David Woodhouse <dwmw2@infradead.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>

/*********************************************************************
 * This global is set by the pcf50606 driver to the correct callback
 *********************************************************************/

extern int (*pmu_bat_get_property)(struct power_supply *,
				   enum power_supply_property,
				   union power_supply_propval *);


/*********************************************************************
 *		Battery properties
 *********************************************************************/
static int gta01_bat_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	if (pmu_bat_get_property)
		return (pmu_bat_get_property)(psy, psp, val);
	else
		return -ENODEV;
}

static enum power_supply_property gta01_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
};

/*********************************************************************
 *		Initialisation
 *********************************************************************/

static struct platform_device *bat_pdev;

static struct power_supply gta01_bat = {
	.properties = gta01_bat_props,
	.num_properties = ARRAY_SIZE(gta01_bat_props),
	.get_property = gta01_bat_get_property,
	.use_for_apm = 0,  /* pcf50606 driver has its own apm driver */
};

static int __init gta01_bat_init(void)
{
	int ret;

	bat_pdev = platform_device_register_simple("gta01-battery", 0, NULL, 0);
	if (IS_ERR(bat_pdev))
		return PTR_ERR(bat_pdev);

	gta01_bat.name = bat_pdev->name;

	ret = power_supply_register(&bat_pdev->dev, &gta01_bat);
	if (ret)
		platform_device_unregister(bat_pdev);

	return ret;
}

static void __exit gta01_bat_exit(void)
{
	power_supply_unregister(&gta01_bat);
	platform_device_unregister(bat_pdev);
}

module_init(gta01_bat_init);
module_exit(gta01_bat_exit);

MODULE_AUTHOR("Mike Westerhof <mwester@dls.net>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for GTA01");
