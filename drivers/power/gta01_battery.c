/*
 * Dumb driver for gta01 battery
 *
 * Copyright 2009 Openmoko, Inc
 * Balaji Rao <balajirrao@openmoko.org>
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/gta01_battery.h>

struct gta01_battery {
	struct power_supply psy;
	struct gta01_bat_platform_data *pdata;
};

static enum power_supply_property gta01_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int gta01_bat_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct gta01_battery *bat = container_of(psy, struct gta01_battery, psy);
	
	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bat->pdata->get_charging_status())
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->pdata->get_voltage();
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bat->pdata->get_current();
		break;
	default:
		printk(KERN_ERR "Unknown property benig asked for\n");
		return -EINVAL;
	}

	return 0;
}

static void gta01_bat_ext_changed(struct power_supply *psy)
{
	struct gta01_battery *bat = container_of(psy, struct gta01_battery, psy);
	power_supply_changed(&bat->psy);
}

static int gta01_battery_probe(struct platform_device *pdev)
{
	struct gta01_battery *gta01_bat;

	gta01_bat = kzalloc(sizeof(*gta01_bat), GFP_KERNEL);
	if (!gta01_bat)
		return -ENOMEM;

	gta01_bat->psy.name = "battery";
	gta01_bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	gta01_bat->psy.properties = gta01_bat_props;
	gta01_bat->psy.num_properties = ARRAY_SIZE(gta01_bat_props);
	gta01_bat->psy.get_property = gta01_bat_get_property;
	gta01_bat->psy.external_power_changed = gta01_bat_ext_changed;

	gta01_bat->pdata = pdev->dev.platform_data;
	power_supply_register(&pdev->dev, &gta01_bat->psy);

	return 0;
}

static int gta01_battery_remove(struct platform_device *pdev)
{
	struct gta01_battery *bat = platform_get_drvdata(pdev);

	power_supply_unregister(&bat->psy);
	kfree(bat);

	return 0;
}

static struct platform_driver gta01_battery_driver = {
	.driver = {
		.name = "gta01_battery",
	},
	.probe	  = gta01_battery_probe,
	.remove   = gta01_battery_remove,
};

static int __init gta01_battery_init(void)
{
	return platform_driver_register(&gta01_battery_driver);
}

static void __exit gta01_battery_exit(void)
{
	platform_driver_unregister(&gta01_battery_driver);
}

module_init(gta01_battery_init);
module_exit(gta01_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("gta01 battery driver");
