/*
 * Driver for platform battery
 *
 * Copyright (c) Paul Fertser <fercerpav@gmail.com>
 * Inspired by Balaji Rao <balajirrao@openmoko.org>
 *
 * This driver can be used for dumb batteries when all knowledge about
 * their state belongs to the platform that does necessary ADC readings,
 * conversions, guessimations etc.
 *
 * Use consistent with the GNU GPL is permitted, provided that this
 * copyright notice is preserved in its entirety in all copies and derived
 * works.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/delay.h>
#include <linux/platform_battery.h>

struct platform_battery {
	struct power_supply psy;
	struct platform_bat_platform_data *pdata;
};

static int platform_bat_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct platform_battery *bat =
				container_of(psy, struct platform_battery, psy);
	size_t i;
	int present=1;

	if (bat->pdata->is_present)
		present = bat->pdata->is_present();

	if (psp != POWER_SUPPLY_PROP_PRESENT && !present)
		return -ENODEV;

	for (i = 0; i < psy->num_properties; i++)
		if (psy->properties[i] == psp) {
			val->intval = bat->pdata->get_property[i]();
			return 0;
		}

	return -EINVAL;
}

static void platform_bat_ext_changed(struct power_supply *psy)
{
	struct platform_battery *bat =
				container_of(psy, struct platform_battery, psy);
	power_supply_changed(&bat->psy);
}

static int platform_battery_probe(struct platform_device *pdev)
{
	struct platform_battery *platform_bat;
	struct platform_bat_platform_data *pdata =
		(struct platform_bat_platform_data *)pdev->dev.platform_data;

	platform_bat = kzalloc(sizeof(*platform_bat), GFP_KERNEL);
	if (!platform_bat)
		return -ENOMEM;

	if (pdata->name)
		platform_bat->psy.name = pdata->name;
	else
		platform_bat->psy.name = dev_name(&pdev->dev);
	platform_bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	platform_bat->psy.properties = pdata->properties;
	platform_bat->psy.num_properties = pdata->num_properties;
	platform_bat->psy.get_property = platform_bat_get_property;
	platform_bat->psy.external_power_changed = platform_bat_ext_changed;

	platform_bat->pdata = pdata;
	platform_set_drvdata(pdev, platform_bat);
	power_supply_register(&pdev->dev, &platform_bat->psy);

	return 0;
}

static int platform_battery_remove(struct platform_device *pdev)
{
	struct platform_battery *bat = platform_get_drvdata(pdev);

	power_supply_unregister(&bat->psy);
	kfree(bat);

	return 0;
}

static struct platform_driver platform_battery_driver = {
	.driver = {
		.name = "platform_battery",
	},
	.probe	  = platform_battery_probe,
	.remove   = platform_battery_remove,
};

static int __init platform_battery_init(void)
{
	return platform_driver_register(&platform_battery_driver);
}
module_init(platform_battery_init);

static void __exit platform_battery_exit(void)
{
	platform_driver_unregister(&platform_battery_driver);
}
module_exit(platform_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Fertser <fercerpav@gmail.com>");
MODULE_DESCRIPTION("platform battery driver");
