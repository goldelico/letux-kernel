/*
 * Dumb driver for LiIon batteries using TWL4030 madc.
 *
 * Copyright 2013 Golden Delicious Computers
 * Lukas Märdian <lukas@goldelico.com>
 *
 * Based on dumb driver for gta01 battery
 * Copyright 2009 Openmoko, Inc
 * Balaji Rao <balajirrao@openmoko.org>
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/power/twl4030_madc_battery.h>

struct twl4030_madc_battery {
	struct power_supply psy;
	struct twl4030_madc_bat_platform_data *pdata;
};

static enum power_supply_property twl4030_madc_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

/*  MADC channels
0       Battery type(BTYPE)
1       BCI: Battery temperature (BTEMP)
2       GP analog input
3       GP analog input
4       GP analog input
5       GP analog input
6       GP analog input
7       GP analog input
8       BCI: VBUS voltage(VBUS)
9       Backup Battery voltage (VBKP)
10      BCI: Battery charger current (ICHG)
11      BCI: Battery charger voltage (VCHG)
12      BCI: Main battery voltage (VBAT)
13      Reserved
14      Reserved
15      VRUSB Supply/Speaker left/Speaker right polarization level
*/
enum channel { BTYPE=0, BTEMP=1, VBUS=8, VBKP=9, ICHG=10, VCHG=11, VBAT=12, VRUSB=15 };

static int madc_read(int index)
{
    struct twl4030_madc_request req;
    int val;

    req.channels = (1 << index);
    req.method = TWL4030_MADC_SW2;
    req.func_cb = NULL;
    val = twl4030_madc_conversion(&req);
    if (val < 0)
        return val;

    return req.rbuf[index];
}

static int twl4030_madc_bat_get_charging_status(void)
{
    int curr;
    curr = madc_read(ICHG);
    if (curr > 0)
        return 1;
    else
        return 0;
}

static int twl4030_madc_bat_get_voltage(void)
{
    return madc_read(VBAT) * 1000;
}

static int twl4030_madc_bat_get_current(void)
{
    return madc_read(ICHG) * 1000;
}

static int twl4030_madc_bat_get_temp(void)
{
    return madc_read(BTEMP) * 10;
}

static int twl4030_madc_bat_voltscale(struct twl4030_madc_battery *bat, int volt)
{
	struct twl4030_madc_bat_calibration *calibration;
	int i, res = 0;

    /* choose charging curve */
    if (twl4030_madc_bat_get_charging_status())
		calibration = bat->pdata->charging;
	else
		calibration = bat->pdata->discharging;

	/* FIXME: this assumes that the first entry is the largest one */
	if (volt > calibration[0].voltage)
		res = calibration[0].level;
	else
		for (i = 0; calibration[i+1].voltage >= 0; i++) {
			if (volt <= calibration[i].voltage && volt >= calibration[i+1].voltage) {
				/* interval found - interpolate within range */
				res = calibration[i].level - ((calibration[i].voltage - volt)*
					(calibration[i].level - calibration[i+1].level))/
					(calibration[i].voltage - calibration[i+1].voltage);
				break;
			}
		}
	return res;
}

static int twl4030_madc_bat_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct twl4030_madc_battery *bat = container_of(psy, struct twl4030_madc_battery, psy);
	
	switch(psp) {

	case POWER_SUPPLY_PROP_STATUS:
        if (twl4030_madc_bat_voltscale(bat, twl4030_madc_bat_get_voltage()/1000) > 95)
            val->intval = POWER_SUPPLY_STATUS_FULL;
        else
        {
		    if (twl4030_madc_bat_get_charging_status())
		    	val->intval = POWER_SUPPLY_STATUS_CHARGING;
		    else
		    	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl4030_madc_bat_get_voltage();
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = twl4030_madc_bat_get_current();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
			/* val->intval = twl4030_madc_bat_get_temp() < 560; /* madc appears to report 56C for a missing battery (i.e. no thermistor connected) */
			/* but only during charging! */
			val->intval = 1;	/* assume battery is always present */
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW: {
			int percent = twl4030_madc_bat_voltscale(bat, twl4030_madc_bat_get_voltage()/1000);
			val->intval = (percent * bat->pdata->capacity) / 100;
			break;
		}
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = twl4030_madc_bat_voltscale(bat, twl4030_madc_bat_get_voltage()/1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bat->pdata->capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = twl4030_madc_bat_get_temp();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void twl4030_madc_bat_ext_changed(struct power_supply *psy)
{
	struct twl4030_madc_battery *bat = container_of(psy, struct twl4030_madc_battery, psy);
	power_supply_changed(&bat->psy);
}

static int twl4030_madc_battery_probe(struct platform_device *pdev)
{
	struct twl4030_madc_battery *twl4030_madc_bat;

	twl4030_madc_bat = kzalloc(sizeof(*twl4030_madc_bat), GFP_KERNEL);
	if (!twl4030_madc_bat)
		return -ENOMEM;

	twl4030_madc_bat->psy.name = "twl4030_battery";
	twl4030_madc_bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	twl4030_madc_bat->psy.properties = twl4030_madc_bat_props;
	twl4030_madc_bat->psy.num_properties = ARRAY_SIZE(twl4030_madc_bat_props);
	twl4030_madc_bat->psy.get_property = twl4030_madc_bat_get_property;
	twl4030_madc_bat->psy.external_power_changed = twl4030_madc_bat_ext_changed;

	twl4030_madc_bat->pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, twl4030_madc_bat);
	power_supply_register(&pdev->dev, &twl4030_madc_bat->psy);

	return 0;
}

static int twl4030_madc_battery_remove(struct platform_device *pdev)
{
	struct twl4030_madc_battery *bat = platform_get_drvdata(pdev);

	power_supply_unregister(&bat->psy);
	kfree(bat);

	return 0;
}

static struct platform_driver twl4030_madc_battery_driver = {
	.driver = {
		.name = "twl4030_madc_battery",
	},
	.probe	  = twl4030_madc_battery_probe,
	.remove   = twl4030_madc_battery_remove,
};

static int __init twl4030_madc_battery_init(void)
{
	return platform_driver_register(&twl4030_madc_battery_driver);
}

static void __exit twl4030_madc_battery_exit(void)
{
	platform_driver_unregister(&twl4030_madc_battery_driver);
}

module_init(twl4030_madc_battery_init);
module_exit(twl4030_madc_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lukas Märdian <lukas@goldelico.com>");
MODULE_DESCRIPTION("twl4030_madc battery driver");
