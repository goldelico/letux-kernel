// SPDX-License-Identifier: GPL-2.0+
/*
 * Fuel gauge driver for the RICOH RN5T618 power management chip family
 *
 * Copyright (C) 2020 Andreas Kemnade
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/rn5t618.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define CHG_STATE_CHG_OFF	0
#define CHG_STATE_CHG_READY_VADP	1
#define CHG_STATE_CHG_TRICKLE	2
#define CHG_STATE_CHG_RAPID	3
#define CHG_STATE_CHG_COMPLETE	4
#define CHG_STATE_SUSPEND	5
#define CHG_STATE_VCHG_OVER_VOL	6
#define CHG_STATE_BAT_ERROR	7
#define CHG_STATE_NO_BAT	8
#define CHG_STATE_BAT_OVER_VOL	9
#define CHG_STATE_BAT_TEMP_ERR	10
#define CHG_STATE_DIE_ERR	11
#define CHG_STATE_DIE_SHUTDOWN	12
#define CHG_STATE_NO_BAT2	13
#define CHG_STATE_CHG_READY_VUSB	14

struct rn5t618_gauge_info {
	struct rn5t618 *rn5t618;
	struct platform_device *pdev;
	struct power_supply *psy;
};

static enum power_supply_property rn5t618_gauge_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int rn5t618_gauge_read_doublereg(struct rn5t618_gauge_info *info,
					u8 reg, u16 *result)
{
	int ret;
	u8 data[2];

	ret = regmap_bulk_read(info->rn5t618->regmap, reg, data, sizeof(data));
	if (ret)
		return ret;

	*result = data[0] << 8;
	*result |= data[1];

	return 0;
}

static int rn5t618_gauge_status(struct rn5t618_gauge_info *info,
				union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &v);
	if (ret)
		return ret;

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

	if (v & 0xc0) { /* USB or ADP plugged */	
		switch(v & 0x1f) {
		case CHG_STATE_CHG_OFF:
		case CHG_STATE_SUSPEND:
		case CHG_STATE_VCHG_OVER_VOL:
		case CHG_STATE_DIE_SHUTDOWN:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;

		case CHG_STATE_CHG_READY_VADP:
		case CHG_STATE_CHG_READY_VUSB:
		case CHG_STATE_BAT_ERROR:
		case CHG_STATE_NO_BAT:
		case CHG_STATE_NO_BAT2:
		case CHG_STATE_BAT_OVER_VOL:
		case CHG_STATE_BAT_TEMP_ERR:
		case CHG_STATE_DIE_ERR:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;

		case CHG_STATE_CHG_TRICKLE:
		case CHG_STATE_CHG_RAPID:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;

		case CHG_STATE_CHG_COMPLETE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;

		}
	} else
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

	return ret;
}

static int rn5t618_gauge_voltage_now(struct rn5t618_gauge_info *info,
				     union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_VOLTAGE_1, &res);
	if (ret)
		return ret;

	val->intval = res * 2 * 2500 / 4095 * 1000;

	return 0;
}

static int rn5t618_gauge_current_now(struct rn5t618_gauge_info *info,
				union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_CC_AVEREG1, &res);
	if (ret)
		return ret;

	val->intval = res;
	if (val->intval & (1 << 13))
		val->intval = val->intval - (1 << 14);

	val->intval *= 1000;

	return 0;
}

static int rn5t618_gauge_capacity(struct rn5t618_gauge_info *info,
				  union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_SOC, &v);
	if (ret)
		return ret;

	val->intval = v;

	return 0;
}

static int rn5t618_gauge_temp(struct rn5t618_gauge_info *info,
			      union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_TEMP_1, &res);
	if (ret)
		return ret;

	val->intval = res;
	if (val->intval & (1 << 11))
		val->intval = val->intval - (1 << 12);

	val->intval /= 16;

	return 0;
}

static int rn5t618_gauge_tte(struct rn5t618_gauge_info *info,
			     union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_TT_EMPTY_H, &res);
	if (ret)
		return ret;

	if (res == 65535)
		return -ENODATA;

	val->intval = res * 60;

	return 0;
}

static int rn5t618_gauge_ttf(struct rn5t618_gauge_info *info,
			     union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_TT_FULL_H, &res);
	if (ret)
		return ret;

	if (res == 65535)
		return -ENODATA;

	val->intval = res * 60;

	return 0;
}

static int rn5t618_gauge_charge_full(struct rn5t618_gauge_info *info,
				     union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_FA_CAP_H, &res);
	if (ret)
		return ret;

	val->intval = res * 1000;

	return 0;
}

static int rn5t618_gauge_charge_now(struct rn5t618_gauge_info *info,
				    union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_gauge_read_doublereg(info, RN5T618_RE_CAP_H, &res);
	if (ret)
		return ret;

	val->intval = res * 1000;

	return 0;
}

static int rn5t618_gauge_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;
        struct rn5t618_gauge_info *info = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = rn5t618_gauge_status(info, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = rn5t618_gauge_voltage_now(info, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = rn5t618_gauge_current_now(info, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = rn5t618_gauge_capacity(info, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = rn5t618_gauge_temp(info, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = rn5t618_gauge_tte(info, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = rn5t618_gauge_ttf(info, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = rn5t618_gauge_charge_full(info, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = rn5t618_gauge_charge_now(info, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void rn5t618_gauge_external_power_changed(struct power_supply *psy)
{
}

static const struct power_supply_desc rn5t618_gauge_desc = {
	.name                   = "rn5t618-gauge",
        .type                   = POWER_SUPPLY_TYPE_BATTERY,
        .properties             = rn5t618_gauge_props,
        .num_properties         = ARRAY_SIZE(rn5t618_gauge_props),
        .get_property           = rn5t618_gauge_get_property,
        .external_power_changed = rn5t618_gauge_external_power_changed,

};

static int rn5t618_gauge_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int v;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {};
	struct rn5t618_gauge_info *info;

        info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	info->rn5t618 = dev_get_drvdata(pdev->dev.parent);

        platform_set_drvdata(pdev, info);

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CONTROL, &v);
	if (ret)
		return ret;

	if (! (v & 1)) {
		dev_info(&pdev->dev, "Fuel gauge not enabled, enabling now\n");
		dev_info(&pdev->dev, "Expect unprecise results\n");
		regmap_update_bits(info->rn5t618->regmap, RN5T618_CONTROL,
				   1, 1);
	}

	psy_cfg.drv_data = info;
	info->psy = devm_power_supply_register_no_ws(&pdev->dev, &rn5t618_gauge_desc, &psy_cfg);
	if (IS_ERR(info->psy)) {
		ret = PTR_ERR(info->psy);
		dev_err(&pdev->dev, "failed to register gauge: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver rn5t618_gauge_driver = {
        .driver = {
                .name   = "rn5t618-gauge",
        },
        .probe = rn5t618_gauge_probe,
};

module_platform_driver(rn5t618_gauge_driver);
MODULE_ALIAS("platform:rn5t618-gauge");
MODULE_DESCRIPTION("RICOH RN5T618 Fuel gauge driver");
MODULE_LICENSE("GPL");

