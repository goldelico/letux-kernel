/*
 * Driver for Regulator part of TPS65917 PMIC
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/mfd/tps65917.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/of_regulator.h>

struct regs_info {
	char	*name;
	char	*sname;
	u8	vsel_addr;
	u8	ctrl_addr;
	int	sleep_id;
};

static const struct regs_info tps65917_regs_info[] = {
	{
		.name		= "SMPS1",
		.sname		= "smps1-in",
		.vsel_addr	= TPS65917_SMPS1_VOLTAGE,
		.ctrl_addr	= TPS65917_SMPS1_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_SMPS1,
	},
	{
		.name		= "SMPS2",
		.sname		= "smps2-in",
		.vsel_addr	= TPS65917_SMPS2_VOLTAGE,
		.ctrl_addr	= TPS65917_SMPS2_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_SMPS2,
	},
	{
		.name		= "SMPS3",
		.sname		= "smps3-in",
		.vsel_addr	= TPS65917_SMPS3_VOLTAGE,
		.ctrl_addr	= TPS65917_SMPS3_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_SMPS3,
	},
	{
		.name		= "SMPS4",
		.sname		= "smps4-in",
		.vsel_addr	= TPS65917_SMPS4_VOLTAGE,
		.ctrl_addr	= TPS65917_SMPS4_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_SMPS4,
	},
	{
		.name		= "SMPS5",
		.sname		= "smps5-in",
		.vsel_addr	= TPS65917_SMPS5_VOLTAGE,
		.ctrl_addr	= TPS65917_SMPS5_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_SMPS5,
	},
	{
		.name		= "LDO1",
		.sname		= "ldo1-in",
		.vsel_addr	= TPS65917_LDO1_VOLTAGE,
		.ctrl_addr	= TPS65917_LDO1_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_LDO1,
	},
	{
		.name		= "LDO2",
		.sname		= "ldo2-in",
		.vsel_addr	= TPS65917_LDO2_VOLTAGE,
		.ctrl_addr	= TPS65917_LDO2_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_LDO2,
	},
	{
		.name		= "LDO3",
		.sname		= "ldo3-in",
		.vsel_addr	= TPS65917_LDO3_VOLTAGE,
		.ctrl_addr	= TPS65917_LDO3_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_LDO3,
	},
	{
		.name		= "LDO4",
		.sname		= "ldo4-in",
		.vsel_addr	= TPS65917_LDO4_VOLTAGE,
		.ctrl_addr	= TPS65917_LDO4_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_LDO4,
	},
	{
		.name		= "LDO5",
		.sname		= "ldo5-in",
		.vsel_addr	= TPS65917_LDO5_VOLTAGE,
		.ctrl_addr	= TPS65917_LDO5_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_LDO5,
	},
	{
		.name		= "REGEN1",
		.ctrl_addr	= TPS65917_REGEN1_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_REGEN1,
	},
	{
		.name		= "REGEN2",
		.ctrl_addr	= TPS65917_REGEN2_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_REGEN2,
	},
	{
		.name		= "REGEN3",
		.ctrl_addr	= TPS65917_REGEN3_CTRL,
		.sleep_id	= TPS65917_EXTERNAL_REQSTR_ID_REGEN3,
	},
};

#define SMPS_CTRL_MODE_OFF		0x00
#define SMPS_CTRL_MODE_ON		0x01
#define SMPS_CTRL_MODE_ECO		0x02
#define SMPS_CTRL_MODE_PWM		0x03

#define TPS65917_SMPS_NUM_VOLTAGES	122
#define TPS65917_LDO_NUM_VOLTAGES		50

#define REGULATOR_SLAVE			0

static int tps65917_smps_read(struct tps65917 *tps65917, unsigned int reg,
			      unsigned int *dest)
{
	unsigned int addr;

	addr = TPS65917_BASE_TO_REG(TPS65917_SMPS_BASE, reg);

	return regmap_read(tps65917->regmap[REGULATOR_SLAVE], addr, dest);
}

static int tps65917_smps_write(struct tps65917 *tps65917, unsigned int reg,
			       unsigned int value)
{
	unsigned int addr;

	addr = TPS65917_BASE_TO_REG(TPS65917_SMPS_BASE, reg);

	return regmap_write(tps65917->regmap[REGULATOR_SLAVE], addr, value);
}

static int tps65917_ldo_read(struct tps65917 *tps65917, unsigned int reg,
			     unsigned int *dest)
{
	unsigned int addr;

	addr = TPS65917_BASE_TO_REG(TPS65917_LDO_BASE, reg);

	return regmap_read(tps65917->regmap[REGULATOR_SLAVE], addr, dest);
}

static int tps65917_ldo_write(struct tps65917 *tps65917, unsigned int reg,
			      unsigned int value)
{
	unsigned int addr;

	addr = TPS65917_BASE_TO_REG(TPS65917_LDO_BASE, reg);

	return regmap_write(tps65917->regmap[REGULATOR_SLAVE], addr, value);
}

static int tps65917_is_enabled_smps(struct regulator_dev *dev)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;

	tps65917_smps_read(pmic->tps65917,
			   tps65917_regs_info[id].ctrl_addr, &reg);

	reg &= TPS65917_SMPS1_CTRL_STATUS_MASK;
	reg >>= TPS65917_SMPS1_CTRL_STATUS_SHIFT;

	return !!(reg);
}

static int tps65917_enable_smps(struct regulator_dev *dev)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;

	tps65917_smps_read(pmic->tps65917,
			   tps65917_regs_info[id].ctrl_addr, &reg);

	reg &= ~TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;
	if (pmic->current_reg_mode[id])
		reg |= pmic->current_reg_mode[id];
	else
		reg |= SMPS_CTRL_MODE_ON;

	tps65917_smps_write(pmic->tps65917, tps65917_regs_info[id].ctrl_addr,
			    reg);

	return 0;
}

static int tps65917_disable_smps(struct regulator_dev *dev)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;

	tps65917_smps_read(pmic->tps65917, tps65917_regs_info[id].ctrl_addr,
			   &reg);

	reg &= ~TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;

	tps65917_smps_write(pmic->tps65917, tps65917_regs_info[id].ctrl_addr,
			    reg);

	return 0;
}

static int tps65917_set_mode_smps(struct regulator_dev *dev, unsigned int mode)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;
	bool rail_enable = true;

	tps65917_smps_read(pmic->tps65917, tps65917_regs_info[id].ctrl_addr,
			   &reg);
	reg &= ~TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;

	if (reg == SMPS_CTRL_MODE_OFF)
		rail_enable = false;

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		reg |= SMPS_CTRL_MODE_ON;
		break;
	case REGULATOR_MODE_IDLE:
		reg |= SMPS_CTRL_MODE_ECO;
		break;
	case REGULATOR_MODE_FAST:
		reg |= SMPS_CTRL_MODE_PWM;
		break;
	default:
		return -EINVAL;
	}

	pmic->current_reg_mode[id] = reg & TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;
	if (rail_enable)
		tps65917_smps_write(pmic->tps65917,
				    tps65917_regs_info[id].ctrl_addr, reg);
	return 0;
}

static unsigned int tps65917_get_mode_smps(struct regulator_dev *dev)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;

	reg = pmic->current_reg_mode[id] & TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;

	switch (reg) {
	case SMPS_CTRL_MODE_ON:
		return REGULATOR_MODE_NORMAL;
	case SMPS_CTRL_MODE_ECO:
		return REGULATOR_MODE_IDLE;
	case SMPS_CTRL_MODE_PWM:
		return REGULATOR_MODE_FAST;
	}

	return 0;
}

static int tps65917_list_voltage_smps(struct regulator_dev *dev,
				      unsigned selector)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	int mult = 1;

	/* Read the multiplier set in VSEL register to return
	 * the correct voltage.
	 */
	if (pmic->range[id])
		mult = 2;

	if (selector == 0)
		return 0;
	else if (selector < 6)
		return 500000 * mult;
	else
		/* Voltage is linear mapping starting from selector 6,
		 * volt = (0.49V + ((selector - 5) * 0.01V)) * RANGE
		 * RANGE is either x1 or x2
		 */
		return (490000 + ((selector - 5) * 10000)) * mult;
}

static int tps65917_map_voltage_smps(struct regulator_dev *rdev,
				     int min_uV, int max_uV)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret, voltage;

	if (min_uV == 0)
		return 0;

	if (pmic->range[id]) { /* RANGE is x2 */
		if (min_uV < 1000000)
			min_uV = 1000000;
		ret = DIV_ROUND_UP(min_uV - 1000000, 20000) + 6;
	} else {		/* RANGE is x1 */
		if (min_uV < 500000)
			min_uV = 500000;
		ret = DIV_ROUND_UP(min_uV - 500000, 10000) + 6;
	}

	/* Map back into a voltage to verify we're still in bounds */
	voltage = tps65917_list_voltage_smps(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static struct regulator_ops tps65917_ops_smps = {
	.is_enabled		= tps65917_is_enabled_smps,
	.enable			= tps65917_enable_smps,
	.disable		= tps65917_disable_smps,
	.set_mode		= tps65917_set_mode_smps,
	.get_mode		= tps65917_get_mode_smps,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= tps65917_list_voltage_smps,
	.map_voltage		= tps65917_map_voltage_smps,
};

static struct regulator_ops tps65917_ops_ext_control_smps = {
	.set_mode		= tps65917_set_mode_smps,
	.get_mode		= tps65917_get_mode_smps,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= tps65917_list_voltage_smps,
	.map_voltage		= tps65917_map_voltage_smps,
};

static int tps65917_is_enabled_ldo(struct regulator_dev *dev)
{
	struct tps65917_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	unsigned int reg;

	tps65917_ldo_read(pmic->tps65917, tps65917_regs_info[id].ctrl_addr,
			  &reg);

	reg &= TPS65917_LDO1_CTRL_STATUS;

	return !!(reg);
}

static struct regulator_ops tps65917_ops_ldo = {
	.is_enabled		= tps65917_is_enabled_ldo,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
};

static struct regulator_ops tps65917_ops_ext_control_ldo = {
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
};

static struct regulator_ops tps65917_ops_extreg = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
};

static struct regulator_ops tps65917_ops_ext_control_extreg = {
};

static int tps65917_regulator_config_external(struct tps65917 *tps65917, int id,
					      struct tps65917_reg_init *reg_init)
{
	int sleep_id = tps65917_regs_info[id].sleep_id;
	int ret;

	ret = tps65917_ext_control_req_config(tps65917, sleep_id,
					      reg_init->roof_floor, true);
	if (ret < 0)
		dev_err(tps65917->dev,
			"Ext control config for regulator %d failed %d\n",
			id, ret);
	return ret;
}

/*
 * setup the hardware based sleep configuration of the SMPS/LDO regulators
 * from the platform data. This is different to the software based control
 * supported by the regulator framework as it is controlled by toggling
 * pins on the PMIC such as PREQ, SYSEN, ...
 */
static int tps65917_smps_init(struct tps65917 *tps65917, int id,
			      struct tps65917_reg_init *reg_init)
{
	unsigned int reg;
	unsigned int addr;
	int ret;

	addr = tps65917_regs_info[id].ctrl_addr;

	ret = tps65917_smps_read(tps65917, addr, &reg);
	if (ret)
		return ret;

	if (reg_init->warm_reset)
		reg |= TPS65917_SMPS1_CTRL_WR_S;
	else
		reg &= ~TPS65917_SMPS1_CTRL_WR_S;

	if (reg_init->roof_floor)
		reg |= TPS65917_SMPS1_CTRL_ROOF_FLOOR_EN;
	else
		reg &= ~TPS65917_SMPS1_CTRL_ROOF_FLOOR_EN;

	reg &= ~TPS65917_SMPS1_CTRL_MODE_SLEEP_MASK;
	if (reg_init->mode_sleep)
		reg |= reg_init->mode_sleep <<
				TPS65917_SMPS1_CTRL_MODE_SLEEP_SHIFT;


	ret = tps65917_smps_write(tps65917, addr, reg);
	if (ret)
		return ret;

	if (tps65917_regs_info[id].vsel_addr && reg_init->vsel) {
		addr = tps65917_regs_info[id].vsel_addr;

		reg = reg_init->vsel;

		ret = tps65917_smps_write(tps65917, addr, reg);
		if (ret)
			return ret;
	}

	if (reg_init->roof_floor) {
		/* Enable externally controlled regulator */
		addr = tps65917_regs_info[id].ctrl_addr;
		ret = tps65917_smps_read(tps65917, addr, &reg);
		if (ret < 0)
			return ret;

		if (!(reg & TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK)) {
			reg |= SMPS_CTRL_MODE_ON;
			ret = tps65917_smps_write(tps65917, addr, reg);
			if (ret < 0)
				return ret;
		}
		return tps65917_regulator_config_external(tps65917, id,
							  reg_init);
	}
	return 0;
}

static int tps65917_ldo_init(struct tps65917 *tps65917, int id,
			     struct tps65917_reg_init *reg_init)
{
	unsigned int reg;
	unsigned int addr;
	int ret;

	addr = tps65917_regs_info[id].ctrl_addr;

	ret = tps65917_ldo_read(tps65917, addr, &reg);
	if (ret)
		return ret;

	if (reg_init->warm_reset)
		reg |= TPS65917_LDO1_CTRL_WR_S;
	else
		reg &= ~TPS65917_LDO1_CTRL_WR_S;

	if (reg_init->mode_sleep)
		reg |= TPS65917_LDO1_CTRL_MODE_SLEEP;
	else
		reg &= ~TPS65917_LDO1_CTRL_MODE_SLEEP;

	ret = tps65917_ldo_write(tps65917, addr, reg);
	if (ret)
		return ret;

	if (reg_init->roof_floor) {
		/* Enable externally controlled regulator */
		addr = tps65917_regs_info[id].ctrl_addr;
		ret = tps65917_update_bits(tps65917, TPS65917_LDO_BASE,
					   addr, TPS65917_LDO1_CTRL_MODE_ACTIVE,
					   TPS65917_LDO1_CTRL_MODE_ACTIVE);
		if (ret < 0) {
			dev_err(tps65917->dev,
				"LDO Register 0x%02x update failed %d\n",
				addr, ret);
			return ret;
		}
		return tps65917_regulator_config_external(tps65917, id,
							  reg_init);
	}
	return 0;
}

static int tps65917_extreg_init(struct tps65917 *tps65917, int id,
				struct tps65917_reg_init *reg_init)
{
	unsigned int addr;
	int ret;
	unsigned int val = 0;

	addr = tps65917_regs_info[id].ctrl_addr;

	if (reg_init->mode_sleep)
		val = TPS65917_REGEN1_CTRL_MODE_SLEEP;

	ret = tps65917_update_bits(tps65917, TPS65917_RESOURCE_BASE,
				   addr, TPS65917_REGEN1_CTRL_MODE_SLEEP, val);
	if (ret < 0) {
		dev_err(tps65917->dev, "Resource reg 0x%02x update failed %d\n",
			addr, ret);
		return ret;
	}

	if (reg_init->roof_floor) {
		/* Enable externally controlled regulator */
		addr = tps65917_regs_info[id].ctrl_addr;
		ret = tps65917_update_bits(tps65917, TPS65917_RESOURCE_BASE,
					   addr, TPS65917_REGEN1_CTRL_MODE_ACTIVE,
					   TPS65917_REGEN1_CTRL_MODE_ACTIVE);
		if (ret < 0) {
			dev_err(tps65917->dev,
				"Resource Register 0x%02x update failed %d\n",
				addr, ret);
			return ret;
		}
		return tps65917_regulator_config_external(tps65917, id,
							  reg_init);
	}
	return 0;
}

static struct of_regulator_match tps65917_matches[] = {
	{ .name = "smps1", },
	{ .name = "smps2", },
	{ .name = "smps3", },
	{ .name = "smps4", },
	{ .name = "smps5", },
	{ .name = "ldo1", },
	{ .name = "ldo2", },
	{ .name = "ldo3", },
	{ .name = "ldo4", },
	{ .name = "ldo5", },
	{ .name = "regen1", },
	{ .name = "regen2", },
	{ .name = "regen3", },
	{ .name = "sysen1", },
	{ .name = "sysen2", },
};

static void tps65917_dt_to_pdata(struct device *dev,
				 struct device_node *node,
				 struct tps65917_pmic_platform_data *pdata)
{
	struct device_node *regulators;
	u32 prop;
	int idx, ret;

	node = of_node_get(node);
	regulators = of_get_child_by_name(node, "regulators");
	if (!regulators) {
		dev_info(dev, "regulator node not found\n");
		return;
	}

	ret = of_regulator_match(dev, regulators, tps65917_matches,
				 TPS65917_NUM_REGS);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(dev, "Error parsing regulator init data: %d\n", ret);
		return;
	}

	for (idx = 0; idx < TPS65917_NUM_REGS; idx++) {
		if (!tps65917_matches[idx].init_data ||
		    !tps65917_matches[idx].of_node)
			continue;

		pdata->reg_data[idx] = tps65917_matches[idx].init_data;

		pdata->reg_init[idx] = devm_kzalloc(dev,
						    sizeof(struct tps65917_reg_init), GFP_KERNEL);

		pdata->reg_init[idx]->warm_reset =
			of_property_read_bool(tps65917_matches[idx].of_node,
					      "ti,warm-reset");

		ret = of_property_read_u32(tps65917_matches[idx].of_node,
					   "ti,roof-floor", &prop);
		/* EINVAL: Property not found */
		if (ret != -EINVAL) {
			int econtrol;

			/* use default value, when no value is specified */
			econtrol = TPS65917_EXT_CONTROL_NSLEEP;
			if (!ret) {
				switch (prop) {
				case 1:
					econtrol = TPS65917_EXT_CONTROL_ENABLE1;
					break;
				case 2:
					econtrol = TPS65917_EXT_CONTROL_ENABLE2;
					break;
				case 3:
					econtrol = TPS65917_EXT_CONTROL_NSLEEP;
					break;
				default:
					WARN_ON(1);
					dev_warn(dev,
						 "%s: Invalid roof-floor option: %u\n",
						 tps65917_matches[idx].name, prop);
					break;
				}
			}
			pdata->reg_init[idx]->roof_floor = econtrol;
		}

		ret = of_property_read_u32(tps65917_matches[idx].of_node,
					   "ti,mode-sleep", &prop);
		if (!ret)
			pdata->reg_init[idx]->mode_sleep = prop;

		ret = of_property_read_bool(tps65917_matches[idx].of_node,
					    "ti,smps-range");
		if (ret)
			pdata->reg_init[idx]->vsel =
				TPS65917_SMPS1_VOLTAGE_RANGE;
	}
}


static int tps65917_regulators_probe(struct platform_device *pdev)
{
	struct tps65917 *tps65917 = dev_get_drvdata(pdev->dev.parent);
	struct tps65917_pmic_platform_data *pdata;
	struct device_node *node = pdev->dev.of_node;
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	struct tps65917_pmic *pmic;
	struct tps65917_reg_init *reg_init;
	int id = 0, ret;
	unsigned int addr, reg;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return -ENOMEM;

	tps65917_dt_to_pdata(&pdev->dev, node, pdata);

	pmic = devm_kzalloc(&pdev->dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->dev = &pdev->dev;
	pmic->tps65917 = tps65917;
	tps65917->pmic = pmic;
	platform_set_drvdata(pdev, pmic);

	ret = tps65917_smps_read(tps65917, TPS65917_SMPS_CTRL, &reg);
	if (ret)
		return ret;

	if (reg & TPS65917_SMPS_CTRL_SMPS1_SMPS12_EN)
		pmic->smps12 = 1;

	config.regmap = tps65917->regmap[REGULATOR_SLAVE];
	config.dev = &pdev->dev;
	config.driver_data = pmic;

	for (id = 0; id < TPS65917_REG_LDO1; id++) {
		/*
		 * Miss out regulators which are not available due
		 * to slaving configurations.
		 */
		if ((id == TPS65917_REG_SMPS2) && pmic->smps12)
			continue;

		/* Initialise sleep/init values from platform data */
		if (pdata && pdata->reg_init[id]) {
			reg_init = pdata->reg_init[id];
			ret = tps65917_smps_init(tps65917, id, reg_init);
			if (ret)
				return ret;
		} else {
			reg_init = NULL;
		}

		/* Register the regulators */
		pmic->desc[id].name = tps65917_regs_info[id].name;
		pmic->desc[id].id = id;

		/*
		 * Read and store the RANGE bit for later use
		 * This must be done before regulator is probed,
		 * otherwise we error in probe with unsupportable
		 * ranges. Read the current smps mode for later use.
		 */
		addr = tps65917_regs_info[id].vsel_addr;

		ret = tps65917_smps_read(pmic->tps65917, addr, &reg);
		if (ret)
			return ret;
		if (reg & TPS65917_SMPS1_VOLTAGE_RANGE)
			pmic->range[id] = 1;

		if (reg_init && reg_init->roof_floor)
			pmic->desc[id].ops =
					&tps65917_ops_ext_control_smps;
		else
			pmic->desc[id].ops = &tps65917_ops_smps;
		pmic->desc[id].n_voltages = TPS65917_SMPS_NUM_VOLTAGES;
		pmic->desc[id].vsel_reg =
				TPS65917_BASE_TO_REG(TPS65917_SMPS_BASE,
						     tps65917_regs_info[id].vsel_addr);
		pmic->desc[id].vsel_mask =
				TPS65917_SMPS1_VOLTAGE_VSEL_MASK;

		/* Read the smps mode for later use. */
		addr = tps65917_regs_info[id].ctrl_addr;
		ret = tps65917_smps_read(pmic->tps65917, addr, &reg);
		if (ret)
			return ret;
		pmic->current_reg_mode[id] = reg &
				TPS65917_SMPS1_CTRL_MODE_ACTIVE_MASK;

		pmic->desc[id].type = REGULATOR_VOLTAGE;
		pmic->desc[id].owner = THIS_MODULE;

		if (pdata)
			config.init_data = pdata->reg_data[id];
		else
			config.init_data = NULL;

		pmic->desc[id].supply_name = tps65917_regs_info[id].sname;
		config.of_node = tps65917_matches[id].of_node;

		rdev = regulator_register(&pmic->desc[id], &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				pdev->name);
			return PTR_ERR(rdev);
		}

		/* Save regulator for cleanup */
		pmic->rdev[id] = rdev;
	}

	/* Start this loop from the id left from previous loop */
	for (; id < TPS65917_NUM_REGS; id++) {
		if (pdata && pdata->reg_init[id])
			reg_init = pdata->reg_init[id];
		else
			reg_init = NULL;

		/* Miss out regulators which are not available due
		 * to alternate functions.
		 */

		/* Register the regulators */
		pmic->desc[id].name = tps65917_regs_info[id].name;
		pmic->desc[id].id = id;
		pmic->desc[id].type = REGULATOR_VOLTAGE;
		pmic->desc[id].owner = THIS_MODULE;

		if (id < TPS65917_REG_REGEN1) {
			pmic->desc[id].n_voltages = TPS65917_LDO_NUM_VOLTAGES;
			if (reg_init && reg_init->roof_floor)
				pmic->desc[id].ops =
					&tps65917_ops_ext_control_ldo;
			else
				pmic->desc[id].ops = &tps65917_ops_ldo;
			pmic->desc[id].min_uV = 900000;
			pmic->desc[id].uV_step = 50000;
			pmic->desc[id].linear_min_sel = 1;
			pmic->desc[id].enable_time = 500;
			pmic->desc[id].vsel_reg =
					TPS65917_BASE_TO_REG(TPS65917_LDO_BASE,
							     tps65917_regs_info[id].vsel_addr);
			pmic->desc[id].vsel_mask =
					TPS65917_LDO1_VOLTAGE_VSEL_MASK;
			pmic->desc[id].enable_reg =
					TPS65917_BASE_TO_REG(TPS65917_LDO_BASE,
							     tps65917_regs_info[id].ctrl_addr);
			pmic->desc[id].enable_mask =
					TPS65917_LDO1_CTRL_MODE_ACTIVE;
		} else {
			pmic->desc[id].n_voltages = 1;
			if (reg_init && reg_init->roof_floor)
				pmic->desc[id].ops =
					&tps65917_ops_ext_control_extreg;
			else
				pmic->desc[id].ops = &tps65917_ops_extreg;
			pmic->desc[id].enable_reg =
					TPS65917_BASE_TO_REG(TPS65917_RESOURCE_BASE,
							     tps65917_regs_info[id].ctrl_addr);
			pmic->desc[id].enable_mask =
					TPS65917_REGEN1_CTRL_MODE_ACTIVE;
		}

		if (pdata)
			config.init_data = pdata->reg_data[id];
		else
			config.init_data = NULL;

		pmic->desc[id].supply_name = tps65917_regs_info[id].sname;
		config.of_node = tps65917_matches[id].of_node;

		rdev = regulator_register(&pmic->desc[id], &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				pdev->name);
			return PTR_ERR(rdev);
		}

		/* Save regulator for cleanup */
		pmic->rdev[id] = rdev;

		/* Initialise sleep/init values from platform data */
		if (pdata) {
			reg_init = pdata->reg_init[id];
			if (reg_init) {
				if (id < TPS65917_REG_REGEN1)
					ret = tps65917_ldo_init(tps65917,
								id, reg_init);
				else
					ret = tps65917_extreg_init(tps65917,
								   id, reg_init);
				if (ret)
					return ret;
			}
		}
	}


	return 0;
}

static struct of_device_id of_tps65917_match_tbl[] = {
	{ .compatible = "ti,tps65917-pmic", },
};

static struct platform_driver tps65917_driver = {
	.driver = {
		.name = "tps65917-pmic",
		.of_match_table = of_tps65917_match_tbl,
		.owner = THIS_MODULE,
	},
	.probe = tps65917_regulators_probe,
};

static int __init tps65917_init(void)
{
	return platform_driver_register(&tps65917_driver);
}
subsys_initcall(tps65917_init);

static void __exit tps65917_exit(void)
{
	platform_driver_unregister(&tps65917_driver);
}
module_exit(tps65917_exit);

MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
MODULE_DESCRIPTION("TPS65917 voltage regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps65917-pmic");
MODULE_DEVICE_TABLE(of, of_tps65917_match_tbl);
