/* Philips PCF50633 PMIC Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50633 driver mainly by
 * Harald Welte and Andy Green
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/regulator/driver.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/pmic.h>

#define PCF50633_REGULATOR(_name, _id) 		\
	{					\
		.name = _name, 			\
		.id = _id,			\
		.ops = &pcf50633_regulator_ops,	\
		.type = REGULATOR_VOLTAGE, 	\
		.owner = THIS_MODULE, 		\
	}
static const u8 pcf50633_regulator_registers[PCF50633_NUM_REGULATORS] = {
	[PCF50633_REGULATOR_AUTO]	= PCF50633_REG_AUTOOUT,
	[PCF50633_REGULATOR_DOWN1]	= PCF50633_REG_DOWN1OUT,
	[PCF50633_REGULATOR_DOWN2]	= PCF50633_REG_DOWN2OUT,
	[PCF50633_REGULATOR_MEMLDO]	= PCF50633_REG_MEMLDOOUT,
	[PCF50633_REGULATOR_LDO1]	= PCF50633_REG_LDO1OUT,
	[PCF50633_REGULATOR_LDO2]	= PCF50633_REG_LDO2OUT,
	[PCF50633_REGULATOR_LDO3]	= PCF50633_REG_LDO3OUT,
	[PCF50633_REGULATOR_LDO4]	= PCF50633_REG_LDO4OUT,
	[PCF50633_REGULATOR_LDO5]	= PCF50633_REG_LDO5OUT,
	[PCF50633_REGULATOR_LDO6]	= PCF50633_REG_LDO6OUT,
	[PCF50633_REGULATOR_HCLDO]	= PCF50633_REG_HCLDOOUT,
};

/* Bits from voltage value */
static u_int8_t auto_voltage_bits(unsigned int millivolts)
{
	if (millivolts < 1800)
		return 0;
	if (millivolts > 3800)
		return 0xff;

	millivolts -= 625;
	return millivolts/25;
}

static u_int8_t down_voltage_bits(unsigned int millivolts)
{
	if (millivolts < 625)
		return 0;
	else if (millivolts > 3000)
		return 0xff;

	millivolts -= 625;
	return millivolts/25;
}

static u_int8_t ldo_voltage_bits(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3600)
		return 0x1f;

	millivolts -= 900;
	return millivolts/100;
}

/* Obtain voltage value from bits */

static unsigned int auto_voltage_value(uint8_t bits)
{
	if (bits < 0x2f)
		return 0;
	return 625 + (bits * 25);
}


static unsigned int down_voltage_value(uint8_t bits)
{
	return 625 + (bits*25);
}


static unsigned int ldo_voltage_value(uint8_t bits)
{
	bits &= 0x1f;
	return 900 + (bits * 100);
}

static int pcf50633_regulator_set_voltage(struct regulator_dev *rdev,
			int min_uV, int max_uV)
{
	uint8_t volt_bits;
	uint8_t regnr;
	int regulator_id;
	int millivolts;
	struct pcf50633 *pcf = rdev_get_drvdata(rdev);;

	regulator_id = rdev_get_id(rdev);

	if (regulator_id >= PCF50633_NUM_REGULATORS)
		return -EINVAL;

	millivolts = min_uV / 1000;

	regnr = pcf50633_regulator_registers[regulator_id];

	switch (regulator_id) {
	case PCF50633_REGULATOR_AUTO:
		volt_bits = auto_voltage_bits(millivolts);
		break;
	case PCF50633_REGULATOR_DOWN1:
		volt_bits = down_voltage_bits(millivolts);
		break;
	case PCF50633_REGULATOR_DOWN2:
		volt_bits = down_voltage_bits(millivolts);
		break;
	case PCF50633_REGULATOR_LDO1:
	case PCF50633_REGULATOR_LDO2:
	case PCF50633_REGULATOR_LDO3:
	case PCF50633_REGULATOR_LDO4:
	case PCF50633_REGULATOR_LDO5:
	case PCF50633_REGULATOR_LDO6:
	case PCF50633_REGULATOR_HCLDO:
		volt_bits = ldo_voltage_bits(millivolts);
		break;
	default:
		return -EINVAL;
	}

	return pcf50633_reg_write(pcf, regnr, volt_bits);
}

static int pcf50633_regulator_get_voltage(struct regulator_dev *rdev)
{
	uint8_t volt_bits;
	uint8_t regnr;
	unsigned int rc = 0;
	int regulator_id = rdev_get_id(rdev);
	struct pcf50633 *pcf = rdev_get_drvdata(rdev);

	if (regulator_id >= PCF50633_NUM_REGULATORS)
		return -EINVAL;

	regnr = pcf50633_regulator_registers[regulator_id];
	volt_bits = pcf50633_reg_read(pcf, regnr);

	switch (regulator_id) {
	case PCF50633_REGULATOR_AUTO:
		rc = auto_voltage_value(volt_bits);
		break;
	case PCF50633_REGULATOR_DOWN1:
		rc = down_voltage_value(volt_bits);
		break;
	case PCF50633_REGULATOR_DOWN2:
		rc = down_voltage_value(volt_bits);
		break;
	case PCF50633_REGULATOR_LDO1:
	case PCF50633_REGULATOR_LDO2:
	case PCF50633_REGULATOR_LDO3:
	case PCF50633_REGULATOR_LDO4:
	case PCF50633_REGULATOR_LDO5:
	case PCF50633_REGULATOR_LDO6:
	case PCF50633_REGULATOR_HCLDO:
		rc = ldo_voltage_value(volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc * 1000;
}

static int pcf50633_regulator_enable(struct regulator_dev *rdev)
{
	uint8_t regnr;
	int regulator_id = rdev_get_id(rdev);
	struct pcf50633 *pcf = rdev_get_drvdata(rdev);

	if (regulator_id >= PCF50633_NUM_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	regnr = pcf50633_regulator_registers[regulator_id] + 1;

	pcf50633_reg_set_bit_mask(pcf, regnr, PCF50633_REGULATOR_ON,
		       PCF50633_REGULATOR_ON);

	return 0;
}

static int pcf50633_regulator_disable(struct regulator_dev *rdev)
{
	uint8_t regnr;
	int regulator_id = rdev_get_id(rdev);
	struct pcf50633 *pcf = rdev_get_drvdata(rdev);

	if (regulator_id >= PCF50633_NUM_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	regnr = pcf50633_regulator_registers[regulator_id] + 1;

	pcf50633_reg_set_bit_mask(pcf, regnr, PCF50633_REGULATOR_ON, 0);

	return 0;
}

static int pcf50633_regulator_is_enabled(struct regulator_dev *rdev)
{
	uint8_t val, regnr;
	int regulator_id = rdev_get_id(rdev);
	struct pcf50633 *pcf = rdev_get_drvdata(rdev);

	if (regulator_id >= PCF50633_NUM_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	regnr = pcf50633_regulator_registers[regulator_id] + 1;
	val = pcf50633_reg_read(pcf, regnr) & PCF50633_REGULATOR_ON;

	return val;
}

struct regulator_ops pcf50633_regulator_ops = {
	.set_voltage = pcf50633_regulator_set_voltage,
	.get_voltage = pcf50633_regulator_get_voltage,
	.enable = pcf50633_regulator_enable,
	.disable = pcf50633_regulator_disable,
	.is_enabled = pcf50633_regulator_is_enabled,
};

static struct regulator_desc regulators[] = {
	[PCF50633_REGULATOR_AUTO] =
		PCF50633_REGULATOR("auto", PCF50633_REGULATOR_AUTO),
	[PCF50633_REGULATOR_DOWN1] =
		PCF50633_REGULATOR("down1", PCF50633_REGULATOR_DOWN1),
	[PCF50633_REGULATOR_DOWN2] =
		PCF50633_REGULATOR("down2", PCF50633_REGULATOR_DOWN2),
	[PCF50633_REGULATOR_LDO1] =
		PCF50633_REGULATOR("ldo1", PCF50633_REGULATOR_LDO1),
	[PCF50633_REGULATOR_LDO2] =
		PCF50633_REGULATOR("ldo2", PCF50633_REGULATOR_LDO2),
	[PCF50633_REGULATOR_LDO3] =
		PCF50633_REGULATOR("ldo3", PCF50633_REGULATOR_LDO3),
	[PCF50633_REGULATOR_LDO4] =
		PCF50633_REGULATOR("ldo4", PCF50633_REGULATOR_LDO4),
	[PCF50633_REGULATOR_LDO5] =
		PCF50633_REGULATOR("ldo5", PCF50633_REGULATOR_LDO5),
	[PCF50633_REGULATOR_LDO6] =
		PCF50633_REGULATOR("ldo6", PCF50633_REGULATOR_LDO6),
	[PCF50633_REGULATOR_HCLDO] =
		PCF50633_REGULATOR("hcldo", PCF50633_REGULATOR_HCLDO),
	[PCF50633_REGULATOR_MEMLDO] =
		PCF50633_REGULATOR("memldo", PCF50633_REGULATOR_MEMLDO),
};

int __init pcf50633_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct pcf50633 *pcf;

	pcf = pdev->dev.driver_data;

	rdev = regulator_register(&regulators[pdev->id], &pdev->dev, pcf);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	if (pcf->pdata->regulator_registered)
		pcf->pdata->regulator_registered(pcf, pdev->id);

	return 0;
}

static int __devexit pcf50633_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

struct platform_driver pcf50633_regulator_driver = {
	.driver = {
		.name = "pcf50633-regltr",
	},
	.probe = pcf50633_regulator_probe,
	.remove = __devexit_p(pcf50633_regulator_remove),
};

static int __init pcf50633_regulator_init(void)
{
	return platform_driver_register(&pcf50633_regulator_driver);
}
module_init(pcf50633_regulator_init);

static void __exit pcf50633_regulator_exit(void)
{
	platform_driver_unregister(&pcf50633_regulator_driver);
}
module_exit(pcf50633_regulator_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-regulator");
