/* NXP PCF50606 PMIC Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50606 driver mainly by
 * Harald Welte and Andy Green and Werner Almesberger
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <linux/mfd/pcf50606/core.h>
#include <linux/mfd/pcf50606/pmic.h>

#define PCF50606_REGULATOR(_name, _id) 		\
	{					\
		.name = _name, 			\
		.id = _id,			\
		.ops = &pcf50606_regulator_ops,	\
		.type = REGULATOR_VOLTAGE, 	\
		.owner = THIS_MODULE, 		\
	}

static const u8 pcf50606_regulator_registers[PCF50606_NUM_REGULATORS] = {
	[PCF50606_REGULATOR_DCD]	= PCF50606_REG_DCDC1,
	[PCF50606_REGULATOR_DCDE]	= PCF50606_REG_DCDEC1,
	[PCF50606_REGULATOR_DCUD]	= PCF50606_REG_DCUDC1,
	[PCF50606_REGULATOR_D1REG]	= PCF50606_REG_D1REGC1,
	[PCF50606_REGULATOR_D2REG]	= PCF50606_REG_D2REGC1,
	[PCF50606_REGULATOR_D3REG]	= PCF50606_REG_D3REGC1,
	[PCF50606_REGULATOR_LPREG]	= PCF50606_REG_LPREGC1,
	[PCF50606_REGULATOR_IOREG]	= PCF50606_REG_IOREGC,
};

static u8 dcudc_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	if (millivolts > 5500)
		return 0x1f;
	if (millivolts <= 3300) {
		millivolts -= 900;
		return millivolts/300;
	}
	if (millivolts < 4000)
		return 0x0f;
	else {
		millivolts -= 4000;
		return millivolts/100;
	}
}

static unsigned int dcudc_2voltage(u8 bits)
{
	bits &= 0x1f;
	if (bits < 0x08)
		return 900 + bits * 300;
	else if (bits < 0x10)
		return 3300;
	else
		return 4000 + bits * 100;
}

static u8 dcdec_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3300)
		return 0x0f;

	millivolts -= 900;
	return millivolts/300;
}

static unsigned int dcdec_2voltage(u8 bits)
{
	bits &= 0x0f;
	return 900 + bits*300;
}

static u8 dcdc_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3600)
		return 0x1f;

	if (millivolts < 1500) {
		millivolts -= 900;
		return millivolts/25;
	} else {
		millivolts -= 1500;
		return 0x18 + millivolts/300;
	}
}

static unsigned int dcdc_2voltage(u8 bits)
{
	bits &= 0x1f;
	if ((bits & 0x18) == 0x18)
		return 1500 + ((bits & 0x7) * 300);
	else
		return 900 + (bits * 25);
}

static u8 dx_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3300)
		return 0x18;

	millivolts -= 900;
	return millivolts/100;
}

static unsigned int dx_2voltage(u8 bits)
{
	bits &= 0x1f;
	return 900 + (bits * 100);
}

static int pcf50606_regulator_set_voltage(struct regulator_dev *rdev,
						int min_uV, int max_uV)
{
	struct pcf50606 *pcf;
	int regulator_id, millivolts, rc;
	u8 volt_bits, regnr;

	pcf = rdev_get_drvdata(rdev);

	regulator_id = rdev_get_id(rdev);
	if (regulator_id >= PCF50606_NUM_REGULATORS)
		return -EINVAL;

	millivolts = min_uV / 1000;

	switch (regulator_id) {
	case PCF50606_REGULATOR_DCD:
		volt_bits = dcdc_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_DCDC1, 0x1f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_DCDE:
		volt_bits = dcdec_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_DCDEC1, 0x0f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_DCUD:
		volt_bits = dcudc_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_DCUDC1, 0x1f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_D1REG:
	case PCF50606_REGULATOR_D2REG:
	case PCF50606_REGULATOR_D3REG:
		regnr = PCF50606_REG_D1REGC1 +
		       		(regulator_id - PCF50606_REGULATOR_D1REG);
		volt_bits = dx_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, regnr, 0x1f, volt_bits);
		break;
	case PCF50606_REGULATOR_LPREG:
		volt_bits = dx_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_LPREGC1, 0x1f,
					      volt_bits);
		break;
	case PCF50606_REGULATOR_IOREG:
		if (millivolts < 1800)
			return -EINVAL;
		volt_bits = dx_voltage(millivolts);
		rc = pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_IOREGC, 0x1f,
					      volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int pcf50606_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct pcf50606 *pcf;
	u8 volt_bits, regnr;
	int rc = 0, regulator_id;


	pcf = rdev_get_drvdata(rdev);

	regulator_id = rdev_get_id(rdev);
	if (regulator_id >= PCF50606_NUM_REGULATORS)
		return -EINVAL;

	switch (regulator_id) {
	case PCF50606_REGULATOR_DCD:
		volt_bits = pcf50606_reg_read(pcf, PCF50606_REG_DCDC1) & 0x1f;
		rc = dcdc_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_DCDE:
		volt_bits = pcf50606_reg_read(pcf, PCF50606_REG_DCDEC1) & 0x0f;
		rc = dcdec_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_DCUD:
		volt_bits = pcf50606_reg_read(pcf, PCF50606_REG_DCUDC1) & 0x1f;
		rc = dcudc_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_D1REG:
	case PCF50606_REGULATOR_D2REG:
	case PCF50606_REGULATOR_D3REG:
		regnr = PCF50606_REG_D1REGC1 + (regulator_id - PCF50606_REGULATOR_D1REG);
		volt_bits = pcf50606_reg_read(pcf, regnr) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_LPREG:
		volt_bits = pcf50606_reg_read(pcf, PCF50606_REG_LPREGC1) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_IOREG:
		volt_bits = pcf50606_reg_read(pcf, PCF50606_REG_IOREGC) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc * 1000;

}

static int pcf50606_regulator_enable(struct regulator_dev *rdev)
{
	struct pcf50606 *pcf = rdev_get_drvdata(rdev);
	int regulator_id;
	u8 regnr;

	regulator_id = rdev_get_id(rdev);
	if (regulator_id >= PCF50606_NUM_REGULATORS)
		return -EINVAL;
	
	regnr = pcf50606_regulator_registers[regulator_id];

	return pcf50606_reg_set_bit_mask(pcf, regnr, 0xe0, 0xe0);
}

static int pcf50606_regulator_disable(struct regulator_dev *rdev)
{
	struct pcf50606 *pcf = rdev_get_drvdata(rdev);
	int regulator_id;
	u8 regnr;

	regulator_id = rdev_get_id(rdev);
	if (regulator_id >= PCF50606_NUM_REGULATORS)
		return -EINVAL;

	/* IOREG cannot be powered off since it powers the PMU I2C */
	if (regulator_id == PCF50606_REGULATOR_IOREG)
		return -EINVAL;
	
	regnr = pcf50606_regulator_registers[regulator_id];

	return pcf50606_reg_set_bit_mask(pcf, regnr, 0xe0, 0);
}

static int pcf50606_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct pcf50606 *pcf = rdev_get_drvdata(rdev);
	int regulator_id = rdev_get_id(rdev);
	u8 regnr, val;

	regulator_id = rdev_get_id(rdev);
	if (regulator_id >= PCF50606_NUM_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	regnr = pcf50606_regulator_registers[regulator_id];
	val = (pcf50606_reg_read(pcf, regnr) & 0xe0) >> 5;

	/* PWREN1 = 1, PWREN2 = 1, see table 16 of datasheet */
	if (val == 0 || val == 5)
		return 0;

	return 1;
}

static struct regulator_ops pcf50606_regulator_ops = {
	.set_voltage = pcf50606_regulator_set_voltage,
	.get_voltage = pcf50606_regulator_get_voltage,
	.enable = pcf50606_regulator_enable,
	.disable = pcf50606_regulator_disable,
	.is_enabled = pcf50606_regulator_is_enabled,
};

static struct regulator_desc regulators[] = {
	[PCF50606_REGULATOR_DCD] =
		PCF50606_REGULATOR("dcd", PCF50606_REGULATOR_DCD),
	[PCF50606_REGULATOR_DCDE] =
		PCF50606_REGULATOR("dcde", PCF50606_REGULATOR_DCDE),
	[PCF50606_REGULATOR_DCUD] =
		PCF50606_REGULATOR("dcud", PCF50606_REGULATOR_DCUD),
	[PCF50606_REGULATOR_D1REG] =
		PCF50606_REGULATOR("d1reg", PCF50606_REGULATOR_D1REG),
	[PCF50606_REGULATOR_D2REG] =
		PCF50606_REGULATOR("d2reg", PCF50606_REGULATOR_D2REG),
	[PCF50606_REGULATOR_D3REG] =
		PCF50606_REGULATOR("d3reg", PCF50606_REGULATOR_D3REG),
	[PCF50606_REGULATOR_LPREG] =
		PCF50606_REGULATOR("lpreg", PCF50606_REGULATOR_LPREG),
	[PCF50606_REGULATOR_IOREG] =
		PCF50606_REGULATOR("ioreg", PCF50606_REGULATOR_IOREG),
};

static int __devinit pcf50606_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct pcf50606 *pcf;

	/* Already set by core driver */
	pcf = platform_get_drvdata(pdev);

	rdev = regulator_register(&regulators[pdev->id], &pdev->dev, pcf);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	if (pcf->pdata->regulator_registered)
		pcf->pdata->regulator_registered(pcf, pdev->id);

	return 0;
}

static int __devexit pcf50606_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

static struct platform_driver pcf50606_regulator_driver = {
	.driver = {
		.name = "pcf50606-regltr",
	},
	.probe = pcf50606_regulator_probe,
	.remove = __devexit_p(pcf50606_regulator_remove),
};

static int __init pcf50606_regulator_init(void)
{
	return platform_driver_register(&pcf50606_regulator_driver);
}
module_init(pcf50606_regulator_init);

static void __exit pcf50606_regulator_exit(void)
{
	platform_driver_unregister(&pcf50606_regulator_driver);
}
module_exit(pcf50606_regulator_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50606 regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50606-regulator");
