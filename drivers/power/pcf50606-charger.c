/* NXP PCF50606 Main Battery Charger Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50606 driver mainly by
 * Harald Welte, Andy Green and Werner Almesberger
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
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <linux/mfd/pcf50606/core.h>
#include <linux/mfd/pcf50606/mbc.h>

struct pcf50606_mbc {
	struct pcf50606 *pcf;

	int charger_online;
	struct power_supply charger;
};

void pcf50606_charge_fast(struct pcf50606 *pcf, int on)
{
	struct pcf50606_mbc *mbc = platform_get_drvdata(pcf->mbc_pdev);

	if (on) {
		pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_MBCC1,
				 PCF50606_MBCC1_AUTOFST,
				 PCF50606_MBCC1_AUTOFST);\
			mbc->charger_online = 1;
	} else {
		/* disable automatic fast-charge */
		pcf50606_reg_clear_bits(pcf, PCF50606_REG_MBCC1,
					PCF50606_MBCC1_AUTOFST);
		/* switch to idle mode to abort existing charge process */
		pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_MBCC1,
				PCF50606_MBCC1_CHGMOD_MASK,
				PCF50606_MBCC1_CHGMOD_IDLE);
			mbc->charger_online = 0;
	}
}
EXPORT_SYMBOL_GPL(pcf50606_charge_fast);

static ssize_t
show_chgmode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pcf50606_mbc *mbc = dev_get_drvdata(dev);

	u8 mbcc1 = pcf50606_reg_read(mbc->pcf, PCF50606_REG_MBCC1);
	u8 chgmod = (mbcc1 & PCF50606_MBCC1_CHGMOD_MASK);

	return sprintf(buf, "%d\n", chgmod);
}

static ssize_t set_chgmode(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct pcf50606_mbc *mbc = dev_get_drvdata(dev);
	u_int8_t mbcc1 = pcf50606_reg_read(mbc->pcf, PCF50606_REG_MBCC1);

	mbcc1 &= ~PCF50606_MBCC1_CHGMOD_MASK;

	if (!strcmp(buf, "qualification"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_QUAL;
	else if (!strcmp(buf, "pre"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_PRE;
	else if (!strcmp(buf, "trickle"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_TRICKLE;
	else if (!strcmp(buf, "fast_cccv"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_FAST_CCCV;
	/* We don't allow the other fast modes for security reasons */
	else if (!strcmp(buf, "idle"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_IDLE;
	else
		return -EINVAL;

	pcf50606_reg_write(mbc->pcf, PCF50606_REG_MBCC1, mbcc1);

	return count;
}

static DEVICE_ATTR(chgmode, S_IRUGO, show_chgmode, set_chgmode);


static struct attribute *pcf50606_mbc_sysfs_entries[] = {
	&dev_attr_chgmode.attr,
	NULL,
};

static struct attribute_group mbc_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf50606_mbc_sysfs_entries,
};

static void
pcf50606_mbc_irq_handler(int irq, void *data)
{
	struct pcf50606_mbc *mbc = data;

	power_supply_changed(&mbc->charger);

	if (mbc->pcf->pdata->mbc_event_callback)
		mbc->pcf->pdata->mbc_event_callback(mbc->pcf, irq);
}

static int charger_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pcf50606_mbc *mbc = container_of(psy, struct pcf50606_mbc, charger);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =  mbc->charger_online;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const u8 mbc_irq_handlers[] = {
	PCF50606_IRQ_CHGINS,
	PCF50606_IRQ_CHGRM,
	PCF50606_IRQ_CHGFOK,
	PCF50606_IRQ_CHGERR,
	PCF50606_IRQ_CHGFRDY,
	PCF50606_IRQ_CHGPROT,
};

static int __devinit pcf50606_mbc_probe(struct platform_device *pdev)
{
	struct pcf50606_mbc *mbc;
	struct pcf50606_subdev_pdata *pdata = pdev->dev.platform_data;
	int ret;
	int i;
	u8 oocs;

	mbc = kzalloc(sizeof(*mbc), GFP_KERNEL);
	if (!mbc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mbc);
	mbc->pcf = pdata->pcf;

	/* Set up IRQ handlers */
	for (i = 0; i < ARRAY_SIZE(mbc_irq_handlers); i++)
		pcf50606_register_irq(mbc->pcf, mbc_irq_handlers[i],
					pcf50606_mbc_irq_handler, mbc);

	mbc->charger.name		= "charger";
	mbc->charger.type		= POWER_SUPPLY_TYPE_MAINS;
	mbc->charger.properties		= power_props;
	mbc->charger.num_properties	= ARRAY_SIZE(power_props);
	mbc->charger.get_property	= &charger_get_property;
	mbc->charger.supplied_to	= mbc->pcf->pdata->batteries;
	mbc->charger.num_supplicants	= mbc->pcf->pdata->num_batteries;

	ret = power_supply_register(&pdev->dev, &mbc->charger);
	if (ret) {
		dev_err(mbc->pcf->dev, "failed to register charger\n");
		kfree(mbc);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &mbc_attr_group);
	if (ret)
		dev_err(mbc->pcf->dev, "failed to create sysfs entries\n");

	oocs = pcf50606_reg_read(mbc->pcf, PCF50606_REG_OOCS);
	if (oocs & PCF50606_OOCS_CHGOK)
		pcf50606_mbc_irq_handler(PCF50606_IRQ_CHGINS, mbc);

	return 0;
}

static int __devexit pcf50606_mbc_remove(struct platform_device *pdev)
{
	struct pcf50606_mbc *mbc = platform_get_drvdata(pdev);
	int i;

	/* Remove IRQ handlers */
	for (i = 0; i < ARRAY_SIZE(mbc_irq_handlers); i++)
		pcf50606_free_irq(mbc->pcf, mbc_irq_handlers[i]);

	power_supply_unregister(&mbc->charger);

	kfree(mbc);

	return 0;
}

static struct platform_driver pcf50606_mbc_driver = {
	.driver = {
		.name = "pcf50606-mbc",
	},
	.probe = pcf50606_mbc_probe,
	.remove = __devexit_p(pcf50606_mbc_remove),
};

static int __init pcf50606_mbc_init(void)
{
	return platform_driver_register(&pcf50606_mbc_driver);
}
module_init(pcf50606_mbc_init);

static void __exit pcf50606_mbc_exit(void)
{
	platform_driver_unregister(&pcf50606_mbc_driver);
}
module_exit(pcf50606_mbc_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50606 mbc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50606-mbc");
