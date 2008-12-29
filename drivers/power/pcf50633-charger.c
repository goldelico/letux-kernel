/* Philips PCF50633 Main Battery Charger Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50633 driver mainly by
 * Harald Welte, Andy Green and Werner Almesberger
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

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>

void pcf50633_mbc_usb_curlim_set(struct pcf50633 *pcf, int ma)
{
	int ret;
	u8 bits;
	int charging_start = 1;
	u8 mbcs2, chgmod;

	if (ma >= 1000)
		bits = PCF50633_MBCC7_USB_1000mA;
	else if (ma >= 500)
		bits = PCF50633_MBCC7_USB_500mA;
	else if (ma >= 100)
		bits = PCF50633_MBCC7_USB_100mA;
	else {
		bits = PCF50633_MBCC7_USB_SUSPEND;
		charging_start = 0;
	}

	ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC7,
					PCF50633_MBCC7_USB_MASK, bits);
	if (ret)
		dev_err(pcf->dev, "error setting usb curlim to %d mA\n", ma);
	else
		dev_info(pcf->dev, "usb curlim to %d mA\n", ma);

	mbcs2 = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
	chgmod = (mbcs2 & PCF50633_MBCS2_MBC_MASK);

	/* If chgmod == BATFULL, setting chgena has no effect.
	 * We need to set resume instead.
	 */
	if (chgmod != PCF50633_MBCS2_MBC_BAT_FULL)
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_CHGENA, PCF50633_MBCC1_CHGENA);
	else
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_RESUME, PCF50633_MBCC1_RESUME);

	pcf->mbc.usb_active = charging_start;
	power_supply_changed(&pcf->mbc.usb);
}
EXPORT_SYMBOL_GPL(pcf50633_mbc_usb_curlim_set);

static const char *chgmode_names[] = {
	[PCF50633_MBCS2_MBC_PLAY]		= "play-only",
	[PCF50633_MBCS2_MBC_USB_PRE]		= "pre",
	[PCF50633_MBCS2_MBC_ADP_PRE]		= "pre",
	[PCF50633_MBCS2_MBC_USB_PRE_WAIT]	= "pre-wait",
	[PCF50633_MBCS2_MBC_ADP_PRE_WAIT]	= "pre-wait",
	[PCF50633_MBCS2_MBC_USB_FAST]		= "fast",
	[PCF50633_MBCS2_MBC_ADP_FAST]		= "fast",
	[PCF50633_MBCS2_MBC_USB_FAST_WAIT]	= "fast-wait",
	[PCF50633_MBCS2_MBC_ADP_FAST_WAIT]	= "fast-wait",
	[PCF50633_MBCS2_MBC_BAT_FULL]	= "bat-full",
};

static ssize_t show_chgmode(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct pcf50633 *pcf = dev_get_drvdata(dev);

	u8 mbcs2 = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
	u8 chgmod = (mbcs2 & PCF50633_MBCS2_MBC_MASK);

	return sprintf(buf, "%s %d\n", chgmode_names[chgmod], chgmod);
}
static DEVICE_ATTR(chgmode, S_IRUGO | S_IWUSR, show_chgmode, NULL);

static ssize_t show_usblim(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct pcf50633 *pcf = dev_get_drvdata(dev);
	u8 usblim = pcf50633_reg_read(pcf, PCF50633_REG_MBCC7) &
						PCF50633_MBCC7_USB_MASK;
	unsigned int ma;

	if (usblim == PCF50633_MBCC7_USB_1000mA)
		ma = 1000;
	else if (usblim == PCF50633_MBCC7_USB_500mA)
		ma = 500;
	else if (usblim == PCF50633_MBCC7_USB_100mA)
		ma = 100;
	else
		ma = 0;

	return sprintf(buf, "%u\n", ma);
}
static DEVICE_ATTR(usb_curlim, S_IRUGO | S_IWUSR, show_usblim, NULL);

static ssize_t force_usb_limit_dangerous(struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t count)
{
	struct pcf50633 *pcf = dev_get_drvdata(dev);
	unsigned long ma;

	strict_strtoul(buf, 10, &ma);

	pcf50633_mbc_usb_curlim_set(pcf, ma);

	return count;
}

static DEVICE_ATTR(force_usb_limit_dangerous, 0600,
					       NULL, force_usb_limit_dangerous);

static struct attribute *mbc_sysfs_entries[] = {
	&dev_attr_chgmode.attr,
	&dev_attr_usb_curlim.attr,
	&dev_attr_force_usb_limit_dangerous.attr,
	NULL,
};

static struct attribute_group mbc_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= mbc_sysfs_entries,
};

static void pcf50633_mbc_irq_handler(struct pcf50633 *pcf, int irq, void *data)
{
	struct pcf50633_mbc *mbc;

	mbc = &pcf->mbc;

	/* USB */
	if (irq == PCF50633_IRQ_USBINS)
		mbc->usb_online = 1;
	else if (irq == PCF50633_IRQ_USBREM) {
		mbc->usb_online = 0;
		mbc->usb_active = 0;
		pcf50633_mbc_usb_curlim_set(pcf, 0);
	}

	/* Adapter */
	if (irq == PCF50633_IRQ_ADPINS) {
		pcf->mbc.adapter_online = 1;
		pcf->mbc.adapter_active = 1;
	} else if (irq == PCF50633_IRQ_ADPREM) {
		mbc->adapter_online = 0;
		mbc->adapter_active = 0;
	}

	if (irq == PCF50633_IRQ_BATFULL) {
		mbc->usb_active = 0;
		mbc->adapter_active = 0;
	} else if (irq == PCF50633_IRQ_USBLIMON)
		mbc->usb_active = 0;
	else if (irq == PCF50633_IRQ_USBLIMOFF)
		mbc->usb_active = 1;

	power_supply_changed(&mbc->ac);
	power_supply_changed(&mbc->usb);
	power_supply_changed(&mbc->adapter);

	if (pcf->pdata->mbc_event_callback)
		pcf->pdata->mbc_event_callback(pcf, irq);
}

static int adapter_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct pcf50633_mbc *mbc = container_of(psy, struct pcf50633_mbc, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =  mbc->adapter_online;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct pcf50633_mbc *mbc = container_of(psy, struct pcf50633_mbc, usb);
	struct pcf50633 *pcf = container_of(mbc, struct pcf50633, mbc);

	u8 usblim = pcf50633_reg_read(pcf, PCF50633_REG_MBCC7) &
						PCF50633_MBCC7_USB_MASK;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mbc->usb_online && (usblim == PCF50633_MBCC7_USB_500mA);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct pcf50633_mbc *mbc = container_of(psy, struct pcf50633_mbc, ac);
	struct pcf50633 *pcf = container_of(mbc, struct pcf50633, mbc);

	u8 usblim = pcf50633_reg_read(pcf, PCF50633_REG_MBCC7) &
						PCF50633_MBCC7_USB_MASK;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mbc->usb_online && (usblim == PCF50633_MBCC7_USB_1000mA);
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

int __init pcf50633_mbc_probe(struct platform_device *pdev)
{
	struct pcf50633 *pcf;
	struct pcf50633_mbc *mbc;
	int ret;

	pcf = platform_get_drvdata(pdev);
	mbc = &pcf->mbc;

	/* Set up IRQ handlers */
	pcf->irq_handler[PCF50633_IRQ_ADPINS].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_ADPREM].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_USBINS].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_USBREM].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_BATFULL].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_CHGHALT].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_THLIMON].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_THLIMOFF].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_USBLIMON].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_USBLIMOFF].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_LOWSYS].handler =
					pcf50633_mbc_irq_handler;
	pcf->irq_handler[PCF50633_IRQ_LOWBAT].handler =
					pcf50633_mbc_irq_handler;

	/* Create power supplies */

	mbc->adapter.name		= "adapter";
	mbc->adapter.type		= POWER_SUPPLY_TYPE_MAINS;
	mbc->adapter.properties		= power_props;
	mbc->adapter.num_properties	= ARRAY_SIZE(power_props);
	mbc->adapter.get_property	= &adapter_get_property;
	mbc->adapter.supplied_to	= pcf->pdata->batteries;
	mbc->adapter.num_supplicants	= pcf->pdata->num_batteries;

	mbc->usb.name			= "usb";
	mbc->usb.type			= POWER_SUPPLY_TYPE_USB;
	mbc->usb.properties		= power_props;
	mbc->usb.num_properties		= ARRAY_SIZE(power_props);
	mbc->usb.get_property		= usb_get_property;
	mbc->usb.supplied_to		= pcf->pdata->batteries;
	mbc->usb.num_supplicants	= pcf->pdata->num_batteries;

	mbc->ac.name			= "ac";
	mbc->ac.type			= POWER_SUPPLY_TYPE_MAINS;
	mbc->ac.properties		= power_props;
	mbc->ac.num_properties		= ARRAY_SIZE(power_props);
	mbc->ac.get_property		= ac_get_property;
	mbc->ac.supplied_to		= pcf->pdata->batteries;
	mbc->ac.num_supplicants		= pcf->pdata->num_batteries;

	ret = power_supply_register(&pdev->dev, &mbc->adapter);
	if (ret)
		dev_err(pcf->dev, "failed to register adapter\n");

	ret = power_supply_register(&pdev->dev, &mbc->usb);
	if (ret)
		dev_err(pcf->dev, "failed to register usb\n");

	ret = power_supply_register(&pdev->dev, &mbc->ac);
	if (ret)
		dev_err(pcf->dev, "failed to register ac\n");

	return sysfs_create_group(&pdev->dev.kobj, &mbc_attr_group);
}

static int __devexit pcf50633_mbc_remove(struct platform_device *pdev)
{
	struct pcf50633 *pcf;

	pcf = platform_get_drvdata(pdev);

	return 0;
}

struct platform_driver pcf50633_mbc_driver = {
	.driver = {
		.name = "pcf50633-mbc",
	},
	.probe = pcf50633_mbc_probe,
	.remove = __devexit_p(pcf50633_mbc_remove),
};

static int __init pcf50633_mbc_init(void)
{
		return platform_driver_register(&pcf50633_mbc_driver);
}
module_init(pcf50633_mbc_init);

static void __exit pcf50633_mbc_exit(void)
{
		platform_driver_unregister(&pcf50633_mbc_driver);
}
module_exit(pcf50633_mbc_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 mbc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-mbc");
