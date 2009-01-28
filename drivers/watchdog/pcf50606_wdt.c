/* Philips PCF50606 Watchdog Timer Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50606 driver mainly by
 * Harald Welte, Matt Hsu, Andy Green and Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>

#include <linux/mfd/pcf50606/core.h>

static struct pcf50606 *pcf = NULL;
static unsigned long wdt_status;

#define WDT_IN_USE        0
#define WDT_OK_TO_CLOSE   1
#define WDT_REGION_INITED 2
#define WDT_DEVICE_INITED 3

static int allow_close;
#define CLOSE_STATE_NOT		0x0000
#define CLOSE_STATE_ALLOW	0x2342

#define PCF50606_REG_OOCC1 	0x08
#define PCF50606_REG_OOCS 	0x01

#define PCF50606_OOCS_WDTEXP 	0x80
#define PCF50606_OOCC1_WDTRST 	0x08

static void pcf50606_wdt_start(void)
{
	pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_OOCC1, PCF50606_OOCC1_WDTRST,
			 PCF50606_OOCC1_WDTRST);
}

static void pcf50606_wdt_stop(void)
{
	pcf50606_reg_clear_bits(pcf, PCF50606_REG_OOCS, PCF50606_OOCS_WDTEXP);
}

static void pcf50606_wdt_keepalive(void)
{
	pcf50606_wdt_start();
}

static int pcf50606_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	pcf50606_wdt_start();

	return nonseekable_open(inode, file);
}

static int pcf50606_wdt_release(struct inode *inode, struct file *file)
{
	if (allow_close == CLOSE_STATE_ALLOW)
		pcf50606_wdt_stop();
	else {
		printk(KERN_CRIT "Unexpected close, not stopping watchdog!\n");
		pcf50606_wdt_keepalive();
	}

	allow_close = CLOSE_STATE_NOT;
	clear_bit(WDT_IN_USE, &wdt_status);

	return 0;
}

static ssize_t pcf50606_wdt_write(struct file *file, const char __user *data,
				  size_t len, loff_t *ppos)
{
	if (len) {
		size_t i;

		for (i = 0; i != len; i++) {
			char c;
			if (get_user(c, data + i))
				return -EFAULT;
			if (c == 'V')
				allow_close = CLOSE_STATE_ALLOW;
		}
		pcf50606_wdt_keepalive();
	}

	return len;
}

static struct watchdog_info pcf50606_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE,
	.firmware_version = 0,
	.identity	= "PCF50606 Watchdog",
};

static int pcf50606_wdt_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &pcf50606_wdt_ident,
				    sizeof(pcf50606_wdt_ident)) ? -EFAULT : 0;
		break;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		pcf50606_wdt_keepalive();
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(8, p);
	default:
		return -ENOIOCTLCMD;
	}
}

static struct file_operations pcf50606_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= &pcf50606_wdt_write,
	.ioctl		= &pcf50606_wdt_ioctl,
	.open		= &pcf50606_wdt_open,
	.release	= &pcf50606_wdt_release,
};

static struct miscdevice pcf50606_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &pcf50606_wdt_fops,
};

static void pcf50606_wdt_irq(int irq, void *unused)
{
	pcf50606_reg_set_bit_mask(pcf, PCF50606_REG_OOCC1,
				 PCF50606_OOCC1_WDTRST,
				 PCF50606_OOCC1_WDTRST);
}

int __init pcf50606_wdt_probe(struct platform_device *pdev)
{
	struct pcf50606_subdev_pdata *pdata;
	int err;

	if (pcf) {
		dev_err(pcf->dev, "Only one instance of WDT supported\n");
		return -ENODEV;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data available\n");
		return -EINVAL;
	}

	pcf = pdata->pcf;

	err = misc_register(&pcf50606_wdt_miscdev);
	if (err) {
		dev_err(&pdev->dev, "cannot register miscdev on "
		       "minor=%d (%d)\n", WATCHDOG_MINOR, err);
		return err;
	}
	set_bit(WDT_DEVICE_INITED, &wdt_status);

	pcf50606_register_irq(pcf, PCF50606_IRQ_CHGWD10S, pcf50606_wdt_irq, NULL);

	return 0;
}

static int __devexit pcf50606_wdt_remove(struct platform_device *pdev)
{
	pcf50606_free_irq(pcf, PCF50606_IRQ_CHGWD10S);
	misc_deregister(&pcf50606_wdt_miscdev);
	pcf = NULL;

	return 0;
}

struct platform_driver pcf50606_wdt_driver = {
	.driver = {
		.name = "pcf50606-wdt",
	},
	.probe = pcf50606_wdt_probe,
	.remove = __devexit_p(pcf50606_wdt_remove),
};

static int __init pcf50606_wdt_init(void)
{
		return platform_driver_register(&pcf50606_wdt_driver);
}
module_init(pcf50606_wdt_init);

static void __exit pcf50606_wdt_exit(void)
{
		platform_driver_unregister(&pcf50606_wdt_driver);
}
module_exit(pcf50606_wdt_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50606 wdt driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50606-wdt");

