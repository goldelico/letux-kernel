/*
 * GSM Management code for the FIC Neo1973 GSM Phone
 *
 * (C) 2007 by Openmoko Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <mach/gta01.h>
#include <asm/plat-s3c24xx/neo1973.h>
#include <mach/s3c24xx-serial.h>

#include <mach/hardware.h>

/* For GTA02 */
#include <mach/gta02.h>
#include <linux/pcf50633.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

int gta_gsm_interrupts;
EXPORT_SYMBOL(gta_gsm_interrupts);

extern void s3c24xx_serial_console_set_silence(int);

struct gta01pm_priv {
	int gpio_ngsm_en;
        int gpio_ndl_gsm;

	struct console *con;
};

static struct gta01pm_priv gta01_gsm;

static struct console *find_s3c24xx_console(void)
{
	struct console *con;

	acquire_console_sem();

	for (con = console_drivers; con; con = con->next) {
		if (!strcmp(con->name, "ttySAC"))
			break;
	}

	release_console_sem();

	return con;
}

static ssize_t gsm_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if (!strcmp(attr->attr.name, "power_on")) {
		if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_ON))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "reset")) {
		if (machine_is_neo1973_gta01() && s3c2410_gpio_getpin(GTA01_GPIO_MODEM_RST))
			goto out_1;
		else if (machine_is_neo1973_gta02() && s3c2410_gpio_getpin(GTA02_GPIO_MODEM_RST))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "download")) {
		if (machine_is_neo1973_gta01()) {
			if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_DNLOAD))
				goto out_1;
		} else if (machine_is_neo1973_gta02()) {
			if (!s3c2410_gpio_getpin(GTA02_GPIO_nDL_GSM))
				goto out_1;
		}
	} else if (!strcmp(attr->attr.name, "flowcontrolled")) {
		if (s3c2410_gpio_getcfg(S3C2410_GPH1) == S3C2410_GPIO_OUTPUT)
			goto out_1;
	}

	return strlcpy(buf, "0\n", 3);
out_1:
	return strlcpy(buf, "1\n", 3);
}

static ssize_t gsm_write(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		if (on) {
			if (gta01_gsm.con) {
				dev_dbg(dev, "powering up GSM, thus "
					 "disconnecting serial console\n");

				console_stop(gta01_gsm.con);
				s3c24xx_serial_console_set_silence(1);
			}

			if (gta01_gsm.gpio_ngsm_en)
				s3c2410_gpio_setpin(gta01_gsm.gpio_ngsm_en, 0);

			if (machine_is_neo1973_gta02()) {
				switch (system_rev) {
				case GTA02v2_SYSTEM_REV:
				case GTA02v3_SYSTEM_REV:
				case GTA02v4_SYSTEM_REV:
				case GTA02v5_SYSTEM_REV:
				case GTA02v6_SYSTEM_REV:
					pcf50633_gpio_set(pcf50633_global,
							  PCF50633_GPIO2, 1);
					break;
				}
			}

			neo1973_gpb_setpin(GTA01_GPIO_MODEM_ON, 1);
		} else {
			neo1973_gpb_setpin(GTA01_GPIO_MODEM_ON, 0);

			if (machine_is_neo1973_gta02()) {
				switch (system_rev) {
				case GTA02v2_SYSTEM_REV:
				case GTA02v3_SYSTEM_REV:
				case GTA02v4_SYSTEM_REV:
				case GTA02v5_SYSTEM_REV:
				case GTA02v6_SYSTEM_REV:
					pcf50633_gpio_set(pcf50633_global,
							  PCF50633_GPIO2, 0);
					break;
				}
			}

			if (gta01_gsm.gpio_ngsm_en)
				s3c2410_gpio_setpin(gta01_gsm.gpio_ngsm_en, 1);

			if (gta01_gsm.con) {
				s3c24xx_serial_console_set_silence(0);
				console_start(gta01_gsm.con);

				dev_dbg(dev, "powered down GSM, thus enabling "
					 "serial console\n");
			}
		}
	} else if (!strcmp(attr->attr.name, "reset")) {
		if (machine_is_neo1973_gta01())
			neo1973_gpb_setpin(GTA01_GPIO_MODEM_RST, on);
		else if (machine_is_neo1973_gta02())
			neo1973_gpb_setpin(GTA02_GPIO_MODEM_RST, on);
	} else if (!strcmp(attr->attr.name, "download")) {
		if (machine_is_neo1973_gta01())
			s3c2410_gpio_setpin(GTA01_GPIO_MODEM_DNLOAD, on);

		if (machine_is_neo1973_gta02()) {
			/*
			 * the keyboard / buttons driver requests and enables
			 * the JACK_INSERT IRQ.  We have to take care about
			 * not enabling and disabling the IRQ when it was
			 * already in that state or we get "unblanaced IRQ"
			 * kernel warnings and stack dumps.  So we use the
			 * copy of the ndl_gsm state to figure out if we should
			 * enable or disable the jack interrupt
			 */
			if (on) {
				if (gta01_gsm.gpio_ndl_gsm)
					disable_irq(gpio_to_irq(
						       GTA02_GPIO_JACK_INSERT));
			} else {
				if (!gta01_gsm.gpio_ndl_gsm)
					enable_irq(gpio_to_irq(
						       GTA02_GPIO_JACK_INSERT));
			}

			gta01_gsm.gpio_ndl_gsm = !on;
			s3c2410_gpio_setpin(GTA02_GPIO_nDL_GSM, !on);
		}
	} else if (!strcmp(attr->attr.name, "flowcontrolled")) {
		if (on) {
			gta_gsm_interrupts = 0;
			s3c2410_gpio_setpin(S3C2410_GPH1, 1);
			s3c2410_gpio_cfgpin(S3C2410_GPH1, S3C2410_GPH1_OUTP);
		} else
			s3c2410_gpio_cfgpin(S3C2410_GPH1, S3C2410_GPH1_nRTS0);
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(reset, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(download, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(flowcontrolled, 0644, gsm_read, gsm_write);

#ifdef CONFIG_PM

static int gta01_gsm_resume(struct platform_device *pdev);
static int gta01_gsm_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do much here. */

	/* If flowcontrol asserted, abort if GSM already interrupted */
	if (s3c2410_gpio_getcfg(S3C2410_GPH1) == S3C2410_GPIO_OUTPUT) {
		if (gta_gsm_interrupts)
			goto busy;
	}

	/* disable DL GSM to prevent jack_insert becoming 'floating' */
	if (machine_is_neo1973_gta02())
		s3c2410_gpio_setpin(GTA02_GPIO_nDL_GSM, 1);
	return 0;

busy:
	return -EBUSY;
}

static int
gta01_gsm_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	/* Last chance: abort if GSM already interrupted */
	if (s3c2410_gpio_getcfg(S3C2410_GPH1) == S3C2410_GPIO_OUTPUT) {
		if (gta_gsm_interrupts)
			return -EBUSY;
	}
	return 0;
}

static int gta01_gsm_resume(struct platform_device *pdev)
{
	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do much here. */

	/* Make sure that the kernel console on the serial port is still
	 * disabled. FIXME: resume ordering race with serial driver! */
	if (gta01_gsm.con && s3c2410_gpio_getpin(GTA01_GPIO_MODEM_ON))
		console_stop(gta01_gsm.con);

	if (machine_is_neo1973_gta02())
		s3c2410_gpio_setpin(GTA02_GPIO_nDL_GSM, gta01_gsm.gpio_ndl_gsm);

	return 0;
}
#else
#define gta01_gsm_suspend	NULL
#define gta01_gsm_suspend_late	NULL
#define gta01_gsm_resume	NULL
#endif /* CONFIG_PM */

static struct attribute *gta01_gsm_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
	&dev_attr_download.attr,
	&dev_attr_flowcontrolled.attr,
	NULL
};

static struct attribute_group gta01_gsm_attr_group = {
	.name	= NULL,
	.attrs	= gta01_gsm_sysfs_entries,
};

static int __init gta01_gsm_probe(struct platform_device *pdev)
{
	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = GTA01v3_GPIO_nGSM_EN;
		break;
	case GTA01v4_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = 0;
		break;
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = GTA01Bv2_GPIO_nGSM_EN;
		s3c2410_gpio_setpin(GTA01v3_GPIO_nGSM_EN, 0);
		break;
	case GTA02v1_SYSTEM_REV:
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = 0;
		break;
	default:
		dev_warn(&pdev->dev, "Unknown Neo1973 Revision 0x%x, "
			 "some PM features not available!!!\n",
			 system_rev);
		break;
	}

	switch (system_rev) {
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		gta01_gsm_sysfs_entries[ARRAY_SIZE(gta01_gsm_sysfs_entries)-2] =
							&dev_attr_download.attr;
		break;
	default:
		break;
	}

	if (machine_is_neo1973_gta01()) {
		gta01_gsm.con = find_s3c24xx_console();
		if (!gta01_gsm.con)
			dev_warn(&pdev->dev,
				 "cannot find S3C24xx console driver\n");
	} else
		gta01_gsm.con = NULL;

	/* note that download initially disabled, and enforce that */
	gta01_gsm.gpio_ndl_gsm = 1;
	if (machine_is_neo1973_gta02())
		s3c2410_gpio_setpin(GTA02_GPIO_nDL_GSM, 1);

	return sysfs_create_group(&pdev->dev.kobj, &gta01_gsm_attr_group);
}

static int gta01_gsm_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gta01_gsm_attr_group);

	return 0;
}

static struct platform_driver gta01_gsm_driver = {
	.probe		= gta01_gsm_probe,
	.remove		= gta01_gsm_remove,
	.suspend	= gta01_gsm_suspend,
	.suspend_late	= gta01_gsm_suspend_late,
	.resume		= gta01_gsm_resume,
	.driver		= {
		.name		= "neo1973-pm-gsm",
	},
};

static int __devinit gta01_gsm_init(void)
{
	return platform_driver_register(&gta01_gsm_driver);
}

static void gta01_gsm_exit(void)
{
	platform_driver_unregister(&gta01_gsm_driver);
}

module_init(gta01_gsm_init);
module_exit(gta01_gsm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("FIC Neo1973 GSM Power Management");
