/*
 * HDQ driver for the FIC Neo1973 GTA02 GSM phone
 *
 * (C) 2006-2007 by OpenMoko, Inc.
 * Author: Andy Green <andy@openmoko.com>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/gta02.h>
#include <asm/arch/fiq_ipc_gta02.h>



#define HDQ_READ 0
#define HDQ_WRITE 0x80


int gta02hdq_initialized(void)
{
	return fiq_ipc.hdq_probed;
}
EXPORT_SYMBOL_GPL(gta02hdq_initialized);

int gta02hdq_read(int address)
{
	int count_sleeps = 5;
	int ret = -ETIME;

	mutex_lock(&fiq_ipc.hdq_lock);

	fiq_ipc.hdq_ads = address | HDQ_READ;
	fiq_ipc.hdq_request_ctr++;
	fiq_kick();
	/*
	 * FIQ takes care of it while we block our calling process
	 * But we're not spinning -- other processes run normally while
	 * we wait for the result
	 */
	while (count_sleeps--) {
		msleep(10); /* valid transaction always completes in < 10ms */

		if (fiq_ipc.hdq_request_ctr != fiq_ipc.hdq_transaction_ctr)
			continue;

		if (fiq_ipc.hdq_error)
			goto done; /* didn't see a response in good time */

		ret = fiq_ipc.hdq_rx_data;
		goto done;
	}
	ret = -EINVAL;

done:
	mutex_unlock(&fiq_ipc.hdq_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gta02hdq_read);

int gta02hdq_write(int address, u8 data)
{
	int count_sleeps = 5;
	int ret = -ETIME;

	mutex_lock(&fiq_ipc.hdq_lock);

	fiq_ipc.hdq_ads = address | HDQ_WRITE;
	fiq_ipc.hdq_tx_data = data;
	fiq_ipc.hdq_request_ctr++;
	fiq_kick();
	/*
	 * FIQ takes care of it while we block our calling process
	 * But we're not spinning -- other processes run normally while
	 * we wait for the result
	 */
	while (count_sleeps--) {
		msleep(10); /* valid transaction always completes in < 10ms */

		if (fiq_ipc.hdq_request_ctr != fiq_ipc.hdq_transaction_ctr)
			continue; /* something bad with FIQ */

		if (fiq_ipc.hdq_error)
			goto done; /* didn't see a response in good time */
	}
	ret = -EINVAL;

done:
	mutex_unlock(&fiq_ipc.hdq_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gta02hdq_write);

/* sysfs */

static ssize_t hdq_sysfs_dump(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int n;
	int v;
	u8 u8a[128]; /* whole address space for HDQ */
	char *end = buf;

	/* the dump does not take care about 16 bit regs, because at this
	 * bus level we don't know about the chip details
	 */
	for (n = 0; n < sizeof(u8a); n++) {
		v = gta02hdq_read(n);
		if (v < 0)
			goto bail;
		u8a[n] = v;
	}

	for (n = 0; n < sizeof(u8a); n += 16) {
		hex_dump_to_buffer(u8a + n, sizeof(u8a), 16, 1, end, 4096, 0);
		end += strlen(end);
		*end++ = '\n';
		*end = '\0';
	}
	return (end - buf);

bail:
	return sprintf(buf, "ERROR %d\n", v);
}

/* you write by <address> <data>, eg, "34 128" */

#define atoi(str) simple_strtoul(((str != NULL) ? str : ""), NULL, 0)

static ssize_t hdq_sysfs_write(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	const char *end = buf + count;
	int address = atoi(buf);

	while ((buf != end) && (*buf != ' '))
		buf++;
	if (buf >= end)
		return 0;
	while ((buf < end) && (*buf == ' '))
		buf++;
	if (buf >= end)
		return 0;

	gta02hdq_write(address, atoi(buf));

	return count;
}

static DEVICE_ATTR(dump, 0400, hdq_sysfs_dump, NULL);
static DEVICE_ATTR(write, 0600, NULL, hdq_sysfs_write);

static struct attribute *gta02hdq_sysfs_entries[] = {
	&dev_attr_dump.attr,
	&dev_attr_write.attr,
	NULL
};

static struct attribute_group gta02hdq_attr_group = {
	.name	= "hdq",
	.attrs	= gta02hdq_sysfs_entries,
};


#ifdef CONFIG_PM
static int gta02hdq_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* after 18s of this, the battery monitor will also go to sleep */
	s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 0);
	s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_OUTPUT);
	return 0;
}

static int gta02hdq_resume(struct platform_device *pdev)
{
	s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 1);
	s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_OUTPUT);
	return 0;
}
#endif

static int __init gta02hdq_probe(struct platform_device *pdev)
{
	struct resource *r = platform_get_resource(pdev, 0, 0);

	if (!machine_is_neo1973_gta02())
		return -EIO;

	if (!r)
		return -EINVAL;

	platform_set_drvdata(pdev, NULL);

	mutex_init(&fiq_ipc.hdq_lock);

	/* set our HDQ comms pin from the platform data */
	fiq_ipc.hdq_gpio_pin = r->start;

	s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 1);
	s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_OUTPUT);

	fiq_ipc.hdq_probed = 1; /* we are ready to do stuff now */

	return sysfs_create_group(&pdev->dev.kobj, &gta02hdq_attr_group);
}

static int gta02hdq_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gta02hdq_attr_group);
	return 0;
}

static struct platform_driver gta02hdq_driver = {
	.probe		= gta02hdq_probe,
	.remove		= gta02hdq_remove,
#ifdef CONFIG_PM
	.suspend	= gta02hdq_suspend,
	.resume		= gta02hdq_resume,
#endif
	.driver		= {
		.name		= "gta02-hdq",
	},
};

static int __init gta02hdq_init(void)
{
	return platform_driver_register(&gta02hdq_driver);
}

static void __exit gta02hdq_exit(void)
{
 	platform_driver_unregister(&gta02hdq_driver);
}

module_init(gta02hdq_init);
module_exit(gta02hdq_exit);

MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("GTA02 HDQ driver");
MODULE_LICENSE("GPL");
