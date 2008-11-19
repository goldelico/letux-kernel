/*
 * Memory access timing control sysfs for the s3c24xx based device
 *
 * (C) 2008 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
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

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/arch/regs-mem.h>

static ssize_t neo1973_memconfig_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int index = attr->attr.name[strlen(attr->attr.name) - 1] - '0';
	u32 reg = *((u32 *)(S3C2410_MEMREG(((index + 1) << 2))));
	static const char *meaning[][8] = {
		{
			[0] = "normal (1 data)",
			[1] = "4 data",
			[2] = "8 data",
			[3] = "16 data",
		}, {
			[0] = "2 clocks",
			[1] = "3 clocks",
			[2] = "4 clocks",
			[3] = "6 clocks",
		}, {
			[0] = "0 clocks",
			[1] = "1 clock",
			[2] = "2 clocks",
			[3] = "4 clocks",
		}, {
			[0] = "1 clock",
			[1] = "2 clocks",
			[2] = "3 clocks",
			[3] = "4 clocks",
			[4] = "6 clocks",
			[5] = "8 clocks",
			[6] = "10 clocks",
			[7] = "14 clocks",
		}, { /* after this, only for CS6 and CS7 */
			[0] = "ROM / SRAM",
			[1] = "(illegal)",
			[2] = "(illegal)",
			[3] = "Sync DRAM",
		}, {
			[0] = "8 Column bits",
			[1] = "9 Column bits",
			[2] = "10 Column bits",
			[3] = "(illegal)",
		}, {
			[0] = "2 clocks",
			[1] = "3 clocks",
			[2] = "4 clocks",
			[3] = "(illegal)",
		}
	};

	if (index >= 6)
		if (((reg >> 15) & 3) == 3) /* DRAM */
			return sprintf(buf, "BANKCON%d = 0x%08X\n DRAM:\n"
					    "  Trcd = %s\n  SCAN = %s\n", index,
					    reg, meaning[5][reg & 3],
					    meaning[1][(reg >> 2) & 3]);

	return sprintf(buf, "BANKCON%d = 0x%08X\n Type = %s\n PMC  = %s\n"
			    " Tacp = %s\n Tcah = %s\n Tcoh = %s\n Tacc = %s\n"
			    " Tcos = %s\n Tacs = %s\n",
			    index, reg, meaning[4][(reg >> 15) & 3],
			    meaning[0][reg & 3],
			    meaning[1][(reg >> 2) & 3],
			    meaning[2][(reg >> 4) & 3],
			    meaning[2][(reg >> 6) & 3],
			    meaning[3][(reg >> 8) & 7],
			    meaning[2][(reg >> 11) & 3],
			    meaning[2][(reg >> 13) & 3]);
}

static ssize_t neo1973_memconfig_write(struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t count)
{
	int index = attr->attr.name[strlen(attr->attr.name) - 1] - '0';
	unsigned long val = simple_strtoul(buf, NULL, 16);

	dev_info(dev, "setting BANKCON%d <- 0x%08X\n", index, (u32)val);

	*((u32 *)(S3C2410_MEMREG(((index + 1) << 2)))) = (u32)val;

	return count;
}


static DEVICE_ATTR(BANKCON0, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON1, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON2, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON3, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON4, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON5, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON6, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);
static DEVICE_ATTR(BANKCON7, 0644, neo1973_memconfig_read,
						       neo1973_memconfig_write);

static struct attribute *neo1973_memconfig_sysfs_entries[] = {
	&dev_attr_BANKCON0.attr,
	&dev_attr_BANKCON1.attr,
	&dev_attr_BANKCON2.attr,
	&dev_attr_BANKCON3.attr,
	&dev_attr_BANKCON4.attr,
	&dev_attr_BANKCON5.attr,
	&dev_attr_BANKCON6.attr,
	&dev_attr_BANKCON7.attr,
	NULL
};

static struct attribute_group neo1973_memconfig_attr_group = {
	.name	= NULL,
	.attrs	= neo1973_memconfig_sysfs_entries,
};

static int __init neo1973_memconfig_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "starting\n");

	switch (machine_arch_type) {
#ifdef CONFIG_MACH_NEO1973_GTA01
	case MACH_TYPE_NEO1973_GTA01:
		return -EINVAL;
#endif /* CONFIG_MACH_NEO1973_GTA01 */
	default:
		break;
	}

	return sysfs_create_group(&pdev->dev.kobj,
						 &neo1973_memconfig_attr_group);
}

static int neo1973_memconfig_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &neo1973_memconfig_attr_group);
	return 0;
}

static struct platform_driver neo1973_memconfig_driver = {
	.probe		= neo1973_memconfig_probe,
	.remove		= neo1973_memconfig_remove,
	.driver		= {
		.name		= "neo1973-memconfig",
	},
};

static int __devinit neo1973_memconfig_init(void)
{
	return platform_driver_register(&neo1973_memconfig_driver);
}

static void neo1973_memconfig_exit(void)
{
	platform_driver_unregister(&neo1973_memconfig_driver);
}

module_init(neo1973_memconfig_init);
module_exit(neo1973_memconfig_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("neo1973 memconfig");

