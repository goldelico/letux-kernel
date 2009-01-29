/*
 *
 * /sys/kernel/iblock/
 * All times are in microseconds (us).
 *
 * - limit
 *   Interrupt blocking time reporting limit, in microseconds.
 *   0 disables reporting. Auto-resets to zero after reporting.
 *
 * - max
 *   Maximum blocking time recorded. Reset to zero by writing anything.
 *
 * - test
 *   Force a delay with interrupts disabled.
 *
 */


#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/irqflags.h>


unsigned long s3c2410_gettimeoffset(void);


static unsigned long iblock_t0;
static int iblock_limit;
static int iblock_max;


void iblock_start(void)
{
	unsigned long flags;

	raw_local_save_flags(flags);
	if (raw_irqs_disabled_flags(flags))
		return;
	iblock_t0 = s3c2410_gettimeoffset();
}
EXPORT_SYMBOL_GPL(iblock_start);

void iblock_end(void)
{
	unsigned long flags;
	unsigned long t, us;

	raw_local_save_flags(flags);
	if (!raw_irqs_disabled_flags(flags))
		return;
	if (!iblock_t0)
		return;
	t = s3c2410_gettimeoffset();
	us = t-iblock_t0;
	if (us > 40000000)
		return;
	if (us > iblock_max)
		iblock_max = us;
	if (!iblock_limit)
		return;
	if (us < iblock_limit)
		return;
	iblock_limit = 0;
	printk(KERN_ERR "interrupts were disabled for %lu us !\n", us);
	WARN_ON(1);
}
EXPORT_SYMBOL_GPL(iblock_end);

void iblock_end_maybe(unsigned long flags)
{
	if (raw_irqs_disabled_flags(flags))
		return;
	iblock_end();
}
EXPORT_SYMBOL_GPL(iblock_end_maybe);

static ssize_t limit_read(struct device *dev, struct device_attribute *attr,
    char *buf)
{
	return sprintf(buf, "%u us\n", iblock_limit);
}


static ssize_t limit_write(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count)
{
	unsigned long tmp;
	char *end;

	tmp = simple_strtoul(buf, &end, 0);
	if (end == buf)
		return -EINVAL;
	iblock_limit = tmp;
	return count;
}


static ssize_t max_read(struct device *dev, struct device_attribute *attr,
    char *buf)
{
	return sprintf(buf, "%u us\n", iblock_max);
}


static ssize_t max_write(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count)
{
	iblock_max = 0;
	return count;
}


static ssize_t test_write(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp, flags;
	char *end;

	tmp = simple_strtoul(buf, &end, 0);
	if (end == buf)
		return -EINVAL;
	local_irq_save(flags);
	udelay(tmp);
	local_irq_restore(flags);
	return count;
}


static DEVICE_ATTR(limit, 0644, limit_read, limit_write);
static DEVICE_ATTR(max, 0644, max_read, max_write);
static DEVICE_ATTR(test, 0200, NULL, test_write);


static struct attribute *sysfs_entries[] = {
	&dev_attr_limit.attr,
	&dev_attr_max.attr,
	&dev_attr_test.attr,
	NULL
};


static struct attribute_group attr_group = {
	.name	= "iblock",
	.attrs	= sysfs_entries,
};


static int __devinit iblock_init(void)
{
	return sysfs_create_group(kernel_kobj, &attr_group);
}


module_init(iblock_init);
