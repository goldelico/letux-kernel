// SPDX-License-Identifier: GPL-2.0

/*
 * driver for Retrode 3 game cartridge driver
 *
 * based on ideas and fragents from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c
 *
 *  Copyright (C) 2022-23, H. Nikolaus Schaller
 *
 * FIXME: make this an independent driver module
 */

#if FIXME
#include "../gpio/gpiolib.h"
#include "retrode3_bus.h"

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif
#endif

// #define RETRODE3_MAJOR FLOPPY_MAJOR
#define RETRODE3_MINORS	16
static DEFINE_IDA(retrode3_minors);
static dev_t retrode3_first;
static struct class *retrode3_class;

/*
 * game cart driver
 */

/*
 * This function reads the slot. The ppos points directly to the
 * memory location.
 */

static ssize_t retrode3_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct retrode3_slot *slot = file->private_data;
	int err;
#ifndef CONFIG_RETRODE3_BUFFER
	ssize_t read, sz;
#else
	ssize_t read;
	unsigned long remaining;
	unsigned char buffer[1024];
	unsigned int fill = 0;
#endif

//	dev_info(&slot->dev, "%s\n", __func__);

	read = 0;

	select_slot(slot->bus, slot);

	if (*ppos + count >= EOF) {
		if (*ppos >= EOF)
			count = 0;	// read nothing
		else
			count = EOF - *ppos;	// limit to EOF
	}

	while (count > 0) {
#ifndef CONFIG_RETRODE3_BUFFER
		unsigned long remaining;
#endif

		if (count >= 2 && slot->bus_width == 16 && ((*ppos & 1) == 0)) {
			u16 word;

			err = set_address(slot->bus, *ppos);
			if(err < 0)
				goto failed;

			err = read_word(slot->bus);
			if(err < 0)
				goto failed;

#ifndef CONFIG_RETRODE3_BUFFER
			sz = 2;	// handle word reads
			word = swab16(err);	// use htons()?

			remaining = copy_to_user(buf, (char *) &word, sz);
#else

			word = err;

			if (fill == sizeof(buffer)) { // flush buffer
				remaining = copy_to_user(buf+read, buffer, fill);
				if (remaining)
					goto failed;
				fill = 0;
			}
			*((u16 *) &buffer[fill]) = swab16(word);	// use htons()?
			fill += 2;
			*ppos += 2;
			count -= 2;
			read += 2;
#endif
		} else {
			u8 byte;

			if (slot->bus_width == 16) {
				err = set_address(slot->bus, *ppos);	// A0 determines lower/upper byte
				// FIXME: should we read a word and take either half based on A0?
				// we can get rid of read_byte()
				// but it is slower!
				byte = err = read_byte(slot->bus);
			}
			else { // 8 bit bus
				err = set_address(slot->bus, *ppos);	// includes setting physical A0
				// FIXME: should we read a word and take D0..D7 only?
				// we can get rid of read_half()
				// but it is slower!
				byte = err = read_half(slot->bus, 1);	// D0..D7
			}
			if(err < 0)
				goto failed;

#ifndef CONFIG_RETRODE3_BUFFER
			sz = 1;	// handle byte read

			remaining = copy_to_user(buf, (char *) &byte, sz);
#else

			if (fill == sizeof(buffer)) { // flush buffer
				remaining = copy_to_user(buf+read, buffer, fill);
				if (remaining)
					goto failed;
				fill = 0;
			}
			*((u8 *) &buffer[fill]) = byte;
			fill += 1;
			*ppos += 1;
			count -= 1;
			read += 1;
#endif
		}

#ifndef CONFIG_RETRODE3_BUFFER
                if (remaining)
                        goto failed;

		*ppos += sz;
		buf += sz;
		count -= sz;
		read += sz;
#endif
	}

#ifdef CONFIG_RETRODE3_BUFFER
#warning buffered
	if (fill > 0) { // flush remaining bytes
		remaining = copy_to_user(buf, buffer, fill);
		if (remaining)
			goto failed;
	}
#endif

	select_slot(slot->bus, NULL);

	return read;

failed:
	select_slot(slot->bus, NULL);

	return err;
}

static ssize_t retrode3_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct retrode3_slot *slot = file->private_data;
	ssize_t written, sz;
	unsigned long copied;
	void *ptr;

//	dev_info(&slot->dev, "%s\n", __func__);

	written = 0;

	select_slot(slot->bus, slot);

	if (*ppos + count >= EOF) {
		if (*ppos >= EOF)
			count = 0;	// write nothing
		else
			count = EOF - *ppos;	// limit to EOF
	}

	while (count > 0) {
		int allowed;

// FIXME: loop over bytes and write them to consecutive addresses
// handle words for 16 bit bus and faster write

		sz = 1;
		set_address(slot->bus, *ppos);

#if FIXME	// ptr is uninitialized here
		copied = copy_from_user(ptr, buf, sz);
#endif
		if (copied) {
			written += sz - copied;
			if (written)
				break;
			return -EFAULT;
		}

		*ppos += sz;
		buf += sz;
		count -= sz;
		written += sz;
	}

	select_slot(slot->bus, NULL);

	return written;
}

/*
 * The memory devices use the full 32/64 bits of the offset, and so we cannot
 * check against negative addresses: they are ok. The return value is weird,
 * though, in that case (0).
 *
 * also note that seeking relative to the "end of file" isn't supported:
 * it has no meaning, so it returns -EINVAL.
 */
static loff_t retrode3_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;

	inode_lock(file_inode(file));
	switch (orig) {
	case SEEK_CUR:
		offset += file->f_pos;
		fallthrough;
	case SEEK_SET:
		/* to avoid userland mistaking f_pos=-9 as -EBADF=-9 */
		if ((unsigned long long)offset >= -MAX_ERRNO) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		force_successful_syscall_return();
		break;
	default:
		ret = -EINVAL;
	}
	inode_unlock(file_inode(file));
	return ret;
}

static int retrode3_open(struct inode *inode, struct file *file)
{
	struct retrode3_slot *slot;
	int ret = 0;

	slot = container_of(inode->i_cdev, struct retrode3_slot, cdev);

        dev_dbg(&slot->dev, "%s\n", __func__);

	ret = gpiod_get_value(slot->cd);	// check cart detect

	if (!ret)
		return -ENODEV;	// no cart is inserted

	get_device(&slot->dev);	// increment the reference count for the device

	ret = generic_file_open(inode, file);
	if (ret < 0)
		return ret;

	file->private_data = slot;

	return ret;
}

static int retrode3_release(struct inode *inode, struct file *file)
{
	struct retrode3_slot *slot = file->private_data;

        dev_dbg(&slot->dev, "%s\n", __func__);

	put_device(&slot->dev);

	return 0;
}

static const struct file_operations slot_fops = {
	.llseek		= retrode3_lseek,
	.read		= retrode3_read,
	.write		= retrode3_write,
	.open		= retrode3_open,
	.release	= retrode3_release,
};

// FIXME: duplicate to retrode3_release??

static void retrode3_slot_release(struct device *dev)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

        dev_dbg(&slot->dev, "%s\n", __func__);

	ida_simple_remove(&retrode3_minors, slot->id);
	// FIXME: what else to release
	kfree(slot);
}

int retrode3_probe_slot(struct retrode3_slot *slot, struct device_node *child)
{
	struct device *	dev = &slot->dev;
	int id;
	int ret;

	id = ida_simple_get(&retrode3_minors, 0, RETRODE3_MINORS, GFP_KERNEL);
	if (id < 0)
		return id;

	slot->id = id;

	device_initialize(dev);
	dev->devt = retrode3_first + id;
	dev->class = retrode3_class;
	dev->release = retrode3_slot_release;
	dev_set_drvdata(dev, slot);
	dev_set_name(dev, "slot%d", id);
	dev->of_node = child;

	slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
	gpiod_set_value(slot->ce, 0);	// turn inactive
	slot->cd = devm_gpiod_get(dev, "cd", GPIOD_IN);
	slot->led = devm_gpiod_get(dev, "status", GPIOD_OUT_HIGH);
	if (!IS_ERR_OR_NULL(slot->led))
		gpiod_set_value(slot->led, 0);	// turn inactive
	slot->power = devm_gpiod_get(dev, "power", GPIOD_IN);
	of_property_read_u32_index(child, "address-width", 0, &slot->addr_width);
	of_property_read_u32_index(child, "bus-width", 0, &slot->bus_width);

	cdev_init(&slot->cdev, &slot_fops);
	slot->cdev.owner = THIS_MODULE;

// FIXME: should be some devm_cdev_device_add
	ret = cdev_device_add(&slot->cdev, &slot->dev);
	if (ret)
		return ret;

	if(!IS_ERR_OR_NULL(slot->power)) {
//		set_slot_power(slot, 5000);	// switch to 5V if possible
		set_slot_power(slot, 3300);	// switch to 3.3V if possible
	}

	slot->cd_state = -1;	// enforce a state update event for initial state
#if 1	// use polling
	INIT_DELAYED_WORK(&slot->work, retrode3_cd_work);
	schedule_delayed_work(&slot->work,
			msecs_to_jiffies(50));	// start first check

#else	// use interrupt (untested)
	ret = devm_request_threaded_irq(dev, gpiod_to_irq(slot->cd),
		NULL, retrode3_gpio_cd_irqt,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"cart-detect", slot);
#endif

	return 0;
}

/*
 * cart detect and /sysfs
 */

static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf);

static void retrode3_update_cd(struct retrode3_slot *slot)
{
	int cd_state = gpiod_get_value(slot->cd);	// check cart detect pin

	if (cd_state < 0)
		return;	// ignore

	if (cd_state != slot-> cd_state) {
		char *envp[3];
		char buf[20];
		int len;

// printk("%s: state changed to %d\n", __func__, cd_state);

		slot-> cd_state = cd_state;

		envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
		len = sense_show(&slot->dev, NULL, buf);
		if (len < 0)
			return;
		if (buf[len - 1] == '\n')
			buf[len - 1] = 0;	// strip off \n
		envp[1] = kasprintf(GFP_KERNEL, "SENSE=%s", buf);
		envp[2] = NULL;
// printk("%s: %s %s\n", __func__, envp[0], envp[1]);
		// check with: udevadm monitor --environment
		kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
	}
}

static irqreturn_t retrode3_gpio_cd_irqt(int irq, void *dev_id)
{
	struct retrode3_slot *slot = dev_id;

	retrode3_update_cd(slot);

	return IRQ_HANDLED;
}

static void retrode3_cd_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);

	retrode3_update_cd(slot);

	schedule_delayed_work(&slot->work,
			msecs_to_jiffies(50));	// start next check
}

#if FIXME

static const struct retrode3_device_id retrode3_slot_idtable[] = {
	{ "retrode3-slot", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, retrode3_slot_idtable);

static const struct of_device_id retrode3_slot_of_match[] = {
	{ .compatible = "openpandora,retrode3-slot" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, retrode3_slot_of_match);

static struct retrode3_driver retrode3_slot_driver = {
	.driver = {
		.name	= "retrode3-controller",
		.of_match_table = retrode3_slot_of_match,
	},
	.id_table	= retrode3_slot_idtable,
	.probe		= retrode3_probe_slot,
};

module_retrode3_driver(retrode3_slot_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Retrode3 Game Cartridge Slot Driver");
MODULE_LICENSE("GPL");

#endif
