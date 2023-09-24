// SPDX-License-Identifier: GPL-2.0

/*
 * based on ideas and fragents from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c

to be solved
- 8 bit vs. 16 bit bus width
- add read/write for RAM
- completion of slot4/gamecontrols driver
- handle status LEDs
- handle megadrive power

 *
 *  Copyright (C) 2022-23, H. Nikolaus Schaller
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

#include "../gpio/gpiolib.h"

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif

// #define RETRODE3_MAJOR FLOPPY_MAJOR
#define RETRODE3_MINORS	16
static DEFINE_IDA(retrode3_minors);
static dev_t retrode3_first;
static struct class *retrode3_class;

/* one slot */

struct retrode3_controller {
	struct input_dev *input;
	u32 data_offset;	// 0 or 8
	int last_state;
	int state_valid;
};

struct retrode3_slot {
	struct device dev;	// the /dev/slot
	struct cdev cdev;	// the /dev/slot character device
	int id;
	struct retrode3_bus *bus;	// backpointer to bus
	struct gpio_desc *ce;	// slot enable
	struct gpio_desc *cd;	// slot detect
	struct gpio_desc *led;	// status led (optional)
	struct gpio_desc *power;	// power control (optional)
	struct delayed_work work;
	int cd_state;
	u32 addr_width;
	u32 bus_width;
	struct retrode3_controller controllers[2];
};

/* bus for all slots */

struct retrode3_bus {
	struct gpio_descs *addrs;	// addr-gpios
	struct gpio_descs *datas;	// data-gpios
	struct gpio_descs *we;		// we-gpios
	struct gpio_desc *oe;		// oe-gpio
	struct gpio_desc *time;		// time-gpio
	struct gpio_desc *reset;	// reset-gpio
	struct retrode3_slot *slots[4];
	struct mutex select_lock;
	int a0;
	u32 prev_addr;
};

#define EOF	(1L<<24)	// 24 address lines = 16 MByte

/* low level address and data bus access */

// FXIME: get_multiple / set_multiple could be much faster, if supported by device driver

static inline int get_bus_bit(struct gpio_desc *desc)
{
#if 1
	struct gpio_chip *gc = desc->gdev->chip;
	return gc->get(gc, gpio_chip_hwgpio(desc));
#else
	return gpiod_get_value(desc);
#endif

}

static inline void set_bus_bit(struct gpio_desc *desc, int value)
{
#if 1
	struct gpio_chip *gc = desc->gdev->chip;
	gc->set(gc, gpio_chip_hwgpio(desc), value);
#else
	gpiod_set_value(desc, value);
#endif

}

/* access to cart bus */

static inline int set_address(struct retrode3_bus *bus, u32 addr)
{ /* set address on all gpios */
	int a;

	if (addr >= EOF)
		return -EINVAL;

// printk("%s:\n", __func__);
	bus->a0 = addr & 1;	// save for 16 bit bus access

// NOTE: this will not use the A0 gpio

	for (a = 1; a < bus->addrs->ndescs; a++) {
		if ((addr ^ bus->prev_addr) & (1 << a))	{ // address bit has really changed
// printk("%s: %d -> %d\n", __func__, a, (addr >> a) & 1);
			set_bus_bit(bus->addrs->desc[a], (addr >> a) & 1);
		}
	}

	bus->prev_addr = addr;
	return 0;
}

// FIXME switch direction of D lines on demand?

static inline int read_byte(struct retrode3_bus *bus)
{ /* read data from data lines */
	int d;
	u8 data;
// printk("%s:\n", __func__);

	set_bus_bit(bus->oe, true);

	/* read data bits either on D0..D7 or D8..D15 */
	data = 0;
	if(bus->a0 == 0)
		for (d = 0; d < bus->datas->ndescs-8; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
			if (bit < 0)
				return bit;
			data |= bit << d;
		}
	else
		for (d = 8; d < bus->datas->ndescs; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
			if (bit < 0)
				return bit;
			data |= bit << (d-8);
		}
	set_bus_bit(bus->oe, false);
	return data;
}

static inline int read_word(struct retrode3_bus *bus)
{ /* read data from data lines */
	int d;
	u16 data;

// printk("%s:\n", __func__);

	set_bus_bit(bus->oe, true);
	/* read data bits */
	data = 0;
	for (d = 0; d < bus->datas->ndescs; d++) {
		int bit = get_bus_bit(bus->datas->desc[d]);

// printk("bit=%d\n", bit);

		if (bit < 0)
			return bit;
		data |= bit << d;
	}
	set_bus_bit(bus->oe, false);
	return data;
}

static inline void write_word(struct retrode3_bus *bus, u16 data, int mode)
{ /* write data to data lines */
	int d;
	/* set data bits */

// printk("%s:\n", __func__);

	data = 0;
	for (d = 0; d < bus->datas->ndescs; d++) {
		gpiod_direction_output(bus->datas->desc[d], (data>>d) & 1);
	}
	/* should single pulse on upper/lower or both bytes depending on mode! */
	gpiod_set_value(bus->we->desc[0], true);
	gpiod_set_value(bus->we->desc[0], false);
	for (d = 0; d < bus->datas->ndescs; d++) {
#if FIXME
		gpiod_direction_input(bus->datas->desc[d]);
#endif
	}
}

/* cart select */

static void set_slot_power(struct retrode3_slot *slot, int mV)
{
// printk("%s: %d %px\n", __func__, mV, slot->power);

	if (IS_ERR_OR_NULL(slot->power))
		return;

	switch(mV) {
		case 5000:
			gpiod_direction_output(slot->power, false);	// switch to output and pull down
			break;
		case 3300:
			gpiod_direction_input(slot->power);	// floating
			break;
		default:
printk("%s: unknown voltage %d\n", __func__, mV);
	}
}

static void select(struct retrode3_bus *bus, struct retrode3_slot *slot)
{ /* chip select */
	int i;

// printk("%s:\n", __func__);

	if (slot)
		mutex_lock(&bus->select_lock);

	for(i=0; i<ARRAY_SIZE(bus->slots); i++) {
		if (!bus->slots[i])
			continue;
		gpiod_set_value(bus->slots[i]->ce, (bus->slots[i] == slot) ? true:false);
	}

	if (!slot && mutex_is_locked(&bus->select_lock))
		mutex_unlock(&bus->select_lock);
}

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

	select(slot->bus, slot);

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
			word = err;

			remaining = copy_to_user(buf, (char *) &word, sz);
#else

			word = err;

			if (fill == sizeof(buffer)) { // flush buffer
				remaining = copy_to_user(buf+read, buffer, fill);
				if (remaining)
					goto failed;
				fill = 0;
			}
			*((u16 *) &buffer[fill]) = word;
			fill += 2;
			*ppos += 2;
			count -= 2;
			read += 2;
#endif
		} else {
			u8 byte;

			if (slot->bus_width == 16)
				err = set_address(slot->bus, *ppos);	// A0 determines lower/upper byte
			else
				err = set_address(slot->bus, *ppos*2);	// shift A0->A1 etc., read D0..D7 only
			if(err < 0)
				goto failed;

			err = read_byte(slot->bus);
			if(err < 0)
				goto failed;

#ifndef CONFIG_RETRODE3_BUFFER
			sz = 1;	// handle byte read
			byte = err;

			remaining = copy_to_user(buf, (char *) &byte, sz);
#else
			byte = err;

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

	select(slot->bus, NULL);

	return read;

failed:
	select(slot->bus, NULL);

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

	select(slot->bus, slot);

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

	select(slot->bus, NULL);

	return written;
}

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
		round_jiffies_relative(
			msecs_to_jiffies(50)));	// start next check
}

static void retrode3_polling_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);
	int i;
	u32 word;	// left and right controllers and two mux states combined

	select(slot->bus, slot);

	set_address(slot->bus, slot->bus->prev_addr | BIT(22));	// clear A22 (select MUX)
	word = read_word(slot->bus);	// 16 bits for both channels
	if(word < 0) { // invalid read
		select(slot->bus, NULL);
		return;
	}
	set_address(slot->bus, slot->bus->prev_addr | BIT(22));	// set A22 (select MUX)
	word |= read_word(slot->bus) << 16;
	// no error handling
	select(slot->bus, NULL);

#define GENESIS_U BIT(3)		// pin 5 / D3
#define GENESIS_D BIT(2)		// pin 4 / D2
#define GENESIS_L BIT(1+16)		// pin 3 / D1 (select = 1)
#define GENESIS_R BIT(0+16)		// pin 2 / D0 (select = 1)
#define GENESIS_A BIT(4)		// pin 9 / D4 (select = 0)
#define GENESIS_B BIT(4+16)		// pin 9 / D4 (select = 1)
#define GENESIS_S BIT(5)		// pin 6 / D5 (select = 0)
#define GENESIS_C BIT(5+16)		// pin 6 / D5(select = 1)

	for (i=0; i < 2; i++) {
		struct retrode3_controller *c = &slot->controllers[i];
		int state = word >> c->data_offset;

		if (c->state_valid) { // skip first analysis after boot
			int changes = state ^ c->last_state;

if (changes) printk("%s: controller %d changes %08x state %08x\n", __func__, i, changes, state);
// FIXME: define a macro that takes the GENESIS_* and the KEY_* as arguments
			if (changes & GENESIS_U)
				input_report_key(c->input, KEY_U, state & GENESIS_U);
			if (changes & GENESIS_D)
				input_report_key(c->input, KEY_D, state & GENESIS_D);
			if (changes & GENESIS_L)
				input_report_key(c->input, KEY_L, state & GENESIS_L);
			if (changes & GENESIS_R)
				input_report_key(c->input, KEY_R, state & GENESIS_R);
			if (changes & GENESIS_A)
				input_report_key(c->input, KEY_A, state & GENESIS_A);
			if (changes & GENESIS_B)
				input_report_key(c->input, KEY_B, state & GENESIS_B);
			if (changes & GENESIS_C)
				input_report_key(c->input, KEY_C, state & GENESIS_C);
			if (changes & GENESIS_S)
				input_report_key(c->input, KEY_ENTER, state & GENESIS_S);
		}

		c->last_state = state;
		c->state_valid = true;
	}

	schedule_delayed_work(&slot->work,
		round_jiffies_relative(
			msecs_to_jiffies(20)));	// start next check
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

static int retrode3_probe(struct platform_device *pdev)
{
        struct retrode3_bus *bus;
        int i = 0;
	struct device_node *slots, *child = NULL;

        dev_dbg(&pdev->dev, "%s\n", __func__);

        if (!pdev->dev.of_node) {
                dev_err(&pdev->dev, "No device tree data\n");
                return EINVAL;
        }

        bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
        if (bus == NULL)
                return -ENOMEM;

// printk("%s: a\n", __func__);

	bus->addrs = devm_gpiod_get_array(&pdev->dev, "addr", GPIOD_OUT_HIGH);
// if (IS_ERR(bus->addrs)) usw...
// printk("%s: %px\n", __func__, bus->addrs);
	bus->datas = devm_gpiod_get_array(&pdev->dev, "data", GPIOD_IN);
// printk("%s: %px\n", __func__, bus->datas);
	bus->oe = devm_gpiod_get(&pdev->dev, "oe", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: %px\n", __func__, bus->oe);
	gpiod_set_value(bus->oe, false);	// turn inactive
	bus->we = devm_gpiod_get_array(&pdev->dev, "we", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: %px\n", __func__, bus->we);
	gpiod_set_value(bus->we->desc[0], false);	// make both inactive
	gpiod_set_value(bus->we->desc[1], false);
	bus->time = devm_gpiod_get(&pdev->dev, "time", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: %px\n", __func__, bus->time);
	gpiod_set_value(bus->time, false);	// make inactive
	bus->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: %px\n", __func__, bus->reset);
	gpiod_set_value(bus->reset, false);	// make inactive

#if 0
printk("%s: 1\n", __func__);
printk("%s: bus=%px\n", __func__, bus);
printk("%s: addrs=%px\n", __func__, bus->addrs);
printk("%s: desc[0]=%px\n", __func__, bus->addrs->desc[0]);
printk("%s: gdev=%px\n", __func__, bus->addrs->desc[0]->gdev);
printk("%s: chip=%px\n", __func__, bus->addrs->desc[0]->gdev->chip);
#endif

	if (bus->addrs->ndescs != 24 ||
	    bus->datas->ndescs != 16 ||
	    bus->we->ndescs != 2) {
		dev_err(&pdev->dev, "Invalid number of gpios (addr=%d, data=%d, we=%d)\n",
			bus->addrs->ndescs, bus->datas->ndescs, bus->we->ndescs);
		return EINVAL;
	}
#if 0
{
	struct gpio_chip *gc = bus->addrs->desc[0]->gdev->chip;
	printk("%s: get %ps\n", __func__, gc->get);
	printk("%s: getmult %ps\n", __func__, gc->get_multiple);
	printk("%s: set %ps\n", __func__, gc->set);
	printk("%s: setmult %ps\n", __func__, gc->set_multiple);
	printk("%s: dirout %ps\n", __func__, gc->direction_output);	// no direction_output_multiple!
	printk("%s: dirin %ps\n", __func__, gc->direction_input);	// no direction_input_multiple!
	printk("%s: setconf %ps\n", __func__, gc->set_config);
}
#endif

	mutex_init(&bus->select_lock);

	slots = of_get_child_by_name(pdev->dev.of_node, "slots");
	while ((child = of_get_next_child(slots, child))) {
		struct retrode3_slot *slot;
		struct device *dev;
		int id;
		int ret;

		if (i >= ARRAY_SIZE(bus->slots)) {
			dev_err(&slot->dev, "too many slots\n");
// FIXME: better error handling
			// FIXME: dealloc previously added devices
			kfree(bus);
			return -EINVAL;
		}

		slot = kzalloc(sizeof(*slot), GFP_KERNEL);
		if (!slot) {
			kfree(bus);
			return -ENOMEM;
		}

		slot->bus = bus;
		bus->slots[i++] = slot;

		if (of_property_match_string(child, "compatible", "openpandora,retrode3-slot") >= 0) {

// FIXME: make this separate functions

			id = ida_simple_get(&retrode3_minors, 0, RETRODE3_MINORS, GFP_KERNEL);
			if (id < 0) {
				// FIXME: dealloc previously added devices
				kfree(bus);
				kfree(slot);
				return -EINVAL;
			}

			slot->id = id;
			dev = &slot->dev;
			device_initialize(dev);
			dev->devt = retrode3_first + id;
			dev->class = retrode3_class;
			dev->parent = &pdev->dev;
			dev->release = retrode3_slot_release;
			dev_set_drvdata(dev, slot);
			dev_set_name(dev, "slot%d", id);
			dev->of_node = child;

			slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
			gpiod_set_value(slot->ce, false);	// turn inactive
			slot->cd = devm_gpiod_get(dev, "cd", GPIOD_IN);
			slot->led = devm_gpiod_get(dev, "status", GPIOD_OUT_HIGH);
			if (!IS_ERR_OR_NULL(slot->led))
				gpiod_set_value(slot->led, false);	// turn inactive
			slot->power = devm_gpiod_get(dev, "power", GPIOD_IN);
			of_property_read_u32_index(child, "address-width", 0, &slot->addr_width);
			of_property_read_u32_index(child, "bus-width", 0, &slot->bus_width);

			cdev_init(&slot->cdev, &slot_fops);
			slot->cdev.owner = THIS_MODULE;

// FIXME: should be some devm_cdev_device_add
			ret = cdev_device_add(&slot->cdev, &slot->dev);
			if (ret) {
				dev_err(&slot->dev, "failed to add device: %d\n", ret);
				// FIXME: dealloc previously added devices
				kfree(bus);
				kfree(slot);
				return ret;
			}

//			set_slot_power(slot, 5000);	// switch to 5V if possible
			set_slot_power(slot, 3300);	// switch to 3.3V if possible

			bus->prev_addr = EOF - 1;	// needed to bring all address gpios in a defined state
//			set_address(slot->bus, EOF-1);	// bring all address gpios in a defined state
			set_address(slot->bus, 0);

			slot->cd_state = -1;	// enforce a state update event for initial state
#if 1	// use polling
			INIT_DELAYED_WORK(&slot->work, retrode3_cd_work);
			schedule_delayed_work(&slot->work,
				round_jiffies_relative(
					msecs_to_jiffies(50)));	// start first check

#else	// use interrupt (untested)
			ret = devm_request_threaded_irq(dev, gpiod_to_irq(slot->cd),
				NULL, retrode3_gpio_cd_irqt,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"cart-detect", slot);

#endif
		} else if (of_property_match_string(child, "compatible", "openpandora,retrode3-controller") >= 0) {
			struct device_node *controller = NULL;

			dev = &slot->dev;
			device_initialize(dev);
			dev->class = retrode3_class;
			dev->parent = &pdev->dev;
			dev_set_name(dev, "gamecontroller");
			dev->of_node = child;

			ret = device_add(dev);
			if (ret) {
				dev_err(&slot->dev, "failed to add device: %d\n", ret);
				// FIXME: dealloc previously added devices
				kfree(bus);
				kfree(slot);
				return ret;
			}

			slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
			gpiod_set_value(slot->ce, false);	// turn inactive

			id = 0;
			while ((controller = of_get_next_child(child, controller))) {
				struct input_dev *input_dev;

				dev_info(&slot->dev, "add game controller %d\n", id);

				if (id >= ARRAY_SIZE(bus->slots)) {
					dev_err(&slot->dev, "too many game controllers\n");
					// FIXME: better error handling
					// FIXME: dealloc previously added devices
					kfree(bus);
					return -EINVAL;
				}

				input_dev = devm_input_allocate_device(dev);
				if (!input_dev)
					return -ENOMEM;

				slot->controllers[id].input = input_dev;

				input_dev->name = kasprintf(GFP_KERNEL, "Retrode 3 Game Controller %d", i);
				input_dev->phys = kasprintf(GFP_KERNEL, "%s/input%d", dev_name(dev), i);

				input_dev->id.bustype = BUS_GAMEPORT;

				input_set_capability(input_dev, EV_KEY, KEY_A);
				input_set_capability(input_dev, EV_KEY, KEY_B);
				input_set_capability(input_dev, EV_KEY, KEY_C);
				input_set_capability(input_dev, EV_KEY, KEY_U);
				input_set_capability(input_dev, EV_KEY, KEY_D);
				input_set_capability(input_dev, EV_KEY, KEY_L);
				input_set_capability(input_dev, EV_KEY, KEY_R);
				input_set_capability(input_dev, EV_KEY, KEY_ENTER);

				ret = input_register_device(input_dev);
				if (ret) {
					dev_err(dev,
						"Failed to register input device: %d\n", ret);
					return ret;
				}

				of_property_read_u32(controller, "data-offset", &slot->controllers[id].data_offset);

				id++;
			}

			INIT_DELAYED_WORK(&slot->work, retrode3_polling_work);
			schedule_delayed_work(&slot->work,
				round_jiffies_relative(
					msecs_to_jiffies(50)));	// start polling
		} else {
			dev_err(&slot->dev, "unknown child type\n");
			// FIXME: dealloc previously added devices
			kfree(bus);
			kfree(slot);
			return ret;
		}
		dev_dbg(dev, "%s added\n", __func__);
	}

        platform_set_drvdata(pdev, bus);

        dev_dbg(&pdev->dev, "%s successful\n", __func__);

	select(bus, NULL);	// deselect all slots

        return 0;
}

static int retrode3_remove(struct platform_device *pdev)
{
        struct retrode3_bus *bus = platform_get_drvdata(pdev);
	int i;

	select(bus, NULL);	// deselect all slots

	for (i=0; i<ARRAY_SIZE(bus->slots); i++) {
		struct retrode3_slot *slot = bus->slots[i];

		cancel_delayed_work_sync(&slot->work);
		cdev_device_del(&slot->cdev, &slot->dev);
	}

	mutex_destroy(&bus->select_lock);

        return 0;
}

static const struct of_device_id retrode3_of_match[] = {
        { .compatible = "openpandora,retrode3" },
        {},
};
MODULE_DEVICE_TABLE(of, retrode3_of_match);

static struct platform_driver retrode3_driver = {
        .probe          = retrode3_probe,
        .remove         = retrode3_remove,
        .driver = {
                .name   = "retrode3",
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(retrode3_of_match)
        },
};

static int retrode3_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct retrode3_slot *slot = container_of(dev, struct retrode3_slot, dev);
	int ret;
	char buf[100];

        dev_info(&slot->dev, "%s\n", __func__);

	ret = add_uevent_var(env, "SLOT=%s", dev_name(dev));
	if (ret)
		return ret;

	sense_show(dev, NULL, buf);
	// strip off \n?
	ret = add_uevent_var(env, "STATE=%s", buf);
	if (ret)
		return ret;

	return 0;
}

static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", gpiod_get_value(slot->cd)?"active":"empty");
}
static DEVICE_ATTR_RO(sense);

static struct attribute *retrode3_attrs[] = {
	&dev_attr_sense.attr,
	NULL,
};
ATTRIBUTE_GROUPS(retrode3);

static int __init retrode3_module_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&retrode3_first, 0, RETRODE3_MINORS, "retrode3");
	if (ret < 0) {
		pr_err("failed to allocate device numbers: %d\n", ret);
		return ret;
	}

	retrode3_class = class_create(THIS_MODULE, "retrode3");
	if (IS_ERR(retrode3_class)) {
		ret = PTR_ERR(retrode3_class);
		pr_err("failed to create class: %d\n", ret);
		goto err_unregister_chrdev;
	}

	retrode3_class->dev_groups = retrode3_groups;	// for additional /sys attributes

// triggers permanent sequence of uEvents...
//	retrode3_class->dev_uevent = retrode3_uevent;

	ret = platform_driver_register(&retrode3_driver);

	if (ret < 0)
		goto err_destroy_class;

	pr_info("retrode3 driver registered with major %d\n", MAJOR(retrode3_first));

	return ret;

err_destroy_class:
	class_destroy(retrode3_class);
err_unregister_chrdev:
	unregister_chrdev_region(retrode3_first, RETRODE3_MINORS);

	return ret;
}

static void __exit retrode3_module_exit(void)
{

	class_destroy(retrode3_class);
	unregister_chrdev_region(retrode3_first, RETRODE3_MINORS);
	ida_destroy(&retrode3_minors);

	platform_driver_unregister(&retrode3_driver);
}

module_init(retrode3_module_init);
module_exit(retrode3_module_exit);

MODULE_ALIAS("retrode3");

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("retrode 3 slot reader driver");
MODULE_LICENSE("GPL v2");
