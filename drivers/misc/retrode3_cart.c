// SPDX-License-Identifier: GPL-2.0

/*
 * driver for Retrode 3 game cartridge driver
 *
 * based on ideas and fragments from
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
 * memory location. But also includes a special access mode in the upper bits.
 */

/* special mode constants - should be exported as ABI to user-space */

#define MODE_SIMPLE_BUS	0	// default read/write with just CE, RD/WR0/WR8
#define MD_ROM		MODE_SIMPLE_BUS
#define MD_P10		1	// 10 toggle pulses on CLK
#define MD_P1		2	// 1 toggle pulses on CLK
#define MD_TIME		3	// address with TIME impulse with WE
#define MD_FLASH	unused 0x04 unused
#define MD_ENSRAM	5	// TIME impulse without WE (despite write?)
#define MD_EEPMODE	6

// define MD specific gpios:	gpio-time = pb5

#define SNES_REGULAR	MODE_SIMPLE_BUS
#define SNES_HIROM	9

// define SNES specific gpios:	gpio-? =

// for NES address mapping: https://www.nesdev.org/wiki/CPU_memory_map

#define NES_PRG		10	// CPU d0..d7	ROM $8000-$ffff
#define NES_CHR		11	// PPU d8..d15	?
#define NES_CHR_M2	12	// PPU d8..d15	?
#define NES_MMC5_SRAM	13	// CPU d0..d7	special RAM? $0000-$07ff
#define NES_REG		14	// PPU d8..d15	PPU registers? $2000-$2007
#define NES_RAM		15	// PPU d8..d15	internal RAM? $0000-$07ff
#define NES_WRAM	16	// PPU d8..d15?	Cartridge RAM  $6000-$7fff

/* NES special wiring
 d0..d7  <-> CPU
 d8..d15 <-> PPU
 a0..a14  -> A0..A14
 a15..a23 -> ignored
 romsel   -> A15
 a13      -> inverted A16
 for an example: https://www.nesdev.org/wiki/UxROM
*/

#define NES_A0_A14	(0x7fff)	// note that we ignore A15 here since the user selects
					// ROMSEL through different mode to allow for different
					// hardware connections
#define NES_A13		(1<<13)		// connected inside Cart to /VRAM_CS
#define NES_ROMSEL	(1<<15)		// connected to A15
#define NES_A16		(1<<16)		// PPU_/A13

/* should depend on .compatible since it controls hardware peculiarities */

#define IS_MD()		(mode >= MD_ROM && mode <= MD_EEPMODE)
#define IS_SNES()	(mode == SNES_REGULAR || mode == SNES_HIROM)
#define IS_NES()	(mode >= NES_PRG && mode <= NES_WRAM)

static ssize_t retrode3_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct retrode3_slot *slot = file->private_data;
	uint32_t addr = *ppos & 0xffffff;	/* max address is 24 bit */
	uint32_t mode = *ppos >> 24;		/* access mode control */
	int err;

#ifndef CONFIG_RETRODE3_BUFFER
	ssize_t read, sz;
#else
	ssize_t read;
	unsigned long remaining;
	unsigned char buffer[1024];
	unsigned int fill = 0;
#endif

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	if (IS_NES()) { // NES mapping to address bus
// printk("%s: pos %08x\n", __func__, *ppos);
// printk("%s: addr 1 %08x\n", __func__, addr);
		addr &= NES_A0_A14;	// limit to 15 bit
// printk("%s: addr 2 %08x\n", __func__, addr);
		// manipulate A15 if needed
		if (!(addr & NES_A13))
			addr |= NES_A16;	// A16 (PPU_/A13) is !A13
// printk("%s: addr 3 %08x\n", __func__, addr);
	}

//	dev_info(&slot->dev, "%s\n", __func__);

	read = 0;

	select_slot(slot->bus, slot);

	if (addr + count >= EOF) {
		if (addr >= EOF)
			count = 0;	// read nothing
		else
			count = EOF - addr;	// limit to EOF
	}

	while (count > 0) {
#ifndef CONFIG_RETRODE3_BUFFER
		unsigned long remaining;
#endif

		if (mode == MODE_SIMPLE_BUS && slot->bus_width == 16 && count >= 2 && ((addr & 1) == 0)) {
			uint16_t word;

			err = set_address(slot->bus, addr);
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
			*((uint16_t *) &buffer[fill]) = swab16(word);	// use htons()?
			fill += 2;
			*ppos += 2;
			addr += 2;
			count -= 2;
			read += 2;
#endif
		} else {
			uint8_t byte;

			if (slot->bus_width == 16) { // megadrive
				err = set_address(slot->bus, addr);	// A0 determines lower/upper byte
				byte = err = read_byte(slot->bus);	// read half based on a0
			}
			else { // 8 bit bus SNES or NES
				err = set_address(slot->bus, addr);	// includes setting physical A0
				switch (mode) {
					case NES_PRG:
					case NES_MMC5_SRAM:
						byte = err = read_half(slot->bus, 1);	// NES CPU bus = D0..D7
						break;
					case NES_CHR:
					case NES_CHR_M2:	// FIXME: what is the difference? Clocking M2 = CE-NES
					case NES_REG:		// hier evtl. A15 = 1?
					case NES_RAM:
					case NES_WRAM:
						byte = err = read_half(slot->bus, 0);	// NES PPU bus = D8..D15
						break;
					case MODE_SIMPLE_BUS:
						byte = err = read_half(slot->bus, 1);	// D0..D7
						break;
					default:
						dev_err(&slot->dev, "%s: mode (%d) not implemented\n", __func__, mode);
						return -EIO;
				}
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
			*((uint8_t *) &buffer[fill]) = byte;
			fill += 1;
			*ppos += 1;
			addr += 1;
			count -= 1;
			read += 1;
#endif
		}

#ifndef CONFIG_RETRODE3_BUFFER
                if (remaining)
                        goto failed;

		*ppos += sz;
		addr += sz;
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
	uint32_t addr = *ppos & 0xffffff;	/* max address is 24 bit */
	uint32_t mode = *ppos >> 24;		/* access mode control */
	ssize_t written, sz;
	unsigned long copied;
	int err;

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	if (IS_NES()) { // NES mapping to address bus
		addr &= NES_A0_A14;	// limit to 15 bit
		// manipulate A15 if needed
		if (!(addr & NES_A13))
			addr |= NES_A16;	// A16 (PPU_/A13) is !A13
// printk("%s: addr 2 %08x\n", __func__, addr);
	}
//	dev_info(&slot->dev, "%s\n", __func__);

	written = 0;

	select_slot(slot->bus, slot);

	if (addr + count >= EOF) {
		if (addr >= EOF)
			count = 0;	// write nothing
		else
			count = EOF - addr;	// limit to EOF
	}

	while (count > 0) {
		int allowed;
		uint8_t byte;

// FIXME: loop over bytes and write them to consecutive addresses
// handle words for 16 bit bus and faster write

		sz = sizeof(byte);

		copied = copy_from_user(&byte, buf, sz);

		switch (mode) {
			case MD_TIME:
				dev_info(&slot->dev, "%s: write MD_TIME %08x %02x\n", __func__, addr, byte);
				err = set_address(slot->bus, addr);
				if (err < 0)
					return err;
				// write with TIME impulse
				;;
			case NES_PRG:
			case NES_MMC5_SRAM:
				dev_info(&slot->dev, "%s: write NES PRG/SRAM %08x %02x\n", __func__, addr, byte);
				err = set_address(slot->bus, addr);
				if (err < 0)
					return err;
// CHECKME: do we have to play the WE0/WE8 differently?
				write_half(slot->bus, byte, 1);	// NES CPU bus = D0..D7 and WE0
				break;
			case NES_CHR:
			case NES_CHR_M2:	// FIXME: what is the difference? Clocking M2 = CE-NES
			case NES_REG:
			case NES_RAM:
			case NES_WRAM:
				dev_info(&slot->dev, "%s: write CHR %08x %02x\n", __func__, addr, byte);
				err = set_address(slot->bus, addr);
				if (err < 0)
					return err;
// CHECKME: do we have to play the WE0/WE8 differently?
				write_half(slot->bus, byte, 0);	// NES PPU bus = D8..D15 and WE8
				break;
			case MODE_SIMPLE_BUS:
				dev_info(&slot->dev, "%s: write MD %08x %02x\n", __func__, addr, byte);
				err = set_address(slot->bus, addr);
				if (err < 0)
					return err;
				write_half(slot->bus, byte, 1);	// D0..D7 and WE0
				break;
			default:
				dev_err(&slot->dev, "%s: mode (%d) not implemented\n", __func__, mode);
				return -EIO;
		}

		if (copied) {
			written += sz - copied;
			if (written)
				break;
			return -EFAULT;
		}

		*ppos += sz;
		addr += sz;
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
//		set_slot_power_mV(slot, 5000);	// switch to 5V if possible
		set_slot_power_mV(slot, 3300);	// switch to 3.3V if possible
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
/* not yet used */
	{ .compatible = "openpandora,retrode3-slot-snes", .data = (void *) 0 },
	{ .compatible = "openpandora,retrode3-slot-megadrive", .data = (void *) 1 },
	{ .compatible = "openpandora,retrode3-slot-nnes", .data = (void *) 2 },
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
