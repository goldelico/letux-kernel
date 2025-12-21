/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/ptrace.h>

#include "retrode3.h"

#define RETRODE3_MINORS	16
static DEFINE_IDA(retrode3_minors);
static dev_t retrode3_first;
static struct class *retrode3_class;

/*
 * game cart driver
 */

/*
 * This function reads the slot. The ppos points directly to the
 * memory location. But also includes a special access mode in the upper 8 bits.
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

// can we simplify this? E.g. make the distinction between ROM (PRG) and RAM depend on address?
// so that /dev/slot mimicks the CPU_memory_map

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
 ppu/a13  -> inverted A16
 unused   -> A17..A23
 cpu_r/w  -> WE0
 prg/ce   -> CE-NES & !A15
 ppurd/wr -> RD / WE8
 cpu_m2   -> CE-NES (PHI2)
 for an example: https://www.nesdev.org/wiki/UxROM
*/

#define NES_A0_A14	(0x7fff)	// note that we ignore A15 here since the user selects
					// ROMSEL through different mode to allow for different
					// hardware connections
#define NES_A13		(1<<13)		// connected inside Cart to /VRAM_CS
#define NES_ROMSEL	(1<<15)		// connected to A15
#define NES_A16		(1<<16)		// PPU_/A13

/* this section needs cleanup!
 * it is a partial and modified and adapted copy of:
 *    https://github.com/sanni/cartreader/blob/0b8ac2ec14675ae55952ddf0b4590ba8d2899664/Cart_Reader/NES.ino#L1012
 */

#define PORTK(data) (retrode3_drive_half(slot->controller, data, 0))		// D0..D7 := data (and output)

#define ROMSEL_HI retrode3_bus_set_address(slot->controller, slot->controller->current_address |= (1 << 15))		// A15 = 1 | /CE_NES
#define ROMSEL_LO retrode3_bus_set_address(slot->controller, slot->controller->current_address &= ~(1 << 15))		// A15 = 0 | /CE_NES
#define PHI2_HI (gpiod_set_value(slot->ce, 0))			// /CE_NES = inactive
#define PHI2_LOW (gpiod_set_value(slot->ce, 1))			// /CE_NES = active
// PRG = CPU
// FIXME:
#define PRG_READ // (retrode3_set_we(slot->controller, 0, 0)) // (gpiod_set_value(slot->controller->we->desc[0], 0))	// /WE0 = inactive
#define PRG_WRITE // (retrode3_set_we(slot->controller, 0, 1)) // (gpiod_set_value(slot->controller->we->desc[0], 1))	// /WE0 = active
// CHR = PPU
#define CHR_READ_HI PORTF // (gpiod_set_value(slot->oe, 1), gpiod_set_value(slot->controller->we->desc[1], 0))
#define CHR_READ_LOW PORTF // (gpiod_set_value(slot->oe, 0), gpiod_set_value(slot->controller->we->desc[1], 0))
#define CHR_WRITE_HI PORTF // (gpiod_set_value(slot->oe, 1), gpiod_set_value(slot->controller->we->desc[1], 1))
#define CHR_WRITE_LOW // (gpiod_set_value(slot->oe, 0), gpiod_set_value(slot->controller->we->desc[1], 1))

#define MODE_READ (retrode3_end_drive(slot->controller))			// D0..D15 = input
#define MODE_WRITE						// D0..D7 = output (done automatically)

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

// dev_info(&slot->dev, "%s: %08llx\n", __func__, *ppos);

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	if (IS_NES()) { // NES mapping to address bus
// printk("%s: pos %08llx\n", __func__, *ppos);
// prints("%s: addr 1 %08x\n", __func__, addr);
		addr &= NES_A0_A14;	// limit to 15 bit
// printk("%s: addr 2 %08x\n", __func__, addr);
		if (!(addr & NES_A13))
			addr |= NES_A16;	// A16 (PPU_/A13) is !A13
// printk("%s: addr 3 %08x\n", __func__, addr);
	}

	read = 0;

	retrode3_bus_select_slot(slot->controller, slot);

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

			err = retrode3_bus_set_address(slot->controller, addr);
			if(err < 0)
				goto failed;

			err = retrode3_read_word(slot->controller);
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
				err = retrode3_bus_set_address(slot->controller, addr);	// A0 determines lower/upper byte
				byte = err = retrode3_read_byte(slot->controller);	// read half based on a0
			}
			else { // 8 bit bus SNES or NES
				switch (mode) {
					case NES_PRG:
					case NES_MMC5_SRAM:
						// bei A15 = 0 RAM auslesen

  MODE_READ;
  PRG_READ;
						err = retrode3_bus_set_address(slot->controller, addr);	// includes setting physical A0
//  PHI2_HI; // does not work on Retrode3
  PHI2_LOW;
						if (!(addr & NES_ROMSEL))
							ROMSEL_LO;	// ROMSEL is A15
						else
							ROMSEL_HI;	// ROMSEL is A15
						byte = err = retrode3_read_half(slot->controller, 1);	// NES CPU bus = D0..D7
						break;
					case NES_CHR:
					case NES_CHR_M2:	// FIXME: what is the difference? Clocking M2 = CE-NES
					case NES_REG:		// hier evtl. A15 = 1?
					case NES_RAM:
					case NES_WRAM:
						// bei A15 = 0 RAM auslesen
						err = retrode3_bus_set_address(slot->controller, addr);	// includes setting physical A0
						byte = err = retrode3_read_half(slot->controller, 0);	// NES PPU bus = D8..D15
						break;
					case MODE_SIMPLE_BUS:
						// bei NES und A15 = 0 RAM auslesen
						err = retrode3_bus_set_address(slot->controller, addr);	// includes setting physical A0
						byte = err = retrode3_read_half(slot->controller, 1);	// D0..D7
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

	retrode3_bus_select_slot(slot->controller, NULL);

	return read;

failed:
	retrode3_bus_select_slot(slot->controller, NULL);

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

//	dev_info(&slot->dev, "%s\n", __func__);

// if (mode) dev_warn(&slot->dev, "%s: mode = %d\n", __func__, mode);

	if (IS_NES()) { // NES mapping to address bus
 printk("%s: pos %08llx\n", __func__, *ppos);
 printk("%s: addr 1 %08x\n", __func__, addr);
		addr &= NES_A0_A14;	// limit to 15 bit
 printk("%s: addr 2 %08x\n", __func__, addr);
		if (!(addr & NES_A13))
			addr |= NES_A16;	// A16 (PPU_/A13) is !A13
 printk("%s: addr 3 %08x\n", __func__, addr);
	}

	written = 0;

	retrode3_bus_select_slot(slot->controller, slot);

	if (addr + count >= EOF) {
		if (addr >= EOF)
			count = 0;	// write nothing
		else
			count = EOF - addr;	// limit to EOF
	}

	while (count > 0) {
		// int allowed;
		uint8_t byte;

// FIXME: loop over bytes and write them to consecutive addresses
// handle words for 16 bit bus and faster write

		sz = sizeof(byte);

		copied = copy_from_user(&byte, buf, sz);

		switch (mode) {
			case MD_TIME:
				dev_info(&slot->dev, "%s: write MD_TIME %08x %02x\n", __func__, addr, byte);
				err = retrode3_bus_set_address(slot->controller, addr);
				if (err < 0)
					return err;
				// write with TIME impulse
				break;
				;;
			case NES_PRG:
			case NES_MMC5_SRAM:
				dev_info(&slot->dev, "%s: write NES PRG/SRAM %08x %02x\n", __func__, addr, byte);


/*
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
  PORTK = data;

  retrode3_bus_set_address(slot->controller, addr);  // PHI2 low, ROMSEL always HIGH
  //  _delay_us(1);
  PHI2_HI;
  //_delay_us(10);
  set_romsel(address);  // ROMSEL is low if need, PHI2 high
  _delay_us(1);         // WRITING
  //_delay_ms(1); // WRITING
  // PHI2 low, ROMSEL high
  PHI2_LOW;
  _delay_us(1);
  ROMSEL_HI;
  // Back to read mode
  //  _delay_us(1);
  PRG_READ;
  MODE_READ;
  set_address(0);
  // Set phi2 to high state to keep cartridge unreseted
  //  _delay_us(1);
  PHI2_HI;
*/


  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
				err =retrode3_bus_set_address(slot->controller, addr);
				if (err < 0)
					return err;
  PORTK(byte);
			//	set_byte(slot->controller, byte, 1);	// NES CPU bus = D0..D7 and WE0
  PHI2_HI;
  // set_romsel(address);  // ROMSEL is low if need, PHI2 high
				if (!(addr & NES_ROMSEL))
					ROMSEL_LO;	// ROMSEL is inverted A15
				else
					ROMSEL_HI;	// ROMSEL is inverted A15
 // _delay_us(1);         // WRITING
  PHI2_LOW;	// may activate PRG-CE if ROMSEL_LOW (A15 = HI)
//  _delay_us(1);
  ROMSEL_HI;
  // Back to read mode
  PRG_READ;
  MODE_READ;
  // Set phi2 to high state to keep cartridge unresetted
  PHI2_HI;

				break;
			case NES_CHR:
			case NES_CHR_M2:	// FIXME: what is the difference? Clocking M2 = CE-NES
			case NES_REG:
			case NES_RAM:
			case NES_WRAM:
				dev_info(&slot->dev, "%s: write CHR %08x %02x\n", __func__, addr, byte);
				err = retrode3_bus_set_address(slot->controller, addr);
				if (err < 0)
					return err;
// CHECKME: do we have to play the WE0/WE8 differently?
				retrode3_write_half(slot->controller, byte, 0);	// NES PPU bus = D8..D15 and WE8
				break;
			case MODE_SIMPLE_BUS:
				dev_info(&slot->dev, "%s: write BUS %08x %02x\n", __func__, addr, byte);
				err = retrode3_bus_set_address(slot->controller, addr);
				if (err < 0)
					return err;
				retrode3_write_half(slot->controller, byte, 1);	// D0..D7 and WE0
// CHECKME: do we have to play the WE0/WE8 differently?
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

	retrode3_bus_select_slot(slot->controller, NULL);

	return written;
}

#define CONFIG_RETRODE3_MDSLOT 294

int retrode3_get_slot_power_mV(struct retrode3_slot *slot)
{
	if(!slot || IS_ERR_OR_NULL(slot->power))
		return -ENODEV;
	return gpiod_get_direction(slot->power) ? 3300 : 5000;
}

int retrode3_set_slot_power_mV(struct retrode3_slot *slot, int mV)
{
printk("%s: %dmV %px\n", __func__, mV, slot->power);

	if (IS_ERR_OR_NULL(slot->power))
		return -ENODEV;

	switch(mV) {
		case 5000:
			gpiod_direction_output(slot->power, 0);	// switch to output 0
			break;
		case 3300:
#if CONFIG_RETRODE3_MDSLOT == 293
			return -EINVAL;	// broken
#endif
			gpiod_direction_input(slot->power);	// switch to floating
			break;
		default:
printk("%s: unknown voltage %dmV\n", __func__, mV);
			return -EINVAL;
	}
	return 0;
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

	printk("%s %d: inode=%px file=%px\n", __func__, __LINE__, inode, file);

	slot = container_of(inode->i_cdev, struct retrode3_slot, cdev);

		dev_dbg(&slot->dev, "%s\n", __func__);

	ret = gpiod_get_value_cansleep(slot->cd);	// check cart detect

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

/**
 * cart detect
 */

static ssize_t sense_show(struct device *dev, struct device_attribute *attr, char *buf);

static void retrode3_update_cd(struct retrode3_slot *slot)
{
	int cd_state = gpiod_get_value_cansleep(slot->cd);	// check cart detect pin

//	printk("%s %d: cd_state=%d\n", __func__, __LINE__, cd_state);

	if (cd_state < 0)
		return;	// ignore

	if (cd_state != slot-> cd_state) {
		char *envp[3];
		char buf[20];
		int len;

// printk("%s %d: state changed to %d\n", __func__, __LINE__, cd_state);

		slot->cd_state = cd_state;

		envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
		len = sense_show(&slot->dev, NULL, buf);
		if (len < 0)
			return;
		if (buf[len - 1] == '\n')
			buf[len - 1] = 0;	// strip off \n
		envp[1] = kasprintf(GFP_KERNEL, "SENSE=%s", buf);
		envp[2] = NULL;
// printk("%s %d: %s %s\n", __func__, __LINE__, envp[0], envp[1]);
		// check with: udevadm monitor --environment
		kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
	}
}

#if UNUSED
static irqreturn_t retrode3_gpio_cd_irqt(int irq, void *dev_id)
{
	struct retrode3_slot *slot = dev_id;

	retrode3_update_cd(slot);

	return IRQ_HANDLED;
}
#endif

static void retrode3_cd_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);

	retrode3_update_cd(slot);

	schedule_delayed_work(&slot->work,
			msecs_to_jiffies(50));	// start next check
}

/* device_create helper: allocates a retrode3_slot, sets parent to controller device */

static void retrode3_slot_release(struct device *dev)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

		dev_dbg(&slot->dev, "%s\n", __func__);

	ida_free(&retrode3_minors, slot->id);
	// FIXME: what else to release
}

// FIXME: umbenennen in retrode3_slot_probe?
struct retrode3_slot *retrode3_slot_create_from_of(struct retrode3_bus_controller *controller,
												   struct device_node *np)
{
	struct retrode3_slot *slot;
	struct device *dev;
	int id;
	const char *name = "unknown";
	int ret;

	printk("%s %d\n", __func__, __LINE__);

	if (!controller || !np)
		return ERR_PTR(-EINVAL);

	slot = kzalloc(sizeof(*slot), GFP_KERNEL);
	if (!slot)
		return ERR_PTR(-ENOMEM);

	dev = &slot->dev;
	printk("%s %d\n", __func__, __LINE__);
	device_initialize(dev);
	printk("%s %d\n", __func__, __LINE__);

	dev->parent = controller->dev;
	dev->bus = &retrode3_bus_type;

	dev->of_node = of_node_get(np);

	ret = dev_set_name(dev, np->name);
	if (ret) {
		put_device(dev);	// FIXME: where is the get_device()? in device_initialize()?
		return ERR_PTR(ret);
	}

	slot->controller = controller;
	printk("%s %d\n", __func__, __LINE__);

	ret = device_add(dev);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}
	printk("%s %d\n", __func__, __LINE__);

	list_add_tail(&slot->list, &controller->slots);
	printk("%s %d\n", __func__, __LINE__);

	id = ida_alloc_max(&retrode3_minors, RETRODE3_MINORS-1, GFP_KERNEL);
	if (id < 0) {
		device_del(dev);
		put_device(dev);
		return ERR_PTR(id);
	}
	slot->id = id;

	device_initialize(dev);
	dev->devt = retrode3_first + id;
	dev->class = retrode3_class;
	dev->release = retrode3_slot_release;
	dev_set_drvdata(dev, slot);
	of_property_read_string(np, "name", &name);
	dev_set_name(dev, "slot-%s", name);

	slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
	gpiod_set_value(slot->ce, 0);	// turn inactive
	of_property_read_u32_index(np, "address-width", 0, &slot->addr_width);
	of_property_read_u32_index(np, "bus-width", 0, &slot->bus_width);

	slot->cd = devm_gpiod_get(dev, "cd", GPIOD_IN);
	slot->led = devm_gpiod_get(dev, "status-led", GPIOD_OUT_HIGH);
	if (!IS_ERR_OR_NULL(slot->led))
		gpiod_set_value(slot->led, 0);	// turn inactive
	slot->power = devm_gpiod_get(dev, "power", GPIOD_IN);

	cdev_init(&slot->cdev, &slot_fops);
	slot->cdev.owner = THIS_MODULE;

	// FIXME: should be some devm_cdev_device_add
	ret = cdev_device_add(&slot->cdev, &slot->dev);
	if (ret) {
		device_del(dev);
		put_device(dev);
		return ERR_PTR(id);
	}

	if(!IS_ERR_OR_NULL(slot->power)) {
		if (retrode3_set_slot_power_mV(slot, 3300) < 0)	// switch to 3.3V if possible
			retrode3_set_slot_power_mV(slot, 5000);	// fall back to 5V
	}

	slot->cd_state = -1;	// enforce a state update event for initial state
	INIT_DELAYED_WORK(&slot->work, retrode3_cd_work);
	if(!IS_ERR_OR_NULL(slot->cd)) {
#if 1	// use polling
		schedule_delayed_work(&slot->work,
							  msecs_to_jiffies(50));	// start first check

#else	// use interrupt (untested)
		ret = devm_request_threaded_irq(dev, gpiod_to_irq(slot->cd),
										NULL, retrode3_gpio_cd_irqt,
										IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
										"cart-detect", slot);
#endif
	}

	   dev_info(controller->dev, "created retrode3 device %s\n", dev_name(dev));
	   return slot;
		}
EXPORT_SYMBOL_GPL(retrode3_slot_create_from_of);

void retrode3_bus_client_remove(struct retrode3_slot *slot)
{
	if (!slot)
		return;

	cancel_delayed_work_sync(&slot->work);
	cdev_device_del(&slot->cdev, &slot->dev);

	list_del(&slot->list);

	device_del(&slot->dev);
	put_device(&slot->dev);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_remove);

/*
 * /sysfs
 */

static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

	if (!slot || IS_ERR_OR_NULL(slot->cd))
		return sprintf(buf, "%s\n", "unknown");

// sould use sysfs_emit()
	return sprintf(buf, "%s\n", gpiod_get_value_cansleep(slot->cd)?"active":"empty");
}
static DEVICE_ATTR_RO(sense);

static ssize_t vcc_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	int mV = retrode3_get_slot_power_mV(slot);

	if (mV < 0)
		return sprintf(buf, "fixed\n");

	return sprintf(buf, "%dmV\n", mV);
}

static ssize_t vcc_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	unsigned int mV;
	int err;

	if(IS_ERR_OR_NULL(slot->power))
		return -ENODEV;	// can't control

	err = kstrtouint(buf, 10, &mV);
	if (err < 0)
		return err;

	err = retrode3_set_slot_power_mV(slot, mV);
	if (err < 0)
		return err;

	return count;
}
static DEVICE_ATTR_RW(vcc);

static ssize_t addr_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

	if (!retrode3_is_selected(slot))
		return sprintf(buf, "deselected\n");

	return sprintf(buf, "%08x\n", slot->controller->current_address);
}

static ssize_t addr_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	uint32_t addr;
	int err;

	if (strncmp(buf, "deselect", 8) == 0) {
		retrode3_bus_select_slot(slot->controller, NULL);
		return count;
	}

	if (!retrode3_is_selected(slot))
		retrode3_bus_select_slot(slot->controller, NULL);	// auto-deselect other slot

	// check for 0x prefix?
	err = kstrtouint(buf, 16, &addr);
	if (err < 0)
		return err;

	retrode3_bus_select_slot(slot->controller, slot);
	err = retrode3_bus_set_address(slot->controller, addr);
	if (err < 0)
		return err;

	return count;
}
static DEVICE_ATTR_RW(addr);

static ssize_t data8_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	int byte;

	if (!retrode3_is_selected(slot))
		return sprintf(buf, "deselected\n");

	byte = retrode3_read_byte(slot->controller);

	if (byte < 0)
		return byte;

	return sprintf(buf, "%02x\n", byte);
}

static ssize_t data8_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	unsigned int value;
	int err;

	if (!retrode3_is_selected(slot))
		return -EINVAL;

	err = kstrtouint(buf, 16, &value);
	if (err < 0)
		return err;

	if (value > 0x0ff)
		return -EINVAL;

	/* leave bus active */
	retrode3_drive_half(slot->controller, value, slot->controller->current_address & 1);

	return count;
}
static DEVICE_ATTR_RW(data8);

static ssize_t data16_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	int word;

	if (!retrode3_is_selected(slot))
		return sprintf(buf, "deselected\n");

	word = retrode3_read_word(slot->controller);

	if (word < 0)
		return word;

	return sprintf(buf, "%04x\n", word);
}

static ssize_t data16_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);
	unsigned int value;
	int err;

	if (!retrode3_is_selected(slot))
		return -EINVAL;

	err = kstrtouint(buf, 16, &value);
	if (err < 0)
		return err;

	if (value > 0x0ffff)
		return -EINVAL;

	/* leave bus active */
	retrode3_drive_word(slot->controller, value);

	return count;
}
static DEVICE_ATTR_RW(data16);

static struct attribute *retrode3_attrs[] = {
	&dev_attr_sense.attr,
	&dev_attr_vcc.attr,
	&dev_attr_addr.attr,
	&dev_attr_data8.attr,
	&dev_attr_data16.attr,
	NULL,
};
ATTRIBUTE_GROUPS(retrode3);

/**
 * call if you want to create a /dev/slot-name and automatic /sys entry
 * otherwise call driver_register() directly and drv->driver.bus = &retrode3_bus_type;
 */

int retrode3_slot_driver_register(struct retrode3_slot_driver *drv)
{
	int ret;

	ret = alloc_chrdev_region(&retrode3_first, 0, RETRODE3_MINORS, "retrode3");
	if (ret < 0) {
		pr_err("failed to allocate device numbers: %d\n", ret);
		return ret;
	}

	/* FIXME: this should probably be done only once or for the first driver? */
	retrode3_class = class_create("retrode3");
	if (IS_ERR(retrode3_class)) {
		ret = PTR_ERR(retrode3_class);
		pr_err("failed to create class: %d\n", ret);
		goto err_unregister_chrdev;
	}

	retrode3_class->dev_groups = retrode3_groups;	// for additional /sys attributes

	drv->driver.bus = &retrode3_bus_type;

	ret = /* platform_*/ driver_register(&drv->driver);

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
EXPORT_SYMBOL_GPL(retrode3_slot_driver_register);

void retrode3_slot_driver_unregister(struct retrode3_slot_driver *drv)
{
	class_destroy(retrode3_class);
	unregister_chrdev_region(retrode3_first, RETRODE3_MINORS);
	ida_destroy(&retrode3_minors);

	/* platform_*/ driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(retrode3_slot_driver_unregister);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 core");
