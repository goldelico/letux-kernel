// SPDX-License-Identifier: GPL-2.0


/*
 * Ideen:
 * auf Device-Tree-Treiber mit .compatible = "openpandora,retrode3" umbauen
 * je Device-Tree-CS-Node ein /dev/cart-n anlegen
 * welcher Treiber ist da am ähnlichsten?
 *     /dev/gpiochip0, /dev/i2c-1, /dev/rtc0
 */

/*
 * based on
 *  linux/drivers/char/mem.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Added devfs support.
 *    Jan-11-1998, C. Scott Ananian <cananian@alumni.princeton.edu>
 *  Shared /dev/zero mmapping support, Feb 2000, Kanoj Sarcar <kanoj@sgi.com>
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif

#define DEVMEM_MINOR	1
#define DEVPORT_MINOR	4

static struct gpio_descs *addrs;
static struct gpio_descs *datas;
static struct gpio_desc *cs;
static struct gpio_desc *oe;
static struct gpio_desc *we;

static void set_address(u32 addr)
{ /* set address on all gpios */
	int a;
// A0 anders behandeln?
	for (a = 0; a < addrs->ndescs; a++)
		gpiod_set_value(addrs->desc[a], (addr>>a) & 1);
}

static u16 read_data(void)
{ /* read data from data lines */
	int d;
	u16 data;
	gpiod_set_value(oe, 0);
	/* read data bits */
	data = 0;
	for (d = 0; d < datas->ndescs; d++)
		data |= gpiod_get_value(datas->desc[d]) << d;
	gpiod_set_value(oe, 1);
	return data;
}

static void write_data(u16 data, int mode)
{ /* write data to data lines */
	int d;
	/* set data bits */
	data = 0;
	for (d = 0; d < datas->ndescs; d++)
		gpiod_set_value(datas->desc[d], (data>>d) & 1);
	/* single pulse on upper/lower or both */
	gpiod_set_value(we, 0);
	gpiod_set_value(we, 1);
}

static void select(int device)
{
// immer nur eines oder keines...
	/* chip select */
	gpiod_set_value(cs, 0);	/* sollte ausserhalb dieser Funktionen passieren! */

	gpiod_set_value(cs, 1);
}

/*
 * This funcion reads the cart. The f_pos points directly to the
 * memory location.
 */
static ssize_t read_mem(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	ssize_t read, sz;
	void *ptr;
	char *bounce;
	int err;

	read = 0;

	select(0);

	while (count > 0) {
		unsigned long remaining;
		int allowed, probe;

		probe = copy_from_kernel_nofault(bounce, ptr, sz);
		remaining = copy_to_user(buf, bounce, sz);

// FIXME: loop over bytes and read them from consecutive addresses

		set_address(*ppos);

		if (remaining)
			goto failed;

		buf += sz;
		count -= sz;
		read += sz;
	}

	select(-1);

	*ppos += read;
	return read;

failed:
	return err;
}

static ssize_t write_mem(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	ssize_t written, sz;
	unsigned long copied;
	void *ptr;

	written = 0;

	select(0);

	while (count > 0) {
		int allowed;

// FIXME: loop over bytes and write them to consecutive addresses

		set_address(*ppos);


		copied = copy_from_user(ptr, buf, sz);
		if (copied) {
			written += sz - copied;
			if (written)
				break;
			return -EFAULT;
		}

		buf += sz;
		count -= sz;
		written += sz;
	}

	select(-1);

	*ppos += written;
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
static loff_t memory_lseek(struct file *file, loff_t offset, int orig)
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

static int open_mem(struct inode *inode, struct file *filp)
{
	int rc;

	if (iminor(inode) != DEVMEM_MINOR)
		return 0;

#if OLD
	/*
	 * Use a unified address space to have a single point to manage
	 * revocations when drivers want to take over a /dev/mem mapped
	 * range.
	 */
	filp->f_mapping = iomem_get_mapping();
#endif
	return 0;
}


static const struct file_operations __maybe_unused mem_fops = {
	.llseek		= memory_lseek,
	.read		= read_mem,
	.write		= write_mem,
//	.mmap		= mmap_mem,
	.open		= open_mem,
};

static const struct memdev {
	const char *name;
	umode_t mode;
	const struct file_operations *fops;
	fmode_t fmode;
} devlist[] = {
	 [DEVMEM_MINOR] = { "retrode", 0, &mem_fops, FMODE_UNSIGNED_OFFSET },
};

static int memory_open(struct inode *inode, struct file *filp)
{
	int minor;
	const struct memdev *dev;

	minor = iminor(inode);
	if (minor >= ARRAY_SIZE(devlist))
		return -ENXIO;

	dev = &devlist[minor];
	if (!dev->fops)
		return -ENXIO;

	filp->f_op = dev->fops;
	filp->f_mode |= dev->fmode;

	if (dev->fops->open)
		return dev->fops->open(inode, filp);

	return 0;
}

static const struct file_operations memory_fops = {
	.open = memory_open,
	.llseek = noop_llseek,
};

static char *mem_devnode(struct device *dev, umode_t *mode)
{
	if (mode && devlist[MINOR(dev->devt)].mode)
		*mode = devlist[MINOR(dev->devt)].mode;
	return NULL;
}

static struct class *mem_class;

static int __init chr_dev_init(void)
{
	int minor;

// FIXME: we should have multiple instances with different names, i.e. /dev/cart0 /dev/cart1 /dev/cart2
/*
	addrs = devm_gpiod_get_array(struct device *dev,
		"addr", GPIOD_OUT_HIGH);
	datas = devm_gpiod_get_array(struct device *dev,
		"data", GPIOD_OUT_HIGH);
	cs = devm_gpiod_get(struct device *dev,
		"cs", GPIOD_OUT_HIGH);
	oe = devm_gpiod_get(struct device *dev,
		"cs", GPIOD_OUT_HIGH);

*/

	if (register_chrdev(MEM_MAJOR, "cart", &memory_fops))
		printk("unable to get major %d for retrode dev\n", MEM_MAJOR);

	mem_class = class_create(THIS_MODULE, "cart");
	if (IS_ERR(mem_class))
		return PTR_ERR(mem_class);

	mem_class->devnode = mem_devnode;
	for (minor = 1; minor < ARRAY_SIZE(devlist); minor++) {
		if (!devlist[minor].name)
			continue;

		/*
		 * Create /dev/port?
		 */
		if ((minor == DEVPORT_MINOR) && !arch_has_dev_port())
			continue;

		device_create(mem_class, NULL, MKDEV(MEM_MAJOR, minor),
			      NULL, devlist[minor].name);
	}

	return 0;
}

fs_initcall(chr_dev_init);

MODULE_LICENSE("GPL v2");
