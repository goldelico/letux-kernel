// SPDX-License-Identifier: GPL-2.0


/*
 * Ideen:
 * auf Device-Tree-Treiber mit .compatible = "openpandora,retrode3" umbauen
 * je Device-Tree-CS-Node ein /dev/cart-n anlegen
 * welcher Treiber ist da am ähnlichsten?
 *     /dev/gpiochip0, /dev/i2c-1, /dev/rtc0
 * cart-detect mit integrieren, incl. polling?
 * und /dev nur bei Bedarf anlegen oder löschen?
 * kann man dann ein udev-event abfragen und LEDs sowie Auslesen steuern?
 * wie steuert man RAM?
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
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif

#define DEVMEM_MINOR	1
#define DEVPORT_MINOR	4

struct retrode3_cart {
	struct retrode3_data *data;	// backpointer
	struct gpio_desc *cs;	// cart select
	struct gpio_desc *cd;	// cart detect
	int addr_width;
	int bus_width;
	int cd_state;	// previous cart detect state
};

struct retrode3_data {
	struct gpio_descs *addrs;
	struct gpio_descs *datas;
	struct gpio_desc *oe;
	struct gpio_descs *we;
	struct gpio_desc *time;
	struct retrode3_cart carts[4];
};

static void set_address(struct retrode3_data *r3, u32 addr)
{ /* set address on all gpios */
	int a;
// A0 anders behandeln?
	for (a = 0; a < r3->addrs->ndescs; a++)
		gpiod_set_value(r3->addrs->desc[a], (addr>>a) & 1);
}

// block wise read/write?

static u16 read_data(struct retrode3_data *r3)
{ /* read data from data lines */
	int d;
	u16 data;
	gpiod_set_value(r3->oe, true);
	/* read data bits */
	data = 0;
	for (d = 0; d < r3->datas->ndescs; d++)
		data |= gpiod_get_value(r3->datas->desc[d]) << d;
	gpiod_set_value(r3->oe, false);
	return data;
}

static void write_data(struct retrode3_data *r3, u16 data, int mode)
{ /* write data to data lines */
	int d;
	/* set data bits */
	data = 0;
	for (d = 0; d < r3->datas->ndescs; d++)
		gpiod_set_value(r3->datas->desc[d], (data>>d) & 1);
	/* single pulse on upper/lower or both depending on mode! */
	gpiod_set_value(r3->we->desc[0], true);
	gpiod_set_value(r3->we->desc[0], false);
}

static void select(struct retrode3_data *r3, int device)
{
// immer nur eines oder keines...
	/* chip select */
	int i;
	for(i=0; i<ARRAY_SIZE(r3->carts); i++)
		gpiod_set_value(r3->carts[i].cs, (i == device) ? true:false);
}

/*
 * This function reads the cart. The f_pos points directly to the
 * memory location.
 */
static ssize_t read_mem(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct retrode3_data *r3;	// get from *file
	ssize_t read, sz;
	void *ptr;
	char *bounce;
	int err;

	read = 0;

// select specific cart and deselect others

	select(r3, 0);

	while (count > 0) {
		unsigned long remaining;
		int allowed, probe;

		probe = copy_from_kernel_nofault(bounce, ptr, sz);
		remaining = copy_to_user(buf, bounce, sz);

// FIXME: loop over bytes and read them from consecutive addresses
// handle words for 16 bit bus and faster read

		set_address(r3, *ppos);

		if (remaining)
			goto failed;

		buf += sz;
		count -= sz;
		read += sz;
	}

	select(r3, -1);

	*ppos += read;
	return read;

failed:
	return err;
}

static ssize_t write_mem(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct retrode3_data *r3;	// get from *file
	ssize_t written, sz;
	unsigned long copied;
	void *ptr;

	written = 0;

	select(r3, 0);

	while (count > 0) {
		int allowed;

// FIXME: loop over bytes and write them to consecutive addresses
// handle words for 16 bit bus and faster write

		set_address(r3, *ppos);


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

	select(r3, -1);

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

#if OLD

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
	cs = devm_gpiod_get_array(struct device *dev,
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
#endif

static int retrode3_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct retrode3_data *data;
        int i;

        pr_debug("%s()\n", __func__);

        if (!pdev->dev.of_node) {
                dev_err(&pdev->dev, "No device tree data\n");
                return EINVAL;
        }

        data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
        if (data == NULL)
                return -ENOMEM;

	data->addrs = devm_gpiod_get_array(&pdev->dev, "addr", GPIOD_OUT_HIGH);
	data->datas = devm_gpiod_get_array(&pdev->dev, "data", GPIOD_OUT_HIGH);
	data->oe = devm_gpiod_get(&pdev->dev, "oe", GPIOD_OUT_LOW);
// FIXME - one per child:	data->cs = devm_gpiod_get_array(&pdev->dev, "cs", GPIOD_OUT_LOW);
// FIXME - one per child:	data->cd = devm_gpiod_get_array(&pdev->dev, "cd", GPIOD_OUT_LOW);
	data->we = devm_gpiod_get_array(&pdev->dev, "we", GPIOD_OUT_LOW);
	data->time = devm_gpiod_get(&pdev->dev, "time", GPIOD_OUT_LOW);

// FIXME: scan carts and create data structures
// setup chip cart detect polling

#if 0
        for (i=0; i<NUMBER_OF_GPIOS; i++) {
                enum of_gpio_flags flags;

                data->gpios[i] = of_get_named_gpio_flags(dev->of_node,
                                        "gpios", i,
                                        &flags);

                if (data->gpios[i] == -EPROBE_DEFER)
                        return -EPROBE_DEFER;   /* defer until we have all gpios */
                data->gpioflags[i] = flags;
        }

        data->uart = devm_serial_get_uart_by_phandle(&pdev->dev, "uart", 0);

        if (!data->uart) {
                dev_err(&pdev->dev, "No UART link\n");
                return -EINVAL;
        }

        if (IS_ERR(data->uart)) {
                if (PTR_ERR(data->uart) == -EPROBE_DEFER)
                        return -EPROBE_DEFER;   /* we can't probe yet */
                data->uart = NULL;      /* no UART */
        }


        for (i=0; i<NUMBER_OF_GPIOS; i++)
                if(gpio_is_valid(data->gpios[i]))
                        devm_gpio_request(&pdev->dev, data->gpios[i], "mctrl-gpio");
#endif

        platform_set_drvdata(pdev, data);

        dev_info(&pdev->dev, "retrode3 probed\n");

        return 0;
}

static int retrode3_remove(struct platform_device *pdev)
{
        struct retrode3_data *data = platform_get_drvdata(pdev);

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

module_platform_driver(retrode3_driver);

MODULE_ALIAS("retrode3");

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("retrode 3 cart reader driver");
MODULE_LICENSE("GPL v2");
