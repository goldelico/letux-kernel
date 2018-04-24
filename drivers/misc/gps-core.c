// SPDX-License-Identifier: GPL-2.0
/*
 * GPS core framework
 *
 * Copyright (C) 2015-2018 H. Nikolaus Schaller <hns@goldelico.com>,
 *						Golden Delicious Computers
 *
 * Its task is to mediate NMEA records between drivers
 * and user-space and tell them when to power on/off
 * the GPS chips.
 *
 * Currently it just opens a /dev/ttyGPS0 which allows
 * for bare communication of raw NMEA records.
 *
 * More sophisticated APIs (e.g. ioctls) can be added in
 * the future.
 */

#ifdef CONFIG_W2SG0004_DEBUG
#define DEBUG 1
#endif

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include "gps-core.h"

/* should become gps_by_minor[n] if we want to support multiple devices */
static struct gps_dev *gps_by_minor;

int gps_recv_nmea_chars(struct gps_dev *gdev, const char *data, int count)
{ /* pass received nmea fragment (not necessarily a full line!) to user-space */
	int n;

	/* simply put into the tty buffer */
	n = tty_insert_flip_string(&gdev->port, data, count);

	if (n != count)
		pr_err("%s: did loose %lu characters\n",
			__func__,
			(unsigned long) (count - n));

	tty_flip_buffer_push(&gdev->port);

	return n;
}
EXPORT_SYMBOL(gps_recv_nmea_chars);

static struct gps_dev *gpsdev_get_by_minor(unsigned int minor)
{
	if (minor >= 1)
		return NULL;

	return gps_by_minor;
}

static int gps_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct gps_dev *data;
	int retval;

	pr_debug("%s() tty = %p\n", __func__, tty);

	data = gpsdev_get_by_minor(tty->index);
	pr_debug("%s() data = %p\n", __func__, data);

	if (!data)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = data;

	return 0;

error_init_termios:
	tty_port_put(&data->port);
	return retval;
}

static int gps_tty_open(struct tty_struct *tty, struct file *file)
{
	struct gps_dev *data = tty->driver_data;

	pr_debug("%s() data = %p open_count = ++%d\n", __func__,
		 data, data->open_count);

	if (data->open_count == 0 && data->open)
		(void) data->open(data);	/* notify first open */

	data->open_count++;

	return tty_port_open(&data->port, tty, file);
}

static void gps_tty_close(struct tty_struct *tty, struct file *file)
{
	struct gps_dev *data = tty->driver_data;

	pr_debug("%s()\n", __func__);

	if (data->open_count > 0)
		data->open_count--;
	
	if (data->open_count == 0 && data->close)
		(void) data->close(data);	/* notify last close */

	tty_port_close(&data->port, tty, file);
}

static int gps_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{ /* send raw data */
	struct gps_dev *data = tty->driver_data;

	if (!data->send)
		return -EIO;

	return data->send(data, buffer, count);
}

static const struct tty_operations gps_serial_ops = {
	.install = gps_tty_install,
	.open = gps_tty_open,
	.close = gps_tty_close,
	.write = gps_tty_write,
};

static const struct tty_port_operations gps_port_ops = {
	/* none defined, but we need the struct */
};

int gps_register_dev(struct gps_dev *gdev, const char *name)
{
	int err;
	int minor;

	pr_debug("%s()\n", __func__);

	/*
	 * For future consideration:
	 * for multiple such GPS receivers in one system
	 * we need a mechanism to define distinct minor values
	 * and search for an unused slot.
	 */
	minor = 0;
	if (gpsdev_get_by_minor(minor)) {
		pr_err("gps minor is already in use!\n");
		return -ENODEV;
	}

	pr_debug("gps alloc_tty_driver\n");

	/* allocate the tty driver */
	gdev->tty_drv = alloc_tty_driver(1);
	if (!gdev->tty_drv)
		return -ENOMEM;

	/* initialize the tty driver */
	gdev->tty_drv->owner = THIS_MODULE;
	gdev->tty_drv->driver_name = name;
	gdev->tty_drv->name = "ttyGPS";
	gdev->tty_drv->major = 0;
	gdev->tty_drv->minor_start = 0;
	gdev->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	gdev->tty_drv->subtype = SERIAL_TYPE_NORMAL;
	gdev->tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	gdev->tty_drv->init_termios = tty_std_termios;
	gdev->tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD |
					      HUPCL | CLOCAL;
	/*
	 * optional:
	 * tty_termios_encode_baud_rate(&gdev->tty_drv->init_termios,
					115200, 115200);
	 * tty_termios(&gdev->tty_drv->init_termios);
	 */
	tty_set_operations(gdev->tty_drv, &gps_serial_ops);

	pr_debug("gps tty_register_driver\n");

	/* register the tty driver */
	err = tty_register_driver(gdev->tty_drv);
	if (err) {
		pr_err("%s - tty_register_driver failed(%d)\n",
			__func__, err);
		put_tty_driver(gdev->tty_drv);
		return err;
	}

	/* minor (0) is now in use */
	gps_by_minor = gdev;

	tty_port_init(&gdev->port);
	gdev->port.ops = &gps_port_ops;

	pr_debug("gps call tty_port_register_device\n");

	gdev->dev = tty_port_register_device(&gdev->port,
			gdev->tty_drv, minor, gdev->dev);

	pr_debug("gps tty_port_register_device -> %p\n", gdev->dev);
	pr_debug("gps port.tty = %p\n", gdev->port.tty);

	pr_debug("gps probed\n");

	return 0;

}
EXPORT_SYMBOL(gps_register_dev);

void gps_unregister_dev(struct gps_dev *gdev)
{
	int minor;

	/* we should lookup gdev in gps_by_minor */
	minor = 0;

	tty_unregister_device(gdev->tty_drv, minor);

	tty_unregister_driver(gdev->tty_drv);
}
EXPORT_SYMBOL(gps_unregister_dev);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Simple GPS tty framework driver");
MODULE_LICENSE("GPL v2");
