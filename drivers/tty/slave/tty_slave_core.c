/*
 * tty-slave-core - device bus for tty slaves
 *
 * Copyright (C) 2015 NeilBrown <neil@brown.name>
 *
 *    This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

/*
 * A "tty-slave" is a device permanently attached to a particularly
 * tty, typically wired to a UART.
 * A tty-slave has two particular roles.
 * Firstly it can intercept any tty_operations to provide extra control
 * of the device.  For example it might intercept "open" and "close"
 * in order to power the device up and down.  It might intercept
 * "hangup" to toggle a reset line on the device.
 *
 * Secondly it appears as a parent of the tty in the device model, so
 * that any attributes it presents are visible to udev when the tty
 * is added.  This allows udev to start appropriate handlers such as
 * hciattach or inputattach.
 *
 * tty-slave devices must be described in devicetree as a child node
 * of the node which described the parent of the tty, typically a
 * UART.
 * If such a child is present, the tty device will not be registered
 * until the slave device is fully probed and initialized.
 */
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_slave.h>


static int tty_slave_match(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static void tty_slave_release(struct device *dev)
{
	kfree(dev);
}

struct bus_type tty_slave_bus_type = {
	.name		= "tty-slave",
	.match		= tty_slave_match,
};

int tty_slave_register(struct device *parent, struct device_node *node,
		       struct device *tty, struct tty_driver *drv)
{
	struct tty_slave *slave;
	int retval;

	if (!of_get_property(node, "compatible", NULL))
		return -ENODEV;

	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	slave->dev.bus = &tty_slave_bus_type;
	slave->dev.parent = parent;
	slave->dev.release = tty_slave_release;
	slave->dev.of_node = of_node_get(node);
	dev_set_name(&slave->dev, "%s", node->name);
	slave->tty_dev = tty;
	slave->tty_drv = drv;
	slave->ops = *drv->ops;
	retval = device_register(&slave->dev);
	if (retval) {
		of_node_put(node);
		kfree(slave);
	}
	return retval;
}
EXPORT_SYMBOL(tty_slave_register);

void tty_slave_activate(struct tty_struct *tty)
{
	struct device *parent = NULL;
	if (tty->dev)
		parent = tty->dev->parent;
	if (parent &&
	    parent->bus == &tty_slave_bus_type)
	{
		struct tty_slave *dev =
			container_of(parent, struct tty_slave, dev);
		tty->ops = &dev->ops;
	}
}
EXPORT_SYMBOL(tty_slave_activate);

int tty_slave_finalize(struct tty_slave *slave)
{
	slave->tty_dev->parent = &slave->dev;
	return tty_register_finalize(slave->tty_drv,
				     slave->tty_dev);
}
EXPORT_SYMBOL(tty_slave_finalize);

int tty_slave_driver_register(struct device_driver *drv)
{
	drv->bus = &tty_slave_bus_type;
	return driver_register(drv);
}
EXPORT_SYMBOL(tty_slave_driver_register);

static int __init tty_slave_init(void)
{
	return bus_register(&tty_slave_bus_type);
}

static void __exit tty_slave_exit(void)
{
	bus_unregister(&tty_slave_bus_type);
}

postcore_initcall(tty_slave_init);
module_exit(tty_slave_exit);
