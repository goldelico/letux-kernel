// SPDX-License-Identifier: GPL-2.0

/*
 * based on ideas and fragents from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c
 *
 *  Copyright (C) 2022-23, H. Nikolaus Schaller
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3_slot.h"

/* bus for all slots */

struct retrode3_bus {
	struct gpio_descs *addrs;	// addr-gpios
	struct gpio_descs *datas;	// data-gpios
	struct gpio_descs *we;		// we-gpios
	struct gpio_desc *oe;		// oe-gpio
	struct gpio_desc *time;		// time-gpio
	struct gpio_desc *reset;	// reset-gpio
	struct retrode3_slot *slots[4];
	struct mutex select_lock;	// used by select_slot
	int a0;
	u32 prev_addr;
};

/* access to cart bus */

// set the address on the bus
static int set_address(struct retrode3_bus *bus, u32 addr);

// read a byte from the bus (taking A0 into account)
static int read_byte(struct retrode3_bus *bus);

// read a word from the bus (ignoring A0)
static int read_word(struct retrode3_bus *bus);

// write a word - potentially a byte
static void write_word(struct retrode3_bus *bus, u16 data, int mode);
