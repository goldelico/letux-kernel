/*
 * Virtual gpio to allow ON/OFF control of w2sg0004 GPS receiver.
 *
 * Copyright (C) 2011 Neil Brown <neil@brown.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */



#ifndef __LINUX_W2SG0004_H
#define __LINUX_W2SG0004_H

#include <linux/regulator/consumer.h>

struct w2sg_pdata {
	int	gpio_base;		/* (not used by DT) - defines the gpio.base */
	struct regulator *lna_regulator;	/* enable LNA power */
	int	on_off_gpio;	/* connected to the on-off input of the GPS module */
};
#endif /* __LINUX_W2SG0004_H */
