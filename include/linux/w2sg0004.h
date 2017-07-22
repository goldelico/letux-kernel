/*
 * UART slave to allow ON/OFF control of w2sg0004 GPS receiver.
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
 */



#ifndef __LINUX_W2SG0004_H
#define __LINUX_W2SG0004_H

#include <linux/regulator/consumer.h>

struct w2sg_pdata {
	struct regulator *lna_regulator;	/* enable LNA power */
	int	on_off_gpio;	/*  on-off input of the GPS module */
};
#endif /* __LINUX_W2SG0004_H */
