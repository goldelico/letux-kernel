/*
 * wdt.h -- WDT driver for NXP PCF50606
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50606_WDT_H
#define __LINUX_MFD_PCF50606_WDT_H

#define PCF50606_REG_OOCC1 	0x08
#define PCF50606_REG_OOCS 	0x01

#define PCF50606_OOCS_WDTEXP 	0x80
#define PCF50606_OOCC1_WDTRST 	0x08

#define CLOSE_STATE_NOT		0x0000
#define CLOSE_STATE_ALLOW	0x2342

struct pcf50606;

struct pcf50606_wdt {
	struct platform_device *pdev;
};
#endif /* __LINUX_MFD_PCF50606_WDT_H */


