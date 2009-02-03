/*
 * led.h  -- LED driver for NXP PCF50633
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_LED_H
#define __LINUX_MFD_PCF50633_LED_H

#include <linux/platform_device.h>

#define PCF50633_REG_LEDOUT 0x28
#define PCF50633_REG_LEDENA 0x29
#define PCF50633_REG_LEDCTL 0x2a
#define PCF50633_REG_LEDDIM 0x2b

#endif

