/*
 * input.h  -- Input driver for NXP PCF50633
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_INPUT_H
#define __LINUX_MFD_PCF50633_INPUT_H

#include <linux/platform_device.h>
#include <linux/input.h>

#define PCF50633_OOCSTAT_ONKEY	0x01
#define PCF50633_REG_OOCSTAT	0x12
#define PCF50633_REG_OOCMODE	0x10

struct pcf50633_input {
	struct input_dev *input_dev;
	struct platform_device *pdev;
};

#endif

