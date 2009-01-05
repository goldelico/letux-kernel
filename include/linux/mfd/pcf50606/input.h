/*
 * input.h  -- Input driver for NXP PCF50606
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50606_INPUT_H
#define __LINUX_MFD_PCF50606_INPUT_H

#include <linux/platform_device.h>
#include <linux/input.h>

#define PCF50606_OOCS_ONKEY	 0x01
#define PCF50606_OOCS_EXTON	 0x02

#define PCF50606_OOCC2_ONKEYDB_NONE	 0x00
#define PCF50606_OOCC2_ONKEYDB_14ms	 0x01
#define PCF50606_OOCC2_ONKEYDB_62ms	 0x02
#define PCF50606_OOCC2_ONKEYDB_500ms	 0x03
#define PCF50606_OOCC2_EXTONDB_NONE	 0x00
#define PCF50606_OOCC2_EXTONDB_14ms	 0x04
#define PCF50606_OOCC2_EXTONDB_62ms	 0x08
#define PCF50606_OOCC2_EXTONDB_500ms	 0x0c

struct pcf50606_input {
	struct input_dev *input_dev;
	struct platform_device *pdev;
};

#endif

