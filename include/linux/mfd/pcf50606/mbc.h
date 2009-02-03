/*
 * mbc.h  -- Driver for NXP PCF50606 Main Battery Charger
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50606_MBC_H
#define __LINUX_MFD_PCF50606_MBC_H

#include <linux/platform_device.h>

#define PCF50606_REG_MBCC1	 0x29
#define PCF50606_REG_MBCC2	 0x2a
#define PCF50606_REG_MBCC3	 0x2b
#define PCF50606_REG_MBCS1	 0x2c

enum pcf50606_reg_mbcc1 {
	PCF50606_MBCC1_CHGAPE		= 0x01,
	PCF50606_MBCC1_AUTOFST		= 0x02,
#define	PCF50606_MBCC1_CHGMOD_MASK	  0x1c
#define	PCF50606_MBCC1_CHGMOD_SHIFT	  2
	PCF50606_MBCC1_CHGMOD_QUAL	= 0x00,
	PCF50606_MBCC1_CHGMOD_PRE	= 0x04,
	PCF50606_MBCC1_CHGMOD_TRICKLE	= 0x08,
	PCF50606_MBCC1_CHGMOD_FAST_CCCV	= 0x0c,
	PCF50606_MBCC1_CHGMOD_FAST_NOCC	= 0x10,
	PCF50606_MBCC1_CHGMOD_FAST_NOCV	= 0x14,
	PCF50606_MBCC1_CHGMOD_FAST_SW	= 0x18,
	PCF50606_MBCC1_CHGMOD_IDLE	= 0x1c,
	PCF50606_MBCC1_DETMOD_LOWCHG	= 0x20,
	PCF50606_MBCC1_DETMOD_WDRST	= 0x40,
};

struct pcf50606;

void pcf50606_mbc_usb_curlim_set(struct pcf50606 *pcf, int ma);

struct pcf50606_mbc {
	int charger_online;
	int charger_active;

	struct power_supply charger;

	struct platform_device *pdev;
};
#endif

