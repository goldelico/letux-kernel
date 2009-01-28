/* Philips PCF50606 GPO Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50606 driver mainly by
 * Harald Welte, Andy Green Werner Almesberger and Matt Hsu
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/kernel.h>

#include <linux/mfd/pcf50606/core.h>
#include <linux/mfd/pcf50606/gpo.h>

void pcf50606_gpo_set_active(struct pcf50606 *pcf, int gpo, int val)
{
	u8 reg, value, mask;

	reg = gpo;
	value = val;
	mask = 0x07;

	if (gpo == PCF50606_GPO2) {
		value = val << 4;
		mask = 0x07 << 4;
	}
	pcf50606_reg_set_bit_mask(pcf, reg, mask, value);
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_set_active);

int pcf50606_gpo_get_active(struct pcf50606 *pcf, int gpo)
{
	u8 reg, value, shift = 0;

	reg = gpo;
	if (gpo == PCF50606_GPO2)
		shift = 4;
	
	value = pcf50606_reg_read(pcf, reg);

	return (value >> shift) & 0x07;
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_get_active);

void pcf50606_gpo_set_standby(struct pcf50606 *pcf, int gpo, int val)
{
	u8 reg;

	if (gpo == PCF50606_GPO1 || gpo == PCF50606_GPO2) {
		dev_err(pcf->dev, "Can't set standby settings for GPO[12]n");
		return;
	}

	reg = gpo;

	pcf50606_reg_set_bit_mask(pcf, gpo, 0x07 << 3, val);
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_set_standby);

int pcf50606_gpo_get_standby(struct pcf50606 *pcf, int gpo)
{
	u8 reg, value;

	if (gpo == PCF50606_GPO1 || gpo == PCF50606_GPO2) {
		dev_err(pcf->dev, "Can't get standby settings for GPO[12]n");
		return -EINVAL;
	}

	reg = gpo;
	value = pcf50606_reg_read(pcf, reg);

	return (value >> 3) & 0x07;
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_get_standby);

void pcf50606_gpo_invert_set(struct pcf50606 *pcf, int gpo, int invert)
{
	u8 reg, value, mask;

	reg = gpo;
	value = !!invert << 6;
	mask = 0x01 << 6;

	if (gpo == PCF50606_GPO1) {
		mask = 0x01 << 4;
		value = !!invert << 4;
	}
	else if (gpo == PCF50606_GPO2) {
		mask = 0x01 << 7;
		value = !!invert << 7;
	}

	pcf50606_reg_set_bit_mask(pcf, reg, mask, value);
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_invert_set);

int pcf50606_gpo_invert_get(struct pcf50606 *pcf, int gpo)
{
	u8 reg, value, shift;

	reg = gpo;
	shift = 6;

	if (gpo == PCF50606_GPO1)
		shift = 4;
	else if (gpo == PCF50606_GPO2)
		shift = 7;

	value =  pcf50606_reg_read(pcf, reg);

	return (value >> shift) & 0x01;
}
EXPORT_SYMBOL_GPL(pcf50606_gpo_invert_get);
