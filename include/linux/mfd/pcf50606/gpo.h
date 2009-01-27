/*
 * gpo.h -- GPO driver for NXP PCF50606
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50606_GPO_H
#define __LINUX_MFD_PCF50606_GPO_H

#include <linux/mfd/pcf50633/core.h>

#define PCF50606_REG_GPOC1 0x38
#define PCF50606_REG_GPOC2 0x39
#define PCF50606_REG_GPOC3 0x3a
#define PCF50606_REG_GPOC4 0x3b
#define PCF50606_REG_GPOC5 0x3c

#define PCF50606_GPO1	PCF50606_REG_GPOC1
#define PCF50606_GPO2	PCF50606_REG_GPOC1
#define PCF50606_GPOOD1	PCF50606_REG_GPOC2
#define PCF50606_GPOOD2	PCF50606_REG_GPOC3
#define PCF50606_GPOOD3	PCF50606_REG_GPOC4
#define PCF50606_GPOOD4	PCF50606_REG_GPOC5

#define PCF50606_GPOCFG_GPOSEL_MASK	0x07

void pcf50606_gpo_set_active(struct pcf50606 *pcf, int gpo, int value);
int pcf50606_gpo_get_active(struct pcf50606 *pcf, int gpo);
void pcf50606_gpo_set_standby(struct pcf50606 *pcf, int gpo, int value);
int pcf50606_gpo_get_standby(struct pcf50606 *pcf, int gpo);

void pcf50606_gpo_invert_set(struct pcf50606 *, int gpo, int invert);
int pcf50606_gpo_invert_get(struct pcf50606 *pcf, int gpo);

#endif /* __LINUX_MFD_PCF50606_GPIO_H */

