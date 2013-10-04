/*
 * linux/arch/arm/mach-omap2/tvout.c
 *
 * Copyright (C) 2013 Golden Delicious Computers
 * Author: Marek Belisko <marek@goldelico.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/omap-tvout.h>

#include "soc.h"
#include "control.h"

static u16 tvout_reg_offset(void)
{
	if (cpu_is_omap2430())
		return OMAP243X_CONTROL_DEVCONF1;
	else
		return OMAP343X_CONTROL_DEVCONF1;
}

int tvout_read(void)
{
	u16 devconf1_offset = tvout_reg_offset();

	return omap_ctrl_readl(devconf1_offset);
}

void tvout_write(int reg)
{
	u16 devconf1_offset = tvout_reg_offset();
	omap_ctrl_writel(reg, devconf1_offset);
}

