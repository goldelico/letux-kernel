/*
 * OMAP4 OPP management functions
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Rajendra Nayak (rnayak@ti.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include "opp4xxx.h"

struct opp_table omap4_vdd1_table[NO_OF_VDD1_OPP] = {
	{.freq	= M300, .vsel = 0x30 },
	{.freq	= M600, .vsel = 0x39 },
	{.freq	= M720, .vsel = 0x39 },
};

struct opp_table omap4_vdd2_table[NO_OF_VDD2_OPP] = {
	{.freq	= M233, .vsel = 0x30 },
	{.freq	= M466, .vsel = 0x39 },
	{.freq	= M554, .vsel = 0x39 },
};

struct opp_table omap4_vdd3_table[NO_OF_VDD3_OPP] = {
	{.freq	= M200, .vsel = 0x30 },
	{.freq	= M400, .vsel = 0x39 },
};

int get_vdd1_voltage(int freq)
{
	int i;

	for (i = 0; i < NO_OF_VDD1_OPP; i++)
		if (omap4_vdd1_table[i].freq == freq)
			return omap4_vdd1_table[i].vsel;

	return -EINVAL;
}

int get_vdd2_voltage(int freq)
{
	int i;

	for (i = 0; i < NO_OF_VDD2_OPP; i++)
		if (omap4_vdd2_table[i].freq == freq)
			return omap4_vdd2_table[i].vsel;

	return -EINVAL;
}

int get_vdd3_voltage(int freq)
{
	int i;

	for (i = 0; i < NO_OF_VDD3_OPP; i++)
		if (omap4_vdd3_table[i].freq == freq)
			return omap4_vdd3_table[i].vsel;

	return -EINVAL;
}

