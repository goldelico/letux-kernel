/*
 * opp4xxx.h - macros and tables for OMAP4 OPP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Raajendra Nayak <rnayak@ti.com>
 *
 */

#ifndef __ARCH_ARM_MACH_OMAP2_OPP4XXX_H
#define __ARCH_ARM_MACH_OMAP2_OPP4XXX_H

struct opp_table {
	int	freq;
	int	vsel;
};

#define NO_OF_VDD1_OPP 3
#define NO_OF_VDD2_OPP 3
#define NO_OF_VDD3_OPP 2

#define M300	300000000
#define M600	600000000
#define M720	720000000

#define M233	233000000
#define M466	466000000
#define M554	554000000

#define M200	190464000
#define M400	380928000

int get_vdd1_voltage(int freq);
int get_vdd2_voltage(int freq);
int get_vdd3_voltage(int freq);

#endif
