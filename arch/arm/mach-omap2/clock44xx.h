/*
 * OMAP4 clock function prototypes and macros
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK_44XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK_44XX_H

#define OMAP4430_MAX_DPLL_MULT	2048
#define OMAP4430_MAX_DPLL_DIV	128

int omap4_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate);
extern const struct clkops clkops_noncore_dpll_ops;

#endif
