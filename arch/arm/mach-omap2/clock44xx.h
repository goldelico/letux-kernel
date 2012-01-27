/*
 * OMAP4 clock function prototypes and macros
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK44XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK44XX_H

#define OMAP4430_REGM4XEN_MULT 4

int omap4xxx_clk_init(void);
int omap4_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate);
long omap4_dpll_regm4xen_round_rate(struct clk *clk, unsigned long target_rate);
unsigned long omap4_dpll_regm4xen_recalc(struct clk *clk);
int omap4_core_dpll_m5x2_set_rate(struct clk *clk, unsigned long rate);

#endif
