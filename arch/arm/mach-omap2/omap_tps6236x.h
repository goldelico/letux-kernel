/**
 * OMAP and TPS PMIC specific headers.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated.
 * Vishwanath BS
 * Moiz Sonasath
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_OMAP_TPS6236X_H
#define __ARCH_ARM_MACH_OMAP2_OMAP_TPS6236X_H

#define TPS62361_GPIO            7

#ifdef CONFIG_OMAP4_TPS62361
int omap4_tps62361_init(void);
#else
static inline int omap4_tps62361_init(void) { return 0; }
#endif  /* CONFIG_OMAP4_TPS62361 */
#endif
