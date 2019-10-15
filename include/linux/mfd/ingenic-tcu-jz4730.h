/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Header file for the Ingenic JZ4730 OST driver (corresponding to the TCU)
 */
#ifndef __LINUX_MFD_INGENIC_TCU_JZ4730_H_
#define __LINUX_MFD_INGENIC_TCU_JZ4730_H_

#include <linux/bitops.h>

#define TCU_REG_WDT_TCSR	0x00
#define TCU_REG_WDT_TCNT	0x04

#define TCU_REG_TER		0x00
#define TCU_REG_TDFR0		0x10	/* TRDR */
#define TCU_REG_TCNT0		0x14
#define TCU_REG_TCSR0		0x18

#define TCU_TCSR_RESERVED_BITS		0x3f
#define TCU_TCSR_PARENT_CLOCK_MASK	0x3f

/* No applicable prescaling control in the JZ4730. */

#define TCU_TCSR_PRESCALE_LSB		0
#define TCU_TCSR_PRESCALE_MASK		0x00

#define TCU_WDT_TCER_TCEN	BIT(0)	/* Watchdog timer enable */

#define TCU_CHANNEL_STRIDE	0x20
#define TCU_REG_TDFRc(c)	(TCU_REG_TDFR0 + ((c) * TCU_CHANNEL_STRIDE))
#define TCU_REG_TCNTc(c)	(TCU_REG_TCNT0 + ((c) * TCU_CHANNEL_STRIDE))
#define TCU_REG_TCSRc(c)	(TCU_REG_TCSR0 + ((c) * TCU_CHANNEL_STRIDE))

#endif /* __LINUX_MFD_INGENIC_TCU_JZ4730_H_ */
