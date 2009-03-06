/* linux/arch/arm/plat-s3c64xx/dma.c
 *
 * Copyright 2009 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C64XX DMA core - fake
 *
 * http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/dma.h>


int s3c2410_dma_ctrl(unsigned int channel, enum s3c2410_chan_op op)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_ctrl);
