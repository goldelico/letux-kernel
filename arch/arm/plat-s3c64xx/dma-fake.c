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

int s3c2410_dma_enqueue(unsigned int channel, void *id,
			dma_addr_t data, int size)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_enqueue);

int s3c2410_dma_devconfig(int channel,
			  enum s3c2410_dmasrc source,
			  unsigned long devaddr)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_devconfig);


int s3c2410_dma_getposition(unsigned int channel, dma_addr_t *src, dma_addr_t *dst)
{
	if (src != NULL)
 		*src = 0;

 	if (dst != NULL)
 		*dst = 0;

 	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_getposition);

int s3c2410_dma_config(unsigned int channel, int xferunit)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_config);

int s3c2410_dma_free(unsigned int channel, struct s3c2410_dma_client *client)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_free);

int s3c2410_dma_request(unsigned int channel,
			struct s3c2410_dma_client *client,
			void *dev)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_request);

int s3c2410_dma_set_buffdone_fn(unsigned int channel, s3c2410_dma_cbfn_t rtn)
{
	return 0;
}

EXPORT_SYMBOL(s3c2410_dma_set_buffdone_fn);
