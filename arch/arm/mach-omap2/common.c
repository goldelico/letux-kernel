/*
 * linux/arch/arm/mach-omap2/common.c
 *
 * Code common to all OMAP2+ machines.
 *
 * Copyright (C) 2009 Texas Instruments
 * Copyright (C) 2010 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_data/dsp-omap.h>
#include <linux/platform_data/remoteproc-omap.h>
#include <linux/memblock.h>

#include "common.h"
#include "omap-secure.h"

#ifdef CONFIG_RADIO_CMEM_BUF
/*
 * Define the CMEM base address explicitly.
 * Consider system memory map before changing this address
 */
#define DRA7_PHYS_ADDR_CMEM_BASE	(0x95400000)
#define DRA7_CMEM_SIZE			(SZ_4M)

#endif

/*
 * Stub function for OMAP2 so that common files
 * continue to build when custom builds are used
 */
int __weak omap_secure_ram_reserve_memblock(void)
{
	return 0;
}

void __init omap4_reserve(void)
{
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
	omap_reserve();
}

void __init omap5_reserve(void)
{
	omap_rproc_reserve_cma(RPROC_CMA_OMAP5);
	omap_reserve();
}

void __init dra7_reserve(void)
{
#ifdef CONFIG_RADIO_CMEM_BUF
	/* Reserve memory for CMEM pool used by Radio application */
	if (memblock_remove(DRA7_PHYS_ADDR_CMEM_BASE, DRA7_CMEM_SIZE))
		pr_err("Failed to reserve memory pool for CMEM Allocator\n");
#endif
	omap_rproc_reserve_cma(RPROC_CMA_DRA7);
	omap_reserve();
}

void __init omap_reserve(void)
{
	omap_dsp_reserve_sdram_memblock();
	omap_secure_ram_reserve_memblock();
	omap_barrier_reserve_memblock();
}
