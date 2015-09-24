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
#include <linux/reset.h>

#include "common.h"
#include "omap-secure.h"
#include "soc.h"

static struct reset_control *omap_reset_control;

/*
 * Stub function for OMAP2 so that common files
 * continue to build when custom builds are used
 */
int __weak omap_secure_ram_reserve_memblock(void)
{
	return 0;
}

void __init omap_reserve(void)
{
	omap_secure_ram_reserve_memblock();
	omap_barrier_reserve_memblock();
}

void omap_restart(enum reboot_mode mode, const char *cmd)
{
	if (omap_reset_control)
		reset_control_assert(omap_reset_control);

	while (1)
		;
}

static int __init omap_init_restart(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "system_reset");
	omap_reset_control = of_reset_control_get(np, "system");
	if (IS_ERR(omap_reset_control)) {
		pr_err("%s: no reset controller, reboot not functional.\n",
		       __func__);
		omap_reset_control = NULL;
	}

	return 0;
}
omap_late_initcall(omap_init_restart);
