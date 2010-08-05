/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board-zoom.h>

#include <plat/common.h>
#include <plat/board.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "omap3-opp.h"

/* Added for FlexST */
#include "board-connectivity.h"

#include "board-zoom2-wifi.h"

static void __init omap_zoom_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

static struct omap_board_config_kernel zoom_config[] __initdata = {
};

static struct mtd_partition zoom_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= 0x0080000,
		.size		= 0x0180000, /* 1.5 M */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= 0x01C0000,
		.size		= 0x0040000,
	},
	{
		.name		= "Kernel-NAND",
		.offset		= 0x0200000,
		.size		= 0x1C00000,   /* 30M */
	},
#ifdef CONFIG_ANDROID
	{
		.name		= "system",
		.offset		= 0x2000000,
		.size		= 0xB400000,   /* 180M */
	},
	{
		.name		= "userdata",
		.offset		= 0xD400000,
		.size		= 0x4000000,    /* 64M */
	},
	{
		.name		= "cache",
		.offset		= 0x11400000,
		.size		= 0x2000000,    /* 32M */
	},
#endif
};

static struct flash_partitions zoom_flash_partitions[] = {
	{
		.parts = zoom_nand_partitions,
		.nr_parts = ARRAY_SIZE(zoom_nand_partitions),
	},
};

static void __init omap_zoom_init_irq(void)
{
	omap_board_config = zoom_config;
	omap_board_config_size = ARRAY_SIZE(zoom_config);
	omap_init_irq();
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params,
			     omap3630_mpu_rate_table,
			     omap3630_dsp_rate_table,
			     omap3630_l3_rate_table);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap_zoom_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	zoom_peripherals_init();
	zoom_flash_init(zoom_flash_partitions, ZOOM_NAND_CS);
#ifdef CONFIG_TIWLAN_SDIO
	config_wlan_mux();
#endif
	zoom_debugboard_init();
	conn_board_init(); /* wl127x VIO leakage fix*/
	conn_add_plat_device(); /* Added for FlexST */
	conn_config_gpios(); /* configure BT,FM and GPS gpios */
}

MACHINE_START(OMAP_ZOOM3, "OMAP Zoom3 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_zoom_map_io,
	.init_irq	= omap_zoom_init_irq,
	.init_machine	= omap_zoom_init,
	.timer		= &omap_timer,
MACHINE_END
