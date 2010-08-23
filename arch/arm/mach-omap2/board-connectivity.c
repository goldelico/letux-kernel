/*
 * linux/arch/arm/mach-omap2/board-connectivity.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt)     "(wl12xx): " fmt
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <mach/gpio.h>

#include "mux.h"
#include <asm/io.h>
#include <asm/delay.h>

#include "board-connectivity.h"

#if defined(CONFIG_MACH_OMAP_ZOOM3)\
	|| defined(CONFIG_MACH_OMAP_ZOOM2)
#ifdef CONFIG_MACH_OMAP_FST_WL127x
#define CONFIG_MACH_OMAP_FST_OMAP3_127x
#warning zoom2 or zoom3 with wl127x
#else
#define CONFIG_MACH_OMAP_FST_OMAP3_128x
#warning  zoom2 or zoom3 with wl128x
#endif

#elif defined(CONFIG_MACH_OMAP_4430SDP)
#ifdef CONFIG_MACH_OMAP_FST_WL127x
#define CONFIG_MACH_OMAP_FST_OMAP4_127x
#warning omap4 sdp with wl127x
#else
#define CONFIG_MACH_OMAP_FST_OMAP4_128x
#warning omap4 sdp with wl128x
#endif
#endif

/* GPIOS need to be in order of BT, FM and GPS;
 * provide -1 is if EN GPIO not applicable for a core */
#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
#define BT_EN_GPIO 109
#define FM_EN_GPIO 161
#define GPS_EN_GPIO -1
#define McBSP3_BT_GPIO 164
#elif defined(CONFIG_MACH_OMAP_FST_OMAP4_128x)\
	|| defined(CONFIG_MACH_OMAP_FST_OMAP4_127x)
#define BT_EN_GPIO 55
#define FM_EN_GPIO -1
#define GPS_EN_GPIO -1
#else
#define BT_EN_GPIO -1
#define FM_EN_GPIO -1
#define GPS_EN_GPIO -1
#endif

static int conn_gpios[] = { BT_EN_GPIO, FM_EN_GPIO, GPS_EN_GPIO };

static struct platform_device conn_device = {
	.name = "kim",		/* named after init manager for ST */
	.id = -1,
	.dev.platform_data = &conn_gpios,
};

static struct platform_device *conn_plat_devices[] __initdata = {
	&conn_device,
};

static void config_bt_mux_gpio(void)
{
	/* Configure Connectivity GPIOs */
	if (BT_EN_GPIO != -1)
		omap_mux_init_gpio(BT_EN_GPIO, OMAP_PIN_OUTPUT);
	if (FM_EN_GPIO != -1)
		omap_mux_init_gpio(FM_EN_GPIO, OMAP_PIN_OUTPUT);
	if (GPS_EN_GPIO != -1)
		omap_mux_init_gpio(GPS_EN_GPIO, OMAP_PIN_OUTPUT);

	return;
}

static void config_mux_mcbsp3(void)
{
	/*Mux setting for GPIO164 McBSP3 */
#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
	omap_mux_init_gpio(McBSP3_BT_GPIO, OMAP_PIN_OUTPUT);
#endif
	return;
}

#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
/* Fix to prevent VIO leakage on wl127x */
static int wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	pr_info(" wl127x_vio_leakage_fix\n");

	ret = gpio_request(conn_gpios[0], "wl127x_bten");
	if (ret < 0) {
		pr_err("wl127x_bten gpio_%d request fail",
				conn_gpios[0]);
		goto fail;
	}

	gpio_direction_output(conn_gpios[0], 1);
	mdelay(10);
	gpio_direction_output(conn_gpios[0], 0);
	udelay(64);

	gpio_free(conn_gpios[0]);
fail:
	return ret;
}
#endif /* MACH_OMAP_FST_OMAP3_127x */

void conn_config_gpios(void)
{
	pr_info(" Configuring Connectivity GPIOs\n");

	config_bt_mux_gpio();
	config_mux_mcbsp3();
}

void __init conn_add_plat_device(void)
{
	pr_info(" Adding Connectivity platform device\n");

	platform_add_devices(conn_plat_devices, ARRAY_SIZE(conn_plat_devices));
}

void conn_board_init(void)
{
	pr_info(" Connectivity board init\n");

#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
	pr_info(" Connectivity board init for OMAP3+WL127x\n");

	wl127x_vio_leakage_fix();
#endif
}
