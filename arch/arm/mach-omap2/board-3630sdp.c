/*
 * Copyright (C) 2009 Texas Instruments Inc.
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

#include <plat/common.h>
#include <plat/board.h>
#include <plat/gpmc-smc91x.h>
#include <plat/mux.h>
#include <plat/usb.h>

#include <mach/board-zoom.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"

#if defined(CONFIG_SMC91X) || defined(CONFIG_SMC91X_MODULE)

static struct omap_smc91x_platform_data board_smc91x_data = {
	.cs             = 3,
	.flags          = GPMC_MUX_ADD_DATA | IORESOURCE_IRQ_LOWLEVEL,
};

static void __init board_smc91x_init(void)
{
	board_smc91x_data.gpio_irq = 158;
	gpmc_smc91x_init(&board_smc91x_data);
}

#else

static inline void board_smc91x_init(void)
{
}

#endif /* defined(CONFIG_SMC91X) || defined(CONFIG_SMC91X_MODULE) */

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

/*-------------------------------------------------------------*/
/*  EHCI FUNCTIONS */

#define EXT_PHY_RESET_GPIO_PORT1 126
#define EXT_PHY_RESET_GPIO_PORT2 61
static int default_usb_port_startup(struct platform_device *dev, int port)
{
	int r;
	int gpio;
	const char *name;

	if (port == 0) {
		gpio = EXT_PHY_RESET_GPIO_PORT1;
		name = "ehci port 1 reset";
	} else if (port == 1) {
		gpio = EXT_PHY_RESET_GPIO_PORT2;
		name = "ehci port 2 reset";
	} else {
		return -EINVAL;
	}

	r = gpio_request(gpio, name);
	if (r < 0) {
		printk(KERN_WARNING "Could not request GPIO %d"
		       " for port %d reset\n",
		       gpio, port);
		return r;
	}
	gpio_direction_output(gpio, 1);
	return 0;
}

static void default_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 0)
		gpio_free(EXT_PHY_RESET_GPIO_PORT1);
	else if (port == 1)
		gpio_free(EXT_PHY_RESET_GPIO_PORT2);
}

static void default_usb_port_reset(struct platform_device *dev,
				   int port, int reset)
{
	if (port == 0)
		gpio_set_value(EXT_PHY_RESET_GPIO_PORT1, !reset);
	else if (port == 1)
		gpio_set_value(EXT_PHY_RESET_GPIO_PORT2, !reset);
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_data[0].mode = EHCI_HCD_OMAP_MODE_ULPI_PHY,
	.port_data[1].mode = EHCI_HCD_OMAP_MODE_ULPI_PHY,
	.port_data[2].mode = -EINVAL,

	.port_data[0].reset_delay = 10,
	.port_data[1].reset_delay = 10,
	.port_data[2].reset_delay = 10,

	.port_data[0].shutdown = &default_usb_port_shutdown,
	.port_data[1].shutdown = &default_usb_port_shutdown,
	.port_data[2].shutdown = NULL,

	.port_data[0].reset = &default_usb_port_reset,
	.port_data[1].reset = &default_usb_port_reset,
	.port_data[2].reset = NULL,

	.port_data[0].startup = &default_usb_port_startup,
	.port_data[1].startup = &default_usb_port_startup,
	.port_data[2].startup = NULL,

};
/*-------------------------------------------------------------*/

static void __init omap_sdp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

static struct omap_board_config_kernel sdp_config[] __initdata = {
};

static void __init omap_sdp_init_irq(void)
{
	omap_board_config = sdp_config;
	omap_board_config_size = ARRAY_SIZE(sdp_config);
	omap_init_irq();
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params,
                             NULL, NULL, NULL);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap_sdp_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	zoom_peripherals_init();
	board_smc91x_init();
	enable_board_wakeup_source();

	omap_mux_init_signal("gpio_126", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpio_61", OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);
}

MACHINE_START(OMAP_3630SDP, "OMAP 3630SDP board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_sdp_map_io,
	.init_irq	= omap_sdp_init_irq,
	.init_machine	= omap_sdp_init,
	.timer		= &omap_timer,
MACHINE_END
