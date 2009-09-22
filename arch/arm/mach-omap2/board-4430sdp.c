/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/usb/otg.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/mux.h>

#include <plat/mcspi.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/usb.h>
#include <plat/syntm12xx.h>
#include <plat/keypad.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>

static int ts_gpio;

#define OMAP4_KBDOCP_BASE               0x4A31C000

static int omap_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(0, 1, KEY_D),
	KEY(0, 2, KEY_X),
	KEY(0, 3, KEY_Z),
	KEY(0, 4, KEY_W),
	KEY(0, 5, KEY_S),
	KEY(0, 6, KEY_Q),
	KEY(0, 7, KEY_UNKNOWN),

	KEY(1, 0, KEY_R),
	KEY(1, 1, KEY_F),
	KEY(1, 2, KEY_C),
	KEY(1, 3, KEY_KPPLUS),
	KEY(1, 4, KEY_Y),
	KEY(1, 5, KEY_H),
	KEY(1, 6, KEY_A),
	KEY(1, 7, KEY_UNKNOWN),

	KEY(2, 0, KEY_T),
	KEY(2, 1, KEY_G),
	KEY(2, 2, KEY_V),
	KEY(2, 3, KEY_B),
	KEY(2, 4, KEY_U),
	KEY(2, 5, KEY_J),
	KEY(2, 6, KEY_N),
	KEY(2, 7, KEY_UNKNOWN),

	KEY(3, 0, KEY_HOME),
	KEY(3, 1, KEY_SEND),
	KEY(3, 2, KEY_END),
	KEY(3, 3, KEY_F1),
	KEY(3, 4, KEY_F2),
	KEY(3, 5, KEY_F3),
	KEY(3, 6, KEY_BACK),
	KEY(3, 7, KEY_UNKNOWN),

	KEY(4, 0, KEY_UNKNOWN),
	KEY(4, 1, KEY_UNKNOWN),
	KEY(4, 2, KEY_UNKNOWN),
	KEY(4, 3, KEY_UNKNOWN),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(4, 5, KEY_UNKNOWN),
	KEY(4, 6, KEY_BACKSPACE),
	KEY(4, 7, KEY_F4),

	KEY(5, 0, KEY_UNKNOWN),
	KEY(5, 1, KEY_UNKNOWN),
	KEY(5, 2, KEY_UNKNOWN),
	KEY(5, 3, KEY_UNKNOWN),
	KEY(5, 4, KEY_UNKNOWN),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_UNKNOWN),
	KEY(5, 7, KEY_UNKNOWN),

	KEY(6, 0, KEY_I),
	KEY(6, 1, KEY_K),
	KEY(6, 2, KEY_DOT),
	KEY(6, 3, KEY_O),
	KEY(6, 4, KEY_L),
	KEY(6, 5, KEY_M),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_SELECT),

	KEY(7, 0, KEY_UNKNOWN),
	KEY(7, 1, KEY_ENTER),
	KEY(7, 2, KEY_CAPSLOCK),
	KEY(7, 3, KEY_SPACE),
	KEY(7, 4, KEY_LEFT),
	KEY(7, 5, KEY_RIGHT),
	KEY(7, 6, KEY_UP),
	KEY(7, 7, KEY_DOWN),
	0,
};

static struct resource sdp4430_kp_resources[] = {
	{
		.start  = OMAP4_KBDOCP_BASE,
		.end    = OMAP4_KBDOCP_BASE,
		.flags  = IORESOURCE_MEM,
	},
};

static struct omap_kp_platform_data omap_kp_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= omap_keymap,
	.keymapsize	= ARRAY_SIZE(omap_keymap),
	.delay		= 4,
	.rep		= 1,
};

static struct platform_device omap_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &omap_kp_data,
	},
	.num_resources	= ARRAY_SIZE(sdp4430_kp_resources),
	.resource	= sdp4430_kp_resources,
};

/* Begin Synaptic Touchscreen TM-01217 */

static char *tm12xx_idev_names[] = {
	"Synaptic TM12XX TouchPoint 1",
	"Synaptic TM12XX TouchPoint 2",
	"Synaptic TM12XX TouchPoint 3",
	"Synaptic TM12XX TouchPoint 4",
	"Synaptic TM12XX TouchPoint 5",
	"Synaptic TM12XX TouchPoint 6",
	NULL,
};

static u8 tm12xx_button_map[] = {
	KEY_F1,
	KEY_F2,
};

static struct tm12xx_ts_platform_data tm12xx_platform_data[] = {
	[0] = { /* Primary Controller */
		.gpio_intr = 35,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
	},
	[1] = { /* Secondary Controller */
		.gpio_intr = 36,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
	},
};

/* End Synaptic Touchscreen TM-01217 */

static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct omap2_mcspi_device_config dummy1_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

#ifdef CONFIG_SPI_TI_OMAP_TEST
static struct omap2_mcspi_device_config dummy2_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 0,  /* 0: slave, 1: master */
};
#endif

static struct platform_device sdp4430_lcd_device = {
	.name		= "sdp4430_lcd",
	.id		= -1,
};

static struct platform_device *sdp4430_devices[] __initdata = {
	&sdp4430_lcd_device,
	&omap_kp_device,
};

static struct omap_uart_config sdp4430_uart_config __initdata = {
	.enabled_uarts	= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
};

static struct omap_lcd_config sdp4430_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
	{ OMAP_TAG_LCD,		&sdp4430_lcd_config },
};

static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
	},
	{
		.supply = "vmmc",
	},
	{
		.supply = "vmmc",
	},
	{
		.supply = "vmmc",
	},
	{
		.supply = "vmmc",
	},
};

#ifdef CONFIG_CACHE_L2X0
static int __init omap_l2_cache_init(void)
{
	void __iomem *l2cache_base;

	/* Static mapping, never released */
	l2cache_base = ioremap(OMAP44XX_L2CACHE_BASE, SZ_4K);
	BUG_ON(!l2cache_base);

	/* Enable L2 Cache using secure api
	 * Save/Restore relevant registers
	 */
	__asm__ __volatile__(
		"stmfd r13!, {r0-r12, r14}\n"
		"mov r0, #1\n"
		"ldr r12, =0x102\n"
		"dsb\n"
		"smc\n"
		"ldmfd r13!, {r0-r12, r14}");

	/* 32KB way size, 16-way associativity,
	* parity disabled
	*/
	l2x0_init(l2cache_base, 0x0e050000, 0xc0000fff);

	return 0;
}
early_initcall(omap_l2_cache_init);
#endif

static void __init gic_init_irq(void)
{
	void __iomem *base;

	/* Static mapping, never released */
	base = ioremap(OMAP44XX_GIC_DIST_BASE, SZ_4K);
	BUG_ON(!base);
	gic_dist_init(0, base, 29);

	/* Static mapping, never released */
	gic_cpu_base_addr = ioremap(OMAP44XX_GIC_CPU_BASE, SZ_512);
	BUG_ON(!gic_cpu_base_addr);
	gic_cpu_init(0, gic_cpu_base_addr);
}

static void __init omap_4430sdp_init_irq(void)
{
	omap_board_config = sdp4430_config;
	omap_board_config_size = ARRAY_SIZE(sdp4430_config);
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	omap_gpio_init();
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV		 	= 1000000,
		.max_uV		 	= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV		 	= 1200000,
		.max_uV		 	= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV		 	= 1000000,
		.max_uV		 	= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV		 	= 1200000,
		.max_uV		 	= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 5,
	.consumer_supplies      = &sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV		 	= 1800000,
		.max_uV		 	= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV		 	= 1200000,
		.max_uV		 	= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV		 	= 2100000,
		.max_uV		 	= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV		 	= 1800000,
		.max_uV		 	= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV		 	= 1800000,
		.max_uV		 	= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV		 	= 3300000,
		.max_uV		 	= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.madc           = &sdp4430_gpadc_data,
};

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_44XX_SYS_NIRQ,
		.platform_data = &sdp4430_twldata,
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO("tm12xx_ts_primary", 0x4b),
		.platform_data = &tm12xx_platform_data[0],
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tm12xx_ts_secondary", 0x4b),
		.platform_data = &tm12xx_platform_data[1],
	},
};

static int __init omap4_i2c_init(void)
{
	/* Phoenix Audio IC needs I2C1 to srat with 400 KHz and less */
	omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, sdp4430_i2c_3_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_3_boardinfo));

	return 0;
}

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "ks8851",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.irq			= 34,
	},
	[1] = {
		.modalias		= "spitst",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode                   = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode                   = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode                   = MUSB_PERIPHERAL,
 #endif
	.power                  = 100,
 };

static void omap_ethernet_init(void)
{
	gpio_request(48, "ethernet");
	gpio_direction_output(48, 1);
	gpio_request(138, "quart");
	gpio_direction_output(138, 1);
	gpio_request(34, "ks8851");
	gpio_direction_input(34);
}

static void __init omap_4430sdp_init(void)
{
	omap4_i2c_init();
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	omap_serial_init();
	/* OMAP4 SDP uses internal transceiver so register nop transceiver */
	usb_nop_xceiv_register();
	usb_musb_init(&musb_board_data);
	omap_ethernet_init();
	sdp4430_spi_board_info[0].irq = gpio_to_irq(34);
	spi_register_board_info(sdp4430_spi_board_info,
			ARRAY_SIZE(sdp4430_spi_board_info));

}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_4430SDP, "OMAP4430 4430SDP board")
	/* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_4430sdp_map_io,
	.init_irq	= omap_4430sdp_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
