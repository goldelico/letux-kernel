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
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>

static int ts_gpio;

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
};

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_44XX_SYS_NIRQ,
		.platform_data = &sdp4430_twldata,
	},
};

static int __init omap4_i2c_init(void)
{
	/* Phoenix Audio IC needs I2C1 to srat with 400 KHz and less */
	omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "spitst",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
	},
};

static void __init omap_4430sdp_init(void)
{
	omap4_i2c_init();
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	omap_serial_init();
	sdp4430_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
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
