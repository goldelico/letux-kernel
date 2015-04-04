/*
 * linux/arch/mips/jz4740/board-minipc.c
 *
 * Mini PC board support
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Copyright (c) 2009 Qi Hardware inc.,
 * Author: Xiangfu Liu <xiangfu@qi-hardware.com>
 * Copyright 2010, Lars-Peter Clausen <lars@metafoo.de>
 * Copyright 2015, Paul Boddie <paul@boddie.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio/machine.h>

#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_mouse.h>
#include <linux/power_supply.h>
#include <linux/power/jz4740-battery.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <asm/mach-jz4740/jz4740_fb.h>
#include <asm/mach-jz4740/jz4740_mmc.h>
#include <asm/mach-jz4740/jz4740_nand.h>
#include <asm/mach-jz4740/platform.h>
#include <asm/mach-jz4740/jz4730_gpio.h>

#include "clock.h"

/* Specific GPIO definitions. */
#define MINIPC_GPIO_SD_CD		JZ_GPIO_PORTC(0)
#define MINIPC_GPIO_SD_WP		JZ_GPIO_PORTC(2)
#define MINIPC_GPIO_SD_VCC_EN_N         JZ_GPIO_PORTA(21)
#define MINIPC_GPIO_LED_EN		JZ_GPIO_PORTC(28)
#define MINIPC_GPIO_LED_EN_PORT		2
#define MINIPC_GPIO_LED_EN_NUM		28

struct jz4740_clock_board_data jz4740_clock_bdata = {
        .ext_rate = 12000000,
        .rtc_rate = 32768,
};

/* Display */
static struct fb_videomode minipc_video_modes[] = {
	{
		.name = "800x480",
		.xres = 800,
		.yres = 480,
		.refresh = 60,
		.left_margin = 0,
		.right_margin = 0,
		.upper_margin = 0,
		.lower_margin = 0,
		.hsync_len = 80,
		.vsync_len = 20,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data minipc_fb_pdata = {
	.width		= 60,
	.height		= 45,
	.num_modes	= ARRAY_SIZE(minipc_video_modes),
	.modes		= minipc_video_modes,
	.bpp		= 16,
	.lcd_type	= JZ_LCD_TYPE_GENERIC_16_BIT,
	.pixclk_falling_edge = 1,
};

/* backlight platform device */

static struct platform_pwm_backlight_data minipc_bl_data = {
	.pwm_id			= 0,
	.max_brightness		= 300,
	.dft_brightness		= 250,
	.lth_brightness		= 50,
	.pwm_period_ns		= 81250, // (1s / 2**6) * 299
	.enable_gpio		= -1,
};

static struct platform_device minipc_bl_device = {
        .name = "pwm-backlight",
	.id = -1,
        .dev = {
		.parent = &jz4740_pwm_device.dev,
                .platform_data = &minipc_bl_data,
        },
};

static struct jz4740_mmc_platform_data minipc_mmc_pdata = {
        .gpio_card_detect       = MINIPC_GPIO_SD_CD,
        .gpio_read_only         = MINIPC_GPIO_SD_WP,
        .gpio_power             = MINIPC_GPIO_SD_VCC_EN_N,
        .power_active_low       = 1,
};

/* Battery...
static struct jz_battery_platform_data minipc_battery_pdata = {
        .gpio_charge =  JZ_GPIO_CHARGING,
        .gpio_charge_active_low = 1,
        .info = {
                .name = "battery",
                .technology = POWER_SUPPLY_TECHNOLOGY_LIPO,
                .voltage_max_design = 4200000,
                .voltage_min_design = 3700000,
        },
};
*/

/* touch pad buttons */

#define MINIPC_GPIO_TS_LEFT_BUTTON	16
#define MINIPC_GPIO_TS_RIGHT_BUTTON	13

static struct gpio_mouse_platform_data touchpad_buttons_board_info[] = {
	[0] = {
		.scan_ms = 20,
		.polarity = GPIO_MOUSE_POLARITY_ACT_HIGH,
		{ {
			.up = MINIPC_GPIO_TS_LEFT_BUTTON,
			.down = MINIPC_GPIO_TS_LEFT_BUTTON,	// should result in 0 unless the low probability event that the gpio value changes within some fraction of a us. Anyway. will only report +/-1. 
			.left = MINIPC_GPIO_TS_LEFT_BUTTON,
			.right = MINIPC_GPIO_TS_LEFT_BUTTON,
			.bleft = MINIPC_GPIO_TS_LEFT_BUTTON,
			.bmiddle = -1,	// n/a
			.bright = MINIPC_GPIO_TS_RIGHT_BUTTON,
		} }
	}
};

static struct platform_device touchpad_buttons_device = {
	.name = "gpio-mouse",
	.id = -1,
	.dev = {
		.platform_data  = touchpad_buttons_board_info,
	},
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_udc_device,
	&jz4740_udc_xceiv_device,
	&jz4740_mmc_device,
	//&jz4740_nand_device,
	&jz4740_framebuffer_device,
	&jz4740_pcm_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_rtc_device,
	//&jz4740_adc_device,
	&jz4740_pwm_device,
	&jz4740_dma_device,
	&minipc_bl_device,
	&touchpad_buttons_device,
};

static void __init board_gpio_setup(void)
{
	/* We only need to enable/disable pullup here for pins used in generic
	 * drivers. Everything else is done by the drivers themselves. */

	jz_gpio_set_function(MINIPC_GPIO_LED_EN, JZ_GPIO_FUNC_NONE);
	jz_gpio_port_direction_output(MINIPC_GPIO_LED_EN_PORT, 1 << MINIPC_GPIO_LED_EN_NUM);

	jz_gpio_set_function(JZ_GPIO_DMA_DREQ0, JZ_GPIO_FUNC_DMA_DREQ0);
	jz_gpio_set_function(JZ_GPIO_DMA_DACK0, JZ_GPIO_FUNC_DMA_DACK0);
	jz_gpio_set_function(JZ_GPIO_DMA_AEN, JZ_GPIO_FUNC_DMA_AEN);
	jz_gpio_set_function(JZ_GPIO_DMA_EOP, JZ_GPIO_FUNC_DMA_EOP);

	jz_gpio_set_function(JZ_GPIO_UHC_CLK, JZ_GPIO_FUNC_UHC_CLK);
	jz_gpio_set_function(JZ_GPIO_UHC_PPWR0, JZ_GPIO_FUNC_UHC_PPWR0);

	jz_gpio_set_function(JZ_GPIO_AIC_SYSCLK, JZ_GPIO_FUNC_AIC_SYSCLK);
	jz_gpio_set_function(JZ_GPIO_AIC_SDATA_OUT, JZ_GPIO_FUNC_AIC_SDATA_OUT);
	jz_gpio_set_function(JZ_GPIO_AIC_SDATA_IN, JZ_GPIO_FUNC_AIC_SDATA_IN);
	jz_gpio_set_function(JZ_GPIO_AIC_BITCLK, JZ_GPIO_FUNC_AIC_BITCLK);
	jz_gpio_set_function(JZ_GPIO_AIC_SYNC, JZ_GPIO_FUNC_AIC_SYNC);

	jz_gpio_set_function(JZ_GPIO_UART0_RXD, JZ_GPIO_FUNC_UART0_RXD);
	jz_gpio_set_function(JZ_GPIO_UART0_TXD, JZ_GPIO_FUNC_UART0_TXD);

	/* SPI...
	jz_gpio_set_function(JZ_GPIO_SPI_CLK, JZ_GPIO_FUNC_SPI_CLK);
	jz_gpio_set_function(JZ_GPIO_SPI_CE1, JZ_GPIO_FUNC_SPI_CE1);
	jz_gpio_set_function(JZ_GPIO_SPI_DT, JZ_GPIO_FUNC_SPI_DT);
	jz_gpio_set_function(JZ_GPIO_SPI_DR, JZ_GPIO_FUNC_SPI_DR);
	jz_gpio_set_function(JZ_GPIO_SPI_CE2, JZ_GPIO_FUNC_SPI_CE2);
	*/
}

static int __init minipc_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &minipc_fb_pdata;
	//jz4740_nand_device.dev.platform_data = &minipc_nand_pdata;
	//jz4740_adc_device.dev.platform_data = &minipc_battery_pdata;
	jz4740_mmc_device.dev.platform_data = &minipc_mmc_pdata;

	jz4740_serial_device_register();

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}

static int __init minipc_board_setup(void)
{
	printk(KERN_INFO "Mini PC JZ4740 setup\n");

	board_gpio_setup();

	if (minipc_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}
arch_initcall(minipc_board_setup);
