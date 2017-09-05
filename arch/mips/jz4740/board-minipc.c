/*
 * linux/arch/mips/jz4740/board-minipc.c
 *
 * Letux400 MiniPC board support
 *
 * Copyright (c) 2009 Qi Hardware inc.,
 * Author: Xiangfu Liu <xiangfu@qi-hardware.com>
 * Copyright 2010, Lars-Peter Clausen <lars@metafoo.de>
 * Copyright 2017 Paul Boddie <paul@boddie.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio/machine.h>

#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_mouse.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/power_supply.h>
#include <linux/power/jz4740-battery.h>
#include <linux/power/gpio-charger.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>

#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>
#include <asm/mach-jz4740/jz4740_mmc.h>
#include <asm/mach-jz4740/jz4740_nand.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <asm/mach-jz4740/platform.h>

#include "clock.h"

/* GPIOs */
#define MINIPC_GPIO_SD_CD		JZ_GPIO_PORTC(26)
#define MINIPC_GPIO_SD_VCC_EN_N		JZ_GPIO_PORTC(27)
#define MINIPC_GPIO_SD_WP		JZ_GPIO_PORTC(18)

#define MINIPC_GPIO_LOWBAT		JZ_GPIO_PORTA(17)

#define MINIPC_GPIO_KEYOUT(x)		(JZ_GPIO_PORTD(0) + (x))
#define MINIPC_GPIO_KEYOUT16		JZ_GPIO_PORTD(29)
#define MINIPC_GPIO_KEYIN(x)		(JZ_GPIO_PORTA(0) + (x))

#define MINIPC_GPIO_TS_LEFT_BUTTON	JZ_GPIO_PORTA(16)
#define MINIPC_GPIO_TS_RIGHT_BUTTON	JZ_GPIO_PORTA(13)

/* NAND ignored for the time being */

/* Keyboard*/

static const uint32_t minipc_keymap[] = {
	KEY(0, 0, KEY_PAUSE),
	KEY(0, 1, KEY_Q),
	KEY(0, 2, KEY_W),
	KEY(0, 3, KEY_E),
	KEY(0, 4, KEY_R),
	KEY(0, 5, KEY_U),
	KEY(0, 6, KEY_I),
	KEY(0, 7, KEY_O),
	KEY(0, 11, KEY_P),

	KEY(1, 1, KEY_TAB),
	KEY(1, 2, KEY_CAPSLOCK),
	KEY(1, 3, KEY_F3),
	KEY(1, 4, KEY_T),
	KEY(1, 5, KEY_Y),
	KEY(1, 6, KEY_RIGHTBRACE),
	KEY(1, 7, KEY_F7),
	KEY(1, 9, KEY_BACKSPACE),
	KEY(1, 11, KEY_LEFTBRACE),
	KEY(1, 12, KEY_DISPLAY_OFF),
	KEY(1, 16, KEY_LEFTSHIFT),

	KEY(2, 1, KEY_A),
	KEY(2, 2, KEY_S),
	KEY(2, 3, KEY_D),
	KEY(2, 4, KEY_F),
	KEY(2, 5, KEY_J),
	KEY(2, 6, KEY_K),
	KEY(2, 7, KEY_L),
	KEY(2, 11, KEY_SEMICOLON),
	KEY(2, 15, KEY_UP),
	KEY(2, 16, KEY_RIGHTSHIFT),

	KEY(3, 1, KEY_ESC),
	KEY(3, 2, KEY_102ND),
	KEY(3, 3, KEY_F4),
	KEY(3, 4, KEY_G),
	KEY(3, 5, KEY_H),
	KEY(3, 6, KEY_F6),
	KEY(3, 8, KEY_SPACE),
	KEY(3, 10, KEY_LEFTALT),
	KEY(3, 11, KEY_APOSTROPHE),
	KEY(3, 15, KEY_DOWN),

	KEY(4, 1, KEY_Z),
	KEY(4, 2, KEY_X),
	KEY(4, 3, KEY_C),
	KEY(4, 4, KEY_V),
	KEY(4, 5, KEY_M),
	KEY(4, 6, KEY_COMMA),
	KEY(4, 7, KEY_DOT),
	KEY(4, 8, KEY_NUMLOCK),
	KEY(4, 9, KEY_ENTER),
	KEY(4, 11, KEY_BACKSLASH),
	KEY(4, 15, KEY_LEFT),

	KEY(5, 4, KEY_B),
	KEY(5, 5, KEY_N),
	KEY(5, 7, KEY_F19),
	KEY(5, 11, KEY_SLASH),
	KEY(5, 15, KEY_RIGHT),

	KEY(6, 0, KEY_LEFTCTRL),
	KEY(6, 1, KEY_GRAVE),
	KEY(6, 4, KEY_5),
	KEY(6, 5, KEY_6),
	KEY(6, 6, KEY_EQUAL),
	KEY(6, 7, KEY_F8),
	KEY(6, 8, KEY_DELETE),
	KEY(6, 9, KEY_F9),
	KEY(6, 11, KEY_MINUS),
	KEY(6, 13, KEY_F2),
	KEY(6, 14, KEY_INSERT),
	KEY(6, 16, KEY_F1),

	KEY(7, 0, KEY_F5),
	KEY(7, 1, KEY_1),
	KEY(7, 2, KEY_2),
	KEY(7, 3, KEY_3),
	KEY(7, 4, KEY_4),
	KEY(7, 5, KEY_7),
	KEY(7, 6, KEY_8),
	KEY(7, 7, KEY_9),
	KEY(7, 10, KEY_SYSRQ),
	KEY(7, 11, KEY_0),
	KEY(7, 12, KEY_F10),
	KEY(7, 16, KEY_FN),
};

static const struct matrix_keymap_data minipc_keymap_data = {
	.keymap		= minipc_keymap,
	.keymap_size	= ARRAY_SIZE(minipc_keymap),
};

static const unsigned int minipc_keypad_cols[] = {
	MINIPC_GPIO_KEYOUT(0),
	MINIPC_GPIO_KEYOUT(1),
	MINIPC_GPIO_KEYOUT(2),
	MINIPC_GPIO_KEYOUT(3),
	MINIPC_GPIO_KEYOUT(4),
	MINIPC_GPIO_KEYOUT(5),
	MINIPC_GPIO_KEYOUT(6),
	MINIPC_GPIO_KEYOUT(7),
	MINIPC_GPIO_KEYOUT(8),
	MINIPC_GPIO_KEYOUT(9),
	MINIPC_GPIO_KEYOUT(10),
	MINIPC_GPIO_KEYOUT(11),
	MINIPC_GPIO_KEYOUT(12),
	MINIPC_GPIO_KEYOUT(13),
	MINIPC_GPIO_KEYOUT(14),
	MINIPC_GPIO_KEYOUT(15),
	MINIPC_GPIO_KEYOUT16,
};

static const unsigned int minipc_keypad_rows[] = {
	MINIPC_GPIO_KEYIN(0),
	MINIPC_GPIO_KEYIN(1),
	MINIPC_GPIO_KEYIN(2),
	MINIPC_GPIO_KEYIN(3),
	MINIPC_GPIO_KEYIN(4),
	MINIPC_GPIO_KEYIN(5),
	MINIPC_GPIO_KEYIN(6),
	MINIPC_GPIO_KEYIN(7),
};

static struct matrix_keypad_platform_data minipc_pdata = {
	.keymap_data = &minipc_keymap_data,
	.col_gpios	= minipc_keypad_cols,
	.row_gpios	= minipc_keypad_rows,
	.num_col_gpios	= ARRAY_SIZE(minipc_keypad_cols),
	.num_row_gpios	= ARRAY_SIZE(minipc_keypad_rows),
	.col_scan_delay_us	= 10,
	.debounce_ms		= 10,
	.wakeup			= 1,
	.active_low		= 1,
};

static struct platform_device minipc_keypad_device = {
	.name		= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &minipc_pdata,
	},
};

/* touchpad */
static struct gpio_mouse_platform_data minipc_touchpad_data = {
	.scan_ms = 20,
	.polarity = GPIO_MOUSE_POLARITY_ACT_HIGH,
	{
		.up = MINIPC_GPIO_TS_LEFT_BUTTON,
		.down = MINIPC_GPIO_TS_LEFT_BUTTON,
		.left = MINIPC_GPIO_TS_LEFT_BUTTON,
		.right = MINIPC_GPIO_TS_LEFT_BUTTON,
		.bleft = MINIPC_GPIO_TS_LEFT_BUTTON,
		.bmiddle = -1,
		.bright = MINIPC_GPIO_TS_RIGHT_BUTTON,
	}
};

static struct platform_device minipc_touchpad_device = {
	.name		= "gpio-mouse",
	.id		= -1,
	.dev		= {
		.platform_data = &minipc_touchpad_data,
	},
};

/* display */
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

/* MMC */
static struct jz4740_mmc_platform_data minipc_mmc_pdata = {
	.gpio_card_detect	= MINIPC_GPIO_SD_CD,
	.gpio_read_only		= MINIPC_GPIO_SD_WP,
	.gpio_power		= MINIPC_GPIO_SD_VCC_EN_N,
	.power_active_low	= 1,
};

/* backlight */
static struct platform_pwm_backlight_data minipc_backlight_data = {
        .pwm_id                 = 0,
        .max_brightness         = 300,
        .dft_brightness         = 250,
        .lth_brightness         = 50,
        .pwm_period_ns          = 81250, // (1s / 2**6) * 299
        .enable_gpio            = -1,
};

static struct platform_device minipc_backlight_device = {
        .name = "pwm-backlight",
        .id = -1,
        .dev = {
                .parent = &jz4730_pwm_device.dev,
                .platform_data = &minipc_backlight_data,
        },
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_udc_device,
	&jz4740_udc_xceiv_device,
	&jz4740_mmc_device,
	/* &jz4740_nand_device, */
	&jz4740_framebuffer_device,
	&jz4740_pcm_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_dma_device,
	&jz4730_pwm_device,
	&minipc_backlight_device,
	&minipc_keypad_device,
	&minipc_touchpad_device,
};

static unsigned long pin_cfg_bias_disable[] = {
	    PIN_CONFIG_BIAS_DISABLE,
};

static struct pinctrl_map pin_map[] __initdata = {
	/* NAND pin configuration */
	/* PIN_MAP_MUX_GROUP_DEFAULT("jz4740-nand",
			"10010000.jz4730-pinctrl", "nand", "nand-cs3"), */

	/* fbdev pin configuration */
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_DEFAULT,
			"10010000.jz4730-pinctrl", "lcd", "lcd-8bit"),
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_DEFAULT,
			"10010000.jz4730-pinctrl", "lcd", "lcd-16bit"),
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_DEFAULT,
			"10010000.jz4730-pinctrl", "lcd", "lcd-16bit-tft"),
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_SLEEP,
			"10010000.jz4730-pinctrl", "lcd", "lcd-no-pins"),

	/* MMC pin configuration */
	PIN_MAP_MUX_GROUP_DEFAULT("jz4740-mmc.0",
			"10010000.jz4730-pinctrl", "mmc", "mmc-1bit"),
	PIN_MAP_MUX_GROUP_DEFAULT("jz4740-mmc.0",
			"10010000.jz4730-pinctrl", "mmc", "mmc-4bit"),
	/* MMC detect and power */
	PIN_MAP_CONFIGS_PIN_DEFAULT("jz4740-mmc.0",
			"10010000.jz4740-pinctrl", "PC6", pin_cfg_bias_disable),
	PIN_MAP_CONFIGS_PIN_DEFAULT("jz4740-mmc.0",
			"10010000.jz4740-pinctrl", "PC7", pin_cfg_bias_disable),

	/* PWM pin configuration */
	PIN_MAP_MUX_GROUP_DEFAULT("jz4730-pwm",
			"10010000.jz4730-pinctrl", "pwm0", "pwm0"),
	PIN_MAP_MUX_GROUP_DEFAULT("jz4730-pwm",
			"10010000.jz4730-pinctrl", "pwm1", "pwm1"),
};


static int __init minipc_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &minipc_fb_pdata;
	/* jz4740_nand_device.dev.platform_data = &minipc_nand_pdata; */
	jz4740_mmc_device.dev.platform_data = &minipc_mmc_pdata;

	/* gpiod_add_lookup_table(&minipc_nand_gpio_table); */

	pinctrl_register_mappings(pin_map, ARRAY_SIZE(pin_map));

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}

static int __init minipc_board_setup(void)
{
	printk(KERN_INFO "Letux400 MiniPC JZ4730 setup\n");

	if (minipc_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}
arch_initcall(minipc_board_setup);
