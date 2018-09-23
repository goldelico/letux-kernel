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

#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <asm/mach-jz4740/platform.h>

#include "clock.h"

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_udc_device,
	&jz4740_udc_xceiv_device,
	&jz4740_pcm_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
};

static int __init minipc_init_platform_devices(void)
{
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
