/*
 * gpio.h -- GPIO driver for NXP PCF50633
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50633_GPIO_H
#define __LINUX_MFD_PCF50633_GPIO_H

#include <linux/mfd/pcf50633/core.h>

#define PCF50633_GPIO1		0
#define PCF50633_GPIO2		1
#define PCF50633_GPIO3		2
#define PCF50633_GPO		3

#define PCF50633_REG_GPIOCFG(x) (0x14 + (x))

enum pcf50633_gpio_config {
	PCF50633_GPIO_CONFIG_OUTPUT	= 0x0,
	PCF50633_GPIO_CONFIG_SYSxOK	= 0x2,
	PCF50633_GPIO_CONFIG_CHARGING	= 0x3,
	PCF50633_GPIO_CONFIG_MOBILE_MODE = 0x4,
	PCF50633_GPIO_CONFIG_USBxOK	= 0x5,
	PCF50633_GPIO_CONFIG_ACTPH	= 0x6,
	PCF50633_GPIO_CONFIG_INPUT	= 0x7,

	PCF50633_GPIO_CONFIG_INVERT	= 0x8,

	PCF50633_GPO_CONFIG_OUTPUT	= 0x0,
	PCF50633_GPO_CONFIG_LED_NFET	= 0x1,
	PCF50633_GPO_CONFIG_SYSxOK	= 0x2,
	PCF50633_GPO_CONFIG_CLK32K	= 0x3,
	PCF50633_GPO_CONFIG_MOBILE_MODE = 0x4,
	PCF50633_GPO_CONFIG_USBxOK	= 0x5,
	PCF50633_GPO_CONFIG_ACTPH	= 0x6,
	PCF50633_GPO_CONFIG_INPUT	= 0x7,

	PCF50633_GPO_CONFIG_INVERT	= 0x8,
};

int pcf50633_gpio_set_config(struct pcf50633 *pcf, unsigned gpio,
                              enum pcf50633_gpio_config config);

int pcf50633_gpio_power_supply_set(struct pcf50633 *pcf,
					int gpio, int regulator, int on);
#endif /* __LINUX_MFD_PCF50633_GPIO_H */


