/* linux/include/asm-arm/arch-s3c2410/gpio.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - GPIO lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#if 0
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#endif
#define gpio_cansleep	__gpio_cansleep

#ifndef __ASM_ARCH_S3C2410_GPIO_H
#define __ASM_ARCH_S3C2410_GPIO_H

#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);

#define gpio_get_value(gpio)		s3c2410_gpio_getpin(gpio)
#define gpio_set_value(gpio,value)	s3c2410_gpio_setpin(gpio, value)

#include <asm-generic/gpio.h>			/* cansleep wrappers */

#ifdef CONFIG_CPU_S3C2400
#define gpio_to_irq(gpio)		s3c2400_gpio_getirq(gpio)
#else
#define gpio_to_irq(gpio)		s3c2410_gpio_getirq(gpio)
#define irq_to_gpio(irq)		s3c2410_gpio_irq2pin(irq)
#endif

#include <asm-generic/gpio.h>
#endif

