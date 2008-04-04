#ifndef _HXD8_H
#define _HXD8_H

#include <asm/arch/regs-gpio.h>
#include <asm/arch/irqs.h>

#define HXD8v1_SYSTEM_REV	0x00000110

#define HXD8_GPIO_USB_CUR_SEL	S3C2410_GPA0
#define HXD8_GPIO_BACKLIGHT	S3C2410_GPB0
#define HXD8_GPIO_USB_PULLUP	S3C2410_GPB9
#define HXD8_GPIO_PCF50606	S3C2410_GPF6

#define HXD8_IRQ_PCF50606	IRQ_EINT6

#endif
