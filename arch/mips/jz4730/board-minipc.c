/*
 * linux/arch/mips/jz4730/board-minipc.c
 *
 * JZ4730 minipc board setup routines.
 * Author: <ard@kwaak.net>
 *
 * Derived from:
 * board-pmp.c
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 * and:
 * skytone celinux:arc/mips/jz4730/pmpv1/ *.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/gpio_mouse.h>
#include <linux/i2c.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

extern void (*jz_timer_callback)(void);

#if 0
static void dancing(void)
{
	static unsigned int count = 0;

	count ++;
	count &= 1;
	if (count)
		__gpio_set_pin(GPIO_LED_EN);
	else
		__gpio_clear_pin(GPIO_LED_EN);
}

static void pmp_timer_ack(void)
{
	static unsigned int count = 0;
	count ++;
	if (count % 100 == 0) {
		count = 0;
		dancing();
	}
}
#endif

static void __init board_cpm_setup(void)
{
	__cpm_start_all();
}

static void __init board_gpio_setup(void)
{
	/*
	 * Most of the gpios have been setup in the bootloader.
	 */

	__harb_usb0_uhc();
	__gpio_as_emc();
	__gpio_as_uart0();
	__gpio_as_dma();
	__gpio_as_eth();
	__gpio_as_usb();
	__gpio_as_lcd_master();
#if defined(CONFIG_I2S_AK4642EN)
	__gpio_as_i2s_master();
#endif
#if defined(CONFIG_I2S_TSC2301) || defined(CONFIG_I2S_TLC320AIC23)
	__gpio_as_ssi();
#endif
	//__gpio_as_ac97();
#if defined(CONFIG_I2S_TSC2301) || defined(CONFIG_I2S_TLC320AIC23) || defined(CONFIG_I2S_CS42L51)
	__gpio_as_i2s_slave();
#endif
//	__gpio_as_msc();

/* skytone:
	__gpio_as_output(GPIO_LED_EN);
	__gpio_set_pin(GPIO_LED_EN);
	 __gpio_as_output(GPIO_DISP_OFF_N);
	__gpio_set_pin(GPIO_DISP_OFF_N);
*/
	__gpio_as_output(GPIO_PWM0);
	__gpio_set_pin(GPIO_PWM0);
	__gpio_as_pwm();
	__pwm_set_duty(0, 300);
	__pwm_set_period(0, 299);
	__pwm_set_prescale(0, 0xbf);

/* skytone:
	__gpio_as_input(GPIO_RTC_IRQ);
*/
	__gpio_as_output(GPIO_USB_CLK_EN);
	__gpio_set_pin(GPIO_USB_CLK_EN);

	/* ard: We have a uC controlling charging. */
#if 0
	__gpio_as_input(GPIO_CHARG_STAT);
	__gpio_disable_pull(GPIO_CHARG_STAT);
#endif

	/* We are not doing UDC (yet?) */
#if 0
        __gpio_as_input(GPIO_UDC_HOTPLUG);
        __gpio_disable_pull(GPIO_UDC_HOTPLUG);
        __gpio_disable_pull(54); /* fixed ic bug, the pull of gpio pin 86 is as pin 54 */
#endif
}

void minipc_bl_set_intensity(int intensity)
{
	printk(KERN_ERR "MiniPC BL, set intensity to %d\n", intensity);
	__pwm_set_duty(0, intensity);
}

static struct generic_bl_info minipc_bl_machinfo = {
	.name			= "minipc-bl",
	.max_intensity		= 300,
	.default_intensity	= 250,
	.limit_mask		= 0x0b,
	.set_bl_intensity = minipc_bl_set_intensity,
	// .kick_battery = corgi_bl_kick_battery,
};

struct platform_device minipc_bl_device = {
	.name		= "generic-bl",
	.dev		= {
		.platform_data  = &minipc_bl_machinfo,
	},
	.id		= -1,
};

static struct i2c_board_info pcf8563_rtc_board_info[] = {
	[0] = {
		.type = "pcf8563",
		.addr = 0x51,
		}
};

/* touch pad buttons */

int gpio_get_value(int pin)
{ // poll this gpio
	return __gpio_get_pin(pin);
}

int gpio_free(int pin)
{ // release
	return 0;	// ignored
}

int gpio_request(int pin, char *name)
{ // request
	return 0;	// ignored
}

int gpio_direction_input(int pin)
{
	__gpio_as_input(pin);
	return 0;	// ignored, we assume to only have inputs...
}

static struct gpio_mouse_platform_data touchpad_buttons_board_info[] = {
	[0] = {
		.scan_ms = 20,
		.polarity = GPIO_MOUSE_POLARITY_ACT_HIGH,
{ {
			.up = GPIO_TS_LEFT_BUTTON,
			.down = GPIO_TS_LEFT_BUTTON,	// should result in 0 unless the low probability event that the gpio value changes within some fraction of a us. Anyway. will only report +/-1. 
			.left = GPIO_TS_LEFT_BUTTON,
			.right = GPIO_TS_LEFT_BUTTON,
			.bleft = GPIO_TS_LEFT_BUTTON,
			.bmiddle = -1,	// n/a
			.bright = GPIO_TS_RIGHT_BUTTON,
} }
	}
};

struct platform_device touchpad_buttons_device = {
	.name		= "gpio_mouse",
	.dev		= {
//		.num_resources	= ARRAY_SIZE(touchpad_buttons_board_info),
		.platform_data  = touchpad_buttons_board_info,
	},
	.id		= -1,
};

/* initialization */

void __init jz_board_setup(void)
{
	printk("JZ4730 MINIPC board setup\n");

	board_cpm_setup();
	board_gpio_setup();

/*
 * ard:
 *  We don't have pmp timer ack led stuff
	jz_timer_callback = pmp_timer_ack;
 */
	
	// this is how it *should* better work than the probing workaround in pcf8563_attach()
	//	i2c_register_board_info(0, pcf8563_rtc_board_info, 1);	// before registering adapters
	
}
