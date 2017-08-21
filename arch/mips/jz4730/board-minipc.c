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

void overwrite_root_name(/* full path to root device */ char *root_name, /* comma separated list of file systems */ char **root_fs_names)
{
	extern int jz_kbd_get_col(int col);
	/* check GPIOs if there is a boot key pressed and override boot arguments
	 F1:				boot from first SD partition (/dev/mmcblk0p1, ext3)
	 F2:				boot from second SD partition (/dev/mmcblk0p2, ext3)
	 F3:				boot minifs from internal partition (/dev/mtdblock3, jffs2)
	 F4:				boot from internal partition (/dev/mtdblock4, yaffs2)
	 Fn:				boot from second SD partition (/dev/mmcblk0p2, ext3) (kernel was booted from SD on Fn+LeftShift)
	 none:				boot from internal partition (/dev/mtdblock4, yaffs2)
	 
	 Notes:
		- the first condition in this list is taken, i.e. Fn+F4 results in F4 mode
		- there may be shadow keys (like F5) if Fn-Ctrl-LShift are pressed
		- pressed keys return a 0 bit
		- this check is done some seconds after starting the kernel right before the init process starts

	 */
#if 0
	{
		int i;
		for(i=0; i <= 16; i++) {
			if(jz_kbd_get_col(i) != 0xff)
				printk(KERN_INFO "col %d: %02x\n", i, jz_kbd_get_col(i));
		}
	}
#endif
	if ((jz_kbd_get_col(16) & 0x40) == 0)	// F1
		strcpy(root_name, "/dev/mmcblk0p1");
	else if ((jz_kbd_get_col(13) & 0x40) == 0)	// F2
		strcpy(root_name, "/dev/mmcblk0p2");
	else if ((jz_kbd_get_col(3) & 0x02) == 0)	// F3
		strcpy(root_name, "/dev/mtdblock3"), *root_fs_names="jffs2";
	else if ((jz_kbd_get_col(3) & 0x08) == 0)	// F4
		strcpy(root_name, "/dev/mtdblock4");
/*
 else if ((jz_kbd_get_col(0) & 0x80) == 0)	// F5
 strcpy(root_name, "/dev/mtdblock4");
 else if ((jz_kbd_get_col(6) & 0x08) == 0)	// F6
 strcpy(root_name, "/dev/mtdblock4");
 else if ((jz_kbd_get_col(7) & 0x02) == 0)	// F7
 strcpy(root_name, "/dev/mtdblock4");
 else if ((jz_kbd_get_col(6) & 0x40) == 0)	// F8
 strcpy(root_name, "/dev/mtdblock4");
 else if ((jz_kbd_get_col(9) & 0x40) == 0)	// F9
 strcpy(root_name, "/dev/mtdblock4");
 else if ((jz_kbd_get_col(12) & 0x80) == 0)	// F10
 strcpy(root_name, "/dev/mtdblock4");
 */
	else if ((jz_kbd_get_col(16) & 0x80) == 0)	// Fn
		strcpy(root_name, "/dev/mmcblk0p2");
	else	// neither
		strcpy (root_name, "/dev/mtdblock4");
	printk(KERN_INFO "Root file system (%s) overwritten as: %s\n", *root_fs_names, root_name);
}

/* initialization */

void __init jz_board_setup(void)
{
	printk("JZ4730 MINIPC board setup\n");
	
	board_cpm_setup();
	board_gpio_setup();
}

/* backlight platform device */

void minipc_bl_set_intensity(int intensity)
{
//	printk(KERN_INFO "MiniPC BL, set intensity to %d\n", intensity);
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

static struct platform_device minipc_bl_device = {
	.name		= "generic-bl",
	.dev		= {
		.platform_data  = &minipc_bl_machinfo,
	},
	.id		= -1,
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

static struct platform_device touchpad_buttons_device = {
	.name		= "gpio_mouse",
	.dev		= {
//		.num_resources	= ARRAY_SIZE(touchpad_buttons_board_info),
		.platform_data  = touchpad_buttons_board_info,
	},
	.id		= -1,
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&minipc_bl_device,
	&touchpad_buttons_device,
};

static int __init board_init(void)
{
	printk(KERN_DEBUG "adding board specific platform devices\n");
	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

arch_initcall(board_init);

/* RTC I2C device */

struct i2c_board_info i2c_bus_devices[] = {
	/* can't autoprobe this device
	[0] = {
		.type = "pcf8563",
		.addr = 0x51,
	}
	 */
};

static int __init i2c_init(void)
{
	printk(KERN_DEBUG "adding I2C devices\n");
	i2c_register_board_info(0, i2c_bus_devices, ARRAY_SIZE(i2c_bus_devices));
	return 0;
}

postcore_initcall(i2c_init);	// but before arch_initcall()
