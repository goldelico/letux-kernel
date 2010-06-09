/*
 * linux/drivers/input/keyboard/jz_keypad.c
 *
 * JZ Keypad Driver
 *
 * Copyright (c) 2005 - 2008  Ingenic Semiconductor Inc.
 *
 * Author: Richard <cjfeng@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/gpio.h>

#include <asm/jzsoc.h>


// #define DEBUG

#ifdef CONFIG_JZ4730_MINIPC

#define KB_ROWS         8
#define KB_COLS         17

#define SCAN_INTERVAL       (2)

static unsigned short col[KB_COLS] = {96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 125};
static unsigned short row[KB_ROWS] = {0, 1, 2, 3, 4, 5, 6, 7};
static unsigned short s0[KB_COLS];
static unsigned short s1[KB_COLS] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
// static unsigned short precol, prerow;

static unsigned int jz_kbd_keycode[KB_COLS][KB_ROWS] = {
/* 0*/ {KEY_PAUSE, 0, 0, 0, 0, 0, KEY_LEFTCTRL, KEY_F5,},
/* 1*/ {KEY_Q, KEY_TAB, KEY_A, KEY_ESC, KEY_Z, 0, KEY_GRAVE, KEY_1,},
/* 2*/ {KEY_W, KEY_CAPSLOCK, KEY_S, KEY_102ND, KEY_X, 0, 0, KEY_2,},//CZJ delet the KEY_BACKSLASH
/* 3*/ {KEY_E, KEY_F3, KEY_D, KEY_F4, KEY_C, 0, 0, KEY_3,},
/* 4*/ {KEY_R, KEY_T, KEY_F, KEY_G, KEY_V, KEY_B, KEY_5, KEY_4,},
/* 5*/ {KEY_U, KEY_Y, KEY_J, KEY_H, KEY_M, KEY_N, KEY_6, KEY_7,},
/* 6*/ {KEY_I, KEY_RIGHTBRACE, KEY_K, KEY_F6, KEY_COMMA, 0, KEY_EQUAL, KEY_8,},
/* 7*/ {KEY_O, KEY_F7, KEY_L, 0, KEY_DOT, KEY_F19, KEY_F8, KEY_9,},
/* 8*/ {0, 0, 0, KEY_SPACE, KEY_NUMLOCK, 0, KEY_DELETE, 0,},
/* 9*/ {0, KEY_BACKSPACE, 0, 0, KEY_ENTER, 0, KEY_F9, 0,},
/*10*/ {0, 0, 0, KEY_LEFTALT, 0, 0, 0, KEY_SYSRQ,},
/*11*/ {KEY_P, KEY_LEFTBRACE, KEY_SEMICOLON, KEY_APOSTROPHE, KEY_BACKSLASH, KEY_SLASH, KEY_MINUS, KEY_0,},
/*12*/ {KEY_KP0, KEY_F20, KEY_KP1, KEY_KP2, KEY_KP3, KEY_KP4, KEY_KP5, KEY_F10,}, //CZJ ADD NUM PACK
/*13*/ {KEY_KP6, KEY_KP7, KEY_KP8, KEY_KP9, KEY_KPPLUS, KEY_KPMINUS, KEY_F2, KEY_KPSLASH,}, //CZJ ADD NUM PACK
/*14*/ {KEY_KPDOT, KEY_KPASTERISK, 0, 0, 0, 0, KEY_INSERT, 0,},  //CZJ ADD NUM PACK
/*15*/ {0, 0, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, 0, 0,},
/*16*/ {0, KEY_LEFTSHIFT, KEY_RIGHTSHIFT, 0, 0, 0, KEY_F1, KEY_FN}
};

#else

#define KB_ROWS         3
#define KB_COLS         3

#define SCAN_INTERVAL       (5)

static unsigned short col[KB_COLS] = {85,87,91};
static unsigned short row[KB_ROWS] = {60,61,62};
static unsigned short s0[KB_COLS];
static unsigned short s1[KB_COLS]={7,7,7};
static unsigned short precol,prerow;

static const unsigned int jz_kbd_keycode[KB_COLS * KB_ROWS] = {
	KEY_1, KEY_4, KEY_7,
	KEY_2, KEY_5, 0,
	KEY_3, KEY_6, 0,
};
#endif

#define NO_ROW ((1<<(KB_ROWS-1))-1)

struct jz_kbd {
	unsigned int keycode[ARRAY_SIZE(jz_kbd_keycode)];
	struct input_dev *input;
	char phys[32];
	
	spinlock_t lock;
        struct timer_list timer;
	
	unsigned int suspended;
	unsigned long suspend_jiffies;
};

static struct jz_kbd g_jz_kbd;

int jz_kbd_get_col(int col)
{ // provide key status of given column
	if(col < 0 || col >= KB_COLS)
		return 0;
	return s0[col];
}

EXPORT_SYMBOL(jz_kbd_get_col);

static inline void jz_scan_kbd(unsigned short *s)
{
	int i;
	int row_cnt, col_cnt;

	if (!s)
		return;

	for (i = 0; i < KB_COLS; i++) {
#if 0
		for (row_cnt=0; row_cnt<KB_ROWS; row_cnt++)
			__gpio_as_input(row[row_cnt]);
		for (col_cnt=0; col_cnt<KB_COLS; col_cnt++) {
			__gpio_as_input(col[col_cnt]);
		}
#endif
//		udelay(1000);
		__gpio_clear_pin(col[i]);
		__gpio_as_output(col[i]);

		s[i]=0;
		for (row_cnt=0; row_cnt<KB_ROWS; row_cnt++) {
			s[i] |= __gpio_get_pin(row[row_cnt]) << row_cnt;
		}
		__gpio_set_pin(col[i]);
		__gpio_as_input(col[i]);
	}
}

static void jz_kbd_scankeyboard(struct jz_kbd *kbd_data)
{
	unsigned int row_cnt, col_cnt, i, r;
	unsigned long flags;
	unsigned int num_pressed;

	if (kbd_data->suspended)
		return;

//	spin_lock_irqsave(&kbd_data->lock, flags);

	num_pressed = 0;
	jz_scan_kbd(s0);

	/* we now have the new state in s0[KB_COLS] and the previous
	 * state in s1[KB_COLS]
	 * each bit in *(unsigned short)s0 represents one row
	 * if a bit is 0 then the key was pressed, 1 if released
	 * so first compare s0[] with s1[], if different a key was pressed
	 * or released, then find the corresponding changed bit and report
	 * the correspong key as pressed or released
	 * e.g. a 0 of bit 4 in s0[3] and 1 in the same bit of s1[3] means that
	 * the hey in col #4 and row #5 has been pressed which equals
	 * keycode = jz_kbd_keycode[4 * 5]
	 */
	 for (col_cnt=0; col_cnt<KB_COLS; col_cnt++) {
	 	if (s0[col_cnt] != s1[col_cnt]) {
	 		for (row_cnt=0; row_cnt<KB_ROWS; row_cnt++) {
	 			r = 1 << row_cnt;
	 			if ((s0[col_cnt]&r) != (s1[col_cnt]&r)) {
	 				if (s0[col_cnt]&r) {
	 					input_report_key(g_jz_kbd.input, jz_kbd_keycode[col_cnt][row_cnt], 0);
	 					input_sync(g_jz_kbd.input);
#ifdef DEBUG
	 					printk(KERN_ERR "released col %d row %d = code %d\n", col_cnt, row_cnt, jz_kbd_keycode[col_cnt][row_cnt]);
#endif
					} else {
						input_report_key(g_jz_kbd.input, jz_kbd_keycode[col_cnt][row_cnt], 1);
						input_sync(g_jz_kbd.input);
#ifdef DEBUG
	 					printk(KERN_ERR "pressed col %d row %d = code %d\n", col_cnt, row_cnt, jz_kbd_keycode[col_cnt][row_cnt]);
#endif
	}
	}
			}
	 		s1[col_cnt] = s0[col_cnt];
		}
#ifdef DEBUG
	 	printk (KERN_ERR "%02x ", s0[i]);
#endif
	}
#ifdef DEBUG
	printk (KERN_ERR "\n");
#endif
//	spin_unlock_irqrestore(&kbd_data->lock, flags);
}

static void jz_kbd_timer_callback(unsigned long data)
{
	jz_kbd_scankeyboard(&g_jz_kbd);
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);
}

#ifdef CONFIG_PM
static int jz_kbd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);
	jz_kbd->suspended = 1;
	
	return 0;
}

static int jz_kbd_resume(struct platform_device *dev)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);
	
	jz_kbd->suspend_jiffies = jiffies;
	jz_kbd->suspended = 0;
	
	return 0;
}
#else
#define jz_kbd_suspend NULL
#define jz_kbd_resume      NULL
#endif

static int __devinit jz_kbd_probe(struct platform_device *dev)
{
	struct input_dev *input_dev;
	int i, error, col_cnt, row_cnt;

#ifdef DEBUG
	printk(KERN_ERR "jz_keypad: probe() called\n");
#endif

	input_dev = input_allocate_device();
	if (!input_dev) {
		printk(KERN_ERR "jz_keypad: could not allocate input device\n");
		return -ENOMEM;
	}
#ifdef DEBUG
	 else
		printk(KERN_ERR "jz_keypad: allocated input device\n");
#endif
	
	platform_set_drvdata(dev, &g_jz_kbd);
    
	strcpy(g_jz_kbd.phys, "input/kbd0");
	
	spin_lock_init(&g_jz_kbd.lock);
	
	g_jz_kbd.suspend_jiffies = jiffies;
	g_jz_kbd.input = input_dev;
	
	input_dev->private = &g_jz_kbd;
	input_dev->name = "JZ Keypad";
	input_dev->phys = g_jz_kbd.phys;
	input_dev->cdev.dev = &dev->dev;

	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP) | BIT(EV_SYN);
	input_dev->keycode = jz_kbd_keycode /*g_jz_kbd.keycode*/; /* keycode array address */
	input_dev->keycodesize = sizeof(unsigned int);
	input_dev->keycodemax = KB_ROWS * KB_COLS /*ARRAY_SIZE(jz_kbd_keycode)*/;

	// memcpy(g_jz_kbd.keycode, jz_kbd_keycode, sizeof(g_jz_kbd.keycode));

//	for (i = 0; i < ARRAY_SIZE(jz_kbd_keycode); i++)
//		set_bit(g_jz_kbd.keycode[i], input_dev->keybit);
	
	for (col_cnt = 0; col_cnt < KB_COLS; col_cnt++)
		for (row_cnt = 0; row_cnt < KB_ROWS; row_cnt++)
			set_bit(jz_kbd_keycode[col_cnt][row_cnt], input_dev->keybit);
	
	clear_bit(0, input_dev->keybit);

	for (row_cnt=0; row_cnt<KB_ROWS; row_cnt++) {
		__gpio_enable_pull(row[row_cnt]);
		__gpio_as_input(row[row_cnt]);
	}
	for (col_cnt=0; col_cnt<KB_COLS; col_cnt++) {
		__gpio_as_input(col[col_cnt]);
		__gpio_clear_pin(col[col_cnt]);
	}

	error = input_register_device(input_dev);
	if (error) {
		pr_err("jz-keypad: Unable to register input device, "
			"error: %d\n", error);
		udelay(1000000);
		input_free_device(input_dev);
		platform_set_drvdata(dev, NULL);
		return error;
	}
#ifdef DEBUG
	 else
		printk(KERN_ERR "jz_keypad: registered input device\n");
#endif

	/* Init Keyboard rescan timer */
	init_timer(&g_jz_kbd.timer);
	g_jz_kbd.timer.function = jz_kbd_timer_callback;
	g_jz_kbd.timer.data = (unsigned long)&g_jz_kbd;
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);

	device_init_wakeup(&dev->dev, 1);
#ifdef DEBUG
	printk(KERN_ERR "jz_keypad: timer setup, probe done\n");
#endif
	return 0;
}

static int __devexit jz_kbd_remove(struct platform_device *dev)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);
	int row_cnt, col_cnt;
	 
	del_timer_sync(&jz_kbd->timer);
	
#if 0
	__gpio_as_input(85);
	__gpio_as_input(87);
	__gpio_as_input(91);

	/* These pins is conficting with cs8900a's CS RD WE pins on JZ4740-PAVO board */
	__gpio_as_input(60);
	__gpio_as_input(61);
	__gpio_as_input(62);	 
#else
	for (row_cnt=0; row_cnt<KB_ROWS; row_cnt++)
		__gpio_as_input(row[row_cnt]);
	for (col_cnt=0; col_cnt<KB_COLS; col_cnt++)
		__gpio_as_input(col[col_cnt]);
#endif
	input_unregister_device(jz_kbd->input);

	return 0;
}

struct platform_driver jz_kbd_driver = {
	.probe      = jz_kbd_probe,
	.remove     = __devexit_p(jz_kbd_remove),
	.suspend    = jz_kbd_suspend,
	.resume     = jz_kbd_resume,
	.driver     = {
		.name   = "jz-keypad",
	}
};

/*
 * Jz Keyboard Device
 */
static struct platform_device jzkbd_device = {
	.name		= "jz-keypad",
	.id		= -1,
	.num_resources	= 0,
};

static int __init jz_kbd_init(void)
{
int err=0;

#ifdef DEBUG
	printk(KERN_ERR "jz_keypad: registering\n");
#endif
	platform_device_register(&jzkbd_device);
	err = platform_driver_register(&jz_kbd_driver);

	if (err) {
		printk(KERN_ERR "jz_keypad: platform_driver_register() failed\n\n");
	}
#ifdef DEBUG
	 else
		printk(KERN_ERR "jz_keypad: registered\n");
#endif

	return err;
}

static void __exit jz_kbd_exit(void)
{
	platform_device_unregister(&jzkbd_device);
	platform_driver_unregister(&jz_kbd_driver);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("JZ keypad driver");
MODULE_LICENSE("GPL");
