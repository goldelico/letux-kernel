/* linux/drivers/misc/smdk6410-sleeptest.c
 *
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <plat/gpio-cfg.h>

static irqreturn_t sleep_action(int irq, void *pw)
{
	printk(KERN_INFO "%s: irq %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static void sleep_setup(unsigned int irq, unsigned int gpio)
{
	int ret;

	WARN_ON(s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2)) < 0);
	WARN_ON(s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP) < 0);

	ret = request_irq(irq, sleep_action, IRQF_TRIGGER_FALLING,
			  "sleep", NULL);
	if (ret < 0)
		printk(KERN_ERR "%s: request_irq() failed\n", __func__);

	ret = set_irq_wake(irq, 1);
	if (ret < 0)
		printk(KERN_ERR "%s: set_irq_wake() failed\n", __func__);
}

static void sleep_led(unsigned int gpio)
{
//	gpio_request(gpio, "sleep led");
//	gpio_direction_output(gpio, 0);
}

static __init int smdk6410_sleeptest_init(void)
{
	sleep_setup(IRQ_EINT(10), S3C64XX_GPN(10));
//	sleep_led(S3C64XX_GPN(15));
//	sleep_led(S3C64XX_GPN(14));
//	sleep_led(S3C64XX_GPN(13));
//	sleep_led(S3C64XX_GPN(12));

	return 0;
}

module_init(smdk6410_sleeptest_init);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");
