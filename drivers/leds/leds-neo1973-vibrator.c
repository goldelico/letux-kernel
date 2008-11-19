/*
 * LED driver for the vibrator of the Openmoko GTA01/GTA02 GSM Phones
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Javi Roman <javiroman@kernel-labs.org>:
 *	Implement PWM support for GTA01Bv4 and later
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <mach/pwm.h>
#include <mach/gta01.h>
#include <plat/regs-timer.h>

#ifdef CONFIG_MACH_NEO1973_GTA02
#include <mach/fiq_ipc_gta02.h>
#endif
#include <asm/plat-s3c24xx/neo1973.h>

#define COUNTER 64

struct neo1973_vib_priv {
	struct led_classdev cdev;
	unsigned int gpio;
	spinlock_t lock;
	unsigned int has_pwm;
	struct s3c2410_pwm pwm;
};

static void neo1973_vib_vib_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	unsigned long flags;
	struct neo1973_vib_priv *vp = container_of(led_cdev,
						   struct neo1973_vib_priv,
						   cdev);

#ifdef CONFIG_MACH_NEO1973_GTA02
	if (machine_is_neo1973_gta02()) { /* use FIQ to control GPIO */
		fiq_ipc.vib_pwm = value; /* set it for FIQ */
		fiq_kick(); /* start up FIQs if not already going */
		return;
	}
#endif
	/*
	 * value == 255 -> 99% duty cycle (full power)
	 * value == 128 -> 50% duty cycle (medium power)
	 * value == 0 -> 0% duty cycle (zero power)
	 */
	spin_lock_irqsave(&vp->lock, flags);
	if (vp->has_pwm) {
		s3c2410_pwm_duty_cycle(value / 4, &vp->pwm);
	}
	else {
		neo1973_gpb_setpin(vp->gpio, value ? 1 : 0);
	}
	spin_unlock_irqrestore(&vp->lock, flags);
}

static struct neo1973_vib_priv neo1973_vib_led = {
	.cdev = {
		.name = "neo1973:vibrator",
		.brightness_set = neo1973_vib_vib_set,
	},
};

static int neo1973_vib_init_hw(struct neo1973_vib_priv *vp)
{
	int rc;

	rc = s3c2410_pwm_init(&vp->pwm);
	if (rc)
		return rc;

	vp->pwm.timerid = PWM3;
	/* use same prescaler as arch/arm/plat-s3c24xx/time.c */
	vp->pwm.prescaler = (6 - 1) / 2;
	vp->pwm.divider = S3C2410_TCFG1_MUX3_DIV2;
	vp->pwm.counter = COUNTER;
	vp->pwm.comparer = COUNTER;

	rc = s3c2410_pwm_enable(&vp->pwm);
	if (rc)
		return rc;

	s3c2410_pwm_start(&vp->pwm);

	return 0;
}

#ifdef CONFIG_PM
static int neo1973_vib_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&neo1973_vib_led.cdev);
	return 0;
}

static int neo1973_vib_resume(struct platform_device *dev)
{
	struct neo1973_vib_priv *vp = platform_get_drvdata(dev);

	if (vp->has_pwm)
		neo1973_vib_init_hw(vp);

	led_classdev_resume(&neo1973_vib_led.cdev);

	return 0;
}
#endif /* CONFIG_PM */

static int __init neo1973_vib_probe(struct platform_device *pdev)
{
	struct resource *r;
	int rc;

	if (!machine_is_neo1973_gta01() && !machine_is_neo1973_gta02())
		return -EIO;

	r = platform_get_resource(pdev, 0, 0);
	if (!r || !r->start)
		return -EIO;

	neo1973_vib_led.gpio = r->start;
	platform_set_drvdata(pdev, &neo1973_vib_led);

#ifdef CONFIG_MACH_NEO1973_GTA02
	if (machine_is_neo1973_gta02()) { /* use FIQ to control GPIO */
		neo1973_gpb_setpin(neo1973_vib_led.gpio, 0); /* off */
		s3c2410_gpio_cfgpin(neo1973_vib_led.gpio, S3C2410_GPIO_OUTPUT);
		/* safe, kmalloc'd copy needed for FIQ ISR */
		fiq_ipc.vib_gpio_pin = neo1973_vib_led.gpio;
		fiq_ipc.vib_pwm = 0; /* off */
		goto configured;
	}
#endif

	/* TOUT3 */
	if (neo1973_vib_led.gpio == S3C2410_GPB3) {
		rc = neo1973_vib_init_hw(&neo1973_vib_led);
		if (rc)
			return rc;

		s3c2410_pwm_duty_cycle(0, &neo1973_vib_led.pwm);
		s3c2410_gpio_cfgpin(neo1973_vib_led.gpio, S3C2410_GPB3_TOUT3);
		neo1973_vib_led.has_pwm = 1;
	}
#ifdef CONFIG_MACH_NEO1973_GTA02
configured:
#endif
	spin_lock_init(&neo1973_vib_led.lock);

	return led_classdev_register(&pdev->dev, &neo1973_vib_led.cdev);
}

static int neo1973_vib_remove(struct platform_device *pdev)
{
#ifdef CONFIG_MACH_NEO1973_GTA02
	if (machine_is_neo1973_gta02()) /* use FIQ to control GPIO */
		fiq_ipc.vib_pwm = 0; /* off */
	/* would only need kick if already off so no kick needed */
#endif

	if (neo1973_vib_led.has_pwm)
		s3c2410_pwm_disable(&neo1973_vib_led.pwm);

	led_classdev_unregister(&neo1973_vib_led.cdev);

	return 0;
}

static struct platform_driver neo1973_vib_driver = {
	.probe		= neo1973_vib_probe,
	.remove		= neo1973_vib_remove,
#ifdef CONFIG_PM
	.suspend	= neo1973_vib_suspend,
	.resume		= neo1973_vib_resume,
#endif
	.driver		= {
		.name		= "neo1973-vibrator",
	},
};

static int __init neo1973_vib_init(void)
{
	return platform_driver_register(&neo1973_vib_driver);
}

static void __exit neo1973_vib_exit(void)
{
	platform_driver_unregister(&neo1973_vib_driver);
}

module_init(neo1973_vib_init);
module_exit(neo1973_vib_exit);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Openmoko GTA01/GTA02 vibrator driver");
MODULE_LICENSE("GPL");
