/*
 *  Backlight Driver for FIC GTA01 (Neo1973) GSM Phone
 *
 * Copyright (C) 2006-2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 *  based on corgi_cl.c, Copyright (c) 2004-2006 Richard Purdie
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Javi Roman <javiroman@kernel-labs.org>:
 * 	implement PWM, instead of simple on/off switching
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/clk.h>

#include <asm/arch/hardware.h>
#include <asm/arch/gta01.h>
#include <asm/arch/pwm.h>

#include <asm/plat-s3c/regs-timer.h>
#include <asm/plat-s3c24xx/neo1973.h>

static struct backlight_properties gta01bl_prop;
static struct backlight_device *gta01_backlight_device;
static struct gta01bl_machinfo *bl_machinfo;

static unsigned long gta01bl_flags;

struct gta01bl_data {
	int intensity;
	struct mutex mutex;
	struct clk *clk;
	struct s3c2410_pwm pwm;
};

static struct gta01bl_data gta01bl;

static int gta01bl_defer_resume_backlight;

#define GTA01BL_SUSPENDED     0x01
#define GTA01BL_BATTLOW       0x02

/* On the GTA01 / Neo1973, we use a 50 or 66MHz PCLK, which gives
 * us a 6.25..8.25MHz DIV8 clock, which is further divided by a
 * prescaler of 4, resulting in a 1.56..2.06MHz tick.  This results in a
 * minimum frequency of 24..31Hz.  At 400Hz, we need to set the count
 * to something like 3906..5156, providing us a way sufficient resolution
 * for display brightness adjustment. */
#define GTA01BL_COUNTER 5156

static int gta01bl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (gta01bl_flags & GTA01BL_SUSPENDED)
		intensity = 0;
	if (gta01bl_flags & GTA01BL_BATTLOW)
		intensity &= bl_machinfo->limit_mask;

	mutex_lock(&gta01bl.mutex);
#ifdef GTA01_BACKLIGHT_ONOFF_ONLY
	if (intensity)
		neo1973_gpb_setpin(GTA01_GPIO_BACKLIGHT, 1);
	else
		neo1973_gpb_setpin(GTA01_GPIO_BACKLIGHT, 0);
#else
	if (intensity == bd->props.max_brightness) {
		neo1973_gpb_setpin(GTA01_GPIO_BACKLIGHT, 1);
		s3c2410_gpio_cfgpin(GTA01_GPIO_BACKLIGHT, S3C2410_GPIO_OUTPUT);
	} else  {
		s3c2410_pwm_duty_cycle(intensity & 0xffff, &gta01bl.pwm);
		s3c2410_gpio_cfgpin(GTA01_GPIO_BACKLIGHT, S3C2410_GPB0_TOUT0);
	}
#endif
	mutex_unlock(&gta01bl.mutex);

	gta01bl.intensity = intensity;
	return 0;
}

static int gta01bl_init_hw(void)
{
	int rc;

	rc = s3c2410_pwm_init(&gta01bl.pwm);
	if (rc)
		return rc;

	gta01bl.pwm.timerid = PWM0;
	gta01bl.pwm.prescaler = (4 - 1);
	gta01bl.pwm.divider = S3C2410_TCFG1_MUX0_DIV8;
	gta01bl.pwm.counter = GTA01BL_COUNTER;
	gta01bl.pwm.comparer = gta01bl.pwm.counter;

	rc = s3c2410_pwm_enable(&gta01bl.pwm);
	if (rc)
		return rc;

	s3c2410_pwm_start(&gta01bl.pwm);

	gta01bl_prop.max_brightness = gta01bl.pwm.counter;

	return 0;
}

#ifdef CONFIG_PM
static int gta01bl_suspend(struct platform_device *dev, pm_message_t state)
{
	gta01bl_flags |= GTA01BL_SUSPENDED;
	gta01bl_send_intensity(gta01_backlight_device);
	neo1973_gpb_setpin(GTA01_GPIO_BACKLIGHT, 0);
	s3c2410_gpio_cfgpin(GTA01_GPIO_BACKLIGHT, S3C2410_GPIO_OUTPUT);
	return 0;
}

void gta01bl_deferred_resume(void)
{
	mutex_lock(&gta01bl.mutex);
	gta01bl_init_hw();
	mutex_unlock(&gta01bl.mutex);

	gta01bl_flags &= ~GTA01BL_SUSPENDED;
	gta01bl_send_intensity(gta01_backlight_device);
}
EXPORT_SYMBOL_GPL(gta01bl_deferred_resume);

static int gta01bl_resume(struct platform_device *dev)
{
	if (! gta01bl_defer_resume_backlight)
		gta01bl_deferred_resume();
	return 0;
}
#else
#define gta01bl_suspend	NULL
#define gta01bl_resume	NULL
#endif

static int gta01bl_get_intensity(struct backlight_device *bd)
{
	return gta01bl.intensity;
}

static int gta01bl_set_intensity(struct backlight_device *bd)
{
	gta01bl_send_intensity(gta01_backlight_device);
	return 0;
}

/*
 * Called when the battery is low to limit the backlight intensity.
 * If limit==0 clear any limit, otherwise limit the intensity
 */
void gta01bl_limit_intensity(int limit)
{
	if (limit)
		gta01bl_flags |= GTA01BL_BATTLOW;
	else
		gta01bl_flags &= ~GTA01BL_BATTLOW;
	gta01bl_send_intensity(gta01_backlight_device);
}
EXPORT_SYMBOL_GPL(gta01bl_limit_intensity);


static struct backlight_ops gta01bl_ops = {
	.get_brightness = gta01bl_get_intensity,
	.update_status  = gta01bl_set_intensity,
};

static int __init gta01bl_probe(struct platform_device *pdev)
{
	struct gta01bl_machinfo *machinfo = pdev->dev.platform_data;
	int rc;

#ifdef GTA01_BACKLIGHT_ONOFF_ONLY
	s3c2410_gpio_cfgpin(GTA01_GPIO_BACKLIGHT, S3C2410_GPIO_OUTPUT);
	gta01bl_prop.max_brightness = 1;
#else
	rc = gta01bl_init_hw();
	if (rc < 0)
		return rc;
#endif
	mutex_init(&gta01bl.mutex);

	if (!machinfo->limit_mask)
		machinfo->limit_mask = -1;

	gta01bl_defer_resume_backlight = machinfo->defer_resume_backlight;

	gta01_backlight_device = backlight_device_register("gta01-bl",
							   &pdev->dev, NULL,
							   &gta01bl_ops);
	if (IS_ERR(gta01_backlight_device))
		return PTR_ERR(gta01_backlight_device);

	gta01bl_prop.power = FB_BLANK_UNBLANK;
	gta01bl_prop.brightness = gta01bl_prop.max_brightness;
	memcpy(&gta01_backlight_device->props,
	       &gta01bl_prop, sizeof(gta01bl_prop));
	gta01bl_send_intensity(gta01_backlight_device);

	return 0;
}

static int gta01bl_remove(struct platform_device *dev)
{
#ifndef GTA01_BACKLIGHT_ONOFF_ONLY
	s3c2410_pwm_disable(&gta01bl.pwm);
#endif
	backlight_device_unregister(gta01_backlight_device);
	mutex_destroy(&gta01bl.mutex);

	s3c2410_gpio_cfgpin(GTA01_GPIO_BACKLIGHT, S3C2410_GPIO_OUTPUT);
	neo1973_gpb_setpin(GTA01_GPIO_BACKLIGHT, 1);

	return 0;
}

static struct platform_driver gta01bl_driver = {
	.probe		= gta01bl_probe,
	.remove		= gta01bl_remove,
	.suspend	= gta01bl_suspend,
	.resume		= gta01bl_resume,
	.driver		= {
		.name	= "gta01-bl",
	},
};

static int __init gta01bl_init(void)
{
	return platform_driver_register(&gta01bl_driver);
}

static void __exit gta01bl_exit(void)
{
	platform_driver_unregister(&gta01bl_driver);
}

module_init(gta01bl_init);
module_exit(gta01bl_exit);

MODULE_DESCRIPTION("FIC GTA01 (Neo1973) Backlight Driver");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");
