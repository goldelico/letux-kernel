/*
 *  Backlight Driver for Ingenic JZ4730 based MiniPC
 *
 *  Copyright (c) 2008 Nils Faerber
 *
 *  Based on Richard Purdie's Corgi Backlight Driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>


static int minipc_bl_intensity;
static struct backlight_properties minipc_bl_data;
static struct backlight_device *minipc_backlight_device;
static struct generic_bl_info *bl_machinfo;

static unsigned long minipc_bl_flags;
#define MINIPC_BL_SUSPENDED     0x01
#define MINIPC_BL_BATTLOW       0x02

static int minipc_bl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (minipc_bl_flags & MINIPC_BL_SUSPENDED)
		intensity = 0;
	if (minipc_bl_flags & MINIPC_BL_BATTLOW)
		intensity &= bl_machinfo->limit_mask;

	bl_machinfo->set_bl_intensity(intensity);

	minipc_bl_intensity = intensity;

	return 0;
}

#ifdef CONFIG_PM
static int minipc_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	minipc_bl_flags |= MINIPC_BL_SUSPENDED;
	backlight_update_status(bd);

	return 0;
}

static int minipc_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	minipc_bl_flags &= ~MINIPC_BL_SUSPENDED;
	backlight_update_status(bd);

        return 0;
}
#else

#define minipc_bl_suspend NULL
#define minipc_bl_resume  NULL

#endif

static int minipc_bl_get_intensity(struct backlight_device *bd)
{
	return minipc_bl_intensity;
}

/*
 * Called when the battery is low to limit the backlight intensity.
 * If limit==0 clear any limit, otherwise limit the intensity
 */
void minipc_bl_limit_intensity(int limit)
{
	if (limit)
		minipc_bl_flags |= MINIPC_BL_BATTLOW;
	else
		minipc_bl_flags &= ~MINIPC_BL_BATTLOW;

	backlight_update_status(minipc_backlight_device);
}
EXPORT_SYMBOL(minipc_bl_limit_intensity);

static struct backlight_ops minipc_bl_ops = {
	.get_brightness = minipc_bl_get_intensity,
	.update_status  = minipc_bl_send_intensity,
};


static int minipc_bl_probe(struct platform_device *pdev)
{
	struct generic_bl_info *machinfo = pdev->dev.platform_data;
	const char *name = "generic-bl";

	bl_machinfo = machinfo;
	if (!machinfo->limit_mask)
		machinfo->limit_mask = -1;

	if (machinfo->name)
		name = machinfo->name;

	minipc_backlight_device = backlight_device_register (name, &pdev->dev, NULL, &minipc_bl_ops);
	if (IS_ERR (minipc_backlight_device))
		return PTR_ERR (minipc_backlight_device);

	platform_set_drvdata(pdev, minipc_backlight_device);

	minipc_backlight_device->props.max_brightness = machinfo->max_intensity;
	minipc_backlight_device->props.power = FB_BLANK_UNBLANK;
	minipc_backlight_device->props.brightness = machinfo->default_intensity;
	backlight_update_status(minipc_backlight_device);

	printk("MiniPC Backlight Driver Initialized.\n");
	return 0;
}

static int minipc_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	minipc_bl_data.power = 0;
	minipc_bl_data.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	printk("MiniPC Backlight Driver Unloaded\n");

	return 0;
}

static struct platform_driver minipc_bl_driver = {
	.probe		= minipc_bl_probe,
	.remove		= minipc_bl_remove,
	.suspend	= minipc_bl_suspend,
	.resume		= minipc_bl_resume,
	.driver         = {
		.name   = "generic-bl",
	},
};

static int __init minipc_bl_init(void)
{
	return platform_driver_register(&minipc_bl_driver);
}
        
static void __exit minipc_bl_exit(void)
{
	platform_driver_unregister(&minipc_bl_driver);
}

module_init(minipc_bl_init);
module_exit(minipc_bl_exit);

MODULE_AUTHOR("Nils Faerber <nils.faerber@kernelconcepts.de");
MODULE_DESCRIPTION("MiniPC Backlight Driver");
MODULE_LICENSE("GPL");
