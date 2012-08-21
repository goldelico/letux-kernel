/*
 * Sharp LQ050W1lC1B panel support
 * also supports LQ050W1LA0A (with touch screen)
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 * Author: H. Nikolaus Schaller <hns@goldelico.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <asm/mach-types.h>

#include <plat/display.h>
#include <linux/gpio.h>

static struct omap_video_timings lq050w1lc1b_panel_timings = {
#error FIXME
	.x_res		= 1024,
	.y_res		= 600,
	.pixel_clock	= 48836,
	.hfp		= 96,
	.hsw		= 96,
	.hbp		= 96,
	.vfp		= 7,
	.vsw		= 7,
	.vbp		= 7,
};


// FIXME: this should be passed from the board initialization structure or should be set by driver parameters

#define GPIO_POWER 12			/* McBSP5-CLKX GTA04 controls 5V power for the display */
#define GPIO_BLSHUTDOWN 19		/* McBSP5-FSX controls Backlight SHUTDOWN (shutdown if high) */
#define GPIO_SHUTDOWN 20		/* McBSP5-DX controls LVDS SHUTDOWN (shutdown if low) */

static int lq050w1lc1b_panel_probe(struct omap_dss_device *dssdev)
{
	int rc = 0;
	printk("lq050w1lc1b_panel_probe()\n");
	/* not set: OMAP_DSS_LCD_IEO, OMAP_DSS_LCD_IPC, ACBI */
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = lq050w1lc1b_panel_timings;
	rc = gpio_request(GPIO_POWER, "LQ050_POWER");
	if(rc < 0)
		printk(KERN_ERR "Unable to get LQ050_POWER GPIO %d\n", GPIO_POWER);
	gpio_direction_output(GPIO_POWER, true);
	rc = gpio_request(GPIO_SHUTDOWN, "LQ050_SHUTDOWN");
	if(rc < 0)
		printk(KERN_ERR "Unable to get LQ050_SHUTDOWN GPIO %d\n", GPIO_SHUTDOWN);
	gpio_direction_output(GPIO_SHUTDOWN, true);
	rc = gpio_request(GPIO_BLSHUTDOWN, "LQ050_BLSHUTDOWN");
	if(rc < 0)
		printk(KERN_ERR "Unable to get LQ050_BLSHUTDOWN GPIO %d\n", GPIO_BLSHUTDOWN);
	gpio_direction_output(GPIO_BLSHUTDOWN, true);
	
	return rc;
}

static void lq050w1lc1b_panel_remove(struct omap_dss_device *dssdev)
{
	printk("lq050w1lc1b_panel_remove()\n");
	gpio_free(GPIO_POWER);
	gpio_free(GPIO_SHUTDOWN);
	gpio_free(GPIO_BLSHUTDOWN);
}

static int lq050w1lc1b_panel_suspend(struct omap_dss_device *dssdev)
{ // set STBY to 1
	printk("lq050w1lc1b_panel_suspend()\n");
	gpio_set_value(GPIO_BLSHUTDOWN, 1);	// disable backlight
	mdelay(5);	// should wait t7 (>5ms)
	gpio_set_value(GPIO_SHUTDOWN, 0);	// switch off LVDS driver
	mdelay(0);		// must wait t3 (>0ms)
	gpio_set_value(GPIO_POWER, 0);		// switch off 5V supply
	mdelay(200);	// must wait t4+t5 (>200ms)
	return 0;
}

static int lq050w1lc1b_panel_resume(struct omap_dss_device *dssdev)
{ // set STBY to 0
	printk("lq050w1lc1b_panel_resume()\n");
	gpio_set_value(GPIO_POWER, 1);		// switch on 5V power
	mdelay(10);		// must wait t1+t2 (>0ms) - this is determined by the DC/DC converter
	gpio_set_value(GPIO_SHUTDOWN, 1);	// switch on LVDS
	mdelay(180);	// should wait t6 (>180ms)
	gpio_set_value(GPIO_BLSHUTDOWN, 0);	// enable backlight control through GPT11
	return 0;
}

static int lq050w1lc1b_panel_enable(struct omap_dss_device *dssdev)
{
	int rc = 0;
	
	printk("lq050w1lc1b_panel_enable()\n");
	if (dssdev->platform_enable)
		rc = dssdev->platform_enable(dssdev);	// enable e.g. power, backlight

	if(rc)
		return rc;

	// 1. standby_to_sleep()
	
	// 2. sleep_to_normal()
	
	lq050w1lc1b_panel_resume(dssdev);
	
	return rc ? -EIO : 0;
}

static void lq050w1lc1b_panel_disable(struct omap_dss_device *dssdev)
{
	
	printk("lq050w1lc1b_panel_disable()\n");
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	
	// 1. normal_to_sleep()
	
	lq050w1lc1b_panel_suspend(dssdev);

	// 2. sleep_to_standby()
	
	// FIXME
}

static struct omap_dss_driver lq050w1lc1b_driver = {
	.probe		= lq050w1lc1b_panel_probe,
	.remove		= lq050w1lc1b_panel_remove,

	.enable		= lq050w1lc1b_panel_enable,
	.disable	= lq050w1lc1b_panel_disable,
	.suspend	= lq050w1lc1b_panel_suspend,
	.resume		= lq050w1lc1b_panel_resume,

	.driver         = {
		.name   = "lq050w1lc1b_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init lq050w1lc1b_panel_drv_init(void)
{
	return omap_dss_register_driver(&lq050w1lc1b_driver);
}

static void __exit lq050w1lc1b_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&lq050w1lc1b_driver);
}

module_init(lq050w1lc1b_panel_drv_init);
module_exit(lq050w1lc1b_panel_drv_exit);
MODULE_LICENSE("GPL");
