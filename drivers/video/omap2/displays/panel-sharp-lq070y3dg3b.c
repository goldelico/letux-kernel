/*
 * Ortustech lq070y3dg3b panel support
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

static struct omap_video_timings lq070y3dg3b_panel_timings = {
#error FIXME
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 33260,
	.hfp		= 64,
	.hsw		= 128,
	.hbp		= 64,
	.vfp		= 8,
	.vsw		= 2,
	.vbp		= 35,
};


// FIXME: this should be passed from the board initialization structure or should be set by driver parameters

#define GPIO_STBY (machine_is_gta04()?12:162)		/* McBSP5-CLKX of GTA04 */

static int lq070y3dg3b_panel_probe(struct omap_dss_device *dssdev)
{
	int rc = 0;
	printk("lq070y3dg3b_panel_probe()\n");
	/* not set: OMAP_DSS_LCD_IEO, OMAP_DSS_LCD_IPC, ACBI */
	
	// FIXME: match polarity
	
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = lq070y3dg3b_panel_timings;
	rc = gpio_request(GPIO_STBY, "LQ070_STBY");
	if(rc < 0)
		printk(KERN_ERR "Unable to get LQ070_STBY GPIO %d\n", GPIO_STBY);
	gpio_direction_output(GPIO_STBY, true);
	
	return rc;
}

static void lq070y3dg3b_panel_remove(struct omap_dss_device *dssdev)
{
	printk("lq070y3dg3b_panel_remove()\n");
	gpio_free(GPIO_STBY);
}

static int lq070y3dg3b_panel_suspend(struct omap_dss_device *dssdev)
{ // set STBY to 1
	printk("lq070y3dg3b_panel_suspend()\n");
	gpio_set_value(GPIO_STBY, 0);	// disable 3.3V LDO
	return 0;
}

static int lq070y3dg3b_panel_resume(struct omap_dss_device *dssdev)
{ // set STBY to 0
	printk("lq070y3dg3b_panel_resume()\n");
	gpio_set_value(GPIO_STBY, 1);	// enable 3.3V LDO
	return 0;
}

static int lq070y3dg3b_panel_enable(struct omap_dss_device *dssdev)
{
	int rc = 0;
	
	printk("lq070y3dg3b_panel_enable()\n");
	if (dssdev->platform_enable)
		rc = dssdev->platform_enable(dssdev);	// enable e.g. power, backlight

	if(rc)
		return rc;

	// 1. standby_to_sleep()
	
	// 2. sleep_to_normal()
	
	lq070y3dg3b_panel_resume(dssdev);
	
	return rc ? -EIO : 0;
}

static void lq070y3dg3b_panel_disable(struct omap_dss_device *dssdev)
{
	
	printk("lq070y3dg3b_panel_disable()\n");
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	
	// 1. normal_to_sleep()
	
	lq070y3dg3b_panel_suspend(dssdev);

	// 2. sleep_to_standby()
	
	// FIXME
}

static struct omap_dss_driver lq070y3dg3b_driver = {
	.probe		= lq070y3dg3b_panel_probe,
	.remove		= lq070y3dg3b_panel_remove,

	.enable		= lq070y3dg3b_panel_enable,
	.disable	= lq070y3dg3b_panel_disable,
	.suspend	= lq070y3dg3b_panel_suspend,
	.resume		= lq070y3dg3b_panel_resume,

	.driver         = {
		.name   = "lq070y3dg3b_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init lq070y3dg3b_panel_drv_init(void)
{
	return omap_dss_register_driver(&lq070y3dg3b_driver);
}

static void __exit lq070y3dg3b_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&lq070y3dg3b_driver);
}

module_init(lq070y3dg3b_panel_drv_init);
module_exit(lq070y3dg3b_panel_drv_exit);
MODULE_LICENSE("GPL");
