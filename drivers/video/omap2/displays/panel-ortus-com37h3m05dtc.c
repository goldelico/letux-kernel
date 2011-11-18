/*
 * Ortustech COM37H3M05DTC panel support
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

#include <video/omapdss.h>
#include <linux/gpio.h>

static struct omap_video_timings com37h3m05dtc_panel_timings = {
	.x_res		= 480,
	.y_res		= 640,
	.pixel_clock	= 22400,
	.hfp		= 2,
	.hsw		= 2,
	.hbp		= 9,
	.vfp		= 2,
	.vsw		= 1,
	.vbp		= 3,
};


// FIXME: this should be passed from the board initialization structure or should be set by driver parameters
#define GPIO_STBY 158

static int com37h3m05dtc_panel_probe(struct omap_dss_device *dssdev)
{
	int rc = 0;
	printk("com37h3m05dtc_panel_probe()\n");
	/* not set: OMAP_DSS_LCD_IEO, OMAP_DSS_LCD_IPC, ACBI */
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = com37h3m05dtc_panel_timings;
	rc = gpio_request(GPIO_STBY, "COM37_STBY");
	if(rc < 0)
		printk(KERN_ERR "Unable to get COM37_STBY GPIO %d\n", GPIO_STBY);
	gpio_direction_output(GPIO_STBY, true);
	
	return rc;
}

static void com37h3m05dtc_panel_remove(struct omap_dss_device *dssdev)
{
	printk("com37h3m05dtc_panel_remove()\n");
	gpio_free(GPIO_STBY);
}

static int com37h3m05dtc_panel_suspend(struct omap_dss_device *dssdev)
{ // set STBY to 1
	printk("com37h3m05dtc_panel_suspend()\n");
	gpio_set_value(GPIO_STBY, 0);
	return 0;
}

static int com37h3m05dtc_panel_resume(struct omap_dss_device *dssdev)
{ // set STBY to 0
	printk("com37h3m05dtc_panel_resume()\n");
	gpio_set_value(GPIO_STBY, 1);
	return 0;
}

static int com37h3m05dtc_panel_enable(struct omap_dss_device *dssdev)
{
	int rc = 0;
	
	printk("com37h3m05dtc_panel_enable()\n");
	if (dssdev->platform_enable)
		rc = dssdev->platform_enable(dssdev);	// enable e.g. power, backlight

	if(rc)
		return rc;

	// 1. standby_to_sleep()
	
	// 2. sleep_to_normal()
	
	com37h3m05dtc_panel_resume(dssdev);
	
	return rc ? -EIO : 0;
}

static void com37h3m05dtc_panel_disable(struct omap_dss_device *dssdev)
{
	
	printk("com37h3m05dtc_panel_disable()\n");
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	
	// 1. normal_to_sleep()
	
	com37h3m05dtc_panel_suspend(dssdev);

	// 2. sleep_to_standby()
	
	// FIXME
}

static struct omap_dss_driver com37h3m05dtc_driver = {
	.probe		= com37h3m05dtc_panel_probe,
	.remove		= com37h3m05dtc_panel_remove,

	.enable		= com37h3m05dtc_panel_enable,
	.disable	= com37h3m05dtc_panel_disable,
	.suspend	= com37h3m05dtc_panel_suspend,
	.resume		= com37h3m05dtc_panel_resume,

	.driver         = {
		.name   = "com37h3m05dtc_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init com37h3m05dtc_panel_drv_init(void)
{
	return omap_dss_register_driver(&com37h3m05dtc_driver);
}

static void __exit com37h3m05dtc_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&com37h3m05dtc_driver);
}

module_init(com37h3m05dtc_panel_drv_init);
module_exit(com37h3m05dtc_panel_drv_exit);
MODULE_LICENSE("GPL");
