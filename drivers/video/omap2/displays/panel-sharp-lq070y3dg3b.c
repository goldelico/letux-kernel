/*
 * Sharp lq070y3dg3b panel support
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

static struct omap_video_timings lq070y3dg3b_panel_timings = {
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 33260,
	.hfp		= 64,
	.hsw		= 128,
	.hbp		= 64,
	.vfp		= 8,
	.vsw		= 2,
	.vbp		= 35,
	
	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,	// OMAP_DSS_LCD_IVS
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,	// OMAP_DSS_LCD_IHS
	
	.data_pclk_edge	= OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge	= OMAPDSS_DRIVE_SIG_RISING_EDGE,	
	
};


// FIXME: this should be passed from the board initialization structure or should be set by driver parameters

#define GPIO_STBY (machine_is_gta04()?12:162)		/* 3.3V LDO controlled through McBSP5-CLKX of GTA04 */

struct panel_drv_data {
	struct mutex lock;
};

static int lq070y3dg3b_panel_probe(struct omap_dss_device *dssdev)
{
	int rc = 0;
	struct panel_drv_data *drv_data = NULL;
	
	drv_data = devm_kzalloc(dssdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;
	mutex_init(&drv_data->lock);
	
	dev_set_drvdata(dssdev->dev, drv_data);
	
	printk("lq070y3dg3b_panel_probe()\n");
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
	// release mutex?
	dev_set_drvdata(dssdev->dev, NULL);
}

static void lq070y3dg3b_panel_disable(struct omap_dss_device *dssdev)
{ // set STBY to 1
	struct panel_drv_data *drv_data = dev_get_drvdata(dssdev->dev);
	printk("lq070y3dg3b_panel_disable()\n");
	mutex_lock(&drv_data->lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		mutex_unlock(&drv_data->lock);
		return;
	}	

	omapdss_dpi_display_disable(dssdev);

	gpio_set_value(GPIO_STBY, 0);	// disable 3.3V LDO
	
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	mutex_unlock(&drv_data->lock);
}

static int lq070y3dg3b_panel_enable(struct omap_dss_device *dssdev)
{ // set STBY to 0
	struct panel_drv_data *drv_data = dev_get_drvdata(dssdev->dev);
	int rc = 0;	
	printk("lq070y3dg3b_panel_enable() - state %d\n", dssdev->state);
	mutex_lock(&drv_data->lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_unlock(&drv_data->lock);
		return rc;
	}
	
	gpio_set_value(GPIO_STBY, 1);	// enable 3.3V LDO
	
	omapdss_dpi_set_timings(dssdev, &dssdev->panel.timings);
	omapdss_dpi_set_data_lines(dssdev, dssdev->phy.dpi.data_lines);
	rc |= omapdss_dpi_display_enable(dssdev);
	if(rc) {
		mutex_unlock(&drv_data->lock);
		return -EIO;
	}
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	mutex_unlock(&drv_data->lock);
	
	return 0;
}

static void lq070y3dg3b_panel_set_timings(struct omap_dss_device *dssdev,
											struct omap_video_timings *timings)
{
	omapdss_dpi_set_timings(dssdev, timings);
	dssdev->panel.timings = *timings;
}

static void lq070y3dg3b_panel_get_timings(struct omap_dss_device *dssdev,
											struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int lq070y3dg3b_panel_check_timings(struct omap_dss_device *dssdev,
											 struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver lq070y3dg3b_driver = {
	.probe		= lq070y3dg3b_panel_probe,
	.remove		= lq070y3dg3b_panel_remove,

	.enable		= lq070y3dg3b_panel_enable,
	.disable	= lq070y3dg3b_panel_disable,

	.set_timings	= lq070y3dg3b_panel_set_timings,
	.get_timings	= lq070y3dg3b_panel_get_timings,
	.check_timings	= lq070y3dg3b_panel_check_timings,
	
	.driver         = {
		.name   = "lq070y3dg3b_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init lq070y3dg3b_panel_drv_init(void)
{
	printk("lq070y3dg3b_panel_drv_init()\n");
	return omap_dss_register_driver(&lq070y3dg3b_driver);
}

static void __exit lq070y3dg3b_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&lq070y3dg3b_driver);
}

module_init(lq070y3dg3b_panel_drv_init);
module_exit(lq070y3dg3b_panel_drv_exit);
MODULE_LICENSE("GPL");
