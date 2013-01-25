/*
 * Driver for Camera Module Board used in GTA04
 *
 * this should be integrated into the board file, but it did not compile
 *
 * adapted from gta04Board (XM) camera driver
 * http://gitorious.org/gta04board-validation/linux/commits/gta04bardXM-camwork
 *
 * please note that the MT9 cameras are not compatible to the GTA04 hardware!
 *
 * Copyright (C) 2010 Texas Instruments Inc
 * Author: Sergio Aguirre <saaguirre@ti.com>
 *
 * Based on work done by:
 *     Vaibhav Hiremath <hvaibhav@ti.com>
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <media/v4l2-int-device.h>

#include "control.h"
#include "mux.h"

#define CAM_USE_XCLKA		0

#define CAMERA_RESET_GPIO	98	/* CAM_FLD */
#define CAMERA_PWDN_GPIO	165	/* CAM_WEN */
#define CAMERA_STROBE_GPIO	126	/* CAM_STROBE */

static struct regulator *cam_2v5_reg;

#if defined(CONFIG_VIDEO_OV9655) || defined(CONFIG_VIDEO_OV9655_MODULE)
#include <media/ov9655.h>

#define ISP_OV9655_MCLK	(216*1000000)

/* Arbitrary memory handling limit */
#define OV9655_MAX_FRAME_SIZE	PAGE_ALIGN(1280 * 1024 * 4)

// FIXME: adjust sync polarity to what the OV9655 delivers

static struct isp_interface_config ov9655_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
//	.ccdc_par_ser		= ISP_PARLL_YUV_BT,
	.dataline_shift		= ISPCTRL_SHIFT_2>>ISPCTRL_SHIFT_SHIFT,
//	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_HSFALL,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.cam_mclk		= ISP_OV9655_MCLK,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 2,
//	.u.par.par_bridge	= 0x2,
//	.u.par.par_bridge	= ISPCTRL_PAR_BRIDGE_LENDIAN>>ISPCTRL_PAR_BRIDGE_SHIFT,
	.u.par.par_bridge	= ISPCTRL_PAR_BRIDGE_BENDIAN>>ISPCTRL_PAR_BRIDGE_SHIFT,
	.u.par.par_clk_pol	= 0&ISPCTRL_PAR_CLK_POL_INV>>ISPCTRL_PAR_CLK_POL_SHIFT,
//	.u.par.par_clk_pol	= ISPCTRL_PAR_CLK_POL_INV>>ISPCTRL_PAR_CLK_POL_SHIFT,
};

static struct v4l2_ifparm ov9655_ifparm_s = {
	.if_type = V4L2_IF_TYPE_YCbCr,
	.u 	 = {
		.ycbcr = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,
			.nobt_vs_inv		= 0,
			.clock_min		= OV9655_CLK_MIN,
			.clock_max		= OV9655_CLK_MAX,
		},
	},
};

/**
 * @brief ov9655_ifparm - Returns the ov9655 interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int ov9655_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;
	
	*p = ov9655_ifparm_s;
	return 0;
}

static struct omap34xxcam_hw_config ov9655_hwc = {
	.dev_index		= 0,
	.dev_minor		= 0,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 1,
	.u.sensor.capture_mem	= OV9655_MAX_FRAME_SIZE * 2,
	.u.sensor.ival_default	= { 1, 10 },
};

/**
 * @brief ov9655_set_prv_data - Returns ov9655 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int ov9655_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;
	
	if (priv == NULL)
		return -EINVAL;
	
	*hwc = ov9655_hwc;
	return 0;
}

/**
 * @brief ov9655_power_set - Power-on or power-off OV9655 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int ov9655_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	switch (power) {
		case V4L2_POWER_STANDBY:
			/* activate powerdown??? */
			printk("ov9655_power_set(V4L2_POWER_STANDBY): %d\n", power);

			break;
			
		case V4L2_POWER_OFF:
			printk("ov9655_power_set(V4L2_POWER_OFF): %d\n", power);
			isp_set_xclk(vdev->cam->isp, 0, CAM_USE_XCLKA);
			
			if (regulator_is_enabled(cam_2v5_reg))
				regulator_disable(cam_2v5_reg);
			
			break;
			
		case V4L2_POWER_ON:
			
			printk("ov9655_power_set(V4L2_POWER_ON): %d\n", power);
			isp_configure_interface(vdev->cam->isp, &ov9655_if_config);
			/* Set RESET_BAR to 0 (this is the definition for the Rev 5 camera chip!) */
			gpio_set_value(CAMERA_RESET_GPIO, 0);
			/* remove powerdown signal */
			gpio_set_value(CAMERA_PWDN_GPIO, 0);
			
			/* turn on VDD */
			regulator_enable(cam_2v5_reg);
			mdelay(50);
			
			/* Enable EXTCLK */
			isp_set_xclk(vdev->cam->isp, OV9655_CLK_MIN*2, CAM_USE_XCLKA);
			/*
			 * Wait at least 70 CLK cycles (w/EXTCLK = 6MHz, or CLK_MIN):
			 * ((1000000 * 70) / 6000000) = aprox 12 us.
			 */
			udelay(12);
			/* Set RESET_BAR to 1 */
			gpio_set_value(CAMERA_RESET_GPIO, 1);
			/*
			 * Wait at least 1 ms
			 */
			mdelay(1000);
			
			break;
			
		default:
			return -ENODEV;
	}
	
	return 0;
}

struct ov9655_platform_data ov9655_pdata = {
	.master		= "omap34xxcam",
	.power_set	= ov9655_power_set,
	.priv_data_set	= ov9655_set_prv_data,
	.ifparm		= ov9655_ifparm,
};

#endif				/* #ifdef CONFIG_VIDEO_OV9655 */

static int gta04_cam_probe(struct platform_device *pdev)
{
	cam_2v5_reg = regulator_get(&pdev->dev, "cam_2v5");
	if (IS_ERR(cam_2v5_reg)) {
		dev_err(&pdev->dev, "cam_2v5 regulator missing\n");
		return PTR_ERR(cam_2v5_reg);
	}

	if (gpio_request(CAMERA_RESET_GPIO, "cam_rst") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
			CAMERA_RESET_GPIO);
		regulator_put(cam_2v5_reg);
		return -ENODEV;
	}

	if (gpio_request(CAMERA_PWDN_GPIO, "cam_pwdn") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
				CAMERA_PWDN_GPIO);
		gpio_free(CAMERA_RESET_GPIO);
		regulator_put(cam_2v5_reg);
		return -ENODEV;
	}
	
	/* FIXME: this should have been done by U-Boot!
	 * Or we should use the pinmux framework similar to this:
	 *		omap_mux_init_signal("mcbsp3_clkx.uart2_tx", OMAP_PIN_OUTPUT);	// gpio 142 / GPS TX
	 */
	
	/* set to output mode + default value */
	gpio_direction_output(CAMERA_RESET_GPIO, 0);	/* 0: activate reset */
	gpio_direction_output(CAMERA_PWDN_GPIO, 1);		/* 1: activate power down */

#if 1	// standard mode
	/* MUX init */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x10C); /* CAM_HS */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x10E); /* CAM_VS */
	omap_ctrl_writew(OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
			 0x110); /* CAM_XCLKA */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x112); /* CAM_PCLK */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x116); /* CAM_D0 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x118); /* CAM_D1 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11A); /* CAM_D2 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11C); /* CAM_D3 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11E); /* CAM_D4 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x120); /* CAM_D5 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x122); /* CAM_D6 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x124); /* CAM_D7 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x126); /* CAM_D8 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x128); /* CAM_D9 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x12A); /* CAM_D10 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x12C); /* CAM_D11 */

#else	// special experimental mode - map most camera lines to GPIOs in /sys
	
	omap_ctrl_writew(OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
					 0x110); /* CAM_XCLKA */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
					 0x112); /* CAM_PCLK */
	{ // map camera interface to GPIOs
		int i;
		for(i=94; i <= 110; i++) {
			gpio_request(i, "camera");
			omap_mux_init_gpio(i, OMAP_PIN_INPUT_PULLUP);
			gpio_direction_input(i);
			gpio_export(i, 0);	// no direction change through /sys
			if(i == 95) i+=3;
		}
	}

#endif	
	
	printk(KERN_INFO "omap3gta04lmb: Driver registration complete\n");

	return 0;
}

static int gta04_cam_remove(struct platform_device *pdev)
{
	if (regulator_is_enabled(cam_2v5_reg))
		regulator_disable(cam_2v5_reg);
	regulator_put(cam_2v5_reg);

	gpio_free(CAMERA_RESET_GPIO);
	gpio_free(CAMERA_PWDN_GPIO);

	return 0;
}

static int gta04_cam_suspend(struct device *dev)
{
	/* activate powerdown? */
	return 0;
}

static int gta04_cam_resume(struct device *dev)
{
	/* deactivate powerdown? */
	return 0;
}

static struct dev_pm_ops gta04_cam_pm_ops = {
	.suspend = gta04_cam_suspend,
	.resume  = gta04_cam_resume,
};

static struct platform_driver gta04_cam_driver = {
	.probe		= gta04_cam_probe,
	.remove		= gta04_cam_remove,
	.driver		= {
		.name	= "gta04_cam",
		.pm	= &gta04_cam_pm_ops,
	},
};

/**
 * @brief omap3gta04lmb_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
int __init omap3gta04lmb_init(void)
{
	platform_driver_register(&gta04_cam_driver);

	return 0;
}
late_initcall(omap3gta04lmb_init);
