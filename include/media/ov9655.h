/*
 * drivers/media/video/ov9655.h
 *
 * adapted from mt9v113
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Sivaraj R <sivaraj@ti.com>
 *     Brijesh R Jadav <brijesh.j@ti.com>
 *     Hardik Shah <hardik.shah@ti.com>
 *     Manjunath Hadli <mrh@ti.com>
 *     Karicheri Muralidharan <m-karicheri2@ti.com>
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
 *
 */

#ifndef _OV9655_H
#define _OV9655_H

#include <media/v4l2-int-device.h>

/*i2c adress for OV9655*/
#define OV9655_I2C_ADDR  		(0x60 >>1)

#define OV9655_CLK_MAX 	(48000000) /* 48MHz */
#define OV9655_CLK_MIN	(10000000) /* 10Mhz */

/*
 * Other macros
 */
#define OV9655_MODULE_NAME		"ov9655"

/**
 * struct ov9655_platform_data - Platform data values and access functions.
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function.
 * @priv_data_set: Device private data (pointer) access function.
 * @clk_polarity: Clock polarity of the current interface.
 * @ hs_polarity: HSYNC Polarity configuration for current interface.
 * @ vs_polarity: VSYNC Polarity configuration for current interface.
 */
struct ov9655_platform_data {
	char *master;
	int (*power_set) (struct v4l2_int_device *s, enum v4l2_power on);
	int (*ifparm) (struct v4l2_ifparm *p);
	int (*priv_data_set) (void *);
	/* Interface control params */
	bool clk_polarity;
	bool hs_polarity;
	bool vs_polarity;
	
	/* new - needs driver to be rewritten */
	int (*set_xclk)(struct v4l2_subdev *subdev, int hz);
	int reset;	/* GPIO */
	int ext_freq;
	int target_freq;
};

#endif				/* ifndef _OV9655_H */

