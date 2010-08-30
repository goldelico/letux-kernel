/*
 * linux/arch/arm/plat-omap/i2c.c
 *
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2007 Nokia Corporation.
 *
 * Contact: Jarkko Nikula <jhnikula@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <plat/omap_device.h>
#include <plat/hdq.h>

static const char name[] = "omap_hdq";

#define MAX_OMAP_HDQ_HWMOD_NAME_LEN		16

static struct omap_hdq_platform_data omap_hdq_pdata;

struct omap_device_pm_latency omap_hdq_latency[] = {
	[0] = {
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func   = omap_device_enable_hwmods,
		.flags		 = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

struct omap_hdq_platform_data * __init omap_hdq_get_pdata(void)
{
	return &omap_hdq_pdata;
}

/**
 * omap_plat_register_hdq_bus - register HDQ bus with device descriptors
 * @info: pointer into I2C device descriptor table or NULL
 *
 * Returns 0 on success or an error code.
 */
int __init omap_plat_register_hdq_bus(struct omap2_hdq_platform_config *pdata)
{
	int l;
	struct omap_hwmod *oh;
	struct omap_device *od;
	char oh_name[MAX_OMAP_HDQ_HWMOD_NAME_LEN];

	l = snprintf(oh_name, MAX_OMAP_HDQ_HWMOD_NAME_LEN,
		     "hdq1w");
	WARN(l >= MAX_OMAP_HDQ_HWMOD_NAME_LEN,
	     "String buffer overflow in HDQ device setup\n");
	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -EEXIST;
	}

	omap_hdq_pdata.mode = pdata->mode;
	omap_hdq_pdata.id = pdata->id;
	omap_hdq_pdata.dev_attr = oh->dev_attr;
	omap_hdq_pdata.device_enable = omap_device_enable;
	omap_hdq_pdata.device_idle = omap_device_idle;
	omap_hdq_pdata.device_shutdown = omap_device_shutdown;

	od = omap_device_build(name, 0, oh, &omap_hdq_pdata,
			       sizeof(struct omap_hdq_platform_data),
			       omap_hdq_latency,
			       ARRAY_SIZE(omap_hdq_latency), 0);
	WARN(IS_ERR(od), "Could not build omap_device for %s %s\n",
	     name, oh_name);

	return PTR_ERR(od);
}
