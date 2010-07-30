/*
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2009 Nokia Corporation.
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
#include <linux/i2c.h>
#include <linux/err.h>

#include <plat/cpu.h>
#include <plat/i2c.h>
#include <plat/irqs.h>
#include <plat/mux.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>

#include "mux.h"

static const char name[] = "i2c_omap";

#define MAX_OMAP_I2C_HWMOD_NAME_LEN		16

int __init omap2_i2c_nr_ports(void)
{
	int ports = 0;

	if (cpu_is_omap24xx())
		ports = 2;
	else if (cpu_is_omap34xx())
		ports = 3;
	else if (cpu_is_omap44xx())
		ports = 4;

	return ports;
}

static struct omap_device_pm_latency omap_i2c_latency[] = {
	[0] = {
		.deactivate_func	= omap_device_idle_hwmods,
		.activate_func		= omap_device_enable_hwmods,
		.flags			= OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

int __init omap2_i2c_add_bus(int bus_id)
{
	struct omap_hwmod *oh;
	struct omap_device *od;
	char oh_name[MAX_OMAP_I2C_HWMOD_NAME_LEN];
	int l, idx;
	struct omap_i2c_platform_data *pdata;

	idx = bus_id - 1;

	l = snprintf(oh_name, MAX_OMAP_I2C_HWMOD_NAME_LEN,
		     "i2c%d", bus_id);
	WARN(l >= MAX_OMAP_I2C_HWMOD_NAME_LEN,
	     "String buffer overflow in I2C%d device setup\n", bus_id);
	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -EEXIST;
	}

	pdata = omap_i2c_get_pdata(idx);
	pdata->dev_attr = oh->dev_attr;
	pdata->device_enable = omap_device_enable;
	pdata->device_idle = omap_device_idle;
	pdata->device_shutdown = omap_device_shutdown;

	od = omap_device_build(name, bus_id, oh, pdata,
			       sizeof(struct omap_i2c_platform_data),
			       omap_i2c_latency, ARRAY_SIZE(omap_i2c_latency), 0);
	WARN(IS_ERR(od), "Could not build omap_device for %s %s\n",
	     name, oh_name);

	return PTR_ERR(od);
}

int __init omap_register_i2c_bus(int bus_id, u32 clkrate,
			  struct omap_i2c_bus_board_data *pdata,
			  struct i2c_board_info const *info,
			  unsigned len)
{
	if (cpu_is_omap24xx()) {
		const int omap24xx_pins[][2] = {
			{ M19_24XX_I2C1_SCL, L15_24XX_I2C1_SDA },
			{ J15_24XX_I2C2_SCL, H19_24XX_I2C2_SDA },
		};
		int scl, sda;

		scl = omap24xx_pins[bus_id - 1][0];
		sda = omap24xx_pins[bus_id - 1][1];
		omap_cfg_reg(sda);
		omap_cfg_reg(scl);
	}

	/* First I2C bus is not muxable */
	if (cpu_is_omap34xx() && bus_id > 1) {
		char mux_name[sizeof("i2c2_scl.i2c2_scl")];

		sprintf(mux_name, "i2c%i_scl.i2c%i_scl", bus_id, bus_id);
		omap_mux_init_signal(mux_name, OMAP_PIN_INPUT);
		sprintf(mux_name, "i2c%i_sda.i2c%i_sda", bus_id, bus_id);
		omap_mux_init_signal(mux_name, OMAP_PIN_INPUT);
	}

	return omap_plat_register_i2c_bus(bus_id, clkrate, pdata, info, len);
}
