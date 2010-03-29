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
#include <linux/clk.h>

#include <plat/i2c.h>
#include <plat/mux.h>
#include <plat/cpu.h>
#include <plat/irqs.h>

#include "mux.h"

#define OMAP1_I2C_SIZE         0x3f
#define OMAP1_I2C_BASE         0xfffb3800

static const char name[] = "i2c_omap";

#define OMAP1_I2C_RESOURCE_BUILDER(base, irq)		\
{
	{						\
		.start	= (base),			\
		.end	= (base) + OMAP1_I2C_SIZE,	\
		.flags	= IORESOURCE_MEM,		\
	},						\
	{						\
		.start	= (irq),			\
		.flags	= IORESOURCE_IRQ,		\
	},						\
}

static struct resource i2c_resources[][2] = {
	{ OMAP1_I2C_RESOURCE_BUILDER(OMAP1_I2C_BASE, INT_I2C) },
};

#define OMAP1_I2C_DEV_BUILDER(bus_id, res)		\
	{						\
		.id	= (bus_id),			\
		.name	= name,				\
		.num_resources	= ARRAY_SIZE(res),	\
		.resource	= (res),		\
	}

static struct platform_device omap_i2c_devices[] = {
	OMAP1_I2C_DEV_BUILDER(1, i2c_resources[0]),
};

#define I2C_ICLK	0
#define I2C_FCLK	1
static struct clk *omap_i2c_clks[ARRAY_SIZE(omap_i2c_devices)][2];

static struct omap_i2c_dev_attr omap1_i2c_dev_attr;

int __init omap1_i2c_nr_ports(void)
{
	return 1;
}

static int omap1_i2c_device_enable(struct platform_device *pdev)
{
	struct clk *c;
	c = omap_i2c_clks[pdev->id - 1][I2C_ICLK];
	if (c && !IS_ERR(c))
		clk_enable(c);

	c = omap_i2c_clks[pdev->id - 1][I2C_FCLK];
	if (c && !IS_ERR(c))
		clk_enable(c);

	return 0;
}

static int omap1_i2c_device_idle(struct platform_device *pdev)
{
	struct clk *c;

	c = omap_i2c_clks[pdev->id - 1][I2C_FCLK];
	if (c && !IS_ERR(c))
		clk_disable(c);

	c = omap_i2c_clks[pdev->id - 1][I2C_ICLK];
	if (c && !IS_ERR(c))
		clk_disable(c);

	return 0;
}

int __init omap1_i2c_add_bus(int bus_id)
{
	struct platform_device *pdev;
	struct omap_i2c_platform_data *pdata;

	pdev = &omap_i2c_devices[bus_id - 1];
	pdata = omap_i2c_get_pdata(bus_id - 1);

	/* idle and shutdown share the same code */
	pdata->device_enable = omap1_i2c_device_enable;
	pdata->device_idle = omap1_i2c_device_idle;
	pdata->device_shutdown = omap1_i2c_device_idle;
	pdata->dev_attr = &omap1_i2c_dev_attr;

	omap_i2c_clks[bus_id - 1][I2C_ICLK] = clk_get(&pdev->dev, "ick");
	omap_i2c_clks[bus_id - 1][I2C_FCLK] = clk_get(&pdev->dev, "fck");

	return platform_device_register(pdev);
}

int __init omap_register_i2c_bus(int bus_id, u32 clkrate,
			  struct i2c_board_info const *info,
			  unsigned len)
{
	if (cpu_is_omap7xx()) {
		omap_cfg_reg(I2C_7XX_SDA);
		omap_cfg_reg(I2C_7XX_SCL);
	} else {
		omap_cfg_reg(I2C_SDA);
		omap_cfg_reg(I2C_SCL);
	}

	return omap_plat_register_i2c_bus(bus_id, clkrate, info, len);
}
