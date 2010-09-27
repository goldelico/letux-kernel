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
#include <linux/i2c.h>
#include <mach/irqs.h>
#include <plat/mux.h>
#include <plat/i2c.h>

/*
 * Indicates to the OMAP platform I2C init code that the rate was set
 * from the kernel command line
 */

#define OMAP_I2C_CMDLINE_SETUP	(BIT(31))
#define OMAP_I2C_MAX_CONTROLLERS	4

static struct omap_i2c_platform_data omap_i2c_pdata[OMAP_I2C_MAX_CONTROLLERS];

struct omap_i2c_platform_data * __init omap_i2c_get_pdata(int bus_id)
{
	return &omap_i2c_pdata[bus_id];
}

/**
 * omap_i2c_bus_setup - Process command line options for the I2C bus speed
 * @str: String of options
 *
 * This function allow to override the default I2C bus speed for given I2C
 * bus with a command line option.
 *
 * Format: i2c_bus=bus_id,clkrate (in kHz)
 *
 * Returns 1 on success, 0 otherwise.
 */
static int __init omap_i2c_bus_setup(char *str)
{
	int ports;
	int ints[3];
	int rate;

	if (cpu_class_is_omap1())
		ports = omap1_i2c_nr_ports();
	else if (cpu_class_is_omap2())
		ports = omap2_i2c_nr_ports();
	else
		ports = 0;

	get_options(str, 3, ints);
	if (ints[0] < 2 || ints[1] < 1 || ints[1] > ports)
		return 0;

	rate = ints[2];
	rate |= OMAP_I2C_CMDLINE_SETUP;

	omap_i2c_pdata[ints[1] - 1].rate = rate;

	return 1;
}
__setup("i2c_bus=", omap_i2c_bus_setup);

/*
 * Register busses defined in command line but that are not registered with
 * omap_register_i2c_bus from board initialization code.
 */
static int __init omap_register_i2c_bus_cmdline(void)
{
	int i, err = 0;

	for (i = 0; i < ARRAY_SIZE(omap_i2c_pdata); i++)
		if (omap_i2c_pdata[i].rate & OMAP_I2C_CMDLINE_SETUP) {
			omap_i2c_pdata[i].rate &= ~OMAP_I2C_CMDLINE_SETUP;
			err = -EINVAL;
			if (cpu_class_is_omap1())
				err = omap1_i2c_add_bus(i + 1);
			else if (cpu_class_is_omap2())
				err = omap2_i2c_add_bus(i + 1);
			if (err)
				goto out;
		}

out:
	return err;
}
subsys_initcall(omap_register_i2c_bus_cmdline);

/**
 * omap_plat_register_i2c_bus - register I2C bus with device descriptors
 * @bus_id: bus id counting from number 1
 * @clkrate: clock rate of the bus in kHz
 * @info: pointer into I2C device descriptor table or NULL
 * @len: number of descriptors in the table
 *
 * Returns 0 on success or an error code.
 */
int __init omap_plat_register_i2c_bus(int bus_id, u32 clkrate,
			  struct omap_i2c_bus_board_data *pdata,
			  struct i2c_board_info const *info,
			  unsigned len)
{
	int err;
	int nr_ports = 0;

	if (cpu_class_is_omap1())
		nr_ports = omap1_i2c_nr_ports();
	else if (cpu_class_is_omap2())
		nr_ports = omap2_i2c_nr_ports();

	BUG_ON(bus_id < 1 || bus_id > nr_ports);

	if (info) {
		err = i2c_register_board_info(bus_id, info, len);
		if (err)
			return err;
	}

	if (!omap_i2c_pdata[bus_id - 1].rate)
		omap_i2c_pdata[bus_id - 1].rate = clkrate;
	omap_i2c_pdata[bus_id - 1].rate &= ~OMAP_I2C_CMDLINE_SETUP;

	if (pdata != NULL && pdata->handle != NULL) {
		omap_i2c_pdata[bus_id - 1].handle = pdata->handle;
		omap_i2c_pdata[bus_id - 1].hwspinlock_lock
						= pdata->hwspinlock_lock;
		omap_i2c_pdata[bus_id - 1].hwspinlock_unlock
						= pdata->hwspinlock_unlock;
	}


	if (cpu_class_is_omap1())
		return omap1_i2c_add_bus(bus_id);
	else if (cpu_class_is_omap2())
		return omap2_i2c_add_bus(bus_id);

	return 0;
}
