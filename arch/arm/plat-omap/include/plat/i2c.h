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

#include <linux/i2c.h>
#include <linux/platform_device.h>

struct omap_i2c_bus_board_data {
	struct hwspinlock *handle;
	int (*hwspinlock_lock) (struct hwspinlock *handle);
	int (*hwspinlock_unlock) (struct hwspinlock *handle);
};

#if defined(CONFIG_I2C_OMAP) || defined(CONFIG_I2C_OMAP_MODULE)
extern int omap_register_i2c_bus(int bus_id, u32 clkrate,
				 struct omap_i2c_bus_board_data *pdata,
				 struct i2c_board_info const *info,
				 unsigned len);
#else
static inline int omap_register_i2c_bus(int bus_id, u32 clkrate,
				 struct omap_i2c_bus_board_data *pdata,
				 struct i2c_board_info const *info,
				 unsigned len)
{
	return 0;
}
#endif

int omap_plat_register_i2c_bus(int bus_id, u32 clkrate,
				 struct omap_i2c_bus_board_data *pdata,
				 struct i2c_board_info const *info,
				 unsigned len);

/**
 * i2c_dev_attr - OMAP I2C controller device attributes for omap_hwmod
 * @fifo_depth: total controller FIFO size (in bytes)
 * @flags: differences in hardware support capability
 *
 * @fifo_depth represents what exists on the hardware, not what is
 * actually configured at runtime by the device driver.
 */
struct omap_i2c_dev_attr {
	u8      fifo_depth;
	u8      flags;
};

/**
 * struct omap_i2c_platform_data - OMAP I2C controller platform data
 */
struct omap_i2c_platform_data {
	u32 rate;
	struct omap_i2c_dev_attr *dev_attr;
	struct hwspinlock *handle;
	void (*set_mpu_wkup_lat)(struct device *dev, int set);
	int (*device_enable) (struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle) (struct platform_device *pdev);
	int (*hwspinlock_lock) (struct hwspinlock *handle);
	int (*hwspinlock_unlock) (struct hwspinlock *handle);
};

/* Prototypes for OMAP platform I2C core initialization code */

struct omap_i2c_platform_data * __init omap_i2c_get_pdata(int bus_id);

int __init omap1_i2c_nr_ports(void);
int __init omap2_i2c_nr_ports(void);

int __init omap1_i2c_add_bus(int bus_id);
int __init omap2_i2c_add_bus(int bus_id);

