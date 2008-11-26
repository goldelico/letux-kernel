/* Philips PCF50633 GPIO Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50633 driver mainly by
 * Harald Welte, Andy Green and Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/pmic.h>

void pcf50633_gpio_set(struct pcf50633 *pcf, int gpio, int val)
{
	u8 reg;

	reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;

	pcf50633_reg_set_bit_mask(pcf, reg, 0x07, val);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set);

int pcf50633_gpio_get(struct pcf50633 *pcf, int gpio)
{
	u8 reg, val;

	reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;
	val = pcf50633_reg_read(pcf, reg);

	return val;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_get);

void pcf50633_gpio_invert_set(struct pcf50633 *pcf, int gpio, int invert)
{
	u8 val, reg;

	reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;
	val = !!invert << 3;

	pcf50633_reg_set_bit_mask(pcf, reg, val, val);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_invert_set);

int pcf50633_gpio_invert_get(struct pcf50633 *pcf, int gpio)
{
	u8 reg, val;

	reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;
	val = pcf50633_reg_read(pcf, reg);

	return val & (1 << 3);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_invert_get);

static const u8 pcf50633_regulator_registers[PCF50633_NUM_REGULATORS] = {
	[PCF50633_REGULATOR_AUTO]	= PCF50633_REG_AUTOOUT,
	[PCF50633_REGULATOR_DOWN1]	= PCF50633_REG_DOWN1OUT,
	[PCF50633_REGULATOR_DOWN2]	= PCF50633_REG_DOWN2OUT,
	[PCF50633_REGULATOR_MEMLDO]	= PCF50633_REG_MEMLDOOUT,
	[PCF50633_REGULATOR_LDO1]	= PCF50633_REG_LDO1OUT,
	[PCF50633_REGULATOR_LDO2]	= PCF50633_REG_LDO2OUT,
	[PCF50633_REGULATOR_LDO3]	= PCF50633_REG_LDO3OUT,
	[PCF50633_REGULATOR_LDO4]	= PCF50633_REG_LDO4OUT,
	[PCF50633_REGULATOR_LDO5]	= PCF50633_REG_LDO5OUT,
	[PCF50633_REGULATOR_LDO6]	= PCF50633_REG_LDO6OUT,
	[PCF50633_REGULATOR_HCLDO]	= PCF50633_REG_HCLDOOUT,
};

void pcf50633_gpio_power_supply_set(struct pcf50633 *pcf,
					int gpio, int regulator, int on)
{
	u8 reg, val;

	/* the *ENA register is always one after the *OUT register */
	reg = pcf50633_regulator_registers[regulator] + 1;

	val = (!!on << (gpio - PCF50633_GPIO1));

	pcf50633_reg_set_bit_mask(pcf, reg, val, val);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_power_supply_set);
