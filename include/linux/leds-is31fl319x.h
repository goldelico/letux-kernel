/*
 * IS31FL3196 LED chip driver.
 *
 * Copyright (C) 2015 H. Nikolaus Schaller <hns@goldelico.com>
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
 */

#ifndef __LINUX_IS31FL319X_H
#define __LINUX_IS31FL319X_H
#include <linux/leds.h>

struct is31fl319x_platform_data {
	struct led_platform_data leds;
};

#endif /* __LINUX_IS31FL319X_H */
