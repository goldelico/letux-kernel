/*
 * omap_ocp2scp.h -- ocp2scp header file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
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

#ifndef __DRIVERS_OMAP_OCP2SCP_H
#define __DRIVERS_OMAP_OCP2SCP_H

#include <linux/ioport.h>

enum omap_ocp2scp_bus_devices {
	DEV_TYPE_UNDEFINED,
	DEV_TYPE_USB2PHY,
	DEV_TYPE_USB3PHY,
	DEV_TYPE_SATAPHY,
};

struct omap_ocp2scp_dev {
	u8				rev_id;
	const char			*dev_name;
	enum omap_ocp2scp_bus_devices	dev_type;
	struct resource			*res;
};
#endif /* __DRIVERS_OMAP_OCP2SCP_H */
