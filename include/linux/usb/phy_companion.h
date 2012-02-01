/*
 * phy-companion.h -- phy companion to indicate the comparator part of PHY
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

#ifndef __DRIVERS_PHY_COMPANION_H
#define __DRIVERS_PHY_COMPANION_H

#include <linux/usb/otg.h>

/* phy_companion to take care of VBUS, ID and srp capabilities */
struct phy_companion {
	u8				linkstat;

	/* for notification of comparator interrupt events */
	struct atomic_notifier_head	notifier;

	/* effective for A-peripheral, ignored for B devices */
	int	(*set_vbus)(struct phy_companion *x, bool enabled);

	/* for B devices only:  start session with A-Host */
	int	(*start_srp)(struct phy_companion *x);
};

static inline struct phy_companion *get_phy_twl6030_companion
					(struct notifier_block *nb)
{
	return NULL;
}
#endif /* __DRIVERS_PHY_COMPANION_H */
