/*
 * Copyright (C) 2014 Motorola Mobility LLC.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DISPLAY_NOTIFY_H
#define DISPLAY_NOTIFY_H

#ifdef __KERNEL__

enum display_event {
	DISPLAY_EVENT_DISPLAYON,
	DISPLAY_EVENT_DISPLAYOFF,
	DISPLAY_EVENT_END
};


extern void display_register_notify(struct notifier_block *nb);
extern void display_unregister_notify(struct notifier_block *nb);
extern void display_notify_subscriber(unsigned long event);
#endif /* __KERNEL__ */

#endif /* DISPLAY_NOTIFY_H */
