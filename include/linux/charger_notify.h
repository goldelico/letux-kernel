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

#ifndef CHARGER_NOTIFY_H
#define CHARGER_NOTIFY_H

#ifdef __KERNEL__

enum charger_event {
	EVENT_DOCKON,
	EVENT_DOCKOFF,
	EVENT_END
};


extern void charger_register_notify(struct notifier_block *nb);
extern void charger_unregister_notify(struct notifier_block *nb);
extern void charger_notify_subscriber(unsigned long event);
#endif /* __KERNEL__ */

#endif /* CHARGER_NOTIFY_H */
