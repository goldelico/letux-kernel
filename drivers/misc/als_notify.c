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

#include <linux/notifier.h>
#include <linux/als_notify.h>

static BLOCKING_NOTIFIER_HEAD(als_notifier_list);

/**
 * als_register_notify - register a notifier callback for triggering display init
 * @nb: pointer to the notifier block for the callback events.
 *
 */
void als_register_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&als_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(als_register_notify);

/**
 * als_unregister_notify - unregister a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 * als_register_notify() must have been previously called
 * for this function to work properly.
 */
void als_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&als_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(als_unregister_notify);

void als_notify_subscriber(unsigned long event)
{
	blocking_notifier_call_chain(&als_notifier_list, event, NULL);
}
EXPORT_SYMBOL_GPL(als_notify_subscriber);
