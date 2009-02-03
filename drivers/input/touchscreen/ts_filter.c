/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2008 Andy Green <andy@openmoko.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/ts_filter.h>

int ts_filter_create_chain(struct platform_device *pdev,
			   struct ts_filter_api **api, void **config,
			   struct ts_filter **list, int count_coords)
{
	int count = 0;
	struct ts_filter *last = NULL;

	if (!api)
		return 0;

	while (*api && count < MAX_TS_FILTER_CHAIN) {
		*list = ((*api)->create)(pdev, *config++, count_coords);
		if (!*list) {
			printk(KERN_ERR "Filter %d failed init\n", count);
			return count;
		}
		(*list)->api = *api++;
		if (last)
			last->next = *list;
		last = *list;
		list++;
		count++;
	}

	return count;
}
EXPORT_SYMBOL_GPL(ts_filter_create_chain);

void ts_filter_destroy_chain(struct platform_device *pdev,
			     struct ts_filter **list)
{
	struct ts_filter **first;
	int count = 0;

	first = list;
	while (*list && count++ < MAX_TS_FILTER_CHAIN) {
		((*list)->api->destroy)(pdev, *list);
		list++;
	}
	*first = NULL;
}
EXPORT_SYMBOL_GPL(ts_filter_destroy_chain);
