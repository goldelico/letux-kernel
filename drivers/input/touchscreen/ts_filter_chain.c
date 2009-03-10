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
 * Copyright (c) 2008,2009 Andy Green <andy@openmoko.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include "ts_filter.h"

/*
 * Tux, would you like the following function in /lib?
 * It helps us avoid silly code.
 */

/**
 * sptrlen - Count how many non-null pointers are in a pointer array
 * @arr: The array of pointers
 */
static int sptrlen(const void *arr)
{
	/* All pointers have the same size. */
	const int **p = (const int **)arr;
	int len = 0;

	while (*(p++))
		len++;

	return len;
}

/* FIXME: rename & remove this temporal hack. */
static struct ts_filter **revchain;

struct ts_filter **ts_filter_chain_create(
	struct platform_device *pdev,
	const struct ts_filter_configuration conf[],
	int count_coords)
{
	struct ts_filter **arr;
	int count = 0;
	int len;
	int nrev = 0;

	BUG_ON((count_coords < 1));
	BUG_ON(count_coords > MAX_TS_FILTER_COORDS);

	len = (sptrlen(conf) + 1);
	/* memory for two null-terminated arrays of filters */
	arr = kzalloc(2 * sizeof(struct ts_filter *) * len, GFP_KERNEL);
	if (!arr)
		goto create_err;
	revchain = arr + len;

	while (conf->api) {
		/* TODO: Can we get away with only sending pdev->dev? */
		struct ts_filter *f =
			(conf->api->create)(pdev, conf->config, count_coords);
		if (!f) {
			dev_info(&pdev->dev, "Filter %d creation failed\n",
				 count);
			goto create_err;
		}

		f->api = conf->api;
		arr[count++] = f;

		/* Filters that can propagate values in the chain. */
		if (f->api->haspoint && f->api->getpoint && f->api->process)
			revchain[nrev++] = f;

		conf++;
	}

	dev_info(&pdev->dev, "%d filter(s) initialized\n", count);

	return arr;

create_err:

	dev_info(&pdev->dev, "Error in filter chain initialization\n");

	ts_filter_chain_destroy(arr);

	return NULL;
}
EXPORT_SYMBOL_GPL(ts_filter_chain_create);

void ts_filter_chain_destroy(struct ts_filter **arr)
{
	struct ts_filter **a = arr;
	int count = 0;

	while (arr && *a) {
		((*a)->api->destroy)(*a);
		a++;
		count++;
	}

	kfree(arr);
}
EXPORT_SYMBOL_GPL(ts_filter_chain_destroy);

void ts_filter_chain_clear(struct ts_filter **arr)
{
	while (*arr) {
		if ((*arr)->api->clear)
			((*arr)->api->clear)(*arr);
		arr++;
	}
}
EXPORT_SYMBOL_GPL(ts_filter_chain_clear);

static void ts_filter_chain_scale(struct ts_filter **a, int *coords)
{
	while (*a) {
		if ((*a)->api->scale)
			((*a)->api->scale)(*a, coords);
		a++;
	}
}

int ts_filter_chain_feed(struct ts_filter **arr, int *coords)
{
	/* FIXME: only using revchain */
	int len = sptrlen(revchain); /* FIXME: save this */
	int i = len - 1;

	if (!arr[0])
		return 1; /* Nothing to do. Filtering disabled. */

	BUG_ON(arr[0]->api->haspoint(arr[0]));

	if (arr[0]->api->process(arr[0], coords))
		return -1;

	while (i >= 0 && i < len) {
		if (revchain[i]->api->haspoint(revchain[i])) {
			revchain[i]->api->getpoint(revchain[i], coords);
			if (++i < len &&
			    revchain[i]->api->process(revchain[i], coords))
				return -1; /* Error. */
		} else {
			i--;
		}
	}

	if (i >= 0) {
		BUG_ON(i != len); /* FIXME: Remove BUG_ON. */
		ts_filter_chain_scale(arr, coords); /* TODO: arr! */
	}

	return i >= 0; /* Same as i == len. */
}
EXPORT_SYMBOL_GPL(ts_filter_chain_feed);

