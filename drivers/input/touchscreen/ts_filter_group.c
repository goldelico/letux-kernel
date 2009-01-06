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
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 * All rights reserved.
 *
 * This filter is useful to reject samples that are not reliable. We consider
 * that a sample is not reliable if it deviates form the Majority.
 *
 * 1) We collect S samples.
 *
 * 2) For each dimension:
 *
 *  - We sort the points.
 *  - Points that are "close enough" are considered to be in the same set.
 *  - We choose the set with more elements. If more than "threshold"
 *    points are in this set we use the first and the last point of the set
 *    to define the valid range for this dimension [min, max], otherwise we
 *    discard all the points and go to step 1.
 *
 * 3) We consider the unsorted S samples and try to feed them to the next
 *    filter in the chain. If one of the points of each sample
 *    is not in the allowed range for its dimension, we discard the sample.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/ts_filter_group.h>

static void ts_filter_group_clear_internal(struct ts_filter_group *tsfg,
					   int attempts)
{
	tsfg->N = 0;
	tsfg->tries_left = attempts;
}

static void ts_filter_group_clear(struct ts_filter *tsf)
{
	struct ts_filter_group *tsfg = (struct ts_filter_group *)tsf;

	ts_filter_group_clear_internal(tsfg, tsfg->config->attempts);

	if (tsf->next) /* chain */
		(tsf->next->api->clear)(tsf->next);
}

static struct ts_filter *ts_filter_group_create(struct platform_device *pdev,
						void *conf, int count_coords)
{
	struct ts_filter_group *tsfg;
	int i;

	BUG_ON((count_coords < 1) || (count_coords > MAX_TS_FILTER_COORDS));

	tsfg = kzalloc(sizeof(struct ts_filter_group), GFP_KERNEL);
	if (!tsfg)
		return NULL;

	tsfg->config = (struct ts_filter_group_configuration *)conf;
	tsfg->tsf.count_coords = count_coords;

	BUG_ON(tsfg->config->attempts <= 0);

	tsfg->samples[0] = kmalloc((2 + count_coords) * sizeof(int) *
				   tsfg->config->extent, GFP_KERNEL);
	if (!tsfg->samples[0]) {
		kfree(tsfg);
		return NULL;
	}
	for (i = 1; i < count_coords; ++i)
		tsfg->samples[i] = tsfg->samples[0] + i * tsfg->config->extent;
	tsfg->sorted_samples = tsfg->samples[0] + count_coords *
			       tsfg->config->extent;
	tsfg->group_size = tsfg->samples[0] + (1 + count_coords) *
			       tsfg->config->extent;

	ts_filter_group_clear_internal(tsfg, tsfg->config->attempts);

	printk(KERN_INFO"  Created group ts filter len %d depth %d close %d "
			"thresh %d\n", tsfg->config->extent, count_coords,
			tsfg->config->close_enough, tsfg->config->threshold);

	return &tsfg->tsf;
}

static void ts_filter_group_destroy(struct platform_device *pdev,
				    struct ts_filter *tsf)
{
	struct ts_filter_group *tsfg = (struct ts_filter_group *)tsf;

	kfree(tsfg->samples[0]); /* first guy has pointer from kmalloc */
	kfree(tsf);
}

static void ts_filter_group_scale(struct ts_filter *tsf, int *coords)
{
	if (tsf->next)
		(tsf->next->api->scale)(tsf->next, coords);
}

static int int_cmp(const void *_a, const void *_b)
{
	const int *a = _a;
	const int *b = _b;

	if (*a > *b)
		return 1;
	if (*a < *b)
		return -1;
	return 0;
}

static int ts_filter_group_process(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_group *tsfg = (struct ts_filter_group *)tsf;
	int n;
	int i;
	int ret = 0; /* ask for more samples by default */

	BUG_ON(tsfg->N >= tsfg->config->extent);

	for (n = 0; n < tsf->count_coords; n++)
		tsfg->samples[n][tsfg->N] = coords[n];

	if (++tsfg->N < tsfg->config->extent)
		return 0;	/* we meed more samples */

	for (n = 0; n < tsfg->tsf.count_coords; n++) {
		int *v = tsfg->sorted_samples;
		int ngroups = 0;
		int best_size;
		int best_idx = 0;
		int idx = 0;

		memcpy(v, tsfg->samples[n], tsfg->N * sizeof(int));
		sort(v, tsfg->N, sizeof(int), int_cmp, NULL);

		tsfg->group_size[0] = 1;
		for (i = 1; i < tsfg->N; ++i) {
			if (v[i] - v[i - 1] <= tsfg->config->close_enough)
				tsfg->group_size[ngroups]++;
			else
				tsfg->group_size[++ngroups] = 1;
		}
		ngroups++;

		best_size = tsfg->group_size[0];
		for (i = 1; i < ngroups; i++) {
			idx += tsfg->group_size[i - 1];
			if (best_size < tsfg->group_size[i]) {
				best_size = tsfg->group_size[i];
				best_idx = idx;
			}
		}

		if (best_size < tsfg->config->threshold) {
			/* this set is not good enough for us */
			if (--tsfg->tries_left) {
				ts_filter_group_clear_internal
					(tsfg, tsfg->tries_left);
				return 0; /* ask for more samples */
			}
			return -1; /* we give up */
		}

		tsfg->range_min[n] = v[best_idx];
		tsfg->range_max[n] = v[best_idx + best_size - 1];
	}

	for (i = 0; i < tsfg->N; ++i) {
		int r;

		for (n = 0; n < tsfg->tsf.count_coords; ++n) {
			coords[n] = tsfg->samples[n][i];
			if (coords[n] < tsfg->range_min[n] ||
			    coords[n] > tsfg->range_max[n])
				break;
		}

		if (n != tsfg->tsf.count_coords) /* sample not OK */
			continue;

		if (tsf->next) {
			r = (tsf->next->api->process)(tsf->next, coords);
			if (r)  {
				ret = r;
				break;
			}
		} else if (i == tsfg->N - 1) {
			ret = 1;
		}
	}

	ts_filter_group_clear_internal(tsfg, tsfg->config->attempts);

	return ret;
}

struct ts_filter_api ts_filter_group_api = {
	.create = ts_filter_group_create,
	.destroy = ts_filter_group_destroy,
	.clear = ts_filter_group_clear,
	.process = ts_filter_group_process,
	.scale = ts_filter_group_scale,
};

