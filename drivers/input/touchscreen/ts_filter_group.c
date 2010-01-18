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
 * Copyright (C) 2008,2009 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 * All rights reserved.
 *
 *
 * This filter is useful to reject samples that are not reliable. We consider
 * that a sample is not reliable if it deviates form the Majority.
 * This filter mixes information from all the available dimensions. It means
 * that for two dimensions we draw a rectangle where the thought-to-be good
 * points can be found.
 *
 * The implementation would be more efficient with a double-linked list but
 * let's keep it simple for now.
 *
 * 1) We collect S samples and keep it in sorted sets.
 *  - Points that are "close enough" are considered to be in the same set.
 *    We don't actually keep the sets but ranges of points.
 *
 * 2) For each dimension:
 *  - We choose the range with more elements. If more than "threshold"
 *    points are in this range we use the minimum and the maximum point
 *    of the range to define the valid range for this dimension [min, max],
 *    otherwise we discard all the points and the ranges and go to step 1.
 *
 * 3) We consider the unsorted S samples and try to feed them to the next
 *    filter in the chain. If one of the points of each sample
 *    is not in the allowed range for its dimension we discard the sample.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include "ts_filter_group.h"

struct coord_range {
	int min;	/* Minimum value of the range. */
	int max;	/* Maximum value of the range  */
	int N;		/* Number of points in the range. */
};

struct ts_filter_group {
	/* Private filter configuration. */
	struct ts_filter_group_configuration *config;
	/* Filter API. */
	struct ts_filter tsf;

	int N;			/* How many samples we have. */
	int *samples[MAX_TS_FILTER_COORDS];	/* The samples: our input. */

	/* Temporal values that help us compute range_min and range_max. */
	struct coord_range *ranges[MAX_TS_FILTER_COORDS];	/* Ranges. */
	int n_ranges[MAX_TS_FILTER_COORDS];		/* Number of ranges */

	/* Computed ranges that help us filter the points. */
	int range_max[MAX_TS_FILTER_COORDS];	/* Max. computed ranges. */
	int range_min[MAX_TS_FILTER_COORDS];	/* Min. computed ranges. */

	int tries_left;		/* We finish if we can't get enough samples. */
	int ready;		/* If we are ready to deliver samples. */
	int result;		/* Index of the point being returned. */
};

#define ts_filter_to_filter_group(f) \
	container_of(f, struct ts_filter_group, tsf)


static void ts_filter_group_clear_internal(struct ts_filter_group *tsfg,
					   int attempts)
{
	int n;
	tsfg->N = 0;
	tsfg->tries_left = attempts;
	tsfg->ready = 0;
	tsfg->result = 0;
	for (n = 0; n < tsfg->tsf.count_coords; n++)
		tsfg->n_ranges[n] = 0;
}

static void ts_filter_group_clear(struct ts_filter *tsf)
{
	struct ts_filter_group *tsfg = ts_filter_to_filter_group(tsf);

	ts_filter_group_clear_internal(tsfg, tsfg->config->attempts);
}

static struct ts_filter *ts_filter_group_create(
	struct platform_device *pdev,
	const struct ts_filter_configuration *conf,
	int count_coords)
{
	struct ts_filter_group *tsfg;
	int i;

	tsfg = kzalloc(sizeof(struct ts_filter_group), GFP_KERNEL);
	if (!tsfg)
		return NULL;

	tsfg->config = container_of(conf,
				    struct ts_filter_group_configuration,
				    config);
	tsfg->tsf.count_coords = count_coords;

	BUG_ON(tsfg->config->attempts <= 0);
	BUG_ON(tsfg->config->length < tsfg->config->threshold);

	tsfg->samples[0] = kmalloc(count_coords * sizeof(int) *
				   tsfg->config->length, GFP_KERNEL);
	if (!tsfg->samples[0]) {
		kfree(tsfg);
		return NULL;
	}
	for (i = 1; i < count_coords; ++i)
		tsfg->samples[i] = tsfg->samples[0] + i * tsfg->config->length;

	tsfg->ranges[0] = kmalloc(count_coords * sizeof(struct coord_range) *
				  tsfg->config->length, GFP_KERNEL);
	if (!tsfg->ranges[0]) {
		kfree(tsfg->samples[0]);
		kfree(tsfg);
		return NULL;
	}
	for (i = 1; i < count_coords; ++i)
		tsfg->ranges[i] = tsfg->ranges[0] + i * tsfg->config->length;

	ts_filter_group_clear_internal(tsfg, tsfg->config->attempts);

	dev_info(&pdev->dev, "Created Group filter len:%d coords:%d close:%d "
		 "thresh:%d\n", tsfg->config->length, count_coords,
		 tsfg->config->close_enough, tsfg->config->threshold);

	return &tsfg->tsf;
}

static void ts_filter_group_destroy(struct ts_filter *tsf)
{
	struct ts_filter_group *tsfg = ts_filter_to_filter_group(tsf);

	kfree(tsfg->samples[0]);
	kfree(tsfg->ranges[0]);
	kfree(tsf);
}

static void ts_filter_group_prepare_next(struct ts_filter *tsf);

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define IN_RANGE(c, r) ((c) >= (r).min - tsfg->config->close_enough && \
			(c) <= (r).max + tsfg->config->close_enough)

static void delete_spot(struct coord_range *v, int n, int size)
{
	int i;
	for (i = n; i < size - 1; ++i)
		v[i] = v[i + 1];
}

static int ts_filter_group_process(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_group *tsfg = ts_filter_to_filter_group(tsf);
	int n;
	int j;

	BUG_ON(tsfg->N >= tsfg->config->length);
	BUG_ON(tsfg->ready);

	for (n = 0; n < tsfg->tsf.count_coords; n++) {
		int i;
		struct coord_range *range = tsfg->ranges[n];
		int *n_ranges = &tsfg->n_ranges[n];
		int found = 0;

		tsfg->samples[n][tsfg->N] = coords[n];

		for (i = 0; i < *n_ranges; ++i) {
			if (IN_RANGE(coords[n], range[i])) {
				range[i].min  = MIN(range[i].min, coords[n]);
				range[i].max  = MAX(range[i].max, coords[n]);
				range[i].N++;
				found = 1;
				break;
			} else if (coords[n] <= range[i].min)
				break;	/* We need to insert a range. */
		}
		if (found) { /* We might need to melt ranges. */
			if (i && range[i - 1].max + tsfg->config->close_enough
			    >= range[i].min) {
				BUG_ON(range[i - 1].max >= range[i].max);
				range[i - 1].max = range[i].max;
				range[i - 1].N += range[i].N;
				delete_spot(range, i, *n_ranges);
				(*n_ranges)--;
				i--;
			}
			if (i < *n_ranges - 1 && range[i + 1].min -
			    tsfg->config->close_enough <= range[i].max) {
				range[i].max = range[i + 1].max;
				range[i].N += range[i + 1].N;
				delete_spot(range, i + 1, *n_ranges);
				(*n_ranges)--;
			}
		} else {
			BUG_ON((*n_ranges) >= tsfg->config->length);
			(*n_ranges)++;
			for (j = *n_ranges - 1; j > i; --j)
				range[j] = range[j - 1];
			range[i].N = 1;
			range[i].min = coords[n];
			range[i].max = coords[n];
		}
	}

	if (++tsfg->N < tsfg->config->length)
		return 0;

	for (n = 0; n < tsfg->tsf.count_coords; ++n) {
		int best = 0;
		for (j = 1; j < tsfg->n_ranges[n]; ++j)
			if (tsfg->ranges[n][best].N  < tsfg->ranges[n][j].N)
				best = j;
		if (tsfg->ranges[n][best].N < tsfg->config->threshold) {
			/* This set of points is not good enough for us. */
			if (--tsfg->tries_left) {
				ts_filter_group_clear_internal
					(tsfg, tsfg->tries_left);
				/* No errors but we need more samples. */
				return 0;
			}
			return 1; /* We give up: error. */
		}
		tsfg->range_min[n] = tsfg->ranges[n][best].min;
		tsfg->range_max[n] = tsfg->ranges[n][best].max;
	}

	ts_filter_group_prepare_next(tsf);

	return 0;
}

/*
 * This private function prepares a point that will be returned
 * in ts_filter_group_getpoint if it is available. It updates
 * the priv->ready state also.
 */
static void ts_filter_group_prepare_next(struct ts_filter *tsf)
{
	struct ts_filter_group *priv = ts_filter_to_filter_group(tsf);
	int n;

	while (priv->result < priv->N) {
		for (n = 0; n < priv->tsf.count_coords; ++n) {
			if (priv->samples[n][priv->result] <
			    priv->range_min[n] ||
			    priv->samples[n][priv->result] > priv->range_max[n])
				break;
		}

		if (n == priv->tsf.count_coords) /* Sample is OK. */
			break;

		priv->result++;
	}

	if (unlikely(priv->result >= priv->N)) { /* No sample to deliver. */
		ts_filter_group_clear_internal(priv, priv->config->attempts);
		priv->ready = 0;
	} else {
		priv->ready = 1;
	}
}

static int ts_filter_group_haspoint(struct ts_filter *tsf)
{
	struct ts_filter_group *priv = ts_filter_to_filter_group(tsf);

	return priv->ready;
}

static void ts_filter_group_getpoint(struct ts_filter *tsf, int *point)
{
	struct ts_filter_group *priv = ts_filter_to_filter_group(tsf);
	int n;

	BUG_ON(!priv->ready);

	for (n = 0; n < priv->tsf.count_coords; n++)
		point[n] = priv->samples[n][priv->result];

	priv->result++;

	/* This call will update priv->ready. */
	ts_filter_group_prepare_next(tsf);
}

/*
 * Get ready to process the next batch of points, forget
 * points we could have delivered.
 */
static void ts_filter_group_scale(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_group *priv = ts_filter_to_filter_group(tsf);

	ts_filter_group_clear_internal(priv, priv->config->attempts);
}

const struct ts_filter_api ts_filter_group_api = {
	.create =	ts_filter_group_create,
	.destroy =	ts_filter_group_destroy,
	.clear =	ts_filter_group_clear,
	.process =	ts_filter_group_process,
	.haspoint =	ts_filter_group_haspoint,
	.getpoint =	ts_filter_group_getpoint,
	.scale =	ts_filter_group_scale,
};
EXPORT_SYMBOL_GPL(ts_filter_group_api);

