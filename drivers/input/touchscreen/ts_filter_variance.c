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
 * This filter is useful to reject clicks that are not reliable. We
 * only care about what happens when we receive DOWN events for the fist time.
 * If this filter does not reject the first samples then it will change
 * its internal state to "passed" and the remaining samples
 * will be passed to the next filter in the chain.
 *
 * First we collect N samples, then then we sort them. We discard the borders
 * (with a window) and then compute the variance of the remaining set.
 * If the computed variance is bigger than a threshold, we reject the click.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/ts_filter_variance.h>

static void ts_filter_variance_clear_internal(struct ts_filter_variance *tsfv,
					      int attempts)
{
	tsfv->N = 0;
	tsfv->passed = 0;
	tsfv->tries_left = attempts;
}

static void ts_filter_variance_clear(struct ts_filter *tsf)
{
	struct ts_filter_variance *tsfv = (struct ts_filter_variance *)tsf;

	ts_filter_variance_clear_internal(tsfv, tsfv->config->attempts);

	if (tsf->next) /* chain */
		(tsf->next->api->clear)(tsf->next);
}

static struct ts_filter *ts_filter_variance_create(void *conf, int count_coords)
{
	struct ts_filter_variance *tsfv;
	int i;

	BUG_ON((count_coords < 1) || (count_coords > MAX_TS_FILTER_COORDS));

	tsfv = kzalloc(sizeof(struct ts_filter_variance), GFP_KERNEL);
	if (!tsfv)
		return NULL;

	tsfv->config = (struct ts_filter_variance_configuration *)conf;
	tsfv->tsf.count_coords = count_coords;

	BUG_ON(tsfv->config->attempts <= 0);

	tsfv->samples[0] = kmalloc(count_coords * sizeof(int) *
				   tsfv->config->extent, GFP_KERNEL);
	if (!tsfv->samples[0]) {
		kfree(tsfv);
		return NULL;
	}
	for (i = 1; i < count_coords; ++i)
		tsfv->samples[i] = tsfv->samples[0] + i * tsfv->config->extent;

	ts_filter_variance_clear_internal(tsfv, tsfv->config->attempts);

	printk(KERN_INFO"  Created Variance ts filter len %d depth %d window"
			" %d thresh %d\n", tsfv->config->extent,
			count_coords, tsfv->config->window,
			tsfv->config->threshold);

	/* scale the threshold to avoid divisions later */
	tsfv->config->threshold *= tsfv->config->extent -
				   (tsfv->config->window << 1);

	return &tsfv->tsf;
}

static void ts_filter_variance_destroy(struct ts_filter *tsf)
{
	struct ts_filter_variance *tsfv = (struct ts_filter_variance *)tsf;

	kfree(tsfv->samples[0]); /* first guy has pointer from kmalloc */
	kfree(tsf);
}

static void ts_filter_variance_scale(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_variance *tsfv = (struct ts_filter_variance *)tsf;

	if (!tsfv->passed)
		return;

	if (tsf->next) {
		(tsf->next->api->scale)(tsf->next, coords);
	} else {
		int n;
		for (n = 0; n < tsf->count_coords; n++) {
			int c = tsfv->samples[n][tsfv->N / 2] +
				tsfv->samples[n][tsfv->N / 2 + 1] +
				tsfv->samples[n][tsfv->N / 2 - 1];
			coords[n] = (c + 2) / 3;
		}
	}
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

/* give us the raw sample data coords, and if we return 1 then you can
 * get a filtered coordinate from coords: if we return 0 you didn't
 * fill all the filters with samples yet.
 */

static int ts_filter_variance_process(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_variance *tsfv = (struct ts_filter_variance *)tsf;
	int n;

	if (tsfv->passed) { /* chain */
		if (tsf->next)
			return (tsf->next->api->process)(tsf->next, coords);
		return 1;
	}

	for (n = 0; n < tsf->count_coords; n++)
		tsfv->samples[n][tsfv->N] = coords[n];

	if (++tsfv->N < tsfv->config->extent)
		return 0;	/* we meed more samples */

	tsfv->passed = 1;

	for (n = 0; n < tsfv->tsf.count_coords; n++) {
		int i;
		int avg = 0;
		int variance = 0;

		sort(tsfv->samples[n], tsfv->config->extent, sizeof(int),
		     int_cmp, NULL);

		for (i = tsfv->config->window; i < tsfv->config->extent -
					       tsfv->config->window; ++i)
			avg += tsfv->samples[n][i];

		avg /= tsfv->config->extent - (tsfv->config->window << 1);

		for (i = tsfv->config->window; i < tsfv->config->extent -
						   tsfv->config->window; ++i) {
			int s = tsfv->samples[n][i] - avg;
			variance += s * s;
		}

		if (variance > tsfv->config->threshold) {
			tsfv->passed = 0;
			break;
		}
	}

	if (tsfv->passed) /* Let's reuse the last sample */
		return ts_filter_variance_process(tsf, coords);

	if (--tsfv->tries_left) {
		ts_filter_variance_clear_internal(tsfv, tsfv->tries_left);
		return 0; /* ask for more samples */
	}

	/* avoid overflow if we are called again without clearing the filter */
	ts_filter_variance_clear_internal(tsfv, tsfv->config->attempts);

	return -1;
}

struct ts_filter_api ts_filter_variance_api = {
	.create = ts_filter_variance_create,
	.destroy = ts_filter_variance_destroy,
	.clear = ts_filter_variance_clear,
	.process = ts_filter_variance_process,
	.scale = ts_filter_variance_scale,
};

