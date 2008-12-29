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
 *
 *
 * Median averaging stuff.  We sort incoming raw samples into an array of
 * MEDIAN_SIZE length, discarding the oldest sample each time once we are full.
 * We then return the sum of the middle three samples for X and Y.  It means
 * the final result must be divided by (3 * scaling factor) to correct for
 * avoiding the repeated /3.
 *
 * This strongly rejects brief excursions away from a central point that is
 * sticky in time compared to the excursion duration.
 *
 * Thanks to Dale Schumacher (who wrote some example code) and Carl-Daniel
 * Halifinger who pointed out this would be a good method.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ts_filter_median.h>

static void ts_filter_median_insert(int *p, int sample, int count)
{
	int n;

	/* search through what we got so far to find where to put sample */
	for (n = 0; n < count; n++)
		 /* we met somebody bigger than us? */
		if (sample < p[n]) {
			/* starting from the end, push bigger guys down one */
			for (count--; count >= n; count--)
				p[count + 1] = p[count];
			p[n] = sample; /* and put us in place of first bigger */
			return;
		}

	p[count] = sample; /* nobody was bigger than us, add us on the end */
}

static void ts_filter_median_del(int *p, int value, int count)
{
	int index;

	for (index = 0; index < count; index++)
		if (p[index] == value) {
			for (; index < count; index++)
				p[index] = p[index + 1];
			return;
		}
}


static void ts_filter_median_clear_internal(struct ts_filter *tsf)
{
	struct ts_filter_median *tsfm = (struct ts_filter_median *)tsf;

	tsfm->pos = 0;
	tsfm->valid = 0;

}
static void ts_filter_median_clear(struct ts_filter *tsf)
{
	ts_filter_median_clear_internal(tsf);

	if (tsf->next) /* chain */
		(tsf->next->api->clear)(tsf->next);
}

static struct ts_filter *ts_filter_median_create(struct platform_device *pdev,
						 void *conf, int count_coords)
{
	int *p;
	int n;
	struct ts_filter_median *tsfm = kzalloc(sizeof(struct ts_filter_median),
								    GFP_KERNEL);

	if (!tsfm)
		return NULL;

	tsfm->config = (struct ts_filter_median_configuration *)conf;
	BUG_ON((count_coords < 1) || (count_coords > MAX_TS_FILTER_COORDS));
	tsfm->tsf.count_coords = count_coords;

	tsfm->config->midpoint = (tsfm->config->extent >> 1) + 1;

	p = kmalloc(2 * count_coords * sizeof(int) * (tsfm->config->extent + 1),
								    GFP_KERNEL);
	if (!p) {
		kfree(tsfm);
		return NULL;
	}

	for (n = 0; n < count_coords; n++) {
		tsfm->sort[n] = p;
		p += tsfm->config->extent + 1;
		tsfm->fifo[n] = p;
		p += tsfm->config->extent + 1;
	}

	ts_filter_median_clear_internal(&tsfm->tsf);

	printk(KERN_INFO"  Created Median ts filter len %d depth %d dec %d\n",
	       tsfm->config->extent, count_coords,
	       tsfm->config->decimation_threshold);

	return &tsfm->tsf;
}

static void ts_filter_median_destroy(struct platform_device *pdev,
				     struct ts_filter *tsf)
{
	struct ts_filter_median *tsfm = (struct ts_filter_median *)tsf;

	kfree(tsfm->sort[0]); /* first guy has pointer from kmalloc */
	kfree(tsf);
}

static void ts_filter_median_scale(struct ts_filter *tsf, int *coords)
{
	int n;

	for (n = 0; n < tsf->count_coords; n++)
		coords[n] = (coords[n] + 2) / 3;

	if (tsf->next) /* chain */
		(tsf->next->api->scale)(tsf->next, coords);
}

/* give us the raw sample data coords, and if we return 1 then you can
 * get a filtered coordinate from coords: if we return 0 you didn't
 * fill all the filters with samples yet.
 */

static int ts_filter_median_process(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_median *tsfm = (struct ts_filter_median *)tsf;
	int n;
	int movement = 1;

	for (n = 0; n < tsf->count_coords; n++) {
		/* grab copy in insertion order to remove when oldest */
		tsfm->fifo[n][tsfm->pos] = coords[n];
		/* insert these samples in sorted order in the median arrays */
		ts_filter_median_insert(tsfm->sort[n], coords[n], tsfm->valid);
	}
	/* move us on in the fifo */
	if (++tsfm->pos == (tsfm->config->extent + 1))
		tsfm->pos = 0;

	/* we have finished a median sampling? */
	if (++tsfm->valid != tsfm->config->extent)
		return 0; /* no valid sample to use */

	/* discard the oldest sample in median sorted array */
	tsfm->valid--;

	/* sum the middle 3 in the median sorted arrays.  We don't divide back
	 * down which increases the sum resolution by a factor of 3 until the
	 * scale API is called
	 */
	for (n = 0; n < tsfm->tsf.count_coords; n++)
		/* perform the deletion of the oldest sample */
		ts_filter_median_del(tsfm->sort[n], tsfm->fifo[n][tsfm->pos],
								   tsfm->valid);

	tsfm->decimation_count--;
	if (tsfm->decimation_count >= 0)
		return 0;

	for (n = 0; n < tsfm->tsf.count_coords; n++) {
		/* give the coordinate result from summing median 3 */
		coords[n] = tsfm->sort[n][tsfm->config->midpoint - 1] +
			    tsfm->sort[n][tsfm->config->midpoint] +
			    tsfm->sort[n][tsfm->config->midpoint + 1]
			;

		movement += abs(tsfm->last_issued[n] - coords[n]);
	}

	if (movement > tsfm->config->decimation_threshold) /* fast */
		tsfm->decimation_count = tsfm->config->decimation_above;
	else
		tsfm->decimation_count = tsfm->config->decimation_below;

	memcpy(&tsfm->last_issued[0], coords,
	       tsfm->tsf.count_coords * sizeof(int));

	if (tsf->next) /* chain */
		return (tsf->next->api->process)(tsf->next, coords);

	return 1;
}

struct ts_filter_api ts_filter_median_api = {
	.create = ts_filter_median_create,
	.destroy = ts_filter_median_destroy,
	.clear = ts_filter_median_clear,
	.process = ts_filter_median_process,
	.scale = ts_filter_median_scale,
};
