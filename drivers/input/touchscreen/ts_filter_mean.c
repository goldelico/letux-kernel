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
 * Mean has no effect if the samples are changing by more that the
 * threshold set by averaging_threshold in the configuration.
 *
 * However while samples come in that don't go outside this threshold from
 * the last reported sample, Mean replaces the samples with a simple mean
 * of a configurable number of samples (set by bits_filter_length in config,
 * which is 2^n, so 5 there makes 32 sample averaging).
 *
 * Mean works well if the input data is already good quality, reducing + / - 1
 * sample jitter when the stylus is still, or moving very slowly, without
 * introducing abrupt transitions or reducing ability to follow larger
 * movements.  If you set the threshold higher than the dynamic range of the
 * coordinates, you can just use it as a simple mean average.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ts_filter_mean.h>

static void ts_filter_mean_clear_internal(struct ts_filter *tsf)
{
	struct ts_filter_mean *tsfs = (struct ts_filter_mean *)tsf;
	int n;

	for (n = 0; n < tsfs->tsf.count_coords; n++) {
		tsfs->fhead[n] = 0;
		tsfs->ftail[n] = 0;
		tsfs->lowpass[n] = 0;
	}
}

static void ts_filter_mean_clear(struct ts_filter *tsf)
{
	ts_filter_mean_clear_internal(tsf);

	if (tsf->next) /* chain */
		(tsf->next->api->clear)(tsf->next);
}

static struct ts_filter *ts_filter_mean_create(struct platform_device *pdev,
					       void *config, int count_coords)
{
	int *p;
	int n;
	struct ts_filter_mean *tsfs = kzalloc(
				  sizeof(struct ts_filter_mean), GFP_KERNEL);
	
	if (!tsfs)
		return NULL;

	BUG_ON((count_coords < 1) || (count_coords > MAX_TS_FILTER_COORDS));
	tsfs->tsf.count_coords = count_coords;

	tsfs->config = (struct ts_filter_mean_configuration *)config;

	tsfs->config->extent = 1 << tsfs->config->bits_filter_length;
	BUG_ON((tsfs->config->extent > 256) || (!tsfs->config->extent));

	p = kmalloc(tsfs->config->extent * sizeof(int) * count_coords,
								    GFP_KERNEL);
	if (!p)
		return NULL;

	for (n = 0; n < count_coords; n++) {
		tsfs->fifo[n] = p;
		p += tsfs->config->extent;
	}

	if (!tsfs->config->averaging_threshold)
		tsfs->config->averaging_threshold = 0xffff; /* always active */

	ts_filter_mean_clear_internal(&tsfs->tsf);

	printk(KERN_INFO"  Created Mean ts filter len %d depth %d thresh %d\n",
	       tsfs->config->extent, count_coords,
	       tsfs->config->averaging_threshold);

	return &tsfs->tsf;
}

static void ts_filter_mean_destroy(struct platform_device *pdev,
				   struct ts_filter *tsf)
{
	struct ts_filter_mean *tsfs = (struct ts_filter_mean *)tsf;

	kfree(tsfs->fifo[0]); /* first guy has pointer from kmalloc */
	kfree(tsf);
}

static void ts_filter_mean_scale(struct ts_filter *tsf, int *coords)
{
	if (tsf->next) /* chain */
		(tsf->next->api->scale)(tsf->next, coords);
}

/* give us the raw sample data in x and y, and if we return 1 then you can
 * get a filtered coordinate from tsm->x and tsm->y: if we return 0 you didn't
 * fill the filter with samples yet.
 */

static int ts_filter_mean_process(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_mean *tsfs = (struct ts_filter_mean *)tsf;
	int n;
	int len;

	for (n = 0; n < tsf->count_coords; n++) {

		/* has he moved far enough away that we should abandon current
		 * low pass filtering state?
		 */
		if ((coords[n] < (tsfs->reported[n] -
					  tsfs->config->averaging_threshold)) ||
		    (coords[n] > (tsfs->reported[n] +
					  tsfs->config->averaging_threshold))) {
			tsfs->fhead[n] = 0;
			tsfs->ftail[n] = 0;
			tsfs->lowpass[n] = 0;
		}

		/* capture this sample into fifo and sum */
		tsfs->fifo[n][tsfs->fhead[n]++] = coords[n];
		if (tsfs->fhead[n] == tsfs->config->extent)
			tsfs->fhead[n] = 0;
		tsfs->lowpass[n] += coords[n];

		/* adjust the sum into an average and use that*/
		len = (tsfs->fhead[n] - tsfs->ftail[n]) &
						     (tsfs->config->extent - 1);
		coords[n] = (tsfs->lowpass[n] + (len >> 1)) / len;
		tsfs->reported[n] = coords[n];

		/* remove oldest sample if we are full */
		if (len == (tsfs->config->extent - 1)) {
			tsfs->lowpass[n] -= tsfs->fifo[n][tsfs->ftail[n]++];
			if (tsfs->ftail[n] == tsfs->config->extent)
				tsfs->ftail[n] = 0;
		}
	}

	if (tsf->next) /* chain */
		return (tsf->next->api->process)(tsf->next, coords);

	return 1;
}

struct ts_filter_api ts_filter_mean_api = {
	.create = ts_filter_mean_create,
	.destroy = ts_filter_mean_destroy,
	.clear = ts_filter_mean_clear,
	.process = ts_filter_mean_process,
	.scale = ts_filter_mean_scale,
};
