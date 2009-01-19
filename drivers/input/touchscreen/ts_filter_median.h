#ifndef __TS_FILTER_MEDIAN_H__
#define __TS_FILTER_MEDIAN_H__

#include "ts_filter.h"

/*
 * Touchscreen filter.
 *
 * median
 *
 * (c) 2008 Andy Green <andy@openmoko.com>
 */

struct ts_filter_median_configuration {
	int extent;
	int midpoint;
	int decimation_threshold;
	int decimation_above;
	int decimation_below;
};

struct ts_filter_median {
	struct ts_filter tsf;
	struct ts_filter_median_configuration *config;

	int decimation_count;
	int last_issued[MAX_TS_FILTER_COORDS];
	int valid; /* how many samples in the sort buffer are valid */
	int *sort[MAX_TS_FILTER_COORDS]; /* samples taken for median */
	int *fifo[MAX_TS_FILTER_COORDS]; /* samples taken for median */
	int pos; /* where we are in the fifo sample memory */
};

extern struct ts_filter_api ts_filter_median_api;

#endif
