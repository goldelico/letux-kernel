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

/* TODO: comment every field */
struct ts_filter_median_configuration {
	int extent;
	int midpoint;
	int decimation_threshold;
	int decimation_above;
	int decimation_below;
};

extern struct ts_filter_api ts_filter_median_api;

#endif
