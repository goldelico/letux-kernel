#ifndef __TS_FILTER_MEAN_H__
#define __TS_FILTER_MEAN_H__

#include "ts_filter.h"

/*
 * Touchscreen filter.
 *
 * mean
 *
 * (c) 2008 Andy Green <andy@openmoko.com>
 */

struct ts_filter_mean_configuration {
	int bits_filter_length;
	int averaging_threshold;

	int extent;
};

struct ts_filter_mean {
	struct ts_filter tsf;
	struct ts_filter_mean_configuration *config;

	int reported[MAX_TS_FILTER_COORDS];
	int lowpass[MAX_TS_FILTER_COORDS];
	int *fifo[MAX_TS_FILTER_COORDS];
	int fhead[MAX_TS_FILTER_COORDS];
	int ftail[MAX_TS_FILTER_COORDS];
};

extern struct ts_filter_api ts_filter_mean_api;

#endif
