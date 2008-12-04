#ifndef __TS_FILTER_VARIANCE_H__
#define __TS_FILTER_VARIANCE_H__

#include <linux/ts_filter.h>

/*
 * touchscreen filter
 *
 * Variance
 *
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 *
 */

struct ts_filter_variance_configuration {
	int extent;
	int window;
	int threshold;
	int attempts;
};

struct ts_filter_variance {
	struct ts_filter tsf;
	struct ts_filter_variance_configuration *config;

	int *samples[2];
	int N;			/* How many samples we have */

	int tries_left;		/* How many times we can try to get a point */
	int passed;		/* Did the samples pass the test? */
};

extern struct ts_filter_api ts_filter_variance_api;

#endif
