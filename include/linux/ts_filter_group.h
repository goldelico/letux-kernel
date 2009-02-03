#ifndef __TS_FILTER_GROUP_H__
#define __TS_FILTER_GROUP_H__

#include <linux/ts_filter.h>

/*
 * Touchscreen group filter.
 *
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 *
 */

struct ts_filter_group_configuration {
	int extent;
	int close_enough;
	int threshold;
	int attempts;
};

struct ts_filter_group {
	struct ts_filter tsf;
	struct ts_filter_group_configuration *config;

	int N;		/* How many samples we have */
	int *samples[MAX_TS_FILTER_COORDS];	/* the samples, our input */

	int *group_size;	/* used for temporal computations */
	int *sorted_samples;	/* used for temporal computations */

	int range_max[MAX_TS_FILTER_COORDS];	/* max computed ranges */
	int range_min[MAX_TS_FILTER_COORDS];	/* min computed ranges */

	int tries_left;		/* We finish if we don't get enough samples */
};

extern struct ts_filter_api ts_filter_group_api;

#endif
