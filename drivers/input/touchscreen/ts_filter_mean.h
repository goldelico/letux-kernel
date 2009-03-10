#ifndef __TS_FILTER_MEAN_H__
#define __TS_FILTER_MEAN_H__

#include "ts_filter.h"

/*
 * Touchscreen filter.
 *
 * mean
 *
 * (c) 2008,2009
 *     Andy Green <andy@openmoko.com>
 *     Nelson Castillo <arhuaco@freaks-unidos.net>
 */

/* Configuration for this filter. */
struct ts_filter_mean_configuration {
	/* Number of points for the mean. */
	int length;
};

/* API functions for the mean filter */
extern struct ts_filter_api ts_filter_mean_api;

#endif /* __TS_FILTER_MEAN_H__ */
