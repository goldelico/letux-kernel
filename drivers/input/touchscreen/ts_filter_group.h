#ifndef __TS_FILTER_GROUP_H__
#define __TS_FILTER_GROUP_H__

#include "ts_filter.h"

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

extern struct ts_filter_api ts_filter_group_api;

#endif
