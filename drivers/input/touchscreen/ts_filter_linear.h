#ifndef __TS_FILTER_LINEAR_H__
#define __TS_FILTER_LINEAR_H__

#include "ts_filter.h"
#include <linux/kobject.h>

/*
 * Touchscreen linear filter.
 *
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 *
 */

#define TS_FILTER_LINEAR_NCONSTANTS 7

/* filter configuration */

/* FIXME: comment every field. */
struct ts_filter_linear_configuration {
	int constants[TS_FILTER_LINEAR_NCONSTANTS];
	int coord0;
	int coord1;
};

extern struct ts_filter_api ts_filter_linear_api;

#endif
