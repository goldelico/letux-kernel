#ifndef __TS_FILTER_LINEAR_H__
#define __TS_FILTER_LINEAR_H__

#include <linux/ts_filter.h>
#include <linux/kobject.h>

/*
 * touchscreen linear filter.
 *
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 *
 */

#define TS_FILTER_LINEAR_NCONSTANTS 7

/* sysfs */

struct ts_filter_linear;

struct const_obj {
	struct ts_filter_linear *tsfl;
	struct kobject kobj;
};

#define to_const_obj(x) container_of(x, struct const_obj, kobj)

struct const_attribute {
	struct attribute attr;
	ssize_t (*show)(struct const_obj *const, struct const_attribute *attr,
			char *buf);
	ssize_t (*store)(struct const_obj *const, struct const_attribute *attr,
			 const char *buf, size_t count);
};

#define to_const_attr(x) container_of(x, struct const_attribute, attr)

/* filter configuration */

struct ts_filter_linear_configuration {
	int constants[TS_FILTER_LINEAR_NCONSTANTS];
	int coord0;
	int coord1;
};

/* the filter */

struct ts_filter_linear {
	struct ts_filter tsf;
	struct ts_filter_linear_configuration *config;

	int constants[TS_FILTER_LINEAR_NCONSTANTS];

	/* sysfs */
	struct const_obj c_obj;
	struct kobj_type const_ktype;
	struct const_attribute kattrs[TS_FILTER_LINEAR_NCONSTANTS];
	struct attribute *attrs[TS_FILTER_LINEAR_NCONSTANTS + 1];
	char attr_names[TS_FILTER_LINEAR_NCONSTANTS][2];
};

extern struct ts_filter_api ts_filter_linear_api;

#endif
