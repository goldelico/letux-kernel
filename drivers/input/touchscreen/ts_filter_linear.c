/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License, or
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
 * Copyright (C) 2008 by Openmoko, Inc.
 * Author: Nelson Castillo <arhuaco@freaks-unidos.net>
 * All rights reserved.
 *
 * Linearly scale touchscreen values.
 *
 * Expose the TS_FILTER_LINEAR_NCONSTANTS for the linear transformation
 * using sysfs.
 *
 */

#include <linux/ts_filter_linear.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>


/*
 * sysfs functions
 */


static ssize_t const_attr_show(struct kobject *kobj,
			       struct attribute *attr,
			       char *buf)
{
	struct const_attribute *a = to_const_attr(attr);

	return a->show(to_const_obj(kobj), a, buf);
}

static ssize_t const_attr_store(struct kobject *kobj,
				struct attribute *attr,
				const char *buf, size_t len)
{
	struct const_attribute *a = to_const_attr(attr);

	return a->store(to_const_obj(kobj), a, buf, len);
}

static struct sysfs_ops const_sysfs_ops = {
	.show =		const_attr_show,
	.store =	const_attr_store,
};

static void const_release(struct kobject *kobj)
{
	kfree(to_const_obj(kobj)->tsfl);
}

static ssize_t const_show(struct const_obj *obj, struct const_attribute *attr,
			  char *buf)
{
	int who;

	sscanf(attr->attr.name, "%d", &who);
	return sprintf(buf, "%d\n", obj->tsfl->constants[who]);
}

static ssize_t const_store(struct const_obj *obj, struct const_attribute *attr,
			   const char *buf, size_t count)
{
	int who;

	sscanf(attr->attr.name, "%d", &who);
	sscanf(buf, "%d", &obj->tsfl->constants[who]);
	return count;
}

/*
 * filter functions
 */

static struct ts_filter *ts_filter_linear_create(struct platform_device *pdev,
						 void *conf, int count_coords)
{
	struct ts_filter_linear *tsfl;
	int i;
	int ret;

	tsfl = kzalloc(sizeof(struct ts_filter_linear), GFP_KERNEL);
	if (!tsfl)
		return NULL;

	tsfl->config = (struct ts_filter_linear_configuration *)conf;
	tsfl->tsf.count_coords = count_coords;

	for (i = 0; i < TS_FILTER_LINEAR_NCONSTANTS; ++i) {
		tsfl->constants[i] = tsfl->config->constants[i];

		/* sysfs */
		sprintf(tsfl->attr_names[i], "%d", i);
		tsfl->kattrs[i].attr.name = tsfl->attr_names[i];
		tsfl->kattrs[i].attr.mode = 0666;
		tsfl->kattrs[i].show = const_show;
		tsfl->kattrs[i].store = const_store;
		tsfl->attrs[i] = &tsfl->kattrs[i].attr;
	}
	tsfl->attrs[i] = NULL;

	tsfl->const_ktype.sysfs_ops = &const_sysfs_ops;
	tsfl->const_ktype.release = const_release;
	tsfl->const_ktype.default_attrs = tsfl->attrs;
	tsfl->c_obj.tsfl = tsfl; /* kernel frees tsfl in const_release */

	/* TODO: /sys/ts-calibration is not OK */
	ret = kobject_init_and_add(&tsfl->c_obj.kobj, &tsfl->const_ktype,
				   &pdev->dev.kobj, "calibration");
	if (ret) {
		kobject_put(&tsfl->c_obj.kobj);
		return NULL;
	}

	printk(KERN_INFO"  Created Linear ts filter depth %d\n", count_coords);

	return &tsfl->tsf;
}

static void ts_filter_linear_destroy(struct platform_device *pdev,
				     struct ts_filter *tsf)
{
	struct ts_filter_linear *tsfl = (struct ts_filter_linear *)tsf;

	/* kernel frees tsfl in const_release */
	kobject_put(&tsfl->c_obj.kobj);
}

static void ts_filter_linear_clear(struct ts_filter *tsf)
{
	if (tsf->next) /* chain */
		(tsf->next->api->clear)(tsf->next);
}


static void ts_filter_linear_scale(struct ts_filter *tsf, int *coords)
{
	struct ts_filter_linear *tsfl = (struct ts_filter_linear *)tsf;
	int *k = tsfl->constants;
	int c0 = coords[tsfl->config->coord0];
	int c1 = coords[tsfl->config->coord1];

	coords[tsfl->config->coord0] = (k[2] + k[0] * c0 + k[1] * c1) / k[6];
	coords[tsfl->config->coord1] = (k[5] + k[3] * c0 + k[4] * c1) / k[6];

	if (tsf->next)
		(tsf->next->api->scale)(tsf->next, coords);
}

static int ts_filter_linear_process(struct ts_filter *tsf, int *coords)
{
	if (tsf->next)
		return (tsf->next->api->process)(tsf->next, coords);

	return 1;
}

struct ts_filter_api ts_filter_linear_api = {
	.create = ts_filter_linear_create,
	.destroy = ts_filter_linear_destroy,
	.clear = ts_filter_linear_clear,
	.process = ts_filter_linear_process,
	.scale = ts_filter_linear_scale,
};
