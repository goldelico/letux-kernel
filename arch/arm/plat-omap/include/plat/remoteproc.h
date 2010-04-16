/*
 * OMAP Remote Processor driver
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Written by Ohad Ben-Cohen <ohad@wizery.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef REMOTEPROC_H
#define REMOTEPROC_H

#include <linux/ioctl.h>
#include <linux/cdev.h>

#define RPROC_IOC_MAGIC		'P'

#define RPROC_IOCMONITOR	_IO(RPROC_IOC_MAGIC, 0)
#define RPROC_IOCSTART		_IO(RPROC_IOC_MAGIC, 1)
#define RPROC_IOCSTOP		_IO(RPROC_IOC_MAGIC, 2)
#define RPROC_IOCGETSTATE	_IOR(RPROC_IOC_MAGIC, 3, int)

#define RPROC_IOC_MAXNR		(3)

struct omap_rproc;

struct omap_rproc_ops {
	int (*startup)(struct omap_rproc *rproc);
	int (*shutdown)(struct omap_rproc *rproc);
	int (*start)(struct omap_rproc *rproc);
	int (*stop)(struct omap_rproc *rproc);
	int (*get_state)(struct omap_rproc *rproc);
};

struct omap_rproc_clk_t {
	void *clk_handle;
	const char *dev_id;
	const char *con_id;
};

struct omap_rproc_platform_data {
	struct omap_rproc_ops *ops;
	char *name;
	char *oh_name;
};

struct omap_rproc {
	struct device *dev;
	struct cdev cdev;
	atomic_t count;
	int state;
	int minor;
};

extern struct omap_rproc_platform_data *remoteproc_get_plat_data(void);
extern int remoteproc_get_plat_data_size(void);

struct omap_rproc *rproc_get(const char *name);
void rproc_put(struct omap_rproc *obj);

#endif /* REMOTEPROC_H */
