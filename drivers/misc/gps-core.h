// SPDX-License-Identifier: GPL-2.0
/*
 * GPS core framework
 *
 * Copyright (C) 2015-2017 H. Nikolaus Schaller <hns@goldelico.com>,
 *						Golden Delicious Computers
 */

#include <linux/tty.h>

struct gps_dev {
	/* public */

	/* how often we were opened */
	int	open_count;
	/* should power on */
	int	(*open)(struct gps_dev *gdev);
	/* should power off */
	int	(*close)(struct gps_dev *gdev);
	/* send raw commands to chip */
	int	(*send)(struct gps_dev *gdev, const char *data, int count);

	/* internal */
	struct	tty_driver *tty_drv;	/* this is the user space tty */
	struct	device *dev;	/* from tty_port_register_device() */
	struct	tty_port port;
};

int gps_register_dev(struct gps_dev *gdev, const char *name);
void gps_unregister_dev(struct gps_dev *gdev);
int gps_recv_nmea_chars(struct gps_dev *gdev, const char *data, int count);
