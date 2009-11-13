/*
 * isp_csi.h
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_CSI_H
#define OMAP_ISP_CSI_H

#include <linux/videodev2.h>

struct isp_csi_interface_cfg {
	unsigned crc:1;
	unsigned mode:1;
	unsigned edge:1;
	unsigned signalling:1;
	unsigned strobe_clock_inv:1;
	unsigned vs_edge:1;
	unsigned channel:3;
	unsigned vpclk:2;
	unsigned int data_start;
	unsigned int data_size;
	u32 format;
};

/**
 * struct isp_csi_device - ISP CSI1/CCP2 device structure
 * @if_device: Interface enable.
 */
struct isp_csi_device {
	bool if_enabled;
};

void isp_csi_enable(struct isp_csi_device *isp_csi, u8 enable);
int isp_csi_configure_interface(struct isp_csi_device *isp_csi,
				struct isp_csi_interface_cfg *config);

#endif	/* OMAP_ISP_CSI_H */

