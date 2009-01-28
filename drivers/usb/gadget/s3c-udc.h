/*
 * drivers/usb/gadget/s3c-udc.h
 * Samsung S3C on-chip full/high speed USB device controllers
 *
 * Copyright (C) 2008 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __S3C_UDC_H
#define __S3C_UDC_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <asm/byteorder.h>
#include <linux/io.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/hardware.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

/* Max packet size */
#if defined(CONFIG_USB_GADGET_S3C_FS)
#define EP0_FIFO_SIZE		8
#define EP_FIFO_SIZE		64
#define S3C_MAX_ENDPOINTS	5
#elif defined(CONFIG_USB_GADGET_S3C_HS)
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define EP_FIFO_SIZE2		1024
#define S3C_MAX_ENDPOINTS	9
#else
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define EP_FIFO_SIZE2		1024
#define S3C_MAX_ENDPOINTS	16
#endif

#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4

enum ep_type {
	ep_control, ep_bulk_in, ep_bulk_out, ep_interrupt
};

struct s3c_ep {
	struct usb_ep ep;
	struct s3c_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;

	u8 stopped;
	u8 bEndpointAddress;
	u8 bmAttributes;

	u32 ep_type;
	u32 fifo;
#ifdef CONFIG_USB_GADGET_S3C_FS
	u32 csr1;
	u32 csr2;
#endif
};

struct s3c_request {
	struct usb_request req;
	struct list_head queue;
};

struct s3c_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *dev;
	spinlock_t lock;

	int phyclk;
	int ep0state;
	struct s3c_ep ep[S3C_MAX_ENDPOINTS];

	unsigned char usb_address;

	unsigned req_pending:1;
	unsigned req_std:1;
	unsigned req_config:1;
};

extern struct s3c_udc *the_controller;

#define ep_is_in(EP)		(((EP)->bEndpointAddress & USB_DIR_IN) \
								== USB_DIR_IN)
#define ep_index(EP) 		((EP)->bEndpointAddress & 0xF)
#define ep_maxpacket(EP) 	((EP)->ep.maxpacket)

#endif
