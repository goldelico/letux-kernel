/*
 * drivers/usb/gadget/s3c_hs_otg.c
 * Samsung S3C on-chip full/high speed USB OTG 2.0 device controllers
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

#include "s3c-udc.h"
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/map.h>
#include <plat/regs-clock.h>
#include <plat/regs-usb-hs-otg.h>
#include <plat/regs-sys.h>
#include <plat/devs.h>

static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV"
};

#define S3C_USB_DBG_LEVEL 0

#define DBG(level, fmt, args...) do { \
	if (level >= S3C_USB_DBG_LEVEL) { \
		printk(KERN_INFO "[%s] " fmt, \
				__func__, ##args); \
	} } while (0)


#define	DRIVER_DESC "Samsung Dual-speed USB 2.0 OTG Device Controller"
#define DRIVER_AUTHOR "Samsung Electronics"
#define	DRIVER_VERSION "04 Dec 2008"


struct s3c_udc	*the_controller;

static const char driver_name[] = "s3c-otg-device";
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

static u32 tx_ep_num = 2;

static u32 ep0_fifo_size = EP0_FIFO_SIZE;
static u32 ep_fifo_size =  EP_FIFO_SIZE;
static u32 ep_fifo_size2 = EP_FIFO_SIZE2;

struct usb_ctrlrequest ctrl;
static int reset_available = 1;

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

static const char proc_node_name[] = "driver/otg";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char *buf = page;
	struct s3c_udc *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size,
		      DRIVER_DESC "\n"
		      "%s version: %s\n"
		      "Gadget driver: %s\n"
		      "\n",
		      driver_name, DRIVER_VERSION,
		      dev->driver ? dev->driver->driver.name : "(none)");
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif	/* CONFIG_USB_GADGET_DEBUG_FILES */


static u32 s3c_otg_readl(struct s3c_udc *dev, u32 reg)
{
	return __raw_readl((u32)dev->reg_base + reg);
}

static void s3c_otg_writel(struct s3c_udc *dev, u32 val, u32 reg)
{
	__raw_writel(val, ((u32)dev->reg_base) + reg);
}

static void s3c_otg_orl(struct s3c_udc *dev, u32 val, u32 reg)
{
	u32 temp = __raw_readl(((u32)dev->reg_base) + reg);

	__raw_writel(val|temp, ((u32)dev->reg_base) + reg);
}

/*
 *	retire a request
 */
static void s3c_otg_done(struct s3c_ep *ep, struct s3c_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	DBG(1, "%s %p, stopped = %d\n", ep->ep.name, ep, stopped);
	list_del_init(&req->queue);

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	if (status && (status != -ESHUTDOWN))
		DBG(2, "complete %s stat %d len %u/%u\n",
			ep->ep.name, status, req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);

	ep->stopped = stopped;
}

/*
 * 	dequeue ALL requests
 */
void s3c_otg_nuke(struct s3c_ep *ep, int status)
{
	struct s3c_request *req;

	DBG(1, "%s %p\n", ep->ep.name, ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c_request, queue);
		s3c_otg_done(ep, req, status);
	}
}

static void s3c_otg_ep_control(int ep, int dir, u32 val, int update)
{
	u32 epctrl;

	switch (ep) {
	case 0:
		if (dir)
			epctrl = (u32)S3C_UDC_OTG_DIEPCTL0;
		else
			epctrl = (u32)S3C_UDC_OTG_DOEPCTL0;
		break;
	case 1:
		if (dir)
			epctrl = -EOPNOTSUPP;
		else
			epctrl = (u32)S3C_UDC_OTG_DOEPCTL1;
		break;
	case 2:
		if (dir)
			epctrl = (u32)S3C_UDC_OTG_DIEPCTL2;
		else
			epctrl = -EOPNOTSUPP;
		break;
	case 3:
		if (dir)
			epctrl = (u32)S3C_UDC_OTG_DIEPCTL3;
		else
			epctrl = -EOPNOTSUPP;
		break;
	default:
		DBG(3, "ep%d is unused Endpoint", ep);
		return;
	}

	if (epctrl < 0) {
		DBG(3, "ep%d - %s is invalid direction\n",
				ep, dir ? "IN" : "OUT");
		return;
	}

	if (update)
		s3c_otg_orl(the_controller, val, epctrl);
	else
		s3c_otg_writel(the_controller, val, epctrl);
}

static int s3c_otg_write_packet(struct s3c_ep *ep,
		struct s3c_request *req, int max)
{
	u32 *buf;
	int length;
	int count;
	u32 fifo = ep->fifo;
	u32 epsize;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	length = min(length, max);
	req->req.actual += length;

	DBG(1, "%s: %d/%d, fifo=0x%x\n", ep->ep.name, length, max, fifo);

	switch (ep_index(ep)) {
	case 0:
		epsize = (u32)S3C_UDC_OTG_DIEPTSIZ0;
		break;
	case 2:
		epsize = (u32)S3C_UDC_OTG_DIEPTSIZ2;
		break;
	case 3:
		epsize = (u32)S3C_UDC_OTG_DIEPTSIZ3;
		break;
	default:
		DBG(3, "ep%d is unused Endpoint", ep_index(ep));
		return 0;
	}

	s3c_otg_writel(ep->dev, PKT_CNT(0x1)|XFERSIZE(length), epsize);
	s3c_otg_ep_control(ep_index(ep), USB_DIR_IN,
			DEPCTL_EPENA|DEPCTL_CNAK, 1);

	for (count = 0; count < length; count += 4)
		s3c_otg_writel(ep->dev, *buf++, fifo);

	return length;
}

static int s3c_otg_write_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 max;
	unsigned count;
	int is_last;

	max = ep_maxpacket(ep);
	count = s3c_otg_write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (count != max) {
		is_last = 1;
	} else {
		if ((req->req.length != req->req.actual) || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
	}

	DBG(2, "wrote %s %d bytes%s %d left %p\n",
			ep->ep.name, count,	is_last ? "/L" : "",
			req->req.length - req->req.actual, req);

	/* requests complete when all IN data is in the FIFO */
	return is_last;
}

static int s3c_otg_read_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u32 *buf;
	unsigned bufferspace;
	unsigned count;
	unsigned is_short;
	unsigned bytes;
	u32 fifo = ep->fifo;

	csr = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GRXSTSP);
	bytes = BYTE_COUNT(csr);

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (EP_NUM(csr) == 0) {
		count = bytes / 4 + (bytes % 4 ? 1 : 0);
		req->req.actual += min(bytes, bufferspace);
	} else {
		count = 0;
		bytes = 0;
	}

	is_short = (bytes < ep->ep.maxpacket);

	DBG(2, "read %s %d bytes%s %d/%d\n",
		  ep->ep.name, bytes, is_short ? "/S" : "",
		  req->req.actual, req->req.length);

	while (count--) {
		u32 byte = s3c_otg_readl(ep->dev, fifo);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DBG(3, "%s overflow %d\n", ep->ep.name, count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace -= 4;
		}
	}

	/* completion */
	if (is_short || req->req.actual == req->req.length)
		return 1;

	return 0;
}

static int s3c_otg_write_ep0(struct s3c_udc *dev)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &dev->ep[0];
	int ret;
	int need_zlp = 0;

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (!req) {
		DBG(2, "NULL REQ\n");
		return 0;
	}

	DBG(2, "length = 0x%x, actual = 0x%x\n",
			req->req.length, req->req.actual);

	if (req->req.length == 0) {
		dev->ep0state = WAIT_FOR_SETUP;
		s3c_otg_done(ep, req, 0);
		return 1;
	}

	/* Next write will end with the packet size, */
	/* so we need Zero-length-packet */
	if (req->req.length - req->req.actual == ep0_fifo_size)
		need_zlp = 1;

	ret = s3c_otg_write_fifo_ep0(ep, req);

	if ((ret == 1) && !need_zlp) {
		/* Last packet */
		DBG(1, "finished, waiting for status\n");
		dev->ep0state = WAIT_FOR_SETUP;
	}

	if (need_zlp) {
		DBG(1, "Need ZLP!\n");
		dev->ep0state = DATA_STATE_NEED_ZLP;
	}

	if (ret)
		s3c_otg_done(ep, req, 0);

	return ret;
}

static int first_time = 1;

static int s3c_otg_read_ep0(struct s3c_udc *dev)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &dev->ep[0];
	int ret;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct s3c_request, queue);
	else {
		DBG(3, "---> BUG\n");
		BUG();
		return 0;
	}

	DBG(2, "length = 0x%x, actual = 0x%x\n",
			req->req.length, req->req.actual);

	if (req->req.length == 0) {
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		s3c_otg_done(ep, req, 0);
		return 1;
	}

	if (!req->req.actual && first_time) {
		first_time = 0;
		return 1;
	}

	ret = s3c_otg_read_fifo_ep0(ep, req);

	if (ret)
		s3c_otg_done(ep, req, 0);

	dev->ep0state = WAIT_FOR_SETUP;
	first_time = 1;

	return ret;
}

static void s3c_otg_kick_ep0(struct s3c_udc *dev, struct s3c_ep *ep)
{
	int res = 0;

	DBG(1, "ep_is_in = %d\n", ep_is_in(ep));

	if (ep_is_in(ep)) {
		dev->ep0state = DATA_STATE_XMIT;
		while (!res)
			res = s3c_otg_write_ep0(dev);
	} else {
		dev->ep0state = DATA_STATE_RECV;
		s3c_otg_read_ep0(dev);
	}
}

/*
 * Write request to FIFO
 */
static int s3c_otg_write_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 max;
	u32 gintmsk;
	unsigned count;
	int is_last = 0;
	int is_short = 0;

	gintmsk = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GINTMSK);

	max = le16_to_cpu(ep->desc->wMaxPacketSize);
	count = s3c_otg_write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (count != max) {
		is_last = 1;
		is_short = 1;
	} else {
		if ((req->req.length != req->req.actual) || req->req.zero)
			is_last = 0;
		else
			is_last = 1;

		/* interrupt/iso maxpacket may not fill the fifo */
		is_short = (max < ep_maxpacket(ep));
	}

	DBG(2, "wrote %s %d bytes%s%s %d/%d\n",
			ep->ep.name, count,
			is_last ? "/L" : "", is_short ? "/S" : "",
			req->req.actual, req->req.length);

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		if (ep_index(ep) == 0) {
			DBG(3, "--> EP0 must not come here!\n");
			BUG();
		}

		s3c_otg_writel(ep->dev, gintmsk & (~INT_TX_FIFO_EMPTY),
				(u32)S3C_UDC_OTG_GINTMSK);
		s3c_otg_done(ep, req, 0);

		return 1;
	}

	s3c_otg_writel(ep->dev, gintmsk|INT_TX_FIFO_EMPTY,
			(u32)S3C_UDC_OTG_GINTMSK);

	return 0;
}

/*
 * Read to request from FIFO (max read == bytes in fifo)
 */
static int s3c_otg_read_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u32 gintmsk;
	u32 *buf;
	unsigned bufferspace;
	unsigned count;
	unsigned is_short = 0;
	unsigned bytes;
	u32 fifo = ep->fifo;

	csr = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GRXSTSP);
	bytes = BYTE_COUNT(csr);
	gintmsk = readl(S3C_UDC_OTG_GINTMSK);

	if (!bytes) {
		DBG(2, "%d bytes\n", bytes);
		s3c_otg_orl(ep->dev, INT_RX_FIFO_NOT_EMPTY,
				(u32)S3C_UDC_OTG_GINTMSK);
		return 0;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	count = bytes / 4 + (bytes % 4 ? 1 : 0);
	req->req.actual += min(bytes, bufferspace);

	is_short = (bytes < ep->ep.maxpacket);

	DBG(2, "read %s %d bytes%s %d/%d\n",
			ep->ep.name, bytes, is_short ? "/S" : "",
			req->req.actual, req->req.length);

	while (count--) {
		u32 byte = s3c_otg_readl(ep->dev, fifo);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DBG(3, "%s overflow %d\n", ep->ep.name, count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace -= 4;
		}
	}

	s3c_otg_writel(ep->dev, gintmsk|INT_RX_FIFO_NOT_EMPTY,
			(u32)S3C_UDC_OTG_GINTMSK);

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		s3c_otg_done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

static struct usb_request *s3c_otg_alloc_request(
		struct usb_ep *ep, gfp_t gfp_flags)
{
	struct s3c_request *req;

	if (!ep)
		return NULL;

	DBG(1, "%s %p\n", ep->name, ep);

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void s3c_otg_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c_request *req;

	if (!ep)
		return;

	DBG(1, "%s %p\n", ep->name, ep);

	if (!_req)
		return;

	req = container_of(_req, struct s3c_request, req);

	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*
 * Queue one request
 *  Kickstart transfer if needed
 */
static int s3c_otg_queue(struct usb_ep *_ep,
		struct usb_request *_req, gfp_t gfp_flags)
{
	struct s3c_request *req;
	struct s3c_ep *ep;
	struct s3c_udc *dev;
	unsigned long flags;
	u32 csr;

	req = container_of(_req, struct s3c_request, req);
	if (!_req || !_req->complete || !_req->buf
			|| !list_empty(&req->queue)) {
		DBG(3, "bad params\n");
		return -EINVAL;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || (!ep->desc && ep->ep.name != ep0name)) {
		DBG(3, "bad ep\n");
		return -EINVAL;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DBG(3, "bogus device state %p\n", dev->driver);
		return -ESHUTDOWN;
	}

	DBG(2, "%s queue req %p, len %d buf %p\n",
			_ep->name, _req, _req->length, _req->buf);

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	DBG(2, "ep=%d, Q empty=%d, stopped=%d\n",
			ep_index(ep), list_empty(&ep->queue), ep->stopped);

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep_index(ep) == 0) {
			list_add_tail(&req->queue, &ep->queue);
			s3c_otg_kick_ep0(dev, ep);
			req = NULL;
		} else if (ep_is_in(ep)) {
			csr = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GINTSTS);

			if ((csr & INT_TX_FIFO_EMPTY) &&
			   (s3c_otg_write_fifo(ep, req) == 1))
				req = NULL;
			else
				tx_ep_num = ep_index(ep);
		} else {
			csr = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GINTSTS);

			if ((csr & INT_RX_FIFO_NOT_EMPTY) &&
			   (s3c_otg_read_fifo(ep, req) == 1))
				req = NULL;
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (req)
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/*
 * dequeue JUST ONE request
 */
static int s3c_otg_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	unsigned long flags;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}

	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	s3c_otg_done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

static int s3c_otg_set_halt(struct usb_ep *_ep, int value)
{
	return 0;
}

static int s3c_otg_fifo_status(struct usb_ep *_ep)
{
	int count = 0;
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep) {
		DBG(3, "bad ep\n");
		return -ENODEV;
	}

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	return count;
}

static void s3c_otg_fifo_flush(struct usb_ep *_ep)
{
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DBG(3, "bad ep\n");
		return;
	}
}

static int s3c_otg_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct s3c_ep *ep;
	struct s3c_udc *dev;
	unsigned long flags;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress
	    || ep_maxpacket(ep) < le16_to_cpu(desc->wMaxPacketSize)) {
		DBG(3, "bad ep or descriptor\n");
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
			&& ep->bmAttributes != USB_ENDPOINT_XFER_BULK
			&& desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DBG(3, "%s type mismatch\n", _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
			&& le16_to_cpu(desc->wMaxPacketSize)
			!= ep_maxpacket(ep))
			|| !desc->wMaxPacketSize) {
		DBG(3, "bad %s maxpacket\n", _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DBG(3, "bogus device state\n");
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* Reset halt state */
	s3c_otg_set_halt(_ep, 0);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DBG(2, "enabled %s, stopped = %d, maxpacket = %d\n",
		_ep->name, ep->stopped, ep->ep.maxpacket);
	return 0;
}

static int s3c_otg_ep_disable(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !ep->desc) {
		DBG(3, "%s not enabled\n", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Nuke all pending requests */
	s3c_otg_nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DBG(2, "disabled %s\n", _ep->name);
	return 0;
}

static struct usb_ep_ops s3c_ep_ops = {
	.enable = s3c_otg_ep_enable,
	.disable = s3c_otg_ep_disable,

	.alloc_request = s3c_otg_alloc_request,
	.free_request = s3c_otg_free_request,

	.queue = s3c_otg_queue,
	.dequeue = s3c_otg_dequeue,

	.set_halt = s3c_otg_set_halt,
	.fifo_status = s3c_otg_fifo_status,
	.fifo_flush = s3c_otg_fifo_flush,
};

void s3c_otg_set_ep(struct s3c_udc *dev, enum usb_device_speed speed)
{
	u32 ep0_mps = DEPCTL0_MPS_64;

	if (speed == USB_SPEED_FULL) {
		ep0_fifo_size = 8;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;

		ep0_mps = DEPCTL0_MPS_8;
	}

	dev->gadget.speed = speed;

	dev->ep[0].ep.maxpacket = ep0_fifo_size;
	dev->ep[1].ep.maxpacket = ep_fifo_size;
	dev->ep[2].ep.maxpacket = ep_fifo_size;
	dev->ep[3].ep.maxpacket = ep_fifo_size;
	dev->ep[4].ep.maxpacket = ep_fifo_size;
	dev->ep[5].ep.maxpacket = ep_fifo_size2;
	dev->ep[6].ep.maxpacket = ep_fifo_size2;
	dev->ep[7].ep.maxpacket = ep_fifo_size2;
	dev->ep[8].ep.maxpacket = ep_fifo_size2;

	/* EP0 - Control */
	s3c_otg_ep_control(0, USB_DIR_OUT, ep0_mps, 1);
	s3c_otg_ep_control(0, USB_DIR_IN, ep0_mps, 1);

	/* EP1 - Bulk Data OUT */
	s3c_otg_ep_control(1, USB_DIR_OUT, ep_fifo_size, 1);

	/* EP2 - Bulk Data IN */
	s3c_otg_ep_control(2, USB_DIR_IN, ep_fifo_size, 1);

	/* EP3 - INTR Data IN */
	s3c_otg_ep_control(3, USB_DIR_IN, ep_fifo_size, 1);

	DBG(2, "%s Speed Detection\n",
			speed == USB_SPEED_HIGH ? "High" : "Full");
}

/*
 * set the USB address for this device
 *
 * Called from control endpoint function
 * after it decodes a set address setup packet.
 */
static void s3c_otg_set_address(struct s3c_udc *dev, unsigned char addr)
{
	s3c_otg_orl(dev, DEVICE_ADDR(addr), S3C_UDC_OTG_DCFG);
	s3c_otg_ep_control(0, USB_DIR_IN, DEPCTL_EPENA|DEPCTL_CNAK, 1);

	DBG(2, "USB OTG 2.0 Device Address=%d\n", addr);

	dev->usb_address = addr;
}

static inline int s3c_otg_read_setup(struct s3c_ep *ep, u32 *ctrl, int max)
{
	int bytes;
	int count;
	u32 csr = s3c_otg_readl(ep->dev, S3C_UDC_OTG_GRXSTSP);

	bytes = BYTE_COUNT(csr);

	/* 32 bits interface */
	count = bytes / 4;

	while (count--)
		*ctrl++ = s3c_otg_readl(ep->dev, S3C_UDC_OTG_EP0_FIFO);

	return bytes;
}

static void s3c_otg_setup(struct s3c_udc *dev)
{
	struct s3c_ep *ep = &dev->ep[0];
	int bytes;
	int is_in;
	int ret;

	/* Nuke all previous transfers */
	s3c_otg_nuke(ep, -EPROTO);

	/* read control req from fifo (8 bytes) */
	bytes = s3c_otg_read_setup(ep, (u32 *)&ctrl, 8);

	DBG(2, "SETUP REQ %02x %02x %04x %04x %d\n",
			ctrl.bRequestType, ctrl.bRequest,
			ctrl.wValue, ctrl.wIndex, ctrl.wLength);

	/* Set direction of EP0 */
	if (ctrl.bRequestType & USB_DIR_IN) {
		ep->bEndpointAddress |= USB_DIR_IN;
		is_in = 1;
	} else {
		ep->bEndpointAddress &= ~USB_DIR_IN;
		is_in = 0;
	}

	dev->req_pending = 1;

	/* Handle some SETUP packets ourselves */
	switch (ctrl.bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (ctrl.bRequestType != (USB_TYPE_STANDARD|USB_RECIP_DEVICE))
			break;

		s3c_otg_set_address(dev, ctrl.wValue);
		return;

	case USB_REQ_SET_INTERFACE:
		DBG(2, "USB_REQ_SET_INTERFACE (%d)\n", ctrl.wValue);
		/* FALLTHROUGH */

	case USB_REQ_SET_CONFIGURATION:
		DBG(2, "USB_REQ_SET_CONFIGURATION (%d)\n", ctrl.wValue);

		s3c_otg_ep_control(0, USB_DIR_IN,
				DEPCTL_EPENA|DEPCTL_CNAK, 1);
		s3c_otg_ep_control(1, USB_DIR_OUT,
				DEPCTL_EPDIS|DEPCTL_CNAK|
				DEPCTL_BULK_TYPE|DEPCTL_USBACTEP, 1);
		s3c_otg_ep_control(2, USB_DIR_IN,
				DEPCTL_BULK_TYPE|DEPCTL_USBACTEP, 1);
		s3c_otg_ep_control(3, USB_DIR_IN,
				DEPCTL_BULK_TYPE|DEPCTL_USBACTEP, 1);

		reset_available = 1;
		dev->req_config = 1;
		break;

	case USB_REQ_GET_DESCRIPTOR:
		DBG(2, "USB_REQ_GET_DESCRIPTOR\n");
		break;

	case USB_REQ_GET_CONFIGURATION:
		DBG(2, "USB_REQ_GET_CONFIGURATION\n");
		break;

	case USB_REQ_GET_STATUS:
		DBG(2, "USB_REQ_GET_STATUS\n");
		s3c_otg_ep_control(0, USB_DIR_IN, DEPCTL_EPENA|DEPCTL_CNAK, 1);
		break;

	case USB_REQ_CLEAR_FEATURE:
		DBG(2, "USB_REQ_CLEAR_FEATURE\n");
		break;

	case USB_REQ_SET_FEATURE:
		DBG(2, "USB_REQ_SET_FEATURE\n");
		break;

	default:
		DBG(3, "Default of ctrl.bRequest=0x%x\n", ctrl.bRequest);
		break;
	}

	if (dev->driver) {
		/* device-2-host (IN) or no data setup command,
		 * process immediately */
		spin_unlock(&dev->lock);

		DBG(1, "usb_ctrlrequest will be passed to fsg_setup()\n");

		ret = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (ret < 0) {
			/* setup processing failed, force stall */
			DBG(3, "gadget setup FAILED (stalling) - %d\n", ret);
			dev->ep0state = WAIT_FOR_SETUP;
		}
	}
}

/*
 * handle ep0 interrupt
 */
static void s3c_otg_handle_ep0(struct s3c_udc *dev)
{
	if (dev->ep0state == WAIT_FOR_SETUP)
		s3c_otg_setup(dev);
	else
		DBG(3, "strange state!! - %s\n", state_names[dev->ep0state]);
}

static void s3c_otg_handle_ep_out(struct s3c_udc *dev, u32 ep_num)
{
	struct s3c_ep *ep = &dev->ep[ep_num];
	struct s3c_request *req;

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		DBG(3, "No descriptor?!?\n");
		return;
	}

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (unlikely(!req))
		DBG(2, "NULL REQ on OUT EP-%d\n", ep_num);
	else
		s3c_otg_read_fifo(ep, req);
}

static void s3c_otg_handle_ep_in(struct s3c_udc *dev, u32 ep_num)
{
	struct s3c_ep *ep = &dev->ep[ep_num];
	struct s3c_request *req;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (unlikely(!req)) {
		DBG(2, "NULL REQ on IN EP-%d\n", ep_num);
		return;
	} else
		s3c_otg_write_fifo(ep, req);
}

static void s3c_otg_handle_ep(struct s3c_udc *dev, u32 gintmsk)
{
	u32 csr;
	u32 packet_status;
	u32 ep_num;
	u32 bytes = 0;

	gintmsk &= ~INT_RX_FIFO_NOT_EMPTY;
	s3c_otg_writel(dev, gintmsk, S3C_UDC_OTG_GINTMSK);

	csr = s3c_otg_readl(dev, S3C_UDC_OTG_GRXSTSR);

	packet_status = PKT_STS(csr);
	bytes = BYTE_COUNT(csr);
	ep_num = EP_NUM(csr);

	switch (packet_status) {
	case SETUP_PKT_RECEIVED:
		DBG(2, "SETUP received : %d bytes\n", bytes);
		if (!bytes)
			break;

		s3c_otg_handle_ep0(dev);
		gintmsk |= INT_RX_FIFO_NOT_EMPTY;
		break;

	case OUT_PKT_RECEIVED:
		if (!bytes)
			break;

		if (ep_num == 0) {
			DBG(2, "CONTROL OUT received : %d bytes\n", bytes);

			dev->ep0state = DATA_STATE_RECV;
			s3c_otg_read_ep0(dev);

			gintmsk |= INT_RX_FIFO_NOT_EMPTY;
		} else if (ep_num == 1) {
			DBG(2, " Bulk OUT received : %d bytes\n", bytes);

			s3c_otg_handle_ep_out(dev, 1);
			gintmsk = s3c_otg_readl(dev,S3C_UDC_OTG_GINTMSK);

			s3c_otg_ep_control(1, USB_DIR_OUT, DEPCTL_CNAK, 1);
		} else
			DBG(2, "Unused EP%d: %d bytes\n", ep_num, bytes);
		break;

	case SETUP_COMPLETED:
		DBG(2, "SETUP_COMPLETED\n");
		s3c_otg_ep_control(0, USB_DIR_OUT, DEPCTL_CNAK, 1);
		break;

	case OUT_COMPLELTED:
		DBG(2, "OUT_COMPLELTED - ep%d\n", ep_num);
		s3c_otg_ep_control(ep_num, USB_DIR_OUT, DEPCTL_CNAK, 1);
		break;

	default:
		gintmsk |= INT_RX_FIFO_NOT_EMPTY;
		s3c_otg_ep_control(0, USB_DIR_OUT, DEPCTL_CNAK, 1);
		DBG(1, "reserved packet received : scr=0x%08X bytes\n", csr);
		break;
	}

	if (!bytes) {
		csr = s3c_otg_readl(dev, S3C_UDC_OTG_GRXSTSP);
		gintmsk |= INT_RX_FIFO_NOT_EMPTY;
	}

	s3c_otg_writel(dev, gintmsk, S3C_UDC_OTG_GINTMSK);
}

/*
 * disable USB device controller
 */
static void s3c_otg_disable(struct s3c_udc *dev)
{
	s3c_otg_set_address(dev, 0);

	dev->ep0state = WAIT_FOR_SETUP;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->usb_address = 0;

	s3c_otg_orl(dev, ANALOG_PWR_DOWN, S3C_USBOTG_PHYPWR);
}

/*
 * initialize software state
 */
static void s3c_otg_reinit(struct s3c_udc *dev)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}
}

#define GUSBCFG_INIT	(PHY_CLK_480M|TXFIFO_RE_EN| \
		TURN_AROUND|HNP_DISABLE|SRP_DISABLE|ULPI_DDR| \
		HS_UTMI|INTERF_UTMI|PHY_INTERF_16|TIME_OUT_CAL)

#define GINTMSK_INIT	(INT_RESUME|INT_ENUMDONE| \
		INT_RESET|INT_SUSPEND|INT_RX_FIFO_NOT_EMPTY)

#define DOEPMSK_INIT	(AHB_ERROR)

#define DIEPMSK_INIT	(IN_EP_TIMEOUT|AHB_ERROR)

#define GAHBCFG_INIT	(PTXFE_HALF|NPTXFE_HALF| \
		MODE_SLAVE|BURST_INCR16|GBL_INT_UNMASK)

static void s3c_otg_config(struct s3c_udc *dev)
{
	u32 reg;

	/* OTG USB configuration */
	s3c_otg_writel(dev, GUSBCFG_INIT, S3C_UDC_OTG_GUSBCFG);

	/* Soft-reset OTG Core and then unreset again */
	s3c_otg_writel(dev, CORE_SOFT_RESET, S3C_UDC_OTG_GRSTCTL);

	/* Put the OTG device core in the disconnected state */
	s3c_otg_orl(dev, SOFT_DISCONNECT, S3C_UDC_OTG_DCTL);

	udelay(20);

	/* Make the OTG device core exit from the disconnected state */
	reg = s3c_otg_readl(dev, S3C_UDC_OTG_DCTL);
	s3c_otg_writel(dev, reg & ~SOFT_DISCONNECT, S3C_UDC_OTG_DCTL);

	/* Configure OTG Core to initial settings of device mode */
	s3c_otg_orl(dev, EP_MIS_CNT(0x1)|SPEED_2_HIGH, S3C_UDC_OTG_DCFG);

	udelay(1000);

	/* Unmask the core interrupts */
	s3c_otg_writel(dev, GINTMSK_INIT, S3C_UDC_OTG_GINTMSK);

	/* Set NAK bit of EP0, EP1, EP2 */
	s3c_otg_ep_control(0, USB_DIR_OUT,
			DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL0_MPS_64, 0);
	s3c_otg_ep_control(0, USB_DIR_IN,
			DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL0_MPS_64, 0);
	s3c_otg_ep_control(1, USB_DIR_OUT,
			DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE, 0);
	s3c_otg_ep_control(2, USB_DIR_IN,
			DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE, 0);
	s3c_otg_ep_control(3, USB_DIR_IN,
			DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE, 0);

	/* Unmask EP interrupts */
	s3c_otg_writel(dev, S3C_UDC_INT_IN_EP0
			|S3C_UDC_INT_IN_EP2
			|S3C_UDC_INT_IN_EP3
			|S3C_UDC_INT_OUT_EP0
			|S3C_UDC_INT_OUT_EP1,
			S3C_UDC_OTG_DAINTMSK);

	/* Unmask device OUT EP common interrupts */
	s3c_otg_writel(dev, DOEPMSK_INIT, S3C_UDC_OTG_DOEPMSK);

	/* Unmask device IN EP common interrupts */
	s3c_otg_writel(dev, DIEPMSK_INIT, S3C_UDC_OTG_DIEPMSK);

	/* Set Rx FIFO Size */
	s3c_otg_writel(dev, RX_FIFO_SIZE, S3C_UDC_OTG_GRXFSIZ);

	/* Set Non Periodic Tx FIFO Size */
	s3c_otg_writel(dev, NPTX_FIFO_SIZE|NPTX_FIFO_START_ADDR,
			(u32)S3C_UDC_OTG_GNPTXFSIZ);

	/* Clear NAK bit of EP0 For Slave mode */
	s3c_otg_ep_control(0, USB_DIR_OUT, DEPCTL_EPDIS|DEPCTL_CNAK, 0);

	/* Initialize OTG Link Core */
	s3c_otg_writel(dev, GAHBCFG_INIT, S3C_UDC_OTG_GAHBCFG);
}

static int s3c_otg_enable(struct s3c_udc *dev)
{
	/* USB_SIG_MASK */
	__raw_writel(S3C64XX_OTHERS_USBMASK | __raw_readl(S3C64XX_OTHERS),
								S3C64XX_OTHERS);

	/* Initializes OTG Phy. */
	s3c_otg_writel(dev, SUSPEND_DISABLE, S3C_USBOTG_PHYPWR);
	
	s3c_otg_writel(dev, dev->phyclk, S3C_USBOTG_PHYCLK);

	s3c_otg_writel(dev, SW_RST_ON, S3C_USBOTG_RSTCON);
	udelay(50);

	s3c_otg_writel(dev, SW_RST_OFF, S3C_USBOTG_RSTCON);
	udelay(50);

	s3c_otg_config(dev);

	DBG(2, "S3C USB 2.0 OTG Controller Core Initialized\n");

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	return 0;
}

/*
 *	usb client interrupt handler.
 */
static irqreturn_t s3c_otg_irq(int irq, void *_dev)
{
	struct s3c_udc *dev = _dev;
	u32 intr_status;
	u32 usb_status;
	u32 gintmsk;

	spin_lock(&dev->lock);

	intr_status = s3c_otg_readl(dev, S3C_UDC_OTG_GINTSTS);
	gintmsk = s3c_otg_readl(dev, S3C_UDC_OTG_GINTMSK);

	DBG(1, "GINTSTS=0x%x(on state %s), GINTMSK : 0x%x\n",
			intr_status, state_names[dev->ep0state], gintmsk);

	if (!intr_status) {
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (intr_status & INT_ENUMDONE) {
		DBG(2, "Speed Detection interrupt\n");
		s3c_otg_writel(dev, INT_ENUMDONE, S3C_UDC_OTG_GINTSTS);

		usb_status = ENUM_SPEED(s3c_otg_readl(dev, S3C_UDC_OTG_DSTS));

		if (usb_status & (USB_FULL_30_60MHZ | USB_FULL_48MHZ))
			s3c_otg_set_ep(dev, USB_SPEED_FULL);
		else
			s3c_otg_set_ep(dev, USB_SPEED_HIGH);
	}

	if (intr_status & INT_EARLY_SUSPEND) {
		DBG(2, "Early suspend interrupt\n");
		s3c_otg_writel(dev, INT_EARLY_SUSPEND, S3C_UDC_OTG_GINTSTS);
	}

	if (intr_status & INT_SUSPEND) {
		DBG(2, "Suspend interrupt\n");
		s3c_otg_writel(dev, INT_SUSPEND, S3C_UDC_OTG_GINTSTS);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->driver
		    && dev->driver->suspend) {
			dev->driver->suspend(&dev->gadget);
		}
	}

	if (intr_status & INT_RESUME) {
		DBG(2, "Resume interrupt\n");
		s3c_otg_writel(dev, INT_RESUME, S3C_UDC_OTG_GINTSTS);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->driver
		    && dev->driver->resume) {
			dev->driver->resume(&dev->gadget);
		}
	}

	if (intr_status & INT_RESET) {
		DBG(2, "Reset interrupt\n");
		s3c_otg_writel(dev, INT_RESET, S3C_UDC_OTG_GINTSTS);

		usb_status = s3c_otg_readl(dev, S3C_UDC_OTG_GOTGCTL);

		if (usb_status | (A_SESSION_VALID|B_SESSION_VALID)) {
			if (reset_available) {
				s3c_otg_config(dev);
				dev->ep0state = WAIT_FOR_SETUP;
				reset_available = 0;
			}
		} else {
			reset_available = 1;
			DBG(2, "RESET handling skipped\n");
		}
	}

	if (intr_status & INT_RX_FIFO_NOT_EMPTY) {
		s3c_otg_handle_ep(dev, gintmsk);
		spin_unlock(&dev->lock);

		return IRQ_HANDLED;
	}


	if (intr_status & INT_TX_FIFO_EMPTY) {
		DBG(2, "INT_TX_FIFO_EMPTY ep_num=%d\n",	tx_ep_num);
		s3c_otg_handle_ep_in(dev, tx_ep_num);
	}

	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

static void s3c_otg_stop_activity(struct s3c_udc *dev,
		struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &dev->ep[i];
		ep->stopped = 1;
		s3c_otg_nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	/* re-init driver-visible data structures */
	s3c_otg_reinit(dev);
}

/*
 * Register the gadget driver. Used by gadget drivers when
 * registering themselves with the controller.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct s3c_udc *dev = the_controller;
	int retval;

	if (!driver
	    || driver->speed != USB_SPEED_HIGH
	    || !driver->bind
	    || !driver->setup)
		return -EINVAL;

	if (!dev) {
		DBG(3, "No device\n");
		return -ENODEV;
	}

	if (dev->driver) {
		DBG(3, "Already bound to %s\n", driver->driver.name);
		return -EBUSY;
	}

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
	retval = device_add(&dev->gadget.dev);

	if (retval) { /* TODO */
		DBG(3, "target device_add failed, error %d\n", retval);
		return retval;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		DBG(3, "%s: bind to driver %s --> error %d\n",
				dev->gadget.name, driver->driver.name, retval);
		device_del(&dev->gadget.dev);

		dev->driver = 0;
		dev->gadget.dev.driver = 0;
		return retval;
	}

	dev_info(&dev->gadget.dev, "Registered gadget driver '%s'\n",
							   driver->driver.name);
	s3c_otg_enable(dev);

	enable_irq(IRQ_OTG);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

/*
  Unregister entry point for the peripheral controller driver.
*/
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c_udc *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;

	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);

	dev->driver = 0;
	s3c_otg_stop_activity(dev, driver);

	spin_unlock_irqrestore(&dev->lock, flags);

	if (driver->unbind)
		driver->unbind(&dev->gadget);

	device_del(&dev->gadget.dev);

	disable_irq(IRQ_OTG);

	dev_info(&dev->gadget.dev, "Unregistered gadget driver '%s'\n",
							   driver->driver.name);

	s3c_otg_disable(dev);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*
 * 	device-scoped parts of the api to the usb controller hardware
 */
static int s3c_otg_get_frame(struct usb_gadget *gadget)
{
	u32 frame = s3c_otg_readl(the_controller, S3C_UDC_OTG_DSTS);
	return FRAME_CNT(frame);
}

static int s3c_otg_wakeup(struct usb_gadget *gadget)
{
	return -EOPNOTSUPP;
}

static int s3c_otg_set_selfpowered(
		struct usb_gadget *gadget, int is_selfpowered)
{
	return -EOPNOTSUPP;
}

static int s3c_otg_pullup(struct usb_gadget *gadget, int is_on)
{
	return -EOPNOTSUPP;
}

static int s3c_otg_vbus_session(struct usb_gadget *gadget, int is_active)
{
	return -EOPNOTSUPP;
}

static int s3c_otg_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	return -EOPNOTSUPP;
}

static const struct usb_gadget_ops s3c_udc_ops = {
	.get_frame = s3c_otg_get_frame,
	.wakeup = s3c_otg_wakeup,
	.set_selfpowered = s3c_otg_set_selfpowered,
	.vbus_session = s3c_otg_vbus_session,
	.vbus_draw = s3c_otg_vbus_draw,
	.pullup = s3c_otg_pullup,
};

static void nop_release(struct device *dev)
{
	DBG(2, "%s\n", dev->bus_id);
}

static struct s3c_udc memory = {
	.usb_address = 0,
	.gadget = {
		   .ops = &s3c_udc_ops,
		   .ep0 = &memory.ep[0].ep,
		   .name = driver_name,
		   .dev = {
			   .bus_id = "gadget",
			   .release = nop_release,
			   },
		   },
	.ep[0] = {
		  .ep = {
			 .name = ep0name,
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP0_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = 0,
		  .bmAttributes = 0,

		  .ep_type = ep_control,
		  .fifo = (u32) S3C_UDC_OTG_EP0_FIFO,
		  },
	.ep[1] = {
		  .ep = {
			 .name = "ep1-bulk",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = 1,
		  .bmAttributes = USB_ENDPOINT_XFER_BULK,

		  .ep_type = ep_bulk_out,
		  .fifo = (u32) S3C_UDC_OTG_EP1_FIFO,
		  },
	.ep[2] = {
		  .ep = {
			 .name = "ep2-bulk",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 2,
		  .bmAttributes = USB_ENDPOINT_XFER_BULK,

		  .ep_type = ep_bulk_in,
		  .fifo = (u32) S3C_UDC_OTG_EP2_FIFO,
		  },

	.ep[3] = {
		  .ep = {
			 .name = "ep3-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 3,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP3_FIFO,
		  },
	.ep[4] = {
		  .ep = {
			 .name = "ep4-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 4,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP4_FIFO,
		  },
	.ep[5] = {
		  .ep = {
			 .name = "ep5-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 5,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP5_FIFO,
		  },
	.ep[6] = {
		  .ep = {
			 .name = "ep6-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 6,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP6_FIFO,
		  },
	.ep[7] = {
		  .ep = {
			 .name = "ep7-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 7,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP7_FIFO,
		  },
	.ep[8] = {
		  .ep = {
			 .name = "ep8-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 8,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP8_FIFO,
		  },
};

static struct clk *otg_clock;

/*
 * binds to the platform device
 */
static int s3c_otg_probe(struct platform_device *pdev)
{
	struct s3c_udc *dev = &memory;
	struct s3c_plat_otg_data *pdata = pdev->dev.platform_data;
	int retval;

	dev->reg_base = ioremap(pdev->resource[0].start,
			pdev->resource[0].end - pdev->resource[0].start);
	if (dev->reg_base == NULL) {
		dev_err(&pdev->dev, "Unable to map USB OTG physical regs\n");
		return -ENOMEM;
	}


	DBG(2, "%p\n", pdev);

	spin_lock_init(&dev->lock);
	dev->dev = pdev;

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = &pdev->dev;

	dev->gadget.is_dualspeed = 1;
	dev->gadget.is_otg = 0;
	dev->gadget.is_a_peripheral = 0;
	dev->gadget.b_hnp_enable = 0;
	dev->gadget.a_hnp_support = 0;
	dev->gadget.a_alt_hnp_support = 0;

	dev->phyclk = pdata->phyclk;

	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	otg_clock = clk_get(&pdev->dev, "otg");
	if (otg_clock == NULL) {
		DBG(3, "failed to find otg clock source\n");
		return -ENOENT;
	}
	clk_enable(otg_clock);

	s3c_otg_reinit(dev);

	local_irq_disable();

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(pdev->resource[1].start, s3c_otg_irq,
			IRQF_DISABLED, driver_name, dev);

	if (retval != 0) {
		DBG(3, "%s: can't get irq %i - %d\n",
				driver_name, IRQ_OTG, retval);
		return -EBUSY;
	}

	disable_irq(IRQ_OTG);
	local_irq_enable();
	create_proc_files();

	return retval;
}

static int s3c_otg_remove(struct platform_device *pdev)
{
	struct s3c_udc *dev = platform_get_drvdata(pdev);

	if (otg_clock != NULL) {
		clk_disable(otg_clock);
		clk_put(otg_clock);
		otg_clock = NULL;
	}

	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	free_irq(IRQ_OTG, dev);

	platform_set_drvdata(pdev, 0);

	the_controller = 0;

	if (dev->reg_base)
		iounmap(dev->reg_base);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_otg_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s3c_udc *dev = the_controller;

	if (dev->driver) {
		disable_irq(IRQ_OTG);
		s3c_otg_disable(dev);
		clk_disable(otg_clock);
	}

	return 0;
}

static int s3c_otg_resume(struct platform_device *pdev)
{
	struct s3c_udc *dev = the_controller;

	if (dev->driver) {
		clk_enable(otg_clock);
		s3c_otg_enable(dev);
		s3c_otg_reinit(dev);
		enable_irq(IRQ_OTG);
	}

	return 0;
}
#else
#define s3c_otg_suspend NULL
#define s3c_otg_resume  NULL
#endif

/*-------------------------------------------------------------------------*/
static struct platform_driver s3c_otg_driver = {
	.probe		= s3c_otg_probe,
	.remove		= s3c_otg_remove,
	.suspend	= s3c_otg_suspend,
	.resume		= s3c_otg_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-otg-usbgadget",
	},
};

static int __init otg_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_otg_driver);
	if (!ret)
		printk(KERN_INFO "Loaded %s version %s %s\n",
				driver_name, DRIVER_VERSION, "(Slave Mode)");

	return ret;
}

static void __exit otg_exit(void)
{
	platform_driver_unregister(&s3c_otg_driver);
	printk(KERN_INFO "Unloaded %s version %s\n",
			driver_name, DRIVER_VERSION);
}

module_init(otg_init);
module_exit(otg_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
