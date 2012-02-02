/*
 * xHCI host controller driver platform Bus Glue.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sebastian Andrzej Siewior < bigeasy @ linutronix . de >
 *
 * A lot of code borrowed from the Linux xHCI driver.
 */

#include <linux/platform_device.h>
#include <linux/slab.h>

#include "xhci.h"

static const char hcd_name[] = "xhci_hcd";

static void xhci_plat_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_BROKEN_MSI;
}

/* called during probe() after chip reset completes */
static int xhci_plat_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, xhci_plat_quirks);
}

static const struct hc_driver xhci_plat_xhci_driver = {
	.description =		hcd_name,
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_plat_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_bus_resume,
};

static int usb_hcd_plat_probe(struct platform_device *dev,
		const struct hc_driver *driver)
{
	struct usb_hcd		*hcd;
	int			retval;
	int			irq;
	struct resource         *res;

	if (usb_disabled())
		return -ENODEV;

	if (!driver)
		return -EINVAL;

	irq = platform_get_irq(dev, 0);
	if (irq < 0)
		return -ENODEV;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	hcd = usb_create_hcd(driver, &dev->dev, dev_name(&dev->dev));
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&dev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto put_hcd;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&dev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto release_mem_region;
	}

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval != 0)
		goto unmap_registers;
	return retval;

unmap_registers:
	iounmap(hcd->regs);
release_mem_region:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
put_hcd:
	usb_put_hcd(hcd);
	return retval;
}

/*
 * We need to register our own PCI probe function (instead of the USB core's
 * function) in order to create a second roothub under xHCI.
 */
static int xhci_plat_probe(struct platform_device *dev)
{
	int retval;
	struct xhci_hcd *xhci;
	const struct hc_driver *driver;
	struct usb_hcd *hcd;
	int irq;

	driver = &xhci_plat_xhci_driver;
	/*
	 * Register the USB 2.0 roothub.
	 * FIXME: USB core must know to register the USB 2.0 roothub first.
	 * This is sort of silly, because we could just set the HCD driver flags
	 * to say USB 2.0, but I'm not sure what the implications would be in
	 * the other parts of the HCD code.
	 */
	retval = usb_hcd_plat_probe(dev, driver);
	if (retval)
		return retval;

	irq = platform_get_irq(dev, 0);

	/* USB 2.0 roothub is stored in the PCI device now. */
	hcd = dev_get_drvdata(&dev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &dev->dev,
				dev_name(&dev->dev), hcd);
	if (!xhci->shared_hcd) {
		retval = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	/*
	 * Set the xHCI pointer before xhci_plat_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	retval = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (retval)
		goto put_usb3_hcd;
	/* Roothub already marked as USB 3.0 speed */
	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	return retval;
}

static int xhci_plat_remove(struct platform_device *dev)
{
	struct xhci_hcd *xhci;

	xhci = hcd_to_xhci(platform_get_drvdata(dev));
	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}

	usb_remove_hcd(xhci->main_hcd);
	iounmap(xhci->main_hcd->regs);
	usb_put_hcd(xhci->main_hcd);
	kfree(xhci);
	return 0;
}

static struct platform_driver usb_xhci_driver = {
	.probe = xhci_plat_probe,
	.remove = xhci_plat_remove,
	.driver = {
		.name = "xhci",
	},
};
MODULE_ALIAS("platform:usb-xhci");

int xhci_register_plat(void)
{
	return platform_driver_register(&usb_xhci_driver);
}

void xhci_unregister_plat(void)
{
	platform_driver_unregister(&usb_xhci_driver);
}
