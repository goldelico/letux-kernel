/*
 * omap-usb2.c - USB PHY, talking to musb controller in OMAP.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/omap_usb.h>
#include <linux/usb/phy_companion.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/mfd/omap4_scm.h>

/**
 * omap_usb_update_state - update the phy state based on the event status
 * @phy - struct omap_usb *
 * @status - enum usb_phy_events
 *
 * On an interrupt event, omap_usb_update state will be called to update the
 * state of the phy. These phy state can be used by the core controller.
 */
static void omap_usb_update_state(struct omap_usb *phy,
						enum usb_phy_events status)
{
	struct usb_otg		*otg = (&phy->phy)->otg;

	switch (status) {
	case USB_EVENT_ID:
		otg->default_a = true;
		phy->phy.state = OTG_STATE_A_IDLE;
		phy->phy.last_event = status;
		break;
	case USB_EVENT_VBUS:
		otg->default_a = false;
		phy->phy.state = OTG_STATE_B_IDLE;
		phy->phy.last_event = status;
		break;
	case USB_EVENT_NONE:
		phy->phy.last_event = status;
		break;
	default:
		break;
	}
}

/**
 * omap_usb2_power - mailbox communication between OMAP and musb
 * @phy - struct omap_usb *
 * @ID - if the controller should be in host mode or device mode
 * @on - on or off staus
 *
 * In OMAP4, to pass the detection of the attached device to the MUSB,
 * system control registers should be used. This function calls scm
 * driver API's to pass the detected device info to MUSB.
 */
static int omap_usb2_power(struct omap_usb *phy, int ID, int on)
{
	if (on) {
		if (ID)
			omap4plus_scm_usb_host_mode(phy->scm_dev);
		else
			omap4plus_scm_usb_device_mode(phy->scm_dev);
	} else {
		omap4plus_scm_usb_set_sessionend(phy->scm_dev);
	}

	return 0;
}

/**
 * omap_usb_irq - usb phy interrupt handler
 * @x - the phy, this driver has created
 * @status - enum usb_phy_events
 *
 * The interrupt event that the phy comparator (e.g. twl6030-usb.c) obtained
 * will be propagated to omap-usb using this function. This function is
 * responsible for updating the state of the PHY and sending the notifcation
 * to the contoller.
 *
 * For use by phy comparator.
 */
static int omap_usb_irq(struct notifier_block *nb, unsigned long status,
								void *unused)
{
	struct omap_usb		*phy = container_of(nb, struct omap_usb, nb);
	struct usb_otg		*otg = (&phy->phy)->otg;

	omap_usb_update_state(phy, status);

	atomic_notifier_call_chain(&phy->phy.notifier, status, otg->gadget);

	return 0;
}

/**
 * omap_usb2_init - links the comparator present in the sytem with this phy
 * @x - the phy, this driver has created
 *
 * Gets the comparator that is already created in the system passing it the
 * notifier function to receive any interrupts.
 * In order to handle any events that might have occured before loading this
 * driver, omap_usb2_power is been called.
 *
 * For use by usb controller to initialize phy (usb_phy_init())
 */
static int omap_usb2_init(struct usb_phy *x)
{
	struct omap_usb	*phy = phy_to_omapusb(x);

	phy->nb.notifier_call = omap_usb_irq;

	if (phy->rev_id == 1) {
		phy->comparator		= get_phy_twl6030_companion(&phy->nb);
		if (!phy->comparator) {
			dev_err(x->dev, "unable to get comparator"
							" for usb2 phy\n");
			return -ENODEV;
		}

		if (phy->comparator->linkstat == USB_EVENT_ID)
			omap_usb2_power(phy, 1, 1);
		else if (phy->comparator->linkstat == USB_EVENT_VBUS)
			omap_usb2_power(phy, 0, 1);

		omap_usb_update_state(phy, phy->comparator->linkstat);
	} else {
		phy->comparator		= get_phy_palmas_companion(&phy->nb);
		if (!phy->comparator) {
			dev_err(x->dev, "unable to get comparator for"
								" usb2 phy\n");
			return -ENODEV;
		}
	}

	if (phy->comparator->linkstat != USB_EVENT_NONE)
		omap_usb_irq(&phy->nb, phy->comparator->linkstat, NULL);

	return 0;
}

static int omap_usb_set_vbus(struct usb_otg *otg, bool enabled)
{
	struct omap_usb *phy = phy_to_omapusb(otg->phy);

	if (phy->comparator)
		return phy->comparator->set_vbus(phy->comparator, enabled);
	else
		return -ENODEV;
}

static int omap_usb_start_srp(struct usb_otg *otg)
{
	struct omap_usb *phy = phy_to_omapusb(otg->phy);
	if (phy->comparator)
		return phy->comparator->start_srp(phy->comparator);
	else
		return -ENODEV;
}

static int omap_usb_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct usb_phy	*phy = otg->phy;

	otg->host = host;
	if (!host)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int omap_usb_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
	struct usb_phy	*phy = otg->phy;

	otg->gadget = gadget;
	if (!gadget)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static void omap_usb2_shutdown(struct usb_phy *x)
{
	struct omap_usb	*phy = phy_to_omapusb(x);

	omap_usb2_power(phy, 0, 0);
}

static int omap_usb2_suspend(struct usb_phy *x, int suspend)
{
	struct omap_usb *phy = phy_to_omapusb(x);

	if (suspend && !phy->is_suspended) {
		omap4plus_scm_phy_power(phy->scm_dev, 0);

		pm_runtime_put_sync(phy->dev);
		clk_disable(phy->optclk);
		clk_disable(phy->wkupclk);

		phy->is_suspended = 1;
		if (phy->rev_id == 1 && phy->phy.last_event == USB_EVENT_NONE)
			omap_usb2_power(phy, 0, 0);
	} else if (!suspend && phy->is_suspended) {
		clk_enable(phy->optclk);
		clk_enable(phy->wkupclk);
		pm_runtime_get_sync(phy->dev);

		omap4plus_scm_phy_power(phy->scm_dev, 1);

		phy->is_suspended = 0;
		if (phy->rev_id == 1) {
			if (phy->phy.last_event == USB_EVENT_ID)
				omap_usb2_power(phy, 1, 1);
			else if (phy->phy.last_event == USB_EVENT_VBUS)
				omap_usb2_power(phy, 0, 1);
		}
	}

	return 0;
}

static int __devinit omap_usb2_probe(struct platform_device *pdev)
{
	struct omap_usb			*phy;
	struct usb_otg			*otg;
	struct omap_usb_platform_data	*pdata;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "OMAP USB2 device initialized without"
							"platform data\n");
		return -EINVAL;
	}

	phy = kzalloc(sizeof *phy, GFP_KERNEL);
	if (!phy) {
		dev_err(&pdev->dev, "unable to allocate memory for OMAP"
							 "USB2 PHY\n");
		return -ENOMEM;
	}

	otg = kzalloc(sizeof *otg, GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for "
							 "USB OTG\n");
		return -ENOMEM;
	}

	phy->dev		= &pdev->dev;

	phy->phy.dev		= phy->dev;
	phy->phy.label		= "omap-usb2";
	phy->phy.init		= omap_usb2_init;
	phy->phy.shutdown	= omap_usb2_shutdown;
	phy->phy.set_suspend	= omap_usb2_suspend;
	phy->phy.otg		= otg;

	phy->scm_dev		= omap_get_scm_dev();

	phy->is_suspended	= 1;
	omap4plus_scm_phy_power(phy->scm_dev, 0);

	otg->set_host		= omap_usb_set_host;
	otg->set_peripheral	= omap_usb_set_peripheral;
	otg->set_vbus		= omap_usb_set_vbus;
	otg->start_srp		= omap_usb_start_srp;
	otg->phy		= &phy->phy;

	phy->rev_id		= pdata->rev_id;

	phy->wkupclk = clk_get(phy->dev, "usb_phy_cm_clk32k");
	phy->optclk = clk_get(phy->dev, "usb_otg_ss_refclk960m_ck");

	usb_add_phy(&phy->phy, USB_PHY_TYPE_USB2);

	platform_set_drvdata(pdev, phy);

	ATOMIC_INIT_NOTIFIER_HEAD(&phy->phy.notifier);

	pm_runtime_enable(phy->dev);

	return 0;
}

static int __devexit omap_usb2_remove(struct platform_device *pdev)
{
	struct omap_usb	*phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);
	platform_set_drvdata(pdev, NULL);

	kfree(phy);

	return 0;
}

static struct platform_driver omap_usb2_driver = {
	.probe		= omap_usb2_probe,
	.remove		= __devexit_p(omap_usb2_remove),
	.driver		= {
		.name	= "omap-usb2",
		.owner	= THIS_MODULE,
	},
};

static int __init usb2_omap_init(void)
{
	return platform_driver_register(&omap_usb2_driver);
}
arch_initcall(usb2_omap_init);

static void __exit usb2_omap_exit(void)
{
	platform_driver_unregister(&omap_usb2_driver);
}
module_exit(usb2_omap_exit);

MODULE_ALIAS("platform: omap_usb2");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_DESCRIPTION("OMAP USB2 PHY DRIVER");
MODULE_LICENSE("GPL");
