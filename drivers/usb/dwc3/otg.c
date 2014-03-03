/**
 * otg.c - DesignWare USB3 DRD Controller OTG
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: George Cherian <george.cherian@ti.com>,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "core.h"
#include "io.h"
#include "../host/xhci.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DWC3_GSTS_OTG_IP	(1 << 10)
#define DWC3_OSTS_CONID_STS	(1 << 0)
#define DWC3_OSTS_VBUS_VLD	(1 << 1)
#define DWC3_OSTS_BSES_VLD	(1 << 2)

#define DRD_HOST_MODE		0
#define DRD_DEVICE_MODE		1
#define DRD_NOMODE		2

static irqreturn_t dwc3_otg_interrupt(int irq , void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	u32 reg;

	dev_dbg(dwc->dev, "OTG Interrupt\n");

	reg = dwc3_readl(dwc->regs, DWC3_GSTS);
	if (reg & DWC3_GSTS_OTG_IP) {

		reg = dwc3_readl(dwc->regs, DWC3_OEVT);
		dwc->oevt = reg;
		dwc3_writel(dwc->regs, DWC3_OEVT, reg);

		dev_dbg(dwc->dev, "DWC3_GSTS = %x OCFG = %x OCTL = %x\n",
			dwc3_readl(dwc->regs, DWC3_GSTS),
			dwc3_readl(dwc->regs, DWC3_OCFG),
			dwc3_readl(dwc->regs, DWC3_OCTL));
		dev_dbg(dwc->dev, "OEVT = %x OSTS = %x OEVEN = %x ADRL = %x\n",
			dwc3_readl(dwc->regs, DWC3_OEVT),
			dwc3_readl(dwc->regs, DWC3_OSTS),
			dwc3_readl(dwc->regs, DWC3_OEVTEN),
			dwc3_readl(dwc->regs, DWC3_GEVNTADRLO(0)));

		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

int dwc3_otg_role_switch(void *_dwc, int mode)
{

	struct dwc3 *dwc = _dwc;
	u32 irq1;
	struct usb_hcd  *hcd;
	struct xhci_hcd *xhci;

	dev_dbg(dwc->dev, "OTG usb-role switch to %s\n",
		mode ? "device" : "host");

	dev_dbg(dwc->dev, "OEVT = %x OSTS = %x\n",
		dwc3_readl(dwc->regs, DWC3_OEVT),
		dwc3_readl(dwc->regs, DWC3_OSTS));

	/* Check for ConIDSTS == 1 for B-dev*/
	if (mode && dwc->xhci_loaded && !dwc->drd_state) {
		hcd  = platform_get_drvdata(dwc->xhci);
		xhci  = hcd_to_xhci(hcd);

		usb_remove_hcd(xhci->shared_hcd);
		usb_remove_hcd(xhci->main_hcd);
		dwc3_writel(dwc->regs, DWC3_OCTL, 0x48);

		if (dwc->gadget_loaded) {
			/* Put in Peripheral mode */
			dwc3_gadget_start_peripheral(dwc);
		} else {
			dwc3_gadget_init(dwc);
			dwc->gadget_loaded = 1;
		}
		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x01000020);
		dwc->drd_state = 1;

	} else if (!mode && dwc->gadget_loaded && dwc->drd_state) {

		dwc3_gadget_stop_peripheral(dwc);
		if (dwc->xhci_loaded) {
			hcd  = platform_get_drvdata(dwc->xhci);
			xhci  = hcd_to_xhci(hcd);
			irq1 = platform_get_irq(dwc->xhci, 0);
			usb_add_hcd(xhci->main_hcd, irq1, IRQF_SHARED);
			usb_add_hcd(xhci->shared_hcd, irq1, IRQF_SHARED);
		} else {
			dwc3_host_init(dwc);
			dwc->xhci_loaded = 1;
		}
		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x01090000);
		/* OCFG.DisPrtPwrCutoff =1*/
		dwc3_writel(dwc->regs, DWC3_OCFG, 0x20);
		/*OCTL.PrtPwrCtl = 1*/
		dwc3_writel(dwc->regs, DWC3_OCTL, 0x20);
		dwc->drd_state = 0;
	}
	/* drive vbus always ON */
	gpio_request(dwc->gpio, NULL);
	gpio_set_value(dwc->gpio, 1);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_otg_thread_interrupt(int irq, void *_dwc)
{

	struct dwc3 *dwc = _dwc;
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_OSTS);
	dev_dbg(dwc->dev, "OTG thread interrupt DRD(%s) OEVT = %x\n",
		dwc->drd_state ? "device" : "host", reg);

	/* Check for ConIDSTS == 1 for B-dev*/
	if ((reg & DWC3_OSTS_CONID_STS))
		dwc3_otg_role_switch(_dwc, 1);
	else if (!(reg & DWC3_OSTS_CONID_STS))
		dwc3_otg_role_switch(_dwc, 0);

	return IRQ_HANDLED;
}

void dwc3_otg_enable_event(struct dwc3 *dwc)
{
	unsigned int otgevten;

	dev_dbg(dwc->dev, "OTG EVENT ENABLE\n");
	dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x1ff0f00);
	otgevten = dwc3_readl(dwc->regs, DWC3_OEVTEN);
	dev_dbg(dwc->dev, "OTG EVENT ENABLE %x\n", otgevten);
}

int dwc3_otg_init(struct dwc3 *dwc)
{
	int reg, ret, irq;

	dev_dbg(dwc->dev, "OTG INIT\n");
	reg = dwc3_readl(dwc->regs, DWC3_OEVT);
	dwc3_writel(dwc->regs, DWC3_OEVT, reg);

	dev_dbg(dwc->dev, "OEVT = %x OCFG = %x OCTL = %x OEVTEN = %x\n",
		reg, dwc3_readl(dwc->regs, DWC3_OCFG),
		dwc3_readl(dwc->regs, DWC3_OCTL),
		dwc3_readl(dwc->regs, DWC3_OEVTEN));

	/* clear all OTG events */
	dwc3_writel(dwc->regs, DWC3_OEVT, 0xFFFF);
	/* Get the interrupt and install the handler */
	irq = platform_get_irq(to_platform_device(dwc->dev), 1);
	ret = devm_request_threaded_irq(dwc->dev, irq, dwc3_otg_interrupt,
		dwc3_otg_thread_interrupt, IRQF_SHARED, "dwc3-otg", dwc);
	/* Put in Peripheral mode */
	dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x0);
	/* Enable Con ID sts CHG evt */
	dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x01000000);
	/* Enable OCTL peripheral mode */
	dwc3_writel(dwc->regs, DWC3_OCTL, 0x40);
	reg = dwc3_readl(dwc->regs, DWC3_OSTS);

	/* Check for ConIDSTS == 1 for B-dev*/
	if ((reg & DWC3_OSTS_CONID_STS)) {

		dev_dbg(dwc->dev, "Gadget  init\n");

		dwc3_writel(dwc->regs, DWC3_OCFG, 0x8);
		/* Put in Peripheral mode */
		dwc3_writel(dwc->regs, DWC3_OCTL, 0x48);
		/* Enable OEVTEN.OTGBDevSesVldDetEvntEn */
		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x01000020);
		dwc3_gadget_init(dwc);
		dwc->gadget_loaded = 1;
		/* OCTL.SesReq = 1*/
		dwc3_writel(dwc->regs, DWC3_OCTL, 0x48);
		dwc->drd_state = 1;

	} else if (!(reg & DWC3_OSTS_CONID_STS)) {
		dev_dbg(dwc->dev, "Host  init\n");
		/* OCFG.DisPrtPwrCutoff =1*/
		dwc3_writel(dwc->regs, DWC3_OCFG, 0x20);
		dwc3_gadget_init(dwc);
		dwc->gadget_loaded = 1;
		dwc3_host_init(dwc);
		dwc->xhci_loaded = 1;

		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0x01090000);
		/*OCTL.PrtPwrCtl = 1*/
		dwc3_writel(dwc->regs, DWC3_OCTL, 0x20);
		dwc->drd_state = 0;
	}

	return 0;
}
