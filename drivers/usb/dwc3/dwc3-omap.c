/**
 * dwc3-omap.c - OMAP Specific Glue layer
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/platform_data/dwc3-omap.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>

#include "core.h"
#include "io.h"

/*
 * All these registers belong to OMAP's Wrapper around the
 * DesignWare USB3 Core.
 */

#define USBOTGSS_REVISION			0x0000
#define USBOTGSS_SYSCONFIG			0x0010
#define USBOTGSS_IRQ_EOI			0x0020
#define USBOTGSS_IRQSTATUS_RAW_0		0x0024
#define USBOTGSS_IRQSTATUS_0			0x0028
#define USBOTGSS_IRQENABLE_SET_0		0x002c
#define USBOTGSS_IRQENABLE_CLR_0		0x0030
#define USBOTGSS_IRQSTATUS_RAW_1		0x0034
#define USBOTGSS_IRQSTATUS_1			0x0038
#define USBOTGSS_IRQENABLE_SET_1		0x003c
#define USBOTGSS_IRQENABLE_CLR_1		0x0040
#define USBOTGSS_UTMI_OTG_CTRL			0x0080
#define USBOTGSS_UTMI_OTG_STATUS		0x0084
#define USBOTGSS_MMRAM_OFFSET			0x0100
#define USBOTGSS_FLADJ				0x0104
#define USBOTGSS_DEBUG_CFG			0x0108
#define USBOTGSS_DEBUG_DATA			0x010c

/* SYSCONFIG REGISTER */
#define USBOTGSS_SYSCONFIG_DMADISABLE		(1 << 16)

/* IRQ_EOI REGISTER */
#define USBOTGSS_IRQ_EOI_LINE_NUMBER		(1 << 0)

/* IRQS0 BITS */
#define USBOTGSS_IRQO_COREIRQ_ST		(1 << 0)

/* IRQ1 BITS */
#define USBOTGSS_IRQ1_DMADISABLECLR		(1 << 17)
#define USBOTGSS_IRQ1_OEVT			(1 << 16)
#define USBOTGSS_IRQ1_DRVVBUS_RISE		(1 << 13)
#define USBOTGSS_IRQ1_CHRGVBUS_RISE		(1 << 12)
#define USBOTGSS_IRQ1_DISCHRGVBUS_RISE		(1 << 11)
#define USBOTGSS_IRQ1_IDPULLUP_RISE		(1 << 8)
#define USBOTGSS_IRQ1_DRVVBUS_FALL		(1 << 5)
#define USBOTGSS_IRQ1_CHRGVBUS_FALL		(1 << 4)
#define USBOTGSS_IRQ1_DISCHRGVBUS_FALL		(1 << 3)
#define USBOTGSS_IRQ1_IDPULLUP_FALL		(1 << 0)

/* UTMI_OTG_CTRL REGISTER */
#define USBOTGSS_UTMI_OTG_CTRL_DRVVBUS		(1 << 5)
#define USBOTGSS_UTMI_OTG_CTRL_CHRGVBUS		(1 << 4)
#define USBOTGSS_UTMI_OTG_CTRL_DISCHRGVBUS	(1 << 3)
#define USBOTGSS_UTMI_OTG_CTRL_IDPULLUP		(1 << 0)

/* UTMI_OTG_STATUS REGISTER */
#define USBOTGSS_UTMI_OTG_STATUS_SW_MODE	(1 << 31)
#define USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT	(1 << 9)
#define USBOTGSS_UTMI_OTG_STATUS_TXBITSTUFFENABLE (1 << 8)
#define USBOTGSS_UTMI_OTG_STATUS_IDDIG		(1 << 4)
#define USBOTGSS_UTMI_OTG_STATUS_SESSEND	(1 << 3)
#define USBOTGSS_UTMI_OTG_STATUS_SESSVALID	(1 << 2)
#define USBOTGSS_UTMI_OTG_STATUS_VBUSVALID	(1 << 1)

struct dwc3_omap {
	/* device lock */
	struct usb_phy		*usb2_phy;
	struct usb_phy		*usb3_phy;

	spinlock_t		lock;

	struct platform_device	*dwc3;
	struct device		*dev;

	int			irq;
	void __iomem		*base;

	void			*context;
	u32			resource_size;

	u32			dma_status:1;
	struct notifier_block	nb;
};

static irqreturn_t dwc3_omap_interrupt(int irq, void *_omap)
{
	struct dwc3_omap	*omap = _omap;
	u32			reg;

	spin_lock(&omap->lock);

	reg = dwc3_readl(omap->base, USBOTGSS_IRQSTATUS_1);

	if (reg & USBOTGSS_IRQ1_DMADISABLECLR) {
		dev_dbg(omap->dev, "DMA Disable was Cleared\n");
		omap->dma_status = false;
	}

	if (reg & USBOTGSS_IRQ1_OEVT)
		dev_dbg(omap->dev, "OTG Event\n");

	if (reg & USBOTGSS_IRQ1_DRVVBUS_RISE)
		dev_dbg(omap->dev, "DRVVBUS Rise\n");

	if (reg & USBOTGSS_IRQ1_CHRGVBUS_RISE)
		dev_dbg(omap->dev, "CHRGVBUS Rise\n");

	if (reg & USBOTGSS_IRQ1_DISCHRGVBUS_RISE)
		dev_dbg(omap->dev, "DISCHRGVBUS Rise\n");

	if (reg & USBOTGSS_IRQ1_IDPULLUP_RISE)
		dev_dbg(omap->dev, "IDPULLUP Rise\n");

	if (reg & USBOTGSS_IRQ1_DRVVBUS_FALL)
		dev_dbg(omap->dev, "DRVVBUS Fall\n");

	if (reg & USBOTGSS_IRQ1_CHRGVBUS_FALL)
		dev_dbg(omap->dev, "CHRGVBUS Fall\n");

	if (reg & USBOTGSS_IRQ1_DISCHRGVBUS_FALL)
		dev_dbg(omap->dev, "DISCHRGVBUS Fall\n");

	if (reg & USBOTGSS_IRQ1_IDPULLUP_FALL)
		dev_dbg(omap->dev, "IDPULLUP Fall\n");

	dwc3_writel(omap->base, USBOTGSS_IRQSTATUS_1, reg);

	reg = dwc3_readl(omap->base, USBOTGSS_IRQSTATUS_0);
	dwc3_writel(omap->base, USBOTGSS_IRQSTATUS_0, reg);

	spin_unlock(&omap->lock);

	return IRQ_HANDLED;
}

/* blocking notifier support */
static int dwc3_otg_notifications(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	u32			val;
	struct dwc3_omap	*omap = container_of(nb, struct dwc3_omap, nb);

	switch (event) {
	case USB_EVENT_ID:
		dev_dbg(omap->dev, "ID GND\n");

		dwc3_core_late_init(&omap->dwc3->dev);

		pm_runtime_get_sync(omap->dev);
		val = dwc3_readl(omap->base, USBOTGSS_UTMI_OTG_STATUS);
		val &= ~(USBOTGSS_UTMI_OTG_STATUS_IDDIG
				| USBOTGSS_UTMI_OTG_STATUS_VBUSVALID
				| USBOTGSS_UTMI_OTG_STATUS_SESSEND);
		val |= USBOTGSS_UTMI_OTG_STATUS_SESSVALID
				| USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT;
		dwc3_writel(omap->base, USBOTGSS_UTMI_OTG_STATUS, val);
		pm_runtime_put_sync(omap->dev);

		break;

	case USB_EVENT_VBUS:
		dev_dbg(omap->dev, "VBUS Connect\n");

		dwc3_core_late_init(&omap->dwc3->dev);

		pm_runtime_get_sync(omap->dev);
		val = dwc3_readl(omap->base, USBOTGSS_UTMI_OTG_STATUS);
		val &= ~USBOTGSS_UTMI_OTG_STATUS_SESSEND;
		val |= USBOTGSS_UTMI_OTG_STATUS_IDDIG
				| USBOTGSS_UTMI_OTG_STATUS_VBUSVALID
				| USBOTGSS_UTMI_OTG_STATUS_SESSVALID
				| USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT;
		dwc3_writel(omap->base, USBOTGSS_UTMI_OTG_STATUS, val);
		pm_runtime_put_sync(omap->dev);

		break;

	case USB_EVENT_NONE:
		dev_dbg(omap->dev, "VBUS Disconnect\n");

		pm_runtime_get_sync(omap->dev);
		val = dwc3_readl(omap->base, USBOTGSS_UTMI_OTG_STATUS);
		val &= ~(USBOTGSS_UTMI_OTG_STATUS_SESSVALID
				| USBOTGSS_UTMI_OTG_STATUS_VBUSVALID
				| USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT);
		val |= USBOTGSS_UTMI_OTG_STATUS_SESSEND
				| USBOTGSS_UTMI_OTG_STATUS_IDDIG;
		dwc3_writel(omap->base, USBOTGSS_UTMI_OTG_STATUS, val);
		pm_runtime_put_sync(omap->dev);

		/* Give enough time for the core to process disconnect intr */
		msleep(20);
		dwc3_core_shutdown(&omap->dwc3->dev);

		break;
	default:
		dev_dbg(omap->dev, "ID float\n");
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int omap_dwc3_init(struct dwc3_omap *omap)
{
	u32		status = 0;

	omap->usb2_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!omap->usb2_phy) {
		dev_err(omap->dev, "no usb2 phy configured\n");
		return -ENODEV;
	}

	omap->usb3_phy = usb_get_phy(USB_PHY_TYPE_USB3);
	if (!omap->usb3_phy) {
		dev_err(omap->dev, "no usb3 phy configured\n");
		return -ENODEV;
	}

	usb_phy_init(omap->usb2_phy);

	omap->nb.notifier_call = dwc3_otg_notifications;
	status = usb_register_notifier(omap->usb2_phy, &omap->nb);

	if (status)
		dev_dbg(omap->dev, "notification register failed\n");

	return 0;
}

static int omap_dwc3_exit(struct dwc3_omap *omap)
{
	usb_put_phy(omap->usb2_phy);
	usb_put_phy(omap->usb3_phy);

	usb_unregister_notifier(omap->usb2_phy, &omap->nb);

	return 0;
}

static void dwc3_omap_enable_irqs(struct dwc3_omap *omap)
{
	u32	reg;

	reg = USBOTGSS_IRQO_COREIRQ_ST;
	dwc3_writel(omap->base, USBOTGSS_IRQENABLE_SET_0, reg);

	reg = (USBOTGSS_IRQ1_OEVT |
			USBOTGSS_IRQ1_DRVVBUS_RISE |
			USBOTGSS_IRQ1_CHRGVBUS_RISE |
			USBOTGSS_IRQ1_DISCHRGVBUS_RISE |
			USBOTGSS_IRQ1_IDPULLUP_RISE |
			USBOTGSS_IRQ1_DRVVBUS_FALL |
			USBOTGSS_IRQ1_CHRGVBUS_FALL |
			USBOTGSS_IRQ1_DISCHRGVBUS_FALL |
			USBOTGSS_IRQ1_IDPULLUP_FALL);

	dwc3_writel(omap->base, USBOTGSS_IRQENABLE_SET_1, reg);
}

static int __devinit dwc3_omap_probe(struct platform_device *pdev)
{
	struct dwc3_omap_data		*pdata = pdev->dev.platform_data;
	struct dwc3_platform_data	*dwc3_pdata;
	struct device_node		*node = pdev->dev.of_node;

	struct platform_device	*dwc3;
	struct dwc3_omap	*omap;
	struct resource         dwc3_res[2];
	struct resource		*res;

	int			devid;
	int			size;
	int			ret = -ENOMEM;
	int			irq;

	const u32		*utmi_mode;
	u32			reg;

	void __iomem		*base;
	void			*context;

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err0;
	}

	dwc3_pdata = kzalloc(sizeof(*dwc3_pdata), GFP_KERNEL);
	if (!omap) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err1;
	}

	platform_set_drvdata(pdev, omap);

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing IRQ resource\n");
		ret = -EINVAL;
		goto err2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing memory base resource\n");
		ret = -EINVAL;
		goto err2;
	}

	base = ioremap_nocache(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err2;
	}

	devid = dwc3_get_device_id();
	if (devid < 0)
		goto err3;

	dwc3 = platform_device_alloc("dwc3", devid);
	if (!dwc3) {
		dev_err(&pdev->dev, "couldn't allocate dwc3 device\n");
		goto err4;
	}

	context = kzalloc(resource_size(res), GFP_KERNEL);
	if (!context) {
		dev_err(&pdev->dev, "couldn't allocate dwc3 context memory\n");
		goto err5;
	}

	ret = omap_dwc3_init(omap);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize omap platform\n");
		goto err6;
	}

	spin_lock_init(&omap->lock);
	dma_set_coherent_mask(&dwc3->dev, pdev->dev.coherent_dma_mask);

	dwc3->dev.parent = &pdev->dev;
	dwc3->dev.dma_mask = pdev->dev.dma_mask;
	dwc3->dev.dma_parms = pdev->dev.dma_parms;
	omap->resource_size = resource_size(res);
	omap->context	= context;
	omap->dev	= &pdev->dev;
	omap->irq	= irq;
	omap->base	= base;
	omap->dwc3	= dwc3;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	dwc3_pdata->usb2_phy	= omap->usb2_phy;
	dwc3_pdata->usb3_phy	= omap->usb3_phy;

	reg = dwc3_readl(omap->base, USBOTGSS_UTMI_OTG_STATUS);

	utmi_mode = of_get_property(node, "utmi-mode", &size);
	if (utmi_mode && size == sizeof(*utmi_mode)) {
		reg |= *utmi_mode;
	} else {
		if (!pdata) {
			dev_dbg(&pdev->dev, "missing platform data\n");
		} else {
			switch (pdata->utmi_mode) {
			case DWC3_OMAP_UTMI_MODE_SW:
				reg |= USBOTGSS_UTMI_OTG_STATUS_SW_MODE;
				break;
			case DWC3_OMAP_UTMI_MODE_HW:
				reg &= ~USBOTGSS_UTMI_OTG_STATUS_SW_MODE;
				break;
			default:
				dev_dbg(&pdev->dev, "UNKNOWN utmi mode %d\n",
						pdata->utmi_mode);
			}
		}
	}

	dwc3_writel(omap->base, USBOTGSS_UTMI_OTG_STATUS, reg);

	/* check the DMA Status */
	reg = dwc3_readl(omap->base, USBOTGSS_SYSCONFIG);
	omap->dma_status = !!(reg & USBOTGSS_SYSCONFIG_DMADISABLE);

	ret = request_irq(omap->irq, dwc3_omap_interrupt, 0,
			"dwc3-omap", omap);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ #%d --> %d\n",
				omap->irq, ret);
		goto err7;
	}

	/* enable all IRQs */
	reg = USBOTGSS_IRQO_COREIRQ_ST;
	dwc3_writel(omap->base, USBOTGSS_IRQENABLE_SET_0, reg);

	reg = (USBOTGSS_IRQ1_OEVT |
			USBOTGSS_IRQ1_DRVVBUS_RISE |
			USBOTGSS_IRQ1_CHRGVBUS_RISE |
			USBOTGSS_IRQ1_DISCHRGVBUS_RISE |
			USBOTGSS_IRQ1_IDPULLUP_RISE |
			USBOTGSS_IRQ1_DRVVBUS_FALL |
			USBOTGSS_IRQ1_CHRGVBUS_FALL |
			USBOTGSS_IRQ1_DISCHRGVBUS_FALL |
			USBOTGSS_IRQ1_IDPULLUP_FALL);

	dwc3_writel(omap->base, USBOTGSS_IRQENABLE_SET_1, reg);

	pm_runtime_put_sync(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "missing memory base resource for dwc3\n");
		ret = -EINVAL;
		goto err8;
	}

	memset(dwc3_res, 0, sizeof(dwc3_res));

	dwc3_res[0].start	= res->start;
	dwc3_res[0].end		= res->end;
	dwc3_res[0].flags	= res->flags;
	dwc3_res[0].name	= res->name;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing IRQ\n");
		ret = -EINVAL;
		goto err8;
	}

	dwc3_res[1].start	= irq;
	dwc3_res[1].flags	= IORESOURCE_IRQ;
	dwc3_res[1].name	= res->name;

	ret = platform_device_add_resources(dwc3, dwc3_res,
							ARRAY_SIZE(dwc3_res));
	if (ret) {
		dev_err(&pdev->dev, "couldn't add resources to dwc3 device\n");
		goto err8;
	}

	ret = platform_device_add_data(dwc3, dwc3_pdata, sizeof(*dwc3_pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err8;
	}

	ret = platform_device_add(dwc3);
	if (ret) {
		dev_err(&pdev->dev, "failed to register dwc3 device\n");
		goto err8;
	}

	if (omap->usb2_phy->last_event)
		dwc3_otg_notifications(&omap->nb, omap->usb2_phy->last_event,
									NULL);

	return 0;

err8:
	free_irq(omap->irq, omap);

err7:
	omap_dwc3_exit(omap);

err6:
	kfree(omap->context);

err5:
	platform_device_put(dwc3);

err4:
	dwc3_put_device_id(devid);

err3:
	iounmap(base);

err2:
	kfree(omap);

err1:
	kfree(dwc3_pdata);

err0:
	return ret;
}

static int __devexit dwc3_omap_remove(struct platform_device *pdev)
{
	struct dwc3_omap	*omap = platform_get_drvdata(pdev);

	omap_dwc3_exit(omap);

	platform_device_unregister(omap->dwc3);

	dwc3_put_device_id(omap->dwc3->id);
	free_irq(omap->irq, omap);
	iounmap(omap->base);

	kfree(omap->context);
	kfree(omap);

	return 0;
}

static const struct of_device_id of_dwc3_matach[] = {
	{
		"ti,dwc3",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_matach);

#ifdef CONFIG_PM

static int dwc3_omap_runtime_resume(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct dwc3_omap	*omap = platform_get_drvdata(pdev);

	dwc3_omap_enable_irqs(omap);

	return 0;
}

static const struct dev_pm_ops dwc3_omap_pm_ops = {
	SET_RUNTIME_PM_OPS(NULL, dwc3_omap_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_omap_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver dwc3_omap_driver = {
	.probe		= dwc3_omap_probe,
	.remove		= __devexit_p(dwc3_omap_remove),
	.driver		= {
		.name	= "omap-dwc3",
		.of_match_table	= of_dwc3_matach,
		.pm	= DEV_PM_OPS,
	},
};

MODULE_ALIAS("platform:omap-dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 OMAP Glue Layer");

static int __devinit dwc3_omap_init(void)
{
	return platform_driver_register(&dwc3_omap_driver);
}
module_init(dwc3_omap_init);

static void __exit dwc3_omap_exit(void)
{
	platform_driver_unregister(&dwc3_omap_driver);
}
module_exit(dwc3_omap_exit);
