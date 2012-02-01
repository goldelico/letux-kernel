/*
 * omap-ocp2scp.c - transform ocp interface protocol to scp protocol
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
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/omap_ocp2scp.h>
#include <linux/usb/omap_usb.h>

static LIST_HEAD(ocp2scp_list);
static DEFINE_SPINLOCK(ocp2scp_lock);

static inline u32 omap_ocp2scp_readl(const void __iomem *addr, unsigned offset)
{
	return __raw_readl(addr + offset);
}

static inline void omap_ocp2scp_writel(void __iomem *addr, unsigned offset,
								u32 data)
{
	__raw_writel(data, addr + offset);
}

/**
 * omap_ocp2scp_setsync - program SYNC1 and SYNC2 timing values
 * @id: ocp2scp id
 * @val: SYNC timing values
 *
 * user software must configure OCP2SCP_TIMING register settings before a
 * software reset.
 * TODO: TRM states the module has to be softreset after the timing value has
 * been changed.
 */
int omap_ocp2scp_setsync(int id, u8 val)
{
	u32			reg;
	struct omap_ocp2scp	*ocp2scp;
	unsigned long		flags;

	spin_lock_irqsave(&ocp2scp_lock, flags);

	list_for_each_entry(ocp2scp, &ocp2scp_list, head) {
		if (ocp2scp->id == id) {
			reg = omap_ocp2scp_readl(ocp2scp->base,
							OCP2SCP_TIMING);
			reg &= ~(OCP2SCP_SYNC1_MASK | OCP2SCP_SYNC2_MASK);
			reg |= val;
			omap_ocp2scp_writel(ocp2scp->base, OCP2SCP_TIMING,
									reg);
			spin_unlock_irqrestore(&ocp2scp_lock, flags);
			return 0;
		}
	}

	spin_unlock_irqrestore(&ocp2scp_lock, flags);

	return -EINVAL;
}
EXPORT_SYMBOL(omap_ocp2scp_setsync);

/**
 * _count_resources - count for the number of resources
 * @res: struct resource *
 *
 * Count and return the number of resources populated for the device that is
 * connected to ocp2scp.
 */
static int _count_resources(struct resource *res)
{
	int cnt	= 0;

	while (res->start != res->end) {
		cnt++;
		res++;
	}

	return cnt;
}

static int __devinit omap_ocp2scp_probe(struct platform_device *pdev)
{
	int					ret = 0, i;
	int					res_cnt;
	unsigned long				flags;
	struct omap_ocp2scp			*ocp2scp;
	struct resource				*res;
	struct platform_device			*omap_usb;
	struct omap_ocp2scp_platform_data	*pdata;
	struct omap_usb_platform_data		*usb_pdata;
	struct omap_ocp2scp_dev			*dev;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "OCP2SCP device initialized without"
							"platform data\n");
		return -EINVAL;
	}

	ocp2scp = kzalloc(sizeof *ocp2scp, GFP_KERNEL);
	if (!ocp2scp) {
		dev_err(&pdev->dev, "unable to allocate memory for ocp2scp\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "ocp2scp get memory resource failed\n");
		ret = -ENODEV;
		goto err_base;
	}

	ocp2scp->base = ioremap(res->start, resource_size(res));
	if (!ocp2scp->base) {
		dev_err(&pdev->dev, "ocp2scp ioremap failed\n");
		ret = -ENOMEM;
		goto err_base;
	}

	for (i = 0, dev = *pdata->devices; i < pdata->dev_cnt; i++, dev++) {
		res_cnt = _count_resources(dev->res);
		if (dev->dev_type == DEV_TYPE_USB2PHY) {
			omap_usb = platform_device_alloc("omap-usb2", -1);
			if (!omap_usb) {
				dev_err(&pdev->dev, "failed to allocate omap"
							" usb2 phy device\n");
				ret = PTR_ERR(omap_usb);
				goto err_io;
			}

			ret = platform_device_add_resources(omap_usb,
							dev->res, res_cnt);

			if (ret) {
				dev_err(&pdev->dev, "failed to add resources"
						" to usb2 phy device\n");
				goto err_io;
			}

			usb_pdata = kzalloc(sizeof *usb_pdata, GFP_KERNEL);
			if (!usb_pdata) {
				dev_err(&pdev->dev, "unable to allocate memory"
						"for usb platform data\n");
				return -ENOMEM;
			}

			usb_pdata->rev_id = dev->rev_id;

			ret = platform_device_add_data(omap_usb, usb_pdata,
							sizeof *usb_pdata);
			if (ret) {
				dev_err(&pdev->dev, "failed to add pdata"
						" to usb2 phy device\n");
				kfree(usb_pdata);
				goto err_io;
			}

			omap_usb->dev.parent		= &pdev->dev;
			ocp2scp->omap_usb2		= omap_usb;

			ret = platform_device_add(omap_usb);
			if (ret) {
				dev_err(&pdev->dev, "failed to register omap"
							"usb2 phy device\n");
				kfree(usb_pdata);
				goto err_io;
			}

			/* optional clock used in OMAP4 */
			ocp2scp->clk = clk_get(&pdev->dev,
						"ocp2scp_usb_phy_phy_48m");
		}

		if (dev->dev_type == DEV_TYPE_USB3PHY) {
			omap_usb = platform_device_alloc("omap-usb3", -1);
			if (!omap_usb) {
				dev_err(&pdev->dev, "failed to allocate omap"
							" usb3 phy device\n");
				ret = PTR_ERR(omap_usb);
				goto err_io;
			}

			ret = platform_device_add_resources(omap_usb,
							dev->res, res_cnt);

			if (ret) {
				dev_err(&pdev->dev, "failed to add resources"
						" to usb3 phy device\n");
				goto err_io;
			}

			omap_usb->dev.parent		= &pdev->dev;
			ocp2scp->omap_usb3		= omap_usb;

			ret = platform_device_add(omap_usb);
			if (ret) {
				dev_err(&pdev->dev, "failed to register omap"
							"usb3 phy device\n");
				goto err_io;
			}
		}
	}

	ocp2scp->dev			= &pdev->dev;
	ocp2scp->id			= pdev->id;

	spin_lock_irqsave(&ocp2scp_lock, flags);
	list_add_tail(&ocp2scp->head, &ocp2scp_list);
	spin_unlock_irqrestore(&ocp2scp_lock, flags);

	platform_set_drvdata(pdev, ocp2scp);

	pm_runtime_enable(ocp2scp->dev);

	return 0;

err_io:
	iounmap(ocp2scp->base);

err_base:
	kfree(ocp2scp);

	return ret;
}

static int __devexit omap_ocp2scp_remove(struct platform_device *pdev)
{
	struct omap_ocp2scp	*ocp2scp = platform_get_drvdata(pdev);
	unsigned long		flags;

	spin_lock_irqsave(&ocp2scp_lock, flags);
	list_del(&ocp2scp->head);
	spin_unlock_irqrestore(&ocp2scp_lock, flags);

	if (ocp2scp->omap_usb3)
		platform_device_put(ocp2scp->omap_usb3);

	if (ocp2scp->omap_usb2) {
		clk_put(ocp2scp->clk);
		platform_device_put(ocp2scp->omap_usb2);
	}

	platform_set_drvdata(pdev, NULL);

	iounmap(ocp2scp->base);
	kfree(ocp2scp);

	return 0;
}

#ifdef CONFIG_PM
static int ocp2scp_runtime_suspend(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct omap_ocp2scp	*ocp2scp = platform_get_drvdata(pdev);

	if (ocp2scp->clk)
		clk_disable(ocp2scp->clk);

	return 0;
}

static int ocp2scp_runtime_resume(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct omap_ocp2scp	*ocp2scp = platform_get_drvdata(pdev);

	if (ocp2scp->clk)
		clk_enable(ocp2scp->clk);

	return 0;
}

static const struct dev_pm_ops ocp2scp_pm_ops = {
	.runtime_suspend	= ocp2scp_runtime_suspend,
	.runtime_resume		= ocp2scp_runtime_resume,
};

#define DEV_PM_OPS	(&ocp2scp_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver omap_ocp2scp_driver = {
	.probe		= omap_ocp2scp_probe,
	.remove		= __devexit_p(omap_ocp2scp_remove),
	.driver		= {
		.name	= "omap-ocp2scp",
		.owner	= THIS_MODULE,
		.pm	= DEV_PM_OPS,
	},
};

static int __init omap_ocp2scp_init(void)
{
	return platform_driver_register(&omap_ocp2scp_driver);
}
arch_initcall(omap_ocp2scp_init);

static void __exit omap_ocp2scp_exit(void)
{
	platform_driver_unregister(&omap_ocp2scp_driver);
}
module_exit(omap_ocp2scp_exit);

MODULE_ALIAS("platform: omap-ocp2scp");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_DESCRIPTION("OMAP OCP2SCP DRIVER");
MODULE_LICENSE("GPL");
