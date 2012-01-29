/*
 * OMAP4 system control module driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
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

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <plat/omap_device.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <plat/scm.h>
#include <linux/mfd/omap4_scm.h>

static struct scm *scm_ptr;

u32 omap4plus_scm_readl(struct scm *scm_ptr, u32 reg)
{
	return __raw_readl(scm_ptr->base + reg);
}
EXPORT_SYMBOL_GPL(omap4plus_scm_readl);

void omap4plus_scm_writel(struct scm *scm_ptr, u32 val, u32 reg)
{
	__raw_writel(val, scm_ptr->base + reg);
}
EXPORT_SYMBOL_GPL(omap4plus_scm_writel);

/**
 * omap_get_scm_dev - returns the scm device pinter
 *
 * The modules which has to use SCM API's should call this API to get the scm
 * device pointer.
 */
struct device *omap_get_scm_dev(void)
{
	return scm_ptr->dev;
}
EXPORT_SYMBOL_GPL(omap_get_scm_dev);

/**
 * omap4plus_scm_phy_power - power on/off the phy using control module register
 * @dev: struct device *
 * @on: 0 or 1, based on powering on or off the PHY
 *
 * omap_usb2 can call this API to power on or off the PHY.
 */
void omap4plus_scm_phy_power(struct device *dev, int on)
{
	struct scm *scm_ptr	= dev_get_drvdata(dev);

	if (on) {
		if (omap4plus_scm_readl(scm_ptr, CONTROL_DEV_CONF) & PHY_PD) {
			omap4plus_scm_writel(scm_ptr, ~PHY_PD,
							CONTROL_DEV_CONF);
			mdelay(200);
		}
	} else {
		omap4plus_scm_writel(scm_ptr, PHY_PD, CONTROL_DEV_CONF);
	}
}
EXPORT_SYMBOL_GPL(omap4plus_scm_phy_power);

/**
 * omap4plus_scm_usb_host_mode - set AVALID, VBUSVALID and ID pin in grounded
 * @dev: struct device *
 *
 * On detection of a device (ID pin is grounded), OMAP_USB2 calls this API
 * to set AVALID, VBUSVALID and ID pin is grounded.
 */
void omap4plus_scm_usb_host_mode(struct device *dev)
{
	struct scm *scm_ptr	= dev_get_drvdata(dev);
	int val;

	val = AVALID | VBUSVALID;

	omap4plus_scm_writel(scm_ptr, val, CONTROL_USBOTGHS_CONTROL);
}
EXPORT_SYMBOL_GPL(omap4plus_scm_usb_host_mode);

/**
 * omap4plus_scm_usb_device_mode - set AVALID, VBUSVALID and ID pin in high
 * impedance
 * @dev: struct device *
 *
 * When OMAP is connected to a host (OMAP in device mode), OMAP_USB2 calls
 * this API to set AVALID, VBUSVALID and ID pin in high impedance.
 */
void omap4plus_scm_usb_device_mode(struct device *dev)
{
	struct scm *scm_ptr	= dev_get_drvdata(dev);
	int val;

	val = IDDIG | AVALID | VBUSVALID;

	omap4plus_scm_writel(scm_ptr, val, CONTROL_USBOTGHS_CONTROL);
}
EXPORT_SYMBOL_GPL(omap4plus_scm_usb_device_mode);

/**
 * omap4plus_scm_usb_set_sessionend - Enable SESSIONEND and IDIG to high
 * impedance
 * @dev: struct device *
 *
 * OMAP_USB2 calls this API, if OMAP is not connected to anything (neither act
 * as host or device).
 */
void omap4plus_scm_usb_set_sessionend(struct device *dev)
{
	struct scm *scm_ptr	= dev_get_drvdata(dev);
	int val;

	val = SESSEND | IDDIG;

	omap4plus_scm_writel(scm_ptr, val, CONTROL_USBOTGHS_CONTROL);
}
EXPORT_SYMBOL_GPL(omap4plus_scm_usb_set_sessionend);

/**
 * omap5_scm_usb3_phy_power - power on/off the serializer using control module
 * @dev: struct device *
 * @on: 0 or 1, based on powering on or off the PHY
 *
 * omap_usb3 can call this API to power on or off the PHY.
 */
void omap5_scm_usb3_phy_power(struct device *dev, int on)
{
	u32		val;
	unsigned long	rate;
	struct clk	*sys_clk;
	struct scm	*scm_ptr = dev_get_drvdata(dev);

	sys_clk	= clk_get(NULL, "sys_clkin_ck");
	if (IS_ERR(sys_clk)) {
		pr_err("unable to get sys_clkin_ck\n");
		return;
	}

	rate = clk_get_rate(sys_clk);
	rate = rate/1000000;

	val = omap4plus_scm_readl(scm_ptr, CONTROL_PHY_POWER_USB);

	if (on) {
		val &= ~(USB_PWRCTL_CLK_CMD_MASK | USB_PWRCTL_CLK_FREQ_MASK);
		val |= USB3_PHY_TX_RX_POWERON << USB_PWRCTL_CLK_CMD_SHIFT;
		val |= rate << USB_PWRCTL_CLK_FREQ_SHIFT;
	} else {
		val &= ~USB_PWRCTL_CLK_CMD_MASK;
		val |= USB3_PHY_TX_RX_POWEROFF << USB_PWRCTL_CLK_CMD_SHIFT;
	}

	omap4plus_scm_writel(scm_ptr, val, CONTROL_PHY_POWER_USB);
}
EXPORT_SYMBOL_GPL(omap5_scm_usb3_phy_power);

static int __devinit omap4plus_scm_probe(struct platform_device *pdev)
{
	struct omap4plus_scm_pdata *pdata = pdev->dev.platform_data;
	struct resource *mem;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -EINVAL;
	}

		scm_ptr = kzalloc(sizeof(*scm_ptr), GFP_KERNEL);
	if (!scm_ptr) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	scm_ptr->cnt = pdata->cnt;

	mutex_init(&scm_ptr->scm_mutex);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource\n");
			ret = -ENOMEM;
		goto get_res_err;
	}

	scm_ptr->irq = platform_get_irq_byname(pdev, "thermal_alert");
	if (scm_ptr->irq < 0) {
		dev_err(&pdev->dev, "get_irq_byname failed\n");
		ret = scm_ptr->irq;
		goto get_res_err;
	}

	scm_ptr->base = ioremap(mem->start, resource_size(mem));
	if (!scm_ptr->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto get_res_err;
	}

	scm_ptr->rev = pdata->rev;
	scm_ptr->accurate = pdata->accurate;

	scm_ptr->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, scm_ptr);

	return ret;

get_res_err:
	mutex_destroy(&scm_ptr->scm_mutex);
	kfree(scm_ptr);

	return ret;
}

static int __devexit omap4plus_scm_remove(struct platform_device *pdev)
{
	struct scm *scm_ptr = platform_get_drvdata(pdev);

	free_irq(scm_ptr->irq, scm_ptr);
	clk_put(scm_ptr->fclock);
	clk_put(scm_ptr->div_clk);
	iounmap(scm_ptr->base);
	dev_set_drvdata(&pdev->dev, NULL);
	mutex_destroy(&scm_ptr->scm_mutex);
	kfree(scm_ptr);

	return 0;
}

static struct platform_driver omap4plus_scm_driver = {
	.probe = omap4plus_scm_probe,
	.remove = omap4plus_scm_remove,
	.driver = {
			.name = "omap4plus_scm",
		   },
};

static int __init omap4plus_scm_init(void)
{
	return platform_driver_register(&omap4plus_scm_driver);
}

arch_initcall(omap4plus_scm_init);

static void __exit omap4plus_scm_exit(void)
{
	platform_driver_unregister(&omap4plus_scm_driver);
}

module_exit(omap4plus_scm_exit);

MODULE_DESCRIPTION("OMAP4 plus system control module Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
