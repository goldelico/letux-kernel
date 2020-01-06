// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2010, Paul Cercueil <paul@crapouillou.net>
 *  JZ4740 Watchdog driver
 *  Copyright (c) 2017 Paul Boddie <paul@boddie.org.uk>
 *  JZ4730 Watchdog driver
 */

#include <linux/mfd/ingenic-tcu.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>

#include <asm/mach-jz4740/timer.h>

#define JZ_WDT_CLOCK_PCLK 0x1
#define JZ_WDT_CLOCK_RTC  0x2
#define JZ_WDT_CLOCK_EXT  0x4

#define JZ_WDT_CLOCK_DIV_1    (0 << TCU_TCSR_PRESCALE_LSB)
#define JZ_WDT_CLOCK_DIV_4    (1 << TCU_TCSR_PRESCALE_LSB)
#define JZ_WDT_CLOCK_DIV_16   (2 << TCU_TCSR_PRESCALE_LSB)
#define JZ_WDT_CLOCK_DIV_64   (3 << TCU_TCSR_PRESCALE_LSB)
#define JZ_WDT_CLOCK_DIV_256  (4 << TCU_TCSR_PRESCALE_LSB)
#define JZ_WDT_CLOCK_DIV_1024 (5 << TCU_TCSR_PRESCALE_LSB)

#define DEFAULT_HEARTBEAT 5
#define MAX_HEARTBEAT     2048

/* The following are used on the jz4730 instead */
#define JZ_REG_WDT_TIMER_STATUS		0x00
#define JZ_REG_WDT_TIMER_COUNTER32	0x04

#define JZ_WDT_TIMER_STATUS_START	BIT(4)

#define JZ_REG_CPM_OCR			0x1c
#define JZ_CPM_OCR_EXT_RTC_CLK		BIT(8)

enum jz4740_wdt_type {
	ID_JZ4730,
	ID_JZ4740,
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int heartbeat = DEFAULT_HEARTBEAT;
module_param(heartbeat, uint, 0);
MODULE_PARM_DESC(heartbeat,
		"Watchdog heartbeat period in seconds from 1 to "
		__MODULE_STRING(MAX_HEARTBEAT) ", default "
		__MODULE_STRING(DEFAULT_HEARTBEAT));

struct jz4740_wdt_drvdata {
	struct watchdog_device wdt;
	struct regmap *map;
	struct clk *clk;
	unsigned long clk_rate;
	void __iomem *base;
	void __iomem *cgu_base;		/* JZ4730 only */
	struct clk *rtc_clk;
	enum jz4740_wdt_type type;
	spinlock_t lock;
};

static int jz4730_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	unsigned int rtc_clk_rate;
	unsigned long timeout_value;

	rtc_clk_rate = clk_get_rate(drvdata->rtc_clk);

	/* On the JZ4730, the timer limit raises the alarm, and so the timeout
	   must be subtracted from the limit to produce a starting value. */

	timeout_value = 0xffffffff - rtc_clk_rate * wdt_dev->timeout;

	writel(timeout_value, drvdata->base + JZ_REG_WDT_TIMER_COUNTER32);
	return 0;
}

static int jz4740_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	regmap_write(drvdata->map, TCU_REG_WDT_TCNT, 0);

	return 0;
}

static void jz4730_cpm_update(struct jz4740_wdt_drvdata *wdt, size_t reg,
	uint32_t affected, uint32_t value)
{
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);

	writel((readl(wdt->cgu_base + reg) & ~affected) | value,
		wdt->cgu_base + reg);

	spin_unlock_irqrestore(&wdt->lock, flags);
}

static void jz4730_wdt_update_status(struct jz4740_wdt_drvdata *wdt,
	uint8_t affected, uint8_t value)
{
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);

	writeb((readl(wdt->cgu_base + JZ_REG_WDT_TIMER_STATUS)
		& ~affected) | value,
		wdt->cgu_base + JZ_REG_WDT_TIMER_STATUS);

	spin_unlock_irqrestore(&wdt->lock, flags);
}

static int jz4730_wdt_set_timeout(struct watchdog_device *wdt_dev,
				    unsigned int new_timeout)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	jz4730_wdt_update_status(drvdata, JZ_WDT_TIMER_STATUS_START, 0);

	wdt_dev->timeout = new_timeout;
	jz4730_wdt_ping(wdt_dev);

	/* Clock source selection is done using a CPM register. */

	jz4730_cpm_update(drvdata, JZ_REG_CPM_OCR,
		JZ_CPM_OCR_EXT_RTC_CLK, JZ_CPM_OCR_EXT_RTC_CLK);

	jz4730_wdt_update_status(drvdata, JZ_WDT_TIMER_STATUS_START,
		JZ_WDT_TIMER_STATUS_START);

	return 0;
}

static int jz4740_wdt_set_timeout(struct watchdog_device *wdt_dev,
				    unsigned int new_timeout)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	u16 timeout_value = (u16)(drvdata->clk_rate * new_timeout);
	unsigned int tcer;

	regmap_read(drvdata->map, TCU_REG_WDT_TCER, &tcer);
	regmap_write(drvdata->map, TCU_REG_WDT_TCER, 0);

	regmap_write(drvdata->map, TCU_REG_WDT_TDR, timeout_value);
	regmap_write(drvdata->map, TCU_REG_WDT_TCNT, 0);

	if (tcer & TCU_WDT_TCER_TCEN)
		regmap_write(drvdata->map, TCU_REG_WDT_TCER, TCU_WDT_TCER_TCEN);

	wdt_dev->timeout = new_timeout;
	return 0;
}

static int jz4730_wdt_start(struct watchdog_device *wdt_dev)
{
	jz4730_wdt_set_timeout(wdt_dev, wdt_dev->timeout);

	return 0;
}

static int jz4740_wdt_start(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	unsigned int tcer;
	int ret;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		return ret;

	regmap_read(drvdata->map, TCU_REG_WDT_TCER, &tcer);

	jz4740_wdt_set_timeout(wdt_dev, wdt_dev->timeout);

	/* Start watchdog if it wasn't started already */
	if (!(tcer & TCU_WDT_TCER_TCEN))
		regmap_write(drvdata->map, TCU_REG_WDT_TCER, TCU_WDT_TCER_TCEN);

	return 0;
}

static int jz4730_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	jz4730_wdt_update_status(drvdata, JZ_WDT_TIMER_STATUS_START, 0);

	return 0;
}

static int jz4740_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	regmap_write(drvdata->map, TCU_REG_WDT_TCER, 0);
	clk_disable_unprepare(drvdata->clk);

	return 0;
}


static int jz4740_wdt_restart(struct watchdog_device *wdt_dev,
			      unsigned long action, void *data)
{
	wdt_dev->timeout = 0;
	jz4740_wdt_start(wdt_dev);
	return 0;
}

static int jz4730_wdt_init_cgu(struct platform_device *pdev, struct jz4740_wdt_drvdata *wdt)
{
	struct platform_device *cgu_pdev;
	struct device_node *np;
	struct resource *mem;

	/* Find the CGU in the device tree. */

	np = of_find_compatible_node(NULL, NULL, "ingenic,jz4730-cgu");
	if (!np) {
		dev_err(&pdev->dev, "Could not locate CGU for JZ4730 support.\n");
		return -ENOENT;
	}

	/* Obtain the CGU device. */

	cgu_pdev = of_find_device_by_node(np);
	if (!cgu_pdev) {
		dev_err(&pdev->dev, "Could not obtain CGU device for JZ4730 support.\n");
		return -ENOENT;
	}

	/* Obtain the memory resource. */

	mem = platform_get_resource(cgu_pdev, IORESOURCE_MEM, 0);
	wdt->cgu_base = devm_ioremap_resource(&cgu_pdev->dev, mem);
	if (IS_ERR(wdt->cgu_base))
		return PTR_ERR(wdt->cgu_base);

	return 0;
}

static const struct watchdog_info jz4740_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "jz4740 Watchdog",
};

static const struct watchdog_ops jz4730_wdt_ops = {
	.owner = THIS_MODULE,
	.start = jz4730_wdt_start,
	.stop = jz4730_wdt_stop,
	.ping = jz4730_wdt_ping,
	.set_timeout = jz4730_wdt_set_timeout,
};

static const struct watchdog_ops jz4740_wdt_ops = {
	.owner = THIS_MODULE,
	.start = jz4740_wdt_start,
	.stop = jz4740_wdt_stop,
	.ping = jz4740_wdt_ping,
	.set_timeout = jz4740_wdt_set_timeout,
	.restart = jz4740_wdt_restart,
};

static const struct of_device_id jz4740_wdt_of_matches[] = {
	{ .compatible = "ingenic,jz4730-watchdog", .data = (void *) ID_JZ4730},
	{ .compatible = "ingenic,jz4740-watchdog", .data = (void *) ID_JZ4740},
	{ .compatible = "ingenic,jz4780-watchdog", .data = (void *) ID_JZ4740},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jz4740_wdt_of_matches);

static int jz4740_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4740_wdt_drvdata *drvdata;
	struct watchdog_device *jz4740_wdt;
	long rate;
	int ret;
	const struct platform_device_id *id = platform_get_device_id(pdev);
	const struct of_device_id *of_id = of_match_device(
			jz4740_wdt_of_matches, &pdev->dev);

	drvdata = devm_kzalloc(dev, sizeof(struct jz4740_wdt_drvdata),
			       GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->clk = devm_clk_get(&pdev->dev, "wdt");
	if (IS_ERR(drvdata->clk)) {
		dev_err(&pdev->dev, "cannot find WDT clock\n");
		return PTR_ERR(drvdata->clk);
	}

	/* Set smallest clock possible */
	rate = clk_round_rate(drvdata->clk, 1);
	if (rate < 0)
		return rate;
	if (of_id)
		drvdata->type = (enum jz4740_wdt_type)of_id->data;
	else
		drvdata->type = id->driver_data;

	if (heartbeat < 1 || heartbeat > MAX_HEARTBEAT)
		heartbeat = DEFAULT_HEARTBEAT;

	ret = clk_set_rate(drvdata->clk, rate);
	if (ret)
		return ret;

	drvdata->clk_rate = rate;
	jz4740_wdt = &drvdata->wdt;
	jz4740_wdt->info = &jz4740_wdt_info;
	jz4740_wdt->ops = &jz4740_wdt_ops;
	jz4740_wdt->min_timeout = 1;
	jz4740_wdt->max_timeout = 0xffff / rate;
	jz4740_wdt->timeout = clamp(heartbeat,
				    jz4740_wdt->min_timeout,
				    jz4740_wdt->max_timeout);
	jz4740_wdt->parent = dev;

	/* Choose different operations for the JZ4730. */

	if (drvdata->type == ID_JZ4730)
		jz4740_wdt->ops = &jz4730_wdt_ops;
	else
		jz4740_wdt->ops = &jz4740_wdt_ops;

	watchdog_set_nowayout(jz4740_wdt, nowayout);
	watchdog_set_drvdata(jz4740_wdt, drvdata);

	drvdata->map = device_node_to_regmap(dev->parent->of_node);
	if (!drvdata->map) {
		dev_err(dev, "regmap not found\n");
		return -EINVAL;
	}

	/* Timer configuration requires CPM register access on the JZ4730. */

	if (drvdata->type == ID_JZ4730)
		jz4730_wdt_init_cgu(pdev, drvdata);

	ret = devm_watchdog_register_device(dev, &drvdata->wdt);
	if (ret < 0)
		return ret;

	return 0;
}

static struct platform_driver jz4740_wdt_driver = {
	.probe = jz4740_wdt_probe,
	.driver = {
		.name = "jz4740-wdt",
		.of_match_table = of_match_ptr(jz4740_wdt_of_matches),
	},
};

module_platform_driver(jz4740_wdt_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("jz4740 Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4740-wdt");
