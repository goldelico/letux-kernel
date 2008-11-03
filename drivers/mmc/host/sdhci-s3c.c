/* linux/drivers/mmc/host/sdhci-s3c.c
 *
 * Copyright 2008 Openmoko Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * SDHCI (HSMMC) support for Samsung SoC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <linux/mmc/host.h>

#include <plat/sdhci.h>

#include "sdhci.h"

#define MAX_BUS_CLK	(4)

struct sdhci_s3c {
	struct sdhci_host	*host;
	struct platform_device	*pdev;
	struct resource		*ioarea;
	struct s3c_sdhci_platdata *pdata;

	struct clk		*clk_io;	/* clock for io bus */
	struct clk		*clk_bus[MAX_BUS_CLK];
};

static inline struct sdhci_s3c *to_s3c(struct sdhci_host *host)
{
	return sdhci_priv(host);
}


static void sdhci_s3c_sel_sclk(struct sdhci_host *host)
{
	struct sdhci_s3c *ourhost = to_s3c(host);

	/* select sclk */
	u32 tmp = readl(host->ioaddr + 0x80);

	if ((tmp & (3 << 4)) == (2 << 4))
		return;

	tmp &= ~(3<<4);
	tmp |= (2 << 4);
	writel(tmp, host->ioaddr + 0x80);
}


static unsigned int sdhci_s3c_get_max_clk(struct sdhci_host *host)
{
	struct sdhci_s3c *ourhost = to_s3c(host);
	u32 control2;
	unsigned int rate;
	int clk;

	/* note, a reset will reset the clock source */

	sdhci_s3c_sel_sclk(host);

	control2 = readl(host->ioaddr + 0x80);
	clk = clk_get_rate(ourhost->clk_bus[(control2 >> 4) & 3]);

	return clk;
}

static unsigned int sdhci_s3c_get_timeout_clk(struct sdhci_host *host)
{
	return sdhci_s3c_get_max_clk(host) / 1000000;
}

static void sdhci_s3c_set_ios(struct sdhci_host *host,
			      struct mmc_ios *ios)
{
	struct sdhci_s3c *ourhost = to_s3c(host);
	struct s3c_sdhci_platdata *pdata = ourhost->pdata;
	int width;

	sdhci_s3c_sel_sclk(host);

	if (ios->power_mode != MMC_POWER_OFF) {
		switch (ios->bus_width) {
		case MMC_BUS_WIDTH_4:
			width = 4;
			break;
		case MMC_BUS_WIDTH_1:
			width = 1;
			break;
		default:
			BUG();
		}

		if (pdata->cfg_gpio)
			pdata->cfg_gpio(ourhost->pdev, width);
	}

	if (pdata->cfg_card)
		pdata->cfg_card(ourhost->pdev, host->ioaddr,
				ios, host->mmc->card);
}

static struct sdhci_ops sdhci_s3c_ops = {
	.get_max_clock		= sdhci_s3c_get_max_clk,
	.get_timeout_clock	= sdhci_s3c_get_timeout_clk,
	.set_ios		= sdhci_s3c_set_ios,
};

static int __devinit sdhci_s3c_probe(struct platform_device *pdev)
{
	struct s3c_sdhci_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_s3c *sc;
	struct resource *res;
	int ret, irq, ptr, clks;

	if (!pdata) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_s3c));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;

	sc->clk_io = clk_get(dev, "hsmmc");
	if (IS_ERR(sc->clk_io)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(sc->clk_io);
		goto err_io_clk;
	}

	/* enable the local io clock and keep it running for the moment. */
	clk_enable(sc->clk_io);

	for (clks = 0, ptr = 0; ptr < MAX_BUS_CLK; ptr++) {
		struct clk *clk;
		char *name = pdata->clocks[ptr];

		if (name == NULL)
			continue;

		clk = clk_get(dev, name);
		if (IS_ERR(clk)) {
			dev_err(dev, "failed to get clock %s\n", name);
			continue;
		}

		clks++;
		sc->clk_bus[ptr] = clk;
		clk_enable(clk);

		dev_info(dev, "clock source %d: %s (%ld Hz)\n",
			 ptr, name, clk_get_rate(clk));
	}

	if (clks == 0) {
		dev_err(dev, "failed to find any bus clocks\n");
		ret = -ENOENT;
		goto err_no_busclks;
	}

	sc->ioarea = request_mem_region(res->start, resource_size(res),
					mmc_hostname(host->mmc));
	if (!sc->ioarea) {
		dev_err(dev, "failed to reserve register area\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	host->ioaddr = ioremap_nocache(res->start, resource_size(res));
	if (!host->ioaddr) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	/* Ensure we have minimal gpio selected CMD/CLK/Detect */
	if (pdata->cfg_gpio)
		pdata->cfg_gpio(pdev, 0);

	sdhci_s3c_sel_sclk(host);

	host->hw_name = "samsung-hsmmc";
	host->ops = &sdhci_s3c_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Setup quirks for the controller */

	/* Currently with ADMA enabled we are getting some length
	 * interrupts that are not being dealt with, do disable
	 * ADMA until this is sorted out. */
	host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
	host->quirks |= SDHCI_QUIRK_32BIT_ADMA_SIZE;

	/* It seems we do not get an DATA transfer complete on non-busy
	 * transfers, not sure if this is a problem with this specific
	 * SDHCI block, or a missing configuration that needs to be set. */
	host->quirks |= SDHCI_QUIRK_NO_TCIRQ_ON_NOT_BUSY;

	host->quirks |= (SDHCI_QUIRK_32BIT_DMA_ADDR |
			 SDHCI_QUIRK_32BIT_DMA_SIZE);

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

	return 0;

 err_add_host:
	release_resource(sc->ioarea);
	kfree(sc->ioarea);

 err_req_regs:
	for (ptr = 0; ptr < MAX_BUS_CLK; ptr++) {
		clk_disable(sc->clk_bus[ptr]);
		clk_put(sc->clk_bus[ptr]);
	}

 err_no_busclks:
	clk_disable(sc->clk_io);
	clk_put(sc->clk_io);

 err_io_clk:
	sdhci_free_host(host);

	return ret;
}

static int __devexit sdhci_s3c_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sdhci_s3c_driver = {
	.probe		= sdhci_s3c_probe,
	.remove		= __devexit_p(sdhci_s3c_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-sdhci",
	},
};

static int __init sdhci_s3c_init(void)
{
	return platform_driver_register(&sdhci_s3c_driver);
}

static void __exit sdhci_s3c_exit(void)
{
	platform_driver_unregister(&sdhci_s3c_driver);
}

module_init(sdhci_s3c_init);
module_exit(sdhci_s3c_exit);

MODULE_DESCRIPTION("Samsung SDHCI (HSMMC) glue");
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("platform:s3c-sdhci");
