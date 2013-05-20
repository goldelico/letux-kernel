/*
 * dra7-atl.c  --  DRA7xx Audio Tracking Logic driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * Author: Misael Lopez Cruz <misael.lopez@ti.com>
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

/* ATL instances */
#define ATL0			0
#define ATL1			1
#define ATL2			2
#define ATL3			3
#define ATL_INSTANCES		4

/* ATL instance offsets */
#define ATL_INST(id)		(0x200 + (id * 0x80))

/* ATL registers */
#define ATL_REVID		0x000
#define ATL_PPMR(inst)		(inst + 0x00)
#define ATL_BBSR(inst)		(inst + 0x04)
#define ATL_ATLCR(inst)		(inst + 0x08)
#define ATL_SWEN(inst)		(inst + 0x10)
#define ATL_BWSMUX(inst)	(inst + 0x14)
#define ATL_AWSMUX(inst)	(inst + 0x18)
#define ATL_PCLKMUX(inst)	(inst + 0x1c)

/* ATL_PCLKMUX register */
#define PCLKMUX_OCP_CLK		0
#define PCLKMUX_ATLPCLK		1

/* ATL_SWEN register */
#define ATL_DISABLE		0
#define ATL_ENABLE		1

struct atl_data {
	struct device *dev;
	void __iomem *io_base;
	unsigned int atlpclk_freq;
};

static inline void dra7_atl_write(struct atl_data *atl, u32 reg, u32 val)
{
	__raw_writel(val, atl->io_base + reg);
}

static inline int dra7_atl_read(struct atl_data *atl, u32 reg)
{
	return __raw_readl(atl->io_base + reg);
}

static int dra7_atl_init(struct atl_data *atl)
{
	struct clk *gfclk, *parent_clk;
	int ret;

	gfclk = clk_get(atl->dev, "atl_gfclk_mux");
	if (IS_ERR(gfclk)) {
		dev_err(atl->dev, "failed to get ATLPCLK\n");
		return PTR_ERR(gfclk);
	}

	parent_clk = clk_get(atl->dev, "dpll_abe_m2_ck");
	if (IS_ERR(parent_clk)) {
		dev_err(atl->dev, "failed to get new parent clock\n");
		ret = PTR_ERR(parent_clk);
		goto err1;
	}

	ret = clk_set_parent(gfclk, parent_clk);
	if (ret < 0) {
		dev_err(atl->dev, "failed to reparent ATLPCLK\n");
		goto err2;
	}

	atl->atlpclk_freq = clk_get_rate(gfclk);

err2:
	clk_put(parent_clk);
err1:
	clk_put(gfclk);
	return ret;
}

static void dra7_atl_dump(struct atl_data *atl, int id)
{
	int inst = ATL_INST(id);

	dev_dbg(atl->dev, "ATL_PPMR%d    = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_PPMR(inst)));
	dev_dbg(atl->dev, "ATL_BBSR%d    = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_BBSR(inst)));
	dev_dbg(atl->dev, "ATL_ATLCR%d   = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_ATLCR(inst)));
	dev_dbg(atl->dev, "ATL_SWEN%d    = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_SWEN(inst)));
	dev_dbg(atl->dev, "ATL_BWSMUX%d  = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_BWSMUX(inst)));
	dev_dbg(atl->dev, "ATL_AWSMUX%d  = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_AWSMUX(inst)));
	dev_dbg(atl->dev, "ATL_PCLKMUX%d = 0x%08x\n",
		id, dra7_atl_read(atl, ATL_PCLKMUX(inst)));
}

static int dra7_atl_configure(struct atl_data *atl, int id,
			      unsigned int atclk_freq)
{
	int inst = ATL_INST(id);
	u32 divider;

	dev_dbg(atl->dev, "configure ATL%d to %u Hz\n", id, atclk_freq);

	/*
	 * Internal divider cannot be 0 (divide-by-1), so ATCLK max freq
	 * cannot be higher than ATLPCLK / 2
	 */
	if ((2 * atclk_freq) > atl->atlpclk_freq) {
		dev_err(atl->dev, "requested freq %u is not allowed\n",
			atclk_freq);
		return -EINVAL;
	} else if (atclk_freq == 0) {
		/* Keep ATL instance disabled */
		return 0;
	}

	/* Disable ATL while reparenting and changing ATLPCLK input */
	dra7_atl_write(atl, ATL_SWEN(inst), ATL_DISABLE);

	/* ATL divider (ATL_INTERNAL_DIVIDER): ATLPCLK-to-ATCLK ratio - 1 */
	divider = (atl->atlpclk_freq / atclk_freq) - 1;
	dra7_atl_write(atl, ATL_ATLCR(inst), divider);

	/* Enable ATL */
	dra7_atl_write(atl, ATL_SWEN(inst), ATL_ENABLE);

	dra7_atl_dump(atl, id);

	return 0;
}

static void dra7_atl_reset(struct atl_data *atl, int id)
{
	dev_err(atl->dev, "reset ATL%d\n", id);
	dra7_atl_write(atl, ATL_SWEN(ATL_INST(id)), ATL_DISABLE);
}

static int dra7_atl_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct atl_data *atl;
	struct resource *res;
	unsigned int atclk_freq;
	char prop[128];
	int i;
	int ret;

	atl = devm_kzalloc(&pdev->dev, sizeof(*atl), GFP_KERNEL);
	if (atl == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, atl);
	atl->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;

	if (!devm_request_mem_region(&pdev->dev, res->start,
				     resource_size(res), "atl"))
		return -EBUSY;

	atl->io_base = devm_ioremap(&pdev->dev, res->start,
				      resource_size(res));
	if (!atl->io_base)
		return -ENOMEM;

	if (!node) {
		dev_err(atl->dev, "missing of_node\n");
		return -ENODEV;
	}

	ret = dra7_atl_init(atl);
	if (ret) {
		dev_err(atl->dev, "failed to initialize ATL\n");
		return ret;
	}

	pm_runtime_enable(atl->dev);
	pm_runtime_get_sync(atl->dev);

	/*
	 * There is a single ATLPCLK mux for all ATL instances and
	 * is controlled by PCLKMUX0, as opposite to what TRM says
	 */
	dra7_atl_write(atl, ATL_PCLKMUX(ATL_INST(0)), PCLKMUX_ATLPCLK);

	for (i = 0; i < ATL_INSTANCES; i++) {
		atclk_freq = 0;
		snprintf(prop, sizeof(prop), "ti,atclk%u-freq", i);
		of_property_read_u32(node, prop, &atclk_freq);
		ret = dra7_atl_configure(atl, i, atclk_freq);
		if (ret) {
			dev_err(atl->dev, "failed to configure ATL%d\n", i);
			goto err;
		}
	}

	return 0;

 err:
	for ( ; i >= 0; --i)
		dra7_atl_reset(atl, i);

	pm_runtime_put_sync(atl->dev);
	pm_runtime_disable(atl->dev);

	return ret;
}

static int dra7_atl_remove(struct platform_device *pdev)
{
	struct atl_data *atl = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ATL_INSTANCES; i++)
		dra7_atl_reset(atl, i);

	pm_runtime_put_sync(atl->dev);
	pm_runtime_disable(atl->dev);

	return 0;
}

static const struct of_device_id dra7_atl_of_match[] = {
	{.compatible = "ti,dra7-atl", },
	{ },
};
MODULE_DEVICE_TABLE(of, dra7_atl_of_match);

static struct platform_driver dra7_atl_driver = {
	.driver = {
		.name = "dra7-atl-sound",
		.owner = THIS_MODULE,
		.of_match_table = dra7_atl_of_match,
	},
	.probe = dra7_atl_probe,
	.remove = dra7_atl_remove,
};

module_platform_driver(dra7_atl_driver);

MODULE_AUTHOR("Misael Lopez Cruz <misael.lopez@ti.com>");
MODULE_DESCRIPTION("ATL");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dra7-atl");
