/*
 * ALSA SoC driver for OMAP4/5 AESS (Audio Engine Sub-System)
 *
 * Copyright (C) 2010-2013 Texas Instruments
 *
 * Authors: Liam Girdwood <lrg@ti.com>
 *          Misael Lopez Cruz <misael.lopez@ti.com>
 *          Sebastien Guiriec <s-guiriec@ti.com>
 *          Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * Contact: Misael Lopez Cruz <misael.lopez@ti.com>
 *          Peter Ujfalusi <peter.ujfalusi@ti.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/pm_opp.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

#include <sound/soc.h>
#include <sound/soc-topology.h>
#ifdef FIXME	// mechanism does no longer exist since v4.18 and wasn't used anywhere else for long time
#include "../../../arch/arm/mach-omap2/omap-pm.h"
#endif

#define pr_err pr_info
#define dev_err dev_info

#include "omap-aess-priv.h"
#include "aess_port.h"
#include "omap-aess.h"
#include "aess_utils.h"
#include "port_mgr.h"
#include "aess_opp.h"
#include "aess_gain.h"

static const char *aess_memory_bank[5] = {
	"dmem",
	"cmem",
	"smem",
	"pmem",
	"mpu"
};

void omap_aess_pm_get(struct omap_aess *aess)
{
	pm_runtime_get_sync(aess->dev);
}
EXPORT_SYMBOL_GPL(omap_aess_pm_get);

void omap_aess_pm_put(struct omap_aess *aess)
{
	pm_runtime_put_sync(aess->dev);
}
EXPORT_SYMBOL_GPL(omap_aess_pm_put);

void omap_aess_pm_shutdown(struct omap_aess *aess)
{
	int ret;

	if (aess->active && omap_aess_check_activity(aess))
		return;

	aess_set_opp_processing(aess, OMAP_AESS_OPP_25);
	aess->opp.level = 25;

	omap_aess_write_event_generator(aess, EVENT_STOP);
	udelay(250);
	if (aess->device_scale) {
		ret = aess->device_scale(aess->dev, aess->dev,
					 aess->opp.freqs[0]);
		if (ret)
			dev_err(aess->dev, "failed to scale to lowest OPP\n");
	}
}
EXPORT_SYMBOL_GPL(omap_aess_pm_shutdown);

void omap_aess_pm_set_mode(struct omap_aess *aess, int mode)
{
	aess->dc_offset.power_mode = mode;
}
EXPORT_SYMBOL(omap_aess_pm_set_mode);

static int omap_aess_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_aess *aess;
	int i, ret;

	aess = devm_kzalloc(&pdev->dev, sizeof(struct omap_aess), GFP_KERNEL);
	if (aess == NULL)
		return -ENOMEM;
printk("%s %d: platform_set_drvdata(%px, %px)\n", __func__, __LINE__, pdev, aess);
	platform_set_drvdata(pdev, aess);

	for (i = 0; i < OMAP_AESS_IO_RESOURCES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   aess_memory_bank[i]);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource: %s\n",
				aess_memory_bank[i]);
			return -ENODEV;
		}
		if (!devm_request_mem_region(&pdev->dev, res->start, 
					     resource_size(res),
					     aess_memory_bank[i]))
			return -EBUSY;

		aess->io_base[i] = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (!aess->io_base[i])
			return -ENOMEM;

		if (i == 0)
			aess->dmem_l4 = res->start;
	}

	/* Get needed L3 I/O addresses */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmem_dma");
	if (res == NULL) {
		dev_err(&pdev->dev, "no L3 resource: dmem\n");
		return -ENODEV;
	}
	aess->dmem_l3 = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	if (res == NULL) {
		dev_err(&pdev->dev, "no L3 resource: AESS configuration\n");
		return -ENODEV;
	}
	aess->aess_config_l3 = res->start;

	aess->irq = platform_get_irq(pdev, 0);
	if (aess->irq < 0)
		return aess->irq;


#ifdef CONFIG_PM
#ifdef FIXME	// mechanism does no longer exist since v4.18 and wasn't used anywhere else for long time
	aess->get_context_lost_count = omap_pm_get_dev_context_loss_count;
#endif
	aess->device_scale = NULL;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

#endif	/* CONFIG_PM */
	aess->dev = &pdev->dev;
printk("%s %d: pdev=%px aess=%px dev=%px\n", __func__, __LINE__, pdev, aess, aess->dev);

	mutex_init(&aess->mutex);
	mutex_init(&aess->opp.mutex);
	mutex_init(&aess->opp.req_mutex);
	INIT_LIST_HEAD(&aess->opp.req);

	spin_lock_init(&aess->lock);
	INIT_LIST_HEAD(&aess->ports);

#ifdef CONFIG_DEBUG_FS
	aess->debugfs_root = debugfs_create_dir("omap-aess", NULL);
	if (!aess->debugfs_root)
		dev_warn(aess->dev, "Failed to create root for debugfs\n");
#endif

	omap_aess_port_mgr_init(aess);

	get_device(aess->dev);
	aess->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	aess->dev->dma_mask = &aess->dev->coherent_dma_mask;

printk("%s %d: aess=%px dev=%px dma_mask=%px\n", __func__, __LINE__, aess, aess->dev, aess->dev->dma_mask);
printk("%s %d: aess=%px dev=%px coherent_dma_mask=%px\n", __func__, __LINE__, aess, aess->dev, &aess->dev->coherent_dma_mask);

	put_device(aess->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &omap_aess_component, omap_aess_dai,
					 ARRAY_SIZE(omap_aess_dai));
	if (ret < 0) {
		aess->dev->dma_mask = NULL;
		dev_err(aess->dev, "failed to register AESS PCM %d\n", ret);
#ifdef CONFIG_PM
		pm_runtime_disable(&pdev->dev);
#endif
	}

	return ret;
}

static void omap_aess_engine_remove(struct platform_device *pdev)
{
	struct omap_aess *aess = platform_get_drvdata(pdev);

#ifdef CONFIG_PM
	pm_runtime_disable(&pdev->dev);
#endif
	omap_aess_port_mgr_cleanup(aess);
#ifdef CONFIG_DEBUG_FS
	if (aess->debugfs_root)
		debugfs_remove_recursive(aess->debugfs_root);
#endif
}

static const struct of_device_id omap_aess_of_match[] = {
	{ .compatible = "ti,omap4-aess", },
	{ }
};
MODULE_DEVICE_TABLE(of, omap_aess_of_match);

static struct platform_driver omap_aess_driver = {
	.driver = {
		.name = "omap-aess",
		.owner = THIS_MODULE,
		.of_match_table = omap_aess_of_match,
	},
	.probe = omap_aess_engine_probe,
	.remove = omap_aess_engine_remove,
};

module_platform_driver(omap_aess_driver);

MODULE_ALIAS("platform:omap-aess");
MODULE_DESCRIPTION("ASoC OMAP4 AESS");
MODULE_AUTHOR("Liam Girdwood <lrg@ti.com>");
MODULE_LICENSE("GPL");
