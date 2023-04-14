/*
 *  sound/soc/ingenic/asoc-aic.c
 *  ALSA Soc Audio Layer -- ingenic aic device driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cli <chen.li@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <sound/dmaengine_pcm.h>
#include "asoc-aic.h"
#include <linux/clk-provider.h>

#define INGENIC_I2S_RATE (24*1000000)
static const char *aic_no_mode = "no mode";
static const char *aic_i2s_mode = "i2s mode";
static const char *aic_spdif_mode = "spdif mode";
static const char *aic_ac97_mode = "ac97 mode";

static const struct of_device_id aic_dt_match[];
static const struct ingenic_aic_priv common_priv_data[];

const char* aic_work_mode_str(enum aic_mode mode)
{
	switch (mode) {
	default:
	case AIC_NO_MODE:
		return aic_no_mode;
	case AIC_I2S_MODE:
		return aic_i2s_mode;
	case AIC_SPDIF_MODE:
		return aic_spdif_mode;
	case AIC_AC97_MODE:
		return aic_ac97_mode;
	}
}
EXPORT_SYMBOL_GPL(aic_work_mode_str);

enum aic_mode aic_set_work_mode(struct device *aic,
		enum aic_mode module_mode, bool enable)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	enum aic_mode working_mode;

	spin_lock(&ingenic_aic->mode_lock);
	if  (module_mode != AIC_AC97_MODE &&
			module_mode != AIC_I2S_MODE &&
			module_mode != AIC_SPDIF_MODE)
		goto out;

	if (enable && ingenic_aic->aic_working_mode == AIC_NO_MODE) {
		ingenic_aic->aic_working_mode = module_mode;
	} else if (!enable && ingenic_aic->aic_working_mode == module_mode) {
		ingenic_aic->aic_working_mode = AIC_NO_MODE;
	}
out:
	working_mode = ingenic_aic->aic_working_mode;
	spin_unlock(&ingenic_aic->mode_lock);
	return working_mode;
}
EXPORT_SYMBOL_GPL(aic_set_work_mode);

int aic_set_rate(struct device *aic, unsigned long freq, int stream)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	unsigned long clk_rate;
	int ret;

	if (stream == STREAM_PLAY) {
		if (ingenic_aic->rate_t != freq) {
		ret = clk_set_rate(ingenic_aic->clk_t, freq);
			if (ret == -EBUSY) {
				clk_disable_unprepare(ingenic_aic->clk_t);
				ret = clk_set_rate(ingenic_aic->clk_t, freq);
				clk_prepare_enable(ingenic_aic->clk_t);
			}
			ingenic_aic->rate_t = clk_get_rate(ingenic_aic->clk_t);
		}
		clk_rate = ingenic_aic->rate_t;
	} else if (stream == STREAM_RECORD) {
		if (ingenic_aic->rate_r != freq) {
			ret = clk_set_rate(ingenic_aic->clk_r, freq);
			if (ret == -EBUSY) {
				clk_disable_unprepare(ingenic_aic->clk_r);
				ret = clk_set_rate(ingenic_aic->clk_r, freq);
				clk_prepare_enable(ingenic_aic->clk_r);
			}
			ingenic_aic->rate_r = clk_get_rate(ingenic_aic->clk_r);
		}
		clk_rate = ingenic_aic->rate_r;
	} else if (stream == STREAM_COMMON) {
		if (ingenic_aic->rate != freq) {
			ret = clk_set_rate(ingenic_aic->clk, freq);
			if (ret == -EBUSY) {
				clk_disable_unprepare(ingenic_aic->clk);
				ret = clk_set_rate(ingenic_aic->clk, freq);
				clk_prepare_enable(ingenic_aic->clk);
			}
			ingenic_aic->rate = clk_get_rate(ingenic_aic->clk);
		}
		clk_rate = ingenic_aic->rate;
	}

	return clk_rate;
}
EXPORT_SYMBOL_GPL(aic_set_rate);

int aic_clk_ctrl(struct device *aic, bool enable)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	int i;

	for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE) {
			if (!ingenic_aic->priv[i].is_bus_clk) {
				struct clk *clk = devm_clk_get(ingenic_aic->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(clk))
					return PTR_ERR(clk);
				if (enable)
					clk_prepare_enable(clk);
				else
					clk_disable_unprepare(clk);
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(aic_clk_ctrl);

static irqreturn_t ingenic_aic_irq_thread(int irq, void *dev_id)
{
	struct ingenic_aic *ingenic_aic = (struct ingenic_aic *)dev_id;

	if ((ingenic_aic->mask & 0x8) && __aic_test_ror(ingenic_aic->dev)) {
		ingenic_aic->ror++;
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"recieve fifo [overrun] interrupt time [%d]\n",
				ingenic_aic->ror);
	}

	if ((ingenic_aic->mask & 0x4) && __aic_test_tur(ingenic_aic->dev)) {
		ingenic_aic->tur++;
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"transmit fifo [underrun] interrupt time [%d]\n",
				ingenic_aic->tur);
	}

	if ((ingenic_aic->mask & 0x2) && __aic_test_rfs(ingenic_aic->dev)) {
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"[recieve] fifo at or above threshold interrupt time\n");
	}

	if ((ingenic_aic->mask & 0x1) && __aic_test_tfs(ingenic_aic->dev)) {
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"[transmit] fifo at or blow threshold interrupt time\n");
	}

	/*sleep, avoid frequently interrupt*/
	msleep(200);
	__aic_clear_all_irq_flag(ingenic_aic->dev);
	__aic_set_irq_enmask(ingenic_aic->dev, ingenic_aic->mask);
	return IRQ_HANDLED;
}

static irqreturn_t ingenic_aic_irq_handler(int irq, void *dev_id)
{
	struct ingenic_aic *ingenic_aic = (struct ingenic_aic *)dev_id;

	ingenic_aic->mask = __aic_get_irq_enmask(ingenic_aic->dev);
	if (ingenic_aic->mask && (ingenic_aic->mask & __aic_get_irq_flag(ingenic_aic->dev))) {
		/*Disable all aic interrupt*/
		__aic_set_irq_enmask(ingenic_aic->dev, 0);
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

extern int ingenic_dma_pcm_register(struct device *dev,  const struct snd_dmaengine_pcm_config *config, struct ingenic_dma_pcm *ingenic_pcm);
extern void ingenic_dma_pcm_unregister(struct ingenic_dma_pcm *ingenic_pcm);

static int ingenic_aic_probe(struct platform_device *pdev)
{
	struct ingenic_aic *ingenic_aic;
	const struct of_device_id *match;
	struct resource *res = NULL;
	struct device_node *subdev_node = NULL;
	int ret, nr_child, i = 0;

	nr_child = of_get_child_count(pdev->dev.of_node);

	ingenic_aic = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_aic) + nr_child * sizeof(void *),
			GFP_KERNEL);
	if (!ingenic_aic)
		return -ENOMEM;

	match = of_match_node(aic_dt_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;

	ingenic_aic->priv = !match->data ? common_priv_data : (struct ingenic_aic_priv *)match->data;

	ingenic_aic->dev = &pdev->dev;
	ingenic_aic->subdevs = nr_child;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	ingenic_aic->vaddr_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ingenic_aic->vaddr_base))
		return -ENOMEM;

	for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE) {
			if (ingenic_aic->priv[i].is_bus_clk) {
				struct clk *clk = devm_clk_get(ingenic_aic->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(clk))
					return PTR_ERR(clk);
				clk_prepare_enable(clk);
			}
		}

		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_CE) {
				struct clk *clk = devm_clk_get(ingenic_aic->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(clk))
					return PTR_ERR(clk);
				clk_prepare_enable(clk);
		}

		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_MUX) {
			struct clk *clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
			clk_set_parent(clk_get(NULL, ingenic_aic->priv[i].clk_name), clk_get(NULL, ingenic_aic->priv[i].mux_select));
		}

		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_DIV) {
			if (ingenic_aic->priv[i].stream_mode == AIC_STREAM_PLAYBACK) {
				ingenic_aic->clk_t = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(ingenic_aic->clk_t))
					return PTR_ERR(ingenic_aic->clk_t);
			} else if (ingenic_aic->priv[i].stream_mode == AIC_STREAM_RECORD) {
				ingenic_aic->clk_r = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(ingenic_aic->clk_r))
					return PTR_ERR(ingenic_aic->clk_r);
			} else if (ingenic_aic->priv[i].stream_mode == AIC_STREAM_COMMON) {
				ingenic_aic->clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(ingenic_aic->clk))
					return PTR_ERR(ingenic_aic->clk);
			}
		}
	}

	spin_lock_init(&ingenic_aic->mode_lock);

	ingenic_aic->irqno = platform_get_irq(pdev, 0);
	if (ingenic_aic->irqno >= 0)
		ret = devm_request_threaded_irq(&pdev->dev, ingenic_aic->irqno,
				ingenic_aic_irq_handler, ingenic_aic_irq_thread,
				IRQF_SHARED , pdev->name, (void *)ingenic_aic);
	if (!ret)
		dev_info(&pdev->dev, "register aic irq\n");

	platform_set_drvdata(pdev, (void *)ingenic_aic);

	for_each_child_of_node(pdev->dev.of_node, subdev_node)
		ingenic_aic->psubdev[i++] = of_platform_device_create(subdev_node, NULL, &pdev->dev);

	ret = ingenic_dma_pcm_register(&pdev->dev, NULL, ingenic_aic->ingenic_pcm);
	if (ret) {
		int i;
		for (i = 0; i < ingenic_aic->subdevs; i++)
			platform_device_unregister(ingenic_aic->psubdev[i]);

		for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
			if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE || ingenic_aic->priv[i].clk_mode == AIC_CLOCK_CE) {
				struct clk *clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
				if (IS_ERR(clk))
					return PTR_ERR(clk);
				clk_disable_unprepare(clk);
			}
		}

		return ret;
	}

	dev_info(&pdev->dev, "Aic core probe success\n");
	return 0;
}

static int ingenic_aic_remove(struct platform_device *pdev)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	int i;

	ingenic_dma_pcm_unregister(ingenic_aic->ingenic_pcm);

	if (!ingenic_aic)
		return 0;

	for (i = 0; i < ingenic_aic->subdevs; i++)
		platform_device_unregister(ingenic_aic->psubdev[i]);

	platform_set_drvdata(pdev, NULL);

	for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE || ingenic_aic->priv[i].clk_mode == AIC_CLOCK_CE) {
			struct clk *clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
			clk_disable_unprepare(clk);
		}
	}

	return 0;
}

#ifdef CONFIG_PM
int ingenic_aic_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	int i;

	for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE || ingenic_aic->priv[i].clk_mode == AIC_CLOCK_CE) {
			struct clk *clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
			if (__clk_is_enabled(clk))
				clk_disable_unprepare(clk);
		}
	}

	return 0;
}

int ingenic_aic_resume(struct platform_device *pdev)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	int i;

	for (i = 0; ingenic_aic->priv[i].clk_name != NULL; i++) {
		if (ingenic_aic->priv[i].clk_mode == AIC_CLOCK_GATE || ingenic_aic->priv[i].clk_mode == AIC_CLOCK_CE) {
			struct clk *clk = devm_clk_get(&pdev->dev, ingenic_aic->priv[i].clk_name);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
			clk_prepare_enable(clk);
		}
	}

	return 0;
}
#endif

static const struct ingenic_aic_priv common_priv_data[] = {
	{ .clk_mode = AIC_CLOCK_GATE, .clk_name = "gate_aic" },
	{ .clk_mode = AIC_CLOCK_DIV,  .clk_name = "cgu_i2s", .stream_mode = AIC_STREAM_PLAYBACK },
	{},
};

static const struct ingenic_aic_priv x1600_priv_data[] = {
	{ .clk_mode = AIC_CLOCK_GATE, .clk_name = "gate_audio", .is_bus_clk = true },
	{ .clk_mode = AIC_CLOCK_GATE, .clk_name = "gate_i2s0t", .is_bus_clk = false },
	{ .clk_mode = AIC_CLOCK_GATE, .clk_name = "gate_i2s0r", .is_bus_clk = false },
	{ .clk_mode = AIC_CLOCK_CE,   .clk_name = "ce_i2s0t" },
	{ .clk_mode = AIC_CLOCK_CE,   .clk_name = "ce_i2s0r" },
	{ .clk_mode = AIC_CLOCK_DIV,  .clk_name = "div_i2s0t", .stream_mode = AIC_STREAM_PLAYBACK },
	{ .clk_mode = AIC_CLOCK_DIV,  .clk_name = "div_i2s0r", .stream_mode = AIC_STREAM_RECORD },
	{ .clk_mode = AIC_CLOCK_MUX,  .clk_name = "mux_i2s0t", .mux_select = "epll" },
	{ .clk_mode = AIC_CLOCK_MUX,  .clk_name = "mux_i2s0r", .mux_select = "epll" },
	{},
};

static const struct ingenic_aic_priv x2500_priv_data[] = {
	{ .clk_mode = AIC_CLOCK_GATE, .clk_name = "gate_aic" , .is_bus_clk = true },
	{ .clk_mode = AIC_CLOCK_CE,   .clk_name = "ce_i2st" },
	{ .clk_mode = AIC_CLOCK_CE,   .clk_name = "ce_i2sr" },
	{ .clk_mode = AIC_CLOCK_DIV,  .clk_name = "div_i2st", .stream_mode = AIC_STREAM_PLAYBACK },
	{ .clk_mode = AIC_CLOCK_DIV,  .clk_name = "div_i2sr", .stream_mode = AIC_STREAM_RECORD },
	{},
};

static const struct of_device_id aic_dt_match[] = {
	{ .compatible = "ingenic,aic", .data = (void *)common_priv_data },
	{ .compatible = "ingenic,x1600-aic", .data = (void *)x1600_priv_data },
	{ .compatible = "ingenic,x2500-aic", .data = (void *)x2500_priv_data },
	{},
};
MODULE_DEVICE_TABLE(of, aic_dt_match);

static struct platform_driver ingenic_asoc_aic_driver = {
	.driver = {
		.name   = "asoc-aic",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(aic_dt_match),
	},
	.probe  = ingenic_aic_probe,
	.remove = ingenic_aic_remove,
#ifdef CONFIG_PM
	.suspend = ingenic_aic_suspend,
	.resume = ingenic_aic_resume,
#endif
};
module_platform_driver(ingenic_asoc_aic_driver);

MODULE_DESCRIPTION("INGENIC ASOC AIC core driver");
MODULE_AUTHOR("cli<chen.li@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-asoc-aic");
