/*
 *  sound/soc/ingenic/asoc-i2s-tloop.c
 *  ALSA Soc Audio Layer -- ingenic i2s tloop (part of aic controller) driver
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
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <sound/dmaengine_pcm.h>
#include "asoc-aic.h"
#include "asoc-dma.h"

static int ingenic_i2s_tloop_debug = 0;
module_param(ingenic_i2s_tloop_debug, int, 0644);
#define I2S_DEBUG_MSG(msg...)					\
	do {										\
		if (ingenic_i2s_tloop_debug)					\
			pr_debug("I2S: " msg);				\
	} while(0)

struct ingenic_i2s {
	struct device *aic;	/*register access device*/
#define I2S_WRITE 0x1
#define I2S_READ  0x2
	int i2s_mode;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_soc_dai_driver dai_driver;
	dma_addr_t dma_base;
	struct clk *cgu_clk;
	int playback_channels;
	struct ingenic_dma_pcm *ingenic_pcm;
};

#define I2S_RFIFO_DEPTH 32
#define I2S_TFIFO_DEPTH 64
#define TX_FIFO_LEVEL 16

static void dump_registers(struct device *aic)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);

	pr_info("AIC_FR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base+AICFR),ingenic_aic_read_reg(aic, AICFR));
	pr_info("AIC_CR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base+AICCR),ingenic_aic_read_reg(aic, AICCR));
	pr_info("AIC_I2SCR\t%p : 0x%08x\n",(ingenic_aic->vaddr_base+I2SCR),ingenic_aic_read_reg(aic, I2SCR));
	pr_info("AIC_SR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base+AICSR),ingenic_aic_read_reg(aic, AICSR));
	pr_info("AIC_I2SSR\t%p : 0x%08x\n",(ingenic_aic->vaddr_base+I2SSR),ingenic_aic_read_reg(aic, I2SSR));
	pr_info("AIC_I2SDIV\t%p : 0x%08x\n",(ingenic_aic->vaddr_base+I2SDIV),ingenic_aic_read_reg(aic, I2SDIV));
	pr_info("AIC_DR\t\t%p\n", (ingenic_aic->vaddr_base+AICDR));
	return;
}

static int ingenic_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	return 0;
}

static int ingenic_set_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	return 0;
}

static int ingenic_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	if (!ingenic_i2s->i2s_mode) {

		__i2s_set_tfifo_loop_trigger(aic, (I2S_RFIFO_DEPTH/4 - 1));
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		__i2s_disable_tloop(aic);
		__i2s_disable_etfl(aic);
	}

	return 0;
}


static int ingenic_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int phys_width = snd_pcm_format_physical_width(params_format(params));
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	enum dma_slave_buswidth buswidth;
	int trigger;


	/* format */
	if (phys_width == 8)
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (phys_width == 16)
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else if (phys_width == 24)
		buswidth = DMA_SLAVE_BUSWIDTH_3_BYTES;
	else if (phys_width == 32)
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
	else
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ingenic_i2s->capture_dma_data.addr_width = buswidth;
		ingenic_i2s->capture_dma_data.maxburst = I2S_RFIFO_DEPTH/2;

		trigger = ingenic_i2s->capture_dma_data.maxburst;

		__i2s_set_tfifo_loop_trigger(aic, (trigger/2 - 1));
	}

	return 0;
}

static void ingenic_i2s_start_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		__i2s_enable_tloop(aic);
		__i2s_enable_etfl(aic);
	}

	return;
}

static void ingenic_i2s_stop_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		__i2s_disable_tloop(aic);
		__i2s_disable_etfl(aic);
	}

	return;
}

static int ingenic_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_pcm_runtime_data *prtd = substream_to_prtd(substream);

	I2S_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",  __func__,
				  (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
				  cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (atomic_read(&prtd->wait_stopdma))
			return -EPIPE;
		I2S_DEBUG_MSG("i2s start\n");
		ingenic_i2s_start_substream(substream, dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (atomic_read(&prtd->wait_stopdma))
			return 0;
		I2S_DEBUG_MSG("i2s stop\n");
		ingenic_i2s_stop_substream(substream, dai);
		break;
	}
	return 0;
}

static void ingenic_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{

	return;
}

static int ingenic_i2s_probe(struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);

	ingenic_i2s->capture_dma_data.addr = ingenic_i2s->dma_base + AICLR;
	ingenic_i2s->capture_dma_data.fifo_size = I2S_RFIFO_DEPTH;

	snd_soc_dai_init_dma_data(dai, NULL,
			&ingenic_i2s->capture_dma_data);

	return 0;
}

static struct snd_soc_dai_ops ingenic_i2s_dai_ops = {
	.startup	= ingenic_i2s_startup,
	.trigger 	= ingenic_i2s_trigger,
	.hw_params 	= ingenic_i2s_hw_params,
	.shutdown	= ingenic_i2s_shutdown,
	.set_fmt	= ingenic_set_dai_fmt,
	.set_sysclk	= ingenic_set_sysclk,
};

#define ingenic_i2s_suspend	NULL
#define ingenic_i2s_resume	NULL

static ssize_t ingenic_i2s_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dev);
	dump_registers(ingenic_i2s->aic);
	return 0;
}

static DEVICE_ATTR(i2s_regs, S_IRUGO, ingenic_i2s_regs_show, NULL);
static const struct attribute *dump_attrs[] = {
	&dev_attr_i2s_regs.attr,
	NULL,
};

static const struct attribute_group dump_attr_group = {
	.attrs = (struct attribute **)dump_attrs,
};

static const struct snd_soc_component_driver ingenic_i2s_component = {
	.name		= "aic-i2s-tloop",
};

#define INGENIC_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | \
		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_U16_BE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_U24_BE | \
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_U24_3LE | \
		SNDRV_PCM_FMTBIT_S24_3BE | SNDRV_PCM_FMTBIT_U24_3BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_U20_3LE | \
		SNDRV_PCM_FMTBIT_S20_3BE | SNDRV_PCM_FMTBIT_U20_3BE | \
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_U18_3LE | \
		SNDRV_PCM_FMTBIT_S18_3BE | SNDRV_PCM_FMTBIT_U18_3BE )

extern int ingenic_dma_pcm_register(struct device *dev,  const struct snd_dmaengine_pcm_config *config, struct ingenic_dma_pcm *ingenic_pcm);
extern void ingenic_dma_pcm_unregister(struct ingenic_dma_pcm *ingenic_pcm);

static int ingenic_i2s_platfrom_probe(struct platform_device *pdev)
{
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
	struct resource res;
	struct ingenic_i2s *ingenic_i2s;
	int ret;

	if (!parent)
		return -ENOMEM;

	ingenic_i2s = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_i2s), GFP_KERNEL);
	if (!ingenic_i2s)
		return -ENOMEM;

	ret = of_address_to_resource(parent, 0, &res);
	if (ret)
		return ret;

	ingenic_i2s->dma_base = res.start;
	ingenic_i2s->aic = pdev->dev.parent;
	ingenic_i2s->i2s_mode = 0;
	ingenic_i2s->dai_driver.probe = ingenic_i2s_probe;
	//ingenic_i2s->dai_driver.suspend = ingenic_i2s_suspend;
	//ingenic_i2s->dai_driver.resume = ingenic_i2s_resume;
	ingenic_i2s->dai_driver.ops = &ingenic_i2s_dai_ops;
	ingenic_i2s->dai_driver.capture.channels_min = 1;
	ingenic_i2s->dai_driver.capture.channels_max = 2;
	ingenic_i2s->dai_driver.capture.rates = SNDRV_PCM_RATE_8000_192000;
	ingenic_i2s->dai_driver.capture.formats = INGENIC_I2S_FORMATS;

	platform_set_drvdata(pdev, (void *)ingenic_i2s);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_i2s_component,
			&ingenic_i2s->dai_driver, 1);
	if (!ret)
		dev_info(&pdev->dev, "i2s-tloop platform probe success\n");
	else
		dev_err(&pdev->dev, "i2s-tloop platform probe fail !\n");


	ret = sysfs_create_group(&pdev->dev.kobj, &dump_attr_group);
	if (ret)
		dev_info(&pdev->dev, "i2s attr create failed\n");

	ret = ingenic_dma_pcm_register(&pdev->dev, NULL, ingenic_i2s->ingenic_pcm);

	return ret;
}

static const struct of_device_id i2s_dt_match[] = {
	{ .compatible = "ingenic,i2s-tloop", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, i2s_dt_match);

static struct platform_driver ingenic_i2s_plat_driver = {
	.probe  = ingenic_i2s_platfrom_probe,
	.driver = {
		.name = "asoc-aic-i2s-tloop",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(i2s_dt_match),
	},
};
module_platform_driver(ingenic_i2s_plat_driver);

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("INGENIC AIC I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-aic-i2s");
