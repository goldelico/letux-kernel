/*
 * Copyright (C) 2011 John Ogness
 *   Author: John Ogness <john.ogness@linutronix.de>
 *
 * based on sound/soc/omap/omap3beagle.c by
 *   Steve Sakoman <steve@sakoman.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/platform_device.h>

#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>

#include "omap-mcbsp.h"
#include "../codecs/w2cbw003-bt.h"

static int gta04_headset_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	/* setup codec dai and cpu dai hardware params */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;

	fmt =	SND_SOC_DAIFMT_I2S |	// I2S
			SND_SOC_DAIFMT_IB_IF |	// positive sync pulse, driven on rising, sampled on falling clock
			SND_SOC_DAIFMT_CBM_CFM;	// clocks come from bluetooth modem - but this can be configured in the Modem chip

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT, 0,
								 SND_SOC_CLOCK_IN);
	// FIXME: set clock divisor
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	return 0;
}

static int gta04_headset_init(struct snd_soc_pcm_runtime *runtime)
{
	/* add controls */
	/* add routes */
	/* setup pins */
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_sync(dapm);
	return 0;
}

static int gta04_headset_startup(struct snd_pcm_substream *substream)
{
	/* enable clock used by codec */
	return 0;
}

static void gta04_headset_shutdown(struct snd_pcm_substream *substream)
{
	/* disable clock used by codec */
}

static struct snd_soc_ops gta04_headset_ops = {
	.startup	= gta04_headset_startup,
	.hw_params	= gta04_headset_hw_params,
	.shutdown	= gta04_headset_shutdown,
};

/* digital headset interface glue - connects codec <--> cpu */
static struct snd_soc_dai_link gta04_headset_dai = {
	.name 		= "W2CBW003",
	.stream_name 	= "W2CBW003",
	.cpu_dai_name	= "omap-mcbsp.3",
	.platform_name	= "omap-pcm-audio",
	.codec_dai_name	= "W2CBW003",
	.codec_name	= "w2cbw003_codec_audio",
	.init		= gta04_headset_init,
	.ops 		= &gta04_headset_ops,
};

/* headset machine driver */
static struct snd_soc_card gta04_headset_card = {
	.name		= "gta04-headset",
	.dai_link	= &gta04_headset_dai,
	.num_links	= 1,
};

static int gta04_headset_soc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &gta04_headset_card;
	int err;

	if (!of_machine_is_compatible("ti,omap3-gta04"))
		return -ENODEV;

	card->dev = &pdev->dev;

	if (np) {
		struct device_node *dai_node;

		dai_node = of_parse_phandle(np, "gta04,cpu-dai", 0);
		if (!dai_node) {
			dev_err(&pdev->dev, "McBSP node is not provided\n");
			return -EINVAL;
		}
		gta04_headset_dai.cpu_dai_name = NULL;
		gta04_headset_dai.platform_name = NULL;
		gta04_headset_dai.cpu_of_node = dai_node;

		dai_node = of_parse_phandle(np, "gta04,codec", 0);
		if (!dai_node) {
			dev_err(&pdev->dev, "Codec node is not provided\n");
			return -EINVAL;
		}

		gta04_headset_dai.codec_name = NULL;
		gta04_headset_dai.codec_of_node = dai_node;
	}

	err = devm_snd_soc_register_card(card->dev, card);
	if (err) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", err);
		return err;
	}

	return 0;
}

static int gta04_headset_soc_remove(struct platform_device *pdev)
{
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id gta04_headset_of_match[] = {
	{ .compatible = "goldelico,gta04-headset", },
	{},
};
MODULE_DEVICE_TABLE(of, gta04_headset_of_match);
#endif

static struct platform_driver gta04_headset_soc_driver = {
	.driver = {
		.name = "gta04-headset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gta04_headset_of_match),
	},
	.probe = gta04_headset_soc_probe,
	.remove = gta04_headset_soc_remove,
};

module_platform_driver(gta04_headset_soc_driver);

MODULE_AUTHOR("John Ogness <john.ogness@linutronix.de>");
MODULE_DESCRIPTION("ALSA SoC GTA04 Headset");
MODULE_LICENSE("GPL v2");
