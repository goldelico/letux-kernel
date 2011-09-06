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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/w2cbw003-bt.h"

static int gta04_headset_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	/* setup codec dai and cpu dai hardware params */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
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

static int gta04_headset_init(struct snd_soc_codec *codec)
{
	/* add controls */
	/* add routes */
	/* setup pins */

	snd_soc_dapm_sync(codec);
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
	.cpu_dai 	= &omap_mcbsp_dai[2],
	.codec_dai 	= &w2cbw003_dai,
	.init		= gta04_headset_init,
	.ops 		= &gta04_headset_ops,
};

/* headset machine driver */
static struct snd_soc_card gta04_headset_card = {
	.name		= "gta04-headset",
	.platform	= &omap_soc_platform,
	.dai_link	= &gta04_headset_dai,
	.num_links	= 1,
};

/* headset subsystem */
static struct snd_soc_device gta04_headset_devdata = {
	.card		= &gta04_headset_card,
	.codec_dev	= &soc_codec_dev_w2cbw003,
};

static struct platform_device *gta04_headset_snd_device;

static int __init gta04_headset_soc_init(void)
{
	struct device *dev;
	int ret;

	pr_info("gta04-headset SoC init\n");

	gta04_headset_snd_device = platform_device_alloc("soc-audio", 2);
	if (!gta04_headset_snd_device) {
		printk(KERN_ERR "platform device allocation failed\n");
		return -ENOMEM;
	}

	dev = &gta04_headset_snd_device->dev;

	platform_set_drvdata(gta04_headset_snd_device, &gta04_headset_devdata);
	gta04_headset_devdata.dev = &gta04_headset_snd_device->dev;
	*(unsigned int *)gta04_headset_dai.cpu_dai->private_data = 2;	// McBSP3

	ret = platform_device_add(gta04_headset_snd_device);
	if (ret) {
		printk(KERN_ERR "unable to add platform device\n");
		platform_device_put(gta04_headset_snd_device);
	}

	return ret;
}

static void __exit gta04_headset_soc_exit(void)
{
	platform_device_unregister(gta04_headset_snd_device);
}

module_init(gta04_headset_soc_init);
module_exit(gta04_headset_soc_exit);

MODULE_AUTHOR("John Ogness <john.ogness@linutronix.de>");
MODULE_DESCRIPTION("ALSA SoC GTA04 Headset");
MODULE_LICENSE("GPL v2");
