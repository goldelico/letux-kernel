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
#include "../codecs/si47xx.h"

static int gta04_fm_hw_params(struct snd_pcm_substream *substream,
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
			SND_SOC_DAIFMT_CBM_CFM;	// clocks come from FM tuner
	
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

static int gta04_fm_init(struct snd_soc_codec *codec)
{
	/* add controls */
	/* add routes */
	/* setup pins */

	snd_soc_dapm_sync(codec);
	return 0;
}

static int gta04_fm_startup(struct snd_pcm_substream *substream)
{
	/* enable clock used by codec */
	return 0;
}

static void gta04_fm_shutdown(struct snd_pcm_substream *substream)
{
	/* disable clock used by codec */
}

static struct snd_soc_ops gta04_fm_ops = {
	.startup	= gta04_fm_startup,
	.hw_params	= gta04_fm_hw_params,
	.shutdown	= gta04_fm_shutdown,
};

/* digital fm interface glue - connects codec <--> cpu */
static struct snd_soc_dai_link gta04_fm_dai = {
	.name 		= "Si47xx",
	.stream_name 	= "Si47xx",
	.cpu_dai 	= &omap_mcbsp_dai[3],
	.codec_dai 	= &si47xx_dai,
	.init		= gta04_fm_init,
	.ops 		= &gta04_fm_ops,
};

/* fm machine driver */
static struct snd_soc_card gta04_fm_card = {
	.name		= "gta04-fm",
	.platform	= &omap_soc_platform,
	.dai_link	= &gta04_fm_dai,
	.num_links	= 1,
};

/* fm subsystem */
static struct si47xx_setup_data gta04_fm_soc_data = {
	.i2c_bus	 = 2,
	.i2c_address	 = 0x11,
};
static struct snd_soc_device gta04_fm_devdata = {
	.card		= &gta04_fm_card,
	.codec_dev	= &soc_codec_dev_si47xx,
	.codec_data	= &gta04_fm_soc_data,
};

static struct platform_device *gta04_fm_snd_device;

static int __init gta04_fm_soc_init(void)
{
	struct device *dev;
	int ret;

	pr_info("gta04-fm SoC init\n");

	gta04_fm_snd_device = platform_device_alloc("soc-audio", 3);
	if (!gta04_fm_snd_device) {
		printk(KERN_ERR "platform device allocation failed\n");
		return -ENOMEM;
	}

	dev = &gta04_fm_snd_device->dev;

	platform_set_drvdata(gta04_fm_snd_device, &gta04_fm_devdata);
	gta04_fm_devdata.dev = &gta04_fm_snd_device->dev;
	*(unsigned int *)gta04_fm_dai.cpu_dai->private_data = 0;	// McBSP1

	ret = platform_device_add(gta04_fm_snd_device);
	if (ret) {
		printk(KERN_ERR "unable to add platform device\n");
		platform_device_put(gta04_fm_snd_device);
	}

	return ret;
}

static void __exit gta04_fm_soc_exit(void)
{
	platform_device_unregister(gta04_fm_snd_device);
}

module_init(gta04_fm_soc_init);
module_exit(gta04_fm_soc_exit);

MODULE_AUTHOR("John Ogness <john.ogness@linutronix.de>");
MODULE_DESCRIPTION("ALSA SoC GTA04 FM");
MODULE_LICENSE("GPL v2");
