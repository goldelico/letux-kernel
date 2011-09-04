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
#include "../codecs/twl4030.h"

static int gta04_audio_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	/* setup codec dai and cpu dai hardware params */
	return 0;
}

static int gta04_audio_init(struct snd_soc_codec *codec)
{
	/* add controls */
	/* add routes */
	/* setup pins */

	snd_soc_dapm_sync(codec);
	return 0;
}

static int gta04_audio_startup(struct snd_pcm_substream *substream)
{
	/* enable clock used by codec */
	return 0;
}

static void gta04_audio_shutdown(struct snd_pcm_substream *substream)
{
	/* disable clock used by codec */
}

static struct snd_soc_ops gta04_audio_ops = {
	.startup	= gta04_audio_startup,
	.hw_params	= gta04_audio_hw_params,
	.shutdown	= gta04_audio_shutdown,
};

/* digital audio interface glue - connects codec <--> cpu */
static struct snd_soc_dai_link gta04_audio_dai = {
	.name 		= "twl4030",
	.stream_name 	= "twl4030",
	.cpu_dai 	= &omap_mcbsp_dai[0],
	.codec_dai 	= &twl4030_dai[TWL4030_DAI_HIFI],
	.init		= gta04_audio_init,
	.ops 		= &gta04_audio_ops,
};

/* audio machine driver */
static struct snd_soc_card gta04_audio_card = {
	.name		= "gta04-audio",
	.platform	= &omap_soc_platform,
	.dai_link	= &gta04_audio_dai,
	.num_links	= 1,
};

/* audio subsystem */
static struct snd_soc_device gta04_audio_devdata = {
	.card		= &gta04_audio_card,
	.codec_dev	= &soc_codec_dev_twl4030,
	.codec_data	= NULL, /* set if necessary */
};

static struct platform_device *gta04_audio_snd_device;

static int __init gta04_audio_soc_init(void)
{
	struct device *dev;
	int ret;

	pr_info("gta04-audio SoC init\n");

	gta04_audio_snd_device = platform_device_alloc("soc-audio", 0);
	if (!gta04_audio_snd_device) {
		printk(KERN_ERR "platform device allocation failed\n");
		return -ENOMEM;
	}

	dev = &gta04_audio_snd_device->dev;

	platform_set_drvdata(gta04_audio_snd_device, &gta04_audio_devdata);
	gta04_audio_devdata.dev = &gta04_audio_snd_device->dev;
	*(unsigned int *)gta04_audio_dai.cpu_dai->private_data = 0;

	ret = platform_device_add(gta04_audio_snd_device);
	if (ret) {
		printk(KERN_ERR "unable to add platform device\n");
		platform_device_put(gta04_audio_snd_device);
	}

	return ret;
}

static void __exit gta04_audio_soc_exit(void)
{
	platform_device_unregister(gta04_audio_snd_device);
}

module_init(gta04_audio_soc_init);
module_exit(gta04_audio_soc_exit);

MODULE_AUTHOR("John Ogness <john.ogness@linutronix.de>");
MODULE_DESCRIPTION("ALSA SoC GTA04 Audio");
MODULE_LICENSE("GPL v2");
