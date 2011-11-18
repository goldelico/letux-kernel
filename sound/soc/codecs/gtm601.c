/*
 * FIXME: this is a blueprint for the GTM601 Voice PCM interface
 * needs to be adapted for GTA04
 * CHECKME: can we use the generic AC97 or SPDIF driver instead of defining our own?
 *
 * gtm601.c
 *
 *  Created on: 15-Oct-2009
 *      Author: neil.jones@imgtec.com
 *
 * Copyright (C) 2009 Imagination Technologies Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "gtm601.h"
/*
 * Note this is a simple chip with no configuration interface, sample rate is
 * determined automatically by examining the Master clock and Bit clock ratios
 */

#define GTM601_RATES  (SNDRV_PCM_RATE_8000)	// CHECKME

struct snd_soc_dai_driver gtm601_dai = {
	.name = "GTM601",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,	// CHECKME
		.channels_max = 1,
		.rates = GTM601_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,	/* this is the only format the omap-mcbsp-dai understands */
		},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = GTM601_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};
EXPORT_SYMBOL_GPL(gtm601_dai);

static int gtm601_soc_probe(struct snd_soc_codec *codec)
{
	int ret = 0;

	/* register pcms */
	ret = snd_soc_new_ac97_codec(codec, &soc_ac97_ops, 0);
	if (ret < 0)
		printk(KERN_ERR "gtm601: failed to create pcms\n");
	return ret;
}

static int gtm601_soc_remove(struct snd_soc_codec *codec)
{
	if (codec == NULL)
		return 0;
	snd_soc_free_ac97_codec(codec);
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_gtm601 = {
	.probe = 	gtm601_soc_probe,
	.remove = 	gtm601_soc_remove,
// 	.reg_cache_size = ARRAY_SIZE(ad1980_reg),
// 	.reg_word_size = sizeof(u16),
// 	.reg_cache_step = 2,
// 	.write = ac97_write,
// 	.read = ac97_read,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_gtm601);


static __devinit int gtm601_platform_probe(struct platform_device *pdev)
{
// 	gtm601_dai.dev = &pdev->dev;
// 	return snd_soc_register_dai(&gtm601_dai);
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_gtm601, &gtm601_dai, 1);
}

static int __devexit gtm601_platform_remove(struct platform_device *pdev)
{
// 	snd_soc_unregister_dai(&gtm601_dai);
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

MODULE_ALIAS("platform:gtm601_codec_audio");

static struct platform_driver gtm601_codec_driver = {
	.driver = {
			.name = "gtm601_codec_audio",
			.owner = THIS_MODULE,
	},

	.probe = gtm601_platform_probe,
	.remove = __devexit_p(gtm601_platform_remove),
};

static int __init gtm601_init(void)
{
	return platform_driver_register(&gtm601_codec_driver);
}
module_init(gtm601_init);

static void __exit gtm601_exit(void)
{
	platform_driver_unregister(&gtm601_codec_driver);
}
module_exit(gtm601_exit);

MODULE_DESCRIPTION("ASoC GTM601 driver");
MODULE_AUTHOR("Neil Jones");
MODULE_LICENSE("GPL");
