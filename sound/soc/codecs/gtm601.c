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

// FIXME: adjust what the GTM601 PCM I/F supports...

#define GTM601_RATES  (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |\
			SNDRV_PCM_RATE_192000)


struct snd_soc_dai gtm601_dai = {
	.name = "GTM601",
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = GTM601_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
		},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = GTM601_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
};
EXPORT_SYMBOL_GPL(gtm601_dai);

static int gtm601_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	printk("gtm601_codec_probe\n");

	gtm601_dai.dev = &pdev->dev;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
	mutex_init(&codec->mutex);

	socdev->card->codec = codec;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "GTM601";
	codec->owner = THIS_MODULE;
	codec->dai = &gtm601_dai;
	codec->num_dai = 1;
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "gtm601: failed to create pcms\n");
		goto pcm_err;
	}
	snd_soc_register_codec(codec);
	snd_soc_register_dai(&gtm601_dai);
	printk("gtm601_codec_probe ok\n");
	
	return ret;

pcm_err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int gtm601_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec == NULL)
		return 0;
	snd_soc_free_pcms(socdev);
	snd_soc_unregister_dai(&gtm601_dai);
	snd_soc_unregister_codec(codec);
	kfree(codec);
	return 0;
}

MODULE_ALIAS("platform:gtm601_codec_audio");

static struct platform_driver gtm601_codec_driver = {
	.probe = gtm601_codec_probe,
	.remove = __devexit_p(gtm601_codec_remove),
	.driver = {
			.name = "gtm601_codec_audio",
			.owner = THIS_MODULE,
	},
};

struct snd_soc_codec_device soc_codec_dev_gtm601 = {
	.probe = 	gtm601_codec_probe,
	.remove = 	gtm601_codec_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_gtm601);

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
