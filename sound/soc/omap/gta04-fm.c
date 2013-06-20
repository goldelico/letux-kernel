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

#include "mcbsp.h"
#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/si47xx.h"

/* FCLK is 96 MHz and is divided by the CLOCK_DIVISOR to generate the McBSP CLKX signal
   going to DCLK of the Si47xx.

   The DCLK clock cycle time should be between 26 ns (38.4 MHz) and 420 ns (2.38 MHz).

   Please note that the Si47xx requires to set the sampling rate property to 0 Hz
   through I2C/SPI before removing (lowering < 2 MHz) or applying the clock.
   Otherwise the chip may need a reset.

   2.5 MHz is sufficient for 24 bit stereo with 48 kHz sample rate while 2.594 MHz
   tries to avoid harmonics at typical FM frequencies like 93.5 or 98.5 MHz. But
   we can't avoid the 96.0 MHz jamming. So we choose a CLOCK_DIVISOR of 37.
 
 */

#define IN_FREQUENCY (96000000)
#define CLOCK_DIVISOR (IN_FREQUENCY / 2594000)	/* 37 */

static int gta04_fm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	/* setup codec dai and cpu dai hardware params */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;
	
	printk("gta04_fm_hw_params\n");
	fmt =	SND_SOC_DAIFMT_I2S |	/* I2S */
			SND_SOC_DAIFMT_NB_NF |	// is this the right setting?
			SND_SOC_DAIFMT_CBS_CFS;	/* clock and frame signals go from McBSP to Si47xx receiver */
	
	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}
	
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK, IN_FREQUENCY,
								 SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}	
	
	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, CLOCK_DIVISOR);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu clock divisor\n");
		return ret;
	}		
	
	return 0;
}

static int gta04_fm_init(struct snd_soc_pcm_runtime *runtime)
{
	/* add controls */
	/* add routes */
	/* setup pins */
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_sync(dapm);
	return 0;
}

static int gta04_fm_startup(struct snd_pcm_substream *substream)
{
	printk("gta04_fm_startup\n");
	/* enable clock used by codec */
	return 0;
}

static void gta04_fm_shutdown(struct snd_pcm_substream *substream)
{
	printk("gta04_fm_shutdown\n");
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
	.cpu_dai_name	= "omap-mcbsp.1",
	.platform_name	= "omap-pcm-audio",
	.codec_dai_name = "Si47xx",
	.codec_name	= "si47xx_codec_audio",
	.init		= gta04_fm_init,
	.ops 		= &gta04_fm_ops,
};

/* fm machine driver */
static struct snd_soc_card gta04_fm_card = {
	.name		= "gta04-fm",
	.dai_link	= &gta04_fm_dai,
	.num_links	= 1,
};

#if 0	// this code did configure for I2C in 2.6.32 - how to do it now?

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
#endif

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

	platform_set_drvdata(gta04_fm_snd_device, &gta04_fm_card);

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
