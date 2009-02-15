/*
 * om_gta03_wm8753.c  --  SoC audio for GTA03
 *
 * Based on neo1973_gta02_wm8753
 *
 * Copyright 2009 Openmoko Inc
 * Author: Ben Dooks <ben@simtec.co.uk>
 * Copyright 2007 Openmoko Inc
 * Author: Graeme Gregory <graeme@openmoko.org>
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory <linux@wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>

#include <plat/regs-s3c2412-iis.h>

#include "../codecs/wm8753.h"
#include "s3c24xx-pcm.h"
#include "s3c64xx-i2s.h"

/* define the scenarios */
#define NEO_AUDIO_OFF			0
#define NEO_GSM_CALL_AUDIO_HANDSET	1
#define NEO_GSM_CALL_AUDIO_HEADSET	2
#define NEO_GSM_CALL_AUDIO_BLUETOOTH	3
#define NEO_STEREO_TO_SPEAKERS		4
#define NEO_STEREO_TO_HEADPHONES	5
#define NEO_CAPTURE_HANDSET		6
#define NEO_CAPTURE_HEADSET		7
#define NEO_CAPTURE_BLUETOOTH		8
#define NEO_STEREO_TO_HANDSET_SPK	9

static struct snd_soc_card om_gta03;

static int om_gta03_hifi_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	iis_clkrate = s3c64xx_i2s_get_clockrate(cpu_dai);

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		pll_out = 12288000;
		break;
	case 48000:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, pll_out,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

#if 0
	/* do not think we need to set this if the cpu is not the bitclk
	 * master */
	/* set MCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK,
				     S3C2410_IISMOD_32FS);
	if (ret < 0)
		return ret;
#endif

	/* set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C64XX_DIV_PRESCALER, 4);
	if (ret < 0)
		return ret;

	/* codec PLL input is PCLK/4 */
	ret = snd_soc_dai_set_pll(codec_dai, WM8753_PLL1,
				  iis_clkrate / 4, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int om_gta03_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return snd_soc_dai_set_pll(codec_dai, WM8753_PLL1, 0, 0);
}

/*
 * GTA03 WM8753 HiFi DAI opserations.
 */
static struct snd_soc_ops om_gta03_hifi_ops = {
	.hw_params	= om_gta03_hifi_hw_params,
	.hw_free	= om_gta03_hifi_hw_free,
};

static int om_gta03_voice_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pcmdiv = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	iis_clkrate = s3c64xx_i2s_get_clockrate(rtd->dai->cpu_dai);

	if (params_rate(params) != 8000)
		return -EINVAL;
	if (params_channels(params) != 1)
		return -EINVAL;

	pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */

	/* todo: gg check mode (DSP_B) against CSR datasheet */
	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, (SND_SOC_DAIFMT_DSP_B |
					      SND_SOC_DAIFMT_NB_NF |
					      SND_SOC_DAIFMT_CBS_CFS));
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_PCMCLK,
				     12288000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	/* configue and enable PLL for 12.288MHz output */
	ret = snd_soc_dai_set_pll(codec_dai, WM8753_PLL2,
				  iis_clkrate / 4, 12288000);
	if (ret < 0)
		return ret;

	return 0;
}

static int om_gta03_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return snd_soc_dai_set_pll(codec_dai, WM8753_PLL2, 0, 0);
}

static struct snd_soc_ops om_gta03_voice_ops = {
	.hw_params	= om_gta03_voice_hw_params,
	.hw_free	= om_gta03_voice_hw_free,
};

static int om_gta03_set_stereo_out(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "Stereo Out", val);
	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_stereo_out(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "Stereo Out");

	return 0;
}


static int om_gta03_set_gsm_out(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "GSM Line Out", val);
	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_gsm_out(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "GSM Line Out");

	return 0;
}

static int om_gta03_set_gsm_in(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "GSM Line In", val);
	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_gsm_in(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "GSM Line In");

	return 0;
}

static int om_gta03_set_headset_mic(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "Headset Mic", val);
	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_headset_mic(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "Headset Mic");

	return 0;
}

static int om_gta03_set_handset_mic(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "Handset Mic", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_handset_mic(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "Handset Mic");

	return 0;
}

static int om_gta03_set_handset_spk(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_set_endpoint(codec, "Handset Spk", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int om_gta03_get_handset_spk(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint(codec, "Handset Spk");

	return 0;
}

static const struct snd_soc_dapm_widget wm8753_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Stereo Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line In", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_SPK("Handset Spk", NULL),
};


/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	{"Stereo Out", NULL, "LOUT1"},
	{"Stereo Out", NULL, "ROUT1"},

	/* Connections to the GSM Module */
	{"GSM Line Out", NULL, "MONO1"},
	{"GSM Line Out", NULL, "MONO2"},
	{"RXP", NULL, "GSM Line In"},
	{"RXN", NULL, "GSM Line In"},

	/* Connections to Headset */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Headset Mic"},

	/* Call Mic */
	{"MIC2", NULL, "Mic Bias"},
	{"MIC2N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Handset Mic"},

	/* Call Speaker */
	{"Handset Spk", NULL, "LOUT2"},
	{"Handset Spk", NULL, "ROUT2"},

	/* Connect the ALC pins */
	{"ACIN", NULL, "ACOP"},
};

static const struct snd_kcontrol_new wm8753_om_gta03_controls[] = {
	SOC_SINGLE_EXT("DAPM Stereo Out Switch", 0, 0, 1, 0,
		om_gta03_get_stereo_out,
		om_gta03_set_stereo_out),
	SOC_SINGLE_EXT("DAPM GSM Line Out Switch", 1, 0, 1, 0,
		om_gta03_get_gsm_out,
		om_gta03_set_gsm_out),
	SOC_SINGLE_EXT("DAPM GSM Line In Switch", 2, 0, 1, 0,
		om_gta03_get_gsm_in,
		om_gta03_set_gsm_in),
	SOC_SINGLE_EXT("DAPM Headset Mic Switch", 3, 0, 1, 0,
		om_gta03_get_headset_mic,
		om_gta03_set_headset_mic),
	SOC_SINGLE_EXT("DAPM Handset Mic Switch", 4, 0, 1, 0,
		om_gta03_get_handset_mic,
		om_gta03_set_handset_mic),
	SOC_SINGLE_EXT("DAPM Handset Spk Switch", 5, 0, 1, 0,
		om_gta03_get_handset_spk,
		om_gta03_set_handset_spk),
};

/*
 * This is an example machine initialisation for a wm8753 connected to a
 * neo1973 GTA02.
 */
static int om_gta03_wm8753_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* set up NC codec pins */
	snd_soc_dapm_set_endpoint(codec, "OUT3",  0);
	snd_soc_dapm_set_endpoint(codec, "OUT4",  0);
	snd_soc_dapm_set_endpoint(codec, "LINE1", 0);
	snd_soc_dapm_set_endpoint(codec, "LINE2", 0);


	/* Add neo1973 gta02 specific widgets */
	snd_soc_dapm_new_controls(codec, wm8753_dapm_widgets,
				  ARRAY_SIZE(wm8753_dapm_widgets));

	/* add neo1973 gta02 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_om_gta03_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&wm8753_om_gta03_controls[i],
			codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up neo1973 gta02 specific audio path audio_mapnects */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* set endpoints to default off mode */
	snd_soc_dapm_set_endpoint(codec, "Stereo Out",  0);
	snd_soc_dapm_set_endpoint(codec, "GSM Line Out",0);
	snd_soc_dapm_set_endpoint(codec, "GSM Line In", 0);
	snd_soc_dapm_set_endpoint(codec, "Headset Mic", 0);
	snd_soc_dapm_set_endpoint(codec, "Handset Mic", 0);
	snd_soc_dapm_set_endpoint(codec, "Handset Spk", 0);

	snd_soc_dapm_sync(codec);

	return 0;
}

/*
 * BT Codec DAI
 */
static struct snd_soc_dai bt_dai =
{	.name = "Bluetooth",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_dai_link om_gta03_dai[] = {
	{ /* Hifi Playback - for similatious use with voice below */
		.name		= "WM8753",
		.stream_name	= "WM8753 HiFi",
		.cpu_dai	= &s3c64xx_i2s_dai,
		.codec_dai	= &wm8753_dai[WM8753_DAI_HIFI],
		.init		= om_gta03_wm8753_init,
		.ops		= &om_gta03_hifi_ops,
	},
	{ /* Voice via BT */
		.name		= "Bluetooth",
		.stream_name	= "Voice",
		.cpu_dai	= &bt_dai,
		.codec_dai	= &wm8753_dai[WM8753_DAI_VOICE],
		.ops		= &om_gta03_voice_ops,
	},
};

static struct snd_soc_card om_gta03 = {
	.name		= "om-gta03",
	.platform	= &s3c24xx_soc_platform,
	.dai_link	= om_gta03_dai,
	.num_links	= ARRAY_SIZE(om_gta03_dai),
};

/* Audio private data */
static struct wm8753_setup_data soc_codec_data_wm8753_gta02 = {
	.i2c_bus = 0,
	.i2c_address = 0x1a,
};

static struct snd_soc_device om_gta03_snd_devdata = {
	.card		= &om_gta03,
	.codec_dev	= &soc_codec_dev_wm8753,
	.codec_data	= &soc_codec_data_wm8753_gta02,
};

static struct platform_device *om_gta03_snd_device;

static int __init om_gta03_init(void)
{
	int ret;

	if (!machine_is_openmoko_gta03()) {
		printk(KERN_INFO "Only GTA03 supported by ASoC driver\n");
		return -ENODEV;
	}

	/* register bluetooth DAI here */
	ret = snd_soc_register_dai(&bt_dai);
	if (ret)
		return ret;

	om_gta03_snd_device = platform_device_alloc("soc-audio", 0);
	if (!om_gta03_snd_device)
		return -ENOMEM;

	platform_set_drvdata(om_gta03_snd_device, &om_gta03_snd_devdata);
	om_gta03_snd_devdata.dev = &om_gta03_snd_device->dev;
	ret = platform_device_add(om_gta03_snd_device);

	if (ret) {
		platform_device_put(om_gta03_snd_device);
		return ret;
	}

	return ret;
}

static void __exit om_gta03_exit(void)
{
	platform_device_unregister(om_gta03_snd_device);
}

module_init(om_gta03_init);
module_exit(om_gta03_exit);

/* Module information */
MODULE_AUTHOR("Graeme Gregory, graeme@openmoko.org; Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("ALSA SoC WM8753 OM GTA03");
MODULE_LICENSE("GPL");
