/*
 * gta04.c  --  SoC audio for GTA04 (based on OMAP3 Beagle)
 *
 * Author: Steve Sakoman <steve@sakoman.com>
 * Author: Nikolaus Schaller <hns@goldelico.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/twl4030.h"
#include "../codecs/gtm601.h"
#include "../codecs/si47xx.h"
#include "../codecs/w2cbw003-bt.h"

static int gta04_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int fmt;
	int ret;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 4: /* Four channel TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

/* this shows how we could control the AUX in/out switch or the Video in/out */

static int gta04_hp_event(struct snd_soc_dapm_widget *w,
								 struct snd_kcontrol *k, int event)
{
	/*
	 if (SND_SOC_DAPM_EVENT_ON(event)) {
	 gpio_set_value(OMAP3_PANDORA_DAC_POWER_GPIO, 1);
	 gpio_set_value(OMAP3_PANDORA_AMP_POWER_GPIO, 1);
	 } else {
	 gpio_set_value(OMAP3_PANDORA_AMP_POWER_GPIO, 0);
	 mdelay(1);
	 gpio_set_value(OMAP3_PANDORA_DAC_POWER_GPIO, 0);
	 }
	 */	
	return 0;
}

static const struct snd_soc_dapm_widget gta04_audio_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("PCM DAC", "HiFi Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA_E("Headphone Amplifier", SND_SOC_NOPM,
					   0, 0, NULL, 0, gta04_hp_event,
					   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_MIC("Headphone Mic", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route gta04_audio_map[] = {
	{"Headphone Amplifier", NULL, "PCM DAC"},
	{"Line Out", NULL, "PCM DAC"},
	{"Headphone Jack", NULL, "Headphone Amplifier"},

	{"AUXL", NULL, "Line In"},
	{"AUXR", NULL, "Line In"},
	
	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headphone Mic"},

	{"MAINMIC", NULL, "Mic Bias 1"},
	{"Mic Bias 1", NULL, "Internal Mic"},
	/*
	 {"SUBMIC", NULL, "Mic Bias 2"},
	 {"Mic Bias 2", NULL, "Mic (external)"},
	 */
};

static int gta04_audio_init(struct snd_soc_codec *codec)
{
	int ret;
	
	ret = snd_soc_dapm_new_controls(codec, gta04_audio_dapm_widgets,
									ARRAY_SIZE(gta04_audio_dapm_widgets));
	if (ret < 0)
		return ret;
	
	snd_soc_dapm_add_routes(codec, gta04_audio_map,
							ARRAY_SIZE(gta04_audio_map));

//	snd_soc_dapm_enable_pin(codec, "Ext Mic");
//	snd_soc_dapm_enable_pin(codec, "Ext Spk");
//	snd_soc_dapm_disable_pin(codec, "Headphone Mic");
//	snd_soc_dapm_disable_pin(codec, "Headphone Amplifier");

	/* TWL4030 not connected pins */
	//	snd_soc_dapm_nc_pin(codec, "OUTL");
	//	snd_soc_dapm_nc_pin(codec, "OUTR");
	//	snd_soc_dapm_nc_pin(codec, "EARPIECE");
	snd_soc_dapm_nc_pin(codec, "PREDRIVEL");
	snd_soc_dapm_nc_pin(codec, "PREDRIVER");
	//	snd_soc_dapm_nc_pin(codec, "HSOL");
	//	snd_soc_dapm_nc_pin(codec, "HSOR");
	snd_soc_dapm_nc_pin(codec, "CARKITMIC");
	snd_soc_dapm_nc_pin(codec, "CARKITL");
	snd_soc_dapm_nc_pin(codec, "CARKITR");
	//	snd_soc_dapm_nc_pin(codec, "HFL");
	//	snd_soc_dapm_nc_pin(codec, "HFR");
	//	snd_soc_dapm_nc_pin(codec, "VIBRA");
	//	snd_soc_dapm_nc_pin(codec, "HSMIC");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC0");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC1");
		
	return snd_soc_dapm_sync(codec);
}

static int gta04_voice_init(struct snd_soc_codec *codec)
{
//	int ret;
	printk("gta04_voice_init()\n");
	return snd_soc_dapm_sync(codec);
}

static int gta04_headset_init(struct snd_soc_codec *codec)
{
//	int ret;
	printk("gta04_headset_init()\n");
	return snd_soc_dapm_sync(codec);
}

static int gta04_fm_init(struct snd_soc_codec *codec)
{
//	int ret;
	printk("gta04_fm_init()\n");
	return snd_soc_dapm_sync(codec);
}

// FIXME: do we need different ops for all sound cards

static struct snd_soc_ops gta04_ops = {
	.hw_params = gta04_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link gta04_audio_dai[] = {
	{
		.name = "TWL4030",
		.stream_name = "TWL4030",
		.cpu_dai = &omap_mcbsp_dai[0],
		.codec_dai = &twl4030_dai[TWL4030_DAI_HIFI],
		.ops = &gta04_ops,
		.init = &gta04_audio_init
	}
};

/* Audio machine driver */
static struct snd_soc_card gta04_audio_soc = {
	.name = "gta04-audio",
	.platform = &omap_soc_platform,
	.dai_link = &gta04_audio_dai[0],
	.num_links = ARRAY_SIZE(gta04_audio_dai),
};

/* Audio subsystem */
static struct snd_soc_device gta04_audio_devdata = {
	.card = &gta04_audio_soc,
	.codec_dev = &soc_codec_dev_twl4030,
};

static struct platform_device *gta04_audio_device;

/* UMTS voice link */

static struct snd_soc_dai_link gta04_voice_dai[] = { /* maybe we can merge all DAI links into single array? */
	{
	.name = "GTM601",
	.stream_name = "GTM601",
	.cpu_dai = &omap_mcbsp_dai[1],
	.codec_dai = &gtm601_dai,
	.ops = &gta04_ops,
	.init = &gta04_voice_init
	}
};

/* Voice machine driver */
static struct snd_soc_card gta04_voice_soc = {
	.name = "gta04-voice",
	.platform = &omap_soc_platform,
	.dai_link = &gta04_voice_dai[0],
	.num_links = ARRAY_SIZE(gta04_voice_dai),
};

/* Voice subsystem */
static struct snd_soc_device gta04_voice_devdata = {
	.card = &gta04_voice_soc,
	.codec_dev = &soc_codec_dev_gtm601,
};

static struct platform_device *gta04_voice_device;

/* Bluetooth headset link */

static struct snd_soc_dai_link gta04_headset_dai[] = { /* maybe we can merge all DAI links into single array? */
	{
	.name = "W2CBW003",
	.stream_name = "W2CBW003",
	.cpu_dai = &omap_mcbsp_dai[2],
	.codec_dai = &w2cbw003_dai,
	.ops = &gta04_ops,
	.init = &gta04_headset_init
	}
};

/* BT machine driver */
static struct snd_soc_card gta04_headset_soc = {
	.name = "gta04-headset",
	.platform = &omap_soc_platform,
	.dai_link = &gta04_headset_dai[0],
	.num_links = ARRAY_SIZE(gta04_headset_dai),
};

/* BR subsystem */
static struct snd_soc_device gta04_headset_devdata = {
	.card = &gta04_headset_soc,
	.codec_dev = &soc_codec_dev_w2cbw003,
};

static struct platform_device *gta04_headset_device;

/* FM radio link */

static struct platform_device *gta04_fm_device;

static struct snd_soc_dai_link gta04_fm_dai[] = { /* maybe we can merge all DAI links into single array? */
	{
	.name = "SI47XX",
	.stream_name = "SI47XX",
	.cpu_dai = &omap_mcbsp_dai[3],
	.codec_dai = &si47xx_dai,
	.ops = &gta04_ops,
	.init = &gta04_fm_init
	}
};

/* FM machine driver */
static struct snd_soc_card gta04_fm_soc = {
	.name = "gta04-fm",
	.platform = &omap_soc_platform,
	.dai_link = &gta04_fm_dai[0],
	.num_links = ARRAY_SIZE(gta04_fm_dai),
};

/* FM radio subsystem */

// CHECKME
static struct si47xx_setup_data gta04_fm_soc_data = {
	.i2c_bus = 2,	/* I2C2 */
	.i2c_address = 0x11
};

static struct snd_soc_device gta04_fm_devdata = {
	.card = &gta04_fm_soc,
	.codec_dev = &soc_codec_dev_si47xx,
	.codec_data = &gta04_fm_soc_data
};

static int __init gta04_soc_init(void)
{
	int ret;

	if (!machine_is_gta04() && !machine_is_omap3_beagle()) {
		pr_debug("Not GTA04!\n");
		return -ENODEV;
	}
	pr_info("GTA04 SoC snd init\n");
	
	// FIXME: set any GPIOs i.e. enable Audio in/out switch
	// microphone power etc.

	gta04_audio_device = platform_device_alloc("soc-audio", -1);
	if (!gta04_audio_device) {
		printk(KERN_ERR "Platform device allocation failed (audio)\n");
		return -ENOMEM;
	}

	platform_set_drvdata(gta04_audio_device, &gta04_audio_devdata);
	gta04_audio_devdata.dev = &gta04_audio_device->dev;
	*(unsigned int *)gta04_audio_dai[0].cpu_dai->private_data = 2-1; /* McBSP2 = TPS65950 */
	ret = platform_device_add(gta04_audio_device);
	if (ret)
		goto err1;

	/* allocate Voice and assign to McBSP4 */

	gta04_voice_device = platform_device_alloc("soc-voice", -1);
	if (!gta04_voice_device) {
		platform_device_put(gta04_audio_device);
		printk(KERN_ERR "Platform device allocation failed (void)\n");
		return -ENOMEM;
	}
	
	platform_set_drvdata(gta04_voice_device, &gta04_voice_devdata);
	gta04_voice_devdata.dev = &gta04_voice_device->dev;
	*(unsigned int *)gta04_voice_dai[0].cpu_dai->private_data = 4-1; /* McBSP4 = GTM601 */
	ret = platform_device_add(gta04_voice_device);
	if (ret)
		goto err2;
	
	/* allocate FM and assign to McBSP1 */
	
	gta04_fm_device = platform_device_alloc("soc-fm", -1);
	if (!gta04_fm_device) {
		platform_device_put(gta04_audio_device);
		platform_device_put(gta04_voice_device);
		printk(KERN_ERR "Platform device allocation failed (fm)\n");
		return -ENOMEM;
	}
	
	platform_set_drvdata(gta04_fm_device, &gta04_fm_devdata);
	gta04_fm_devdata.dev = &gta04_fm_device->dev;
	*(unsigned int *)gta04_fm_dai[0].cpu_dai->private_data = 1-1; /* McBSP1 = Si47xx */
	ret = platform_device_add(gta04_fm_device);
	if (ret)
		goto err3;
	
	/* allocate Bluetooth and assign to McBSP3 */
	
	gta04_headset_device = platform_device_alloc("soc-headset", -1);
	if (!gta04_headset_device) {
		platform_device_put(gta04_audio_device);
		platform_device_put(gta04_voice_device);
		platform_device_put(gta04_fm_device);
		printk(KERN_ERR "Platform device allocation failed (fm)\n");
		return -ENOMEM;
	}
	
	platform_set_drvdata(gta04_headset_device, &gta04_headset_devdata);
	gta04_headset_devdata.dev = &gta04_headset_device->dev;
	*(unsigned int *)gta04_headset_dai[0].cpu_dai->private_data = 3-1; /* McBSP3 = W2CBW003 */
	ret = platform_device_add(gta04_headset_device);
	if (ret)
		goto err4;
	
	return 0;

err4:
	printk(KERN_ERR "Unable to add platform device (headset)\n");
	platform_device_put(gta04_fm_device);
err3:
	printk(KERN_ERR "Unable to add platform device (fm)\n");
	platform_device_put(gta04_fm_device);
err2:
	printk(KERN_ERR "Unable to add platform device (voice)\n");
	platform_device_put(gta04_voice_device);
err1:
	printk(KERN_ERR "Unable to add platform device (audio)\n");
	platform_device_put(gta04_audio_device);

	return ret;
}

static void __exit gta04_soc_exit(void)
{
	platform_device_unregister(gta04_audio_device);
	if(gta04_voice_device)
		platform_device_unregister(gta04_voice_device);
	if(gta04_fm_device)
		platform_device_unregister(gta04_fm_device);
	// switch off power
}

module_init(gta04_soc_init);
module_exit(gta04_soc_exit);

MODULE_AUTHOR("Steve Sakoman <steve@sakoman.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 GTA04");
MODULE_LICENSE("GPL");
