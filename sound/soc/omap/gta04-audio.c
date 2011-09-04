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

static int omap3beagle_hw_params(struct snd_pcm_substream *substream,
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

static int omap3pandora_hp_event(struct snd_soc_dapm_widget *w,
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

static const struct snd_soc_dapm_widget gta04_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("PCM DAC", "HiFi Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA_E("Headphone Amplifier", SND_SOC_NOPM,
					   0, 0, NULL, 0, omap3pandora_hp_event,
					   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_MIC("Headphone Mic", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
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

static int omap3gta04_init(struct snd_soc_codec *codec)
{
	int ret;
	
	ret = snd_soc_dapm_new_controls(codec, gta04_dapm_widgets,
									ARRAY_SIZE(gta04_dapm_widgets));
	if (ret < 0)
		return ret;
	
	snd_soc_dapm_add_routes(codec, audio_map,
							ARRAY_SIZE(audio_map));

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

static struct snd_soc_ops omap3beagle_ops = {
	.hw_params = omap3beagle_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3beagle_dai[] = {
	{
		.name = "TWL4030",
		.stream_name = "TWL4030",
		.cpu_dai = &omap_mcbsp_dai[0],
		.codec_dai = &twl4030_dai[TWL4030_DAI_HIFI],
		.ops = &omap3beagle_ops,
		.init = &omap3gta04_init
	}
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap3beagle = {
	.name = "gta04",
	.platform = &omap_soc_platform,
	.dai_link = &omap3beagle_dai[0],
	.num_links = ARRAY_SIZE(omap3beagle_dai),
};

/* Audio subsystem */
static struct snd_soc_device omap3beagle_snd_devdata = {
	.card = &snd_soc_omap3beagle,
	.codec_dev = &soc_codec_dev_twl4030,
};

static struct platform_device *omap3beagle_snd_device;

static int __init omap3beagle_soc_init(void)
{
	int ret;

	if (!machine_is_gta04() && !machine_is_omap3_beagle()) {
		pr_debug("Not GTA04!\n");
		return -ENODEV;
	}
	pr_info("GTA04 OMAP3 SoC snd init\n");
	
	// FIXME: set any GPIOs i.e. enable Audio in/out switch
	// microphone power etc.

	omap3beagle_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap3beagle_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap3beagle_snd_device, &omap3beagle_snd_devdata);
	omap3beagle_snd_devdata.dev = &omap3beagle_snd_device->dev;
	*(unsigned int *)omap3beagle_dai[0].cpu_dai->private_data = 1; /* McBSP2 = TPS65950 */
	ret = platform_device_add(omap3beagle_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap3beagle_snd_device);

	return ret;
}

static void __exit omap3beagle_soc_exit(void)
{
	platform_device_unregister(omap3beagle_snd_device);
	// switch off power
}

module_init(omap3beagle_soc_init);
module_exit(omap3beagle_soc_exit);

MODULE_AUTHOR("Steve Sakoman <steve@sakoman.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 GTA04");
MODULE_LICENSE("GPL");
