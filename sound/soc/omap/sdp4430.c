/*
 * sdp4430.c  --  SoC audio for TI OMAP4430 SDP
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
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
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/mux.h>

#include "mcpdm.h"
#include "omap-abe.h"
#include "omap-pcm.h"
#include "omap-mcbsp.h"
#include "../codecs/twl6040.h"
#include "../codecs/abe-twl6040.h"

#ifdef CONFIG_SND_OMAP_SOC_HDMI
#include "omap-hdmi.h"
#endif

static struct snd_soc_dai_link sdp4430_dai[];
static struct snd_soc_card snd_soc_sdp4430;
static int twl6040_power_mode;
static struct snd_soc_card snd_soc_sdp4430;

static struct i2c_client *tps61305_client;
static struct i2c_board_info tps61305_hwmon_info = {
        I2C_BOARD_INFO("tps61305", 0x33),
};

static int sdp4430_tps61305_configure(void)
{
	u8 data[2];

	data[0] = 0x01;
	data[1] = 0x60;
	if (i2c_master_send(tps61305_client, data, 2) != 2)
		printk(KERN_ERR "I2C write to TSP61305 failed\n");

	data[0] = 0x02;
	if (i2c_master_send(tps61305_client, data, 2) != 2)
		printk(KERN_ERR "I2C write to TSP61305 failed\n");

	return 0;
}

static int sdp4430_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int clk_id, freq;
	int ret;

	if (twl6040_power_mode) {
		clk_id = TWL6040_SYSCLK_SEL_HPPLL;
		freq = 38400000;
	} else {
		clk_id = TWL6040_SYSCLK_SEL_LPPLL;
		freq = 32768;
	}

	/* set the codec mclk */
	ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
				SND_SOC_CLOCK_IN);
	if (ret) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}
	return 0;
}

static struct snd_soc_ops sdp4430_ops = {
	.hw_params = sdp4430_hw_params,
};

#ifdef CONFIG_SND_OMAP_VOICE_TEST
static int sdp4430_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int clk_id, freq;
	int ret;

	if (twl6040_power_mode) {
		clk_id = TWL6040_SYSCLK_SEL_HPPLL;
		freq = 38400000;
	} else {
		clk_id = TWL6040_SYSCLK_SEL_LPPLL;
		freq = 32768;
	}

	/* set the codec mclk */
	ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
				SND_SOC_CLOCK_IN);
	if (ret) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set McBSP clock to external */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
				     64 * params_rate(params),
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 193);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu clock div\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops sdp4430_voice_ops = {
	.hw_params = sdp4430_voice_hw_params,
};
#endif

/* Headset jack */
static struct snd_soc_jack hs_jack;

/*Headset jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headset Stereophone",
		.mask = SND_JACK_HEADPHONE,
	},
};

static int sdp4430_get_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = twl6040_power_mode;
	return 0;
}

static int sdp4430_set_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (twl6040_power_mode == ucontrol->value.integer.value[0])
		return 0;

	twl6040_power_mode = ucontrol->value.integer.value[0];

	return 1;
}

static const char *power_texts[] = {"Low-Power", "High-Performance"};

static const struct soc_enum sdp4430_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, power_texts),
};

static const struct snd_kcontrol_new sdp4430_controls[] = {
	SOC_ENUM_EXT("TWL6040 Power Mode", sdp4430_enum[0],
		sdp4430_get_power_mode, sdp4430_set_power_mode),
};

/* SDP4430 machine DAPM */
static const struct snd_soc_dapm_widget sdp4430_twl6040_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_SPK("Earphone Spk", NULL),
	SND_SOC_DAPM_INPUT("Aux/FM Stereo In"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Main Mic Bias"},
	{"SUBMIC", NULL, "Main Mic Bias"},
	{"Main Mic Bias", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Headset Stereophone (Headphone): HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},

	/* Earphone speaker */
	{"Earphone Spk", NULL, "EP"},

	/* Aux/FM Stereo In: AFML, AFMR */
	{"AFML", NULL, "Aux/FM Stereo In"},
	{"AFMR", NULL, "Aux/FM Stereo In"},
};

static int sdp4430_twl6040_init(struct snd_soc_codec *codec)
{
	int ret;

	/* Add SDP4430 specific controls */
	ret = snd_soc_add_controls(codec, sdp4430_controls,
				ARRAY_SIZE(sdp4430_controls));
	if (ret)
		return ret;

	/* Add SDP4430 specific widgets */
	ret = snd_soc_dapm_new_controls(codec, sdp4430_twl6040_dapm_widgets,
				ARRAY_SIZE(sdp4430_twl6040_dapm_widgets));
	if (ret)
		return ret;

	/* Set up SDP4430 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* SDP4430 connected pins */
	snd_soc_dapm_enable_pin(codec, "Ext Mic");
	snd_soc_dapm_enable_pin(codec, "Ext Spk");
	snd_soc_dapm_enable_pin(codec, "AFML");
	snd_soc_dapm_enable_pin(codec, "AFMR");
	snd_soc_dapm_disable_pin(codec, "Headset Mic");
	snd_soc_dapm_disable_pin(codec, "Headset Stereophone");

	ret = snd_soc_dapm_sync(codec);
	if (ret)
		return ret;

	/*Headset jack detection */
	ret = snd_soc_jack_new(&snd_soc_sdp4430, "Headset Jack",
				SND_JACK_HEADSET, &hs_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
				hs_jack_pins);

	twl6040_hs_jack_detect(codec, &hs_jack, SND_JACK_HEADSET);

	return ret;
}

#ifdef CONFIG_SND_OMAP_SOC_HDMI
struct snd_soc_dai null_dai = {
	.name = "null",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
};
#endif

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link sdp4430_dai[] = {
	{
		.name = "abe-twl6040",
		.stream_name = "Multimedia",
		.cpu_dai = &omap_abe_dai[OMAP_ABE_MM_DAI],
		.codec_dai = &abe_dai[0],
		.init = sdp4430_twl6040_init,
		.ops = &sdp4430_ops,
	},
	{
		.name = "abe-twl6040",
		.stream_name = "Tones DL",
		.cpu_dai = &omap_abe_dai[OMAP_ABE_TONES_DL_DAI],
		.codec_dai = &abe_dai[1],
		.ops = &sdp4430_ops,
	},
	{
		.name = "abe-twl6040",
		.stream_name = "Voice",
		.cpu_dai = &omap_abe_dai[OMAP_ABE_VOICE_DAI],
		.codec_dai = &abe_dai[2],
#ifdef CONFIG_SND_OMAP_VOICE_TEST
		.ops = &sdp4430_voice_ops,
#else
		.ops = &sdp4430_ops,
#endif
	},
	{
		.name = "abe-twl6040",
		.stream_name = "Digital Uplink",
		.cpu_dai = &omap_abe_dai[OMAP_ABE_DIG_UPLINK_DAI],
		.codec_dai = &abe_dai[3],
		.ops = &sdp4430_ops,
	},
	{
		.name = "abe-twl6040",
		.stream_name = "Vibrator",
		.cpu_dai = &omap_abe_dai[OMAP_ABE_VIB_DAI],
		.codec_dai = &abe_dai[4],
		.ops = &sdp4430_ops,
	},
#ifdef CONFIG_SND_OMAP_SOC_HDMI
	{
		.name = "hdmi",
		.stream_name = "HDMI",
		.cpu_dai = &omap_hdmi_dai,
		.codec_dai = &null_dai,
	},
#endif
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_sdp4430 = {
	.name = "SDP4430",
	.platform = &omap_soc_platform,
	.dai_link = sdp4430_dai,
	.num_links = ARRAY_SIZE(sdp4430_dai),
};

/* Audio subsystem */
static struct snd_soc_device sdp4430_snd_devdata = {
	.card = &snd_soc_sdp4430,
	.codec_dev = &soc_codec_dev_abe_twl6040,
};

static struct platform_device *sdp4430_snd_device;

static int __init sdp4430_soc_init(void)
{
	struct i2c_adapter *adapter;
	int ret;

	if (!machine_is_omap_4430sdp()) {
		pr_debug("Not SDP4430!\n");
		return -ENODEV;
	}
	pr_info("SDP4430 SoC init\n");

#ifdef CONFIG_SND_OMAP_SOC_HDMI
	snd_soc_register_dais(&null_dai, 1);
#endif

	adapter = i2c_get_adapter(1);
	if (!adapter) {
		printk(KERN_ERR "can't get i2c adapter\n");
		return -ENODEV;
	}

	tps61305_client = i2c_new_device(adapter, &tps61305_hwmon_info);
	if (!tps61305_client) {
		printk(KERN_ERR "can't add i2c device\n");
		return -ENODEV;
	}

	sdp4430_tps61305_configure();

	sdp4430_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sdp4430_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(sdp4430_snd_device, &sdp4430_snd_devdata);
	sdp4430_snd_devdata.dev = &sdp4430_snd_device->dev;

	ret = platform_device_add(sdp4430_snd_device);
	if (ret)
		goto err;

	ret = snd_soc_dai_set_sysclk(sdp4430_dai[0].codec_dai,
				TWL6040_SYSCLK_SEL_HPPLL, 38400000,
				SND_SOC_CLOCK_IN);
	if (ret) {
		printk(KERN_ERR "can't set codec system clock\n");
		goto err;
	}

	/* Codec starts in HP mode */
	twl6040_power_mode = 1;

	pr_info("SDP4430 SoC init\n");
	return 0;

err:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(sdp4430_snd_device);
	return ret;
}
module_init(sdp4430_soc_init);

static void __exit sdp4430_soc_exit(void)
{
	platform_device_unregister(sdp4430_snd_device);
	i2c_unregister_device(tps61305_client);
}
module_exit(sdp4430_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC SDP4430");
MODULE_LICENSE("GPL");

