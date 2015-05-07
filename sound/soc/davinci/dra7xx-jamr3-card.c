/*
 * dra7xx-jamr3-card.c  --  SoC audio for TI JAMR3 companion board
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
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

struct jamr3_snd_data {
	struct clk *mclk;
	unsigned int mclk_freq;
	unsigned int slots;
	int always_on;
	const int *rates;
	int num_rates;
};

static const int jamr3_snd_rates[] = {
	 5512,  8000, 11025, 16000, 22050,
	32000, 44100, 48000, 88200, 96000,
};

static unsigned int jamr3_get_bclk(struct snd_pcm_hw_params *params, int slots)
{
	int sample_size = snd_pcm_format_width(params_format(params));
	int rate = params_rate(params);

	return sample_size * slots * rate;
}

static int jamr3_snd_hwrule_rates(struct snd_pcm_hw_params *params,
					      struct snd_pcm_hw_rule *rule)
{
	struct jamr3_snd_data *card_data = rule->private;
	unsigned int slots = card_data->slots;
	unsigned int mclk_freq = card_data->mclk_freq;
	unsigned int bclk_freq;
	int width = snd_pcm_format_physical_width(params_format(params));
	int rates[10], count = 0;
	int i;

	for (i = 0; i < card_data->num_rates; i++) {
		bclk_freq = slots * card_data->rates[i] * width;
		if ((mclk_freq % bclk_freq) == 0)
			rates[count++] = card_data->rates[i];
	}

	return snd_interval_list(hw_param_interval(params, rule->var),
				 count, rates, 0);
}

static int jamr3_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct jamr3_snd_data *card_data = snd_soc_card_get_drvdata(card);

	/* CODEC's TDM slot mask is always 2, aligned on 2-ch boundaries */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_CHANNELS, 2);

	snd_pcm_hw_rule_add(substream->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			    jamr3_snd_hwrule_rates, card_data,
			    SNDRV_PCM_HW_PARAM_RATE, -1);

	return 0;
}

static int jamr3_snd_hw_params(struct snd_pcm_substream *substream,
					   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct jamr3_snd_data *card_data = snd_soc_card_get_drvdata(card);
	unsigned int fmt = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_DSP_B |
			   SND_SOC_DAIFMT_IB_NF;
	unsigned int bclk_freq;
	unsigned int slot_mask = 3;
	int sample_size = snd_pcm_format_width(params_format(params));
	int i, ret;

	bclk_freq = jamr3_get_bclk(params, card_data->slots);

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		dev_err(card->dev, "can't set CPU DAI format %d\n", ret);
		return ret;
	}

	/* Set McASP BCLK divider (clkid = 1) */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 1,
				card_data->mclk_freq / bclk_freq);
	if (ret < 0) {
		dev_err(card->dev, "can't set CPU DAI clock divider %d\n", ret);
		return ret;
	}

	/* Set McASP sysclk from AHCLKX sourced from ATL */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0,
				     card_data->mclk_freq,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "can't set CPU DAI sysclk %d\n", ret);
		return ret;
	}

	for (i = 0; i < rtd->num_codecs; i++) {
		codec_dai = rtd->codecs[i].codec_dai;

		ret = snd_soc_dai_set_fmt(codec_dai, fmt);
		if (ret < 0) {
			dev_err(card->dev, "can't set CODEC DAI format %d\n",
				ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(codec_dai, slot_mask, slot_mask,
					       card_data->slots,
					       sample_size);
		if (ret < 0) {
			dev_err(card->dev, "can't set CODEC DAI tdm slots %d\n",
				ret);
			return ret;
		}
		slot_mask <<= 2;

		/* Set MCLK as clock source for tlv320aic3106 */
		ret = snd_soc_dai_set_sysclk(codec_dai, 0,
					     card_data->mclk_freq,
					     SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(card->dev, "can't set CODEC sysclk %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static struct snd_soc_ops jamr3_snd_ops = {
	.startup = jamr3_snd_startup,
	.hw_params = jamr3_snd_hw_params,
};

static int jamr3_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					     struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* Each tlv320aic3x receives two channels from McASP in TDM mode */
	channels->min = 2;
	channels->max = 2;

	return 0;
}

static int jamr3_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct jamr3_snd_data *card_data = snd_soc_card_get_drvdata(card);

	/* Minimize artifacts as much as possible if can be afforded */
	if (card_data->always_on) {
		rtd->pmdown_time = INT_MAX;
		if (card_data->mclk)
			clk_prepare_enable(card_data->mclk);
	} else {
		rtd->pmdown_time = 0;
	}

	return 0;
}

static struct snd_soc_dai_link_codec jamr3_snd_codecs[] = {
	{
		.codec_dai_name = "tlv320aic3x-hifi",
		.hw_params_fixup = jamr3_hw_params_fixup,
	},
	{
		.codec_dai_name = "tlv320aic3x-hifi",
		.hw_params_fixup = jamr3_hw_params_fixup,
	},
	{
		.codec_dai_name = "tlv320aic3x-hifi",
		.hw_params_fixup = jamr3_hw_params_fixup,
	},
};

static struct snd_soc_dai_link jamr3_snd_dai = {
	/* Multichannel: McASP6 + 3 * tlv320aic3106 */
	.name = "Multichannel",
	.ops = &jamr3_snd_ops,
	.codecs = jamr3_snd_codecs,
	.num_codecs = ARRAY_SIZE(jamr3_snd_codecs),
	.init = jamr3_snd_init,
};

static int jamr3_snd_add_dai_link(struct snd_soc_card *card)
{
	struct jamr3_snd_data *card_data = snd_soc_card_get_drvdata(card);
	struct device_node *node = card->dev->of_node;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *dai_node;
	int index = 0, ret;

	if (!node) {
		dev_err(card->dev, "card node is invalid\n");
		return -EINVAL;
	}

	if (card_data->mclk == NULL) {
		ret = of_property_read_u32(node, "ti,audio-mclk-freq",
				&card_data->mclk_freq);
		if (ret < 0) {
			dev_err(card->dev,
				"Must Specify codec clock.\n");
			return -EINVAL;
		}
	}

	dai_node = of_parse_phandle(node, "ti,mcasp-controller", 0);
	if (!dai_node) {
		dev_err(card->dev, "cpu dai node is invalid\n");
		return -EINVAL;
	}

	dai_link->cpu_of_node = dai_node;

	dai_link->platform_of_node = dai_node;

	dai_node = of_parse_phandle(node, "ti,audio-codec-a", 0);
	if (!dai_node) {
		dev_err(card->dev, "codec dai node is invalid\n");
		return -EINVAL;
	}

	dai_link->codecs[index].codec_of_node = dai_node;
	index++;

	dai_node = of_parse_phandle(node, "ti,audio-codec-b", 0);
	if (!dai_node) {
		dev_err(card->dev, "codec dai node is invalid\n");
		return -EINVAL;
	}

	dai_link->codecs[index].codec_of_node = dai_node;
	index++;

	dai_node = of_parse_phandle(node, "ti,audio-codec-c", 0);
	if (!dai_node) {
		dev_err(card->dev, "codec dai node is invalid\n");
		return -EINVAL;
	}

	dai_link->codecs[index].codec_of_node = dai_node;

	of_property_read_u32(node, "ti,audio-slots", &card_data->slots);
	if ((card_data->slots < 1) ||
	    (card_data->slots > 32)) {
		dev_err(card->dev, "invalid slot count %d\n",
			card_data->slots);
		return -EINVAL;
	}

	return 0;
}

/* DRA7 JAMR3 board widgets */
static const struct snd_soc_dapm_widget jamr3_snd_dapm_widgets[] = {

	/* JAMR3 board inputs */
	SND_SOC_DAPM_LINE("JAMR3 Stereo Aux In", NULL),
	SND_SOC_DAPM_LINE("JAMR3 Mono Mic 1", NULL),
	SND_SOC_DAPM_LINE("JAMR3 Mono Mic 2", NULL),

	/* JAMR3 board outputs */
	SND_SOC_DAPM_LINE("JAMR3 Line Out 1", NULL),
	SND_SOC_DAPM_LINE("JAMR3 Line Out 2", NULL),
	SND_SOC_DAPM_LINE("JAMR3 Line Out 3", NULL),
};

/* Audio machine driver */
static struct snd_soc_card jamr3_snd_card = {
	.owner = THIS_MODULE,
	.dapm_widgets = jamr3_snd_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(jamr3_snd_dapm_widgets),
	.dai_link = &jamr3_snd_dai,
	.num_links = 1,
};

static int jamr3_snd_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct snd_soc_card *card = &jamr3_snd_card;
	struct jamr3_snd_data *card_data;
	int ret;

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	if (!node) {
		dev_err(card->dev, "missing of_node\n");
		return -ENODEV;
	}

	card_data->rates = jamr3_snd_rates;
	card_data->num_rates = ARRAY_SIZE(jamr3_snd_rates);

	ret = snd_soc_of_parse_card_name(card, "ti,model");
	if (ret) {
		dev_err(card->dev, "card name is not provided\n");
		return -ENODEV;
	}

	card_data->mclk = devm_clk_get(&pdev->dev, "ti,codec-clock");
	if (PTR_ERR(card_data->mclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(card_data->mclk)) {
		dev_dbg(&pdev->dev, "ti,codec-clock not found.\n");
		card_data->mclk = NULL;
	} else {
		card_data->mclk_freq = clk_get_rate(card_data->mclk);
	}

	ret = snd_soc_of_parse_audio_routing(card,
					     "ti,audio-routing");
	if (ret) {
		dev_err(card->dev, "failed to parse DAPM routing\n");
		return ret;
	}

	snd_soc_card_set_drvdata(card, card_data);

	if (of_find_property(node, "ti,always-on", NULL))
		card_data->always_on = 1;

	ret = jamr3_snd_add_dai_link(card);
	if (ret) {
		dev_err(card->dev, "failed to add dai link %d\n",
			ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(card->dev, "failed to register sound card %d\n", ret);

	return ret;
}

static const struct of_device_id jamr3_snd_of_match[] = {
	{.compatible = "ti,dra7xx-jamr3-snd", },
	{ },
};
MODULE_DEVICE_TABLE(of, jamr3_snd_of_match);

static struct platform_driver jamr3_snd_driver = {
	.driver = {
		.name = "dra7xx-jamr3-snd",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = jamr3_snd_of_match,
	},
	.probe = jamr3_snd_probe,
};

module_platform_driver(jamr3_snd_driver);

MODULE_AUTHOR("Texas Instrument Inc.");
MODULE_DESCRIPTION("ALSA SoC for DRA7XX JAMR3 EVM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dra7xx-jamr3-snd");
