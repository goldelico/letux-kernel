// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PCM1754 DAC ASoC codec driver
 *
 * Copyright (c) 2022 Alvin Šipraga <alsi@bang-olufsen.dk>
 * Copyright (c) 2025 Stefan Kerkmann <s.kerkmann@pengutronix.de>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

struct pcm1754_variant {
	const char *dai_name;
	unsigned int rate_min;
	unsigned int rate_max;
	u64 selectable_formats;
};

struct pcm1754_priv {
	unsigned int format;
	struct gpio_desc *gpiod_mute;
	struct gpio_desc *gpiod_format;
	struct gpio_desc *gpiod_enable;
	const struct pcm1754_variant *variant;
};

static const struct pcm1754_variant pcm1754_variant = {
	.dai_name = "pcm1754",
	.rate_min = 5000,
	.rate_max = 200000,
	.selectable_formats = SND_SOC_POSSIBLE_DAIFMT_I2S |
			      SND_SOC_POSSIBLE_DAIFMT_RIGHT_J,
};

static const struct pcm1754_variant pcm1773_variant = {
	.dai_name = "pcm1773",
	.rate_min = 8000,
	.rate_max = 48000,
	.selectable_formats = SND_SOC_POSSIBLE_DAIFMT_I2S |
			      SND_SOC_POSSIBLE_DAIFMT_LEFT_J,
};

static int pcm1754_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int format)
{
	struct snd_soc_component *component = codec_dai->component;
	struct pcm1754_priv *priv = snd_soc_component_get_drvdata(component);

	priv->format = format;

	return 0;
}

static int pcm1754_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *codec_dai)
{
	struct snd_soc_component *component = codec_dai->component;
	struct pcm1754_priv *priv = snd_soc_component_get_drvdata(component);
	int format;

	switch (priv->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		if (priv->variant != &pcm1754_variant ||
		    params_width(params) != 16)
			return -EINVAL;
		format = 1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		if (priv->variant != &pcm1773_variant ||
		    (params_width(params) != 16 && params_width(params) != 24))
			return -EINVAL;
		format = 0;
		break;
	case SND_SOC_DAIFMT_I2S:
		if (params_width(params) != 16 && params_width(params) != 24)
			return -EINVAL;
		format = priv->variant == &pcm1773_variant ? 1 : 0;
		break;
	default:
		dev_err(component->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	if (priv->gpiod_format)
		gpiod_set_value_cansleep(priv->gpiod_format, format);

	return 0;
}

static int pcm1754_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct pcm1754_priv *priv = snd_soc_component_get_drvdata(dai->component);

	if (priv->gpiod_mute)
		gpiod_set_value_cansleep(priv->gpiod_mute, mute);

	return 0;
}

static const struct snd_soc_dai_ops pcm1754_dai_ops = {
	.set_fmt = pcm1754_set_dai_fmt,
	.hw_params = pcm1754_hw_params,
	.mute_stream = pcm1754_mute_stream,
};

static int pcm1754_enable_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct pcm1754_priv *priv = snd_soc_component_get_drvdata(component);

	if (!priv->gpiod_enable)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(priv->gpiod_enable, 1);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		gpiod_set_value_cansleep(priv->gpiod_enable, 0);
		usleep_range(1000, 2000);
		break;
	}

	return 0;
}

static const struct snd_soc_dai_driver pcm1754_dai = {
	.name = "pcm1754",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 5000,
		.rate_max	= 200000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
	},
	.ops = &pcm1754_dai_ops,
};

static const struct snd_soc_dapm_widget pcm1754_dapm_widgets[] = {
	SND_SOC_DAPM_REGULATOR_SUPPLY("VCC", 0, 0),
	SND_SOC_DAPM_SUPPLY("Enable", SND_SOC_NOPM, 0, 0,
			    pcm1754_enable_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_DAC("DAC1", "Channel 1 Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC2", "Channel 2 Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("VOUTL"),
	SND_SOC_DAPM_OUTPUT("VOUTR"),
};

static const struct snd_soc_dapm_route pcm1754_dapm_routes[] = {
	{ "DAC1", NULL, "Playback" },
	{ "DAC2", NULL, "Playback" },

	{ "DAC1", NULL, "Enable" },
	{ "DAC2", NULL, "Enable" },
	{ "Enable", NULL, "VCC" },

	{ "VOUTL", NULL, "DAC1" },
	{ "VOUTR", NULL, "DAC2" },
};

static const struct snd_soc_component_driver soc_component_dev_pcm1754 = {
	.dapm_widgets = pcm1754_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pcm1754_dapm_widgets),
	.dapm_routes = pcm1754_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(pcm1754_dapm_routes),
};

static int pcm1754_probe(struct platform_device *pdev)
{
	struct pcm1754_priv *priv;
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_driver *dai_drv;
	struct snd_soc_dai_ops *dai_ops;
	const struct pcm1754_variant *variant;
	int ret;

	variant = device_get_match_data(dev);
	if (!variant)
		return -ENODEV;

	dai_drv = devm_kmemdup(dev, &pcm1754_dai, sizeof(*dai_drv), GFP_KERNEL);
	if (!dai_drv)
		return -ENOMEM;

	priv = devm_kzalloc_obj(dev, *priv);
	if (!priv)
		return -ENOMEM;

	priv->variant = variant;

	priv->gpiod_mute = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpiod_mute))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_mute),
					 "failed to get mute gpio");

	priv->gpiod_format = devm_gpiod_get_optional(dev, "format", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_format))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_format),
					 "failed to get format gpio");

	priv->gpiod_enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_enable))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_enable),
				     "failed to get enable gpio");

	dai_ops = devm_kmemdup(dev, &pcm1754_dai_ops, sizeof(*dai_ops),
			      GFP_KERNEL);
	if (!dai_ops)
		return -ENOMEM;

	dai_ops->auto_selectable_formats = &variant->selectable_formats;
	dai_ops->num_auto_selectable_formats = 1;
	dai_drv->ops = dai_ops;
	dai_drv->name = variant->dai_name;
	dai_drv->playback.rate_min = variant->rate_min;
	dai_drv->playback.rate_max = variant->rate_max;

	dev_set_drvdata(dev, priv);

	ret = devm_snd_soc_register_component(
		&pdev->dev, &soc_component_dev_pcm1754, dai_drv, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id pcm1754_of_match[] = {
	{ .compatible = "ti,pcm1754", .data = &pcm1754_variant },
	{ .compatible = "ti,pcm1773", .data = &pcm1773_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm1754_of_match);
#endif

static struct platform_driver pcm1754_codec_driver = {
	.driver = {
		.name = "pcm1754-codec",
		.of_match_table = of_match_ptr(pcm1754_of_match),
	},
	.probe = pcm1754_probe,
};

module_platform_driver(pcm1754_codec_driver);

MODULE_DESCRIPTION("ASoC PCM1754 driver");
MODULE_AUTHOR("Alvin Šipraga <alsi@bang-olufsen.dk>");
MODULE_AUTHOR("Stefan Kerkmann <s.kerkmann@pengutronix.de>");
MODULE_LICENSE("GPL");
