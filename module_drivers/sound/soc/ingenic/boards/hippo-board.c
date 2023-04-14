/*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author: zhihao.xiao <zhihao.xiao@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include "../as-v1/asoc-aic.h"

struct x2500_icdc {
	struct snd_soc_card card;
	int spk_gpio;
	int spk_en_level;
};

static struct x2500_icdc *icodec_spk_power;
typedef void (*p_icodec_playback_power)(bool);
extern void set_playback_pwr_callback(p_icodec_playback_power func);

int x2500_i2s_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd,0);
	int ret;
	int sysclk = 24000000;
	int clk_id = 0;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		clk_id |= INGENIC_I2S_PLAYBACK;
	else
		clk_id |= INGENIC_I2S_CAPTURE;

	clk_id |= INGENIC_I2S_INNER_CODEC;

	sysclk = 256 * params_rate(params);
	ret = snd_soc_dai_set_sysclk(cpu_dai, clk_id, sysclk, SND_SOC_CLOCK_OUT);
	if (ret)
		return ret;
	return 0;
};

int x2500_i2s_hw_free(struct snd_pcm_substream *substream)
{
	/*notify release pll*/
	return 0;
};

static struct snd_soc_ops x2500_i2s_cdc_ops = {
	.hw_params = x2500_i2s_hw_params,
	.hw_free = x2500_i2s_hw_free,
};


void icodec_playback_power_ctr(bool enable)
{
	if (!gpio_is_valid(icodec_spk_power->spk_gpio))
		return;

	if (enable) {
		gpio_direction_output(icodec_spk_power->spk_gpio, icodec_spk_power->spk_en_level);
	} else {
		gpio_direction_output(icodec_spk_power->spk_gpio, !icodec_spk_power->spk_en_level);
	}
}


static const struct snd_soc_dapm_widget x2500_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("MICBIAS", NULL),
	SND_SOC_DAPM_MIC("MICL", NULL),
	SND_SOC_DAPM_MIC("MICR", NULL),
};

static int x2500_i2s_cdc_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "MICBIAS");
	snd_soc_dapm_enable_pin(dapm, "MICL");
	snd_soc_dapm_enable_pin(dapm, "MICR");

	return 0;
}

static int snd_x2500_probe(struct platform_device *pdev)
{
	struct device_node *snd_node = pdev->dev.of_node;
	struct x2500_icdc *x2500;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_dai_link_component *cpus_comp;
	struct snd_soc_dai_link_component *platforms_comp;
	struct snd_soc_dai_link_component *codecs_comp;

	enum of_gpio_flags flags;
	int num_links;
	int ret = 0, i;

	num_links = of_property_count_strings(snd_node, "ingenic,dai-link");
	if (num_links < 0)
		return num_links;
	BUG_ON(!num_links);

	x2500 = (struct x2500_icdc *)devm_kzalloc(&pdev->dev,
			sizeof(struct x2500_icdc) +
			sizeof(struct snd_soc_dai_link) * num_links,
			GFP_KERNEL);
	if (!x2500)
		return -ENOMEM;
	card = &x2500->card;
	dai_link = (struct snd_soc_dai_link *)(x2500 + 1);

	card->num_dapm_widgets = ARRAY_SIZE(x2500_dapm_widgets);
	card->dapm_widgets = x2500_dapm_widgets;
	card->num_links = num_links;
	card->dai_link = dai_link;
	card->owner = THIS_MODULE;
	card->dev = &pdev->dev;


	ret = snd_soc_of_parse_card_name(card, "ingenic,model");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "ingenic,audio-routing");
	if (ret)
		return ret;

	x2500->spk_gpio = of_get_named_gpio_flags(card->dev->of_node, "ingenic,spken-gpio", 0, &flags);
	icodec_spk_power = x2500;
	if (gpio_is_valid(x2500->spk_gpio)) {
		unsigned long init_flags;
		x2500->spk_en_level = (flags == OF_GPIO_ACTIVE_LOW ? 0 : 1);
		init_flags = (flags == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW);
		ret = devm_gpio_request_one(card->dev, x2500->spk_gpio, init_flags, "Speaker_en");
		if (ret)
			printk("dorado speaker enable pin(%d) request failed\n",x2500->spk_gpio);
		else
			printk("dorado speaker enable pin(%d) request ok\n", x2500->spk_gpio);

		/* icodec_playback_power = icodec_playback_power_ctr; */
		set_playback_pwr_callback(icodec_playback_power_ctr);
	}
//TODO
	for (i = 0; i < card->num_links; i++) {
		dai_link[i].cpus = devm_kzalloc(&pdev->dev,sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);
		dai_link[i].codecs = devm_kzalloc(&pdev->dev,sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);
		dai_link[i].platforms = devm_kzalloc(&pdev->dev,sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);

		dai_link[i].num_cpus = 1;
		dai_link[i].num_platforms = 1;
		dai_link[i].num_codecs = 1;
		dai_link[i].cpus->of_node = of_parse_phandle(snd_node, "ingenic,cpu-dai" , i);
		dai_link[i].platforms->of_node = of_parse_phandle(snd_node, "ingenic,platform", i);
		dai_link[i].codecs->of_node = of_parse_phandle(snd_node, "ingenic,codec", i);
		ret = of_property_read_string_index(snd_node, "ingenic,codec-dai", i,
				&(dai_link[i].codecs->dai_name));
		if (ret || !dai_link[i].cpus->of_node ||
				!dai_link[i].codecs->of_node ||
				!dai_link[i].platforms->of_node)
			return -ENODEV;
		ret = of_property_read_string_index(snd_node, "ingenic,dai-link", i,
				&(dai_link[i].name));
		printk("%s:%d %s\n", __func__, __LINE__, dai_link[i].name);
		if (ret)
			return -ENODEV;
		ret = of_property_read_string_index(snd_node, "ingenic,stream", i,
				&(dai_link[i].stream_name));
		if (ret)
			dai_link[i].stream_name = dai_link[i].name;
/*
		dev_dbg(&pdev->dev, "dai_link %s\n", dai_link[i].name);
		dev_dbg(&pdev->dev, "stream_name %s\n", dai_link[i].stream_name);
		dev_dbg(&pdev->dev, "cpu %s(%s)\n", dai_link[i].cpus,
				dai_link[i].cpu_of_node->full_name);
		dev_dbg(&pdev->dev, "platform %s(%s)\n", dai_link[i].platform_of_node->name,
				dai_link[i].platform_of_node->full_name);
		dev_dbg(&pdev->dev, "codec dai %s\n", dai_link[i].codec_dai_name);
		dev_dbg(&pdev->dev, "codec %s(%s)\n", dai_link[i].codec_of_node->name,
				dai_link[i].codec_of_node->full_name);
*/


		if (!strcmp(dai_link[i].name, "i2s-icodec") ||
				!strcmp(dai_link[i].codecs->dai_name, "dmic")) {
			dai_link->ops = &x2500_i2s_cdc_ops;
			dai_link->init = x2500_i2s_cdc_dai_link_init;
		}
	}
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, x2500);
	dev_info(&pdev->dev, "Sound Card successed\n");

	return ret;
}

static int snd_x2500_remove(struct platform_device *pdev)
{
	struct x2500_icdc *x2500 = platform_get_drvdata(pdev);
	snd_soc_unregister_card(&x2500->card);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, x2500);
	return 0;
}

static const struct of_device_id sound_dt_match[] = {
	{ .compatible = "ingenic,x2500-sound", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, sound_dt_match);

static struct platform_driver snd_x2500_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "x2500-sound",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(sound_dt_match),
	},
	.probe = snd_x2500_probe,
	.remove = snd_x2500_remove,
};
module_platform_driver(snd_x2500_driver);

MODULE_AUTHOR("zhxiao<zhihao.xiao@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC x2500 Snd Card");
MODULE_LICENSE("GPL");
