/*
 * ALSA SoC HMDI codec driver for OMAP5
 *
 * Author: Ricardo Neri <ricardo.neri@ti.com> with portions of
 * code by Axel Castañeda
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <plat/omap_hwmod.h>
#include <video/omapdss.h>

#include "../../../drivers/video/omap2/dss/dss_features.h"
#include "../../../drivers/video/omap2/dss/dss.h"
#include "../../../drivers/video/omap2/dss/ti_hdmi_5xxx_ip.h"
#include "../../../drivers/video/omap2/dss/ti_hdmi.h"

#define HDMI_WP		0x0
#define HDMI_CORE_SYS	0x20000
#define HDMI_CORE_AV	0x30000

/* codec private data */
struct hdmi_data {
	struct hdmi_audio_format audio_fmt;
	struct hdmi_audio_dma audio_dma;
	struct hdmi_core_audio_config audio_core_cfg;
	struct hdmi_core_infoframe_audio aud_if_cfg;
	struct hdmi_ip_data ip_data;
	struct omap_hwmod *oh;
};

static int hdmi_compute_acr_params(u32 sample_freq, u32 color_mode,
		u32 pclk, u32 *n, u32 *cts)
{
	u32 deep_color = 0;

	if (n == NULL || cts == NULL)
		return -EINVAL;
	/*
	 * Color mode configuration is needed
	 * to calculate the TMDS clock based on the pixel clock.
	 */
	switch (color_mode) {
	case HDMI_DEEP_COLOR_24BIT:
		deep_color = 100;
		break;
	case HDMI_DEEP_COLOR_30BIT:
		deep_color = 125;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		deep_color = 150;
		break;
	default:
		return -EINVAL;
	}

	switch (sample_freq) {
	case 32000:
		if ((deep_color == 125) && ((pclk == 54054)
				|| (pclk == 74250)))
			*n = 8192;
		else
			*n = 4096;
		break;
	case 44100:
		*n = 6272;
		break;
	case 48000:
		if ((deep_color == 125) && ((pclk == 54054)
				|| (pclk == 74250)))
			*n = 8192;
		else
			*n = 6144;
		break;
	default:
		*n = 0;
		return -EINVAL;
	}

	/* Calculate CTS. See HDMI 1.3a or 1.4a specifications */
	*cts = pclk * (*n / 128) * deep_color / (sample_freq / 10);

	return 0;
}

static int hdmi_audio_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct platform_device *pdev = to_platform_device(codec->dev);
	struct hdmi_data *priv = snd_soc_codec_get_drvdata(codec);
	struct hdmi_audio_format *audio_format = &priv->audio_fmt;
	struct hdmi_audio_dma *audio_dma = &priv->audio_dma;
	struct hdmi_core_audio_config *core_cfg = &priv->audio_core_cfg;
	struct hdmi_core_infoframe_audio *aud_if_cfg = &priv->aud_if_cfg;
	struct omap_overlay_manager *mgr = NULL;
	enum hdmi_core_audio_sample_freq sample_freq;
	int err, n, cts, i;
	u32 pclk;
	u32 color_mode = omapdss_hdmi_get_deepcolor();

	/* Obtain pixel clock from DSS data */
	for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
		mgr = omap_dss_get_overlay_manager(i);
		if (mgr && mgr->device
			&& mgr->device->type == OMAP_DISPLAY_TYPE_HDMI)
			break;
	}

	if (i == omap_dss_get_num_overlay_managers()) {
		dev_err(&pdev->dev, "HDMI display device not found!\n");
		return -ENODEV;
	}

	pclk = mgr->device->panel.timings.pixel_clock;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_format->samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format->sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format->justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_dma->transfer_size = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		audio_format->samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format->sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format->justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		core_cfg->i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		audio_dma->transfer_size = 0x20;
		break;
	default:
		return -EINVAL;
	}

	/* OMAP5 ES1.0 supports only 44100Hz */
	if (params_rate(params) != 44100) {
		dev_err(&pdev->dev, "Unsupported sample rate.\n");
		return -EINVAL;
	}

	sample_freq = HDMI_AUDIO_FS_44100;


	err = hdmi_compute_acr_params(params_rate(params), color_mode, pclk,
		&n, &cts);

	if (err < 0)
		return err;

	/* Audio wrapper config */
	audio_format->type = HDMI_AUDIO_TYPE_LPCM;
	audio_format->sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;
	/* Disable start/stop signals of IEC 60958 blocks */
	audio_format->en_sig_blk_strt_end = HDMI_AUDIO_BLOCK_SIG_STARTEND_OFF;

	audio_dma->block_size = 0xC0;
	audio_dma->mode = HDMI_AUDIO_TRANSF_DMA;
	audio_dma->fifo_threshold = 0x20; /* in number of samples */

	hdmi_wp_audio_config_dma(&priv->ip_data, audio_dma);
	hdmi_ti_5xxx_wp_audio_config_format(&priv->ip_data, audio_format);

	/* Core audio config */
	core_cfg->freq_sample = sample_freq;
	core_cfg->n = n;
	core_cfg->cts = cts;

	/* Only 2 channels two channels are supported atm.*/
	if (params_channels(params) != 2) {
		dev_err(&pdev->dev, "Unsupported number of channels\n");
		return -EINVAL;
	}

	core_cfg->layout = HDMI_AUDIO_LAYOUT_2CH;

	hdmi_ti_5xxx_core_audio_config(&priv->ip_data, core_cfg);
	hdmi_ti_5xxx_wp_audio_config_format(&priv->ip_data, audio_format);

	/*
	 * Configure packet
	 * info frame audio see doc CEA861-D page 74
	 */
	aud_if_cfg->db1_coding_type = HDMI_INFOFRAME_AUDIO_DB1CT_FROM_STREAM;
	aud_if_cfg->db1_channel_count = params_channels(params);
	aud_if_cfg->db2_sample_freq = HDMI_INFOFRAME_AUDIO_DB2SF_FROM_STREAM;
	aud_if_cfg->db2_sample_size = HDMI_INFOFRAME_AUDIO_DB2SS_FROM_STREAM;
	aud_if_cfg->db4_channel_alloc = 0;
	aud_if_cfg->db5_downmix_inh = false;
	aud_if_cfg->db5_lsv = 0;

	hdmi_ti_5xxx_core_audio_infoframe_config(&priv->ip_data, aud_if_cfg);
	return 0;
}

static int hdmi_audio_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct hdmi_data *priv = snd_soc_codec_get_drvdata(codec);
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		omap_hwmod_set_slave_idlemode(priv->oh,
			HWMOD_IDLEMODE_NO);
		hdmi_ti_5xxx_audio_enable(&priv->ip_data, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hdmi_ti_5xxx_audio_enable(&priv->ip_data, 0);
		omap_hwmod_set_slave_idlemode(priv->oh,
			HWMOD_IDLEMODE_SMART_WKUP);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static int hdmi_audio_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	if (omapdss_hdmi_get_hdmi_mode() != HDMI_HDMI) {
		pr_err("Current video settings do not support audio.\n");
		return -EIO;
	}
	return 0;
}
static int hdmi_probe(struct snd_soc_codec *codec)
{
	struct hdmi_data *priv;
	struct platform_device *pdev = to_platform_device(codec->dev);
	struct resource *hdmi_rsrc;

	priv = kzalloc(sizeof(struct hdmi_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	snd_soc_codec_set_drvdata(codec, priv);


	hdmi_rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!hdmi_rsrc) {
		dev_err(&pdev->dev, "Cannot obtain IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	priv->oh = omap_hwmod_lookup("dss_hdmi");

	if (!priv->oh) {
		dev_err(&pdev->dev, "can't find omap_hwmod for hdmi\n");
			return -ENODEV;
	}

	/* Base address taken from platform */
	priv->ip_data.base_wp = ioremap(hdmi_rsrc->start,
					resource_size(hdmi_rsrc));

	if (!priv->ip_data.base_wp) {
		dev_err(&pdev->dev, "can't ioremap WP\n");
		return -ENOMEM;
	}

	priv->ip_data.core_sys_offset = HDMI_CORE_SYS;
	priv->ip_data.core_av_offset = HDMI_CORE_AV;

	return 0;
}

static int hdmi_remove(struct snd_soc_codec *codec)
{
	struct hdmi_data *priv = snd_soc_codec_get_drvdata(codec);
	iounmap(priv->ip_data.base_wp);
	kfree(priv);
	return 0;
}


static struct snd_soc_codec_driver hdmi_audio_codec_drv = {
	.probe = hdmi_probe,
	.remove = hdmi_remove,
};

static struct snd_soc_dai_ops hdmi_audio_codec_ops = {
	.hw_params = hdmi_audio_hw_params,
	.trigger = hdmi_audio_trigger,
	.startup = hdmi_audio_startup,
};

static struct snd_soc_dai_driver hdmi_codec_dai_drv = {
		.name = "hdmi-audio-codec",
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &hdmi_audio_codec_ops,
};

static __devinit int hdmi_codec_probe(struct platform_device *pdev)
{
	int r;

	/* Register ASoC codec DAI */
	r = snd_soc_register_codec(&pdev->dev, &hdmi_audio_codec_drv,
					&hdmi_codec_dai_drv, 1);
	if (r) {
		dev_err(&pdev->dev, "can't register ASoC HDMI audio codec\n");
		return r;
	}

	return 0;
}

static int __devexit hdmi_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}


static struct platform_driver hdmi_codec_driver = {
	.probe          = hdmi_codec_probe,
	.remove         = __devexit_p(hdmi_codec_remove),
	.driver         = {
		.name   = "omap-hdmi-codec",
		.owner  = THIS_MODULE,
	},
};


static int __init hdmi_codec_init(void)
{
	return platform_driver_register(&hdmi_codec_driver);
}
module_init(hdmi_codec_init);

static void __exit hdmi_codec_exit(void)
{
	platform_driver_unregister(&hdmi_codec_driver);
}
module_exit(hdmi_codec_exit);


MODULE_AUTHOR("Ricardo Neri <ricardo.neri@ti.com>");
MODULE_DESCRIPTION("ASoC OMAP5 HDMI codec driver");
MODULE_LICENSE("GPL");
