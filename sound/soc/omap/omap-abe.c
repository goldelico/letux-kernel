/*
 * omap-abe.c  --  OMAP ALSA SoC DAI driver using Audio Backend
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Contact: Misael Lopez Cruz <x0052729@ti.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <plat/control.h>
#include <plat/dma.h>
#include "mcpdm.h"
#include "omap-pcm.h"
#include "omap-abe.h"
#include "../codecs/abe/abe_main.h"

#define OMAP_ABE_FORMATS	(SNDRV_PCM_FMTBIT_S32_LE)

struct omap_mcpdm_data {
	struct omap_mcpdm_link *links;
	int active[2];
	int requested;
};

static struct omap_mcpdm_link omap_mcpdm_links[] = {
	/* downlink */
	{
		.irq_mask = DN_IRQ_EMTPY | DN_IRQ_FULL,
		.threshold = 1,
		.format = PDMOUTFORMAT_LJUST,
		.channels = PDM_DN_MASK | PDM_CMD_MASK,
	},
	/* uplink */
	{
		.irq_mask = UP_IRQ_EMPTY | UP_IRQ_FULL,
		.threshold = 1,
		.format = PDMOUTFORMAT_LJUST,
		.channels = PDM_UP1_EN | PDM_UP2_EN |
				PDM_DN_MASK | PDM_CMD_MASK,
	},
};

static struct omap_mcpdm_data mcpdm_data = {
	.links = omap_mcpdm_links,
	.active = {0},
	.requested = 0,
};

/*
 * Stream DMA parameters
 */
static struct omap_pcm_dma_data omap_abe_dai_dma_params[] = {
	{
		.name = "Audio downlink",
		.data_type = OMAP_DMA_DATA_TYPE_S32,
		.sync_mode = OMAP_DMA_SYNC_PACKET,
	},
	{
		.name = "Audio uplink",
		.data_type = OMAP_DMA_DATA_TYPE_S32,
		.sync_mode = OMAP_DMA_SYNC_PACKET,
	},
};

static int omap_abe_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	int err = 0;

	if (!mcpdm_priv->requested++)
		err = omap_mcpdm_request();

	return err;
}

static void omap_abe_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;

	if (!--mcpdm_priv->requested)
		omap_mcpdm_free();
}

static int omap_abe_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	int stream = substream->stream;
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!mcpdm_priv->active[stream]++)
			omap_mcpdm_start(stream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!--mcpdm_priv->active[stream])
			omap_mcpdm_stop(stream);
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

static int omap_abe_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int err, stream = substream->stream, dma_req;
	abe_dma_t dma_params;

	/* get abe dma data */
	switch (cpu_dai->id) {
	case OMAP_ABE_MM_DAI:
		if (stream) {
			abe_read_port_address(MM_UL2_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ4;
		} else {
			abe_read_port_address(MM_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ0;
		}
		break;
	case OMAP_ABE_TONES_DL_DAI:
		if (stream)
			return -EINVAL;
		else {
			abe_read_port_address(TONES_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ5;
		}
		break;
	case OMAP_ABE_VOICE_DAI:
		if (stream) {
			abe_read_port_address(VX_UL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ2;
		} else {
			abe_read_port_address(VX_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ1;
		}
		break;
	case OMAP_ABE_DIG_UPLINK_DAI:
		if (stream) {
			abe_read_port_address(MM_UL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ3;

		} else
			return -EINVAL;
		break;
	case OMAP_ABE_VIB_DAI:
		if (stream)
			return -EINVAL;
		else {
			abe_read_port_address(VIB_DL_PORT, &dma_params);
                        dma_req = OMAP44XX_DMA_ABE_REQ6;
		}
		break;
	default:
		return -EINVAL;
	}

	omap_abe_dai_dma_params[stream].dma_req = dma_req;
	omap_abe_dai_dma_params[stream].port_addr =
					(unsigned long)dma_params.data;
	omap_abe_dai_dma_params[stream].packet_size = dma_params.iter;
	cpu_dai->dma_data = &omap_abe_dai_dma_params[stream];

	if (stream)
		err = omap_mcpdm_set_uplink(&mcpdm_links[stream]);
	else
		err = omap_mcpdm_set_downlink(&mcpdm_links[stream]);

	return err;
}

static int omap_abe_dai_hw_free(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int err;

	if (substream->stream)
		err = omap_mcpdm_clr_uplink(&mcpdm_links[substream->stream]);
	else
		err = omap_mcpdm_clr_downlink(&mcpdm_links[substream->stream]);

	return err;
}

static struct snd_soc_dai_ops omap_abe_dai_ops = {
	.startup	= omap_abe_dai_startup,
	.shutdown	= omap_abe_dai_shutdown,
	.trigger	= omap_abe_dai_trigger,
	.hw_params	= omap_abe_dai_hw_params,
	.hw_free	= omap_abe_dai_hw_free,
};

struct snd_soc_dai omap_abe_dai[] = {
	{
		.name = "omap-abe-mm",
		.id = OMAP_ABE_MM_DAI,
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
	{
		.name = "omap-abe-tone-dl",
		.id = OMAP_ABE_TONES_DL_DAI,
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
	{
		.name = "omap-abe-voice",
		.id = OMAP_ABE_VOICE_DAI,
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = OMAP_ABE_FORMATS,
		},
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
	{
		.name = "omap-abe-dig-ul",
		.id = OMAP_ABE_DIG_UPLINK_DAI,
		.capture = {
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
	{
		.name = "omap-abe-vib",
		.id = OMAP_ABE_VIB_DAI,
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
};
EXPORT_SYMBOL_GPL(omap_abe_dai);

static int __init snd_omap_abe_init(void)
{
	return snd_soc_register_dais(omap_abe_dai, ARRAY_SIZE(omap_abe_dai));
}
module_init(snd_omap_abe_init);

static void __exit snd_omap_abe_exit(void)
{
	snd_soc_unregister_dais(omap_abe_dai, ARRAY_SIZE(omap_abe_dai));
}
module_exit(snd_omap_abe_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("OMAP ABE SoC Interface");
MODULE_LICENSE("GPL");
