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
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <plat/control.h>
#include <plat/dma-44xx.h>
#include <plat/dma.h>
#include "mcpdm.h"
#include "omap-pcm.h"
#include "omap-abe.h"
#include "../codecs/abe/abe_main.h"

#ifdef CONFIG_SND_OMAP_VOICE_TEST
#include "omap-mcbsp.h"
#include <plat/mcbsp.h>
#endif

#define OMAP_ABE_FORMATS	(SNDRV_PCM_FMTBIT_S32_LE)

#ifdef CONFIG_SND_OMAP_VOICE_TEST
static struct omap_pcm_dma_data omap_mcbsp_dai_dma_params[NUM_LINKS][2];

static const int omap44xx_dma_reqs[][2] = {
	{ OMAP44XX_DMA_MCBSP1_TX, OMAP44XX_DMA_MCBSP1_RX },
	{ OMAP44XX_DMA_MCBSP2_TX, OMAP44XX_DMA_MCBSP2_RX },
	{ OMAP44XX_DMA_MCBSP3_TX, OMAP44XX_DMA_MCBSP3_RX },
	{ OMAP44XX_DMA_MCBSP4_TX, OMAP44XX_DMA_MCBSP4_RX },
};

static const unsigned long omap44xx_mcbsp_port[][2] = {
	{ OMAP44XX_MCBSP1_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP44XX_MCBSP1_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP44XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP44XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP44XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP44XX_MCBSP3_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP44XX_MCBSP4_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP44XX_MCBSP4_BASE + OMAP_MCBSP_REG_DRR },
};
#endif

struct omap_mcpdm_data {
	struct omap_mcpdm_link *links;
	int active[2];
	int requested;
#ifdef CONFIG_SND_OMAP_VOICE_TEST
	int mcbsp_requested;
	struct omap_mcbsp_reg_cfg	regs;
	unsigned int fmt;
	unsigned int in_freq;
	int clk_div;
#endif
};

static struct omap_mcpdm_link omap_mcpdm_links[] = {
	/* downlink */
	{
		.irq_mask = MCPDM_DN_IRQ_EMPTY | MCPDM_DN_IRQ_FULL,
		.threshold = 2,
		.format = PDMOUTFORMAT_LJUST,
		.channels = PDM_DN_MASK | PDM_CMD_MASK,
	},
	/* uplink */
	{
		.irq_mask = MCPDM_UP_IRQ_EMPTY | MCPDM_UP_IRQ_FULL,
		.threshold = 2,
		.format = PDMOUTFORMAT_LJUST,
		.channels = PDM_UP1_EN | PDM_UP2_EN |
				PDM_DN_MASK | PDM_CMD_MASK,
	},
};

static struct omap_mcpdm_data mcpdm_data = {
	.links = omap_mcpdm_links,
	.active = {0},
	.requested = 0,
#ifdef CONFIG_SND_OMAP_VOICE_TEST
	.mcbsp_requested = 0,
	.clk_div = 0,
#endif
};

/*
 * Stream DMA parameters
 */
static struct omap_pcm_dma_data omap_abe_dai_dma_params[][2] = {
	/* Multimedia */
	{
		{
			.name = "multimedia playback",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
		{
			.name = "multimedia capture",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
	},
	/* Tones */
	{
		{
			.name = "tones playback",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
		{
			/* not used */
		},
	},
	/* Voice */
	{
		{
			.name = "voice playback",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
		{
			.name = "voice capture",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
	},
	/* Digital mic */
	{
		{
			/* not used */
		},
		{
			.name = "digital capture",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
	},
	/* Vibrator */
	{
		{
			.name = "vibrator playback",
			.data_type = OMAP_DMA_DATA_TYPE_S32,
			.sync_mode = OMAP_DMA_SYNC_PACKET,
		},
		{
			/* not used */
		},
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

static int omap_abe_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int stream = substream->stream;
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream) {
			if (!mcpdm_priv->active[stream]++) {
				err = omap_mcpdm_capture_open(&mcpdm_links[stream]);
				omap_mcpdm_start(stream);
			}
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		err = -EINVAL;
	}

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

static int omap_abe_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int err=0, stream = substream->stream, dma_req;
	int cpu_id = cpu_dai->id;
	abe_dma_t dma_params;

	/* get abe dma data */
	switch (cpu_id) {
	case OMAP_ABE_MM_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			abe_read_port_address(MM_UL2_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_4;
		} else {
			abe_read_port_address(MM_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_0;
		}
		break;
	case OMAP_ABE_TONES_DL_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			return -EINVAL;
		} else {
			abe_read_port_address(TONES_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_5;
		}
		break;
	case OMAP_ABE_VOICE_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			abe_read_port_address(VX_UL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_2;
		} else {
			abe_read_port_address(VX_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_1;
		}
		break;
	case OMAP_ABE_DIG_UPLINK_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			abe_read_port_address(MM_UL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_3;

		} else {
			return -EINVAL;
		}
		break;
	case OMAP_ABE_VIB_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			return -EINVAL;
		} else {
			abe_read_port_address(VIB_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_6;
		}
		break;
	default:
		return -EINVAL;
	}

	omap_abe_dai_dma_params[cpu_id][stream].dma_req = dma_req;
	omap_abe_dai_dma_params[cpu_id][stream].port_addr =
					(unsigned long)dma_params.data;
	omap_abe_dai_dma_params[cpu_id][stream].packet_size = dma_params.iter;
	cpu_dai->dma_data = &omap_abe_dai_dma_params[cpu_id][stream];

	if (!substream->stream) {
		if (!mcpdm_priv->active[stream]++) {
			err = omap_mcpdm_playback_open(&mcpdm_links[stream]);
			msleep(5);
			omap_mcpdm_start(stream);
		}
		/* Increment by 2 because 2 calls of HW free */
		mcpdm_priv->active[stream]++;
	}

	return err;
}

static int omap_abe_dai_hw_free(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int stream = substream->stream;
	int err = 0;

	if (mcpdm_priv->active[stream] == 1) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			err = omap_mcpdm_playback_close(&mcpdm_links[stream]);
			if (mcpdm_priv->active[0] == 0)
				err = omap_mcpdm_capture_close(&mcpdm_links[stream]);
		} else {
			err = omap_mcpdm_capture_close(&mcpdm_links[stream]);
		}
		omap_mcpdm_stop(stream);
		mcpdm_priv->active[stream] = 0;
	} else if (mcpdm_priv->active[stream] != 0) {
		mcpdm_priv->active[stream]--;
	}

	return err;
}

#ifdef CONFIG_SND_OMAP_VOICE_TEST
static int omap_abe_vx_dai_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	int err = 0, bus_id = 1, max_period, dma_op_mode;

	if (!mcpdm_priv->requested++) {
		err = omap_mcpdm_request();
	}
	if (!mcpdm_priv->mcbsp_requested++) {
		omap_mcbsp_request(bus_id);

		dma_op_mode = omap_mcbsp_get_dma_op_mode(bus_id);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			max_period = omap_mcbsp_get_max_rx_threshold(bus_id);
		else
			max_period = omap_mcbsp_get_max_tx_threshold(bus_id);

		max_period++;
		max_period <<= 1;

		if (dma_op_mode == MCBSP_DMA_MODE_THRESHOLD)
			snd_pcm_hw_constraint_minmax(substream->runtime,
				SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
							32, max_period);
	}

	return err;
}

static void omap_abe_vx_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	int bus_id = 1;

	if (!--mcpdm_priv->requested)
		omap_mcpdm_free();

	if (!--mcpdm_priv->mcbsp_requested)
		omap_mcbsp_free(bus_id);
}

static int omap_abe_vx_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int stream = substream->stream, bus_id = 1;
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream) {
			if (!mcpdm_priv->active[stream]++) {
				err = omap_mcpdm_capture_open(&mcpdm_links[stream]);
				omap_mcpdm_start(stream);
			}
			omap_mcbsp_start(bus_id, stream, !stream);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

static int omap_abe_vx_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int err = 0, stream = substream->stream, dma_req, dma, bus_id = 1, id = 1;
	int wlen, channels, wpf, sync_mode = OMAP_DMA_SYNC_ELEMENT;
	unsigned long port;
	abe_dma_t dma_params;
	struct omap_mcbsp_reg_cfg *regs = &mcpdm_priv->regs;
	unsigned int format, framesize, master, div;
	int cpu_id = cpu_dai->id;

	/* get abe dma data */
	switch (cpu_id) {
	case OMAP_ABE_VOICE_DAI:
		if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			abe_read_port_address(VX_UL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_2;
		} else {
			abe_read_port_address(VX_DL_PORT, &dma_params);
			dma_req = OMAP44XX_DMA_ABE_REQ_1;
		}
		break;
	default:
		return -EINVAL;
	}

	omap_abe_dai_dma_params[cpu_id][stream].dma_req = dma_req;
	omap_abe_dai_dma_params[cpu_id][stream].port_addr =
					(unsigned long)dma_params.data;
	omap_abe_dai_dma_params[cpu_id][stream].packet_size = dma_params.iter;
	cpu_dai->dma_data = &omap_abe_dai_dma_params[cpu_id][stream];

	dma = omap44xx_dma_reqs[bus_id][substream->stream];
	port = omap44xx_mcbsp_port[bus_id][substream->stream];

	omap_mcbsp_dai_dma_params[id][substream->stream].name =
	substream->stream ? "Audio Capture" : "Audio Playback";
	omap_mcbsp_dai_dma_params[id][substream->stream].dma_req = dma;
	omap_mcbsp_dai_dma_params[id][substream->stream].port_addr = port;
	omap_mcbsp_dai_dma_params[id][substream->stream].sync_mode = sync_mode;
	omap_mcbsp_dai_dma_params[id][substream->stream].data_type =
							OMAP_DMA_DATA_TYPE_S16;

	format = mcpdm_priv->fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* FIX-ME: Using mono format per default */
	wpf = channels = 1;
	if (channels == 2 && format == SND_SOC_DAIFMT_I2S) {
		/* Use dual-phase frames */
		regs->rcr2	|= RPHASE;
		regs->xcr2	|= XPHASE;
		/* Set 1 word per (McBSP) frame for phase1 and phase2 */
		wpf--;
		regs->rcr2	|= RFRLEN2(wpf - 1);
		regs->xcr2	|= XFRLEN2(wpf - 1);
	}

	regs->rcr1	|= RFRLEN1(wpf - 1);
	regs->xcr1	|= XFRLEN1(wpf - 1);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		/* Set word lengths */
		wlen = 16;
		regs->rcr2	|= RWDLEN2(OMAP_MCBSP_WORD_16);
		regs->rcr1	|= RWDLEN1(OMAP_MCBSP_WORD_16);
		regs->xcr2	|= XWDLEN2(OMAP_MCBSP_WORD_16);
		regs->xcr1	|= XWDLEN1(OMAP_MCBSP_WORD_16);
		break;
	default:
		/* Unsupported PCM format */
		return -EINVAL;
	}

	/* In McBSP master modes, FRAME (i.e. sample rate) is generated
	 * by _counting_ BCLKs. Calculate frame size in BCLKs */
	master = mcpdm_priv->fmt & SND_SOC_DAIFMT_MASTER_MASK;
	if (master == SND_SOC_DAIFMT_CBS_CFS) {
		div = mcpdm_priv->clk_div ? mcpdm_priv->clk_div : 1;
		framesize = (mcpdm_priv->in_freq / div) / params_rate(params);

		if (framesize < wlen * channels) {
			printk(KERN_ERR "%s: not enough bandwidth for desired rate and"
				"channels\n", __func__);
			return -EINVAL;
		}
	} else {
		framesize = wlen * channels;
	}

	/* Set FS period and length in terms of bit clock periods */
	switch (format) {
	case SND_SOC_DAIFMT_I2S:
		regs->srgr2	|= FPER(framesize - 1);
		regs->srgr1	|= FWID((framesize >> 1) - 1);
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		regs->srgr2	|= FPER(framesize - 1);
		regs->srgr1	|= FWID(0);
		break;
	}

	omap_mcbsp_config(bus_id, &mcpdm_priv->regs);
	if (!substream->stream) {
		if (!mcpdm_priv->active[stream]++) {
			err = omap_mcpdm_playback_open(&mcpdm_links[stream]);
			msleep(5);
			omap_mcpdm_start(stream);
		}
		omap_mcbsp_start(bus_id, stream, !stream);
		/* Increment by 2 because 2 calls of HW free */
		mcpdm_priv->active[stream]++;
	}

	return err;
}

static int omap_abe_vx_dai_hw_free(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcpdm_link *mcpdm_links = mcpdm_priv->links;
	int stream = substream->stream;
	int err=0, bus_id = 1;

	if (mcpdm_priv->active[stream] == 1) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			err = omap_mcpdm_playback_close(&mcpdm_links[stream]);
			if (mcpdm_priv->active[0] == 0)
				err = omap_mcpdm_capture_close(&mcpdm_links[stream]);
		}
		else {
			if (mcpdm_priv->active[1] == 0)
				err = omap_mcpdm_capture_close(&mcpdm_links[stream]);
		}
		omap_mcpdm_stop(stream);
		/* Stop McBSP */
		omap_mcbsp_stop(bus_id, !stream, stream);
		mcpdm_priv->active[stream] = 0;
	} else if (mcpdm_priv->active[stream] != 0) {
		mcpdm_priv->active[stream]--;
	}

	return err;
}

/*
 * This must be called before _set_clkdiv and _set_sysclk since McBSP register
 * cache is initialized here
 */
static int omap_abe_mcbsp_dai_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				      unsigned int fmt)
{
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcbsp_reg_cfg *regs = &mcpdm_priv->regs;
	unsigned int temp_fmt = fmt;

	mcpdm_priv->fmt = fmt;
	memset(regs, 0, sizeof(*regs));
	/* Generic McBSP register settings */
	regs->spcr2	|= XINTM(3) | FREE;
	regs->spcr1	|= RINTM(3);

	if (cpu_is_omap2430() || cpu_is_omap34xx() || cpu_is_omap44xx()) {
		regs->xccr = DXENDLY(1) | XDMAEN;
		regs->rccr = RFULL_CYCLE | RDMAEN;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* 1-bit data delay */
		regs->rcr2	|= RDATDLY(1);
		regs->xcr2	|= XDATDLY(1);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* 1-bit data delay */
		regs->rcr2      |= RDATDLY(1);
		regs->xcr2      |= XDATDLY(1);
		/* Invert FS polarity configuration */
		temp_fmt ^= SND_SOC_DAIFMT_NB_IF;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* 0-bit data delay */
		regs->rcr2      |= RDATDLY(0);
		regs->xcr2      |= XDATDLY(0);
		/* Invert FS polarity configuration */
		temp_fmt ^= SND_SOC_DAIFMT_NB_IF;
		break;
	default:
		/* Unsupported data format */
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* McBSP master. Set FS and bit clocks as outputs */
		regs->pcr0	|= FSXM | FSRM |
				   CLKXM | CLKRM;
		/* Sample rate generator drives the FS */
		regs->srgr2	|= FSGM;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* McBSP slave */
		break;
	default:
		/* Unsupported master/slave configuration */
		return -EINVAL;
	}

	/* Set bit clock (CLKX/CLKR) and FS polarities */
	switch (temp_fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/*
		 * Normal BCLK + FS.
		 * FS active low. TX data driven on falling edge of bit clock
		 * and RX data sampled on rising edge of bit clock.
		 */
		regs->pcr0	|= FSXP | FSRP |
				   CLKXP | CLKRP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		regs->pcr0	|= CLKXP | CLKRP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		regs->pcr0	|= FSXP | FSRP;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int omap_abe_mcbsp_dai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
					 int clk_id, unsigned int freq,
					 int dir)
{
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcbsp_reg_cfg *regs = &mcpdm_priv->regs;
	int err = 0;

	mcpdm_priv->in_freq = freq;
	switch (clk_id) {
	case OMAP_MCBSP_SYSCLK_CLK:
		regs->srgr2	|= CLKSM;
		break;
	case OMAP_MCBSP_SYSCLK_CLKS_FCLK:
		if (cpu_is_omap44xx()) {
			regs->srgr2     |= CLKSM;
			break;
		}
	default:
		err = -ENODEV;
	}

	return err;
}

static int omap_abe_mcbsp_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
						int div_id, int div)
{
	struct omap_mcpdm_data *mcpdm_priv = cpu_dai->private_data;
	struct omap_mcbsp_reg_cfg *regs = &mcpdm_priv->regs;

	if (div_id != OMAP_MCBSP_CLKGDV)
		return -ENODEV;

	regs->srgr1 |= CLKGDV(div - 1);

	return 0;
}
#endif

static struct snd_soc_dai_ops omap_abe_dai_ops = {
	.startup	= omap_abe_dai_startup,
	.shutdown	= omap_abe_dai_shutdown,
	.trigger	= omap_abe_dai_trigger,
	.hw_params	= omap_abe_dai_hw_params,
	.hw_free	= omap_abe_dai_hw_free,
};

static struct snd_soc_dai_ops omap_abe_vx_dai_ops = {
#ifdef CONFIG_SND_OMAP_VOICE_TEST
	.startup	= omap_abe_vx_dai_startup,
	.shutdown	= omap_abe_vx_dai_shutdown,
	.hw_params	= omap_abe_vx_dai_hw_params,
	.hw_free	= omap_abe_vx_dai_hw_free,
	.trigger	= omap_abe_vx_dai_trigger,
	.set_fmt	= omap_abe_mcbsp_dai_set_dai_fmt,
	.set_sysclk	= omap_abe_mcbsp_dai_set_dai_sysclk,
	.set_clkdiv	= omap_abe_mcbsp_dai_set_clkdiv,
#else
	.startup	= omap_abe_dai_startup,
	.shutdown	= omap_abe_dai_shutdown,
	.trigger	= omap_abe_dai_trigger,
	.hw_params	= omap_abe_dai_hw_params,
	.hw_free	= omap_abe_dai_hw_free,
#endif
};

struct snd_soc_dai omap_abe_dai[] = {
	{
		.name = "omap-abe-mm",
		.id = OMAP_ABE_MM_DAI,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.capture = {
			.channels_min = 1,
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
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_dai_ops,
		.private_data = &mcpdm_data,
	},
	{
		.name = "omap-abe-voice",
		.id = OMAP_ABE_VOICE_DAI,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = OMAP_ABE_FORMATS,
		},
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = OMAP_ABE_FORMATS,
		},
		.ops = &omap_abe_vx_dai_ops,
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
