/*
 * omap-mcbsp.c  --  OMAP ALSA SoC DAI driver using McBSP port
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Contact: Jarkko Nikula <jhnikula@gmail.com>
 *          Peter Ujfalusi <peter.ujfalusi@nokia.com>
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
#include <plat/mcbsp.h>
#include "omap-mcbsp.h"
#include "omap-pcm.h"

#define OMAP_MCBSP_RATES	(SNDRV_PCM_RATE_8000_96000)

struct omap_mcbsp_data {
	unsigned int			bus_id;
	struct omap_mcbsp_reg_cfg	regs;
	unsigned int			fmt;
	int				clk_id;
	/*
	 * Flags indicating is the bus already configured by
	 * another substream
	 */
	int		configured;
	int		tx_active;
	int		rx_active;
	int		wlen;
};

#define to_mcbsp(priv)	container_of((priv), struct omap_mcbsp_data, bus_id)

static struct omap_mcbsp_data mcbsp_data[NUM_LINKS];

/*
 * Stream DMA parameters. DMA request line and port address are set runtime
 * since they are different between OMAP1 and later OMAPs
 */
static struct omap_pcm_dma_data omap_mcbsp_dai_dma_params[NUM_LINKS][2];

#if defined(CONFIG_ARCH_OMAP15XX) || defined(CONFIG_ARCH_OMAP16XX)
static const int omap1_dma_reqs[][2] = {
	{ OMAP_DMA_MCBSP1_TX, OMAP_DMA_MCBSP1_RX },
	{ OMAP_DMA_MCBSP2_TX, OMAP_DMA_MCBSP2_RX },
	{ OMAP_DMA_MCBSP3_TX, OMAP_DMA_MCBSP3_RX },
};
static const unsigned long omap1_mcbsp_port[][2] = {
	{ OMAP1510_MCBSP1_BASE + OMAP_MCBSP_REG_DXR1,
	  OMAP1510_MCBSP1_BASE + OMAP_MCBSP_REG_DRR1 },
	{ OMAP1510_MCBSP2_BASE + OMAP_MCBSP_REG_DXR1,
	  OMAP1510_MCBSP2_BASE + OMAP_MCBSP_REG_DRR1 },
	{ OMAP1510_MCBSP3_BASE + OMAP_MCBSP_REG_DXR1,
	  OMAP1510_MCBSP3_BASE + OMAP_MCBSP_REG_DRR1 },
};
#else
static const int omap1_dma_reqs[][2] = {};
static const unsigned long omap1_mcbsp_port[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP24XX) || defined(CONFIG_ARCH_OMAP34XX)
static const int omap24xx_dma_reqs[][2] = {
	{ OMAP24XX_DMA_MCBSP1_TX, OMAP24XX_DMA_MCBSP1_RX },
	{ OMAP24XX_DMA_MCBSP2_TX, OMAP24XX_DMA_MCBSP2_RX },
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	{ OMAP24XX_DMA_MCBSP3_TX, OMAP24XX_DMA_MCBSP3_RX },
	{ OMAP24XX_DMA_MCBSP4_TX, OMAP24XX_DMA_MCBSP4_RX },
	{ OMAP24XX_DMA_MCBSP5_TX, OMAP24XX_DMA_MCBSP5_RX },
#endif
};
#else
static const int omap24xx_dma_reqs[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP4)
static const int omap44xx_dma_reqs[][2] = {
	{ OMAP44XX_DMA_MCBSP1_TX, OMAP44XX_DMA_MCBSP1_RX },
	{ OMAP44XX_DMA_MCBSP2_TX, OMAP44XX_DMA_MCBSP2_RX },
	{ OMAP44XX_DMA_MCBSP3_TX, OMAP44XX_DMA_MCBSP3_RX },
	{ OMAP44XX_DMA_MCBSP4_TX, OMAP44XX_DMA_MCBSP4_RX },
};
#else
static const int omap44xx_dma_reqs[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP2420)
static const unsigned long omap2420_mcbsp_port[][2] = {
	{ OMAP24XX_MCBSP1_BASE + OMAP_MCBSP_REG_DXR1,
	  OMAP24XX_MCBSP1_BASE + OMAP_MCBSP_REG_DRR1 },
	{ OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR1,
	  OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR1 },
};
#else
static const unsigned long omap2420_mcbsp_port[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP2430)
static const unsigned long omap2430_mcbsp_port[][2] = {
	{ OMAP24XX_MCBSP1_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP24XX_MCBSP1_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP2430_MCBSP3_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP2430_MCBSP3_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP2430_MCBSP4_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP2430_MCBSP4_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP2430_MCBSP5_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP2430_MCBSP5_BASE + OMAP_MCBSP_REG_DRR },
};
#else
static const unsigned long omap2430_mcbsp_port[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP34XX)
static const unsigned long omap34xx_mcbsp_port[][2] = {
	{ OMAP34XX_MCBSP1_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP34XX_MCBSP1_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP34XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP34XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP34XX_MCBSP4_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP34XX_MCBSP4_BASE + OMAP_MCBSP_REG_DRR },
	{ OMAP34XX_MCBSP5_BASE + OMAP_MCBSP_REG_DXR,
	  OMAP34XX_MCBSP5_BASE + OMAP_MCBSP_REG_DRR },
};
#else
static const unsigned long omap34xx_mcbsp_port[][2] = {};
#endif

#if defined(CONFIG_ARCH_OMAP4)
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
#else
static const unsigned long omap44xx_mcbsp_port[][2] = {};
#endif

static int omap_mcbsp_dai_set_clks_src(struct omap_mcbsp_data *mcbsp_data,
					int clk_id);

static void omap_mcbsp_set_threshold(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	struct omap_pcm_dma_data *dma_data;
	int dma_op_mode = omap_mcbsp_get_dma_op_mode(mcbsp_data->bus_id);
	int words;

	dma_data = rtd->dai->cpu_dai->dma_data;

	/* TODO: Currently, MODE_ELEMENT == MODE_FRAME */
	if (dma_op_mode == MCBSP_DMA_MODE_THRESHOLD)
		/*
		 * Configure McBSP threshold based on either:
		 * packet_size, when the sDMA is in packet mode, or
		 * based on the period size.
		 */
		if (dma_data->packet_size)
			words = dma_data->packet_size;
		else
			words = snd_pcm_lib_period_bytes(substream) /
							(mcbsp_data->wlen / 8);
	else
		words = 1;

	/* Configure McBSP internal buffer usage */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		omap_mcbsp_set_tx_threshold(mcbsp_data->bus_id, words);
	else
		omap_mcbsp_set_rx_threshold(mcbsp_data->bus_id, words);
}

static int omap_mcbsp_hwrule_min_buffersize(struct snd_pcm_hw_params *params,
				    struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *buffer_size = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct omap_mcbsp_data *mcbsp_data = rule->private;
	struct snd_interval frames;
	int size;

	snd_interval_any(&frames);
	size = omap_mcbsp_get_fifo_size(mcbsp_data->bus_id);

	frames.min = size / channels->min;
	frames.integer = 1;
	return snd_interval_refine(buffer_size, &frames);
}

static int omap_mcbsp_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	int bus_id = mcbsp_data->bus_id;
	int err = 0;

	/*
	 * OMAP3 McBSP FIFO is word structured.
	 * McBSP2 has 1024 + 256 = 1280 word long buffer,
	 * McBSP1,3,4,5 has 128 word long buffer
	 * This means that the size of the FIFO depends on the sample format.
	 * For example on McBSP3:
	 * 16bit samples: size is 128 * 2 = 256 bytes
	 * 32bit samples: size is 128 * 4 = 512 bytes
	 * It is simpler to place constraint for buffer and period based on
	 * channels.
	 * McBSP3 as example again (16 or 32 bit samples):
	 * 1 channel (mono): size is 128 frames (128 words)
	 * 2 channels (stereo): size is 128 / 2 = 64 frames (2 * 64 words)
	 * 4 channels: size is 128 / 4 = 32 frames (4 * 32 words)
	 */
	if (cpu_is_omap343x() || cpu_is_omap44xx()) {
		/*
		* Rule for the buffer size. We should not allow
		* smaller buffer than the FIFO size to avoid underruns
		*/
		snd_pcm_hw_rule_add(substream->runtime, 0,
				    SNDRV_PCM_HW_PARAM_CHANNELS,
				    omap_mcbsp_hwrule_min_buffersize,
				    mcbsp_data,
				    SNDRV_PCM_HW_PARAM_BUFFER_SIZE, -1);

		/* Make sure, that the period size is always even */
		snd_pcm_hw_constraint_step(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 2);
	}

	if (!cpu_dai->active) {
		err = omap_mcbsp_request(bus_id);
		cpu_dai->active = 1;
	}

	return err;
}

static void omap_mcbsp_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);

	if (!cpu_dai->active) {
		omap_mcbsp_free(mcbsp_data->bus_id);
		cpu_dai->active = 0;
		mcbsp_data->configured = 0;
	}
}

static int omap_mcbsp_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	int err = 0, play = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (cpu_dai->active)
			omap_mcbsp_dai_set_clks_src(mcbsp_data,
					mcbsp_data->clk_id);
		omap_mcbsp_start(mcbsp_data->bus_id, play, !play);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mcbsp_data->tx_active = 1;
		else
			mcbsp_data->rx_active = 1;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (cpu_dai->active)
			omap_mcbsp_dai_set_clks_src(mcbsp_data,
				OMAP_MCBSP_SYSCLK_CLKS_FCLK);
		omap_mcbsp_stop(mcbsp_data->bus_id, play, !play);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mcbsp_data->tx_active = 0;
		else
			mcbsp_data->rx_active = 0;
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

static int omap_mcbsp_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	struct omap_mcbsp_reg_cfg *regs = &mcbsp_data->regs;
	struct omap_pcm_dma_data *dma_data;
	int dma, bus_id = mcbsp_data->bus_id;
	int wlen, channels, wpf, sync_mode = OMAP_DMA_SYNC_ELEMENT;
	int pkt_size = 0;
	unsigned long port;
	unsigned int format;

	dma_data = &omap_mcbsp_dai_dma_params[cpu_dai->id][substream->stream];
	if (cpu_class_is_omap1()) {
		dma = omap1_dma_reqs[bus_id][substream->stream];
		port = omap1_mcbsp_port[bus_id][substream->stream];
	} else if (cpu_is_omap2420()) {
		dma = omap24xx_dma_reqs[bus_id][substream->stream];
		port = omap2420_mcbsp_port[bus_id][substream->stream];
	} else if (cpu_is_omap2430()) {
		dma = omap24xx_dma_reqs[bus_id][substream->stream];
		port = omap2430_mcbsp_port[bus_id][substream->stream];
	} else if (cpu_is_omap343x()) {
		dma = omap24xx_dma_reqs[bus_id][substream->stream];
		port = omap34xx_mcbsp_port[bus_id][substream->stream];
	} else if (cpu_is_omap44xx()) {
		dma = omap44xx_dma_reqs[bus_id][substream->stream];
		port = omap44xx_mcbsp_port[bus_id][substream->stream];
	} else {
		return -ENODEV;
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dma_data->data_type = OMAP_DMA_DATA_TYPE_S16;
		wlen = 16;
		break;
	default:
		return -EINVAL;
	}
	if (cpu_is_omap343x()) {
		dma_data->set_threshold = omap_mcbsp_set_threshold;
		/* TODO: Currently, MODE_ELEMENT == MODE_FRAME */
		if (omap_mcbsp_get_dma_op_mode(bus_id) ==
						MCBSP_DMA_MODE_THRESHOLD) {
			int period_words, max_thrsh;

			period_words = params_period_bytes(params) / (wlen / 8);
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				max_thrsh = omap_mcbsp_get_max_tx_threshold(
							    mcbsp_data->bus_id);
			else
				max_thrsh = omap_mcbsp_get_max_rx_threshold(
							    mcbsp_data->bus_id);
			/*
			 * If the period contains less or equal number of words,
			 * we are using the original threshold mode setup:
			 * McBSP threshold = sDMA frame size = period_size
			 * Otherwise we switch to sDMA packet mode:
			 * McBSP threshold = sDMA packet size
			 * sDMA frame size = period size
			 */
			if (period_words > max_thrsh) {
				int divider = 0;

				/*
				 * Look for the biggest threshold value, which
				 * divides the period size evenly.
				 */
				divider = period_words / max_thrsh;
				if (period_words % max_thrsh)
					divider++;
				while (period_words % divider &&
					divider < period_words)
					divider++;
				if (divider == period_words)
					return -EINVAL;

				pkt_size = period_words / divider;
				sync_mode = OMAP_DMA_SYNC_PACKET;
			} else {
				sync_mode = OMAP_DMA_SYNC_FRAME;
			}

			/* Wake enable for threshold mode */
			regs->wken = XRDYEN | RRDYEN;
		}
	}

	dma_data->name = substream->stream ? "Audio Capture" : "Audio Playback";
	dma_data->dma_req = dma;
	dma_data->port_addr = port;
	dma_data->sync_mode = sync_mode;
	dma_data->packet_size = pkt_size;

	cpu_dai->dma_data = dma_data;

	if (mcbsp_data->configured) {
		/* McBSP already configured by another stream */
		return 0;
	}

	format = mcbsp_data->fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	wpf = channels = params_channels(params);
	switch (channels) {
	case 2:
		if (format == SND_SOC_DAIFMT_I2S) {
			/* Use dual-phase frames */
			regs->rcr2	|= RPHASE;
			regs->xcr2	|= XPHASE;
			/* Set 1 word per (McBSP) frame for phase1 and phase2 */
			wpf--;
			regs->rcr2	|= RFRLEN2(wpf - 1);
			regs->xcr2	|= XFRLEN2(wpf - 1);
		}
	case 1:
	case 4:
		/* Set word per (McBSP) frame for phase1 */
		regs->rcr1	|= RFRLEN1(wpf - 1);
		regs->xcr1	|= XFRLEN1(wpf - 1);
		break;
	default:
		/* Unsupported number of channels */
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		/* Set word lengths */
		regs->rcr2	|= RWDLEN2(OMAP_MCBSP_WORD_16);
		regs->rcr1	|= RWDLEN1(OMAP_MCBSP_WORD_16);
		regs->xcr2	|= XWDLEN2(OMAP_MCBSP_WORD_16);
		regs->xcr1	|= XWDLEN1(OMAP_MCBSP_WORD_16);
		break;
	default:
		/* Unsupported PCM format */
		return -EINVAL;
	}

	/* Set FS period and length in terms of bit clock periods */
	switch (format) {
	case SND_SOC_DAIFMT_I2S:
		regs->srgr2	|= FPER(wlen * channels - 1);
		regs->srgr1	|= FWID(wlen - 1);
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		regs->srgr2	|= FPER(wlen * channels - 1);
		regs->srgr1	|= FWID(0);
		break;
	}

	omap_mcbsp_config(bus_id, &mcbsp_data->regs);
	mcbsp_data->wlen = wlen;
	mcbsp_data->configured = 1;

	return 0;
}

/*
 * This must be called before _set_clkdiv and _set_sysclk since McBSP register
 * cache is initialized here
 */
static int omap_mcbsp_dai_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				      unsigned int fmt)
{
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	struct omap_mcbsp_reg_cfg *regs = &mcbsp_data->regs;
	unsigned int temp_fmt = fmt;

	if (mcbsp_data->configured)
		return 0;

	mcbsp_data->fmt = fmt;
	memset(regs, 0, sizeof(*regs));
	/* Generic McBSP register settings */
	regs->spcr2	|= XINTM(3) | FREE;
	regs->spcr1	|= RINTM(3);
	/* RFIG and XFIG are not defined in 34xx */
	if (!cpu_is_omap34xx() && !cpu_is_omap44xx()) {
		regs->rcr2	|= RFIG;
		regs->xcr2	|= XFIG;
	}
	if (cpu_is_omap2430() || cpu_is_omap34xx() || cpu_is_omap44xx()) {
		regs->xccr = DXENDLY(1) | XDMAEN | XDISABLE;
		regs->rccr = RFULL_CYCLE | RDMAEN | RDISABLE;
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

static int omap_mcbsp_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
				     int div_id, int div)
{
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	struct omap_mcbsp_reg_cfg *regs = &mcbsp_data->regs;

	if (div_id != OMAP_MCBSP_CLKGDV)
		return -ENODEV;

	regs->srgr1	|= CLKGDV(div - 1);

	return 0;
}

static int omap_mcbsp_dai_set_clks_src(struct omap_mcbsp_data *mcbsp_data,
				       int clk_id)
{
	int sel_bit;
	u16 reg, reg_devconf1 = OMAP243X_CONTROL_DEVCONF1;

	if (cpu_class_is_omap1()) {
		/* OMAP1's can use only external source clock */
		if (unlikely(clk_id == OMAP_MCBSP_SYSCLK_CLKS_FCLK))
			return -EINVAL;
		else
			return 0;
	}

	if (cpu_is_omap2420() && mcbsp_data->bus_id > 1)
		return -EINVAL;

	if (cpu_is_omap343x())
		reg_devconf1 = OMAP343X_CONTROL_DEVCONF1;

	switch (mcbsp_data->bus_id) {
	case 0:
		reg = OMAP2_CONTROL_DEVCONF0;
		sel_bit = 2;
		break;
	case 1:
		reg = OMAP2_CONTROL_DEVCONF0;
		sel_bit = 6;
		break;
	case 2:
		reg = reg_devconf1;
		sel_bit = 0;
		break;
	case 3:
		reg = reg_devconf1;
		sel_bit = 2;
		break;
	case 4:
		reg = reg_devconf1;
		sel_bit = 4;
		break;
	default:
		return -EINVAL;
	}

	if (clk_id == OMAP_MCBSP_SYSCLK_CLKS_FCLK)
		omap_ctrl_writel(omap_ctrl_readl(reg) & ~(1 << sel_bit), reg);
	else
		omap_ctrl_writel(omap_ctrl_readl(reg) | (1 << sel_bit), reg);

	return 0;
}

static int omap_mcbsp_dai_set_rcvr_src(struct omap_mcbsp_data *mcbsp_data,
				       int clk_id)
{
	int sel_bit, set = 0;
	u16 reg = OMAP2_CONTROL_DEVCONF0;

	if (cpu_class_is_omap1())
		return -EINVAL; /* TODO: Can this be implemented for OMAP1? */
	if (mcbsp_data->bus_id != 0)
		return -EINVAL;

	switch (clk_id) {
	case OMAP_MCBSP_CLKR_SRC_CLKX:
		set = 1;
	case OMAP_MCBSP_CLKR_SRC_CLKR:
		sel_bit = 3;
		break;
	case OMAP_MCBSP_FSR_SRC_FSX:
		set = 1;
	case OMAP_MCBSP_FSR_SRC_FSR:
		sel_bit = 4;
		break;
	default:
		return -EINVAL;
	}

	if (set)
		omap_ctrl_writel(omap_ctrl_readl(reg) | (1 << sel_bit), reg);
	else
		omap_ctrl_writel(omap_ctrl_readl(reg) & ~(1 << sel_bit), reg);

	return 0;
}

static int omap_mcbsp_dai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
					 int clk_id, unsigned int freq,
					 int dir)
{
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);
	struct omap_mcbsp_reg_cfg *regs = &mcbsp_data->regs;
	int err = 0;

	switch (clk_id) {
	case OMAP_MCBSP_SYSCLK_CLK:
		regs->srgr2	|= CLKSM;
		break;
	case OMAP_MCBSP_SYSCLK_CLKS_FCLK:
		if (cpu_is_omap44xx()) {
			regs->srgr2	|= CLKSM;
			break;
		}
	case OMAP_MCBSP_SYSCLK_CLKS_EXT:
		err = omap_mcbsp_dai_set_clks_src(mcbsp_data, clk_id);
		mcbsp_data->clk_id = clk_id;
		break;

	case OMAP_MCBSP_SYSCLK_CLKX_EXT:
		regs->srgr2	|= CLKSM;
	case OMAP_MCBSP_SYSCLK_CLKR_EXT:
		regs->pcr0	|= SCLKME;
		break;

	case OMAP_MCBSP_CLKR_SRC_CLKR:
	case OMAP_MCBSP_CLKR_SRC_CLKX:
	case OMAP_MCBSP_FSR_SRC_FSR:
	case OMAP_MCBSP_FSR_SRC_FSX:
		err = omap_mcbsp_dai_set_rcvr_src(mcbsp_data, clk_id);
		break;
	default:
		err = -ENODEV;
	}

	return err;
}

static int omap_mcbsp_dai_prepare(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct omap_mcbsp_data *mcbsp_data =
				to_mcbsp(cpu_dai->private_data);
	int bus_id = mcbsp_data->bus_id;

	if (!(mcbsp_data->tx_active || mcbsp_data->rx_active)) {
		omap_mcbsp_config(bus_id, &mcbsp_data->regs);

		/*@ TODO: for for mcbsp2 (busid=1), reset the transfer size */
	}
	return 0;
}

int omap_mcbsp_dai_suspend(struct snd_soc_dai *cpu_dai)
{
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);

	if (cpu_dai->active) {
		omap_mcbsp_dai_set_clks_src(mcbsp_data,
				OMAP_MCBSP_SYSCLK_CLKS_FCLK);
		omap_mcbsp_disable_fclk(mcbsp_data->bus_id);
	}

	return 0;
}

int omap_mcbsp_dai_resume(struct snd_soc_dai *cpu_dai)
{
	struct omap_mcbsp_data *mcbsp_data = to_mcbsp(cpu_dai->private_data);

	if (cpu_dai->active) {
		omap_mcbsp_enable_fclk(mcbsp_data->bus_id);
		omap_mcbsp_config(mcbsp_data->bus_id, &mcbsp_data->regs);
		omap_mcbsp_dai_set_clks_src(mcbsp_data, mcbsp_data->clk_id);
	}

	return 0;
}

static struct snd_soc_dai_ops omap_mcbsp_dai_ops = {
	.startup	= omap_mcbsp_dai_startup,
	.shutdown	= omap_mcbsp_dai_shutdown,
	.prepare = omap_mcbsp_dai_prepare,
	.trigger	= omap_mcbsp_dai_trigger,
	.hw_params	= omap_mcbsp_dai_hw_params,
	.set_fmt	= omap_mcbsp_dai_set_dai_fmt,
	.set_clkdiv	= omap_mcbsp_dai_set_clkdiv,
	.set_sysclk	= omap_mcbsp_dai_set_dai_sysclk,
};

#define OMAP_MCBSP_DAI_BUILDER(link_id)				\
{								\
	.name = "omap-mcbsp-dai-"#link_id,			\
	.id = (link_id),					\
	.playback = {						\
		.channels_min = 1,				\
		.channels_max = 4,				\
		.rates = OMAP_MCBSP_RATES,			\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
	},							\
	.capture = {						\
		.channels_min = 1,				\
		.channels_max = 4,				\
		.rates = OMAP_MCBSP_RATES,			\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
	},							\
	.suspend = omap_mcbsp_dai_suspend,			\
	.resume = omap_mcbsp_dai_resume,			\
	.ops = &omap_mcbsp_dai_ops,				\
	.private_data = &mcbsp_data[(link_id)].bus_id,		\
}

struct snd_soc_dai omap_mcbsp_dai[] = {
	OMAP_MCBSP_DAI_BUILDER(0),
	OMAP_MCBSP_DAI_BUILDER(1),
#if NUM_LINKS >= 3
	OMAP_MCBSP_DAI_BUILDER(2),
#endif
#if NUM_LINKS >= 4
	OMAP_MCBSP_DAI_BUILDER(3),
#endif
#if NUM_LINKS == 5
	OMAP_MCBSP_DAI_BUILDER(4),
#endif
};

EXPORT_SYMBOL_GPL(omap_mcbsp_dai);

static int __init snd_omap_mcbsp_init(void)
{
	return snd_soc_register_dais(omap_mcbsp_dai,
				     ARRAY_SIZE(omap_mcbsp_dai));
}
module_init(snd_omap_mcbsp_init);

static void __exit snd_omap_mcbsp_exit(void)
{
	snd_soc_unregister_dais(omap_mcbsp_dai, ARRAY_SIZE(omap_mcbsp_dai));
}
module_exit(snd_omap_mcbsp_exit);

MODULE_AUTHOR("Jarkko Nikula <jhnikula@gmail.com>");
MODULE_DESCRIPTION("OMAP I2S SoC Interface");
MODULE_LICENSE("GPL");
