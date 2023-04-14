/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) dma driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *     cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <sound/soc.h>
#include <linux/clk.h>
#include <sound/pcm_params.h>
#include "as-dma.h"


/* normal */
#ifndef CONFIG_SND_ASOC_INGENIC_SPDIF_DUMMY_TIMES
#define CONFIG_SND_ASOC_INGENIC_SPDIF_DUMMY_TIMES (0) //ms
#endif
#ifndef CONFIG_SND_ASOC_INGENIC_PLAYBACK_DUMMY_TIMES
#define CONFIG_SND_ASOC_INGENIC_PLAYBACK_DUMMY_TIMES (0) //ms
#endif

#ifdef DEBUG
static int ingenic_dma_debug = 1;
#else
static int ingenic_dma_debug = 0;
#endif

#define GET_PHYADDR(a)  \
({						\
	unsigned int v;        \
	if (unlikely((unsigned int)(a) & 0x40000000)) {    \
	v = page_to_phys(vmalloc_to_page((const void *)(a))) | ((unsigned int)(a) & ~PAGE_MASK); \
	} else     \
	v = ((unsigned int)(a) & 0x1fffffff);                   \
	v;                                             \
 })

module_param(ingenic_dma_debug, int, 0644);

#define DMA_DEBUG_MSG(msg...)	\
	do { \
		if (ingenic_dma_debug)	\
			printk(KERN_DEBUG"ADMA: " msg);	\
	} while(0)

#ifdef VERBOSE_DEBUG
#define DMA_DEBUG_VERBOSE_MSG(msg...)	\
	do {\
		if (ingenic_dma_debug)	\
			printk(KERN_DEBUG"ADMA: " msg);	\
	} while(0)
#else
#define	DMA_DEBUG_VERBOSE_MSG(msg...) {}
#endif

int __attribute__((weak)) ingenic_as_fmtcov_cfg(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	return 0;
}
void __attribute__((weak)) ingenic_as_fmtcov_enable(u8 dai_id, bool enable){}

static const struct snd_pcm_hardware ingenic_as_dma_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_MMAP_VALID),
	.buffer_bytes_max = INGENIC_DMA_BUFFERSIZE_MAX,
	.period_bytes_min = INGENIC_PERIODS_BYTES_MIN,
	.period_bytes_max = (INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_MIN),
	.periods_min = INGENIC_PERIODS_MIN,
	.periods_max = (INGENIC_DMA_BUFFERSIZE_MAX/INGENIC_PERIODS_BYTES_MIN),
	.fifo_size = 0,
	.formats = ~0ULL,
};

static snd_pcm_uframes_t ingenic_as_dma_pointer(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);
	struct ingenic_as_dma *as_dma = prtd->as_dma;
	dma_addr_t now;
	ssize_t pos;

	if (likely(substream->runtime && substream->runtime->dma_addr)) {
//		now = readl_relaxed(as_dma->dma_base + DBA(prtd->dai_id));
		now = prtd->hw_ptr;
		if (now)
			pos = (ssize_t)(now - substream->runtime->dma_addr);
		else
			pos = 0;
	} else {
		WARN("%s: %s has no runtime\n", substream->name);
		pos = 0;
	}
	DMA_DEBUG_VERBOSE_MSG("enter %s[dai%d substream = %s] pos = %x(%d)(now %x dma %x)\n",
			__func__, prtd->dai_id,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			pos, pos, now, substream->runtime->dma_addr);
	if (pos >= snd_pcm_lib_buffer_bytes(substream))
		pos = 0;
	return bytes_to_frames(substream->runtime, pos);
}

static int ingenic_as_dma_period_bytes_quirk(struct snd_pcm_hw_params *params,
		struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *iperiod_bytes = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	struct snd_interval *iframe_bits = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_FRAME_BITS);
	int align_bytes = DCM_TSZ_MAX_WORD << 2;
	int min_frame_bytes = iframe_bits->min >> 3;
	int max_frame_bytes = iframe_bits->max >> 3;
	int min_period_bytes = iperiod_bytes->min;
	int max_period_bytes = iperiod_bytes->max;
	int min_align_bytes, max_align_bytes;
	struct snd_interval nperiod_bytes;

	snd_interval_any(&nperiod_bytes);
	min_align_bytes = lcm(align_bytes, min_frame_bytes);
	min_period_bytes = (min_period_bytes + min_align_bytes - 1) / min_align_bytes;
	nperiod_bytes.min = min_period_bytes * min_align_bytes;

	max_align_bytes = lcm(align_bytes, max_frame_bytes);
	max_period_bytes = max_period_bytes / max_align_bytes;
	nperiod_bytes.max = max_period_bytes * max_align_bytes;

	DMA_DEBUG_MSG("==> %s %d : align_bytes = %d \n\
			frame_bytes.min (%d)\t\tframe_bytes.max (%d) \n\
			period_bytes.min  [%d]\tperiod_bytes.max  [%d] \n\
			nperiod_bytes.min [%d]\tnperiod_bytes.max [%d]\n",
			__func__, __LINE__, align_bytes,
			min_frame_bytes, max_frame_bytes, iperiod_bytes->min,
			iperiod_bytes->max, nperiod_bytes.min, nperiod_bytes.max);
	return snd_interval_refine(iperiod_bytes, &nperiod_bytes);
}

static int ingenic_as_dma_open(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ingenic_as_dma *as_dma = snd_soc_component_get_drvdata(component);
	struct ingenic_as_dma_runtime *prtd = NULL;
	struct snd_pcm_hardware pcm_hardware = ingenic_as_dma_pcm_hardware;
	int ret;

	DMA_DEBUG_MSG("enter %s[dai%d substream = %s], process is \"%s\" (pid %i)\n",
			__func__, cpu_dai->id,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture", current->comm, current->pid);

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	if (ret < 0)
		return ret;

	if (as_dma->dma_fth_quirk) {
		snd_pcm_hw_rule_add(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				ingenic_as_dma_period_bytes_quirk,
				NULL,
				SNDRV_PCM_HW_PARAM_FRAME_BITS,
				SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				-1);
	}

	prtd->as_dma = as_dma;
	prtd->substream = substream;
	prtd->dai_id = cpu_dai->id;
	runtime->private_data = prtd;

	/* dummy desc */
	prtd->dummy_descs = (struct ingenic_as_dma_desc *)dma_alloc_coherent(as_dma->dev, 
			sizeof(struct ingenic_as_dma_desc), &prtd->dummy_descs_phy, GFP_KERNEL);
	if (IS_ERR_OR_NULL(prtd->dummy_descs)) {
		return -ENOMEM;
	}

	spin_lock(&as_dma->lock);
	as_dma->rtd[prtd->dai_id] = prtd;
	as_dma->refcnt++;
	prtd->fifo_depth = as_dma->fifo_depth[prtd->dai_id];
	spin_unlock(&as_dma->lock);

	pcm_hardware.fifo_size = prtd->fifo_depth << 2;
	snd_soc_set_runtime_hwparams(substream, &pcm_hardware);
	return 0;
}

static int ingenic_as_dma_close(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ingenic_as_dma *as_dma = snd_soc_component_get_drvdata(component);

	DMA_DEBUG_MSG("enter %s[dai%d substream = %s] process is \"%s\" (pid %i)\n",
			__func__, cpu_dai->id,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture", current->comm, current->pid);

	spin_lock(&as_dma->lock);
	as_dma->refcnt--;
	as_dma->rtd[cpu_dai->id] = NULL;
	spin_unlock(&as_dma->lock);

	dma_free_coherent(as_dma->dev, sizeof(struct ingenic_as_dma_desc), prtd->dummy_descs, prtd->dummy_descs_phy);
	kfree(prtd);
	substream->runtime->private_data = NULL;
	return 0;
}

static int ingenic_as_dma_ioctl(struct snd_soc_component *component, struct snd_pcm_substream *substream, unsigned int cmd, void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

/* RETURN 0 not change, 1 change, < 0 failed */
static int ingenic_as_dma_alloc_descs(struct ingenic_as_dma_runtime *prtd, int cnt)
{
	struct ingenic_as_dma *as_dma = prtd->as_dma;
	int i, old_cnts = prtd->desc_cnts;

	if (cnt == prtd->desc_cnts)
		return 0;

	for (i = cnt; i < old_cnts; i++) {
		dma_pool_free(as_dma->desc_pool, prtd->descs[i], prtd->descs_phy[i]);
		prtd->descs[i] = NULL;
		prtd->descs_phy[i] = 0;
		prtd->desc_cnts--;
	}
	for (i = old_cnts; i < cnt; i++) {
		prtd->descs[i] = dma_pool_alloc(as_dma->desc_pool,
				GFP_KERNEL, &prtd->descs_phy[i]);
		if (!prtd->descs[i])
			return -ENOMEM;
		prtd->desc_cnts++;
	}

	return 1;
}

static void ingenic_as_dma_free_descs(struct ingenic_as_dma_runtime *prtd)
{
	struct ingenic_as_dma *as_dma = prtd->as_dma;
	int i;

	for (i = 0; i < prtd->desc_cnts; i++) {
		dma_pool_free(as_dma->desc_pool, prtd->descs[i], prtd->descs_phy[i]);
		prtd->descs[i] = NULL;
		prtd->descs_phy[i] = 0;
	}
	prtd->desc_cnts = 0;
}

extern int ingenic_dsp_is_spdif_out(uint32_t dma_device_id);

static void ingenic_as_dma_cyclic_descs_init(struct ingenic_as_dma_runtime *prtd,
		 dma_addr_t dma_addr, struct snd_pcm_hw_params *params)
{
	struct ingenic_as_dma *as_dma = prtd->as_dma;
	int i, tsz, ts_size;
	int period_bytes = params_period_bytes(params);

	DMA_DEBUG_MSG("%s [dai%d substream %s] args: \n\
			fifo_depth(words) = %d, periods_bytes = %d\n",
			__func__, prtd->dai_id,
			(prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			prtd->fifo_depth, period_bytes);

	if (unlikely(!as_dma->dma_fth_quirk)) {
		tsz = ingenic_as_dma_get_tsz(prtd->fifo_depth, period_bytes, &ts_size);
	} else {
		ts_size = DCM_TSZ_MAX_WORD << 2;
		tsz = DCM_TSZ_MAX;
	}
	prtd->tsz_words = ts_size >> 2;
	DMA_DEBUG_MSG("%s [dai%d substream %s] tsz result: tsz(words) = %d\n",
			__func__, prtd->dai_id,
			(prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			prtd->tsz_words);

	if(prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		uint32_t dummy_time;

		dummy_time = (ingenic_dsp_is_spdif_out(prtd->dai_id) && CONFIG_SND_ASOC_INGENIC_SPDIF_DUMMY_TIMES) ?
			CONFIG_SND_ASOC_INGENIC_SPDIF_DUMMY_TIMES : CONFIG_SND_ASOC_INGENIC_PLAYBACK_DUMMY_TIMES;

		if(dummy_time) {
			uint32_t rate = params_rate(params);
			uint32_t channels = params_channels(params);
			uint32_t fmt_width = snd_pcm_format_width(params_format(params));
			uint32_t dummy_bytes = dummy_time * (rate * channels * fmt_width) / (8 * 1000);

			prtd->dummy_data_flag = 1;
			prtd->dummy_data_irq = 0;

			memset(prtd->dummy_descs, 0, sizeof(*prtd->dummy_descs));
			prtd->dummy_descs->link = 1;
			prtd->dummy_descs->tie = prtd->dummy_descs->bai = 0;
			prtd->dummy_descs->tsz = tsz;
			prtd->dummy_descs->dba = GET_PHYADDR(as_dma->dummy_data);
			prtd->dummy_descs->ndoa = *(prtd->descs_phy);
			prtd->dummy_descs->dtc = ((dummy_bytes)/ts_size);
		}
	}

	for (i = 0; i < prtd->desc_cnts; i++) {
		memset(prtd->descs[i], 0, sizeof(prtd->descs[i]));
		prtd->descs[i]->link = prtd->descs[i]->tie = prtd->descs[i]->bai = 1;
		prtd->descs[i]->tsz = tsz;
		prtd->descs[i]->dba = dma_addr + i * period_bytes;
		prtd->descs[i]->ndoa = *(prtd->descs_phy + ((i + 1)%prtd->desc_cnts));
		prtd->descs[i]->dtc = (period_bytes/ts_size);
	}
	iob();
}

static int ingenic_as_dma_hw_params(struct snd_soc_component *component, struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);
	struct ingenic_as_dma *as_dma = prtd->as_dma;
	bool is_out = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? true : false;
	int dev_id = cpu_dai->id;
	int *x = NULL;
	int ret;

	DMA_DEBUG_MSG("%s [dai%d substream %s] periodbyte = %u, rate= %u, channels=%u, \n\
			bufferbyte %d, periods %d, fmtwidth %d, periodsize %d\n",
			__func__, dev_id,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			params_period_bytes(params), params_rate(params),
			params_channels(params), params_buffer_bytes(params),
			params_periods(params), params_width(params),
			params_period_size(params));

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	ret = ingenic_as_dma_alloc_descs(prtd, params_periods(params));
	if (ret < 0)
		return ret;
	ingenic_as_dma_cyclic_descs_init(prtd,
			substream->runtime->dma_addr,
			params);

	x = (int *)prtd->descs[0];
	DMA_DEBUG_MSG("%s [dai%d substream %s] DMA buffer addr: 0x%08x \n\
			DMA desc[0] addr(0x%p:0x%08x) \n\
			DES0(%p: 0x%08x) DES1(%p: 0x%08x)\n\
			DES2(%p: 0x%08x) DES3(%p: 0x%08x)\n",
			__func__, dev_id,
			(prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			substream->runtime->dma_addr,
			prtd->descs[0], prtd->descs_phy[0],
			x, *(x),
			x + 1, *(x + 1),
			x + 2, *(x + 2),
			x + 3, *(x + 3));
	ret = ingenic_as_fmtcov_cfg(substream, params);
	if (ret < 0)
		return ret;

	regmap_write(as_dma->fifo_regmap, FFR(dev_id), FFR_FTH(prtd->tsz_words) | (is_out ? 0 : FFR_FIFO_TD));
	return 0;
}

static int ingenic_as_dma_hw_free(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);

	ingenic_as_dma_free_descs(prtd);

	return snd_pcm_lib_free_pages(substream);
}

void ingenic_as_dma_reg_dump(struct ingenic_as_dma *as_dma, int dev_id)
{
	unsigned int val;

	printk("==>DMA\n");
	regmap_read(as_dma->dma_regmap, DBA(dev_id), &val);
	printk("DBA\t\t%x : 0x%08x\n",DBA(dev_id), val);
	regmap_read(as_dma->dma_regmap, DTC(dev_id), &val);
	printk("DTC\t\t%x : 0x%08x\n",DTC(dev_id), val);
	regmap_read(as_dma->dma_regmap, DCR(dev_id), &val);
	printk("DCR\t\t%x : 0x%08x\n",DCR(dev_id), val);
	regmap_read(as_dma->dma_regmap, DSR(dev_id), &val);
	printk("DSR\t\t%x : 0x%08x\n",DSR(dev_id), val);
	regmap_read(as_dma->dma_regmap, DCM(dev_id), &val);
	printk("DCM\t\t%x : 0x%08x\n",DCM(dev_id), val);
	regmap_read(as_dma->dma_regmap, DDA(dev_id), &val);
	printk("DDA\t\t%x : 0x%08x\n",DDA(dev_id), val);
	regmap_read(as_dma->dma_regmap, DGER, &val);
	printk("DGER\t\t%x : 0x%08x\n",DGER, val);
	regmap_read(as_dma->dma_regmap, AEER, &val);
	printk("AEER\t\t%x : 0x%08x\n",AEER, val);
	regmap_read(as_dma->dma_regmap, AESR, &val);
	printk("AESR\t\t%x : 0x%08x\n",AESR, val);
	regmap_read(as_dma->dma_regmap, AIPR, &val);
	printk("AIPR\t\t%x : 0x%08x\n",AIPR, val);

	printk("==>FIFO\n");
	regmap_read(as_dma->fifo_regmap, FAS(dev_id), &val);
	printk("FAS\t\t%x : 0x%08x\n",FAS(dev_id), val);
	regmap_read(as_dma->fifo_regmap, FCR(dev_id), &val);
	printk("FCR\t\t%x : 0x%08x\n",FCR(dev_id), val);
	regmap_read(as_dma->fifo_regmap, FFR(dev_id), &val);
	printk("FFR\t\t%x : 0x%08x\n",FFR(dev_id), val);
	regmap_read(as_dma->fifo_regmap, FSR(dev_id), &val);
	printk("FSR\t\t%x : 0x%08x\n",FSR(dev_id), val);
}

static int ingenic_as_dma_start(struct ingenic_as_dma *as_dma, int dev_id)
{
	struct ingenic_as_dma_runtime *prtd = as_dma->rtd[dev_id];

	regmap_write(as_dma->dma_regmap, DSR(dev_id), DSR_TT_INT|DSR_LTT_INT|DSR_LTT|DSR_TT);

	regmap_update_bits(as_dma->dma_regmap, DCM(dev_id), DCM_NDES|DCM_TIE, DCM_TIE);

	/* Note: DMA enable first FIFO enable, read operation
	 * to ensure that the DMA module work
	 * */
	if (prtd->dummy_data_flag) {
		prtd->dummy_data_irq = 1;
		regmap_write(as_dma->dma_regmap, DDA(dev_id), prtd->dummy_descs_phy);
	} else {
		regmap_write(as_dma->dma_regmap, DDA(dev_id), prtd->descs_phy[0]);
	}
	readl_relaxed(as_dma->dma_base + DDA(dev_id));

	ingenic_as_fmtcov_enable(dev_id, true);

	regmap_write(as_dma->dma_regmap, DCR(dev_id), DCR_CTE);

//	regmap_write(as_dma->fifo_regmap, FCR(dev_id), FCR_FLUSH);

	regmap_write(as_dma->fifo_regmap, FCR(dev_id), FCR_FIFO_EN);

	DMA_DEBUG_MSG("%s [dai%d substream %s] dma trigger start dba reg val = 0x%08x\n",
			__func__, dev_id,
			(prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			"playback" : "capture",
			readl_relaxed(as_dma->dma_base + DBA(prtd->dai_id)));

	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		regmap_write(as_dma->fifo_regmap, FSR(dev_id), FSR_TURROR);
		regmap_update_bits(as_dma->fifo_regmap, FFR(dev_id), FFR_TURRORE, FFR_TURRORE);
	}
	return 0;
}

static int ingenic_as_dma_stop(struct ingenic_as_dma *as_dma, int dev_id)
{
	u32 val, timeout = 0xfff;

	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		regmap_update_bits(as_dma->fifo_regmap, FFR(dev_id), FFR_TURRORE, 0);
		regmap_write(as_dma->fifo_regmap, FSR(dev_id), FSR_TURROR);
	}

	regmap_update_bits(as_dma->dma_regmap, DCM(dev_id), DCM_TIE, 0);
	regmap_write(as_dma->dma_regmap, DSR(dev_id), DSR_TT_INT|DSR_LTT_INT|DSR_LTT|DSR_TT);

	regmap_write(as_dma->fifo_regmap, FCR(dev_id), 0);

	regmap_write(as_dma->dma_regmap, DCR(dev_id), 0);
	do {
		regmap_read(as_dma->dma_regmap, DSR(dev_id), &val);
	} while ((val & DSR_RST_EN) && (--timeout));

	regmap_write(as_dma->dma_regmap, DCR(dev_id), DCR_RESET);
	udelay(10);

	ingenic_as_fmtcov_enable(dev_id, false);

	return 0;
}

#ifdef CONFIG_DMIC_AND_AMIC_SYNC
static int ingenic_as_dma_dmic_and_amic_sync_trigger(struct snd_soc_component *component, struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ingenic_as_dma *as_dma = snd_soc_platform_get_drvdata(component);
	int ret;
	static int dmic_arrive = 0, amic_arrive = 0;
	int timeout = 10000;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:	/*FIXME*/
		DMA_DEBUG_MSG("enter %s[dai%d substream = %s] start\n", __func__,
				cpu_dai->id,
				(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				"playback" : "capture");
		if((substream->stream == SNDRV_PCM_STREAM_CAPTURE) &&
			(strcmp("DMA5", cpu_dai->name) == 0 || strcmp("DMA6", cpu_dai->name) == 0)) {
			snd_pcm_stream_unlock_irq(substream);
		}
		if(substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			if(!(strcmp("DMA5", cpu_dai->name))) {
				/*  dmic regular use DMA5 */
				dmic_arrive = 1;
				while(timeout --) {
					if(amic_arrive == 1) {
						amic_arrive = 0;
						break;
					}
					msleep(1);
				}
			}

			if(!(strcmp("DMA6", cpu_dai->name))) {
				/*amic regular use DMA6 */
				amic_arrive = 1;
				while(timeout --) {
					if(dmic_arrive == 1) {
						dmic_arrive = 0;
						break;
					}
					msleep(1);
				}
			}
		}

		ret = ingenic_as_dma_start(as_dma, cpu_dai->id);
		if((substream->stream == SNDRV_PCM_STREAM_CAPTURE) &&
			(strcmp("DMA5", cpu_dai->name) == 0 || strcmp("DMA6", cpu_dai->name) == 0)) {
			snd_pcm_stream_lock_irq(substream);
		}
		return ret;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		DMA_DEBUG_MSG("enter %s[dai%d substream = %s] stop\n", __func__,
				cpu_dai->id,
				(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				"playback" : "capture");
		ret = ingenic_as_dma_stop(as_dma, cpu_dai->id);
		return ret;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*JUST stop or resume the be dai*/
		return 0;
	default:
		return -EINVAL;
	}
}
#endif

static int ingenic_as_dma_trigger(struct snd_soc_component *component, struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ingenic_as_dma *as_dma = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:	/*FIXME*/
		DMA_DEBUG_MSG("enter %s[dai%d substream = %s] start\n", __func__,
				cpu_dai->id,
				(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				"playback" : "capture");
		return ingenic_as_dma_start(as_dma, cpu_dai->id);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		DMA_DEBUG_MSG("enter %s[dai%d substream = %s] stop\n", __func__,
				cpu_dai->id,
				(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				"playback" : "capture");
		return ingenic_as_dma_stop(as_dma, cpu_dai->id);
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*JUST stop or resume the be dai*/
		return 0;
	default:
		return -EINVAL;
	}
}

static int ingenic_as_dma_pcm_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;


	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, card->dev,
			INGENIC_DMA_BUFFERSIZE_PREALLOC, INGENIC_DMA_BUFFERSIZE_MAX);

	return 0;
}

static void ingenic_as_dma_pcm_free(struct snd_soc_component *component, struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}


static const struct snd_soc_component_driver ingenic_as_dma_platform = {
	.name		= "as-dma",
	.open		= ingenic_as_dma_open,
	.close		= ingenic_as_dma_close,
	.ioctl		= ingenic_as_dma_ioctl,
	.hw_params	= ingenic_as_dma_hw_params,
	.hw_free	= ingenic_as_dma_hw_free,
	.trigger	= ingenic_as_dma_trigger,
#ifdef CONFIG_DMIC_AND_AMIC_SYNC
	.trigger        = ingenic_as_dma_dmic_and_amic_sync_trigger,
#endif
	.pointer	= ingenic_as_dma_pointer,
	.pcm_construct	= ingenic_as_dma_pcm_new,
	.pcm_destruct	= ingenic_as_dma_pcm_free,
};

static irqreturn_t ingenic_as_dma_irq_handler(int irq, void *dev_id)
{
	struct ingenic_as_dma *as_dma = (void *)dev_id;
	struct ingenic_as_dma_runtime *prtd = NULL;
	unsigned int dma_status, fifo_status, pending, pending1;
	struct snd_pcm_substream *substream = NULL;
	int ch;

	regmap_read(as_dma->dma_regmap, AIPR, &pending);
	dev_dbg(as_dma->dev, "interrupt (irq%d) pending %x\n", irq, pending);
	pending &= AIPR_DMA_INT_MSK;
	pending1 = pending;

	while((ch = ffs(pending1))) {
		ch -= 1;
		pending1 &= ~BIT(ch);
		regmap_read(as_dma->dma_regmap, DSR(ch), &dma_status);
		regmap_write(as_dma->dma_regmap, DSR(ch), dma_status);
		if (!(dma_status & DSR_TT_INT))
			continue;
		dev_dbg(as_dma->dev, "dma ch(%x) pending %x\n", dma_status, ch);
		spin_lock(&as_dma->lock);
		substream = NULL;
		prtd = as_dma->rtd[ch];
		if (likely(prtd)) {
			substream = prtd->substream;
			spin_unlock(&as_dma->lock);
			if (prtd->dummy_data_flag &&
					prtd->dummy_data_irq) {
				prtd->dummy_data_irq = 0;
			} else {
				prtd->hw_ptr = readl_relaxed(as_dma->dma_base + DBA(prtd->dai_id));
				snd_pcm_period_elapsed(substream);
			}

		} else {
			spin_unlock(&as_dma->lock);
			dev_warn(as_dma->dev, "dma channel(%d) \
					stream has been stopped\n", ch);
		}
	}

	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		while((ch = ffs(pending))) { /*handle ror/tur debug*/
			ch -= 1;
			pending &= ~BIT(ch);
			regmap_read(as_dma->fifo_regmap, FSR(ch), &fifo_status);
			regmap_write(as_dma->fifo_regmap, FSR(ch), fifo_status);
			if (fifo_status & FSR_TURROR_INT)
				dev_warn(as_dma->dev, "Fifo %d xrun\n", ch);
		}
	}

	return IRQ_HANDLED;
}

ssize_t fifo_depth_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ingenic_as_dma *as_dma = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ch;

	for (ch = 0;  ch < as_dma->chan_cnts; ch++) {
		len += snprintf(buf + len, PAGE_SIZE - len, ch ? "-%d": "%d", as_dma->fifo_depth[ch]);
	}
	return len;
}

ssize_t fifo_depth_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ingenic_as_dma *as_dma = dev_get_drvdata(dev);
	char *val, *tmp = (char *)vmalloc(strlen(buf) + 1);
	unsigned long depth, res_depth = as_dma->fifo_total_depth;
	int ret, ch = 0, res_chs = as_dma->chan_cnts;

	memcpy(tmp, buf, strlen(buf) + 1);

	spin_lock(&as_dma->lock);
	if (as_dma->refcnt) {
		spin_unlock(&as_dma->lock);
		return 0;
	}

	while (*tmp && res_chs && res_depth) {
		val = strsep(&tmp, "-");
		ret = kstrtoul(val, 10, &depth);
		WARN_ON(ret);
		as_dma->fifo_depth[ch++] = min(min(depth, res_depth),
				(unsigned long)(FAS_FAD >> FAS_FAD_SFT));
		res_depth -= depth;
		res_chs--;
	}

	for (;ch < as_dma->chan_cnts; ch++)
		as_dma->fifo_depth[ch++] = res_depth/res_chs;

	for (ch = 0; ch < as_dma->chan_cnts; ch++)
		regmap_write(as_dma->fifo_regmap, FAS(ch), as_dma->fifo_depth[ch]);
	spin_unlock(&as_dma->lock);
	return count;
}

static DEVICE_ATTR_RW(fifo_depth);

static struct attribute *ingenic_as_dma_attrs[] = {
	&dev_attr_fifo_depth.attr,
	NULL,
};

static struct attribute_group ingenic_as_dma_attr_grp = {
	.attrs = ingenic_as_dma_attrs,
};

static const struct attribute_group *ingenic_as_dma_groups[] = {
	&ingenic_as_dma_attr_grp,
	NULL,
};

static const struct of_device_id ingenic_as_dma_match_table[];
static int ingenic_as_dma_probe(struct platform_device *pdev)
{
	struct ingenic_as_dma *as_dma;
	const struct of_device_id *match;
	struct resource *res;
	int ret, chan_cnts, i;
	u32 fifo_depth;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	match = of_match_node(ingenic_as_dma_match_table, pdev->dev.of_node);
	if (!match)
		return -ENODEV;
	chan_cnts = (int)match->data;

	as_dma = (struct ingenic_as_dma *)devm_kzalloc(&pdev->dev, sizeof(*as_dma) +
			(sizeof (*as_dma->rtd) + sizeof(*as_dma->fifo_depth)) * chan_cnts,
			GFP_KERNEL);
	if (!as_dma)
		return -ENOMEM;

	as_dma->dev = &pdev->dev;
	as_dma->rtd = (void *)(as_dma + 1);
	as_dma->fifo_depth = (void *)(as_dma->rtd + chan_cnts);
	as_dma->chan_cnts = chan_cnts;

	/* dummy data */
	as_dma->dummy_data = (uint8_t *)dma_alloc_coherent(&pdev->dev, 
			sizeof(uint8_t) * 4 * 32, &as_dma->dummy_data_pyaddr, GFP_KERNEL);
	if (IS_ERR_OR_NULL(as_dma->dummy_data)) {
		return -ENOMEM;
	}

	spin_lock_init(&as_dma->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	as_dma->dma_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(as_dma->dma_base))
		return PTR_ERR(as_dma->dma_base);
	regmap_config.max_register = resource_size(res) - 0x4;
	regmap_config.name = res->name;
	as_dma->dma_regmap = devm_regmap_init_mmio(&pdev->dev,
			as_dma->dma_base,
			&regmap_config);
	if (IS_ERR(as_dma->dma_regmap))
		return PTR_ERR(as_dma->dma_regmap);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fifo");
	as_dma->fifo_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(as_dma->fifo_base))
		return PTR_ERR(as_dma->fifo_base);
	regmap_config.max_register = resource_size(res) - 0x4;
	regmap_config.name = res->name;
	as_dma->fifo_regmap = devm_regmap_init_mmio(&pdev->dev,
			as_dma->fifo_base,
			&regmap_config);
	if (IS_ERR(as_dma->fifo_regmap))
		return PTR_ERR(as_dma->fifo_regmap);

	as_dma->irq = of_irq_get(pdev->dev.of_node, 0);
	if (as_dma->irq < 0)
		return as_dma->irq;

	as_dma->audio_clk = devm_clk_get(&pdev->dev, "gate_audio");
	if(IS_ERR_OR_NULL(as_dma->audio_clk)) {
		dev_err(&pdev->dev, "Failed to get gate_audio clk!\n");
	}
	clk_prepare_enable(as_dma->audio_clk);

	platform_set_drvdata(pdev, as_dma);

	regmap_write(as_dma->dma_regmap, DGRR, DGRR_RESET);

	/*INIT FIFO*/
	if (of_property_read_u32(pdev->dev.of_node, "ingenic,fifo_size", &as_dma->fifo_total_depth))
		as_dma->fifo_total_depth = INGENIC_DEF_FIFO_DEPTH;

	as_dma->dma_fth_quirk = of_property_read_bool(pdev->dev.of_node, "ingenic,fth_quirk");

	fifo_depth = 384;
	for (i = 0; i < as_dma->chan_cnts; i++) {
		as_dma->fifo_depth[i] = fifo_depth;
		regmap_write(as_dma->fifo_regmap, FAS(i), fifo_depth);
	}

	ret = devm_request_threaded_irq(&pdev->dev, as_dma->irq, NULL,
			ingenic_as_dma_irq_handler, IRQF_SHARED|IRQF_ONESHOT,
			"as-dma", as_dma);
	if (ret)
		return ret;

	as_dma->desc_pool = dma_pool_create("as_dma_desc_pool", &pdev->dev,
			sizeof(struct ingenic_as_dma_desc),
			__alignof__(struct ingenic_as_dma_desc), 0);
	if (!as_dma->desc_pool)
		return -ENOMEM;

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_as_dma_platform, NULL, 0);


	if (ret) {
		dma_pool_destroy(as_dma->desc_pool);
		return ret;
	}

	ret = sysfs_create_groups(&pdev->dev.kobj, ingenic_as_dma_groups);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "probe success!!!\n");
	return 0;
}

static int ingenic_as_dma_remove(struct platform_device *pdev)
{
	struct ingenic_as_dma *as_dma = platform_get_drvdata(pdev);

	dma_pool_destroy(as_dma->desc_pool);
	dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, as_dma->dummy_data, as_dma->dummy_data_pyaddr);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id ingenic_as_dma_match_table[] = {
#define INGENIC_AS_DMA_DEF_CHS (10)
	{ .compatible = "ingenic,as-platform", .data = (void*)INGENIC_AS_DMA_DEF_CHS},
#undef INGENIC_AS_DMA_DEF_CHS
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_as_dma_match_table);

#ifdef CONFIG_PM
static int ingenic_as_dma_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_as_dma *as_dma = platform_get_drvdata(pdev);
	int i;
	unsigned int fcr;

	for(i = 0; i < as_dma->chan_cnts; i++) {
		regmap_read(as_dma->fifo_regmap, FCR(i), &fcr);
		if(fcr & FCR_FIFO_EN) {
			return -1;
		}
	}
	return 0;
}

static int ingenic_as_dma_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver ingenic_as_dma_platform_driver = {
	.driver = {
		.name = "as-platform",
		.of_match_table = ingenic_as_dma_match_table,
	},
	.probe = ingenic_as_dma_probe,
	.remove = ingenic_as_dma_remove,
#ifdef CONFIG_PM
	.suspend = ingenic_as_dma_suspend,
	.resume = ingenic_as_dma_resume,
#endif
};

extern int ingenic_as_fmtcov_platform_driver_modinit(void);
extern int ingenic_as_dsp_platform_driver_modinit(void);
extern int ingenic_as_mixer_platform_driver_modinit(void);
static int ingenic_as_dma_platform_driver_modinit(void)
{
	platform_driver_register(&ingenic_as_dma_platform_driver);
	ingenic_as_dsp_platform_driver_modinit();
	ingenic_as_mixer_platform_driver_modinit();
	return ingenic_as_fmtcov_platform_driver_modinit();
}

extern void ingenic_as_mixer_platform_driver_exit(void);
extern void ingenic_as_fmtcov_platform_driver_exit(void);
extern void ingenic_as_dsp_platform_driver_exit(void);
static void ingenic_as_dma_platform_driver_exit(void)
{
	ingenic_as_fmtcov_platform_driver_exit();
	ingenic_as_mixer_platform_driver_exit();
	ingenic_as_dsp_platform_driver_exit();
	platform_driver_unregister(&ingenic_as_dma_platform_driver);
}

module_init(ingenic_as_dma_platform_driver_modinit);
module_exit(ingenic_as_dma_platform_driver_exit);
/* module_platform_driver(ingenic_as_dma_platform_driver); */

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS DMA SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-dma");
