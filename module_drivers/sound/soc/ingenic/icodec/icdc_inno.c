/*
 * sound/soc/ingenic/icodec/icdc_inno.c
 * ALSA SoC Audio driver -- ingenic internal codec (icodec) driver

 * Copyright 2019 Ingenic Semiconductor Co.,Ltd
 *	tjyang <taojiang.yang@ingenic.com>
 *
 * Note: icodec is an internal codec for ingenic SOC
 *	 used for x2000 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/soc-dai.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/tlv.h>


#define INNO_CODEC_BASE 0xb0020000
static bool	    inno_codec_master = true;

#define ICODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_LE)



/*icodec for ingenic*/
#define RESET_REG_00			0x00  /*0x00 * 4*/
#define ADC_DAC_CONFIG1_02		0x08  /*0x02 * 4*/
#define ADC_DAC_CONFIG2_03		0x0c  /*0x03 * 4*/
#define ADC_DAC_CONFIG3_04		0x10  /*0x04 * 4*/
#define ADC_DAC_CONFIG4_05		0x14  /*0x05 * 4*/
#define ALC_GAIN_0A			0x28  /*0x0a * 4*/
#define PRE_DIS_CHARGE_21		0x84  /*0x21 * 4*/

#define CONTROL_REG1_22			0x88  /*0x22 * 4*/
#define CONTROL_REG2_23			0x8c  /*0x23 * 4*/
#define CONTROL_REG3_24			0x90  /*0x24 * 4*/
#define CONTROL_REG4_25			0x94  /*0x25 * 4*/
#define CONTROL_REG5_26			0x98  /*0x26 * 4*/
#define CONTROL_REG6_27			0x9c  /*0x27 * 4*/
#define CONTROL_REG7_28			0xa0  /*0x28 * 4*/
#define ALG_CONFIG1_44			0x110  /*0x44 * 4*/
#define ALG_MAXL_45			0x114  /*0x45 * 4*/
#define ALG_MAXH_46			0x118  /*0x46 * 4*/
#define ALG_MINL_47			0x11c  /*0x47 * 4*/
#define ALG_MINH_48			0x120  /*0x48 * 4*/
#define ALG_CONFIG2_49			0x124  /*0x49 * 4*/


static int icodec_mute = 0;    //1:静音； 0:非静音

struct icodec {
	struct device		*dev;
	struct snd_soc_component	*component;
	spinlock_t		io_lock;
	void * __iomem io_base;
	bool powered;
	int spk_gpio;
	unsigned char spk_en_level;
	unsigned char alcl_gain;
	unsigned char hpoutl_gain;
	unsigned char micbias;
};

typedef void (*p_icodec_playback_power)(bool);
static p_icodec_playback_power g_icodec_playback_pwr;// playback power control funtion pointer assignment in boards

unsigned int reg_tab[] = {0x00, 0x02, 0x03, 0x4, 0x5, 0x0a, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
	0x27, 0x28, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49};

void set_playback_pwr_callback(p_icodec_playback_power func)
{
	g_icodec_playback_pwr = func;
}
EXPORT_SYMBOL(set_playback_pwr_callback);

static unsigned int icodec_read(struct snd_soc_component *component, unsigned int reg)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	int value = readl(icodec->io_base + reg);
	return value;
}

static int icodec_write(struct snd_soc_component *component, unsigned int reg , unsigned value)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	writel(value ,icodec->io_base + reg);
	return 0;
}

void dump_regs(struct snd_soc_component *component)
{
	int i;
	for(i=0; i < ARRAY_SIZE(reg_tab); i++) {
		printk("reg(0x%x) = 0x%x\n", reg_tab[i], *(volatile unsigned int *)(INNO_CODEC_BASE + 4 * reg_tab[i]));
	}
}


static void inno_poweron(struct snd_soc_component *component)
{
	unsigned int reg = 0x00;
	int i ;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg &= ~(3<<0);
	reg |= (1<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = 0x01;
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x21*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;

	for(i = 0; i < 0x100; i++) {
		*(volatile unsigned int *)(INNO_CODEC_BASE + 0x21*4) = i;
		udelay(200);
	}
}

static int inno_dac_configure(struct snd_soc_component *component,
			struct snd_pcm_hw_params *params)
{
	int fmt_width = snd_pcm_format_width(params_format(params));
	unsigned int reg = 0x00;

	reg = 0x02;
	if(fmt_width > 16) {
		reg |= (0x3 << 2);
	} else if(fmt_width > 8) {
		reg |= (0x0 << 2);
	}
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x05*4) = reg;

	reg = 0x10;
	if(fmt_width == 32) {
		reg |= (0x3 << 5);
	} else if(fmt_width == 24) {
		reg |= (0x2 << 5);
	} else if(fmt_width == 20)  {
		reg |= (0x1 << 5);
	} else if(fmt_width == 16){
		reg |= (0x0 << 5);
	} else {
		dev_err(component->dev, "icodec not support format width\n");
		return -EINVAL;
	}
	reg |= (0x1 << 2);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x04*4) = reg;

	return 0;
}

static void inno_dac_enable (struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int reg = 0x00;

	mdelay(30);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = 0;
	mdelay(30);

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<3);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<2);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg &= ~(3<<0);
	reg |= (2<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg |= (1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;


/***3*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg |= (1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg |= (1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg |= (1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;


/***4*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg |= (1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

/***5*/

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg &= ~(0x1f<<0);
	reg |= (icodec->hpoutl_gain << 0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;
}

static void inno_dac_disable (struct snd_soc_component *component)
{
	unsigned int reg = 0x00;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg &= ~(0x1f<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg &= ~(0x1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg &= ~(0x1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
	reg &= ~(0x1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg &= ~(0x1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg &= ~(0x1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg &= ~(0x1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg &= ~(0x3<<0);
	reg |= (0x1<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg &= ~(0x1<<2);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg &= ~(0x1<<3);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) ;
	reg &= ~(0x1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x27*4) = reg;
}


static void inno_poweroff(struct snd_soc_component *component)
{
	unsigned int reg = 0x00;
	int i ;

	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x21*4) = 0x01;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) ;
	reg &= ~(0x1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;

	for(i = 0; i < 0x100; i++) {
		*(volatile unsigned int *)(INNO_CODEC_BASE + 0x21*4) = i;
		udelay(200);
	}

	/*soft reset*/
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x00*4) = 0x0;
	udelay(200);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x00*4) = 0x3;
}

static int inno_adc_configure(struct snd_soc_component *component,
			struct snd_pcm_hw_params *params)
{
	int fmt_width = snd_pcm_format_width(params_format(params));
	unsigned int reg = 0x00;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x03*4);
	reg |= (0x1 << 1);
	reg &= ~(0x1 << 0);
	if(fmt_width > 16) {
		reg |= (0x3 << 2);
	} else if(fmt_width > 8) {
		reg |= (0x0 << 2);
	}
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x03*4) = reg;

	reg = 0x10;
	if(fmt_width == 32) {
		reg |= (0x3 << 5);
	} else if(fmt_width == 24) {
		reg |= (0x2 << 5);
	} else if(fmt_width == 20)  {
		reg |= (0x1 << 5);
	} else if(fmt_width == 16){
		reg |= (0x0 << 5);
	} else {
		dev_err(component->dev, "icodec not support format width\n");
		return -EINVAL;
	}
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x02*4) = reg;

	return 0;
}



static void inno_adc_enable (struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int reg = 0x00;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg |= (1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	/*add  enable signal  of  current source for ADC */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) ;
	reg |= (1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;

	/* set adc current value*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) ;
	reg &= ~(0xf<<0);
	reg |= (0x8<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;


	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) ;
	reg |= (1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) ;
	reg |= (1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;


/**3***/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg |= (1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	/*set high filter */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) ;
	reg &= ~(0x7);
	reg |= 0x1;
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) = reg;

	/*set gain of mic module in ADCL, invalid*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) ;
	reg &= ~(3<<6);
	reg |= (3<<6);  //30db
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	/* slecet 0x25[0:4] set gain */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) ;
	reg &= ~(1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) = reg;

	/* set gain of ALC */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x25*4) ;
	reg &= ~(0x1f<<0);
	reg |= (icodec->alcl_gain << 0);   //28.5db
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x25*4) = reg;

#if 0
	/* slecet ALCL module to control gain*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) ;
	reg |= (1<<5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0xa*4) = reg;

	/*clwang test  do nothing */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x49*4) ;
	reg |= (1<<6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x49*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x43*4) ;
	reg &= ~(0x1f<<0);
	reg |= (0x1f<<0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x43*4) = reg;

#endif

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) ;
	reg |= (1<<3);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;


	/*set maicbias*/
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) ;
	reg &= ~(0x7<<0);
	reg |= (icodec->micbias << 0);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;

	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg |= (1<<4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	/* close adc mute */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg &= ~(1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	udelay(100);

	/* open adc mute */
	reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) ;
	reg |= (1<<7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;
}

static void inno_adc_disable (struct snd_soc_component *component)
{
/***1*/
	unsigned int reg = 0x00;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4);
	reg &= ~(0x1 << 4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4);
	reg &= ~(0x1 << 5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4);
	reg &= ~(0x1 << 6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4);
	reg &= ~(0x1 << 5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4);
	reg &= ~(0x1 << 4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x24*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4);
	reg &= ~(0x1 << 5);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4);
	reg &= ~(0x1 << 4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x22*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4);
	reg &= ~(0x1 << 4);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4);
	reg &= ~(0x1 << 7);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x26*4) = reg;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4);
	reg &= ~(0x1 << 6);
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x23*4) = reg;
}

static int icodec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (icodec->powered)
		return 0;
	icodec->powered = true;

	inno_poweron(component);

	return 0;
}

static int icodec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{


	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		inno_codec_master = true;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	/*codec frame slave */
		inno_codec_master = false;
		break;
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int icodec_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	unsigned int reg = 0x00;
	int ret;

	reg = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x03*4);
	if(inno_codec_master) {
		reg |= (0xf << 4);
	}else{
		reg &= ~(0xf << 4);
	}
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x03*4) = reg;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = inno_dac_configure(component, params);
	} else {
		ret = inno_adc_configure(component, params);
	}

	return ret;
}

static int icodec_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if(g_icodec_playback_pwr != NULL)
				(*g_icodec_playback_pwr)(1);
			inno_dac_enable(component);

			if(icodec_mute == 1){
				inno_dac_disable(component);
			}
		} else {
			inno_adc_enable(component);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if(g_icodec_playback_pwr != NULL)
				(*g_icodec_playback_pwr)(0);
			inno_dac_disable(component);
			mdelay(25);
		} else {
			inno_adc_disable(component);
		}
		break;
	}
//	dump_regs(component);
	return 0;
}


static void icodec_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (!icodec->powered || snd_soc_component_active(component))
		return;
	icodec->powered = false;

	inno_poweroff(component);
}

static struct snd_soc_dai_ops icodec_dai_ops = {
	.hw_params	= icodec_hw_params,
	.set_fmt	= icodec_set_dai_fmt,
	.trigger	= icodec_trigger,
	.shutdown	= icodec_shutdown,
	.startup	= icodec_startup,
};

static struct snd_soc_dai_driver  icodec_codec_dai = {
	.name = "icodec",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = ICODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = ICODEC_FORMATS,
	},
	.ops = &icodec_dai_ops,
};

#ifdef CONFIG_PM
static int icodec_suspend(struct snd_soc_component *component)
{
	return 0;
}

static int icodec_resume(struct snd_soc_component *component)
{
	return 0;
}
#endif

static int icodec_probe(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	dev_info(component->dev, "icodec probe enter\n");

	icodec->component = component;
	return 0;
}

static void icodec_remove(struct snd_soc_component *component)
{
	dev_info(component->dev, "codec icodec remove enter\n");
}

static int icodec_controls_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if(!strcmp(kcontrol->id.name, "ICODEC MICL GAIN")) {
		ucontrol->value.integer.value[0] = icodec->alcl_gain;
	}
	if(!strcmp(kcontrol->id.name, "ICODEC HPOUTL GAIN")) {
		ucontrol->value.integer.value[0] = icodec->hpoutl_gain;
	}
	if(!strcmp(kcontrol->id.name, "ICODEC MICBIAS")) {
		ucontrol->value.integer.value[0] = icodec->micbias;
	}

	return 0;
}

static int icodec_controls_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int reg = 0x00;

	if(!strcmp(kcontrol->id.name, "ICODEC MICL GAIN")) {
		icodec->alcl_gain = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x25*4) ;
			reg &= ~(0x1f<<0);
			reg |= (icodec->alcl_gain << 0);
			*(volatile unsigned int *)(INNO_CODEC_BASE + 0x25*4) = reg;
		}
	}
	if(!strcmp(kcontrol->id.name, "ICODEC HPOUTL GAIN")) {
		icodec->hpoutl_gain = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			reg  = *(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) ;
			reg &= ~(0x1f<<0);
			reg |= (icodec->hpoutl_gain << 0);
			*(volatile unsigned int *)(INNO_CODEC_BASE + 0x28*4) = reg;
		}

		if(!icodec->hpoutl_gain){
			icodec_mute = 1;
			inno_dac_disable(component);

		}else if(icodec_mute ==1 && icodec->hpoutl_gain){
			inno_dac_enable(component);
			icodec_mute = 0;
		}
	}

	if(!strcmp(kcontrol->id.name, "ICODEC MICBIAS")) {
		icodec->micbias = ucontrol->value.integer.value[0];
	}

	return 0;
}

static int icodec_spk_power(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (g_icodec_playback_pwr == NULL)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		(*g_icodec_playback_pwr)(1);
	} else {
		(*g_icodec_playback_pwr)(0);
	}
	return 0;
}


/*
 * ALCL GAIN volume control:
 * from -18dB to 28.5dB in 1.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(alcl_gain_tlv, -1800, 150, 0);

/*
 * HPOUTL GAIN volume control:
 * from -39dB to 6dB in 1.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(hpoutl_gain_tlv, -3900, 150, 0);

static const struct snd_kcontrol_new icodec_controls[] = {
#if 1
	SOC_SINGLE_EXT_TLV("ICODEC MICL GAIN", 0, 0, 0x1f, 0 , icodec_controls_get, icodec_controls_put, alcl_gain_tlv),
	SOC_SINGLE_EXT_TLV("ICODEC HPOUTL GAIN", 0, 0, 0x1f, 0 , icodec_controls_get, icodec_controls_put, hpoutl_gain_tlv),
	SOC_SINGLE_EXT("ICODEC MICBIAS", 0, 0, 0x7, 0 , icodec_controls_get, icodec_controls_put),
#else
	SOC_SINGLE_TLV("ICODEC MICL GAIN", 0x25*4, 0, 0x1f, 0, alcl_gain_tlv),
	SOC_SINGLE_TLV("ICODEC HPOUTL GAIN", 0x28*4, 0, 0x1f, 0 , hpoutl_gain_tlv),
	SOC_SINGLE("ICODEC MICBIAS", 0x22*4, 0, 0x7, 0),
#endif
};

static const struct snd_soc_dapm_widget icodec_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),/*pw*/
	SND_SOC_DAPM_ADC("ADCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ALCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SPK("Speaker", icodec_spk_power),
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_INPUT("MICL"),
};

static const struct snd_soc_dapm_route icodec_route[] = {
	{"DACL",		NULL,	"SDTI"},
	{"HPOUTL",		NULL,	"DACL"},
	{"Speaker",		NULL,	"HPOUTL"},
	{"SDTO",		NULL,	"ADCL"},
	{"ADCL",		NULL,	"ALCL"},
	{"ALCL",		NULL,	"MICL"},
};

static struct snd_soc_component_driver soc_component_dev_icodec = {
	.probe = 	icodec_probe,
	.remove = 	icodec_remove,
#ifdef CONFIG_PM
	.suspend =	icodec_suspend,
	.resume =	icodec_resume,
#endif

	.read = 	icodec_read,
	.write = 	icodec_write,

	.controls = 	icodec_controls,
	.num_controls = ARRAY_SIZE(icodec_controls),
	.dapm_widgets = icodec_widgets,
	.num_dapm_widgets = ARRAY_SIZE(icodec_widgets),
	.dapm_routes = icodec_route,
	.num_dapm_routes = ARRAY_SIZE(icodec_route),
};

static int icodec_platform_probe(struct platform_device *pdev)
{
	struct icodec *icodec = NULL;
	struct resource *res;
	enum of_gpio_flags flags;
	int ret = 0;

	icodec = (struct icodec*)devm_kzalloc(&pdev->dev,
			sizeof(struct icodec), GFP_KERNEL);
	if (!icodec)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	icodec->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(icodec->io_base)) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return PTR_ERR(icodec->io_base);
	}

	icodec->dev = &pdev->dev;
	icodec->alcl_gain = 0x1f;
	icodec->hpoutl_gain = 0x1f;
	icodec->micbias = 0;
	printk(" alcl_gain = 0x%x hpoutl_gain = 0x%x micbias = 0x%x\n", icodec->alcl_gain, icodec->hpoutl_gain, icodec->micbias);
	spin_lock_init(&icodec->io_lock);
	platform_set_drvdata(pdev, (void *)icodec);

	ret = snd_soc_register_component(&pdev->dev,
			&soc_component_dev_icodec, &icodec_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register component\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	dev_info(&pdev->dev, "component icodec platfrom probe success\n");
	return 0;
}

static int icodec_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "component icodec platform remove\n");
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id component_dt_match[] = {
	{ .compatible = "ingenic,icodec", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, component_dt_match);

static struct platform_driver icodec_component_driver = {
	.driver = {
		.name = "icodec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(component_dt_match),
	},
	.probe = icodec_platform_probe,
	.remove = icodec_platform_remove,
};


static int icodec_modinit(void)
{
	return platform_driver_register(&icodec_component_driver);
}
module_init(icodec_modinit);

static void icodec_exit(void)
{
	platform_driver_unregister(&icodec_component_driver);
}
module_exit(icodec_exit);

MODULE_DESCRIPTION("Icodec Codec Driver");
MODULE_AUTHOR("tjyang<taojiang.yang@ingenic.com>");
MODULE_LICENSE("GPL");
