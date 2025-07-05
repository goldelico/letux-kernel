/*
 * sound/soc/ingenic/icodec/icdc_inno_v2.c
 * ALSA SoC Audio driver -- ingenic internal codec (icodec) driver

 * Copyright 2019 Ingenic Semiconductor Co.,Ltd
 *	zhxiao <zhihao.xiao@ingenic.com>
 *
 * Note: icodec is an internal codec for ingenic SOC
 *	 used for x2500
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


#define INNO_CODEC_BASE 0xb0021000
static bool	    icodec_codec_master = true;

#define ICODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_LE)


#define TS_CODEC_CGR_00			0x00
#define TS_CODEC_CACR_02		0x08
#define TS_CODEC_CACR2_03       0x0c
#define TS_CODEC_CDCR1_04       0x10
#define TS_CODEC_CDCR2_05       0x14
#define TS_CODEC_CDCR2_06       0x18
#define TS_CODEC_CAVR_08        0x20
#define TS_CODEC_CGAINLR_09     0x24
#define TS_CODEC_CGAINLR_0a     0x28
#define TS_CODEC_POWER_20       0x80
#define TS_CODEC_CCR_21         0x84
#define TS_CODEC_CAACR_22       0x88
#define TS_CODEC_CMICCR_23      0x8c
#define TS_CODEC_CMICGAINR_24   0x90
#define TS_CODEC_CALCGR_25      0x94
#define TS_CODEC_CANACR_26      0x98
#define TS_CODEC_CANACR2_27     0x9c
#define TS_CODEC_CHR_28         0xA0
#define TS_CODEC_CHR_29         0xA4
#define TS_CODEC_CHR_2a         0xA8
#define TS_CODEC_CHR_2b         0xAC
#define TS_CODEC_CMR_40			0x100
#define TS_CODEC_CTR_41			0x104
#define TS_CODEC_CAGCCR_42		0x108
#define TS_CODEC_CPGR_43		0x10c
#define TS_CODEC_CSRR_44		0x110
#define TS_CODEC_CALMR_45		0x114
#define TS_CODEC_CAHMR_46		0x118
#define TS_CODEC_CALMINR_47		0x11c
#define TS_CODEC_CAHMINR_48		0x120
#define TS_CODEC_CAFG_49		0x124
#define TS_CODEC_CCAGVR_4c		0x130
#define TS_CODEC_CMR_50			0x140
#define TS_CODEC_CTR_51			0x144
#define TS_CODEC_CAGCCR_52		0x148
#define TS_CODEC_CPGR_53		0x14c
#define TS_CODEC_CSRR_54		0x150
#define TS_CODEC_CALMR_55		0x154
#define TS_CODEC_CAHMR_56		0x158
#define TS_CODEC_CALMINR_57		0x15c
#define TS_CODEC_CAHMINR_58		0x160
#define TS_CODEC_CAFG_59		0x164
#define TS_CODEC_CCAGVR_5c		0x170


#define MONO_LEFT   1
#define MONO_RIGHT  2
#define STEREO 3


struct icodec {
	struct device		*dev;
	struct snd_soc_component	*component;
	spinlock_t		io_lock;
	void * __iomem io_base;
	bool powered;
	int spk_gpio;
	int sgm8903_gpio;
	unsigned char spk_en_level;
	unsigned char alcl_gain;
	unsigned char alcr_gain;
	unsigned char micl_dgain;
	unsigned char micr_dgain;
	unsigned char hpoutl_again;
	unsigned char hpoutl_dgain;
	unsigned char micbias;
	unsigned char michpf;
	unsigned char channel;
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

static unsigned int icodec_read(struct snd_soc_component *component , unsigned int reg)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	int value = readl(icodec->io_base + reg);
	return value;
}

static int icodec_write(struct snd_soc_component *component , unsigned int reg , unsigned value)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	writel(value ,icodec->io_base + reg);
	return 0;
}


static inline unsigned int icodec_reg_read(struct icodec *icodec, int offset)
{
	return readl(icodec->io_base + offset);
}

static inline void icodec_reg_write(struct icodec *icodec, int offset, int data)
{
	writel(data, icodec->io_base + offset);
}

int icodec_reg_set(struct icodec *icodec, unsigned int reg, int start, int end, int val)
{
	int ret = 0;
	int i = 0, mask = 0;
	unsigned int oldv = 0;
	unsigned int new = 0;
	for (i = 0;  i < (end - start + 1); i++) {
		mask += (1 << (start + i));
	}

	oldv = icodec_reg_read(icodec, reg);
	new = oldv & (~mask);
	new |= val << start;
	icodec_reg_write(icodec, reg, new);

	if ((new & 0x000000FF) != icodec_reg_read(icodec, reg)) {
		printk("%s(%d):codec write  0x%08x error!!\n",__func__, __LINE__ ,reg);
		printk("new = 0x%08x, read val = 0x%08x\n", new, icodec_reg_read(icodec, reg));
		return -1;
	}

	return ret;
}

static void icodec_poweron(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	int i = 0;
	char value = 0;
	/* step1. reset codec */
	icodec_reg_set(icodec, TS_CODEC_CGR_00, 0, 1, 0);
	msleep(1);
	icodec_reg_set(icodec, TS_CODEC_CGR_00, 0, 1, 0x3);
	/* step2. setup dc voltags of DAC channel output */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 0, 1, 0x1);
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 4, 5, 0x1);
	/* step3. no desc */
	icodec_reg_set(icodec, TS_CODEC_CCR_21, 0, 7, 0x1);
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 5, 5, 0x1);//setup reference voltage

	for (i = 0; i <= 7; i++) {
		value |= value<<1 | 0x01;
		icodec_reg_set(icodec, TS_CODEC_CCR_21, 0, 7, value);
		msleep(20);
	}
	msleep(20);
	icodec_reg_set(icodec, TS_CODEC_CCR_21, 0, 7, 0x02);//Set the min charge current for reduce power.

}

static int icodec_set_sample_rate(struct icodec *icodec, unsigned int rate)
{
	int i = 0;
	unsigned int mrate[8] = {8000, 12000, 16000, 24000, 32000, 44100, 48000, 96000};
	unsigned int rate_fs[8] = {7, 6, 5, 4, 3, 2, 1, 0};
	for (i = 0; i < 8; i++) {
		if (rate <= mrate[i])
			break;
	}
	if (i == 8)
		i = 0;
	icodec_reg_set(icodec, TS_CODEC_CSRR_44, 0, 2, rate_fs[i]);
	icodec_reg_set(icodec, TS_CODEC_CSRR_54, 0, 2, rate_fs[i]);

	return 0;
}

static int icodec_dac_configure(struct snd_soc_component *component,
			struct snd_pcm_hw_params *params)
{
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int reg = 0x00;

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
	icodec_reg_set(icodec,TS_CODEC_CDCR1_04, 5, 6, reg);


	return 0;
}

static void icodec_dac_enable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* DAC i2S interface */
	icodec_reg_set(icodec, TS_CODEC_CACR2_03, 6, 7, 0x3);
	icodec_reg_set(icodec, TS_CODEC_CDCR1_04, 0, 7, 0x10);
	icodec_reg_set(icodec, TS_CODEC_CDCR2_05, 0, 7, 0x0e);

	/* enable current source */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 7, 7, 0x1);
	icodec_reg_set(icodec, TS_CODEC_POWER_20, 0, 3, 0x07);

	/* enable reference voltage */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 6, 6, 0x1);

	/* enable POP sound */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 4, 5, 0x2);

	/* enable HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 4, 4, 1);

	/* end initialization HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 5, 5, 1);

	/* enable reference voltage */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 3, 3, 1);

	/* enable DACL clock module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 2, 2, 1);

	/* enable DACL module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 1, 1, 1);

	/* end initialization DAC module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 0, 0, 1);

	/* end mute station DRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 6, 6, 1);

	/* set default again of HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2b, 0, 4, icodec->hpoutl_again);

	/* set default dgain of HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CDCR2_06, 0, 7, icodec->hpoutl_dgain);
}

static void icodec_dac_disable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	icodec_reg_set(icodec, TS_CODEC_CHR_2b, 0, 4, 0);

	/* mute HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 6, 6, 0);

	/* initialize DAC module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 5, 5, 0);

	/* disable HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_2a, 4, 4, 0);

	/* disable DACL module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 1, 1, 0);

	/* disable DACL clock module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 2, 2, 0);

	/* disable DACL reference voltage */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 3, 3, 0);

	/* initialize POP sound */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 4, 5, 0x1);

	/* disable reference voltage buffer */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 6, 6, 0);

	/* disable current source DAC */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 7, 7, 0);

	/* begin initialization HPDRV module */
	icodec_reg_set(icodec, TS_CODEC_CHR_29, 0, 0, 0);
}



static int icodec_adc_configure(struct snd_soc_component *component,
			struct snd_pcm_hw_params *params)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int fmt_width = snd_pcm_format_width(params_format(params));
	unsigned int samplerate = params_rate(params);
	unsigned int channel = params_channels(params);
	unsigned int reg = 0x00;

	if(channel == 2)
		icodec->channel = STEREO;
	else if(channel == 1) {
#ifdef CONFIG_AUDIO_MONO_LEFT
		icodec->channel = MONO_LEFT;
#endif
#ifdef CONFIG_AUDIO_MONO_RIGHT
		icodec->channel = MONO_RIGHT;
#endif
	} else {
		dev_err(component->dev, "channel is not support !\n");
	}

	reg = 0x10;
	if(fmt_width == 32) {
		reg |= (0x3 << 5);
	} else if(fmt_width == 24) {
		reg |= (0x2 << 5);
	} else if(fmt_width == 20) {
		reg |= (0x1 << 5);
	} else if(fmt_width == 16){
		reg |= (0x0 << 5);
	} else {
		dev_err(component->dev, "icodec not support format width\n");
		return -EINVAL;
	}
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 5, 6, reg);

	if(samplerate > 96000 || samplerate < 8000) {
		dev_err(component->dev, "icodec not support samplerate %d \n", samplerate);
		return -EINVAL;
	}
	icodec_set_sample_rate(icodec, samplerate);

	return 0;
}

/* ADC left enable configuration */
static int icodec_enable_left_record(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* select ADC mono mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 0, 0, 1);//mono

	/* configure ADC I2S interface mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 3, 4, 0x2);
	icodec_reg_set(icodec, TS_CODEC_CACR2_03, 0, 5, 0x3e);

	/* end mute station of ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 7, 7, 1);

	/* enable current source of audio */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 1);

	/* bias current:reduce power,max is 7*/
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 7);

	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 3, 3, 1);
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 0, 2, icodec->micbias);

	/* enable reference voltage buffer in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 5, 5, 1);

	/* enable MIC module in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 2, 2, 1);

	/* enable ALC module in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 3, 3, 1);

	/* enable clock module in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 6, 6, 1);

	/* enable ADC module in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 5, 5, 1);

	/* end initialization of ADCL module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 4, 4, 1);

	/* end initialization of ALC left module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 7, 7, 1);

	/* end initialization of mic left module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 6, 6, 1);

	/* set default mic gain of left module */
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 6, 7, 3);

	/* set default codec gain of left module */
	icodec_reg_set(icodec, TS_CODEC_CANACR_26, 0, 4, icodec->alcl_gain);

	/* enable zero-crossing detection function in ADC left channel */
	//if enable zero-crossing detection function,will cause small voice at first.
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 4, 4, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 3);

	return 0;
}


/* ADC left disable cofiguration */
static int icodec_disable_left_record(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* disable zero-crossing detection function in ADC left channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 4, 4, 0);

	/* disable ADC module for left channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 5, 5, 0);

	icodec_reg_set(icodec, TS_CODEC_CHR_28, 6, 6, 0);

	/* disable ALC module for left channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 5, 5, 0);

	/* disable MIC module for left channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 4, 4, 0);

	/* disable reference voltage buffer for left channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 5, 5, 0);

	/* disable mic bias voltage buffer */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 0);

	/* begin initiialization ADC left module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 4, 4, 0);

	/* begin initialization ALC left module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 7, 7, 0);

	/* begin initialization MIC left module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 6, 6, 0);
//	msleep(20);

	return 0;
}

/* ADC right enable configuration */
static int icodec_enable_right_record(struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	/* select ADC stereo mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 0, 0, 0);

	/* configure ADC I2S interface mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 3, 4, 0x2);
	icodec_reg_set(icodec, TS_CODEC_CACR2_03, 0, 5, 0x3e);

	/* end mute station of ADC  right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 7, 7, 1); //left
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 3, 3, 1); //right

	/* enable current source of audio */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 1);
	/* bias current:reduce power,max is 7*/
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 7);//add 10:57

	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 3, 3, 1);
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 0, 2, icodec->micbias);

	/* enable reference voltage buffer in ADC right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 5, 5, 1);	//left
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 1, 1, 1); //right

	/* enable MIC module in ADC right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 0, 0, 1);	//right
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 2, 2, 1);

	/* enable ALC module in ADC right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 1, 1, 1);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 3, 3, 1);

	/* enable clock module in ADC  right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 2, 2, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 6, 6, 1);

	/* enable ADC module in ADC right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 1, 1, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 5, 5, 1);

	/* end initialization of ADCR module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 0, 0, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 4, 4, 1);

	/* end initialization of ALC right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 3, 3, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 7, 7, 1);

	/* end initialization of mic right module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 2, 2, 1);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 6, 6, 1);

	/* set default mic gain of right module */
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 4, 5, 3);
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 6, 7, 3);

	/* set default codec gain of right module */
	icodec_reg_set(icodec, TS_CODEC_CANACR_26, 0, 4, icodec->alcl_gain);
	icodec_reg_set(icodec, TS_CODEC_CANACR2_27, 0, 4, icodec->alcr_gain);

	/* disable zero-crossing detection function in ADC right channel */
	//if enable zero-crossing detection function,will cause small voice at first.
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 0, 0, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 4, 4, 0);


	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 0, 7, 0x0e);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 0, 7, 0x03);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 0, 7, 0x0F);
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 3);
	/* now begin recording... */

	return 0;
}

/* ADC right disable cofiguration */
static int icodec_disable_right_record(struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	/* disable zero-crossing detection function in ADC right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 0, 0, 0);

	/* disable ADC module for right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 1, 1, 0);

	icodec_reg_set(icodec, TS_CODEC_CHR_28, 2, 2, 0);

	/* disable ALC module for right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 1, 1, 0);

	/* disable MIC module for right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 0, 0, 0);

	/* disable reference voltage buffer for right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 1, 1, 0);

	/* disable mic bias voltage buffer */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 0);

	/* begin initiialization ADC right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 0, 0, 0);

	/* begin initialization ALC right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 3, 3, 0);

	/* begin initialization MIC right module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 2, 2, 0);

	return 0;
}

/* ADC stereo enable configuration */
static int icodec_enable_stereo_record(struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	/* select ADC stereo mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 0, 0, 0);

	/* configure ADC I2S interface mode */
	icodec_reg_set(icodec, TS_CODEC_CACR_02, 3, 4, 0x2);
	icodec_reg_set(icodec, TS_CODEC_CACR2_03, 0, 5, 0x3e);

	/* end mute station of ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 7, 7, 1);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 3, 3, 1);

	/* enable current source of audio */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 1);
	/* bias current:reduce power,max is 7*/
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 7);//add 10:57

	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 3, 3, 1);
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 0, 2, icodec->micbias);

	/* enable reference voltage buffer in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 5, 5, 1);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 1, 1, 1);

	/* enable MIC module in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 0, 0, 1);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 2, 2, 1);

	/* enable ALC module in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 1, 1, 1);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 3, 3, 1);

	/* enable clock module in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 2, 2, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 6, 6, 1);

	/* enable ADC module in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 1, 1, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 5, 5, 1);

	/* end initialization of ADCR and ADCL module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 0, 0, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 4, 4, 1);

	/* end initialization of ALC left and right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 3, 3, 1);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 7, 7, 1);

	/* end initialization of mic left and right module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 2, 2, 1);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 6, 6, 1);

	/* set default mic gain of left and right module */
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 4, 5, 3);
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 6, 7, 3);

	/* set default icodec gain of left and right module */
	icodec_reg_set(icodec, TS_CODEC_CANACR_26, 0, 4, icodec->alcl_gain);
	icodec_reg_set(icodec, TS_CODEC_CANACR2_27, 0, 4, icodec->alcr_gain);

	/* disable zero-crossing detection function in ADC left and right channel */
	//if enable zero-crossing detection function,will cause small voice at first.
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 0, 0, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 4, 4, 0);

	/* now begin recording... */
	icodec_reg_set(icodec, TS_CODEC_CMICGAINR_24, 0, 3, 3);

	return 0;
}

/* ADC stereo disable cofiguration */
static int icodec_disable_stereo_record(struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	/* disable zero-crossing detection function in ADC left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 0, 0, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 4, 4, 0);

	/* disable ADC module for left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 1, 1, 0);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 5, 5, 0);

	icodec_reg_set(icodec, TS_CODEC_CHR_28, 2, 2, 0);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 6, 6, 0);

	/* disable ALC module for left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 1, 1, 0);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 5, 5, 0);

	/* disable MIC module for left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 0, 0, 0);
	icodec_reg_set(icodec, TS_CODEC_CALCGR_25, 4, 4, 0);

	/* disable reference voltage buffer for left and right channel */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 1, 1, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 5, 5, 0);

	/* disable mic bias voltage buffer */
	icodec_reg_set(icodec, TS_CODEC_CAACR_22, 4, 4, 0);

	/* begin initiialization ADC left and right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 0, 0, 0);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 4, 4, 0);

	/* begin initialization ALC left and right module */
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 3, 3, 0);
	icodec_reg_set(icodec, TS_CODEC_CHR_28, 7, 7, 0);

	/* begin initialization MIC left and right module */
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 2, 2, 0);
	icodec_reg_set(icodec, TS_CODEC_CMICCR_23, 6, 6, 0);

	return 0;
}

static void icodec_adc_enable (struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	if(icodec->channel == STEREO)
		icodec_enable_stereo_record(codec);
	else if(icodec->channel == MONO_LEFT)
		icodec_enable_left_record(codec);
	else if(icodec->channel == MONO_RIGHT)
		icodec_enable_right_record(codec);
	else
		dev_err(codec->dev, "%s channel not support\n",__func__);
}

static void icodec_adc_disable (struct snd_soc_component *codec)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	if(icodec->channel == STEREO)
		icodec_disable_stereo_record(codec);
	else if(icodec->channel == MONO_LEFT)
		icodec_disable_left_record(codec);
	else if(icodec->channel == MONO_RIGHT)
		icodec_disable_right_record(codec);
	else
		dev_err(codec->dev, "%s channel not support\n",__func__);
}

static void icodec_poweroff(struct snd_soc_component *codec)
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

static int icodec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (icodec->powered)
		return 0;
	icodec->powered = true;

	icodec_poweron(component);

	return 0;
}

static int icodec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		icodec_codec_master = true;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	/*codec frame slave */
		icodec_codec_master = false;
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
	if(icodec_codec_master) {
		reg |= (0x3 << 4);
	}else{
		reg &= ~(0x3 << 4);
	}
	*(volatile unsigned int *)(INNO_CODEC_BASE + 0x03*4) = reg;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = icodec_dac_configure(component, params);
	} else {
		ret = icodec_adc_configure(component, params);
	}

	return ret;
}

static int icodec_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			icodec_dac_enable(codec);
			if(g_icodec_playback_pwr != NULL)
				(*g_icodec_playback_pwr)(1);
		} else {
			icodec_adc_enable(codec);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if(g_icodec_playback_pwr != NULL)
				(*g_icodec_playback_pwr)(0);
			icodec_dac_disable(codec);
		} else {
			icodec_adc_disable(codec);
		}
		break;
	}

	return 0;
}


static void icodec_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(codec);

	if (!icodec->powered || snd_soc_component_active(codec))
		return;
	icodec->powered = false;

	icodec_poweroff(codec);
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
static int icodec_suspend(struct snd_soc_component *codec)
{
	return 0;
}

static int icodec_resume(struct snd_soc_component *codec)
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

	if(!strcmp(kcontrol->id.name, "HPOUTL AGAIN")) {
		ucontrol->value.integer.value[0] = icodec->hpoutl_again;
	}
	if(!strcmp(kcontrol->id.name, "HPOUTL DGAIN")) {
		ucontrol->value.integer.value[0] = icodec->hpoutl_dgain;
	}
	if(!strcmp(kcontrol->id.name, "MICBIAS")) {
		ucontrol->value.integer.value[0] = icodec->micbias;
	}
	if(!strcmp(kcontrol->id.name, "MIC HPF SWITCH")) {
		ucontrol->value.integer.value[0] = icodec->michpf;
	}
	if(!strcmp(kcontrol->id.name, "Left ADC Digital Volume")) {
		ucontrol->value.integer.value[0] = icodec->micl_dgain;
	}
	if(!strcmp(kcontrol->id.name, "Right ADC Digital Volume")) {
		ucontrol->value.integer.value[0] = icodec->micr_dgain;
	}
	if(!strcmp(kcontrol->id.name, "ALCL GAIN")) {
		ucontrol->value.integer.value[0] = icodec->alcl_gain;
	}
	if(!strcmp(kcontrol->id.name, "ALCR GAIN")) {
		ucontrol->value.integer.value[0] = icodec->alcr_gain;
	}

	return 0;
}

static int icodec_controls_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if(!strcmp(kcontrol->id.name, "Left ADC Digital Volume")) {
		icodec->micl_dgain = ucontrol->value.integer.value[0];
			icodec_reg_set(icodec, TS_CODEC_CAVR_08, 0, 7, icodec->micl_dgain);
	}
	if(!strcmp(kcontrol->id.name, "Right ADC Digital Volume")) {
		icodec->micr_dgain = ucontrol->value.integer.value[0];
			icodec_reg_set(icodec, TS_CODEC_CGAINLR_09, 0, 7, icodec->micr_dgain);
	}
	if(!strcmp(kcontrol->id.name, "HPOUTL DGAIN")) {
		icodec->hpoutl_dgain = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			icodec_reg_set(icodec, TS_CODEC_CDCR2_06, 0, 7, icodec->hpoutl_dgain);
		}
	}
	if(!strcmp(kcontrol->id.name, "HPOUTL AGAIN")) {
		icodec->hpoutl_again = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			icodec_reg_set(icodec, TS_CODEC_CHR_2b, 0, 4, icodec->hpoutl_again);
		}
	}
	if(!strcmp(kcontrol->id.name, "MICBIAS")) {
		icodec->micbias = ucontrol->value.integer.value[0];
	}
	if(!strcmp(kcontrol->id.name, "MIC HPF SWITCH")) {
		icodec->michpf = ucontrol->value.integer.value[0];
			icodec_reg_set(icodec, TS_CODEC_CGAINLR_0a, 2, 2, icodec->michpf);
	}
	if(!strcmp(kcontrol->id.name, "ALCL GAIN")) {
		icodec->alcl_gain = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			icodec_reg_set(icodec, TS_CODEC_CANACR_26, 0, 4, icodec->alcl_gain);
		}
	}
	if(!strcmp(kcontrol->id.name, "ALCR GAIN")) {
		icodec->alcr_gain = ucontrol->value.integer.value[0];
		if(snd_soc_component_active(component)) {
			icodec_reg_set(icodec, TS_CODEC_CANACR2_27, 0, 4, icodec->alcr_gain);
		}
	}

	return 0;
}

static int icodec_spk_power(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if ((!gpio_is_valid(icodec->spk_gpio)) || (!gpio_is_valid(icodec->sgm8903_gpio)))
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
			gpio_direction_output(icodec->spk_gpio, icodec->spk_en_level);
			gpio_direction_output(icodec->sgm8903_gpio, icodec->spk_en_level);
		} else {
			gpio_direction_output(icodec->spk_gpio, !icodec->spk_en_level);
			gpio_direction_output(icodec->sgm8903_gpio, !icodec->spk_en_level);
	}
	return 0;
}



/*
 * ALCL GAIN volume control:
 * from -18dB to 28.5dB in 1.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(alcl_gain_tlv, -1800, 150, 0);

/*
 * ALCR GAIN volume control:
 * from -18dB to 28.5dB in 1.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(alcr_gain_tlv, -1800, 150, 0);

/*
 * MICL DGAIN volume control:
 * from -97dB to 30dB in 0.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(micl_dgain_tlv, -9700, 50, 0);

/*
 * MICL DGAIN volume control:
 * from -97dB to 30dB in 0.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(micr_dgain_tlv, -9700, 50, 0);

/*
 * HPOUTL AGAIN volume control:
 * from -39dB to 6dB in 1.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(hpoutl_again_tlv, -3900, 150, 0);

/*
 * HPOUTL DGAIN volume control:
 * from -120dB to 7dB in 0.5 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(hpoutl_dgain_tlv, -12000, 50, 0);

static const struct snd_kcontrol_new icodec_controls[] = {
#if 1
	SOC_SINGLE_EXT_TLV("ALCL GAIN", TS_CODEC_CANACR_26, 0, 0x1f, 0 , icodec_controls_get, icodec_controls_put, alcl_gain_tlv),
	SOC_SINGLE_EXT_TLV("ALCR GAIN", TS_CODEC_CANACR2_27, 0, 0x1f, 0 , icodec_controls_get, icodec_controls_put, alcr_gain_tlv),
//	SOC_SINGLE_EXT_TLV("Left ADC Digital Volume", TS_CODEC_CAVR_08, 0, 0xff, 0 , icodec_controls_get, icodec_controls_put, micl_dgain_tlv),
//	SOC_SINGLE_EXT_TLV("Right ADC Digital Volume", TS_CODEC_CGAINLR_09, 0, 0xff, 0 , icodec_controls_get, icodec_controls_put, micr_dgain_tlv),
	SOC_SINGLE_EXT_TLV("HPOUTL DGAIN", TS_CODEC_CDCR2_06, 0, 0xff, 0 , icodec_controls_get, icodec_controls_put, hpoutl_dgain_tlv),
	SOC_SINGLE_EXT_TLV("HPOUTL AGAIN", TS_CODEC_CHR_2b, 0, 0x1f, 0 , icodec_controls_get, icodec_controls_put, hpoutl_again_tlv),
	SOC_SINGLE_EXT("MICBIAS", TS_CODEC_CAACR_22, 0, 0x7, 0 , icodec_controls_get, icodec_controls_put),
	SOC_SINGLE_EXT("MIC HPF SWITCH", TS_CODEC_CGAINLR_0a, 2, 1, 0 , icodec_controls_get, icodec_controls_put),
#else
	SOC_SINGLE_TLV("ALCL GAIN MAX", TS_CODEC_CAFG_49, 3, 7, 0 , alcl_gain_max_tlv),
	SOC_SINGLE_TLV("ALCL GAIN MIN", TS_CODEC_CAFG_49, 0, 7, 0 , alcl_gain_min_tlv),
	SOC_SINGLE_TLV("ALCR GAIN MAX", TS_CODEC_CAFG_59, 3, 7, 0 , alcr_gain_max_tlv),
	SOC_SINGLE_TLV("ALCR GAIN MIN", TS_CODEC_CAFG_59, 0, 7, 0 , alcr_gain_min_tlv),
	SOC_SINGLE_TLV("ALCL GAIN", TS_CODEC_CANACR_26, 0, 0x1f, 0 ,alcl_gain_tlv),
	SOC_SINGLE_TLV("ALCR GAIN", TS_CODEC_CANACR2_27, 0, 0x1f, 0 ,alcr_gain_tlv),
	SOC_SINGLE_TLV("MICL DGAIN", TS_CODEC_CAVR_08, 0, 0xff, 0 ,micl_dgain_tlv),
	SOC_SINGLE_TLV("MICR DGAIN", TS_CODEC_CGAINLR_09, 0, 0xff, 0 ,micr_dgain_tlv),
	SOC_SINGLE_TLV("HPOUTL DGAIN", TS_CODEC_CDCR2_06, 0, 0xff, 0 ,hpoutl_dgain_tlv),
	SOC_SINGLE_TLV("HPOUTL AGAIN", TS_CODEC_CHR_2b, 0, 0x1f, 0 ,hpoutl_again_tlv),
	SOC_SINGLE("MICL GAIN", TS_CODEC_CMICGAINR_24, 6, 0x3, 0),
	SOC_SINGLE("MICR GAIN", TS_CODEC_CMICGAINR_24, 4, 0x3, 0),
	SOC_SINGLE("MICBIAS", TS_CODEC_CAACR_22, 0, 0x7, 0),
	SOC_SINGLE("MIC HPF SWITCH", TS_CODEC_CGAINLR_0a, 2, 1, 0),
	SOC_SINGLE_EXT("ALCL GAIN AUTO", 0, 0, 1, 0 , icodec_controls_get, icodec_controls_put),
	SOC_SINGLE_EXT("ALCR GAIN AUTO", 0, 0, 1, 0 , icodec_controls_get, icodec_controls_put),
#endif
};

static const struct snd_soc_dapm_widget icodec_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),/*pw*/
	SND_SOC_DAPM_ADC("ADCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADCR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ALCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ALCR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SPK("Speaker", icodec_spk_power),
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_INPUT("MICL"),
	SND_SOC_DAPM_INPUT("MICR"),
};

static const struct snd_soc_dapm_route icodec_route[] = {
	{"DACL",		NULL,	"SDTI"},
	{"HPOUTL",		NULL,	"DACL"},
	{"Speaker",		NULL,	"HPOUTL"},
	{"SDTO",		NULL,	"ADCL"},
	{"ADCL",		NULL,	"ALCL"},
	{"ALCL",		NULL,	"MICL"},
	{"SDTO",		NULL,	"ADCR"},
	{"ADCR",		NULL,	"ALCR"},
	{"ALCR",		NULL,	"MICR"},
};

static struct snd_soc_component_driver soc_codec_dev_icodec_component = {
	.probe = 	icodec_probe,
	.remove = 	icodec_remove,
#ifdef CONFIG_PM
	.suspend =	icodec_suspend,
	.resume =	icodec_resume,
#endif

	.read = 	icodec_read,
	.write = 	icodec_write,
	//.reg_word_size = sizeof(u32),

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

	icodec->spk_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "ingenic,spken-gpio", 0, &flags);
	icodec->sgm8903_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "ingenic,sgm8903en-gpio", 0, &flags);
	if (gpio_is_valid(icodec->spk_gpio) && gpio_is_valid(icodec->sgm8903_gpio)) {
		unsigned long init_flags;
		icodec->spk_en_level = (flags == OF_GPIO_ACTIVE_LOW ? 0 : 1);
		init_flags = (flags == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW);
		ret = devm_gpio_request_one(&pdev->dev, icodec->spk_gpio, init_flags, "Speaker_en");
		if (ret)
			pr_warn("Speaker enable pin(%d) request failed\n",icodec->spk_gpio);
		else
			pr_info("Speaker enable pin(%d) request ok\n", icodec->spk_gpio);

		ret = devm_gpio_request_one(&pdev->dev, icodec->sgm8903_gpio, init_flags, "Sgm8903_en");
		if (ret)
			pr_warn("icodec: SGM8903 enable pin(%d) request failed\n",icodec->sgm8903_gpio);
		else
			pr_info("icodec: SGM8903 enable pin(%d) request ok\n", icodec->sgm8903_gpio);

	}

	icodec->dev = &pdev->dev;
	icodec->alcl_gain = 0x1f;	//left record gain 0-0x1f
	icodec->alcr_gain = 0x1f;	//right record gain 0-0x1f
	icodec->hpoutl_again = 0x18;	//replay analog gain 0-0x1f
	icodec->hpoutl_dgain = 0xf1;	//replay digital gain 0-0xff
	icodec->micbias = 0;

	spin_lock_init(&icodec->io_lock);
	platform_set_drvdata(pdev, (void *)icodec);

	ret = snd_soc_register_component(&pdev->dev,
			&soc_codec_dev_icodec_component, &icodec_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register codec\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	dev_info(&pdev->dev, "codec icodec platfrom probe success\n");
	return 0;
}

static int icodec_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "codec icodec platform remove\n");
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id codec_dt_match[] = {
	{ .compatible = "ingenic,icodec", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, codec_dt_match);

static struct platform_driver icodec_codec_driver = {
	.driver = {
		.name = "icodec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(codec_dt_match),
	},
	.probe = icodec_platform_probe,
	.remove = icodec_platform_remove,
};


static int icodec_modinit(void)
{
	return platform_driver_register(&icodec_codec_driver);
}
module_init(icodec_modinit);

static void icodec_exit(void)
{
	platform_driver_unregister(&icodec_codec_driver);
}
module_exit(icodec_exit);

MODULE_DESCRIPTION("Icodec Codec Driver");
MODULE_AUTHOR("zhxiao<zhihao.xiao@ingenic.com>");
MODULE_LICENSE("GPL");
