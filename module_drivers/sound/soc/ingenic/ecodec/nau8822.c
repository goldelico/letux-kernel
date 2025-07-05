/*
 * nau8822.c  --  NAU8822 ALSA Soc Audio Codec driver
 *
 * Copyright 2015 Nuvoton Technology Corp.
 *
 * Author: David Lin <ctlin0@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include "nau8822.h"

#if 0
#define ENTER()								\
	do {								\
		printk("%03d nau8822 ENTER %s()\n", __LINE__, __PRETTY_FUNCTION__); \
	} while (0)
#define MYDBG(sss, aaa...)						\
	do {								\
		printk("%03d nau8822 debug %s() " sss "\n", __LINE__, __PRETTY_FUNCTION__, ##aaa); \
	} while (0)
#else
#define ENTER()					\
	do {					\
	} while (0)
#define LEAVE()					\
	do {					\
	} while (0)
#define MYDBG(sss, aaa...)			\
	do {					\
	} while (0)
#endif


static int nau8822_reg_read2(void *context, unsigned int reg, unsigned int *value)
{
	struct i2c_client *client = (struct i2c_client *)context;
	//struct nau8822_priv * nau8822 = (struct nau8822_priv *)i2c_get_clientdata(client);
	uint8_t buf[2] = {(reg&0xff)<<1,};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	MYDBG("ret=%x, value=%x", ret, *value);
	if(ret > 0)
		ret = 0;

	return ret;
}

static int nau8822_reg_read(struct nau8822_priv * nau8822, unsigned short reg,
		unsigned short *value)
{
	struct i2c_client *client = nau8822->i2c;
	uint8_t buf[2] = {(reg&0xff)<<1,};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	MYDBG("ret=%x, value=%x", ret, *value);
	if(ret > 0)
		ret = 0;

	return ret;
}




#define NAU_PLL_FREQ_MAX 100000000
#define NAU_PLL_FREQ_MIN 90000000
#define NAU_PLL_REF_MAX 33000000
#define NAU_PLL_REF_MIN 8000000
#define NAU_PLL_OPTOP_MIN 6

static const int nau8822_mclk_scaler[] = { 10, 15, 20, 30, 40, 60, 80, 120 };

static const struct reg_default nau8822_reg_defaults[] = {
	{ NAU8822_REG_POWER_MANAGEMENT_1, 0x0000 },
	{ NAU8822_REG_POWER_MANAGEMENT_2, 0x0000 },
	{ NAU8822_REG_POWER_MANAGEMENT_3, 0x0000 },
	{ NAU8822_REG_AUDIO_INTERFACE, 0x0050 },
	{ NAU8822_REG_COMPANDING_CONTROL, 0x0000 },
	{ NAU8822_REG_CLOCKING, 0x0140 },
	{ NAU8822_REG_ADDITIONAL_CONTROL, 0x0000 },
	{ NAU8822_REG_GPIO_CONTROL, 0x0000 },
	{ NAU8822_REG_JACK_DETECT_CONTROL_1, 0x0000 },
	{ NAU8822_REG_DAC_CONTROL, 0x0000 },
	{ NAU8822_REG_LEFT_DAC_DIGITAL_VOLUME, 0x00ff },
	{ NAU8822_REG_RIGHT_DAC_DIGITAL_VOLUME, 0x00ff },
	{ NAU8822_REG_JACK_DETECT_CONTROL_2, 0x0000 },
	{ NAU8822_REG_ADC_CONTROL, 0x0100 },
	{ NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME, 0x00ff },
	{ NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME, 0x00ff },
	{ NAU8822_REG_EQ1, 0x012c },
	{ NAU8822_REG_EQ2, 0x002c },
	{ NAU8822_REG_EQ3, 0x002c },
	{ NAU8822_REG_EQ4, 0x002c },
	{ NAU8822_REG_EQ5, 0x002c },
	{ NAU8822_REG_DAC_LIMITER_1, 0x0032 },
	{ NAU8822_REG_DAC_LIMITER_2, 0x0000 },
	{ NAU8822_REG_NOTCH_FILTER_1, 0x0000 },
	{ NAU8822_REG_NOTCH_FILTER_2, 0x0000 },
	{ NAU8822_REG_NOTCH_FILTER_3, 0x0000 },
	{ NAU8822_REG_NOTCH_FILTER_4, 0x0000 },
	{ NAU8822_REG_ALC_CONTROL_1, 0x0038 },
	{ NAU8822_REG_ALC_CONTROL_2, 0x000b },
	{ NAU8822_REG_ALC_CONTROL_3, 0x0032 },
	{ NAU8822_REG_NOISE_GATE, 0x0010 },
	{ NAU8822_REG_PLL_N, 0x0008 },
	{ NAU8822_REG_PLL_K1, 0x000c },
	{ NAU8822_REG_PLL_K2, 0x0093 },
	{ NAU8822_REG_PLL_K3, 0x00e9 },
	{ NAU8822_REG_3D_CONTROL, 0x0000 },
	{ NAU8822_REG_RIGHT_SPEAKER_CONTROL, 0x0000 },
	{ NAU8822_REG_INPUT_CONTROL, 0x0033 },
	{ NAU8822_REG_LEFT_INP_PGA_CONTROL, 0x0010 },
	{ NAU8822_REG_RIGHT_INP_PGA_CONTROL, 0x0010 },
	{ NAU8822_REG_LEFT_ADC_BOOST_CONTROL, 0x0100 },
	{ NAU8822_REG_RIGHT_ADC_BOOST_CONTROL, 0x0100 },
	{ NAU8822_REG_OUTPUT_CONTROL, 0x0002 },
	{ NAU8822_REG_LEFT_MIXER_CONTROL, 0x0001 },
	{ NAU8822_REG_RIGHT_MIXER_CONTROL, 0x0001 },
	{ NAU8822_REG_LHP_VOLUME, 0x0039 },
	{ NAU8822_REG_RHP_VOLUME, 0x0039 },
	{ NAU8822_REG_LSPKOUT_VOLUME, 0x0039 },
	{ NAU8822_REG_RSPKOUT_VOLUME, 0x0039 },
	{ NAU8822_REG_AUX2_MIXER, 0x0001 },
	{ NAU8822_REG_AUX1_MIXER, 0x0001 },
	{ NAU8822_REG_POWER_MANAGEMENT_4, 0x0000 },
	{ NAU8822_REG_LEFT_TIME_SLOT, 0x0000 },
	{ NAU8822_REG_MISC, 0x0020 },
	{ NAU8822_REG_RIGHT_TIME_SLOT, 0x0000 },
	{ NAU8822_REG_DEVICE_REVISION, 0x007f },
	{ NAU8822_REG_DEVICE_ID, 0x001a },
	{ NAU8822_REG_DAC_DITHER, 0x0114 },
	{ NAU8822_REG_ALC_ENHANCE_1, 0x0000 },
	{ NAU8822_REG_ALC_ENHANCE_2, 0x0000 },
	{ NAU8822_REG_192KHZ_SAMPLING, 0x0008 },
	{ NAU8822_REG_MISC_CONTROL, 0x0000 },
	{ NAU8822_REG_INPUT_TIEOFF, 0x0000 },
	{ NAU8822_REG_POWER_REDUCTION, 0x0000 },
	{ NAU8822_REG_AGC_PEAK2PEAK, 0x0000 },
	{ NAU8822_REG_AGC_PEAK_DETECT, 0x0000 },
	{ NAU8822_REG_AUTOMUTE_CONTROL, 0x0000 },
	{ NAU8822_REG_OUTPUT_TIEOFF, 0x0000 },
};

static bool nau8822_readable_reg(struct device *dev, unsigned int reg)
{
	//MYDBG("reg=%x", reg);
	switch (reg) {
	case NAU8822_REG_RESET ... NAU8822_REG_JACK_DETECT_CONTROL_1:
	case NAU8822_REG_DAC_CONTROL ... NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME:
	case NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME:
	case NAU8822_REG_EQ1 ... NAU8822_REG_EQ5:
	case NAU8822_REG_DAC_LIMITER_1 ... NAU8822_REG_DAC_LIMITER_2:
	case NAU8822_REG_NOTCH_FILTER_1 ... NAU8822_REG_NOTCH_FILTER_4:
	case NAU8822_REG_ALC_CONTROL_1 ...NAU8822_REG_PLL_K3:
	case NAU8822_REG_3D_CONTROL:
	case NAU8822_REG_RIGHT_SPEAKER_CONTROL ... NAU8822_REG_LEFT_ADC_BOOST_CONTROL:
	case NAU8822_REG_RIGHT_ADC_BOOST_CONTROL ... NAU8822_REG_AUX1_MIXER:
	case NAU8822_REG_POWER_MANAGEMENT_4 ... NAU8822_REG_DEVICE_ID:
	case NAU8822_REG_DAC_DITHER:
	case NAU8822_REG_ALC_ENHANCE_1 ... NAU8822_REG_MISC_CONTROL:
	case NAU8822_REG_INPUT_TIEOFF ... NAU8822_REG_OUTPUT_TIEOFF:
		return true;
	default:
		return false;
	}

	return true;
}

static bool nau8822_writeable_reg(struct device *dev, unsigned int reg)
{
	//MYDBG("reg=%x", reg);
	switch (reg) {
	case NAU8822_REG_RESET ... NAU8822_REG_JACK_DETECT_CONTROL_1:
	case NAU8822_REG_DAC_CONTROL ... NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME:
	case NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME:
	case NAU8822_REG_EQ1 ... NAU8822_REG_EQ5:
	case NAU8822_REG_DAC_LIMITER_1 ... NAU8822_REG_DAC_LIMITER_2:
	case NAU8822_REG_NOTCH_FILTER_1 ... NAU8822_REG_NOTCH_FILTER_4:
	case NAU8822_REG_ALC_CONTROL_1 ...NAU8822_REG_PLL_K3:
	case NAU8822_REG_3D_CONTROL:
	case NAU8822_REG_RIGHT_SPEAKER_CONTROL ... NAU8822_REG_LEFT_ADC_BOOST_CONTROL:
	case NAU8822_REG_RIGHT_ADC_BOOST_CONTROL ... NAU8822_REG_AUX1_MIXER:
	case NAU8822_REG_POWER_MANAGEMENT_4 ... NAU8822_REG_DEVICE_ID:
	case NAU8822_REG_DAC_DITHER:
	case NAU8822_REG_ALC_ENHANCE_1 ... NAU8822_REG_MISC_CONTROL:
	case NAU8822_REG_INPUT_TIEOFF ... NAU8822_REG_OUTPUT_TIEOFF:
		return true;
	default:
		return false;
	}

	return true;
}

static bool nau8822_volatile(struct device *dev, unsigned int reg)
{
	//MYDBG("reg=%x", reg);
	//return true;
	switch (reg) {
	case NAU8822_REG_RESET:
	case NAU8822_REG_DEVICE_REVISION:
	case NAU8822_REG_DEVICE_ID:
	case NAU8822_REG_AGC_PEAK2PEAK:
	case NAU8822_REG_AGC_PEAK_DETECT:
	case NAU8822_REG_AUTOMUTE_CONTROL:
		return true;
	default:
		return false;
	}
}

static const char *nau8822_companding[] = {"Off", "NC", "u-law", "A-law"};
static const char *nau8822_eqmode[] = {"Capture", "Playback"};
static const char *nau8822_bw[] = {"Narrow", "Wide"};
static const char *nau8822_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz"};
static const char *nau8822_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz"};
static const char *nau8822_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz"};
static const char *nau8822_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz"};
static const char *nau8822_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz"};
static const char *nau8822_alc3[] = {"ALC", "Limiter"};
static const char *nau8822_alc1[] = {"Off", "Right", "Left", "Both"};

static const SOC_ENUM_SINGLE_DECL(adc_compand, NAU8822_REG_COMPANDING_CONTROL, 1,
				  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(dac_compand, NAU8822_REG_COMPANDING_CONTROL, 3,
				  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(eqmode, NAU8822_REG_EQ1, 8, nau8822_eqmode);
static const SOC_ENUM_SINGLE_DECL(eq1, NAU8822_REG_EQ1, 5, nau8822_eq1);
static const SOC_ENUM_SINGLE_DECL(eq2bw, NAU8822_REG_EQ2, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq2, NAU8822_REG_EQ2, 5, nau8822_eq2);
static const SOC_ENUM_SINGLE_DECL(eq3bw, NAU8822_REG_EQ3, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq3, NAU8822_REG_EQ3, 5, nau8822_eq3);
static const SOC_ENUM_SINGLE_DECL(eq4bw, NAU8822_REG_EQ4, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq4, NAU8822_REG_EQ4, 5, nau8822_eq4);
static const SOC_ENUM_SINGLE_DECL(eq5, NAU8822_REG_EQ5, 5, nau8822_eq5);
static const SOC_ENUM_SINGLE_DECL(alc3, NAU8822_REG_ALC_CONTROL_3, 8, nau8822_alc3);
static const SOC_ENUM_SINGLE_DECL(alc1, NAU8822_REG_ALC_CONTROL_1, 7, nau8822_alc1);

static const DECLARE_TLV_DB_SCALE(digital_tlv, -12750, 50, 1);
static const DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);
static const DECLARE_TLV_DB_SCALE(inpga_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(spk_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(boost_tlv, -1500, 300, 1);
static const DECLARE_TLV_DB_SCALE(limiter_tlv, 0, 100, 0);

static const struct snd_kcontrol_new nau8822_snd_controls[] = {

	SOC_SINGLE("Digital Loopback Switch",
		NAU8822_REG_COMPANDING_CONTROL, 0, 1, 0),

	SOC_ENUM("ADC Companding", adc_compand),
	SOC_ENUM("DAC Companding", dac_compand),

	SOC_DOUBLE("DAC Inversion Switch", NAU8822_REG_DAC_CONTROL, 0, 1, 1, 0),

	SOC_DOUBLE_R_TLV("PCM Volume",
		NAU8822_REG_LEFT_DAC_DIGITAL_VOLUME, NAU8822_REG_RIGHT_DAC_DIGITAL_VOLUME,
		0, 255, 0, digital_tlv),

	SOC_SINGLE("High Pass Filter Switch", NAU8822_REG_ADC_CONTROL, 8, 1, 0),
	SOC_SINGLE("High Pass Cut Off", NAU8822_REG_ADC_CONTROL, 4, 7, 0),
	SOC_DOUBLE("ADC Inversion Switch", NAU8822_REG_ADC_CONTROL, 0, 1, 1, 0),

	SOC_DOUBLE_R_TLV("ADC Volume",
		NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME, NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME,
		0, 255, 0, digital_tlv),

	SOC_ENUM("Equaliser Function", eqmode),
	SOC_ENUM("EQ1 Cut Off", eq1),
	SOC_SINGLE_TLV("EQ1 Volume", NAU8822_REG_EQ1,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ2 Bandwidth", eq2bw),
	SOC_ENUM("EQ2 Cut Off", eq2),
	SOC_SINGLE_TLV("EQ2 Volume", NAU8822_REG_EQ2,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ3 Bandwidth", eq3bw),
	SOC_ENUM("EQ3 Cut Off", eq3),
	SOC_SINGLE_TLV("EQ3 Volume", NAU8822_REG_EQ3,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ4 Bandwidth", eq4bw),
	SOC_ENUM("EQ4 Cut Off", eq4),
	SOC_SINGLE_TLV("EQ4 Volume", NAU8822_REG_EQ4,  0, 24, 1, eq_tlv),

	SOC_ENUM("EQ5 Cut Off", eq5),
	SOC_SINGLE_TLV("EQ5 Volume", NAU8822_REG_EQ5, 0, 24, 1, eq_tlv),

	SOC_SINGLE("DAC Playback Limiter Switch",
		NAU8822_REG_DAC_LIMITER_1, 8, 1, 0),
	SOC_SINGLE("DAC Playback Limiter Decay",
		NAU8822_REG_DAC_LIMITER_1, 4, 15, 0),
	SOC_SINGLE("DAC Playback Limiter Attack",
		NAU8822_REG_DAC_LIMITER_1, 0, 15, 0),

	SOC_SINGLE("DAC Playback Limiter Threshold",
		NAU8822_REG_DAC_LIMITER_2, 4, 7, 0),
	SOC_SINGLE_TLV("DAC Playback Limiter Volume",
		NAU8822_REG_DAC_LIMITER_2, 0, 12, 0, limiter_tlv),

	SOC_ENUM("ALC Enable Switch", alc1),
	SOC_SINGLE("ALC Capture Min Gain", NAU8822_REG_ALC_CONTROL_1, 0, 7, 0),
	SOC_SINGLE("ALC Capture Max Gain", NAU8822_REG_ALC_CONTROL_1, 3, 7, 0),

	SOC_SINGLE("ALC Capture Hold", NAU8822_REG_ALC_CONTROL_2, 4, 10, 0),
	SOC_SINGLE("ALC Capture Target", NAU8822_REG_ALC_CONTROL_2, 0, 15, 0),

	SOC_ENUM("ALC Capture Mode", alc3),
	SOC_SINGLE("ALC Capture Decay", NAU8822_REG_ALC_CONTROL_3, 4, 10, 0),
	SOC_SINGLE("ALC Capture Attack", NAU8822_REG_ALC_CONTROL_3, 0, 10, 0),

	SOC_SINGLE("ALC Capture Noise Gate Switch", NAU8822_REG_NOISE_GATE, 3, 1, 0),
	SOC_SINGLE("ALC Capture Noise Gate Threshold",
		NAU8822_REG_NOISE_GATE, 0, 7, 0),

	SOC_DOUBLE_R("Capture PGA ZC Switch",
		NAU8822_REG_LEFT_INP_PGA_CONTROL, NAU8822_REG_RIGHT_INP_PGA_CONTROL,
		7, 1, 0),

	SOC_DOUBLE_R("Headphone Playback ZC Switch",
		NAU8822_REG_LHP_VOLUME, NAU8822_REG_RHP_VOLUME, 7, 1, 0),

	SOC_DOUBLE_R_TLV("Headphone Playback Volume",
		NAU8822_REG_LHP_VOLUME, NAU8822_REG_RHP_VOLUME,	0, 63, 0, spk_tlv),

	SOC_DOUBLE_R("Speaker Playback ZC Switch",
		NAU8822_REG_LSPKOUT_VOLUME, NAU8822_REG_RSPKOUT_VOLUME, 7, 1, 0),

	SOC_DOUBLE_R_TLV("Speaker Playback Volume",
		NAU8822_REG_LSPKOUT_VOLUME, NAU8822_REG_RSPKOUT_VOLUME,
		0, 63, 0, spk_tlv),

	SOC_DOUBLE_R("Line Playback Switch",
		NAU8822_REG_AUX2_MIXER, NAU8822_REG_AUX1_MIXER, 6, 1, 1),

	SOC_DOUBLE_R("PGA Boost (+20dB)",
		NAU8822_REG_LEFT_ADC_BOOST_CONTROL, NAU8822_REG_RIGHT_ADC_BOOST_CONTROL,
		8, 1, 0),
	SOC_DOUBLE_R_TLV("L2/R2 Boost Volume",
		NAU8822_REG_LEFT_ADC_BOOST_CONTROL, NAU8822_REG_RIGHT_ADC_BOOST_CONTROL,
		4, 7, 0, boost_tlv),
	SOC_DOUBLE_R_TLV("Aux Boost Volume",
		NAU8822_REG_LEFT_ADC_BOOST_CONTROL, NAU8822_REG_RIGHT_ADC_BOOST_CONTROL,
		0, 7, 0, boost_tlv),

	SOC_DOUBLE_R_TLV("Input PGA Volume",
		NAU8822_REG_LEFT_INP_PGA_CONTROL, NAU8822_REG_RIGHT_INP_PGA_CONTROL,
		0, 63, 0, inpga_tlv),

	SOC_DOUBLE_R("Headphone Switch",
		NAU8822_REG_LHP_VOLUME, NAU8822_REG_RHP_VOLUME, 6, 1, 1),

	SOC_DOUBLE_R("Speaker Switch",
		NAU8822_REG_LSPKOUT_VOLUME, NAU8822_REG_RSPKOUT_VOLUME, 6, 1, 1),

	SOC_SINGLE("DAC 128x Oversampling Switch", NAU8822_REG_DAC_CONTROL,
		   5, 1, 0),
	SOC_SINGLE("ADC 128x Oversampling Switch", NAU8822_REG_ADC_CONTROL,
		   5, 1, 0),
};

/* LMAIN and RMAIN Mixer */
static const struct snd_kcontrol_new nau8822_left_out_mixer[] = {
	SOC_DAPM_SINGLE("LINMIX Switch", NAU8822_REG_LEFT_MIXER_CONTROL, 1, 1, 0),
	SOC_DAPM_SINGLE("LAUX Switch", NAU8822_REG_LEFT_MIXER_CONTROL, 5, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_REG_LEFT_MIXER_CONTROL, 0, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", NAU8822_REG_OUTPUT_CONTROL, 5, 1, 0),
};

static const struct snd_kcontrol_new nau8822_right_out_mixer[] = {
	SOC_DAPM_SINGLE("RINMIX Switch", NAU8822_REG_RIGHT_MIXER_CONTROL, 1, 1, 0),
	SOC_DAPM_SINGLE("RAUX Switch", NAU8822_REG_RIGHT_MIXER_CONTROL, 5, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", NAU8822_REG_RIGHT_MIXER_CONTROL, 0, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_REG_OUTPUT_CONTROL, 6, 1, 0),
};

/* AUX1 and AUX2 Mixer */
static const struct snd_kcontrol_new nau8822_auxout1_mixer[] = {
	SOC_DAPM_SINGLE("RDAC Switch", NAU8822_REG_AUX1_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("RMIX Switch", NAU8822_REG_AUX1_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("RINMIX Switch", NAU8822_REG_AUX1_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_REG_AUX1_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("LMIX Switch", NAU8822_REG_AUX1_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new nau8822_auxout2_mixer[] = {
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_REG_AUX2_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("LMIX Switch", NAU8822_REG_AUX2_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("LINMIX Switch", NAU8822_REG_AUX2_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("AUX1MIX Output Switch", NAU8822_REG_AUX1_MIXER, 3, 1, 0),
};

/* Input PGA */
static const struct snd_kcontrol_new nau8822_left_input_mixer[] = {
	SOC_DAPM_SINGLE("L2 Switch", NAU8822_REG_INPUT_CONTROL, 2, 1, 0),
	SOC_DAPM_SINGLE("MicN Switch", NAU8822_REG_INPUT_CONTROL, 1, 1, 0),
	SOC_DAPM_SINGLE("MicP Switch", NAU8822_REG_INPUT_CONTROL, 0, 1, 0),
};
static const struct snd_kcontrol_new nau8822_right_input_mixer[] = {
	SOC_DAPM_SINGLE("R2 Switch", NAU8822_REG_INPUT_CONTROL, 6, 1, 0),
	SOC_DAPM_SINGLE("MicN Switch", NAU8822_REG_INPUT_CONTROL, 5, 1, 0),
	SOC_DAPM_SINGLE("MicP Switch", NAU8822_REG_INPUT_CONTROL, 4, 1, 0),
};

static int check_mclk_select_pll(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;

	ENTER();
	regmap_read(nau8822->regmap, NAU8822_REG_CLOCKING, &value);

	return (value & NAU8822_CLKM_MASK);
}

static const struct snd_soc_dapm_widget nau8822_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback",
			 NAU8822_REG_POWER_MANAGEMENT_3, 0, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback",
			 NAU8822_REG_POWER_MANAGEMENT_3, 1, 0),
	SND_SOC_DAPM_ADC("Left ADC", "Left HiFi Capture",
			 NAU8822_REG_POWER_MANAGEMENT_2, 0, 0),
	SND_SOC_DAPM_ADC("Right ADC", "Right HiFi Capture",
			 NAU8822_REG_POWER_MANAGEMENT_2, 1, 0),

	SOC_MIXER_ARRAY("Left Output Mixer", NAU8822_REG_POWER_MANAGEMENT_3,
			2, 0, nau8822_left_out_mixer),
	SOC_MIXER_ARRAY("Right Output Mixer", NAU8822_REG_POWER_MANAGEMENT_3,
			3, 0, nau8822_right_out_mixer),
	SOC_MIXER_ARRAY("AUX1 Output Mixer", NAU8822_REG_POWER_MANAGEMENT_1,
			7, 0, nau8822_auxout1_mixer),
	SOC_MIXER_ARRAY("AUX2 Output Mixer", NAU8822_REG_POWER_MANAGEMENT_1,
			6, 0, nau8822_auxout2_mixer),

	SOC_MIXER_ARRAY("Left Input Mixer", NAU8822_REG_POWER_MANAGEMENT_2,
			2, 0, nau8822_left_input_mixer),
	SOC_MIXER_ARRAY("Right Input Mixer", NAU8822_REG_POWER_MANAGEMENT_2,
			3, 0, nau8822_right_input_mixer),

	SND_SOC_DAPM_PGA("Left Boost Mixer", NAU8822_REG_POWER_MANAGEMENT_2,
			 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Boost Mixer", NAU8822_REG_POWER_MANAGEMENT_2,
			 5, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left Capture PGA", NAU8822_REG_LEFT_INP_PGA_CONTROL,
			 6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Capture PGA", NAU8822_REG_RIGHT_INP_PGA_CONTROL,
			 6, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Left Headphone Out", NAU8822_REG_POWER_MANAGEMENT_2,
			 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Headphone Out", NAU8822_REG_POWER_MANAGEMENT_2,
			 8, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left Speaker Out", NAU8822_REG_POWER_MANAGEMENT_3,
			 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Speaker Out", NAU8822_REG_POWER_MANAGEMENT_3,
			 5, 0, NULL, 0),

	SND_SOC_DAPM_PGA("AUX1 Out", NAU8822_REG_POWER_MANAGEMENT_3,
			   8, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX2 Out", NAU8822_REG_POWER_MANAGEMENT_3,
			   7, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Mic Bias", NAU8822_REG_POWER_MANAGEMENT_1,
			4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL", NAU8822_REG_POWER_MANAGEMENT_1,
			5, 0, NULL, 0),

	SND_SOC_DAPM_INPUT("LMICN"),
	SND_SOC_DAPM_INPUT("LMICP"),
	SND_SOC_DAPM_INPUT("RMICN"),
	SND_SOC_DAPM_INPUT("RMICP"),
	SND_SOC_DAPM_INPUT("LAUX"),
	SND_SOC_DAPM_INPUT("RAUX"),
	SND_SOC_DAPM_INPUT("L2"),
	SND_SOC_DAPM_INPUT("R2"),
	SND_SOC_DAPM_OUTPUT("LHP"),
	SND_SOC_DAPM_OUTPUT("RHP"),
	SND_SOC_DAPM_OUTPUT("LSPK"),
	SND_SOC_DAPM_OUTPUT("RSPK"),
	SND_SOC_DAPM_OUTPUT("AUXOUT1"),
	SND_SOC_DAPM_OUTPUT("AUXOUT2"),
};

static const struct snd_soc_dapm_route nau8822_dapm_routes[] = {
	{"Right DAC", NULL, "PLL", check_mclk_select_pll},
	{"Left DAC", NULL, "PLL", check_mclk_select_pll},

	/* LMAIN and RMAIN mixer */
	{"Right Output Mixer", "LDAC Switch", "Left DAC"},
	{"Right Output Mixer", "RDAC Switch", "Right DAC"},
	{"Right Output Mixer", "RAUX Switch", "RAUX"},
	{"Right Output Mixer", "RINMIX Switch", "Right Boost Mixer"},

	{"Left Output Mixer", "LDAC Switch", "Left DAC"},
	{"Left Output Mixer", "RDAC Switch", "Right DAC"},
	{"Left Output Mixer", "LAUX Switch", "LAUX"},
	{"Left Output Mixer", "LINMIX Switch", "Left Boost Mixer"},

	/* AUX1 and AUX2 mixer */
	{"AUX1 Output Mixer", "RDAC Switch", "Right DAC"},
	{"AUX1 Output Mixer", "RMIX Switch", "Right Output Mixer"},
	{"AUX1 Output Mixer", "RINMIX Switch", "Right Boost Mixer"},
	{"AUX1 Output Mixer", "LDAC Switch", "Left DAC"},
	{"AUX1 Output Mixer", "LMIX Switch", "Left Output Mixer"},

	{"AUX2 Output Mixer", "LDAC Switch", "Left DAC"},
	{"AUX2 Output Mixer", "LMIX Switch", "Left Output Mixer"},
	{"AUX2 Output Mixer", "LINMIX Switch", "Left Boost Mixer"},
	{"AUX2 Output Mixer", "AUX1MIX Output Switch", "AUX1 Output Mixer"},

	/* Outputs */
	{"Right Headphone Out", NULL, "Right Output Mixer"},
	{"RHP", NULL, "Right Headphone Out"},

	{"Left Headphone Out", NULL, "Left Output Mixer"},
	{"LHP", NULL, "Left Headphone Out"},

	{"Right Speaker Out", NULL, "Right Output Mixer"},
	{"RSPK", NULL, "Right Speaker Out"},

	{"Left Speaker Out", NULL, "Left Output Mixer"},
	{"LSPK", NULL, "Left Speaker Out"},

	{"AUX1 Out", NULL, "AUX1 Output Mixer"},
	{"AUX2 Out", NULL, "AUX2 Output Mixer"},
	{"AUXOUT1", NULL, "AUX1 Out"},
	{"AUXOUT2", NULL, "AUX2 Out"},

	/* Boost Mixer */
	{"Right ADC", NULL, "PLL", check_mclk_select_pll},
	{"Left ADC", NULL, "PLL", check_mclk_select_pll},

	{"Right ADC", NULL, "Right Boost Mixer"},

	{"Right Boost Mixer", NULL, "RAUX"},
	{"Right Boost Mixer", NULL, "Right Capture PGA"},
	{"Right Boost Mixer", NULL, "R2"},

	{"Left ADC", NULL, "Left Boost Mixer"},

	{"Left Boost Mixer", NULL, "LAUX"},
	{"Left Boost Mixer", NULL, "Left Capture PGA"},
	{"Left Boost Mixer", NULL, "L2"},

	/* Input PGA */
	{"Right Capture PGA", NULL, "Right Input Mixer"},
	{"Left Capture PGA", NULL, "Left Input Mixer"},

	/*enable microphone power*/
	{"Right Capture PGA", NULL, "Mic Bias"},
	{"Left Capture PGA", NULL, "Mic Bias"},

	{"Right Input Mixer", "R2 Switch", "R2"},
	{"Right Input Mixer", "MicN Switch", "RMICN"},
	{"Right Input Mixer", "MicP Switch", "RMICP"},

	{"Left Input Mixer", "L2 Switch", "L2"},
	{"Left Input Mixer", "MicN Switch", "LMICN"},
	{"Left Input Mixer", "MicP Switch", "LMICP"},
};

static int nau8822_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				 unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	ENTER();

		nau8822->div_id = clk_id;
		nau8822->sysclk = freq;
		dev_dbg(nau8822->dev, "master sysclk %dHz, source %s\n",freq,
			clk_id == NAU8822_CLK_PLL ? "PLL" : "MCLK");

	return 0;
}

static int nau8822_calc_pll(unsigned int pll_in, unsigned int fs,
				struct nau8822_pll *pll_param)
{
	u64 f2, f2_max, pll_ratio;
	int i, scal_sel;
	ENTER();

	if (pll_in > NAU_PLL_REF_MAX || pll_in < NAU_PLL_REF_MIN)
		return -EINVAL;
	f2_max = 0;
	scal_sel = ARRAY_SIZE(nau8822_mclk_scaler);

	for (i = 0; i < scal_sel; i++) {
		f2 = 256 * fs * 4 * nau8822_mclk_scaler[i] / 10;

	if (f2 > NAU_PLL_FREQ_MIN && f2 < NAU_PLL_FREQ_MAX && f2_max < f2) {
			f2_max = f2;
			scal_sel = i;
		}
	}

		if (ARRAY_SIZE(nau8822_mclk_scaler) == scal_sel){
			return -EINVAL;
		}

	/* Calculate the PLL 4-bit integer input and the PLL 24-bit fractional
	 * input; round up the 24+4bit.
	 */
	pll_ratio = div_u64(f2 << 28, pll_in);
	pll_param->pre_factor = 0;
	if (((pll_ratio >> 28) & 0xF) < NAU_PLL_OPTOP_MIN)
	{
		pll_ratio <<= 1;
		pll_param->pre_factor = 1;
	}
	pll_param->pll_int = (pll_ratio >> 28) & 0xF;
	pll_param->pll_frac = ((pll_ratio & 0xFFFFFFF) >> 4);

	return 0;
}

static int nau8822_config_clkdiv(struct nau8822_priv *nau8822, int div,
				int rate)
{
	struct regmap *regmap = nau8822->regmap;
	struct nau8822_pll *pll = &nau8822->pll;
	int i, sclk, imclk;
	ENTER();

	switch (nau8822->div_id) {
	case NAU8822_CLK_MCLK:
		/* Configure the master clock prescaler div to make system
		 * clock to approximate the internal master clock (IMCLK);
		 * and large or equal to IMCLK.
		 */
		div = 0;
//		imclk = rate * 256;
		imclk = rate * 256 * 2;
		for (i = 1; i < ARRAY_SIZE(nau8822_mclk_scaler); i++) {
			sclk = (nau8822->sysclk * 10) /	nau8822_mclk_scaler[i];
			if (sclk < imclk)
				break;
			div = i;
		}
		dev_dbg(nau8822->dev,"master clock prescaler %x for fs %d\n", div,
			rate);

		/* master clock from MCLK and disable PLL */
		regmap_update_bits(regmap, NAU8822_REG_CLOCKING, NAU8822_MCLKSEL_MASK,
			(div << NAU8822_MCLKSEL_SFT));
		regmap_update_bits(regmap, NAU8822_REG_CLOCKING, NAU8822_CLKM_MASK,
			NAU8822_CLKM_MCLK);
		break;

	case NAU8822_CLK_PLL:
		/* master clock from PLL and enable PLL */
		if (pll->mclk_scaler != div) {
			dev_err(nau8822->dev, "master clock prescaler not meet PLL parameters\n");
			return -EINVAL;
		}
		regmap_update_bits(regmap, NAU8822_REG_CLOCKING, NAU8822_MCLKSEL_MASK,
			(div << NAU8822_MCLKSEL_SFT));
		regmap_update_bits(regmap, NAU8822_REG_CLOCKING, NAU8822_CLKM_MASK,
			NAU8822_CLKM_PLL);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int nau8822_set_pll(struct snd_soc_dai *codec_dai, int pll_id,
				int source,unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	struct regmap *map = nau8822->regmap;
	struct nau8822_pll *pll_param = &nau8822->pll;
	int ret, fs;
	ENTER();

	fs = freq_out / 256;

	ret = nau8822_calc_pll(freq_in, fs, pll_param);
	if (ret < 0) {
		dev_err(codec->dev, "Unsupported input clock %d\n", freq_in);
		return ret;
	}

	dev_info(nau8822->dev, "pll_int=%x pll_frac=%x mclk_scaler=%x\
		pre_factor=%x\n",pll_param->pll_int, pll_param->pll_frac,
		pll_param->mclk_scaler, pll_param->pre_factor);

	regmap_update_bits(map, NAU8822_REG_PLL_N,NAU8822_PLLMCLK_DIV2 |
		NAU8822_PLLN_MASK,(pll_param->pre_factor ? NAU8822_PLLMCLK_DIV2 : 0) |
		pll_param->pll_int);
	regmap_write(map, NAU8822_REG_PLL_K1,(pll_param->pll_frac >>
		NAU8822_PLLK1_SFT) & NAU8822_PLLK1_MASK);
	regmap_write(map, NAU8822_REG_PLL_K2,(pll_param->pll_frac >>
		NAU8822_PLLK2_SFT) & NAU8822_PLLK2_MASK);
	regmap_write(map, NAU8822_REG_PLL_K3, pll_param->pll_frac &
		NAU8822_PLLK3_MASK);
	regmap_update_bits(map, NAU8822_REG_CLOCKING, NAU8822_MCLKSEL_MASK,
		pll_param->mclk_scaler << NAU8822_MCLKSEL_SFT);
	regmap_update_bits(map, NAU8822_REG_CLOCKING, NAU8822_CLKM_MASK,
		NAU8822_CLKM_PLL);

	return 0;
}

static int nau8822_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	u16 ctrl1_val = 0, ctrl2_val = 0;

	dev_dbg(codec->dev, "%s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		ctrl2_val |= 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		ctrl2_val &= ~1;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= 0x10;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= 0x8;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= 0x18;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ctrl1_val |= 0x180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= 0x100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		ctrl1_val |= 0x80;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8822->regmap, NAU8822_REG_AUDIO_INTERFACE,
		NAU8822_AIFMT_MASK | NAU8822_LRP_MASK | NAU8822_BCLKP_MASK, ctrl1_val);
	regmap_update_bits(nau8822->regmap, NAU8822_REG_CLOCKING,
		NAU8822_CLKIOEN_MASK, ctrl2_val);

	return 0;
}

static int nau8822_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	int val_len = 0, val_rate = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	ENTER();

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val_len |= NAU8822_WLEN_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val_len |= NAU8822_WLEN_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val_len |= NAU8822_WLEN_32;
		break;
	}

	switch (params_rate(params)) {
	case 8000:
		val_rate |= NAU8822_SMPLR_8K;
		break;
	case 11025:
		val_rate |= NAU8822_SMPLR_12K;
		break;
	case 16000:
		val_rate |= NAU8822_SMPLR_16K;
		break;
	case 22050:
		val_rate |= NAU8822_SMPLR_24K;
		break;
	case 32000:
		val_rate |= NAU8822_SMPLR_32K;
		break;
	case 44100:
	case 48000:
		break;
	}

	regmap_update_bits(nau8822->regmap, NAU8822_REG_AUDIO_INTERFACE,
		NAU8822_WLEN_MASK, val_len);
	regmap_update_bits(nau8822->regmap, NAU8822_REG_ADDITIONAL_CONTROL,
		NAU8822_SMPLR_MASK, val_rate);

	/* If the master clock is from MCLK, provide the runtime FS for driver
	 * to get the master clock prescaler configuration.
	 */
	if (nau8822->div_id == NAU8822_CLK_MCLK)
		nau8822_config_clkdiv(nau8822, 0, params_rate(params));

	return 0;
}

static int nau8822_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s: %d\n", __func__, mute);

	if (mute)
		regmap_update_bits(nau8822->regmap, NAU8822_REG_DAC_CONTROL, 0x40, 0x40);
	else
		regmap_update_bits(nau8822->regmap, NAU8822_REG_DAC_CONTROL, 0x40, 0);

	return 0;
}

static int nau8822_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	struct regmap *map = nau8822->regmap;
	MYDBG("bias_leve=%x", level);
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		regmap_update_bits(map, NAU8822_REG_POWER_MANAGEMENT_1,
			NAU8822_REFIMP_MASK, NAU8822_REFIMP_80K);
		break;
	case SND_SOC_BIAS_STANDBY:
		regmap_update_bits(map, NAU8822_REG_POWER_MANAGEMENT_1,
			NAU8822_IOBUF_EN | NAU8822_ABIAS_EN, NAU8822_IOBUF_EN | NAU8822_ABIAS_EN);

		if (snd_soc_codec_get_bias_level(codec) == SND_SOC_BIAS_OFF) {
			regmap_update_bits(map, NAU8822_REG_POWER_MANAGEMENT_1,
				NAU8822_REFIMP_MASK, NAU8822_REFIMP_3K);
			mdelay(100);
		}
		regmap_update_bits(map, NAU8822_REG_POWER_MANAGEMENT_1,
			NAU8822_REFIMP_MASK, NAU8822_REFIMP_300K);
		break;
	case SND_SOC_BIAS_OFF:
		regmap_write(map, NAU8822_REG_POWER_MANAGEMENT_1, 0);
		regmap_write(map, NAU8822_REG_POWER_MANAGEMENT_2, 0);
		regmap_write(map, NAU8822_REG_POWER_MANAGEMENT_3, 0);
		break;
	}

	return 0;
}

#define NAU8822_RATES (SNDRV_PCM_RATE_8000_48000)

#define NAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops nau8822_dai_ops = {
	.hw_params	= nau8822_hw_params,
	.digital_mute	= nau8822_mute,
	.set_fmt	= nau8822_set_dai_fmt,
	.set_sysclk	= nau8822_set_dai_sysclk,
	.set_pll	= nau8822_set_pll,
};

static struct snd_soc_dai_driver nau8822_dai = {
	.name = "nau8822-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = NAU8822_RATES,
		.formats = NAU8822_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = NAU8822_RATES,
		.formats = NAU8822_FORMATS,
	},
	.ops = &nau8822_dai_ops,
	.symmetric_rates = 1,
};

static int nau8822_suspend(struct snd_soc_codec *codec)
{
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
	regmap_write(nau8822->regmap, NAU8822_REG_POWER_MANAGEMENT_1, 0);

	regcache_mark_dirty(nau8822->regmap);
	gpiod_direction_output(nau8822->pwdn_gpiod, 0);

	return 0;
}

static int nau8822_resume(struct snd_soc_codec *codec)
{
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	gpiod_direction_output(nau8822->pwdn_gpiod, 1);

	regcache_sync(nau8822->regmap);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 * These registers contain an "update" bit - bit 8. This means, for example,
 * that one can write new DAC digital volume for both channels, but only when
 * the update bit is set, will also the volume be updated - simultaneously for
 * both channels.
 */
static const int update_reg[] = {
	NAU8822_REG_LEFT_DAC_DIGITAL_VOLUME,
	NAU8822_REG_RIGHT_DAC_DIGITAL_VOLUME,
	NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME,
	NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME,
	NAU8822_REG_LEFT_INP_PGA_CONTROL,
	NAU8822_REG_RIGHT_INP_PGA_CONTROL,
	NAU8822_REG_LHP_VOLUME,
	NAU8822_REG_RHP_VOLUME,
	NAU8822_REG_LSPKOUT_VOLUME,
	NAU8822_REG_RSPKOUT_VOLUME,
};

static int nau8822_probe(struct snd_soc_codec *codec)
{
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	int i;

	/*
	 * Set the update bit in all registers, that have one. This way all
	 * writes to those registers will also cause the update bit to be
	 * written.
	 */
	for (i = 0; i < ARRAY_SIZE(update_reg); i++)
		regmap_update_bits(nau8822->regmap, update_reg[i], 0x100, 0x100);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int nau8822_remove(struct snd_soc_codec *codec)
{
	nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static struct regmap *nau8822_get_regmap(struct device *dev)
{
	struct nau8822_priv *nau8822 = dev_get_drvdata(dev);
	ENTER();
	return nau8822->regmap;
}

static struct snd_soc_codec_driver soc_codec_dev_nau8822 = {
	.probe =	nau8822_probe,
	.get_regmap	   = nau8822_get_regmap,
	.set_bias_level = nau8822_set_bias_level,

	.remove =	nau8822_remove,
	.suspend =	nau8822_suspend,
	.resume =	nau8822_resume,

	.controls = nau8822_snd_controls,
	.num_controls = ARRAY_SIZE(nau8822_snd_controls),
	.dapm_widgets = nau8822_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(nau8822_dapm_widgets),
	.dapm_routes = nau8822_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(nau8822_dapm_routes),
};

static const struct regmap_config nau8822_regmap_config = {
	.reg_bits = 7,
	.val_bits = 9,

	//.reg_read = nau8822_reg_read2,
	.max_register = NAU8822_REG_MAX_REGISTER,
	.volatile_reg = nau8822_volatile,

	.readable_reg = nau8822_readable_reg,
	.writeable_reg = nau8822_writeable_reg,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8822_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8822_reg_defaults),
};

static int nau8822_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct nau8822_priv *nau8822 = dev_get_platdata(dev);
	int ret;
	int value;
	unsigned int flags;
	ENTER();
	if(!nau8822){
		nau8822 = devm_kzalloc(dev, sizeof(struct nau8822_priv),GFP_KERNEL);
		if(nau8822 == NULL)
			return -ENOMEM;
	}
	nau8822->pwdn_gpiod = devm_gpiod_get_optional(&i2c->dev, "ingenic,pwdn",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(nau8822->pwdn_gpiod))
		return PTR_ERR(nau8822->pwdn_gpiod);

	nau8822->i2c = i2c;
	i2c_set_clientdata(i2c, nau8822);

	ret = nau8822_reg_read(nau8822, NAU8822_REG_DEVICE_ID, &value);
	dev_info(&i2c->dev, "NAU8822_REG_DEVICE_ID=%x , ret=%x\n", value, ret);


	//nau8822->regmap = devm_regmap_init(dev, NULL, i2c, &nau8822_regmap_config);
	nau8822->regmap = devm_regmap_init_i2c(i2c, &nau8822_regmap_config);
	if (IS_ERR(nau8822->regmap)) {
		ret = PTR_ERR(nau8822->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	nau8822->dev = dev;
#if 0
	regcache_cache_bypass(nau8822->regmap, true);
	MYDBG("nau8822=%p\n", nau8822);
	MYDBG("nau8822->regmap=%p\n", nau8822->regmap);
	ret=0;
	ret = regmap_read(nau8822->regmap, NAU8822_REG_DEVICE_ID, &value);
	MYDBG("NAU8822_REG_DEVICE_ID=%x , ret=%x\n", value, ret);
	ret=0;
	ret = regmap_read(nau8822->regmap, NAU8822_REG_DEVICE_REVISION, &value);
	MYDBG("NAU8822_REG_DEVICE_REVISION=%x, ret=%x\n", value, ret);
	regcache_cache_bypass(nau8822->regmap, false);


	MYDBG("nau8822->regmap=%p\n", nau8822->regmap);
	ret=0;
	ret = regmap_read(nau8822->regmap, NAU8822_REG_DEVICE_ID, &value);
	MYDBG("NAU8822_REG_DEVICE_ID=%x , ret=%x\n", value, ret);
	ret=0;
	ret = regmap_read(nau8822->regmap, NAU8822_REG_DEVICE_REVISION, &value);
	MYDBG("NAU8822_REG_DEVICE_REVISION=%x, ret=%x\n", value, ret);
	regcache_cache_bypass(nau8822->regmap, false);
#endif

	/* Reset the codec */
	ret = regmap_write(nau8822->regmap, NAU8822_REG_RESET, 0x00);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to issue reset: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(dev,&soc_codec_dev_nau8822, &nau8822_dai, 1);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register CODEC: %d\n", ret);
		return ret;
	}

	return 0;
}

static int nau8822_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id nau8822_i2c_id[] = {
	{ "nau8822", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8822_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id nau8822_of_match[] = {
	{ .compatible = "nuvoton,nau8822", },
	{ },
};
MODULE_DEVICE_TABLE(of, nau8822_of_match);
#endif

static struct i2c_driver nau8822_i2c_driver = {
	.driver = {
		.name = "nau8822",
		.of_match_table = of_match_ptr(nau8822_of_match),
	},
	.probe =    nau8822_i2c_probe,
	.remove =   nau8822_i2c_remove,
	.id_table = nau8822_i2c_id,
};
module_i2c_driver(nau8822_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU8822 codec driver");
MODULE_AUTHOR("Nuvoton CTLIN <ctlin0@nuvoton.com>");
MODULE_LICENSE("GPL v2");
