/*
 * ALSA SoC TWL6030 codec driver
 *
 * Author:      Misael Lopez Cruz <x0052729@ti.com>
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "twl6030.h"

#define TWL6030_RATES	 (SNDRV_PCM_RATE_48000)
#define TWL6030_FORMATS	 (SNDRV_PCM_FMTBIT_S32_LE)

/* codec private data */
struct twl6030_data {
	struct snd_soc_codec codec;
	int audpwron;
	int codec_powered;
};

/*
 * twl6030 register cache & default register settings
 */
static const u8 twl6030_reg[TWL6030_CACHEREGNUM] = {
	0x00, /* not used		0x00	*/
	0x4B, /* TWL6030_ASICID (ro)	0x01	*/
	0x00, /* TWL6030_ASICREV (ro)	0x02	*/
	0x00, /* TWL6030_INTID		0x03	*/
	0x7B, /* TWL6030_INTMR		0x04	*/
	0x00, /* TWL6030_NCPCTRL	0x05	*/
	0x00, /* TWL6030_LDOCTL		0x06	*/
	0x00, /* TWL6030_HPPLLCTL	0x07	*/
	0x00, /* TWL6030_LPPLLCTL	0x08	*/
	0x00, /* TWL6030_LPPLLDIV	0x09	*/
	0x00, /* TWL6030_AMICBCTL	0x0A	*/
	0x00, /* TWL6030_DMICBCTL	0x0B	*/
	0x18, /* TWL6030_MICLCTL	0x0C	*/
	0x18, /* TWL6030_MICRCTL	0x0D	*/
	0x00, /* TWL6030_MICGAIN	0x0E	*/
	0x1B, /* TWL6030_LINEGAIN	0x0F	*/
	0x00, /* TWL6030_HSLCTL		0x10	*/
	0x00, /* TWL6030_HSRCTL		0x11	*/
	0x00, /* TWL6030_HSGAIN		0x12	*/
	0x06, /* TWL6030_EARCTL		0x13	*/
	0x00, /* TWL6030_HFLCTL		0x14	*/
	0x03, /* TWL6030_HFLGAIN	0x15	*/
	0x00, /* TWL6030_HFRCTL		0x16	*/
	0x03, /* TWL6030_HFRGAIN	0x17	*/
	0x00, /* TWL6030_VIBCTLL	0x18	*/
	0x00, /* TWL6030_VIBDATL	0x19	*/
	0x00, /* TWL6030_VIBCTLR	0x1A	*/
	0x00, /* TWL6030_VIBDATR	0x1B	*/
	0x00, /* TWL6030_HKCTL1		0x1C	*/
	0x00, /* TWL6030_HKCTL2		0x1D	*/
	0x02, /* TWL6030_GPOCTL		0x1E	*/
	0x00, /* TWL6030_ALB		0x1F	*/
	0x00, /* TWL6030_DLB		0x20	*/
	0x00, /* not used		0x21	*/
	0x00, /* not used		0x22	*/
	0x00, /* not used		0x23	*/
	0x00, /* not used		0x24	*/
	0x00, /* not used		0x25	*/
	0x00, /* not used		0x26	*/
	0x00, /* not used		0x27	*/
	0x00, /* TWL6030_TRIM1		0x28	*/
	0x00, /* TWL6030_TRIM2		0x29	*/
	0x00, /* TWL6030_TRIM3		0x2A	*/
	0x00, /* TWL6030_HSOTRIM	0x2B	*/
	0x00, /* TWL6030_HFOTRIM	0x2C	*/
	0x09, /* TWL6030_ACCCTL		0x2D	*/
	0x00, /* TWL6030_STATUS (ro)	0x2E	*/
};

/*
 * twl6030 vio/gnd registers:
 * registers under vio/gnd supply can be accessed
 * before the power-up sequence, after NRESPWRON goes high
 */
static const int twl6030_vio_reg[TWL6030_VIOREGNUM] = {
	TWL6030_REG_ASICID,
	TWL6030_REG_ASICREV,
	TWL6030_REG_INTID,
	TWL6030_REG_INTMR,
	TWL6030_REG_NCPCTL,
	TWL6030_REG_LDOCTL,
	TWL6030_REG_AMICBCTL,
	TWL6030_REG_DMICBCTL,
	TWL6030_REG_HKCTL1,
	TWL6030_REG_HKCTL2,
	TWL6030_REG_GPOCTL,
	TWL6030_REG_TRIM1,
	TWL6030_REG_TRIM2,
	TWL6030_REG_TRIM3,
	TWL6030_REG_HSOTRIM,
	TWL6030_REG_HFOTRIM,
	TWL6030_REG_ACCCTL,
	TWL6030_REG_STATUS,
};

/*
 * twl6030 vdd/vss registers:
 * registers under vdd/vss supplies can only be accessed
 * after the power-up sequence
 */
static const int twl6030_vdd_reg[TWL6030_VDDREGNUM] = {
	TWL6030_REG_HPPLLCTL,
	TWL6030_REG_LPPLLCTL,
	TWL6030_REG_LPPLLDIV,
	TWL6030_REG_MICLCTL,
	TWL6030_REG_MICRCTL,
	TWL6030_REG_MICGAIN,
	TWL6030_REG_LINEGAIN,
	TWL6030_REG_HSLCTL,
	TWL6030_REG_HSRCTL,
	TWL6030_REG_HSGAIN,
	TWL6030_REG_EARCTL,
	TWL6030_REG_HFLCTL,
	TWL6030_REG_HFLGAIN,
	TWL6030_REG_HFRCTL,
	TWL6030_REG_HFRGAIN,
	TWL6030_REG_VIBCTLL,
	TWL6030_REG_VIBDATL,
	TWL6030_REG_VIBCTLR,
	TWL6030_REG_VIBDATR,
	TWL6030_REG_ALB,
	TWL6030_REG_DLB,
};

/*
 * read twl6030 register cache
 */
static inline unsigned int twl6030_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6030_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl6030 register cache
 */
static inline void twl6030_write_reg_cache(struct snd_soc_codec *codec,
						u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6030_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * read from twl6030 hardware register
 */
static int twl6030_read(struct snd_soc_codec *codec,
			unsigned int reg)
{
	u8 value;

	if (reg >= TWL6030_CACHEREGNUM)
		return -EIO;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &value, reg);
	twl6030_write_reg_cache(codec, reg, value);

	return value;
}

/*
 * write to the twl6030 register space
 */
static int twl6030_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	if (reg >= TWL6030_CACHEREGNUM)
		return -EIO;

	twl6030_write_reg_cache(codec, reg, value);
	return twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, value, reg);
}

static void twl6030_init_vio_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	/* allow registers to be accessed by i2c */
	twl6030_write(codec, TWL6030_REG_ACCCTL, cache[TWL6030_REG_ACCCTL]);

	for (i = 0; i < TWL6030_VIOREGNUM; i++) {
		reg = twl6030_vio_reg[i];
		/* skip read-only registers (ASICID, ASICREV, STATUS) */
		if ((reg == TWL6030_REG_ASICID) ||
		    (reg == TWL6030_REG_ASICREV) ||
		    (reg == TWL6030_REG_STATUS))
			continue;
		twl6030_write(codec, reg, cache[reg]);
	}
}

static void twl6030_init_vdd_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	for (i = 0; i < TWL6030_VDDREGNUM; i++) {
		reg = twl6030_vdd_reg[i];
		twl6030_write(codec, reg, cache[reg]);
	}
}

/*
 * MICATT volume control:
 * from -6 to 0 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_preamp_tlv, -600, 600, 0);

/*
 * MICGAIN volume control:
 * from 6 to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_amp_tlv, 600, 600, 0);

/*
 * HSGAIN volume control:
 * from -30 to 0 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hs_tlv, -3000, 200, 0);

/*
 * HFGAIN volume control:
 * from -52 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(hf_tlv, -5200, 200, 0);

/* Left analog microphone selection */
static const char *twl6030_amicl_texts[] =
	{"Headset Mic", "Main Mic", "Aux/FM Left", "Off"};

/* Right analog microphone selection */
static const char *twl6030_amicr_texts[] =
	{"Headset Mic", "Sub Mic", "Aux/FM Right", "Off"};

static const struct soc_enum twl6030_enum[] = {
	SOC_ENUM_SINGLE(TWL6030_REG_MICLCTL, 3, 3, twl6030_amicl_texts),
	SOC_ENUM_SINGLE(TWL6030_REG_MICRCTL, 3, 3, twl6030_amicr_texts),
};

static const struct snd_kcontrol_new amicl_control =
	SOC_DAPM_ENUM("Route", twl6030_enum[0]);

static const struct snd_kcontrol_new amicr_control =
	SOC_DAPM_ENUM("Route", twl6030_enum[1]);

/* Headset DAC playback switches */
static const struct snd_kcontrol_new hsdacl_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HSLCTL, 5, 1, 0);

static const struct snd_kcontrol_new hsdacr_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HSRCTL, 5, 1, 0);

/* Handsfree DAC playback switches */
static const struct snd_kcontrol_new hfdacl_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HFLCTL, 2, 1, 0);

static const struct snd_kcontrol_new hfdacr_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HFRCTL, 2, 1, 0);

/* Headset driver switches */
static const struct snd_kcontrol_new hsl_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HSLCTL, 2, 1, 0);

static const struct snd_kcontrol_new hsr_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HSRCTL, 2, 1, 0);

/* Handsfree driver switches */
static const struct snd_kcontrol_new hfl_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HFLCTL, 4, 1, 0);

static const struct snd_kcontrol_new hfr_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6030_REG_HFRCTL, 4, 1, 0);

static const struct snd_kcontrol_new twl6030_snd_controls[] = {
	/* Capture gains */
	SOC_DOUBLE_TLV("Capture Preamplifier Volume",
		TWL6030_REG_MICGAIN, 6, 7, 1, 1, mic_preamp_tlv),
	SOC_DOUBLE_TLV("Capture Volume",
		TWL6030_REG_MICGAIN, 0, 3, 4, 0, mic_amp_tlv),

	/* Playback gains */
	SOC_DOUBLE_TLV("Headset Playback Volume",
		TWL6030_REG_HSGAIN, 0, 4, 0xF, 1, hs_tlv),
	SOC_DOUBLE_R_TLV("Handsfree Playback Volume",
		TWL6030_REG_HFLGAIN, TWL6030_REG_HFRGAIN, 0, 0x1D, 1, hf_tlv),

};

static const struct snd_soc_dapm_widget twl6030_dapm_widgets[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MAINMIC"),
	SND_SOC_DAPM_INPUT("HSMIC"),
	SND_SOC_DAPM_INPUT("SUBMIC"),
	SND_SOC_DAPM_INPUT("AFML"),
	SND_SOC_DAPM_INPUT("AFMR"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HSOL"),
	SND_SOC_DAPM_OUTPUT("HSOR"),
	SND_SOC_DAPM_OUTPUT("HFL"),
	SND_SOC_DAPM_OUTPUT("HFR"),

	/* Analog input muxes for the capture amplifiers */
	SND_SOC_DAPM_MUX("Analog Left Capture Route",
			SND_SOC_NOPM, 0, 0, &amicl_control),
	SND_SOC_DAPM_MUX("Analog Right Capture Route",
			SND_SOC_NOPM, 0, 0, &amicr_control),

	/* Analog capture PGAs */
	SND_SOC_DAPM_PGA("MicAmpL",
			TWL6030_REG_MICLCTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MicAmpR",
			TWL6030_REG_MICRCTL, 0, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("ADC Left", "Left Front Capture",
			TWL6030_REG_MICLCTL, 2, 0),
	SND_SOC_DAPM_ADC("ADC Right", "Right Front Capture",
			TWL6030_REG_MICRCTL, 2, 0),

	/* Microphone bias */
	SND_SOC_DAPM_MICBIAS("Headset Mic Bias",
			TWL6030_REG_AMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Main Mic Bias",
			TWL6030_REG_AMICBCTL, 4, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic1 Bias",
			TWL6030_REG_DMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic2 Bias",
			TWL6030_REG_DMICBCTL, 4, 0),

	/* DACs */
	SND_SOC_DAPM_DAC("HSDAC Left", "Headset Playback",
			TWL6030_REG_HSLCTL, 0, 0),
	SND_SOC_DAPM_DAC("HSDAC Right", "Headset Playback",
			TWL6030_REG_HSRCTL, 0, 0),
	SND_SOC_DAPM_DAC("HFDAC Left", "Handsfree Playback",
			TWL6030_REG_HFLCTL, 0, 0),
	SND_SOC_DAPM_DAC("HFDAC Right", "Handsfree Playback",
			TWL6030_REG_HFRCTL, 0, 0),

	/* Analog playback switches */
	SND_SOC_DAPM_SWITCH("HSDAC Left Playback",
			SND_SOC_NOPM, 0, 0, &hsdacl_switch_controls),
	SND_SOC_DAPM_SWITCH("HSDAC Right Playback",
			SND_SOC_NOPM, 0, 0, &hsdacr_switch_controls),
	SND_SOC_DAPM_SWITCH("HFDAC Left Playback",
			SND_SOC_NOPM, 0, 0, &hfdacl_switch_controls),
	SND_SOC_DAPM_SWITCH("HFDAC Right Playback",
			SND_SOC_NOPM, 0, 0, &hfdacr_switch_controls),

	SND_SOC_DAPM_SWITCH("Headset Left Driver",
			SND_SOC_NOPM, 0, 0, &hsl_driver_switch_controls),
	SND_SOC_DAPM_SWITCH("Headset Right Driver",
			SND_SOC_NOPM, 0, 0, &hsr_driver_switch_controls),
	SND_SOC_DAPM_SWITCH("Handsfree Left Driver",
			SND_SOC_NOPM, 0, 0, &hfl_driver_switch_controls),
	SND_SOC_DAPM_SWITCH("Handsfree Right Driver",
			SND_SOC_NOPM, 0, 0, &hfr_driver_switch_controls),

	/* Analog playback PGAs */
	SND_SOC_DAPM_PGA("HFDAC Left PGA",
			TWL6030_REG_HFLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HFDAC Right PGA",
			TWL6030_REG_HFRCTL, 1, 0, NULL, 0),

};

static const struct snd_soc_dapm_route intercon[] = {
	/* Capture path */
	{"Analog Left Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Left Capture Route", "Main Mic", "MAINMIC"},
	{"Analog Left Capture Route", "Aux/FM Left", "AFML"},

	{"Analog Right Capture Route", "Headset Mic", "HSMIC"},
	{"Analog Right Capture Route", "Sub Mic", "SUBMIC"},
	{"Analog Right Capture Route", "Aux/FM Right", "AFMR"},

	{"MicAmpL", NULL, "Analog Left Capture Route"},
	{"MicAmpR", NULL, "Analog Right Capture Route"},

	{"ADC Left", NULL, "MicAmpL"},
	{"ADC Right", NULL, "MicAmpR"},

	/* Headset playback path */
	{"HSDAC Left Playback", "Switch", "HSDAC Left"},
	{"HSDAC Right Playback", "Switch", "HSDAC Right"},

	{"Headset Left Driver", "Switch", "HSDAC Left Playback"},
	{"Headset Right Driver", "Switch", "HSDAC Right Playback"},

	{"HSOL", NULL, "Headset Left Driver"},
	{"HSOR", NULL, "Headset Right Driver"},

	/* Handsfree playback path */
	{"HFDAC Left Playback", "Switch", "HFDAC Left"},
	{"HFDAC Right Playback", "Switch", "HFDAC Right"},

	{"HFDAC Left PGA", NULL, "HFDAC Left Playback"},
	{"HFDAC Right PGA", NULL, "HFDAC Right Playback"},

	{"Handsfree Left Driver", "Switch", "HFDAC Left PGA"},
	{"Handsfree Right Driver", "Switch", "HFDAC Right PGA"},

	{"HFL", NULL, "Handsfree Left Driver"},
	{"HFR", NULL, "Handsfree Right Driver"},
};

static int twl6030_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, twl6030_dapm_widgets,
				 ARRAY_SIZE(twl6030_dapm_widgets));

	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);

	return 0;
}

static int twl6030_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct twl6030_data *priv = codec->private_data;
	int audpwron = priv->audpwron;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 1);

			/* power-up sequence latency */
			mdelay(16);
		}

		/* sync registers updated during power-up sequence */
		twl6030_read(codec, TWL6030_REG_NCPCTL);
		twl6030_read(codec, TWL6030_REG_LDOCTL);
		twl6030_read(codec, TWL6030_REG_LPPLLCTL);

		/* initialize vdd/vss registers with reg_cache */
		twl6030_init_vdd_regs(codec);

		priv->codec_powered = 1;
		break;
	case SND_SOC_BIAS_OFF:
		if (!priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 0);

			/* power-down sequence latency */
			udelay(500);
		}

		/* sync registers updated during power-down sequence */
		twl6030_read(codec, TWL6030_REG_NCPCTL);
		twl6030_read(codec, TWL6030_REG_LDOCTL);
		twl6030_write_reg_cache(codec, TWL6030_REG_LPPLLCTL, 0x00);

		priv->codec_powered = 0;
		break;
	}

	codec->bias_level = level;

	return 0;
}

static int twl6030_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 hppllctl, lppllctl;

	hppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_HPPLLCTL);
	hppllctl &= ~TWL6030_MCLK_MSK;

	switch (freq) {
	case 12000000:
		/* mclk input, pll enabled */
		hppllctl |= TWL6030_MCLK_12000KHZ |
			    TWL6030_HPLLSQRBP |
			    TWL6030_HPLLENA;
		break;
	case 19200000:
		/* mclk input, pll disabled */
		hppllctl |= TWL6030_MCLK_19200KHZ |
			    TWL6030_HPLLSQRBP |
			    TWL6030_HPLLBP;
		break;
	case 26000000:
		/* mclk input, pll enabled */
		hppllctl |= TWL6030_MCLK_26000KHZ |
			    TWL6030_HPLLSQRBP |
			    TWL6030_HPLLENA;
		break;
	case 38400000:
		/* clk slicer, pll disabled */
		hppllctl |= TWL6030_MCLK_38400KHZ |
			    TWL6030_HPLLSQRENA |
			    TWL6030_HPLLBP;
		break;
	default:
		dev_err(codec->dev, "unknown sysclk freq %d\n", freq);
		return -EINVAL;
	}

	twl6030_write(codec, TWL6030_REG_HPPLLCTL, hppllctl);

	/* Disable LPPLL and select HPPLL */
	lppllctl = TWL6030_HPLLSEL;
	twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);

	return 0;
}

static struct snd_soc_dai_ops twl6030_dai_ops = {
	.set_sysclk	= twl6030_set_dai_sysclk,
};

struct snd_soc_dai twl6030_dai = {
	.name = "twl6030",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 4,
		.rates = TWL6030_RATES,
		.formats = TWL6030_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = TWL6030_RATES,
		.formats = TWL6030_FORMATS,
	},
	.ops = &twl6030_dai_ops,
};
EXPORT_SYMBOL_GPL(twl6030_dai);

static int twl6030_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int twl6030_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	twl6030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	twl6030_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

static struct snd_soc_codec *twl6030_codec;

static int twl6030_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	BUG_ON(!twl6030_codec);

	codec = twl6030_codec;
	socdev->card->codec = codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create pcms\n");
		return ret;
	}

	snd_soc_add_controls(codec, twl6030_snd_controls,
				ARRAY_SIZE(twl6030_snd_controls));
	twl6030_add_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	return ret;
}

static int twl6030_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_twl6030 = {
	.probe = twl6030_probe,
	.remove = twl6030_remove,
	.suspend = twl6030_suspend,
	.resume = twl6030_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_twl6030);

static int __devinit twl6030_codec_probe(struct platform_device *pdev)
{
	struct twl4030_codec_data *twl_codec = pdev->dev.platform_data;
	struct snd_soc_codec *codec;
	struct twl6030_data *priv;
	int audpwron;
	int ret = 0;

	priv = kzalloc(sizeof(struct twl6030_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	if (twl_codec)
		audpwron = twl_codec->audpwron_gpio;
	else
		audpwron = -EINVAL;

	priv->audpwron = audpwron;

	codec = &priv->codec;
	codec->dev = &pdev->dev;
	twl6030_dai.dev = &pdev->dev;

	codec->name = "twl6030";
	codec->owner = THIS_MODULE;
	codec->read = twl6030_read_reg_cache;
	codec->write = twl6030_write;
	codec->set_bias_level = twl6030_set_bias_level;
	codec->private_data = priv;
	codec->dai = &twl6030_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(twl6030_reg);
	codec->reg_cache = kmemdup(twl6030_reg, sizeof(twl6030_reg),
					GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto cache_err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	if (gpio_is_valid(audpwron)) {
		ret = gpio_request(audpwron, "audpwron");
		if (ret)
			goto gpio1_err;

		ret = gpio_direction_output(audpwron, 0);
		if (ret)
			goto gpio2_err;

		priv->codec_powered = 0;
	} else {
		/* if no gpio is provided, then assume its always on */
		priv->codec_powered = 1;
	}

	/* init vio registers */
	twl6030_init_vio_regs(codec);

	/* power on device */
	ret = twl6030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto gpio2_err;

	ret = snd_soc_register_codec(codec);
	if (ret)
		goto reg_err;

	twl6030_codec = codec;

	ret = snd_soc_register_dai(&twl6030_dai);
	if (ret)
		goto dai_err;

	return 0;

dai_err:
	snd_soc_unregister_codec(codec);
	twl6030_codec = NULL;
reg_err:
	twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);
gpio2_err:
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	kfree(codec->reg_cache);
cache_err:
	kfree(priv);
	return ret;
}

static int __devexit twl6030_codec_remove(struct platform_device *pdev)
{
	struct twl6030_data *priv = twl6030_codec->private_data;
	int audpwron = priv->audpwron;

	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

	snd_soc_unregister_dai(&twl6030_dai);
	snd_soc_unregister_codec(twl6030_codec);

	kfree(twl6030_codec);
	twl6030_codec = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int twl6030_codec_suspend(struct platform_device *pdev,
				 pm_message_t msg)
{
	return snd_soc_suspend_device(&pdev->dev);
}

static int twl6030_codec_resume(struct platform_device *pdev)
{
	return snd_soc_resume_device(&pdev->dev);
}
#else
#define twl6030_codec_suspend NULL
#define twl6030_codec_resume NULL
#endif

static struct platform_driver twl6030_codec_driver = {
	.driver = {
		.name = "twl6030_codec",
		.owner = THIS_MODULE,
	},
	.probe = twl6030_codec_probe,
	.remove = __devexit_p(twl6030_codec_remove),
	.suspend = twl6030_codec_suspend,
	.resume = twl6030_codec_resume,
};

static int __init twl6030_codec_init(void)
{
	return platform_driver_register(&twl6030_codec_driver);
}
module_init(twl6030_codec_init);

static void __exit twl6030_codec_exit(void)
{
	platform_driver_unregister(&twl6030_codec_driver);
}
module_exit(twl6030_codec_exit);

MODULE_DESCRIPTION("ASoC TWL6030 codec driver");
MODULE_AUTHOR("Misael Lopez Cruz");
MODULE_LICENSE("GPL");
