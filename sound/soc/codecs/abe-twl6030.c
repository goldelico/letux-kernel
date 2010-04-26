/*
 * ALSA SoC ABE-TWL6030 codec driver
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
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "twl6030.h"
#include "abe-twl6030.h"
#include "abe/abe_main.h"

#define ABE_FORMATS	 (SNDRV_PCM_FMTBIT_S32_LE)

/* codec private data */
struct twl6030_data {
	struct snd_soc_codec codec;
	int audpwron;
	int naudint;
	int codec_powered;
	int pll;
	int non_lp;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct completion ready;
	struct clk *clk;
	int configure;
};

/*
 * twl6030 register cache & default register settings
 */
static const u8 twl6030_reg[TWL6030_CACHEREGNUM] = {
	0x00, /* not used		0x00	*/
	0x4B, /* TWL6030_ASICID (ro)	0x01	*/
	0x00, /* TWL6030_ASICREV (ro)	0x02	*/
	0x00, /* TWL6030_INTID		0x03	*/
	0x00, /* TWL6030_INTMR		0x04	*/
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

static void abe_init_chip(struct snd_soc_codec *codec)
{
	struct twl6030_data *priv = codec->private_data;
	abe_opp_t OPP;

	abe_init_mem();

	/* aess_clk has to be enabled to access hal registers.
	 * Disable the clk after it has been used.
	 */
	clk_enable(priv->clk);
	abe_reset_hal();
	/* Config OPP 100 FOR NOW */
	abe_set_opp_processing(OPP);
	/* "tick" of the audio engine */
	abe_write_event_generator(EVENT_TIMER);

	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_0MS, MIX_DL1_INPUT_MM_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_MM_UL2);
	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_0MS, MIX_DL1_INPUT_VX_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_TONES);

	abe_write_mixer(MIXDL2, GAIN_M6dB, RAMP_0MS, MIX_DL2_INPUT_TONES);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_0MS, MIX_DL2_INPUT_VX_DL);
	abe_write_mixer(MIXDL2, GAIN_M6dB, RAMP_0MS, MIX_DL2_INPUT_MM_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_0MS, MIX_DL2_INPUT_MM_UL2);

	clk_disable(priv->clk);
}

/* twl6030 codec manual power-up sequence */
static void twl6030_power_up(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6030_read_reg_cache(codec, TWL6030_REG_NCPCTL);
	ldoctl = twl6030_read_reg_cache(codec, TWL6030_REG_LDOCTL);
	lppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_LPPLLCTL);
	accctl = twl6030_read_reg_cache(codec, TWL6030_REG_ACCCTL);

	/* enable reference system */
	ldoctl |= TWL6030_REFENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	mdelay(10);
	/* enable internal oscillator */
	ldoctl |= TWL6030_OSCENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(10);
	/* enable high-side ldo */
	ldoctl |= TWL6030_HSLDOENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable negative charge pump */
	ncpctl |= TWL6030_NCPENA | TWL6030_NCPOPEN;
	twl6030_write(codec, TWL6030_REG_NCPCTL, ncpctl);
	udelay(488);
	/* enable low-side ldo */
	ldoctl |= TWL6030_LSLDOENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable low-power pll */
	lppllctl |= TWL6030_LPLLENA;
	twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
	/* reset state machine */
	accctl |= TWL6030_RESETSPLIT;
	twl6030_write(codec, TWL6030_REG_ACCCTL, accctl);
	mdelay(5);
	accctl &= ~TWL6030_RESETSPLIT;
	twl6030_write(codec, TWL6030_REG_ACCCTL, accctl);
	/* disable internal oscillator */
	ldoctl &= ~TWL6030_OSCENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
}

/* twl6030 codec manual power-down sequence */
static void twl6030_power_down(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6030_read_reg_cache(codec, TWL6030_REG_NCPCTL);
	ldoctl = twl6030_read_reg_cache(codec, TWL6030_REG_LDOCTL);
	lppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_LPPLLCTL);
	accctl = twl6030_read_reg_cache(codec, TWL6030_REG_ACCCTL);

	/* enable internal oscillator */
	ldoctl |= TWL6030_OSCENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(10);
	/* disable low-power pll */
	lppllctl &= ~TWL6030_LPLLENA;
	twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
	/* disable low-side ldo */
	ldoctl &= ~TWL6030_LSLDOENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable negative charge pump */
	ncpctl &= ~(TWL6030_NCPENA | TWL6030_NCPOPEN);
	twl6030_write(codec, TWL6030_REG_NCPCTL, ncpctl);
	udelay(488);
	/* disable high-side ldo */
	ldoctl &= ~TWL6030_HSLDOENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable internal oscillator */
	ldoctl &= ~TWL6030_OSCENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	/* disable reference system */
	ldoctl &= ~TWL6030_REFENA;
	twl6030_write(codec, TWL6030_REG_LDOCTL, ldoctl);
	mdelay(10);
}

/* set headset dac and driver power mode */
static int headset_power_mode(struct snd_soc_codec *codec, int high_perf)
{
	int hslctl, hsrctl;
	int mask = TWL6030_HSDRVMODEL | TWL6030_HSDACMODEL;

	hslctl = twl6030_read_reg_cache(codec, TWL6030_REG_HSLCTL);
	hsrctl = twl6030_read_reg_cache(codec, TWL6030_REG_HSRCTL);

	if (high_perf) {
		hslctl &= ~mask;
		hsrctl &= ~mask;
	} else {
		hslctl |= mask;
		hsrctl |= mask;
	}

	twl6030_write(codec, TWL6030_REG_HSLCTL, hslctl);
	twl6030_write(codec, TWL6030_REG_HSRCTL, hsrctl);

	return 0;
}

static int twl6030_power_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct twl6030_data *priv = codec->private_data;

	if (SND_SOC_DAPM_EVENT_ON(event))
		priv->non_lp++;
	else
		priv->non_lp--;

	return 0;
}

/* audio interrupt handler */
static irqreturn_t twl6030_naudint_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct twl6030_data *priv = codec->private_data;
	u8 intid;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid, TWL6030_REG_INTID);

	switch (intid) {
	case TWL6030_THINT:
		dev_alert(codec->dev, "die temp over-limit detection\n");
		break;
	case TWL6030_PLUGINT:
	case TWL6030_UNPLUGINT:
	case TWL6030_HOOKINT:
		break;
	case TWL6030_HFINT:
		dev_alert(codec->dev, "hf drivers over current detection\n");
		break;
	case TWL6030_VIBINT:
		dev_alert(codec->dev, "vib drivers over current detection\n");
		break;
	case TWL6030_READYINT:
		complete(&priv->ready);
		break;
	default:
		dev_err(codec->dev, "unknown audio interrupt %d\n", intid);
		break;
	}

	return IRQ_HANDLED;
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
	SND_SOC_DAPM_DAC_E("HFDAC Left", "Handsfree Playback",
			TWL6030_REG_HFLCTL, 0, 0,
			twl6030_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Right", "Handsfree Playback",
			TWL6030_REG_HFRCTL, 0, 0,
			twl6030_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Analog playback switches */
	SND_SOC_DAPM_SWITCH("HSDAC Left Playback",
			SND_SOC_NOPM, 0, 0, &hsdacl_switch_controls),
	SND_SOC_DAPM_SWITCH("HSDAC Right Playback",
			SND_SOC_NOPM, 0, 0, &hsdacr_switch_controls),
	SND_SOC_DAPM_SWITCH("HFDAC Left Playback",
			SND_SOC_NOPM, 0, 0, &hfdacl_switch_controls),
	SND_SOC_DAPM_SWITCH("HFDAC Right Playback",
			SND_SOC_NOPM, 0, 0, &hfdacr_switch_controls),

	/* Analog playback drivers */
	SND_SOC_DAPM_PGA_E("Handsfree Left Driver",
			TWL6030_REG_HFLCTL, 4, 0, NULL, 0,
			twl6030_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("Handsfree Right Driver",
			TWL6030_REG_HFRCTL, 4, 0, NULL, 0,
			twl6030_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA("Headset Left Driver",
			TWL6030_REG_HSLCTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Headset Right Driver",
			TWL6030_REG_HSRCTL, 2, 0, NULL, 0),

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

	{"Headset Left Driver", NULL, "HSDAC Left Playback"},
	{"Headset Right Driver", NULL, "HSDAC Right Playback"},

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

static int abe_twl6030_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, twl6030_dapm_widgets,
				 ARRAY_SIZE(twl6030_dapm_widgets));

	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);

	return 0;
}

static int twl6030_power_up_completion(struct snd_soc_codec *codec,
					int naudint)
{
	struct twl6030_data *priv = codec->private_data;
	int time_left;
	u8 intid;

	if (naudint) {
		/* wait for ready interrupt with 48 ms timeout */
		time_left = wait_for_completion_timeout(&priv->ready,
					msecs_to_jiffies(48));
	} else {
		/* retry 3 times only */
		for (time_left = 3; time_left > 0; time_left--) {
			mdelay(16);
			twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid,
					TWL6030_REG_INTID);
			if (intid & TWL6030_READYINT)
				break;
		}
	}

	if (!time_left) {
		dev_err(codec->dev, "timeout waiting for READYINT\n");
		return -ETIMEDOUT;
	}

	priv->codec_powered = 1;

	return 0;
}

static int abe_twl6030_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct twl6030_data *priv = codec->private_data;
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 1);

			/* wait for power-up completion */
			ret = twl6030_power_up_completion(codec, naudint);
			if (ret)
				return ret;

			/* sync registers updated during power-up sequence */
			twl6030_read(codec, TWL6030_REG_NCPCTL);
			twl6030_read(codec, TWL6030_REG_LDOCTL);
			twl6030_read(codec, TWL6030_REG_LPPLLCTL);
		} else {
			/* use manual power-up sequence */
			twl6030_power_up(codec);
			priv->codec_powered = 1;
		}

		/* initialize vdd/vss registers with reg_cache */
		twl6030_init_vdd_regs(codec);
		break;
	case SND_SOC_BIAS_OFF:
		if (!priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 0);

			/* power-down sequence latency */
			udelay(500);

			/* sync registers updated during power-down sequence */
			twl6030_read(codec, TWL6030_REG_NCPCTL);
			twl6030_read(codec, TWL6030_REG_LDOCTL);
			twl6030_write_reg_cache(codec, TWL6030_REG_LPPLLCTL,
						0x00);
		} else {
			/* use manual power-down sequence */
			twl6030_power_down(codec);
		}

		priv->codec_powered = 0;
		break;
	}

	codec->bias_level = level;

	return 0;
}

/* set of rates for each pll: low-power and high-performance */

static unsigned int lp_rates[] = {
	44100,
	48000,
};

static struct snd_pcm_hw_constraint_list lp_constraints = {
	.count	= ARRAY_SIZE(lp_rates),
	.list	= lp_rates,
};

static unsigned int hp_rates[] = {
	8000,
	16000,
	48000,
};

static struct snd_pcm_hw_constraint_list hp_constraints = {
	.count	= ARRAY_SIZE(hp_rates),
	.list	= hp_rates,
};

static int twl6030_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl6030_data *priv = codec->private_data;
	u8 hppllctl, lppllctl;

	hppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_HPPLLCTL);
	lppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_LPPLLCTL);

	switch (clk_id) {
	case TWL6030_SYSCLK_SEL_LPPLL:
		switch (freq) {
		case 32768:
			/* headset dac and driver must be in low-power mode */
			headset_power_mode(codec, 0);

			/* clk32k input requires low-power pll */
			lppllctl |= TWL6030_LPLLENA;
			twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
			mdelay(5);
			lppllctl &= ~TWL6030_HPLLSEL;
			twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
			hppllctl &= ~TWL6030_HPLLENA;
			twl6030_write(codec, TWL6030_REG_HPPLLCTL, hppllctl);
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* lppll divider */
		switch (priv->sysclk) {
		case 17640000:
			lppllctl |= TWL6030_LPLLFIN;
			break;
		case 19200000:
			lppllctl &= ~TWL6030_LPLLFIN;
			break;
		default:
			/* sysclk not yet configured */
			lppllctl &= ~TWL6030_LPLLFIN;
			priv->sysclk = 19200000;
			break;
		}

		twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);

		priv->pll = TWL6030_LPPLL_ID;
		priv->sysclk_constraints = &lp_constraints;
		break;
	case TWL6030_SYSCLK_SEL_HPPLL:
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
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* headset dac and driver must be in high-performance mode */
		headset_power_mode(codec, 1);

		twl6030_write(codec, TWL6030_REG_HPPLLCTL, hppllctl);
		udelay(500);
		lppllctl |= TWL6030_HPLLSEL;
		twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
		lppllctl &= ~TWL6030_LPLLENA;
		twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);

		/* high-performance pll can provide only 19.2 MHz */
		priv->pll = TWL6030_HPPLL_ID;
		priv->sysclk = 19200000;
		priv->sysclk_constraints = &hp_constraints;
		break;
	default:
		dev_err(codec->dev, "unknown clk_id %d\n", clk_id);
		return -EINVAL;
	}

	return 0;
}
static int abe_mm_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;

	if (!priv->sysclk) {
		dev_err(codec->dev,
			"no mclk configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	/*
	 * capture is not supported at 17.64 MHz,
	 * it's reserved for headset low-power playback scenario
	 */
	if ((priv->sysclk == 17640000) && substream->stream) {
		dev_err(codec->dev,
			"capture mode is not supported at %dHz\n",
			priv->sysclk);
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				priv->sysclk_constraints);
	if (!priv->configure++) {
		clk_enable(priv->clk);
		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_AMIC,
			(abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_AMIC]);

		abe_write_gain(GAINS_DL1, GAIN_0dB,  RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_DL1, GAIN_0dB,  RAMP_0MS, GAIN_RIGHT_OFFSET);
		abe_write_gain(GAINS_DL2, GAIN_0dB,  RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_DL2, GAIN_0dB,  RAMP_0MS, GAIN_RIGHT_OFFSET);

		abe_write_gain(GAINS_AMIC , GAIN_M6dB, RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_AMIC, GAIN_M6dB, RAMP_0MS, GAIN_RIGHT_OFFSET);

		abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_0MS, GAIN_RIGHT_OFFSET);

		abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_0MS, GAIN_RIGHT_OFFSET);
		abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_0MS, GAIN_LEFT_OFFSET);
		abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_0MS, GAIN_RIGHT_OFFSET);
	}
	return 0;
}

static int abe_mm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;
	u8 lppllctl;
	int rate;
	unsigned int sysclk;
	abe_data_format_t format;
	abe_dma_t dma_sink;

	lppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_LPPLLCTL);

	rate = params_rate(params);
	switch (rate) {
	case 44100:
		lppllctl |= TWL6030_LPLLFIN;
		sysclk = 17640000;
		break;
	case 48000:
	/* Select output frequency 19.2 MHz */
		lppllctl &= ~TWL6030_LPLLFIN;
		sysclk = 19200000;
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

if (priv->pll == TWL6030_LPPLL_ID) {
		priv->sysclk = sysclk;
		twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
	}

	format.f = rate;
	format.samp_format = STEREO_MSB;
	if (!substream->stream)
	abe_connect_cbpr_dmareq_port(MM_DL_PORT, &format, ABE_CBPR0_IDX, &dma_sink);
	else
	abe_connect_cbpr_dmareq_port(MM_UL_PORT, &format, ABE_CBPR3_IDX, &dma_sink);

	return 0;
}

static int abe_mm_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/*
		* low-power playback mode is restricted
		* for headset path only
		*/
		if ((priv->sysclk == 17640000) && priv->non_lp) {
			dev_err(codec->dev,
				"some enabled paths aren't supported at %dHz\n",
				priv->sysclk);
			return -EPERM;
		}
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!substream->stream)
			abe_enable_data_transfer(MM_DL_PORT);
		else
			abe_enable_data_transfer(MM_UL_PORT);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!substream->stream)
			abe_disable_data_transfer(MM_DL_PORT);
		else
			abe_disable_data_transfer(MM_UL_PORT);
		break;
	default:
		break;
	}

	return 0;
}

static void abe_mm_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;

	if (!--priv->configure)
		clk_disable(priv->clk);
}

static struct snd_soc_dai_ops abe_mm_dai_ops = {
	.startup	= abe_mm_startup,
	.hw_params	= abe_mm_hw_params,
	.shutdown	= abe_mm_shutdown,
	.trigger	= abe_mm_trigger,
	.set_sysclk	= twl6030_set_dai_sysclk,
};

static int abe_voice_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;
	u8 lppllctl;
	int rate;
	unsigned int sysclk;
	abe_data_format_t format;
	abe_dma_t dma_sink;

	rate = params_rate(params);
	switch (rate) {
	case 16000:
	case 8000:
	/* Select output frequency 19.2 MHz */
		if (priv->pll == TWL6030_LPPLL_ID) {
			lppllctl = twl6030_read_reg_cache(codec, TWL6030_REG_LPPLLCTL);
			lppllctl &= ~TWL6030_LPLLFIN;
			priv->sysclk = 19200000;
			twl6030_write(codec, TWL6030_REG_LPPLLCTL, lppllctl);
		}
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	format.f = rate;
	format.samp_format = STEREO_MSB;
	if (!substream->stream)
		abe_connect_cbpr_dmareq_port(VX_DL_PORT, &format, ABE_CBPR1_IDX, &dma_sink);
	else
		abe_connect_cbpr_dmareq_port(VX_UL_PORT, &format, ABE_CBPR2_IDX, &dma_sink);

	return 0;
}

static int abe_voice_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6030_data *priv = codec->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/*
		 * low-power playback mode is restricted
		 * for headset path only
		 */
		if ((priv->sysclk == 17640000) && priv->non_lp) {
			dev_err(codec->dev,
				"some enabled paths aren't supported at %dHz\n",
				priv->sysclk);
				return -EPERM;
		}
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!substream->stream)
			abe_enable_data_transfer(VX_DL_PORT);
		else
			abe_enable_data_transfer(VX_UL_PORT);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!substream->stream)
			abe_disable_data_transfer(VX_DL_PORT);
		else
			abe_disable_data_transfer(VX_UL_PORT);
		break;
	default:
		break;
	}

	return 0;
}

static struct snd_soc_dai_ops abe_voice_dai_ops = {
	.startup	= abe_mm_startup,
	.hw_params	= abe_voice_hw_params,
	.shutdown	= abe_mm_shutdown,
	.trigger	= abe_voice_trigger,
	.set_sysclk	= twl6030_set_dai_sysclk,
};

/* Audio Backend DAIs */
struct snd_soc_dai abe_dai[] = {
	/* Multimedia: MM-UL2, MM-DL */
	{
		.name = "Multimedia",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
		.ops = &abe_mm_dai_ops,
	},
	/* Tones DL: MM-DL2 */
	{
		.name = "Tones DL",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
		.ops = &abe_mm_dai_ops,
	},
	/* Voice: VX-UL, VX-DL */
	{
		.name = "Voice",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = ABE_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = ABE_FORMATS,
		},
		.ops = &abe_voice_dai_ops,
	},
	/* Digital Uplink: MM-UL */
	{
		.name = "Digital Uplink",
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
	},
	/* Vibrator: VIB-DL */
	{
		.name = "Vibrator",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
	},
 };

static int abe_twl6030_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int abe_twl6030_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	abe_twl6030_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

static struct snd_soc_codec *twl6030_codec;

static int abe_twl6030_probe(struct platform_device *pdev)
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

	abe_init_chip(codec);
	snd_soc_add_controls(codec, twl6030_snd_controls,
				ARRAY_SIZE(twl6030_snd_controls));
	abe_twl6030_add_widgets(codec);

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

static int abe_twl6030_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_abe_twl6030 = {
	.probe = abe_twl6030_probe,
	.remove = abe_twl6030_remove,
	.suspend = abe_twl6030_suspend,
	.resume = abe_twl6030_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_abe_twl6030);

static int __devinit abe_twl6030_codec_probe(struct platform_device *pdev)
{
	struct twl4030_codec_data *twl_codec = pdev->dev.platform_data;
	struct snd_soc_codec *codec;
	struct twl6030_data *priv;
	int audpwron, naudint;
	int ret = 0;

	priv = kzalloc(sizeof(struct twl6030_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	if (twl_codec) {
		audpwron = twl_codec->audpwron_gpio;
		naudint = twl_codec->naudint_irq;
	} else {
		audpwron = -EINVAL;
		naudint = 0;
	}

	priv->audpwron = audpwron;
	priv->naudint = naudint;

	codec = &priv->codec;
	codec->pop_time = 1;
	codec->dev = &pdev->dev;
	codec->name = "twl6030";
	codec->owner = THIS_MODULE;
	codec->read = twl6030_read_reg_cache;
	codec->write = twl6030_write;
	codec->set_bias_level = abe_twl6030_set_bias_level;
	codec->private_data = priv;
	codec->dai = abe_dai;
	codec->num_dai = ARRAY_SIZE(abe_dai);
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
	init_completion(&priv->ready);

	if (gpio_is_valid(audpwron)) {
		ret = gpio_request(audpwron, "audpwron");
		if (ret)
			goto gpio1_err;

		ret = gpio_direction_output(audpwron, 0);
		if (ret)
			goto gpio2_err;

		priv->codec_powered = 0;
	}

	if (naudint) {
		/* audio interrupt */
		ret = request_threaded_irq(naudint, NULL,
				twl6030_naudint_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"twl6030_codec", codec);
		if (ret)
			goto gpio2_err;
	} else {
		if (gpio_is_valid(audpwron)) {
			/* enable only codec ready interrupt */
			twl6030_write_reg_cache(codec, TWL6030_REG_INTMR,
					~TWL6030_READYMSK & TWL6030_ALLINT_MSK);
		} else {
			/* no interrupts at all */
			twl6030_write_reg_cache(codec, TWL6030_REG_INTMR,
						TWL6030_ALLINT_MSK);
		}
	}

	priv->clk = clk_get(&pdev->dev, "aess_fclk");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_err(&pdev->dev, "unable to get aess_fclk: %d\n", ret);
		goto clk_err;
	}

	/* init vio registers */
	twl6030_init_vio_regs(codec);

	/* power on device */
	ret = abe_twl6030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto irq_err;

	ret = snd_soc_register_codec(codec);
	if (ret)
		goto reg_err;

	twl6030_codec = codec;

	ret = snd_soc_register_dais(abe_dai, ARRAY_SIZE(abe_dai));
	if (ret)
		goto dai_err;

	return 0;

dai_err:
	snd_soc_unregister_codec(codec);
	twl6030_codec = NULL;
reg_err:
	abe_twl6030_set_bias_level(codec, SND_SOC_BIAS_OFF);
irq_err:
	if (naudint)
		free_irq(naudint, codec);
gpio2_err:
	clk_put(priv->clk);
clk_err:
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	kfree(codec->reg_cache);
cache_err:
	kfree(priv);
	return ret;
}

static int __devexit abe_twl6030_codec_remove(struct platform_device *pdev)
{
	struct twl6030_data *priv = twl6030_codec->private_data;
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;

	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

	if (naudint)
		free_irq(naudint, twl6030_codec);

	clk_put(priv->clk);

	snd_soc_unregister_dais(abe_dai, ARRAY_SIZE(abe_dai));
	snd_soc_unregister_codec(twl6030_codec);

	kfree(twl6030_codec);
	twl6030_codec = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int abe_twl6030_codec_suspend(struct platform_device *pdev,
				 pm_message_t msg)
{
	return snd_soc_suspend_device(&pdev->dev);
}

static int abe_twl6030_codec_resume(struct platform_device *pdev)
{
	return snd_soc_resume_device(&pdev->dev);
}
#else
#define abe_twl6030_codec_suspend NULL
#define abe_twl6030_codec_resume NULL
#endif

static struct platform_driver abe_twl6030_codec_driver = {
	.driver = {
		.name = "twl6030_codec",
		.owner = THIS_MODULE,
	},
	.probe = abe_twl6030_codec_probe,
	.remove = __devexit_p(abe_twl6030_codec_remove),
	.suspend = abe_twl6030_codec_suspend,
	.resume = abe_twl6030_codec_resume,
};

static int __init abe_twl6030_codec_init(void)
{
	return platform_driver_register(&abe_twl6030_codec_driver);
}
module_init(abe_twl6030_codec_init);

static void __exit abe_twl6030_codec_exit(void)
{
	platform_driver_unregister(&abe_twl6030_codec_driver);
}
module_exit(abe_twl6030_codec_exit);

MODULE_DESCRIPTION("ASoC ABE-TWL6030 codec driver");
MODULE_AUTHOR("Misael Lopez Cruz");
MODULE_LICENSE("GPL");
