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
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/clk.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "twl6040.h"
#include "abe-twl6040.h"
#include "abe/abe_main.h"

#define ABE_FORMATS	 (SNDRV_PCM_FMTBIT_S32_LE)

struct twl6040_jack_data {
	struct snd_soc_jack *jack;
	int report;
	struct switch_dev sdev;
	struct work_struct work;
	int state;
};

/* codec private data */
struct twl6040_data {
	struct snd_soc_codec codec;
	int audpwron;
	int naudint;
	struct twl6040_jack_data hs_jack;
	int codec_powered;
	int pll;
	int non_lp;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct completion ready;
	int configure;
	int mcpdm_dl_enable;
	int mcpdm_ul_enable;
	struct clk *clk;
};

/*
 * twl6040 register cache & default register settings
 */
static const u8 twl6040_reg[TWL6040_CACHEREGNUM] = {
	0x00, /* not used		0x00	*/
	0x4B, /* TWL6040_ASICID (ro)	0x01	*/
	0x00, /* TWL6040_ASICREV (ro)	0x02	*/
	0x00, /* TWL6040_INTID		0x03	*/
	0x00, /* TWL6040_INTMR		0x04	*/
	0x00, /* TWL6040_NCPCTRL	0x05	*/
	0x00, /* TWL6040_LDOCTL		0x06	*/
	0x60, /* TWL6040_HPPLLCTL	0x07	*/
	0x00, /* TWL6040_LPPLLCTL	0x08	*/
	0x4A, /* TWL6040_LPPLLDIV	0x09	*/
	0x00, /* TWL6040_AMICBCTL	0x0A	*/
	0x00, /* TWL6040_DMICBCTL	0x0B	*/
	0x18, /* TWL6040_MICLCTL	0x0C	*/
	0x18, /* TWL6040_MICRCTL	0x0D	*/
	0x00, /* TWL6040_MICGAIN	0x0E	*/
	0x1B, /* TWL6040_LINEGAIN	0x0F	*/
	0x00, /* TWL6040_HSLCTL		0x10	*/
	0x00, /* TWL6040_HSRCTL		0x11	*/
	0xFF, /* TWL6040_HSGAIN		0x12	*/
	0x1E, /* TWL6040_EARCTL		0x13	*/
	0x00, /* TWL6040_HFLCTL		0x14	*/
	0x1D, /* TWL6040_HFLGAIN	0x15	*/
	0x00, /* TWL6040_HFRCTL		0x16	*/
	0x1D, /* TWL6040_HFRGAIN	0x17	*/
	0x00, /* TWL6040_VIBCTLL	0x18	*/
	0x00, /* TWL6040_VIBDATL	0x19	*/
	0x00, /* TWL6040_VIBCTLR	0x1A	*/
	0x00, /* TWL6040_VIBDATR	0x1B	*/
	0x00, /* TWL6040_HKCTL1		0x1C	*/
	0x00, /* TWL6040_HKCTL2		0x1D	*/
	0x02, /* TWL6040_GPOCTL		0x1E	*/
	0x00, /* TWL6040_ALB		0x1F	*/
	0x00, /* TWL6040_DLB		0x20	*/
	0x00, /* not used		0x21	*/
	0x00, /* not used		0x22	*/
	0x00, /* not used		0x23	*/
	0x00, /* not used		0x24	*/
	0x00, /* not used		0x25	*/
	0x00, /* not used		0x26	*/
	0x00, /* not used		0x27	*/
	0x00, /* TWL6040_TRIM1		0x28	*/
	0x00, /* TWL6040_TRIM2		0x29	*/
	0x00, /* TWL6040_TRIM3		0x2A	*/
	0x00, /* TWL6040_HSOTRIM	0x2B	*/
	0x00, /* TWL6040_HFOTRIM	0x2C	*/
	0x09, /* TWL6040_ACCCTL		0x2D	*/
	0x00, /* TWL6040_STATUS (ro)	0x2E	*/
	0x00, /* TWL6040_SHADOW		0x2F	*/
	0xAA, /* ABE_AMIC_GAIN_SHADOW	0x30	*/
};

/*
 * twl6040 vio/gnd registers:
 * registers under vio/gnd supply can be accessed
 * before the power-up sequence, after NRESPWRON goes high
 */
static const int twl6040_vio_reg[TWL6040_VIOREGNUM] = {
	TWL6040_REG_ASICID,
	TWL6040_REG_ASICREV,
	TWL6040_REG_INTID,
	TWL6040_REG_INTMR,
	TWL6040_REG_NCPCTL,
	TWL6040_REG_LDOCTL,
	TWL6040_REG_AMICBCTL,
	TWL6040_REG_DMICBCTL,
	TWL6040_REG_HKCTL1,
	TWL6040_REG_HKCTL2,
	TWL6040_REG_GPOCTL,
	TWL6040_REG_TRIM1,
	TWL6040_REG_TRIM2,
	TWL6040_REG_TRIM3,
	TWL6040_REG_HSOTRIM,
	TWL6040_REG_HFOTRIM,
	TWL6040_REG_ACCCTL,
	TWL6040_REG_STATUS,
};

/*
 * twl6040 vdd/vss registers:
 * registers under vdd/vss supplies can only be accessed
 * after the power-up sequence
 */
static const int twl6040_vdd_reg[TWL6040_VDDREGNUM] = {
	TWL6040_REG_HPPLLCTL,
	TWL6040_REG_LPPLLCTL,
	TWL6040_REG_LPPLLDIV,
	TWL6040_REG_MICLCTL,
	TWL6040_REG_MICRCTL,
	TWL6040_REG_MICGAIN,
	TWL6040_REG_LINEGAIN,
	TWL6040_REG_HSLCTL,
	TWL6040_REG_HSRCTL,
	TWL6040_REG_HSGAIN,
	TWL6040_REG_EARCTL,
	TWL6040_REG_HFLCTL,
	TWL6040_REG_HFLGAIN,
	TWL6040_REG_HFRCTL,
	TWL6040_REG_HFRGAIN,
	TWL6040_REG_VIBCTLL,
	TWL6040_REG_VIBDATL,
	TWL6040_REG_VIBCTLR,
	TWL6040_REG_VIBDATR,
	TWL6040_REG_ALB,
	TWL6040_REG_DLB,
};

/*
 * Gain values for ABE PGA components
 */
static abe_gain_t abe_twl6040_gain_map[ABE_TWL6040_GAINNUM] = {
	MUTE_GAIN,
	GAIN_M50dB,
	GAIN_M40dB,
	GAIN_M30dB,
	GAIN_M24dB,
	GAIN_M18dB,
	GAIN_M12dB,
	GAIN_M6dB,
	GAIN_0dB,
	GAIN_6dB,
	GAIN_12dB,
	GAIN_18dB,
	GAIN_24dB,
	GAIN_MAXIMUM,
};

/*
 * read twl6040 register cache
 */
static inline unsigned int twl6040_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl6040 register cache
 */
static inline void twl6040_write_reg_cache(struct snd_soc_codec *codec,
						u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL6040_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * read from twl6040 hardware register
 */
static int twl6040_read_reg_volatile(struct snd_soc_codec *codec,
			unsigned int reg)
{
	u8 value = 0;

	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	if (likely(reg < TWL6040_REG_SHADOW)) {
		twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &value, reg);
		twl6040_write_reg_cache(codec, reg, value);
		return value;
	} else {
		return twl6040_read_reg_cache(codec, reg);
	}
}

/*
 * write to the twl6040 register space
 */
static int twl6040_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	if (reg >= TWL6040_CACHEREGNUM)
		return -EIO;

	twl6040_write_reg_cache(codec, reg, value);
	 if (likely(reg < TWL6040_REG_SHADOW))
		return twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, value, reg);
	else
		return 0;
}

static void twl6040_init_vio_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	/* allow registers to be accessed by i2c */
	twl6040_write(codec, TWL6040_REG_ACCCTL, cache[TWL6040_REG_ACCCTL]);

	for (i = 0; i < TWL6040_VIOREGNUM; i++) {
		reg = twl6040_vio_reg[i];
		/* skip read-only registers (ASICID, ASICREV, STATUS) */
		switch (reg) {
		case TWL6040_REG_ASICID:
		case TWL6040_REG_ASICREV:
		case TWL6040_REG_STATUS:
			continue;
		default:
			break;
		}
		twl6040_write(codec, reg, cache[reg]);
	}
}

static void twl6040_init_vdd_regs(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int reg, i;

	for (i = 0; i < TWL6040_VDDREGNUM; i++) {
		reg = twl6040_vdd_reg[i];
		twl6040_write(codec, reg, cache[reg]);
	}
}

static void abe_init_chip(struct snd_soc_codec *codec,
			struct platform_device *pdev)
{
	struct twl4030_codec_data *pdata = codec->dev->platform_data;
	abe_opp_t OPP = ABE_OPP100;
	abe_equ_t dl2_eq;
	const abe_int32 DL2_COEF[25] =	{
		-7554223, 708210, -708206, 7554225,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 6802833, -682266, 731554
	};
	dl2_eq.equ_length = 25;

	/* build the coefficient parameter for the equalizer api */
	memcpy(dl2_eq.coef.type1, DL2_COEF, sizeof(DL2_COEF));

	abe_init_mem();
	/* aess_clk has to be enabled to access hal register.
	 * Disabel the clk after it has been used.
	 */
	pm_runtime_get_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
	if (pdata->device_enable)
		pdata->device_enable(pdev);
#endif

	abe_reset_hal();
	abe_load_fw();
	/* Config OPP 100 for now */
	abe_set_opp_processing(OPP);
	/* "tick" of the audio engine */
	abe_write_event_generator(EVENT_TIMER);

	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_MM_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_MM_UL2);
	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_0MS, MIX_DL1_INPUT_VX_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_TONES);

	abe_write_mixer(MIXDL2, GAIN_M6dB, RAMP_0MS, MIX_DL2_INPUT_TONES);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_0MS, MIX_DL2_INPUT_VX_DL);
	abe_write_mixer(MIXDL2, GAIN_M6dB, RAMP_0MS, MIX_DL2_INPUT_MM_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_0MS, MIX_DL2_INPUT_MM_UL2);

	abe_write_mixer(MIXSDT, MUTE_GAIN, RAMP_0MS, MIX_SDT_INPUT_UP_MIXER);
	abe_write_mixer(MIXSDT, GAIN_M6dB, RAMP_0MS, MIX_SDT_INPUT_DL1_MIXER);

	abe_write_mixer(MIXECHO, MUTE_GAIN, RAMP_0MS, GAIN_LEFT_OFFSET);
	abe_write_mixer(MIXECHO, MUTE_GAIN, RAMP_0MS, GAIN_RIGHT_OFFSET);

	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_0MS, MIX_AUDUL_INPUT_TONES);
	abe_write_mixer(MIXAUDUL, GAIN_M6dB, RAMP_0MS, MIX_AUDUL_INPUT_UPLINK);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_0MS, MIX_AUDUL_INPUT_MM_DL);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_0MS, MIX_AUDUL_INPUT_VX_DL);

	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_0MS, MIX_VXREC_INPUT_TONES);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_0MS, MIX_VXREC_INPUT_VX_DL);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_0MS, MIX_VXREC_INPUT_MM_DL);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_0MS, MIX_VXREC_INPUT_VX_UL);

	/* write volumes */
	abe_write_gain(GAINS_DL1, GAIN_M6dB, RAMP_0MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL1, GAIN_M6dB, RAMP_0MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_6dB, RAMP_0MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_6dB, RAMP_0MS, GAIN_RIGHT_OFFSET);

	abe_write_gain(GAINS_AMIC, GAIN_12dB, RAMP_0MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_AMIC, GAIN_12dB, RAMP_0MS, GAIN_RIGHT_OFFSET);

	abe_write_gain(GAINS_SPLIT, GAIN_M6dB, RAMP_0MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_SPLIT, GAIN_M6dB, RAMP_0MS, GAIN_RIGHT_OFFSET);

	/* load the high-pass coefficient of IHF-Right */
	abe_write_equalizer(EQ2L, &dl2_eq);
	/* load the high-pass coefficient of IHF-Left */
	abe_write_equalizer(EQ2R, &dl2_eq);

	/* Vx in HS, MM in HF and Tones in HF */
	twl6040_write(codec, TWL6040_REG_SHADOW, 0x92);

	pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
	if (pdata->device_idle)
		pdata->device_idle(pdev);
#endif
}

/* twl6040 codec manual power-up sequence */
static void twl6040_power_up(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable reference system */
	ldoctl |= TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* enable high-side ldo */
	ldoctl |= TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable negative charge pump */
	ncpctl |= TWL6040_NCPENA | TWL6040_NCPOPEN;
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* enable low-side ldo */
	ldoctl |= TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* enable low-power pll */
	lppllctl |= TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* reset state machine */
	accctl |= TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	mdelay(5);
	accctl &= ~TWL6040_RESETSPLIT;
	twl6040_write(codec, TWL6040_REG_ACCCTL, accctl);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
}

/* twl6040 codec manual power-down sequence */
static void twl6040_power_down(struct snd_soc_codec *codec)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;

	ncpctl = twl6040_read_reg_cache(codec, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_read_reg_cache(codec, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_read_reg_cache(codec, TWL6040_REG_ACCCTL);

	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
	/* disable low-power pll */
	lppllctl &= ~TWL6040_LPLLENA;
	twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	/* disable low-side ldo */
	ldoctl &= ~TWL6040_LSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable negative charge pump */
	ncpctl &= ~(TWL6040_NCPENA | TWL6040_NCPOPEN);
	twl6040_write(codec, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
	/* disable high-side ldo */
	ldoctl &= ~TWL6040_HSLDOENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	/* disable reference system */
	ldoctl &= ~TWL6040_REFENA;
	twl6040_write(codec, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);
}

/* set headset dac and driver power mode */
static int headset_power_mode(struct snd_soc_codec *codec, int high_perf)
{
	int hslctl, hsrctl;
	int mask = TWL6040_HSDRVMODEL | TWL6040_HSDACMODEL;

	hslctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSLCTL);
	hsrctl = twl6040_read_reg_cache(codec, TWL6040_REG_HSRCTL);

	if (high_perf) {
		hslctl &= ~mask;
		hsrctl &= ~mask;
	} else {
		hslctl |= mask;
		hsrctl |= mask;
	}

	twl6040_write(codec, TWL6040_REG_HSLCTL, hslctl);
	twl6040_write(codec, TWL6040_REG_HSRCTL, hsrctl);

	return 0;
}

static int twl6040_hs_power_event(struct snd_soc_dapm_widget *w,
                        struct snd_kcontrol *kcontrol, int event)
{
	msleep(1);
        return 0;
}

static int twl6040_power_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct twl6040_data *priv = codec->private_data;

	if (SND_SOC_DAPM_EVENT_ON(event))
		priv->non_lp++;
	else
		priv->non_lp--;

	msleep(1);

	return 0;
}

static void twl6040_hs_jack_detect_work(struct work_struct *work)
{
	struct twl6040_jack_data *jack;
	int state, status;

	jack = container_of(work, struct twl6040_jack_data, work);
	state = jack->state;

	/*
	 * Early interrupt, CODEC driver cannot report jack status
	 * since jack is not registered yet. MACHINE driver will
	 * register jack and report status through twl6040_hs_jack_detect
	 */
	if (jack->jack) {
		status = state ? jack->report : 0;
		snd_soc_jack_report(jack->jack, status, jack->report);
	}

	switch_set_state(&jack->sdev, state);
}

/* audio interrupt handler */
static irqreturn_t twl6040_naudint_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct twl6040_data *priv = codec->private_data;
	struct twl6040_jack_data *jack = &priv->hs_jack;
	u8 intid = 0;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid, TWL6040_REG_INTID);

	if (intid & TWL6040_THINT)
		dev_alert(codec->dev, "die temp over-limit detection\n");

	if (intid & TWL6040_PLUGINT) {
		/* Debounce */
		msleep(200);
		jack->state = 1;
		schedule_work(&jack->work);
	}

	if (intid & TWL6040_UNPLUGINT) {
		jack->state = 0;
		schedule_work(&jack->work);
	}

	if (intid & TWL6040_HFINT)
		dev_alert(codec->dev, "hf drivers over current detection\n");

	if (intid & TWL6040_VIBINT)
		dev_alert(codec->dev, "vib drivers over current detection\n");

	if (intid & TWL6040_READYINT)
		complete(&priv->ready);

	return IRQ_HANDLED;
}

static int snd_soc_put_volsw_amic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (val > max)
		val = max;
	if (invert)
		val = max - val;
	/* write to actual ABE HAL register for left channel */
	abe_write_gain(GAINS_AMIC, abe_twl6040_gain_map[val],
		       RAMP_0MS, GAIN_LEFT_OFFSET);
	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (val2 > max)
			val2 = max;
		if (invert)
			val2 = max - val2;
		/* write to actual ABE HAL register for right channel */
		abe_write_gain(GAINS_AMIC, abe_twl6040_gain_map[val2],
			       RAMP_0MS, GAIN_RIGHT_OFFSET);
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}

	return snd_soc_update_bits(codec, reg, val_mask, val);
}

static int snd_soc_put_dl1_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	unsigned int shift = mc->shift;
	int mask;
	int err;
	unsigned short val;
	char *name = kcontrol->id.name;

	mask = 1 << shift;
	val = (ucontrol->value.integer.value[0] << shift);

	if (strcmp(name, "DL1 Mixer Tones") == 0) {
		if (val)
			abe_write_mixer(MIXDL1, GAIN_M6dB,
					RAMP_0MS, MIX_DL1_INPUT_TONES);
		else
			abe_write_mixer(MIXDL1, MUTE_GAIN,
					RAMP_0MS, MIX_DL1_INPUT_TONES);
	} else if (strcmp(name, "DL1 Mixer Voice") == 0) {
		if (val)
			abe_write_mixer(MIXDL1, GAIN_M6dB,
					RAMP_1MS, MIX_DL1_INPUT_VX_DL);
		else
			abe_write_mixer(MIXDL1, MUTE_GAIN,
					RAMP_0MS, MIX_DL1_INPUT_VX_DL);
	} else if (strcmp(name, "DL1 Mixer Multimedia") == 0) {
		if (val)
			abe_write_mixer(MIXDL1, GAIN_M6dB,
					RAMP_2MS, MIX_DL1_INPUT_MM_DL);
		else
			abe_write_mixer(MIXDL1, MUTE_GAIN,
					RAMP_0MS, MIX_DL1_INPUT_MM_DL);
	} else if (strcmp(name, "DL1 Mixer Multimedia Uplink") == 0) {
		if (val)
			abe_write_mixer(MIXDL1, GAIN_M6dB,
					RAMP_5MS, MIX_DL1_INPUT_MM_UL2);
		else
			abe_write_mixer(MIXDL1, MUTE_GAIN,
					RAMP_0MS, MIX_DL1_INPUT_MM_UL2);
	}

	err = snd_soc_update_bits(widget->codec, TWL6040_REG_SHADOW, mask, val);

	return err;
}

static int snd_soc_put_dl2_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	unsigned int shift = mc->shift;
	int mask;
	int err;
	unsigned short val;
	char *name = kcontrol->id.name;

	mask = 1 << shift;
	val = (ucontrol->value.integer.value[0] << shift);

	if (strcmp(name, "DL2 Mixer Tones") == 0) {
		if (val)
			abe_write_mixer(MIXDL2, GAIN_M6dB,
				RAMP_0MS, MIX_DL2_INPUT_TONES);
		else
			abe_write_mixer(MIXDL2, MUTE_GAIN,
				RAMP_0MS, MIX_DL2_INPUT_TONES);
	} else if (strcmp(name, "DL2 Mixer Voice") == 0) {
		if (val)
			abe_write_mixer(MIXDL2, GAIN_M6dB,
				RAMP_1MS, MIX_DL2_INPUT_VX_DL);
		else
			abe_write_mixer(MIXDL2, MUTE_GAIN,
				RAMP_0MS, MIX_DL2_INPUT_VX_DL);
	} else if (strcmp(name, "DL2 Mixer Multimedia") == 0) {
		if (val)
			abe_write_mixer(MIXDL2, GAIN_M6dB,
				RAMP_2MS, MIX_DL2_INPUT_MM_DL);
		else
			abe_write_mixer(MIXDL2, MUTE_GAIN,
				RAMP_0MS, MIX_DL2_INPUT_MM_DL);
	} else if (strcmp(name, "DL2 Mixer Multimedia Uplink") == 0) {
		if (val)
			abe_write_mixer(MIXDL2, GAIN_M6dB,
				RAMP_5MS, MIX_DL2_INPUT_MM_UL2);
		else
			abe_write_mixer(MIXDL2, MUTE_GAIN,
				RAMP_0MS, MIX_DL2_INPUT_MM_UL2);
	}

	err = snd_soc_update_bits(widget->codec, TWL6040_REG_SHADOW, mask, val);

	return err;
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
 * (ABE HAL) GAINS_AMIC volume control:
 * -42 dB to 30 dB in 6 dB steps (mute instead of -48)
 */
static DECLARE_TLV_DB_SCALE(mic_dig_tlv, -4800, 600, 1);

/*
 * AFMGAIN volume control:
 * from 18 to 24 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(afm_amp_tlv, 600, 600, 0);

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

/*
 * EPGAIN volume control:
 * from -24 to 6 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(ep_tlv, -2400, 200, 0);

/* Left analog microphone selection */
static const char *twl6040_amicl_texts[] = {
	"Headset Mic", "Main Mic", "Aux/FM Left", "Off"};

/* Right analog microphone selection */
static const char *twl6040_amicr_texts[] = {
	"Headset Mic", "Sub Mic", "Aux/FM Right", "Off"};

static const char *twl6040_hs_texts[] =
	{"Off", "HS DAC", "Line-In amp"};

static const char *twl6040_hf_texts[] =
	{"Off", "HF DAC", "Line-In amp"};

static const struct soc_enum twl6040_enum[] = {
	SOC_ENUM_SINGLE(TWL6040_REG_MICLCTL, 3, 4, twl6040_amicl_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_MICRCTL, 3, 4, twl6040_amicr_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HSLCTL, 5, 3, twl6040_hs_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HSRCTL, 5, 3, twl6040_hs_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HFLCTL, 2, 3, twl6040_hf_texts),
	SOC_ENUM_SINGLE(TWL6040_REG_HFRCTL, 2, 3, twl6040_hf_texts),
};

static const struct snd_kcontrol_new amicl_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[0]);

static const struct snd_kcontrol_new amicr_control =
	SOC_DAPM_ENUM("Route", twl6040_enum[1]);

static const struct snd_kcontrol_new dl1_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones", TWL6040_REG_SHADOW, 0, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl1_mixer),
	SOC_SINGLE_EXT("Voice", TWL6040_REG_SHADOW, 1, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl1_mixer),
	SOC_SINGLE_EXT("Multimedia Uplink", TWL6040_REG_SHADOW, 2, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl1_mixer),
	SOC_SINGLE_EXT("Multimedia", TWL6040_REG_SHADOW, 3, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl1_mixer),
};

static const struct snd_kcontrol_new dl2_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones", TWL6040_REG_SHADOW, 4, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl2_mixer),
	SOC_SINGLE_EXT("Voice", TWL6040_REG_SHADOW, 5, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl2_mixer),
	SOC_SINGLE_EXT("Multimedia Uplink", TWL6040_REG_SHADOW, 6, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl2_mixer),
	SOC_SINGLE_EXT("Multimedia", TWL6040_REG_SHADOW, 7, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_dl2_mixer),
};


/* Headset DAC playback switches */
static const struct snd_kcontrol_new hsl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[2]);

static const struct snd_kcontrol_new hsr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[3]);

/* Handsfree DAC playback switches */
static const struct snd_kcontrol_new hfl_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[4]);

static const struct snd_kcontrol_new hfr_mux_controls =
	SOC_DAPM_ENUM("Route", twl6040_enum[5]);

static const struct snd_kcontrol_new ep_driver_switch_controls =
	SOC_DAPM_SINGLE("Switch", TWL6040_REG_EARCTL, 0, 1, 0);

static const struct snd_kcontrol_new twl6040_snd_controls[] = {
	/* Capture gains */
	SOC_DOUBLE_TLV("Capture Preamplifier Volume",
		TWL6040_REG_MICGAIN, 6, 7, 1, 1, mic_preamp_tlv),
	SOC_DOUBLE_TLV("Capture Volume",
		TWL6040_REG_MICGAIN, 0, 3, 4, 0, mic_amp_tlv),
	SOC_DOUBLE_EXT_TLV("Capture Digital Volume",
		ABE_AMIC_GAIN_SHADOW, 0, 4, ABE_TWL6040_GAINNUM - 1, 0,
		snd_soc_get_volsw, snd_soc_put_volsw_amic, mic_dig_tlv),

	/* AFM gains */
	SOC_DOUBLE_TLV("Aux FM Volume",
		TWL6040_REG_LINEGAIN, 0, 5, 0xF, 0, afm_amp_tlv),

	/* Playback gains */
	SOC_DOUBLE_TLV("Headset Playback Volume",
		TWL6040_REG_HSGAIN, 0, 4, 0xF, 1, hs_tlv),
	SOC_DOUBLE_R_TLV("Handsfree Playback Volume",
		TWL6040_REG_HFLGAIN, TWL6040_REG_HFRGAIN, 0, 0x1D, 1, hf_tlv),
	SOC_SINGLE_TLV("Earphone Playback Volume",
		TWL6040_REG_EARCTL, 1, 0xF, 1, ep_tlv),
};

static const struct snd_soc_dapm_widget twl6040_dapm_widgets[] = {
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
	SND_SOC_DAPM_OUTPUT("EP"),

	/* Analog input muxes for the capture amplifiers */
	SND_SOC_DAPM_MUX("Analog Left Capture Route",
			SND_SOC_NOPM, 0, 0, &amicl_control),
	SND_SOC_DAPM_MUX("Analog Right Capture Route",
			SND_SOC_NOPM, 0, 0, &amicr_control),

	SND_SOC_DAPM_MIXER("DL1 Mixer",
			SND_SOC_NOPM, 0, 0, dl1_mixer_controls,
			ARRAY_SIZE(dl1_mixer_controls)),
	SND_SOC_DAPM_MIXER("DL2 Mixer",
			SND_SOC_NOPM, 0, 0, dl2_mixer_controls,
			ARRAY_SIZE(dl2_mixer_controls)),

	/* Analog capture PGAs */
	SND_SOC_DAPM_PGA("MicAmpL",
			TWL6040_REG_MICLCTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MicAmpR",
			TWL6040_REG_MICRCTL, 0, 0, NULL, 0),

	/* Auxiliary FM PGAs */
	SND_SOC_DAPM_PGA("AFMAmpL",
			TWL6040_REG_MICLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AFMAmpR",
			TWL6040_REG_MICRCTL, 1, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("ADC Left", "Left Front Capture",
			TWL6040_REG_MICLCTL, 2, 0),
	SND_SOC_DAPM_ADC("ADC Right", "Right Front Capture",
			TWL6040_REG_MICRCTL, 2, 0),

	/* Microphone bias */
	SND_SOC_DAPM_MICBIAS("Headset Mic Bias",
			TWL6040_REG_AMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Main Mic Bias",
			TWL6040_REG_AMICBCTL, 4, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic1 Bias",
			TWL6040_REG_DMICBCTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Digital Mic2 Bias",
			TWL6040_REG_DMICBCTL, 4, 0),

	SND_SOC_DAPM_AIF_IN("AIFIN Tones", "Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN Voice", "Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN Multimedia Uplink", "Playback", 0,
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN Multimedia", "Playback", 0,
			SND_SOC_NOPM, 0, 0),

	/* DACs */
	SND_SOC_DAPM_DAC_E("HSDAC Left", "Headset Playback",
			TWL6040_REG_HSLCTL, 0, 0,
			twl6040_hs_power_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HSDAC Right", "Headset Playback",
			TWL6040_REG_HSRCTL, 0, 0,
                        twl6040_hs_power_event,
                        SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Left", "Handsfree Playback",
			TWL6040_REG_HFLCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("HFDAC Right", "Handsfree Playback",
			TWL6040_REG_HFRCTL, 0, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("HF Left Playback",
			SND_SOC_NOPM, 0, 0, &hfl_mux_controls),
	SND_SOC_DAPM_MUX("HF Right Playback",
			SND_SOC_NOPM, 0, 0, &hfr_mux_controls),
	/* Analog playback Muxes */
	SND_SOC_DAPM_MUX("HS Left Playback",
			SND_SOC_NOPM, 0, 0, &hsl_mux_controls),
	SND_SOC_DAPM_MUX("HS Right Playback",
			SND_SOC_NOPM, 0, 0, &hsr_mux_controls),

	/* Analog playback drivers */
	SND_SOC_DAPM_PGA_E("Handsfree Left Driver",
			TWL6040_REG_HFLCTL, 4, 0, NULL, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("Handsfree Right Driver",
			TWL6040_REG_HFRCTL, 4, 0, NULL, 0,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA("Headset Left Driver",
			TWL6040_REG_HSLCTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Headset Right Driver",
			TWL6040_REG_HSRCTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH_E("Earphone Driver",
			SND_SOC_NOPM, 0, 0, &ep_driver_switch_controls,
			twl6040_power_mode_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Analog playback PGAs */
	SND_SOC_DAPM_PGA("HFDAC Left PGA",
			TWL6040_REG_HFLCTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HFDAC Right PGA",
			TWL6040_REG_HFRCTL, 1, 0, NULL, 0),

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

	/* AFM path */
	{"AFMAmpL", "NULL", "AFML"},
	{"AFMAmpR", "NULL", "AFMR"},

	/* Headset playback path */
	{"DL1 Mixer", "Tones", "AIFIN Tones"},
	{"DL1 Mixer", "Voice", "AIFIN Voice"},
	{"DL1 Mixer", "Multimedia Uplink", "AIFIN Multimedia Uplink"},
	{"DL1 Mixer", "Multimedia", "AIFIN Multimedia"},

	{"HSDAC Left", NULL, "DL1 Mixer"},
	{"HSDAC Right", NULL, "DL1 Mixer"},

	{"HS Left Playback", "HS DAC", "HSDAC Left"},
	{"HS Left Playback", "Line-In amp", "AFMAmpL"},

	{"HS Right Playback", "HS DAC", "HSDAC Right"},
	{"HS Right Playback", "Line-In amp", "AFMAmpR"},

	{"Headset Left Driver", "NULL", "HS Left Playback"},
	{"Headset Right Driver", "NULL", "HS Right Playback"},

	{"HSOL", NULL, "Headset Left Driver"},
	{"HSOR", NULL, "Headset Right Driver"},

	/* Earphone playback path */
	{"Earphone Driver", "Switch", "HSDAC Left"},
	{"EP", NULL, "Earphone Driver"},

	/* Handsfree playback path */
	{"DL2 Mixer", "Tones", "AIFIN Tones"},
	{"DL2 Mixer", "Voice", "AIFIN Voice"},
	{"DL2 Mixer", "Multimedia Uplink", "AIFIN Multimedia Uplink"},
	{"DL2 Mixer", "Multimedia", "AIFIN Multimedia"},

	{"HFDAC Left", NULL, "DL2 Mixer"},
	{"HFDAC Right", NULL, "DL2 Mixer"},

	{"HF Left Playback", "HF DAC", "HFDAC Left"},
	{"HF Left Playback", "Line-In amp", "AFMAmpL"},

	{"HF Right Playback", "HF DAC", "HFDAC Right"},
	{"HF Right Playback", "Line-In amp", "AFMAmpR"},

	{"HFDAC Left PGA", NULL, "HF Left Playback"},
	{"HFDAC Right PGA", NULL, "HF Right Playback"},

	{"Handsfree Left Driver", "Switch", "HFDAC Left PGA"},
	{"Handsfree Right Driver", "Switch", "HFDAC Right PGA"},

	{"HFL", NULL, "Handsfree Left Driver"},
	{"HFR", NULL, "Handsfree Right Driver"},
};

static int abe_twl6040_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, twl6040_dapm_widgets,
				 ARRAY_SIZE(twl6040_dapm_widgets));

	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);

	return 0;
}

static int twl6040_power_up_completion(struct snd_soc_codec *codec,
					int naudint)
{
	struct twl6040_data *priv = codec->private_data;
	int time_left;
	u8 intid = 0;

	time_left = wait_for_completion_timeout(&priv->ready,
				msecs_to_jiffies(48));

	if (!time_left) {
		twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &intid,
							TWL6040_REG_INTID);
		if (!(intid & TWL6040_READYINT)) {
			dev_err(codec->dev, "timeout waiting for READYINT\n");
			return -ETIMEDOUT;
		}
	}

	priv->codec_powered = 1;

	return 0;
}

static int abe_twl6040_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct twl6040_data *priv = codec->private_data;
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (priv->codec_powered)
			break;

		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 1);

			/* wait for power-up completion */
			ret = twl6040_power_up_completion(codec, naudint);
			if (ret)
				return ret;

			/* sync registers updated during power-up sequence */
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LPPLLCTL);
		} else {
			/* use manual power-up sequence */
			twl6040_power_up(codec);
			priv->codec_powered = 1;
		}

		/* initialize vdd/vss registers with reg_cache */
		twl6040_init_vdd_regs(codec);
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
			twl6040_read_reg_volatile(codec, TWL6040_REG_NCPCTL);
			twl6040_read_reg_volatile(codec, TWL6040_REG_LDOCTL);
			twl6040_write_reg_cache(codec, TWL6040_REG_LPPLLCTL,
						0x00);
		} else {
			/* use manual power-down sequence */
			twl6040_power_down(codec);
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

static int twl6040_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl6040_data *priv = codec->private_data;
	u8 hppllctl, lppllctl;

	hppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_HPPLLCTL);
	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	switch (clk_id) {
	case TWL6040_SYSCLK_SEL_LPPLL:
		switch (freq) {
		case 32768:
			/* headset dac and driver must be in low-power mode */
			headset_power_mode(codec, 0);

			/* clk32k input requires low-power pll */
			lppllctl |= TWL6040_LPLLENA;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			mdelay(5);
			lppllctl &= ~TWL6040_HPLLSEL;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
			hppllctl &= ~TWL6040_HPLLENA;
			twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* lppll divider */
		switch (priv->sysclk) {
		case 17640000:
			lppllctl |= TWL6040_LPLLFIN;
			break;
		case 19200000:
			lppllctl &= ~TWL6040_LPLLFIN;
			break;
		default:
			/* sysclk not yet configured */
			lppllctl &= ~TWL6040_LPLLFIN;
			priv->sysclk = 19200000;
			break;
		}

		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		priv->pll = TWL6040_LPPLL_ID;
		priv->sysclk_constraints = &lp_constraints;
		break;
	case TWL6040_SYSCLK_SEL_HPPLL:
		hppllctl &= ~TWL6040_MCLK_MSK;

		switch (freq) {
		case 12000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_12000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 19200000:
			/* mclk input, pll disabled */
			hppllctl |= TWL6040_MCLK_19200KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		case 26000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_26000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 38400000:
			/* clk slicer, pll disabled */
			hppllctl |= TWL6040_MCLK_38400KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		default:
			dev_err(codec->dev, "unknown mclk freq %d\n", freq);
			return -EINVAL;
		}

		/* headset dac and driver must be in high-performance mode */
		headset_power_mode(codec, 1);

		twl6040_write(codec, TWL6040_REG_HPPLLCTL, hppllctl);
		udelay(500);
		lppllctl |= TWL6040_HPLLSEL;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
		lppllctl &= ~TWL6040_LPLLENA;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);

		/* high-performance pll can provide only 19.2 MHz */
		priv->pll = TWL6040_HPPLL_ID;
		priv->sysclk = 19200000;
		priv->sysclk_constraints = &hp_constraints;
		break;
	default:
		dev_err(codec->dev, "unknown clk_id %d\n", clk_id);
		return -EINVAL;
	}

	return 0;
}

static inline void abe_mcpdm_dl_hw_params(struct twl6040_data *priv)
{
	if (!priv->mcpdm_dl_enable++)
		abe_enable_data_transfer(PDM_DL_PORT);
}

static inline void abe_mcpdm_ul_hw_params(struct twl6040_data *priv)
{
	if (!priv->mcpdm_ul_enable++) {
		/*
		 * Check if downlink is not running to move
		 * to McPDM Uplink port for drift management
		 */
		if (!priv->mcpdm_dl_enable)
			abe_select_main_port(PDM_UL_PORT);
		abe_enable_data_transfer(PDM_UL_PORT);
	}
}

static inline void abe_mcpdm_dl_shutdown(struct twl6040_data *priv)
{
	if (!--priv->mcpdm_dl_enable) {
		abe_disable_data_transfer(PDM_DL_PORT);
		/*
		 * Check if uplink is running to move
		 * to McPDM uplink port for drift management
		 */
		if (priv->mcpdm_ul_enable)
			abe_select_main_port(PDM_UL_PORT);
		else
			abe_disable_data_transfer(PDM_UL_PORT);
	}
}

static inline void abe_mcpdm_ul_shutdown(struct twl6040_data *priv)
{
	if (!--priv->mcpdm_ul_enable) {
		/* Disable ATC only if no DL in parallel (HW issue) */
		if (!priv->mcpdm_dl_enable)
			abe_disable_data_transfer(PDM_UL_PORT);

		/* Always move back to McPDM downlink after playback */
		abe_select_main_port(PDM_DL_PORT);
	}
}

static int abe_mm_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	struct twl4030_codec_data *pdata = codec->dev->platform_data;
	struct platform_device *pdev = to_platform_device(codec->dev);

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
		pm_runtime_get_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
		if (pdata->device_enable)
			pdata->device_enable(pdev);
#endif

		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_AMIC,
			(abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_AMIC]);
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
	struct twl6040_data *priv = codec->private_data;
	u8 lppllctl;
	int rate;
	int channels;
	unsigned int sysclk;
	abe_data_format_t format;
	abe_dma_t dma_sink;

	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	rate = params_rate(params);
	switch (rate) {
	case 44100:
		lppllctl |= TWL6040_LPLLFIN;
		sysclk = 17640000;
		break;
	case 48000:
	/* Select output frequency 19.2 MHz */
		lppllctl &= ~TWL6040_LPLLFIN;
		sysclk = 19200000;
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	channels = params_channels(params);
	switch (channels) {
	case 1:
		format.samp_format = MONO_MSB;
		break;
	case 2:
		format.samp_format = STEREO_MSB;
		break;
	default:
		dev_err(codec->dev, "%d channels not supported", channels);
		return -EINVAL;
	}

	if (priv->pll == TWL6040_LPPLL_ID) {
		priv->sysclk = sysclk;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	}

	format.f = rate;
	if (!substream->stream) {
		abe_connect_cbpr_dmareq_port(MM_DL_PORT, &format,
						ABE_CBPR0_IDX, &dma_sink);
		abe_enable_data_transfer(MM_DL_PORT);
		abe_mcpdm_dl_hw_params(priv);
	} else {
		/*  Route UL2 port for MM-UL audio capture */
		abe_connect_cbpr_dmareq_port(MM_UL2_PORT, &format,
						ABE_CBPR4_IDX, &dma_sink);
		abe_enable_data_transfer(MM_UL2_PORT);
		abe_mcpdm_ul_hw_params(priv);
	}

	return 0;
}

static int abe_mm_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	unsigned int snd_reg_shadow;

	snd_reg_shadow = twl6040_read_reg_cache(codec, TWL6040_REG_SHADOW);

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
		if (!substream->stream){
			if ((snd_reg_shadow & 0x08) == 0x08)
				abe_write_mixer(MIXDL1, GAIN_M6dB,
						RAMP_0MS, MIX_DL1_INPUT_MM_DL);

			if ((snd_reg_shadow & 0x80) == 0x80)
				abe_write_mixer(MIXDL2, GAIN_M6dB,
						RAMP_0MS, MIX_DL2_INPUT_MM_DL);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!substream->stream){
			if ((snd_reg_shadow & 0x08) == 0x08)
				abe_write_mixer(MIXDL1, MUTE_GAIN,
						RAMP_0MS, MIX_DL1_INPUT_MM_DL);

			if ((snd_reg_shadow & 0x80) == 0x80)
				abe_write_mixer(MIXDL2, MUTE_GAIN,
						RAMP_0MS, MIX_DL2_INPUT_MM_DL);
		}
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
	struct twl6040_data *priv = codec->private_data;
	struct twl4030_codec_data *pdata = codec->dev->platform_data;
	struct platform_device *pdev = to_platform_device(codec->dev);

        if (!substream->stream) {
                abe_disable_data_transfer(MM_DL_PORT);
		abe_mcpdm_dl_shutdown(priv);
        } else {
                abe_disable_data_transfer(MM_UL2_PORT);
		abe_mcpdm_ul_shutdown(priv);
        }

	if (!--priv->configure) {
		pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
		if (pdata->device_idle)
			pdata->device_idle(pdev);
#endif
	}
}

static int twl6040_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct twl6040_data *priv = codec->private_data;

	int hs_gain, hf_gain;

	hf_gain = twl6040_read_reg_cache(codec, TWL6040_REG_HFRGAIN);
	hs_gain = twl6040_read_reg_cache(codec, TWL6040_REG_HSGAIN);

	if (mute) {
		if (priv->mcpdm_dl_enable == 1) {
			twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, 0x1D, TWL6040_REG_HFRGAIN);
			twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, 0x1D, TWL6040_REG_HFLGAIN);
			twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, 0xFF, TWL6040_REG_HSGAIN);
		}
	} else {
		twl6040_write(codec, TWL6040_REG_HFRGAIN, hf_gain);
		twl6040_write(codec, TWL6040_REG_HFLGAIN, hf_gain);
		twl6040_write(codec, TWL6040_REG_HSGAIN, hs_gain);
	}

	return 0;
}

static struct snd_soc_dai_ops abe_mm_dai_ops = {
	.startup	= abe_mm_startup,
	.hw_params	= abe_mm_hw_params,
	.digital_mute	= twl6040_mute,
	.shutdown	= abe_mm_shutdown,
	.trigger	= abe_mm_trigger,
	.set_sysclk	= twl6040_set_dai_sysclk,
};

static int abe_tones_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	u8 lppllctl;
	int rate;
	int channels;
	unsigned int sysclk;
	abe_data_format_t format;
	abe_dma_t dma_sink;

	lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);

	rate = params_rate(params);
	switch (rate) {
	case 44100:
		lppllctl |= TWL6040_LPLLFIN;
		sysclk = 17640000;
		break;
	case 48000:
	/* Select output frequency 19.2 MHz */
		lppllctl &= ~TWL6040_LPLLFIN;
		sysclk = 19200000;
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	channels = params_channels(params);
	switch (channels) {
	case 1:
		format.samp_format = MONO_MSB;
		break;
	case 2:
		format.samp_format = STEREO_MSB;
		break;
	default:
		dev_err(codec->dev, "%d channels not supported", channels);
		return -EINVAL;
	}

	if (priv->pll == TWL6040_LPPLL_ID) {
		priv->sysclk = sysclk;
		twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
	}

	format.f = rate;
	if (!substream->stream) {
		abe_connect_cbpr_dmareq_port(TONES_DL_PORT, &format,
							ABE_CBPR5_IDX, &dma_sink);
		abe_enable_data_transfer(TONES_DL_PORT);
		abe_mcpdm_dl_hw_params(priv);
	}

	return 0;
}

static int abe_tones_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	unsigned int snd_reg_shadow;

	snd_reg_shadow = twl6040_read_reg_cache(codec, TWL6040_REG_SHADOW);

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
		if (!substream->stream){
			if ((snd_reg_shadow & 0x01) == 0x01)
				abe_write_mixer(MIXDL1, GAIN_M6dB,
						RAMP_0MS, MIX_DL1_INPUT_TONES);

			if ((snd_reg_shadow & 0x10) == 0x10)
				abe_write_mixer(MIXDL2, GAIN_M6dB,
						RAMP_0MS, MIX_DL2_INPUT_TONES);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!substream->stream){
			if ((snd_reg_shadow & 0x01) == 0x01)
				abe_write_mixer(MIXDL1, MUTE_GAIN,
						RAMP_0MS, MIX_DL1_INPUT_TONES);

			if ((snd_reg_shadow & 0x10) == 0x10)
				abe_write_mixer(MIXDL2, MUTE_GAIN,
						RAMP_0MS, MIX_DL2_INPUT_TONES);
		}
		break;
	default:
		break;
	}

	return 0;
}

static void abe_tones_shutdown(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_device *socdev = rtd->socdev;
        struct snd_soc_codec *codec = socdev->card->codec;
        struct twl6040_data *priv = codec->private_data;
        struct twl4030_codec_data *pdata = codec->dev->platform_data;
	struct platform_device *pdev = to_platform_device(codec->dev);

        if (!substream->stream) {
                abe_disable_data_transfer(TONES_DL_PORT);
		abe_mcpdm_dl_shutdown(priv);
        }

	if (!--priv->configure) {
		pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
		if (pdata->device_idle)
			pdata->device_idle(pdev);
#endif
	}
}

static struct snd_soc_dai_ops abe_tones_dai_ops = {
	.startup	= abe_mm_startup,
	.hw_params	= abe_tones_hw_params,
	.digital_mute	= twl6040_mute,
	.trigger	= abe_tones_trigger,
	.shutdown	= abe_tones_shutdown,
	.set_sysclk	= twl6040_set_dai_sysclk,
};

static int abe_voice_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	struct twl4030_codec_data *pdata = codec->dev->platform_data;
	struct platform_device *pdev = to_platform_device(codec->dev);

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
		pm_runtime_get_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
		if (pdata->device_enable)
			pdata->device_enable(pdev);
#endif

		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_AMIC,
			(abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_AMIC]);
	}
	return 0;
}

static int abe_voice_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	u8 lppllctl;
	int rate;
	int channels;
	abe_data_format_t format;
#ifndef CONFIG_SND_OMAP_VOICE_TEST
	abe_dma_t dma_sink;
#endif

	rate = params_rate(params);
	switch (rate) {
	case 16000:
	case 8000:
	/* Select output frequency 19.2 MHz */
		if (priv->pll == TWL6040_LPPLL_ID) {
			lppllctl = twl6040_read_reg_cache(codec, TWL6040_REG_LPPLLCTL);
			lppllctl &= ~TWL6040_LPLLFIN;
			priv->sysclk = 19200000;
			twl6040_write(codec, TWL6040_REG_LPPLLCTL, lppllctl);
		}
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n", rate);
		return -EINVAL;
	}

	channels = params_channels(params);
	switch (channels) {
	case 1:
		format.samp_format = MONO_MSB;
		break;
	case 2:
		format.samp_format = STEREO_MSB;
		break;
	default:
		dev_err(codec->dev, "%d channels not supported", channels);
		return -EINVAL;
	}

	format.f = rate;
#ifdef CONFIG_SND_OMAP_VOICE_TEST
	if (!substream->stream) {
		/* Vx_DL connection to McBSP 2 ports */
		format.f = 8000;
		format.samp_format = STEREO_RSHIFTED_16;
		abe_connect_serial_port(VX_DL_PORT, &format, MCBSP2_RX);
		/* Enable downlink port */
		abe_enable_data_transfer(VX_DL_PORT);
		abe_mcpdm_dl_hw_params(priv);
	} else {
		/* Vx_UL connection to McBSP 2 ports */
		format.f = 8000;
		format.samp_format = STEREO_RSHIFTED_16;
		abe_connect_serial_port(VX_UL_PORT, &format, MCBSP2_TX);
		/* Enable uplink port */
		abe_enable_data_transfer(VX_UL_PORT);
		abe_mcpdm_ul_hw_params(priv);
	}
#else
	if (!substream->stream) {
		abe_connect_cbpr_dmareq_port(VX_DL_PORT, &format, ABE_CBPR1_IDX, &dma_sink);
		abe_enable_data_transfer(VX_DL_PORT);
		abe_mcpdm_dl_hw_params(priv);
	} else {
		abe_connect_cbpr_dmareq_port(VX_UL_PORT, &format, ABE_CBPR2_IDX, &dma_sink);
		abe_enable_data_transfer(VX_UL_PORT);
		abe_mcpdm_ul_hw_params(priv);
	}
#endif

	return 0;
}

static int abe_voice_trigger(struct snd_pcm_substream *substream,
					int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct twl6040_data *priv = codec->private_data;
	unsigned int snd_reg_shadow;

	snd_reg_shadow = twl6040_read_reg_cache(codec, TWL6040_REG_SHADOW);

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
		if (!substream->stream){
			if ((snd_reg_shadow & 0x02) == 0x02)
				abe_write_mixer(MIXDL1, GAIN_M6dB,
						RAMP_0MS, MIX_DL1_INPUT_VX_DL);

			if ((snd_reg_shadow & 0x20) == 0x20)
				abe_write_mixer(MIXDL2, GAIN_M6dB,
						RAMP_0MS, MIX_DL2_INPUT_VX_DL);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!substream->stream){
			if ((snd_reg_shadow & 0x02) == 0x02)
				abe_write_mixer(MIXDL1, MUTE_GAIN,
						RAMP_0MS, MIX_DL1_INPUT_VX_DL);

			if ((snd_reg_shadow & 0x20) == 0x20)
				abe_write_mixer(MIXDL2, MUTE_GAIN,
						RAMP_0MS, MIX_DL2_INPUT_VX_DL);
		}
		break;
	default:
		break;
	}

	return 0;
}

static void abe_voice_shutdown(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_device *socdev = rtd->socdev;
        struct snd_soc_codec *codec = socdev->card->codec;
        struct twl6040_data *priv = codec->private_data;
        struct twl4030_codec_data *pdata = codec->dev->platform_data;
	struct platform_device *pdev = to_platform_device(codec->dev);

        if (!substream->stream) {
                abe_disable_data_transfer(VX_DL_PORT);
		abe_mcpdm_dl_shutdown(priv);
        } else {
		abe_disable_data_transfer(VX_UL_PORT);
		abe_mcpdm_ul_shutdown(priv);
	}

        if(!--priv->configure) {
		pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
		if (pdata->device_idle)
			pdata->device_idle(pdev);
#endif
	}
}

static struct snd_soc_dai_ops abe_voice_dai_ops = {
	.startup	= abe_voice_startup,
	.hw_params	= abe_voice_hw_params,
	.digital_mute	= twl6040_mute,
	.shutdown	= abe_voice_shutdown,
	.trigger	= abe_voice_trigger,
	.set_sysclk	= twl6040_set_dai_sysclk,
};

/* Audio Backend DAIs */
struct snd_soc_dai abe_dai[] = {
	/* Multimedia: MM-UL2, MM-DL */
	{
		.name = "Multimedia",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
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
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = ABE_FORMATS,
		},
		.ops = &abe_tones_dai_ops,
	},
	/* Voice: VX-UL, VX-DL */
	{
		.name = "Voice",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = ABE_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
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
			.channels_max = 10,
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

#ifdef CONFIG_PM
static int abe_twl6040_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int abe_twl6040_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	abe_twl6040_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}
#else
#define abe_twl6040_suspend NULL
#define abe_twl6040_resume NULL
#endif

void twl6040_hs_jack_detect(struct snd_soc_codec *codec,
			    struct snd_soc_jack *jack, int report)
{
	struct twl6040_data *priv = codec->private_data;
	struct twl6040_jack_data *hs_jack = &priv->hs_jack;
	int status;

	hs_jack->jack = jack;
	hs_jack->report = report;
	status = hs_jack->state ? report : 0;

	snd_soc_jack_report(hs_jack->jack, status, hs_jack->report);
}
EXPORT_SYMBOL_GPL(twl6040_hs_jack_detect);

static struct snd_soc_codec *twl6040_codec;

static int abe_twl6040_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	BUG_ON(!twl6040_codec);

	codec = twl6040_codec;
	socdev->card->codec = codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create pcms\n");
		return ret;
	}

	abe_init_chip(codec, pdev);
	snd_soc_add_controls(codec, twl6040_snd_controls,
				ARRAY_SIZE(twl6040_snd_controls));
	abe_twl6040_add_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
			dev_err(&pdev->dev, "failed to register card\n");
			goto card_err;
	}


	return 0;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	return ret;
}

static int abe_twl6040_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	abe_twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_abe_twl6040 = {
	.probe = abe_twl6040_probe,
	.remove = abe_twl6040_remove,
	.suspend = abe_twl6040_suspend,
	.resume = abe_twl6040_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_abe_twl6040);

static int __devinit abe_twl6040_codec_probe(struct platform_device *pdev)
{
	struct twl4030_codec_data *twl_codec = pdev->dev.platform_data;
	struct snd_soc_codec *codec;
	struct twl6040_data *priv;
	struct twl6040_jack_data *jack;
	int audpwron, naudint;
	int ret = 0;
	u8 icrev, status;

	priv = kzalloc(sizeof(struct twl6040_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &icrev, TWL6040_REG_ASICREV);

	if (twl_codec && (icrev > 0)) {
		audpwron = twl_codec->audpwron_gpio;
	} else {
		audpwron = -EINVAL;
	}

	if (twl_codec)
		naudint = twl_codec->naudint_irq;
	else
		naudint = 0;

	priv->audpwron = audpwron;
	priv->naudint = naudint;

	codec = &priv->codec;
	codec->dev = &pdev->dev;
	codec->name = "twl6040";
	codec->owner = THIS_MODULE;
	codec->read = twl6040_read_reg_cache;
	codec->write = twl6040_write;
	codec->set_bias_level = abe_twl6040_set_bias_level;
	codec->private_data = priv;
	codec->dai = abe_dai;
	codec->num_dai = ARRAY_SIZE(abe_dai);
	codec->reg_cache_size = ARRAY_SIZE(twl6040_reg);
	codec->reg_cache = kmemdup(twl6040_reg, sizeof(twl6040_reg),
					GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto cache_err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	init_completion(&priv->ready);

	/* switch-class based headset detection */
	jack = &priv->hs_jack;
	INIT_WORK(&jack->work, twl6040_hs_jack_detect_work);
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret)
		goto switch_err;

	/* initial headset state */
	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &status, TWL6040_REG_STATUS);
	jack->state = !!(status & TWL6040_PLUGCOMP);
	twl6040_hs_jack_detect_work(&jack->work);

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
				twl6040_naudint_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"twl6040_codec", codec);
		if (ret)
			goto gpio2_err;
	} else {
		if (gpio_is_valid(audpwron)) {
			/* enable only codec ready interrupt */
			twl6040_write_reg_cache(codec, TWL6040_REG_INTMR,
					~TWL6040_READYMSK & TWL6040_ALLINT_MSK);
		} else {
			/* no interrupts at all */
			twl6040_write_reg_cache(codec, TWL6040_REG_INTMR,
						TWL6040_ALLINT_MSK);
		}
	}

	/* init vio registers */
	twl6040_init_vio_regs(codec);

	/* power on device */
	ret = abe_twl6040_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto irq_err;

	ret = snd_soc_register_codec(codec);
	if (ret)
		goto reg_err;

	twl6040_codec = codec;

	ret = snd_soc_register_dais(abe_dai, ARRAY_SIZE(abe_dai));
	if (ret)
		goto dai_err;

	return 0;

dai_err:
	snd_soc_unregister_codec(codec);
	twl6040_codec = NULL;
reg_err:
	abe_twl6040_set_bias_level(codec, SND_SOC_BIAS_OFF);
irq_err:
	if (naudint)
		free_irq(naudint, codec);
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio2_err:
	pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
	if (twl_codec->device_shutdown)
		twl_codec->device_shutdown(pdev);
#endif
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	switch_dev_unregister(&jack->sdev);
switch_err:
	kfree(codec->reg_cache);
cache_err:
	kfree(priv);
	return ret;
}

static int __devexit abe_twl6040_codec_remove(struct platform_device *pdev)
{
	struct twl6040_data *priv = twl6040_codec->private_data;
	struct twl4030_codec_data *pdata = pdev->dev.platform_data;
	struct twl6040_jack_data *jack = &priv->hs_jack;
	int audpwron = priv->audpwron;
	int naudint = priv->naudint;

	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

	if (naudint)
		free_irq(naudint, twl6040_codec);

	pm_runtime_put_sync(&pdev->dev);
#ifndef CONFIG_PM_RUNTIME
	if (pdata->device_shutdown)
		pdata->device_shutdown(pdev);
#endif

	snd_soc_unregister_dais(abe_dai, ARRAY_SIZE(abe_dai));
	snd_soc_unregister_codec(twl6040_codec);
	cancel_work_sync(&jack->work);
	switch_dev_unregister(&jack->sdev);

	kfree(twl6040_codec);
	twl6040_codec = NULL;

	return 0;
}

static struct platform_driver abe_twl6040_codec_driver = {
	.driver = {
		.name = "twl6040_codec",
		.owner = THIS_MODULE,
	},
	.probe = abe_twl6040_codec_probe,
	.remove = __devexit_p(abe_twl6040_codec_remove),
};

static int __init abe_twl6040_codec_init(void)
{
	return platform_driver_register(&abe_twl6040_codec_driver);
}
module_init(abe_twl6040_codec_init);

static void __exit abe_twl6040_codec_exit(void)
{
	platform_driver_unregister(&abe_twl6040_codec_driver);
}
module_exit(abe_twl6040_codec_exit);

MODULE_DESCRIPTION("ASoC ABE-TWL6040 codec driver");
MODULE_AUTHOR("Misael Lopez Cruz");
MODULE_LICENSE("GPL");
