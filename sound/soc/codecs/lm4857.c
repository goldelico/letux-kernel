/*
 * LM4857 AMP driver
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "lm4857.h"

static struct {
	struct i2c_client *i2c;
	uint8_t regs[4];
	uint8_t state;
} lm4857 = {
	.regs = {0x00, 0x40, 0x80, 0xC0},
};

static void lm4857_write_regs(void)
{
	if (!lm4857.i2c)
		return;

	if (i2c_master_send(lm4857.i2c, lm4857.regs, 4) != 4)
		dev_err(&lm4857_i2c->dev, "lm4857: i2c write failed\n");
}

static int lm4857_get_reg(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int reg = mc->reg;
	int shift = mc->shift;
	int mask = mc->max;

	ucontrol->value.integer.value[0] = (lm4857.regs[reg] >> shift) & mask;
	return 0;
}

static int lm4857_set_reg(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int reg = mc->reg;
	int shift = mc->shift;
	int mask = mc->max;

	if (((lm4857.regs[reg] >> shift) & mask) ==
		ucontrol->value.integer.value[0])
		return 0;

	lm4857.regs[reg] &= ~(mask << shift);
	lm4857.regs[reg] |= ucontrol->value.integer.value[0] << shift;

	lm4857_write_regs();

	return 1;
}

static int lm4857_get_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	uint8_t value = lm4857.regs[LM4857_CTRL] & 0x0F;

	if (value)
		value -= 5;

	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int lm4857_set_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	uint8_t value = ucontrol->value.integer.value[0];

	if (value)
		value += 5;

	if ((lm4857.regs[LM4857_CTRL] & 0x0F) == value)
		return 0;

	lm4857.regs[LM4857_CTRL] &= 0xF0;
	lm4857.regs[LM4857_CTRL] |= value;
	lm4857_write_regs();
	return 1;
}

static const char *lm4857_mode[] = {
	"Off",
	"Call Speaker",
	"Stereo Speakers",
	"Stereo Speakers + Headphones",
	"Headphones"
};

static const struct soc_enum lm4857_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lm4857_mode), lm4857_mode),
};

static const DECLARE_TLV_DB_SCALE(stereo_tlv, -4050, 150, 0);
static const DECLARE_TLV_DB_SCALE(mono_tlv, -3450, 150, 0);

static const struct snd_kcontrol_new lm4857_controls[] = {
	SOC_SINGLE_EXT_TLV("Amp Left Playback Volume", LM4857_LVOL, 0, 31, 0,
		lm4857_get_reg, lm4857_set_reg, stereo_tlv),
	SOC_SINGLE_EXT_TLV("Amp Right Playback Volume", LM4857_RVOL, 0, 31, 0,
		lm4857_get_reg, lm4857_set_reg, stereo_tlv),
	SOC_SINGLE_EXT_TLV("Amp Mono Playback Volume", LM4857_MVOL, 0, 31, 0,
		lm4857_get_reg, lm4857_set_reg, mono_tlv),
	SOC_ENUM_EXT("Amp Mode", lm4857_mode_enum[0],
		lm4857_get_mode, lm4857_set_mode),
	SOC_SINGLE_EXT("Amp Spk 3D Playback Switch", LM4857_LVOL, 5, 1, 0,
		lm4857_get_reg, lm4857_set_reg),
	SOC_SINGLE_EXT("Amp HP 3d Playback Switch", LM4857_RVOL, 5, 1, 0,
		lm4857_get_reg, lm4857_set_reg),
	SOC_SINGLE_EXT("Amp Fast Wakeup Playback Switch", LM4857_CTRL, 5, 1, 0,
		lm4857_get_reg, lm4857_set_reg),
	SOC_SINGLE_EXT("Amp Earpiece 6dB Playback Switch", LM4857_CTRL, 4, 1, 0,
		lm4857_get_reg, lm4857_set_reg),
};

int lm4857_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, lm4857_controls,
				ARRAY_SIZE(lm5857_controls));
}
EXPORT_GPL(lm4857_add_controlls);

static int __devinit lm4857_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	lm4857.i2c = client;
	lm4857_write_regs();
	return 0;
}

static int __devexit lm4857_remove(struct i2c_client *client)
{
	lm4857.i2c = NULL;
	return 0;
}

static void lm4857_shutdown(struct i2c_client *client)
{
	lm4857_regs[LM4857_CTRL] &= 0xf0;
	lm4857_write_regs();
}

#ifdef CONFIG_PM

static int lm4857_suspend(struct i2c_client *client, pm_message_t state)
{
	lm4857.state = lm4857_regs[LM4857_CTRL] & 0xf;

	if (lm4857.state)
		lm4857_shutdown(client);

	return 0;
}

static int lm4857_resume(struct i2c_client *dev)
{
	if (lm4857.state) {
		lm4857_regs[LM4857_CTRL] |= (lm4857.state & 0x0f);
		lm4857_write_regs();
	}
	return 0;
}

#else
#define lm4857_suspend NULL
#define lm4857_resume NULL
#endif

static const struct i2c_device_id lm4857_i2c_id[] = {
	{ "lm4857", 0 },
	{ }
};

static struct i2c_driver lm4857_i2c_driver = {
	.driver = {
		.name = "LM4857 I2C Amp",
		.owner = THIS_MODULE,
	},
	.probe =          lm4857_probe,
	.remove =         __devexit_p(lm4857_remove),
	.suspend =        lm4857_suspend,
	.resume	=         lm4857_resume,
	.shutdown =       lm4857_shutdown,
	.id_table =       lm4857_i2c_id,
};

static int __init lm4857_init(void)
{
	return i2c_add_driver(&lm4857_i2c_driver);
}
module_init(lm4857_init);

static void __exit lm4857_exit(void)
{
	i2c_del_driver(&lm4857_i2c_driver);
}
module_exit(lm4857_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("LM4857 amplifier driver");
MODULE_LICENSE("GPL");
