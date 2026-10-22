/* (not yet) automatically extracted from official include/sound/soc.h */

// we #include <sound/soc.h> for
// struct soc_mixer_control - used in lib.c
// struct soc_enum - used in lib.c and abe-dl1dl2-twl6040.c
// but this this file is NOT uapi
// and tries to source many <linux/*.h>
// hence we have a local copy in ./sound/soc.h
// latest check with v7.3-rc7

#ifndef __LINUX_SND_SOC_H
#define __LINUX_SND_SOC_H

/* mixer control */
struct soc_mixer_control {
	int min, max, platform_max;
	int reg, rreg;
	unsigned int shift, rshift;
	unsigned int sign_bit;
	u32 num_channels;
	unsigned int invert:1;
	unsigned int autodisable:1;
#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj;
#endif
};

/* enumerated kcontrol */
struct soc_enum {
	int reg;
	unsigned char shift_l;
	unsigned char shift_r;
	unsigned int items;
	unsigned int mask;
	const char * const *texts;
	const unsigned int *values;
	unsigned int autodisable:1;
#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj;
#endif
};

#endif // __LINUX_SND_SOC_H
