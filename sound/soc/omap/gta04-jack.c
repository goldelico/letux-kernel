
/*
 * Jack driver for GTA04.
 * Copyright Neil Brown <neilb@suse.de> 2013
 *
 * The DC current through the headset microphone pins is
 * converted to a voltage which is presented on TWL4030 madc 7.
 * To be able to read a current, the Headset Mic Bias must
 * be enabled.
 *
 * When the jack device is open, enable the Headset Mic Bias
 * and poll mdac 7 every 500msec.  Once we see an insertion,
 * we increase the rate to ever 50msec until we see a removal.
 *
 * There are 4 possible states:
 * - Nothing plugged in, open circuit - voltage is low
 * - short circuit due to headphone with no mic (3-contact TRS)
 *   inserted.  Voltage is high.
 * - short circuit due to button on headset being pushed.
 *   Voltage is also high.
 * - Microphone is in circuit.  Voltage is even higher.  I don't
 *   understand how it can be higher than than with a short
 *   circuit, but that is what I measure.
 *
 * To differentiate between the two short circuits we look at how
 * we got there.  A transition from open to short means a 3-contact
 * TRS with no mic.  A transition from mic to short means the button
 * on the mic was pressed.
 *
 * As different devices report different actual voltages we need
 * some calibration.  As we cannot do this automatically we complete
 * precision, we allow user-space to tell us the calibration.
 * We assume that open-circuit is always below 100, and other
 * readings are above that.
 * The highest level we see for 3 consecutive readings is assumed
 * to be the 'microphone' level, and short-circuit is 5% below that.
 * If headphones with no mic are inserted this will be wrong, but not
 * terribly wrong.  As soon as a headset with a mic is inserted it
 * will get corrected and stay corrected.
 * In order to keep this correct across a reboot, user-space can
 * read the current setting from
 *      /sys/modules/snd_soc_gta04/parameters/jack_level.
 * and then write back the value after reboot.  Once a value is
 * written, auto-calibration is disabled.
 * Writing the value '0' can re-enable auto-calibration.
 */

#include <linux/input.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <linux/suspend.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/module.h>

static struct {
	struct snd_soc_jack hs_jack;
	struct delayed_work jack_work;
	struct snd_soc_codec *codec;
	int open;
	/* When any jack is present, we:
	 * - poll more quickly to catch button presses
	 * - assume a 'short' is 'button press', not 'headset has
	 *   no mic
	 * 'present' stores SND_JACK_HEADPHONE or SND_JACK_HEADSET
	 * indicating what we think is present.
	 */
	int present;
	/* Calibration reports a single number which roughly
	 * points to 'short'.
	 * Less than half this is 'open circuit'.
	 * More than this is 'microphone
	 */
	long level;
	int level_fixed;
	long level_new;
	int level_count;
} jack;

static void gta04_jack_work(struct work_struct *work)
{
	long val;
	long delay = msecs_to_jiffies(500);
	int jackbits;

	/* choose delay *before* checking presence so we still get
	 * one long delay on first insertion to help with debounce.
	 */
	if (jack.present)
		delay = msecs_to_jiffies(50);

	val = twl4030_get_madc_conversion(7);
	if (val < 0)
		goto out;
	/* On my device:
	 * open circuit = around 20
	 * short circuit = around 800, or 325 on another device
	 * microphone   = around 830-840 !!! 345 on other device.
	 */
	if (!jack.level_fixed) {
		if (jack.level * 21/20 + 2 < val) {
			if (jack.level_count == 0 ||
			    val < jack.level_new*21/20)
				jack.level_new = val*20/21;
			if (jack.level_count >= 3) {
				jack.level = jack.level_new;
				jack.level_count = 0;
			} else
				jack.level_count += 1;
		} else
			jack.level_count = 0;
	}
	if (val < jack.level / 2) {
		/* open circuit */
		jackbits = 0;
		jack.present = 0;
		/* debounce */
		delay = msecs_to_jiffies(500);
	} else if (val < jack.level) {
		/* short */
		if (jack.present == 0) {
			/* Inserted headset with no mic */
			jack.present = SND_JACK_HEADPHONE;
			jackbits = jack.present;
		} else if (jack.present & SND_JACK_MICROPHONE) {
			/* mic shorted -> button press */
			jackbits = SND_JACK_BTN_0 | jack.present;
		} else {
			/* headphones still present */
			jackbits = jack.present;
		}
	} else {
		/* There is a microphone there */
		jack.present = SND_JACK_HEADSET;
		jackbits = jack.present;
	}
	snd_soc_jack_report(&jack.hs_jack, jackbits,
			    SND_JACK_HEADSET | SND_JACK_BTN_0);

out:
	if (jack.open)
		schedule_delayed_work(&jack.jack_work, delay);
}

static int gta04_jack_pm_notify(struct notifier_block *b, unsigned long v, void *d)
{
	if (!jack.codec || !jack.open)
		return 0;
	switch(v) {
	case PM_SUSPEND_PREPARE:
		/* Disable Headset Mic Bias while asleep */
		snd_soc_dapm_disable_pin(&jack.codec->dapm, "Headset Mic Bias");
		snd_soc_dapm_sync(&jack.codec->dapm);
		break;

	case PM_POST_SUSPEND:
		snd_soc_dapm_force_enable_pin(&jack.codec->dapm, "Headset Mic Bias");
		snd_soc_dapm_sync(&jack.codec->dapm);
		break;
	default: break;
	}
	return 0;
}

static struct notifier_block gta04_jack_pm_notify_block = {
	.notifier_call = gta04_jack_pm_notify,
};

static int gta04_jack_open(struct input_dev *dev)
{
	snd_soc_dapm_force_enable_pin(&jack.codec->dapm, "Headset Mic Bias");
	snd_soc_dapm_sync(&jack.codec->dapm);
	jack.open = 1;
	schedule_delayed_work(&jack.jack_work, msecs_to_jiffies(100));
	return 0;
}

static void gta04_jack_close(struct input_dev *dev)
{
	jack.open = 0;
	cancel_delayed_work_sync(&jack.jack_work);
	snd_soc_dapm_disable_pin(&jack.codec->dapm, "Headset Mic Bias");
	snd_soc_dapm_sync(&jack.codec->dapm);
}

int gta04_jack_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	ret = snd_soc_jack_new(codec, "Headset Jack",
			       SND_JACK_HEADSET | SND_JACK_BTN_0,
			       &jack.hs_jack);
	if (ret)
		return ret;
	register_pm_notifier(&gta04_jack_pm_notify_block);

	INIT_DELAYED_WORK(&jack.jack_work, gta04_jack_work);
	jack.codec = codec;
	if (jack.level < 100)
		jack.level = 100;
	jack.hs_jack.jack->input_dev->open = gta04_jack_open;
	jack.hs_jack.jack->input_dev->close = gta04_jack_close;

	return snd_soc_dapm_sync(dapm);
}

void gta04_jack_remove(struct snd_soc_codec *codec)
{
	unregister_pm_notifier(&gta04_jack_pm_notify_block);
	cancel_delayed_work(&jack.jack_work);
}

static int get_level(char *buffer, struct kernel_param *kp)
{
	return sprintf(buffer, "%ld", jack.level);
}
static int set_level(const char *val, struct kernel_param *kp)
{
	long num;
	if (kstrtol(val, 10, &num) < 0)
		return -EINVAL;
	if (num == 0) {
		jack.level = 100;
		jack.level_fixed = 0;
	} else {
		jack.level = num;
		jack.level_fixed = 1;
	}
	return 0;
}
module_param_call(jack_level, set_level, get_level, NULL, S_IRUSR|S_IWUSR);
