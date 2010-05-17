/*
 * PWM LED driver data - see drivers/leds/leds-pwm.c
 */
#ifndef __LINUX_LEDS_PWM_H
#define __LINUX_LEDS_PWM_H

struct led_pwm {
	const char	*name;
	const char	*default_trigger;
	unsigned	pwm_id;
	u8 		active_low;
	unsigned 	max_brightness;
	unsigned	pwm_period_ns;
};

struct led_pwm_platform_data {
	int			num_leds;
	struct led_pwm	*leds;

	/* @init: The init callback is called after the pwm device for a led has
	 * been successfully configured. If the return value is negative it will be
	 * seen as an error and initzalisation of the leds-pwm device will fail.
	 */
	int (*init)(struct device *dev, struct led_pwm *led);

	/* @notify: The notify callback is called whenever the brightness of a led
	 * is changed.
	 * The return value of the callback will be the brightness which is used to
	 * configure the pwm device.
	 */
	enum led_brightness (*notify)(struct device *dev, struct led_pwm *led,
	    enum led_brightness brightness);

	/* @exit: The exit callback is called, whenever a led device registered by
	 * the leds-pwm device is unregistered. It will be called prior to freeing
	 * the pwm device.
	 */
	void (*exit)(struct device *dev, struct led_pwm *led);
};

#endif
