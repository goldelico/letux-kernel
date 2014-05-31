/*
 * gpio-reg: virtual gpio which enables/disabled a regulator
 */

struct gpio_reg_data {
	int gpio;
	int uV;
};
