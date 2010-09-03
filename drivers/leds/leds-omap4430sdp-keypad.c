/*
 * Phoenix Keypad LED Driver for the OMAP4430 SDP
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl.h>

#define OMAP4430_KEYPAD_LED_DBG 1

struct keypad_led_data {
	struct led_classdev keypad_led_class_dev;
};

#define KP_LED_PWM1ON		0x00
#define KP_LED_PWM1OFF		0x01
#define KP_LED_TOGGLE3		TWL6030_TOGGLE3

#if OMAP4430_KEYPAD_LED_DBG
struct omap4430_keypad_led_regs {
	const char *name;
	uint8_t reg;
} omap4430sdp_keypad_regs[] = {
	{ "PWM1ON",	KP_LED_PWM1ON },
	{ "PWM1OFF",	KP_LED_PWM1OFF },
	{ "TOGGLE3",	KP_LED_TOGGLE3 },
};
#endif

static void omap4430_keypad_led_store(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	u8 brightness = 0;

	if (value > 1) {
		if (value == LED_FULL)
			brightness = 0x7f;
		else
			brightness = (~(value/2)) & 0x7f;

		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, KP_LED_PWM1ON);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x36, TWL6030_TOGGLE3);
	} else {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x31, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x37, TWL6030_TOGGLE3);
	}

}

#if OMAP4430_KEYPAD_LED_DBG
static ssize_t ld_omap4430_keypad_led_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	unsigned i, n, reg_count;
	uint8_t value;

	reg_count = sizeof(omap4430sdp_keypad_regs) /
			sizeof(omap4430sdp_keypad_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		twl_i2c_read_u8(TWL_MODULE_PWM, &value,
				omap4430sdp_keypad_regs[i].reg);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       omap4430sdp_keypad_regs[i].name,
			       value);
	}

	return n;
}

static ssize_t ld_omap4430_keypad_led_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(omap4430sdp_keypad_regs) /
			sizeof(omap4430sdp_keypad_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, omap4430sdp_keypad_regs[i].name)) {
			if (!strcmp("TOGGLE3",
				omap4430sdp_keypad_regs[i].name)) {
				error = twl_i2c_write_u8(TWL6030_MODULE_ID1,
					value,
					omap4430sdp_keypad_regs[i].reg);

			} else {
				error = twl_i2c_write_u8(TWL_MODULE_PWM, value,
					omap4430sdp_keypad_regs[i].reg);
			}
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, 0644, ld_omap4430_keypad_led_registers_show,
		ld_omap4430_keypad_led_registers_store);
#endif

static int omap4430_keypad_led_probe(struct platform_device *pdev)
{
	int ret;
	struct keypad_led_data *info;

	pr_info("%s:Enter\n", __func__);

	info = kzalloc(sizeof(struct keypad_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(pdev, info);

	info->keypad_led_class_dev.name = "keyboard-backlight";
	info->keypad_led_class_dev.brightness_set =
			omap4430_keypad_led_store;
	info->keypad_led_class_dev.max_brightness = LED_FULL;

	ret = led_classdev_register(&pdev->dev,
				    &info->keypad_led_class_dev);
	if (ret < 0) {
		pr_err("%s: Register led class failed\n", __func__);
		kfree(info);
		return ret;
	}

#if OMAP4430_KEYPAD_LED_DBG
	ret = device_create_file(info->keypad_led_class_dev.dev,
			&dev_attr_registers);
	if (ret < 0) {
		pr_err("%s: Could not register registers fd \n",
			__func__);

	}
#endif
	/*TO DO pass in these values from the board file */
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, KP_LED_PWM1ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7f, KP_LED_PWM1OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x06, TWL6030_TOGGLE3);

	pr_info("%s:Exit\n", __func__);

	return ret;
}

static int omap4430_keypad_led_remove(struct platform_device *pdev)
{
	struct keypad_led_data *info = platform_get_drvdata(pdev);
#if OMAP4430_KEYPAD_LED_DBG
	device_remove_file(info->keypad_led_class_dev.dev,
			&dev_attr_registers);
#endif
	led_classdev_unregister(&info->keypad_led_class_dev);
	return 0;
}

static struct platform_driver omap4430_keypad_led_driver = {
	.probe = omap4430_keypad_led_probe,
	.remove = omap4430_keypad_led_remove,
	.driver = {
		   .name = "keypad_led",
		   .owner = THIS_MODULE,
		   },
};

static int __init omap4430_keypad_led_init(void)
{
	return platform_driver_register(&omap4430_keypad_led_driver);
}

static void __exit omap4430_keypad_led_exit(void)
{
	platform_driver_unregister(&omap4430_keypad_led_driver);
}

module_init(omap4430_keypad_led_init);
module_exit(omap4430_keypad_led_exit);

MODULE_DESCRIPTION("OMAP4430 SDP Keypad Lighting driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
