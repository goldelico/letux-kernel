/*
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * Author:zhxiao<zhihao.xiao@ingenic.com>
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/of_gpio.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include "../ingenicfb.h"
#include "../jz_dsim.h"

struct board_gpio {
	short gpio;
	short active_level;
};

struct panel_dev {
	/* ingenic frame buffer */
	struct device *dev;
	struct lcd_panel *panel;

	/* common lcd framework */
	struct lcd_device *lcd;
	struct backlight_device *backlight;
	int power;

	struct regulator *vcc;
	struct board_gpio vdd_en;
	struct board_gpio bl;
	struct board_gpio rst;
	struct board_gpio pwm;
};

static struct panel_dev *panel;

static void kd050wvfpa029_power_on(struct lcd_panel *ppanel)
{
	struct panel_dev *panel_kd050wvfpa029 = dev_get_drvdata(panel->dev);

	if(panel_kd050wvfpa029->bl.gpio > 0){
		gpio_direction_output(panel_kd050wvfpa029->bl.gpio, panel_kd050wvfpa029->bl.active_level);
	}
	if(panel_kd050wvfpa029->vdd_en.gpio > 0){
		gpio_direction_output(panel_kd050wvfpa029->vdd_en.gpio, panel_kd050wvfpa029->vdd_en.active_level);
	}
}

static void kd050wvfpa029_power_off(struct lcd_panel *ppanel)
{
	struct panel_dev *panel_kd050wvfpa029 = dev_get_drvdata(panel->dev);

	if(panel_kd050wvfpa029->bl.gpio > 0){
		gpio_direction_output(panel_kd050wvfpa029->bl.gpio, !panel_kd050wvfpa029->bl.active_level);
	}
	if(panel_kd050wvfpa029->vdd_en.gpio > 0){
		gpio_direction_output(panel_kd050wvfpa029->vdd_en.gpio, !panel_kd050wvfpa029->vdd_en.active_level);
	}
	return;
}

static struct lcd_panel_ops kd050wvfpa029_ops = {
	.enable  = (void*)kd050wvfpa029_power_on,
	.disable = (void*)kd050wvfpa029_power_off,
};

static struct fb_videomode jzfb_kd050wvfpa029_videomode[] = {
	[0] = {
		.name = "800x480",
		.refresh = 60,
		.xres = 800,
		.yres = 480,
		.pixclock = KHZ2PICOS(27000),
		.left_margin = 48,
		.right_margin = 48,
		.upper_margin = 12,
		.lower_margin = 12,
		.hsync_len = 8,
		.vsync_len = 8,
		.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};


struct lcd_panel lcd_panel = {
	.name = "kd050wvfpa029",
	.num_modes = ARRAY_SIZE(jzfb_kd050wvfpa029_videomode),
	.modes = jzfb_kd050wvfpa029_videomode,
	.lcd_type = LCD_TYPE_TFT,
	.bpp = 32,
	.width = 800,
	.height = 480,

	/*T40 RGB real line is 666, so, for RGB888, we discard low 2 bytes, config as follows:*/
	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,

	.ops = &kd050wvfpa029_ops,
};

#define RESET(n)\
	gpio_direction_output(panel->rst.gpio, n)
#define POWER_IS_ON(pwr)     ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/**
 * @ panel_kd050wvfpa029_lcd_ops, register to kernel common backlight/lcd.c frameworks.
 */
static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;

	panel->vdd_en.gpio = of_get_named_gpio_flags(np, "ingenic,vdd-en-gpio", 0, &flags);
	if(gpio_is_valid(panel->vdd_en.gpio)) {
		panel->vdd_en.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request(dev, panel->vdd_en.gpio, "vdd-en");
		if(ret < 0) {
			dev_err(dev, "Failed to request vdd_en pin!\n");
		}
	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

	panel->bl.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags);
	if (gpio_is_valid(panel->bl.gpio)) {
		panel->bl.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request(dev, panel->bl.gpio, "backlight");
		if (ret < 0) {
			dev_err(dev, "Failed to request backlight pin!\n");
		//	return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio backlight.gpio: %d\n", panel->bl.gpio);
	}


	return 0;
}

static struct tft_config kd050wvfpa029_bgr_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.vsync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_BGR,
	.color_odd = TFT_LCD_COLOR_ODD_BGR,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

static struct tft_config kd050wvfpa029_rgb_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.vsync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,kd050wvfpa029", .data = (struct tft_config *)&kd050wvfpa029_bgr_cfg, },
	{ .compatible = "ingenic,kd050wvfpa029-rgb", .data = (struct tft_config *)&kd050wvfpa029_rgb_cfg, },
	{},
};

/**
 * @panel_probe
 *
 *	1. register to ingenicfb.
 *	2. register to lcd.
 *	3. register to backlight if possible.
 *
 *	@pdev
 *
 *	@return -
 * */
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct backlight_properties props;   //backlight properties
	const struct of_device_id *priv;

	memset(&props, 0, sizeof(props));
	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (NULL == panel) {
		dev_err(&pdev->dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, panel);

	/* panel pinctrl parse */
	ret = of_panel_parse(&pdev->dev);
	if (ret < 0) {
		goto err_of_parse;
	}

	panel->lcd = lcd_device_register("panel_lcd", &pdev->dev, panel, &panel_lcd_ops);
	if (IS_ERR_OR_NULL(panel->lcd)) {
		dev_err(&pdev->dev, "Error register lcd!");
		ret = -EINVAL;
		goto err_of_parse;
	}

	priv = of_match_node(panel_of_match, pdev->dev.of_node);
	lcd_panel.tft_config = (struct tft_config *)priv->data;

	/* TODO: should this power status sync from uboot */
	panel->power = FB_BLANK_POWERDOWN;
	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);

	ret = ingenicfb_register_panel(&lcd_panel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register lcd panel!\n");
		goto err_lcd_register;
	}

	return 0;

err_lcd_register:
	lcd_device_unregister(panel->lcd);
err_of_parse:
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	struct panel_dev *panel = dev_get_drvdata(&pdev->dev);

	panel_set_power(panel->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

#ifdef CONFIG_PM
static int panel_suspend(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

static int panel_resume(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_suspend,
	.resume = panel_resume,
};
#endif

static struct platform_driver panel_driver = {
	.probe = panel_probe,
	.remove = panel_remove,
	.driver = {
		.name = "kd050wvfpa029",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
