/*
 * driver/video/fbdev/ingenic/fb_stage/displays/panel-gc9203.c
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

#include "../ingenicfb.h"

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
	struct board_gpio cs;
	struct board_gpio rst;
	struct board_gpio vdd_en;
	struct board_gpio vddio_en;
	struct board_gpio pwm;
};

static struct smart_lcd_data_table gc9203_data_table[] = {

    /* LCD init code */
	{SMART_CONFIG_UDELAY, 120000},
	{SMART_CONFIG_CMD, 0xff},
	{SMART_CONFIG_PRM, 0x5a},
	{SMART_CONFIG_PRM, 0xa5},

	{SMART_CONFIG_CMD, 0xf6},
	{SMART_CONFIG_PRM, 0x01},
	{SMART_CONFIG_PRM, 0x12},

	{SMART_CONFIG_CMD, 0xef},
	{SMART_CONFIG_PRM, 0x14},
	{SMART_CONFIG_PRM, 0x52},

	{SMART_CONFIG_CMD, 0xa5},
	{SMART_CONFIG_PRM, 0x07},
	{SMART_CONFIG_PRM, 0x80},

	{SMART_CONFIG_CMD, 0x02},
	{SMART_CONFIG_PRM, 0x01},
	{SMART_CONFIG_PRM, 0x00},

	{SMART_CONFIG_CMD, 0x03},
	{SMART_CONFIG_PRM, 0x10},	//0x10, 65k color, rgb565
//	{SMART_CONFIG_PRM, 0x12},	//0x12, 262k color, rgb666
	{SMART_CONFIG_PRM, 0x18},

	{SMART_CONFIG_CMD, 0xf6},
	{SMART_CONFIG_PRM, 0x01},
	{SMART_CONFIG_PRM, 0x12},

	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_PRM, 0x10},
	{SMART_CONFIG_PRM, 0x00},

	{SMART_CONFIG_CMD, 0xeb},
	{SMART_CONFIG_PRM, 0x16},
	{SMART_CONFIG_PRM, 0x0a},

	{SMART_CONFIG_CMD, 0xec},
	{SMART_CONFIG_PRM, 0x16},
	{SMART_CONFIG_PRM, 0x0a},

	{SMART_CONFIG_CMD, 0x50},
	{SMART_CONFIG_PRM, 0xf9},
	{SMART_CONFIG_PRM, 0x80},

	{SMART_CONFIG_CMD, 0x51},
	{SMART_CONFIG_PRM, 0x13},
	{SMART_CONFIG_PRM, 0x06},

	{SMART_CONFIG_CMD, 0x52},
	{SMART_CONFIG_PRM, 0x0e},
	{SMART_CONFIG_PRM, 0x09},

	{SMART_CONFIG_CMD, 0x53},
	{SMART_CONFIG_PRM, 0x33},
	{SMART_CONFIG_PRM, 0x09},

	{SMART_CONFIG_CMD, 0x54},
	{SMART_CONFIG_PRM, 0x4e},
	{SMART_CONFIG_PRM, 0x24},

	{SMART_CONFIG_CMD, 0x55},
	{SMART_CONFIG_PRM, 0x1b},
	{SMART_CONFIG_PRM, 0x1c},

	{SMART_CONFIG_CMD, 0x56},
	{SMART_CONFIG_PRM, 0x37},
	{SMART_CONFIG_PRM, 0x36},

	{SMART_CONFIG_CMD, 0x57},
	{SMART_CONFIG_PRM, 0xf5},
	{SMART_CONFIG_PRM, 0x80},

	{SMART_CONFIG_CMD, 0x58},
	{SMART_CONFIG_PRM, 0x13},
	{SMART_CONFIG_PRM, 0x06},

	{SMART_CONFIG_CMD, 0x59},
	{SMART_CONFIG_PRM, 0x0e},
	{SMART_CONFIG_PRM, 0x13},

	{SMART_CONFIG_CMD, 0x5a},
	{SMART_CONFIG_PRM, 0x3f},
	{SMART_CONFIG_PRM, 0x09},

	{SMART_CONFIG_CMD, 0x5b},
	{SMART_CONFIG_PRM, 0x52},
	{SMART_CONFIG_PRM, 0x24},

	{SMART_CONFIG_CMD, 0x5c},
	{SMART_CONFIG_PRM, 0x0f},
	{SMART_CONFIG_PRM, 0x0d},

	{SMART_CONFIG_CMD, 0x5d},
	{SMART_CONFIG_PRM, 0x37},
	{SMART_CONFIG_PRM, 0x36},

	{SMART_CONFIG_CMD, 0x07},
	{SMART_CONFIG_PRM, 0x10},
	{SMART_CONFIG_PRM, 0x13},

	{SMART_CONFIG_CMD, 0x01},
	{SMART_CONFIG_PRM, 0x01},
	{SMART_CONFIG_PRM, 0x1c},

	{SMART_CONFIG_CMD, 0xfe},

/* set display resolution */
/* 6.2.18. Horizontal and Vertical RAM Address Position (R36h/R37h,R38h/R39h) */
	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_PRM, 0x00},
	{SMART_CONFIG_PRM, 0x98},
	{SMART_CONFIG_CMD, 0x37},
	{SMART_CONFIG_PRM, 0x00},
	{SMART_CONFIG_PRM, 0x18},

	{SMART_CONFIG_UDELAY, 120000},

	{SMART_CONFIG_CMD, 0x22},
};


static struct fb_videomode panel_modes[] = {
	[0] = {
		.name = "220x128",
		.refresh = 60,
		.xres = 220,
		.yres = 128+1,
		.pixclock = KHZ2PICOS(30000),
		.left_margin = 0,
		.right_margin = 0,
		.upper_margin = 0,
		.lower_margin = 0,
		.hsync_len = 0,
		.vsync_len = 0,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

struct smart_config gc9203_cfg = {
	.te_anti_jit = 1,
	.te_md = 1,
	.te_dp = 1,
	.te_switch = 0,
	.dc_md = 0,
	.wr_md = 1,
	.smart_type = SMART_LCD_TYPE_SPI_4,
	.pix_fmt = SMART_LCD_FORMAT_565,

	.write_gram_cmd = 0x22222222,
	.data_table = gc9203_data_table,
	.length_data_table = ARRAY_SIZE(gc9203_data_table),
};

struct lcd_panel lcd_panel = {
	.name = "gc9203",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.lcd_type = LCD_TYPE_SLCD,
	.width = 220,
	.height = 128,

	.smart_config = &gc9203_cfg,

	.dither_enable = 0,
	.dither.dither_red = 2,
	.dither.dither_green = 1,
	.dither.dither_blue = 2,
};

/* SGM3146 supports 16 brightness step */
#define MAX_BRIGHTNESS_STEP     16
/* System support 256 brightness step */
#define CONVERT_FACTOR          (256/MAX_BRIGHTNESS_STEP)

static int panel_update_status(struct backlight_device *bd)
{
	struct panel_dev *panel = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	unsigned int i;
	int pulse_num = MAX_BRIGHTNESS_STEP - brightness / CONVERT_FACTOR - 1;

	if (bd->props.fb_blank == FB_BLANK_POWERDOWN) {
		return 0;
	}

	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	if (brightness) {
		gpio_direction_output(panel->pwm.gpio,0);
		udelay(5000);
		gpio_direction_output(panel->pwm.gpio,1);
		udelay(100);

		for (i = pulse_num; i > 0; i--) {
			gpio_direction_output(panel->pwm.gpio,0);
			udelay(1);
			gpio_direction_output(panel->pwm.gpio,1);
			udelay(3);
		}
	} else
		gpio_direction_output(panel->pwm.gpio, 0);

	return 0;
}

static struct backlight_ops panel_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = panel_update_status,
};

static void panel_power_reset(struct board_gpio *rst)
{
	gpio_direction_output(rst->gpio, 0);
	mdelay(20);
	gpio_direction_output(rst->gpio, 1);
	mdelay(10);
}

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);
	struct board_gpio *cs = &panel->cs;
	struct board_gpio *rst = &panel->rst;
	struct board_gpio *vdd_en = &panel->vdd_en;
	struct board_gpio *vddio_en = &panel->vddio_en;

	if(POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
		gpio_direction_output(vdd_en->gpio, 1);
		gpio_direction_output(vddio_en->gpio, 1);
		gpio_direction_output(rst->gpio, 1);
		gpio_direction_output(cs->gpio, 1);
		mdelay(5);
		panel_power_reset(rst);
		gpio_direction_output(cs->gpio, 0);
	}
	if(!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
		gpio_direction_output(cs->gpio, 0);
		gpio_direction_output(rst->gpio, 0);
		gpio_direction_output(vddio_en->gpio, 0);
		gpio_direction_output(vdd_en->gpio, 0);
	}

	panel->power = power;
        return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

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

	panel->cs.gpio = of_get_named_gpio_flags(np, "ingenic,cs-gpio", 0, &flags);
	if(gpio_is_valid(panel->cs.gpio)) {
		panel->cs.active_level = OF_GPIO_ACTIVE_LOW ? 0 : 1;
		ret = gpio_request_one(panel->cs.gpio, GPIOF_DIR_OUT, "cs");
		if(ret < 0) {
			dev_err(dev, "Failed to request cs pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio cs.gpio: %d\n", panel->cs.gpio);
	}

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = OF_GPIO_ACTIVE_LOW ? 0 : 1;
		ret = gpio_request_one(panel->rst.gpio, GPIOF_DIR_OUT, "rst");
		if(ret < 0) {
			dev_err(dev, "Failed to request rst pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	panel->vdd_en.gpio = of_get_named_gpio_flags(np, "ingenic,vdd-en-gpio", 0, &flags);
	if(gpio_is_valid(panel->vdd_en.gpio)) {
		panel->vdd_en.active_level = OF_GPIO_ACTIVE_LOW ? 0 : 1;
		ret = gpio_request_one(panel->vdd_en.gpio, GPIOF_DIR_OUT, "vdd_en");
		if(ret < 0) {
			dev_err(dev, "Failed to request vdd_en pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

	panel->vddio_en.gpio = of_get_named_gpio_flags(np, "ingenic,vddio-en-gpio", 0, &flags);
	if(gpio_is_valid(panel->vddio_en.gpio)) {
		panel->vddio_en.active_level = OF_GPIO_ACTIVE_LOW ? 0 : 1;
		ret = gpio_request_one(panel->vddio_en.gpio, GPIOF_DIR_OUT, "vddio_en");
		if(ret < 0) {
			dev_err(dev, "Failed to request vddio_en pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio vddio_en.gpio: %d\n", panel->vddio_en.gpio);
	}

	panel->pwm.gpio = of_get_named_gpio_flags(np, "ingenic,pwm-gpio", 0, &flags);
	if(gpio_is_valid(panel->pwm.gpio)) {
		panel->pwm.active_level = OF_GPIO_ACTIVE_LOW ? 0 : 1;
		ret = gpio_request_one(panel->pwm.gpio, GPIOF_DIR_OUT, "pwm");
		if(ret < 0) {
			dev_err(dev, "Failed to request pwm pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio pwm.gpio: %d\n", panel->pwm.gpio);
	}

	return 0;
err_request_rst:
	gpio_free(panel->cs.gpio);
	return ret;
}
/**
* @panel_probe
*
* 	1. Register to ingenicfb.
* 	2. Register to lcd.
* 	3. Register to backlight if possible.
*
* @pdev
*
* @Return -
*/
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct panel_dev *panel;
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if(panel == NULL) {
		dev_err(&pdev->dev, "Faile to alloc memory!");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, panel);

	ret = of_panel_parse(&pdev->dev);
	if(ret < 0) {
		goto err_of_parse;
	}

	panel->lcd = lcd_device_register("panel_lcd", &pdev->dev, panel, &panel_lcd_ops);
	if(IS_ERR_OR_NULL(panel->lcd)) {
		dev_err(&pdev->dev, "Error register lcd!\n");
		ret = -EINVAL;
		goto err_of_parse;
	}

	/* TODO: should this power status sync from uboot */
	panel->power = FB_BLANK_POWERDOWN;
	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);

	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	panel->backlight = backlight_device_register("pwm-backlight.0",
						&pdev->dev, panel,
						&panel_backlight_ops,
						&props);
	if (IS_ERR_OR_NULL(panel->backlight)) {
		dev_err(panel->dev, "failed to register 'pwm-backlight.0'.\n");
		goto err_lcd_register;
	}
	panel->backlight->props.brightness = props.max_brightness;
	backlight_update_status(panel->backlight);

	ret = ingenicfb_register_panel(&lcd_panel);
	if(ret < 0) {
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
static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,gc9203", },
	{},
};

static struct platform_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.driver		= {
		.name	= "gc9203",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
