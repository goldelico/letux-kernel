/*
 * driver/video/fbdev/ingenic/fb_stage/displays/panel-fw040.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
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

struct gpio_spi {
	short sdo;
	short sdi;
	short sck;
	short cs;
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
	struct board_gpio rst;
	struct board_gpio pwm;
	struct gpio_spi spi;
};

struct fw040_pinctrl{
	struct pinctrl_state *sda_high;
	struct pinctrl_state *sda_low;
	struct pinctrl_state *sck_high;
	struct pinctrl_state *sck_low;
	struct pinctrl_state *i2c;
};

static struct panel_dev *panel;
static struct fw040_pinctrl *state;
static struct pinctrl *p = NULL;

#define RESET(n)\
     gpio_direction_output(panel->rst.gpio, n)

#define CS(n)\
     gpio_direction_output(panel->spi.cs, n)

#define SCK_HIGH()\
	pinctrl_select_state(p, state->sck_high)

#define SCK_LOW()\
	pinctrl_select_state(p, state->sck_low)
#define SDA_HIGH()\
	pinctrl_select_state(p, state->sda_high)

#define SDA_LOW()\
	pinctrl_select_state(p, state->sda_low)

void SPI_SendData(unsigned short data)
{
   unsigned char n;

   for(n = 0; n < 16; n++) {
	   if(data&0x8000)
		   SDA_HIGH();
	   else
		   SDA_LOW();
	   data = data << 1;

	   SCK_HIGH();
	   udelay(1);
	   SCK_LOW();
	   udelay(1);
   }
}

void spi_writecomm(unsigned short cmd)
{
	unsigned short addr_h = (0x2000 + ((cmd >> 8) & 0xff));
	unsigned short addr_l = (cmd & 0xff);

	CS(0);
	udelay(50);

	SPI_SendData(addr_h);

	SPI_SendData(addr_l);

	CS(1);
	udelay(50);
}

void spi_writedata(unsigned char d)
{
	unsigned short data = (0x4000 + d);

	CS(0);
	udelay(50);

	SPI_SendData(data);

	CS(1);
	udelay(50);
}

void Initial_IC(void)
{
	unsigned char data = 0;

	SDA_LOW();
	SCK_LOW();
	CS(1);

	udelay(1200);
	spi_writecomm(0xFF00);spi_writedata(0x77);
	spi_writecomm(0XFF01);spi_writedata(0x01);
	spi_writecomm(0xFF02);spi_writedata(0x00);
	spi_writecomm(0xFF03);spi_writedata(0x00);
	spi_writecomm(0xFF04);spi_writedata(0x10);

	spi_writecomm(0xC000);spi_writedata(0x63);
	spi_writecomm(0xC001);spi_writedata(0x00);

	spi_writecomm(0xC100);spi_writedata(0x11);
	spi_writecomm(0xC101);spi_writedata(0x02);

	spi_writecomm(0xC200);spi_writedata(0x01);
	spi_writecomm(0xC201);spi_writedata(0x08);

	spi_writecomm(0xCC00);spi_writedata(0x10);

	spi_writecomm(0xB000);spi_writedata(0x00);
	spi_writecomm(0xB001);spi_writedata(0x0d);
	spi_writecomm(0xB002);spi_writedata(0x14);
	spi_writecomm(0xB003);spi_writedata(0x0e);
	spi_writecomm(0xB004);spi_writedata(0x11);
	spi_writecomm(0xB005);spi_writedata(0x07);
	spi_writecomm(0xB006);spi_writedata(0x44);
	spi_writecomm(0xB007);spi_writedata(0x08);
	spi_writecomm(0xB008);spi_writedata(0x08);
	spi_writecomm(0xB009);spi_writedata(0x60);
	spi_writecomm(0xB00A);spi_writedata(0x03);
	spi_writecomm(0xB00B);spi_writedata(0x11);
	spi_writecomm(0xB00C);spi_writedata(0x10);
	spi_writecomm(0xB00D);spi_writedata(0x2d);
	spi_writecomm(0xB00E);spi_writedata(0x34);
	spi_writecomm(0xB00F);spi_writedata(0x1F);

	spi_writecomm(0xB100);spi_writedata(0x00);
	spi_writecomm(0xB101);spi_writedata(0x0c);
	spi_writecomm(0xB102);spi_writedata(0x13);
	spi_writecomm(0xB103);spi_writedata(0x0d);
	spi_writecomm(0xB104);spi_writedata(0x10);
	spi_writecomm(0xB105);spi_writedata(0x05);
	spi_writecomm(0xB106);spi_writedata(0x43);
	spi_writecomm(0xB107);spi_writedata(0x08);
	spi_writecomm(0xB108);spi_writedata(0x07);
	spi_writecomm(0xB109);spi_writedata(0x60);
	spi_writecomm(0xB10A);spi_writedata(0x04);
	spi_writecomm(0xB10B);spi_writedata(0x11);
	spi_writecomm(0xB10C);spi_writedata(0x10);
	spi_writecomm(0xB10D);spi_writedata(0x2d);
	spi_writecomm(0xB10E);spi_writedata(0x34);
	spi_writecomm(0xB10F);spi_writedata(0x1F);

	spi_writecomm(0xFF00);spi_writedata(0x77);
	spi_writecomm(0XFF01);spi_writedata(0x01);
	spi_writecomm(0xFF02);spi_writedata(0x00);
	spi_writecomm(0xFF03);spi_writedata(0x00);
	spi_writecomm(0xFF04);spi_writedata(0x11);

	spi_writecomm(0xB000);spi_writedata(0x67);

	spi_writecomm(0xB100);spi_writedata(0x66);

	spi_writecomm(0xB200);spi_writedata(0x07);

	spi_writecomm(0xB300);spi_writedata(0x80);

	spi_writecomm(0xB500);spi_writedata(0x45);

	spi_writecomm(0xB700);spi_writedata(0x85);

	spi_writecomm(0xB800);spi_writedata(0x21);

	spi_writecomm(0xB900);spi_writedata(0x00);
	spi_writecomm(0xB901);spi_writedata(0x10);

	spi_writecomm(0xC100);spi_writedata(0x78);

	spi_writecomm(0xC200);spi_writedata(0x78);

	spi_writecomm(0xD000);spi_writedata(0x88);

	spi_writecomm(0xE000);spi_writedata(0x00);
	spi_writecomm(0xE001);spi_writedata(0x00);
	spi_writecomm(0xE002);spi_writedata(0x02);

	spi_writecomm(0xE100);spi_writedata(0x08);
	spi_writecomm(0xE101);spi_writedata(0x00);
	spi_writecomm(0xE102);spi_writedata(0x0a);
	spi_writecomm(0xE105);spi_writedata(0x00);
	spi_writecomm(0xE104);spi_writedata(0x07);
	spi_writecomm(0xE105);spi_writedata(0x00);
	spi_writecomm(0xE106);spi_writedata(0x09);
	spi_writecomm(0xE107);spi_writedata(0x00);
	spi_writecomm(0xE108);spi_writedata(0x00);
	spi_writecomm(0xE109);spi_writedata(0x33);
	spi_writecomm(0xE10A);spi_writedata(0x33);

	spi_writecomm(0xE200);spi_writedata(0x00);
	spi_writecomm(0xE201);spi_writedata(0x00);
	spi_writecomm(0xE202);spi_writedata(0x00);
	spi_writecomm(0xE203);spi_writedata(0x00);
	spi_writecomm(0xE204);spi_writedata(0x00);
	spi_writecomm(0xE205);spi_writedata(0x00);
	spi_writecomm(0xE206);spi_writedata(0x00);
	spi_writecomm(0xE207);spi_writedata(0x00);
	spi_writecomm(0xE208);spi_writedata(0x00);
	spi_writecomm(0xE209);spi_writedata(0x00);
	spi_writecomm(0xE20A);spi_writedata(0x00);
	spi_writecomm(0xE20B);spi_writedata(0x00);
	spi_writecomm(0xE20C);spi_writedata(0x00);

	spi_writecomm(0xE300);spi_writedata(0x00);
	spi_writecomm(0xE301);spi_writedata(0x00);
	spi_writecomm(0xE302);spi_writedata(0x33);
	spi_writecomm(0xE303);spi_writedata(0x33);

	spi_writecomm(0xE400);spi_writedata(0x44);
	spi_writecomm(0xE401);spi_writedata(0x44);

	spi_writecomm(0xE500);spi_writedata(0x0e);
	spi_writecomm(0xE501);spi_writedata(0x2d);
	spi_writecomm(0xE502);spi_writedata(0xa0);
	spi_writecomm(0xE503);spi_writedata(0xA0);
	spi_writecomm(0xE504);spi_writedata(0x10);
	spi_writecomm(0xE505);spi_writedata(0x2d);
	spi_writecomm(0xE506);spi_writedata(0xa0);
	spi_writecomm(0xE507);spi_writedata(0xA0);
	spi_writecomm(0xE508);spi_writedata(0x0a);
	spi_writecomm(0xE509);spi_writedata(0x2d);
	spi_writecomm(0xE50A);spi_writedata(0xa0);
	spi_writecomm(0xE50B);spi_writedata(0xA0);
	spi_writecomm(0xE50C);spi_writedata(0x0c);
	spi_writecomm(0xE50D);spi_writedata(0x2d);
	spi_writecomm(0xE50E);spi_writedata(0xa0);
	spi_writecomm(0xE50F);spi_writedata(0xA0);

	spi_writecomm(0xE600);spi_writedata(0x00);
	spi_writecomm(0xE601);spi_writedata(0x00);
	spi_writecomm(0xE602);spi_writedata(0x33);
	spi_writecomm(0xE603);spi_writedata(0x33);

	spi_writecomm(0xE700);spi_writedata(0x44);
	spi_writecomm(0xE701);spi_writedata(0x44);

	spi_writecomm(0xE800);spi_writedata(0x0d);
	spi_writecomm(0xE801);spi_writedata(0x2d);
	spi_writecomm(0xE802);spi_writedata(0xa0);
	spi_writecomm(0xE803);spi_writedata(0xA0);
	spi_writecomm(0xE804);spi_writedata(0x0f);
	spi_writecomm(0xE805);spi_writedata(0x2d);
	spi_writecomm(0xE806);spi_writedata(0xa0);
	spi_writecomm(0xE807);spi_writedata(0xA0);
	spi_writecomm(0xE808);spi_writedata(0x09);
	spi_writecomm(0xE809);spi_writedata(0x2d);
	spi_writecomm(0xE80A);spi_writedata(0xa0);
	spi_writecomm(0xE80B);spi_writedata(0xA0);
	spi_writecomm(0xE80C);spi_writedata(0x0b);
	spi_writecomm(0xE80D);spi_writedata(0x2d);
	spi_writecomm(0xE80E);spi_writedata(0xa0);
	spi_writecomm(0xE80F);spi_writedata(0xA0);

	spi_writecomm(0xEB00);spi_writedata(0x02);
	spi_writecomm(0xEB01);spi_writedata(0x01);
	spi_writecomm(0xEB02);spi_writedata(0xe4);
	spi_writecomm(0xEB03);spi_writedata(0xe4);
	spi_writecomm(0xEB04);spi_writedata(0x44);
	spi_writecomm(0xEB05);spi_writedata(0x00);
	spi_writecomm(0xEB06);spi_writedata(0x40);

	spi_writecomm(0xEC00);spi_writedata(0x02);
	spi_writecomm(0xEC01);spi_writedata(0x01);

	spi_writecomm(0xED00);spi_writedata(0xab);
	spi_writecomm(0xED01);spi_writedata(0x89);
	spi_writecomm(0xED02);spi_writedata(0x76);
	spi_writecomm(0xED03);spi_writedata(0x54);
	spi_writecomm(0xED04);spi_writedata(0x01);
	spi_writecomm(0xED05);spi_writedata(0xFF);
	spi_writecomm(0xED06);spi_writedata(0xFF);
	spi_writecomm(0xED07);spi_writedata(0xFF);
	spi_writecomm(0xED08);spi_writedata(0xFF);
	spi_writecomm(0xED09);spi_writedata(0xFF);
	spi_writecomm(0xED0A);spi_writedata(0xFf);
	spi_writecomm(0xED0B);spi_writedata(0x10);
	spi_writecomm(0xED0C);spi_writedata(0x45);
	spi_writecomm(0xED0D);spi_writedata(0x67);
	spi_writecomm(0xED0E);spi_writedata(0x98);
	spi_writecomm(0xED0F);spi_writedata(0xba);

	spi_writecomm(0xFF00);spi_writedata(0x77);
	spi_writecomm(0XFF01);spi_writedata(0x01);
	spi_writecomm(0xFF02);spi_writedata(0x00);
	spi_writecomm(0xFF03);spi_writedata(0x00);
	spi_writecomm(0xFF04);spi_writedata(0x00);

	spi_writecomm(0x3a00);spi_writedata(0x77);
	spi_writecomm(0x3600);spi_writedata(0x00);
	spi_writecomm(0x1100);spi_writedata(0x00);
	msleep(150);
	spi_writecomm(0x2900);spi_writedata(0xFF);
}


static void panel_enable(struct lcd_panel *panel)
{
}

static void panel_disable(struct lcd_panel *panel)
{
}

static struct lcd_panel_ops panel_ops = {
	.enable  = (void*)panel_enable,
	.disable = (void*)panel_disable,
};

static struct fb_videomode panel_modes[] = {
	[0] = {
		.name                   = "fw040",
		.refresh                = 60,
		.xres                   = 480,
		.yres                   = 800,
		.pixclock               = KHZ2PICOS(14000),
		.left_margin            = 42,
		.right_margin           = 16,
		.upper_margin           = 28,
		.lower_margin           = 12,

		.hsync_len              = 2,
		.vsync_len              = 2,
		.sync                   = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
		.vmode                  = FB_VMODE_NONINTERLACED,
		.flag                   = 0,
	},
};

static struct tft_config fw040_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.vsync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

struct lcd_panel lcd_panel = {
	.name = "fw040",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.bpp = 32,
	.width = 480,
	.height = 800,

	.lcd_type = LCD_TYPE_TFT,

	.tft_config = &fw040_cfg,

	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,

	.ops = &panel_ops,
};

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);
	struct board_gpio *vdd_en = &panel->vdd_en;
	struct board_gpio *rst = &panel->rst;
	struct board_gpio *pwm = &panel->pwm;

	if(!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
		gpio_direction_output(vdd_en->gpio, 0);
	}else{
		if(gpio_is_valid(pwm->gpio)){
			gpio_direction_output(pwm->gpio, 1);
			msleep(180);
		}
	gpio_direction_output(vdd_en->gpio, 0);
	gpio_direction_output(rst->gpio, 1);
	msleep(100);

	gpio_direction_output(vdd_en->gpio, 1);
	msleep(180);

	gpio_direction_output(rst->gpio, 0);
	msleep(20);
	gpio_direction_output(rst->gpio, 1);
	msleep(20);

	Initial_IC();
	pinctrl_select_state(p, state->i2c);
	gpio_free(panel->spi.cs);
	}

	panel->power = power;
        return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/**
* @ pannel_fw040_lcd_ops, register to kernel common backlight/lcd.c framworks.
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
		ret = gpio_request_one(panel->vdd_en.gpio, GPIOF_DIR_OUT, "vdd_en");
		if(ret < 0) {
			dev_err(dev, "Failed to request vdd_en pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->rst.gpio, GPIOF_DIR_OUT, "rst");
		if(ret < 0) {
			dev_err(dev, "Failed to request rst pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	panel->pwm.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags);
	if(gpio_is_valid(panel->pwm.gpio)) {
		panel->pwm.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->pwm.gpio, GPIOF_DIR_OUT, "pwm");
		if(ret < 0) {
			dev_err(dev, "Failed to request pwm pin!\n");
			goto err_request_pwm;
		}
	} else {
		dev_warn(dev, "invalid gpio pwm.gpio: %d\n", panel->pwm.gpio);
	}

	panel->spi.cs = of_get_named_gpio_flags(np, "ingenic,lcd-cs-gpio", 0, &flags);
	if(gpio_is_valid(panel->spi.cs)) {
		ret = gpio_request_one(panel->spi.cs, GPIOF_DIR_OUT, "cs");
		if(ret < 0) {
			dev_err(dev, "Failed to request cs pin!\n");
			goto err_request_cs;
		}
	} else {
		dev_warn(dev, "invalid gpio spi.cs: %d\n", panel->spi.cs);
	}

	state->i2c = pinctrl_lookup_state(p, "i2c-func");
	state->sda_high = pinctrl_lookup_state(p, "sda-high");
	state->sda_low = pinctrl_lookup_state(p, "sda-low");
	state->sck_high = pinctrl_lookup_state(p, "sck-high");
	state->sck_low = pinctrl_lookup_state(p, "sck-low");

	return 0;
err_request_cs:
	if(gpio_is_valid(panel->spi.sck))
		gpio_free(panel->spi.sck);
err_request_sck:
	if(gpio_is_valid(panel->spi.sdo))
		gpio_free(panel->spi.sdo);
err_request_sdo:
	if(gpio_is_valid(panel->pwm.gpio))
		gpio_free(panel->pwm.gpio);
err_request_pwm:
	if(gpio_is_valid(panel->rst.gpio))
		gpio_free(panel->rst.gpio);
err_request_rst:
	if(gpio_is_valid(panel->vdd_en.gpio))
		gpio_free(panel->vdd_en.gpio);
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
	/* struct panel_dev *panel; */
	struct backlight_properties props;

	state = kzalloc(sizeof(struct fw040_pinctrl), GFP_KERNEL);
	if(state == NULL) {
		dev_err(&pdev->dev, "Failed to alloc memory!");
		return -ENOMEM;
	}

	memset(&props, 0, sizeof(props));
	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if(panel == NULL) {
		dev_err(&pdev->dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	p = pinctrl_get(&pdev->dev);
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
	{ .compatible = "ingenic,fw040", },
	{},
};

static struct platform_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.driver		= {
		.name	= "fw040",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
