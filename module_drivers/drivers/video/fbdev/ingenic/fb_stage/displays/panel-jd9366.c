/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-jd9366.c
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 * This program is free software, you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
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

static int test = 0;
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
	struct board_gpio rst;
	struct board_gpio lcd_te;
	struct board_gpio lcd_pwm;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct panel_dev *panel;

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)

struct jd9366 {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;

};

static struct dsi_cmd_packet fitipower_jd9366_800_1280_cmd_list1[] =
{
		//Page0
	{0x15,0xE0,0x00},

	//--- PASSWORD  ----//
	{0x15, 0xE1,0x93},
	{0x15, 0xE2,0x65},
	{0x15, 0xE3,0xF8},


	//Page0
	{0x15, 0xE0,0x00},
	//--- Sequence Ctrl  ----//
	{0x15, 0x70,0x10},	//DC0,DC1
	{0x15, 0x71,0x13},	//DC2,DC3
	{0x15, 0x72,0x06},	//DC7

	{0x15, 0x80,0x03},	//0x03:4-Lane；0x02:3-Lane ; 0x01 : 2-lane
	//--- Page4  ----//
	{0x15, 0xE0,0x04},
	{0x15, 0x2D,0x03},
	//--- Page1  ----//
	{0x15, 0xE0,0x01},

	//Set VCOM
	{0x15, 0x00,0x00},
	{0x15, 0x01,0xA0},
	//Set VCOM_Reverse
	{0x15, 0x03,0x00},
	{0x15, 0x04,0xA0},

	//Set Gamma Power, VGMP,VGMN,VGSP,VGSN
	{0x15, 0x17,0x00},
	{0x15, 0x18,0xB1},
	{0x15, 0x19,0x01},
	{0x15, 0x1A,0x00},
	{0x15, 0x1B,0xB1},  //VGMN=0
	{0x15, 0x1C,0x01},
	//Set Gate Power
	{0x15, 0x1F,0x3E},     //VGH_R  = 15V
	{0x15, 0x20,0x2D},     //VGL_R  = -12V
	{0x15, 0x21,0x2D},     //VGL_R2 = -12V
	{0x15, 0x22,0x0E},     //PA[6]=0, PA[5]=0, PA[4]=0, PA[0]=0

	//SETPANEL
	{0x15, 0x37,0x19},	//SS=1,BGR=1

	//SET RGBCYC
	{0x15, 0x38,0x05},	//JDT=101 zigzag inversion
	{0x15, 0x39,0x08},	//RGB_N_EQ1, modify 20140806
	{0x15, 0x3A,0x12},	//RGB_N_EQ2, modify 20140806
	{0x15, 0x3C,0x78},	//SET EQ3 for TE_H
	{0x15, 0x3E,0x80},	//SET CHGEN_OFF, modify 20140806
	{0x15, 0x3F,0x80},	//SET CHGEN_OFF2, modify 20140806 --------------


	//Set TCON
	{0x15, 0x40,0x06},	//RSO=800 RGB
	{0x15, 0x41,0xA0},	//LN=640->1280 line

	//--- power voltage  ----//
	{0x15, 0x55,0x01},	//DCDCM=0001, JD PWR_IC
	{0x15, 0x56,0x01},
	{0x15, 0x57,0x69},
	{0x15, 0x58,0x0A},
	{0x15, 0x59,0x0A},	//VCL = -2.9V
	{0x15, 0x5A,0x28},	//VGH = 19V
	{0x15, 0x5B,0x19},	//VGL = -11V



	//--- Gamma  ----//
	{0x15, 0x5D,0x7C},
	{0x15, 0x5E,0x65},
	{0x15, 0x5F,0x53},
	{0x15, 0x60,0x48},
	{0x15, 0x61,0x43},
	{0x15, 0x62,0x35},
	{0x15, 0x63,0x39},
	{0x15, 0x64,0x23},
	{0x15, 0x65,0x3D},
	{0x15, 0x66,0x3C},
	{0x15, 0x67,0x3D},
	{0x15, 0x68,0x5A},
	{0x15, 0x69,0x46},
	{0x15, 0x6A,0x57},
	{0x15, 0x6B,0x4B},
	{0x15, 0x6C,0x49},
	{0x15, 0x6D,0x2F},
	{0x15, 0x6E,0x03},
	{0x15, 0x6F,0x00},
	{0x15, 0x70,0x7C},
	{0x15, 0x71,0x65},
	{0x15, 0x72,0x53},
	{0x15, 0x73,0x48},
	{0x15, 0x74,0x43},
	{0x15, 0x75,0x35},
	{0x15, 0x76,0x39},
	{0x15, 0x77,0x23},
	{0x15, 0x78,0x3D},
	{0x15, 0x79,0x3C},
	{0x15, 0x7A,0x3D},
	{0x15, 0x7B,0x5A},
	{0x15, 0x7C,0x46},
	{0x15, 0x7D,0x57},
	{0x15, 0x7E,0x4B},
	{0x15, 0x7F,0x49},
	{0x15, 0x80,0x2F},
	{0x15, 0x81,0x03},
	{0x15, 0x82,0x00},


	//Page2, for GIP
	{0x15, 0xE0,0x02},

	//GIP_L Pin mapping
	{0x15, 0x00,0x47},
	{0x15, 0x01,0x47},
	{0x15, 0x02,0x45},
	{0x15, 0x03,0x45},
	{0x15, 0x04,0x4B},
	{0x15, 0x05,0x4B},
	{0x15, 0x06,0x49},
	{0x15, 0x07,0x49},
	{0x15, 0x08,0x41},
	{0x15, 0x09,0x1F},
	{0x15, 0x0A,0x1F},
	{0x15, 0x0B,0x1F},
	{0x15, 0x0C,0x1F},
	{0x15, 0x0D,0x1F},
	{0x15, 0x0E,0x1F},
	{0x15, 0x0F,0x43},
	{0x15, 0x10,0x1F},
	{0x15, 0x11,0x1F},
	{0x15, 0x12,0x1F},
	{0x15, 0x13,0x1F},
	{0x15, 0x14,0x1F},
	{0x15, 0x15,0x1F},

	//GIP_R Pin mapping
	{0x15, 0x16,0x46},
	{0x15, 0x17,0x46},
	{0x15, 0x18,0x44},
	{0x15, 0x19,0x44},
	{0x15, 0x1A,0x4A},
	{0x15, 0x1B,0x4A},
	{0x15, 0x1C,0x48},
	{0x15, 0x1D,0x48},
	{0x15, 0x1E,0x40},
	{0x15, 0x1F,0x1F},
	{0x15, 0x20,0x1F},
	{0x15, 0x21,0x1F},
	{0x15, 0x22,0x1F},
	{0x15, 0x23,0x1F},
	{0x15, 0x24,0x1F},
	{0x15, 0x25,0x42},
	{0x15, 0x26,0x1F},
	{0x15, 0x27,0x1F},
	{0x15, 0x28,0x1F},
	{0x15, 0x29,0x1F},
	{0x15, 0x2A,0x1F},
	{0x15, 0x2B,0x1F},
	//GIP_L_GS Pin mapping
	{0x15, 0x2C,0x11},
	{0x15, 0x2D,0x0F},
	{0x15, 0x2E,0x0D},
	{0x15, 0x2F,0x0B},
	{0x15, 0x30,0x09},
	{0x15, 0x31,0x07},
	{0x15, 0x32,0x05},
	{0x15, 0x33,0x18},
	{0x15, 0x34,0x17},
	{0x15, 0x35,0x1F},
	{0x15, 0x36,0x01},
	{0x15, 0x37,0x1F},
	{0x15, 0x38,0x1F},
	{0x15, 0x39,0x1F},
	{0x15, 0x3A,0x1F},
	{0x15, 0x3B,0x1F},
	{0x15, 0x3C,0x1F},
	{0x15, 0x3D,0x1F},
	{0x15, 0x3E,0x1F},
	{0x15, 0x3F,0x13},
	{0x15, 0x40,0x1F},
	{0x15, 0x41,0x1F},
	//GIP_R_GS Pin mapping
	{0x15, 0x42,0x10},
	{0x15, 0x43,0x0E},
	{0x15, 0x44,0x0C},
	{0x15, 0x45,0x0A},
	{0x15, 0x46,0x08},
	{0x15, 0x47,0x06},
	{0x15, 0x48,0x04},
	{0x15, 0x49,0x18},
	{0x15, 0x4A,0x17},
	{0x15, 0x4B,0x1F},
	{0x15, 0x4C,0x00},
	{0x15, 0x4D,0x1F},
	{0x15, 0x4E,0x1F},
	{0x15, 0x4F,0x1F},
	{0x15, 0x50,0x1F},
	{0x15, 0x51,0x1F},
	{0x15, 0x52,0x1F},
	{0x15, 0x53,0x1F},
	{0x15, 0x54,0x1F},
	{0x15, 0x55,0x12},
	{0x15, 0x56,0x1F},
	{0x15, 0x57,0x1F},

	//GIP Timing
	{0x15, 0x58,0x40},
	{0x15, 0x59,0x00},
	{0x15, 0x5A,0x00},
	{0x15, 0x5B,0x30},
	{0x15, 0x5C,0x03},
	{0x15, 0x5D,0x30},
	{0x15, 0x5E,0x01},
	{0x15, 0x5F,0x02},
	{0x15, 0x60,0x00},
	{0x15, 0x61,0x01},
	{0x15, 0x62,0x02},
	{0x15, 0x63,0x03},
	{0x15, 0x64,0x6B},
	{0x15, 0x65,0x00},
	{0x15, 0x66,0x00},
	{0x15, 0x67,0x73},
	{0x15, 0x68,0x05},
	{0x15, 0x69,0x06},
	{0x15, 0x6A,0x6B},
	{0x15, 0x6B,0x08},
	{0x15, 0x6C,0x00},
	{0x15, 0x6D,0x04},
	{0x15, 0x6E,0x04},
	{0x15, 0x6F,0x88},
	{0x15, 0x70,0x00},
	{0x15, 0x71,0x00},
	{0x15, 0x72,0x06},
	{0x15, 0x73,0x7B},
	{0x15, 0x74,0x00},
	{0x15, 0x75,0x07},
	{0x15, 0x76,0x00},
	{0x15, 0x77,0x5D},
	{0x15, 0x78,0x17},
	{0x15, 0x79,0x1F},
	{0x15, 0x7A,0x00},
	{0x15, 0x7B,0x00},
	{0x15, 0x7C,0x00},
	{0x15, 0x7D,0x03},
	{0x15, 0x7E,0x7B},


	//Page1
	{0x15, 0xE0,0x01},
	{0x15, 0x0E,0x01},	//LEDON output VCSW2


	//Page3
	{0x15, 0xE0,0x03},
	{0x15, 0x98,0x2F},	//From 2E to 2F, LED_VOL

	//Page4
	{0x15, 0xE0,0x04},
	{0x15, 0x09,0x10},
	{0x15, 0x2B,0x2B},
	{0x15, 0x2E,0x44},

	//Page0
	{0x15, 0xE0,0x00},
	{0x15, 0xE6,0x02},
	{0x15, 0xE7,0x02},

	{0x15,0x51,0xFF},
	{0x15,0x53,0x2C},
	{0x15,0x55,0x01},

};


static void panel_dev_sleep_in(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_sleep_out(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};
	/* struct dsi_cmd_packet data_to_send1 = {0x15, 0xc2, 0x20};//color bar test mode. */
//	while (1) {
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
	/* ops->cmd_write(lcd_to_master(lcd), data_to_send1); */
//	}
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);//color bar turn off 0x29 command.
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};
	/* struct dsi_cmd_packet data_to_send1 = {0x15, 0x51, 250}; */

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
	/* ops->cmd_write(lcd_to_master(lcd), data_to_send1); */
}

static void panel_dev_display_off(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static int panel_dev_rdid(struct panel_dev *lcd, unsigned char *buf, int count)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);

	struct dsi_cmd_packet data_to_send0 = {0x14, 0xDA, 0x0};
	struct dsi_cmd_packet data_to_send1 = {0x14, 0xDB, 0x0};
	struct dsi_cmd_packet data_to_send2 = {0x14, 0xDC, 0x0};
	int ret = 0;

	ret = ops->cmd_read(lcd_to_master(lcd), data_to_send0, &buf[0], 1);
	ret = ops->cmd_read(lcd_to_master(lcd), data_to_send1, &buf[1], 1);
	ret = ops->cmd_read(lcd_to_master(lcd), data_to_send2, &buf[2], 1);
	printk("----------%s, %d id[0]: %x id[1]: %x, id[2]: %x, id[3]: %x\n", __func__, __LINE__, buf[0], buf[1], buf[2], buf[3]);

	return ret;
}

static void panel_dev_panel_init(struct panel_dev *lcd)
{
	int  i;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	char id[4];

	memset(id, 0, 4);

	panel_dev_rdid(lcd, id, 4);

	for (i = 0; i < ARRAY_SIZE(fitipower_jd9366_800_1280_cmd_list1); i++)
	{
		ops->cmd_write(dsi,  fitipower_jd9366_800_1280_cmd_list1[i]);
	}
}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}
static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct jd9366 *lcd = dev_get_drvdata(&dsim_dev->dev);

	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
	if(test != 0)
		return;
	test++;

	panel_dev_panel_init(panel);
	panel_dev_sleep_out(panel);
	msleep(120);
	panel_dev_display_on(panel);
	msleep(5);
	/* dump_dsi_reg(dsi); */
	lcd->power = FB_BLANK_UNBLANK;
}
static void panel_dev_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct board_gpio *vdd_en = &panel->vdd_en;
	struct board_gpio *rst = &panel->rst;
	struct board_gpio *lcd_te = &panel->lcd_te;
	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

	if(test != 0)
		return;

	gpio_direction_output(vdd_en->gpio, vdd_en->active_level);

	if (gpio_is_valid(lcd_te->gpio)) {
		gpio_direction_input(lcd_te->gpio);
	}
	msleep(50);
	gpio_direction_output(rst->gpio, 0);
	msleep(10);
	gpio_direction_output(rst->gpio, 1);
	msleep(120);

	panel->power = power;
}

static struct fb_videomode panel_modes = {
	.name = "fitipower_jd9366-lcd",
	.xres = 800,
	.yres = 1280,

	.refresh = 60,
	.left_margin = 20,//hbp
	.right_margin = 32,//hfp
	.hsync_len =20,//hsync

	.upper_margin = 4,//vbp
	.lower_margin = 8,//vfp
	.vsync_len = 4, //vsync

	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 4,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	/* .video_config.video_mode = VIDEO_NON_BURST_WITH_SYNC_PULSES, */
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL3_DIV2, // byte_clock *3/2.

	.dsi_config.max_lanes = 4,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	//.dsi_config.max_hs_to_lp_cycles = 200,
	//.dsi_config.max_lp_to_hs_cycles = 80,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
//	.max_bps = 650,  // 650 Mbps
	.bpp_info = 24,
};
static struct tft_config kd050hdfia019_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};


struct lcd_panel lcd_panel = {
	.num_modes = 1,
	.modes = &panel_modes,
	.dsi_pdata = &jzdsi_pdata,

	.lcd_type = LCD_TYPE_MIPI_TFT,
	.tft_config = &kd050hdfia019_cfg,
	.bpp = 24,
	.width = 107,
	.height = 172,
	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,


};

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
        return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/*
* @ pannel_jd9366_lcd_ops, register to kernel common backlight/lcd.c framworks.
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


	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if(gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		gpio_direction_output(panel->rst.gpio,0);
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
		panel->vdd_en.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		gpio_direction_output(panel->vdd_en.gpio, !panel->vdd_en.active_level);
		ret = gpio_request_one(panel->vdd_en.gpio, GPIOF_DIR_OUT, "vdd_en");
		if(ret < 0) {
			dev_err(dev, "Failed to request vdd_en pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

	/* panel->lcd_pwm.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags); */
	/* if(gpio_is_valid(panel->lcd_pwm.gpio)) { */
	/* 	panel->lcd_pwm.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1; */
	/* 	ret = gpio_request_one(panel->lcd_pwm.gpio, GPIOF_DIR_OUT, "lcd-pwm"); */
	/* 	ret = gpio_direction_output(panel->lcd_pwm.gpio,0); */
	/* 	if(ret < 0) { */
	/* 		dev_err(dev, "Failed to request lcd-pwm pin!\n"); */
	/* 		return ret; */
	/* 	} */
	/* } else { */
	/* 	dev_warn(dev, "invalid gpio lcd-pwm.gpio: %d\n", panel->lcd_pwm.gpio); */
	/* } */


	return 0;
err_request_lcd_te:
	if(gpio_is_valid(panel->rst.gpio))
		gpio_free(panel->rst.gpio);
err_request_rst:
	if(gpio_is_valid(panel->vdd_en.gpio))
		gpio_free(panel->vdd_en.gpio);
	return ret;
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct jd9366 *lcd;
	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct jd9366), GFP_KERNEL);
	if (!lcd)
	{
		dev_err(&dsim_dev->dev, "failed to allocate fitipower_jd9366 structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("fitipower_jd9366", lcd->dev, lcd,
	                              &panel_lcd_ops);
	if (IS_ERR(lcd->ld))
	{
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);


	dev_dbg(lcd->dev, "probed fitipower_jd9366 panel driver.\n");


	panel->dsim_dev = dsim_dev;


	return 0;

}

static int panel_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct board_gpio *vdd_en = &panel->vdd_en;

	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
	panel_dev_display_off(panel);
	panel_dev_sleep_in(panel);
	gpio_direction_output(vdd_en->gpio, !vdd_en->active_level);

	return 0;
}

static int panel_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct board_gpio *vdd_en = &panel->vdd_en;
	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

	gpio_direction_output(vdd_en->gpio, vdd_en->active_level);

	return 0;
}

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "fitipower_jd9366-lcd",
	.id = -1,

	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
//	.suspend = panel_suspend,
	.resume = panel_resume,
};


struct mipi_dsim_lcd_device panel_dev_device={
	.name		= "fitipower_jd9366-lcd",
	.id = 0,
};

/*
* @panel_probe

* 	1. Register to ingenicfb.
* 	2. Register to lcd.
* 	3. Register to backlight if possible.

* @pdev

* @Return -
*/
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;

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

	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to register lcd panel!\n");
		goto err_of_parse;
	}

	return 0;

err_of_parse:
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,jd9366", },
	{},
};

static struct platform_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.driver		= {
		.name	= "jd9366",
		.of_match_table = panel_of_match,
	},
};

module_platform_driver(panel_driver);
