/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-G104X1L04.c
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

struct panel_dev {
	/* ingenic frame buffer */
	struct i2c_client *client;
	struct device *dev;
	struct lcd_panel *panel;

	/* common lcd framework */
	struct lcd_device *lcd;
	int power;

	int i2c_id;
	struct board_gpio reset;
	struct board_gpio lcd_en;
	struct board_gpio bl_en;

	char *panel_name;
};

#define FPS    60
#define HACT   1024

#define VACT   768

#define HFP    160
#define HBP    140
#define HS     20

#define VFP    20
#define VBP    10
#define VS     8


////////////////////////////LT9211 begin/////////////////////////////////////////////
//#include "lt9211.h"
/******************* MIPI Input Config ********************/
#define INPUT_PORTA
//#define INPUT_PORTB
#define Lane_Num 2
/******************* MIPI Input Config ********************/

/******************* Lvds Output Config ********************/
enum LT9211_LVDSPORT_ENUM
{
    LVDS_1PORT = 0,
    LVDS_2PORT = 1
};
#define LVDS_PORTNUM LVDS_1PORT

enum LT9211_LVDSMODE_ENUM
{
    DE_MODE = 0,
    SYNC_MODE = 1
};
#define LVDS_MODE DE_MODE

enum LT9211_LVDSDATAFORMAT_ENUM
{
    VESA = 0,
    JEIDA = 1
};
#define LVDS_DATAFORMAT VESA

enum LT9211_LVDSCOLORDEPTH_ENUM
{
    DEPTH_6BIT = 0,
    DEPTH_8BIT = 1
};
#define LVDS_COLORDEPTH DEPTH_8BIT

//#define LVDS_2PORT_SWAP
/******************* Lvds Output Config ********************/

struct video_timing {
unsigned short hfp;
unsigned short hs;
unsigned short hbp;
unsigned short hact;
unsigned short htotal;
unsigned short vfp;
unsigned short vs;
unsigned short vbp;
unsigned short vact;
unsigned short vtotal;
unsigned int pclk_khz;
};

//#include "lt9211.c"
static unsigned char mipi_fmt = 0;
struct video_timing *pVideo_Format;
//#define DEBUG

#ifdef DEBUG
#define debug(info, ...) printk(info, ##__VA_ARGS__)
#else
#define debug(info, ...)
#endif

#define HTOTAL (HFP + HACT + HBP + HS)
#define VTOTAL (VFP + VACT + VBP + VS)

#define PIXCLK (HTOTAL * VTOTAL * FPS / 1000)

//hfp, hs, hbp, hact, htotal, vfp, vs, vbp, vact, vtotal, pixclk
//struct video_timing video_1920x1080_60Hz   ={HFP, HS, HBP, HACT, HTOTAL, VFP, VS, VBP, VACT, VTOTAL, PIXCLK};
struct video_timing video_1024x768_60Hz = {HFP, HS, HBP, HACT, HTOTAL, VFP, VS, VBP, VACT, VTOTAL, PIXCLK};

static unsigned int i2c_write(struct i2c_client *client,unsigned char addr,unsigned char value);
static unsigned char i2c_read(struct i2c_client *client, u8 addr);

#ifdef DEBUG
static void LT9211_ChipID(struct i2c_client *client)
{
	i2c_write(client, 0xff,0x81);//register bank
	printk("LT9211 Chip ID:%x,",i2c_read(client, 0x00));
	printk("%02x, ",i2c_read(client, 0x01));
	printk("%02x\n",i2c_read(client, 0x02));
}
#endif

/** lvds rx logic rst **/
static void lt9211_mipirx_logic_rst(struct i2c_client *client)
{
	i2c_write(client, 0xff,0x81);
	i2c_write(client, 0x0a,0xc0);
	i2c_write(client, 0x20,0xbf);  //mipi rx div logic reset,for portb input
	mdelay(10);
	i2c_write(client, 0x0a,0xc1);
	i2c_write(client, 0x20,0xff);
}

static void LT9211_SystemInt(struct i2c_client *client)
{
	/* system clock init */
	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x01,0x18);

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x06,0x61);
	i2c_write(client, 0x07,0xa8); //fm for sys_clk

	i2c_write(client, 0xff,0x87);
	i2c_write(client, 0x14,0x08); //default value
	i2c_write(client, 0x15,0x00); //default value
	i2c_write(client, 0x18,0x0f);
	i2c_write(client, 0x22,0x08); //default value
	i2c_write(client, 0x23,0x00); //default value
	i2c_write(client, 0x26,0x0f);

//	i2c_write(client, 0xff,0x81);
//	i2c_write(client, 0x0B,0xFE); //rpt reset
}

static void LT9211_MipiRxPhy(struct i2c_client *client)
{
#ifdef INPUT_PORTA
	debug("Port A PHY Config\n");
	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x02,0x44);  //Port A MIPI mode enable
	i2c_write(client, 0x04,0xa0);  //select port A clk as byteclk
	i2c_write(client, 0x05,0x22);  //port A CLK lane swap
	i2c_write(client, 0x07,0x9f);  //port A clk enable
	i2c_write(client, 0x08,0xfc);  //port A clk enable
	i2c_write(client, 0x09,0x01);  //port A P/N swap
	i2c_write(client, 0x17,0x0c);

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x33,0x1b); //port a lane swap	1b:no swap
#endif

#ifdef INPUT_PORTB
	debug("Port B PHY Config\n");
	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x02,0x44);   //Port A/B MIPI mode enable
	i2c_write(client, 0x04,0xa1);  //select port A clk as byteclk
	i2c_write(client, 0x05,0x26);  //port A CLK lane swap
	i2c_write(client, 0x0d,0x26);   //port B CLK lane swap
	i2c_write(client, 0x07,0x9f);   //port A clk enable  (??Portb?,porta?lane0 clk???)
	i2c_write(client, 0x0f,0x9f);   //port B clk enable
	i2c_write(client, 0x10,0xfc);   //select port B clk as byteclk
	i2c_write(client, 0x11,0x01);   //port B P/N swap
	i2c_write(client, 0x17,0x0c);
	i2c_write(client, 0x1d,0x0c);

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x34,0x1b);   //Port B Lane swap
#endif

	i2c_write(client, 0xff,0x81);
	i2c_write(client, 0x20,0x7f);
	i2c_write(client, 0x20,0xff);  //mlrx calib reset
}

static void LT9211_MipiRxDigital(struct i2c_client *client)
{
	i2c_write(client, 0xff,0x86);
#ifdef INPUT_PORTA
	i2c_write(client, 0x30,0x85);      //mipirx input port sel
#endif

#ifdef INPUT_PORTB
	i2c_write(client, 0x30,0x8f);      //mipirx input port sel
#endif

	i2c_write(client, 0xff,0x85);
	i2c_write(client, 0x88,0x40);       //select mipi in lvds out

#ifdef MIPI_CSI
	debug("Set to CSI Mode\n");
	i2c_write(client, 0xff,0xd0);  //CSI_EN
	i2c_write(client, 0x04,0x10);
	i2c_write(client, 0x21,0xc6);  //CSI_SEL
#else
	debug("Set to DSI Mode\n");
#endif

	i2c_write(client, 0xff,0xd0);
	i2c_write(client, 0x00,0x02);        //4Lane:0x00, 2Lane:0x02, 1Lane:0x01
	i2c_write(client, 0x02,0x05);        //settle
}

static void LT9211_SetVideoTiming(struct i2c_client *client, struct video_timing *video_format)
{
	mdelay(100);
	i2c_write(client, 0xff,0xd0);
	i2c_write(client, 0x0d,(unsigned char)(video_format->vtotal>>8)); //vtotal[15:8]
	i2c_write(client, 0x0e,(unsigned char)(video_format->vtotal)); //vtotal[7:0]
	i2c_write(client, 0x0f,(unsigned char)(video_format->vact>>8)); //vactive[15:8]
	i2c_write(client, 0x10,(unsigned char)(video_format->vact)); //vactive[7:0]
	i2c_write(client, 0x15,(unsigned char)(video_format->vs)); //vs[7:0]
	i2c_write(client, 0x17,(unsigned char)(video_format->vfp>>8)); //vfp[15:8]
	i2c_write(client, 0x18,(unsigned char)(video_format->vfp)); //vfp[7:0]

	i2c_write(client, 0x11,(unsigned char)(video_format->htotal>>8)); //htotal[15:8]
	i2c_write(client, 0x12,(unsigned char)(video_format->htotal)); //htotal[7:0]
	i2c_write(client, 0x13,(unsigned char)(video_format->hact>>8)); //hactive[15:8]
	i2c_write(client, 0x14,(unsigned char)(video_format->hact)); //hactive[7:0]
	i2c_write(client, 0x16,(unsigned char)(video_format->hs)); //hs[7:0]
	i2c_write(client, 0x19,(unsigned char)(video_format->hfp>>8)); //hfp[15:8]
	i2c_write(client, 0x1a,(unsigned char)(video_format->hfp)); //hfp[7:0]
}

#ifdef DEBUG
static void LT9211_debug(struct i2c_client *client)
{
	int i;
	unsigned char start = 0x80;
	i2c_write(client, 0xff,0xd0);
	for(i=0;i<30;i++)
		printk("read 0x%02x = 0x%02x\n", start+i, i2c_read(client, start+i));
}
#endif

static int LT9211_TimingSet(struct i2c_client *client)
{
	unsigned short hact ;
	unsigned short vact ;
	unsigned char pa_lpn = 0;

	lt9211_mipirx_logic_rst(client);
	mdelay(100);

	i2c_write(client, 0xff,0xd0);
	hact = (i2c_read(client, 0x82)<<8) + i2c_read(client, 0x83) ;
	mipi_fmt = (i2c_read(client, 0x84) & 0x0f);
	vact = (i2c_read(client, 0x85)<<8) +i2c_read(client, 0x86);
	pa_lpn = i2c_read(client, 0x9c);
#ifdef DEBUG
	LT9211_debug(client);
#endif

	if(mipi_fmt == 0x03) {
		debug("Input MIPI FMT: CSI_YUV422_16\n");
		hact = hact / 2;
	} else if(mipi_fmt == 0x0a) {
		debug("Input MIPI FMT: RGB888\n");
		hact = hact / 3;
	}

	debug("\033[33m hact = %d |\033[0m\n", hact);
	debug("\033[33m vact = %d |\033[0m\n", vact);

	debug("fmt = %x\n", mipi_fmt);
	debug("pa_lpn = %x\n", pa_lpn);

	mdelay(100);
	pVideo_Format = &video_1024x768_60Hz;
	LT9211_SetVideoTiming(client, &video_1024x768_60Hz);

	return 0;
}

static void LT9211_DesscPll(struct i2c_client *client)
{
	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x2d,0x48);

	if(pVideo_Format->pclk_khz < 44000) {
		i2c_write(client, 0x35,0x83);
	} else if(pVideo_Format->pclk_khz < 88000) {
		i2c_write(client, 0x35,0x82);
	} else if(pVideo_Format->pclk_khz < 176000) {
		i2c_write(client, 0x35,0x81);
	}
}

/*
static int LT9211_MipiPcr(struct i2c_client *client)
{
	unsigned char loopx;
	i2c_write(client, 0xff,0xd0);
	i2c_write(client, 0x26,0x17);
	i2c_write(client, 0x27,0xC3);
	i2c_write(client, 0x2d,0x30);  //PCR M overflow limit setting.
	i2c_write(client, 0x31,0x10);  //PCR M underflow limit setting.
	i2c_write(client, 0x23,0x20);

	i2c_write(client, 0x38,0x02);
	i2c_write(client, 0x39,0x10);
	i2c_write(client, 0x3a,0x20);
	i2c_write(client, 0x3b,0x60);
	i2c_write(client, 0x3f,0x04);
	i2c_write(client, 0x40,0x08);
	i2c_write(client, 0x41,0x10);

	i2c_write(client, 0xff,0x81);
	i2c_write(client, 0x0B,0xEE);
	i2c_write(client, 0x0B,0xFE);

	for(loopx = 0; loopx < 5; loopx++) //Check pcr_stable
	{
		mdelay(200);
		i2c_write(client, 0xff,0xd0);
		if(i2c_read(client, 0x87)&0x08) {
			debug("\033[32mLT9211 pcr stable\033[0m\n");
			return 0;
		}
	}
	printk("LT9211 pcr unstable!!!!\n");
	return -1;
}
*/

static int  LT9211_MipiPcr(struct i2c_client *client)
{
	u8 loopx;
	u8 pcr_m;

    i2c_write(client, 0xff,0xd0);
    i2c_write(client, 0x0c,0x60);  //fifo position
	i2c_write(client, 0x1c,0x60);  //fifo position
	i2c_write(client, 0x24,0x70);  //pcr mode( de hs vs)

	i2c_write(client, 0x2d,0x30); //M up limit
	i2c_write(client, 0x31,0x0a); //M down limit

	/*stage1 hs mode*/
	i2c_write(client, 0x25,0xf0);  //line limit
	i2c_write(client, 0x2a,0x30);  //step in limit
	i2c_write(client, 0x21,0x4f);  //hs_step
	i2c_write(client, 0x22,0x00);

	/*stage2 hs mode*/
	i2c_write(client, 0x1e,0x01);  //RGD_DIFF_SND[7:4],RGD_DIFF_FST[3:0]
	i2c_write(client, 0x23,0x80);  //hs_step
    /*stage2 de mode*/
	i2c_write(client, 0x0a,0x02); //de adjust pre line
	i2c_write(client, 0x38,0x02); //de_threshold 1
	i2c_write(client, 0x39,0x04); //de_threshold 2
	i2c_write(client, 0x3a,0x08); //de_threshold 3
	i2c_write(client, 0x3b,0x10); //de_threshold 4

	i2c_write(client, 0x3f,0x04); //de_step 1
	i2c_write(client, 0x40,0x08); //de_step 2
	i2c_write(client, 0x41,0x10); //de_step 3
	i2c_write(client, 0x42,0x20); //de_step 4

	i2c_write(client, 0x2b,0xa0); //stable out
	//Timer0_Delay1ms(100);
    i2c_write(client, 0xff,0xd0);   //enable HW pcr_m
	pcr_m = i2c_read(client, 0x26);
	pcr_m &= 0x7f;
	i2c_write(client, 0x26,pcr_m);
	i2c_write(client, 0x27,0x0f);

	i2c_write(client, 0xff,0x81);  //pcr reset
	i2c_write(client, 0x20,0xbf); // mipi portB div issue
	i2c_write(client, 0x20,0xff);
	mdelay(5);
	i2c_write(client, 0x0B,0x6F);
	i2c_write(client, 0x0B,0xFF);


	mdelay(800);//800->120
	for(loopx = 0; loopx < 10; loopx++) //Check pcr_stable 10
	{
		mdelay(200);
		i2c_write(client, 0xff,0xd0);
		if(i2c_read(client, 0x87)&0x08)
		{
			printk("\r\nLT9211 pcr stable");
			i2c_write(client, 0xff,0xd0);
			printk("LT9211 pcr_stable_M=%x\n",(i2c_read(client, 0x94)&0x7F));//打印M值
			return 0;
		}
	}


	printk("\r\nLT9211 pcr unstable!!!!");
	i2c_write(client, 0xff,0xd0);
	printk("LT9211 pcr_stable_M=%x\n",(i2c_read(client, 0x94)&0x7F));//打印M值
	return -1;
}

static void LT9211_CSC(struct i2c_client *client)
{
	//yuv422 to rgb888
	if(mipi_fmt == 0x03) {
		debug("csc:yuv422 to rgb888\n");
		i2c_write(client, 0xff,0xf9);
		i2c_write(client, 0x90,0x03);
		i2c_write(client, 0x91,0x03);
	}
}

static void LT9211_TxPhy(struct i2c_client *client)
{
	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x62,0x00); //ttl output disable
	if( LVDS_PORTNUM == LVDS_2PORT ) {
		i2c_write(client, 0x3b,0xb8);
	} else {
		i2c_write(client, 0x3b,0x38);  //dual-port lvds tx phy
	}
	i2c_write(client, 0x3e,0x92);
	i2c_write(client, 0x3f,0x48);
	i2c_write(client, 0x40,0x31);
	i2c_write(client, 0x43,0x80);
	i2c_write(client, 0x44,0x00);
	i2c_write(client, 0x45,0x00);
	i2c_write(client, 0x49,0x00);
	i2c_write(client, 0x4a,0x01);
	i2c_write(client, 0x4e,0x00);
	i2c_write(client, 0x4f,0x00);
	i2c_write(client, 0x50,0x00);
	i2c_write(client, 0x53,0x00);
	i2c_write(client, 0x54,0x01);

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x46,0x10);
#ifdef LVDS_2PORT_SWAP
	debug("LVDS Output Port Swap!\n");
	i2c_write(client, 0x46,0x40);
#endif

	i2c_write(client, 0xff,0x81);
	i2c_write(client, 0x20,0x7b);
	i2c_write(client, 0x20,0xff); //mlrx mltx calib reset
}

static void LT9211_TxDigital(struct i2c_client *client)
{
	debug("LT9211 LVDS_OUTPUT_MODE: \n");
	i2c_write(client, 0xff,0x85); /* lvds tx controller */
	i2c_write(client, 0x59,0x40);
	if( LVDS_DATAFORMAT == VESA ) {
		debug("Data Format: VESA\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) & 0x7f));
	} else if( LVDS_DATAFORMAT == JEIDA ) {
		debug("Data Format: JEIDA\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) | 0x80));
	}
	if( LVDS_COLORDEPTH == DEPTH_6BIT ) {
		debug("ColorDepth: 6Bit\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) & 0xef));
	} else if( LVDS_COLORDEPTH == DEPTH_8BIT ) {
		debug("ColorDepth: 8Bit\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) | 0x10));
	}
	if( LVDS_MODE == SYNC_MODE ) {
		debug("LVDS_MODE: Sync Mode\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) & 0xdf));
	} else if( LVDS_MODE == DE_MODE ) {
		debug("LVDS_MODE: De Mode\n");
		i2c_write(client, 0x59, (i2c_read(client, 0x59) | 0x20));
	}

	i2c_write(client, 0x5a,0xaa);
	i2c_write(client, 0x5b,0xaa);
	if( LVDS_PORTNUM == LVDS_2PORT ) {
		debug("LVDS Output Port Num: 2Port\n");
		i2c_write(client, 0x5c,0x01);	//lvdstx port sel 01:dual;00:single
	} else {
		debug("LVDS Output Port Num: 1Port\n");
		i2c_write(client, 0x5c,0x00);
	}
//	i2c_write(client, 0xa1,0x77);
	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x40,0x40); //tx_src_sel
	/*port src sel*/
	i2c_write(client, 0x41,0x34);
	i2c_write(client, 0x42,0x10);
	i2c_write(client, 0x43,0x23); //pt0_tx_src_sel
	i2c_write(client, 0x44,0x41);
	i2c_write(client, 0x45,0x02); //pt1_tx_src_scl
}

static void LT9211_Txpll(struct i2c_client *client)
{
	unsigned char loopx;

	i2c_write(client, 0xff,0x82);
	i2c_write(client, 0x36,0x01); //b7:txpll_pd
	if( LVDS_PORTNUM == LVDS_1PORT ) {
		i2c_write(client, 0x37,0x29);
	} else {
		i2c_write(client, 0x37,0x2a);
	}
	i2c_write(client, 0x38,0x06);
	i2c_write(client, 0x39,0x30);
	i2c_write(client, 0x3a,0x8e);
	i2c_write(client, 0xff,0x87);
	i2c_write(client, 0x37,0x14);
	i2c_write(client, 0x13,0x00);
	i2c_write(client, 0x13,0x80);
	mdelay(100);
	for(loopx = 0; loopx < 10; loopx++) //Check Tx PLL cal
	{
		i2c_write(client, 0xff,0x87);
		if(i2c_read(client, 0x1f)& 0x80) {
			if(i2c_read(client, 0x20)& 0x80) {
				debug("LT9211 tx pll lock\n");
			} else {
				printk("LT9211 tx pll unlocked\n");
			}
			debug("LT9211 tx pll cal done\n");
			break;
		} else {
			printk("LT9211 tx pll unlocked\n");
		}
	}
}

#ifdef DEBUG
static void LT9211_LvdsClkDebug(struct i2c_client *client)
{
	unsigned int fm_value;

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x00,0x12);
	mdelay(100);
	fm_value = 0;
	fm_value = (i2c_read(client, 0x08) &(0x0f));
	fm_value = (fm_value<<8) ;
	fm_value = fm_value + i2c_read(client, 0x09);
	fm_value = (fm_value<<8) ;
	fm_value = fm_value + i2c_read(client, 0x0a);

	printk("\033[33m lvds pixclk = %d |\033[0m\n", fm_value);
}

static void LT9211_MipiByteClkDebug(struct i2c_client *client)
{
	unsigned int fm_value;

	i2c_write(client, 0xff,0x86);
#ifdef INPUT_PORTA
	i2c_write(client, 0x00,0x01);
#endif
#ifdef INPUT_PORTB
	i2c_write(client, 0x00,0x02);
#endif
	mdelay(100);
	fm_value = 0;
	fm_value = (i2c_read(client, 0x08) &(0x0f));
	fm_value = (fm_value<<8) ;
	fm_value = fm_value + i2c_read(client, 0x09);
	fm_value = (fm_value<<8) ;
	fm_value = fm_value + i2c_read(client, 0x0a);
	printk("\033[33m mipi byteclk = %d |\033[0m\n", fm_value);
}

static void LT9211_VideoCheckDebug(struct i2c_client *client)
{
	unsigned char sync_polarity;
	unsigned short hact, vact;
	unsigned short hs, vs;
	unsigned short hbp, vbp;
	unsigned short htotal, vtotal;
	unsigned short hfp, vfp;

	i2c_write(client, 0xff,0x86);
	i2c_write(client, 0x20,0x00);

	sync_polarity = i2c_read(client, 0x70);
	vs = i2c_read(client, 0x71);

	hs = i2c_read(client, 0x72);
	hs = (hs<<8) + i2c_read(client, 0x73);

	vbp = i2c_read(client, 0x74);
	vfp = i2c_read(client, 0x75);

	hbp = i2c_read(client, 0x76);
	hbp = (hbp<<8) + i2c_read(client, 0x77);

	hfp = i2c_read(client, 0x78);
	hfp = (hfp<<8) + i2c_read(client, 0x79);

	vtotal = i2c_read(client, 0x7A);
	vtotal = (vtotal<<8) + i2c_read(client, 0x7B);

	htotal = i2c_read(client, 0x7C);
	htotal = (htotal<<8) + i2c_read(client, 0x7D);

	vact = i2c_read(client, 0x7E);
	vact = (vact<<8)+ i2c_read(client, 0x7F);

	hact = i2c_read(client, 0x80);
	hact = (hact<<8) + i2c_read(client, 0x81);

	printk("sync_polarity = %x\n", sync_polarity);

	printk("\033[33m hfp = %d , hs = %d , hbp = %d , hact = %d , htotal = %d\033[0m\n", hfp, hs, hbp, hact, htotal);

	printk("\033[33m vfp = %d , vs = %d , vbp = %d , vact = %d , vtotal = %d\033[0m\n", vfp, vs, vbp, vact, vtotal);
}
#endif

int LT9211_MIPI2LVDS_Config(struct i2c_client *client)
{
	int retry = 5;
	int ret = 0;
	debug("*************LT9211 MIPI2LVDS Config*************\n");
#ifdef DEBUG
	LT9211_ChipID(client);
#endif
	LT9211_SystemInt(client);
	LT9211_MipiRxPhy(client);
	LT9211_MipiRxDigital(client);

	while(retry--)
		if ((ret = LT9211_TimingSet(client)) == 0) break;

	if( pVideo_Format != NULL )
	{
#ifdef DEBUG
		LT9211_MipiByteClkDebug(client);
#endif
		LT9211_DesscPll(client);

		retry = 5;
		while(retry--)
			if ((ret = LT9211_MipiPcr(client)) == 0) break;

		LT9211_CSC(client);
		/********LVDS OUTPUT CONFIG********/
		LT9211_TxPhy(client);
		LT9211_TxDigital(client);
		LT9211_Txpll(client);

#ifdef DEBUG
//		 i2c_write(client, 0xff,0x85);
//		 i2c_write(client, 0x88,0xC0); // debug: colorbar
//		 i2c_write(client, 0xA1,0x04); // single color 0x1: blue, 0x2: green, 0x4: red.

		LT9211_LvdsClkDebug(client);
		LT9211_VideoCheckDebug(client);
#endif
	}
	return ret;
}

////////////////////////////LT9211 end/////////////////////////////////////////////

static struct panel_dev *lt9211_info;

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

static struct fb_videomode panel_modes = {
	.name                   = "lt9211-G104X1L04",
	.refresh                = FPS,
	.xres                   = HACT,
	.yres                   = VACT,
	.left_margin            = HFP,
	.right_margin           = HBP,
	.upper_margin           = VFP,
	.lower_margin           = VBP,

	.hsync_len              = HS,
	.vsync_len              = VS,
	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag                   = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 2,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL1,

	.dsi_config.max_lanes = 2,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
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

struct lcd_panel lcd_panel[] = {
	[0] = {
		.name = "G104X1L04",
		.modes = &panel_modes,
		.num_modes = 1,
		.dsi_pdata = &jzdsi_pdata,

		.bpp = 24,
		.width = 699,
		.height = 197,

		.lcd_type = LCD_TYPE_MIPI_TFT,

		.tft_config = &kd050hdfia019_cfg,
		.dsi_pdata = &jzdsi_pdata,

		.dither_enable = 0,
		.dither.dither_red = 0,
		.dither.dither_green = 0,
		.dither.dither_blue = 0,

		.ops = &panel_ops,
	},
};

static unsigned char i2c_read(struct i2c_client *client, unsigned char addr)
{
	return i2c_smbus_read_byte_data(client, addr);
}

static unsigned int i2c_write(struct i2c_client *client,unsigned char addr,unsigned char value)
{
	return i2c_smbus_write_byte_data(client, addr, value);
}

void lt9211_reset(struct panel_dev *lt9211_info)
{
	struct board_gpio *rst = &lt9211_info->reset;

	gpio_direction_output(rst->gpio, 1);
	mdelay(1);
	gpio_direction_output(rst->gpio, 0);
	mdelay(100);
	gpio_direction_output(rst->gpio, 1);
	mdelay(100);
}

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *lt9211_info = lcd_get_data(lcd);
	struct i2c_client *client = lt9211_info->client;

	struct board_gpio *lcd_en = &lt9211_info->lcd_en;
	struct board_gpio *bl_en = &lt9211_info->bl_en;
	struct board_gpio *rst = &lt9211_info->reset;

	if(POWER_IS_ON(power) && !POWER_IS_ON(lt9211_info->power)) {
		gpio_direction_output(rst->gpio, 0);
		gpio_direction_output(lcd_en->gpio, 1);
		mdelay(15);

		lt9211_reset(lt9211_info);
		if (LT9211_MIPI2LVDS_Config(client))
			return -1;

	    gpio_direction_output(bl_en->gpio, 1);
	}
	if(!POWER_IS_ON(power) && POWER_IS_ON(lt9211_info->power)) {
		gpio_direction_output(bl_en->gpio, 0);
		gpio_direction_output(lcd_en->gpio, 0);
	}

	lt9211_info->power = power;
	return 0;
}

int panel_bridge_init(void)
{
	int ret = 0;
	struct i2c_client *client = lt9211_info->client;

	ret = LT9211_MIPI2LVDS_Config(client);

	return ret;
}
static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *lt9211_info = lcd_get_data(lcd);

	return lt9211_info->power;
}

static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};


int _of_get_named_gpio_lvl(struct device *dev, short *gpio, short *lvl, char *of_name)
{
	int ret = 0;
	enum of_gpio_flags flags;

	/* devm_gpio_request_one */
	*gpio = of_get_named_gpio_flags(dev->of_node, of_name, 0, &flags);
	if(gpio_is_valid(*gpio)) {
		*lvl = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(*gpio, GPIOF_DIR_OUT, of_name);
		if(ret < 0) {
			dev_err(dev, "Failed to request reset pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio: %s\n", of_name);
		return -1;
	}
	return 0;
}

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);
	int ret = 0;


	if ((ret = _of_get_named_gpio_lvl(dev,
				&lt9211_info->reset.gpio,
				&lt9211_info->reset.active_level,
				"lt9211,reset-gpio")))
		return ret;

	if ((ret = _of_get_named_gpio_lvl(dev,
				&lt9211_info->lcd_en.gpio,
				&lt9211_info->lcd_en.active_level,
				"lt9211,lcd_en-gpio")))
		goto err_request_reset;

	if ((ret = _of_get_named_gpio_lvl(dev,
				&lt9211_info->bl_en.gpio,
				&lt9211_info->bl_en.active_level,
				"lt9211,bl_en-gpio")))
		goto err_request_reset;

	/* devm_ */

	return 0;
err_request_reset:
	if(gpio_is_valid(lt9211_info->reset.gpio))
		gpio_free(lt9211_info->reset.gpio);
	return ret;
}
#if 0
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
static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int panel_suspend(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

static int panel_resume(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_UNBLANK);
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_suspend,
	.resume = panel_resume,
};
#endif

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,G104X1L04", },
	{},
};

static struct platform_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.driver		= {
		.name	= "G104X1L04",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};
#endif

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "lt9211-G104X1L04",
	.id = -1,

	/* .power_on = panel_dev_power_on, */
	/* .set_sequence = panel_dev_set_sequence, */
	/* .probe = panel_dev_probe, */
	/* .suspend = panel_suspend, */
	/* .resume = panel_resume, */
};


struct mipi_dsim_lcd_device panel_dev_device={
	.name		= "lt9211-G104X1L04",
	.id = 0,
};

static int lt9211_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, i;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

	lt9211_info = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if(lt9211_info == NULL) {
		dev_err(dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	lt9211_info->dev = dev;
	lt9211_info->client = client;
	dev_set_drvdata(dev, lt9211_info);

	// set dev_info.
	ret = of_panel_parse(dev);
	if(ret < 0) {
		goto err_of_parse;
	}

	// register lcd device.
	lt9211_info->lcd = lcd_device_register("panel_lcd", dev, lt9211_info, &panel_lcd_ops);
	if(IS_ERR_OR_NULL(lt9211_info->lcd)) {
		dev_err(dev, "Error register lcd!\n");
		ret = -EINVAL;
		goto err_of_parse;
	}

	of_property_read_string(np, "lt9211,panel_name", (const char**)&lt9211_info->panel_name);

	if (lt9211_info->panel_name) {
		for (i = 0; i < ARRAY_SIZE(lcd_panel); ++i) {
			if (strcmp(lcd_panel[i].name, lt9211_info->panel_name) == 0)
				break;
		}
		if (i == ARRAY_SIZE(lcd_panel)) {
			printk("\033[31munsupport panel!\033[0m\n");
			goto err_lcd_register;
		}
	} else {
		printk("\033[31mno found specid lcd panel!\033[0m\n");
		goto err_lcd_register;
	}

	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel[i]);
	if(ret < 0) {
		dev_err(dev, "Failed to register lcd panel!\n");
		goto err_lcd_register;
	}

	// register ingenicfb device.
	/* TODO: should this power status sync from uboot */
	lt9211_info->power = FB_BLANK_POWERDOWN;
	if (panel_set_power(lt9211_info->lcd, FB_BLANK_UNBLANK))
		goto err_lcd_register;

	printk("\033[32mregister %s sucess.\033[0m\n", lt9211_info->panel_name);

	return 0;

err_lcd_register:
	lcd_device_unregister(lt9211_info->lcd);
err_of_parse:
	kfree(lt9211_info);
	printk("\033[31mregister %s dailed.\033[0m\n", lt9211_info->panel_name);
	return ret;
}

static int lt9211_remove(struct i2c_client *client)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(&client->dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_POWERDOWN);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lt9211_match_table[] = {
		{.compatible = "mipi2lvds,lt9211",},
		{ },
};
MODULE_DEVICE_TABLE(of, lt9211_match_table);
#endif

static const struct i2c_device_id lt9211_id[] = {
	{ "lt9211", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt9211_id);

static struct i2c_driver lt9211_driver = {
    .probe      = lt9211_probe,
    .remove     = lt9211_remove,
    .driver = {
        .name     = "lt9211",
        .owner    = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lt9211_match_table),
#endif
    },
	.id_table = lt9211_id,
};

static int __init lt9211_init(void)
{
	int ret;
	printk("\033[33m%s\033[0m\n", "lt9211 driver installing...");
	ret = i2c_add_driver(&lt9211_driver);
    if ( ret != 0 ) {
        printk("lt9211 driver init failed!\n");
    }

    return ret;
}

static void __exit lt9211_exit(void)
{
	printk("lt9211 driver exited.\n");
	i2c_del_driver(&lt9211_driver);
}

/* module_i2c_driver(lt9211_driver); */
module_init(lt9211_init);
module_exit(lt9211_exit);

MODULE_DESCRIPTION("LT9211 Driver");
MODULE_LICENSE("GPL");
