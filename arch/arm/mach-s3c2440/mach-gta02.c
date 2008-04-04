/*
 * linux/arch/arm/mach-s3c2440/mach-gta02.c
 *
 * S3C2440 Machine Support for the FIC GTA02 (Neo1973)
 *
 * Copyright (C) 2006-2007 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/glamo.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <linux/pcf50633.h>
#include <linux/lis302dl.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/fb.h>
#include <asm/arch/mci.h>
#include <asm/arch/ts.h>
#include <asm/arch/spi.h>
#include <asm/arch/spi-gpio.h>
#include <asm/arch/usb-control.h>

#include <asm/arch/gta01.h>
#include <asm/arch/gta02.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/pm.h>
#include <asm/plat-s3c24xx/udc.h>

#include <linux/glamofb.h>

static struct map_desc gta02_iodesc[] __initdata = {
	{
		.virtual	= 0xe0000000,
		.pfn		= __phys_to_pfn(S3C2410_CS3+0x01000000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg gta02_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},

};

/* PMU driver info */

static int pmu_callback(struct device *dev, unsigned int feature,
			enum pmu_event event)
{
	switch (feature) {
	case PCF50633_FEAT_MBC:
		switch (event) {
		case PMU_EVT_INSERT:
		case PMU_EVT_USB_INSERT:
			pcf50633_charge_enable(pcf50633_global, 1);
			break;
		case PMU_EVT_REMOVE:
		case PMU_EVT_USB_REMOVE:
			pcf50633_charge_enable(pcf50633_global, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static struct pcf50633_platform_data gta02_pcf_pdata = {
	.used_features	= PCF50633_FEAT_MBC |
			  PCF50633_FEAT_BBC |
			  PCF50633_FEAT_RTC |
			  PCF50633_FEAT_CHGCUR |
			  PCF50633_FEAT_BATVOLT |
			  PCF50633_FEAT_BATTEMP |
			  PCF50633_FEAT_PWM_BL,
	.onkey_seconds_sig_init = 4,
	.onkey_seconds_shutdown = 8,
	.cb		= &pmu_callback,
	.r_fix_batt	= 10000,
	.r_fix_batt_par	= 10000,
	.r_sense_milli	= 220,
	.rails	= {
		[PCF50633_REGULATOR_AUTO] = {
			.name		= "io_3v3",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.name		= "core_1v3",
			.voltage	= {
				.init	= 1300,
				.max	= 1600,
			},
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.name		= "core_1v8",
			.voltage	= {
				.init	= 1800,
				.max	= 1800,
			},
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.name		= "sd_3v3",
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_LDO1] = {
			.name		= "stby_1v3",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 1300,
				.max	= 1330,
			},
		},
		[PCF50633_REGULATOR_LDO2] = {
			.name		= "codec_3v3",
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_LDO3] = {
			.name		= "lcm_3v",
			.voltage	= {
				.init	= 3000,
				.max	= 3000,
			},
		},
		[PCF50633_REGULATOR_LDO4] = {
			.name		= "gl_2v5",
			.voltage	= {
				.init	= 2500,
				.max	= 2500,
			},
		},
		[PCF50633_REGULATOR_LDO5] = {
			.name		= "gl_1v5",
			.voltage	= {
				.init	= 1500,
				.max	= 1500,
			},
		},
		[PCF50633_REGULATOR_LDO6] = {
			.name		= "user1",
			.voltage	= {
				.init	= 0,
				.max	= 3300,
			},
		},
	},
};

#if 0 /* currently unused */
static void cfg_pmu_vrail(struct pmu_voltage_rail *vrail, char *name,
			  unsigned int flags, unsigned int init,
			  unsigned int max)
{
	vrail->name = name;
	vrail->flags = flags;
	vrail->voltage.init = init;
	vrail->voltage.max = max;
}
#endif

static void mangle_pmu_pdata_by_system_rev(void)
{
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		/* FIXME: this is only in v1 due to wrong PMU variant */
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].flags =
							PMU_VRAIL_F_SUSPEND_ON;
		break;
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		/* we need to keep the 1.8V going since this is the SDRAM
		 * self-refresh voltage */
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].flags =
							PMU_VRAIL_F_SUSPEND_ON;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].name =
							"io_1v8",
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].name =
							"gsensor_3v3",
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].voltage.init =
							3300;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].voltage.max =
							3300;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].flags &=
							~PMU_VRAIL_F_SUSPEND_ON;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO3].flags =
							PMU_VRAIL_F_UNUSED;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO5] = ((struct pmu_voltage_rail) {
							.name = "rf_3v",
							.voltage = {
								.init = 0,
								.max = 3000,
							}
						});
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO6] = ((struct pmu_voltage_rail) {
							.name = "lcm_3v",
							.voltage = {
								.init = 3000,
								.max = 3000,
							}
						});
		break;
	default:
		break;
	}
}

static struct resource gta02_pmu_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= GTA02_IRQ_PCF50633,
		.end	= GTA02_IRQ_PCF50633,
	},
};

struct platform_device gta02_pmu_dev = {
	.name 		= "pcf50633",
	.num_resources	= ARRAY_SIZE(gta02_pmu_resources),
	.resource	= gta02_pmu_resources,
	.dev		= {
		.platform_data = &gta02_pcf_pdata,
	},
};


/* NOR Flash */

#define GTA02_FLASH_BASE	0x18000000 /* GCS3 */
#define GTA02_FLASH_SIZE	0x200000 /* 2MBytes */

static struct physmap_flash_data gta02_nor_flash_data = {
	.width		= 2,
};

static struct resource gta02_nor_flash_resource = {
	.start		= GTA02_FLASH_BASE,
	.end		= GTA02_FLASH_BASE + GTA02_FLASH_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gta02_nor_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
				.platform_data	= &gta02_nor_flash_data,
			},
	.resource	= &gta02_nor_flash_resource,
	.num_resources	= 1,
};



static struct resource gta02_sdio_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SDI,
		.end	= IRQ_SDI,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = S3C2410_PA_SDI,
		.end   = S3C2410_PA_SDI + S3C24XX_SZ_SDI - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = 0, /* Channel 0 for SDI */
		.end = 0,
	},
};


static struct platform_device gta02_sdio_dev = {
        .name           = "s3c24xx-sdio",
        .id             = -1,
        .dev            = {
                                .coherent_dma_mask      = 0xffffffff,
        },
        .resource       = gta02_sdio_resources,
        .num_resources  = ARRAY_SIZE(gta02_sdio_resources),
};

static struct platform_device *gta02_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	// &s3c_device_sdi, /* FIXME: temporary disable to avoid s3cmci bind */
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&s3c_device_ts,
	&s3c_device_spi0,
	&s3c_device_spi1,
	&gta02_nor_flash,
};

static struct s3c2410_nand_set gta02_nand_sets[] = {
	[0] = {
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flags		= S3C2410_NAND_BBT,
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
 */

static struct s3c2410_platform_nand gta02_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(gta02_nand_sets),
	.sets		= gta02_nand_sets,
};

static struct s3c24xx_mci_pdata gta02_mmc_cfg = {
	.gpio_detect	= GTA02v1_GPIO_nSD_DETECT,
	.set_power	= NULL,
	.ocr_avail	= MMC_VDD_32_33,
};

static void gta02_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, cmd);

	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		s3c2410_gpio_setpin(GTA02_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		s3c2410_gpio_setpin(GTA02_GPIO_USB_PULLUP, 0);
		break;
	case S3C2410_UDC_P_RESET:
		/* FIXME! */
		break;
	default:
		break;
	}
}

/* use a work queue, since I2C API inherently schedules
 * and we get called in hardirq context from UDC driver */

struct vbus_draw {
	struct work_struct work;
	int ma;
};
static struct vbus_draw gta02_udc_vbus_drawer;

static void __gta02_udc_vbus_draw(struct work_struct *work)
{
	if (!pcf50633_global) {
		printk(KERN_ERR  "pcf50633 not initialized yet, can't change "
		       "vbus_draw\n");
		return;
	}
	pcf50633_usb_curlim_set(pcf50633_global, gta02_udc_vbus_drawer.ma);
}

static void gta02_udc_vbus_draw(unsigned int ma)
{
	gta02_udc_vbus_drawer.ma = ma;
	schedule_work(&gta02_udc_vbus_drawer.work);
}

static struct s3c2410_udc_mach_info gta02_udc_cfg = {
	.vbus_draw	= gta02_udc_vbus_draw,
	.udc_command	= gta02_udc_command,

};

static struct s3c2410_ts_mach_info gta02_ts_cfg = {
	.delay = 10000,
	.presc = 65,
	.oversampling_shift = 5,
};

/* SPI: LCM control interface attached to Glamo3362 */

static struct spi_board_info gta02_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		/* platform_data */
		/* controller_data */
		/* irq */
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 2,
		/* chip_select */
	},
};

static struct glamo_spi_info glamo_spi_cfg = {
	.board_size	= ARRAY_SIZE(gta02_spi_board_info),
	.board_info	= gta02_spi_board_info,
};

static struct glamo_spigpio_info glamo_spigpio_cfg = {
	.pin_clk	= GLAMO_GPIO10_OUTPUT,
	.pin_mosi	= GLAMO_GPIO11_OUTPUT,
	.pin_cs		= GLAMO_GPIO12_OUTPUT,
	.pin_miso	= 0,
	.board_size	= ARRAY_SIZE(gta02_spi_board_info),
	.board_info	= gta02_spi_board_info,
};

static struct resource gta01_led_resources[] = {
	[0] = {
		.start	= GTA02_GPIO_VIBRATOR_ON,
		.end	= GTA02_GPIO_VIBRATOR_ON,
	},
};

static struct platform_device gta01_led_dev = {
	.name		= "neo1973-vibrator",
	.num_resources	= ARRAY_SIZE(gta01_led_resources),
	.resource	= gta01_led_resources,
};

/* SPI: Accelerometers attached to SPI of s3c244x */

static void gta02_spi_acc_set_cs(struct s3c2410_spi_info *spi, int cs, int pol)
{
	s3c2410_gpio_setpin(cs, pol);
}

static const struct lis302dl_platform_data lis302_pdata[] = {
	{
		.name		= "lis302-1 (top)"
	}, {
		.name		= "lis302-2 (bottom)"
	},
};

static struct spi_board_info gta02_spi_acc_bdinfo[] = {
	{
		.modalias	= "lis302dl",
		.platform_data	= &lis302_pdata[0],
		.irq		= GTA02_IRQ_GSENSOR_1,
		.max_speed_hz	= 400 * 1000,
		.bus_num	= 1,
		.chip_select	= S3C2410_GPD12,
		.mode		= SPI_MODE_3,
	},
	{
		.modalias	= "lis302dl",
		.platform_data	= &lis302_pdata[1],
		.irq		= GTA02_IRQ_GSENSOR_2,
		.max_speed_hz	= 400 * 1000,
		.bus_num	= 1,
		.chip_select	= S3C2410_GPD13,
		.mode		= SPI_MODE_3,
	},
};

static struct s3c2410_spi_info gta02_spi_acc_cfg = {
	.set_cs		= gta02_spi_acc_set_cs,
	.board_size	= ARRAY_SIZE(gta02_spi_acc_bdinfo),
	.board_info	= gta02_spi_acc_bdinfo,
};

static struct resource gta02_led_resources[] = {
	{
		.name	= "gta02-power:orange",
		.start	= GTA02_GPIO_PWR_LED1,
		.end	= GTA02_GPIO_PWR_LED1,
	}, {
		.name	= "gta02-power:blue",
		.start	= GTA02_GPIO_PWR_LED2,
		.end	= GTA02_GPIO_PWR_LED2,
	}, {
		.name	= "gta02-aux:red",
		.start	= GTA02_GPIO_AUX_LED,
		.end	= GTA02_GPIO_AUX_LED,
	},
};

struct platform_device gta02_led_dev = {
	.name		= "gta02-led",
	.num_resources	= ARRAY_SIZE(gta02_led_resources),
	.resource	= gta02_led_resources,
};

static struct resource gta01_button_resources[] = {
	[0] = {
		.start = GTA02_GPIO_AUX_KEY,
		.end   = GTA02_GPIO_AUX_KEY,
	},
	[1] = {
		.start = GTA02_GPIO_HOLD_KEY,
		.end   = GTA02_GPIO_HOLD_KEY,
	},
	[2] = {
		.start = GTA02_GPIO_JACK_INSERT,
		.end   = GTA02_GPIO_JACK_INSERT,
	},
};

static struct platform_device gta01_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(gta01_button_resources),
	.resource	= gta01_button_resources,
};

static struct platform_device gta01_pm_gsm_dev = {
	.name		= "neo1973-pm-gsm",
};

/* USB */
static struct s3c2410_hcd_info gta02_usb_info = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

static int glamo_irq_is_wired(void)
{
	int rc;
	int count = 0;

	/*
	* GTA02 S-Media IRQs prior to A5 are broken due to a lack of
	* a pullup on the INT# line.  Check for the bad behaviour.
	*/
	s3c2410_gpio_setpin(S3C2410_GPG4, 0);
	s3c2410_gpio_cfgpin(S3C2410_GPG4, S3C2410_GPG4_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPG4, S3C2410_GPG4_INP);
	/*
	* we force it low ourselves for a moment and resume being input.
	* If there is a pullup, it won't stay low for long.  But if the
	* level converter is there as on < A5 revision, the weak keeper
	* on the input of the LC will hold the line low indefinitiely
	*/
	do
		rc = s3c2410_gpio_getpin(S3C2410_GPG4);
	while ((!rc) && ((count++) < 10));
	if (rc) { /* it got pulled back up, it's good */
		printk(KERN_INFO "Detected S-Media IRQ# pullup, "
		"enabling interrupt\n");
		return 0;
	} else  /* Gah we can't work with this level converter */
		printk(KERN_WARNING "** Detected bad IRQ# circuit found"
		" on pre-A5 GTA02: S-Media interrupt disabled **\n");
	return -ENODEV;
}


static void
gta02_glamo_mmc_set_power(unsigned char power_mode, unsigned short vdd)
{
	int mv = 1650;

	printk(KERN_DEBUG "mmc_set_power(power_mode=%u, vdd=%u\n",
	       power_mode, vdd);

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
	case GTA02v2_SYSTEM_REV:
		break;
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		/* depend on pcf50633 driver init */
		if (!pcf50633_global)
			while (!pcf50633_global)
				msleep(10);
		switch (power_mode) {
		case MMC_POWER_ON:
		case MMC_POWER_UP:
			/* select and set the voltage */
			if (vdd > 7) {
				mv += 300 + 100 * (vdd - 8);
				if (mv > 3500)
					mv = 3500;
			}
			pcf50633_voltage_set(pcf50633_global,
					     PCF50633_REGULATOR_HCLDO, mv);
			msleep(10);
			pcf50633_onoff_set(pcf50633_global,
					   PCF50633_REGULATOR_HCLDO, 1);
			msleep(1);
			break;
		case MMC_POWER_OFF:
			pcf50633_onoff_set(pcf50633_global,
					   PCF50633_REGULATOR_HCLDO, 0);
			msleep(1);
			break;
		}
		break;
	}
}

/* Smedia Glamo 3362 */

static struct glamofb_platform_data gta02_glamo_pdata = {
	.width		= 43,
	.height		= 58,
	 /* 24.5MHz --> 40.816ns */
	.pixclock	= 40816,
	.left_margin	= 8,
	.right_margin	= 16,
	.upper_margin	= 2,
	.lower_margin	= 16,
	.hsync_len	= 8,
	.vsync_len	= 2,
	.fb_mem_size	= 0x400000, /* glamo has 8 megs of SRAM. we use 4 */
	.xres		= {
		.min	= 240,
		.max	= 640,
		.defval	= 480,
	},
	.yres		= {
		.min	= 320,
		.max	= 640,
		.defval	= 640,
	},
	.bpp		= {
		.min	= 16,
		.max	= 16,
		.defval	= 16,
	},
	//.spi_info	= &glamo_spi_cfg,
	.spigpio_info	= &glamo_spigpio_cfg,

	/* glamo MMC function platform data */
	.glamo_set_mci_power = gta02_glamo_mmc_set_power,
	.glamo_irq_is_wired = glamo_irq_is_wired,
};

static struct resource gta02_glamo_resources[] = {
	[0] = {
		.start	= S3C2410_CS1,
		.end	= S3C2410_CS1 + 0x1000000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= GTA02_IRQ_3D,
		.end	= GTA02_IRQ_3D,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start = GTA02v1_GPIO_3D_RESET,
		.end   = GTA02v1_GPIO_3D_RESET,
	},
};

static struct platform_device gta02_glamo_dev = {
	.name		= "glamo3362",
	.num_resources	= ARRAY_SIZE(gta02_glamo_resources),
	.resource	= gta02_glamo_resources,
	.dev		= {
		.platform_data	= &gta02_glamo_pdata,
	},
};

static void mangle_glamo_res_by_system_rev(void)
{
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		break;
	default:
		gta02_glamo_resources[2].start = GTA02_GPIO_3D_RESET;
		gta02_glamo_resources[2].end = GTA02_GPIO_3D_RESET;
		break;
	}

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	/* case GTA02v4_SYSTEM_REV: - FIXME: handle this later */
		/* The hardware is missing a pull-up resistor and thus can't
		 * support the Smedia Glamo IRQ */
		gta02_glamo_resources[1].start = 0;
		gta02_glamo_resources[1].end = 0;
		break;
	}
}

static void __init gta02_map_io(void)
{
	s3c24xx_init_io(gta02_iodesc, ARRAY_SIZE(gta02_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(gta02_uartcfgs, ARRAY_SIZE(gta02_uartcfgs));
}

static irqreturn_t gta02_modem_irq(int irq, void *param)
{
	printk(KERN_DEBUG "modem wakeup interrupt\n");
	return IRQ_HANDLED;
}

static void __init gta02_machine_init(void)
{
	int rc;

	s3c_device_usb.dev.platform_data = &gta02_usb_info;
	s3c_device_nand.dev.platform_data = &gta02_nand_info;
	s3c_device_sdi.dev.platform_data = &gta02_mmc_cfg;
	s3c_device_spi1.dev.platform_data = &gta02_spi_acc_cfg;

	/* Only GTA02v1 has a SD_DETECT GPIO.  Since the slot is not
	 * hot-pluggable, this is not required anyway */
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		break;
	default:
		gta02_mmc_cfg.gpio_detect = 0;
		break;
	}

	INIT_WORK(&gta02_udc_vbus_drawer.work, __gta02_udc_vbus_draw);
	s3c24xx_udc_set_platdata(&gta02_udc_cfg);
	set_s3c2410ts_info(&gta02_ts_cfg);

	/* FIXME: hardcoded WLAN module power-up */
	s3c2410_gpio_cfgpin(GTA02_CHIP_PWD, S3C2410_GPIO_OUTPUT);

	/* Power is down */
	s3c2410_gpio_setpin(GTA02_CHIP_PWD, 1);
	mdelay(100);

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA02_CHIP_PWD, 0);
		break;
	default:
		s3c2410_gpio_cfgpin(GTA02_GPIO_nWLAN_RESET, S3C2410_GPIO_OUTPUT);
		/* Chip is in reset state */
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 0);
		mdelay(100);
		/* Power is up */
		s3c2410_gpio_setpin(GTA02_CHIP_PWD, 0);
		mdelay(100);
		/* Chip is out of reset */
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 1);
		break;
	}

	platform_device_register(&gta01_button_dev);
	platform_device_register(&gta01_pm_gsm_dev);

	mangle_pmu_pdata_by_system_rev();
	platform_device_register(&gta02_pmu_dev);
	platform_device_register(&gta01_led_dev);
	platform_device_register(&gta02_led_dev);

	mangle_glamo_res_by_system_rev();
	platform_device_register(&gta02_glamo_dev);

	platform_device_register(&gta02_sdio_dev);

	platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));

	s3c2410_pm_init();

	/* Set LCD_RESET / XRES to high */
	s3c2410_gpio_cfgpin(GTA01_GPIO_LCD_RESET, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(GTA01_GPIO_LCD_RESET, 1);

	/* Make sure the modem can wake us up */
	set_irq_type(GTA02_IRQ_MODEM, IRQT_RISING);
	rc = request_irq(GTA02_IRQ_MODEM, gta02_modem_irq, IRQF_DISABLED,
			 "modem", NULL);
	if (rc < 0)
		printk(KERN_ERR "GTA02: can't request GSM modem wakeup IRQ\n");
	enable_irq_wake(GTA02_IRQ_MODEM);
}

MACHINE_START(NEO1973_GTA02, "GTA02")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= gta02_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= gta02_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
