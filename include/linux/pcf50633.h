#ifndef _LINUX_PCF50633_H
#define _LINUX_PCF50633_H

#include <linux/pcf506xx.h>
#include <linux/resume-dependency.h>
#include <linux/regulator/machine.h>

#define PCF50633_FIDX_CHG_ENABLED	0	/* Charger enabled */
#define PCF50633_FIDX_CHG_PRESENT	1	/* Charger present */
#define PCF50633_FIDX_CHG_ERR		3	/* Charger Error */
#define PCF50633_FIDX_CHG_PROT		4	/* Charger Protection */
#define PCF50633_FIDX_CHG_READY		5	/* Charging completed */
#define PCF50633_FIDX_PWR_PRESSED	8
#define PCF50633_FIDX_RTC_SECOND	9
#define PCF50633_FIDX_USB_PRESENT	10

#define PCF50633_F_CHG_ENABLED	(1 << PCF50633_FIDX_CHG_ENABLED)
#define PCF50633_F_CHG_PRESENT	(1 << PCF50633_FIDX_CHG_PRESENT)
#define PCF50633_F_CHG_ERR	(1 << PCF50633_FIDX_CHG_ERR)
#define PCF50633_F_CHG_PROT	(1 << PCF50633_FIDX_CHG_PROT)
#define PCF50633_F_CHG_READY	(1 << PCF50633_FIDX_CHG_READY)

#define PCF50633_F_CHG_MASK	0x000000fc

#define PCF50633_F_PWR_PRESSED	(1 << PCF50633_FIDX_PWR_PRESSED)

#define PCF50633_F_RTC_SECOND	(1 << PCF50633_FIDX_RTC_SECOND)
#define PCF50633_F_USB_PRESENT	(1 << PCF50633_FIDX_USB_PRESENT)

/* public in-kernel pcf50633 api */
enum pcf50633_regulator_id {
	PCF50633_REGULATOR_AUTO,
	PCF50633_REGULATOR_DOWN1,
	PCF50633_REGULATOR_DOWN2,
	PCF50633_REGULATOR_LDO1,
	PCF50633_REGULATOR_LDO2,
	PCF50633_REGULATOR_LDO3,
	PCF50633_REGULATOR_LDO4,
	PCF50633_REGULATOR_LDO5,
	PCF50633_REGULATOR_LDO6,
	PCF50633_REGULATOR_HCLDO,
	PCF50633_REGULATOR_MEMLDO,
	__NUM_PCF50633_REGULATORS
};

enum pcf50633_reg_int1 {
	PCF50633_INT1_ADPINS	= 0x01,	/* Adapter inserted */
	PCF50633_INT1_ADPREM	= 0x02,	/* Adapter removed */
	PCF50633_INT1_USBINS	= 0x04,	/* USB inserted */
	PCF50633_INT1_USBREM	= 0x08,	/* USB removed */
	/* reserved */
	PCF50633_INT1_ALARM	= 0x40, /* RTC alarm time is reached */
	PCF50633_INT1_SECOND	= 0x80,	/* RTC periodic second interrupt */
};

enum pcf50633_reg_int2 {
	PCF50633_INT2_ONKEYR	= 0x01, /* ONKEY rising edge */
	PCF50633_INT2_ONKEYF	= 0x02, /* ONKEY falling edge */
	PCF50633_INT2_EXTON1R	= 0x04, /* EXTON1 rising edge */
	PCF50633_INT2_EXTON1F	= 0x08, /* EXTON1 falling edge */
	PCF50633_INT2_EXTON2R	= 0x10, /* EXTON2 rising edge */
	PCF50633_INT2_EXTON2F	= 0x20, /* EXTON2 falling edge */
	PCF50633_INT2_EXTON3R	= 0x40, /* EXTON3 rising edge */
	PCF50633_INT2_EXTON3F	= 0x80, /* EXTON3 falling edge */
};

enum pcf50633_reg_int3 {
	PCF50633_INT3_BATFULL	= 0x01, /* Battery full */
	PCF50633_INT3_CHGHALT	= 0x02,	/* Charger halt */
	PCF50633_INT3_THLIMON	= 0x04,
	PCF50633_INT3_THLIMOFF	= 0x08,
	PCF50633_INT3_USBLIMON	= 0x10,
	PCF50633_INT3_USBLIMOFF	= 0x20,
	PCF50633_INT3_ADCRDY	= 0x40,	/* ADC conversion finished */
	PCF50633_INT3_ONKEY1S	= 0x80,	/* ONKEY pressed 1 second */
};

enum pcf50633_reg_int4 {
	PCF50633_INT4_LOWSYS		= 0x01,
	PCF50633_INT4_LOWBAT		= 0x02,
	PCF50633_INT4_HIGHTMP		= 0x04,
	PCF50633_INT4_AUTOPWRFAIL	= 0x08,
	PCF50633_INT4_DWN1PWRFAIL	= 0x10,
	PCF50633_INT4_DWN2PWRFAIL	= 0x20,
	PCF50633_INT4_LEDPWRFAIL	= 0x40,
	PCF50633_INT4_LEDOVP		= 0x80,
};

enum pcf50633_reg_int5 {
	PCF50633_INT5_LDO1PWRFAIL	= 0x01,
	PCF50633_INT5_LDO2PWRFAIL	= 0x02,
	PCF50633_INT5_LDO3PWRFAIL	= 0x04,
	PCF50633_INT5_LDO4PWRFAIL	= 0x08,
	PCF50633_INT5_LDO5PWRFAIL	= 0x10,
	PCF50633_INT5_LDO6PWRFAIL	= 0x20,
	PCF50633_INT5_HCLDOPWRFAIL	= 0x40,
	PCF50633_INT5_HCLDOOVL		= 0x80,
};

struct pcf50633_data;

extern void
pcf50633_go_standby(struct pcf50633_data *pcf);

enum pcf50633_gpio {
	PCF50633_GPIO1 = 1,
	PCF50633_GPIO2 = 2,
	PCF50633_GPIO3 = 3,
	PCF50633_GPO = 4,
};

extern void
pcf50633_gpio_set(struct pcf50633_data *pcf, enum pcf50633_gpio gpio, int on);

extern int
pcf50633_gpio_get(struct pcf50633_data *pcf, enum pcf50633_gpio gpio);

extern int
pcf50633_adc_async_read(struct pcf50633_data *pcf, int mux, int avg,
		void (*callback)(struct pcf50633_data *, void *, int),
		void *callback_param);
		
extern int
pcf50633_adc_sync_read(struct pcf50633_data *pcf, int mux, int avg);

extern void
pcf50633_backlight_resume(struct pcf50633_data *pcf);

extern u_int16_t
pcf50633_battvolt(struct pcf50633_data *pcf);

extern int
pcf50633_report_resumers(struct pcf50633_data *pcf, char *buf);

extern void
pcf50633_register_resume_dependency(struct pcf50633_data *pcf,
					struct resume_dependency *dep);

extern int
pcf50633_notify_usb_current_limit_change(struct pcf50633_data *pcf,
							       unsigned int ma);
extern int
pcf50633_wait_for_ready(struct pcf50633_data *pcf, int timeout_ms,
								char *name);

/* 0 = initialized and resumed and ready to roll, !=0 = either not
 * initialized or not resumed yet
 */
extern int
pcf50633_ready(struct pcf50633_data *pcf);

#define PCF50633_FEAT_EXTON	0x00000001	/* not yet supported */
#define PCF50633_FEAT_MBC	0x00000002
#define PCF50633_FEAT_BBC	0x00000004	/* not yet supported */
#define PCF50633_FEAT_RTC	0x00000040
#define PCF50633_FEAT_CHGCUR	0x00000100
#define PCF50633_FEAT_BATVOLT	0x00000200
#define PCF50633_FEAT_BATTEMP	0x00000400
#define PCF50633_FEAT_PWM_BL	0x00000800

enum charger_type {
	CHARGER_TYPE_NONE = 0,
	CHARGER_TYPE_HOSTUSB,
	CHARGER_TYPE_1A
};

#define ADC_NOM_CHG_DETECT_1A 6
#define ADC_NOM_CHG_DETECT_NONE 43

#define MAX_ADC_FIFO_DEPTH 8

enum pcf50633_suspend_states {
	PCF50633_SS_RUNNING,
	PCF50633_SS_STARTING_SUSPEND,
	PCF50633_SS_COMPLETED_SUSPEND,
	PCF50633_SS_RESUMING_BUT_NOT_US_YET,
	PCF50633_SS_STARTING_RESUME,
	PCF50633_SS_COMPLETED_RESUME,
};

struct pcf50633_data;

struct pcf50633_platform_data {
	/* general */
	unsigned int used_features;
	unsigned int onkey_seconds_sig_init;
	unsigned int onkey_seconds_shutdown;

	/* callback to attach platform children (to enforce suspend / resume
	 * ordering */
	void (*attach_child_devices)(struct device *parent_device);

	/* charger related */
	unsigned int r_fix_batt;
	unsigned int r_fix_batt_par;
	unsigned int r_sense_milli;
	int flag_use_apm_emulation;

	unsigned char resumers[5];

	struct {
		u_int8_t mbcc3; /* charger voltage / current */
	} charger;
	pmu_cb cb;

	struct regulator_init_data reg_init_data[__NUM_PCF50633_REGULATORS];

	/* Called when a regulator has been registered */
	void (*regulator_registered)(struct pcf50633_data *pcf, int id);

	/* Runtime data */
	struct pcf50633_data *pcf;
};

enum pfc50633_regs {
	PCF50633_REG_VERSION	= 0x00,
	PCF50633_REG_VARIANT	= 0x01,
	PCF50633_REG_INT1	= 0x02,	/* Interrupt Status */
	PCF50633_REG_INT2	= 0x03,	/* Interrupt Status */
	PCF50633_REG_INT3	= 0x04,	/* Interrupt Status */
	PCF50633_REG_INT4	= 0x05,	/* Interrupt Status */
	PCF50633_REG_INT5	= 0x06,	/* Interrupt Status */
	PCF50633_REG_INT1M	= 0x07,	/* Interrupt Mask */
	PCF50633_REG_INT2M	= 0x08,	/* Interrupt Mask */
	PCF50633_REG_INT3M	= 0x09,	/* Interrupt Mask */
	PCF50633_REG_INT4M	= 0x0a,	/* Interrupt Mask */
	PCF50633_REG_INT5M	= 0x0b,	/* Interrupt Mask */
	PCF50633_REG_OOCSHDWN	= 0x0c,
	PCF50633_REG_OOCWAKE	= 0x0d,
	PCF50633_REG_OOCTIM1	= 0x0e,
	PCF50633_REG_OOCTIM2	= 0x0f,
	PCF50633_REG_OOCMODE	= 0x10,
	PCF50633_REG_OOCCTL	= 0x11,
	PCF50633_REG_OOCSTAT	= 0x12,
	PCF50633_REG_GPIOCTL	= 0x13,
	PCF50633_REG_GPIO1CFG	= 0x14,
	PCF50633_REG_GPIO2CFG	= 0x15,
	PCF50633_REG_GPIO3CFG	= 0x16,
	PCF50633_REG_GPOCFG	= 0x17,
	PCF50633_REG_BVMCTL	= 0x18,
	PCF50633_REG_SVMCTL	= 0x19,
	PCF50633_REG_AUTOOUT	= 0x1a,
	PCF50633_REG_AUTOENA	= 0x1b,
	PCF50633_REG_AUTOCTL	= 0x1c,
	PCF50633_REG_AUTOMXC	= 0x1d,
	PCF50633_REG_DOWN1OUT	= 0x1e,
	PCF50633_REG_DOWN1ENA	= 0x1f,
	PCF50633_REG_DOWN1CTL	= 0x20,
	PCF50633_REG_DOWN1MXC	= 0x21,
	PCF50633_REG_DOWN2OUT	= 0x22,
	PCF50633_REG_DOWN2ENA	= 0x23,
	PCF50633_REG_DOWN2CTL	= 0x24,
	PCF50633_REG_DOWN2MXC	= 0x25,
	PCF50633_REG_MEMLDOOUT	= 0x26,
	PCF50633_REG_MEMLDOENA	= 0x27,
	PCF50633_REG_LEDOUT	= 0x28,
	PCF50633_REG_LEDENA	= 0x29,
	PCF50633_REG_LEDCTL	= 0x2a,
	PCF50633_REG_LEDDIM	= 0x2b,
	/* reserved */
	PCF50633_REG_LDO1OUT	= 0x2d,
	PCF50633_REG_LDO1ENA	= 0x2e,
	PCF50633_REG_LDO2OUT	= 0x2f,
	PCF50633_REG_LDO2ENA	= 0x30,
	PCF50633_REG_LDO3OUT	= 0x31,
	PCF50633_REG_LDO3ENA	= 0x32,
	PCF50633_REG_LDO4OUT	= 0x33,
	PCF50633_REG_LDO4ENA	= 0x34,
	PCF50633_REG_LDO5OUT	= 0x35,
	PCF50633_REG_LDO5ENA	= 0x36,
	PCF50633_REG_LDO6OUT	= 0x37,
	PCF50633_REG_LDO6ENA	= 0x38,
	PCF50633_REG_HCLDOOUT	= 0x39,
	PCF50633_REG_HCLDOENA	= 0x3a,
	PCF50633_REG_STBYCTL1	= 0x3b,
	PCF50633_REG_STBYCTL2	= 0x3c,
	PCF50633_REG_DEBPF1	= 0x3d,
	PCF50633_REG_DEBPF2	= 0x3e,
	PCF50633_REG_DEBPF3	= 0x3f,
	PCF50633_REG_HCLDOOVL	= 0x40,
	PCF50633_REG_DCDCSTAT	= 0x41,
	PCF50633_REG_LDOSTAT	= 0x42,
	PCF50633_REG_MBCC1	= 0x43,
	PCF50633_REG_MBCC2	= 0x44,
	PCF50633_REG_MBCC3	= 0x45,
	PCF50633_REG_MBCC4	= 0x46,
	PCF50633_REG_MBCC5	= 0x47,
	PCF50633_REG_MBCC6	= 0x48,
	PCF50633_REG_MBCC7	= 0x49,
	PCF50633_REG_MBCC8	= 0x4a,
	PCF50633_REG_MBCS1	= 0x4b,
	PCF50633_REG_MBCS2	= 0x4c,
	PCF50633_REG_MBCS3	= 0x4d,
	PCF50633_REG_BBCCTL	= 0x4e,
	PCF50633_REG_ALMGAIN	= 0x4f,
	PCF50633_REG_ALMDATA	= 0x50,
	/* reserved */
	PCF50633_REG_ADCC3	= 0x52,
	PCF50633_REG_ADCC2	= 0x53,
	PCF50633_REG_ADCC1	= 0x54,
	PCF50633_REG_ADCS1	= 0x55,
	PCF50633_REG_ADCS2	= 0x56,
	PCF50633_REG_ADCS3	= 0x57,
	/* reserved */
	PCF50633_REG_RTCSC	= 0x59,	/* Second */
	PCF50633_REG_RTCMN	= 0x5a,	/* Minute */
	PCF50633_REG_RTCHR	= 0x5b,	/* Hour */
	PCF50633_REG_RTCWD	= 0x5c,	/* Weekday */
	PCF50633_REG_RTCDT	= 0x5d,	/* Day */
	PCF50633_REG_RTCMT	= 0x5e,	/* Month */
	PCF50633_REG_RTCYR	= 0x5f,	/* Year */
	PCF50633_REG_RTCSCA	= 0x60, /* Alarm Second */
	PCF50633_REG_RTCMNA	= 0x61, /* Alarm Minute */
	PCF50633_REG_RTCHRA	= 0x62, /* Alarm Hour */
	PCF50633_REG_RTCWDA	= 0x63, /* Alarm Weekday */
	PCF50633_REG_RTCDTA	= 0x64, /* Alarm Day */
	PCF50633_REG_RTCMTA	= 0x65, /* Alarm Month */
	PCF50633_REG_RTCYRA	= 0x66, /* Alarm Year */

	PCF50633_REG_MEMBYTE0	= 0x67,
	PCF50633_REG_MEMBYTE1	= 0x68,
	PCF50633_REG_MEMBYTE2	= 0x69,
	PCF50633_REG_MEMBYTE3	= 0x6a,
	PCF50633_REG_MEMBYTE4	= 0x6b,
	PCF50633_REG_MEMBYTE5	= 0x6c,
	PCF50633_REG_MEMBYTE6	= 0x6d,
	PCF50633_REG_MEMBYTE7	= 0x6e,
	/* reserved */
	PCF50633_REG_DCDCPFM	= 0x84,
	__NUM_PCF50633_REGS
};


enum pcf50633_reg_oocshdwn {
	PCF50633_OOCSHDWN_GOSTDBY	= 0x01,
	PCF50633_OOCSHDWN_TOTRST	= 0x04,
	PCF50633_OOCSHDWN_COLDBOOT	= 0x08,
};

enum pcf50633_reg_oocwake {
	PCF50633_OOCWAKE_ONKEY		= 0x01,
	PCF50633_OOCWAKE_EXTON1		= 0x02,
	PCF50633_OOCWAKE_EXTON2		= 0x04,
	PCF50633_OOCWAKE_EXTON3		= 0x08,
	PCF50633_OOCWAKE_RTC		= 0x10,
	/* reserved */
	PCF50633_OOCWAKE_USB		= 0x40,
	PCF50633_OOCWAKE_ADP		= 0x80,
};

enum pcf50633_reg_mbcc1 {
	PCF50633_MBCC1_CHGENA		= 0x01,	/* Charger enable */
	PCF50633_MBCC1_AUTOSTOP		= 0x02,
	PCF50633_MBCC1_AUTORES		= 0x04, /* automatic resume */
	PCF50633_MBCC1_RESUME		= 0x08, /* explicit resume cmd */
	PCF50633_MBCC1_RESTART		= 0x10, /* restart charging */
	PCF50633_MBCC1_PREWDTIME_60M	= 0x20,	/* max. precharging time */
	PCF50633_MBCC1_WDTIME_1H	= 0x00,
	PCF50633_MBCC1_WDTIME_2H	= 0x40,
	PCF50633_MBCC1_WDTIME_4H	= 0x80,
	PCF50633_MBCC1_WDTIME_6H	= 0xc0,
};
#define PCF50633_MBCC1_WDTIME_MASK	  0xc0

enum pcf50633_reg_mbcc2 {
	PCF50633_MBCC2_VBATCOND_2V7	= 0x00,
	PCF50633_MBCC2_VBATCOND_2V85	= 0x01,
	PCF50633_MBCC2_VBATCOND_3V0	= 0x02,
	PCF50633_MBCC2_VBATCOND_3V15	= 0x03,
	PCF50633_MBCC2_VMAX_4V		= 0x00,
	PCF50633_MBCC2_VMAX_4V20	= 0x28,
	PCF50633_MBCC2_VRESDEBTIME_64S	= 0x80,	/* debounce time (32/64sec) */
};
#define	PCF50633_MBCC2_VBATCOND_MASK	  0x03
#define PCF50633_MBCC2_VMAX_MASK	  0x3c

enum pcf50633_reg_adcc1 {
	PCF50633_ADCC1_ADCSTART		= 0x01,
	PCF50633_ADCC1_RES_10BIT	= 0x02,
	PCF50633_ADCC1_AVERAGE_NO	= 0x00,
	PCF50633_ADCC1_AVERAGE_4	= 0x04,
	PCF50633_ADCC1_AVERAGE_8	= 0x08,
	PCF50633_ADCC1_AVERAGE_16	= 0x0c,

	PCF50633_ADCC1_MUX_BATSNS_RES	= 0x00,
	PCF50633_ADCC1_MUX_BATSNS_SUBTR	= 0x10,
	PCF50633_ADCC1_MUX_ADCIN2_RES	= 0x20,
	PCF50633_ADCC1_MUX_ADCIN2_SUBTR	= 0x30,
	PCF50633_ADCC1_MUX_BATTEMP	= 0x60,
	PCF50633_ADCC1_MUX_ADCIN1	= 0x70,
};
#define PCF50633_ADCC1_AVERAGE_MASK	0x0c
#define	PCF50633_ADCC1_ADCMUX_MASK	0xf0

enum pcf50633_reg_adcc2 {
	PCF50633_ADCC2_RATIO_NONE	= 0x00,
	PCF50633_ADCC2_RATIO_BATTEMP	= 0x01,
	PCF50633_ADCC2_RATIO_ADCIN1	= 0x02,
	PCF50633_ADCC2_RATIO_BOTH	= 0x03,
	PCF50633_ADCC2_RATIOSETTL_100US	= 0x04,
};
#define PCF50633_ADCC2_RATIO_MASK	0x03

enum pcf50633_reg_adcc3 {
	PCF50633_ADCC3_ACCSW_EN		= 0x01,
	PCF50633_ADCC3_NTCSW_EN		= 0x04,
	PCF50633_ADCC3_RES_DIV_TWO	= 0x10,
	PCF50633_ADCC3_RES_DIV_THREE	= 0x00,
};

enum pcf50633_reg_adcs3 {
	PCF50633_ADCS3_REF_NTCSW	= 0x00,
	PCF50633_ADCS3_REF_ACCSW	= 0x10,
	PCF50633_ADCS3_REF_2V0		= 0x20,
	PCF50633_ADCS3_REF_VISA		= 0x30,
	PCF50633_ADCS3_REF_2V0_2	= 0x70,
	PCF50633_ADCS3_ADCRDY		= 0x80,
};
#define PCF50633_ADCS3_ADCDAT1L_MASK	0x03
#define PCF50633_ADCS3_ADCDAT2L_MASK	0x0c
#define PCF50633_ADCS3_ADCDAT2L_SHIFT	2
#define PCF50633_ASCS3_REF_MASK		0x70

enum pcf50633_regulator_enable {
	PCF50633_REGULATOR_ON		= 0x01,
	PCF50633_REGULATOR_ON_GPIO1	= 0x02,
	PCF50633_REGULATOR_ON_GPIO2	= 0x04,
	PCF50633_REGULATOR_ON_GPIO3	= 0x08,
};
#define PCF50633_REGULATOR_ON_MASK	0x0f

enum pcf50633_regulator_phase {
	PCF50633_REGULATOR_ACTPH1	= 0x00,
	PCF50633_REGULATOR_ACTPH2	= 0x10,
	PCF50633_REGULATOR_ACTPH3	= 0x20,
	PCF50633_REGULATOR_ACTPH4	= 0x30,
};
#define PCF50633_REGULATOR_ACTPH_MASK	0x30

enum pcf50633_reg_gpocfg {
	PCF50633_GPOCFG_GPOSEL_0	= 0x00,
	PCF50633_GPOCFG_GPOSEL_LED_NFET	= 0x01,
	PCF50633_GPOCFG_GPOSEL_SYSxOK	= 0x02,
	PCF50633_GPOCFG_GPOSEL_CLK32K	= 0x03,
	PCF50633_GPOCFG_GPOSEL_ADAPUSB	= 0x04,
	PCF50633_GPOCFG_GPOSEL_USBxOK	= 0x05,
	PCF50633_GPOCFG_GPOSEL_ACTPH4	= 0x06,
	PCF50633_GPOCFG_GPOSEL_1	= 0x07,
	PCF50633_GPOCFG_GPOSEL_INVERSE	= 0x08,
};
#define PCF50633_GPOCFG_GPOSEL_MASK	0x07

#if 0
enum pcf50633_reg_mbcc1 {
	PCF50633_MBCC1_CHGENA		= 0x01,
	PCF50633_MBCC1_AUTOSTOP		= 0x02,
	PCF50633_MBCC1_AUTORES		= 0x04,
	PCF50633_MBCC1_RESUME		= 0x08,
	PCF50633_MBCC1_RESTART		= 0x10,
	PCF50633_MBCC1_PREWDTIME_30MIN	= 0x00,
	PCF50633_MBCC1_PREWDTIME_60MIN	= 0x20,
	PCF50633_MBCC1_WDTIME_2HRS	= 0x40,
	PCF50633_MBCC1_WDTIME_4HRS	= 0x80,
	PCF50633_MBCC1_WDTIME_6HRS	= 0xc0,
};

enum pcf50633_reg_mbcc2 {
	PCF50633_MBCC2_VBATCOND_2V7	= 0x00,
	PCF50633_MBCC2_VBATCOND_2V85	= 0x01,
	PCF50633_MBCC2_VBATCOND_3V0	= 0x02,
	PCF50633_MBCC2_VBATCOND_3V15	= 0x03,
	PCF50633_MBCC2_VRESDEBTIME_64S	= 0x80,
};
#define PCF50633_MBCC2_VMAX_MASK	0x3c
#endif

enum pcf50633_reg_mbcc7 {
	PCF50633_MBCC7_USB_100mA	= 0x00,
	PCF50633_MBCC7_USB_500mA	= 0x01,
	PCF50633_MBCC7_USB_1000mA	= 0x02,
	PCF50633_MBCC7_USB_SUSPEND	= 0x03,
	PCF50633_MBCC7_BATTEMP_EN	= 0x04,
	PCF50633_MBCC7_BATSYSIMAX_1A6	= 0x00,
	PCF50633_MBCC7_BATSYSIMAX_1A8	= 0x40,
	PCF50633_MBCC7_BATSYSIMAX_2A0	= 0x80,
	PCF50633_MBCC7_BATSYSIMAX_2A2	= 0xc0,
};
#define PCF56033_MBCC7_USB_MASK		0x03

enum pcf50633_reg_mbcc8 {
	PCF50633_MBCC8_USBENASUS	= 0x10,
};

enum pcf50633_reg_mbcs1 {
	PCF50633_MBCS1_USBPRES		= 0x01,
	PCF50633_MBCS1_USBOK		= 0x02,
	PCF50633_MBCS1_ADAPTPRES	= 0x04,
	PCF50633_MBCS1_ADAPTOK		= 0x08,
	PCF50633_MBCS1_TBAT_OK		= 0x00,
	PCF50633_MBCS1_TBAT_ABOVE	= 0x10,
	PCF50633_MBCS1_TBAT_BELOW	= 0x20,
	PCF50633_MBCS1_TBAT_UNDEF	= 0x30,
	PCF50633_MBCS1_PREWDTEXP	= 0x40,
	PCF50633_MBCS1_WDTEXP		= 0x80,
};

enum pcf50633_reg_mbcs2_mbcmod {
	PCF50633_MBCS2_MBC_PLAY		= 0x00,
	PCF50633_MBCS2_MBC_USB_PRE	= 0x01,
	PCF50633_MBCS2_MBC_USB_PRE_WAIT	= 0x02,
	PCF50633_MBCS2_MBC_USB_FAST	= 0x03,
	PCF50633_MBCS2_MBC_USB_FAST_WAIT= 0x04,
	PCF50633_MBCS2_MBC_USB_SUSPEND	= 0x05,
	PCF50633_MBCS2_MBC_ADP_PRE	= 0x06,
	PCF50633_MBCS2_MBC_ADP_PRE_WAIT	= 0x07,
	PCF50633_MBCS2_MBC_ADP_FAST	= 0x08,
	PCF50633_MBCS2_MBC_ADP_FAST_WAIT= 0x09,
	PCF50633_MBCS2_MBC_BAT_FULL	= 0x0a,
	PCF50633_MBCS2_MBC_HALT		= 0x0b,
};
#define PCF50633_MBCS2_MBC_MASK		0x0f
enum pcf50633_reg_mbcs2_chgstat {
	PCF50633_MBCS2_CHGS_NONE	= 0x00,
	PCF50633_MBCS2_CHGS_ADAPTER	= 0x10,
	PCF50633_MBCS2_CHGS_USB		= 0x20,
	PCF50633_MBCS2_CHGS_BOTH	= 0x30,
};
#define PCF50633_MBCS2_RESSTAT_AUTO	0x40

enum pcf50633_reg_mbcs3 {
	PCF50633_MBCS3_USBLIM_PLAY	= 0x01,
	PCF50633_MBCS3_USBLIM_CGH	= 0x02,
	PCF50633_MBCS3_TLIM_PLAY	= 0x04,
	PCF50633_MBCS3_TLIM_CHG		= 0x08,
	PCF50633_MBCS3_ILIM		= 0x10,	/* 1: Ibat > Icutoff */
	PCF50633_MBCS3_VLIM		= 0x20,	/* 1: Vbat == Vmax */
	PCF50633_MBCS3_VBATSTAT		= 0x40,	/* 1: Vbat > Vbatcond */
	PCF50633_MBCS3_VRES		= 0x80, /* 1: Vbat > Vth(RES) */
};

struct adc_request {
	int mux;
	int avg;
	int result;
	void (*callback)(struct pcf50633_data *, void *, int);
	void *callback_param;

	/* Used in case of sync requests */
	struct completion completion;
};

struct pcf50633_data {
	struct i2c_client *client;
	struct pcf50633_platform_data *pdata;
	struct backlight_device *backlight;
	struct mutex lock;
	unsigned int flags;
	unsigned int working;
	struct mutex working_lock;
	struct work_struct work;
	struct rtc_device *rtc;
	struct input_dev *input_dev;
	int allow_close;
	int onkey_seconds;
	int irq;
	enum pcf50633_suspend_states suspend_state;
	int usb_removal_count;
	u8 pcfirq_resume[5];
	int probe_completed;
	int suppress_onkey_events;

	/* if he pulls battery while charging, we notice that and correctly
	 * report that the charger is idle.  But there is no interrupt that
	 * fires if he puts a battery back in and charging resumes.  So when
	 * the battery is pulled, we run this work function looking for
	 * either charger resumption or USB cable pull
	 */
	struct mutex working_lock_nobat;
	struct work_struct work_nobat;
	int working_nobat;
	int usb_removal_count_nobat;
	int jiffies_last_bat_ins;

	/* current limit notification handler stuff */
	struct mutex working_lock_usb_curlimit;
	struct work_struct work_usb_curlimit;
	int pending_curlimit;
	int usb_removal_count_usb_curlimit;

	int last_curlim_set;

	int coldplug_done; /* cleared by probe, set by first work service */
	int flag_bat_voltage_read; /* ipc to /sys batt voltage read func */

	/* we have a FIFO of ADC measurement requests that are used only by
	 * the workqueue service code after the ADC completion interrupt
	 */
	struct adc_request *adc_queue[MAX_ADC_FIFO_DEPTH]; /* amount of averaging */
	int adc_queue_head; /* head owned by foreground code */
	int adc_queue_tail; /* tail owned by service code */

	struct platform_device *regulator_pdev[__NUM_PCF50633_REGULATORS];
};

/* this is to be provided by the board implementation */
extern const u_int8_t pcf50633_initial_regs[__NUM_PCF50633_REGS];

int pcf50633_read(struct pcf50633_data *pcf, u_int8_t reg,
						int nr_regs, u_int8_t *data);

int pcf50633_write(struct pcf50633_data *pcf, u_int8_t reg,
						int nr_regs, u_int8_t *data);

int pcf50633_reg_write(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t val);

u_int8_t pcf50633_reg_read(struct pcf50633_data *pcf, u_int8_t reg);

int pcf50633_reg_set_bit_mask(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t mask, u_int8_t val);
int pcf50633_reg_clear_bits(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t bits);

void pcf50633_charge_autofast(int on);
#endif /* _PCF50633_H */

