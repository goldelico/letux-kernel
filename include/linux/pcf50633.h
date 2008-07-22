#ifndef _LINUX_PCF50633_H
#define _LINUX_PCF50633_H

#include <linux/pcf506xx.h>
#include <linux/resume-dependency.h>


/* public in-kernel pcf50633 api */
enum pcf50633_regulator_id {
	PCF50633_REGULATOR_AUTO,
	PCF50633_REGULATOR_DOWN1,
	PCF50633_REGULATOR_DOWN2,
	PCF50633_REGULATOR_MEMLDO,
	PCF50633_REGULATOR_LDO1,
	PCF50633_REGULATOR_LDO2,
	PCF50633_REGULATOR_LDO3,
	PCF50633_REGULATOR_LDO4,
	PCF50633_REGULATOR_LDO5,
	PCF50633_REGULATOR_LDO6,
	PCF50633_REGULATOR_HCLDO,
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
extern struct pcf50633_data *pcf50633_global;

extern void
pcf50633_go_standby(void);

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
pcf50633_voltage_set(struct pcf50633_data *pcf,
		     enum pcf50633_regulator_id reg,
		     unsigned int millivolts);
extern unsigned int
pcf50633_voltage_get(struct pcf50633_data *pcf,
		     enum pcf50633_regulator_id reg);

extern int
pcf50633_onoff_get(struct pcf50633_data *pcf,
		   enum pcf50633_regulator_id reg);

extern int
pcf50633_onoff_set(struct pcf50633_data *pcf,
		   enum pcf50633_regulator_id reg, int on);

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

struct pcf50633_platform_data {
	/* general */
	unsigned int used_features;
	unsigned int onkey_seconds_sig_init;
	unsigned int onkey_seconds_shutdown;

	/* callback to attach platform children (to enforce suspend / resume
	 * ordering */
	void (*attach_child_devices)(struct device *parent_device);

	/* voltage regulator related */
	struct pmu_voltage_rail rails[__NUM_PCF50633_REGULATORS];
	unsigned int used_regulators;

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

	/* post-resume backlight bringup */
	int defer_resume_backlight;
	u8 resume_backlight_ramp_speed;
};

#endif /* _PCF50633_H */
