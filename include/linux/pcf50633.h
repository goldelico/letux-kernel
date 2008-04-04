#ifndef _LINUX_PCF50633_H
#define _LINUX_PCF50633_H

#include <linux/pcf506xx.h>


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
pcf50633_usb_curlim_set(struct pcf50633_data *pcf, int ma);

extern void
pcf50633_charge_enable(struct pcf50633_data *pcf, int on);

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

	/* voltage regulator related */
	struct pmu_voltage_rail rails[__NUM_PCF50633_REGULATORS];
	unsigned int used_regulators;

	/* charger related */
	unsigned int r_fix_batt;
	unsigned int r_fix_batt_par;
	unsigned int r_sense_milli;

	struct {
		u_int8_t mbcc3; /* charger voltage / current */
	} charger;
	pmu_cb *cb;
};

#endif /* _PCF50633_H */
