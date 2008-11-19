#ifndef _LINUX_PCF50606_H
#define _LINUX_PCF50606_H

#include <linux/pcf506xx.h>


/* public in-kernel pcf50606 api */
enum pcf50606_regulator_id {
	PCF50606_REGULATOR_DCD,
	PCF50606_REGULATOR_DCDE,
	PCF50606_REGULATOR_DCUD,
	PCF50606_REGULATOR_D1REG,
	PCF50606_REGULATOR_D2REG,
	PCF50606_REGULATOR_D3REG,
	PCF50606_REGULATOR_LPREG,
	PCF50606_REGULATOR_IOREG,
	__NUM_PCF50606_REGULATORS
};

struct pcf50606_data;

/* This is an ugly construct on how to access the (currently single/global)
 * pcf50606 handle from other code in the kernel.  I didn't really come up with
 * a more decent method of dynamically resolving this */
extern struct pcf50606_data *pcf50606_global;

extern void
pcf50606_go_standby(void);

extern void
pcf50606_gpo0_set(struct pcf50606_data *pcf, int on);

extern int
pcf50606_gpo0_get(struct pcf50606_data *pcf);

extern int
pcf50606_voltage_set(struct pcf50606_data *pcf,
		     enum pcf50606_regulator_id reg,
		     unsigned int millivolts);
extern unsigned int
pcf50606_voltage_get(struct pcf50606_data *pcf,
		     enum pcf50606_regulator_id reg);
extern int
pcf50606_onoff_get(struct pcf50606_data *pcf,
		   enum pcf50606_regulator_id reg);

extern int
pcf50606_onoff_set(struct pcf50606_data *pcf,
		   enum pcf50606_regulator_id reg, int on);

extern void
pcf50606_charge_fast(struct pcf50606_data *pcf, int on);


#define PCF50606_FEAT_EXTON	0x00000001	/* not yet supported */
#define PCF50606_FEAT_MBC	0x00000002
#define PCF50606_FEAT_BBC	0x00000004	/* not yet supported */
#define PCF50606_FEAT_TSC	0x00000008	/* not yet supported */
#define PCF50606_FEAT_WDT	0x00000010
#define PCF50606_FEAT_ACD	0x00000020
#define PCF50606_FEAT_RTC	0x00000040
#define PCF50606_FEAT_PWM	0x00000080
#define PCF50606_FEAT_CHGCUR	0x00000100
#define PCF50606_FEAT_BATVOLT	0x00000200
#define PCF50606_FEAT_BATTEMP	0x00000400
#define PCF50606_FEAT_PWM_BL	0x00000800

struct pcf50606_platform_data {
	/* general */
	unsigned int used_features;
	unsigned int onkey_seconds_required;

	/* voltage regulator related */
	struct pmu_voltage_rail rails[__NUM_PCF50606_REGULATORS];
	unsigned int used_regulators;

	/* charger related */
	unsigned int r_fix_batt;
	unsigned int r_fix_batt_par;
	unsigned int r_sense_milli;

	/* backlight related */
	unsigned int init_brightness;

	struct {
		u_int8_t mbcc3; /* charger voltage / current */
	} charger;
	pmu_cb *cb;
};

#endif
