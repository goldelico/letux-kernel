#ifndef	_LINUX_AXP_SPLY_H_
#define	_LINUX_AXP_SPLY_H_

static 	struct input_dev * powerkeydev;

/*      AXP      */
#define AXP_CHARGE_STATUS		AXP_STATUS
#define AXP_IN_CHARGE			(1 << 6)
#define AXP_PDBC			(0x32)
#define AXP_CHARGE_CONTROL1		AXP_CHARGE1
#define AXP_CHARGER_ENABLE		(1 << 7)
#define AXP_CHARGE_CONTROL2		AXP_CHARGE2
#define AXP_CHARGE_VBUS		AXP_IPS_SET
#define AXP_CAP			(0xB9)
#define AXP_BATCAP0			(0xe0)
#define AXP_BATCAP1			(0xe1)
#define AXP_RDC0			(0xba)
#define AXP_RDC1			(0xbb)
#define AXP_WARNING_LEVEL		(0xe6)
#define AXP_ADJUST_PARA		(0xe8)
#define AXP_FAULT_LOG1		AXP_MODE_CHGSTATUS
#define AXP_FAULT_LOG_CHA_CUR_LOW	(1 << 2)
#define AXP_FAULT_LOG_BATINACT	(1 << 3)
#define AXP_FAULT_LOG_OVER_TEMP	(1 << 7)
#define AXP_FAULT_LOG2		AXP_INTSTS2
#define AXP_FAULT_LOG_COLD		(1 << 0)
#define AXP_FINISH_CHARGE		(1 << 2)
#define AXP_COULOMB_CONTROL		AXP_COULOMB_CTL
#define AXP_COULOMB_ENABLE		(1 << 7)
#define AXP_COULOMB_SUSPEND		(1 << 6)
#define AXP_COULOMB_CLEAR		(1 << 5)

#define AXP_ADC_CONTROL				AXP_ADC_EN
#define AXP_ADC_BATVOL_ENABLE				(1 << 7)
#define AXP_ADC_BATCUR_ENABLE				(1 << 6)
#define AXP_ADC_DCINVOL_ENABLE			(1 << 5)
#define AXP_ADC_DCINCUR_ENABLE			(1 << 4)
#define AXP_ADC_USBVOL_ENABLE				(1 << 3)
#define AXP_ADC_USBCUR_ENABLE				(1 << 2)
#define AXP_ADC_APSVOL_ENABLE				(1 << 1)
#define AXP_ADC_TSVOL_ENABLE				(1 << 0)
#define AXP_ADC_INTERTEM_ENABLE			(1 << 7)
#define AXP_ADC_GPIO0_ENABLE				(1 << 3)
#define AXP_ADC_GPIO1_ENABLE				(1 << 2)
#define AXP_ADC_GPIO2_ENABLE				(1 << 1)
#define AXP_ADC_GPIO3_ENABLE				(1 << 0)
#define AXP_ADC_CONTROL3				(0x84)
#define AXP_VBATH_RES					(0x78)
#define AXP_VTS_RES					(0x58)
#define AXP_VBATL_RES					(0x79)
#define AXP_OCVBATH_RES				(0xBC)
#define AXP_OCVBATL_RES				(0xBD)
#define AXP_INTTEMP					(0x56)
#define AXP_DATA_BUFFER0				AXP_BUFFER1
#define AXP_DATA_BUFFER1				AXP_BUFFER2
#define AXP_DATA_BUFFER2				AXP_BUFFER3
#define AXP_DATA_BUFFER3				AXP_BUFFER4
#define AXP_DATA_BUFFER4				AXP_BUFFER5
#define AXP_DATA_BUFFER5				AXP_BUFFER6
#define AXP_DATA_BUFFER6				AXP_BUFFER7
#define AXP_DATA_BUFFER7				AXP_BUFFER8
#define AXP_DATA_BUFFER8				AXP_BUFFER9
#define AXP_DATA_BUFFER9				AXP_BUFFERA
#define AXP_DATA_BUFFERA				AXP_BUFFERB
#define AXP_DATA_BUFFERB				AXP_BUFFERC


#define AXP_CHG_ATTR(_name)					\
{								\
	.attr = { .name = #_name,.mode = 0644 },		\
	.show =  _name##_show,					\
	.store = _name##_store,					\
}

struct axp_adc_res {//struct change
	uint16_t vbat_res;
	uint16_t ocvbat_res;
	uint16_t ibat_res;
	uint16_t ichar_res;
	uint16_t idischar_res;
	uint16_t vac_res;
	uint16_t iac_res;
	uint16_t vusb_res;
	uint16_t iusb_res;
	uint16_t ts_res;
};

struct axp_charger {
	/*power supply sysfs*/
	struct power_supply *batt;
	struct power_supply *	ac;
	struct power_supply *	usb;
	struct power_supply *bubatt;

	/*i2c device*/
	struct device *master;

	/* adc */
	struct axp_adc_res *adc;
	unsigned int sample_time;

	/*monitor*/
	struct delayed_work work;
	unsigned int interval;

	/*battery info*/
	struct power_supply_info *battery_info;

	/*charger control*/
	bool chgen;
	bool limit_on;
	unsigned int chgcur;
	unsigned int chgvol;
	unsigned int chgend;

	/*charger time */
	int chgpretime;
	int chgcsttime;

	/*external charger*/
	bool chgexten;
	int chgextcur;

	/* charger status */
	bool bat_det;
	bool is_on;
	bool is_finish;
	bool ac_not_enough;
	bool ac_det;
	bool usb_det;
	bool ac_valid;
	bool usb_valid;
	bool ext_valid;
	bool bat_current_direction;
	bool in_short;
	bool batery_active;
	bool low_charge_current;
	bool int_over_temp;
	uint8_t fault;
	int charge_on;

	int vbat;
	int ibat;
	int pbat;
	int vac;
	int iac;
	int vusb;
	int iusb;
	int ocv;

	int disvbat;
	int disibat;

	/*rest time*/
	int rest_vol;
	int ocv_rest_vol;
	int base_restvol;
	int rest_time;

	/*ic temperature*/
	int ic_temp;
	int bat_temp;

	/*irq*/
	struct notifier_block nb;

	/* platform callbacks for battery low and critical events */
	void (*battery_low)(void);
	void (*battery_critical)(void);

	struct dentry *debug_file;
};

static struct task_struct *main_task;
static struct axp_charger *axp_charger;
static int Total_Cap = 0;
static int flag_state_change = 0;
static int Bat_Cap_Buffer[AXP_VOL_MAX];

#endif
