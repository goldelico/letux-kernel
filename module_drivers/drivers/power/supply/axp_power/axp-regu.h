#ifndef _LINUX_AXP_REGU_H_
#define _LINUX_AXP_REGU_H_

#include "axp-cfg.h"
#include "axp-mfd.h"

/* AXP216 Regulator Registers */
#define AXP_LDO1			AXP_STATUS
#define AXP_LDO5	        AXP_DLDO1OUT_VOL
#define AXP_LDO6	        AXP_DLDO2OUT_VOL
#define AXP_LDO7	        AXP_DLDO3OUT_VOL
#define AXP_LDO8	        AXP_DLDO4OUT_VOL
#define AXP_LDO9			AXP_ELDO1OUT_VOL
#define AXP_LDO10			AXP_ELDO2OUT_VOL
#define AXP_LDO11			AXP_ELDO3OUT_VOL
#define AXP_LDO12			AXP_DC5LDOOUT_VOL
#define AXP_DCDC1	        AXP_DC1OUT_VOL
#define AXP_DCDC2	        AXP_DC2OUT_VOL
#define AXP_DCDC3	        AXP_DC3OUT_VOL
#define AXP_DCDC4	        AXP_DC4OUT_VOL
#define AXP_DCDC5	        AXP_DC5OUT_VOL

#define AXP_LDOIO0		AXP_GPIO0LDOOUT_VOL
#define AXP_LDOIO1		AXP_GPIO1LDOOUT_VOL
#define AXP_LDO2	        AXP_ALDO1OUT_VOL
#define AXP_LDO3	        AXP_ALDO2OUT_VOL
#define AXP_LDO4	        AXP_ALDO3OUT_VOL
#define AXP_SW0		AXP_STATUS

#define AXP_LDO1EN		AXP_STATUS
#define AXP_LDO2EN		AXP_LDO_DC_EN1
#define AXP_LDO3EN		AXP_LDO_DC_EN1
//#ifdef CONFIG_AXP809
#define AXP_LDO4EN		AXP_LDO_DC_EN2
//#else
//#define AXP_LDO4EN		AXP_LDO_DC_EN3
//#endif
#define AXP_LDO5EN		AXP_LDO_DC_EN2
#define AXP_LDO6EN		AXP_LDO_DC_EN2
#define AXP_LDO7EN		AXP_LDO_DC_EN2
#define AXP_LDO8EN		AXP_LDO_DC_EN2
#define AXP_LDO9EN		AXP_LDO_DC_EN2
#define AXP_LDO10EN		AXP_LDO_DC_EN2
#define AXP_LDO11EN		AXP_LDO_DC_EN2
#define AXP_LDO12EN		AXP_LDO_DC_EN1
#define AXP_DCDC1EN		AXP_LDO_DC_EN1
#define AXP_DCDC2EN		AXP_LDO_DC_EN1
#define AXP_DCDC3EN		AXP_LDO_DC_EN1
#define AXP_DCDC4EN		AXP_LDO_DC_EN1
#define AXP_DCDC5EN		AXP_LDO_DC_EN1
#define AXP_LDOIO0EN		AXP_GPIO0_CTL
#define AXP_LDOIO1EN		AXP_GPIO1_CTL
#define AXP_DC1SW1EN		AXP_LDO_DC_EN2
#define AXP_SW0EN		AXP_LDO_DC_EN2

#define AXP_BUCKMODE		AXP_DCDC_MODESET
#define AXP_BUCKFREQ		AXP_DCDC_FREQSET

#define AXP_LDO(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)	\
{									\
	.desc	= {							\
/*		.name	= #_pmic"_LDO" #_id,				*/\
		.name	= #_id,					        \
		.type	= REGULATOR_VOLTAGE,				\
		.id	= _pmic##_ID_##_id,				\
		.n_voltages = (step1) ? ( (switch_vol) ? ((switch_vol - min) / step1 +	\
				(max - switch_vol) / step2  +1):	\
				((max - min) / step1 +1) ): 1,		\
		.owner	= THIS_MODULE,					\
	},								\
	.min_uV		= (min) * 1000,					\
	.max_uV		= (max) * 1000,					\
	.step1_uV	= (step1) * 1000,				\
	.vol_reg	= _pmic##_##vreg,				\
	.vol_shift	= (shift),					\
	.vol_nbits	= (nbits),					\
	.enable_reg	= _pmic##_##ereg,				\
	.enable_bit	= (ebit),					\
	.switch_uV	= (switch_vol)*1000,				\
	.step2_uV	= (step2)*1000,					\
	.new_level_uV	= (new_level)*1000,				\
}

#define AXP_BUCK(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level) \
{									\
	.desc	= {							\
		.name	= #_pmic"_BUCK" #_id,					\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= _pmic##_ID_BUCK##_id,				\
		.n_voltages = (step1) ? ( (switch_vol) ? ((new_level)?((switch_vol - min) / step1 +	\
				(max - new_level) / step2  +2) : ((switch_vol - min) / step1 +	\
				(max - switch_vol) / step2  +1)):	\
				((max - min) / step1 +1) ): 1,		\
	},								\
	.min_uV		= (min) * 1000,					\
	.max_uV		= (max) * 1000,					\
	.step1_uV	= (step1) * 1000,				\
	.vol_reg	= _pmic##_##vreg,				\
	.vol_shift	= (shift),					\
	.vol_nbits	= (nbits),					\
	.enable_reg	= _pmic##_##ereg,				\
	.enable_bit	= (ebit),					\
	.switch_uV	= (switch_vol)*1000,				\
	.step2_uV	= (step2)*1000,					\
	.new_level_uV	= (new_level)*1000,				\
}

#define AXP_DCDC(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)	\
{									\
	.desc	= {							\
		.name	= "DCDC"#_id,					\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= _pmic##_ID_DCDC##_id,				\
		.n_voltages = (step1) ? ( (switch_vol) ? ((new_level)?((switch_vol - min) / step1 +	\
				(max - new_level) / step2  +2) : ((switch_vol - min) / step1 +	\
				(max - switch_vol) / step2  +1)):	\
				((max - min) / step1 +1) ): 1,		\
		.owner	= THIS_MODULE,					\
	},								\
	.min_uV		= (min) * 1000,					\
	.max_uV		= (max) * 1000,					\
	.step1_uV	= (step1) * 1000,				\
	.vol_reg	= _pmic##_##vreg,				\
	.vol_shift	= (shift),					\
	.vol_nbits	= (nbits),					\
	.enable_reg	= _pmic##_##ereg,				\
	.enable_bit	= (ebit),					\
	.switch_uV	= (switch_vol)*1000,				\
	.step2_uV	= (step2)*1000,					\
	.new_level_uV	= (new_level)*1000,				\
}
//AXP_LDO2EN
#define AXP_SW(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level) \
{									\
	.desc	= {							\
		.name	= #_pmic"_SW" #_id,					\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= _pmic##_ID_SW##_id,				\
		.n_voltages = (step1) ? ( (switch_vol) ? ((new_level)?((switch_vol - min) / step1 +	\
				(max - new_level) / step2  +2) : ((switch_vol - min) / step1 +	\
				(max - switch_vol) / step2  +1)):	\
				((max - min) / step1 +1) ): 1,		\
		.owner	= THIS_MODULE,					\
	},								\
	.min_uV		= (min) * 1000,					\
	.max_uV		= (max) * 1000,					\
	.step1_uV	= (step1) * 1000,				\
	.vol_reg	= _pmic##_##vreg,				\
	.vol_shift	= (shift),					\
	.vol_nbits	= (nbits),					\
	.enable_reg	= _pmic##_##ereg,				\
	.enable_bit	= (ebit),					\
	.switch_uV	= (switch_vol)*1000,				\
	.step2_uV	= (step2)*1000,					\
	.new_level_uV	= (new_level)*1000,				\
}

#define AXP_REGU_ATTR(_name)						\
{									\
	.attr = { .name = #_name,.mode = 0644 },			\
	.show =  _name##_show,						\
	.store = _name##_store,						\
}

struct axp_regulator_info {
	struct regulator_desc desc;

	int	min_uV;
	int	max_uV;
	int	step1_uV;
	int	vol_reg;
	int	vol_shift;
	int	vol_nbits;
	int	enable_reg;
	int	enable_bit;
	int	switch_uV;
	int	step2_uV;
	int	new_level_uV;
};

struct  axp_reg_init {
	struct regulator_init_data axp_reg_init_data;
	struct axp_regulator_info *info;
};

#endif
