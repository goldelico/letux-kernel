#ifndef _LINUX_LP5521_H_
#define _LINUX_LP5521_H_

#define LP5521_REG_ENABLE 	0x00
#define LP5521_REG_OP_MODE 	0x01

#define LP5521_REG_R_PWM 	0x02
#define LP5521_REG_G_PWM 	0x03
#define LP5521_REG_B_PWM 	0x04

#define LP5521_REG_R_CUR 	0x05
#define LP5521_REG_G_CUR	0x06
#define LP5521_REG_B_CUR	0x07

#define LP5521_REG_CONFIG	0x08

#define LP5521_REG_R_PC		0x09
#define LP5521_REG_G_PC		0x0a
#define LP5521_REG_B_PC		0x0b

#define LP5521_REG_STATUS	0x0c
#define LP5521_REG_RESET	0x0d
#define LP5521_REG_GPO		0x0e

enum {
	LP5521_NC,
	LP5521_CONNECTED,
};

enum {
	LP5521_BLUE,
	LP5521_GREEN,
	LP5521_RED,
	LP5521_NUM_CH,
};

enum {
	LP5521_MODE_DISABLE,
	LP5521_MODE_LOAD,
	LP5521_MODE_RUN,
	LP5521_MODE_DIRECT,
};

enum CP_MODE {
	LP5521_CPM_OFF,
	LP5521_CPM_BY_PASS,
	LP5521_CPM_1_5X,
	LP5521_CPM_AUTO,
};

enum CLK_SRC {
	LP5521_EXT_CLK,
	LP5521_INT_CLK,
	LP5521_AUTO_CLK,
};

#define LP5521_FEAT_TRIG	0x00000001
#define LP5521_FEAT_GPO		0x00000002


struct lp5521_platform_data {
	int channels[LP5521_NUM_CH];
	/* chip enable */
	void (*ext_enable)(int level);
};

struct lp5521 {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;
	int irq;

	struct lp5521_platform_data *pdata;
};
#endif /* LINUX_LP5521_H */
