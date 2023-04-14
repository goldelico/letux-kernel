#ifndef __TOUCH_FT6236_H_
#define __TOUCH_FT6236_H_

#define I2C_ADDR                    0x38
#define I2C_MAX_TRANSFER_SIZE		255
#define RETRY_MAX_TIMES			5
#define TPD_DRIVER_NAME     "ft6236-ts"
#define TPD_DRIVER_PHY     "input/ts"

struct ft6236_platform_data {
	int irq_gpio;
	int rst_gpio;
	u32 irq_flags;
	u32 max_x;
	u32 max_y;
	u32 invert_x;
	u32 invert_y;
	u32 swap_xy;
};

struct ft6236_data {
	unsigned long flags; /* This member record the device status */
	struct i2c_client *client;
	struct input_dev *input;
	struct ft6236_platform_data *pdata;
	struct mutex lock;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	spinlock_t irq_lock;
};

struct ft6236_touchpoint {
	union {
		u8 xhi;
		u8 event;
	};
	u8 xlo;
	union {
		u8 yhi;
		u8 id;
	};
	u8 ylo;
};

struct ft6236_packet {
	u8 dev_mode;
	u8 gest_id;
	u8 touches;
	struct ft6236_touchpoint points[2];
};

#endif //__TOUCH_FT6236_h_
