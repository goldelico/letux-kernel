#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

/*		Deserializer registers		*/
#define FPD3_DES_DEV_ID			0x00
#define FPD3_DES_RESET			0x01
#define FPD3_DES_CONFIG0		0x02
#define FPD3_DES_CONFIG1		0x03

#define FPD3_DES_SER_ID			0x06
#define FPD3_DES_SER_AL			0x07
#define FPD3_DES_SLAVE_ID0		0x08
#define FPD3_DES_SLAVE_AL0		0x10

#define FPD3_DES_GN_STS			0x1c
	#define SIGNAL_DETECT	(1<<1)
#define FPD3_DES_GPIO_01		0x1d
#define FPD3_DES_GPIO_23		0x1e

#define FPD3_DES_I2C_CTRL1		0x21
#define FPD3_DES_I2C_CTRL2		0x22

/*		Serializer registers		*/
#define FPD3_SER_DEV_ID			0x00
#define FPD3_SER_RESET			0x01
#define FPD3_SER_CONFIG0		0x02
#define FPD3_SER_CONFIG1		0x03
#define FPD3_SER_CONFIG2		0x04

#define FPD3_SER_DES_ID			0x06
#define FPD3_SER_DES_AL			0x06
#define FPD3_SER_SLAVE_ID0		0x07
#define FPD3_SER_SLAVE_AL0		0x08

#define FPD3_SER_GN_STS			0x0c
	#define SIGNAL_DETECT	(1<<1)
#define FPD3_SER_GPIO_01		0x0d
#define FPD3_SER_GPIO_23		0x0e

#define FPD3_SER_DATA_CTRL		0x12
#define FPD3_SER_I2C_CTRL1		0x21
#define FPD3_SER_I2C_CTRL2		0x22

/*		Generic macros			*/
#define GPIO_SHIFT_DIR	3
#define GPIO_SHIFT_VAL	1
#define GPIO_DIR_INPUT	1
#define GPIO_DIR_OUTPUT	0

#define FPD3_MAX_POLL_COUNT		100
#define FPD3_SERDES_MAX_SLAVES		10

enum fpd3_device_type {
	FPD3_SER_DEV,
	FPD3_DES_DEV,
};

struct fpd3_serdes_data {
	struct i2c_client	*client;
	bool			slave_mode;

	struct gpio_chip	chip;
	bool			gpio_export;

	struct i2c_adapter	*adap;
	unsigned int		slave_addr[FPD3_SERDES_MAX_SLAVES];
	unsigned int		slave_alias[FPD3_SERDES_MAX_SLAVES];
	const char		*slave_name[FPD3_SERDES_MAX_SLAVES];
	int			num_slaves;
	int			link_ok;

	const struct fpd3_serdes_platform_data *pdata;
};

struct fpd3_serdes_platform_data {
	char				*name;
	enum fpd3_device_type		dev_type;
	int				device_id;
	int				ngpio;
	int				nslaves;
	int				gpio_2reg;
	const unsigned int		*init_seq;
	int				init_len;
};

static inline int i2c_write_le8(struct i2c_client *client, unsigned addr,
			unsigned data)
{
	int ret = i2c_smbus_write_byte_data(client, addr, data);
	if (ret)
		dev_dbg(&client->dev, "Failed to write 0x%02x to 0x%02x",
					data, addr);
	return ret;
}

static inline int i2c_read_le8(struct i2c_client *client, unsigned addr)
{
	int ret = (int)i2c_smbus_read_byte_data(client, addr);
	if (ret < 0)
		dev_dbg(&client->dev, "Failed to read 0x%02x, Error = %d",
					addr, ret);
	return ret;
}

int fpd3_register_i2c_adapter(struct i2c_client *client);
int fpd3_serdes_initialize(struct i2c_client *client);

