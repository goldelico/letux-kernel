#ifndef __LINUX_I2C_BMP085_H
#define __LINUX_I2C_BMP085_H

/* linux/i2c/bmp085.h */

struct bmp085_platform_data {
	int	irq;

        int     (*init_platform_hw)(void);
        void    (*exit_platform_hw)(void);
};

#endif
