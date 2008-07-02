#ifndef __JBT6K74_H__
#define __JBT6K74_H__

#include <linux/spi/spi.h>

struct jbt6k74_platform_data {
	void (*reset)(int devindex, int level);
	void (*resuming)(int devindex); /* called when LCM is resumed */
	void (*suspending)(int devindex, struct spi_device *spi);
	int (*all_dependencies_resumed)(int devindex);
};

#endif
