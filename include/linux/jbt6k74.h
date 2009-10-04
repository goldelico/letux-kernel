#ifndef __JBT6K74_H__
#define __JBT6K74_H__

#include <linux/spi/spi.h>

/*
 *  struct jbt6k74_platform_data - Platform data for jbt6k74 driver
 *  @probe_completed: Callback to be called when the driver has been
 *  successfully probed.
 *  @enable_pixel_clock: Callback to enable or disable the pixelclock of the
 *  gpu.
 *  @gpio_reset: Reset gpio pin number.
 */
struct jbt6k74_platform_data {
	void (*probe_completed)(struct device *dev);
	void (*enable_pixel_clock)(struct device *dev, int enable);

	int gpio_reset;
};

#endif
