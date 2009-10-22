#ifndef __PLATFORM_BATTERY_H__
#define __PLATFORM_BATTERY_H__

struct platform_bat_platform_data {
	const char *name;
	int (**get_property)(void);
	int (*is_present)(void);
	enum power_supply_property *properties;
	size_t num_properties;
};

#endif
