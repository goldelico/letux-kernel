#ifndef __GTA01_BATTERY_H__
#define __GTA01_BATTERY_H__

struct gta01_bat_platform_data {
	int (*get_charging_status)(void);
	int (*get_voltage)(void);
	int (*get_current)(void);
};

#endif
