#ifndef __BQ27000_BATTERY_H__
#define __BQ27000_BATTERY_H__

struct bq27000_platform_data {
	const char 	*name;
	int		rsense_mohms;
	int (*hdq_read)(int);
	int (*hdq_write)(int, u8);
	int (*hdq_initialized)(void);
};

#endif
