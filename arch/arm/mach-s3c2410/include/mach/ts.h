/* arch/arm/mach-s3c2410/include/mach/ts.h
 *
 * Copyright (c) 2005 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *  Changelog:
 *     24-Mar-2005     RTP     Created file
 *     03-Aug-2005     RTP     Renamed to ts.h
 */

#ifndef __ASM_ARM_TS_H
#define __ASM_ARM_TS_H

#include <../drivers/input/touchscreen/ts_filter.h>

struct s3c2410_ts_mach_info {
	/* Touchscreen delay. */
	int delay;
	/* Prescaler value. */
	int presc;
	/*
	 * Null-terminated array of pointers to filter APIs and configurations
	 * we want to use. In the same order they will be applied.
	 */
	const struct ts_filter_chain_configuration *filter_config;
};

void set_s3c2410ts_info(const struct s3c2410_ts_mach_info *hard_s3c2410ts_info);

#endif /* __ASM_ARM_TS_H */
