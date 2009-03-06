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
        int delay;
        int presc;
        /* array of pointers to filter APIs we want to use, in order
         * ends on first NULL, all NULL is OK
         */
        struct ts_filter_api *filter_sequence[MAX_TS_FILTER_CHAIN];
        /* array of configuration ints, one for each filter above */
        void *filter_config[MAX_TS_FILTER_CHAIN];
};

void set_s3c2410ts_info(struct s3c2410_ts_mach_info *hard_s3c2410ts_info);

#endif /* __ASM_ARM_TS_H */

