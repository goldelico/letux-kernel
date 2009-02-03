/*
 * rtc.h  -- RTC driver for NXP PCF50606
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_MFD_PCF50606_RTC_H
#define __LINUX_MFD_PCF50606_RTC_H

#include <linux/rtc.h>
#include <linux/platform_device.h>

#define PCF50606_REG_RTCSC	 0x0a	/* Second */
#define PCF50606_REG_RTCMN	 0x0b	/* Minute */
#define PCF50606_REG_RTCHR	 0x0c	/* Hour */
#define PCF50606_REG_RTCWD	 0x0d	/* Weekday */
#define PCF50606_REG_RTCDT	 0x0e	/* Day */
#define PCF50606_REG_RTCMT	 0x0f	/* Month */
#define PCF50606_REG_RTCYR	 0x10	/* Year */
#define PCF50606_REG_RTCSCA	 0x11 /* Alarm Second */
#define PCF50606_REG_RTCMNA	 0x12 /* Alarm Minute */
#define PCF50606_REG_RTCHRA	 0x13 /* Alarm Hour */
#define PCF50606_REG_RTCWDA	 0x14 /* Alarm Weekday */
#define PCF50606_REG_RTCDTA	 0x15 /* Alarm Day */
#define PCF50606_REG_RTCMTA	 0x16 /* Alarm Month */
#define PCF50606_REG_RTCYRA	 0x17 /* Alarm Year */

struct pcf50606_rtc {
	int alarm_enabled;
	int second_enabled;

	struct rtc_device *rtc_dev;
	struct platform_device *pdev;
};

#endif

