#ifndef __LINUX_MFD_PCF50633_RTC_H
#define __LINUX_MFD_PCF50633_RTC_H

#include <linux/rtc.h>
#include <linux/platform_device.h>

#define PCF50633_REG_RTCSC	0x59 /* Second */
#define PCF50633_REG_RTCMN	0x5a /* Minute */
#define PCF50633_REG_RTCHR	0x5b /* Hour */
#define PCF50633_REG_RTCWD	0x5c /* Weekday */
#define PCF50633_REG_RTCDT	0x5d /* Day */
#define PCF50633_REG_RTCMT	0x5e /* Month */
#define PCF50633_REG_RTCYR	0x5f /* Year */
#define PCF50633_REG_RTCSCA	0x60 /* Alarm Second */
#define PCF50633_REG_RTCMNA	0x61 /* Alarm Minute */
#define PCF50633_REG_RTCHRA	0x62 /* Alarm Hour */
#define PCF50633_REG_RTCWDA	0x63 /* Alarm Weekday */
#define PCF50633_REG_RTCDTA	0x64 /* Alarm Day */
#define PCF50633_REG_RTCMTA	0x65 /* Alarm Month */
#define PCF50633_REG_RTCYRA	0x66 /* Alarm Year */

#define PCF50633_F_RTC_SECOND	(1 << PCF50633_FIDX_RTC_SECOND)

struct pcf50633_rtc {
	int alarm_enabled;
	int second_enabled;

	struct rtc_device *rtc_dev;
	struct platform_device *pdev;
};

#endif

