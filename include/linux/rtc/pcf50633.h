#ifndef __LINUX_RTC_PCF50633_H
#define __LINUX_RTC_PCF50633_H


enum pcf50633_rtc_event {
	PCF50633_RTC_EVENT_ALARM,
	PCF50633_RTC_EVENT_SECOND,
};

void pcf50633_rtc_handle_event(struct pcf50633_data *pcf,
						enum pcf50633_rtc_event evt);

#endif
