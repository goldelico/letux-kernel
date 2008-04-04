#ifndef _LINUX_PCF506XX_H
#define _LINUX_PCF506XX_H


#define PMU_VRAIL_F_SUSPEND_ON	0x00000001	/* Remains on during suspend */
#define PMU_VRAIL_F_UNUSED	0x00000002	/* This rail is not used */
struct pmu_voltage_rail {
	char *name;
	unsigned int flags;
	struct {
		unsigned int init;
		unsigned int max;
	} voltage;
};

enum pmu_event {
	PMU_EVT_NONE,
	PMU_EVT_INSERT,
	PMU_EVT_REMOVE,
#ifdef CONFIG_SENSORS_PCF50633
	PMU_EVT_USB_INSERT,
	PMU_EVT_USB_REMOVE,
#endif
	__NUM_PMU_EVTS
};

typedef int pmu_cb(struct device *dev, unsigned int feature,
		   enum pmu_event event);


#endif /* !_LINUX_PCF506XX_H */
