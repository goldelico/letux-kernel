#ifndef __LINUX_L1K002_H_
#define __LINUX_L1K002_H_

struct l1k002_platform_data {
	void (*pwr_onoff)(int level);
};

#endif /* __LINUX_L1K002_H_ */
