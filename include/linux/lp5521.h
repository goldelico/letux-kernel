#ifndef _LINUX_LP5521_H_
#define _LINUX_LP5521_H_

struct lp5521_platform_data {

	/* chip enable */
	void (*ext_enable)(int level);
};

#endif /* LINUX_LP5521_H */
