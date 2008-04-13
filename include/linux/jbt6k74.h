#ifndef __JBT6K74_H__
#define __JBT6K74_H__

struct jbt6k74_platform_data {
	void (* reset)(int devindex, int level);
};

#endif
