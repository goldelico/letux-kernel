#ifndef _LINUX_PCPA7200_H
#define _LINUX_PCPA7200_H

enum op_mode {
	SLEEP,
	WAKEUP,
	SINGLE_TOUCH,
	MULTI_TOUCH,
};

enum gesture {
	ZOOM,
	FST_ZOOM,
	SND_ZOOM,
	ROTATE,
	FST_SLIDE,
	SND_SLIDE,
};

struct pcap7200_platform_data {
	enum op_mode mode;
	void (*reset)(void);
};

#endif 	/* _LINUX_PCPA7200_H */
