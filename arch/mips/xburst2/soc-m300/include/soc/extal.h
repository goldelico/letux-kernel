
#ifndef __JZSOC_EXTAL_H__
#define __JZSOC_EXTAL_H__

#define JZ_EXTAL_RTC	32768     /* RTC extal freq: 32.768 KHz */
#define JZ_EXTAL	(CONFIG_EXTAL_CLOCK * 1000000)

#ifdef CONFIG_PALLADIUM_PLATFORM
#define PALLADIUM_CLK_INPURT  (500*1000000)
#endif

#endif
