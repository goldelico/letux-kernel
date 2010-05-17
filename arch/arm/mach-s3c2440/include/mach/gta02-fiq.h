#ifndef __GTA02_FIQ_H
#define __GTA02_FIQ_H

extern void gta02_fiq_handler(void);
extern void gta02_fiq_kick(void);
extern int gta02_fiq_enable(void);
extern void gta02_fiq_disable(void);

#endif
