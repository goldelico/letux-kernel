#ifndef __TIZIANO_SYS_H__
#define __TIZIANO_SYS_H__

extern int system_reg_write(unsigned int reg, unsigned int value);
extern unsigned int  system_reg_read(unsigned int reg);
extern int system_lock(void);
extern int system_unlock(void);
extern int system_irq_func_main_set(int irq, void *func);
extern int system_irq_func_sec_set(int irq, void *func);

#endif
