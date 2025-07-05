#ifndef __TIZIANO_SYS_H__
#define __TIZIANO_SYS_H__

extern int system_reg_write(void *hdl, unsigned int reg, unsigned int value);
extern unsigned int  system_reg_read(void *hdl, unsigned int reg);
extern int system_lock(void *hdl);
extern int system_unlock(void *hdl);
extern int system_irq_func_set(void *hdl, int irq, void *func, void *data);

#endif
