#ifndef __GTA02HDQ_H__
#define __GTA02HDQ_H__

int gta02hdq_read(int address);
int gta02hdq_write(int address, u8 data);
int gta02hdq_initialized(void);

#endif
