#ifndef __DEVMEM_H__
#define __DEVMEM_H__

typedef struct devmem_info {
	unsigned long userptr;
	unsigned long paddr;
	unsigned long length;
	unsigned long kva;
	int coherent;
	int sync_dir;
} devmem_info_t;

#define DEVMEM_MAGIC	'D'

#define DEVMEM_SYNC_NONE	0
#define DEVMEM_SYNC_TO_DEVICE	1
#define DEVMEM_SYNC_FROM_DEVICE	2

#define DEVMEM_ALLOC	_IOWR(DEVMEM_MAGIC, 10, devmem_info_t)
#define DEVMEM_FREE	_IOW(DEVMEM_MAGIC, 11, devmem_info_t)
#define DEVMEM_SYNC	_IOW(DEVMEM_MAGIC, 12, devmem_info_t)




#endif
