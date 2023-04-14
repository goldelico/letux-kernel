#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "devmem.h"

#define DMA_TO_DEVICE		DEVMEM_SYNC_TO_DEVICE
#define DMA_FROM_DEVICE 	DEVMEM_SYNC_FROM_DEVICE

struct devmem_ctx;

typedef struct devmem_ops {
	int (*alloc)(struct devmem_ctx *ctx, devmem_info_t *di);
	void (*free)(struct devmem_ctx *ctx, devmem_info_t *di);
	void (*sync)(struct devmem_ctx *ctx, devmem_info_t *di);

} devmem_ops_t;

typedef struct devmem_ctx {
	int fd;
	devmem_ops_t *ops;
	struct device *dev;
} devmem_ctx_t;

int devmem_ctx_init(devmem_ctx_t *ctx, devmem_ops_t *ops);
void devmem_ctx_uninit(devmem_ctx_t *ctx);




typedef struct AVBuffer {
	unsigned char *buffer;
	unsigned int buffer_pa;	/* physical address of buffer. */
	unsigned int size;

	devmem_info_t di;	/*mem info used by devmem allocator.*/
} AVBuffer;




/*
 * create: 	use exists buffer fill AVBuffer struct.
 * alloc:	allocate for AVBuffer.
 * free:	free AVBuffer.
 *
 * */


/**
 * @brief 使用已经分配好的内存，创建AVBuffer结构体.
 * 	  caller: 必须要保证内存有效.
 *
 * @param va  va 是虚拟地址
 * @param pa  pa 是对应的物理地址，给到dma使用.
 * @param size
 * @param buf 输出buf.
 *
 * @return
 */
AVBuffer *av_buffer_create(char *va, unsigned int pa, unsigned int size);
void av_buffer_del(AVBuffer *buf);

AVBuffer *av_buffer_alloc(devmem_ctx_t *ctx, unsigned int size);


void av_buffer_sync(devmem_ctx_t *ctx, AVBuffer *buf, int dir);


void av_buffer_free(devmem_ctx_t *ctx, AVBuffer *buf);
#endif
