#ifdef __KERNEL__
#include <linux/dma-mapping.h>

#else
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#endif

#include "error.h"
#include "mem.h"
#include "devmem.h"
#include "buffer.h"

#define DEBUG_MM_ALLOC	1
//#define DEBUG_MM_ALLOC_PRINT

static int devmem_alloc(devmem_ctx_t *ctx, devmem_info_t *di)
{
	int ret = 0;
	dma_addr_t dma_addr;
	di->coherent = 0;

	if(di->coherent) {
		di->kva = (unsigned long)dma_alloc_coherent(ctx->dev, di->length, &dma_addr, GFP_KERNEL);
	} else {
		di->kva = (unsigned long)dma_alloc_noncoherent(ctx->dev, di->length, &dma_addr, DMA_BIDIRECTIONAL, GFP_KERNEL);
	}

	if(!di->kva)
		return -ENOMEM;

	di->paddr = dma_addr;
	di->userptr = di->kva;	/* kernel mode, userptr = kernel address */
	return ret;
}

static void devmem_free(devmem_ctx_t *ctx, devmem_info_t *di)
{
	if(di->coherent) {
		dma_free_coherent(ctx->dev, di->length, (void *)di->kva, (dma_addr_t)di->paddr);
	} else {
		dma_free_noncoherent(ctx->dev, di->length, (void *)di->kva, (dma_addr_t)di->paddr, DMA_BIDIRECTIONAL);
	}
}

static void devmem_sync(devmem_ctx_t *ctx, devmem_info_t *di)
{
	if(di->coherent)
		return;

	dma_sync_single_for_device(ctx->dev, (dma_addr_t)(di->paddr), di->length, di->sync_dir);

}

#ifdef DEBUG_MM_ALLOC
struct debug_mm_alloc {
	unsigned long alloc_size[128];
	unsigned long index;

	unsigned long long total_alloc_size;
	unsigned long long total_alloc_cnt;
	unsigned long long total_free_size;
	unsigned long long total_free_cnt;
} dbg_alloc;
#endif

int devmem_ctx_init(devmem_ctx_t *ctx, devmem_ops_t *ops)
{

#ifdef DEBUG_MM_ALLOC
	memset(&dbg_alloc, 0, sizeof(struct debug_mm_alloc));
#endif

	return 0;
}

void devmem_ctx_uninit(devmem_ctx_t *ctx)
{

}


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

AVBuffer *av_buffer_create(char *va, unsigned int pa, unsigned int size)
{
	AVBuffer *buf = av_malloc(sizeof(AVBuffer));

	if(!buf)
		return NULL;

	buf->buffer = va;
	buf->buffer_pa = pa;
	buf->size = size;
	return buf;
}

void av_buffer_del(AVBuffer *buf)
{
	if(buf)
		av_free(buf);
}

AVBuffer *av_buffer_alloc(devmem_ctx_t *ctx, unsigned int size)
{
	AVBuffer *buf = av_mallocz(sizeof(AVBuffer));
	int ret;

	if(!buf) {
		return NULL;
	}

	buf->di.length = size;

	ret = devmem_alloc(ctx, &buf->di);
	if(ret < 0) {
		goto err_alloc;
	}

	buf->buffer = buf->di.userptr;
	buf->buffer_pa = buf->di.paddr;
	buf->size = size;

#ifdef DEBUG_MM_ALLOC
	dbg_alloc.total_alloc_size += buf->size;
	dbg_alloc.total_alloc_cnt ++;
#ifdef DEBUG_MM_ALLOC_PRINT
	printk("total_alloc_cnt: %llu, alloc_size: %d, total_alloc_size: %llu, total_free_size:%llu, total_free_cnt:%llu\n",
			dbg_alloc.total_alloc_cnt, size, dbg_alloc.total_alloc_size, dbg_alloc.total_free_size, dbg_alloc.total_free_cnt);
#endif
#endif


	return buf;
err_alloc:
#ifdef DEBUG_MM_ALLOC
	printk("total_alloc_cnt: %llu, alloc_size: %d, total_alloc_size: %llu, total_free_size:%llu, total_free_cnt:%llu\n",
			dbg_alloc.total_alloc_cnt, size, dbg_alloc.total_alloc_size, dbg_alloc.total_free_size, dbg_alloc.total_free_cnt);
#endif
	av_freep(buf);
	return NULL;
}



void av_buffer_sync(devmem_ctx_t *ctx, AVBuffer *buf, int dir)
{
	buf->di.sync_dir = dir;

	devmem_sync(ctx, &buf->di);
}


void av_buffer_free(devmem_ctx_t *ctx, AVBuffer *buf)
{
	if(!buf)
		return;

	if(buf->buffer) {
#ifdef DEBUG_MM_ALLOC
		dbg_alloc.total_free_size += buf->size;
		dbg_alloc.total_free_cnt ++;
#endif
		devmem_free(ctx, &buf->di);
		buf->buffer = NULL;
	}


	av_free(buf);
}

void av_buffer_dump(void)
{
#ifdef DEBUG_MM_ALLOC
	printk("total_alloc_cnt: %llu, total_alloc_size: %llu, total_free_size:%llu, total_free_cnt:%llu\n",
			dbg_alloc.total_alloc_cnt, dbg_alloc.total_alloc_size, dbg_alloc.total_free_size, dbg_alloc.total_free_cnt);
#endif

}

