#ifdef __KERNEL__
#else
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#endif

#include "vpu_ops.h"


static struct vpu_ctx *gvpu_ctx;

int vpu_ctx_set_ops(struct vpu_ctx *ctx, void *priv, struct vpu_ops *ops)
{
	ctx->priv = priv;
	ctx->ops = ops;

	return 0;
}

#ifdef __KERNEL__

static int vpu_start(void *priv)
{
	 return 0;
}


static int vpu_wait(void *priv)
{
	int ret = 0;

	return ret;
}


static int vpu_end(void *priv)
{
	return 0;
}


struct vpu_ops hw_vpu_ops = {
	.start = vpu_start,
	.wait = vpu_wait,
	.end = vpu_end,
};

int vpu_ctx_init(struct vpu_ctx *ctx)
{

	/*ops will be replaced.*/
	ctx->ops = &hw_vpu_ops;
	ctx->priv = ctx;
	gvpu_ctx = ctx;
	return 0;
}


void vpu_ctx_uninit(struct vpu_ctx *ctx)
{
}


#else

static int vpu_start(struct vpu_ctx *ctx)
{
	unsigned int glbc = 0;
	int ret = 0;
	/*reset vpu*/
	ret = ioctl(ctx->fd, CMD_VPU_RESET, 0);

	/* this function will set st_h264 var dma */
	glbc = (SCH_GLBC_HIAXI | SCH_INTE_ACFGERR | SCH_INTE_BSERR | SCH_INTE_ENDF);
	 *((volatile unsigned int *)(ctx->vpu_base + REG_SCH_GLBC)) = glbc;

	 /* start vmd regs config */
	 *(volatile unsigned int *)(ctx->vpu_base + REG_VDMA_TASKRG) = (ctx->desc_pa & 0xffffff80) | 0x1;

	 return 0;
}


static int vpu_wait(struct vpu_ctx *ctx)
{

	int vpu_status, ret;

	ret = ioctl(ctx->fd, WAIT_COMPLETE, &vpu_status);
	if((vpu_status & 0x1) == 1){
		//printf("wait vpu ops complete done success.\n");
	} else {
		printf("vpu sch status=0x%08x,vdma status=0x%08x,vdma dha=0x%08x, sde id=0x%08x,sde cfg0=0x%08x,sde bsaddr=0x%08x\n",
				*(volatile unsigned int *)(ctx->vpu_base + REG_SCH_STAT),
				*(volatile unsigned int *)(ctx->vpu_base + REG_VDMA_TASKST),
				*(volatile unsigned int *)(ctx->vpu_base + REG_VDMA_TASKRG),
				*(volatile unsigned int *)(ctx->vpu_base + REG_SDE_CODEC_ID),
				*(volatile unsigned int *)(ctx->vpu_base + REG_SDE_CFG0),
				*(volatile unsigned int *)(ctx->vpu_base + REG_SDE_CFG2)
		      );

		printf("sde status=0x%08x\n", *(volatile unsigned int *)(ctx->vpu_base + REG_SDE_STAT));
	};

	return ret;
}


static int vpu_end(struct vpu_ctx *ctx)
{
	printf("vpu_end -----\n");

	return 0;
}


struct vpu_ops hw_vpu_ops = {
	.start = vpu_start,
	.wait = vpu_wait,
	.end = vpu_end,
};





#define VPU_DEVICE_NAME	"/dev/jz-vpu"
#define VPU_IOSIZE	(0xF0000 - 1)
int vpu_ctx_init(struct vpu_ctx *ctx)
{
	ctx->fd = open("/dev/jz-vpu", O_RDWR);
	if(ctx->fd < 0) {
		printf("failed to open vpu devie!\n");
		return -1;
	}

	ctx->vpu_base = mmap(NULL, VPU_IOSIZE, PROT_READ | PROT_WRITE, MAP_SHARED,  ctx->fd, VPU_BASE);

	ctx->ops = &hw_vpu_ops;
	ctx->priv = ctx;
	gvpu_ctx = ctx;
	return 0;
}

void vpu_ctx_uninit(struct vpu_ctx *ctx)
{
	close(ctx->fd);

	munmap(ctx->vpu_base, VPU_IOSIZE);
}
#endif

int vpu_hw_start(struct vpu_ctx *ctx)
{
	return ctx->ops->start(ctx->priv);
}
int vpu_hw_wait(struct vpu_ctx *ctx)
{
	return ctx->ops->wait(ctx->priv);
}

int vpu_hw_end(struct vpu_ctx *ctx)
{
	return ctx->ops->end(ctx->priv);
}


