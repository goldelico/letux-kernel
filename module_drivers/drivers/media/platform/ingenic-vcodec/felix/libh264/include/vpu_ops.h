#ifndef __VPU_OPS_H__
#define __VPU_OPS_H__

#include "api/jzm_vpu.h"

struct vpu_ops;
struct vpu_ctx;

typedef struct vpu_ctx {
	int fd;	/*vpu handle*/
	unsigned int vpu_base;
	struct vpu_ops *ops;

	unsigned int desc_va;
	unsigned int desc_pa;

	unsigned int sch_stat;
	unsigned int sde_stat;
	unsigned int error;

	void *priv;
} vpu_ctx_t;

struct vpu_ops {
	int (*start)(void *priv);
	int (*wait)(void *priv);
	int (*end)(void *priv);
};

#define WAIT_COMPLETE		0
#define CMD_VPU_RESET           105


int vpu_ctx_init(struct vpu_ctx *ctx);
void vpu_ctx_uninit(struct vpu_ctx *ctx);

int vpu_hw_start(struct vpu_ctx *ctx);

int vpu_hw_wait(struct vpu_ctx *ctx);

int vpu_hw_end(struct vpu_ctx *ctx);

int vpu_ctx_set_ops(struct vpu_ctx *ctx, void *priv, struct vpu_ops *ops);

#endif
