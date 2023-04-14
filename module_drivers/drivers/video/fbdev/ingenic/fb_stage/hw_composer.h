#ifndef __HW_COMPOSER_H__
#define __HW_COMPOSER_H__

#include "dpu_ctrl.h"


struct hw_composer_master {
	int mode;
	struct mutex lock;
	struct dpu_ctrl *dctrl;
};

struct hw_composer_ctx {
	struct hw_composer_master *master;
	struct list_head list;
	int block;	// 当前composer工作模式，是阻塞的还是非阻塞的.

	struct comp_setup_info comp_info;	/* current comp settings.*/
};

/*hw_composer驱动初始化*/
struct hw_composer_master *hw_composer_init(struct dpu_ctrl *dctrl);

void hw_composer_exit(struct hw_composer_master *comp_master);

struct hw_composer_ctx *hw_composer_create(void *data);

int hw_composer_destroy(struct hw_composer_ctx *ctx);

int hw_composer_setup(struct hw_composer_ctx *ctx, struct comp_setup_info *comp_info);

int hw_composer_start(struct hw_composer_ctx *ctx);

int hw_composer_stop(struct hw_composer_ctx *ctx);

void hw_composer_lock(struct hw_composer_ctx *ctx);

void hw_composer_unlock(struct hw_composer_ctx *ctx);

#endif
