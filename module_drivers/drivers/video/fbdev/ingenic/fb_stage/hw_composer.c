#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>

#include "hw_composer.h"

struct hw_composer_master *hw_comp_master;

struct hw_composer_master * hw_composer_init(struct dpu_ctrl *dctrl)
{
	int ret = 0;

	hw_comp_master = kzalloc(sizeof(struct hw_composer_master), GFP_KERNEL);
	if(IS_ERR_OR_NULL(hw_comp_master)) {
		return -ENOMEM;
	}

	hw_comp_master->dctrl = dctrl;

	/*comp mutex??*/
	mutex_init(&hw_comp_master->lock);

	return hw_comp_master;
}
EXPORT_SYMBOL(hw_composer_init);


void hw_composer_exit(struct hw_composer_master *comp_master)
{
	mutex_destroy(&comp_master->lock);

	if(comp_master) {
		kfree(comp_master);
	}
}
EXPORT_SYMBOL(hw_composer_exit);

struct hw_composer_ctx *hw_composer_create(void *priv_data)
{
	struct hw_composer_ctx *ctx = kzalloc(sizeof(struct hw_composer_ctx), GFP_KERNEL);

	if(IS_ERR_OR_NULL(ctx)) {
		return -ENOMEM;
	}
	/*设置ctx的master.*/
	ctx->master = hw_comp_master;

	ctx->block = 1;

	INIT_LIST_HEAD(&ctx->list);

	/*添加到ctx链表.*/

	return ctx;
}
EXPORT_SYMBOL(hw_composer_create);

int hw_composer_destroy(struct hw_composer_ctx *ctx)
{

	list_del(&ctx->list);
	kfree(ctx);

	return 0;
}
EXPORT_SYMBOL(hw_composer_destroy);

void hw_composer_lock(struct hw_composer_ctx *ctx)
{
	mutex_lock(&ctx->master->lock);
}
EXPORT_SYMBOL(hw_composer_lock);

void hw_composer_unlock(struct hw_composer_ctx *ctx)
{
	mutex_unlock(&ctx->master->lock);
}
EXPORT_SYMBOL(hw_composer_unlock);

int hw_composer_setup(struct hw_composer_ctx *ctx, struct comp_setup_info *comp_info)
{
	/*TODO: check comp_info.*/

	memcpy(&ctx->comp_info, comp_info, sizeof(struct comp_setup_info));
	return 0;
}
EXPORT_SYMBOL(hw_composer_setup);


int hw_composer_start(struct hw_composer_ctx *ctx)
{
	struct hw_composer_master *master = ctx->master;
	struct dpu_ctrl *dctrl = master->dctrl;
	int ret = 0;

	ret = dpu_ctrl_comp_setup(dctrl, &ctx->comp_info);
	ret = dpu_ctrl_comp_start(dctrl, ctx->block);

	return ret;
}
EXPORT_SYMBOL(hw_composer_start);

int hw_composer_stop(struct hw_composer_ctx *ctx)
{
	struct hw_composer_master *master = ctx->master;
	struct dpu_ctrl *dctrl = master->dctrl;
	int ret = 0;

	// TODO:
	dpu_ctrl_comp_stop(dctrl, QCK_STOP);

	return ret;
}
EXPORT_SYMBOL(hw_composer_stop);
