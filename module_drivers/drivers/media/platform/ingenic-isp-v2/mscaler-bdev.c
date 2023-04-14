#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig-ingenic.h>

#include "helix_drv.h"
#include "jpge.h"

#include "isp-drv.h"


int ms_bdev_qbuf(struct mscaler_backend_device *ms_bdev, struct isp_video_buffer *isp_buffer)
{

	unsigned long flags;

//	printk("--------%s, %d\n", __func__, __LINE__);

	spin_lock_irqsave(&ms_bdev->lock, flags);

	list_add_tail(&isp_buffer->list_entry, &ms_bdev->processing_list);

	spin_unlock_irqrestore(&ms_bdev->lock, flags);

	/* wakeup possible process thread. */
	wake_up(&ms_bdev->wq);
}


static int ms_bdev_kthread(void *data)
{

	struct mscaler_backend_device *ms_bdev = data;
	int ret = 0;
	unsigned long flags;

	struct ingenic_venc_ctx *ctx = ms_bdev->ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;

	struct video_frame_buffer src_frame;

	while(1) {

		if(kthread_should_stop())
			break;


		ret = wait_event_interruptible(ms_bdev->wq, !list_empty(&ms_bdev->processing_list) || ms_bdev->activated == 0);
		if(ret || ms_bdev->activated ==  0) {
			/*wakeup by interrupt?*/
			/* STOP*/
			continue;
		}

		spin_lock_irqsave(&ms_bdev->lock, flags);
		struct isp_video_buffer *isp_buffer = list_first_entry_or_null(&ms_bdev->processing_list, struct isp_video_buffer, list_entry);

		if(!isp_buffer) {
			dev_err(ms_bdev->dev, "No Buffer available!\n");
		} else {

			list_del(&isp_buffer->list_entry);
		}
		spin_unlock_irqrestore(&ms_bdev->lock, flags);

		struct vb2_v4l2_buffer *vb2 = &isp_buffer->vb2;


		mutex_lock(&ctx->dev->dev_mutex);
		/* Processing ....*/

		memset(&src_frame, 0, sizeof(struct video_frame_buffer));

		src_frame.num_planes = 2;
		src_frame.fb_addr[0].pa = ingenic_vb2_dma_contig_plane_dma_addr(&vb2->vb2_buf, 0);
		src_frame.fb_addr[1].pa = src_frame.fb_addr[0].pa + isp_buffer->uv_offset;

		src_frame.fb_addr[0].va = vb2_plane_vaddr(&vb2->vb2_buf, 0);
		src_frame.fb_addr[1].va = src_frame.fb_addr[0].va + isp_buffer->uv_offset;

		src_frame.fb_addr[0].size = isp_buffer->uv_offset;
		src_frame.fb_addr[1].size = vb2_get_plane_payload(&vb2->vb2_buf, 0) - src_frame.fb_addr[0].size;

		dma_sync_single_for_device(ms_bdev->dev,(unsigned long)jpge_ctx->bs->pa, jpge_ctx->bs->size, DMA_FROM_DEVICE);

		jpeg_encoder_encode(jpge_ctx, &src_frame, jpge_ctx->bs);

		memcpy(vb2_plane_vaddr(&vb2->vb2_buf, 0), jpge_ctx->bs->va, jpge_ctx->bslen);

		vb2_set_plane_payload(&vb2->vb2_buf, 0, jpge_ctx->bslen);

#if 0
		wait_process_buffer();

		done_process_buffer();
#endif
		mutex_unlock(&ctx->dev->dev_mutex);
//		printk("----done_processing_buffer: %d\n", isp_buffer->vb2.vb2_buf.index);
		vb2_buffer_done(&isp_buffer->vb2.vb2_buf, VB2_BUF_STATE_DONE);

	}

	return 0;
}


static int venc_create_ctx(struct mscaler_backend_device *ms_bdev)
{

	struct ingenic_venc_ctx *ctx = kzalloc(sizeof(struct ingenic_venc_ctx), GFP_KERNEL);
	struct h264e_ctx * h264e_ctx = NULL;
	struct jpge_ctx * jpge_ctx = NULL;
	struct ingenic_vcodec_mem *bs = NULL;
	int ret = 0;

	if(ctx == NULL) {
		return -ENOMEM;
	}

	ctx->dev = ingenic_venc_dev_get();

	mutex_lock(&ctx->dev->dev_mutex);

#if 0
	if (!ctx->dev->id_counter) {
		vpu_on(ctx->dev);
	}
#endif

	ctx->id = ctx->dev->id_counter++;
	init_waitqueue_head(&ctx->queue);

	mutex_unlock(&ctx->dev->dev_mutex);

	ms_bdev->ctx = ctx;

	/*switch FMT: JPEG H264.*/
	jpge_ctx = &ctx->jpge_ctx;
	ctx->codec_id = CODEC_ID_JPGE;

	ret = jpeg_encoder_init(jpge_ctx);
	jpeg_encoder_set_priv(jpge_ctx, ctx);


	jpeg_encoder_set_fmt(jpge_ctx, ms_bdev->width, ms_bdev->height, HELIX_NV12_MODE);

	ret = jpeg_encoder_alloc_workbuf(jpge_ctx);
	if(ret < 0) {
		goto err_alloc_workbuf;
	}

	/* alloc bs output buffer. */
	bs = kzalloc(sizeof(struct ingenic_vcodec_mem), GFP_KERNEL);
	if(!bs) {
		goto err_bs_alloc;
	}
	bs->size = ms_bdev->width * ms_bdev->height;

	bs->va = dma_alloc_noncoherent(ms_bdev->dev, bs->size, &bs->pa, GFP_KERNEL);
	if(!bs->va) {
		goto err_bs_va_alloc;
	}

	jpge_ctx->bs = bs;


	return 0;
err_bs_va_alloc:
	kfree(bs);
err_bs_alloc:
	jpeg_encoder_free_workbuf(jpge_ctx);
err_alloc_workbuf:
	jpeg_encoder_deinit(jpge_ctx);

	kfree(ctx);
	ms_bdev->ctx = NULL;
	return ret;
}

static int venc_free_ctx(struct mscaler_backend_device *ms_bdev)
{
	struct ingenic_venc_ctx *ctx = ms_bdev->ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct ingenic_vcodec_mem *bs = jpge_ctx->bs;



	mutex_lock(&ctx->dev->dev_mutex);
#if 0
	if (!--ctx->dev->id_counter) {
		vpu_off(ctx->dev);
	}
#endif
	mutex_unlock(&ctx->dev->dev_mutex);

	dma_free_noncoherent(ms_bdev->dev, bs->size, bs->va, bs->pa);
	kfree(bs);

	jpeg_encoder_free_workbuf(jpge_ctx);
	jpeg_encoder_deinit(jpge_ctx);

	kfree(ctx);
}


int ms_bdev_init(struct mscaler_backend_device *ms_bdev)
{
	int ret = 0;
	printk("--------%s, %d\n", __func__, __LINE__);

	ret = venc_create_ctx(ms_bdev);
	if(ret < 0) {
		return -EINVAL;
	}

	spin_lock_init(&ms_bdev->lock);
	INIT_LIST_HEAD(&ms_bdev->processing_list);
	init_waitqueue_head(&ms_bdev->wq);

	ms_bdev->process_thread = kthread_run(ms_bdev_kthread, ms_bdev, "ms-bdev-ch%d", ms_bdev->ch);
	if(IS_ERR_OR_NULL(ms_bdev->process_thread)) {

		dev_err(ms_bdev->dev, "Failed to run kthread for ms-bdev-ch%d", ms_bdev->ch);

		return -EINVAL;
	}

	return 0;
}

int ms_bdev_deinit(struct mscaler_backend_device *ms_bdev)
{

	printk("--------%s, %d\n", __func__, __LINE__);

	if(!ms_bdev->activated) {
		return 0;
	}


	kthread_stop(ms_bdev->process_thread);

	ms_bdev->process_thread = NULL;

	venc_free_ctx(ms_bdev);

	memset(ms_bdev, 0, sizeof(struct mscaler_backend_device));

	return 0;
}
