//v4l2_video.c --> export to v4l2 subsystem.
//support hw acc functions:
// CSC:
//	YUV -> RGB; RGB -> YUV
// Scale:
//	Scale functions.
//
// v4l2 m2m frameworks.
// Output : mmap/userptr/dmabuf.
//	userptr can use dmmu for input.
// Capture: mmap/userptr/dmabuf.


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/platform_device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/ingenic_video_nr.h>

#include "hw_composer.h"
#include "hw_composer_v4l2.h"

#define DPU_DRIVER_NAME "dpu-csc-scaler"


static struct dpu_fmt formats[] = {
	{
		.name	= "NV12",
		.fourcc	= V4L2_PIX_FMT_NV12,
		.layer_fmt = LAYER_CFG_FORMAT_NV12,
		.depth	= 16,
		.types	= MEM2MEM_OUTPUT,
	},
	{
		.name	= "NV21",
		.fourcc	= V4L2_PIX_FMT_NV21,
		.layer_fmt = LAYER_CFG_FORMAT_NV21,
		.depth	= 16,
		.types	= MEM2MEM_OUTPUT,
	},
	{
		.name	= "YUV 422",
		.fourcc	= V4L2_PIX_FMT_VYUY,
		.layer_fmt = LAYER_CFG_FORMAT_YUV422,
		.depth	= 16,
		.types	= MEM2MEM_OUTPUT,
	},
	{//TODO
		.name	= "MONO8",
		.fourcc	= V4L2_PIX_FMT_SBGGR8,
		.depth	= 16,
		.types	= MEM2MEM_OUTPUT,
	},
	{//TODO
		.name = "MONO16",
		.fourcc = V4L2_PIX_FMT_SBGGR16,
		.depth = 16,
		.types = MEM2MEM_OUTPUT,
	},
	/*input and wback formts.*/
	{
		.name	= "RGB_565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.layer_fmt = LAYER_CFG_FORMAT_RGB565,
		.wb_fmt = DC_WB_FORMAT_565,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.name	= "RGB_888",
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.layer_fmt = LAYER_CFG_FORMAT_RGB888,
		.wb_fmt = DC_WB_FORMAT_888,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.name	= "ARGB_8888",
		.fourcc	= V4L2_PIX_FMT_ARGB32,
		.layer_fmt = LAYER_CFG_FORMAT_ARGB8888,
		.wb_fmt = DC_WB_FORMAT_8888,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

static struct dpu_fmt *find_fmt(struct v4l2_format *f)
{
	unsigned int i;
	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix.pixelformat)
			return &formats[i];
	}
	return NULL;
}

static int vidioc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strncpy(cap->driver, DPU_DRIVER_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, DPU_DRIVER_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(4, 4, 19);
	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	cap->capabilities =  cap->device_caps |
		V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT |
		V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static inline struct ingenic_dpu_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct ingenic_dpu_ctx, fh);
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct dpu_fmt *fmt;

	num = 0;
	for (i = 0; i < NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		fmt = &formats[i];
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct dpu_fmt *fmt;
	enum v4l2_field *field;

	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;

	field = &f->fmt.pix.field;
	if (*field == V4L2_FIELD_ANY)
		*field = V4L2_FIELD_NONE;
	else if (*field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (f->fmt.pix.width > MAX_WIDTH
		|| f->fmt.pix.height > MAX_HEIGHT
		|| f->fmt.pix.width < MIN_WIDTH
		|| f->fmt.pix.height < MIN_HEIGHT) {
		return -EINVAL;
	}

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
	return 0;
}
static struct dpu_frame_info *get_frame(struct ingenic_dpu_ctx *ctx,
		enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->out_info;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->cap_info;
	default:
		return ERR_PTR(-EINVAL);
	}
}
static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_OUTPUT);
}

static int vidioc_g_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct ingenic_dpu_ctx *ctx = fh_to_ctx(prv);
	struct vb2_queue *vq;
	struct dpu_frame_info *frame;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;
	frame = get_frame(ctx, f->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	f->fmt.pix.width		= frame->width;
	f->fmt.pix.height		= frame->height;
	f->fmt.pix.field		= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat		= frame->fmt->fourcc;
	f->fmt.pix.bytesperline		= (frame->width * frame->fmt->depth) >> 3;
	f->fmt.pix.sizeimage		= frame->size;
	return 0;
}

static int vidioc_s_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct ingenic_dpu_ctx *ctx = fh_to_ctx(prv);
	struct ingenic_dpu_dev *dev = ctx->dpu_dev;
	struct vb2_queue *vq;
	struct dpu_frame_info *frame;
	struct dpu_fmt *fmt;
	int ret = 0;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format. */
	ret = vidioc_try_fmt(file, prv, f);
	if (ret)
		return ret;
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}
	frame = get_frame(ctx, f->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);
	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;
	frame->width	= f->fmt.pix.width;
	frame->height	= f->fmt.pix.height;
	frame->size		= f->fmt.pix.sizeimage;
	frame->fmt		= fmt;
	frame->bytesperline	= f->fmt.pix.bytesperline;
	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct ingenic_dpu_ctx *ctx = priv;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct ingenic_dpu_ctx *ctx = priv;

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int dpu_queue_setup(struct vb2_queue *vq,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], struct device *alloc_ctxs[])
{
	struct ingenic_dpu_ctx *ctx = vb2_get_drv_priv(vq);
	struct dpu_frame_info *frame = get_frame(ctx, vq->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);

	*nplanes = 1;
	sizes[0] = frame->size;
	alloc_ctxs[0] = (struct device *)ctx->dpu_dev->alloc_ctx;

	if (*nbuffers == 0)
		*nbuffers = 1;

	return 0;
}

static int dpu_s_ctrl(struct v4l2_ctrl *ctrl)
{
	//struct ingenic_dpu_ctx *ctx = container_of(ctrl->handler, struct ingenic_dpu_ctx,ctrl_handler);
	//TODO
	return 0;
}

static const struct v4l2_ctrl_ops dpu_ctrl_ops = {
	.s_ctrl		= dpu_s_ctrl,
};

static int dpu_setup_ctrls(struct ingenic_dpu_ctx *ctx)
{
	struct ingenic_dpu_dev *dev = ctx->dpu_dev;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);

	// TODO...
	ctx->ctrl_hflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &dpu_ctrl_ops,
						V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctx->ctrl_vflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &dpu_ctrl_ops,
						V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->ctrl_rot = v4l2_ctrl_new_std(&ctx->ctrl_handler, &dpu_ctrl_ops,
						V4L2_CID_ROTATE, 0, 270, 90, 0);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;
		v4l2_err(&dev->v4l2_dev, "rot_setup_ctrls failed\n");
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

static int dpu_buf_prepare(struct vb2_buffer *vb)
{
	struct ingenic_dpu_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct dpu_frame_info *frame = get_frame(ctx, vb->vb2_queue->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);
	vb2_set_plane_payload(vb, 0, frame->size);
	return 0;
}

static void dpu_buf_queue(struct vb2_buffer *vb)
{
	struct ingenic_dpu_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (ctx->m2m_ctx)
		v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static struct vb2_ops dpu_qops = {
	.queue_setup	= dpu_queue_setup,
	.buf_prepare	= dpu_buf_prepare,
	.buf_queue	= dpu_buf_queue,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
						struct vb2_queue *dst_vq)
{
	struct ingenic_dpu_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->ops = &dpu_qops;
	src_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dpu_dev->dev_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &dpu_qops;
	dst_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dpu_dev->dev_mutex;

	return vb2_queue_init(dst_vq);
}

static void set_default_fmt(struct ingenic_dpu_ctx *ctx)
{
	struct dpu_frame_info *cap, *out;

	cap = &ctx->cap_info;
	out = &ctx->out_info;

	cap->width = DEFAULT_WIDTH;
	cap->height = DEFAULT_HEIGHT;
	cap->fmt = &formats[0];
	cap->bytesperline = DEFAULT_WIDTH * cap->fmt->depth >> 3;
	cap->size = cap->bytesperline * cap->height;

	out->width = DEFAULT_WIDTH;
	out->height = DEFAULT_HEIGHT;
	out->fmt = &formats[0];
	out->bytesperline = DEFAULT_WIDTH * out->fmt->depth >> 3;
	out->size = out->bytesperline * out->height;
}


static int dpu_open(struct file *file)
{
	struct ingenic_dpu_dev *dev = video_drvdata(file);
	struct ingenic_dpu_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&dev->dev_mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}

	ctx->hw_comp = hw_composer_create(ctx);
	if(!ctx->hw_comp) {
		goto err_hw_comp;
	}

	ctx->dpu_dev = dev;

	/* Set default formats */
	set_default_fmt(ctx);

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);


	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto err;
	}
	ctx->fh.m2m_ctx = ctx->m2m_ctx;


	ret = dpu_setup_ctrls(ctx);
	if(ret)
		goto err;

	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	mutex_unlock(&dev->dev_mutex);

	return 0;
err:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
err_hw_comp:
	mutex_unlock(&dev->dev_mutex);
	kfree(ctx);
	return ret;
}

static int dpu_release(struct file *file)
{
	struct ingenic_dpu_dev *dev = video_drvdata(file);
	struct ingenic_dpu_ctx *ctx = fh_to_ctx(file->private_data);

	mutex_lock(&dev->dev_mutex);

	hw_composer_destroy(ctx->hw_comp);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&dev->dev_mutex);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

/*
 初始化comp_info，
该驱动主要使用layer0进行CSC和缩放操作.
将frame_cfg 和 layer_cfg 都初始化，
在实际执行叠加时，只更新涉及部分.
*/
static int hw_comp_v4l2_init_info(struct ingenic_dpu_ctx *ctx)
{
	struct comp_setup_info *comp_info = &ctx->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	int i = 0;

	memset(comp_info, 0, sizeof(struct comp_setup_info));

	for(i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		lay_cfg[i].lay_z_order  = i; /* top */
	}

	return 0;
}

static int hw_comp_v4l2_update_info(struct ingenic_dpu_ctx *ctx, struct vb2_v4l2_buffer *src, struct vb2_v4l2_buffer *dst)
{
	struct comp_setup_info *comp_info = &ctx->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct wback_cfg *wback_info = &frm_cfg->wback_info;

	struct dpu_frame_info *out_info = &ctx->out_info;
	struct dpu_frame_info *cap_info = &ctx->cap_info;

	// TODO, handle with userptr address.
	unsigned long src_paddr = ingenic_vb2_dma_contig_plane_dma_addr(&src->vb2_buf,0);
	unsigned long dst_paddr = ingenic_vb2_dma_contig_plane_dma_addr(&dst->vb2_buf,0);

	unsigned int src_w = out_info->width;
	unsigned int src_h = out_info->height;
	unsigned int src_fmt = out_info->fmt->layer_fmt;
	unsigned int dst_w = cap_info->width;
	unsigned int dst_h = cap_info->height;
	unsigned int dst_fmt = cap_info->fmt->wb_fmt;

	lay_cfg[0].lay_en = 1;
	lay_cfg[0].source_w = src_w;
	lay_cfg[0].source_h = src_h;
	lay_cfg[0].disp_pos_x = 0;
	lay_cfg[0].disp_pos_y = 0;

	if(src_w != dst_w || src_h != dst_h) {
		lay_cfg[0].scale_w = dst_w;
		lay_cfg[0].scale_h = dst_h;
		lay_cfg[0].lay_scale_en = 1;
	}

	lay_cfg[0].g_alpha_en = 0;
	lay_cfg[0].g_alpha_val = 128;
	lay_cfg[0].stride = src_w;
	lay_cfg[0].uv_stride = src_w;

	lay_cfg[0].color = LAYER_CFG_COLOR_RGB; //TODO: export ctrls to user?
	lay_cfg[0].format = src_fmt;

	lay_cfg[0].addr[0] = src_paddr;
	lay_cfg[0].uv_addr[0] = src_paddr + src_w * src_h;
	lay_cfg[0].tlb_en = 0; // TODO. for USERPTR Mode


	//frm_cfg
	frm_cfg->width = dst_w;
	frm_cfg->height = dst_h;

	//wback_cfg
	wback_info->en = 1;
	wback_info->fmt = dst_fmt; //TODO.
	wback_info->addr = dst_paddr; //TODO
	wback_info->stride = dst_w; // TODO
	wback_info->dither_en = 0;

	//printk("src_paddr: %x, src_w: %d, src_h: %d\n", src_paddr, src_w, src_h);
	//printk("dst_paddr: %x, dst_w: %d, dst_h: %d\n", dst_paddr, dst_w, dst_h);

	return 0;
}

static void device_run(void *prv)
{
	struct ingenic_dpu_ctx *ctx = prv;
	struct ingenic_dpu_dev *dev = ctx->dpu_dev;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if(unlikely(src == NULL)) {
		printk("DPU:src == NULL\n");
	}
	dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if(unlikely(dst == NULL)) {
		printk("DPU:dst == NULL\n");
	}
	hw_composer_lock(ctx->hw_comp);

	hw_comp_v4l2_init_info(ctx);
	hw_comp_v4l2_update_info(ctx, src, dst);
	hw_composer_setup(ctx->hw_comp, &ctx->comp_info);

	hw_composer_start(ctx->hw_comp); //block compose.
	hw_composer_unlock(ctx->hw_comp);


	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);

	v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);
}



static void job_abort(void *prv)
{
	printk("----abort\n");
}

static const struct v4l2_file_operations dpu_fops = {
	.owner		= THIS_MODULE,
	.open		= dpu_open,
	.release	= dpu_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops dpu_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt,

	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_out		= vidioc_try_fmt,
	.vidioc_s_fmt_vid_out		= vidioc_s_fmt,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,

	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,
};


static struct video_device dpu_videodev = {
	.name		= "dpu-hwcomp",
	.fops		= &dpu_fops,
	.ioctl_ops	= &dpu_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
	.device_caps	= V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M,
};

static struct v4l2_m2m_ops dpu_m2m_ops = {
	.device_run	= device_run,
	.job_abort	= job_abort,
};


void * hw_comp_v4l2_init(struct device *dev)
{
	struct ingenic_dpu_dev *dpu_dev;
	struct video_device *vfd;
	int ret = 0;

	dpu_dev = devm_kzalloc(dev, sizeof(*dpu_dev), GFP_KERNEL);
	if (!dpu_dev)
		return NULL;

	dpu_dev->dev = dev;
	dpu_dev->name = dpu_videodev.name;

	mutex_init(&dpu_dev->dev_mutex);

	dpu_dev->alloc_ctx = ingenic_vb2_dma_contig_init_ctx(dpu_dev->dev);
	if (IS_ERR(dpu_dev->alloc_ctx)) {
		ret = PTR_ERR(dpu_dev->alloc_ctx);
	}

	ret = v4l2_device_register(dpu_dev->dev, &dpu_dev->v4l2_dev);
	if (ret)
		goto alloc_ctx_cleanup;


	vfd = &dpu_videodev;
	vfd->lock = &dpu_dev->dev_mutex;
	vfd->v4l2_dev = &dpu_dev->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, INGENIC_DPU_COMP_VIDEO_NR);
	if (ret) {
		v4l2_err(&dpu_dev->v4l2_dev, "Failed to register video device\n");
		goto free_video_device;
	}

	video_set_drvdata(vfd, dpu_dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", dpu_videodev.name);
	dpu_dev->vfd = vfd;
	v4l2_info(&dpu_dev->v4l2_dev, "device registered as /dev/video%d\n",vfd->num);

	dpu_dev->m2m_dev = v4l2_m2m_init(&dpu_m2m_ops);

	if (IS_ERR(dpu_dev->m2m_dev)) {
		v4l2_err(&dpu_dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dpu_dev->m2m_dev);
		goto unreg_video_dev;
	}

	return dpu_dev;

unreg_video_dev:
	video_unregister_device(vfd);
free_video_device:
	v4l2_device_unregister(&dpu_dev->v4l2_dev);
alloc_ctx_cleanup:
	ingenic_vb2_dma_contig_cleanup_ctx(dpu_dev->alloc_ctx);
	return NULL;
}

int hw_comp_v4l2_exit(void *priv)
{
	struct ingenic_dpu_dev *dev = (struct ingenic_dpu_dev *)priv;

	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
	video_device_release(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	ingenic_vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);

	return 0;
}
