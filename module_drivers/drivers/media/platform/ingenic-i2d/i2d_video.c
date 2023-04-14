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

#include "i2d_video.h"
#include "ingenic_i2d.h"

#define JZ_I2D_NAME		"jz-i2d"


static struct i2d_fmt formats[] = {
	{
		.name	= "YUV 420",
		.fourcc	= V4L2_PIX_FMT_NV12,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.name	= "RAW8",
		.fourcc	= V4L2_PIX_FMT_SBGGR8,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.name	= "RGB_565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
	{
		.name	= "ARGB_8888",
		.fourcc	= V4L2_PIX_FMT_ARGB32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

static struct i2d_fmt *find_fmt(struct v4l2_format *f)
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
	strncpy(cap->driver, JZ_I2D_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, JZ_I2D_NAME, sizeof(cap->card) - 1);
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

static inline struct ingenic_i2d_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct ingenic_i2d_ctx, fh);
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct i2d_fmt *fmt;

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
	struct i2d_fmt *fmt;
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
static struct i2d_frame_info *get_frame(struct ingenic_i2d_ctx *ctx,
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
	struct ingenic_i2d_ctx *ctx = fh_to_ctx(prv);
	struct vb2_queue *vq;
	struct i2d_frame_info *frame;

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
	struct ingenic_i2d_ctx *ctx = fh_to_ctx(prv);
	struct ingenic_i2d_dev *dev = ctx->i2d_dev;
	struct vb2_queue *vq;
	struct i2d_frame_info *frame;
	struct i2d_fmt *fmt;
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
	struct ingenic_i2d_ctx *ctx = priv;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct ingenic_i2d_ctx *ctx = priv;

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int i2d_queue_setup(struct vb2_queue *vq,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct ingenic_i2d_ctx *ctx = vb2_get_drv_priv(vq);
	struct i2d_frame_info *frame = get_frame(ctx, vq->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);

	*num_planes = 1;
	sizes[0] = frame->size;
	alloc_devs[0] = ctx->i2d_dev->dev;

	if (*num_buffers == 0)
		*num_buffers = 1;

	return 0;
}

static int i2d_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ingenic_i2d_ctx *ctx = container_of(ctrl->handler, struct ingenic_i2d_ctx,ctrl_handler);
	unsigned long flags;

	spin_lock_irqsave(&ctx->i2d_dev->ctrl_lock, flags);
	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctx->hflip = ctx->ctrl_hflip->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctx->ctrl_vflip->val;
		break;
	case V4L2_CID_ROTATE:
		ctx->angle = ctx->ctrl_rot->val;
		break;
	}
	spin_unlock_irqrestore(&ctx->i2d_dev->ctrl_lock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops i2d_ctrl_ops = {
	.s_ctrl		= i2d_s_ctrl,
};

static int i2d_setup_ctrls(struct ingenic_i2d_ctx *ctx)
{
	struct ingenic_i2d_dev *dev = ctx->i2d_dev;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);

	ctx->ctrl_hflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &i2d_ctrl_ops,
						V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctx->ctrl_vflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &i2d_ctrl_ops,
						V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->ctrl_rot = v4l2_ctrl_new_std(&ctx->ctrl_handler, &i2d_ctrl_ops,
						V4L2_CID_ROTATE, 0, 270, 90, 0);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;
		v4l2_err(&dev->v4l2_dev, "rot_setup_ctrls failed\n");
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

static int i2d_buf_prepare(struct vb2_buffer *vb)
{
	struct ingenic_i2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct i2d_frame_info *frame = get_frame(ctx, vb->vb2_queue->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);
	vb2_set_plane_payload(vb, 0, frame->size);
	return 0;
}

static void i2d_buf_queue(struct vb2_buffer *vb)
{
	struct ingenic_i2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (ctx->m2m_ctx)
		v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static struct vb2_ops i2d_qops = {
	.queue_setup	= i2d_queue_setup,
	.buf_prepare	= i2d_buf_prepare,
	.buf_queue	= i2d_buf_queue,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
						struct vb2_queue *dst_vq)
{
	struct ingenic_i2d_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->ops = &i2d_qops;
	src_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->i2d_dev->dev_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &i2d_qops;
	dst_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->i2d_dev->dev_mutex;

	return vb2_queue_init(dst_vq);
}

static void set_default_fmt(struct ingenic_i2d_ctx *ctx)
{
	struct i2d_frame_info *cap, *out;

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


static int i2d_open(struct file *file)
{
	struct ingenic_i2d_dev *dev = video_drvdata(file);
	struct ingenic_i2d_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&dev->dev_mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	ctx->i2d_dev = dev;

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

	ret = i2d_setup_ctrls(ctx);
	if(ret)
		goto err;

	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	clk_prepare_enable(dev->clk);

	mutex_unlock(&dev->dev_mutex);

	return 0;
err:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&dev->dev_mutex);
	kfree(ctx);
	return ret;
}

static int i2d_release(struct file *file)
{
	struct ingenic_i2d_dev *dev = video_drvdata(file);
	struct ingenic_i2d_ctx *ctx = fh_to_ctx(file->private_data);

	clk_disable_unprepare(dev->clk);

	mutex_lock(&dev->dev_mutex);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&dev->dev_mutex);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static irqreturn_t i2d_irq_handler(int irq, void *prv)
{
	struct ingenic_i2d_dev *dev = prv;
	struct ingenic_i2d_ctx *ctx = dev->cur_ctx;
	struct vb2_v4l2_buffer *src, *dst;

	unsigned int status = i2d_irq_clear(dev);

	if(unlikely(ctx == NULL)) {
		printk("I2D:ctx == NULL\n");
		return IRQ_HANDLED;
	}

	src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if(unlikely(src == NULL)) {
		printk("I2D:src == NULL\n");
		return IRQ_HANDLED;
	}
	dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if(unlikely(dst == NULL)) {
		printk("I2D:dst == NULL\n");
		return IRQ_HANDLED;
	}

	/* dst->timecode = src->timecode; */
	/* dst->timestamp = src->timestamp; */

	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);

	dev->cur_ctx = NULL;
	if(status & 0x1)
		wake_up(&dev->irq_queue);
	return IRQ_HANDLED;
}

static void device_run(void *prv)
{
	struct ingenic_i2d_ctx *ctx = prv;
	struct ingenic_i2d_dev *dev = ctx->i2d_dev;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned long flags;
	unsigned int srcaddr,dstaddr;
	dev->cur_ctx = ctx;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	spin_lock_irqsave(&dev->ctrl_lock, flags);

	/* clk_prepare_enable(dev->clk); */
	i2d_safe_reset(dev);
	srcaddr = ingenic_vb2_dma_contig_plane_dma_addr(&src->vb2_buf,0);
	dstaddr = ingenic_vb2_dma_contig_plane_dma_addr(&dst->vb2_buf,0);
	i2d_set_init_info(dev, &ctx->out_info,srcaddr,dstaddr);
	/* do_gettimeofday(&time_now); */
	i2d_start(dev);
	spin_unlock_irqrestore(&dev->ctrl_lock, flags);
}



#define I2D_TIMEOUT 2000
static void job_abort(void *prv)
{
	struct ingenic_i2d_ctx *ctx = prv;
	struct ingenic_i2d_dev *dev = ctx->i2d_dev;
	int ret;

	if (dev->cur_ctx == NULL) /* No job currently running */
		return;

	ret = wait_event_timeout(dev->irq_queue,dev->cur_ctx == NULL,msecs_to_jiffies(I2D_TIMEOUT));
	if(!ret) {
		struct vb2_v4l2_buffer *src_vb, *dst_vb;

		i2d_safe_reset(dev);

		src_vb = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		dst_vb = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		dst_vb->vb2_buf.timestamp = src_vb->vb2_buf.timestamp;
		dst_vb->timecode = src_vb->timecode;
		dst_vb->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
		dst_vb->flags |=
			src_vb->flags
			& V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

		v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_ERROR);

		v4l2_m2m_job_finish(dev->m2m_dev,
				    ctx->m2m_ctx);
		dev->cur_ctx = NULL;
		/* clk_disable_unprepare(dev->clk); */
	} else {
		printk("I2D : wait_event_timeout %s []%d] >>>>>>>\n",__func__,__LINE__);
	}
}

static const struct v4l2_file_operations i2d_fops = {
	.owner		= THIS_MODULE,
	.open		= i2d_open,
	.release	= i2d_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops i2d_ioctl_ops = {
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


static struct video_device i2d_videodev = {
	.name		= "ingenic-i2d",
	.fops		= &i2d_fops,
	.ioctl_ops	= &i2d_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
};

static struct v4l2_m2m_ops i2d_m2m_ops = {
	.device_run	= device_run,
	.job_abort	= job_abort,
};

static int ingenic_i2d_probe(struct platform_device *pdev)
{
	struct ingenic_i2d_dev *i2d_dev;
	struct video_device *vfd;
	struct resource *res;
	int ret = 0;
	i2d_dev = devm_kzalloc(&pdev->dev, sizeof(*i2d_dev), GFP_KERNEL);
	if (!i2d_dev)
		return -ENOMEM;
	i2d_dev->dev = &pdev->dev;
	spin_lock_init(&i2d_dev->irqlock);
	spin_lock_init(&i2d_dev->ctrl_lock);
	mutex_init(&i2d_dev->dev_mutex);
	atomic_set(&i2d_dev->num_inst,0);
	init_waitqueue_head(&i2d_dev->irq_queue);
	init_completion(&i2d_dev->done_i2d);

	res = platform_get_resource(pdev,IORESOURCE_MEM,0);

	i2d_dev->base = devm_ioremap_resource(&pdev->dev,res);
	if(IS_ERR(i2d_dev->base))
		return PTR_ERR(i2d_dev->base);

	i2d_dev->irq = platform_get_irq(pdev,0);
	if(i2d_dev->irq < 0){
		dev_err(&pdev->dev,"get i2d irq num error\n");
		return i2d_dev->irq;
	}

	i2d_dev->name = pdev->name;

	ret = devm_request_irq(i2d_dev->dev, i2d_dev->irq, i2d_irq_handler,
						IRQF_SHARED, i2d_dev->name, i2d_dev);
	if(ret < 0){
		dev_err(i2d_dev->dev,"i2d request irq error\n");
		return ret;
	}
	/* i2d_dev->irq_is_request = 0; */

	/* clocks */
	i2d_dev->clk = clk_get(&pdev->dev, "gate_i2d");
	if (IS_ERR(i2d_dev->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(i2d_dev->clk);
	}
	dev_dbg(&pdev->dev, "i2d clock source %p\n", i2d_dev->clk);


	i2d_dev->alloc_ctx = ingenic_vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(i2d_dev->alloc_ctx)) {
		ret = PTR_ERR(i2d_dev->alloc_ctx);
		goto clk_get_cleanup;
	}
	ret = v4l2_device_register(&pdev->dev, &i2d_dev->v4l2_dev);
	if (ret)
		goto alloc_ctx_cleanup;

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&i2d_dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_v4l2_dev;
	}

	*vfd = i2d_videodev;
	vfd->lock = &i2d_dev->dev_mutex;
	vfd->v4l2_dev = &i2d_dev->v4l2_dev;
	vfd->device_caps = V4L2_CAP_STREAMING |
		  V4L2_CAP_VIDEO_M2M;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, INGENIC_ROTATE_VIDEO_NR);
	if (ret) {
		v4l2_err(&i2d_dev->v4l2_dev, "Failed to register video device\n");
		goto free_video_device;
	}

	video_set_drvdata(vfd, i2d_dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", i2d_videodev.name);
	i2d_dev->vfd = vfd;
	v4l2_info(&i2d_dev->v4l2_dev, "device registered as /dev/video%d\n",vfd->num);

	platform_set_drvdata(pdev, i2d_dev);
	i2d_dev->m2m_dev = v4l2_m2m_init(&i2d_m2m_ops);

	if (IS_ERR(i2d_dev->m2m_dev)) {
		v4l2_err(&i2d_dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(i2d_dev->m2m_dev);
		goto unreg_video_dev;
	}
	printk("%s ok >>>>>>>>>>>>>>>>>>>>>>\n",__func__);
	return 0;

unreg_video_dev:
	video_unregister_device(vfd);
free_video_device:
	video_device_release(vfd);
unreg_v4l2_dev:
	v4l2_device_unregister(&i2d_dev->v4l2_dev);
alloc_ctx_cleanup:
	ingenic_vb2_dma_contig_cleanup_ctx(i2d_dev->alloc_ctx);
clk_get_cleanup:
	clk_put(i2d_dev->clk);

	return 0;
}

static int ingenic_i2d_remove(struct platform_device *pdev)
{
	struct ingenic_i2d_dev *dev = (struct ingenic_i2d_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " JZ_I2D_NAME);
	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
	video_device_release(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	ingenic_vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	clk_put(dev->clk);
	return 0;
}


static const struct of_device_id ingenic_i2d_match[] = {
	{
		.compatible = "ingenic,x2500-i2d",
		.data = NULL,
	},
	{ },
};

MODULE_DEVICE_TABLE(of, ingenic_i2d_match);


static struct platform_driver i2d_pdrv = {
	.probe		= ingenic_i2d_probe,
	.remove		= ingenic_i2d_remove,
	.driver		= {
		.owner = THIS_MODULE,
		.of_match_table	= of_match_ptr(ingenic_i2d_match),
		.name = JZ_I2D_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(i2d_pdrv);

MODULE_DESCRIPTION("X2500 i2d driver");
MODULE_LICENSE("GPL");

