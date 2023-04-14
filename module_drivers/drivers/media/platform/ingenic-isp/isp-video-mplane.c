#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>
#include <linux/mutex.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig-ingenic.h>

#include "isp-drv.h"
#define ISP_VIDEO_DRIVER_NAME	"ispvideo"



static struct v4l2_subdev *
isp_video_remote_subdev(struct isp_video_device *ispvideo, u32 *pad)
{
        struct media_pad *remote;

	printk("------%s, %d\n", __func__, __LINE__);

        remote = media_entity_remote_pad(&ispvideo->pad);

        if (remote == NULL ||
            media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
                return NULL;

        if (pad)
                *pad = remote->index;

        return media_entity_to_v4l2_subdev(remote->entity);
}


/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
isp_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
        struct isp_video_device *ispvideo = video_drvdata(file);

        strlcpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
        strlcpy(cap->card, ispvideo->video.name, sizeof(cap->card));
        strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;

        return 0;
}

static int isp_video_enum_fmt_vid_cap_mplane(struct file *file, void *fh,
					    struct v4l2_fmtdesc *f)
{
        struct isp_video_device *ispvideo = video_drvdata(file);
	const struct isp_video_format *fmt = NULL;

	fmt = ispvideo->ops->find_format(NULL, NULL, f->index);
	if(fmt == NULL) {
		return -EINVAL;
	}
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}
static int
isp_video_g_fmt_vid_cap_mplane(struct file *file, void *fh, struct v4l2_format *format)
{
        struct isp_video_ctx *ctx = to_isp_video_ctx(fh);
        struct isp_video_device *ispvideo = video_drvdata(file);

	printk("------%s, %d\n", __func__, __LINE__);

        //mutex_lock(&ispvideo->mutex);
	if(ctx->format.type == 0) {
		dev_warn(ispvideo->dev, "Format not setted before calling g_fmt\n");
	} else {
        	*format = ctx->format;
	}
        //mutex_unlock(&ispvideo->mutex);

        return 0;
}

static int
__isp_video_try_format(struct isp_video_device *ispvideo, struct v4l2_pix_format_mplane *pix_mp,
		const struct isp_video_format **ofmt)
{
	struct v4l2_plane_pix_format *plane_fmt;
	struct isp_video_format *fmt = NULL;

	fmt = ispvideo->ops->find_format(&pix_mp->pixelformat, NULL, -1);
	if(fmt == NULL) {
		dev_err(ispvideo->dev, "Cannot find appropriate format for pix->pixelformat:[%x]\n", pix_mp->pixelformat);
		return -EINVAL;
	}

	pix_mp->colorspace = fmt->colorspace;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->num_planes = fmt->num_planes;

	plane_fmt = &pix_mp->plane_fmt[0];
	plane_fmt->bytesperline = pix_mp->width * fmt->depth[0] / 8;
	plane_fmt->sizeimage = plane_fmt->bytesperline * pix_mp->height;

	plane_fmt = &pix_mp->plane_fmt[1];
	plane_fmt->bytesperline = pix_mp->width * fmt->depth[1] / 8;
	plane_fmt->sizeimage = plane_fmt->bytesperline * pix_mp->height;

	if(ofmt) {
		*ofmt = fmt;
	}

	/*TODO, bound width and height!*/
	printk("=======%s,%d, pixelformat: %x, widht: %d, height: %d, num_planes: %d\n", __func__, __LINE__, pix_mp->pixelformat, pix_mp->width, pix_mp->height, pix_mp->num_planes);

	return 0;
}

static void to_v4l2_mbus_framefmt(struct isp_video_device *ispvideo, struct v4l2_format *format, struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_mp = &format->fmt.pix_mp;
	struct isp_video_format *ispfmt = NULL;

	ispfmt = ispvideo->ops->find_format(&pix_mp->pixelformat, NULL, -1);

	fmt->width = pix_mp->width;
	fmt->height = pix_mp->height;
	fmt->code = ispfmt->mbus_code;
	fmt->field = pix_mp->field;
	fmt->colorspace = pix_mp->colorspace;
	fmt->ycbcr_enc = pix_mp->ycbcr_enc;
	fmt->quantization = pix_mp->quantization;
	fmt->xfer_func = pix_mp->xfer_func;
}

static int
isp_video_s_fmt_vid_cap_mplane(struct file *file, void *fh, struct v4l2_format *format)
{
        struct isp_video_ctx *ctx = to_isp_video_ctx(fh);
        struct isp_video_device *ispvideo = video_drvdata(file);
	struct v4l2_subdev *sd = NULL;
	struct v4l2_subdev_format subdev_fmt;
	unsigned int pad = 0;
	int ret = 0;

	if(vb2_is_busy(&ctx->queue)) {
		return -EBUSY;
	}

	ret = __isp_video_try_format(ispvideo, &format->fmt.pix_mp, NULL);
	if(ret < 0) {
		dev_err(ispvideo->dev, "Failed to set format cap!\n");
		return -EINVAL;
	}

	sd = isp_video_remote_subdev(ispvideo, &pad);
	if(sd == NULL) {
		return -EINVAL;
	}

	ctx->format = *format;

	to_v4l2_mbus_framefmt(ispvideo, format, &subdev_fmt.format);

	subdev_fmt.pad = pad;
	subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &subdev_fmt);
	if(ret < 0) {
		dev_err(ispvideo->dev, "Failed to set subdev format\n");
		return -EINVAL;
	}

	//printk("-%s, %d--ctx: %x--queue: %x, queue->lock: %x\n", __func__, __LINE__, ctx, &ctx->queue, ctx->queue.lock);
	return 0;
}

static int
isp_video_try_format_cap_mplane(struct file *file, void *fh, struct v4l2_format *format)
{
        struct isp_video_device *ispvideo = video_drvdata(file);
	int ret = 0;

	printk("-----------%s, %d------------\n", __func__, __LINE__);

	ret = __isp_video_try_format(ispvideo, &format->fmt.pix_mp, NULL);
	if(ret < 0) {
		dev_err(ispvideo->dev, "Try format cap mplane error!\n");
		return -EINVAL;
	}
	return 0;
}

static int
isp_video_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	printk("-----------%s, %d------------\n", __func__, __LINE__);
	return 0;
}

static int
isp_video_get_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	printk("-----------%s, %d------------\n", __func__, __LINE__);
	return 0;
}

static int
isp_video_set_crop(struct file *file, void *fh, const struct v4l2_crop *crop)
{
	printk("-----------%s, %d------------\n", __func__, __LINE__);
	return 0;
}

static int isp_video_enum_input(struct file *file, void *priv,
				 struct v4l2_input *inp)
{
        struct isp_video_device *ispvideo = video_drvdata(file);

	if (inp->index != 0)
		return -EINVAL;

	/* default is camera */
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = ispvideo->video.tvnorms;
	strcpy(inp->name, "Camera");

	return 0;
}

static int isp_video_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int isp_video_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
        .vidioc_querycap                = isp_video_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= isp_video_enum_fmt_vid_cap_mplane,
        .vidioc_g_fmt_vid_cap_mplane    = isp_video_g_fmt_vid_cap_mplane,
        .vidioc_s_fmt_vid_cap_mplane    = isp_video_s_fmt_vid_cap_mplane,
        .vidioc_try_fmt_vid_cap_mplane  = isp_video_try_format_cap_mplane,
        .vidioc_cropcap                 = isp_video_cropcap,
        .vidioc_g_crop                  = isp_video_get_crop,
        .vidioc_s_crop                  = isp_video_set_crop,
        .vidioc_reqbufs                 = vb2_ioctl_reqbufs,
        .vidioc_querybuf                = vb2_ioctl_querybuf,
        .vidioc_qbuf                    = vb2_ioctl_qbuf,
        .vidioc_dqbuf                   = vb2_ioctl_dqbuf,
        .vidioc_streamon                = vb2_ioctl_streamon,
        .vidioc_streamoff               = vb2_ioctl_streamoff,
	.vidioc_enum_input		= isp_video_enum_input,
	.vidioc_g_input			= isp_video_g_input,
	.vidioc_s_input			= isp_video_s_input,
};

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static int isp_video_queue_setup(struct vb2_queue *queue,
                                 const void *parg,
                                 unsigned int *count, unsigned int *num_planes,
                                 unsigned int sizes[], void *alloc_ctxs[])
{
        struct isp_video_ctx *ctx = vb2_get_drv_priv(queue);
        struct isp_video_device *ispvideo = ctx->ispvideo;
	struct v4l2_format *format = &ctx->format;
	int i = 0;

	if(*num_planes) {
		for(i = 0; i < *num_planes; i++) {
			if(sizes[i] < format->fmt.pix_mp.plane_fmt[i].sizeimage) {
				return -EINVAL;
			}
		}
	} else {
		*num_planes = format->fmt.pix_mp.num_planes;
		for(i = 0; i < *num_planes; i++) {
			sizes[i] = format->fmt.pix_mp.plane_fmt[i].sizeimage;
			alloc_ctxs[i] = ispvideo->alloc_ctx;
		}
	}

	/*MAX_BUFFER_NUMS:*/
	if(*count >= 3) {
		*count = 3;
	}

        return 0;
}

static int isp_video_buffer_prepare(struct vb2_buffer *buf)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(buf->vb2_queue);
	struct v4l2_format *format = &ctx->format;
	struct v4l2_pix_format_mplane *pix_mp = &format->fmt.pix_mp;
	int i;

	for(i = 0; i < buf->num_planes; i++) {
		vb2_set_plane_payload(buf, i, pix_mp->plane_fmt[i].sizeimage);
	}

        return 0;
}

static void isp_video_buffer_queue(struct vb2_buffer *buf)
{
        struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct isp_video_ctx *ctx = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_video_buffer *isp_buffer = to_isp_buffer(vbuf);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	int ret = 0;

	ret = ispvideo->ops->qbuf(ispvideo, isp_buffer);
	if(ret < 0) {
		dev_err(ispvideo->dev, "failed to queue buf!\n");
	}
}

static int isp_video_subdev_init(struct isp_video_device *ispvideo)
{
	struct media_entity *entity = &ispvideo->video.entity;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	int ret = 0;

	/*subdevs reset.*/
	while(1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if(!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
				media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		v4l2_set_subdev_hostdata(sd, ispvideo);
		ret = v4l2_subdev_call(sd, core, init, 1);
		if(ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(ispvideo->dev, "failed to reset subdev[%s]\n", entity->name);
		} else {
			ret = 0;
		}
	}

	return ret;
}

static int isp_video_subdev_reset(struct isp_video_device *ispvideo)
{
	struct media_entity *entity = &ispvideo->video.entity;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	int ret = 0;

	/*subdevs reset.*/
	while(1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if(!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
				media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		v4l2_set_subdev_hostdata(sd, ispvideo);
		ret = v4l2_subdev_call(sd, core, reset, 1);
		if(ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(ispvideo->dev, "failed to reset subdev[%s]\n", entity->name);
		} else {
			ret = 0;
		}
	}

	return ret;

}

static int isp_video_subdev_stream(struct isp_video_device *ispvideo, int enable)
{
	struct media_entity *entity = &ispvideo->video.entity;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	int ret = 0;

	/*subdevs stream on.*/
	while(1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if(!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
				media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		v4l2_set_subdev_hostdata(sd, ispvideo);
		ret = v4l2_subdev_call(sd, video, s_stream, enable);
		if(ret < 0) {
			dev_err(ispvideo->dev, "unable to start stream[%s]\n", entity->name);
		}

	}

	return ret;
}

static int isp_video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(q);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	struct media_entity *entity = &ispvideo->video.entity;
	struct isp_pipeline *pipe = &ispvideo->pipe;
	int ret = 0;
	printk("-----*********  -%s, %d\n", __func__, __LINE__);

	mutex_lock(&ispvideo->stream_lock);
	ret = media_entity_pipeline_start(entity, &pipe->pipeline);
	if(ret < 0) {
		dev_err(ispvideo->dev, "Failed to start pipeline\n");
		goto err_pipeline_start;
	}

	ret = isp_video_subdev_reset(ispvideo);


	ret = isp_video_subdev_init(ispvideo);


	ret = isp_video_subdev_stream(ispvideo, 1);

	if(ret < 0) {
		goto err_start_stream;
	}

	mutex_unlock(&ispvideo->stream_lock);

	return 0;

err_start_stream:
	media_entity_pipeline_stop(entity);
err_pipeline_start:
	mutex_unlock(&ispvideo->stream_lock);
	return ret;
}
static void isp_video_stop_streaming(struct vb2_queue *q)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(q);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	struct media_entity *entity = &ispvideo->video.entity;
	//struct isp_pipeline *pipe = &ispvideo->pipe;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	unsigned int is_streaming = 0;
	int ret = 0;

	mutex_lock(&ispvideo->stream_lock);

	is_streaming = vb2_is_streaming(q);

	if(!is_streaming)
		goto done;

	/*subdevs stream off.*/
	while(1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if(!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
				media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		v4l2_set_subdev_hostdata(sd, ispvideo);
		ret = v4l2_subdev_call(sd, video, s_stream, 0);
		if(ret < 0) {
			dev_err(ispvideo->dev, "unable to start stream[%s]\n", entity->name);
			break;
		}

	}
	media_entity_pipeline_stop(&ispvideo->video.entity);

done:
	mutex_unlock(&ispvideo->stream_lock);
}

static const struct vb2_ops isp_video_queue_ops = {
        .queue_setup = isp_video_queue_setup,
        .buf_prepare = isp_video_buffer_prepare,
        .buf_queue = isp_video_buffer_queue,
	.start_streaming = isp_video_start_streaming,
	.stop_streaming = isp_video_stop_streaming,
};


static int isp_video_open(struct file *file)
{
        struct isp_video_device *ispvideo = video_drvdata(file);
	struct video_device *vdev = &ispvideo->video;
	struct isp_video_ctx *ctx = NULL;
	struct vb2_queue *queue = NULL;
	int ret = 0;
	printk("-----name: %s %s, %d \n", ispvideo->name, __func__, __LINE__);
        mutex_lock(&ispvideo->mutex);
	if(ispvideo->queue != NULL) {
		mutex_unlock(&ispvideo->mutex);
		return -EBUSY;
	}

	ctx = kzalloc(sizeof(struct isp_video_ctx), GFP_KERNEL);
	if(IS_ERR_OR_NULL(ctx)) {
		dev_err(ispvideo->ispcam->dev, "Failed to alloc ctx for isp_video_ctx\n");
		ret = -ENOMEM;
		goto err_ctx_alloc;
	}

	v4l2_fh_init(&ctx->fh, &ispvideo->video);
	v4l2_fh_add(&ctx->fh);

	queue = &ctx->queue;
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	queue->io_modes = VB2_MMAP;
	queue->drv_priv = ctx;
	queue->ops = &isp_video_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct isp_video_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &ispvideo->queue_lock;

	ret = vb2_queue_init(&ctx->queue);
	if(ret < 0) {
		goto err_vb2_queue_init;
	}

	ctx->ispvideo = ispvideo;
	vdev->queue = ispvideo->queue = queue;

	file->private_data = &ctx->fh;

	mutex_unlock(&ispvideo->mutex);

	return 0;
err_vb2_queue_init:
	v4l2_fh_del(&ctx->fh);
	kfree(ctx);
err_ctx_alloc:
	mutex_unlock(&ispvideo->mutex);
	return ret;
}
static int isp_video_release(struct file *file)
{
        struct isp_video_device *ispvideo = video_drvdata(file);
	struct video_device *vdev = &ispvideo->video;
	struct v4l2_fh *fh = file->private_data;
	struct isp_video_ctx *ctx = to_isp_video_ctx(fh);

	printk("-----name: %s %s, %d \n", ispvideo->name, __func__, __LINE__);
	/*lock*/

	mutex_lock(&ispvideo->mutex);

	vb2_queue_release(&ctx->queue);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	kfree(ctx);
	file->private_data = NULL;
	vdev->queue = ispvideo->queue = NULL;

	mutex_unlock(&ispvideo->mutex);

	/*unlock*/

	return 0;
}

#if 0
static unsigned int isp_video_poll(struct file *file, poll_table *wait)
{
        struct isp_video_device *ispvideo = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct isp_video_ctx *ctx = to_isp_video_ctx(fh);
	int ret = 0;
	printk("-----name: %s %s, %d \n", ispvideo->name, __func__, __LINE__);

	mutex_lock(&ispvideo->queue_lock);
	ret = vb2_poll(&ctx->queue, file, wait);
	mutex_unlock(&ispvideo->queue_lock);

	return ret;
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct isp_video_ctx *ctx = to_isp_video_ctx(fh);
	printk("-----name: %s %s, %d \n", ispvideo->name, __func__, __LINE__);

	return vb2_mmap(&ctx->queue, vma);
}
#endif

static struct v4l2_file_operations isp_video_fops = {
        .owner = THIS_MODULE,
        .unlocked_ioctl = video_ioctl2,
        .open = isp_video_open,
        .release = isp_video_release,
        .poll = vb2_fop_poll, /*isp_video_poll,*/
        .mmap = vb2_fop_mmap, /*isp_video_mmap,*/
};


int isp_video_init(struct isp_video_device *ispvideo, char *name, const struct isp_video_ops *video_ops)
{
	int ret = 0;
	struct video_device *video = &ispvideo->video;


	/*创建buffer分配器*/
	ispvideo->alloc_ctx = ingenic_vb2_dma_contig_init_ctx(ispvideo->ispcam->dev);
	if(IS_ERR(ispvideo)) {
		return -ENOMEM;
	}

	/*初始化一个sink pad*/
	ispvideo->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_init(&video->entity, 1, &ispvideo->pad, 0);
	if(ret < 0) {
		/*TODO: error case*/
	}

	mutex_init(&ispvideo->queue_lock);
	mutex_init(&ispvideo->mutex);
	mutex_init(&ispvideo->stream_lock);

	snprintf(video->name, sizeof(video->name), "isp-%s", name);
	video->fops = &isp_video_fops;
	video->ioctl_ops = &isp_video_ioctl_ops;
	video->release = video_device_release_empty;

	ispvideo->name = video->name;
	ispvideo->ops = video_ops;
	ispvideo->dev = ispvideo->ispcam->dev;

	video_set_drvdata(&ispvideo->video, ispvideo);

	return 0;

}

int isp_video_cleanup(struct isp_video_device *ispvideo)
{

	return 0;
}

int isp_video_register(struct isp_video_device *ispvideo, struct v4l2_device *v4l2_dev)
{
	struct video_device *video = &ispvideo->video;
	int ret = 0;

	video->v4l2_dev = v4l2_dev;

	ret = video_register_device(video, VFL_TYPE_GRABBER, -1);
	if(ret < 0) {
	/*TODO: error case*/
	}

	dev_info(ispvideo->dev, "register video device %s @ /dev/video%d ok\n", video->name, video->index);

	return ret;
}
int isp_video_unregister(struct isp_video_device *ispvideo)
{

	return 0;
}
