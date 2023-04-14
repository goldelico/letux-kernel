/*
*	Copyright (c) 2014 Ingenic Inc.
*	Author: qipengzhen <aric.pzqi@ingenic.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/module.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig-ingenic.h>

#include <linux/slab.h>
#include <linux/crc32.h>

#include "felix_drv.h"
#include "felix_ops.h"

#undef pr_debug
#define pr_debug pr_info

#define fh_to_ctx(__fh) container_of(__fh, struct ingenic_vdec_ctx, fh)

#define pixelformat_str(a) a >> 0, a >> 8, a >> 16, a >> 24


#define INGENIC_VDEC_MIN_W	160U
#define INGENIC_VDEC_MIN_H	120U
#define INGENIC_VDEC_MAX_W	2560U
#define INGENIC_VDEC_MAX_H	2048U


/*#define MAX_SUPPORT_FRAME_BUFFERS 16*/

#define MAX_SUPPORT_FRAME_BUFFERS 6

#define av_frame_to_vcode_buf(f)	\
		container_of(f, struct ingenic_vcodec_buf, frame) \


static unsigned int max_frame_buffers = MAX_SUPPORT_FRAME_BUFFERS;
module_param(max_frame_buffers, uint, S_IRUGO | S_IWUSR);

static struct ingenic_video_fmt ingenic_video_formats[] = {
	/* out */
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = INGENIC_FMT_DEC,
		.num_planes = 1,
		.format = FELIX_FORMAT_NONE,
	},
#ifndef CONFIG_SOC_M200
	/* cap */
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.type = INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = FELIX_NV12_MODE,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.type = INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = FELIX_NV21_MODE,
	},
#endif
	{
		.fourcc = V4L2_PIX_FMT_JZ420B,
		.type   = INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = FELIX_TILE_MODE,
	}
};

#define NUM_FORMATS		ARRAY_SIZE(ingenic_video_formats)
#define OUT_FMT_IDX		0
#define NUM_OUT_FORMATS		1
#define CAP_FMT_IDX 		(OUT_FMT_IDX + NUM_OUT_FORMATS)

static const struct ingenic_vcodec_framesizes ingenic_vcodec_framesizes[] = {
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.stepwise = { INGENIC_VDEC_MIN_W, INGENIC_VDEC_MAX_W, 2,
			      INGENIC_VDEC_MIN_H, INGENIC_VDEC_MAX_H, 2},
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.stepwise = { INGENIC_VDEC_MIN_W, INGENIC_VDEC_MAX_W, 2,
			      INGENIC_VDEC_MIN_H, INGENIC_VDEC_MAX_H, 2},
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.stepwise = { INGENIC_VDEC_MIN_W, INGENIC_VDEC_MAX_W, 2,
			      INGENIC_VDEC_MIN_H, INGENIC_VDEC_MAX_H, 2},
	},
	{
		.fourcc = V4L2_PIX_FMT_JZ420B,
		.stepwise = { INGENIC_VDEC_MIN_W, INGENIC_VDEC_MAX_W, 2,
			      INGENIC_VDEC_MIN_H, INGENIC_VDEC_MAX_H, 2},
	},
};

#define NUM_SUPPORTED_FRAMESIZE		ARRAY_SIZE(ingenic_vcodec_framesizes)

static int vidioc_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strlcpy(cap->driver, INGENIC_VCODEC_DEC_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "vpu-felix", sizeof(cap->bus_info));
	strlcpy(cap->card, "vpu-felix", sizeof(cap->card));

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING |
                    V4L2_CAP_VIDEO_CAPTURE_MPLANE |
                    V4L2_CAP_VIDEO_OUTPUT_MPLANE |
		    V4L2_CAP_DEVICE_CAPS;

	return 0;
}


static int vidioc_enum_fmt(struct v4l2_fmtdesc *f, bool output_queue)
{
	struct ingenic_video_fmt *fmt;
	int i,j = 0;


	for(i = 0; i < NUM_FORMATS; i++) {
		if(output_queue && ingenic_video_formats[i].type != INGENIC_FMT_DEC)
			continue;
		if(!output_queue && ingenic_video_formats[i].type != INGENIC_FMT_FRAME)
			continue;

		if(j == f->index) {
			fmt = &ingenic_video_formats[i];
			f->pixelformat = fmt->fourcc;
			memset(f->reserved, 0, sizeof(f->reserved));
			return 0;
		}

		j++;
	}

	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, false);
}
static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, true);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct ingenic_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int i;

	pix_fmt_mp->field = V4L2_FIELD_NONE;

	if(f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_fmt_mp->num_planes = 1;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;
	} else if(f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
		int tmp_w, tmp_h;

		pix_fmt_mp->height = clamp(pix_fmt_mp->height,
					INGENIC_VDEC_MIN_H,
					INGENIC_VDEC_MAX_H);
		pix_fmt_mp->width = clamp(pix_fmt_mp->width,
					INGENIC_VDEC_MIN_W,
					INGENIC_VDEC_MAX_W);

		tmp_w = pix_fmt_mp->width;
		tmp_h = pix_fmt_mp->height;


#if 0
		v4l_bound_align_image(&pix_fmt_mp->width,
					INGENIC_VDEC_MIN_W,
					INGENIC_VDEC_MAX_W, 7,	//128Bytes align
					&pix_fmt_mp->height,
					INGENIC_VDEC_MIN_H,
					INGENIC_VDEC_MAX_H, 0, 0);
		/*v4l2_bound_align_image will only round near, will round_down width.*/
#endif
		tmp_w = pix_fmt_mp->width;
		tmp_h = pix_fmt_mp->height;
		pr_debug("tmp_w %d tmp_h %d, w %d h %d\n",
				tmp_w, tmp_h,
				pix_fmt_mp->width,
				pix_fmt_mp->height);


		pix_fmt_mp->num_planes = fmt->num_planes;
		pix_fmt_mp->plane_fmt[0].sizeimage =
				tmp_w * tmp_h;

		pix_fmt_mp->plane_fmt[0].bytesperline = tmp_w;

		if(pix_fmt_mp->num_planes == 2) {
			pix_fmt_mp->plane_fmt[1].sizeimage =
				tmp_w * tmp_h / 2;
			pix_fmt_mp->plane_fmt[2].sizeimage = 0;

			pix_fmt_mp->plane_fmt[1].bytesperline = tmp_w;
			pix_fmt_mp->plane_fmt[2].bytesperline = 0;
		} else if(pix_fmt_mp->num_planes == 3) {
			pix_fmt_mp->plane_fmt[1].sizeimage =
			pix_fmt_mp->plane_fmt[2].sizeimage =
				(tmp_w * tmp_h) / 4;

			pix_fmt_mp->plane_fmt[1].bytesperline =
			pix_fmt_mp->plane_fmt[2].bytesperline =
				tmp_w / 2;
		}
	}

	for(i = 0; i < pix_fmt_mp->num_planes; i++) {
		memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
				sizeof(pix_fmt_mp->plane_fmt[0].reserved));
	}

//	pix_fmt_mp->flags = 0;

	memset(&pix_fmt_mp->reserved, 0x0,
			sizeof(pix_fmt_mp->reserved));

	return 0;
}

static struct ingenic_vcodec_q_data *ingenic_vcodec_get_q_data(struct ingenic_vdec_ctx *ctx,
							enum v4l2_buf_type type)
{
	if(V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[INGENIC_Q_DATA_SRC];

	return &ctx->q_data[INGENIC_Q_DATA_DST];
}

static struct ingenic_video_fmt *ingenic_vcodec_find_format(struct v4l2_format *f)
{
	struct ingenic_video_fmt *fmt;
	int i;

	for(i = 0; i < NUM_FORMATS; i++) {
		fmt = &ingenic_video_formats[i];
		if(fmt->fourcc == f->fmt.pix_mp.pixelformat)
			return fmt;
	}

	return NULL;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	unsigned int pixelformat;
	int i, ret;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		pr_err("failed to get cap vq !\n");
		return -EINVAL;
	}

	if(vb2_is_busy(vq)) {
		pr_err("cap vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);
	if(!q_data) {
		pr_err("fail to get cap q data!\n");
		return -EINVAL;
	}


	fmt = ingenic_vcodec_find_format(f);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		/* if buf type MPLANE, use pix_mp ..*/
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}


	q_data->fmt = fmt;

	ret = vidioc_try_fmt(f, q_data->fmt);
	if(ret)
		return ret;

	q_data->coded_width = ALIGN(f->fmt.pix_mp.width, 128);
	q_data->coded_height = ALIGN(f->fmt.pix_mp.height, 16);
	q_data->visible_width = f->fmt.pix_mp.width;
	q_data->visible_height = f->fmt.pix_mp.height;
	q_data->field = f->fmt.pix_mp.field;

	for(i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
		q_data->real_sizeimage[i] = q_data->sizeimage[i] / q_data->visible_height * ALIGN(q_data->visible_height, 16);
	}

	pixelformat = f->fmt.pix_mp.pixelformat;
	switch(pixelformat) {
		case V4L2_PIX_FMT_NV12:
			h264_set_output_format(ctx->h, VPU_FORMAT_NV12, q_data->coded_width, q_data->coded_height);
			break;
		case V4L2_PIX_FMT_NV21:
			h264_set_output_format(ctx->h, VPU_FORMAT_NV21, q_data->coded_width, q_data->coded_height);
			break;
		case V4L2_PIX_FMT_JZ420B:
			h264_set_output_format(ctx->h, VPU_FORMAT_TILE420, q_data->coded_width, q_data->coded_height);
			break;
		default:
			pr_err("Unsupported pix fmt: %x\n", pixelformat);
			ret = -EINVAL;
			break;
	}


	return ret;
}

static int vidioc_s_fmt_out(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int ret, i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		pr_err("fail to get out vq!\n");
		return -EINVAL;
	}
	if(vb2_is_busy(vq)) {
		pr_err("out vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);
	if(!q_data) {
		pr_err("failed to get out q_data!\n");
		return -EINVAL;
	}


	fmt = ingenic_vcodec_find_format(f);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}


	pix_fmt_mp->height = clamp(pix_fmt_mp->height,
			INGENIC_VDEC_MIN_H,
			INGENIC_VDEC_MAX_H);
	pix_fmt_mp->width = clamp(pix_fmt_mp->width,
			INGENIC_VDEC_MIN_W,
			INGENIC_VDEC_MAX_W);

	q_data->visible_width = f->fmt.pix_mp.width;
	q_data->visible_height = f->fmt.pix_mp.height;
	q_data->fmt = fmt;
	ret = vidioc_try_fmt(f, q_data->fmt);
	if (ret)
		return ret;

	q_data->coded_width = ALIGN(f->fmt.pix_mp.width, 128);
	q_data->coded_height = ALIGN(f->fmt.pix_mp.height, 16);

	q_data->field = f->fmt.pix_mp.field;
	ctx->colorspace = f->fmt.pix_mp.colorspace;
	//ctx->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	//ctx->quantization = f->fmt.pix_mp.quantization;
	//ctx->xfer_func = f->fmt.pix_mp.xfer_func;


	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
		q_data->real_sizeimage[i] = plane_fmt->sizeimage;
	}

	{ /* set capture as same as output, necessary for gstream v4l2videodec. */
		struct v4l2_format fmt;
		memset(&fmt, 0, sizeof(struct v4l2_format));
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.width = f->fmt.pix_mp.width;
		fmt.fmt.pix_mp.height = f->fmt.pix_mp.height;
		vidioc_s_fmt_cap(file, priv, &fmt);
	}



	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);

	if (q_data->fmt == NULL)
		return -EINVAL;

	pix->width = q_data->visible_width;
	pix->height = q_data->visible_height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = q_data->field;
	pix->num_planes = q_data->fmt->num_planes;
	for (i = 0; i < pix->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		memset(&(pix->plane_fmt[i].reserved[0]), 0x0,
				sizeof(pix->plane_fmt[i].reserved));
	}

//	pix->flags = 0;
	pix->colorspace = ctx->colorspace;
//	pix->ycbcr_enc = ctx->ycbcr_enc;
//	pix->quantization = ctx->quantization;
//	pix->xfer_func = ctx->xfer_func;

	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_video_fmt *fmt;
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	fmt = ingenic_vcodec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}
	f->fmt.pix_mp.colorspace = ctx->colorspace;
	//f->fmt.pix_mp.ycbcr_enc = ctx->ycbcr_enc;
	//f->fmt.pix_mp.quantization = ctx->quantization;
	//f->fmt.pix_mp.xfer_func = ctx->xfer_func;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct ingenic_video_fmt *fmt;

	fmt = ingenic_vcodec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}
	if (!f->fmt.pix_mp.colorspace) {
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	//	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	//	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	//	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbufs)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int vidioc_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);

}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	int ret = 0;

	if(ctx->state == INGENIC_STATE_ABORT) {
		return -EIO;
	}

	ret = v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);

#if 0
	{
		struct vb2_queue *vq;
		vq = v4l2_m2m_get_vq(ctx->m2m_ctx, buf->type);
		printk("---%s, %d, index: %d, type:%d [%s], state:%d\n",
				__func__, __LINE__, buf->index, buf->type, buf->type == 10 ? "OUTPUT":"CAPTUR", vq->bufs[buf->index]->state);
	}
#endif
	return ret;
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	int ret = 0;

	ret = v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
	if(buf->flags & V4L2_BUF_FLAG_LAST)
		buf->flags &= ~V4L2_BUF_FLAG_LAST;
#if 0
	{
		struct vb2_queue *vq;
		vq = v4l2_m2m_get_vq(ctx->m2m_ctx, buf->type);
		printk("---%s, %d, index: %d, type:%d [%s], state:%d\n",
				__func__, __LINE__, buf->index, buf->type, buf->type == 10 ? "OUTPUT":"CAPTUR", vq->bufs[buf->index]->state);
	}
#endif
	return ret;
}

static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{

	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, eb);
}

static int vidioc_create_bufs(struct file *file, void *priv,
			      struct v4l2_create_buffers *create)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct ingenic_vcodec_q_data *q_data = NULL;
	struct ingenic_video_fmt *fmt = NULL;

	q_data = ingenic_vcodec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	fmt = q_data->fmt;

	if(fmt->fourcc == V4L2_PIX_FMT_JZ420B && create->count < 2)
		create->count = 2;

	return v4l2_m2m_create_bufs(file, ctx->m2m_ctx, create);
}
static int vidioc_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{

	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}


static int vidioc_g_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct ingenic_vcodec_q_data *q_data;

	q_data = ingenic_vcodec_get_q_data(ctx, s->type);

	s->r.left = 0;
	s->r.top = 0;
	s->r.width = q_data->visible_width;
	s->r.height = q_data->visible_height;

	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	int i = 0;
	if(fsize->index != 0)
		return -EINVAL;

	for(i = 0; i < NUM_SUPPORTED_FRAMESIZE; i++) {
		if(fsize->pixel_format != ingenic_vcodec_framesizes[i].fourcc)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise = ingenic_vcodec_framesizes[i].stepwise;
		return 0;
	}

	return -EINVAL;
}

static int vidioc_enum_frameintervals(struct file *file, void *priv,
			    struct v4l2_frmivalenum *fe)
{
	if (fe->index)
		return -EINVAL;

	fe->type = V4L2_FRMIVAL_TYPE_STEPWISE;

	fe->stepwise.min.numerator = 1;
	fe->stepwise.min.denominator = 0;

	fe->stepwise.max.numerator = 1001;
	fe->stepwise.max.denominator = 80000;

	fe->stepwise.step.numerator = 1;
	fe->stepwise.step.denominator = 1;
	return 0;
}

static int vidioc_try_decoder_cmd (struct file *file, void *priv,
		struct v4l2_decoder_cmd *cmd)
{
	switch (cmd->cmd) {
		case V4L2_DEC_CMD_STOP:
		case V4L2_DEC_CMD_START:
			if (cmd->flags != 0) {
				pr_err("cmd->flags=%u", cmd->flags);
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
	}


	return 0;
}
static int vidioc_decoder_cmd (struct file *file, void *priv,
		struct v4l2_decoder_cmd *cmd)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *src_vq, *dst_vq;
	int ret = 0;

	ret = vidioc_try_decoder_cmd(file, priv, cmd);
	if(ret < 0) {
		return ret;
	}

	dst_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	switch(cmd->cmd) {
		case V4L2_DEC_CMD_STOP:
			src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
			if (!vb2_is_streaming(src_vq)) {
				pr_info("Output stream is off. No need to flush.");
				return 0;
			}
			if (!vb2_is_streaming(dst_vq)) {
				pr_info("Capture stream is off. No need to flush.");
				return 0;
			}


			v4l2_m2m_buf_queue(ctx->m2m_ctx, &ctx->empty_flush_buf->vb);

			v4l2_m2m_try_schedule(ctx->m2m_ctx);
			break;
		case V4L2_DEC_CMD_START:
			vb2_clear_last_buffer_dequeued(dst_vq);
			break;

		default:
			return -EINVAL;

	}

	return 0;
}



const struct v4l2_ioctl_ops ingenic_vdec_ioctl_ops = {

	/* VIDIOC_QUERYCAP handler */
	.vidioc_querycap		= vidioc_querycap,

	/* VIDIOC_ENUM_FMT handlers */
	.vidioc_enum_fmt_vid_cap 	= vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out 	= vidioc_enum_fmt_vid_out_mplane,

	/* VIDIOC_G_FMT handlers */
	.vidioc_g_fmt_vid_cap_mplane	= vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_g_fmt,

	/* VIDIOC_S_FMT handlers */
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_s_fmt_out,

	/* VIDIOC_TRY_FMT handlers */
	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,

	/* Buffer handlers */
	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,
	.vidioc_expbuf			= vidioc_expbuf,

	.vidioc_create_bufs		= vidioc_create_bufs,
	.vidioc_prepare_buf		= vidioc_prepare_buf,

	/* Stream on/off */
	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,

	/* Crop ioctls */
	.vidioc_g_selection		= vidioc_g_selection,
	.vidioc_s_selection		= vidioc_s_selection,

	.vidioc_decoder_cmd		= vidioc_decoder_cmd,
	.vidioc_try_decoder_cmd		= vidioc_try_decoder_cmd,

	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_enum_frameintervals = vidioc_enum_frameintervals,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

};


static void __maybe_unused dump_vb2_buffer(struct vb2_buffer *vb2, const char *str)
{
	int i;
	printk("======dump vb2: %s=========\n", str);
	printk("vb2->num_planes:        %d\n", vb2->num_planes);
	printk("vb2->index:    %d\n", vb2->index);
	printk("vb2->type:     %d\n", vb2->type);
	//printk("vb2->v4l2_buf.sequence: %d\n", vb2->v4l2_buf.sequence);
	//printk("vb2->v4l2_buf.length:   %d\n", vb2->v4l2_buf.length);
	for(i = 0; i < vb2->num_planes; i++) {
		printk("planes@	%d : \n", i);
		printk("\tbyteused: 	%d\n", vb2->planes[i].bytesused);
		printk("\tlength: 	%d\n", vb2->planes[i].length);

		printk("\tvaddr: 0x%p\n", vb2_plane_vaddr(vb2, i));
		printk("\tpaddr: 0x%08x\n", ingenic_vb2_dma_contig_plane_dma_addr(vb2, i));

		/* data from vaddr to vaddr + length */
		print_hex_dump(KERN_INFO, "data@ ", DUMP_PREFIX_ADDRESS, 16, 1, vb2_plane_vaddr(vb2, i), 128, true);

		printk("----------------------------------------\n");
	}
}

static void ingenic_vdec_worker(struct work_struct *work)
{
	struct ingenic_vdec_ctx *ctx = container_of(work, struct ingenic_vdec_ctx, decode_work);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct ingenic_vcodec_buf *vcodec_src_buf;
	AVCodecContext *avctx = ctx->avctx;
	H264Context *h = ctx->h;
	AVFrame *f = NULL;
	AVPacket avpkt;
	int got_frame = 0;
	int ret = 0;
	int decode_error = 0;
	int eos = 0;


	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if(src_buf == NULL) {
		pr_info("src_buf empty\n");
		v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
		return;
	}
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if(dst_buf == NULL) {
		pr_info("dst_buf empty\n");
		v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
		return;
	}

	//dump_vb2_buffer(src_buf, "src_buf");

	if(src_buf->vb2_buf.planes[0].bytesused > src_buf->vb2_buf.planes[0].length) {
		printk("**** Error, src_buf out of range [%d > %d]\n", src_buf->vb2_buf.planes[0].bytesused, src_buf->vb2_buf.planes[0].length);
	}


	vcodec_src_buf = container_of(src_buf, struct ingenic_vcodec_buf, vb);

	/*如果当前处理的是empty buf, 不用处理解码，直接从队列中删除src_buf.*/
	if((vcodec_src_buf->is_last_frame == true) ||
		(src_buf->vb2_buf.planes[0].bytesused == 0)) {

		/*Got Empty stream, End of stream.*/
		eos = 1;
		v4l2_m2m_src_buf_remove(ctx->m2m_ctx);

		/*将vpu残留的中间buffer全部还给内核.*/
		goto done;
	}

	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);


	avpkt.size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);
	avpkt.data_pa = ingenic_vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	if (src_buf->vb2_buf.memory == VB2_MEMORY_USERPTR)
		avpkt.data = phys_to_virt(avpkt.data_pa);
	else
		avpkt.data = vb2_plane_vaddr(&src_buf->vb2_buf, 0);

	if(dst_buf) {
		struct ingenic_vcodec_buf *buf = container_of(dst_buf,
				struct ingenic_vcodec_buf, vb);

		dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
		dst_buf->timecode = src_buf->timecode;
		dst_buf->sequence = src_buf->sequence;

		h264_enqueue_frame(h, &buf->frame);
	}

	ret = h264_decode_frame(avctx, NULL, &got_frame, &avpkt);
	if(ret < 0) {
		decode_error = 1;
	}

	if(got_frame) {
		/*drain display frame*/
		f = h264_dequeue_frame(h);
		if(f) {
			struct ingenic_vcodec_buf *done_buf = av_frame_to_vcode_buf(f);
			//printk("------done_buf: %x done frame f: %x-----buf->index:%d, f->index: %d\n", done_buf, f, done_buf->vb.v4l2_buf.index, f->index);


			vb2_set_plane_payload(&done_buf->vb.vb2_buf, 0, f->buf[0]->size);
			vb2_set_plane_payload(&done_buf->vb.vb2_buf, 1, f->buf[1]->size);
			//dump_vb2_buffer(&done_buf->vb, "dst_buf");
			v4l2_m2m_buf_done(&done_buf->vb, VB2_BUF_STATE_DONE);
		}
	}

	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);

done:
	if((decode_error) || (eos)) {

		if(eos) {
			while((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
				dst_buf->vb2_buf.planes[0].bytesused = 0;
				dst_buf->flags |= V4L2_BUF_FLAG_DONE;
				dst_buf->flags |= V4L2_BUF_FLAG_LAST;
				dst_buf->vb2_buf.timestamp = ktime_get();
				/*保证两个的时间戳不一样.*/
				dst_buf->vb2_buf.timestamp = ktime_add_ns(dst_buf->vb2_buf.timestamp, 10);
				/* work around, return buffer to userspace with 0 byteused. */
				v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
			}
		}

		/* retrain all enqueued frame, do not stop decode.*/
		while((f = h264_get_queued_frame(h))) {
			struct ingenic_vcodec_buf *err_buf = av_frame_to_vcode_buf(f);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 0, 0);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 1, 0);
			v4l2_m2m_buf_done(&err_buf->vb, VB2_BUF_STATE_ERROR);
		}

		while((f = h264_dequeue_frame(h))) {
			struct ingenic_vcodec_buf *err_buf = av_frame_to_vcode_buf(f);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 0, 0);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 1, 0);
			v4l2_m2m_buf_done(&err_buf->vb, VB2_BUF_STATE_ERROR);
		}
	}

	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
}

static void m2mops_vdec_device_run(void *priv)
{
	struct ingenic_vdec_ctx *ctx = priv;
	struct ingenic_vdec_dev *dev = ctx->dev;

	queue_work(dev->dec_workqueue, &ctx->decode_work);
}

static int m2mops_vdec_job_ready(void *m2m_priv)
{
	struct ingenic_vdec_ctx *ctx = m2m_priv;

	if(ctx->state == INGENIC_STATE_ABORT || ctx->state == INGENIC_STATE_IDLE) {
		pr_info("job not ready, ctx->state %d\n", ctx->state);
		return 0;
	}

	return 1;
}


static void m2mops_vdec_job_abort(void *priv)
{
	struct ingenic_vdec_ctx *ctx = priv;
	ctx->state = INGENIC_STATE_ABORT;
}

const struct v4l2_m2m_ops ingenic_vdec_m2m_ops = {
	.device_run		= m2mops_vdec_device_run,
	.job_ready		= m2mops_vdec_job_ready,
	.job_abort		= m2mops_vdec_job_abort,
};

static int vb2ops_vcodec_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct ingenic_vcodec_q_data *q_data;
	int i;

	q_data = ingenic_vcodec_get_q_data(ctx, vq->type);
	if(q_data == NULL)
		return -EINVAL;

	if(*nplanes) {
		for(i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->real_sizeimage[i];
			if(sizes[i] < q_data->real_sizeimage[i]) {
				return -EINVAL;
			}
		}
	} else {
		*nplanes = q_data->fmt->num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->real_sizeimage[i];
			alloc_ctxs[i] = ctx->dev->dev;
		}
	}

	if(*nbuffers > max_frame_buffers)
		*nbuffers = max_frame_buffers;

	return 0;
}

static int vb2ops_vcodec_buf_prepare(struct vb2_buffer *vb)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ingenic_vcodec_q_data *q_data;
	int i;

	q_data = ingenic_vcodec_get_q_data(ctx, vb->vb2_queue->type);

	for(i = 0; i < q_data->fmt->num_planes; i++) {
		if(vb2_plane_size(vb, i) > q_data->real_sizeimage[i]) {

			pr_err("Failed to prepare buf!\n");

			return -EINVAL;
		}
	}

	return 0;
}


static int vb2ops_vcodec_buf_init(struct vb2_buffer *vb)
{
	struct ingenic_vcodec_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
                                struct ingenic_vcodec_buf, vb);

	int i;
	AVFrame *f = &buf->frame;

	f->index = vb->index;
	for(i = 0; i < vb->num_planes; i++) {
		f->buf[i] = av_buffer_create(vb2_plane_vaddr(vb, i),
					ingenic_vb2_dma_contig_plane_dma_addr(vb, i),
					vb2_plane_size(vb, i)
					);
	}
	return 0;
}
static void vb2ops_vcodec_buf_cleanup(struct vb2_buffer *vb)
{
	struct ingenic_vcodec_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
                                struct ingenic_vcodec_buf, vb);
	int i;
	AVFrame *f = &buf->frame;

	for(i = 0; i < vb->num_planes; i++) {
		av_buffer_del(f->buf[i]);
	}
}

static void vb2ops_vcodec_buf_queue(struct vb2_buffer *vb)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));
	return;
}

static void vb2ops_vcodec_buf_finish(struct vb2_buffer *vb)
{
	return;
}


static int vb2ops_vcodec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(q);
	struct ingenic_vcodec_q_data *q_data_src;
	struct ingenic_vcodec_q_data *q_data_dst;

	if(V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->output_stopped = 0;
	} else {
		ctx->capture_stopped = 0;
	}

	if(ctx->output_stopped  || ctx->capture_stopped)
		return 0;

	q_data_src = ingenic_vcodec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = ingenic_vcodec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	ctx->state = INGENIC_STATE_RUNNING;
	return 0;
}

static void vb2ops_vcodec_stop_streaming(struct vb2_queue *q)
{

	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	H264Context *h = ctx->h;
	AVFrame *f = NULL;

	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {

		while((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
			dst_buf->vb2_buf.planes[0].bytesused = 0;
			dst_buf->flags |= V4L2_BUF_FLAG_DONE;
			dst_buf->vb2_buf.timestamp = ktime_get();
			/*保证两个的时间戳不一样.*/
			dst_buf->vb2_buf.timestamp = ktime_add_ns(dst_buf->vb2_buf.timestamp, 10);
			/* work around, return buffer to userspace with 0 byteused. */
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		}

		while((f = h264_get_queued_frame(h))) {
			struct ingenic_vcodec_buf *err_buf = av_frame_to_vcode_buf(f);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 0, 0);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 1, 0);
			v4l2_m2m_buf_done(&err_buf->vb, VB2_BUF_STATE_DONE);
		}

		while((f = h264_dequeue_frame(h))) {
			struct ingenic_vcodec_buf *err_buf = av_frame_to_vcode_buf(f);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 0, 0);
			vb2_set_plane_payload(&err_buf->vb.vb2_buf, 1, 0);
			v4l2_m2m_buf_done(&err_buf->vb, VB2_BUF_STATE_DONE);
		}

	} else {
		while((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx))) {
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		}
	}

	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ctx->capture_stopped = 1;
	else
		ctx->output_stopped = 1;

	if((ctx->capture_stopped && ctx->output_stopped) == 0) {
		return;
	}

	ctx->state = INGENIC_STATE_IDLE;

	return;
}

static const struct vb2_ops ingenic_vcodec_vb2_ops = {
	.queue_setup		= vb2ops_vcodec_queue_setup,
	.buf_prepare		= vb2ops_vcodec_buf_prepare,
	.buf_init		= vb2ops_vcodec_buf_init,
	.buf_cleanup		= vb2ops_vcodec_buf_cleanup,
	.buf_queue		= vb2ops_vcodec_buf_queue,
	.buf_finish		= vb2ops_vcodec_buf_finish,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vb2ops_vcodec_start_streaming,
	.stop_streaming		= vb2ops_vcodec_stop_streaming,
};

int ingenic_vcodec_vdec_queue_init(void *priv, struct vb2_queue *src_vq,
	struct vb2_queue *dst_vq)
{
	struct ingenic_vdec_ctx *ctx = priv;
	int ret = 0;

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size	= sizeof(struct ingenic_vcodec_buf);
	src_vq->ops		= &ingenic_vcodec_vb2_ops;
	src_vq->mem_ops		= &ingenic_vb2_dma_contig_memops;
	src_vq->lock		= &ctx->dev->dev_mutex;
	src_vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->dev		= ctx->dev->dev;

	ret = vb2_queue_init(src_vq);
	if(ret)
		return ret;


	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size	= sizeof(struct ingenic_vcodec_buf);
	dst_vq->ops		= &ingenic_vcodec_vb2_ops;
	dst_vq->mem_ops		= &ingenic_vb2_dma_contig_memops;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	dst_vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->dev		= ctx->dev->dev;

	ret = vb2_queue_init(dst_vq);
	if(ret) {
		vb2_queue_release(src_vq);
	}

	return ret;
}


int ingenic_vcodec_init_default_params(struct ingenic_vdec_ctx *ctx)
{
	struct vb2_queue *src_vq;
	int ret = 0;
	ctx->capture_stopped = 1;
	ctx->output_stopped = 1;
	ctx->state = INGENIC_STATE_IDLE;
	INIT_WORK(&ctx->decode_work, ingenic_vdec_worker);

	src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	ctx->empty_flush_buf = kzalloc(sizeof(struct ingenic_vcodec_buf), GFP_KERNEL);
	if(!ctx->empty_flush_buf) {
		pr_err("failed to alloc empty_flush_buf\n");
		return -ENOMEM;
	}

	ctx->empty_flush_buf->vb.vb2_buf.vb2_queue = src_vq;
	ctx->empty_flush_buf->is_last_frame = true;


	return ret;
}

int ingenic_vcodec_deinit_default_params(struct ingenic_vdec_ctx *ctx)
{

	kfree(ctx->empty_flush_buf);

	return 0;
}
