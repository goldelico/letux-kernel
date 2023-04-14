/*
* Copyright© 2014 Ingenic Semiconductor Co.,Ltd
*
* Author: qipengzhen <aric.pzqi@ingenic.com>
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

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig-ingenic.h>

#include <linux/slab.h>
#include <linux/crc32.h>

#include "helix_drv.h"
#include "helix_ops.h"

#include "api/helix.h"

#include "h264e_rc.h"
#include "jpge.h"
#include "jpgd.h"


#define fh_to_ctx(__fh) container_of(__fh, struct ingenic_venc_ctx, fh)
#define NUM_SUPPORTED_FRAMESIZE		3

#define INGENIC_VENC_MIN_W	160U
#define INGENIC_VENC_MIN_H	120U
#define INGENIC_VENC_MAX_W	2560U
#define INGENIC_VENC_MAX_H	2048U

#define MAX_SUPPORT_FRAME_BUFFERS 3



#define pixelformat_str(a) a >> 0, a >> 8, a >> 16, a >> 24


static struct ingenic_video_fmt ingenic_video_formats[] = {
	/* output */
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.type	= INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = HELIX_NV12_MODE,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.type = INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = HELIX_NV21_MODE,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.type = INGENIC_FMT_FRAME,
		.num_planes = 3,
		.format = HELIX_420P_MODE,
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.type = INGENIC_FMT_DEC,
		.num_planes = 1,
	},
	/*capture*/
	/* video encoder idx */
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = INGENIC_FMT_ENC,
		.num_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.type = INGENIC_FMT_ENC,
		.num_planes = 1,
	},
	{	/* jpeg decoder */
		.fourcc = V4L2_PIX_FMT_NV12,
		.type	= INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = HELIX_NV12_MODE,
	},

};

#define NUM_FORMATS	ARRAY_SIZE(ingenic_video_formats)
#define OUT_FMT_IDX	0
#define CAP_FMT_IDX 	(ARRAY_SIZE(ingenic_video_formats) - 4)

static const struct ingenic_codec_framesizes ingenic_venc_framesizes[] = {
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.stepwise = { INGENIC_VENC_MIN_W, INGENIC_VENC_MAX_W, 16,
			      INGENIC_VENC_MIN_H, INGENIC_VENC_MAX_H, 16},
	},
};

static struct ingenic_venc_q_data *ingenic_venc_get_q_data(struct ingenic_venc_ctx *ctx,
							enum v4l2_buf_type type);


static int vidioc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ingenic_venc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct h264e_params *h264_p = &h264e_ctx->p;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpge_params *jpge_p = &jpge_ctx->p;
	int ret = 0;

	switch(ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_BITRATE val = %d\n", ctrl->val);
		h264_p->bitrate = ctrl->val;
		break;
#if 0
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_B_FRAMES val = %d\n", ctrl->val);
		break;
#endif
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP val = %d\n", ctrl->val);
		h264_p->i_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP val = %d\n", ctrl->val);
		h264_p->p_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE val = %d\n", ctrl->val);
		h264_p->frame_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_MIN_QP val = %d\n", ctrl->val);
		h264_p->h264_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_MAX_QP val = %d\n", ctrl->val);
		h264_p->h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_HEADER_MODE val = %d\n", ctrl->val);
		h264_p->h264_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE val = %d\n", ctrl->val);
		h264_p->mb_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_PROFILE val = %d\n", ctrl->val);
		h264_p->h264_profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_LEVEL val = %d\n", ctrl->val);
		h264_p->h264_level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_BITRATE_MODE val = %d\n", ctrl->val);
		h264_p->rc_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_H264_I_PERIOD val = %d\n", ctrl->val);
		break;

	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		dev_dbg(ctx->dev->dev, "V4L2_CID_MPEG_VIDEO_GOP_SIZE val = %d\n", ctrl->val);
		h264_p->gop_size = ctrl->val;
		break;

	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		dev_info(ctx->dev->dev, "V4L2_CID_JPEG_COMPRESSION_QUALITY = %d\n", ctrl->val);
		jpge_p->compr_quality = ctrl->val;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;

}

static const struct v4l2_ctrl_ops ingenic_venc_ctrl_ops = {
	.s_ctrl = vidioc_s_ctrl,
};

static int vidioc_enum_fmt(struct v4l2_fmtdesc *f, bool output_queue)
{
	struct ingenic_video_fmt *fmt;
	int i,j = 0;

	int start_index = output_queue ? OUT_FMT_IDX : CAP_FMT_IDX;

	for(i = start_index; i < NUM_FORMATS; i++) {

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


static int vidioc_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	int i = 0;
	if(fsize->index != 0)
		return -EINVAL;

	for(i = 0; i < NUM_SUPPORTED_FRAMESIZE; i++) {
		if(fsize->pixel_format != ingenic_venc_framesizes[i].fourcc)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise = ingenic_venc_framesizes[i].stepwise;
		return 0;
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

static int vidioc_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strlcpy(cap->driver, INGENIC_VCODEC_ENC_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "vpu-helix", sizeof(cap->bus_info));
	strlcpy(cap->card, "vpu-helix", sizeof(cap->card));

	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING |
                    V4L2_CAP_VIDEO_CAPTURE_MPLANE |
                    V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}


static int vidioc_s_parm(struct file *file, void *priv,
			struct v4l2_streamparm *a)
{
	if(a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	return 0;
}

static ssize_t
h264e_encoded_headers(struct ingenic_venc_ctx *ctx)
{
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct ingenic_venc_q_data *q_data_src;
	struct ingenic_venc_q_data *q_data_dst;
	int ret = 0;

	q_data_src = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	ret = h264e_set_fmt(h264e_ctx, q_data_src->coded_width,
			q_data_src->coded_height, q_data_src->fmt->format);
	if(ret < 0) {
		return ret;
	}

	ret = h264e_generate_headers(h264e_ctx, 0);
	if(ret < 0)
		return ret;

	return h264e_ctx->bs_header_size;
}

struct h264_header {
	unsigned int size;
	unsigned char header;
};

static int vidioc_g_parm(struct file *file, void *priv,
			struct v4l2_streamparm *a)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_venc_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct h264_header *h = (struct h264_header *)a->parm.raw_data;
	unsigned int *size = 0;

	if(ctx->codec_id == CODEC_ID_H264E) {
		h264e_encoded_headers(ctx);

		h->size = h264e_ctx->bs_header_size;
		memcpy(&h->header, h264e_ctx->bs_header, h264e_ctx->bs_header_size);
	}
	return 0;
}

static struct ingenic_venc_q_data *ingenic_venc_get_q_data(struct ingenic_venc_ctx *ctx,
							enum v4l2_buf_type type)
{
	if(V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[INGENIC_Q_DATA_SRC];

	return &ctx->q_data[INGENIC_Q_DATA_DST];
}

static struct ingenic_video_fmt *ingenic_venc_find_format(struct v4l2_format *f, int isout)
{
	struct ingenic_video_fmt *fmt;
	int i;
	int start_index = isout ? OUT_FMT_IDX : CAP_FMT_IDX;

	for(i = start_index; i < NUM_FORMATS; i++) {
		fmt = &ingenic_video_formats[i];
		if(fmt->fourcc == f->fmt.pix_mp.pixelformat)
			return fmt;
	}

	return NULL;
}

static int vidioc_try_fmt(struct v4l2_format *f, struct ingenic_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int i;

	pix_fmt_mp->field = V4L2_FIELD_NONE;

	if(fmt->type == INGENIC_FMT_FRAME) {
		int tmp_w, tmp_h;

		pix_fmt_mp->height = clamp(pix_fmt_mp->height,
				INGENIC_VENC_MIN_H,
				INGENIC_VENC_MAX_H);
		pix_fmt_mp->width = clamp(pix_fmt_mp->width,
				INGENIC_VENC_MIN_W,
				INGENIC_VENC_MAX_W);

#if 0
		tmp_w = ALIGN(pix_fmt_mp->width, 16);
		tmp_h = ALIGN(pix_fmt_mp->height, 16);
		pr_info("tmp_w %d tmp_h %d, w %d h %d\n",
				tmp_w, tmp_h,
				pix_fmt_mp->width,
				pix_fmt_mp->height);
#else
		tmp_w = pix_fmt_mp->width;
		tmp_h = pix_fmt_mp->height;
#endif
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

	} else {
		pix_fmt_mp->num_planes = 1;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;

		/*restrict output bs buffer size, max width * height.
		  sizeimage can be set by userspace.
		*/
		if(pix_fmt_mp->plane_fmt[0].sizeimage) {
			pix_fmt_mp->plane_fmt[0].sizeimage = min(pix_fmt_mp->plane_fmt[0].sizeimage , pix_fmt_mp->width * pix_fmt_mp->height * 3 / 4);
		} else {
			pix_fmt_mp->plane_fmt[0].sizeimage = pix_fmt_mp->width * pix_fmt_mp->height * 3 / 4;
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

static int vidioc_s_fmt_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_venc_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	int i, ret;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		dev_err(ctx->dev->dev, "failed to get cap vq !\n");
		return -EINVAL;
	}

	if(vb2_is_busy(vq)) {
		dev_err(ctx->dev->dev, "cap vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_venc_get_q_data(ctx, f->type);
	if(!q_data) {
		dev_err(ctx->dev->dev, "fail to get cap q data!\n");
		return -EINVAL;
	}


	fmt = ingenic_venc_find_format(f, 0);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		/* if buf type MPLANE, use pix_mp ..*/
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_venc_find_format(f, 0);
	}


	q_data->fmt = fmt;

	ret = vidioc_try_fmt(f, q_data->fmt);
	if(ret)
		return ret;

	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;
	q_data->field = f->fmt.pix_mp.field;

	for(i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
	}

	/* init hw interface. */
	if(f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_H264) {
		ctx->codec_id = CODEC_ID_H264E;

		h264e_set_priv(h264e_ctx, ctx);
		ret = h264e_encoder_init(h264e_ctx);

	} else if(f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_JPEG) {
		ctx->codec_id = CODEC_ID_JPGE;

		jpeg_encoder_set_priv(jpge_ctx, ctx);
		ret = jpeg_encoder_init(jpge_ctx);

	}

	return ret;
}


static int vidioc_s_fmt_out(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_venc_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;
	int ret, i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		dev_err(ctx->dev->dev, "fail to get out vq!\n");
		return -EINVAL;
	}
	if(vb2_is_busy(vq)) {
		dev_err(ctx->dev->dev, "out vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_venc_get_q_data(ctx, f->type);
	if(!q_data) {
		dev_err(ctx->dev->dev, "failed to get out q_data!\n");
		return -EINVAL;
	}


	dev_dbg(ctx->dev->dev, "s_fmt_out f->fmt %x, %c%c%c%c \n", f->fmt.pix_mp.pixelformat, pixelformat_str(f->fmt.pix_mp.pixelformat));
	fmt = ingenic_venc_find_format(f, 1);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_venc_find_format(f, 1);
	}


	pix_fmt_mp->height = clamp(pix_fmt_mp->height,
			INGENIC_VENC_MIN_H,
			INGENIC_VENC_MAX_H);
	pix_fmt_mp->width = clamp(pix_fmt_mp->width,
			INGENIC_VENC_MIN_W,
			INGENIC_VENC_MAX_W);

	q_data->visible_width = f->fmt.pix_mp.width;
	q_data->visible_height = f->fmt.pix_mp.height;
	q_data->fmt = fmt;
	ret = vidioc_try_fmt(f, q_data->fmt);
	if (ret)
		return ret;

	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;

	dev_dbg(ctx->dev->dev, "s_fmt_out q_data->coded_width: %d, q_data->coded_height: %d\n",
			q_data->coded_width, q_data->coded_height);

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
	}

	if(f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_JPEG) {
		ctx->codec_id = CODEC_ID_JPGD;

		jpeg_decoder_set_priv(jpgd_ctx, ctx);
		jpeg_decoder_init(jpgd_ctx);
	}


	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_venc_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = ingenic_venc_get_q_data(ctx, f->type);

	pix->width = q_data->coded_width;
	pix->height = q_data->coded_height;
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


	dev_dbg(ctx->dev->dev, "g_fmt f->type %d\n", f->type);
	dev_dbg(ctx->dev->dev, "pixel_format = %x, %c%c%c%c\n", pix->pixelformat, pixelformat_str(pix->pixelformat));
	dev_dbg(ctx->dev->dev, "pix->num_planes %d\n", pix->num_planes);

	return 0;
}


static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_video_fmt *fmt;
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	fmt = ingenic_venc_find_format(f, 0);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_venc_find_format(f, 0);
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

	fmt = ingenic_venc_find_format(f, 1);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_venc_find_format(f, 1);
	}
	if (!f->fmt.pix_mp.colorspace) {
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	//	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	//	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	//	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_g_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	// TODO

	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	//TODO
	return 0;
}


static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	if(ctx->state == INGENIC_STATE_ABORT) {
		return -EIO;
	}

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);
	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbufs)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);

}

static int vidioc_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);

}

static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{

	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, eb);
}

static int vidioc_create_bufs(struct file *file, void *priv,
			      struct v4l2_create_buffers *create)
{
	struct ingenic_venc_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_create_bufs(file, ctx->m2m_ctx, create);
}
static int vidioc_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	return 0;
}

const struct v4l2_ioctl_ops ingenic_venc_ioctl_ops = {

	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,

	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,

	.vidioc_querycap		= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap 	= vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out 	= vidioc_enum_fmt_vid_out_mplane,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,
	.vidioc_expbuf			= vidioc_expbuf,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

	.vidioc_s_parm			= vidioc_s_parm,
	.vidioc_g_parm			= vidioc_g_parm,
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_s_fmt_out,

	.vidioc_g_fmt_vid_cap_mplane	= vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_g_fmt,

	.vidioc_create_bufs		= vidioc_create_bufs,
	.vidioc_prepare_buf		= vidioc_prepare_buf,

	.vidioc_g_selection		= vidioc_g_selection,
	.vidioc_s_selection		= vidioc_s_selection,
};


static int vb2ops_venc_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct ingenic_venc_ctx *ctx = vb2_get_drv_priv(vq);
	struct ingenic_venc_q_data *q_data;
	int i;

	q_data = ingenic_venc_get_q_data(ctx, vq->type);
	if(q_data == NULL)
		return -EINVAL;

	if(*nplanes) {
		for(i = 0; i < *nplanes; i++) {
			if(vq->memory != VB2_MEMORY_USERPTR){
				q_data->sizeimage[i] = q_data->sizeimage[i]/q_data->coded_height * ALIGN(q_data->coded_height, 16);
				sizes[i]= sizes[i] / q_data->coded_height * ALIGN(q_data->coded_height, 16);
			}
			if(sizes[i] < q_data->sizeimage[i]) {
				return -EINVAL;
			}
		}
	} else {
		*nplanes = q_data->fmt->num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->sizeimage[i];
			alloc_ctxs[i] = ctx->dev->dev;
		}
	}


	dev_dbg(ctx->dev->dev, "*nplanes %d, sizes[0] %d\n", *nplanes, sizes[0]);
	for(i = 0; i<*nplanes ; i++) {
		dev_dbg(ctx->dev->dev, "plane %d, sizes[%d] %d, type  %d\n", i, i, sizes[i], vq->type);
	}

	if(*nbuffers > MAX_SUPPORT_FRAME_BUFFERS)
		*nbuffers = MAX_SUPPORT_FRAME_BUFFERS;

	return 0;
}

static int vb2ops_venc_buf_prepare(struct vb2_buffer *vb)
{
	struct ingenic_venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ingenic_venc_q_data *q_data;
	int i;

	q_data = ingenic_venc_get_q_data(ctx, vb->vb2_queue->type);

	for(i = 0; i < q_data->fmt->num_planes; i++) {
		if(vb2_plane_size(vb, i) > q_data->sizeimage[i]) {

			dev_err(ctx->dev->dev, "Failed to prepare buf!\n");

			return -EINVAL;
		}
	}

	return 0;
}

static void vb2ops_venc_buf_queue(struct vb2_buffer *vb)
{
	struct ingenic_venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));
	return;
}


static int vb2ops_venc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ingenic_venc_ctx *ctx = vb2_get_drv_priv(q);
	struct ingenic_venc_q_data *q_data_src;
	struct ingenic_venc_q_data *q_data_dst;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;
	int ret = 0;

	if(V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->output_stopped = 0;
	} else {
		ctx->capture_stopped = 0;
	}

	if(ctx->output_stopped || ctx->capture_stopped)
		return 0;

	q_data_src = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	if(ctx->codec_id == CODEC_ID_H264E) {
		dev_info(ctx->dev->dev, "h264 start streaming \n");
		ret = h264e_set_fmt(h264e_ctx, q_data_src->coded_width,
			q_data_src->coded_height, q_data_src->fmt->format);
		if(ret < 0) {
			return ret;
		}
		ret = h264e_alloc_workbuf(h264e_ctx);
		if(ret < 0) {
			dev_err(ctx->dev->dev, "h264e failed to alloc work buffer!\n");
			return ret;
		}
		ctx->state = INGENIC_STATE_HEADER;

		ret = h264e_generate_headers(h264e_ctx, 1);
		if(ret < 0)
			return ret;

	} else if(ctx->codec_id == CODEC_ID_JPGE) {
		dev_info(ctx->dev->dev, "jpge start streaming \n");

		ret = jpeg_encoder_set_fmt(jpge_ctx, q_data_src->coded_width,
			q_data_src->coded_height, q_data_src->fmt->format);
		if(ret < 0) {
			return ret;
		}
		ret = jpeg_encoder_alloc_workbuf(jpge_ctx);
		if(ret < 0) {
			dev_err(ctx->dev->dev, "jpge failed to alloc work buffer!\n");
			return ret;
		}
		ctx->state = INGENIC_STATE_RUNNING;

	} else if(ctx->codec_id == CODEC_ID_JPGD) {
		dev_info(ctx->dev->dev, "jpgd start streaming \n");

		ret = jpeg_decoder_set_fmt(jpgd_ctx, q_data_dst->coded_width,
			q_data_dst->coded_height, q_data_dst->fmt->format);
		if(ret < 0) {
			return ret;
		}
		ret = jpeg_decoder_alloc_workbuf(jpgd_ctx);
		if(ret < 0) {
			dev_err(ctx->dev->dev, "jpgd failed to alloc work buffer!\n");
			return ret;
		}
		ctx->state = INGENIC_STATE_RUNNING;

	} else {
		dev_info(ctx->dev->dev, "Invalid Codec ID: %d\n", ctx->codec_id);
		return -EINVAL;
	}

	return 0;
}

static void vb2ops_venc_stop_streaming(struct vb2_queue *q)
{

	struct ingenic_venc_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct ingenic_venc_q_data *q_data_src;
	struct ingenic_venc_q_data *q_data_dst;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;

//	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {

	/* KERNEL 3.10 .*/
		while((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
			dst_buf->vb2_buf.planes[0].bytesused = 0;
			dst_buf->flags |= V4L2_BUF_FLAG_DONE;
//			ktime_get_real_ts64(&dst_buf->vb2_buf.timestamp);
			dst_buf->vb2_buf.timestamp = ktime_get();
			/*保证两个的时间戳不一样.*/
			dst_buf->vb2_buf.timestamp = ktime_add_ns(dst_buf->vb2_buf.timestamp, 10);
			/* work around, return buffer to userspace with 0 byteused. */
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		}
//	} else {
		while((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx))) {
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		}
//	}

	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ctx->capture_stopped = 1;
	else
		ctx->output_stopped = 1;

	if((ctx->capture_stopped && ctx->output_stopped) == 0) {
		return;
	}

	q_data_src = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = ingenic_venc_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	switch(ctx->codec_id) {
		case CODEC_ID_H264E:
			dev_info(ctx->dev->dev, "h264 stop streaming !\n");
			h264e_free_workbuf(h264e_ctx);
			break;
		case CODEC_ID_JPGE:
			dev_info(ctx->dev->dev, "jpge stop streaming !\n");
			jpeg_encoder_free_workbuf(jpge_ctx);
			break;
		case CODEC_ID_JPGD:
			dev_info(ctx->dev->dev, "jpgd stop streaming !\n");
			jpeg_decoder_free_workbuf(jpgd_ctx);
			break;
		default:
			break;
	}

	ctx->state = INGENIC_STATE_IDLE;
	return;
}

static const struct vb2_ops ingenic_venc_vb2_ops = {
	.queue_setup		= vb2ops_venc_queue_setup,
	.buf_prepare		= vb2ops_venc_buf_prepare,
	.buf_queue		= vb2ops_venc_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vb2ops_venc_start_streaming,
	.stop_streaming		= vb2ops_venc_stop_streaming,
};

static void ingenic_venc_worker(struct work_struct *work)
{

	struct ingenic_venc_ctx *ctx = container_of(work, struct ingenic_venc_ctx, encode_work);
	struct ingenic_venc_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct ingenic_video_buf *src_video_buf, *dst_video_buf;

	struct video_frame_buffer *src_frame = NULL;
	struct video_frame_buffer *dst_frame = NULL;

	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;
	int force_idr = 0;
	int keyframe = 0;

	int i = 0;
	int ret = 0;


	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	src_video_buf = container_of(src_buf, struct ingenic_video_buf, vb);
	src_frame = &src_video_buf->buf;

	memset(src_frame, 0, sizeof(src_frame));
	src_frame->num_planes = src_buf->vb2_buf.num_planes;
	for(i = 0; i < src_buf->vb2_buf.num_planes; i++) {
		src_frame->fb_addr[i].pa = ingenic_vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, i);
		if (src_buf->vb2_buf.memory == VB2_MEMORY_USERPTR)
			src_frame->fb_addr[i].va = phys_to_virt(src_frame->fb_addr[i].pa);
		else
			src_frame->fb_addr[i].va = vb2_plane_vaddr(&src_buf->vb2_buf, i);

		src_frame->fb_addr[i].size = vb2_get_plane_payload(&src_buf->vb2_buf, i);

#if 0
		printk("---va %x, length %d, bytesused %d\n", vb2_plane_vaddr(src_buf, i),
				(size_t)src_buf->vb2_buf.planes[i].length,
				src_buf->vb2_buf.planes[i].bytesused);

		print_hex_dump(KERN_INFO, "input", DUMP_PREFIX_ADDRESS, 16, 1, src_frame->fb_addr[i].va, 64, 1);
#endif
	}

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	dst_buf->vb2_buf.planes[0].bytesused = 0;
	dst_video_buf = container_of(dst_buf, struct ingenic_video_buf, vb);
	dst_frame = &dst_video_buf->buf;
	dst_frame->num_planes = dst_buf->vb2_buf.num_planes;

	if(ctx->state == INGENIC_STATE_HEADER) {

		if(ctx->codec_id != CODEC_ID_H264E) {
			dev_err(ctx->dev->dev, "invalid state!\n");
			ctx->state = INGENIC_STATE_ABORT;
			return;
		}
		dst_frame->fb_addr[0].pa = ingenic_vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
		if (dst_buf->vb2_buf.memory == VB2_MEMORY_USERPTR)
			dst_frame->fb_addr[0].va = phys_to_virt(dst_frame->fb_addr[0].pa);
		else
			dst_frame->fb_addr[0].va = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

		h264e_ctx->encoded_bs_len = 0;
		ret = h264e_encode_headers(h264e_ctx, &dst_frame->fb_addr[0]);
		if(ret < 0) {
			ctx->state = INGENIC_STATE_ABORT;
			return;
		}
#if 0 //TODO:
		if(h264e_ctx->p.h264_hdr_mode == V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME) {
			dst_frame->fb_addr[0].va += h264e_ctx->encoded_bs_len;
			dst_frame->fb_addr[0].pa += h264e_ctx->encoded_bs_len;
			dst_buf->vb2_buf.planes[0].bytesused = h264e_ctx->encoded_bs_len;
		} else {

			dst_buf->vb2_buf.planes[0].bytesused = h264e_ctx->encoded_bs_len;
			dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
			dst_buf->timecode = src_buf->timecode;

			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

			dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
			if(!dst_buf) {
				dev_err(ctx->dev->dev, "Failed to get dst buf!\n");
				ctx->state = INGENIC_STATE_ABORT;
				return;
			}

			dst_frame->fb_addr[0].pa = ingenic_vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
			if (dst_buf->vb2_buf.memory == VB2_MEMORY_USERPTR)
				dst_frame->fb_addr[0].va = phys_to_virt(dst_frame->fb_addr[0].pa);
			else
				dst_frame->fb_addr[0].va = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

			dst_buf->vb2_buf.planes[0].bytesused = 0;
		}
#endif

		ctx->state = INGENIC_STATE_RUNNING;
	} else {
		for(i = 0; i < dst_buf->vb2_buf.num_planes; i++) {
			dst_frame->fb_addr[i].pa = ingenic_vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, i);
			if (dst_buf->vb2_buf.memory == VB2_MEMORY_USERPTR)
				dst_frame->fb_addr[i].va = phys_to_virt(dst_frame->fb_addr[i].pa);
			else
				dst_frame->fb_addr[i].va = vb2_plane_vaddr(&dst_buf->vb2_buf, i);
		}
		//bs_buf->size = (size_t)dst_buf->vb2_buf.planes[0].length;
	}

	switch(ctx->codec_id) {
		case CODEC_ID_H264E:
			h264e_ctx->encoded_bs_len = 0;
			if(src_buf->flags & V4L2_BUF_FLAG_KEYFRAME) {
				force_idr = 1;
			}

			ret = h264e_encode(h264e_ctx, src_frame, &dst_frame->fb_addr[0], force_idr, &keyframe);

			if(keyframe) {
				dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
			} else {
				dst_buf->flags &= ~V4L2_BUF_FLAG_KEYFRAME;
			}
			dst_buf->vb2_buf.planes[0].bytesused += h264e_ctx->encoded_bs_len; /* returned bs size. */
			if(dst_buf->vb2_buf.planes[0].bytesused > dst_buf->vb2_buf.planes[0].length) {
				dev_err(dev->dev, "[dangerous] vpu output bs length too large, system will crash!! encoded_bs_len: %d\n",
					h264e_ctx->encoded_bs_len);
				ctx->state = INGENIC_STATE_ABORT;
			}
			break;
		case CODEC_ID_JPGE:

			ret = jpeg_encoder_encode(jpge_ctx, src_frame, &dst_frame->fb_addr[0]);
			if(ret < 0) {
				ctx->state = INGENIC_STATE_ABORT;
			}

			vb2_set_plane_payload(&dst_buf->vb2_buf, 0, jpge_ctx->bslen);

			//dst_buf->vb2_buf.planes[0].bytesused = jpge_ctx->bslen;
			break;
		case CODEC_ID_JPGD:
			/*src bs? dst raw? */
			ret = jpeg_decoder_decode(jpgd_ctx, &src_frame->fb_addr[0], dst_frame);
			if(ret < 0) {
				ctx->state = INGENIC_STATE_ABORT;
			}

			break;
		default:
			break;
	}

	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);

	dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
	dst_buf->timecode = src_buf->timecode;
	dst_buf->sequence = src_buf->sequence;

//	printk("dst_buf->timestamp.tv_sec: %ld, dst_buf->timestamp.tv_usec: %ld, sequence: %d, bytesused: %d\n", dst_buf->timestamp.tv_sec, dst_buf->timestamp.tv_usec, dst_buf->sequence, vb2_get_plane_payload(&dst_buf->vb2_buf, 0));

	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
}

static void m2mops_venc_device_run(void * priv)
{
	struct ingenic_venc_ctx *ctx = priv;
	struct ingenic_venc_dev *dev = ctx->dev;

	queue_work(dev->encode_workqueue, &ctx->encode_work);
}

static int m2mops_venc_job_ready(void *m2m_priv)
{
	struct ingenic_venc_ctx *ctx = m2m_priv;

	if(ctx->state == INGENIC_STATE_ABORT || ctx->state == INGENIC_STATE_IDLE) {
		dev_info(ctx->dev->dev, "job not ready, ctx->state %d\n", ctx->state);
		return 0;
	}

	return 1;
}


static void m2mops_venc_job_abort(void *priv)
{
	struct ingenic_venc_ctx *ctx = priv;

	ctx->state = INGENIC_STATE_ABORT;
}

const struct v4l2_m2m_ops ingenic_venc_m2m_ops = {
	.device_run		= m2mops_venc_device_run,
	.job_ready		= m2mops_venc_job_ready,
	.job_abort		= m2mops_venc_job_abort,
};



int ingenic_vcodec_enc_init_default_params(struct ingenic_venc_ctx *ctx)
{

	int ret = 0;


	ctx->capture_stopped = 1;
	ctx->output_stopped = 1;

	//ctx->m2m_ctx->q_lock = &ctx->dev->dev_mutex;
	//ctx->fh.m2m_ctx = ctx->m2m_ctx;
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	INIT_WORK(&ctx->encode_work, ingenic_venc_worker);

	return ret;
}

int ingenic_vcodec_enc_deinit_default_params(struct ingenic_venc_ctx *ctx)
{

	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;

	switch(ctx->codec_id) {
		case CODEC_ID_H264E:
			h264e_encoder_deinit(h264e_ctx);
			break;
		case CODEC_ID_JPGE:
			jpeg_encoder_deinit(jpge_ctx);
			break;
		case CODEC_ID_JPGD:
			jpeg_decoder_deinit(jpgd_ctx);
			break;
		default:
			break;
	}

	return 0;
}


#define INGENIC_MAX_CTRLS_HINT	20
int ingenic_vcodec_enc_ctrls_setup(struct ingenic_venc_ctx *ctx)
{
	const struct v4l2_ctrl_ops *ops = &ingenic_venc_ctrl_ops;
	struct v4l2_ctrl_handler *handler = &ctx->ctrl_hdl;

	v4l2_ctrl_handler_init(handler, INGENIC_MAX_CTRLS_HINT);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_JPEG_COMPRESSION_QUALITY,
			0, 2, 1, 1);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_BITRATE,
			1, 4000000, 1, 4000000);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
			0, 51, 1, 30);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
			0, 51, 1, 30);
//	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_B_FRAMES,
//			0, 2, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
			0, 1, 1, 1);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			0, 51, 1, 40);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
			0, 51, 1, 10);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			0, 65535, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			0, 65535, 1, 50);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
			0, 1, 1, 0);
	v4l2_ctrl_new_std_menu(handler, ops,
			V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			0, V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			0, V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			V4L2_MPEG_VIDEO_H264_LEVEL_4_2,
			0, V4L2_MPEG_VIDEO_H264_LEVEL_3_0);
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			V4L2_MPEG_VIDEO_BITRATE_MODE_CQ,
			0, V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	if (handler->error) {
		dev_err(ctx->dev->dev, "Init control handler fail %d\n",
				handler->error);
		return handler->error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);

	return 0;
}

int ingenic_vcodec_enc_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct ingenic_venc_ctx *ctx = priv;
	int ret = 0;

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size	= sizeof(struct ingenic_video_buf);
	src_vq->ops		= &ingenic_venc_vb2_ops;
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
	dst_vq->buf_struct_size	= sizeof(struct ingenic_video_buf);
	dst_vq->ops		= &ingenic_venc_vb2_ops;
	dst_vq->mem_ops		= &ingenic_vb2_dma_contig_memops;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	dst_vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->dev		= ctx->dev->dev;

	return vb2_queue_init(dst_vq);
}
