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
#include <linux/slab.h>
#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <linux/netlink.h>
#include <net/sock.h>

#include "api/helix_x264_enc.h"
#include "h264enc/common.h"
#include "h264enc/set.h"
#include "h264e_rc_nl.h"
#include "h264e_rc.h"
#include "helix_drv.h"

static unsigned add_sps_pps_iframe = 1;
module_param(add_sps_pps_iframe, uint, S_IRUGO | S_IWUSR);

#define MAX_REF_FRAMES	1
#define NUM_WORK_BUFFERS (MAX_REF_FRAMES + 1)

#if NUM_WORK_BUFFERS > 2

#error "Multi refrence encoding not support Yet, NUM_WORK_BUFFERS should be 2."

#endif

/*TODO: export to high level ????*/
int h264e_set_params(struct h264e_ctx *ctx, unsigned id, unsigned int val)
{
	int ret = 0;


	return ret;
}

int h264e_get_params(struct h264e_ctx *ctx, int id)
{
	unsigned int val = 0;

	return val;
}

int h264e_set_fmt(struct h264e_ctx *ctx, int width, int height, int format)
{
	struct h264e_params *p = &ctx->p;
	unsigned int align_w = width;
	unsigned int align_h = height;

	int ret = 0;


	p->width = width;
	p->height = height;
	switch(format) {
		case HELIX_NV12_MODE:
		case HELIX_NV21_MODE:
		case HELIX_TILE_MODE:
		case HELIX_420P_MODE:
			p->format = format;
			break;
		default:
			pr_err("Unsupported pix fmt: %x\n", format);
			ret = -EINVAL;
			break;
	}

	align_w = ALIGN(width, 16);
	align_h = ALIGN(height, 16);

	ctx->framesize = align_w * align_h;

	h264e_nl_rc_video_cfg(ctx, &ctx->p);

	return ret;
}

//to profile_idc
static int __maybe_unused v4l2_profile(int i_profile_idc)
{
	switch (i_profile_idc) {
		case PROFILE_BASELINE:           return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
		case PROFILE_MAIN:               return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
		case PROFILE_HIGH10:             return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10;
		case PROFILE_HIGH422:            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422;
		case PROFILE_HIGH444_PREDICTIVE: return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
		default: return -EINVAL;
	}

}

static int __maybe_unused h264e_profile_idc(int v4l2_profile)
{
	switch (v4l2_profile) {
		case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE: 		return PROFILE_BASELINE;
		case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN: 		return PROFILE_MAIN;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10: 		return PROFILE_HIGH10;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422: 		return PROFILE_HIGH422;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE: 	return PROFILE_HIGH444_PREDICTIVE;
		default: return -EINVAL;
	}
}

static int __maybe_unused v4l2_h264_level(int i_level_idc)
{
	switch (i_level_idc) {
		case 10: return V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
		case 9:  return V4L2_MPEG_VIDEO_H264_LEVEL_1B;
		case 11: return V4L2_MPEG_VIDEO_H264_LEVEL_1_1;
		case 12: return V4L2_MPEG_VIDEO_H264_LEVEL_1_2;
		case 13: return V4L2_MPEG_VIDEO_H264_LEVEL_1_3;
		case 20: return V4L2_MPEG_VIDEO_H264_LEVEL_2_0;
		case 21: return V4L2_MPEG_VIDEO_H264_LEVEL_2_1;
		case 22: return V4L2_MPEG_VIDEO_H264_LEVEL_2_2;
		case 30: return V4L2_MPEG_VIDEO_H264_LEVEL_3_0;
		case 31: return V4L2_MPEG_VIDEO_H264_LEVEL_3_1;
		case 32: return V4L2_MPEG_VIDEO_H264_LEVEL_3_2;
		case 40: return V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
		case 41: return V4L2_MPEG_VIDEO_H264_LEVEL_4_1;
		default: return -EINVAL;
	}
}

static int h264e_level_idc(int level)
{
	switch (level) {
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_0: return 10;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1B:  return 9;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_1: return 11;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_2: return 12;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_3: return 13;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_0: return 20;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_1: return 21;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_2: return 22;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_0: return 30;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_1: return 31;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_2: return 32;
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_0: return 40;
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_1: return 41;
		default: return -EINVAL;
	}
}

/* ----------------------------- Picture Parameter Sets ---------------*/
static void h264e_sps_init(struct h264e_ctx *ctx, h264_sps_t *sps, int i_id)
{
	struct h264e_params *p = &ctx->p;
	int max_frame_num = 1;

	sps->i_id 			= i_id;
	sps->i_mb_width 		= C_ALIGN(p->width, 16) / 16;
	sps->i_mb_height 		= C_ALIGN(p->height, 16) / 16;
	sps->i_chroma_format_idc 	= CHROMA_420;
	sps->b_qpprime_y_zero_transform_bypass = 0;
	sps->i_profile_idc 		= h264e_profile_idc(p->h264_profile);
	sps->b_constraint_set0 		= sps->i_profile_idc == PROFILE_BASELINE;
	sps->b_constraint_set1 		=  sps->i_profile_idc <= PROFILE_MAIN;
	sps->b_constraint_set2 		= 0;
	sps->b_constraint_set3 		= 0;

	sps->i_level_idc 		= h264e_level_idc(p->h264_level);
	if((sps->i_level_idc == 9) && (sps->i_profile_idc == PROFILE_BASELINE || sps->i_profile_idc == PROFILE_MAIN)) {
		sps->b_constraint_set0 	= 1; /* level 1b with Baseline or Main profile is signalled via constraint_set3 */
		sps->i_level_idc 	= 11;
	}


	sps->i_num_ref_frames 		= p->max_ref_pic;
	sps->vui.i_num_reorder_frames 	= 0;
	sps->vui.i_max_dec_frame_buffering = sps->i_num_ref_frames;

	sps->i_log2_max_frame_num 	= 4;
	while((1 << sps->i_log2_max_frame_num) <= max_frame_num)
		sps->i_log2_max_frame_num++;

	sps->i_poc_type = 2;

	if(sps->i_poc_type == 0) {
	}

	sps->b_vui = 0;

	sps->b_gaps_in_frame_num_value_allowed = 0;
	sps->b_frame_mbs_only = 1;
	if(!sps->b_frame_mbs_only)
		sps->i_mb_height = (sps->i_mb_height + 1) & ~1;
	sps->b_mb_adaptive_frame_field = p->interlaced;
	sps->b_direct8x8_inference = 1;
	sps->crop.i_left = 0;
	sps->crop.i_right = sps->i_mb_width * 16 - p->width;
	sps->crop.i_top = 0;
	sps->crop.i_bottom = sps->i_mb_height * 16 - p->height;

	if(sps->crop.i_right || sps->crop.i_bottom) {
		sps->b_crop = 1;
	} else {
		sps->b_crop = 0;
	}

//	h264_sps_init_reconfigurable(sps, sw);


	sps->vui.b_overscan_info_present = 0;
	sps->vui.b_overscan_info = 0;

	sps->i_cqm_preset = 0;
	sps->b_avcintra = 0;

}

static void h264e_pps_init(struct h264e_ctx *ctx, h264_pps_t *pps, int i_id, h264_sps_t *sps)
{
	struct h264e_params *p = &ctx->p;

	pps->i_id = i_id;
	pps->i_sps_id = sps->i_id;
	pps->b_cabac = sps->i_profile_idc < PROFILE_MAIN ? 0 : p->i_cabac;

	pps->b_pic_order = 0;
	pps->i_num_slice_groups = 1; /* base & main profile.*/

	pps->i_num_ref_idx_l0_default_active = p->max_ref_pic;
	pps->i_num_ref_idx_l1_default_active = 1;

	pps->b_weighted_pred = 0;
	pps->b_weighted_bipred = 0;

	pps->i_pic_init_qp = 26;
	pps->i_pic_init_qs = 26;

	pps->i_chroma_qp_index_offset = 0;
	pps->b_deblocking_filter_control = 0;
	pps->b_constrained_intra_pred = 0;
	pps->b_redundant_pic_cnt = 0;

	pps->b_transform_8x8_mode = p->h264_8x8_transform;
}


static int encapsulate_nal(bs_t *s, int s_off, char *dst, int i_type, int i_ref_idc)
{

	uint8_t *src = s->p_start + s_off;
	uint8_t *end = src + (bs_pos(s) / 8);
	char *orig_dst = dst;

	/* prefix */
	*dst++ = 0x00;
	*dst++ = 0x00;
	*dst++ = 0x00;
	*dst++ = 0x01;

	/* nal header */
	*dst++ = (0 << 7) | (i_ref_idc << 5) | (i_type);

	if(src < end) *dst++ = *src++;
	if(src < end) *dst++ = *src++;
	while( src < end )
	{
		if( src[0] <= 0x03 && !src[-2] && !src[-1] ) {
			*dst++ = 0x03;
		}
		*dst++ = *src++;
	}

	return dst - orig_dst;
}

/**
* @h264e_generate_headers generate sps/pps headers according to params.
*			and store the header until encoder closed.
* @ctx h264e_ctx.
*
* @Return 0
*/
int h264e_generate_headers(struct h264e_ctx *ctx, int is_stream_on)
{
	/* used to encode header for tmp.*/
	char *bs_tmp = NULL;
	char *bs_header = ctx->bs_header;
	bs_t bs;
	int bs_size = 0;
	h264_sps_t *sps = NULL;
	h264_pps_t *pps = NULL;
	ctx->bs_header_size = 0;

	bs_tmp = kzalloc(MAX_BS_HEADER_SIZE, GFP_KERNEL);
	if(!bs_tmp) {
		return -ENOMEM;
	}

	sps = &ctx->sps;
	pps = &ctx->pps;

	h264e_sps_init(ctx, sps, 0);
	h264e_pps_init(ctx, pps, 0, sps);

	/* 1. encode sps */
	bs_init(&bs, bs_tmp, MAX_BS_HEADER_SIZE);
	h264e_sps_write(&bs, sps);

	bs_size = encapsulate_nal(&bs, 0, bs_header, NAL_SPS, NAL_PRIORITY_HIGHEST);

	ctx->bs_header_size += bs_size;
	bs_header	    += bs_size;

	/* 2. encode pps */
	bs_init(&bs, bs_tmp, MAX_BS_HEADER_SIZE);
	h264e_pps_write(&bs, sps, pps);

	bs_size = encapsulate_nal(&bs, 0, bs_header, NAL_PPS, NAL_PRIORITY_HIGHEST);
	ctx->bs_header_size += bs_size;
	bs_header 	   += bs_size;


	if(ctx->bs_header_size > (MAX_BS_HEADER_SIZE)) {
		pr_warn("bs_header buffer almost overflow!\n");
	}

	/* 3. encode sei */

	/* 4. encode more */

	if(is_stream_on) {

		struct nl_header_info h_info;

		memcpy(&h_info.pps, pps, sizeof(h264_pps_t));
		memcpy(&h_info.sps, sps, sizeof(h264_sps_t));

		h264e_nl_setup_headers(ctx, &h_info);
	}

	/* release temp buffer. */
	kfree(bs_tmp);
	return 0;
}

/* add 0xff nal .*/
static int h264e_filler_nal(int size, char *p)
{
        if (size < 6)
                return -EINVAL;

        p[0] = 0x00;
        p[1] = 0x00;
        p[2] = 0x00;
        p[3] = 0x01;
        p[4] = 0x0c;
        memset(p + 5, 0xff, size - 6);
        /* Add rbsp stop bit and trailing at the end */
        p[size - 1] = 0x80;

        return 0;
}

static int h264e_padding(int size, char *p)
{
	int ret = 0;

	if(size == 0)
		return 0;

        ret = h264e_filler_nal(size, p);
	if(ret < 0)
		return ret;

        return size;
}

int h264e_encode_headers(struct h264e_ctx *ctx, struct ingenic_vcodec_mem *bs)
{
	int ret = 0;
	unsigned int align = 0;

	if(ctx->encoded_bs_len != 0) {
		pr_info("set encoded_bs_len to 0 before encode!\n");
	}

	memcpy(bs->va, ctx->bs_header, ctx->bs_header_size);

	/*align to 256*/
	align = (ctx->bs_header_size + 255) / 256 * 256;
	ret = h264e_padding(align - ctx->bs_header_size, bs->va + ctx->bs_header_size);
	ctx->encoded_bs_len = align;

	ret = ctx->encoded_bs_len;

	return ret;
}

void xhexdump(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%s%08x: ", i ? "\n" : "",
					(unsigned int)&buf[i]);
		printf("%02x ", buf[i]);
	}
	printf("\n");
}


int h264e_encode(struct h264e_ctx *ctx, struct video_frame_buffer *frame,
		struct ingenic_vcodec_mem *bs_mem, int force_idr, int *keyframe)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	unsigned int slice_header_size = 0;
	unsigned int slice_header_len = 0;
	int idr = 0;
	int i_nal_type = 0;
	int i_nal_ref_idc = 0;
	bs_t bs;
	int ret = 0;
	int i = 0;
#if 0
	unsigned char bmap[3][3] = {
		{0, 2, 1},
		{1, 0, 2},
		{2, 1, 0}
	};
#endif

	unsigned char bmap[2][2] = {
		{0, 1},
		{1, 0},
	};
	unsigned int encoded_header_size = 0;

	struct h264e_frame_start_params p;

	memset(&p, 0, sizeof(struct h264e_frame_start_params));

	ctx->frame = frame;
	ctx->bs = bs_mem;

	if(!(ctx->i_frame % ctx->p.gop_size) || force_idr) {
		ctx->i_frame = 0;
		idr = 1;
	}

	unsigned char *bufidx = bmap[ctx->i_frame % NUM_WORK_BUFFERS];

	p.bIFrmReq = idr;
	if(idr) {
		p.frmSkipType = 0;
	} else {
		p.frmSkipType = 1;
	}

	p.raw_format	= ctx->p.format;
	for(i = 0; i < ctx->frame->num_planes; i++) {
		p.raw[i] = ctx->frame->fb_addr[i].pa;
	}

	for(i = 0; i < NUM_WORK_BUFFERS; i++) {
		p.fb[i][0] = ctx->fb[bufidx[i]].yaddr_pa;
		p.fb[i][1] = ctx->fb[bufidx[i]].caddr_pa;
	}

	p.stride[0] 		= ctx->p.width;
	p.stride[1]		= p.raw_format == HELIX_420P_MODE ?
				  ctx->p.width / 2 : ctx->p.width;

	if(idr) {
		*keyframe = idr;
		if(*keyframe && add_sps_pps_iframe) {
			/* insert sps/pps.*/
			encoded_header_size = h264e_encode_headers(ctx, ctx->bs);
			ctx->bs->va += encoded_header_size;
			ctx->bs->pa += encoded_header_size;
		}

	}

	p.emc_bs_pa = ctx->bs->pa;
	p.emc_dblk_pa = ctx->emc_buf_pa;

	h264e_nl_rc_frame_start(ctx, &p);

	dma_sync_single_for_device(venc_ctx->dev->dev, ctx->desc_pa, ctx->vdma_chain_len, DMA_TO_DEVICE);

	/*2. 启动编码器硬件，开始编码*/
	ret = ingenic_vpu_start(ctx->priv);
	if(ret < 0) {
		return ret;
	}

	//print_hex_dump(KERN_INFO, "bs@", DUMP_PREFIX_ADDRESS, 16, 1, ctx->bs->va, 64, 1);


	ctx->encoded_bs_len = encoded_header_size + ctx->r_bs_len + ctx->r_rbsp_len + p.slice_header_len;

	unsigned char *xp = (unsigned char *)((unsigned long)ctx->bs->va | 0xa0000000);
	xp[0] = 0x00;
	xp[1] = 0x00;
	xp[2] = 0x00;


	struct h264e_frame_end_params ep;
	ep.u32FrmActBs = ctx->r_bs_len * 8;
	h264e_nl_rc_frame_end(ctx, &ep);


	//printk("ctx->encoded_bs_len: %d\n", ctx->encoded_bs_len);

	ctx->i_frame++;

	return ret;
}



/* start streaming.*/
int h264e_alloc_workbuf(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	int allocate_fb = 0;
	int ret = 0;
	int i = 0;



	/* init yaddr. */
	for(i = 0; i < 3; i++) {
		ctx->fb[i].yaddr = NULL;
		ctx->fb[i].caddr = NULL;
	}

	for(i = 0; i < NUM_WORK_BUFFERS; i++) {
		ctx->fb[i].yaddr = dma_alloc_coherent(venc_ctx->dev->dev, ctx->framesize, &ctx->fb[i].yaddr_pa, GFP_KERNEL);
		ctx->fb[i].caddr = dma_alloc_coherent(venc_ctx->dev->dev, ctx->framesize/2, &ctx->fb[i].caddr_pa, GFP_KERNEL);

		allocate_fb = i;
		if(!ctx->fb[i].yaddr || !ctx->fb[i].caddr) {
			ret = -ENOMEM;
			goto err_fb;
		}
	}

	return ret;

err_emc_buf:
err_fb:
	for(i = 0; i < NUM_WORK_BUFFERS; i++) {
		if(ctx->fb[i].yaddr) {
			dma_free_coherent(venc_ctx->dev->dev, ctx->framesize, ctx->fb[i].yaddr, ctx->fb[i].yaddr_pa);
		}
		if(ctx->fb[i].caddr) {
			dma_free_coherent(venc_ctx->dev->dev, ctx->framesize/2, ctx->fb[i].caddr, ctx->fb[i].caddr_pa);
		}

	}

	return ret;
}

/* stop streaming.*/
int h264e_free_workbuf(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	int i;

	for(i = 0; i < NUM_WORK_BUFFERS; i++) {
		dma_free_coherent(venc_ctx->dev->dev, ctx->framesize, ctx->fb[i].yaddr, ctx->fb[i].yaddr_pa);
		dma_free_coherent(venc_ctx->dev->dev, ctx->framesize/2, ctx->fb[i].caddr, ctx->fb[i].caddr_pa);
	}
	return 0;
}

int h264e_encoder_init(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	struct h264e_params *p = NULL;
	int ret = 0;

	ctx->emc_buf = dma_alloc_coherent(venc_ctx->dev->dev, EMC_SIZE, &ctx->emc_buf_pa, GFP_KERNEL);
	if(!ctx->emc_buf) {
		pr_err("Failed to alloc vpu emc_buf!\n");
		ret = -ENOMEM;
		goto err_emc_buf;
	}



	p = &ctx->p;

	ctx->vdma_chain_len 	= 40960 + 256;
	ctx->frameindex		= 0;
	ctx->i_frame		= 0;

	p->height		= 240;
	p->width		= 160;
	p->max_ref_pic		= 1;
	p->i_idr_pic_id		= 0;
	p->i_global_qp 		= 30;
	p->i_cabac		= 1;
	p->interlaced		= 0;
	p->h264_8x8_transform	= 0;
	p->deblock		= 0;
//	p->h264_hdr_mode	= 1;

	ctx->desc = dma_alloc_noncoherent(venc_ctx->dev->dev, ctx->vdma_chain_len, &ctx->desc_pa, DMA_BIDIRECTIONAL, GFP_KERNEL);
	if(!ctx->desc) {
		pr_err("Failed to alloc vpu desc buffer!\n");
		ret = -ENOMEM;
		goto err_desc;
	}

	struct nl_ctx_info info;

	info.emc_buf_paddr	= (unsigned int)ctx->emc_buf_pa;
	info.desc_paddr		= (unsigned int)ctx->desc_pa;
	info.rc_mode		= p->rc_mode;

	ret = h264e_nl_init_ctx(ctx, &info);

	return ret;
err_desc:
err_emc_buf:

	return ret;
}

int h264e_encoder_deinit(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	if(!ctx)
		return 0;

	if(ctx->emc_buf) {
		dma_free_coherent(venc_ctx->dev->dev, EMC_SIZE, ctx->emc_buf, ctx->emc_buf_pa);
		ctx->emc_buf = NULL;
	}

	if(ctx->desc) {
		dma_free_noncoherent(venc_ctx->dev->dev, ctx->vdma_chain_len, ctx->desc, ctx->desc_pa, DMA_BIDIRECTIONAL);
		ctx->desc = NULL;
	}

	h264e_nl_free_ctx(ctx);

	return 0;
}

void h264e_set_priv(struct h264e_ctx *ctx, void *data)
{
	ctx->priv = data;
}
