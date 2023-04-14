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

#include "api/helix_x264_enc.h"

#include "h264enc/common.h"
#include "h264enc/set.h"
#include "h264e.h"
#include "helix_drv.h"


static uint8_t cqm_8iy_tab[64] = {
    6+10,10+10,13+10,16+10,18+10,23+10,25+10,27+10,
    10+10,11+10,16+10,18+10,23+10,25+10,27+10,29+10,
    13+10,16+10,18+10,23+10,25+10,27+10,29+10,31+10,
    16+10,18+10,23+10,25+10,27+10,29+10,31+10,33+10,
    18+10,23+10,25+10,27+10,29+10,31+10,33+10,36+10,
    23+10,25+10,27+10,29+10,31+10,33+10,36+10,38+10,
    25+10,27+10,29+10,31+10,33+10,36+10,38+10,40+10,
    27+10,29+10,31+10,33+10,36+10,38+10,40+10,42+10
};
static uint8_t cqm_8py_tab[64] = {
    9+10,13+10,15+10,17+10,19+10,21+10,22+10,24+10,
    13+10,13+10,17+10,19+10,21+10,22+10,24+10,25+10,
    15+10,17+10,19+10,21+10,22+10,24+10,25+10,27+10,
    17+10,19+10,21+10,22+10,24+10,25+10,27+10,28+10,
    19+10,21+10,22+10,24+10,25+10,27+10,28+10,30+10,
    21+10,22+10,24+10,25+10,27+10,28+10,30+10,32+10,
    22+10,24+10,25+10,27+10,28+10,30+10,32+10,33+10,
    24+10,25+10,27+10,28+10,30+10,32+10,33+10,35+10
};


/* static int      rc_deadzone[9]         = {0x1d,0x12,0x15,0xb,0x1d,0x12,0x1d,0x15,0xb}; */
static uint16_t rc_skin_ofst[4]        = {97, 183, 271, 884};//[0]: 1.5, [1]:0.4, [2]:0.6, [3]: 5.1
static uint8_t  rc_mult_factor[3]      = {3,9,5};
//static int8_t   rc_skin_qp_ofst[4]     = {-3, -2, -1, 0};
static int8_t   rc_skin_qp_ofst[4]     = {-6, -4, -2, 0};
static uint8_t  rc_skin_pxlu_thd[3][2] = {{100, 120},{50, 100},{120, 150}};
static uint8_t  rc_skin_pxlv_thd[3][2] = {{140, 175},{125, 140},{175, 225}};
static uint8_t  rc_shift_factor[3][3]  = {{3,5,8},{4,5,6},{4,5,6}};//[0][]:0.4, [1][]:0.6, [2][]: 5.1


static unsigned add_sps_pps_iframe = 1;
module_param(add_sps_pps_iframe, uint, S_IRUGO | S_IWUSR);

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

	ctx->framesize = width * height;

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
	sps->b_crop = 0;

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
	pps->b_cabac = p->i_cabac;

	pps->b_pic_order = 0;
	pps->i_num_slice_groups = 1; /* base & main profile.*/

	pps->i_num_ref_idx_l0_default_active = p->max_ref_pic;
	pps->i_num_ref_idx_l1_default_active = 1;

	pps->b_weighted_pred = 0;
	pps->b_weighted_bipred = 0;

	pps->i_pic_init_qp = 0;
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
int h264e_generate_headers(struct h264e_ctx *ctx)
{
	/* used to encode header for tmp.*/
	char *bs_tmp = NULL;
	char *bs_header = ctx->bs_header;
	bs_t bs;
	int bs_size = 0;
	h264_sps_t *sps = NULL;
	h264_pps_t *pps = NULL;

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


	if(ctx->bs_header_size > (MAX_BS_HEADER_SIZE - 512)) {
		pr_warn("bs_header buffer almost overflow!\n");
	}

	/* 3. encode sei */

	/* 4. encode more */

	/* release temp buffer. */
	kfree(bs_tmp);
	return 0;
}

static int h264e_slice_init(struct h264e_ctx * ctx, int i_nal_type)
{
	if(i_nal_type == NAL_SLICE_IDR) {

		h264e_slice_header_init(&ctx->sh, &ctx->sps, &ctx->pps, ctx->p.i_idr_pic_id, ctx->i_frame, ctx->p.i_qp);
		ctx->p.i_idr_pic_id ^= 1;
	} else {
		h264e_slice_header_init(&ctx->sh, &ctx->sps, &ctx->pps, -1, ctx->i_frame, ctx->p.p_qp);

		ctx->sh.i_num_ref_idx_l0_active = 1;
		ctx->sh.i_num_ref_idx_l1_active = 1;
	}

	return 0;
}

#if 0
void H264E_DumpInfo(_H264E_SliceInfo *s)
{
    printk(" emc_qpt_va              : %p\n",s->emc_qpt_va);
    printk(" emc_rc_va               : %p\n",s->emc_rc_va);
    printk(" emc_cpx_va              : %p\n",s->emc_cpx_va);
    printk(" emc_mod_va              : %p\n",s->emc_mod_va);
    printk(" emc_ncu_va              : %p\n",s->emc_ncu_va);
    printk(" emc_sad_va              : %p\n",s->emc_sad_va);
    printk(" mb_mode_info            : %p\n",s->mb_mode_info);

    printk("s->des_va   = 0x%08x\n", (unsigned int)s->des_va);
    printk("s->des_pa   = 0x%08x\n", (unsigned int)s->des_pa);
    //	printk("s->emc_bs_va = 0x%08x\n", s->emc_bs_va);
    printk("s->emc_bs_pa = 0x%08x\n", s->emc_bs_pa);

    /*x2000 add*/
    printk(" bs_head_en = %d\n", s->bs_head_en);
    printk(" bs_head_va = 0x%08x\n", (unsigned int)s->bs_head_va);
    printk(" bs_head_len = 0x%d\n", (unsigned int)s->bs_head_len);
    printk(" bs_rbsp_en = %d\n", s->bs_rbsp_en);

    printk(" stride[0]    = %d\n", s->stride[0]);
    printk(" stride[1]    = %d\n", s->stride[1]);
    printk(" state	= 0x%08x\n", (unsigned int)s->state);
    printk(" raw[0]   = 0x%08x\n", s->raw[0]);
    printk(" raw[1]   = 0x%08x\n", s->raw[1]);
    printk(" raw[2]   = 0x%08x\n", s->raw[2]);
    printk(" fb[0][0] = 0x%08x\n", s->fb[0][0]);
    printk(" fb[0][1] = 0x%08x\n", s->fb[0][1]);
    printk(" fb[1][0] = 0x%08x\n", s->fb[1][0]);
    printk(" fb[1][1] = 0x%08x\n", s->fb[1][1]);
    printk(" fb[2][0] = 0x%08x\n", s->fb[2][0]);
    printk(" fb[2][1] = 0x%08x\n", s->fb[2][1]);
    printk(" jh[0][0] = 0x%08x\n", s->jh[0][0]);
    printk(" jh[0][1] = 0x%08x\n", s->jh[0][1]);
    printk(" jh[1][0] = 0x%08x\n", s->jh[1][0]);
    printk(" jh[1][1] = 0x%08x\n", s->jh[1][1]);
    printk(" jh[2][0] = 0x%08x\n", s->jh[2][0]);
    printk(" jh[2][1] = 0x%08x\n", s->jh[2][1]);
    printk(" spe_y_addr = 0x%08x\n", s->spe_y_addr);
    printk(" spe_c_addr = 0x%08x\n", s->spe_c_addr);
    printk(" emc_recon_pa= 0x%08x\n", s->emc_recon_pa);
    printk(" emc_qpt_pa  = 0x%08x\n", s->emc_qpt_pa);
    printk(" emc_mv_pa   = 0x%08x\n", s->emc_mv_pa);
    printk(" emc_se_pa   = 0x%08x\n", s->emc_se_pa);
    printk(" emc_rc_pa   = 0x%08x\n", s->emc_rc_pa);
    printk(" emc_cpx_pa  = 0x%08x\n", s->emc_cpx_pa);
    printk(" emc_mod_pa  = 0x%08x\n", s->emc_mod_pa);
    printk(" emc_ncu_pa  = 0x%08x\n", s->emc_ncu_pa);
    printk(" emc_sad_pa  = 0x%08x\n", s->emc_sad_pa);
    /* 1. rc output [11]*/
    {
        printk(" frame_type              : %d\n",s->frame_type);
        printk(" mb_width                : %d\n",s->mb_width);
        printk(" mb_height               : %d\n",s->mb_height);
        printk(" frame_width             : %d\n",s->frame_width);
        printk(" frame_height            : %d\n",s->frame_height);
        printk(" first_mby               : %d\n",s->first_mby);
        printk(" last_mby                : %d\n",s->last_mby);
        printk(" qp                      : %d\n",s->qp);
        printk(" base_qp                 : %d\n",s->base_qp);
        printk(" max_qp                  : %d\n",s->max_qp);
        printk(" min_qp                  : %d\n",s->min_qp);
    }
    /* 2. motion cfg [25]*/
    {
        printk(" frm_re[0]              : %d\n",s->frm_re[0]);
        printk(" frm_re[1]              : %d\n",s->frm_re[1]);
        printk(" frm_re[2]              : %d\n",s->frm_re[2]);
        printk(" frm_re[3]              : %d\n",s->frm_re[3]);
        printk(" pskip_en               : %d\n",s->pskip_en);
        printk(" mref_en                : %d\n",s->mref_en);
        printk(" scl                    : %d\n",s->scl);
        printk(" hpel_en                : %d\n",s->hpel_en);
        printk(" qpel_en                : %d\n",s->qpel_en);
        printk(" ref_mode               : %d\n",s->ref_mode);
        printk(" max_sech_step_i        : %d\n",s->max_sech_step_i);
        printk(" max_mvrx_i             : %d\n",s->max_mvrx_i);
        printk(" max_mvry_i             : %d\n",s->max_mvry_i);
        printk(" lambda_scale_parameter : %d\n",s->lambda_scale_parameter);
        printk(" fs_en                  : %d\n",s->fs_en);
        printk(" fs_md                  : %d\n",s->fs_md);
        printk(" fs_px                  : %d\n",s->fs_px);
        printk(" fs_py                  : %d\n",s->fs_py);
        printk(" fs_rx                  : %d\n",s->fs_rx);
        printk(" fs_ry                  : %d\n",s->fs_ry);
        printk(" frm_mv_en              : %d\n",s->frm_mv_en);
        printk(" frm_mv_size            : %d\n",s->frm_mv_size);
        printk(" glb_mv_en              : %d\n",s->glb_mv_en);
        printk(" glb_mvx                : %d\n",s->glb_mvx);
        printk(" glb_mvy                : %d\n",s->glb_mvy);
        printk(" me_step_en             : %d\n",s->me_step_en);
        printk(" me_step_0              : %d\n",s->me_step_0);
        printk(" me_step_1              : %d\n",s->me_step_1);
    }
    /* 3. quant cfg [4]*/
    {
        int i=0,j=0;
        printk(" dct8x8_en    : %d\n",s->dct8x8_en);
        for (i=0;i<4;i++)
            for (j=0;j<16;j++)
                printk(" scaling_list[%d][%d] : %d\n",i,j,s->scaling_list[i][j]);
        for (i=0;i<2;i++)
            for (j=0;j<64;j++)
                printk(" scaling_list8[%d][%d] : %d\n",i,j,s->scaling_list8[i][j]);
        for (i=0;i<9;i++)
            printk(" deadzone[%d] : %d\n",i,s->deadzone[i]);
    }

    /* 4. loop filter cfg [4]*/
    {
        printk(" deblock                : %d\n",s->deblock);
        printk(" rotate                 : %d\n",s->rotate);
        printk(" alpha_c0_offset        : %d\n",s->alpha_c0_offset);
        printk(" beta_offset            : %d\n",s->beta_offset);
    }
    /* 5. do not douch, default value [56]*/
    {
        printk(" acmask_mode : %d\n",s->acmask_mode);
        printk(" intra_mode_msk : %d\n",s->intra_mode_msk);
        printk(" i_4x4_dis              : %d\n",s->i_4x4_dis);
        printk(" i_8x8_dis              : %d\n",s->i_8x8_dis);
        printk(" i_16x16_dis            : %d\n",s->i_16x16_dis);
        printk(" p_l0_dis               : %d\n",s->p_l0_dis);
        printk(" p_t8_dis               : %d\n",s->p_t8_dis);
        printk(" p_skip_dis             : %d\n",s->p_skip_dis);
        printk(" p_skip_pl0f_dis        : %d\n",s->p_skip_pl0f_dis);
        printk(" p_skip_pt8f_dis        : %d\n",s->p_skip_pt8f_dis);

        printk(" cost_bias_en           : %d\n",s->cost_bias_en);
        printk(" cost_bias_i_4x4        : %d\n",s->cost_bias_i_4x4);
        printk(" cost_bias_i_8x8        : %d\n",s->cost_bias_i_8x8);
        printk(" cost_bias_i_16x16      : %d\n",s->cost_bias_i_16x16);
        printk(" cost_bias_p_l0         : %d\n",s->cost_bias_p_l0);
        printk(" cost_bias_p_t8         : %d\n",s->cost_bias_p_t8);
        printk(" cost_bias_p_skip       : %d\n",s->cost_bias_p_skip);

        printk(" intra_lambda_y_bias_en : %d\n",s->intra_lambda_y_bias_en);
        printk(" intra_lambda_c_bias_en : %d\n",s->intra_lambda_c_bias_en);
        printk(" intra_lambda_bias_qp0  : %d\n",s->intra_lambda_bias_qp0);
        printk(" intra_lambda_bias_qp1  : %d\n",s->intra_lambda_bias_qp1);
        printk(" intra_lambda_bias_0    : %d\n",s->intra_lambda_bias_0);
        printk(" intra_lambda_bias_1    : %d\n",s->intra_lambda_bias_1);
        printk(" intra_lambda_bias_2    : %d\n",s->intra_lambda_bias_2);

        printk(" chroma_sse_bias_en     : %d\n",s->chroma_sse_bias_en);
        printk(" chroma_sse_bias_qp0    : %d\n",s->chroma_sse_bias_qp0);
        printk(" chroma_sse_bias_qp1    : %d\n",s->chroma_sse_bias_qp1);
        printk(" chroma_sse_bias_0      : %d\n",s->chroma_sse_bias_0);
        printk(" chroma_sse_bias_1      : %d\n",s->chroma_sse_bias_1);
        printk(" chroma_sse_bias_2      : %d\n",s->chroma_sse_bias_2);

        printk(" sse_lambda_bias_en     : %d\n",s->sse_lambda_bias_en);
        printk(" sse_lambda_bias        : %d\n",s->sse_lambda_bias);
        printk(" fbc_ep                 : %d\n",s->fbc_ep);
        printk(" jm_lambda2_en          : %d\n",s->jm_lambda2_en);
        printk(" inter_nei_en           : %d\n",s->inter_nei_en);
        printk(" skip_bias_en           : %d\n",s->skip_bias_en);

        printk(" ysse_thr               : %d\n",s->ysse_thr);
        printk(" csse_thr               : %d\n",s->csse_thr);
        printk(" dcm_en                 : %d\n",s->dcm_en);
        printk(" dcm_param              : %d\n",s->dcm_param);
        printk(" sde_prior              : %d\n",s->sde_prior);
        printk(" db_prior               : %d\n",s->db_prior);

        printk(" use_intra_in_pframe    : %d\n",s->use_intra_in_pframe);
        printk(" use_fast_mvp           : %d\n",s->use_fast_mvp);
        printk(" skip_en                : %d\n",s->skip_en);
        printk(" cqp_offset             : %d\n",s->cqp_offset);

        printk(" daisy_chain_en         : %d\n",s->daisy_chain_en);
        printk(" curr_thread_id         : %d\n",s->curr_thread_id);
        printk(" qp_tab_mode            : %d\n",s->qp_tab_mode);
        printk(" bs_size_en             : %d\n",s-> bs_size_en);
        printk(" bs_size                : %d\n",s->bs_size);
        printk(" raw_format             : %d\n",s->raw_format);

        printk(" size_mode              : %d\n",s->size_mode);
        printk(" step_mode              : %d\n",s->step_mode);
        printk(" mode_ctrl              : %d\n",s->mode_ctrl);
        int i=0;
        for (i=0;i<40;i++)
            printk("mode_ctrl_param[%d] : %d\n",i,s->mode_ctrl_param[i]);

    }
    /* 6. select hardware output mode [11]*/
    {
        printk(" info_en                : %d\n",s->info_en);
        printk(" mvd_sum_all            : %d\n",s->mvd_sum_all);
        printk(" mvd_sum_abs            : %d\n",s->mvd_sum_abs);
        printk(" mv_sum_all             : %d\n",s->mv_sum_all);
        printk(" mv_sum_abs             : %d\n",s->mv_sum_abs);


        printk(" cfg_size_x             : %d\n",s->cfg_size_x);
        printk(" cfg_size_y             : %d\n",s->cfg_size_y);
        printk(" cfg_iw_thr             : %d\n",s->cfg_iw_thr);

        printk(" cfg_mvr_thr1           : %d\n",s->cfg_mvr_thr1);
        printk(" cfg_mvr_thr2           : %d\n",s->cfg_mvr_thr2);
        printk(" cfg_mvr_thr3           : %d\n",s->cfg_mvr_thr3);
    }
    /* 7. ipred bit&lambda ctrl [33]*/
    {
        printk(" mb_mode_val            : %d\n",s->mb_mode_val);
        printk(" bit_16_en              : %d\n",s->bit_16_en);
        printk(" bit_8_en               : %d\n",s->bit_8_en);
        printk(" bit_4_en               : %d\n",s->bit_4_en);
        printk(" bit_uv_en              : %d\n",s->bit_uv_en);
        printk(" lamb_16_en             : %d\n",s->lamb_16_en);
        printk(" lamb_8_en              : %d\n",s->lamb_8_en);
        printk(" lamb_4_en              : %d\n",s->lamb_4_en);
        printk(" lamb_uv_en             : %d\n",s->lamb_uv_en);
        printk(" lamb_uv_en             : %d\n",s->c_16_en);
        printk(" c_8_en                 : %d\n",s->c_8_en);
        printk(" c_4_en                 : %d\n",s->c_4_en);
        printk(" c_uv_en                : %d\n",s->c_uv_en);
        printk(" pri_16                 : %d\n",s->pri_16);
        printk(" pri_8                  : %d\n",s->pri_8);
        printk(" pri_4                  : %d\n",s->pri_4);
        printk(" pri_uv                 : %d\n",s->pri_uv);
        printk(" ref_neb_4              : %d\n",s->ref_neb_4);
        printk(" ref_neb_8              : %d\n",s->ref_neb_8);
        printk(" lambda_info16          : %d\n",s->lambda_info16);
        printk(" lambda_info8           : %d\n",s->lambda_info8);
        printk(" lambda_info4           : %d\n",s->lambda_info4);
        printk(" lambda_infouv          : %d\n",s->lambda_infouv);
        printk(" ref_4                  : %d\n",s->ref_4);
        printk(" ref_8                  : %d\n",s->ref_8);

        int i=0;
        for (i=0;i<4;i++)
            printk(" bit_16[%d]             : %d\n",i,s->bit_16[i]);
        for (i=0;i<4;i++)
            printk(" bit_uv[%d]             : %d\n",i,s->bit_uv[i]);
        for (i=0;i<4;i++)
            printk(" bit_4[%d]              : %d\n",i,s->bit_4[i]);
        for (i=0;i<4;i++)
            printk(" bit_8[%d]              : %d\n",i,s->bit_8[i]);
        for (i=0;i<4;i++)
            printk(" const_16[%d]           : %d\n",i,s->const_16[i]);
        for (i=0;i<4;i++)
            printk(" const_uv[%d]           : %d\n",i,s->const_uv[i]);
        for (i=0;i<4;i++)
            printk(" const_4[%d]            : %d\n",i,s->const_4[i]);
        for (i=0;i<4;i++)
            printk(" const_8[%d]            : %d\n",i,s->const_8[i]);
    }
    /* 9. jrfc,jrfd [5]*/
    {
        printk(" jrfcd_flag              : %d\n",s->jrfcd_flag);
        printk(" jrfc_enable             : %d\n",s->jrfc_enable);
        printk(" jrfd_enable             : %d\n",s->jrfd_enable);
        printk(" lm_head_total           : %d\n",s->lm_head_total);
        printk(" cm_head_total           : %d\n",s->cm_head_total);
    }
    /* 10. eigen cfg for mosaic, color error [42]*/
    {
        printk(" mb_mode_use             : %d\n",s->mb_mode_use);
        printk(" force_i16dc             : %d\n",s->force_i16dc);//ipred
        printk(" force_i16               : %d\n",s->force_i16);
        printk(" refresh_en              : %d\n",s->refresh_en);
        printk(" refresh_mode            : %d\n",s->refresh_mode);
        printk(" refresh_bias            : %d\n",s->refresh_bias);
        printk(" refresh_cplx_thd        : %d\n",s->refresh_cplx_thd);
        printk(" cplx_thd_sel            : %d\n",s->cplx_thd_sel);
        printk(" diff_cplx_sel           : %d\n",s->diff_cplx_sel);
        printk(" diff_thd_sel            : %d\n",s->diff_thd_sel);
        printk(" i16dc_cplx_thd          : %d\n",s->i16dc_cplx_thd);
        printk(" i16dc_qp_base           : %d\n",s->i16dc_qp_base);
        printk(" i16dc_qp_sel            : %d\n",s->i16dc_qp_sel);
        printk(" i16_qp_base             : %d\n",s->i16_qp_base);
        printk(" i16_qp_sel              : %d\n",s->i16_qp_sel);
        printk(" diff_cplx_thd           : %d\n",s->diff_cplx_thd);

        int i=0;
        for (i=0;i<2;i++)
            printk(" diff_qp_base[%d]        : %d\n",i,s->diff_qp_base[i]);
        for (i=0;i<2;i++)
            printk(" diff_qp_sel[%d]         : %d\n",i,s->diff_qp_sel[i]);
        for (i=0;i<24;i++)
            printk(" cplx_thd_idx[%d]        : %d\n",i,s->cplx_thd_idx[i]);
        for (i=0;i<8;i++)
            printk(" cplx_thd[%d]            : %d\n",i,s->cplx_thd[i]);
        for (i=0;i<3;i++)
            printk(" diff_thd_base[%d]       : %d\n",i,s->diff_thd_base[i]);
        for (i=0;i<3;i++)
            printk(" diff_thd_ofst[%d]       : %d\n",i,s->diff_thd_ofst[i]);
        printk(" sas_eigen_en            : %d\n",s->sas_eigen_en);
        printk(" crp_eigen_en            : %d\n",s->crp_eigen_en);
        printk(" sas_eigen_dump          : %d\n",s->sas_eigen_dump);
        printk(" crp_eigen_dump          : %d\n",s->crp_eigen_dump);
        printk(" rrs_en                  : %d\n",s->rrs_en);
        printk(" rrs_dump_en             : %d\n",s->rrs_dump_en);
        printk(" rrs_uv_en               : %d\n",s->rrs_uv_en);
        printk(" rrs_size_y              : %d\n",s->rrs_size_y);
        printk(" rrs_size_c              : %d\n",s->rrs_size_c);
        printk(" rrs_thrd_y              : %d\n",s->rrs_thrd_y);
        printk(" rrs_thrd_u              : %d\n",s->rrs_thrd_u);
        printk(" rrs_thrd_v              : %d\n",s->rrs_thrd_v);
    }
    /* 11. skin judge cfg [14]*/
    {
        printk(" skin_dt_en              : %d\n",s->skin_dt_en);
        printk(" skin_lvl                : %d\n",s->skin_lvl);
        printk(" skin_cnt_thd            : %d\n",s->skin_cnt_thd);
        printk(" ncu_mov_en              : %d\n",s->ncu_mov_en);
        printk(" ncu_move_len            : %d\n",s->ncu_move_len);
        if (s->ncu_move_info == NULL)
            printk(" ncu_move_info is NULL \n");
        else
            printk(" ncu_move_info       : %p\n",s->ncu_move_info);
        printk(" buf_share_en            : %d\n",s->buf_share_en);
        printk(" buf_share_size          : %d\n",s->buf_share_size);
        printk(" frame_idx               : %d\n",s->frame_idx);
        printk(" is_first_Pframe         : %d\n",s->is_first_Pframe);
        int i=0,j=0;
        for (i=0;i<3;i++)
            for (j=0;j<2;j++)
                printk(" skin_pxlu_thd[%d][%d] : %d\n",i,j,s->skin_pxlu_thd[i][j]);
        for (i=0;i<3;i++)
            for (j=0;j<2;j++)
                printk(" skin_pxlv_thd[%d][%d] : %d\n",i,j,s->skin_pxlv_thd[i][j]);
        for (i=0;i<4;i++)
            printk(" skin_qp_ofst[%d] : %d\n",i,s->skin_qp_ofst[i]);
        for (i=0;i<3;i++)
            printk(" mult_factor[%d] : %d\n",i,s->mult_factor[i]);
        for (i=0;i<3;i++)
            for (j=0;j<3;j++)
                printk(" shift_factor[%d][%d] : %d\n",i,j,s->shift_factor[i][j]);
        for (i=0;i<4;i++)
            printk(" skin_ofst[%d] : %d\n",i,s->skin_ofst[i]);
    }
    /* 12.qpg */
    {
        /* qp map */
        printk(" qp_tab_en               : %d\n",s->qp_tab_en);
        printk(" qp_tab_len              : %d\n",s->qp_tab_len);
        if (s->qp_tab == NULL)
            printk(" qp_tab is NULL");
        else
            printk(" qp_tab : %p\n", s->qp_tab);
        int i=0;
        for (i=0;i<8;i++) {
            printk("roi_en[%d]   : %d\n",i,s->roi_info[i].roi_en);
            printk("roi_md[%d]   : %d\n",i,s->roi_info[i].roi_md);
            printk("roi_qp[%d]   : %d\n",i,s->roi_info[i].roi_qp);
            printk("roi_lmbx[%d] : %d\n",i,s->roi_info[i].roi_lmbx);
            printk("roi_rmbx[%d] : %d\n",i,s->roi_info[i].roi_rmbx);
            printk("roi_umby[%d] : %d\n",i,s->roi_info[i].roi_umby);
            printk("roi_bmby[%d] : %d\n",i,s->roi_info[i].roi_bmby);
        }
        /* qp cplx */
        printk(" sas_en                  : %d\n",s->sas_en);
        printk(" crp_en                  : %d\n",s->crp_en);
        printk(" sas_mthd                : %d\n",s->sas_mthd);
        for (i=0;i<7;i++)
            printk(" qpg_mb_thd[%d] : %d\n", i,s->qpg_mb_thd[i]);
        for (i=0;i<8;i++)
            printk(" qpg_mbqp_ofst[%d] : %d\n", i,s->qpg_mbqp_ofst[i]);
        for (i=0;i<5;i++)
            printk(" qpg_flt_thd[%d] : %d\n", i,s->qpg_flt_thd[i]);
        printk(" mbrc_qpg_sel            : %d\n",s->mbrc_qpg_sel);
        /* bu rc */
        printk(" rc_mb_en                : %d\n",s->rc_mb_en);
        printk(" rc_bu_wait_en           : %d\n",s->rc_bu_wait_en);
        printk(" rc_mb_wait_en           : %d\n",s->rc_mb_wait_en);
        printk(" rc_bu_num               : %d\n",s->rc_bu_num);
        printk(" rc_bu_size              : %d\n",s->rc_bu_size);
        printk(" rc_bu_level             : %d\n",s->rc_bu_level);
        printk(" rc_mb_level             : %d\n",s->rc_mb_level);
        if (s->mb_ref_info == NULL)
            printk(" mb_ref_info is NULL");
        else
            printk(" mb_ref_info : %p\n", s->mb_ref_info);
        if (s->bu_ref_info == NULL)
            printk(" bu_ref_info is NULL");
        else
            printk(" bu_ref_info : %p\n", s->bu_ref_info);
        printk(" rc_frm_tbs              : %d\n",s->rc_frm_tbs);
        printk(" avg_bu_bs               : %d\n",s->avg_bu_bs);
        for (i=0;i<6;i++)
            printk(" tar_bs_thd[%d] : %d\n",s->tar_bs_thd[i]);
        for (i=0;i<6;i++)
            printk(" tar_bs_thd[%d] : %d\n",s->bu_alg0_qpo[i]);
        for (i=0;i<6;i++)
            printk(" bu_alg1_qpo[%d] : %d\n",s->bu_alg1_qpo[i]);
        for (i=0;i<7;i++)
            printk(" mb_cs_qpo[%d] : %d\n",s->mb_cs_qpo[i]);
        for (i=0;i<2;i++)
            printk(" mb_top_bs_qpo[%d] : %d\n",s->mb_top_bs_qpo[i]);
        for (i=0;i<2;i++)
            printk(" mb_rinfo_qpo[%d] : %d\n",s->mb_rinfo_qpo[i]);
        for (i=0;i<2;i++)
            printk(" mb_target_avg_bs[%d] : %d\n",s->mb_target_avg_bs[i]);
        printk(" mb_gp_num               : %d\n",s->mb_gp_num);
        printk(" last_bu_size            : %d\n",s->last_bu_size);
        printk(" rc_bcfg_mode            : %d\n",s->rc_bcfg_mode);
    }

}
#endif


static int h264e_fill_slice_info(struct h264e_ctx *ctx, unsigned int sh_size_in_bits)
{
	int i = 0;
	int ret = 0;
	unsigned char bmap[3][3] = {
		{0, 2, 1},
		{1, 0, 2},
		{2, 1, 0}
	};

	unsigned char *bufidx = bmap[ctx->i_frame % 3];
	H264E_SliceInfo_t *s = ctx->s;


	h264_sps_t *sps 	= &ctx->sps;
	h264_pps_t *pps 	= &ctx->pps;

	/* 1. rc output [11] */
	{
		s->mb_width 			= sps->i_mb_width;
		s->mb_height 			= sps->i_mb_height;

		s->frame_type 			= ctx->sh.i_type == M_SLICE_TYPE_I ? 0:1;
		s->first_mby 			= 0;
		s->last_mby 			= s->mb_height - 1;
		s->frame_width 			= ctx->p.width;
		s->frame_height 		= ctx->p.height;
		s->qp				= ctx->sh.i_type == M_SLICE_TYPE_I ? ctx->p.i_qp : ctx->p.p_qp;
		s->base_qp			= s->qp;
		s->max_qp			= ctx->p.h264_max_qp;
		s->min_qp			= ctx->p.h264_min_qp;


		s->bs_head_en 			= 1;	/*X2000 new add, fix value*/
		s->bs_head_va 			= (paddr_t *)ctx->slice_header;
		s->bs_head_len 			= (sh_size_in_bits + 7) / 8;
		s->bs_rbsp_en 			= 1;
	}

	/*2. motion cfg [25]*/
	{
		s->frm_re[0] 			= 0;/*sipe->frm_re0A;*/
		s->frm_re[1] 			= 0;/*sipe->frm_re1A;*/
		s->frm_re[2] 			= 0;/*sipe->frm_re2A;*/
		s->frm_re[3] 			= 0;/*sipe->frm_re3A;*/

		s->pskip_en               	= 1;
		s->mref_en                	= 0;
		s->scl                    	= 3;/*sipe->scl;*/
		s->hpel_en                	= 1;/*sipe->hpel_en;*/
		s->qpel_en                	= 1;/*sipe->qpel_en;*/
		s->ref_mode               	= 1;
		s->max_sech_step_i        	= 63;/*sipe->max_sech_step_i;*/
		s->max_mvrx_i             	= 255;/*sipe->max_mvrx_i;*/          //FixMe, for the buf_share_size is 3 when bufshare function opened
		s->max_mvry_i             	= 255;/*sipe->max_mvry_i;*/          //FixMe, for the buf_share_size is 3 when bufshare function opened
		s->lambda_scale_parameter 	= 8;

		if(s->mb_width > 50 && s->mb_height > 38) {
			s->fs_en   			= 1;/*sipe->fs_en;*/
			s->fs_md   			= 0;/*sipe->fs_md;*/
			s->fs_px   			= 3;/*sipe->fs_px;*/
			s->fs_py   			= 3;/*sipe->fs_py;*/
			s->fs_rx   			= 8;/*sipe->fs_rx;*/
			s->fs_ry   			= 8;/*sipe->fs_ry;*/
		} else {
			s->fs_en   			= 0;/*sipe->fs_en;*/
			s->fs_md   			= 0;/*sipe->fs_md;*/
			s->fs_px   			= 0;/*sipe->fs_px;*/
			s->fs_py   			= 0;/*sipe->fs_py;*/
			s->fs_rx   			= 0;/*sipe->fs_rx;*/
			s->fs_ry   			= 0;/*sipe->fs_ry;*/
		}

		s->frm_mv_en   			= 1;/*sipe->frm_mv_en;*/
		s->frm_mv_size 			= 3;/*sipe->frm_mv_size;*/
		s->glb_mv_en   			= 1;/*sipe->glb_mv_en;*/
#if 0
		if((frm->mvCnt & 0xffff) == 0 || ((frm->mvCnt >> 16) & 0xffff) == 0) {
			s->glb_mvx   		= 0;
			s->glb_mvy   		= 0;
		} else {
			s->glb_mvx   		= frm->mvxSum / (frm->mvCnt & 0xffff);
			s->glb_mvy   		= frm->mvySum / ((frm->mvCnt >> 16) & 0xffff);
		}
#endif
		s->me_step_en 			= 1; /*sipe->me_step_en;*/
		s->me_step_0  			= 8; /*sipe->me_step_0;*/
		s->me_step_1  			= 10;/*sipe->me_step_1;*/
	}

	/*3. quant cfg [4]*/
	{
		s->dct8x8_en = 0;
		memset(s->scaling_list, 0x10, sizeof(s->scaling_list));
		memcpy(s->scaling_list8[0], cqm_8iy_tab, sizeof(uint8_t) * 64);
		memcpy(s->scaling_list8[1], cqm_8py_tab, sizeof(uint8_t) * 64);

		s->deadzone[0] = 0x15;
		s->deadzone[1] = 0x0b;
		s->deadzone[2] = 0x15;
		s->deadzone[3] = 0x0b;
		s->deadzone[4] = 0x15;
		s->deadzone[5] = 0x0b;
		s->deadzone[6] = 0x15;
		s->deadzone[7] = 0x15;
		s->deadzone[8] = 0x0b;
	}

	/*4.loop filter cfg [4] */
	{
		s->deblock 			= ctx->p.deblock;
		s->rotate 			= 0;
		s->alpha_c0_offset 		= 0;
		s->beta_offset			= 0;
	}

	/*5.*/
	{
		s->acmask_mode             	= 0x3fffe; /*sipe->acmask_mode;*/
		s->intra_mode_msk          	= 0;
		s->i_4x4_dis               	= 0;
		s->i_8x8_dis               	= 0;
		s->i_16x16_dis             	= 0;
		s->p_l0_dis                	= 0;
		s->p_t8_dis                	= 0;
		s->p_skip_dis              	= 0;
		s->p_skip_pl0f_dis         	= 0;
		s->p_skip_pt8f_dis         	= 0;

		s->cost_bias_en            	= 0;
		s->cost_bias_i_4x4         	= 0;
		s->cost_bias_i_8x8         	= 0;
		s->cost_bias_i_16x16       	= 0;
		s->cost_bias_p_l0          	= 0;
		s->cost_bias_p_t8          	= 0;
		s->cost_bias_p_skip        	= 0;

		s->intra_lambda_y_bias_en  	= 0;
		s->intra_lambda_c_bias_en  	= 0;
		s->intra_lambda_bias_qp0   	= 0;
		s->intra_lambda_bias_qp1   	= 0;
		s->intra_lambda_bias_0     	= 0;
		s->intra_lambda_bias_1     	= 0;
		s->intra_lambda_bias_2     	= 0;

		s->chroma_sse_bias_en      	= 0;
		s->chroma_sse_bias_qp0     	= 0;
		s->chroma_sse_bias_qp1     	= 0;
		s->chroma_sse_bias_0       	= 0;
		s->chroma_sse_bias_1       	= 0;
		s->chroma_sse_bias_2       	= 0;

		s->sse_lambda_bias_en      	= 0;
		s->sse_lambda_bias         	= 0;
		s->fbc_ep                  	= 204;
		s->jm_lambda2_en           	= 0; /*sipe->jm_lambda2_en;*/
		s->inter_nei_en            	= 0; /*sipe->inter_nei_en;*/
		s->skip_bias_en            	= 0; /*sipe->skip_bias_en;*/
		s->ysse_thr                	= 0;
		s->csse_thr                	= 0;
		s->dcm_en                  	= 0; /*sipe->dcm_en;*/
		s->dcm_param               	= 0x4304; /*sipe->dcm_param;*/
		s->sde_prior               	= 5;
		s->db_prior                	= 5;

		s->use_intra_in_pframe     	= 1;/*sipe->use_intra_in_pframe;*/
		s->use_fast_mvp            	= 1;

		s->skip_en                 	= 1;
		s->cqp_offset              	= 0;/*sipe->cqp_offset;*/

		s->daisy_chain_en          	= 0;
		s->curr_thread_id          	= 0;
		s->qp_tab_mode             	= 0;

		s->bs_size_en              	= 1;
		s->bs_size                 	= 1024;
		s->raw_format              	= 8;
		s->size_mode               	= 0;
		s->step_mode               	= 0;
		s->mode_ctrl               	= 0;

		memset(s->mode_ctrl_param,  0, sizeof(s->mode_ctrl_param));

	}

	/* 6. select hardware output mode [11]*/
	{
		s->info_en      		= 1;
		s->mvd_sum_all  		= 0;
		s->mvd_sum_abs  		= 0;
		s->mv_sum_all   		= 1;
		s->mv_sum_abs   		= 1;

		s->cfg_size_x   		= 1;
		s->cfg_size_y   		= 1;
		s->cfg_iw_thr   		= 0;

		s->cfg_mvr_thr1 		= 0;
		s->cfg_mvr_thr2 		= 0;
		s->cfg_mvr_thr3 		= 0;
	}

	/* 7. ipred bit&lambda ctrl [33]*/
	{
		s->mb_mode_val   		= 1;
		s->bit_16_en     		= 0;
		s->bit_8_en      		= 1;
		s->bit_4_en      		= 1;
		s->bit_uv_en     		= 0;
		s->lamb_16_en    		= 0;
		s->lamb_8_en     		= 0;
		s->lamb_4_en     		= 0;
		s->lamb_uv_en    		= 0;
		s->c_16_en       		= 0;
		s->c_8_en        		= 0;
		s->c_4_en        		= 0;
		s->c_uv_en       		= 0;
		s->pri_16        		= 0;
		s->pri_8         		= 0;
		s->pri_4         		= 0;
		s->pri_uv        		= 0;
		s->ref_neb_4     		= 1;
		s->ref_neb_8     		= 1;
		s->lambda_info16 		= 0;
		s->lambda_info8  		= 0;
		s->lambda_info4  		= 0;
		s->lambda_infouv 		= 0;
		s->ref_4         		= 4;
		s->ref_8         		= 3;
		memset(s->bit_16,   0,  sizeof(s->bit_16));
		memset(s->bit_uv,   0,  sizeof(s->bit_uv));
		for (i = 0; i < 4; ++i)
			s->bit_4[i] = 4;
		for (i = 0; i < 4; ++i)
			s->bit_8[i] = 4;
		memset(s->const_16, 0,  sizeof(s->const_16));
		memset(s->const_uv, 0,  sizeof(s->const_uv));
		memset(s->const_4,  0,  sizeof(s->const_4));
		memset(s->const_8,  0,  sizeof(s->const_8));
	}


	/* 9. jrfc,jrfd [5]*/
	{
		s->jrfcd_flag = 0;
		s->jrfc_enable = 0;
		s->jrfd_enable = 0;
		s->lm_head_total = ((ctx->p.width+31) >> 5) * ((ctx->p.height+15) >> 4);
		s->cm_head_total = ((ctx->p.width+63) >> 6) * ((ctx->p.height+15) >> 4);
	}


	/* 10. eigen cfg for mosaic, color error [42]*/
	{
		//x264_sinfo_eigenValue_cfg(rc,p,s);
	}

	/*11. skin judge cfg[14]*/
	{
		/*TODO.*/
		s->skin_dt_en     = 0; //sipe->skin_dt_en;
		s->skin_lvl       = 0; //sipe->skin_lvl;
		s->skin_cnt_thd   = 20; //sipe->skin_cnt_thd;
		s->ncu_mov_en     = 0;
		s->ncu_move_len   = 20;
		s->ncu_move_info  = NULL;
		s->buf_share_en   = 0;
		s->buf_share_size = 0;
		s->frame_idx      = 0;
		//s->is_first_Pframe = 0;

		memcpy(s->skin_pxlu_thd, rc_skin_pxlu_thd, sizeof(s->skin_pxlu_thd));
		memcpy(s->skin_pxlv_thd, rc_skin_pxlv_thd, sizeof(s->skin_pxlv_thd));
		memcpy(s->skin_qp_ofst,  rc_skin_qp_ofst,  sizeof(s->skin_qp_ofst));
		memcpy(s->mult_factor,   rc_mult_factor,   sizeof(s->mult_factor));
		memcpy(s->shift_factor,  rc_shift_factor,  sizeof(s->shift_factor));
		memcpy(s->skin_ofst,     rc_skin_ofst,     sizeof(s->skin_ofst));
	}

	/* 12.qpg */
	{
		//h264_get_mb_qp(p,s);
	}

	s->state 		= ctx->cb.state;
	s->raw_format 		= ctx->p.format;

	s->stride[0] 		= s->frame_width;
	s->stride[1]		= s->raw_format == HELIX_420P_MODE ?
				  s->frame_width / 2 : s->frame_width;
	for(i = 0; i < ctx->frame->num_planes; i++) {
		s->raw[i] = ctx->frame->fb_addr[i].pa;
	}

	for(i = 0; i < 3; i++) {
		s->fb[i][0] = ctx->fb[bufidx[i]].yaddr_pa;
		s->fb[i][1] = ctx->fb[bufidx[i]].caddr_pa;
	}

	s->des_va 	= ctx->desc;
	s->des_pa 	= ctx->desc_pa;

	s->emc_bs_pa = ctx->bs->pa;

	s->emc_dblk_va 	= ctx->emc_buf;
	s->emc_recon_va = s->emc_dblk_va + DBLK_SIZE;
	s->emc_mv_va 	= s->emc_recon_va + RECON_SIZE;
	s->emc_se_va 	= s->emc_mv_va + MV_SIZE;
	s->emc_qpt_va 	= s->emc_se_va + SE_SIZE;
	s->emc_rc_va 	= s->emc_qpt_va + QPT_SIZE;
	s->emc_cpx_va 	= s->emc_rc_va + RC_SIZE;
	s->emc_mod_va 	= s->emc_cpx_va + CPX_SIZE;
	s->emc_sad_va 	= s->emc_mod_va + MOD_SIZE;
	s->emc_ncu_va 	= s->emc_sad_va + SAD_SIZE;

	s->emc_dblk_pa 	= ctx->emc_buf_pa;
	s->emc_recon_pa = s->emc_dblk_pa + DBLK_SIZE;
	s->emc_mv_pa 	= s->emc_recon_pa + RECON_SIZE;
	s->emc_se_pa 	= s->emc_mv_pa + MV_SIZE;
	s->emc_qpt_pa 	= s->emc_se_pa + SE_SIZE;
	s->emc_rc_pa 	= s->emc_qpt_pa + QPT_SIZE;
	s->emc_cpx_pa 	= s->emc_rc_pa + RC_SIZE;
	s->emc_mod_pa 	= s->emc_cpx_pa + CPX_SIZE;
	s->emc_sad_pa 	= s->emc_mod_pa + MOD_SIZE;
	s->emc_ncu_pa 	= s->emc_sad_pa + SAD_SIZE;


	H264E_SliceInit(s);
//	H264E_DumpInfo(s);

	dma_cache_sync(NULL, ctx->desc, ctx->vdma_chain_len, DMA_TO_DEVICE);

	return ret;
}

static int h264e_encode_slice(struct h264e_ctx *ctx, int force_idr, int *keyframe)
{
	int idr = 0;
	int i_nal_type = 0;
	int i_nal_ref_idc = 0;
	bs_t bs;
	int ret = 0;
	unsigned int encoded_header_size = 0;

	/*decide I/P Frame*/
	if(!(ctx->i_frame % ctx->p.gop_size) || force_idr) {
		ctx->i_frame = 0;
		idr = 1;
	}

	/* only I/P support.*/
	if(idr) {
		i_nal_type = NAL_SLICE_IDR;
		i_nal_ref_idc = NAL_PRIORITY_HIGHEST;
		ctx->sh.i_type = M_SLICE_TYPE_I;
		*keyframe = 1;
	} else {
		i_nal_type = NAL_SLICE;
		i_nal_ref_idc = NAL_PRIORITY_HIGH;
		ctx->sh.i_type = M_SLICE_TYPE_P;
		*keyframe = 0;
	}

	if(*keyframe && add_sps_pps_iframe) {
		/* insert sps/pps.*/
		encoded_header_size = h264e_encode_headers(ctx, ctx->bs);
		ctx->bs->va += encoded_header_size;
		ctx->bs->pa += encoded_header_size;
	}

	bs_init(&bs, ctx->slice_header, MAX_SLICE_HEADER_SIZE);

	ret = h264e_slice_init(ctx, i_nal_type);
	if(ret < 0)
		return ret;

	/*1. nal_header*/
	bs_write(&bs, 8, 0x00);
	bs_write(&bs, 8, 0xFF);
	bs_write(&bs, 8, 0x00);
	bs_write(&bs, 8, 0x01);
	/* nal header */
	bs_write(&bs, 8, (0 << 7 | (i_nal_ref_idc << 5) | (i_nal_type)));

	/*2. slice_header, */
	h264e_slice_header_write(&bs, &ctx->sh, i_nal_ref_idc);


	if(ctx->p.i_cabac) {
		bs_align_1(&bs);
		h264_cabac_context_init(&ctx->cb, ctx->sh.i_type, ctx->sh.i_qp_delta, ctx->sh.i_cabac_init_idc);
	}

	unsigned int slice_header_size = bs_pos(&bs);
	unsigned int slice_header_len = (slice_header_size + 7) / 8;
	h264e_fill_slice_info(ctx, slice_header_size);

	//print_hex_dump(KERN_INFO, "sl_header@", DUMP_PREFIX_ADDRESS, 16, 1, ctx->slice_header, 64, 1);

	ret = ingenic_vpu_start(ctx->priv);
	if(ret < 0) {
		return ret;
	}

	//print_hex_dump(KERN_INFO, "bs@", DUMP_PREFIX_ADDRESS, 16, 1, ctx->bs->va, ctx->r_bs_len, 1);

	ctx->encoded_bs_len = encoded_header_size + ctx->r_bs_len + ctx->r_rbsp_len + slice_header_len;

	unsigned char *xp = (unsigned char *)((unsigned long)ctx->bs->va | 0xa0000000);
	xp[0] = 0x00;
	xp[1] = 0x00;
	xp[2] = 0x00;

	//printk("-----------%s, %d, vpu ctx->r_bs_len: %d, ctx->r_rbsp_len: %d, ctx->encoded_bs_len: %d\n", __func__, __LINE__, ctx->r_bs_len, ctx->r_rbsp_len, ctx->encoded_bs_len);

	ctx->i_frame++;
	ctx->frameindex++;

	return ret;
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

	if(ctx->p.h264_hdr_mode == V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME) {

		/*align to 256*/
		align = (ctx->bs_header_size + 255) / 256 * 256;

		ret = h264e_padding(align - ctx->bs_header_size, bs->va + ctx->bs_header_size);
		ctx->encoded_bs_len = align;
	} else {
		ctx->encoded_bs_len = ctx->bs_header_size;
	}

	ret = ctx->encoded_bs_len;

	return ret;
}

int h264e_encode(struct h264e_ctx *ctx, struct video_frame_buffer *frame,
		struct ingenic_vcodec_mem *bs, int force_idr, int *keyframe)
{
	int ret = 0;
	ctx->frame = frame;
	ctx->bs = bs;

	ret = h264e_encode_slice(ctx, force_idr, keyframe);

	/* output bistream */

	return ret;
}



/* start streaming.*/
int h264e_alloc_workbuf(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	int allocate_fb = 0;
	int ret = 0;
	int i = 0;

	ctx->bs_header = kzalloc(MAX_BS_HEADER_SIZE, GFP_KERNEL);
	if(!ctx->bs_header) {
		pr_err("Failed to alloc memory for bs_header\n");
		return -ENOMEM;
	}
	ctx->bs_header_size = 0;


	ctx->desc = dma_alloc_noncoherent(venc_ctx->dev->dev, ctx->vdma_chain_len, &ctx->desc_pa, GFP_KERNEL);
	if(!ctx->desc) {
		pr_err("Failed to alloc vpu desc buffer!\n");
		ret = -ENOMEM;
		goto err_desc;
	}


	/* init yaddr. */
	for(i = 0; i < 3; i++) {
		ctx->fb[i].yaddr = NULL;
		ctx->fb[i].caddr = NULL;
	}

	for(i = 0; i < 3; i++) {
		ctx->fb[i].yaddr = dma_alloc_coherent(venc_ctx->dev->dev, ctx->framesize, &ctx->fb[i].yaddr_pa, GFP_KERNEL);
		ctx->fb[i].caddr = dma_alloc_coherent(venc_ctx->dev->dev, ctx->framesize/2, &ctx->fb[i].caddr_pa, GFP_KERNEL);

		allocate_fb = i;
		if(!ctx->fb[i].yaddr || !ctx->fb[i].caddr) {
			ret = -ENOMEM;
			goto err_fb;
		}
	}

	ctx->emc_buf = dma_alloc_coherent(venc_ctx->dev->dev, EMC_SIZE, &ctx->emc_buf_pa, GFP_KERNEL);
	if(!ctx->emc_buf) {
		pr_err("Failed to alloc vpu emc_buf!\n");
		ret = -ENOMEM;
		goto err_emc_buf;
	}

	return ret;

err_emc_buf:
err_fb:
	for(i = 0; i < 3; i++) {
		if(ctx->fb[i].yaddr) {
			dma_free_coherent(venc_ctx->dev->dev, ctx->framesize, ctx->fb[i].yaddr, ctx->fb[i].yaddr_pa);
		}
		if(ctx->fb[i].caddr) {
			dma_free_coherent(venc_ctx->dev->dev, ctx->framesize/2, ctx->fb[i].caddr, ctx->fb[i].caddr_pa);
		}

	}

	dma_free_noncoherent(venc_ctx->dev->dev, ctx->vdma_chain_len, ctx->desc, ctx->desc_pa);
err_desc:
	kfree(ctx->bs_header);
	ctx->bs_header = NULL;

	return ret;
}

/* stop streaming.*/
int h264e_free_workbuf(struct h264e_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	int i;

	if(ctx->bs_header) {
		kfree(ctx->bs_header);
		ctx->bs_header = NULL;
	}
	dma_free_noncoherent(venc_ctx->dev->dev, ctx->vdma_chain_len, ctx->desc, ctx->desc_pa);
	dma_free_coherent(venc_ctx->dev->dev, EMC_SIZE, ctx->emc_buf, ctx->emc_buf_pa);

	for(i = 0; i < 3; i++) {
		dma_free_coherent(venc_ctx->dev->dev, ctx->framesize, ctx->fb[i].yaddr, ctx->fb[i].yaddr_pa);
		dma_free_coherent(venc_ctx->dev->dev, ctx->framesize/2, ctx->fb[i].caddr, ctx->fb[i].caddr_pa);
	}
	return 0;
}

int h264e_encoder_init(struct h264e_ctx *ctx)
{
	struct h264e_params *p = NULL;
	H264E_SliceInfo_t *s = NULL;

	s = kzalloc(sizeof(H264E_SliceInfo_t), GFP_KERNEL);
	if(!s) {
		return -ENOMEM;
	}

	p = &ctx->p;
	ctx->s = s;

	ctx->vdma_chain_len 	= 40960 + 256;
	ctx->frameindex		= 0;
	ctx->i_frame		= 0;

	p->height		= 240;
	p->width		= 160;
	p->max_ref_pic		= 1;
	p->gop_size		= 50;
	p->i_idr_pic_id		= 0;
	p->i_global_qp 		= 30;
	p->h264_profile		= V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
	p->h264_level		= V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
	p->i_cabac		= 1;
	p->interlaced		= 0;
	p->h264_8x8_transform	= 0;
	p->h264_hdr_mode	= V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME;
	p->h264_min_qp		= 0;
	p->h264_max_qp		= 51;
	p->i_qp			= 30;
	p->p_qp			= 30;
	p->deblock		= 0;

	h264_cabac_init();

	return 0;
}

int h264e_encoder_deinit(struct h264e_ctx *ctx)
{
	if(!ctx)
		return 0;

	if(ctx->s)
		kfree(ctx->s);
	return 0;
}

void h264e_set_priv(struct h264e_ctx *ctx, void *data)
{
	ctx->priv = data;
}
