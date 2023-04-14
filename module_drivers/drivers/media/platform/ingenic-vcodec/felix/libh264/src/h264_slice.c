#include <linux/dma-mapping.h>
#include "avassert.h"
#include "avcodec.h"
#include "h264.h"
#include "h264dec.h"
#include "h264data.h"
#include "h264_ps.h"
#include "golomb.h"
#include "mathops.h"
#include "mpegutils.h"
#include "api/jzm_h264_dec.h"

void hexdump(unsigned char *buf, int len);


static void release_picture(H264Context *h, H264Picture *pic)
{
	if(!pic->f)
		return;

    	pic->f->data[0] = NULL;
    	pic->f->data[1] = NULL;
    	pic->f->linesize[0] = 0;
    	pic->f->linesize[1] = 0;
	pic->f = NULL;

	av_buffer_free(&h->devmem_ctx, pic->frm_info_ctrl);
	av_buffer_free(&h->devmem_ctx, pic->frm_info_mv);
	if(h->format == VPU_FORMAT_NV12 || h->format == VPU_FORMAT_NV21) {
		av_buffer_free(&h->devmem_ctx, pic->dec_result_y);
		av_buffer_free(&h->devmem_ctx, pic->dec_result_uv);
	}
}

void ff_h264_release_picture(H264Context *h, H264Picture *pic)
{
	release_picture(h, pic);
}

static void release_unused_pictures(H264Context *h, int remove_current)
{
    int i;

    /* release non reference frames */
    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        //if (!h->DPB[i].inuse && !h->DPB[i].reference &&
        if (h->DPB[i].f && !h->DPB[i].reference &&
            (remove_current || &h->DPB[i] != h->cur_pic_ptr)) {

	    release_picture(h, &h->DPB[i]);
            ff_h264_unref_picture(h, &h->DPB[i]);
        }
    }
}

static int alloc_picture(H264Context *h, H264Picture *pic)
{
    int i, ret = 0;
    AVFrame *f = NULL;
    unsigned int mb_array_size = 0;

    f = list_first_entry_or_null(&h->queued_list, AVFrame, queued_entry);
    if(!f) {
	av_log(h->avctx, AV_LOG_ERROR, "cannot get AVframe from queued_list.");
	return AVERROR(ENOMEM);
    }

    f->data[0] = f->buf[0]->buffer;
    f->data[1] = f->buf[1]->buffer;
    f->linesize[0] = f->buf[0]->size;
    f->linesize[1] = f->buf[1]->size;

    mb_array_size = h->mb_width * h->mb_height;

#define SDE_FMV_ADS	(1 << 8)
    pic->frm_info_ctrl = av_buffer_alloc(&h->devmem_ctx, mb_array_size * sizeof(unsigned int) * 2 + SDE_FMV_ADS);
    if(!pic->frm_info_ctrl){
	av_log(NULL, AV_LOG_ERROR, "Failed to alloc frm_info_ctrl!\n");
	ret = AVERROR(ENOMEM);
	goto err_frm_info_ctrl;
    }
    pic->frm_info_mv = av_buffer_alloc(&h->devmem_ctx, mb_array_size * sizeof(unsigned int) * 32 + SDE_FMV_ADS);
    if(!pic->frm_info_mv) {
	av_log(NULL, AV_LOG_ERROR, "aFailed to alloc frm_info_mv!\n");
	ret = AVERROR(ENOMEM);
	goto err_frm_info_mv;
    }


    if(h->format == VPU_FORMAT_NV12 || h->format == VPU_FORMAT_NV21) {
	pic->dec_result_y = av_buffer_alloc(&h->devmem_ctx, h->width * h->height);
	if(!pic->dec_result_y) {
		av_log(NULL, AV_LOG_ERROR, "aFailed to alloc dec_result_y!\n");
		ret = AVERROR(ENOMEM);
		goto err_dec_result_y;
	}
	pic->dec_result_uv = av_buffer_alloc(&h->devmem_ctx, h->width * h->height / 2);
	if(!pic->dec_result_uv) {
		av_log(NULL, AV_LOG_ERROR, "aFailed to alloc dec_result_uv!\n");
		ret = AVERROR(ENOMEM);
		goto err_dec_result_uv;
	}
    } else {
	/* reference to output buffer.*/
	pic->dec_result_y  = f->buf[0];
	pic->dec_result_uv = f->buf[1];
    }

    pic->f = f;

    list_del(&f->queued_entry);

    return 0;

err_dec_result_uv:
    av_buffer_free(&h->devmem_ctx, pic->dec_result_y);
err_dec_result_y:
    av_buffer_free(&h->devmem_ctx, pic->frm_info_mv);
err_frm_info_mv:
    av_buffer_free(&h->devmem_ctx, pic->frm_info_ctrl);
err_frm_info_ctrl:
fail:
    ff_h264_unref_picture(h, pic);
    return (ret < 0) ? ret : AVERROR(ENOMEM);
}

static int find_unused_picture(H264Context *h)
{
    int i;

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
	if(!h->DPB[i].f)
            return i;
    }

    return AVERROR_INVALIDDATA;
}


static int h264_slice_header_init(H264Context *h);


static int h264_frame_start(H264Context *h)
{
    H264Picture *pic;
    int i, ret;
    const int pixel_shift = h->pixel_shift;
    int c[4] = {
        1<<(h->ps.sps->bit_depth_luma-1),
        1<<(h->ps.sps->bit_depth_chroma-1),
        1<<(h->ps.sps->bit_depth_chroma-1),
        -1
    };

    release_unused_pictures(h, 1);
    h->cur_pic_ptr = NULL;


    i = find_unused_picture(h);
    if (i < 0) {
        av_log(h->avctx, AV_LOG_ERROR, "no frame buffer available\n");
        return i;
    }
    pic = &h->DPB[i];

    if ((ret = alloc_picture(h, pic)) < 0)
        return ret;

    pic->reference              = h->droppable ? 0 : h->picture_structure;
    pic->f->coded_picture_number = h->coded_picture_number++;
    pic->field_picture          = h->picture_structure != PICT_FRAME;
    pic->frame_num               = h->poc.frame_num;
    /*
     * Zero key_frame here; IDR markings per slice in frame or fields are ORed
     * in later.
     * See decode_nal_units().
     */
    pic->f->key_frame = 0;
    pic->mmco_reset  = 0;
    pic->recovered   = 0;
    pic->invalid_gap = 0;
#if 0
    pic->sei_recovery_frame_cnt = h->sei.recovery_point.recovery_frame_cnt;
#endif

    pic->f->pict_type = h->slice_ctx[0].slice_type;

    pic->f->crop_left   = h->crop_left;
    pic->f->crop_right  = h->crop_right;
    pic->f->crop_top    = h->crop_top;
    pic->f->crop_bottom = h->crop_bottom;

    h->cur_pic_ptr = pic;

    for (i = 0; i < h->nb_slice_ctx; i++) {
        h->slice_ctx[i].linesize   = h->cur_pic_ptr->f->linesize[0];
        h->slice_ctx[i].uvlinesize = h->cur_pic_ptr->f->linesize[1];
    }

    /* We mark the current picture as non-reference after allocating it, so
     * that if we break out due to an error it can be released automatically
     * in the next ff_mpv_frame_start().
     */
    h->cur_pic_ptr->reference = 0;

    h->cur_pic_ptr->field_poc[0] = h->cur_pic_ptr->field_poc[1] = INT_MAX;

    h->next_output_pic = NULL;

    h->postpone_filter = 0;

    h->mb_aff_frame = h->ps.sps->mb_aff && (h->picture_structure == PICT_FRAME);

#if 0
    if (h->sei.unregistered.x264_build >= 0)
        h->x264_build = h->sei.unregistered.x264_build;
#endif

    //assert(h->cur_pic_ptr->long_ref == 0);

    return 0;
}

/**
 * Initialize implicit_weight table.
 * @param field  0/1 initialize the weight for interlaced MBAFF
 *                -1 initializes the rest
 */
static void implicit_weight_table(const H264Context *h, H264SliceContext *sl, int field)
{
    int ref0, ref1, i, cur_poc, ref_start, ref_count0, ref_count1;

    for (i = 0; i < 2; i++) {
        sl->pwt.luma_weight_flag[i]   = 0;
        sl->pwt.chroma_weight_flag[i] = 0;
    }

    if (field < 0) {
        if (h->picture_structure == PICT_FRAME) {
            cur_poc = h->cur_pic_ptr->poc;
        } else {
            cur_poc = h->cur_pic_ptr->field_poc[h->picture_structure - 1];
        }
        if (sl->ref_count[0] == 1 && sl->ref_count[1] == 1 && !FRAME_MBAFF(h) &&
            sl->ref_list[0][0].poc + (int64_t)sl->ref_list[1][0].poc == 2LL * cur_poc) {
            sl->pwt.use_weight        = 0;
            sl->pwt.use_weight_chroma = 0;
            return;
        }
        ref_start  = 0;
        ref_count0 = sl->ref_count[0];
        ref_count1 = sl->ref_count[1];
    } else {
        cur_poc    = h->cur_pic_ptr->field_poc[field];
        ref_start  = 16;
        ref_count0 = 16 + 2 * sl->ref_count[0];
        ref_count1 = 16 + 2 * sl->ref_count[1];
    }

    sl->pwt.use_weight               = 2;
    sl->pwt.use_weight_chroma        = 2;
    sl->pwt.luma_log2_weight_denom   = 5;
    sl->pwt.chroma_log2_weight_denom = 5;

    for (ref0 = ref_start; ref0 < ref_count0; ref0++) {
        int64_t poc0 = sl->ref_list[0][ref0].poc;
        for (ref1 = ref_start; ref1 < ref_count1; ref1++) {
            int w = 32;
            if (!sl->ref_list[0][ref0].parent->long_ref && !sl->ref_list[1][ref1].parent->long_ref) {
                int poc1 = sl->ref_list[1][ref1].poc;
                int td   = av_clip_int8(poc1 - poc0);
                if (td) {
                    int tb = av_clip_int8(cur_poc - poc0);
                    int tx = (16384 + (FFABS(td) >> 1)) / td;
                    int dist_scale_factor = (tb * tx + 32) >> 8;
                    if (dist_scale_factor >= -64 && dist_scale_factor <= 128)
                        w = 64 - dist_scale_factor;
                }
            }
            if (field < 0) {
                sl->pwt.implicit_weight[ref0][ref1][0] =
                sl->pwt.implicit_weight[ref0][ref1][1] = w;
            } else {
                sl->pwt.implicit_weight[ref0][ref1][field] = w;
            }
        }
    }
}

/* export coded and cropped frame dimensions to AVCodecContext */
static int init_dimensions(H264Context *h)
{
    const SPS *sps = (const SPS*)h->ps.sps;
    int cr = sps->crop_right;
    int cl = sps->crop_left;
    int ct = sps->crop_top;
    int cb = sps->crop_bottom;
    int width  = h->width  - (cr + cl);
    int height = h->height - (ct + cb);
    av_assert0(sps->crop_right + sps->crop_left < (unsigned)h->width);
    av_assert0(sps->crop_top + sps->crop_bottom < (unsigned)h->height);

    /* handle container cropping */
    if (h->width_from_caller > 0 && h->height_from_caller > 0     &&
        !sps->crop_top && !sps->crop_left                         &&
        FFALIGN(h->width_from_caller,  16) == FFALIGN(width,  16) &&
        FFALIGN(h->height_from_caller, 16) == FFALIGN(height, 16) &&
        h->width_from_caller  <= width &&
        h->height_from_caller <= height) {
        width  = h->width_from_caller;
        height = h->height_from_caller;
        cl = 0;
        ct = 0;
        cr = h->width - width;
        cb = h->height - height;
    } else {
        h->width_from_caller  = 0;
        h->height_from_caller = 0;
    }

    h->avctx->coded_width  = h->width;
    h->avctx->coded_height = h->height;
    h->avctx->width        = width;
    h->avctx->height       = height;
    h->crop_right          = cr;
    h->crop_left           = cl;
    h->crop_top            = ct;
    h->crop_bottom         = cb;

    return 0;
}

static int h264_slice_header_init(H264Context *h)
{
    const SPS *sps = h->ps.sps;
    int i, ret;

    h->first_field           = 0;
    h->prev_interlaced_frame = 1;

    if (sps->bit_depth_luma < 8 || sps->bit_depth_luma > 14 ||
        sps->bit_depth_luma == 11 || sps->bit_depth_luma == 13
    ) {
        av_log(h->avctx, AV_LOG_ERROR, "Unsupported bit depth %d\n",
               sps->bit_depth_luma);
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }

    h->cur_bit_depth_luma         = sps->bit_depth_luma;
    h->cur_chroma_format_idc      = sps->chroma_format_idc;
    h->pixel_shift                = sps->bit_depth_luma > 8;
    h->chroma_format_idc          = sps->chroma_format_idc;
    h->bit_depth_luma             = sps->bit_depth_luma;


    h->context_initialized = 1;

    return 0;
fail:
    h->context_initialized = 0;
    return ret;
}


static int h264_init_ps(H264Context *h, const H264SliceContext *sl, int first_slice)
{
    const SPS *sps;
    int needs_reinit = 0, must_reinit, ret;

    if (first_slice) {
	h->ps.pps = &h->ps.pps_list[sl->pps_id];
    }

    if (h->ps.sps != &h->ps.sps_list[h->ps.pps->sps_id]) {
	h->ps.sps = &h->ps.sps_list[h->ps.pps->sps_id];

        if (h->mb_width  != h->ps.sps->mb_width ||
            h->mb_height != h->ps.sps->mb_height ||
            h->cur_bit_depth_luma    != h->ps.sps->bit_depth_luma ||
            h->cur_chroma_format_idc != h->ps.sps->chroma_format_idc
        )
            needs_reinit = 1;

        if (h->bit_depth_luma    != h->ps.sps->bit_depth_luma ||
            h->chroma_format_idc != h->ps.sps->chroma_format_idc)
            needs_reinit         = 1;
    }
    sps = h->ps.sps;

    must_reinit = (h->context_initialized &&
                    (   16*sps->mb_width != h->avctx->coded_width
                     || 16*sps->mb_height != h->avctx->coded_height
                     || h->cur_bit_depth_luma    != sps->bit_depth_luma
                     || h->cur_chroma_format_idc != sps->chroma_format_idc
                     || h->mb_width  != sps->mb_width
                     || h->mb_height != sps->mb_height
                    ));

    if (!h->setup_finished) {
        h->avctx->profile = ff_h264_get_profile(sps);
        h->avctx->level   = sps->level_idc;
        h->avctx->refs    = sps->ref_frame_count;

        h->mb_width  = sps->mb_width;
        h->mb_height = sps->mb_height;
        h->mb_num    = h->mb_width * h->mb_height;
        h->mb_stride = h->mb_width + 1;

        h->b_stride = h->mb_width * 4;

        h->chroma_y_shift = sps->chroma_format_idc <= 1; // 400 uses yuv420p

        h->width  = 16 * h->mb_width;
        h->height = 16 * h->mb_height;

        ret = init_dimensions(h);
        if (ret < 0)
            return ret;
    }

    if (!h->context_initialized || must_reinit || needs_reinit) {
        int flush_changes = h->context_initialized;
        h->context_initialized = 0;
        if (sl != h->slice_ctx) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "changing width %d -> %d / height %d -> %d on "
                   "slice %d\n",
                   h->width, h->avctx->coded_width,
                   h->height, h->avctx->coded_height,
                   h->current_slice + 1);
            return AVERROR_INVALIDDATA;
        }

        av_assert1(first_slice);

        if (flush_changes)
            ff_h264_flush_change(h);

        if ((ret = h264_slice_header_init(h)) < 0) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "h264_slice_header_init() failed\n");
            return ret;
        }
    }

    return 0;
}

static int h264_select_output_frame(H264Context *h)
{
    const SPS *sps = h->ps.sps;
    H264Picture *out = h->cur_pic_ptr;
    H264Picture *cur = h->cur_pic_ptr;
    int i, pics, out_of_order, out_idx;

    cur->mmco_reset = h->mmco_reset;
    h->mmco_reset = 0;

    if (sps->bitstream_restriction_flag
	/*|| h->avctx->strict_std_compliance >= FF_COMPLIANCE_STRICT*/) {
        h->avctx->has_b_frames = FFMAX(h->avctx->has_b_frames, sps->num_reorder_frames);
    }

    for (i = 0; 1; i++) {

        if(i == MAX_DELAYED_PIC_COUNT || cur->poc < h->last_pocs[i]){
            if(i)
                h->last_pocs[i-1] = cur->poc;
            break;
        } else if(i) {
            h->last_pocs[i-1]= h->last_pocs[i];
        }
    }

    out_of_order = MAX_DELAYED_PIC_COUNT - i;
    if(   cur->f->pict_type == AV_PICTURE_TYPE_B
       || (h->last_pocs[MAX_DELAYED_PIC_COUNT-2] > INT_MIN && h->last_pocs[MAX_DELAYED_PIC_COUNT-1] - (int64_t)h->last_pocs[MAX_DELAYED_PIC_COUNT-2] > 2))
        out_of_order = FFMAX(out_of_order, 1);
    if (out_of_order == MAX_DELAYED_PIC_COUNT) {
        av_log(h->avctx, AV_LOG_VERBOSE, "Invalid POC %d<%d\n", cur->poc, h->last_pocs[0]);
        for (i = 1; i < MAX_DELAYED_PIC_COUNT; i++)
            h->last_pocs[i] = INT_MIN;
        h->last_pocs[0] = cur->poc;
        cur->mmco_reset = 1;
    } else if(h->avctx->has_b_frames < out_of_order && !sps->bitstream_restriction_flag){
	int loglevel = AV_LOG_WARNING;
        av_log(h->avctx, loglevel, "Increasing reorder buffer to %d\n", out_of_order);
        h->avctx->has_b_frames = out_of_order;
    }

    pics = 0;
    while (h->delayed_pic[pics])
        pics++;

    av_assert0(pics <= MAX_DELAYED_PIC_COUNT);

    h->delayed_pic[pics++] = cur;
    if (cur->reference == 0)
        cur->reference = DELAYED_PIC_REF;

    out     = h->delayed_pic[0];
    out_idx = 0;
    for (i = 1; h->delayed_pic[i] &&
                !h->delayed_pic[i]->f->key_frame &&
                !h->delayed_pic[i]->mmco_reset;
         i++)
        if (h->delayed_pic[i]->poc < out->poc) {
            out     = h->delayed_pic[i];
            out_idx = i;
        }


    if (h->avctx->has_b_frames == 0 &&
        (h->delayed_pic[0]->f->key_frame || h->delayed_pic[0]->mmco_reset))
        h->next_outputed_poc = INT_MIN;
    out_of_order = out->poc < h->next_outputed_poc;

    if (out_of_order || pics > h->avctx->has_b_frames) {
        out->reference &= ~DELAYED_PIC_REF;
        for (i = out_idx; h->delayed_pic[i]; i++)
            h->delayed_pic[i] = h->delayed_pic[i + 1];
    }
    if (!out_of_order && pics > h->avctx->has_b_frames) {
        h->next_output_pic = out;
        if (out_idx == 0 && h->delayed_pic[0] && (h->delayed_pic[0]->f->key_frame || h->delayed_pic[0]->mmco_reset)) {
            h->next_outputed_poc = INT_MIN;
        } else
            h->next_outputed_poc = out->poc;

        if (out->recovered) {
            // We have reached an recovery point and all frames after it in
            // display order are "recovered".
            h->frame_recovered |= FRAME_RECOVERED_SEI;
        }
        out->recovered |= !!(h->frame_recovered & FRAME_RECOVERED_SEI);

        if (!out->recovered) {
                h->next_output_pic = NULL;
#if 0
            if (!(h->avctx->flags & AV_CODEC_FLAG_OUTPUT_CORRUPT) &&
                !(h->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL)) {
                h->next_output_pic = NULL;
            } else {
                out->f->flags |= AV_FRAME_FLAG_CORRUPT;
            }
#endif
        }
    } else {
        av_log(h->avctx, AV_LOG_DEBUG, "no picture %s\n", out_of_order ? "ooo" : "");
    }

    return 0;
}

/* This function is called right after decoding the slice header for a first
 * slice in a field (or a frame). It decides whether we are decoding a new frame
 * or a second field in a pair and does the necessary setup.
 */
static int h264_field_start(H264Context *h, const H264SliceContext *sl,
                            const H2645NAL *nal, int first_slice)
{
    int i;
    const SPS *sps;

    int last_pic_structure, last_pic_droppable, ret;

    ret = h264_init_ps(h, sl, first_slice);
    if (ret < 0)
        return ret;

    sps = h->ps.sps;

    if (sps && sps->bitstream_restriction_flag &&
        h->avctx->has_b_frames < sps->num_reorder_frames) {
        h->avctx->has_b_frames = sps->num_reorder_frames;
    }

    last_pic_droppable   = h->droppable;
    last_pic_structure   = h->picture_structure;
    h->droppable         = (nal->ref_idc == 0);
    h->picture_structure = sl->picture_structure;

    h->poc.frame_num        = sl->frame_num;
    h->poc.poc_lsb          = sl->poc_lsb;
    h->poc.delta_poc_bottom = sl->delta_poc_bottom;
    h->poc.delta_poc[0]     = sl->delta_poc[0];
    h->poc.delta_poc[1]     = sl->delta_poc[1];

    /* Shorten frame num gaps so we don't have to allocate reference
     * frames just to throw them away */
    if (h->poc.frame_num != h->poc.prev_frame_num) {
        int unwrap_prev_frame_num = h->poc.prev_frame_num;
        int max_frame_num         = 1 << sps->log2_max_frame_num;

        if (unwrap_prev_frame_num > h->poc.frame_num)
            unwrap_prev_frame_num -= max_frame_num;

        if ((h->poc.frame_num - unwrap_prev_frame_num) > sps->ref_frame_count) {
            unwrap_prev_frame_num = (h->poc.frame_num - sps->ref_frame_count) - 1;
            if (unwrap_prev_frame_num < 0)
                unwrap_prev_frame_num += max_frame_num;

            h->poc.prev_frame_num = unwrap_prev_frame_num;
        }
    }

    /* See if we have a decoded first field looking for a pair...
     * Here, we're using that to see if we should mark previously
     * decode frames as "finished".
     * We have to do that before the "dummy" in-between frame allocation,
     * since that can modify h->cur_pic_ptr. */
    if (h->first_field) {
        int last_field = last_pic_structure == PICT_BOTTOM_FIELD;
        av_assert0(h->cur_pic_ptr);
        av_assert0(h->cur_pic_ptr->f->buf[0]);
        //assert(h->cur_pic_ptr->reference != DELAYED_PIC_REF);

        /* figure out if we have a complementary field pair */
        if (!FIELD_PICTURE(h) || h->picture_structure == last_pic_structure) {
            /* Previous field is unmatched. Don't display it, but let it
             * remain for reference if marked as such. */
            if (last_pic_structure != PICT_FRAME) {
            }
        } else {
            if (h->cur_pic_ptr->frame_num != h->poc.frame_num) {
                /* This and previous field were reference, but had
                 * different frame_nums. Consider this field first in
                 * pair. Throw away previous field except for reference
                 * purposes. */
                if (last_pic_structure != PICT_FRAME) {
                }
            } else {
                /* Second field in complementary pair */
                if (!((last_pic_structure   == PICT_TOP_FIELD &&
                       h->picture_structure == PICT_BOTTOM_FIELD) ||
                      (last_pic_structure   == PICT_BOTTOM_FIELD &&
                       h->picture_structure == PICT_TOP_FIELD))) {
                    av_log(h->avctx, AV_LOG_ERROR,
                           "Invalid field mode combination %d/%d\n",
                           last_pic_structure, h->picture_structure);
                    h->picture_structure = last_pic_structure;
                    h->droppable         = last_pic_droppable;
                    return AVERROR_INVALIDDATA;
                } else if (last_pic_droppable != h->droppable) {
                    av_log(h->avctx, AV_LOG_ERROR,
                                          "Found reference and non-reference fields in the same frame, which");
                    h->picture_structure = last_pic_structure;
                    h->droppable         = last_pic_droppable;
                    return AVERROR_PATCHWELCOME;
                }
            }
        }
    }

    while (h->poc.frame_num != h->poc.prev_frame_num && !h->first_field &&
           h->poc.frame_num != (h->poc.prev_frame_num + 1) % (1 << sps->log2_max_frame_num)) {
        H264Picture *prev = h->short_ref_count ? h->short_ref[0] : NULL;
        av_log(h->avctx, AV_LOG_DEBUG, "Frame num gap %d %d\n",
               h->poc.frame_num, h->poc.prev_frame_num);
        if (!sps->gaps_in_frame_num_allowed_flag)
            for(i=0; i<FF_ARRAY_ELEMS(h->last_pocs); i++)
                h->last_pocs[i] = INT_MIN;

        ret = h264_frame_start(h);
        if (ret < 0) {
            h->first_field = 0;
            return ret;
        }

        h->poc.prev_frame_num++;
        h->poc.prev_frame_num        %= 1 << sps->log2_max_frame_num;
        h->cur_pic_ptr->frame_num = h->poc.prev_frame_num;
        h->cur_pic_ptr->invalid_gap = !sps->gaps_in_frame_num_allowed_flag;
        h->explicit_ref_marking = 0;
        ret = ff_h264_execute_ref_pic_marking(h);
        if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
            return ret;
    }

    /* See if we have a decoded first field looking for a pair...
     * We're using that to see whether to continue decoding in that
     * frame, or to allocate a new one. */
    if (h->first_field) {
        av_assert0(h->cur_pic_ptr);
        av_assert0(h->cur_pic_ptr->f->buf[0]);
        //assert(h->cur_pic_ptr->reference != DELAYED_PIC_REF);

        /* figure out if we have a complementary field pair */
        if (!FIELD_PICTURE(h) || h->picture_structure == last_pic_structure) {
            /* Previous field is unmatched. Don't display it, but let it
             * remain for reference if marked as such. */
            h->missing_fields ++;
            h->cur_pic_ptr = NULL;
            h->first_field = FIELD_PICTURE(h);
        } else {
            h->missing_fields = 0;
            if (h->cur_pic_ptr->frame_num != h->poc.frame_num) {
                /* This and the previous field had different frame_nums.
                 * Consider this field first in pair. Throw away previous
                 * one except for reference purposes. */
                h->first_field = 1;
                h->cur_pic_ptr = NULL;
            } else if (h->cur_pic_ptr->reference & DELAYED_PIC_REF) {
                /* This frame was already output, we cannot draw into it
                 * anymore.
                 */
                h->first_field = 1;
                h->cur_pic_ptr = NULL;
            } else {
                /* Second field in complementary pair */
                h->first_field = 0;
            }
        }
    } else {
        /* Frame or first field in a potentially complementary pair */
        h->first_field = FIELD_PICTURE(h);
    }

    if (!FIELD_PICTURE(h) || h->first_field) {
        if (h264_frame_start(h) < 0) {
            h->first_field = 0;
            return AVERROR_INVALIDDATA;
        }
    } else {
        int field = h->picture_structure == PICT_BOTTOM_FIELD;
        release_unused_pictures(h, 0);
    }


    ret = ff_h264_init_poc(h->cur_pic_ptr->field_poc, &h->cur_pic_ptr->poc,
                     h->ps.sps, &h->poc, h->picture_structure, nal->ref_idc);
    if (ret < 0)
        return ret;

    memcpy(h->mmco, sl->mmco, sl->nb_mmco * sizeof(*h->mmco));
    h->nb_mmco = sl->nb_mmco;
    h->explicit_ref_marking = sl->explicit_ref_marking;

    h->picture_idr = nal->type == H264_NAL_IDR_SLICE;

    h->cur_pic_ptr->f->key_frame |= (nal->type == H264_NAL_IDR_SLICE);

    if (nal->type == H264_NAL_IDR_SLICE ||
        (h->recovery_frame == h->poc.frame_num && nal->ref_idc)) {
        h->recovery_frame         = -1;
        h->cur_pic_ptr->recovered = 1;
    }
    // If we have an IDR, all frames after it in decoded order are
    // "recovered".
    if (nal->type == H264_NAL_IDR_SLICE)
        h->frame_recovered |= FRAME_RECOVERED_IDR;

    h->cur_pic_ptr->recovered |= h->frame_recovered;

    /* Set the frame properties/side data. Only done for the second field in
     * field coded frames, since some SEI information is present for each field
     * and is merged by the SEI parsing code. */
    if (!FIELD_PICTURE(h) || !h->first_field || h->missing_fields > 1) {

        ret = h264_select_output_frame(h);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int h264_slice_header_parse(const H264Context *h, H264SliceContext *sl,
                                   const H2645NAL *nal)
{
    const SPS *sps;
    const PPS *pps;
    int ret;
    unsigned int slice_type, tmp, i;
    int field_pic_flag, bottom_field_flag;
    int first_slice = sl == h->slice_ctx && !h->current_slice;
    int picture_structure;

    if (first_slice)
        av_assert0(!h->setup_finished);

    sl->first_mb_addr = get_ue_golomb_long(&sl->gb);

    slice_type = get_ue_golomb_31(&sl->gb);
    if (slice_type > 9) {
        av_log(h->avctx, AV_LOG_ERROR,
               "slice type %d too large at %d\n",
               slice_type, sl->first_mb_addr);
        return AVERROR_INVALIDDATA;
    }
    if (slice_type > 4) {
        slice_type -= 5;
        sl->slice_type_fixed = 1;
    } else
        sl->slice_type_fixed = 0;

    slice_type         = ff_h264_golomb_to_pict_type[slice_type];
    sl->slice_type     = slice_type;
    sl->slice_type_nos = slice_type & 3;

    if (nal->type  == H264_NAL_IDR_SLICE &&
        sl->slice_type_nos != AV_PICTURE_TYPE_I) {
        av_log(h->avctx, AV_LOG_ERROR, "A non-intra slice in an IDR NAL unit.\n");
        return AVERROR_INVALIDDATA;
    }

    sl->pps_id = get_ue_golomb(&sl->gb);
    if (sl->pps_id >= MAX_PPS_COUNT) {
        av_log(h->avctx, AV_LOG_ERROR, "pps_id %u out of range, max:%u\n", sl->pps_id, MAX_PPS_COUNT);
        return AVERROR_INVALIDDATA;
    }
    if (!&h->ps.pps_list[sl->pps_id]) {
        av_log(h->avctx, AV_LOG_ERROR,
               "non-existing PPS %u referenced\n",
               sl->pps_id);
        return AVERROR_INVALIDDATA;
    }
#if 0
    pps = (const PPS*)h->ps.pps_list[sl->pps_id]->data;
#else
    pps = &h->ps.pps_list[sl->pps_id];
#endif
    if (!&h->ps.sps_list[pps->sps_id]) {
        av_log(h->avctx, AV_LOG_ERROR,
               "non-existing SPS %u referenced\n", pps->sps_id);
        return AVERROR_INVALIDDATA;
    }
    sps = &h->ps.sps_list[pps->sps_id];

    sl->frame_num = get_bits(&sl->gb, sps->log2_max_frame_num);
    if (!first_slice) {
        if (h->poc.frame_num != sl->frame_num) {
            av_log(h->avctx, AV_LOG_ERROR, "Frame num change from %d to %d\n",
                   h->poc.frame_num, sl->frame_num);
            return AVERROR_INVALIDDATA;
        }
    }

    sl->mb_mbaff       = 0;

    if (sps->frame_mbs_only_flag) {
        picture_structure = PICT_FRAME;
    } else {
        if (!sps->direct_8x8_inference_flag && slice_type == AV_PICTURE_TYPE_B) {
            av_log(h->avctx, AV_LOG_ERROR, "This stream was generated by a broken encoder, invalid 8x8 inference\n");
            return -1;
        }
        field_pic_flag = get_bits1(&sl->gb);
        if (field_pic_flag) {
            bottom_field_flag = get_bits1(&sl->gb);
            picture_structure = PICT_TOP_FIELD + bottom_field_flag;
        } else {
            picture_structure = PICT_FRAME;
        }
    }
    sl->picture_structure      = picture_structure;
    sl->mb_field_decoding_flag = picture_structure != PICT_FRAME;

    if (picture_structure == PICT_FRAME) {
        sl->curr_pic_num = sl->frame_num;
        sl->max_pic_num  = 1 << sps->log2_max_frame_num;
    } else {
        sl->curr_pic_num = 2 * sl->frame_num + 1;
        sl->max_pic_num  = 1 << (sps->log2_max_frame_num + 1);
    }

    if (nal->type == H264_NAL_IDR_SLICE)
        get_ue_golomb_long(&sl->gb); /* idr_pic_id */

    if (sps->poc_type == 0) {
        sl->poc_lsb = get_bits(&sl->gb, sps->log2_max_poc_lsb);

        if (pps->pic_order_present == 1 && picture_structure == PICT_FRAME)
            sl->delta_poc_bottom = get_se_golomb(&sl->gb);
    }

    if (sps->poc_type == 1 && !sps->delta_pic_order_always_zero_flag) {
        sl->delta_poc[0] = get_se_golomb(&sl->gb);

        if (pps->pic_order_present == 1 && picture_structure == PICT_FRAME)
            sl->delta_poc[1] = get_se_golomb(&sl->gb);
    }

    sl->redundant_pic_count = 0;
    if (pps->redundant_pic_cnt_present)
        sl->redundant_pic_count = get_ue_golomb(&sl->gb);

    if (sl->slice_type_nos == AV_PICTURE_TYPE_B)
        sl->direct_spatial_mv_pred = get_bits1(&sl->gb);

    ret = ff_h264_parse_ref_count(&sl->list_count, sl->ref_count,
                                  &sl->gb, pps, sl->slice_type_nos,
                                  picture_structure, h->avctx);
    if (ret < 0)
        return ret;

    if (sl->slice_type_nos != AV_PICTURE_TYPE_I) {
       ret = ff_h264_decode_ref_pic_list_reordering(sl, h->avctx);
       if (ret < 0) {
           sl->ref_count[1] = sl->ref_count[0] = 0;
           return ret;
       }
    }

    sl->pwt.use_weight = 0;
    for (i = 0; i < 2; i++) {
        sl->pwt.luma_weight_flag[i]   = 0;
        sl->pwt.chroma_weight_flag[i] = 0;
    }
    if ((pps->weighted_pred && sl->slice_type_nos == AV_PICTURE_TYPE_P) ||
        (pps->weighted_bipred_idc == 1 &&
         sl->slice_type_nos == AV_PICTURE_TYPE_B)) {
        ret = ff_h264_pred_weight_table(&sl->gb, sps, sl->ref_count,
                                  sl->slice_type_nos, &sl->pwt,
                                  picture_structure, h->avctx);
        if (ret < 0)
            return ret;
    }

    sl->explicit_ref_marking = 0;
    if (nal->ref_idc) {
        ret = ff_h264_decode_ref_pic_marking(sl, &sl->gb, nal, h->avctx);
        if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
            return AVERROR_INVALIDDATA;
    }

    if (sl->slice_type_nos != AV_PICTURE_TYPE_I && pps->cabac) {
        tmp = get_ue_golomb_31(&sl->gb);
        if (tmp > 2) {
            av_log(h->avctx, AV_LOG_ERROR, "cabac_init_idc %u overflow\n", tmp);
            return AVERROR_INVALIDDATA;
        }
        sl->cabac_init_idc = tmp;
    }

    sl->last_qscale_diff = 0;
    tmp = pps->init_qp + (unsigned)get_se_golomb(&sl->gb);
    if (tmp > 51 + 6 * (sps->bit_depth_luma - 8)) {
        av_log(h->avctx, AV_LOG_ERROR, "QP %u out of range\n", tmp);
        return AVERROR_INVALIDDATA;
    }
    sl->qscale       = tmp;
    sl->chroma_qp[0] = get_chroma_qp(pps, 0, sl->qscale);
    sl->chroma_qp[1] = get_chroma_qp(pps, 1, sl->qscale);
    // FIXME qscale / qp ... stuff
    if (sl->slice_type == AV_PICTURE_TYPE_SP)
        get_bits1(&sl->gb); /* sp_for_switch_flag */
    if (sl->slice_type == AV_PICTURE_TYPE_SP ||
        sl->slice_type == AV_PICTURE_TYPE_SI)
        get_se_golomb(&sl->gb); /* slice_qs_delta */

    sl->deblocking_filter     = 1;
    sl->slice_alpha_c0_offset = 0;
    sl->slice_beta_offset     = 0;
    if (pps->deblocking_filter_parameters_present) {
        tmp = get_ue_golomb_31(&sl->gb);
        if (tmp > 2) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "deblocking_filter_idc %u out of range\n", tmp);
            return AVERROR_INVALIDDATA;
        }
        sl->deblocking_filter = tmp;
        if (sl->deblocking_filter < 2)
            sl->deblocking_filter ^= 1;  // 1<->0

        if (sl->deblocking_filter) {
            int slice_alpha_c0_offset_div2 = get_se_golomb(&sl->gb);
            int slice_beta_offset_div2     = get_se_golomb(&sl->gb);
            if (slice_alpha_c0_offset_div2 >  6 ||
                slice_alpha_c0_offset_div2 < -6 ||
                slice_beta_offset_div2 >  6     ||
                slice_beta_offset_div2 < -6) {
                av_log(h->avctx, AV_LOG_ERROR,
                       "deblocking filter parameters %d %d out of range\n",
                       slice_alpha_c0_offset_div2, slice_beta_offset_div2);
                return AVERROR_INVALIDDATA;
            }
            sl->slice_alpha_c0_offset = slice_alpha_c0_offset_div2 * 2;
            sl->slice_beta_offset     = slice_beta_offset_div2 * 2;
        }
    }

    return 0;
}

/* do all the per-slice initialization needed before we can start decoding the
 * actual MBs */
static int h264_slice_init(H264Context *h, H264SliceContext *sl,
                           const H2645NAL *nal)
{
    int i, j, ret = 0;

    if (h->picture_idr && nal->type != H264_NAL_IDR_SLICE) {
        av_log(h->avctx, AV_LOG_ERROR, "Invalid mix of IDR and non-IDR slices\n");
        return AVERROR_INVALIDDATA;
    }

    av_assert1(h->mb_num == h->mb_width * h->mb_height);
    if (sl->first_mb_addr << FIELD_OR_MBAFF_PICTURE(h) >= h->mb_num ||
        sl->first_mb_addr >= h->mb_num) {
        av_log(h->avctx, AV_LOG_ERROR, "first_mb_in_slice overflow\n");
        return AVERROR_INVALIDDATA;
    }

    sl->mb_x =  sl->first_mb_addr % h->mb_width;
    sl->mb_y = (sl->first_mb_addr / h->mb_width) <<
                                 FIELD_OR_MBAFF_PICTURE(h);
    av_assert1(sl->mb_y < h->mb_height);

    ret = ff_h264_build_ref_list(h, sl);
    if (ret < 0)
        return ret;

    if (h->ps.pps->weighted_bipred_idc == 2 &&
        sl->slice_type_nos == AV_PICTURE_TYPE_B) {
        implicit_weight_table(h, sl, -1);
        if (FRAME_MBAFF(h)) {
            implicit_weight_table(h, sl, 0);
            implicit_weight_table(h, sl, 1);
        }
    }

    if (sl->slice_type_nos == AV_PICTURE_TYPE_B && !sl->direct_spatial_mv_pred)
        ff_h264_direct_dist_scale_factor(h, sl);
    if (!h->setup_finished)
        ff_h264_direct_ref_list_init(h, sl);
    sl->qp_thresh = 15 -
                   FFMIN(sl->slice_alpha_c0_offset, sl->slice_beta_offset) -
                   FFMAX3(0,
                          h->ps.pps->chroma_qp_index_offset[0],
                          h->ps.pps->chroma_qp_index_offset[1]) +
                   6 * (h->ps.sps->bit_depth_luma - 8);

    sl->slice_num       = ++h->current_slice;

    return 0;
}

void dump_slice_header(H264SliceContext *sl)
{
        printf("sl->slice_num: %d\n", sl->slice_num);
        printf("sl->slice_type: %d\n", sl->slice_type);
        printf("sl->slice_type_nos: %d\n", sl->slice_type_nos);

        printf("sl->mb_x: %d\n", sl->mb_x);
        printf("sl->mb_y: %d\n", sl->mb_y);
	printf("sl->ref_count[0]: %d, sl->ref_count[1]:%d\n", sl->ref_count[0], sl->ref_count[1]);
        printf("sl->mb_mbaff: %d\n", sl->mb_mbaff);
        printf("sl->frame_num: %d\n", sl->frame_num);
        printf("sl->curr_pic_num: %d\n", sl->curr_pic_num);
        printf("sl->delta_poc_bottom: %d\n", sl->delta_poc_bottom);
}


#define MAX_SLICE_HEADER_SIZE 32
int ff_h264_queue_decode_slice(H264Context *h, const H2645NAL *nal)
{
    H264SliceContext *sl = h->slice_ctx + h->nb_slice_ctx_queued;
    int first_slice = sl == h->slice_ctx && !h->current_slice;
    int ret;
    unsigned char slice_header[MAX_SLICE_HEADER_SIZE];
    unsigned char removed_at[10];
    GetBitContext sh_gb;

    sl->gb = nal->gb;

    /*1. copy 256 raw data.*/
    memcpy(slice_header, nal->gb.buffer, 32);


//    hexdump(slice_header, 32);

    /*2. remove 0x3 from slice_header */
    char *src;
    char *dst;
    unsigned int si, di;
    int removed_bytes = 0;
    unsigned int length = 0;
    int i = 0;

    unsigned int slice_data_index = 0;

    ret = init_get_bits(&sh_gb, slice_header, MAX_SLICE_HEADER_SIZE * 8);
    sh_gb.index += 8;

    /* Try to remove 0x3 in slice header. */
    si = di = 0;
    src = dst = slice_header;
    length = MAX_SLICE_HEADER_SIZE;
    while (si + 2 < length) {
	    // remove escapes (very rare 1:2^22)
	    if (src[si + 2] > 3) {
		    dst[di++] = src[si++];
		    dst[di++] = src[si++];
	    } else if (src[si] == 0 && src[si + 1] == 0 && src[si + 2] == 3) {
		    dst[di++] = 0;
		    dst[di++] = 0;
		    si       += 3;
		    removed_at[i++] = (si-1) * 8;
	    }
	    dst[di++] = src[si++];
    }

    while (si < length)
	    dst[di++] = src[si++];
    removed_bytes = si - di;

    sl->gb = sh_gb;
    ret = h264_slice_header_parse(h, sl, nal);
    if (ret < 0)
        return ret;

    slice_data_index = sl->gb.index;

#if 0
    for(i = 0; i < removed_bytes; i++) {
    	if(removed_at[i] < sl->gb.index) {
		slice_data_index += 8;
	}
    }
#endif

    sl->gb = nal->gb;
    sl->gb.index = slice_data_index;

    //dump_slice_header(sl);

    // discard redundant pictures
    if (sl->redundant_pic_count > 0) {
        sl->ref_count[0] = sl->ref_count[1] = 0;
        return 0;
    }

    /*如果第一个macroblock地址为0, ？？？？*/
    if (sl->first_mb_addr == 0 || !h->current_slice) {
        if (h->setup_finished) {
            av_log(h->avctx, AV_LOG_ERROR, "Too many fields\n");
            return AVERROR_INVALIDDATA;
        }
    }

    if (!h->current_slice)
        av_assert0(sl == h->slice_ctx);

    if (!first_slice) {
        const PPS *pps = &h->ps.pps_list[sl->pps_id];

        if (h->ps.pps->sps_id != pps->sps_id ||
            h->ps.pps->transform_8x8_mode != pps->transform_8x8_mode /*||
            (h->setup_finished && h->ps.pps != pps)*/) {
            av_log(h->avctx, AV_LOG_ERROR, "PPS changed between slices\n");
            return AVERROR_INVALIDDATA;
        }
        if (h->ps.sps != &h->ps.sps_list[h->ps.pps->sps_id]) {
            av_log(h->avctx, AV_LOG_ERROR,
               "SPS changed in the middle of the frame\n");
            return AVERROR_INVALIDDATA;
        }
    }

    if (h->current_slice == 0) {
        ret = h264_field_start(h, sl, nal, first_slice);
        if (ret < 0)
            return ret;
    } else {
        if (h->picture_structure != sl->picture_structure ||
            h->droppable         != (nal->ref_idc == 0)) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "Changing field mode (%d -> %d) between slices is not allowed\n",
                   h->picture_structure, sl->picture_structure);
            return AVERROR_INVALIDDATA;
        } else if (!h->cur_pic_ptr) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "unset cur_pic_ptr on slice %d\n",
                   h->current_slice + 1);
            return AVERROR_INVALIDDATA;
        }
    }

    ret = h264_slice_init(h, sl, nal);
    if (ret < 0)
        return ret;

    h->nb_slice_ctx_queued++;

    return 0;
}

int ff_h264_get_slice_type(const H264SliceContext *sl)
{
    switch (sl->slice_type) {
    case AV_PICTURE_TYPE_P:
        return 0;
    case AV_PICTURE_TYPE_B:
        return 1;
    case AV_PICTURE_TYPE_I:
        return 2;
    case AV_PICTURE_TYPE_SP:
        return 3;
    case AV_PICTURE_TYPE_SI:
        return 4;
    default:
        return AVERROR_INVALIDDATA;
    }
}

void dump_st_h264(struct JZM_H264 *st_h264)
{
	printf("st_h264->des_va			     :0x%x\n", st_h264->des_va			);
	printf("st_h264->slice_type                  :0x%x\n", st_h264->slice_type              );
	printf("st_h264->slice_num                   :0x%x\n", st_h264->slice_num               );
	printf("st_h264->start_mb_x                  :0x%x\n", st_h264->start_mb_x              );
	printf("st_h264->start_mb_y                  :0x%x\n", st_h264->start_mb_y              );
	printf("st_h264->mb_width                    :0x%x\n", st_h264->mb_width                );
	printf("st_h264->mb_height                   :0x%x\n", st_h264->mb_height               );
	printf("st_h264->field_picture               :0x%x\n", st_h264->field_picture           );
	printf("st_h264->cabac                       :0x%x\n", st_h264->cabac                   );
	printf("st_h264->qscale                      :0x%x\n", st_h264->qscale                  );
	printf("st_h264->transform_8x8_mode          :0x%x\n", st_h264->transform_8x8_mode      );
	printf("st_h264->constrained_intra_pred      :0x%x\n", st_h264->constrained_intra_pred  );
	printf("st_h264->direct_8x8_inference_flag   :0x%x\n", st_h264->direct_8x8_inference_flag);
	printf("st_h264->direct_spatial_mv_pred      :0x%x\n", st_h264->direct_spatial_mv_pred  );
	printf("st_h264->x264_build                  :0x%x\n", st_h264->x264_build              );
	printf("st_h264->ref_count_0                 :0x%x\n", st_h264->ref_count_0             );
	printf("st_h264->ref_count_1                 :0x%x\n", st_h264->ref_count_1             );
	printf("st_h264->deblocking_filter           :0x%x\n", st_h264->deblocking_filter       );
	printf("st_h264->slice_alpha_c0_offset       :0x%x\n", st_h264->slice_alpha_c0_offset   );
	printf("st_h264->slice_beta_offset           :0x%x\n", st_h264->slice_beta_offset       );
	printf("st_h264->bs_buffer                   :0x%x\n", st_h264->bs_buffer               );
	printf("st_h264->bs_index                    :0x%x\n", st_h264->bs_index                );
	printf("st_h264->bs_size_in_bits             :0x%x\n", st_h264->bs_size_in_bits         );
	printf("st_h264->cabac_init_idc              :0x%x\n", st_h264->cabac_init_idc          );
	printf("st_h264->ref_frm_ctrl                :0x%x\n", st_h264->ref_frm_ctrl            );
	printf("st_h264->ref_frm_mv                  :0x%x\n", st_h264->ref_frm_mv              );
	printf("st_h264->curr_frm_ctrl               :0x%x\n", st_h264->curr_frm_ctrl           );
	printf("st_h264->curr_frm_mv                 :0x%x\n", st_h264->curr_frm_mv             );
	printf("st_h264->use_weight                  :0x%x\n", st_h264->use_weight              );
	printf("st_h264->use_weight_chroma           :0x%x\n", st_h264->use_weight_chroma       );
	printf("st_h264->use_weight_chroma           :0x%x\n", st_h264->use_weight_chroma       );
	printf("st_h264->luma_log2_weight_denom      :0x%x\n", st_h264->luma_log2_weight_denom  );
	printf("st_h264->chroma_log2_weight_denom    :0x%x\n", st_h264->chroma_log2_weight_denom);
	printf("st_h264->dblk_left_en                :0x%x\n", st_h264->dblk_left_en            );
	printf("st_h264->dblk_top_en                 :0x%x\n", st_h264->dblk_top_en             );
	printf("st_h264->dec_result_y                :0x%x\n", st_h264->dec_result_y            );
	printf("st_h264->dec_result_uv               :0x%x\n", st_h264->dec_result_uv           );
	printf("st_h264->new_odma_flag		     :0x%x\n", st_h264->new_odma_flag		);
	printf("st_h264->new_odma_format	     :0x%x\n", st_h264->new_odma_format		);
	printf("st_h264->dec_result1_y		     :0x%x\n", st_h264->dec_result1_y		);
	printf("st_h264->dec_result1_uv		     :0x%x\n", st_h264->dec_result1_uv		);
	printf("st_h264->frm_y_stride		     :0x%x\n", st_h264->frm_y_stride		);
	printf("st_h264->frm_c_stride		     :0x%x\n", st_h264->frm_c_stride		);
	printf("st_h264->bs_rbsp_en		     :0x%x\n", st_h264->bs_rbsp_en		);

	int i;
	printf("st_h264->dir_scale_table:    \n--------------------\n");
	for (i=0; i<16/4; i++) {
		printf("%x   %x   %x   %x\n", st_h264->dir_scale_table[i], st_h264->dir_scale_table[i+1], st_h264->dir_scale_table[i+2], st_h264->dir_scale_table[i+3]);
	}
	printf("--------------------------------------------------\n");

#if 1

	printf("st_h264->mc_ref_y[0]: \n");
	for(i = 0; i < 16; i++) {
		printf("%x ", st_h264->mc_ref_y[0][i]);

	}
	printf("\n");
	printf("st_h264->mc_ref_c[0]: \n");
	for(i = 0; i < 16; i++) {
		printf("%x ", st_h264->mc_ref_c[0][i]);

	}
	printf("\n");
	printf("st_h264->mc_ref_y[1]: \n");
	for(i = 0; i < 16; i++) {
		printf("%x ", st_h264->mc_ref_y[1][i]);

	}
	printf("\n");
	printf("st_h264->mc_ref_c[1]: \n");
	for(i = 0; i < 16; i++) {
		printf("%x ", st_h264->mc_ref_c[1][i]);

	}
	printf("\n");

	printf("st_h264->chroma_qp_table:---------------\n");
	for (i=0; i<128/16; i++) {
		int j;
		for(j = i * 16; j < i * 16 + 16; j++) {
			printf("%08x  ", st_h264->chroma_qp_table[j]);
		}
		printf("\n");
	}

	printf("##luma_weight:\n");
	for(i = 0; i<2; i++) {
		int j;
		for(j = 0; j< 16; j++) {
			printf("%08d  ", st_h264->luma_weight[i][j]);
		}
		printf("\n");
	}
	printf("luma_offset:\n");
	for(i = 0; i<2; i++) {
		int j;
		for(j = 0; j< 16; j++) {
			printf("%08d  ", st_h264->luma_offset[i][j]);
		}
		printf("\n");
	}

	printf("##chroma_weight:\n");
	for(i = 0; i<2; i++) {
		int j;
		for(j = 0; j< 16; j++) {
			int k;
			if(!(j%8))
				printf("\n");
			for(k = 0 ; k < 2; k ++)
				printf("%08d ", st_h264->chroma_weight[i][j][k]);


		}
		printf("\n");
	}
	printf("chroma_offset:\n");
	for(i = 0; i<2; i++) {
		int j;
		for(j = 0; j< 16; j++) {
			int k;
			if(!(j%8))
				printf("\n");
			for(k = 0; k < 2; k++) {
				printf("%08d ", st_h264->chroma_offset[i][j][k]);
			}
		}
		printf("\n");
	}

	printf("### implicit_weight !\n");
	for(i = 0; i<16; i++) {
		int j;
		for(j = 0; j < 16; j++) {
			printf("%08d ",st_h264->implicit_weight[i][j]);
		}
		printf("\n");
	}

	printf("dump matrix 4x4:\n");
	for(i=0; i< 6; i++) {
		int j;
		for(j = 0; j < 16; j++) {
			printf("%08d ", st_h264->scaling_matrix4[i][j]);
		}
		printf("\n");
	}

	printf("dump matrix 8x8:\n");
	for(i = 0; i <2; i++) {
		int j;
		for(j=0; j<64; j++) {
			if(!(j%16))
				printf("%08d ", st_h264->scaling_matrix8[i][j]);
		}
		printf("\n");
	}
#endif


}

void hexdump(unsigned char *buf, int len)
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

void dump_bs_buffer(struct JZM_H264 *st_h264, char *bs, unsigned long len)
{
	int i;
	printf("dump bs buffer- bs_index :%d, %s\n", st_h264->bs_index, st_h264->bs_index % 8 ? "Not 8Bits align": "8Bits align");
	printf("----\n");
	int bs_index = st_h264->bs_index;
	unsigned char *bs_buffer = bs;
	/* dump 32bytes more after bs_index */
	for(i = 0; i < bs_index/8 + 32; i++) {
		printf("%02x", bs_buffer[i]);
		if(bs_index > i*8) {
			printf(". ");
		} else if(bs_index == i*8) {
			printf("=>");
		} else {
			printf(" ");
		}
	}
	printf("\n----\n");

	printf("bs @ start:\n");
	hexdump(bs, 64);
	printf("bs @ end:\n");
	hexdump(bs + len - 64, 64);
}

void dump_picture(H264Picture *pic)
{
	AVFrame *f = pic->f;
	printf("######## dump H264Picture #########\n");

	printf("dump_picture f:%p\n", f);
	printf("f->buf[0]: %p\n", f->buf[0]);
	printf("f->buf[1]: %p\n", f->buf[1]);
	printf("f->buf[0]->buffer_pa: %lx\n", f->buf[0]->buffer_pa);
	printf("f->buf[1]->buffer_pa: %lx\n", f->buf[1]->buffer_pa);
	printf("pic->frame_num: %d\n", pic->frame_num);

}

#if 0
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif


static int decode_slice(struct AVCodecContext *avctx, void *arg)
{

	/*我怎么知道要解码这一帧图片，不是对于B帧的解码要延后处理吗??? */
    H264SliceContext *sl = arg;
    const H264Context *h = sl->h264;
    H264Picture *cur_pic = h->cur_pic_ptr;
    jzm_h264 *st_h264 = h->st_h264;
    int lf_x_start = sl->mb_x;
    int orig_deblock = sl->deblocking_filter;
    int ret;
    int i;
    unsigned int offset_y, offset_uv;

    for(i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
	    if(h->cur_pic_ptr == &h->DPB[i])
		    break;
    }


#if 0
    printf("use: %d in DPB\n", i);
    dump_picture(h->cur_pic_ptr);
    printf("---decode_slice----\n");
    printf("ref_count[0]: %d\n", sl->ref_count[0]);
    printf("ref_count[1]: %d\n", sl->ref_count[1]);
    printf("list_count: %d\n", sl->list_count);
    for(i = 0; i < sl->ref_count[0]; i++) {
    	printf("H264Ref[i] : %x\n", &sl->ref_list[0][i]);
    }
#endif

    if(h->width > h->coded_width || h->height > h->coded_height) {
	printf("App allocated buffer is too small, which is illegal.\n");
	printf("BS parsed width:%d, height:%d, while allocated width: %d, height: %d\n", h->width, h->height, h->coded_width, h->coded_height);
	return AVERROR(EINVAL);
    }

    st_h264->slice_type = (sl->slice_type==AV_PICTURE_TYPE_B) ? JZM_H264_B_TYPE	\
			  :(sl->slice_type==AV_PICTURE_TYPE_P) ? JZM_H264_P_TYPE \
			  :JZM_H264_I_TYPE;

    st_h264->slice_num                   = sl->slice_num-1;
    st_h264->start_mb_x                  = sl->mb_x;
    st_h264->start_mb_y                  = sl->mb_y;
    st_h264->mb_width                    = h->mb_width;
    st_h264->mb_height                   = h->mb_height;
    st_h264->field_picture               = sl->picture_structure != PICT_FRAME;
    st_h264->cabac                       = h->ps.pps->cabac;
    st_h264->qscale                      = sl->qscale;
    st_h264->transform_8x8_mode          = !!h->ps.pps->transform_8x8_mode;
    st_h264->constrained_intra_pred      = !!h->ps.pps->constrained_intra_pred;
    st_h264->direct_8x8_inference_flag   = !!h->ps.sps->direct_8x8_inference_flag;
    st_h264->direct_spatial_mv_pred      = !!sl->direct_spatial_mv_pred;
    st_h264->x264_build                  = 0xffff;
    st_h264->ref_count_0                 = sl->ref_count[0] - 1;
    st_h264->ref_count_1                 = sl->ref_count[1] - 1;
    st_h264->deblocking_filter           = !!sl->deblocking_filter;
    st_h264->slice_alpha_c0_offset       = sl->slice_alpha_c0_offset;
    st_h264->slice_beta_offset           = sl->slice_beta_offset;

	/*sl->gb.buffer 转物理地址. */
    //st_h264->bs_buffer			 = h->pkt.rbsp_buf->buffer_pa + sl->gb.buffer - h->pkt.rbsp_buf->buffer;
    st_h264->bs_buffer			 = (unsigned int)sl->gb.buffer & 0x1fffffff;
    st_h264->bs_index                    = sl->gb.index;
    st_h264->bs_size_in_bits             = sl->gb.size_in_bits;

#if 1	/*在M200上需要处理以下bs_addr，否则会出bs_error.
	原来media部门提供的api在api内部会做处理，现在需要在外面作处理了.
	*/
    unsigned int bs_addr;
    unsigned int bs_ofst;
    if (st_h264->cabac) {
	    int n = (-st_h264->bs_index) & 7;
	    if(n) st_h264->bs_index += n;
	    bs_addr = st_h264->bs_buffer + (st_h264->bs_index >> 3);
	    bs_ofst = (bs_addr & 0x3) << 3;
	    bs_addr = bs_addr & (~0x3);
	    st_h264->bs_rbsp_en 		 = 1;
    } else {
	    bs_addr = (unsigned int)(st_h264->bs_buffer + (st_h264->bs_index >> 3)) & (~0x3);
	    bs_ofst = ((((unsigned int)st_h264->bs_buffer & 0x3) << 3) + ((unsigned int)st_h264->bs_index & 0x1F)) & 0x1F;
	    unsigned int slice_bs_size = st_h264->bs_size_in_bits - st_h264->bs_index + bs_ofst;
	    //st_h264->bs_size_in_bits = slice_bs_size;

	    char *src, *dst;
	    unsigned long length;
	    int si, di;
	    int removed_bytes = 0;
	    si = di = 0;

	    src = dst = sl->gb.buffer;
	    length = (sl->gb.size_in_bits + 7) / 8;

	    while (si + 2 < length) {
		    // remove escapes (very rare 1:2^22)
		    if (src[si + 2] > 3) {
			    dst[di++] = src[si++];
			    dst[di++] = src[si++];
		    } else if (src[si] == 0 && src[si + 1] == 0 && src[si + 2] == 3) {
				    dst[di++] = 0;
				    dst[di++] = 0;
				    si       += 3;
		    }
		    dst[di++] = src[si++];
	    }

	    while (si < length)
		    dst[di++] = src[si++];

	    removed_bytes = si - di;

	    st_h264->bs_size_in_bits = slice_bs_size - 8 * removed_bytes;
	    dma_sync_single_for_device(h->devmem_ctx.dev, (dma_addr_t)(virt_to_phys((void *)dst)), di, DMA_TO_DEVICE);

	    st_h264->bs_rbsp_en 		 = 0;
    }

    st_h264->bs_buffer = bs_addr;	/*　slice_header之后的slice_data*/
    st_h264->bs_index  = bs_ofst;

#endif

    st_h264->cabac_init_idc              = sl->cabac_init_idc;
    if(sl->ref_list[1][0].parent) {
	st_h264->ref_frm_ctrl                = sl->ref_list[1][0].parent->frm_info_ctrl->buffer_pa;
    }
    if(sl->ref_list[1][0].parent) {
	st_h264->ref_frm_mv                  = sl->ref_list[1][0].parent->frm_info_mv->buffer_pa;
    }
    st_h264->curr_frm_ctrl               = cur_pic->frm_info_ctrl->buffer_pa;
    st_h264->curr_frm_mv                 = cur_pic->frm_info_mv->buffer_pa;

#if 0
    /*新版的api没有这两个变量.*/
    st_h264->curr_frm_slice_start_mb     = cur_pic->frm_info_slice_start_mb;
    st_h264->ref_frm_slice_start_mb      = sl->ref_list[1][0].frm_info_slice_start_mb;
#endif
    st_h264->use_weight                  = sl->pwt.use_weight;
    st_h264->use_weight_chroma           = sl->pwt.use_weight_chroma;
    st_h264->luma_log2_weight_denom      = sl->pwt.luma_log2_weight_denom;
    st_h264->chroma_log2_weight_denom    = sl->pwt.chroma_log2_weight_denom;
    st_h264->dblk_left_en                = /*h->s.mb_x!=*/0;
    st_h264->dblk_top_en                 = /*h->s.mb_y!=*/0;

#define ROUND_UP(x, align) (((int) (x) + (align - 1)) & ~(align - 1))

    if(h->format == VPU_FORMAT_NV12 || h->format == VPU_FORMAT_NV21) {
	st_h264->new_odma_flag = 1;	/* enable new output dma */
	st_h264->new_odma_format = h->format == VPU_FORMAT_NV12 ? 0 : 1;	/*0: NV12, 1 : NV21*/
	//st_h264->dec_result1_y = cur_pic->f->buf[0]->buffer_pa;
	//st_h264->dec_result1_uv = cur_pic->f->buf[1]->buffer_pa;
	st_h264->frm_y_stride = ROUND_UP(h->width, 128);
	st_h264->frm_c_stride = st_h264->frm_y_stride;

	offset_y = st_h264->start_mb_y * 16 * st_h264->frm_y_stride + st_h264->start_mb_x * 16;
	offset_uv = st_h264->start_mb_y * 8 * st_h264->frm_y_stride + st_h264->start_mb_x * 16;
	st_h264->dec_result1_y = cur_pic->f->buf[0]->buffer_pa + offset_y;
	st_h264->dec_result1_uv = cur_pic->f->buf[1]->buffer_pa + offset_uv;

    } else {
	st_h264->new_odma_flag = 0;
    }

    /* see alloc_picture()*/
    st_h264->dec_result_y = cur_pic->dec_result_y->buffer_pa;
    st_h264->dec_result_uv = cur_pic->dec_result_uv->buffer_pa;

    for (i=0; i<16; i++) {
	    st_h264->dir_scale_table[i] = sl->map_col_to_list0[0][i] + (sl->map_col_to_list0[1][i] << 5) + (sl->dist_scale_factor[i] << 16);
    }

    for (i=0; i<128; i++) {

	    unsigned int qp_c0 = h->ps.pps->chroma_qp_table[0][i] & 0xFF;
	    unsigned int qp_c1 = h->ps.pps->chroma_qp_table[1][i] & 0xFF;
	    unsigned int qp_c0_d6 = qp_c0/6;
	    unsigned int qp_c0_m6 = qp_c0%6;
	    unsigned int qp_c1_d6 = qp_c1/6;
	    unsigned int qp_c1_m6 = qp_c1%6;
	    st_h264->chroma_qp_table[i] = ((((((qp_c1_d6 & 0xF) + ((qp_c1_m6 & 0x7) << 4)) << 7) +
					    (((qp_c0_d6 & 0xF) + ((qp_c0_m6 & 0x7) << 4)) << 0) ) << 16) +
			    ((qp_c1 & 0x3F) << 6) +
			    ((qp_c0 & 0x3F) << 0)
			    );
    }


    for (i=0; i<16; i++) {
	    H264Picture *parent;
	    parent = sl->ref_list[0][i].parent;

	    st_h264->mc_ref_y[0][i] = parent ? parent->dec_result_y->buffer_pa: 0;
	    st_h264->mc_ref_c[0][i] = parent ? parent->dec_result_uv->buffer_pa : 0;

	    parent = sl->ref_list[1][i].parent;
	    st_h264->mc_ref_y[1][i] = parent? parent->dec_result_y->buffer_pa : 0;
	    st_h264->mc_ref_c[1][i] = parent? parent->dec_result_uv->buffer_pa : 0;

	    int j;
	    for (j=0; j<2; j++) {
		    st_h264->luma_weight[j][i]      = sl->pwt.luma_weight[i][j][0];
		    st_h264->chroma_weight[j][i][0] = sl->pwt.chroma_weight[i][j][0][0];
		    st_h264->chroma_weight[j][i][1] = sl->pwt.chroma_weight[i][j][1][0];
		    st_h264->luma_offset[j][i]      = sl->pwt.luma_weight[i][j][1];
		    st_h264->chroma_offset[j][i][0] = sl->pwt.chroma_weight[i][j][0][1];
		    st_h264->chroma_offset[j][i][1] = sl->pwt.chroma_weight[i][j][1][1];
	    }
	    for (j=0; j<16; j++) {
		    st_h264->implicit_weight[i][j] = sl->pwt.implicit_weight[i][j][0];
	    }
    }

    memcpy(st_h264->scaling_matrix4, h->ps.pps->scaling_matrix4, sizeof(h->ps.pps->scaling_matrix4));
    memcpy(st_h264->scaling_matrix8, h->ps.pps->scaling_matrix8, sizeof(h->ps.pps->scaling_matrix8));

    st_h264->des_va = (int *)h->dma_desc->buffer;

#if 0
    dump_st_h264(h->st_h264);
    dump_bs_buffer(h->st_h264, st_h264->bs_buffer | 0x80000000, (st_h264->bs_size_in_bits + 7) / 8);
#endif

    jzm_h264_slice_init_vdma(st_h264);


    //memset(st_h264->dec_result_y | 0x80000000, 0, 256);
    //memset(st_h264->dec_result_uv | 0x80000000, 0, 256);

    av_buffer_sync(&h->devmem_ctx, h->dma_desc, DMA_TO_DEVICE);
    //av_buffer_sync(&h->devmem_ctx, h->pkt.rbsp_buf, DMA_TO_DEVICE);

    ret = vpu_hw_start(&h->vpu_ctx);
    if(ret < 0) {
	    printf("h->mb_width: %d, h->mb_height: %d, current_decode mb_y: %d, mb_x: %d\n",
		h->mb_width, h->mb_height, (h->vpu_ctx.sde_stat >> 24) & 0xff, (h->vpu_ctx.sde_stat >> 16) & 0xff);
	    //return AVERROR(EINVAL);
    }

    vpu_hw_wait(&h->vpu_ctx);

    if(h->vpu_ctx.error) {
	    sl->mb_y = h->mb_height;
    } else {

	    sl->mb_y = (h->vpu_ctx.sde_stat >> 24) & 0xff;
    }
//    printf("----------decode_slice done ----------------\n");

    return ret;
}

/**
 * Call decode_slice() for each context.
 *
 * @param h h264 master context
 */
int ff_h264_execute_decode_slices(H264Context *h)
{
    AVCodecContext *const avctx = h->avctx;
    H264SliceContext *sl;
    int context_count = h->nb_slice_ctx_queued;
    int ret = 0;
    int i, j;


    h->slice_ctx[0].next_slice_idx = INT_MAX;

    if (/*h->avctx->hwaccel || */context_count < 1)
        return 0;

    av_assert0(context_count && h->slice_ctx[context_count - 1].mb_y < h->mb_height);

    /* context_count 是不是可以理解为由多少个线程or ???*/
    if (context_count == 1) {

        h->slice_ctx[0].next_slice_idx = h->mb_width * h->mb_height;
        h->postpone_filter = 0;

	/* 能够到这里说明已经确切的知道要解码的SLICE.*/
        ret = decode_slice(avctx, &h->slice_ctx[0]);
	/*设置当前解码到了哪一个mb*/
        h->mb_y = h->slice_ctx[0].mb_y;
        if (ret < 0) {
            h->next_output_pic = NULL;
            goto finish;
	}
    } else {
	av_log(NULL, AV_LOG_ERROR, "too many context in decode queue!\n");
	ret = -EINVAL;
    }

finish:
    h->nb_slice_ctx_queued = 0;
    return ret;
}
