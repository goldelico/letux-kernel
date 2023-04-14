#include "bytestream.h"
#include "get_bits.h"
#include "golomb.h"
#include "h264.h"
#include "h264dec.h"
#include "h264_parse.h"
#include "h264_ps.h"

int ff_h264_pred_weight_table(GetBitContext *gb, const SPS *sps,
                              const int *ref_count, int slice_type_nos,
                              H264PredWeightTable *pwt,
                              int picture_structure, void *logctx)
{
    int list, i, j;
    int luma_def, chroma_def;

    pwt->use_weight             = 0;
    pwt->use_weight_chroma      = 0;

    pwt->luma_log2_weight_denom = get_ue_golomb(gb);
    if (pwt->luma_log2_weight_denom > 7U) {
        av_log(logctx, AV_LOG_ERROR, "luma_log2_weight_denom %d is out of range\n", pwt->luma_log2_weight_denom);
        pwt->luma_log2_weight_denom = 0;
    }
    luma_def = 1 << pwt->luma_log2_weight_denom;

    if (sps->chroma_format_idc) {
        pwt->chroma_log2_weight_denom = get_ue_golomb(gb);
        if (pwt->chroma_log2_weight_denom > 7U) {
            av_log(logctx, AV_LOG_ERROR, "chroma_log2_weight_denom %d is out of range\n", pwt->chroma_log2_weight_denom);
            pwt->chroma_log2_weight_denom = 0;
        }
        chroma_def = 1 << pwt->chroma_log2_weight_denom;
    }

    for (list = 0; list < 2; list++) {
        pwt->luma_weight_flag[list]   = 0;
        pwt->chroma_weight_flag[list] = 0;
        for (i = 0; i < ref_count[list]; i++) {
            int luma_weight_flag, chroma_weight_flag;

            luma_weight_flag = get_bits1(gb);
            if (luma_weight_flag) {
                pwt->luma_weight[i][list][0] = get_se_golomb(gb);
                pwt->luma_weight[i][list][1] = get_se_golomb(gb);
                if ((int8_t)pwt->luma_weight[i][list][0] != pwt->luma_weight[i][list][0] ||
                    (int8_t)pwt->luma_weight[i][list][1] != pwt->luma_weight[i][list][1])
                    goto out_range_weight;
                if (pwt->luma_weight[i][list][0] != luma_def ||
                    pwt->luma_weight[i][list][1] != 0) {
                    pwt->use_weight             = 1;
                    pwt->luma_weight_flag[list] = 1;
                }
            } else {
		if(luma_def > 127) {
			luma_def = 127;
		}
                pwt->luma_weight[i][list][0] = luma_def;
                pwt->luma_weight[i][list][1] = 0;
            }

            if (sps->chroma_format_idc) {
                chroma_weight_flag = get_bits1(gb);
                if (chroma_weight_flag) {
                    int j;
                    for (j = 0; j < 2; j++) {
                        pwt->chroma_weight[i][list][j][0] = get_se_golomb(gb);
                        pwt->chroma_weight[i][list][j][1] = get_se_golomb(gb);
                        if ((int8_t)pwt->chroma_weight[i][list][j][0] != pwt->chroma_weight[i][list][j][0] ||
                            (int8_t)pwt->chroma_weight[i][list][j][1] != pwt->chroma_weight[i][list][j][1]) {
                            pwt->chroma_weight[i][list][j][0] = chroma_def;
                            pwt->chroma_weight[i][list][j][1] = 0;
                            goto out_range_weight;
                        }
                        if (pwt->chroma_weight[i][list][j][0] != chroma_def ||
                            pwt->chroma_weight[i][list][j][1] != 0) {
                            pwt->use_weight_chroma        = 1;
                            pwt->chroma_weight_flag[list] = 1;
                        }
                    }
                } else {
                    int j;
                    for (j = 0; j < 2; j++) {
			if(chroma_def > 127) {
				chroma_def = 127;
			}
                        pwt->chroma_weight[i][list][j][0] = chroma_def;
                        pwt->chroma_weight[i][list][j][1] = 0;
                    }
                }
            }

            // for MBAFF
            if (picture_structure == PICT_FRAME) {
                pwt->luma_weight[16 + 2 * i][list][0] = pwt->luma_weight[16 + 2 * i + 1][list][0] = pwt->luma_weight[i][list][0];
                pwt->luma_weight[16 + 2 * i][list][1] = pwt->luma_weight[16 + 2 * i + 1][list][1] = pwt->luma_weight[i][list][1];
                if (sps->chroma_format_idc) {
                    for (j = 0; j < 2; j++) {
                        pwt->chroma_weight[16 + 2 * i][list][j][0] = pwt->chroma_weight[16 + 2 * i + 1][list][j][0] = pwt->chroma_weight[i][list][j][0];
                        pwt->chroma_weight[16 + 2 * i][list][j][1] = pwt->chroma_weight[16 + 2 * i + 1][list][j][1] = pwt->chroma_weight[i][list][j][1];
                    }
                }
            }
        }
        if (slice_type_nos != AV_PICTURE_TYPE_B)
            break;
    }
    pwt->use_weight = pwt->use_weight || pwt->use_weight_chroma;
    return 0;
out_range_weight:
    av_log(logctx, AV_LOG_WARNING, "Out of range weight");
    return AVERROR_INVALIDDATA;
}


int ff_h264_parse_ref_count(int *plist_count, int ref_count[2],
                            GetBitContext *gb, const PPS *pps,
                            int slice_type_nos, int picture_structure, void *logctx)
{
    int list_count;
    int num_ref_idx_active_override_flag;

    // set defaults, might be overridden a few lines later
    ref_count[0] = pps->ref_count[0];
    ref_count[1] = pps->ref_count[1];

    if (slice_type_nos != AV_PICTURE_TYPE_I) {
        unsigned max[2];
        max[0] = max[1] = picture_structure == PICT_FRAME ? 15 : 31;

        num_ref_idx_active_override_flag = get_bits1(gb);

        if (num_ref_idx_active_override_flag) {
            ref_count[0] = get_ue_golomb(gb) + 1;
            if (slice_type_nos == AV_PICTURE_TYPE_B) {
                ref_count[1] = get_ue_golomb(gb) + 1;
            } else
                // full range is spec-ok in this case, even for frames
                ref_count[1] = 1;
        }

        if (ref_count[0] - 1 > max[0] || ref_count[1] - 1 > max[1]) {
            av_log(logctx, AV_LOG_ERROR, "reference overflow %u > %u or %u > %u\n",
                   ref_count[0] - 1, max[0], ref_count[1] - 1, max[1]);
            ref_count[0] = ref_count[1] = 0;
            *plist_count = 0;
            goto fail;
        }

        if (slice_type_nos == AV_PICTURE_TYPE_B)
            list_count = 2;
        else
            list_count = 1;
    } else {
        list_count   = 0;
        ref_count[0] = ref_count[1] = 0;
    }

    *plist_count = list_count;

    return 0;
fail:
    *plist_count = 0;
    ref_count[0] = 0;
    ref_count[1] = 0;
    return AVERROR_INVALIDDATA;
}

int ff_h264_init_poc(int pic_field_poc[2], int *pic_poc,
                     const SPS *sps, H264POCContext *pc,
                     int picture_structure, int nal_ref_idc)
{
    const int max_frame_num = 1 << sps->log2_max_frame_num;
    int64_t field_poc[2];

    pc->frame_num_offset = pc->prev_frame_num_offset;
    if (pc->frame_num < pc->prev_frame_num)
        pc->frame_num_offset += max_frame_num;

    if (sps->poc_type == 0) {
        const int max_poc_lsb = 1 << sps->log2_max_poc_lsb;

        if (pc->poc_lsb < pc->prev_poc_lsb &&
            pc->prev_poc_lsb - pc->poc_lsb >= max_poc_lsb / 2)
            pc->poc_msb = pc->prev_poc_msb + max_poc_lsb;
        else if (pc->poc_lsb > pc->prev_poc_lsb &&
                 pc->prev_poc_lsb - pc->poc_lsb < -max_poc_lsb / 2)
            pc->poc_msb = pc->prev_poc_msb - max_poc_lsb;
        else
            pc->poc_msb = pc->prev_poc_msb;
        field_poc[0] =
        field_poc[1] = pc->poc_msb + pc->poc_lsb;
        if (picture_structure == PICT_FRAME)
            field_poc[1] += pc->delta_poc_bottom;
    } else if (sps->poc_type == 1) {
        int abs_frame_num;
        int64_t expected_delta_per_poc_cycle, expectedpoc;
        int i;

        if (sps->poc_cycle_length != 0)
            abs_frame_num = pc->frame_num_offset + pc->frame_num;
        else
            abs_frame_num = 0;

        if (nal_ref_idc == 0 && abs_frame_num > 0)
            abs_frame_num--;

        expected_delta_per_poc_cycle = 0;
        for (i = 0; i < sps->poc_cycle_length; i++)
            // FIXME integrate during sps parse
            expected_delta_per_poc_cycle += sps->offset_for_ref_frame[i];

        if (abs_frame_num > 0) {
            int poc_cycle_cnt          = (abs_frame_num - 1) / sps->poc_cycle_length;
            int frame_num_in_poc_cycle = (abs_frame_num - 1) % sps->poc_cycle_length;

            expectedpoc = poc_cycle_cnt * expected_delta_per_poc_cycle;
            for (i = 0; i <= frame_num_in_poc_cycle; i++)
                expectedpoc = expectedpoc + sps->offset_for_ref_frame[i];
        } else
            expectedpoc = 0;

        if (nal_ref_idc == 0)
            expectedpoc = expectedpoc + sps->offset_for_non_ref_pic;

        field_poc[0] = expectedpoc + pc->delta_poc[0];
        field_poc[1] = field_poc[0] + sps->offset_for_top_to_bottom_field;

        if (picture_structure == PICT_FRAME)
            field_poc[1] += pc->delta_poc[1];
    } else {
        int poc = 2 * (pc->frame_num_offset + pc->frame_num);

        if (!nal_ref_idc)
            poc--;

        field_poc[0] = poc;
        field_poc[1] = poc;
    }

    if (   field_poc[0] != (int)field_poc[0]
        || field_poc[1] != (int)field_poc[1])
        return AVERROR_INVALIDDATA;

    if (picture_structure != PICT_BOTTOM_FIELD)
        pic_field_poc[0] = field_poc[0];
    if (picture_structure != PICT_TOP_FIELD)
        pic_field_poc[1] = field_poc[1];
    *pic_poc = FFMIN(pic_field_poc[0], pic_field_poc[1]);

    return 0;
}

/**
 * Compute profile from profile_idc and constraint_set?_flags.
 *
 * @param sps SPS
 *
 * @return profile as defined by FF_PROFILE_H264_*
 */
int ff_h264_get_profile(const SPS *sps)
{
    int profile = sps->profile_idc;

    switch (sps->profile_idc) {
    case FF_PROFILE_H264_BASELINE:
        // constraint_set1_flag set to 1
        profile |= (sps->constraint_set_flags & 1 << 1) ? FF_PROFILE_H264_CONSTRAINED : 0;
        break;
    case FF_PROFILE_H264_HIGH_10:
    case FF_PROFILE_H264_HIGH_422:
    case FF_PROFILE_H264_HIGH_444_PREDICTIVE:
        // constraint_set3_flag set to 1
        profile |= (sps->constraint_set_flags & 1 << 3) ? FF_PROFILE_H264_INTRA : 0;
        break;
    }

    return profile;
}
