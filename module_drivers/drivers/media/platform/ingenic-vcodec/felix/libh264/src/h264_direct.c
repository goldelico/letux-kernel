#include "avcodec.h"
#include "h264dec.h"
#include "h264_ps.h"

static int get_scale_factor(H264SliceContext *sl,
                            int poc, int poc1, int i)
{
    int poc0 = sl->ref_list[0][i].poc;
    int64_t pocdiff = poc1 - (int64_t)poc0;
    int td = av_clip_int8(pocdiff);

    if (pocdiff != (int)pocdiff)
        av_log(sl->h264->avctx, AV_LOG_WARNING, "pocdiff overflow\n");

    if (td == 0 || sl->ref_list[0][i].parent->long_ref) {
        return 256;
    } else {
        int64_t pocdiff0 = poc - (int64_t)poc0;
        int tb = av_clip_int8(pocdiff0);
        int tx = (16384 + (FFABS(td) >> 1)) / td;

        if (pocdiff0 != (int)pocdiff0)
            av_log(sl->h264->avctx, AV_LOG_DEBUG, "pocdiff0 overflow\n");

        return av_clip_intp2((tb * tx + 32) >> 6, 10);
    }
}

void ff_h264_direct_dist_scale_factor(const H264Context *const h,
                                      H264SliceContext *sl)
{
    const int poc  = FIELD_PICTURE(h) ? h->cur_pic_ptr->field_poc[h->picture_structure == PICT_BOTTOM_FIELD]
                                      : h->cur_pic_ptr->poc;
    const int poc1 = sl->ref_list[1][0].poc;
    int i, field;

    if (FRAME_MBAFF(h))
        for (field = 0; field < 2; field++) {
            const int poc  = h->cur_pic_ptr->field_poc[field];
            const int poc1 = sl->ref_list[1][0].parent->field_poc[field];
            for (i = 0; i < 2 * sl->ref_count[0]; i++)
                sl->dist_scale_factor_field[field][i ^ field] =
                    get_scale_factor(sl, poc, poc1, i + 16);
        }

    for (i = 0; i < sl->ref_count[0]; i++)
        sl->dist_scale_factor[i] = get_scale_factor(sl, poc, poc1, i);
}

static void fill_colmap(const H264Context *h, H264SliceContext *sl,
                        int map[2][16 + 32], int list,
                        int field, int colfield, int mbafi)
{
    H264Picture *const ref1 = sl->ref_list[1][0].parent;
    int j, old_ref, rfield;
    int start  = mbafi ? 16                       : 0;
    int end    = mbafi ? 16 + 2 * sl->ref_count[0] : sl->ref_count[0];
    int interl = mbafi || h->picture_structure != PICT_FRAME;

    /* bogus; fills in for missing frames */
    memset(map[list], 0, sizeof(map[list]));

    for (rfield = 0; rfield < 2; rfield++) {
        for (old_ref = 0; old_ref < ref1->ref_count[colfield][list]; old_ref++) {
            int poc = ref1->ref_poc[colfield][list][old_ref];

            if (!interl)
                poc |= 3;
            // FIXME: store all MBAFF references so this is not needed
            else if (interl && (poc & 3) == 3)
                poc = (poc & ~3) + rfield + 1;

            for (j = start; j < end; j++) {
                if (4 * sl->ref_list[0][j].parent->frame_num +
                    (sl->ref_list[0][j].reference & 3) == poc) {
                    int cur_ref = mbafi ? (j - 16) ^ field : j;
                    if (ref1->mbaff)
                        map[list][2 * old_ref + (rfield ^ field) + 16] = cur_ref;
                    if (rfield == field || !interl)
                        map[list][old_ref] = cur_ref;
                    break;
                }
            }
        }
    }
}

void ff_h264_direct_ref_list_init(const H264Context *const h, H264SliceContext *sl)
{
    H264Ref *const ref1 = &sl->ref_list[1][0];
    H264Picture *const cur = h->cur_pic_ptr;
    int list, j, field;
    int sidx     = (h->picture_structure & 1) ^ 1;
    int ref1sidx = (ref1->reference      & 1) ^ 1;

    for (list = 0; list < sl->list_count; list++) {
        cur->ref_count[sidx][list] = sl->ref_count[list];
        for (j = 0; j < sl->ref_count[list]; j++)
            cur->ref_poc[sidx][list][j] = 4 * sl->ref_list[list][j].parent->frame_num +
                                          (sl->ref_list[list][j].reference & 3);
    }

    if (h->picture_structure == PICT_FRAME) {
        memcpy(cur->ref_count[1], cur->ref_count[0], sizeof(cur->ref_count[0]));
        memcpy(cur->ref_poc[1],   cur->ref_poc[0],   sizeof(cur->ref_poc[0]));
    }

    if (h->current_slice == 0) {
        cur->mbaff = FRAME_MBAFF(h);
    } else {
        av_assert0(cur->mbaff == FRAME_MBAFF(h));
    }

    sl->col_fieldoff = 0;

    if (sl->list_count != 2 || !sl->ref_count[1])
        return;

    if (h->picture_structure == PICT_FRAME) {
        int cur_poc  = h->cur_pic_ptr->poc;
        int *col_poc = sl->ref_list[1][0].parent->field_poc;
        if (col_poc[0] == INT_MAX && col_poc[1] == INT_MAX) {
            av_log(h->avctx, AV_LOG_ERROR, "co located POCs unavailable\n");
            sl->col_parity = 1;
        } else
            sl->col_parity = (FFABS(col_poc[0] - (int64_t)cur_poc) >=
                              FFABS(col_poc[1] - (int64_t)cur_poc));
        ref1sidx =
        sidx     = sl->col_parity;
    // FL -> FL & differ parity
    } else if (!(h->picture_structure & sl->ref_list[1][0].reference) &&
               !sl->ref_list[1][0].parent->mbaff) {
        sl->col_fieldoff = 2 * sl->ref_list[1][0].reference - 3;
    }

    if (sl->slice_type_nos != AV_PICTURE_TYPE_B || sl->direct_spatial_mv_pred)
        return;

    for (list = 0; list < 2; list++) {
        fill_colmap(h, sl, sl->map_col_to_list0, list, sidx, ref1sidx, 0);
        if (FRAME_MBAFF(h))
            for (field = 0; field < 2; field++)
                fill_colmap(h, sl, sl->map_col_to_list0_field[field], list, field,
                            field, 1);
    }
}
