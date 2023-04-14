#include "avassert.h"
#include "avcodec.h"
#include "h264dec.h"
#include "h264data.h"
#include "mpegutils.h"

void ff_h264_unref_picture(H264Context *h, H264Picture *pic)
{
    if (!pic->f || !pic->f->buf[0])
        return;

}

int ff_h264_ref_picture(H264Context *h, H264Picture *dst, H264Picture *src)
{
    int i;

    for (i = 0; i < 2; i++)
        dst->field_poc[i] = src->field_poc[i];

    memcpy(dst->ref_poc,   src->ref_poc,   sizeof(src->ref_poc));
    memcpy(dst->ref_count, src->ref_count, sizeof(src->ref_count));

    dst->poc           = src->poc;
    dst->frame_num     = src->frame_num;
    dst->mmco_reset    = src->mmco_reset;
    dst->long_ref      = src->long_ref;
    dst->mbaff         = src->mbaff;
    dst->field_picture = src->field_picture;
    dst->reference     = src->reference;
    dst->recovered     = src->recovered;
    dst->invalid_gap   = src->invalid_gap;

    return 0;
}

int ff_h264_field_end(H264Context *h, H264SliceContext *sl, int in_setup)
{
    int err = 0;
    h->mb_y = 0;


    if (in_setup || 1 /*|| !(avctx->active_thread_type & FF_THREAD_FRAME)*/) {
        if (!h->droppable) {
            err = ff_h264_execute_ref_pic_marking(h);
            h->poc.prev_poc_msb = h->poc.poc_msb;
            h->poc.prev_poc_lsb = h->poc.poc_lsb;
        }
        h->poc.prev_frame_num_offset = h->poc.frame_num_offset;
        h->poc.prev_frame_num        = h->poc.frame_num;
    }

    h->current_slice = 0;

    return err;
}
