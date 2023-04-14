#include "avassert.h"
#include "avcodec.h"
#include "h264.h"
#include "h264dec.h"
#include "h2645_parse.h"
#include "h264data.h"
#include "h264_ps.h"
#include "golomb.h"
#include "mathops.h"


/**
 * Init context
 * Allocate buffers which are not shared amongst multiple threads.
 */

AVFrame *h264_dequeue_frame(H264Context *h)
{
	int index = 0;
	AVFrame *f = NULL;

	f = list_first_entry_or_null(&h->done_list, AVFrame, done_entry);

	if(!f) {
		return NULL;
	}

	/*remove picture from output picture list.*/
	list_del(&f->done_entry);

	/*返回AVFrame.*/
	return f;
}

int h264_enqueue_frame(H264Context *h, AVFrame *f)
{
	INIT_LIST_HEAD(&f->queued_entry);

	list_add_tail(&f->queued_entry, &h->queued_list);
	f->queued = 1;

	return 0;
}

AVFrame * h264_get_queued_frame(H264Context *h)
{
	AVFrame *f = NULL;
	int i;

	/* 1. AVFrame From DPB, which is in processing. */
	for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
		f = h->DPB[i].f;
		if(f && f->queued) {
			f->queued = 0;
			return f;
		}
	}

	f = list_first_entry_or_null(&h->queued_list, AVFrame, queued_entry);
	if(!f) {
		return NULL;
	}

	/*remove picture from output picture list.*/
	list_del(&f->queued_entry);

	return f;
}

static int h264_init_context(AVCodecContext *avctx, H264Context *h)
{
    int i;
    int ret;

    h->avctx                 = avctx;
    h->cur_chroma_format_idc = -1;

    h->width_from_caller     = avctx->width;
    h->height_from_caller    = avctx->height;
    h->format		     = VPU_FORMAT_TILE420;

    h->picture_structure     = PICT_FRAME;

    h->flags                 = avctx->flags;
    h->poc.prev_poc_msb      = 1 << 16;
    h->recovery_frame        = -1;
    h->frame_recovered       = 0;
    h->poc.prev_frame_num    = -1;

    h->next_outputed_poc = INT_MIN;
    for (i = 0; i < MAX_DELAYED_PIC_COUNT; i++)
        h->last_pocs[i] = INT_MIN;

    h->nb_slice_ctx = 1;
    h->slice_ctx = av_mallocz_array(h->nb_slice_ctx, sizeof(*h->slice_ctx));
    if (!h->slice_ctx) {
        h->nb_slice_ctx = 0;
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        h->DPB[i].f = NULL;
    }


    ret = devmem_ctx_init(&h->devmem_ctx, NULL);
    if(ret < 0) {
    	return AVERROR(EINVAL);
    }


    h->st_h264 = av_mallocz(sizeof(*h->st_h264));
    if(!h->st_h264)
	    return AVERROR(ENOMEM);

    h->dma_desc = av_buffer_alloc(&h->devmem_ctx, DMA_DESC_SIZE);
    if(!h->dma_desc)
	    return AVERROR(ENOMEM);
    memset(h->dma_desc->buffer, 0, DMA_DESC_SIZE);

    ret = vpu_ctx_init(&h->vpu_ctx);
    if(ret < 0)
	    return AVERROR(EINVAL);
    h->vpu_ctx.desc_va = h->dma_desc->buffer;
    h->vpu_ctx.desc_pa = h->dma_desc->buffer_pa;

    for (i = 0; i < h->nb_slice_ctx; i++)
        h->slice_ctx[i].h264 = h;

    INIT_LIST_HEAD(&h->done_list);
    INIT_LIST_HEAD(&h->queued_list);

    return 0;
}

extern void av_buffer_dump(void);
int h264_decode_end(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    ff_h264_remove_all_refs(h);

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        ff_h264_unref_picture(h, &h->DPB[i]);
	ff_h264_release_picture(h, &h->DPB[i]);
    }
    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));

    h->cur_pic_ptr = NULL;

    av_freep(&h->slice_ctx);
    h->nb_slice_ctx = 0;

    av_buffer_free(&h->devmem_ctx, h->dma_desc);

    av_free(h->st_h264);


#if 0
    ff_h264_sei_uninit(&h->sei);
#endif
    ff_h264_ps_uninit(&h->ps);

    ff_h2645_packet_uninit(&h->pkt);

    vpu_ctx_uninit(&h->vpu_ctx);
    devmem_ctx_uninit(&h->devmem_ctx);

    av_buffer_dump();

    return 0;
}

//static AVOnce h264_vlc_init = AV_ONCE_INIT;

int h264_decode_init(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int ret;

    ret = h264_init_context(avctx, h);
    if (ret < 0)
        return ret;

    ff_h264_flush_change(h);

    av_log(avctx, AV_LOG_ERROR, "h264_decode_init done!\n");

    return 0;
}

int h264_set_vpu_ops(H264Context *h, void *priv_data, struct vpu_ops *ops)
{
	return vpu_ctx_set_ops(&h->vpu_ctx, priv_data, ops);
}

/**
 * instantaneous decoder refresh.
 */
static void idr(H264Context *h)
{
    int i;
    ff_h264_remove_all_refs(h);
    h->poc.prev_frame_num        =
    h->poc.prev_frame_num_offset = 0;
    h->poc.frame_num= 0;
    h->poc.prev_poc_msb          = 1<<16;
    h->poc.prev_poc_lsb          = 0;
    for (i = 0; i < MAX_DELAYED_PIC_COUNT; i++)
        h->last_pocs[i] = INT_MIN;
}

/* forget old pics after a seek */
void ff_h264_flush_change(H264Context *h)
{
    int i, j;

    h->next_outputed_poc = INT_MIN;
    h->prev_interlaced_frame = 1;
    idr(h);

    h->poc.prev_frame_num = -1;
    if (h->cur_pic_ptr) {
        h->cur_pic_ptr->reference = 0;
        for (j=i=0; h->delayed_pic[i]; i++)
            if (h->delayed_pic[i] != h->cur_pic_ptr)
                h->delayed_pic[j++] = h->delayed_pic[i];
        h->delayed_pic[j] = NULL;
    }

    h->first_field = 0;
    h->recovery_frame = -1;
    h->frame_recovered = 0;
    h->current_slice = 0;
    h->mmco_reset = 1;
}

/* forget old pics after a seek */
static void flush_dpb(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));

    ff_h264_flush_change(h);
#if 0
    ff_h264_sei_uninit(&h->sei);
#endif

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++)
        ff_h264_unref_picture(h, &h->DPB[i]);
    h->cur_pic_ptr = NULL;

    h->mb_y = 0;

    h->context_initialized = 0;
}

static int get_last_needed_nal(H264Context *h)
{
    int nals_needed = 0;
    int first_slice = 0;
    int i, ret;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        GetBitContext gb;

        /* packets can sometimes contain multiple PPS/SPS,
         * e.g. two PAFF field pictures in one packet, or a demuxer
         * which splits NALs strangely if so, when frame threading we
         * can't start the next thread until we've read all of them */
        switch (nal->type) {
        case H264_NAL_SPS:
        case H264_NAL_PPS:
            nals_needed = i;
            break;
        case H264_NAL_DPA:
        case H264_NAL_IDR_SLICE:
        case H264_NAL_SLICE:
            ret = init_get_bits8(&gb, nal->data + 1, nal->size - 1);
            if (ret < 0) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid zero-sized VCL NAL unit\n");
                if (h->avctx->err_recognition & AV_EF_EXPLODE)
                    return ret;

                break;
            }
            if (!get_ue_golomb_long(&gb) ||  // first_mb_in_slice
                !first_slice ||
                first_slice != nal->type)
                nals_needed = i;
            if (!first_slice)
                first_slice = nal->type;
        }
    }

    return nals_needed;
}

static int decode_nal_units(H264Context *h, const uint8_t *buf, int buf_size)
{
    AVCodecContext *const avctx = h->avctx;
    int nals_needed = 0; ///< number of NALs that need decoding before the next frame thread starts
    int idr_cleared=0;
    int i, ret = 0;

    h->has_slice = 0;
    h->nal_unit_type= 0;

#if 0
    h->current_slice = 0;
    if (!h->first_field) {
	    h->cur_pic_ptr = NULL;
	    ff_h264_sei_uninit(&h->sei);
    }
#endif

#if 0
    if (h->nal_length_size == 4) {
        if (buf_size > 8 && AV_RB32(buf) == 1 && AV_RB32(buf+5) > (unsigned)buf_size) {
            h->is_avc = 0;
        }else if(buf_size > 3 && AV_RB32(buf) > 1 && AV_RB32(buf) <= (unsigned)buf_size)
            h->is_avc = 1;
    }
#endif
    ret = ff_h2645_packet_split(&h->pkt, buf, buf_size, avctx, 0,
                                h->nal_length_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Error splitting the input into NAL units.\n");
        return ret;
    }

    if (nals_needed < 0)
        return nals_needed;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        int max_slice_ctx, err;
        // FIXME these should stop being context-global variables
        h->nal_ref_idc   = nal->ref_idc;
        h->nal_unit_type = nal->type;

        err = 0;
        switch (nal->type) {
        case H264_NAL_IDR_SLICE:
            if ((nal->data[1] & 0xFC) == 0x98) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid inter IDR frame\n");
                h->next_outputed_poc = INT_MIN;
                ret = -1;
                goto end;
            }
            if(!idr_cleared) {
                idr(h); // FIXME ensure we don't lose some frames if there is reordering
            }
            idr_cleared = 1;
            h->has_recovery_point = 1;
        case H264_NAL_SLICE:
            h->has_slice = 1;

	    	/* current_slice = 0 的。。。*/
            if ((err = ff_h264_queue_decode_slice(h, nal))) {
		    /*如果入队出错, 将 ref_count都清0.*/
                H264SliceContext *sl = h->slice_ctx + h->nb_slice_ctx_queued;
                sl->ref_count[0] = sl->ref_count[1] = 0;
                break;
            }


            if((ret = ff_h264_execute_decode_slices(h)) < 0) {
		flush_dpb(h->avctx);
		printk("===============%s, %d, ret:%d\n", __func__, __LINE__, ret);
		goto end;
	    }
            break;
        case H264_NAL_DPA:
        case H264_NAL_DPB:
        case H264_NAL_DPC:
            av_log(avctx, AV_LOG_ERROR, "data partitioning");
            break;
        case H264_NAL_SEI:
#if 0
            ret = ff_h264_sei_decode(&h->sei, &nal->gb, &h->ps, avctx);
            h->has_recovery_point = h->has_recovery_point || h->sei.recovery_point.recovery_frame_cnt != -1;
#endif
#if 0
            if (avctx->debug & FF_DEBUG_GREEN_MD)
                debug_green_metadata(&h->sei.green_metadata, h->avctx);
#endif
#if 0
            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
#endif
            break;
        case H264_NAL_SPS: {
            GetBitContext tmp_gb = nal->gb;

            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;

            av_log(h->avctx, AV_LOG_DEBUG,
                   "SPS decoding failure, trying again with the complete NAL\n");
            init_get_bits8(&tmp_gb, nal->raw_data + 1, nal->raw_size - 1);
            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;
            ff_h264_decode_seq_parameter_set(&nal->gb, avctx, &h->ps, 1);
            break;
        }
        case H264_NAL_PPS:
            ret = ff_h264_decode_picture_parameter_set(&nal->gb, avctx, &h->ps,
                                                       nal->size_bits);

            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
            break;
        case H264_NAL_AUD:
        case H264_NAL_END_SEQUENCE:
        case H264_NAL_END_STREAM:
        case H264_NAL_FILLER_DATA:
        case H264_NAL_SPS_EXT:
        case H264_NAL_AUXILIARY_SLICE:
            break;
        default:
            av_log(avctx, AV_LOG_DEBUG, "Unknown NAL code: %d (%d bits)\n",
                   nal->type, nal->size_bits);
        }

        if (err < 0) {
            av_log(h->avctx, AV_LOG_ERROR, "decode_slice_header error\n");
	    ret = AVERROR_INVALIDDATA;
	    goto end;
        }
    }

#if 0
    ret = ff_h264_execute_decode_slices(h);
    if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
        goto end;
#endif
    ret = 0;
end:
    return (ret < 0) ? ret : buf_size;
}

/**
 * Return the number of bytes consumed for building the current frame.
 */
static int get_consumed_bytes(int pos, int buf_size)
{
    if (pos == 0)
        pos = 1;        // avoid infinite loops (I doubt that is needed but...)
    if (pos + 10 > buf_size)
        pos = buf_size; // oops ;)

    return pos;
}

static int output_frame(H264Context *h, AVFrame *dst, H264Picture *srcp)
{

    AVFrame *f = srcp->f;

    f->queued = 0;
    INIT_LIST_HEAD(&f->done_entry);

    list_add_tail(&f->done_entry, &h->done_list);

    if(dst) {
	    memcpy(dst, f, sizeof(AVFrame));
    }
    return 0;
}

static int is_extra(const uint8_t *buf, int buf_size)
{
    int cnt= buf[5]&0x1f;
    const uint8_t *p= buf+6;
    if (!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 7)
            return 0;
        p += nalsize;
    }
    cnt = *(p++);
    if(!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 8)
            return 0;
        p += nalsize;
    }
    return 1;
}

static int finalize_frame(H264Context *h, AVFrame *dst, H264Picture *out, int *got_frame)
{
    int ret;

     if(out->recovered) {

        ret = output_frame(h, dst, out);
        if (ret < 0)
            return ret;

        *got_frame = 1;
    }

    return 0;
}

static int send_next_delayed_frame(H264Context *h, AVFrame *dst_frame,
                                   int *got_frame, int buf_index)
{
    int ret, i, out_idx;
    H264Picture *out = h->delayed_pic[0];

    h->cur_pic_ptr = NULL;
    h->first_field = 0;

    out_idx = 0;
    for (i = 1;
         h->delayed_pic[i] &&
         !h->delayed_pic[i]->f->key_frame &&
         !h->delayed_pic[i]->mmco_reset;
         i++)
        if (h->delayed_pic[i]->poc < out->poc) {
            out     = h->delayed_pic[i];
            out_idx = i;
        }

    for (i = out_idx; h->delayed_pic[i]; i++)
        h->delayed_pic[i] = h->delayed_pic[i + 1];

    if (out) {
        out->reference &= ~DELAYED_PIC_REF;
        ret = finalize_frame(h, dst_frame, out, got_frame);
        if (ret < 0)
            return ret;
    }

    return buf_index;
}

/**
 * @brief 	解码一个packet中包含的帧数据,有可能有数据，有可能没有.通过got_frame指示.
 *
 * @param avctx	句柄
 * @param data	输出buffer.
 * @param got_frame 是否有数据
 * @param avpkt 输入数据.
 *
 * @return <0，解码出错 >=0 返回消耗的数据长度.
 */
int h264_decode_frame(AVCodecContext *avctx, void *data,
                             int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;  /* 码流地址*/
    int buf_size       = avpkt->size;  /* 码流大小*/
    H264Context *h     = avctx->priv_data;
    AVFrame *pict      = data; /*解码到的buffer.*/
    int buf_index;
    int ret;

    h->flags = avctx->flags;
    h->setup_finished = 0;
    h->nb_slice_ctx_queued = 0;

    /* end of stream, output what is still in the buffers */
    if (buf_size == 0)
        return send_next_delayed_frame(h, pict, got_frame, 0); /*为什么叫做delayed_frame， 不明白. */

    /* *************重点处理的内容 *******/
    buf_index = decode_nal_units(h, buf, buf_size);
    if (buf_index < 0)
        return AVERROR_INVALIDDATA;

    if (!h->cur_pic_ptr && h->nal_unit_type == H264_NAL_END_SEQUENCE) {
        av_assert0(buf_index <= buf_size);
        return send_next_delayed_frame(h, pict, got_frame, buf_index);
    }


#if 0
    if ((!h->cur_pic_ptr || !h->has_slice)) {
        if (/*avctx->skip_frame >= AVDISCARD_NONREF ||*/
            buf_size >= 4 && !memcmp("Q264", buf, 4))
            return buf_size;
        av_log(avctx, AV_LOG_ERROR, "no frame!\n");
        return AVERROR_INVALIDDATA;
    }
#endif


    if(h->mb_y >= h->mb_height && h->mb_height) {
	    if ((ret = ff_h264_field_end(h, &h->slice_ctx[0], 0)) < 0)
		    return ret;

	    if (h->next_output_pic) {
		    ret = finalize_frame(h, pict, h->next_output_pic, got_frame);
		    if (ret < 0)
			    return ret;
	    }
    }

    return get_consumed_bytes(buf_index, buf_size);
}

