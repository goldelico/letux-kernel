#ifndef __AVCODEC_H__
#define __AVCODEC_H__

#include "buffer.h"
#include "frame.h"

/**
 * Allow decoders to produce frames with data planes that are not aligned
 * to CPU requirements (e.g. due to cropping).
 */
#define AV_CODEC_FLAG_UNALIGNED       (1 <<  0)
/**
 * Use fixed qscale.
 */
#define AV_CODEC_FLAG_QSCALE          (1 <<  1)
/**
 * 4 MV per MB allowed / advanced prediction for H.263.
 */
#define AV_CODEC_FLAG_4MV             (1 <<  2)
/**
 * Output even those frames that might be corrupted.
 */
#define AV_CODEC_FLAG_OUTPUT_CORRUPT  (1 <<  3)
/**
 * Use qpel MC.
 */
#define AV_CODEC_FLAG_QPEL            (1 <<  4)
/**
 * Use internal 2pass ratecontrol in first pass mode.
 */
#define AV_CODEC_FLAG_PASS1           (1 <<  9)
/**
 * Use internal 2pass ratecontrol in second pass mode.
 */
#define AV_CODEC_FLAG_PASS2           (1 << 10)
/**
 * loop filter.
 */
#define AV_CODEC_FLAG_LOOP_FILTER     (1 << 11)
/**
 * Only decode/encode grayscale.
 */
#define AV_CODEC_FLAG_GRAY            (1 << 13)
/**
 * error[?] variables will be set during encoding.
 */
#define AV_CODEC_FLAG_PSNR            (1 << 15)
/**
 * Input bitstream might be truncated at a random location
 * instead of only at frame boundaries.
 */
#define AV_CODEC_FLAG_TRUNCATED       (1 << 16)
/**
 * Use interlaced DCT.
 */
#define AV_CODEC_FLAG_INTERLACED_DCT  (1 << 18)
/**
 * Force low delay.
 */
#define AV_CODEC_FLAG_LOW_DELAY       (1 << 19)
/**
 * Place global headers in extradata instead of every keyframe.
 */
#define AV_CODEC_FLAG_GLOBAL_HEADER   (1 << 22)
/**
 * Use only bitexact stuff (except (I)DCT).
 */
#define AV_CODEC_FLAG_BITEXACT        (1 << 23)
/* Fx : Flag for H.263+ extra options */
/**
 * H.263 advanced intra coding / MPEG-4 AC prediction
 */
#define AV_CODEC_FLAG_AC_PRED         (1 << 24)
/**
 * interlaced motion estimation
 */
#define AV_CODEC_FLAG_INTERLACED_ME   (1 << 29)
#define AV_CODEC_FLAG_CLOSED_GOP      (1U << 31)

/**
 * Allow non spec compliant speedup tricks.
 */
#define AV_CODEC_FLAG2_FAST           (1 <<  0)
/**
 * Skip bitstream encoding.
 */
#define AV_CODEC_FLAG2_NO_OUTPUT      (1 <<  2)
/**
 * Place global headers at every keyframe instead of in extradata.
 */
#define AV_CODEC_FLAG2_LOCAL_HEADER   (1 <<  3)

/**
 * timecode is in drop frame format. DEPRECATED!!!!
 */
#define AV_CODEC_FLAG2_DROP_FRAME_TIMECODE (1 << 13)

/**
 * Input bitstream might be truncated at a packet boundaries
 * instead of only at frame boundaries.
 */
#define AV_CODEC_FLAG2_CHUNKS         (1 << 15)
/**
 * Discard cropping information from SPS.
 */
#define AV_CODEC_FLAG2_IGNORE_CROP    (1 << 16)

/**
 * Show all frames before the first keyframe
 */
#define AV_CODEC_FLAG2_SHOW_ALL       (1 << 22)
/**
 * Export motion vectors through frame side data
 */
#define AV_CODEC_FLAG2_EXPORT_MVS     (1 << 28)
/**
 * Do not skip samples and export skip information as frame side data
 */
#define AV_CODEC_FLAG2_SKIP_MANUAL    (1 << 29)
/**
 * Do not reset ASS ReadOrder field on flush (subtitles decoding)
 */
#define AV_CODEC_FLAG2_RO_FLUSH_NOOP  (1 << 30)

/**
 * Identify the syntax and semantics of the bitstream.
 * The principle is roughly:
 * Two decoders with the same ID can decode the same streams.
 * Two encoders with the same ID can encode compatible streams.
 * There may be slight deviations from the principle due to implementation
 * details.
 *
 * If you add a codec ID to this list, add it so that
 * 1. no value of an existing codec ID changes (that would break ABI),
 * 2. it is as close as possible to similar codecs
 *
 * After adding new codec IDs, do not forget to add an entry to the codec
 * descriptor list and bump libavcodec minor version.
 */
enum AVCodecID {
    AV_CODEC_ID_NONE,

    AV_CODEC_ID_H264
};


typedef struct AVCodecContext {


    int log_level_offset;

    enum AVCodecID   codec_id;

    /**
     * AV_CODEC_FLAG_*.
     * - encoding: Set by user.
     * - decoding: Set by user.
     */
    int flags;

    /**
     * AV_CODEC_FLAG2_*
     * - encoding: Set by user.
     * - decoding: Set by user.
     */
    int flags2;

    /**
     * number of reference frames
     * - encoding: Set by user.
     * - decoding: Set by lavc.
     */
    int refs;

    /* video only */
    /**
     * picture width / height.
     *
     * @note Those fields may not match the values of the last
     * AVFrame output by avcodec_decode_video2 due frame
     * reordering.
     *
     * - encoding: MUST be set by user.
     * - decoding: May be set by the user before opening the decoder if known e.g.
     *             from the container. Some decoders will require the dimensions
     *             to be set by the caller. During decoding, the decoder may
     *             overwrite those values as required while parsing the data.
     */
    int width, height;

    /**
     * Bitstream width / height, may be different from width/height e.g. when
     * the decoded frame is cropped before being output or lowres is enabled.
     *
     * @note Those field may not match the value of the last
     * AVFrame output by avcodec_receive_frame() due frame
     * reordering.
     *
     * - encoding: unused
     * - decoding: May be set by the user before opening the decoder if known
     *             e.g. from the container. During decoding, the decoder may
     *             overwrite those values as required while parsing the data.
     */
    int coded_width, coded_height;

    /**
     * Size of the frame reordering buffer in the decoder.
     * For MPEG-2 it is 1 IPB or 0 low delay IP.
     * - encoding: Set by libavcodec.
     * - decoding: Set by libavcodec.
     */
    int has_b_frames;

    /**
     * debug
     * - encoding: Set by user.
     * - decoding: Set by user.
     */
    int debug;
#define FF_DEBUG_PICT_INFO   1
#define FF_DEBUG_RC          2
#define FF_DEBUG_BITSTREAM   4
#define FF_DEBUG_MB_TYPE     8
#define FF_DEBUG_QP          16

#define FF_DEBUG_DCT_COEFF   0x00000040
#define FF_DEBUG_SKIP        0x00000080
#define FF_DEBUG_STARTCODE   0x00000100
#define FF_DEBUG_ER          0x00000400
#define FF_DEBUG_MMCO        0x00000800
#define FF_DEBUG_BUGS        0x00001000

#define FF_DEBUG_BUFFERS     0x00008000
#define FF_DEBUG_THREADS     0x00010000
#define FF_DEBUG_GREEN_MD    0x00800000
#define FF_DEBUG_NOMC        0x01000000

    /**
     * Error recognition; may misdetect some more or less valid parts as errors.
     * - encoding: unused
     * - decoding: Set by user.
     */
    int err_recognition;

/**
 * Verify checksums embedded in the bitstream (could be of either encoded or
 * decoded data, depending on the codec) and print an error message on mismatch.
 * If AV_EF_EXPLODE is also set, a mismatching checksum will result in the
 * decoder returning an error.
 */
#define AV_EF_CRCCHECK  (1<<0)
#define AV_EF_BITSTREAM (1<<1)          ///< detect bitstream specification deviations
#define AV_EF_BUFFER    (1<<2)          ///< detect improper bitstream length
#define AV_EF_EXPLODE   (1<<3)          ///< abort decoding on minor error detection

#define AV_EF_IGNORE_ERR (1<<15)        ///< ignore errors and continue
#define AV_EF_CAREFUL    (1<<16)        ///< consider things that violate the spec, are fast to calculate and have not been seen in the wild as errors
#define AV_EF_COMPLIANT  (1<<17)        ///< consider all spec non compliances as errors
#define AV_EF_AGGRESSIVE (1<<18)        ///< consider things that a sane encoder should not do as an error

    /**
     * profile
     * - encoding: Set by user.
     * - decoding: Set by libavcodec.
     */
     int profile;
#define FF_PROFILE_UNKNOWN -99
#define FF_PROFILE_RESERVED -100

#define FF_PROFILE_AAC_MAIN 0
#define FF_PROFILE_AAC_LOW  1
#define FF_PROFILE_AAC_SSR  2
#define FF_PROFILE_AAC_LTP  3
#define FF_PROFILE_AAC_HE   4
#define FF_PROFILE_AAC_HE_V2 28
#define FF_PROFILE_AAC_LD   22
#define FF_PROFILE_AAC_ELD  38
#define FF_PROFILE_MPEG2_AAC_LOW 128
#define FF_PROFILE_MPEG2_AAC_HE  131

#define FF_PROFILE_DNXHD         0
#define FF_PROFILE_DNXHR_LB      1
#define FF_PROFILE_DNXHR_SQ      2
#define FF_PROFILE_DNXHR_HQ      3
#define FF_PROFILE_DNXHR_HQX     4
#define FF_PROFILE_DNXHR_444     5

#define FF_PROFILE_DTS         20
#define FF_PROFILE_DTS_ES      30
#define FF_PROFILE_DTS_96_24   40
#define FF_PROFILE_DTS_HD_HRA  50
#define FF_PROFILE_DTS_HD_MA   60
#define FF_PROFILE_DTS_EXPRESS 70

#define FF_PROFILE_MPEG2_422    0
#define FF_PROFILE_MPEG2_HIGH   1
#define FF_PROFILE_MPEG2_SS     2
#define FF_PROFILE_MPEG2_SNR_SCALABLE  3
#define FF_PROFILE_MPEG2_MAIN   4
#define FF_PROFILE_MPEG2_SIMPLE 5

#define FF_PROFILE_H264_CONSTRAINED  (1<<9)  // 8+1; constraint_set1_flag
#define FF_PROFILE_H264_INTRA        (1<<11) // 8+3; constraint_set3_flag

#define FF_PROFILE_H264_BASELINE             66
#define FF_PROFILE_H264_CONSTRAINED_BASELINE (66|FF_PROFILE_H264_CONSTRAINED)
#define FF_PROFILE_H264_MAIN                 77
#define FF_PROFILE_H264_EXTENDED             88
#define FF_PROFILE_H264_HIGH                 100
#define FF_PROFILE_H264_HIGH_10              110
#define FF_PROFILE_H264_HIGH_10_INTRA        (110|FF_PROFILE_H264_INTRA)
#define FF_PROFILE_H264_MULTIVIEW_HIGH       118
#define FF_PROFILE_H264_HIGH_422             122
#define FF_PROFILE_H264_HIGH_422_INTRA       (122|FF_PROFILE_H264_INTRA)
#define FF_PROFILE_H264_STEREO_HIGH          128
#define FF_PROFILE_H264_HIGH_444             144
#define FF_PROFILE_H264_HIGH_444_PREDICTIVE  244
#define FF_PROFILE_H264_HIGH_444_INTRA       (244|FF_PROFILE_H264_INTRA)
#define FF_PROFILE_H264_CAVLC_444            44

#define FF_PROFILE_VC1_SIMPLE   0
#define FF_PROFILE_VC1_MAIN     1
#define FF_PROFILE_VC1_COMPLEX  2
#define FF_PROFILE_VC1_ADVANCED 3

#define FF_PROFILE_MPEG4_SIMPLE                     0
#define FF_PROFILE_MPEG4_SIMPLE_SCALABLE            1
#define FF_PROFILE_MPEG4_CORE                       2
#define FF_PROFILE_MPEG4_MAIN                       3
#define FF_PROFILE_MPEG4_N_BIT                      4
#define FF_PROFILE_MPEG4_SCALABLE_TEXTURE           5
#define FF_PROFILE_MPEG4_SIMPLE_FACE_ANIMATION      6
#define FF_PROFILE_MPEG4_BASIC_ANIMATED_TEXTURE     7
#define FF_PROFILE_MPEG4_HYBRID                     8
#define FF_PROFILE_MPEG4_ADVANCED_REAL_TIME         9
#define FF_PROFILE_MPEG4_CORE_SCALABLE             10
#define FF_PROFILE_MPEG4_ADVANCED_CODING           11
#define FF_PROFILE_MPEG4_ADVANCED_CORE             12
#define FF_PROFILE_MPEG4_ADVANCED_SCALABLE_TEXTURE 13
#define FF_PROFILE_MPEG4_SIMPLE_STUDIO             14
#define FF_PROFILE_MPEG4_ADVANCED_SIMPLE           15

#define FF_PROFILE_JPEG2000_CSTREAM_RESTRICTION_0   1
#define FF_PROFILE_JPEG2000_CSTREAM_RESTRICTION_1   2
#define FF_PROFILE_JPEG2000_CSTREAM_NO_RESTRICTION  32768
#define FF_PROFILE_JPEG2000_DCINEMA_2K              3
#define FF_PROFILE_JPEG2000_DCINEMA_4K              4

#define FF_PROFILE_VP9_0                            0
#define FF_PROFILE_VP9_1                            1
#define FF_PROFILE_VP9_2                            2
#define FF_PROFILE_VP9_3                            3

#define FF_PROFILE_HEVC_MAIN                        1
#define FF_PROFILE_HEVC_MAIN_10                     2
#define FF_PROFILE_HEVC_MAIN_STILL_PICTURE          3
#define FF_PROFILE_HEVC_REXT                        4

#define FF_PROFILE_AV1_MAIN                         0
#define FF_PROFILE_AV1_HIGH                         1
#define FF_PROFILE_AV1_PROFESSIONAL                 2

#define FF_PROFILE_MJPEG_HUFFMAN_BASELINE_DCT            0xc0
#define FF_PROFILE_MJPEG_HUFFMAN_EXTENDED_SEQUENTIAL_DCT 0xc1
#define FF_PROFILE_MJPEG_HUFFMAN_PROGRESSIVE_DCT         0xc2
#define FF_PROFILE_MJPEG_HUFFMAN_LOSSLESS                0xc3
#define FF_PROFILE_MJPEG_JPEG_LS                         0xf7

#define FF_PROFILE_SBC_MSBC                         1

    /**
     * level
     * - encoding: Set by user.
     * - decoding: Set by libavcodec.
     */
     int level;
#define FF_LEVEL_UNKNOWN -99

    void *priv_data;

} AVCodecContext;

/**
 * @ingroup lavc_decoding
 * Required number of additionally allocated bytes at the end of the input bitstream for decoding.
 * This is mainly needed because some optimized bitstream readers read
 * 32 or 64 bit at once and could read over the end.<br>
 * Note: If the first 23 bits of the additional bytes are not 0, then damaged
 * MPEG bitstreams could cause overread and segfault.
 */
#define AV_INPUT_BUFFER_PADDING_SIZE 64


typedef struct AVPacket {

	uint8_t *data;
	unsigned int data_pa;	/*physical address of data.*/
	int	size;

} AVPacket;



#endif
