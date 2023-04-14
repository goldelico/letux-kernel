#ifndef __H264E_RC_NL_H__
#define __H264E_RC_NL_H__


#include "h264e_rc_proto.h"
#include "h264e_rc.h"


int h264e_nl_init_ctx(struct h264e_ctx *ctx, struct nl_ctx_info *info);
int h264e_nl_free_ctx(struct h264e_ctx *ctx);
int h264e_nl_setup_headers(struct h264e_ctx *ctx, struct nl_header_info *info);
int h264e_nl_rc_video_cfg(struct h264e_ctx *ctx, struct h264e_params *p);
int h264e_nl_rc_frame_start(struct h264e_ctx *ctx, struct h264e_frame_start_params *p);
int h264e_nl_rc_frame_end(struct h264e_ctx *ctx, struct h264e_frame_end_params *p);

#endif
