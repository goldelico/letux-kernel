#ifndef __H264E_RC_PROTO_H__
#define __H264E_RC_PROTO_H__


/* 创建ctx时，内核提供的地址信息. */
struct nl_ctx_info {
	unsigned int vpu_rc_paddr;	/*内核申请的VPU_RC_S 的物理地址.*/
	unsigned int slice_info_paddr;	/*内核申请的Sliceinfo_t 的物理地址.*/
	unsigned int emc_buf_paddr;	/*内核申请的emc_buf的物理地址.*/
	unsigned int desc_paddr;	/*内核申请的dma buf的物理地址.*/
	uint8_t rc_mode;
};

struct nl_header_info {
	h264_pps_t pps;
	h264_sps_t sps;
};

/**
* @bitrate, set by V4L2_CID_MPEG_VIDEO_BITRATE.
* @gop_size, group of picture when h264 encoding.
*
* used by kernel and app. params across kernel and app. for video_cfg.
*/
struct h264e_params {

	/* params modify by s_ctrl*/
	uint32_t bitrate;
	uint32_t framerate;
	int h264_hdr_mode;
#if 0
	uint8_t h264_intra_qp;
	uint8_t h264_inter_qp;
#endif
	uint8_t	i_qp;
	uint8_t p_qp;
	uint8_t h264_min_qp;
	uint8_t h264_max_qp;
	uint8_t h264_profile;
	uint8_t h264_level;
	uint8_t gop_size;
	uint8_t frame_rc_enable;
	uint8_t mb_rc_enable;
	uint8_t num_bframe;
	uint8_t interlaced;
	uint8_t max_ref_pic;
	uint8_t h264_8x8_transform;
	uint8_t rc_mode;

	uint8_t i_cabac; /*is cabac entropy?*/

	int i_idr_pic_id;
	int i_global_qp;

	uint8_t deblock;

	int height;
	int width;
	int format;

};

struct h264e_frame_start_params {
	unsigned int bIFrmReq;
	unsigned int frmSkipType;


	/* Slice info. */
	//unsigned int des_pa;
	unsigned int raw_format;
	unsigned int emc_bs_pa;
	unsigned int emc_dblk_pa;

	/* VPU input buffer address. */
	unsigned int raw[3];
	unsigned int fb[3][2];
	unsigned int stride[2];

	unsigned int slice_header_len; /*TO kernel.*/
};

struct h264e_frame_end_params {
	unsigned int u32FrmActBs;
};

enum {
	VPU_CMD_CREATE_CTX = 1,
	VPU_CMD_FREE_CTX,
	VPU_CMD_SETUP_HEADERS,
	VPU_CMD_EPRC_SET_DEFAULT,
	VPU_CMD_RC_VIDEO_CFG,
	VPU_CMD_RC_FRAME_START,
	VPU_CMD_RC_FRAME_END,
	VPU_CMD_SERVER_ONLINE,	/*From server to kernel*/
	VPU_CMD_SERVER_OFFLINE, /*From server to kernel*/
};

struct h264e_nl_msg {
	unsigned int ctx_id;	/*当前msg对应的ctx id. 当命令为VPU_CMD_CREATE_CTX 时，此值由应用server填充.*/
	unsigned int cmd;	/*当前msg对应的调用命令.*/
	int ret_val;

	unsigned char buf[0];	/*Dynamic malloc buffer.*/
};


#define NETLINK_H264_ENCODER	30
#define MAXPALOAD	64

#define NETLINK_H264_PORT	0x26402640


#endif
