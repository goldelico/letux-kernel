#ifndef __UAPI_INGENICFB_H__
#define __UAPI_INGENICFB_H__

/* structs export to userspace */

#define PIXEL_ALIGN 4
#define DESC_ALIGN 8
#define MAX_BITS_PER_PIX (32)
#define DPU_MAX_SIZE (2047)
#define DPU_MIN_SIZE (4)
#define DPU_STRIDE_SIZE (4096)
#define DPU_SCALE_MIN_SIZE (20)


#define MAX_STRIDE_VALUE (4096)

/* Maximum layers supported by DPU hardware */
#define DPU_SUPPORT_MAX_LAYERS 4

/* Maximum frames supported by DPU drivers */
#define DPU_SUPPORT_MAX_FRAMES 3


enum {
	DC_WB_FORMAT_8888 = 0,
	DC_WB_FORMAT_565 = 1,
	DC_WB_FORMAT_555 = 2,
	DC_WB_FORMAT_YUV422 = 3,	/*Not support*/
	DC_WB_FORMAT_MONO = 4,		/*Not support*/
	DC_WB_FORMAT_888 = 6,
};

enum {
	LAYER_CFG_FORMAT_RGB555 = 0,
	LAYER_CFG_FORMAT_ARGB1555 = 1,
	LAYER_CFG_FORMAT_RGB565	= 2,
	LAYER_CFG_FORMAT_RGB888	= 4,
	LAYER_CFG_FORMAT_ARGB8888 = 5,
	LAYER_CFG_FORMAT_MONO8 = 6,
	LAYER_CFG_FORMAT_MONO16 = 7,
	LAYER_CFG_FORMAT_NV12 = 8,
	LAYER_CFG_FORMAT_NV21 = 9,
	LAYER_CFG_FORMAT_YUV422 = 10,
	LAYER_CFG_FORMAT_TILE_H264 = 12,
};

enum {
	LAYER_CFG_COLOR_RGB = 0,
	LAYER_CFG_COLOR_RBG = 1,
	LAYER_CFG_COLOR_GRB = 2,
	LAYER_CFG_COLOR_GBR = 3,
	LAYER_CFG_COLOR_BRG = 4,
	LAYER_CFG_COLOR_BGR = 5,
};

enum {
	LAYER_Z_ORDER_0 = 0, /* bottom */
	LAYER_Z_ORDER_1 = 1,
	LAYER_Z_ORDER_2 = 2,
	LAYER_Z_ORDER_3 = 3, /* top */
};


struct ingenicfb_lay_cfg {
	unsigned int lay_en:1;
	unsigned int tlb_en:1;
	unsigned int lay_scale_en:1;
	unsigned int lay_z_order:3;
	unsigned int format:4;
	unsigned int color:3;
	unsigned int g_alpha_en:1;
	unsigned int g_alpha_val:8;
	unsigned int source_w;
	unsigned int source_h;
	unsigned int stride;
	unsigned int uv_stride;	/*NV12 only.*/
	unsigned int scale_w;
	unsigned int scale_h;
	unsigned int disp_pos_x;
	unsigned int disp_pos_y;
	unsigned int addr[3];
	unsigned int uv_addr[3]; /*uv_addr for NV12 format.*/
	unsigned int reserve[4];
};

struct wback_cfg {
	int en;
	unsigned int fmt;
	unsigned int addr;
	unsigned int stride;
	int dither_en;
};


struct ingenicfb_frm_cfg {
	struct ingenicfb_lay_cfg lay_cfg[DPU_SUPPORT_MAX_LAYERS];
	struct wback_cfg wback_info;
	unsigned int width;
	unsigned int height;
};


#define JZFB_PUT_FRM_CFG		_IOWR('F', 0x101, struct ingenicfb_frm_cfg *)
#define JZFB_GET_FRM_CFG		_IOWR('F', 0x102, struct ingenicfb_frm_cfg *)

#define JZFB_SET_CSC_MODE		_IOW('F', 0x120, csc_mode_t)
#define JZFB_USE_TLB			_IOW('F', 0x124, unsigned int)
#define JZFB_DMMU_MAP			_IOWR('F', 0x130, struct dpu_dmmu_map_info)
#define JZFB_DMMU_UNMAP			_IOWR('F', 0x131, struct dpu_dmmu_map_info)
#define JZFB_DMMU_UNMAP_ALL		_IOWR('F', 0x132, struct dpu_dmmu_map_info)
#define JZFB_DMMU_MEM_SMASH		_IOWR('F', 0x133, struct smash_mode)
#define JZFB_DMMU_DUMP_MAP		_IOWR('F', 0x134, unsigned long)
#define JZFB_DMMU_FLUSH_CACHE		_IOWR('F', 0x135, struct dpu_dmmu_map_info)
#define JZFB_DUMP_LCDC_REG		_IOW('F', 0x150, int)

#define JZFB_SET_VSYNCINT		_IOW('F', 0x210, int)
#define JZFB_GET_LAYERS_NUM		_IOWR('F', 0x211, unsigned int)



#endif
