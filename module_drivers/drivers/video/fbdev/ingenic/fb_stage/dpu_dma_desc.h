#ifndef __DPU_DMA_DESC_H__
#define __DPU_DMA_DESC_H__


typedef union frm_size {
	uint32_t d32;
	struct {
		uint32_t width:12;
	        uint32_t reserve12_15:4;
	        uint32_t height:12;
	        uint32_t reserve28_31:4;
	}b;
} frm_size_t;

typedef union frm_ctrl {
	uint32_t d32;
	struct {
		uint32_t stop:1;
		uint32_t wb_en:1;
		uint32_t direct_en:1;
		uint32_t change_2_rdma:1;
		uint32_t wb_dither_en:1;
		uint32_t wb_dither_auto:1;
		uint32_t reserve6_15:10;
		uint32_t wb_format:3;
		uint32_t reserve19:1;
		uint32_t wb_dither_b_dw:2;
		uint32_t wb_dither_g_dw:2;
		uint32_t wb_dither_r_dw:2;
		uint32_t reserve26_31:6;
	}b;
} frm_ctrl_t;

typedef union lay_cfg_en {
	uint32_t d32;
	struct {
		uint32_t lay0_scl_en:1;
		uint32_t lay1_scl_en:1;
		uint32_t lay2_scl_en:1;
		uint32_t lay3_scl_en:1;
		uint32_t lay0_en:1;
		uint32_t lay1_en:1;
		uint32_t lay2_en:1;
		uint32_t lay3_en:1;
		uint32_t lay0_z_order:2;
		uint32_t lay1_z_order:2;
		uint32_t lay2_z_order:2;
		uint32_t lay3_z_order:2;
		uint32_t leserve16_31:16;
	}b;
} lay_cfg_en_t;

typedef union cmp_irq_ctrl {
	uint32_t d32;
	struct {
		uint32_t reserve0_8:9;
		uint32_t eoc_msk:1;
		uint32_t reserve10_13:4;
		uint32_t soc_msk:1;
		uint32_t eow_msk:1;
		uint32_t reserve_16:1;
		uint32_t eod_msk:1;
		uint32_t reserve18_31:14;
	}b;
} cmp_irq_ctrl_t;

typedef union lay_size {
	uint32_t d32;
	struct {
		uint32_t width:12;
	        uint32_t reserve12_15:4;
	        uint32_t height:12;
	        uint32_t reserve28_31:4;
	}b;
} lay_size_t;

typedef union lay_cfg {
	uint32_t d32;
	struct {
		uint32_t g_alpha:8;
		uint32_t reserve8_9:2;
		uint32_t color:3;
		uint32_t g_alpha_en:1;
		uint32_t domain_multi:1;
		uint32_t reserve15:1;
		uint32_t format:4;
		uint32_t sharpl:2;
		uint32_t reserve22_31:10;
	}b;
} lay_cfg_t;

typedef union lay_scale {
	uint32_t d32;
	struct {
		uint32_t target_width:12;
	        uint32_t reserve12_15:4;
	        uint32_t target_height:12;
	        uint32_t reserve28_31:4;
	}b;
} lay_scale_t;

typedef union lay_rotation {
	uint32_t d32;
	struct {
		uint32_t rotation:3;
		uint32_t reserve3_31:29;
	}b;
} lay_rotation_t;

typedef union lay_pos {
	uint32_t d32;
	struct {
		uint32_t x_pos:12;
	        uint32_t reserve12_15:4;
	        uint32_t y_pos:12;
	        uint32_t reserve28_31:4;
	}b;
} lay_pos_t;

typedef union chain_cfg {
	uint32_t d32;
	struct {
		uint32_t chain_end:1;
		uint32_t change_2_cmp:1;
		uint32_t reserve2_15:14;
		uint32_t color:3;
		uint32_t format:4;
		uint32_t reserve23_31:9;
	}b;
} chain_cfg_t;

typedef union rdma_irq_ctrl {
	uint32_t d32;
	struct {
		uint32_t reserve_0:1;
		uint32_t eos_msk:1;
		uint32_t sos_msk:1;
		uint32_t reserve3_16:14;
		uint32_t eod_msk:1;
		uint32_t reserve18_31:4;
	}b;
} rdma_irq_ctrl_t;

struct ingenicfb_framedesc {
	uint32_t	   FrameNextCfgAddr;
	frm_size_t	   FrameSize;
	frm_ctrl_t	   FrameCtrl;
	uint32_t	   WritebackAddr;
	uint32_t	   WritebackStride;
	uint32_t	   Layer0CfgAddr;
	uint32_t	   Layer1CfgAddr;
	uint32_t 	   Layer2CfgAddr;
	uint32_t	   Layer3CfgAddr;
	lay_cfg_en_t       LayCfgEn;
	cmp_irq_ctrl_t	   InterruptControl;
};

struct ingenicfb_layerdesc {
	lay_size_t	LayerSize;
	lay_cfg_t	LayerCfg;
	uint32_t	LayerBufferAddr;
	lay_scale_t	LayerScale;
	lay_rotation_t	LayerRotation;
	uint32_t	LayerScratch;
	lay_pos_t	LayerPos;
	uint32_t	layerresizecoef_x;
	uint32_t	layerresizecoef_y;
	uint32_t	LayerStride;
	uint32_t        UVBufferAddr;
	uint32_t        UVStride;

};

struct ingenicfb_sreadesc {
	uint32_t	RdmaNextCfgAddr;
	uint32_t	FrameBufferAddr;
	uint32_t	Stride;
	chain_cfg_t	ChainCfg;
	rdma_irq_ctrl_t	InterruptControl;
};


#endif
