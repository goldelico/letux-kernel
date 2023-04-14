#ifndef __TIZIANO_ISP_H__
#define __TIZIANO_ISP_H__

typedef struct {
	unsigned int max_again;	//the format is .16
	unsigned int max_dgain;	//the format is .16
	unsigned int again;
	unsigned int dgain;
	unsigned int fps;
	unsigned short min_integration_time;
	unsigned short min_integration_time_native;
	unsigned short max_integration_time_native;
	unsigned short integration_time_limit;
	unsigned int integration_time;
	unsigned short total_width;
	unsigned short total_height;
	unsigned short max_integration_time;
	unsigned short integration_time_apply_delay;
	unsigned short again_apply_delay;
	unsigned short dgain_apply_delay;
	unsigned short one_line_expr_in_us;
} sensor_info_t;

typedef struct {
	int width;
	int height;
	int bayer;
	char sensor[16];
	sensor_info_t sensor_info;
} tisp_init_param_t;

typedef struct {
	int sensor_width;
	int sensor_height;
	unsigned int buf_ae_num;
	unsigned int buf_ae_vaddr;
	unsigned int buf_ae_paddr;
	unsigned int buf_aehist_num;
	unsigned int buf_aehist_vaddr;
	unsigned int buf_aehist_paddr;
	unsigned int buf_awb_num;
	unsigned int buf_awb_vaddr;
	unsigned int buf_awb_paddr;
	unsigned int buf_adr_num;
	unsigned int buf_adr_vaddr;
	unsigned int buf_adr_paddr;
	unsigned int buf_defog_num;
	unsigned int buf_defog_vaddr;
	unsigned int buf_defog_paddr;
	unsigned int buf_af_num;
	unsigned int buf_af_vaddr;
	unsigned int buf_af_paddr;

} tisp_info_t;


typedef struct {
	int scaler_en;
	int scaler_width;
	int scaler_height;
	int crop_en;
	int crop_x;
	int crop_y;
	int crop_width;
	int crop_height;
} tisp_channel_attr_t;



typedef struct {
	uint32_t tisp_wb_manual;
	uint32_t tisp_wb_rg;
	uint32_t tisp_wb_bg;
	uint32_t tisp_wb_rg_sta_global;
	uint32_t tisp_wb_bg_sta_global;
	uint32_t tisp_wb_rg_sta_weight;
	uint32_t tisp_wb_bg_sta_weight;
} tisp_wb_attr_t;

typedef struct {
	int ae_hist[256];
	int ae_hist_5bin[5];
	int ae_hist_nodes[4];
	int ae_hist_hv[2];
} tisp_ae_sta_t;

typedef struct {
	unsigned int af_metrics;
	unsigned int af_metrics_alt;
	unsigned char af_enable;
	unsigned char af_metrics_shift;
	unsigned short af_delta;
	unsigned short af_theta;
	unsigned short af_hilight_th;
	unsigned short af_alpha_alt;
	unsigned char  af_hstart;
	unsigned char  af_vstart;
	unsigned char  af_stat_nodeh;
	unsigned char  af_stat_nodev;
} tisp_af_attr;

//extern tisp_ae_ctrls_t tisp_ae_ctrls;
extern tisp_init_param_t sensor_info;
#endif
