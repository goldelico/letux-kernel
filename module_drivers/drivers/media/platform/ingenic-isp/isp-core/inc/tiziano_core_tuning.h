#ifndef __TIZIANO_TIZIANO_CORE_TUNING_H__
#define __TIZIANO_TIZIANO_CORE_TUNING_H__
#include "tiziano_isp.h"

extern int day_night;

typedef enum tisp_mode_day_and_night {
	TISP_MODE_DAY_MODE,
	TISP_MODE_NIGHT_MODE,
	TISP_MODE_BUTT,
} TISP_MODE_DN_E;

typedef struct tisp_custom_effect {
	unsigned char brightness;
	unsigned char sharpness;
	unsigned char contrast;
	unsigned char saturation;
	unsigned int brightness_ori_day[20];
	unsigned int brightness_ori_night[20];
	unsigned int saturation_ori_day[9];
	unsigned int sharpen_ori_day[36];
	unsigned int sharpen_ori_night[36];

} tisp_custom_effect_t;

typedef struct tisp_mdns_ratio {
	int dysad_thres_def_day[9];
	int dysta_thres_def_day[9];
	int dypbt_thres_def_day[9];
	int dywei_max_def_day[9];
	int dywei_min_def_day[9];
	int dysad_thres_def_night[9];
	int dysta_thres_def_night[9];
	int dypbt_thres_def_night[9];
	int dywei_max_def_night[9];
	int dywei_min_def_night[9];
} tisp_mdns_ratio_t;

typedef struct tisp_ncu_info {
	uint32_t width;
	uint32_t height;
	uint32_t sta_y_block_size;
	uint32_t sta_y_stride;
	uint32_t sta_y_buf_size;
} tisp_ncu_info_t;

typedef struct tisp_gamma_lut{
	unsigned int gamma[129];
} tisp_gamma_lut_t;

typedef struct tisp_3a_weight{
	unsigned int weight[225];
} tisp_3a_weight_t;

typedef struct tisp_zone_info{
	unsigned int zone[15][15];
} tisp_zone_info_t;

typedef struct tisp_ev_attr {
	unsigned int integration_time;
	unsigned int ev;
	unsigned int expr_us;
	unsigned int ev_log2;
	unsigned int again;
	unsigned int dgain;
	unsigned int gain_log2;
	unsigned int total_gain;
	unsigned int max_again;
	unsigned int max_isp_dgain;
	unsigned int sensor_dgain;
	unsigned int max_sensor_dgain;
	int manual_it;
	int manual_ag;
	int manual_isp_dgain;
} tisp_ev_attr_t;

typedef struct tisp_ae_ex_min {
	unsigned int min_it;
	unsigned int min_again;
} tisp_ae_ex_min_t;

typedef union tisp_module_control {
	unsigned int key;
	struct {
		unsigned int bitBypassDPC : 1; /* [0]  */
		unsigned int bitBypassGIB : 1; /* [1]  */
		unsigned int bitBypassLSC : 1; /* [2]  */
		unsigned int bitBypassAWB : 1; /* [3]  */
		unsigned int bitBypassADR : 1; /* [4]  */
		unsigned int bitBypassDMSC : 1; /* [5]  */
		unsigned int bitBypassCCM : 1; /* [6]  */
		unsigned int bitBypassGAMMA : 1; /* [7]  */
		unsigned int bitBypassDEFOG : 1; /* [8]  */
		unsigned int bitBypassCLM : 1; /* [9]  */
		unsigned int bitBypassYSHARPEN : 1; /* [10]  */
		unsigned int bitBypassMDNS : 1; /* [11]  */
		unsigned int bitBypassSDNS : 1; /* [12]  */
		unsigned int bitBypassHLDC : 1; /* [13]  */
		unsigned int bitBypassTP : 1; /* [14]  */
		unsigned int bitBypassFONT : 1; /* [15]  */
		unsigned int bitRsv : 15; /* [16 ~ 30]  */
		unsigned int bitRsv2 : 1; /* [31]  */
	};
} tisp_module_control_t;



typedef struct tisp_core_tuning {
	int day_night;
	int flicker_hz;
	void *core;	/*tisp_core_t in tiziano_core.h*/
} tisp_core_tuning_t;


int32_t tisp_day_or_night_s_ctrl(tisp_core_tuning_t *core_tuning, TISP_MODE_DN_E dn);
TISP_MODE_DN_E tisp_day_or_night_g_ctrl(tisp_core_tuning_t *core_tuning);
void tisp_mirror_enable(tisp_core_tuning_t *core_tuning, int en);
void tisp_flip_enable(tisp_core_tuning_t *core_tuning, int en);
int tisp_set_fps(tisp_core_tuning_t *core_tuning, int fps);

void tisp_set_brightness(tisp_core_tuning_t *core_tuning, unsigned char brightness);
void tisp_set_sharpness(tisp_core_tuning_t *core_tuning, unsigned char sharpness);
void tisp_set_saturation(tisp_core_tuning_t *core_tuning, unsigned char saturation);
void tisp_set_contrast(tisp_core_tuning_t *core_tuning, unsigned char contrast);

unsigned char tisp_get_brightness(tisp_core_tuning_t *core_tuning);
unsigned char tisp_get_sharpness(tisp_core_tuning_t *core_tuning);
unsigned char tisp_get_saturation(tisp_core_tuning_t *core_tuning);
unsigned char tisp_get_contrast(tisp_core_tuning_t *core_tuning);
void tisp_top_sel(tisp_core_tuning_t *core_tuning, int sel);
int tisp_g_ncuinfo(tisp_core_tuning_t *core_tuning, tisp_ncu_info_t *ncuinfo);
unsigned int tisp_top_read(tisp_core_tuning_t *core_tuning);
int tisp_s_antiflick(tisp_core_tuning_t *core_tuning, int hz);
int tisp_s_Hilightdepress(tisp_core_tuning_t *core_tuning, unsigned int strength);
int tisp_g_Hilightdepress(tisp_core_tuning_t *core_tuning, unsigned int *strength);
int tisp_s_Gamma(tisp_core_tuning_t *core_tuning, tisp_gamma_lut_t *gammas);
int tisp_g_Gamma(tisp_core_tuning_t *core_tuning, tisp_gamma_lut_t *gammag);
int tisp_s_aeroi_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * roi_weight);
int tisp_g_aeroi_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * roi_weight);
int tisp_s_aezone_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * zone_weight);
int tisp_g_aezone_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * zone_weight);
int tisp_g_ev_attr(tisp_core_tuning_t *core_tuning, tisp_ev_attr_t *ev_attr);
int tisp_g_wb_attr(tisp_core_tuning_t *core_tuning, tisp_wb_attr_t *wb_attr);
int tisp_s_wb_frz_attr(tisp_core_tuning_t *core_tuning, unsigned char frz);
int tisp_s_wb_attr(tisp_core_tuning_t *core_tuning, tisp_wb_attr_t wb_attr);
int tisp_g_ae_hist(tisp_core_tuning_t *core_tuning, tisp_ae_sta_t *ae_hist);
int tisp_s_ae_hist(tisp_core_tuning_t *core_tuning, tisp_ae_sta_t ae_hist);
int tisp_s_3dns_ratio(tisp_core_tuning_t *core_tuning, unsigned int ratio);
int tisp_s_ae_attr(tisp_core_tuning_t *core_tuning, tisp_ev_attr_t ae_attr);
int tisp_s_ae_attr_ag(tisp_core_tuning_t *core_tuning, tisp_ev_attr_t ae_attr);
int tisp_g_ae_attr(tisp_core_tuning_t *core_tuning, tisp_ev_attr_t *ae_attr);
int tisp_s_ae_min(tisp_core_tuning_t *core_tuning, tisp_ae_ex_min_t ae_min);
int tisp_g_ae_min(tisp_core_tuning_t *core_tuning, tisp_ae_ex_min_t *ae_min);
int tisp_g_ae_zone(tisp_core_tuning_t *core_tuning, tisp_zone_info_t *ae_zone);
void tisp_g_ae_luma(tisp_core_tuning_t *core_tuning, unsigned char *luma, int width, int height);
int tisp_g_af_metric(tisp_core_tuning_t *core_tuning, unsigned int *metric);
int tisp_g_af_attr(tisp_core_tuning_t *core_tuning, tisp_af_attr *af_info);
int tisp_s_af_attr(tisp_core_tuning_t *core_tuning, tisp_af_attr af_info);
int tisp_s_af_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * af_weight);
int tisp_g_af_weight(tisp_core_tuning_t *core_tuning, tisp_3a_weight_t * af_weight);
void tisp_s_wb_frz(tisp_core_tuning_t *core_tuning, unsigned char frz);
void tisp_g_wb_frz(tisp_core_tuning_t *core_tuning, unsigned char *frz);
void tisp_s_module_control(tisp_core_tuning_t *core_tuning, tisp_module_control_t top);
void tisp_g_module_control(tisp_core_tuning_t *core_tuning, tisp_module_control_t *top);
void tisp_s_ev_start(tisp_core_tuning_t *core_tuning, unsigned int ev_start);
void tisp_s_max_again(tisp_core_tuning_t *core_tuning, unsigned int max_again);
void tisp_s_max_isp_dgain(tisp_core_tuning_t *core_tuning, unsigned int max_isp_dgain);
int tisp_g_drc_strength(tisp_core_tuning_t *core_tuning, unsigned int *strength);
int tisp_s_drc_strength(tisp_core_tuning_t *core_tuning, unsigned int strength);
#endif
