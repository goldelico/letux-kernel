#ifndef __TIZIANO_ISP_CORE_API_H__
#define __TIZIANO_ISP_CORE_API_H__

#include "system_sensor_drv.h"
#include "tiziano_core_tuning.h"
#include "tiziano_isp.h"
#include "tiziano_sys.h"
#include "tiziano_priv.h"

/*
   -----------core---------
   | awb, gamma,

*/

/* --------- defog ---------*/

typedef struct Tiziano_Isp_Defog_Ct_Detect_Para
{
  uint32_t *DefogBlockRStat        ;
  uint32_t *DefogBlockGStat        ;
  uint32_t *DefogBlockBStat        ;
  uint32_t *DefogBlockYStat        ;
  uint32_t *DefogBlockDetailStat   ;
  uint32_t *DefogBlockVarianceStat ;
  uint32_t *DefogDefogBlockTX      ;
  uint32_t *DefogDefogBlockTY      ;
  uint32_t *DefogDetailX           ;
  uint32_t *DefogDetailY           ;
  uint32_t *DefogColorVarX         ;
  uint32_t *DefogColorVarY         ;
  uint32_t *DefogFpgaPara          ;
  uint32_t *DefogBlockTransmitT    ;
  uint32_t *DefogBlockAirLightR    ;
  uint32_t *DefogBlockAirLightG    ;
  uint32_t *DefogBlockAirLightB    ;
}Isp_Defog_Ct_Detect_Para;

typedef struct tisp_defog {
	uint32_t ev_now;
	uint32_t ev_changed;

	uint32_t defog_ev_list[9];
	uint32_t defog_trsy0_list[9];
	uint32_t defog_trsy1_list[9];
	uint32_t defog_trsy2_list[9];
	uint32_t defog_trsy3_list[9];
	uint32_t defog_trsy4_list[9];
	uint32_t defog_rgbra_list[9];
	uint32_t defog_meam_block_r[200];
	uint32_t defog_meam_block_g[200];
	uint32_t defog_meam_block_b[200];
	uint32_t defog_meam_block_y[200];
	uint32_t defog_variance_block[200];
	uint32_t defog_detial_block[200];
	uint32_t defog_defog_block_t_x[5];
	uint32_t defog_defog_block_t_y[5];
	uint32_t defog_detail_x[5];
	uint32_t defog_detail_y[5];
	uint32_t defog_color_var_x[5];
	uint32_t defog_color_var_y[5];
	uint32_t defog_fpga_para[8];
	uint32_t param_defog_weightlut20[32];
	uint32_t param_defog_weightlut02[32];
	uint32_t param_defog_weightlut12[32];
	uint32_t param_defog_weightlut22[32];
	uint32_t param_defog_weightlut21[32];
	uint32_t param_defog_ct_par_array[4];
	uint32_t param_defog_dark_ct_array[14];
	uint32_t param_defog_over_expo_w_array[11];
	uint32_t param_defog_dark_spe_w_array[11];
	uint32_t param_defog_col_ct_array[14];
	uint32_t param_defog_cent3_w_dis_array[24];
	uint32_t param_defog_cent5_w_dis_array[31];
	uint32_t param_defog_blk_trans_array[10];
	uint32_t param_defog_detail_ct_array[10];
	uint32_t param_defog_col_var_array[10];
	uint32_t param_defog_soft_ct_par_array[6];

	uint32_t defog_block_transmit_t[200];
	uint32_t defog_block_air_light_r[200];
	uint32_t defog_block_air_light_g[200];
	uint32_t defog_block_air_light_b[200];

	Isp_Defog_Ct_Detect_Para TizianoDefogStructMe;

	void *core;
} tisp_defog_t;


/* -------- adr -------- */
typedef struct Tiziano_Isp_Adr_Ct_Detect_Para
{
	int *CtcKneepointX;
	int *CtcKneepointMux;
	int *MinKneepointX;
	int *MinKneepointY;
	int *MapKneepointX;
	int *MapKneepointY;
	int *ContrastWDistance;
	int *AdrHist;
	int *AdrBlockY;
	int *AdrBlockHist;
	int *TmBaseLut;
	int *AdrGamX;
	int *AdrGamY;
	int *AdrMapMode;
	int *AdrLightEnd;
	int *AdrBlockLight;
}Isp_Adr_Ct_Detect_Para;

typedef struct tisp_adr {

	unsigned int ev_changed;
	unsigned int ev_now;
	unsigned int width_def;
	unsigned int height_def;
	unsigned int adr_ratio;
	uint32_t adr_ev_list[9];
	uint32_t adr_ligb_list[9];
	uint32_t adr_mapb1_list[9];
	uint32_t adr_mapb2_list[9];
	uint32_t adr_mapb3_list[9];
	uint32_t adr_mapb4_list[9];
	uint32_t adr_blp2_list[9];

	Isp_Adr_Ct_Detect_Para TizianoAdrFpgaStructMe;

	uint32_t param_adr_ct_par_array[8];
	uint32_t param_adr_weight_20_lut_array[32];
	uint32_t param_adr_weight_02_lut_array[32];
	uint32_t param_adr_weight_12_lut_array[32];
	uint32_t param_adr_weight_22_lut_array[32];
	uint32_t param_adr_weight_21_lut_array[32];
	uint32_t param_adr_ctc_kneepoint_array[8];
	uint32_t param_adr_min_kneepoint_array[23];
	uint32_t param_adr_map_kneepoint_array[23];
	uint32_t param_adr_coc_kneepoint_array[21];
	uint32_t param_adr_centre_w_dis_array[31];
	uint32_t param_adr_contrast_w_dis_array[32];
	uint32_t param_adr_stat_block_hist_diff_array[4];

	uint32_t adr_block_y[20];
	uint32_t adr_block_hist[100];
	uint32_t adr_hist[512];

	uint32_t ctc_kneepoint_x[4];
	uint32_t ctc_kneepoint_mux[4];
	uint32_t min_kneepoint_x[11];
	uint32_t min_kneepoint_y[11];
	/* {0,2,4,6,8,30,60,70,80,100,120}; */
	uint32_t min_kneepoint_pow[12];
	uint32_t map_kneepoint_x[11];
	uint32_t map_kneepoint_y[220];
	uint32_t map_kneepoint_pow[12];
	uint32_t coc_kneepoint_x[11];
	uint32_t coc_kneepoint_pow[10];
	uint32_t centre_w_distance[31];
	uint32_t contrast_w_distance[32];

	uint32_t adr_stat_block_hist_diff[4];
	uint32_t adr_tm_base_lut[9];
	uint32_t adr_gam_x[129];
	uint32_t adr_gam_y[129];
	uint32_t adr_light_end[29];
	uint32_t adr_block_light[15];
	uint32_t adr_map_mode[11];
	uint32_t histSub_4096[9];
	uint32_t histSub_4096_out[9];
	uint32_t histSub_4096_diff[8];






	void *core;
} tisp_adr_t;

/*ae struct*/
typedef struct {
	uint32_t tisp_ae_manual;
	uint32_t tisp_ae_sensor_again;
	uint32_t tisp_ae_sensor_dgain;
	uint32_t tisp_ae_sensor_integration_time;
	uint32_t tisp_ae_isp_dgian;
	uint32_t tisp_ae_max_sensor_again;
	uint32_t tisp_ae_max_sensor_dgain;
	uint32_t tisp_ae_max_sensor_integration_time;
	uint32_t tisp_ae_max_isp_dgain;
	uint32_t tisp_ae_ev;
	uint32_t tisp_ae_tgain_db;
	uint32_t tisp_ae_again_db;
	uint32_t tisp_ae_ir;
	uint32_t tisp_ae_it_manual;
	uint32_t tisp_ae_ag_manual;
	uint32_t tisp_ae_dg_manual;
} tisp_ae_ctrls_t;

typedef struct {
	uint32_t tisp_ae_sensor_agian;
	uint32_t tisp_ae_sensor_dgian;
	uint32_t tisp_ae_sensor_integration_time;
	uint32_t tisp_ae_isp_dgian;
} tisp_ae_ctrls_internal_t;

typedef struct Tiziano_Isp_Ae_Wmean_Param {
	uint32_t *a_ae_array_d;
	uint32_t *a_ae_array_m;
	uint32_t *a_ae_array_s;
	uint32_t *a_ae_array_ir;
	uint32_t *a_ae_array_dc;
	uint32_t *a_ae_array_sc;
	uint32_t *a_ae_parameter;
	uint32_t *a_ae_zone_weight;
	uint32_t *a_exp_parameter;
	uint32_t *a_ae_stat;
	uint32_t *a_scene_roui_weight;
	uint32_t *a_scene_roi_weight;
	uint32_t *a_log2_lut;
	uint32_t *a_weight_lut;
	uint32_t *a_AePointPos;
} Isp_Ae_Wmean_Param;

typedef struct Tiziano_Isp_Ae_Tune_Param {
	uint32_t *a_exp_parameter;
	uint32_t *a_ev_list;
	uint32_t *a_lum_list;
	uint32_t *a_at_list;
	uint32_t *a_ae_result;
	uint32_t *a_ae_reg;
	uint32_t *a_ae_stat;
	uint32_t *a_ae_wm_q;
	uint32_t *a_flicker_t;
	uint32_t *a_deflicker_para;
	uint32_t *a_ae_ev_step;
	uint32_t *a_ae_scene_mode_th;
	uint32_t *a_ae_stable_tol;
	uint32_t *a_AePointPos;
	uint32_t *a_ae_hist_ir_array;
	uint32_t *a_ae_nodes_num;
	uint32_t *a_ae_compensation;
} Isp_Ae_Tune_Param;

typedef struct tisp_ae {
	int ftune;
	uint32_t min_it;
	uint32_t min_ag;
	uint32_t min_dg;
	uint32_t y_zone[15][15];
	uint32_t y_zone_last[15][15];
	spinlock_t slock;
	spinlock_t slock_hist;
	spinlock_t aelock;
	tisp_ae_sta_t tisp_ae_hist;
	tisp_ae_sta_t tisp_ae_hist_last;
	int ae_first;
	int expt_first_frame;

	Isp_Ae_Wmean_Param IspAeWmeanParam;
	Isp_Ae_Tune_Param IspAeTuneParam;
	tisp_ae_ctrls_t tisp_ae_ctrls;
	tisp_ae_ctrls_internal_t ae_ctrls;

	uint32_t ae_array_d[225];
	uint32_t ae_array_m[225];
	uint32_t ae_array_s[225];
	uint32_t ae_array_ir[225];
	uint32_t ae_array_dc[225];
	uint32_t ae_array_sc[225];

	uint32_t ae_hist_array[256];
	uint32_t ae_hist_ir_array[256];
	uint32_t _ae_reg[4]; //--->awb
	uint32_t _ae_ev;
	uint32_t ae_dn_refresh_flag;
	uint32_t ae_compensation;
	uint32_t ae_ev_init_strict;
	uint32_t ae_ev_init_en;

	/*parameter*/
	uint32_t _ae_parameter[38];
	uint32_t ae_switch_night_mode[4];
	uint32_t _AePointPos[2];
	uint32_t _exp_parameter[11];
	uint32_t ae_ev_step[5];
	uint32_t ae_stable_tol[4];
	uint32_t _ev_list[10];
	uint32_t _lum_list[6];
	uint32_t _at_list[10];
	uint32_t _deflicker_para[3];
	uint32_t _flicker_t[6];
	uint32_t _deflick_lut[120];
	uint32_t _nodes_num;
	uint32_t _scene_para[11];
	uint32_t ae_scene_mode_th[4];
	uint32_t _log2_lut[20];
	uint32_t _weight_lut[20];
	uint32_t _ae_zone_weight[225];
	uint32_t _scene_roui_weight[225];
	uint32_t _scene_roi_weight[225];
	uint32_t ev_cache[10];
	uint32_t ad_cache[10];
	uint32_t ag_cache[9];
	uint32_t dg_cache[9];

	/*find ones*/
	uint32_t _ae_result[4];
	uint32_t _ae_stat[5];
	uint32_t _ae_wm_q[15];

	int trig;
	int force_trig;
	int trig_deflick;
	int trig_cal;

	uint32_t again_old;
	uint32_t again_new;
	uint32_t total_gain_old;
	uint32_t total_gain_new;
	uint32_t ag_old;
	uint32_t dg_old;
	uint32_t ag_new;
	uint32_t dg_new;
	uint32_t EffectFrame;
	int32_t EffectCount;

	void *core;
} tisp_ae_t;

/*awb struct*/
typedef struct Tiziano_Isp_Awb_Ct_Detect_Param {
	unsigned int *auint16LightSrc;
	unsigned int uint16LightSrcNum;
	unsigned int *auint32CtTh;
	unsigned int *auint32CtThPara;
	unsigned int *auint16RgBgWeight;
	unsigned int uint32HorZone;
	unsigned int uint32VerZone;
	unsigned int *auint16RgPos;
	unsigned int *auint16BgPos;
	unsigned int *auint16ColorTempMesh;
	unsigned int *auint16AwbWght;
	uint32_t *auint64AwbDisTw;
	unsigned int *uint32AwbCt;
	unsigned int *auint32LsWLut;
	unsigned int *auint32AwbPointPos;
} ISP_AWB_CT_DETECT_PARAM;

typedef struct Tiziano_Isp_Awb_Fpga_Param
{
	unsigned int *a_r_sum;
	unsigned int *a_g_sum;
	unsigned int *a_b_sum;
	unsigned int *a_pix_cnt;
	unsigned int *a_awb_cof;
	unsigned int *a_awb_mf_para;
	unsigned int *a_awb_parameter;
	unsigned int  pixel_cnt_th;
	uint32_t *a_awb_static;
	unsigned int *a_AwbPointPos;
} ISP_AWB_FPGA_PARAM;

typedef struct tisp_awb {
	tisp_wb_attr_t tisp_wb_attr;
	int awb_moa;
	int awb_frz;
	int awb_first;
	uint32_t awb_dn_refresh_flag;
	ISP_AWB_FPGA_PARAM IspAwbFpgaParam;
	ISP_AWB_CT_DETECT_PARAM IspAwbCtDetectParam;
	unsigned int awb_array_r[225];
	unsigned int awb_array_g[225];
	unsigned int awb_array_b[225];
	unsigned int awb_array_p[225];

	unsigned int awb_rg_global;
	unsigned int awb_bg_global;
	unsigned int awb_pix_cnt[2];

	uint32_t zone_rgbg_last[450];
	uint32_t zone_rgbg[450];
	unsigned int zone_pix_cnt[225];
	unsigned int awb_gain_original[2];
	unsigned int first_frame;
	int offet_thres;

	/*parameter*/
	unsigned int _awb_parameter[45];
	unsigned int _pixel_cnt_th;
	unsigned int _awb_lowlight_rg_th[2];
	unsigned int _AwbPointPos[2];
	unsigned int _awb_cof[2];
	unsigned int _awb_mf_para[6];
	unsigned int _awb_mode[3];
	// ce_detcet
	unsigned int _awb_ct;
	unsigned int _awb_ct_last;
	uint32_t  _wb_static[2];
	unsigned int _light_src[20];
	unsigned int _light_src_num;
	unsigned int _rg_pos[15];
	unsigned int _bg_pos[15];
	unsigned int _awb_ct_th_ot_luxhigh[4];
	unsigned int _awb_ct_th_ot_luxlow[4];
	unsigned int _awb_ct_th_in[4];
	unsigned int _awb_ct_para_ot[2];
	unsigned int _awb_ct_para_in[2];
	uint32_t _awb_dis_tw[3];
	unsigned int _rgbg_weight[225];
	unsigned int _color_temp_mesh[225];
	unsigned int _awb_wght[225];
	unsigned int _rgbg_weight_ot[225];
	unsigned int _ls_w_lut[514];
	uint32_t _ev;

	void *core;			/*pointer to isp core top.*/
} tisp_awb_t;

typedef struct tisp_gamma {
	u32 tiziano_gamma_lut[129];
	void *core;			/*pinter to isp core.*/
} tisp_gamma_t;

/* ------ af ------- */

typedef struct TizianoIsp_Af_Static_Param {
	unsigned int *a_af_zone;
	unsigned int *a_af_array_fird0;
	unsigned int *a_af_array_fird1;
	unsigned int *a_af_array_iird0;
	unsigned int *a_af_array_iird1;
	unsigned int *a_af_array_y_sum;
	unsigned int *a_af_array_high_luma_cnt;
	unsigned int *a_af_weight;
	unsigned int *a_af_fv;
	unsigned int *a_af_fvwmean;
	uint32_t *a_af_tilt;
	uint32_t *a_AfPointPos;
	uint32_t width;
	uint32_t height;
} Isp_Af_Static_Param;

typedef struct tisp_af {
	Isp_Af_Static_Param IspAfStaticParam;
	uint32_t af_array_fird0[225];
	uint32_t af_array_fird1[225];
	uint32_t af_array_iird0[225];
	uint32_t af_array_iird1[225];
	uint32_t af_array_y_sum[225];
	uint32_t af_array_high_luma_cnt[225];
	unsigned int stAFParam_Zone[36];
	unsigned int stAFParam_ThresEnable[13];
	unsigned int stAFParam_FIR0_V[5];
	unsigned int stAFParam_FIR0_Ldg[8];
	unsigned int stAFParam_FIR0_Coring[4];
	unsigned int stAFParam_FIR1_V[5];
	unsigned int stAFParam_FIR1_Ldg[8];
	unsigned int stAFParam_FIR1_Coring[4];
	unsigned int stAFParam_IIR0_H[10];
	unsigned int stAFParam_IIR0_Ldg[8];
	unsigned int stAFParam_IIR0_Coring[4];
	unsigned int stAFParam_IIR1_H[10];
	unsigned int stAFParam_IIR1_Ldg[8];
	unsigned int stAFParam_IIR1_Coring[4];

	/******Algorithm Param********/
	uint32_t AFParam_PointPos[2];
	uint32_t AFParam_Fv_Alt;
	/* Alpha,Belta,Delta,Theta,BlendShift */
	uint32_t AFParam_Tilt[5]; //[0 ~ 1] * 64 6bit point float

	unsigned int AFParam_FvWmean[15];
	unsigned int AFParam_Fv[3];
	/* Fv1-->Alpha*h1+(1-Alpha)*v1 */
	/* Fv2-->Belta*h2+(1-Belta)*v2 */
	/* Fv -->Delta*Fv1+Theta*Fv2 */

	unsigned int AFWeight_Param[225];

	unsigned int FvWmean_num;

	tisp_af_attr af_attr;
	unsigned char af_set_trig;


	void *core;
} tisp_af_t;

typedef struct tisp_gib {
	uint32_t trig_set_deir;
	/*parameter*/
	uint32_t tiziano_gib_config_line[6];
	uint32_t tiziano_gib_r_g_linear[2];
	uint32_t tiziano_gib_b_ir_linear[2];

	uint32_t tiziano_gib_deirm_blc_r_linear[9];
	uint32_t tiziano_gib_deirm_blc_gr_linear[9];
	uint32_t tiziano_gib_deirm_blc_gb_linear[9];
	uint32_t tiziano_gib_deirm_blc_b_linear[9];
	uint32_t tiziano_gib_deirm_blc_ir_linear[9];

	uint32_t gib_ir_value[2];
	uint32_t gib_ir_point[4];
	uint32_t gib_ir_reser[15];
	uint32_t tiziano_gib_deir_r_h[33];
	uint32_t tiziano_gib_deir_g_h[33];
	uint32_t tiziano_gib_deir_b_h[33];

	uint32_t tiziano_gib_deir_r_m[33];
	uint32_t tiziano_gib_deir_g_m[33];

	uint32_t tiziano_gib_deir_b_m[33];

	uint32_t tiziano_gib_deir_r_l[33];
	uint32_t tiziano_gib_deir_g_l[33];
	uint32_t tiziano_gib_deir_b_l[33];

	uint32_t tiziano_gib_deir_matrix_h[15];
	uint32_t tiziano_gib_deir_matrix_m[15];
	uint32_t tiziano_gib_deir_matrix_l[15];
	void *core;
} tisp_gib_t;

typedef struct tisp_lsc {
	uint32_t lut_num;
	uint32_t mesh_scale;
	uint32_t lut_stride;
	uint32_t mesh_size[2];
	uint32_t a_linear[2047];
	uint32_t t_linear[2047];
	uint32_t d_linear[2047];
	uint32_t mesh_lsc_str[9];
	uint32_t lsc_change_flag;
	uint32_t lsc_change_flag_last;
	uint32_t tiziano_lsc_ct_points[4];
	uint32_t ct;
	uint32_t ct_last;
	uint32_t lsc_gain_thres;
	uint32_t lsc_gain_old;
	uint32_t gain_no_change;
	void *core;
} tisp_lsc_t;


/*ccm struct*/
struct tisp_ccm_real {
	uint32_t first_tune;
	uint32_t ev_old;
	uint32_t ev_threshold;
	uint32_t ct_old;
	uint32_t ct_threshold;
	uint32_t cm_sat;
};
typedef struct tisp_ccm {

	uint32_t tiziano_ccm_dp_cfg[1];
	uint32_t tiziano_ccm_a_linear[9];
	uint32_t tiziano_ccm_t_linear[9];
	uint32_t tiziano_ccm_d_linear[9];
	uint32_t tiziano_linear_value;
	int32_t ccm_parameter[9];

	uint32_t _ev;
	uint32_t _ct;
	struct tisp_ccm_real ccm_real;

	int32_t _ccm_a_parameter[9];
	int32_t _ccm_t_parameter[9];
	int32_t _ccm_d_parameter[9];
	uint32_t cm_ev_list[9];
	uint32_t cm_sat_list[9];
	uint32_t cm_awb_list[2];
	void *core;
} tisp_ccm_t;

typedef struct tisp_dmsc {
	int32_t dmsc_uu_np_array[16];
	int32_t dmsc_sp_d_sigma_3_np_array[16];
	int32_t dmsc_sp_d_w_wei_np_array[16];
	int32_t dmsc_sp_d_b_wei_np_array[16];
	int32_t dmsc_sp_ud_w_wei_np_array[16];
	int32_t dmsc_sp_ud_b_wei_np_array[16];
	/* out option */
	int32_t dmsc_out_opt;
	/* direction(hv aa hvaa value) tuning parameters */
	int32_t dmsc_hv_thres_1_array[9];
	int32_t dmsc_hv_stren_array[9];
	int32_t dmsc_aa_thres_1_array[9];
	int32_t dmsc_aa_stren_array[9];
	int32_t dmsc_hvaa_thres_1_array[9];
	int32_t dmsc_hvaa_stren_array[9];
	int32_t dmsc_dir_par_array[9];
	/* std and uu value tuning parameters */
	int32_t dmsc_uu_thres_array[9];
	int32_t dmsc_uu_stren_array[9];
	int32_t dmsc_uu_par_array[3];
	/* alias tuning parameters */
	int32_t dmsc_alias_stren_array[9];
	int32_t dmsc_alias_thres_1_array[9];
	int32_t dmsc_alias_thres_2_array[9];
	int32_t dmsc_alias_dir_thres_array[9];
	int32_t dmsc_alias_par_array[4];
	/* rgb nor blur wei tuning parameters */
	int32_t dmsc_nor_alias_thres_array[9];
	int32_t dmsc_nor_par_array[4];
	/* d sharpen tuning parameters */
	int32_t dmsc_sp_d_w_stren_array[9];
	int32_t dmsc_sp_d_b_stren_array[9];
	int32_t dmsc_sp_d_brig_thres_array[9];
	int32_t dmsc_sp_d_dark_thres_array[9];
	int32_t dmsc_sp_d_par_array[8];
	/* ud sharpen tuning parameters */
	int32_t dmsc_sp_ud_w_stren_array[9];
	int32_t dmsc_sp_ud_b_stren_array[9];
	int32_t dmsc_sp_ud_brig_thres_array[9];
	int32_t dmsc_sp_ud_dark_thres_array[9];
	int32_t dmsc_sp_ud_par_array[8];
	/* sp wei tuning parameters */
	int32_t dmsc_sp_alias_thres_array[9];
	int32_t dmsc_sp_alias_par_array[2];
	/* alias rgb tuning parameters */
	int32_t dmsc_rgb_dir_thres_array[9];
	int32_t dmsc_rgb_alias_stren_array[9];
	int32_t dmsc_rgb_alias_par_array[2];
	/* fc tuning parameters */
	int32_t dmsc_fc_alias_stren_array[9];
	int32_t dmsc_fc_t1_thres_array[9];
	int32_t dmsc_fc_t1_stren_array[9];
	int32_t dmsc_fc_t2_stren_array[9];
	int32_t dmsc_fc_t3_stren_array[9];
	int32_t dmsc_fc_par_array[9];

	uint32_t dmsc_hv_thres_1_intp;
	uint32_t dmsc_hv_stren_intp;
	uint32_t dmsc_aa_thres_1_intp;
	uint32_t dmsc_aa_stren_intp;
	uint32_t dmsc_hvaa_thres_1_intp;
	uint32_t dmsc_hvaa_stren_intp;
	uint32_t dmsc_uu_thres_intp;
	uint32_t dmsc_uu_stren_intp;
	uint32_t dmsc_alias_stren_intp;
	uint32_t dmsc_alias_thres_1_intp;
	uint32_t dmsc_alias_thres_2_intp;
	uint32_t dmsc_alias_dir_thres_intp;
	uint32_t dmsc_nor_alias_thres_intp;
	uint32_t dmsc_sp_d_w_stren_intp;
	uint32_t dmsc_sp_d_b_stren_intp;
	uint32_t dmsc_sp_d_brig_thres_intp;
	uint32_t dmsc_sp_d_dark_thres_intp;
	uint32_t dmsc_sp_ud_w_stren_intp;
	uint32_t dmsc_sp_ud_b_stren_intp;
	uint32_t dmsc_sp_ud_brig_thres_intp;
	uint32_t dmsc_sp_ud_dark_thres_intp;
	uint32_t dmsc_sp_alias_thres_intp;
	uint32_t dmsc_rgb_dir_thres_intp;
	uint32_t dmsc_rgb_alias_stren_intp;
	uint32_t dmsc_fc_alias_stren_intp;
	uint32_t dmsc_fc_t1_thres_intp;
	uint32_t dmsc_fc_t1_stren_intp;
	uint32_t dmsc_fc_t2_stren_intp;
	uint32_t dmsc_fc_t3_stren_intp;

	uint32_t gain_old;
	uint32_t gain_thres;
	uint32_t shadow_en;

	void *core;
} tisp_dmsc_t;

typedef struct tisp_sharpen {
	int32_t sharpen_uu_np_array[16];    //8 bits  0~255
	int32_t sharpen_v1_sigma_np_array[16];    //5 bits  0~16
	int32_t sharpen_w_wei_np_array[16];    //6 bits  0~32
	int32_t sharpen_b_wei_np_array[16];    //6 bits  0~32
	int32_t sharpen_uu_thres_array[9];
	int32_t sharpen_uu_stren_array[9];
	int32_t sharpen_uu_par_array[3];
	int32_t sharpen_pixel_thres_array[9];
	int32_t sharpen_w_stren_array[9];
	int32_t sharpen_b_stren_array[9];
	int32_t sharpen_brig_thres_array[9];
	int32_t sharpen_dark_thres_array[9];
	int32_t sharpen_con_par_array[8];

	/* sharpen intp parameters */
	uint32_t sharpen_uu_thres_intp;
	uint32_t sharpen_uu_stren_intp;
	uint32_t sharpen_pixel_thres_intp;
	uint32_t sharpen_w_stren_intp;
	uint32_t sharpen_b_stren_intp;
	uint32_t sharpen_birg_thres_intp;
	uint32_t sharpen_dark_thres_intp;

	uint32_t gain_old;
	uint32_t gain_thres;
	uint32_t shadow_en;
	void *core;
} tisp_sharpen_t;

typedef struct tisp_sdns {
	int32_t sdns_top_func_array[2];
	int32_t sdns_y_dtl_thres_array[9];
	int32_t sdns_y_fus_slope_array[9];
	int32_t sdns_y_lum_divop_array[9];
	int32_t sdns_y_dtl_segop_array[9];
	int32_t sdns_y_fus_segop_array[9];
	int32_t sdns_y_lum_segop_array[9];
	int32_t sdns_y_dsp_segop_array[9];
	int32_t sdns_y_bsp_segop_array[9];
	int32_t sdns_y_dtl_stren_array[9];
	int32_t sdns_y_fus_stren_array[9];
	int32_t sdns_y_lum_stren_array[9];
	int32_t sdns_y_dsp_stren_array[9];
	int32_t sdns_y_bsp_stren_array[9];
	int32_t sdns_y_dtl_npv_0_array[9];
	int32_t sdns_y_dtl_npv_1_array[9];
	int32_t sdns_y_dtl_npv_2_array[9];
	int32_t sdns_y_dtl_npv_3_array[9];
	int32_t sdns_y_dtl_npv_4_array[9];
	int32_t sdns_y_fus_npv_array[16];
	int32_t sdns_y_lum_npv_array[16];
	int32_t sdns_y_dsp_npv_array[16];
	int32_t sdns_y_bsp_npv_array[16];
	int32_t sdns_y_bil_stren_array[9];
	int32_t sdns_y_bil_npv_array[15];
	int32_t sdns_c_bas_wei_array[9];
	int32_t sdns_c_fus_mod_array[9];
	int32_t sdns_c_flu_cal_array[9];
	int32_t sdns_c_flu_stren_array[9];
	int32_t sdns_c_flu_npv_array[16];

	/* sdns intp parameters */
	uint32_t sdns_y_dtl_thres_intp;
	uint32_t sdns_y_fus_slope_intp;
	uint32_t sdns_y_lum_divop_intp;
	uint32_t sdns_y_dtl_segop_intp;
	uint32_t sdns_y_fus_segop_intp;
	uint32_t sdns_y_lum_segop_intp;
	uint32_t sdns_y_dsp_segop_intp;
	uint32_t sdns_y_bsp_segop_intp;
	uint32_t sdns_y_dtl_stren_intp;
	uint32_t sdns_y_fus_stren_intp;
	uint32_t sdns_y_lum_stren_intp;
	uint32_t sdns_y_dsp_stren_intp;
	uint32_t sdns_y_bsp_stren_intp;
	uint32_t sdns_y_bil_stren_intp;
	uint32_t sdns_y_dtl_npv_0_intp;
	uint32_t sdns_y_dtl_npv_1_intp;
	uint32_t sdns_y_dtl_npv_2_intp;
	uint32_t sdns_y_dtl_npv_3_intp;
	uint32_t sdns_y_dtl_npv_4_intp;
	uint32_t sdns_c_bas_wei_intp;
	uint32_t sdns_c_fus_mod_intp;
	uint32_t sdns_c_flu_cal_intp;
	uint32_t sdns_c_flu_stren_intp;

	uint32_t gain_old;
	uint32_t gain_thres;
	void *core;
} tisp_sdns_t;

/* ----------- dpc ------------*/
typedef struct tisp_dpc {
	/* dpc parameters */
	int32_t ctr_md_np_array[16];
	int32_t rdns_uu_np_array[16];
	int32_t rdns_g_lum_np_array[16];
	int32_t rdns_g_std_np_array[16];
	int32_t rdns_rb_lum_np_array[16];
	int32_t rdns_rb_std_np_array[16];
	int32_t dpc_s_text_thres_array[9];
	int32_t dpc_s_con_par_array[5];
	int32_t dpc_d_m1_level_array[9];
	int32_t dpc_d_m1_l0_fthres_array[9];
	int32_t dpc_d_m1_l0_dthres_array[9];
	int32_t dpc_d_m1_l1_hthres_array[9];
	int32_t dpc_d_m1_l1_lthres_array[9];
	int32_t dpc_d_m1_l1_d1_thres_array[9];
	int32_t dpc_d_m1_l1_d2_thres_array[9];
	int32_t dpc_d_m1_con_par_array[14];
	int32_t dpc_d_m2_level_array[9];
	int32_t dpc_d_m2_l0_thres_array[9];
	int32_t dpc_d_m2_l1_ldthres_array[9];
	int32_t dpc_d_m2_l1_pdthres_array[9];
	int32_t dpc_d_m2_con_par_array[6];
	int32_t ctr_stren_array[9];
	int32_t ctr_md_thres_array[9];
	int32_t ctr_el_thres_array[9];
	int32_t ctr_eh_thres_array[9];
	int32_t ctr_con_par_array[5];
	int32_t rdns_stren_array[9];
	int32_t rdns_std_thres_array[9];
	int32_t rdns_y_fthres_array[9];
	int32_t rdns_y_tthres_array[9];
	int32_t rdns_uv_fthres_array[9];
	int32_t rdns_uv_tthres_array[9];
	int32_t rdns_con_par_array[10];
	int32_t dpc_s_text_thres_intp;
	int32_t dpc_d_m1_level_intp;
	int32_t dpc_d_m1_l0_fthres_intp;
	int32_t dpc_d_m1_l0_dthres_intp;
	int32_t dpc_d_m1_l1_hthres_intp;
	int32_t dpc_d_m1_l1_lthres_intp;
	int32_t dpc_d_m1_l1_d1_thres_intp;
	int32_t dpc_d_m1_l1_d2_thres_intp;
	int32_t dpc_d_m2_level_intp;
	int32_t dpc_d_m2_l0_thres_intp;
	int32_t dpc_d_m2_l1_ldthres_intp;
	int32_t dpc_d_m2_l1_pdthres_intp;
	int32_t ctr_stren_intp;
	int32_t ctr_md_thres_intp;
	int32_t ctr_el_thres_intp;
	int32_t ctr_eh_thres_intp;
	int32_t rdns_stren_intp;
	int32_t rdns_std_thres_intp;
	int32_t rdns_y_fthres_intp;
	int32_t rdns_y_tthres_intp;
	int32_t rdns_uv_fthres_intp;
	int32_t rdns_uv_tthres_intp;
	void *core;
} tisp_dpc_t;

/* --------- mdns --------------*/

typedef struct tisp_mdns {
	int vin_width;
	int vin_height;

	/* mdns parameters */
	int32_t mdns_top_func_array[24];
	int32_t mdns_sta_size_array[9];
	int32_t mdns_pbt_size_array[9];
	int32_t mdns_pbt_ponit_array[2];
	int32_t mdns_y_sad_thres_array[9];
	int32_t mdns_y_sta_thres_array[9];
	int32_t mdns_y_pbt_thres_array[9];
	int32_t mdns_y_sad_stren_array[9];
	int32_t mdns_y_sta_stren_array[9];
	int32_t mdns_y_pbt_stren_array[9];
	int32_t mdns_y_sad_win_opt_array[9];
	int32_t mdns_y_sta_win_opt_array[9];
	int32_t mdns_y_pbt_win_opt_array[9];
	int32_t mdns_y_sad_win_wei_array[9];
	int32_t mdns_y_sta_win_wei_array[9];
	int32_t mdns_y_pbt_win_wei_array[9];
	int32_t mdns_y_sta_mv_num_array[9];
	int32_t mdns_y_pbt_mv_num_array[9];
	int32_t mdns_y_sta_mx_num_array[9];
	int32_t mdns_y_pbt_mx_num_array[9];
	int32_t mdns_y_sad_npv_array[4];
	int32_t mdns_y_sta_npv_array[32];
	int32_t mdns_y_pbt_npv_array[32];
	int32_t mdns_y_ref_wei_min_array[9];
	int32_t mdns_y_ref_wei_max_array[9];
	int32_t mdns_y_edge_type_array[1];
	int32_t mdns_y_edge_win_array[9];
	int32_t mdns_y_bi_thres_array[9];
	int32_t mdns_y_smj_thres_array[9];
	int32_t mdns_y_shp_swei_array[9];
	int32_t mdns_y_shp_mwei_array[9];
	int32_t mdns_y_cs_bhold_array[9];
	int32_t mdns_y_cm_bhold_array[9];
	int32_t mdns_y_rs_bhold_array[9];
	int32_t mdns_y_rm_bhold_array[9];
	int32_t mdns_y_cs_bwin_array[9];
	int32_t mdns_y_cm_bwin_array[9];
	int32_t mdns_y_rs_bwin_array[9];
	int32_t mdns_y_rm_bwin_array[9];
	int32_t mdns_y_cs_iwin_array[9];
	int32_t mdns_y_cm_iwin_array[9];
	int32_t mdns_y_rs_iwin_array[9];
	int32_t mdns_y_rm_iwin_array[9];
	int32_t mdns_y_cs_ewin_array[9];
	int32_t mdns_y_cm_ewin_array[9];
	int32_t mdns_y_rs_ewin_array[9];
	int32_t mdns_y_rm_ewin_array[9];
	int32_t mdns_y_cs_sego_array[9];
	int32_t mdns_y_cm_sego_array[9];
	int32_t mdns_y_rs_sego_array[9];
	int32_t mdns_y_rm_sego_array[9];
	int32_t mdns_y_cs_stren_array[9];
	int32_t mdns_y_cm_stren_array[9];
	int32_t mdns_y_rs_stren_array[9];
	int32_t mdns_y_rm_stren_array[9];
	int32_t mdns_y_cs_npv_array[16];
	int32_t mdns_y_cm_npv_array[16];
	int32_t mdns_y_rs_npv_array[16];
	int32_t mdns_y_rm_npv_array[16];
	int32_t mdns_y_shp_c_0_npv_array[9];
	int32_t mdns_y_shp_c_1_npv_array[9];
	int32_t mdns_y_shp_c_2_npv_array[9];
	int32_t mdns_y_shp_c_3_npv_array[9];
	int32_t mdns_y_shp_c_4_npv_array[9];
	int32_t mdns_y_shp_c_5_npv_array[9];
	int32_t mdns_y_shp_c_6_npv_array[9];
	int32_t mdns_y_shp_c_7_npv_array[9];
	int32_t mdns_y_shp_c_8_npv_array[9];
	int32_t mdns_y_shp_c_9_npv_array[9];
	int32_t mdns_y_shp_c_a_npv_array[9];
	int32_t mdns_y_shp_c_b_npv_array[9];
	int32_t mdns_y_shp_c_c_npv_array[9];
	int32_t mdns_y_shp_c_d_npv_array[9];
	int32_t mdns_y_shp_c_e_npv_array[9];
	int32_t mdns_y_shp_c_f_npv_array[9];
	int32_t mdns_y_shp_r_0_npv_array[9];
	int32_t mdns_y_shp_r_1_npv_array[9];
	int32_t mdns_y_shp_r_2_npv_array[9];
	int32_t mdns_y_shp_r_3_npv_array[9];
	int32_t mdns_y_shp_r_4_npv_array[9];
	int32_t mdns_y_shp_r_5_npv_array[9];
	int32_t mdns_y_shp_r_6_npv_array[9];
	int32_t mdns_y_shp_r_7_npv_array[9];
	int32_t mdns_y_shp_r_8_npv_array[9];
	int32_t mdns_y_shp_r_9_npv_array[9];
	int32_t mdns_y_shp_r_a_npv_array[9];
	int32_t mdns_y_shp_r_b_npv_array[9];
	int32_t mdns_y_shp_r_c_npv_array[9];
	int32_t mdns_y_shp_r_d_npv_array[9];
	int32_t mdns_y_shp_r_e_npv_array[9];
	int32_t mdns_y_shp_r_f_npv_array[9];
	int32_t mdns_y_adj_cnr_array[4];
	int32_t mdns_y_fluct_lmt_array[8];
	int32_t mdns_y_adj_sta_array[9];
	int32_t mdns_y_adj_tedg_s_array[9];
	int32_t mdns_y_adj_wedg_s_array[9];
	int32_t mdns_y_adj_tedg_n_array[8];
	int32_t mdns_y_adj_wedg_n_array[8];
	int32_t mdns_y_adj_lum_win_array[1];
	int32_t mdns_y_adj_tlum_s_array[9];
	int32_t mdns_y_adj_wlum_s_array[9];
	int32_t mdns_y_adj_tlum_n_array[16];
	int32_t mdns_y_adj_wlum_n_array[16];
	int32_t mdns_c_sad_thres_array[9];
	int32_t mdns_c_sad_win_opt_array[9];
	int32_t mdns_c_sad_stren_array[9];
	int32_t mdns_c_sta_stren_array[9];
	int32_t mdns_c_pbt_stren_array[9];
	int32_t mdns_c_sad_npv_array[4];
	int32_t mdns_c_sta_npv_array[32];
	int32_t mdns_c_pbt_npv_array[32];
	int32_t mdns_c_ref_wei_min_array[9];
	int32_t mdns_c_ref_wei_max_array[9];
	int32_t mdns_c_cur_blur_array[9];
	int32_t mdns_c_ref_blur_array[9];
	int32_t mdns_c_smj_thres_array[9];
	int32_t mdns_c_sfla_boh_t_array[9];
	int32_t mdns_c_sfla_cut_t_array[9];
	int32_t mdns_c_sfla_boh_s_array[9];
	int32_t mdns_c_sfla_cut_s_array[9];
	int32_t mdns_c_mfla_boh_t_array[9];
	int32_t mdns_c_mfla_cut_t_array[9];
	int32_t mdns_c_mfla_boh_s_array[9];
	int32_t mdns_c_mfla_cut_s_array[9];
	int32_t mdns_c_adj_cnr_array[4];
	int32_t mdns_c_adj_sta_array[6];
	int32_t mdns_c_adj_tcrm_s_array[9];
	int32_t mdns_c_adj_wcrm_s_array[9];
	int32_t mdns_c_adj_tcrm_n_array[8];
	int32_t mdns_c_adj_wcrm_n_array[8];

	/* mdns intp parameters */
	uint32_t mdns_sta_size_intp;
	uint32_t mdns_pbt_size_intp;
	uint32_t mdns_y_sad_thres_intp;
	uint32_t mdns_y_sta_thres_intp;
	uint32_t mdns_y_pbt_thres_intp;
	uint32_t mdns_y_sad_stren_intp;
	uint32_t mdns_y_sta_stren_intp;
	uint32_t mdns_y_pbt_stren_intp;
	uint32_t mdns_y_sad_win_opt_intp;
	uint32_t mdns_y_sta_win_opt_intp;
	uint32_t mdns_y_pbt_win_opt_intp;
	uint32_t mdns_y_sad_win_wei_intp;
	uint32_t mdns_y_sta_win_wei_intp;
	uint32_t mdns_y_pbt_win_wei_intp;
	uint32_t mdns_y_sta_mv_num_intp;
	uint32_t mdns_y_pbt_mv_num_intp;
	uint32_t mdns_y_sta_mx_num_intp;
	uint32_t mdns_y_pbt_mx_num_intp;
	uint32_t mdns_y_ref_wei_min_intp;
	uint32_t mdns_y_ref_wei_max_intp;
	uint32_t mdns_y_edge_win_intp;
	uint32_t mdns_y_bi_thres_intp;
	uint32_t mdns_y_smj_thres_intp;
	uint32_t mdns_y_shp_swei_intp;
	uint32_t mdns_y_shp_mwei_intp;
	uint32_t mdns_y_cs_bhold_intp;
	uint32_t mdns_y_cm_bhold_intp;
	uint32_t mdns_y_rs_bhold_intp;
	uint32_t mdns_y_rm_bhold_intp;
	uint32_t mdns_y_cs_bwin_intp;
	uint32_t mdns_y_cm_bwin_intp;
	uint32_t mdns_y_rs_bwin_intp;
	uint32_t mdns_y_rm_bwin_intp;
	uint32_t mdns_y_cs_iwin_intp;
	uint32_t mdns_y_cm_iwin_intp;
	uint32_t mdns_y_rs_iwin_intp;
	uint32_t mdns_y_rm_iwin_intp;
	uint32_t mdns_y_cs_ewin_intp;
	uint32_t mdns_y_cm_ewin_intp;
	uint32_t mdns_y_rs_ewin_intp;
	uint32_t mdns_y_rm_ewin_intp;
	uint32_t mdns_y_cs_sego_intp;
	uint32_t mdns_y_cm_sego_intp;
	uint32_t mdns_y_rs_sego_intp;
	uint32_t mdns_y_rm_sego_intp;
	uint32_t mdns_y_cs_stren_intp;
	uint32_t mdns_y_cm_stren_intp;
	uint32_t mdns_y_rs_stren_intp;
	uint32_t mdns_y_rm_stren_intp;
	uint32_t mdns_y_shp_c_0_npv_intp;
	uint32_t mdns_y_shp_c_1_npv_intp;
	uint32_t mdns_y_shp_c_2_npv_intp;
	uint32_t mdns_y_shp_c_3_npv_intp;
	uint32_t mdns_y_shp_c_4_npv_intp;
	uint32_t mdns_y_shp_c_5_npv_intp;
	uint32_t mdns_y_shp_c_6_npv_intp;
	uint32_t mdns_y_shp_c_7_npv_intp;
	uint32_t mdns_y_shp_c_8_npv_intp;
	uint32_t mdns_y_shp_c_9_npv_intp;
	uint32_t mdns_y_shp_c_a_npv_intp;
	uint32_t mdns_y_shp_c_b_npv_intp;
	uint32_t mdns_y_shp_c_c_npv_intp;
	uint32_t mdns_y_shp_c_d_npv_intp;
	uint32_t mdns_y_shp_c_e_npv_intp;
	uint32_t mdns_y_shp_c_f_npv_intp;
	uint32_t mdns_y_shp_r_0_npv_intp;
	uint32_t mdns_y_shp_r_1_npv_intp;
	uint32_t mdns_y_shp_r_2_npv_intp;
	uint32_t mdns_y_shp_r_3_npv_intp;
	uint32_t mdns_y_shp_r_4_npv_intp;
	uint32_t mdns_y_shp_r_5_npv_intp;
	uint32_t mdns_y_shp_r_6_npv_intp;
	uint32_t mdns_y_shp_r_7_npv_intp;
	uint32_t mdns_y_shp_r_8_npv_intp;
	uint32_t mdns_y_shp_r_9_npv_intp;
	uint32_t mdns_y_shp_r_a_npv_intp;
	uint32_t mdns_y_shp_r_b_npv_intp;
	uint32_t mdns_y_shp_r_c_npv_intp;
	uint32_t mdns_y_shp_r_d_npv_intp;
	uint32_t mdns_y_shp_r_e_npv_intp;
	uint32_t mdns_y_shp_r_f_npv_intp;
	uint32_t mdns_y_adj_tedg_s_intp;
	uint32_t mdns_y_adj_wedg_s_intp;
	uint32_t mdns_y_adj_tlum_s_intp;
	uint32_t mdns_y_adj_wlum_s_intp;
	uint32_t mdns_c_sad_thres_intp;
	uint32_t mdns_c_sad_win_opt_intp;
	uint32_t mdns_c_sad_stren_intp;
	uint32_t mdns_c_sta_stren_intp;
	uint32_t mdns_c_pbt_stren_intp;
	uint32_t mdns_c_ref_wei_min_intp;
	uint32_t mdns_c_ref_wei_max_intp;
	uint32_t mdns_c_cur_blur_intp;
	uint32_t mdns_c_ref_blur_intp;
	uint32_t mdns_c_smj_thres_intp;
	uint32_t mdns_c_sfla_boh_t_intp;
	uint32_t mdns_c_sfla_cut_t_intp;
	uint32_t mdns_c_sfla_boh_s_intp;
	uint32_t mdns_c_sfla_cut_s_intp;
	uint32_t mdns_c_mfla_boh_t_intp;
	uint32_t mdns_c_mfla_cut_t_intp;
	uint32_t mdns_c_mfla_boh_s_intp;
	uint32_t mdns_c_mfla_cut_s_intp;
	uint32_t mdns_c_adj_tcrm_s_intp;
	uint32_t mdns_c_adj_wcrm_s_intp;
	void *core;
} tisp_mdns_t;

/* -------- clm --------*/
typedef struct tisp_clm {
	int32_t tiziano_clm_h_lut[30 * 7 * 5];
	int32_t tiziano_clm_s_lut[30 * 7 * 5];
	int32_t tiziano_clm_s_reg[420];
	int32_t tiziano_clm_h_reg[420];
	uint32_t tiziano_clm_lut_shift;

	void *core;
} tisp_clm_t;


/* ------------ hldc ------------*/
typedef struct tisp_hldc {
	uint32_t hldc_con_par_array[18];

	void *core;
} tisp_hldc_t;

typedef int32_t (*net_event_cb)(void *cb_data, void *data, uint32_t len);
typedef struct tisp_netlink {
	struct sock *nlsk;
	net_event_cb net_event_process;
	void *cb_data;
} tisp_netlink_t;


/*------------param_op-----------*/
typedef struct {
	uint32_t msg_type;
	uint32_t msg_data1;
	uint32_t msg_data2;
	uint32_t msg_data3;
	uint32_t msg_data4;
	uint32_t msg_ret;
	uint8_t  msg_buf[0];
} tisp_param_op_msg_t;

typedef struct tisp_param_operate {
	tisp_param_op_msg_t *opmsg;	/*tisp_param_op_msg_t 类型.*/
	tisp_netlink_t nl;

	void *core;
} tisp_param_operate_t;


/* ---------- event ----------*/
enum {
	//ae
	TISP_EVENT_TYPE_START = 0,
	TISP_EVENT_TYPE_EXIT = TISP_EVENT_TYPE_START,
	TISP_EVENT_TYPE_AE_PROCESS,
	TISP_EVENT_TYPE_ADR_PROCESS,
	TISP_EVENT_TYPE_DEFOG_PROCESS,
	TISP_EVENT_TYPE_AE_TGAIN_UPDATE,
	TISP_EVENT_TYPE_AE_AGAIN_UPDATE,
	//for awb
	TISP_EVENT_TYPE_AE_EV,
	TISP_EVENT_TYPE_AE_IR,
	TISP_EVENT_TYPE_AWB_CT,
	//for awb
	TISP_EVENT_TYPE_WB_STAITCS,
	TISP_EVENT_TYPE_MAX,
};

#define TISP_PROCESS_EVENT_NUM 20

typedef struct {
	struct list_head entry;
	uint32_t type;
	uint64_t data1;
	uint64_t data2;
	uint64_t data3;
	uint64_t data4;
} tisp_event_t;

typedef struct {
	struct completion event_compl;
	tisp_event_t events[TISP_PROCESS_EVENT_NUM];
	struct list_head event_working;
	struct list_head event_free;
	spinlock_t event_slock;
} tisp_event_info_t;

typedef int32_t (*event_process_cb)(void *cb_data, uint64_t data1, uint64_t data2, uint64_t data3, uint64_t data4);

/* event manager .*/
typedef struct tisp_event_mg {
	tisp_event_info_t tevent_info;

	event_process_cb cb[TISP_EVENT_TYPE_MAX];
	void *cb_data[TISP_EVENT_TYPE_MAX];

	void *core;

} tisp_event_mg_t;



typedef struct tisp_core {
	tisp_info_t tispinfo;

	sensor_control_t sensor_ctrl;
	struct sensor_control_ops *sensor_ctrl_ops;

	tisp_custom_effect_t custom_eff;
	tisp_mdns_ratio_t mdns_ratio;
	int deir_en;
	int ae_switch;
	uint32_t topreg_val;
	tisp_init_param_t sensor_info;

	/* sub handles. */
	tisp_awb_t awb;
	tisp_gamma_t gamma;
	tisp_defog_t defog;
	tisp_adr_t   adr;
	tisp_ae_t    ae;
	tisp_af_t    af;
	tisp_gib_t   gib;
	tisp_dmsc_t  dmsc;
	tisp_lsc_t   lsc;
	tisp_ccm_t   ccm;
	tisp_sharpen_t sharpen;
	tisp_dpc_t   dpc;
	tisp_sdns_t  sdns;
	tisp_mdns_t  mdns;
	tisp_clm_t   clm;
	tisp_hldc_t  hldc;
	tisp_param_operate_t param_operate;

	tisp_core_tuning_t core_tuning;
	tisp_event_mg_t tisp_event_mg;

	void *tuned_params;
	loff_t tuned_params_size;

	void *priv_data;
} tisp_core_t;

int tisp_core_init(tisp_core_t *core, tisp_init_param_t *p, void *priv_data);
int tisp_core_deinit(tisp_core_t *core);

int tisp_channel_start(tisp_core_t *core, int chx);
int tisp_channel_start_restore(tisp_core_t *core);
int tisp_channel_stop(tisp_core_t *core, int chx);
int tisp_channel_stop_save(tisp_core_t *core);
int tisp_channel_attr_set(tisp_core_t *core, int chx ,tisp_channel_attr_t * cattr);
int tisp_channel_attr_set_crop_scaler(tisp_core_t *core, int chx ,tisp_channel_attr_t * cattr);

int tisp_fw_process(tisp_core_t *core);

#endif
