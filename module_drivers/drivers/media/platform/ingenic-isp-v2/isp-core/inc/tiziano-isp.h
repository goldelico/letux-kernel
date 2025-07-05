#ifndef __TIZIANO_ISP_H__
#define __TIZIANO_ISP_H__

#include <txx-funcs.h>

typedef enum {
	TISP_OPS_MODE_DISABLE,			/**< DISABLE mode of the current module */
	TISP_OPS_MODE_ENABLE,			/**< ENABLE mode of the current module */
	TISP_OPS_MODE_BUTT,			/**< effect paramater, parameters have to be less than this value*/
} TISP_OPS_MODE;

/**
 * ISP Function Mode
 */
typedef enum {
	TISP_OPS_TYPE_AUTO,			/**< AUTO mode of the current module*/
	TISP_OPS_TYPE_MANUAL,			/**< MANUAL mode of the current module*/
	TISP_OPS_TYPE_BUTT,			/**< effect paramater, parameters have to be less than this value*/
} TISP_OPS_TYPE;

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
	unsigned short min_integration_time_short;
	unsigned short max_integration_time_short;
	unsigned int integration_time_short;
	unsigned int max_again_short;	//the format is .16
	unsigned int again_short;
	unsigned int max_dgain_short;	//the format is .16
	unsigned int dgain_short;
	unsigned short again_short_apply_delay;
	unsigned short dgain_short_apply_delay;
	unsigned short integration_time_short_apply_delay;
} sensor_info_t;

typedef enum {
	IMPISP_DUALSENSOR_SIGLE_BYPASS_MODE = 0,
	IMPISP_DUALSENSOR_DUAL_DIRECT_MODE,
	IMPISP_DUALSENSOR_DUAL_SELECT_MODE,
	IMPISP_DUALSENSOR_DUAL_SINGLECACHED_MODE,
	IMPISP_DUALSENSOR_DUAL_ALLCACHED_MODE,
	IMPISP_DUALSENSOR_MODE_BUTT,
} dual_sensor_mode_t;

typedef struct {
	TISP_OPS_MODE en;
	uint32_t switch_con;
 	uint32_t switch_con_num;
} tisp_dual_sensor_switch_t;

typedef enum {
	IMPISP_TOTAL_ONE = 1,
	IMPISP_TOTAL_TWO,
	IMPISP_TOTAL_THR,
	IMPISP_TOTAL_BUTT,
} sensor_num_t;


typedef enum{
	IMPISP_NOT_JOINT = 0,
	IMPISP_MAIN_ON_THE_LEFT,
	IMPISP_MAIN_ON_THE_RIGHT,
	IMPISP_MAIN_ON_THE_ABOVE,
	IMPISP_MAIN_ON_THE_UNDER,
	IMPISP_MIAN_JOINT_BUTT,
} dual_sensor_split_joint;

typedef struct {
	sensor_num_t sensor_num;
	dual_sensor_mode_t dual_mode;
	tisp_dual_sensor_switch_t dual_mode_switch;
	dual_sensor_split_joint joint_mode;
} multisensor_mode_t;

typedef struct {
	int width;
	int height;
	int bayer;
	char sensor[16];
	sensor_info_t sensor_info;
	int WdrEn;
	int DNMode;
	int sensorId;  //the sensor id:mark the struct for sensor 0/1
	multisensor_mode_t multi_mode;

	void *tuned_params;
	loff_t tuned_params_size;
} tisp_init_param_t;

typedef struct {
	tisp_init_param_t sensor[3];
	multisensor_mode_t multi_mode;
} tisp_init_par_t;

typedef struct {
	int scaler_en;
	int scaler_width;
	int scaler_height;
	int crop_en;
	int crop_x;
	int crop_y;
	int crop_width;
	int crop_height;
	int fcrop_en;
	int fcrop_x;
	int fcrop_y;
	int fcrop_width;
	int fcrop_height;
} tisp_channel_attr_t;

//ae
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
	uint32_t tisp_ae_it_short_manual;
	uint32_t tisp_ae_ag_short_manual;
	uint32_t tisp_ae_sensor_again_short;
	uint32_t tisp_ae_sensor_integration_time_short;
	uint32_t tisp_ae_max_sensor_again_short;
	uint32_t tisp_ae_max_sensor_integration_time_short;
	uint32_t tisp_ae_max_sensor_dgain_short;
	uint32_t tisp_ae_max_isp_dgain_short;
	uint32_t tisp_ae_manual_short;
	uint32_t tisp_ae_sensor_dgain_short;
	uint32_t tisp_ae_dg_manual_short;
	uint32_t tisp_ae_isp_dgian_short;
	uint32_t tisp_ae_min_sensor_again;
	uint32_t tisp_ae_min_sensor_dgain;
	uint32_t tisp_ae_min_sensor_integration_time;
	uint32_t tisp_ae_min_isp_dgain;
	uint32_t tisp_ae_min_sensor_again_short;
	uint32_t tisp_ae_min_sensor_integration_time_short;
	uint32_t tisp_ae_min_sensor_dgain_short;
	uint32_t tisp_ae_min_isp_dgain_short;
	uint32_t tisp_ae_sensor_dg_manual;
	uint32_t tisp_ae_sensor_dg_short_manual;
	uint32_t tisp_ae_ev_short;
	uint32_t tisp_ae_tgain_db_short;
	uint32_t tisp_ae_again_db_short;
} tisp_ae_ctrls_t;

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
	uint32_t ae_hist[256];
	uint32_t ae_hist_5bin[5];
	uint8_t ae_hist_nodes[4];
	uint8_t ae_hist_hv[2];
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
	unsigned short  af_belta_alt;
} tisp_af_attr;

typedef struct {
	int vinum;
	char path[64];
} tx_isp_bin_path;

extern struct completion ae_algo_comp[2];
extern struct completion ae_algo_close_comp[2];
extern struct completion awb_algo_comp[2];
extern struct completion awb_algo_close_comp[2];
extern tisp_ae_ctrls_t tisp_ae_main_ctrls;
extern tisp_ae_ctrls_t tisp_ae_sec_ctrls;
extern tisp_init_par_t tisp_par_info;
extern int main_day_night_switch;
extern int event_busy;

int tisp_main_init(tisp_init_param_t *p);
int tisp_sec_init(tisp_init_param_t *p);
void tisp_ipc_triger(void);
int tisp_deinit(int vinum);
void tisp_stream_on(tisp_init_param_t *p);
void tisp_process_init(void);
void tisp_process_deinit(int vinum);
void tisp_slake_all(void);
void tisp_activate_all(void);

int tisp_channel_main_start(int chx);
int tisp_channel_sec_start(int chx);
int tisp_channel_main_stop(int chx);
int tisp_channel_sec_stop(int chx);
int tisp_channel_main_fifo_clear(int chx);
int tisp_channel_sec_fifo_clear(int chx);
int tisp_channel_main_attr_set(int chx ,tisp_channel_attr_t * cattr);
int tisp_channel_sec_attr_set(int chx ,tisp_channel_attr_t * cattr);

int tisp_fw_process(int vinum);
int tiziano_sync_sensor_attr(tisp_init_param_t *sensor_attr);

void Tzn_Msca_addr_fifo_write(uint8_t sensor_num/*{{{*/
			      ,uint8_t channel
			      ,uint32_t phy_y_addr
			      ,uint32_t phy_c_addr
	);/*}}}*/

void Tzn_Msca_addr_fifo_read(uint8_t sensor_num/*{{{*/
			     ,uint8_t channel
			     ,uint32_t * phy_y_addr
			     ,uint32_t * phy_c_addr
	);/*}}}*/

uint8_t Tzn_Msca_addr_fifo_have_read(uint8_t sensor_num/*{{{*/
				     ,uint8_t channel
	);/*}}}*/

void tisp_hardware_reg_refresh(int vinum);
void tisp_lce_get_debug(unsigned int *deinfo);

#endif
