#ifndef __ISP_SENSOR_H__
#define __ISP_SENSOR_H__

#define V4L2_CID_USER_ANALOG_GAIN_SHORT             (V4L2_CID_USER_BASE + 0x1090)
#define V4L2_CID_USER_EXPOSURE_SHORT             (V4L2_CID_USER_BASE + 0x1091)

enum  mipi_sensor_data_type_value {
	YUV422_8BIT = 0x1e,
	RAW8 = 0x2a,
	RAW10 = 0x2b,
	RAW12 = 0x2c,
};

enum tx_sensor_frm_mode {
	TX_SENSOR_DEFAULT_FRAME_MODE = 0,
	TX_SENSOR_WDR_2_FRAME_MODE,
	TX_SENSOR_WDR_3_FRAME_MODE,
	TX_SENSOR_WDR_4_FRAME_MODE,
};

enum tx_sensor_mode {
	TX_SENSOR_DEFAULT_MODE = 0,
	TX_SENSOR_NOT_VC_MODE,
	TX_SENSOR_VC_MODE,
};

enum tx_sensor_csi_fmt {
	TX_SENSOR_RAW8 = 0,
	TX_SENSOR_RAW10,
	TX_SENSOR_RAW12,
};

typedef enum {
	SENSOR_MIPI_OTHER_MODE,
	SENSOR_MIPI_SONY_MODE,
}sensor_mipi_mode;

struct mipi_cfg {
	unsigned int clk;
	unsigned int twidth;
	unsigned int theight;
	sensor_mipi_mode mipi_mode;
	unsigned int hcrop_diff_en;
	unsigned int mipi_vcomp_en;
	unsigned int mipi_hcomp_en;
	unsigned short mipi_crop_start0x;
	unsigned short mipi_crop_start0y;
	unsigned short mipi_crop_start1x;
	unsigned short mipi_crop_start1y;
	unsigned short mipi_crop_start2x;
	unsigned short mipi_crop_start2y;
	unsigned short mipi_crop_start3x;
	unsigned short mipi_crop_start3y;
	unsigned int line_sync_mode;
	unsigned int work_start_flag;
	unsigned int data_type_en;
	enum  mipi_sensor_data_type_value data_type_value;
	unsigned int del_start;
	unsigned int sensor_fid_mode;
	enum tx_sensor_frm_mode sensor_frame_mode;
	enum tx_sensor_mode sensor_mode;
	enum tx_sensor_csi_fmt sensor_csi_fmt;
};

struct dvp_cfg {

};

struct sensor_info {
	unsigned int fps;
	unsigned int total_width;
	unsigned int total_height;
	int wdr_en;
	union {
		struct mipi_cfg mipi_cfg;
		struct dvp_cfg dvp_cfg;
	};
};

#endif
