#ifndef __SENSOR_DRV_H__
#define __SENSOR_DRV_H__

#include "linux/types.h"

#define LOG2_GAIN_SHIFT 16

typedef struct _sensor_context_t
{
	int sensor_id;
	uint16_t again;
	uint16_t dgain;
	uint8_t n_context;
	uint8_t wdr_mode;
	uint16_t again_x2;
	uint16_t dgain_coarse;
	uint16_t dgain_fine;
	uint8_t column_buffer_gain_index;
	uint16_t again_short;
	uint16_t it;
	uint16_t it_short;
	uint16_t dgain_short;
} sensor_context_t;

typedef struct _image_resolution_t
{
	uint16_t width;
	uint16_t height;
} image_resolution_t;

typedef struct _sensor_param_t
{
	uint8_t mode;
	image_resolution_t total;
	image_resolution_t active;
	sensor_context_t sensor_ctx;
	int32_t again_log2_max;
	int32_t dgain_log2_max;
	uint32_t integration_time_min;
	uint32_t integration_time_max;
	uint32_t integration_time_long_max;
	uint32_t integration_time_limit;
	uint16_t day_light_integration_time_max;
	uint8_t integration_time_apply_delay;
	uint8_t analog_gain_apply_delay;
	uint8_t digital_gain_apply_delay;
	int32_t xoffset;
	int32_t yoffset;
	int32_t anti_flicker_pos;
	uint32_t lines_per_second;
	uint32_t integration_time_min_short;
	uint32_t integration_time_max_short;
	uint32_t again_log2_max_short;
	uint32_t dgain_log2_max_short;
	uint8_t analog_gain_short_apply_delay;
	uint8_t digital_gain_short_apply_delay;
	uint8_t integration_time_short_apply_delay;
} sensor_param_t;

typedef struct _sensor_control_t
{
	sensor_param_t param[3];
	void (*hw_reset_disable)(void);
	void (*hw_reset_enable)(void);
	int32_t (*alloc_analog_gain)(int32_t gain, sensor_context_t *p_ctx);
	int32_t (*alloc_analog_gain_short)(int32_t gain, sensor_context_t *p_ctx);
	int32_t (*alloc_digital_gain)(int32_t gain, sensor_context_t *p_ctx);
	int32_t (*alloc_digital_gain_short)(int32_t gain, sensor_context_t *p_ctx);
	uint32_t (*alloc_integration_time)(uint32_t int_time, sensor_context_t *p_ctx);
	uint32_t (*alloc_integration_time_short)(uint32_t int_time, sensor_context_t *p_ctx);
	void (*set_integration_time)(uint16_t int_time, sensor_param_t *param);
	void (*set_integration_time_short)(uint16_t int_time, sensor_param_t *param);
	void (*start_changes)(sensor_context_t *p_ctx);
	void (*end_changes)(sensor_context_t *p_ctx);
	void (*set_analog_gain)(uint32_t again_reg_val, sensor_context_t *p_ctx);
	void (*get_analog_gain)(uint32_t *again_reg_val, sensor_context_t *p_ctx);
	void (*set_analog_gain_short)(uint32_t again_reg_val, sensor_context_t *p_ctx);
	void (*get_analog_gain_short)(uint32_t *again_reg_val, sensor_context_t *p_ctx);
	void (*set_digital_gain)(uint32_t dgain_reg_val, sensor_context_t *p_ctx);
	void (*set_digital_gain_short)(uint32_t dgain_reg_val, sensor_context_t *p_ctx);
	uint16_t (*get_normal_fps)(sensor_param_t *param);
	uint16_t (*read_black_pedestal)(int i, uint32_t gain);
	void (*set_mode)( uint8_t mode, sensor_param_t *param);
	void (*set_wdr_mode)(uint8_t wdr, sensor_param_t *param);
	uint32_t (*fps_control)(uint8_t fps, sensor_param_t *param);
	uint16_t (*get_id)(sensor_context_t *p_ctx);
	void (*disable_isp)(sensor_context_t *p_ctx);
	uint32_t (*get_lines_per_second)(sensor_param_t *param);
} sensor_control_t;

typedef sensor_control_t *sensor_control_ptr_t;

void sensor_init(sensor_control_t *ctrl, int vinum);
int sensor_early_init(void *ispcore);

#endif /* __SENSOR_DRV_H__ */
