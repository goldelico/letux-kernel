#ifndef __OV9655_H
#define __OV9655_H

struct v4l2_subdev;

struct ov9655_platform_data {
	/*
	 * struct ov9655_platform_data - OV9655 platform data
	 * @reset: Chip reset GPIO (set to -1 if not used)
	 * @ext_freq: Input clock frequency --- not used OV9655 is running at fixed 24 MHz
	 */
		int reset;
		int ext_freq;
	};

#endif
