#ifndef __OV9655_H
#define __OV9655_H

struct v4l2_subdev;

struct ov9655_platform_data {
	/*
	* struct mt9p031_platform_data - MT9P031 platform data
	* @set_xclk: Clock frequency set callback
	* @reset: Chip reset GPIO (set to -1 if not used)
	* @ext_freq: Input clock frequency
	* @target_freq: Pixel clock frequency
	*/
	int (*set_xclk)(struct v4l2_subdev *subdev, int hz);
	int reset;	/* reset GPIO */
};

#endif
