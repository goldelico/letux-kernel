#ifndef __OV9655_H
#define __OV9655_H

struct v4l2_subdev;

struct ov9655_platform_data {
	/* from OV7670 - why do we ned that? */
	int min_width;			/* Filter out smaller sizes */
	int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
	bool use_smbus;			/* Use smbus I/O instead of I2C */
	
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
