#ifndef __I2D_VIDEO_H__
#define __I2D_VIDEO_H__

#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define MAX_WIDTH	3840
#define MAX_HEIGHT	2160
#define MIN_WIDTH	0
#define	MIN_HEIGHT	0

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480


#define MEM2MEM_CAPTURE	(1 << 0)
#define MEM2MEM_OUTPUT	(1 << 1)

struct dpu_fmt {
	char	*name;
	u32	fourcc;
	int	depth;
	u32	types;
	u32	layer_fmt;
	u32	wb_fmt;
};

struct dpu_frame_info{
	uint16_t width;
	uint16_t height;
	struct dpu_fmt *fmt;
	uint32_t	size;
	uint32_t	bytesperline;
};

struct ingenic_dpu_dev {
	char* name;
	struct device *dev;
	struct v4l2_device	v4l2_dev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct video_device	*vfd;
	atomic_t			num_inst;
	struct mutex		dev_mutex;
	struct vb2_alloc_ctx	*alloc_ctx;
};

struct ingenic_dpu_ctx {
	struct v4l2_fh		fh;
	struct ingenic_dpu_dev	*dpu_dev;
	struct v4l2_m2m_ctx	*m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*ctrl_hflip;
	struct v4l2_ctrl	*ctrl_vflip;
	struct v4l2_ctrl	*ctrl_rot;
	struct dpu_frame_info	cap_info;
	struct dpu_frame_info	out_info;
	uint32_t			vflip;
	uint32_t			hflip;
	uint32_t			angle;

	struct hw_composer_ctx *hw_comp;
	struct comp_setup_info	comp_info;
};


void *hw_comp_v4l2_init(struct device *dev);
int hw_comp_v4l2_exit(void *priv);

#endif		// __I2D_VIDEO_H__


