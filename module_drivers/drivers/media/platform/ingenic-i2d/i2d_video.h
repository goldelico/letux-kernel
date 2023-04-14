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

struct i2d_fmt {
	char	*name;
	u32	fourcc;
	int	depth;
	u32	types;
};

struct i2d_frame_info{
	uint16_t width;
	uint16_t height;
	struct i2d_fmt *fmt;
	uint32_t	size;
	uint32_t	bytesperline;
};

struct ingenic_i2d_dev {
	char* name;
	struct device *dev;
	struct v4l2_device	v4l2_dev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct video_device	*vfd;
	atomic_t			num_inst;
	struct mutex		dev_mutex;
	spinlock_t			irqlock;
	spinlock_t			ctrl_lock;
	void __iomem		*base;
	struct clk			*clk;
	int irq;
	int irq_is_request;
	wait_queue_head_t	irq_queue;
	void*	*alloc_ctx;
	struct ingenic_i2d_ctx	*cur_ctx;
	struct completion done_i2d;
};

struct ingenic_i2d_ctx {
	struct v4l2_fh		fh;
	struct ingenic_i2d_dev	*i2d_dev;
	struct v4l2_m2m_ctx	*m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*ctrl_hflip;
	struct v4l2_ctrl	*ctrl_vflip;
	struct v4l2_ctrl	*ctrl_rot;
	struct i2d_frame_info	cap_info;
	struct i2d_frame_info	out_info;
	uint32_t			vflip;
	uint32_t			hflip;
	uint32_t			angle;
	dma_addr_t			paddr;
};



#endif		// __I2D_VIDEO_H__


