#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/pinctrl/consumer.h>
#include <linux/of_i2c.h>
#include <linux/of_device.h>

#include <media/dra7xx_vip.h>

#include <linux/of_i2c.h>
#include "vip.h"

#define VIP_MODULE_NAME "dra7xx-vip"
#define VIP_INPUT_NAME "Vin0"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

MODULE_DESCRIPTION("TI DRA7xx VIP driver");
MODULE_AUTHOR("Dale Farnsworth, <dale@farnsworth.org>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

/*
 * Minimum and maximum frame sizes
 */
#define MIN_W		128
#define MIN_H		128
#define MAX_W		1920
#define MAX_H		1080

/*
 * Required alignments
 */
#define S_ALIGN		0 /* multiple of 1 */
#define H_ALIGN		1 /* multiple of 2 */
#define W_ALIGN		1 /* multiple of 2 */
#define L_ALIGN		7 /* multiple of 128, line stride, 16 bytes */

/*
 * Need a descriptor entry for each of up to 15 outputs,
 * and up to 2 control transfers.
 */
#define VIP_DESC_LIST_SIZE	(17 * sizeof(struct vpdma_dtd))

#define vip_dprintk(dev, fmt, arg...) \
	v4l2_dbg(1, 1, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)

/*
 * The srce_info structure contains per-srce data.
 */
struct vip_srce_info {
	u8	base_channel;	/* the VPDMA channel nummber */
	u8	vb_index;	/* input frame f, f-1, f-2 index */
	u8	vb_part;	/* identifies section of co-planar formats */
};

#define VIP_VPDMA_FIFO_SIZE	2

/*
 * Define indices into the srce_info tables
 */

#define VIP_SRCE_MULT_PORT		0
#define VIP_SRCE_MULT_ANC		1
#define VIP_SRCE_LUMA		2
#define VIP_SRCE_CHROMA		3
#define VIP_SRCE_RGB		4

static struct vip_srce_info srce_info[5] = {
	[VIP_SRCE_MULT_PORT] = {
		.base_channel	= VIP1_CHAN_NUM_MULT_PORT_A_SRC0,
		.vb_index	= 0,
		.vb_part	= VIP_CHROMA,
	},
	[VIP_SRCE_MULT_ANC] = {
		.base_channel	= VIP1_CHAN_NUM_MULT_ANC_A_SRC0,
		.vb_index	= 0,
		.vb_part	= VIP_LUMA,
	},
	[VIP_SRCE_LUMA] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_LUMA,
		.vb_index	= 1,
		.vb_part	= VIP_LUMA,
	},
	[VIP_SRCE_CHROMA] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_CHROMA,
		.vb_index	= 1,
		.vb_part	= VIP_CHROMA,
	},
	[VIP_SRCE_RGB] = {
		.base_channel	= VIP1_CHAN_NUM_PORT_B_RGB,
		.vb_part	= VIP_LUMA,
	},
};

struct vip_fmt {
	char	*name;			/* Human-readable name */
	u32	fourcc;			/* The standard format identifier */
	enum v4l2_colorspace colorspace;/* Colorspace: YPE_YUV or YPE_RGB */
	u8	coplanar;		/* set for unpacked Luma and Chroma */
	const struct vpdma_data_format *vpdma_fmt[VIP_MAX_PLANES];
};

static struct vip_fmt vip_formats[] = {
	{
		.name		= "YUV 444 co-planar",
		.fourcc		= V4L2_PIX_FMT_NV24,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 1,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_Y444],
				    &vpdma_yuv_fmts[VPDMA_DATA_FMT_C444],
				  },
	},
	{
		.name		= "YUV 422 co-planar",
		.fourcc		= V4L2_PIX_FMT_NV16,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 1,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_Y422],
				    &vpdma_yuv_fmts[VPDMA_DATA_FMT_C422],
				  },
	},
	{
		.name		= "YUV 420 co-planar",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 1,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_Y420],
				    &vpdma_yuv_fmts[VPDMA_DATA_FMT_C420],
				  },
	},
	{
		.name		= "UYVY 422 packed",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_CY422],
				  },
	},
	{
		.name		= "YUYV 422 packed",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_YC422],
				  },
	},
	{
		.name		= "YUYV 422 packed",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_yuv_fmts[VPDMA_DATA_FMT_YC422],
				  },
	},
	{
		.name		= "RGB888 packed",
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_rgb_fmts[VPDMA_DATA_FMT_RGB24],
				  },
	},
	{
		.name		= "ARGB888 packed",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.coplanar	= 0,
		.vpdma_fmt	= { &vpdma_rgb_fmts[VPDMA_DATA_FMT_ARGB32],
				  },
	},
};

/*
 * Find our format description corresponding to the passed v4l2_format
 */
static struct vip_fmt *find_format(struct v4l2_format *f)
{
	struct vip_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(vip_formats); k++) {
		fmt = &vip_formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

static LIST_HEAD(vip_shared_list);

static inline struct vip_dev *notifier_to_vip_dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct vip_dev, notifier);
}

/*
 * port flag bits
 */
#define FLAG_FRAME_1D		(1 << 0)
#define FLAG_EVEN_LINE_SKIP	(1 << 1)
#define FLAG_ODD_LINE_SKIP	(1 << 2)
#define FLAG_MODE_TILED		(1 << 3)
#define FLAG_INTERLACED		(1 << 4)
#define FLAG_MULTIPLEXED	(1 << 5)
#define FLAG_MULT_PORT		(1 << 6)
#define FLAG_MULT_ANC		(1 << 7)

/*
 * Function prototype declarations
 */
static int alloc_port(struct vip_dev *, int);
static void free_port(struct vip_port *);
static int find_or_alloc_shared(struct platform_device *, struct vip_dev *,
		struct resource *);
static int vip_setup_parser(struct vip_port *port);

static u32 read_sreg(struct vip_shared *shared, int offset)
{
	return ioread32(shared->base + offset);
}

static void write_sreg(struct vip_shared *shared, int offset, u32 value)
{
	iowrite32(value, shared->base + offset);
}

static u32 read_vreg(struct vip_dev *dev, int offset)
{
	return ioread32(dev->base + offset);
}

static void write_vreg(struct vip_dev *dev, int offset, u32 value)
{
	iowrite32(value, dev->base + offset);
}

/*
 * Insert a masked field into a 32-bit field
 */
static void insert_field(u32 *valp, u32 field, u32 mask, int shift)
{
	u32 val = *valp;

	val &= ~(mask << shift);
	val |= (field & mask) << shift;
	*valp = val;
}

/*
 * Set the system idle mode
 */
static void vip_set_idle_mode(struct vip_shared *shared, int mode)
{
	u32 reg = read_sreg(shared, VIP_SYSCONFIG);
	insert_field(&reg, mode, VIP_SYSCONFIG_IDLE_MASK,
		     VIP_SYSCONFIG_IDLE_SHIFT);
	write_sreg(shared, VIP_SYSCONFIG, reg);
}

/*
 * Set the VIP standby mode
 */
static void vip_set_standby_mode(struct vip_shared *shared, int mode)
{
	u32 reg = read_sreg(shared, VIP_SYSCONFIG);
	insert_field(&reg, mode, VIP_SYSCONFIG_STANDBY_MASK,
		     VIP_SYSCONFIG_STANDBY_SHIFT);
	write_sreg(shared, VIP_SYSCONFIG, reg);
}

static struct mutex		vip_driver_mutex;

/*
 * Enable or disable the VIP clocks
 */

static void vip_set_clock_enable(struct vip_dev *dev, bool on)
{
	u32 val = 0;

	if (on) {
		if (dev->slice_id == VIP_SLICE1)
			val = VIP_VIP1_DATA_PATH_CLK_ENABLE | VIP_VPDMA_CLK_ENABLE;
		else
			val = VIP_VIP2_DATA_PATH_CLK_ENABLE | VIP_VPDMA_CLK_ENABLE;
	}

	write_vreg(dev, VIP_CLK_ENABLE, val);
}

static void vip_top_reset(struct vip_dev *dev)
{
	u32 val = 0;

	val = read_vreg(dev, VIP_CLK_RESET);

	if (dev->slice_id == VIP_SLICE1)
		insert_field(&val, 1, VIP_DATA_PATH_CLK_RESET_MASK,
			VIP_VIP1_DATA_PATH_RESET_SHIFT);
	else
		insert_field(&val, 1, VIP_DATA_PATH_CLK_RESET_MASK,
			VIP_VIP2_DATA_PATH_RESET_SHIFT);

	write_vreg(dev, VIP_CLK_RESET, val);

	udelay(200);

	val = read_vreg(dev, VIP_CLK_RESET);

	if (dev->slice_id == VIP_SLICE1)
		insert_field(&val, 0, VIP_DATA_PATH_CLK_RESET_MASK,
			VIP_VIP1_DATA_PATH_RESET_SHIFT);
	else
		insert_field(&val, 0, VIP_DATA_PATH_CLK_RESET_MASK,
			VIP_VIP2_DATA_PATH_RESET_SHIFT);
	write_vreg(dev, VIP_CLK_RESET, val);
}

static void vip_top_vpdma_reset(struct vip_shared *shared)
{
	u32 val;

	val = read_sreg(shared, VIP_CLK_RESET);
	insert_field(&val, 1, VIP_VPDMA_CLK_RESET_MASK,
		VIP_VPDMA_CLK_RESET_SHIFT);
	write_sreg(shared, VIP_CLK_RESET, val);

	udelay(200);

	val = read_sreg(shared, VIP_CLK_RESET);
	insert_field(&val, 0, VIP_VPDMA_CLK_RESET_MASK,
		VIP_VPDMA_CLK_RESET_SHIFT);
	write_sreg(shared, VIP_CLK_RESET, val);
}

static void vip_xtra_set_repack_sel(struct vip_port *port, int repack_mode)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_1);
		insert_field(&val, repack_mode, VIP_REPACK_SEL_MASK, VIP_REPACK_SEL_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_1, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_1);
		insert_field(&val, repack_mode, VIP_REPACK_SEL_MASK, VIP_REPACK_SEL_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_1, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_1);
		insert_field(&val, repack_mode, VIP_REPACK_SEL_MASK, VIP_REPACK_SEL_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_1, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_1);
		insert_field(&val, repack_mode, VIP_REPACK_SEL_MASK, VIP_REPACK_SEL_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_1, val);
	}
}

static void vip_set_discrete_basic_mode(struct vip_port *port)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		val |= VIP_DISCRETE_BASIC_MODE;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		val |= VIP_DISCRETE_BASIC_MODE;

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		val |= VIP_DISCRETE_BASIC_MODE;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		val |= VIP_DISCRETE_BASIC_MODE;

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_pclk_polarity(struct vip_port *port, int polarity)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_PIXCLK_EDGE_POLARITY;
		else
			val &= ~VIP_PIXCLK_EDGE_POLARITY;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_PIXCLK_EDGE_POLARITY;
		else
			val &= ~VIP_PIXCLK_EDGE_POLARITY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_PIXCLK_EDGE_POLARITY;
		else
			val &= ~VIP_PIXCLK_EDGE_POLARITY;
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_PIXCLK_EDGE_POLARITY;
		else
			val &= ~VIP_PIXCLK_EDGE_POLARITY;

		write_vreg(port->dev,  VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_vsync_polarity(struct vip_port *port, int polarity)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_VSYNC_POLARITY;
		else
			val &= ~VIP_VSYNC_POLARITY;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_VSYNC_POLARITY;
		else
			val &= ~VIP_VSYNC_POLARITY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_VSYNC_POLARITY;
		else
			val &= ~VIP_VSYNC_POLARITY;
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_VSYNC_POLARITY;
		else
			val &= ~VIP_VSYNC_POLARITY;

		write_vreg(port->dev,  VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_hsync_polarity(struct vip_port *port, int polarity)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_HSYNC_POLARITY;
		else
			val &= ~VIP_HSYNC_POLARITY;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_HSYNC_POLARITY;
		else
			val &= ~VIP_HSYNC_POLARITY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_HSYNC_POLARITY;
		else
			val &= ~VIP_HSYNC_POLARITY;
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_HSYNC_POLARITY;
		else
			val &= ~VIP_HSYNC_POLARITY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_actvid_polarity(struct vip_port *port, int polarity)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_ACTVID_POLARITY;
		else
			val &= ~VIP_ACTVID_POLARITY;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (polarity)
			val |= VIP_ACTVID_POLARITY;
		else
			val &= ~VIP_ACTVID_POLARITY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_ACTVID_POLARITY;
		else
			val &= ~VIP_ACTVID_POLARITY;
		write_vreg(port->dev,  VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (polarity)
			val |= VIP_ACTVID_POLARITY;
		else
			val &= ~VIP_ACTVID_POLARITY;

		write_vreg(port->dev,  VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_actvid_hsync_n(struct vip_port *port, int enable)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (enable)
			val |= VIP_USE_ACTVID_HSYNC_ONLY;
		else
			val &= ~VIP_USE_ACTVID_HSYNC_ONLY;

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		if (enable)
			val |= VIP_USE_ACTVID_HSYNC_ONLY;
		else
			val &= ~VIP_USE_ACTVID_HSYNC_ONLY;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (enable)
			val |= VIP_USE_ACTVID_HSYNC_ONLY;
		else
			val &= ~VIP_USE_ACTVID_HSYNC_ONLY;
		write_vreg(port->dev,  VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		if (enable)
			val |= VIP_USE_ACTVID_HSYNC_ONLY;
		else
			val &= ~VIP_USE_ACTVID_HSYNC_ONLY;

		write_vreg(port->dev,  VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}


static void vip_sync_type(struct vip_port *port, enum sync_types sync)
{
	u32 val;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		insert_field(&val, sync, VIP_SYNC_TYPE_MASK,
				VIP_SYNC_TYPE_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);
		insert_field(&val, sync, VIP_SYNC_TYPE_MASK,
				VIP_SYNC_TYPE_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		insert_field(&val, sync, VIP_SYNC_TYPE_MASK,
				VIP_SYNC_TYPE_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);
		insert_field(&val, sync, VIP_SYNC_TYPE_MASK,
				VIP_SYNC_TYPE_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_data_interface(struct vip_port *port, enum data_interface_modes mode)
{
	u32 val = 0;

	if (port->dev->slice_id == VIP_SLICE1) {
		insert_field(&val, mode, VIP_DATA_INTERFACE_MODE_MASK, VIP_DATA_INTERFACE_MODE_SHFT);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET, val);
	} else if (port->dev->slice_id == VIP_SLICE2) {
		insert_field(&val, mode, VIP_DATA_INTERFACE_MODE_MASK, VIP_DATA_INTERFACE_MODE_SHFT);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET, val);
	}
}

#if 0
static void vip_reset_port(struct vip_port *port)
{
	u32 val = 0;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, VIP_SW_RESET);

		udelay(200);

		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, 0);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, VIP_SW_RESET);

		udelay(200);

		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, 0);

	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, VIP_SW_RESET);

		udelay(200);

		val = read_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);

		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, 0);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, VIP_SW_RESET);

		udelay(200);

		val = read_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0);

		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, 0);
	}
}
#endif

static void vip_set_port_enable(struct vip_port *port, bool on)
{
	u32 val = 0;

	if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE1) {
		if (on)
			val |= VIP_PORT_ENABLE;
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 0 && port->dev->slice_id == VIP_SLICE2) {
		if (on)
			val |= VIP_PORT_ENABLE;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTA_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE1) {
		if (on)
			val |= VIP_PORT_ENABLE;
		write_vreg(port->dev, VIP1_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	} else if (port->port_id == 1 && port->dev->slice_id == VIP_SLICE2) {
		if (on)
			val |= VIP_PORT_ENABLE;
		write_vreg(port->dev, VIP2_PARSER_REG_OFFSET + VIP_PARSER_PORTB_0, val);
	}
}

static void vip_set_slice_path(struct vip_dev *dev, enum data_path_select data_path)
{
	u32 val = 0;

	switch (data_path) {
	case VIP_MULTI_CHANNEL_DATA_SELECT:
		if (dev->slice_id == VIP_SLICE1) {
			val |= VIP_MULTI_CHANNEL_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK, VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP1_DATA_PATH_SELECT, val);
		} else if (dev->slice_id == VIP_SLICE2) {
			val |= VIP_MULTI_CHANNEL_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK, VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP2_DATA_PATH_SELECT, val);
		}
		break;
	case VIP_RGB_OUT_LO_DATA_SELECT:
		if (dev->slice_id == VIP_SLICE1) {
			val |= VIP_RGB_OUT_LO_SRC_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK, VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP1_DATA_PATH_SELECT, val);
		} else if (dev->slice_id == VIP_SLICE2) {
			val |= VIP_RGB_OUT_LO_SRC_SELECT;
			insert_field(&val, data_path, VIP_DATAPATH_SELECT_MASK, VIP_DATAPATH_SELECT_SHFT);

			write_vreg(dev, VIP_VIP2_DATA_PATH_SELECT, val);
		}
		break;
	default:
		BUG();
	}
}

/*
 * Return the vip_stream structure for a given struct file
 */
static struct vip_stream *file2stream(struct file *file)
{
	return container_of(file->private_data, struct vip_stream, fh);
}

/*
 * Append a destination descriptor to the current descriptor list,
 * setting up dma to the given srce.
 */
static int add_out_dtd(struct vip_stream *stream, int srce_type)
{
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_srce_info *sinfo = &srce_info[srce_type];
	struct vb2_buffer *vb = &stream->cur_buf->vb;
	struct v4l2_rect *c_rect = &port->c_rect;
	struct vip_fmt *fmt = port->fmt;
	struct vpdma_dtd *dtd;
	int channel, plane = 0;
	dma_addr_t dma_addr;
	u32 flags;

	channel = sinfo->base_channel;

	switch (srce_type) {
	case VIP_SRCE_MULT_PORT:
	case VIP_SRCE_MULT_ANC:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_MULT_PORTB_OFFSET;
		channel += stream->stream_id;
		flags = 0;
		break;
	case VIP_SRCE_CHROMA:
		plane = 1;
	case VIP_SRCE_LUMA:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_YUV_PORTB_OFFSET;
		flags = port->flags;
		break;
	case VIP_SRCE_RGB:
		if (port->port_id == VIP_PORTB)
			channel += VIP_CHAN_RGB_PORTB_OFFSET;
		flags = port->flags;
		break;
	default:
		BUG();
	}

	if (dev->slice_id == VIP_SLICE2)
		channel += VIP_CHAN_VIP2_OFFSET;

	dma_addr = vb2_dma_contig_plane_dma_addr(vb, plane);
	if (!dma_addr) {
		v4l2_err(&dev->v4l2_dev, "Acquiring buffer dma_addr failed\n");
		return -1;
	}

	vpdma_add_out_dtd(&dev->desc_list, c_rect->width,
		fmt->vpdma_fmt[plane], dma_addr, channel, flags);

	/*
	 * add_out_dtd sets the max WIDTH and HEIGHT to be 1920x1080
	 * Change this later so that max WIDTH and HEIGHT are taken from
	 * VPDMA_MAX_SIZE1 or VPDMA_MAX_SIZE2 register
	 * This allows to use different sets for different slices
	 */

	dtd = dev->desc_list.buf.addr;
	if (dev->slice_id == VIP_SLICE1) {
		vpdma_set_max_size(dev->shared->vpdma, VPDMA_MAX_SIZE1,
			stream->width, stream->height);

		dtd_set_max_width_height(dtd,
			MAX_OUT_WIDTH_REG1, MAX_OUT_HEIGHT_REG1);
	} else {
		vpdma_set_max_size(dev->shared->vpdma, VPDMA_MAX_SIZE2,
			stream->width, stream->height);

		dtd_set_max_width_height(dtd,
			MAX_OUT_WIDTH_REG2, MAX_OUT_HEIGHT_REG2);
	}

	return 0;
}

/*
 * add_stream_dtds - prepares and starts DMA for pending transfers
 */
static void add_stream_dtds(struct vip_stream *stream)
{
	struct vip_port *port = stream->port;
	int srce_type;

	if (port->flags & FLAG_MULT_PORT) {
		srce_type = VIP_SRCE_MULT_PORT;
	} else if (port->flags & FLAG_MULT_ANC) {
		srce_type = VIP_SRCE_MULT_ANC;
	} else if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB) {
		srce_type = VIP_SRCE_RGB;
	} else {
		srce_type = VIP_SRCE_LUMA;
	}

	add_out_dtd(stream, srce_type);

	if (srce_type == VIP_SRCE_LUMA && port->fmt->coplanar) {
		add_out_dtd(stream, VIP_SRCE_CHROMA);
	}
}

static void enable_irqs(struct vip_dev *dev, int irq_num)
{
	u32 reg_addr = VIP_INT0_ENABLE0_SET +
			VIP_INTC_INTX_OFFSET * irq_num;
	int list_num = dev->slice_id;

	write_sreg(dev->shared, reg_addr, 1 << (list_num * 2));

	vpdma_enable_list_complete_irq(dev->shared->vpdma,
		irq_num, list_num, true);
}

static void disable_irqs(struct vip_dev *dev, int irq_num)
{
	u32 reg_addr = VIP_INT0_ENABLE0_CLR +
			VIP_INTC_INTX_OFFSET * irq_num;

	write_sreg(dev->shared, reg_addr, 0xffffffff);

	vpdma_enable_list_complete_irq(dev->shared->vpdma,
		irq_num, 0, false);
}

static void populate_desc_list(struct vip_stream *stream)
{
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	unsigned int list_length;

	dev->desc_next = dev->desc_list.buf.addr;
	add_stream_dtds(stream);

	list_length = dev->desc_next - dev->desc_list.buf.addr;
	vpdma_buf_map(dev->shared->vpdma, &dev->desc_list.buf);
}

/*
 * start_dma - adds descriptors to the dma list and submits them.
 * Should be called after a new vb is queued and on a vpdma list
 * completion interrupt.
 */
static void start_dma(struct vip_dev *dev, struct vip_buffer *buf)
{
	struct vpdma_data *vpdma = dev->shared->vpdma;
	dma_addr_t dma_addr;
	int drop_data;

	if (vpdma_list_busy(vpdma, dev->slice_id)) {
		v4l2_err(&dev->v4l2_dev, "vpdma list busy, cannot post");
		return;				/* nothing to do */
	}

	if (buf) {
		dma_addr = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
		drop_data = 0;
	} else {
		dma_addr = NULL;
		drop_data = 1;
	}

	if (buf)
		dma_addr_global_complete[buf->vb.v4l2_buf.index] = dma_addr;

	enable_irqs(dev, dev->slice_id);

	vpdma_update_dma_addr(dev->shared->vpdma, &dev->desc_list,
				dma_addr, drop_data);
	vpdma_submit_descs(dev->shared->vpdma, &dev->desc_list, dev->slice_id);
}

static void vip_active_buf_next(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	struct vip_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&stream->vidq)) {
		v4l2_dbg(1, debug, &dev->v4l2_dev, "%s No buffers to queue, dropping frame, Queue faster or increase no of buffers");
		buf = kzalloc(sizeof(*buf), GFP_KERNEL);
                if (!buf) {
                    v4l2_err(&dev->v4l2_dev, "No memory!!");
                    spin_unlock_irqrestore(&dev->slock, flags);
                    return;
                }
                buf->drop = true;
                list_add_tail(&buf->list, &dev->vip_bufs);
                spin_unlock_irqrestore(&dev->slock, flags);
                start_dma(dev, NULL);
        } else if (vb2_is_streaming(&stream->vb_vidq)) {
		buf = list_entry(stream->vidq.next,
				struct vip_buffer, list);
		list_move_tail(&buf->list, &dev->vip_bufs);
		spin_unlock_irqrestore(&dev->slock, flags);
		start_dma(dev, buf);
	} else
            spin_unlock_irqrestore(&dev->slock, flags);

	if (list_empty(&dev->vip_bufs))
		stream->cur_buf = NULL;
	else
		stream->cur_buf = list_first_entry(&dev->vip_bufs,
				struct vip_buffer, list);
}

static void vip_process_buffer_complete(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;
	struct vb2_buffer *vb = NULL;
	unsigned long flags;

	if (stream->cur_buf) {
		vb = &stream->cur_buf->vb;
		do_gettimeofday(&vb->v4l2_buf.timestamp);

		spin_lock_irqsave(&dev->slock, flags);
		list_del(&stream->cur_buf->list);
		spin_unlock_irqrestore(&dev->slock, flags);

		if (stream->cur_buf->drop)
                    kfree(stream->cur_buf);
                else
                    vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
        }

	vip_active_buf_next(stream);
}

static irqreturn_t vip_irq(int irq_vip, void *data)
{
	struct vip_dev *dev = (struct vip_dev *)data;
	int list_num = dev->slice_id;
	int irq_num = dev->slice_id;
	u32 irqst, reg_addr;

	if (!dev->shared)
		return IRQ_HANDLED;

	reg_addr = VIP_INT0_STATUS0 +
			VIP_INTC_INTX_OFFSET * irq_num;
	irqst = read_sreg(dev->shared, reg_addr);

	if (irqst) {
		reg_addr = VIP_INT0_STATUS0_CLR +
			VIP_INTC_INTX_OFFSET * irq_num;
		write_sreg(dev->shared, reg_addr, irqst);
	}

	if (irqst) {
		if (irqst & 1 << (list_num * 2))
			vpdma_clear_list_stat(dev->shared->vpdma, list_num);

		irqst &= ~((1 << list_num * 2));
	}

	if (dev->num_skip_irq) {
		dev->num_skip_irq--;
		return IRQ_HANDLED;
	}

	/* disable_irqs(dev); */

	vip_process_buffer_complete(dev->ports[0]->cap_streams[0]);

	return IRQ_HANDLED;
}

/*
 * video ioctls
 */
static struct v4l2_mbus_framefmt *vip_video_pix_to_mbus(const struct v4l2_pix_format *pix,
				  struct v4l2_mbus_framefmt *mbus)
{
	unsigned int i;

	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	for (i = 0; i < ARRAY_SIZE(vip_formats) - 1; ++i) {
		if (vip_formats[i].fourcc == pix->pixelformat)
			break;
	}

	mbus->code = V4L2_MBUS_FMT_YUYV8_2X8;
	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;

	return mbus;
}

static int vip_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strncpy(cap->driver, VIP_MODULE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, VIP_MODULE_NAME, sizeof(cap->card) - 1);
	strlcpy(cap->bus_info, VIP_MODULE_NAME, sizeof(cap->bus_info));
	cap->device_caps  = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vip_enuminput(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index != 0)
		return -EINVAL;
	strncpy(inp->name, VIP_INPUT_NAME, sizeof(inp->name) - 1);
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int vip_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vip_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int vip_querystd(struct file *file, void *fh, v4l2_std_id *std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_dev *dev = stream->port->dev;

	v4l2_subdev_call(dev->sensor, video, querystd, std);
	return 0;
}

static int vip_g_std(struct file *file, void *fh, v4l2_std_id *std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_dev *dev = stream->port->dev;

	*std = 0;
	v4l2_subdev_call(dev->sensor, video, g_std_output, std);
	return 0;
}

static int vip_s_std(struct file *file, void *fh, v4l2_std_id *std)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_dev *dev = stream->port->dev;

	v4l2_subdev_call(dev->sensor, video, s_std_output, *std);
	return 0;
}

static int vip_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *a)
{
	return 0;
}

static int vip_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	return 0;
}

static int vip_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	return 0;
}

static int vip_enum_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	struct vip_fmt *fmt;

	if (f->index >= ARRAY_SIZE(vip_formats))
		return -EINVAL;

	fmt = &vip_formats[f->index];

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;

	return 0;
}

/*
  * TODO: Change from hard coding values to reading these through
  * IOCTLS directly from sensor
  */

static int vip_enum_framesizes(struct file *file, void *priv,
			struct v4l2_frmsizeenum *f)
{
	/* For now, hard coded resolutions for OV10635 sensor */
	int cam_width[7] =	{ 1280, 1280, 752, 640, 600, 352, 320 };
	int cam_height[7] =	{  800,  720, 480, 480, 400, 288, 240 };

	if (f->index >= 7)
		return -EINVAL;

	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	f->discrete.width = cam_width[f->index];
	f->discrete.height = cam_height[f->index];

	return 0;
}

static int vip_enum_frameintervals(struct file *file, void *priv,
			struct v4l2_frmivalenum *f)
{
	if (f->index >= ARRAY_SIZE(vip_formats))
		return -EINVAL;

	f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	f->discrete.numerator = 30;
	f->discrete.denominator = 1;

	return 0;
}

static int vip_s_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	parm->parm.capture.timeperframe.numerator = 30;
	parm->parm.capture.timeperframe.denominator = 1;

	return 0;
}

static int vip_g_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;

	f->fmt.pix.width	= stream->width;
	f->fmt.pix.height	= stream->height;
	f->fmt.pix.pixelformat	= port->fmt->fourcc;
	f->fmt.pix.field	= stream->sup_field;
	f->fmt.pix.colorspace	= port->fmt->colorspace;
	f->fmt.pix.bytesperline	= stream->bytesperline;
	f->fmt.pix.sizeimage	= stream->sizeimage;

	return 0;
}

static int vip_try_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_dev *dev = stream->port->dev;
	struct vip_fmt *fmt = find_format(f);
	enum v4l2_field field;
	int depth;

	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != field)
		return -EINVAL;

	f->fmt.pix.field = field;

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W, W_ALIGN,
			      &f->fmt.pix.height, MIN_H, MAX_H, H_ALIGN,
			      S_ALIGN);

	if (fmt->coplanar)
		depth = 8;

	f->fmt.pix.bytesperline = round_up((f->fmt.pix.width * fmt->vpdma_fmt[0]->depth) >> 3,
					   1 << L_ALIGN);
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.width * \
		(fmt->vpdma_fmt[0]->depth + (fmt->coplanar ? fmt->vpdma_fmt[1]->depth : 0)) >> 3;
	f->fmt.pix.colorspace = fmt->colorspace;

	return 0;
}

/*
 * Set the registers that are modified when the video format changes.
 */
static void set_fmt_params(struct vip_stream *stream)
{
	struct vip_dev *dev = stream->port->dev;

	stream->sequence = 0;
	stream->field = V4L2_FIELD_TOP;

	if (stream->port->fmt->colorspace == V4L2_COLORSPACE_SRGB) {
		vip_set_slice_path(dev, VIP_RGB_OUT_LO_DATA_SELECT);
		/* Set alpha component in background color */
		vpdma_set_bg_color(dev->shared->vpdma,
			stream->port->fmt->vpdma_fmt[0], 0xff);
	}
}

int vip_s_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vb2_queue *vq = &stream->vb_vidq;
	struct v4l2_subdev_format sfmt;
	struct v4l2_mbus_framefmt *mf;
	int ret;

	ret = vip_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

#if 0
	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}
#endif
	port->fmt		= find_format(f);
	stream->width		= f->fmt.pix.width;
	stream->height		= f->fmt.pix.height;
	port->fmt->colorspace	= f->fmt.pix.colorspace;
	stream->bytesperline	= f->fmt.pix.bytesperline;
	stream->sizeimage	= f->fmt.pix.sizeimage;
	stream->sup_field	= f->fmt.pix.field;

	port->c_rect.left	= 0;
	port->c_rect.top	= 0;
	port->c_rect.width	= stream->width;
	port->c_rect.height	= stream->height;

	if (stream->sup_field == V4L2_FIELD_ALTERNATE)
		port->flags |= FLAG_INTERLACED;
	else
		port->flags &= ~FLAG_INTERLACED;

	vip_dprintk(dev,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, stream->width, stream->height, port->fmt->fourcc);

	set_fmt_params(stream);

	mf = vip_video_pix_to_mbus(&f->fmt.pix, &sfmt.format);

	ret = v4l2_subdev_call(dev->sensor, video, try_mbus_fmt, mf);
	if (ret) {
		vip_dprintk(dev, "try_mbus_fmt failed in subdev\n");
		return ret;
	}
	ret = v4l2_subdev_call(dev->sensor, video, s_mbus_fmt, mf);
	if (ret) {
		vip_dprintk(dev, "s_mbus_fmt failed in subdev\n");
		return ret;
	}

	vip_setup_parser(dev->ports[0]);

	return 0;
}
EXPORT_SYMBOL(vip_s_fmt_vid_cap);

static int vip_g_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vip_stream *stream = file2stream(file);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = stream->width;
		s->r.height = stream->height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_CROP:
		s->r = stream->port->c_rect;
		return 0;
	}

	return -EINVAL;
}

static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;
	if (a->left + a->width > b->left + b->width)
		return 0;
	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

static int vip_s_selection(struct file *file, void *fh,
			   struct v4l2_selection *s)
{
	struct vip_stream *stream = file2stream(file);
	struct vip_port *port = stream->port;
	struct v4l2_rect r = s->r;

	v4l_bound_align_image(&r.width, 0, stream->width, 0,
			      &r.height, 0, stream->height, 0, 0);

	r.left = clamp_t(unsigned int, r.left, 0, stream->width - r.width);
	r.top  = clamp_t(unsigned int, r.top, 0, stream->height - r.height);

	if (s->flags & V4L2_SEL_FLAG_LE && !enclosed_rectangle(&r, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE && !enclosed_rectangle(&s->r, &r))
		return -ERANGE;

	s->r = stream->port->c_rect = r;

	set_fmt_params(stream);

	vip_dprintk(port->dev, "cropped (%d,%d)/%dx%d of %dx%d\n",
		    r.left, r.top, r.width, r.height,
		    stream->width, stream->height);

	return 0;
}

static long vip_ioctl_default(struct file *file, void *fh, bool valid_prio,
			      int cmd, void *arg)
{
	switch (cmd) {
	default:
		return -ENOTTY;
	}
}

static const struct v4l2_ioctl_ops vip_ioctl_ops = {
	.vidioc_querycap	= vip_querycap,
	.vidioc_enum_input	= vip_enuminput,
	.vidioc_g_input		= vip_g_input,
	.vidioc_s_input		= vip_s_input,

	.vidioc_querystd	= vip_querystd,
	.vidioc_g_std		= vip_g_std,
	.vidioc_s_std		= vip_s_std,

	.vidioc_queryctrl	= vip_queryctrl,
	.vidioc_g_ctrl		= vip_g_ctrl,
	.vidioc_s_ctrl		= vip_s_ctrl,

	.vidioc_enum_fmt_vid_cap = vip_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vip_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vip_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vip_s_fmt_vid_cap,

	.vidioc_enum_frameintervals 	= vip_enum_frameintervals,
	.vidioc_enum_framesizes		= vip_enum_framesizes,
	.vidioc_s_parm			= vip_s_parm,

	.vidioc_g_selection	= vip_g_selection,
	.vidioc_s_selection	= vip_s_selection,
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,

	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,
	.vidioc_log_status	= v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_default		= vip_ioctl_default,
};

/*
 * Videobuf operations
 */
static int vip_queue_setup(struct vb2_queue *vq,
			   const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_dev *dev = stream->port->dev;

	*nplanes = 1;
	sizes[0] = stream->sizeimage;
	alloc_ctxs[0] = dev->alloc_ctx;
	vip_dprintk(dev, "get %d buffer(s) of size %d each.\n",
		    *nbuffers, sizes[0]);

	return 0;
}

static int vip_buf_prepare(struct vb2_buffer *vb)
{
	struct vip_stream *stream = vb2_get_drv_priv(vb->vb2_queue);
	struct vip_dev *dev = stream->port->dev;

	if (vb2_plane_size(vb, 0) < stream->sizeimage) {
		vip_dprintk(dev,
			    "%s data will not fit into plane (%lu < %lu)\n",
			    __func__, vb2_plane_size(vb, 0),
			    (long)stream->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, stream->sizeimage);

	return 0;
}

static void vip_buf_queue(struct vb2_buffer *vb)
{
	struct vip_stream *stream = vb2_get_drv_priv(vb->vb2_queue);
	struct vip_dev *dev = stream->port->dev;
	struct vip_buffer *buf = container_of(vb, struct vip_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	list_add_tail(&buf->list, &stream->vidq);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int vip_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_buffer *buf;
	unsigned long flags, i;

	if (count <= VIP_VPDMA_FIFO_SIZE) {
		vip_dprintk(dev, "Less buffers queued, at least %d needed\n",
				VIP_VPDMA_FIFO_SIZE + 1);
		return -EIO;
	}

	stream->cur_buf = list_entry(stream->vidq.next,
				struct vip_buffer, list);
	stream->cur_buf->vb.state = VB2_BUF_STATE_ACTIVE;
	stream->field = V4L2_FIELD_TOP;

	populate_desc_list(stream);
	dev->num_skip_irq = VIP_VPDMA_FIFO_SIZE;

	for (i = 0; i < count; i++) {

		spin_lock_irqsave(&dev->slock, flags);
		if (!vpdma_list_busy(dev->shared->vpdma, dev->slice_id)) {

			buf = list_entry(stream->vidq.next,
					struct vip_buffer, list);
			list_move_tail(&buf->list, &dev->vip_bufs);
			spin_unlock_irqrestore(&dev->slock, flags);

			if (!stream->cur_buf)
				stream->cur_buf = buf;
			start_dma(dev, buf);

			if (i == VIP_VPDMA_FIFO_SIZE)
				break;
			while (vpdma_list_busy(dev->shared->vpdma,
					dev->slice_id))
				;
		} else
			spin_unlock_irqrestore(&dev->slock, flags);
	}

	return 0;
}

/*
 * Abort streaming and wait for last buffer
 */
static int vip_stop_streaming(struct vb2_queue *vq)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vip_buffer *buf;

	if (!vb2_is_streaming(vq))
		return 0;

	vpdma_buf_unmap(dev->shared->vpdma, &dev->desc_list.buf);
	vpdma_reset_desc_list(&dev->desc_list);

	disable_irqs(dev, dev->slice_id);
	/* release all active buffers */
	while (!list_empty(&dev->vip_bufs)) {
		buf = list_entry(dev->vip_bufs.next, struct vip_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	while (!list_empty(&stream->vidq)) {
		buf = list_entry(stream->vidq.next, struct vip_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	return 0;
}

/*
 * Lock access to the device
 */
static void vip_lock(struct vb2_queue *vq)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);

	mutex_lock(&stream->port->dev->mutex);
}

static void vip_unlock(struct vb2_queue *vq)
{
	struct vip_stream *stream = vb2_get_drv_priv(vq);
	mutex_unlock(&stream->port->dev->mutex);
}

static struct vb2_ops vip_video_qops = {
	.queue_setup		= vip_queue_setup,
	.buf_prepare		= vip_buf_prepare,
	.buf_queue			= vip_buf_queue,
	.start_streaming	= vip_start_streaming,
	.stop_streaming		= vip_stop_streaming,
	.wait_prepare		= vip_unlock,
	.wait_finish		= vip_lock,
};

/*
 * File operations
 */

static int vip_init_dev(struct vip_dev *dev)
{
	int ret;

	if (dev->num_ports != 0)
		goto done;

	ret = vpdma_create_desc_list(&dev->desc_list, VIP_DESC_LIST_SIZE,
			VPDMA_LIST_TYPE_NORMAL);

	if (ret != 0)
		return ret;

	vip_set_clock_enable(dev, 1);
done:
	dev->num_ports++;

	return 0;
}

static void vip_release_dev(struct vip_dev *dev)
{
	vpdma_buf_free(&dev->desc_list.buf);
	vpdma_free_desc_list(&dev->desc_list);

	/*
	 * On last close, disable clocks to conserve power
	 */

	if (--dev->num_ports == 0)
		vip_set_clock_enable(dev, 0);
}

static int vip_setup_parser(struct vip_port *port)
{
	struct vip_dev *dev = port->dev;
	struct v4l2_of_endpoint *endpoint = dev->endpoint;
	int iface = DUAL_8B_INTERFACE;
	int sync_type;
	unsigned int flags;

	vip_set_port_enable(port, 1);

	if (endpoint->bus_type == V4L2_MBUS_BT656) {
		iface = DUAL_8B_INTERFACE;

		/* Ideally, this should come from sensor
		   port->fmt can be anything once CSC is enabled */
		if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB)
			sync_type = EMBEDDED_SYNC_SINGLE_RGB_OR_YUV444;
		else {
			switch (endpoint->bus.parallel.num_channels) {
			case 4:
				sync_type = EMBEDDED_SYNC_4X_MULTIPLEXED_YUV422;
			break;
			case 2:
				sync_type = EMBEDDED_SYNC_2X_MULTIPLEXED_YUV422;
			break;
			case 1:
			default:
				sync_type = EMBEDDED_SYNC_SINGLE_YUV422;
			}
		}

	} else if (endpoint->bus_type == V4L2_MBUS_PARALLEL) {
		switch (endpoint->bus.parallel.bus_width) {
		case 24:
			iface = SINGLE_24B_INTERFACE;
		break;
		case 16:
			iface = SINGLE_16B_INTERFACE;
		break;
		case 8:
		default:
			iface = DUAL_8B_INTERFACE;
		}

		if (port->fmt->colorspace == V4L2_COLORSPACE_SRGB)
			sync_type = DISCRETE_SYNC_SINGLE_RGB_24B;
		else
			sync_type = DISCRETE_SYNC_SINGLE_YUV422;

		flags = endpoint->bus.parallel.flags;
		if (flags & (V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_HSYNC_ACTIVE_LOW))
			vip_set_vsync_polarity(port,
				flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH ? 1 : 0);

		if (flags & (V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_LOW))
			vip_set_hsync_polarity(port,
				flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH ? 1 : 0);

		if (flags & (V4L2_MBUS_PCLK_SAMPLE_RISING |
			V4L2_MBUS_PCLK_SAMPLE_FALLING))
			vip_set_pclk_polarity(port,
				flags & V4L2_MBUS_PCLK_SAMPLE_RISING ? 1 : 0);

		vip_set_actvid_polarity(port, 1);
		vip_set_discrete_basic_mode(port);

	} else {
		v4l2_err(&port->dev->v4l2_dev, "Device doesn't support CSI2");
		return -EINVAL;
	}

	vip_set_data_interface(port, iface);
	vip_sync_type(port, sync_type);
}

static int vip_init_port(struct vip_port *port)
{
	int ret;

	if (port->num_streams != 0)
		goto done;

	ret = vip_init_dev(port->dev);
	if (ret)
		goto done;

	port->fmt = &vip_formats[5];
	port->src_colorspace = port->fmt->colorspace;
	port->c_rect.left = 0;
	port->c_rect.top = 0;

done:
	port->num_streams++;
	return 0;
}

static void vip_release_port(struct vip_port *port)
{
	if (--port->num_streams == 0)
		vip_release_dev(port->dev);
}

dma_addr_t dma_addr_global_complete[VIP_VPDMA_MAX_BUFFER_COUNT];
EXPORT_SYMBOL(dma_addr_global_complete);

int vip_open(struct file *file)
{
	struct vip_stream *stream = video_drvdata(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct platform_device *pdev = dev->pdev;
	int ret;

	mutex_lock(&dev->mutex);
	if (stream->open) {
		v4l2_warn(&dev->v4l2_dev,
			    "%s stream is already open\n", __func__);
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	if (vb2_is_busy(&stream->vb_vidq)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	if (!dev->setup_done) {
		dev->setup_done = 1;
		vip_top_reset(dev);
		ret = find_or_alloc_shared(pdev, dev, dev->res);
		if (ret)
			goto done;

		dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
		if (IS_ERR(dev->alloc_ctx)) {
			v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
			ret = PTR_ERR(dev->alloc_ctx);
			goto done;
		}

		vip_set_idle_mode(dev->shared, VIP_SMART_IDLE_MODE);
		vip_set_standby_mode(dev->shared, VIP_SMART_STANDBY_MODE);
		vip_set_slice_path(dev, VIP_MULTI_CHANNEL_DATA_SELECT);
	}

	ret = vip_init_port(port);
	if (ret)
		goto done;

	stream->width = 1280;
	stream->height = 720;
	stream->sizeimage = stream->width * stream->height *
			(port->fmt->vpdma_fmt[0]->depth + (port->fmt->coplanar ?
				port->fmt->vpdma_fmt[1]->depth : 0)) >> 3;
	stream->sup_field = V4L2_FIELD_NONE;
	port->c_rect.width = stream->width;
	port->c_rect.height = stream->height;

	v4l2_fh_init(&stream->fh, video_devdata(file));
	file->private_data = &stream->fh;

	v4l2_fh_add(&stream->fh);

	vip_dprintk(dev, "Created stream instance %p\n", stream);

	stream->open = 1;
	mutex_unlock(&dev->mutex);

	return 0;

done:
	mutex_unlock(&dev->mutex);
	return ret;
}
EXPORT_SYMBOL(vip_open);

int vip_release(struct file *file)
{
	struct vip_stream *stream = video_drvdata(file);
	struct vip_port *port = stream->port;
	struct vip_dev *dev = port->dev;
	struct vb2_queue *q = &stream->vb_vidq;
	int i;

	vip_dprintk(dev, "Releasing stream instance %p\n", stream);

	mutex_lock(&dev->mutex);

	vip_stop_streaming(q);
	vip_release_port(stream->port);

	if (file->private_data) {
		v4l2_fh_del((struct v4l2_fh *)file->private_data);
		v4l2_fh_exit((struct v4l2_fh *)file->private_data);
	}
	vb2_queue_release(q);

	for (i = 0; i < VIP_VPDMA_MAX_BUFFER_COUNT; i++)
		dma_addr_global_complete[i] = (dma_addr_t)NULL;

	stream->open = 0;
	mutex_unlock(&dev->mutex);

	return 0;
}
EXPORT_SYMBOL(vip_release);

static const struct v4l2_file_operations vip_fops = {
	.owner		= THIS_MODULE,
	.open		= vip_open,
	.release	= vip_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

static struct video_device vip_videodev = {
	.name		= VIP_MODULE_NAME,
	.fops		= &vip_fops,
	.ioctl_ops	= &vip_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

static int alloc_stream(struct vip_port *port, int stream_id, int vfl_type)
{
	struct vip_stream *stream;
	struct vip_dev *dev = port->dev;
	struct vb2_queue *q;
	struct video_device *vfd;
	int ret;

	stream = kzalloc (sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->port = port;
	stream->stream_id = stream_id;
	stream->vfl_type = vfl_type;

	if (vfl_type == VFL_TYPE_GRABBER)
		port->cap_streams[stream_id] = stream;
	else
		port->vbi_streams[stream_id] = stream;

	/*
	 * Initialize queue
	 */
	q = &stream->vb_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = stream;
	q->buf_struct_size = sizeof(struct vip_buffer);
	q->ops = &vip_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	ret = vb2_queue_init(q);
	if (ret)
		goto do_free_stream;

	INIT_LIST_HEAD(&stream->vidq);

	vfd = &stream->vdev;
	*vfd = vip_videodev;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->queue = q;

	vfd->lock = &dev->mutex;
	video_set_drvdata(vfd, stream);

	ret = video_register_device(vfd, vfl_type, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto do_free_stream;
	}

	snprintf(vfd->name, sizeof(vfd->name), "%s", vip_videodev.name);
	stream->vfd = vfd;

	v4l2_info(&dev->v4l2_dev, VIP_MODULE_NAME
			" Device registered as /dev/video%d\n", vfd->num);
	return 0;

do_free_stream:
	kfree(stream);
	return ret;
}

static void free_stream(struct vip_stream *stream)
{
	if (!stream)
		return;

	video_unregister_device(stream->vfd);
	video_device_release(stream->vfd);
	kfree(stream);
}

static int alloc_port(struct vip_dev *dev, int id)
{
	struct vip_port *port;
	int ret;

	port = kzalloc (sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	dev->ports[id] = port;
	port->dev = dev;
	port->port_id = id;
	port->num_streams = 0;

	ret = alloc_stream(port, 0, VFL_TYPE_GRABBER);

	return 0;
}

static void free_port(struct vip_port *port)
{
	if (!port)
		return;

	free_stream(port->cap_streams[0]);

	kfree(port);
}

static int get_field(u32 value, u32 mask, int shift)
{
	return (value & (mask << shift)) >> shift;
}

static int find_or_alloc_shared(struct platform_device *pdev, struct vip_dev *dev,
		struct resource *res)
{
	struct vip_shared *shared;
	struct vpdma_data *vpdma;
	u32 pid;
	u32 tmp;
	int ret;

	mutex_lock(&vip_driver_mutex);
	shared = platform_get_drvdata(pdev);
	if (shared) {
		/* Use existing shared structure */
		dev->shared = shared;
		shared->devs[atomic_read(&shared->devs_allocated)] = dev;
		atomic_inc(&shared->devs_allocated);
		mutex_unlock(&vip_driver_mutex);
		return 0;
	}

	shared = kzalloc (sizeof(*shared), GFP_KERNEL);
	if (!shared) {
		ret = -ENOMEM;
		goto unlock;
	}

	shared->res = res;

	if (devm_request_mem_region(&pdev->dev, res->start,
	    resource_size(res), VIP_MODULE_NAME) == NULL) {
		ret = -ENOMEM;
		goto free_shared;
	}
	shared->base = devm_ioremap(&pdev->dev, res->start,
				    resource_size(res));

	if (!shared->base) {
		ret = -ENOMEM;
		goto rel_mem_region;
	}

	vpdma = shared->vpdma = &shared->vpdma_storage;
	vpdma->base = shared->base + VIP_VPDMA_REG_OFFSET;

	dev->shared = shared;

	pid = read_sreg(shared, VIP_PID);

	tmp = get_field(pid, VIP_PID_FUNC_MASK, VIP_PID_FUNC_SHIFT);

	if (tmp != VIP_PID_FUNC) {
		vip_dprintk(dev, "vip: unexpected PID function: 0x%x\n",
		       tmp);
		ret = -ENODEV;
		goto do_iounmap;
	}

	vip_top_vpdma_reset(shared);

	ret = vpdma_init(pdev, &shared->vpdma);

	shared->devs[0] = dev;
	atomic_set(&shared->devs_allocated, 1);

	list_add_tail(&shared->list, &vip_shared_list);

	platform_set_drvdata(pdev, shared);
	mutex_unlock(&vip_driver_mutex);

	ret = 0;
	goto unlock;

do_iounmap:
	iounmap(shared->base);
rel_mem_region:
	release_mem_region(res->start, resource_size(res));
free_shared:
	kfree(shared);
unlock:
	return ret;
}

static void remove_shared(struct vip_shared *shared)
{
	if (atomic_dec_return(&shared->devs_allocated) != 0)
		return;

	iounmap(shared->base);
	release_mem_region(shared->res->start, resource_size(shared->res));
	kfree(shared);
}

static int vip_runtime_get(struct platform_device *pdev)
{
	int r;

	r = pm_runtime_get_sync(&pdev->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

static int vip_async_bound(struct v4l2_async_notifier *notifier,
			struct v4l2_subdev *subdev,
			struct v4l2_async_subdev *asd)
{
	struct vip_dev *dev = notifier_to_vip_dev(notifier);
	unsigned int idx = asd - &dev->config->asd[0];

	if (idx > dev->config->asd_sizes)
		return -EINVAL;

	if (dev->sensor) {
		if (asd < dev->sensor->asdl.asd) {
			/* Notified of a subdev earlier in the array */
			v4l2_info(&dev->v4l2_dev, "Switching to subdev i2c address %x (High priority)",
				asd->match.i2c.address);
		} else {
			v4l2_info(&dev->v4l2_dev, "Rejecting subdev i2c address %x (Low priority)",
				asd->match.i2c.address);
			return 0;
		}
	} else
		alloc_port(dev, 0);

	dev->sensor = subdev;
	dev->endpoint = &dev->config->endpoints[idx];
	v4l2_info(&dev->v4l2_dev, "Using sensor i2c addr %x for capture\n",
		asd->match.i2c.address);
	return 0;
}

static int vip_async_complete(struct v4l2_async_notifier *notifier)
{
	printk(KERN_NOTICE "vip_async_complete\n");
	return 0;
}

static struct v4l2_async_subdev vip_sensor_subdev = {
	.bus_type = V4L2_ASYNC_BUS_I2C,
	.match.i2c = {
		.adapter_id = 1,
		.address = 0x30,
	}
};

static int vip_of_probe(struct platform_device *pdev, struct vip_dev *dev)
{
	struct device_node *ep_node = NULL, *port, *remote_ep, *sensor_node;
	struct v4l2_of_endpoint *endpoint;
	struct v4l2_async_subdev *asd;
	u32 sensor_addr, regval = 0;
	int ret, i = 0;

	dev->config = kzalloc(sizeof(struct vip_config), GFP_KERNEL);
	if (!dev->config)
		return -ENOMEM;

	dev->config->card_name = "DRA7XX VIP Driver";

	while (i < VIP_MAX_SUBDEV) {

		asd = &dev->config->asd[i];
		endpoint = &dev->config->endpoints[i];
		*asd = vip_sensor_subdev;

		ep_node = v4l2_of_get_next_endpoint(pdev->dev.of_node, ep_node);
		if (!ep_node)
			break;

		/* Check wheather the endpoint belongs to correct slice */
		port = of_get_next_parent(ep_node);
		of_property_read_u32(port, "reg", &regval);
		if (regval != dev->slice_id)
			continue;

		sensor_node = v4l2_of_get_remote_port_parent(ep_node);
		if (!sensor_node)
			continue;

		of_property_read_u32(sensor_node, "reg", &sensor_addr);
		v4l2_err(&dev->v4l2_dev, "Waiting for I2C subdevice %x",
				sensor_addr);
		asd->match.i2c.address = sensor_addr;

		remote_ep = of_parse_phandle(ep_node, "remote-endpoint", 0);
		if (!remote_ep)
			continue;

		v4l2_of_parse_endpoint(remote_ep, endpoint);

		dev->config->asd_list[i++] = asd;
	}

	dev->config->asd_sizes = i;
	dev->notifier.subdev = dev->config->asd_list;
	dev->notifier.num_subdevs = dev->config->asd_sizes;
	dev->notifier.bound = vip_async_bound;
	dev->notifier.complete = vip_async_complete;

	ret = v4l2_async_notifier_register(&dev->v4l2_dev, &dev->notifier);
	if (ret) {
		vip_dprintk(dev, "Error registering async notifier\n");
		ret = -EINVAL;
		goto free_config;
	}

	return 0;
free_config:
	kfree(dev->config);
	return ret;
}

static const struct of_device_id vip_of_match[];

static int vip_probe(struct platform_device *pdev)
{
	struct vip_dev *dev;
	struct resource *res;
	const struct of_device_id *of_dev_id;
	struct pinctrl *pinctrl;
	void __iomem *base;
	int ret, slice = VIP_SLICE1;

	mutex_init(&vip_driver_mutex);
	pm_runtime_enable(&pdev->dev);

	ret = vip_runtime_get(pdev);
	if (ret)
		goto err_runtime_get;

	of_dev_id = of_match_device(vip_of_match, &pdev->dev);
	if (!of_dev_id) {
		dev_err(&pdev->dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Missing platform resources data\n");
		ret = -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		goto err_runtime_get;
	}

	base = devm_ioremap(&pdev->dev, res->start, SZ_64K);
	if (!base)
		ret = -ENOMEM;

	for (slice = VIP_SLICE1; slice < VIP_NUM_SLICES; slice++) {
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev)
			return -ENOMEM;

		dev->irq = platform_get_irq(pdev, slice);
		if (!dev->irq) {
			dev_err(&pdev->dev, "Could not get IRQ");
			goto err_runtime_get;
		}

		if (devm_request_irq(&pdev->dev, dev->irq, vip_irq,
				     0, VIP_MODULE_NAME, dev) < 0) {
			ret = -ENOMEM;
			goto dev_unreg;
		}

		spin_lock_init(&dev->slock);
		spin_lock_init(&dev->lock);

		INIT_LIST_HEAD(&dev->vip_bufs);

		dev->vip_name = (const char *)of_dev_id->data;

		snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
			"%s %s-%d", VIP_MODULE_NAME, dev->vip_name, slice);

		ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
		if (ret)
			goto err_runtime_get;

		mutex_init(&dev->mutex);

		dev->slice_id = slice;
		dev->pdev = pdev;
		dev->res = res;
		dev->base = base;

		if (dev->pdev->dev.of_node) {
			ret = vip_of_probe(pdev, dev);
			if (ret)
				goto free_port;
		}
	}

	platform_set_drvdata(pdev, NULL);

	return 0;

free_port:
	free_port(dev->ports[0]);
dev_unreg:
	v4l2_device_unregister(&dev->v4l2_dev);
err_runtime_get:
	if (slice == VIP_SLICE1) {
		pm_runtime_disable(&pdev->dev);
		return ret;
	} else
		return 0;
}

static int vip_remove(struct platform_device *pdev)
{
	struct vip_shared *shared = platform_get_drvdata(pdev);
	struct vip_dev *dev;
	int slice;

	for (slice = 0; slice < atomic_read(&shared->devs_allocated); slice++) {
		dev = shared->devs[slice];
		if (!dev)
			continue;
		v4l2_info(&dev->v4l2_dev, "Removing " VIP_MODULE_NAME);
		free_port(dev->ports[0]);
		v4l2_async_notifier_unregister(&dev->notifier);
		vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
		free_irq(dev->irq, dev);
		kfree(dev);
	}
	remove_shared(shared);

	return 0;
}

#if defined (CONFIG_OF)
static const struct of_device_id vip_of_match[] = {
	{
		.compatible = "ti,vip1", .data = "vip1",
	},
#if 0
	{
		.compatible = "ti,vip2", .data = "vip2",
	},
#endif
	{
		.compatible = "ti,vip3", .data = "vip3",
	},
	{},
};
#else
#define vip_of_match NULL
#endif

static struct platform_driver vip_pdrv = {
	.probe		= vip_probe,
	.remove		= vip_remove,
	.driver		= {
		.name	= VIP_MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = vip_of_match,
	},
};

static void __exit vip_exit(void)
{
	platform_driver_unregister(&vip_pdrv);
}

static int __init vip_init(void)
{
	return platform_driver_register(&vip_pdrv);
}

module_init(vip_init);
module_exit(vip_exit);

