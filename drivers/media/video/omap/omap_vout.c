/*
 * drivers/media/video/omap/omap_vout.c
 *
 * Copyright (C) 2005-2009 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP2 camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2009 Texas Instruments.
 *
 * History:
 * 20-APR-2006	Khasim		Modified VRFB based Rotation,
 *				The image data is always read from 0 degree
 *				view and written
 *				to the virtual space of desired rotation angle
 * 4-DEC-2006 Jian		Changed to support better memory management
 *
 * 17-Nov-2008 Hardik		Changed to used the new DSS paches by Tomi
 *				Changed driver to use video_ioctl2
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/delay.h>

#ifndef CONFIG_ARCH_OMAP4
#include <mach/isp_user.h>
#include <linux/ispdss.h>
#include "../isp/ispresizer.h"
#endif

#include <media/videobuf-dma-sg.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>

#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <plat/dma.h>
#include <plat/vram.h>
#include <plat/vrfb.h>
#include <plat/display.h>

#include "omap_voutlib.h"
#include "omap_voutdef.h"

#ifdef CONFIG_TILER_OMAP
#include <mach/tiler.h>
#define TILER_ALLOCATE_V4L2
#endif

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP Video for Linux Video out driver");
MODULE_LICENSE("GPL");

#define OMAP_VIDEO1 0
#define OMAP_VIDEO2 1
#define OMAP_VIDEO3 2

/* configuration macros */
#define VOUT_NAME		"omap_vout"

#define QQVGA_WIDTH		160
#define QQVGA_HEIGHT		120

#ifdef CONFIG_ARCH_OMAP4
#define OMAP4_MAX_OVERLAYS	4
#define NUM_OF_VIDEO_CHANNELS	3
#define VID_MAX_WIDTH		2048 /* Largest width */
#define VID_MAX_HEIGHT		2048 /* Largest height */
#define OMAP_VOUT_MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*4)
#else
#define NUM_OF_VIDEO_CHANNELS	2
#define VID_MAX_WIDTH		1280 /* Largest width */
#define VID_MAX_HEIGHT		720  /* Largest height */
#define OMAP_VOUT_MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*2)
#endif
/* Mimimum requirement is 2x2 for DSS */
#define VID_MIN_WIDTH		2
#define VID_MIN_HEIGHT		2

/* 2048 x 2048 is max res supported by OMAP display controller */
#define DMA_CHAN_ALLOTED        1
#define DMA_CHAN_NOT_ALLOTED    0
#define MAX_PIXELS_PER_LINE     2048
#define VRFB_TX_TIMEOUT         1000

#define VDD1_FREQ_CONST	(cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 600000000 : 500000000) : 0)
#define VDD1_FREQ_MIN	(cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 300000000 : 125000000) : 0)
#define VDD2_OCP_FREQ_CONST	(cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 200000 : 166000) : 0)

int cacheable_buffers;
int flushable_buffers;

extern void tick_nohz_disable(int nohz);

#ifndef CONFIG_ARCH_OMAP4
/* ISP resizer related code*/
struct isp_node pipe;
static int vrfb_configured;
#endif
/* End ISP resizer*/

static struct videobuf_queue_ops video_vbq_ops;

static u32 video1_numbuffers = OMAP_VOUT_MAX_BUFFERS;
static u32 video2_numbuffers = OMAP_VOUT_MAX_BUFFERS;
static u32 video3_numbuffers = 3;
static u32 video1_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 video2_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 video3_bufsize = OMAP_VOUT_MAX_BUF_SIZE;

static u32 vid1_static_vrfb_alloc;
static u32 vid2_static_vrfb_alloc;
static int debug;

struct mutex my_lock;
/* Module parameters */
module_param(video1_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video1_numbuffers,
	"Number of buffers to be allocated at init time for Video1 device.");

module_param(video2_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video2_numbuffers,
	"Number of buffers to be allocated at init time for Video2 device.");

module_param(video1_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video1_bufsize,
	"Size of the buffer to be allocated for video1 device");

module_param(video2_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video2_bufsize,
	"Size of the buffer to be allocated for video2 device");

#ifdef CONFIG_ARCH_OMAP4
module_param(video3_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video3_numbuffers, "Number of buffers to be allocated at \
		init time for Video3 device.");

module_param(video3_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video1_bufsize, "Size of the buffer to be allocated for \
		video3 device");
#endif

#ifdef CONFIG_OMAP2_VRFB
module_param(vid1_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid1_static_vrfb_alloc,
	"Static allocation of the VRFB buffer for video1 device");

module_param(vid2_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid2_static_vrfb_alloc,
	"Static allocation of the VRFB buffer for video2 device");
#endif

module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/* Local Helper functions */
static int omap_vout_create_video_devices(struct platform_device *pdev);
static int omapvid_apply_changes(struct omap_vout_device *vout);
static int omapvid_init(struct omap_vout_device *vout, u32 addr, u32 uv_addr);
static int omapvid_setup_overlay(struct omap_vout_device *vout,
		struct omap_overlay *ovl, int posx, int posy,
		int outw, int outh, u32 addr, u32 uv_addr);
static enum omap_color_mode video_mode_to_dss_mode(
		struct v4l2_pix_format *pix);
static void omap_vout_isr(void *arg, unsigned int irqstatus);
static void omap_vout_cleanup_device(struct omap_vout_device *vout);
/*
 * Maximum amount of memory to use for rendering buffers.
 * Default is enough to four (RGB24) DVI 720P buffers.
 */
#define MAX_ALLOWED_VIDBUFFERS            6

/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap_formats[] = {
	{
	 /* Note:  V4L2 defines RGB565 as:
	  *
	  *      Byte 0                    Byte 1
	  *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
	  *
	  * We interpret RGB565 as:
	  *
	  *      Byte 0                    Byte 1
	  *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
	  */
	 .description = "RGB565, le",
	 .pixelformat = V4L2_PIX_FMT_RGB565,
	 },
	{
	 /* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
	  *  this for RGB24 unpack mode, the last 8 bits are ignored
	  * */
	 .description = "RGB32, le",
	 .pixelformat = V4L2_PIX_FMT_RGB32,
	 },
	{
	 /* Note:  V4L2 defines RGB24 as: RGB-8-8-8  we use
	  *        this for RGB24 packed mode
	  *
	  */
	 .description = "RGB24, le",
	 .pixelformat = V4L2_PIX_FMT_RGB24,
	 },
	{
	 .description = "YUYV (YUV 4:2:2), packed",
	 .pixelformat = V4L2_PIX_FMT_YUYV,
	 },
	{
	 .description = "UYVY, packed",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	 },
	{
	 .description = "NV12 - YUV420 format",
	 .pixelformat = V4L2_PIX_FMT_NV12,
	},

};

#define NUM_OUTPUT_FORMATS (ARRAY_SIZE(omap_formats))

#ifndef TILER_ALLOCATE_V4L2
/* Allocate buffers */
static unsigned long omap_vout_alloc_buffer(u32 buf_size, u32 *phys_addr)
{
	unsigned long virt_addr, addr;
	u32 size;

	size = PAGE_ALIGN(buf_size);
	virt_addr = (u32) alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
	addr = virt_addr;
	if (virt_addr) {
		while (size > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}
	*phys_addr = (u32) virt_to_phys((void *) virt_addr);
	return virt_addr;
}
#endif

/* Free buffers */
static void omap_vout_free_buffer(unsigned long virtaddr, u32 phys_addr,
			 u32 buf_size)
{
	unsigned long addr = virtaddr;
	u32 size;

	size = PAGE_ALIGN(buf_size);
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages_exact((void *) virtaddr, size);
}

#ifndef CONFIG_ARCH_OMAP4
/* Function for allocating video buffers */
static int omap_vout_allocate_vrfb_buffers(struct omap_vout_device *vout,
		unsigned int *count, int startindex)
{
	int i, j;
	int buffer_set;

	for (i = 0; i < *count; i++) {
		buffer_set = vout->smsshado_virt_addr[i];
		if (!buffer_set) {
			vout->smsshado_virt_addr[i] =
				omap_vout_alloc_buffer(vout->smsshado_size,
						&vout->smsshado_phy_addr[i]);
		}
		if (!vout->smsshado_virt_addr[i] && startindex != -1) {
			if (V4L2_MEMORY_MMAP == vout->memory
				&& i >= startindex)
				break;
		}
		if (!vout->smsshado_virt_addr[i]) {
			for (j = 0; j < i; j++) {
				omap_vout_free_buffer(
						vout->smsshado_virt_addr[j],
						vout->smsshado_phy_addr[j],
						vout->smsshado_size);
				vout->smsshado_virt_addr[j] = 0;
				vout->smsshado_phy_addr[j] = 0;
			}
			*count = 0;
			return -ENOMEM;
		}
		if (!buffer_set) {
			memset((void *) vout->smsshado_virt_addr[i], 0,
				vout->smsshado_size);
		}
	}
	return 0;
}

/* This function wakes up the application once
 * the DMA transfer to VRFB space is completed by the
 * ISP resizer block
 */
void omap_vout_isp_rsz_dma_tx_callback(void *arg)
{
	struct omap_vout_device *vout;
	vout = (struct omap_vout_device *) arg;

	vout->vrfb_dma_tx.tx_status = 1;
	wake_up_interruptible(&vout->vrfb_dma_tx.wait);
}

/* This function configures and initializes the ISP resizer*/
static int init_isp_rsz(struct omap_vout_device *vout)
{
	int num_video_buffers = 0;
	int ret = 0;

	/* get the ISP resizer resource and configure it*/
	if (vout->use_isp_rsz_for_downscale) {
		if (vout->rsz_configured == 0) {
			ret = ispdss_get_resource();
			if (ret) {
				printk(KERN_ERR "<%s>: <%s> failed to get ISP "
						"resizer resource = %d\n",
				       __FILE__, __func__, ret);
				vout->use_isp_rsz_for_downscale = 0;
				return ret;
			}

			num_video_buffers = (vout->vid == OMAP_VIDEO1) ?
			video1_numbuffers : video2_numbuffers;

			/* clear data */
			memset(&pipe, 0, sizeof(pipe));
			pipe.in.path = RSZ_MEM_YUV;
			/* setup source parameters */
			pipe.in.image = vout->pix;
			pipe.in.crop.left = 0; pipe.in.crop.top = 0;
			pipe.in.crop.width = vout->pix.width;
			pipe.in.crop.height = vout->pix.height;
			/* setup destination parameters */
			pipe.out.image.width = vout->win.w.width;
			pipe.out.image.height = vout->win.w.height;

			num_video_buffers = (vout->vid == OMAP_VIDEO1) ?
			video1_numbuffers : video2_numbuffers;

			ret = ispdss_configure(&pipe,
					  omap_vout_isp_rsz_dma_tx_callback,
					  num_video_buffers,
					  (void *)vout);
			printk(KERN_ERR "<%s>: rsz_configure = %d\n",
			       __func__, ret);
			if (ret) {
				printk(KERN_ERR "<%s> failed to configure "
						"ISP_resizer = %d\n",
				       __func__, ret);
				vout->use_isp_rsz_for_downscale = 0;
				return ret;
			}
			vout->rsz_configured = 1;
		}
	}

	return ret;
}
#endif

/* Try format */
static int omap_vout_try_format(struct v4l2_pix_format *pix)
{
	int ifmt, bpp = 0;
	int img_size = pix->height * pix->width;

	if (img_size <= VID_MAX_HEIGHT * VID_MAX_WIDTH) {
		if (cpu_is_omap34xx())
			pix->height = clamp(pix->height,
				(u32)VID_MIN_HEIGHT,
				(u32)VID_MAX_WIDTH);
		else
			pix->height = clamp(pix->height,
				(u32)VID_MIN_HEIGHT,
				(u32)VID_MAX_HEIGHT);
	} else {
		pix->height = clamp(pix->height, (u32)VID_MIN_HEIGHT,
			(u32)VID_MAX_HEIGHT);
	}
	pix->width = clamp(pix->width, (u32)VID_MIN_WIDTH, (u32)VID_MAX_WIDTH);

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
		break;
	case V4L2_PIX_FMT_NV12:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 1; /* TODO: check this? */
		break;
	}

	/* :NOTE: NV12 has width bytes per line in both Y and UV sections */
		pix->bytesperline = pix->width * bpp;

#ifdef TILER_ALLOCATE_V4L2
	pix->bytesperline = (pix->bytesperline + PAGE_SIZE - 1) &
		~(PAGE_SIZE - 1);
#endif

	/* :TODO: add 2-pixel round restrictions to YUYV and NV12 formats */
	pix->sizeimage = pix->bytesperline * pix->height;
	if (V4L2_PIX_FMT_NV12 == pix->pixelformat)
		pix->sizeimage += pix->sizeimage >> 1;

	return bpp;
}

/*
 * omap_vout_uservirt_to_phys: This inline function is used to convert user
 * space virtual address to physical address.
 */
static inline u32 omap_vout_uservirt_to_phys(u32 virtp)
{
	unsigned long physp = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	vma = find_vma(mm, virtp);
	/* For kernel direct-mapped memory, take the easy way */
	if (virtp >= PAGE_OFFSET) {
		physp = virt_to_phys((void *) virtp);
	} else if (vma && (vma->vm_flags & VM_IO)
			&& vma->vm_pgoff) {
		/* this will catch, kernel-allocated,
		   mmaped-to-usermode addresses */
		physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
	} else {
		/* otherwise, use get_user_pages() for general userland pages */
		int res, nr_pages = 1;
		struct page *pages;
		down_read(&current->mm->mmap_sem);

		res = get_user_pages(current, current->mm, virtp, nr_pages,
				1, 0, &pages, NULL);
		up_read(&current->mm->mmap_sem);

		if (res == nr_pages) {
			physp =  __pa(page_address(&pages[0]) +
					(virtp & ~PAGE_MASK));
		} else {
			printk(KERN_WARNING VOUT_NAME
				"get_user_pages failed\n");
			return 0;
		}
	}

	return physp;
}

#ifndef CONFIG_ARCH_OMAP4
/* This functions wakes up the application once
 * the DMA transfer to VRFB space is completed. */
static void omap_vout_vrfb_dma_tx_callback(int lch, u16 ch_status, void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;

	t->tx_status = 1;
	wake_up_interruptible(&t->wait);
}

/* Release the VRFB context once the module exits */
static void omap_vout_release_vrfb(struct omap_vout_device *vout)
{
#ifndef CONFIG_ARCH_OMAP4
/* TODO: this is temporary disabling of vrfb to test V4L2: needs to be
   corrected for future
*/
	int i;
	for (i = 0; i < OMAP_VOUT_MAX_VBUF_CTXT; i++)
		omap_vrfb_release_ctx(&vout->vrfb_context[i]);
#endif
	if (vout->vrfb_dma_tx.req_status == DMA_CHAN_ALLOTED) {
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma(vout->vrfb_dma_tx.dma_ch);
	}

}
#endif

/* Return true if rotation is 90 or 270 */
static inline int rotate_90_or_270(const struct omap_vout_device *vout)
{
	return (vout->rotation == dss_rotation_90_degree ||
		vout->rotation == dss_rotation_270_degree);
}

/* Return true if rotation is enabled */
static inline int rotation_enabled(const struct omap_vout_device *vout)
{
	/* rotation needs to be enabled by default for OMAP3 in order
	 * to use VRFB. ISP resizer also requires VRFB to be used for all
	 * rotation angles including 0 degree. for OMAP3 this function needs
	 * to return true even for 0 degree rotation
	 */
	if (cpu_is_omap34xx())
		return 1;
	else
		return vout->rotation || vout->mirror;
}

/* Reverse the rotation degree if mirroring is enabled */
static inline int calc_rotation(const struct omap_vout_device *vout)
{
#ifndef CONFIG_ARCH_OMAP4
	if (!vout->mirror)
		return vout->rotation;
	switch (vout->rotation) {
	case dss_rotation_90_degree:
		return dss_rotation_270_degree;
	case dss_rotation_270_degree:
		return dss_rotation_90_degree;
	case dss_rotation_180_degree:
		return dss_rotation_0_degree;
	default:
		return dss_rotation_180_degree;
	}
#else
	return vout->rotation;
#endif
}

/* Free the V4L2 buffers */
static void omap_vout_free_buffers(struct omap_vout_device *vout)
{
	int i, numbuffers;

	/* Allocate memory for the buffers */
	if (OMAP_VIDEO3 == vout->vid) {
		numbuffers = video3_numbuffers;
		vout->buffer_size = video3_bufsize;
	} else	{
		numbuffers = (vout->vid)	? video2_numbuffers
						: video1_numbuffers;
		vout->buffer_size = (vout->vid)	? video2_bufsize
						: video1_bufsize;
	}

	for (i = 0; i < numbuffers; i++) {
		omap_vout_free_buffer(vout->buf_virt_addr[i],
			 vout->buf_phy_addr[i], vout->buffer_size);
		vout->buf_phy_addr[i] = 0;
		vout->buf_virt_addr[i] = 0;
	}
}

#ifndef CONFIG_ARCH_OMAP4
/* Free VRFB buffers */
static void omap_vout_free_vrfb_buffers(struct omap_vout_device *vout)
{
	int j;

	for (j = 0; j < OMAP_VOUT_MAX_BUFFERS; j++) {
		omap_vout_free_buffer(vout->smsshado_virt_addr[j],
				vout->smsshado_phy_addr[j],
				vout->smsshado_size);
		vout->smsshado_virt_addr[j] = 0;
		vout->smsshado_phy_addr[j] = 0;
	}
}

/* Allocate the buffers for the VRFB space.  Data is copied from V4L2
 * buffers to the VRFB buffers using the DMA engine.*/
static int omap_vout_vrfb_buffer_setup(struct omap_vout_device *vout,
			  unsigned int *count, unsigned int startindex)
{
	int i;
	bool yuv_mode;
	s32 width, height;

	/* Allocate the VRFB buffers only if the buffers are not
	 * allocated during init time.
	 */
	if ((rotation_enabled(vout)) &&
			!vout->vrfb_static_allocation)
		if (omap_vout_allocate_vrfb_buffers(vout, count, startindex))
			return -ENOMEM;

	if (vout->dss_mode == OMAP_DSS_COLOR_YUV2 ||
			vout->dss_mode == OMAP_DSS_COLOR_UYVY)
		yuv_mode = true;
	else
		yuv_mode = false;

/* TODO: this is temporary disabling of vrfb to test V4L2: needs to be
 corrected for future
*/
	for (i = 0; i < *count; i++) {
		if (vout->use_isp_rsz_for_downscale) {
			width = vout->win.w.width;
			height = vout->win.w.height;
		} else {
			width = vout->pix.width;
			height = vout->pix.height;
		}
			omap_vrfb_setup(&vout->vrfb_context[i],
				vout->smsshado_phy_addr[i],
				width, height,
				vout->bpp, yuv_mode,
				vout->rotation);
	}
	return 0;
}
#endif

static void omap_vout_tiler_buffer_free(struct omap_vout_device *vout,
					unsigned int count,
					unsigned int startindex)
{
#ifdef CONFIG_TILER_OMAP
	int i;

	if (startindex < 0)
		startindex = 0;
	if (startindex + count > VIDEO_MAX_FRAME)
		count = VIDEO_MAX_FRAME - startindex;

	for (i = startindex; i < startindex + count; i++) {
		if (vout->buf_phy_addr_alloced[i])
			tiler_free(vout->buf_phy_addr_alloced[i]);
		if (vout->buf_phy_uv_addr_alloced[i])
			tiler_free(vout->buf_phy_uv_addr_alloced[i]);
		vout->buf_phy_addr[i] = 0;
		vout->buf_phy_addr_alloced[i] = 0;
		vout->buf_phy_uv_addr[i] = 0;
		vout->buf_phy_uv_addr_alloced[i] = 0;
	}
#endif
}

/* Allocate the buffers for  TILER space.  Ideally, the buffers will be ONLY
 in tiler space, with different rotated views available by just a convert.
 */
static int omap_vout_tiler_buffer_setup(struct omap_vout_device *vout,
					unsigned int *count, unsigned int startindex,
					struct v4l2_pix_format *pix)
{
#ifdef CONFIG_TILER_OMAP
	int i, aligned = 1;
	enum tiler_fmt fmt;

	/* normalize buffers to allocate so we stay within bounds */
	int start = (startindex < 0) ? 0 : startindex;
	int n_alloc = (start + *count > VIDEO_MAX_FRAME)
		? VIDEO_MAX_FRAME - start : *count;
	int bpp = omap_vout_try_format(pix);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "tiler buffer alloc:\n"
		"count - %d, start -%d :\n", *count, startindex);

	/* special allocation scheme for NV12 format */
	if (OMAP_DSS_COLOR_NV12 == video_mode_to_dss_mode(pix)) {
		tiler_alloc_packed_nv12(&n_alloc, ALIGN(pix->width, 128),
			pix->height,
			(void **) vout->buf_phy_addr + start,
			(void **) vout->buf_phy_uv_addr + start,
			(void **) vout->buf_phy_addr_alloced + start,
			(void **) vout->buf_phy_uv_addr_alloced + start,
			aligned);
	} else {
		/* Only bpp of 1, 2, and 4 is supported by tiler */
		fmt = (bpp == 1 ? TILFMT_8BIT :
			bpp == 2 ? TILFMT_16BIT :
			bpp == 4 ? TILFMT_32BIT : TILFMT_INVALID);
		if (fmt == TILFMT_INVALID)
			return -ENOMEM;

		tiler_alloc_packed(&n_alloc, fmt, ALIGN(pix->width, 128 / bpp),
			pix->height,
			(void **) vout->buf_phy_addr + start,
			(void **) vout->buf_phy_addr_alloced + start,
			aligned);
	}

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
			"allocated %d buffers\n", n_alloc);

	if (n_alloc < *count) {
		if (n_alloc && (startindex == -1 ||
			V4L2_MEMORY_MMAP != vout->memory)) {
			/* TODO: check this condition's logic */
			omap_vout_tiler_buffer_free(vout, n_alloc, start);
			*count = 0;
			return -ENOMEM;
		}
	}

	for (i = start; i < start + n_alloc; i++) {
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
				"y=%08lx (%d) uv=%08lx (%d)\n",
				vout->buf_phy_addr[i],
				vout->buf_phy_addr_alloced[i] ? 1 : 0,
				vout->buf_phy_uv_addr[i],
				vout->buf_phy_uv_addr_alloced[i] ? 1 : 0);
	}

	*count = n_alloc;
#else
	*count = 0;
#endif

	return 0;
}

/* Free tiler buffers */
static void omap_vout_free_tiler_buffers(struct omap_vout_device *vout)
{
#ifdef CONFIG_TILER_OMAP
	omap_vout_tiler_buffer_free(vout, vout->buffer_allocated, 0);
	vout->buffer_allocated = 0;
#endif
}

/* Convert V4L2 rotation to DSS rotation
 * V4L2 understand 0, 90, 180, 270.
 * convert to 0, 1, 2 and 3 repsectively for DSS */
static int v4l2_rot_to_dss_rot(int v4l2_rotation, enum dss_rotation *rotation,
		bool mirror)
{
	switch (v4l2_rotation) {
	case 90:
		*rotation = dss_rotation_270_degree;
		return 0;
	case 180:
		*rotation = dss_rotation_180_degree;
		return 0;
	case 270:
		*rotation = dss_rotation_90_degree;
		return 0;
	case 0:
		*rotation = dss_rotation_0_degree;
		return 0;
	default:
		return -EINVAL;
	}

}

/* Calculate the buffer offsets from which the streaming should
 * start.  This offset calculation is mainly required because of
 * the VRFB 32 pixels alignment with rotation.
 */
static int omap_vout_calculate_offset(struct omap_vout_device *vout, int idx)
{
	struct v4l2_pix_format *pix = &vout->pix;
	struct v4l2_rect *crop = &vout->crop;
	enum dss_rotation rotation;
	int vr_ps = 1, ps = 2;
#ifndef CONFIG_ARCH_OMAP4
	struct v4l2_window *win = &(vout->win);
	int offset = 0, temp_ps = 2;
	bool mirroring = vout->mirror;
#endif
	int ctop = 0, cleft = 0, line_length = 0;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_dss_device *cur_display;
	int *cropped_offset = vout->cropped_offset + idx;
#ifdef CONFIG_ARCH_OMAP4
	int *cropped_uv_offset = vout->cropped_uv_offset + idx;
	unsigned long addr = 0, uv_addr = 0;
#endif

#ifndef CONFIG_ARCH_OMAP4
	if (vout->use_isp_rsz_for_downscale) {
		crop->height = win->w.height;
		crop->width = win->w.width;
	}
#endif

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];
	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device)
		return -1;
	cur_display = ovl->manager->device;

	rotation = calc_rotation(vout);

	if (V4L2_PIX_FMT_YUYV == pix->pixelformat ||
	    V4L2_PIX_FMT_UYVY == pix->pixelformat) {
		if (rotation_enabled(vout)) {
			/*
			 * ps    - Actual pixel size for YUYV/UYVY for
			 *              VRFB/Mirroring is 4 bytes
			 * vr_ps - Virtually pixel size for YUYV/UYVY is
			 *              2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		} else {
			ps = 2;	/* otherwise the pixel size is 2 byte */
		}
	} else if (V4L2_PIX_FMT_RGB32 == pix->pixelformat) {
		ps = 4;
	} else if (V4L2_PIX_FMT_RGB24 == pix->pixelformat) {
		ps = 3;
	}
	vout->ps = ps;
	vout->vr_ps = vr_ps;
	if (rotation_enabled(vout)) {
		line_length = MAX_PIXELS_PER_LINE;
#ifndef CONFIG_ARCH_OMAP4
		if (vout->use_isp_rsz_for_downscale) {
			ctop = crop->top;
			cleft = crop->left;
		} else {
			ctop = (pix->height - crop->height) - crop->top;
			cleft = (pix->width - crop->width) - crop->left;
		}
#else
		ctop = (pix->height - crop->height) - crop->top;
		cleft = (pix->width - crop->width) - crop->left;
#endif

	} else {
		line_length = pix->width;
	}
	vout->line_length = line_length;
#ifndef CONFIG_ARCH_OMAP4
	switch (rotation) {
	case dss_rotation_90_degree:
		offset = MAX_PIXELS_PER_LINE * vout->vrfb_context[0].yoffset *
			vout->vrfb_context[0].bytespp;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			*cropped_offset = offset + line_length *
				temp_ps * cleft + crop->top * temp_ps;
		} else {
			*cropped_offset = offset + line_length * temp_ps *
				cleft + crop->top * temp_ps + (line_length *
				((crop->width / (vr_ps)) - 1) * ps);
		}
		break;
	case dss_rotation_180_degree:
		offset = ((MAX_PIXELS_PER_LINE * vout->vrfb_context[0].yoffset *
			vout->vrfb_context[0].bytespp) +
			(vout->vrfb_context[0].xoffset *
			vout->vrfb_context[0].bytespp));
		if (mirroring == 0) {
			*cropped_offset = offset + (line_length * ps * ctop) +
				(cleft / vr_ps) * ps;

		} else {
			*cropped_offset = offset + (line_length * ps * ctop) +
				(cleft / vr_ps) * ps + (line_length *
				(crop->height - 1) * ps);
		}
		break;
	case dss_rotation_270_degree:
		offset = vout->vrfb_context[0].xoffset *
			vout->vrfb_context[0].bytespp;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			*cropped_offset = offset + line_length *
			    temp_ps * crop->left + ctop * ps;
		} else {
			*cropped_offset = offset + line_length *
				temp_ps * crop->left + ctop * ps +
				(line_length * ((crop->width / vr_ps) - 1) *
				 ps);
		}
		break;
	case dss_rotation_0_degree:
		if (mirroring == 0) {
			*cropped_offset = (line_length * ps) *
				crop->top + (crop->left / vr_ps) * ps;
		} else {
			*cropped_offset = (line_length * ps) *
				crop->top + (crop->left / vr_ps) * ps +
				(line_length * (crop->height - 1) * ps);
		}
		break;
	default:
		*cropped_offset = (line_length * ps * crop->top) /
			vr_ps + (crop->left * ps) / vr_ps +
			((crop->width / vr_ps) - 1) * ps;
		break;
	}
#else
	/* :TODO: change v4l2 to send TSPtr as tiled addresses to DSS2 */
	addr = tiler_get_natural_addr(vout->queued_buf_addr[idx]);

	if (OMAP_DSS_COLOR_NV12 == vout->dss_mode) {
		*cropped_offset = tiler_stride(addr) * crop->top + crop->left;
		uv_addr = tiler_get_natural_addr(
			vout->queued_buf_uv_addr[idx]);
		/* :TODO: only allow even crops for NV12 */
		*cropped_uv_offset = tiler_stride(uv_addr) * (crop->top >> 1)
			+ (crop->left & ~1);
	} else {
		*cropped_offset =
			tiler_stride(addr) * crop->top + crop->left * ps;
	}
#endif
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
	"%s Offset:%x\n", __func__, *cropped_offset);
	return 0;
}

/* convert V4L2 pixel format to DSS pixel format */
static enum omap_color_mode video_mode_to_dss_mode(
			struct v4l2_pix_format *pix)
{

	switch (pix->pixelformat) {
#ifdef CONFIG_ARCH_OMAP4
	case V4L2_PIX_FMT_NV12:
		return OMAP_DSS_COLOR_NV12;
#endif
	case 0:
		break;
	case V4L2_PIX_FMT_YUYV:
		return OMAP_DSS_COLOR_YUV2;

	case V4L2_PIX_FMT_UYVY:
		return OMAP_DSS_COLOR_UYVY;

	case V4L2_PIX_FMT_RGB565:
		return OMAP_DSS_COLOR_RGB16;

	case V4L2_PIX_FMT_RGB24:
		return OMAP_DSS_COLOR_RGB24P;

#ifdef CONFIG_ARCH_OMAP4
	case V4L2_PIX_FMT_RGB32:
		return OMAP_DSS_COLOR_ARGB32;
#endif
	case V4L2_PIX_FMT_BGR32:
		return OMAP_DSS_COLOR_RGBX32;

	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/* helper function: for NV12, returns uv buffer address given single buffer
 * for yuv - y buffer will still be in the input.
 * used only for non-TILER case
*/
u32 omapvid_get_uvbase_nv12(u32 paddr, int height, int width)
{
	u32 puv_addr = 0;

	puv_addr = (paddr + (height * width));
	return puv_addr;
}

/* Setup the overlay */
int omapvid_setup_overlay(struct omap_vout_device *vout,
		struct omap_overlay *ovl, int posx, int posy, int outw,
		int outh, u32 addr, u32 uv_addr)
{
	int r = 0;
	enum dss_rotation rotation;
	bool mirror;
	int cropheight, cropwidth, pixheight, pixwidth;
#ifndef CONFIG_ARCH_OMAP4
	s32 new_crop_width, new_crop_height, new_pix_width, new_pix_height;
#endif
	struct omap_overlay_info info;

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0 &&
			(outw != vout->pix.width || outh != vout->pix.height)) {
		r = -EINVAL;
		goto err;
	}

#ifndef CONFIG_ARCH_OMAP4
	if (vout->pix.pixelformat == V4L2_PIX_FMT_RGB32) {
		/* TODO: OMAP4: check this ??*/
		vout->dss_mode = (vout->vid == OMAP_VIDEO1) ?
				 OMAP_DSS_COLOR_RGB24U : OMAP_DSS_COLOR_ARGB32;
	} else
#endif
		vout->dss_mode = video_mode_to_dss_mode(&vout->pix);

	if (vout->dss_mode == -EINVAL) {
		r = -EINVAL;
		goto err;
	}

	rotation = vout->rotation;
	mirror = vout->mirror;

#ifndef CONFIG_ARCH_OMAP4
	/* For 720p set the width and height for ISP resizer downscaling*/
	if (vout->use_isp_rsz_for_downscale) {
		new_crop_width = vout->win.w.width;
		new_crop_height = vout->win.w.height;
		new_pix_width = vout->win.w.width;
		new_pix_height = vout->win.w.height;

	} else {
		new_crop_width = vout->crop.width;
		new_crop_height = vout->crop.height;
		new_pix_width = vout->pix.width;
		new_pix_height = vout->pix.height;
	}
#endif

	/* Setup the input plane parameters according to
	 * rotation value selected.
	 */
#ifndef CONFIG_ARCH_OMAP4
	if (rotate_90_or_270(vout)) {
		cropheight = new_crop_width;
		cropwidth = new_crop_height;
		pixheight = new_pix_width;
		pixwidth = new_pix_height;
	} else {
		cropheight = new_crop_height;
		cropwidth = new_crop_width;
		pixheight = new_pix_height;
		pixwidth = new_pix_width;
	}
#else
	if (rotate_90_or_270(vout)) {
		cropheight = vout->crop.width;
		cropwidth = vout->crop.height;
		pixheight = vout->pix.width;
		pixwidth = vout->pix.height;
	} else {
		cropheight = vout->crop.height;
		cropwidth = vout->crop.width;
		pixheight = vout->pix.height;
		pixwidth = vout->pix.width;
	}
#endif
	ovl->get_overlay_info(ovl, &info);
	if (addr)
		info.paddr = addr;
#ifdef CONFIG_ARCH_OMAP4
	if (OMAP_DSS_COLOR_NV12 == vout->dss_mode)
		info.p_uv_addr = uv_addr;
	else
		info.p_uv_addr = (u32) NULL;
#endif
	info.vaddr = NULL;
	info.width = cropwidth;
	info.height = cropheight;
	info.color_mode = vout->dss_mode;
	info.mirror = mirror;
	info.pos_x = posx;
	info.pos_y = posy;
	info.out_width = outw;
	info.out_height = outh;
	/* info.global_alpha = vout->win.global_alpha; */
	/* FIXME figure out why old logic failed */
	info.global_alpha =
		vout->vid_info.overlays[0]->info.global_alpha;
#ifdef CONFIG_ARCH_OMAP4
	info.rotation_type = OMAP_DSS_ROT_TILER;
	info.screen_width = pixwidth;
	info.rotation = vout->rotation;
#else
	info.flicker_filter = vout->flicker_filter;
	if (!rotation_enabled(vout)) {
		info.rotation = 0;
		info.rotation_type = OMAP_DSS_ROT_DMA;
		info.screen_width = pixwidth;
	} else {
		info.rotation = vout->rotation;
		info.rotation_type = OMAP_DSS_ROT_VRFB;
		info.screen_width = 2048;
	}
#endif

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
	"%s info.enable=%d info.addr=%x info.width=%d\n info.height=%d "
	"info.color_mode=%d info.rotation=%d info.mirror=%d\n "
	"info.posx=%d info.posy=%d info.out_width = %d info.out_height=%d\n "
	"info.rotation_type=%d info.screen_width=%d\n", __func__, info.enabled,
	info.paddr, info.width, info.height, info.color_mode, info.rotation,
	info.mirror, info.pos_x, info.pos_y, info.out_width, info.out_height,
	info.rotation_type, info.screen_width);

#ifdef CONFIG_ARCH_OMAP4
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "info.puvaddr=%x\n",
			info.p_uv_addr);
#endif
	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_WARNING VOUT_NAME "setup_overlay failed\n");
	return r;
}

/* Initialize the overlay structure */
int omapvid_init(struct omap_vout_device *vout, u32 addr, u32 uv_addr)
{
	int r = 0;
	struct omapvideo_info *ovid = &vout->vid_info;
	struct omap_overlay *ovl;
	int posx, posy;
	int outw, outh, temp, rotation;
	int i;
	struct v4l2_window *win;
	struct omap_video_timings *timing;

	win = &vout->win;
	rotation = vout->rotation;
	for (i = 0; i < ovid->num_overlays; i++) {
		ovl = ovid->overlays[i];
		if (!ovl->manager || !ovl->manager->device)
			return -EINVAL;

		timing = &ovl->manager->device->panel.timings;

		outw = win->w.width;
		outh = win->w.height;
		posx = win->w.left;
		posy = win->w.top;
		switch (rotation) {
		case dss_rotation_90_degree:
			/* Invert the height and width for 90
			 * and 270 degree rotation
			 */
			temp = outw;
			outw = outh;
			outh = temp;
#ifndef CONFIG_ARCH_OMAP4
			posy = (timing->y_res - win->w.width)-
				win->w.left;
			posx = win->w.top;
#endif
			break;

		case dss_rotation_180_degree:
#ifndef CONFIG_ARCH_OMAP4
			posx = (timing->x_res - win->w.width) -
				win->w.left;
			posy = (timing->y_res - win->w.height) -
				win->w.top;
#endif
			break;

		case dss_rotation_270_degree:
			temp = outw;
			outw = outh;
			outh = temp;
#ifndef CONFIG_ARCH_OMAP4
			posy = win->w.left;
			posx = (timing->x_res - win->w.height)
				- win->w.top;
#endif
			break;

		default:
			break;
		}

		r = omapvid_setup_overlay(vout, ovl, posx, posy, outw,
				outh, addr, uv_addr);
		if (r)
			goto err;
	}
	return 0;
err:
	printk(KERN_WARNING VOUT_NAME "apply_changes failed\n");
	return r;
}

/*  Apply the changes set the go bit of DSS */
int omapvid_apply_changes(struct omap_vout_device *vout)
{
	struct omapvideo_info *ovid = &vout->vid_info;
	struct omap_overlay *ovl;
	int i;

	for (i = 0; i < ovid->num_overlays; i++) {
		ovl = ovid->overlays[i];
		if (!ovl->manager || !ovl->manager->device)
			return -EINVAL;
		ovl->manager->apply(ovl->manager);
	}
	return 0;

}

static int omapvid_link_en_ovl(int enable, u32 addr, u32 uv_addr)
{
	int t;

	for (t = 0; t < NUM_OF_VIDEO_CHANNELS; t++) {
		struct omap_overlay *ovl = omap_dss_get_overlay(t+1);
		if (!ovl) {
			pr_err("Could not get overlay %d\n", t+1);
			WARN_ON(1);
			continue;
		}
		if (ovl->manager && ovl->manager->device) {
			struct omap_overlay_info info;
			ovl->get_overlay_info(ovl, &info);
			info.enabled = enable;
			info.paddr = addr;

			if (cpu_is_omap44xx())
				info.p_uv_addr = uv_addr;

			if (ovl->set_overlay_info(ovl, &info))
				return -EINVAL;
		}
	}

	return 0;
}

/* Video buffer call backs */

/* Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers
 */
static int omap_vout_buffer_setup(struct videobuf_queue *q, unsigned int *count,
			  unsigned int *size)
{
	struct omap_vout_device *vout = q->priv_data;
	int i;
#ifndef TILER_ALLOCATE_V4L2
	int startindex = 0, j;
	u32 phy_addr = 0, virt_addr = 0;
#endif
	if (!vout)
		return -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

#ifndef CONFIG_TILER_OMAP
	/* :TODO: ??? */
	startindex = (vout->vid == OMAP_VIDEO1 ? video1_numbuffers :
		      vout->vid == OMAP_VIDEO2 ? video2_numbuffers :
		      video3_numbuffers);

#ifndef CONFIG_ARCH_OMAP4
	if (rotation_enabled(vout) && *count > OMAP_VOUT_MAX_BUFFERS)
		*count = OMAP_VOUT_MAX_BUFFERS;

	/* If rotation is enabled, allocate memory for VRFB space also */
	if (rotation_enabled(vout)) {
		if (omap_vout_vrfb_buffer_setup(vout, count, startindex))
			return -ENOMEM;
	}
#endif /* CONFIG_ARCH_OMAP4 */

	/* Now allocated the V4L2 buffers */
	*size = vout->pix.width * vout->pix.height * vout->bpp;
	*size = PAGE_ALIGN(*size);

	for (i = startindex; i < *count; i++) {
		vout->buffer_size = *size;

		virt_addr = omap_vout_alloc_buffer(vout->buffer_size,
				&phy_addr);
		if (!virt_addr) {
			if (!rotation_enabled(vout))
				break;
			/* Free the VRFB buffers if no space for V4L2 buffers */
			for (j = i; j < *count; j++) {
				omap_vout_free_buffer(
					vout->smsshado_virt_addr[j],
					vout->smsshado_phy_addr[j],
					vout->smsshado_size);
				vout->smsshado_virt_addr[j] = 0;
				vout->smsshado_phy_addr[j] = 0;
			}
		}
		vout->buf_virt_addr[i] = virt_addr;
		vout->buf_phy_addr[i] = phy_addr;
	}

	if (startindex < *count)
		*count = vout->buffer_allocated = i;
	else
		*count = vout->buffer_allocated = *count;

#else /* TILER_ALLOCATE_V4L2 */

	/* tiler_alloc_buf to be called here
	pre-requisites: rotation, format?
	based on that buffers will be allocated.
	*/
	/* Now allocated the V4L2 buffers */
	/* i is the block-width - either 4K or 8K, depending upon input width*/
	i = PAGE_ALIGN(vout->pix.width * vout->bpp);

	/* for NV12 format, buffer is height + height / 2*/
	if (OMAP_DSS_COLOR_NV12 == vout->dss_mode)
		*size = vout->buffer_size = (vout->pix.height * 3/2 * i);
	else
		*size = vout->buffer_size = (vout->pix.height * i);

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
			"\nheight=%d, size = %d, vout->buffer_sz=%d\n",
			vout->pix.height, *size, vout->buffer_size);
	if (omap_vout_tiler_buffer_setup(vout, count, 0, &vout->pix))
			return -ENOMEM;
#endif  /* TILER_ALLOCATE_V4L2 */

	if (V4L2_MEMORY_MMAP != vout->memory)
		return 0;

	return 0;
}

#ifndef TILER_ALLOCATE_V4L2
/* Free the V4L2 buffers additionally allocated than default
 * number of buffers and free all the VRFB buffers */
static void omap_vout_free_allbuffers(struct omap_vout_device *vout)
{
	int num_buffers = 0, i;

	/* :TODO: ??? */
	num_buffers = (vout->vid == OMAP_VIDEO1 ? video1_numbuffers :
		       vout->vid == OMAP_VIDEO2 ? video2_numbuffers :
		       video3_numbuffers);

	for (i = num_buffers; i < vout->buffer_allocated; i++) {
		if (vout->buf_virt_addr[i]) {
			omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i], vout->buffer_size);
		}

		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}
	/* Free the VRFB buffers only if they are allocated
	 * during reqbufs.  Don't free if init time allocated
	 */
	if (!vout->vrfb_static_allocation) {
		for (i = 0; i < OMAP_VOUT_MAX_BUFFERS; i++) {
			if (vout->smsshado_virt_addr[i]) {
				omap_vout_free_buffer(
						vout->smsshado_virt_addr[i],
						vout->smsshado_phy_addr[i],
						vout->smsshado_size);
				vout->smsshado_virt_addr[i] = 0;
				vout->smsshado_phy_addr[i] = 0;
			}
		}
	}
	vout->buffer_allocated = num_buffers;
}
#endif

/* This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * user space virtual address into physical address if userptr memory
 * exchange mechanism is used. If rotation is enabled, it copies entire
 * buffer into VRFB memory space before giving it to the DSS.
 */
static int omap_vout_buffer_prepare(struct videobuf_queue *q,
			    struct videobuf_buffer *vb,
			    enum v4l2_field field)
{
	struct omap_vout_device *vout = q->priv_data;
	struct videobuf_dmabuf *dmabuf = NULL;
	struct vout_platform_data *pdata = (vout->dev)->platform_data;

#ifndef CONFIG_ARCH_OMAP4
	u32 dest_frame_index = 0, src_element_index = 0;
	u32 dest_element_index = 0, src_frame_index = 0;
	u32 elem_count = 0, frame_count = 0, pixsize = 2;
	enum dss_rotation rotation;
	struct vid_vrfb_dma *tx;
	int ret = 0;

	rotation = calc_rotation(vout);
#endif


	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vout->pix.width;
		vb->height = vout->pix.height;
		vb->size = vb->width * vb->height * vout->bpp;
		vb->field = field;
	}
	vb->state = VIDEOBUF_PREPARED;

#ifndef TILER_ALLOCATE_V4L2
	/* if user pointer memory mechanism is used, get the physical
	 * address of the buffer
	 */
	if (V4L2_MEMORY_USERPTR == vb->memory) {
		if (0 == vb->baddr)
			return -EINVAL;
		/* Virtual address */
		/* priv points to struct videobuf_pci_sg_memory. But we went
		 * pointer to videobuf_dmabuf, which is member of
		 * videobuf_pci_sg_memory */
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);
		dmabuf->vmalloc = (void *) vb->baddr;

		/* Physical address */
		dmabuf->bus_addr =
			(dma_addr_t) omap_vout_uservirt_to_phys(vb->baddr);
	}

	dmabuf = videobuf_to_dma(q->bufs[vb->i]);
	 /* Flush only cacheable memory region */
	 if (cacheable_buffers == 1 && flushable_buffers == 1) {
		dmac_clean_range(dmabuf->vmalloc,
		dmabuf->vmalloc + vout->buffer_size);
		outer_clean_range(dmabuf->bus_addr,
		dmabuf->bus_addr + vout->buffer_size);
	}

	if (vout->use_isp_rsz_for_downscale) {
		/*Start resizing*/
		ret = ispdss_begin(&pipe, vb->i, vb->i,
				MAX_PIXELS_PER_LINE *\
				vout->bpp * vout->vrfb_bpp,
				(u32)vout->vrfb_context[vb->i].\
				paddr[rotation],
				vout->buf_phy_addr[vb->i],
				vout->buffer_size);

		if (ret) {
			printk(KERN_ERR "<%s> ISP Resizer Failed to resize "
					"the buffer"
					" = %d\n", __func__, ret);
			return ret;
		}

	} else if (!rotation_enabled(vout)) {

			vout->queued_buf_addr[vb->i] = (u8 *) dmabuf->bus_addr;
			return 0;
	} else {
#ifndef CONFIG_TILER_OMAP
		/* If rotation is enabled, copy input buffer into VRFB
		 * memory space using DMA. We are copying input buffer
		 * into VRFB memory space of desired angle and DSS will
		 * read image VRFB memory for 0 degree angle
		 */
		pixsize = vout->bpp * vout->vrfb_bpp;
		/*
		 * DMA transfer in double index mode
		 */

		/* Frame index */
		dest_frame_index = ((MAX_PIXELS_PER_LINE * pixsize) -
				(vout->pix.width * vout->bpp)) + 1;

		/* Source and destination parameters */
		src_element_index = 0;
		src_frame_index = 0;
		dest_element_index = 1;
		/* Number of elements per frame */
		elem_count = vout->pix.width * vout->bpp;
		frame_count = vout->pix.height;
		tx = &vout->vrfb_dma_tx;
		tx->tx_status = 0;
		omap_set_dma_transfer_params(tx->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					     (elem_count / 4), frame_count,
					     OMAP_DMA_SYNC_ELEMENT,
					     tx->dev_id, 0x0);
		/* src_port required only for OMAP1 */
		omap_set_dma_src_params(tx->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
					dmabuf->bus_addr, src_element_index,
					src_frame_index);
		/* enable src data packing */
		omap_set_dma_src_data_pack(tx->dma_ch, 1);
		/*set dma source burst mode for VRFB */
		omap_set_dma_src_burst_mode(tx->dma_ch, OMAP_DMA_DATA_BURST_16);

		/* dest_port required only for OMAP1 */
		omap_set_dma_dest_params(tx->dma_ch, 0,
					 OMAP_DMA_AMODE_DOUBLE_IDX,
					 vout->vrfb_context[vb->i].\
					 paddr[rotation], dest_element_index,
					 dest_frame_index);
		/*set dma dest burst mode for VRFB */
		omap_set_dma_dest_burst_mode(tx->dma_ch,
					     OMAP_DMA_DATA_BURST_16);
		/* enable dest data packing */
		omap_set_dma_dest_data_pack(tx->dma_ch, 1);
		omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

		/*
		 * Through-put requirement.
		 * Set max OCP freq:
		 * 		For 3630 is 200 MHz
		 *		For 3430 is 166 MHz
		 * Through-put is in KByte/s
		 * so OCP_FREQ KHz * 4 =
		 *			800 KByte/s for 3630
		 *			664 KByte/s for 3430
		 *
		 * L3 interface is only needed for reading pixel data
		 * So, hold L3 constraint over DMA transfer.
		 * Released at streamoff.
		 * REVISIT: To scale L3 only at DMA trasfers and VRFB->LCD
		 * trasfers.
		 */
#ifdef CONFIG_PM
		if (!cpu_is_omap44xx())
			pdata->set_min_bus_tput(vout->dev , OCP_INITIATOR_AGENT,
					VDD2_OCP_FREQ_CONST * 4);
#endif
		omap_start_dma(tx->dma_ch);
		interruptible_sleep_on_timeout(&tx->wait, VRFB_TX_TIMEOUT);

		if (tx->tx_status == 0) {
			omap_stop_dma(tx->dma_ch);
			return -EINVAL;
		}
	}
		/* Store buffers physical address into an array. Addresses
		 * from this array will be used to configure DSS */
	vout->queued_buf_addr[vb->i] = (u8 *)
		vout->vrfb_context[vb->i].paddr[0];
#endif
#else 	/* TILER to be used */

	/* Here, we need to use the physical addresses given by Tiler:
	*/
	dmabuf = videobuf_to_dma(q->bufs[vb->i]);
	vout->queued_buf_addr[vb->i] = (u8 *) dmabuf->bus_addr;
	vout->queued_buf_uv_addr[vb->i] = (u8 *) dmabuf->vmalloc;

#endif

	return 0;
}

/* Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed. */
static void omap_vout_buffer_queue(struct videobuf_queue *q,
			  struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

	/* Driver is also maintainig a queue. So enqueue buffer in the driver
	 * queue */
	list_add_tail(&vb->queue, &vout->dma_queue);

	vb->state = VIDEOBUF_QUEUED;
}

/* Buffer release function is called from videobuf layer to release buffer
 * which are already allocated */
static void omap_vout_buffer_release(struct videobuf_queue *q,
			    struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

	vb->state = VIDEOBUF_NEEDS_INIT;

	if (V4L2_MEMORY_MMAP != vout->memory)
		return;
}

/*
 *  File operations
 */
static void omap_vout_vm_open(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count++;
}

static void omap_vout_vm_close(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);

	vout->mmap_count--;
}

static struct vm_operations_struct omap_vout_vm_ops = {
	.open = omap_vout_vm_open,
	.close = omap_vout_vm_close,
};

static int omap_vout_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q = &vout->vbq;
#ifndef TILER_ALLOCATE_V4L2
	unsigned long size = (vma->vm_end - vma->vm_start);
	unsigned long start = vma->vm_start;
	struct page *cpage;
#endif
	int i;
	void *pos;
	struct videobuf_dmabuf *dmabuf = NULL;
#ifdef TILER_ALLOCATE_V4L2
	int j = 0, k = 0, m = 0, p = 0, m_increment = 0;
#endif

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i) {
		v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"offset invalid [offset=0x%lx]\n",
			(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}
	q->bufs[i]->baddr = vma->vm_start;

	vma->vm_flags |= VM_RESERVED;
	if (cacheable_buffers == 0) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		printk(KERN_DEBUG "Requested uncached buffers\n");
	 }

	vma->vm_ops = &omap_vout_vm_ops;
	vma->vm_private_data = (void *) vout;
	dmabuf = videobuf_to_dma(q->bufs[i]);

	pos = (void *)(dmabuf->bus_addr);

#ifndef TILER_ALLOCATE_V4L2
	while (size > 0) {
		cpage = pfn_to_page(((unsigned int) pos) >> PAGE_SHIFT);
		if (vm_insert_page(vma, start, cpage)) {
			printk(KERN_ERR "vout_mmap: Failed to insert bus_addr"
							"page to VMA \n");
			return -EAGAIN;
			}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
#else /* Tiler remapping */
	pos = (void *) dmabuf->bus_addr;
	/* get line width */
	/* for NV12, Y buffer is 1bpp*/
	if (OMAP_DSS_COLOR_NV12 == vout->dss_mode) {
		p = (vout->pix.width +
			TILER_PAGE - 1) & ~(TILER_PAGE - 1);
		m_increment = 64 * TILER_WIDTH;
	} else {
		p = (vout->pix.width * vout->bpp +
			TILER_PAGE - 1) & ~(TILER_PAGE - 1);

		if (vout->bpp > 1)
			m_increment = 2*64*TILER_WIDTH;
		else
			m_increment = 64 * TILER_WIDTH;
	}

	for (j = 0; j < vout->pix.height; j++) {
		/* map each page of the line */
	#if 0
		if (0)
			printk(KERN_NOTICE
				"Y buffer %s::%s():%d: vm_start+%d = 0x%lx,"
				"dma->vmalloc+%d = 0x%lx, w=0x%x\n",
				__FILE__, __func__, __LINE__,
				k, vma->vm_start + k, m,
				(pos + m), p);
	#endif
		vma->vm_pgoff =
			((unsigned long)pos + m) >> PAGE_SHIFT;

		if (remap_pfn_range(vma, vma->vm_start + k,
			((unsigned long)pos + m) >> PAGE_SHIFT,
			p, vma->vm_page_prot))
				return -EAGAIN;
		k += p;
		m += m_increment;
	}
	m = 0;

	/* UV Buffer in case of NV12 format */
	if (OMAP_DSS_COLOR_NV12 == vout->dss_mode) {
		pos = dmabuf->vmalloc;
		/* UV buffer is 2 bpp, but half size, so p remains */
		m_increment = 2*64*TILER_WIDTH;

		/* UV buffer is height / 2*/
		for (j = 0; j < vout->pix.height / 2; j++) {
			/* map each page of the line */
		#if 0
			if (0)
				printk(KERN_NOTICE
				"UV buffer %s::%s():%d: vm_start+%d = 0x%lx,"
				"dma->vmalloc+%d = 0x%lx, w=0x%x\n",
				__FILE__, __func__, __LINE__,
				k, vma->vm_start + k, m,
				(pos + m), p);
		#endif
			vma->vm_pgoff =
				((unsigned long)pos + m) >> PAGE_SHIFT;

			if (remap_pfn_range(vma, vma->vm_start + k,
				((unsigned long)pos + m) >> PAGE_SHIFT,
				p, vma->vm_page_prot))
				return -EAGAIN;
			k += p;
			m += m_increment;
		}
	}

#endif
	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */
	vout->mmap_count++;
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);
	return 0;
}

static int omap_vout_release(struct file *file)
{

	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q;
	unsigned int t;
	struct omapvideo_info *ovid;
	unsigned int r;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Entering %s\n", __func__);
	ovid = &vout->vid_info;

	if (!vout)
		return 0;
	q = &vout->vbq;

	/* Disable all the overlay managers connected with this interface */
	for (t = 0; t < ovid->num_overlays; t++) {
			struct omap_overlay *ovl = ovid->overlays[t];
			if (ovl->manager && ovl->manager->device) {
				struct omap_overlay_info info;
				ovl->get_overlay_info(ovl, &info);
				info.enabled = 0;
				ovl->set_overlay_info(ovl, &info);
			}

		}
	/* Turn off the pipeline */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_WARNING VOUT_NAME "Unable to apply changes\n");

#ifndef CONFIG_ARCH_OMAP4
	/* Release the ISP resizer resource if not already done so*/
	if (vout->use_isp_rsz_for_downscale) {
		if (vout->rsz_configured == 1) {
			ispdss_put_resource();
			vout->rsz_configured = 0;
			vout->use_isp_rsz_for_downscale = 0;
		}
	}

	vrfb_configured = 0;
#endif

	/* Free all buffers */
#ifndef TILER_ALLOCATE_V4L2
	omap_vout_free_allbuffers(vout);
#else
	omap_vout_free_tiler_buffers(vout);
#endif

	videobuf_mmap_free(q);

	/* Even if apply changes fails we should continue
	   freeing allocated memeory */
	if (vout->streaming) {
		u32 mask = 0;

	mask = DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_EVEN |
			DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_FRAMEDONE;
#ifdef CONFIG_ARCH_OMAP4
	mask = mask | DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2;
#endif
	omap_dispc_unregister_isr(omap_vout_isr, vout, mask);
	vout->streaming = 0;

	videobuf_streamoff(q);
	videobuf_queue_cancel(q);

}

	if (vout->mmap_count != 0)
		vout->mmap_count = 0;

	vout->opened -= 1;
	file->private_data = NULL;

	if (vout->buffer_allocated)
		videobuf_mmap_free(q);
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);
	return r;
}

static int omap_vout_open(struct file *file)
{
	struct omap_vout_device *vout = NULL;
	struct videobuf_queue *q;

	vout = video_drvdata(file);
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Entering %s\n", __func__);

	if (vout == NULL)
		return -ENODEV;

	/* for now, we only support single open */
	if (vout->opened)
		return -EBUSY;

	vout->opened += 1;

	file->private_data = vout;
	vout->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	q = &vout->vbq;
	video_vbq_ops.buf_setup = omap_vout_buffer_setup;
	video_vbq_ops.buf_prepare = omap_vout_buffer_prepare;
	video_vbq_ops.buf_release = omap_vout_buffer_release;
	video_vbq_ops.buf_queue = omap_vout_buffer_queue;
	spin_lock_init(&vout->vbq_lock);

	videobuf_queue_sg_init(q, &video_vbq_ops, NULL, &vout->vbq_lock,
			       vout->type, V4L2_FIELD_NONE, sizeof
			       (struct videobuf_buffer), vout);
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Exiting %s\n", __func__);
	return 0;
}

/* V4L2 ioctls */
static int vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct omap_vout_device *vout = fh;

	strlcpy(cap->driver, VOUT_NAME,
		sizeof(cap->driver));
	strlcpy(cap->card, vout->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
			sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;

	f->fmt.pix = vout->pix;
	return 0;

}

static int vidioc_try_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	if (vout->streaming)
		return -EBUSY;

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	if (!ovl->manager || !ovl->manager->device)
		return -EINVAL;
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	vout->fbuf.fmt.height = timing->y_res;
	vout->fbuf.fmt.width = timing->x_res;

	omap_vout_try_format(&f->fmt.pix);
	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	int bpp;
	int r;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->lock);

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	timing = &ovl->manager->device->panel.timings;

/* TODO: check if TILER ADAPTATION is needed here. */
	/* We dont support RGB24-packed mode if vrfb rotation
	 * is enabled*/
	if ((rotation_enabled(vout)) &&
			f->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}

	/* get the framebuffer parameters */

#ifndef CONFIG_ARCH_OMAP4
	if (rotate_90_or_270(vout)) {
		vout->fbuf.fmt.height = timing->x_res;
		vout->fbuf.fmt.width = timing->y_res;
	} else {
#endif
		vout->fbuf.fmt.height = timing->y_res;
		vout->fbuf.fmt.width = timing->x_res;
#ifndef CONFIG_ARCH_OMAP4
	}
#endif
	/* change to samller size is OK */

	bpp = omap_vout_try_format(&f->fmt.pix);
	if (V4L2_PIX_FMT_NV12 == f->fmt.pix.pixelformat)
		f->fmt.pix.sizeimage = f->fmt.pix.width *
					f->fmt.pix.height * 3/2;
	else
		f->fmt.pix.sizeimage = f->fmt.pix.width *
					f->fmt.pix.height * bpp;

	/* try & set the new output format */
	vout->bpp = bpp;
	vout->pix = f->fmt.pix;
	vout->vrfb_bpp = 1;

	if (cpu_is_omap34xx())
		/*check for 720p format*/
		if (vout->pix.height * vout->pix.width == VID_MAX_WIDTH * 720)
			vout->use_isp_rsz_for_downscale = 1;


	/* If YUYV then vrfb bpp is 2, for  others its 1 */
	if (V4L2_PIX_FMT_YUYV == vout->pix.pixelformat ||
		V4L2_PIX_FMT_UYVY == vout->pix.pixelformat)
		vout->vrfb_bpp = 2;

	/* set default crop and win */
	omap_vout_new_format(&vout->pix, &vout->fbuf, &vout->crop, &vout->win);

	/* Save the changes in the overlay strcuture */
	r = omapvid_init(vout, 0, 0);
	if (r) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	int err = -EINVAL;
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;

	err = omap_vout_try_window(&vout->fbuf, win);

	if (err)
		return err;

	if (vout->vid == OMAP_VIDEO1)
		win->global_alpha = 255;
	else
		win->global_alpha = f->fmt.win.global_alpha;

	return 0;
}

static int vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	int err = -EINVAL;
	struct omap_overlay *ovl;
	struct omapvideo_info *ovid;
	struct v4l2_window *win = &f->fmt.win;
	int r = 0;

	mutex_lock(&vout->lock);
	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

#ifndef CONFIG_ARCH_OMAP4
	if (vout->use_isp_rsz_for_downscale) {
		/* align the output width to 16 bytes
		 * ISP resizer requires the output width to be
		 * aligned to 16 bytes.
		 */
		win->w.width = ((win->w.width + 0x0f) & ~0x0f);
	}
#endif
	err = omap_vout_new_window(&vout->crop, &vout->win, &vout->fbuf, win);
	if (err) {
		mutex_unlock(&vout->lock);
		return err;
	}
	/* Video1 plane does not support global alpha */
	if (ovl->id == OMAP_DSS_VIDEO1)
		vout->win.global_alpha = 255;
	else
		vout->win.global_alpha = f->fmt.win.global_alpha;

	vout->win.chromakey = f->fmt.win.chromakey;

	/* Save the changes in the overlay strcuture */
	r = omapvid_init(vout, 0, 0);
	if (r) {
		printk(KERN_ERR VOUT_NAME "failed to update window\n");
		mutex_unlock(&vout->lock);
		return r;
	}
	/* Apply changes and set the Go bit */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_WARNING VOUT_NAME "Unable to apply changes\n");

	mutex_unlock(&vout->lock);
	return r;
}

static int vidioc_enum_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
		sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct omap_overlay *ovl;
	struct omapvideo_info *ovid;
	struct omap_overlay_manager_info info;
	struct v4l2_window *win = &f->fmt.win;
	u32 key_value =  0;

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	win->w = vout->win.w;
	win->field = vout->win.field;
	win->global_alpha = vout->win.global_alpha;

	if (ovl->manager && ovl->manager->get_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		key_value = info.trans_key;
	}
	win->chromakey = key_value;
	return 0;
}

static int vidioc_cropcap(struct file *file, void *fh,
			struct v4l2_cropcap *cropcap)
{
	struct omap_vout_device *vout = fh;
	enum v4l2_buf_type type = cropcap->type;
	struct v4l2_pix_format *pix = &vout->pix;

	cropcap->type = type;
	if (type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Width and height are always even */
	cropcap->bounds.width = pix->width & ~1;
	cropcap->bounds.height = pix->height & ~1;

	omap_vout_default_crop(&vout->pix, &vout->fbuf, &cropcap->defrect);
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh,
	struct v4l2_crop *crop)
{
	struct omap_vout_device *vout = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = vout->crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh,
			struct v4l2_crop *crop)
{
	struct omap_vout_device *vout = fh;
	int err = -EINVAL;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	/* Currently we only allow changing the crop position while
	   streaming.  */
	if (vout->streaming &&
	    (crop->c.height != vout->crop.height ||
	     crop->c.width != vout->crop.width))
		return -EBUSY;

	mutex_lock(&vout->lock);
	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	if (!ovl->manager || !ovl->manager->device) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	if (rotate_90_or_270(vout)) {
		vout->fbuf.fmt.height = timing->x_res;
		vout->fbuf.fmt.width = timing->y_res;
	} else {
		vout->fbuf.fmt.height = timing->y_res;
		vout->fbuf.fmt.width = timing->x_res;
	}

	if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		/* omap_vout_new_corp sets the use_isp_rsz_for_downscale
		 * flag based on whether there is a need for scaling beyond
		 * 4x. ISP resizer is used for downscaling beyond 1/4x as DSS
		 * doesn't support it.
		 */
		err = omap_vout_new_crop(&vout->pix, &vout->crop, &vout->win,
					 &vout->fbuf, &crop->c,
					 &vout->use_isp_rsz_for_downscale);
		mutex_unlock(&vout->lock);
		return err;
	} else {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
}

static int vidioc_queryctrl(struct file *file, void *fh,
		struct v4l2_queryctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		v4l2_ctrl_query_fill(ctrl, 0, 270, 90, 0);
		break;
	case V4L2_CID_BG_COLOR:
		v4l2_ctrl_query_fill(ctrl, 0, 0xFFFFFF, 1, 0);
		break;
	case V4L2_CID_VFLIP:
		v4l2_ctrl_query_fill(ctrl, 0, 1, 1, 0);
	default:
		ctrl->name[0] = '\0';
		return -EINVAL;
	}
	return 0;
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	struct omap_vout_device *vout = fh;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctrl->value = vout->control[0].value;
		return 0;
	case V4L2_CID_BG_COLOR:
	{
		struct omap_overlay_manager_info info;
		struct omap_overlay *ovl;
		ovl = vout->vid_info.overlays[0];

		if (!ovl->manager || !ovl->manager->get_manager_info)
			return -EINVAL;

		ovl->manager->get_manager_info(ovl->manager, &info);
		ctrl->value = info.default_color;
		return 0;
	}
	case V4L2_CID_VFLIP:
		ctrl->value = vout->control[2].value;
		return 0;
	case V4L2_CID_TI_DISPC_OVERLAY:
		ctrl->value = vout->vid_info.overlays[0]->id;
		return 0;
	case V4L2_CID_LINK:
		ctrl->value = vout->control[3].value;
		return 0;
	default:
		return -EINVAL;
	}
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct omap_vout_device *vout = fh;

	switch (a->id) {
	case V4L2_CID_ROTATE:
	{
		int rotation = a->value;

		mutex_lock(&vout->lock);

		if (rotation &&
			vout->pix.pixelformat == V4L2_PIX_FMT_RGB24) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}

		if ((v4l2_rot_to_dss_rot(rotation, &vout->rotation,
				vout->mirror))) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}

		vout->control[0].value = rotation;
		mutex_unlock(&vout->lock);
		return 0;
	}
	case V4L2_CID_BG_COLOR:
	{
		unsigned int  color = a->value;
		struct omap_overlay_manager_info info;
		struct omap_overlay *ovl;
		ovl = vout->vid_info.overlays[0];

		mutex_lock(&vout->lock);
		if (!ovl->manager || !ovl->manager->get_manager_info) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}

		ovl->manager->get_manager_info(ovl->manager, &info);
		info.default_color = color;
		if (ovl->manager->set_manager_info(ovl->manager, &info)) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}

		vout->control[1].value = color;
		mutex_unlock(&vout->lock);
		return 0;
	}
	case V4L2_CID_VFLIP:
	{
		unsigned int  mirror = a->value;
		struct omapvideo_info *ovid;
		struct omap_overlay *ovl;
		ovid = &vout->vid_info;
		ovl = ovid->overlays[0];

		mutex_lock(&vout->lock);

		if (mirror  && vout->pix.pixelformat == V4L2_PIX_FMT_RGB24) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}
		vout->mirror = mirror;
		vout->control[2].value = mirror;
		mutex_unlock(&vout->lock);
		return 0;
	}
	case V4L2_CID_LINK:
	{
		unsigned int linked = a->value;
		mutex_lock(&vout->lock);
		vout->linked = linked;
		vout->control[3].value = linked;
		mutex_unlock(&vout->lock);
		return 0;
	}
	default:
		return -EINVAL;
	}

}

static int vidioc_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	unsigned int i;
#ifndef TILER_ALLOCATE_V4L2
	unsigned int num_buffers = 0;
#endif
	int ret = 0;
	struct videobuf_dmabuf *dmabuf = NULL;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "entered REQbuf: \n");

	if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) || (req->count < 0))
		return -EINVAL;
	/* if memory is not mmp or userptr
	   return error */
	if ((V4L2_MEMORY_MMAP != req->memory) &&
		(V4L2_MEMORY_USERPTR != req->memory))
		return -EINVAL;

	mutex_lock(&vout->lock);
	/* Cannot be requested when streaming is on */
	if (vout->streaming) {
		mutex_unlock(&vout->lock);
		return -EBUSY;
	}

	/*
	* struct v4l2_requestbuffers reserved field used to define
	* cacheable/non-cacheable and flushable/non-flushable buffers
	* from user space
	*/
	cacheable_buffers = (req->reserved[0] == 1) ? 1 : 0;
	flushable_buffers = (req->reserved[1] == 1) ? 1 : 0;

#ifndef CONFIG_ARCH_OMAP4
	/*check for 720p format*/
	if (vout->pix.height * vout->pix.width == VID_MAX_WIDTH * 720)
		vout->use_isp_rsz_for_downscale = 1;

	vout->rsz_configured = 0;
#endif
	/* If buffers are already allocated free them */
	if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory)) {
		if (vout->mmap_count) {
			mutex_unlock(&vout->lock);
			return -EBUSY;
		}

#ifndef TILER_ALLOCATE_V4L2
#ifdef CONFIG_ARCH_OMAP4
		if (OMAP_VIDEO3 == vout->vid)
			num_buffers = video3_numbuffers;
		else
#endif
			num_buffers = (vout->vid == OMAP_VIDEO1) ?
				video1_numbuffers : video2_numbuffers;

		for (i = num_buffers; i < vout->buffer_allocated; i++) {
			dmabuf = videobuf_to_dma(q->bufs[i]);
			omap_vout_free_buffer((u32)dmabuf->vmalloc,
				dmabuf->bus_addr, vout->buffer_size);
			vout->buf_virt_addr[i] = 0;
			vout->buf_phy_addr[i] = 0;
		}
		vout->buffer_allocated = num_buffers;
#else  /* TILER_ALLOCATE_V4L2*/
		omap_vout_tiler_buffer_free(vout, vout->buffer_allocated, 0);
		vout->buffer_allocated = 0;

#endif /* TILER_ALLOCATE_V4L2*/
	videobuf_mmap_free(q);


	} else if (q->bufs[0] && (V4L2_MEMORY_USERPTR == q->bufs[0]->memory)) {
		if (vout->buffer_allocated) {
			videobuf_mmap_free(q);
			for (i = 0; i < vout->buffer_allocated; i++) {
				kfree(q->bufs[i]);
				q->bufs[i] = NULL;
			}
			vout->buffer_allocated = 0;
		}
	}
	/*store the memory type in data structure */
	vout->memory = req->memory;

	INIT_LIST_HEAD(&vout->dma_queue);

	/* call videobuf_reqbufs api */
	ret = videobuf_reqbufs(q, req);
	if (ret < 0) {
		mutex_unlock(&vout->lock);
		return ret;
	}

	vout->buffer_allocated = req->count;
	for (i = 0; i < req->count; i++) {
		dmabuf = videobuf_to_dma(q->bufs[i]);
#ifdef CONFIG_ARCH_OMAP4
		if (V4L2_PIX_FMT_NV12 == vout->pix.pixelformat) {
#ifndef TILER_ALLOCATE_V4L2
		dmabuf->vmalloc = (void *) omapvid_get_uvbase_nv12(
							vout->buf_phy_addr[i],
							vout->pix.height,
							vout->pix.width);
#else
		dmabuf->vmalloc = (void *) vout->buf_phy_uv_addr[i];
#endif /* TILER_ALLOCATE_V4L2 */
		} else
		dmabuf->vmalloc = NULL;
#else
		dmabuf->vmalloc = (void *) vout->buf_virt_addr[i];
#endif
		dmabuf->bus_addr = (dma_addr_t) vout->buf_phy_addr[i];
		dmabuf->sglen = 1;
	}
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_querybuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct omap_vout_device *vout = fh;

	return videobuf_querybuf(&vout->vbq, b);
}

static int vidioc_qbuf(struct file *file, void *fh,
			struct v4l2_buffer *buffer)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	unsigned int count;
	int ret = 0;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"entered qbuf: buffer address: %x \n", (unsigned int) buffer);

	if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != buffer->type) ||
			(buffer->index >= vout->buffer_allocated) ||

			(q->bufs[buffer->index]->memory != buffer->memory)) {
		return -EINVAL;
	}
	if (V4L2_MEMORY_USERPTR == buffer->memory) {
		if ((buffer->length < vout->pix.sizeimage) ||
			(0 == buffer->m.userptr)) {
			return -EINVAL;
		}
	}

#ifndef CONFIG_ARCH_OMAP4
	if ((rotation_enabled(vout)) &&
			vout->vrfb_dma_tx.req_status == DMA_CHAN_NOT_ALLOTED) {
		printk(KERN_WARNING VOUT_NAME
				"DMA Channel not allocated for Rotation\n");
		return -EINVAL;
	}

	/* initialize the ISP resizer*/
	ret = init_isp_rsz(vout);
	if (ret)
		return ret;

	/* setup the vrfb so that the first frames are also setup
	* correctly in the vrfb
	*/
	if (vrfb_configured == 0) {
		count = vout->buffer_allocated;
		omap_vout_vrfb_buffer_setup(vout, &count, 0);
		vrfb_configured = 1;
	}
#endif

	ret = videobuf_qbuf(q, buffer);
	/* record buffer offset from crop window */
	if (omap_vout_calculate_offset(vout, buffer->index)) {
		printk(KERN_ERR "Could not calculate buffer offset\n");
		return -EINVAL;
	}
	return ret;
}

static int vidioc_dqbuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int ret = 0;

	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev,
		"entered DQbuf: buffer address: %x \n", (unsigned int) b);

	if (!vout->streaming)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
		/* Call videobuf_dqbuf for non blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *)b, 1);
	else
		/* Call videobuf_dqbuf for  blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *)b, 0);
	return ret;
}

static int vidioc_streamon(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	struct vout_platform_data *pdata = (vout->dev)->platform_data;
	u32 addr = 0, uv_addr = 0;
	int r = 0;
	int t;
	struct omapvideo_info *ovid = &vout->vid_info;
	u32 mask = 0;
#ifndef CONFIG_ARCH_OMAP4
	unsigned int count;
#endif

	mutex_lock(&vout->lock);

	if (vout->streaming) {
		mutex_unlock(&vout->lock);
		return -EBUSY;
	}

	r = videobuf_streamon(q);
	if (r < 0) {
		mutex_unlock(&vout->lock);
		return r;
	}

	if (list_empty(&vout->dma_queue)) {
		mutex_unlock(&vout->lock);
		return -EIO;
	}

#ifndef CONFIG_ARCH_OMAP4
	/*check for 720p format*/
	if (vout->pix.height * vout->pix.width == VID_MAX_WIDTH * 720)
		vout->use_isp_rsz_for_downscale = 1;
#endif

	/* Get the next frame from the buffer queue */
	vout->next_frm = vout->cur_frm = list_entry(vout->dma_queue.next,
				struct videobuf_buffer, queue);
	/* Remove buffer from the buffer queue */
	list_del(&vout->cur_frm->queue);
	/* Mark state of the current frame to active */
	vout->cur_frm->state = VIDEOBUF_ACTIVE;
	/* Initialize field_id and started member */
	vout->field_id = 0;

	/* set flag here. Next QBUF will start DMA */
	vout->streaming = 1;

	vout->first_int = 1;

	addr = (unsigned long) vout->queued_buf_addr[vout->cur_frm->i]
		+ vout->cropped_offset[vout->cur_frm->i];
#ifdef CONFIG_ARCH_OMAP4
	uv_addr = (unsigned long) vout->queued_buf_uv_addr[vout->cur_frm->i]
		+ vout->cropped_uv_offset[vout->cur_frm->i];
#else
	count = vout->buffer_allocated;
	omap_vout_vrfb_buffer_setup(vout, &count, 0);
#endif

	if (vout->linked) {
		if (omapvid_link_en_ovl(1, addr, uv_addr))
			return -EINVAL;
	} else {
		for (t = 0; t < ovid->num_overlays; t++) {
			struct omap_overlay *ovl = ovid->overlays[t];
			if (ovl->manager && ovl->manager->device) {
				struct omap_overlay_info info;
				ovl->get_overlay_info(ovl, &info);
				info.enabled = 1;
				info.paddr = addr;

				if (cpu_is_omap44xx())
					info.p_uv_addr = uv_addr;

				if (ovl->set_overlay_info(ovl, &info)) {
					mutex_unlock(&vout->lock);
					return -EINVAL;
				}
			}
		}
	}
	/* First save the configuration in ovelray structure */
	r = omapvid_init(vout, addr, uv_addr);
	if (r)
		printk(KERN_ERR VOUT_NAME "failed to set overlay info\n");
	/* Enable the pipeline and set the Go bit */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");

#ifdef CONFIG_PM
	if (!cpu_is_omap44xx()) {
		/*
		* Hold a max constraint on ARM. i.e., limit ARM
		* to scale beyond 1G. As DSP freq is 65Mhz when
		* ARM runs above 1.2G.
		* Below api is provided by PM SRF layer to hold max
		* constraint. Release this in streamoff.
		*/
		omap_pm_vdd1_set_max_opp(vout->dev, VDD1_OPP4);

		/*
		* Through-put requirement.
		* Set max OCP freq:
		* 		For 3630 is 200 MHz
		*		For 3430 is 166 MHz
		* Through-put is in KByte/s
		* so OCP_FREQ KHz * 4 =
		*			800 KByte/s for 3630
		*			664 KByte/s for 3430
		*/
		pdata->set_min_bus_tput(vout->dev, OCP_INITIATOR_AGENT,
				VDD2_OCP_FREQ_CONST * 4);
		tick_nohz_disable(1);
		/* Wait for 2 vsyncs before start processing buffers */
		mdelay(32);
	}
#endif

	mask = DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_EVEN |
			DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_FRAMEDONE;
#ifdef CONFIG_ARCH_OMAP4
	mask = mask | DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2;
#endif
	omap_dispc_register_isr(omap_vout_isr, vout, mask);
	mutex_unlock(&vout->lock);
	return r;
}

static int vidioc_streamoff(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct omap_vout_device *vout = fh;
	int t, r = 0;
	struct vout_platform_data *pdata = (vout->dev)->platform_data;
	struct omapvideo_info *ovid = &vout->vid_info;
	u32 mask = 0;

	if (!vout->streaming)
		return -EINVAL;

	vout->streaming = 0;
	mask = DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_EVEN |
			DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_FRAMEDONE;
#ifdef CONFIG_ARCH_OMAP4
	mask =	mask | DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2;
#endif
	omap_dispc_unregister_isr(omap_vout_isr, vout, mask);

	if (vout->linked) {
		if (omapvid_link_en_ovl(0, 0, 0))
			return -EINVAL;
	} else {
		for (t = 0; t < ovid->num_overlays; t++) {
			struct omap_overlay *ovl = ovid->overlays[t];
			if (ovl->manager && ovl->manager->device) {
				struct omap_overlay_info info;
				ovl->get_overlay_info(ovl, &info);
				info.enabled = 0;
				if (ovl->set_overlay_info(ovl, &info)) {
					printk(KERN_ERR VOUT_NAME "failed to \
						update overlay info\n");
					return -EINVAL;
				}
			}
		}
	}

	/* Turn of the pipeline */
	r = omapvid_apply_changes(vout);
	if (r) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		return r;
	}
	INIT_LIST_HEAD(&vout->dma_queue);

#ifndef CONFIG_ARCH_OMAP4
	/*release resizer now */
	if (vout->use_isp_rsz_for_downscale) {
		if (vout->rsz_configured == 1) {
			ispdss_put_resource();
			printk(KERN_ERR "<%s> rsz_put_resource\n",
			       __func__);
			vout->rsz_configured = 0;
			vout->use_isp_rsz_for_downscale = 0;
		}
	}

	vrfb_configured = 0;
#endif
	videobuf_streamoff(&vout->vbq);
	videobuf_queue_cancel(&vout->vbq);

#ifdef CONFIG_PM
	if (!cpu_is_omap44xx()) {
		/*Wait for 2 vsyncs after finish processing buffers */
		mdelay(32);
		/* Releasing PM constraints */
		omap_pm_vdd1_set_max_opp(vout->dev, 0);
		pdata->set_min_bus_tput(vout->dev, OCP_INITIATOR_AGENT, 0);
		tick_nohz_disable(0);
	}
#endif
	return 0;
}

static int vidioc_s_fbuf(struct file *file, void *fh,
		struct v4l2_framebuffer *a)
{
	struct omap_vout_device *vout = fh;
	struct omap_overlay_manager_info info;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	enum omap_dss_trans_key_type key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
	int enable = 0;

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	/* OMAP DSS doesn't support Source and Destination color
	   key together */
	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_CHROMAKEY))
		return -EINVAL;
	/* OMAP DSS Doesn't support the Destination color key
	   and alpha blending together */
	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA))
		return -EINVAL;

	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY)) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
		key_type =  OMAP_DSS_COLOR_KEY_VID_SRC;
	} else
		vout->fbuf.flags &= ~V4L2_FBUF_FLAG_SRC_CHROMAKEY;

	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY)) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
		key_type =  OMAP_DSS_COLOR_KEY_GFX_DST;
	} else
		vout->fbuf.flags &=  ~V4L2_FBUF_FLAG_CHROMAKEY;

	if (a->flags & (V4L2_FBUF_FLAG_CHROMAKEY |
				V4L2_FBUF_FLAG_SRC_CHROMAKEY))
		enable = 1;
	else
		enable = 0;
	if (ovl->manager && ovl->manager->get_manager_info &&
			ovl->manager->set_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		info.trans_enabled = enable;
		info.trans_key_type = key_type;
		info.trans_key = vout->win.chromakey;

		if (ovl->manager->set_manager_info(ovl->manager, &info))
			return -EINVAL;
	}
	if (a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 1;
	} else {
		vout->fbuf.flags &= ~V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 0;
	}
	if (ovl->manager && ovl->manager->get_manager_info &&
			ovl->manager->set_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		info.alpha_enabled = enable;
		if (ovl->manager->set_manager_info(ovl->manager, &info))
			return -EINVAL;
	}

	return 0;
}

static int vidioc_g_fbuf(struct file *file, void *fh,
		struct v4l2_framebuffer *a)
{
	struct omap_vout_device *vout = fh;
	struct omap_overlay_manager_info info;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;

	ovid = &vout->vid_info;
	ovl = ovid->overlays[0];

	a->flags = 0x0;

	a->capability = V4L2_FBUF_CAP_LOCAL_ALPHA | V4L2_FBUF_CAP_CHROMAKEY
		| V4L2_FBUF_CAP_SRC_CHROMAKEY;

	if (ovl->manager && ovl->manager->get_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		if (info.trans_key_type == OMAP_DSS_COLOR_KEY_VID_SRC)
			a->flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
		if (info.trans_key_type == OMAP_DSS_COLOR_KEY_GFX_DST)
			a->flags |= V4L2_FBUF_FLAG_CHROMAKEY;
	}
	if (ovl->manager && ovl->manager->get_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		if (info.alpha_enabled)
			a->flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
	}

	return 0;
}

static const struct v4l2_ioctl_ops vout_ioctl_ops = {
	.vidioc_querycap      			= vidioc_querycap,
	.vidioc_enum_fmt_vid_out 		= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out			= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out			= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out			= vidioc_s_fmt_vid_out,
	.vidioc_queryctrl    			= vidioc_queryctrl,
	.vidioc_g_ctrl       			= vidioc_g_ctrl,
	.vidioc_s_fbuf				= vidioc_s_fbuf,
	.vidioc_g_fbuf				= vidioc_g_fbuf,
	.vidioc_s_ctrl       			= vidioc_s_ctrl,
	.vidioc_try_fmt_vid_overlay 		= vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay		= vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_overlay		= vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay		= vidioc_g_fmt_vid_overlay,
	.vidioc_cropcap				= vidioc_cropcap,
	.vidioc_g_crop				= vidioc_g_crop,
	.vidioc_s_crop				= vidioc_s_crop,
	.vidioc_reqbufs				= vidioc_reqbufs,
	.vidioc_querybuf			= vidioc_querybuf,
	.vidioc_qbuf				= vidioc_qbuf,
	.vidioc_dqbuf				= vidioc_dqbuf,
	.vidioc_streamon			= vidioc_streamon,
	.vidioc_streamoff			= vidioc_streamoff,
};

static const struct v4l2_file_operations omap_vout_fops = {
	.owner 		= THIS_MODULE,
	.ioctl 		= video_ioctl2,
	.mmap 		= omap_vout_mmap,
	.open 		= omap_vout_open,
	.release 	= omap_vout_release,
};

/* Init functions used during driver intitalization */
/* Initial setup of video_data */
static int __init omap_vout_setup_video_data(struct omap_vout_device *vout)
{
	struct v4l2_pix_format *pix;
	struct video_device *vfd;
	struct v4l2_control *control;
	struct omap_dss_device *display =
		vout->vid_info.overlays[0]->manager->device;

	/* set the default pix */
	pix = &vout->pix;

	/* Set the default picture of QVGA  */
	pix->width = QQVGA_WIDTH;
	pix->height = QQVGA_HEIGHT;

	/* Default pixel format is RGB 5-6-5 */
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_ANY;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	vout->bpp = RGB565_BPP;
	vout->fbuf.fmt.width  =  display->panel.timings.x_res;
	vout->fbuf.fmt.height =  display->panel.timings.y_res;

	/* Set the data structures for the overlay parameters*/
	vout->win.global_alpha = 255;
	vout->fbuf.flags = 0;
	vout->fbuf.capability = V4L2_FBUF_CAP_LOCAL_ALPHA |
		V4L2_FBUF_CAP_SRC_CHROMAKEY | V4L2_FBUF_CAP_CHROMAKEY;
	vout->win.chromakey = 0;

	omap_vout_new_format(pix, &vout->fbuf, &vout->crop, &vout->win);

	/*Initialize the control variables for
	  rotation, flipping and background color. */
	control = vout->control;
	control[0].id = V4L2_CID_ROTATE;
	control[0].value = 0;
	vout->rotation = 0;
	vout->mirror = 0;
	vout->control[2].id = V4L2_CID_HFLIP;
	vout->control[2].value = 0;
	vout->vrfb_bpp = 2;
	vout->linked = 0;
	vout->use_isp_rsz_for_downscale = 0;

	control[1].id = V4L2_CID_BG_COLOR;
	control[1].value = 0;

	control[3].id = V4L2_CID_LINK;
	control[3].value = 0;

	/* initialize the video_device struct */
	vfd = vout->vfd = video_device_alloc();

	if (!vfd) {
		printk(KERN_ERR VOUT_NAME ": could not allocate"
				" video device struct\n");
		return -ENOMEM;
	}
	vfd->release = video_device_release;
	vfd->ioctl_ops = &vout_ioctl_ops;

	strlcpy(vfd->name, VOUT_NAME, sizeof(vfd->name));
	vfd->vfl_type = VFL_TYPE_GRABBER;

	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->fops = &omap_vout_fops;
	mutex_init(&vout->lock);

	vfd->minor = -1;
	return 0;

}

/* Setup video buffers */
static int __init omap_vout_setup_video_bufs(struct platform_device *pdev,
		int vid_num)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct omap2video_device *vid_dev = container_of(v4l2_dev, struct
			omap2video_device, v4l2_dev);
	struct omap_vout_device *vout;
	struct video_device *vfd;
#ifndef TILER_ALLOCATE_V4L2 /* TODO: related to rotation */
	int i, j, r = 0;
	unsigned numbuffers;
	int image_width, image_height;
	int static_vrfb_allocation = 0, vrfb_num_bufs = OMAP_VOUT_MAX_BUFFERS;
#endif
	vout = vid_dev->vouts[vid_num];
	vfd = vout->vfd;

#ifndef TILER_ALLOCATE_V4L2
#ifdef CONFIG_ARCH_OMAP4
	if (OMAP_VIDEO3 == vid_num) {
		numbuffers = video3_numbuffers;
		vout->buffer_size = video3_bufsize;
	} else
#endif
	{
		numbuffers = (vid_num == 0)	? video1_numbuffers
						: video2_numbuffers;
		vout->buffer_size = (vid_num == 0)	? video1_bufsize
							: video2_bufsize;
	}
	v4l2_dbg(1, debug, &vout->vid_dev->v4l2_dev, "Buffer Size = %d\n",
		vout->buffer_size);

	for (i = 0; i < numbuffers; i++) {
		vout->buf_virt_addr[i] =
			omap_vout_alloc_buffer(vout->buffer_size,
					(u32 *) &vout->buf_phy_addr[i]);
		if (!vout->buf_virt_addr[i]) {
			numbuffers = i;
			r = -ENOMEM;
			goto free_buffers;
		}
		vout->cropped_offset[i] = 0;
#ifdef CONFIG_ARCH_OMAP4
		vout->cropped_uv_offset[i] = 0;
#endif
	}

#ifndef CONFIG_ARCH_OMAP4
	for (i = 0; i < OMAP_VOUT_MAX_VBUF_CTXT; i++) {
		if (omap_vrfb_request_ctx(&vout->vrfb_context[i])) {
			printk(KERN_INFO VOUT_NAME ": VRFB Region allocation "
					"for rotation failed\n");
			r = -ENOMEM;
			break;
		}
	}
	if (r == -ENOMEM) {
		for (j = 0; j < i; j++)
			omap_vrfb_release_ctx(&vout->vrfb_context[j]);

		goto free_buffers;
	}

	/* Calculate VRFB memory size */
	/* allocate for worst case size */
	image_width = VID_MAX_WIDTH / TILE_SIZE;
	if (VID_MAX_WIDTH % TILE_SIZE)
		image_width++;

	image_width = image_width * TILE_SIZE;
	image_height = VID_MAX_HEIGHT / TILE_SIZE;

	if (VID_MAX_HEIGHT % TILE_SIZE)
		image_height++;

	image_height = image_height * TILE_SIZE;
	vout->smsshado_size = PAGE_ALIGN(image_width * image_height * 2 * 2);

	/*
	 * Request and Initialize DMA, for DMA based VRFB transfer
	 */
	vout->vrfb_dma_tx.dev_id = OMAP_DMA_NO_DEVICE;
	vout->vrfb_dma_tx.dma_ch = -1;
	vout->vrfb_dma_tx.req_status = DMA_CHAN_ALLOTED;
	r = omap_request_dma(vout->vrfb_dma_tx.dev_id, "VRFB DMA TX",
			omap_vout_vrfb_dma_tx_callback,
			(void *) &vout->vrfb_dma_tx, &vout->vrfb_dma_tx.dma_ch);
	if (r < 0) {
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		printk(KERN_INFO VOUT_NAME ": DMA Channel not alloted "
				"for video%d [v4l2]\n", vfd->minor);
	}
	init_waitqueue_head(&vout->vrfb_dma_tx.wait);

	/* Allocate VRFB buffers if selected through bootargs */
	static_vrfb_allocation = (vid_num == 0) ?
		vid1_static_vrfb_alloc : vid2_static_vrfb_alloc;

	/* statically allocated the VRFB buffer is done through
	   commands line aruments */
	if (static_vrfb_allocation) {
		if (omap_vout_allocate_vrfb_buffers(vout, &vrfb_num_bufs, -1)) {
			r =  -ENOMEM;
			goto free_buffers;
		}
		vout->vrfb_static_allocation = 1;
	}
	return 0;
#endif /* CONFIG_ARCH_OMAP4 */

free_buffers:
	for (i = 0; i < numbuffers; i++) {
		omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i], vout->buffer_size);
		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}
	return r;
#endif /* TILER_ALLOCATE_V4L2 */

	/* NOTE: OMAP4, if TILER allocation, then nothing to pre-allocate */
	return 0;
}

/* Create video out devices */
static int __init omap_vout_create_video_devices(struct platform_device *pdev)
{
	int r = 0, k;
	struct omap_vout_device *vout;
	struct video_device *vfd = NULL;
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);

	struct omap2video_device *vid_dev = container_of(v4l2_dev, struct
			omap2video_device, v4l2_dev);

	for (k = 0; k < pdev->num_resources; k++) {

		vout = kmalloc(sizeof(struct omap_vout_device), GFP_KERNEL);
		if (!vout) {
			printk(KERN_ERR VOUT_NAME
					": could not allocate memory\n");
			return -ENOMEM;
		}

		memset(vout, 0, sizeof(struct omap_vout_device));

		vout->dev = &pdev->dev;
		vout->vid = k;
		vid_dev->vouts[k] = vout;
		vout->vid_dev = vid_dev;
#ifndef CONFIG_ARCH_OMAP4
		/* Select video2 if only 1 overlay is controlled by V4L2 */
		if (pdev->num_resources == 1)
			vout->vid_info.overlays[0] = vid_dev->overlays[k + 2];
		else
			/* Else select video1 and video2 one by one. */
			vout->vid_info.overlays[0] = vid_dev->overlays[k + 1];
#else
		vout->vid_info.overlays[0] =
			vid_dev->overlays[
				k + (OMAP4_MAX_OVERLAYS - pdev->num_resources)];
#endif
		vout->vid_info.num_overlays = 1;
		vout->vid_info.id = k + 1;
		vid_dev->num_videos++;

		/* Setup the default configuration for the video devices
		 */
		if (omap_vout_setup_video_data(vout) != 0) {
			r = -ENOMEM;
			goto error;
		}

		/* Allocate default number of buffers for the video streaming
		 * and reserve the VRFB space for rotation
		 */
		if (omap_vout_setup_video_bufs(pdev, k) != 0) {
			r = -ENOMEM;
			goto error1;
		}

		/* Register the Video device with V4L2
		 */
		vfd = vout->vfd;
		if (video_register_device(vfd, VFL_TYPE_GRABBER, k + 1) < 0) {
			printk(KERN_ERR VOUT_NAME ": could not register "
					"Video for Linux device\n");
			vfd->minor = -1;
			r = -ENODEV;
			goto error2;
		}
		video_set_drvdata(vfd, vout);

		/* Configure the overlay structure */
		r = omapvid_init(vid_dev->vouts[k], 0, 0);

		if (r)
			goto error2;
		else
			goto success;
error2:
#ifndef CONFIG_ARCH_OMAP4
	omap_vout_release_vrfb(vout);
#endif
	omap_vout_free_buffers(vout);
error1:
	video_device_release(vfd);
error:
	kfree(vout);
	return r;

success:
	printk(KERN_INFO VOUT_NAME ": registered and initialized "
			"video device %d [v4l2]\n", vfd->minor);
	if (k == (pdev->num_resources - 1))
		return 0;
	}
	return -ENODEV;

}
/* Driver functions */
static int omap_vout_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct omap2video_device *vid_dev = container_of(v4l2_dev, struct
			omap2video_device, v4l2_dev);
	int k;

	v4l2_device_unregister(v4l2_dev);
	for (k = 0; k < pdev->num_resources; k++)
		omap_vout_cleanup_device(vid_dev->vouts[k]);

	for (k = 0; k < vid_dev->num_displays; k++) {
		if (vid_dev->displays[k]->state != OMAP_DSS_DISPLAY_DISABLED)
			vid_dev->displays[k]->disable(vid_dev->displays[k]);

		omap_dss_put_device(vid_dev->displays[k]);
	}
	kfree(vid_dev);
	return 0;
}

static int __init omap_vout_probe(struct platform_device *pdev)
{
	int r = 0, i;
	struct omap2video_device *vid_dev = NULL;
	struct omap_overlay *ovl;
	struct omap_dss_device *def_display;
	struct omap_dss_device *dssdev;

	if (pdev->num_resources == 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		r = -ENODEV;
		return r;
	}

	vid_dev = kzalloc(sizeof(struct omap2video_device), GFP_KERNEL);
	if (vid_dev == NULL) {
		r = -ENOMEM;
		return r;
	}

	vid_dev->num_displays = 0;
	dssdev = NULL;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		vid_dev->displays[vid_dev->num_displays++] = dssdev;
	}

	if (vid_dev->num_displays == 0) {
		dev_err(&pdev->dev, "no displays\n");
		r = -EINVAL;
		goto error0;
	}

	vid_dev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < vid_dev->num_overlays; i++)
		vid_dev->overlays[i] = omap_dss_get_overlay(i);

	vid_dev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < vid_dev->num_managers; i++)
		vid_dev->managers[i] = omap_dss_get_overlay_manager(i);


	/* Get the Video1, video2 (and Video3 for OMAP4) overlay.
	 * Setup the Display attached to that overlays
	 */
	for (i = 1; i < (NUM_OF_VIDEO_CHANNELS + 1); i++) {
		ovl = omap_dss_get_overlay(i);
		if (!ovl) {
			pr_err("Could not get overlay %d\n", i);
			WARN_ON(1);
			continue;
		}
		if (ovl->manager && ovl->manager->device) {
			def_display = ovl->manager->device;
		} else {
			dev_warn(&pdev->dev, "cannot find display\n");
			def_display = NULL;
		}
		if (def_display) {
			r = 0;
			if (def_display->state == OMAP_DSS_DISPLAY_DISABLED) {
				r = def_display->enable(def_display);
			}
			if (r) {
				/* Here we are not considering a error
				 *  as display may be enabled by frame
				 *  buffer driver
				 */
				dev_warn(&pdev->dev,
					"'%s' Display already enabled\n",
					def_display->name);
			}
			/* set the update mode */
			if (def_display->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
#ifdef CONFIG_FB_OMAP2_FORCE_AUTO_UPDATE
				if (def_display->enable_te)
					def_display->enable_te(def_display, 1);
				if (def_display->set_update_mode)
					def_display->set_update_mode(def_display,
							OMAP_DSS_UPDATE_AUTO);
#else	/* MANUAL_UPDATE */
				if (def_display->enable_te)
					def_display->enable_te(def_display, 0);
				if (def_display->set_update_mode)
					def_display->set_update_mode(def_display,
							OMAP_DSS_UPDATE_MANUAL);
#endif
			} else {
				if (def_display->set_update_mode)
					def_display->set_update_mode(def_display,
							OMAP_DSS_UPDATE_AUTO);
			}
		}
	}

	if (v4l2_device_register(&pdev->dev, &vid_dev->v4l2_dev) < 0) {
		printk(KERN_ERR VOUT_NAME "v4l2_device_register failed\n");
		return -ENODEV;
	}

	r = omap_vout_create_video_devices(pdev);
	if (r)
		goto error0;

	for (i = 0; i < vid_dev->num_displays; i++) {
		struct omap_dss_device *display = vid_dev->displays[i];

		if (display->update)
			display->update(display, 0, 0,
					display->panel.timings.x_res,
					display->panel.timings.y_res);
	}
	return 0;

error0:
	kfree(vid_dev);
	return r;
}

static struct platform_driver omap_vout_driver = {
	.driver = {
		   .name = VOUT_NAME,
		   },
	.probe = omap_vout_probe,
	.remove = omap_vout_remove,
};

void omap_vout_isr(void *arg, unsigned int irqstatus)
{
	int r;
	struct timeval timevalue = {0};
	struct omap_vout_device *vout =
	    (struct omap_vout_device *) arg;
	u32 addr, uv_addr = 0;
	unsigned long flags;

#ifndef CONFIG_OMAP2_DSS_HDMI
	u32 fid;
#endif
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_dss_device *cur_display;
	int irq = 0;

	if (!vout->streaming)
		return;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];
	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device)
		return;
	cur_display = ovl->manager->device;

	if (cur_display->channel == OMAP_DSS_CHANNEL_LCD)
		irq = DISPC_IRQ_FRAMEDONE;

#ifdef CONFIG_ARCH_OMAP4
	else if (cur_display->channel == OMAP_DSS_CHANNEL_LCD2)
		irq = DISPC_IRQ_FRAMEDONE2;
#endif
	spin_lock_irqsave(&vout->vbq_lock, flags);
	do_gettimeofday(&timevalue);

	switch (cur_display->type) {

	case OMAP_DISPLAY_TYPE_DSI:
		if (!(irqstatus & irq)) {
			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
		break;

	case OMAP_DISPLAY_TYPE_DPI:
		if (!(irqstatus & (DISPC_IRQ_VSYNC | DISPC_IRQ_VSYNC2))) {
			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
#ifdef CONFIG_PANEL_PICO_DLP
		if (dispc_go_busy(OMAP_DSS_CHANNEL_LCD2)) {
			spin_unlock(&vout->vbq_lock);
			printk(KERN_WARNING "dpi busy %d !! \n",
				cur_display->type);
			return;
		}
#endif
		break;

#ifdef CONFIG_OMAP2_DSS_HDMI

	case OMAP_DISPLAY_TYPE_HDMI:
		if (!(irqstatus & DISPC_IRQ_EVSYNC_EVEN)) {
			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
		break;
#else
	case OMAP_DISPLAY_TYPE_VENC:
		if (vout->first_int) {
			vout->first_int = 0;
			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
		if (irqstatus & DISPC_IRQ_EVSYNC_ODD) {
			fid = 1;
		} else if (irqstatus & DISPC_IRQ_EVSYNC_EVEN) {
			fid = 0;
		} else {
			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
		vout->field_id ^= 1;
		if (fid != vout->field_id) {
			if (0 == fid)
				vout->field_id = fid;

			spin_unlock_irqrestore(&vout->vbq_lock, flags);
			return;
		}
		if (0 == fid) {
			if (vout->cur_frm == vout->next_frm) {
				spin_unlock_irqrestore(&vout->vbq_lock, flags);
				return;
			}
			vout->cur_frm->ts = timevalue;
			vout->cur_frm->state = VIDEOBUF_DONE;
			wake_up_interruptible(&vout->cur_frm->done);
			vout->cur_frm = vout->next_frm;
			goto end;
		} else if (1 == fid) {
			if (list_empty(&vout->dma_queue) ||
			    (vout->cur_frm != vout->next_frm)) {
				spin_unlock_irqrestore(&vout->vbq_lock, flags);
				return;
			}
			goto venc;
		}
#endif

	default:
		spin_unlock_irqrestore(&vout->vbq_lock, flags);
		return;
	}
	if (!vout->first_int && (vout->cur_frm != vout->next_frm)) {
		vout->cur_frm->ts = timevalue;
		vout->cur_frm->state = VIDEOBUF_DONE;
		wake_up_interruptible(&vout->cur_frm->done);
		vout->cur_frm = vout->next_frm;
	}

	vout->first_int = 0;
	if (list_empty(&vout->dma_queue)) {
		spin_unlock_irqrestore(&vout->vbq_lock, flags);
		return;
	}
#ifndef CONFIG_OMAP2_DSS_HDMI
venc:
#endif
	vout->next_frm = list_entry(vout->dma_queue.next,
		struct videobuf_buffer, queue);
	list_del(&vout->next_frm->queue);

	vout->next_frm->state = VIDEOBUF_ACTIVE;

	addr = (unsigned long)
		vout->queued_buf_addr[vout->next_frm->i]
		+ vout->cropped_offset[vout->next_frm->i];
#ifdef CONFIG_ARCH_OMAP4
	uv_addr = (unsigned long)vout->queued_buf_uv_addr[
		vout->next_frm->i]
		+ vout->cropped_uv_offset[vout->next_frm->i];
#endif

	if (vout->linked) {
		if (omapvid_link_en_ovl(1, addr, uv_addr)) {
			spin_unlock(&vout->vbq_lock);
			return;
		}
	} else {
		/* First save the configuration in ovelray structure */
		r = omapvid_init(vout, addr, uv_addr);
		if (r)
			printk(KERN_ERR VOUT_NAME
					"failed to set overlay info\n");
	}
	/* Enable the pipeline and set the Go bit */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");

	if (sysfs_streq(cur_display->name, "pico_DLP"))
		dispc_go(OMAP_DSS_CHANNEL_LCD2);

#ifndef CONFIG_OMAP2_DSS_HDMI
end:
#endif

	spin_unlock_irqrestore(&vout->vbq_lock, flags);
}

static void omap_vout_cleanup_device(struct omap_vout_device *vout)
{
	struct video_device *vfd;

	if (!vout)
		return;
	vfd = vout->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
	}

#ifndef CONFIG_ARCH_OMAP4
	omap_vout_release_vrfb(vout);
#endif
	omap_vout_free_buffers(vout);
#ifdef CONFIG_ARCH_OMAP4
#ifdef TILER_ALLOCATE_V4L2
	omap_vout_free_tiler_buffers(vout);
			/* TODO: check if this needs to be done? */
#endif
#else
	/* Free the VRFB buffer if allocated
	 * init time
	 */
	if (vout->vrfb_static_allocation)
		omap_vout_free_vrfb_buffers(vout);
#endif
	kfree(vout);
}

static int __init omap_vout_init(void)
{

	if (platform_driver_register(&omap_vout_driver) != 0) {
		printk(KERN_ERR VOUT_NAME ": could not register \
				Video driver\n");
		return -EINVAL;
	}
	mutex_init(&my_lock);
	return 0;
}

static void omap_vout_cleanup(void)
{
	platform_driver_unregister(&omap_vout_driver);
}

late_initcall(omap_vout_init);
module_exit(omap_vout_cleanup);
