/*
 * . 2004-01-03: SW.LEE <hitchcar@sec.samsung.com>
 *   
 * This file is subject to the terms and conditions of the GNU General Public
 * License 2. See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/tqueue.h>
#include <linux/locks.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>

#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>

#include <asm/arch/cpu_s3c2440.h>
#include <asm/arch/S3C2440.h>

#include "camif.h"
#include "videodev.h"

/* 
  Codec_formats/Preview_format[0] must be same to initial value of 
  preview_init_param/codec_init_param 
*/

const struct v4l2_fmtdesc codec_formats[] = {
	{
		.index     = 0,
		.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE,
//		.flags     = FORMAT_FLAGS_PLANAR,
		.description = "4:2:2, planar, Y-Cb-Cr",
		.pixelformat = V4L2_PIX_FMT_YUV422P,

	},{
		.index    = 1,
		.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
//		.flags    = FORMAT_FLAGS_PLANAR,
		.name     = "4:2:0, planar, Y-Cb-Cr",
		.fourcc   = V4L2_PIX_FMT_YUV420,
	}
};


/* Todo
   FIMC V4L2_PIX_FMT_RGB565 is not same to that of V4L2spec 
   and so we need image convert to FIMC V4l2_PIX_FMT_RGB565.
*/
const struct v4l2_fmtdesc preview_formats[] = {
	{
		.index    = 1,
		.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.description = "16 bpp RGB, le",
		.fourcc   = V4L2_PIX_FMT_RGB565,
//		.flags    = FORMAT_FLAGS_PACKED,
	},
	{
		.index    = 0,
		.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
//		.flags    = FORMAT_FLAGS_PACKED,
		.description = "32 bpp RGB, le",
		.fourcc   = V4L2_PIX_FMT_BGR32,
	}
}

#define NUM_F       ARRARY_SIZE(preview_formats)


/* 
 * This function and v4l2 structure made for V4L2 API functions 
 *     App <--> v4l2 <--> logical param <--> hardware
 */
static int camif_get_v4l2(camif_cfg_t *cfg)
{
	return 0;
}


/*
** Gives the depth of a video4linux2 fourcc aka pixel format in bits.
*/
static int pixfmt2depth(int pixfmt,int *fmtptr)
{
	int fmt, depth;    
	
	switch (pixfmt) {
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		fmt = CAMIF_RGB_16;
		depth = 16;
		break;
	case V4L2_PIX_FMT_BGR24: /* Not tested */
	case V4L2_PIX_FMT_RGB24:
		fmt = CAMIF_RGB_24;
		depth = 24;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		fmt = CAMIF_RGB_24;
		depth 32;
		break;
	case V4L2_PIX_FMT_GREY:	/* Not tested  */
		fmt = CAMIF_OUT_YCBCR420;
		depth = 8;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUV422P:
		fmt = CAMIF_OUT_YCBCR422;
		depth = 16;
		break;
	case V4L2_PIX_FMT_YUV420:
		fmt = CAMIF_OUT_YCBCR420;
		depth = 12;
		break;
	}
	if (fmtptr) *fmtptr = fmt;		
	return depth;
}



static int camif_s_v4l2(camif_cfg_t *cfg)
{
	int num = cfg->v2.used_fmt;

	if ( !(cfg->v2.status&CAMIF_V4L2_INIT)) {
		int depth;
		int fourcc = v2.fmtdesc[num].pixelformat;

		/* To define v4l2_fmtsdesc */
		if (cfg->dma_type == CAMIF_CODEC)
			cfg->v2->fmtdesc = codec_formats;
		else
			cfg->v2->fmtdesc = preview_formats;

		/* To define v4l2_format used currently */
		cfg->v2.fmt.width    = cfg->target_x;
		cfg->v2.fmt.height   = cfg->target_y;
		cfg->v2.fmt.field    = V4L2_FIELD_NONE;
		cfg->v2.fmt.pixelformat = fourcc;
		depth       = pixfmt2depth(fourcc,NULL);
		cfg->v2.fmt.bytesperline= cfg->v2.fmt.width*depth >> 3;
		cfg->v2.fmt.sizeimage = 
                       cfg->v2.fmt.height * cfg->v2.fmt.bytesperline;

		/* To define v4l2_input */
		cfg->v2.input.index  = 0;
		if (cfg->dma_type == CAMIF_CODEC)
			snprintf(cfg->v2.input.name, 31, "CAMIF CODEC");
		else
			snprintf(cfg->v2.input.name, 31, "CAMIF PREVIEW");
		cfg->v2.input.type = V4L2_INPUT_TYPE_CAMERA;
			
		/* Write the Status of v4l2 machine */
		cfg->v2.status |= CAMIF_V4L2_INIT;
	 }
	return 0;
}


static int camif_g_fmt(camif_cfg_t *cfg, struct v4l2_format *f)
{
	int size = sizeof(struct v4l2_pix_format);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(&f->fmt.pix,0,size);
		memcpy(&f->fmt.pix,&cfg->v2.fmt,size);
		return 0;
	default:
		return -EINVAL;
	}
}


/* Copy v4l2 parameter into other element of camif_cfg_t */
static int camif_s_try(camif_cfg_t *cfg, int f)
{
	int fmt;
	cfg->target_x = cfg->v2.fmt.width;
	cfg->target_y = cfg->v2.fmt.height;
	pixfmt2depth(cfg->v2.fmt.pixelformat,&fmt);
	cfg->fmt = fmt;
	camif_dynamic_conf(cfg);
}


static int camif_s_fmt(camif_cfg_t *cfg, struct v4l2_format *f)
{
	int retval;
	
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	{
		/* update our state informations */
//		down(&fh->cap.lock);
		cfg->v2.fmt          = f->pix;
		cfg->v2.status       |= CAMIF_v4L2_DIRTY;
		camif_dynamic_conf(cfg);
		cfg->v2.status      &= ~CAMIF_v4L2_DIRTY; /* dummy ? */
//		up(&fh->cap.lock);
		
		return 0;
	}
	default:
		return -EINVAL;
	}

}

/* Refer ioctl of  videodeX.c  and bttv-driver.c */
int camif_do_ioctl
(struct inode *inode, struct file *file,unsigned int cmd, void * arg)
{
	camif_cfg_t *cfg = file->private_data;
	int ret = 0;

	switch (cmd) {
        case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap = arg;
                                                                                                      
			strcpy(cap->driver,"Fimc Camera");
			strlcpy(cap->card,cfg->v->name,sizeof(cap->card));
			sprintf(cap->bus_info,"FIMC 2.0 AHB Bus");
			cap->version = 0;
			cap->capabilities =
				V4L2_CAP_VIDEO_CAPTURE |V4L2_CAP_READWRITE;
			return 0;
		}
	case VIDIOC_G_FMT:
		{
			struct v4l2_format *f = arg;
			return camif_g_fmt(cfg,f);
		}
	case VIDIOC_S_FMT:
		{
			struct v4l2_format *f = arg;
			return camif_s_fmt(cfg,f);
		}

	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *f = arg;
			enum v4l2_buf_type type = f->type;
			int index = f->index;

			if (index >= NUM_F) 
				return -EINVAL;
			switch (f->type) {
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				break;
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
			case V4L2_BUF_TYPE_VBI_CAPTURE:
					default:
			return -EINVAL;
			}
			memset(f,0,sizeof(*f));
			memcpy(f,cfg->v2.fmtdesc+index,sizeof(*f));
			return 0;
		}
	case VIDIOC_G_INPUT:
		{
			u32 *i = arg;
			*i = cfg->v2.input;
			return 0;
		}
	case VIDIOC_S_INPUT:
		{
			int index = *((int *)arg);
			if (index != 0) 
				return -EINVAL;
			cfg->v2.input.index = index;
			return 0;
		}
		
	default:
		return -ENOIOCTLCMD; /* errno.h */
	} /* End of Switch  */


}







/* 
 * Local variables:
 * tab-width: 8
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  c-set-style: "K&R"
 * End:
 */
