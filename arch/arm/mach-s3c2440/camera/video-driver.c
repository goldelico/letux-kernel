/*  
    Copyright (C) 2004 Samsung Electronics 
                     SW.LEE <hitchcar@sec.samsung.com>
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <linux/miscdevice.h>

//#define SW_DEBUG 

#include "camif.h"
#include "videodev.h"
#include "miscdevice.h"
#include "cam_reg.h"
#include "sensor.h"
#include "userapp.h"

#ifdef Z_API
#include "qt.h"
#endif

/* Codec and Preview */
#define CAMIF_NUM  2
static camif_cfg_t  fimc[CAMIF_NUM];

static const char *driver_version = 
	"$Id: video-driver.c,v 1.9 2004/06/02 03:10:36 swlee Exp $";
extern const char *fimc_version;
extern const char *fsm_version;


camif_cfg_t * get_camif(int nr)
{
	camif_cfg_t *ret = NULL;
	switch(nr) {
	case CODEC_MINOR:
		ret = &fimc[0];
		break;
	case PREVIEW_MINOR:
		ret = &fimc[1];
		break;
	default:
		panic("Unknow Minor Number \n");
	}
	return ret;
}


static int camif_codec_start(camif_cfg_t *cfg)
{
	int  ret = 0;
	ret =camif_check_preview(cfg);
	switch(ret) {
	case 0:			/* Play alone */
		DPRINTK("Start Alone \n");
		camif_4fsm_start(cfg);
		cfg->gc->status |= C_WORKING;
		break;
	case -ERESTARTSYS:	 /* Busy , retry */
		//DPRINTK("Error \n");
		printk("Error \n");
		break; 
	case 1:
		DPRINTK("need callback \n");
		ret = camif_callback_start(cfg);
		if(ret < 0 ) {
			printk(KERN_INFO "Busy RESTART \n");
			return ret; /* Busy, retry */
		}
		break;
	}
	return ret;
}


ssize_t camif_write (struct file *f, const char *b, size_t c,loff_t *offset)
{
	camif_cfg_t *cfg;

	c = 0;			/* return value */
	DPRINTK("\n");
	cfg = get_camif(MINOR(f->f_dentry->d_inode->i_rdev));
	switch (*b) {
		case 'O':
			if (cfg->dma_type & CAMIF_PREVIEW) {
				if (cfg->gc->status & C_WORKING) {
					camif_start_c_with_p(cfg,get_camif(CODEC_MINOR));
				}	
				else  {
					camif_4fsm_start(cfg);
				}
			}
			else{
				c = camif_codec_start(cfg);
				if(c < 0) c = 1; /* Error and neet to retry */
			}

			break;
		case 'X':
			camif_p_stop(cfg);
			break;
		default:
			panic("CAMERA:camif_write: Unexpected Param\n");
	}
	DPRINTK("end\n");

	return c;
}


ssize_t camif_p_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	camif_cfg_t *cfg = NULL;
	size_t end;

	cfg = get_camif(MINOR(file->f_dentry->d_inode->i_rdev));
	cfg->status = CAMIF_STARTED;

	if (wait_event_interruptible(cfg->waitq,cfg->status == CAMIF_INT_HAPPEN))
		return -ERESTARTSYS;

	cfg->status = CAMIF_STOPPED;
	end = min_t(size_t, cfg->pp_totalsize /cfg->pp_num, count);
	if (copy_to_user(buf, camif_g_frame(cfg), end))
		return -EFAULT;

	return end;
}


static ssize_t 
camif_c_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	camif_cfg_t *cfg = NULL;
	size_t end;

	/* cfg = file->private_data; */
	cfg = get_camif(MINOR(file->f_dentry->d_inode->i_rdev));
#if 0
	if(file->f_flags & O_NONBLOCK) {
		printk(KERN_ERR"Don't Support NON_BLOCK \n");
	}
#endif		

      /* Change the below wait_event_interruptible func */
       if (wait_event_interruptible(cfg->waitq,cfg->status == CAMIF_INT_HAPPEN))
	       return -ERESTARTSYS;
       cfg->status = CAMIF_STOPPED;
       end = min_t(size_t, cfg->pp_totalsize /cfg->pp_num, count);
       if (copy_to_user(buf, camif_g_frame(cfg), end))
	       return -EFAULT;
       return end;
}


static void camif_c_irq(int irq, void *dev_id, struct pt_regs *regs)
{
        camif_cfg_t *cfg = (camif_cfg_t *)dev_id;
	DPRINTK("\n");
	camif_g_fifo_status(cfg);
	camif_g_frame_num(cfg);
	if(camif_enter_c_4fsm(cfg) == INSTANT_SKIP) return;
	wake_up_interruptible(&cfg->waitq);
}

static void camif_p_irq(int irq, void *dev_id, struct pt_regs * regs)
{
        camif_cfg_t *cfg = (camif_cfg_t *)dev_id;
	DPRINTK("\n");
	camif_g_fifo_status(cfg);
	camif_g_frame_num(cfg);
	if(camif_enter_p_4fsm(cfg) == INSTANT_SKIP) return;
	wake_up_interruptible(&cfg->waitq);
#if 0
	if( (cfg->perf.frames % 5) == 0)
		DPRINTK("5\n");
#endif
}

static void camif_release_irq(camif_cfg_t *cfg)
{
	disable_irq(cfg->irq);
	free_irq(cfg->irq, cfg);
}

static int camif_irq_request(camif_cfg_t *cfg)
{
	int ret = 0;

	if (cfg->dma_type & CAMIF_CODEC) {
		if ((ret = request_irq(cfg->irq, camif_c_irq, 
			       SA_INTERRUPT,cfg->shortname, cfg))) {
			printk("request_irq(CAM_C) failed.\n");
		}
	}
	if (cfg->dma_type & CAMIF_PREVIEW) {
		if ((ret = request_irq(cfg->irq, camif_p_irq,
			       SA_INTERRUPT,cfg->shortname, cfg))) {
			printk("request_irq(CAM_P) failed.\n");
		}
	}
	return 0;
}

static void camif_init_sensor(camif_cfg_t *cfg)
{
	camif_gc_t *gc =  cfg->gc;
	if (!gc->sensor) 
		panic("CAMERA:I2C Client(Img Sensor)Not registered\n");
	if(!gc->init_sensor) {
		camif_reset(gc->reset_type, gc->reset_udelay);
		gc->sensor->driver->command(gc->sensor,SENSOR_INIT,NULL);
		gc->init_sensor = 1; /*sensor init done */
	}
	gc->sensor->driver->command(gc->sensor, USER_ADD, NULL);
}

static int camif_open(struct inode *inode, struct file *file)
{
	int err;
	camif_cfg_t * cfg = get_camif(MINOR(inode->i_rdev));

	if(cfg->dma_type & CAMIF_PREVIEW) {
		if(down_interruptible(&cfg->gc->lock))
			return -ERESTARTSYS;
		if (cfg->dma_type & CAMIF_PREVIEW) {
			cfg->gc->status &= ~PNOTWORKING;
		}
		up(&cfg->gc->lock);
	}
	err = video_exclusive_open(inode,file);
	cfg->gc->user++;
	cfg->status = CAMIF_STOPPED;
	if (err < 0)  return err;
	if (file->f_flags & O_NONCAP ) {
		printk("Don't Support Non-capturing open \n");
		return 0;
	}
	file->private_data = cfg;
	camif_irq_request(cfg);
	camif_init_sensor(cfg);
	return 0;
}

#if 0
static void print_pregs(void)
{
	printk(" CISRCFMT 0x%08X  \n", CISRCFMT);
	printk(" CIWDOFST 0x%08X  \n", CIWDOFST);
	printk(" CIGCTRL  0x%08X  \n", CIGCTRL);
	printk(" CIPRTRGFMT 0x%08X  \n", CIPRTRGFMT);
	printk(" CIPRCTRL 0x%08X  \n", CIPRCTRL);
	printk(" CIPRSCPRERATIO 0x%08X  \n", CIPRSCPRERATIO);
	printk(" CIPRSCPREDST 0x%08X  \n", CIPRSCPREDST);
	printk(" CIPRSCCTRL 0x%08X  \n", CIPRSCCTRL);
	printk(" CIPRTAREA 0x%08X  \n", CIPRTAREA);
	printk(" CIPRSTATUS 0x%08X  \n", CIPRSTATUS);
	printk(" CIIMGCPT 0x%08X  \n", CIIMGCPT);
}

static void print_cregs(void)
{
	printk(" CISRCFMT 0x%08X  \n", CISRCFMT);
	printk(" CIWDOFST 0x%08X  \n", CIWDOFST);
	printk(" CIGCTRL  0x%08X  \n", CIGCTRL);
	printk(" CICOCTRL 0x%8X   \n", CICOCTRL);
	printk(" CICOSCPRERATIO 0x%08X  \n", CICOSCPRERATIO);
	printk(" CICOSCPREDST 0x%08X  \n", CICOSCPREDST);
	printk(" CICOSCCTRL 0x%08X  \n", CICOSCCTRL);
	printk(" CICOTAREA 0x%08X  \n", CICOTAREA);
	printk(" CICOSTATUS 0x%8X \n", CICOSTATUS);
	printk(" CIIMGCPT 0x%08X  \n", CIIMGCPT);
}
#endif


static int camif_release(struct inode *inode, struct file *file)
{
	camif_cfg_t * cfg = get_camif(MINOR(inode->i_rdev));

	//DPRINTK(" cfg->status 0x%0X cfg->gc->status 0x%0X \n", cfg->status,cfg->gc->status );
	if (cfg->dma_type & CAMIF_PREVIEW) {
		if(down_interruptible(&cfg->gc->lock))
			return -ERESTARTSYS;
		cfg->gc->status &= ~PWANT2START;
		cfg->gc->status |= PNOTWORKING;
		up(&cfg->gc->lock);
	} 
	else {
		cfg->gc->status &= ~CWANT2START; /* No need semaphore */
	}
	camif_dynamic_close(cfg);
	camif_release_irq(cfg);
	video_exclusive_release(inode,file);
	camif_p_stop(cfg);
	cfg->gc->sensor->driver->command(cfg->gc->sensor, USER_EXIT, NULL);
	cfg->gc->user--;
	cfg->status = CAMIF_STOPPED;
	return 0;
}

static void fimc_config(camif_cfg_t *cfg,u32 x, u32 y, int bpp)
{
	cfg->target_x = x;
	cfg->target_y = y;

	switch (bpp) {
	case 16:
		cfg->fmt = CAMIF_RGB16;
		break;
	case 24:
		cfg->fmt = CAMIF_RGB24;
		break;
	case 420:
		cfg->fmt = CAMIF_IN_YCBCR422|CAMIF_OUT_YCBCR420; 
		break;
	case 422:
		cfg->fmt = CAMIF_IN_YCBCR422|CAMIF_OUT_YCBCR422; 
		break;
	default: 
		panic("Wrong BPP \n");
	}
}


static int 
camif_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	camif_cfg_t *cfg  = file->private_data;
	camif_param_t par;

	switch (cmd) {
		case CMD_CAMERA_INIT:
			if (copy_from_user(&par,(camif_param_t *)arg,
						sizeof(camif_param_t)))
				return -EFAULT;
			fimc_config(cfg,par.dst_x, par.dst_y, par.bpp);
			if (camif_dynamic_open(cfg)) {
				printk(" Eror Happens \n");
				ret = -1;
			}

			switch (par.flip) {
			case 3 : 
				cfg->flip = CAMIF_FLIP_MIRROR;
				break;
			case 1 : 
				cfg->flip = CAMIF_FLIP_X;
				break;
			case 2 : 
				cfg->flip = CAMIF_FLIP_Y;
				break;
			case 0 : 
			default:
				cfg->flip = CAMIF_FLIP;
			}
			break;
	/* Todo
		case CMD_SENSOR_BRIGHTNESS:
			cfg->gc->sensor->driver->command(cfg->gc->sensor, SENSOR_BRIGHTNESS, NULL);
			break;
	*/
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}


#if 0
static int camif_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
//	camif_cfg_t *cfg  = file->private_data;


	switch (cmd) {
/*	case Some_other_action */
	default:
		return video_usercopy(inode, file, cmd, arg, camif_do_ioctl);
	}
}
#endif

static struct file_operations camif_c_fops =
{
	.owner	  = THIS_MODULE,
	.open	  = camif_open,
	.release  = camif_release,
	.ioctl	  = camif_ioctl,
	.read	  = camif_c_read,
	.write 	  = camif_write,
};

static struct file_operations camif_p_fops =
{
	.owner	  = THIS_MODULE,
	.open	  = camif_open,
	.release  = camif_release,
	.ioctl	  = camif_ioctl,
#ifdef Z_API
	.read	  = z_read,
	.write 	  = z_write,
#else
	.read	  = camif_p_read,
	.write 	  = camif_write,
#endif
};

static struct video_device codec_template =
{
	.name     = "CODEC_IF",
	.type     = VID_TYPE_CAPTURE|VID_TYPE_CLIPPING|VID_TYPE_SCALES,
	.hardware = VID_HARDWARE_SAMSUNG_FIMC20,
	.fops     = &camif_c_fops,
//	.release  = camif_release
	.minor    = -1,
};

static struct video_device preview_template =
{
	.name     = "PREVIEW_IF",
	.type     = VID_TYPE_CAPTURE|VID_TYPE_CLIPPING|VID_TYPE_SCALES,
	.hardware = VID_HARDWARE_SAMSUNG_FIMC20,
	.fops     = &camif_p_fops,
	.minor    = -1,
};

static int preview_init(camif_cfg_t *cfg)
{
	char name[16]="CAM_PREVIEW";

	memset(cfg, 0, sizeof(camif_cfg_t));
	cfg->target_x = 640;
	cfg->target_y = 480;
	cfg->pp_num   = 4;
	cfg->dma_type = CAMIF_PREVIEW;
	cfg->fmt      = CAMIF_RGB16;
	cfg->flip     = CAMIF_FLIP_Y;
	cfg->v        = &preview_template;
	init_MUTEX(&cfg->v->lock);
	cfg->irq       = IRQ_CAM_P;
	
	strcpy(cfg->shortname,name);
        init_waitqueue_head(&cfg->waitq);
	cfg->status = CAMIF_STOPPED;
	return cfg->status;
}

static int codec_init(camif_cfg_t *cfg)
{
	char name[16]="CAM_CODEC";

	memset(cfg, 0, sizeof(camif_cfg_t));
	cfg->target_x = 176;
	cfg->target_y = 144;
	cfg->pp_num   = 4; 
	cfg->dma_type = CAMIF_CODEC;
	cfg->fmt      = CAMIF_IN_YCBCR422|CAMIF_OUT_YCBCR420;
	cfg->flip     = CAMIF_FLIP_X;
	cfg->v         = &codec_template;
	init_MUTEX(&cfg->v->lock);
	cfg->irq       = IRQ_CAM_C;
	strcpy(cfg->shortname,name);
	init_waitqueue_head(&cfg->waitq);
	cfg->status = CAMIF_STOPPED;
	return cfg->status;
}

static void camif_init(void)
{
	camif_setup_sensor();
}



static void print_version(void)
{
	printk(KERN_INFO"FIMC built:"__DATE__ " "__TIME__"\n%s\n%s\n%s\n", 
					fimc_version, driver_version,fsm_version);
}


static int camif_m_in(void)
{
	int ret = 0;
 	camif_cfg_t * cfg;

	camif_init();		
	cfg = get_camif(CODEC_MINOR);
	codec_init(cfg);

	if (video_register_device(cfg->v,0,CODEC_MINOR)!=0) {
			DPRINTK("Couldn't register codec driver.\n");
			return 0;
	}
	cfg = get_camif(PREVIEW_MINOR);
	preview_init(cfg);
	if (video_register_device(cfg->v,0,PREVIEW_MINOR)!=0) {
			DPRINTK("Couldn't register preview driver.\n");
			return 0;
	}
	
	print_version();
	return ret;
}

static void unconfig_device(camif_cfg_t *cfg)
{
	video_unregister_device(cfg->v);
	camif_hw_close(cfg);
	//memset(cfg, 0, sizeof(camif_cfg_t));
}

static void camif_m_out(void)	/* module out */
{
	camif_cfg_t *cfg;

	cfg = get_camif(CODEC_MINOR);
	unconfig_device(cfg);
	cfg = get_camif(PREVIEW_MINOR);
	unconfig_device(cfg);
	return;
}

void camif_register_decoder(struct i2c_client *ptr)
{
	camif_cfg_t *cfg;

	cfg =get_camif(CODEC_MINOR);
	cfg->gc = (camif_gc_t *)(ptr->data);

	cfg =get_camif(PREVIEW_MINOR);
	cfg->gc = (camif_gc_t *)(ptr->data);

	sema_init(&cfg->gc->lock,1); /* global lock for both Codec and Preview */
	cfg->gc->status |= PNOTWORKING; /* Default Value */
	camif_hw_open(cfg->gc);
}

void camif_unregister_decoder(struct i2c_client *ptr)
{
	camif_gc_t *gc;
		
	gc = (camif_gc_t *)(ptr->data);	
	gc->init_sensor = 0; /* need to modify */
}

module_init(camif_m_in);
module_exit(camif_m_out);

EXPORT_SYMBOL(camif_register_decoder);
EXPORT_SYMBOL(camif_unregister_decoder);

MODULE_AUTHOR("SW.LEE <hitchcar@sec.samsung.com>");
MODULE_DESCRIPTION("Video-Driver For Fimc2.0 MISC Drivers");
MODULE_LICENSE("GPL");


/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
