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
#include <linux/pagemap.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <linux/miscdevice.h>

#define CONFIG_VIDEO_V4L1_COMPAT
#include <linux/videodev.h>
#include "camif.h"

//#define SW_DEBUG 
static void camif_start_p_with_c(camif_cfg_t *cfg);

#include "camif.h"
const char *fsm_version =
        "$Id: camif_fsm.c,v 1.3 2004/04/27 10:26:28 swlee Exp $";


/* 
 * FSM function is the place where Synchronization in not necessary
 * because IRS calls this functions.
 */

ssize_t camif_p_1fsm_start(camif_cfg_t *cfg)
{
	//camif_reset(CAMIF_RESET,0);
	cfg->exec = CAMIF_DMA_ON;
	camif_capture_start(cfg);
	camif_last_irq_en(cfg);
	cfg->status = CAMIF_STARTED;
	cfg->fsm = CAMIF_1nd_INT;
	return 0;
}


ssize_t camif_p_2fsm_start(camif_cfg_t *cfg)
{
	camif_reset(CAMIF_RESET,0);/* FIFO Count goes to zero */
	cfg->exec = CAMIF_DMA_ON;
	camif_capture_start(cfg);
	cfg->status = CAMIF_STARTED;
	cfg->fsm = CAMIF_1nd_INT;
	return 0;
}


ssize_t camif_4fsm_start(camif_cfg_t *cfg)
{
	camif_reset(CAMIF_RESET,0); /* FIFO Count goes to zero */
	cfg->exec = CAMIF_DMA_ON;
	camif_capture_start(cfg);
	cfg->status = CAMIF_STARTED;
	cfg->fsm = CAMIF_1nd_INT;
	cfg->perf.frames = 0;
	return 0;
}


/* Policy:  
     cfg->perf.frames  set in camif_fsm.c
     cfg->status set in video-driver.c
 */

/* 
 * Don't insert camif_reset(CAM_RESET, 0 ) into this func 
 */
ssize_t camif_p_stop(camif_cfg_t *cfg)
{
	cfg->exec = CAMIF_DMA_OFF;
//	cfg->status = CAMIF_STOPPED;
	camif_capture_stop(cfg);
	cfg->perf.frames = 0;	/* Dupplicated ? */
	return 0;
}

/* When C working, P asks C to play togehter */
/* Only P must call this function */
void camif_start_c_with_p (camif_cfg_t *cfg, camif_cfg_t *other)
{
//	cfg->gc->other = get_camif(CODEC_MINOR);
	cfg->gc->other = other;
	camif_start_p_with_c(cfg);
}

static void camif_start_p_with_c(camif_cfg_t *cfg)
{
	camif_cfg_t *other = (camif_cfg_t *)cfg->gc->other;
	/* Preview Stop */
	cfg->exec = CAMIF_DMA_OFF;
	camif_capture_stop(cfg);
	/* Start P and C */
	camif_reset(CAMIF_RESET, 0);
	cfg->exec =CAMIF_BOTH_DMA_ON;
	camif_capture_start(cfg);
	cfg->fsm = CAMIF_1nd_INT; /* For Preview */
	if(!other) panic("Unexpected Error \n");
	other->fsm = CAMIF_1nd_INT; /* For Preview */
}

static void camif_auto_restart(camif_cfg_t *cfg)
{	
//	if (cfg->dma_type & CAMIF_CODEC) return;
	if (cfg->auto_restart)
		camif_start_p_with_c(cfg);
}


/* Supposed that PREVIEW already running 
 * request PREVIEW to start with Codec 
 */
static int camif_check_global(camif_cfg_t *cfg)
{
	int ret = 0;
 	 
        if (down_interruptible(&cfg->gc->lock)) 
                       return -ERESTARTSYS;
        if ( cfg->gc->status & CWANT2START ) {
		cfg->gc->status &= ~CWANT2START;
		cfg->auto_restart = 1;
		ret = 1;
	}
	else {
	        ret = 0; /* There is no codec */
		cfg->auto_restart = 0; /* Duplicated ..Dummy */
	}

	up(&cfg->gc->lock);

        return ret;
}

/*
 *    1nd INT : Start Interrupt
 *    Xnd INT : enable Last IRQ : pingpong get the valid data
 *    Ynd INT : Stop Codec or Preview : pingpong get the valid data
 *    Znd INT : Last IRQ : valid data 
 */
#define CHECK_FREQ 5
int camif_enter_p_4fsm(camif_cfg_t *cfg)
{
	int ret = 0;

	cfg->perf.frames++;
	if (cfg->fsm == CAMIF_NORMAL_INT)
	if (cfg->perf.frames % CHECK_FREQ == 0) 
		ret = camif_check_global(cfg);
	if (ret > 0) cfg->fsm = CAMIF_Xnd_INT; /* Codec wait for Preview */
	
	switch (cfg->fsm) {
		case CAMIF_1nd_INT:           /* Start IRQ */
			cfg->fsm = CAMIF_NORMAL_INT;
			ret = INSTANT_SKIP;
			DPRINTK(KERN_INFO "1nd INT \n");	
			break;
		case CAMIF_NORMAL_INT:		
			cfg->status = CAMIF_INT_HAPPEN;
			cfg->fsm = CAMIF_NORMAL_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "NORMAL INT \n");	
			break;
		case CAMIF_Xnd_INT:		
			camif_last_irq_en(cfg);/* IRQ for Enabling LAST IRQ */
			cfg->status = CAMIF_INT_HAPPEN;
			cfg->fsm = CAMIF_Ynd_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "Xnd INT \n");	
			break;	
		case CAMIF_Ynd_INT:        /* Capture Stop */
			cfg->exec = CAMIF_DMA_OFF;
			cfg->status = CAMIF_INT_HAPPEN;
			camif_capture_stop(cfg);
			cfg->fsm = CAMIF_Znd_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "Ynd INT \n");	
			break;
		case CAMIF_Znd_INT: 		/*  LAST IRQ (Dummy IRQ */
			cfg->fsm = CAMIF_DUMMY_INT;
			cfg->status = CAMIF_INT_HAPPEN;
			ret = INSTANT_GO;
			camif_auto_restart(cfg);  /* Automatically Restart Camera */
			DPRINTK(KERN_INFO "Znd INT \n");
			break;	
	        case CAMIF_DUMMY_INT:
			cfg->status = CAMIF_STOPPED; /* Dupplicate ? */
			ret = INSTANT_SKIP;
//			DPRINTK(KERN_INFO "Dummy INT \n"); 
			break;
		default:
			printk(KERN_INFO "Unexpect INT %d \n",cfg->fsm); 
			ret = INSTANT_SKIP;
			break;
	}
	return ret;
}


/* 
 * NO autorestart included in this function 
 */
int camif_enter_c_4fsm(camif_cfg_t *cfg)
{
	int ret;
	
	cfg->perf.frames++;
#if 0
	if (   (cfg->fsm==CAMIF_NORMAL_INT)
	    && (cfg->perf.frames>cfg->restart_limit-1)
	   ) 
		cfg->fsm = CAMIF_Xnd_INT;
#endif
	switch (cfg->fsm) {
		case CAMIF_1nd_INT:           /* Start IRQ */
			cfg->fsm = CAMIF_NORMAL_INT;
//			cfg->status = CAMIF_STARTED; /* need this to meet auto-restart */
			ret = INSTANT_SKIP;
			DPRINTK(KERN_INFO "1nd INT \n");
			break;
		case CAMIF_NORMAL_INT:		
			cfg->status = CAMIF_INT_HAPPEN;
			cfg->fsm = CAMIF_NORMAL_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "NORMALd INT \n");
			break;
		case CAMIF_Xnd_INT:		
			camif_last_irq_en(cfg);/* IRQ for Enabling LAST IRQ */
			cfg->status = CAMIF_INT_HAPPEN;
			cfg->fsm = CAMIF_Ynd_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "Xnd INT \n");
			break;	
		case CAMIF_Ynd_INT:        /* Capture Stop */
			cfg->exec = CAMIF_DMA_OFF;
			cfg->status = CAMIF_INT_HAPPEN;
			camif_capture_stop(cfg);
			cfg->fsm = CAMIF_Znd_INT;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "Ynd INT \n");
			break;
		case CAMIF_Znd_INT: 		/*  LAST IRQ (Dummy IRQ */
			cfg->fsm = CAMIF_DUMMY_INT;
			cfg->status = CAMIF_INT_HAPPEN;
			ret = INSTANT_GO;
			DPRINTK(KERN_INFO "Znd INT \n");
			break;	
	        case CAMIF_DUMMY_INT:
			cfg->status = CAMIF_STOPPED; /* Dupplicate ? */
			ret = INSTANT_SKIP;
			break;
		default:
			printk(KERN_INFO "Unexpect INT %d \n",cfg->fsm); 
			ret = INSTANT_SKIP;
			break;
	}
	return ret;
}

/* 4 Interrups State Machine is for two pingpong  
 *    1nd INT : Start Interrupt
 *    Xnd INT : enable Last IRQ : pingpong get the valid data
 *    Ynd INT : Stop Codec or Preview : pingpong get the valid data
 *    Znd INT : Last IRQ : valid data 
 *    
 * Note:
 *    Before calling this func, you must call camif_reset
 */

int camif_enter_2fsm(camif_cfg_t *cfg) /* Codec FSM */
{
	int ret;

	cfg->perf.frames++;
	switch (cfg->fsm) {
		case CAMIF_1nd_INT:           /* Start IRQ */
			cfg->fsm = CAMIF_Xnd_INT;
			ret = INSTANT_SKIP;
//			printk(KERN_INFO "1nd INT \n");	
			break;
		case CAMIF_Xnd_INT:		
			camif_last_irq_en(cfg);/* IRQ for Enabling LAST IRQ */
			cfg->now_frame_num = 0;
			cfg->status = CAMIF_INT_HAPPEN;
			cfg->fsm = CAMIF_Ynd_INT;
			ret = INSTANT_GO;
//			printk(KERN_INFO "2nd INT \n");	
			break;	
		case CAMIF_Ynd_INT:        /* Capture Stop */
			cfg->exec = CAMIF_DMA_OFF;
			cfg->now_frame_num = 1;
			cfg->status = CAMIF_INT_HAPPEN;
			camif_capture_stop(cfg);
			cfg->fsm = CAMIF_Znd_INT;
			ret = INSTANT_GO;
//			printk(KERN_INFO "Ynd INT \n");	
			break;
		case CAMIF_Znd_INT: 		/*  LAST IRQ (Dummy IRQ */
			cfg->now_frame_num = 0;
//			cfg->fsm = CAMIF_DUMMY_INT;
			cfg->status = CAMIF_INT_HAPPEN;
			ret = INSTANT_GO;
//			printk(KERN_INFO "Znd INT \n");
			break;	
	        case CAMIF_DUMMY_INT:
			cfg->status = CAMIF_STOPPED; /* Dupplicate ? */
			ret = INSTANT_SKIP;
			printk(KERN_INFO "Dummy INT \n"); 
			break;
		default:  /* CAMIF_PENDING_INT */
			printk(KERN_INFO "Unexpect INT \n"); 
			ret = INSTANT_SKIP;
			break;
	}
	return ret;
}


/* 2 Interrups State Machine is for one pingpong  
 *    1nd INT : Stop Codec or Preview : pingpong get the valid data
 *    2nd INT : Last IRQ : dummy data 
 */
int camif_enter_1fsm(camif_cfg_t *cfg) /* Codec FSM */
{
	int ret;

	cfg->perf.frames++;
	switch (cfg->fsm) {
		case CAMIF_Ynd_INT:		/* IRQ for Enabling LAST IRQ */
			cfg->exec = CAMIF_DMA_OFF;
			camif_capture_stop(cfg);
			cfg->fsm = CAMIF_Znd_INT;
			ret = INSTANT_SKIP;
	//		printk(KERN_INFO "Ynd INT \n");	
			break;	
		case CAMIF_Znd_INT: 		/*  LAST IRQ (Dummy IRQ */
			cfg->fsm = CAMIF_DUMMY_INT;
			cfg->status = CAMIF_INT_HAPPEN;
			ret = INSTANT_GO;
	//		printk(KERN_INFO "Znd INT \n");	
			break;
	        case CAMIF_DUMMY_INT:
			cfg->status = CAMIF_STOPPED; /* Dupplicate ? */
			ret = INSTANT_SKIP;
			printk(KERN_INFO "Dummy INT \n"); 
			break;
		default:
			printk(KERN_INFO "Unexpect INT \n"); 
			ret = INSTANT_SKIP;
			break;
	}
	return ret;
}


/*
 *  GLOBAL STATUS CONTROL FUNCTION
 *
 */


/* Supposed that PREVIEW already running 
 * request PREVIEW to start with Codec 
 */
int camif_callback_start(camif_cfg_t *cfg)
{
        int  doit = 1;
        while (doit) {
                if (down_interruptible(&cfg->gc->lock)) {
                        return -ERESTARTSYS;
                }
                cfg->gc->status = CWANT2START;
                cfg->gc->other = cfg;
                up(&cfg->gc->lock);
                doit = 0;
        }
        return 0;
}

/*
 * Return status of Preview Machine
  ret value :
      0: Preview is not working
      X: Codec must follow PREVIEW start
*/
int camif_check_preview(camif_cfg_t *cfg)
{
        int ret = 0;

        if (down_interruptible(&cfg->gc->lock)) {
                        ret = -ERESTARTSYS;
                        return ret;
        }
        if (cfg->gc->user == 1) ret = 0;
     //   else if (cfg->gc->status & PNOTWORKING) ret = 0;
        else ret = 1;
        up(&cfg->gc->lock);
        return ret;
}




/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
