/*
    FIMC2.0  Camera Header File

    Copyright (C) 2003 Samsung Electronics (SW.LEE: hitchcar@samsung.com)

	Author : SW.LEE <hitchcar@samsung.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*
*/


#ifndef __FIMC20_CAMIF_H_
#define __FIMC20_CAMIF_H_

#ifdef __KERNEL__

#include "bits.h"
#include "videodev.h"
#include <asm/types.h>
#include <linux/i2c.h>

#endif  /* __KERNEL__ */

#ifndef O_NONCAP
#define O_NONCAP O_TRUNC
#endif

/* Codec or Preview Status */
#define CAMIF_STARTED         BIT1
#define CAMIF_STOPPED         BIT2
#define CAMIF_INT_HAPPEN      BIT3

/* Codec or Preview  : Interrupt FSM */
#define CAMIF_1nd_INT         BIT7
#define CAMIF_Xnd_INT         BIT8
#define CAMIF_Ynd_INT         BIT9 
#define CAMIF_Znd_INT         BIT10
#define CAMIF_NORMAL_INT      BIT11
#define CAMIF_DUMMY_INT       BIT12
#define CAMIF_PENDING_INT     0


/* CAMIF RESET Definition */
#define CAMIF_RESET           BIT0
#define CAMIF_EX_RESET_AL     BIT1 	/* Active Low */
#define CAMIF_EX_RESET_AH     BIT2	/* Active High */


enum camif_itu_fmt {
	CAMIF_ITU601 = BIT31,
	CAMIF_ITU656 = 0
};

/* It is possbie to use two device simultaneously */
enum camif_dma_type {
       CAMIF_PREVIEW = BIT0,
       CAMIF_CODEC   = BIT1,
};

enum camif_order422 {
       CAMIF_YCBYCR = 0,
       CAMIF_YCRYCB = BIT14,
       CAMIF_CBYCRY = BIT15,
       CAMIF_CRYCBY = BIT14 | BIT15
};

enum flip_mode {
        CAMIF_FLIP   = 0,
        CAMIF_FLIP_X = BIT14,
        CAMIF_FLIP_Y = BIT15,
        CAMIF_FLIP_MIRROR = BIT14 |BIT15,
};

enum camif_codec_fmt {
	/* Codec part */
        CAMIF_IN_YCBCR420  = BIT0, /* Currently IN_YCBCR format fixed */
        CAMIF_IN_YCBCR422  = BIT1,
        CAMIF_OUT_YCBCR420 = BIT4,
        CAMIF_OUT_YCBCR422 = BIT5,
	/* Preview Part */
	CAMIF_RGB16        = BIT2,
	CAMIF_RGB24        = BIT3,
};

enum camif_capturing {
        CAMIF_BOTH_DMA_ON  = BIT4,
	CAMIF_DMA_ON   	   = BIT3,
	CAMIF_BOTH_DMA_OFF = BIT1,
	CAMIF_DMA_OFF      = BIT0,
	/*------------------------*/
	CAMIF_DMA_OFF_L_IRQ= BIT5,
};

typedef struct camif_performance
{
	int	frames;
	int	framesdropped;
	__u64	bytesin;
	__u64	bytesout;
	__u32	reserved[4];
} camif_perf_t;


typedef struct {
        dma_addr_t  phys_y;
        dma_addr_t  phys_cb;
        dma_addr_t  phys_cr;
        u8          *virt_y;  
        u8          *virt_cb;
        u8          *virt_cr;
        dma_addr_t  phys_rgb;
        u8          *virt_rgb;
}img_buf_t;


/* this structure convers the CIWDOFFST, prescaler, mainscaler */
typedef struct {
	u32 modified_src_x;	/* After windows applyed to source_x */
	u32 modified_src_y;
	u32 hfactor;
	u32 vfactor;
	u32 shfactor;     	/* SHfactor = 10 - ( hfactor + vfactor ) */
	u32 prehratio;
	u32 prevratio;
	u32 predst_x;
	u32 predst_y;
	u32 scaleup_h;      	
	u32 scaleup_v;
	u32 mainhratio;
	u32 mainvratio;
	u32 scalerbypass;	/* only codec */
} scaler_t;


enum v4l2_status {
        CAMIF_V4L2_INIT    = BIT0,
        CAMIF_v4L2_DIRTY   = BIT1,
};


/* Global Status Definition */
#define PWANT2START   BIT0
#define CWANT2START   BIT1
#define BOTH_STARTED  (PWANT2START|CWANT2START)
#define PNOTWORKING   BIT4
#define C_WORKING     BIT5

typedef struct {
	struct semaphore         lock;
        enum camif_itu_fmt       itu_fmt;
        enum camif_order422      order422;
	u32                      win_hor_ofst;
	u32                      win_ver_ofst;
        u32                      camclk;  /* External Image Sensor Camera Clock */
        u32                      source_x;
        u32                      source_y;
	u32                      polarity_pclk;
	u32                      polarity_vsync;
	u32                      polarity_href;
	struct i2c_client        *sensor;
	u32                      user; /* MAX 2 (codec, preview) */
	u32    			 old_priority; /* BUS PRIORITY register */
	u32                      status;
	u32                      init_sensor;/* initializing sensor */
	void	  		 *other;    /* Codec camif_cfg_t */
	u32			 reset_type;	/* External Sensor Reset  Type */
	u32		         reset_udelay;	
} camif_gc_t;			/* gobal control register */


/* when  App want to change v4l2 parameter,
 * we instantly store it into v4l2_t v2 
 * and then reflect it to hardware
 */	
typedef struct v4l2 {
	struct v4l2_fmtdesc     *fmtdesc;
	struct v4l2_pix_format   fmt; /* current pixel format */
	struct v4l2_input        input;
        struct video_picture     picture;
	enum v4l2_status         status;
	int     used_fmt ;	/* used format index  */
} v4l2_t;


typedef struct camif_c_t {
        struct video_device      *v;
	/* V4L2 param only for v4l2 driver */
	v4l2_t                    v2;
	camif_gc_t               *gc; /* Common between Codec and Preview */
	/* logical parameter */
	wait_queue_head_t        waitq;
	u32                      status; /* Start/Stop */
	u32                      fsm; /* Start/Stop */
	u32                      open_count; /* duplicated */
	int                      irq;
	char                     shortname[16];
        u32                      target_x;
        u32                      target_y;
	scaler_t                 sc;
	enum flip_mode           flip;
        enum camif_dma_type      dma_type;
       /* 4 pingpong Frame memory */
        u8                       *pp_virt_buf;
        dma_addr_t               pp_phys_buf;
        u32                      pp_totalsize;
	u32                      pp_num; /* used pingpong memory number */
        img_buf_t                img_buf[4];
        enum camif_codec_fmt     fmt;
        enum camif_capturing     exec;
	camif_perf_t             perf;
	u32			 now_frame_num;
	u32                      auto_restart; /* Only For Preview */
} camif_cfg_t;

#ifdef SW_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif


#ifdef SW_DEBUG
#define assert(expr) \
        if(!(expr)) {                                   \
        printk( "Assertion failed! %s,%s,%s,line=%d\n", \
        #expr,__FILE__,__FUNCTION__,__LINE__);          \
        }
#else
#define assert(expr)
#endif



extern int camif_capture_start(camif_cfg_t *);
extern int camif_capture_stop(camif_cfg_t *);
extern int camif_g_frame_num(camif_cfg_t *);
extern u8 * camif_g_frame(camif_cfg_t *);
extern int  camif_win_offset(camif_gc_t *);
extern void camif_hw_open(camif_gc_t *);
extern void camif_hw_close(camif_cfg_t *);
extern int camif_dynamic_open(camif_cfg_t *);
extern int camif_dynamic_close(camif_cfg_t *);
extern void camif_reset(int,int);
extern void camif_setup_sensor(void);
extern int camif_g_fifo_status(camif_cfg_t *);
extern void camif_last_irq_en(camif_cfg_t *);
extern void camif_change_flip(camif_cfg_t *);


/* Todo
 *  API Interface function to both Character and V4L2 Drivers
 */
extern int camif_do_write(struct file *,const char *, size_t, loff_t *);
extern int camif_do_ioctl(struct inode *, struct file *,unsigned int, void *);


/* 
 * API for Decoder (S5x532, OV7620..) 
 */
void camif_register_decoder(struct i2c_client *);
void camif_unregister_decoder(struct i2c_client*);



/* API for FSM */
#define INSTANT_SKIP 0 
#define INSTANT_GO   1 

extern ssize_t camif_p_1fsm_start(camif_cfg_t *);
extern ssize_t camif_p_2fsm_start(camif_cfg_t *);
extern ssize_t camif_4fsm_start(camif_cfg_t *);
extern ssize_t camif_p_stop(camif_cfg_t *);
extern int camif_enter_p_4fsm(camif_cfg_t *);
extern int camif_enter_c_4fsm(camif_cfg_t *);
extern int camif_enter_2fsm(camif_cfg_t *);
extern int camif_enter_1fsm(camif_cfg_t *);
extern int camif_check_preview(camif_cfg_t *);
extern int camif_callback_start(camif_cfg_t *);
extern int camif_clock_init(camif_gc_t *);

/* 
 *  V4L2 Part
 */
#define VID_HARDWARE_SAMSUNG_FIMC20   236





#endif


/* 
 * Local variables:
 * tab-width: 8
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  c-set-style: "K&R"
 * End:
 */
