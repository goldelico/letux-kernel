/*
 *   Copyright (C) 2004 Samsung Electronics 
 *       SW.LEE <hitchcar@samsung.com>
 *   
 * This file is subject to the terms and conditions of the GNU General Public
 * License 2. See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#ifdef CONFIG_ARCH_S3C24A0A
#include <asm/arch/S3C24A0.h>
#include <asm/arch/clocks.h>
#else
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/regs-irq.h>
#endif

#include "cam_reg.h"
//#define SW_DEBUG
#define CONFIG_VIDEO_V4L1_COMPAT
#include <linux/videodev.h>
#include "camif.h"
#include "miscdevice.h"

static int camif_dma_burst(camif_cfg_t *);
static int camif_scaler(camif_cfg_t *);

/* For SXGA Image */
#define RESERVE_MEM  15*1024*1024
#define YUV_MEM      10*1024*1024
#define RGB_MEM      (RESERVE_MEM - YUV_MEM)

static int camif_malloc(camif_cfg_t *cfg)
{
	unsigned int t_size;
	unsigned int daon = cfg->target_x *cfg->target_y;

	if(cfg->dma_type & CAMIF_CODEC) {
		if (cfg->fmt & CAMIF_OUT_YCBCR420) {
			t_size = daon * 3 / 2 ;
		}
		else  { t_size = daon * 2; /* CAMIF_OUT_YCBCR422 */ }
		t_size = t_size *cfg->pp_num;

#ifndef SAMSUNG_SXGA_CAM
		cfg->pp_virt_buf = dma_alloc_coherent(cfg->v->dev,
						      t_size, &cfg->pp_phys_buf,
						      GFP_KERNEL);
#else
		printk(KERN_INFO "Reserving High RAM Addresses \n");
		cfg->pp_phys_buf = PHYS_OFFSET + (MEM_SIZE - RESERVE_MEM);
		cfg->pp_virt_buf = ioremap_nocache(cfg->pp_phys_buf, YUV_MEM);
#endif

		if ( !cfg->pp_virt_buf ) {
			printk(KERN_ERR"CAMERA:Failed to request YCBCR MEM\n");
			return -ENOMEM;
		}
		memset(cfg->pp_virt_buf, 0, t_size);
		cfg->pp_totalsize = t_size;
		return 0;
	}
	if ( cfg->dma_type & CAMIF_PREVIEW ) {
		if (cfg->fmt & CAMIF_RGB16) 
			t_size = daon * 2; /*  4byte per two pixel*/
		else {
			assert(cfg->fmt & CAMIF_RGB24);
			t_size = daon * 4; /* 4byte per one pixel */
		}
		t_size = t_size * cfg->pp_num;
#ifndef SAMSUNG_SXGA_CAM
		cfg->pp_virt_buf = dma_alloc_coherent(cfg->v->dev,
						      t_size, &cfg->pp_phys_buf,
						      GFP_KERNEL);
#else
		printk(KERN_INFO "Reserving High RAM Addresses \n");
		cfg->pp_phys_buf = PHYS_OFFSET + (MEM_SIZE - RESERVE_MEM ) + YUV_MEM;
		cfg->pp_virt_buf = ioremap_nocache(cfg->pp_phys_buf,RGB_MEM);
#endif
		if ( !cfg->pp_virt_buf ) { 
			printk(KERN_ERR"CAMERA:Failed to request RGB MEM\n");
			return -ENOMEM;
		}
		memset(cfg->pp_virt_buf, 0, t_size);
		cfg->pp_totalsize = t_size;
		return 0;
	}

	return 0;		/* Never come. */
}

static int camif_demalloc(camif_cfg_t *cfg)
{
#ifndef SAMSUNG_SXGA_CAM
	if ( cfg->pp_virt_buf ) {
		dma_free_coherent(cfg->v->dev, cfg->pp_totalsize,
				  cfg->pp_virt_buf, cfg->pp_phys_buf);
		cfg->pp_virt_buf = 0;
	}
#else
	iounmap(cfg->pp_virt_buf);
	cfg->pp_virt_buf = 0;
#endif
	return 0;
}

/* 
 * advise a person to use this func in ISR 
 * index value indicates the next frame count to be used 
 */
int camif_g_frame_num(camif_cfg_t *cfg)
{
	int index = 0;

	if (cfg->dma_type & CAMIF_CODEC ) {
		index = FRAME_CNT(readl(camregs + S3C2440_CAM_REG_CICOSTATUS));
		DPRINTK("CAMIF_CODEC frame %d \n", index);
	}
	else {
		assert(cfg->dma_type & CAMIF_PREVIEW );
		index = FRAME_CNT(readl(camregs + S3C2440_CAM_REG_CIPRSTATUS));
		DPRINTK("CAMIF_PREVIEW frame %d  0x%08X \n", index,
			readl(camregs + S3C2440_CAM_REG_CIPRSTATUS));
	}
	cfg->now_frame_num = (index + 2) % 4; /* When 4 PingPong */
	return index; /* meaningless */
}

static int camif_pp_codec(camif_cfg_t *cfg)
{
	u32 i, c_size; /* Cb,Cr size */
	u32 one_p_size;
	u32 daon = cfg->target_x * cfg->target_y;
	if (cfg->fmt & CAMIF_OUT_YCBCR420)
		c_size = daon / 4;
	else {
		assert(cfg->fmt & CAMIF_OUT_YCBCR422);
		c_size = daon / 2;
	}
	switch ( cfg->pp_num ) {
	case 1 :
		for (i =0 ; i < 4; i++) {
			cfg->img_buf[i].virt_y = cfg->pp_virt_buf;
			cfg->img_buf[i].phys_y = cfg->pp_phys_buf;
			cfg->img_buf[i].virt_cb = cfg->pp_virt_buf + daon;
			cfg->img_buf[i].phys_cb = cfg->pp_phys_buf + daon;
			cfg->img_buf[i].virt_cr = cfg->pp_virt_buf + daon + c_size;
			cfg->img_buf[i].phys_cr = cfg->pp_phys_buf + daon + c_size;
			writel(cfg->img_buf[i].phys_y, camregs + S3C2440_CAM_REG_CICOYSA(i));
			writel(cfg->img_buf[i].phys_cb, camregs + S3C2440_CAM_REG_CICOCBSA(i));
			writel(cfg->img_buf[i].phys_cr, camregs + S3C2440_CAM_REG_CICOCRSA(i));
		}
		break;
	case 2:
#define  TRY   (( i%2 ) ? 1 :0)
		one_p_size = daon + 2*c_size;
		for (i = 0; i < 4  ; i++) {
			cfg->img_buf[i].virt_y = cfg->pp_virt_buf + TRY * one_p_size;
			cfg->img_buf[i].phys_y = cfg->pp_phys_buf + TRY * one_p_size;
			cfg->img_buf[i].virt_cb = cfg->pp_virt_buf + daon + TRY * one_p_size;
			cfg->img_buf[i].phys_cb = cfg->pp_phys_buf + daon + TRY * one_p_size;
			cfg->img_buf[i].virt_cr = cfg->pp_virt_buf + daon + c_size + TRY * one_p_size;
			cfg->img_buf[i].phys_cr = cfg->pp_phys_buf + daon + c_size + TRY * one_p_size;
			writel(cfg->img_buf[i].phys_y, camregs + S3C2440_CAM_REG_CICOYSA(i));
			writel(cfg->img_buf[i].phys_cb, camregs + S3C2440_CAM_REG_CICOCBSA(i));
			writel(cfg->img_buf[i].phys_cr, camregs + S3C2440_CAM_REG_CICOCRSA(i));
		}
		break;
	case 4:
		one_p_size = daon + 2*c_size;
		for (i = 0; i < 4 ; i++) {
			cfg->img_buf[i].virt_y = cfg->pp_virt_buf + i * one_p_size;
			cfg->img_buf[i].phys_y = cfg->pp_phys_buf + i * one_p_size;
			cfg->img_buf[i].virt_cb = cfg->pp_virt_buf + daon + i * one_p_size;
			cfg->img_buf[i].phys_cb = cfg->pp_phys_buf + daon + i * one_p_size;
			cfg->img_buf[i].virt_cr = cfg->pp_virt_buf + daon + c_size + i * one_p_size;
			cfg->img_buf[i].phys_cr = cfg->pp_phys_buf + daon + c_size + i * one_p_size;
			writel(cfg->img_buf[i].phys_y, camregs + S3C2440_CAM_REG_CICOYSA(i));
			writel(cfg->img_buf[i].phys_cb, camregs + S3C2440_CAM_REG_CICOCBSA(i));
			writel(cfg->img_buf[i].phys_cr, camregs + S3C2440_CAM_REG_CICOCRSA(i));
		}
		break;
	default:
		printk("Invalid PingPong Number %d \n",cfg->pp_num);
		panic("halt\n");
}
	return 0;
}

/* RGB Buffer Allocation */
static int camif_pp_preview(camif_cfg_t *cfg)
{
	int i;
	u32 daon = cfg->target_x * cfg->target_y;

	if(cfg->fmt & CAMIF_RGB24)  
		daon = daon * 4 ;
	else {
		assert (cfg->fmt & CAMIF_RGB16);
		daon = daon *2;
	}  
	switch ( cfg->pp_num ) {
		case 1:
			for ( i = 0; i < 4 ; i++ ) {
				cfg->img_buf[i].virt_rgb = cfg->pp_virt_buf ;
				cfg->img_buf[i].phys_rgb = cfg->pp_phys_buf ;
				writel(cfg->img_buf[i].phys_rgb, camregs + S3C2440_CAM_REG_CICOCRSA(i));
			}
			break;
		case 2:
			for ( i = 0; i < 4 ; i++) {
				cfg->img_buf[i].virt_rgb = cfg->pp_virt_buf + TRY * daon;
				cfg->img_buf[i].phys_rgb = cfg->pp_phys_buf + TRY * daon;
				writel(cfg->img_buf[i].phys_rgb, camregs + S3C2440_CAM_REG_CICOCRSA(i));
			}
			break;
		case 4:
			for ( i = 0; i < 4 ; i++) {
				cfg->img_buf[i].virt_rgb = cfg->pp_virt_buf + i * daon;
				cfg->img_buf[i].phys_rgb = cfg->pp_phys_buf + i * daon;
				writel(cfg->img_buf[i].phys_rgb, camregs + S3C2440_CAM_REG_CICOCRSA(i));
			}
			break;
		default:
			printk("Invalid PingPong Number %d \n",cfg->pp_num);
			panic("halt\n");
	}
	return 0;
}

static int camif_pingpong(camif_cfg_t *cfg)
{
	if (cfg->dma_type & CAMIF_CODEC ) {
		camif_pp_codec(cfg);
	}

	if ( cfg->dma_type & CAMIF_PREVIEW) {
		camif_pp_preview(cfg);
	}
	return 0;
}


/***********       Image Convert *******************************/
/*  Return Format 
 *  Supported by Hardware
 *  V4L2_PIX_FMT_YUV420,
 *  V4L2_PIX_FMT_YUV422P,
 *  V4L2_PIX_FMT_BGR32 (BGR4)
 * -----------------------------------
 *  V4L2_PIX_FMT_RGB565(X) 
 *  Currenly 2byte --> BGR656 Format
 *  S3C2440A,S3C24A0 supports vairants with reversed FMT_RGB565 
    i.e  blue toward the least, red towards the most significant bit 
    --  by SW.LEE
 */


/* 
 * After calling camif_g_frame_num,
 * this func must be called 
 */
u8 * camif_g_frame(camif_cfg_t *cfg)
{
	u8 * ret = NULL;
	int cnt = cfg->now_frame_num;

	if(cfg->dma_type & CAMIF_PREVIEW) {
		ret = cfg->img_buf[cnt].virt_rgb;
	}
	if (cfg->dma_type & CAMIF_CODEC) {
		ret = cfg->img_buf[cnt].virt_y;
	}
	return ret;
}

/* This function must be called in module initial time */
static int camif_source_fmt(camif_gc_t *gc) 
{
	u32 cmd = 0;

	/* Configure CISRCFMT --Source Format */
	if (gc->itu_fmt & CAMIF_ITU601) {
		cmd = CAMIF_ITU601;
	}
	else {
		assert ( gc->itu_fmt & CAMIF_ITU656);
		cmd = CAMIF_ITU656;
	}
	cmd  |= SOURCE_HSIZE(gc->source_x)| SOURCE_VSIZE(gc->source_y);
	/* Order422 */
	cmd |=  gc->order422;
	writel(cmd, camregs + S3C2440_CAM_REG_CISRCFMT);

	return 0 ;
}


/* 
 * Codec Input YCBCR422 will be Fixed 
 */
static int camif_target_fmt(camif_cfg_t *cfg)
{
	u32 cmd = 0;

	if (cfg->dma_type & CAMIF_CODEC) {
		/* YCBCR setting */  
		cmd = TARGET_HSIZE(cfg->target_x)| TARGET_VSIZE(cfg->target_y);
		if ( cfg->fmt & CAMIF_OUT_YCBCR420 ) {
			cmd |= OUT_YCBCR420|IN_YCBCR422;
		}
		else { 
			assert(cfg->fmt & CAMIF_OUT_YCBCR422);
			cmd |= OUT_YCBCR422|IN_YCBCR422;
		}
		writel(cmd | cfg->flip, camregs + S3C2440_CAM_REG_CICOTRGFMT);

	} else {
		assert(cfg->dma_type & CAMIF_PREVIEW);
		writel(TARGET_HSIZE(cfg->target_x)|TARGET_VSIZE(cfg->target_y)|cfg->flip,
			camregs + S3C2440_CAM_REG_CIPRTRGFMT); 
	}
	return 0;
}

void camif_change_flip(camif_cfg_t *cfg)
{
	u32 cmd = readl(camregs + S3C2440_CAM_REG_CICOTRGFMT);

	cmd &= ~(BIT14|BIT15);
	cmd |= cfg->flip;

	writel(cmd, camregs + S3C2440_CAM_REG_CICOTRGFMT);
}



/* Must:
 * Before calling this function,
 * you must use "camif_dynamic_open"
 * If you want to enable both CODEC and preview
 *  you must do it at the same time.
 */
int camif_capture_start(camif_cfg_t *cfg)
{
	u32 n_cmd = 0;		/* Next Command */

	switch(cfg->exec) {
	case CAMIF_BOTH_DMA_ON:
		camif_reset(CAMIF_RESET, 0); /* Flush Camera Core Buffer */
		writel(readl(camregs + S3C2440_CAM_REG_CIPRSCCTRL) |
			     SCALERSTART, camregs + S3C2440_CAM_REG_CIPRSCCTRL);
		writel(readl(camregs + S3C2440_CAM_REG_CICOSCCTRL) |
			     SCALERSTART, camregs + S3C2440_CAM_REG_CICOSCCTRL);
		n_cmd = CAMIF_CAP_PREVIEW_ON | CAMIF_CAP_CODEC_ON;
		break;
	case CAMIF_DMA_ON:
		camif_reset(CAMIF_RESET, 0); /* Flush Camera Core Buffer */
		if (cfg->dma_type&CAMIF_CODEC) {
			writel(readl(camregs + S3C2440_CAM_REG_CICOSCCTRL) |
			     SCALERSTART, camregs + S3C2440_CAM_REG_CICOSCCTRL);
			n_cmd = CAMIF_CAP_CODEC_ON;
		} else {
			writel(readl(camregs + S3C2440_CAM_REG_CIPRSCCTRL) |
			     SCALERSTART, camregs + S3C2440_CAM_REG_CIPRSCCTRL);
			n_cmd = CAMIF_CAP_PREVIEW_ON;
		}

		/* wait until Sync Time expires */
		/* First settting, to wait VSYNC fall  */
		/* By VESA spec,in 640x480 @60Hz
			MAX Delay Time is around 64us which "while" has.*/
		while(VSYNC & readl(camregs + S3C2440_CAM_REG_CICOSTATUS));
		break;
	default:
		break;
}
	writel(n_cmd | CAMIF_CAP_ON, camregs + S3C2440_CAM_REG_CIIMGCPT); 
	return 0;
}


int camif_capture_stop(camif_cfg_t *cfg)
{
	u32 n_cmd = readl(camregs + S3C2440_CAM_REG_CIIMGCPT);	/* Next Command */

	switch(cfg->exec) {
	case CAMIF_BOTH_DMA_OFF:
		writel(readl(camregs + S3C2440_CAM_REG_CIPRSCCTRL) &
			~SCALERSTART, camregs + S3C2440_CAM_REG_CIPRSCCTRL);
		writel(readl(camregs + S3C2440_CAM_REG_CICOSCCTRL) &
			~SCALERSTART, camregs + S3C2440_CAM_REG_CICOSCCTRL);
		n_cmd = 0;
		break;
	case CAMIF_DMA_OFF_L_IRQ: /* fall thru */
	case CAMIF_DMA_OFF:
		if (cfg->dma_type&CAMIF_CODEC) {
			writel(readl(camregs + S3C2440_CAM_REG_CICOSCCTRL) &
			    ~SCALERSTART, camregs + S3C2440_CAM_REG_CICOSCCTRL);
			n_cmd &= ~CAMIF_CAP_CODEC_ON;
			if (!(n_cmd & CAMIF_CAP_PREVIEW_ON))
				n_cmd = 0;
		} else {
			writel(readl(camregs + S3C2440_CAM_REG_CIPRSCCTRL) &
			    ~SCALERSTART, camregs + S3C2440_CAM_REG_CIPRSCCTRL);
			n_cmd &= ~CAMIF_CAP_PREVIEW_ON;
			if (!(n_cmd & CAMIF_CAP_CODEC_ON))
				n_cmd = 0;
		}
		break;
	default:
		panic("Unexpected \n");
	}
	writel(n_cmd, camregs + S3C2440_CAM_REG_CIIMGCPT);

	if (cfg->exec == CAMIF_DMA_OFF_L_IRQ) { /* Last IRQ  */
		if (cfg->dma_type & CAMIF_CODEC)
			writel(readl(camregs + S3C2440_CAM_REG_CICOCTRL) |
			     LAST_IRQ_EN, camregs + S3C2440_CAM_REG_CICOCTRL);
		else
			writel(readl(camregs + S3C2440_CAM_REG_CIPRCTRL) |
			     LAST_IRQ_EN, camregs + S3C2440_CAM_REG_CIPRCTRL);
	}
#if 0
	else {				/* to make internal state machine of CAMERA stop */
		camif_reset(CAMIF_RESET, 0);
	}
#endif
	return 0;
}


/* LastIRQEn is autoclear */
void camif_last_irq_en(camif_cfg_t *cfg)
{
	if ((cfg->exec == CAMIF_BOTH_DMA_ON) || (cfg->dma_type & CAMIF_CODEC))
		writel(readl(camregs + S3C2440_CAM_REG_CICOCTRL) |
			LAST_IRQ_EN, camregs + S3C2440_CAM_REG_CICOCTRL);

	if ((cfg->exec == CAMIF_BOTH_DMA_ON) || !(cfg->dma_type & CAMIF_CODEC))
		writel(readl(camregs + S3C2440_CAM_REG_CIPRCTRL) |
			LAST_IRQ_EN, camregs + S3C2440_CAM_REG_CIPRCTRL);
}

static int  
camif_scaler_internal(u32 srcWidth, u32 dstWidth, u32 *ratio, u32 *shift)
{
	if(srcWidth>=64*dstWidth){
		printk(KERN_ERR"CAMERA:out of prescaler range: srcWidth /dstWidth = %d(< 64)\n",
			 srcWidth/dstWidth);
		return 1;
	}
	else if(srcWidth>=32*dstWidth){
		*ratio=32;
		*shift=5;
	}
	else if(srcWidth>=16*dstWidth){
		*ratio=16;
		*shift=4;
	}
	else if(srcWidth>=8*dstWidth){
		*ratio=8;
		*shift=3;
	}
	else if(srcWidth>=4*dstWidth){
		*ratio=4;
		*shift=2;
	}
	else if(srcWidth>=2*dstWidth){
		*ratio=2;
		*shift=1;
	}
	else {
		*ratio=1;
		*shift=0;
	}  
	return 0;
}


int camif_g_fifo_status(camif_cfg_t *cfg) 
{
	u32 reg;

	if (cfg->dma_type & CAMIF_CODEC) {
		u32 flag = CO_OVERFLOW_Y | CO_OVERFLOW_CB | CO_OVERFLOW_CR;
		reg = readl(camregs + S3C2440_CAM_REG_CICOSTATUS);
		if (reg & flag) {
			printk("CODEC: FIFO error(0x%08x) and corrected\n",reg);
			/* FIFO Error Count ++  */
			writel(readl(camregs + S3C2440_CAM_REG_CIWDOFST) |
				CO_FIFO_Y | CO_FIFO_CB | CO_FIFO_CR,
				camregs + S3C2440_CAM_REG_CIWDOFST);

			writel(readl(camregs + S3C2440_CAM_REG_CIWDOFST) &
				~(CO_FIFO_Y | CO_FIFO_CB | CO_FIFO_CR),
				camregs + S3C2440_CAM_REG_CIWDOFST);
			return 1; /* Error */
		}
	}
	if (cfg->dma_type & CAMIF_PREVIEW) {
		u32 flag = PR_OVERFLOW_CB | PR_OVERFLOW_CR;
		reg = readl(camregs + S3C2440_CAM_REG_CIPRSTATUS);
		if (reg & flag) {
			printk("PREVIEW:FIFO error(0x%08x) and corrected\n",reg);
			writel(readl(camregs + S3C2440_CAM_REG_CIWDOFST) |
				CO_FIFO_CB | CO_FIFO_CR,
				camregs + S3C2440_CAM_REG_CIWDOFST);

			writel(readl(camregs + S3C2440_CAM_REG_CIWDOFST) &
				~(CO_FIFO_Y | CO_FIFO_CB | CO_FIFO_CR),
				camregs + S3C2440_CAM_REG_CIWDOFST);
			/* FIFO Error Count ++  */
			return 1; /* Error */
		}
	}
	return 0;		/* No Error */
}


/* Policy:
 * if codec or preview define the win offset,
 *    other must follow that value.
 */
int camif_win_offset(camif_gc_t *gc )
{
	u32 h = gc->win_hor_ofst;
	u32 v = gc->win_ver_ofst;

	/*Clear Overflow */
	writel(CO_FIFO_Y | CO_FIFO_CB | CO_FIFO_CR | PR_FIFO_CB | PR_FIFO_CB,
		camregs + S3C2440_CAM_REG_CIWDOFST);
	writel(0, camregs + S3C2440_CAM_REG_CIWDOFST);

	if (!h && !v)	{
		writel(0, camregs + S3C2440_CAM_REG_CIWDOFST);
		return 0;
	}

	writel(WINOFEN | WINHOROFST(h) | WINVEROFST(v), camregs + S3C2440_CAM_REG_CIWDOFST);
	return 0;
}

/*  
 * when you change the resolution in a specific camera,
 * sometimes, it is necessary to change the polarity 
 *                                       -- SW.LEE
 */
static void camif_polarity(camif_gc_t *gc)
{
	u32 cmd = readl(camregs + S3C2440_CAM_REG_CIGCTRL);;
	
	cmd = cmd & ~(BIT26|BIT25|BIT24); /* clear polarity */
	if (gc->polarity_pclk)
		cmd |= GC_INVPOLPCLK;
	if (gc->polarity_vsync)
		cmd |= GC_INVPOLVSYNC;
	if (gc->polarity_href)
		cmd |= GC_INVPOLHREF;
	writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) |
	       cmd, camregs + S3C2440_CAM_REG_CIGCTRL);
}


int camif_dynamic_open(camif_cfg_t *cfg)
{
	camif_win_offset(cfg->gc);
	camif_polarity(cfg->gc);
	
	if(camif_scaler(cfg)) {
		printk(KERN_ERR "CAMERA:Preview Scaler, Change WinHorOfset or Target Size\n");
		return 1;
	}
	camif_target_fmt(cfg);
	if (camif_dma_burst(cfg)) {
		printk(KERN_ERR "CAMERA:DMA Busrt Length Error \n");
		return 1;
	}
	if(camif_malloc(cfg) ) {
		printk(KERN_ERR "    Instead of using consistent_alloc()\n"
			        "    lease use dedicated memory allocation for DMA memory\n");
		return -1;
	}
	camif_pingpong(cfg);
	return 0;
}

int camif_dynamic_close(camif_cfg_t *cfg)
{
	camif_demalloc(cfg);
	return 0;
}

static int camif_target_area(camif_cfg_t *cfg) 
{
	u32 rect = cfg->target_x * cfg->target_y;

	if (cfg->dma_type & CAMIF_CODEC)
		writel(rect, camregs + S3C2440_CAM_REG_CICOTAREA);

	if (cfg->dma_type & CAMIF_PREVIEW)
		writel(rect, camregs + S3C2440_CAM_REG_CIPRTAREA);

	return 0;
}

static int inline camif_hw_reg(camif_cfg_t *cfg)
{
	u32 cmd = 0;

	if (cfg->dma_type & CAMIF_CODEC) {
		writel(PRE_SHIFT(cfg->sc.shfactor) |
		       PRE_HRATIO(cfg->sc.prehratio) |
		       PRE_VRATIO(cfg->sc.prevratio),
		       camregs + S3C2440_CAM_REG_CICOSCPRERATIO);
		writel(PRE_DST_WIDTH(cfg->sc.predst_x) |
		       PRE_DST_HEIGHT(cfg->sc.predst_y),
		       camregs + S3C2440_CAM_REG_CICOSCPREDST);

		/* Differ from Preview */
		if (cfg->sc.scalerbypass)
			cmd |= SCALERBYPASS;
		if (cfg->sc.scaleup_h & cfg->sc.scaleup_v)
			cmd |= BIT30|BIT29;
		writel(cmd | MAIN_HRATIO(cfg->sc.mainhratio) |
		       MAIN_VRATIO(cfg->sc.mainvratio),
		       camregs + S3C2440_CAM_REG_CICOSCCTRL);
		return 0;
	}
	if (cfg->dma_type & CAMIF_PREVIEW) {
		writel(PRE_SHIFT(cfg->sc.shfactor) |
		       PRE_HRATIO(cfg->sc.prehratio) |
		       PRE_VRATIO(cfg->sc.prevratio),
		       camregs + S3C2440_CAM_REG_CIPRSCPRERATIO);
		writel(PRE_DST_WIDTH(cfg->sc.predst_x) |
		       PRE_DST_HEIGHT(cfg->sc.predst_y),
		       camregs + S3C2440_CAM_REG_CIPRSCPREDST);
		/* Differ from Codec */
		if (cfg->fmt & CAMIF_RGB24)
			cmd |= RGB_FMT24;  
		if (cfg->sc.scaleup_h & cfg->sc.scaleup_v)
			cmd |= BIT29 | BIT28;
		writel(cmd | MAIN_HRATIO(cfg->sc.mainhratio) | S_METHOD |
		       MAIN_VRATIO(cfg->sc.mainvratio),
		       camregs + S3C2440_CAM_REG_CIPRSCCTRL);
		return 0;
	}

	panic("CAMERA:DMA_TYPE Wrong \n");
	return 0;
}


/* Configure Pre-scaler control  & main scaler control register */
static int camif_scaler(camif_cfg_t *cfg)
{
	int tx = cfg->target_x, ty = cfg->target_y;
	int sx, sy;

	if (tx <= 0 || ty <= 0)
		panic("CAMERA: Invalid target size \n");

	sx = cfg->gc->source_x - 2 * cfg->gc->win_hor_ofst;
	sy = cfg->gc->source_y - 2 * cfg->gc->win_ver_ofst;	
	if (sx <= 0 || sy <= 0)
		panic("CAMERA: Invalid source size \n");

	cfg->sc.modified_src_x = sx;
	cfg->sc.modified_src_y = sy;

	/* Pre-scaler control register 1 */
	camif_scaler_internal(sx, tx, &cfg->sc.prehratio, &cfg->sc.hfactor);
	camif_scaler_internal(sy, ty, &cfg->sc.prevratio, &cfg->sc.vfactor);

	if (cfg->dma_type & CAMIF_PREVIEW)
		if ((sx / cfg->sc.prehratio) > 640) {
			printk(KERN_INFO "CAMERA: Internal Preview line "
					 "buffer is 640 pixels\n");
			return 1; /* Error */
		}

	cfg->sc.shfactor = 10 - (cfg->sc.hfactor + cfg->sc.vfactor);
	/* Pre-scaler control register 2 */
	cfg->sc.predst_x = sx / cfg->sc.prehratio;
	cfg->sc.predst_y = sy / cfg->sc.prevratio;

	/* Main-scaler control register */
	cfg->sc.mainhratio = (sx << 8) / (tx << cfg->sc.hfactor);
	cfg->sc.mainvratio = (sy << 8) / (ty << cfg->sc.vfactor);
	DPRINTK(" sx %d, sy %d tx %d ty %d  \n", sx, sy, tx, ty);
	DPRINTK(" hfactor %d  vfactor %d \n",cfg->sc.hfactor, cfg->sc.vfactor);

	cfg->sc.scaleup_h  = (sx <= tx) ? 1: 0;
	cfg->sc.scaleup_v  = (sy <= ty) ? 1: 0;
	if (cfg->sc.scaleup_h != cfg->sc.scaleup_v)
		printk(KERN_ERR "scaleup_h must be same to scaleup_v \n");

	camif_hw_reg(cfg);
	camif_target_area(cfg);

	return 0;
}

/******************************************************
 CalculateBurstSize - Calculate the busrt lengths
 Description:
 - dstHSize: the number of the byte of H Size.
********************************************************/
static void camif_g_bsize(u32 hsize, u32 *mburst, u32 *rburst)
{
	u32 tmp;

	tmp = (hsize / 4) % 16;
	switch(tmp) {
	case 0:
		*mburst=16;
		*rburst=16;
		break;
	case 4:
		*mburst=16;
		*rburst=4;
		break;
	case 8:
		*mburst=16;
		*rburst=8;
		break;
	default:
		tmp=(hsize / 4) % 8;
		switch(tmp) {
		case 0:
			*mburst = 8;
			*rburst = 8;
			break;
		case 4:
			*mburst = 8;
			*rburst = 4;
		default:
			*mburst = 4;
			tmp = (hsize / 4) % 4;
			*rburst= (tmp) ? tmp: 4;
			break;
	}
		break;
	}
}

/* SXGA 1028x1024*/
/* XGA 1024x768 */
/* SVGA 800x600 */
/* VGA 640x480 */
/* CIF 352x288 */
/* QVGA 320x240 */
/* QCIF 176x144 */
/* ret val 
        1 : DMA Size Error 
*/
#define BURST_ERR 1 
static int camif_dma_burst(camif_cfg_t *cfg)
{
	int width = cfg->target_x;

	if (cfg->dma_type & CAMIF_CODEC ) {
		u32 yburst_m, yburst_r;
		u32 cburst_m, cburst_r;
		/* CODEC DMA WIDHT is multiple of 16 */
		if (width % 16)
			return BURST_ERR;   /* DMA Burst Length Error */
		camif_g_bsize(width, &yburst_m, &yburst_r);
		camif_g_bsize(width / 2, &cburst_m, &cburst_r);

		writel(YBURST_M(yburst_m) | CBURST_M(cburst_m) |
		       YBURST_R(yburst_r) | CBURST_R(cburst_r),
		       camregs + S3C2440_CAM_REG_CICOCTRL);
	}

	if (cfg->dma_type & CAMIF_PREVIEW) {
		u32 rgburst_m, rgburst_r;
		if(cfg->fmt == CAMIF_RGB24) {
			if (width % 2)
				return BURST_ERR;   /* DMA Burst Length Error */
			camif_g_bsize(width*4,&rgburst_m,&rgburst_r);
		} else {		/* CAMIF_RGB16 */
			if ((width / 2) %2)
				return BURST_ERR; /* DMA Burst Length Error */
			camif_g_bsize(width*2,&rgburst_m,&rgburst_r);  
		}

		writel(RGBURST_M(rgburst_m) | RGBURST_R(rgburst_r),
		       camregs + S3C2440_CAM_REG_CIPRCTRL);
	}
	return 0;
}

static int camif_gpio_init(void)
{
#ifdef CONFIG_ARCH_S3C24A0A
	/* S3C24A0A has the dedicated signal pins for Camera */
#else
	s3c2410_gpio_cfgpin(S3C2440_GPJ0, S3C2440_GPJ0_CAMDATA0);
	s3c2410_gpio_cfgpin(S3C2440_GPJ1, S3C2440_GPJ1_CAMDATA1);
	s3c2410_gpio_cfgpin(S3C2440_GPJ2, S3C2440_GPJ2_CAMDATA2);
	s3c2410_gpio_cfgpin(S3C2440_GPJ3, S3C2440_GPJ3_CAMDATA3);
	s3c2410_gpio_cfgpin(S3C2440_GPJ4, S3C2440_GPJ4_CAMDATA4);
	s3c2410_gpio_cfgpin(S3C2440_GPJ5, S3C2440_GPJ5_CAMDATA5);
	s3c2410_gpio_cfgpin(S3C2440_GPJ6, S3C2440_GPJ6_CAMDATA6);
	s3c2410_gpio_cfgpin(S3C2440_GPJ7, S3C2440_GPJ7_CAMDATA7);

	s3c2410_gpio_cfgpin(S3C2440_GPJ8, S3C2440_GPJ8_CAMPCLK);
	s3c2410_gpio_cfgpin(S3C2440_GPJ9, S3C2440_GPJ9_CAMVSYNC);
	s3c2410_gpio_cfgpin(S3C2440_GPJ10, S3C2440_GPJ10_CAMHREF);
	s3c2410_gpio_cfgpin(S3C2440_GPJ11, S3C2440_GPJ11_CAMCLKOUT);
	s3c2410_gpio_cfgpin(S3C2440_GPJ12, S3C2440_GPJ12_CAMRESET);
#endif
        return 0;
}


#define ROUND_ADD 0x100000

#ifdef CONFIG_ARCH_S3C24A0A
int camif_clock_init(camif_gc_t *gc)
{
	unsigned int upll, camclk_div, camclk;

	if (!gc) camclk = 24000000;
	else  {
		camclk = gc->camclk;
		if (camclk > 48000000)
			printk(KERN_ERR "Wrong Camera Clock\n");
	}

	CLKCON |= CLKCON_CAM_UPLL | CLKCON_CAM_HCLK;
	upll = get_bus_clk(GET_UPLL);
	printk(KERN_INFO "CAMERA:Default UPLL %08d and Assing 96Mhz to UPLL\n",upll);	
	UPLLCON = FInsrt(56, fPLL_MDIV) | FInsrt(2, fPLL_PDIV)| FInsrt(1, fPLL_SDIV);
	upll = get_bus_clk(GET_UPLL);

	camclk_div = (upll+ROUND_ADD) / camclk - 1;
	CLKDIVN = (CLKDIVN & 0xFF) | CLKDIVN_CAM(camclk_div);
	printk(KERN_INFO"CAMERA:upll %d MACRO 0x%08X CLKDIVN 0x%08X \n",
				upll, CLKDIVN_CAM(camclk_div), CLKDIVN);
	writel(0, camregs + S3C2440_CAM_REG_CIIMGCPT);	/* Dummy ? */

	return 0;
}
#else
int camif_clock_init(camif_gc_t *gc)
{
	unsigned int camclk;
	struct clk *clk_camif = clk_get(NULL, "camif");
	struct clk *clk_camif_upll = clk_get(NULL, "camif-upll");

	if (!gc)
		camclk = 24000000;
	else {
		camclk = gc->camclk;
		if (camclk > 48000000)
			printk(KERN_ERR "Wrong Camera Clock\n");
	}

	clk_set_rate(clk_camif, camclk);

	clk_enable(clk_camif);
	clk_enable(clk_camif_upll);
	

#if 0
	CLKCON |= CLKCON_CAMIF;
	upll = elfin_get_bus_clk(GET_UPLL);
	printk(KERN_INFO "CAMERA:Default UPLL %08d and Assing 96Mhz to UPLL\n",upll);	
	{
		UPLLCON = FInsrt(60, fPLL_MDIV) | FInsrt(4, fPLL_PDIV)| FInsrt(1, fPLL_SDIV);
		CLKDIVN |= DIVN_UPLL;	/* For USB */
		upll = elfin_get_bus_clk(GET_UPLL);
	}

	camclk_div = (upll+ROUND_ADD) /(camclk * 2) -1;
	CAMDIVN = CAMCLK_SET_DIV|(camclk_div&0xf);
	printk(KERN_INFO "CAMERA:upll %08d  cam_clk %08d CAMDIVN 0x%08x \n",upll,camclk, CAMDIVN);
#endif
	writel(0, camregs + S3C2440_CAM_REG_CIIMGCPT);	/* Dummy ? */

	return 0;
}
#endif

/* 
   Reset Camera IP in CPU
   Reset External Sensor 
 */
void camif_reset(int is, int delay)
{
	switch (is) {
		case CAMIF_RESET:
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) |
			       GC_SWRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			mdelay(1);
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) &
			       ~GC_SWRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			break;
		case CAMIF_EX_RESET_AH: /*Active High */
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) &
			       ~GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			udelay(200);
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) |
			       GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			udelay(delay);
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) &
			       ~GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			break;
		case CAMIF_EX_RESET_AL:	/*Active Low */
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) |
			       GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			udelay(200);
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) &
			       ~GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			udelay(delay);
			writel(readl(camregs + S3C2440_CAM_REG_CIGCTRL) |
			       GC_CAMRST,
			       camregs + S3C2440_CAM_REG_CIGCTRL);
			break;
		default:
			break;
	}
}
		
/* For Camera Operation,
 * we can give the high priority to REQ2 of ARBITER1 
 */

/* Please move me into proper place 
 *  camif_gc_t is not because "rmmod imgsenor" will delete the instance of camif_gc_t  
 */
static u32 old_priority; 

static void camif_bus_priority(int flag)
{
	if (flag) {
#ifdef CONFIG_ARCH_S3C24A0A
		old_priority = PRIORITY0;
		PRIORITY0 = PRIORITY_I_FIX;
		PRIORITY1 = PRIORITY_I_FIX;

#else
		old_priority = readl(S3C2410_PRIORITY);
		writel(readl(S3C2410_PRIORITY) & ~(3<<7), S3C2410_PRIORITY);
		writel(readl(S3C2410_PRIORITY) |  (1<<7), S3C2410_PRIORITY); /* Arbiter 1, REQ2 first */
		writel(readl(S3C2410_PRIORITY) & ~(1<<1), S3C2410_PRIORITY); /* Disable Priority Rotate */
#endif
	} 
	else {
#ifdef CONFIG_ARCH_S3C24A0A
		PRIORITY0 = old_priority;
		PRIORITY1 = old_priority;
#else
		writel(old_priority, S3C2410_PRIORITY);
#endif
	}
}

static void inline camif_clock_off(void)
{
#if defined (CONFIG_ARCH_S3C24A0A)
	writel(0, camregs + S3C2440_CAM_REG_CIIMGCPT);

	CLKCON &= ~CLKCON_CAM_UPLL;
	CLKCON &= ~CLKCON_CAM_HCLK;
#else
	struct clk *clk_camif = clk_get(NULL, "camif");
	struct clk *clk_camif_upll = clk_get(NULL, "camif-upll");

	writel(0, camregs + S3C2440_CAM_REG_CIIMGCPT);

	clk_disable(clk_camif);
	clk_disable(clk_camif_upll);
#endif
}


/* Init external image sensor 
 *  Before make some value into image senor,
 *  you must set up the pixel clock.
 */
void camif_setup_sensor(void)
{
	camif_reset(CAMIF_RESET, 0);
	camif_gpio_init();
	camif_clock_init(NULL);
/* Sometimes ,Before loading I2C module, we need the reset signal */
#ifdef CONFIG_ARCH_S3C24A0A
	camif_reset(CAMIF_EX_RESET_AL,1000);
#else
	camif_reset(CAMIF_EX_RESET_AH,1000);
#endif
}

void camif_hw_close(camif_cfg_t *cfg)
{
	camif_bus_priority(0);
	camif_clock_off();
}

void camif_hw_open(camif_gc_t *gc)
{
	camif_source_fmt(gc);
	camif_win_offset(gc);
	camif_bus_priority(1);
}



/* 
 * Local variables:
 * tab-width: 8
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  c-set-style: "K&R"
 * End:
 */
