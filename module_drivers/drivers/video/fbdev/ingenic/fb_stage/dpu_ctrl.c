#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>

#include "dpu_reg.h"
#include "dpu_ctrl.h"


static void dump_dc_reg(struct dpu_ctrl *dctrl)
{
	printk("-----------------dc_reg------------------\n");
	printk("DC_FRM_CFG_ADDR(0x%04x):    0x%08lx\n", DC_FRM_CFG_ADDR,reg_read(dctrl, DC_FRM_CFG_ADDR));
	printk("DC_FRM_CFG_CTRL(0x%04x):    0x%08lx\n", DC_FRM_CFG_CTRL,reg_read(dctrl, DC_FRM_CFG_CTRL));
	printk("DC_RDMA_CHAIN_ADDR(0x%04x):    0x%08lx\n", DC_RDMA_CHAIN_ADDR,reg_read(dctrl, DC_RDMA_CHAIN_ADDR));
	printk("DC_RDMA_CHAIN_CTRL(0x%04x):    0x%08lx\n", DC_RDMA_CHAIN_CTRL,reg_read(dctrl, DC_RDMA_CHAIN_CTRL));
	printk("DC_CTRL(0x%04x):            0x%08lx\n", DC_CTRL, reg_read(dctrl, DC_CTRL));
	printk("DC_CSC_MULT_YRV(0x%04x):    0x%08lx\n", DC_CSC_MULT_YRV,reg_read(dctrl, DC_CSC_MULT_YRV));
	printk("DC_CSC_MULT_GUGV(0x%04x):   0x%08lx\n", DC_CSC_MULT_GUGV,reg_read(dctrl, DC_CSC_MULT_GUGV));
	printk("DC_CSC_MULT_BU(0x%04x):     0x%08lx\n", DC_CSC_MULT_BU,reg_read(dctrl, DC_CSC_MULT_BU));
	printk("DC_CSC_SUB_YUV(0x%04x):     0x%08lx\n", DC_CSC_SUB_YUV,reg_read(dctrl, DC_CSC_SUB_YUV));
	printk("DC_ST(0x%04x):              0x%08lx\n", DC_ST, reg_read(dctrl, DC_ST));
	printk("DC_INTC(0x%04x):            0x%08lx\n", DC_INTC, reg_read(dctrl, DC_INTC));
	printk("DC_INT_FLAG(0x%04x):	   0x%08lx\n", DC_INT_FLAG, reg_read(dctrl, DC_INT_FLAG));
	printk("DC_COM_CONFIG(0x%04x):      0x%08lx\n", DC_COM_CONFIG ,reg_read(dctrl, DC_COM_CONFIG));
	printk("DC_TLB_GLBC(0x%04x):        0x%08lx\n", DC_TLB_GLBC,reg_read(dctrl, DC_TLB_GLBC));
	printk("DC_TLB_TLBA(0x%04x):        0x%08lx\n", DC_TLB_TLBA,reg_read(dctrl, DC_TLB_TLBA));
	printk("DC_TLB_TLBC(0x%04x):        0x%08lx\n", DC_TLB_TLBC,reg_read(dctrl, DC_TLB_TLBC));
	printk("DC_TLB0_VPN(0x%04x):        0x%08lx\n", DC_TLB0_VPN,reg_read(dctrl, DC_TLB0_VPN));
	printk("DC_TLB1_VPN(0x%04x):        0x%08lx\n", DC_TLB1_VPN,reg_read(dctrl, DC_TLB1_VPN));
	printk("DC_TLB2_VPN(0x%04x):        0x%08lx\n", DC_TLB2_VPN,reg_read(dctrl, DC_TLB2_VPN));
	printk("DC_TLB3_VPN(0x%04x):        0x%08lx\n", DC_TLB3_VPN,reg_read(dctrl, DC_TLB3_VPN));
	printk("DC_TLB_TLBV(0x%04x):        0x%08lx\n", DC_TLB_TLBV,reg_read(dctrl, DC_TLB_TLBV));
	printk("DC_TLB_STAT(0x%04x):        0x%08lx\n", DC_TLB_STAT,reg_read(dctrl, DC_TLB_STAT));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL,reg_read(dctrl, DC_PCFG_RD_CTRL));
	printk("DC_PCFG_WR_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_WR_CTRL,reg_read(dctrl, DC_PCFG_WR_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG,reg_read(dctrl, DC_OFIFO_PCFG));
	printk("DC_WDMA_PCFG(0x%04x):	   0x%08lx\n", DC_WDMA_PCFG,reg_read(dctrl, DC_WDMA_PCFG));
	printk("DC_CMPW_PCFG_CTRL(0x%04x): 0x%08lx\n", DC_CMPW_PCFG_CTRL,reg_read(dctrl, DC_CMPW_PCFG_CTRL));
	printk("DC_CMPW_PCFG0(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG0,reg_read(dctrl, DC_CMPW_PCFG0));
	printk("DC_CMPW_PCFG1(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG1,reg_read(dctrl, DC_CMPW_PCFG1));
	printk("DC_CMPW_PCFG2(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG2,reg_read(dctrl, DC_CMPW_PCFG2));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL,reg_read(dctrl, DC_PCFG_RD_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG,reg_read(dctrl, DC_OFIFO_PCFG));
	printk("DC_DISP_COM(0x%04x):        0x%08lx\n", DC_DISP_COM,reg_read(dctrl, DC_DISP_COM));
	printk("-----------------dc_reg------------------\n");
}

static void dump_tft_reg(struct dpu_ctrl *dctrl)
{
	printk("----------------tft_reg------------------\n");
	printk("TFT_TIMING_HSYNC(0x%04x):   0x%08lx\n", DC_TFT_HSYNC, reg_read(dctrl, DC_TFT_HSYNC));
	printk("TFT_TIMING_VSYNC(0x%04x):   0x%08lx\n", DC_TFT_VSYNC, reg_read(dctrl, DC_TFT_VSYNC));
	printk("TFT_TIMING_HDE(0x%04x):     0x%08lx\n", DC_TFT_HDE, reg_read(dctrl, DC_TFT_HDE));
	printk("TFT_TIMING_VDE(0x%04x):     0x%08lx\n", DC_TFT_VDE, reg_read(dctrl, DC_TFT_VDE));
	printk("TFT_TRAN_CFG(0x%04x):       0x%08lx\n", DC_TFT_CFG, reg_read(dctrl, DC_TFT_CFG));
	printk("TFT_ST(0x%04x):             0x%08lx\n", DC_TFT_ST, reg_read(dctrl, DC_TFT_ST));
	printk("----------------tft_reg------------------\n");
}

static void dump_slcd_reg(struct dpu_ctrl *dctrl)
{
	printk("---------------slcd_reg------------------\n");
	printk("SLCD_CFG(0x%04x):           0x%08lx\n", DC_SLCD_CFG, reg_read(dctrl, DC_SLCD_CFG));
	printk("SLCD_WR_DUTY(0x%04x):       0x%08lx\n", DC_SLCD_WR_DUTY, reg_read(dctrl, DC_SLCD_WR_DUTY));
	printk("SLCD_TIMING(0x%04x):        0x%08lx\n", DC_SLCD_TIMING, reg_read(dctrl, DC_SLCD_TIMING));
	printk("SLCD_FRM_SIZE(0x%04x):      0x%08lx\n", DC_SLCD_FRM_SIZE, reg_read(dctrl, DC_SLCD_FRM_SIZE));
	printk("SLCD_SLOW_TIME(0x%04x):     0x%08lx\n", DC_SLCD_SLOW_TIME, reg_read(dctrl, DC_SLCD_SLOW_TIME));
	printk("SLCD_REG_IF(0x%04x):	    0x%08lx\n", DC_SLCD_REG_IF, reg_read(dctrl, DC_SLCD_REG_IF));
	printk("SLCD_ST(0x%04x):            0x%08lx\n", DC_SLCD_ST, reg_read(dctrl, DC_SLCD_ST));
	printk("---------------slcd_reg------------------\n");
}

static void dump_frm_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(dctrl, DC_CTRL, ctrl);

	printk("--------Frame Descriptor register--------\n");
	printk("FrameNextCfgAddr:   %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("FrameSize:          %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("FrameCtrl:          %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("WritebackAddr:      %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("WritebackStride:    %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("Layer0CfgAddr:      %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("Layer1CfgAddr:      %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("Layer2CfgAddr:      %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("Layer3CfgAddr:      %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("LayCfgEn:	    %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("InterruptControl:   %lx\n",reg_read(dctrl, DC_FRM_DES));
	printk("--------Frame Descriptor register--------\n");
}

static void dump_layer_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(dctrl, DC_CTRL, ctrl);

	printk("--------layer0 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerCfg:           %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerScale:         %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerRotation:      %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerScratch:       %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerPos:           %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("LayerStride:        %lx\n",reg_read(dctrl, DC_LAY0_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(dctrl, DC_LAY0_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("--------layer0 Descriptor register-------\n");

	printk("--------layer1 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerCfg:           %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerScale:         %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerRotation:      %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerScratch:       %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerPos:           %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("LayerStride:        %lx\n",reg_read(dctrl, DC_LAY1_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(dctrl, DC_LAY1_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("--------layer1 Descriptor register-------\n");

	printk("--------layer2 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerCfg:           %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerScale:         %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerRotation:      %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerScratch:       %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerPos:           %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("LayerStride:        %lx\n",reg_read(dctrl, DC_LAY2_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(dctrl, DC_LAY2_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("--------layer2 Descriptor register-------\n");

	printk("--------layer3 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerCfg:           %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerScale:         %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerRotation:      %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerScratch:       %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerPos:           %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("LayerStride:        %lx\n",reg_read(dctrl, DC_LAY3_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(dctrl, DC_LAY3_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("--------layer3 Descriptor register-------\n");
}

static void dump_rdma_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= (1 << 2);
	reg_write(dctrl, DC_CTRL, ctrl);
	printk("====================rdma Descriptor register======================\n");
	printk("RdmaNextCfgAddr:    %lx\n",reg_read(dctrl, DC_RDMA_DES));
	printk("FrameBufferAddr:    %lx\n",reg_read(dctrl, DC_RDMA_DES));
	printk("Stride:             %lx\n",reg_read(dctrl, DC_RDMA_DES));
	printk("ChainCfg:           %lx\n",reg_read(dctrl, DC_RDMA_DES));
	printk("InterruptControl:   %lx\n",reg_read(dctrl, DC_RDMA_DES));
	printk("==================rdma Descriptor register end======================\n");
}

void dump_lcdc_registers(struct dpu_ctrl *dctrl)
{
	dump_dc_reg(dctrl);
	dump_tft_reg(dctrl);
	dump_slcd_reg(dctrl);
	dump_frm_desc_reg(dctrl);
	dump_layer_desc_reg(dctrl);
	dump_rdma_desc_reg(dctrl);
}
static irqreturn_t dpu_ctrl_irq_handler(int irq, void *data)
{
	unsigned int irq_flag;
	struct dpu_ctrl *dctrl = (struct dpu_ctrl *)data;

	spin_lock(&dctrl->irq_lock);

	dbg_irqcnt_inc(dctrl->dbg_irqcnt, irqcnt);

	irq_flag = reg_read(dctrl, DC_INT_FLAG);
	if(likely(irq_flag & DC_CMP_START)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_START);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, cmp_start);
		//TODO:
		//ingenicfb_set_vsync_value(dctrl);
		//dctrl->set_vsync_value(dctrl->vsync_data);
		dctrl->comp_hw_framedesc = reg_read(dctrl, DC_FRM_CHAIN_SITE);
		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
#define IRQ_P_N 50

	if(unlikely(irq_flag & DC_STOP_CMP_ACK)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_STOP_CMP_ACK);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, stop_disp_ack);

		dctrl->comp_stopped = 1;
		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	if(unlikely(irq_flag & DC_DISP_END)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_DISP_END);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, disp_end);
		//dev_dbg(dctrl->dev, "DC_DISP_END");
		//dctrl->dsi->master_ops->query_te(dctrl->dsi);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}


	if(unlikely(irq_flag & DC_TFT_UNDR)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_TFT_UNDR);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, tft_under);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	if(likely(irq_flag & DC_WDMA_OVER)) {
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, wdma_over);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_WDMA_OVER);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_WDMA_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, wdma_end);
		dctrl->wback_end = 1;
		reg_write(dctrl, DC_CLR_ST, DC_CLR_WDMA_END);
		if(cnt < IRQ_P_N)
			dev_dbg(dctrl->dev, "DC_WDMA_END irq came here!!!!!!!!!!!!!!");

		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_LAY3_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer3_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY3_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_LAY3_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_LAY2_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer2_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY2_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_LAY2_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_LAY1_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer1_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY1_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_LAY1_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_LAY0_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer0_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY0_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_LAY0_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_CMP_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, clr_cmp_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_CMP_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_STOP_SRD_ACK)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, stop_wrbk_ack);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_STOP_SRD_ACK);
		dctrl->srd_stopped = 1;
		wake_up_interruptible(&dctrl->wq);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_STOP_SRD irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_SRD_START)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, srd_start);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_SRD_START);
		if(dctrl->rdma_desc_changing) {
			dctrl->rdma_desc_changing = 0;
			/*SRD_START
			 *
			 * 1. descriptor is done loading, it's save to  update rdma_last_frame descriptor.
			 *
			 * 	[rdma_last_frame] --     [current_refresh_frame]
			 * 		^	   |
			 *		|----------|
			 * */
			dctrl->sreadesc[dctrl->rdma_last_frame]->RdmaNextCfgAddr = dctrl->sreadesc_phys[dctrl->rdma_last_frame];
			dctrl->rdma_last_frame = dctrl->rdma_changing_frame;
		}
		dctrl->set_vsync_value(dctrl->vsync_data);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_SRD_START irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_SRD_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, srd_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_SRD_END);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_SRD_END irq came here!!!!!!!!!!!!!!");
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if(likely(irq_flag & DC_CMP_W_SLOW)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, cmp_w_slow);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_W_SLOW);
		if(cnt < IRQ_P_N)
		dev_dbg(dctrl->dev, "DC_CMP_W_SLOW came here!!!!!!!!!!!!!! DC_ST = 0x%lx cnt = %d",reg_read(dctrl,DC_ST),cnt);
		if(cnt > 10)
		reg_write(dctrl, DC_CMPW_PCFG_CTRL, 0 << 10 | 30);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	dev_err(dctrl->dev, "DPU irq nothing do, please check!!! DC_ST = 0x%lx\n",reg_read(dctrl,DC_ST));
	spin_unlock(&dctrl->irq_lock);
	return IRQ_HANDLED;
}

static void dctrl_tlb_invalidate_ch(struct dpu_ctrl *dctrl , int ch)
{
	int val;

	val = ch & 0xF;
	/*Invalid for ch layers.*/
	reg_write(dctrl, DC_TLB_TLBC, val);
}

static void dctrl_tlb_enable(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	unsigned int glbc, val;
	int i;

	lay_cfg = frm_cfg->lay_cfg;

	val = reg_read(dctrl, DC_TLB_GLBC);
	glbc = val;

	for(i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if(lay_cfg[i].lay_en) {
			if(lay_cfg[i].tlb_en != (glbc>>i & 0x1)) {
				glbc = lay_cfg[i].tlb_en ?
					(glbc | (0x1 << i)) : (glbc & ~(0x1 << i));
			}
		} else {
			glbc &= ~(0x1 << i);
		}
	}
	if(val != glbc) {
		dctrl_tlb_invalidate_ch(dctrl, glbc);	/*invalidate tlb*/
		if(val > glbc) {			/*When a TLB channel is disable, val is greater than glbc*/
			dctrl->tlb_disable = true;	/*The TLB should be disable after the current descriptor execution completes*/
			dctrl->tlb_disable_ch = glbc;
		} else {
			reg_write(dctrl, DC_TLB_GLBC, glbc);
		}
	}
}

static void dctrl_tlb_configure(struct dpu_ctrl *dctrl)
{
	unsigned int tlbv = 0;

	tlbv |= (1 << DC_CNM_LBIT);
	tlbv |= (1 << DC_GCN_LBIT);

	/*mutex_lock(dctrl->lock);*/
	reg_write(dctrl, DC_TLB_TLBV, tlbv);
	reg_write(dctrl, DC_TLB_TLBA, dctrl->tlba);
#if 0
	reg_write(dctrl, DC_TLB_TLBC, DC_CH0_INVLD | DC_CH1_INVLD |
			DC_CH2_INVLD | DC_CH3_INVLD);
#endif
	/*mutex_unlock(dctrl->lock);*/
}

static void dctrl_tlb_disable(struct dpu_ctrl *dctrl)
{
	/*Invalid and disable for all layers.*/
	reg_write(dctrl, DC_TLB_TLBC, DC_CH0_INVLD | DC_CH1_INVLD |
			DC_CH2_INVLD | DC_CH3_INVLD);
	reg_write(dctrl, DC_TLB_GLBC, 0);
}

static void csc_mode_set(struct dpu_ctrl *dctrl, csc_mode_t mode) {
	switch(mode) {
	case CSC_MODE_0:
		reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD0 | DC_CSC_MULT_RV_MD0);
		reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD0 | DC_CSC_MULT_GV_MD0);
		reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD0);
		reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD0 | DC_CSC_SUB_UV_MD0);
		break;
	case CSC_MODE_1:
		reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD1 | DC_CSC_MULT_RV_MD1);
		reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD1 | DC_CSC_MULT_GV_MD1);
		reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD1);
		reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD1 | DC_CSC_SUB_UV_MD1);
		break;
	case CSC_MODE_2:
		reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD2 | DC_CSC_MULT_RV_MD2);
		reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD2 | DC_CSC_MULT_GV_MD2);
		reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD2);
		reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD2 | DC_CSC_SUB_UV_MD2);
		break;
	case CSC_MODE_3:
		reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD3 | DC_CSC_MULT_RV_MD3);
		reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD3 | DC_CSC_MULT_GV_MD3);
		reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD3);
		reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD3 | DC_CSC_SUB_UV_MD3);
		break;
	default:
		dev_err(dctrl->dev, "Set csc mode err!\n");
		break;
	}
}

static void slcd_send_mcu_command(struct dpu_ctrl *dctrl, unsigned long cmd)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg & ~DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_CMD | (cmd & ~DC_SLCD_REG_IF_FLAG_MASK));
}

static void slcd_send_mcu_data(struct dpu_ctrl *dctrl, unsigned long data)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg | DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_DATA | (data & ~DC_SLCD_REG_IF_FLAG_MASK));
}

static void slcd_send_mcu_prm(struct dpu_ctrl *dctrl, unsigned long data)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg & ~DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_PRM | (data & ~DC_SLCD_REG_IF_FLAG_MASK));
}

static void wait_slcd_busy(struct dpu_ctrl *dctrl)
{
	int count = 100000;
	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY)
			&& count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev,"SLCDC wait busy state wrong");
	}
}

static int wait_dc_state(struct dpu_ctrl *dctrl, uint32_t state, uint32_t flag)
{
	unsigned long timeout = 20000;
	while(((!(reg_read(dctrl, DC_ST) & state)) == flag) && timeout) {
		timeout--;
		udelay(10);
	}
	if(timeout <= 0) {
		printk("LCD wait state timeout! state = 0x%08x, DC_ST = 0x%08x\n", state, reg_read(dctrl, DC_ST));
		return -1;
	}

	return 0;
}

static void ingenicfb_cmp_start(struct dpu_ctrl *dctrl)
{
	reg_write(dctrl, DC_FRM_CFG_CTRL, DC_FRM_START);
}

static void ingenicfb_slcd_mcu_init(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct smart_config *smart_config;
	struct smart_lcd_data_table *data_table;
	uint32_t length_data_table;
	uint32_t i;

	smart_config = panel->smart_config;
	if (panel->lcd_type != LCD_TYPE_SLCD || smart_config == NULL)
		return;

	data_table = smart_config->data_table;
	length_data_table = smart_config->length_data_table;
	if(length_data_table && data_table) {
		for(i = 0; i < length_data_table; i++) {
			switch (data_table[i].type) {
			case SMART_CONFIG_DATA:
				slcd_send_mcu_data(dctrl, data_table[i].value);
				break;
			case SMART_CONFIG_PRM:
				slcd_send_mcu_prm(dctrl, data_table[i].value);
				break;
			case SMART_CONFIG_CMD:
				slcd_send_mcu_command(dctrl, data_table[i].value);
				break;
			case SMART_CONFIG_UDELAY:
				udelay(data_table[i].value);
				break;
			default:
				printk("Unknow SLCD data type\n");
				break;
			}
		}
	}
}

static void tft_timing_init(struct dpu_ctrl *dctrl, struct fb_videomode *modes) {
	uint32_t hps;
	uint32_t hpe;
	uint32_t vps;
	uint32_t vpe;
	uint32_t hds;
	uint32_t hde;
	uint32_t vds;
	uint32_t vde;

	hps = modes->hsync_len;
	hpe = hps + modes->left_margin + modes->xres + modes->right_margin;
	vps = modes->vsync_len;
	vpe = vps + modes->upper_margin + modes->yres + modes->lower_margin;

	hds = modes->hsync_len + modes->left_margin;
	hde = hds + modes->xres;
	vds = modes->vsync_len + modes->upper_margin;
	vde = vds + modes->yres;

	reg_write(dctrl, DC_TFT_HSYNC,
		  (hps << DC_HPS_LBIT) |
		  (hpe << DC_HPE_LBIT));
	reg_write(dctrl, DC_TFT_VSYNC,
		  (vps << DC_VPS_LBIT) |
		  (vpe << DC_VPE_LBIT));
	reg_write(dctrl, DC_TFT_HDE,
		  (hds << DC_HDS_LBIT) |
		  (hde << DC_HDE_LBIT));
	reg_write(dctrl, DC_TFT_VDE,
		  (vds << DC_VDS_LBIT) |
		  (vde << DC_VDE_LBIT));
}

void tft_cfg_init(struct dpu_ctrl *dctrl, struct tft_config *tft_config) {
	uint32_t tft_cfg;
	uint32_t lcd_cgu;

	lcd_cgu = *(volatile unsigned int *)(0xb0000064);
	if(tft_config->pix_clk_inv) {
		lcd_cgu |= (0x1 << 26);
	} else {
		lcd_cgu &= ~(0x1 << 26);
	}
	*(volatile unsigned int *)(0xb0000064) = lcd_cgu;

	tft_cfg = reg_read(dctrl, DC_TFT_CFG);
	if(tft_config->de_dl) {
		tft_cfg |= DC_DE_DL;
	} else {
		tft_cfg &= ~DC_DE_DL;
	}

	if(tft_config->sync_dl) {
		tft_cfg |= DC_SYNC_DL;
	} else {
		tft_cfg &= ~DC_SYNC_DL;
	}
	if(tft_config->vsync_dl) {
		tft_cfg |= DC_VSYNC_DL;
	} else {
		tft_cfg &= ~DC_VSYNC_DL;
	}

	tft_cfg &= ~DC_COLOR_EVEN_MASK;
	switch(tft_config->color_even) {
	case TFT_LCD_COLOR_EVEN_RGB:
		tft_cfg |= DC_EVEN_RGB;
		break;
	case TFT_LCD_COLOR_EVEN_RBG:
		tft_cfg |= DC_EVEN_RBG;
		break;
	case TFT_LCD_COLOR_EVEN_BGR:
		tft_cfg |= DC_EVEN_BGR;
		break;
	case TFT_LCD_COLOR_EVEN_BRG:
		tft_cfg |= DC_EVEN_BRG;
		break;
	case TFT_LCD_COLOR_EVEN_GBR:
		tft_cfg |= DC_EVEN_GBR;
		break;
	case TFT_LCD_COLOR_EVEN_GRB:
		tft_cfg |= DC_EVEN_GRB;
		break;
	default:
		printk("err!\n");
		break;
	}

	tft_cfg &= ~DC_COLOR_ODD_MASK;
	switch(tft_config->color_odd) {
	case TFT_LCD_COLOR_ODD_RGB:
		tft_cfg |= DC_ODD_RGB;
		break;
	case TFT_LCD_COLOR_ODD_RBG:
		tft_cfg |= DC_ODD_RBG;
		break;
	case TFT_LCD_COLOR_ODD_BGR:
		tft_cfg |= DC_ODD_BGR;
		break;
	case TFT_LCD_COLOR_ODD_BRG:
		tft_cfg |= DC_ODD_BRG;
		break;
	case TFT_LCD_COLOR_ODD_GBR:
		tft_cfg |= DC_ODD_GBR;
		break;
	case TFT_LCD_COLOR_ODD_GRB:
		tft_cfg |= DC_ODD_GRB;
		break;
	default:
		printk("err!\n");
		break;
	}

	tft_cfg &= ~DC_MODE_MASK;
	switch(tft_config->mode) {
	case TFT_LCD_MODE_PARALLEL_888:
		tft_cfg |= DC_MODE_PARALLEL_888;
		break;
	case TFT_LCD_MODE_PARALLEL_666:
		tft_cfg |= DC_MODE_PARALLEL_666;
		break;
	case TFT_LCD_MODE_PARALLEL_565:
		tft_cfg |= DC_MODE_PARALLEL_565;
		break;
	case TFT_LCD_MODE_SERIAL_RGB:
		tft_cfg |= DC_MODE_SERIAL_8BIT_RGB;
		break;
	case TFT_LCD_MODE_SERIAL_RGBD:
		tft_cfg |= DC_MODE_SERIAL_8BIT_RGBD;
		break;
	default:
		printk("err!\n");
		break;
	}
	reg_write(dctrl, DC_TFT_CFG, tft_cfg);
}

static int ingenicfb_tft_set_par(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct lcd_panel_ops *panel_ops;
	struct fb_videomode *mode = dctrl->active_video_mode;

	panel_ops = panel->ops;

	tft_timing_init(dctrl, mode);
	tft_cfg_init(dctrl, panel->tft_config);
	if(panel_ops && panel_ops->enable)
		panel_ops->enable(panel);

	return 0;
}

static void slcd_cfg_init(struct dpu_ctrl *dctrl,
		struct smart_config *smart_config) {
	uint32_t slcd_cfg;

	if (smart_config == NULL) {
		dev_info(dctrl->dev, "SLCD use default config\n");
		return;
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	slcd_cfg &= ~DC_RDY_SWITCH;
	slcd_cfg &= ~DC_CS_EN;

	if(smart_config->te_switch) {
		slcd_cfg |= DC_TE_SWITCH;

		if(smart_config->te_dp) {
			slcd_cfg |= DC_TE_DP;
		} else {
			slcd_cfg &= ~DC_TE_DP;
		}
		if(smart_config->te_md) {
			slcd_cfg |= DC_TE_MD;
		} else {
			slcd_cfg &= ~DC_TE_MD;
		}
		if(smart_config->te_anti_jit) {
			slcd_cfg |= DC_TE_ANTI_JIT;
		} else {
			slcd_cfg &= ~DC_TE_ANTI_JIT;
		}
	} else {
		slcd_cfg &= ~DC_TE_SWITCH;
	}

	if(smart_config->te_mipi_switch) {
		slcd_cfg |= DC_TE_MIPI_SWITCH;
	} else {
		slcd_cfg &= ~DC_TE_MIPI_SWITCH;
	}

	if(smart_config->dc_md) {
		slcd_cfg |= DC_DC_MD;
	} else {
		slcd_cfg &= ~DC_DC_MD;
	}

	if(smart_config->wr_md) {
		slcd_cfg |= DC_WR_DP;
	} else {
		slcd_cfg &= ~DC_WR_DP;
	}

	slcd_cfg &= ~DC_DBI_TYPE_MASK;
	switch(smart_config->smart_type){
	case SMART_LCD_TYPE_8080:
		slcd_cfg |= DC_DBI_TYPE_B_8080;
		break;
	case SMART_LCD_TYPE_6800:
		slcd_cfg |= DC_DBI_TYPE_A_6800;
		break;
	case SMART_LCD_TYPE_SPI_3:
		slcd_cfg |= DC_DBI_TYPE_C_SPI_3;
		break;
	case SMART_LCD_TYPE_SPI_4:
		slcd_cfg |= DC_DBI_TYPE_C_SPI_4;
		break;
	default:
		printk("err!\n");
		break;
	}

	slcd_cfg &= ~DC_DATA_FMT_MASK;
	switch(smart_config->pix_fmt) {
	case SMART_LCD_FORMAT_888:
		slcd_cfg |= DC_DATA_FMT_888;
		break;
	case SMART_LCD_FORMAT_666:
		slcd_cfg |= DC_DATA_FMT_666;
		break;
	case SMART_LCD_FORMAT_565:
		slcd_cfg |= DC_DATA_FMT_565;
		break;
	default:
		printk("err!\n");
		break;
	}

	slcd_cfg &= ~DC_DWIDTH_MASK;
	switch(smart_config->dwidth) {
	case SMART_LCD_DWIDTH_8_BIT:
		slcd_cfg |= DC_DWIDTH_8BITS;
		break;
	case SMART_LCD_DWIDTH_9_BIT:
		slcd_cfg |= DC_DWIDTH_9BITS;
		break;
	case SMART_LCD_DWIDTH_16_BIT:
		slcd_cfg |= DC_DWIDTH_16BITS;
		break;
	case SMART_LCD_DWIDTH_18_BIT:
		slcd_cfg |= DC_DWIDTH_18BITS;
		break;
	case SMART_LCD_DWIDTH_24_BIT:
		slcd_cfg |= DC_DWIDTH_24BITS;
		break;
	default:
		printk("err!\n");
		break;
	}

	slcd_cfg &= ~DC_CWIDTH_MASK;
	switch(smart_config->cwidth) {
	case SMART_LCD_CWIDTH_8_BIT:
		slcd_cfg |= DC_CWIDTH_8BITS;
		break;
	case SMART_LCD_CWIDTH_9_BIT:
		slcd_cfg |= DC_CWIDTH_9BITS;
		break;
	case SMART_LCD_CWIDTH_16_BIT:
		slcd_cfg |= DC_CWIDTH_16BITS;
		break;
	case SMART_LCD_CWIDTH_18_BIT:
		slcd_cfg |= DC_CWIDTH_18BITS;
		break;
	case SMART_LCD_CWIDTH_24_BIT:
		slcd_cfg |= DC_CWIDTH_24BITS;
		break;
	default:
		printk("err!\n");
		break;
	}

	reg_write(dctrl, DC_SLCD_CFG, slcd_cfg);

	return;
}

static int slcd_timing_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel,
		struct fb_videomode *mode)
{
	uint32_t width = mode->xres;
	uint32_t height = mode->yres;
	uint32_t dhtime = 0;
	uint32_t dltime = 0;
	uint32_t chtime = 0;
	uint32_t cltime = 0;
	uint32_t tah = 0;
	uint32_t tas = 0;
	uint32_t slowtime = 0;

	/*frm_size*/
	reg_write(dctrl, DC_SLCD_FRM_SIZE,
		  ((width << DC_SLCD_FRM_H_SIZE_LBIT) |
		   (height << DC_SLCD_FRM_V_SIZE_LBIT)));

	/* wr duty */
	reg_write(dctrl, DC_SLCD_WR_DUTY,
		  ((dhtime << DC_DSTIME_LBIT) |
		   (dltime << DC_DDTIME_LBIT) |
		   (chtime << DC_CSTIME_LBIT) |
		   (cltime << DC_CDTIME_LBIT)));

	/* slcd timing */
	reg_write(dctrl, DC_SLCD_TIMING,
		  ((tah << DC_TAH_LBIT) |
		  (tas << DC_TAS_LBIT)));

	/* slow time */
	reg_write(dctrl, DC_SLCD_SLOW_TIME, slowtime);

	return 0;
}

static int ingenicfb_slcd_set_par(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct lcd_panel_ops *panel_ops;
	struct fb_videomode *mode = dctrl->active_video_mode;

	panel_ops = panel->ops;

	slcd_cfg_init(dctrl, panel->smart_config);
	slcd_timing_init(dctrl, panel, mode);

	if(panel_ops && panel_ops->enable)
		    panel_ops->enable(panel);

	ingenicfb_slcd_mcu_init(dctrl);

	return 0;
}



static void disp_common_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel) {
	uint32_t disp_com;

	disp_com = reg_read(dctrl, DC_DISP_COM);
	disp_com &= ~DC_DP_IF_SEL_MASK;
	if(panel->lcd_type == LCD_TYPE_SLCD) {
		disp_com |= DC_DISP_COM_SLCD;
	} else if(panel->lcd_type == LCD_TYPE_MIPI_SLCD) {
		disp_com |= DC_DISP_COM_MIPI_SLCD;
	} else {
		disp_com |= DC_DISP_COM_TFT;
	}
	if(panel->dither_enable) {
		disp_com |= DC_DP_DITHER_EN;
		disp_com &= ~DC_DP_DITHER_DW_MASK;
		disp_com |= panel->dither.dither_red
			     << DC_DP_DITHER_DW_RED_LBIT;
		disp_com |= panel->dither.dither_green
			    << DC_DP_DITHER_DW_GREEN_LBIT;
		disp_com |= panel->dither.dither_blue
			    << DC_DP_DITHER_DW_BLUE_LBIT;
	} else {
		disp_com &= ~DC_DP_DITHER_EN;
	}
	reg_write(dctrl, DC_DISP_COM, disp_com);

	/* QOS */
#if 0
	reg_write(dctrl, DC_PCFG_RD_CTRL, 7);
	reg_write(dctrl, DC_PCFG_WR_CTRL, 1);
	reg_write(dctrl, DC_WDMA_PCFG, 0x1ff << 18 | 0x1f0 << 9 | 0x1e0 << 0);
	reg_write(dctrl, DC_OFIFO_PCFG, 0x1ff << 18 | 0x1f0 << 9 | 0x1e0 << 0);
	reg_write(dctrl, DC_CMPW_PCFG_CTRL, 1 << 10);
#endif
}

static void common_cfg_init(struct dpu_ctrl *dctrl)
{
	unsigned com_cfg = 0;

	com_cfg = reg_read(dctrl, DC_COM_CONFIG);
	/*Keep COM_CONFIG reg first bit 0 */

	com_cfg &= ~DC_OUT_SEL;
	if(dctrl->chan == DATA_CH_RDMA) {
		com_cfg |= DC_OUT_SEL_RDMA;
	} else {
		com_cfg |= DC_OUT_SEL_CMP;
	}

	/* set burst length 32*/
	com_cfg &= ~DC_BURST_LEN_BDMA_MASK;
	com_cfg |= DC_BURST_LEN_BDMA_32;
	com_cfg &= ~DC_BURST_LEN_WDMA_MASK;
	com_cfg |= DC_BURST_LEN_WDMA_32;
	com_cfg &= ~DC_BURST_LEN_RDMA_MASK;
	com_cfg |= DC_BURST_LEN_RDMA_32;

	reg_write(dctrl, DC_COM_CONFIG, com_cfg);
}

static void ingenic_set_pixclk(struct dpu_ctrl *dctrl, unsigned int pixclock_khz)
{
	unsigned long rate, prate;
	struct clk *clk;

	rate = PICOS2KHZ(pixclock_khz) * 1000;
	clk = clk_get_parent(dctrl->pclk);
	prate = clk_get_rate(clk);

	if(prate % rate)
		rate = prate / (prate / rate) + 1;

	clk_set_rate(dctrl->pclk, rate);
}

void dump_rdma_desc(struct dpu_ctrl *dctrl)
{
	struct ingenicfb_sreadesc **sreadesc;
	struct rdma_setup_info *rdma_info = &dctrl->rdma_info;
	int i = 0;

	sreadesc = dctrl->sreadesc;
	for(i = 0; i < rdma_info->nframes; i++) {

		dev_info(dctrl->dev, "sreadesc[%d]->RdmaNextCfgAddr: 0x%x\n", i, sreadesc[i]->RdmaNextCfgAddr);
		dev_info(dctrl->dev, "sreadesc[%d]->FrameBufferAddr: 0x%x\n", i, sreadesc[i]->FrameBufferAddr);
		dev_info(dctrl->dev, "sreadesc[%d]->Stride	: %d\n", i, sreadesc[i]->Stride);
		dev_info(dctrl->dev, "sreadesc[%d]->ChainCfg	: %x\n", i, sreadesc[i]->ChainCfg.d32);
		dev_info(dctrl->dev, "sreadesc[%d]->InterruptControl: %x\n", i, sreadesc[i]->InterruptControl.d32);
	}

}
/*
 *
 *	配置寄LCD控制器, 可以配置到寄存器.
 *
 *	1. 根据rdma需求的配置信息，配置rdma描述符.
 *	2. 控制器需要处于停止状态?
 * */
int dpu_ctrl_rdma_setup(struct dpu_ctrl *dctrl, struct rdma_setup_info *rdma_info)
{

	struct ingenicfb_sreadesc **sreadesc;
	int i = 0;

	/* Init RDMA DESC as continous. */
	sreadesc = dctrl->sreadesc;
	for(i = 0; i < rdma_info->nframes; i++) {
		sreadesc[i]->RdmaNextCfgAddr = dctrl->sreadesc_phys[i];
		sreadesc[i]->FrameBufferAddr = rdma_info->vidmem_phys[i];
		sreadesc[i]->Stride = rdma_info->stride;
		sreadesc[i]->ChainCfg.d32 = 0;
		sreadesc[i]->ChainCfg.b.format = rdma_info->format;
		sreadesc[i]->ChainCfg.b.color = rdma_info->color;
		sreadesc[i]->ChainCfg.b.change_2_cmp = 0;
		sreadesc[i]->ChainCfg.b.chain_end = !rdma_info->continuous;
#ifdef TEST_IRQ
		sreadesc[i]->InterruptControl.d32 = DC_SOS_MSK | DC_EOS_MSK | DC_EOD_MSK;
#else
		sreadesc[i]->InterruptControl.d32 = DC_SOS_MSK;
#endif
	}
	sreadesc[0]->RdmaNextCfgAddr = dctrl->sreadesc_phys[0];
	dctrl->rdma_last_frame = 0;

	memcpy(&dctrl->rdma_info, rdma_info, sizeof(struct rdma_setup_info));

	//dump_rdma_desc(dctrl);



	return 0;
}

int dpu_ctrl_rdma_change(struct dpu_ctrl *dctrl, int frame)
{
	struct ingenicfb_sreadesc **sreadesc;
	unsigned long flags;

	sreadesc = dctrl->sreadesc;

	/* TODO: 可能改变的内容，包括格式，大小等。这里只是切换frame, 什么时候真正切换还不一定.*/


	spin_lock_irqsave(&dctrl->irq_lock, flags);


	/*如果此时正处于直通显示, 需要停止直通，切换到rdma通道.*/

	if(dctrl->rdma_desc_changing) {
		dev_warn(dctrl->dev, "[rdma] last frame [%d] still in changing!\n", dctrl->rdma_changing_frame);
		dctrl_tlb_invalidate_ch(dctrl, DC_ALL_INVLD);
	}

	sreadesc[dctrl->rdma_last_frame]->RdmaNextCfgAddr = dctrl->sreadesc_phys[frame];

	dctrl->rdma_changing_frame = frame;
	dctrl->rdma_desc_changing = 1;

	spin_unlock_irqrestore(&dctrl->irq_lock, flags);

	return 0;
}

int dpu_ctrl_rdma_start(struct dpu_ctrl *dctrl)
{
	reg_write(dctrl, DC_RDMA_CHAIN_ADDR, dctrl->sreadesc_phys[0]);
	reg_write(dctrl, DC_RDMA_CHAIN_CTRL, DC_RDMA_START);

	//dump_rdma_desc_reg(dctrl);

	return 0;
}

int dpu_ctrl_rdma_stop(struct dpu_ctrl *dctrl, enum stop_mode mode)
{
	int ret = 0;

	if(mode == QCK_STOP) {
		reg_write(dctrl, DC_CTRL, DC_QCK_STP_RDMA);
		wait_dc_state(dctrl, DC_SRD_WORKING, 0);
	} else {
		dctrl->srd_stopped = 0;
		reg_write(dctrl, DC_CTRL, DC_GEN_STP_RDMA);
		dev_info(dctrl->dev, "stopping rdma ...\n");
		ret = wait_event_interruptible_timeout(dctrl->wq,
				dctrl->srd_stopped == 1,msecs_to_jiffies(1000/dctrl->active_video_mode->refresh + 3));
		if((ret == 0) && reg_read(dctrl, DC_ST) & DC_SRD_WORKING) {
			dev_err(dctrl->dev, "dpu rdma wait gen stop timeout!!!\n");
			dump_lcdc_registers(dctrl);
		}
	}

	return 0;
}

static void dump_frm_desc(struct ingenicfb_framedesc *framedesc, int index)
{
	printk("-------User Frame Descriptor index[%d]-----\n", index);
	printk("FramedescAddr:	    0x%x\n",(uint32_t)framedesc);
	printk("FrameNextCfgAddr:   0x%x\n",framedesc->FrameNextCfgAddr);
	printk("FrameSize:          0x%x\n",framedesc->FrameSize.d32);
	printk("FrameCtrl:          0x%x\n",framedesc->FrameCtrl.d32);
	printk("WritebackAddr:	    0x%x\n",framedesc->WritebackAddr);
	printk("WritebackStride:    0X%x\n",framedesc->WritebackStride);
	printk("Layer0CfgAddr:      0x%x\n",framedesc->Layer0CfgAddr);
	printk("Layer1CfgAddr:      0x%x\n",framedesc->Layer1CfgAddr);
	printk("Layer2CfgAddr:      0x%x\n",framedesc->Layer2CfgAddr);
	printk("Layer3CfgAddr:      0x%x\n",framedesc->Layer3CfgAddr);
	printk("LayerCfgEn:	    0x%x\n",framedesc->LayCfgEn.d32);
	printk("InterruptControl:   0x%x\n",framedesc->InterruptControl.d32);
	printk("-------User Frame Descriptor index[%d]-----\n", index);
}

static void dump_layer_desc(struct ingenicfb_layerdesc *layerdesc, int row, int col)
{
	printk("------User layer Descriptor index[%d][%d]------\n", row, col);
	printk("LayerdescAddr:	    0x%x\n",(uint32_t)layerdesc);
	printk("LayerSize:          0x%x\n",layerdesc->LayerSize.d32);
	printk("LayerCfg:           0x%x\n",layerdesc->LayerCfg.d32);
	printk("LayerBufferAddr:    0x%x\n",layerdesc->LayerBufferAddr);
	printk("LayerScale:         0x%x\n",layerdesc->LayerScale.d32);
	printk("LayerResizeCoef_X:  0x%x\n",layerdesc->layerresizecoef_x);
	printk("LayerResizeCoef_Y:  0x%x\n",layerdesc->layerresizecoef_y);
	printk("LayerPos:           0x%x\n",layerdesc->LayerPos.d32);
	printk("LayerStride:        0x%x\n",layerdesc->LayerStride);
	printk("Layer_UV_addr:	    0x%x\n",layerdesc->UVBufferAddr);
	printk("Layer_UV_stride:    0x%x\n",layerdesc->UVStride);
	printk("------User layer Descriptor index[%d][%d]------\n", row, col);
}

void dump_lay_cfg(struct ingenicfb_lay_cfg * lay_cfg, int index)
{
	printk("------User disp set index[%d]------\n", index);
	printk("lay_en:		   0x%x\n",lay_cfg->lay_en);
	printk("tlb_en:		   0x%x\n",lay_cfg->tlb_en);
	printk("lay_scale_en:	   0x%x\n",lay_cfg->lay_scale_en);
	printk("lay_z_order:	   0x%x\n",lay_cfg->lay_z_order);
	printk("source_w:	   0x%x\n",lay_cfg->source_w);
	printk("source_h:	   0x%x\n",lay_cfg->source_h);
	printk("disp_pos_x:	   0x%x\n",lay_cfg->disp_pos_x);
	printk("disp_pos_y:	   0x%x\n",lay_cfg->disp_pos_y);
	printk("scale_w:	   0x%x\n",lay_cfg->scale_w);
	printk("scale_h:	   0x%x\n",lay_cfg->scale_h);
	printk("g_alpha_en:	   0x%x\n",lay_cfg->g_alpha_en);
	printk("g_alpha_val:	   0x%x\n",lay_cfg->g_alpha_val);
	printk("color:		   0x%x\n",lay_cfg->color);
	printk("format:		   0x%x\n",lay_cfg->format);
	printk("stride:		   0x%x\n",lay_cfg->stride);
	printk("addr[0]:	   0x%x\n",lay_cfg->addr[0]);
	printk("addr[1]:	   0x%x\n",lay_cfg->addr[1]);
	printk("addr[2]:	   0x%x\n",lay_cfg->addr[2]);

	printk("------User disp set index[%d]------\n", index);
}

static void dump_comp_info(struct comp_setup_info *comp_info)
{
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct wback_cfg *wback_info	= &frm_cfg->wback_info;
	int i;

	printk("\n\n");
	printk("width: %d, height: %d\n",frm_cfg->width, frm_cfg->height);

	printk("wback_info: en:%d  dither_en:%d  fmt:%d  stride:%d  addr:%08x \n",wback_info->en, wback_info->dither_en, wback_info->fmt, wback_info->stride, wback_info->addr);

	for(i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		printk("####################lay_cfg info %d ###################\n", i);
		printk("lay_en:%d  tlb_en:%d \n", lay_cfg[i].lay_en, lay_cfg[i].tlb_en);
		printk("source_w:%d  source_h:%d disp_pos_x:%d disp_pos_y:%d \n", lay_cfg[i].source_w, lay_cfg[i].source_h, lay_cfg[i].disp_pos_x, lay_cfg[i].disp_pos_y);
		printk("g_alpha_en:%d g_alpha_val:%d color:%d format:%d stride:%d \n", lay_cfg[i].g_alpha_en, lay_cfg[i].g_alpha_val, lay_cfg[i].color, lay_cfg[i].format, lay_cfg[i].stride);
		printk("scale_w:%d scale_h:%d addr:%08x uv_addr:%08x uv_stride:%d\n",lay_cfg[i].scale_w, lay_cfg[i].scale_h, lay_cfg[i].addr, lay_cfg[i].uv_addr, lay_cfg[i].uv_stride);

		printk("\n");
	}
	printk("\n\n");
}

void dump_comp_desc(struct dpu_ctrl *dctrl)
{
	int i, j;
	for(i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		for(j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
			dump_layer_desc(dctrl->layerdesc[i][j], i, j);
		}
		dump_frm_desc(dctrl->framedesc[i], i);
	}
}

static int dpu_comp_framedesc_setup(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info)
{
	struct ingenicfb_framedesc **framedesc;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct wback_cfg *wback_info	= &frm_cfg->wback_info;
	struct fb_videomode *mode = dctrl->active_video_mode;
	int i = 0;
	int next_frame = 0;

	if(wback_info->en) {
		comp_info->out_mode = COMP_WRITE_BACK;
	} else {
		comp_info->out_mode = COMP_DIRECT_OUT;
		frm_cfg->width = mode->xres;
		frm_cfg->height = mode->yres;
	}

	if(dctrl->comp_stopped) {
		/*如果composer没有启动，则初始化为第0帧.*/
		dctrl->comp_next_frame = 0;
		next_frame = 0;
	} else {

		/*如果composer启动，每次setup的修改的是下一个待刷新的帧描述符.*/
		dctrl->comp_next_frame += 1;
		dctrl->comp_next_frame %= DPU_COMP_MAX_FRAMEDESC;

		next_frame = dctrl->comp_next_frame;
	}

	/*update framedesc*/

	framedesc = dctrl->framedesc;

	/*构建链接自己的帧描述符.*/

	framedesc[next_frame]->FrameNextCfgAddr = dctrl->framedesc_phys[next_frame];
	framedesc[next_frame]->FrameSize.d32 = 0;
	framedesc[next_frame]->FrameSize.b.width = frm_cfg->width;
	framedesc[next_frame]->FrameSize.b.height = frm_cfg->height;
	framedesc[next_frame]->FrameCtrl.d32 = 0;
	framedesc[next_frame]->FrameCtrl.b.stop = comp_info->out_mode == COMP_DIRECT_OUT ? 0 : 1; //当直接显示时，帧描述符不停止.
	framedesc[next_frame]->FrameCtrl.b.wb_en = !!wback_info->en;
	framedesc[next_frame]->FrameCtrl.b.direct_en = comp_info->out_mode == COMP_DIRECT_OUT ? 1 : 0;
	framedesc[next_frame]->FrameCtrl.b.change_2_rdma = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_en = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_auto = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_auto = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_b_dw = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_g_dw = 0;
	framedesc[next_frame]->FrameCtrl.b.wb_dither_r_dw = 0;

	framedesc[next_frame]->WritebackAddr = wback_info->addr;
	framedesc[next_frame]->WritebackStride = wback_info->stride;
	framedesc[next_frame]->FrameCtrl.b.wb_format = wback_info->fmt;
	framedesc[next_frame]->Layer0CfgAddr = dctrl->layerdesc_phys[next_frame][0];
	framedesc[next_frame]->Layer1CfgAddr = dctrl->layerdesc_phys[next_frame][1];
	framedesc[next_frame]->Layer2CfgAddr = dctrl->layerdesc_phys[next_frame][2];
	framedesc[next_frame]->Layer3CfgAddr = dctrl->layerdesc_phys[next_frame][3];
	framedesc[next_frame]->LayCfgEn.d32 = 0;
	framedesc[next_frame]->LayCfgEn.b.lay0_scl_en = lay_cfg[0].lay_scale_en;
	framedesc[next_frame]->LayCfgEn.b.lay1_scl_en = lay_cfg[1].lay_scale_en;
	framedesc[next_frame]->LayCfgEn.b.lay2_scl_en = lay_cfg[2].lay_scale_en;
	framedesc[next_frame]->LayCfgEn.b.lay3_scl_en = lay_cfg[3].lay_scale_en;
	framedesc[next_frame]->LayCfgEn.b.lay0_en = lay_cfg[0].lay_en;
	framedesc[next_frame]->LayCfgEn.b.lay1_en = lay_cfg[1].lay_en;
	framedesc[next_frame]->LayCfgEn.b.lay2_en = lay_cfg[2].lay_en;
	framedesc[next_frame]->LayCfgEn.b.lay3_en = lay_cfg[3].lay_en;
	framedesc[next_frame]->LayCfgEn.b.lay0_z_order = lay_cfg[0].lay_z_order;
	framedesc[next_frame]->LayCfgEn.b.lay1_z_order = lay_cfg[1].lay_z_order;
	framedesc[next_frame]->LayCfgEn.b.lay2_z_order = lay_cfg[2].lay_z_order;
	framedesc[next_frame]->LayCfgEn.b.lay3_z_order = lay_cfg[3].lay_z_order;

	/*使能写回功能，则认为是需要开启EOW中断.*/
	if((comp_info->out_mode == COMP_WRITE_BACK) && (wback_info->en == 1)) {
		framedesc[next_frame]->InterruptControl.d32 = DC_EOW_MSK;
	} else {
		framedesc[next_frame]->InterruptControl.d32 = DC_SOC_MSK;
	}
	/*
#ifndef TEST_IRQ
framedesc[i]->InterruptControl.d32 = DC_SOC_MSK;
#else
framedesc[i]->InterruptControl.d32 = DC_SOC_MSK | DC_EOD_MSK | DC_EOW_MSK | DC_EOC_MSK;
#endif
*/

	return 0;
}

static int dpu_comp_layerdesc_setup(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info)
{
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct wback_cfg *wback_info	= &frm_cfg->wback_info;
	int j = 0;

	unsigned int next_frame = dctrl->comp_next_frame;

	for(j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
		if(!lay_cfg[j].lay_en) {
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_width = lay_cfg[j].scale_w = 0;
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_height = lay_cfg[j].scale_h = 0;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_x = 0;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_y = 0;
			lay_cfg[j].lay_scale_en = 0;
			lay_cfg[j].tlb_en = 0;
			continue;
		}

		dctrl->layerdesc[next_frame][j]->LayerSize.d32 		= 0;
		dctrl->layerdesc[next_frame][j]->LayerSize.b.width 	= lay_cfg[j].source_w;
		dctrl->layerdesc[next_frame][j]->LayerSize.b.height 	= lay_cfg[j].source_h;
		dctrl->layerdesc[next_frame][j]->LayerPos.d32 		= 0;
		dctrl->layerdesc[next_frame][j]->LayerPos.b.x_pos 	= lay_cfg[j].disp_pos_x;
		dctrl->layerdesc[next_frame][j]->LayerPos.b.y_pos 	= lay_cfg[j].disp_pos_y;
		dctrl->layerdesc[next_frame][j]->LayerCfg.d32 		= 0;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.g_alpha_en 	= lay_cfg[j].g_alpha_en;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.g_alpha 	= lay_cfg[j].g_alpha_val;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.color 	= lay_cfg[j].color;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.domain_multi = 1;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.format 	= lay_cfg[j].format;
		dctrl->layerdesc[next_frame][j]->LayerCfg.b.sharpl 	= 0;
		dctrl->layerdesc[next_frame][j]->LayerStride 		= lay_cfg[j].stride;
		dctrl->layerdesc[next_frame][j]->LayerScale.d32 	= 0;
		if(lay_cfg[j].lay_scale_en) {
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_width 	= lay_cfg[j].scale_w;
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_height 	= lay_cfg[j].scale_h;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_x 		= (512 * lay_cfg[j].source_w) / lay_cfg[j].scale_w;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_y 		= (512 * lay_cfg[j].source_h) / lay_cfg[j].scale_h;
		} else {
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_width 	= 0;
			dctrl->layerdesc[next_frame][j]->LayerScale.b.target_height 	= 0;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_x 		= 0;
			dctrl->layerdesc[next_frame][j]->layerresizecoef_y 		= 0;
		}

		dctrl->layerdesc[next_frame][j]->LayerBufferAddr 				= lay_cfg[j].addr[0];
		dctrl->layerdesc[next_frame][j]->UVBufferAddr 					= lay_cfg[j].uv_addr[0];
		dctrl->layerdesc[next_frame][j]->UVStride 					= lay_cfg[j].uv_stride;
	}

	return 0;
}

int dpu_ctrl_comp_setup(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info)
{
	int ret = 0;

	/*调用者需要保证comp_info的合法性.*/
//	dump_comp_info(comp_info);

	ret = dpu_comp_framedesc_setup(dctrl, comp_info);
	ret = dpu_comp_layerdesc_setup(dctrl, comp_info);

	//dump_comp_desc(dctrl);

	memcpy(&dctrl->comp_info, comp_info, sizeof(struct comp_setup_info));

	/*Only Composer Need CSC settings.*/
	dctrl->csc_mode = CSC_MODE_1;
	csc_mode_set(dctrl, dctrl->csc_mode);

	return ret;
}

static inline long timeval_sub_to_us(struct timespec64 lhs,
				struct timespec64 rhs)
{
	long sec, nsec;
	sec = lhs.tv_sec - rhs.tv_sec;
	nsec = lhs.tv_nsec - rhs.tv_nsec;

	return (sec*1000000 + nsec/1000);
}


static int dpu_ctrl_comp_start_block(struct dpu_ctrl *dctrl)
{
	//Mutex lock, this function used for comp writeback, oneshot.
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	int ret = 0;

	struct timespec64 time_start, time_end;
	unsigned int interval_in_us;
	unsigned int interval_in_ms;


	dctrl->wback_end = 0;

	ktime_get_real_ts64(&time_start);

	reg_write(dctrl, DC_FRM_CFG_CTRL, DC_FRM_START);

	ret = wait_event_interruptible_timeout(dctrl->wq, dctrl->wback_end == 1, msecs_to_jiffies(3000));
	if(ret == 0) {
		dev_err(dctrl->dev, "wait for wback timeout!\n");
		return -ETIMEDOUT;
	}

	ktime_get_real_ts64(&time_end);
	interval_in_us = timeval_sub_to_us(time_end, time_start);
	interval_in_ms = interval_in_us / 1000;

//	printk("composer cost %d us -- %dms\n", interval_in_us, interval_in_ms);


	return 0;
}

static int dpu_ctrl_comp_start_nonblock(struct dpu_ctrl *dctrl)
{
	//Mutex lock. this function used for comp direct output.

	if(!(reg_read(dctrl, DC_ST) & DC_DIRECT_WORKING)) {
		reg_write(dctrl, DC_FRM_CFG_CTRL, DC_FRM_START);
		dctrl->comp_stopped = 0;
	}

	return 0;
}

static void dctrl_flush_cache(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	unsigned int next_frame = dctrl->comp_next_frame;
	unsigned int buf_size;
	int i = 0;

	/* TODO:
	 * 如果使用的是tlb方式，则每次在更新描述符时，需要flush cache, 保证dpu取道的是正确的数据.
	 * */

	for(i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {

		if((!lay_cfg[i].lay_en) || (!lay_cfg[i].tlb_en)) {
			continue;
		}

		if(lay_cfg[i].format == LAYER_CFG_FORMAT_NV12 ||
				lay_cfg[i].format == LAYER_CFG_FORMAT_NV21) {
			unsigned int y_size = 0;
			unsigned int uv_size = 0;

			y_size = lay_cfg[i].stride * lay_cfg[i].source_h;
			uv_size = lay_cfg[i].stride * lay_cfg[i].source_h / 2;

			/* Y */
			if(lay_cfg[i].addr[0]) {
				dma_cache_wback_inv(lay_cfg[i].addr[0], y_size);
			}

			/* UV */
			if(lay_cfg[i].uv_addr[0]) {
				dma_cache_wback_inv(lay_cfg[i].uv_addr[0], uv_size);
			}

		} else if(lay_cfg[i].format == LAYER_CFG_FORMAT_RGB888 ||
				lay_cfg[i].format == LAYER_CFG_FORMAT_ARGB8888) {
			/*RGB888/ARGB888*/
			unsigned int buf_size = 0;
			buf_size = lay_cfg[i].stride * lay_cfg[i].source_h * 4;
			dma_cache_wback_inv(lay_cfg[i].addr[0], buf_size);
		} else {

			/*RGB565/RGB555.*/
			unsigned int buf_size = 0;
			buf_size = lay_cfg[i].stride * lay_cfg[i].source_h * 2;
			dma_cache_wback_inv(lay_cfg[i].addr[0], buf_size);
		}
	}
}

int dpu_ctrl_comp_start(struct dpu_ctrl *dctrl, int block)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	int ret = 0;
	unsigned int com_cfg = 0;
	unsigned int curr_frame_addr = 0;
	unsigned int curr_frame_site = 0;

	dctrl_tlb_configure(dctrl);
	dctrl_tlb_enable(dctrl);

	/*Flush cache if tlb en.*/
	dctrl_flush_cache(dctrl);


	if(0) { //DEBUG.
		curr_frame_addr = reg_read(dctrl, DC_FRM_CFG_ADDR);
		curr_frame_site = reg_read(dctrl, DC_FRM_CHAIN_SITE);
		if(curr_frame_addr != curr_frame_site) {
			dev_err(dctrl->dev, "--------error, framedesc overflow!!!curr_frame_addr: %x , curr_frame_site: %x\n", curr_frame_addr, curr_frame_site);
		}
	}

	/*comp_next_frame, 等待硬件更新的buffer.*/
	dctrl->comp_last_framedesc = dctrl->framedesc_phys[dctrl->comp_next_frame];
	dctrl->comp_last_frame = dctrl->comp_next_frame;

	reg_write(dctrl, DC_FRM_CFG_ADDR, dctrl->framedesc_phys[dctrl->comp_next_frame]);
	/*等待上一帧描述符确实被硬件读取，才继续更新下一帧描述符.*/

	if(!dctrl->comp_stopped) {
		ret = wait_event_interruptible_timeout(dctrl->wq,
				(dctrl->comp_hw_framedesc == dctrl->comp_last_framedesc), msecs_to_jiffies(400)); //40ms 25FPS 理论上肯定能够更新完成描述符.

		if(ret == 0) {
			dctrl_tlb_invalidate_ch(dctrl, DC_ALL_INVLD);
			dev_err(dctrl->dev, "wait fo dpu update comp framdesc timed out, dctrl->comp_hw_framedesc: %x, dctrl->comp_last_framedesc: %x\n",
					dctrl->comp_hw_framedesc, dctrl->comp_last_framedesc);
		} else {
#if 0
			dev_err(dctrl->dev, "last_frame: %d dctrl->comp_hw_framedesc: %x, dctrl->comp_last_framedesc: %x, LayerBufferAddr: %x\n",
					dctrl->comp_last_frame, dctrl->comp_hw_framedesc, dctrl->comp_last_framedesc, dctrl->layerdesc[dctrl->comp_last_frame][2]->LayerBufferAddr);
#endif

		}
	}

	if(dctrl->tlb_disable == true) {
		reg_write(dctrl, DC_TLB_GLBC, dctrl->tlb_disable_ch);
		dctrl->tlb_disable = false;
	}

	dctrl->comp_fps++;

	/*切换到comp 之前，先停止rdma.*/
	if((comp_info->out_mode == COMP_DIRECT_OUT) && (dctrl->chan == DATA_CH_RDMA)) {
		/*stop rdma ??*/
		if(reg_read(dctrl, DC_ST) & DC_SRD_WORKING) {
			dpu_ctrl_rdma_stop(dctrl, GEN_STOP);
		}
	}

	/* 只有当comp停止的时候，才需要重新启动合成.
	 * 初始化时，comp_stopped = 1.
	 * 启动一次之后, comp_stopped = 0.
	 *
	 * 当处于不直通的时候
	 * */
	if(block && comp_info->out_mode == COMP_WRITE_BACK) {
		ret = dpu_ctrl_comp_start_block(dctrl);
	} else {
		ret = dpu_ctrl_comp_start_nonblock(dctrl);
	}


	/* 切换显示数据通路到composer */
	if((comp_info->out_mode == COMP_DIRECT_OUT) && (dctrl->chan == DATA_CH_RDMA)) {

		com_cfg = reg_read(dctrl, DC_COM_CONFIG);
		com_cfg &= ~DC_OUT_SEL;

		com_cfg |= DC_OUT_SEL_CMP;
		reg_write(dctrl, DC_COM_CONFIG, com_cfg);
		dctrl->chan = DATA_CH_COMP;
	}

	//dump_lcdc_registers(dctrl);

	return ret;
}

int dpu_ctrl_comp_stop(struct dpu_ctrl *dctrl, enum stop_mode mode)
{
	struct ingenicfb_framedesc **framedesc = dctrl->framedesc;
	int ret = 0;
	int i = 0;

	if(!dctrl->support_comp) {
		return 0;
	}

	for(i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		/*当comp stop时，将描述符的LayCfgEn 都设置为0, 防止TLB没有初始化时出错.*/
		framedesc[i]->LayCfgEn.d32 = 0;
	}

	if(mode == QCK_STOP) {
		reg_write(dctrl, DC_CTRL, DC_QCK_STP_CMP);
		wait_dc_state(dctrl, DC_DIRECT_WORKING | DC_WRBK_WORKING, 0);
		dctrl->comp_stopped = 1;
	} else {
		dctrl->comp_stopped = 0;
		reg_write(dctrl, DC_CTRL, DC_GEN_STP_CMP);
		ret = wait_event_interruptible_timeout(dctrl->wq,
				dctrl->comp_stopped == 1,msecs_to_jiffies(3000));
		if(ret == 0) {
			dev_err(dctrl->dev, "dpu composer gen stop timeout!!!\n");
		}
	}

	dctrl_tlb_disable(dctrl);
	dctrl->comp_fps = 0;

	return ret;
}


/* 1. slcd/tft 接口初始化.
 * 2. 设置pixclk.
 * */
int dpu_ctrl_setup(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	unsigned int intc = 0;
	int ret = 0;

	disp_common_init(dctrl, dctrl->panel);
	common_cfg_init(dctrl);

	reg_write(dctrl, DC_CLR_ST, 0x01FFFFFE);

	switch (panel->lcd_type) {
		case LCD_TYPE_TFT:
			ingenicfb_tft_set_par(dctrl);
			break;
		case LCD_TYPE_MIPI_TFT:
			ingenicfb_tft_set_par(dctrl);
			dctrl->dsi->master_ops->mode_cfg(dctrl->dsi, 1);
			break;
		case LCD_TYPE_SLCD:
			ingenicfb_slcd_set_par(dctrl);
			break;
		case LCD_TYPE_MIPI_SLCD:
			ingenicfb_slcd_set_par(dctrl);
			dctrl->dsi->master_ops->mode_cfg(dctrl->dsi, 0);
			break;
	}

        /*       disp end      gen_stop    tft_under    frm_start    frm_end     wback over  GEN_STOP_SRD*/
//	intc = DC_EOD_MSK | DC_SDA_MSK | DC_UOT_MSK | DC_SOC_MSK | DC_EOF_MSK | DC_OOW_MSK | DC_SSA_MSK;
	intc = DC_EOD_MSK | DC_SDA_MSK | DC_UOT_MSK | DC_SOC_MSK | DC_OOW_MSK | DC_EOW_MSK | DC_SOS_MSK | DC_STOP_SRD_ACK;
	reg_write(dctrl, DC_INTC, intc);


	/* tlb init disabled. */
	dctrl_tlb_configure(dctrl);


	return ret;
}


/* 申请和释放rdma的描述符. */
static int dpu_ctrl_alloc_rdma_desc(struct dpu_ctrl *dctrl)
{
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	dma_addr_t addr_phy;
	uint8_t * addr = 0;
	int i = 0;

	dctrl->sreadesc_size = 0;
	size = sizeof(struct ingenicfb_sreadesc);

	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * CONFIG_FB_INGENIC_NR_FRAMES;
	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);

	if(IS_ERR_OR_NULL(addr)) {
		dev_err(dctrl->dev, "Failed to alloc dma desc for rdma!\n");
		return -ENOMEM;
	}

	for(i = 0; i < CONFIG_FB_INGENIC_NR_FRAMES; i++) {
		dctrl->sreadesc[i] =
			(struct ingenicfb_sreadesc *)(addr + i * size);
		dctrl->sreadesc_phys[i] = addr_phy + i * size;
	}

	dctrl->sreadesc_size = alloc_size;

	dev_info(dctrl->dev, "rdma_desc @ 0x%x size: %d", dctrl->sreadesc[0], dctrl->sreadesc_size);

	return 0;
}

static int dpu_ctrl_release_rdma_desc(struct dpu_ctrl *dctrl)
{
	if(dctrl->sreadesc_size) {
		dma_free_coherent(dctrl->dev, dctrl->sreadesc_size, dctrl->sreadesc[0], dctrl->sreadesc_phys[0]);
		dctrl->sreadesc_size = 0;
	}

	return 0;
}

/* 申请和释放composer的描述符
 * dpu控制器所拥有的描述符是固定的，当启动时，每个描述的内容需要由外部填充.
 * 例如:
 * 	1. 当用户调用composer进行合成时，可以先修改上层的内容，最后再反馈到实际的buffer.
 * */
static int dpu_ctrl_alloc_comp_desc(struct dpu_ctrl *dctrl)
{
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	dma_addr_t addr_phy = 0;
	uint8_t * addr = 0;
	int ret = 0;
	int i = 0, j = 0;

	/*Composer Frame desc. */
	dctrl->framedesc_size = 0;
	size = sizeof(struct ingenicfb_framedesc);

	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * DPU_COMP_MAX_FRAMEDESC;
	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);

	if(IS_ERR_OR_NULL(addr)) {
		dev_err(dctrl->dev, "Failed to alloc dma desc for rdma!\n");
		return -ENOMEM;
	}

	for(i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		dctrl->framedesc[i] =
			(struct ingenicfb_framedesc *)(addr + i * size);
		dctrl->framedesc_phys[i] = addr_phy + i * size;
	}
	dctrl->framedesc_size = alloc_size;

	/*Composer Layer Desc*/
	size = sizeof(struct ingenicfb_layerdesc);
	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * DPU_COMP_MAX_FRAMEDESC * DPU_SUPPORT_MAX_LAYERS;

	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);
	if(addr == NULL) {
		ret = -ENOMEM;
		goto err_layer_desc;
	}
	for(i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		for(j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
			dctrl->layerdesc[i][j] = (struct ingenicfb_layerdesc *)
				(addr + j * size + i * size * DPU_SUPPORT_MAX_LAYERS);
			dctrl->layerdesc_phys[i][j] =
				addr_phy + j * size + i * size * DPU_SUPPORT_MAX_LAYERS;
		}
	}
	dctrl->layerdesc_size = alloc_size;

	dctrl->comp_stopped = 1;
	dctrl->srd_stopped = 0;


	dev_info(dctrl->dev, "composer framedesc @ 0x%x size %d\n", dctrl->framedesc[0], dctrl->framedesc_size);
	dev_info(dctrl->dev, "composer layerdesc @ 0x%x size %d\n", dctrl->layerdesc[0][0], dctrl->layerdesc_size);
	return ret;

err_layer_desc:
	dma_free_coherent(dctrl->dev, dctrl->framedesc_size, dctrl->framedesc[0], dctrl->framedesc_phys[0]);
	return ret;
}

static int dpu_ctrl_release_comp_desc(struct dpu_ctrl *dctrl)
{
	if(dctrl->framedesc_size) {
		dma_free_coherent(dctrl->dev, dctrl->framedesc_size, dctrl->framedesc[0], dctrl->framedesc_phys[0]);
	}

	if(dctrl->layerdesc_size) {
		dma_free_coherent(dctrl->dev, dctrl->layerdesc_size, dctrl->layerdesc[0][0], dctrl->layerdesc_phys[0][0]);
	}

	return 0;
}


int dpu_ctrl_try_rdma_release(struct dpu_ctrl *dctrl)
{
	unsigned int stat = reg_read(dctrl, DC_ST);
	int ret = 0;
	/*如果当前rdma没有使用，可以尝试释放rdma申请的dma描述符资源，成功返回0.*/
	if((dctrl->srd_stopped == 1) || !(stat & DC_SRD_WORKING)) {
		dpu_ctrl_release_rdma_desc(dctrl);
	} else {
		ret = -EBUSY;
	}

	return ret;
}



/*
 *	初始化LCD相关数据结构资源. 与具体硬件的操作无关.
 *
 * */
int dpu_ctrl_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel)
{
	int ret = 0;

	dev_info(dctrl->dev, "dpu_ctrl_init start!\n");

	spin_lock_init(&dctrl->irq_lock);

	sprintf(dctrl->clk_name, "gate_lcd");
	sprintf(dctrl->pclk_name, "div_lcd");

	dctrl->clk = devm_clk_get(dctrl->dev, dctrl->clk_name);
	if(IS_ERR(dctrl->clk)) {
		ret = PTR_ERR(dctrl->clk);
		dev_err(dctrl->dev, "Failed to get lcd clk\n");

		goto err_clk;
	}
	dctrl->pclk = devm_clk_get(dctrl->dev, dctrl->pclk_name);
	if(IS_ERR(dctrl->pclk)) {
		ret = PTR_ERR(dctrl->pclk);
		dev_err(dctrl->dev, "Failed to get lcd pclk\n");

		goto err_pclk;
	}

	dctrl->base = of_iomap(dctrl->dev->of_node, 0);
	if (!dctrl->base) {
		dev_err(dctrl->dev,
				"Failed to ioremap register memory region\n");
		ret = -EBUSY;
		goto err_put_clk;
	}

	dctrl->irq = platform_get_irq(dctrl->pdev, 0);
	sprintf(dctrl->irq_name, "lcdc%d", dctrl->pdev->id);
	if (devm_request_irq(dctrl->dev, dctrl->irq, dpu_ctrl_irq_handler, 0,
				dctrl->irq_name, dctrl)) {
		dev_err(dctrl->dev, "request irq failed\n");
		ret = -EINVAL;
		goto err_irq_req;
	}


	dctrl->panel = panel;

	ingenic_set_pixclk(dctrl, dctrl->active_video_mode->pixclock);

	clk_prepare_enable(dctrl->clk);
	clk_prepare_enable(dctrl->pclk);


	ret = dpu_ctrl_alloc_rdma_desc(dctrl);
	if(ret < 0) {
		goto err_rdma_desc;
	}

	if(dctrl->support_comp) {
		ret = dpu_ctrl_alloc_comp_desc(dctrl);
		if(ret < 0) {
			goto err_comp_desc;
		}
	}

	init_waitqueue_head(&dctrl->wq);

	/*DSI 初始化会依赖pixclk.所以要先设置pixclk.**/
	if(panel->lcd_type == LCD_TYPE_MIPI_SLCD || panel->lcd_type == LCD_TYPE_MIPI_TFT){
		dpu_ctrl_comp_stop(dctrl, QCK_STOP);//关闭composer

		dctrl->dsi = jzdsi_init(panel->dsi_pdata);
		if (!dctrl->dsi) {
			ret = -EINVAL;
			goto err_iounmap;
		}
	}


	dev_info(dctrl->dev, "dpu_ctrl_init success\n");

	return 0;
err_comp_desc:
	dpu_ctrl_release_rdma_desc(dctrl);
err_rdma_desc:
	jzdsi_remove(dctrl->dsi);
err_iounmap:
	clk_disable_unprepare(dctrl->pclk);
	clk_disable_unprepare(dctrl->clk);
	free_irq(dctrl->irq, dctrl);
err_irq_req:
	iounmap(dctrl->base);
err_put_clk:
	devm_clk_put(dctrl->dev, dctrl->pclk);
err_pclk:
	devm_clk_put(dctrl->dev, dctrl->clk);
err_clk:
	return ret;
}


void dpu_ctrl_exit(struct dpu_ctrl *dctrl)
{
	dpu_ctrl_rdma_stop(dctrl, QCK_STOP);
	dpu_ctrl_comp_stop(dctrl, QCK_STOP);

	dpu_ctrl_release_rdma_desc(dctrl);
	dpu_ctrl_release_comp_desc(dctrl);
	if(dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD ||dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT){
		jzdsi_remove(dctrl->dsi);
	}
	iounmap(dctrl->base);
	free_irq(dctrl->irq, dctrl);

	devm_clk_put(dctrl->dev, dctrl->pclk);
	devm_clk_put(dctrl->dev, dctrl->clk);
}

int dpu_ctrl_suspend(struct dpu_ctrl *dctrl)
{

	if(dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD || dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT){
		dctrl->dsi->master_ops->set_blank_mode(dctrl->dsi, FB_BLANK_POWERDOWN);
	}
	//TODO: Lock??
	if(dctrl->chan == DATA_CH_RDMA) {
		dpu_ctrl_rdma_stop(dctrl, QCK_STOP);
	} else {
		dpu_ctrl_comp_stop(dctrl, QCK_STOP);
	}

	clk_disable_unprepare(dctrl->clk);
	clk_disable_unprepare(dctrl->pclk);

	return 0;
}

int dpu_ctrl_resume(struct dpu_ctrl *dctrl)
{
	clk_prepare_enable(dctrl->pclk);
	clk_prepare_enable(dctrl->clk);

	/*!! dpu must not enabled before dsi UNBLANK.*/
	if(dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD || dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT){
		dctrl->dsi->master_ops->set_blank_mode(dctrl->dsi, FB_BLANK_UNBLANK);
	}

	//TODO: lock??
	dpu_ctrl_setup(dctrl);

	if(dctrl->chan == DATA_CH_RDMA) {
		dpu_ctrl_rdma_setup(dctrl, &dctrl->rdma_info);
		dpu_ctrl_rdma_start(dctrl);
	} else {
		dpu_ctrl_comp_setup(dctrl, &dctrl->comp_info);
		dpu_ctrl_comp_start(dctrl, 0);
	}

	return 0;
}


