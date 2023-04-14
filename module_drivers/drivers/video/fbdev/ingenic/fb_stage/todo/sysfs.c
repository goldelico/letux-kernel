
static void dump_dc_reg(void)
{
	printk("-----------------dc_reg------------------\n");
	printk("DC_FRM_CFG_ADDR(0x%04x):    0x%08lx\n", DC_FRM_CFG_ADDR,reg_read(fbdev, DC_FRM_CFG_ADDR));
	printk("DC_FRM_CFG_CTRL(0x%04x):    0x%08lx\n", DC_FRM_CFG_CTRL,reg_read(fbdev, DC_FRM_CFG_CTRL));
	printk("DC_CTRL(0x%04x):            0x%08lx\n", DC_CTRL, reg_read(fbdev, DC_CTRL));
	printk("DC_CSC_MULT_YRV(0x%04x):    0x%08lx\n", DC_CSC_MULT_YRV,reg_read(fbdev, DC_CSC_MULT_YRV));
	printk("DC_CSC_MULT_GUGV(0x%04x):   0x%08lx\n", DC_CSC_MULT_GUGV,reg_read(fbdev, DC_CSC_MULT_GUGV));
	printk("DC_CSC_MULT_BU(0x%04x):     0x%08lx\n", DC_CSC_MULT_BU,reg_read(fbdev, DC_CSC_MULT_BU));
	printk("DC_CSC_SUB_YUV(0x%04x):     0x%08lx\n", DC_CSC_SUB_YUV,reg_read(fbdev, DC_CSC_SUB_YUV));
	printk("DC_ST(0x%04x):              0x%08lx\n", DC_ST, reg_read(fbdev, DC_ST));
	printk("DC_INTC(0x%04x):            0x%08lx\n", DC_INTC, reg_read(fbdev, DC_INTC));
	printk("DC_INT_FLAG(0x%04x):	   0x%08lx\n", DC_INT_FLAG, reg_read(fbdev, DC_INT_FLAG));
	printk("DC_COM_CONFIG(0x%04x):      0x%08lx\n", DC_COM_CONFIG ,reg_read(fbdev, DC_COM_CONFIG));
	printk("DC_TLB_GLBC(0x%04x):        0x%08lx\n", DC_TLB_GLBC,reg_read(fbdev, DC_TLB_GLBC));
	printk("DC_TLB_TLBA(0x%04x):        0x%08lx\n", DC_TLB_TLBA,reg_read(fbdev, DC_TLB_TLBA));
	printk("DC_TLB_TLBC(0x%04x):        0x%08lx\n", DC_TLB_TLBC,reg_read(fbdev, DC_TLB_TLBC));
	printk("DC_TLB0_VPN(0x%04x):        0x%08lx\n", DC_TLB0_VPN,reg_read(fbdev, DC_TLB0_VPN));
	printk("DC_TLB1_VPN(0x%04x):        0x%08lx\n", DC_TLB1_VPN,reg_read(fbdev, DC_TLB1_VPN));
	printk("DC_TLB2_VPN(0x%04x):        0x%08lx\n", DC_TLB2_VPN,reg_read(fbdev, DC_TLB2_VPN));
	printk("DC_TLB3_VPN(0x%04x):        0x%08lx\n", DC_TLB3_VPN,reg_read(fbdev, DC_TLB3_VPN));
	printk("DC_TLB_TLBV(0x%04x):        0x%08lx\n", DC_TLB_TLBV,reg_read(fbdev, DC_TLB_TLBV));
	printk("DC_TLB_STAT(0x%04x):        0x%08lx\n", DC_TLB_STAT,reg_read(fbdev, DC_TLB_STAT));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL,reg_read(fbdev, DC_PCFG_RD_CTRL));
	printk("DC_PCFG_WR_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_WR_CTRL,reg_read(fbdev, DC_PCFG_WR_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG,reg_read(fbdev, DC_OFIFO_PCFG));
	printk("DC_WDMA_PCFG(0x%04x):	   0x%08lx\n", DC_WDMA_PCFG,reg_read(fbdev, DC_WDMA_PCFG));
	printk("DC_CMPW_PCFG_CTRL(0x%04x): 0x%08lx\n", DC_CMPW_PCFG_CTRL,reg_read(fbdev, DC_CMPW_PCFG_CTRL));
	printk("DC_CMPW_PCFG0(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG0,reg_read(fbdev, DC_CMPW_PCFG0));
	printk("DC_CMPW_PCFG1(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG1,reg_read(fbdev, DC_CMPW_PCFG1));
	printk("DC_CMPW_PCFG2(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG2,reg_read(fbdev, DC_CMPW_PCFG2));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL,reg_read(fbdev, DC_PCFG_RD_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG,reg_read(fbdev, DC_OFIFO_PCFG));
	printk("DC_DISP_COM(0x%04x):        0x%08lx\n", DC_DISP_COM,reg_read(fbdev, DC_DISP_COM));
	printk("-----------------dc_reg------------------\n");
}

static void dump_tft_reg(void)
{
	printk("----------------tft_reg------------------\n");
	printk("TFT_TIMING_HSYNC(0x%04x):   0x%08lx\n", DC_TFT_HSYNC, reg_read(fbdev, DC_TFT_HSYNC));
	printk("TFT_TIMING_VSYNC(0x%04x):   0x%08lx\n", DC_TFT_VSYNC, reg_read(fbdev, DC_TFT_VSYNC));
	printk("TFT_TIMING_HDE(0x%04x):     0x%08lx\n", DC_TFT_HDE, reg_read(fbdev, DC_TFT_HDE));
	printk("TFT_TIMING_VDE(0x%04x):     0x%08lx\n", DC_TFT_VDE, reg_read(fbdev, DC_TFT_VDE));
	printk("TFT_TRAN_CFG(0x%04x):       0x%08lx\n", DC_TFT_CFG, reg_read(fbdev, DC_TFT_CFG));
	printk("TFT_ST(0x%04x):             0x%08lx\n", DC_TFT_ST, reg_read(fbdev, DC_TFT_ST));
	printk("----------------tft_reg------------------\n");
}

static void dump_slcd_reg(void)
{
	printk("---------------slcd_reg------------------\n");
	printk("SLCD_CFG(0x%04x):           0x%08lx\n", DC_SLCD_CFG, reg_read(fbdev, DC_SLCD_CFG));
	printk("SLCD_WR_DUTY(0x%04x):       0x%08lx\n", DC_SLCD_WR_DUTY, reg_read(fbdev, DC_SLCD_WR_DUTY));
	printk("SLCD_TIMING(0x%04x):        0x%08lx\n", DC_SLCD_TIMING, reg_read(fbdev, DC_SLCD_TIMING));
	printk("SLCD_FRM_SIZE(0x%04x):      0x%08lx\n", DC_SLCD_FRM_SIZE, reg_read(fbdev, DC_SLCD_FRM_SIZE));
	printk("SLCD_SLOW_TIME(0x%04x):     0x%08lx\n", DC_SLCD_SLOW_TIME, reg_read(fbdev, DC_SLCD_SLOW_TIME));
	printk("SLCD_REG_IF(0x%04x):	    0x%08lx\n", DC_SLCD_REG_IF, reg_read(fbdev, DC_SLCD_REG_IF));
	printk("SLCD_ST(0x%04x):            0x%08lx\n", DC_SLCD_ST, reg_read(fbdev, DC_SLCD_ST));
	printk("---------------slcd_reg------------------\n");
}

static void dump_frm_desc_reg(void)
{
	unsigned int ctrl;
	ctrl = reg_read(fbdev, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(fbdev, DC_CTRL, ctrl);

	printk("--------Frame Descriptor register--------\n");
	printk("FrameNextCfgAddr:   %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("FrameSize:          %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("FrameCtrl:          %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("WritebackAddr:      %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("WritebackStride:    %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("Layer0CfgAddr:      %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("Layer1CfgAddr:      %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("Layer2CfgAddr:      %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("Layer3CfgAddr:      %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("LayCfgEn:	    %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("InterruptControl:   %lx\n",reg_read(fbdev, DC_FRM_DES));
	printk("--------Frame Descriptor register--------\n");
}

static void dump_layer_desc_reg(void)
{
	unsigned int ctrl;
	ctrl = reg_read(fbdev, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(fbdev, DC_CTRL, ctrl);

	printk("--------layer0 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerCfg:           %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerScale:         %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerRotation:      %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerScratch:       %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerPos:           %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("LayerStride:        %lx\n",reg_read(fbdev, DC_LAY0_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(fbdev, DC_LAY0_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(fbdev, DC_LAY0_DES));
	printk("--------layer0 Descriptor register-------\n");

	printk("--------layer1 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerCfg:           %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerScale:         %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerRotation:      %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerScratch:       %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerPos:           %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("LayerStride:        %lx\n",reg_read(fbdev, DC_LAY1_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(fbdev, DC_LAY1_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(fbdev, DC_LAY1_DES));
	printk("--------layer1 Descriptor register-------\n");

	printk("--------layer2 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerCfg:           %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerScale:         %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerRotation:      %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerScratch:       %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerPos:           %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("LayerStride:        %lx\n",reg_read(fbdev, DC_LAY2_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(fbdev, DC_LAY2_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(fbdev, DC_LAY2_DES));
	printk("--------layer2 Descriptor register-------\n");

	printk("--------layer3 Descriptor register-------\n");
	printk("LayerSize:          %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerCfg:           %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerBufferAddr:    %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerScale:         %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerRotation:      %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerScratch:       %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerPos:           %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerResizeCoef_X:  %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerResizeCoef_Y:  %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("LayerStride:        %lx\n",reg_read(fbdev, DC_LAY3_DES));
//	printk("Layer_UV_addr:	    %lx\n",reg_read(fbdev, DC_LAY3_DES));
//	printk("Layer_UV_stride:    %lx\n",reg_read(fbdev, DC_LAY3_DES));
	printk("--------layer3 Descriptor register-------\n");
}

static void dump_rdma_desc_reg(void)
{
	unsigned int ctrl;
	  ctrl = reg_read(fbdev, DC_CTRL);
	  ctrl |= (1 << 2);
	  reg_write(fbdev, DC_CTRL, ctrl);
	printk("====================rdma Descriptor register======================\n");
	printk("RdmaNextCfgAddr:    %lx\n",reg_read(fbdev, DC_RDMA_DES));
	printk("FrameBufferAddr:    %lx\n",reg_read(fbdev, DC_RDMA_DES));
	printk("Stride:             %lx\n",reg_read(fbdev, DC_RDMA_DES));
	printk("ChainCfg:           %lx\n",reg_read(fbdev, DC_RDMA_DES));
	printk("InterruptControl:   %lx\n",reg_read(fbdev, DC_RDMA_DES));
	printk("==================rdma Descriptor register end======================\n");
}


static void dump_frm_desc(struct ingenicfb_framedesc *framedesc, int index)
{
	printk("-------User Frame Descriptor index[%d]-----\n", index);
	printk("FramedescAddr:	    0x%x\n",(uint32_t)framedesc);
	printk("FrameNextCfgAddr:   0x%x\n",framedesc->FrameNextCfgAddr);
	printk("FrameSize:          0x%x\n",framedesc->FrameSize.d32);
	printk("FrameCtrl:          0x%x\n",framedesc->FrameCtrl.d32);
	printk("Layer0CfgAddr:      0x%x\n",framedesc->Layer0CfgAddr);
	printk("Layer1CfgAddr:      0x%x\n",framedesc->Layer1CfgAddr);
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
//	printk("Layer_UV_addr:	    0x%x\n",layer_config->BufferAddr_UV);
//	printk("Layer_UV_stride:    0x%x\n",layer_config->stride_UV);
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

static void dump_lcdc_registers(void)
{
	dump_dc_reg();
	dump_tft_reg();
	dump_slcd_reg();
	dump_frm_desc_reg();
	dump_layer_desc_reg();
	dump_rdma_desc_reg();
}

static void dump_desc(struct ingenicfb_device *fbdev)
{
	int i, j;
	for(i = 0; i < CONFIG_FB_INGENIC_NR_FRAMES*2; i++) {
		for(j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
			dump_layer_desc(fbdev->layerdesc[i][j], i, j);
		}
		dump_frm_desc(fbdev->framedesc[i], i);
	}
}

	static ssize_t
dump_lcd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	printk("\nDisp_end_num = %d\n\n",fbdev->irq_cnt);
	printk("\nTFT_UNDR_num = %d\n\n",fbdev->tft_undr_cnt);
	printk("\nFrm_start_num = %d\n\n", fbdev->frm_start);
	printk(" Pand display count=%d\n",fbdev->pan_display_count);
	printk("timestamp.wp = %d , timestamp.rp = %d\n\n", fbdev->timestamp.wp, fbdev->timestamp.rp);
	dump_lcdc_registers();
	dump_desc(fbdev);
	return 0;
}

static void dump_all(struct ingenicfb_device *fbdev)
{
	printk("\ndisp_end_num = %d\n\n", fbdev->irq_cnt);
	printk("\nTFT_UNDR_num = %d\n\n", fbdev->tft_undr_cnt);
	printk("\nFrm_start_num = %d\n\n", fbdev->frm_start);
	dump_lcdc_registers();
	dump_desc(fbdev);
}

#ifdef CONFIG_DEBUG_DPU_IRQCNT
	static ssize_t
dump_irqcnts(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dbg_irqcnt *dbg = &fbdev->dbg_irqcnt;
	char *p = buf;
	p += sprintf(p, "irqcnt		: %lld\n", dbg->irqcnt);
	p += sprintf(p, "cmp_start	: %lld\n", dbg->cmp_start);
	p += sprintf(p, "stop_disp_ack	: %lld\n", dbg->stop_disp_ack);
	p += sprintf(p, "disp_end	: %lld\n", dbg->disp_end);
	p += sprintf(p, "tft_under	: %lld\n", dbg->tft_under);
	p += sprintf(p, "wdma_over	: %lld\n", dbg->wdma_over);
	p += sprintf(p, "wdma_end	: %lld\n", dbg->wdma_end);
	p += sprintf(p, "layer3_end	: %lld\n", dbg->layer3_end);
	p += sprintf(p, "layer2_end	: %lld\n", dbg->layer2_end);
	p += sprintf(p, "layer1_end	: %lld\n", dbg->layer1_end);
	p += sprintf(p, "layer0_end	: %lld\n", dbg->layer0_end);
	p += sprintf(p, "clr_cmp_end	: %lld\n", dbg->clr_cmp_end);
	p += sprintf(p, "stop_wrbk_ack	: %lld\n", dbg->stop_wrbk_ack);
	p += sprintf(p, "srd_start	: %lld\n", dbg->srd_start);
	p += sprintf(p, "srd_end	: %lld\n", dbg->srd_end);
	p += sprintf(p, "cmp_w_slow	: %lld\n", dbg->cmp_w_slow);

	return p - buf;
}
#endif

/*Sysfs interface.*/
	static ssize_t
dump_h_color_bar(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	ingenicfb_display_h_color_bar(fbdev->fb);
	ingenicfb_cmp_start(fbdev->fb);
	return 0;
}

	static ssize_t
dump_v_color_bar(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	ingenicfb_display_v_color_bar(fbdev->fb);
	ingenicfb_cmp_start(fbdev->fb);
	return 0;
}

	static ssize_t
vsync_skip_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	mutex_lock(&fbdev->lock);
	snprintf(buf, 3, "%d\n", fbdev->vsync_skip_ratio);
	printk("vsync_skip_map = 0x%08x\n", fbdev->vsync_skip_map);
	mutex_unlock(&fbdev->lock);
	return 3;		/* sizeof ("%d\n") */
}

static int vsync_skip_set(struct ingenicfb_device *fbdev, int vsync_skip)
{
	unsigned int map_wide10 = 0;
	int rate, i, p, n;
	int fake_float_1k;

	if (vsync_skip < 0 || vsync_skip > 9)
		return -EINVAL;

	rate = vsync_skip + 1;
	fake_float_1k = 10000 / rate;	/* 10.0 / rate */

	p = 1;
	n = (fake_float_1k * p + 500) / 1000;	/* +0.5 to int */

	for (i = 1; i <= 10; i++) {
		map_wide10 = map_wide10 << 1;
		if (i == n) {
			map_wide10++;
			p++;
			n = (fake_float_1k * p + 500) / 1000;
		}
	}
	mutex_lock(&fbdev->lock);
	fbdev->vsync_skip_map = map_wide10;
	fbdev->vsync_skip_ratio = rate - 1;	/* 0 ~ 9 */
	mutex_unlock(&fbdev->lock);

	printk("vsync_skip_ratio = %d\n", fbdev->vsync_skip_ratio);
	printk("vsync_skip_map = 0x%08x\n", fbdev->vsync_skip_map);

	return 0;
}

	static ssize_t
vsync_skip_w(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	if ((count != 1) && (count != 2))
		return -EINVAL;
	if ((*buf < '0') && (*buf > '9'))
		return -EINVAL;

	vsync_skip_set(fbdev, *buf - '0');

	return count;
}

static ssize_t fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("\n-----you can choice print way:\n");
	printk("Example: echo NUM > show_fps\n");
	printk("NUM = 0: close fps statistics\n");
	printk("NUM = 1: print recently fps\n");
	printk("NUM = 2: print interval between last and this pan_display\n");
	printk("NUM = 3: print pan_display count\n\n");
	return 0;
}

static ssize_t fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int num = 0;
	num = simple_strtoul(buf, NULL, 0);
	if(num < 0){
		printk("\n--please 'cat show_fps' to view using the method\n\n");
		return n;
	}
	showFPS = num;
	if(showFPS == 3)
		printk(KERN_DEBUG " Pand display count=%d\n",fbdev->pan_display_count);
	return n;
}


	static ssize_t
debug_clr_st(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	reg_write(fbdev, DC_CLR_ST, 0xffffffff);
	return 0;
}

static ssize_t test_suspend(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int num = 0;
	num = simple_strtoul(buf, NULL, 0);
	printk("0 --> resume | 1 --> suspend\nnow input %d\n", num);
	if (num == 0) {
		ingenicfb_do_resume(fbdev);
	} else {
		ingenicfb_do_suspend(fbdev);
	}
	return n;
}

/*************************self test******************************/
static ssize_t test_slcd_send_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int num = 0;
	num = simple_strtoul(buf, NULL, 0);
	if (num == 0) {
		slcd_send_mcu_command(fbdev, 0x0);
		slcd_send_mcu_command(fbdev, 0xffffff);
		slcd_send_mcu_command(fbdev, 0x0);
		slcd_send_mcu_command(fbdev, 0xffffff);
		slcd_send_mcu_command(fbdev, 0x0);
		slcd_send_mcu_command(fbdev, 0xffffff);
		slcd_send_mcu_command(fbdev, 0x0);
		slcd_send_mcu_command(fbdev, 0xffffff);
		slcd_send_mcu_command(fbdev, 0x0);
		slcd_send_mcu_command(fbdev, 0xffffff);
		printk("Send command value 101\n");
	} else if(num == 1){
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		slcd_send_mcu_prm(fbdev, 0x0);
		slcd_send_mcu_prm(fbdev, 0xffffffff);
		printk("Send prm value 0x101\n");
	} else {
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		slcd_send_mcu_data(fbdev, 0x0);
		slcd_send_mcu_data(fbdev, 0xffffffff);
		printk("Send data value 0x101\n");
	}
	return n;
}

static ssize_t test_fifo_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int num = 0;
	unsigned int ctrl;
	unsigned int cfg = 0;
	num = simple_strtoul(buf, NULL, 0);
	if (num == 0) {
		ctrl = reg_read(fbdev, DC_PCFG_RD_CTRL);
		ctrl &=  ~DC_ARQOS_CTRL;
		reg_write(fbdev, DC_PCFG_RD_CTRL, ctrl);
		printk("Close software control %d\n", num);
	} else {
		ctrl = reg_read(fbdev, DC_PCFG_RD_CTRL);
		ctrl |=  DC_ARQOS_CTRL;
		ctrl &= ~DC_ARQOS_VAL_MASK;
		switch(num) {
			case 1:
				ctrl |= DC_ARQOS_VAL_0;
				break;
			case 2:
				ctrl |= DC_ARQOS_VAL_1;
				break;
			case 3:
				ctrl |= DC_ARQOS_VAL_2;
				break;
			case 4:
				ctrl |= DC_ARQOS_VAL_3;
				break;
			default:
				printk("Input err value %d\n", num);
				return n;
		}
		reg_write(fbdev, DC_PCFG_RD_CTRL, ctrl);
		cfg = (60 << DC_PCFG0_LBIT) | (120 << DC_PCFG1_LBIT) | (180 << DC_PCFG2_LBIT);
		reg_write(fbdev, DC_OFIFO_PCFG, cfg);
		printk("DC_OFIFO_PCFG = 0x%x  DC_PCFG_RD_CTRL = 0x%x\n", DC_OFIFO_PCFG, DC_PCFG_RD_CTRL);
	}
	return n;
}
static ssize_t test_slcd_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned int num = 0;
	unsigned int type = 0;
	unsigned int value;
	num = simple_strtoul(buf, NULL, 0);

	type = (num >> 8) & 0xff;
	switch(type) {
		case 0:
			printk("\n0: print set ways\n");
			printk("1: CDTIME\n");
			printk("2: CSTIME\n");
			printk("3: DDTIME\n");
			printk("4: DSTIME\n");
			printk("5: TAS\n");
			printk("6: TAH\n");
			printk("7: TCS\n");
			printk("8: TCH\n");
			printk("9: Slow time\n\n");
			break;
		case 1:
			value = reg_read(fbdev, DC_SLCD_WR_DUTY);
			reg_write(fbdev, DC_SLCD_WR_DUTY, ((num & 0xff) << DC_CDTIME_LBIT) | (value & ~DC_CDTIME_MASK));
			break;
		case 2:
			value = reg_read(fbdev, DC_SLCD_WR_DUTY);
			reg_write(fbdev, DC_SLCD_WR_DUTY, ((num & 0xff) << DC_CSTIME_LBIT) | (value & ~DC_CSTIME_MASK));
			break;
		case 3:
			value = reg_read(fbdev, DC_SLCD_WR_DUTY);
			reg_write(fbdev, DC_SLCD_WR_DUTY, ((num & 0xff) << DC_DDTIME_LBIT) | (value & ~DC_DDTIME_MASK));
			break;
		case 4:
			value = reg_read(fbdev, DC_SLCD_WR_DUTY);
			reg_write(fbdev, DC_SLCD_WR_DUTY, ((num & 0xff) << DC_DSTIME_LBIT) | (value & ~DC_DSTIME_MASK));
			break;
		case 5:
			value = reg_read(fbdev, DC_SLCD_TIMING);
			reg_write(fbdev, DC_SLCD_TIMING, ((num & 0xff) << DC_TAS_LBIT) | (value & ~DC_TAS_MASK));
			break;
		case 6:
			value = reg_read(fbdev, DC_SLCD_TIMING);
			reg_write(fbdev, DC_SLCD_TIMING, ((num & 0xff) << DC_TAH_LBIT) | (value & ~DC_TAH_MASK));
			break;
		case 7:
			value = reg_read(fbdev, DC_SLCD_TIMING);
			reg_write(fbdev, DC_SLCD_TIMING, ((num & 0xff) << DC_TCS_LBIT) | (value & ~DC_TCS_MASK));
			break;
		case 8:
			value = reg_read(fbdev, DC_SLCD_TIMING);
			reg_write(fbdev, DC_SLCD_TIMING, ((num & 0xff) << DC_TCH_LBIT) | (value & ~DC_TCH_MASK));
			break;
		case 9:
			value = reg_read(fbdev, DC_SLCD_SLOW_TIME);
			reg_write(fbdev, DC_SLCD_SLOW_TIME, ((num & 0xff) << DC_SLOW_TIME_LBIT) | (value & ~DC_SLOW_TIME_MASK));
			break;
		default:
			printk("Input err value %d\n", num);
			return n;
	}
	printk("DC_SLCD_WR_DUTY = 0x%lx\n", reg_read(fbdev, DC_SLCD_WR_DUTY));
	printk("DC_SLCD_TIMING = 0x%lx\n", reg_read(fbdev, DC_SLCD_TIMING));
	printk("DC_SLCD_SLOW_TIME = 0x%lx\n", reg_read(fbdev, DC_SLCD_SLOW_TIME));
	return n;
}
static ssize_t show_irq_msg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	p += printk(p, "0 -> set: all irq\n");
	p += printk(p, "1 -> set: DC_CMP_W_SLOW\n");
	p += printk(p, "2 -> set: DC_DISP_END\n");
	p += printk(p, "3 -> set: DC_WDMA_OVER\n");
	p += printk(p, "4 -> set: DC_WDMA_END\n");
	p += printk(p, "5 -> set: DC_CMP_START\n");
	p += printk(p, "6 -> set: DC_LAY3_END\n");
	p += printk(p, "7 -> set: DC_LAY2_END\n");
	p += printk(p, "8 -> set: DC_LAY1_END\n");
	p += printk(p, "9 -> set: DC_LAY0_END\n");
	p += printk(p, "10 -> set: DC_CMP_END\n");
	p += printk(p, "11 -> set: DC_TFT_UNDR\n");
	p += printk(p, "12 -> set: DC_STOP_WRBK_ACK\n");
	p += printk(p, "13 -> set: DC_STOP_DISP_ACK\n");
	p += printk(p, "14 -> set: DC_SRD_START\n");
	p += printk(p, "15 -> set: DC_SRD_END\n");

	return p - buf;
}
static ssize_t set_irq_bit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int num = 0;
	unsigned int irq;
	num = simple_strtoul(buf, NULL, 0);
	irq = reg_read(fbdev, DC_INTC);
	switch(num) {
		case 0:
			reg_write(fbdev, DC_INTC, 0x820f46);
			break;
		case 1:
			reg_write(fbdev, DC_INTC, DC_CMP_W_SLOW | irq);
			break;
		case 2:
			reg_write(fbdev, DC_INTC, DC_DISP_END | irq);
			break;
		case 3:
			reg_write(fbdev, DC_INTC, DC_WDMA_OVER | irq);
			break;
		case 4:
			reg_write(fbdev, DC_INTC, DC_WDMA_END | irq);
			break;
		case 5:
			reg_write(fbdev, DC_INTC, DC_CMP_START | irq);
			break;
		case 6:
			reg_write(fbdev, DC_INTC, DC_LAY3_END | irq);
			break;
		case 7:
			reg_write(fbdev, DC_INTC, DC_LAY2_END | irq);
			break;
		case 8:
			reg_write(fbdev, DC_INTC, DC_LAY1_END | irq);
			break;
		case 9:
			reg_write(fbdev, DC_INTC, DC_LAY0_END | irq);
			break;
		case 10:
			reg_write(fbdev, DC_INTC, DC_CMP_END | irq);
			break;
		case 11:
			reg_write(fbdev, DC_INTC, DC_TFT_UNDR | irq);
			break;
		case 12:
			reg_write(fbdev, DC_INTC, DC_STOP_WRBK_ACK | irq);
			break;
		case 13:
			reg_write(fbdev, DC_INTC, DC_STOP_DISP_ACK | irq);
			break;
		case 14:
			reg_write(fbdev, DC_INTC, DC_SRD_START | irq);
			break;
		case 15:
			reg_write(fbdev, DC_INTC, DC_SRD_END | irq);
			break;
		default:
			printk("Err: Please check num = %d", num);
			reg_write(fbdev, DC_INTC, DC_CMP_START);
			break;
	}

	return n;
}

#ifdef CONFIG_FB_INGENIC_MIPI_DSI
	static ssize_t
dump_dsi(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct dsi_device *dsi = fbdev->dsi;

	mutex_lock(&fbdev->lock);
	dump_dsi_reg(dsi);
	mutex_unlock(&fbdev->lock);
	return 0;
}

	static ssize_t
dsi_query_te(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct dsi_device *dsi = fbdev->dsi;

	mutex_lock(&fbdev->lock);
	fbdev->dsi->master_ops->query_te(dsi);
	mutex_unlock(&fbdev->lock);
	return 0;
}
#endif


	static ssize_t
test_fb_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	ingenicfb_disable(fbdev->fb, GEN_STOP);
	return 0;
}

	static ssize_t
test_fb_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	ingenicfb_enable(fbdev->fb);
	return 0;
}

static ssize_t show_color_modes(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct ingenicfb_colormode *current_mode = NULL;
	char *p = buf;
	int i = 0;

	p += sprintf(p, "supported color modes:\n");
	for(i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); i++) {
		struct ingenicfb_colormode * m = &ingenicfb_colormodes[i];
		p += sprintf(p, "[%d]:%s\n", i, m->name);

		if(m->mode == fbdev->current_frm_mode.frm_cfg.lay_cfg[0].format) {
			current_mode = m;
		}
	}
	p += sprintf(p, "Current color mode: [%s]\n", current_mode ? current_mode->name:"none");
	p += sprintf(p, "Tips: echo [%s] to select one of supported color modes\n", current_mode ? current_mode->name:"none");

	return p - buf;

}

static ssize_t store_color_modes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct fb_info *fb = fbdev->fb;
	struct ingenicfb_colormode *m = NULL;
	int index = 0;
	int ret = 0;

	index = simple_strtol(buf, NULL, 10);

	if(index < 0 && index > ARRAY_SIZE(ingenicfb_colormodes))
		return -EINVAL;

	m = &ingenicfb_colormodes[index];

	/* modify fb var. */
	ingenicfb_colormode_to_var(&fb->var, m);

	/* reset params. */
	ret = ingenicfb_set_par(fbdev->fb);
	if(ret < 0)
		return ret;

	return n;
}
static ssize_t show_wback_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "help: write [1/0] to [en/dis] wback\n");
	p += sprintf(p, "wback_en: %d\n", fbdev->wback_en);

	return p - buf;
}

static ssize_t store_wback_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int en = 0;
	int ret = 0;

	en = simple_strtol(buf, NULL, 10);

	fbdev->wback_en = !!en;

	/* reset params. */
	ret = ingenicfb_set_par(fbdev->fb);
	if(ret < 0)
		return ret;

	return n;
}

static ssize_t show_csc_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "current csc_mode: %d\n", fbdev->csc_mode);

	return p - buf;
}

static ssize_t store_csc_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int csc_mode = 0;
	int ret = 0;

	csc_mode = simple_strtol(buf, NULL, 10);

	if(csc_mode < 0 || csc_mode > 3) {
		return -EINVAL;
	}

	csc_mode_set(fbdev, csc_mode);
	fbdev->csc_mode = csc_mode;

	/* reset params. */
	ret = ingenicfb_set_par(fbdev->fb);
	if(ret < 0)
		return ret;

	return n;
}

static ssize_t show_wbackbuf(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);

	char *wbackbuf = fbdev->sread_vidmem[0];	/* TODO: where should wback addr is?*/
	unsigned int wbackbuf_len = fbdev->frm_size;
	int copy_len = wbackbuf_len < 1024 ? wbackbuf_len : 1024; /* only copy first 1024 bytes for quik debug.*/

	if(fbdev->wback_en == 0) {
		return -EINVAL;
	}

	print_hex_dump(KERN_INFO, "wback data: ", DUMP_PREFIX_ADDRESS, 16, 1, wbackbuf, 128, true);

	memcpy(buf, wbackbuf, copy_len);

	return copy_len;
}

/*************************self test******************************/

/**********************lcd_debug***************************/
static DEVICE_ATTR(dump_lcd, S_IRUGO|S_IWUSR, dump_lcd, NULL);
static DEVICE_ATTR(dump_h_color_bar, S_IRUGO|S_IWUSR, dump_h_color_bar, NULL);
static DEVICE_ATTR(dump_v_color_bar, S_IRUGO|S_IWUSR, dump_v_color_bar, NULL);
static DEVICE_ATTR(vsync_skip, S_IRUGO|S_IWUSR, vsync_skip_r, vsync_skip_w);
static DEVICE_ATTR(show_fps, S_IRUGO|S_IWUSR, fps_show, fps_store);
static DEVICE_ATTR(debug_clr_st, S_IRUGO|S_IWUSR, debug_clr_st, NULL);
static DEVICE_ATTR(test_suspend, S_IRUGO|S_IWUSR, NULL, test_suspend);

#ifdef CONFIG_DEBUG_DPU_IRQCNT
static DEVICE_ATTR(dump_irqcnts, (S_IRUGO|S_IWUGO) & (~S_IWOTH), dump_irqcnts, NULL);
#endif

#ifdef CONFIG_FB_INGENIC_MIPI_DSI
static DEVICE_ATTR(dump_dsi, S_IRUGO|S_IWUSR, dump_dsi, NULL);
static DEVICE_ATTR(dsi_query_te, S_IRUGO|S_IWUSR, dsi_query_te, NULL);
#endif
static DEVICE_ATTR(test_fb_disable, (S_IRUGO|S_IWUGO)&(~S_IWOTH), test_fb_disable, NULL);
static DEVICE_ATTR(test_fb_enable, (S_IRUGO|S_IWUGO)&(~S_IWOTH) , test_fb_enable, NULL);

static DEVICE_ATTR(test_irq, S_IRUGO|S_IWUSR, show_irq_msg, set_irq_bit);
static DEVICE_ATTR(test_fifo_threshold, S_IRUGO|S_IWUSR, NULL, test_fifo_threshold);
static DEVICE_ATTR(test_slcd_time, S_IRUGO|S_IWUSR, NULL, test_slcd_time);
static DEVICE_ATTR(test_slcd_send_value, S_IRUGO|S_IWUSR, NULL, test_slcd_send_value);
static DEVICE_ATTR(color_modes, S_IRUGO|S_IWUSR, show_color_modes, store_color_modes);
static DEVICE_ATTR(wback_en, S_IRUGO|S_IWUSR, show_wback_en, store_wback_en);
static DEVICE_ATTR(wbackbuf, S_IRUGO|S_IWUSR, show_wbackbuf, NULL);
static DEVICE_ATTR(csc_mode, S_IRUGO|S_IWUSR, show_csc_mode, store_csc_mode);


static struct attribute *lcd_debug_attrs[] = {
	&dev_attr_dump_lcd.attr,
	&dev_attr_dump_h_color_bar.attr,
	&dev_attr_dump_v_color_bar.attr,
	&dev_attr_vsync_skip.attr,
	&dev_attr_show_fps.attr,
	&dev_attr_debug_clr_st.attr,
	&dev_attr_test_suspend.attr,

#ifdef CONFIG_DEBUG_DPU_IRQCNT
	&dev_attr_dump_irqcnts.attr,
#endif

#ifdef CONFIG_FB_INGENIC_MIPI_DSI
	&dev_attr_dump_dsi.attr,
	&dev_attr_dsi_query_te.attr,
#endif
	&dev_attr_test_fb_disable.attr,
	&dev_attr_test_fb_enable.attr,
	&dev_attr_test_irq.attr,
	&dev_attr_test_fifo_threshold.attr,
	&dev_attr_test_slcd_time.attr,
	&dev_attr_test_slcd_send_value.attr,
	&dev_attr_color_modes.attr,
	&dev_attr_wback_en.attr,
	&dev_attr_wbackbuf.attr,
	&dev_attr_csc_mode.attr,
	NULL,
};

const char lcd_group_name[] = "debug";
static struct attribute_group lcd_debug_attr_group = {
	.name	= lcd_group_name,
	.attrs	= lcd_debug_attrs,
};

struct layer_device_attr {
	struct device_attribute attr;
	unsigned int id;
};

#define to_layer_attr(attr)	\
	container_of(attr, struct layer_device_attr, attr)


static ssize_t show_src_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p = buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}
	lay_cfg = &frm_cfg->lay_cfg[layer];

	p += sprintf(p, "layer: %d, src_w: %d, src_h: %d\n", layer, lay_cfg->source_w, lay_cfg->source_h);

	return p - buf;
}
static ssize_t store_src_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{

	return n;
}
static ssize_t show_src_fmt(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct ingenicfb_colormode *current_mode = NULL;
	int layer = to_layer_attr(attr)->id;
	char *p = buf;
	int i = 0;

	p += sprintf(p, "supported color modes:\n");
	for(i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); i++) {
		struct ingenicfb_colormode * m = &ingenicfb_colormodes[i];
		p += sprintf(p, "[%d]:%s\n", i, m->name);

		if(m->mode == fbdev->current_frm_mode.frm_cfg.lay_cfg[layer].format) {
			current_mode = m;
		}
	}
	p += sprintf(p, "Current color mode: [%s]\n", current_mode ? current_mode->name:"none");
	p += sprintf(p, "Tips: echo [%s] to select one of supported color modes\n", current_mode ? current_mode->name:"none");

	return p - buf;
}
static ssize_t store_src_fmt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct fb_info *fb = fbdev->fb;
	struct ingenicfb_colormode *m = NULL;
	int index = 0;
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;

	index = simple_strtol(buf, NULL, 10);

	if(index < 0 && index > ARRAY_SIZE(ingenicfb_colormodes))
		return -EINVAL;

	m = &ingenicfb_colormodes[index];

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];
	lay_cfg->format = m->mode;

	/*update fb_info.*/
	fb = fbdev->fbs[layer];
	ingenicfb_colormode_to_var(&fb->var, m);
	fb->mode = ingenicfb_get_mode(&fb->var, fb);

	return n;

}
static ssize_t show_target_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p = buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	p += sprintf(p, "%dx%d\n", lay_cfg->scale_w, lay_cfg->scale_h);

	return p - buf;
}
static ssize_t store_target_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p;
	char *s = (char *)buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}

	lay_cfg->scale_w = simple_strtoul(p, NULL, 0);
	lay_cfg->scale_h = simple_strtoul(s, NULL, 0);


	printk("scale_w: %d, scale_h: %d\n", lay_cfg->scale_w, lay_cfg->scale_h);

	return n;
}
static ssize_t show_target_pos(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p = buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	p += sprintf(p, "layer:%d, pos_x: %d, pos_y:%d\n", layer, lay_cfg->disp_pos_x, lay_cfg->disp_pos_y);

	return p - buf;
}
static ssize_t store_target_pos(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p;
	char *s = (char *)buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	p = strsep(&s, "x");
	if(!s) {
		return -EINVAL;
	}

	lay_cfg->disp_pos_x = simple_strtoul(p, NULL, 0);
	lay_cfg->disp_pos_y = simple_strtoul(s, NULL, 0);


	printk("-pos_x: %d, pos_y: %d\n", lay_cfg->disp_pos_x, lay_cfg->disp_pos_y);

	return n;
}

static ssize_t show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	int layer = to_layer_attr(attr)->id;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	char *p = buf;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	p += sprintf(p, "layer: %d, enable: %d\n", layer, lay_cfg->lay_en);

	return p - buf;
}
static ssize_t store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct ingenicfb_device *fbdev = dev_get_drvdata(dev);
	struct fb_info *fb = fbdev->fb;
	struct ingenicfb_frm_cfg *frm_cfg = &fbdev->current_frm_mode.frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg;
	int layer = to_layer_attr(attr)->id;
	int enable = 0;

	if(layer > CONFIG_FB_INGENIC_NR_LAYERS) {
		return -EINVAL;
	}

	lay_cfg = &frm_cfg->lay_cfg[layer];

	if(lay_cfg->scale_w == 0 || lay_cfg->scale_h == 0) {
		return -EINVAL;
	}

	enable = simple_strtoul(buf, NULL, 0);
	if(enable) {
		lay_cfg->lay_en = 1;
		if(lay_cfg->scale_w != lay_cfg->source_w || lay_cfg->scale_h != lay_cfg->source_h) {
			lay_cfg->lay_scale_en = 1;
		} else {
			lay_cfg->lay_scale_en = 0;
		}
	} else {
		lay_cfg->lay_en = 0;
		lay_cfg->lay_scale_en = 0;
	}



#if 0
	printk("layer: %d src_w: %d, src_h: %d, scale_w: %d, scale_h: %d, pos_x:%d, pos_y:%d\n",
		layer, lay_cfg->source_w, lay_cfg->source_h, lay_cfg->scale_w, lay_cfg->scale_h, lay_cfg->disp_pos_x, lay_cfg->disp_pos_y);
#endif

	/*update all layers?*/
	if(layer == 0) {
		ingenicfb_disable(fbdev->fb, GEN_STOP);
		ingenicfb_set_par(fb);
		ingenicfb_enable(fbdev->fb);
	}
	return n;
}


#define LAYER_ATTR(layer, _name, _mode, _show, _store)		\
	{							\
	.attr 	= __ATTR(_name, _mode, _show, _store),	\
	.id 	= layer,						\
	}


#define LAYER_DEVICE_ATTR(_name, _mode, _show, _store)							  \
	static struct layer_device_attr dev_attr_##_name##layer0 = LAYER_ATTR(0, _name, _mode, _show, _store); \
	static struct layer_device_attr dev_attr_##_name##layer1 = LAYER_ATTR(1, _name, _mode, _show, _store); \
	static struct layer_device_attr dev_attr_##_name##layer2 = LAYER_ATTR(2, _name, _mode, _show, _store); \
	static struct layer_device_attr dev_attr_##_name##layer3 = LAYER_ATTR(3, _name, _mode, _show, _store)

LAYER_DEVICE_ATTR(src_size, S_IRUGO|S_IWUSR, show_src_size, store_src_size);
LAYER_DEVICE_ATTR(src_fmt, S_IRUGO|S_IWUSR, show_src_fmt, store_src_fmt);
LAYER_DEVICE_ATTR(target_size, S_IRUGO|S_IWUSR, show_target_size, store_target_size);
LAYER_DEVICE_ATTR(target_pos, S_IRUGO|S_IWUSR, show_target_pos, store_target_pos);
LAYER_DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, show_enable, store_enable);

#define LAYER_ATTRIBUTE_GROUP(name)					\
	static struct attribute *lcd_##name##_attrs[] = {		\
		&dev_attr_src_size##name.attr.attr,			\
		&dev_attr_src_fmt##name.attr.attr,                      \
		&dev_attr_target_size##name.attr.attr,                  \
		&dev_attr_target_pos##name.attr.attr,                   \
		&dev_attr_enable##name.attr.attr,                       \
		NULL,							\
	};

LAYER_ATTRIBUTE_GROUP(layer0);
LAYER_ATTRIBUTE_GROUP(layer1);
LAYER_ATTRIBUTE_GROUP(layer2);
LAYER_ATTRIBUTE_GROUP(layer3);

static struct attribute_group lcd_layer0_group = {
		.name = "layer0",
		.attrs = lcd_layer0_attrs,
};
static struct attribute_group lcd_layer1_group = {
		.name = "layer1",
		.attrs = lcd_layer1_attrs,
};
static struct attribute_group lcd_layer2_group = {
		.name = "layer2",
		.attrs = lcd_layer2_attrs,
};
static struct attribute_group lcd_layer3_group = {
		.name = "layer3",
		.attrs = lcd_layer3_attrs,
};

static const struct attribute_group *lcd_layerx_groups[] = {
	&lcd_layer0_group,
	&lcd_layer1_group,
	&lcd_layer2_group,
	&lcd_layer3_group,
	NULL,
};





int ingenicfb_sysfs_init()
{
	ret = sysfs_create_group(&fbdev->dev->kobj, &lcd_debug_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_free_irq;
	}

	ret = sysfs_create_groups(&fbdev->dev->kobj, lcd_layerx_groups);
	if(ret) {
		dev_err(&pdev->dev, "Failed to create sysfs groups\n");
	}



}
