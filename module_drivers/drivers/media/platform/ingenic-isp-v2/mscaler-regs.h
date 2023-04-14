#ifndef	__INGENIC_ISP_MSCALER_REGS_H__
#define __INGENIC_ISP_MSCALER_REGS_H__

#define MSCA_BASE	0

//============================================================
// Mscaler
//============================================================
#define    MSCA_CH_EN                        (MSCA_BASE+0x008)
#define    MSCA_INT_STA                      (MSCA_BASE+0x00C)
#define    MSCA_INT_MSK                      (MSCA_BASE+0x010)
#define    MSCA_MASK_EN                      (MSCA_BASE+0x014)
#define    MSCA_DMAOUT_ARB                   (MSCA_BASE+0x018)
#define    MSCA_CLK_GATE_EN                  (MSCA_BASE+0x01C)
#define    MSCA_CLK_DIS                      (MSCA_BASE+0x020)
#define    MSCA_SRC_IN                       (MSCA_BASE+0x030)
#define    MSCA_SRC_SIZE                     (MSCA_BASE+0x034)
#define    MSCA_GLO_RSZ_COEF_WR              (MSCA_BASE+0x040)
#define    MSCA_SYS_PRO_CLK_EN               (MSCA_BASE+0x050)
#define    MSCA_DS0_CLK_NUM                  (MSCA_BASE+0x054)
#define    MSCA_DS1_CLK_NUM                  (MSCA_BASE+0x058)
#define    MSCA_DS2_CLK_NUM                  (MSCA_BASE+0x05C)

#define CHx_RSZ_OSIZE(n)				(MSCA_BASE+(0x0100*(n))+0x100)
#define CHx_RSZ_STEP(n)					(MSCA_BASE+(0x0100*(n))+0x104)

#define CHx_CROP_OPOS(n)				(MSCA_BASE+(0x0100*(n))+0x128)
#define CHx_CROP_OSIZE(n)				(MSCA_BASE+(0x0100*(n))+0x12c)
#define CHx_FRA_CTRL_LOOP(n)				(MSCA_BASE+(0x0100*(n))+0x130)
#define CHx_FRA_CTRL_MASK(n)				(MSCA_BASE+(0x0100*(n))+0x134)
#define CHx_MSx_POS(n,m)				(MSCA_BASE+(0x0100*(n))+0x138+(0x0c*(m)))
#define CHx_MSx_SIZE(n,m)				(MSCA_BASE+(0x0100*(n))+0x13c+(0x0c*(m)))
#define CHx_MSx_VALUE(n,m)				(MSCA_BASE+(0x0100*(n))+0x140+(0x0c*(m)))
#define CHx_MS0_POS(n)					(MSCA_BASE+(0x0100*(n))+0x138)
#define CHx_MS0_SIZE(n)					(MSCA_BASE+(0x0100*(n))+0x13c)
#define CHx_MS0_VALUE(n)				(MSCA_BASE+(0x0100*(n))+0x140)
#define CHx_MS1_POS(n)					(MSCA_BASE+(0x0100*(n))+0x144)
#define CHx_MS1_SIZE(n)					(MSCA_BASE+(0x0100*(n))+0x148)
#define CHx_MS1_VALUE(n)				(MSCA_BASE+(0x0100*(n))+0x14c)
#define CHx_MS2_POS(n)					(MSCA_BASE+(0x0100*(n))+0x150)
#define CHx_MS2_SIZE(n)					(MSCA_BASE+(0x0100*(n))+0x154)
#define CHx_MS2_VALUE(n)				(MSCA_BASE+(0x0100*(n))+0x158)
#define CHx_MS3_POS(n)					(MSCA_BASE+(0x0100*(n))+0x15c)
#define CHx_MS3_SIZE(n)					(MSCA_BASE+(0x0100*(n))+0x160)
#define CHx_MS3_VALUE(n)				(MSCA_BASE+(0x0100*(n))+0x164)
#define CHx_OUT_FMT(n)					(MSCA_BASE+(0x0100*(n))+0x168)
#define CHx_DMAOUT_Y_ADDR(n)				(MSCA_BASE+(0x0100*(n))+0x16c)
#define CHx_Y_ADDR_FIFO_STA(n)				(MSCA_BASE+(0x0100*(n))+0x170)
#define CHx_DMAOUT_Y_LAST_ADDR(n)			(MSCA_BASE+(0x0100*(n))+0x174)
#define CHx_DMAOUT_Y_LAST_STATS_NUM(n)			(MSCA_BASE+(0x0100*(n))+0x178)
#define CHx_Y_LAST_ADDR_FIFO_STA(n)			(MSCA_BASE+(0x0100*(n))+0x17c)
#define CHx_DMAOUT_Y_STRI(n)				(MSCA_BASE+(0x0100*(n))+0x180)
#define CHx_DMAOUT_UV_ADDR(n)				(MSCA_BASE+(0x0100*(n))+0x184)
#define CHx_UV_ADDR_FIFO_STA(n)				(MSCA_BASE+(0x0100*(n))+0x188)
#define CHx_DMAOUT_UV_LAST_ADDR(n)			(MSCA_BASE+(0x0100*(n))+0x18c)
#define CHx_DMAOUT_UV_LAST_STATS_NUM(n)			(MSCA_BASE+(0x0100*(n))+0x190)
#define CHx_UV_LAST_ADDR_FIFO_STA(n)			(MSCA_BASE+(0x0100*(n))+0x194)
#define CHx_DMAOUT_UV_STRI(n)				(MSCA_BASE+(0x0100*(n))+0x198)
#define CHx_DMAOUT_Y_ADDR_CLR(n)			(MSCA_BASE+(0x0100*(n))+0x19c)
#define CHx_DMAOUT_UV_ADDR_CLR(n)			(MSCA_BASE+(0x0100*(n))+0x1a0)
#define CHx_DMAOUT_Y_LAST_ADDR_CLR(n)			(MSCA_BASE+(0x0100*(n))+0x1a4)
#define CHx_DMAOUT_UV_LAST_ADDR_CLR(n)			(MSCA_BASE+(0x0100*(n))+0x1a8)
#define CHx_DMAOUT_Y_ADDR_SEL(n)			(MSCA_BASE+(0x0100*(n))+0x1ac)
#define CHx_DMAOUT_UV_ADDR_SEL(n)			(MSCA_BASE+(0x0100*(n))+0x1b0)

#define MSCA_CH0_FRM_DONE_INT	(0)
#define MSCA_CH1_FRM_DONE_INT	(1)
#define MSCA_CH2_FRM_DONE_INT	(2)
#define MSCA_CH0_CROP_ERR_INT	(3)
#define MSCA_CH1_CROP_ERR_INT	(4)
#define MSCA_CH2_CROP_ERR_INT	(5)

#define CHx_OUT_FMT_NV12 0x0
#define CHx_OUT_FMT_NV21 0x1
#define CHx_OUT_FMT_ARGB8888 0x2
#define CHx_OUT_FMT_RGB565 0x3
#define CHx_OUT_FMT_Y_OUT_ONLY (1 << 6)


#endif
