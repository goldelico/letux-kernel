#ifndef __INGENIC_ISP_CSI_H__
#define __INGENIC_ISP_CSI_H__

#define VERSION				0x00
#define N_LANES				0x04
#define PHY_SHUTDOWNZ			0x08
#define DPHY_RSTZ			0x0C
#define CSI2_RESETN			0x10
#define PHY_STATE			0x14
#define DATA_IDS_1			0x18
#define DATA_IDS_2			0x1C
#define ERR1				0x20
#define ERR2				0x24
#define MASK1				0x28
#define MASK2				0x2C
#define PHY_TST_CTRL0       		0x30
#define PHY_TST_CTRL1       		0x34
#define CTRL_DUAL_ENABLE                        0x080
#define RXVALID_MASK                            0x100


#define VC0_FRAME_NUM			0x40	/*[31:16] FS End, [15:0] FS Start*/
#define VC1_FRAME_NUM			0x44
#define VC2_FRAME_NUM			0x48
#define VC3_FRAME_NUM			0x4c
#define VC0_LINE_NUM			0x50	/*[31:16] Line End, [15:0] Line Start*/
#define VC1_LINE_NUM			0x54
#define VC2_LINE_NUM			0x58
#define VC3_LINE_NUM			0x60



#define csi_readl(port, reg)						\
	__raw_readl((unsigned int *)((port)->base + reg))
#define csi_writel(port, reg, value)					\
	__raw_writel((value), (unsigned int *)((port)->base + reg))

#define csi_core_writel(csi, addr, value)				\
	writel(value, (unsigned int *)((csi)->iobase + addr))
#define csi_core_readl(csi, addr)					\
	readl((unsigned int *)((csi)->iobase + addr))


#define CSI_PHY_IOBASE	0x10022000

#define PHY_ENB				0x000
#define PHY_DUAL_CLK_ENB		0x080
#define PHY_CK0_CONTI			0x128
#define PHY_CK0_SETTLE			0x160
#define PHY_DATA0_CONTI			0x1A8
#define PHY_DATA0_SETTLE		0x1E0
#define PHY_DATA1_CONTI			0x228
#define PHY_DATA1_SETTLE		0x260
#define PHY_DATA2_CONTI			0x2A8
#define PHY_DATA2_SETTLE		0x2E0
#define PHY_DATA3_CONTI			0x328
#define PHY_DATA3_SETTLE		0x360
#define PHY_CK1_CONTI			0x3A8
#define PHY_CK1_SETTLE			0x3E0


#define csi_phy_writel(csi, addr, value)					\
	writel(value, (unsigned int *)((csi)->phy_base + addr))

#define csi_phy_readl(csi, addr)						\
       	readl((unsigned int *)((csi)->phy_base + addr))


#endif
