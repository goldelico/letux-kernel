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


#define CSI_PHY_IOBASE	0x10076000
#define CSI_PHY_1C2C_MODE	0xc4
#define CSI_PHY_PRECOUNTER_IN_CLK	0xb8
#define CSI_PHY_PRECOUNTER_IN_DATA	0xbc

#define csi_phy_writel(csi, addr, value)					\
	writel(value, (unsigned int *)((csi)->phy_base + addr))

#define csi_phy_readl(csi, addr)						\
       	readl((unsigned int *)((csi)->phy_base + addr))


#endif
