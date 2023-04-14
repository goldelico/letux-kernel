/* Intel PRO/1000 Linux driver
 * Copyright(c) 1999 - 2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 */

/* ethtool support for ingenic_mac */

#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/ethtool.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <linux/phy.h>

#include "ingenic_mac.h"

struct ingenic_mac_reg
{
	u32    addr;
	char   * name;
};

static struct ingenic_mac_reg mac[] =
{
	{ 0x0000, "                  Config" },
	{ 0x0004, "            Frame Filter" },
	{ 0x0008, "             MAC HT High" },
	{ 0x000C, "              MAC HT Low" },
	{ 0x0010, "               GMII Addr" },
	{ 0x0014, "               GMII Data" },
	{ 0x0018, "            Flow Control" },
	{ 0x001C, "                VLAN Tag" },
	{ 0x0020, "            GMAC Version" },
	{ 0x0024, "            GMAC Debug  " },
	{ 0x0028, "Remote Wake-Up Frame Filter" },
	{ 0x002C, "  PMT Control and Status" },
	{ 0x0030, "  LPI Control and status" },
	{ 0x0034, "      LPI Timers Control" },
	{ 0x0038, "        Interrupt Status" },
	{ 0x003c, "        Interrupt Mask" },
	{ 0x0040, "          MAC Addr0 High" },
	{ 0x0044, "           MAC Addr0 Low" },
	{ 0x0048, "          MAC Addr1 High" },
	{ 0x004c, "           MAC Addr1 Low" },
	{ 0x0100, "           MMC Ctrl Reg " },
	{ 0x010c, "        MMC Intr Msk(rx)" },
	{ 0x0110, "        MMC Intr Msk(tx)" },
	{ 0x0200, "    MMC Intr Msk(rx ipc)" },
	{ 0x0700, "       Timestamp control" },
	{ 0x0704, "    Sub-Second Increment" },
	{ 0x0708, "                 Seconds" },
	{ 0x070c, "             Nanoseconds" },
	{ 0x0710, "          Update seconds" },
	{ 0x0714, "      Update Nanoseconds" },
	{ 0x0718, "        Adjust frequency" },
	{ 0x0738, "          AVMAC Ctrl Reg" },
	{ 0x00D8, "           RGMII C/S Reg" },
	{ 0, 0 }
};
static struct ingenic_mac_reg dma0[] =
{
	{ 0x0000, "[CH0] CSR0   Bus Mode" },
	{ 0x0004, "[CH0] CSR1   TxPlDmnd" },
	{ 0x0008, "[CH0] CSR2   RxPlDmnd" },
	{ 0x000C, "[CH0] CSR3    Rx Base" },
	{ 0x0010, "[CH0] CSR4    Tx Base" },
	{ 0x0014, "[CH0] CSR5     Status" },
	{ 0x0018, "[CH0] CSR6    Control" },
	{ 0x001C, "[CH0] CSR7 Int Enable" },
	{ 0x0020, "[CH0] CSR8 Missed Fr." },
	{ 0x0028, "[CH0] CSR10 AXI Mode." },
	{ 0x0024, "[CH0] Recv Intr Wd.Tm." },
	{ 0x0028, "[CH0] AXI Bus Mode   " },
	{ 0x002c, "[CH0] AHB or AXI Status" },
	{ 0x0048, "[CH0] CSR18 Tx Desc  " },
	{ 0x004C, "[CH0] CSR19 Rx Desc  " },
	{ 0x0050, "[CH0] CSR20 Tx Buffer" },
	{ 0x0054, "[CH0] CSR21 Rx Buffer" },
	{ 0x0058, "CSR22 HWCFG          " },
	{ 0, 0 }
};

__attribute__((__unused__)) static void ingenic_mac_dump_dma_regs(synopGMACdevice *gmacdev)
{
	struct ingenic_mac_reg *reg = dma0;

	printk("======================DMA Regs start===================\n");
	while(reg->name) {
		printk("===>%s:\t0x%08x\n", reg->name, synopGMACReadReg((u32 *)gmacdev->DmaBase,reg->addr));
		reg++;
	}
	printk("======================DMA Regs end===================\n");
}

__attribute__((__unused__)) static void ingenic_mac_dump_mac_regs(synopGMACdevice *gmacdev)
{
	struct ingenic_mac_reg *reg = mac;

	printk("======================MAC Regs start===================\n");
	while(reg->name) {
		printk("===>%s:\t0x%08x\n", reg->name, synopGMACReadReg((u32 *)gmacdev->MacBase,reg->addr));
		reg++;
	}
	printk("======================MAC Regs end===================\n");
}

__attribute__((__unused__)) static void ingenic_mac_dump_phy_regs(struct ingenic_mac_local *lp) {
	u16 phy[] = {0, 1, 4, 5, 6, 9, 10, 15, 16, 17, 18, 20, 21, 24, 0x1c};

	u16 data[sizeof(phy) / sizeof(u16)];
	int i;

	printk("======================PHY Regs start===================\n");
	printk("\n-------->PHY dump: %08x\n", lp->phydev->phy_id);
	for (i = 0; i < sizeof(phy) / sizeof(u16); i++)
		data[i] = lp->mii_bus->read(lp->mii_bus, lp->phydev->mdio.addr, phy[i]);

	for (i = 0; i < sizeof(phy) / sizeof(u16); i++)
		printk("PHY reg%d, value %04x\n", phy[i], data[i]);
	printk("======================PHY Regs end===================\n");

}

__attribute__((__unused__)) static void ingenic_mac_dump_all_regs(struct net_device *netdev) {

	struct ingenic_mac_local *lp = netdev_priv(netdev);
	synopGMACdevice *gmacdev = lp->gmacdev;
	ingenic_mac_dump_phy_regs(lp);
	ingenic_mac_dump_dma_regs(gmacdev);
	ingenic_mac_dump_mac_regs(gmacdev);

}

static void ingenic_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo)
{
	struct ingenic_mac_local *lp = netdev_priv(netdev);
	strlcpy(drvinfo->driver, INGENIC_MAC_DRV_NAME,sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, INGENIC_MAC_DRV_VERSION,sizeof(drvinfo->version));
}

static int ingenic_get_regs_len(struct net_device __always_unused *netdev)
{
	return 0;
}

static void ingenic_get_regs(struct net_device *netdev,
			   struct ethtool_regs *regs, void *p)
{
	pm_runtime_get_sync(netdev->dev.parent);
	ingenic_mac_dump_all_regs(netdev);
	pm_runtime_put_sync(netdev->dev.parent);
}

static void ingenic_get_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *pp)
{
	const struct ingenic_mac_local *lp = netdev_priv(ndev);

	pp->autoneg = lp->flowcontrol.autoneg;
	pp->tx_pause = lp->flowcontrol.tx;
	pp->rx_pause = lp->flowcontrol.rx;
}

static int ingenic_set_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *pp)
{
	struct ingenic_mac_local *lp = netdev_priv(ndev);
	int ret = 0;

	lp->flowcontrol.autoneg = pp->autoneg;
	if (pp->autoneg) {
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, lp->phydev->advertising);
		linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, lp->phydev->advertising);
	} else {
		linkmode_clear_bit(ETHTOOL_LINK_MODE_Pause_BIT, lp->phydev->advertising);
		linkmode_clear_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, lp->phydev->advertising);
		lp->flowcontrol.rx = pp->rx_pause;
		lp->flowcontrol.tx = pp->tx_pause;
	}

	if (netif_running(ndev))
		ret = phy_start_aneg(lp->phydev);

	return ret;
}


static int ingenic_set_coalesce(struct net_device *netdev,
			      struct ethtool_coalesce *ec)
{
	struct ingenic_mac_local *lp = netdev_priv(netdev);

	/* Check not supported parameters  */
	if ((ec->rx_max_coalesced_frames) || (ec->rx_coalesce_usecs_irq) ||
	    (ec->rx_max_coalesced_frames_irq) || (ec->tx_coalesce_usecs_irq) ||
	    (ec->use_adaptive_rx_coalesce) || (ec->use_adaptive_tx_coalesce) ||
	    (ec->pkt_rate_low) || (ec->rx_coalesce_usecs_low) ||
	    (ec->rx_max_coalesced_frames_low) || (ec->tx_coalesce_usecs_high) ||
	    (ec->tx_max_coalesced_frames_low) || (ec->pkt_rate_high) ||
	    (ec->tx_coalesce_usecs_low) || (ec->rx_coalesce_usecs_high) ||
	    (ec->rx_max_coalesced_frames_high) ||
	    (ec->tx_max_coalesced_frames_irq) ||
	    (ec->stats_block_coalesce_usecs) ||
	    (ec->tx_max_coalesced_frames) ||
	    (ec->tx_max_coalesced_frames_high) || (ec->rate_sample_interval))
		return -EOPNOTSUPP;

	if (ec->tx_coalesce_usecs > INGENIC_TX_MAX_COALESCE_USECS ||
		ec->tx_coalesce_usecs < INGENIC_TX_MIN_COALESCE_USECS)
		return -EINVAL;
	if (ec->rx_coalesce_usecs > INGENIC_RX_MAX_COALESCE_USECS ||
		ec->rx_coalesce_usecs < INGENIC_RX_MIN_COALESCE_USECS)
		return -EINVAL;

	lp->tx_coalesce_usecs = ec->tx_coalesce_usecs;
	lp->rx_coalesce_usecs = ec->rx_coalesce_usecs;

	return 0;
}

static int ingenic_get_coalesce(struct net_device *netdev,
			      struct ethtool_coalesce *ec)
{
	struct ingenic_mac_local *lp = netdev_priv(netdev);

	ec->tx_coalesce_usecs = lp->tx_coalesce_usecs;
	ec->rx_coalesce_usecs = lp->rx_coalesce_usecs;

	return 0;
}
static u32 ingenic_get_msglevel(struct net_device *ndev)
{
	const struct ingenic_mac_local *lp = netdev_priv(ndev);

	return lp->msg_enable;
}

static void ingenic_set_msglevel(struct net_device *ndev, u32 msglevel)
{
	struct ingenic_mac_local *lp = netdev_priv(ndev);

	lp->msg_enable = msglevel;
}

static const struct ethtool_ops ingenic_mac_ethtool_ops = {

	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.get_drvinfo		= ingenic_get_drvinfo,
	.get_regs_len		= ingenic_get_regs_len,
	.get_regs		= ingenic_get_regs,
	.get_link		= ethtool_op_get_link,
	.get_pauseparam		= ingenic_get_pauseparam,
	.set_pauseparam		= ingenic_set_pauseparam,
	.get_coalesce		= ingenic_get_coalesce,
	.set_coalesce		= ingenic_set_coalesce,
	.get_msglevel		= ingenic_get_msglevel,
	.set_msglevel		= ingenic_set_msglevel,
	.supported_coalesce_params = ETHTOOL_COALESCE_RX_USECS
				| ETHTOOL_COALESCE_RX_MAX_FRAMES
				| ETHTOOL_COALESCE_TX_USECS
				| ETHTOOL_COALESCE_TX_MAX_FRAMES,
};

void ingenic_mac_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &ingenic_mac_ethtool_ops;
}
