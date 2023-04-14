#ifndef __INGENIC_MAC_H__
#define __INGENIC_MAC_H__


#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timer.h>
#include <linux/mii.h>
#include "synopGMAC_Dev.h"

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct ingenic_mac_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	unsigned long time_stamp;
	u16 length;
	volatile u8 transfering; /* used by tx */
	volatile u8 invalid;	 /* used by rx */
	u16 mapped_as_page;
	unsigned int segs;
};

#define INGENIC_MAC_DRV_NAME		"dwc-mac"
#define INGENIC_MAC_DRV_VERSION		"1.0"
#define INGENIC_MAC_DRV_DESC		"Ingenic on-chip Ethernet MAC driver"

#define INGENIC_MAC_MSG_DEFAULT     (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP)


/* TX/RX descriptor defines */
#define INGENIC_MAC_TX_DESC_COUNT                128
#define INGENIC_MAC_MAX_TXD                      256
#define INGENIC_MAC_MIN_TXD                       80

#define INGENIC_MAC_RX_DESC_COUNT                CONFIG_INGENIC_GMAC_RX_DESC_COUNT
#define INGENIC_MAC_MAX_RXD                      10240
#define INGENIC_MAC_MIN_RXD                       80

#define INGENIC_RX_MAX_COALESCE_USECS	200
#define INGENIC_RX_MIN_COALESCE_USECS	100
#define INGENIC_TX_MAX_COALESCE_USECS	2000
#define INGENIC_TX_MIN_COALESCE_USECS	200

#define INGENIC_WAKE_IRQ_NUM		20000

struct ingenic_mac_tx_ring {
	/* pointer to the descriptor ring memory */
	DmaDesc *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for trans done status */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct ingenic_mac_buffer *buffer_info;
};

struct ingenic_mac_rx_ring {
	/* pointer to the descriptor ring memory */
	DmaDesc *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct ingenic_mac_buffer *buffer_info;
};

#define INGENIC_MAC_DESC_UNUSED(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->count) + (R)->next_to_clean - (R)->next_to_use - 1)

#define INGENIC_MAC_DESC_USED(R)  (((R)->count - 1) - INGENIC_MAC_DESC_UNUSED(R))

#define INGENIC_MAC_GET_DESC(R, i)	(&(((DmaDesc *)((R).desc))[i]))
#define INGENIC_MAC_RX_DESC(R, i)		INGENIC_MAC_GET_DESC(R, i)
#define INGENIC_MAC_TX_DESC(R, i)		INGENIC_MAC_GET_DESC(R, i)

struct ingenic_mac_flowcontrol {
	int autoneg;
	int rx;
	int rx_current;
	int tx;
	int tx_current;
};

struct ingenic_mac_dts_control {
	unsigned int force;
	unsigned int autoneg;
	unsigned int speed;
	unsigned int duplex;
};

struct virtual_phy{
	u16 reg[MII_NCONFIG + 1];
};

struct ingenic_mac_local {
	struct clk *clk_gate;
	struct clk *clk_cgu;
	struct clk *clk_tx;
	void __iomem *baseaddr;

	struct ingenic_mac_tx_ring tx_ring;
	unsigned int restart_queue;
	u32 tx_timeout_count;

	struct timer_list watchdog_timer;
	struct ingenic_mac_rx_ring rx_ring;

	struct napi_struct napi;
	spinlock_t napi_poll_lock;

	struct net_device *netdev;
	struct platform_device *pdev;
	struct net_device_stats net_stats;

	spinlock_t stats_lock;

	atomic_t tx_fifo_used;

	unsigned char Mac[6];	/* MAC address of the board */
	spinlock_t link_lock;

	/* MII and PHY stuffs */
	int old_link;
	int old_speed;
	int old_duplex;

	bool no_phy_connect;
	struct phy_device *phydev;
	struct mii_bus *mii_bus;
	bool use_mdio_goio;
	struct platform_device *mii_pdev;

	u32 alloc_rx_buff_failed;

	struct work_struct reset_task;
	struct mii_if_info mii;

	u32 msg_enable;
	struct ingenic_mac_flowcontrol flowcontrol;
	/* Spinlock for register read-modify-writes. */
	spinlock_t hw_lock;

	/*power*/
	int pwr_gpio;
	u32 pwr_lvl;

	/*hw reset*/
	u32 reset_ms;
	u32 reset_delay_ms;
	int reset_gpio;
	u32 reset_lvl;
	int id;

	synopGMACdevice *gmacdev;
	unsigned int interface;
	struct ingenic_mac_dts_control dts_ctl;
	struct virtual_phy virt_phy;

	int rx_coalesce_usecs;
	struct hrtimer rx_coalesce_timer;
	int tx_coalesce_usecs;
	struct hrtimer tx_coalesce_timer;
	unsigned long wake_irq_num;

#ifdef CONFIG_INGENIC_GMAC_USE_HWSTAMP
	struct hwtstamp_config stamp_cfg;
	struct ptp_clock_info caps;
	struct ptp_clock *clock;
	struct clk *cgu_ptp;
	int phc_index;
	u32 ptp_addend;
	u32 ptp_freq;
	spinlock_t phc_lock;
#endif
};

struct ingenic_gmac_priv {
	unsigned int support_multi_if;
	void (*get_clk_name)(struct ingenic_mac_local *lp, char *gate_name, char *cgu_name);
};

void ingenic_mac_set_ethtool_ops(struct net_device *netdev);
#endif	/* __INGENIC_MAC_H__ */
