#include <linux/delay.h>
#include "phy-ingenic.h"

#define CPM_USBPCR                      (0x3C)
#define CPM_USBRDT                      (0x40)
#define CPM_USBVBFIL                    (0x44)
#define CPM_USBPCR1                     (0x48)

#define CPM_SRBC                        (0xC4)
#define CPM_OPCR                        (0x24)

#define OPCR_SPENDN0_BIT			7
#define OPCR_GATE_USBPHY_CLK_BIT	23
#define SRBC_USB_SR					12

#define USBRDT_RESUME_IRQ_ENABLE			31
#define USBRDT_RESUME_CLEAR_IRQ				30
#define USBRDT_RESUME_STATUS				27

static int x1600_priv_data_init(struct usb_phy_data *usb_phy)
{
	/* reset usb */
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	return 0;
}

static int x1600_phy_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0x00200000, CPM_USBPCR1);
	usb_cpm_writel(usb_phy, 0x80400000, CPM_USBPCR);
	udelay(500);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x70000000, CPM_USBPCR1);
	udelay(500);

	/* always enable pre-emphasis */
	value = usb_phy_readl(usb_phy, 0x30);
	// value &= ~(0x7 << 0);
	value |= 0x7 << 0;
	usb_phy_writel(usb_phy, value, 0x30);

	/* Tx HS pre_emphasize strength configure */
	value = usb_phy_readl(usb_phy, 0x40);
	// value &= ~(0x7 << 3);
	value |= 0x7 << 3;
	usb_phy_writel(usb_phy, value, 0x40);

	/* Vbus 5V mode */
	value = usb_phy_readl(usb_phy, 0x10C);
	value &= ~((0x7 << 0) | (0x7 << 3));
	value |= ((0x5 << 0) | (0x5 << 3));
	usb_phy_writel(usb_phy, value, 0x10C);

	/* Vbus 5V mode */
	value = usb_phy_readl(usb_phy, 0x110);
	value &= ~((0x7 << 0) | (0x7 << 3));
	value |= ((0x5 << 0) | (0x5 << 3));
	usb_phy_writel(usb_phy, value, 0x110);

#ifdef CONFIG_USB_DWC2_EXT_VBUS_DETECT
	/* VBUS voltage level detection power down. */
	value = usb_phy_readl(usb_phy, 0x108);
	value |= 0x1 << 3;
	usb_phy_writel(usb_phy, value, 0x108);
#endif

	/* HS eye height tuning */
	value = usb_phy_readl(usb_phy, 0x124);
	value &= ~(0x7 << 2);
	value |= 0x1 << 2;
	usb_phy_writel(usb_phy, value, 0x124);

	return 0;
}

static int x1600_phy_set_wakeup(struct usb_phy_data *usb_phy, bool enabled)
{
	unsigned long value;

	if (enabled) {
#ifndef CONFIG_USB_DWC2_EXT_VBUS_DETECT
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(usb_phy, 0x108);
		value |= 1 << 3;
		usb_phy_writel(usb_phy, value, 0x108);
#endif

		/* disable full/low speed driver at the receiver */
		value = usb_phy_readl(usb_phy, 0x100);
		value &= ~(1 << 6);
		usb_phy_writel(usb_phy, value, 0x100);

		usb_cpm_clear_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);
		if (usb_phy->usb_wakeup) {
			usb_cpm_clear_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
		}
	} else {
		if (usb_phy->usb_wakeup)
			usb_cpm_clear_bit(usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
		usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

		/* enable full/low speed driver at the receiver */
		value = usb_phy_readl(usb_phy, 0x100);
		value |= 1 << 6;
		usb_phy_writel(usb_phy, value, 0x100);

#ifndef CONFIG_USB_DWC2_EXT_VBUS_DETECT
		/* VBUS voltage level detection power on. */
		value = usb_phy_readl(usb_phy, 0x108);
		value &= ~(1 << 3);
		usb_phy_writel(usb_phy, value, 0x108);
#endif
	}

	return 0;
}

static int x1600_phy_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS,CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

struct usb_phy_priv usb_phy_x1600_priv = {
	.priv_data_init = x1600_priv_data_init,

	.phy_init = x1600_phy_init,
	.phy_set_wakeup = x1600_phy_set_wakeup,
	.phy_get_wakeup = x1600_phy_get_wakeup,
};
