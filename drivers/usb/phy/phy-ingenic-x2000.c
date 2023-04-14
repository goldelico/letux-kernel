#include <linux/delay.h>
#include "phy-ingenic.h"

#define CPM_USBPCR     0x3C
#define CPM_USBRDT     0x40
#define CPM_USBVBFIL   0x44
#define CPM_USBPCR1    0x48

#define CPM_SRBC    0xC4
#define CPM_OPCR    0x24

#define OPCR_SPENDN0_BIT			7
#define OPCR_GATE_USBPHY_CLK_BIT	23
#define SRBC_USB_SR					12

#define USBRDT_RESUME_IRQ_ENABLE			31
#define USBRDT_RESUME_CLEAR_IRQ				30
#define USBRDT_RESUME_STATUS				27

static int x2000_priv_data_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);
	return 0;
}

static int x2000_phy_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0x00200000, CPM_USBPCR1);
	usb_cpm_writel(usb_phy, 0x80400000, CPM_USBPCR);
	udelay(800);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x70000000, CPM_USBPCR1);
	udelay(800);

	/* TX HS driver strength configure */
	value = usb_phy_readl(usb_phy, 0x40);
	value |= 0x7 << 3;
	usb_phy_writel(usb_phy, value, 0x40);

	return 0;
}

static int x2000_phy_set_wakeup(struct usb_phy_data *usb_phy, bool enabled)
{
	if (enabled) {
		usb_cpm_clear_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);
		if (usb_phy->usb_wakeup) {
			usb_cpm_clear_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
		}
	} else {
		if (usb_phy->usb_wakeup)
			usb_cpm_clear_bit(usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
		usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);
	}

	return 0;
}

static int x2000_phy_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS,CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

struct usb_phy_priv usb_phy_x2000_priv = {
	.priv_data_init = x2000_priv_data_init,

	.phy_init = x2000_phy_init,
	.phy_set_wakeup = x2000_phy_set_wakeup,
	.phy_get_wakeup = x2000_phy_get_wakeup,
};
