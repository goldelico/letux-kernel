#include <linux/delay.h>
#include "phy-ingenic.h"

#define CPM_USBPCR    (0x3c)
#define CPM_USBRDT    (0x40)
#define CPM_USBVBFIL  (0x44)
#define CPM_USBPCR1   (0x48)

#define CPM_SRBC      (0xc4)
#define CPM_OPCR      (0x24)

#define OPCR_SPENDN0_BIT			7
#define OPCR_GATE_USBPHY_CLK_BIT	23
#define SRBC_USB_SR					12

static int x1000_priv_data_init(struct usb_phy_data *usb_phy)
{
	/* reset usb */
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	return 0;
}

static int x1000_phy_init(struct usb_phy_data *usb_phy)
{
	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0xA3C919FF, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x0D280000, CPM_USBPCR1);
	udelay(500);
#ifdef CONFIG_USB_DWC2_EXT_VBUS_DETECT
	usb_cpm_writel(usb_phy, 0xA38919FF, CPM_USBPCR);
#else
	usb_cpm_writel(usb_phy, 0xA20919FF, CPM_USBPCR);
#endif
	usb_cpm_writel(usb_phy, 0x0D080000, CPM_USBPCR1);
	udelay(500);

	/* jitter filter */
	usb_cpm_writel(usb_phy, 0x00FF0080, CPM_USBVBFIL);

	/* reset detect time */
	usb_cpm_writel(usb_phy, 0x02000096, CPM_USBRDT);

	return 0;
}

static int x1000_phy_set_wakeup(struct usb_phy_data *usb_phy, bool enabled)
{
	if (enabled)
		usb_cpm_clear_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);
	else
		usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	return 0;
}

struct usb_phy_priv usb_phy_x1000_priv = {
	.priv_data_init = x1000_priv_data_init,

	.phy_init = x1000_phy_init,
	.phy_set_wakeup = x1000_phy_set_wakeup,
};
