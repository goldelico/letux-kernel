#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/usb/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <soc/cpm.h>
#include "../dwc2/core.h"
#include "../dwc2/hcd.h"
#include <linux/usb/hcd.h>

#define usb_phy_readb(addr)	readb((addr))
#define usb_phy_writeb(val, addr) writeb(val,(addr))

#define PHY_TX_HS_STRENGTH_CONF	(0x40)
#define PHY_EYES_MAP_ADJ_CONF	(0x60)
#define PHY_REG_100		(0x100)
#define PHY_SUSPEND_LPM		(0x108)

#define PHY_RX_SQU_TRI		(0x64)
#define PHY_RX_SQU_TRI_112MV    (0x0)
#define PHY_RX_SQU_TRI_125MV    (0x8)

/* just define for x2500  */
#define SRBC_USB_SR			BIT (12)
#define USBRDT_UTMI_RST                 BIT(27)


static const struct of_device_id of_matchs[];

struct inno_phy_tuning {
	u8 val;
	u8 msk;
	u8 bit_off;
	u32 reg_addr;
};

struct ingenic_usb_phy_priv {
	bool phy_remote_wakeup;
	bool phy_vbus_detect_ctl;
	bool phy_ls_fs_driver_ctl;
	int (*phyinit)(struct usb_phy*);
	u32 phy_tuning_num;
	struct inno_phy_tuning phy_tuning[];
};

struct ingenic_usb_phy {
	struct usb_phy phy;
	struct device *dev;
	struct clk *gate_clk;
	struct regmap *regmap;
	void __iomem *base;
	char hsotg_sw_switch[8];
	struct ingenic_usb_phy_priv *iphy_priv;

	/*otg phy*/
	spinlock_t phy_lock;
	struct gpio_desc	*gpiod_drvvbus;
	struct gpio_desc	*gpiod_id;
	struct gpio_desc	*gpiod_vbus;

#define USB_DETE_VBUS	0x1
#define USB_DETE_ID	0x2
	unsigned int usb_dete_state;
	unsigned int forced_dev_flags;
	struct delayed_work	work;

	enum usb_device_speed	roothub_port_speed;
};

static inline int inno_usbphy_read(struct usb_phy *x, u32 reg)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	u32 val = 0;
	regmap_read(iphy->regmap, reg, &val);
	return val;
}

static inline int inno_usbphy_write(struct usb_phy *x, u32 val, u32 reg)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);

	return regmap_write(iphy->regmap, reg, val);
}

static inline int inno_usbphy_update_bits(struct usb_phy *x, u32 reg, u32 mask, u32 bits)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);

	return regmap_update_bits_check(iphy->regmap, reg, mask, bits, NULL);
}

#ifdef CONFIG_USB_DWC2_DETECT_CONNECT
static irqreturn_t usb_dete_irq_handler(int irq, void *data)
{
	struct ingenic_usb_phy *iphy = (struct ingenic_usb_phy *)data;

	schedule_delayed_work(&iphy->work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static void usb_dete_work(struct work_struct *work)
{
	struct ingenic_usb_phy *iphy =
		container_of(work, struct ingenic_usb_phy, work.work);
	struct usb_otg		*otg = iphy->phy.otg;
	int vbus = 0, id_level = 1, usb_state = 0;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };

	if (iphy->gpiod_vbus)
		vbus = gpiod_get_value_cansleep(iphy->gpiod_vbus);

	if (iphy->gpiod_id)
		id_level = gpiod_get_value_cansleep(iphy->gpiod_id);

	if (!strcmp(iphy->hsotg_sw_switch, "device")) {
		id_level = 1;
	}

	if (vbus && id_level)
		usb_state |= USB_DETE_VBUS;
	else
		usb_state &= ~USB_DETE_VBUS;

	if (id_level)
		usb_state |= USB_DETE_ID;
	else
		usb_state &= ~USB_DETE_ID;


	if (usb_state != iphy->usb_dete_state) {
		enum usb_phy_events	status;

		iphy->usb_dete_state = usb_state;
		printk(KERN_DEBUG "%s() usb_state=0x%x, iphy->phy.devname=%s\n", __func__, usb_state, dev_name(iphy->phy.dev));
		if (!(usb_state & USB_DETE_VBUS)) {
			status = USB_EVENT_NONE;
			otg->state = OTG_STATE_B_IDLE;
			iphy->phy.last_event = status;
#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
			if (otg->gadget){
				usb_gadget_vbus_disconnect(otg->gadget);
				atomic_notifier_call_chain(&iphy->phy.notifier, status,
					otg->gadget);
			}
#endif
			kobject_uevent_env(&iphy->phy.dev->kobj,
					   KOBJ_CHANGE, disconnected);
		}

		if (!(usb_state & USB_DETE_ID)) {
			status = USB_EVENT_ID;
			otg->state = OTG_STATE_A_IDLE;
			iphy->phy.last_event = status;
#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
			if (otg->gadget) {
				usb_gadget_vbus_connect(otg->gadget);
			}
#endif
			if (otg->host) {
				atomic_notifier_call_chain(&iphy->phy.notifier, status,
						otg->host);
			}
			kobject_uevent_env(&iphy->phy.dev->kobj,
					   KOBJ_CHANGE, connected);
		}

		if (usb_state & USB_DETE_VBUS) {
			status = USB_EVENT_VBUS;
			otg->state = OTG_STATE_B_PERIPHERAL;
			iphy->phy.last_event = status;

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
			if (otg->gadget){
				usb_gadget_vbus_connect(otg->gadget);
				atomic_notifier_call_chain(&iphy->phy.notifier, status,
                                         otg->gadget);
			}
#endif
			kobject_uevent_env(&iphy->phy.dev->kobj,
					   KOBJ_CHANGE, connected);
		}
		iphy->usb_dete_state = usb_state;
	}

	return;
}
#endif

static int iphy_init(struct usb_phy *x)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	struct inno_phy_tuning *phy_tuning = iphy->iphy_priv->phy_tuning;
	u32 phy_tuning_num = iphy->iphy_priv->phy_tuning_num;
	u32 usbpcr, usbpcr1;
	unsigned long flags;
	u32 i;
	u8 reg;

	/* USB phy clk opened. */
	if (!IS_ERR_OR_NULL(iphy->gate_clk))
		clk_prepare_enable(iphy->gate_clk);

	spin_lock_irqsave(&iphy->phy_lock, flags);
	usbpcr1 = inno_usbphy_read(x, CPM_USBPCR1);
	usbpcr1 &= ~(USBPCR1_DPPULLDOWN | USBPCR1_DMPULLDOWN);
	inno_usbphy_write(x, usbpcr1, CPM_USBPCR1);

	usbpcr = inno_usbphy_read(x, CPM_USBPCR);
	usbpcr &= ~USBPCR_IDPULLUP_MASK;
#if (IS_ENABLED(CONFIG_USB_DWC2_HOST) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE))
	usbpcr |= USBPCR_USB_MODE | USBPCR_IDPULLUP_OTG;
#elif IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL)
	usbpcr &= ~USBPCR_USB_MODE | USBPCR_IDPULLUP_OTG;
#endif
	inno_usbphy_write(x, usbpcr, CPM_USBPCR);

	inno_usbphy_update_bits(x, CPM_USBPCR1, USBPCR1_PORT_RST, USBPCR1_PORT_RST);
	inno_usbphy_update_bits(x, CPM_USBPCR, USBPCR_POR, USBPCR_POR);
	udelay(11);
	inno_usbphy_update_bits(x, CPM_USBPCR, USBPCR_POR, 0);
	udelay(11);
	inno_usbphy_update_bits(x, CPM_OPCR, OPCR_USB_SPENDN, OPCR_USB_SPENDN);
	udelay(501);
	inno_usbphy_update_bits(x, CPM_USBPCR1, USBPCR1_PORT_RST, 0);
	udelay(1);

	usbpcr1 = inno_usbphy_read(x, CPM_USBPCR1);
	usbpcr1 |= USBPCR1_DPPULLDOWN | USBPCR1_DMPULLDOWN;
	inno_usbphy_write(x, usbpcr1, CPM_USBPCR1);

	/* set squelch trigger point */
	reg = usb_phy_readb(iphy->base + PHY_RX_SQU_TRI);
	reg &= ~(0xf << 3);
	reg |= PHY_RX_SQU_TRI_125MV << 3;
	usb_phy_writeb(reg,iphy->base + PHY_RX_SQU_TRI);

	/* set inno usb phy tuning */
	for (i = 0; i < phy_tuning_num; i++) {
		reg = usb_phy_readb(iphy->base + phy_tuning[i].reg_addr);
		reg &= ~(phy_tuning[i].msk << phy_tuning[i].bit_off);
		reg |= (phy_tuning[i].val << phy_tuning[i].bit_off);
		usb_phy_writeb(reg, iphy->base + phy_tuning[i].reg_addr);
	}

	spin_unlock_irqrestore(&iphy->phy_lock, flags);
	return 0;
}

static int iphy_init_x2500(struct usb_phy *x)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	u32 usbpcr, usbpcr1;
	unsigned long flags;
	u8 reg;

	if (!IS_ERR_OR_NULL(iphy->gate_clk))
		clk_prepare_enable(iphy->gate_clk);

	spin_lock_irqsave(&iphy->phy_lock, flags);
	usbpcr1 = inno_usbphy_read(x, CPM_USBPCR1);
	usbpcr1 |= USBPCR1_DPPULLDOWN | USBPCR1_DMPULLDOWN;
	inno_usbphy_write(x, usbpcr1, CPM_USBPCR1);

	usbpcr = inno_usbphy_read(x, CPM_USBPCR);
	usbpcr &= ~USBPCR_IDPULLUP_MASK;
#if IS_ENABLED(CONFIG_USB_DWC2_HOST)
	usbpcr |= USBPCR_USB_MODE | USBPCR_IDPULLUP_OTG;
#elif IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL)
	usbpcr &= ~USBPCR_USB_MODE;
#endif
	inno_usbphy_write(x, usbpcr, CPM_USBPCR);
	inno_usbphy_update_bits(x, CPM_USBPCR1, USBPCR1_PORT_RST, USBPCR1_PORT_RST);
	inno_usbphy_update_bits(x, CPM_USBRDT, USBRDT_UTMI_RST, 0);
	inno_usbphy_update_bits(x, CPM_USBPCR, USBPCR_POR, USBPCR_POR);
	inno_usbphy_update_bits(x, CPM_SRBC, SRBC_USB_SR, SRBC_USB_SR);
	udelay(5);
	inno_usbphy_update_bits(x, CPM_USBPCR, USBPCR_POR, 0);
	udelay(10);
	inno_usbphy_update_bits(x, CPM_OPCR, OPCR_USB_SPENDN, OPCR_USB_SPENDN);
	udelay(550);
	inno_usbphy_update_bits(x, CPM_USBRDT, USBRDT_UTMI_RST, USBRDT_UTMI_RST);
	udelay(10);
	inno_usbphy_update_bits(x, CPM_SRBC, SRBC_USB_SR, 0);

	/* set squelch trigger point */
	reg = usb_phy_readb(iphy->base + PHY_RX_SQU_TRI);
	reg &= ~(0xf << 3);
	reg |= PHY_RX_SQU_TRI_125MV << 3;
	usb_phy_writeb(reg,iphy->base + PHY_RX_SQU_TRI);


	spin_unlock_irqrestore(&iphy->phy_lock, flags);
	return 0;
}

static int iphy_set_suspend(struct usb_phy *x, int suspend);
static void iphy_shutdown(struct usb_phy *x)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	bool vbus_detect_ctl = iphy->iphy_priv->phy_vbus_detect_ctl;
	bool ls_fs_driver_ctl = iphy->iphy_priv->phy_ls_fs_driver_ctl;

	iphy_set_suspend(x, 1);

	/* VBUS voltage level detection power down. */
	if (vbus_detect_ctl)
		usb_phy_writeb(0x8, iphy->base + PHY_SUSPEND_LPM);

	/* disable full/low speed driver at the receiver */
	/* NOTE: it is used to reducepower consumption. */
	if (ls_fs_driver_ctl) {
		unsigned int reg = usb_phy_readb(iphy->base + PHY_REG_100);
		reg &= ~(1<<6);
		usb_phy_writeb(reg, iphy->base + PHY_REG_100);
	}

	/* USB phy clk gate. */
	if (!IS_ERR_OR_NULL(iphy->gate_clk))
		clk_disable_unprepare(iphy->gate_clk);
}

static int iphy_set_vbus(struct usb_phy *x, int on)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);

	if (!(IS_ERR_OR_NULL(iphy->gpiod_drvvbus))) {
		printk("OTG VBUS %s\n", on ? "ON" : "OFF");

#ifdef CONFIG_HALLEY5_V10_HOST_TO_4GMODULE
		if(!(IS_ENABLED(CONFIG_DT_HALLEY5_V10) && IS_ENABLED(CONFIG_USB_DWC2_HOST)))
#endif
			gpiod_set_value(iphy->gpiod_drvvbus, on);
	}
	return 0;
}

static int iphy_set_wakeup(struct usb_phy *x, bool enabled);
static int iphy_set_suspend(struct usb_phy *x, int suspend)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	unsigned long flags;

	spin_lock_irqsave(&iphy->phy_lock, flags);

	if(suspend){
		/* USB phy port reset is low. */
		inno_usbphy_update_bits(x, CPM_OPCR, USBPCR1_PORT_RST, 0);

		/* USB phy enter suspend. */
		inno_usbphy_update_bits(x, CPM_OPCR, OPCR_USB_SPENDN, 0);

		udelay(10);
	} else {
		/* USB PHY resume. */
		inno_usbphy_update_bits(x, CPM_OPCR, OPCR_USB_SPENDN, OPCR_USB_SPENDN);
		udelay(501);

		inno_usbphy_update_bits(x, CPM_USBPCR1, USBPCR1_PORT_RST, USBPCR1_PORT_RST);
		udelay(2);

		inno_usbphy_update_bits(x, CPM_USBPCR1, USBPCR1_PORT_RST, 0);
		udelay(1);

	}

	spin_unlock_irqrestore(&iphy->phy_lock, flags);

	return 0;
}

static int iphy_set_wakeup(struct usb_phy *x, bool enabled)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);
	struct usb_otg *otg = iphy->phy.otg;
	enum usb_device_speed speed;
	unsigned long flags;
	int timeout;
	unsigned int usbrdt;
#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
	struct dwc2_hsotg *hsotg = (struct dwc2_hsotg *)container_of(otg->gadget, struct dwc2_hsotg, gadget);
#endif

	spin_lock_irqsave(&iphy->phy_lock, flags);

	if (enabled) {
		/* Enable resume interrupt */
		usbrdt = inno_usbphy_read(x, CPM_USBRDT);
		usbrdt &= ~USBRDT_RESUME_SPEED_MSK;
#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
		if (dwc2_is_device_mode(hsotg)) {
			speed = (otg && otg->gadget) ? otg->gadget->speed : USB_SPEED_FULL;
			if(speed == USB_SPEED_HIGH)
				usbrdt |= USBRDT_RESUME_SPEED_HIGH;
			if(speed == USB_SPEED_FULL)
				usbrdt |= USBRDT_RESUME_SPEED_FULL;
			if(speed == USB_SPEED_LOW)
				usbrdt |= USBRDT_RESUME_SPEED_LOW;
		} else {
			/* Remote wakeup device speed. */
			usbrdt |= USBRDT_RESUME_SPEED_LOW;
		}
#else
		usbrdt |= USBRDT_RESUME_SPEED_LOW;
#endif
		inno_usbphy_write(x, usbrdt, CPM_USBRDT);

		//TODO: the function be in suspend.
		inno_usbphy_update_bits(x, CPM_USBRDT, USBRDT_RESUME_INTEEN, USBRDT_RESUME_INTEEN);

	} else {
		/* Disable usb resume interrupt. */
		inno_usbphy_update_bits(x, CPM_USBRDT, USBRDT_RESUME_INTEEN, 0);

		/* Clear usb resume interrupt. */
		usbrdt = inno_usbphy_read(x, CPM_USBRDT);
		if(usbrdt & (USBRDT_RESUME_STATUS)){
			usbrdt |= USBRDT_RESUME_INTERCLR;
			inno_usbphy_write(x, usbrdt, CPM_USBRDT);
			timeout = 100;
			while(1){
				if(timeout-- < 0){
					printk("%s:%d resume interrupt clear failed\n", __func__, __LINE__);
					return -EAGAIN;
				}
				usbrdt = inno_usbphy_read(x, CPM_USBRDT);
				if(!(usbrdt & (USBRDT_RESUME_STATUS)))
					break;
			}
		}

	}
	/* Clear usb resume interrupt bit. */
	inno_usbphy_update_bits(x, CPM_USBRDT, USBRDT_RESUME_INTERCLR, 0);

	spin_unlock_irqrestore(&iphy->phy_lock, flags);

	return 0;
}

static int iphy_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (!otg)
		return -ENODEV;
	otg->host = host;
	return 0;
}

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
static int iphy_set_peripheral(struct usb_otg *otg, struct usb_gadget *gadget)
{
	if (!otg)
		return -ENODEV;

	otg->gadget = gadget;
#ifdef CONFIG_USB_DWC2_DETECT_CONNECT
	struct ingenic_usb_phy *iphy = container_of(otg->usb_phy, struct ingenic_usb_phy, phy);
	if(gadget){
		iphy->usb_dete_state = ~iphy->usb_dete_state;
		schedule_delayed_work(&iphy->work, msecs_to_jiffies(100));
	}
#endif
	return 0;
}
#else
#define iphy_set_peripheral NULL
#endif

static int iphy_notify_connect(struct usb_phy *x,
		enum usb_device_speed speed)
{
	struct ingenic_usb_phy *iphy = container_of(x, struct ingenic_usb_phy, phy);

	iphy->roothub_port_speed = speed;

	return 0;
}

static int usb_phy_ingenic_probe(struct platform_device *pdev)
{
	struct ingenic_usb_phy *iphy;
	struct resource *res;
	const struct of_device_id *match;
	int ret;
#ifdef CONFIG_USB_DWC2_DETECT_CONNECT
	int vbus;
#endif

	iphy = (struct ingenic_usb_phy *)
		devm_kzalloc(&pdev->dev, sizeof(*iphy), GFP_KERNEL);
	if (!iphy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	iphy->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(iphy->base))
		return PTR_ERR(iphy->base);
	match = of_match_node(of_matchs, pdev->dev.of_node);
	if (!match)
	        return -ENODEV;
	iphy->iphy_priv = (struct ingenic_usb_phy_priv*)match->data;
	iphy->phy.init = iphy->iphy_priv->phyinit;
	iphy->phy.shutdown = iphy_shutdown;
	iphy->phy.dev = &pdev->dev;
	iphy->regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, NULL);
	if (IS_ERR(iphy->regmap)) {
		dev_err(&pdev->dev, "failed to find regmap for usb phy %ld\n", PTR_ERR(iphy->regmap));
		return PTR_ERR(iphy->regmap);
	}

	spin_lock_init(&iphy->phy_lock);

#ifdef CONFIG_USB_DWC2_DETECT_CONNECT
	iphy->gpiod_id = devm_gpiod_get_optional(&pdev->dev,"ingenic,id-dete", GPIOD_IN);
	iphy->gpiod_vbus = devm_gpiod_get_optional(&pdev->dev,"ingenic,vbus-dete", GPIOD_ASIS);
	if (iphy->gpiod_id || iphy->gpiod_vbus) {
		INIT_DELAYED_WORK(&iphy->work, usb_dete_work);
	}


	if (!IS_ERR_OR_NULL(iphy->gpiod_id)) {
		ret = devm_request_irq(&pdev->dev,
				gpiod_to_irq(iphy->gpiod_id),
				usb_dete_irq_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				"id_dete",
				(void *)iphy);
		if (ret)
			return ret;
	} else {
		iphy->usb_dete_state |= USB_DETE_ID;
		iphy->gpiod_id = NULL;
	}

	if (!IS_ERR_OR_NULL(iphy->gpiod_vbus)) {
		ret = devm_request_irq(&pdev->dev,
				gpiod_to_irq(iphy->gpiod_vbus),
				usb_dete_irq_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				"vbus_dete",
				(void *)iphy);
		if (ret)
			return ret;
		vbus = gpiod_get_value_cansleep(iphy->gpiod_vbus);
		if(vbus)
			iphy->usb_dete_state |= USB_DETE_VBUS;
		else
			iphy->usb_dete_state &= ~USB_DETE_VBUS;
	} else {
		iphy->usb_dete_state &= ~USB_DETE_VBUS;
		iphy->gpiod_vbus = NULL;
	}
#else
	iphy->usb_dete_state |= USB_DETE_ID;
	iphy->gpiod_id = NULL;

	iphy->usb_dete_state &= ~USB_DETE_VBUS;
	iphy->gpiod_vbus = NULL;
#endif
	iphy->gpiod_drvvbus = devm_gpiod_get_optional(&pdev->dev,"ingenic,drvvbus", GPIOD_OUT_LOW);
	if (IS_ERR_OR_NULL(iphy->gpiod_drvvbus))
		iphy->gpiod_drvvbus = NULL;
	iphy->phy.set_vbus = iphy_set_vbus;
	iphy->phy.set_suspend = iphy_set_suspend;
	if (iphy->iphy_priv->phy_remote_wakeup) {
		iphy->phy.set_wakeup = iphy_set_wakeup;
	}

	iphy->phy.notify_connect = iphy_notify_connect;
	iphy->phy.otg = devm_kzalloc(&pdev->dev, sizeof(*iphy->phy.otg),
			GFP_KERNEL);
	iphy->phy.io_ops = devm_kzalloc(&pdev->dev, sizeof(*iphy->phy.io_ops),
			GFP_KERNEL);

	if (!iphy->phy.otg)
		return -ENOMEM;

	iphy->phy.type			= USB_PHY_TYPE_USB2;
	iphy->phy.otg->state		= OTG_STATE_UNDEFINED;
	iphy->phy.otg->usb_phy		= &iphy->phy;
	iphy->phy.otg->set_host		= iphy_set_host;
	iphy->phy.otg->set_peripheral	= iphy_set_peripheral;
	iphy->phy.io_ops->read		= inno_usbphy_read;
	iphy->phy.io_ops->write		= inno_usbphy_write;

	iphy->gate_clk = devm_clk_get(&pdev->dev, "gate_usbphy");
	if (IS_ERR_OR_NULL(iphy->gate_clk)){
		iphy->gate_clk = NULL;
		dev_err(&pdev->dev, "cannot get usbphy clock !\n");
		return -ENODEV;
	}

	ret = usb_add_phy_dev(&iphy->phy);
	if (ret) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
				ret);
		return ret;
	}
	platform_set_drvdata(pdev, iphy);
	pr_info("inno phy probe success\n");
	return 0;
}

static int usb_phy_ingenic_remove(struct platform_device *pdev)
{
	struct ingenic_usb_phy *iphy = platform_get_drvdata(pdev);

	usb_remove_phy(&iphy->phy);
	return 0;
}

static int usb_gpio_suspend(struct platform_device *dev, pm_message_t state)
{

	return 0;
}

static int usb_gpio_resume(struct platform_device *dev)
{
#ifdef CONFIG_USB_DWC2_DETECT_CONNECT
	struct ingenic_usb_phy *iphy = platform_get_drvdata(dev);


	schedule_delayed_work(&iphy->work, msecs_to_jiffies(100));
#endif

	return 0;
}

/* for x1600 */
struct ingenic_usb_phy_priv usb_phy_x1600_priv = {
	.phyinit = iphy_init,
	.phy_tuning_num = 8,
	.phy_tuning = {
		/* set 45ohm HS ODT value */
		[0].reg_addr = 0x11c,
		[0].msk = 0x1f,
		[0].bit_off = 0,
		[0].val = 0x1c,

		/* always enable pre-emphasis */
		[1].reg_addr = 0x30,
		[1].msk = 0x7,
		[1].bit_off = 0,
		[1].val = 0x7,

		/* Tx HS pre_emphasize strength */
		[2].reg_addr = 0x40,
		[2].msk = 0x7,
		[2].bit_off = 3,
		[2].val = 0x7,

		/* set A session valid reference tuning */
		[3].reg_addr = 0x10c,
		[3].msk = 0x7,
		[3].bit_off = 0,
		[3].val = 0x5,

		/* set B session valid reference tuning */
		[4].reg_addr = 0x10c,
		[4].msk = 0x7,
		[4].bit_off = 3,
		[4].val = 0x5,

		/* set vbus valid reference tuning */
		[5].reg_addr = 0x110,
		[5].msk = 0x7,
		[5].bit_off = 0,
		[5].val = 0x5,

		/* set session end reference tuning */
		[6].reg_addr = 0x110,
		[6].msk = 0x7,
		[6].bit_off = 3,
		[6].val = 0x5,

		/* enable control of full/low speed driver at the receiver */
		/* NOTE: Set to 0 during suspend, it is used to reducepower consumption. Restore here. */
		[7].reg_addr = 0x100,
		[7].msk = 0x1,
		[7].bit_off = 6,
		[7].val = 0x1,
	},
	.phy_remote_wakeup = 1,
	.phy_vbus_detect_ctl = 1,
	.phy_ls_fs_driver_ctl = 1,
};

/* for x2500 */
struct ingenic_usb_phy_priv usb_phy_x2500_priv = {
	.phyinit = iphy_init_x2500,
	.phy_tuning_num = 0,
	.phy_remote_wakeup = 0,
	.phy_vbus_detect_ctl = 0,
	.phy_ls_fs_driver_ctl = 0,
};

/* usb_phy_common for x2000/x2000E x2100 m300 */
struct ingenic_usb_phy_priv usb_phy_common_priv = {
	.phyinit = iphy_init,
	.phy_tuning_num = 0,
	.phy_remote_wakeup = 0,
	.phy_vbus_detect_ctl = 1,
	.phy_ls_fs_driver_ctl = 0,
};

static const struct of_device_id of_matchs[] = {
	{ .compatible = "ingenic,innophy",.data = (void*)&usb_phy_common_priv},
	{ .compatible = "ingenic,innophy-x1600",.data = (void*)&usb_phy_x1600_priv},
	{ .compatible = "ingenic,innophy-x2500",.data = (void*)&usb_phy_x2500_priv},
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_xceiv_dt_ids);

static struct platform_driver usb_phy_ingenic_driver = {
	.probe		= usb_phy_ingenic_probe,
	.remove		= usb_phy_ingenic_remove,
	.suspend	= usb_gpio_suspend,
	.resume		= usb_gpio_resume,
	.driver		= {
		.name	= "usb_phy",
		.of_match_table = of_matchs,
	},
};

static int __init usb_phy_ingenic_init(void)
{
	return platform_driver_register(&usb_phy_ingenic_driver);
}
subsys_initcall(usb_phy_ingenic_init);

static void __exit usb_phy_ingenic_exit(void)
{
	platform_driver_unregister(&usb_phy_ingenic_driver);
}
module_exit(usb_phy_ingenic_exit);
