
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/usb/role.h>

#include "phy-ingenic.h"

/*
 * This driver relies on "both edges" triggering.  VBUS has 100 msec to
 * stabilize, so the peripheral controller driver may need to cope with
 * some bouncing due to current surges (e.g. charging local capacitance)
 * and contact chatter.
 *
 * REVISIT in desperate straits, toggling between rising and falling
 * edges might be workable.
 */
#define VBUS_IRQ_FLAGS \
	(IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)

#define ID_IRQ_FLAGS \
	(IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)

#define WAKEUP_IRQ_FLAGS \
	(IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)


static const struct of_device_id of_matchs[] = {
	{ .compatible = "ingenic,usbphy-x1000", .data = (void*)&usb_phy_x1000_priv},
	{ .compatible = "ingenic,usbphy-x1600", .data = (void*)&usb_phy_x1600_priv},
	{ .compatible = "ingenic,usbphy-x2000", .data = (void*)&usb_phy_x2000_priv},
	{ .compatible = "ingenic,usbphy-m300", .data = (void*)&usb_phy_x2000_priv},
	{ },
};

static int common_phy_init(struct usb_phy *phy)
{
	int ret = 0;
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);

	if (usb_phy->gate_clk)
		clk_prepare_enable(usb_phy->gate_clk);

	msleep(5);

	if (usb_phy->phy_priv->phy_init)
		ret = usb_phy->phy_priv->phy_init(usb_phy);

	return ret;
}

static void common_phy_shutdown(struct usb_phy *phy)
{
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);

	if (usb_phy->phy_priv->phy_shutdown)
		usb_phy->phy_priv->phy_shutdown(usb_phy);

	if (usb_phy->gate_clk)
		clk_disable_unprepare(usb_phy->gate_clk);
}

static int common_phy_set_suspend(struct usb_phy *phy, int suspend)
{
	int ret = 0;
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);

	if (usb_phy->phy_priv->phy_set_suspend)
		ret = usb_phy->phy_priv->phy_set_suspend(usb_phy, suspend);

	return ret;
}

static int common_phy_set_wakeup(struct usb_phy *phy, bool enabled)
{
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);

	if (enabled){
		if (usb_phy->phy_priv->phy_set_wakeup)
			usb_phy->phy_priv->phy_set_wakeup(usb_phy, 1);

		/* change usb switch to gpio */
		if (usb_phy->switch_gpiod)
			gpiod_set_value(usb_phy->switch_gpiod, 1);

		/* enable usb wakeup irq */
		if (usb_phy->wakeup_gpiod) {
#ifdef TEST_USB_WAKEUP_FLAG
			usb_phy->wakeup_flag = 0;
#endif
			enable_irq(usb_phy->wakeup_irq);
			enable_irq_wake(usb_phy->wakeup_irq);
		}
	} else {
		/* disable usb wakeup irq */
		if (usb_phy->wakeup_gpiod) {
			disable_irq_wake(usb_phy->wakeup_irq);
			disable_irq(usb_phy->wakeup_irq);
		}

		/* change usb switch to phy */
		if (usb_phy->switch_gpiod)
			gpiod_set_value(usb_phy->switch_gpiod, 0);

		if (usb_phy->phy_priv->phy_set_wakeup)
			usb_phy->phy_priv->phy_set_wakeup(usb_phy, 0);
	}

	return 0;
}

static int common_phy_get_wakeup(struct usb_phy *phy)
{
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);
	int ret = 0;

	if (usb_phy->phy_priv->phy_get_wakeup)
		ret = usb_phy->phy_priv->phy_get_wakeup(usb_phy);

#ifdef TEST_USB_WAKEUP_FLAG
	if (usb_phy->wakeup_flag) {
		usb_phy->wakeup_flag = 0;
		ret = 1;
	}
#endif

	return ret;
}

static irqreturn_t usb_wakeup_irq_handler(int irq, void *data)
{
#ifdef TEST_USB_WAKEUP_FLAG
	struct usb_phy_data *usb_phy = (struct usb_phy_data *)data;
	usb_phy->wakeup_flag = 1;
#endif

	return IRQ_HANDLED;
}

static int is_vbus_powered(struct usb_phy_data *usb_phy)
{
	return gpiod_get_value(usb_phy->vbus_gpiod);
}

static int is_id_host(struct usb_phy_data *usb_phy)
{
	return gpiod_get_value(usb_phy->id_gpiod);
}

static void gpio_vbus_work(struct work_struct *work)
{
	struct usb_phy_data *usb_phy =
		container_of(work, struct usb_phy_data, vbus_work.work);
	int status, vbus;

	if (!usb_phy->phy.otg->gadget)
		return;

	vbus = is_vbus_powered(usb_phy);
	if ((vbus ^ usb_phy->vbus) == 0)
		return;
	usb_phy->vbus = vbus;

	/* Peripheral controllers which manage the pullup themselves won't have
	 * a pullup GPIO configured here.  If it's configured here, we'll do
	 * what isp1301_omap::b_peripheral() does and enable the pullup here...
	 * although that may complicate usb_gadget_{,dis}connect() support.
	 */

	if (vbus) {
		status = USB_EVENT_VBUS;
		usb_phy->phy.otg->state = OTG_STATE_B_PERIPHERAL;
		usb_phy->phy.last_event = status;
		usb_gadget_vbus_connect(usb_phy->phy.otg->gadget);

		atomic_notifier_call_chain(&usb_phy->phy.notifier,
					   status, usb_phy->phy.otg->gadget);
		usb_phy_set_event(&usb_phy->phy, USB_EVENT_ENUMERATED);
	} else {
		usb_gadget_vbus_disconnect(usb_phy->phy.otg->gadget);
		status = USB_EVENT_NONE;
		usb_phy->phy.otg->state = OTG_STATE_B_IDLE;
		usb_phy->phy.last_event = status;

		atomic_notifier_call_chain(&usb_phy->phy.notifier,
					   status, usb_phy->phy.otg->gadget);
		usb_phy_set_event(&usb_phy->phy, USB_EVENT_NONE);
	}
}

/* VBUS change IRQ handler */
static irqreturn_t gpio_vbus_irq(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct usb_phy_data *usb_phy = platform_get_drvdata(pdev);
	struct usb_otg *otg = usb_phy->phy.otg;

	dev_dbg(&pdev->dev, "VBUS %s (gadget: %s)\n",
		is_vbus_powered(usb_phy) ? "supplied" : "inactive",
		otg->gadget ? otg->gadget->name : "none");

	if (otg->gadget)
		schedule_delayed_work(&usb_phy->vbus_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static void gpio_id_work(struct work_struct *work)
{
	struct usb_phy_data *usb_phy =
		container_of(work, struct usb_phy_data, id_work.work);
	const struct software_node *swnode;
	struct fwnode_handle *fwnode;
	struct usb_role_switch *role_sw;
	int id;

	if (!usb_phy->phy.otg->gadget)
		return;

	id = is_id_host(usb_phy);
	if ((id ^ usb_phy->id) == 0)
		return;
	usb_phy->id = id;

	swnode = software_node_find_by_name(NULL, "ingenic-usb-sw");
	if (!swnode)
		return;

	fwnode = software_node_fwnode(swnode);
	role_sw = usb_role_switch_find_by_fwnode(fwnode);
	fwnode_handle_put(fwnode);

	if (role_sw) {
		usb_role_switch_set_role(role_sw, id ? USB_ROLE_HOST : USB_ROLE_DEVICE);
		usb_role_switch_put(role_sw);
	}

}

/* ID change IRQ handler */
static irqreturn_t gpio_id_irq(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct usb_phy_data *usb_phy = platform_get_drvdata(pdev);
	struct usb_otg *otg = usb_phy->phy.otg;

	dev_dbg(&pdev->dev, "ID %s (gadget: %s)\n",
		is_id_host(usb_phy) ? "host" : "device",
		otg->gadget ? otg->gadget->name : "none");

	if (otg->gadget)
		schedule_delayed_work(&usb_phy->id_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int usb_phy_set_vbus(struct usb_phy *phy, int on)
{
	struct usb_phy_data *usb_phy = container_of(phy, struct usb_phy_data, phy);

	dev_dbg(usb_phy->dev, "OTG VBUS %s\n", on ? "ON" : "OFF");

	if (usb_phy->drvvbus_gpiod)
		gpiod_set_value(usb_phy->drvvbus_gpiod, on);

	return 0;
}

/* bind/unbind the peripheral controller */
static int usb_phy_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	struct usb_phy_data *usb_phy;
	struct platform_device *pdev;

	usb_phy = container_of(otg->usb_phy, struct usb_phy_data, phy);
	pdev = to_platform_device(usb_phy->dev);

	if (!gadget) {
		dev_dbg(&pdev->dev, "unregistering gadget '%s'\n",
			otg->gadget->name);

		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_UNDEFINED;

		otg->gadget = NULL;
		return 0;
	}

	otg->gadget = gadget;
	dev_dbg(&pdev->dev, "registered gadget '%s'\n", gadget->name);

	/* initialize connection state */
	usb_phy->vbus = 0; /* start with disconnected */
	if (usb_phy->vbus_gpiod)
		gpio_vbus_irq(usb_phy->vbus_irq, pdev);
	return 0;
}

/* platform driver interface */
static int ingenic_usb_phy_probe(struct platform_device *pdev)
{
	struct usb_phy_data *usb_phy;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	int err;

	usb_phy = devm_kzalloc(dev, sizeof(struct usb_phy_data),
				 GFP_KERNEL);
	if (!usb_phy)
		return -ENOMEM;

	usb_phy->phy.otg = devm_kzalloc(dev, sizeof(struct usb_otg),
					  GFP_KERNEL);
	if (!usb_phy->phy.otg)
		return -ENOMEM;

	usb_phy->cpm_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usb_phy->cpm_base))
		return PTR_ERR(usb_phy->cpm_base);

	usb_phy->phy_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(usb_phy->phy_base))
		dev_warn(dev, "dont have usb phy base addr !\n");

	match = of_match_node(of_matchs, dev->of_node);
	if (!match)
	        return -ENODEV;
	usb_phy->phy_priv = (struct usb_phy_priv*)match->data;

	platform_set_drvdata(pdev, usb_phy);
	usb_phy->dev = dev;
	usb_phy->phy.label = "ingenic_usb_phy";
	usb_phy->phy.dev = dev;
	usb_phy->phy.set_vbus = usb_phy_set_vbus;

	usb_phy->phy.init = common_phy_init;
	usb_phy->phy.shutdown = common_phy_shutdown;
	usb_phy->phy.set_suspend = common_phy_set_suspend;
	usb_phy->phy.set_wakeup = common_phy_set_wakeup;
	usb_phy->phy.get_wakeup = common_phy_get_wakeup;

	usb_phy->phy.otg->state = OTG_STATE_UNDEFINED;
	usb_phy->phy.otg->usb_phy = &usb_phy->phy;
	usb_phy->phy.otg->set_peripheral = usb_phy_set_peripheral;

	usb_phy->gate_clk = devm_clk_get(&pdev->dev, "gate_usbphy");
	if (IS_ERR(usb_phy->gate_clk)){
		usb_phy->gate_clk = NULL;
		dev_err(dev, "cannot get usbphy clock !\n");
		return -ENODEV;
	}

	usb_phy->drvvbus_gpiod = devm_gpiod_get_optional(dev, "ingenic,drvvbus", GPIOD_OUT_LOW);
	if (IS_ERR(usb_phy->drvvbus_gpiod)) {
		err = PTR_ERR(usb_phy->drvvbus_gpiod);
		dev_err(dev, "can't request vbus draw gpio, err: %d\n", err);
		return err;
	}
	if (usb_phy->drvvbus_gpiod)
		gpiod_set_consumer_name(usb_phy->drvvbus_gpiod, "drvvbus");

	usb_phy->vbus_gpiod = devm_gpiod_get_optional(dev, "ingenic,vbus-dete", GPIOD_IN);
	if (IS_ERR(usb_phy->vbus_gpiod)) {
		err = PTR_ERR(usb_phy->vbus_gpiod);
		dev_err(dev, "can't request vbus gpio, err: %d\n", err);
		return err;
	}

	if (usb_phy->vbus_gpiod) {
		gpiod_set_consumer_name(usb_phy->vbus_gpiod, "vbus_detect");
		INIT_DELAYED_WORK(&usb_phy->vbus_work, gpio_vbus_work);
		usb_phy->vbus_irq = gpiod_to_irq(usb_phy->vbus_gpiod);

		err = devm_request_irq(dev, usb_phy->vbus_irq, gpio_vbus_irq, VBUS_IRQ_FLAGS,
					"vbus_detect", pdev);
		if (err) {
			dev_err(dev, "can't request vbus irq %i, err: %d\n",
				usb_phy->vbus_irq, err);
			return err;
		}
	}

	usb_phy->id_gpiod = devm_gpiod_get_optional(dev, "ingenic,id-dete", GPIOD_IN);
	if (IS_ERR(usb_phy->id_gpiod)) {
		err = PTR_ERR(usb_phy->id_gpiod);
		dev_err(dev, "can't request id gpio, err: %d\n", err);
		return err;
	}

	if (usb_phy->id_gpiod) {
		gpiod_set_consumer_name(usb_phy->id_gpiod, "id_detect");
		INIT_DELAYED_WORK(&usb_phy->id_work, gpio_id_work);
		usb_phy->id_irq = gpiod_to_irq(usb_phy->id_gpiod);

		err = devm_request_irq(dev, usb_phy->id_irq, gpio_id_irq, ID_IRQ_FLAGS,
					"id_detect", pdev);
		if (err) {
			dev_err(dev, "can't request id irq %i, err: %d\n",
				usb_phy->id_irq, err);
			return err;
		}
	}

	device_init_wakeup(dev, true);

	if (of_find_property(dev->of_node, "enable-vbus-wakeup", NULL))
		usb_phy->vbus_wakeup = true;

	if (of_find_property(dev->of_node, "enable-id-wakeup", NULL))
		usb_phy->id_wakeup = true;

	if (of_find_property(dev->of_node, "enable-usb-wakeup", NULL))
		usb_phy->usb_wakeup = true;

	if (usb_phy->usb_wakeup) {
		usb_phy->switch_gpiod = devm_gpiod_get_optional(dev, "ingenic,usb-switch", GPIOD_OUT_LOW);
		if (IS_ERR(usb_phy->switch_gpiod)) {
			err = PTR_ERR(usb_phy->switch_gpiod);
			dev_err(dev, "can't request usb switch gpio, err: %d\n", err);
			return err;
		}
		if (usb_phy->switch_gpiod)
			gpiod_set_consumer_name(usb_phy->switch_gpiod, "usb_switch");

		usb_phy->wakeup_gpiod = devm_gpiod_get_optional(dev, "ingenic,usb-wakeup", GPIOD_IN);
		if (IS_ERR(usb_phy->wakeup_gpiod)) {
			err = PTR_ERR(usb_phy->wakeup_gpiod);
			dev_err(dev, "can't request usb wakeup gpio, err: %d\n", err);
			return err;
		}

		if (usb_phy->wakeup_gpiod) {
			gpiod_set_consumer_name(usb_phy->wakeup_gpiod, "usb_wakeup");
			usb_phy->wakeup_irq = gpiod_to_irq(usb_phy->wakeup_gpiod);
			err = devm_request_irq(dev, usb_phy->wakeup_irq, usb_wakeup_irq_handler,
					WAKEUP_IRQ_FLAGS, "usb_wakeup", usb_phy);
			if (err) {
				dev_err(dev, "can't request usb wakeup irq, err: %d\n", err);
				return err;
			}

			disable_irq(usb_phy->wakeup_irq);
		}
	}

	err = usb_phy->phy_priv->priv_data_init(usb_phy);
	if (err) {
		dev_err(dev, "priv_data_init fail, err: %d\n", err);
		return err;
	}

	/* only active when a gadget is registered */
	err = usb_add_phy(&usb_phy->phy, USB_PHY_TYPE_USB2);
	if (err) {
		dev_err(dev, "can't register transceiver, err: %d\n", err);
		return err;
	}

	return 0;
}

static int ingenic_usb_phy_remove(struct platform_device *pdev)
{
	struct usb_phy_data *usb_phy = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

	if (usb_phy->vbus_gpiod)
		cancel_delayed_work_sync(&usb_phy->vbus_work);

	if (usb_phy->id_gpiod)
		cancel_delayed_work_sync(&usb_phy->id_work);

	usb_phy->phy_priv->priv_data_exit(usb_phy);

	usb_remove_phy(&usb_phy->phy);

	return 0;
}

#ifdef CONFIG_PM
static int ingenic_usb_phy_pm_suspend(struct device *dev)
{
	struct usb_phy_data *usb_phy = dev_get_drvdata(dev);

	if (usb_phy->vbus_wakeup && usb_phy->vbus_gpiod)
		enable_irq_wake(usb_phy->vbus_irq);

	if (usb_phy->id_wakeup && usb_phy->id_gpiod)
		enable_irq_wake(usb_phy->id_irq);

	return 0;
}

static int ingenic_usb_phy_pm_resume(struct device *dev)
{
	struct usb_phy_data *usb_phy = dev_get_drvdata(dev);

	if (usb_phy->vbus_wakeup && usb_phy->vbus_gpiod)
		disable_irq_wake(usb_phy->vbus_irq);

	if (usb_phy->id_wakeup && usb_phy->id_gpiod)
		disable_irq_wake(usb_phy->id_irq);

	return 0;
}

static const struct dev_pm_ops ingenic_usb_phy_pm_ops = {
	.suspend	= ingenic_usb_phy_pm_suspend,
	.resume		= ingenic_usb_phy_pm_resume,
};
#endif

static struct platform_driver ingenic_usb_phy_driver = {
	.driver = {
		.name  = "ingenic_usb_phy",
		.of_match_table = of_matchs,
#ifdef CONFIG_PM

		.pm = &ingenic_usb_phy_pm_ops,
#endif
	},
	.probe		= ingenic_usb_phy_probe,
	.remove		= ingenic_usb_phy_remove,
};

module_platform_driver(ingenic_usb_phy_driver);
