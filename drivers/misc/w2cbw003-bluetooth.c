/*
 * w2scbw003.c
 * Driver for power controlling the w2cbw003 WiFi/Bluetooth chip.
 *
 * powers on the chip if the tty port associated/connected to
 * the bluetooth subsystem is opened (DTR asserted)
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/serial_core.h>
#include <linux/regulator/consumer.h>

struct w2cbw_data {
	struct		regulator *vdd_regulator;
	struct uart_port *uart;		/* the drvdata of the uart or tty */
};

/* called by uart modem control line changes (e.g. DTR) */

static void w2cbw_mctrl(void *pdata, int val)
{
	struct w2cbw_data *data = (struct w2cbw_data *) pdata;
	pr_debug("%s(...,%x)\n", __func__, val);
	
	/* DTR controls power on/off */

	if ((val & TIOCM_DTR) != 0)
		WARN_ON(regulator_enable(data->vdd_regulator));
	else
		regulator_disable(data->vdd_regulator);
}

static int w2cbw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct w2cbw_data *data;

	pr_debug("%s()\n", __func__);

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No device tree data\n");
		return EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->vdd_regulator = devm_regulator_get_optional(dev, "vdd");

	if (IS_ERR(data->vdd_regulator)) {
		if (PTR_ERR(data->vdd_regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* we can't probe yet */
		data->uart = NULL;	/* no regulator yet */
	}

	pr_debug("%s() vdd_regulator = %p\n", __func__, data->vdd_regulator);

	data->uart = devm_serial_get_uart_by_phandle(&pdev->dev, "uart", 0);

	if (!data->uart) {
		dev_err(&pdev->dev, "No UART link\n");
		return EINVAL;
	}

	if (IS_ERR(data->uart)) {
		if (PTR_ERR(data->uart) == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* we can't probe yet */
		data->uart = NULL;	/* no UART */
	}

	uart_register_slave(data->uart, data);
	uart_register_mctrl_notification(data->uart, w2cbw_mctrl);

	platform_set_drvdata(pdev, data);

	pr_debug("w2cbw003 probed\n");

	return 0;
}

static int w2cbw_remove(struct platform_device *pdev)
{
	struct w2cbw_data *data = platform_get_drvdata(pdev);

	uart_register_slave(data->uart, NULL);

	return 0;
}

static const struct of_device_id w2cbw_of_match[] = {
	{ .compatible = "wi2wi,w2cbw003-bluetooth" },
	{},
};
MODULE_DEVICE_TABLE(of, w2cbw_of_match);

static struct platform_driver w2cbw_driver = {
	.probe		= w2cbw_probe,
	.remove		= w2cbw_remove,
	.driver = {
		.name	= "w2cbw003",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(w2cbw_of_match)
	},
};

module_platform_driver(w2cbw_driver);

MODULE_ALIAS("w2cbw003-bluetooth");

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("w2cbw003 power management driver");
MODULE_LICENSE("GPL v2");
