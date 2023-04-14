#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>

#include "ft6236_touch.h"

static const struct i2c_device_id ft6236_tpd_id[] = {{TPD_DRIVER_NAME,0},{}};

static int ft_parse_dt(struct device *dev,
			struct ft6236_platform_data *pdata)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "irq-flags",
				   &pdata->irq_flags);
	if (ret) {
		dev_info(dev,
			 "Failed get int-trigger-type from dts,set default\n");
	}

	ret = of_property_read_u32(np, "touchscreen-max-x",
				   &pdata->max_x);
	if (ret) {
		dev_info(dev,
			 "Failed get max_x from dts,set default\n");
	}

	ret = of_property_read_u32(np, "touchscreen-max-y",
				   &pdata->max_y);
	if (ret) {
		dev_info(dev,
			 "Failed get max_y from dts,set default\n");
	}

	ret = of_property_read_u32(np, "touchscreen-invert-x",
				   &pdata->invert_x);
	if (ret) {
		dev_info(dev,
			 "Failed get invert_x from dts,set default\n");
	}

	ret = of_property_read_u32(np, "touchscreen-invert-y",
				   &pdata->invert_y);
	if (ret) {
		dev_info(dev,
			 "Failed get invert_y from dts,set default\n");
	}

	of_property_read_u32(np, "ft6236,swap-xy", &pdata->swap_xy);
	if (pdata->swap_xy)
		dev_info(dev, "swap-xy enabled\n");

	pdata->irq_gpio = of_get_named_gpio(np, "irq-gpios", 0);
	if (!gpio_is_valid(pdata->irq_gpio))
		dev_err(dev, "No valid irq gpio");

	pdata->rst_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(pdata->rst_gpio))
		dev_err(dev, "No valid rst gpio");

	return 0;
}

static int ft_power_init(struct ft6236_data *ts)
{
	int ret;

	ts->vdd_ana = regulator_get(&ts->client->dev, "vdd_ana");
	if (IS_ERR(ts->vdd_ana)) {
		ts->vdd_ana = NULL;
		ret = PTR_ERR(ts->vdd_ana);
		dev_info(&ts->client->dev,
			 "Regulator get failed vdd ret=%d\n", ret);
	}

	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ts->vcc_i2c = NULL;
		ret = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev,
			 "Regulator get failed vcc_i2c ret=%d\n", ret);
	}
	return 0;
}

static int ft_power_deinit(struct ft6236_data *ts)
{
	if (ts->vdd_ana)
		regulator_put(ts->vdd_ana);
	if (ts->vcc_i2c)
		regulator_put(ts->vcc_i2c);

	return 0;
}

static int ft_power_on(struct ft6236_data *ts)
{
	int ret = 0;

	if (ts->vdd_ana) {
		ret = regulator_enable(ts->vdd_ana);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed ret=%d\n",
				ret);
			goto err_enable_vdd_ana;
		}
	}

	if (ts->vcc_i2c) {
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vcc_i2c enable failed ret=%d\n",
				ret);
			goto err_enable_vcc_i2c;
		}
	}
	clear_bit(6, &ts->flags);
	return 0;

err_enable_vcc_i2c:
	if (ts->vdd_ana)
		regulator_disable(ts->vdd_ana);
err_enable_vdd_ana:
	set_bit(6, &ts->flags);
	return ret;
}

static int ft6236_chip_power(struct ft6236_data *ft6236)
{
	int ret = 0;

	if (NULL != ft6236->vdd_ana){
		ret = regulator_enable(ft6236->vdd_ana);
		if(ret){
			dev_err(&ft6236->client->dev,
				"Regulator vdd enable failed ret=%d\n",
				ret);
			goto err_enable_vdd_ana;
		}
	}
	if (NULL != ft6236->vcc_i2c) {
		ret = regulator_enable(ft6236->vcc_i2c);
		if (ret) {
			dev_err(&ft6236->client->dev,
				"Regulator vcc_i2c enable failed ret=%d\n",
				ret);
			goto err_enable_vcc_i2c;
		}
	}

	return 0;

err_enable_vcc_i2c:
	if (ft6236->vdd_ana)
		regulator_disable(ft6236->vdd_ana);
err_enable_vdd_ana:
/*	set_bit(POWER_OFF_MODE, &ft6236->flags);*/
	return ret;
}

static void ft6236_chip_reset(struct ft6236_data *ft6236)
{
	if (ft6236->pdata->rst_gpio) {
		gpio_direction_output(ft6236->pdata->rst_gpio, 0);
		msleep(50);
		gpio_direction_output(ft6236->pdata->rst_gpio, 1);
		msleep(300);
	}
}


static int ft6236_read(struct i2c_client *client, u8 address, u8 len, u8 *buf)
{
	unsigned int transfer_length = 0;
	unsigned int pos = 0;
	unsigned char get_buf[64], addr_buf[2];
	int retry, r = 2;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = !I2C_M_RD,
			.buf = &addr_buf[0],
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
		}
	};

	if (likely(len < sizeof(get_buf))) {
		/* code optimize, use stack memory */
		msgs[1].buf = &get_buf[0];
	} else {
		msgs[1].buf = kzalloc(len > I2C_MAX_TRANSFER_SIZE
				? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
		if (!msgs[1].buf)
			return -ENOMEM;
	}

	while (pos != len) {
		if (unlikely(len - pos > I2C_MAX_TRANSFER_SIZE))
			transfer_length = I2C_MAX_TRANSFER_SIZE;
		else
			transfer_length = len - pos;
		msgs[0].buf[0] = address;
		msgs[1].len = transfer_length;
		for (retry = 0; retry < RETRY_MAX_TIMES; retry++) {
			if (likely(i2c_transfer(client->adapter, msgs, 2) == 2)) {
				memcpy(&buf[pos], msgs[1].buf, transfer_length);
				pos += transfer_length;
				address += transfer_length;
				break;
			}
			dev_dbg(&client->dev, "I2c read retry[%d]:0x%x\n",
				retry + 1, address);
			usleep_range(2000, 2100);
		}
		if (unlikely(retry == RETRY_MAX_TIMES)) {
			dev_err(&client->dev,
				"I2c read failed,dev:%02x,reg:%04x,size:%u\n",
				client->addr, address, len);
			r = -EAGAIN;
			goto read_exit;
		}
	}
read_exit:
	if (len >= sizeof(get_buf))
		kfree(msgs[1].buf);

	return r;
}

static int ft6236_i2c_test(struct i2c_client *client)
{
	u8 test;

	ft6236_read(client, 0xa8, 1, &test);
	if (0x11 == test)
		return 0;

	return -EAGAIN;
}
static int ft_request_io_port(struct ft6236_data *ts)
{
	int ret;

	if (gpio_is_valid(ts->pdata->irq_gpio)) {
		ret = gpio_request(ts->pdata->irq_gpio, "ft6236_ts_int");
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Failed to request GPIO:%d, ERRNO:%d\n",
				(s32)ts->pdata->irq_gpio, ret);
			return -ENODEV;
		}
		gpio_direction_input(ts->pdata->irq_gpio);
		dev_info(&ts->client->dev, "Success request irq-gpio\n");
	}
	if (gpio_is_valid(ts->pdata->rst_gpio)) {
		ret = gpio_request(ts->pdata->rst_gpio, "ft6236_ts_rst");
		if (ret < 0) {
			dev_err(&ts->client->dev,
				"Failed to request GPIO:%d, ERRNO:%d\n",
				(s32)ts->pdata->rst_gpio, ret);

			if (gpio_is_valid(ts->pdata->irq_gpio))
				gpio_free(ts->pdata->irq_gpio);

			return -ENODEV;
		}

		gpio_direction_input(ts->pdata->rst_gpio);
		dev_info(&ts->client->dev,  "Success request rst-gpio\n");
	}
	return 0;
}

static void ft6236_touch_report(struct ft6236_data *ft6236)
{
	u8 touches;
	int i, error;
	struct ft6236_packet buf;

	struct input_dev *input = ft6236->input;

	error = ft6236_read(ft6236->client, 0, sizeof(buf), (u8 *)&buf);
	if (error < 0) {
		printk(KERN_ERR "ft6236_touch: read touchdata failed %d\n", error);
		return;
	}

	touches = buf.touches & 0xf;

	for (i = 0; i < touches; i++) {
		struct ft6236_touchpoint *point = &buf.points[i];
		u16 x = ((point->xhi & 0xf) << 8) | buf.points[i].xlo;
		u16 y = ((point->yhi & 0xf) << 8) | buf.points[i].ylo;
		u8 id = point->id >> 4;

		if (ft6236->pdata->invert_x)
			x = ft6236->pdata->max_x - x;

		if (ft6236->pdata->invert_y)
			y = ft6236->pdata->max_y - y;

		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_MT_TOOL_TYPE, 1);
		input_report_abs(input, ABS_MT_TRACKING_ID, id);
		input_report_abs(input, ABS_MT_PRESSURE, 20);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 20);
		input_report_abs(input, ABS_MT_WIDTH_MAJOR, 20);

		input_report_key(input, BTN_TOOL_FINGER, true);

		if (ft6236->pdata->swap_xy) {
			input_report_abs(input, ABS_MT_POSITION_X, y);
			input_report_abs(input, ABS_MT_POSITION_Y, x);
		} else {
			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
		}

		input_mt_sync(input);
	}

	if (touches == 0) {
		input_report_key(input, BTN_TOUCH, 0);
		input_mt_sync(input);
	}

	input_sync(input);
}

static irqreturn_t ft6236_ts_irq_handler(int irq, void *dev_id)
{
	struct ft6236_data *ft6236 = dev_id;

	ft6236_touch_report(ft6236);

	return IRQ_HANDLED;
}

static int ft_request_irq(struct ft6236_data *ts)
{
	int ret = -1;

	if (gpio_is_valid(ts->pdata->irq_gpio))
		ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);

	dev_info(&ts->client->dev, "INT num %d, trigger type:%d\n",
		 ts->client->irq, ts->pdata->irq_flags);
	ret = request_threaded_irq(ts->client->irq, NULL,
			ft6236_ts_irq_handler,
			ts->pdata->irq_flags | IRQF_ONESHOT,
			ts->client->name,
			ts);
	if (ret < 0) {
		dev_err(&ts->client->dev,
			"Failed to request irq %d\n", ts->client->irq);
		return ret;
	}
	return ret;
}

static int ft6236_tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct input_dev *input;
	struct device *dev = &client->dev;
	struct ft6236_platform_data *pdata;
	struct ft6236_data *ft6236;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Failed check I2C functionality");
		return -ENODEV;
	}

	ft6236 = devm_kzalloc(&client->dev, sizeof(*ft6236), GFP_KERNEL);
	if (ft6236 == NULL) {
		printk(KERN_ERR "ft6236_touch: alloc ft6236 memory failed.");
		return -ENOMEM;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (ft6236 == NULL) {
		printk(KERN_ERR "ft6236_touch: alloc ft6236_platform_data memory failed.");
		devm_kfree(&client->dev, ft6236);
		return -EINVAL;
	}

	ret = ft_parse_dt(&client->dev, pdata);
	if (ret) {
		dev_err(&client->dev, "Failed parse dts\n");
		goto exit_free_client_data;
	}

	ft6236->client = client;
	ft6236->pdata = pdata;

	ret = ft_request_io_port(ft6236);
	if (ret < 0) {
		dev_err(&client->dev, "Failed request IO port\n");
		goto free_gpio;
	}

	ret = ft_power_init(ft6236);
	if (ret) {
		dev_err(&client->dev, "Failed get regulator\n");
		ret = -EINVAL;
		goto free_gpio;
	}

	i2c_set_clientdata(client, ft6236);
	ft6236_chip_power(ft6236);
	ft6236_chip_reset(ft6236);

	input = devm_input_allocate_device(dev);
	if (input == NULL) {
		ret = -ENOMEM;
		goto exit_deinit_power;
	}

	ft6236->input = input;
	input->name = TPD_DRIVER_NAME;
	input->phys = TPD_DRIVER_PHY;
	input->id.bustype = BUS_I2C;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	if (ft6236->pdata->swap_xy) {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->max_y, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->max_x, 0, 0);
	} else {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->max_x, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->max_y, 0, 0);
	}

	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_set_abs_params(input, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 479, 0, 0);

	ret = ft_request_irq(ft6236);
	if (ret < 0) {
		dev_err(&client->dev, "Failed create work thread");
		goto free_input;
	}

	ret = input_register_device(input);
	if (ret) {
		printk(KERN_ERR "ft6236_touch: failed to register input device: %d\n", ret);
		goto free_irq;
	}

	spin_lock_init(&ft6236->irq_lock);

	ret = ft6236_i2c_test(client);
	if (ret) {
		dev_err(&client->dev, "Failed communicate with IC use I2C\n");
		goto free_input_dev;
	}

	return 0;

free_input_dev:
	input_unregister_device(ft6236->input);
free_irq:
	free_irq(ft6236->client->irq, ft6236);
free_input:
	input_free_device(ft6236->input);
exit_deinit_power:
	ft_power_deinit(ft6236);
free_gpio:
	if (gpio_is_valid(ft6236->pdata->rst_gpio))
		gpio_free(ft6236->pdata->rst_gpio);
	if (gpio_is_valid(ft6236->pdata->irq_gpio))
		gpio_free(ft6236->pdata->irq_gpio);
exit_free_client_data:
	devm_kfree(&client->dev, pdata);
	devm_kfree(&client->dev, ft6236);
	i2c_set_clientdata(client, NULL);

	return ret;
}

static int ft6236_tpd_remove(struct i2c_client *client)
{
	struct ft6236_data *ft6236 = i2c_get_clientdata(client);

	free_irq(ft6236->client->irq, ft6236);

	if (gpio_is_valid(ft6236->pdata->rst_gpio))
		gpio_free(ft6236->pdata->rst_gpio);
	if (gpio_is_valid(ft6236->pdata->irq_gpio))
		gpio_free(ft6236->pdata->irq_gpio);

	input_unregister_device(ft6236->input);
	input_free_device(ft6236->input);
	kfree(ft6236);

	return 0;
}

static int ft6236_tpd_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct ft6236_data *ft6236 = i2c_get_clientdata(client);
    unsigned long irqflags;

    spin_lock_irqsave(&ft6236->irq_lock, irqflags);
    disable_irq(client->irq);
    spin_unlock_irqrestore(&ft6236->irq_lock, irqflags);
	return 0;
}

static int ft6236_tpd_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct ft6236_data *ft6236 = i2c_get_clientdata(client);
    unsigned long irqflags;

    spin_lock_irqsave(&ft6236->irq_lock, irqflags);
    enable_irq(client->irq);
    spin_unlock_irqrestore(&ft6236->irq_lock, irqflags);
	return 0;
}

static const struct dev_pm_ops ft6236_pm_ops = {
	.suspend = ft6236_tpd_suspend,
	.resume  = ft6236_tpd_resume,
};

static const struct of_device_id ft_match_table[] = {
	{.compatible = "goodix,ft6236",},
	{ },
};

static struct i2c_driver ft6236_ts_driver = {
	.driver = {
		.name = TPD_DRIVER_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = ft_match_table,
		.pm = &ft6236_pm_ops,
	},

	.probe    = ft6236_tpd_probe,
	.remove   = ft6236_tpd_remove,
	.id_table = ft6236_tpd_id,
};

static int __init ft6236_touch_init(void)
{
	int ret = i2c_add_driver(&ft6236_ts_driver);
	if (ret) {
		printk(KERN_ERR "ft6236_touch: failed to register i2c driver\n");
		return ret;
	}

	return 0;
}

static void __exit ft6236_touch_exit(void)
{
	i2c_del_driver(&ft6236_ts_driver);
}

module_init(ft6236_touch_init);
module_exit(ft6236_touch_exit);

MODULE_DESCRIPTION("Ft6236 Series Driver");
MODULE_LICENSE("GPL");
