// SPDX-License-Identifier: GPL-2.0
/*
 * Driver core interface to the pinctrl subsystem.
 *
 * Copyright (C) 2012 ST-Ericsson SA
 * Written on behalf of Linaro for ST-Ericsson
 * Based on bits of regulator core, gpio core and clk core
 *
 * Author: Linus Walleij <linus.walleij@linaro.org>
 */

#include <linux/device.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>

/**
 * pinctrl_bind_pins() - called by the device core before probe
 * @dev: the device that is just about to probe
 */
int pinctrl_bind_pins(struct device *dev)
{
	int ret;

ll_printk("%s: 0\n", __func__);

	if (dev->of_node_reused)
		return 0;

ll_printk("%s: 1\n", __func__);

	dev->pins = devm_kzalloc(dev, sizeof(*(dev->pins)), GFP_KERNEL);
	if (!dev->pins)
		return -ENOMEM;

ll_printk("%s: 2\n", __func__);

	dev->pins->p = devm_pinctrl_get(dev);
	if (IS_ERR(dev->pins->p)) {
		dev_dbg(dev, "no pinctrl handle\n");
		ret = PTR_ERR(dev->pins->p);
		goto cleanup_alloc;
	}

ll_printk("%s: 3\n", __func__);

	dev->pins->default_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_DEFAULT);
	if (IS_ERR(dev->pins->default_state)) {
		dev_dbg(dev, "no default pinctrl state\n");
		ret = 0;
		goto cleanup_get;
	}

ll_printk("%s: 4\n", __func__);


	dev->pins->init_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_INIT);
	if (IS_ERR(dev->pins->init_state)) {
		/* Not supplying this state is perfectly legal */
ll_printk("%s: 5\n", __func__);
		dev_dbg(dev, "no init pinctrl state\n");

		ret = pinctrl_select_state(dev->pins->p,
					   dev->pins->default_state);
	} else {
ll_printk("%s: 6\n", __func__);
		ret = pinctrl_select_state(dev->pins->p, dev->pins->init_state);
	}

ll_printk("%s: 7\n", __func__);

	if (ret) {
		dev_dbg(dev, "failed to activate initial pinctrl state\n");
		goto cleanup_get;
	}

ll_printk("%s: 8\n", __func__);

#ifdef CONFIG_PM
	/*
	 * If power management is enabled, we also look for the optional
	 * sleep and idle pin states, with semantics as defined in
	 * <linux/pinctrl/pinctrl-state.h>
	 */
	dev->pins->sleep_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_SLEEP);

ll_printk("%s: 9\n", __func__);

	if (IS_ERR(dev->pins->sleep_state))
		/* Not supplying this state is perfectly legal */
		dev_dbg(dev, "no sleep pinctrl state\n");

	dev->pins->idle_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_IDLE);
	if (IS_ERR(dev->pins->idle_state))
		/* Not supplying this state is perfectly legal */
		dev_dbg(dev, "no idle pinctrl state\n");
#endif

ll_printk("%s: 10\n", __func__);

	return 0;

	/*
	 * If no pinctrl handle or default state was found for this device,
	 * let's explicitly free the pin container in the device, there is
	 * no point in keeping it around.
	 */
cleanup_get:
ll_printk("%s: 11\n", __func__);
	devm_pinctrl_put(dev->pins->p);
cleanup_alloc:
ll_printk("%s: 12\n", __func__);
	devm_kfree(dev, dev->pins);
	dev->pins = NULL;

	/* Return deferrals */
	if (ret == -EPROBE_DEFER)
		return ret;
	/* Return serious errors */
	if (ret == -EINVAL)
		return ret;
	/* We ignore errors like -ENOENT meaning no pinctrl state */

ll_printk("%s: 13\n", __func__);
	return 0;
}
