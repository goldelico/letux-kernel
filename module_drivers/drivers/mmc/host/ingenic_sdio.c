#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include<linux/regulator/consumer.h>

#if (defined(CONFIG_INGENIC_MMC_MMC0) || defined(CONFIG_INGENIC_MMC_MMC1))
#include "ingenic_mmc.h"
#else
#include "sdhci-ingenic.h"
#endif

struct wifi_data {
	int sdio_index;
	/* struct wake_lock                wifi_wake_lock; */
	/* struct regulator                *wifi_vbat; */
	/* struct regulator                *wifi_vddio; */
	uint         wifi_reset;
	uint         wifi_reset_flags;
	uint         wifi_irq;
	uint         wifi_irq_flags;
	struct pinctrl *pctrl;
	atomic_t rtc32k_ref;
	struct regulator *wlreg_on;
	struct regulator *wl_vccio;
	struct clk *clk;
};

#define RESET  0
#define NORMAL 1

struct wifi_data wifi_data;
static void rtc32k_init(struct device *dev, struct wifi_data *wdata)
{
	atomic_set(&wdata->rtc32k_ref, 0);
	wdata->pctrl = NULL;
	wdata->clk = NULL;
#ifdef CONFIG_RTC_DRV_PCF8563
	{
		struct of_phandle_args phandle;
		phandle.np = of_find_compatible_node(NULL, NULL, "nxp,pcf8563");
		if (phandle.np) {
			wdata->clk = of_clk_get_from_provider(&phandle);
		}
	}
#else
	wdata->pctrl = devm_pinctrl_get(dev);
#endif
}
void rtc32k_enable(void)
{

	if (atomic_inc_return(&wifi_data.rtc32k_ref) == 1) {
		if(wifi_data.clk)
		{
			printk("@@ rtc.pcf8563 %s @@\n", __func__);
			clk_prepare_enable(wifi_data.clk);
		}
		if(wifi_data.pctrl)
		{
			struct pinctrl_state *state = NULL;
			struct pinctrl *p = wifi_data.pctrl;
			state = pinctrl_lookup_state(p, "enable");
			if (!IS_ERR_OR_NULL(state)) {
				pinctrl_select_state(p, state);
			}
		}
	}
}
EXPORT_SYMBOL(rtc32k_enable);
void rtc32k_disable(void)
{
	if (atomic_dec_return(&wifi_data.rtc32k_ref) == 0) {
		if(wifi_data.clk){
			printk("@@ rtc.pcf8563 %s @@\n", __func__);
			clk_disable_unprepare(wifi_data.clk);
		}
		if(wifi_data.pctrl) {
			struct pinctrl_state *state = NULL;
			struct pinctrl *p = wifi_data.pctrl;
			state = pinctrl_lookup_state(p, "disable");
			if (!IS_ERR_OR_NULL(state)) {
				pinctrl_select_state(p, state);
			}
		}
	}
}
EXPORT_SYMBOL(rtc32k_disable);
static const struct of_device_id wlan_ingenic_of_match[] = {
	{.compatible = "android,bcmdhd_wlan"},
	{.compatible = "rtk,rtl8723ds_wlan"},
	{},
};

int wlan_regulator_get(struct device *dev)
{
	int ret;

	wifi_data.wlreg_on= devm_regulator_get(dev, "wlreg_on");

	if (IS_ERR(wifi_data.wlreg_on)) {
	printk("wlreg_on is error %s, %s, %d\n", __FILE__, __func__, __LINE__);
		if (PTR_ERR(wifi_data.wlreg_on) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		}
	}

	if (!IS_ERR(wifi_data.wlreg_on)) {
		ret = regulator_enable(wifi_data.wlreg_on);
		if (ret) {
			dev_err(dev, "wlreg_on enable failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "wlreg_on is invalid\n");
	}

	wifi_data.wl_vccio = devm_regulator_get(dev, "wl_vccio");

	if (IS_ERR(wifi_data.wl_vccio)) {
		printk("wl_vccio is error %s, %s, %d\n", __FILE__, __func__, __LINE__);
		if (PTR_ERR(wifi_data.wl_vccio) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		}
	}

	if (!IS_ERR(wifi_data.wl_vccio)) {
		ret = regulator_enable(wifi_data.wl_vccio);
		if (ret) {
			dev_err(dev, "wl_vccio enable failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "wl_vccio is invalid\n");
	}

	return 0;
}

int ingenic_sdio_wlan_init(struct device *dev, int index)
{
	struct device_node *np = dev->of_node, *cnp;
	unsigned int flags, gpio;

	for_each_child_of_node(np, cnp) {
		if(of_device_is_compatible(cnp, "android,bcmdhd_wlan")) {
			printk("----android,bcmdhd_wlan!\n");
			wifi_data.wifi_reset = of_get_named_gpio_flags(cnp, "ingenic,sdio-reset", 0, &flags);
			wifi_data.wifi_reset_flags = flags;
			wifi_data.wifi_irq = of_get_named_gpio_flags(cnp, "ingenic,sdio-irq", 0, &flags);
			wifi_data.wifi_irq_flags = flags;
		}
		if(of_device_is_compatible(cnp, "rtk,rtl8723ds_wlan")) {
			printk("----rtk,rtl8723ds_wlan\n");
			wifi_data.wifi_reset = of_get_named_gpio_flags(cnp, "ingenic,sdio-reset", 0, &flags);
			wifi_data.wifi_reset_flags = flags;
			wifi_data.wifi_irq = of_get_named_gpio_flags(cnp, "ingenic,sdio-irq", 0, &flags);
			wifi_data.wifi_irq_flags = flags;
		}
	}

  wlan_regulator_get(dev);

	rtc32k_init(dev, &wifi_data);

	gpio = wifi_data.wifi_reset;
	if (devm_gpio_request(dev, gpio, "wifi_reset")) {
		printk("ERROR: no wifi_reset pin available !!\n");
		return -EINVAL;
	} else {
		gpio_direction_output(gpio, !wifi_data.wifi_reset_flags);
	}
	wifi_data.wifi_reset = gpio;
	wifi_data.sdio_index = index;

	return 0;
}
EXPORT_SYMBOL(ingenic_sdio_wlan_init);

int ingenic_sdio_wlan_get_irq(unsigned long *flag)
{
	*flag = wifi_data.wifi_irq_flags;
	return wifi_data.wifi_irq;
}
EXPORT_SYMBOL(ingenic_sdio_wlan_get_irq);
int ingenic_sdio_wlan_power_onoff(int onoff, int flag)
{
	int err = 0;
	int reset = wifi_data.wifi_reset;
	if(!reset)
		return -EINVAL;
	if(onoff) {
		printk("reset %d wlan power on:%d\n", reset, flag);
		if (!IS_ERR(wifi_data.wlreg_on)) {
			err = regulator_enable(wifi_data.wlreg_on);
			if (err) {
				printk("wifi_data.wlreg_on enable failed\n");
				return err;
			}
		}
		if (!IS_ERR(wifi_data.wl_vccio)) {
			err = regulator_enable(wifi_data.wl_vccio);
			if (err) {
				printk("wifi_data.wl_vccio enable failed\n");
				return err;
			}
		}
		rtc32k_enable();
		switch(flag) {
		case RESET:
			ingenic_mmc_clk_ctrl(wifi_data.sdio_index, 1);
			gpio_set_value(reset, 0);
			msleep(10);
			gpio_set_value(reset, 1);
			break;
		case NORMAL:
			gpio_set_value(reset, 1);
			ingenic_mmc_manual_detect(wifi_data.sdio_index, 1);
			break;
		}
	} else {
		printk("wlan power off:%d\n", flag);
		switch(flag) {
		case RESET:
			gpio_set_value(reset, 0);
			break;
		case NORMAL:
			gpio_set_value(reset, 0);
			break;
		}
		rtc32k_disable();
		if (!IS_ERR(wifi_data.wlreg_on)) {
			err = regulator_disable(wifi_data.wlreg_on);
			if (err) {
				printk("wifi_data.wlreg_on disable failed\n");
				return err;
			}
		}
		if (!IS_ERR(wifi_data.wl_vccio)) {
			err = regulator_disable(wifi_data.wl_vccio);
			if (err) {
				printk("wifi_data.wl_vccio disable failed\n");
				return err;
			}
		}
	}
	if (err < 0) {
		printk("%s: regulator enable/disable failed", __FUNCTION__);
		return -1;
	}
	return 0;

}
EXPORT_SYMBOL(ingenic_sdio_wlan_power_onoff);
