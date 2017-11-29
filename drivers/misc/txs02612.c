/*
 * txs02612.c
 * Driver for controlling the txs02612 sd level shifter and switch.
 *
 * currently, this is a very simple switch - it just provides a /sysfs node
 * to throw the switch.
 *
 * in the future this should become a MMC card driver (similar to e.g. WLAN
 * chips) and register as two new MMC ports to the system.
 *
 * switching should be done on the fly, i.e. block by block request, so that
 * both cards can be read or written in parallel and simply appear as two
 * separate drives (mmcblk) in user space.
 *
 * so we should probably convert to use sdio_register_driver()
 * maybe this is a hint: http://www.varsanofiev.com/inside/WritingLinuxSDIODrivers.htm
 * and of course http://lxr.free-electrons.com/source/drivers/mmc/core/
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#undef pr_debug
#define pr_debug printk

struct txs_data {
	int			control_gpio;	/* the control gpio number */
	struct mmc_host		*mmc[2];
};

struct tsx0612_mmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
#if STUFF
	struct	clk		*fclk;
	struct	clk		*dbclk;
	struct	regulator	*pbias;
	bool			pbias_enabled;
	void	__iomem		*base;
	int			vqmmc_enabled;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	int			suspended;
	u32			con;
	u32			hctl;
	u32			sysctl;
	u32			capa;
	int			irq;
	int			wake_irq;
	int			use_dma, dma_ch;
	struct dma_chan		*tx_chan;
	struct dma_chan		*rx_chan;
	int			response_busy;
	int			context_loss;
	int			protect_card;
	int			reqs_blocked;
	int			req_in_progress;
	unsigned long		clk_rate;
	unsigned int		flags;
#define AUTO_CMD23		(1 << 0)        /* Auto CMD23 support */
#define HSMMC_SDIO_IRQ_ENABLED	(1 << 1)        /* SDIO irq enabled */
	struct omap_hsmmc_next	next_data;
	struct	omap_hsmmc_platform_data	*pdata;

	/* return MMC cover switch state, can be NULL if not supported.
	 *
	 * possible return values:
	 *   0 - closed
	 *   1 - open
	 */
	int (*get_cover_state)(struct device *dev);

	int (*card_detect)(struct device *dev);
#endif
};

// FIXME: throwing the switch manually makes no more sense
static ssize_t set_switch(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct txs_data *data = dev_get_drvdata(dev);
	unsigned long val;
// FIXME: we could decode the state as string (eMMC vs uSD)
	int err = kstrtoul(buf, 10, &val);

	pr_debug("%s() to %ld\n", __func__, val);

	if (err)
		return err;
	if (val > 1)
		return -EINVAL;

	gpio_set_value_cansleep(data->control_gpio, val);

// maybe: mmc_detect_change(host, delay);

	return count;
}

static ssize_t show_switch(struct device *dev,
			struct device_attribute *attr, char *buf)
	{
	struct txs_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", gpio_get_value(data->control_gpio)?"uSD":"eMMC");
	}

static DEVICE_ATTR(switch, S_IWUSR | S_IRUGO,
		show_switch, set_switch);

static struct attribute *txs_attributes[] = {
	&dev_attr_switch.attr,
	NULL
};

static const struct attribute_group txs_attr_group = {
	.attrs = txs_attributes,
};

/* downstream ops */
static struct mmc_host_ops mmc_ops = {
	.post_req = NULL,
	.pre_req = NULL,
	.request = NULL,
	.set_ios = NULL,
	.get_cd = NULL,
	.get_ro = NULL,
	.init_card = NULL,
	.enable_sdio_irq = NULL,
};

// static int txs_probe(struct platform_device *dev)
static int txs_probe(struct sdio_func *dev,
				  const struct sdio_device_id *id)
{
	struct device_node *np = dev->dev.of_node, *child;
	struct txs_data *data;
	enum of_gpio_flags flags;
	int err;
	int initial;
	int childindex = 0;

	pr_debug("%s()\n", __func__);

	data = devm_kzalloc(&dev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		goto out;
	}

//	data->if_ops = &sdio_ops;

	sdio_set_drvdata(dev, data);

	err = data->control_gpio = of_get_named_gpio_flags(np,
						"select-gpio", 0,
							&flags);

//	if (err == -EPROBE_DEFER)
//		return -EPROBE_DEFER;
	if (err < 0)
		goto out;	/* defer until we have all gpios */

	err = devm_gpio_request(&dev->dev, data->control_gpio, "txs-control");
	if (err < 0)
		goto out;

#if 1	/* no longer needed if we have dynamic switching */
	initial = of_property_read_bool(np, "switch-to-b");
	gpio_direction_output(data->control_gpio, initial);
	pr_debug("%s() initial = %d\n", __func__, initial);
#endif

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->dev.kobj, &txs_attr_group);
	if (err)
		goto out;

	pr_debug("%s() probed\n", __func__);

	/* new variant */

// base structure like drivers/net/wireless/ti/wlcore/sdio.c i.e. mmc subnode
// create Host ports like drivers/mmc/host/omap_hsmmc.c i.e. mmc interfaces

	// here stuff from wlcore/sdio.c
	// to make us a nice subnode of the omap mmc interface

	if (of_get_child_count(np) != 2) {
		dev_err(&dev->dev, "needs exactly two child nodes but found %d\n",
		of_get_child_count(np));
		return -EINVAL;
	}

	for_each_child_of_node(np, child) {
		struct platform_device *subdev;
		struct mmc_host *mmc;
		// get subnode subdev

		mmc = mmc_alloc_host(sizeof(struct tsx0612_mmc_host), &subdev->dev);
		if (!mmc) {
			err = -ENOMEM;
			goto out;
		}

		data->mmc[childindex++] = mmc;
		subdev->dev.of_node=child;

		err = mmc_of_parse(mmc);
		if (err)
			goto err1;

#if SPECIAL_OMAP_STUFF
		host = mmc_priv(mmc);
		host->mmc = mmc;
// we have no pdata...
	//	host->pdata = pdata;
		host->dev = &subdev->dev;
		host->use_dma = 0;
		host->dma_ch = -1;
		host->irq = irq;
#if DMA
		host->use_dma = 1;
		host->mapbase = res->start + pdata->reg_offset;
		host->base = base + pdata->reg_offset;
#endif
		host->power_mode = MMC_POWER_OFF;
		host->next_data.cookie = 1;
		host->pbias_enabled = 0;
		host->vqmmc_enabled = 0;

		platform_set_drvdata(subdev, host);
#endif

		mmc->ops = &mmc_ops;

		mmc->f_min = 0;
		mmc->f_max = 60000000;

#if OMAP_STUFF
		if (pdata->max_freq > 0)
			mmc->f_max = pdata->max_freq;
		else if (mmc->f_max == 0)
			mmc->f_max = OMAP_MMC_MAX_CLOCK;
#endif
		device_init_wakeup(&subdev->dev, true);

#if SPECIAL_OMAP_STUFF
		pm_runtime_enable(host->dev);
		pm_runtime_get_sync(host->dev);
		pm_runtime_set_autosuspend_delay(host->dev, MMC_AUTOSUSPEND_DELAY);
		pm_runtime_use_autosuspend(host->dev);
#endif

		/* Since we do only SG emulation, we can have as many segs
		 * as we want. */
		mmc->max_segs = 1024;

		mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
		mmc->max_blk_count = 0xFFFF;    /* No. of Blocks is 16 bits */
		mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
		mmc->max_seg_size = mmc->max_req_size;

		mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
			 MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_ERASE | MMC_CAP_CMD23;

#if OMAP_STUFF
		mmc->caps |= mmc_pdata(host)->caps;
		if (mmc->caps & MMC_CAP_8_BIT_DATA)
			mmc->caps |= MMC_CAP_4_BIT_DATA;

		if (mmc_pdata(host)->nonremovable)
			mmc->caps |= MMC_CAP_NONREMOVABLE;

		mmc->pm_caps |= mmc_pdata(host)->pm_caps;

		if (!mmc->ocr_avail)
			mmc->ocr_avail = mmc_pdata(host)->ocr_mask;

		/* register as mmc interface */
		mmc_add_host(mmc);

		if (mmc_pdata(host)->name != NULL) {
			err = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
			if (err < 0)
				goto err_slot_name;
		}

		if (host->get_cover_state) {
			err = device_create_file(&mmc->class_dev,
									 &dev_attr_cover_switch);
			if (err < 0)
				goto err_slot_name;

			}
#endif
	}

	return 0;

err1:
	while(childindex > 0) {
		mmc_remove_host(data->mmc[--childindex]);
		mmc_free_host(data->mmc[childindex]);
	}

out:
	pr_debug("%s() error %d\n", __func__, err);

	return err;
}

static void txs_remove(struct sdio_func *dev)
{
	struct txs_data *data = sdio_get_drvdata(dev);

	sysfs_remove_group(&dev->dev.kobj, &txs_attr_group);
}

static int txs_suspend(struct device *dev)
{
	struct txs_data *data = dev_get_drvdata(dev);

	return 0;
}

static int txs_resume(struct device *dev)
{
	struct txs_data *data = dev_get_drvdata(dev);

	return 0;
}

static const struct of_device_id txs_of_match[] = {
	{ .compatible = "ti,txs02612" },
	{},
};
MODULE_DEVICE_TABLE(of, txs_of_match);

SIMPLE_DEV_PM_OPS(txs_pm_ops, txs_suspend, txs_resume);

static struct sdio_driver txs_driver = {
	.probe		= txs_probe,
	.remove		= txs_remove,
	.drv = {
		.name	= "txs02612",
		.owner	= THIS_MODULE,
		.pm	= &txs_pm_ops,
		.of_match_table = of_match_ptr(txs_of_match)
	},
};

#define module_sdio_driver(NAME) static int __init NAME##_init(void) { return sdio_register_driver(&NAME); } \
	static void __exit NAME##_exit(void) { sdio_unregister_driver(&txs_driver); } \
	module_init(NAME##_init); module_exit(NAME##_exit);

module_sdio_driver(txs_driver);

// MODULE_ALIAS("platform:txs02612");

MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("txs02612 SD level shifter and switch");
MODULE_LICENSE("GPL v2");
