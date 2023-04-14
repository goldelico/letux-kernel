/*
 * linux/drivers/misc/ingenic_efuse_v2.c - Ingenic efuse driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: <zhihao.xiao@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define	DEBUG
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <soc/base.h>
#include <soc/efuse.h>
#include <ingenic_proc.h>
#include <linux/delay.h>



#define DRV_NAME	"ingenic-efuse"

#define CMD_WRITE	 _IOR('E', 100, unsigned int)
#define CMD_READ	 _IOR('E', 101, unsigned int)

struct efuse_wr_info {
	uint32_t seg_id;
	uint8_t *data;
};

struct board_gpio {
	short gpio;
	short level;
};

struct jz_efuse {
	struct device *dev;
	struct miscdevice mdev;
	struct efuse_wr_info *wr_info;
	struct mutex lock;
	void __iomem *iomem;
#ifdef CONFIG_INGENIC_EFUSE_V2_WRITABLE
	struct timer_list vddq_protect_timer;
	struct board_gpio avd_efuse_en;
#endif
};

static struct jz_efuse *efuse;
static struct seg_info info;

static uint32_t efuse_readl(uint32_t reg_off)
{
	return readl(efuse->iomem + reg_off);
}

static void efuse_writel(uint32_t val, uint32_t reg_off)
{
	writel(val, efuse->iomem + reg_off);
}

#ifdef CONFIG_INGENIC_EFUSE_V2_WRITABLE
static void efuse_vddq_set_timer(struct timer_list *t)
{
	struct jz_efuse *efuse_test = from_timer(efuse, t, vddq_protect_timer);
	gpio_direction_output(efuse->avd_efuse_en.gpio, !efuse->avd_efuse_en.level);
}

static void efuse_vddq_set(unsigned long on)
{
	if(on){
		mod_timer(&efuse->vddq_protect_timer, jiffies + msecs_to_jiffies(800));
	}
	gpio_direction_output(efuse->avd_efuse_en.gpio, on);
}
#else
static void efuse_vddq_set(unsigned long on) {}
#endif

static int efuse_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int efuse_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static void otp_r(uint32_t addr, uint32_t blen)
{
	unsigned int val;
	int n;

	efuse_writel(0, EFUSE_CTRL);

	for(n = 0; n < 8; n++)
		efuse_writel(0, EFUSE_DATA(n));

	/* set read address and data length */
	val =  addr << EFUSE_CTRL_ADDR | (blen - 1) << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	/* enable read */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_RDEN;
	efuse_writel(val, EFUSE_CTRL);

	dev_dbg(efuse->dev,"efuse ctrl regval=0x%x\n",val);
	/* wait read done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_RD_DONE));
}

static int jz_efuse_read(struct seg_info *info, uint8_t *buf)
{
	int i;
	unsigned int *data = (volatile unsigned int *)(efuse->iomem + EFUSE_DATA(0));
	unsigned byte_len = (info->bit_num >> 3);

	dev_dbg(efuse->dev,"segment name: %s\nsegment addr: 0x%02x\nbyte num: %d\nbit num: %d\n",
			info->seg_name, info->offset_address, byte_len, info->bit_num);

	otp_r(info->offset_address, byte_len);

	dev_dbg(efuse->dev, "efuse read data:\n");
	for(i = 0; i < 8; i++ )
		dev_dbg(efuse->dev,"0x%08x\n", data[i]);

	memcpy(buf, data, 32);

	/* clear read done status */
	efuse_writel(0, EFUSE_STATE);

	return 0;
}
EXPORT_SYMBOL_GPL(jz_efuse_read);

static void otp_w(uint32_t addr, uint32_t blen)
{
	unsigned int val;

	efuse_writel(0, EFUSE_CTRL);

	/* set write Programming address and data length */
	val =  addr << EFUSE_CTRL_ADDR | (blen - 1) << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	/* Programming EFUSE enable */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_PGEN;
	efuse_writel(val, EFUSE_CTRL);

	/* Connect VDDQ pin from 2.5V */
	efuse_vddq_set(1);
	mdelay(1);

	/* enable write */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_WREN;
	efuse_writel(val, EFUSE_CTRL);

	/* wait write done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_WR_DONE));

	/* Disconnect VDDQ pin from 2.5V. */
	efuse_vddq_set(0);
	mdelay(1);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~(EFUSE_CTRL_PGEN);
	efuse_writel(val, EFUSE_CTRL);
}

static int jz_efuse_write(struct seg_info *info, uint8_t *buf)
{
	int i;
	unsigned int *data = (volatile unsigned int *)(efuse->iomem + EFUSE_DATA(0));
	unsigned int byte_len = (info->bit_num >> 3);
	unsigned int regval = 0;

	dev_dbg(efuse->dev,"segment name: %s\nsegment addr: 0x%02x\nbyte num: %d\nbit num: %d\n",
			info->seg_name, info->offset_address, byte_len, info->bit_num);

	if(info->seg_id != PRT) {
		regval = efuse_readl(EFUSE_STATE);
		if(info->prt_bit & regval) {
			dev_err(efuse->dev, "segment[%s] has been protected!\n", info->seg_name);
			return -1;
		}
	}

	memset(data, 0, 4 * 8);
	memcpy(data, buf, 32);
	dev_dbg(efuse->dev, "efuse write data:\n");
	for(i = 0; i < 8; i++)
		dev_dbg(efuse->dev,"%08x\n", data[i]);

	otp_w(info->offset_address, byte_len);

	return 0;
}


static long efuse_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_efuse *efuse = container_of(dev, struct jz_efuse, mdev);
	int ret = 0;

	mutex_lock(&efuse->lock);
	efuse->wr_info = (struct efuse_wr_info *)arg;

	if(efuse->wr_info->seg_id > ARRAY_SIZE(seg_info_array)) {
		dev_err(efuse->dev, "unknow segment id!\n");
		goto exit;
	}

	info = seg_info_array[efuse->wr_info->seg_id];
	switch (cmd) {
		case CMD_READ:
			ret = jz_efuse_read(&info, efuse->wr_info->data);
			break;
		case CMD_WRITE:
			ret = jz_efuse_write(&info, efuse->wr_info->data);
			break;
		default:
			dev_err(efuse->dev, "unknow cmd!\n");
			break;
	}
exit:
	mutex_unlock(&efuse->lock);
	return ret;
}

static struct file_operations efuse_misc_fops = {
	.open		= efuse_open,
	.release	= efuse_release,
	.unlocked_ioctl	= efuse_ioctl,
};


static int set_efuse_timing(struct device *dev)
{
	struct clk *h2clk;
	struct clk *devclk;
	unsigned long rate;
	uint32_t val, ns;
	int i, rd_strobe, wr_strobe;
	uint32_t rd_adj, wr_adj;
	int flag = 0;

	h2clk = devm_clk_get(dev, "div_ahb2");
	if (IS_ERR(h2clk)) {
		dev_err(efuse->dev, "get h2clk rate fail!\n");
		return -1;
	}

	devclk = devm_clk_get(dev, "gate_efuse");
	if (IS_ERR(devclk)) {
		dev_err(efuse->dev, "get efuse clk rate fail!\n");
		return -1;
	}
	clk_prepare_enable(devclk);

	rate = clk_get_rate(h2clk);
	ns = 1000000000 / rate;
	printk("rate = %lu, ns = %d\n", rate, ns);


	for(i = 0; i < 0x4; i++)
		if((( i + 1) * ns ) > 7)
			break;
	if(i == 0x4) {
		dev_err(efuse->dev, "get efuse cfg rd_adj fail!\n");
		return -1;
	}
	rd_adj = wr_adj = i;

	for(i = 0; i < 0x8; i++)
		if(((rd_adj + i + 5) * ns ) > 35)
			break;
	if(i == 0x8) {
		dev_err(efuse->dev, "get efuse cfg rd_strobe fail!\n");
		return -1;
	}
	rd_strobe = i;

	for(i = 0; i < 0x7ff; i++) {
		val = (wr_adj + i + 1666) * ns;
		if(val > 11 * 1000) {
			val = (wr_adj - i + 1666) * ns;
			flag = 1;
		}
		if(val > 9 * 1000 && val < 11 * 1000)
			break;
	}
	if(i >= 0x7ff) {
		dev_err(efuse->dev, "get efuse cfg wd_strobe fail!\n");
		return -1;
	}

	if(flag)
		i |= 1 << 11;

	wr_strobe = i;

	dev_info(efuse->dev, "rd_adj = %d | rd_strobe = %d | "
		 "wr_adj = %d | wr_strobe = %d\n", rd_adj, rd_strobe,
		 wr_adj, wr_strobe);

	/*set configer register*/
	val = rd_adj << EFUSE_CFG_RD_ADJ | rd_strobe << EFUSE_CFG_RD_STROBE;
	val |= wr_adj << EFUSE_CFG_WR_ADJ | wr_strobe;
	efuse_writel(val, EFUSE_CFG);

	clk_put(h2clk);

	return 0;
}

static int show_segment(uint32_t seg_id,  char *buf)
{
	int ret = 0;
	int i = 0;
	unsigned char val[32] = {0};
	char *last = NULL;
	info = seg_info_array[seg_id];
	last = (char *)val + info.bit_num / 8 - 1;

	jz_efuse_read(&info, val);
	for(i = 0; i < info.bit_num / 8; i++)
		ret += snprintf(buf + (i * 2), 3, "%02x", *((uint8_t *)last - i));
	strcat(buf, "\n");

	return ret + 1;
}

static ssize_t chipid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(CHIPID, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t cutid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(CUTID, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t trim0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(TRIM0, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t trim1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(TRIM1, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t trim2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(TRIM2, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t trim3_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(TRIM3, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t socinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(SOCINFO, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t hideblk_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(HIDEBLK, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t prt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(PRT, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static void store_segment(uint32_t seg_id, const char *buf, int len)
{
	int i = 0;
	unsigned char val[32] = {0};
	char tmp[9] = {'\0'};
	char *last = (char *)buf + (len - 1);
	int bit_num = (len - 1) * 4;
	int byte_len = (len - 1) / 2;
	info = seg_info_array[seg_id];

	memset(val, 0, 32);

	if (bit_num == info.bit_num) {
		for (i = 0; i < byte_len; i++) {
			memcpy(tmp, last - ((i + 1) * 2), 2);
			sscanf(tmp, "%02hhx", &val[i]);
			printk("%02x\n", val[i]);
		}

		jz_efuse_write(&info, val);
	} else {
		printk("%s segment size is %d bits!\n", info.seg_name, info.bit_num);
	}
}

static ssize_t chipid_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(CHIPID, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t cutid_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(CUTID, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t trim0_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(TRIM0, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t trim1_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(TRIM1, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t trim2_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(TRIM2, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t trim3_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(TRIM3, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t socinfo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(SOCINFO, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t hideblk_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(HIDEBLK, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t prt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(PRT, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}


static DEVICE_ATTR(chipid,  S_IRUGO|S_IWUSR, chipid_show,  chipid_store);
static DEVICE_ATTR(cutid,   S_IRUGO|S_IWUSR, cutid_show,   cutid_store);
static DEVICE_ATTR(trim0,   S_IRUGO|S_IWUSR, trim0_show,   trim0_store);
static DEVICE_ATTR(trim1,   S_IRUGO|S_IWUSR, trim1_show,   trim1_store);
static DEVICE_ATTR(trim2,   S_IRUGO|S_IWUSR, trim2_show,   trim2_store);
static DEVICE_ATTR(trim3,   S_IRUGO|S_IWUSR, trim3_show,   trim3_store);
static DEVICE_ATTR(socinfo, S_IRUGO|S_IWUSR, socinfo_show, socinfo_store);
static DEVICE_ATTR(hideblk, S_IRUGO|S_IWUSR, hideblk_show, hideblk_store);
static DEVICE_ATTR(prt,     S_IRUGO|S_IWUSR, prt_show,     prt_store);
static DEVICE_ATTR(chipkey, S_IRUGO|S_IWUSR, NULL, NULL);
static DEVICE_ATTR(userkey, S_IRUGO|S_IWUSR, NULL, NULL);
static DEVICE_ATTR(nku,     S_IRUGO|S_IWUSR, NULL, NULL);

static struct attribute *efuse_rw_attrs[] = {
	&dev_attr_chipid.attr,
	&dev_attr_cutid.attr,
	&dev_attr_trim0.attr,
	&dev_attr_trim1.attr,
	&dev_attr_trim2.attr,
	&dev_attr_trim3.attr,
	&dev_attr_socinfo.attr,
	&dev_attr_hideblk.attr,
	&dev_attr_prt.attr,
	&dev_attr_chipkey.attr,
	&dev_attr_userkey.attr,
	&dev_attr_nku.attr,
	NULL,
};

const char efuse_group_name[] = "efuse_rw";
static struct attribute_group efuse_rw_attr_group = {
	.name   = efuse_group_name,
	.attrs  = efuse_rw_attrs,
};


#ifdef CONFIG_INGENIC_EFUSE_V2_WRITABLE
static int of_avd_efuse(struct device *dev)
{
	struct jz_efuse *efuse = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;
#if 1
	efuse->avd_efuse_en.gpio = of_get_named_gpio_flags(np, "ingenic,efuse-en-gpio", 0, &flags);
	if(!gpio_is_valid(efuse->avd_efuse_en.gpio)) {
		ret = efuse->avd_efuse_en.gpio;
		dev_err(dev, "efuse_en gpio invalid! %d\n", ret);
		return ret;
	}

	ret = devm_gpio_request(dev, efuse->avd_efuse_en.gpio, "efuse-en");
	if(ret < 0) {
		dev_err(dev, "efuse_en gpio request failed! %d\n", ret);
		return ret;
	}
	efuse->avd_efuse_en.level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
#else
	efuse->pmu_efuse_en = regulator_get(NULL, "LDO2_1V8");
	if(!efuse->pmu_efuse_en) {
		dev_err(dev, "get efuse regulator failed\n");
		return -1;
	}
#endif
	return 0;
}
#endif

static int jz_efuse_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	int ret = 0;

	efuse = devm_kzalloc(&pdev->dev, sizeof(struct jz_efuse), GFP_KERNEL);
	if (!efuse) {
		printk("efuse malloc failed!\n");
		return -ENOMEM;
	}

	mutex_init(&efuse->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res)) {
		dev_err(&pdev->dev, "get efuse resource failed!\n");
		ret = -ENXIO;
		goto err_free_efuse;
	}

	efuse->iomem = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(efuse->iomem)) {
		dev_err(&pdev->dev, "efuse ioremap failed!\n");
		ret = -EBUSY;
		goto err_free_efuse;
	}

	efuse->dev = &pdev->dev;
	efuse->wr_info = NULL;
	efuse->mdev.minor = MISC_DYNAMIC_MINOR;
	efuse->mdev.name =  DRV_NAME;
	efuse->mdev.fops = &efuse_misc_fops;

	ret = misc_register(&efuse->mdev);
	if (ret < 0) {
		dev_err(efuse->dev, "efuse misc_register failed\n");
		ret = -EINVAL;
		goto err_free_efuse_io;
	}
	platform_set_drvdata(pdev, efuse);
#ifdef CONFIG_INGENIC_EFUSE_V2_WRITABLE
	ret = of_avd_efuse(efuse->dev);
	if (ret < 0) {
		goto err_free_efuse_io;
	}

	dev_info(efuse->dev, "setup vddq_protect_timer!\n");
	timer_setup(&efuse->vddq_protect_timer, efuse_vddq_set_timer, 0);
	add_timer(&efuse->vddq_protect_timer);

#endif
	ret = set_efuse_timing(efuse->dev);
	if(ret) {
		dev_err(efuse->dev, "efuse timing setting failed!\n");
		ret = -EINVAL;
		goto err_free_efuse_io;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &efuse_rw_attr_group);
	if (ret) {
		dev_err(efuse->dev, "device create sysfs group failed\n");
		ret = -EINVAL;
		goto err_free_efuse_io;
	}


	dev_info(efuse->dev, "efuse probe success.\n");
	return 0;


err_free_efuse_io:
	iounmap(efuse->iomem);
err_free_efuse:
	kfree(efuse);

	return ret;
}


static int jz_efuse_remove(struct platform_device *dev)
{
	struct jz_efuse *efuse = platform_get_drvdata(dev);

	misc_deregister(&efuse->mdev);
#ifdef CONFIG_INGENIC_EFUSE_V2_WRITABLE
	dev_info(efuse->dev, "del vddq_protect_timer!\n");
	del_timer(&efuse->vddq_protect_timer);
#endif
	iounmap(efuse->iomem);
	kfree(efuse);

	return 0;
}

static const struct of_device_id efuse_of_match[] = {
	{ .compatible = "ingenic,x1600-efuse"},
	{},
};

static struct platform_driver jz_efuse_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(efuse_of_match),
	},
	.probe		= jz_efuse_probe,
	.remove		= jz_efuse_remove,
};

static int __init jz_efuse_init(void)
{
	return platform_driver_register(&jz_efuse_driver);
}

static void __exit jz_efuse_exit(void)
{
	platform_driver_unregister(&jz_efuse_driver);
}


module_init(jz_efuse_init);
module_exit(jz_efuse_exit);

MODULE_DESCRIPTION("efuse v2 driver");
MODULE_AUTHOR("zhxiao <zhihao.xiao@ingenic.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20210823");
