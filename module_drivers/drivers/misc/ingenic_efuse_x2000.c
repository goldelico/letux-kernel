/*
 * linux/drivers/misc/ingenic_efuse_x2000.c - Ingenic efuse driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: <chongji.wang@ingenic.com>.
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

#include "hamming.c"


#define DRV_NAME	"ingenic-efuse"

#define CMD_WRITE	 _IOR('E', 100, unsigned int)
#define CMD_READ	 _IOR('E', 101, unsigned int)

struct efuse_wr_info {
	uint32_t seg_id;
	uint32_t *data;
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
#ifdef CONFIG_INGENIC_EFUSE_WRITABLE
	struct timer_list vddq_protect_timer;
//	struct regulator *pmu_efuse_en;
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

#ifdef CONFIG_INGENIC_EFUSE_WRITABLE
static void efuse_vddq_set(unsigned long on)
{
	if(on){
		mod_timer(&efuse->vddq_protect_timer, jiffies + HZ);
	}
	gpio_direction_output(efuse->avd_efuse_en.gpio, !on);
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

static void otp_r(uint32_t addr, uint32_t wlen)
{
	unsigned int val;
	int n;

	efuse_writel(0, EFUSE_CTRL);

	for(n = 0; n < 8; n++)
		efuse_writel(0, EFUSE_DATA(n));

	/* set read address and data length */
	val =  addr << EFUSE_CTRL_ADDR | (wlen - 1) << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~EFUSE_CTRL_PD;
	efuse_writel(val, EFUSE_CTRL);

	/* enable read */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_RDEN;
	efuse_writel(val, EFUSE_CTRL);

	dev_dbg(efuse->dev,"efuse ctrl regval=0x%x\n",val);
	/* wait read done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_RD_DONE));

	efuse_writel(0, EFUSE_CTRL);
	efuse_writel(EFUSE_CTRL_PD, EFUSE_CTRL);
}

void rir_w(uint32_t addr, uint32_t value)
{
	unsigned int val;

	efuse_writel(value, EFUSE_DATA(0));
	efuse_writel(0, EFUSE_CTRL);

	val =  addr << EFUSE_CTRL_ADDR | 0 << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~EFUSE_CTRL_PD;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_PS | EFUSE_CTRL_RWL;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_PGEN;
	efuse_writel(val, EFUSE_CTRL);

	efuse_vddq_set(1);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_WREN;
	efuse_writel(val, EFUSE_CTRL);

	/* wait write done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_WR_DONE));

	efuse_vddq_set(0);

	efuse_writel(0, EFUSE_CTRL);
	efuse_writel(EFUSE_CTRL_PD, EFUSE_CTRL);
}

static void rir_r(void)
{
	unsigned int val;

	efuse_writel(0, EFUSE_CTRL);
	efuse_writel(0, EFUSE_DATA(0));
	efuse_writel(0, EFUSE_DATA(1));

	/* set rir read address and data length */
	val =  0x1f << EFUSE_CTRL_ADDR | 0x1 << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~EFUSE_CTRL_PD;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~EFUSE_CTRL_PS;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_RWL;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_RDEN;
	efuse_writel(val, EFUSE_CTRL);

	/* wait read done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_RD_DONE));

	efuse_writel(0, EFUSE_CTRL);
	efuse_writel(EFUSE_CTRL_PD, EFUSE_CTRL);

	dev_dbg(efuse->dev, "RIR0=0x%08x\n", efuse_readl(EFUSE_DATA(0)));
	dev_dbg(efuse->dev, "RIR1=0x%08x\n", efuse_readl(EFUSE_DATA(1)));
}

static int rir_op(uint32_t value, uint32_t flag)
{
	unsigned int addr = 0, rf_addr = 0;
	unsigned int fb_disable = 0;
	unsigned int ret1, ret2;
	int rir_num = 0;

	if(value == 0)
		return -1;

	rir_r();
	ret1 = efuse_readl(EFUSE_DATA(0));
	ret2 = efuse_readl(EFUSE_DATA(1));

	if((ret1 & 0xFFFF) && (ret1 & (0xFFFF << 16)) &&
			(ret2 & 0xFFFF) && (ret2 & (0xFFFF << 16))) {
		if(flag == 1) {
			fb_disable = 0x1 << 31;
			rf_addr = 0x20;
			rir_w(rf_addr, fb_disable);
		}
		dev_err(efuse->dev, "not redundancy bits!\n");
		return -1;
	}

	if(((ret1 & (0xFFFF)) && (ret1 & (0xFFFF << 16)))) {
		addr = 0x20;
		if(ret2 & 0xFFFF) {
			value = value << 16;
			rir_num = 4;
		} else {
			rir_num = 3;
		}
	} else {
		addr = 0;
		if(ret1 & 0xFFFF) {
			value = value << 16;
			rir_num = 2;
		} else {
			rir_num = 1;
		}
	}

	if(flag == 1) {
		switch(rir_num) {
			case 2:
				fb_disable = 0x1 << 15;
				rf_addr = 0x0;
				break;
			case 3:
				fb_disable = 0x1 << 31;
				rf_addr = 0x0;
				break;
			case 4:
				fb_disable = 0x1 << 15;
				rf_addr = 0x20;
				break;
			default:
				dev_err(efuse->dev, "not rir %d!\n", rir_num);
				return -1;
		}

		rir_w(rf_addr, fb_disable);
	}

	rir_w(addr, value);

	return 0;
}

static int rir_check(struct seg_info *info, uint32_t woffs, uint32_t val)
{
	unsigned int rval, errbits;

	rir_r();
	otp_r(info->word_address + woffs, 1);

	rval = efuse_readl(EFUSE_DATA(0));
	if(woffs == 0)
		rval &= 0xffffffff << info->begin_align * 8;
	else if(woffs == info->word_num - 1)
		rval &= 0xffffffff >> info->end_align* 8;

	dev_info(efuse->dev, "%08x ^ %08x\n", rval, val);
	errbits = rval ^ val;

	return errbits;
}

static int rir_repair(struct seg_info *info, uint32_t *buf)
{
	unsigned int errbits, rir_data, repair_result, repair_fail;
	int ret, n, ebit;

	for(n = 0; n < info->word_num; n++) {
		errbits = rir_check(info, n, buf[n]);
		dev_dbg(efuse->dev, "addr=%x, errbits=0x%08x\n", info->word_address + n, errbits);

		while((ebit = ffs(errbits)) > 0) {
			rir_data = 0x1 << EFUSE_RIR_RF;
			rir_data |= (buf[n] & ebit) << EFUSE_RIR_DATA;
			rir_data |= (info->word_address + n + ((ebit + info->begin_align * 8) << 6)) << EFUSE_RIR_ADDR;
//			rir_data &= 0 << EFUSE_RIR_DISABLE;

			ret = rir_op(rir_data, 0);
			if(ret) {
				dev_err(efuse->dev, "rir repair failed!\n");
				return -1;
			}

			do {
				repair_result = rir_check(info, n, buf[n]);
				repair_fail = repair_result & (0x1 << ebit);
				if(repair_fail) {
					ret = rir_op(rir_data, 1);
					if(ret) {
						dev_err(efuse->dev, "rir repair failed!\n");
						return -1;
					}
				}
			} while(repair_fail);
			errbits &= 0 << ebit;
		}
	}

	return 0;
}

static void rir_disable_all(void)
{
	rir_r();
	rir_w(0x0, (1 << 15));
	rir_w(0x0, (1 << 31));
	rir_w(0x20, (1 << 15));
	rir_w(0x20, (1 << 31));
}

static int jz_efuse_read(struct seg_info *info, uint32_t *buf)
{
	uint32_t val;
	uint32_t rbuf[8] = {0};
	uint32_t hamming_buf[8] = {0};
	uint32_t byte_num = 0;
	uint32_t half_bit_num = 0;
	uint32_t half_byte_num = 0;
	uint32_t half_bit_align = 0;
	uint32_t hamming_bit_num = 0;
	int n, ret;

	dev_dbg(efuse->dev,"segment name: %s\nsegment addr: 0x%02x\nbegin align: %d\nend align: %d\n"
			"word num: %d\nbit num: %d\nverify mode: %d\n",
			info->seg_name, info->word_address, info->begin_align, info->end_align,
			info->word_num, info->bit_num, info->verify_mode);

	rir_r();
	otp_r(info->word_address, info->word_num);

	dev_dbg(efuse->dev, "efuse read data:\n");
	for(n = 0; n < info->word_num; n++) {
		val = efuse_readl(EFUSE_DATA(n));
		dev_dbg(efuse->dev, "%08x\n", val);
		if(n == 0)
			rbuf[n] = val & (0xffffffff << info->begin_align * 8);
		if(n == info->word_num - 1)
			rbuf[n] = val & (0xffffffff >> info->end_align * 8);
		else
			rbuf[n] = val;
	}

	byte_num = info->bit_num / 8;
	byte_num += info->bit_num % 8 ? 1 : 0;

	switch(info->verify_mode) {
		case HAMMING:
			hamming_bit_num = info->bit_num + cal_k(info->bit_num);
//			dump(rbuf, 0, hamming_bit_num);
			decode((unsigned int *)((char *)rbuf + info->begin_align),
					hamming_bit_num, hamming_buf);
			memcpy((char *)buf, (char *)hamming_buf, byte_num);
			break;
		case DOUBLE:
			half_bit_num = info->bit_num / 2;
			half_byte_num = half_bit_num / 8;
			half_bit_align = half_bit_num % 8;
//			dump((unsigned int *)((char *)rbuf + info->begin_align), 0, half_bit_num);
//			dump((unsigned int *)((char *)rbuf + info->begin_align + half_byte_num), half_bit_align, half_bit_num + half_bit_align);
			ret = checkbit((unsigned int *)((char *)rbuf + info->begin_align),
					(unsigned int *)((char *)rbuf + info->begin_align + half_byte_num),
					0, half_bit_align, half_bit_num);
			if(ret){
				dev_err(efuse->dev, "double verify failed!\n");
				return -1;
			}
			memcpy((char *)buf, ((char *)rbuf + info->begin_align), byte_num);
			break;
		case NONE:
		default:
			memcpy((char *)buf, ((char *)rbuf + info->begin_align), byte_num);
			break;
	}

	/* clear read done status */
	efuse_writel(0, EFUSE_STATE);

	return 0;
}
EXPORT_SYMBOL_GPL(jz_efuse_read);


static void otp_w(uint32_t addr, uint32_t wlen)
{
	unsigned int val;

	efuse_writel(0, EFUSE_CTRL);

	/* set write Programming address and data length */
	val =  addr << EFUSE_CTRL_ADDR | (wlen - 1) << EFUSE_CTRL_LEN;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val &= ~EFUSE_CTRL_PD;
	efuse_writel(val, EFUSE_CTRL);

	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_PS;
	efuse_writel(val, EFUSE_CTRL);

	/* Programming EFUSE enable */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_PGEN;
	efuse_writel(val, EFUSE_CTRL);

	/* Connect VDDQ pin from 1.8V */
	efuse_vddq_set(1);

	/* enable write */
	val = efuse_readl(EFUSE_CTRL);
	val |= EFUSE_CTRL_WREN;
	efuse_writel(val, EFUSE_CTRL);

	/* wait write done status */
	while(!(efuse_readl(EFUSE_STATE) & EFUSE_STA_WR_DONE));

	/* Disconnect VDDQ pin from 1.8V. */
	efuse_vddq_set(0);

	efuse_writel(0, EFUSE_CTRL);
	efuse_writel(EFUSE_CTRL_PD, EFUSE_CTRL);
}

static int jz_efuse_write(struct seg_info *info, uint32_t *buf)
{
	unsigned int val[8] = {0};
	unsigned char *pbuf = (unsigned char *)val;
	unsigned char *sbuf = (unsigned char *)buf;
	uint32_t regval = 0;
	uint32_t byte_num = 0;
	uint32_t half_bit_num = 0;
	uint32_t half_byte_num = 0;
	uint32_t half_bit_align = 0;
	uint32_t hamming_bit_num = 0;
	int ret = 0;
	int n = 0;


	dev_dbg(efuse->dev,"segment name: %s\nsegment addr: 0x%02x\nbegin align: %d\nend align: %d\n"
			"word num: %d\nbit num: %d\nverify mode: %d\n",
			info->seg_name, info->word_address, info->begin_align, info->end_align,
			info->word_num, info->bit_num, info->verify_mode);

	if(info->seg_id != PRT) {
		regval = efuse_readl(EFUSE_STATE);
		if(info->prt_bit & regval) {
			dev_err(efuse->dev, "segment[%s] has been protected!\n", info->seg_name);
			return -1;
		}
	}

	byte_num = info->bit_num / 8;
	byte_num += info->bit_num % 8 ? 1 : 0;

	switch(info->verify_mode) {
		case HAMMING:
			hamming_bit_num = info->bit_num + cal_k(info->bit_num);
//			dumphex(buf, info->word_num);
			encode(buf, info->bit_num, (unsigned int *)(pbuf + info->begin_align));
//			dump((unsigned int *)pbuf, 0, hamming_bit_num + info->begin_align * 8);
//			dumphex((unsigned int *)(pbuf + info->begin_align), info->word_num);
			break;
		case DOUBLE:
			half_bit_num = info->bit_num / 2;
			half_byte_num = half_bit_num / 8;
			half_bit_align = half_bit_num % 8;
			dump(buf, 0, half_bit_num);
			dump((unsigned int *)(sbuf + half_byte_num), half_bit_align, half_bit_num + half_bit_align);
			ret = checkbit(buf,(unsigned int *)(sbuf + half_byte_num), 0, half_bit_align, half_bit_num);
			if(ret){
				dev_err(efuse->dev, "double verify failed!\n");
				return -1;
			}
			memcpy(pbuf + info->begin_align, (char *)buf, byte_num);
			break;
		case NONE:
		default:
			memcpy(pbuf + info->begin_align, (char *)buf, byte_num);
			break;
	}

	dev_dbg(efuse->dev, "efuse write data:\n");
	for(n = 0; n < info->word_num; n++) {
		dev_dbg(efuse->dev,"%08x\n", val[n]);
		efuse_writel(val[n], EFUSE_DATA(n));
	}

	otp_w(info->word_address, info->word_num);

	if(info->verify_mode == HAMMING) {
		ret = rir_repair(info, val);
		if(ret < 0){
			dev_err(efuse->dev, "hamming verify failed!\n");
			return -1;
		}
	}

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
		if((( i + 1) * ns ) > 2)
			break;
	if(i == 0x4) {
		dev_err(efuse->dev, "get efuse cfg rd_adj fail!\n");
		return -1;
	}
	rd_adj = wr_adj = i;

	for(i = 0; i < 0x8; i++)
		if(((rd_adj + i + 30) * ns ) > 100)
			break;
	if(i == 0x8) {
		dev_err(efuse->dev, "get efuse cfg rd_strobe fail!\n");
		return -1;
	}
	rd_strobe = i;

	for(i = 0; i < 0x3ff; i++) {
		val = (wr_adj + i + 3000) * ns;
		if(val > 13 * 1000) {
			val = (wr_adj - i + 3000) * ns;
			flag = 1;
		}
		if(val > 11 * 1000 && val < 13 * 1000)
			break;
	}
	if(i >= 0x3ff) {
		dev_err(efuse->dev, "get efuse cfg wd_strobe fail!\n");
		return -1;
	}

	if(flag)
		i |= 1 << 10;

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

int jz_efuse_read_chipd(uint32_t *chipid){
	int ret = 0,i;
	uint32_t val[8] = {0};

	mutex_init(&efuse->lock);
	info = seg_info_array[CHIPID];
    for(i = (info.bit_num / 8/4 -1); i >=0 ; i--){
        chipid[(info.bit_num / 8/4 -1) - i] = val[i];
    }
    printk("chipid :%08X %08X %08X %08X\n",chipid[3],chipid[2],chipid[1],chipid[0]);
    ret = info.bit_num / 8/4;
	mutex_unlock(&efuse->lock);
	return ret;
}
EXPORT_SYMBOL(jz_efuse_read_chipd);

static int show_segment(uint32_t seg_id,  char *buf)
{
	int ret = 0;
	int i = 0;
	uint32_t val[8] = {0};
    char *ptr;
	char *last = NULL;
	info = seg_info_array[seg_id];
	last = (char *)val + info.bit_num / 8 - 1;

	jz_efuse_read(&info, val);
    if(seg_id == CHIPID){
        ptr = buf;
        *ptr=0;
        for(i = (info.bit_num / 8/4 -1); i >=0 ; i--) {
            sprintf(ptr,"%08x", val[i]);
            if(i>0){
                strcat(buf, " ");
                ptr = buf + strlen(buf);
            }
        }
        strcat(buf, "\n");
        ret = strlen(buf);
        printk("chipid :%s\n",buf);
    }else {
        for(i = 0; i < info.bit_num / 8; i++)
            ret += snprintf(buf + (i * 2), 3, "%02x", *((uint8_t *)last - i));
        strcat(buf, "\n");
    }

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

static ssize_t custid0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(CUSTID0, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t custid1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(CUSTID1, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static ssize_t custid2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(CUSTID2, buf);
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

static ssize_t socinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(SOCINFO, buf);
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

static ssize_t hideblk_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	mutex_init(&efuse->lock);
	ret = show_segment(HIDEBLK, buf);
	mutex_unlock(&efuse->lock);
	return ret;
}

static void store_segment(uint32_t seg_id, const char *buf, int len)
{
	int i = 0;
	uint32_t val[8] = {0};
	uint32_t tmpval = 0;
	char tmp[9] = {'\0'};
	char *last = (char *)buf + (len - 1);
	int bit_num = (len - 1) * 4;
	int word_num =(len - 1) / 8;
	int remain_num = (len - 1) % 8;
	info = seg_info_array[seg_id];

	if (bit_num == info.bit_num) {
		for (i = 0; i < word_num; i++) {
			memcpy(tmp, last - ((i + 1) * 8), 8);
			sscanf(tmp, "%08x", &val[i]);
			printk("%08x\n", val[i]);

		}

		if (remain_num > 0)  {
			memcpy(tmp, buf, remain_num);
			sscanf(tmp, "%08x", &tmpval);
			tmpval &= (0xffffffff << (8 - remain_num) * 4);
			tmpval >>= ((8 - remain_num) * 4);
			val[word_num] = tmpval;
			printk("%08x\n", val[word_num]);

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

static ssize_t custid0_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(CUSTID0, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t custid1_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(CUSTID1, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}

static ssize_t custid2_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(CUSTID2, buf, n);
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

static ssize_t socinfo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(SOCINFO, buf, n);
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

static ssize_t hideblk_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	mutex_init(&efuse->lock);
	store_segment(HIDEBLK, buf, n);
	mutex_unlock(&efuse->lock);
	return n;
}


static DEVICE_ATTR(chipid,  S_IRUGO|S_IWUSR, chipid_show,  chipid_store);
static DEVICE_ATTR(custid0, S_IRUGO|S_IWUSR, custid0_show, custid0_store);
static DEVICE_ATTR(custid1, S_IRUGO|S_IWUSR, custid1_show, custid1_store);
static DEVICE_ATTR(custid2, S_IRUGO|S_IWUSR, custid2_show, custid2_store);
static DEVICE_ATTR(trim0,   S_IRUGO|S_IWUSR, trim0_show,   trim0_store);
static DEVICE_ATTR(trim1,   S_IRUGO|S_IWUSR, trim1_show,   trim1_store);
static DEVICE_ATTR(trim2,   S_IRUGO|S_IWUSR, trim2_show,   trim2_store);
static DEVICE_ATTR(socinfo, S_IRUGO|S_IWUSR, socinfo_show, socinfo_store);
static DEVICE_ATTR(prt,     S_IRUGO|S_IWUSR, prt_show,     prt_store);
static DEVICE_ATTR(hideblk, S_IRUGO|S_IWUSR, hideblk_show, hideblk_store);
static DEVICE_ATTR(chipkey, S_IRUGO|S_IWUSR, NULL, NULL);
static DEVICE_ATTR(userkey0,S_IRUGO|S_IWUSR, NULL, NULL);
static DEVICE_ATTR(userkey1,S_IRUGO|S_IWUSR, NULL, NULL);
static DEVICE_ATTR(nku,     S_IRUGO|S_IWUSR, NULL, NULL);

static struct attribute *efuse_rw_attrs[] = {
	&dev_attr_chipid.attr,
	&dev_attr_custid0.attr,
	&dev_attr_custid1.attr,
	&dev_attr_custid2.attr,
	&dev_attr_trim0.attr,
	&dev_attr_trim1.attr,
	&dev_attr_trim2.attr,
	&dev_attr_socinfo.attr,
	&dev_attr_prt.attr,
	&dev_attr_hideblk.attr,
	&dev_attr_chipkey.attr,
	&dev_attr_userkey0.attr,
	&dev_attr_userkey1.attr,
	&dev_attr_nku.attr,
	NULL,
};

const char efuse_group_name[] = "efuse_rw";
static struct attribute_group efuse_rw_attr_group = {
	.name   = efuse_group_name,
	.attrs  = efuse_rw_attrs,
};


#ifdef CONFIG_INGENIC_EFUSE_WRITABLE
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

#ifdef CONFIG_INGENIC_EFUSE_WRITABLE
	dev_info(efuse->dev, "setup vddq_protect_timer!\n");
	setup_timer(&efuse->vddq_protect_timer, efuse_vddq_set, 0);
	add_timer(&efuse->vddq_protect_timer);

	ret = of_avd_efuse(efuse->dev);
	if (ret < 0) {
		goto err_free_efuse_io;
	}
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
#ifdef CONFIG_INGENIC_EFUSE_WRITABLE
	dev_info(efuse->dev, "del vddq_protect_timer!\n");
	del_timer(&efuse->vddq_protect_timer);
#endif
	iounmap(efuse->iomem);
	kfree(efuse);

	return 0;
}

static const struct of_device_id efuse_of_match[] = {
	{ .compatible = "ingenic,x2000-efuse"},
	{ .compatible = "ingenic,m300-efuse"},
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

MODULE_DESCRIPTION("X2000_v12 efuse driver");
MODULE_AUTHOR("cjwang <chongji.wang@ingenic.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20200312");
