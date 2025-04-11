/* drivers/pwm/pwm-ingenic-x1600.c
 * PWM driver of Ingenic's SoC X1600
 *
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author:	sihui.liu <sihui.liu@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/stat.h>
#include <linux/pwm.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm-generic/div64.h>
#include <linux/sysfs.h>
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <dt-bindings/dma/ingenic-pdma.h>

#define PWM_DBUG
#ifndef PWM_DBUG
#define print_dbg(fmt, argv...)
#else
#define print_dbg(fmt, argv...) printk(fmt, ##argv)
#endif

#define INGENIC_PWM_NUM		(8)
#define BUFFER_SIZE		PAGE_SIZE
#define NS_IN_HZ		(1000000000UL)
#define DEFAULT_PWM_CLK_RATE	(40000000)
#define SG_PWM_NUM	(100)
unsigned int add_off = 0;

struct ingenic_pwm_channel {
	u32 period_ns;
	u32 duty_ns;
};

struct ingenic_pwm_chan {
	int id;
	struct ingenic_pwm_chip *chip;
	struct dma_chan 	*dma_chan;
	struct scatterlist  	*sg;	/* I/O scatter list */
	void			*buffer;
	dma_addr_t		buffer_dma;
	int mode;
	unsigned int duty_ns;
	unsigned int period_ns;
	bool init_level;
	bool finish_level;
	bool trigger_en;
	bool trigger_posedge;
	bool trigger_negedge;
	int trigger_filter;
	unsigned int store_irq_num;
	unsigned int sg_pwm_num;
	unsigned int full_duty_status; /*Record last output level*/
};

enum pwm_mode_sel{
	COMMON_MODE,
	DMA_MODE_SG,
	DMA_MODE_CYCLIC,
};

struct ingenic_pwm_chip {
	struct pwm_chip *chip;
	struct clk *clk_pwm;
	struct clk *clk_gate;
	void __iomem    *iomem;
	int irq;
	struct mutex mutex;
	unsigned int output_mask;

	struct pwm_device *debug_pwm[INGENIC_PWM_NUM];
	int debug_current_id;

	unsigned long		phys;
	struct ingenic_pwm_chan chan[INGENIC_PWM_NUM];
};

// #define GENMASK(hi, lo)   (((1ULL << ((hi) - (lo) + 1)) - 1) << (lo))
#define	PWM_ENS		(0x00)
#define	PWM_ENC		(0x04)
#define	PWM_EN		(0x08)
#define	PWM_UPT		(0x10)
#define	PWM_BUSY	(0x14)
#define	PWM_MS		(0x20)
#define PWM_INL		(0x24)
#define PWM_IDL		(0x28)
#define PWM_CCFG0	(0x40)
#define PWM_WCFG0	(0x80)
#define PWM_DR0		(0xc0)
#define PWM_DFN0	(0x100)
#define PWM_DRTN0	(0x140)
#define PWM_DRE		(0x180)
#define PWM_DRS		(0x184)
#define PWM_DFIE	(0x188)
#define PWM_DFS		(0x18c)
#define PWM_DAFF	(0x190)
#define PWM_DCFF	(0x194)
#define PWM_SS		(0x200)
#define PWM_SIE		(0x204)
#define PWM_SC0		(0x210)
#define PWM_SN0		(0x250)
#define PWM_ON0		(0x290)
#define PWM_CCFG(n)	(PWM_CCFG0 + 4*n)
#define PWM_WCFG(n)	(PWM_WCFG0 + 4*n)
#define PWM_DR(n)	(PWM_DR0 + 4*n)
#define PWM_DFN(n)	(PWM_DFN0 + 4*n)
#define PWM_DRTN(n)	(PWM_DRTN0 + 4*n)
#define PWM_SC(n)	(PWM_SC0 + 4*n)
#define PWM_SN(n)	(PWM_SN0 + 4*n)
#define PWM_ON(n)	(PWM_ON0 + 4*n)

#define PWM_WCFG_HIGH		(16)
#define PWM_WCFG_LOW		(0)

#define PWM_STORE_SE	BIT(18)
#define PWM_STORE_SPE	BIT(17)
#define PWM_STORE_SNE	BIT(16)
#define PWM_STORE_SFN_LBIT	0
#define PWM_STORE_SFN_HBIT	9
#define PWM_STORE_SFN_MASK	\
	GENMASK(PWM_STORE_SFN_HBIT, PWM_STORE_SFN_LBIT)


/**
 **		pwm basic operation configuration interface
 **/
static void pwm_writel(struct ingenic_pwm_chip *ingenic_pwm, unsigned int value, unsigned int offset)
{
	writel(value, ingenic_pwm->iomem + offset);
}

static unsigned int pwm_readl(struct ingenic_pwm_chip *ingenic_pwm, unsigned int offset)
{
	return readl(ingenic_pwm->iomem + offset);
}

static unsigned int pwm_store_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_SS);
}

static void pwm_store_clear_status(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_SS);
}

static void pwm_trigger_enable_interrupt(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SIE);
	pwm_writel(ingenic_pwm, val | (1 << channel), PWM_SIE);
}

static void pwm_store_disable_interrupt(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SIE);
	pwm_writel(ingenic_pwm, val & ~(1 << channel), PWM_SIE);
}

static void pwm_trigger_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SE, PWM_SC(channel));
}

static void pwm_store_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SE, PWM_SC(channel));
}

static void pwm_trigger_posedge_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SPE, PWM_SC(channel));
}

static void pwm_trigger_posedge_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SPE, PWM_SC(channel));
}
static void pwm_trigger_negedge_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SNE, PWM_SC(channel));
}

static void pwm_trigger_negedge_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SNE, PWM_SC(channel));
}

static void pwm_trigger_filter_set(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel,
		unsigned int number)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, (val & ~PWM_STORE_SFN_MASK) | number, PWM_SC(channel));
}

static unsigned int pwm_get_store_num(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_SN(channel));
}

static unsigned int pwm_get_output_num(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_ON(channel));
}

static void pwm_clk_config(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, unsigned int prescale)
{
	pwm_writel(ingenic_pwm, prescale, PWM_CCFG(channel));
}

static int pwm_get_prescale(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_CCFG(channel));
}

static void pwm_enable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_ENS);
}

static void pwm_disable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_ENC);
}

static int pwm_enable_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_EN);
}
static void pwm_update(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_UPT);
}

static int pwm_busy(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_BUSY);
	return tmp & (1 << channel);
}

static void pwm_set_init_level(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_INL);

	if(level)
		tmp |= 1 << channel;
	else
		tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_INL);
}

static void pwm_set_finish_level(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_IDL);
	if(level)
		tmp |= 1 << channel;
	else
		tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_IDL);
}

static void pwm_waveform_high(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, unsigned int high_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG(channel));
	tmp &= ~(0xffff << PWM_WCFG_HIGH);
	tmp |= high_num << PWM_WCFG_HIGH;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG(channel));
}

static void pwm_waveform_low(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, unsigned int low_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG(channel));
	tmp &= ~(0xffff);
	tmp |= low_num << PWM_WCFG_LOW;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG(channel));
}

/*pwm DMA mode control register operation*/
static void pwm_dma_enable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_DRE);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DRE);
}

static void pwm_dma_disable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_DRE);
	tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DRE);
}

static void pwm_set_update_mode(struct ingenic_pwm_chip *ingenic_pwm,
		unsigned int channel, int mode_sel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_MS);
	if(mode_sel == DMA_MODE_SG || mode_sel == DMA_MODE_CYCLIC)
		tmp |= 1 << channel;
	if(mode_sel == COMMON_MODE)
		tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_MS);
}
static void pwm_dma_under_irq_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_DFIE);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_DFIE);
}

static void pwm_dma_under_irq_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_DFIE);
	tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_DFIE);
}

static unsigned int pwm_dma_under_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm,PWM_DFS);
}

static void pwm_dma_clear_under(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm,PWM_DFS);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm,tmp,PWM_DFS);
}

static void pwm_set_fifo_throshold(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 32, PWM_DRTN(channel));
}

static void pwm_dma_fifo_flush(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_DCFF);
}

static void dump_pwm_reg(struct ingenic_pwm_chip *ingenic_pwm)
{
	int i;
	printk("PWM_EN(%08x)	= %08x\n", PWM_EN, pwm_readl(ingenic_pwm, PWM_EN));
	printk("PWM_UPT(%08x)	= %08x\n", PWM_UPT, pwm_readl(ingenic_pwm, PWM_UPT));
	printk("PWM_BUSY(%08x)  = %08x\n", PWM_BUSY, pwm_readl(ingenic_pwm, PWM_BUSY));
	printk("PWM_MS(%08x)	= %08x\n", PWM_MS, pwm_readl(ingenic_pwm, PWM_MS));
	printk("PWM_INL(%08x)	= %08x\n", PWM_INL, pwm_readl(ingenic_pwm, PWM_INL));
	printk("PWM_IDL(%08x)  = %08x\n", PWM_IDL, pwm_readl(ingenic_pwm, PWM_IDL));
	printk("PWM_DRS(%08x)  = %08x\n", PWM_DRS, pwm_readl(ingenic_pwm, PWM_DRS));
	printk("PWM_DFIE(%08x)  = %08x\n", PWM_DFIE, pwm_readl(ingenic_pwm, PWM_DFIE));
	printk("PWM_DFS(%08x)  = %08x\n", PWM_DFS, pwm_readl(ingenic_pwm, PWM_DFS));
	printk("PWM_DRE(%08x)  = %08x\n", PWM_DRE,pwm_readl(ingenic_pwm, PWM_DRE));
	printk("PWM_SS(%08x)  = %08x\n", PWM_SS,pwm_readl(ingenic_pwm, PWM_SS));
	printk("PWM_SIE(%08x)  = %08x\n", PWM_SIE,pwm_readl(ingenic_pwm, PWM_SIE));
	for(i = 0; i < INGENIC_PWM_NUM; i++) {
		if(!(pwm_readl(ingenic_pwm, PWM_EN) & (1 << i)))
			continue;
		printk("\n>>>Channel-%d store_num = (0x%x)%d:\n",i, ingenic_pwm->chan[i].store_irq_num, ingenic_pwm->chan[i].store_irq_num);
		printk("PWM_CCFG(%08x)  = %08x\n", PWM_CCFG(i),pwm_readl(ingenic_pwm, PWM_CCFG(i)));
		printk("PWM_WCFG(%08x)  = %08x\n", PWM_WCFG(i),pwm_readl(ingenic_pwm, PWM_WCFG(i)));
		printk("PWM_DR(%08x)  = %08x\n", PWM_DR(i),pwm_readl(ingenic_pwm, PWM_DR(i)));
		printk("PWM_DFN(%08x)  = %08x\n", PWM_DFN(i),pwm_readl(ingenic_pwm, PWM_DFN(i)));
		printk("PWM_DRTN(%08x)  = %08x\n\n", PWM_DRTN(i),pwm_readl(ingenic_pwm, PWM_DRTN(i)));
		printk("PWM_SC(%08x)  = %08x\n\n", PWM_SC(i),pwm_readl(ingenic_pwm, PWM_SC(i)));
		printk("PWM_SN(%08x)  = %08x\n\n", PWM_SN(i),pwm_readl(ingenic_pwm, PWM_SN(i)));
		printk("PWM_ON(%08x)  = %08x\n\n", PWM_ON(i),pwm_readl(ingenic_pwm, PWM_ON(i)));
	}
}

static inline struct ingenic_pwm_chip *to_ingenic_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static void dma_tx_callback(void *data)
{
	/* print_dbg(">>%s %d\n",__func__,__LINE__); */
}

static int ingenic_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns, bool enabled)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_pwm(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];
	unsigned int period = 0;
	unsigned int duty = 0;
	unsigned int clk_in = 0;
	unsigned int prescale = 0;
	unsigned long long tmp;
	int mode = 0;
	int stable_output_flag = 0;

	mutex_lock(&ingenic_pwm->mutex);
	if (duty_ns < 0 || duty_ns > period_ns) {
		dev_err(&ingenic_pwm->chip->dev, "%s: duty_ns(%d) < 0 or duty_ns > period_ns(%d)\n", __func__, duty_ns, period_ns);
		mutex_unlock(&ingenic_pwm->mutex);
		return -EINVAL;
	}

	if(duty_ns < 0 || duty_ns > period_ns){
		mutex_unlock(&ingenic_pwm->mutex);
		return -ERANGE;
	}

	/*select mode */
	mode = ingenic_chan->mode;
	ingenic_chan->duty_ns = duty_ns;
	ingenic_chan->period_ns = period_ns;

	/*set prescale*/
	clk_in = clk_get_rate(ingenic_pwm->clk_pwm);

	/* period = period_ns / (1000000000 / clk_in) */
	tmp = (unsigned long long)clk_in * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;
	while (period > 0xffff && prescale < 8) {
		period >>= 1;
		++prescale;
	}
	if (prescale == 8){
		mutex_unlock(&ingenic_pwm->mutex);
		return -EINVAL;
	}

	/* duty = period / (period_ns / duty_ns) */
	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = tmp;

	print_dbg("period_ns=%d duty_ns = %d high_ns = %d clk_in = %u\n",
			period_ns, duty_ns,
			period_ns-duty_ns,
			clk_in);
	print_dbg("period = %d LOW=%d HIGH=%d init_level=%d finsh_level = %d div = %d\n",
			period, duty, period-duty,
			ingenic_chan->init_level,
			ingenic_chan->finish_level,
			prescale);
	print_dbg("trigger_en = %d negedge = %d posedge  =%d filter = %d\n",
			ingenic_chan->trigger_en,
			ingenic_chan->trigger_posedge,
			ingenic_chan->trigger_negedge,
			ingenic_chan->trigger_filter);
	if(duty_ns == period_ns){
		ingenic_chan->finish_level = !ingenic_chan->init_level;
		stable_output_flag = 1;
	}
	else if(duty_ns == 0){
		ingenic_chan->finish_level = ingenic_chan->init_level;
		stable_output_flag = 1;
	}

	/*set init level*/
	pwm_clk_config(ingenic_pwm, channel, prescale);
	pwm_set_init_level(ingenic_pwm,channel,ingenic_chan->init_level);
	pwm_set_finish_level(ingenic_pwm,channel,ingenic_chan->finish_level);
	pwm_set_update_mode(ingenic_pwm, channel, mode);
	if(ingenic_chan->trigger_en) {
		pwm_trigger_enable(ingenic_pwm, channel);
		if(ingenic_chan->trigger_negedge)
			pwm_trigger_negedge_enable(ingenic_pwm, channel);
		else
			pwm_trigger_negedge_disable(ingenic_pwm, channel);
		if(ingenic_chan->trigger_posedge)
			pwm_trigger_posedge_enable(ingenic_pwm, channel);
		else
			pwm_trigger_posedge_disable(ingenic_pwm, channel);
		pwm_trigger_enable_interrupt(ingenic_pwm, channel);
		pwm_trigger_filter_set(ingenic_pwm, channel, ingenic_chan->trigger_filter);
	}

	/*set waveform high_num and low_num*/
	if(COMMON_MODE == mode){
		pwm_waveform_high(ingenic_pwm, channel, duty);
		pwm_waveform_low(ingenic_pwm, channel, period - duty);

		if(stable_output_flag == 1){
			pwm_disable_hw(ingenic_pwm, channel);
			ingenic_chan->full_duty_status = 1;
		}
		else if(ingenic_chan->full_duty_status == 1){
			pwm_enable_hw(ingenic_pwm, channel);
			ingenic_chan->full_duty_status = 0;
		}
		else if(ingenic_pwm->chip->pwms[channel].flags) {
			pwm_update(ingenic_pwm, channel);
			while(pwm_busy(ingenic_pwm, channel));
		}
	}

	if(DMA_MODE_SG == mode || DMA_MODE_CYCLIC == mode)
	{
		struct dma_slave_config tx_config;
		struct dma_async_tx_descriptor *txdesc;
		struct dma_chan *txchan = ingenic_chan->dma_chan;
		unsigned int *wave;
		unsigned int val;
		int i;
		tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.dst_maxburst = 4;
		tx_config.src_maxburst = 4;
		tx_config.dst_addr = (dma_addr_t)(ingenic_pwm->phys+PWM_DR(channel));
// FIXME:		tx_config.slave_id = 0;
		tx_config.direction = DMA_MEM_TO_DEV;
		dmaengine_slave_config(txchan, &tx_config);

		ingenic_chan->buffer = dma_alloc_coherent(&ingenic_pwm->chip->dev, BUFFER_SIZE,
						&ingenic_chan->buffer_dma, GFP_KERNEL);
		if (!ingenic_chan->buffer) {
			dev_err(&ingenic_pwm->chip->dev, "PWM request temp dma buffer failed");
		}
		memset(ingenic_chan->buffer, 0, BUFFER_SIZE);
		wave = ingenic_chan->buffer;
		val = duty | ((period-duty) << 16);
		for(i = 0; i < BUFFER_SIZE / 4; i++) {
			wave[i] = val;
		}
		if(mode == DMA_MODE_CYCLIC) {
			txdesc = txchan->device->device_prep_dma_cyclic(txchan,
						      ingenic_chan->buffer_dma,
						      BUFFER_SIZE,
						      BUFFER_SIZE/4,
						      DMA_MEM_TO_DEV,
						      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		} else {
			sg_init_one(ingenic_chan->sg, ingenic_chan->buffer, ingenic_chan->sg_pwm_num*4);
			if (dma_map_sg(&ingenic_pwm->chip->dev,
				       ingenic_chan->sg, 1, DMA_TO_DEVICE) != 1) {
				dev_err(&ingenic_pwm->chip->dev, "dma_map_sg tx error\n");
			}

			txdesc = dmaengine_prep_slave_sg(txchan,
						      ingenic_chan->sg,
						      1,
						      DMA_MEM_TO_DEV,
						      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		}
		if(!txdesc) {
			dev_err(&ingenic_pwm->chip->dev, "PWM request dma desc failed");
		}
		txdesc->callback = dma_tx_callback;
		txdesc->callback_param = ingenic_chan;
		dmaengine_submit(txdesc);
		if(mode == DMA_MODE_CYCLIC)
			pwm_dma_under_irq_enable(ingenic_pwm, channel);
		pwm_set_fifo_throshold(ingenic_pwm, channel);
		pwm_dma_fifo_flush(ingenic_pwm, channel);

		dma_async_issue_pending(txchan);
	}
	mutex_unlock(&ingenic_pwm->mutex);
	return 0;
}

static int ingenic_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_pwm(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];

	if(ingenic_chan->full_duty_status)
		return 0;

	mutex_lock(&ingenic_pwm->mutex);
	ingenic_chan->store_irq_num = 0;

	if(DMA_MODE_SG == ingenic_chan->mode ||
			DMA_MODE_CYCLIC == ingenic_chan->mode)
		pwm_dma_enable_hw(ingenic_pwm, channel);
	pwm_enable_hw(ingenic_pwm, channel);
//	dump_pwm_reg(ingenic_pwm);
	mutex_unlock(&ingenic_pwm->mutex);

	return 0;
}

static void ingenic_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_pwm(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];

	mutex_lock(&ingenic_pwm->mutex);

	if(ingenic_chan->mode == DMA_MODE_SG || ingenic_chan->mode == DMA_MODE_CYCLIC){
		dmaengine_terminate_all(ingenic_chan->dma_chan);
		pwm_dma_disable_hw(ingenic_pwm,channel);
		dma_free_coherent(&ingenic_pwm->chip->dev, BUFFER_SIZE,
				ingenic_chan->buffer, ingenic_chan->buffer_dma);
		if(ingenic_chan->mode == DMA_MODE_SG)
			dma_unmap_sg(&ingenic_pwm->chip->dev, ingenic_chan->sg, 1, DMA_TO_DEVICE);
		pwm_dma_under_irq_disable(ingenic_pwm,channel);
	}

	if(ingenic_chan->trigger_en) {
		pwm_store_disable(ingenic_pwm, channel);
		pwm_trigger_negedge_disable(ingenic_pwm, channel);
		pwm_trigger_posedge_disable(ingenic_pwm, channel);
		pwm_store_disable_interrupt(ingenic_pwm, channel);
		pwm_trigger_filter_set(ingenic_pwm, channel, 0);
	}
	pwm_disable_hw(ingenic_pwm, channel);

	mutex_unlock(&ingenic_pwm->mutex);
}

static int ingenic_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			 const struct pwm_state *state)
{
	int err;
	bool enabled = pwm->state.enabled;
#if FIXME
	if (state->polarity != pwm->state.polarity) {
		if (enabled) {
			ingenic_pwm_disable(chip, pwm);
			enabled = false;
		}

		err = ingenic_pwm_set_polarity(chip, pwm, state->polarity);
		if (err)
			return err;
	}
#endif
	if (!state->enabled) {
		if (enabled)
			ingenic_pwm_disable(chip, pwm);

		return 0;
	}

	err = ingenic_pwm_config(pwm->chip, pwm,
			     state->duty_cycle, state->period, enabled);
	if (err)
		return err;

	if (!enabled)
		err = ingenic_pwm_enable(chip, pwm);

	return err;
}

static const struct pwm_ops ingenic_pwm_ops = {
	.apply = ingenic_pwm_apply,
};


static ssize_t pwm_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	return sprintf(buf, "0x%x", pwm_enable_status(ingenic_pwm));
}

static ssize_t pwm_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	int enable;
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count)
			return count;
	}
	enable = simple_strtoul(str, (char **)&str, 10);

	print_dbg("input enable value %d,current_id %d\n",enable,ingenic_pwm->debug_current_id);
	if (enable) {
		pwm_config(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id],
				ingenic_chan->duty_ns, ingenic_chan->period_ns);
		pwm_enable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	} else
		pwm_disable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	return count;
}


static ssize_t pwm_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);

	return sprintf(buf, "channel = %x", ingenic_pwm->debug_current_id);

}

static ssize_t pwm_channel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	const char *str = buf;
	int ret_count = 0;
	int pwm_id = 0;
	struct pwm_device *pwm = NULL;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count)
			return count;
	}
	pwm_id = simple_strtoul(str, (char **)&str, 10);
	pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(pwm)) {
		printk("unable to request pwm\n");
		return -1;
	}
	ingenic_pwm->debug_pwm[pwm_id] = pwm;
	ingenic_pwm->debug_current_id = pwm_id;
	print_dbg("channel = %d\n", ingenic_pwm->debug_current_id);
	return count;
}

static ssize_t pwm_free_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	const char *str = buf;
	int ret_count = 0;
	int pwm_id = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count)
			return count;
	}
	pwm_id = simple_strtoul(str, (char **)&str, 10);

	pwm_disable(ingenic_pwm->debug_pwm[pwm_id]);
	ingenic_pwm->debug_pwm[pwm_id] = NULL;
	return count;
}

static ssize_t pwm_show_requested_channel(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	int i = 0;
	int ret = 0;

	for (i = 0; i <INGENIC_PWM_NUM ; i++) {
		if (ingenic_pwm->debug_pwm[i] == &(ingenic_pwm->chip->pwms[i]))
			ret += sprintf(buf + ret, "ch: %02d requested\n", i);
		else
			ret += sprintf(buf + ret, "ch: %02d unrequested\n", i);
	}
	return ret;
}

static ssize_t pwm_duty_ns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->duty_ns);
}

static ssize_t pwm_duty_ns_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO duty_ns config!\n");
			return count;
		}
	}
	ingenic_chan->duty_ns = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_sg_pwm_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->sg_pwm_num);
}

static ssize_t pwm_sg_pwm_num_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO sg_pwm_num config!\n");
			return count;
		}
	}
	ingenic_chan->sg_pwm_num = simple_strtoul(str, (char **)&str, 10);

	return count;
}
static ssize_t pwm_period_ns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->period_ns);
}

static ssize_t pwm_period_ns_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO period_ns config!\n");
			return count;
		}
	}
	ingenic_chan->period_ns = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	int ret;

	ret = sprintf(buf, "0 --> CPU mode, Continuous waveform.\n"
			   "1 --> DMA mode,Specified number of waveforms.\n"
			   "2 --> DMA mode,Continuous waveform.\n");
	ret += sprintf(buf + ret, "current mode: %d\n", ingenic_chan->mode);
	return ret;
}

static ssize_t pwm_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO mode config!\n");
			return count;
		}
	}
	ingenic_chan->mode = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_init_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->init_level);
}

static ssize_t pwm_init_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO init_level config!\n");
			return count;
		}
	}
	ingenic_chan->init_level = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_finish_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->finish_level);
}

static ssize_t pwm_finish_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "NO finish_level config!\n");
			return count;
		}
	}
	ingenic_chan->finish_level = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		&ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "trigger_en:%d trigger_negedge:%d trigger_posedge:%d trigger_filter:%d\n",
			ingenic_chan->trigger_en, ingenic_chan->trigger_negedge,
			ingenic_chan->trigger_posedge, ingenic_chan->trigger_filter);
}

static ssize_t pwm_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "No trigger_en config!\n");
			ingenic_chan->trigger_en = 0;
			return count;
		}
	}
	ingenic_chan->trigger_en = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "No trigger_negedge config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_negedge = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "No trigger_posedge config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_posedge = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if(++ret_count >= count) {
			dev_err(dev, "No trigger_filter config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_filter = simple_strtoul(str, (char **)&str, 10);

	return count;
}


static struct device_attribute pwm_device_attributes[] = {
	__ATTR(request, S_IRUGO|S_IWUSR, pwm_channel_show, pwm_channel_store),
	__ATTR(free, S_IWUSR, NULL, pwm_free_store),

	__ATTR(duty_ns, S_IRUGO|S_IWUSR, pwm_duty_ns_show, pwm_duty_ns_store),
	__ATTR(period_ns, S_IRUGO|S_IWUSR, pwm_period_ns_show, pwm_period_ns_store),
	__ATTR(sg_pwm_num, S_IRUGO|S_IWUSR, pwm_sg_pwm_num_show, pwm_sg_pwm_num_store),
	__ATTR(mode, S_IRUGO|S_IWUSR, pwm_mode_show, pwm_mode_store),
	__ATTR(init_level, S_IRUGO|S_IWUSR, pwm_init_level_show, pwm_init_level_store),
	__ATTR(finish_level, S_IRUGO|S_IWUSR, pwm_finish_level_show, pwm_finish_level_store),
	__ATTR(trigger, S_IRUGO|S_IWUSR, pwm_trigger_show, pwm_trigger_store),
	__ATTR(enable, S_IRUGO|S_IWUSR, pwm_enable_show, pwm_enable_store),

	__ATTR(channels, S_IRUGO, pwm_show_requested_channel, NULL),
};

static bool pwm_dma_chan_filter(struct dma_chan *chan, void *param)
{
	struct ingenic_pwm_chan *pwm_chan = param;

	return (INGENIC_DMA_REQ_PWM0_TX + pwm_chan->id) == (int)chan->private;
}

static irqreturn_t ingenic_pwm_interrupt(int irq, void *dev_id)
{
	unsigned int under_st, store_st;
	struct ingenic_pwm_chip *ingenic_pwm = (struct ingenic_pwm_chip *)(dev_id);
	int n;

	under_st = pwm_dma_under_status(ingenic_pwm) & pwm_readl(ingenic_pwm,PWM_DFIE);

	store_st = pwm_store_status(ingenic_pwm) & pwm_readl(ingenic_pwm,PWM_SIE);

	if(store_st) {
		n = ffs(store_st) - 1;
		pwm_store_clear_status(ingenic_pwm, n);
		ingenic_pwm->chan[n].store_irq_num++;
		printk("%d %d %d %d\n",n ,pwm_get_store_num(ingenic_pwm, n),
				pwm_get_output_num(ingenic_pwm, n), ingenic_pwm->chan[n].store_irq_num);
		goto end;
	}

	if(under_st) {
		n = ffs(under_st) - 1;
		pwm_dma_clear_under(ingenic_pwm, n);
		/* printk("pwm%d under come!\n", n); */
		goto end;
	}

	printk("pwm irq entry, but not pwm valid irq!\n");
end:
	return IRQ_HANDLED;
}

static int ingenic_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct ingenic_pwm_chip *ingenic_pwm;
	struct resource *res;
	int err = 0;
	int ret = 0;
	int i = 0;

	/* there is a single chip with multiple channels */
	chip = devm_pwmchip_alloc(&pdev->dev, 1, sizeof(*ingenic_pwm));

	if (!chip) {
		dev_err(&pdev->dev, "Ingenic pwm_chip alloc error\n");
		return -ENOMEM;
	}
	ingenic_pwm = to_ingenic_pwm(chip);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		return  -ENOENT;
	}

	ingenic_pwm->irq = platform_get_irq(pdev, 0);
	if (ingenic_pwm->irq < 0){
		dev_err(&pdev->dev, "Cannot get %d  IORESOURCE_IRQ\n", ingenic_pwm->irq);
		return  -ENOENT;
	}

	ingenic_pwm->iomem = ioremap(res->start, (res->end - res->start) + 1);
	if (ingenic_pwm->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	ingenic_pwm->clk_gate = devm_clk_get(&pdev->dev, "gate_pwm");
	if (IS_ERR(ingenic_pwm->clk_gate)) {
		dev_err(&pdev->dev, "get pwm clk gate failed %ld\n", PTR_ERR(ingenic_pwm->clk_gate));
		return PTR_ERR(ingenic_pwm->clk_gate);
	}

	ingenic_pwm->clk_pwm = devm_clk_get(&pdev->dev, "div_pwm");
	if (IS_ERR(ingenic_pwm->clk_pwm)){
		dev_err(&pdev->dev, "get pwm clk failed %ld\n", PTR_ERR(ingenic_pwm->clk_pwm));
		return PTR_ERR(ingenic_pwm->clk_pwm);
	}

	if (ingenic_pwm->clk_gate) {
		ret = clk_prepare_enable(ingenic_pwm->clk_gate);
		if(ret) {
			dev_err(&pdev->dev, "enable pwm clock gate failed!\n");
		}
	}

	if (ingenic_pwm->clk_pwm) {
		ret = clk_set_rate(ingenic_pwm->clk_pwm, DEFAULT_PWM_CLK_RATE);
		if(ret) {
			dev_err(&pdev->dev, "set pwm clock rate failed!\n");
		}
		ret = clk_prepare_enable(ingenic_pwm->clk_pwm);
		if(ret) {
			dev_err(&pdev->dev, "enable pwm clock failed!\n");
		}
	}

	chip->ops = &ingenic_pwm_ops;
	chip->npwm = INGENIC_PWM_NUM;
	ingenic_pwm->phys = res->start;

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret < 0)
		return ret;

	{
		dma_cap_mask_t mask;
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		for(i = 0; i < INGENIC_PWM_NUM; i++) {
			char str[2] = "0";

			ingenic_pwm->chan[i].id = i;
			ingenic_pwm->chan[i].chip = ingenic_pwm;
			sprintf(str, "%d", i);
			ingenic_pwm->chan[i].dma_chan = dma_request_chan(&pdev->dev, str);
			if(!ingenic_pwm->chan[i].dma_chan) {
				dev_err(&pdev->dev, "PWM request dma tx channel failed");
			}
			ingenic_pwm->chan[i].sg = kmalloc(sizeof(struct scatterlist), GFP_KERNEL);
			if (!ingenic_pwm->chan[i].sg) {
				dev_err(&pdev->dev, "Failed to alloc tx scatterlist\n");
			}
			ingenic_pwm->chan[i].init_level = 0;
			ingenic_pwm->chan[i].finish_level = 0;
			ingenic_pwm->chan[i].trigger_en = 0;
			ingenic_pwm->chan[i].sg_pwm_num = SG_PWM_NUM;
		}
	}

	ret = request_irq(ingenic_pwm->irq,ingenic_pwm_interrupt,
			IRQF_SHARED | IRQF_TRIGGER_LOW,"pwm-interrupt", chip);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed !! %d-\n", ingenic_pwm->irq);
		goto err_no_iomap;
	}

	platform_set_drvdata(pdev, ingenic_pwm);
	mutex_init(&ingenic_pwm->mutex);

	for (i = 0; i < ARRAY_SIZE(pwm_device_attributes); i++) {
		ret = device_create_file(&pdev->dev, &pwm_device_attributes[i]);
		if (ret)
			dev_warn(&pdev->dev, "attribute %d create failed", i);
	}

	dev_info(&pdev->dev, "ingenic-x1600 Probe of pwm success!\n");
	return 0;

err_no_iomap:
	iounmap(ingenic_pwm->iomem);

	return err;
}

static void ingenic_pwm_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *ingenic_pwm;
	ingenic_pwm =platform_get_drvdata(pdev);
	if (!ingenic_pwm)
		return -ENODEV;

	devm_clk_put(&pdev->dev, ingenic_pwm->clk_pwm);
	devm_clk_put(&pdev->dev, ingenic_pwm->clk_gate);
}

static const struct of_device_id ingenic_pwm_matches[] = {
	{ .compatible = "ingenic,x1600-pwm", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_pwm_matches);

#ifdef CONFIG_PM_SLEEP
static int ingenic_pwm_suspend(struct device *dev)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	unsigned int i;

	/*
	 * No one preserves these values during suspend so reset them.
	 * Otherwise driver leaves PWM unconfigured if same values are
	 * passed to pwm_config() next time.
	 */
	for (i = 0; i < INGENIC_PWM_NUM; ++i) {
#if FIXME
		struct pwm_device *pwm = &ingenic_pwm->chip->pwms[i];
		struct ingenic_pwm_channel *chan = pwm_get_chip_data(pwm);

		if (!chan)
			continue;
		chan->period_ns = 0;
		chan->duty_ns = 0;
#endif
	}

	return 0;
}

static int ingenic_pwm_resume(struct device *dev)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	unsigned int chan;

	/*
	 * Inverter setting must be preserved across suspend/resume
	 * as nobody really seems to configure it more than once.
	 */
	for (chan = 0; chan < INGENIC_PWM_NUM; ++chan) {
		if (ingenic_pwm->output_mask & BIT(chan)) {
			/*TODO: ??*/
		}
	}
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ingenic_pwm_pm_ops, ingenic_pwm_suspend, ingenic_pwm_resume);

static struct platform_driver ingenic_pwm_driver = {
	.driver = {
		.name = "ingenic-pwm",
		.pm = &ingenic_pwm_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_pwm_matches),
	},
	.probe = ingenic_pwm_probe,
	.remove = ingenic_pwm_remove,
};
module_platform_driver(ingenic_pwm_driver);

MODULE_DESCRIPTION("Ingenic SoC PWM driver");
MODULE_ALIAS("platform:ingenic-pwm");
MODULE_LICENSE("GPL");
