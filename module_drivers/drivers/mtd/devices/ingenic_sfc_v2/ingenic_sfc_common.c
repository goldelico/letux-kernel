/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "ingenic_sfc_common.h"
#include "spinor.h"


#define SFC_REG_DEBUG

#define SFC_ASSERT(sfc, cond) do {				\
    if (!(cond)) {						\
        __assert(__FILE__, __LINE__, sfc, #cond);		\
    }								\
} while (0)

#define GET_PHYADDR(a)  \
({						\
	unsigned int v;        \
	if (unlikely((unsigned int)(a) & 0x40000000)) {    \
	v = page_to_phys(vmalloc_to_page((const void *)(a))) | ((unsigned int)(a) & ~PAGE_MASK); \
	} else     \
	v = ((unsigned int)(a) & 0x1fffffff);                   \
	v;                                             \
 })

static inline void sfc_writel(struct sfc *sfc, unsigned short offset, u32 value)
{
	writel(value, sfc->iomem + offset);
}

static inline unsigned int sfc_readl(struct sfc *sfc, unsigned short offset)
{
	return readl(sfc->iomem + offset);
}

#ifdef SFC_REG_DEBUG
void dump_sfc_reg(struct sfc *sfc)
{
	int i = 0;

	printk("\nSFC_GLB0	 = %08x\n", sfc_readl(sfc, SFC_GLB0));
	printk("SFC_DEV_CONF	 = %08x\n", sfc_readl(sfc, SFC_DEV_CONF));
	printk("SFC_DEV_STA_EXP	 = %08x\n", sfc_readl(sfc, SFC_DEV_STA_EXP));
	printk("SFC_DEV0_STA_RT	 = %08x\n", sfc_readl(sfc, SFC_DEV0_STA_RT));
	printk("SFC_DEV_STA_MASK = %08x\n", sfc_readl(sfc, SFC_DEV_STA_MSK));
	for(i = 0; i < 6; i++){
		printk("SFC_TRAN_CONF0(%d) = %08x\n", i, sfc_readl(sfc, SFC_TRAN_CONF0(i)));
	}
	printk("SFC_TRAN_LEN	 = %08x\n", sfc_readl(sfc, SFC_TRAN_LEN));
	for(i = 0; i < 6; i++){
		printk("SFC_DEV_ADDR%d		= %08x\n",	i, sfc_readl(sfc, SFC_DEV_ADDR(i)));
		printk("SFC_DEV_ADDR_PLUS%d	= %08x\n",	i, sfc_readl(sfc, SFC_DEV_ADDR_PLUS(i)));
	}

	printk("SFC_MEM_ADDR	= %08x\n", sfc_readl(sfc, SFC_MEM_ADDR));
	printk("SFC_TRIG	= %08x\n", sfc_readl(sfc, SFC_TRIG));
	printk("SFC_SR		= %08x\n", sfc_readl(sfc, SFC_SR));
	printk("SFC_SCR		= %08x\n", sfc_readl(sfc, SFC_SCR));
	printk("SFC_INTC	= %08x\n", sfc_readl(sfc, SFC_INTC));
	printk("SFC_FSM		= %08x\n", sfc_readl(sfc, SFC_FSM));
	printk("SFC_CGE		= %08x\n", sfc_readl(sfc, SFC_CGE));
	printk("SFC_CMD_IDX	= %08x\n", sfc_readl(sfc, SFC_CMD_IDX));
	printk("SFC_COL_ADDR	= %08x\n", sfc_readl(sfc, SFC_COL_ADDR));
	printk("SFC_ROW_ADDR	= %08x\n", sfc_readl(sfc, SFC_ROW_ADDR));
	printk("SFC_STA_ADDR0	= %08x\n", sfc_readl(sfc, SFC_STA_ADDR0));
	printk("SFC_STA_ADDR1	= %08x\n", sfc_readl(sfc, SFC_STA_ADDR1));
	printk("SFC_DES_ADDR	= %08x\n", sfc_readl(sfc, SFC_DES_ADDR));
	printk("SFC_GLB1	= %08x\n", sfc_readl(sfc, SFC_GLB1));
	printk("SFC_DEV1_STA_RT = %08x\n", sfc_readl(sfc, SFC_DEV1_STA_RT));
	for(i = 0; i < 6; i++) {
		printk("SFC_TRAN_CONF1(%d)	= %08x\n", i, sfc_readl(sfc, SFC_TRAN_CONF1(i)));
	}
	//printk("SFC_CDT	= %08x\n", sfc_readl(sfc, SFC_CDT));
	//printk("SFC_DR	= %08x\n", sfc_readl(sfc, SFC_RM_DR));
}

void dump_cdt(struct sfc *sfc)
{
	struct sfc_cdt *cdt;
	int i;

	if(sfc->iomem == NULL){
		printk("%s error: sfc res not init !\n", __func__);
		return;
	}

	cdt = sfc->iomem + 0x800;

	for(i = 0; i < 32; i++){
		printk("\nnum------->%d\n", i);
		printk("link:%02x, ENDIAN:%02x, WORD_UINT:%02x, TRAN_MODE:%02x, ADDR_KIND:%02x\n",
				(cdt[i].link >> 31) & 0x1, (cdt[i].link >> 18) & 0x1,
				(cdt[i].link >> 16) & 0x3, (cdt[i].link >> 4) & 0xf,
				(cdt[i].link >> 0) & 0x3
				);
		printk("CLK_MODE:%02x, ADDR_WIDTH:%02x, POLL_EN:%02x, CMD_EN:%02x,PHASE_FORMAT:%02x, DMY_BITS:%02x, DATA_EN:%02x, TRAN_CMD:%04x\n",
				(cdt[i].xfer >> 29) & 0x7, (cdt[i].xfer >> 26) & 0x7,
				(cdt[i].xfer >> 25) & 0x1, (cdt[i].xfer >> 24) & 0x1,
				(cdt[i].xfer >> 23) & 0x1, (cdt[i].xfer >> 17) & 0x3f,
				(cdt[i].xfer >> 16) & 0x1, (cdt[i].xfer >> 0) & 0xffff
				);
		printk("DEV_STA_EXP:%08x\n", cdt[i].staExp);
		printk("DEV_STA_MSK:%08x\n", cdt[i].staMsk);
	}
}

void dump_desc(struct sfc *sfc, uint32_t desc_num)
{
	struct sfc_desc *desc = sfc->desc;
	int i = 0;

	for(; i < desc_num; i++){
		printk("\nDMA Descriptor ---->num: %d, addr: 0x%08x\n", i, (unsigned int)virt_to_phys(&desc[i]));
		printk("next_desc_addr: 0x%08x\n", desc[i].next_des_addr);
		printk("mem_addr: 0x%08x\n", desc[i].mem_addr);
		printk("tran_len: %d\n", desc[i].tran_len);
		printk("link: %d\n\n", desc[i].link);
	}
}

void dump_sfc_debug(struct sfc *sfc)
{
	pr_err("\n######################### sfc dump ###########################\n");
	pr_err("\nsfc reg: \n");
	dump_sfc_reg(sfc);
	pr_err("\nsfc DMA Descriptor chain: \n");
	dump_desc(sfc, sfc->desc_max_num);
	pr_err("\nsfc CDT table: \n");
	dump_cdt(sfc);
	pr_err("\n######################### sfc dump end ###########################\n");
}

#endif

void __assert(const char *file, int line, struct sfc *sfc, const char *cond)
{
    printk("assert %s failed in %s %d\n", cond, file, line);
#ifdef SFC_REG_DEBUG
	dump_sfc_debug(sfc);
#endif
	while(1);
}

static int32_t sfc_stop(struct sfc *sfc)
{
	int32_t timeout = 0xffff;
	sfc_writel(sfc, SFC_TRIG, TRIG_STOP);

	while((sfc_readl(sfc, SFC_SR) & SFC_BUSY) && timeout--);
	if(timeout < 0)
		return -EIO;
	return 0;
}

static inline void sfc_init(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_TRIG, TRIG_STOP);
	sfc_writel(sfc, SFC_DEV_CONF, 0);

	/* X1000 need set to 0,but X2000 can be set to 1*/
	sfc_writel(sfc, SFC_CGE, 0);

}

static inline void sfc_start(struct sfc *sfc)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_START;
	sfc_writel(sfc, SFC_TRIG, tmp);
}

static inline void sfc_flush_fifo(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_FLUSH;
	sfc_writel(sfc, SFC_TRIG, tmp);
}
static inline void  sfc_clear_end_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_END);
}

static inline void sfc_clear_treq_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_TREQ);
}

static inline void sfc_clear_rreq_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_RREQ);
}

static inline void sfc_clear_over_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_OVER);
}

static inline void sfc_clear_under_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_UNDER);
}

static inline void sfc_clear_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, 0x1f);
}

static inline void sfc_mask_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0x1f);
}

static void sfc_dev_hw_init(struct sfc *sfc)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_DEV_CONF);

	/*cpha bit:0 , cpol bit:0 */
	tmp &= ~(DEV_CONF_CPHA | DEV_CONF_CPOL);
	/*ce_dl bit:1, hold bit:1,wp bit:1*/
	tmp |= (DEV_CONF_CEDL | DEV_CONF_HOLDDL | DEV_CONF_WPDL);
	sfc_writel(sfc, SFC_DEV_CONF, tmp);

	/* use CDT mode */
	printk("Enter 'CDT' mode.\n");
	tmp = sfc_readl(sfc, SFC_GLB0);
	tmp |= GLB0_CDT_EN;
	sfc_writel(sfc, SFC_GLB0, tmp);

	/* use DMA Descriptor chain mode */
	printk("Enter 'DMA Descriptor chain' mode.\n");
	tmp = sfc_readl(sfc, SFC_GLB0);
	tmp |= GLB0_DES_EN;
	sfc_writel(sfc, SFC_GLB0, tmp);
}

static void sfc_threshold(struct sfc *sfc, uint32_t value)
{
	uint32_t tmp = sfc_readl(sfc, SFC_GLB0);
	tmp &= ~GLB0_THRESHOLD_MSK;
	tmp |= value << GLB0_THRESHOLD_OFFSET;
	sfc_writel(sfc, SFC_GLB0, tmp);
}

static void sfc_smp_delay(struct sfc *sfc, uint32_t value)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_DEV_CONF);
	tmp &= ~DEV_CONF_SMP_DELAY_MSK;
	tmp |= value << DEV_CONF_SMP_DELAY_OFFSET;
	sfc_writel(sfc, SFC_DEV_CONF, tmp);
}

static int timing_is_valid(uint32_t *p_thold, uint32_t *p_tsetup, uint32_t *p_tsh)
{
	int ret = 0;

#define THOLD_LO_VAL 0x0
#define THOLD_HI_VAL 0x3
#define TSETUP_LO_VAL 0x0
#define TSETUP_HI_VAL 0x3
#define TSH_LO_VAL 0x0
#define TSH_HI_VAL 0xf

	if ((*p_thold > THOLD_HI_VAL) || (*p_thold < THOLD_LO_VAL)) {
		pr_err("ERROR: Check that the SFC timing parameter is invalid, thold:%d !\n", *p_thold);
		*p_thold = clamp((uint32_t)*p_thold, (uint32_t)THOLD_LO_VAL, (uint32_t)THOLD_HI_VAL);
		ret = -EINVAL;
	}

	if ((*p_tsetup > TSETUP_HI_VAL) || (p_thold < TSETUP_LO_VAL)) {
		pr_err("ERROR: Check that the SFC timing parameter is invalid, tsetup:%d !\n", *p_tsetup);
		*p_tsetup = clamp((uint32_t)*p_tsetup, (uint32_t)TSETUP_LO_VAL, (uint32_t)TSETUP_HI_VAL);
		ret = -EINVAL;
	}

	if ((*p_tsh > TSH_HI_VAL) || (*p_tsh < TSH_LO_VAL)){
		pr_err("ERROR: Check that the SFC timing parameter is invalid, tsh:%d !\n", *p_tsh);
		*p_tsh = clamp((uint32_t)*p_tsh, (uint32_t)TSH_LO_VAL, (uint32_t)TSH_HI_VAL);
		ret = -EINVAL;
	}

	return ret;
}


int32_t set_flash_timing(struct sfc *sfc, uint32_t t_hold, uint32_t t_setup, uint32_t t_shslrd, uint32_t t_shslwr)
{
	uint32_t c_hold, c_setup, t_in, c_in;
	uint32_t c_half_hold, c_half_setup, c_half_in;
	uint32_t thold, tsetup, tsh;
	uint32_t tmp;
	unsigned long cycle;
	unsigned long half_cycle;
	unsigned long long ns;

	/* NOTE: 4 frequency division. */
	sfc->src_clk /= 4;

	ns = 1000000000ULL;
	do_div(ns, sfc->src_clk);
	cycle = ns;
	half_cycle = cycle / 2;


	/* Calculate the number of cycle */
	c_hold = t_hold / cycle;
	c_half_hold = t_hold % cycle;
	if(c_half_hold > half_cycle) {
		c_half_hold = 0;
		c_hold += 1;
	}

	c_setup = t_setup / cycle;
	c_half_setup = t_setup % cycle;
	if(c_half_setup > half_cycle) {
		c_half_setup = 0;
		c_setup += 1;
	}

	t_in = max(t_shslrd, t_shslwr);
	c_in = t_in / cycle;
	c_half_in = t_in % cycle;
	if(c_half_in > half_cycle) {
		c_half_in = 0;
		c_in += 1;
	}

	/* Calculate timing parameters */
	if(!c_hold && !c_half_hold)
		thold = 0;
	else
		thold = (2 * c_hold) - 1 + (!!c_half_hold);

	if(!c_setup && !c_half_setup)
		tsetup = 0;
	else
		tsetup = (2 * c_setup) - 1 + (!!c_half_setup);

	if(!c_in && !c_half_in)
		tsh = 0;
	else
		tsh = (2 * c_in) - 1 + (!!c_half_in);

	/* Verify parameters validity */
	timing_is_valid(&thold, &tsetup, &tsh);

	tmp = sfc_readl(sfc, SFC_DEV_CONF);
	tmp &= ~(DEV_CONF_THOLD_MSK | DEV_CONF_TSETUP_MSK | DEV_CONF_TSH_MSK);
	tmp |= (thold << DEV_CONF_THOLD_OFFSET) | \
		  (tsetup << DEV_CONF_TSETUP_OFFSET) | \
		  (tsh << DEV_CONF_TSH_OFFSET);

	sfc_writel(sfc, SFC_DEV_CONF, tmp);
	return 0;
}

static void sfc_set_length(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_TRAN_LEN, value);
}

static inline void sfc_transfer_mode(struct sfc *sfc, uint32_t value)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_GLB0);
	if(value == 0)
		tmp &= ~GLB0_OP_MODE;
	else
		tmp |= GLB0_OP_MODE;
	sfc_writel(sfc, SFC_GLB0, tmp);
}

static void sfc_read_data(struct sfc *sfc, uint32_t *value)
{
	*value = sfc_readl(sfc, SFC_RM_DR);
}

static void sfc_write_data(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_RM_DR, value);
}

static void cpu_read_rxfifo(struct sfc *sfc, struct sfc_cdt_xfer *xfer)
{
	int32_t i = 0;
	uint32_t align_len = 0;
	uint32_t fifo_num = 0;
	uint32_t last_word = 0;
	uint32_t unalign_data;
	uint8_t *c;

	align_len = ALIGN(xfer->config.datalen, 4);

	if(((align_len - xfer->config.cur_len) / 4) > sfc->threshold) {
		fifo_num = sfc->threshold;
		last_word = 0;
	} else {
		/* last aligned THRESHOLD data*/
		if(xfer->config.datalen % 4) {
			fifo_num = (align_len - xfer->config.cur_len) / 4 - 1;
			last_word = 1;
		} else {
			fifo_num = (align_len - xfer->config.cur_len) / 4;
			last_word = 0;
		}
	}

	if ((uint32_t)xfer->config.buf & 0x3) {
		/* addr not align */
		for (i = 0; i < fifo_num; i++) {
			sfc_read_data(sfc, &unalign_data);
			c = xfer->config.buf;
			c[0] = (unalign_data >> 0) & 0xff;
			c[1] = (unalign_data >> 8) & 0xff;
			c[2] = (unalign_data >> 16) & 0xff;
			c[3] = (unalign_data >> 24) & 0xff;

			xfer->config.buf += 4;
			xfer->config.cur_len += 4;
		}
	} else {
		/* addr align */
		for (i = 0; i < fifo_num; i++) {
			sfc_read_data(sfc, (uint32_t *)xfer->config.buf);
			xfer->config.buf += 4;
			xfer->config.cur_len += 4;
		}
	}

	/* last word */
	if(last_word == 1) {
		sfc_read_data(sfc, &unalign_data);
		c = (uint8_t *)xfer->config.buf;

		for(i = 0; i < xfer->config.datalen % 4; i++) {
			c[i] = (unalign_data >> (i * 8)) & 0xff;
		}

		xfer->config.buf += xfer->config.datalen % 4;
		xfer->config.cur_len += xfer->config.datalen % 4;
	}

}

static void cpu_write_txfifo(struct sfc *sfc, struct sfc_cdt_xfer *xfer)
{
	uint32_t align_len = 0;
	uint32_t fifo_num = 0;
	uint32_t data = 0;
	uint32_t i;
	uint32_t nbytes = xfer->config.datalen % 4;

	align_len = xfer->config.datalen / 4 * 4;

	if (((align_len - xfer->config.cur_len) / 4) >= sfc->threshold) {
		fifo_num = sfc->threshold;
		nbytes = 0;
	} else {
		fifo_num = (align_len - xfer->config.cur_len) / 4;
	}

	if ((uint32_t)xfer->config.buf & 0x3) {
		/* addr not align */
		for(i = 0; i < fifo_num; i++) {
			data = xfer->config.buf[3] << 24 | xfer->config.buf[2] << 16 | xfer->config.buf[1] << 8 | xfer->config.buf[0];
			sfc_write_data(sfc, data);
			xfer->config.buf += 4;
			xfer->config.cur_len += 4;
		}
	} else {
		/* addr align */
		for(i = 0; i < fifo_num; i++) {
			sfc_write_data(sfc, *(uint32_t *)xfer->config.buf);
			xfer->config.buf += 4;
			xfer->config.cur_len += 4;
		}
	}

	if(nbytes) {
		data = 0;
		for(i = 0; i < nbytes; i++)
			data |= xfer->config.buf[i] << i * 8;
		sfc_write_data(sfc, data);
		xfer->config.cur_len += nbytes;
	}

}

uint32_t sfc_get_sta_rt0(struct sfc *sfc)
{
	return sfc_readl(sfc, SFC_DEV0_STA_RT);
}


static void sfc_enable_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0);
}

static void sfc_set_mem_addr(struct sfc *sfc, unsigned int addr)
{
	sfc_writel(sfc, SFC_MEM_ADDR, addr);
}

static void sfc_set_desc_addr(struct sfc *sfc, unsigned int addr)
{
	sfc_writel(sfc, SFC_DES_ADDR, addr);
}

void *sfc_get_paddr(void *vaddr)
{
	unsigned long paddr;
	unsigned int pfn = 0;
	unsigned int page_offset = 0;

	if (is_vmalloc_addr(vaddr)) {
		pfn = vmalloc_to_pfn(vaddr);
		page_offset = (unsigned int)vaddr & (PAGE_SIZE - 1);
		paddr = (pfn << 12) + page_offset;
	} else {
		paddr = virt_to_phys(vaddr);
	}

	return (void *)paddr;
}

int32_t create_sfc_desc(struct sfc *sfc, unsigned char *vaddr, size_t len)
{
	struct sfc_desc *desc = sfc->desc;
	uint32_t ualign_size, off_len, last_len, step_len, page_num;
	int current_pfn = 0, next_pfn = 0;
	int32_t i = 0, j = 0;

	ualign_size = (unsigned int)vaddr & (PAGE_SIZE - 1);
	off_len = PAGE_SIZE - ualign_size;

	if(is_vmalloc_addr(vaddr) && (len > off_len)){
		page_num = (len - off_len) >> (ffs(PAGE_SIZE) - 1);
		last_len = (len - off_len) & (PAGE_SIZE - 1);
		current_pfn = vmalloc_to_pfn(vaddr);

		desc[i].next_des_addr = 0;
		desc[i].mem_addr = (unsigned int)sfc_get_paddr((void *)vaddr);
		desc[i].tran_len = off_len;
		desc[i].link = 1;

		vaddr += off_len;
		step_len = PAGE_SIZE;

		/* case 1. Handle physical address discontinuity */
		do{
			if(!page_num){
				if(last_len)
					step_len = last_len;
				else{
					break;
				}
			}

			next_pfn = vmalloc_to_pfn(vaddr);
			if((current_pfn + 1) != next_pfn){
				if(++i > (sfc->desc_max_num - 1)){
					dev_err(sfc->dev, "%s The number of descriptors exceeds the maximum limit.\n", __func__);
					return -ENOMEM;
				}

				desc[i-1].next_des_addr = (unsigned int)sfc_get_paddr((void *)&desc[i]);

				desc[i].next_des_addr = 0;
				desc[i].mem_addr = (unsigned int)sfc_get_paddr((void *)vaddr);
				desc[i].tran_len = step_len;
				desc[i].link = 1;
			}else{
				desc[i].tran_len += step_len;
			}


			if(page_num){
				current_pfn = next_pfn;
				vaddr += step_len;
			}
		}while(page_num--);
	}else{
		/* case 2. Physical Address Continuity and only need one descriptor */
		desc[i].next_des_addr = 0;
		desc[i].mem_addr = (unsigned int)sfc_get_paddr((void *)vaddr);
		desc[i].tran_len = len;
	}

	/* last descriptor is not link */
	desc[i].link = 0;

	return i;
}

int request_sfc_desc(struct sfc *sfc, int desc_max_num)
{
	sfc->desc = (struct sfc_desc *)dma_alloc_coherent(sfc->dev,
			sizeof(struct sfc_desc) * desc_max_num, &sfc->desc_pyaddr, GFP_KERNEL);
	if (IS_ERR_OR_NULL(sfc->desc)) {
		return -ENOMEM;
	}
	sfc->desc_max_num = desc_max_num;

	return 0;
}

void free_sfc_desc(struct sfc *sfc)
{
	return dma_free_coherent(sfc->dev, sizeof(struct sfc_desc) * sfc->desc_max_num, sfc->desc, sfc->desc_pyaddr);
}

#define SFC_TRANSFER_TIMEOUT	3000	//3000ms for timeout
static int32_t sfc_start_transfer(struct sfc *sfc)
{
	int32_t err;
	sfc_clear_all_intc(sfc);
	sfc_enable_all_intc(sfc);
	sfc_start(sfc);
	err = wait_for_completion_timeout(&sfc->done, msecs_to_jiffies(SFC_TRANSFER_TIMEOUT));
	if (!err) {
#ifdef SFC_REG_DEBUG
		dump_sfc_debug(sfc);
#endif
		sfc_mask_all_intc(sfc);
		sfc_clear_all_intc(sfc);

		sfc_stop(sfc);
		sfc_flush_fifo(sfc);

		printk("line:%d Timeout for ACK from SFC device\n",__LINE__);
		return -ETIMEDOUT;
	}
	return 0;
}

void write_cdt(struct sfc *sfc, struct sfc_cdt *cdt, uint16_t start_index, uint16_t end_index)
{
	uint32_t cdt_num, cdt_size;

	cdt_num = end_index - start_index + 1;
	cdt_size = sizeof(struct sfc_cdt);

	memcpy((void *)sfc->iomem + SFC_CDT + (start_index * cdt_size), (void *)cdt + (start_index * cdt_size), cdt_num * cdt_size);
	printk("create CDT index: %d ~ %d,  index number:%d.\n", start_index, end_index, cdt_num);
}

static void sfc_set_index(struct sfc *sfc, unsigned short index)
{

	uint32_t tmp = sfc_readl(sfc, SFC_CMD_IDX);
	tmp &= ~CMD_IDX_MSK;
	tmp |= index;
	sfc_writel(sfc, SFC_CMD_IDX, tmp);
}

static void sfc_set_dataen(struct sfc *sfc, uint8_t dataen)
{

	uint32_t tmp = sfc_readl(sfc, SFC_CMD_IDX);
	tmp &= ~CDT_DATAEN_MSK;
	tmp |= (dataen << CDT_DATAEN_OFF);
	sfc_writel(sfc, SFC_CMD_IDX, tmp);
}

static void sfc_set_datadir(struct sfc *sfc, uint8_t datadir)
{

	uint32_t tmp = sfc_readl(sfc, SFC_CMD_IDX);
	tmp &= ~CDT_DIR_MSK;
	tmp |= (datadir << CDT_DIR_OFF);
	sfc_writel(sfc, SFC_CMD_IDX, tmp);
}


int sfc_sync_cdt(struct sfc *sfc, struct sfc_cdt_xfer *xfer)
{
	/*0.reset transfer length*/
	sfc_set_length(sfc, 0);

	/*1. set index*/
	sfc_set_index(sfc, xfer->cmd_index);

	/*2. set addr*/
	sfc_writel(sfc, SFC_COL_ADDR, xfer->columnaddr);
	sfc_writel(sfc, SFC_ROW_ADDR, xfer->rowaddr);
	sfc_writel(sfc, SFC_STA_ADDR0, xfer->staaddr0);
	sfc_writel(sfc, SFC_STA_ADDR1, xfer->staaddr1);

	/*3. config data*/
	sfc_set_dataen(sfc, xfer->dataen);
	if(xfer->dataen){
		sfc_set_datadir(sfc, xfer->config.data_dir);
		sfc_transfer_mode(sfc, xfer->config.ops_mode);
		sfc_set_length(sfc, xfer->config.datalen);

		/* Memory address for DMA when do not use DMA descriptor */
		sfc_set_mem_addr(sfc, 0);

		if(xfer->config.ops_mode == DMA_OPS){
			if(xfer->config.data_dir == GLB0_TRAN_DIR_READ){
				dma_sync_single_for_device(sfc->dev, (dma_addr_t)(sfc_get_paddr((void *)xfer->config.buf)), xfer->config.datalen, DMA_FROM_DEVICE);
			}else{
				dma_sync_single_for_device(sfc->dev, (dma_addr_t)(sfc_get_paddr((void *)xfer->config.buf)), xfer->config.datalen, DMA_TO_DEVICE);
			}
			/* Set Descriptor address for DMA */
			sfc_set_desc_addr(sfc, virt_to_phys(sfc->desc));
		}
		sfc->xfer = xfer;
	}

	return sfc_start_transfer(sfc);

}

static irqreturn_t ingenic_sfc_pio_irq_callback(int32_t irq, void *dev)
{
	struct sfc *sfc = dev;
	uint32_t val;
	uint8_t err_flag = 0;

	val = sfc_readl(sfc, SFC_SR) & 0x1f;

	spin_lock(&sfc->spin_lock);

	if(val & CLR_RREQ) {
		sfc_clear_rreq_intc(sfc);
		cpu_read_rxfifo(sfc, sfc->xfer);
	} else if(val & CLR_TREQ) {
		sfc_clear_treq_intc(sfc);
		cpu_write_txfifo(sfc, sfc->xfer);
	} else if(val & CLR_OVER) {
		sfc_clear_over_intc(sfc);
		pr_err("sfc OVER !\n");
		err_flag = 1;
	} else if(val & CLR_UNDER) {
		sfc_clear_under_intc(sfc);
		pr_err("sfc UNDR !\n");
		err_flag = 1;
	} else if(val & CLR_END) {
		sfc_mask_all_intc(sfc);
		sfc_clear_end_intc(sfc);
		sfc->retry_count = 0;
		complete(&sfc->done);
	}

	if (err_flag) {
#ifdef SFC_REG_DEBUG
		dump_sfc_debug(sfc);
#endif
		if (sfc->retry_count > 0) {
			sfc->retry_count--;
		} else if (sfc->retry_count == 0){
			sfc->retry_count = RETRY_COUNT;
		}

		sfc_clear_all_intc(sfc);
		sfc_mask_all_intc(sfc);
		complete(&sfc->done);
	}

	spin_unlock(&sfc->spin_lock);
	return IRQ_HANDLED;
}

static void ingenic_sfc_init_setup(struct sfc *sfc)
{
	sfc_init(sfc);
	sfc_threshold(sfc, sfc->threshold);
	sfc_dev_hw_init(sfc);

	sfc_transfer_mode(sfc, SLAVE_MODE);
}

int32_t sfc_clk_set_init(struct sfc *sfc)
{
	struct sfc_flash *flash = dev_get_drvdata(sfc->dev);
	if(flash->sfc_init_frequency)
		sfc->src_clk = flash->sfc_init_frequency;
	else
		sfc->src_clk = 200000000;


	/* set clock rate */
	clk_set_rate(sfc->clk, sfc->src_clk);

	/* sample delay */
	if(sfc->src_clk >= 200000000){
		sfc_smp_delay(sfc, DEV_CONF_SMP_DELAY_180);
	}

	return 0;
}

int32_t sfc_clk_set_highspeed(struct sfc *sfc)
{
	struct sfc_flash *flash = dev_get_drvdata(sfc->dev);
	struct spinor_flashinfo *nor_info = flash->flash_info;

	if(nor_info->quad_succeed)
		sfc->src_clk = flash->sfc_max_frequency;
	else {
		sfc->src_clk = flash->sfc_max_frequency;
		if (sfc->src_clk > 200000000) {
			printk("clk freq too hight! set to 200M\n");
			sfc->src_clk = 200000000;
		}
	}

	/* set clock rate */
	clk_set_rate(sfc->clk, sfc->src_clk);

	/* sample delay */
	if(sfc->src_clk >= 200000000){
		sfc_smp_delay(sfc, DEV_CONF_SMP_DELAY_180);
	}

	return 0;
}


struct sfc *sfc_res_init(struct platform_device *pdev)
{
	struct device_node* np = pdev->dev.of_node;
	struct ingenic_sfc_info *pdata_params;
	struct sfc *sfc;
	struct resource *res;
	int32_t err = 0;

	pdata_params = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_sfc_info), GFP_KERNEL);
	if(!pdata_params){
		printk("ERROR: %s %d devm_kzalloc() error !\n",__func__,__LINE__);
		return ERR_PTR(-ENOMEM);
	}

	sfc = devm_kzalloc(&pdev->dev, sizeof(struct sfc), GFP_KERNEL);
	if (!sfc) {
		printk("ERROR: %s %d devm_kzalloc() error !\n",__func__,__LINE__);
		return ERR_PTR(-ENOMEM);
	}

	sfc->dev = &pdev->dev;

	err = of_property_read_u32(np, "ingenic,spiflash_param_offset", (unsigned int *)&pdata_params->param_offset);
	if (err < 0) {
		dev_err(&pdev->dev, "No dts param_offset, use default.\n");
		pdata_params->param_offset = -EINVAL;
	}

	err = of_property_read_u8(np, "ingenic,use_ofpart_info", &pdata_params->use_ofpart_info);
	if (err < 0) {
		dev_err(&pdev->dev, "Cannot get sfc use_ofpart_info\n");
		return ERR_PTR(-ENOENT);
	}

	err = platform_device_add_data(pdev, pdata_params, sizeof(struct ingenic_sfc_info));
	if(err){
		printk("ERROR: %s %d error !\n",__func__,__LINE__);
		return ERR_PTR(-ENOMEM);
	}

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		return ERR_PTR(-ENOENT);
	}

	sfc->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (sfc->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		return ERR_PTR(-ENXIO);
	}

	sfc->clk = devm_clk_get(&pdev->dev, "div_sfc");
	if (IS_ERR(sfc->clk)) {
		dev_err(&pdev->dev, "Cannot get div_sfc clock\n");
		return ERR_PTR(-ENOENT);
	}

	sfc->clk_gate = devm_clk_get(&pdev->dev, "gate_sfc");
	if (IS_ERR(sfc->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get sfc clock\n");
		return ERR_PTR(-ENOENT);
	}

	err = sfc_clk_set_init(sfc);
	if (err) {
		return ERR_PTR(-ENOENT);
	}

	if(clk_prepare_enable(sfc->clk)) {
		dev_err(&pdev->dev, "cgu clk error\n");
		return ERR_PTR(-ENOENT);
	}
	if(clk_prepare_enable(sfc->clk_gate)) {
		dev_err(&pdev->dev, "gate clk error\n");
		return ERR_PTR(-ENOENT);
	}

	/* SFC parameter initialization */
	sfc->threshold = THRESHOLD;
	sfc->retry_count = 0;

	/* request SFC irq */
	sfc->irq = platform_get_irq(pdev, 0);
	if (sfc->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		return ERR_PTR(-ENOENT);
	}

	err = devm_request_irq(&pdev->dev, sfc->irq, ingenic_sfc_pio_irq_callback, 0, pdev->name, sfc);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return ERR_PTR(-EINVAL);
	}

	/* SFC controller initializations for SFC */
	ingenic_sfc_init_setup(sfc);
	spin_lock_init(&sfc->spin_lock);
	init_completion(&sfc->done);
	return sfc;
}
