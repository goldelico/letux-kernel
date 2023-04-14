/* linux/drivers/spi/ingenic_sslv.c
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 * base-to: linux/drivers/spi/spi_bitbang.c
 *
 * Copyright (c) 2011 Ingenic
 * Author:Shumb <sbhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/spi/spidev.h>

#include "ingenic_slv.h"


#define FPGA_TEST_SPI_SLV
#define INTERRUPT_TEST
/* #define SSI_DEGUG */
#ifdef SSI_DEGUG
#define  print_dbg(format,arg...)			\
	printk(format,## arg)
#else
#define  print_dbg(format,arg...)
#endif

#if 0
#define INGENIC_SPI_RX_BUF(type) 				\
	u32 ingenic_sslv_rx_buf_##type(struct ingenic_sslv *hw) 		\
{								\
	u32 data  = ss_read(hw, SSLV_DR);			\
	printk("read data : 0x%08x\n",data); \
	type * rx = (type *)hw->rx;				\
	*rx++ = (type)(data);			  		\
	hw->rx = (u8 *)rx;					\
	return (u32)data;					\
}

#define INGENIC_SPI_TX_BUF(type)					\
	u32 ingenic_sslv_tx_buf_##type(struct ingenic_sslv *hw)		\
{								\
	u32 data;						\
	const type * tx = (type *)hw->tx;			\
	data = *tx++;						\
	hw->tx = (u8 *)tx;					\
	transmit_data(hw, data);				\
	return (u32)data;					\
}
	/* printk("data regs:   0x%08x\n",ss_read(hw, SSLV_DR));\ */
	/* printk("write data : 0x%08x\n",data); \ */
#else
#define INGENIC_SPI_RX_BUF(type) 				\
	u32 ingenic_sslv_rx_buf_##type(struct ingenic_sslv *hw) 		\
{								\
	u32 data  = ss_read(hw, SSLV_DR);			\
	type * rx = (type *)hw->rx;				\
	*rx++ = (type)(data);			  		\
	hw->rx = (u8 *)rx;					\
	return (u32)data;					\
}

#define INGENIC_SPI_TX_BUF(type)					\
	u32 ingenic_sslv_tx_buf_##type(struct ingenic_sslv *hw)		\
{								\
	u32 data;						\
	const type * tx = (type *)hw->tx;			\
	data = *tx++;						\
	hw->tx = (u8 *)tx;					\
	transmit_data(hw, data);				\
	return (u32)data;					\
}
#endif

INGENIC_SPI_RX_BUF(u8)
INGENIC_SPI_TX_BUF(u8)

INGENIC_SPI_RX_BUF(u16)
INGENIC_SPI_TX_BUF(u16)

INGENIC_SPI_RX_BUF(u32)
INGENIC_SPI_TX_BUF(u32)


void dump_spi_slaver_regs(struct ingenic_sslv *hw)
{
	printk("******************** dump slaver regs *********************\n");
	printk("CTRLR0(0x%08x) =		0x%08x\n",CTRLR0, ss_read(hw, CTRLR0));
	printk("ENABLE(0x%08x) =		0x%08x\n",0x8,  ss_read(hw, 0x8));
	printk("MIC CONTROL(0x%08x) =		0x%08x\n",0xc,  ss_read(hw, 0xc));
	printk("TFIFO THRES(0x%08x) = 		0x%08x\n",0x18, ss_read(hw, 0x18));
	printk("RFIFO THRES(0x%08x) = 		0x%08x\n",0x1c, ss_read(hw, 0x1c));
	printk("TFIFO LEVEL(0x%08x) = 		0x%08x\n",0x20, ss_read(hw, 0x20));
	printk("RFIFO LEVEL(0x%08x) = 		0x%08x\n",0x24, ss_read(hw, 0x24));
	printk("status reg(0x%08x) =		0x%08x\n",0x28, ss_read(hw, 0x28));
	printk("intc mask(0x%08x) = 		0x%08x\n",0x2c, ss_read(hw, 0x2c));
	printk("intc status(0x%08x) =		0x%08x\n",0x30, ss_read(hw, 0x30));
	printk("intc raw status(0x%08x) =	0x%08x\n",0x34, ss_read(hw, 0x34));
	printk("DMA control(0x%08x) =		0x%08x\n",0x4c, ss_read(hw, 0x4c));
	printk("DMA t level(0x%08x) =		0x%08x\n",0x50, ss_read(hw, 0x50));
	printk("DMA r level(0x%08x) = 		0x%08x\n",0x54, ss_read(hw, 0x54));
	printk("******************** dump slaver end *********************\n");
}

static void dma_tx_callback(void *data)
{
	struct ingenic_sslv *hw = data;
	dma_unmap_sg(hw->txchan->device->dev, hw->sg_tx, 1, DMA_TO_DEVICE);
	complete(&hw->done_tx_dma);
}

static void dma_rx_callback(void *data)
{
	struct ingenic_sslv *hw = data;
	dma_unmap_sg(hw->rxchan->device->dev, hw->sg_rx, 1, DMA_FROM_DEVICE);
	complete(&hw->done_rx_dma);
}

static int ingenic_sslv_dma_txrx(struct ingenic_sslv *hw, struct spi_transfer *t)
{
	int ret;
	struct dma_slave_config rx_config, tx_config;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;
	struct dma_chan *rxchan = hw->rxchan;
	struct dma_chan *txchan = hw->txchan;
	u32 entries = 0;
	int dma_ds[] = {64, 32, 16, 4, 2, 1};
	unsigned int dma_unit = 0;
	int i;

	hw->len = t->len;
	/* Check that the channels are available */
	if (!hw->txchan || !hw->rxchan) {
		dev_err(NULL, "no dma channel\n");
		return -ENODEV;
	}

	if (t->len % hw->transfer_unit_size) {
		pr_err("The length of tranfer data is error\n");
		return -EFAULT;
	}

	// config controller
	disable_all_interrupts(hw);

	switch (hw->transfer_unit_size) {
		case SPI_8BITS:
			tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			tx_config.dst_maxburst = 1;
			tx_config.src_maxburst = 1;
			rx_config.src_maxburst = 1;
			rx_config.dst_maxburst = 1;
			break;
		case SPI_16BITS:
			tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
			tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
			rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
			rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
			tx_config.dst_maxburst = 1;
			tx_config.src_maxburst = 1;
			rx_config.src_maxburst = 1;
			rx_config.dst_maxburst = 1;
			break;
		case SPI_32BITS:
			tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			tx_config.dst_maxburst = 1;
			tx_config.src_maxburst = 1;
			rx_config.src_maxburst = 1;
			rx_config.dst_maxburst = 1;
			break;
	}

	tx_config.dst_addr = (dma_addr_t)(hw->phys + SSLV_DR);
	rx_config.src_addr = (dma_addr_t)(hw->phys + SSLV_DR);
	tx_config.direction = DMA_MEM_TO_DEV;
	rx_config.direction = DMA_DEV_TO_MEM;
	tx_config.slave_id = 0;
	rx_config.slave_id = 0;

	dmaengine_slave_config(txchan, &tx_config);
	dmaengine_slave_config(rxchan, &rx_config);

	hw->tx_trigger = 1;
	hw->rx_trigger = 1;
	set_tx_trigger(hw, hw->tx_trigger);
	set_rx_trigger(hw, hw->rx_trigger);
	/* set tx dma trigger */
	set_txdma_threshold(hw, hw->tx_trigger);
	/* set rx dma trigger */
	set_rxdma_threshold(hw, hw->rx_trigger);

	hw->tx = hw->txbuffer;
	hw->rx = hw->rxbuffer;

	print_dbg("t->len: %d, tx fifo width: %d, set tx trigger value to %d\n", t->len, hw->txfifo_width, hw->tx_trigger);

	//set_rx_trigger(hw, 1); //the rx trigger is steady for tranfer
	print_dbg("t->len: %d, rx fifo width: %d, set rx trigger value to %d\n", t->len, hw->rxfifo_width, hw->rx_trigger);

	hw->rw_mode = 0;
	/* all transfer starts with tx, ends with rx. */
	if(t->tx_buf){
		hw->rw_mode |= W_MODE;
	}
	if(t->rx_buf){
		hw->rw_mode |= R_MODE;
	}

	/* config tx dma  */
	sg_init_one(hw->sg_tx, hw->tx, t->len);
	if (dma_map_sg(hw->txchan->device->dev,
				hw->sg_tx, 1, DMA_TO_DEVICE) != 1) {
		dev_err(NULL, "dma_map_sg tx error\n");
		printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
		goto err_tx_sgmap;
	}

	hw->sg_tx->dma_address &= 0xfffffff;

	txdesc = dmaengine_prep_slave_sg(txchan,
			hw->sg_tx,
			1,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc) {
		dev_err(NULL, "device_prep_slave_sg error\n");
		printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
		goto err_txdesc;
	}

	txdesc->callback = dma_tx_callback;
	txdesc->callback_param = hw;

	/* config rx dma  */
	sg_init_one(hw->sg_rx, hw->rx, t->len);
	if (dma_map_sg(hw->rxchan->device->dev,
				hw->sg_rx, 1, DMA_FROM_DEVICE) != 1) {
		dev_err(NULL, "dma_map_sg rx error\n");
		goto err_rx_sgmap;
	}

	hw->sg_rx->dma_address &= 0xfffffff;

	rxdesc = dmaengine_prep_slave_sg(rxchan,
			hw->sg_rx,
			1,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc) {
		dev_err(NULL, "device_prep_slave_sg error\n");
		goto err_rxdesc;
	}

	rxdesc->callback = dma_rx_callback;
	rxdesc->callback_param = hw;

	enable_rxfifo_overflow_intr(hw);
	enable_txfifo_overflow_intr(hw);

	enable_tx_dmamode(hw);
	enable_rx_dmamode(hw);

	dmaengine_submit(txdesc);
	dmaengine_submit(rxdesc);

	start_transmit(hw);

	dma_async_issue_pending(txchan);
	dma_async_issue_pending(rxchan);

	/* start transfer */
	if(hw->rw_mode & W_MODE)

	ret = wait_for_completion_interruptible_timeout(&hw->done_tx_dma, 240 * HZ);
	if (ret <= 0) {
		printk("The tx_dma umap wait timeout\n");
		goto err_txdesc;
	}

	ret = wait_for_completion_interruptible_timeout(&hw->done_rx_dma, 240 * HZ);
	if (ret <= 0) {
		printk("The spi dam_callback wait timeout\n");
		goto err_rxdesc;
	}

	/* dump_spi_slaver_regs(hw); */
	if(hw->rw_mode & W_MODE)

	finish_transmit(hw);
	disable_all_interrupts(hw);
	disable_tx_dmamode(hw);
	disable_rx_dmamode(hw);

	return t->len;

err_rxdesc:
err_txdesc:
	dma_unmap_sg(rxchan->device->dev, hw->sg_rx, 1, DMA_FROM_DEVICE);
err_rx_sgmap:
	dma_unmap_sg(txchan->device->dev, hw->sg_tx, 1, DMA_TO_DEVICE);
err_tx_sgmap:
	printk("<< dma_txrx error. out of memory >>\n");
	finish_transmit(hw);
	return -ENOMEM;
}

static irqreturn_t ingenic_sslv_dma_irq_callback(struct ingenic_sslv *hw)
{
	unsigned int status;
	unsigned int irq_status;

	status = ss_read(hw, SLV_SR);
	irq_status = ss_read(hw, SLV_ISR);
	clear_all_interrupt(hw);

	if(status & SR_TXE){
		printk("%s[%d]: transfer error!\n",__func__,__LINE__);
	}

	/* rxfifo overflow */
	if(status & SR_RFF){
		printk("%s[%d]: rxfifo overflow!\n",__func__,__LINE__);
	}

	/* rxfifo is empty */
	if(!(status & SR_RFNE)){
		printk("%s[%d]: rxfifo is empty!\n",__func__,__LINE__);
	}

	/* txfifo is empty */
	if(status & SR_TFE){
		printk("%s[%d]: txfifo is empty!\n",__func__,__LINE__);
	}

	/* txfifo is full */
	if(!(status & SR_TFNF)){
		printk("%s[%d]: txfifo is full!\n",__func__,__LINE__);
	}

	/* rxfifo more than threshold */
	if(irq_status & RXFIM){
		printk("%s[%d]:dma error rxfifo more than threshold!\n",__func__,__LINE__);
	}

	/* txfifo less than threshold */
	if(irq_status & ISR_TXEIS){
		printk("%s[%d]:dma error txfifo less than threshold!\n",__func__,__LINE__);
	}

	return IRQ_HANDLED;
}

/* return value that is reveiced counter */
static inline u32 cpu_read_rxfifo(struct ingenic_sslv *hw, unsigned int cnt)
{
	int i = 0;
	u32 dat;

//	print_dbg("The count of RxFIFO is %d \n", get_rxfifo_count(hw));

	if ((hw->rw_mode & RW_MODE) == W_MODE) {
		print_dbg("W_MODE\n");
	}

	for(i = 0; i < cnt; i++){
		dat = hw->get_rx(hw);
	}

	return cnt;
}

/* return value that is transfered counter */
static inline u32 cpu_write_txfifo(struct ingenic_sslv *hw, u32 entries)
{
	u32 i = 0;
	u32 dat;

	if ((!entries ) || (!(hw->rw_mode & RW_MODE)))
		return 0;

	if (hw->rw_mode & W_MODE) {
		for (i = 0; i < entries; i++) {
			dat = (u32)(hw->get_tx(hw));
		}
	}

	return entries;
}


static void inline cale_rxfifo_trigger(struct ingenic_sslv *hw, unsigned int entries)
{
	if(entries <= INGENIC_SSI_MAX_FIFO_ENTRIES / 2){
		hw->rx_trigger = entries;
	}else{
		hw->rx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES / 2;
	}
}

static void inline cale_txfifo_trigger(struct ingenic_sslv *hw, unsigned int entries)
{
	if(entries <= INGENIC_SSI_MAX_FIFO_ENTRIES / 2){
		hw->tx_trigger = entries;
	}else{
		hw->tx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES / 2;
	}
}

static void wait_read(struct ingenic_sslv *hw)
{
	int i;
	u32 thr;
	/* u32 read_num = hw->len; */
	while(hw->rentries > 0)
	{
		if(hw->rentries < hw->rx_trigger){
			set_rx_trigger(hw, hw->rentries);
		}
		while((ss_read(hw, SLV_RISR) & ISR_RXFIR) == 0);
		thr = read_rxfifo_entries(hw);
		cpu_read_rxfifo(hw, thr);
		hw->rentries -= thr;
	}
}
static void wait_write(struct ingenic_sslv *hw)
{
	int i;
	u32 thr;
	u32 write_num = hw->len;
	while(hw->wentries > 0)
	{
		while((ss_read(hw,SLV_RISR) & ISR_TXEIR) == 0);
		thr = read_txfifo_entries(hw);
		thr = hw->wentries > (INGENIC_SSI_MAX_FIFO_ENTRIES-thr) ? (INGENIC_SSI_MAX_FIFO_ENTRIES-thr) : hw->wentries;
		cpu_write_txfifo(hw, thr);
		hw->wentries -= thr;
	}
}

static int ingenic_sslv_pio_txrx(struct ingenic_sslv *hw, struct spi_transfer *t)
{
	struct ingenic_intr_cnt *g_ingenic_intr = hw->g_ingenic_intr;
	u32 entries = 0, unit_size = 0, send_entries = 0;
	unsigned long flags;
	int retlen = 0;
	hw->len = t->len;
	hw->wentries = 0;
	hw->rentries = 0;
	hw->dma_flag &= ~SPI_DMA_ACK;

	unit_size = hw->transfer_unit_size;
	if (unit_size == SPI_8BITS)
		entries = hw->len;
	else if (unit_size == SPI_16BITS )
		entries = hw->len >> 1;
	else if (unit_size == SPI_32BITS )
		entries = hw->len >> 2;
	else {
		dev_err(hw->dev,"transfer_unit_size error!\n");
		return -1;
	}
	/* set default trigger  */
	hw->wentries = hw->rentries = entries;
	hw->rx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES / 2;
	hw->tx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES / 2;

	/* config mode and trigger */
	hw->rw_mode = 0;
	if(t->tx_buf){
		hw->rw_mode |= W_MODE;
		hw->tx = hw->txbuffer;
		cale_txfifo_trigger(hw, entries);
	}

	if(t->rx_buf){
		hw->rw_mode |= R_MODE;
		hw->rx = hw->rxbuffer;
		cale_rxfifo_trigger(hw, entries);
	}

	print_dbg("tx trigger = %d, rx trigger = %d\n",hw->tx_trigger, hw->rx_trigger);

	set_tx_trigger(hw, hw->tx_trigger);
	set_rx_trigger(hw, hw->rx_trigger);

	/* wait for sslv idle */
	disable_all_interrupts(hw);
	clear_all_interrupt(hw);
	memset(g_ingenic_intr, 0, sizeof(struct ingenic_intr_cnt));

#ifdef INTERRUPT_TEST
	/* This start SSI transfer, write data or 0 to txFIFO.
	 * irq is locked to protect SSI config registers */
	spin_lock_irqsave(&hw->txrx_lock, flags);

	/* enable interrupts */
	if(hw->rw_mode & R_MODE){
		enable_rxfifo_overflow_intr(hw);
		enable_rxfifo_threshold_intr(hw);
	}
	if(hw->rw_mode & W_MODE){
		enable_txfifo_overflow_intr(hw);
		enable_txfifo_threshold_intr(hw);
	}
	/* start transfer */
	start_transmit(hw);

	spin_unlock_irqrestore(&hw->txrx_lock, flags);

	/* wait the interrupt finish the transfer( one spi_transfer be sent ) */
	if(hw->rw_mode & R_MODE){
		int cnt = 6;
		wait_for_completion_interruptible(&hw->done_rx);
	}else{
		unsigned int status;
		wait_for_completion_interruptible(&hw->done);
		status = ss_read(hw, SLV_SR);
		while(!(status & SR_TFE)){
			udelay(100000);
			status = ss_read(hw, SLV_SR);
		}
	}

#else
	spin_lock_irqsave(&hw->txrx_lock, flags);

	/* start transfer */
	start_transmit(hw);

	spin_unlock_irqrestore(&hw->txrx_lock, flags);
	if(hw->rw_mode & R_MODE){
		printk("%s[%d]:wait read\n",__func__,__LINE__);
		wait_read(hw);
	}else{
		printk("%s[%d]: wait write\n",__func__,__LINE__);
		wait_write(hw);
	}
#endif
	finish_transmit(hw);
	disable_all_interrupts(hw);

	retlen = hw->len;
	if(hw->rw_mode & R_MODE){
		if(hw->rentries){
			dev_info(hw->dev,"The received count isn't enough, the lost count is %d!\n", hw->rentries);
			retlen = -1;
		}
	}else{
		if(hw->wentries){
			dev_info(hw->dev,"The transfer count isn't enough, the lost count is %d!\n", hw->wentries);
			retlen = -1;
		}
	}

	return retlen;
}

static irqreturn_t ingenic_sslv_pio_irq_callback(struct ingenic_sslv *hw)
{
	u32 cnt;
	unsigned int status;
	unsigned int irq_status;
	int entries = 0;

	status = ss_read(hw, SLV_SR);
	irq_status = ss_read(hw, SLV_ISR);
	clear_all_interrupt(hw);
#if 0
	if(status & SR_TXE){
		printk("%s[%d]: transfer error!\n",__func__,__LINE__);
	}

	/* rxfifo overflow */
	if(status & SR_RFF){
		printk("%s[%d]: rxfifo overflow!\n",__func__,__LINE__);
	}

	/* rxfifo is empty */
	if(!(status & SR_RFNE)){
		printk("%s[%d]: rxfifo is empty!\n",__func__,__LINE__);
	}

	/* txfifo is empty */
	if(status & SR_TFE){
		printk("%s[%d]: txfifo is empty!\n",__func__,__LINE__);
	}

	/* txfifo is full */
	if(!(status & SR_TFNF)){
		printk("%s[%d]: txfifo is full!\n",__func__,__LINE__);
	}
#endif

	/* rxfifo more than threshold */
	if(irq_status & ISR_RXFIS){
		cnt = read_rxfifo_entries(hw);
		entries = hw->rentries > cnt ? cnt : hw->rentries;
		cnt = cpu_read_rxfifo(hw, entries);
		if(hw->rentries){
			hw->rentries -= cnt;
		}
		if((hw->rentries > 0) && (hw->rentries < hw->rx_trigger)){
			cale_rxfifo_trigger(hw, hw->rentries);
			set_rx_trigger(hw, hw->rx_trigger);
		}else if(hw->rentries <= 0){
			disable_rxfifo_threshold_intr(hw);
			disable_rxfifo_overflow_intr(hw);
			if(hw->rw_mode & R_MODE){
			complete(&hw->done_rx);
			}
		}
	}
	/* txfifo less than threshold */
	if(irq_status & ISR_TXEIS){
		cnt = read_txfifo_entries(hw);
		entries = hw->wentries > INGENIC_SSI_MAX_FIFO_ENTRIES - cnt ?
			INGENIC_SSI_MAX_FIFO_ENTRIES - cnt : hw->wentries ;
		cpu_write_txfifo(hw, entries);
		if(hw->wentries){
			hw->wentries -= entries;
		}
		if((hw->wentries > 0)&&(hw->wentries < hw->tx_trigger)){
			cale_txfifo_trigger(hw, hw->wentries);
			set_tx_trigger(hw, hw->tx_trigger);
		}
		else if(hw->wentries <= 0){
			disable_txfifo_threshold_intr(hw);
			disable_txfifo_overflow_intr(hw);
			wait_for_sslv_idle(hw);
			if(hw->rw_mode & W_MODE)
				complete(&hw->done);
		}
	}

	return IRQ_HANDLED;
}

/* every spi_transfer could call this routine to setup itself */
static int ingenic_sslv_setupxfer(struct ingenic_sslv *hw, struct spi_transfer *t)
{
	u8  bpw = 0, fifo_width = 0;

	if (t) {
		if(t->bits_per_word)
			bpw = t->bits_per_word;
	}

	if (bpw < 4 || bpw > 32) {
		printk("invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	if (hw->use_dma) {
		hw->txrx_bufs = ingenic_sslv_dma_txrx;
		hw->irq_callback = ingenic_sslv_dma_irq_callback;
	} else {
		hw->txrx_bufs = ingenic_sslv_pio_txrx;
		hw->irq_callback = ingenic_sslv_pio_irq_callback;
	}

	hw->bits_per_word = bpw;
	if (bpw <= 8) {
		hw->transfer_unit_size = SPI_8BITS;
		hw->get_rx = ingenic_sslv_rx_buf_u8;
		hw->get_tx = ingenic_sslv_tx_buf_u8;
		fifo_width = FIFO_W8;
	} else if (bpw <= 16) {
		hw->transfer_unit_size = SPI_16BITS;
		hw->get_rx = ingenic_sslv_rx_buf_u16;
		hw->get_tx = ingenic_sslv_tx_buf_u16;
		fifo_width = FIFO_W16;
	} else {
		hw->transfer_unit_size = SPI_32BITS;
		hw->get_rx = ingenic_sslv_rx_buf_u32;
		hw->get_tx = ingenic_sslv_tx_buf_u32;
		fifo_width = FIFO_W32;
	}

	hw->txfifo_width = fifo_width;
	hw->rxfifo_width = fifo_width;
	set_frame_length(hw, fifo_width);

	return 0;
}

static irqreturn_t ingenic_sslv_irq(int irq, void *dev)
{
	struct ingenic_sslv *hw = dev;
	if(hw->irq_callback)
		return hw->irq_callback(hw);
	else
		return IRQ_HANDLED;
}

int ingenic_sslv_init_setup(struct ingenic_sslv *hw)
{
#ifndef FPGA_TEST_SPI_SLV
	if (!clk_is_enabled(hw->clk_gate)){
		printk("%s[%d]: enable spi slave clk\n",__func__,__LINE__);
		clk_enable(hw->clk_gate);
	}
#endif
	/* disable the SSI controller */
	start_config_regs(hw);

	/* set default half_intr trigger */
	hw->tx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES/2;
	hw->rx_trigger = INGENIC_SSI_MAX_FIFO_ENTRIES/2;
	set_tx_trigger(hw, hw->tx_trigger);
	set_rx_trigger(hw, hw->rx_trigger);

	/**********************
	0:Motorola spi mode
	1:TI ssp mode
	2:NSM
	3:reserved
	**********************/
	set_default_format(hw,0);

	/**********************
	0:pol=0,pha=0
	1:pol=0,pha=1
	2:pol=1,pha=0
	3:pol=1,pha=1
	**********************/
	set_polcph_mode(hw,3);

	/**********************
	0:transfer & receive
	1:transfer only
	2:receive only
	3:reserved
	**********************/
	set_transfer_mode(hw, 0);

	/* First,mask the interrupt, while verify the status ? */
	disable_all_interrupts(hw);
	clear_all_interrupt(hw);

	return 0;
}

static int test_loop(struct ingenic_sslv *hw)
{
	int i = 0, flag = 1;
	int number = 32;
	int thr = 8;
	int cnt = 32;
	int read_num = 0, write_num = 0;
	unsigned char read_c[64], write_c[64];

	for(i = 0; i < 64; i++)
		write_c[i]=i;

	start_config_regs(hw);
	set_loop(hw, 1);
	set_tx_trigger(hw, 8);
	set_rx_trigger(hw, 32);
	set_frame_length(hw, FIFO_W8);
	set_transfer_mode(hw, 0);
	start_transmit(hw);

	printk("slv ready\n");
	while(number > 0)
	{
		/* write fifo */
		while((ss_read(hw, SLV_RISR) & ISR_TXEIR) == 0);
		cnt = number > cnt ? cnt : number;
		for(i = 0; i < cnt; i++)
		{
			transmit_data(hw, write_c[write_num]);
			printk("write data %d\n",write_c[write_num]);
			write_num++;
		}

		/* read fifo */
		while((ss_read(hw, SLV_RISR) & ISR_RXFIR) == 0);
		for(i = 0; i < cnt; i++)
		{
			read_c[read_num] = ss_read(hw, SSLV_DR);
			read_num++;
		}
		number -= cnt;
	}
	finish_transmit(hw);
	set_loop(hw, 0);

	printk("write number: %d ; reads number:%d\n", write_num, read_num);
	for(i = 0; i < write_num; i++){
		printk("read_c[%d] = %d\n", i, read_c[i]);
		if(read_c[i] != write_c[i]){
			flag = 0;
			printk("**********************************\n");
			printk(" slv loop test error!!!\n");
			printk(" read_c[%d] = %d; write_c[%d] = %d\n",i, read_c[i], i, write_c[i]);
			printk("**********************************\n");
		}
	}
	if(flag)
		printk("@@@@@@ SLV LOOP TEST OK  @@@@@@\n");
	return 0;
}

static int ingenic_sslv_open(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct ingenic_sslv *hw = container_of(dev, struct ingenic_sslv, mdev);
	int ret = 0;
	u32 value;

	/*select old (SOC T31) SSI_SLV control */
	value = ss_read(hw,0x0);
	if(value & (0x1<<31))
		printk("SSI_SLV CTRLR0 : %x\n",ss_read(hw,0x0));
	else
		ss_write(hw,0x0,0x1<<31);

	/* config registers*/
	ret = ingenic_sslv_init_setup(hw);

	return ret;
}

static int ingenic_sslv_release(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct ingenic_sslv *hw = container_of(dev, struct ingenic_sslv, mdev);
	ingenic_sslv_init_setup(hw);

	return 0;
}

static long ingenic_sslv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct ingenic_sslv *hw = container_of(dev, struct ingenic_sslv, mdev);
	struct spi_transfer t;
	long ret = 0;
	int cnt = 0;

	switch(cmd){
		case INGENIC_SPI_LOOP_OPS:
			test_loop(hw);
			ingenic_sslv_init_setup(hw);
			ret = 1;
			break;
		case INGENIC_SPI_INT_OPS:
			printk("No Support!!!\n");
			ret = 1;
			break;
		case INGENIC_SPI_CPU_OPS:
			hw->use_dma = 0;
			ret = 0;
			break;
		case INGENIC_SPI_DMA_OPS:
			hw->use_dma = 1;
			ret = 0;
			break;
		default:
			printk("The cmd is invalid\n");
			ret = -EINVAL;
			break;
	}
	if(ret)
		goto out;

	if(arg){
		if ((cnt = copy_from_user(&t, (void __user *)arg, sizeof(t)))) {
			printk("cnt:	%d\n", cnt);
			printk("[%s][%d] copy from user error\n",
					__func__, __LINE__);
			return -EFAULT;
		}
	}

	if(t.len <= 0 || t.len > BUFFER_SIZE){
		ret = -EINVAL;
		printk("%s[%d]:The length(%d) is invalid\n",__func__,__LINE__, t.len);
		goto out;
	}
	print_dbg("siziof(t) = %d, cs_change = %d, bits_per_word = %d\n",sizeof(t), t.cs_change, t.bits_per_word);

	memset(hw->txbuffer, 0, BUFFER_SIZE);
	memset(hw->rxbuffer, 0, BUFFER_SIZE);

	if(t.tx_buf){
		if (cnt = copy_from_user(hw->txbuffer, (void __user *)t.tx_buf,
					t.len)) {
			printk("cnt:	%d\n", cnt);
			printk("[%s][%d] copy from user error\n",
					__func__, __LINE__);
			return -EFAULT;
		}
	}

	if((ret = ingenic_sslv_setupxfer(hw, &t))){
		printk("%s[%d]:Failed to setupxfer\n",__func__,__LINE__);
		goto out;
	}

	//tranfer or receive data
	ret = hw->txrx_bufs(hw, &t);
	if(ret != t.len){
		printk("%s[%d]: transfer %ld, len = %d\n",__func__,__LINE__, ret, t.len);
	}else{
		ret = 0;
	}

	if(t.rx_buf){
		if (copy_to_user((void __user *)t.rx_buf, hw->rxbuffer,
					t.len)) {
			dev_err(NULL, "[%s][%d] copy to user error\n",
					__func__, __LINE__);
			return -EFAULT;
		}
	}
out:
	return ret;
}


static struct file_operations ingenic_sslv_fops = {
	.open = ingenic_sslv_open,
	.release = ingenic_sslv_release,
	.unlocked_ioctl = ingenic_sslv_ioctl,
};

static bool spi_dma_chan_filter(struct dma_chan *chan, void *param)
{
	struct ingenic_sslv *hw = param;

	printk("chan->chan_id = %d, dma_type = %d\n", chan->chan_id, hw->dma_type);
	return hw->dma_type == (int)chan->private;
}

static int ingenic_sslv_probe(struct platform_device *pdev)
{
	struct ingenic_sslv *hw;
	struct resource *res;
	dma_cap_mask_t mask;
	int err = 0;

	hw = kzalloc(sizeof(struct ingenic_sslv), GFP_KERNEL);
	if(!hw){
		dev_err(&pdev->dev, "No memory for ingenic_sslv_slv\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	memset(hw, 0, sizeof(struct ingenic_sslv));

	hw->g_ingenic_intr = kzalloc(sizeof(struct ingenic_intr_cnt),GFP_KERNEL);
	if(hw->g_ingenic_intr == NULL)
	{
		dev_err(&pdev->dev, "No memory for ingenic_intr_cnt\n");
		err = -ENOMEM;
		goto err_intr;
	}

	hw->dev = &pdev->dev;

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, resource_size(res),
			pdev->name);
	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve iomem region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->phys = res->start;

	print_dbg("%s[%d]: res->start = 0x%08x\n",__func__,__LINE__, res->start);
	hw->iomem = ioremap(res->start, (res->end - res->start)+1);
	if (hw->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	hw->clk_gate = devm_clk_get(&pdev->dev, "gate_ssislv");
	clk_prepare_enable(hw->clk_gate);

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);
	init_completion(&hw->done_rx);
	init_completion(&hw->done_tx_dma);
	init_completion(&hw->done_rx_dma);
	spin_lock_init(&hw->lock);
	spin_lock_init(&hw->txrx_lock);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	hw->txchan = dma_request_chan(hw->dev, "tx");
	if(IS_ERR(hw->txchan)) {
		dev_err(&pdev->dev, "SPI request dma tx channel failed");
		goto err_no_irq;
	}

	hw->rxchan = dma_request_chan(hw->dev, "rx");
	if(IS_ERR(hw->rxchan)) {
		dev_err(&pdev->dev, "SPI request dma rx channel failed");
		goto free_txchan;
	}

	//alloc temp buffer for dma
	hw->txbuffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
			&hw->txbuffer_dma, GFP_KERNEL);
	if (!hw->txbuffer) {
		dev_err(&pdev->dev, "SPI request temp dma txbuffer failed");
		goto free_rxchan;
	}

	hw->rxbuffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
			&hw->rxbuffer_dma, GFP_KERNEL);
	if (!hw->rxbuffer) {
		dev_err(&pdev->dev, "SPI request temp dma rxbuffer failed");
		goto free_txbuffer;
	}

	hw->sg_tx = devm_kmalloc(&pdev->dev,sizeof(struct scatterlist), GFP_KERNEL);
	if (!hw->sg_tx) {
		dev_err(&pdev->dev, "Failed to alloc tx scatterlist\n");
		goto err_tx_sgmap;
	}

	hw->sg_rx = devm_kmalloc(&pdev->dev,sizeof(struct scatterlist), GFP_KERNEL);
	if(!hw->sg_rx) {
		dev_err(&pdev->dev, "Failed to alloc rx scatterlist\n");
		goto err_rx_sgmap;
	}

	/* request SSI irq */
	err = devm_request_irq(&pdev->dev, hw->irq, ingenic_sslv_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}


	hw->fifodepth = INGENIC_SSI_MAX_FIFO_ENTRIES;

	/* SSI controller initializations for SPI */

	hw->mdev.minor = MISC_DYNAMIC_MINOR;
	hw->mdev.name = "spi_slv";
	hw->mdev.fops = &ingenic_sslv_fops;
	err = misc_register(&hw->mdev);
	if (err < 0) {
		err = -ENOENT;
		dev_err(&pdev->dev, "misc_register failed\n");
		goto error_misc_register;
	}

	printk("INGENIC SSI slaver Controller ok!!\n");

	return 0;

error_misc_register:
	free_irq(hw->irq, hw);
free_rxchan:
	dma_release_channel(hw->rxchan);
free_txchan:
	dma_release_channel(hw->txchan);
err_no_irq:
	if (hw->sg_rx)
		kfree(hw->sg_rx);
err_rx_sgmap:
	if (hw->sg_tx)
		kfree(hw->sg_tx);
err_tx_sgmap:
	if (hw->rxbuffer)
		kfree(hw->rxbuffer);
free_txbuffer:
	if (hw->txbuffer)
		kfree(hw->txbuffer);
	iounmap(hw->iomem);

err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

err_no_iores:
	kfree(hw->g_ingenic_intr);
err_intr:
	kfree(hw);
err_nomem:
	return err;
}

static int ingenic_sslv_remove(struct platform_device *dev)
{
	struct ingenic_sslv *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	free_irq(hw->irq, hw);
	iounmap(hw->iomem);
	release_resource(hw->ioarea);
	kfree(hw->ioarea);
#ifndef FPGA_TEST_SPI_SLV
	if(clk_is_enabled(hw->clk_gate))
		clk_disable(hw->clk_gate);

	clk_put(hw->clk_gate);
#endif

	/* release DMA channel */
	if (hw->rxchan) {
		dma_release_channel(hw->rxchan);
		printk("dma_rx_chnl release\n");
	}
	if (hw->txchan) {
		dma_release_channel(hw->txchan);
		printk("dma_tx_chnl release\n");
	}

	kfree(hw->g_ingenic_intr);
	kfree(hw);
	printk(KERN_INFO "INGENIC SSI slaver Controller driver removed\n");

	return 0;
}

MODULE_ALIAS("ingenic_spi_slave");			/* for platform bus hotplug */
static const struct of_device_id ingenic_slv_match[] = {
        { .compatible = "ingenic,slv", },
        {},
};
MODULE_DEVICE_TABLE(of, ingenic_slv_match);
static struct platform_driver ingenic_sslvdrv = {
	.probe		= ingenic_sslv_probe,
	.remove		= ingenic_sslv_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ingenic.slv",
		.of_match_table = of_match_ptr(ingenic_slv_match),
	},
};

static int __init ingenic_sslv_init(void)
{
	return platform_driver_register(&ingenic_sslvdrv);
}

static void __exit ingenic_sslv_exit(void)
{
	platform_driver_unregister(&ingenic_sslvdrv);
	printk(KERN_INFO "INGENIC SSI Controller Module EXIT\n");

}

module_init(ingenic_sslv_init);
module_exit(ingenic_sslv_exit);

MODULE_DESCRIPTION("INGENIC SPI SLAVER Driver");
MODULE_LICENSE("GPL");
