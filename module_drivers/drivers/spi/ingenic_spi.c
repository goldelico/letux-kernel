/* linux/drivers/spi/ingenic_spi.c
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 * base-to: linux/drivers/spi/spi_bitbang.c
 *
 * Copyright (c) 2010 Ingenic
 * Author:Shumb <sbhuang@ingenic.cn>
 *
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
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "ingenic_spi.h"

/* #define SSI_DEGUG */
#ifdef SSI_DEGUG
#define  print_dbg(format,arg...)				\
	printk(format,## arg)
#else
#define  print_dbg(format,arg...)
#endif

#define INGENIC_SPI_RX_BUF(type)							\
	u32 ingenic_spi_rx_buf_##type(struct ingenic_spi *ingspi)	\
	{														\
		u32 data  = spi_readl(ingspi, SSI_DR);					\
		type * rx = (type *)ingspi->rx;							\
		*rx++ = (type)(data);								\
		ingspi->rx = (u8 *)rx;									\
		return (u32)data;									\
	}

#define INGENIC_SPI_TX_BUF(type)							\
	u32 ingenic_spi_tx_buf_##type(struct ingenic_spi *ingspi)	\
	{														\
		u32 data;											\
		const type * tx = (type *)ingspi->tx;					\
		data = *tx++;										\
		ingspi->tx = (u8 *)tx;									\
		transmit_data(ingspi, data);							\
		return (u32)data;									\
	}

INGENIC_SPI_RX_BUF(u8)
INGENIC_SPI_TX_BUF(u8)

INGENIC_SPI_RX_BUF(u16)
INGENIC_SPI_TX_BUF(u16)

INGENIC_SPI_RX_BUF(u32)
INGENIC_SPI_TX_BUF(u32)

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_3WIRE | SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_LOOP)
#define SPI_BITS_SUPPORT  (SPI_BITS_8 | SPI_BITS_16 | SPI_BITS_32)

static void ingenic_spi_cs(struct ingenic_spi_info *spi, u8 cs, unsigned int pol)
{
	u32 cs_gpio = spi->chipselects[cs];
	if(gpio_is_valid(cs_gpio)) {
		gpio_direction_output(cs_gpio, !pol ? 0 : 1);
	}
}

static void ingenic_spi_chipsel(struct spi_device *spi, int value)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

	switch (value) {
	case BITBANG_CS_INACTIVE:
		/* chip disable selected */
		if (ingspi->set_cs && ingspi->pdata)
			ingspi->set_cs(ingspi->pdata, spi->chip_select, cspol^1);
		break;
	case BITBANG_CS_ACTIVE:
		if (spi->mode & SPI_CPHA)
			set_spi_clock_phase(ingspi, 1);
		else
			set_spi_clock_phase(ingspi, 0);

		if (spi->mode & SPI_CPOL)
			set_spi_clock_polarity(ingspi, 1);
		else
			set_spi_clock_polarity(ingspi, 0);

		if (!(spi->mode & SPI_LSB_FIRST)) {
			set_tx_msb(ingspi);
			set_rx_msb(ingspi);
		} else {
			set_tx_lsb(ingspi);
			set_rx_lsb(ingspi);
		}

		if (spi->mode & SPI_LOOP)
			enable_loopback(ingspi);
		else
			disable_loopback(ingspi);

		/* chip enable selected */
		if (ingspi->set_cs && ingspi->pdata)
			ingspi->set_cs(ingspi->pdata, spi->chip_select, cspol);
		break;
	default:
		break;
	}
}

static void ingenic_spi_clk_enable(struct ingenic_spi *ingspi) {
	if(ingspi->clk_flag == 0)
		return;

	clk_prepare_enable(ingspi->clk_cgu);
	clk_prepare_enable(ingspi->clk_gate);
	ingspi->clk_flag = 0;
}

static void ingenic_spi_clk_disable(struct ingenic_spi *ingspi) {
	if(ingspi->clk_flag)
		return;

	clk_disable_unprepare(ingspi->clk_cgu);
	clk_disable_unprepare(ingspi->clk_gate);
	ingspi->clk_flag = 1;
}

static unsigned long ingenic_spi_clk_get_rate(struct spi_device *spi)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned long rate;
	u16 cgv;

	spin_lock(&ingspi->lock);
	rate = clk_get_rate(ingspi->clk_cgu);
	cgv = spi_readl(ingspi, SSI_GR);
	spin_unlock(&ingspi->lock);
	return (rate / (2 * (cgv + 1)));
}
static int ingenic_spi_clk_set_rate(struct spi_device *spi, unsigned long rate)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	int cgv;

#ifndef CONFIG_MACH_XBURST2
	{
		unsigned long cur_rate;
		unsigned long src_rate;

		cur_rate = ingenic_spi_clk_get_rate(spi);
		if(cur_rate == rate)
			return 0;
		spin_lock(&ingspi->lock);
		src_rate = clk_get_rate(ingspi->clk_cgu);
		cgv = (src_rate / (rate * 2)) - 1;
		if(cgv < 0) {
			printk("spi clk set %ld not support\n", rate);
			return -1;
		}
	}

#else
	spin_lock(&ingspi->lock);
	clk_set_rate(ingspi->clk_cgu, 2 * rate);
	cgv = 0;  /* SSI clock 2 division */
#endif
	spi_writel(ingspi, SSI_GR, cgv);
	spin_unlock(&ingspi->lock);
	return 0;
}

static void dma_tx_callback(void *data)
{
	struct ingenic_spi *ingspi = data;
	complete(&ingspi->done_tx_dma);
}

static void dma_rx_callback(void *data)
{
	struct ingenic_spi *ingspi = data;

	dma_unmap_sg(ingspi->txchan->device->dev, ingspi->sg_tx, 1, DMA_TO_DEVICE);
	complete(&ingspi->done_rx_dma);
}

static int ingenic_spi_dma_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	int ret;
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	struct dma_slave_config rx_config, tx_config;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;
	struct dma_chan *rxchan = ingspi->rxchan;
	struct dma_chan *txchan = ingspi->txchan;
	struct ingenic_intr_cnt *g_ingenic_intr;
	int dma_ds[] = {64, 32, 16, 4, 2, 1};
	int i;

	/* Check that the channels are available */
	if (!txchan || !rxchan) {
		dev_err(&spi->dev, "no dma channel\n");
		return -ENODEV;
	}

	if (t->len % ingspi->transfer_unit_size) {
		pr_err("The length of tranfer data is error\n");
		return -EFAULT;
	}

	ingspi->rw_mode = 0;
	if(t->tx_buf)
		ingspi->rw_mode |= W_MODE;
	if(t->rx_buf)
		ingspi->rw_mode |= R_MODE;

	/* all transfer starts with tx, ends with rx. */
	if (ingspi->rw_mode & W_MODE)
		ingspi->tx = t->tx_buf;
	else
		ingspi->tx = ingspi->buffer;

	if (ingspi->rw_mode & R_MODE)
		ingspi->rx = t->rx_buf;
	else
		ingspi->rx = ingspi->buffer;

	memset(ingspi->buffer, 0, BUFFER_SIZE);

	switch (ingspi->transfer_unit_size) {
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

	tx_config.dst_addr = (dma_addr_t)(ingspi->phys + SSI_DR);
	rx_config.src_addr = (dma_addr_t)(ingspi->phys + SSI_DR);

	tx_config.direction = DMA_MEM_TO_DEV;
	rx_config.direction = DMA_DEV_TO_MEM;

	tx_config.slave_id = 0;
	rx_config.slave_id = 0;

	dmaengine_slave_config(txchan, &tx_config);
	dmaengine_slave_config(rxchan, &rx_config);

	/* set tx dma trigger */
	for (i = 0; i < ARRAY_SIZE(dma_ds); i++) {
		if (t->len / dma_ds[i])
			break;
	}

	if (i < ARRAY_SIZE(dma_ds)) {
		ingspi->dma_tx_unit = dma_ds[i];
	} else {
		print_dbg("DMA tx block_size force to defaut set!!!");
		ingspi->dma_tx_unit = INGENIC_SSI_DMA_BURST_LENGTH;
	}

	ingspi->tx_trigger = ingspi->dma_tx_unit / (ingspi->txfifo_width >> 3);
	//set_tx_trigger(ingspi, ingspi->tx_trigger);
	set_tx_trigger(ingspi, 0); //The transfer is steady if the trigger number is used
	print_dbg("t->len: %d, tx fifo width: %d, set tx trigger value to %d\n", t->len, ingspi->txfifo_width, ingspi->tx_trigger);

	if (t->tx_buf) {
		txdesc = dmaengine_prep_slave_sg(
				txchan,
				t->tx_sg.sgl, t->tx_sg.nents,
				DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	} else {
		sg_init_one(ingspi->sg_tx, ingspi->buffer, t->len);

		if (dma_map_sg(ingspi->txchan->device->dev,
				ingspi->sg_tx, 1, DMA_TO_DEVICE) != 1) {
			dev_err(&spi->dev, "dma_map_sg tx error\n");
			printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
			goto err_tx_sgmap;
		}
		txdesc = dmaengine_prep_slave_sg(
					txchan,
					ingspi->sg_tx, 1,
					DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	}

	if (!txdesc) {
		dev_err(&spi->dev, "device_prep_slave_sg error\n");
		printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
		goto err_txdesc;
	}

	// config controller
	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	//revisit
	disable_tx_error_intr(ingspi);
	disable_rx_error_intr(ingspi);

	start_transmit(ingspi);
	//finish_transmit(ingspi);

	flush_fifo(ingspi);

	enable_receive(ingspi);
	clear_errors(ingspi);

	g_ingenic_intr = ingspi->g_ingenic_intr;
	memset(g_ingenic_intr, 0, sizeof *g_ingenic_intr);

	if (!(ingspi->rw_mode & R_MODE)) {
		txdesc->callback = dma_tx_callback;
		txdesc->callback_param = ingspi;
		enable_tx_error_intr(ingspi);

		dmaengine_submit(txdesc);
		dma_async_issue_pending(txchan);

		ret = wait_for_completion_interruptible_timeout(&ingspi->done_tx_dma, 60 * HZ);
		if (ret <= 0) {
			printk("The tx_dma umap wait timeout\n");
			goto err_txdesc;
		}
		ret = wait_for_completion_interruptible_timeout(&ingspi->done, 60 * HZ);
		if (ret <= 0) {
			printk("The spi transfer wait timeout\n");
			goto err_txdesc;
		}

		finish_transmit(ingspi);
		flush_rxfifo(ingspi);
		clear_errors(ingspi);

		return t->len;
	}
	/*  prepare spi dma rx */
	for (i = 0; i < ARRAY_SIZE(dma_ds); i++) {
		if (!(t->len % dma_ds[i]))
			break;
	}

	if (i < ARRAY_SIZE(dma_ds)) {
		ingspi->dma_rx_unit = dma_ds[i];
	} else {
		print_dbg("DMA rx block_size force to defaut set!!!");
		ingspi->dma_rx_unit = INGENIC_SSI_DMA_BURST_LENGTH;
	}

	ingspi->rx_trigger = ingspi->dma_rx_unit/(ingspi->rxfifo_width >> 3);
	//set_rx_trigger(ingspi, ingspi->rx_trigger);
	set_rx_trigger(ingspi, 0); //the rx trigger is steady for tranfer
	print_dbg("t->len: %d, rx fifo width: %d, set rx trigger value to %d\n", t->len, ingspi->rxfifo_width, ingspi->rx_trigger);

	if (t->rx_buf) {
		rxdesc = dmaengine_prep_slave_sg(
				rxchan,
				t->rx_sg.sgl, t->rx_sg.nents,
				DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	}

	if (!rxdesc) {
		dev_err(&spi->dev, "device_prep_slave_sg error\n");
		goto err_rxdesc;
	}

	txdesc->callback = NULL;
	txdesc->callback_param = NULL;

	rxdesc->callback = dma_rx_callback;
	rxdesc->callback_param = ingspi;
	enable_rx_error_intr(ingspi);
	enable_tx_error_intr(ingspi);

	dmaengine_submit(txdesc);
	dmaengine_submit(rxdesc);
	dma_async_issue_pending(rxchan);
	dma_async_issue_pending(txchan);

	ret = wait_for_completion_interruptible_timeout(&ingspi->done_rx, 60 * HZ);
	if (ret <= 0) {
		dump_spi_reg(ingspi);
		printk("The spi receiver wait timeout\n");
		goto err_rxdesc;
	}

	ret = wait_for_completion_interruptible_timeout(&ingspi->done_rx_dma, 60 * HZ);
	if (ret <= 0) {
		dump_spi_reg(ingspi);
		printk("The spi dam_callback wait timeout\n");
		goto err_rxdesc;
	}

	finish_transmit(ingspi);
	//flush_rxfifo(ingspi);
	clear_errors(ingspi);

	return t->len;

err_rxdesc:
err_txdesc:
	dma_unmap_sg(txchan->device->dev, ingspi->sg_tx, 1, DMA_TO_DEVICE);
err_tx_sgmap:
	printk("<< dma_txrx error. out of memory >>\n");
	return -ENOMEM;
}

static irqreturn_t ingenic_spi_dma_irq_callback(struct ingenic_spi *ingspi)
{
	struct ingenic_intr_cnt *g_ingenic_intr = ingspi->g_ingenic_intr;
	print_dbg("%s: status register: %08x\n", __func__, spi_readl(ingspi, SSI_SR));

	if (ssi_underrun(ingspi) && tx_error_intr(ingspi)) {
		print_dbg("UNDR:\n");

		g_ingenic_intr->ssi_eti++;
		disable_tx_error_intr(ingspi);

		clear_errors(ingspi);
		complete(&ingspi->done);
		complete(&ingspi->done_rx);

		goto irq_done;
	}

	if (ssi_overrun(ingspi) && rx_error_intr(ingspi)) {
			print_dbg(" overrun:\n");
			g_ingenic_intr->ssi_eri++;

			clear_errors(ingspi);
			complete(&ingspi->done);
			complete(&ingspi->done_rx);
	}

irq_done:
	return IRQ_HANDLED;
}

static inline u32 cpu_read_rxfifo(struct ingenic_spi *ingspi)
{
	u8 unit_size = ingspi->transfer_unit_size;
	u32 cnt, dat;
	int dummy_read = 0;

	print_dbg("The count of RxFIFO is %d \n", get_rxfifo_count(ingspi));
	if (get_rxfifo_count(ingspi) < 1)
		return 0;

	cnt = ingspi->rlen;
	if ((ingspi->rw_mode & RW_MODE) == W_MODE) {
		print_dbg("W_MODE\n");
		dummy_read = 1;
	}

	spin_lock(&ingspi->lock);

	while (!rxfifo_empty(ingspi)) {
		ingspi->rlen += unit_size;
		if (dummy_read){
			dat = spi_readl(ingspi, SSI_DR);
		} else {
			dat = ingspi->get_rx(ingspi);
			ingspi->r_count--;
		}
	}

	spin_unlock(&ingspi->lock);

	return (ingspi->rlen - cnt);
}

static inline u32 cpu_write_txfifo(struct ingenic_spi *ingspi, u32 entries)
{
	u8 unit_size = ingspi->transfer_unit_size;
	u32 i, cnt, count;
	u32 dat;

	if ((!entries ) || (!(ingspi->rw_mode & RW_MODE)))
		return 0;

	cnt = entries;
	count = cnt * unit_size;

	spin_lock(&ingspi->lock);
	if (ingspi->rw_mode & W_MODE) {
		for (i = 0; i < cnt; i++) {
			ingspi->count += unit_size;
			dat = (u32)(ingspi->get_tx(ingspi));
			ingspi->t_count--;
		}
	} else {		 /* read, fill txfifo with 0 */
		for (i = 0; i < cnt; i++) {
			ingspi->count += unit_size;
			transmit_data(ingspi, 0);
			ingspi->t_count--;
		}
	}
	spin_unlock(&ingspi->lock);

	return count;
}

static int ingenic_spi_cpu_transfer(struct ingenic_spi *ingspi)
{
	u32 retlen, w_entries;

	if (ingspi->t_count <= ingspi->fifodepth) {
		/* last transfer */
		w_entries = ingspi->t_count;
		ingspi->last_transfer = 1;

	} else {

		/* continue transfer */
		w_entries = ingspi->fifodepth - ingspi->tx_trigger;	/* 128 - tx_trigger */
		start_transmit(ingspi);
	}

	/* write fifo */
	retlen = cpu_write_txfifo(ingspi, w_entries);
	if (!retlen) {
		dev_info(ingspi->dev,"cpu_write_txfifo error!\n");
		return -1;
	}

	/* wait fifo empty to finish transfer */
	if(ingspi->last_transfer)
		finish_transmit(ingspi);

	/* enable interrupt */
	enable_tx_intr(ingspi);
	if (ingspi->rw_mode & R_MODE)
		enable_rx_intr(ingspi);

	print_dbg("+:(%d)\n", retlen);

#ifdef SSI_DEGUG
	dump_spi_reg(ingspi);
#endif
	return 0;
}

/*
 * Wait until SSI is idle, or timeout occurs.
 * Returns non-zero if error.
 */
static int ingenic_wait_transfer_busy(struct ingenic_spi *ingspi)
{
	unsigned long deadline;
	int timeout = 0;

	/* timeout period is 1s */
	deadline = jiffies + msecs_to_jiffies(1000);

	while (!timeout) {
		if (time_after_eq(jiffies, deadline))
			timeout = 1;

		if (!transfer_busy(ingspi))
			return 0;

		cond_resched();
	}

	return -ETIMEDOUT;
}

static int ingenic_spi_pio_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	u32 entries;
	int status;
	unsigned long flags;
	int is_timeout;

	ingspi->tx = t->tx_buf;
	ingspi->rx = t->rx_buf;
	ingspi->len = t->len;
	ingspi->count = 0;
	ingspi->r_count = 0;
	ingspi->t_count = 0;
	ingspi->rlen = 0;
	ingspi->rw_mode = 0;
	ingspi->last_transfer = 0;
	ingspi->dma_flag &= ~SPI_DMA_ACK;
	ingspi->tx_trigger = SSI_TX_FIFO_THRESHOLD * 8;
	ingspi->rx_trigger = SSI_RX_FIFO_THRESHOLD * 8;


	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	finish_transmit(ingspi);
	disable_receive_continue(ingspi);
	flush_fifo(ingspi);
	clear_errors(ingspi);

	/* deal entries */
	entries = (t->len << 3) >> (ffs(ingspi->txfifo_width) - 1);
	ingspi->t_count = entries;

	if(ingspi->tx)
		ingspi->rw_mode |= W_MODE;

	if(ingspi->rx) {
		ingspi->rw_mode |= R_MODE;
		ingspi->r_count = entries;
	}

	/* deal receive mode */
	if (ingspi->rw_mode & R_MODE) {
		/* enable receive */
		if (ingspi->r_count < ingspi->rx_trigger) {
			ingspi->rx_trigger = ingspi->r_count;
		}
		/* set rx trigger */
		set_rx_trigger(ingspi, ingspi->rx_trigger);
		enable_receive(ingspi);
	} else {
		/* disable receive */
		disable_receive(ingspi);
	}
	/* set rx trigger */
	set_tx_trigger(ingspi, ingspi->tx_trigger);


	/* This start SSI transfer, write data or 0 to txFIFO.
	 * irq is locked to protect SSI config registers */
	spin_lock_irqsave(&ingspi->txrx_lock, flags);
	status = ingenic_spi_cpu_transfer(ingspi);
	if (status < 0) {
		dev_err(ingspi->dev,"ERROR:spi_transfer error(%d)!\n", status);
		disable_tx_intr(ingspi);
		disable_rx_intr(ingspi);
		spin_unlock_irqrestore(&ingspi->txrx_lock, flags);

		return status;
	}
	spin_unlock_irqrestore(&ingspi->txrx_lock, flags);

	/* wait the interrupt finish the transfer( one spi_transfer be sent ) */
	wait_for_completion_interruptible(&ingspi->done);

	/* wait until SSI is idle */
	is_timeout = ingenic_wait_transfer_busy(ingspi);

	/* finish transfer and receive */
	if (ingspi->rw_mode & W_MODE)
		finish_transmit(ingspi);
	if (ingspi->rw_mode & R_MODE)
		disable_receive(ingspi);

	/* finish clear state */
	flush_fifo(ingspi);
	clear_errors(ingspi);

	/*
	 * in order to flush fifo and clear errors
	 * if wait until SSI is idle timeout.
	 */
	if (is_timeout == -ETIMEDOUT) {
		dev_err(ingspi->dev, "Wait until SSI is idle timeout!\n");
		return -ETIMEDOUT;
	}

	/* overrun error */
	if(ingspi->r_count)
		return -EIO;

	if ((ingspi->rw_mode & R_MODE) && (ingspi->rlen != t->len)) {
		dev_info(ingspi->dev, "Length error:ingspi->rlen=%d  t->len=%d\n", ingspi->rlen,t->len);

		if(ingspi->rlen > ingspi->len)
			ingspi->rlen = ingspi->len;
	} else if ((ingspi->rw_mode & RW_MODE) == W_MODE) {
		ingspi->rlen = ingspi->count;
	}

	return ingspi->rlen;
}

static irqreturn_t ingenic_spi_pio_irq_callback(struct ingenic_spi *ingspi)
{
	u32 cnt;
	int status;

	/* Write only or full duplex transmission done */
	if (!ingspi->t_count && !ingspi->r_count) {
		print_dbg("done:");
		goto done;
	}

	if ( ssi_underrun(ingspi) && tx_error_intr(ingspi) ) {

		print_dbg("TEI:(underrun):");
		clear_errors(ingspi);

		if (!ingspi->t_count && !ingspi->r_count)
			goto done;

	}

	if ( ssi_overrun(ingspi) && rx_error_intr(ingspi) ) {

		dev_err(ingspi->dev,"overrun error!\n");
		print_dbg("REI:(overrun):");
		goto done;
	}

	if ( rxfifo_half_full(ingspi) &&
		rxfifo_half_full_intr(ingspi)) {

		print_dbg("RXI:");
		cnt = cpu_read_rxfifo(ingspi);
		print_dbg("-:(%d)\n",cnt);

		if (!ingspi->t_count && !ingspi->r_count)
			goto done;

		if (ingspi->r_count < ingspi->rx_trigger) {
			ingspi->rx_trigger = ingspi->r_count;
			set_rx_trigger(ingspi, ingspi->rx_trigger);
		}
	}

	if ( txfifo_half_empty_intr(ingspi) &&
		txfifo_half_empty(ingspi)) {

		print_dbg("TXI:");

		if (!ingspi->t_count && !ingspi->r_count)
			goto done;

		if (ingspi->t_count) {
			status = ingenic_spi_cpu_transfer(ingspi);
			if (status < 0) {
				dev_err(ingspi->dev,"ingenic_spi_cpu_transfer error!\n");
				goto done;
			}
		} else {
			disable_tx_intr(ingspi);
		}
	}

	return IRQ_HANDLED;

done:
	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	// flush_fifo(ingspi);
	// clear_errors(ingspi);
	complete(&ingspi->done);

	return IRQ_HANDLED;
}

/* every spi_transfer could call this routine to setup itself */
static int ingenic_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	u8  bpw, fifo_width;
	u32 hz;
	int ret;

	bpw = spi->bits_per_word;
	hz  = spi->max_speed_hz;

	if (t) {
		if(t->bits_per_word)
			bpw = t->bits_per_word;
		if(t->speed_hz)
			hz = t->speed_hz;
	}

	if (bpw < 2 || bpw > 32) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	if (ingspi->use_dma) {
		ingspi->txrx_bufs = &ingenic_spi_dma_txrx;
		ingspi->irq_callback = &ingenic_spi_dma_irq_callback;
	} else {
		ingspi->txrx_bufs = &ingenic_spi_pio_txrx;
		ingspi->irq_callback = &ingenic_spi_pio_irq_callback;
	}

	ingspi->bits_per_word = bpw;
	if (bpw <= 8) {
		ingspi->transfer_unit_size = SPI_8BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u8;
		ingspi->get_tx = ingenic_spi_tx_buf_u8;
		fifo_width = FIFO_W8;
	} else if (bpw <= 16) {
		ingspi->transfer_unit_size = SPI_16BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u16;
		ingspi->get_tx = ingenic_spi_tx_buf_u16;
		fifo_width = FIFO_W16;
	} else {
		ingspi->transfer_unit_size = SPI_32BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u32;
		ingspi->get_tx = ingenic_spi_tx_buf_u32;
		fifo_width = FIFO_W32;
	}

	ingspi->txfifo_width = fifo_width;
	ingspi->rxfifo_width = fifo_width;
	set_frame_length(ingspi, fifo_width);

	if (spi->mode & SPI_LSB_FIRST) {
		set_tx_lsb(ingspi);
		set_rx_lsb(ingspi);
	} else {
		set_tx_msb(ingspi);
		set_rx_msb(ingspi);
	}

	if((ret = ingenic_spi_clk_set_rate(spi, hz)))
		return ret;

	dev_dbg(&spi->dev, "The real SPI CLK is %ld Hz\n", ingenic_spi_clk_get_rate(spi));

	mutex_lock(&ingspi->bitbang.lock);
	if (!ingspi->bitbang.busy) {
		ingspi->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	mutex_unlock(&ingspi->bitbang.lock);

	return 0;
}

static int ingenic_spi_setup(struct spi_device *spi)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&ingspi->lock, flags);
	if (ingspi->state & SUSPND) {
		spin_unlock_irqrestore(&ingspi->lock, flags);
		dev_err(&spi->dev,
			"setup: SPI-%d not active!\n", spi->master->bus_num);
		return -ESHUTDOWN;
	}
	spin_unlock_irqrestore(&ingspi->lock, flags);

	if (spi->chip_select >= spi->master->num_chipselect) {
		dev_err(&spi->dev, "cs%d >= max %d\n",
			spi->chip_select,
			spi->master->num_chipselect);
		return -EINVAL;
	}

	if (spi->chip_select == 0) {
		select_ce(ingspi);
	} else if (spi->chip_select == 1) {
		select_ce2(ingspi);
	} else
		return -EINVAL;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~MODEBITS) {
		dev_info(&spi->dev, "Warning: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}
	ingspi->spi_mode = spi->mode;

	if (spi->mode & SPI_LSB_FIRST) {
		set_tx_lsb(ingspi);
		set_rx_lsb(ingspi);
	} else {
		set_tx_msb(ingspi);
		set_rx_msb(ingspi);
	}

	if (spi->bits_per_word & ~SPI_BITS_SUPPORT) {
		dev_info(&spi->dev, "Warning: unsupported bits_per_word: %d\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	if (!spi->max_speed_hz) {
		return -EINVAL;
	}

	if (ingspi->max_clk < spi->max_speed_hz) {
		dev_info(&spi->dev, "Warning:invalid clock(%d Hz) be set to source clk(%d Hz)!\n",
				 spi->max_speed_hz,(uint)ingspi->max_clk);
		spi->max_speed_hz = ingspi->max_clk;
	}

	mutex_lock(&ingspi->bitbang.lock);
	if (!ingspi->bitbang.busy) {
		ingspi->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	mutex_unlock(&ingspi->bitbang.lock);

	return 0;
}

/**
 * ingenic_spi_txrx - functions which will handle transfer data
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function will put data to be transferred into data register
 * of SPI controller and then wait until the completion will be marked
 * by the IRQ Handler.
 */
static int ingenic_spi_txrx(struct spi_device * spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&ingspi->lock, flags);
	if (ingspi->state & SUSPND) {
		ingspi->state &= ~SPIBUSY;
		spin_unlock_irqrestore(&ingspi->lock, flags);
		printk("Now enter suspend, so cann't tranfer data\n");
		return -ESHUTDOWN;
	}
	ingspi->state |= SPIBUSY;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	ret = ingspi->txrx_bufs(spi, t);

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state &= ~SPIBUSY;
	spin_unlock_irqrestore(&ingspi->lock, flags);
	return ret;
}

static irqreturn_t ingenic_spi_irq(int irq, void *dev)
{
	struct ingenic_spi *ingspi = dev;

	return ingspi->irq_callback(ingspi);
}

static int ingenic_spi_init_setup(struct ingenic_spi *ingspi)
{
	ingspi->clk_flag = 1;
	ingenic_spi_clk_enable(ingspi);

	/* disable the SSI controller */
	ssi_disable(ingspi);

	/* set default half_intr trigger */
	ingspi->tx_trigger = SSI_TX_FIFO_THRESHOLD * 8;
	ingspi->rx_trigger = SSI_RX_FIFO_THRESHOLD * 8;
	set_tx_trigger(ingspi, ingspi->tx_trigger);
	set_rx_trigger(ingspi, ingspi->rx_trigger);

	/* First,mask the interrupt, while verify the status ? */
	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	disable_receive(ingspi);

	set_spi_clock_phase(ingspi, 0);
	set_spi_clock_polarity(ingspi, 0);
	set_tx_msb(ingspi);
	set_rx_msb(ingspi);

	set_spi_format(ingspi);
	set_frame_length(ingspi, 8);
	disable_loopback(ingspi);
	flush_fifo(ingspi);

	underrun_auto_clear(ingspi);
	clear_errors(ingspi);
	ssi_enable(ingspi);

	return 0;
}

#ifdef CONFIG_OF
static struct ingenic_spi_info *ingenic_spi_parse_dt(struct ingenic_spi *ingspi)
{
	struct ingenic_spi_info *isi;
	struct device *dev = ingspi->dev;
	unsigned int value;
	isi = devm_kzalloc(dev, sizeof(*isi), GFP_KERNEL);
	if (!isi)
		return ERR_PTR(-ENOMEM);

	if(of_property_read_u32(dev->of_node, "spi-max-frequency", &value)) {
		dev_warn(dev, "spi-max-frequency not specified\n");
		isi->max_clk = 0;
	} else {
		isi->max_clk = value;
	}

	if(of_property_read_u32(dev->of_node, "ingenic,has_dma_support", &value)) {
		dev_warn(dev, "spi-max-frequency not specified\n");
		ingspi->use_dma = 0;
	} else {
		ingspi->use_dma = value;
	}

	if (of_property_read_u32(dev->of_node, "ingenic,chnl", &value)) {
		dev_warn(dev, "ingenic,channel not specified\n");
		isi->chnl = 0;
	} else {
		isi->chnl = value;
	}

	if (of_property_read_u32(dev->of_node, "num-cs", &value)) {
		dev_warn(dev, "num_cs not specified\n");
		isi->num_chipselect = 0;
	} else {
		isi->num_chipselect = value;
	}

	if (of_property_read_u32(dev->of_node, "ingenic,spi-src-clk", &value)) {
		dev_warn(dev, "ingenic,spi-src-clk not specified\n");
		isi->src_clk = 0;
	} else {
		isi->src_clk = value;
	}

	if (of_property_read_u32(dev->of_node, "ingenic,allow_cs_same", &value)) {
		dev_warn(dev, "ingenic,allow_cs_same not specified\n");
		isi->allow_cs_same = 0;
	} else {
		isi->allow_cs_same = value;
	}


	if( (value = of_alias_get_id(dev->of_node, "spi")) < 0) {
		isi->bus_num = -1;
	} else {
		isi->bus_num = value;
	}

	{
		int i;
		for (i = 0; i < isi->num_chipselect; i++) {
			int cs_gpio = of_get_named_gpio(dev->of_node, "cs-gpios", i);

			if (cs_gpio == -EPROBE_DEFER) {
				break;
			}
			isi->chipselects[i] = cs_gpio;
			if (gpio_is_valid(cs_gpio)) {
				if (devm_gpio_request(dev, cs_gpio, "INGENIC_SPI_CS")) {
					if(!isi->allow_cs_same)
						dev_err(dev, "could not request %d gpio\n", cs_gpio);

				} else if (gpio_direction_output(cs_gpio, 1))
					dev_err(dev, "could not set gpio %d as output\n", cs_gpio);
			}
		}
	}

	return isi;
}

#else
static struct ingenic_spi_info *ingenic_spi_parse_dt(struct device *dev)
{
	return dev_get_platdata(dev);
}
#endif

static const struct ingenic_priv default_priv = {
	.clk_flag = SSI_CLK,
};

static const struct ingenic_priv cgu_priv = {
	.clk_flag = SFC_CLK,
};

static const struct of_device_id ingenic_spi_match[] = {

	{ .compatible = "ingenic,m300-spi", .data = (void *)&default_priv },
	{ .compatible = "ingenic,x1600-spi", .data = (void *)&default_priv },
	{ .compatible = "ingenic,x2000-spi", .data = (void *)&default_priv },
	{ .compatible = "ingenic,x2100-spi", .data = (void *)&default_priv },
	{ .compatible = "ingenic,x2500-spi", .data = (void *)&default_priv },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_spi_match);

static int ingenic_spi_clk_init(struct platform_device *pdev, struct ingenic_spi *ingspi)
{
	const struct of_device_id *match;
	struct clk *clk;
	char clkname[16];
	int err = 0;

	pdev->id = of_alias_get_id(pdev->dev.of_node, "spi");
	sprintf(clkname, "gate_ssi%d", pdev->id);
	ingspi->clk_gate = devm_clk_get(&pdev->dev, clkname);

	match = of_match_node(ingenic_spi_match, pdev->dev.of_node);
	ingspi->priv = match->data;

	if(ingspi->priv->clk_flag == SSI_CLK)
		ingspi->clk_cgu = devm_clk_get(&pdev->dev, "div_ssi");
	else
		ingspi->clk_cgu = devm_clk_get(&pdev->dev, "cgu_ssi");

	if (IS_ERR(ingspi->clk_cgu) || IS_ERR(ingspi->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get spi clock\n");
		err = PTR_ERR(ingspi->clk_cgu);
		return err;
	}

	if(!ingspi->pdata->src_clk) {
		/* Use external parent clock */
		clk = devm_clk_get(&pdev->dev, "ext");
		if(!clk) {
			dev_err(&pdev->dev, "Cannot get ext clock\n");
			return -1;
		}
	} else {
		/* Use SSI clock */
		if(ingspi->priv->clk_flag == SSI_CLK)
			return 0;

		/* Use SFC parent clock */
		clk = devm_clk_get(&pdev->dev, "cgu_sfc");
		if(!clk) {
			dev_err(&pdev->dev, "Cannot get cgu_sfc clock\n");
			return -1;
		}
	}

	err = clk_set_parent(ingspi->clk_cgu, clk);
	if(err < 0) {
		dev_err(&pdev->dev, "clk set  parent failed\n");
		return -1;
	}
	return 0;
}

static int ingenic_spi_configure_dma(struct ingenic_spi *ingspi)
{
	struct device *dev = ingspi->dev;
	dma_cap_mask_t mask;
	int err = 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	ingspi->txchan = dma_request_chan(dev, "tx");
	if (IS_ERR(ingspi->txchan)) {
		err = PTR_ERR(ingspi->txchan);
		if (err == -EPROBE_DEFER) {
			dev_warn(dev, "no DMA channel available at the moment\n");
			return err;
		}
		dev_err(dev, "DMA TX channel not available, SPI unable to use DMA\n");
		err = -EBUSY;
		goto error;
	}

	/*
	 * No reason to check EPROBE_DEFER here since we have already requested
	 * tx channel. If it fails here, it's for another reason.
	 */
	ingspi->rxchan = dma_request_chan(dev, "rx");

	if (!ingspi->rxchan) {
		dev_err(dev, "DMA RX channel not available, SPI unable to use DMA\n");
		err = -EBUSY;
		goto error;
	}

	//alloc temp buffer for dma
	ingspi->buffer = dma_alloc_coherent(dev, BUFFER_SIZE, &ingspi->buffer_dma, GFP_KERNEL);
	if (!ingspi->buffer) {
		dev_err(dev, "SPI request temp dma buffer failed");
		goto error;
	}

#if 0
	ingspi->buffer = kmalloc(BUFFER_SIZE, GFP_KERNEL);
	if (!ingspi->buffer) {
		dev_err(dev, "SPI request temp dma buffer failed");
		goto error;
	}
	print_dbg("<< ingspi->buffer addr:%p >>\n", ingspi->buffer);
#endif

	ingspi->sg_tx = devm_kmalloc(dev, sizeof(struct scatterlist), GFP_KERNEL);
	if (!ingspi->sg_tx) {
		dev_err(dev, "Failed to alloc tx scatterlist\n");
		goto error;
	}

	ingspi->sg_rx = devm_kmalloc(dev, sizeof(struct scatterlist), GFP_KERNEL);
	if(!ingspi->sg_rx) {
		dev_err(dev, "Failed to alloc rx scatterlist\n");
		goto error;
	}


	dev_info(dev, "Using %s (tx) and %s (rx) for DMA transfers\n",
			 dma_chan_name(ingspi->txchan),
			 dma_chan_name(ingspi->rxchan));
	return 0;
error:
	if (ingspi->rxchan)
		dma_release_channel(ingspi->rxchan);
	if (!IS_ERR(ingspi->txchan))
		dma_release_channel(ingspi->txchan);
	return err;
}

static bool ingenic_spi_can_dma(struct spi_master *master, struct spi_device *spi,
			 struct spi_transfer *transfer)
{
	return true;
}

static int ingenic_spi_probe(struct platform_device *pdev)
{
	struct ingenic_spi *ingspi;
	struct spi_master *master;
	struct device_node *np = pdev->dev.of_node;
	struct ingenic_spi_info *pdata = dev_get_platdata(&pdev->dev);
	struct resource	*res;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ingenic_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	/* the spi->mode bits understood by this drivers: */
	master->mode_bits = MODEBITS;

	ingspi = spi_master_get_devdata(master);
	ingspi->g_ingenic_intr = devm_kzalloc(&pdev->dev,
										  sizeof(struct ingenic_intr_cnt),GFP_KERNEL);
	if(!ingspi->g_ingenic_intr) {
		dev_err(&pdev->dev, "No memory for ingenic_intr_cnt\n");
		return -ENOMEM;
	}

	ingspi->master = spi_master_get(master);
	ingspi->dev = &pdev->dev;

	if (!pdata && np) {
		pdata = ingenic_spi_parse_dt(ingspi);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!pdata) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}

	ingspi->pdata = pdata;
	ingspi->chnl= ingspi->pdata->chnl;
	master->bus_num = (s16)ingspi->pdata->bus_num;
	if(master->bus_num != 0 && master->bus_num != 1){
		dev_err(&pdev->dev, "No this channel, bus_num= %d.\n", master->bus_num);
		err = -ENOENT;
		goto err_no_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}
	ingspi->phys = res->start;
	ingspi->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (!ingspi->iomem) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	ingspi->irq = platform_get_irq(pdev, 0);
	if (ingspi->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	ingenic_spi_clk_init(pdev, ingspi);

	ingspi->spi_clk = ingspi->pdata->src_clk;
	ingspi->max_clk = ingspi->pdata->max_clk;

	platform_set_drvdata(pdev, ingspi);
	init_completion(&ingspi->done);
	init_completion(&ingspi->done_rx);
	init_completion(&ingspi->done_tx_dma);
	init_completion(&ingspi->done_rx_dma);
	spin_lock_init(&ingspi->lock);
	spin_lock_init(&ingspi->txrx_lock);

	master->num_chipselect = ingspi->pdata->num_chipselect;
	master->dev.of_node = pdev->dev.of_node;

	/* setup the state for the bitbang driver */
	ingspi->bitbang.master         = ingspi->master;
	ingspi->bitbang.setup_transfer = ingenic_spi_setupxfer;
	ingspi->bitbang.chipselect     = ingenic_spi_chipsel;
	ingspi->bitbang.txrx_bufs      = ingenic_spi_txrx;
	ingspi->bitbang.master->setup  = ingenic_spi_setup;
	ingspi->fifodepth = INGENIC_SSI_MAX_FIFO_ENTRIES;
	ingspi->set_cs = &ingenic_spi_cs;

	ingenic_spi_init_setup(ingspi);

	if (ingspi->use_dma) {
		master->can_dma = ingenic_spi_can_dma;
		ingenic_spi_configure_dma(ingspi);
	}

	/* request SSI irq */
	err = devm_request_irq(&pdev->dev, ingspi->irq, ingenic_spi_irq, 0, pdev->name, ingspi);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_register;
	}

	dev_dbg(ingspi->dev, "bitbang at %p\n", &ingspi->bitbang);
	err = spi_bitbang_start(&ingspi->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master ERR_NO:%d\n",err);
		goto err_register;
	}

	printk(KERN_INFO "INGENIC SSI Controller for SPI channel %d driver register\n",ingspi->chnl);
	return 0;

err_register:
	free_irq(ingspi->irq, ingspi);
err_no_irq:
	if(ingspi->clk_gate)
		clk_put(ingspi->clk_gate);
	if(ingspi->clk_cgu)
		clk_put(ingspi->clk_cgu);
	iounmap(ingspi->iomem);
err_no_iomap:
	release_resource(ingspi->ioarea);
	kfree(ingspi->ioarea);
err_no_pdata:
	spi_master_put(ingspi->master);

	return err;
}

static int ingenic_spi_remove(struct platform_device *dev)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(dev);

	spi_master_put(ingspi->master);
	spi_bitbang_stop(&ingspi->bitbang);

	platform_set_drvdata(dev, NULL);

	free_irq(ingspi->irq, ingspi);
	iounmap(ingspi->iomem);

	ingenic_spi_clk_disable(ingspi);
	clk_put(ingspi->clk_gate);
	clk_put(ingspi->clk_cgu);

	release_resource(ingspi->ioarea);
	kfree(ingspi->ioarea);

	/* release DMA channel */
	if (ingspi->rxchan) {
		dma_release_channel(ingspi->rxchan);
	}
	if (ingspi->txchan) {
		dma_release_channel(ingspi->txchan);
	}

	/* release chipselect gpio */
	{
		int i;
		for (i = 0; i < ingspi->pdata->num_chipselect; i++)
			if(gpio_is_valid(ingspi->pdata->chipselects[i])) {
				gpio_free(ingspi->pdata->chipselects[i]);
			}
	}

	kfree(ingspi->g_ingenic_intr);
	kfree(ingspi);
	printk(KERN_INFO "INGENIC SSI Controller for SPI channel %d driver removed\n",ingspi->chnl);

	return 0;
}

#ifdef CONFIG_PM
static int ingenic_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state |= SUSPND;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	while (ingspi->state & SPIBUSY)
		printk("Now spi is busy, waitting!\n");

	ingenic_spi_clk_disable(ingspi);

	return 0;
}

static int ingenic_spi_resume(struct platform_device *pdev)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(pdev);
	unsigned long	flags;

	ingenic_spi_clk_enable(ingspi);

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state &= ~SUSPND;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	return 0;
}

#else
#define ingenic_spi_suspend NULL
#define ingenic_spi_resume  NULL
#endif

static struct platform_driver ingenic_spidrv = {
	.probe	    = ingenic_spi_probe,
	.remove		= ingenic_spi_remove,
	.suspend	= ingenic_spi_suspend,
	.resume		= ingenic_spi_resume,
	.driver		= {
		.name	= "ingenic-spi",
		.of_match_table	= ingenic_spi_match,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ingenic_spidrv);

MODULE_ALIAS("ingenic_spi");
MODULE_AUTHOR("Bo Liu <bo.liu@ingenic.com>");
MODULE_DESCRIPTION("INGENIC SPI controller driver");
MODULE_LICENSE("GPL");
