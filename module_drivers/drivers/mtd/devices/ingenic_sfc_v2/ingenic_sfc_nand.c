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
#include <linux/mtd/rawnand.h>
#include <linux/mtd/nand.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "sfc_flash.h"
#include "spinand.h"
#include "ingenic_sfc_common.h"
#include "./nand_device/nand_common.h"
#include "internals.h"
#include "ingenic_sfc_drv.h"


#ifdef CONFIG_INGENIC_SFCNAND_FMW
#include "fmw.h"

struct sfc_flash *fmw_flash = NULL;
EXPORT_SYMBOL_GPL(fmw_flash);
#endif

#define STATUS_SUSPND	(1<<0)
#define to_ingenic_spi_nand(mtd_info) container_of(mtd_info, struct sfc_flash, mtd)

/*
 * below is the informtion about nand
 * that user should modify according to nand spec
 * */

static LIST_HEAD(nand_list);

void dump_flash_info(struct sfc_flash *flash)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	struct ingenic_sfcnand_base_param *param = &nand_info->param;
	struct mtd_partition *partition = nand_info->partition.partition;
	uint8_t num_partition = nand_info->partition.num_partition;

	printk("id_manufactory = 0x%02x\n", nand_info->id_manufactory);
	printk("id_device = 0x%02x\n", nand_info->id_device);

	printk("pagesize = %d\n", param->pagesize);
	printk("blocksize = %d\n", param->blocksize);
	printk("oobsize = %d\n", param->oobsize);
	printk("flashsize = %d\n", param->flashsize);

	printk("tHOLD = %d\n", param->tHOLD);
	printk("tSETUP = %d\n", param->tSETUP);
	printk("tSHSL_R = %d\n", param->tSHSL_R);
	printk("tSHSL_W = %d\n", param->tSHSL_W);

	printk("ecc_max = %d\n", param->ecc_max);
	printk("need_quad = %d\n", param->need_quad);

	while(num_partition--) {
		printk("partition(%d) name=%s\n", num_partition, partition[num_partition].name);
		printk("partition(%d) size = 0x%llx\n", num_partition, partition[num_partition].size);
		printk("partition(%d) offset = 0x%llx\n", num_partition, partition[num_partition].offset);
		printk("partition(%d) mask_flags = 0x%x\n", num_partition, partition[num_partition].mask_flags);
	}
	return;
}

static int badblk_check(int len, unsigned char *buf)
{
	int  j;
	unsigned char *check_buf = buf;

	for(j = 0; j < len; j++){
		if(check_buf[j] != 0xff){
			return 1;
		}
	}
	return 0;
}

static int32_t sfcnand_do_read(struct sfc_flash *flash, struct flash_address *flash_address, u_char *buffer, size_t len)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	struct ingenic_sfcnand_ops *ops = nand_info->ops;
	struct sfc_cdt_xfer xfer;
	int32_t ret = 0;

	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if(nand_info->param.need_quad){
		xfer.cmd_index = NAND_QUAD_READ_TO_CACHE;
	}else{
		xfer.cmd_index = NAND_STANDARD_READ_TO_CACHE;
	}

	/* set addr */
	xfer.rowaddr = flash_address->pageaddr;

	if(nand_info->param.plane_select){
		xfer.columnaddr = CONVERT_COL_ADDR(flash_address->pageaddr, flash_address->columnaddr);
	}else{
		xfer.columnaddr = flash_address->columnaddr;
	}

	xfer.staaddr0 = SPINAND_ADDR_STATUS;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = flash_address->ops_mode;
	xfer.config.buf = buffer;

retry:
	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	/* if underrun, retry read */
	if (flash->sfc->retry_count > 0) {
		dev_warn(flash->dev,"SFC retry transfer! %s %s %d\n",__FILE__,__func__,__LINE__);
		goto retry;
	}

	/* get status to check nand ecc status */
	ret = ops->get_feature(flash, GET_ECC_STATUS);

	if(xfer.config.ops_mode == DMA_OPS) {
		dma_sync_single_for_device(flash->dev, (dma_addr_t)(sfc_get_paddr((void *)xfer.config.buf)), xfer.config.datalen, DMA_FROM_DEVICE);
	}

	return ret;
}

static int32_t sfcnand_do_write(struct sfc_flash *flash, u_char *buffer, struct flash_address *flash_address, size_t len)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	struct ingenic_sfcnand_ops *ops = nand_info->ops;
	struct sfc_cdt_xfer xfer;
	int32_t ret = 0;

	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if(nand_info->param.need_quad){
		xfer.cmd_index = NAND_QUAD_WRITE_ENABLE;
	}else{
		xfer.cmd_index = NAND_STANDARD_WRITE_ENABLE;
	}

	/* set addr */
	xfer.rowaddr = flash_address->pageaddr;

	if(nand_info->param.plane_select){
		xfer.columnaddr = CONVERT_COL_ADDR(flash_address->pageaddr, flash_address->columnaddr);
	}else{
		xfer.columnaddr = flash_address->columnaddr;
	}

	xfer.staaddr0 = SPINAND_ADDR_STATUS;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = flash_address->ops_mode;
	xfer.config.buf = buffer;

retry:
	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	/* if overrun, retry write */
	if (flash->sfc->retry_count > 0) {
		dev_warn(flash->dev,"SFC retry transfer! %s %s %d\n",__FILE__,__func__,__LINE__);
		goto retry;
	}

	/* get status to be sure nand write completed */
	ret = ops->get_feature(flash, GET_WRITE_STATUS);

	return  ret;
}

static int32_t sfcnand_do_erase(struct sfc_flash *flash, uint32_t pageaddr)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	struct ingenic_sfcnand_ops *ops = nand_info->ops;
	struct sfc_cdt_xfer xfer;
	int32_t ret = 0;

	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NAND_ERASE_WRITE_ENABLE;

	/* set addr */
	xfer.rowaddr = pageaddr;
	xfer.staaddr0 = SPINAND_ADDR_STATUS;

	/* set transfer config */
	xfer.dataen = DISABLE;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	/* get status to be sure nand write completed */
	ret = ops->get_feature(flash, GET_ERASE_STATUS);
	if(ret){
		dev_err(flash->dev, "Erase error, get state error ! %s %s %d \n",__FILE__,__func__,__LINE__);
	}

	return ret;
}


static int ingenic_sfcnand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct sfc_flash *flash = to_ingenic_spi_nand(mtd);
	uint32_t addr = (uint32_t)instr->addr;
	uint32_t end;
	int32_t ret;

	if(addr % mtd->erasesize) {
		dev_err(flash->dev, "ERROR:%s line %d eraseaddr no align\n", __func__,__LINE__);
		return -EINVAL;
	}
	end = addr + instr->len;
	mutex_lock(&flash->lock);
	while (addr < end) {
		if((ret = sfcnand_do_erase(flash, addr / mtd->writesize))) {
			dev_err(flash->dev, "spi nand erase error blk id  %d !\n",addr / mtd->erasesize);
			goto erase_exit;
		}
		addr += mtd->erasesize;
	}

erase_exit:
	mutex_unlock(&flash->lock);
	return ret;
}

static int ingenic_sfcnand_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct sfc_flash *flash = to_ingenic_spi_nand(mtd);
	uint32_t pagesize = mtd->writesize;
	uint32_t pageaddr;
	uint32_t columnaddr;
	uint32_t rlen;
	size_t align_len;
	u_char* pbuf;
	struct flash_address flash_address;
	int32_t ret = 0, reterr = 0, ret_eccvalue = 0;

	while(len) {
		pageaddr = (uint32_t)from / pagesize;
		columnaddr = (uint32_t)from % pagesize;
		rlen = min_t(uint32_t, len, pagesize - columnaddr);

		/* align length */
		align_len = ALIGN(rlen, 4);

		/* create DMA Descriptors */
		if (align_len != rlen) {
			align_len -= 4;
			pbuf = flash->sfc->tmp_buffer;
		} else {
			pbuf = buf;
		}

		if (align_len > 0) {
			flash_address.pageaddr = pageaddr;
			flash_address.columnaddr = columnaddr;
			flash_address.ops_mode = DMA_OPS;

			ret = create_sfc_desc(flash->sfc, pbuf, align_len);
			if(ret < 0){
				dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
				reterr = ret;
				goto read_exit;
			}

		/* DMA Descriptors read */
			ret = sfcnand_do_read(flash, &flash_address, buf, align_len);
			if(ret < 0) {
				dev_err(flash->dev, "%s %s %d: sfcnand_do_read error, ret = %d, \
						pageaddr = %u, columnaddr = %u, rlen = %u\n",
						__FILE__, __func__, __LINE__,
						ret, pageaddr, columnaddr, align_len);
				reterr = ret;
				if(ret == -EIO)
					break;
			} else if (ret > 0) {
				dev_dbg(flash->dev, "%s %s %d: sfcnand_do_read, ecc value = %d, \
						pageaddr = %u, columnaddr = %u, rlen = %u\n",
						__FILE__, __func__, __LINE__,
						ret, pageaddr, columnaddr, align_len);
				ret_eccvalue = ret;
			}
		}

		if (align_len != rlen) {

			if (align_len > 0) {
				memcpy(buf, pbuf, align_len);
			}

			flash_address.pageaddr = pageaddr;
			flash_address.columnaddr = columnaddr + align_len;
			flash_address.ops_mode = CPU_OPS;

			ret = sfcnand_do_read(flash, &flash_address, (u_char *)buf + align_len, rlen - align_len);
			if(ret < 0) {
				dev_err(flash->dev, "%s %s %d: sfcnand_do_read error, ret = %d, \
						pageaddr = %u, columnaddr = %u, rlen = %u\n",
						__FILE__, __func__, __LINE__,
						ret, flash_address.pageaddr, flash_address.columnaddr, rlen - align_len);
				reterr = ret;
				if(ret == -EIO)
					break;
			} else if (ret > 0) {
				dev_dbg(flash->dev, "%s %s %d: sfcnand_do_read, ecc value = %d, \
						pageaddr = %u, columnaddr = %u, rlen = %u\n",
						__FILE__, __func__, __LINE__,
						ret, flash_address.pageaddr, flash_address.columnaddr, rlen - align_len);
				ret_eccvalue = ret;
			}

		}

		len -= rlen;
		from += rlen;
		buf += rlen;
		*retlen += rlen;
	}

read_exit:
	return reterr ? reterr : (ret_eccvalue ? ret_eccvalue : ret);
}

static int ingenic_sfcnand_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	struct sfc_flash *flash = to_ingenic_spi_nand(mtd);
	uint32_t pagesize = mtd->writesize;
	uint32_t pageaddr;
	uint32_t columnaddr;
	uint32_t wlen;
	struct flash_address flash_address;
	int32_t ret;

	while(len) {
		pageaddr = (uint32_t)to / pagesize;
		columnaddr = (uint32_t)to % pagesize;
		wlen = min_t(uint32_t, pagesize - columnaddr, len);

		flash_address.pageaddr = pageaddr;
		flash_address.columnaddr = columnaddr;
		flash_address.ops_mode = DMA_OPS;

		/* create DMA Descriptors */
		ret = create_sfc_desc(flash->sfc, (unsigned char *)buf, wlen);
		if(ret < 0){
			dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
			goto write_exit;
		}

		/* DMA Descriptors write */
		if((ret = sfcnand_do_write(flash, (u_char *)buf, &flash_address, wlen))) {
			dev_err(flash->dev, "%s %s %d : spi nand write fail, ret = %d, \
				pageaddr = %u, columnaddr = %u, wlen = %u\n",
				__FILE__, __func__, __LINE__, ret,
				pageaddr, columnaddr, wlen);
			break;
		}
		*retlen += wlen;
		len -= wlen;
		to += wlen;
		buf += wlen;
	}

write_exit:
	return ret;
}

static int ingenic_sfcnand_write_oob(struct mtd_info *mtd, loff_t addr, struct mtd_oob_ops *ops)
{
	struct sfc_flash *flash = to_ingenic_spi_nand(mtd);
	uint32_t oob_addr = (uint32_t)addr;
	struct flash_address flash_address;
	int32_t ret;

	mutex_lock(&flash->lock);

	if(ops->datbuf) {
		ret = ingenic_sfcnand_write(mtd, addr, ops->len, &ops->retlen, ops->datbuf);
	}
	if (ops->oobbuf) {
		flash_address.pageaddr = oob_addr / mtd->writesize;
		flash_address.columnaddr =  mtd->writesize;
		flash_address.ops_mode = DMA_OPS;

		/* create DMA Descriptors */
		ret = create_sfc_desc(flash->sfc, (unsigned char *)ops->oobbuf, ops->ooblen);
		if(ret < 0){
			dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
			goto write_oob_exit;
		}

		if((ret = sfcnand_do_write(flash, ops->oobbuf, &flash_address, ops->ooblen))) {
			dev_err(flash->dev, "spi nand write oob error %s %s %d \n",__FILE__,__func__,__LINE__);
			goto write_oob_exit;
		}
		ops->oobretlen = ops->ooblen;
	}

write_oob_exit:
	mutex_unlock(&flash->lock);
	return ret;
}

static int32_t ingenic_sfcnand_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	struct sfc_flash *flash = to_ingenic_spi_nand(mtd);
	uint32_t addr = (uint32_t)from;
	int32_t ret = 0, ret_eccvalue = 0;
	struct flash_address flash_address;

	mutex_lock(&flash->lock);
	if(ops->datbuf) {
		ret = ingenic_sfcnand_read(mtd, from, ops->len, &ops->retlen, ops->datbuf);
	}

	if(ops->oobbuf) {
		flash_address.pageaddr = addr / mtd->writesize;
		flash_address.columnaddr = mtd->writesize + ops->ooboffs;
		flash_address.ops_mode = DMA_OPS;

		/* create DMA Descriptors */
		ret = create_sfc_desc(flash->sfc, (unsigned char *)ops->oobbuf, ops->ooblen);
		if(ret < 0){
			dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
			goto read_oob_exit;
		}

		ret = sfcnand_do_read(flash, &flash_address, ops->oobbuf, ops->ooblen);
		if(ret < 0)
			dev_err(flash->dev, "%s %s %d : spi nand read oob error ,ret= %d\n", __FILE__, __func__, __LINE__, ret);

		if(ret != -EIO)
			ops->oobretlen = ops->ooblen;

	}

read_oob_exit:
	mutex_unlock(&flash->lock);

	return ret ? ret : ret_eccvalue;
}

static int ingenic_sfcnand_chip_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t buf[2] = { 0, 0 };
	int  ret = 0, i = 0;
	int write_oob = !(chip->bbt_options & NAND_BBT_NO_OOB_BBM);

	/* Write bad block marker to OOB */
	if (write_oob) {
		struct mtd_oob_ops ops;
		loff_t wr_ofs = ofs;
		ops.datbuf = NULL;
		ops.oobbuf = buf;
		ops.ooboffs = chip->badblockpos;
		if (chip->options & NAND_BUSWIDTH_16) {
			ops.ooboffs &= ~0x01;
			ops.len = ops.ooblen = 2;
		} else {
			ops.len = ops.ooblen = 1;
		}
		ops.mode = MTD_OPS_PLACE_OOB;

		/* Write to first/last page(s) if necessary */
		if (chip->bbt_options & NAND_BBM_LASTPAGE)
			wr_ofs += mtd->erasesize - mtd->writesize;
		do {
			ret = ingenic_sfcnand_write_oob(mtd, wr_ofs, &ops);
			if (ret)
				return ret;
			wr_ofs += mtd->writesize;
			i++;
		} while ((chip->bbt_options & (NAND_BBM_FIRSTPAGE || NAND_BBM_SECONDPAGE)) && i < 2);
	}
	/* Update flash-based bad block table */
	if (chip->bbt_options & NAND_BBT_USE_FLASH) {
		ret = nand_markbad_bbt(chip, ofs);
	}

	return ret;
}

static int ingenic_sfcnand_block_bad_check(struct mtd_info *mtd, loff_t ofs)
{
	int check_len = 1;
	unsigned char check_buf[2] = {0x0};
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	struct mtd_oob_ops ops;

	memset(&ops, 0, sizeof(ops));
	if (chip->options & NAND_BUSWIDTH_16)
		check_len = 2;

	ops.oobbuf = check_buf;
	ops.ooblen = check_len;
	ingenic_sfcnand_read_oob(mtd, ofs, &ops);
	if(badblk_check(check_len, check_buf))
		return 1;
	return 0;
}

static int ingenic_sfcnand_block_isbab(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	if(!chip->bbt) {
		return ingenic_sfcnand_block_bad_check(mtd, ofs);
	}
	return nand_isbad_bbt(chip, ofs, 0);
}

static int ingenic_sfcnand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	int ret = ingenic_sfcnand_block_isbab(mtd, ofs);
	if(ret > 0) {
		/* If it was bad already, return success and do nothing */
			return 0;
	}
	return ingenic_sfcnand_chip_block_markbad(mtd, ofs);
}

static int ingenic_sfc_nand_set_feature(struct sfc_flash *flash, uint8_t addr, uint32_t val)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NAND_SET_FEATURE;

	/* set addr */
	xfer.staaddr0 = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 1;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = (uint8_t *)&val;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int ingenic_sfc_nand_get_feature(struct sfc_flash *flash, uint8_t addr, uint8_t *val)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NAND_GET_FEATURE;

	/* set addr */
	xfer.staaddr0 = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 1;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = (uint8_t *)val;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int32_t __init ingenic_sfc_nand_dev_init(struct sfc_flash *flash)
{
	int32_t ret;
	/*release protect*/
	uint8_t feature = 0;
	if((ret = ingenic_sfc_nand_set_feature(flash, SPINAND_ADDR_PROTECT, feature)))
		goto exit;

	if((ret = ingenic_sfc_nand_get_feature(flash, SPINAND_ADDR_FEATURE, &feature)))
		goto exit;

	feature |= (1 << 4) | (1 << 3) | (1 << 0);
	if((ret = ingenic_sfc_nand_set_feature(flash, SPINAND_ADDR_FEATURE, feature)))
		goto exit;

	return 0;
exit:
	return ret;
}

static int32_t __init ingenic_sfc_nand_try_id(struct sfc_flash *flash)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	struct ingenic_sfcnand_device *nand_device;
	struct sfc_cdt_xfer xfer;
	uint8_t id_buf[2] = {0};
	unsigned short index[2] = {NAND_TRY_ID, NAND_TRY_ID_DMY};
	uint8_t i = 0;
	struct device_id_struct *device_id = NULL;
	int32_t id_count = 0;

	for(i = 0; i < 2; i++){
		memset(&xfer, 0, sizeof(xfer));

		/* set index */
		xfer.cmd_index = index[i];

		/* set addr */
		xfer.rowaddr = 0;

		/* set transfer config */
		xfer.dataen = ENABLE;
		xfer.config.datalen = sizeof(id_buf);
		xfer.config.data_dir = GLB0_TRAN_DIR_READ;
		xfer.config.ops_mode = CPU_OPS;
		xfer.config.buf = id_buf;

		if(sfc_sync_cdt(flash->sfc, &xfer)) {
			dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
			return -EIO;
		}

		dev_info(flash->dev, "id_manufactory = %x, id_device %x\n", id_buf[0], id_buf[1]);
		list_for_each_entry(nand_device, &nand_list, list) {
			if(nand_device->id_manufactory == id_buf[0]) {
				device_id = nand_device->id_device_list;
				id_count = nand_device->id_device_count;
				while(id_count--) {
					if(device_id->id_device == id_buf[1]) {
						nand_info->id_manufactory = id_buf[0];
						nand_info->id_device = id_buf[1];
						nand_info->param = *device_id->param;
						goto found_param;
					}
					device_id++;
				}
			}
		}
	}

	if(!nand_info->id_manufactory && !nand_info->id_device) {
		dev_err(flash->dev, " ERROR!: don`t support this nand manufactory, please add nand driver.\n");
		return -ENODEV;
	}

found_param:
	dev_info(flash->dev, "Found Supported device, id_manufactory = 0x%02x, id_device = 0x%02x\n", nand_info->id_manufactory, nand_info->id_device);

	/* fill manufactory special operation and cdt params */
	nand_info->ops = &nand_device->ops;
	nand_info->cdt_params = nand_info->ops->get_cdt_params(flash, nand_info->id_device);

	if (!nand_info->ops->get_feature) {
		if (!nand_info->ops->deal_ecc_status) {
			dev_err(flash->dev,"ERROR:xxx_nand.c \"get_feature()\" and \"deal_ecc_status()\" not define.\n");
			return -ENODEV;
		} else {
			nand_info->ops->get_feature = nand_common_get_feature;
			printk("use nand common get feature interface!\n");
		}
	} else {
		printk("use nand private get feature interface!\n");
	}

	return 0;
}

static int32_t __init nand_partition_param_copy(struct sfc_flash *flash, struct ingenic_sfcnand_burner_param *burn_param) {
	struct ingenic_sfcnand_flashinfo *nand_info = flash->flash_info;
	int i = 0, count = 5, ret;
	size_t retlen = 0;

	/* partition param copy */
	nand_info->partition.num_partition = burn_param->partition_num;

	burn_param->partition = kzalloc(nand_info->partition.num_partition * sizeof(struct ingenic_sfcnand_partition), GFP_KERNEL);
	if (IS_ERR_OR_NULL(burn_param->partition)) {
	    	dev_err(flash->dev, "alloc partition space failed!\n");
			return -ENOMEM;
	}

	nand_info->partition.partition = kzalloc(nand_info->partition.num_partition * sizeof(struct mtd_partition), GFP_KERNEL);
	if (IS_ERR_OR_NULL(nand_info->partition.partition)) {
	    	dev_err(flash->dev, "alloc partition space failed!\n");
		kfree(burn_param->partition);
		return -ENOMEM;
	}

partition_retry_read:
	ret = ingenic_sfcnand_read(&flash->mtd, flash->param_offset + sizeof(*burn_param) - sizeof(burn_param->partition),
		nand_info->partition.num_partition * sizeof(struct ingenic_sfcnand_partition),
		&retlen, (u_char *)burn_param->partition);

	if((ret < 0) && count--)
		goto partition_retry_read;
	if(count < 0) {
		dev_err(flash->dev, "read nand partition failed!\n");
		kfree(burn_param->partition);
		kfree(nand_info->partition.partition);
		return -EIO;
	}

	for(i = 0; i < burn_param->partition_num; i++) {
		nand_info->partition.partition[i].name = burn_param->partition[i].name;
		nand_info->partition.partition[i].size = burn_param->partition[i].size;
		nand_info->partition.partition[i].offset = burn_param->partition[i].offset;
		nand_info->partition.partition[i].mask_flags = burn_param->partition[i].mask_flags;
	}
	return 0;
}

static struct ingenic_sfcnand_burner_param *burn_param;
static int32_t __init flash_part_from_chip(struct sfc_flash *flash) {

	int32_t ret = 0, retlen = 0, count = 5;

	burn_param = kzalloc(sizeof(struct ingenic_sfcnand_burner_param), GFP_KERNEL);
	if(IS_ERR_OR_NULL(burn_param)) {
		dev_err(flash->dev, "alloc burn_param space error!\n");
		return -ENOMEM;
	}

	count = 5;
param_retry_read:
	ret = ingenic_sfcnand_read(&flash->mtd, flash->param_offset,
		sizeof(struct ingenic_sfcnand_burner_param), &retlen, (u_char *)burn_param);
	if((ret < 0) && count--)
		goto param_retry_read;
	if(count < 0) {
		dev_err(flash->dev, "read nand base param failed!\n");
		ret = -EIO;
		goto failed;
	}

	if(burn_param->magic_num != SPINAND_MAGIC_NUM) {
		dev_info(flash->dev, "NOTICE: this flash haven`t param, magic_num:%x\n", burn_param->magic_num);
		ret = -EINVAL;
		goto failed;
	}

	if(nand_partition_param_copy(flash, burn_param)) {
		ret = -ENOMEM;
		goto failed;
	}

	return 0;
failed:
	kfree(burn_param);
	return ret;

}


static int32_t __init ingenic_sfcnand_partition(struct sfc_flash *flash) {
	int32_t ret = 0;
	if((ret = flash_part_from_chip(flash))) {
		dev_err(flash->dev, "read partition from flash failed!\n");
	}
	return ret;
}

int ingenic_sfcnand_register(struct ingenic_sfcnand_device *flash) {
	list_add_tail(&flash->list, &nand_list);
	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_sfcnand_register);

/*
 *MK_CMD(cdt, cmd, LINK, ADDRMODE, DATA_EN)
 *MK_ST(cdt, st, LINK, ADDRMODE, ADDR_WIDTH, POLL_EN, DATA_EN, TRAN_MODE)
 */
static void params_to_cdt(cdt_params_t *params, struct sfc_cdt *cdt)
{
	/* 6. nand standard read */
	MK_CMD(cdt[NAND_STANDARD_READ_TO_CACHE], params->r_to_cache, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NAND_STANDARD_READ_GET_FEATURE], params->oip, 1, STA_ADDR0, 1, ENABLE, DISABLE, TM_STD_SPI);
	MK_CMD(cdt[NAND_STANDARD_READ_FROM_CACHE], params->standard_r, 0, COL_ADDR, ENABLE);

	/* 7. nand quad read */
	MK_CMD(cdt[NAND_QUAD_READ_TO_CACHE], params->r_to_cache, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NAND_QUAD_READ_GET_FEATURE], params->oip, 1, STA_ADDR0, 1, ENABLE, DISABLE, TM_STD_SPI);
	MK_CMD(cdt[NAND_QUAD_READ_FROM_CACHE], params->quad_r, 0, COL_ADDR, ENABLE);

	/* 8. nand standard write */
	MK_CMD(cdt[NAND_STANDARD_WRITE_ENABLE], params->w_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NAND_STANDARD_WRITE_TO_CACHE], params->standard_w_cache, 1, COL_ADDR, ENABLE);
	MK_CMD(cdt[NAND_STANDARD_WRITE_EXEC], params->w_exec, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NAND_STANDARD_WRITE_GET_FEATURE], params->oip, 0, STA_ADDR0, 1, ENABLE, DISABLE, TM_STD_SPI);

	/* 9. nand quad write */
	MK_CMD(cdt[NAND_QUAD_WRITE_ENABLE], params->w_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NAND_QUAD_WRITE_TO_CACHE], params->quad_w_cache, 1, COL_ADDR, ENABLE);
	MK_CMD(cdt[NAND_QUAD_WRITE_EXEC], params->w_exec, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NAND_QUAD_WRITE_GET_FEATURE], params->oip, 0, STA_ADDR0, 1, ENABLE, DISABLE, TM_STD_SPI);

	/* 10. block erase */
	MK_CMD(cdt[NAND_ERASE_WRITE_ENABLE], params->w_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NAND_BLOCK_ERASE], params->b_erase, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NAND_ERASE_GET_FEATURE], params->oip, 0, STA_ADDR0, 1, ENABLE, DISABLE, TM_STD_SPI);

	/* 11. ecc status read */
	MK_CMD(cdt[NAND_ECC_STATUS_READ], params->ecc_r, 0, DEFAULT_ADDRMODE, ENABLE);

}

static void nand_create_cdt_table(struct sfc *sfc, void *flash_info, uint32_t flag)
{
	struct ingenic_sfcnand_flashinfo *nand_info = flash_info;
	cdt_params_t *cdt_params;
	struct sfc_cdt sfc_cdt[INDEX_MAX_NUM];

	memset(sfc_cdt, 0, sizeof(sfc_cdt));
	if(flag & DEFAULT_CDT){

		/* 1. reset */
		sfc_cdt[NAND_RESET].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NAND_RESET].xfer = CMD_XFER(0, DISABLE, 0, DISABLE, SPINAND_CMD_RESET);
		sfc_cdt[NAND_RESET].staExp = 0;
		sfc_cdt[NAND_RESET].staMsk = 0;

		/* 2. try id */
		sfc_cdt[NAND_TRY_ID].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NAND_TRY_ID].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINAND_CMD_RDID);
		sfc_cdt[NAND_TRY_ID].staExp = 0;
		sfc_cdt[NAND_TRY_ID].staMsk = 0;

		/* 3. try id with dummy */
		/*
		 * There are some NAND flash, try ID operation requires 8-bit dummy value to be all 0,
		 * so use 1 byte address instead of dummy here.
		 */
		sfc_cdt[NAND_TRY_ID_DMY].link = CMD_LINK(0, ROW_ADDR, TM_STD_SPI);
		sfc_cdt[NAND_TRY_ID_DMY].xfer = CMD_XFER(1, DISABLE, 0, ENABLE, SPINAND_CMD_RDID);
		sfc_cdt[NAND_TRY_ID_DMY].staExp = 0;
		sfc_cdt[NAND_TRY_ID_DMY].staMsk = 0;

		/* 4. set feature */
		sfc_cdt[NAND_SET_FEATURE].link = CMD_LINK(0, STA_ADDR0, TM_STD_SPI);
		sfc_cdt[NAND_SET_FEATURE].xfer = CMD_XFER(1, DISABLE, 0, ENABLE, SPINAND_CMD_SET_FEATURE);
		sfc_cdt[NAND_SET_FEATURE].staExp = 0;
		sfc_cdt[NAND_SET_FEATURE].staMsk = 0;

		/* 5. get feature */
		sfc_cdt[NAND_GET_FEATURE].link = CMD_LINK(0, STA_ADDR0, TM_STD_SPI);
		sfc_cdt[NAND_GET_FEATURE].xfer = CMD_XFER(1, DISABLE, 0, ENABLE, SPINAND_CMD_GET_FEATURE);
		sfc_cdt[NAND_GET_FEATURE].staExp = 0;
		sfc_cdt[NAND_GET_FEATURE].staMsk = 0;

		if (!(flag & UPDATE_CDT)){
			/* first create cdt table (default)*/
			write_cdt(sfc, sfc_cdt, NAND_RESET, NAND_GET_FEATURE);
			return;
		}
	}

	if(flag & UPDATE_CDT){
		cdt_params = nand_info->cdt_params;
		params_to_cdt(cdt_params, sfc_cdt);

		/* second create cdt table */
		if (!(flag & DEFAULT_CDT)) {
			/* second create cdt table (update)*/
			write_cdt(sfc, sfc_cdt, NAND_STANDARD_READ_TO_CACHE, NAND_ECC_STATUS_READ);
		} else {
			/* create full cdt table (default && update)*/
			write_cdt(sfc, sfc_cdt, NAND_RESET, NAND_ECC_STATUS_READ);
		}
	}
	//dump_cdt(sfc);
}


static int request_sfc_buffer(struct sfc_flash *flash)
{
	struct sfc *sfc = flash->sfc;
	sfc->tmp_buffer = (uint8_t *)dma_alloc_coherent(flash->dev,
			sizeof(uint8_t) * flash->mtd.writesize, &sfc->tbuff_pyaddr, GFP_KERNEL);
	if (IS_ERR_OR_NULL(sfc->tmp_buffer)) {
		return -ENOMEM;
	}

	return 0;
}

#ifdef CONFIG_INGENIC_SFCNAND_FMW

static int32_t read_fmw_config(struct mtd_info *mtd, 
		struct fmw_config *fmw_config, loff_t offset, size_t len, size_t part_size)
{
	loff_t flash_off = offset;
	int i, ret, retlen;

	for (i = 0; i < part_size / mtd->erasesize; i++) {
		ret = ingenic_sfcnand_read(mtd, offset, len, &retlen, (uint8_t *)fmw_config);
		if(ret >= 0 && fmw_config->data_len != 0 && fmw_config->crc_val != -1)
			break;

		flash_off += mtd->erasesize;
	}

	if(fmw_config->data_len == -1 ||
		fmw_config->crc_val == -1 ||
		fmw_config->data_len >= FMW_BUF_LEN) {
		printk("%s %s %d: flash don`t save mac value!\n",
			__FILE__, __func__, __LINE__);
		return -ENODATA;
	}

	if(i == part_size / mtd->erasesize) {
		printk("%s %s %d:flash all blocks ecc error!\n",
			__FILE__, __func__, __LINE__);
	    	return -EIO;
	}

	return 0;
}

static int32_t read_fmw(struct mtd_info *mtd, 
		uint8_t *buf, loff_t offset, size_t part_size)
{
	size_t config_len = sizeof(struct fmw_config);
	struct fmw_config fmw_config;
	int32_t i, retlen;
	int32_t ret = 0;

	ret = read_fmw_config(mtd, &fmw_config, offset, config_len, part_size);
	if(ret) {
		printk("%s %s %d:read sn config failed!\n",
			__FILE__, __func__, __LINE__);
		return -EIO;
	}

	offset += sizeof(struct fmw_config);

	for (i = 0; i < part_size / mtd->erasesize; i++) {
		ret = ingenic_sfcnand_read(mtd, offset, fmw_config.data_len, &retlen, buf);
		if(ret < 0) {
			offset += mtd->erasesize;
			continue;
		}

		if(local_crc32(0xffffffff, buf, fmw_config.data_len) == fmw_config.crc_val)
			break;
		offset += mtd->erasesize;
	}

	if(i == part_size / mtd->erasesize) {
		printk("%s %s %d: sn flash all blocks ecc error!\n",
			__FILE__, __func__, __LINE__);
	    	return -EIO;
	}

	return ret;
}

static int32_t buf_compare(const uint8_t *wbuf, uint8_t *rbuf, uint32_t len) {

	int32_t i = 0;
	for(i = 0; i < len; i++) {
		if(wbuf[i] != rbuf[i]) {
			printk("compare err:wbuf = 0x%02x, rbuf= 0x%02x\n",
				wbuf[i], rbuf[i]);
			return -EIO;
		}
	}
	return 0;
}

static int32_t write_fmw_to_flash(struct mtd_info *mtd, 
		const uint8_t *wbuf, loff_t offset, size_t len, size_t part_size)
{
	uint8_t *rbuf;
	int32_t retry_count = 5;
	int32_t ret, retlen;

w_retry:
	ret = ingenic_sfcnand_write(mtd, offset, len, &retlen, wbuf);
	if(ret < 0) {
		if(retry_count--)
			goto w_retry;
		if(retry_count < 0) {
			printk("%s %s %d:write flash failed! ret = %d\n",
					__FILE__, __func__, __LINE__, ret);
			return -EIO;
		}
	}

	retry_count = 5;
	retlen = 0;
	rbuf = kzalloc(len, GFP_KERNEL);
	if(!rbuf) {
		printk("alloc mem failed!\n");
		return -ENOMEM;
	}

r_retry:
	ret = ingenic_sfcnand_read(mtd, offset, len, &retlen, (uint8_t *)rbuf);
	if(ret < 0) {
	    if(retry_count--)
		    goto r_retry;
	    if(retry_count < 0) {
		    printk("%s %s %d:read flash failed! ret = %d\n",
		    __FILE__, __func__, __LINE__, ret);
		    goto failed;
	    }
	}

	if(buf_compare(wbuf, rbuf, len)) {
		printk("%s %s %d: buf compare err!\n",
			__FILE__, __func__, __LINE__);
		ret = -EIO;
		goto failed;
	}
	kfree(rbuf);
	return 0;

failed:
	kfree(rbuf);
	return ret;
}

static int32_t write_fmw(struct mtd_info *mtd, 
		const uint8_t *buf, loff_t offset, size_t len, size_t part_size)
{
	uint8_t i;
	uint8_t errcount = 0;
	int32_t ret = 0;

	struct erase_info instr = {
		.addr = offset,
		.len = mtd->erasesize,
	};

	for(i = 0; i < part_size / mtd->erasesize; i++) {
		ingenic_sfcnand_erase(mtd, &instr);
		instr.addr += instr.len;
	}

	for(i = 0; i < part_size / mtd->erasesize; i++) {
		ret = write_fmw_to_flash(mtd, buf, offset, len, CONFIG_SN_FLASH_SIZE / 2);
		if(ret) {
			printk("%s %s %d:write data failed! errcount = %d\n",
					__FILE__, __func__, __LINE__, errcount++);
		}
		offset += mtd->erasesize;
	}

	if(errcount == part_size / mtd->erasesize) {
		printk("all blk write failed!\n");
		return -EIO;
	}

	return ret;
}

static ssize_t nand_sn_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct mtd_info *mtd = &fmw_flash->mtd;
	uint8_t *rbuf;
	int32_t ret;
	size_t part_size = CONFIG_SN_FLASH_SIZE;
	loff_t offset = mtd->size + CONFIG_MAC_FLASH_SIZE;

	rbuf = kzalloc(FMW_BUF_LEN, GFP_KERNEL);
	if(!rbuf) {
		printk("%s %s %d: alloc fmw buf error.\n", __FILE__,__func__,__LINE__);
		return -ENOMEM;
	}

	ret = read_fmw(mtd, rbuf, offset, part_size);
	if(ret) {
		printk("%s %s %d: sn_read failed, ret = %d\n",
				__FILE__, __func__, __LINE__, ret);
		return ret;
	}

	ret = sprintf(buf, "%s\n", rbuf);

	kfree(rbuf);
	return ret;
}

static ssize_t nand_sn_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t n)
{
#if 0
	struct mtd_info *mtd = &fmw_flash->mtd;
	loff_t offset = mtd->size + CONFIG_MAC_FLASH_SIZE;
	int32_t ret;

	ret = write_fmw(mtd, buf, offset, n, CONFIG_SN_FLASH_SIZE);
	if (ret) {
		printk("sn firmware write failed!\n");
		ret = -EIO;
	}
#endif
	printk("The write firmware function is disabled.\n");

	return n;
}

static ssize_t nand_mac_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct mtd_info *mtd = &fmw_flash->mtd;
	uint8_t *rbuf;
	size_t part_size = CONFIG_MAC_FLASH_SIZE;
	loff_t offset = mtd->size;
	int32_t ret;

	rbuf = kzalloc(FMW_BUF_LEN, GFP_KERNEL);
	if(!rbuf) {
		printk("%s %s %d: alloc fmw buf error.\n", __FILE__,__func__,__LINE__);
		return -ENOMEM;
	}


	ret = read_fmw(mtd, rbuf, offset, part_size);
	if(ret) {
		printk("%s %s %d: sn_read failed, ret = %d\n",
				__FILE__, __func__, __LINE__, ret);
		return ret;
	}

	ret = sprintf(buf, "%s\n", rbuf);

	kfree(rbuf);
	return ret;
}

static ssize_t nand_mac_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t n)
{
#if 0
	struct mtd_info *mtd = &fmw_flash->mtd;
	loff_t offset = mtd->size;
	int32_t ret;

	ret = write_fmw(mtd, buf, offset, n, CONFIG_MAC_FLASH_SIZE);
	if (ret) {
		printk("sn firmware write failed!\n");
		ret = -EIO;
	}
#endif
	printk("The write firmware function is disabled.\n");

	return n;
}

static ssize_t nand_license_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mtd_info *mtd = &fmw_flash->mtd;
	uint8_t *rbuf;
	size_t part_size = CONFIG_LICENSE_FLASH_SIZE;
	loff_t offset = mtd->size + CONFIG_SN_FLASH_SIZE + CONFIG_MAC_FLASH_SIZE;
	int32_t ret;

	rbuf = kzalloc(FMW_BUF_LEN, GFP_KERNEL);
	if(!rbuf) {
		printk("%s %s %d: alloc fmw buf error.\n", __FILE__,__func__,__LINE__);
		return -ENOMEM;
	}

	ret = read_fmw(mtd, rbuf, offset, part_size);
	if(ret) {
		printk("%s %s %d: sn_read failed, ret = %d\n",
				__FILE__, __func__, __LINE__, ret);
		return ret;
	}

	ret = sprintf(buf, "%s\n", rbuf);

	kfree(rbuf);
	return ret;
}

static ssize_t nand_license_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
#if 0
	struct mtd_info *mtd = &fmw_flash->mtd;
	loff_t offset = mtd->size + CONFIG_MAC_FLASH_SIZE + CONFIG_SN_FLASH_SIZE;
	int32_t ret;

	ret = write_fmw(mtd, buf, offset, n, CONFIG_LICENSE_FLASH_SIZE);
	if (ret) {
		printk("sn firmware write failed!\n");
		ret = -EIO;
	}
#endif
	printk(KERN_ERR "The write firmware function is disabled.\n");

	return n;
}

static DEVICE_ATTR(nand_sn, S_IRUGO|S_IWUSR, nand_sn_show, nand_sn_store);
static DEVICE_ATTR(nand_mac, S_IRUGO|S_IWUSR, nand_mac_show, nand_mac_store);
static DEVICE_ATTR(nand_license, S_IRUGO|S_IWUSR, nand_license_show, nand_license_store);

static struct attribute *nand_fmw_attrs[] = {
	&dev_attr_nand_sn.attr,
	&dev_attr_nand_mac.attr,
	&dev_attr_nand_license.attr,
	NULL,
};

const char nand_group_name[] = "nand_fmw";
static struct attribute_group nand_fmw_attr_group = {
	.name	= nand_group_name,
	.attrs	= nand_fmw_attrs,
};
#endif



int ingenic_sfc_nand_probe(struct sfc_flash *flash)
{
	const char *ingenic_probe_types[] = {"cmdlinepart", "ofpart", NULL};
	struct nand_chip *chip;
	struct ingenic_sfcnand_flashinfo *nand_info;
	int32_t ret;

	chip = kzalloc(sizeof(struct nand_chip), GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip)) {
		return -ENOMEM;
	}
	nand_info = kzalloc(sizeof(struct ingenic_sfcnand_flashinfo), GFP_KERNEL);
	if(IS_ERR_OR_NULL(nand_info)) {
	    	kfree(chip);
		return -ENOMEM;
	}

	flash->flash_info = nand_info;

	if(flash->pdata_params->param_offset > 0) {
		flash->param_offset = flash->pdata_params->param_offset;
	} else {
		flash->param_offset = SPIFLASH_PARAMER_OFFSET;
	}

#define THOLD	5
#define TSETUP	5
#define TSHSL_R	    100
#define TSHSL_W	    100

	set_flash_timing(flash->sfc, THOLD, TSETUP, TSHSL_R, TSHSL_W);

	/* request DMA Descriptor space */
	ret = request_sfc_desc(flash->sfc, DESC_MAX_NUM);
	if(ret){
		dev_err(flash->dev, "Failure to request DMA descriptor space!\n");
		ret = -ENOMEM;
		goto free_base;
	}

	/* Try creating default CDT table */
	flash->create_cdt_table = nand_create_cdt_table;
	flash->create_cdt_table(flash->sfc, flash->flash_info ,DEFAULT_CDT);

	if((ret = ingenic_sfc_nand_dev_init(flash))) {
		dev_err(flash->dev, "nand device init failed!\n");
		goto free_dma_desc;
	}

	if((ret = ingenic_sfc_nand_try_id(flash))) {
		dev_err(flash->dev, "try device id failed\n");
		goto free_dma_desc;
	}

	/* Update to private CDT table */
	flash->create_cdt_table(flash->sfc, flash->flash_info, UPDATE_CDT);

	/* Update sfc rate */
	if((ret = sfc_clk_set_highspeed(flash->sfc))) {
		dev_err(flash->dev, "set sfc rate failed\n");
		goto free_dma_desc;
	}

	set_flash_timing(flash->sfc, nand_info->param.tHOLD,
			nand_info->param.tSETUP, nand_info->param.tSHSL_R, nand_info->param.tSHSL_W);
	flash->mtd.name = "sfc_nand";
	flash->mtd.owner = THIS_MODULE;
	flash->mtd.type = MTD_NANDFLASH;
	flash->mtd.flags |= MTD_CAP_NANDFLASH;
	flash->mtd.erasesize = nand_info->param.blocksize;
	flash->mtd.writesize = nand_info->param.pagesize;
#ifndef CONFIG_INGENIC_SFCNAND_FMW
	flash->mtd.size = nand_info->param.flashsize;
#else
	flash->mtd.size = nand_info->param.flashsize - CONFIG_SN_FLASH_SIZE - CONFIG_MAC_FLASH_SIZE - CONFIG_LICENSE_FLASH_SIZE;
#endif
	flash->mtd.oobsize = nand_info->param.oobsize;
	flash->mtd.writebufsize = flash->mtd.writesize;
	flash->mtd.bitflip_threshold = flash->mtd.ecc_strength = nand_info->param.ecc_max - 1;

	chip->badblockbits = 8;
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(flash->mtd.erasesize) - 1;
	chip->data_buf = kzalloc(nand_info->param.pagesize + nand_info->param.oobsize, GFP_KERNEL);
	if(IS_ERR_OR_NULL(chip->data_buf)) {
		dev_err(flash->dev, "alloc nand buffer->databuf failed\n");
		ret = -ENOMEM;
		kfree(chip->data_buf);
		goto free_dma_desc;
	}

	flash->chip = chip;
	flash->mtd.priv = chip;
	flash->mtd._erase = ingenic_sfcnand_erase;
	flash->mtd._read_oob = ingenic_sfcnand_read_oob;
	flash->mtd._write_oob = ingenic_sfcnand_write_oob;
	flash->mtd._block_isbad = ingenic_sfcnand_block_isbab;
	flash->mtd._block_markbad = ingenic_sfcnand_block_markbad;

	/* request Temporary buffer space */
	ret = request_sfc_buffer(flash);
	if(ret){
		dev_err(flash->dev, "Failure to request Temporary Buffer space!\n");
		ret = -ENOMEM;
		goto free_chip_buffers;
	}

	flash->dev->of_node = NULL;
	if (!flash->pdata_params->use_ofpart_info) {
		/* use burner partitions */
		if((ret = ingenic_sfcnand_partition(flash))) {
			if(ret == -EINVAL)
				return 0;
			dev_err(flash->dev, "read flash partition failed!\n");
			goto free_all;
		}
		ret = mtd_device_parse_register(&flash->mtd, ingenic_probe_types, NULL,
				nand_info->partition.partition, nand_info->partition.num_partition);
	} else {

		/* use ofpart partitions */
		flash->dev->of_node = of_get_child_by_name(flash->dev->of_node, "nandflash");
		if (flash->dev->of_node < 0) {
			dev_err(flash->dev, "Cannot get 'nandflash' node from dtb!\n");
			flash->dev->of_node = NULL;
		}
		ret = mtd_device_parse_register(&flash->mtd, ingenic_probe_types, &flash->ppdata, NULL, 0);
	}

	if (ret) {
		kfree(nand_info->partition.partition);
		if(!flash->pdata_params->use_ofpart_info) {
			kfree(burn_param->partition);
			kfree(burn_param);
		}
		ret = -ENODEV;
		goto free_all;
	}

#ifdef CONFIG_INGENIC_SFCNAND_FMW
	fmw_flash = flash;

	flash->attr_group = &nand_fmw_attr_group;
	ret = sysfs_create_group(&flash->dev->kobj, flash->attr_group);
	if (ret) {
		dev_err(flash->dev, "device create sysfs group failed\n");
		ret = -EINVAL;
		goto free_all;
	}
#endif

	return 0;

free_all:
	dma_free_coherent(flash->dev, flash->mtd.writesize, flash->sfc->tmp_buffer, flash->sfc->tbuff_pyaddr);

free_chip_buffers:
	kfree(chip->data_buf);

free_dma_desc:
	free_sfc_desc(flash->sfc);

free_base:
	kfree(chip);
	kfree(nand_info);
	return ret;
}

