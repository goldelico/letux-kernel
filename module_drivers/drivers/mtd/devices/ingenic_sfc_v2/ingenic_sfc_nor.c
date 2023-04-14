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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/page.h>
#include "sfc_flash.h"
#include "spinor.h"
#include "ingenic_sfc_common.h"
#include "ingenic_sfc_drv.h"

//#define DEBUG_CLONER_PARAMS

#define STATUS_SUSPND	(1<<0)

#define MULTI_DIE_FLASH_NUM 1

static LIST_HEAD(nor_list);

struct sfc_flash *to_ingenic_spi_norflash(struct mtd_info *mtd_info)
{
	return container_of(mtd_info, struct sfc_flash, mtd);
}

#define ACTIVE_DIE(flash, addr)					\
({								\
	uint8_t die_id = addr >> nor_info->die_shift;		\
	if (die_id != nor_info->current_die_id) {			\
		sfc_active_die(flash, die_id);				\
		nor_info->current_die_id = die_id;			\
	}							\
	if (die_id)						\
		addr = addr & ((1 << nor_info->die_shift) - 1);	\
	addr;							\
})								\

static int sfc_die_select(struct sfc_flash *flash, uint8_t die_id)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_DIE_SELECT;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 1;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = &die_id;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int sfc_read_active_die_id(struct sfc_flash *flash, uint8_t *value)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_READ_ACTIVE_DIE_ID;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 1;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = value;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int sfc_active_die(struct sfc_flash *flash, uint8_t die_id)
{
	uint8_t die_id_read;

	sfc_die_select(flash, die_id);
	do {
		sfc_read_active_die_id(flash, &die_id_read);
	}while(die_id != die_id_read);

	return 0;
}

int32_t sfc_nor_reset(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_RESET_ENABLE;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = DISABLE;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	udelay(100);
	return 0;
}

int sfc_nor_read_id(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	unsigned char buf[3];
	unsigned int chip_id = 0;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_READ_ID;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 3;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	chip_id = ((buf[0] & 0xff) << 16) | ((buf[1] & 0xff) << 8) | (buf[2] & 0xff);

	return chip_id;
}

static unsigned int sfc_do_read(struct sfc_flash *flash, unsigned int addr, unsigned char *buf, size_t len)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if (nor_info->quad_succeed) {
		xfer.cmd_index = NOR_READ_QUAD;
	} else {
		xfer.cmd_index = NOR_READ_STANDARD;
	}

	/* active die */
	if (nor_info->die_num > 1)
		addr = ACTIVE_DIE(flash, addr);

	/* set addr */
	xfer.columnaddr = 0;
	xfer.rowaddr = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = DMA_OPS;
	xfer.config.buf = buf;

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

	if(xfer.config.ops_mode == DMA_OPS) {
		dma_sync_single_for_device(flash->dev, (dma_addr_t)(sfc_get_paddr((void *)buf)), len, DMA_FROM_DEVICE);
	}

	return len;
}

static unsigned  int sfc_do_write(struct sfc_flash *flash, unsigned int addr, const unsigned char *buf, size_t len)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if (nor_info->quad_succeed) {
		xfer.cmd_index = NOR_WRITE_QUAD_ENABLE;
	} else {
		xfer.cmd_index = NOR_WRITE_STANDARD_ENABLE;
	}

	/* active die */
	if (nor_info->die_num > 1)
		addr = ACTIVE_DIE(flash, addr);

	/* set addr */
	xfer.columnaddr = 0;
	xfer.rowaddr = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = DMA_OPS;
	xfer.config.buf = (uint8_t *)buf;

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

	return len;
}

static int sfc_do_erase(struct sfc_flash *flash, uint32_t addr)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct sfc_cdt_xfer xfer;

	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_ERASE_WRITE_ENABLE;

	/* active die */
	if (nor_info->die_num > 1)
		addr = ACTIVE_DIE(flash, addr);

	/* set addr */
	xfer.rowaddr = addr;

	/* set transfer config */
	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int sfc_read(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{
	int32_t ret;

	/* create DMA Descriptors */
	ret = create_sfc_desc(flash->sfc, buf, len);
	if(ret < 0){
		dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
		return ret;
	}

	//dump_desc(flash->sfc, ret);

	/* DMA Descriptors read */
	ret = sfc_do_read(flash, (unsigned int)from, buf, len);

	return ret;
}

static int sfc_write(struct sfc_flash *flash, loff_t to, size_t len, const unsigned char *buf)
{
	int32_t ret;

	/* create DMA Descriptors */
	ret = create_sfc_desc(flash->sfc, (unsigned char *)buf, len);
	if(ret < 0){
		dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
		return ret;
	}

	//dump_desc(flash->sfc, ret);

	/* DMA Descriptors write */
	ret = sfc_do_write(flash, (unsigned int)to, buf, len);

	return ret;
}

static int ingenic_spi_norflash_read(struct mtd_info *mtd, loff_t from, size_t len,size_t *retlen, unsigned char *buf)
{
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);

	mutex_lock(&flash->lock);
	*retlen = sfc_read(flash, from, len, buf);
	mutex_unlock(&flash->lock);

	return 0;
}

static int ingenic_spi_norflash_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const unsigned char *buf)
{
	u32 page_offset, actual_len;
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	int ret;

	mutex_lock(&flash->lock);

	page_offset = to & (spi_nor_info->page_size - 1);
	/* do all the bytes fit onto one page? */
	if (page_offset + len <= spi_nor_info->page_size) {
		ret = sfc_write(flash, (unsigned int)to, len, buf);
		*retlen = ret;
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		actual_len = spi_nor_info->page_size - page_offset;
		ret = sfc_write(flash, (unsigned int)to, actual_len, buf);
		*retlen += ret;

		/* write everything in flash->page_size chunks */
		for (i = actual_len; i < len; i += mtd->writesize) {
			actual_len = len - i;
			if (actual_len >= mtd->writesize)
				actual_len = mtd->writesize;

			ret = sfc_write(flash, (unsigned int)to + i, actual_len, buf + i);
			*retlen += ret;
		}
	}
	mutex_unlock(&flash->lock);
	return 0;
}

static int ingenic_spi_norflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	uint32_t addr, end;
	int ret;

	mutex_lock(&flash->lock);

	addr = (instr->addr & (mtd->erasesize - 1));
	if (addr) {
		dev_err(flash->dev, "%s eraseaddr no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	end = (instr->len & (mtd->erasesize - 1));
	if (end) {
		dev_err(flash->dev,"%s erasesize no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;

	while (addr < end) {
		ret = sfc_do_erase(flash, addr);
		if (ret) {
			dev_err(flash->dev,"erase error !\n");
			mutex_unlock(&flash->lock);
			return ret;
		}
		addr += spi_nor_info->erase_size;
	}
	mutex_unlock(&flash->lock);

	return 0;
}

#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
static int32_t ingenic_spi_norflash_read_params(struct sfc_flash *flash, loff_t from, size_t len, uint8_t *buf)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	xfer.cmd_index = NOR_READ_STANDARD;

	xfer.columnaddr = 0;
	xfer.rowaddr = from;

	xfer.dataen =  ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}
#endif

#ifdef DEBUG_CLONER_PARAMS
static void dump_cloner_params(struct burner_params *params)
{
	struct spi_nor_info *spi_nor_info;

	spi_nor_info = &params->spi_nor_info;

	printk("name=%s\n", spi_nor_info->name);
	printk("id=0x%x\n", spi_nor_info->id);

	printk("read_standard->cmd=0x%x\n",		spi_nor_info->read_standard.cmd);
	printk("read_standard->dummy=0x%x\n",		spi_nor_info->read_standard.dummy_byte);
	printk("read_standard->addr_nbyte=0x%x\n",	spi_nor_info->read_standard.addr_nbyte);
	printk("read_standard->transfer_mode=0x%x\n",	spi_nor_info->read_standard.transfer_mode);

	printk("read_quad->cmd=0x%x\n",			spi_nor_info->read_quad.cmd);
	printk("read_quad->dummy=0x%x\n",		spi_nor_info->read_quad.dummy_byte);
	printk("read_quad->addr_nbyte=0x%x\n",		spi_nor_info->read_quad.addr_nbyte);
	printk("read_quad->transfer_mode=0x%x\n",	spi_nor_info->read_quad.transfer_mode);

	printk("write_standard->cmd=0x%x\n",		spi_nor_info->write_standard.cmd);
	printk("write_standard->dummy=0x%x\n",		spi_nor_info->write_standard.dummy_byte);
	printk("write_standard->addr_nbyte=0x%x\n",	spi_nor_info->write_standard.addr_nbyte);
	printk("write_standard->transfer_mode=0x%x\n",	spi_nor_info->write_standard.transfer_mode);

	printk("write_quad->cmd=0x%x\n",		spi_nor_info->write_quad.cmd);
	printk("write_quad->dummy=0x%x\n",		spi_nor_info->write_quad.dummy_byte);
	printk("write_quad->addr_nbyte=0x%x\n",		spi_nor_info->write_quad.addr_nbyte);
	printk("write_quad->transfer_mode=0x%x\n",	spi_nor_info->write_quad.transfer_mode);

	printk("sector_erase->cmd=0x%x\n",		spi_nor_info->sector_erase.cmd);
	printk("sector_erase->dummy=0x%x\n",		spi_nor_info->sector_erase.dummy_byte);
	printk("sector_erase->addr_nbyte=0x%x\n",	spi_nor_info->sector_erase.addr_nbyte);
	printk("sector_erase->transfer_mode=0x%x\n",	spi_nor_info->sector_erase.transfer_mode);

	printk("wr_en->cmd=0x%x\n",		spi_nor_info->wr_en.cmd);
	printk("wr_en->dummy=0x%x\n",		spi_nor_info->wr_en.dummy_byte);
	printk("wr_en->addr_nbyte=0x%x\n",	spi_nor_info->wr_en.addr_nbyte);
	printk("wr_en->transfer_mode=0x%x\n",	spi_nor_info->wr_en.transfer_mode);

	printk("en4byte->cmd=0x%x\n",		spi_nor_info->en4byte.cmd);
	printk("en4byte->dummy=0x%x\n",		spi_nor_info->en4byte.dummy_byte);
	printk("en4byte->addr_nbyte=0x%x\n",	spi_nor_info->en4byte.addr_nbyte);
	printk("en4byte->transfer_mode=0x%x\n",	spi_nor_info->en4byte.transfer_mode);

	printk("quad_set->cmd=0x%x\n",		spi_nor_info->quad_set.cmd);
	printk("quad_set->bit_shift=0x%x\n",		spi_nor_info->quad_set.bit_shift);
	printk("quad_set->mask=0x%x\n",		spi_nor_info->quad_set.mask);
	printk("quad_set->val=0x%x\n",		spi_nor_info->quad_set.val);
	printk("quad_set->len=0x%x\n",		spi_nor_info->quad_set.len);
	printk("quad_set->dummy=0x%x\n",	spi_nor_info->quad_set.dummy);

	printk("quad_get->cmd=0x%x\n",		spi_nor_info->quad_get.cmd);
	printk("quad_get->bit_shift=0x%x\n",		spi_nor_info->quad_get.bit_shift);
	printk("quad_get->mask=0x%x\n",		spi_nor_info->quad_get.mask);
	printk("quad_get->val=0x%x\n",		spi_nor_info->quad_get.val);
	printk("quad_get->len=0x%x\n",		spi_nor_info->quad_get.len);
	printk("quad_get->dummy=0x%x\n",	spi_nor_info->quad_get.dummy);

	printk("busy->cmd=0x%x\n",		spi_nor_info->busy.cmd);
	printk("busy->bit_shift=0x%x\n",		spi_nor_info->busy.bit_shift);
	printk("busy->mask=0x%x\n",		spi_nor_info->busy.mask);
	printk("busy->val=0x%x\n",		spi_nor_info->busy.val);
	printk("busy->len=0x%x\n",		spi_nor_info->busy.len);
	printk("busy->dummy=0x%x\n",		spi_nor_info->busy.dummy);

	printk("quad_ops_mode=%d\n",	spi_nor_info->quad_ops_mode);
	printk("addr_ops_mode=%d\n",	spi_nor_info->addr_ops_mode);

	printk("tCHSH=%d\n",	spi_nor_info->tCHSH);
	printk("tSLCH=%d\n",	spi_nor_info->tSLCH);
	printk("tSHSL_RD=%d\n", spi_nor_info->tSHSL_RD);
	printk("tSHSL_WR=%d\n", spi_nor_info->tSHSL_WR);

	printk("chip_size=%d\n",	spi_nor_info->chip_size);
	printk("page_size=%d\n",	spi_nor_info->page_size);
	printk("erase_size=%d\n",	spi_nor_info->erase_size);

	printk("chip_erase_cmd=0x%x\n",	spi_nor_info->chip_erase_cmd);
}
#endif

#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
static struct burner_params *burner_params = NULL;
#endif

static int ingenic_spi_norflash_get_params(struct sfc_flash *flash)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;

#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
	int32_t ret, err = 0;
#else
	int chip_id;
	struct builtin_params builtin_params;
	struct nor_params_node *nor_device;
#endif


#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
	dev_info(flash->dev, "Use burner params.\n");
	burner_params = kzalloc(sizeof(struct burner_params), GFP_KERNEL);
	if (!burner_params) {
		dev_err(flash->dev, "Failed to alloc mem for params\n");
		err = -ENOMEM;
		goto err_params;
	}

	ret = ingenic_spi_norflash_read_params(flash, SPIFLASH_PARAMER_OFFSET, sizeof(struct burner_params), (uint8_t *)burner_params);
	if (ret) {
		dev_err(flash->dev, "Failed to read params (burned by Burner)\n");
		err = -EINVAL;
		goto err_read_params;
	}
	//add crc check for params
	dev_info(flash->dev, "magic is 0x%x  version is 0x%x\n", burner_params->magic, burner_params->version);
	nor_info->nor_flash_info = NULL;
	if (burner_params->magic == NOR_MAGIC) {
		if (burner_params->version == NOR_VERSION) {
			nor_info->nor_flash_info = &burner_params->spi_nor_info;
			nor_info->norflash_partitions = &burner_params->norflash_partitions;
			nor_info->nor_pri_data = &burner_params->nor_pri_data;
		}
	}

#else

	dev_info(flash->dev, "Use builtin params.\n");
	chip_id = sfc_nor_read_id(flash);
	if (chip_id < 0) {
		dev_err(flash->dev, "Failed to read chip id !\n");
		return -EINVAL;
	}
	dev_info(flash->dev, "chip_id: 0x%x\n", chip_id);

	list_for_each_entry(nor_device, &nor_list, list) {
		if(nor_device->nor_device_info.id == chip_id) {
			builtin_params.spi_nor_info = &nor_device->nor_device_info;
			break;
		}
	}

	if (!builtin_params.spi_nor_info) {
		dev_err(flash->dev, "ERROR: do support this device, id = 0x%x\n", chip_id);
		return -ENODEV;
	}

	get_nor_builtin_params(flash, &builtin_params);

	dev_info(flash->dev, "magic is 0x%x  version is 0x%x\n", builtin_params.magic, builtin_params.version);
	nor_info->nor_flash_info = NULL;
	if (builtin_params.magic == NOR_MAGIC) {
		if (builtin_params.version == NOR_VERSION) {
			nor_info->nor_flash_info = builtin_params.spi_nor_info;
			nor_info->nor_pri_data = builtin_params.nor_pri_data;
		}
	}
#endif


#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
	if ((!nor_info->nor_flash_info) || (!nor_info->norflash_partitions)) {
		printk("WARNING : cannot get nor flash params or partitions !!!\n");
		err = -EINVAL;
		goto err_read_params;
	}

#ifdef DEBUG_CLONER_PARAMS
	dump_cloner_params(burner_params);
#endif
	return 0;

err_read_params:
	kfree(burner_params);
err_params:
	return err;
#else

	if (!nor_info->nor_flash_info) {
		printk("WARNING : cannot get nor flash params !!!\n");
		return -EINVAL;
	}

	return 0;
#endif
}
static ssize_t sfc_nor_partition_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n", SPIFLASH_PARAMER_OFFSET + sizeof(int) * 2 + sizeof(struct spi_nor_info));
}

static DEVICE_ATTR(sfc_nor_partition_offset, S_IRUGO | S_IWUSR,
		sfc_nor_partition_offset_show,
		NULL);

static ssize_t sfc_nor_params_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n",SPIFLASH_PARAMER_OFFSET);
}

static DEVICE_ATTR(sfc_nor_params_offset, S_IRUGO | S_IWUSR,
		sfc_nor_params_offset_show,
		NULL);

/*add your attr in here*/
static struct attribute *sfc_norflash_info_attributes[] = {
	&dev_attr_sfc_nor_partition_offset.attr,
	&dev_attr_sfc_nor_params_offset.attr,
	NULL
};

static struct attribute_group sfc_norflash_info_attr_group = {
	.attrs = sfc_norflash_info_attributes
};

int ingenic_sfcnor_register(struct nor_params_node *flash) {
	list_add_tail(&flash->list, &nor_list);
	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_sfcnor_register);

/*
 *MK_CMD(cdt, cmd, LINK, ADDRMODE, DATA_EN)
 *MK_ST(cdt, st, LINK, ADDRMODE, ADDR_WIDTH, POLL_EN, DATA_EN, TRAN_MODE)
 */
static void params_to_cdt(struct spi_nor_info *params, struct sfc_cdt *cdt)
{

	/* 4.nor singleRead */
	MK_CMD(cdt[NOR_READ_STANDARD], params->read_standard, 0, ROW_ADDR, ENABLE);

	/* 5.nor quadRead */
	MK_CMD(cdt[NOR_READ_QUAD], params->read_quad, 0, ROW_ADDR, ENABLE);

	/* 6. nor writeStandard */
	MK_CMD(cdt[NOR_WRITE_STANDARD_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_WRITE_STANDARD], params->write_standard, 1, ROW_ADDR, ENABLE);
	MK_ST(cdt[NOR_WRITE_STANDARD_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 7. nor writeQuad */
	MK_CMD(cdt[NOR_WRITE_QUAD_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_WRITE_QUAD], params->write_quad, 1, ROW_ADDR, ENABLE);
	MK_ST(cdt[NOR_WRITE_QUAD_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 8. nor erase */
	MK_CMD(cdt[NOR_ERASE_WRITE_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_ERASE], params->sector_erase, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NOR_ERASE_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 9. quad mode */
	if(params->quad_ops_mode){
		MK_CMD(cdt[NOR_QUAD_SET_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
		MK_ST(cdt[NOR_QUAD_SET], params->quad_set, 1, DEFAULT_ADDRMODE, 0, DISABLE, ENABLE, TM_STD_SPI);  //disable poll, enable data

		MK_ST(cdt[NOR_QUAD_FINISH], params->busy, 1, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

		MK_ST(cdt[NOR_QUAD_GET], params->quad_get, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);
	}

	/* 10. nor write ENABLE */
	MK_CMD(cdt[NOR_WRITE_ENABLE], params->wr_en, 0, DEFAULT_ADDRMODE, DISABLE);

	/* 11. entry 4byte mode */
	MK_CMD(cdt[NOR_EN_4BYTE], params->en4byte, 0, DEFAULT_ADDRMODE, DISABLE);

	/* 12. die select */
	cdt[NOR_DIE_SELECT].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	cdt[NOR_DIE_SELECT].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_DIE_SEL);
	cdt[NOR_DIE_SELECT].staExp = 0;
	cdt[NOR_DIE_SELECT].staMsk = 0;

	/* 13. read active die ID */
	cdt[NOR_READ_ACTIVE_DIE_ID].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	cdt[NOR_READ_ACTIVE_DIE_ID].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_READ_DIE_ID);
	cdt[NOR_READ_ACTIVE_DIE_ID].staExp = 0;
	cdt[NOR_READ_ACTIVE_DIE_ID].staMsk = 0;


}

static void nor_create_cdt_table(struct sfc *sfc, void *flash_info, uint32_t flag)
{
	struct spinor_flashinfo *nor_info = flash_info;
	struct sfc_cdt sfc_cdt[INDEX_MAX_NUM];

	memset(sfc_cdt, 0, sizeof(sfc_cdt));

	if(flag & DEFAULT_CDT){

		/* 1.nor reset */
		sfc_cdt[NOR_RESET_ENABLE].link = CMD_LINK(1, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_RESET_ENABLE].xfer = CMD_XFER(0, DISABLE, 0, DISABLE, SPINOR_OP_RSTEN);
		sfc_cdt[NOR_RESET_ENABLE].staExp = 0;
		sfc_cdt[NOR_RESET_ENABLE].staMsk = 0;

		sfc_cdt[NOR_RESET].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_RESET].xfer = CMD_XFER(0, DISABLE, 0, DISABLE, SPINOR_OP_RST);
		sfc_cdt[NOR_RESET].staExp = 0;
		sfc_cdt[NOR_RESET].staMsk = 0;


		/* 2.nor read id */
		sfc_cdt[NOR_READ_ID].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_READ_ID].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDID);
		sfc_cdt[NOR_READ_ID].staExp = 0;
		sfc_cdt[NOR_READ_ID].staMsk = 0;


		/* 3. nor get status */
		sfc_cdt[NOR_GET_STATUS].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_GET_STATUS].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR);
		sfc_cdt[NOR_GET_STATUS].staExp = 0;
		sfc_cdt[NOR_GET_STATUS].staMsk = 0;

		sfc_cdt[NOR_GET_STATUS_1].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_GET_STATUS_1].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR_1);
		sfc_cdt[NOR_GET_STATUS_1].staExp = 0;
		sfc_cdt[NOR_GET_STATUS_1].staMsk = 0;

		sfc_cdt[NOR_GET_STATUS_2].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
		sfc_cdt[NOR_GET_STATUS_2].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR_2);
		sfc_cdt[NOR_GET_STATUS_2].staExp = 0;
		sfc_cdt[NOR_GET_STATUS_2].staMsk = 0;

		if (!(flag & UPDATE_CDT)) {
			/* 4.nor singleRead */
			sfc_cdt[NOR_READ_STANDARD].link = CMD_LINK(0, ROW_ADDR, TM_STD_SPI);
			sfc_cdt[NOR_READ_STANDARD].xfer = CMD_XFER(DEFAULT_ADDRSIZE, DISABLE, 0, ENABLE, SPINOR_OP_READ);
			sfc_cdt[NOR_READ_STANDARD].staExp = 0;
			sfc_cdt[NOR_READ_STANDARD].staMsk = 0;

			/* first create cdt table (default)*/
			write_cdt(sfc, sfc_cdt, NOR_RESET_ENABLE, NOR_READ_STANDARD);
			return;
		}
	}


	if(flag & UPDATE_CDT){

		params_to_cdt(nor_info->nor_flash_info, sfc_cdt);

		if (!(flag & DEFAULT_CDT)) {
			/* second create cdt table (update)*/
			write_cdt(sfc, sfc_cdt, NOR_READ_STANDARD, NOR_READ_ACTIVE_DIE_ID);
		} else {
			/* create full cdt table (default && update)*/
			write_cdt(sfc, sfc_cdt, NOR_RESET_ENABLE, NOR_READ_ACTIVE_DIE_ID);
		}
	}
	//dump_cdt(sfc);
	return;
}


static struct multi_die_flash die_flash[MULTI_DIE_FLASH_NUM] = {
	[0] = {0xc84019, 2, "GD25S512MD"},
};

int ingenic_sfc_nor_probe(struct sfc_flash *flash)
{
	struct mtd_partition *ingenic_mtd_partition;
	const char *ingenic_probe_types[] = {"cmdlinepart", "ofpart", NULL};
	int num_partition_info = 0;
	int err = 0,ret = 0;
	struct spinor_flashinfo *nor_info;
	int i;
	int tchsh;
	int tslch;
	int tshsl_rd;
	int tshsl_wr;
	uint32_t die_size;

	nor_info = kzalloc(sizeof(*nor_info), GFP_KERNEL);
	if(!nor_info) {
		dev_err(flash->dev, "alloc nor_info failed!\n");
		return -ENOMEM;
	}
	flash->flash_info = nor_info;

	set_flash_timing(flash->sfc, DEF_TCHSH, DEF_TSLCH, DEF_TSHSL_R, DEF_TSHSL_W);

	/* request DMA Descriptor space */
	ret = request_sfc_desc(flash->sfc, DESC_MAX_NUM);
	if(ret){
		dev_err(flash->dev, "Failure to request DMA descriptor space!\n");
		ret = -ENOMEM;
		goto err_sfc_desc_request;
	}

	/* try creating default CDT table */
	flash->create_cdt_table = nor_create_cdt_table;
	flash->create_cdt_table(flash->sfc, flash->flash_info, DEFAULT_CDT);

	ret = sfc_nor_reset(flash);
	if(ret) {
		dev_warn(flash->dev, "Failed to reset nor flash, Try to go on\n");
	}

	ret = ingenic_spi_norflash_get_params(flash);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to match correct nor flash device!\n");
		goto err_match_device;
	}

	/* Update to private CDT table */
	flash->create_cdt_table(flash->sfc, flash->flash_info, UPDATE_CDT);

	/* Update sfc rate */
	if((ret = sfc_clk_set_highspeed(flash->sfc))) {
		dev_err(flash->dev, "set sfc rate failed\n");
		goto err_match_device;
	}

#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
	num_partition_info = nor_info->norflash_partitions->num_partition_info;
	ingenic_mtd_partition = (struct mtd_partition*)kzalloc(sizeof(struct mtd_partition) * num_partition_info, GFP_KERNEL);
	if (!ingenic_mtd_partition) {
		ret = -ENOMEM;
		dev_err(flash->dev, "Failed to alloc mem for ingenic_mtd_partition\n");
		goto err_alloc_partition;
	}

	for (i = 0; i < num_partition_info; i++) {
		ingenic_mtd_partition[i].name = nor_info->norflash_partitions->nor_partition[i].name;
		ingenic_mtd_partition[i].offset = nor_info->norflash_partitions->nor_partition[i].offset;

		if (nor_info->norflash_partitions->nor_partition[i].size == -1) {
			ingenic_mtd_partition[i].size = MTDPART_SIZ_FULL;
		} else {
			ingenic_mtd_partition[i].size = nor_info->norflash_partitions->nor_partition[i].size;
		}

		if (nor_info->norflash_partitions->nor_partition[i].mask_flags & NORFLASH_PART_RO) {    //have problem!!!
			ingenic_mtd_partition[i].mask_flags = MTD_CAP_RAM;
		} else {
			ingenic_mtd_partition[i].mask_flags = MTD_CAP_ROM;
		}

	}
#endif

	flash->mtd.name     = "sfc_mtd";
	flash->mtd.owner    = THIS_MODULE;
	flash->mtd.type     = MTD_NORFLASH;
	flash->mtd.flags    = MTD_CAP_NORFLASH;
	flash->mtd.erasesize    = nor_info->nor_pri_data->fs_erase_size;
	flash->mtd.writesize    = nor_info->nor_flash_info->page_size;
	flash->mtd.size     = nor_info->nor_flash_info->chip_size;
	flash->mtd._erase   = ingenic_spi_norflash_erase;
	flash->mtd._read    = ingenic_spi_norflash_read;
	flash->mtd._write   = ingenic_spi_norflash_write;

	tchsh = nor_info->nor_flash_info->tCHSH;
	tslch = nor_info->nor_flash_info->tSLCH;
	tshsl_rd = nor_info->nor_flash_info->tSHSL_RD;
	tshsl_wr = nor_info->nor_flash_info->tSHSL_WR;
	set_flash_timing(flash->sfc, tchsh, tslch, tshsl_rd, tshsl_wr);

	sfc_nor_get_special_ops(flash);

	if (nor_info->nor_pri_data->uk_quad) {
		if (nor_info->nor_flash_ops->set_quad_mode) {
			ret = nor_info->nor_flash_ops->set_quad_mode(flash);
			if (ret < 0) {
				nor_info->quad_succeed = 0;
				dev_info(flash->dev, "set quad mode error !\n");
			} else {
				nor_info->quad_succeed = 1;
				dev_info(flash->dev, "nor flash quad mode is set, now use quad mode!\n");
			}
		}
	}

	/* Multi Die support */
	nor_info->die_num = 1;
	for (i = 0; i < MULTI_DIE_FLASH_NUM; i++) {
		if(!(strcmp(die_flash[i].flash_name, nor_info->nor_flash_info->name))) {
			nor_info->die_num = die_flash[i].die_num;
			die_size = nor_info->nor_flash_info->chip_size / nor_info->die_num;
			nor_info->die_shift = ffs(die_size) - 1;
			nor_info->current_die_id = 0;

			printk("Flash :%s support multi die, die number:%d\n", die_flash[i].flash_name, die_flash[i].die_num);
			break;
		}
	}


	/* if nor flash size is greater than 16M, use 4byte mode */
	if(flash->mtd.size > NOR_SIZE_16M) {
		if (nor_info->nor_flash_ops->set_4byte_mode) {
			nor_info->nor_flash_ops->set_4byte_mode(flash);
		}
	}

	/*
	 * partiton parser:  1.cmdlinepart  2.device tree  3.flash burner
	 */
	flash->dev->of_node = NULL;
	if (flash->pdata_params->use_ofpart_info) {
		flash->dev->of_node = of_get_child_by_name(flash->dev->of_node, "norflash");
		if (flash->dev->of_node < 0) {
			dev_err(flash->dev, "Cannot get 'norflash' node from dtb!\n");
			flash->dev->of_node = NULL;
		}
	}

	ret = mtd_device_parse_register(&flash->mtd, ingenic_probe_types, &flash->ppdata, ingenic_mtd_partition, num_partition_info);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to parse register!\n");
		goto err_parse_register;
	}

	flash->attr_group = &sfc_norflash_info_attr_group;
	ret = sysfs_create_group(&flash->dev->kobj, flash->attr_group);
	if (err) {
		dev_err(flash->dev, "failed to register sysfs\n");
		ret = -EIO;
		goto err_create_group;
	}

	dev_info(flash->dev,"SPI NOR MTD LOAD OK\n");
	return 0;

err_create_group:
	mtd_device_unregister(&flash->mtd);
err_parse_register:
	kfree(ingenic_mtd_partition);
#ifndef CONFIG_INGENIC_BUILTIN_PARAMS
err_alloc_partition:
	kfree(burner_params);
#endif
err_match_device:
err_sfc_desc_request:
	free_sfc_desc(flash->sfc);
	kfree(nor_info);
	return ret;

}

