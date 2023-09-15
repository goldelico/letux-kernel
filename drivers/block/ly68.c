// SPDX-License-Identifier: GPL-2.0-or-later
/*
    Block device driver for Lyontek LY68 series PSRAM chips

    Copyright (C) 2023 SudoMaker, Ltd.
    Author: Reimu NotMoe <reimu@sudomaker.com>
*/

#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>

struct ly68_caps {
	unsigned int size;
};

struct ly68_timing {
	u64			ts_op_start;
	u64			ts_last_op;
	u64			ts_sleep_till;
	unsigned		op_sleep_ratio;
};

struct ly68_ctx {
	struct spi_device	*spi;
	struct gendisk		*disk;
	struct mutex		lock;
	struct ly68_timing	timing;
	unsigned		tx_speed_hz, rx_speed_hz;
	struct blk_mq_tag_set	tag_set;
};

#define LY68_PAGE_SIZE			SZ_1K
#define LY68_CMD_WRITE			0x02
#define LY68_CMD_READ			0x0b
#define LY68_WHOLE_CHIP_REFRESH_TIME	65000000

#define LY68_MAX_TRANS_SEGS		CONFIG_LYONTEK_LY68_MAX_TRANS_SEGS

static inline void ly68_dynamic_sleep(u32 nsecs)
{
	if (nsecs > LY68_WHOLE_CHIP_REFRESH_TIME)
		nsecs = LY68_WHOLE_CHIP_REFRESH_TIME;

	if (nsecs < 10 * 1000)
		ndelay(nsecs);
	else
		usleep_range(nsecs / 1000, nsecs / 1000);
}

static void ly68_timing_start(struct ly68_timing *t)
{
	u64 ts_cur = ktime_get_ns();

	if (ts_cur < t->ts_sleep_till) {
		ly68_dynamic_sleep((u32)(t->ts_sleep_till - ts_cur));
		t->ts_op_start = ktime_get_ns();
	} else {
		t->ts_op_start = ts_cur;
	}
}

static void ly68_timing_end(struct ly68_timing *t)
{
	u64 ts_op_end = ktime_get_ns();
	u32 t_op = (u32)(ts_op_end - t->ts_op_start);
	u32 t_sleep = t_op / t->op_sleep_ratio + ((t_op % t->op_sleep_ratio) ? 1 : 0);

	t->ts_sleep_till = ts_op_end + t_sleep;
}

static inline u32 ly68_cmd_word(u32 addr, u32 cmd)
{
	addr &= 0x00ffffff;
	return (cmd << 24) | addr;
}

static blk_status_t ly68_transfer(struct ly68_ctx *ly68,
			enum req_op op, struct request *rq)
{
	struct spi_transfer transfer[2] = {};
	struct spi_message message;
	u32 command_32b[2];

	struct req_iterator iter;
	struct bio_vec bvec;
	void *data;
	u32 start_pos = blk_rq_pos(rq) << SECTOR_SHIFT;
	u32 len = blk_rq_cur_bytes(rq);

	int ret = 0;

	spi_message_init(&message);

	transfer[0].tx_buf = command_32b;
	transfer[0].bits_per_word = 32;
	spi_message_add_tail(&transfer[0], &message);

	transfer[1].len = len;
	transfer[1].bits_per_word = 32;

	if (op == REQ_OP_READ) {
		if (ly68->rx_speed_hz) {
			transfer[0].speed_hz = ly68->rx_speed_hz;
			transfer[1].speed_hz = ly68->rx_speed_hz;
		}

		command_32b[0] = ly68_cmd_word(start_pos - 3, LY68_CMD_READ);
		transfer[0].len = sizeof(u32) * 2;

		data = kmap(bio_page(rq->bio)) + bio_offset(rq->bio);

		transfer[1].tx_buf = NULL;
		transfer[1].rx_buf = data;
		spi_message_add_tail(&transfer[1], &message);

		ly68_timing_start(&ly68->timing);
		ret = spi_sync(ly68->spi, &message);
		ly68_timing_end(&ly68->timing);

		kunmap(bio_page(rq->bio));

		rq_for_each_segment(bvec, rq, iter)
			flush_dcache_page(bvec.bv_page);
	} else if (op == REQ_OP_WRITE) {
		if (ly68->tx_speed_hz) {
			transfer[0].speed_hz = ly68->tx_speed_hz;
			transfer[1].speed_hz = ly68->tx_speed_hz;
		}

		command_32b[0] = ly68_cmd_word(start_pos, LY68_CMD_WRITE);
		transfer[0].len = sizeof(u32) * 1;

		rq_for_each_segment(bvec, rq, iter)
			flush_dcache_page(bvec.bv_page);

		data = kmap(bio_page(rq->bio)) + bio_offset(rq->bio);

		transfer[1].tx_buf = data;
		transfer[1].rx_buf = NULL;
		spi_message_add_tail(&transfer[1], &message);

		ly68_timing_start(&ly68->timing);
		ret = spi_sync(ly68->spi, &message);
		ly68_timing_end(&ly68->timing);

		kunmap(bio_page(rq->bio));
	}

	return ret ? BLK_STS_IOERR : BLK_STS_OK;
}

static blk_status_t ly68_queue_rq(struct blk_mq_hw_ctx *hctx,
				 const struct blk_mq_queue_data *bd)
{
	struct ly68_ctx *ly68 = hctx->queue->queuedata;
	struct request *rq = bd->rq;
	enum req_op op = req_op(rq);
	blk_status_t err;

	switch (op) {
	case REQ_OP_READ:
	case REQ_OP_WRITE: {
		mutex_lock(&ly68->lock);

		blk_mq_start_request(rq);
		do {
			err = ly68_transfer(ly68, op, rq);
		} while (blk_update_request(rq, err, blk_rq_cur_bytes(rq)));
		blk_mq_end_request(rq, err);

		mutex_unlock(&ly68->lock);

		return BLK_STS_OK;
	}
	default:
		return BLK_STS_IOERR;
	}
}

static const struct block_device_operations ly68_fops = {
	.owner		= THIS_MODULE,
};

static const struct blk_mq_ops ly68_mq_ops = {
	.queue_rq	= ly68_queue_rq,
};

static const struct ly68_caps ly68l6400_caps = {
	.size = SZ_8M,
};

static const struct ly68_caps ly68l3200_caps = {
	.size = SZ_4M,
};

static int ly68_probe(struct spi_device *spi)
{
	struct ly68_ctx *ly68;
	const struct ly68_caps *caps;
	struct gendisk *disk;
	int err;

	caps = of_device_get_match_data(&spi->dev);
	if (!caps)
		caps = &ly68l6400_caps;

	ly68 = devm_kzalloc(&spi->dev, sizeof(*ly68), GFP_KERNEL);
	if (!ly68)
		return -ENOMEM;

	ly68->spi = spi;
	ly68->disk = disk;
	mutex_init(&ly68->lock);

	err = blk_mq_alloc_sq_tag_set(&ly68->tag_set, &ly68_mq_ops, 1,
				BLK_MQ_F_SHOULD_MERGE | BLK_MQ_F_NO_SCHED_BY_DEFAULT | BLK_MQ_F_BLOCKING);
	if (err)
		goto out;

	disk = blk_mq_alloc_disk(&ly68->tag_set, ly68);
	if (!disk)
		goto out_nuke_tagset;

	disk->first_minor = 0;
	disk->flags = GENHD_FL_NO_PART;
	disk->fops = &ly68_fops;
	disk->private_data = ly68;
	strcpy(disk->disk_name, "ly68psram");

	set_capacity(disk, caps->size >> SECTOR_SHIFT);

	blk_queue_flag_set(QUEUE_FLAG_NONROT, disk->queue);
	blk_queue_physical_block_size(disk->queue, 4096);
	blk_queue_logical_block_size(disk->queue, 4096);
	blk_queue_max_hw_sectors(disk->queue, (LY68_MAX_TRANS_SEGS * 4096) >> SECTOR_SHIFT);
	blk_queue_max_segments(disk->queue, LY68_MAX_TRANS_SEGS);
	blk_queue_max_segment_size(disk->queue, LY68_MAX_TRANS_SEGS * 4096);

	err = add_disk(disk);
	if (err)
		goto out_cleanup_disk;

	ly68->timing.op_sleep_ratio = 8;
	device_property_read_u32(&spi->dev, "lyontek,op-sleep-ratio", &ly68->timing.op_sleep_ratio);
	device_property_read_u32(&spi->dev, "lyontek,tx-max-frequency", &ly68->tx_speed_hz);
	device_property_read_u32(&spi->dev, "lyontek,rx-max-frequency", &ly68->rx_speed_hz);

	// If you remove this line of code below, you will be violating the GPL license directly.
	// We won't be doing nothing if that's the case. Think twice.
	// 如果你删除下面这行代码，你将会直接违反 GPL 许可。
	// 我们面对侵权不会无动于衷，国内已有相关判例。请三思。
	dev_info(&spi->dev, "Lyontek LY68 series PSRAM support by SudoMaker (https://su.mk)\n");

	ly68->timing.ts_last_op = ktime_get_ns();

	return 0;

out_nuke_tagset:
	blk_mq_free_tag_set(&ly68->tag_set);
out_cleanup_disk:
	put_disk(disk);
out:
	return err;
}

static void ly68_remove(struct spi_device *spi)
{
	struct ly68_ctx *ly68 = spi_get_drvdata(spi);

	put_disk(ly68->disk);
}

static const struct of_device_id ly68_of_table[] = {
	{
		.compatible = "lyontek,ly68l6400",
		.data = &ly68l6400_caps,
	},
	{
		.compatible = "lyontek,ly68l3200",
		.data = &ly68l3200_caps,
	},
	{}
};
MODULE_DEVICE_TABLE(of, ly68_of_table);

static const struct spi_device_id ly68_spi_ids[] = {
	{
		.name = "lyontek_ly68l6400",
		.driver_data = (kernel_ulong_t)&ly68l6400_caps,
	},
	{
		.name = "lyontek_ly68l3200",
		.driver_data = (kernel_ulong_t)&ly68l3200_caps,
	},
	{}
};
MODULE_DEVICE_TABLE(spi, ly68_spi_ids);

static struct spi_driver ly68_driver = {
	.driver = {
		.name	= "lyontek_ly68",
		.of_match_table = ly68_of_table,
	},
	.probe		= ly68_probe,
	.remove		= ly68_remove,
	.id_table	= ly68_spi_ids,
};

module_spi_driver(ly68_driver);

MODULE_DESCRIPTION("Block driver for Lyontek LY68 series SPI PSRAM chips");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:lyontek_ly68");
