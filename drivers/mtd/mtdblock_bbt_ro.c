#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>

struct m_mtd_data {
	struct mtd_blktrans_dev dev;
	void *bbt;
	int block_count;
	int usable_count;
};

#define MY_BLOCK_SIZE CONFIG_MTD_BLOCK_BBT_RO_BLOCK_SIZE

static void create_bbt(struct m_mtd_data *data, struct mtd_info *mtd)
{
	int i, bad_count;
	loff_t offs = 0;
	int block_count = mtd_div_by_eb(mtd->size, mtd);
	int item_size;
	void *bbt;


	if (block_count < 256)
		item_size = 1;
	else if (block_count < 65536)
		item_size = 2;
	else
		item_size = 4;

	bbt = kzalloc(block_count * item_size, GFP_KERNEL);
	BUG_ON(!bbt);

	for (i = 0, bad_count = 0; i < block_count; i++) {
		int is_bad = mtd_block_isbad(mtd, offs) > 0;
		offs += mtd->erasesize;
		if (is_bad) {
			bad_count++;
			continue;
		}

		switch (item_size) {
		case 1:
			((unsigned char *)bbt)[i-bad_count] = i; break;
		case 2:
			((unsigned short *)bbt)[i-bad_count] = i; break;
		case 4:
			((unsigned int *)bbt)[i-bad_count] = i; break;
		}
	}

	printk("mtd: %s %lld %d %d\n",
		mtd->name, mtd->size, mtd->erasesize, bad_count);

	data->usable_count = block_count - bad_count;
	data->block_count = block_count;
	data->bbt = bbt;
}

static unsigned int get_from_bbt(struct m_mtd_data *data, int index)
{
	int block_count = data->block_count;
	void *bbt = data->bbt;
	int ret;

	if (block_count < 256)
		ret = ((unsigned char *)bbt)[index];
	else if (block_count < 65536)
		ret = ((unsigned short *)bbt)[index];
	else
		ret = ((unsigned int *)bbt)[index];

	if (ret == 0)
		return index ? -1 : 0;
	else
		return ret;
}

static int mtdblock_readsect(struct mtd_blktrans_dev *dev,
				  unsigned long block, char *buf)
{
	struct m_mtd_data *data = (struct m_mtd_data *)dev;
	struct mtd_info *mtd = dev->mtd;
	int index = mtd_div_by_eb(((uint64_t)block * MY_BLOCK_SIZE), mtd);
	int delta = mtd_mod_by_eb(((uint64_t)block * MY_BLOCK_SIZE), mtd);
	int err;
	loff_t off;
	size_t retlen;

	if (!data->bbt)
		create_bbt(data, mtd);

	if (!data->usable_count)
		return -EOVERFLOW;

	index = get_from_bbt(data, index);
	if (index < 0)
		return -EOVERFLOW;

	off = index * mtd->erasesize + delta;

	err = mtd_read(dev->mtd, off, MY_BLOCK_SIZE, &retlen, buf);
	if (err) {
			/*
			 * -EUCLEAN is reported if there was a bit-flip which
			 * was corrected, so this is harmless.
			 *
			 * We do not report about it here unless debugging is
			 * enabled. A corresponding message will be printed
			 * later, when it is has been scrubbed.
			 */
		if (mtd_is_bitflip(err) && (MY_BLOCK_SIZE == retlen))
			return 0;
		else
			return 1;
	}

	return 0;
}

static int mtdblock_writesect(struct mtd_blktrans_dev *dev,
				  unsigned long block, char *buf)
{
	return 1;
}

static void mtdblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct m_mtd_data *data = kzalloc(sizeof(*data), GFP_KERNEL);
	struct mtd_blktrans_dev *dev = (struct mtd_blktrans_dev *)data;
	BUG_ON(!data);

	dev->mtd = mtd;
	dev->devnum = mtd->index;
	dev->size = mtd->size / MY_BLOCK_SIZE;
	dev->tr = tr;
	dev->readonly = 1;

	if (add_mtd_blktrans_dev(dev))
		kfree(dev);
}

static void mtdblock_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct m_mtd_data *data = (struct m_mtd_data *)dev;

	if (data->bbt)
		kfree(data->bbt);

	del_mtd_blktrans_dev(dev);

}

static struct mtd_blktrans_ops mtdblock_tr = {
	.name		= "mtdblock_bbt_ro",
	.major		= MTD_BLOCK_MAJOR,
	.part_bits	= 0,
	.blksize 	= MY_BLOCK_SIZE,
	.readsect	= mtdblock_readsect,
	.writesect	= mtdblock_writesect,
	.add_mtd	= mtdblock_add_mtd,
	.remove_dev	= mtdblock_remove_dev,
	.owner		= THIS_MODULE,
};

static int __init mtdblock_init(void)
{
	return register_mtd_blktrans(&mtdblock_tr);
}

static void __exit mtdblock_exit(void)
{
	deregister_mtd_blktrans(&mtdblock_tr);
}

module_init(mtdblock_init);
module_exit(mtdblock_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wujiao");
MODULE_DESCRIPTION("Simple bad block tab based read-only block device emulation access to MTD devices");
