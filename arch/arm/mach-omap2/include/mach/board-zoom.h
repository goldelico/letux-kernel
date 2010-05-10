/*
 * Defines for zoom boards
 */
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct flash_partitions {
	struct mtd_partition *parts;
	int nr_parts;
};

#define ZOOM_NAND_CS    0

extern void __init zoom_flash_init(struct flash_partitions [], int);
extern int __init zoom_debugboard_init(void);
extern void __init zoom_peripherals_init(void);
