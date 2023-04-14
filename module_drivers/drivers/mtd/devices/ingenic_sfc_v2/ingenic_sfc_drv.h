#ifndef __INGENIC_SFC_H
#define __INGENIC_SFC_H

#include "sfc_flash.h"

enum flash_type {
	NAND,
	NOR,
};

struct sfc_data {
	int (*flash_type_auto_detect)(struct platform_device *pdev);
};

int ingenic_sfc_nand_probe(struct sfc_flash *flash);
int ingenic_sfc_nor_probe(struct sfc_flash *flash);

#endif
