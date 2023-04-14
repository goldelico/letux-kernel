#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinor.h"
#include "../ingenic_sfc_common.h"
#include "nor_device.h"


#define NOR_DEVICES_NUM         3

/* base params */
static struct nor_params_node nor_params_node[NOR_DEVICES_NUM] = {

#ifdef CONFIG_INGENIC_GD25Q127C
	/*GD25Q128C*/
	[0] = {
		.nor_device_info.name = "GD25Q127C",
		.nor_device_info.id = 0xc84018,

		.nor_device_info.tCHSH = 5,
		.nor_device_info.tSLCH = 5,
		.nor_device_info.tSHSL_RD = 20,
		.nor_device_info.tSHSL_WR = 50,

		.nor_device_info.chip_size = 16777216,
		.nor_device_info.page_size = 256,
		.nor_device_info.erase_size = 32768,

		.nor_device_info.quad_ops_mode = 1,
		.nor_device_info.addr_ops_mode = 0,
	},
#endif

#ifdef CONFIG_INGENIC_GD25Q256C
	/*GD25Q256C*/
	[1] = {
		.nor_device_info.name = "GD25Q256C",
		.nor_device_info.id = 0xc84019,

		.nor_device_info.tCHSH = 5,
		.nor_device_info.tSLCH = 5,
		.nor_device_info.tSHSL_RD = 20,
		.nor_device_info.tSHSL_WR = 50,

		.nor_device_info.chip_size = 33554432,
		.nor_device_info.page_size = 256,
		.nor_device_info.erase_size = 32768,

		.nor_device_info.quad_ops_mode = 1,
		.nor_device_info.addr_ops_mode = 0,
	},
#endif

#ifdef CONFIG_INGENIC_GD25S512MD
	/*GD25S512MD*/
	[2] = {
		.nor_device_info.name = "GD25S512MD",
		.nor_device_info.id = 0xc84019,

		.nor_device_info.tCHSH = 5,
		.nor_device_info.tSLCH = 8,
		.nor_device_info.tSHSL_RD = 20,
		.nor_device_info.tSHSL_WR = 20,

		.nor_device_info.chip_size = 67108864,
		.nor_device_info.page_size = 256,
		.nor_device_info.erase_size = 32768,

		.nor_device_info.quad_ops_mode = 1,
		.nor_device_info.addr_ops_mode = 0,
	},
#endif

};

/* cdt params */
static struct spi_nor_info *get_cdt_params(struct sfc_flash *flash, struct spi_nor_info *nor_device_info)
{
	switch(nor_device_info->id) {
	    case 0xc84018:
		if(!(strcmp(nor_device_info->name, "GD25Q127C"))) {
			CDT_PARAMS_INIT_COMMON(nor_device_info, 3);
			/* quad set		0x31 1 1 1 1 0 */
			ST_INFO(nor_device_info->quad_set, SPINOR_OP_WRSR_1, 1, 1, 1, 1, TM_STD_SPI);
			/* quad get		0x35 1 1 1 1 0 */
			ST_INFO(nor_device_info->quad_get, SPINOR_OP_RDSR_1, 1, 1, 1, 1, TM_STD_SPI);
			break;
		    }
	    case 0xc84019:
		if(!(strcmp(nor_device_info->name, "GD25Q256C"))) {
			CDT_PARAMS_INIT_COMMON(nor_device_info, 4);
			/* entry 4byte		0xb7 0 0 0 */
			CMD_INFO(nor_device_info->en4byte, SPINOR_OP_EN4B, 0, 0, TM_STD_SPI);
			/* quad set		0x01 6 1 1 1 0 */
			ST_INFO(nor_device_info->quad_set, SPINOR_OP_WRSR, 6, 1, 1, 1, TM_STD_SPI);
			/* quad get		0x05 6 1 1 1 0 */
			ST_INFO(nor_device_info->quad_get, SPINOR_OP_RDSR, 6, 1, 1, 1, TM_STD_SPI);
			break;
		}
		if(!(strcmp(nor_device_info->name, "GD25S512MD"))) {
			CDT_PARAMS_INIT_SPECIAL(nor_device_info);
			/* quad set		0x31 1 1 1 1 0 */
			ST_INFO(nor_device_info->quad_set, SPINOR_OP_WRSR_1, 1, 1, 1, 1, TM_STD_SPI);
			/* quad get		0x35 1 1 1 1 0 */
			ST_INFO(nor_device_info->quad_get, SPINOR_OP_RDSR_1, 1, 1, 1, 1, TM_STD_SPI);
			break;
		}
	    default:
		    dev_err(flash->dev, "device_id err, please check your  device id: device_id = 0x%02x\n", nor_device_info->id);
		    return NULL;
	}

	return nor_device_info;
}


static struct nor_private_data private_data = {
	.fs_erase_size = 32768,
	.uk_quad = 1,
};

struct builtin_params *get_nor_builtin_params(struct sfc_flash *flash, struct builtin_params *params)
{
	/* private params */
	params->magic = NOR_MAGIC;
	params->version = NOR_VERSION;
	params->nor_pri_data = &private_data;

	/* cdt params */
	params->spi_nor_info = get_cdt_params(flash, params->spi_nor_info);

	return params;
}


static int __init nor_device_init(void) {

	int  i, count = 0;
	for(i = 0; i < NOR_DEVICES_NUM; i++) {
		if (nor_params_node[i].nor_device_info.id) {
			ingenic_sfcnor_register(&nor_params_node[i]);
			count++;
		}
	}

	if (!count) {
		pr_err("Register SPI Nor Flash params fail !\n");
		return -EINVAL;
	}

	return 0;
}

fs_initcall(nor_device_init);
