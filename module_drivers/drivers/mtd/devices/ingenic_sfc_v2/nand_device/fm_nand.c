#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinand.h"
#include "../ingenic_sfc_common.h"
#include "nand_common.h"

#define FM_DEVICES_NUM         2
#define TSETUP		5
#define THOLD		5
#define	TSHSL_R		80
#define	TSHSL_W		80

#define TRD		100
#define TPP		900
#define TBE		10

static struct ingenic_sfcnand_device *fm_nand;
static struct ingenic_sfcnand_base_param fm_param[FM_DEVICES_NUM] = {
	[0] = {
		/*FM25S01A*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 1024,
		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x1,
		.need_quad = 1,
	},
	[1] = {
		/*FM25S02A*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 1024 * 2,
		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x1,
		.need_quad = 1,
	},
};

static struct device_id_struct device_id[FM_DEVICES_NUM] = {
	DEVICE_ID_STRUCT(0xE4, "FM25S01A", &fm_param[0]),
	DEVICE_ID_STRUCT(0xE5, "FM25S02A", &fm_param[1]),
};

static cdt_params_t *fm_get_cdt_params(struct sfc_flash *flash, uint8_t device_id)
{
	CDT_PARAMS_INIT(fm_nand->cdt_params);

	switch(device_id) {
	    case 0xE4:
	    case 0xE5:
		    break;
	    default:
		    dev_err(flash->dev, "device_id err, please check your  device id: device_id = 0x%02x\n", device_id);
		    return NULL;
	}

	return &fm_nand->cdt_params;
}

static inline int deal_ecc_status(struct sfc_flash *flash, uint8_t device_id, uint8_t ecc_status)
{
	int ret = 0;

	switch(device_id) {
		case 0xE4:
		case 0xE5:
			switch((ret = ((ecc_status >> 4) & 0x3))) {
				case 0x0 ... 0x1:
					ret = 0;
					break;
				default:
					ret = -EBADMSG;
			}
			break;
		default:
			dev_err(flash->dev, "device_id err, it maybe don`t support this device, check your device id: device_id = 0x%02x\n", device_id);
			ret = -EIO;   //notice!!!

	}
	return ret;
}

static int __init fm_nand_init(void) {
	fm_nand = kzalloc(sizeof(*fm_nand), GFP_KERNEL);
	if(!fm_nand) {
		pr_err("alloc fm_nand struct fail\n");
		return -ENOMEM;
	}

	fm_nand->id_manufactory = 0xA1;
	fm_nand->id_device_list = device_id;
	fm_nand->id_device_count = FM_DEVICES_NUM;

	fm_nand->ops.get_cdt_params = fm_get_cdt_params;
	fm_nand->ops.deal_ecc_status = deal_ecc_status;

	/* use private get feature interface, please define it in this document */
	fm_nand->ops.get_feature = NULL;

	return ingenic_sfcnand_register(fm_nand);
}
fs_initcall(fm_nand_init);
