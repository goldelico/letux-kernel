#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinand.h"
#include "../ingenic_sfc_common.h"
#include "nand_common.h"

#define XTX_MID2C_DEVICES_NUM         1
#define TSETUP		4
#define THOLD		4
#define	TSHSL_R		30
#define	TSHSL_W		30

#define TRD		70
#define TPP		600
#define TBE		10

static struct ingenic_sfcnand_device *xtx_mid2c_nand;

static struct ingenic_sfcnand_base_param xtx_mid2c_param[XTX_MID2C_DEVICES_NUM] = {

	[0] = {
		/*XT26G02E */
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 2048,

		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 1,
		.ecc_max = 0x8,
		.need_quad = 1,
	},

};

static struct device_id_struct device_id[XTX_MID2C_DEVICES_NUM] = {
	DEVICE_ID_STRUCT(0x24, "XT26G02E ", &xtx_mid2c_param[0]),
};


static cdt_params_t *xtx_mid2c_get_cdt_params(struct sfc_flash *flash, uint8_t device_id)
{
	CDT_PARAMS_INIT(xtx_mid2c_nand->cdt_params);

	switch(device_id) {
	    case 0x24:
		    break;
	    default:
		    dev_err(flash->dev, "device_id err, please check your  device id: device_id = 0x%02x\n", device_id);
		    return NULL;
	}

	return &xtx_mid2c_nand->cdt_params;
}


static inline int deal_ecc_status(struct sfc_flash *flash, uint8_t device_id, uint8_t ecc_status)
{
	int ret = 0;

	switch(device_id) {
		case 0x24:
			switch((ecc_status >> 4) & 0x7) {
				case 0x5:
					ret = 8;
					break;
				case 0x3:
					ret = 6;
					break;
				case 0x2:
					ret = -EBADMSG;
					break;
				case 0x1:
					ret = 3;
					break;
				default:
					ret = 0;
					break;
			}
			break;

		default:
			dev_warn(flash->dev, "device_id err, it maybe don`t support this device, check your device id: device_id = 0x%02x\n", device_id);
			ret = -EIO;

	}
	return ret;
}


static int xtx_mid2c_nand_init(void) {

	xtx_mid2c_nand = kzalloc(sizeof(*xtx_mid2c_nand), GFP_KERNEL);
	if(!xtx_mid2c_nand) {
		pr_err("alloc xtx_mid2c_nand struct fail\n");
		return -ENOMEM;
	}

	xtx_mid2c_nand->id_manufactory = 0x2C;
	xtx_mid2c_nand->id_device_list = device_id;
	xtx_mid2c_nand->id_device_count = XTX_MID2C_DEVICES_NUM;

	xtx_mid2c_nand->ops.get_cdt_params = xtx_mid2c_get_cdt_params;
	xtx_mid2c_nand->ops.deal_ecc_status = deal_ecc_status;

	/* use private get feature interface, please define it in this document */
	xtx_mid2c_nand->ops.get_feature = NULL;

	return ingenic_sfcnand_register(xtx_mid2c_nand);
}

fs_initcall(xtx_mid2c_nand_init);
