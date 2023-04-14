#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinand.h"
#include "../ingenic_sfc_common.h"
#include "nand_common.h"

#define	TOSHIBA_DEVICES_NUM         1
#define TSETUP		3
#define THOLD		3
#define	TSHSL_R		100
#define	TSHSL_W		100

#define TRD		300
#define TPP		600
#define TBE		7

static struct ingenic_sfcnand_device *toshiba_nand;
static struct ingenic_sfcnand_base_param toshiba_param[TOSHIBA_DEVICES_NUM] = {

	[0] = {
		/*TC58CVG2S0HRAIJ */
		.pagesize = 4 * 1024,
		.blocksize = 4 * 1024 * 64,
		.oobsize = 128,
		.flashsize = 4 * 1024 * 64 * 2048,

		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.ecc_max = 0x8,
		.need_quad = 1,
	},
};

static struct device_id_struct device_id[TOSHIBA_DEVICES_NUM] = {
	DEVICE_ID_STRUCT(0xed, "TC58CVG2S0HRAIJ", &toshiba_param[0]),
};
static cdt_params_t *toshiba_get_cdt_params(struct sfc_flash *flash, uint8_t device_id)
{
	CDT_PARAMS_INIT(toshiba_nand->cdt_params);
	switch(device_id) {
		case 0xed:
			break;
		default:
			dev_err(flash->dev,"device_id err, please check your  device id: device_id = 0x%02x\n", device_id);
			return NULL;
	}
	return &toshiba_nand->cdt_params;
}

static inline int deal_ecc_status(struct sfc_flash *flash, uint8_t device_id, uint8_t ecc_status)
{
	int ret = 0;
	switch(device_id) {
		case 0xed:
			switch((ecc_status >> 4) & 0x3) {
				case 0x00:
					ret = 0;
					break;
				case 0x01:
					ret = 0x1;
					break;
				case 0x11:
					ret = 0x11;
					break;
				default:
					ret = -EBADMSG;
					break;
			}
			break;
		default:
			dev_warn(flash->dev,"device_id err, it maybe don`t support this device, check your device id: device_id = 0x%02x\n", device_id);
			ret = -EIO;
	}
	return ret;
}

static int toshiba_nand_init(void) {
	toshiba_nand = kzalloc(sizeof(*toshiba_nand), GFP_KERNEL);
	if(!toshiba_nand) {
		pr_err("alloc toshiba_nand struct fail\n");
		return -ENOMEM;
	}
	toshiba_nand->id_manufactory = 0x98;
	toshiba_nand->id_device_list = device_id;
	toshiba_nand->id_device_count = TOSHIBA_DEVICES_NUM;
	toshiba_nand->ops.get_cdt_params = toshiba_get_cdt_params;
	toshiba_nand->ops.deal_ecc_status = deal_ecc_status;
	toshiba_nand->ops.get_feature = NULL;
	return ingenic_sfcnand_register(toshiba_nand);
}
fs_initcall(toshiba_nand_init);
