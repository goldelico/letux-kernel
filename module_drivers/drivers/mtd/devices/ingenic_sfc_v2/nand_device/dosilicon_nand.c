#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinand.h"
#include "../ingenic_sfc_common.h"
#include "nand_common.h"

#define DOSILICON_DEVICES_NUM         5
#define THOLD	    5
#define TSETUP	    5
#define TSHSL_R	    100
#define TSHSL_W	    100

#define TRD	    90
#define TPP	    700
#define TBE	    10

struct ingenic_sfcnand_device *dosilicon_nand;

static struct ingenic_sfcnand_base_param dosilicon_param[DOSILICON_DEVICES_NUM] = {
	[0] = {
	/*DS35Q1GAXXX*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 1024,

		.tHOLD  = THOLD,
		.tSETUP = TSETUP,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = 70,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x4,
		.need_quad = 1,
	},
	[1] = {
	/*DS35Q2GAXXX*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 2048,

		.tHOLD  = THOLD,
		.tSETUP = TSETUP,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = 90,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 1,
		.ecc_max = 0x4,
		.need_quad = 1,
	},
	[2] = {
	/*DS35Q2GBXXX*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 128,
		.flashsize = 2 * 1024 * 64 * 2048,

		.tHOLD  = THOLD,
		.tSETUP = TSETUP,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = 120,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 1,
		.ecc_max = 0x8,
		.need_quad = 1,
	},
	[3] = {
	/*DS35M1GAXXX*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 1024,

		.tHOLD  = THOLD,
		.tSETUP = TSETUP,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = 80,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x4,
		.need_quad = 1,
	},
	[4] = {
	/*DS35Q2GAXXX-1V8*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 64,
		.flashsize = 2 * 1024 * 64 * 2048,

		.tHOLD  = THOLD,
		.tSETUP = TSETUP,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = 90,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 1,
		.ecc_max = 0x4,
		.need_quad = 1,
	},
};

static struct device_id_struct device_id[DOSILICON_DEVICES_NUM] = {
	DEVICE_ID_STRUCT(0x71, "DS35Q1GAXXX", &dosilicon_param[0]),
	DEVICE_ID_STRUCT(0x72, "DS35Q2GAXXX", &dosilicon_param[1]),
	DEVICE_ID_STRUCT(0xF2, "DS35Q2GBXXX", &dosilicon_param[2]),
	DEVICE_ID_STRUCT(0x21, "DS35M1GAXXX", &dosilicon_param[3]),
	DEVICE_ID_STRUCT(0x22, "DS35Q2GAXXX-1V8", &dosilicon_param[4]),
};


static cdt_params_t *dosilicon_get_cdt_params(struct sfc_flash *flash, uint8_t device_id)
{
	CDT_PARAMS_INIT(dosilicon_nand->cdt_params);

	switch(device_id) {
	    case 0x71:
	    case 0x72:
	    case 0xF2:
	    case 0x21:
	    case 0x22:
		    break;
	    default:
		    dev_err(flash->dev, "device_id err, please check your  device id: device_id = 0x%02x\n", device_id);
		    return NULL;
	}

	return &dosilicon_nand->cdt_params;
}


static inline int deal_ecc_status(struct sfc_flash *flash, uint8_t device_id, uint8_t ecc_status)
{
	int ret = 0;

	switch(device_id) {
		case 0x71:
		case 0x72:
		case 0x21:
		case 0x22:
			switch((ecc_status >> 0x4) & 0x3) {
			    case 0x0:
			    case 0x1:
				    ret = 0;
				    break;
			    case 0x2:
				    ret = -EBADMSG;
				    break;
			    default:
				   printk("it is flash Unknown state, device_id: 0x%02x\n", device_id);
				    ret = -EIO;
			}
			break;
		case 0xF2:
			switch((ecc_status >> 4) & 0x7) {
				case 0x2:
					ret = -EBADMSG;
					break;
				default:
					ret = 0;
					break;
			}
			break;

		default:
			dev_err(flash->dev, "device_id err,it maybe don`t support this device, please check your device id: device_id = 0x%02x\n", device_id);
		ret = -EIO;
	}
	return ret;
}


static int dosilicon_nand_init(void) {

	dosilicon_nand = kzalloc(sizeof(*dosilicon_nand), GFP_KERNEL);
	if(!dosilicon_nand) {
		pr_err("alloc dosilicon_nand struct fail\n");
		return -ENOMEM;
	}

	dosilicon_nand->id_manufactory = 0xE5;
	dosilicon_nand->id_device_list = device_id;
	dosilicon_nand->id_device_count = DOSILICON_DEVICES_NUM;

	dosilicon_nand->ops.get_cdt_params = dosilicon_get_cdt_params;
	dosilicon_nand->ops.deal_ecc_status = deal_ecc_status;

	/* use private get feature interface, please define it in this document */
	dosilicon_nand->ops.get_feature = NULL;

	return ingenic_sfcnand_register(dosilicon_nand);
}

fs_initcall(dosilicon_nand_init);
