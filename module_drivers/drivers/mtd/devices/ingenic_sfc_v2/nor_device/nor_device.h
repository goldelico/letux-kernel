#ifndef __NOR_DEVICE_H
#define __NOR_DEVICE_H
#include <linux/types.h>
#include "../sfc.h"
#include "../sfc_flash.h"
#include "../spinor.h"


#define CMD_INFO(_CMD, COMMAND, DUMMY_BIT, ADDR_LEN, TRANSFER_MODE) {	\
	_CMD.cmd = COMMAND;			\
	_CMD.dummy_byte = DUMMY_BIT;		\
	_CMD.addr_nbyte = ADDR_LEN;		\
	_CMD.transfer_mode = TRANSFER_MODE;	\
}

#define ST_INFO(_ST, COMMAND, BIT_SHIFT, MASK, VAL, LEN, DUMMY_BIT) {	\
	_ST.cmd = COMMAND;		\
	_ST.bit_shift = BIT_SHIFT;	\
	_ST.mask = MASK;		\
	_ST.val = VAL;			\
	_ST.len = LEN;			\
	_ST.dummy = DUMMY_BIT;		\
}

/*
 * create default cdt params
 */
#define CDT_PARAMS_INIT_COMMON(cdt_params, addr_bytes) {	\
	/* read standard	0x03 0 3/4 0 */						\
	CMD_INFO(cdt_params->read_standard, SPINOR_OP_READ, 0, addr_bytes, TM_STD_SPI);		\
	/* read quad		0x6b 8 3/4 5 */						\
	CMD_INFO(cdt_params->read_quad, SPINOR_OP_READ_1_1_4, 8, addr_bytes, TM_QI_QO_SPI);	\
	/* write standard	0x02 0 3/4 0 */						\
	CMD_INFO(cdt_params->write_standard, SPINOR_OP_PP, 0, addr_bytes, TM_STD_SPI);		\
	/* write quad		0x32 0 3/4 5 */						\
	CMD_INFO(cdt_params->write_quad, SPINOR_OP_QPP, 0, addr_bytes, TM_QI_QO_SPI);		\
	/* erase sector		0x52 0 3/4 0 */						\
	CMD_INFO(cdt_params->sector_erase, SPINOR_OP_BE_32K, 0, addr_bytes, TM_STD_SPI);		\
	/* write enable		0x06 0 0 0 */						\
	CMD_INFO(cdt_params->wr_en, SPINOR_OP_WREN, 0, 0, TM_STD_SPI);			\
	\
	/* wait oip not busy	0x05 0 1 0 1 0 */					\
	ST_INFO(cdt_params->busy, SPINOR_OP_RDSR, 0, 1, 0, 1, 0);			\
}

#define CDT_PARAMS_INIT_SPECIAL(cdt_params) {	\
	/* read standard	0x13 0 4 0 */						\
	CMD_INFO(cdt_params->read_standard, SPINOR_OP_READ4, 0, 4, TM_STD_SPI);		\
	/* read quad		0x6c 8 4 5 */						\
	CMD_INFO(cdt_params->read_quad, SPINOR_OP_READ4_1_1_4, 8, 4, TM_QI_QO_SPI);	\
	/* write standard	0x12 0 4 0 */						\
	CMD_INFO(cdt_params->write_standard, SPINOR_OP_PP_4B, 0, 4, TM_STD_SPI);		\
	/* write quad		0x34 0 4 5 */						\
	CMD_INFO(cdt_params->write_quad, SPINOR_OP_QPP_4B, 0, 4, TM_QI_QO_SPI);		\
	/* erase sector		0x5c 0 4 0 */						\
	CMD_INFO(cdt_params->sector_erase, SPINOR_OP_BE_32K_4B, 0, 4, TM_STD_SPI);	\
	/* entry 4byte		0xb7 0 0 0 */						\
	CMD_INFO(cdt_params->en4byte, SPINOR_OP_EN4B, 0, 0, TM_STD_SPI);		\
	/* write enable		0x06 0 0 0 */						\
	CMD_INFO(cdt_params->wr_en, SPINOR_OP_WREN, 0, 0, TM_STD_SPI);			\
	\
	/* wait oip not busy	0x05 0 1 0 1 0 */					\
	ST_INFO(cdt_params->busy, SPINOR_OP_RDSR, 0, 1, 0, 1, 0);			\
}

#endif
