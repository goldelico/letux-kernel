#ifndef __SPINOR_H
#define __SPINOR_H
#include "sfc_flash.h"
#include "spinor_cmd.h"

#define SPIFLASH_PARAMER_OFFSET 0x5800
#define NOR_MAJOR_VERSION_NUMBER	2
#define NOR_MINOR_VERSION_NUMBER	0
#define NOR_REVERSION_NUMBER	0
#define NOR_VERSION		(NOR_MAJOR_VERSION_NUMBER | (NOR_MINOR_VERSION_NUMBER << 8) | (NOR_REVERSION_NUMBER << 16))


#define SIZEOF_NAME			32

#define NOR_MAGIC	0x726f6e	//ascii "nor"
#define NOR_PART_NUM	10
#define NORFLASH_PART_RW	0
#define NORFLASH_PART_WO	1
#define NORFLASH_PART_RO	2

/* the max number of DMA Descriptor */
#define DESC_MAX_NUM	64


struct spi_nor_cmd_info {
	unsigned short cmd;
	unsigned char dummy_byte;
	unsigned char addr_nbyte;
	unsigned char transfer_mode;

};

struct spi_nor_st_info {
	unsigned short cmd;
	unsigned char bit_shift;
	unsigned char mask;
	unsigned char val;
	unsigned char len; //length of byte to operate from register
	unsigned char dummy;
};

struct spi_nor_info {
	unsigned char name[32];
	unsigned int id;

	struct spi_nor_cmd_info read_standard;
	struct spi_nor_cmd_info read_quad;

	struct spi_nor_cmd_info write_standard;
	struct spi_nor_cmd_info write_quad;

	struct spi_nor_cmd_info sector_erase;

	struct spi_nor_cmd_info wr_en;
	struct spi_nor_cmd_info en4byte;
	struct spi_nor_st_info	quad_set;
	struct spi_nor_st_info	quad_get;
	struct spi_nor_st_info	busy;

	unsigned short quad_ops_mode;
	unsigned short addr_ops_mode;

	unsigned int tCHSH;      //hold
	unsigned int tSLCH;      //setup
	unsigned int tSHSL_RD;   //interval
	unsigned int tSHSL_WR;

	unsigned int chip_size;
	unsigned int page_size;
	unsigned int erase_size;

	unsigned char chip_erase_cmd;
};

struct nor_partition {
	char name[32];
	uint32_t size;
	uint32_t offset;
	uint32_t mask_flags;//bit 0-1 mask the partiton RW mode, 0:RW  1:W  2:R
	uint32_t manager_mode;
};

struct norflash_partitions {
	struct nor_partition nor_partition[NOR_PART_NUM];
	uint32_t num_partition_info;
};

struct nor_private_data {
	unsigned int fs_erase_size;
	unsigned char uk_quad;
};


struct burner_params {
	uint32_t magic;
	uint32_t version;
	struct spi_nor_info spi_nor_info;
	struct norflash_partitions norflash_partitions;
	struct nor_private_data nor_pri_data;
};

struct nor_params_node {
	struct spi_nor_info nor_device_info;
	struct list_head list;
};

struct builtin_params {
	uint32_t magic;
	uint32_t version;
	struct spi_nor_info *spi_nor_info;
	struct nor_private_data *nor_pri_data;
};

struct spi_nor_flash_ops {
	int (*set_4byte_mode)(struct sfc_flash *flash);
	int (*set_quad_mode)(struct sfc_flash *flash);
};

struct spinor_flashinfo {

	uint8_t	current_die_id;
	uint32_t die_shift;
	uint32_t die_num;
	int quad_succeed;
	struct spi_nor_flash_ops *nor_flash_ops;
	struct spi_nor_info *nor_flash_info;
	struct spi_nor_cmd_info *cur_r_cmd;
	struct spi_nor_cmd_info *cur_w_cmd;
	struct norflash_partitions *norflash_partitions;
	struct nor_private_data *nor_pri_data;

};

struct multi_die_flash {
	uint32_t flash_id;
	uint32_t die_num;
	char* flash_name;
};

int ingenic_sfcnor_register(struct nor_params_node *);
struct builtin_params *get_nor_builtin_params(struct sfc_flash *, struct builtin_params *);
int32_t sfc_nor_get_special_ops(struct sfc_flash *);

/* SFC CDT Maximum INDEX number */
#define INDEX_MAX_NUM 32

enum {
	/* 1. nor reset */
	NOR_RESET_ENABLE,
	NOR_RESET,

	/* 2. nor read id */
	NOR_READ_ID,

	/* 3. nor get status */
	NOR_GET_STATUS,
	NOR_GET_STATUS_1,
	NOR_GET_STATUS_2,

	/* 4. nor singleRead */
	NOR_READ_STANDARD,

	/* 5. nor quadRead */
	NOR_READ_QUAD,

	/* 6. nor writeStandard */
	NOR_WRITE_STANDARD_ENABLE,
	NOR_WRITE_STANDARD,
	NOR_WRITE_STANDARD_FINISH,

	/* 7. nor writeQuad */
	NOR_WRITE_QUAD_ENABLE,
	NOR_WRITE_QUAD,
	NOR_WRITE_QUAD_FINISH,

	/* 8. nor erase */
	NOR_ERASE_WRITE_ENABLE,
	NOR_ERASE,
	NOR_ERASE_FINISH,

	/* 9. quad mode */
	NOR_QUAD_SET_ENABLE,
	NOR_QUAD_SET,
	NOR_QUAD_FINISH,
	NOR_QUAD_GET,

	/* 10. nor write ENABLE */
	NOR_WRITE_ENABLE,

	/* 11. entry 4byte mode */
	NOR_EN_4BYTE,

	/* 13. active die */
	NOR_DIE_SELECT,

	/* 14. read die id */
	NOR_READ_ACTIVE_DIE_ID,

	/* index count */
	NOR_MAX_INDEX,
};


#endif

