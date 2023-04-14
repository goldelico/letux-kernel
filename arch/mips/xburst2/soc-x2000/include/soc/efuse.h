#ifndef __INGENIC_EFUSE_H__
#define __INGENIC_EFUSE_H__

#define EFUSE_CTRL		0x0
#define EFUSE_CFG		0x4
#define EFUSE_STATE		0x8
#define EFUSE_DATA(n)		(0xC + (n) * 4)

#define EFUSE_RIR_RF                 (0)
#define EFUSE_RIR_DATA               (1)
#define EFUSE_RIR_ADDR               (2)
#define EFUSE_RIR_DISABLE            (15)


// efuse ctrl bits
#define EFUSE_CTRL_ADDR              (21)
#define EFUSE_CTRL_ADDR_MASK         (0x3f)
#define EFUSE_CTRL_LEN               (16)
#define EFUSE_CTRL_LEN_MASK          (7)
#define EFUSE_CTRL_PGEN              (1 << 15)
#define EFUSE_CTRL_RWL               (1 << 11)
#define EFUSE_CTRL_MR                (1 << 10)
#define EFUSE_CTRL_PS                (1 << 9)
#define EFUSE_CTRL_PD                (1 << 8)
#define EFUSE_CTRL_WREN              (1 << 1)
#define EFUSE_CTRL_RDEN              (1 << 0)


// efuse cfg bits
#define EFUSE_CFG_INT_EN             (1 << 31)
#define EFUSE_CFG_RD_ADJ             (24)
#define EFUSE_CFG_RD_ADJ_MASK        (0xf)
#define EFUSE_CFG_RD_STROBE          (16)
#define EFUSE_CFG_RD_STROBE_MASK     (0x1f)
#define EFUSE_CFG_WR_ADJ             (12)
#define EFUSE_CFG_WR_ADJ_MASK        (0xf)
#define EFUSE_CFG_WR_STROBE          (16)
#define EFUSE_CFG_WR_STROBE_MASK     (0x3ff)


// efuse state bits
#define EFUSE_STA_NKU_PRT            (1 << 23)
#define EFUSE_STA_USERKEY1_PRT       (1 << 22)
#define EFUSE_STA_USERKEY0_PRT       (1 << 21)
#define EFUSE_STA_CHIPKEY_PRT        (1 << 20)
#define EFUSE_STA_HIDEBLK_PRT        (1 << 19)
#define EFUSE_STA_SOCINFO_PRT        (1 << 18)
#define EFUSE_STA_TRIM2_PRT          (1 << 17)
#define EFUSE_STA_TRIM1_PRT          (1 << 16)
#define EFUSE_STA_TRIM0_PRT          (1 << 15)
#define EFUSE_STA_CUSTID2_PRT        (1 << 14)
#define EFUSE_STA_CUSTID1_PRT        (1 << 13)
#define EFUSE_STA_CUSTID0_PRT        (1 << 12)
#define EFUSE_STA_CHIPID_PRT         (1 << 11)
#define EFUSE_STA_SECBOOT_PRT        (1 << 10)
#define EFUSE_STA_DIS_JTAG           (1 << 9)
#define EFUSE_STA_SECBOOT_EN         (1 << 8)
#define EFUSE_STA_WR_DONE            (1 << 1)
#define EFUSE_STA_RD_DONE            (1 << 0)



#define CHIPID_OFFSET_ADDR            (0x0)
#define CUSTID0_OFFSET_ADDR           (0x11)
#define CUSTID1_OFFSET_ADDR           (0x22)
#define CUSTID2_OFFSET_ADDR           (0x3f)
#define TRIM0_OFFSET_ADDR             (0x5c)
#define TRIM1_OFFSET_ADDR             (0x61)
#define TRIM2_OFFSET_ADDR             (0x66)
#define SOCINFO_OFFSET_ADDR           (0x6b)
#define PRT_OFFSET_ADDR               (0x70)
#define HIDEBLK_OFFSET_ADDR           (0x74)
#define CHIPKEY_OFFSET_ADDR           (0x78)
#define USERKEY0_OFFSET_ADDR          (0x9a)
#define USERKEY1_OFFSET_ADDR          (0xbc)
#define NKU_OFFSET_ADDR               (0xde)

#define CHIPID_WORD_ADDR            (0x0)
#define CUSTID0_WORD_ADDR           (0x4)
#define CUSTID1_WORD_ADDR           (0x8)
#define CUSTID2_WORD_ADDR           (0xf)
#define TRIM0_WORD_ADDR             (0x17)
#define TRIM1_WORD_ADDR             (0x18)
#define TRIM2_WORD_ADDR             (0x19)
#define SOCINFO_WORD_ADDR           (0x1a)
#define PRT_WORD_ADDR               (0x1c)
#define HIDEBLK_WORD_ADDR           (0x1d)
#define CHIPKEY_WORD_ADDR           (0x1e)
#define USERKEY0_WORD_ADDR          (0x26)
#define USERKEY1_WORD_ADDR          (0x2f)
#define NKU_WORD_ADDR               (0x37)

#define CHIPID_BIT_NUM		    (128)
#define CUSTID0_BIT_NUM		    (128)
#define CUSTID1_BIT_NUM             (216)
#define CUSTID2_BIT_NUM             (216)
#define TRIM0_BIT_NUM               (32)
#define TRIM1_BIT_NUM               (32)
#define TRIM2_BIT_NUM               (32)
#define SOCINFO_BIT_NUM             (32)
#define PRT_BIT_NUM                 (16)
#define HIDEBLK_BIT_NUM             (16)
#define CHIPKEY_BIT_NUM             (256)
#define USERKEY0_BIT_NUM            (256)
#define USERKEY1_BIT_NUM            (256)
#define NKU_BIT_NUM                 (256)

#define CHIPID_EXT_BIT_NUM           (8)
#define CUSTID0_EXT_BIT_NUM          (8)
#define CUSTID1_EXT_BIT_NUM          (16)
#define CUSTID2_EXT_BIT_NUM          (16)
#define TRIM0_EXT_BIT_NUM            (8)
#define TRIM1_EXT_BIT_NUM            (8)
#define TRIM2_EXT_BIT_NUM            (8)
#define SOCINFO_EXT_BIT_NUM          (8)
#define PRT_EXT_BIT_NUM              (16)
#define HIDEBLK_EXT_BIT_NUM          (16)
#define CHIPKEY_EXT_BIT_NUM          (16)
#define USERKEY0_EXT_BIT_NUM         (16)
#define USERKEY1_EXT_BIT_NUM         (16)
#define NKU_EXT_BIT_NUM              (16)

#define CHIPID_WORD_NUM           (((CHIPID_BIT_NUM	+ CHIPID_EXT_BIT_NUM 	) / 32) + (((CHIPID_BIT_NUM	+ CHIPID_EXT_BIT_NUM 	) % 32) ? 1 : 0))
#define CUSTID0_WORD_NUM          (((CUSTID0_BIT_NUM	+ CUSTID0_EXT_BIT_NUM	) / 32) + (((CUSTID0_BIT_NUM	+ CUSTID0_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define CUSTID1_WORD_NUM          (((CUSTID1_BIT_NUM	+ CUSTID1_EXT_BIT_NUM	) / 32) + (((CUSTID1_BIT_NUM	+ CUSTID1_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define CUSTID2_WORD_NUM          (((CUSTID2_BIT_NUM	+ CUSTID2_EXT_BIT_NUM	) / 32) + (((CUSTID2_BIT_NUM	+ CUSTID2_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define TRIM0_WORD_NUM            (((TRIM0_BIT_NUM	+ TRIM0_EXT_BIT_NUM	) / 32) + (((TRIM0_BIT_NUM	+ TRIM0_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define TRIM1_WORD_NUM            (((TRIM1_BIT_NUM	+ TRIM1_EXT_BIT_NUM	) / 32) + (((TRIM1_BIT_NUM	+ TRIM1_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define TRIM2_WORD_NUM            (((TRIM2_BIT_NUM	+ TRIM2_EXT_BIT_NUM	) / 32) + (((TRIM2_BIT_NUM	+ TRIM2_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define SOCINFO_WORD_NUM          (((SOCINFO_BIT_NUM	+ SOCINFO_EXT_BIT_NUM	) / 32) + (((SOCINFO_BIT_NUM	+ SOCINFO_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define PRT_WORD_NUM              (((PRT_BIT_NUM	+ PRT_EXT_BIT_NUM	) / 32) + (((PRT_BIT_NUM	+ PRT_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define HIDEBLK_WORD_NUM          (((HIDEBLK_BIT_NUM	+ HIDEBLK_EXT_BIT_NUM	) / 32) + (((HIDEBLK_BIT_NUM	+ HIDEBLK_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define CHIPKEY_WORD_NUM          (((CHIPKEY_BIT_NUM	+ CHIPKEY_EXT_BIT_NUM	) / 32) + (((CHIPKEY_BIT_NUM	+ CHIPKEY_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define USERKEY0_WORD_NUM         (((USERKEY0_BIT_NUM	+ USERKEY0_EXT_BIT_NUM	) / 32) + (((USERKEY0_BIT_NUM	+ USERKEY0_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define USERKEY1_WORD_NUM         (((USERKEY1_BIT_NUM	+ USERKEY1_EXT_BIT_NUM	) / 32) + (((USERKEY1_BIT_NUM	+ USERKEY1_EXT_BIT_NUM	) % 32) ? 1 : 0))
#define NKU_WORD_NUM              (((NKU_BIT_NUM	+ NKU_EXT_BIT_NUM	) / 32) + (((NKU_BIT_NUM	+ NKU_EXT_BIT_NUM	) % 32) ? 1 : 0))


#define CHIPID_BEGIN_ALIGN          (CHIPID_OFFSET_ADDR   - (CHIPID_WORD_ADDR   << 2))
#define CUSTID0_BEGIN_ALIGN         (CUSTID0_OFFSET_ADDR  - (CUSTID0_WORD_ADDR  << 2))
#define CUSTID1_BEGIN_ALIGN         (CUSTID1_OFFSET_ADDR  - (CUSTID1_WORD_ADDR  << 2))
#define CUSTID2_BEGIN_ALIGN         (CUSTID2_OFFSET_ADDR  - (CUSTID2_WORD_ADDR  << 2))
#define TRIM0_BEGIN_ALIGN           (TRIM0_OFFSET_ADDR    - (TRIM0_WORD_ADDR    << 2))
#define TRIM1_BEGIN_ALIGN           (TRIM1_OFFSET_ADDR    - (TRIM1_WORD_ADDR    << 2))
#define TRIM2_BEGIN_ALIGN           (TRIM2_OFFSET_ADDR	  - (TRIM2_WORD_ADDR    << 2))
#define SOCINFO_BEGIN_ALIGN         (SOCINFO_OFFSET_ADDR  - (SOCINFO_WORD_ADDR  << 2))
#define PRT_BEGIN_ALIGN             (PRT_OFFSET_ADDR      - (PRT_WORD_ADDR      << 2))
#define HIDEBLK_BEGIN_ALIGN         (HIDEBLK_OFFSET_ADDR  - (HIDEBLK_WORD_ADDR  << 2))
#define CHIPKEY_BEGIN_ALIGN         (CHIPKEY_OFFSET_ADDR  - (CHIPKEY_WORD_ADDR  << 2))
#define USERKEY0_BEGIN_ALIGN        (USERKEY0_OFFSET_ADDR - (USERKEY0_WORD_ADDR << 2))
#define USERKEY1_BEGIN_ALIGN        (USERKEY1_OFFSET_ADDR - (USERKEY1_WORD_ADDR << 2))
#define NKU_BEGIN_ALIGN             (NKU_OFFSET_ADDR      - (NKU_WORD_ADDR      << 2))


#define CHIPID_END_ALIGN	    (4 - CHIPID_BEGIN_ALIGN   - (((CHIPID_BIT_NUM   + CHIPID_EXT_BIT_NUM   ) % 32) >> 3))
#define CUSTID0_END_ALIGN           (4 - CUSTID0_BEGIN_ALIGN  - (((CUSTID0_BIT_NUM  + CUSTID0_EXT_BIT_NUM  ) % 32) >> 3))
#define CUSTID1_END_ALIGN           (4 - CUSTID1_BEGIN_ALIGN  - (((CUSTID1_BIT_NUM  + CUSTID1_EXT_BIT_NUM  ) % 32) >> 3))
#define CUSTID2_END_ALIGN           (4 - CUSTID2_BEGIN_ALIGN  - (((CUSTID2_BIT_NUM  + CUSTID2_EXT_BIT_NUM  ) % 32) >> 3))
#define TRIM0_END_ALIGN             (4 - TRIM0_BEGIN_ALIGN    - (((TRIM0_BIT_NUM    + TRIM0_EXT_BIT_NUM	   ) % 32) >> 3))
#define TRIM1_END_ALIGN             (4 - TRIM1_BEGIN_ALIGN    - (((TRIM1_BIT_NUM    + TRIM1_EXT_BIT_NUM	   ) % 32) >> 3))
#define TRIM2_END_ALIGN             (4 - TRIM2_BEGIN_ALIGN    - (((TRIM2_BIT_NUM    + TRIM2_EXT_BIT_NUM	   ) % 32) >> 3))
#define SOCINFO_END_ALIGN           (4 - SOCINFO_BEGIN_ALIGN  - (((SOCINFO_BIT_NUM  + SOCINFO_EXT_BIT_NUM  ) % 32) >> 3))
#define PRT_END_ALIGN               (4 - PRT_BEGIN_ALIGN      - (((PRT_BIT_NUM      + PRT_EXT_BIT_NUM	   ) % 32) >> 3))
#define HIDEBLK_END_ALIGN           (4 - HIDEBLK_BEGIN_ALIGN  - (((HIDEBLK_BIT_NUM  + HIDEBLK_EXT_BIT_NUM  ) % 32) >> 3))
#define CHIPKEY_END_ALIGN           (4 - CHIPKEY_BEGIN_ALIGN  - (((CHIPKEY_BIT_NUM  + CHIPKEY_EXT_BIT_NUM  ) % 32) >> 3))
#define USERKEY0_END_ALIGN          (4 - USERKEY0_BEGIN_ALIGN - (((USERKEY0_BIT_NUM + USERKEY0_EXT_BIT_NUM ) % 32) >> 3))
#define USERKEY1_END_ALIGN          (4 - USERKEY1_BEGIN_ALIGN - (((USERKEY1_BIT_NUM + USERKEY1_EXT_BIT_NUM ) % 32) >> 3))
#define NKU_END_ALIGN               (4 - NKU_BEGIN_ALIGN      - (((NKU_BIT_NUM      + NKU_EXT_BIT_NUM	   ) % 32) >> 3))


enum segment_id {
	CHIPID = 0,
	CUSTID0,
	CUSTID1,
	CUSTID2,
	TRIM0,
	TRIM1,
	TRIM2,
	SOCINFO,
	PRT,
	HIDEBLK,
	CHIPKEY,
	USERKEY0,
	USERKEY1,
	NKU
};

enum verify_mode {
	NONE = 0,
	DOUBLE,
	HAMMING
};

struct seg_info {
	char seg_name[32];
	uint32_t seg_id;
	uint32_t word_address;
	uint32_t word_num;
	uint32_t bit_num;
	uint32_t begin_align;
	uint32_t end_align;
	uint32_t prt_bit;
	uint32_t verify_mode;
};

static struct seg_info seg_info_array[] = {
	[0] = {
		.seg_name = "CHIPID",
		.seg_id = CHIPID,
		.word_address = CHIPID_WORD_ADDR,
		.begin_align = CHIPID_BEGIN_ALIGN,
		.end_align = CHIPID_END_ALIGN,
		.bit_num = CHIPID_BIT_NUM,
		.word_num = CHIPID_WORD_NUM,
		.prt_bit = EFUSE_STA_CHIPID_PRT,
		.verify_mode = HAMMING,
	},
	[1] = {
		.seg_name = "CUSTID0",
		.seg_id = CUSTID0,
		.word_address = CUSTID0_WORD_ADDR,
		.begin_align = CUSTID0_BEGIN_ALIGN,
		.end_align = CUSTID0_END_ALIGN,
		.bit_num = CUSTID0_BIT_NUM,
		.word_num = CUSTID0_WORD_NUM,
		.prt_bit = EFUSE_STA_CUSTID0_PRT,
		.verify_mode = HAMMING,
	},
	[2] = {
		.seg_name = "CUSTID1",
		.seg_id = CUSTID1,
		.word_address = CUSTID1_WORD_ADDR,
		.begin_align = CUSTID1_BEGIN_ALIGN,
		.end_align = CUSTID1_END_ALIGN,
		.bit_num = CUSTID1_BIT_NUM,
		.word_num = CUSTID1_WORD_NUM,
		.prt_bit = EFUSE_STA_CUSTID1_PRT,
		.verify_mode = HAMMING,
	},
	[3] = {
		.seg_name = "CUSTID2",
		.seg_id = CUSTID2,
		.word_address = CUSTID2_WORD_ADDR,
		.begin_align = CUSTID2_BEGIN_ALIGN,
		.end_align = CUSTID2_END_ALIGN,
		.bit_num = CUSTID2_BIT_NUM,
		.word_num = CUSTID2_WORD_NUM,
		.prt_bit = EFUSE_STA_CUSTID2_PRT,
		.verify_mode = HAMMING,
	},
	[4] = {
		.seg_name = "TRIM0",
		.seg_id = TRIM0,
		.word_address = TRIM0_WORD_ADDR,
		.begin_align = TRIM0_BEGIN_ALIGN,
		.end_align = TRIM0_END_ALIGN,
		.bit_num = TRIM0_BIT_NUM,
		.word_num = TRIM0_WORD_NUM,
		.prt_bit = EFUSE_STA_TRIM0_PRT,
		.verify_mode = HAMMING,
	},
	[5] = {
		.seg_name = "TRIM1",
		.seg_id = TRIM1,
		.word_address = TRIM1_WORD_ADDR,
		.begin_align = TRIM1_BEGIN_ALIGN,
		.end_align = TRIM1_END_ALIGN,
		.bit_num = TRIM1_BIT_NUM,
		.word_num = TRIM1_WORD_NUM,
		.prt_bit = EFUSE_STA_TRIM1_PRT,
		.verify_mode = HAMMING,
	},
	[6] = {
		.seg_name = "TRIM2",
		.seg_id = TRIM2,
		.word_address = TRIM2_WORD_ADDR,
		.begin_align = TRIM2_BEGIN_ALIGN,
		.end_align = TRIM2_END_ALIGN,
		.bit_num = TRIM2_BIT_NUM,
		.word_num = TRIM2_WORD_NUM,
		.prt_bit = EFUSE_STA_TRIM2_PRT,
		.verify_mode = NONE,
	},
	[7] = {
		.seg_name = "SOCINFO",
		.seg_id = SOCINFO,
		.word_address = SOCINFO_WORD_ADDR,
		.begin_align = SOCINFO_BEGIN_ALIGN,
		.end_align = SOCINFO_END_ALIGN,
		.bit_num = SOCINFO_BIT_NUM + SOCINFO_EXT_BIT_NUM,
		.word_num = SOCINFO_WORD_NUM,
		.prt_bit = EFUSE_STA_SOCINFO_PRT,
		.verify_mode = DOUBLE,
	},
	[8] = {
		.seg_name = "PRT",
		.seg_id = PRT,
		.word_address = PRT_WORD_ADDR,
		.begin_align = PRT_BEGIN_ALIGN,
		.end_align = PRT_END_ALIGN,
		.bit_num = PRT_BIT_NUM + PRT_EXT_BIT_NUM,
		.word_num = PRT_WORD_NUM,
		.prt_bit = 0xffffffff,
		.verify_mode = DOUBLE,
	},
	[9] = {
		.seg_name = "HIDEBLK",
		.seg_id = HIDEBLK,
		.word_address = HIDEBLK_WORD_ADDR,
		.begin_align = HIDEBLK_BEGIN_ALIGN,
		.end_align = HIDEBLK_END_ALIGN,
		.bit_num = HIDEBLK_BIT_NUM,
		.word_num = HIDEBLK_WORD_NUM,
		.prt_bit = EFUSE_STA_HIDEBLK_PRT,
		.verify_mode = HAMMING,
	},
	[10] = {
		.seg_name = "CHIPKEY",
		.seg_id = CHIPKEY,
		.word_address = CHIPKEY_WORD_ADDR,
		.begin_align = CHIPKEY_BEGIN_ALIGN,
		.end_align = CHIPKEY_END_ALIGN,
		.bit_num = CHIPKEY_BIT_NUM,
		.word_num = CHIPKEY_WORD_NUM,
		.prt_bit = EFUSE_STA_CHIPKEY_PRT,
		.verify_mode = HAMMING,
	},
	[11] = {
		.seg_name = "USERKEY0",
		.seg_id = USERKEY0,
		.word_address = USERKEY0_WORD_ADDR,
		.begin_align = USERKEY0_BEGIN_ALIGN,
		.end_align = USERKEY0_END_ALIGN,
		.bit_num = USERKEY0_BIT_NUM,
		.word_num = USERKEY0_WORD_NUM,
		.prt_bit = EFUSE_STA_USERKEY0_PRT,
		.verify_mode = HAMMING,
	},
	[12] = {
		.seg_name = "USERKEY1",
		.seg_id = USERKEY1,
		.word_address = USERKEY1_WORD_ADDR,
		.begin_align = USERKEY1_BEGIN_ALIGN,
		.end_align = USERKEY1_END_ALIGN,
		.bit_num = USERKEY1_BIT_NUM,
		.word_num = USERKEY1_WORD_NUM,
		.prt_bit = EFUSE_STA_USERKEY1_PRT,
		.verify_mode = HAMMING,
	},
	[13] = {
		.seg_name = "NKU",
		.seg_id = NKU,
		.word_address = NKU_WORD_ADDR,
		.begin_align = NKU_BEGIN_ALIGN,
		.end_align = NKU_END_ALIGN,
		.bit_num = NKU_BIT_NUM,
		.word_num = NKU_WORD_NUM,
		.prt_bit = EFUSE_STA_NKU_PRT,
		.verify_mode = HAMMING,
	},
};

void jz_efuse_id_read(int is_chip_id, uint32_t *buf);

#endif
