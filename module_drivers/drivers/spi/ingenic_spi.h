#ifndef __LINUX_SPI_JZ_H
#define __LINUX_SPI_JZ_H

#include <linux/interrupt.h>
#include <linux/dmaengine.h>

#define NR_DMA_CHANNELS	8

#define CH_DSA	0x00
#define CH_DTA	0x04
#define CH_DTC	0x08
#define CH_DRT	0x0C
#define CH_DCS	0x10
#define CH_DCM	0x14
#define CH_DDA	0x18
#define CH_DSD	0x1C

#define TCSM	0x2000

#define DMAC	0x1000
#define DIRQP	0x1004
#define DDR	0x1008
#define DDRS	0x100C
#define DMACP	0x101C
#define DSIRQP	0x1020
#define DSIRQM	0x1024
#define DCIRQP	0x1028
#define DCIRQM	0x102C

/* MCU of PDMA */
#define DMCS	0x1030
#define DMNMB	0x1034
#define DMSMB	0x1038
#define DMINT	0x103C

/* MCU of PDMA */
#define DMINT_S_IP      BIT(17)
#define DMINT_N_IP      BIT(16)

#define DMAC_HLT	BIT(3)
#define DMAC_AR		BIT(2)

#define DCS_NDES	BIT(31)
#define DCS_AR		BIT(4)
#define DCS_TT		BIT(3)
#define DCS_HLT		BIT(2)
#define DCS_CTE		BIT(0)

#define DCM_SAI		BIT(23)
#define DCM_DAI		BIT(22)
#define DCM_SP_MSK	(0x3 << 14)
#define DCM_SP_32	DCM_SP_MSK
#define DCM_SP_16	BIT(15)
#define DCM_SP_8	BIT(14)
#define DCM_DP_MSK	(0x3 << 12)
#define DCM_DP_32	DCM_DP_MSK
#define DCM_DP_16	BIT(13)
#define DCM_DP_8	BIT(12)
#define DCM_TSZ_MSK	(0x7 << 8)
#define DCM_TSZ_SHF	8
#define DCM_STDE	BIT(2)
#define DCM_TIE		BIT(1)
#define DCM_LINK	BIT(0)

#define DCM_CH1_SRC_TCSM    (0x0 << 26)
#define DCM_CH1_SRC_NEMC    (0x1 << 26)
#define DCM_CH1_SRC_DDR     (0x2 << 26)

#define DCM_CH1_DST_TCSM    (0x0 << 24)
#define DCM_CH1_DST_NEMC    (0x1 << 24)
#define DCM_CH1_DST_DDR     (0x2 << 24)

#define DCM_CH1_DDR_TO_NAND  (DCM_CH1_SRC_DDR  | DCM_CH1_DST_NEMC)
#define DCM_CH1_NAND_TO_DDR  (DCM_CH1_SRC_NEMC | DCM_CH1_DST_DDR)

#define DCM_CH1_TCSM_TO_NAND (DCM_CH1_SRC_TCSM | DCM_CH1_DST_NEMC)
#define DCM_CH1_NAND_TO_TCSM (DCM_CH1_SRC_NEMC | DCM_CH1_DST_TCSM)

#define DCM_CH1_TCSM_TO_DDR  (DCM_CH1_SRC_TCSM | DCM_CH1_DST_DDR)
#define DCM_CH1_DDR_TO_TCSM  (DCM_CH1_SRC_DDR  | DCM_CH1_DST_TCSM)

#define MCU_MSG_TYPE_NORMAL	0x1
#define MCU_MSG_TYPE_INTC	0x2
#define MCU_MSG_TYPE_INTC_MASKA	0x3

	enum jzdma_req_type {
#define _RTP(NAME) JZDMA_REQ_##NAME##_TX,JZDMA_REQ_##NAME##_RX
		JZDMA_REQ_RESERVED0 = 0x03,
		_RTP(I2S1),
		_RTP(I2S0),
		JZDMA_REQ_AUTO_TXRX = 0x08,
		JZDMA_REQ_SADC_RX,
		JZDMA_REQ_RESERVED1 = 0x0b,
		_RTP(UART4),
		_RTP(UART3),
		_RTP(UART2),
		_RTP(UART1),
		_RTP(UART0),
		_RTP(SSI0),
		_RTP(SSI1),
		_RTP(MSC0),
		_RTP(MSC1),
		_RTP(MSC2),
		_RTP(PCM0),
		_RTP(PCM1),
		_RTP(I2C0),
		_RTP(I2C1),
		_RTP(I2C2),
		_RTP(I2C3),
		_RTP(I2C4),
		_RTP(DES),
#undef _RTP
	};

enum jzdma_type {
	JZDMA_REQ_INVAL = 0,
#define _RTP(NAME) JZDMA_REQ_##NAME = JZDMA_REQ_##NAME##_TX
	_RTP(I2S1),
	_RTP(I2S0),
	JZDMA_REQ_AUTO = JZDMA_REQ_AUTO_TXRX,
	JZDMA_REQ_SADC = JZDMA_REQ_SADC_RX,
	_RTP(UART4),
	_RTP(UART3),
	_RTP(UART2),
	_RTP(UART1),
	_RTP(UART0),
	_RTP(SSI0),
	_RTP(SSI1),
	_RTP(MSC0),
	_RTP(MSC1),
	_RTP(MSC2),
	_RTP(PCM0),
	_RTP(PCM1),
	_RTP(I2C0),
	_RTP(I2C1),
	_RTP(I2C2),
	_RTP(I2C3),
	_RTP(I2C4),
	_RTP(DES),
	JZDMA_REQ_NAND0 = JZDMA_REQ_AUTO_TXRX | (1 << 16),
	JZDMA_REQ_NAND1 = JZDMA_REQ_AUTO_TXRX | (2 << 16),
	JZDMA_REQ_NAND2 = JZDMA_REQ_AUTO_TXRX | (3 << 16),
	JZDMA_REQ_NAND3 = JZDMA_REQ_AUTO_TXRX | (4 << 16),
	JZDMA_REQ_NAND4 = JZDMA_REQ_AUTO_TXRX | (5 << 16),
	TYPE_MASK = 0xffff,
#undef _RTP
};

#define GET_MAP_TYPE(type) (type & (TYPE_MASK))

struct jzdma_platform_data {
	int irq_base;
	int irq_end;
	enum jzdma_type map[NR_DMA_CHANNELS];
};


struct jzdma_master;

struct dma_desc {
	unsigned long dcm;
	dma_addr_t dsa;
	dma_addr_t dta;
	unsigned long dtc;
	unsigned long sd;
	unsigned long drt;
	unsigned long reserved[2];
};

struct jzdma_channel {
	int			id;
	int			residue;
	struct dma_chan		chan;
	enum jzdma_type		type;
	struct dma_async_tx_descriptor	tx_desc;
	dma_cookie_t		last_completed;
	dma_cookie_t		last_good;
	struct tasklet_struct	tasklet;
	spinlock_t		lock;
#define CHFLG_SLAVE		BIT(0)
	unsigned short		flags;
	unsigned short		status;
	unsigned long		dcs_saved;
	struct dma_desc		*desc;
	dma_addr_t		desc_phys;
	unsigned short		desc_nr;
	unsigned short		desc_max;
	struct scatterlist	*sgl;
	unsigned long		sg_len;
	unsigned short		last_sg;
#define FAKECYCLIC_ACTIVE (1 << 15)
#define FAKECYCLIC_POSSIBLE (1 << 14)
#define FAKECYCLIC_IDX 0x3fff
	unsigned short fake_cyclic;
	unsigned long		tx_dcm_def;
	unsigned long		rx_dcm_def;
	struct dma_slave_config	*config;
	void __iomem		*iomem;
	struct jzdma_master	*master;
};

enum channel_status {
	STAT_STOPED,STAT_SUBED,STAT_PREPED,STAT_RUNNING,
};

enum ssi_clk_flag {
	SFC_CLK, SSI_CLK,
};

struct jzdma_master {
	struct device		*dev;
	void __iomem		*iomem;
	struct clk		*clk;
	int			irq;
	int                     irq_pdmam;   /* irq_pdmam for PDMAM irq */
	struct dma_device	dma_device;
	enum jzdma_type		*map;
	struct irq_chip		irq_chip;
	struct jzdma_channel	channel[NR_DMA_CHANNELS];
};

static inline struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static inline struct jzdma_channel *to_jzdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct jzdma_channel, chan);
}

void jzdma_dump(struct dma_chan *chan);

/*SPI NOR FLASH Instructions*/
#define CMD_WREN        0x06 /* Write Enable */
#define CMD_WRDI        0x04 /* Write Disable */
#define CMD_RDSR        0x05 /* Read Status Register */
#define CMD_RDSR_1      0x35 /* Read Status1 Register */
#define CMD_RDSR_2      0x15 /* Read Status2 Register */
#define CMD_WRSR        0x01 /* Write Status Register */
#define CMD_WRSR_1      0x31 /* Write Status1 Register */
#define CMD_WRSR_2      0x11 /* Write Status2 Register */
#define CMD_READ        0x03 /* Read Data */
#define CMD_DUAL_READ   0x3b /* DUAL Read Data */
#define CMD_QUAD_READ   0x6b /* QUAD Read Data */
#define CMD_QUAD_IO_FAST_READ   0xeb /* QUAD FAST Read Data */
#define CMD_QUAD_IO_WORD_FAST_READ   0xe7 /* QUAD IO WORD Read Data */
#define CMD_FAST_READ   0x0B /* Read Data at high speed */
#define CMD_PP          0x02 /* Page Program(write data) */
#define CMD_QPP         0x32 /* QUAD Page Program(write data) */
#define CMD_SE          0x20 /* Sector Erase */
#define CMD_BE_32K      0x52 /* Block Erase */
#define CMD_BE_64K		0XD8 /* Block Erase */
#define CMD_CE          0xC7 /* Bulk or Chip Erase */
#define CMD_DP          0xB9 /* Deep Power-Down */
#define CMD_RES         0xAB /* Release from Power-Down and Read Electronic Signature */
#define CMD_REMS        0x90 /* Read Manufacture ID/ Device ID */
#define CMD_RDID        0x9F /* Read Identification */
#define CMD_NON         0x00 /* Read Identification */
#define CMD_EN4B		0xB7 /* Enter 4 bytes address mode */
#define CMD_EX4B		0xE9 /* Exit 4 bytes address mode */

/* SPI NAND Flash Instructions */
//#define CMD_WREN    0x06    /* Write Enable */
//#define CMD_WRDI    0x04    /* Write Disable */
#define CMD_G_FEATURE   0x0F    /* Get Feature */
#define CMD_S_FEATURE   0x1F    /* Set Feature */
#define CMD_R_PAGE  0x13    /* Page Read (to cache) */
#define CMD_R_CACHE 0x03    /* Read From Cache */
#define CMD_P_LOAD  0x02    /* Program Load(write data to cache) */
#define CMD_P_EXECUTE   0x10    /* Program Execute(write data) */
#define CMD_E_BLOCK 0xD8    /* Block Erase */
//#define CMD_RDID    0x9F    /* Read ID */
#define CMD_RESET   0xFF    /* Reset */

/*for sfc register config*/
#define TRAN_SPI_QUAD   (0x5 )
#define TRAN_SPI_IO_QUAD   (0x6 )


#define SPIFLASH_PARAMER_OFFSET 0x3c00
#define SPI_NORFLASH_PART_OFFSET 0x3c6c

#define NORFLASH_PART_RW		0
#define NORFLASH_PART_WO		1
#define NORFLASH_PART_RO		2

#define NOR_MAJOR_VERSION_NUMBER	1
#define NOR_MINOR_VERSION_NUMBER	0
#define NOR_REVERSION_NUMBER	0
#define NOR_VERSION		(NOR_MAJOR_VERSION_NUMBER | (NOR_MINOR_VERSION_NUMBER << 8) | (NOR_REVERSION_NUMBER << 16))

#define NOR_MAGIC	0x726f6e	//ascii "nor"

struct sfc_nor_info {
	u8 cmd;
	u8 addr_len;
	u8 daten;
	u8 pollen;
	u8 M7_0;// some cmd must be send the M7-0
	u8 dummy_byte;
	u8 dma_mode;
};

struct spi_nor_block_info {
	u32 blocksize;
	u8 cmd_blockerase;
	/* MAX Busytime for block erase, unit: ms */
	u32 be_maxbusy;
};

struct spi_quad_mode {
	u8 dummy_byte;
	u8 RDSR_CMD;
	u8 WRSR_CMD;
	unsigned int RDSR_DATE;//the data is write the spi status register for QE bit
	unsigned int RD_DATE_SIZE;//the data is write the spi status register for QE bit
	unsigned int WRSR_DATE;//this bit should be the flash QUAD mode enable
	unsigned int WD_DATE_SIZE;//the data is write the spi status register for QE bit
	u8 cmd_read;
	u8 sfc_mode;
};

#define SIZEOF_NAME         32
#define NOR_PART_NUM	10
struct norflash_params {
	char name[SIZEOF_NAME];
	u32 pagesize;
	u32 sectorsize;
	u32 chipsize;
	u32 erasesize;
	int id;
	/* Flash Address size, unit: Bytes */
	int addrsize;

	/* MAX Busytime for page program, unit: ms */
	u32 pp_maxbusy;
	/* MAX Busytime for sector erase, unit: ms */
	u32 se_maxbusy;
	/* MAX Busytime for chip erase, unit: ms */
	u32 ce_maxbusy;

	/* Flash status register num, Max support 3 register */
	int st_regnum;
	/* Some NOR flash has different blocksize and block erase command,
	 *          * One command with One blocksize. */
	struct spi_nor_block_info block_info;
	struct spi_quad_mode quad_mode;
};

struct ingenic_priv {
	uint32_t clk_flag;
};

struct nor_partition {
	char name[SIZEOF_NAME];
	uint32_t size;
	uint32_t offset;
	uint32_t mask_flags;//bit0-1 mask the partition RW mode, 0:RW  1:WO  2:RO
	uint32_t manager_mode;
};

struct norflash_partitions {
	struct nor_partition nor_partition[NOR_PART_NUM];
	uint32_t num_partition_info;
};

struct nor_sharing_params {
	uint32_t magic;
	uint32_t version;
	struct norflash_params norflash_params;
	struct norflash_partitions norflash_partitions;
};


struct spi_nor_platform_data {
	char *name;
	u32 pagesize;
	u32 sectorsize;
	u32 chipsize;
	u32 erasesize;
	int id;
	/* Some NOR flash has different blocksize and block erase command,
	 *          * One command with One blocksize. */
	struct spi_nor_block_info *block_info;
	int num_block_info;

	/* Flash Address size, unit: Bytes */
	int addrsize;

	/* MAX Busytime for page program, unit: ms */
	u32 pp_maxbusy;
	/* MAX Busytime for sector erase, unit: ms */
	u32 se_maxbusy;
	/* MAX Busytime for chip erase, unit: ms */
	u32 ce_maxbusy;

	/* Flash status register num, Max support 3 register */
	int st_regnum;
	struct mtd_partition *mtd_partition;
	struct spi_quad_mode *quad_mode;
	int num_partition_info;
};

struct ingenic_spi_info {
	u8	chnl;				/* the chanel of SSI controller */
	u16	bus_num;			/* spi_master.bus_num */
	u8 src_clk;			/* source clock: 1---sfcclk;0---exclk */
	unsigned long	max_clk;
	unsigned long	board_size;		/* spi_master.num_chipselect */
	struct spi_board_info	*board_info; 	/* link to spi devices info */
	u32	 num_chipselect;
	u32	 allow_cs_same;
	unsigned int chipselects[2];

	void (*set_cs)(struct ingenic_spi_info *spi, u8 cs,unsigned int pol); /* be defined by spi devices driver user */
	void (*pins_config)(void);		/* configure spi function pins (CLK,DR,RT) by user if need. */
};

struct jz_sfc_info {
	u8	chnl;				/* the chanel of SSI controller */
	u16	bus_num;			/* spi_master.bus_num */
	unsigned is_pllclk:1;			/* source clock: 1---pllclk;0---exclk */
	unsigned long	board_size;		/* spi_master.num_chipselect */
	u32	 num_chipselect;
	u32	 allow_cs_same;
	void  *board_info;
	u32  board_info_size;
};
struct jz_sfc_nand_info{
	u8      chnl;                           /* the chanel of SSI controller */
	u16     bus_num;                        /* spi_master.bus_num */
	unsigned is_pllclk:1;                   /* source clock: 1---pllclk;0---exclk */
	unsigned long   board_size;             /* spi_master.num_chipselect */
	u32      num_chipselect;
	u32      allow_cs_same;
	void  *board_info;
	u32  board_info_size;
};
#define SPL_TYPE_FLAG_LEN 6
struct jz_spi_support {
	unsigned int id_manufactory;
	unsigned char id_device;
	char name[SIZEOF_NAME];
	int page_size;
	int oobsize;
	int sector_size;
	int block_size;
	int size;
	int page_num;

	/* MAX Busytime for page read, unit: us */
	u32 tRD_maxbusy;
	/* MAX Busytime for Page Program, unit: us */
	u32 tPROG_maxbusy;
	/* MAX Busytime for block erase, unit: us */
	u32 tBERS_maxbusy;

	unsigned short column_cmdaddr_bits;/* read from cache ,the bits of cmd + addr */

};

struct jz_spi_support_from_burner {
	unsigned int chip_id;
	unsigned char id_device;
	char name[32];
	int page_size;
	int oobsize;
	int sector_size;
	int block_size;
	int size;
	int page_num;
	uint32_t tRD_maxbusy;
	uint32_t tPROG_maxbusy;
	uint32_t tBERS_maxbusy;
	unsigned short column_cmdaddr_bits;

};
struct jz_spinand_partition {
	char name[32];         /* identifier string */
	uint32_t size;          /* partition size */
	uint32_t offset;        /* offset within the master MTD space */
	u_int32_t mask_flags;       /* master MTD flags to mask out for this partition */
	u_int32_t manager_mode;         /* manager_mode mtd or ubi */
};
struct get_chip_param {
	int version;
	int flash_type;
	int para_num;
	struct jz_spi_support_from_burner *addr;
	int partition_num;
	struct jz_spinand_partition *partition;
};
struct jz_spi_nand_platform_data {
	struct jz_spi_support *jz_spi_support;
	int num_spi_flash;
	struct mtd_partition *mtd_partition;
	int num_partitions;
};

/*************************************************************************
 * SSI (Synchronous Serial Interface)
 *************************************************************************/
/* n = 0, 1 (SSI0, SSI1) */
#define	SSI_DR			0x000
#define	SSI_CR0			0x004
#define	SSI_CR1			0x008
#define	SSI_SR			0x00C
#define	SSI_ITR			0x010
#define	SSI_ICR			0x014
#define	SSI_GR			0x018
#define	SSI_RCNT		0x01C

/* SSI Data Register (SSI_DR) */
#define	DR_GPC_BIT		0
#define	DR_GPC_MASK		(0x1ff << SSI_DR_GPC_BIT)

/* SSI Control Register 0 (SSI_CR0) */
#define CR0_TENDIAN_BIT		18
#define CR0_TENDIAN_MASK	(3 << CR0_TENDIAN_BIT)
#define CR0_RENDIAN_BIT		16
#define CR0_RENDIAN_MASK	(3 << CR0_RENDIAN_BIT)
#define CR0_SSIE		(1 << 15)
#define CR0_TIE			(1 << 14)
#define CR0_RIE			(1 << 13)
#define CR0_TEIE		(1 << 12)
#define CR0_REIE		(1 << 11)
#define CR0_LOOP		(1 << 10)
#define CR0_RFINE		(1 << 9)
#define CR0_RFINC		(1 << 8)
#define CR0_EACLRUN		(1 << 7) /* hardware auto clear underrun when TxFifo no empty */
#define CR0_FSEL		(1 << 6)
#define CR0_VRCNT		(1 << 4)
#define CR0_TFMODE		(1 << 3)
#define CR0_TFLUSH		(1 << 2)
#define CR0_RFLUSH		(1 << 1)
#define CR0_DISREV		(1 << 0)

/* SSI Control Register 1 (SSI_CR1) */
#define CR1_FRMHL_BIT		30
#define CR1_FRMHL_MASK		(0x3 << CR1_FRMHL_BIT)
#define CR1_FRMHL_CELOW_CE2LOW	(0 << CR1_FRMHL_BIT) /* SSI_CE_ is low valid and SSI_CE2_ is low valid */
#define CR1_FRMHL_CEHIGH_CE2LOW	(1 << CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is low valid */
#define CR1_FRMHL_CELOW_CE2HIGH	(2 << CR1_FRMHL_BIT) /* SSI_CE_ is low valid  and SSI_CE2_ is high valid */
#define CR1_FRMHL_CEHIGH_CE2HIGH	(3 << CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is high valid */
#define CR1_TFVCK_BIT		28
#define CR1_TFVCK_MASK		(0x3 << CR1_TFVCK_BIT)
  #define CR1_TFVCK_0		  (0 << CR1_TFVCK_BIT)
  #define CR1_TFVCK_1		  (1 << CR1_TFVCK_BIT)
  #define CR1_TFVCK_2		  (2 << CR1_TFVCK_BIT)
  #define CR1_TFVCK_3		  (3 << CR1_TFVCK_BIT)
#define CR1_TCKFI_BIT		26
#define CR1_TCKFI_MASK		(0x3 << CR1_TCKFI_BIT)
  #define CR1_TCKFI_0		  (0 << CR1_TCKFI_BIT)
  #define CR1_TCKFI_1		  (1 << CR1_TCKFI_BIT)
  #define CR1_TCKFI_2		  (2 << CR1_TCKFI_BIT)
  #define CR1_TCKFI_3		  (3 << CR1_TCKFI_BIT)
#define CR1_ITFRM		(1 << 24)
#define CR1_UNFIN		(1 << 23)
#define CR1_FMAT_BIT		20
#define CR1_FMAT_MASK		(0x3 << CR1_FMAT_BIT)
  #define CR1_FMAT_SPI		  (0 << CR1_FMAT_BIT) /* Motorola¡¯s SPI format */
  #define CR1_FMAT_SSP		  (1 << CR1_FMAT_BIT) /* TI's SSP format */
  #define CR1_FMAT_MW1		  (2 << CR1_FMAT_BIT) /* National Microwire 1 format */
  #define CR1_FMAT_MW2		  (3 << CR1_FMAT_BIT) /* National Microwire 2 format */
#define CR1_TTRG_BIT		16 /* SSI1 TX trigger */
#define CR1_TTRG_MASK		(0xf << CR1_TTRG_BIT)
#define CR1_MCOM_BIT		12
#define CR1_MCOM_MASK		(0xf << CR1_MCOM_BIT)
//  #define CR1_MCOM_BIT(NO)	  (##NO## << CR1_MCOM_BIT) /* N-bit command selected */
#define CR1_RTRG_BIT		8 /* SSI RX trigger */
#define CR1_RTRG_MASK		(0xf << CR1_RTRG_BIT)
#define CR1_FLEN_BIT		3
#define CR1_FLEN_MASK		(0x1f << CR1_FLEN_BIT)
  #define CR1_FLEN_2BIT		  (0x0 << CR1_FLEN_BIT)
#define CR1_PHA			(1 << 1)
#define CR1_POL			(1 << 0)

/* SSI Status Register (SSI_SR) */
#define SR_TFIFONUM_BIT		16
#define SR_TFIFONUM_MASK	(0xff << SR_TFIFONUM_BIT)
#define SR_RFIFONUM_BIT		8
#define SR_RFIFONUM_MASK	(0xff << SR_RFIFONUM_BIT)
#define SR_END			(1 << 7)
#define SR_BUSY			(1 << 6)
#define SR_TFF			(1 << 5)
#define SR_RFE			(1 << 4)
#define SR_TFHE			(1 << 3)
#define SR_RFHF			(1 << 2)
#define SR_UNDR			(1 << 1)
#define SR_OVER			(1 << 0)

/* SSI Interval Time Control Register (SSI_ITR) */
#define	ITR_CNTCLK		(1 << 15)
#define ITR_IVLTM_BIT		0
#define ITR_IVLTM_MASK		(0x7fff << ITR_IVLTM_BIT)



#define R_MODE			0x1
#define W_MODE			0x2
#define RW_MODE			(R_MODE | W_MODE)

#define R_DMA			0x4
#define W_DMA			0x8
#define RW_DMA			(R_DMA |W_DMA)

#define SPI_DMA_ACK		0x1

#define SPI_DMA_ERROR  		-3
#define SPI_CPU_ERROR		-4

#define SPI_COMPLETE		5

#define INGENIC_SSI_MAX_FIFO_ENTRIES 	128
#define INGENIC_SSI_DMA_BURST_LENGTH 	16

#define FIFO_W8			8
#define FIFO_W16		16
#define FIFO_W32		32

#define SPI_BITS_8		8
#define SPI_BITS_16		16
#define SPI_BITS_32		32

#define SPI_8BITS		1
#define SPI_16BITS		2
#define SPI_32BITS		4


/* tx rx threshold from 0x0 to 0xF */
#define SSI_FULL_THRESHOLD		0xF
#define SSI_TX_FIFO_THRESHOLD		0x1
#define SSI_RX_FIFO_THRESHOLD		(SSI_FULL_THRESHOLD - SSI_TX_FIFO_THRESHOLD)
#define SSI_SAFE_THRESHOLD		0x1

#define CPU_ONCE_BLOCK_ENTRIES 		((SSI_FULL_THRESHOLD-SSI_TX_FIFO_THRESHOLD)*8)

#define MAX_SSI_INTR		10000

#define MAX_SSICDR			63
#define MAX_CGV				255

#define SSI_DMA_FASTNESS_CHNL 	 0   // SSI controller [n] FASTNESS when probe();

#define JZ_NEW_CODE_TYPE

#define BUFFER_SIZE	PAGE_SIZE

#define CONFIG_DMA_ENGINE 1

#define SUSPND    (1<<0)
#define SPIBUSY   (1<<1)
#define RXBUSY    (1<<2)
#define TXBUSY    (1<<3)

struct ingenic_spi {
	/* bitbang has to be first */
	struct spi_bitbang	bitbang;
	struct clk		*clk_gate;
	struct clk		*clk_cgu;
	unsigned int		clk_flag;
	unsigned int		set_clk_flag;
	struct completion	done;
	struct completion	done_rx;
	struct completion	done_tx_dma;
	struct completion	done_rx_dma;

	spinlock_t		lock;
	spinlock_t		txrx_lock;

	unsigned int		state;

	u8			chnl;
	u8			rw_mode;
	u8			spi_mode;
	u8			use_dma;
	u8			last_transfer;

	u8			bits_per_word;		/*8 or 16 (or 32)*/
	u8			transfer_unit_size;	/* 1 or 2 (or 4) */
	u8			tx_trigger;		/* 0-128 */
	u8			rx_trigger;		/* 0-128 */
	u8			dma_tx_unit;		/* 1 or 2 or 4 or 16 or 32*/
	u8			dma_rx_unit;		/* 1 or 2 or 4 or 16 or 32*/
	u8			txfifo_width;
	u8			rxfifo_width;
	u32			fifodepth;
	u32			t_count;
	u32			r_count;

	/* data buffers */
	const u8		*tx;
	u8			*rx;

	/* temp buffers */
	void			*buffer;
	dma_addr_t		buffer_dma;

	void __iomem		*iomem;
	unsigned long		phys;
	int			 irq;
	u32			 len;
	u32			 rlen;	  /* receive len */
	u32			 count;   /* sent count */
	enum jzdma_type     	 dma_type;
	u32			 dma_flag;

	void			(*set_cs)(struct ingenic_spi_info *spi, u8 cs, unsigned int pol);

	/* functions to deal with different size buffers */
	u32 (*get_rx) (struct ingenic_spi *);
	u32 (*get_tx) (struct ingenic_spi *);
	int (*txrx_bufs)(struct spi_device *spi, struct spi_transfer *t);
	irqreturn_t (*irq_callback)(struct ingenic_spi *);

#ifdef CONFIG_DMA_ENGINE
	struct dma_chan 	*txchan;
	struct dma_chan 	*rxchan;
	struct scatterlist  	*sg_rx;	/* I/O scatter list */
	struct scatterlist  	*sg_tx;	/* I/O scatter list */
#endif

	unsigned long max_clk;
	unsigned long spi_clk;

	struct ingenic_intr_cnt *g_ingenic_intr;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct device		*dev;
	struct ingenic_spi_info *pdata;
	struct ingenic_priv     *priv;
//	struct spi_board_info *pdata;
};


struct ingenic_intr_cnt{
	int dma_tx_cnt;
	int dma_rx_cnt;
	int ssi_intr_cnt;
	int max_ssi_intr;
	int ssi_txi;
	int ssi_rxi;
	int ssi_eti;
	int ssi_eri;
	int ssi_rlen;
	int dma_tx_err;
	int dma_tx_end;
	int dma_rx_err;
	int dma_rx_end;
};
/* the max number of spi devices */
#define MAX_SPI_DEVICES				10
#define MAX_SPI_HOST				2

#define INGENIC_SPI_ID_INVALID(ssi_id) ( ((ssi_id) < 0) || ((ssi_id) > (MAX_SPI_HOST - 1)) )

#define MAX_SPI_CHIPSELECT_NUM 		MAX_GPIO_NUM


static inline void spi_writel(struct ingenic_spi *spi, unsigned short offset, u32 value)
{
	writel(value, spi->iomem + offset);
}

static inline u32 spi_readl(struct ingenic_spi *spi,
				      unsigned short offset)
{
	return readl(spi->iomem + offset);
}

static inline void set_frmhl(struct ingenic_spi *spi, unsigned int frmhl)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_FRMHL_MASK) | frmhl;
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void set_spi_clock_phase(struct ingenic_spi *spi, unsigned int cpha)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_PHA) | (cpha ? CR1_PHA : 0);
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void set_spi_clock_polarity(struct ingenic_spi *spi,
					  unsigned int cpol)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_POL) | (cpol ? CR1_POL : 0);
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void set_tx_msb(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_TENDIAN_MASK;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void set_tx_lsb(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= (tmp & ~CR0_TENDIAN_MASK) | (0x3 << CR0_TENDIAN_BIT);
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void set_rx_msb(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_RENDIAN_MASK;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void set_rx_lsb(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= (tmp & ~CR0_RENDIAN_MASK) | (0x3 << CR0_RENDIAN_BIT);
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_loopback(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_LOOP;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_loopback(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_LOOP;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void transmit_data(struct ingenic_spi *spi, u32 value)
{
	spi_writel(spi, SSI_DR, value);
}

static inline void set_frame_length(struct ingenic_spi *spi, u32 len)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_FLEN_MASK) | (((len) - 2) << CR1_FLEN_BIT);
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void set_tx_trigger(struct ingenic_spi *spi, u32 val)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_TTRG_MASK) | ((val)/8) << CR1_TTRG_BIT;
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void set_rx_trigger(struct ingenic_spi *spi, u32 val)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp = (tmp & ~CR1_RTRG_MASK) | ((val)/8) << CR1_RTRG_BIT;
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void enable_txfifo_half_empty_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_TIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_txfifo_half_empty_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_TIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_rxfifo_half_full_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_RIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_rxfifo_half_full_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_RIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_tx_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_TIE | CR0_TEIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_tx_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~(CR0_TIE | CR0_TEIE);
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_rx_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_RIE | CR0_REIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_rx_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~(CR0_RIE | CR0_REIE);
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_tx_error_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_TEIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_tx_error_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_TEIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void enable_rx_error_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_REIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_rx_error_intr(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_REIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void underrun_auto_clear(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_EACLRUN;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void clear_errors(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_SR);
	tmp &= ~(SR_UNDR | SR_OVER);
	spi_writel(spi, SSI_SR, tmp);
}

static inline void set_spi_format(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp &= ~CR1_FMAT_MASK;
	tmp |= CR1_FMAT_SPI;
	tmp &= ~(CR1_TFVCK_MASK | CR1_TCKFI_MASK);
	tmp |= (CR1_TFVCK_0 | CR1_TCKFI_0);
//	tmp |= (CR1_TFVCK_1 | CR1_TCKFI_1);
//	tmp |= (CR1_TFVCK_2 | CR1_TCKFI_2);
//	tmp |= (CR1_TFVCK_3 | CR1_TCKFI_3);
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void enable_receive(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_DISREV;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_receive(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_DISREV;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void flush_fifo(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_TFLUSH | CR0_RFLUSH;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void disable_receive_continue(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~(CR0_RFINE | CR0_RFINC);
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void finish_transmit(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp &= ~CR1_UNFIN;
	spi_writel(spi, SSI_CR1, tmp);
}

static inline void start_transmit(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR1);
	tmp |= CR1_UNFIN;
	spi_writel(spi, SSI_CR1, tmp);
}

static inline int rxfifo_empty(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_RFE;
}

static inline int ssi_busy(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_BUSY;
}

static inline void ssi_disable(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_SSIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void ssi_enable(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_SSIE;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline u32 get_rxfifo_count(struct ingenic_spi *spi)
{
	return (spi_readl(spi, SSI_SR) & SR_RFIFONUM_MASK) >> SR_RFIFONUM_BIT;
}

static inline u32 transfer_busy(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_BUSY;
}

static inline void flush_txfifo(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_TFLUSH;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void flush_rxfifo(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_RFLUSH;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline int ssi_underrun(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_UNDR;
}

static inline int ssi_overrun(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_OVER;
}

static inline int ssi_transfer_end(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_END;
}

static inline int tx_error_intr(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_CR0) & CR0_TEIE;
}

static inline int rx_error_intr(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_CR0) & CR0_REIE;
}

static inline int rxfifo_half_full(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_RFHF;
}

static inline int txfifo_half_empty(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_SR) & SR_TFHE;
}

static inline int txfifo_half_empty_intr(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_CR0) & CR0_TIE;
}

static inline int rxfifo_half_full_intr(struct ingenic_spi *spi)
{
	return spi_readl(spi, SSI_CR0) & CR0_RIE;
}

static inline void select_ce(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp &= ~CR0_FSEL;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void select_ce2(struct ingenic_spi *spi)
{
	u32 tmp;
	tmp = spi_readl(spi, SSI_CR0);
	tmp |= CR0_FSEL;
	spi_writel(spi, SSI_CR0, tmp);
}

static inline void dump_spi_reg(struct ingenic_spi *spi)
{
//	printk("SSI_DR	:%08x\n", spi_readl(spi, SSI_DR	));
	printk("SSI_CR0	:%08x\n", spi_readl(spi, SSI_CR0 ));
	printk("SSI_CR1	:%08x\n", spi_readl(spi, SSI_CR1 ));
	printk("SSI_SR	:%08x\n", spi_readl(spi, SSI_SR	));
	printk("SSI_ITR	:%08x\n", spi_readl(spi, SSI_ITR ));
	printk("SSI_ICR	:%08x\n", spi_readl(spi, SSI_ICR ));
	printk("SSI_GR	:%08x\n", spi_readl(spi, SSI_GR	));
	printk("SSI_RCNT:%08x\n", spi_readl(spi, SSI_RCNT));
}

#endif /* __LINUX_SPI_JZ_H */
