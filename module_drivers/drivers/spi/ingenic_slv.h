#ifndef __SPI_SLV_H__
#define __SPI_SLV_H__

#include <linux/miscdevice.h>
#include <linux/dmaengine.h>

/*SSI register*/
#define CTRLR0	0x0	/* Control Register 0 */
#define SPI_FRF_SSF	(0<<21) /* Standard SPI Format */
#define	DFS_MASK_BITS	((0x1f)<<16)	/* Date Frame Size */
#define	DFS_32_BITS	((32-1)<<16)	/* Date Frame Size, 32 bits */
#define	DFS_16_BITS	((16-1)<<16)	/* Date Frame Size, 16 bits */
#define	DFS_8_BITS	((8-1)<<16)		/* Date Frame Size, 8 bits */
#define	DCS_MASK_BITS	((0xf)<<12)	/* Control Frame Size */
#define	SRL_NOMAL	(0<<11) /* Shift Register Loop,Nomal Mode Operation */
#define	SRL_TEST	(1<<11) /* Shift Register Loop,Test Mode Operation */
#define SLV_OE	(0<<10) /* Slave Output Enable */
#define TMOD_T_R	(0<<8) /* Transmit & Receive */
#define TMOD_T	(1<<8) /* Transmit Only */
#define TMOD_R	(2<<8) /* Receive Only */
#define	SCPOL_L	(0<<7) /* Serial Clock Polarity,Inactive state of serial clock is low */
#define	SCPOL_H	(1<<7) /* Serial Clock Polarity,Inactive state of serial clock is high */
#define SCPH_M	(0<<6) /* Serial Clock Phase,Serial clock toggles in middle of first data bit */
#define SCPH_S	(1<<6) /* Serial Clock Phase,Serial clock toggles at start of first data bit */
#define FRF_SPI	(0<<4) /* Frame Format, Motorola SPI */

#define CTRLR1	0x4 /* Control Register 1 */

#define	SSIENR	0x8 /* SSI Enable Register */
#define SSI_EN	0x1 /* SSI Enable */
#define SSI_DISEN	0x0 /* SSI Disenable */

#define	MWCR	0xc /* Microwire Control Register */

#define	SER		0x10 /* Slave Enable Register */

#define	BAUDR	0x14 /* Baud Rate Select */

#define	TXFTLR	0x18 /* Transmit FIFO Threshold Level */
#define	RXFTLR	0x1c /* Receive FIFO Threshold Level */
#define	TXFLR	0x20 /* Transmit FIFO Level Register */
#define	RXFLR	0x24 /* Receive FIFO Level Register */

#define	SLV_SR		0x28 /* Status Register */
#define SR_TXE	(1<<5) /* Transmission error */
#define SR_RFF	(1<<4) /* Receive FIFO is full */
#define SR_RFNE	(1<<3) /* Receive FIFO is not empty */
#define SR_TFE	(1<<2) /* Transmit FIFO is empty */
#define	SR_TFNF	(1<<1) /* Transmit FIFO is not full */
#define	SR_BUSY	(1<<0) /* SSI_SLV is actively transferring data */

#define	SLV_IMR		0x2c /* Interrupt Mask Register */
#define RXFIM	(1<<4) /* Receive FIFO more than threshold Interrupt Mask */
#define	RXOIM	(1<<3) /* Receive FIFO Overflow Interrupt Mask */
#define	RXUIM	(1<<2) /* Receive FIFO Empty Interrupt Mask */
#define	TXOIM	(1<<1) /* Transmit FIFO Overflow Interrupt Mask */
#define	TXEIM	(1<<0) /* Transmit FIFO less than threshold Interrupt Mask */

#define	SLV_ISR		0x30 /* Interrupt Status Register */
#define	ISR_RXFIS	(1<<4) /* Receive FIFO more than threshold Interrupt Status */
#define ISR_RXOIS	(1<<3) /* Receive FIFO Overflow Interrupt Status */
#define	ISR_RXUIS	(1<<2) /* Receive FIFO empty Interrupt Status */
#define ISR_TXOIS	(1<<1) /* Transmit FIFO Overflow Interrupt Status */
#define ISR_TXEIS	(1<<0) /* Transmit FIFO less than threshold Status */

#define SLV_RISR	0x34 /* Raw Interrupt Status Register */
#define	ISR_RXFIR	(1<<4) /* Receive FIFO more than threshold Interrupt Status */
#define ISR_RXOIR	(1<<3) /* Receive FIFO Overflow Interrupt Status */
#define	ISR_RXUIR	(1<<2) /* Receive FIFO empty Interrupt Status */
#define ISR_TXOIR	(1<<1) /* Transmit FIFO Overflow Interrupt Status */
#define ISR_TXEIR	(1<<0) /* Transmit FIFO less than threshold Status */

#define	TXOICR	0x38 /* Transmit FIFO Overflow Interrupt Clear Register */
#define	RXOICR	0x3c /* Receive FIFO Overflow Interrupt Clear Register */
#define	RXUICR	0x40 /* Receive FIFO Underflow Interrupt Clear Registe */

#define	DMACR	0x4c /* DMA Control Register */
#define	TDMAE	(1<<1) /* Transmit DMA Enable */
#define	RDMAE	(1<<0) /* Receive DMA Enable */

#define	DMATDLR	0x50 /* DMA Transmit Data Level */
#define	DMARDLR	0x54 /* DMA Receive Data Level */

#define	SSLV_IDR		0x58 /* Identification Register */

#define	SSI_COMP_VERSION	0x5c /* coreKit version ID register */

#define	SSLV_DR		0x60 /* Data Registe */

#define	RX_SAMPLE_DLY	0xf0 /* Rx Sample Delay Register */

#define	SPI_CTRLR0	0xf4 /* SPI Control Register*/

#define	RSVD_1	0xf8 /* Reserved location for future use */

#define	RSVD_2	0xfc /* Reserved location for future use */


/* These definitions of structs and macros are used in driver */
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

#define INGENIC_SSI_MAX_FIFO_ENTRIES 	64
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
#define SSI_TX_FIFO_THRESHOLD		0x1
#define SSI_RX_FIFO_THRESHOLD		0x1
#define SSI_SAFE_THRESHOLD		0x1

#define MAX_SSI_INTR		10000

#define MAX_SSICDR			63
#define MAX_CGV				255

#define SSI_DMA_FASTNESS_CHNL 	 0   // SSI controller [n] FASTNESS when probe();

#define INGENIC_NEW_CODE_TYPE

#define BUFFER_SIZE	PAGE_SIZE

#define INGENIC_SPI_LOOP_OPS 0
#define INGENIC_SPI_INT_OPS 1
#define INGENIC_SPI_CPU_OPS 2
#define INGENIC_SPI_DMA_OPS 3

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

struct ingenic_sslv {
	/* bitbang has to be first */
	struct miscdevice mdev;
	struct completion	done;
	struct completion	done_rx;
	struct completion	done_tx_dma;
	struct completion	done_rx_dma;

	struct clk		*clk_gate;
	unsigned int            clk_gate_flag;
	struct clk		*clk;
	unsigned int		clk_flag;
	unsigned int		set_clk_flag;

	spinlock_t		lock;
	spinlock_t		txrx_lock;

	unsigned int		state;

	u8			chnl;
	u8			rw_mode;
	u8			spi_mode;
	u8			use_dma;
	u8			is_first;

	u8			bits_per_word;		/*8 or 16 (or 32)*/
	u8			transfer_unit_size;	/* 1 or 2 (or 4) */
	u8			tx_trigger;		/* 0-128 */
	u8			rx_trigger;		/* 0-128 */
	u8			dma_tx_unit;		/* 1 or 2 or 4 or 16 or 32*/
	u8			dma_rx_unit;		/* 1 or 2 or 4 or 16 or 32*/
	u8			txfifo_width;
	u8			rxfifo_width;
	u32			fifodepth;

	/* data buffers */
	const u8		*tx;
	u8			*rx;

	/* temp buffers */
	void			*txbuffer;
	dma_addr_t		txbuffer_dma;
	void			*rxbuffer;
	dma_addr_t		rxbuffer_dma;

	void __iomem		*iomem;
	unsigned long		phys;
	int			 irq;
	u32			 len;
	int			 rentries;	 /* receive count */
	int			 wentries;   /* sent count */
	enum jzdma_type          dma_type;
	u32			 dma_flag;

	void			(*set_cs)(struct ingenic_spi_info *spi, u8 cs, unsigned int pol);

	/* functions to deal with different size buffers */
	u32 (*get_rx) (struct ingenic_sslv *);
	u32 (*get_tx) (struct ingenic_sslv *);
	int (*txrx_bufs)(struct ingenic_sslv *spi, struct spi_transfer *t);
	irqreturn_t (*irq_callback)(struct ingenic_sslv *);

	struct dma_chan 	*txchan;
	struct dma_chan 	*rxchan;
	struct scatterlist  	*sg_rx;	/* I/O scatter list */
	struct scatterlist  	*sg_tx;	/* I/O scatter list */

	unsigned long src_clk;
	unsigned long spi_clk;

	struct ingenic_intr_cnt *g_ingenic_intr;

	struct resource		*ioarea;
	struct device		*dev;

	struct ingenic_spi_info *pdata;
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

int ingenic_sslv_init_setup(struct ingenic_sslv *);

void inline ss_write(struct ingenic_sslv *hw, unsigned int offset, unsigned int value)
{
	writel(value, (hw->iomem + offset));
}

unsigned int inline ss_read(struct ingenic_sslv *hw, unsigned int offset)
{
	return readl(hw->iomem + offset);
}

/* the functions is about SSI_SLV control register */
void inline set_frame_length(struct ingenic_sslv *hw, unsigned int len)
{
	unsigned int value = ss_read(hw, CTRLR0);
	value &= ~DFS_MASK_BITS;
	value |= ((len - 1) << 16);
	/*err bit, will delete*/
	value &= ~(1<<31);
	ss_write(hw, CTRLR0, value);
}

void inline set_control_length(struct ingenic_sslv *hw, unsigned int len)
{
	unsigned int value = ss_read(hw, CTRLR0);
	value &= ~DCS_MASK_BITS;
	value |= ((len - 1) << 12);
	value &= ~(1<<31);
	ss_write(hw, CTRLR0, value);
}
void inline set_polcph_mode(struct ingenic_sslv *hw,unsigned int mode)
{
	unsigned int value = ss_read(hw, CTRLR0);
	value &= ~(0x3<<6);
	value |= (mode<<6);
	value &= ~(1<<31);
	ss_write(hw, CTRLR0, value);
}

void inline set_default_format(struct ingenic_sslv *hw,unsigned int mode)
{
	unsigned int value = ss_read(hw, CTRLR0);
	value &= ~((3<<21) | (3<<4));
	value &= ~(1<<31);
	ss_write(hw, CTRLR0, value);
}

void inline set_transfer_mode(struct ingenic_sslv *hw, unsigned int mode)
{
	unsigned int value = ss_read(hw, CTRLR0);
	if(mode == 0x2) {
		value |= (0x1 << 10);
	} else {
		value &= ~(0x1 << 10);
	}
	value &= ~(3<<8);
	value |= mode<<8;
	value &= ~(1<<31);
	ss_write(hw, CTRLR0, value);
}

void inline start_config_regs(struct ingenic_sslv *hw)
{
	/* printk(">>>>>>>>>%s %d\n",__func__,__LINE__); */
	ss_write(hw, SSIENR, SSI_DISEN);
}

void inline start_transmit(struct ingenic_sslv *hw)
{
	ss_write(hw, SSIENR, SSI_EN);
}

void inline finish_transmit(struct ingenic_sslv *hw)
{
	/* printk(">>>>>>>>>%s %d\n",__func__,__LINE__); */
	ss_write(hw, SSIENR, SSI_DISEN);
}

void inline set_tx_trigger(struct ingenic_sslv *hw, int value)
{
	value = value >= 1 ? value : 1;
	ss_write(hw, TXFTLR, (value & 0x3f));
}

void inline set_rx_trigger(struct ingenic_sslv *hw, int value)
{
	value = value >= 1 ? value : 1;
	ss_write(hw, RXFTLR, ((value - 1) & 0x3f));
}

int inline read_txfifo_entries(struct ingenic_sslv *hw)
{
	return ss_read(hw, TXFLR);
}

int inline read_rxfifo_entries(struct ingenic_sslv *hw)
{
	return ss_read(hw, RXFLR);
}

void inline disable_all_interrupts(struct ingenic_sslv *hw)
{
	ss_write(hw, SLV_IMR, 0x0);
}

void inline enable_all_interrupts(struct ingenic_sslv *hw)
{
	ss_write(hw, SLV_IMR, 0x1f);
}

void inline disable_rxfifo_threshold_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value &= ~RXFIM;
	ss_write(hw, SLV_IMR, value);
}

void inline enable_rxfifo_threshold_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value |= RXFIM;
	ss_write(hw, SLV_IMR, value);
}

void inline disable_rxfifo_overflow_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value &= ~RXOIM;
	ss_write(hw, SLV_IMR, value);
}

void inline enable_rxfifo_overflow_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value |= RXOIM;
	ss_write(hw, SLV_IMR, value);
}

void inline disable_rxfifo_empty_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value &= ~RXUIM;
	ss_write(hw, SLV_IMR, value);
}

void inline enable_rxfifo_empty_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value |= RXUIM;
	ss_write(hw, SLV_IMR, value);
}

void inline disable_txfifo_threshold_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value &= ~TXEIM;
	ss_write(hw, SLV_IMR, value);
}

void inline enable_txfifo_threshold_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value |= TXEIM;
	ss_write(hw, SLV_IMR, value);
}

void inline disable_txfifo_overflow_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value &= ~TXOIM;
	ss_write(hw, SLV_IMR, value);
}

void inline enable_txfifo_overflow_intr(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, SLV_IMR);
	value |= TXOIM;
	ss_write(hw, SLV_IMR, value);
}

void inline clear_all_interrupt(struct ingenic_sslv *hw)
{
	ss_read(hw, TXOICR);
	ss_read(hw, RXOICR);
	ss_read(hw, RXUICR);
}

void inline wait_for_sslv_idle(struct ingenic_sslv *hw)
{
	while(ss_read(hw, SLV_SR) & SR_BUSY);
}

void inline enable_tx_dmamode(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, DMACR);
	value |= TDMAE;
	ss_write(hw, DMACR, value);
}

void inline enable_rx_dmamode(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, DMACR);
	value |= RDMAE;
	ss_write(hw, DMACR, value);
}

void inline disable_tx_dmamode(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, DMACR);
	value &= ~TDMAE;
	ss_write(hw, DMACR, value);
}

void inline disable_rx_dmamode(struct ingenic_sslv *hw)
{
	unsigned int value = ss_read(hw, DMACR);
	value &= ~RDMAE;
	ss_write(hw, DMACR, value);
}

int inline set_txdma_threshold(struct ingenic_sslv *hw, unsigned int value)
{
	int ret = 0;
	if(value > 0x3f)
		ret = -EINVAL;
	else
		ss_write(hw, DMATDLR, value & 0x3f);
	return ret;
}

int inline set_rxdma_threshold(struct ingenic_sslv *hw, unsigned int value)
{
	int ret = 0;
	if(value > 0x3f)
		ret = -EINVAL;
	else
		ss_write(hw, DMARDLR, (value - 1) & 0x3f);
	return ret;
}

static inline void set_loop(struct ingenic_sslv *hw, u32 mode)
{
	u32 value = ss_read(hw, CTRLR0);
	if(mode)
		value |= SRL_TEST;
	else if(mode == 0)
		value &= ~SRL_TEST;
	value &= ~(0x1<<31);
	ss_write(hw, CTRLR0, value);
}
static inline void transmit_data(struct ingenic_sslv *hw, u32 value)
{
	ss_write(hw, SSLV_DR, value);
}
#endif /*__SPI_SLV_H__*/
