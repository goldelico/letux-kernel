#ifndef __SDIO_S3C24XX_HCD_H___
#define __SDIO_S3C24XX_HCD_H___

#define S3C24XX_HCD_NO_RESPONSE     1
#define S3C24XX_HCD_RESPONSE_SHORT  2
#define S3C24XX_HCD_RESPONSE_LONG   3
#define S3C24XX_HCD_DATA_READ       4
#define S3C24XX_HCD_DATA_WRITE      5

struct s3c24xx_hcd_device {
	OS_PNPDEVICE   pnp_device;     /* the OS device for this HCD */
	OS_PNPDRIVER   pnp_driver;     /* the OS driver for this HCD */
	SDDMA_DESCRIPTION dma;
	struct clk * clock;
	unsigned long max_clock_rate;
	unsigned long actual_clock_rate;
};


/* driver wide data, this driver only supports one device,
 * so we include the per device data here also */
struct s3c24xx_hcd_context {
	PTEXT			  description;       /* human readable device decsription */
	SDHCD			  hcd;                /* HCD description for bus driver */
	struct s3c24xx_hcd_device device;             /* the single device's info */
	struct platform_device    *pdev;
	struct resource           *mem;
	void __iomem		  *base;
	UINT32                    io_irq;
	UINT32                    cd_irq;
	BOOL			  card_inserted;       /* card inserted flag */
	BOOL			  cmd_processed;       /* command phase was processed */
	UINT32			  fifo_depth;          /* FIFO depth for the bus mode */
	BOOL			  irq_masked;
	UINT32		  	  bus_width;
	UINT32		  	  data_size;           /* Word, half word, or byte */
	UINT32		  	  latest_xfer_size;

	void                      *io_buffer;         /* Kernel address */
	dma_addr_t                io_buffer_dma;      /* Bus address */
	UINT32                    io_buffer_size;
	UINT32                    dma_channel;
	UINT32                    dma_en;
	struct completion         dma_complete;
	struct completion         xfer_complete;

	UINT32		  	  int_mask;
	UINT32		  	  int_sdio;            /* Do we have SDIO interrupt on ? */

	UINT32		  	  complete;

	UINT32		  	  cmdsta;
	UINT32		  	  dsta;
	UINT32		  	  fsta;

	spinlock_t		  lock;

	struct work_struct        io_work;
	struct work_struct        irq_work;
};

SDIO_STATUS s3c24xx_hcd_config(PSDHCD hcd, PSDCONFIG config);
SDIO_STATUS s3c24xx_hcd_request(PSDHCD hcd);

struct s3c24xx_hcd_context hcd_context;

#endif
