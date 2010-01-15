#ifndef _OMAP2_MCSPI_H
#define _OMAP2_MCSPI_H

#define OMAP2_MCSPI_MASTER		0
#define OMAP2_MCSPI_SLAVE		1

struct omap2_mcspi_platform_config {
	unsigned short	num_cs;

	/* SPI is master or slave */
	unsigned short	mode;

	/* Use only DMA for data transfers */
	unsigned short	dma_mode;

	/* Force chip select mode */
	unsigned short	force_cs_mode;

	/* FIFO depth in bytes, max value 64 */
	unsigned short fifo_depth;

};

struct omap2_mcspi_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
};

#endif
