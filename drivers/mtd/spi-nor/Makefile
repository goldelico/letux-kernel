# SPDX-License-Identifier: GPL-2.0

spi-nor-objs			:= core.o sfdp.o swp.o otp.o sysfs.o

spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_ATMEL)			+= atmel.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_CATALYST)			+= catalyst.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_EON)				+= eon.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_ESMT)			+= esmt.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_EVERSPIN)			+= everspin.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_FUJITSU)			+= fujitsu.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_GIGADEVICE)			+= gigadevice.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_INTEL)			+= intel.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_ISSI)			+= issi.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_MACRONIX)			+= macronix.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_MICRON_ST)			+= micron-st.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_SPANSION)			+= spansion.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_SST)				+= sst.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_WINBOND)			+= winbond.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_XILINX)			+= xilinx.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_XMC)				+= xmc.o
spi-nor-$(CONFIG_MTD_SPI_NOR_MANUF_XTX)				+= xtx.o

spi-nor-$(CONFIG_DEBUG_FS)	+= debugfs.o
obj-$(CONFIG_MTD_SPI_NOR)	+= spi-nor.o

obj-$(CONFIG_MTD_SPI_NOR)	+= controllers/
