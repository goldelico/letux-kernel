# SPDX-License-Identifier: GPL-2.0

# Paths to driver headers
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng/include
ccflags-y += -I$(PWD)/include -I$(PWD)

# Core objects for esp-hosted-ng
esp-hosted-ng-core := main.o esp_bt.o esp_cmd.o esp_utils.o esp_cfg80211.o \
		      esp_stats.o esp_debugfs.o esp_log.o

# Conditional SDIO build
sdio-y := $(esp-hosted-ng-core) sdio/esp_sdio.o sdio/esp_sdio_api.o
obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += sdio.o

# Conditional SPI build
spi-y := $(esp-hosted-ng-core) spi/esp_spi.o
obj-$(CONFIG_ESP_HOSTED_NG_SPI) += spi.o

# NOTE: this is broken if spi or sdio is configured as "m" and the other as "y"

# Avoid a symbol name clash if both drivers are compiled
ifeq ($(CONFIG_ESP_HOSTED_NG_SPI), y)
CFLAGS_spi/esp_spi.o = -Desp_validate_chipset=spi_validate_chipset \
		-Desp_deinit_module=spi_deinit_module \
		-Desp_adjust_spi_clock=spi_adjust_spi_clock \
		-Dgenerate_slave_intr=spi_generate_slave_intr \
		-Desp_init_interface_layer=spi_init_interface_layer \
		-Desp_deinit_interface_layer=spi_deinit_interface_layer \
		-Dhost_sleep=spi_host_sleep
endif

# Enable debug logging if configured
ifeq ($(CONFIG_ESP_HOSTED_NG_DEBUG_LOGS), y)
CFLAGS_esp_log.o += -DDEBUG
endif

# Optional monitor process
ifeq ($(CONFIG_ESP_HOSTED_NG_ENABLE_MONITOR_PROCESS), y)
ccflags-y += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

# Add SDIO/SPI include paths if building those
ifeq ($(CONFIG_ESP_HOSTED_NG_SDIO), y)
#ccflags-y += -I$(PWD)/sdio
endif

ifeq ($(CONFIG_ESP_HOSTED_NG_SPI), y)
#ccflags-y += -I$(PWD)/spi
endif
