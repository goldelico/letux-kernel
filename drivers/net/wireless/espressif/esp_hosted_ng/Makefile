# SPDX-License-Identifier: GPL-2.0

# Paths to driver headers
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng/include
ccflags-y += -I$(PWD)/include -I$(PWD)

# Core objects for esp-hosted-ng
esp-hosted-ng-objs := main.o esp_bt.o esp_cmd.o esp_utils.o esp_cfg80211.o \
                      esp_stats.o esp_debugfs.o esp_log.o

# SDIO driver objects
esp-hosted-ng-sdio-objs := sdio/esp_sdio.o sdio/esp_sdio_api.o

# SPI driver objects
esp-hosted-ng-spi-objs := spi/esp_spi.o

# Top-level module objects
esp-hosted-ng.o := $(esp-hosted-ng-objs)

# Conditional SDIO build
obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += esp-hosted-ng.o esp-hosted-ng-sdio.o

# Conditional SPI build
obj-$(CONFIG_ESP_HOSTED_NG_SPI) += esp-hosted-ng.o esp-hosted-ng-spi.o

# Enable debug logging if configured
CONFIG_DEBUG_LOGS := y	# there is no Kconfig yet
ifeq ($(CONFIG_DEBUG_LOGS), y)
CFLAGS_esp_log.o += -DDEBUG
endif

# Optional monitor process
ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
ccflags-y += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

# Add SDIO/SPI include paths if building those
ifeq ($(CONFIG_ESP_HOSTED_NG_SDIO), y)
ccflags-y += -I$(PWD)/sdio
endif

ifeq ($(CONFIG_ESP_HOSTED_NG_SPI), y)
ccflags-y += -I$(PWD)/spi
endif
