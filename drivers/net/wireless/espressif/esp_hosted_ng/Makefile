# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng/include

obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += esp-hosted-ng.o

esp-hosted-ng-objs := esp_bt.o esp_cfg80211.o esp_cmd.o esp_debugfs.o esp_log.o esp_stats.o main.o

obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += esp-hosted-ng-sdio.o
esp-hosted-ng-sdio-objs += sdio/esp_sdio.o sdio/esp_sdio_api.o

obj-$(CONFIG_ESP_HOSTED_NG_SPI) += esp-hosted-ng-spi.o
esp-hosted-ng-spi-objs += spi/esp_spi.o
