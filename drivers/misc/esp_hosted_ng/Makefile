libesp-y		:= esp_bt.o esp_cfg80211.o esp_cmd.o esp_debugfs.o esp_log.o esp_stats.o esp_utils.o main.o

ccflags-y += -I$(PWD)/drivers/misc/esp_hosted_ng
ccflags-y += -I$(PWD)/drivers/misc/esp_hosted_ng/include
obj-$(CONFIG_ESP_HOSTED_NG_SPI)		+= spi/esp_spi.o libesp.o
obj-$(CONFIG_ESP_HOSTED_NG_SDIO)	+= sdio/esp_sdio.o sdio/esp_sdio_api.o libesp.o