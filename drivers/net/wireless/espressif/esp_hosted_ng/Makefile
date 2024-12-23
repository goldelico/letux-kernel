# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng
ccflags-y += -I$(PWD)/drivers/net/wireless/espressif/esp_hosted_ng/include

obj-$(CONFIG_ESP_HOSTED_NG) += esp-hosted-ng.o

esp-hosted-ng-objs := main.o esp_bt.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o esp_debugfs.o esp_log.o

obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += esp-hosted-ng.o esp-hosted-ng-sdio.o
esp-hosted-ng-sdio-objs += sdio/esp_sdio.o sdio/esp_sdio_api.o

obj-$(CONFIG_ESP_HOSTED_NG_SPI) += esp-hosted-ng.o esp-hosted-ng-spi.o
esp-hosted-ng-spi-objs += spi/esp_spi.o


ifeq (yes,yes)	#### no idea why, but if this included from original Makefile we can compile an esp32_sdio.ko module ####

KERNEL := /lib/modules/$(shell uname -r)/build

#Default interface is sdio
MODULE_NAME=esp32_sdio

#enable logging by default
CONFIG_DEBUG_LOGS := y

#Targets passed overrrides default value
ifeq ($(CONFIG_ESP_HOSTED_NG_SDIO), m)
	MODULE_NAME=esp32_sdio
endif

ifeq ($(CONFIG_ESP_HOSTED_NG_SPI), m)
	MODULE_NAME=esp32_spi
endif

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

#debug_flags := -DDEBUG

EXTRA_CFLAGS += -I$(PWD)/include -I$(PWD)

ifeq ($(MODULE_NAME), esp32_sdio)
	EXTRA_CFLAGS += -I$(PWD)/sdio
	module_objects += sdio/esp_sdio.o sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	EXTRA_CFLAGS += -I$(PWD)/spi
	module_objects += spi/esp_spi.o
endif

module_objects += esp_bt.o main.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o esp_debugfs.o esp_log.o
#$(foreach obj,$(module_objects),$(eval CFLAGS_$(obj:.o=).o := $(debug_flags)))
CFLAGS_esp_log.o = -DDEBUG

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := $(module_objects)

all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean

endif
