#!/bin/bash
# update code in kernel tree
set +e

ROOT=$PWD/../../../../..	# Root of Letux kernel tree

if [ "$(uname)" == "Darwin" ]
then
	SRC=/Volumes/Retrode3/esp-hosted
else
	SRC=/usr/local/src/esp-hosted
fi

# ./esp-firmware-update.sh ?
(
	cd $SRC/esp_hosted_ng/host
	git pull
	tar cf - --exclude Makefile --exclude spidev_disabler.dts --exclude rpi_init.sh . | (cd $ROOT/drivers/net/wireless/espressif/esp_hosted_ng && tar xvf -)
)
