#!/bin/bash
# build firmware
# see: https://github.com/espressif/esp-hosted/blob/master/esp_hosted_ng/esp/esp_driver/README.md

DEST=$PWD
CHIP=esp32c6

if [ "$(uname)" == "Darwin" ]
then
	SRC=/Volumes/Retrode3/esp-hosted
	PORT=/dev/cu.usbserial-FTH9L0T7	# update to local setup
else
	SRC=/usr/local/src/esp-hosted
	PORT=/dev/tty???	# update to local setup
fi

set -e

(
	cd $SRC/esp_hosted_ng/esp/esp_driver
	cmake .
	cd esp-idf
	. ./export.sh
	cd ../network_adapter
	rm -f CMakeCache.txt
	rm -rf build
	idf.py set-target $CHIP
	rm -rf build
	idf.py build
	cd build
	cp bootloader/bootloader.bin network_adapter.bin ota_data_initial.bin partition_table/partition-table.bin $DEST
	(
	echo "This firmware was built $(date)"
	echo "from $(git rev-parse --short HEAD) @ $(git config --get remote.origin.url)"
	echo "for target $CHIP"
	echo
	echo "To flash, run this command:"
	echo "   python esptool.py -p \$PORT -b 460800 --before default_reset --after hard_reset --chip $CHIP write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 bootloader.bin 0x8000 partition-table.bin 0xd000 ota_data_initial.bin 0x10000 network_adapter.bin"
	) >$DEST/NOTES
)

echo Flash on Goldelico setup:
echo cd $SRC/esp_hosted_ng/esp/esp_driver/network_adapter
echo ~/.espressif/python_env/idf5.1_py3.7_env/bin/python ../esp-idf/components/esptool_py/esptool/esptool.py -p $PORT --before default_reset --after hard_reset --chip esp32c6  write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0xd000 build/ota_data_initial.bin 0x10000 build/network_adapter.bin
