#!/bin/bash
# build firmware
# see: https://github.com/espressif/esp-hosted/blob/master/esp_hosted_ng/esp/esp_driver/README.md

DEST=$PWD
CHIP=esp32c6

# maybe this should go into some letux-python37-for-jessie package

function install_python37
{
	apt-get install python3	# let's hope it is already 3.7 or later
	case "$(python3 --version) 2>/dev/null)" in
		'Python '3.[7.9].* | 'Python '3.1?.* )
			;; # ok, is 3.7 or later
		* )
			apt-get install -t jessie-backports libssl-dev openssl	# we need 1.0.2 from backports
			apt-get install make build-essential zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev
			wget https://www.python.org/ftp/python/3.7.17/Python-3.7.17.tgz
			tar xvf Python-3.7.17.tgz
			cd Python-3.7.17
			./configure --enable-optimizations --enable-shared --with-ensurepip=install
			make altinstall
			# make test ==>
			#   Python build finished successfully!
			#   The necessary bits to build these optional modules were not found:
			#   _dbm                  _gdbm                 _uuid
			#   To find the necessary bits, look in setup.py in detect_modules() for the module's name.
			#
			ldconfig /usr/local/lib	# make it find libpython3.7m.so.1.0
			update-alternatives --install /usr/bin/python python /usr/local/bin/python3.7 50
			update-alternatives --install /usr/bin/python3 python3 /usr/local/bin/python3.7 50
			;;
	esac
}

if [ "$(uname)" == "Darwin" ]
then
	SRC=/Volumes/Retrode3/esp-hosted
	PORT=/dev/cu.usbserial-FTH9L0T7	# update to local setup
else
	SRC=/usr/local/src/esp-hosted
	PORT=/dev/ttyS3	# internal UART3
	[ "$(which cmake)" ] || apt-get install cmake
	[ "$(which g++)" ] || apt-get install g++
	[ "$(which python3)" ] || install_python37
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
