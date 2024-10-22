#!/bin/bash
# update code in kernel tree
set +e

if [ "$(uname)" == "Darwin" ]
then
	SRC=/Volumes/Retrode3/esp-hosted
else
	SRC=/usr/local/src/esp-hosted
fi


(
# check if SRC exists and if not, do a first git clone
# ( mkdir $(dirname "$SRC) && cd $(dirname "$SRC") && git clone https://github.com/espressif/esp-hosted.git $(basename "$SRC") )
	cd $SRC
	echo pull $PWD
	git pull
	echo submodule update $PWD
	git submodule update --init --recursive
)
