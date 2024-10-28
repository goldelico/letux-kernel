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
	if [ ! -x "$SRC" ]
	then
		echo clone from git
		mkdir -p "$(dirname "$SRC")" &&
		cd "$(dirname "$SRC")" &&
		git clone https://github.com/espressif/esp-hosted.git "$(basename "$SRC")"
	fi
	cd "$SRC"
	echo pull to $PWD
	git pull
	echo submodule update $PWD
	git submodule update --init --recursive
)
