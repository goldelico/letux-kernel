#!/bin/bash

usage() {
	echo "$0 <diff-file>"
}

DIFF="$1"

if [ $# -ne 1 ]; then
	usage
	exit 1
fi

my_big_regex() {
	# magic: part 1 = +/- sign at line start
	# part 2: name of the config (CONFIG_ needs to be prepended)
	# part 3: value or setting (n m y "" 0-9)
	# turn into CONFIG_<SOMETHING>=<SOMETHING>
	sed -e "s;\(^+\)\([a-zA-Z0-9_]\+\) \(.\+\);CONFIG_\2=\3;g"
}

cat "$DIFF" | my_big_regex

exit 0
