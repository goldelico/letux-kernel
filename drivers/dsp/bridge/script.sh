#!/bin/sh
grep -rl "	" $1 |
while read filename
do
(
echo $filename
sed "s/	/	/g;" $filename> $filename.xx
mv $filename.xx $filename
)
done
