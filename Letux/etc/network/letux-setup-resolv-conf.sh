#!/bin/bash
# setup trustworthy DNS servers
# some recommended by https://www.privacy-handbuch.de/handbuch_93d.htm

(
echo "nameserver 9.9.9.9	# https://de.wikipedia.org/wiki/9.9.9.9"
echo "nameserver 84.200.69.80	# https://dns.watch"
# these seem to require DoH or DoT now and are therefore usually broken
echo "nameserver 89.233.43.71	# https://blog.uncensoreddns.org"
echo "nameserver 46.182.19.48	# dns2.digitalcourage.de"
echo "nameserver 185.95.218.42	# https://www.digitale-gesellschaft.ch/dns/"
) >/etc/resolv.conf