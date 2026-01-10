#! /bin/bash
#
# configure host for tethering with an Letux device connected through ethernet over USB (RNDIS/NCM ethernet gadget)

[ "$(which ip-forwarding)" ] || apt-get install letux-ip-forwarding

ip-forwarding "$@"