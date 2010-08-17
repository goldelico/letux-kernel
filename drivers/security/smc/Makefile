#
# Copyright (c) 2006-2007 Trusted Logic S.A.
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

ifeq ($(CONFIG_SMC_DRIVER_DEBUG_SUPPORT),y)
	EXTRA_CFLAGS += -O0 -DDEBUG -D_DEBUG
else
	EXTRA_CFLAGS += -DNDEBUG
endif
EXTRA_CFLAGS += -DLINUX
EXTRA_CFLAGS += -Wundef
EXTRA_CFLAGS += -Wno-unused-parameter -Wsign-compare -Wstrict-prototypes -fno-strict-aliasing
EXTRA_CFLAGS += -DSMODULE_SMC_OMAP3XXX_POWER_MANAGEMENT
EXTRA_CFLAGS += -DCOMPILE_AS_LINUX_MODULE

ifdef S_VERSION_BUILD
EXTRA_CFLAGS += -DS_VERSION_BUILD=$(S_VERSION_BUILD)
endif

obj-$(CONFIG_SMC_DRIVER) += scxlnx_util.o scxlnx_conn.o scxlnx_device.o scxlnx_sm_comm.o scx_public_crypto.o scx_public_crypto_Digest.o scx_public_crypto_AES.o scx_public_crypto_DES.o scx_public_dma.o bridge_pub2sec.o
