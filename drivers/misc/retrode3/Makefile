# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Retrode3
#

# core
retrode3-lib-y						+= retrode3-core.o retrode3-slot.o

# bus
obj-$(CONFIG_RETRODE3_BUS)			+= retrode3-lib.o
obj-$(CONFIG_RETRODE3_BUS)			+= retrode3-bus.o retrode3-lib.o

# clients
obj-$(CONFIG_RETRODE3_MEGADRIVE)	+= retrode3-megadrive.o retrode3-lib.o
obj-$(CONFIG_RETRODE3_NES)			+= retrode3-nes.o retrode3-lib.o
obj-$(CONFIG_RETRODE3_SNES)			+= retrode3-snes.o retrode3-lib.o
obj-$(CONFIG_RETRODE3_CONTROLLER)	+= retrode3-controller.o retrode3-lib.o
