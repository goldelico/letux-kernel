# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Retrode3
#

# old
obj-$(CONFIG_RETRODE3)		+= retrode3.o

# new bus driver
obj-$(CONFIG_RETRODE3_GPIO_BUS)		+= retrode3-core.o retrode3-gpio-bus.o
# bus clients
obj-$(CONFIG_RETRODE3_MEGADRIVE)	+= retrode3-core.o retrode3-megadrive.o
obj-$(CONFIG_RETRODE3_NES)		+= retrode3-core.o retrode3-nes.o
obj-$(CONFIG_RETRODE3_SNES)		+= retrode3-core.o retrode3-snes.o
obj-$(CONFIG_RETRODE3_CONTROLLER)	+= retrode3-core.o retrode3-controller.o

# experimental
obj-$(CONFIG_RETRODE3_CLIENT)		+= retrode3-core.o retrode3-client-example.o

