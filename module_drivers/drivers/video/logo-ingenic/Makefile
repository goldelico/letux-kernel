# Makefile for true color logo.

obj-y	+= logo.o
obj-y	+= showlogo.o

ifeq ($(CONFIG_EXT_LOGO),y)
LOGO_FILE:=$(srctree)/$(OUTSIDE_BUILD)/$(CONFIG_TRUE_COLOR_LOGO_FILE)
else
LOGO_FILE:=$(srctree)/$(CONFIG_TRUE_COLOR_LOGO_FILE)
endif
$(obj)/logo.c: scripts/pic2logo FORCE
	@$< $(LOGO_FILE) 32 $(CONFIG_TRUE_COLOR_LOGO_BACKGROUND)> $@
