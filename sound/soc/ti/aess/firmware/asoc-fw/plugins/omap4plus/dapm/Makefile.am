

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/plugins \
	-I$(top_builddir)/plugins/omap4plus \
	-I $(LINUX_DIR)/include \
	-I $(LINUX_DIR)/arch/arm/include \
	-I $(LINUX_DIR)/arch/arm/include/generated \
	-D__KERNEL__ \
	-D__LINUX_ARM_ARCH__=7 \
	-D__iomem= \
	-D_SYS_STAT_H \
	-D_SYS_TYPES_H \
	-D_LINUX_WAIT_H \
	-D_LINUX_TIME_H \
	-D__ASM_ARM_BYTEORDER_H

AM_LDFLAGS = $(NODELETE_LDFLAGS)

MODULE_LDFLAGS = $(AM_LDFLAGS) -module -avoid-version $(LDFLAGS_NOUNDEFINED)

modlibexecdir = $(libdir)/plugins

if HAVE_OMAP
modlibexec_LTLIBRARIES = \
		abe-dl1dl2-twl6040.la

abe_dl1dl2_twl6040_la_SOURCES = abe-dl1dl2-twl6040.c
abe_dl1dl2_twl6040_la_LDFLAGS = $(MODULE_LDFLAGS)
abe_dl1dl2_twl6040_la_LIBADD = $(MODULE_LIBADD)
abe_dl1dl2_twl6040_la_CFLAGS = $(AM_CFLAGS)

endif
