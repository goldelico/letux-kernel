

AM_CFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/coeffs \
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

if HAVE_OMAP

dl1dir = $(libdir)/abe/coeffs
dl1_LTLIBRARIES = dl1.la
dl1_la_SOURCES = dl1.c
dl1_la_LDFLAGS = $(MODULE_LDFLAGS)
dl1_la_LIBADD = $(MODULE_LIBADD)
dl1_la_CFLAGS = $(AM_CFLAGS)

dl2ldir = $(libdir)/abe/coeffs
dl2l_LTLIBRARIES = dl2l.la
dl2l_la_SOURCES = dl2l.c
dl2l_la_LDFLAGS = $(MODULE_LDFLAGS)
dl2l_la_LIBADD = $(MODULE_LIBADD)
dl2l_la_CFLAGS = $(AM_CFLAGS)

dl2rdir = $(libdir)/abe/coeffs
dl2r_LTLIBRARIES = dl2r.la
dl2r_la_SOURCES = dl2r.c
dl2r_la_LDFLAGS = $(MODULE_LDFLAGS)
dl2r_la_LIBADD = $(MODULE_LIBADD)
dl2r_la_CFLAGS = $(AM_CFLAGS)

dmicdir = $(libdir)/abe/coeffs
dmic_LTLIBRARIES = dmic.la
dmic_la_SOURCES = dmic.c
dmic_la_LDFLAGS = $(MODULE_LDFLAGS)
dmic_la_LIBADD = $(MODULE_LIBADD)
dmic_la_CFLAGS = $(AM_CFLAGS)

amicdir = $(libdir)/abe/coeffs
amic_LTLIBRARIES = amic.la
amic_la_SOURCES = amic.c
amic_la_LDFLAGS = $(MODULE_LDFLAGS)
amic_la_LIBADD = $(MODULE_LIBADD)
amic_la_CFLAGS = $(AM_CFLAGS)

sdtdir = $(libdir)/abe/coeffs
sdt_LTLIBRARIES = sdt.la
sdt_la_SOURCES = sdt.c
sdt_la_LDFLAGS = $(MODULE_LDFLAGS)
sdt_la_LIBADD = $(MODULE_LIBADD)
sdt_la_CFLAGS = $(AM_CFLAGS)

endif
