
ccflags-y += -I$(srctree)/module_drivers/drivers/media/platform/ingenic-vcodec/felix/libh264/
ccflags-y += -I$(srctree)/module_drivers/drivers/media/platform/ingenic-vcodec/felix/libh264/include

ingenic_felix-y :=	felix_drv.o \
			felix_ops.o \
			libh264/api/jzm_h264_dec.o\
			libh264/src/log.o	\
			libh264/src/mem.o	\
			libh264/src/h264_picture.o		\
			libh264/src/golomb.o		\
			libh264/src/h264data.o		\
			libh264/src/h264_refs.o		\
			libh264/src/mathtables.o		\
			libh264/src/h2645_parse.o		\
			libh264/src/h264_parse.o		\
			libh264/src/h264_direct.o		\
			libh264/src/h264_ps.o		\
			libh264/src/h264_sei.o		\
			libh264/src/buffer.o		\
			libh264/src/vpu_ops.o		\
			libh264/src/h264_slice.o		\
			libh264/src/h264dec.o

obj-$(CONFIG_INGENIC_FELIX) += ingenic_felix.o




