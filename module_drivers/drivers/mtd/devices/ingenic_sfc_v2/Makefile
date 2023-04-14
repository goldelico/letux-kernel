#
# linux/drivers/mtd/devices/ingenic_sfc_v2/Makefile
#

obj-$(CONFIG_INGENIC_BUILTIN_PARAMS) += nor_device/

ccflags-y += -I$(srctree)/drivers/mtd/nand/raw
obj-y	+= ingenic_sfc_common.o ingenic_sfc_nand.o nand_device/ ingenic_sfc_nor.o ingenic_sfc_ops.o ingenic_sfc_drv.o
