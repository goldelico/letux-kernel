#
# this Makefile builds multiple kernel modules
# representing different SGX variants and flavours
#
# not working:
#  creating / copying subdirectories
#  passing CFLAGS down to subdirectory make
#  untested: if subdirectory make generates varying kernel modules
#  untested: make clean

# build in original subdirectory while we are working on this Makefile
obj-$(CONFIG_SGX_OMAP) += omap3/
obj-$(CONFIG_SGX_OMAP)-n += omap5/

# obj-$(CONFIG_SGX_OMAP) += omap3/ omap3630/ ti335x/ omap5/

# the following stuff is not working as intended :(
targets += copy omap3/ omap3630/ ti335x/ omap5/

.PHONY:	copy

copy:
	@$(kecho) cp -R $(src)/omap3/ $(obj)/omap3/
	cp -R $(src)/omap3/ $(obj)/omap3/
	cp -R $(src)/omap3/ $(obj)/omap3630/
	cp -R $(src)/omap3/ $(obj)/ti335x/
	# somehow set CONFIGs in Makefile copies

ifeq (X,U)

# make a copy
CFLAGS_omap3-121 += -DCONFIG_SGX530=$(CONFIG_SGX530) -DCONFIG_SGX_REV121=y -CONFIG_SGX_530_BUILD_RELEASE=y -DSGX_CORE_REV=121 -DSGX_COMPATIBLE="\"img,sgx530_121\""
CFLAGS_omap3-121 += -Idrivers/gpu/pvr/omap3/services4/system/omap3
$(obj)/omap3-121: $(src)/omap3/Makefile
	@$(kecho) cp -R $(src)/omap3/ $(obj)/omap3-121/
	cp -R $(src)/omap3/ $(obj)/omap3-121/
	# somehow set CONFIG for 121
clean-dirs += omap3-121/

CFLAGS_omap3-125 += -DCONFIG_SGX530=$(CONFIG_SGX530) -DCONFIG_SGX_REV125=y CONFIG_SGX_530_BUILD_RELEASE=y -DSGX_CORE_REV=125 -DSGX_COMPATIBLE="\"img,sgx530_125\""
CFLAGS_omap3-125 += -Idrivers/gpu/pvr/omap3/services4/system/omap3630
$(obj)/omap3-125: $(src)/omap3/Makefile
	@$(kecho) cp -R $(src)/omap3/ $(obj)/omap3-125/
	cp -R $(src)/omap3/ $(obj)/omap3-125/
	# somehow set CONFIG for 125
clean-dirs += omap3-125/
endif

