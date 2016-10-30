# derived from some SDK4 Makefiles

# invent some rule to recursively make ourselves
# multiple times with different CONFIG_SGX_REV
# to build different kernel module flavours
# Problem: Kbuild assumes separate directories to store .cmd and .o files

# although both omap3630 and am335x have the same SGX530 version
# we need to compile different kernel modules because the interfaces
# are not the same and we don't have a generic driver (yet) that
# uses DT for all SoC variance

SOC = omap3630
# SOC = ti335x

### almost generic...

# build into kernel (y) or as a module (m) depending on CONFIG_SGX_530/540
# It would be nice if we could find a trick to build separate
# RELEASE and DEBUG variants without reconfiguring
# and even nicer if we could build them all from one source file set
# and not only one

ifeq ($(SOC),omap3)
REV := 121
SGX := sgx530
ccflags-y += -DSGX530
endif
ifeq ($(SOC),omap3630)
REV := 125
SGX := sgx530
ccflags-y += -DSGX530
endif
ifeq ($(SOC),ti335x)
REV := 125
SGX := sgx530
ccflags-y += -DSGX530
endif
ifeq ($(SOC),omap4)
REV := 110
SGX := sgx540
ccflags-y += -DSGX540
endif
ifeq ($(SOC),omap4)
REV := 120
SGX := sgx540
ccflags-y += -DSGX540
endif
ifeq ($(SOC),omap5)
REV := 100
SGX := sgx544
ccflags-y += -DSGX544
endif

MODULE := pvr_$(SOC)_$(SGX)_$(REV)
TARGET := $(MODULE)-$(CONFIG_SGX_OMAP)

ccflags-y += -DSGX_CORE_REV=$(REV) -DSGX_COMPATIBLE="\"ti,$(SOC)-$(SGX)-$(REV)\""

obj-$(CONFIG_SGX_OMAP) += $(MODULE).o

DATE := $(shell date "+%Y%m%d" )

ccflags-y += \
	-Iinclude \
	-Iarch/arm/mach-omap2 \
	-Idrivers/video/fbdev/omap2 \
	-Idrivers/gpu/pvr/omap3/ \
	-Idrivers/gpu/pvr/omap3/include4 \
	-Idrivers/gpu/pvr/omap3/services4/include \
	-Idrivers/gpu/pvr/omap3/services4/include/env/linux \
	-Idrivers/gpu/pvr/omap3/services4/system/include \
	-Idrivers/gpu/pvr/omap3/services4/system/$(SOC) \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/bridged \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/bridged/sgx \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/devices/sgx \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/env/linux \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/hwdefs \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/include \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/bufferclass_ti

#
# many of the following options must match the compiled binary pvrsrvinit or you will see e.g.:
#
# SGXInit: Mismatch in driver and microkernel build options; extra options present in driver: (0x1)
# SGXInit: Mismatch in driver and microkernel build options; extra options present in microkernel: (0x210008) [1297, drivers/gpu/pvr/omap3/services4/srvkm/devices/sgx/sgxinit.c]
#

ccflags-y += \
	-DLINUX -D__linux__ \
	-DPVR_BUILD_DIR="\"omap_sgx\"" \
	-DPVR_BUILD_DATE="\"$(DATE)\"" \
	-DPVRSRV_MODNAME="\"pvrsrvkm"\" \
	-DPVR_USE_PRE_REGISTERED_PLATFORM_DEV \
	-DPM_RUNTIME_SUPPORT \
	-DSERVICES4 \
	-D_XOPEN_SOURCE=600 \
	-D_POSIX_C_SOURCE=199309 \
	-DPVR2D_VALIDATE_INPUT_PARAMS \
	-DDEBUG_LOG_PATH_TRUNCATE="\"\"" \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_SGX \
	-DSUPPORT_PERCONTEXT_PB \
	-DDISABLE_SGX_PB_GROW_SHRINK \
	-DSUPPORT_PVRSRV_GET_DC_SYSTEM_BUFFER \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DTRANSFER_QUEUE \
	-DPVR_DRI_DRM_NOT_PCI \
	-DSYS_USING_INTERRUPTS \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_DRM_AUTH_EXPORT \
	-DUSE_PTHREADS \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DLDM_PLATFORM \
	-DPVR2D_ALT_2DHW \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DSUPPORT_LINUX_X86_PAT \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DPVR_NO_FULL_CACHE_OPS \
	-DSGX_CLK_CORE_DIV5 \
	-DSUPPORT_SGX_NEW_STATUS_VALS

ifeq ($(CONFIG_SGX_DEBUG),)
ccflags-y += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE \
	-g -Os \
	-fno-strict-aliasing -Wno-pointer-arith -Wno-sign-conversion
else
ccflags-y += \
	-O0 \
	-DPVR_BUILD_TYPE="\"debug\"" \
	-DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-g -O0 \
	-DDLL_METRIC=1 \
	-fno-strict-aliasing -Wno-pointer-arith -Wno-sign-conversion
endif

$(TARGET) := \
	services4/srvkm/bridged/bridged_pvr_bridge.o \
	services4/srvkm/bridged/bridged_support.o \
	services4/srvkm/bridged/sgx/bridged_sgx_bridge.o \
	services4/srvkm/common/buffer_manager.o \
	services4/srvkm/common/devicemem.o \
	services4/srvkm/common/deviceclass.o \
	services4/srvkm/common/handle.o \
	services4/srvkm/common/hash.o \
	services4/srvkm/common/lists.o \
	services4/srvkm/common/mem.o \
	services4/srvkm/common/mem_debug.o \
	services4/srvkm/common/metrics.o \
	services4/srvkm/common/osfunc_common.o \
	services4/srvkm/common/pdump_common.o \
	services4/srvkm/common/perproc.o \
	services4/srvkm/common/power.o \
	services4/srvkm/common/pvrsrv.o \
	services4/srvkm/common/queue.o \
	services4/srvkm/common/ra.o \
	services4/srvkm/common/resman.o \
	services4/srvkm/common/refcount.o \
	services4/srvkm/devices/sgx/mmu.o \
	services4/srvkm/devices/sgx/pb.o \
	services4/srvkm/devices/sgx/sgxinit.o \
	services4/srvkm/devices/sgx/sgxkick.o \
	services4/srvkm/devices/sgx/sgxpower.o \
	services4/srvkm/devices/sgx/sgxreset.o \
	services4/srvkm/devices/sgx/sgxutils.o \
	services4/srvkm/devices/sgx/sgxtransfer.o \
	services4/srvkm/env/linux/event.o \
	services4/srvkm/env/linux/mm.o \
	services4/srvkm/env/linux/mmap.o \
	services4/srvkm/env/linux/module.o \
	services4/srvkm/env/linux/mutex.o \
	services4/srvkm/env/linux/mutils.o \
	services4/srvkm/env/linux/osfunc.o \
	services4/srvkm/env/linux/osperproc.o \
	services4/srvkm/env/linux/pdump.o \
	services4/srvkm/env/linux/proc.o \
	services4/srvkm/env/linux/pvr_bridge_k.o \
	services4/srvkm/env/linux/pvr_debug.o \
	services4/srvkm/env/linux/pvr_drm.o \
	services4/system/$(SOC)/sysconfig.o \
	services4/system/$(SOC)/sysutils.o

ifneq ($(CONFIG_SGX_DRM),)

ccflags-y += -Iinclude/drm \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/linux_drm \
	-DSYS_USING_INTERRUPTS \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_DISPLAY_CONTROLLER_DRM_IOCTL \
	-DSUPPORT_DRM_MODESET \
	-DSUPPORT_DRI_DRM_EXT \
	-DSUPPORT_DRI_DRM	# leads to drivers/gpu/pvr/omap3/services4/srvkm/env/linux/pvr_drm.c:448:1: error: size of array is too large

ccflags-n += \
	-DPVR_DRI_DRM_PLATFORM_DEV

$(TARGET) += \
	services4/3rdparty/linux_drm/pvr_drm_stubs.o

endif

SUPPORT_OMAPLFB =
ifneq ($(SUPPORT_OMAPLFB),)

ccflags-y += \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/dc_omapfb3_linux \
	-DFBDEV_PRESENT=1 \
	-DSUPPORT_OMAP3430_OMAPFB3 \
	-DDISPLAY_CONTROLLER=omaplfb \
	-DPVR_OMAPLFB_DRM_FB	# tries to #include <linux/omap_gpu.h>

# FIXME: this is also SOC specific (different FB for OMAP3 and AM33xx)
$(TARGET) += \
	services4/3rdparty/dc_omapfb3_linux/omaplfb_displayclass.o \
	services4/3rdparty/dc_omapfb3_linux/omaplfb_linux.o

endif

# old
ccflags-n += \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSUPPORT_XWS \
	-DPVR_SECURE_FD_EXPORT \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSUPPORT_MEMINFO_IDS \
	-DCLIENT_DRIVER_DEFAULT_WAIT_RETRIES=50 \
	-DFLIP_TECHNIQUE_OVERLAY \
	-DSUPPORT_SGX_NEW_STATUS_VALS \
	-DSGX_EARLYSUSPEND

 # we are NOT compiling on/for Android
ccflags-n += \
	-DANDROID \
	-DSUPPORT_ANDROID_PLATFORM \
	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=100

# old
ccflags-n += \
	-DDEBUG_BRIDGE_KM_DISPATCH_TABLE \
	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
	-DPVRSRV_DUMP_MK_TRACE

#ccflags-$(CONFIG_SGX_540_BUILD_RELEASE) += \
#	-DPVR_BUILD_TYPE="\"release\"" \
#	-DRELEASE \
#	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
#	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING
#	-Idrivers/gpu/pvr/omap3/omap4  -Idrivers/gpu/pvr/omap3/sgx \
#	-DNO_OMAP_TIMER \
#	-DSGX_CLK_CORE_DIV5 \
#	-DSGX540 -DSUPPORT_SGX540

#ccflags-$(CONFIG_SGX_540_BUILD_DEBUG) += \
#	-DPVR_BUILD_TYPE="\"debug\""  -DDEBUG \
#	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
#	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
#	-DDEBUG_LINUX_MEM_AREAS \
#	-DDEBUG_LINUX_MMAP_AREAS \
#	-DDEBUG_BRIDGE_KM \
#	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
#	-DPVRSRV_DUMP_MK_TRACE \
#	-DDEBUG_LOG_PATH_TRUNCATE="\"eurasia_km\""

# compile from drivers/gpu/drm what we need
# this should be replaced by setting CONFIG_DRM=y
# but when we tried, it failed to compile

# extra flags for some files
CFLAGS_drm_drv.o := -DCONFIG_PCI
CFLAGS_drm_stub.o := -DCONFIG_PCI
CFLAGS_ati_pcigart.o := -DCONFIG_PCI

$(TARGET)-n +=  \
	../../drm/drm_auth.o \
	../../drm/drm_bufs.o \
	../../drm/drm_cache.o \
	../../drm/drm_context.o \
	../../drm/drm_dma.o \
	../../drm/drm_drv.o \
	../../drm/drm_fops.o \
	../../drm/drm_gem.o \
	../../drm/drm_ioctl.o \
	../../drm/drm_irq.o \
	../../drm/drm_lock.o \
	../../drm/drm_memory.o \
	../../drm/drm_stub.o \
	../../drm/drm_vm.o \
	../../drm/drm_agpsupport.o \
	../../drm/drm_scatter.o \
	../../drm/ati_pcigart.o \
	../../drm/drm_pci.o \
	../../drm/drm_sysfs.o \
	../../drm/drm_hashtab.o \
	../../drm/drm_mm.o \
	../../drm/drm_crtc.o \
	../../drm/drm_modes.o \
	../../drm/drm_edid.o \
	../../../video/hdmi.o \
	../../drm/drm_info.o \
	../../drm/drm_debugfs.o \
	../../drm/drm_encoder_slave.o

# stub for SGX544
# end
