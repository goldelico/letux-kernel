# derived from some SDK4 Makefiles

# invent some rule to recursively make ourselves
# multiple times with different CONFIG_SGX_REV
# to build different kernel module flavours
# Problem: Kbuild assumes separate directories to store .cmd and .o files

# build pvr driver if any of those is y or m
ifneq ($(CONFIG_SGX530)$(CONFIG_SGX540),)

DATE := $(shell date "+%Y%m%d" )

# these options must match the compiled binary pvrsrvinit or you will see e.g.
# SGXInit: Mismatch in driver and microkernel build options; extra options present in driver: (0x1)
# SGXInit: Mismatch in driver and microkernel build options; extra options present in microkernel: (0x210008) [1297, drivers/gpu/pvr/omap3/services4/srvkm/devices/sgx/sgxinit.c]

ccflags-y += \
	-Iinclude \
	-Iarch/arm/mach-omap2 \
	-Idrivers/video/fbdev/omap2 \
	-Idrivers/gpu/pvr/omap3/ \
	-Idrivers/gpu/pvr/omap3/include4 \
	-Idrivers/gpu/pvr/omap3/services4/include \
	-Idrivers/gpu/pvr/omap3/services4/include/env/linux \
	-Idrivers/gpu/pvr/omap3/services4/system/include \
	-Idrivers/gpu/pvr/omap3/services4/system/omap3630 \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/bridged \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/bridged/sgx \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/devices/sgx \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/env/linux \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/hwdefs \
	-Idrivers/gpu/pvr/omap3/services4/srvkm/include \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/bufferclass_ti \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/dc_omapfb3_linux \
	-Idrivers/gpu/pvr/omap3/services4/3rdparty/linux_drm

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
	-DDISPLAY_CONTROLLER=omaplfb \
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

# has been disabled because otherwise modules are not loaded automatically
# but need support through X-server (AFAIK)
# and - we must compile as loadable module

ccflags-n += \
	-DSUPPORT_DRI_DRM

# for drm
ccflags-y += -Iinclude/drm \
	-DSYS_USING_INTERRUPTS \
	-DSUPPORT_OMAP3430_OMAPFB3 \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_DISPLAY_CONTROLLER_DRM_IOCTL

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

# build specific flags (only one can be choosen as y)
ccflags-$(CONFIG_SGX_530_BUILD_RELEASE) += \
	-Idrivers/gpu/pvr/omap3 \
	-DSGX530 \
	-DSUPPORT_SGX530 \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE \
	-g -Os \
	-fno-strict-aliasing -Wno-pointer-arith -Wno-sign-conversion

ccflags-$(CONFIG_SGX_530_BUILD_DEBUG) += \
	-Idrivers/gpu/pvr/omap3 \
	-DSGX530 \
	-DSUPPORT_SGX530 \
	-O0 \
	-DPVR_BUILD_TYPE="\"debug\"" \
	-DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-g -O0 -DDLL_METRIC=1 \
	-fno-strict-aliasing -Wno-pointer-arith -Wno-sign-conversion

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

# extra flags for some files
CFLAGS_pvr_drm_stubs.o := -DCONFIG_PCI
CFLAGS_drm_drv.o := -DCONFIG_PCI
CFLAGS_drm_stub.o := -DCONFIG_PCI
CFLAGS_ati_pcigart.o := -DCONFIG_PCI

# put common files into a library

lib-y := \
	omap3/services4/srvkm/env/linux/osfunc.o \
	omap3/services4/srvkm/env/linux/mutils.o \
	omap3/services4/srvkm/env/linux/mmap.o \
	omap3/services4/srvkm/env/linux/module.o \
	omap3/services4/srvkm/env/linux/pdump.o \
	omap3/services4/srvkm/env/linux/proc.o \
	omap3/services4/srvkm/env/linux/pvr_bridge_k.o \
	omap3/services4/srvkm/env/linux/pvr_debug.o \
	omap3/services4/srvkm/env/linux/pvr_drm.o \
	omap3/services4/srvkm/env/linux/mm.o \
	omap3/services4/srvkm/env/linux/mutex.o \
	omap3/services4/srvkm/env/linux/event.o \
	omap3/services4/srvkm/env/linux/osperproc.o \
	omap3/services4/srvkm/common/buffer_manager.o \
	omap3/services4/srvkm/common/devicemem.o \
	omap3/services4/srvkm/common/deviceclass.o \
	omap3/services4/srvkm/common/handle.o \
	omap3/services4/srvkm/common/hash.o \
	omap3/services4/srvkm/common/metrics.o \
	omap3/services4/srvkm/common/pvrsrv.o \
	omap3/services4/srvkm/common/queue.o \
	omap3/services4/srvkm/common/ra.o \
	omap3/services4/srvkm/common/resman.o \
	omap3/services4/srvkm/common/power.o \
	omap3/services4/srvkm/common/mem.o \
	omap3/services4/srvkm/common/pdump_common.o \
	omap3/services4/srvkm/common/perproc.o \
	omap3/services4/srvkm/common/lists.o \
	omap3/services4/srvkm/common/mem_debug.o \
	omap3/services4/srvkm/common/osfunc_common.o \
	omap3/services4/srvkm/common/refcount.o \
	omap3/services4/srvkm/bridged/bridged_support.o \
	omap3/services4/srvkm/bridged/bridged_pvr_bridge.o

lib-y +=  \
	omap3/services4/srvkm/bridged/sgx/bridged_sgx_bridge.o \
	omap3/services4/srvkm/devices/sgx/sgxinit.o \
	omap3/services4/srvkm/devices/sgx/sgxpower.o \
	omap3/services4/srvkm/devices/sgx/sgxreset.o \
	omap3/services4/srvkm/devices/sgx/sgxutils.o \
	omap3/services4/srvkm/devices/sgx/sgxkick.o \
	omap3/services4/srvkm/devices/sgx/sgxtransfer.o \
	omap3/services4/srvkm/devices/sgx/mmu.o \
	omap3/services4/srvkm/devices/sgx/pb.o

# compile from drivers/gpu/drm what we need
# this should be replaced by setting CONFIG_DRM=y
# but when we tried, it failed to compile

lib-n +=  \
	omap3/services4/3rdparty/linux_drm/pvr_drm_stubs.o \
	../drm/drm_auth.o \
	../drm/drm_bufs.o \
	../drm/drm_cache.o \
	../drm/drm_context.o \
	../drm/drm_dma.o \
	../drm/drm_drv.o \
	../drm/drm_fops.o \
	../drm/drm_gem.o \
	../drm/drm_ioctl.o \
	../drm/drm_irq.o \
	../drm/drm_lock.o \
	../drm/drm_memory.o \
	../drm/drm_stub.o \
	../drm/drm_vm.o \
	../drm/drm_agpsupport.o \
	../drm/drm_scatter.o \
	../drm/ati_pcigart.o \
	../drm/drm_pci.o \
	../drm/drm_sysfs.o \
	../drm/drm_hashtab.o \
	../drm/drm_mm.o \
	../drm/drm_crtc.o \
	../drm/drm_modes.o \
	../drm/drm_edid.o \
	../../../video/hdmi.o \
	../drm/drm_info.o \
	../drm/drm_debugfs.o \
	../drm/drm_encoder_slave.o

# not available in Linux 3.7
lib-n +=  \
	../drm/drm_drawable.o \
	../drm/drm_sman.o

# needed for Linux 3.7
lib-n += ../drm/drm_prime.o \
	../drm/drm_global.o

ccflags-n += \
	-DFBDEV_PRESENT=0

lib-n += omap3/services4/3rdparty/dc_omapfb3_linux/omaplfb_displayclass.o \
	omap3/services4/3rdparty/dc_omapfb3_linux/omaplfb_linux.o

# build into kernel (y) or as a module (m) depending on CONFIG_SGX_530/540
# It would be nice if we could find a trick to build separate
# RELEASE and DEBUG variants without reconfiguring
# and even nicer if we could build them all from one source file set
# and not only one

# revision specific flags (only one can currently be choosen as y)

# Revision 1.2.1 - Omap 3430
ifeq ($(CONFIG_SGX_REV121),y)
ccflags-y += -DSGX_CORE_REV=121 -DSGX_COMPATIBLE="\"img,sgx530_121\""
pvr_sgx530_rev121-$(CONFIG_SGX530) := \
	omap3/services4/system/omap3/sysconfig.o \
	omap3/services4/system/omap3/sysutils.o \
	lib.a
obj-$(CONFIG_SGX530) += pvr_sgx530_rev121.o
endif

# Revision 1.2.5 - Omap 3630
ifeq ($(CONFIG_SGX_REV125),y)
ccflags-y += -DSGX_CORE_REV=125 -DSGX_COMPATIBLE="\"img,sgx530_125\""
pvr_sgx530_rev125-$(CONFIG_SGX530) := \
	omap3/services4/system/omap3630/sysconfig.o \
	omap3/services4/system/omap3630/sysutils.o \
	lib.a
obj-$(CONFIG_SGX530) += pvr_sgx530_rev125.o
endif

# Revision 1.1.0 - Omap 4430 ES1
ifeq ($(CONFIG_SGX_REV110),y)
ccflags-y += -DSGX_CORE_REV=110 -DSGX_COMPATIBLE="\"img,sgx540_110\""
pvr_sgx540_rev110-$(CONFIG_SGX540) := \
	omap3/services4/system/omap3/sysconfig.o \
	omap3/services4/system/omap3/sysutils.o \
	lib.a
obj-$(CONFIG_SGX540) += pvr_sgx540_rev110.o
endif

# Revision 1.2.0 - Omap 4430 ES2
ifeq ($(CONFIG_SGX_REV120),y)
ccflags-y += -DSGX_CORE_REV=120 -DSGX_COMPATIBLE="\"img,sgx540_120\""
pvr_sgx540_rev120-$(CONFIG_SGX540) := \
	omap3/services4/system/omap4/sysconfig.o \
	omap3/services4/system/omap4/sysutils.o \
	lib.a
obj-$(CONFIG_SGX540) += pvr_sgx540_rev120.o
endif

endif  # COMPILE

# stub for SGX544
# end
