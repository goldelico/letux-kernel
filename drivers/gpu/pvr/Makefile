# derived from some SDK4 Makefile
DATE := $(shell date "+%Y%m%d%H%M%S" )

# these options must match the compiled pvrsrvinit or you will see e.g.
# SGXInit: Mismatch in driver and microkernel build options; extra options present in driver: (0x1)
# SGXInit: Mismatch in driver and microkernel build options; extra options present in microkernel: (0x210008) [1297, drivers/gpu/pvr/services4/srvkm/devices/sgx/sgxinit.c]

ccflags-y = -Idrivers/gpu/pvr \
	-Idrivers/gpu/pvr/include4 \
	-Idrivers/gpu/pvr/services4/include \
	-Idrivers/gpu/pvr/services4/system/include \
	-Idrivers/gpu/pvr/services4/system/omap3630 \
	-Idrivers/gpu/pvr/services4/srvkm/bridged \
	-Idrivers/gpu/pvr/services4/srvkm/bridged/sgx \
	-Idrivers/gpu/pvr/services4/srvkm/devices/sgx \
	-Idrivers/gpu/pvr/services4/srvkm/env/linux \
	-Idrivers/gpu/pvr/services4/srvkm/hwdefs \
	-Idrivers/gpu/pvr/services4/srvkm/include \
	-DLINUX -D__linux__ \
	-DPVR_BUILD_DIR="\"omap_sgx\"" \
	-DPVR_BUILD_DATE="\"$(DATE)\"" \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSERVICES4 \
	-D_XOPEN_SOURCE=600 \
	-DPVR2D_VALIDATE_INPUT_PARAMS \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_SGX \
	-DSUPPORT_XWS \
	-DSUPPORT_PERCONTEXT_PB \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DTRANSFER_QUEUE \
	-DSYS_USING_INTERRUPTS \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_FD_EXPORT \
	-DUSE_PTHREADS \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DLDM_PLATFORM \
	-DPVR2D_ALT_2DHW \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_LINUX_X86_PAT \
	-DDISPLAY_CONTROLLER=omaplfb \

#	-DSGX_DYNAMIC_TIMING_INFO \
#	-DSYS_CUSTOM_POWERLOCK_WRAP \
#	-DSUPPORT_MEMINFO_IDS \
#	-DCLIENT_DRIVER_DEFAULT_WAIT_RETRIES=50 \
#	-DPVR_LINUX_TIMERS_USING_WORKQUEUES \
#	-DFLIP_TECHNIQUE_OVERLAY \
#	-DPVR_LINUX_USING_WORKQUEUES \
#	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
#	-DSYS_CUSTOM_POWERLOCK_WRAP \
#	-DSUPPORT_SGX_NEW_STATUS_VALS \
#	-DPVRSRV_MODNAME="\"pvrsrvkm"\"

# 	-DANDROID \  # we are NOT compiling on Android
#		-DSUPPORT_ANDROID_PLATFORM \
#	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=100 \

#ccflags-$(CONFIG_SGX540) +=  -Idrivers/gpu/pvr/omap4  -Idrivers/gpu/pvr/sgx \
#	-DNO_OMAP_TIMER \
#	-DSGX_CLK_CORE_DIV5 \
#	-DSGX540 -DSUPPORT_SGX540

# ccflags-$(CONFIG_SGX_REV110) += -DSGX_CORE_REV=110
# ccflags-$(CONFIG_SGX_REV120) += -DSGX_CORE_REV=120

ccflags-$(CONFIG_SGX530) +=  -Idrivers/gpu/pvr/omap3  -Idrivers/gpu/pvr/sgx \
	-DSGX530 -DSUPPORT_SGX530

ccflags-$(CONFIG_SGX_REV125) += -DSGX_CORE_REV=125
ccflags-$(CONFIG_SGX_REV121) += -DSGX_CORE_REV=121

ccflags-$(CONFIG_SGX_530_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING

#	-DSGX_EARLYSUSPEND \

ccflags-$(CONFIG_SGX_530_BUILD_DEBUG) += \
	-O0 \
	-DPVR_BUILD_TYPE="\"debug\"" \
	-DDEBUG \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \

#	-DDEBUG_BRIDGE_KM_DISPATCH_TABLE
	
	
#	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
#	-DPVRSRV_DUMP_MK_TRACE \
#	-DDEBUG_LOG_PATH_TRUNCATE="\"eurasia_km\""

#ccflags-$(CONFIG_SGX_540_BUILD_RELEASE) += \
#	-DPVR_BUILD_TYPE="\"release\"" \
#	-DRELEASE \
#	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
#	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING

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

pvr_common-y := \
	services4/srvkm/env/linux/osfunc.o \
	services4/srvkm/env/linux/mutils.o \
	services4/srvkm/env/linux/mmap.o \
	services4/srvkm/env/linux/module.o \
	services4/srvkm/env/linux/pdump.o \
	services4/srvkm/env/linux/proc.o \
	services4/srvkm/env/linux/pvr_bridge_k.o \
	services4/srvkm/env/linux/pvr_debug.o \
	services4/srvkm/env/linux/mm.o \
	services4/srvkm/env/linux/mutex.o \
	services4/srvkm/env/linux/event.o \
	services4/srvkm/env/linux/osperproc.o \
	services4/srvkm/common/buffer_manager.o \
	services4/srvkm/common/devicemem.o \
	services4/srvkm/common/deviceclass.o \
	services4/srvkm/common/handle.o \
	services4/srvkm/common/hash.o \
	services4/srvkm/common/metrics.o \
	services4/srvkm/common/pvrsrv.o \
	services4/srvkm/common/queue.o \
	services4/srvkm/common/ra.o \
	services4/srvkm/common/resman.o \
	services4/srvkm/common/power.o \
	services4/srvkm/common/mem.o \
	services4/srvkm/common/pdump_common.o \
	services4/srvkm/common/perproc.o \
	services4/srvkm/bridged/bridged_support.o \
	services4/srvkm/bridged/bridged_pvr_bridge.o \
#	services4/srvkm/common/lists.o \
#	services4/srvkm/common/mem_debug.o \
#	osfunc_common.o

#pvr540-$(CONFIG_SGX540) := \
#	omap4/sysconfig.o \
#	omap4/sysutils.o

pvr530-$(CONFIG_SGX_REV121) := \
	services4/system/omap3430/sysconfig.o \
	services4/system/omap3430/sysutils.o

pvr530-$(CONFIG_SGX_REV125) := \
	services4/system/omap3630/sysconfig.o \
	services4/system/omap3630/sysutils.o

sgx-y :=  \
	services4/srvkm/bridged/sgx/bridged_sgx_bridge.o \
	services4/srvkm/devices/sgx/sgxinit.o \
	services4/srvkm/devices/sgx/sgxpower.o \
	services4/srvkm/devices/sgx/sgxreset.o \
	services4/srvkm/devices/sgx/sgxutils.o \
	services4/srvkm/devices/sgx/sgxkick.o \
	services4/srvkm/devices/sgx/sgxtransfer.o \
	services4/srvkm/devices/sgx/mmu.o \
	services4/srvkm/devices/sgx/pb.o

#sgx_displayclass-y := \
#	display/omap_sgx_displayclass.o \
#	display/omap_display.o

omaplfb-y := \
	services4/3rdparty/dc_omap3430_linux/omaplfb_displayclass.o \
	services4/3rdparty/dc_omap3430_linux/omaplfb_linux.o

#obj-$(CONFIG_SGX540) := pvr_common.o sgx.o omaplfb.o pvr540.o
#obj-$(CONFIG_SGX530) := pvr_common.o sgx.o pvr530.o
obj-$(CONFIG_SGX530) := pvr_common.o sgx.o omaplfb.o pvr530.o
#obj-$(CONFIG_VIRTUAL_DISPLAY_SUPPORT) := pvr_common.o sgx.o \
#	sgx_displayclass.o pvr540.o
