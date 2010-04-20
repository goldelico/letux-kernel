#
ccflags-y = -DLINUX -D__linux__ -Idrivers/gpu/pvr \
	-DANDROID \
	-DPVR_BUILD_DIR="\"omap_sgx_android\"" \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSERVICES4 \
	-DPVR_PROC_USE_SEQ_FILE \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_PERCONTEXT_PB \
	-DTRANSFER_QUEUE \
	-DSYS_USING_INTERRUPTS \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_FD_EXPORT \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DLDM_PLATFORM \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DSUPPORT_CACHEFLUSH_ON_ALLOC \
	-DSUPPORT_MEMINFO_IDS \
	-DDISPLAY_CONTROLLER=omaplfb \
	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=100 \
	-DPVR_BUILD_DATE="" \
	-DUSE_PTHREADS \
	-DSUPPORT_PDUMP_DELAYED_INITPHASE_TERMINATION \
	-DSUPPORT_SGX \
	-DCLIENT_DRIVER_DEFAULT_WAIT_RETRIES=50

ccflags-$(CONFIG_SGX540) +=  -Idrivers/gpu/pvr/omap4  -Idrivers/gpu/pvr/sgx \
	-DNO_OMAP_TIMER \
	-DSGX_CLK_CORE_DIV5 \
	-DFLIP_TECHNIQUE_FRAMEBUFFER \
	-DSGX540 -DSUPPORT_SGX540

ccflags-$(CONFIG_SGX_REV110) += -DSGX_CORE_REV=110

ccflags-$(CONFIG_SGX530) +=  -Idrivers/gpu/pvr/omap3  -Idrivers/gpu/pvr/sgx \
	-DSGX530 -DSUPPORT_SGX530

ccflags-$(CONFIG_SGX_REV125) += -DSGX_CORE_REV=125
ccflags-$(CONFIG_SGX_REV121) += -DSGX_CORE_REV=121

ccflags-$(CONFIG_SGX_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\""  -DRELEASE

ccflags-$(CONFIG_SGX_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\""  -DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-DDLL_METRIC=1 \
	-DDEBUG_LOG_PATH_TRUNCATE="\"eurasia_km\""

pvr_common-y := \
	osfunc.o \
	mutils.o \
	mmap.o \
	module.o \
	pdump.o \
	proc.o \
	pvr_bridge_k.o \
	pvr_debug.o \
	mm.o \
	mutex.o \
	event.o \
	osperproc.o \
	buffer_manager.o \
	devicemem.o \
	deviceclass.o \
	handle.o \
	hash.o \
	metrics.o \
	pvrsrv.o \
	queue.o \
	ra.o \
	resman.o \
	power.o \
	mem.o \
	pdump_common.o \
	bridged_support.o \
	bridged_pvr_bridge.o \
	perproc.o \
	lists.o

pvr540-$(CONFIG_SGX540) := \
	omap4/sysconfig.o \
	omap4/sysutils.o

pvr530-$(CONFIG_SGX530) := \
	omap3/sysconfig.o \
	omap3/sysutils.o

sgx-y :=  \
	sgx/bridged_sgx_bridge.o \
	sgx/sgxinit.o \
	sgx/sgxpower.o \
	sgx/sgxreset.o \
	sgx/sgxutils.o \
	sgx/sgxkick.o \
	sgx/sgxtransfer.o \
	sgx/mmu.o \
	sgx/pb.o

omaplfb-y := \
	omaplfb/omaplfb_displayclass.o \
	omaplfb/omaplfb_linux.o

obj-$(CONFIG_SGX540) := pvr_common.o sgx.o omaplfb.o pvr540.o
obj-$(CONFIG_SGX530) := pvr_common.o sgx.o omaplfb.o pvr530.o
