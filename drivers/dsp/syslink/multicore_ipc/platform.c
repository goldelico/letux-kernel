/*
 *  platform.c
 *
 *  Implementation of platform initialization logic for Syslink IPC.
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 *  This package is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.
 */


/* Standard header files */
#include <linux/types.h>
#include <linux/module.h>


/* Utilities & Osal headers */
/*#include <Gate.h>
#include <GateMutex.h>
#include <Memory.h>*/

/* SysLink device specific headers */
#include "../procmgr/proc4430/proc4430.h"

/* Module level headers */
#include <multiproc.h>
#include <sysmgr.h>
#include <_sysmgr.h>
#include <sysmemmgr.h>
#include <platform.h>
#include <gatepeterson.h>
#include <sharedregion.h>
#include <listmp.h>
#include <messageq.h>
#include <messageq_transportshm.h>
#include <notify.h>
/*#include <NotifyDriver.h>*/
#include <notify_ducatidriver.h>
#include <nameserver.h>
#include <nameserver_remote.h>
#include <nameserver_remotenotify.h>
#include <procmgr.h>
#include <heap.h>
#include <heapbuf.h>

#include <platform_mem.h>


/** ============================================================================
 *  Application specific configuration, please change these value according to
 *  your application's need.
 *  ============================================================================
 */
/* App defines */
#define HEAPID				0
#define HEAPNAME			"myHeap"

/*!
 *  @brief  Interrupt ID of physical interrupt handled by the Notify driver to
 *          receive events.
 */
#define BASE_DUCATI2ARM_INTID		26

/*!
 *  @brief  Interrupt ID of physical interrupt handled by the Notify driver to
 *          send events.
 */
#define BASE_ARM2DUCATI_INTID		50

/*!
 *  @brief Maximum events supported by Notify component
 */
#define NOTIFY_MAX_EVENTS		32

/*!
 *  @brief Number of event reserved i.e. can not be used by application
 */
#define NOTIFY_NUMRESERVEDEVENTS	0

/*!
 *  @brief Wait for this much poll count when sending event
 */
#define NOTIFY_SENDEVENTPOLLCOUNT	((u32) -1)

/*!
 *  @brief Align buffer in Heap
 */
#define HEAPBUF_ALIGN			128

/*!
 *  @brief Number of blocks in the heap
 */
#define HEAPBUF_NUMBLOCKS		16

/*!
 *  @brief Size of each blocks in heap
 */
#define HEAPBUF_BLOCKSIZE		256

/*! @brief Start of shared memory */
#define SHAREDMEMORY_BASEADDR		0x87100000
#define SHAREDMEMORY_BASESIZE		0x0007F000

/*! @brief Start of Boot load page  */
#define BOOTLOADPAGE_BASEADDR		0x8717F000
#define BOOTLOADPAGE_BASESIZE		0x00001000

/*! @brief Start of shared memory */
#define SHAREDMEMORY_BASEADDR_APPM3		0x87180000
#define SHAREDMEMORY_BASESIZE_APPM3		0x0007F000

/*! @brief Start of Boot load page  */
#define BOOTLOADPAGE_BASEADDR_APPM3		0x8718F000
#define BOOTLOADPAGE_BASESIZE_APPM3		0x0000F000

/*!
 *  @brief  Size of the shared memory heap, this heap is used for providing
 * shared memory to drivers/instances. Should not be used for any other purpose.
 */
#define SMHEAP_SIZE			0x10000

/*!
 *  @brief  Shared region index for Shared memory heap.
 */
#define SMHEAP_SRINDEX			0

/*!
 *  @brief Event no used by sysmemmgr
 */
#define PLATFORM_SYSMEMMGR_EVENTNO	31


/** ============================================================================
 * Command Id used by bootloadpage logic to transfer info
 *  ============================================================================
 */
/*!
 *  @brief Command ID for notify driver.
 */
#define PLATFORM_CMD_NOTIFYDRIVER	SYSMGR_CMD_SHAREDREGION_ENTRY_END

/*!
 *  @brief Command ID for GatePeterson used by nameserverremotenotify.
 */
#define PLATFORM_CMD_GPNSRN		(PLATFORM_CMD_NOTIFYDRIVER + 1)

/*!
 *  @brief Command ID for nameserverremotenotify.
 */
#define PLATFORM_CMD_NSRN		(PLATFORM_CMD_NOTIFYDRIVER + 2)

/*!
 *  @brief Command ID for GatePeterson used by HeapBuf.
 */
#define PLATFORM_CMD_GPHEAPBUF		(PLATFORM_CMD_NOTIFYDRIVER + 3)

/*!
 *  @brief Command ID for HeapBuf.
 */
#define PLATFORM_CMD_HEAPBUF		(PLATFORM_CMD_NOTIFYDRIVER + 4)

/*!
 *  @brief Command ID for GatePeterson used by MessageQTransportShm.
 */
#define PLATFORM_CMD_GPMQT		(PLATFORM_CMD_NOTIFYDRIVER + 5)

/*!
 *  @brief Command ID for MessageQTransportShm.
 */
#define PLATFORM_CMD_MQT		(PLATFORM_CMD_NOTIFYDRIVER + 6)


/** ============================================================================
 * Handles used by platform logic
 *  ============================================================================
 */
void *platform_nsrn_gate_handle;
void *platform_nsrn_handle;
void *platform_notifydrv_handle;
void *platform_heap_gate_handle;
void *platform_heap_handle;
void *platform_mqt_gate_handle;
void *platform_transport_shm_handle;
void *platform_messageq;

/** ============================================================================
 *  Struct & Enums.
 *  ============================================================================
 */
/* Struct for reading platform specific gate peterson configuration values */
struct platform_gaterpeterson_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u32 remote_proc_id;	/* Remote processor identifier */
};

struct platform_notify_ducatidrv_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u16 remote_proc_id;	/* Remote processor identifier */
};

struct platform_nameserver_remotenotify_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u32 notify_event_no;	/* Notify Event number to used */
};

struct platform_heapbuf_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u32 shared_buf_addr;	/* Shared memory address */
	u32 shared_buf_size;	/* Shared memory size */
	u32 num_blocks;
	u32 block_size;
};

struct platform_messageq_transportshm_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u32 notify_event_no;	/* Notify Event number */
};

struct platform_proc_config_params {
	u32 reserved1;
	u32 use_notify;
	u32 use_messageq;
	u32 use_heapbuf;
	u32 use_listmp;
	u32 use_nameserver;
	u32 reserved2;
};


/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
/*!
 *  @brief  Number of slave memory entries for OMAP4430.
 */
#define NUM_MEM_ENTRIES			2

/*!
 *  @brief  Position of reset vector memory region in the memEntries array.
 */
#define RESET_VECTOR_ENTRY_ID		0


/** ============================================================================
 *  Globals
 *  ============================================================================
 */
/*!
 *  @brief  Array of memory entries for OMAP4430
 */
static struct proc4430_mem_entry mem_entries[NUM_MEM_ENTRIES] = {
	{
		"DUCATI_SHM",	/* NAME	     : Name of the memory region */
		0x87100000,	/* PHYSADDR	     : Physical address */
		0x98000000,	/* SLAVEVIRTADDR  : Slave virtual address */
		(u32) -1u,
			/* MASTERVIRTADDR : Master virtual address (if known) */
		0x80000,	/* SIZE	     : Size of the memory region */
		true,		/* SHARE	     : Shared access memory? */
	},
	{
		"DUCATI_SHM1",	/* NAME	     : Name of the memory region */
		0x87180000,	/* PHYSADDR	     : Physical address */
		0x98080000,	/* SLAVEVIRTADDR  : Slave virtual address */
		(u32) -1u,
			/* MASTERVIRTADDR : Master virtual address (if known) */
		0x80000,	/* SIZE	     : Size of the memory region */
		true,		/* SHARE	     : Shared access memory? */
	}
};


/*!
 *  @brief  Handle to the ProcMgr instance used.
 */
void *procmgr_handle;

/*!
 *  @brief  Handle to the Processor instance used.
 */
void *procmgr_proc_handle;

/*!
 *  @brief  File ID of the file loaded.
 */
u32 procmgr_file_id;

/*!
 *  @brief  Shared memory heap virtual address.
 */
void *platform_sm_heap_virt_addr;

/*!
 *  @brief  Shared memory heap physical address.
 */
void *platform_sm_heap_phys_addr;

/*!
 *  @brief  Scalability info
 */
struct platform_proc_config_params pc_params;

/* =============================================================================
 * APIS
 * =============================================================================
 */
/*
 * ======== platform_setup ========
 *  Purpose:
 *  TBD: logic would change completely in the final system.
 */
s32 platform_setup(struct sysmgr_config *config)
{
	s32 status = 0;
	struct proc4430_config proc_config;
	struct proc_mgr_params params;
	struct proc4430_params proc_params;
	u16 proc_id;
	struct sysmemmgr_config sysmemmgr_cfg;
	struct platform_mem_map_info info;

	if (WARN_ON(config == NULL)) {
		/*! @retval SYSMGR_E_INVALIDARG Argument of type
		 *  (GatePeterson_Config *) passed is null*/
		status = -EINVAL;
		goto invalid_config_fail;
	}

	/* Map the static region */
	info.src = SHAREDMEMORY_BASEADDR;
	info.size = SHAREDMEMORY_BASESIZE;
	info.is_cached = false;
	status = platform_mem_map(&info);
	if (status < 0)
		goto mem_map_fail;

	/* Get default config for System memory manager */
	sysmemmgr_get_config(&sysmemmgr_cfg);
	/* Initialize the System memory manager */
	sysmemmgr_cfg.static_mem_size = SHAREDMEMORY_BASESIZE;
	sysmemmgr_cfg.static_phys_base_addr = SHAREDMEMORY_BASEADDR;
	sysmemmgr_cfg.static_virt_base_addr = info.dst;
	sysmemmgr_cfg.event_no = PLATFORM_SYSMEMMGR_EVENTNO;
	status = sysmemmgr_setup(&sysmemmgr_cfg);
	if (status < 0)
		goto sysmemmgr_setup_fail;

	platform_sm_heap_virt_addr = sysmemmgr_alloc(SMHEAP_SIZE,
					sysmemmgr_allocflag_physical);
	if (platform_sm_heap_virt_addr == NULL)
		goto sysmemmgr_alloc_fail;

	/* Create the shared region entry for the SM heap */
	sharedregion_add(SMHEAP_SRINDEX, platform_sm_heap_virt_addr,
			SMHEAP_SIZE);
	/* Zero out the shared memory */
	memset((void *) platform_sm_heap_virt_addr, 0, SMHEAP_SIZE);

	proc4430_get_config(&proc_config);
	status = proc4430_setup(&proc_config);
	if (status < 0)
		goto proc_setup_fail;

	/* Get MultiProc ID by name. */
	proc_id = multiproc_get_id("SysM3");

	/* Create an instance of the Processor object for OMAP4430 */
	proc4430_params_init(NULL, &proc_params);
	proc_params.num_mem_entries = NUM_MEM_ENTRIES;
	proc_params.mem_entries = (struct proc4430_mem_entry *) &mem_entries;
	proc_params.reset_vector_mem_entry = RESET_VECTOR_ENTRY_ID;

	procmgr_proc_handle = proc4430_create(proc_id, &proc_params);
	if (procmgr_proc_handle == NULL) {
		status = SYSMGR_E_FAIL;
		goto proc_create_fail;
	}

	/* Initialize parameters */
	proc_mgr_params_init(NULL, &params);
	params.proc_handle = procmgr_proc_handle;
	procmgr_handle = proc_mgr_create(proc_id, &params);
	if (procmgr_handle == NULL) {
		status = SYSMGR_E_FAIL;
		goto proc_mgr_create_fail;
	}

proc_mgr_create_fail:
	printk(KERN_ERR "platform_setup: proc_mgr_create failed [0x%x]",
		status);
	goto exit;
proc_create_fail:
	printk(KERN_ERR "platform_setup: proc4430_create failed [0x%x]",
		status);
	goto exit;
proc_setup_fail:
	printk(KERN_ERR "platform_setup: proc4430_setup failed [0x%x]",
		status);
	goto exit;
sysmemmgr_alloc_fail:
	printk(KERN_ERR "platform_setup: sysmemmgr_alloc failed [0x%x]",
		status);
	goto exit;
sysmemmgr_setup_fail:
	printk(KERN_ERR "platform_setup: sysmemmgr_setup failed [0x%x]",
		status);
	goto exit;
mem_map_fail:
	printk(KERN_ERR "platform_setup: platform_mem_map failed [0x%x]",
		status);
	goto exit;
invalid_config_fail:
	printk(KERN_ERR "platform_setup: Argument of type (sysmgr_get_config *)"
		" passed is null [0x%x]", status);
exit:
	return status;
}


/*
 * ======== platform_destroy ========
 *  Purpose:
 *  Function to finalize the platform.
 */
s32 platform_destroy(void)
{
	s32 status = 0;
	struct platform_mem_unmap_info u_info;

	/* Delete the Processor instances */
	if (procmgr_proc_handle != NULL) {
		status = proc4430_delete(&procmgr_proc_handle);
		WARN_ON(status < 0);
	}

	if (procmgr_handle != NULL) {
		status = proc_mgr_delete(&procmgr_handle);
		WARN_ON(status < 0);
	}

	status = proc4430_destroy();
	WARN_ON(status < 0);

	/* Delete the memory map */
	u_info.addr = (u32) platform_sm_heap_virt_addr;
	platform_mem_unmap(&u_info);

	status = sysmemmgr_destroy();
	WARN_ON(status < 0);

	return status;
}


/*
 * ======== platform_load_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in loaded state.
 */
void platform_load_callback(void *arg)
{
	s32 status = 0;
	u16 proc_id = (u32) arg;
	u16 local_id = MULTIPROC_INVALIDID;
	struct sharedregion_info info;
	u32 boot_load_page;
	u32 sh_addr_base;
	u32 nwrite;

	/* Get the boot load page address */
	/* TBD:
	boot_load_page = Proc_getBootLoadPage (procmgr_handle); */
	boot_load_page = BOOTLOADPAGE_BASEADDR;
	status = proc_mgr_translate_addr(procmgr_handle,
			(void *) &sh_addr_base,
			PROC_MGR_ADDRTYPE_MASTERKNLVIRT,
			(void *) boot_load_page,
			PROC_MGR_ADDRTYPE_SLAVEVIRT);
	if (status < 0) {
		printk(KERN_ERR "proc_mgr_translate_addr [0x%x]\n",
			status);
	} else {
		/* Zero out the boot load page */
		memset((void *) sh_addr_base, 0, BOOTLOADPAGE_BASESIZE);

		/* Set the boot load page address */
		sysmgr_set_boot_load_page(proc_id, sh_addr_base);

		/* Get the written entry */
		local_id = multiproc_get_id(NULL);
		sharedregion_get_table_info(SMHEAP_SRINDEX, local_id, &info);
		platform_sm_heap_phys_addr = sysmemmgr_translate(
						platform_sm_heap_virt_addr,
						sysmemmgr_xltflag_kvirt2phys);
		info.base = (void *) platform_sm_heap_phys_addr;

		/* Write info the boot load page */
		nwrite = sysmgr_put_object_config(proc_id,
					(void *) &info,
					SYSMGR_CMD_SHAREDREGION_ENTRY_START,
					sizeof(struct sharedregion_info));
		WARN_ON(nwrite != sizeof(struct sharedregion_info));
	}
}
EXPORT_SYMBOL(platform_load_callback);


/*
 * ======== platform_start_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in started state.
 *  FIXME: logic would change completely in the final system.
 */
void platform_start_callback(void *arg)
{
	s32 status = 0;
	u16 local_id = MULTIPROC_INVALIDID;
	u16 proc_id = (u32) arg;
	u32 nread = 0;
	u32 i = 0;
	u32 cmd_id;
	u32 sh_addr;

	struct notify_ducatidrv_params notify_shm_params;
	struct gatepeterson_params gate_params;
	struct nameserver_remotenotify_params nsr_params;
	struct heapbuf_params heap_params;
	struct messageq_transportshm_params msgqt_params;
	struct sharedregion_config sr_config;
	struct sharedregion_info info;

	struct platform_notify_ducatidrv_params pnds_params;
	struct platform_heapbuf_params phb_params;
	struct platform_gaterpeterson_params pgp_params;
	struct platform_nameserver_remotenotify_params pnsrn_params;
	struct platform_messageq_transportshm_params pmqt_params;
	/*u32 proc_ids[2];*/

	/* Wait for slave to write the scalability info */
	sysmgr_wait_for_scalability_info(proc_id);
	/* Read the scalability info */
	do {
		nread = sysmgr_get_object_config(proc_id, (void *) &pc_params,
				SYSMGR_CMD_SCALABILITY,
				sizeof(struct platform_proc_config_params));
	} while (nread != sizeof(struct platform_proc_config_params));

	if (status >= 0) {
		local_id = multiproc_get_id(NULL);
		status = multiproc_set_local_id(local_id);
		if (status < 0) {
			status = SYSMGR_E_FAIL;
			goto multiproc_fail;
		}
	}

	if (pc_params.use_notify) {
		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &pnds_params,
					PLATFORM_CMD_NOTIFYDRIVER,
					sizeof(struct \
					platform_notify_ducatidrv_params));
		} while (nread != \
			sizeof(struct platform_notify_ducatidrv_params));
		sh_addr = (u32)sharedregion_get_ptr((u32 *)
						pnds_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		notify_ducatidrv_params_init(NULL, &notify_shm_params);
		notify_shm_params.shared_addr = sh_addr;
		notify_shm_params.shared_addr_size = \
						pnds_params.shared_mem_size;
		notify_shm_params.num_events = NOTIFY_MAX_EVENTS;
		notify_shm_params.num_reserved_events = \
						NOTIFY_NUMRESERVEDEVENTS;
		notify_shm_params.send_event_poll_count = \
						NOTIFY_SENDEVENTPOLLCOUNT;
		notify_shm_params.recv_int_id = BASE_DUCATI2ARM_INTID;
		notify_shm_params.send_int_id = BASE_ARM2DUCATI_INTID;
		notify_shm_params.remote_proc_id = proc_id;

		/* Create instance of Notify Ducati Driver */
		platform_notifydrv_handle = notify_ducatidrv_create(
						"NOTIFYDRIVER_DUCATI",
						&notify_shm_params);
		if (platform_notifydrv_handle == NULL) {
			status = SYSMGR_E_FAIL;
			goto notify_ducatidrv_create_fail;
		}
	}

	if (pc_params.use_nameserver) {
		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &pgp_params,
					PLATFORM_CMD_GPNSRN,
					sizeof(struct \
						platform_gaterpeterson_params));
		} while (nread != sizeof(struct platform_gaterpeterson_params));
		sh_addr = (u32)sharedregion_get_ptr((u32 *)
						pgp_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		gatepeterson_params_init(NULL, &gate_params);
		gate_params.shared_addr = (void *) sh_addr;
		gate_params.shared_addr_size = pgp_params.shared_mem_size;
		do {
			status = gatepeterson_open(&platform_nsrn_gate_handle,
						&gate_params);
		} while (status == -ENXIO);

		if (status < 0) {
			status = SYSMGR_E_FAIL;
			goto gatepeterson_open_fail;
		}

		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &pnsrn_params,
					PLATFORM_CMD_NSRN,
					sizeof(struct \
				platform_nameserver_remotenotify_params));
		} while (nread != \
			sizeof(struct platform_nameserver_remotenotify_params));
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						pnsrn_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		/*
		 *  Create the NameServerRemote implementation that is used to
		 *  communicate with the remote processor. It uses some shared
		 *  memory and the Notify module.
		 *
		 *  Note that this implementation uses Notify to communicate, so
		 *  interrupts need to be enabled.
		 */
		nameserver_remotenotify_params_init(NULL, &nsr_params);
		nsr_params.notify_driver = platform_notifydrv_handle;
		nsr_params.notify_event_no = pnsrn_params.notify_event_no;
		nsr_params.shared_addr = (void *) sh_addr;
		nsr_params.shared_addr_size = pnsrn_params.shared_mem_size;
		nsr_params.gate = (void *) platform_nsrn_gate_handle;
		platform_nsrn_handle = nameserver_remotenotify_create(proc_id,
							&nsr_params);
		if (platform_nsrn_handle == NULL) {
			status = SYSMGR_E_FAIL;
			goto nameserver_remotenotify_create_fail;
		}
	}

	if (pc_params.use_heapbuf) {
		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &pgp_params,
					PLATFORM_CMD_GPHEAPBUF,
					sizeof(struct \
						platform_gaterpeterson_params));
		} while (nread != sizeof(struct platform_gaterpeterson_params));
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						pgp_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		gatepeterson_params_init(NULL, &gate_params);
		gate_params.shared_addr = (void *) sh_addr;
		gate_params.shared_addr_size = pgp_params.shared_mem_size;
		do {
			status = gatepeterson_open(&platform_heap_gate_handle,
							&gate_params);
		} while (status == -ENXIO);
		if (status < 0) {
			status = SYSMGR_E_FAIL;
			goto gatepeterson_open_fail;
		}

		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &phb_params,
					PLATFORM_CMD_HEAPBUF,
					sizeof(struct platform_heapbuf_params));
		} while (nread != sizeof(struct platform_heapbuf_params));
		/* Create the heap. */
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						phb_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		heapbuf_params_init(NULL, &heap_params);
		heap_params.name = HEAPNAME;
		heap_params.shared_addr = (void *) sh_addr;
		heap_params.align = HEAPBUF_ALIGN;
		heap_params.num_blocks = phb_params.num_blocks;
		heap_params.block_size = phb_params.block_size;
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						phb_params.shared_buf_size);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		heap_params.shared_buf_size = phb_params.shared_buf_size;
		heap_params.shared_buf = (void *) sh_addr;
		heap_params.gate = platform_heap_gate_handle;
		heap_params.shared_addr_size = phb_params.shared_mem_size;
		do {
			status = heapbuf_open(&platform_heap_handle,
						&heap_params);
		} while (status == -ENXIO);
		if (status < 0) {
			status = SYSMGR_E_FAIL;
			goto heapbuf_open_fail;
		}
	}

	if (pc_params.use_messageq) {
		do {
			nread = sysmgr_get_object_config(proc_id, &pgp_params,
					PLATFORM_CMD_GPMQT,
					sizeof(struct \
						platform_gaterpeterson_params));
		} while (nread != sizeof(struct platform_gaterpeterson_params));
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						pgp_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		gatepeterson_params_init(NULL, &gate_params);
		gate_params.shared_addr = (void *) sh_addr;
		gate_params.shared_addr_size = pgp_params.shared_mem_size;
		do {
			status = gatepeterson_open(&platform_mqt_gate_handle,
						&gate_params);
		} while (status == -ENXIO);
		if (status < 0) {
			status = SYSMGR_E_FAIL;
			goto gatepeterson_open_fail;
		}

		do {
			nread = sysmgr_get_object_config(proc_id,
					(void *) &pmqt_params, PLATFORM_CMD_MQT,
					sizeof(struct \
					platform_messageq_transportshm_params));
		} while (nread != sizeof(
				struct platform_messageq_transportshm_params));
		/* Register this heap with platform_messageq */
		messageq_register_heap(platform_heap_handle, HEAPID);
		sh_addr = (u32) sharedregion_get_ptr((u32 *)
						pmqt_params.shared_mem_addr);
		if (sh_addr == (u32)NULL) {
			status = SYSMGR_E_FAIL;
			goto sharedregion_getptr_fail;
		}
		messageq_transportshm_params_init(NULL, &msgqt_params);
		msgqt_params.shared_addr = (void *) sh_addr;
		msgqt_params.notify_event_no = pmqt_params.notify_event_no;
		msgqt_params.notify_driver = platform_notifydrv_handle;
		msgqt_params.shared_addr_size = pmqt_params.shared_mem_size;
		msgqt_params.gate = platform_mqt_gate_handle;
		platform_transport_shm_handle = messageq_transportshm_create(
							proc_id, &msgqt_params);
		if (platform_transport_shm_handle == NULL) {
			status = SYSMGR_E_FAIL;
			goto messageq_transportshm_create_fail;
		}
	}

	if (status >= 0) {
		/* Wait for slave to complete the setup */
		sysmgr_wait_for_slave_setup(proc_id);

		/* Now get the Shared region entries that may have been created
		 * by Slave, but actual physical memory is not assigned to
		 * those entries, only virtual DSP memory exists.
		 */
		sharedregion_get_config(&sr_config);
		for (i = 0; i < sr_config.max_regions; i++) {
			cmd_id = SYSMGR_CMD_SHAREDREGION_ENTRY_START + i;
			nread = sysmgr_get_object_config(proc_id,
					(void *) &info, cmd_id,
					sizeof(struct sharedregion_info));
			if (nread == sizeof(struct sharedregion_info)) {
				/* FIXME: Do the DMM and convert the entry into
				 * kernel virtual address and put it in the
				 * shared region for host */
			}
		}
	}

messageq_transportshm_create_fail:
	printk(KERN_ERR "platform_start_callback: "
		"messageq_transportshm_create failed status[0x%x]", status);
	goto exit;
heapbuf_open_fail:
	printk(KERN_ERR "platform_start_callback: gatepeterson_open "
		"failed status[0x%x]", status);
	goto exit;
nameserver_remotenotify_create_fail:
	printk(KERN_ERR "platform_start_callback: "
		"nameserver_remotenotify_create failed status[0x%x]", status);
	goto exit;
gatepeterson_open_fail:
	printk(KERN_ERR "platform_start_callback: gatepeterson_open "
		"failed status[0x%x]", status);
	goto exit;
notify_ducatidrv_create_fail:
	printk(KERN_ERR "platform_start_callback: notify_ducatidrv_create "
		"failed status[0x%x]", status);
	goto exit;
sharedregion_getptr_fail:
	printk(KERN_ERR "platform_start_callback: sharedregion_get_ptr failed"
		" status[0x%x]", status);
	goto exit;
multiproc_fail:
	printk(KERN_ERR "platform_start_callback: multiproc_set_local_id failed"
		" status[0x%x]", status);
exit:
	return;
}
EXPORT_SYMBOL(platform_start_callback);
/* FIXME: since application has to call this API for now */


/*
 * ======== platform_stop_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in stopped state.
 *  FIXME: logic would change completely in the final system.
 */
void platform_stop_callback(void *arg)
{
	s32 status = 0;

	if (pc_params.use_messageq) {
		/* Finalize drivers */
		status = gatepeterson_close(&platform_mqt_gate_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : mqt "
				"gatepeterson_close failed [0x%x]", status);
		}

		status = messageq_transportshm_delete(
				&platform_transport_shm_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : "
				"messageq_transportshm_delete failed [0x%x]",
				status);
		}
	}

	if (pc_params.use_nameserver) {
		status = gatepeterson_close(&platform_nsrn_gate_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : nsrn"
				"gatepeterson_close failed [0x%x]", status);
		}

		status = nameserver_remotenotify_delete(&platform_nsrn_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : "
				"nameserver_remotenotify_delete failed [0x%x]",
				status);
		}
	}

	if (pc_params.use_heapbuf) {
		status = messageq_unregister_heap(HEAPID);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : "
				"messageq_unregister_heap failed [0x%x]",
				status);
		}

		status = gatepeterson_close(&platform_heap_gate_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : heap"
				"gatepeterson_close failed [0x%x]", status);
		}

		status = heapbuf_close(&platform_heap_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : "
				"heapbuf_close failed [0x%x]", status);
		}
	}

	if (pc_params.use_notify) {
		status = notify_ducatidrv_delete(
				(struct notify_driver_object **)
				&platform_notifydrv_handle);
		if (status < 0) {
			printk(KERN_ERR "platform_stop_callback : "
				"notify_ducatidrv_delete failed [0x%x]",
				status);
		}
	}

	status = sharedregion_remove(0);
	if (status < 0) {
		printk(KERN_ERR "platform_stop_callback : "
			"sharedregion_remove failed [0x%x]", status);
	}
}
EXPORT_SYMBOL(platform_stop_callback);
/* FIXME: since application has to call this API for now */
