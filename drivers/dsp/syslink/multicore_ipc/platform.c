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



/* SysLink device specific headers */
#include "../procmgr/proc4430/proc4430.h"

/* Module level headers */
#include <multiproc.h>
#include <platform.h>
#include <gatemp.h>
#include <gatepeterson.h>
#if 0
#include <gatehwspinlock.h>*/
#endif
#include <sharedregion.h>
#include <listmp.h>
#include <_listmp.h>
#include <heap.h>
#include <heapbufmp.h>
#include <heapmemmp.h>
#include <messageq.h>
#include <transportshm.h>
#include <notify.h>
#include <ipc.h>

#include <notify_ducatidriver.h>
#include <nameserver.h>
#include <nameserver_remote.h>
#include <nameserver_remotenotify.h>
#include <procmgr.h>

#include <platform_mem.h>


/** ============================================================================
 *  Macros.
 *  ============================================================================
 */
#define RESETVECTOR_SYMBOL          "_Ipc_ResetVector"

/** ============================================================================
 *  Application specific configuration, please change these value according to
 *  your application's need.
 *  ============================================================================
 */
/*! @brief Start of IPC shared memory */
#define SHAREDMEMORY_PHY_BASEADDR	CONFIG_DUCATI_BASEIMAGE_PHYS_ADDR
#define SHAREDMEMORY_PHY_BASESIZE	0x00100000

/*! @brief Start of IPC shared memory for SysM3 */
#define SHAREDMEMORY_PHY_BASEADDR_SYSM3	SHAREDMEMORY_PHY_BASEADDR
#define SHAREDMEMORY_PHY_BASESIZE_SYSM3	0x00054000

/*! @brief Start of IPC shared memory AppM3 */
#define SHAREDMEMORY_PHY_BASEADDR_APPM3 (SHAREDMEMORY_PHY_BASEADDR + 0x55000)
#define SHAREDMEMORY_PHY_BASESIZE_APPM3 0x00054000

/*! @brief Start of IPC SHM for SysM3 */
#define SHAREDMEMORY_SLV_VRT_BASEADDR_SYSM3	0xA0000000
#define SHAREDMEMORY_SLV_VRT_BASESIZE_SYSM3	0x00055000

/*! @brief Start of IPC SHM for AppM3 */
#define SHAREDMEMORY_SLV_VRT_BASEADDR_APPM3	0xA0055000
#define SHAREDMEMORY_SLV_VRT_BASESIZE_APPM3	0x00055000

/*! @brief Start of Boot load page for SysM3 */
#define BOOTLOADPAGE_SLV_VRT_BASEADDR_SYSM3	0xA0054000
#define BOOTLOADPAGE_SLV_VRT_BASESIZE_SYSM3	0x00001000

/*! @brief Start of Boot load page for AppM3 */
#define BOOTLOADPAGE_SLV_VRT_BASEADDR_APPM3	0xA00A9000
#define BOOTLOADPAGE_SLV_VRT_BASESIZE_APPM3	0x00001000

/*! @brief Start of SW DMM shared memory */
#define SHAREDMEMORY_SWDMM_PHY_BASEADDR (SHAREDMEMORY_PHY_BASEADDR + 0x2400000)
#define SHAREDMEMORY_SWDMM_PHY_BASESIZE	0x00C00000

/*! @brief Start of SW DMM SHM for Ducati */
#define SHAREDMEMORY_SWDMM_SLV_VRT_BASEADDR	0x81300000
#define SHAREDMEMORY_SWDMM_SLV_VRT_BASESIZE	0x00C00000

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

struct platform_transportshm_params {
	u32 shared_mem_addr;	/* Shared memory address */
	u32 shared_mem_size;	/* Shared memory size */
	u32 notify_event_no;	/* Notify Event number */
};

/** ============================================================================
 *  Application specific configuration, please change these value according to
 *  your application's need.
 *  ============================================================================
 */
/*
 * Structure defining config parameters for overall System.
 */
struct platform_config {
	struct multiproc_config			multiproc_config;
	/* multiproc_config parameter */

	struct gatemp_config			gatemp_config;
	/* gatemp_config parameter */

	struct gatepeterson_config		gatepeterson_config;
	/* gatepeterson_config parameter */

#if 0
	struct gatehwspinlock_config		gatehwspinlock_config;
	/* gatehwspinlock parameter */
#endif

	struct sharedregion_config		sharedregion_config;
	/* sharedregion_config parameter */

	struct messageq_config			messageq_config;
	/* messageq_config parameter */

	struct notify_config			notify_config;
	/* notify config parameter */

	struct proc_mgr_config			proc_mgr_config;
	/* processor manager config parameter */

	struct heapbufmp_config			heapbufmp_config;
	/* heapbufmp_config parameter */

	struct heapmemmp_config			heapmemmp_config;
	/* heapmemmp_config parameter */
#if 0

	struct heapmultibuf_config		heapmultibuf_config;
	/* heapmultibuf_config parameter */
#endif
	struct listmp_config			listmp_config;
	/* listmp_config parameter */

	struct transportshm_config		transportshm_config;
	/* transportshm_config parameter */
#if 0
	struct ringio_config			ringio_config;
	/* ringio_config parameter */

	struct ringiotransportshm_config	ringiotransportshm_config;
	/* ringiotransportshm_config parameter */
#endif
	struct notify_ducatidrv_config		notify_ducatidrv_config;
	/* notify_ducatidrv_config parameter */

	struct nameserver_remotenotify_config	nameserver_remotenotify_config;
	/* nameserver_remotenotify_config parameter */
#if 0
	struct clientnotifymgr_config		clinotifymgr_config_params;
	/* clientnotifymgr_config parameter */

	struct frameqbufmgr_config		frameqbufmgr_config_params;
	/* frameqbufmgr_config parameter */

	struct frameq_config			frameq_config_params;
	/* frameq_config parameter */
#endif
};


/* struct embedded into slave binary */
struct platform_slave_config {
	u32 cache_line_size;
	u32 br_offset;
	u32 sr0_memory_setup;
	u32 setup_messageq;
	u32 setup_notify;
	u32 proc_sync;
	u32 num_srs;
};

struct platform_proc_config_params {
	u32 use_notify;
	u32 use_messageq;
	u32 use_heapbuf;
	u32 use_frameq;
	u32 use_ring_io;
	u32 use_listmp;
	u32 use_nameserver;
};

/* shared region configuration */
struct platform_slave_sr_config {
	u32 entry_base;
	u32 entry_len;
	u32 owner_proc_id;
	u32 id;
	u32 create_heap;
	u32 cache_line_size;
};

/* Shared region configuration information for host side. */
struct platform_host_sr_config {
	u16 ref_count;
};

/* structure for platform instance */
struct platform_object {
	void				*pm_handle;
	/* handle to the proc_mgr instance used */
	void				*phandle;
	/* handle to the processor instance used */
	struct platform_slave_config	slave_config;
	/* slave embedded config */
	struct platform_slave_sr_config	*slave_sr_config;
	/* shared region details from slave */
};


/* structure for platform instance */
struct platform_module_state {
	bool multiproc_init_flag;
	/* multiproc initialize flag */
	bool gatemp_init_flag;
	/* gatemp initialize flag */
	bool gatepeterson_init_flag;
	/* gatepeterson initialize flag */
	bool gatehwspinlock_init_flag;
	/* gatehwspinlock initialize flag */
	bool sharedregion_init_flag;
	/* sharedregion initialize flag */
	bool listmp_init_flag;
	/* listmp initialize flag */
	bool messageq_init_flag;
	/* messageq initialize flag */
	bool ringio_init_flag;
	/* ringio initialize flag */
	bool notify_init_flag;
	/* notify initialize flag */
	bool proc_mgr_init_flag;
	/* processor manager initialize flag */
	bool heapbufmp_init_flag;
	/* heapbufmp initialize flag */
	bool heapmemmp_init_flag;
	/* heapmemmp initialize flag */
	bool heapmultibuf_init_flag;
	/* heapbufmp initialize flag */
	bool nameserver_init_flag;
	/* nameserver initialize flag */
	bool transportshm_init_flag;
	/* transportshm initialize flag */
	bool ringiotransportshm_init_flag;
	/* ringiotransportshm initialize flag */
	bool notify_ducatidrv_init_flag;
	/* notify_ducatidrv initialize flag */
	bool nameserver_remotenotify_init_flag;
	/* nameserverremotenotify initialize flag */
	bool clientnotifymgr_init_flag;
	/* clientnotifymgr initialize flag */
	bool frameqbufmgr_init_flag;
	/* frameqbufmgr initialize flag */
	bool frameq_init_flag;
	/* frameq initialize flag */
	bool platform_init_flag;
	/* flag to indicate platform initialization status */
};


/* =============================================================================
 * GLOBALS
 * =============================================================================
 */
static struct platform_object platform_objects[MULTIPROC_MAXPROCESSORS];
static struct platform_module_state platform_module_state;
static struct platform_module_state *platform_module = &platform_module_state;
static u16 platform_num_srs_unmapped;
static struct platform_host_sr_config *platform_host_sr_config;

/* ============================================================================
 *  Forward declarations of internal functions.
 * ============================================================================
 */
static int _platform_setup(void);
static int _platform_destroy(void);

/* function to read slave memory */
static int
_platform_read_slave_memory(u16 proc_id,
			    u32 addr,
			    void *value,
			    u32 *num_bytes);

/* function to write slave memory */
static int
_platform_write_slave_memory(u16 proc_id,
			     u32 addr,
			     void *value,
			     u32 *num_bytes);


/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
/*!
 *  @brief  Number of slave memory entries for OMAP4430.
 */
#define NUM_MEM_ENTRIES			3

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
		"DUCATI_SHM_SYSM3",	/* NAME	 : Name of the memory region */
		SHAREDMEMORY_PHY_BASEADDR_SYSM3,
		/* PHYSADDR	 : Physical address */
		SHAREDMEMORY_SLV_VRT_BASEADDR_SYSM3,
		/* SLAVEVIRTADDR  : Slave virtual address */
		(u32) -1u,
		/* MASTERVIRTADDR : Master virtual address (if known) */
		SHAREDMEMORY_SLV_VRT_BASESIZE_SYSM3,
		/* SIZE		 : Size of the memory region */
		true,		/* SHARE : Shared access memory? */
	},
	{
		"DUCATI_SHM_APPM3",	/* NAME	 : Name of the memory region */
		SHAREDMEMORY_PHY_BASEADDR_APPM3,
		/* PHYSADDR : Physical address */
		SHAREDMEMORY_SLV_VRT_BASEADDR_APPM3,
		/* SLAVEVIRTADDR  : Slave virtual address */
		(u32) -1u,
		/* MASTERVIRTADDR : Master virtual address (if known) */
		SHAREDMEMORY_SLV_VRT_BASESIZE_APPM3,
		/* SIZE		 : Size of the memory region */
		true,		/* SHARE : Shared access memory? */
	},
	{
		"DUCATI_SHM_SWDMM",	/* NAME	 : Name of the memory region */
		SHAREDMEMORY_SWDMM_PHY_BASEADDR,
		/* PHYSADDR	     : Physical address */
		SHAREDMEMORY_SWDMM_SLV_VRT_BASEADDR,
		/* SLAVEVIRTADDR  : Slave virtual address */
		(u32) -1u,
		/* MASTERVIRTADDR : Master virtual address (if known) */
		SHAREDMEMORY_SWDMM_SLV_VRT_BASESIZE,
		/* SIZE		: Size of the memory region */
		true,		/* SHARE : Shared access memory? */
	}
};





/* =============================================================================
 * APIS
 * =============================================================================
 */

/*
 * ======== platform_get_config =======
 *  function to get the default values for confiurations.
 */
void
platform_get_config(struct platform_config *config)
{
	int status = PLATFORM_S_SUCCESS;

	BUG_ON(config == NULL);
	if (config == NULL) {
		status = -EINVAL;
		goto exit;
	}

	/* get the gatepeterson default config */
	multiproc_get_config(&config->multiproc_config);

	/* get the gatemp default config */
	gatemp_get_config(&config->gatemp_config);

	/* get the gatepeterson default config */
	gatepeterson_get_config(&config->gatepeterson_config);

#if 0
	/* get the gatehwspinlock default config */
	gatehwspinlock_get_config(&config->gatehwspinlock_config);
#endif

	/* get the sharedregion default config */
	sharedregion_get_config(&config->sharedregion_config);

	/* get the messageq default config */
	messageq_get_config(&config->messageq_config);

	/* get the notify default config */
	notify_get_config(&config->notify_config);

	/* get the procmgr default config */
	proc_mgr_get_config(&config->proc_mgr_config);

	/* get the heapbufmpfault config */
	heapbufmp_get_config(&config->heapbufmp_config);

	/* get the heapmemmpfault config */
	heapmemmp_get_config(&config->heapmemmp_config);
#if 0
	/* get the heapmultibuf default config */
	heapmultibuf_get_config(&config->heapmultibuf_config
#endif
	/* get the listmp default config */
	listmp_get_config(&config->listmp_config);

	/* get the transportshm default config */
	transportshm_get_config(&config->transportshm_config);
	/* get the notifyshmdriver default config */
	notify_ducatidrv_get_config(&config->notify_ducatidrv_config);

	/* get the nameserver_remotenotify default config */
	nameserver_remotenotify_get_config(&config->
				nameserver_remotenotify_config);
#if 0
	/* get the clientnotifymgr default config */
	clientnotifymgr_get_config(&config->clinotifymgr_config_params);

	/*  get the frameqbufmgr default config */
	frameqbufmgr_get_config(&config->frameqbufmgr_config_params);
	/*  get the frameq default config */
	frameq_get_config(&config->frameqcfg_params);

	/*  get the ringio default config */
	ringio_get_config(&config->ringio_config);

	/*  get the ringiotransportshm default config */
	ringiotransportshm_get_config(&config->ringiotransportshm_config);
#endif
exit:
	if (status < 0)
		printk(KERN_ERR "platform_get_config failed! status = 0x%x\n",
								status);
	return;
}


/*
 * ======== platform_override_config ======
 * Function to override the default configuration values.
 *
 */
int
platform_override_config(struct platform_config *config)
{
	int status = PLATFORM_S_SUCCESS;

	BUG_ON(config == NULL);

	if (config == NULL) {
		status = -EINVAL;
		goto exit;
	}

	/* Override the multiproc_config default config */
	config->multiproc_config.num_processors = 4;
	config->multiproc_config.id = 0;
	strcpy(config->multiproc_config.name_list[0], "MPU");
	strcpy(config->multiproc_config.name_list[1], "Tesla");
	strcpy(config->multiproc_config.name_list[2], "SysM3");
	strcpy(config->multiproc_config.name_list[3], "AppM3");

	/* Override the gatepeterson default config */

	/* Override the Sharedregion default config */
	config->sharedregion_config.cache_line_size = 128;

	/* Override the LISTMP default config */

	/* Override the MESSAGEQ default config */

	/* Override the NOTIFY default config */

	/* Override the PROCMGR default config */

	/* Override the HeapBuf default config */

	/* Override the LISTMPSHAREDMEMORY default config */

	/* Override the MESSAGEQTRANSPORTSHM default config */

	/* Override the NOTIFYSHMDRIVER default config */

	/* Override the NAMESERVERREMOTENOTIFY default config */

	/* Override the  ClientNotifyMgr default config */
	/* Override the  FrameQBufMgr default config */

	/* Override the FrameQ default config */


exit:
	if (status < 0)
		printk(KERN_ERR "platform_override_config failed! status "
				"= 0x%x\n", status);
	return status;
}

/*
 * ======= platform_setup ========
 *       function to setup platform.
 *              TBD: logic would change completely in the final system.
 */
int
platform_setup(void)
{
	int status = PLATFORM_S_SUCCESS;
	struct platform_config _config;
	struct platform_config *config;
#if 0
	struct platform_mem_map_info m_info;
#endif

	platform_get_config(&_config);
	config = &_config;

	platform_override_config(config);

	status = multiproc_setup(&(config->multiproc_config));
	if (status < 0) {
		printk(KERN_ERR "platform_setup : multiproc_setup "
			"failed [0x%x]\n", status);
	} else {
		printk(KERN_ERR "platform_setup : status [0x%x]\n", status);
		platform_module->multiproc_init_flag = true;
	}

	/* Initialize ProcMgr */
	if (status >= 0) {
		status = proc_mgr_setup(&(config->proc_mgr_config));
		if (status < 0) {
			printk(KERN_ERR "platform_setup : proc_mgr_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "proc_mgr_setup : status [0x%x]\n",
				status);
			platform_module->proc_mgr_init_flag = true;
		}
	}

	/* Initialize SharedRegion */
	if (status >= 0) {
		status = sharedregion_setup(&config->sharedregion_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : sharedregion_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "sharedregion_setup : status [0x%x]\n",
				status);
			platform_module->sharedregion_init_flag = true;
		}
	}

	/* Initialize Notify DucatiDriver */
	if (status >= 0) {
		status = notify_ducatidrv_setup(&config->
						notify_ducatidrv_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : "
				"notify_ducatidrv_setup failed [0x%x]\n",
				status);
		} else {
			printk(KERN_ERR "notify_ducatidrv_setup : "
				"status [0x%x]\n", status);
			platform_module->notify_ducatidrv_init_flag = true;
		}
	}

	/* Initialize Notify */
	if (status >= 0) {
		status = notify_setup(&config->notify_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : notify_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "notify_setup : status [0x%x]\n",
				status);
			platform_module->notify_init_flag = true;
		}
	}

	/* Initialize NameServer */
	if (status >= 0) {
		status = nameserver_setup();
		if (status < 0) {
			printk(KERN_ERR "platform_setup : nameserver_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "nameserver_setup : status [0x%x]\n",
				status);
			platform_module->nameserver_init_flag = true;
		}
	}

	/* Initialize GateMP */
	if (status >= 0) {
		status = gatemp_setup(&config->gatemp_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : gatemp_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "gatemp_setup : status [0x%x]\n",
				status);
			platform_module->gatemp_init_flag = true;
		}
	}

	/* Initialize GatePeterson */
	if (status >= 0) {
		status = gatepeterson_setup(&config->gatepeterson_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : gatepeterson_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "gatepeterson_setup : status [0x%x]\n",
				status);
			platform_module->gatepeterson_init_flag = true;
		}
	}

#if 0
	/* Initialize GateHWSpinlock */
	if (status >= 0) {
		m_info.src  = 0x480CA800;
		m_info.size = 0x1000;
		m_info.is_cached = false;
		status = platform_mem_map(&m_info);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : platform_mem_map "
				"failed [0x%x]\n", status);
		} else {
			config->gatehwspinlock_config.num_locks = 32;
			config->gatehwspinlock_config.base_addr = m_info.dst;
			status = gatehwspinlock_setup(&config->
							gatehwspinlock_config);
			if (status < 0) {
				printk(KERN_ERR "platform_setup : "
				"gatehwspinlock_setup failed [0x%x]\n",
				status);
			} else
				platform_module->gatehwspinlock_init_flag =
									true;
		}
	}
#endif

	/* Initialize MessageQ */
	if (status >= 0) {
		status = messageq_setup(&config->messageq_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : messageq_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "messageq_setup : status [0x%x]\n",
				status);
			platform_module->messageq_init_flag = true;
		}
	}
#if 0
	/* Initialize RingIO */
	if (status >= 0) {
		status = ringio_setup(&config->ringio_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : ringio_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "ringio_setup : status [0x%x]\n",
				status);
			platform_module->ringio_init_flag = true;
		}
	}

	/* Initialize RingIOTransportShm */
	if (status >= 0) {
		status = ringiotransportshm_setup(&config->
						ringiotransportshm_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : "
				"ringiotransportshm_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "ringiotransportshm_setup : status "
				"[0x%x]\n", status);
			platform_module->ringiotransportshm_init_flag = true;
		}
	}
#endif
	/* Initialize HeapBufMP */
	if (status >= 0) {
		status = heapbufmp_setup(&config->heapbufmp_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : heapbufmp_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "heapbufmp_setup : status [0x%x]\n",
				status);
			platform_module->heapbufmp_init_flag = true;
		}
	}

	/* Initialize HeapMemMP */
	if (status >= 0) {
		status = heapmemmp_setup(&config->heapmemmp_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : heapmemmp_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "heapmemmp_setup : status [0x%x]\n",
				status);
			platform_module->heapmemmp_init_flag = true;
		}
	}
#if 0
	/* Initialize HeapMultiBuf */
	if (status >= 0) {
		status = heapmultibuf_setup(&config->heapmultibuf_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : heapmultibuf_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "heapmultibuf_setup : status [0x%x]\n",
				status);
			platform_module->heapmultibuf_init_flag = true;
		}
	}
#endif
	/* Initialize ListMP */
	if (status >= 0) {
		status = listmp_setup(
				&config->listmp_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : "
				"listmp_setup failed [0x%x]\n",
				status);
		} else {
			printk(KERN_ERR "listmp_setup : "
				"status [0x%x]\n", status);
			platform_module->listmp_init_flag = true;
		}
	}

	/* Initialize TransportShm */
	if (status >= 0) {
		status = transportshm_setup(
				 &config->transportshm_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : "
				"transportshm_setup failed [0x%x]\n",
				status);
		} else {
			printk(KERN_ERR "transportshm_setup : "
				"status [0x%x]\n", status);
			platform_module->transportshm_init_flag = true;
		}
	}

	/* Initialize NameServerRemoteNotify */
	if (status >= 0) {
		status = nameserver_remotenotify_setup(
				 &config->nameserver_remotenotify_config);
		if (status < 0) {
			printk(KERN_ERR "platform_setup : "
				"nameserver_remotenotify_setup failed "
				"[0x%x]\n", status);
		} else {
			printk(KERN_ERR "nameserver_remotenotify_setup : "
				"status [0x%x]\n", status);
			platform_module->nameserver_remotenotify_init_flag =
									true;
		}
	}
#if 0
	/* Get the ClientNotifyMgr default config */
	if (status >= 0) {
		status = ClientNotifyMgr_setup(&config->cliNotifyMgrCfgParams);
		if (status < 0)
			GT_setFailureReason(curTrace,
					 GT_4CLASS,
					 "Platform_setup",
					 status,
					 "ClientNotifyMgr_setup failed!");
		else
			Platform_module->clientNotifyMgrInitFlag = true;
	}

	/* Get the FrameQBufMgr default config */
	if (status >= 0) {
		status = FrameQBufMgr_setup(&config->frameQBufMgrCfgParams);
		if (status < 0)
			GT_setFailureReason(curTrace,
					 GT_4CLASS,
					 "Platform_setup",
					 status,
					 "FrameQBufMgr_setup failed!");
		else
			Platform_module->frameQBufMgrInitFlag = true;
	}
	/* Get the FrameQ default config */
	if (status >= 0) {
		status = FrameQ_setup(&config->frameQCfgParams);
		if (status < 0)
			GT_setFailureReason(curTrace,
				 GT_4CLASS,
				 "Platform_setup",
				 status,
				 "FrameQ_setup failed!");
	else
		Platform_module->frameQInitFlag = true;
	}
#endif

	/* Initialize Platform */
	if (status >= 0) {
		status = _platform_setup();
		if (status < 0) {
			printk(KERN_ERR "platform_setup : _platform_setup "
				"failed [0x%x]\n", status);
		} else {
			printk(KERN_ERR "_platform_setup : status [0x%x]\n",
				status);
			platform_module->platform_init_flag = true;
		}

	}

	return status;
}


/*
 * =========== platform_destroy ==========
 *  Function to destroy the System.
 */
int
platform_destroy(void)
{
	int status = PLATFORM_S_SUCCESS;
#if 0
	struct platform_mem_unmap_info u_info;
#endif

	/* Finalize Platform module*/
	if (platform_module->platform_init_flag == true) {
		status = _platform_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : _platform_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->platform_init_flag = false;
		}
	}
#if 0
	/* Finalize Frame module */
	if (Platform_module->frameQInitFlag == true) {
		status = FrameQ_destroy();
		if (status < 0)
			GT_setFailureReason(curTrace,
					 GT_4CLASS,
					 "Platform_destroy",
					 status,
					 "FrameQ_destroy failed!");
		else
			Platform_module->frameQInitFlag = false;
	}

	/* Finalize FrameQBufMgr module */
	if (Platform_module->frameQBufMgrInitFlag == true) {
		status = FrameQBufMgr_destroy();
		if (status < 0)
			GT_setFailureReason(curTrace,
					 GT_4CLASS,
					 "Platform_destroy",
					 status,
					 "FrameQBufMgr_destroy failed!");
		else
			Platform_module->frameQBufMgrInitFlag = false;
	}

	/* Finalize ClientNotifyMgr module */
	if (Platform_module->clientNotifyMgrInitFlag == true) {
		status = ClientNotifyMgr_destroy();
		if (status < 0)
			GT_setFailureReason(curTrace,
					 GT_4CLASS,
					 "Platform_destroy",
					 status,
					 "ClientNotifyMgr_destroy failed!");
		else
			Platform_module->clientNotifyMgrInitFlag = false;
	}
#endif
	/* Finalize NameServerRemoteNotify module */
	if (platform_module->nameserver_remotenotify_init_flag == true) {
		status = nameserver_remotenotify_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"nameserver_remotenotify_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->nameserver_remotenotify_init_flag \
				= false;
		}
	}

	/* Finalize TransportShm module */
	if (platform_module->transportshm_init_flag == true) {
		status = transportshm_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"transportshm_destroy failed "
				"[0x%x]\n", status);
		} else {
			platform_module->transportshm_init_flag = \
				false;
		}
	}

	/* Finalize ListMP module */
	if (platform_module->listmp_init_flag == true) {
		status = listmp_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"listmp_destroy failed [0x%x]\n",
				status);
		} else {
			platform_module->listmp_init_flag = \
				false;
		}
	}
#if 0
	/* Finalize HeapMultiBuf module */
	if (platform_module->heapmultibuf_init_flag == true) {
		status = heapmultibuf_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"heapmultibuf_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->heapmultibuf_init_flag = false;
		}
	}
#endif
	/* Finalize HeapBufMP module */
	if (platform_module->heapbufmp_init_flag == true) {
		status = heapbufmp_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : heapbufmp_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->heapbufmp_init_flag = false;
		}
	}

	/* Finalize HeapMemMP module */
	if (platform_module->heapmemmp_init_flag == true) {
		status = heapmemmp_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : heapmemmp_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->heapmemmp_init_flag = false;
		}
	}

	/* Finalize MessageQ module */
	if (platform_module->messageq_init_flag == true) {
		status = messageq_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : messageq_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->messageq_init_flag = false;
		}
	}
#if 0
	/* Finalize RingIO module */
	if (platform_module->ringio_init_flag == true) {
		status = ringio_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : ringio_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->ringio_init_flag = false;
		}
	}


	/* Finalize RingIOTransportShm module */
	if (platform_module->ringiotransportshm_init_flag == true) {
		status = ringiotransportshm_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
					"ringiotransportshm_destroy "
					"failed [0x%x]\n", status);
		} else {
			platform_module->ringiotransportshm_init_flag = false;
		}
	}
#endif
	/* Finalize GatePeterson module */
	if (platform_module->gatepeterson_init_flag == true) {
		status = gatepeterson_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"gatepeterson_destroy failed [0x%x]\n", status);
		} else {
			platform_module->gatepeterson_init_flag = false;
		}
	}

#if 0
	/* Finalize GateHWSpinlock module */
	if (platform_module->gatehwspinlock_init_flag == true) {
		status = gatehwspinlock_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"gatehwspinlock_destroy failed "
				"[0x%x]\n", status);
		} else {
			platform_module->gatehwspinlock_init_flag = false;
		}

		u_info.addr = 0x480CA800;
		u_info.size = 0x1000;
		u_info.is_cached = false;
		status = platform_mem_unmap(&u_info);
		if (status < 0)
			printk(KERN_ERR "platform_destroy : platform_mem_unmap"
						" failed [0x%x]\n", status);
	}
#endif

	/* Finalize GateMP module */
	if (platform_module->gatemp_init_flag == true) {
		status = gatemp_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"gatemp_destroy failed [0x%x]\n", status);
		} else {
			platform_module->gatemp_init_flag = false;
		}
	}

	/* Finalize NameServer module */
	if (platform_module->nameserver_init_flag == true) {
		status = nameserver_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : nameserver_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->nameserver_init_flag = false;
		}
	}

	/* Finalize Notify Ducati Driver module */
	if (platform_module->notify_ducatidrv_init_flag == true) {
		status = notify_ducatidrv_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"notify_ducatidrv_destroy failed [0x%x]\n",
				status);
		} else {
			platform_module->notify_ducatidrv_init_flag = false;
		}
	}

	/* Finalize Notify module */
	if (platform_module->notify_init_flag == true) {
		status = notify_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : platform_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->notify_init_flag = false;
		}
	}

	/* Finalize SharedRegion module */
	if (platform_module->sharedregion_init_flag == true) {
		status = sharedregion_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : "
				"sharedregion_destroy failed [0x%x]\n", status);
		} else {
			platform_module->sharedregion_init_flag = false;
		}
	}

	/* Finalize ProcMgr module */
	if (platform_module->proc_mgr_init_flag == true) {
		status = proc_mgr_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : proc_mgr_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->proc_mgr_init_flag = false;
		}
	}

	/* Finalize MultiProc module */
	if (platform_module->multiproc_init_flag == true) {
		status = multiproc_destroy();
		if (status < 0) {
			printk(KERN_ERR "platform_destroy : multiproc_destroy "
				"failed [0x%x]\n", status);
		} else {
			platform_module->multiproc_init_flag = false;
		}
	}



	if (status >= 0)
		memset(platform_objects,
			0,
			(sizeof(struct platform_object) *
				multiproc_get_num_processors()));

	return status;
}

/*
 * ======== platform_setup ========
 *  Purpose:
 *  TBD: logic would change completely in the final system.
 */
s32 _platform_setup(void)
{

	s32 status = 0;
	struct proc4430_config proc_config;
	struct proc_mgr_params params;
	struct proc4430_params proc_params;
	u16 proc_id;
	struct platform_object *handle;
	void *proc_mgr_handle;
	void *proc_mgr_proc_handle;

	proc4430_get_config(&proc_config);
	status = proc4430_setup(&proc_config);
	if (status < 0)
		goto exit;


	/* Get MultiProc ID by name. */
	proc_id = multiproc_get_id("SysM3");
	handle = &platform_objects[proc_id];

	/* Create an instance of the Processor object for OMAP4430 */
	proc4430_params_init(NULL, &proc_params);
	proc_params.num_mem_entries = NUM_MEM_ENTRIES;
	proc_params.mem_entries = mem_entries;
	proc_params.reset_vector_mem_entry = RESET_VECTOR_ENTRY_ID;
	proc_mgr_proc_handle = proc4430_create(proc_id, &proc_params);
	if (proc_mgr_proc_handle == NULL) {
		status = PLATFORM_E_FAIL;
		goto proc_create_fail;
	}

	/* Initialize parameters */
	proc_mgr_params_init(NULL, &params);
	params.proc_handle = proc_mgr_proc_handle;
	proc_mgr_handle = proc_mgr_create(proc_id, &params);
	if (proc_mgr_handle == NULL) {
		status = PLATFORM_E_FAIL;
		goto proc_mgr_create_fail;
	}

	/* SysM3 and AppM3 use the same handle */
	handle->phandle = proc_mgr_proc_handle;
	handle->pm_handle = proc_mgr_handle;

	proc_mgr_handle = NULL;
	proc_mgr_proc_handle = NULL;



	/* Get MultiProc ID by name. */
	proc_id = multiproc_get_id("AppM3");
	handle = &platform_objects[proc_id];

	/* Create an instance of the Processor object for OMAP4430 */
	proc4430_params_init(NULL, &proc_params);
	proc_params.num_mem_entries = NUM_MEM_ENTRIES;
	proc_params.mem_entries = mem_entries;
	proc_params.reset_vector_mem_entry = RESET_VECTOR_ENTRY_ID;
	proc_mgr_proc_handle = proc4430_create(proc_id, &proc_params);
	if (proc_mgr_proc_handle == NULL) {
		status = PLATFORM_E_FAIL;
		goto proc_create_fail;
	}

	/* Initialize parameters */
	proc_mgr_params_init(NULL, &params);
	params.proc_handle = proc_mgr_proc_handle;
	proc_mgr_handle = proc_mgr_create(proc_id, &params);
	if (proc_mgr_handle == NULL) {
		status = PLATFORM_E_FAIL;
		goto proc_mgr_create_fail;
	}

	handle->phandle = proc_mgr_proc_handle;
	handle->pm_handle = proc_mgr_handle;

	return status;
proc_create_fail:
proc_mgr_create_fail:
	/* Clean up created objects */
	_platform_destroy();
exit:
	return status;
}


/*
 * ======== platform_destroy ========
 *  Purpose:
 *  Function to finalize the platform.
 */
s32 _platform_destroy(void)
{
	s32 status = 0;
	struct platform_object *handle;
	int i;

	for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
		handle = &platform_objects[i];

		/* Delete the Processor instances */
		if (handle->phandle != NULL) {
			status = proc4430_delete(&handle->phandle);
			WARN_ON(status < 0);
		}

		if (handle->pm_handle != NULL) {
			status = proc_mgr_delete(&handle->pm_handle);
			WARN_ON(status < 0);
		}
	}

	status = proc4430_destroy();
	WARN_ON(status < 0);

	return status;
}


/*
 * ======== platform_load_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in loaded state.
 */
int platform_load_callback(u16 proc_id, void *arg)
{
	int status = PLATFORM_S_SUCCESS;
	struct platform_object *handle;
	u32 start;
	u32 num_bytes;
	struct sharedregion_entry entry;
	u32 m_addr;
	struct proc_mgr_addr_info ai;
	struct ipc_params ipc_params;
	int i;
	u32 file_id;
	void *pm_handle;

	handle = &platform_objects[multiproc_self()];

	pm_handle = handle->pm_handle;
	file_id = proc_mgr_get_loaded_file_id(pm_handle);

	/* read the symbol from slave binary */
	status = proc_mgr_get_symbol_address(pm_handle,
					     file_id,
					     RESETVECTOR_SYMBOL,
					     &start);
	if (status < 0) {
		status = PLATFORM_E_FAIL;
		goto exit;
	}

	/* Read the slave config */
	num_bytes = sizeof(struct platform_slave_config);
	status = _platform_read_slave_memory(proc_id,
					start,
					&handle->slave_config,
					&num_bytes);
	if (status < 0) {
		status = PLATFORM_E_FAIL;
		goto exit;
	}

	if (platform_host_sr_config == NULL)
		platform_host_sr_config = kmalloc(sizeof(struct
				platform_host_sr_config) * handle->
					slave_config.num_srs, GFP_KERNEL);

	if (platform_host_sr_config == NULL) {
		status = -ENOMEM;
		goto alloced_host_sr_config_exit;
	}

	if (handle->slave_config.num_srs > 0) {
		num_bytes = handle->slave_config.num_srs * sizeof(struct
						platform_slave_sr_config);
		handle->slave_sr_config = kmalloc(num_bytes, GFP_KERNEL);
		if (handle->slave_sr_config == NULL) {
			status = -ENOMEM;
			goto exit;
		} else {
			status = _platform_read_slave_memory(
					proc_id,
					start + sizeof(struct
						platform_slave_config),
					handle->slave_sr_config,
					&num_bytes);
			if (status < 0) {
				status = PLATFORM_E_FAIL;
				goto alloced_slave_sr_config_exit;
			}
		}
	}

	if (status >= 0) {
		ipc_params.setup_messageq = handle->slave_config.
								setup_messageq;
		ipc_params.setup_notify   = handle->slave_config.setup_notify;
		ipc_params.proc_sync      = handle->slave_config.proc_sync;
		status = ipc_create(proc_id, &ipc_params);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto alloced_slave_sr_config_exit;
		}
	}

	/* Setup the shared memory for region with owner == host */
	for (i = 0; i < handle->slave_config.num_srs; i++) {
		status = sharedregion_get_entry(i, &entry);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto alloced_slave_sr_config_exit;
		}
		BUG_ON(!((entry.is_valid == false)
			|| ((entry.is_valid == true)
				&& (entry.len == (handle->
					slave_sr_config[i].entry_len)))));

		platform_host_sr_config[i].ref_count++;

		/* Add the entry only if previously not added */
		if (entry.is_valid == false) {

			/* Translate the slave address to master */
			status = proc_mgr_translate_addr(
				pm_handle,
				(void **)&m_addr,
				PROC_MGR_ADDRTYPE_MASTERPHYS,
				(void *)handle->slave_sr_config[i].entry_base,
				PROC_MGR_ADDRTYPE_SLAVEVIRT);
			if (status < 0) {
				status = PLATFORM_E_FAIL;
				goto alloced_slave_sr_config_exit;
			}

			ai.addr[PROC_MGR_ADDRTYPE_MASTERPHYS] = m_addr;
			ai.addr[PROC_MGR_ADDRTYPE_SLAVEVIRT]  =
				handle->slave_sr_config[i].entry_base;
			ai.size = handle->slave_sr_config[i].entry_len;
			ai.is_cached = false;
			status = proc_mgr_map(pm_handle,
					(PROC_MGR_MAPTYPE_SLAVE
					| PROC_MGR_MAPTYPE_VIRT),
					&ai,
					PROC_MGR_ADDRTYPE_MASTERPHYS);
			if (status < 0) {
				status = PLATFORM_E_FAIL;
				goto alloced_slave_sr_config_exit;
			}

			memset((u32 *)ai.addr[PROC_MGR_ADDRTYPE_MASTERKNLVIRT],
				0, handle->slave_sr_config[i].entry_len);
			memset(&entry, 0, sizeof(struct sharedregion_entry));
			entry.base = (void *)ai.
					addr[PROC_MGR_ADDRTYPE_MASTERKNLVIRT];
			entry.len = handle->slave_sr_config[i].entry_len;
			entry.owner_proc_id = handle->slave_sr_config[i].
							owner_proc_id;
			entry.is_valid = true;
			entry.cache_line_size = handle->slave_sr_config[i].
							cache_line_size;
			entry.create_heap = handle->slave_sr_config[i].
								create_heap;
			sharedregion_set_entry(handle->
						slave_sr_config[i].id, &entry);
		}
	}

	/* Read sr0_memory_setup */
	num_bytes = sizeof(struct platform_slave_config);
	handle->slave_config.sr0_memory_setup = 1;
	status = _platform_write_slave_memory(proc_id,
					      start,
					      &handle->slave_config,
					      &num_bytes);
	if (status < 0) {
		status = PLATFORM_E_FAIL;
		goto alloced_slave_sr_config_exit;
	}

	status = ipc_start();
	if (status < 0) {
		status = PLATFORM_E_FAIL;
		goto alloced_slave_sr_config_exit;
	}

	return 0;

alloced_slave_sr_config_exit:
	kfree(handle->slave_sr_config);

alloced_host_sr_config_exit:
	kfree(platform_host_sr_config);
exit:
	if (status < 0)
		printk(KERN_ERR "platform_load_callback failed, status [0x%x]\n",
			status);

	return status;
}
EXPORT_SYMBOL(platform_load_callback);


/*
 * ======== platform_start_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in started state.
 *  FIXME: logic would change completely in the final system.
 */
int platform_start_callback(u16 proc_id, void *arg)
{
	int status = PLATFORM_S_SUCCESS;

	do {
		status = ipc_attach(proc_id);
	} while (status < 0);

	if (status < 0)
		printk(KERN_ERR "platform_load_callback failed, status [0x%x]\n",
			status);

	return status;
}
EXPORT_SYMBOL(platform_start_callback);
/* FIXME: since application has to call this API for now */


/*
 * ======== platform_stop_callback ========
 *  Purpose:
 *  Function called by proc_mgr when slave is in stopped state.
 *  FIXME: logic would change completely in the final system.
 */
int platform_stop_callback(u16 proc_id, void *arg)
{
	int status = PLATFORM_S_SUCCESS;
	u32 i;
	u32 m_addr;
	struct proc_mgr_addr_info ai;
	struct platform_object *handle;
	void *pm_handle;

	handle = (struct platform_object *)&platform_objects[proc_id];
	pm_handle = handle->pm_handle;

	/* delete the System manager instance here */
	for (i = 0;
		((handle->slave_sr_config != NULL) &&
			(i < handle->slave_config.num_srs));
		i++) {
			platform_host_sr_config[i].ref_count--;
			if (platform_host_sr_config[i].ref_count == 0) {
				platform_num_srs_unmapped++;

			/* Translate the slave address to master */
			status = proc_mgr_translate_addr(pm_handle,
				(void **)&m_addr,
				PROC_MGR_ADDRTYPE_MASTERPHYS,
				(void *)handle->slave_sr_config[i].entry_base,
				PROC_MGR_ADDRTYPE_SLAVEVIRT);
			if (status < 0) {
				status = PLATFORM_E_FAIL;
				continue;
			}
			ai.addr[PROC_MGR_ADDRTYPE_MASTERPHYS] = m_addr;
			ai.addr[PROC_MGR_ADDRTYPE_SLAVEVIRT] =
					handle->slave_sr_config[i].entry_base;
			ai.size = handle->slave_sr_config[i].entry_len;
			ai.is_cached = false;
			status = proc_mgr_unmap(pm_handle,
					(PROC_MGR_MAPTYPE_SLAVE
					| PROC_MGR_MAPTYPE_VIRT),
					&ai,
					PROC_MGR_ADDRTYPE_MASTERPHYS);
		}
	}

	if (platform_num_srs_unmapped == handle->slave_config.num_srs) {
		if (handle->slave_sr_config != NULL) {
			kfree(handle->slave_sr_config);
			handle->slave_sr_config = NULL;
		}
		if (platform_host_sr_config != NULL) {
			kfree(platform_host_sr_config);
			platform_host_sr_config = NULL;
			platform_num_srs_unmapped = 0;
		}
	}

	ipc_detach(proc_id);

	ipc_stop();

	return status;
}
EXPORT_SYMBOL(platform_stop_callback);

/*  ============================================================================
 *  Internal functions
 *  ============================================================================
 */
/* Function to read slave memory */
int
_platform_read_slave_memory(u16 proc_id,
			    u32 addr,
			    void *value,
			    u32 *num_bytes)
{
	int status = 0;
	bool done = false;
	struct platform_object *handle;
	struct proc_mgr_addr_info a_info;
	u32 m_addr;
	void *pm_handle;

	handle = (struct platform_object *)&platform_objects[proc_id];
	BUG_ON(handle == NULL);
	if (handle == NULL) {
		status = -EINVAL;
		goto exit;
	}

	pm_handle = handle->pm_handle;
	BUG_ON(pm_handle == NULL);
	if (pm_handle == NULL) {
		status = -EINVAL;
		goto exit;
	}

	/* Translate the slave address to master address */
	status = proc_mgr_translate_addr(pm_handle,
					(void **)&m_addr,
					PROC_MGR_ADDRTYPE_MASTERPHYS,
					(void *)addr,
					PROC_MGR_ADDRTYPE_SLAVEVIRT);

	if (status >= 0) {
		status = proc_mgr_translate_addr(pm_handle,
						(void **)&m_addr,
						PROC_MGR_ADDRTYPE_MASTERKNLVIRT,
						(void *)m_addr,
						PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status >= 0) {
			memcpy(value, &m_addr, *num_bytes);
			done = true;
		} else {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

	if (done == false) {
		/* Map the address */
		a_info.addr[PROC_MGR_ADDRTYPE_MASTERPHYS] = m_addr;
		a_info.addr[PROC_MGR_ADDRTYPE_SLAVEVIRT] = addr;
		a_info.size = *num_bytes;
		a_info.is_cached = false;
		status = proc_mgr_map(pm_handle,
				     (PROC_MGR_MAPTYPE_VIRT
				     | PROC_MGR_MAPTYPE_SLAVE),
				     &a_info,
				     PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

	if (done == false) {
		status = proc_mgr_read(pm_handle,
				       addr,
				       num_bytes,
				       value);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

	if (done == false) {
		/* Map the address */
		status = proc_mgr_unmap(pm_handle,
				       (PROC_MGR_MAPTYPE_VIRT
				       | PROC_MGR_MAPTYPE_SLAVE),
				       &a_info,
				       PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}
exit:
	return status;
}


/* Function to write slave memory */
int
_platform_write_slave_memory(u16 proc_id,
			     u32 addr,
			     void *value,
			     u32 *num_bytes)
{
	int status = 0;
	bool done = false;
	struct platform_object *handle;
	struct proc_mgr_addr_info a_info;
	u32 m_addr;
	void *pm_handle = NULL;

	handle = (struct platform_object *)&platform_objects[proc_id];
	BUG_ON(handle == NULL);
	if (handle == NULL) {
		status = -EINVAL;
		goto exit;
	}

	pm_handle = handle->pm_handle;
	BUG_ON(pm_handle == NULL);
	if (pm_handle == NULL) {
		status = -EINVAL;
		goto exit;
	}

	/* Translate the slave address to master address */
	status = proc_mgr_translate_addr(pm_handle,
					(void **)&m_addr,
					PROC_MGR_ADDRTYPE_MASTERPHYS,
					(void *)addr,
					PROC_MGR_ADDRTYPE_SLAVEVIRT);

	if (status < 0) {
		status = proc_mgr_translate_addr(pm_handle,
					(void **)&m_addr,
					PROC_MGR_ADDRTYPE_MASTERKNLVIRT,
					(void *)m_addr,
					PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status >= 0) {
			memcpy(&m_addr, value, *num_bytes);
			done = true;
		}
	}

	if (done == false) {
		/* Map the address */
		a_info.addr[PROC_MGR_ADDRTYPE_MASTERPHYS] = m_addr;
		a_info.addr[PROC_MGR_ADDRTYPE_SLAVEVIRT] = addr;
		a_info.size = *num_bytes;
		a_info.is_cached = false;
		status = proc_mgr_map(pm_handle,
				     (PROC_MGR_MAPTYPE_VIRT
				     | PROC_MGR_MAPTYPE_SLAVE),
				     &a_info,
				     PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

	if (done == false) {
		status = proc_mgr_write(pm_handle,
					addr,
					num_bytes,
					value);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

	if (done == false) {
		/* Map the address */
		status = proc_mgr_map(pm_handle,
				     (PROC_MGR_MAPTYPE_VIRT
				     | PROC_MGR_MAPTYPE_SLAVE),
				     &a_info,
				     PROC_MGR_ADDRTYPE_MASTERPHYS);
		if (status < 0) {
			status = PLATFORM_E_FAIL;
			goto exit;
		}
	}

exit:
	return status;
}
