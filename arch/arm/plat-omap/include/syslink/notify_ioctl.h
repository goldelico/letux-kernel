/*
 * notify_driverdefs.h
 *
 * Notify driver support for OMAP Processors.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#if !defined(_NOTIFY_IOCTL_H_)
#define _NOTIFY_IOCTL_H_

/* Linux headers */
#include <linux/ioctl.h>

/* Utilities headers */
#include <syslink/host_os.h>

/* Module headers */
#include <syslink/notify.h>
#include <syslink/notify_shmdriver.h>
#include <syslink/notifydefs.h>


/* Base command ID for Notify */
#define NOTIFYCMDBASE			(0xE0)

/* Base command ID for Notify */
#define NOTIFY_BASE_CMD			(0x170)

/* Command for notify_get_config */
#define CMD_NOTIFY_GETCONFIG		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 1u,		\
					struct notify_cmd_args_get_config)

/* Command for notify_setup */
#define CMD_NOTIFY_SETUP		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 2u,		\
					struct notify_cmd_args_setup)

/* Command for notify_destroy */
#define CMD_NOTIFY_DESTROY		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 3u,		\
					struct notify_cmd_args_destroy)

/* Command for notify_register_event */
#define CMD_NOTIFY_REGISTEREVENT	_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 4u,		\
					struct notify_cmd_args_register_event)

/* Command for notify_unregister_event */
#define CMD_NOTIFY_UNREGISTEREVENT	_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 5u,		\
					struct notify_cmd_args_unregister_event)

/* Command for notify_send_event */
#define CMD_NOTIFY_SENDEVENT		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 6u,		\
					struct notify_cmd_args_send_event)
/* Command for notify_disable */
#define CMD_NOTIFY_DISABLE		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 7u,		\
					struct notify_cmd_args_disable)

/* Command for notify_restore */
#define CMD_NOTIFY_RESTORE		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 8u,		\
					struct notify_cmd_args_restore)

/* Command for notify_disable_event */
#define CMD_NOTIFY_DISABLEEVENT		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 9u,		\
					struct notify_cmd_args_disable_event)

/* Command for notify_enable_event */
#define CMD_NOTIFY_ENABLEEVENT		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 10u,		\
					struct notify_cmd_args_enable_event)

/* Command for notify_attach */
#define CMD_NOTIFY_ATTACH		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 11u,		\
					struct notify_cmd_args_attach)

/* Command for notify_detach */
#define CMD_NOTIFY_DETACH		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 12u,		\
					struct notify_cmd_args_detach)

/* Command for notify_thread_attach */
#define CMD_NOTIFY_THREADATTACH		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 13u,		\
					struct notify_cmd_args)

/* Command for notify_thread_detach */
#define CMD_NOTIFY_THREADDETACH		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 14u,		\
					struct notify_cmd_args)

/* Command for notify_is_registered */
#define CMD_NOTIFY_ISREGISTERED		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 15u,		\
					struct notify_cmd_args_is_registered)

/* Command for notify_shared_mem_req */
#define CMD_NOTIFY_SHAREDMEMREQ		_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 16u,		\
					struct notify_cmd_args_shared_mem_req)
/* Command for notify_register_event_single */
#define CMD_NOTIFY_REGISTEREVENTSINGLE	_IOWR(NOTIFYCMDBASE,		\
					NOTIFY_BASE_CMD + 17u,		\
					struct notify_cmd_args_register_event)

/* Command for notify_unregister_event_single */
#define CMD_NOTIFY_UNREGISTEREVENTSINGLE	_IOWR(NOTIFYCMDBASE,	\
					NOTIFY_BASE_CMD + 18u,		\
					struct notify_cmd_args_unregister_event)


/*Structure of Event Packet read from notify kernel-side..*/
struct notify_drv_event_packet {
	struct list_head element;
	u32 pid;
	u32 proc_id;
	u32 event_id;
	u16 line_id;
	u32 data;
	notify_fn_notify_cbck func;
	void *param;
	bool is_exit;
};

/* Common arguments for all ioctl commands */
struct notify_cmd_args {
	int api_status;
};

/* Command arguments for notify_get_config */
struct notify_cmd_args_get_config {
	struct notify_cmd_args common_args;
	struct notify_config *cfg;
};

/* Command arguments for notify_setup */
struct notify_cmd_args_setup {
	struct notify_cmd_args common_args;
	struct notify_config *cfg;
};

/* Command arguments for notify_destroy */
struct notify_cmd_args_destroy {
	struct notify_cmd_args common_args;
};

/* Command arguments for notify_attach */
struct notify_cmd_args_attach {
	struct notify_cmd_args common_args;
	u16 proc_id;
	void *shared_addr;
};

/* Command arguments for notify_detach */
struct notify_cmd_args_detach {
	struct notify_cmd_args common_args;
	u16 proc_id;
};

/* Command arguments for notify_cmd_args_shared_mem_req */
struct notify_cmd_args_shared_mem_req {
	struct notify_cmd_args common_args;
	u16 proc_id;
	void *shared_addr;
	uint shared_mem_size;
};

/* Command arguments for notify_cmd_args_is_registered */
struct notify_cmd_args_is_registered {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	bool is_registered;
};

/* Command arguments for notify_register_event */
struct notify_cmd_args_register_event {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 event_id;
	notify_fn_notify_cbck fn_notify_cbck;
	uint *cbck_arg;
	u32 pid;
};

/* Command arguments for notify_unregister_event */
struct notify_cmd_args_unregister_event {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 event_id;
	notify_fn_notify_cbck fn_notify_cbck;
	uint *cbck_arg;
	u32 pid;
};

/* Command arguments for notify_send_event */
struct notify_cmd_args_send_event {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 event_id;
	u32 payload;
	bool wait_clear;
};

/* Command arguments for notify_disable */
struct notify_cmd_args_disable {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 flags;
};

/* Command arguments for notify_restore */
struct notify_cmd_args_restore {
	struct notify_cmd_args common_args;
	u32 key;
	u16 proc_id;
	u16 line_id;
};

/* Command arguments for notify_disable_event */
struct notify_cmd_args_disable_event {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 event_id;
};

/* Command arguments for notify_enable_event */
struct notify_cmd_args_enable_event {
	struct notify_cmd_args common_args;
	u16 proc_id;
	u16 line_id;
	u32 event_id;
};

/* Command arguments for notify_exit */
struct notify_cmd_args_exit {
	struct notify_cmd_args common_args;
};


#endif /* !defined(_NOTIFY_IOCTL_H_) */
