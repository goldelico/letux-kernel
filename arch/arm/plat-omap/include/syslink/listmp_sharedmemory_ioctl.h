/*
 *  listmp_sharedmemory_ioctl.h
 *
 *  Definitions of listmp_sharedmemory driver types and structures.
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

#ifndef _LISTMP_SHAREDMEMORY_IOCTL_H_
#define _LISTMP_SHAREDMEMORY_IOCTL_H_

/* Standard headers */
#include <linux/types.h>

/* Syslink headers */
#include <ipc_ioctl.h>
#include <listmp_sharedmemory.h>
#include <sharedregion.h>

/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/* Base command ID for listmp_sharedmemory */
#define LISTMP_SHAREDMEMORY_IOC_MAGIC		IPC_IOC_MAGIC
enum listmp_sharedmemory_drv_cmd {
	LISTMP_SHAREDMEMORY_GETCONFIG = LISTMP_SHAREDMEMORY_BASE_CMD,
	LISTMP_SHAREDMEMORY_SETUP,
	LISTMP_SHAREDMEMORY_DESTROY,
	LISTMP_SHAREDMEMORY_PARAMS_INIT,
	LISTMP_SHAREDMEMORY_CREATE,
	LISTMP_SHAREDMEMORY_DELETE,
	LISTMP_SHAREDMEMORY_OPEN,
	LISTMP_SHAREDMEMORY_CLOSE,
	LISTMP_SHAREDMEMORY_ISEMPTY,
	LISTMP_SHAREDMEMORY_GETHEAD,
	LISTMP_SHAREDMEMORY_GETTAIL,
	LISTMP_SHAREDMEMORY_PUTHEAD,
	LISTMP_SHAREDMEMORY_PUTTAIL,
	LISTMP_SHAREDMEMORY_INSERT,
	LISTMP_SHAREDMEMORY_REMOVE,
	LISTMP_SHAREDMEMORY_NEXT,
	LISTMP_SHAREDMEMORY_PREV,
	LISTMP_SHAREDMEMORY_SHAREDMEMREQ
};

/*  ----------------------------------------------------------------------------
 *  IOCTL command IDs for listmp_sharedmemory
 *  ----------------------------------------------------------------------------
 */
/* Command for listmp_sharedmemory_get_config */
#define CMD_LISTMP_SHAREDMEMORY_GETCONFIG \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_GETCONFIG, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_setup */
#define CMD_LISTMP_SHAREDMEMORY_SETUP \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_SETUP, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_destroy */
#define CMD_LISTMP_SHAREDMEMORY_DESTROY \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_DESTROY, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_params_init */
#define CMD_LISTMP_SHAREDMEMORY_PARAMS_INIT \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_PARAMS_INIT, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_create */
#define CMD_LISTMP_SHAREDMEMORY_CREATE \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_CREATE, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_delete */
#define CMD_LISTMP_SHAREDMEMORY_DELETE \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_DELETE, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_open */
#define CMD_LISTMP_SHAREDMEMORY_OPEN \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_OPEN, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_close */
#define CMD_LISTMP_SHAREDMEMORY_CLOSE \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_CLOSE, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_is_empty */
#define CMD_LISTMP_SHAREDMEMORY_ISEMPTY \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_ISEMPTY, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_get_head */
#define CMD_LISTMP_SHAREDMEMORY_GETHEAD \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_GETHEAD, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_get_tail */
#define CMD_LISTMP_SHAREDMEMORY_GETTAIL \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_GETTAIL, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_put_head */
#define CMD_LISTMP_SHAREDMEMORY_PUTHEAD \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_PUTHEAD, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_put_tail */
#define CMD_LISTMP_SHAREDMEMORY_PUTTAIL \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_PUTTAIL, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_insert */
#define CMD_LISTMP_SHAREDMEMORY_INSERT \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_INSERT, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_remove */
#define CMD_LISTMP_SHAREDMEMORY_REMOVE \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_REMOVE, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_next */
#define CMD_LISTMP_SHAREDMEMORY_NEXT \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_NEXT, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_prev */
#define CMD_LISTMP_SHAREDMEMORY_PREV \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_PREV, \
	struct listmp_sharedmemory_cmd_args)

/* Command for listmp_sharedmemory_shared_memreq */
#define CMD_LISTMP_SHAREDMEMORY_SHAREDMEMREQ \
	_IOWR(LISTMP_SHAREDMEMORY_IOC_MAGIC, LISTMP_SHAREDMEMORY_SHAREDMEMREQ, \
	struct listmp_sharedmemory_cmd_args)

/* Command arguments for listmp_sharedmemory */
struct listmp_sharedmemory_cmd_args {
	union {
		struct {
			void *listmp_handle;
			struct listmp_params *params;
		} params_init;

		struct {
			struct listmp_config *config;
		} get_config;

		struct {
			struct listmp_config *config;
		} setup;

		struct {
			void *listmp_handle;
			struct listmp_params *params;
			u32 name_len;
			u32 shared_addr_srptr;
			void *knl_gate;
		} create;

		struct {
			void *listmp_handle;
		} delete_listmp;

		struct {
			void *listmp_handle;
			struct listmp_params *params;
			u32 name_len;
			u32 shared_addr_srptr;
			void *knl_gate;
		} open;

		struct {
			void *listmp_handle;
		} close;

		struct {
			void *listmp_handle;
			bool is_empty;
		} is_empty;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
		} get_head;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
		} get_tail;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
		} put_head;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
		} put_tail;

		struct {
			void *listmp_handle;
			u32 *new_elem_srptr;
			u32 *cur_elem_srptr;
		} insert;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
		} remove;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
			u32 *next_elem_srptr ;
		} next;

		struct {
			void *listmp_handle;
			u32 *elem_srptr ;
			u32 *prev_elem_srptr ;
		} prev;

		struct {
			void *listmp_handle;
			struct listmp_params *params;
			u32 bytes;
		} shared_memreq;
	} args;

	int api_status;
};

/*  ----------------------------------------------------------------------------
 *  IOCTL functions for listmp_sharedmemory module
 *  ----------------------------------------------------------------------------
 */
/*
 *  ioctl interface function for listmp_sharedmemory
 */
int listmp_sharedmemory_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args);

#endif /* _LISTMP_SHAREDMEMORY_IOCTL_H_ */
