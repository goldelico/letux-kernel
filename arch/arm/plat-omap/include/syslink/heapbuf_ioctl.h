/*
 *  heapbuf_ioctl.h
 *
 *  Heap module manages fixed size buffers that can be used
 *  in a multiprocessor system with shared memory.
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

#ifndef _HEAPBUF_IOCTL_
#define _HEAPBUF_IOCTL_

#include <linux/ioctl.h>
#include <linux/types.h>

#include <ipc_ioctl.h>
#include <heap.h>
#include <heapbuf.h>


enum CMD_HEAPBUF {
	HEAPBUF_GETCONFIG = HEAPBUF_BASE_CMD,
	HEAPBUF_SETUP,
	HEAPBUF_DESTROY,
	HEAPBUF_PARAMS_INIT,
	HEAPBUF_CREATE,
	HEAPBUF_DELETE,
	HEAPBUF_OPEN,
	HEAPBUF_CLOSE,
	HEAPBUF_ALLOC,
	HEAPBUF_FREE,
	HEAPBUF_SHAREDMEMREQ,
	HEAPBUF_GETSTATS,
	HEAPBUF_GETEXTENDEDSTATS
};

/*
 *  Command for heapbuf_get_config
 */
#define CMD_HEAPBUF_GETCONFIG		_IOWR(IPC_IOC_MAGIC, HEAPBUF_GETCONFIG,\
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_setup
 */
#define CMD_HEAPBUF_SETUP		_IOWR(IPC_IOC_MAGIC, HEAPBUF_SETUP, \
					struct heapbuf_cmd_args)
/*
 *  Command for heapbuf_destroy
 */
#define CMD_HEAPBUF_DESTROY 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_DESTROY, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_prams_init
 */
#define CMD_HEAPBUF_PARAMS_INIT 	_IOWR(IPC_IOC_MAGIC, \
					HEAPBUF_PARAMS_INIT, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_create
 */
#define CMD_HEAPBUF_CREATE 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_CREATE, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_delete
 */
#define CMD_HEAPBUF_DELETE 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_DELETE, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_open
 */
#define CMD_HEAPBUF_OPEN 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_OPEN, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_close
 */
#define CMD_HEAPBUF_CLOSE 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_CLOSE, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_alloc
 */
#define CMD_HEAPBUF_ALLOC 		_IOWR(IPC_IOC_MAGIC, HEAPBUF_ALLOC, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_free
 */
#define CMD_HEAPBUF_FREE		_IOWR(IPC_IOC_MAGIC, HEAPBUF_FREE, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_shared_memreq
 */
#define CMD_HEAPBUF_SHAREDMEMREQ	_IOWR(IPC_IOC_MAGIC, \
					HEAPBUF_SHAREDMEMREQ, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_get_stats
 */
#define CMD_HEAPBUF_GETSTATS		_IOWR(IPC_IOC_MAGIC, \
					HEAPBUF_GETSTATS, \
					struct heapbuf_cmd_args)

/*
 *  Command for heapbuf_get_extended_stats
 */
#define CMD_HEAPBUF_GETEXTENDEDSTATS	_IOWR(IPC_IOC_MAGIC, \
					HEAPBUF_GETEXTENDEDSTATS, \
					struct heapbuf_cmd_args)


/*
 *  Command arguments for heapbuf
 */
union heapbuf_arg {
	struct {
		void *handle;
		struct heapbuf_params *params;
	} params_init;

	struct  {
		struct heapbuf_config *config;
	} get_config;

	struct {
		struct heapbuf_config *config;
	} setup;

	struct {
		void *handle;
		struct heapbuf_params *params;
		u32 name_len;
		u32 *shared_addr_srptr;
		u32 *shared_buf_srptr;
		void *knl_gate;
	} create;

	struct {
		void *handle;
	} delete;

	struct {
		void *handle;
		struct heapbuf_params *params;
		u32 name_len;
		u32 *shared_addr_srptr;
		void *knl_gate;
	} open;

	struct {
		void *handle;
	} close;

	struct {
		void *handle;
		u32 size;
		u32 align;
		u32 *block_srptr;
	} alloc;

	struct {
		void *handle;
		u32 *block_srptr;
		u32 size;
	} free;

	struct {
		void *handle;
		struct memory_stats *stats;
	} get_stats;

	struct {
		void *handle;
		struct heapbuf_extended_stats *stats;
	} get_extended_stats;

	struct {
		void *handle;
		struct heapbuf_params *params;
		u32 buf_size;
		u32 bytes;
	} shared_memreq;
};

/*
 *  Command arguments for heapbuf
 */
struct heapbuf_cmd_args{
	union heapbuf_arg args;
	s32 api_status;
};

/*
 *  This ioctl interface for heapbuf module
 */
int heapbuf_ioctl(struct inode *pinode, struct file *filp,
			unsigned int cmd, unsigned long  args);

#endif /* _HEAPBUF_IOCTL_ */
