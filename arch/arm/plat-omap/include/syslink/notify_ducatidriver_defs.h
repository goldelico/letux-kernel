/*
 * notify_ducati_driverdefs.h
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


#ifndef NOTIFY_DUCATIDRV_DEFS
#define NOTIFY_DUCATIDRV_DEFS

#include <syslink/notify_ducatidriver.h>

/*
 *  brief  Base structure for NotifyDriverShm command args. This needs to be
 *          the first field in all command args structures.
 */
struct notify_ducatidrv_cmdargs {
	int api_status;
};


/*
 *  ioctl command IDs for NotifyDriverShm
 *
 */
/*
 *  brief  Base command ID for NotifyDriverShm
 */
#define NOTIFYDRIVERSHM_BASE_CMD  0x100

/*
 *  brief  Command for NotifyDriverShm_getConfig
 */
#define CMD_NOTIFYDRIVERSHM_GETCONFIG           (NOTIFYDRIVERSHM_BASE_CMD + 1u)

/*
 *  brief  Command for NotifyDriverShm_setup
 */
#define CMD_NOTIFYDRIVERSHM_SETUP               (NOTIFYDRIVERSHM_BASE_CMD + 2u)

/*
 *  brief  Command for NotifyDriverShm_setup
 */
#define CMD_NOTIFYDRIVERSHM_DESTROY             (NOTIFYDRIVERSHM_BASE_CMD + 3u)

/*
 *  brief  Command for NotifyDriverShm_destroy
 */
#define CMD_NOTIFYDRIVERSHM_PARAMS_INIT         (NOTIFYDRIVERSHM_BASE_CMD + 4u)

/*
 *  brief  Command for NotifyDriverShm_create
 */
#define CMD_NOTIFYDRIVERSHM_CREATE              (NOTIFYDRIVERSHM_BASE_CMD + 5u)

/*
 *  brief  Command for NotifyDriverShm_delete
 */
#define CMD_NOTIFYDRIVERSHM_DELETE              (NOTIFYDRIVERSHM_BASE_CMD + 6u)

/*
 *  brief  Command for NotifyDriverShm_open
 */
#define CMD_NOTIFYDRIVERSHM_OPEN                (NOTIFYDRIVERSHM_BASE_CMD + 7u)

/*
 *  brief  Command for NotifyDriverShm_close
 */
#define CMD_NOTIFYDRIVERSHM_CLOSE               (NOTIFYDRIVERSHM_BASE_CMD + 8u)


/*
 *  @brief  Command arguments for NotifyDriverShm_getConfig
 */
struct notify_ducatidrv_cmdargs_getconfig {
	struct notify_ducatidrv_cmdargs common_args;
	struct notify_ducatidrv_config *cfg;
};

/*
 *  brief  Command arguments for NotifyDriverShm_setup
 */
struct notify_ducatidrv_cmdargs_setup {
	struct notify_ducatidrv_cmdargs common_args;
	struct notify_ducatidrv_config *cfg;
};

/*
 *  brief  Command arguments for NotifyDriverShm_destroy
 */
struct notify_ducatidrv_cmdargs_destroy  {
	struct notify_ducatidrv_cmdargs common_args;
} ;

/*
 *  brief  Command arguments for NotifyDriverShm_Params_init
 */

struct notify_ducatidrv_cmdargs_paramsinit {
	struct notify_ducatidrv_cmdargs common_args;
	struct notify_driver_object *handle;
	struct notify_ducatidrv_params *params;
};

/*!
 *  @brief  Command arguments for NotifyDriverShm_create
 */
struct notify_ducatidrv_cmdargs_create {
	struct notify_ducatidrv_cmdargs  common_args;
	char  driverName[NOTIFY_MAX_NAMELEN];
	struct notify_ducatidrv_params params;
	struct notify_driver_object *handle;
};

/*
 *  brief  Command arguments for NotifyDriverShm_delete
 */
struct notify_ducatidrv_cmdargs_delete {
	struct notify_ducatidrv_cmdargs common_args;
	struct notify_driver_object *handle;
};

/*
 *  brief  Command arguments for NotifyDriverShm_open
 */
struct notify_ducatidrv_cmdargs_open {
	struct notify_ducatidrv_cmdargs common_args;
	char  driverName[NOTIFY_MAX_NAMELEN];
	struct notify_driver_object *handle;

};

/*
 *  brief  Command arguments for NotifyDriverShm_close
 */
struct notify_ducatidrv_cmdargs_close {
	struct notify_ducatidrv_cmdargs common_args;
	struct notify_driver_object *handle;

};

#endif /*NOTIFY_DUCATIDRV_DEFS*/
