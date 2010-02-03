/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#include "abe_main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DEFINE
 */
#define NO_OUTPUT		0
#define TERMINAL_OUTPUT		1
#define LINE_OUTPUT		2
#define DEBUG_TRACE_OUTPUT	3

#define DBG_LOG_SIZE		1000

#define DBG_BITFIELD_OFFSET	8

#define DBG_API_CALLS		0
#define DBG_MAPI		(1L << (DBG_API_CALLS + DBG_BITFIELD_OFFSET))

#define DBG_EXT_DATA_ACCESS	1
#define DBG_MDATA		(1L << (DBG_EXT_DATA_ACCESS + DBG_BITFIELD_OFFSET))

#define DBG_ERR_CODES		2
#define DBG_MERR		(1L << (DBG_API_CALLS + DBG_BITFIELD_OFFSET))

/*
 * IDs used for traces
 */
#define ID_RESET_HAL			(1 + DBG_MAPI)
#define ID_LOAD_FW			(2 + DBG_MAPI)
#define ID_DEFAULT_CONFIGURATION	(3 + DBG_MAPI)
#define ID_IRQ_PROCESSING		(4 + DBG_MAPI)
#define ID_EVENT_GENERATOR_SWITCH	(5 + DBG_MAPI)
#define ID_SET_MEMORY_CONFIG		(6 + DBG_MAPI)
#define ID_READ_LOWEST_OPP		(7 + DBG_MAPI)
#define ID_SET_OPP_PROCESSING		(8 + DBG_MAPI)
#define ID_SET_PING_PONG_BUFFER		(9 + DBG_MAPI)
#define ID_PLUG_SUBROUTINE		(10 + DBG_MAPI)
#define ID_UNPLUG_SUBROUTINE		(11 + DBG_MAPI)
#define ID_PLUG_SEQUENCE		(12 + DBG_MAPI)
#define ID_LAUNCH_SEQUENCE		(13 + DBG_MAPI)
#define ID_LAUNCH_SEQUENCE_PARAM	(14 + DBG_MAPI)
#define ID_SET_ANALOG_CONTROL		(15 + DBG_MAPI)
#define ID_READ_ANALOG_GAIN_DL		(16 + DBG_MAPI)
#define ID_READ_ANALOG_GAIN_UL		(17 + DBG_MAPI)
#define ID_ENABLE_DYN_UL_GAIN		(18 + DBG_MAPI)
#define ID_DISABLE_DYN_UL_GAIN		(19 + DBG_MAPI)
#define ID_ENABLE_DYN_EXTENSION		(20 + DBG_MAPI)
#define ID_DISABLE_DYN_EXTENSION	(21 + DBG_MAPI)
#define ID_NOTIFY_ANALOG_GAIN_CHANGED	(22 + DBG_MAPI)
#define ID_RESET_PORT			(23 + DBG_MAPI)
#define ID_READ_REMAINING_DATA		(24 + DBG_MAPI)
#define ID_DISABLE_DATA_TRANSFER	(25 + DBG_MAPI)
#define ID_ENABLE_DATA_TRANSFER		(26 + DBG_MAPI)
#define ID_READ_GLOBAL_COUNTER		(27 + DBG_MAPI)
#define ID_SET_DMIC_FILTER		(28 + DBG_MAPI)
#define ID_WRITE_PORT_DESCRIPTOR	(29 + DBG_MAPI)
#define ID_READ_PORT_DESCRIPTOR		(30 + DBG_MAPI)
#define ID_READ_PORT_ADDRESS		(31 + DBG_MAPI)
#define ID_WRITE_PORT_GAIN		(32 + DBG_MAPI)
#define ID_WRITE_HEADSET_OFFSET		(33 + DBG_MAPI)
#define ID_READ_GAIN_RANGES		(34 + DBG_MAPI)
#define ID_WRITE_EQUALIZER		(35 + DBG_MAPI)
#define ID_WRITE_ASRC			(36 + DBG_MAPI)
#define ID_WRITE_APS			(37 + DBG_MAPI)
#define ID_WRITE_MIXER			(38 + DBG_MAPI)
#define ID_WRITE_EANC			(39 + DBG_MAPI)
#define ID_WRITE_ROUTER			(40 + DBG_MAPI)
#define ID_READ_PORT_GAIN		(41 + DBG_MAPI)
#define ID_READ_ASRC			(42 + DBG_MAPI)
#define ID_READ_APS			(43 + DBG_MAPI)
#define ID_READ_APS_ENERGY		(44 + DBG_MAPI)
#define ID_READ_MIXER			(45 + DBG_MAPI)
#define ID_READ_EANC			(46 + DBG_MAPI)
#define ID_READ_ROUTER			(47 + DBG_MAPI)
#define ID_READ_DEBUG_TRACE		(48 + DBG_MAPI)
#define ID_SET_DEBUG_TRACE		(49 + DBG_MAPI)
#define ID_SET_DEBUG_PINS		(50 + DBG_MAPI)
#define ID_CALL_SUBROUTINE		(51 + DBG_MAPI)

/*
 * IDs used for error codes
 */
#define NOERR				0
#define ABE_SET_MEMORY_CONFIG_ERR	(1 + DBG_MERR)
#define ABE_BLOCK_COPY_ERR		(2 + DBG_MERR)
#define ABE_SEQTOOLONG			(3 + DBG_MERR)
#define ABE_BADSAMPFORMAT		(4 + DBG_MERR)
#define ABE_SET_ATC_MEMORY_CONFIG_ERR	(5 + DBG_MERR)
#define ABE_PROTOCOL_ERROR		(6 + DBG_MERR)
#define ABE_PARAMETER_ERROR		(7 + DBG_MERR)
#define ABE_PORT_REPROGRAMMING		(8 + DBG_MERR)	/* port programmed while still running */
#define ABE_READ_USE_CASE_OPP_ERR	(9 + DBG_MERR)
#define ABE_PARAMETER_OVERFLOW		(10 + DBG_MERR)

/*
 * IDs used for error codes
 */
#define ERR_LIB		(1 << 1)	/* error in the LIB.C file */
#define ERR_API		(1 << 2)	/* error in the API.C file */
#define ERR_INI		(1 << 3)	/* error in the INI.C file */
#define ERR_SEQ		(1 << 4)	/* error in the SEQ.C file */
#define ERR_DBG		(1 << 5)	/* error in the DBG.C file */
#define ERR_EXT		(1 << 6)	/* error in the EXT.C file */

/*
 * MACROS
 */
#ifdef HAL_TIME_STAMP
#define _log(x) ((x&abe_dbg_mask)?abe_dbg_log(x);   \
		 abe_dbg_time_stamp(x);	     \
		 abe_dbg_printf(x);		 \
		)
#else
#define _log(x) {if(x&abe_dbg_mask)abe_dbg_log(x);}
#endif

/*
 * PROTOTYPES
 */

/*
 *  ABE_DBG_LOG
 *
 *  Parameter  :
 *      x : data to be logged
 *
 *      abe_dbg_activity_log : global circular buffer holding the data
 *      abe_dbg_activity_log_write_pointer : circular write pointer
 *
 *  Operations :
 *      saves data in the log file
 *
 *  Return value :
 *      none
 */
void abe_dbg_log(abe_uint32 x);

/*
 *  ABE_DBG_ERROR_LOG
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *      log the error codes
 *
 *  Return value :
 *
 */
void abe_dbg_error_log(abe_uint32 x);

#ifdef __cplusplus
}
#endif
