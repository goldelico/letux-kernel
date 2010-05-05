/*
 * errbase.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Central repository for DSP/BIOS Bridge error and status code.
 *
 * Error codes are of the form:
 *     [<MODULE>]_E<ERRORCODE>
 *
 * Success codes are of the form:
 *     [<MODULE>]_S<SUCCESSCODE>
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ERRBASE_
#define ERRBASE_

/* Base of generic errors and component errors */
#define DSP_SBASE               (dsp_status)0x00008000
#define DSP_EBASE               (dsp_status)0x80008000

#define DSP_COMP_EBASE          (dsp_status)0x80040200
#define DSP_COMP_ELAST          (dsp_status)0x80047fff

/* SUCCESS Codes */

/* Generic success code */
#define DSP_SOK                     (DSP_SBASE + 0)

/* GPP is already attached to this DSP processor */
#define DSP_SALREADYATTACHED        (DSP_SBASE + 1)

/* This is the last object available for enumeration. */
#define DSP_SENUMCOMPLETE           (DSP_SBASE + 2)

/* The DSP is already asleep. */
#define DSP_SALREADYASLEEP          (DSP_SBASE + 3)

/* The DSP is already awake. */
#define DSP_SALREADYAWAKE           (DSP_SBASE + 4)

/* TRUE */
#define DSP_STRUE                   (DSP_SBASE + 5)

/* FALSE */
#define DSP_SFALSE                  (DSP_SBASE + 6)

/* A library contains no dependent library references */
#define DSP_SNODEPENDENTLIBS        (DSP_SBASE + 7)

/* A persistent library is already loaded by the dynamic loader */
#define DSP_SALREADYLOADED          (DSP_SBASE + 8)

/* Some error occured, but it is OK to continue */
#define DSP_OKTO_CONTINUE          (DSP_SBASE + 9)

/* FAILURE Codes */

/* During enumeration a change in the number or properties of the objects
 * has occurred. */
#define DSP_ECHANGEDURINGENUM       (DSP_EBASE + 3)

/* I/O is currently pending. */
#define DSP_EPENDING                (DSP_EBASE + 0x11)

/* The state of the specified object is incorrect for the requested
 * operation. */
#define DSP_EWRONGSTATE             (DSP_EBASE + 0x1b)

/* Symbol not found in the COFF file.  DSPNode_Create will return this if
 * the iAlg function table for an xDAIS socket is not found in the COFF file.
 * In this case, force the symbol to be linked into the COFF file.
 * DSPNode_Create, DSPNode_Execute, and DSPNode_Delete will return this if
 * the create, execute, or delete phase function, respectively, could not be
 * found in the COFF file. */
#define DSP_ESYMBOL                 (DSP_EBASE + 0x1c)

/* UUID not found in registry. */
#define DSP_EUUID                   (DSP_EBASE + 0x1d)

/* Unable to read content of DCD data section ; this is typically caused by
 * improperly configured nodes. */
#define DSP_EDCDREADSECT            (DSP_EBASE + 0x1e)

/* Unable to decode DCD data section content ; this is typically caused by
 * changes to DSP/BIOS Bridge data structures. */
#define DSP_EDCDPARSESECT           (DSP_EBASE + 0x1f)

/* Unable to get pointer to DCD data section ; this is typically caused by
 * improperly configured UUIDs. */
#define DSP_EDCDGETSECT             (DSP_EBASE + 0x20)

/* Unable to load file containing DCD data section ; this is typically
 * caused by a missing COFF file. */
#define DSP_EDCDLOADBASE            (DSP_EBASE + 0x21)

/* The specified COFF file does not contain a valid node registration
 * section. */
#define DSP_EDCDNOAUTOREGISTER      (DSP_EBASE + 0x22)

/* A specified entity was not found. */
#define DSP_ENOTFOUND               (DSP_EBASE + 0x2d)

/* A shared memory buffer contained in a message or stream could not be
 * mapped to the GPP client process's virtual space. */
#define DSP_ETRANSLATE              (DSP_EBASE + 0x2f)

/* Unable to find a named section in DSP executable */
#define DSP_ENOSECT                 (DSP_EBASE + 0x32)

/* Unable to open file */
#define DSP_EFOPEN                  (DSP_EBASE + 0x33)

/* Unable to read file */
#define DSP_EFREAD                  (DSP_EBASE + 0x34)

/* A non-existent memory segment identifier was specified */
#define DSP_EOVERLAYMEMORY          (DSP_EBASE + 0x37)

/* Invalid segment ID */
#define DSP_EBADSEGID               (DSP_EBASE + 0x38)

/* Error occurred in a dynamic loader library function */
#define DSP_EDYNLOAD                (DSP_EBASE + 0x3d)

/* Device in 'sleep/suspend' mode due to DPM */
#define DSP_EDPMSUSPEND             (DSP_EBASE + 0x3e)

/* FAILURE Codes : COD */
#define COD_EBASE                   (DSP_COMP_EBASE + 0x400)

/* No symbol table is loaded for this board. */
#define COD_E_NOSYMBOLSLOADED       (COD_EBASE + 0x00)

/* Symbol not found in for this board. */
#define COD_E_SYMBOLNOTFOUND        (COD_EBASE + 0x01)

/* Unable to initialize the ZL COFF parsing module. */
#define COD_E_ZLCREATEFAILED        (COD_EBASE + 0x03)

/* FAILURE Codes : CHNL */
#define CHNL_EBASE                  (DSP_COMP_EBASE + 0x500)

/* Attempt to created channel manager with too many channels. */
#define CHNL_E_MAXCHANNELS          (CHNL_EBASE + 0x00)

/* No free channels are available. */
#define CHNL_E_OUTOFSTREAMS         (CHNL_EBASE + 0x02)

/* Channel ID is out of range. */
#define CHNL_E_BADCHANID            (CHNL_EBASE + 0x03)

/* dwTimeOut parameter was CHNL_IOCNOWAIT, yet no I/O completions were
 * queued. */
#define CHNL_E_NOIOC                (CHNL_EBASE + 0x06)

/* I/O has been cancelled on this channel. */
#define CHNL_E_CANCELLED            (CHNL_EBASE + 0x07)

/* End of stream was already requested on this output channel. */
#define CHNL_E_EOS                  (CHNL_EBASE + 0x09)

/* DSP word size of zero configured for this device. */
#define CHNL_E_INVALIDWORDSIZE      (CHNL_EBASE + 0x0D)

/* A zero length memory base was specified for a shared memory class driver. */
#define CHNL_E_INVALIDMEMBASE       (CHNL_EBASE + 0x0E)

/* Memory map is not configured, or unable to map physical to linear
 * address. */
#define CHNL_E_NOMEMMAP             (CHNL_EBASE + 0x0F)

/* Attempted to create a channel manager  when one already exists. */
#define CHNL_E_MGREXISTS            (CHNL_EBASE + 0x10)

/* Unable to plug channel ISR for configured IRQ. */
#define CHNL_E_ISR                  (CHNL_EBASE + 0x11)

/* No free I/O request packets are available. */
#define CHNL_E_NOIORPS              (CHNL_EBASE + 0x12)

/* Buffer size is larger than the size of physical channel. */
#define CHNL_E_BUFSIZE              (CHNL_EBASE + 0x13)

/* User cannot mark end of stream on an input channel. */
#define CHNL_E_NOEOS                (CHNL_EBASE + 0x14)

/* Wait for flush operation on an output channel timed out. */
#define CHNL_E_WAITTIMEOUT          (CHNL_EBASE + 0x15)

#endif /* ERRBASE_ */
