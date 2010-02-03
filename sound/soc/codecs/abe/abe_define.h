/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_DEFINE_H_
#define _ABE_DEFINE_H_

#define ATC_DESCRIPTOR_NUMBER		64
#define PROCESSING_SLOTS		24
#define TASKS_IN_SLOT			4
#define TASK_POOL_LENGTH		128
#define MCU_IRQ				0x24
#define DSP_IRQ				0x4c
#define DMA_REQ				0x84
#define DCMD_FIFO_NUMBER		2
#define DCMD_FIFO_LENGTH		8
#define DEBUG_FIFO_LENGTH		16
#define IRQ_FIFO_LENGTH			6
#define SIGNAL_L_MF_COPIED		0x0010
#define SIGNAL_L_MMDL_PP		0x0020
#define IO_DESCR_NUMBER			19
#define IO_DESCR_LIST_NELEM		4
#define IO_DESCR_LIST_SIZE		6
#define PING_PONG_NUMBER		8
#define MAX_MM_BUFFERS			2
#define DEBUG_BUF_LENGTH		32
#define PHOENIX_FIFO_LENGTH		8
#define EQUALIZATION_F_ORDER		8
#define DEFAULT_FILTER_ORDER		8
#define GAINS_WITH_RAMP			18
#define GAINS_NO_RAMP			12
#define McPDM_OUT_FIFO_LENGTH		18
#define McPDM_IN_FIFO_LENGTH		127
#define TASK_CLUSTER_NUMBER		4
#define TASK_CLUSTER_LENGTH		8
#define MASK_NB_PHASES			15
#define EANC_IIR_MEMSIZE		17
#define ASRC_UL_VX_FIR_L		19
#define ASRC_DL_VX_FIR_L		19
#define ASRC_DL_MM_FIR_L		18
#define ASRC_N_8k			2
#define ASRC_N_16k			4
#define ASRC_N_48k			12
#define EANC_IIR_MEMSIZE		17
#define VIBRA_N				5
#define VIBRA1_IIR_MEMSIZE		11

#endif	/* _ABE_DEFINE_H_ */
