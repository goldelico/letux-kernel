/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_EXT_H_
#define _ABE_EXT_H_

#define PC_SIMULATION			0		/* Tuning is done on PC ? */

/*
 * OS DEPENDENT MMU CONFIGURATION
 */
#define ABE_ATC_BASE_ADDRESS_L3		0x490F1000L
	/* base address used for L3/DMA access */
#define ABE_DMEM_BASE_ADDRESS_L3	0x49080000L
	/* 64kB  as seen from DMA access */
#define ABE_PMEM_BASE_ADDRESS_MPU	0x401E0000L
	/*  8kB  as seen from MPU access */
#define ABE_CMEM_BASE_ADDRESS_MPU	0x401A0000L
	/* 8kB */
#define ABE_SMEM_BASE_ADDRESS_MPU	0x401C0000L
	/* 24kB */
#define ABE_DMEM_BASE_ADDRESS_MPU	0x40180000L
	/* 64kB */
#define ABE_ATC_BASE_ADDRESS_MPU	0x401F1000L

/*
 * HARDWARE AND PERIPHERAL DEFINITIONS
 */

#define ABE_PMEM_SIZE			8192
#define ABE_CMEM_SIZE			8192
#define ABE_SMEM_SIZE			24576
#define ABE_DMEM_SIZE			65536L
#define ABE_ATC_DESC_SIZE		512

#define ABE_MCU_IRQSTATUS_RAW		0x24
#define ABE_MCU_IRQSTATUS		0x28
#define ABE_DSP_IRQSTATUS_RAW		0x4C
#define ABE_DMASTATUS_RAW		0x84

#define EVENT_GENERATOR_COUNTER		0x68
#define EVENT_GENERATOR_COUNTER_DEFAULT 2050

#define EVENT_GENERATOR_START		0x6C
#define EVENT_GENERATOR_ON		1
#define EVENT_GENERATOR_OFF		0

#define EVENT_SOURCE_SELECTION		0x70
#define EVENT_SOURCE_DMA		0
#define EVENT_SOURCE_COUNTER		1

#define AUDIO_ENGINE_SCHEDULER		0x74
#define ABE_ATC_DMIC_DMA_REQ		1
#define ABE_ATC_MCPDMDL_DMA_REQ		2
#define ABE_ATC_MCPDMUL_DMA_REQ		3
#define ABE_ATC_DIRECTION_IN		0
#define ABE_ATC_DIRECTION_OUT		1

/*
 * DMA requests
 */
#define External_DMA_0		0
#define DMIC_DMA_REQ		1
#define McPDM_DMA_DL		2
#define McPDM_DMA_UP		3
#define MCBSP1_DMA_TX		4
#define MCBSP1_DMA_RX		5
#define MCBSP2_DMA_TX		6
#define MCBSP2_DMA_RX		7
#define MCBSP3_DMA_TX		8
#define MCBSP3_DMA_RX		9
#define SLIMBUS1_DMA_TX0	10
#define SLIMBUS1_DMA_TX1	11
#define SLIMBUS1_DMA_TX2	12
#define SLIMBUS1_DMA_TX3	13
#define SLIMBUS1_DMA_TX4	14
#define SLIMBUS1_DMA_TX5	15
#define SLIMBUS1_DMA_TX6	16
#define SLIMBUS1_DMA_TX7	17
#define SLIMBUS1_DMA_RX0	18
#define SLIMBUS1_DMA_RX1	19
#define SLIMBUS1_DMA_RX2	20
#define SLIMBUS1_DMA_RX3	21
#define SLIMBUS1_DMA_RX4	22
#define SLIMBUS1_DMA_RX5	23
#define SLIMBUS1_DMA_RX6	24
#define SLIMBUS1_DMA_RX7	25
#define McASP1_AXEVT		26
#define McASP1_AREVT		29
#define CBPr_DMA_RTX0		32
#define CBPr_DMA_RTX1		33
#define CBPr_DMA_RTX2		34
#define CBPr_DMA_RTX3		35
#define CBPr_DMA_RTX4		36
#define CBPr_DMA_RTX5		37
#define CBPr_DMA_RTX6		38
#define CBPr_DMA_RTX7		39

/*
 * ATC DESCRIPTORS - DESTINATIONS
 */
#define DEST_DMEM access	0x00
#define DEST_MCBSP1_ TX		0x01
#define DEST_MCBSP2_ TX		0x02
#define DEST_MCBSP3_TX		0x03
#define DEST_SLIMBUS1_TX0	0x04
#define DEST_SLIMBUS1_TX1	0x05
#define DEST_SLIMBUS1_TX2	0x06
#define DEST_SLIMBUS1_TX3	0x07
#define DEST_SLIMBUS1_TX4	0x08
#define DEST_SLIMBUS1_TX5	0x09
#define DEST_SLIMBUS1_TX6	0x0A
#define DEST_SLIMBUS1_TX7	0x0B
#define DEST_MCPDM_DL		0x0C
#define DEST_MCASP_TX0		0x0D
#define DEST_MCASP_TX1		0x0E
#define DEST_MCASP_TX2		0x0F
#define DEST_MCASP_TX3		0x10
#define DEST_EXTPORT0		0x11
#define DEST_EXTPORT1		0x12
#define DEST_EXTPORT2		0x13
#define DEST_EXTPORT3		0x14
#define DEST_MCPDM_ON		0x15
#define DEST_CBP_CBPr		0x3F

/*
 * ATC DESCRIPTORS - SOURCES
 */
#define SRC_DMEM access		0x0
#define SRC_MCBSP1_ RX		0x01
#define SRC_MCBSP2_RX		0x02
#define SRC_MCBSP3_RX		0x03
#define SRC_SLIMBUS1_RX0	0x04
#define SRC_SLIMBUS1_RX1	0x05
#define SRC_SLIMBUS1_RX2	0x06
#define SRC_SLIMBUS1_RX3	0x07
#define SRC_SLIMBUS1_RX4	0x08
#define SRC_SLIMBUS1_RX5	0x09
#define SRC_SLIMBUS1_RX6	0x0A
#define SRC_SLIMBUS1_RX7	0x0B
#define SRC_DMIC_UP		0x0C
#define SRC_MCPDM_UP		0x0D
#define SRC_MCASP_RX0		0x0E
#define SRC_MCASP_RX1		0x0F
#define SRC_MCASP_RX2		0x10
#define SRC_MCASP_RX3		0x11
#define SRC_CBP_CBPr		0x3F
#endif	/* _ABE_EXT_H_ */
