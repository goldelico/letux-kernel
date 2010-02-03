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
#include "abe_dat.h" /* data declaration */

/*
 *  initialize the default values for call-backs to subroutines
 *      - FIFO IRQ call-backs for sequenced tasks
 *      - FIFO IRQ call-backs for audio player/recorders (ping-pong protocols)
 *      - Remote debugger interface
 *      - Error monitoring
 *      - Activity Tracing
 */

/*
 *  ABE_HW_CONFIGURATION
 *
 *  Parameter  :
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_hw_configuration()
{
	abe_uint32 atc_reg;
	abe_port_protocol_t *protocol;
	abe_data_format_t format;

	/* initializes the ABE ATC descriptors in DMEM - MCPDM_UL */
	protocol = &(abe_port[PDM_UL_PORT].protocol);   format = abe_port[PDM_UL_PORT].format;
	abe_init_atc(PDM_UL_PORT);
	abe_init_io_tasks(PDM_UL_PORT, &format, protocol);

	/* initializes the ABE ATC descriptors in DMEM - MCPDM_DL */
	protocol = &(abe_port[PDM_DL1_PORT].protocol);	format = abe_port[PDM_DL1_PORT].format;
	abe_init_atc(PDM_DL1_PORT);
	abe_init_io_tasks(PDM_DL1_PORT, &format, protocol);

	/* one DMIC port enabled = all DMICs enabled, since there is a single DMIC path for all DMICs */
	protocol = &(abe_port[DMIC_PORT1].protocol);	format = abe_port[DMIC_PORT1].format;
	abe_init_atc(DMIC_PORT1);
	abe_init_io_tasks(DMIC_PORT1, &format, protocol);

	/* enables the DMAreq from AESS  AESS_DMAENABLE_SET = 255 */
	atc_reg = 0xFF;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, 0x60, &atc_reg, 4);

#if 0
	/* let the EVENT be configured once the ports are already programmed */
	/* enables EVENT_GENERATOR_START=6C from McPDM */
	atc_reg = 0x01;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, 0x6C, &atc_reg, 4);

	// set McPDM_DL as EVENT_SOURCE_SELECTION
	event = 0L;						// source = DMAreq
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, 0x70, &event, 4);

	event = 2L;						// source = MCPDM_DL to AUDIO_ENGINE_SCHEDULER
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, 0x74, &event, 4);
#endif
}

/*
 *  ABE_BUILD_SCHEDULER_TABLE
 *
 *  Parameter  :
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_build_scheduler_table()
{
	short VirtAudio_aMultiFrame[PROCESSING_SLOTS][TASKS_IN_SLOT];
	abe_uint16 i, n;
	abe_uint8 *ptr;
	char  aUplinkMuxing[16];
	abe_uint32 dFastLoopback;

	for (ptr = (abe_uint8 *)&(VirtAudio_aMultiFrame[0][0]), i=0; i < sizeof(VirtAudio_aMultiFrame); i++)
		*ptr++ = 0;

	VirtAudio_aMultiFrame[0][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_AMIC;
	VirtAudio_aMultiFrame[0][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_VX_DL;
	//VirtAudio_aMultiFrame[0][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_DL_8;

	VirtAudio_aMultiFrame[1][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_8_48_BP;
	VirtAudio_aMultiFrame[1][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_8_48_0SR;
	VirtAudio_aMultiFrame[1][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_8_48_LP;
	VirtAudio_aMultiFrame[1][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_DL;

	// VirtAudio_aMultiFrame[2][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_MM_DL;
	VirtAudio_aMultiFrame[2][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_MM_UL_ROUTING;
	VirtAudio_aMultiFrame[2][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_MM_UL2_ROUTING;
	//VirtAudio_aMultiFrame[2][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_UL;
#if 0
	VirtAudio_aMultiFrame[3][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_TONES_DL;
#endif
	VirtAudio_aMultiFrame[3][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VXRECMixer;
	VirtAudio_aMultiFrame[3][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VXREC_SPLIT;
	VirtAudio_aMultiFrame[3][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_SideTone;

	VirtAudio_aMultiFrame[4][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1Mixer;
	VirtAudio_aMultiFrame[4][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2Mixer;
	VirtAudio_aMultiFrame[4][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_SDTMixer;
	VirtAudio_aMultiFrame[4][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_UL2;

	VirtAudio_aMultiFrame[5][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EchoMixer;
	VirtAudio_aMultiFrame[5][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_EQ;
	VirtAudio_aMultiFrame[5][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_EQ;
	VirtAudio_aMultiFrame[5][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_GAIN;

	VirtAudio_aMultiFrame[6][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_EQ;
	VirtAudio_aMultiFrame[6][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_APS_EQ;
	VirtAudio_aMultiFrame[6][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_GAIN;

	VirtAudio_aMultiFrame[8][0]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EchoMixer;

	VirtAudio_aMultiFrame[9][1]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_0SR;
	VirtAudio_aMultiFrame[9][2]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_LP;
	VirtAudio_aMultiFrame[9][3]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_LP;

	VirtAudio_aMultiFrame[10][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_0SR;
	VirtAudio_aMultiFrame[10][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_LP;
	VirtAudio_aMultiFrame[10][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_LP;

	VirtAudio_aMultiFrame[11][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_McPDM_DL;

	VirtAudio_aMultiFrame[12][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_AMIC;
	VirtAudio_aMultiFrame[12][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_96_48_LP;
	VirtAudio_aMultiFrame[12][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_96_48_DEC;
	VirtAudio_aMultiFrame[12][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_EQ;

	VirtAudio_aMultiFrame[13][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_96_48_LP;
	VirtAudio_aMultiFrame[13][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_96_48_DEC;
	VirtAudio_aMultiFrame[13][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_EQ;
	VirtAudio_aMultiFrame[13][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_SPLIT;

	VirtAudio_aMultiFrame[14][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_96_48_LP;
	VirtAudio_aMultiFrame[14][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_96_48_DEC;
	VirtAudio_aMultiFrame[14][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_EQ;
	VirtAudio_aMultiFrame[14][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_SPLIT;

	VirtAudio_aMultiFrame[15][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_96_48_LP;
	VirtAudio_aMultiFrame[15][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_96_48_DEC;
	VirtAudio_aMultiFrame[15][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_EQ;
	VirtAudio_aMultiFrame[15][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_SPLIT;

	VirtAudio_aMultiFrame[16][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_SPLIT;
	VirtAudio_aMultiFrame[16][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_IIR;
	VirtAudio_aMultiFrame[16][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_CORE;

	VirtAudio_aMultiFrame[17][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_APS_IIR;
	VirtAudio_aMultiFrame[17][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_APS_SPLIT;
	VirtAudio_aMultiFrame[17][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_L_APS_CORE;
	VirtAudio_aMultiFrame[17][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_R_APS_CORE;

	VirtAudio_aMultiFrame[21][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_ROUTING;
	VirtAudio_aMultiFrame[21][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ULMixer;

	VirtAudio_aMultiFrame[22][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_8_LP;
	VirtAudio_aMultiFrame[22][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_8_DEC1;
	VirtAudio_aMultiFrame[22][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_8_BP;

	VirtAudio_aMultiFrame[23][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_McPDM_DL;
	//VirtAudio_aMultiFrame[23][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_UL_8;
	VirtAudio_aMultiFrame[23][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_VX_UL;
	VirtAudio_aMultiFrame[23][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_GAIN_UPDATE;

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof(VirtAudio_aMultiFrame));

	/* DMIC Fast Loopback */
	dFastLoopback = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_DMIC;
	dFastLoopback = dFastLoopback << 16;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, 0x116C,
		(abe_uint32*)&dFastLoopback, sizeof(dFastLoopback));

	/* reset the uplink router */
	n = D_aUplinkRouting_ADDR_END - D_aUplinkRouting_ADDR + 1;
	for(i = 0; i < n; i++)
		aUplinkMuxing[i] = ZERO_labelID;

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_aUplinkRouting_ADDR, (abe_uint32 *)aUplinkMuxing, sizeof(aUplinkMuxing));
}

/*
 *  ABE_INIT_ATC
 *
 *  Parameter  :
 *      prot : protocol being used
 *
 *  Operations :
 *      load the DMEM ATC/AESS descriptors
 *
 *  Return value :
 *
 */
void abe_init_atc(abe_port_id id)
{
	abe_satcdescriptor_aess desc;
	abe_uint8 thr, thr1, thr2, iter, data_shift;
	abe_int32 iterfactor;

	// load default values of the descriptor
	desc.rdpt = desc.wrpt = desc.irqdest = desc.cberr = desc.desen =0;
	desc.reserved0 = desc.reserved1 = desc.reserved2 = 0;
	desc.srcid = desc.destid = desc.badd = desc.iter = desc.cbsize = 0;


	switch ((abe_port[id]).protocol.protocol_switch) {
	case SLIMBUS_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_slimbus.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_slimbus.buf_addr1) >> 4;
		desc.iter = (abe_port[id]).protocol.p.prot_slimbus.iter;
		desc.srcid = abe_atc_srcid [(abe_port[id]).protocol.p.prot_slimbus.desc_addr1 >> 3];
		desc.nw = 1;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, (abe_port[id]).protocol.p.prot_slimbus.desc_addr1, (abe_uint32*)&desc, sizeof(desc));

		desc.badd = (abe_port[id]).protocol.p.prot_slimbus.buf_addr2;
		desc.srcid = abe_atc_srcid [(abe_port[id]).protocol.p.prot_slimbus.desc_addr2 >> 3];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, (abe_port[id]).protocol.p.prot_slimbus.desc_addr2, (abe_uint32*)&desc, sizeof(desc));
		break;
	case SERIAL_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_serial.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_serial.buf_addr) >> 4;
		desc.iter = (abe_port[id]).protocol.p.prot_serial.iter;
		desc.srcid = abe_atc_srcid [(abe_port[id]).protocol.p.prot_serial.desc_addr >> 3];
		desc.nw = 1;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, (abe_port[id]).protocol.p.prot_serial.desc_addr, (abe_uint32*)&desc, sizeof(desc));
		break;
	case DMIC_PORT_PROT:
		desc.cbdir = ABE_ATC_DIRECTION_IN;
		desc.cbsize = (abe_port[id]).protocol.p.prot_dmic.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_dmic.buf_addr) >> 4;
		desc.iter = DMIC_ITER;
		desc.srcid = abe_atc_srcid [ABE_ATC_DMIC_DMA_REQ];
		desc.nw = 1;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, ABE_ATC_DMIC_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case MCPDMDL_PORT_PROT:
		abe_global_mcpdm_control = abe_port[id].protocol.p.prot_mcpdmdl.control;	/* Control allowed on McPDM DL */
		desc.cbdir = ABE_ATC_DIRECTION_OUT;
		desc.cbsize = (abe_port[id]).protocol.p.prot_mcpdmdl.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_mcpdmdl.buf_addr) >> 4;
		desc.iter = MCPDM_DL_ITER;
		desc.destid = abe_atc_dstid [ABE_ATC_MCPDMDL_DMA_REQ];
		desc.nw = 0;
		desc.wrpt = desc.iter;		/* @@@ pre-load the ATC buffer (?) */
		desc.desen = 1;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, ABE_ATC_MCPDMDL_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case MCPDMUL_PORT_PROT:
		desc.cbdir = ABE_ATC_DIRECTION_IN;
		desc.cbsize = (abe_port[id]).protocol.p.prot_mcpdmul.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_mcpdmul.buf_addr) >> 4;
		desc.iter = MCPDM_UL_ITER;
		desc.srcid = abe_atc_srcid [ABE_ATC_MCPDMUL_DMA_REQ];
		desc.wrpt = MCPDM_UL_ITER;				  /* @@@ pre-load the ATC buffer (Virtio bug) */
		desc.nw = 1;
		desc.desen = 1;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, ABE_ATC_MCPDMUL_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case PINGPONG_PORT_PROT:
		/* software protocol, nothing to do on ATC */
		break;
	case DMAREQ_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_dmareq.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_dmareq.buf_addr) >> 4;
		desc.iter = 1;	/* CBPr needs ITER=1. this is the eDMA job to do the iterations */
		desc.nw = 0;	/* data is sent on DMAreq, and never before data is prepared */

		thr = (abe_int8) abe_port[id].protocol.p.prot_dmareq.thr_flow;
		iterfactor = abe_dma_port_iter_factor (&((abe_port[id]).format));
		data_shift = (abe_uint8)((iterfactor > 1)? 1:0);    /* shift = 0 for mono, shift = 1 for stereo - see update of ATC pointers in IOtasks */
		thr = (abe_uint8)(thr * iterfactor);	      /* scales the data threshold to the number of words in DMEM per sample */
		iter = (abe_uint8) abe_dma_port_iteration(&((abe_port[id]).format));

		/* check is input from ABE point of view */
		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN) {
			/* Firmware compares X+Drift to Thresholds */
			thr2 = (abe_uint8)(iter - thr);
			thr1 = (abe_uint8)(iter + thr);
			desc.wrpt = ((thr1 + thr2) >> 1) + 2;
		} else {
			/* thresholds for write access to DMEM */
			thr2 = 0;
			thr1 = (abe_uint8)(thr * 2);
			desc.wrpt = ((thr1 + thr2) >> 1);
		}

		if (desc.cbdir == ABE_ATC_DIRECTION_OUT)
			desc.destid = abe_atc_dstid[(abe_port[id]).protocol.p.prot_dmareq.desc_addr >> 3];
		else
			desc.srcid = abe_atc_srcid[(abe_port[id]).protocol.p.prot_dmareq.desc_addr >> 3];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, (abe_port[id]).protocol.p.prot_dmareq.desc_addr, (abe_uint32*)&desc, sizeof(desc));
		break;
	default:
		break;
	}
}

/*
 *
 *ABE_INIT_DMA_T
 *  Parameter  :
 *      prot : protocol being used
 *
 *  Operations :
 *      load the dma_t with physical information from AE memory mapping
 *
 *  Return value :
 *
 */
void abe_init_dma_t(abe_port_id id, abe_port_protocol_t *prot)
{
	abe_dma_t_offset dma;
	abe_uint32 idx;

	dma.data = 0;	/* default dma_t points to address 0000... */
	dma.iter = 0;

	switch (prot->protocol_switch) {
	case PINGPONG_PORT_PROT:
		dma.data = (prot->p).prot_pingpong.buf_addr;
		dma.iter = (prot->p).prot_pingpong.buf_size;
		break;
	case DMAREQ_PORT_PROT:
		for (idx = 0; idx < 32; idx++) {
			if (((prot->p).prot_dmareq.dma_data) == (abe_uint32)(1 << idx))
				break;
		}
		dma.data = (CIRCULAR_BUFFER_PERIPHERAL_R__0 + idx*4);
		dma.iter = (prot->p).prot_dmareq.iter;
		break;
	case CIRCULAR_PORT_PROT:
		dma.data = (prot->p).prot_dmareq.buf_addr;
		dma.iter = (prot->p).prot_dmareq.iter;
		break;
	case SLIMBUS_PORT_PROT:
	case SERIAL_PORT_PROT:
	case DMIC_PORT_PROT:
	case MCPDMDL_PORT_PROT:
	case MCPDMUL_PORT_PROT:
	default:
		break;
	}

	/* upload the dma type */
	abe_port [id].dma = dma;
}

/*
 * ABE_ENABLE_DMA_REQUEST
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_disable_enable_dma_request(abe_port_id id, abe_uint32 mulfac)
{
	abe_uint8 desc_second_word[4], irq_dmareq_field;
	abe_uint32 sio_desc_address, sio_id;

	if (abe_port[id].protocol.protocol_switch == DMAREQ_PORT_PROT) {
		irq_dmareq_field =
		(abe_uint8)(mulfac * abe_port[id].protocol.p.prot_dmareq.dma_data);
		sio_id = sio_task_index[id];
		sio_desc_address = dmem_port_descriptors +
					(sio_id * sizeof(ABE_SIODescriptor));

		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		       sio_desc_address + 8, (abe_uint32 *)desc_second_word, 4);
		/*
		 * ABE_uchar data_size;        //8
		 * ABE_uchar smem_addr;        //9
		 * ABE_uchar atc_irq_data;     //10
		 * ABE_uchar counter;          //11
		 */
		desc_second_word[2] = irq_dmareq_field;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			sio_desc_address + 8, (abe_uint32 *)desc_second_word, 4);
	}

	if (abe_port[id].protocol.protocol_switch == PINGPONG_PORT_PROT) {
		irq_dmareq_field =
			(abe_uint8)(mulfac * abe_port[id].protocol.p.prot_pingpong.irq_data);
		sio_desc_address = D_PingPongDesc_ADDR;

		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
			sio_desc_address + 8, (abe_uint32 *)desc_second_word, 4);
		desc_second_word[2] = irq_dmareq_field;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			sio_desc_address + 8, (abe_uint32 *)desc_second_word, 4);
	}
}

void abe_enable_dma_request(abe_port_id id)
{
	abe_disable_enable_dma_request(id, 1);
}

/*
 * ABE_DISABLE_DMA_REQUEST
 *
 * Parameter:
 * Operations:
 * Return value:
 *
 */
void abe_disable_dma_request(abe_port_id id)
{
	abe_disable_enable_dma_request(id, 0);
}


/*
 * ABE_ENABLE_ATC
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_enable_atc(abe_port_id id)
{
	abe_satcdescriptor_aess desc;
	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
	desc.desen = 1;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
}


/*
 * ABE_DISABLE_ATC
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_disable_atc(abe_port_id id)
{
	abe_satcdescriptor_aess desc;

	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
	desc.desen = 0;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
}

/*
 *   ABE_INIT_IO_TASKS
 *
 *  Parameter  :
 *      prot : protocol being used
 *
 *  Operations :
 *      load the micro-task parameters doing to DMEM <==> SMEM data moves
 *
 *       I/O descriptors input parameters :
 *	 For Read from DMEM usually THR1/THR2 = X+1/X-1
 *	 For Write to DMEM usually  THR1/THR2 = 2/0
 *	 UP_1/2 =X+1/X-1
 *
 *  Return value :
 *
 */
void abe_init_io_tasks(abe_port_id id, abe_data_format_t *format, abe_port_protocol_t *prot)
{
	ABE_SIODescriptor desc;
	ABE_SPingPongDescriptor desc_pp;
	abe_uint32 thr, thr1, thr2, iter, direction, data_shift, iter_samples, up1, up2, smem, copy_func_index;
	abe_uint32 iterfactor, dmareq_addr, dmareq_field;
	abe_uint32 atc_desc_address, sio_desc_address, sio_id;
	short VirtAudio_aMultiFrame[PROCESSING_SLOTS][TASKS_IN_SLOT];

	dmareq_addr = ABE_DUMMY_ADDRESS;
	dmareq_field = 0;
	thr = DEFAULT_THR_READ;		/* default threshold */
	atc_desc_address = 0;		/* dummy assignment */

	iter = abe_dma_port_iteration(format);
	iterfactor = abe_dma_port_iter_factor(format);
	iter_samples = (iter / iterfactor); /* number of "samples" either mono or stereo */

	switch (prot->protocol_switch) {
	case DMIC_PORT_PROT:
		atc_desc_address = ABE_ATC_DMIC_DMA_REQ;
		break;
	case MCPDMDL_PORT_PROT:
		atc_desc_address = ABE_ATC_MCPDMDL_DMA_REQ;
		break;
	case MCPDMUL_PORT_PROT:
		atc_desc_address = ABE_ATC_MCPDMUL_DMA_REQ;
		break;
	case PINGPONG_PORT_PROT:
		dmareq_addr = abe_port[id].protocol.p.prot_pingpong.irq_addr;
		dmareq_field = 0;
		break;
	case SLIMBUS_PORT_PROT:
		thr = (abe_int8) abe_port[id].protocol.p.prot_dmareq.thr_flow;
		atc_desc_address = abe_port[id].protocol.p.prot_dmareq.desc_addr;
		break;
	case SERIAL_PORT_PROT:	/* McBSP/McASP */
		thr = (abe_int8) abe_port[id].protocol.p.prot_serial.thr_flow;
		atc_desc_address = (abe_int16) abe_port[id].protocol.p.prot_serial.desc_addr;
		break;
	case DMAREQ_PORT_PROT:	/* DMA w/wo CBPr */
		thr = (abe_int8) abe_port[id].protocol.p.prot_dmareq.thr_flow;
		dmareq_addr = abe_port[id].protocol.p.prot_dmareq.dma_addr;
		dmareq_field = 0;
		atc_desc_address = abe_port[id].protocol.p.prot_dmareq.desc_addr;
		break;
	default:
		break;
	}

	smem = (abe_uint8) abe_port[id].smem_buffer1;
	copy_func_index = (abe_uint8) abe_dma_port_copy_subroutine_id(format, abe_port[id].protocol.direction);

	/* special situation of the PING_PONG protocol which has its own SIO descriptor format */
	/*
	   Sequence of operations on ping-pong buffers B0/B1

	   ----------------------------------------------------------------- time --------------------------------------------->>>>
	   Host Application  is ready to send data from DDR to B0
			     SDMA is initialized from "abe_connect_irq_ping_pong_port" to B0

	   ABE HAL	   init FW to B0
			     send DMAreq to fill B0

	   FIRMWARE			starts sending B1 data, sends DMAreq    v   continue with B0, sends DMAreq v   continue with B1
	   DMAreq		     v (direct access from HAL to AESS regs)      v (from ABE_FW)		    v (from ABE_FW)
	   SDMA		       | fills B0				   | fills B1  ...		    | fills B0  ...
	 */

	if (prot->protocol_switch == PINGPONG_PORT_PROT) {
		/* clear pong memory
		 * dmem_addr = abe_base_address_pingpong [0];
		 * abe_clear_memory(ABE_DMEM, (dmem_addr << 2), abe_size_pingpong);
		 */

		/* load the IO descriptor */
		desc_pp.drift_ASRC	= 0;	/* no drift */
		desc_pp.drift_io	= 0;	/* no drift */
		desc_pp.hw_ctrl_addr	= (abe_uint16)dmareq_addr;
		desc_pp.copy_func_index	= (abe_uint8)copy_func_index;
		desc_pp.smem_addr	= (abe_uint8)smem;
		desc_pp.atc_irq_data	= (abe_uint8)dmareq_field;	/* DMA req 0 is used for CBPr0 */
		desc_pp.x_io		= (abe_uint8)iter_samples;	/* size of block transfer */
		desc_pp.data_size	= (abe_uint8)iterfactor;
		desc_pp.workbuff_BaseAddr  = (abe_uint16)(abe_base_address_pingpong [0]);	/* address comunicated in Bytes */
		desc_pp.workbuff_Samples   = (abe_uint16)((abe_size_pingpong/iterfactor) >> 2);	/* size comunicated in XIO sample */
		desc_pp.nextbuff0_BaseAddr = (abe_uint16)(abe_base_address_pingpong [0]);
		desc_pp.nextbuff0_Samples  = (abe_uint16)((abe_size_pingpong/iterfactor) >> 2);
		desc_pp.nextbuff1_BaseAddr = (abe_uint16)(abe_base_address_pingpong [1]);
		desc_pp.nextbuff1_Samples  = (abe_uint16)((abe_size_pingpong/iterfactor) >> 2);
		desc_pp.counter  = 1;

		/* send a DMA req to fill B0 with N samples
		 * abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, ABE_DMASTATUS_RAW, &(abe_port[id].protocol.p.prot_pingpong.irq_data), 4);
		 */
	} else	{
		/* shift = 0 for mono, shift = 1 for stereo - see update of ATC pointers in IOtasks */
		data_shift = (abe_uint8)((iterfactor > 1) ? 1 : 0);

		/* scales the data threshold to the number of words in DMEM per sample */
		thr = (abe_uint8)(thr * iterfactor);

		/* check is input from ABE point of view */
		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN) {
			/* Firmware compares X+Drift to Thresholds */
			thr2 = (abe_uint8)(iter - thr);
			thr1 = (abe_uint8)(iter + thr);
			direction = 0;
		} else {
			/* thresholds for write access to DMEM */
			thr2 = 0;
			thr1 = (abe_uint8)(thr * 2);
			direction = 3;	/* offset of the write pointer in the ATC descriptor */
		}

		up1 = (abe_uint8)(iter + thr);
		up2 = (abe_uint8)(maximum(0, (abe_int8)iter - (abe_int8)thr));

		desc.drift_ASRC		= 0;	/* no drift */
		desc.drift_io		= 0;	/* no drift */
		desc.hw_ctrl_addr	= (abe_uint16)dmareq_addr;
		desc.copy_func_index	= (abe_uint8)copy_func_index;
		desc.x_io		= (abe_uint8)iter_samples;	/* size of block transfer */
		desc.threshold_1	= (abe_uint8)thr1;		/* Read DMEM : X+1  Write DMEM : 0 */
		desc.threshold_2	= (abe_uint8)thr2;		/* Read DMEM : X-1  Write DMEM : 2 */
		desc.update_1		= (abe_uint8)up1;		/* X+1 */
		desc.update_2		= (abe_uint8)up2;		/* X-1 */
		desc.data_size		= (abe_uint8)data_shift;	/* shift one bit XIO+DRIFT for ATC updates */
		desc.smem_addr		= (abe_uint8)smem;
		desc.flow_counter	= 0;
		desc.atc_irq_data	= (abe_uint8)dmareq_field;	/* DMA req 0 is used for CBPr0 */
		desc.atc_address	= (abe_uint8)(atc_desc_address >> 2); /* Address of ABE/ATC descriptor in WORD32 addresses */
		desc.direction_rw	= (abe_uint8)direction;		/* 0 for read_ptr  3 for write_ptr */
		desc.padding16[0]	= 0;				/* To keep 32bits alignment */
		desc.padding16[1]	= 0;
	}

	/* @@@ to be corrected in FW 06.xx
	   @@@ Only one IO task will be implemented in the single task list
	   @@@ Or two tasks will be implemented with one being disabled from the IO descriptor's related indication
	 */
	if (id == MM_DL_PORT) {
		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof(VirtAudio_aMultiFrame));

		if (prot->protocol_switch == PINGPONG_PORT_PROT)
			VirtAudio_aMultiFrame[1][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_PING_PONG;
		else
			VirtAudio_aMultiFrame[1][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_DL;

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof(VirtAudio_aMultiFrame));
	}

	/* @@@ to be corrected in FW 06.xx
	   @@@ PP descriptor for MM_DL will be overlaid with the SIO descriptor for other transfer protocol
	 */
	if (prot->protocol_switch == PINGPONG_PORT_PROT) {
		sio_desc_address = D_PingPongDesc_ADDR;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc_pp, sizeof(desc_pp));
	} else {
		sio_id = sio_task_index [id];
		sio_desc_address = dmem_port_descriptors + (sio_id * sizeof(ABE_SIODescriptor));
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc, sizeof(desc));
	}
}

/*
 *  ABE_INIT_DMIC
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_init_dmic(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}

/*
 *  ABE_INIT_MCPDM
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_init_mcpdm(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}

/*
 *  ABE_RESET_FEATURE
 *
 *  Parameter  :
 *      x : index of the feature to be initialized
 *
 *  Operations :
 *      reload the configuration
 *
 *  Return value :
 *
 */
void abe_reset_one_feature(abe_uint32 x)
{
	all_feature[x] = all_feature_init[x];	/* load default fields */
	/* @@@@ abe_call_subroutine((all_feature[x]).disable_feature, NOPARAMETER, NOPARAMETER, NOPARAMETER, NOPARAMETER);  */
}

/*
 *  ABE_RESET_ALL_FEATURE
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for all features
 *	 struct {
 *		uint16 load_default_data;
 *		uint16 read_parameter;
 *		uint16 write_parameter;
 *		uint16 running_status;
 *		uint16 fw_input_buffer_address;
 *		uint16 fw_output_buffer_address;
 *		uint16 fw_scheduler_slot_position;
 *		uint16 fw_scheduler_subslot_position;
 *		uint16 min_opp;
 *		char name[NBCHARFEATURENAME];
 *	} abe_feature_t;
 *
 *  Return value :
 *
 */
void abe_reset_all_features(void)
{
	abe_uint16 i;

	for (i = 0; i < FEAT_EANC; i++)
		abe_reset_one_feature(i);
}

/*  ABE_RESET_ONE_PORT
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for one port
 *
 *  Return value :
 *
 */
void abe_reset_one_port(abe_uint32 x)
{
	abe_port[x] = abe_port_init[x];

	/* @@@ stop ATC activity .. */
}

/*
 *  ABE_RESET_ALL_PORTS
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for all features
 *
 *  Return value :
 *
 */
void abe_reset_all_ports(void)
{
	abe_uint16 i;

	for (i = 0; i < MAXNBABEPORTS; i++)
		abe_reset_one_port(i);
}
