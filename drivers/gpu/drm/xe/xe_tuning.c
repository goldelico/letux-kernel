// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "xe_tuning.h"

#include <kunit/visibility.h>

#include "regs/xe_gt_regs.h"
#include "xe_gt_types.h"
#include "xe_platform_types.h"
#include "xe_rtp.h"

#undef XE_REG_MCR
#define XE_REG_MCR(...)     XE_REG(__VA_ARGS__, .mcr = 1)

static const struct xe_rtp_entry_sr gt_tunings[] = {
	{ XE_RTP_NAME("Tuning: Blend Fill Caching Optimization Disable"),
	  XE_RTP_RULES(PLATFORM(DG2)),
	  XE_RTP_ACTIONS(SET(XEHP_L3SCQREG7, BLEND_FILL_CACHING_OPT_DIS))
	},
	{ XE_RTP_NAME("Tuning: 32B Access Enable"),
	  XE_RTP_RULES(PLATFORM(DG2)),
	  XE_RTP_ACTIONS(SET(XEHP_SQCM, EN_32B_ACCESS))
	},

	/* Xe2 */

	{ XE_RTP_NAME("Tuning: L3 cache"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(2001, XE_RTP_END_VERSION_UNDEFINED)),
	  XE_RTP_ACTIONS(FIELD_SET(XEHP_L3SQCREG5, L3_PWM_TIMER_INIT_VAL_MASK,
				   REG_FIELD_PREP(L3_PWM_TIMER_INIT_VAL_MASK, 0x7f)))
	},
	{ XE_RTP_NAME("Tuning: L3 cache - media"),
	  XE_RTP_RULES(MEDIA_VERSION(2000)),
	  XE_RTP_ACTIONS(FIELD_SET(XE2LPM_L3SQCREG5, L3_PWM_TIMER_INIT_VAL_MASK,
				   REG_FIELD_PREP(L3_PWM_TIMER_INIT_VAL_MASK, 0x7f)))
	},
	{ XE_RTP_NAME("Tuning: Compression Overfetch"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(2001, XE_RTP_END_VERSION_UNDEFINED)),
	  XE_RTP_ACTIONS(CLR(CCCHKNREG1, ENCOMPPERFFIX),
			 SET(CCCHKNREG1, L3CMPCTRL))
	},
	{ XE_RTP_NAME("Tuning: Compression Overfetch - media"),
	  XE_RTP_RULES(MEDIA_VERSION(2000)),
	  XE_RTP_ACTIONS(CLR(XE2LPM_CCCHKNREG1, ENCOMPPERFFIX),
			 SET(XE2LPM_CCCHKNREG1, L3CMPCTRL))
	},
	{ XE_RTP_NAME("Tuning: Enable compressible partial write overfetch in L3"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(2001, XE_RTP_END_VERSION_UNDEFINED)),
	  XE_RTP_ACTIONS(SET(L3SQCREG3, COMPPWOVERFETCHEN))
	},
	{ XE_RTP_NAME("Tuning: Enable compressible partial write overfetch in L3 - media"),
	  XE_RTP_RULES(MEDIA_VERSION(2000)),
	  XE_RTP_ACTIONS(SET(XE2LPM_L3SQCREG3, COMPPWOVERFETCHEN))
	},
	{ XE_RTP_NAME("Tuning: L2 Overfetch Compressible Only"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(2001, XE_RTP_END_VERSION_UNDEFINED)),
	  XE_RTP_ACTIONS(SET(L3SQCREG2,
			     COMPMEMRD256BOVRFETCHEN))
	},
	{ XE_RTP_NAME("Tuning: L2 Overfetch Compressible Only - media"),
	  XE_RTP_RULES(MEDIA_VERSION(2000)),
	  XE_RTP_ACTIONS(SET(XE2LPM_L3SQCREG2,
			     COMPMEMRD256BOVRFETCHEN))
	},
	{ XE_RTP_NAME("Tuning: Stateless compression control"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(2001, XE_RTP_END_VERSION_UNDEFINED)),
	  XE_RTP_ACTIONS(FIELD_SET(STATELESS_COMPRESSION_CTRL, UNIFIED_COMPRESSION_FORMAT,
				   REG_FIELD_PREP(UNIFIED_COMPRESSION_FORMAT, 0)))
	},
	{ XE_RTP_NAME("Tuning: Stateless compression control - media"),
	  XE_RTP_RULES(MEDIA_VERSION_RANGE(1301, 2000)),
	  XE_RTP_ACTIONS(FIELD_SET(STATELESS_COMPRESSION_CTRL, UNIFIED_COMPRESSION_FORMAT,
				   REG_FIELD_PREP(UNIFIED_COMPRESSION_FORMAT, 0)))
	},
	{ XE_RTP_NAME("Tuning: L3 RW flush all Cache"),
	  XE_RTP_RULES(GRAPHICS_VERSION(2004)),
	  XE_RTP_ACTIONS(SET(SCRATCH3_LBCF, RWFLUSHALLEN))
	},
	{ XE_RTP_NAME("Tuning: L3 RW flush all cache - media"),
	  XE_RTP_RULES(MEDIA_VERSION(2000)),
	  XE_RTP_ACTIONS(SET(XE2LPM_SCRATCH3_LBCF, RWFLUSHALLEN))
	},

	{}
};

static const struct xe_rtp_entry_sr engine_tunings[] = {
	{ XE_RTP_NAME("Tuning: Set Indirect State Override"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(1200, 1274),
		       ENGINE_CLASS(RENDER)),
	  XE_RTP_ACTIONS(SET(SAMPLER_MODE, INDIRECT_STATE_BASE_ADDR_OVERRIDE))
	},
	{}
};

static const struct xe_rtp_entry_sr lrc_tunings[] = {
	/* DG2 */

	{ XE_RTP_NAME("Tuning: L3 cache"),
	  XE_RTP_RULES(PLATFORM(DG2), ENGINE_CLASS(RENDER)),
	  XE_RTP_ACTIONS(FIELD_SET(XEHP_L3SQCREG5, L3_PWM_TIMER_INIT_VAL_MASK,
				   REG_FIELD_PREP(L3_PWM_TIMER_INIT_VAL_MASK, 0x7f)))
	},
	{ XE_RTP_NAME("Tuning: TDS gang timer"),
	  XE_RTP_RULES(PLATFORM(DG2), ENGINE_CLASS(RENDER)),
	  /* read verification is ignored as in i915 - need to check enabling */
	  XE_RTP_ACTIONS(FIELD_SET_NO_READ_MASK(XEHP_FF_MODE2,
						FF_MODE2_TDS_TIMER_MASK,
						FF_MODE2_TDS_TIMER_128))
	},
	{ XE_RTP_NAME("Tuning: TBIMR fast clip"),
	  XE_RTP_RULES(PLATFORM(DG2), ENGINE_CLASS(RENDER)),
	  XE_RTP_ACTIONS(SET(CHICKEN_RASTER_2, TBIMR_FAST_CLIP))
	},

	/* Xe_LPG */

	{ XE_RTP_NAME("Tuning: L3 cache"),
	  XE_RTP_RULES(GRAPHICS_VERSION_RANGE(1270, 1274), ENGINE_CLASS(RENDER)),
	  XE_RTP_ACTIONS(FIELD_SET(XEHP_L3SQCREG5, L3_PWM_TIMER_INIT_VAL_MASK,
				   REG_FIELD_PREP(L3_PWM_TIMER_INIT_VAL_MASK, 0x7f)))
	},

	/* Xe2_HPG */

	{ XE_RTP_NAME("Tuning: vs hit max value"),
	  XE_RTP_RULES(GRAPHICS_VERSION(2001), ENGINE_CLASS(RENDER)),
	  XE_RTP_ACTIONS(FIELD_SET(FF_MODE, VS_HIT_MAX_VALUE_MASK,
				   REG_FIELD_PREP(VS_HIT_MAX_VALUE_MASK, 0x3f)))
	},

	{}
};

void xe_tuning_process_gt(struct xe_gt *gt)
{
	struct xe_rtp_process_ctx ctx = XE_RTP_PROCESS_CTX_INITIALIZER(gt);

	xe_rtp_process_to_sr(&ctx, gt_tunings, &gt->reg_sr);
}
EXPORT_SYMBOL_IF_KUNIT(xe_tuning_process_gt);

void xe_tuning_process_engine(struct xe_hw_engine *hwe)
{
	struct xe_rtp_process_ctx ctx = XE_RTP_PROCESS_CTX_INITIALIZER(hwe);

	xe_rtp_process_to_sr(&ctx, engine_tunings, &hwe->reg_sr);
}
EXPORT_SYMBOL_IF_KUNIT(xe_tuning_process_engine);

/**
 * xe_tuning_process_lrc - process lrc tunings
 * @hwe: engine instance to process tunings for
 *
 * Process LRC table for this platform, saving in @hwe all the tunings that need
 * to be applied on context restore. These are tunings touching registers that
 * are part of the HW context image.
 */
void xe_tuning_process_lrc(struct xe_hw_engine *hwe)
{
	struct xe_rtp_process_ctx ctx = XE_RTP_PROCESS_CTX_INITIALIZER(hwe);

	xe_rtp_process_to_sr(&ctx, lrc_tunings, &hwe->reg_lrc);
}
