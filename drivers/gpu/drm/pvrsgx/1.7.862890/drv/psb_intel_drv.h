/*
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef __INTEL_DRV_H__
#define __INTEL_DRV_H__

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <linux/gpio.h>

/*
 * Display related stuff
 */

/* store information about an Ixxx DVO */
/* The i830->i865 use multiple DVOs with multiple i2cs */
/* the i915, i945 have a single sDVO i2c bus - which is different */
#define MAX_OUTPUTS 6
/* maximum connectors per crtcs in the mode set */
#define INTELFB_CONN_LIMIT 4

#define INTEL_I2C_BUS_DVO 1
#define INTEL_I2C_BUS_SDVO 2

/* these are outputs from the chip - integrated only
 * external chips are via DVO or SDVO output */
#define INTEL_OUTPUT_UNUSED 0
#define INTEL_OUTPUT_ANALOG 1
#define INTEL_OUTPUT_DVO 2
#define INTEL_OUTPUT_SDVO 3
#define INTEL_OUTPUT_LVDS 4
#define INTEL_OUTPUT_TVOUT 5
#define INTEL_OUTPUT_HDMI 6
#define INTEL_OUTPUT_DISPLAYPORT 7
#define INTEL_OUTPUT_EDP 8

#define INTEL_DVO_CHIP_NONE 0
#define INTEL_DVO_CHIP_LVDS 1
#define INTEL_DVO_CHIP_TMDS 2
#define INTEL_DVO_CHIP_TVOUT 4

struct psb_intel_clock_t
{
	/* given values */
	int n;
	int m1, m2;
	int p1, p2;
	/* derived values */
	int dot;
	int vco;
	int m;
	int p;
};
/**
 * Hold information useally put on the device driver privates here,
 * since it needs to be shared across multiple of devices drivers privates.
*/
struct psb_intel_mode_device {

	/*
	 * Abstracted memory manager operations
	 */
	void *(*bo_from_handle) (struct drm_device *dev,
				 struct drm_file *file_priv,
				 unsigned int handle);
	size_t(*bo_size) (struct drm_device *dev, void *bo);
	size_t(*bo_offset) (struct drm_device *dev, void *bo);
	int (*bo_pin_for_scanout) (struct drm_device *dev, void *bo);
	int (*bo_unpin_for_scanout) (struct drm_device *dev, void *bo);

	/*
	 * Cursor
	 */
	int cursor_needs_physical;

	/*
	 * LVDS info
	 */
	int backlight_duty_cycle;	/* restore backlight to this value */
	int backlight_enabled;
	bool panel_wants_dither;
	struct drm_display_mode *panel_fixed_mode;
	struct drm_display_mode *panel_fixed_mode2;
	struct drm_display_mode *vbt_mode;	/* if any */

	uint32_t saveBLC_PWM_CTL;
};

struct psb_intel_i2c_chan {
	/* for getting at dev. private (mmio etc.) */
	struct drm_device *drm_dev;
	u32 reg;		/* GPIO reg */
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo;
	u8 slave_addr;
};

#define DDI0_SELECT	1
#define DDI1_SELECT	2
#define DP_MASK		0x8000
#define DDI_MASK	0x3

struct psb_intel_output {
	struct drm_connector base;

	struct drm_encoder enc;
	int type;
#if 1 /*MDFLD_HDMI_JLIU7*/
	struct i2c_adapter *hdmi_i2c_adapter;	/* for control functions */
#endif
	struct psb_intel_i2c_chan *i2c_bus;	/* for control functions */
	struct psb_intel_i2c_chan *ddc_bus;	/* for DDC only stuff */
	bool load_detect_temp;
	void *dev_priv;
	u32	ddi_select;

	struct psb_intel_mode_device *mode_dev;

};

struct psb_intel_crtc_state {
	uint32_t saveDSPCNTR;
	uint32_t savePIPECONF;
	uint32_t savePIPESRC;
	uint32_t saveDPLL;
	uint32_t saveFP0;
	uint32_t saveFP1;
	uint32_t saveHTOTAL;
	uint32_t saveHBLANK;
	uint32_t saveHSYNC;
	uint32_t saveVTOTAL;
	uint32_t saveVBLANK;
	uint32_t saveVSYNC;
	uint32_t saveDSPSTRIDE;
	uint32_t saveDSPSIZE;
	uint32_t saveDSPPOS;
	uint32_t saveDSPBASE;
	uint32_t savePalette[256];
};

struct psb_intel_crtc {
	struct drm_crtc base;
	int pipe;
	int plane;
	uint32_t cursor_addr;
	u8 lut_r[256], lut_g[256], lut_b[256];
	u8 lut_adj[256];
	struct psb_intel_framebuffer *fbdev_fb;
	/* a mode_set for fbdev users on this crtc */
	struct drm_mode_set mode_set;

	/* current bo we scanout from */
	void *scanout_bo;

	/* current bo we cursor from */
	void *cursor_bo;
	void *cursor_handle;

	struct drm_display_mode saved_mode;
	struct drm_display_mode saved_adjusted_mode;

	struct psb_intel_mode_device *mode_dev;

	/*crtc mode setting flags*/
	u32 mode_flags;

/*FIXME: Workaround to avoid MRST block.*/
	/* Saved Crtc HW states */
	struct psb_intel_crtc_state *crtc_state;

	bool active;
};

#define to_psb_intel_crtc(x)	\
		container_of(x, struct psb_intel_crtc, base)
#define to_psb_intel_output(x)	\
		container_of(x, struct psb_intel_output, base)
#define enc_to_psb_intel_output(x)	\
		container_of(x, struct psb_intel_output, enc)
#define to_psb_intel_framebuffer(x)	\
		container_of(x, struct psb_intel_framebuffer, base)

struct psb_intel_i2c_chan *psb_intel_i2c_create(struct drm_device *dev,
					const u32 reg, const char *name);
void psb_intel_i2c_destroy(struct psb_intel_i2c_chan *chan);
int psb_intel_ddc_get_modes(struct psb_intel_output *psb_intel_output);
extern bool psb_intel_ddc_probe(struct psb_intel_output *psb_intel_output);

extern void psb_intel_crtc_init(struct drm_device *dev, int pipe,
			    struct psb_intel_mode_device *mode_dev);
extern void psb_intel_crt_init(struct drm_device *dev,
			    struct psb_intel_mode_device *mode_dev);
extern void psb_intel_tv_init(struct drm_device *dev);
extern void psb_intel_lvds_init(struct drm_device *dev,
			    struct psb_intel_mode_device *mode_dev);
extern void mdfld_hdmi_init(struct drm_device *dev,
			   struct psb_intel_mode_device *mode_dev, int reg);

extern void psb_intel_dp_init(struct drm_device *dev, struct psb_intel_mode_device *mode_dev, int output_reg);

extern void psb_intel_crtc_load_lut(struct drm_crtc *crtc);
extern void psb_intel_encoder_prepare(struct drm_encoder *encoder);
extern void psb_intel_encoder_commit(struct drm_encoder *encoder);

extern struct drm_encoder *psb_intel_best_encoder(struct drm_connector
					      *connector);

extern struct drm_display_mode *psb_intel_crtc_mode_get(struct drm_device *dev,
						    struct drm_crtc *crtc);
extern void psb_intel_wait_for_vblank(struct drm_device *dev);
extern int psb_intel_get_pipe_from_crtc_id(struct drm_device *dev, void *data,
				struct drm_file *file_priv);
extern struct drm_crtc *psb_intel_get_crtc_from_pipe(struct drm_device *dev,
						 int pipe);
extern int intelfb_probe(struct drm_device *dev);
extern int intelfb_remove(struct drm_device *dev,
			  struct drm_framebuffer *fb);
extern struct drm_framebuffer *psb_intel_framebuffer_create(struct drm_device
							*dev, struct
							drm_mode_fb_cmd
							*mode_cmd,
							void *mm_private);
extern bool psb_intel_lvds_mode_fixup(struct drm_encoder *encoder,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode);
extern int psb_intel_lvds_mode_valid(struct drm_connector *connector,
				     struct drm_display_mode *mode);
extern int psb_intel_lvds_set_property(struct drm_connector *connector,
					struct drm_property *property,
					uint64_t value);
extern void psb_intel_lvds_destroy(struct drm_connector *connector);
extern const struct drm_encoder_funcs psb_intel_lvds_enc_funcs;

extern uint8_t blc_pol;
extern uint8_t blc_freq;

extern int psb_dpll_set_clock_cdv(struct drm_device *dev, struct drm_crtc *crtc,
				  struct psb_intel_clock_t *clock, bool is_lvds, u32 ddi_select);
extern int
psb_sb_read(struct drm_device *dev, u32 reg, u32 *val);
extern void
psb_sb_reset(struct drm_device *dev);
extern int
psb_sb_write(struct drm_device *dev, u32 reg, u32 val);

extern void psb_intel_dp_set_m_n(struct drm_crtc *crtc,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode);

extern void psb_intel_attach_force_audio_property(struct drm_connector *connector);
extern void psb_intel_attach_broadcast_rgb_property(struct drm_connector *connector);

void
psb_intel_fixed_panel_mode(struct drm_display_mode *fixed_mode,
		       struct drm_display_mode *adjusted_mode);

extern void psb_intel_panel_enable_backlight(struct drm_device *dev);
extern void psb_intel_panel_disable_backlight(struct drm_device *dev);

extern void psb_intel_panel_set_backlight(struct drm_device *dev, int level);
/* psb_intel_panel.c */
extern u32 psb_intel_panel_get_max_backlight(struct drm_device *dev);
extern u32 psb_intel_panel_get_backlight(struct drm_device *dev);

extern void psb_intel_panel_destroy_backlight(struct drm_device *dev);
extern int psb_intel_panel_setup_backlight(struct drm_device *dev);

extern void psb_intel_clock_gating (struct drm_device *dev);
#endif				/* __INTEL_DRV_H__ */
