/*
 * Copyright © 2011 Intel Corporation
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
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *	Dave Airlie <airlied@linux.ie>
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
#include <linux/dmi.h>
/* #include <drm/drm_crtc.h> */
/* #include <drm/drm_edid.h> */
#include <drm/drmP.h>

#include "psb_intel_bios.h"
#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"
#include <linux/pm_runtime.h>

/* MRST defines start */
uint8_t blc_freq;
uint8_t blc_minbrightness;
uint8_t blc_i2caddr;
uint8_t blc_brightnesscmd;
int lvds_backlight;		/* restore backlight to this value */

u32 CoreClock;
u32 PWMControlRegFreq;

/**
 * LVDS I2C backlight control macros
 */
#define BLC_I2C_TYPE	0x01
#define BLC_PWM_TYPT	0x02

#define BLC_POLARITY_NORMAL 0
#define BLC_POLARITY_INVERSE 1

#define PSB_BLC_MAX_PWM_REG_FREQ       (0xFFFE)
#define PSB_BLC_MIN_PWM_REG_FREQ	(0x2)
#define PSB_BLC_PWM_PRECISION_FACTOR	(10)
#define PSB_BACKLIGHT_PWM_MODULATION_MASK	(0xFFFF0000)
#define PSB_BACKLIGHT_PWM_CTL_SHIFT	(16)
#define PSB_BACKLIGHT_PWM_POLARITY_BIT_CLEAR (0xFFFE)

struct psb_intel_lvds_priv {
	/**
	 * Saved LVDO output states
	 */
	uint32_t savePP_ON;
	uint32_t savePP_OFF;
	uint32_t saveLVDS;
	uint32_t savePP_CONTROL;
	uint32_t savePP_CYCLE;
	uint32_t savePFIT_CONTROL;
	uint32_t savePFIT_PGM_RATIOS;
	uint32_t saveBLC_PWM_CTL;
};



/**
 * Sets the power state for the panel.
 */
static void psb_intel_lvds_set_power(struct drm_device *dev,
				 struct psb_intel_output *output, bool on)
{
	u32 pp_status;

	if (on) {

		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) |
			  POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while ((pp_status & PP_ON) == 0);

		psb_intel_panel_enable_backlight(dev);
	} else {
		psb_intel_panel_disable_backlight(dev);

		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) &
			  ~POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while (pp_status & PP_ON);
	}
}

static void psb_intel_lvds_enable(struct drm_device *dev, struct psb_intel_output *output)
{
	REG_WRITE(LVDS, REG_READ(LVDS) | LVDS_PORT_EN);

	psb_intel_lvds_set_power(dev, output, true);
}

static void psb_intel_lvds_disable(struct drm_device *dev, struct psb_intel_output *output)
{
	psb_intel_lvds_set_power(dev, output, false);

	REG_WRITE(LVDS, REG_READ(LVDS) & ~LVDS_PORT_EN);
}

static void psb_intel_lvds_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	if (mode == DRM_MODE_DPMS_ON)
		psb_intel_lvds_enable(dev, output);
	else
		psb_intel_lvds_disable(dev, output);

	/* XXX: We never power down the LVDS pairs. */
}

static void psb_intel_lvds_save(struct drm_connector *connector)
{
	return;
}

static void psb_intel_lvds_restore(struct drm_connector *connector)
{
	return;
}

int psb_intel_lvds_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct psb_intel_output *psb_intel_output =
				to_psb_intel_output(connector);
	struct drm_display_mode *fixed_mode =
	    psb_intel_output->mode_dev->panel_fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	if (fixed_mode) {
		if (mode->hdisplay > fixed_mode->hdisplay)
			return MODE_PANEL;
		if (mode->vdisplay > fixed_mode->vdisplay)
			return MODE_PANEL;
	}
	return MODE_OK;
}

bool psb_intel_lvds_mode_fixup(struct drm_encoder *encoder,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	struct psb_intel_mode_device *mode_dev =
	    enc_to_psb_intel_output(encoder)->mode_dev;
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc =
				to_psb_intel_crtc(encoder->crtc);
	struct drm_encoder *tmp_encoder;
	struct drm_display_mode *panel_fixed_mode = mode_dev->panel_fixed_mode;
	struct psb_intel_output *psb_intel_output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("type = 0x%x, pipe = %d.\n",psb_intel_output->type, psb_intel_crtc->pipe);

	/* Should never happen!! */
	if (IS_MID(dev) && !IS_CDV(dev) && psb_intel_crtc->pipe == 1) {
		DRM_ERROR("Can't support LVDS/MIPI on pipe B on MRST\n");
		return false;
	} else if (!IS_MID(dev) && !IS_I965G(dev)
		   && psb_intel_crtc->pipe == 0) {
		DRM_ERROR("Can't support LVDS on pipe A\n");
		return false;
	}

	/* Should never happen!! */
	list_for_each_entry(tmp_encoder, &dev->mode_config.encoder_list,
			    head) {
		if (tmp_encoder != encoder
		    && tmp_encoder->crtc == encoder->crtc) {
			DRM_ERROR("Can't enable LVDS and another "
			       "encoder on the same pipe\n");
			return false;
		}
	}

	/*
	 * If we have timings from the BIOS for the panel, put them in
	 * to the adjusted mode.  The CRTC will be set up for this mode,
	 * with the panel scaling set up to source from the H/VDisplay
	 * of the original mode.
	 */
	if (panel_fixed_mode != NULL)
		psb_intel_fixed_panel_mode(panel_fixed_mode, adjusted_mode);

	/*
	 * XXX: It would be nice to support lower refresh rates on the
	 * panels to reduce power consumption, and perhaps match the
	 * user's requested refresh rate.
	 */

	return true;
}

static void psb_intel_lvds_prepare(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("\n");

	psb_intel_lvds_set_power(dev, output, false);
}

static void psb_intel_lvds_commit(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("\n");

	if (dev_priv->backlight_level == 0)
		dev_priv->backlight_level = psb_intel_panel_get_max_backlight(dev);

	psb_intel_lvds_set_power(dev, output, true);
}

static void psb_intel_lvds_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(
							encoder->crtc);
	u32 pfit_control;

	/*
	 * The LVDS pin pair will already have been turned on in the
	 * psb_intel_crtc_mode_set since it has a large impact on the DPLL
	 * settings.
	 */

	/*
	 * Enable automatic panel scaling so that non-native modes fill the
	 * screen.  Should be enabled before the pipe is enabled, according to
	 * register description and PRM.
	 */
	if (mode->hdisplay != adjusted_mode->hdisplay ||
	    mode->vdisplay != adjusted_mode->vdisplay)
		pfit_control = PFIT_ENABLE;
	else
		pfit_control = 0;

	pfit_control |= psb_intel_crtc->pipe << PFIT_PIPE_SHIFT;

	REG_WRITE(PFIT_CONTROL, pfit_control);
}

/**
 * Detect the LVDS connection.
 *
 * This always returns CONNECTOR_STATUS_CONNECTED.
 * This connector should only have
 * been set up if the LVDS was actually connected anyway.
 */
static enum drm_connector_status psb_intel_lvds_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

/**
 * Return the list of DDC modes if available, or the BIOS fixed mode otherwise.
 */
static int psb_intel_lvds_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *psb_intel_output =
					to_psb_intel_output(connector);
	struct psb_intel_mode_device *mode_dev =
					psb_intel_output->mode_dev;
	int ret = 0;

	ret = psb_intel_ddc_get_modes(psb_intel_output);

	if (ret)
		return ret;

	/* Didn't get an EDID, so
	 * Set wide sync ranges so we get all modes
	 * handed to valid_mode for checking
	 */
	connector->display_info.min_vfreq = 0;
	connector->display_info.max_vfreq = 200;
	connector->display_info.min_hfreq = 0;
	connector->display_info.max_hfreq = 200;

	if (mode_dev->panel_fixed_mode != NULL) {
		struct drm_display_mode *mode =
		    drm_mode_duplicate(dev, mode_dev->panel_fixed_mode);
		drm_mode_probed_add(connector, mode);
		return 1;
	}

	return 0;
}

/**
 * psb_intel_lvds_destroy - unregister and free LVDS structures
 * @connector: connector to free
 *
 * Unregister the DDC bus for this connector then free the driver private
 * structure.
 */
void psb_intel_lvds_destroy(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output =
					to_psb_intel_output(connector);

	psb_intel_panel_destroy_backlight(connector->dev);

	if (psb_intel_output->ddc_bus)
		psb_intel_i2c_destroy(psb_intel_output->ddc_bus);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

int psb_intel_lvds_set_property(struct drm_connector *connector,
				       struct drm_property *property,
				       uint64_t value)
{
	struct drm_encoder *pEncoder = connector->encoder;

	PSB_DEBUG_ENTRY("\n");

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		struct psb_intel_crtc *pPsbCrtc =
					to_psb_intel_crtc(pEncoder->crtc);
		uint64_t curValue;

		PSB_DEBUG_ENTRY("scaling mode \n");

		if (!pPsbCrtc)
			goto set_prop_error;

		switch (value) {
		case DRM_MODE_SCALE_FULLSCREEN:
			break;
		case DRM_MODE_SCALE_NO_SCALE:
			break;
		case DRM_MODE_SCALE_ASPECT:
			break;
		default:
			goto set_prop_error;
		}

		if (drm_connector_property_get_value(connector,
						     property,
						     &curValue))
			goto set_prop_error;

		if (curValue == value)
			goto set_prop_done;

		if (drm_connector_property_set_value(connector,
							property,
							value))
			goto set_prop_error;

		if (pPsbCrtc->saved_mode.hdisplay != 0 &&
		    pPsbCrtc->saved_mode.vdisplay != 0) {
			if (!drm_crtc_helper_set_mode(pEncoder->crtc,
						      &pPsbCrtc->saved_mode,
						      pEncoder->crtc->x,
						      pEncoder->crtc->y,
						      pEncoder->crtc->fb))
				goto set_prop_error;
		}
	} 

set_prop_done:
	return 0;
set_prop_error:
	return -1;
}

static const struct drm_encoder_helper_funcs psb_intel_lvds_helper_funcs = {
	.dpms = psb_intel_lvds_encoder_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = psb_intel_lvds_prepare,
	.mode_set = psb_intel_lvds_mode_set,
	.commit = psb_intel_lvds_commit,
};

static const struct drm_connector_helper_funcs psb_intel_lvds_connector_helper_funcs = {
	.get_modes = psb_intel_lvds_get_modes,
	.mode_valid = psb_intel_lvds_mode_valid,
	.best_encoder = psb_intel_best_encoder,
};

static const struct drm_connector_funcs psb_intel_lvds_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.save = psb_intel_lvds_save,
	.restore = psb_intel_lvds_restore,
	.detect = psb_intel_lvds_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = psb_intel_lvds_set_property,
	.destroy = psb_intel_lvds_destroy,
};


static void psb_intel_lvds_enc_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

const struct drm_encoder_funcs psb_intel_lvds_enc_funcs = {
	.destroy = psb_intel_lvds_enc_destroy,
};

/*
 * Enumerate the child dev array parsed from VBT to check whether
 * the LVDS is present.
 * If it is present, return 1.
 * If it is not present, return false.
 * If no child dev is parsed from VBT, it assumes that the LVDS is present.
 */
static bool lvds_is_present_in_vbt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int i;

	if (!dev_priv->child_dev_num)
		return true;

	for (i = 0; i < dev_priv->child_dev_num; i++) {
		struct child_device_config *child = dev_priv->child_dev + i;

		/* If the device type is not LFP, continue.
		 * We have to check both the new identifiers as well as the
		 * old for compatibility with some BIOSes.
		 */
		if (child->device_type != DEVICE_TYPE_INT_LFP &&
		    child->device_type != DEVICE_TYPE_LFP)
			continue;


		/* However, we cannot trust the BIOS writers to populate
		 * the VBT correctly.  Since LVDS requires additional
		 * information from AIM blocks, a non-zero addin offset is
		 * a good indicator that the LVDS is actually present.
		 */
		if (child->addin_offset)
			return true;

		/* But even then some BIOS writers perform some black magic
		 * and instantiate the device without reference to any
		 * additional data.  Trust that if the VBT was written into
		 * the OpRegion then they have validated the LVDS's existence.
		 */
		if (dev_priv->opregion.vbt)
			return true;
	}

	return false;
}

/**
 * psb_intel_lvds_init - setup LVDS connectors on this device
 * @dev: drm device
 *
 * Create the connector, register the LVDS DDC bus, and try to figure out what
 * modes we can display on the LVDS panel (if present).
 */
void psb_intel_lvds_init(struct drm_device *dev,
		     struct psb_intel_mode_device *mode_dev)
{
	struct psb_intel_output *psb_intel_output;
	struct psb_intel_lvds_priv *lvds_priv;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_display_mode *scan;	/* *modes, *bios_mode; */
	struct drm_crtc *crtc;
	struct drm_psb_private *dev_priv = (struct drm_psb_private *)dev->dev_private;
	u32 lvds;
	int pipe;

	if (!lvds_is_present_in_vbt(dev)) {
		DRM_DEBUG_KMS("LVDS is not present in VBT\n");
		return;
	}

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output) +
				   sizeof(struct psb_intel_lvds_priv), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	lvds_priv = (struct psb_intel_lvds_priv *)(psb_intel_output + 1);

	psb_intel_output->dev_priv = lvds_priv;

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	drm_connector_init(dev, &psb_intel_output->base,
			   &psb_intel_lvds_connector_funcs,
			   DRM_MODE_CONNECTOR_LVDS);

	drm_encoder_init(dev, &psb_intel_output->enc,
			 &psb_intel_lvds_enc_funcs,
			 DRM_MODE_ENCODER_LVDS);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type = INTEL_OUTPUT_LVDS;

	drm_encoder_helper_add(encoder, &psb_intel_lvds_helper_funcs);
	drm_connector_helper_add(connector,
				 &psb_intel_lvds_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	/*Attach connector properties*/
	drm_connector_attach_property(connector,
				      dev->mode_config.scaling_mode_property,
				      DRM_MODE_SCALE_FULLSCREEN);

	/**
	 * Set up I2C bus
	 * FIXME: distroy i2c_bus when exit
	 */
	psb_intel_output->i2c_bus = psb_intel_i2c_create(dev,
							 GPIOB,
							 "LVDSBLC_B");
	if (!psb_intel_output->i2c_bus) {
		DRM_ERROR("I2C bus registration failed.\n");
		goto failed_blc_i2c;
	}
	psb_intel_output->i2c_bus->slave_addr = 0x2C;
	dev_priv->lvds_i2c_bus =  psb_intel_output->i2c_bus;

	/*
	 * LVDS discovery:
	 * 1) check for EDID on DDC
	 * 2) check for VBT data
	 * 3) check to see if LVDS is already on
	 *    if none of the above, no panel
	 * 4) make sure lid is open
	 *    if closed, act like it's not there for now
	 */

	/* Set up the DDC bus. */
	psb_intel_output->ddc_bus = psb_intel_i2c_create(dev,
							 GPIOC,
							 "LVDSDDC_C");
	if (!psb_intel_output->ddc_bus) {
		DRM_ERROR("DDC bus registration failed.\n");
		goto failed_ddc;
	}

	/*
	 * Attempt to get the fixed panel mode from DDC.  Assume that the
	 * preferred mode is the right one.
	 */
	psb_intel_ddc_get_modes(psb_intel_output);
	list_for_each_entry(scan, &connector->probed_modes, head) {
		DRM_DEBUG("scan type: %d hdisplay: %d vdisplay %d", scan->type, scan->hdisplay, scan->vdisplay);
		if (scan->type & DRM_MODE_TYPE_PREFERRED) {
			mode_dev->panel_fixed_mode = drm_mode_duplicate(dev, scan);
			goto out;	/* FIXME: check for quirks */
		}
	}

	/* Failed to get EDID, what about VBT? do we need this?*/

	if (dev_priv->lfp_lvds_vbt_mode) {
		mode_dev->panel_fixed_mode = drm_mode_duplicate(dev, dev_priv->lfp_lvds_vbt_mode);
		if (mode_dev->panel_fixed_mode) {
			mode_dev->panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
			goto out;	/* FIXME: check for quirks */
		}
	}

	/*
	 * If we didn't get EDID, try checking if the panel is already turned
	 * on.	If so, assume that whatever is currently programmed is the
	 * correct mode.
	 */
	lvds = REG_READ(LVDS);
	pipe = (lvds & LVDS_PIPEB_SELECT) ? 1 : 0;
	crtc = psb_intel_get_crtc_from_pipe(dev, pipe);

	if (crtc && (lvds & LVDS_PORT_EN)) {
		mode_dev->panel_fixed_mode = psb_intel_crtc_mode_get(dev, crtc);
		if (mode_dev->panel_fixed_mode) {
			mode_dev->panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
			goto out;	/* FIXME: check for quirks */
		}
	}

	/* If we still don't have a mode after all that, give up. */
	if (!mode_dev->panel_fixed_mode) {
		DRM_DEBUG("Found no modes on the lvds, ignoring the LVDS\n");
		goto failed_find;
	}

	/* setup PWM */
	{
		u32 pwm;

		pwm = REG_READ(BLC_PWM_CTL2);
		if (pipe == 1)
			pwm |= PWM_PIPE_B;
		else
			pwm &= ~PWM_PIPE_B;
		pwm |= PWM_ENABLE;
		REG_WRITE(BLC_PWM_CTL2, pwm);
	}

out:
	drm_sysfs_connector_add(connector);

	dev_priv->int_lvds_connector = connector;
	psb_intel_panel_setup_backlight(dev);

	PSB_DEBUG_ENTRY("hdisplay = %d\n",
		 mode_dev->panel_fixed_mode->hdisplay);
	PSB_DEBUG_ENTRY(" vdisplay = %d\n",
		 mode_dev->panel_fixed_mode->vdisplay);
	PSB_DEBUG_ENTRY(" hsync_start = %d\n",
		 mode_dev->panel_fixed_mode->hsync_start);
	PSB_DEBUG_ENTRY(" hsync_end = %d\n",
		 mode_dev->panel_fixed_mode->hsync_end);
	PSB_DEBUG_ENTRY(" htotal = %d\n",
		 mode_dev->panel_fixed_mode->htotal);
	PSB_DEBUG_ENTRY(" vsync_start = %d\n",
		 mode_dev->panel_fixed_mode->vsync_start);
	PSB_DEBUG_ENTRY(" vsync_end = %d\n",
		 mode_dev->panel_fixed_mode->vsync_end);
	PSB_DEBUG_ENTRY(" vtotal = %d\n",
		 mode_dev->panel_fixed_mode->vtotal);
	PSB_DEBUG_ENTRY(" clock = %d\n",
		 mode_dev->panel_fixed_mode->clock);

	return;

failed_find:
	if (psb_intel_output->ddc_bus)
		psb_intel_i2c_destroy(psb_intel_output->ddc_bus);
failed_ddc:
	if (psb_intel_output->i2c_bus)
		psb_intel_i2c_destroy(psb_intel_output->i2c_bus);
failed_blc_i2c:
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	kfree(connector);
}
