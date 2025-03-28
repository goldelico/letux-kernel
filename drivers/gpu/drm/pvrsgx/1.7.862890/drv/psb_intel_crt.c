/*
 * Copyright © 2011 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 */

#include <linux/i2c.h>
#include <drm/drmP.h>

#include "psb_intel_bios.h"
#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"
#include <linux/pm_runtime.h>


static void psb_intel_crt_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	u32 temp, reg;
	reg = ADPA;

	temp = REG_READ(reg);
	temp &= ~(ADPA_HSYNC_CNTL_DISABLE | ADPA_VSYNC_CNTL_DISABLE);
	temp &= ~ADPA_DAC_ENABLE;

	switch(mode) {
	case DRM_MODE_DPMS_ON:
		temp |= ADPA_DAC_ENABLE;
		break;
	case DRM_MODE_DPMS_STANDBY:
		temp |= ADPA_DAC_ENABLE | ADPA_HSYNC_CNTL_DISABLE;
		break;
	case DRM_MODE_DPMS_SUSPEND:
		temp |= ADPA_DAC_ENABLE | ADPA_VSYNC_CNTL_DISABLE;
		break;
	case DRM_MODE_DPMS_OFF:
		temp |= ADPA_HSYNC_CNTL_DISABLE | ADPA_VSYNC_CNTL_DISABLE;
		break;
	}

	REG_WRITE(reg, temp);
}

static int psb_intel_crt_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	int max_clock = 0;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	/* The lowest clock for CDV is 20000KHz */
	if (mode->clock < 20000)
		return MODE_CLOCK_LOW;

	/* The max clock for CDV is 355 instead of 400 */
	max_clock = 355000;
	if (mode->clock > max_clock)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static bool psb_intel_crt_mode_fixup(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void psb_intel_crt_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{

	struct drm_device *dev = encoder->dev;
	struct drm_crtc *crtc = encoder->crtc;
	struct psb_intel_crtc *psb_intel_crtc =
					to_psb_intel_crtc(crtc);
	int dpll_md_reg;
	u32 adpa, dpll_md;
	u32 adpa_reg;

	if (psb_intel_crtc->pipe == 0)
		dpll_md_reg = DPLL_A_MD;
	else
		dpll_md_reg = DPLL_B_MD;

	adpa_reg = ADPA;

	/*
	 * Disable separate mode multiplier used when cloning SDVO to CRT
	 * XXX this needs to be adjusted when we really are cloning
	 */
	{
		dpll_md = REG_READ(dpll_md_reg);
		REG_WRITE(dpll_md_reg,
			   dpll_md & ~DPLL_MD_UDI_MULTIPLIER_MASK);
	}

	adpa = 0;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
		adpa |= ADPA_HSYNC_ACTIVE_HIGH;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
		adpa |= ADPA_VSYNC_ACTIVE_HIGH;

	if (psb_intel_crtc->pipe == 0) {
		adpa |= ADPA_PIPE_A_SELECT;
	} else {
		adpa |= ADPA_PIPE_B_SELECT;
	}

	REG_WRITE(adpa_reg, adpa);
}


/**
 * Uses CRT_HOTPLUG_EN and CRT_HOTPLUG_STAT to detect CRT presence.
 *
 * \return true if CRT is connected.
 * \return false if CRT is disconnected.
 */
static bool psb_intel_crt_detect_hotplug(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	u32 hotplug_en, orig;
	int i, tries = 0, ret = false;

	/*
	 * On 4 series desktop, CRT detect sequence need to be done twice
	 * to get a reliable result.
	 * This is also applied for CDV.
	 */

	if (IS_CDV(dev))
		tries = 2;
	else
		tries = 1;
	hotplug_en = orig = REG_READ(PORT_HOTPLUG_EN);
	hotplug_en &= ~(CRT_HOTPLUG_DETECT_MASK);
	hotplug_en |= CRT_HOTPLUG_FORCE_DETECT;

	if (IS_CDV(dev))
		hotplug_en |= CRT_HOTPLUG_ACTIVATION_PERIOD_64;

	hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;

	for (i = 0; i < tries ; i++) {
		unsigned long timeout;
		/* turn on the FORCE_DETECT */
		REG_WRITE(PORT_HOTPLUG_EN, hotplug_en);
		timeout = jiffies + msecs_to_jiffies(1000);
		/* wait for FORCE_DETECT to go off */
		do {
			if (!(REG_READ(PORT_HOTPLUG_EN) &
					CRT_HOTPLUG_FORCE_DETECT))
				break;
			msleep(1);
		} while (time_after(timeout, jiffies));
	}

	if ((REG_READ(PORT_HOTPLUG_STAT) & CRT_HOTPLUG_MONITOR_MASK) !=
	    CRT_HOTPLUG_MONITOR_NONE)
		ret = true;

	 /* clear the interrupt we just generated, if any */
	REG_WRITE(PORT_HOTPLUG_STAT, CRT_HOTPLUG_INT_STATUS);

	/* and put the bits back */
	REG_WRITE(PORT_HOTPLUG_EN, orig);

	return ret;
}

static bool psb_intel_crt_detect_ddc(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output =
				to_psb_intel_output(connector);

	/* CRT should always be at 0, but check anyway */
	if (psb_intel_output->type != INTEL_OUTPUT_ANALOG)
		return false;

	return psb_intel_ddc_probe(psb_intel_output);
}


static enum drm_connector_status psb_intel_crt_detect(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;

	if (IS_CDV(dev)) {
		if (psb_intel_crt_detect_hotplug(connector, force))
			return connector_status_connected;
		else
			return connector_status_disconnected;
	}

	if (psb_intel_crt_detect_ddc(connector))
		return connector_status_connected;

	/* If it fails in hotplug/DDC detection, return disconnected */
	return connector_status_disconnected;
}

static void psb_intel_crt_destroy(struct drm_connector *connector)
{
	struct psb_intel_output *intel_output = to_psb_intel_output(connector);

	psb_intel_i2c_destroy(intel_output->ddc_bus);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static int psb_intel_crt_get_modes(struct drm_connector *connector)
{
	int ret;
	struct psb_intel_output *intel_output =
				to_psb_intel_output(connector);


	ret = psb_intel_ddc_get_modes(intel_output);

	return ret;

}

static int psb_intel_crt_set_property(struct drm_connector *connector,
				  struct drm_property *property,
				  uint64_t value)
{
	return 0;
}

/*
 * Routines for controlling stuff on the analog port
 */

static const struct drm_encoder_helper_funcs psb_intel_crt_helper_funcs = {
	.dpms = psb_intel_crt_dpms,
	.mode_fixup = psb_intel_crt_mode_fixup,
	.prepare = psb_intel_encoder_prepare,
	.commit = psb_intel_encoder_commit,
	.mode_set = psb_intel_crt_mode_set,
};

static const struct drm_connector_funcs psb_intel_crt_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = psb_intel_crt_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = psb_intel_crt_destroy,
	.set_property = psb_intel_crt_set_property,
};

static const struct drm_connector_helper_funcs psb_intel_crt_connector_helper_funcs = {
	.mode_valid = psb_intel_crt_mode_valid,
	.get_modes = psb_intel_crt_get_modes,
	.best_encoder = psb_intel_best_encoder,
};

static void psb_intel_crt_enc_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs psb_intel_crt_enc_funcs = {
	.destroy = psb_intel_crt_enc_destroy,
};

void psb_intel_crt_init(struct drm_device *dev,
			struct psb_intel_mode_device *mode_dev)
{

	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;

	u32 i2c_reg;

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	drm_connector_init(dev, connector,
			   &psb_intel_crt_connector_funcs, DRM_MODE_CONNECTOR_VGA);

	encoder = &psb_intel_output->enc;
	drm_encoder_init(dev, encoder,
				&psb_intel_crt_enc_funcs, DRM_MODE_ENCODER_DAC);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);

	/* Set up the DDC bus. */
	i2c_reg = GPIOA;
	/* Remove the following code for CDV */
	/*
	if (dev_priv->crt_ddc_bus != 0)
		i2c_reg = dev_priv->crt_ddc_bus;
	}*/
	psb_intel_output->ddc_bus = psb_intel_i2c_create(dev,
						i2c_reg, "CRTDDC_A");
	if (!psb_intel_output->ddc_bus) {
		DRM_ERROR("DDC bus registration failed.\n");
		goto failed_ddc;
	}

	psb_intel_output->type = INTEL_OUTPUT_ANALOG;
	/*
	psb_intel_output->clone_mask = (1 << INTEL_ANALOG_CLONE_BIT);
	psb_intel_output->crtc_mask = (1 << 0) | (1 << 1);
	*/
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	drm_encoder_helper_add(encoder, &psb_intel_crt_helper_funcs);
	drm_connector_helper_add(connector, &psb_intel_crt_connector_helper_funcs);

	drm_sysfs_connector_add(connector);

	return;
failed_ddc:
	drm_encoder_cleanup(&psb_intel_output->enc);
	drm_connector_cleanup(&psb_intel_output->base);
	kfree(psb_intel_output);
	return;
}
