// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2019, 2020 Paul Boddie <paul@boddie.org.uk>
 *
 * Derived from dw_hdmi-imx.c with i.MX portions removed.
 * Probe and remove operations derived from rcar_dw_hdmi.c.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/bridge/dw_hdmi.h>
#include <drm/drm_of.h>

static const struct dw_hdmi_mpll_config jz4780_mpll_cfg[] = {
	{ 45250000,  { { 0x01e0, 0x0000 }, { 0x21e1, 0x0000 }, { 0x41e2, 0x0000 } } },
	{ 92500000,  { { 0x0140, 0x0005 }, { 0x2141, 0x0005 }, { 0x4142, 0x0005 } } },
	{ 148500000, { { 0x00a0, 0x000a }, { 0x20a1, 0x000a }, { 0x40a2, 0x000a } } },
	{ 216000000, { { 0x00a0, 0x000a }, { 0x2001, 0x000f }, { 0x4002, 0x000f } } },
	{ ~0UL,      { { 0x0000, 0x0000 }, { 0x0000, 0x0000 }, { 0x0000, 0x0000 } } }
};

static const struct dw_hdmi_curr_ctrl jz4780_cur_ctr[] = {
	/*pixelclk     bpp8    bpp10   bpp12 */
	{ 54000000,  { 0x091c, 0x091c, 0x06dc } },
	{ 58400000,  { 0x091c, 0x06dc, 0x06dc } },
	{ 72000000,  { 0x06dc, 0x06dc, 0x091c } },
	{ 74250000,  { 0x06dc, 0x0b5c, 0x091c } },
	{ 118800000, { 0x091c, 0x091c, 0x06dc } },
	{ 216000000, { 0x06dc, 0x0b5c, 0x091c } },
	{ ~0UL,      { 0x0000, 0x0000, 0x0000 } },
};

/*
 * Resistance term 133Ohm Cfg
 * PREEMP config 0.00
 * TX/CK level 10
 */
static const struct dw_hdmi_phy_config jz4780_phy_config[] = {
	/*pixelclk   symbol   term   vlev */
	{ 216000000, 0x800d, 0x0005, 0x01ad},
	{ ~0UL,      0x0000, 0x0000, 0x0000}
};

static enum drm_mode_status
jz4780_dw_hdmi_mode_valid(struct dw_hdmi *hdmi, void *data,
			  const struct drm_display_info *info,
			  const struct drm_display_mode *mode)
{
	if (mode->clock < 13500)
		return MODE_CLOCK_LOW;
	/* FIXME: Hardware is capable of 270MHz, but setup data is missing. */
	if (mode->clock > 216000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static bool
jz4780_dw_hdmi_mode_fixup(struct drm_bridge *bridge,
                           const struct drm_display_mode *mode,
                           struct drm_display_mode *adjusted_mode)
{
	adjusted_mode->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	adjusted_mode->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

	return true;
}

static const struct drm_bridge_timings jz4780_dw_hdmi_timings = {
	.input_bus_flags = DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE,
};

static struct dw_hdmi_plat_data jz4780_dw_hdmi_plat_data = {
	.mpll_cfg   = jz4780_mpll_cfg,
	.cur_ctr    = jz4780_cur_ctr,
	.phy_config = jz4780_phy_config,
	.mode_valid = jz4780_dw_hdmi_mode_valid,
	.mode_fixup = jz4780_dw_hdmi_mode_fixup,
	.timings    = &jz4780_dw_hdmi_timings,
};

static const struct of_device_id jz4780_dw_hdmi_dt_ids[] = {
	{ .compatible = "ingenic,jz4780-dw-hdmi" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, jz4780_dw_hdmi_dt_ids);

static int jz4780_dw_hdmi_probe(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi;

	hdmi = dw_hdmi_probe(pdev, &jz4780_dw_hdmi_plat_data);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	platform_set_drvdata(pdev, hdmi);

	return 0;
}

static int jz4780_dw_hdmi_remove(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi = platform_get_drvdata(pdev);

	dw_hdmi_remove(hdmi);

	return 0;
}

static struct platform_driver jz4780_dw_hdmi_platform_driver = {
	.probe  = jz4780_dw_hdmi_probe,
	.remove = jz4780_dw_hdmi_remove,
	.driver = {
		.name = "dw-hdmi-jz4780",
		.of_match_table = jz4780_dw_hdmi_dt_ids,
	},
};

module_platform_driver(jz4780_dw_hdmi_platform_driver);

MODULE_AUTHOR("Andy Yan <andy.yan@rock-chips.com>");
MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_AUTHOR("Paul Boddie <paul@boddie.org.uk>");
MODULE_DESCRIPTION("Ingenic JZ4780 DW-HDMI Driver Extension");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-hdmi-jz4780");
