// SPDX-License-Identifier: GPL-2.0
/*
 * The LG35583 panel of the HiBy R1
 *
 * Copyright (c) 2026 Olivier Louvignes <olivier@mgcrea.io>
 *
 * A 480x800 parallel RGB panel, set up over a 3-wire SPI bus with 16-bit
 * words: 0x20 and a register's high byte, 0x00 and its low byte, then 0x40
 * and each data byte. HiBy calls it the LG35583; its controller is not
 * identified. The set-up sequence is the one the vendor kernel sends.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define LG35583_ADDR_HI		0x2000
#define LG35583_ADDR_LO		0x0000
#define LG35583_DATA		0x4000

#define LG35583_SLPIN		0x1000
#define LG35583_SLPOUT		0x1100
#define LG35583_DISPOFF		0x2800
#define LG35583_DISPON		0x2900
#define LG35583_RAMWR		0x2c00

struct lg35583_write {
	u16 reg;	/* 0: wait val milliseconds */
	s16 val;	/* -1: no data */
};

#define REG(r, v)	{ (r), (v) }
#define CMD(r)		{ (r), -1 }
#define DELAY(ms)	{ 0, (ms) }

static const struct lg35583_write lg35583_init[] = {
	REG(0xb100, 0x06), REG(0xb101, 0x01), REG(0xb102, 0x54),
	REG(0xb104, 0x07), REG(0xb105, 0x06), REG(0xb106, 0x06),
	REG(0xb107, 0x4a), REG(0xb108, 0x21), REG(0xb109, 0x08),
	REG(0xf101, 0xc0), REG(0xf103, 0x01),
	DELAY(5),
	REG(0xb400, 0x01), REG(0xb600, 0x10),
	REG(0x3a00, 0x77), REG(0x3b00, 0x03), REG(0x5e04, 0x02),
	REG(0x3600, 0x08),
	DELAY(5),
	/* Gamma */
	REG(0xe000, 0x00), REG(0xe001, 0x2f), REG(0xe002, 0x3f),
	REG(0xe003, 0x1f), REG(0xe004, 0x21), REG(0xe005, 0x2c),
	REG(0xe006, 0x2c), REG(0xe007, 0x61), REG(0xe008, 0x1c),
	REG(0xe009, 0x18), REG(0xe00a, 0xbc), REG(0xe00b, 0x27),
	REG(0xe00c, 0x59), REG(0xe00d, 0x6a), REG(0xe00e, 0xbb),
	REG(0xe00f, 0xbe), REG(0xe010, 0x3e), REG(0xe011, 0x48),
	REG(0xe100, 0x00), REG(0xe101, 0x32), REG(0xe102, 0x45),
	REG(0xe103, 0x26), REG(0xe104, 0x21), REG(0xe105, 0x2c),
	REG(0xe106, 0x5c), REG(0xe107, 0x63), REG(0xe108, 0x1c),
	REG(0xe109, 0x19), REG(0xe10a, 0xd3), REG(0xe10b, 0x28),
	REG(0xe10c, 0x59), REG(0xe10d, 0x6b), REG(0xe10e, 0xdb),
	REG(0xe10f, 0xde), REG(0xe110, 0x5f), REG(0xe111, 0x6b),
	REG(0xe200, 0x7f), REG(0xe201, 0x7d), REG(0xe202, 0x86),
	REG(0xe203, 0x89), REG(0xe204, 0x10), REG(0xe205, 0x1e),
	REG(0xe206, 0x41), REG(0xe207, 0x77), REG(0xe208, 0x0d),
	REG(0xe209, 0x1d), REG(0xe20a, 0xc8), REG(0xe20b, 0x30),
	REG(0xe20c, 0x5f), REG(0xe20d, 0x71), REG(0xe20e, 0xc3),
	REG(0xe20f, 0xbe), REG(0xe210, 0x3e), REG(0xe211, 0x48),
	REG(0xe300, 0x7f), REG(0xe301, 0x7f), REG(0xe302, 0x86),
	REG(0xe303, 0x93), REG(0xe304, 0x10), REG(0xe305, 0x1e),
	REG(0xe306, 0x41), REG(0xe307, 0x89), REG(0xe308, 0x0d),
	REG(0xe309, 0x1d), REG(0xe30a, 0xe0), REG(0xe30b, 0x30),
	REG(0xe30c, 0x5f), REG(0xe30d, 0x70), REG(0xe30e, 0xe4),
	REG(0xe30f, 0xdf), REG(0xe310, 0x5f), REG(0xe311, 0x6b),
	REG(0xe400, 0x00), REG(0xe401, 0x43), REG(0xe402, 0x6f),
	REG(0xe403, 0x60), REG(0xe404, 0x21), REG(0xe405, 0x2c),
	REG(0xe406, 0x5a), REG(0xe407, 0x6d), REG(0xe408, 0x1c),
	REG(0xe409, 0x18), REG(0xe40a, 0xc6), REG(0xe40b, 0x27),
	REG(0xe40c, 0x59), REG(0xe40d, 0x6a), REG(0xe40e, 0xbb),
	REG(0xe40f, 0xbe), REG(0xe410, 0x3e), REG(0xe411, 0x48),
	REG(0xe500, 0x00), REG(0xe501, 0x46), REG(0xe502, 0x75),
	REG(0xe503, 0x67), REG(0xe504, 0x21), REG(0xe505, 0x2c),
	REG(0xe506, 0x5a), REG(0xe507, 0x7e), REG(0xe508, 0x1c),
	REG(0xe509, 0x19), REG(0xe50a, 0xdd), REG(0xe50b, 0x28),
	REG(0xe50c, 0x59), REG(0xe50d, 0x6b), REG(0xe50e, 0xdb),
	REG(0xe50f, 0xde), REG(0xe510, 0x5f), REG(0xe511, 0x6b),
	DELAY(5),
	REG(0xc200, 0x70), REG(0xc202, 0x70), REG(0xc203, 0x44),
	DELAY(20),
	REG(0xc200, 0x51), REG(0xc201, 0x06), REG(0xc202, 0x51),
	REG(0xc203, 0x66),
	DELAY(20),
	REG(0xc100, 0x45),
	DELAY(5),
	REG(0xc700, 0x46), REG(0xc000, 0xa3), REG(0xc002, 0x8a),
	DELAY(5),
	CMD(LG35583_SLPOUT),
	DELAY(130),
};

struct lg35583 {
	struct drm_panel panel;
	struct spi_device *spi;
	struct gpio_desc *reset;
	struct regulator *vcc;
	struct regulator *vccio;
};

static const struct drm_display_mode lg35583_mode = {
	.clock = 28000,
	.hdisplay = 480,
	.hsync_start = 480 + 24,
	.hsync_end = 480 + 24 + 24,
	.htotal = 480 + 24 + 24 + 24,
	.vdisplay = 800,
	.vsync_start = 800 + 14,
	.vsync_end = 800 + 14 + 5,
	.vtotal = 800 + 14 + 5 + 8,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static struct lg35583 *to_lg35583(struct drm_panel *panel)
{
	return container_of(panel, struct lg35583, panel);
}

static int lg35583_word(struct lg35583 *ctx, u16 word)
{
	/* Copied to a DMA-safe buffer. */
	return spi_write_then_read(ctx->spi, &word, sizeof(word), NULL, 0);
}

static int lg35583_cmd(struct lg35583 *ctx, u16 reg)
{
	int ret;

	ret = lg35583_word(ctx, LG35583_ADDR_HI | (reg >> 8));
	if (ret)
		return ret;
	return lg35583_word(ctx, LG35583_ADDR_LO | (reg & 0xff));
}

static int lg35583_write(struct lg35583 *ctx, const struct lg35583_write *w)
{
	int ret;

	if (!w->reg) {
		msleep(w->val);
		return 0;
	}

	ret = lg35583_cmd(ctx, w->reg);
	if (ret || w->val < 0)
		return ret;
	return lg35583_word(ctx, LG35583_DATA | w->val);
}

static int lg35583_prepare(struct drm_panel *panel)
{
	struct lg35583 *ctx = to_lg35583(panel);
	unsigned int i;
	int ret;

	ret = regulator_enable(ctx->vccio);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	ret = regulator_enable(ctx->vcc);
	if (ret)
		goto err_vccio;
	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(50);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(30);

	for (i = 0; i < ARRAY_SIZE(lg35583_init); i++) {
		ret = lg35583_write(ctx, &lg35583_init[i]);
		if (ret) {
			dev_err(panel->dev, "Unable to set up the panel: %d\n", ret);
			goto err_vcc;
		}
	}

	return 0;

err_vcc:
	gpiod_set_value_cansleep(ctx->reset, 1);
	regulator_disable(ctx->vcc);
err_vccio:
	regulator_disable(ctx->vccio);
	return ret;
}

static int lg35583_enable(struct drm_panel *panel)
{
	struct lg35583 *ctx = to_lg35583(panel);
	int ret;

	ret = lg35583_cmd(ctx, LG35583_DISPON);
	if (ret)
		return ret;

	return lg35583_cmd(ctx, LG35583_RAMWR);
}

static int lg35583_disable(struct drm_panel *panel)
{
	struct lg35583 *ctx = to_lg35583(panel);
	int ret;

	ret = lg35583_cmd(ctx, LG35583_DISPOFF);
	usleep_range(5000, 6000);
	return ret;
}

static int lg35583_unprepare(struct drm_panel *panel)
{
	struct lg35583 *ctx = to_lg35583(panel);

	lg35583_cmd(ctx, LG35583_SLPIN);
	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(ctx->reset, 1);
	regulator_disable(ctx->vccio);
	regulator_disable(ctx->vcc);
	return 0;
}

static int lg35583_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	/* Red and blue are swapped on the R1 without it. */
	static const u32 bus_format = MEDIA_BUS_FMT_BGR666_1X18;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &lg35583_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.bpc = 6;
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE;
	drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);

	return 1;
}

static const struct drm_panel_funcs lg35583_funcs = {
	.prepare = lg35583_prepare,
	.enable = lg35583_enable,
	.disable = lg35583_disable,
	.unprepare = lg35583_unprepare,
	.get_modes = lg35583_get_modes,
};

static int lg35583_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct lg35583 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct lg35583, panel, &lg35583_funcs,
				   DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->spi = spi;
	spi_set_drvdata(spi, ctx);

	spi->bits_per_word = 16;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to set up SPI\n");

	ctx->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(ctx->vcc))
		return dev_err_probe(dev, PTR_ERR(ctx->vcc), "Unable to get vcc\n");

	ctx->vccio = devm_regulator_get(dev, "vccio");
	if (IS_ERR(ctx->vccio))
		return dev_err_probe(dev, PTR_ERR(ctx->vccio), "Unable to get vccio\n");

	/*
	 * Deasserted, which keeps a panel the boot loader left on as it is, but
	 * as an output: after a reset the pin is an input, and prepare's reset
	 * pulse would never reach the panel.
	 */
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "Unable to get reset\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);
	return 0;
}

static void lg35583_remove(struct spi_device *spi)
{
	struct lg35583 *ctx = spi_get_drvdata(spi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id lg35583_of_match[] = {
	{ .compatible = "hiby,lg35583" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg35583_of_match);

static const struct spi_device_id lg35583_ids[] = {
	{ "lg35583" },
	{ }
};
MODULE_DEVICE_TABLE(spi, lg35583_ids);

static struct spi_driver lg35583_driver = {
	.probe = lg35583_probe,
	.remove = lg35583_remove,
	.id_table = lg35583_ids,
	.driver = {
		.name = "panel-hiby-lg35583",
		.of_match_table = lg35583_of_match,
	},
};
module_spi_driver(lg35583_driver);

MODULE_AUTHOR("Olivier Louvignes <olivier@mgcrea.io>");
MODULE_DESCRIPTION("HiBy R1 LG35583 panel driver");
MODULE_LICENSE("GPL");
