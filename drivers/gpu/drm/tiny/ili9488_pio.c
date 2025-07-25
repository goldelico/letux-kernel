// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for Hardkernel 3.5 ISP TFT display
 *
 * Copyright 2021 Dongjin Kim <tobetter@gmail.com>
 *
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

#define ILI9488_COLUMN_ADDR	0x2a
#define ILI9488_PAGE_ADDR	0x2b
#define ILI9488_MEMORY_WRITE	0x2c
#define ILI9488_ITFCTR1         0xb0
#define ILI9488_FRMCTR1		0xb1
#define ILI9488_PWCTRL1         0xc2
#define ILI9488_VMCTRL1         0xc5
#define ILI9488_PGAMCTRL        0xe0
#define ILI9488_NGAMCTRL        0xe1
#define ILI9488_MADCTL_BGR      BIT(3)
#define ILI9488_MADCTL_MV       BIT(5)
#define ILI9488_MADCTL_MX       BIT(6)
#define ILI9488_MADCTL_MY       BIT(7)

struct ili9488_data {
	struct mipi_dbi_dev *dbidev;
	struct gpio_desc *wr;
	struct gpio_desc *cs;
	struct gpio_desc *db[8];
	void __iomem *membase;
	u32 mask;
	u32 bits[8];
	u32 bits_wr;
};

static struct ili9488_data *pdata;
static u32 *rgb;

static u32 ili9488_rgb565_to_gpiobus(struct ili9488_data *pdata, u8 color)
{
	int i;
	u32 value = 0;

	for (i = 0; i < 8; i++) {
		if (color & 1)
			value |= pdata->bits[i];
		else
			value &= ~(pdata->bits[i]);
		color >>= 1;
	}

	return value;
}

static int ili9488_bus_write(struct mipi_dbi *dbi, u8 data)
{
	int i;

	if (pdata->membase) {
		u32 v = (readl(pdata->membase) & ~pdata->mask) | *(rgb + data);
		writel(v, pdata->membase);
		writel(v | pdata->bits_wr, pdata->membase);
		return 0;
	}

	gpiod_set_value(pdata->wr, 0);
	for (i = 0; i < 8; i++) {
		gpiod_set_value(pdata->db[i], data & 1);
		data >>= 1;
	}
	gpiod_set_value(pdata->wr, 1);

	return 0;
}

static int ili9488_command(struct mipi_dbi *dbi, u8 *cmd, u8 *par, size_t num)
{
	u8 *p = par;

	gpiod_set_value(dbi->dc, 0);
	ili9488_bus_write(dbi, *cmd);
	gpiod_set_value(dbi->dc, 1);

	while (num--)
		ili9488_bus_write(dbi, *p++);

	return 0;
}

static void ili9488_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	ret = mipi_dbi_poweron_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	gpiod_set_value(pdata->cs, 0);

	mipi_dbi_command(dbi, ILI9488_ITFCTR1, 0x00);
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(250);

	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x55);
	mipi_dbi_command(dbi, ILI9488_PWCTRL1, 0x33);
	mipi_dbi_command(dbi, ILI9488_VMCTRL1, 0x00, 0x1e, 0x80, 0x00);
	mipi_dbi_command(dbi, ILI9488_FRMCTR1, 0xb0, 0x11);
	mipi_dbi_command(dbi, ILI9488_PGAMCTRL,
			0x00, 0x04, 0x0e, 0x08, 0x17, 0x0a, 0x40, 0x79,
			0x4d, 0x07, 0x0e, 0x0a, 0x1a, 0x1d, 0x0f);
	mipi_dbi_command(dbi, ILI9488_NGAMCTRL,
			0x00, 0x1b, 0x1f, 0x02, 0x10, 0x05, 0x32, 0x34,
			0x43, 0x02, 0x0a, 0x09, 0x33, 0x37, 0x0f);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);

 out_enable:
	switch (dbidev->rotation) {
		case 90:
			addr_mode = ILI9488_MADCTL_MY;
			break;
		case 180:
			addr_mode = ILI9488_MADCTL_MV;
			break;
		case 270:
			addr_mode = ILI9488_MADCTL_MX;
			break;
		default:
			addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY |
				ILI9488_MADCTL_MX;
			break;
	}

	addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);

out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	.enable = ili9488_pipe_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
	.prepare_fb = drm_gem_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode ili9488_mode = {
	DRM_SIMPLE_MODE(480, 320, 73, 49),
};

DEFINE_DRM_GEM_DMA_FOPS(ili9488_fops);

static struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.name			= "ili9488",
	.desc			= "Ilitek ILI9488",
	.major			= 1,
	.minor			= 0,
};

static int ili9488_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mipi_dbi *dbi;
	struct drm_device *drm;
	struct mipi_dbi_dev *dbidev;
	int ret;
	int i;
	u32 rotation = 0;
	struct resource res;
	char str[32];

	pdata = devm_kzalloc(dev, sizeof(struct ili9488_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	dbidev = devm_drm_dev_alloc(dev, &ili9488_driver,
			struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dbi->dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dbi->dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dbi->dc);
	}

	pdata->wr = devm_gpiod_get(dev, "wr", GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->wr)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'wr'\n");
		return PTR_ERR(pdata->wr);
	}

	pdata->cs = devm_gpiod_get(dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->cs)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'cs'\n");
		return PTR_ERR(pdata->cs);
	}

	for (i = 0; i < 8; i++) {
		struct gpio_desc *desc;
		int gpio = of_get_named_gpio(np, "db-gpios", i);
		if (gpio < 0)
			break;	/* FIXME */

		desc = gpio_to_desc(gpio);

		devm_gpio_request(dev, gpio, NULL);
		gpiod_direction_output(desc, 1);

		pdata->db[i] = desc;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (!ret) {
		pdata->membase = devm_ioremap(dev, res.start,
				resource_size(&res));
		if (!IS_ERR(pdata->membase)) {
			for (i = 0; i < 8; i++) {
				sprintf(str, "db-bits-%d", i);
				ret = of_property_read_u32(np, str,
						&pdata->bits[i]);
				if (ret)
					continue;
				pdata->mask |= pdata->bits[i];
			}

			ret = of_property_read_u32(np, "db-bits-wr",
					&pdata->bits_wr);
			if (!ret)
				pdata->mask |= pdata->bits_wr;
		}
	}

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	rgb = devm_kzalloc(dev, sizeof(u32) * 256, GFP_KERNEL);
	if (rgb) {
		u32 *p = rgb;
		for (i = 0; i < 256; i++)
			*p++ = ili9488_rgb565_to_gpiobus(pdata, i);
	}

	gpiod_set_value(pdata->wr, 1);
	gpiod_set_value(dbi->dc, 0);

	/* override the command function set in  mipi_dbi_spi_init() */
	dbi->command = ili9488_command;
	dbi->read_commands = NULL;
	dbi->swap_bytes = true;

	ret = mipi_dbi_dev_init(dbidev, &ili9488_pipe_funcs,
			&ili9488_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, pdata);
	pdata->dbidev = dbidev;

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int ili9488_remove(struct platform_device *pdev)
{
	struct ili9488_data *pdata = platform_get_drvdata(pdev);
	struct mipi_dbi_dev *dbidev = pdata->dbidev;
	struct drm_device *drm = &dbidev->drm;

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static const struct of_device_id ili9488_dt_ids[] = {
	{ .compatible = "ili9488", 0 },
	{ .compatible = "hardkernel,ili9488", 0 },
	{ },
};

MODULE_DEVICE_TABLE(of, ili9488_dt_ids);

static struct platform_driver ili9488_platform_driver = {
	.driver = {
		.name = "ili9488",
		.of_match_table = ili9488_dt_ids,
	},
	.probe = ili9488_probe,
	.remove = ili9488_remove,
};

static int __init ili9488_init(void)
{
	return platform_driver_register(&ili9488_platform_driver);
}

static void __exit ili9488_exit(void)
{
	platform_driver_unregister(&ili9488_platform_driver);
}

module_init(ili9488_init);
module_exit(ili9488_exit);

MODULE_DESCRIPTION("Ilitek ILI9488 DRM driver (8bit PIO mode)");
MODULE_AUTHOR("Dongjin Kim <tobetter@gmail.com>");
MODULE_LICENSE("GPL");
