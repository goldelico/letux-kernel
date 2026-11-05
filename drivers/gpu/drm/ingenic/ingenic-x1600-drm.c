// SPDX-License-Identifier: GPL-2.0
/*
 * DRM driver for the LCD controller of the Ingenic X1600
 *
 * Copyright (c) 2026 Olivier Louvignes <olivier@mgcrea.io>
 *
 * The X1600's display controller (X1600 programming manual, chapter 10) is
 * the simple read channel of the X2000's, without its composer. It shares no
 * registers with the JZ47xx and X1000 controllers that ingenic-drm drives: it
 * fetches each frame through a descriptor whose address is in SRD_CHAIN_ADDR,
 * and its interface and timings sit in register blocks of their own. It
 * drives parallel RGB (TFT) panels and smart panels; this driver handles
 * parallel RGB panels on a 24-bit bus, from RGB565 framebuffers.
 */

#include <linux/aperture.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#define X1600_LCD_SRD_CHAIN_ADDR	0x1000
#define X1600_LCD_SRD_CHAIN_START	0x1004
#define X1600_LCD_CTRL			0x2000
#define X1600_LCD_ST			0x2004
#define X1600_LCD_CLR_ST		0x2008
#define X1600_LCD_INTC			0x200c
#define X1600_LCD_INT_FLAG		0x2010
#define X1600_LCD_COM_CONFIG		0x2014
#define X1600_LCD_CHAIN_SITE		0x2204
#define X1600_LCD_DISP_COM		0x8000
#define X1600_LCD_TFT_HSYNC		0x9000
#define X1600_LCD_TFT_VSYNC		0x9004
#define X1600_LCD_TFT_HDE		0x9008
#define X1600_LCD_TFT_VDE		0x900c
#define X1600_LCD_TFT_TRAN_CFG		0x9010

/* CTRL: stop the simple read channel at the end of the frame. */
#define X1600_LCD_CTRL_GEN_STP_SRD	BIT(4)

/* ST; CLR_ST (write 1 to clear), INTC and INT_FLAG have the same bits. */
#define X1600_LCD_ST_WORKING		BIT(0)
#define X1600_LCD_ST_STOP_SRD_ACK	BIT(7)
#define X1600_LCD_ST_TFT_UNDR		BIT(8)
#define X1600_LCD_ST_DISP_END		BIT(17)	/* a frame was displayed */

#define X1600_LCD_COM_CONFIG_BURST_LEN	GENMASK(5, 4)
#define X1600_LCD_BURST_LEN_32		3

#define X1600_LCD_DISP_COM_IF		GENMASK(1, 0)
#define X1600_LCD_DISP_COM_IF_TFT	1
#define X1600_LCD_DISP_COM_DITHER_EN	BIT(4)
#define X1600_LCD_DISP_COM_TFT_CLKGATE	BIT(5)
#define X1600_LCD_DISP_COM_SLCD_CLKGATE	BIT(6)

/* TFT_HSYNC, TFT_VSYNC: sync length and total; TFT_HDE, TFT_VDE: active area. */
#define X1600_LCD_TIMING_START		GENMASK(27, 16)
#define X1600_LCD_TIMING_END		GENMASK(11, 0)

#define X1600_LCD_TFT_COLOR_EVEN	GENMASK(21, 19)
#define X1600_LCD_TFT_COLOR_ODD		GENMASK(18, 16)
#define X1600_LCD_TFT_PIX_CLK_INV	BIT(10)
#define X1600_LCD_TFT_DE_DL		BIT(9)
#define X1600_LCD_TFT_HSYNC_DL		BIT(8)
#define X1600_LCD_TFT_VSYNC_DL		BIT(7)
#define X1600_LCD_TFT_MODE		GENMASK(2, 0)
#define X1600_LCD_TFT_MODE_24BIT	0
#define X1600_LCD_TFT_COLOR_RGB		0
#define X1600_LCD_TFT_COLOR_BGR		2

#define X1600_LCD_DESC_CFG_FMT		GENMASK(22, 19)
#define X1600_LCD_DESC_FMT_RGB565	2
/* An interrupt at the end of the frame: DISP_END needs this and INTC set. */
#define X1600_LCD_DESC_CTRL_EOD_MSK	BIT(17)

#define X1600_LCD_MAX_PIXELS		(1280 * 720)
/* A smaller frame breaks a continuous descriptor chain (manual, 10.7). */
#define X1600_LCD_MIN_PIXELS		500
#define X1600_LCD_STOP_TIMEOUT_MS	100

/*
 * The controller reads a descriptor at the start of each frame and then
 * follows its next field. Each descriptor points at itself, so a frame
 * repeats until SRD_CHAIN_ADDR names another one.
 */
struct x1600_lcd_desc {
	u32 next;
	u32 addr;
	u32 stride;	/* in pixels */
	u32 cfg;
	u32 ctrl;
} __aligned(64);

struct x1600_lcd {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;

	void __iomem *base;
	struct clk *clk;

	struct x1600_lcd_desc *desc;
	dma_addr_t desc_phys;
	unsigned int cur;		/* the descriptor last handed over */
	bool running;
	struct completion stopped;
	/* Serialises the read-modify-writes of INTC. */
	spinlock_t irq_lock;

	/* Under drm.event_lock: sent once the controller reads event_desc. */
	struct drm_pending_vblank_event *event;
	u32 event_desc;

	u32 bus_flags;
	u32 color_order;
};

static const u32 x1600_lcd_formats[] = {
	DRM_FORMAT_RGB565,
};

static struct x1600_lcd *to_x1600_lcd(struct drm_device *drm)
{
	return container_of(drm, struct x1600_lcd, drm);
}

static dma_addr_t x1600_lcd_desc_addr(struct x1600_lcd *lcd, unsigned int i)
{
	return lcd->desc_phys + i * sizeof(*lcd->desc);
}

static void x1600_lcd_irq_enable(struct x1600_lcd *lcd, u32 mask, bool enable)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&lcd->irq_lock, flags);
	val = readl(lcd->base + X1600_LCD_INTC);
	if (enable)
		val |= mask;
	else
		val &= ~mask;
	/* Seen on the R1: a bit still set when it is masked keeps the line up. */
	writel(mask, lcd->base + X1600_LCD_CLR_ST);
	writel(val, lcd->base + X1600_LCD_INTC);
	spin_unlock_irqrestore(&lcd->irq_lock, flags);
}

/* Stops at the end of the frame; false if the controller did not report it. */
static bool x1600_lcd_stop(struct x1600_lcd *lcd)
{
	unsigned long left;

	reinit_completion(&lcd->stopped);
	x1600_lcd_irq_enable(lcd, X1600_LCD_ST_STOP_SRD_ACK, true);
	writel(X1600_LCD_CTRL_GEN_STP_SRD, lcd->base + X1600_LCD_CTRL);
	left = wait_for_completion_timeout(&lcd->stopped,
					   msecs_to_jiffies(X1600_LCD_STOP_TIMEOUT_MS));
	x1600_lcd_irq_enable(lcd, X1600_LCD_ST_STOP_SRD_ACK, false);
	return left;
}

static u32 x1600_lcd_timing(unsigned int start, unsigned int end)
{
	return FIELD_PREP(X1600_LCD_TIMING_START, start) |
	       FIELD_PREP(X1600_LCD_TIMING_END, end);
}

/* Sends the commit's event once the controller has read the new descriptor. */
static void x1600_lcd_queue_event(struct x1600_lcd *lcd, struct drm_crtc_state *crtc_state)
{
	struct drm_pending_vblank_event *event = crtc_state->event;
	struct drm_crtc *crtc = &lcd->crtc;

	if (!event)
		return;

	crtc_state->event = NULL;

	/*
	 * The controller takes a new descriptor "at appropriate time" (manual,
	 * 10.6.4): wait until it reports reading the one just handed over.
	 */
	spin_lock_irq(&crtc->dev->event_lock);
	if (lcd->running && !WARN_ON(lcd->event) && drm_crtc_vblank_get(crtc) == 0) {
		lcd->event = event;
		lcd->event_desc = x1600_lcd_desc_addr(lcd, lcd->cur);
	} else {
		drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void x1600_lcd_crtc_atomic_enable(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct x1600_lcd *lcd = to_x1600_lcd(crtc->dev);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	unsigned int hde = mode->htotal - mode->hsync_start;
	unsigned int vde = mode->vtotal - mode->vsync_start;
	u32 val;
	int ret;

	ret = clk_set_rate(lcd->clk, mode->crtc_clock * 1000);
	if (ret)
		dev_warn(lcd->drm.dev, "Unable to set the pixel clock: %d\n", ret);
	ret = clk_prepare_enable(lcd->clk);
	if (ret) {
		dev_err(lcd->drm.dev, "Unable to start the pixel clock: %d\n", ret);
		x1600_lcd_queue_event(lcd, crtc_state);
		return;
	}

	writel(readl(lcd->base + X1600_LCD_ST), lcd->base + X1600_LCD_CLR_ST);
	val = readl(lcd->base + X1600_LCD_COM_CONFIG) & ~X1600_LCD_COM_CONFIG_BURST_LEN;
	writel(val | FIELD_PREP(X1600_LCD_COM_CONFIG_BURST_LEN, X1600_LCD_BURST_LEN_32),
	       lcd->base + X1600_LCD_COM_CONFIG);

	/* In the manual's order: timings, output format, then the interface. */
	writel(x1600_lcd_timing(mode->hsync_end - mode->hsync_start, mode->htotal),
	       lcd->base + X1600_LCD_TFT_HSYNC);
	writel(x1600_lcd_timing(mode->vsync_end - mode->vsync_start, mode->vtotal),
	       lcd->base + X1600_LCD_TFT_VSYNC);
	writel(x1600_lcd_timing(hde, hde + mode->hdisplay),
	       lcd->base + X1600_LCD_TFT_HDE);
	writel(x1600_lcd_timing(vde, vde + mode->vdisplay),
	       lcd->base + X1600_LCD_TFT_VDE);

	val = FIELD_PREP(X1600_LCD_TFT_COLOR_EVEN, lcd->color_order) |
	      FIELD_PREP(X1600_LCD_TFT_COLOR_ODD, lcd->color_order) |
	      FIELD_PREP(X1600_LCD_TFT_MODE, X1600_LCD_TFT_MODE_24BIT);
	if (lcd->bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		val |= X1600_LCD_TFT_PIX_CLK_INV;
	if (lcd->bus_flags & DRM_BUS_FLAG_DE_LOW)
		val |= X1600_LCD_TFT_DE_DL;
	/* The *_DL bits set the level outside the pulse. */
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		val |= X1600_LCD_TFT_HSYNC_DL;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		val |= X1600_LCD_TFT_VSYNC_DL;
	writel(val, lcd->base + X1600_LCD_TFT_TRAN_CFG);

	/* No dithering: it only applies to 888 sources. */
	val = readl(lcd->base + X1600_LCD_DISP_COM);
	val &= ~(X1600_LCD_DISP_COM_IF | X1600_LCD_DISP_COM_DITHER_EN);
	val |= FIELD_PREP(X1600_LCD_DISP_COM_IF, X1600_LCD_DISP_COM_IF_TFT) |
	       X1600_LCD_DISP_COM_TFT_CLKGATE | X1600_LCD_DISP_COM_SLCD_CLKGATE;
	writel(val, lcd->base + X1600_LCD_DISP_COM);

	writel(x1600_lcd_desc_addr(lcd, lcd->cur), lcd->base + X1600_LCD_SRD_CHAIN_ADDR);
	writel(1, lcd->base + X1600_LCD_SRD_CHAIN_START);
	lcd->running = true;

	drm_crtc_vblank_on(crtc);
	x1600_lcd_queue_event(lcd, crtc_state);
}

static void x1600_lcd_crtc_atomic_disable(struct drm_crtc *crtc,
					  struct drm_atomic_commit *state)
{
	struct x1600_lcd *lcd = to_x1600_lcd(crtc->dev);

	/* atomic_enable() could not start the clock: nothing runs. */
	if (!lcd->running)
		return;

	if (!x1600_lcd_stop(lcd))
		dev_warn(lcd->drm.dev, "The controller did not stop\n");
	lcd->running = false;

	spin_lock_irq(&crtc->dev->event_lock);
	if (lcd->event) {
		drm_crtc_send_vblank_event(crtc, lcd->event);
		drm_crtc_vblank_put(crtc);
		lcd->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_crtc_vblank_off(crtc);
	clk_disable_unprepare(lcd->clk);
}

static int x1600_lcd_crtc_atomic_check(struct drm_crtc *crtc,
				       struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* Only when enabled: drm_atomic_helper_shutdown() disables both at once. */
	if (!crtc_state->enable)
		return 0;

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void x1600_lcd_crtc_atomic_flush(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* The planes come first: a CRTC being enabled sends it once started. */
	if (crtc_state->active && drm_atomic_crtc_needs_modeset(crtc_state))
		return;

	x1600_lcd_queue_event(to_x1600_lcd(crtc->dev), crtc_state);
}

static enum drm_mode_status
x1600_lcd_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	unsigned int pixels = mode->hdisplay * mode->vdisplay;

	if (mode->htotal > FIELD_MAX(X1600_LCD_TIMING_END) ||
	    mode->vtotal > FIELD_MAX(X1600_LCD_TIMING_END))
		return MODE_BAD;

	/* The sync pulses' end points must not be 0. */
	if (mode->hsync_end == mode->hsync_start)
		return MODE_HSYNC_NARROW;
	if (mode->vsync_end == mode->vsync_start)
		return MODE_VSYNC_NARROW;

	if (pixels > X1600_LCD_MAX_PIXELS)
		return MODE_MEM;
	if (pixels < X1600_LCD_MIN_PIXELS)
		return MODE_BAD;

	return MODE_OK;
}

static int x1600_lcd_enable_vblank(struct drm_crtc *crtc)
{
	struct x1600_lcd *lcd = to_x1600_lcd(crtc->dev);

	x1600_lcd_irq_enable(lcd, X1600_LCD_ST_DISP_END, true);
	return 0;
}

static void x1600_lcd_disable_vblank(struct drm_crtc *crtc)
{
	struct x1600_lcd *lcd = to_x1600_lcd(crtc->dev);

	x1600_lcd_irq_enable(lcd, X1600_LCD_ST_DISP_END, false);
}

static irqreturn_t x1600_lcd_irq(int irq, void *data)
{
	struct x1600_lcd *lcd = data;
	u32 st = readl(lcd->base + X1600_LCD_INT_FLAG);

	if (!st)
		return IRQ_NONE;

	writel(st, lcd->base + X1600_LCD_CLR_ST);
	if (st & X1600_LCD_ST_STOP_SRD_ACK)
		complete(&lcd->stopped);

	if (st & X1600_LCD_ST_DISP_END) {
		drm_crtc_handle_vblank(&lcd->crtc);

		spin_lock(&lcd->drm.event_lock);
		if (lcd->event &&
		    readl(lcd->base + X1600_LCD_CHAIN_SITE) == lcd->event_desc) {
			drm_crtc_send_vblank_event(&lcd->crtc, lcd->event);
			drm_crtc_vblank_put(&lcd->crtc);
			lcd->event = NULL;
		}
		spin_unlock(&lcd->drm.event_lock);
	}

	return IRQ_HANDLED;
}

static int x1600_lcd_plane_atomic_check(struct drm_plane *plane,
					struct drm_atomic_commit *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	int ret;

	if (!plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, plane_state->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	/* The plane covers the whole screen: no scaling, no position. */
	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret)
		return ret;

	/* The controller scans out a whole frame from the framebuffer. */
	if (crtc_state->enable && !plane_state->visible)
		return -EINVAL;

	return 0;
}

static void x1600_lcd_plane_atomic_update(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct x1600_lcd *lcd = to_x1600_lcd(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = plane_state->fb;
	unsigned int next = lcd->running ? !lcd->cur : lcd->cur;
	struct x1600_lcd_desc *desc = &lcd->desc[next];

	if (!fb)
		return;

	desc->next = x1600_lcd_desc_addr(lcd, next);
	desc->addr = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	desc->stride = fb->pitches[0] / fb->format->cpp[0];
	desc->cfg = FIELD_PREP(X1600_LCD_DESC_CFG_FMT, X1600_LCD_DESC_FMT_RGB565);
	desc->ctrl = X1600_LCD_DESC_CTRL_EOD_MSK;

	if (!lcd->running)
		return;

	/*
	 * The manual hands over a descriptor by writing SRD_CHAIN_ADDR; the
	 * vendor kernel also sets SRD_CHAIN_START, and so does this.
	 */
	wmb();
	writel(x1600_lcd_desc_addr(lcd, next), lcd->base + X1600_LCD_SRD_CHAIN_ADDR);
	writel(1, lcd->base + X1600_LCD_SRD_CHAIN_START);
	lcd->cur = next;
}

/*
 * Panels with 18 data lines take the top six bits of each colour of the
 * 888 mode: in the 666 mode, the HiBy R1's panel shows wrong colours.
 */
static int x1600_lcd_color_order(const struct drm_display_info *info)
{
	if (!info->num_bus_formats)
		return X1600_LCD_TFT_COLOR_RGB;

	switch (info->bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB666_1X18:
	case MEDIA_BUS_FMT_RGB888_1X24:
		return X1600_LCD_TFT_COLOR_RGB;
	case MEDIA_BUS_FMT_BGR666_1X18:
	case MEDIA_BUS_FMT_BGR888_1X24:
		return X1600_LCD_TFT_COLOR_BGR;
	default:
		return -EINVAL;
	}
}

static int x1600_lcd_encoder_atomic_check(struct drm_encoder *encoder,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state)
{
	const struct drm_display_info *info = &conn_state->connector->display_info;

	if (x1600_lcd_color_order(info) < 0) {
		drm_dbg_kms(encoder->dev, "Unsupported bus format 0x%04x\n",
			    info->bus_formats[0]);
		return -EINVAL;
	}

	return 0;
}

static void x1600_lcd_encoder_atomic_mode_set(struct drm_encoder *encoder,
					      struct drm_crtc_state *crtc_state,
					      struct drm_connector_state *conn_state)
{
	struct x1600_lcd *lcd = to_x1600_lcd(encoder->dev);
	const struct drm_display_info *info = &conn_state->connector->display_info;

	lcd->bus_flags = info->bus_flags;
	lcd->color_order = x1600_lcd_color_order(info);
}

static const struct drm_plane_funcs x1600_lcd_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.atomic_create_state	= drm_atomic_helper_plane_create_state,
	.destroy		= drm_plane_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const struct drm_plane_helper_funcs x1600_lcd_plane_helper_funcs = {
	.atomic_check		= x1600_lcd_plane_atomic_check,
	.atomic_update		= x1600_lcd_plane_atomic_update,
};

static const struct drm_crtc_funcs x1600_lcd_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_create_state	= drm_atomic_helper_crtc_create_state,
	.destroy		= drm_crtc_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank		= x1600_lcd_enable_vblank,
	.disable_vblank		= x1600_lcd_disable_vblank,
};

static const struct drm_crtc_helper_funcs x1600_lcd_crtc_helper_funcs = {
	.atomic_check		= x1600_lcd_crtc_atomic_check,
	.atomic_enable		= x1600_lcd_crtc_atomic_enable,
	.atomic_disable		= x1600_lcd_crtc_atomic_disable,
	.atomic_flush		= x1600_lcd_crtc_atomic_flush,
	.mode_valid		= x1600_lcd_crtc_mode_valid,
};

static const struct drm_encoder_helper_funcs x1600_lcd_encoder_helper_funcs = {
	.atomic_check		= x1600_lcd_encoder_atomic_check,
	.atomic_mode_set	= x1600_lcd_encoder_atomic_mode_set,
};

/*
 * drm_atomic_helper_commit_tail(), waiting for the flip instead of a vblank:
 * the old framebuffer stays in use until the controller reads the new
 * descriptor, which can be a frame later.
 */
static void x1600_lcd_commit_tail(struct drm_atomic_commit *state)
{
	struct drm_device *drm = state->dev;

	drm_atomic_helper_commit_modeset_disables(drm, state);
	drm_atomic_helper_commit_planes(drm, state, 0);
	drm_atomic_helper_commit_modeset_enables(drm, state);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_flip_done(drm, state);
	drm_atomic_helper_cleanup_planes(drm, state);
}

static const struct drm_mode_config_funcs x1600_lcd_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs x1600_lcd_mode_config_helpers = {
	.atomic_commit_tail	= x1600_lcd_commit_tail,
};

DEFINE_DRM_GEM_DMA_FOPS(x1600_lcd_fops);

static const struct drm_driver x1600_lcd_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name			= "ingenic-x1600-drm",
	.desc			= "DRM driver for the Ingenic X1600 LCD controller",
	.major			= 1,
	.minor			= 0,
	.fops			= &x1600_lcd_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int x1600_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_connector *connector;
	struct drm_bridge *bridge;
	struct drm_device *drm;
	struct x1600_lcd *lcd;
	int irq, ret;

	lcd = devm_drm_dev_alloc(dev, &x1600_lcd_driver, struct x1600_lcd, drm);
	if (IS_ERR(lcd))
		return PTR_ERR(lcd);

	drm = &lcd->drm;
	platform_set_drvdata(pdev, lcd);
	init_completion(&lcd->stopped);
	spin_lock_init(&lcd->irq_lock);

	lcd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcd->base))
		return PTR_ERR(lcd->base);

	lcd->clk = devm_clk_get(dev, "lcd");
	if (IS_ERR(lcd->clk))
		return dev_err_probe(dev, PTR_ERR(lcd->clk), "Unable to get clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "Unable to get the panel\n");

	lcd->desc = dmam_alloc_coherent(dev, 2 * sizeof(*lcd->desc),
					&lcd->desc_phys, GFP_KERNEL);
	if (!lcd->desc)
		return -ENOMEM;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	/* Framebuffers, not modes: fbdev's double buffer is taller than 720p. */
	drm->mode_config.max_width = FIELD_MAX(X1600_LCD_TIMING_END);
	drm->mode_config.max_height = FIELD_MAX(X1600_LCD_TIMING_END);
	drm->mode_config.funcs = &x1600_lcd_mode_config_funcs;
	drm->mode_config.helper_private = &x1600_lcd_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &lcd->plane, 1, &x1600_lcd_plane_funcs,
				       x1600_lcd_formats, ARRAY_SIZE(x1600_lcd_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&lcd->plane, &x1600_lcd_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &lcd->crtc, &lcd->plane, NULL,
					&x1600_lcd_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&lcd->crtc, &x1600_lcd_crtc_helper_funcs);

	ret = drmm_encoder_init(drm, &lcd->encoder, NULL, DRM_MODE_ENCODER_DPI, NULL);
	if (ret)
		return ret;
	lcd->encoder.possible_crtcs = drm_crtc_mask(&lcd->crtc);
	drm_encoder_helper_add(&lcd->encoder, &x1600_lcd_encoder_helper_funcs);

	ret = drm_bridge_attach(&lcd->encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, &lcd->encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);
	drm_connector_attach_encoder(connector, &lcd->encoder);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	/* Take over from a boot framebuffer, and stop what it left running. */
	ret = aperture_remove_all_conflicting_devices(x1600_lcd_driver.name);
	if (ret)
		return ret;

	ret = clk_prepare_enable(lcd->clk);
	if (ret)
		return ret;

	/* No interrupt from what the boot loader left enabled or pending. */
	writel(0, lcd->base + X1600_LCD_INTC);
	writel(readl(lcd->base + X1600_LCD_ST), lcd->base + X1600_LCD_CLR_ST);

	ret = devm_request_irq(dev, irq, x1600_lcd_irq, 0, dev_name(dev), lcd);
	if (ret) {
		clk_disable_unprepare(lcd->clk);
		return dev_err_probe(dev, ret, "Unable to request the IRQ\n");
	}

	/*
	 * Seen on the R1: after an underrun (TFT_UNDR) the controller can stop
	 * answering GEN_STP_SRD, and only a reset brings it back.
	 */
	if (readl(lcd->base + X1600_LCD_ST) & X1600_LCD_ST_WORKING && !x1600_lcd_stop(lcd))
		dev_err(dev, "The controller did not stop%s\n",
			readl(lcd->base + X1600_LCD_ST) & X1600_LCD_ST_TFT_UNDR ?
			" after an underrun" : "");
	clk_disable_unprepare(lcd->clk);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB565);
	return 0;
}

static void x1600_lcd_remove(struct platform_device *pdev)
{
	struct x1600_lcd *lcd = platform_get_drvdata(pdev);

	drm_dev_unregister(&lcd->drm);
	drm_atomic_helper_shutdown(&lcd->drm);
}

static void x1600_lcd_shutdown(struct platform_device *pdev)
{
	struct x1600_lcd *lcd = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&lcd->drm);
}

static const struct of_device_id x1600_lcd_of_match[] = {
	{ .compatible = "ingenic,x1600-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, x1600_lcd_of_match);

static struct platform_driver x1600_lcd_platform_driver = {
	.driver = {
		.name = "ingenic-x1600-drm",
		.of_match_table = x1600_lcd_of_match,
	},
	.probe = x1600_lcd_probe,
	.remove = x1600_lcd_remove,
	.shutdown = x1600_lcd_shutdown,
};
module_platform_driver(x1600_lcd_platform_driver);

MODULE_AUTHOR("Olivier Louvignes <olivier@mgcrea.io>");
MODULE_DESCRIPTION("DRM driver for the Ingenic X1600 LCD controller");
MODULE_LICENSE("GPL");
