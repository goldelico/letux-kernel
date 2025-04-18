/* -*- mode: c; indent-tabs-mode: t; c-basic-offset: 8; tab-width: 8 -*- */
/* vi: set ts=8 sw=8 sts=8: */
/*************************************************************************/ /*!
@File
@Codingstyle    LinuxKernel
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/jiffies.h>
#include <linux/fence.h>
#include <linux/reservation.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/capability.h>
#include <linux/completion.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_plane_helper.h>

#include <pvrversion.h>

#include "drm_nulldisp_gem.h"
#include "drm_netlink_gem.h"
#include "drm_nulldisp_netlink.h"

#define DRIVER_NAME "nulldisp"
#define DRIVER_DESC "Imagination Technologies Null DRM Display Driver"
#define DRIVER_DATE "20160629"

#define NULLDISP_FB_WIDTH_MIN 0
#define NULLDISP_FB_WIDTH_MAX 2048
#define NULLDISP_FB_HEIGHT_MIN 0
#define NULLDISP_FB_HEIGHT_MAX 2048

enum nulldisp_crtc_flip_status {
	NULLDISP_CRTC_FLIP_STATUS_NONE = 0,
	NULLDISP_CRTC_FLIP_STATUS_PENDING,
	NULLDISP_CRTC_FLIP_STATUS_DONE,
};

struct nulldisp_flip_data {
	struct fence_cb base;
	struct drm_crtc *crtc;
	struct fence *wait_fence;
	struct fence *complete_fence;
};

struct nulldisp_fence_context {
	unsigned int context;
	const char *name;
	atomic_t seqno;
	atomic_t fence_count;
};

struct nulldisp_fence {
	struct nulldisp_fence_context *fence_context;
	struct fence base;
	spinlock_t lock;
};

struct nulldisp_crtc {
	struct drm_crtc base;
	struct delayed_work vb_work;
	struct work_struct flip_work;
	struct delayed_work flip_to_work;
	struct delayed_work copy_to_work;

	struct completion flip_scheduled;
	struct completion copy_done;

	/* Reuse the drm_device event_lock to protect these */
	atomic_t flip_status;
	struct drm_pending_vblank_event *flip_event;
	struct drm_framebuffer *old_fb;
	struct nulldisp_flip_data *flip_data;
	bool flip_async;
};

struct nulldisp_display_device {
	struct drm_device *dev;

	struct nulldisp_fence_context *fctx;

	struct workqueue_struct *workqueue;
	struct nulldisp_crtc *nulldisp_crtc;
	struct nlpvrdpy *nlpvrdpy;
};

struct nulldisp_framebuffer {
	struct drm_framebuffer base;
	struct drm_gem_object *obj;
};

#define to_nulldisp_fence(fence) \
	container_of(fence, struct nulldisp_fence, base)
#define to_nulldisp_crtc(crtc) \
	container_of(crtc, struct nulldisp_crtc, base)
#define to_nulldisp_framebuffer(framebuffer) \
	container_of(framebuffer, struct nulldisp_framebuffer, base)

#define	obj_to_resv(obj) nulldisp_gem_get_resv(obj)

#define PARAM_STRING_LEN 128

/******************************************************************************
 * Fence functions
 ******************************************************************************/

static inline unsigned int
nulldisp_fence_context_seqno_next(struct nulldisp_fence_context *fence_context)
{
	return atomic_inc_return(&fence_context->seqno) - 1;
}

static struct nulldisp_fence_context *
nulldisp_fence_context_create(const char *name)
{
	struct nulldisp_fence_context *fence_context;

	fence_context = kmalloc(sizeof(*fence_context), GFP_KERNEL);
	if (!fence_context)
		return NULL;

	fence_context->context = fence_context_alloc(1);
	fence_context->name = name;
	atomic_set(&fence_context->seqno, 0);
	atomic_set(&fence_context->fence_count, 0);

	return fence_context;
}

static void
nulldisp_fence_context_destroy(struct nulldisp_fence_context *fence_context)
{
	unsigned int fence_count;

	fence_count = atomic_read(&fence_context->fence_count);
	if (WARN_ON(fence_count))
		DRM_DEBUG_DRIVER("%s context has %u fence(s) remaining\n",
				 fence_context->name, fence_count);

	kfree(fence_context);
}

static const char *
nulldisp_fence_get_driver_name(struct fence *fence)
{
	return "nulldisp";
}

static const char *
nulldisp_fence_get_timeline_name(struct fence *fence)
{
	struct nulldisp_fence *nulldisp_fence = to_nulldisp_fence(fence);

	return nulldisp_fence->fence_context->name;
}

static bool
nulldisp_fence_enable_signaling(struct fence *fence)
{
	return true;
}

static void
nulldisp_fence_release(struct fence *fence)
{
	struct nulldisp_fence *nulldisp_fence = to_nulldisp_fence(fence);

	atomic_dec(&nulldisp_fence->fence_context->fence_count);
	kfree(nulldisp_fence);
}

static struct fence_ops nulldisp_fence_ops = {
	.get_driver_name = nulldisp_fence_get_driver_name,
	.get_timeline_name = nulldisp_fence_get_timeline_name,
	.enable_signaling = nulldisp_fence_enable_signaling,
	.wait = fence_default_wait,
	.release = nulldisp_fence_release,
};

static struct fence *
nulldisp_fence_create(struct nulldisp_fence_context *fence_context)
{
	struct nulldisp_fence *nulldisp_fence;
	unsigned int seqno;

	nulldisp_fence = kmalloc(sizeof(*nulldisp_fence), GFP_KERNEL);
	if (!nulldisp_fence)
		return NULL;

	spin_lock_init(&nulldisp_fence->lock);
	nulldisp_fence->fence_context = fence_context;

	seqno = nulldisp_fence_context_seqno_next(fence_context);
	fence_init(&nulldisp_fence->base, &nulldisp_fence_ops,
			&nulldisp_fence->lock, fence_context->context, seqno);

	atomic_inc(&fence_context->fence_count);

	return &nulldisp_fence->base;
}

/******************************************************************************
 * CRTC functions
 ******************************************************************************/

static void nulldisp_crtc_helper_dpms(struct drm_crtc *crtc,
				      int mode)
{
	/*
	 * Change the power state of the display/pipe/port/etc. If the mode
	 * passed in is unsupported, the provider must use the next lowest
	 * power level.
	 */
}

static void nulldisp_crtc_helper_prepare(struct drm_crtc *crtc)
{
	drm_crtc_vblank_off(crtc);

	/*
	 * Prepare the display/pipe/port/etc for a mode change e.g. put them
	 * in a low power state/turn them off
	 */
}

static void nulldisp_crtc_helper_commit(struct drm_crtc *crtc)
{
	/* Turn the display/pipe/port/etc back on */

	drm_crtc_vblank_on(crtc);
}

static bool
nulldisp_crtc_helper_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	/*
	 * Fix up mode so that it's compatible with the hardware. The results
	 * should be stored in adjusted_mode (i.e. mode should be untouched).
	 */
	return true;
}

static int
nulldisp_crtc_helper_mode_set_base_atomic(struct drm_crtc *crtc,
					  struct drm_framebuffer *fb,
					  int x, int y,
					  enum mode_set_atomic atomic)
{
	/* Set the display base address or offset from the base address */
	return 0;
}

static int nulldisp_crtc_helper_mode_set_base(struct drm_crtc *crtc,
					      int x, int y,
					      struct drm_framebuffer *old_fb)
{
	return nulldisp_crtc_helper_mode_set_base_atomic(crtc,
							 crtc->primary->fb,
							 x,
							 y,
							 0);
}

static int
nulldisp_crtc_helper_mode_set(struct drm_crtc *crtc,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode,
			      int x, int y,
			      struct drm_framebuffer *old_fb)
{
	/* Setup the the new mode and/or framebuffer */
	return nulldisp_crtc_helper_mode_set_base(crtc, x, y, old_fb);
}

static void nulldisp_crtc_helper_load_lut(struct drm_crtc *crtc)
{
}

static void nulldisp_crtc_destroy(struct drm_crtc *crtc)
{
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);

	DRM_DEBUG_DRIVER("[CRTC:%d]\n", crtc->base.id);

	drm_crtc_cleanup(crtc);

	BUG_ON(atomic_read(&nulldisp_crtc->flip_status) !=
					NULLDISP_CRTC_FLIP_STATUS_NONE);

	kfree(nulldisp_crtc);
}

static int nulldisp_crtc_set_config(struct drm_mode_set *mode_set)
{
	return drm_crtc_helper_set_config(mode_set);
}

static void nulldisp_crtc_flip_complete(struct drm_crtc *crtc)
{
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);
	unsigned long flags;
	struct fence *fence;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	/* The flipping process has been completed so reset the flip status */
	atomic_set(&nulldisp_crtc->flip_status, NULLDISP_CRTC_FLIP_STATUS_NONE);

	fence = nulldisp_crtc->flip_data->complete_fence;

	fence_put(nulldisp_crtc->flip_data->wait_fence);
	kfree(nulldisp_crtc->flip_data);
	nulldisp_crtc->flip_data = NULL;

	if (nulldisp_crtc->flip_event) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
		drm_crtc_send_vblank_event(crtc, nulldisp_crtc->flip_event);
#else
		drm_send_vblank_event(crtc->dev, 0, nulldisp_crtc->flip_event);
#endif
		nulldisp_crtc->flip_event = NULL;
	}

	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	WARN_ON(fence_signal(fence));
	fence_put(fence);
}

static void nulldisp_crtc_flip_done(struct nulldisp_crtc *nulldisp_crtc)
{
	struct drm_crtc *crtc = &nulldisp_crtc->base;

	struct drm_framebuffer *old_fb;

	WARN_ON(atomic_read(&nulldisp_crtc->flip_status) !=
					NULLDISP_CRTC_FLIP_STATUS_PENDING);

	old_fb = nulldisp_crtc->old_fb;
	nulldisp_crtc->old_fb = NULL;

	(void) nulldisp_crtc_helper_mode_set_base(crtc, crtc->x, crtc->y,
								old_fb);

	atomic_set(&nulldisp_crtc->flip_status, NULLDISP_CRTC_FLIP_STATUS_DONE);

	if (nulldisp_crtc->flip_async)
		nulldisp_crtc_flip_complete(crtc);
}

static inline unsigned long nulldisp_netlink_timeout(void)
{
	return msecs_to_jiffies(30000);
}

static bool nulldisp_set_flip_to(struct nulldisp_crtc *nulldisp_crtc)
{
	struct drm_crtc *crtc = &nulldisp_crtc->base;
	struct nulldisp_display_device *nulldisp_dev = crtc->dev->dev_private;

	/* Returns false if work already queued, else true */
	return queue_delayed_work(nulldisp_dev->workqueue,
			&nulldisp_crtc->flip_to_work,
			nulldisp_netlink_timeout());
}

static bool nulldisp_set_copy_to(struct nulldisp_crtc *nulldisp_crtc)
{
	struct drm_crtc *crtc = &nulldisp_crtc->base;
	struct nulldisp_display_device *nulldisp_dev = crtc->dev->dev_private;

	/* Returns false if work already queued, else true */
	return queue_delayed_work(nulldisp_dev->workqueue,
			&nulldisp_crtc->copy_to_work,
			nulldisp_netlink_timeout());
}

static void nulldisp_flip_to_work(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct nulldisp_crtc *nulldisp_crtc =
		container_of(dw, struct nulldisp_crtc, flip_to_work);

	if (atomic_read(&nulldisp_crtc->flip_status) ==
				NULLDISP_CRTC_FLIP_STATUS_PENDING)
		nulldisp_crtc_flip_done(nulldisp_crtc);
}

static void nulldisp_copy_to_work(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct nulldisp_crtc *nulldisp_crtc =
		container_of(dw, struct nulldisp_crtc, copy_to_work);

	complete(&nulldisp_crtc->copy_done);
}

static void nulldisp_flip_work(struct work_struct *w)
{
	struct nulldisp_crtc *nulldisp_crtc =
		container_of(w, struct nulldisp_crtc, flip_work);
	struct drm_crtc *crtc = &nulldisp_crtc->base;
	struct drm_device *dev = crtc->dev;
	struct nulldisp_display_device *nulldisp_dev = dev->dev_private;
	struct nulldisp_framebuffer *nulldisp_fb =
		to_nulldisp_framebuffer(crtc->primary->fb);
	struct drm_gem_object *obj = nulldisp_fb->obj;
	int err;

	/*
	 * To prevent races with disconnect requests from user space,
	 * set the timeout before sending the flip request.
	 */
	err = drm_gem_create_mmap_offset(obj);
	nulldisp_set_flip_to(nulldisp_crtc);
	if (err)
		DRM_ERROR("No mmap offset to send\n");
	else
		err = nlpvrdpy_send_flip(nulldisp_dev->nlpvrdpy,
				&nulldisp_fb->base,
				drm_vma_node_offset_addr(&obj->vma_node),
				obj->size);
	if (err) {
		/*
		 * We can't flush the work, as we are running on the same
		 * single threaded workqueue as the work to be flushed.
		 */
		cancel_delayed_work(&nulldisp_crtc->flip_to_work);

		nulldisp_crtc_flip_done(nulldisp_crtc);
	}
}

static void nulldisp_crtc_flip_cb(struct fence *fence, struct fence_cb *cb)
{
	struct nulldisp_flip_data *flip_data =
		container_of(cb, struct nulldisp_flip_data, base);
	struct drm_crtc *crtc = flip_data->crtc;
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct nulldisp_display_device *nulldisp_dev = dev->dev_private;

	(void) queue_work(nulldisp_dev->workqueue,
				&nulldisp_crtc->flip_work);

	complete_all(&nulldisp_crtc->flip_scheduled);
}

static void drm_crtc_flip_schedule_cb(struct fence *fence, struct fence_cb *cb)
{
	struct nulldisp_flip_data *flip_data =
		container_of(cb, struct nulldisp_flip_data, base);
	int err = 0;

	if (flip_data->wait_fence)
		err = fence_add_callback(flip_data->wait_fence,
					 &flip_data->base,
					 nulldisp_crtc_flip_cb);

	if (!flip_data->wait_fence || err) {
		if (err && err != -ENOENT)
			DRM_ERROR("flip failed to wait on old buffer\n");
		nulldisp_crtc_flip_cb(flip_data->wait_fence, &flip_data->base);
	}
}

static int drm_crtc_flip_schedule(struct drm_crtc *crtc,
				  struct drm_gem_object *obj,
				  struct drm_gem_object *old_obj)
{
	struct nulldisp_display_device *nulldisp_dev = crtc->dev->dev_private;
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);
	struct reservation_object *resv = obj_to_resv(obj);
	struct reservation_object *old_resv = obj_to_resv(old_obj);
	struct nulldisp_flip_data *flip_data;
	struct fence *fence;
	int err;

	flip_data = kmalloc(sizeof(*flip_data), GFP_KERNEL);
	if (!flip_data)
		return -ENOMEM;

	flip_data->crtc = crtc;

	flip_data->complete_fence = nulldisp_fence_create(nulldisp_dev->fctx);
	if (!flip_data->complete_fence) {
		err = -ENOMEM;
		goto err_free_fence_data;
	}

	ww_mutex_lock(&old_resv->lock, NULL);
	err = reservation_object_reserve_shared(old_resv);
	if (err) {
		ww_mutex_unlock(&old_resv->lock);
		goto err_complete_fence_put;
	}

	reservation_object_add_shared_fence(old_resv,
					    flip_data->complete_fence);

	flip_data->wait_fence =
		fence_get(reservation_object_get_excl(old_resv));

	if (old_resv != resv) {
		ww_mutex_unlock(&old_resv->lock);
		ww_mutex_lock(&resv->lock, NULL);
	}

	fence = fence_get(reservation_object_get_excl(resv));
	ww_mutex_unlock(&resv->lock);

	nulldisp_crtc->flip_data = flip_data;
	reinit_completion(&nulldisp_crtc->flip_scheduled);
	atomic_set(&nulldisp_crtc->flip_status,
					NULLDISP_CRTC_FLIP_STATUS_PENDING);

	if (fence) {
		err = fence_add_callback(fence, &flip_data->base,
					 drm_crtc_flip_schedule_cb);
		fence_put(fence);
		if (err && err != -ENOENT)
			goto err_set_flip_status_none;
	}

	if (!fence || err == -ENOENT) {
		drm_crtc_flip_schedule_cb(fence, &flip_data->base);
		err = 0;
	}

	return err;

err_set_flip_status_none:
	atomic_set(&nulldisp_crtc->flip_status, NULLDISP_CRTC_FLIP_STATUS_NONE);
	fence_put(flip_data->wait_fence);
err_complete_fence_put:
	fence_put(flip_data->complete_fence);
err_free_fence_data:
	kfree(flip_data);
	return err;
}

static int nulldisp_crtc_page_flip(struct drm_crtc *crtc,
				   struct drm_framebuffer *fb,
				   struct drm_pending_vblank_event *event,
				   uint32_t page_flip_flags)
{
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);
	struct nulldisp_framebuffer *nulldisp_fb = to_nulldisp_framebuffer(fb);
	struct nulldisp_framebuffer *nulldisp_old_fb =
		to_nulldisp_framebuffer(crtc->primary->fb);
	enum nulldisp_crtc_flip_status status;
	unsigned long flags;
	int err;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	status = atomic_read(&nulldisp_crtc->flip_status);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	if (status != NULLDISP_CRTC_FLIP_STATUS_NONE)
		return -EBUSY;

	if (!(page_flip_flags & DRM_MODE_PAGE_FLIP_ASYNC)) {
		err = drm_crtc_vblank_get(crtc);
		if (err)
			return err;
	}

	nulldisp_crtc->old_fb = crtc->primary->fb;
	nulldisp_crtc->flip_event = event;
	nulldisp_crtc->flip_async = !!(page_flip_flags &
						DRM_MODE_PAGE_FLIP_ASYNC);

	/* Set the crtc to point to the new framebuffer */
	crtc->primary->fb = fb;

	err = drm_crtc_flip_schedule(crtc, nulldisp_fb->obj,
							nulldisp_old_fb->obj);
	if (err) {
		crtc->primary->fb = nulldisp_crtc->old_fb;
		nulldisp_crtc->old_fb = NULL;
		nulldisp_crtc->flip_event = NULL;
		nulldisp_crtc->flip_async = false;

		DRM_ERROR("failed to schedule flip (err=%d)\n", err);
		goto err_vblank_put;
	}

	return 0;

err_vblank_put:
	if (!(page_flip_flags & DRM_MODE_PAGE_FLIP_ASYNC))
		drm_crtc_vblank_put(crtc);
	return err;
}

static void nulldisp_crtc_helper_disable(struct drm_crtc *crtc)
{
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);

	if (atomic_read(&nulldisp_crtc->flip_status) ==
					NULLDISP_CRTC_FLIP_STATUS_PENDING)
		wait_for_completion(&nulldisp_crtc->flip_scheduled);

	/*
	 * Flush any outstanding page flip related work. The order this
	 * is done is important, to ensure there are no outstanding
	 * page flips.
	 */
	flush_work(&nulldisp_crtc->flip_work);
	flush_delayed_work(&nulldisp_crtc->flip_to_work);
	flush_delayed_work(&nulldisp_crtc->vb_work);

	drm_crtc_vblank_off(crtc);
	flush_delayed_work(&nulldisp_crtc->vb_work);

	/*
	 * Vblank has been disabled, so the vblank handler shouldn't be
	 * able to reschedule itself.
	 */
	BUG_ON(cancel_delayed_work(&nulldisp_crtc->vb_work));

	BUG_ON(atomic_read(&nulldisp_crtc->flip_status) !=
					NULLDISP_CRTC_FLIP_STATUS_NONE);

	/* Flush any remaining dirty FB work */
	flush_delayed_work(&nulldisp_crtc->copy_to_work);
}

static const struct drm_crtc_helper_funcs nulldisp_crtc_helper_funcs = {
	.dpms = nulldisp_crtc_helper_dpms,
	.prepare = nulldisp_crtc_helper_prepare,
	.commit = nulldisp_crtc_helper_commit,
	.mode_fixup = nulldisp_crtc_helper_mode_fixup,
	.mode_set = nulldisp_crtc_helper_mode_set,
	.mode_set_base = nulldisp_crtc_helper_mode_set_base,
	.load_lut = nulldisp_crtc_helper_load_lut,
	.mode_set_base_atomic = nulldisp_crtc_helper_mode_set_base_atomic,
	.disable = nulldisp_crtc_helper_disable,
};

static const struct drm_crtc_funcs nulldisp_crtc_funcs = {
	.reset = NULL,
	.cursor_set = NULL,
	.cursor_move = NULL,
	.gamma_set = NULL,
	.destroy = nulldisp_crtc_destroy,
	.set_config = nulldisp_crtc_set_config,
	.page_flip = nulldisp_crtc_page_flip,
};

static bool nulldisp_queue_vblank_work(struct nulldisp_crtc *nulldisp_crtc)
{
	struct drm_crtc *crtc = &nulldisp_crtc->base;
	struct nulldisp_display_device *nulldisp_dev = crtc->dev->dev_private;
	int vrefresh;
	const int vrefresh_default = 60;

	if (crtc->hwmode.vrefresh) {
		vrefresh = crtc->hwmode.vrefresh;
	} else {
		vrefresh = vrefresh_default;
		DRM_ERROR("vertical refresh rate is zero, defaulting to %d\n",
			  vrefresh);
	}

	/* Returns false if work already queued, else true */
	return queue_delayed_work(nulldisp_dev->workqueue,
			&nulldisp_crtc->vb_work,
			usecs_to_jiffies(1000000/vrefresh));
}

static void nulldisp_handle_vblank(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct nulldisp_crtc *nulldisp_crtc =
		container_of(dw, struct nulldisp_crtc, vb_work);
	struct drm_crtc *crtc = &nulldisp_crtc->base;
	struct drm_device *dev = crtc->dev;
	enum nulldisp_crtc_flip_status status;

	/*
	 * Reschedule the handler, if necessary. This is done before
	 * calling drm_crtc_vblank_put, so that the work can be cancelled
	 * if vblank events are disabled.
	 */
	if (drm_handle_vblank(dev, 0))
		(void) nulldisp_queue_vblank_work(nulldisp_crtc);

	status = atomic_read(&nulldisp_crtc->flip_status);
	if (status == NULLDISP_CRTC_FLIP_STATUS_DONE) {
		if (!nulldisp_crtc->flip_async)
			nulldisp_crtc_flip_complete(crtc);
		drm_crtc_vblank_put(crtc);
	}

}

static struct nulldisp_crtc *
nulldisp_crtc_create(struct nulldisp_display_device *nulldisp_dev)
{
	struct nulldisp_crtc *nulldisp_crtc;
	struct drm_crtc *crtc;

	nulldisp_crtc = kzalloc(sizeof(*nulldisp_crtc), GFP_KERNEL);
	if (!nulldisp_crtc)
		return NULL;

	crtc = &nulldisp_crtc->base;

	atomic_set(&nulldisp_crtc->flip_status, NULLDISP_CRTC_FLIP_STATUS_NONE);
	init_completion(&nulldisp_crtc->flip_scheduled);
	init_completion(&nulldisp_crtc->copy_done);

	drm_crtc_init(nulldisp_dev->dev, crtc, &nulldisp_crtc_funcs);
	drm_crtc_helper_add(crtc, &nulldisp_crtc_helper_funcs);

	INIT_DELAYED_WORK(&nulldisp_crtc->vb_work, nulldisp_handle_vblank);
	INIT_WORK(&nulldisp_crtc->flip_work, nulldisp_flip_work);
	INIT_DELAYED_WORK(&nulldisp_crtc->flip_to_work, nulldisp_flip_to_work);
	INIT_DELAYED_WORK(&nulldisp_crtc->copy_to_work, nulldisp_copy_to_work);

	DRM_DEBUG_DRIVER("[CRTC:%d]\n", crtc->base.id);

	return nulldisp_crtc;
}


/******************************************************************************
 * Connector functions
 ******************************************************************************/

static int
nulldisp_connector_helper_get_modes(struct drm_connector *connector)
{
	/*
	 * Gather modes. Here we can get the EDID data from the monitor and
	 * turn it into drm_display_mode structures.
	 */
	return 0;
}

static int
nulldisp_connector_helper_mode_valid(struct drm_connector *connector,
				     struct drm_display_mode *mode)
{
	/*
	 * This function is called on each gathered mode (e.g. via EDID)
	 * and gives the driver a chance to reject it if the hardware
	 * cannot support it.
	 */
	return MODE_OK;
}

static struct drm_encoder *
nulldisp_connector_helper_best_encoder(struct drm_connector *connector)
{
	/* Pick the first encoder we find */
	if (connector->encoder_ids[0] != 0) {
		struct drm_mode_object *mode_object;

		mode_object = drm_mode_object_find(connector->dev,
						   connector->encoder_ids[0],
						   DRM_MODE_OBJECT_ENCODER);
		if (mode_object) {
			struct drm_encoder *encoder =
				obj_to_encoder(mode_object);

			DRM_DEBUG_DRIVER("[ENCODER:%d:%s] best for "
					 "[CONNECTOR:%d:%s]\n",
					 encoder->base.id,
					 encoder->name,
					 connector->base.id,
					 connector->name);
			return encoder;
		}
	}

	return NULL;
}

static enum drm_connector_status
nulldisp_connector_detect(struct drm_connector *connector,
			  bool force)
{
	/* Return whether or not a monitor is attached to the connector */
	return connector_status_connected;
}

static void nulldisp_connector_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_DRIVER("[CONNECTOR:%d:%s]\n",
			 connector->base.id,
			 connector->name);

	drm_connector_unregister(connector);

	drm_mode_connector_update_edid_property(connector, NULL);
	drm_connector_cleanup(connector);

	kfree(connector);
}

static void nulldisp_connector_force(struct drm_connector *connector)
{
}

static const struct drm_connector_helper_funcs
nulldisp_connector_helper_funcs = {
	.get_modes = nulldisp_connector_helper_get_modes,
	.mode_valid = nulldisp_connector_helper_mode_valid,
	.best_encoder = nulldisp_connector_helper_best_encoder,
};

static const struct drm_connector_funcs nulldisp_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.reset = NULL,
	.detect = nulldisp_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = nulldisp_connector_destroy,
	.force = nulldisp_connector_force,
};

static struct drm_connector *
nulldisp_connector_create(struct nulldisp_display_device *nulldisp_dev,
			  int type)
{
	struct drm_connector *connector;

	connector = kzalloc(sizeof(*connector), GFP_KERNEL);
	if (!connector)
		return NULL;

	drm_connector_init(nulldisp_dev->dev,
			   connector,
			   &nulldisp_connector_funcs,
			   type);
	drm_connector_helper_add(connector, &nulldisp_connector_helper_funcs);

	connector->dpms = DRM_MODE_DPMS_OFF;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;
	connector->display_info.subpixel_order = SubPixelUnknown;

	DRM_DEBUG_DRIVER("[CONNECTOR:%d:%s]\n",
			 connector->base.id,
			 connector->name);

	return connector;
}


/******************************************************************************
 * Encoder functions
 ******************************************************************************/

static void nulldisp_encoder_helper_dpms(struct drm_encoder *encoder,
					 int mode)
{
	/*
	 * Set the display power state or active encoder based on the mode. If
	 * the mode passed in is unsupported, the provider must use the next
	 * lowest power level.
	 */
}

static bool
nulldisp_encoder_helper_mode_fixup(struct drm_encoder *encoder,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	/*
	 * Fix up mode so that it's compatible with the hardware. The results
	 * should be stored in adjusted_mode (i.e. mode should be untouched).
	 */
	return true;
}

static void nulldisp_encoder_helper_prepare(struct drm_encoder *encoder)
{
	/*
	 * Prepare the encoder for a mode change e.g. set the active encoder
	 * accordingly/turn the encoder off
	 */
}

static void nulldisp_encoder_helper_commit(struct drm_encoder *encoder)
{
	/* Turn the encoder back on/set the active encoder */
}

static void
nulldisp_encoder_helper_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	/* Setup the encoder for the new mode */
}

static void nulldisp_encoder_destroy(struct drm_encoder *encoder)
{
	DRM_DEBUG_DRIVER("[ENCODER:%d:%s]\n", encoder->base.id, encoder->name);

	drm_encoder_cleanup(encoder);
	kfree(encoder);
}

static const struct drm_encoder_helper_funcs nulldisp_encoder_helper_funcs = {
	.dpms = nulldisp_encoder_helper_dpms,
	.mode_fixup = nulldisp_encoder_helper_mode_fixup,
	.prepare = nulldisp_encoder_helper_prepare,
	.commit = nulldisp_encoder_helper_commit,
	.mode_set = nulldisp_encoder_helper_mode_set,
	.get_crtc = NULL,
	.detect = NULL,
	.disable = NULL,
};

static const struct drm_encoder_funcs nulldisp_encoder_funcs = {
	.reset = NULL,
	.destroy = nulldisp_encoder_destroy,
};

static struct drm_encoder *
nulldisp_encoder_create(struct nulldisp_display_device *nulldisp_dev,
			int type)
{
	struct drm_encoder *encoder;
	int err;

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return ERR_PTR(-ENOMEM);

	err = drm_encoder_init(nulldisp_dev->dev,
				   encoder,
				   &nulldisp_encoder_funcs,
				   type
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
				   );
#else
				   , NULL);
#endif

	if (err) {
		DRM_ERROR("Failed to initialise encoder\n");
		return ERR_PTR(err);
	}
	drm_encoder_helper_add(encoder, &nulldisp_encoder_helper_funcs);

	/*
	 * This is a bit field that's used to determine which
	 * CRTCs can drive this encoder.
	 */
	encoder->possible_crtcs = 0x1;

	DRM_DEBUG_DRIVER("[ENCODER:%d:%s]\n", encoder->base.id, encoder->name);

	return encoder;
}


/******************************************************************************
 * Framebuffer functions
 ******************************************************************************/

static void nulldisp_framebuffer_destroy(struct drm_framebuffer *framebuffer)
{
	struct nulldisp_framebuffer *nulldisp_framebuffer =
		to_nulldisp_framebuffer(framebuffer);

	DRM_DEBUG_DRIVER("[FB:%d]\n", framebuffer->base.id);

	drm_framebuffer_cleanup(framebuffer);

	drm_gem_object_unreference_unlocked(nulldisp_framebuffer->obj);

	kfree(nulldisp_framebuffer);
}

static int
nulldisp_framebuffer_create_handle(struct drm_framebuffer *framebuffer,
				   struct drm_file *file_priv,
				   unsigned int *handle)
{
	struct nulldisp_framebuffer *nulldisp_framebuffer =
		to_nulldisp_framebuffer(framebuffer);

	DRM_DEBUG_DRIVER("[FB:%d]\n", framebuffer->base.id);

	return drm_gem_handle_create(file_priv,
				     nulldisp_framebuffer->obj,
				     handle);
}

static int
nulldisp_framebuffer_dirty(struct drm_framebuffer *framebuffer,
			   struct drm_file *file_priv,
			   unsigned flags,
			   unsigned color,
			   struct drm_clip_rect *clips,
			   unsigned num_clips)
{
	struct nulldisp_framebuffer *nulldisp_fb =
		to_nulldisp_framebuffer(framebuffer);
	struct nulldisp_display_device *nulldisp_dev =
		framebuffer->dev->dev_private;
	struct nulldisp_crtc *nulldisp_crtc = nulldisp_dev->nulldisp_crtc;
	struct drm_gem_object *obj = nulldisp_fb->obj;
	int err;

	/*
	 * To prevent races with disconnect requests from user space,
	 * set the timeout before sending the copy request.
	 */
	err = drm_gem_create_mmap_offset(obj);
	nulldisp_set_copy_to(nulldisp_crtc);
	if (err)
		DRM_ERROR("No mmap offset to send\n");
	else
		err = nlpvrdpy_send_copy(nulldisp_dev->nlpvrdpy,
				&nulldisp_fb->base,
				drm_vma_node_offset_addr(&obj->vma_node),
				obj->size);
	if (err)
		flush_delayed_work(&nulldisp_crtc->copy_to_work);

	wait_for_completion(&nulldisp_crtc->copy_done);

	return 0;
}

static const struct drm_framebuffer_funcs nulldisp_framebuffer_funcs = {
	.destroy = nulldisp_framebuffer_destroy,
	.create_handle = nulldisp_framebuffer_create_handle,
	.dirty = nulldisp_framebuffer_dirty,
};

static int
nulldisp_framebuffer_init(struct drm_device *dev,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
			  struct drm_mode_fb_cmd2 *mode_cmd,
#else
			  const struct drm_mode_fb_cmd2 *mode_cmd,
#endif
			  struct nulldisp_framebuffer *nulldisp_framebuffer,
			  struct drm_gem_object *obj)
{
	int err;

	err = drm_framebuffer_init(dev,
				   &nulldisp_framebuffer->base,
				   &nulldisp_framebuffer_funcs);
	if (err) {
		DRM_ERROR("failed to initialise framebuffer structure (%d)\n",
			  err);
		return err;
	}

	drm_helper_mode_fill_fb_struct(&nulldisp_framebuffer->base, mode_cmd);

	nulldisp_framebuffer->obj = obj;

	DRM_DEBUG_DRIVER("[FB:%d]\n", nulldisp_framebuffer->base.base.id);

	return 0;
}

static struct drm_framebuffer *
nulldisp_fb_create(struct drm_device *dev,
		   struct drm_file *file_priv,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
		   struct drm_mode_fb_cmd2 *mode_cmd)
#else
		   const struct drm_mode_fb_cmd2 *mode_cmd)
#endif
{
	struct drm_gem_object *obj;
	struct nulldisp_framebuffer *nulldisp_framebuffer;
	int err;

	obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
	if (!obj) {
		DRM_ERROR("failed to find buffer with handle %u\n",
			  mode_cmd->handles[0]);
		return ERR_PTR(-ENOENT);
	}

	nulldisp_framebuffer = kzalloc(sizeof(*nulldisp_framebuffer),
				       GFP_KERNEL);
	if (!nulldisp_framebuffer) {
		drm_gem_object_unreference_unlocked(obj);
		return ERR_PTR(-ENOMEM);
	}

	err = nulldisp_framebuffer_init(dev,
					mode_cmd,
					nulldisp_framebuffer,
					obj);
	if (err) {
		kfree(nulldisp_framebuffer);
		drm_gem_object_unreference_unlocked(obj);
		return ERR_PTR(err);
	}

	DRM_DEBUG_DRIVER("[FB:%d]\n", nulldisp_framebuffer->base.base.id);

	return &nulldisp_framebuffer->base;
}

static const struct drm_mode_config_funcs nulldisp_mode_config_funcs = {
	.fb_create = nulldisp_fb_create,
	.output_poll_changed = NULL,
};

static int nulldisp_nl_flipped_cb(void *data)
{
	struct nulldisp_crtc *nulldisp_crtc = data;

	flush_delayed_work(&nulldisp_crtc->flip_to_work);
	flush_delayed_work(&nulldisp_crtc->vb_work);

	return 0;
}

static int nulldisp_nl_copied_cb(void *data)
{
	struct nulldisp_crtc *nulldisp_crtc = data;

	flush_delayed_work(&nulldisp_crtc->copy_to_work);

	return 0;
}

static void nulldisp_nl_disconnect_cb(void *data)
{
	struct nulldisp_crtc *nulldisp_crtc = data;

	flush_delayed_work(&nulldisp_crtc->flip_to_work);
	flush_delayed_work(&nulldisp_crtc->copy_to_work);
}

static int nulldisp_load(struct drm_device *dev, unsigned long flags)
{
	struct nulldisp_display_device *nulldisp_dev;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	int err;

	platform_set_drvdata(dev->platformdev, dev);

	nulldisp_dev = kzalloc(sizeof(*nulldisp_dev), GFP_KERNEL);
	if (!nulldisp_dev)
		return -ENOMEM;

	dev->dev_private = nulldisp_dev;
	nulldisp_dev->dev = dev;

	drm_mode_config_init(dev);

	dev->mode_config.funcs = (void *)&nulldisp_mode_config_funcs;
	dev->mode_config.min_width = NULLDISP_FB_WIDTH_MIN;
	dev->mode_config.max_width = NULLDISP_FB_WIDTH_MAX;
	dev->mode_config.min_height = NULLDISP_FB_HEIGHT_MIN;
	dev->mode_config.max_height = NULLDISP_FB_HEIGHT_MAX;
	dev->mode_config.fb_base = 0;
	dev->mode_config.async_page_flip = true;

	nulldisp_dev->nulldisp_crtc = nulldisp_crtc_create(nulldisp_dev);
	if (!nulldisp_dev->nulldisp_crtc) {
		DRM_ERROR("failed to create a CRTC.\n");

		err = -ENOMEM;
		goto err_config_cleanup;
	}

	connector = nulldisp_connector_create(nulldisp_dev,
					      DRM_MODE_CONNECTOR_Unknown);
	if (!connector) {
		DRM_ERROR("failed to create a connector.\n");

		err = -ENOMEM;
		goto err_config_cleanup;
	}

	encoder = nulldisp_encoder_create(nulldisp_dev,
					  DRM_MODE_ENCODER_NONE);
	if (IS_ERR(encoder)) {
		DRM_ERROR("failed to create an encoder.\n");

		err = PTR_ERR(encoder);
		goto err_config_cleanup;
	}

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("failed to attach [ENCODER:%d:%s] to "
			  "[CONNECTOR:%d:%s] (err=%d)\n",
			  encoder->base.id,
			  encoder->name,
			  connector->base.id,
			  connector->name,
			  err);
		goto err_config_cleanup;
	}

	err = drm_connector_register(connector);
	if (err) {
		DRM_ERROR("[CONNECTOR:%d:%s] failed to register (err=%d)\n",
			  connector->base.id,
			  connector->name,
			  err);
		goto err_config_cleanup;
	}

	nulldisp_dev->fctx = nulldisp_fence_context_create("nulldisp-nohw");
	if (!nulldisp_dev->fctx) {
		err = -ENOMEM;
		goto err_gem_cleanup;
	}

	nulldisp_dev->workqueue =
		create_singlethread_workqueue(DRIVER_NAME);
	if (!nulldisp_dev->workqueue) {
		DRM_ERROR("failed to create work queue\n");
		goto err_fence_context_cleanup;
	}

	err = drm_vblank_init(nulldisp_dev->dev, 1);
	if (err) {
		DRM_ERROR("failed to complete vblank init (err=%d)\n", err);
		goto err_workqueue_cleanup;
	}

	dev->irq_enabled = true;

	nulldisp_dev->nlpvrdpy = nlpvrdpy_create(dev,
						nulldisp_nl_disconnect_cb,
						nulldisp_dev->nulldisp_crtc,
						nulldisp_nl_flipped_cb,
						nulldisp_dev->nulldisp_crtc,
						nulldisp_nl_copied_cb,
						nulldisp_dev->nulldisp_crtc);
	if (!nulldisp_dev->nlpvrdpy) {
		DRM_ERROR("Netlink initialisation failed (err=%d)\n", err);
		goto err_vblank_cleanup;
	}

	nlpvrdpy_start();

	return 0;

err_vblank_cleanup:
	drm_vblank_cleanup(dev);
err_workqueue_cleanup:
	destroy_workqueue(nulldisp_dev->workqueue);
	dev->irq_enabled = false;
err_fence_context_cleanup:
	nulldisp_fence_context_destroy(nulldisp_dev->fctx);
err_gem_cleanup:
err_config_cleanup:
	drm_mode_config_cleanup(dev);
	kfree(nulldisp_dev);
	return err;
}

static int nulldisp_unload(struct drm_device *dev)
{
	struct nulldisp_display_device *nulldisp_dev = dev->dev_private;

	nlpvrdpy_send_disconnect(nulldisp_dev->nlpvrdpy);
	nlpvrdpy_destroy(nulldisp_dev->nlpvrdpy);

	drm_vblank_cleanup(dev);

	destroy_workqueue(nulldisp_dev->workqueue);

	dev->irq_enabled = false;

	nulldisp_fence_context_destroy(nulldisp_dev->fctx);

	drm_mode_config_cleanup(dev);

	kfree(nulldisp_dev);

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
static void
nulldisp_crtc_flip_event_cancel(struct drm_crtc *crtc, struct drm_file *file)
{
	struct nulldisp_crtc *nulldisp_crtc = to_nulldisp_crtc(crtc);
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	if (nulldisp_crtc->flip_event &&
	    nulldisp_crtc->flip_event->base.file_priv == file) {
		nulldisp_crtc->flip_event->base.destroy(
					&nulldisp_crtc->flip_event->base);
		nulldisp_crtc->flip_event = NULL;
	}

	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
static void nulldisp_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
		nulldisp_crtc_flip_event_cancel(crtc, file);
}
#endif

static void nulldisp_lastclose(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	drm_modeset_lock_all(dev);
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (crtc->primary->fb) {
			struct drm_mode_set mode_set = { .crtc = crtc };
			int err;

			err = drm_mode_set_config_internal(&mode_set);
			if (err)
				DRM_ERROR("failed to disable crtc %p (err=%d)\n",
					  crtc, err);
		}
	}
	drm_modeset_unlock_all(dev);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int nulldisp_enable_vblank(struct drm_device *dev, unsigned int crtc)
#else
static int nulldisp_enable_vblank(struct drm_device *dev, int crtc)
#endif
{
	struct nulldisp_display_device *nulldisp_dev = dev->dev_private;

	switch (crtc) {
	case 0:
		break;
	default:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		DRM_ERROR("invalid crtc %u\n", crtc);
#else
		DRM_ERROR("invalid crtc %d\n", crtc);
#endif
		return -EINVAL;
	}

	if (!nulldisp_queue_vblank_work(nulldisp_dev->nulldisp_crtc)) {
		DRM_ERROR("work already queued\n");
		return -1;
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static void nulldisp_disable_vblank(struct drm_device *dev, unsigned int crtc)
#else
static void nulldisp_disable_vblank(struct drm_device *dev, int crtc)
#endif
{
	struct nulldisp_display_device *nulldisp_dev = dev->dev_private;

	switch (crtc) {
	case 0:
		break;
	default:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		DRM_ERROR("invalid crtc %u\n", crtc);
#else
		DRM_ERROR("invalid crtc %d\n", crtc);
#endif
		return;
	}

	/*
	 * Vblank events may be disabled from within the vblank handler,
	 * so don't wait for the work to complete.
	 */
	(void) cancel_delayed_work(&nulldisp_dev->nulldisp_crtc->vb_work);
}

static const struct vm_operations_struct nulldisp_gem_vm_ops = {
	.fault	= nulldisp_gem_object_vm_fault,
	.open	= nulldisp_gem_vm_open,
	.close	= nulldisp_gem_vm_close,
};

static int nulldisp_gem_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err;

	err = netlink_gem_mmap(file, vma);
	if (!err) {
		struct drm_file *file_priv = file->private_data;
		struct drm_device *dev = file_priv->minor->dev;
		struct drm_gem_object *obj;

		mutex_lock(&dev->struct_mutex);
		obj = vma->vm_private_data;
		(void) nulldisp_gem_object_get_pages(obj);
		mutex_unlock(&dev->struct_mutex);
	}

	return err;
}

static const struct file_operations nulldisp_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
	.mmap		= nulldisp_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= noop_llseek,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
};

static struct drm_driver nulldisp_drm_driver = {
	.load				= nulldisp_load,
	.unload				= nulldisp_unload,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
	.preclose			= nulldisp_preclose,
#endif
	.lastclose			= nulldisp_lastclose,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
	.set_busid			= drm_platform_set_busid,
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	.get_vblank_counter		= drm_vblank_no_hw_counter,
#else
	.get_vblank_counter		= drm_vblank_count,
#endif
	.enable_vblank			= nulldisp_enable_vblank,
	.disable_vblank			= nulldisp_disable_vblank,

	.gem_prime_import		= drm_gem_prime_import,

	.gem_free_object		= nulldisp_gem_object_free,
	.gem_prime_export		= nulldisp_gem_prime_export,
	.gem_prime_pin			= nulldisp_gem_prime_pin,
	.gem_prime_unpin		= nulldisp_gem_prime_unpin,
	.gem_prime_get_sg_table		= nulldisp_gem_prime_get_sg_table,
	.gem_prime_import_sg_table	= nulldisp_gem_prime_import_sg_table,
	.gem_prime_vmap			= nulldisp_gem_prime_vmap,
	.gem_prime_vunmap		= nulldisp_gem_prime_vunmap,
	.gem_prime_res_obj		= nulldisp_gem_prime_res_obj,

	.dumb_create			= nulldisp_gem_dumb_create,
	.dumb_map_offset		= nulldisp_gem_dumb_map_offset,
	.dumb_destroy			= drm_gem_dumb_destroy,

	.gem_vm_ops			= &nulldisp_gem_vm_ops,

	.name				= DRIVER_NAME,
	.desc				= DRIVER_DESC,
	.date				= DRIVER_DATE,
	.major				= PVRVERSION_MAJ,
	.minor				= PVRVERSION_MIN,
	.patchlevel			= PVRVERSION_BUILD,

	.driver_features		= DRIVER_GEM |
					  DRIVER_MODESET |
					  DRIVER_PRIME,
	.fops				= &nulldisp_driver_fops,
};

static int nulldisp_probe(struct platform_device *pdev)
{
	return drm_platform_init(&nulldisp_drm_driver, pdev);
}

static int nulldisp_remove(struct platform_device *pdev)
{
	struct drm_device *dev = platform_get_drvdata(pdev);

	drm_put_dev(dev);

	return 0;
}

static void nulldisp_shutdown(struct platform_device *pdev)
{
}

static struct platform_device_id nulldisp_platform_device_id_table[] = {
	{ .name = "nulldisp", .driver_data = 0 },
	{ },
};

static struct platform_driver nulldisp_platform_driver = {
	.probe		= nulldisp_probe,
	.remove		= nulldisp_remove,
	.shutdown	= nulldisp_shutdown,
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.id_table	= nulldisp_platform_device_id_table,
};


static struct platform_device_info nulldisp_device_info = {
	.name		= "nulldisp",
	.id		= -1,
	.dma_mask	= DMA_BIT_MASK(32),
};

static struct platform_device *nulldisp_dev;

static int __init nulldisp_init(void)
{
	int err;

	err = nlpvrdpy_register();
	if (err) {
		DRM_ERROR("failed to register with netlink (err=%d)\n", err);
		return err;
	}

	nulldisp_dev = platform_device_register_full(&nulldisp_device_info);
	if (IS_ERR(nulldisp_dev)) {
		err = PTR_ERR(nulldisp_dev);
		nulldisp_dev = NULL;
		goto err_unregister_family;
	}

	err = platform_driver_register(&nulldisp_platform_driver);
	if (err)
		goto err_unregister_family;

	return 0;

err_unregister_family:
		(void) nlpvrdpy_unregister();
		return err;
}

static void __exit nulldisp_exit(void)
{
	int err;

	err = nlpvrdpy_unregister();
	BUG_ON(err);

	if (nulldisp_dev)
		platform_device_unregister(nulldisp_dev);

	platform_driver_unregister(&nulldisp_platform_driver);
}

module_init(nulldisp_init);
module_exit(nulldisp_exit);

MODULE_AUTHOR("Imagination Technologies Ltd. <gpl-support@imgtec.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual MIT/GPL");
