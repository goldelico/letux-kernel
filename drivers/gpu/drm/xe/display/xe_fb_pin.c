// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include <drm/ttm/ttm_bo.h>

#include "intel_display_types.h"
#include "intel_dpt.h"
#include "intel_fb.h"
#include "intel_fb_pin.h"
#include "xe_bo.h"
#include "xe_device.h"
#include "xe_ggtt.h"
#include "xe_pm.h"

static void
write_dpt_rotated(struct xe_bo *bo, struct iosys_map *map, u32 *dpt_ofs, u32 bo_ofs,
		  u32 width, u32 height, u32 src_stride, u32 dst_stride)
{
	struct xe_device *xe = xe_bo_device(bo);
	struct xe_ggtt *ggtt = xe_device_get_root_tile(xe)->mem.ggtt;
	u32 column, row;

	/* TODO: Maybe rewrite so we can traverse the bo addresses sequentially,
	 * by writing dpt/ggtt in a different order?
	 */

	for (column = 0; column < width; column++) {
		u32 src_idx = src_stride * (height - 1) + column + bo_ofs;

		for (row = 0; row < height; row++) {
			u64 pte = ggtt->pt_ops->pte_encode_bo(bo, src_idx * XE_PAGE_SIZE,
							      xe->pat.idx[XE_CACHE_NONE]);

			iosys_map_wr(map, *dpt_ofs, u64, pte);
			*dpt_ofs += 8;
			src_idx -= src_stride;
		}

		/* The DE ignores the PTEs for the padding tiles */
		*dpt_ofs += (dst_stride - height) * 8;
	}

	/* Align to next page */
	*dpt_ofs = ALIGN(*dpt_ofs, 4096);
}

static void
write_dpt_remapped(struct xe_bo *bo, struct iosys_map *map, u32 *dpt_ofs,
		   u32 bo_ofs, u32 width, u32 height, u32 src_stride,
		   u32 dst_stride)
{
	struct xe_device *xe = xe_bo_device(bo);
	struct xe_ggtt *ggtt = xe_device_get_root_tile(xe)->mem.ggtt;
	u64 (*pte_encode_bo)(struct xe_bo *bo, u64 bo_offset, u16 pat_index)
		= ggtt->pt_ops->pte_encode_bo;
	u32 column, row;

	for (row = 0; row < height; row++) {
		u32 src_idx = src_stride * row + bo_ofs;

		for (column = 0; column < width; column++) {
			iosys_map_wr(map, *dpt_ofs, u64,
				     pte_encode_bo(bo, src_idx * XE_PAGE_SIZE,
				     xe->pat.idx[XE_CACHE_NONE]));

			*dpt_ofs += 8;
			src_idx++;
		}

		/* The DE ignores the PTEs for the padding tiles */
		*dpt_ofs += (dst_stride - width) * 8;
	}

	/* Align to next page */
	*dpt_ofs = ALIGN(*dpt_ofs, 4096);
}

static int __xe_pin_fb_vma_dpt(const struct intel_framebuffer *fb,
			       const struct i915_gtt_view *view,
			       struct i915_vma *vma)
{
	struct xe_device *xe = to_xe_device(fb->base.dev);
	struct xe_tile *tile0 = xe_device_get_root_tile(xe);
	struct xe_ggtt *ggtt = tile0->mem.ggtt;
	struct xe_bo *bo = intel_fb_obj(&fb->base), *dpt;
	u32 dpt_size, size = bo->ttm.base.size;

	if (view->type == I915_GTT_VIEW_NORMAL)
		dpt_size = ALIGN(size / XE_PAGE_SIZE * 8, XE_PAGE_SIZE);
	else if (view->type == I915_GTT_VIEW_REMAPPED)
		dpt_size = ALIGN(intel_remapped_info_size(&fb->remapped_view.gtt.remapped) * 8,
				 XE_PAGE_SIZE);
	else
		/* display uses 4K tiles instead of bytes here, convert to entries.. */
		dpt_size = ALIGN(intel_rotation_info_size(&view->rotated) * 8,
				 XE_PAGE_SIZE);

	if (IS_DGFX(xe))
		dpt = xe_bo_create_pin_map(xe, tile0, NULL, dpt_size,
					   ttm_bo_type_kernel,
					   XE_BO_FLAG_VRAM0 |
					   XE_BO_FLAG_GGTT |
					   XE_BO_FLAG_PAGETABLE);
	else
		dpt = xe_bo_create_pin_map(xe, tile0, NULL, dpt_size,
					   ttm_bo_type_kernel,
					   XE_BO_FLAG_STOLEN |
					   XE_BO_FLAG_GGTT |
					   XE_BO_FLAG_PAGETABLE);
	if (IS_ERR(dpt))
		dpt = xe_bo_create_pin_map(xe, tile0, NULL, dpt_size,
					   ttm_bo_type_kernel,
					   XE_BO_FLAG_SYSTEM |
					   XE_BO_FLAG_GGTT |
					   XE_BO_FLAG_PAGETABLE);
	if (IS_ERR(dpt))
		return PTR_ERR(dpt);

	if (view->type == I915_GTT_VIEW_NORMAL) {
		u32 x;

		for (x = 0; x < size / XE_PAGE_SIZE; x++) {
			u64 pte = ggtt->pt_ops->pte_encode_bo(bo, x * XE_PAGE_SIZE,
							      xe->pat.idx[XE_CACHE_NONE]);

			iosys_map_wr(&dpt->vmap, x * 8, u64, pte);
		}
	} else if (view->type == I915_GTT_VIEW_REMAPPED) {
		const struct intel_remapped_info *remap_info = &view->remapped;
		u32 i, dpt_ofs = 0;

		for (i = 0; i < ARRAY_SIZE(remap_info->plane); i++)
			write_dpt_remapped(bo, &dpt->vmap, &dpt_ofs,
					   remap_info->plane[i].offset,
					   remap_info->plane[i].width,
					   remap_info->plane[i].height,
					   remap_info->plane[i].src_stride,
					   remap_info->plane[i].dst_stride);

	} else {
		const struct intel_rotation_info *rot_info = &view->rotated;
		u32 i, dpt_ofs = 0;

		for (i = 0; i < ARRAY_SIZE(rot_info->plane); i++)
			write_dpt_rotated(bo, &dpt->vmap, &dpt_ofs,
					  rot_info->plane[i].offset,
					  rot_info->plane[i].width,
					  rot_info->plane[i].height,
					  rot_info->plane[i].src_stride,
					  rot_info->plane[i].dst_stride);
	}

	vma->dpt = dpt;
	vma->node = dpt->ggtt_node[tile0->id];

	/* Ensure DPT writes are flushed */
	xe_device_l2_flush(xe);
	return 0;
}

static void
write_ggtt_rotated(struct xe_bo *bo, struct xe_ggtt *ggtt, u32 *ggtt_ofs, u32 bo_ofs,
		   u32 width, u32 height, u32 src_stride, u32 dst_stride)
{
	struct xe_device *xe = xe_bo_device(bo);
	u32 column, row;

	for (column = 0; column < width; column++) {
		u32 src_idx = src_stride * (height - 1) + column + bo_ofs;

		for (row = 0; row < height; row++) {
			u64 pte = ggtt->pt_ops->pte_encode_bo(bo, src_idx * XE_PAGE_SIZE,
							      xe->pat.idx[XE_CACHE_NONE]);

			ggtt->pt_ops->ggtt_set_pte(ggtt, *ggtt_ofs, pte);
			*ggtt_ofs += XE_PAGE_SIZE;
			src_idx -= src_stride;
		}

		/* The DE ignores the PTEs for the padding tiles */
		*ggtt_ofs += (dst_stride - height) * XE_PAGE_SIZE;
	}
}

static int __xe_pin_fb_vma_ggtt(const struct intel_framebuffer *fb,
				const struct i915_gtt_view *view,
				struct i915_vma *vma)
{
	struct xe_bo *bo = intel_fb_obj(&fb->base);
	struct xe_device *xe = to_xe_device(fb->base.dev);
	struct xe_ggtt *ggtt = xe_device_get_root_tile(xe)->mem.ggtt;
	u32 align;
	int ret;

	/* TODO: Consider sharing framebuffer mapping?
	 * embed i915_vma inside intel_framebuffer
	 */
	xe_pm_runtime_get_noresume(tile_to_xe(ggtt->tile));
	ret = mutex_lock_interruptible(&ggtt->lock);
	if (ret)
		goto out;

	align = XE_PAGE_SIZE;
	if (xe_bo_is_vram(bo) && ggtt->flags & XE_GGTT_FLAGS_64K)
		align = max_t(u32, align, SZ_64K);

	if (bo->ggtt_node[ggtt->tile->id] && view->type == I915_GTT_VIEW_NORMAL) {
		vma->node = bo->ggtt_node[ggtt->tile->id];
	} else if (view->type == I915_GTT_VIEW_NORMAL) {
		u32 x, size = bo->ttm.base.size;

		vma->node = xe_ggtt_node_init(ggtt);
		if (IS_ERR(vma->node)) {
			ret = PTR_ERR(vma->node);
			goto out_unlock;
		}

		ret = xe_ggtt_node_insert_locked(vma->node, size, align, 0);
		if (ret) {
			xe_ggtt_node_fini(vma->node);
			goto out_unlock;
		}

		for (x = 0; x < size; x += XE_PAGE_SIZE) {
			u64 pte = ggtt->pt_ops->pte_encode_bo(bo, x,
							      xe->pat.idx[XE_CACHE_NONE]);

			ggtt->pt_ops->ggtt_set_pte(ggtt, vma->node->base.start + x, pte);
		}
	} else {
		u32 i, ggtt_ofs;
		const struct intel_rotation_info *rot_info = &view->rotated;

		/* display seems to use tiles instead of bytes here, so convert it back.. */
		u32 size = intel_rotation_info_size(rot_info) * XE_PAGE_SIZE;

		vma->node = xe_ggtt_node_init(ggtt);
		if (IS_ERR(vma->node)) {
			ret = PTR_ERR(vma->node);
			goto out_unlock;
		}

		ret = xe_ggtt_node_insert_locked(vma->node, size, align, 0);
		if (ret) {
			xe_ggtt_node_fini(vma->node);
			goto out_unlock;
		}

		ggtt_ofs = vma->node->base.start;

		for (i = 0; i < ARRAY_SIZE(rot_info->plane); i++)
			write_ggtt_rotated(bo, ggtt, &ggtt_ofs,
					   rot_info->plane[i].offset,
					   rot_info->plane[i].width,
					   rot_info->plane[i].height,
					   rot_info->plane[i].src_stride,
					   rot_info->plane[i].dst_stride);
	}

out_unlock:
	mutex_unlock(&ggtt->lock);
out:
	xe_pm_runtime_put(tile_to_xe(ggtt->tile));
	return ret;
}

static struct i915_vma *__xe_pin_fb_vma(const struct intel_framebuffer *fb,
					const struct i915_gtt_view *view)
{
	struct drm_device *dev = fb->base.dev;
	struct xe_device *xe = to_xe_device(dev);
	struct i915_vma *vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	struct xe_bo *bo = intel_fb_obj(&fb->base);
	int ret;

	if (!vma)
		return ERR_PTR(-ENODEV);

	if (IS_DGFX(to_xe_device(bo->ttm.base.dev)) &&
	    intel_fb_rc_ccs_cc_plane(&fb->base) >= 0 &&
	    !(bo->flags & XE_BO_FLAG_NEEDS_CPU_ACCESS)) {
		struct xe_tile *tile = xe_device_get_root_tile(xe);

		/*
		 * If we need to able to access the clear-color value stored in
		 * the buffer, then we require that such buffers are also CPU
		 * accessible.  This is important on small-bar systems where
		 * only some subset of VRAM is CPU accessible.
		 */
		if (tile->mem.vram.io_size < tile->mem.vram.usable_size) {
			ret = -EINVAL;
			goto err;
		}
	}

	/*
	 * Pin the framebuffer, we can't use xe_bo_(un)pin functions as the
	 * assumptions are incorrect for framebuffers
	 */
	ret = ttm_bo_reserve(&bo->ttm, false, false, NULL);
	if (ret)
		goto err;

	if (IS_DGFX(xe))
		ret = xe_bo_migrate(bo, XE_PL_VRAM0);
	else
		ret = xe_bo_validate(bo, NULL, true);
	if (!ret)
		ttm_bo_pin(&bo->ttm);
	ttm_bo_unreserve(&bo->ttm);
	if (ret)
		goto err;

	vma->bo = bo;
	if (intel_fb_uses_dpt(&fb->base))
		ret = __xe_pin_fb_vma_dpt(fb, view, vma);
	else
		ret = __xe_pin_fb_vma_ggtt(fb, view, vma);
	if (ret)
		goto err_unpin;

	return vma;

err_unpin:
	ttm_bo_reserve(&bo->ttm, false, false, NULL);
	ttm_bo_unpin(&bo->ttm);
	ttm_bo_unreserve(&bo->ttm);
err:
	kfree(vma);
	return ERR_PTR(ret);
}

static void __xe_unpin_fb_vma(struct i915_vma *vma)
{
	u8 tile_id = vma->node->ggtt->tile->id;

	if (vma->dpt)
		xe_bo_unpin_map_no_vm(vma->dpt);
	else if (!xe_ggtt_node_allocated(vma->bo->ggtt_node[tile_id]) ||
		 vma->bo->ggtt_node[tile_id]->base.start != vma->node->base.start)
		xe_ggtt_node_remove(vma->node, false);

	ttm_bo_reserve(&vma->bo->ttm, false, false, NULL);
	ttm_bo_unpin(&vma->bo->ttm);
	ttm_bo_unreserve(&vma->bo->ttm);
	kfree(vma);
}

struct i915_vma *
intel_fb_pin_to_ggtt(const struct drm_framebuffer *fb,
		     const struct i915_gtt_view *view,
		     unsigned int alignment,
		     unsigned int phys_alignment,
		     bool uses_fence,
		     unsigned long *out_flags)
{
	*out_flags = 0;

	return __xe_pin_fb_vma(to_intel_framebuffer(fb), view);
}

void intel_fb_unpin_vma(struct i915_vma *vma, unsigned long flags)
{
	__xe_unpin_fb_vma(vma);
}

int intel_plane_pin_fb(struct intel_plane_state *plane_state)
{
	struct drm_framebuffer *fb = plane_state->hw.fb;
	struct xe_bo *bo = intel_fb_obj(fb);
	struct i915_vma *vma;

	/* We reject creating !SCANOUT fb's, so this is weird.. */
	drm_WARN_ON(bo->ttm.base.dev, !(bo->flags & XE_BO_FLAG_SCANOUT));

	vma = __xe_pin_fb_vma(to_intel_framebuffer(fb), &plane_state->view.gtt);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	plane_state->ggtt_vma = vma;
	return 0;
}

void intel_plane_unpin_fb(struct intel_plane_state *old_plane_state)
{
	__xe_unpin_fb_vma(old_plane_state->ggtt_vma);
	old_plane_state->ggtt_vma = NULL;
}

/*
 * For Xe introduce dummy intel_dpt_create which just return NULL,
 * intel_dpt_destroy which does nothing, and fake intel_dpt_ofsset returning 0;
 */
struct i915_address_space *intel_dpt_create(struct intel_framebuffer *fb)
{
	return NULL;
}

void intel_dpt_destroy(struct i915_address_space *vm)
{
	return;
}

u64 intel_dpt_offset(struct i915_vma *dpt_vma)
{
	return 0;
}
