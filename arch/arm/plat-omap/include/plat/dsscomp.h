#ifndef _ARCH_ARM_PLAT_OMAP_DSSCOMP_H
#define _ARCH_ARM_PLAT_OMAP_DSSCOMP_H

#include <video/omapdss.h>

/* queuing operations */
struct dsscomp;
struct dsscomp *dsscomp_new(struct omap_overlay_manager *mgr);
u32 dsscomp_get_ovls(struct dsscomp *comp);
int dsscomp_set_ovl(struct dsscomp *comp, struct dss2_ovl_info *ovl);
int dsscomp_get_ovl(struct dsscomp *comp, u32 ix, struct dss2_ovl_info *ovl);
int dsscomp_set_mgr(struct dsscomp *comp, struct dss2_mgr_info *mgr);
int dsscomp_get_mgr(struct dsscomp *comp, struct dss2_mgr_info *mgr);
int dsscomp_setup(struct dsscomp *comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win);
int dsscomp_delayed_apply(struct dsscomp *comp);
void dsscomp_drop(struct dsscomp *c);

struct tiler_pa_info;
int dsscomp_gralloc_queue(struct dsscomp_setup_dispc_data *d,
			struct tiler_pa_info **pas,
			bool early_callback,
			void (*cb_fn)(void *, int), void *cb_arg);
#endif
