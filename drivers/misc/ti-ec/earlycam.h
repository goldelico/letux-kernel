/*
 * TI Early Camera driver
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Sundar Raman <sunds@ti.com>
 *          Arthur Philpott <arthur.philpott@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <video/omapdss.h>

#define EARLYCAM_MAX_OVLS	4
#define EARLYCAM_MAX_DISPLAYS	10
#define EARLYCAM_MAX_MANAGERS	4

/* Early Camera pipeline and manager selections */
#define EARLYCAM_OVERLAY_IDX 3
#define EARLYCAM_MANAGER_IDX 0

/* Early Camera display window selections */
#define EARLYCAM_WINDOW_FORMAT OMAP_DSS_COLOR_YUV2
#define EARLYCAM_WINDOW_GLOBALALPHA 255
#define EARLYCAM_WINDOW_ZORDER 3
#define EARLYCAM_WINDOW_POSX 0
#define EARLYCAM_WINDOW_POSY 0
#define EARLYCAM_WINDOW_WIDTH 720
#define EARLYCAM_WINDOW_HEIGHT 480

/* Early Camera parameter selections */
#define EARLYCAM_VIP_WIDTH 1280
#define EARLYCAM_VIP_HEIGHT 720
#define EARLYCAM_VIP_NUM_BUFS 6

#define EARLYCAM_LOOP_LENGTH 270

/* standard rectangle */
struct earlycam_rect_t {
	__s32 x;	/* left */
	__s32 y;	/* top */
	__u32 w;	/* width */
	__u32 h;	/* height */
} __aligned(4);

struct earlycam_ovl_cfg {
	__u16 width;	/* buffer width */
	__u16 height;	/* buffer height */
	__u32 stride;	/* buffer stride */

	enum omap_color_mode color_mode;
	__u8 pre_mult_alpha;	/* bool */
	__u8 global_alpha;	/* 0..255 */
	__u8 rotation;		/* 0..3 (*90 degrees clockwise) */
	__u8 mirror;	/* left-to-right: mirroring is applied after rotation */

	struct earlycam_rect_t win;		/* output window - on display */
	struct earlycam_rect_t crop;	/* crop window - in source buffer */

	__u8 ix;	/* ovl index same as sysfs/overlay# */
	__u8 zorder;	/* 0..3 */
	__u8 enabled;	/* bool */
	__u8 zonly;	/* only set zorder and enabled bit */
	__u8 mgr_ix;	/* mgr index */

	bool force_1d;
	bool mflag_en; /* mflag for the overlay */
} __aligned(4);

enum earlycam_buffer_type {
	OMAP_DSS_BUFTYPE_SDMA,
	OMAP_DSS_BUFTYPE_TILER_8BIT,
	OMAP_DSS_BUFTYPE_TILER_16BIT,
	OMAP_DSS_BUFTYPE_TILER_32BIT,
	OMAP_DSS_BUFTYPE_TILER_PAGE,
};

enum earlycam_buffer_addressing_type {
	OMAP_DSS_BUFADDR_DIRECT,	/* using direct addresses */
	OMAP_DSS_BUFADDR_BYTYPE,	/* using buffer types */
	OMAP_DSS_BUFADDR_ION,		/* using ion handle(s) */
	OMAP_DSS_BUFADDR_GRALLOC,	/* using gralloc handle */
};

struct earlycam_ovl_info {
	struct earlycam_ovl_cfg cfg;

	enum earlycam_buffer_addressing_type addressing;

	union {
		/* user-space interfaces */
		struct {
			void *address;		/* main buffer address */
			void *uv_address;	/* uv buffer */
		};

		/*
		 * For DSSCIOC_CHECK_OVL we allow specifying just the
		 * type of each buffer. This is used if we need to
		 * check whether DSS will be able to display a buffer
		 * if using a particular memory type before spending
		 * time to map/copy the buffer into that type of
		 * memory.
		 */
		struct {
			enum earlycam_buffer_type ba_type;
			enum earlycam_buffer_type uv_type;
		};

		/* kernel-space interfaces */

		/*
		 * for fbmem, highest 4-bits of address is fb index,
		 * rest of the bits are the offset
		 */
		struct {
			__u32 ba;	/* base address or index */
			__u32 uv;	/* uv address */
		};
	};
};

/*
 * This structure is used to set up the entire DISPC (all managers),
 * and is analogous to dsscomp_setup_mgr_data.
 *
 * Additional features:
 * - all overlays that were specified in a prior use of this
 * structure, and are no longer specified, will be disabled.
 * - 1D buffers under 4M will be mapped into TILER1D.
 *
 * Limitations:
 * - only DISPLAY mode is supported (DISPLAY and APPLY bits will
 *   automatically be set)
 * - getting a sync object is not supported.
 */
struct earlycam_setup_dispc_data {
	__u16 num_ovls;		/* # of overlays used in the composition */
	__u16 num_mgrs;		/* # of managers used in the composition */

	struct earlycam_ovl_info ovls[5]; /* up to 5 overlays to set up */
};
