// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2026 Andreas Kemnade
// simple display tester displaying a qr code

// compile:
//   apt-get install libdrm-dev libqrencode-dev
//   gcc -o qrdrm qrdrm.c -I/usr/include/libdrm -ldrm -lqrencode
//
// usage:
//   qrdrm "Hello World"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <qrencode.h>

struct drm_buf {
	uint32_t handle;
	uint32_t fb;
	uint32_t size;
	uint32_t pitch;
	void *map;
	uint32_t width, height;
};

static int pageflip_done = 0;

static void die(const char *s) {
	perror(s);
	exit(1);
}

static bool check_connector(int fd, drmModeResPtr res,
							drmModeConnectorPtr conn, uint32_t *conn_id,
							drmModeModeInfo *mode, uint32_t *crtc_id)
{
	int i;
	drmModeEncoderPtr enc = NULL;
	if (conn->connection != DRM_MODE_CONNECTED)
		return false;

	if (conn->count_modes <= 0)
		return false;

	if (!conn->encoder_id) {
		if (conn->count_encoders)
			conn->encoder_id = conn->encoders[0];
	}

	if (!conn->encoder_id)
		return false;

	*conn_id = conn->connector_id;
	*mode = conn->modes[0];
	enc = drmModeGetEncoder(fd, conn->encoder_id);
	if (!enc)
		return false;

	*crtc_id = enc->crtc_id;
	if (!*crtc_id) {
		for (i = 0; i < res->count_crtcs; i ++) {
			if (enc->possible_crtcs & (1 << i))
				*crtc_id = res->crtcs[i];
		}
	}
	drmModeFreeEncoder(enc);
	if (!*crtc_id)
		return false;

	return true;
}

static bool check_drm_dev(int fd, uint32_t *conn_id, drmModeModeInfo *mode, uint32_t *crtc_id)
{
	int i;
	bool found = false;
	drmModeConnectorPtr conn = NULL;

	drmModeResPtr res = drmModeGetResources(fd);
	if (!res)
		return false;

	for (i = 0; i < res->count_connectors; ++i) {
		conn = drmModeGetConnector(fd, res->connectors[i]);
		if (!conn)
			continue;

		found = check_connector(fd, res, conn, conn_id, mode, crtc_id);
		drmModeFreeConnector(conn);

		if (found)
			break;
	}

	drmModeFreeResources(res);
	return found;
}

static void create_dumb_buffer(int drm_fd, struct drm_buf *b, uint32_t width, uint32_t height, uint32_t bpp)
{
	struct drm_mode_create_dumb creq = {
		.width = width,
		.height = height,
		.bpp = bpp
	};
	uint32_t handles[4] = {0};
	uint32_t pitches[4] = {0};
	uint32_t offsets[4] = {0};
	int ret;

	creq.width = width;
	creq.height = height;
	creq.bpp = bpp;
	if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0)
		die("CREATE_DUMB");

	b->handle = creq.handle;
	b->pitch = creq.pitch;
	b->size = creq.size;
	b->width = width; b->height = height;
	handles[0] = b->handle;
	pitches[0] = b->pitch;

	/* create framebuffer object */
	ret = drmModeAddFB2(drm_fd, width, height, DRM_FORMAT_XRGB8888,
						handles, pitches, offsets, &b->fb, 0);
	if (ret)
		die("drmModeAddFB2");

	/* map it */
	struct drm_mode_map_dumb mreq = { .handle = b->handle};
	if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0)
		die("MAP_DUMB");

	b->map = mmap(0, b->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, mreq.offset);
	if (b->map == MAP_FAILED)
		die("mmap");
}

static void destroy_dumb_buffer(int drm_fd, struct drm_buf *b)
{
	if (b->fb)
		drmModeRmFB(drm_fd, b->fb);

	if (b->map && b->size)
		munmap(b->map, b->size);

	if (b->handle) {
		struct drm_mode_destroy_dumb dreq = {
			.handle = b->handle
		};

		ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
}

static void draw_pattern(struct drm_buf *b, int frame) {
	uint32_t *p = b->map;
	for (uint32_t y = 0; y < b->height; ++y) {
		for (uint32_t x = 0; x < b->width; ++x) {
			uint8_t r = (x + frame*4) & 0xff;
			uint8_t g = (y + frame*2) & 0xff;
			uint8_t bb = (x + y + frame) & 0xff;
			uint32_t pix = (r << 16) | (g << 8) | bb;
			p[y * (b->pitch/4) + x] = pix;
		}
	}
}

static void page_flip_handler(int fd, unsigned int sequence, unsigned int tv_sec, unsigned int tv_usec, void *user_data) {
	pageflip_done = 1;
}

static void draw_qr(struct drm_buf *buf, QRcode *qr)
{
	uint32_t *base = (uint32_t *) buf->map;
	uint32_t scale;
	uint32_t qr_left, qr_top, x, y, i;
	uint8_t *qr_data = qr->data;
	uint32_t qr_final_width;

	scale = buf->width / qr->width / 2;
	if (buf->height < buf->width)
		scale = buf->height / qr->width / 2;
	qr_final_width = qr->width * scale;

	qr_left = (buf->width - qr_final_width) / 2;
	qr_top = (buf->height - qr_final_width) / 2;

	base = base + buf->pitch / 4 * (qr_top - 2 *scale) + qr_left - 2 * scale;
	for (y = 0; y < qr_final_width + 4 * scale ; y ++)
		{
		for(x = 0; x < qr_final_width + 4 * scale; x++) {
			base[x] = 0xffffff;
		}
		base += buf->pitch / 4;
		}

	base = buf->map;
	base = base + buf->pitch / 4 * qr_top + qr_left;

	for(y = 0; y < qr->width; y++) {
		for (x = 0; x < qr->width; x++) {
			for(i = 0; i < scale; i ++) {
				base[x * scale + i] = (*qr_data) & 1 ? 0: 0xffffff;
			}
			qr_data++;
		}
		for(i = 1; i < scale; i++) {
			memcpy(base + i * buf->pitch / 4, base, sizeof(*base) * qr->width * scale * 4);
		}
		base += scale * buf->pitch / 4;
	}

}

int main(int argc, char **argv) {
	int i;
	bool found;
	int drm_fd;
	static drmModeModeInfo mode;
	static struct drm_buf bufs;
	static uint32_t conn_id = 0, crtc_id = 0;

	if(!argv[1]) {
		fprintf(stderr, "usage: %s 'text string'\n", argv[0]);
		exit(1);
	}

	for(i = 0; i < 32; i++) {
		char buf[64];

		snprintf(buf, sizeof(buf), "/dev/dri/card%d", i);
		drm_fd = open(buf, O_RDWR);
		if (drm_fd < 0)
			continue;

		drmVersionPtr version = drmGetVersion(drm_fd);
		if (!version)
			die("version");

		printf("Name: %s\n", version->name);

		found = check_drm_dev(drm_fd, &conn_id, &mode, &crtc_id);
		if (found)
			break;

		close(drm_fd);
		drm_fd = -1;
	}
	if (!found) {
		die("no suitable output found");
	}

	uint32_t width = mode.hdisplay;
	uint32_t height = mode.vdisplay;

	create_dumb_buffer(drm_fd, &bufs, width, height, 32);

	/* initial draw */

	QRcode *qr =  QRcode_encodeString(argv[1], 0, QR_ECLEVEL_Q, QR_MODE_8, 1);

	if(!qr)
		return 0;

	draw_qr(&bufs, qr);

	if (drmModeSetCrtc(drm_fd, crtc_id, bufs.fb, 0, 0, &conn_id, 1, &mode))
		die("drmModeSetCrtc");

	sleep(5);

	/* cleanup */
	destroy_dumb_buffer(drm_fd, &bufs);
	close(drm_fd);
	return 0;
}
