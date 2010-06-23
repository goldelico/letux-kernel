#ifndef _LINUX_GLAMOFB_H
#define _LINUX_GLAMOFB_H

#include <linux/fb.h>

#ifdef __KERNEL__

struct glamo_core;
struct glamofb_handle;

struct glamo_fb_platform_data {
    int width, height;

    int num_modes;
    struct fb_videomode *modes;

    struct glamo_core *core;
};

#endif

#define GLAMOFB_ENGINE_ENABLE _IOW('F', 0x1, __u32)
#define GLAMOFB_ENGINE_DISABLE _IOW('F', 0x2, __u32)
#define GLAMOFB_ENGINE_RESET _IOW('F', 0x3, __u32)

#endif
