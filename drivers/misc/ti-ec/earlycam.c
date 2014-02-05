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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>

#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <../drivers/media/platform/ti-vps/vip.h>
#include "earlycam.h"

#define EARLYCAM_IRQ_MASK  (DISPC_IRQ_VSYNC)

/* Early Camera display context */
struct earlycam_device {
	int num_displays;
	struct omap_dss_device *displays[EARLYCAM_MAX_DISPLAYS];
	int num_overlays;
	struct omap_overlay *overlays[EARLYCAM_MAX_OVLS];
	int num_managers;
	struct omap_overlay_manager *managers[EARLYCAM_MAX_MANAGERS];

	int reverse_gpio;
};

static struct earlycam_device *earlycam_dev;
static struct task_struct *main_thread;
static struct omap_overlay_info info;
static int once = 1;

static int main_fn(void *);
static int init_mmap(void);
static void earlycam_isr(void *arg, unsigned int irqstatus);

int thread_init(void)
{
	char our_thread[16] = "earlycam_thread";

	main_thread = kthread_create(main_fn, NULL, our_thread);
	if (!main_thread)
		return -ENOMEM;

	wake_up_process(main_thread);

	return 0;
}

int display_init()
{
	int ret = 0, i;
	struct omap_dss_device *dssdev = NULL;

	ret = omapdss_compat_init();
	if (ret) {
		pr_err("\nomapdss_compat_init returned error ");
		return ret;
	}

	earlycam_dev->num_displays = 0;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (!dssdev->driver) {
			pr_err("\nno driver for display: %s\n",
				   dssdev->name);
			continue;
		}

		earlycam_dev->displays[earlycam_dev->num_displays++] = dssdev;
	}

	if (earlycam_dev->num_displays == 0) {
		pr_warn("\nno displays\n");
		ret = -EINVAL;
		goto err_dss_init;
	}

	earlycam_dev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < earlycam_dev->num_overlays; i++)
		earlycam_dev->overlays[i] = omap_dss_get_overlay(i);

	earlycam_dev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < earlycam_dev->num_managers; i++)
		earlycam_dev->managers[i] = omap_dss_get_overlay_manager(i);

	return ret;

err_dss_init:
	omapdss_compat_uninit();
	return ret;
}

int display_disable(struct earlycam_setup_dispc_data data)
{
	struct omap_overlay *ovl;
	ovl = earlycam_dev->overlays[data.ovls[0].cfg.ix];

	if (ovl->is_enabled(ovl)) {
		omap_dispc_unregister_isr(earlycam_isr,
					  &info, EARLYCAM_IRQ_MASK);
		ovl->disable(ovl);
	}

	return 0;
}

static void earlycam_isr(void *arg, unsigned int irqstatus)
{
	int ret;
	struct omap_overlay *ovl;
	struct omap_overlay_info info = *((struct omap_overlay_info *)arg);

	if (!earlycam_dev)
		return;

	ovl = earlycam_dev->overlays[EARLYCAM_OVERLAY_IDX];
	if (ovl && ovl->is_enabled(ovl)) {
		ret = ovl->set_overlay_info(ovl, &info);
		if (ret)
			pr_err("set_overlay_info failed with error %d", ret);

		ret = ovl->manager->apply(ovl->manager);
		if (ret)
			pr_err("manager->apply failed with error %d", ret);
	}
}

int display_queue(struct earlycam_setup_dispc_data data)
{
	int ret = 0;
	int retry;
	struct omap_overlay *ovl;

	if (!data.ovls[0].cfg.enabled)
		return ret;

	ovl = earlycam_dev->overlays[data.ovls[0].cfg.ix];

	memset(&info, 0, sizeof(info));

	ovl->get_overlay_info(ovl, &info);
	info.paddr = data.ovls[0].ba;
	info.width = data.ovls[0].cfg.width;
	info.height = data.ovls[0].cfg.height;
	info.color_mode = data.ovls[0].cfg.color_mode;
	info.pos_x = data.ovls[0].cfg.win.x;
	info.pos_y = data.ovls[0].cfg.win.y;
	info.out_width = data.ovls[0].cfg.win.w;
	info.out_height = data.ovls[0].cfg.win.h;
	info.global_alpha = data.ovls[0].cfg.global_alpha;
	info.zorder = data.ovls[0].cfg.zorder;
	info.rotation_type = OMAP_DSS_ROT_DMA;
	info.max_x_decim = 1;
	info.max_y_decim = 1;
	info.min_x_decim = 1;
	info.min_y_decim = 1;
	info.mflag_en = 1;

	/* Rotation is not supported currently */
	info.rotation = 0;
	info.mirror = 0;
	info.screen_width = info.width;

#ifdef DEBUG
	pr_debug("%s enable=%d addr=%x width=%d\n height=%d color_mode=%d\n"
		"rotation=%d mirror=%d posx=%d posy=%d out_width = %d\n"
		"out_height=%d rotation_type=%d screen_width=%d\n",
		__func__, ovl->is_enabled(ovl), info.paddr, info.width,
		info.height, info.color_mode, info.rotation, info.mirror,
		info.pos_x, info.pos_y, info.out_width, info.out_height,
		info.rotation_type, info.screen_width);
#endif

	/* Re-try for vid pipe if unavailable */
	if (once) {
		retry = 50;
		while (ovl->is_enabled(ovl) && (retry-- > 0))
			msleep(20);

		if (ovl->is_enabled(ovl))
			return -1;

		once = 0;

		ovl->set_manager(ovl,
			earlycam_dev->managers[data.ovls[0].cfg.mgr_ix]);

		ret = ovl->set_overlay_info(ovl, &info);
		if (ret) {
			pr_err("set_overlay_info failed with error %d", ret);
			return ret;
		}

		ret = ovl->manager->apply(ovl->manager);
		if (ret)
			pr_warn("manager->apply failed with error %d", ret);

		ret = ovl->manager->set_ovl(ovl->manager);
		if (ret)
			pr_warn("manager->set_ovl failed with error %d", ret);

		ret = ovl->enable(ovl);

		omap_dispc_register_isr(earlycam_isr, &info, EARLYCAM_IRQ_MASK);
	}

	return ret;
}

int init_mmap()
{
	int i;
	struct v4l2_requestbuffers req = {0};
	req.count = EARLYCAM_VIP_NUM_BUFS;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == early_reqbufs(&req)) {
		pr_err("early_reqbufs failed");
		return -1;
	}

	for (i = 0; i < req.count; i++) {
		struct v4l2_buffer buf = {0};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (-1 == early_querybuf(&buf)) {
			pr_err("early_querybuf failed");
			return -1;
		}
	}

	return 0;
}

int capture_image(int init)
{
	struct v4l2_buffer buf = {0};
	int i;

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;

	for (i = 0; i < EARLYCAM_VIP_NUM_BUFS; i++) {
		if (-1 == early_qbuf(&buf)) {
			pr_err("early_qbuf failed");
			return -1;
		}
		buf.index++;
	}
	buf.index = 0;
	if ((-1 == (early_streamon(buf.type) && init))) {
		pr_err("early_streamon failed");
		return -1;
	}

	for (i = 0; i < EARLYCAM_VIP_NUM_BUFS; i++) {
		if (-1 == early_dqbuf(&buf)) {
			pr_err("early_dqbuf failed");
			return -1;
		}
		buf.index++;
	}

	return 0;
}


int main_fn(void *arg)
{
	int i;
	int ret;
	int cam_init = 1;
	int val;

	/* need base address for in-page offset */
	struct earlycam_setup_dispc_data comp = {
		.num_mgrs = 1,
		.num_ovls = 1,
		.ovls[0].cfg = {
			.ix = EARLYCAM_OVERLAY_IDX,
			.mgr_ix = EARLYCAM_MANAGER_IDX,
			.width = EARLYCAM_VIP_WIDTH,
			.win.w = EARLYCAM_WINDOW_WIDTH,
			.crop.w = EARLYCAM_VIP_WIDTH,
			.height = EARLYCAM_VIP_HEIGHT,
			.win.h = EARLYCAM_WINDOW_HEIGHT,
			.crop.h = EARLYCAM_VIP_HEIGHT,
			.color_mode = EARLYCAM_WINDOW_FORMAT,
			.global_alpha = (u8) EARLYCAM_WINDOW_GLOBALALPHA,
			.zorder = EARLYCAM_WINDOW_ZORDER,
			.enabled = (u8) 1,
		},
		.ovls[0].ba = (u32) dma_addr_global_complete,
	};

	if (!early_sensor_detect()) {
		pr_err("early_sensor_detect failed with error %d",
			 ret);
		return ret;
	}

	ret = display_init();
	if (ret) {
		pr_err("display_init failed with error %d", ret);
		return ret;
	}

	ret = early_vip_open();
	if (ret) {
		pr_err("early_vip_open failed with error %d", ret);
		return ret;
	}

	ret = init_mmap();
	if (ret) {
		pr_err("init_mmap failed with error %d", ret);
		return ret;
	}

	while (1) {
		while ((val =
			   gpio_get_value_cansleep(
			   earlycam_dev->reverse_gpio)) ==
			   1) {
			/* Spin inside this loop, sleeping for 100 mS
			  * everytime until the user presses the gpio.
			  * This should be replaced by interrupt based
			  * mechanism to avoid waking up the cpu
			  * frequently
			  */
			if (!once) {
				display_disable(comp);
				early_release();
				/* Set a special code for init flag to signal
				  * that the init apis have to be called again
				  * after gpio release
				  */
				cam_init = 2;
				/* Signal to the display that it needs to
				  * re-acquire the pipes and enable
				  * interrupts again next time when gpio
				  * is pressed
				  */
				once = 1;
			}
			msleep(100);
		}

		/* So we got a gpio press, start by initializing camera first */
		if (cam_init == 2) {
			ret = early_vip_open();
			if (ret) {
				pr_err("early_vip_open failed with error %d",
					ret);
				return ret;
			}

			ret = init_mmap();
			if (ret) {
				pr_err("init_mmap failed with error %d", ret);
				return ret;
			}

			/* okay now reset this flag to 1 inorder to signal to
			 * capture_image that it needs to call stream ON
			 */
			cam_init = 1;
		}

		ret = capture_image(cam_init);
		if (ret) {
			pr_err("capture_image failed with error %d", ret);
			return ret;
	    }
		cam_init = 0;

	    comp.ovls[0].ba = (u32) dma_addr_global_complete;
		if (comp.ovls[0].ba != 0) {
			ret = display_queue(comp);
			if (ret)
				pr_err("display_queue failed with error %d",
					   ret);
		}
		msleep(33);
	}

	ret = early_release();
	if (ret) {
		pr_err("early_release failed with error %d", ret);
		return ret;
	}

	ret = display_disable(comp);
	if (ret) {
		pr_err("display_disable failed with error %d", ret);
		return ret;
	}

	return ret;
}

static int ealycam_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = pdev->dev.of_node;
	int gpio_count;
	int gpio;

	/* Create early cam device structure */
	earlycam_dev = devm_kzalloc(&pdev->dev, sizeof(struct earlycam_device),
			   GFP_KERNEL);
	if (earlycam_dev == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	gpio_count = of_gpio_count(node);
	if (gpio_count != 1) {
		pr_err("Wrong number of GPIOs for early cam, exiting..\n");
		return -1;
	}

	gpio = of_get_gpio(node, 0);
	if (gpio_is_valid(gpio)) {
		earlycam_dev->reverse_gpio = gpio;
	} else {
		earlycam_dev->reverse_gpio = 0;
		pr_err("failed to parse reverse gpio\n");
		return -1;
	}

	ret = devm_gpio_request_one(&pdev->dev, earlycam_dev->reverse_gpio,
			  GPIOF_IN, "reverse_gpio");
	if (ret) {
		pr_err("failed to request reverse gpio %d\n", ret);
		return ret;
	}

	ret = thread_init();
	if (ret)
		pr_err("Failed to initialize early camera module");

	return ret;
}

static int earlycam_remove(struct platform_device *pdev)
{
	omapdss_compat_uninit();

	kthread_stop(main_thread);

	return 0;
}

static const struct of_device_id earlycam_of_match[] = {
	{.compatible = "ti,earlycam_mpu", },
	{ },
};


static struct platform_driver earlycam_driver = {
	.probe = ealycam_probe,
	.remove = earlycam_remove,
	.driver = { .name = "earlycam",
				.of_match_table = earlycam_of_match
	}
};

static int __init earlycam_init(void)
{
	return platform_driver_register(&earlycam_driver);
}

static void __exit earlycam_exit(void)
{
	platform_driver_unregister(&earlycam_driver);
}

MODULE_DESCRIPTION("TI Early Camera Driver");
MODULE_AUTHOR("Sundar Raman <sunds@ti.com>, Arthur Philpott <arthur.philpott@ti.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_init(earlycam_init);
module_exit(earlycam_exit);
