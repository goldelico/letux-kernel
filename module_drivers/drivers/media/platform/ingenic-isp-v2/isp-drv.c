#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>

#include "isp-drv.h"


static const struct of_device_id ingenic_isp_dt_match[];

static int ingenic_isp_parse_dt(struct ispcam_device * ispcam)
{
	struct device *dev = ispcam->dev;
	struct device_node *node = NULL;
	struct v4l2_async_notifier *notifier = &ispcam->notifier;
	struct platform_device *pdev = NULL;
	struct v4l2_async_subdev *asd = NULL;
	const char *compatible = NULL;
	unsigned int cplen = 0;
	int ret = 0;


	for_each_child_of_node(dev->of_node, node) {
		compatible = of_get_property(node, "compatible", &cplen);

		if(!compatible && (cplen == 0)) {
			continue;
		}
		if(!of_device_is_available(node)) {
			continue;
		}
		pdev = of_platform_device_create(node, NULL,dev);
		if(!pdev) {
			printk("failed to populate_platform_device, node->full_name");
		}

		ispcam->subdevs[ispcam->subdev_num++] = pdev;
		if(ispcam->subdev_num > MAX_SUB_DEVS) {
			printk("too many sub platform devices!\n");
			return -EINVAL;
		}
	}


	printk("start parse remote-endpoint!\n");
	asd = devm_kzalloc(dev, MAX_ASYNC_SUBDEVS * sizeof(*notifier->sd->asd), GFP_KERNEL);

	/* 解析endpoint, 远程端点，建立关系, 在下面注册回调，等注册完成通知.创建video_device.*/

	while(
		(node = of_graph_get_next_endpoint(dev->of_node, node))) {
		ispcam->isd[ispcam->asd_num].asd = *asd;
		ispcam->isd[ispcam->asd_num].index = ispcam->asd_num;

		/*TODO: 构建与vep对应的数据结构，存储从dts中传入的数据.*/
		ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &ispcam->vep[ispcam->asd_num]);
		if(ret < 0) {
			of_node_put(node);
			return -EINVAL;
		}
		ispcam->isd[ispcam->asd_num].bus_type = ispcam->vep[ispcam->asd_num].bus_type;
		ispcam->isd[ispcam->asd_num].parallel = ispcam->vep[ispcam->asd_num].bus.parallel;
		ispcam->isd[ispcam->asd_num].mipi_csi2 = ispcam->vep[ispcam->asd_num].bus.mipi_csi2;


		asd->match.fwnode = fwnode_graph_get_remote_port_parent(of_fwnode_handle(node));
		asd->match_type = V4L2_ASYNC_MATCH_FWNODE;

		of_node_put(node);

		v4l2_async_notifier_init(&ispcam->notifier);

		ret = v4l2_async_notifier_add_subdev(&ispcam->notifier, asd);
		if (ret) {
			fwnode_handle_put(asd->match.fwnode);
			return ret;
		}

		fwnode_handle_put(asd->match.fwnode);
		ispcam->asd_num ++;
	}

#if 0

	printk("dump subdevs ------------ pdevs:\n");
	for(i = 0; i < ispcam->subdev_num; i++) {
		printk("---sudev: %d, name: %s\n", i, ispcam->subdevs[i]->name);
	}

	printk("dump async subdevs!\n");
	/*dump of parse subdevs.*/
	for(i = 0; i < ispcam->asd_num; i++) {
		asd = &ispcam->isd[i].asd;
		printk("asd: %d, asd->match.of.node->full_name: %s\n", i, asd->match.fwnode->dev->of_node->full_name);
	}

#endif
	return ret;
}

static int ispcam_asd_notifier_bound(struct v4l2_async_notifier *async,
                                     struct v4l2_subdev *subdev,
                                     struct v4l2_async_subdev *asd)
{
        struct ispcam_device *ispcam = container_of(async, struct ispcam_device,
                                              notifier);
	struct isp_async_device *isd = &ispcam->isd[0];

	struct media_entity *entity = &subdev->entity; /*SOURCE_PAD*/
	int i = 0;
	int ret = 0;

	if(entity->num_pads == 0) {
		dev_err(ispcam->dev, "%s, %d No pad found in async entity(%s)\n", __func__, __LINE__, subdev->name);
		return -EINVAL;
	}

	for(i = 0; i < entity->num_pads; i++) {
		if(entity->pads[i].flags & MEDIA_PAD_FL_SOURCE)
			break;
	}


	if(i == entity->num_pads) {
		dev_err(ispcam->dev, "%s, %d, no source pad in async entity (%s)\n", __func__, __LINE__, subdev->name);
		return -EINVAL;
	}

	isd->source_pad = i;

	isd->sd = subdev;
	/*TODO: pass common args from isp host to sensor driver.*/
	isd->sd->host_priv = NULL;


	ret = sensor_device_probe(&ispcam->sensor, ispcam);
	if(ret < 0) {
		/*TODO*/
	}

	dev_info(ispcam->dev, "%s, %d\n", __func__, __LINE__);

	return 0;
}

static int ispcam_asd_notifier_complete(struct v4l2_async_notifier *async)
{
        struct ispcam_device *ispcam = container_of(async, struct ispcam_device,
                                              notifier);

	int ret = 0;
	struct media_entity *input = NULL;		/*SINK PAD*/
	unsigned int flags = 0;
	unsigned int pad = 0;
	struct isp_async_device *isd = &ispcam->isd[0];

	dev_info(ispcam->dev, "%s, %d\n", __func__, __LINE__);
	/*1. 绑定所有的子component,确保所有的子的硬件设备在线, 子设备会在里面进行subdev的初始化和注册等操作.*/
	ret = component_bind_all(ispcam->dev, ispcam);
	if(ret < 0) {
		goto err_component_bind_all;
	}

	/*CSI	<->	VIC*/

	/*VIC	<->	ISP*/
	//printk("creating link vic to isp\n");
	if(ispcam->vic && ispcam->isp)
		ret = media_create_pad_link(
				&ispcam->vic->sd.entity, VIC_PAD_SOURCE,
				&ispcam->isp->sd.entity, ISP_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

#ifdef CONFIG_VIC_DMA_ROUTE
	/*VIC <-> Memory*/
	if(ispcam->vic)
		ret = media_create_pad_link(
				&ispcam->vic->sd.entity, VIC_PAD_SOURCE,
				&ispcam->vic->ispvideo.video.entity, ISP_VIDEO_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
#endif

	/*ISP	<->	MSCALER*/
	//printk("creating link isp to mscaler\n");
	if(ispcam->isp && ispcam->mscaler)
		ret = media_create_pad_link(
				&ispcam->isp->sd.entity, ISP_PAD_SOURCE,
				&ispcam->mscaler->sd.entity, MSCALER_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

	/*Sensor -> 0]CSI[1 -> 0]VIC[1 -> 0]ISP[1 -> 0]Mscaler[1 -> ]Memory*/
	//printk("creating link from mscaler to isp-video\n");
	if(ispcam->mscaler) {
		ret = media_create_pad_link(
				&ispcam->mscaler->sd.entity, MSCALER_PAD_SOURCE_CH0,
				&ispcam->mscaler->ispvideo[0].video.entity, ISP_VIDEO_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

		/*CH1*/
		if(MSCALER_MAX_CH > 1) {
			ret = media_create_pad_link(
					&ispcam->mscaler->sd.entity, MSCALER_PAD_SOURCE_CH1,
					&ispcam->mscaler->ispvideo[1].video.entity, ISP_VIDEO_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		}
		/*CH2*/
		if(MSCALER_MAX_CH > 2) {
			ret = media_create_pad_link(
					&ispcam->mscaler->sd.entity, MSCALER_PAD_SOURCE_CH2,
					&ispcam->mscaler->ispvideo[2].video.entity, ISP_VIDEO_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		}
	}
	if(isd->bus_type == V4L2_MBUS_CSI2_DPHY) {
		dev_info(ispcam->dev, "creating link csi to vic\n");
		if(!ispcam->csi) {
			dev_err(ispcam->dev, "*** Error: MIPI Sensor Found while CSI not enabled in dts ***\n");
			return -EINVAL;
		}
		/*1. connect csi to vic.*/
		if(ispcam->csi && ispcam->vic)
			ret = media_create_pad_link(
					&ispcam->csi->sd.entity, CSI_PAD_SOURCE,
					&ispcam->vic->sd.entity, VIC_PAD_SINK, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		/*connect to csi pad*/
		input = &ispcam->csi->sd.entity;
		flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED;
		pad = CSI_PAD_SINK;
	} else if(isd->bus_type == V4L2_MBUS_PARALLEL) {
		dev_info(ispcam->dev, "creating link dvp to vic.\n");
		/*connect to vic dvp port.*/
		input = &ispcam->vic->sd.entity;
		pad =	VIC_PAD_SINK;
		flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED;

	} else {
		dev_err(ispcam->dev, "Unsupported v4l2 mbus type.\n");
		return -EINVAL;
	}
	/* connect sensor to csi or vic. */
	ret = media_create_pad_link(&isd->sd->entity, isd->source_pad,
			input, pad, MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if(ret < 0) {
		dev_err(ispcam->dev, "%s, %d, failed to create link from [%s] to [%s]\n",
				__func__, __LINE__, isd->sd->entity.name, input->name);
		return -EINVAL;
	}

	/* Expose all subdev's nodes*/
	ret = v4l2_device_register_subdev_nodes(&ispcam->v4l2_dev);
	if (ret) {
		dev_err(ispcam->mdev.dev,
				"vimc subdev nodes registration failed (err=%d)\n",
				ret);
		goto err_register_subdev_nodes;
	}

	return 0;
err_register_subdev_nodes:
err_component_bind_all:
	return ret;
}



static int ispcam_comp_bind(struct device *master)
{
        struct ispcam_device *ispcam = platform_get_drvdata(to_platform_device(master));
	int ret = 0;
	struct v4l2_async_notifier_operations ops = {
		.bound = ispcam_asd_notifier_bound,
		.complete = ispcam_asd_notifier_complete,
	};
	dev_info(master, "master component bind.!\n");
	media_device_init(&ispcam->mdev);

	/* Register the v4l2 struct */
	ret = v4l2_device_register(ispcam->mdev.dev, &ispcam->v4l2_dev);
	if (ret) {
		dev_err(ispcam->mdev.dev,
				"v4l2 device register failed (err=%d)\n", ret);
		return ret;
	}

	ret = media_device_register(&ispcam->mdev);
	if(ret < 0) {
		dev_err(ispcam->mdev.dev, "media device register failed\n");
		goto err_media_device_register;
	}




	/*3. 注册异步设备完成通知,确保所有的sensor在线.在完成通知里面进行media_device和link的创建.*/
	ispcam->notifier.ops = &ops;
	ret = v4l2_async_notifier_register(&ispcam->v4l2_dev, &ispcam->notifier);
	if(ret < 0) {
		goto err_v4l2_async_notifier_register;
	}


	return 0;
err_v4l2_async_notifier_register:
err_media_device_register:
	return ret;
}

static void ispcam_comp_unbind(struct device *master)
{
        struct ispcam_device *ispcam = platform_get_drvdata(to_platform_device(master));

        dev_info(master, "TODO: %p master component unbind!\n", ispcam);

}


static const struct component_master_ops ispcam_comp_ops = {
        .bind = ispcam_comp_bind,
        .unbind = ispcam_comp_unbind,
};


static int comp_compare(struct device *comp, void *data)
{
	return comp == data;
}

static int ingenic_isp_probe(struct platform_device *pdev)
{

	struct ispcam_device *ispcam = NULL;
	struct component_match *match = NULL;
	const struct of_device_id *of_match;
	struct ispcam_data *data = NULL;
	int ret;
	int i;
	char *dev_nr = NULL;

	ispcam = kzalloc(sizeof(struct ispcam_device), GFP_KERNEL);
	if(!ispcam) {
		pr_err("Failed to alloc ispcam dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	ispcam->dev = &pdev->dev;
	platform_set_drvdata(pdev, ispcam);

	of_match = of_match_node(ingenic_isp_dt_match, pdev->dev.of_node);
	if (!of_match){
		return -ENODEV;
	}

	data = (struct ispcam_data *)of_match->data;
	if(data->chip_name)
		memcpy(ispcam->chip_name, data->chip_name, sizeof(ispcam->chip_name));
	else
		return -ENODEV;

	ingenic_isp_parse_dt(ispcam);

	for(i = 0; i < ispcam->subdev_num; i++) {
		component_match_add(ispcam->dev, &match, comp_compare, &ispcam->subdevs[i]->dev);
	}


	mutex_init(&ispcam->mutex);
	ispcam->v4l2_dev.mdev = &ispcam->mdev;

	ispcam->mdev.dev = ispcam->dev;
	strlcpy(ispcam->mdev.model, "ingenic-isp", sizeof(ispcam->mdev.model));

	dev_nr = strchr(pdev->name, '@');
	if(!dev_nr){
		dev_err(ispcam->dev, "failed to get dev_nr\n");
		return -ENODEV;
	}
	ispcam->dev_nr = dev_nr[1]-'0';

	/* add self to component */
	ret = component_master_add_with_match(ispcam->dev, &ispcam_comp_ops, match);
	if(ret < 0) {
		dev_err(ispcam->dev, "failed to add component master\n");
		/*TODO*/
	}

	ret = of_reserved_mem_device_init(ispcam->dev);
	if(ret)
		dev_warn(ispcam->dev, "failed to init reserved mem\n");

	return 0;


	return ret;
}



static int ingenic_isp_remove(struct platform_device *pdev)
{

	return 0;
}

struct ispcam_data ingenic_x2500_camera_data = {
	.chip_name = "x2500",
};


static const struct of_device_id ingenic_isp_dt_match[] = {
        { .compatible = "ingenic,x2500-isp-camera", .data = (void *)&ingenic_x2500_camera_data},
        { }
};

MODULE_DEVICE_TABLE(of, ingenic_isp_dt_match);

static int __maybe_unused ingenic_isp_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
        return 0;
}

static int __maybe_unused ingenic_isp_drv_resume(struct platform_device *pdev)
{
        return 0;
}

static struct platform_driver ingenic_isp_driver = {
        .probe = ingenic_isp_probe,
        .remove = ingenic_isp_remove,
	.suspend = ingenic_isp_drv_suspend,
	.resume = ingenic_isp_drv_resume,
        .driver = {
                .name = "ingenic-ispdrv",
                .of_match_table = ingenic_isp_dt_match,
        },
};

module_platform_driver(ingenic_isp_driver);

MODULE_ALIAS("platform:ingenic-ispdrv");
MODULE_DESCRIPTION("ingenic isp subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");

