#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>


#include "isp-drv.h"
#include "csi-regs.h"

static const struct v4l2_subdev_core_ops csi_subdev_core_ops = {
        .log_status = v4l2_ctrl_subdev_log_status,
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static int csi_subdev_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct csi_device *csi = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	int ret = 0;

	remote = media_entity_remote_pad(&csi->pads[CSI_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	ret = v4l2_subdev_call(remote_sd, pad, set_fmt, NULL, format);
	if(ret < 0) {
		dev_dbg(csi->dev, "Failed to set_fmt from remote pad\n");
	}

	return 0;
}
static int csi_subdev_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct csi_device *csi = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt;
	int ret = 0;

	remote = media_entity_remote_pad(&csi->pads[CSI_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	/*获取源当前格式，复制到输出格式.*/
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if(ret < 0) {
		dev_err(csi->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	memcpy(&format->format, &remote_subdev_fmt.format, sizeof(format->format));

	//printk("----%s, %d, format->pad: %d\n", __func__, __LINE__, format->pad);
	return 0;
}


static const struct v4l2_subdev_pad_ops csi_subdev_pad_ops = {
        .set_fmt                = csi_subdev_set_fmt,
        .get_fmt                = csi_subdev_get_fmt,
	/*
        .init_cfg               = csi_subdev_init_cfg,
        .enum_mbus_code         = csi_subdev_enum_mbus_code,
        .enum_frame_size        = csi_subdev_enum_frame_size,
        .get_fmt                = csi_subdev_get_fmt,
	*/
};

struct phy_reseter {
	struct csi_device *csi[2];
	unsigned int phy_started[2];
};

struct phy_reseter global_phy_reseter = {
	.csi = {NULL, NULL},
	.phy_started = {0, 0}
};

DEFINE_SPINLOCK(reset_lock);

static unsigned char csi_core_write_part(struct csi_device *csi,unsigned int address,
                                                unsigned int data, unsigned char shift, unsigned char width)
{
        unsigned int mask = (1 << width) - 1;
        unsigned int temp = csi_core_readl(csi, address);

        temp &= ~(mask << shift);
        temp |= (data & mask) << shift;
        csi_core_writel(csi, address, temp);

        return 0;
}

static inline int csi_core_write_io(unsigned int iobase, unsigned int address, unsigned int data, unsigned char shift, unsigned char width)
{
        unsigned int mask = (1 << width) - 1;
        unsigned int temp = readl(iobase + address);

        temp &= ~(mask << shift);
        temp |= (data & mask) << shift;
        writel(temp, iobase + address);
	return 0;
}


#define CSI0_IOBASE	0xb0074000
#define CSI1_IOBASE	0xb0073000
static int csi_phy_start(struct csi_device *csi, struct phy_reseter *reseter)
{
	unsigned long flags;
	/*already reseted.*/
	spin_lock_irqsave(&reset_lock, flags);
	if(reseter->phy_started[0] || reseter->phy_started[1]) {
		reseter->phy_started[csi->reset_index] = 1;
		spin_unlock_irqrestore(&reset_lock, flags);
		return 0;
	}

	/* CSI0 and CSI1 reset at the same time. */
	csi_core_write_io(CSI0_IOBASE, DPHY_RSTZ, 0, 0, 1);
	csi_core_write_io(CSI1_IOBASE, DPHY_RSTZ, 0, 0, 1);

	csi_core_write_io(CSI0_IOBASE, DPHY_RSTZ, 1, 0, 1);
	csi_core_write_io(CSI1_IOBASE, DPHY_RSTZ, 1, 0, 1);

	reseter->phy_started[csi->reset_index] = 1;
	spin_unlock_irqrestore(&reset_lock, flags);


	return 0;
}

static void csi_phy_stop(struct csi_device *csi, struct phy_reseter *reseter)
{

	unsigned long flags;

	/* CSI2_RESET */
	spin_lock_irqsave(&reset_lock, flags);
	csi_core_write_part(csi, CSI2_RESETN, 0, 0, 1);

	reseter->phy_started[csi->reset_index] = 0;

	if((reseter->phy_started[0] == 0) && (reseter->phy_started[1] == 0)) {
		/* shutdown phy. */
		csi_core_write_io(CSI0_IOBASE, DPHY_RSTZ, 0, 0, 1);
		csi_core_write_io(CSI1_IOBASE, DPHY_RSTZ, 0, 0, 1);
	}

	spin_unlock_irqrestore(&reset_lock, flags);
}

static int csi_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_device *csi = v4l2_get_subdevdata(sd);
	struct v4l2_fwnode_bus_mipi_csi2 *mipi_csi2 = ispcam_get_bus_mipi_csi2(csi->ispcam);
	unsigned short nlanes = mipi_csi2->num_data_lanes;

	if(enable) {
		if(csi->enabled++ > 0) {
			return 0;
		}

		if(nlanes > 2)
			csi_phy_writel(csi, CSI_PHY_1C2C_MODE, 0);	/*1c mode.*/
		else
			csi_phy_writel(csi, CSI_PHY_1C2C_MODE, 1);	/*2c mode.*/
		csi_core_writel(csi, N_LANES, nlanes - 1);
		csi_phy_start(csi, &global_phy_reseter);

		/* CSI2_RESET */
		csi_core_write_part(csi, CSI2_RESETN, 0, 0, 1);
		csi_core_write_part(csi, CSI2_RESETN, 1, 0, 1);

	} else {

		if(--csi->enabled > 0) {
			return 0;
		}
		csi_phy_stop(csi, &global_phy_reseter);
	}

	return 0;
}


static const struct v4l2_subdev_video_ops csi_subdev_video_ops = {
        .s_stream = csi_subdev_s_stream,
};


static const struct v4l2_subdev_ops csi_subdev_ops = {
        .core = &csi_subdev_core_ops,
        .pad = &csi_subdev_pad_ops,
        .video = &csi_subdev_video_ops,
};

static ssize_t
dump_csi(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct csi_device *csi = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "VERSION	:0x%08x\n",	csi_core_readl(csi, VERSION));
	p += sprintf(p, "N_LANES	:0x%08x\n",	csi_core_readl(csi, N_LANES));
	p += sprintf(p, "PHY_SHUTDOWNZ	:0x%08x\n",	csi_core_readl(csi, PHY_SHUTDOWNZ));
	p += sprintf(p, "DPHY_RSTZ	:0x%08x\n",	csi_core_readl(csi, DPHY_RSTZ));
	p += sprintf(p, "CSI2_RESETN	:0x%08x\n",	csi_core_readl(csi, CSI2_RESETN));
	p += sprintf(p, "PHY_STATE	:0x%08x\n",	csi_core_readl(csi, PHY_STATE));
	p += sprintf(p, "DATA_IDS_1	:0x%08x\n",	csi_core_readl(csi, DATA_IDS_1));
	p += sprintf(p, "DATA_IDS_2	:0x%08x\n",	csi_core_readl(csi, DATA_IDS_2));
	p += sprintf(p, "ERR1		:0x%08x\n",	csi_core_readl(csi, ERR1));
	p += sprintf(p, "ERR2		:0x%08x\n",	csi_core_readl(csi, ERR2));
	p += sprintf(p, "MASK1		:0x%08x\n",	csi_core_readl(csi, MASK1));
	p += sprintf(p, "MASK2		:0x%08x\n",	csi_core_readl(csi, MASK2));
	p += sprintf(p, "PHY_TST_CTRL0  :0x%08x\n",	csi_core_readl(csi, PHY_TST_CTRL0));
	p += sprintf(p, "PHY_TST_CTRL1  :0x%08x\n",	csi_core_readl(csi, PHY_TST_CTRL1));
	p += sprintf(p, "VC0_FRAME_NUM	:0x%08x\n",	csi_core_readl(csi, VC0_FRAME_NUM));
	p += sprintf(p, "VC1_FRAME_NUM	:0x%08x\n",	csi_core_readl(csi, VC1_FRAME_NUM));
	p += sprintf(p, "VC2_FRAME_NUM	:0x%08x\n",	csi_core_readl(csi, VC2_FRAME_NUM));
	p += sprintf(p, "VC3_FRAME_NUM	:0x%08x\n",	csi_core_readl(csi, VC3_FRAME_NUM));
	p += sprintf(p, "VC0_LINE_NUM	:0x%08x\n",	csi_core_readl(csi, VC0_LINE_NUM));
	p += sprintf(p, "VC1_LINE_NUM	:0x%08x\n",	csi_core_readl(csi, VC1_LINE_NUM));
	p += sprintf(p, "VC2_LINE_NUM	:0x%08x\n",	csi_core_readl(csi, VC2_LINE_NUM));
	p += sprintf(p, "VC3_LINE_NUM	:0x%08x\n",	csi_core_readl(csi, VC3_LINE_NUM));

	p += sprintf(p, "CSI_PHY_1C2C_MODE :0x%08x\n", 	csi_phy_readl(csi, CSI_PHY_1C2C_MODE));

	return p - buf;
}


static DEVICE_ATTR(dump_csi, S_IRUGO|S_IWUSR, dump_csi, NULL);

static struct attribute *csi_debug_attrs[] = {
        &dev_attr_dump_csi.attr,
	NULL,
};

static struct attribute_group csi_debug_attr_group = {
        .name   = "debug",
        .attrs  = csi_debug_attrs,
};


static int csi_comp_bind(struct device *comp, struct device *master,
                              void *master_data)
{
	struct csi_device *csi = dev_get_drvdata(comp);
	struct ispcam_device *ispcam = (struct ispcam_device *)master_data;
	struct v4l2_device *v4l2_dev = &ispcam->v4l2_dev;
	struct v4l2_subdev *sd = &csi->sd;
	int ret = 0;
	int i = 0;

	//dev_info(comp, "----dev_name(comp): %s----%s, %d \n", dev_name(comp), __func__, __LINE__);

	/* link subdev to master.*/
	csi->ispcam = (void *)ispcam;
	ispcam->csi = csi;

	/*1. register supported subdev ctrls.*/


	/*2. init v4l2_subdev*/

	v4l2_subdev_init(sd, &csi_subdev_ops);

	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(comp), sizeof(sd->name));
	v4l2_set_subdevdata(sd, csi);


	/* init csi pads. */
	csi->pads = kzalloc(sizeof(struct media_pad) * CSI_NUM_PADS, GFP_KERNEL);
	if(!csi->pads) {
		ret = -ENOMEM;
		goto err_alloc_pads;
	}
	csi->pads[0].index = CSI_PAD_SINK;
	csi->pads[0].flags = MEDIA_PAD_FL_SINK;
	csi->pads[1].index = CSI_PAD_SOURCE;
	csi->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, CSI_NUM_PADS, csi->pads);

	/*3. register v4l2_subdev*/
	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if(ret < 0) {
		dev_err(comp, "Failed to register v4l2_subdev for csi\n");
		goto err_subdev_register;
	}


	return 0;
err_subdev_register:
err_alloc_pads:
	return ret;
}


static void csi_comp_unbind(struct device *comp, struct device *master,
                                 void *master_data)
{
        struct csi_device *csi = dev_get_drvdata(comp);

	dev_info(comp, "----TODO-implement unbind: %p---%s, %d \n",csi, __func__, __LINE__);
}



static const struct component_ops csi_comp_ops = {
        .bind = csi_comp_bind,
        .unbind = csi_comp_unbind,
};


static int ingenic_csi_probe(struct platform_device *pdev)
{

	struct csi_device *csi = NULL;
	struct resource *regs = NULL;

	int ret = 0;
	int i = 0;

	csi = kzalloc(sizeof(struct csi_device), GFP_KERNEL);
	if(!csi) {
		pr_err("Failed to alloc csi dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	csi->dev = &pdev->dev;
	platform_set_drvdata(pdev, csi);
	dev_set_drvdata(csi->dev, csi);

	csi->irq = platform_get_irq(pdev, 0);
	if(csi->irq < 0) {
		dev_warn(&pdev->dev, "No CSI IRQ specified\n");
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!regs) {
		dev_err(&pdev->dev, "No iomem resource!\n");
		goto err_get_resource;
	}

	csi->iobase = devm_ioremap_resource(&pdev->dev, regs);
	if(!csi->iobase) {
		goto err_ioremap;
	}

	of_property_read_u32(csi->dev->of_node, "clk_precounter", &csi->clk_precounter);
	of_property_read_u32(csi->dev->of_node, "data_precounter", &csi->data_precounter);

	csi->phy_base = CSI_PHY_IOBASE | 0xa0000000;

	/*TODO: clk*/
	csi->gate_clk = of_clk_get(csi->dev->of_node, 0);
	if(!csi->gate_clk) {
		dev_err(csi->dev, "failed to get gate clk\n");
		goto err_gate_clk;
	}
	clk_prepare_enable(csi->gate_clk);

	if(csi->clk_precounter)
		csi_phy_writel(csi, CSI_PHY_PRECOUNTER_IN_CLK, csi->clk_precounter);
	if(csi->data_precounter)
		csi_phy_writel(csi, CSI_PHY_PRECOUNTER_IN_DATA, csi->data_precounter);

	ret = sysfs_create_group(&csi->dev->kobj, &csi_debug_attr_group);
	if (ret) {
		dev_err(csi->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_sys_group;
	}

	for(i = 0; i < 2; i++) {
		if(global_phy_reseter.csi[i] == NULL) {
			global_phy_reseter.csi[i] = csi;
			csi->reset_index = i;
			break;
		}
	}

	ret = component_add(csi->dev, &csi_comp_ops);
	if(ret < 0) {
		dev_err(csi->dev, "Failed to add component csi!\n");
	}

	return 0;
err_sys_group:
err_gate_clk:
err_ioremap:
err_get_resource:

	return ret;
}



static int ingenic_csi_remove(struct platform_device *pdev)
{
	struct csi_device *csi = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(csi->gate_clk);
	return 0;
}



static const struct of_device_id ingenic_csi_dt_match[] = {
        { .compatible = "ingenic,x2000-csi" },
        { .compatible = "ingenic,m300-csi" },
        { }
};

MODULE_DEVICE_TABLE(of, ingenic_csi_dt_match);

static int __maybe_unused ingenic_csi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct csi_device *csi = dev_get_drvdata(&pdev->dev);

	if(csi->enabled){
		dev_err(csi->dev, "faild to suspend, csi is streaming on\n");
		return -EBUSY;
	}

	clk_disable_unprepare(csi->gate_clk);
        return 0;
}

static int __maybe_unused ingenic_csi_resume(struct platform_device *pdev)
{
	struct csi_device *csi = dev_get_drvdata(&pdev->dev);

	clk_prepare_enable(csi->gate_clk);
        return 0;
}


static struct platform_driver ingenic_csi_driver = {
        .probe = ingenic_csi_probe,
        .remove = ingenic_csi_remove,
	.suspend = ingenic_csi_suspend,
	.resume = ingenic_csi_resume,
        .driver = {
                .name = "ingenic-csi",
                .of_match_table = ingenic_csi_dt_match,
        },
};

module_platform_driver(ingenic_csi_driver);

MODULE_ALIAS("platform:ingenic-csi");
MODULE_DESCRIPTION("ingenic csi subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");

