#ifndef __DPU_SYSFS_H__
#define __DPU_SYSFS_H__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>


struct dpu_sysfs {

	struct dpu_ctrl *dctrl;
	struct hw_compfb_device *compfb;

};

struct layer_device_attr {
        struct device_attribute attr;
        unsigned int id;
};

#define to_layer_attr(attr)     \
        container_of(attr, struct layer_device_attr, attr)


#define LAYER_ATTR(layer, _name, _mode, _show, _store)          \
        {                                                       \
        .attr   = __ATTR(_name, _mode, _show, _store),  \
        .id     = layer,                                                \
        }


#define LAYER_DEVICE_ATTR(_name, _mode, _show, _store)                                                    \
        static struct layer_device_attr dev_attr_##_name##layer0 = LAYER_ATTR(0, _name, _mode, _show, _store); \
        static struct layer_device_attr dev_attr_##_name##layer1 = LAYER_ATTR(1, _name, _mode, _show, _store); \
        static struct layer_device_attr dev_attr_##_name##layer2 = LAYER_ATTR(2, _name, _mode, _show, _store); \
        static struct layer_device_attr dev_attr_##_name##layer3 = LAYER_ATTR(3, _name, _mode, _show, _store)


int dpu_sysfs_init(struct dpu_sysfs *sysfs, struct dpu_ctrl *dctrl, struct hw_compfb_device *compfb);


int dpu_sysfs_exit(struct dpu_sysfs *sysfs);


#endif
