/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3.h"

struct example_priv {
    struct retrode3_bus_device *mdev;
    struct retrode3_bus_controller *ctlr;
};

static int example_probe(struct device *dev)
{
    struct example_priv *p;
    struct retrode3_bus_controller *ctlr;
    size_t got;
    int ret;
    u8 buf[4];

    p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
    if (!p)
        return -ENOMEM;

    /* Two approaches:
     * 1) If the retrode3 core created devices from DT children, 'dev' is already a retrode3_bus_device.
     *    You can get controller via container_of or retrode3_bus_get_controller helper.
     * 2) Otherwise get controller by phandle in your node:
     */
    ctlr = retrode3_bus_get_controller_by_phandle(dev, "retrode3-controller");
    if (IS_ERR(ctlr)) {
        dev_err(dev, "failed to find retrode3 controller\n");
        return PTR_ERR(ctlr);
    }

    p->ctlr = ctlr;
    dev_set_drvdata(dev, p);

    /* Lock the bus, set address, read 4 bytes */
    ret = retrode3_bus_lock_bus(ctlr);
    if (ret) {
        dev_err(dev, "failed to lock retrode3: %d\n", ret);
        return ret;
    }

    ret = retrode3_bus_set_address(ctlr, 0x1000);
    if (ret) {
        dev_err(dev, "set_addr failed: %d\n", ret);
        retrode3_bus_unlock_bus(ctlr);
        return ret;
    }

    ret = retrode3_bus_xfer(ctlr, retrode3_bus_DIR_READ, buf, sizeof(buf), &got);
    if (ret) {
        dev_err(dev, "xfer read failed: %d\n", ret);
        retrode3_bus_unlock_bus(ctlr);
        return ret;
    }

    dev_info(dev, "read %zu bytes from retrode3 addr 0x1000\n", got);
    retrode3_bus_unlock_bus(ctlr);

    return 0;
}

static int example_remove(struct device *dev)
{
    /* cleanup */
    return 0;
}

static const struct of_device_id example_of_match[] = {
    { .compatible = "dragonbox,retrode3-client-example" },
    {}
};
MODULE_DEVICE_TABLE(of, example_of_match);

static struct device_driver example_driver = {
    .name = "retrode3-client-example",
    .of_match_table = example_of_match,
    .probe = example_probe, /* not standard for device_driver; see note below */
    .remove = example_remove,
/* In practice the cleanest way is to use driver_register with custom device_driver:
   driver_register(&example_driver);
   and unregister on exit.
   Alternatively implement as platform_driver but set .driver.bus = &retrode3_bus_type
*/

//    .bus = &retrode3_bus_type,
};

static int __init example_init(void)
{
    return driver_register(&example_driver);
}
static void __exit example_exit(void)
{
    driver_unregister(&example_driver);
}
module_init(example_init);
module_exit(example_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Example retrode3 client");
