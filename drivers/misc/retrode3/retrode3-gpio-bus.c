/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/io.h>

#include "retrode3.h"

/* Example register offsets - adapt to real hardware */
#define REG_ADDR_LOW    0x00
#define REG_ADDR_HIGH   0x04
#define REG_DATA        0x08
#define REG_CTRL        0x0c
#define CTRL_STROBE     BIT(0)
#define CTRL_DIR_WRITE  BIT(1)

struct retrode3_bus_bus {
    void __iomem *base;
    struct retrode3_bus_controller ctlr;
    struct platform_device *pdev;
};

static int bus_lock(struct retrode3_bus_controller *ctlr)
{
    mutex_lock(&ctlr->lock);
    return 0;
}

static void bus_unlock(struct retrode3_bus_controller *ctlr)
{
    mutex_unlock(&ctlr->lock);
}

static int bus_set_addr(struct retrode3_bus_controller *ctlr, u64 addr)
{
    struct retrode3_bus_bus *m = ctlr->priv;
    /* Write lower/upper 32bits */
    writel((u32)(addr & 0xffffffff), m->base + REG_ADDR_LOW);
    writel((u32)(addr >> 32), m->base + REG_ADDR_HIGH);
    return 0;
}

static int bus_set_select(struct retrode3_bus_controller *ctlr, unsigned int sel)
{
    /* Implementation depends on hardware; maybe write a sel register */
    /* For example, write to REG_CTRL high bits */
    return 0;
}

static int bus_xfer(struct retrode3_bus_controller *ctlr, u8 dir, void *buf, size_t len, size_t *actual)
{
    struct retrode3_bus_bus *m = ctlr->priv;
    size_t i;
    u8 *b = buf;

    for (i = 0; i < len; ++i) {
        if (dir == retrode3_bus_DIR_WRITE) {
            writel(b[i], m->base + REG_DATA);
            /* toggle strobe for write */
            writel(CTRL_STROBE | CTRL_DIR_WRITE, m->base + REG_CTRL);
            writel(0, m->base + REG_CTRL);
        } else {
            /* strobe to read then read data */
            writel(CTRL_STROBE, m->base + REG_CTRL);
            writel(0, m->base + REG_CTRL);
            b[i] = readl(m->base + REG_DATA) & 0xff;
        }
    }

    if (actual)
        *actual = len;
    return 0;
}

static const struct retrode3_bus_controller_ops bus_ops = {
    .lock = bus_lock,
    .unlock = bus_unlock,
    .set_addr = bus_set_addr,
    .set_select = bus_set_select,
    .xfer = bus_xfer,
};

static int retrode3_bus_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct retrode3_bus_bus *m;
    int ret;

    m = devm_kzalloc(&pdev->dev, sizeof(*m), GFP_KERNEL);
    if (!m)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    m->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(m->base))
        return PTR_ERR(m->base);

    m->pdev = pdev;
    m->ctlr.dev = &pdev->dev;
    m->ctlr.ops = &bus_ops;
    m->ctlr.priv = m;
    /* mutex in retrode3_bus_register_controller will be initialized */

    platform_set_drvdata(pdev, m);

    /* register controller with retrode3 core */
    ret = retrode3_bus_register_controller(&m->ctlr);
    if (ret) {
        dev_err(&pdev->dev, "failed to register retrode3 controller: %d\n", ret);
        return ret;
    }

    dev_info(&pdev->dev, "retrode3 bus controller registered\n");
    return 0;
}

static void retrode3_bus_remove(struct platform_device *pdev)
{
    struct retrode3_bus_bus *m = platform_get_drvdata(pdev);

    retrode3_bus_unregister_controller(&m->ctlr);
}

static const struct of_device_id retrode3_bus_of_match[] = {
    { .compatible = "dragonbox,retrode3-bus", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, retrode3_bus_of_match);

static struct platform_driver retrode3_bus_driver = {
    .probe = retrode3_bus_probe,
    .remove = retrode3_bus_remove,
    .driver = {
        .name = "retrode3-bus",
        .of_match_table = retrode3_bus_of_match,
    },
};
module_platform_driver(retrode3_bus_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 bus controller");
