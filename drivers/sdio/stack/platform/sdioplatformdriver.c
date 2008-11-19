/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@file: sdioplatformdriver.c

@abstract: Linux implementation module for SDIO pltaform driver

#notes:

@notice: Copyright (c), 2006 Atheros Communications, Inc.

@license:  This program is free software; you can redistribute it and/or modify
           it under the terms of the GNU General Public License version 2 as
           published by the Free Software Foundation.



 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation;
 *
 *  Software distributed under the License is distributed on an "AS
 *  IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  rights and limitations under the License.
 *
 *  Portions of this code were developed with information supplied from the
 *  SD Card Association Simplified Specifications. The following conditions and disclaimers may apply:
 *
 *   The following conditions apply to the release of the SD simplified specification (�Simplified
 *   Specification�) by the SD Card Association. The Simplified Specification is a subset of the complete
 *   SD Specification which is owned by the SD Card Association. This Simplified Specification is provided
 *   on a non-confidential basis subject to the disclaimers below. Any implementation of the Simplified
 *   Specification may require a license from the SD Card Association or other third parties.
 *   Disclaimers:
 *   The information contained in the Simplified Specification is presented only as a standard
 *   specification for SD Cards and SD Host/Ancillary products and is provided "AS-IS" without any
 *   representations or warranties of any kind. No responsibility is assumed by the SD Card Association for
 *   any damages, any infringements of patents or other right of the SD Card Association or any third
 *   parties, which may result from its use. No license is granted by implication, estoppel or otherwise
 *   under any patent or other rights of the SD Card Association or any third party. Nothing herein shall
 *   be construed as an obligation by the SD Card Association to disclose or distribute any technical
 *   information, know-how or other confidential information to any third party.
 *
 *
 *  The initial developers of the original code are Seung Yi and Paul Lever
 *
 *  sdio@atheros.com
 *
 *

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define DESCRIPTION "SDIO Platform Driver"
#define AUTHOR "Atheros Communications, Inc."

//??for .h

struct sdioplatform_peripheral {
    struct list_head    node;
    struct sdioplatform_controller *controller;
    struct device       dev;
};
struct sdioplatform_driver {
    struct device_driver drv;
    int (*probe)(struct sdioplatform_peripheral *);
    void (*remove)(struct sdioplatform_peripheral *);
    int (*suspend)(struct sdioplatform_peripheral *, pm_message_t);
    int (*resume)(struct sdioplatform_peripheral *);
};


struct sdioplatform_controller {
    struct device       *dev;
};
struct sdioplatform_controller_driver {
    struct device_driver drv;
    int (*probe)(struct sdioplatform_controller *);
    void (*remove)(struct sdioplatform_controller *);
    int (*suspend)(struct sdioplatform_controller *, pm_message_t);
    int (*resume)(struct sdioplatform_controller *);
};



#define device_to_sdioplatform_peripheral(d)  container_of(d, struct sdioplatform_peripheral, dev)
#define driver_to_sdioplatform_driver(d)  container_of(d, struct sdioplatform_driver, drv)

#define device_to_sdioplatform_controller(d)  container_of(d, struct sdioplatform_controller, dev)
#define driver_to_sdioplatform_controller_driver(d)  container_of(d, struct sdioplatform_controller_driver, drv)

#define SDIOPLATFORM_ATTR(name, fmt, args...)                    \
static ssize_t sdio_##name##_show (struct device *dev, struct device_attribute *attr, char *buf) \
{                                   \
    struct sdioplatform_peripheral *peripheral = device_to_sdioplatform_peripheral(dev);  \
    return sprintf(buf, fmt, args);                 \
}

SDIOPLATFORM_ATTR(bus_id, "%s\n", bus_id);
#define SDIOPLATFORM_ATTR_RO(name) __ATTR(name, S_IRUGO, sdioplatform_##name##_show, NULL)

static struct device_attribute sdioplatform_dev_attrs[] = {
    SDIOPLATFORM_ATTR_RO(bus_id),
    __ATTR_NULL
};

static struct bus_type sdioplatform_bus_type = {
    .name       = "sdioplatform",
    .dev_attrs  = sdioplatform_dev_attrs,
    .match      = sdioplatform_bus_match,
    .hotplug    = NULL,
    .suspend    = sdioplatform_bus_suspend,
    .resume     = sdioplatform_bus_resume,
};


/* controller functions */
static int sdioplatform_controllerdrv_probe(struct device *dev)
{
    struct sdioplatform_controller_driver *drv = driver_to_sdioplatform_controller_driver(dev->driver);
    struct sdioplatform_controller *controller = device_to_sdioplatform_controller(dev);

    return drv->probe(controller);
}

static int sdioplatform_controllerdrv_remove(struct device *dev)
{
    struct sdioplatform_controller_driver *drv = driver_to_sdioplatform_controller_driver(dev->driver);
    struct sdioplatform_controller *controller = device_to_sdioplatform_controller(dev);

    return drv->remove(controller);
}

/*
 * sdioplatform_register_controller_driver - register a controller driver
 */
int sdioplatform_register_controller_driver(struct sdioplatform_controller_driver *drv)
{
    drv->drv.bus = &sdioplatform_bus_type;
    drv->drv.probe = sdioplatform_controllerdrv_probe;
    drv->drv.remove = sdioplatform_controllerdrv_remove;
    return driver_register(&drv->drv);
}

/*
 *  sdioplatform_unregister_controller_driver - unregister a controller driver
 */
void sdioplatform_unregister_controller_driver(struct sdioplatform_driver *drv)
{
    driver_unregister(&drv->drv);
}

/*
 * sdioplatform_add_controller - register a controller device
 */
int sdioplatform_add_controller(char *name, struct sdioplatform_controller *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    strncpy(dev->dev.bus_id, BUS_ID_SIZE, name);
    return device_register(&dev->dev);
}

/*
 * sdioplatform_remove_controller - unregister a controller device
 */
int sdioplatform_remove_controller(char *name, struct sdioplatform_controller *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    return device_unregister(&dev->dev);
}

/* peripheral functions */
static int sdioplatform_drv_probe(struct device *dev)
{
    struct sdioplatform_driver *drv = driver_to_sdioplatform_driver(dev->driver);
    struct sdioplatform_peripheral *peripheral = device_to_sdioplatform_peripheral(dev);

    return drv->probe(peripheral);
}

static int sdioplatform_controllerdrv_remove(struct device *dev)
{
    struct sdioplatform_controller_driver *drv = driver_to_sdioplatform_controller_driver(dev->driver);
    struct sdioplatform_controller *controller = device_to_sdioplatform_controller(dev);

    return drv->remove(controller);
}

/*
 * sdioplatform_register_driver - register a driver
 */
int sdioplatform_register_driver(struct sdioplatform_driver *drv)
{
    drv->drv.bus = &sdioplatform_bus_type;
    drv->drv.probe = sdioplatform_drv_probe;
    drv->drv.remove = sdioplatform_drv_remove;
    return driver_register(&drv->drv);
}

/*
 *  sdioplatform_unregister_driver - unregister a driver
 */
void sdioplatform_unregister_driver(struct sdioplatform_driver *drv)
{
    driver_unregister(&drv->drv);
}

/*
 * sdioplatform_add_peripheral - register a peripheral device
 */
int sdioplatform_add_peripheral(char *name, struct sdioplatform_peripheral *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    strncpy(dev->dev.bus_id, BUS_ID_SIZE, name);
    return device_register(&dev->dev);
}

/*
 * sdioplatform_remove_peripheral - unregister a peripheral device
 */
int sdioplatform_remove_peripheral(char *name, struct sdioplatform_peripheral *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    return device_unregister(&dev->dev);
}





static int sdioplatform_bus_match(struct device *dev, struct device_driver *drv)
{
    /* probes handle the matching */
    return 1;
}

static int sdioplatform_bus_suspend(struct device *dev, pm_message_t state)
{
    struct sdioplatform_driver *drv = driver_to_sdioplatform_driver(dev->driver);
    struct sdioplatform_peripheral *peripheral = device_to_sdioplatform_peripheral(dev);
    int ret = 0;

    if (peripheral->driver && drv->suspend) {
        ret = drv->suspend(peripheral, state);
    }
    return ret;
}

static int sdioplatform_bus_resume(struct device *dev)
{
    struct sdioplatform_driver *drv = driver_to_sdioplatform_driver(dev->driver);
    struct sdioplatform_peripheral *peripheral = device_to_sdioplatform_peripheral(dev);
    int ret = 0;

    if (peripheral->driver && drv->resume) {
        ret = drv->resume(card);
    }
    return ret;
}

/*
 * module init
*/
static int __init sdio_platformdriver_init(void) {
    int ret = bus_register(&sdioplatform_bus_type);
    return ret;
}

/*
 * module cleanup
*/
static void __exit sdio_platformdriver_cleanup(void) {
    REL_PRINT(SDDBG_TRACE, ("SDIO unloaded\n"));
    _SDIO_BusDriverCleanup();
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_AUTHOR(AUTHOR);

module_init(sdio_platformdriver_init);
module_exit(sdio_platformdriver_cleanup);
EXPORT_SYMBOL(sdioplatform_register_controller_driver);
EXPORT_SYMBOL(sdioplatform_unregister_controller_driver);
EXPORT_SYMBOL(sdioplatform_add_controller);
EXPORT_SYMBOL(sdioplatform_remove_controller);
EXPORT_SYMBOL(sdioplatform_register_driver);
EXPORT_SYMBOL(sdioplatform_unregister_driver);
EXPORT_SYMBOL(sdioplatform_add_peripheral);
EXPORT_SYMBOL(sdioplatform_remove_peripheral);



