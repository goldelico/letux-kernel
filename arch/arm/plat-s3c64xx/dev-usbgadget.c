/* Base S3C64XX usbgadget resource and device definitions */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>

#include <mach/map.h>
#include <plat/map-base.h>
#include <plat/devs.h>
#include <plat/irqs.h>

static struct resource s3c_usbgadget_resource[] = {
	[0] = {
		.start = S3C64XX_PA_OTG,
		.end   = S3C64XX_PA_OTG + 0x200000  - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_OTG,
		.end   = IRQ_OTG,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_usbgadget = {
	.name		  = "s3c-otg-usbgadget",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_usbgadget_resource),
	.resource	  = s3c_usbgadget_resource,
};
EXPORT_SYMBOL(s3c_device_usbgadget);
