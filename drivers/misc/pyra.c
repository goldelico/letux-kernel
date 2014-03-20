 /* board file extensions for Pyra */

#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/irqdomain.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>

#include <plat/sata.h>

#include "common.h"
#include "common-board-devices.h"
#include "dss-common.h"

/* HACK: create proc dir for pandora's drivers */
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>

static int __init proc_pandora_init(void)
{
	struct proc_dir_entry *ret;

	ret = proc_mkdir("pandora", NULL);
	if (!ret)
		return -ENOMEM;

	ret = gpio_request_one(230, GPIOF_DIR_OUT, "devices reset");
	gpio_set_value(230, 0);
	msleep(10);
	gpio_set_value(230, 1);
	gpio_free(230);

	return 0;
}
fs_initcall(proc_pandora_init);
#endif
