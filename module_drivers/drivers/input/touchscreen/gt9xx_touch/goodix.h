#ifndef TOUCHSCREEN_GT9XX__H
#define TOUCHSCREEN_GT9XX__H

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

/*****************************************************************************
* Debug interface.
*****************************************************************************/

#if GT9_DEBUG_EN

#define GT9_DEBUG(fmt, args...) do { \
    printk("[GT9_TS]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define GT9_FUNC_ENTER() do { \
    printk("[GT9_TS]%s: Enter(%d)\n", __func__,__LINE__); \
} while (0)

#define GT9_FUNC_EXIT() do { \
    printk("[GT9_TS]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)
#else /* #if GT9_DEBUG_EN*/
#define GT9_DEBUG(fmt, args...)
#define GT9_FUNC_ENTER()
#define GT9_FUNC_EXIT()
#endif

#define GT9_INFO(fmt, args...) do { \
    printk(KERN_INFO "[GT9_TS/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define GT9_ERROR(fmt, args...) do { \
    printk(KERN_ERR "[GT9_TS/E]%s:"fmt"\n", __func__, ##args); \
} while (0)

#endif /* TOUCHSCREEN_CST3XX__H  */


