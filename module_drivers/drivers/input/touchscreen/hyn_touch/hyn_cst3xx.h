#ifndef TOUCHSCREEN_CST3XX__H
#define TOUCHSCREEN_CST3XX__H

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
* Private constant and macro definitions using #define
*****************************************************************************/

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	1
#define MAX_AREA	0xff

#define CST3XX_NAME 	"cst3xx"

/*register address*/
#define CST3XX_ADDRESS 	0x1A

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct cst3xx_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct device *dev;
	struct cst3xx_platform_data *pdata;
	struct work_struct  work;
       struct delayed_work dwork;
	struct workqueue_struct *workqueue;
	int irq;

	struct ts_event *events;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	u8 *point_buf;

};

extern struct cst3xx_ts_data *cst3xx_data;
/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct cst3xx_platform_data {
	unsigned int x_min;
	unsigned int y_min;
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int int_gpio;
	unsigned int reset_gpio;
    
	unsigned int reset_gpio_flags;
	spinlock_t irq_lock;
	struct mutex report_mutex;
	struct mutex bus_lock;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

int cst3xx_bus_init(struct cst3xx_ts_data *ts_data);  
int cst3xx_bus_exit(struct cst3xx_ts_data *ts_data);  

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

/*****************************************************************************
* Debug interface.
*****************************************************************************/

#if HYN_DEBUG_EN

#define HYN_DEBUG(fmt, args...) do { \
    printk("[HYN_TS]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define HYN_FUNC_ENTER() do { \
    printk("[HYN_TS]%s: Enter(%d)\n", __func__,__LINE__); \
} while (0)

#define HYN_FUNC_EXIT() do { \
    printk("[HYN_TS]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)
#else /* #if HYN_DEBUG_EN*/
#define HYN_DEBUG(fmt, args...)
#define HYN_FUNC_ENTER()
#define HYN_FUNC_EXIT()
#endif

#define HYN_INFO(fmt, args...) do { \
    printk(KERN_INFO "[HYN_TS/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define HYN_ERROR(fmt, args...) do { \
    printk(KERN_ERR "[HYN_TS/E]%s:"fmt"\n", __func__, ##args); \
} while (0)

#endif /* TOUCHSCREEN_CST3XX__H  */


