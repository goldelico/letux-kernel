/* linux/drivers/char/jz_cdbus.c
 *
 * Ingenic cdbus driver, use kernel char device framework
 *
 * Copyright (c) 2021 Ingenic
 * Author:zhangxu <xu.zhang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>

#define VERSION         0x00
#define SETTING         0x04
#define IDLE_WAIT_LEN   0x08
#define TX_PERMIT_LEN   0x0c
#define MAX_IDLE_LEN    0x10
#define TX_PRE_LEN      0x14
#define FILTER          0x18
#define DIV_LS          0x1c
#define DIV_HS          0x20
#define INT_FLAG        0x24
#define INT_MASK        0x28
#define RX_CTRL         0x2c
#define TX_CTRL         0x30
#define RX_PAGE_FLAG    0x34
#define FILTER_M        0x38
#define TX_FIFO         0x200
#define RX_FIFO         0x500
#define SELFTEST        0x900

/* SETTING */
#define BIT_SETTING_TX_PUSH_PULL    (1 << 0)
#define BIT_SETTING_TX_INVERT       (1 << 1)
#define BIT_SETTING_USER_CRC        (1 << 2)
#define BIT_SETTING_NO_DROP         (1 << 3)
#define BIT_SETTING_ARBITRATION     (1 << 4)
#define BIT_SETTING_BREAK_SYNC      (1 << 5)
#define BIT_SETTING_FULL_DUPLEX     (1 << 6)

/* INT_MASK */
#define BIT_MASK_BUS_IDLE           (1 << 0)
#define BIT_MASK_RX_PENDING         (1 << 1)
#define BIT_MASK_RX_BREAK           (1 << 2)
//#define BIT_MASK_RX_LOST            (1 << 3)
#define BIT_MASK_RX_ERROR           (1 << 4)
#define BIT_MASK_TX_PENDING         (1 << 5)
#define BIT_MASK_TX_CD              (1 << 6)
#define BIT_MASK_TX_ERROR           (1 << 7)

/* INT_FLAG */
#define BIT_FLAG_BUS_IDLE           (1 << 0)
#define BIT_FLAG_RX_PENDING         (1 << 1)
#define BIT_FLAG_RX_BREAK           (1 << 2)
//#define BIT_FLAG_RX_LOST            (1 << 3)
#define BIT_FLAG_RX_ERROR           (1 << 4)
#define BIT_FLAG_TX_BUF_CLEAN       (1 << 5)
#define BIT_FLAG_TX_CD              (1 << 6)
#define BIT_FLAG_TX_ERROR           (1 << 7)

/* TX_CTRL */
#define BIT_TX_RST_POINTER          (1 << 0)
#define BIT_TX_START                (1 << 1)
#define BIT_TX_CLR_CD               (1 << 2)
#define BIT_TX_CLR_ERROR            (1 << 3)
#define BIT_TX_ABORT                (1 << 4)
#define BIT_TX_SEND_BREAK           (1 << 5)

/* RX_CTRL */
#define BIT_RX_RST_POINTER          (1 << 0)
#define BIT_RX_SWITCH_PAGE          (1 << 1)
#define BIT_RX_CLR_LOST             (1 << 2)
#define BIT_RX_CLR_ERROR            (1 << 3)
#define BIT_RX_RST                  (1 << 4)
#define BIT_RX_CLR_BREAK            (1 << 5)


#define CDBUS_BUF_SIZE (4 * 6)
#define CDBUS_KERNEL_BUF_SIZE 500

struct cdbus_kernel_buf_info {
	unsigned char data[CDBUS_KERNEL_BUF_SIZE][256];
	unsigned int wr_ptr;
	unsigned int rd_ptr;
};

struct cdbus_buf_info {
	unsigned char data[CDBUS_BUF_SIZE][256];
	unsigned int wr_ptr;
	unsigned int rd_ptr;
};

struct cdbus_info {
	unsigned int tx_flag;
	unsigned int rx_flag;
	unsigned int rx_buf_full;
	unsigned int rx_underrun;
	unsigned int tx_cd_flag;
	unsigned int tx_err_flag;
	unsigned int rx_err_flag;
	unsigned int rx_break_flag;
	unsigned int rx_page;
	struct cdbus_buf_info tx_buf;
	struct cdbus_buf_info rx_buf;
};

struct jz_cdbus {
	int major;
	int minor;
	dev_t dev_num;
	int nr_devs;

	struct class *class;
	struct cdev cdev;
	struct device *dev;

	void __iomem *iomem;
	struct completion done_tx;
	struct completion done_rx;
	struct timer_list timer;

	struct cdbus_info *dev_info;
	struct cdbus_buf_info *tx_buf_info;
	struct cdbus_buf_info *rx_buf_info;

	struct cdbus_kernel_buf_info *rx_buf;
};

enum cdbus_ioctl_cmd {
	CDBUS_A_MODE,
	CDBUS_BS_MODE,
	CDBUS_HAIF_FULL_DUPLEX_MODE,
	CDBUS_FULL_DUPLEX_MODE,
	CDBUS_SET_LOW_RATE,
	CDBUS_SET_HIGH_RATE,
	CDBUS_WRITE_FILTER,
	CDBUS_WRITE_FILTERM,
	CDBUS_ENABLE_LOOPBACK,
};

static struct jz_cdbus *global_cdbus;

static unsigned char *cdbus_kernel_buf_get_wr_ptr(struct cdbus_kernel_buf_info *buf)
{
	return buf->data[buf->wr_ptr % CDBUS_KERNEL_BUF_SIZE];
}

static unsigned char *cdbus_kernel_buf_get_rd_ptr(struct cdbus_kernel_buf_info *buf)
{
	return buf->data[buf->rd_ptr % CDBUS_KERNEL_BUF_SIZE];
}

static int cdbus_kernel_buf_is_full(struct cdbus_kernel_buf_info *buf)
{
	return buf->wr_ptr - buf->rd_ptr == CDBUS_KERNEL_BUF_SIZE;
}

static int cdbus_kernel_buf_is_empty(struct cdbus_kernel_buf_info *buf)
{
	return buf->wr_ptr == buf->rd_ptr;
}

static unsigned char *cdbus_buf_get_wr_ptr(struct cdbus_buf_info *buf)
{
	return buf->data[buf->wr_ptr % CDBUS_BUF_SIZE];
}

static unsigned char *cdbus_buf_get_rd_ptr(struct cdbus_buf_info *buf)
{
	return buf->data[buf->rd_ptr % CDBUS_BUF_SIZE];
}

static int cdbus_buf_is_full(struct cdbus_buf_info *buf)
{
	return buf->wr_ptr - buf->rd_ptr == CDBUS_BUF_SIZE;
}

static int cdbus_buf_is_empty(struct cdbus_buf_info *buf)
{
	return buf->wr_ptr == buf->rd_ptr;
}

static int cdbus_buf_get_valid_len(struct cdbus_buf_info *buf)
{
	return (buf->wr_ptr - buf->rd_ptr);
}

static void ingenic_cdbus_reset_rx_block(struct jz_cdbus *cdbus)
{
	writel(1 << 4, cdbus->iomem + RX_CTRL);
	cdbus->dev_info->rx_page = 0;
}


static unsigned int ingenic_cdbus_recv_frame(struct jz_cdbus *cdbus, unsigned char *buf)
{
	unsigned int frm_len, data_len;
	unsigned char *rx_buf = cdbus_buf_get_rd_ptr(cdbus->rx_buf_info);

	data_len = rx_buf[2] & 0xff;
	frm_len = data_len + 3;
	memcpy(buf, rx_buf, frm_len);

	cdbus->rx_buf_info->rd_ptr++;

	return frm_len;
}

static int ingenic_cdbus_send_frame(struct jz_cdbus *cdbus, unsigned char *buf, int frm_len)
{
	unsigned char *tx_buf = cdbus_buf_get_wr_ptr(cdbus->tx_buf_info);

	memcpy(tx_buf, buf, frm_len);
	cdbus->tx_buf_info->wr_ptr++;

	if (cdbus_buf_get_valid_len(cdbus->tx_buf_info) == 1) {
		writel(readl(cdbus->iomem + INT_MASK) | (1 << 5), cdbus->iomem + INT_MASK);
	}

	return 0;
}

static int ingenic_cdbus_send_frame_to_user(struct jz_cdbus *cdbus, unsigned char **buf)
{
	unsigned int frm_len, data_len;
	unsigned char *rx_buf = cdbus_kernel_buf_get_rd_ptr(cdbus->rx_buf);

	data_len = rx_buf[2] & 0xff;
	frm_len = data_len + 3;

	*buf = rx_buf;
	cdbus->rx_buf->rd_ptr++;

	//printk("%s():%d >>> ##### read buffer >>> wr_ptr = %d, rd_ptr = %d\n", __func__, __LINE__, cdbus->rx_buf->wr_ptr, cdbus->rx_buf->rd_ptr);
	return frm_len;
}

static unsigned int ingenic_cdbus_recv_frame_from_mcu(struct jz_cdbus *cdbus)
{
	unsigned char *rx_buf;
	unsigned int frm_len;

	while (!cdbus_buf_is_empty(cdbus->rx_buf_info)) {
		rx_buf = cdbus_kernel_buf_get_wr_ptr(cdbus->rx_buf);
		frm_len = ingenic_cdbus_recv_frame(cdbus, rx_buf);
		cdbus->rx_buf->wr_ptr++;
		//printk("%s():%d >>> ##### write buffer >>> wr_ptr = %d, rd_ptr = %d\n", __func__, __LINE__, cdbus->rx_buf->wr_ptr, cdbus->rx_buf->rd_ptr);
	}

	return 0;
}

extern void mygpio_cdbus_output(unsigned gpio, int value);
int ingenic_cdbus_irq_handler(void)
{
	struct jz_cdbus *cdbus = global_cdbus;

	/* Receive */
	if (cdbus->dev_info->rx_flag == 1) {
#if 1
		if (!cdbus_kernel_buf_is_full(cdbus->rx_buf)) {
	//		printk("%s:%d >>> kernel buffer is not full!\n", __func__, __LINE__);
			ingenic_cdbus_recv_frame_from_mcu(cdbus);
			//complete(&cdbus->done_rx);
		} else {
			cdbus->rx_buf->rd_ptr = 0;
			cdbus->rx_buf->wr_ptr = 0;
			printk("%s:%d >>> kernel buffer is full!\n", __func__, __LINE__);
		}
#endif
		cdbus->dev_info->rx_flag = 0;
	}

	/* Rxfifo underrun */
	if (cdbus->dev_info->rx_underrun != 0) {
		//printk("Rxfifo Underrun:%d: wr_ptr = %d, rd_ptr = %d, rx_underrun = %d\n", __LINE__, cdbus->rx_buf_info->wr_ptr, cdbus->rx_buf_info->rd_ptr, cdbus->dev_info->rx_underrun);
		printk("ru%d %d\n", cdbus->rx_buf_info->wr_ptr, cdbus->rx_buf_info->rd_ptr);
		cdbus->dev_info->rx_underrun = 0;
	}

	/* Rxfifo full */
	if (cdbus->dev_info->rx_buf_full != 0) {
		//printk("Rxfifo Full:%d: wr_ptr = %d, rd_ptr = %d, rx_buf_full = %d\n", __LINE__, cdbus->rx_buf_info->wr_ptr, cdbus->rx_buf_info->rd_ptr, cdbus->dev_info->rx_buf_full);
		printk("rf%d %d\n", cdbus->rx_buf_info->wr_ptr, cdbus->rx_buf_info->rd_ptr);
		cdbus->dev_info->rx_buf_full = 0;
		cdbus->rx_buf_info->rd_ptr = 0;
		cdbus->rx_buf_info->wr_ptr = 0;
	}

#if 1
	/* Send */
	if (cdbus->dev_info->tx_flag == 1) {
		if (!cdbus_buf_is_full(cdbus->tx_buf_info)) {
	//		printk("########%s:%d >>> tx_buf is not full\n", __func__, __LINE__);
			complete(&cdbus->done_tx);
		}

		cdbus->dev_info->tx_flag = 0;
	}
#endif

	/* Rx break */
	if (cdbus->dev_info->rx_break_flag == 1) {
		dev_err(cdbus->dev, "%s():%d >>> Receive break character\n", __func__, __LINE__);
		cdbus->dev_info->rx_break_flag = 0;
	}

	/* Rx error */
	if (cdbus->dev_info->rx_err_flag == 1) {
		dev_err(cdbus->dev, "%s():%d >>> RX error: frame broken\n", __func__, __LINE__);
		cdbus->dev_info->rx_err_flag = 0;
	}

	/* Tx collision detected */
	if (cdbus->dev_info->tx_cd_flag == 1) {
		dev_err(cdbus->dev, "%s():%d >>> TX collision detected\n", __func__, __LINE__);
		cdbus->dev_info->tx_cd_flag = 0;
	}

	/* Tx error */
	if (cdbus->dev_info->tx_err_flag == 1) {
		dev_err(cdbus->dev, "%s():%d >>> TX error: tx is 0, but rx is 1\n", __func__, __LINE__);
		cdbus->dev_info->tx_err_flag = 0;
	}

	return 0;
}

static void ingenic_cdbus_timer_handler(unsigned long data)
{
	struct jz_cdbus *cdbus = (struct jz_cdbus *)data;

	if (!cdbus_kernel_buf_is_empty(cdbus->rx_buf)) {
		complete(&cdbus->done_rx);
	} else {
		printk("%s:%d >>> kernel buffer is empty\n", __func__, __LINE__);
	}

	//cdbus->timer.expires = jiffies + msecs_to_jiffies(50);
	cdbus->timer.expires = jiffies + usecs_to_jiffies(100);
	add_timer(&cdbus->timer);
}

//#define CDBUS_CPU_BREAK_SYNC_MODE
/*
 * Init cdbus controler
 */
static void jz_cdbus_hw_init(struct jz_cdbus *cdbus)
{
	/* Enable tx push-pull output */
#ifndef CDBUS_CPU_BREAK_SYNC_MODE
	printk("%s:%d >>> mode CDBUS-A mode\n", __func__, __LINE__);
	writel(BIT_SETTING_ARBITRATION | BIT_SETTING_TX_PUSH_PULL, cdbus->iomem + SETTING);
#else
	printk("%s:%d >>> mode CDBUS-BS mode\n", __func__, __LINE__);
	writel(BIT_SETTING_BREAK_SYNC | BIT_SETTING_TX_PUSH_PULL, cdbus->iomem + SETTING);
#endif

	/* Set baudrates */
	// CDBUS dve_clk = 150Mhz
#ifndef CDBUS_CPU_BREAK_SYNC_MODE
	/* a mode */
	printk("%s:%d >>> clock CDBUS-A mode\n", __func__, __LINE__);
	writel(149, cdbus->iomem + DIV_LS);	// low_speed rate = 1Mhz
	writel(14, cdbus->iomem + DIV_HS);	// high_speed rate = 10Mhz
#else
	/* bs mode */
	printk("%s:%d >>> clock CDBUS-BS mode\n", __func__, __LINE__);
	writel(14, cdbus->iomem + DIV_LS);	// low_speed rate = 50Mhz
	writel(14, cdbus->iomem + DIV_HS);	// high_speed rate = 50Mhz
#endif

	/* Clean RX buffer */
	//writel(BIT_RX_RST_POINTER, cdbus->iomem + RX_CTRL);

	/* Enable interrupts */
#ifndef CDBUS_CPU_BREAK_SYNC_MODE
	printk("%s:%d >>> interrupt CDBUS-A mode\n", __func__, __LINE__);
	writel(BIT_MASK_TX_ERROR | BIT_MASK_TX_CD | BIT_MASK_RX_ERROR |
		BIT_MASK_RX_BREAK | BIT_MASK_RX_PENDING, cdbus->iomem + INT_MASK);
#else
	printk("%s:%d >>> interrupt CDBUS-BS mode\n", __func__, __LINE__);
	writel(BIT_MASK_TX_ERROR | BIT_MASK_TX_CD | BIT_MASK_RX_ERROR |
		BIT_MASK_RX_PENDING, cdbus->iomem + INT_MASK);
#endif

	/* Set time length */
	writel(1, cdbus->iomem + TX_PERMIT_LEN);
	writel(1, cdbus->iomem + MAX_IDLE_LEN);
	writel(1, cdbus->iomem + IDLE_WAIT_LEN);

	/* Set local address and Multicast filter */
	writel(0x0, cdbus->iomem + FILTER);
	writel(0x0, cdbus->iomem + FILTER_M);
}

static int jz_cdbus_open(struct inode *inode, struct file *filp)
{
	struct jz_cdbus *cdbus;
	unsigned int mcu_ddr = *(volatile unsigned int *)(0xb342201c);

	cdbus = container_of(inode->i_cdev, struct jz_cdbus, cdev);
	if (!cdbus)
		return -EINVAL;
	filp->private_data = cdbus;

#if 1
	mcu_ddr = ((mcu_ddr & 0xffff) + 0x2000) | 0xb3420000;
	cdbus->dev_info = (struct cdbus_info *)mcu_ddr;

	cdbus->tx_buf_info = &cdbus->dev_info->tx_buf;
	cdbus->rx_buf_info = &cdbus->dev_info->rx_buf;
	cdbus->rx_buf_info->rd_ptr = 0;
	cdbus->rx_buf_info->wr_ptr = 0;
	cdbus->tx_buf_info->rd_ptr = 0;
	cdbus->tx_buf_info->wr_ptr = 0;
	cdbus->dev_info->rx_underrun = 0;
	cdbus->dev_info->rx_buf_full = 0;
	cdbus->dev_info->tx_cd_flag = 0;
	cdbus->dev_info->tx_err_flag = 0;
	cdbus->dev_info->rx_err_flag = 0;
	cdbus->dev_info->rx_break_flag = 0;
	cdbus->dev_info->rx_page = 0;

	//cdbus->rx_buf = devm_kzalloc(cdbus->dev, sizeof(struct cdbus_kernel_buf_info), GFP_KERNEL);
	cdbus->rx_buf = kzalloc(sizeof(struct cdbus_kernel_buf_info), GFP_KERNEL);
	if (!cdbus->rx_buf) {
		printk("%s:%d >>> rx_buf kzalloc failed!\n", __func__, __LINE__);
		return -ENOMEM;
	}


	cdbus->rx_buf->rd_ptr = 0;
	cdbus->rx_buf->wr_ptr = 0;
#endif

	jz_cdbus_hw_init(cdbus);

	return 0;
}

int cdbus_debug = 0;
static int jz_cdbus_read(struct file *filp, char *user_buf, size_t count, loff_t *f_pos)
{
	struct jz_cdbus *cdbus = filp->private_data;
	//unsigned char rx_buf[256];
	unsigned char *rx_buf;
	int frm_len;

	if (!cdbus_debug) {
		printk("cdbus debug\n");
		cdbus_debug = 1;
	}

#if 1
	if (cdbus_kernel_buf_is_empty(cdbus->rx_buf)) {
		//printk("%s:%d >>> kernel buffer is empty\n", __func__, __LINE__);
		return -1;
	}
#endif
#if 0
	wait_for_completion_interruptible(&cdbus->done_rx);
#endif
	frm_len = ingenic_cdbus_send_frame_to_user(cdbus, &rx_buf);
	copy_to_user(user_buf, rx_buf, frm_len);

	return frm_len;
}

static int jz_cdbus_write(struct file *filp, const char *user_buf, size_t count, loff_t *f_pos)
{
	struct jz_cdbus *cdbus = filp->private_data;
	unsigned char tx_buf[256];

	copy_from_user(tx_buf, user_buf, count);
	ingenic_cdbus_send_frame(cdbus, tx_buf, count);
#if 1
	wait_for_completion_interruptible(&cdbus->done_tx);
#endif

	return count;
}

static long jz_cdbus_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int ret = 0;
	struct jz_cdbus *cdbus = filp->private_data;

	switch (cmd) {
	case CDBUS_A_MODE:
		writel(readl(cdbus->iomem + SETTING) & ~(0x7 << 4), cdbus->iomem + SETTING);
		writel(readl(cdbus->iomem + SETTING) | BIT_SETTING_ARBITRATION, cdbus->iomem + SETTING);
		break;
	case CDBUS_BS_MODE:
		writel(readl(cdbus->iomem + SETTING) & ~(0x7 << 4), cdbus->iomem + SETTING);
		writel(readl(cdbus->iomem + SETTING) | BIT_SETTING_BREAK_SYNC, cdbus->iomem + SETTING);
		break;
	case CDBUS_HAIF_FULL_DUPLEX_MODE:
		writel(readl(cdbus->iomem + SETTING) & ~(0x7 << 4), cdbus->iomem + SETTING);
		break;
	case CDBUS_FULL_DUPLEX_MODE:
		writel(readl(cdbus->iomem + SETTING) & ~(0x7 << 4), cdbus->iomem + SETTING);
		writel(readl(cdbus->iomem + SETTING) | BIT_SETTING_FULL_DUPLEX, cdbus->iomem + SETTING);
		break;
	case CDBUS_SET_LOW_RATE:
		writel(150 / args - 1, cdbus->iomem + DIV_LS);
		break;
	case CDBUS_SET_HIGH_RATE:
		writel(150 / args - 1, cdbus->iomem + DIV_HS);
		break;
	case CDBUS_WRITE_FILTER:
		writel(args, cdbus->iomem + FILTER);
		break;
	case CDBUS_WRITE_FILTERM:
		writel(args, cdbus->iomem + FILTER_M);
		break;
	case CDBUS_ENABLE_LOOPBACK:
		dev_info(cdbus->dev, "Loopback Mode\n");
		writel(0x1, cdbus->iomem + SELFTEST);
		break;
	default:
		break;
	}

	return ret;
}

static int jz_cdbus_close(struct inode *inode, struct file *filp)
{
	struct jz_cdbus *cdbus = filp->private_data;

	/* Disable interrupts */
	writel(0x0, cdbus->iomem + INT_MASK);
	kfree(cdbus->rx_buf);

	return 0;
}

static struct file_operations jz_cdbus_ops = {
	.owner = THIS_MODULE,
	.write = jz_cdbus_write,
	.read = jz_cdbus_read,
	.open = jz_cdbus_open,
	.release = jz_cdbus_close,
	.unlocked_ioctl = jz_cdbus_ioctl,
};

static int jz_cdbus_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct jz_cdbus *cdbus;
	struct resource *res;
	dev_t devno;
	struct clk *clk_cgu;
	struct clk *clk_gate;
	unsigned int cdbus_clk_freq = 150000000;

	cdbus = devm_kzalloc(&pdev->dev, sizeof(struct jz_cdbus), GFP_KERNEL);
	if (!cdbus)
		return -ENOMEM;

	global_cdbus = cdbus;

	cdbus->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		return -ENODEV;
	}

	cdbus->iomem = devm_ioremap(cdbus->dev, res->start, res->end - res->start + 1);
	if (!cdbus->iomem) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, cdbus);

	init_completion(&cdbus->done_tx);
	init_completion(&cdbus->done_rx);

	clk_gate = devm_clk_get(&pdev->dev, "gate_cdbus");
	if (IS_ERR(clk_gate)) {
		dev_err(&pdev->dev, "%s Cannot get clock: %s\n", __func__, "gate_cdbus");
		goto __err0;
	}

	clk_cgu = devm_clk_get(&pdev->dev, "div_cdbus");
	if (IS_ERR(clk_cgu)) {
		dev_err(&pdev->dev, "%s Cannot get clock: %s\n", __func__, "div_cdbus");
		goto __err0;
	}

	clk_set_parent(clk_get(NULL, "mux_cdbus"), clk_get(NULL, "epll"));

	if (clk_set_rate(clk_cgu, cdbus_clk_freq)) {
		dev_err(&pdev->dev, "Set cgu_cdbus clk rate faild\n");
		goto __err0;
	}

	if ((ret = clk_prepare_enable(clk_gate)) < 0) {
		dev_err(&pdev->dev, "Enable gate_cdbus clk failed\n");
		goto __err0;
	}

	//printk("############# cdbus clock: clk_get_clk = %ld, set clk = %d\n", clk_get_rate(clk_cgu), cdbus_clk_freq);

	cdbus->minor = 0;
	cdbus->nr_devs = 1;
	ret = alloc_chrdev_region(&devno, cdbus->minor, cdbus->nr_devs, "jz-cdbus");
	if (ret) {
		dev_err(cdbus->dev, "alloc chrdev failed\n");
		return ret;
	}
	cdbus->major = MAJOR(devno);
	cdbus->dev_num = MKDEV(cdbus->major, cdbus->minor);
	dev_dbg(&pdev->dev, "%s():%d >>> cdbus->major = %d, cdbus->minor = %d, cdbus->dev_num = %x\n",
			__func__, __LINE__, cdbus->major, cdbus->minor, cdbus->dev_num);

	cdev_init(&cdbus->cdev, &jz_cdbus_ops);
	cdbus->cdev.owner = THIS_MODULE;
	cdev_add(&cdbus->cdev, cdbus->dev_num, 1);
	if (ret) {
		dev_err(cdbus->dev, "cdev_add failed\n");
		goto __err1;
	}

	cdbus->class = class_create(THIS_MODULE, "jz-cdbus");
	if (IS_ERR(cdbus->class)) {
		dev_err(cdbus->dev, "class_create failed\n");
		ret = PTR_ERR(cdbus->class);
		goto __err2;
	}

	cdbus->dev = device_create(cdbus->class, NULL, cdbus->dev_num, NULL, "cdbus");
	if (IS_ERR(cdbus->dev)) {
		dev_err(cdbus->dev, "device_create failed\n");
		ret = PTR_ERR(cdbus->dev);
		goto __err3;
	}

	init_timer(&cdbus->timer);
	cdbus->timer.expires = jiffies + msecs_to_jiffies(100);
	cdbus->timer.function = ingenic_cdbus_timer_handler;
	cdbus->timer.data = (unsigned long)cdbus;

	dev_info(cdbus->dev, "Ingenic cdbus driver init successfully!\n");

	return 0;

__err3:
	class_destroy(cdbus->class);
__err2:
	cdev_del(&cdbus->cdev);
__err1:
	unregister_chrdev_region(cdbus->dev_num, cdbus->nr_devs);
__err0:
	return ret;
}

static int jz_cdbus_remove(struct platform_device *pdev)
{
	struct jz_cdbus *cdbus = platform_get_drvdata(pdev);

	device_destroy(cdbus->class, cdbus->dev_num);
	class_destroy(cdbus->class);
	cdev_del(&cdbus->cdev);
	unregister_chrdev_region(cdbus->dev_num, cdbus->nr_devs);

	return 0;
}

static const struct of_device_id ingenic_cdbus_match[] = {
	{ .compatible = "ingenic,x1600-cdbus", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_cdbus_match);

static struct platform_driver jz_cdbus_driver = {
	.probe          = jz_cdbus_probe,
	.remove         = jz_cdbus_remove,
	.driver		= {
		.name	= "jz-cdbus",
		.owner	= THIS_MODULE,
		.of_match_table	= ingenic_cdbus_match,
	},
};

static int __init jz_cdbus_init(void)
{
	int ret;

	ret = platform_driver_register(&jz_cdbus_driver);
	if (ret)
		platform_driver_unregister(&jz_cdbus_driver);

	return ret;
}

static void __exit jz_cdbus_exit(void)
{
	platform_driver_unregister(&jz_cdbus_driver);
}

module_init(jz_cdbus_init);
module_exit(jz_cdbus_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic cdbus driver");
MODULE_AUTHOR("xu.zhang@ingenic.com");

