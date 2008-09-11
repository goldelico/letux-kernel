/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Pötzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c2410 (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C2410_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2007-05-23: Harald Welte <laforge@openmoko.org>
 * 	- Add proper support for S32440
 *
 * 2008-06-18: Andy Green <andy@openmoko.com>
 *      - Outlier removal
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/ts.h>

#include <asm/plat-s3c/regs-adc.h>

/* For ts.dev.id.version */
#define S3C2410TSVERSION	0x0101

#define TSC_SLEEP  (S3C2410_ADCTSC_PULL_UP_DISABLE | S3C2410_ADCTSC_XY_PST(0))

#define WAIT4INT(x)  (((x)<<8) | \
		     S3C2410_ADCTSC_YM_SEN | \
		     S3C2410_ADCTSC_YP_SEN | \
		     S3C2410_ADCTSC_XP_SEN | \
		     S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C2410_ADCTSC_YM_SEN | \
		      S3C2410_ADCTSC_YP_SEN | \
		      S3C2410_ADCTSC_XP_SEN | \
		      S3C2410_ADCTSC_AUTO_PST | \
		      S3C2410_ADCTSC_XY_PST(0))

#define DEBUG_LVL    KERN_DEBUG

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>");
MODULE_DESCRIPTION("s3c2410 touchscreen driver");
MODULE_LICENSE("GPL");

/*
 * Definitions & global arrays.
 */


static char *s3c2410ts_name = "s3c2410 TouchScreen";

/*
 * Per-touchscreen data.
 */

struct s3c2410ts_sample {
	int x;
	int y;
};

struct s3c2410ts {
	struct input_dev *dev;
	long xp;
	long yp;
	int count;
	int shift;
	int extent;  /* 1 << shift */

	/* the raw sample fifo is a lightweight way to track a running average
	 * of all taken samples.  "running average" here means that it gives
	 * correct average for each sample, not only at the end of block of
	 * samples
	 */
	int excursion_filter_len;
	struct s3c2410ts_sample *raw_sample_fifo;
	int head_raw_fifo;
	int tail_raw_fifo;
	struct s3c2410ts_sample raw_running_avg;
	int reject_threshold_vs_avg;
	int flag_previous_exceeded_threshold;
	int flag_first_touch_sent;
};

static struct s3c2410ts ts;
static void __iomem *base_addr;

static void clear_raw_fifo(void)
{
	ts.head_raw_fifo = 0;
	ts.tail_raw_fifo = 0;
	ts.raw_running_avg.x = 0;
	ts.raw_running_avg.y = 0;
	ts.flag_previous_exceeded_threshold = 0;
	ts.flag_first_touch_sent = 0;
}


static inline void s3c2410_ts_connect(void)
{
	s3c2410_gpio_cfgpin(S3C2410_GPG12, S3C2410_GPG12_XMON);
	s3c2410_gpio_cfgpin(S3C2410_GPG13, S3C2410_GPG13_nXPON);
	s3c2410_gpio_cfgpin(S3C2410_GPG14, S3C2410_GPG14_YMON);
	s3c2410_gpio_cfgpin(S3C2410_GPG15, S3C2410_GPG15_nYPON);
}

static void touch_timer_fire(unsigned long data)
{
  	unsigned long data0;
  	unsigned long data1;
	int updown;

	data0 = readl(base_addr + S3C2410_ADCDAT0);
	data1 = readl(base_addr + S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) &&
					    (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	// if we need to send an untouch event, but we haven't yet sent the
	// touch event (this happens if the touchscreen was tapped lightly),
	// send the touch event first
	if (!updown && !ts.flag_first_touch_sent && ts.count != 0) {
		input_report_abs(ts.dev, ABS_X, ts.xp >> ts.shift);
		input_report_abs(ts.dev, ABS_Y, ts.yp >> ts.shift);

		input_report_key(ts.dev, BTN_TOUCH, 1);
		input_report_abs(ts.dev, ABS_PRESSURE, 1);
		input_sync(ts.dev);
		ts.flag_first_touch_sent = 1;
	}

	if (updown) {
		if (ts.count != 0) {
			ts.xp >>= ts.shift;
			ts.yp >>= ts.shift;

#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
			{
				struct timeval tv;

				do_gettimeofday(&tv);
				printk(DEBUG_LVL "T:%06d, X:%03ld, Y:%03ld\n",
						 (int)tv.tv_usec, ts.xp, ts.yp);
			}
#endif

			input_report_abs(ts.dev, ABS_X, ts.xp);
			input_report_abs(ts.dev, ABS_Y, ts.yp);

			input_report_key(ts.dev, BTN_TOUCH, 1);
			input_report_abs(ts.dev, ABS_PRESSURE, 1);
			input_sync(ts.dev);
			ts.flag_first_touch_sent = 1;
		}

		ts.xp = 0;
		ts.yp = 0;
		ts.count = 0;

		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST,
						      base_addr+S3C2410_ADCTSC);
		writel(readl(base_addr+S3C2410_ADCCON) |
			 S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);
	} else {
		ts.count = 0;

		input_report_key(ts.dev, BTN_TOUCH, 0);
		input_report_abs(ts.dev, ABS_PRESSURE, 0);
		input_sync(ts.dev);
		ts.flag_first_touch_sent = 0;

		writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);
	}
}

static struct timer_list touch_timer =
		TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irq, void *dev_id)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) &&
					    (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	/* TODO we should never get an interrupt with updown set while
	 * the timer is running, but maybe we ought to verify that the
	 * timer isn't running anyways. */

	if (updown)
		touch_timer_fire(0);

	return IRQ_HANDLED;
}


static irqreturn_t stylus_action(int irq, void *dev_id)
{
	unsigned long x;
	unsigned long y;
	int length = (ts.head_raw_fifo - ts.tail_raw_fifo) & (ts.extent - 1);
	int scaled_avg_x;
	int scaled_avg_y;

	x = readl(base_addr + S3C2410_ADCDAT0) & S3C2410_ADCDAT0_XPDATA_MASK;
	y = readl(base_addr + S3C2410_ADCDAT1) & S3C2410_ADCDAT1_YPDATA_MASK;

	if (!length)
		goto store_sample;

	scaled_avg_x = ts.raw_running_avg.x / length;
	scaled_avg_y = ts.raw_running_avg.y / length;

	/* we appear to accept every sample into both the running average FIFO
	 * and the summing average.  BUT, if the last sample crossed a
	 * machine-set threshold, each time we do a beauty contest
	 * on the new sample comparing if it is closer to the running
	 * average and the previous sample.  If it is closer to the previous
	 * suspicious sample, we assume the change is real and accept both
	 * if the new sample has returned to being closer to the average than
	 * the previous sample, we take the previous sample as an excursion
	 * and overwrite it in both the running average and summing average.
	 */

	if (ts.flag_previous_exceeded_threshold)
		/* new one closer to "nonconformist" previous, or average?
		 * Pythagoras?  Who?  Don't need it because large excursion
		 * will be accounted for correctly this way
		 */
		if ((abs(x - scaled_avg_x) + abs(y - scaled_avg_y)) <
		    (abs(x - ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							  (ts.extent - 1)].x) +
		     abs(y - ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							 (ts.extent - 1)].y))) {
			/* it's closer to average, reject previous as a one-
			 * shot excursion, by overwriting it
			 */
			ts.xp += x - ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							     (ts.extent - 1)].x;
			ts.yp += y - ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							     (ts.extent - 1)].y;
			ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							 (ts.extent - 1)].x = x;
			ts.raw_sample_fifo[(ts.head_raw_fifo - 1) &
							 (ts.extent - 1)].y = y;
			/* no new sample: replaced previous, so we are done */
			goto completed;
		}
		/* else it was closer to nonconformist previous: it's likely
		 * a genuine consistent move then.
		 * Keep previous and add new guy.
		 */

	if ((x >= scaled_avg_x - ts.reject_threshold_vs_avg) &&
	    (x <= scaled_avg_x + ts.reject_threshold_vs_avg) &&
	    (y >= scaled_avg_y - ts.reject_threshold_vs_avg) &&
	    (y <= scaled_avg_y + ts.reject_threshold_vs_avg))
		ts.flag_previous_exceeded_threshold = 0;
	else
		ts.flag_previous_exceeded_threshold = 1;

store_sample:
	ts.xp += x;
	ts.yp += y;
	ts.count++;

	/* remove oldest sample from avg when we have full pipeline */
	if (((ts.head_raw_fifo + 1) & (ts.extent - 1)) == ts.tail_raw_fifo) {
		ts.raw_running_avg.x -= ts.raw_sample_fifo[ts.tail_raw_fifo].x;
		ts.raw_running_avg.y -= ts.raw_sample_fifo[ts.tail_raw_fifo].y;
		ts.tail_raw_fifo = (ts.tail_raw_fifo + 1) & (ts.extent - 1);
	}
	/* always add current sample to fifo and average */
	ts.raw_sample_fifo[ts.head_raw_fifo].x = x;
	ts.raw_sample_fifo[ts.head_raw_fifo].y = y;
	ts.raw_running_avg.x += x;
	ts.raw_running_avg.y += y;
	ts.head_raw_fifo = (ts.head_raw_fifo + 1) & (ts.extent - 1);

completed:
	if (ts.count >= (1 << ts.shift)) {
		mod_timer(&touch_timer, jiffies + 1);
		writel(WAIT4INT(1), base_addr+S3C2410_ADCTSC);
		goto bail;
	}

	writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST,
						      base_addr+S3C2410_ADCTSC);
	writel(readl(base_addr+S3C2410_ADCCON) |
			 S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);

bail:
	return IRQ_HANDLED;
}

static struct clk	*adc_clock;

/*
 * The functions for inserting/removing us as a module.
 */

static int __init s3c2410ts_probe(struct platform_device *pdev)
{
	int rc;
	struct s3c2410_ts_mach_info *info;
	struct input_dev *input_dev;

	info = (struct s3c2410_ts_mach_info *)pdev->dev.platform_data;

	if (!info)
	{
		dev_err(&pdev->dev, "Hm... too bad: no platform data for ts\n");
		return -EINVAL;
	}

#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
	printk(DEBUG_LVL "Entering s3c2410ts_init\n");
#endif

	adc_clock = clk_get(NULL, "adc");
	if (!adc_clock) {
		dev_err(&pdev->dev, "failed to get adc clock source\n");
		return -ENOENT;
	}
	clk_enable(adc_clock);

#ifdef CONFIG_TOUCHSCREEN_S3C2410_DEBUG
	printk(DEBUG_LVL "got and enabled clock\n");
#endif

	base_addr = ioremap(S3C2410_PA_ADC,0x20);
	if (base_addr == NULL) {
		dev_err(&pdev->dev, "Failed to remap register block\n");
		return -ENOMEM;
	}


	/* If we acutally are a S3C2410: Configure GPIOs */
	if (!strcmp(pdev->name, "s3c2410-ts"))
		s3c2410_ts_connect();

	if ((info->presc & 0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN |
		       S3C2410_ADCCON_PRSCVL(info->presc&0xFF),
						    base_addr + S3C2410_ADCCON);
	else
		writel(0, base_addr+S3C2410_ADCCON);


	/* Initialise registers */
	if ((info->delay & 0xffff) > 0)
		writel(info->delay & 0xffff,  base_addr + S3C2410_ADCDLY);

	writel(WAIT4INT(0), base_addr + S3C2410_ADCTSC);

	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct s3c2410ts));
	input_dev = input_allocate_device();

	if (!input_dev) {
		dev_err(&pdev->dev, "Unable to allocate the input device\n");
		return -ENOMEM;
	}

	ts.dev = input_dev;
	ts.dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
			   BIT_MASK(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(ts.dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, 1, 0, 0);

	ts.dev->private = &ts;
	ts.dev->name = s3c2410ts_name;
	ts.dev->id.bustype = BUS_RS232;
	ts.dev->id.vendor = 0xDEAD;
	ts.dev->id.product = 0xBEEF;
	ts.dev->id.version = S3C2410TSVERSION;

	ts.shift = info->oversampling_shift;
	ts.extent = 1 << info->oversampling_shift;
	ts.reject_threshold_vs_avg = info->reject_threshold_vs_avg;
	ts.excursion_filter_len = 1 << info->excursion_filter_len_bits;

	ts.raw_sample_fifo = kmalloc(sizeof(struct s3c2410ts_sample) *
					   ts.excursion_filter_len, GFP_KERNEL);
	clear_raw_fifo();

	/* Get irqs */
	if (request_irq(IRQ_ADC, stylus_action, IRQF_SAMPLE_RANDOM,
						    "s3c2410_action", ts.dev)) {
		dev_err(&pdev->dev, "Could not allocate ts IRQ_ADC !\n");
		iounmap(base_addr);
		return -EIO;
	}
	if (request_irq(IRQ_TC, stylus_updown, IRQF_SAMPLE_RANDOM,
			"s3c2410_action", ts.dev)) {
		dev_err(&pdev->dev, "Could not allocate ts IRQ_TC !\n");
		free_irq(IRQ_ADC, ts.dev);
		iounmap(base_addr);
		return -EIO;
	}

	dev_info(&pdev->dev, "successfully loaded\n");

	/* All went ok, so register to the input system */
	rc = input_register_device(ts.dev);
	if (rc) {
		free_irq(IRQ_TC, ts.dev);
		free_irq(IRQ_ADC, ts.dev);
		clk_disable(adc_clock);
		iounmap(base_addr);
		return -EIO;
	}

	return 0;
}

static int s3c2410ts_remove(struct platform_device *pdev)
{
	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);
	free_irq(IRQ_TC,ts.dev);
	free_irq(IRQ_ADC,ts.dev);

	if (adc_clock) {
		clk_disable(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}

	kfree(ts.raw_sample_fifo);

	input_unregister_device(ts.dev);
	iounmap(base_addr);

	return 0;
}

#ifdef CONFIG_PM
static int s3c2410ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	writel(TSC_SLEEP, base_addr+S3C2410_ADCTSC);
	writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_STDBM,
	       base_addr+S3C2410_ADCCON);

	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);

	clk_disable(adc_clock);

	return 0;
}

static int s3c2410ts_resume(struct platform_device *pdev)
{
	struct s3c2410_ts_mach_info *info =
		( struct s3c2410_ts_mach_info *)pdev->dev.platform_data;

	clk_enable(adc_clock);
	mdelay(1);

	clear_raw_fifo();

	enable_irq(IRQ_ADC);
	enable_irq(IRQ_TC);

	if ((info->presc&0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN |
		       S3C2410_ADCCON_PRSCVL(info->presc&0xFF),
						      base_addr+S3C2410_ADCCON);
	else
		writel(0,base_addr+S3C2410_ADCCON);

	/* Initialise registers */
	if ((info->delay & 0xffff) > 0)
		writel(info->delay & 0xffff,  base_addr+S3C2410_ADCDLY);

	writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);

	return 0;
}

#else
#define s3c2410ts_suspend NULL
#define s3c2410ts_resume  NULL
#endif

static struct platform_driver s3c2410ts_driver = {
       .driver         = {
	       .name   = "s3c2410-ts",
	       .owner  = THIS_MODULE,
       },
       .probe          = s3c2410ts_probe,
       .remove         = s3c2410ts_remove,
       .suspend        = s3c2410ts_suspend,
       .resume         = s3c2410ts_resume,

};

static struct platform_driver s3c2440ts_driver = {
       .driver         = {
	       .name   = "s3c2440-ts",
	       .owner  = THIS_MODULE,
       },
       .probe          = s3c2410ts_probe,
       .remove         = s3c2410ts_remove,
       .suspend        = s3c2410ts_suspend,
       .resume         = s3c2410ts_resume,

};

static int __init s3c2410ts_init(void)
{
	int rc;

	rc = platform_driver_register(&s3c2410ts_driver);
	if (rc < 0)
		return rc;

	rc = platform_driver_register(&s3c2440ts_driver);
	if (rc < 0)
		platform_driver_unregister(&s3c2410ts_driver);

	return rc;
}

static void __exit s3c2410ts_exit(void)
{
	platform_driver_unregister(&s3c2440ts_driver);
	platform_driver_unregister(&s3c2410ts_driver);
}

module_init(s3c2410ts_init);
module_exit(s3c2410ts_exit);

/*
    Local variables:
        compile-command: "make ARCH=arm CROSS_COMPILE=/usr/local/arm/3.3.2/bin/arm-linux- -k -C ../../.."
        c-basic-offset: 8
    End:
*/
