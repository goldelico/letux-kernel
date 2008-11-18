/*
 * Keyboard driver for E-TEN M800 GSM phone
 *
 * (C) 2008 by Harald Welte <laforge@gnumonks.org>
 * All rights reserved.
 *
 * inspired by corkgbd.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>
#include <asm/mach-types.h>

struct m800kbd {
	struct input_dev *input;
	unsigned int suspended;
	struct work_struct work;
	int work_in_progress;
	int hp_irq_count_in_work;
	int hp_irq_count;
	int jack_irq;
};

static irqreturn_t m800kbd_power_irq(int irq, void *dev_id)
{
	struct m800kbd *m800kbd_data = dev_id;
	int key_pressed = !gpio_get_value(irq_to_gpio(irq));

	input_report_key(m800kbd_data->input, KEY_POWER, key_pressed);
	input_sync(m800kbd_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800kbd_cam_irq(int irq, void *dev_id)
{
	struct m800kbd *m800kbd_data = dev_id;

	int key_pressed = !gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800kbd_data->input, KEY_CAMERA, key_pressed);
	input_sync(m800kbd_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800kbd_rec_irq(int irq, void *dev_id)
{
	struct m800kbd *m800kbd_data = dev_id;

	int key_pressed = !gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800kbd_data->input, KEY_RECORD, key_pressed);
	input_sync(m800kbd_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800kbd_slide_irq(int irq, void *dev_id)
{
	struct m800kbd *m800kbd_data = dev_id;

	int key_pressed = gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800kbd_data->input, SW_LID, key_pressed);
	input_sync(m800kbd_data->input);

	return IRQ_HANDLED;
}

#if 0
static void m800kbd_debounce_jack(struct work_struct *work)
{
	struct m800kbd *kbd = container_of(work, struct m800kbd, work);
	unsigned long flags;
	int loop = 0;

	do {
		/*
		 * we wait out any multiple interrupt
		 * stuttering in 100ms lumps
		 */
		do {
			kbd->hp_irq_count_in_work = kbd->hp_irq_count;
			msleep(100);
		} while (kbd->hp_irq_count != kbd->hp_irq_count_in_work);
		/*
		 * no new interrupts on jack for 100ms...
		 * ok we will report it
		 */
		input_report_switch(kbd->input, SW_HEADPHONE_INSERT,
				    gpio_get_value(irq_to_gpio(kbd->jack_irq)));
		input_sync(kbd->input);
		/*
		 * we go around the outer loop again if we detect that more
		 * interrupts came while we are servicing here.  But we have
		 * to sequence it carefully with interrupts off
		 */
		local_save_flags(flags);
		/* no interrupts during this work means we can exit the work */
		loop = !!(kbd->hp_irq_count != kbd->hp_irq_count_in_work);
		if (!loop)
			kbd->work_in_progress = 0;
		local_irq_restore(flags);
		/*
		 * interrupt that comes here will either queue a new work action
		 * since work_in_progress is cleared now, or be dealt with
		 * when we loop.
		 */
	} while (loop);
}


static irqreturn_t m800kbd_headphone_irq(int irq, void *dev_id)
{
	struct m800kbd *m800kbd_data = dev_id;

	/*
	 * this interrupt is prone to bouncing and userspace doesn't like
	 * to have to deal with that kind of thing.  So we do not accept
	 * that a jack interrupt is equal to a jack event.  Instead we fire
	 * some work on the first interrupt, and it hangs about in 100ms units
	 * until no more interrupts come.  Then it accepts the state it finds
	 * for jack insert and reports it once
	 */

	m800kbd_data->hp_irq_count++;
	/*
	 * the first interrupt we see for a while, we fire the work item
	 * and record the interrupt count when we did that.  If more interrupts
	 * come in the meanwhile, we can tell by the difference in that
	 * stored count and hp_irq_count which increments every interrupt
	 */
	if (!m800kbd_data->work_in_progress) {
		m800kbd_data->jack_irq = irq;
		m800kbd_data->hp_irq_count_in_work =
						m800kbd_data->hp_irq_count;
		if (!schedule_work(&m800kbd_data->work))
			printk(KERN_ERR
				"Unable to schedule headphone debounce\n");
		else
			m800kbd_data->work_in_progress = 1;
	}

	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_PM
static int m800kbd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct m800kbd *m800kbd = platform_get_drvdata(dev);

	m800kbd->suspended = 1;

	return 0;
}

static int m800kbd_resume(struct platform_device *dev)
{
	struct m800kbd *m800kbd = platform_get_drvdata(dev);

	m800kbd->suspended = 0;

	return 0;
}
#else
#define m800kbd_suspend	NULL
#define m800kbd_resume	NULL
#endif

static int m800kbd_probe(struct platform_device *pdev)
{
	struct m800kbd *m800kbd;
	struct input_dev *input_dev;
	int rc, irq_power, irq_cam, irq_rec, irq_slide;

	m800kbd = kzalloc(sizeof(struct m800kbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!m800kbd || !input_dev) {
		kfree(m800kbd);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	if (pdev->resource[0].flags != 0)
		return -EINVAL;

	irq_power = gpio_to_irq(pdev->resource[0].start);
	if (irq_power < 0)
		return -EINVAL;

	irq_cam = gpio_to_irq(pdev->resource[1].start);
	if (irq_cam < 0)
		return -EINVAL;

	irq_rec = gpio_to_irq(pdev->resource[2].start);
	if (irq_rec < 0)
		return -EINVAL;

	irq_slide = gpio_to_irq(pdev->resource[3].start);
	if (irq_slide < 0)
		return -EINVAL;

	platform_set_drvdata(pdev, m800kbd);

	m800kbd->input = input_dev;

	//INIT_WORK(&m800kbd->work, m800kbd_debounce_jack);

	input_dev->name = "M800 Buttons";
	input_dev->phys = "m800kbd/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = m800kbd;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SW);
	set_bit(SW_HEADPHONE_INSERT, input_dev->swbit);
	set_bit(SW_LID, input_dev->swbit);
	set_bit(KEY_POWER, input_dev->keybit);
	set_bit(KEY_CAMERA, input_dev->keybit);
	set_bit(KEY_RECORD, input_dev->keybit);

	rc = input_register_device(m800kbd->input);
	if (rc)
		goto out_register;

	if (request_irq(irq_power, m800kbd_power_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Power button", m800kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_power);
		goto out_aux;
	}

	enable_irq_wake(irq_power);

	if (request_irq(irq_cam, m800kbd_cam_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Camera button", m800kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_cam);
		goto out_hold;
	}

	if (request_irq(irq_rec, m800kbd_rec_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Record button", m800kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_rec);
		goto out_hold;
	}

	if (request_irq(irq_slide, m800kbd_slide_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Slide", m800kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_slide);
		goto out_jack;
	}
	enable_irq_wake(irq_slide);

	return 0;

out_jack:
	free_irq(irq_cam, m800kbd);
out_hold:
	free_irq(irq_power, m800kbd);
out_aux:
	input_unregister_device(m800kbd->input);
out_register:
	input_free_device(m800kbd->input);
	platform_set_drvdata(pdev, NULL);
	kfree(m800kbd);

	return -ENODEV;
}

static int m800kbd_remove(struct platform_device *pdev)
{
	struct m800kbd *m800kbd = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(pdev->resource[2].start), m800kbd);
	free_irq(gpio_to_irq(pdev->resource[1].start), m800kbd);
	free_irq(gpio_to_irq(pdev->resource[0].start), m800kbd);

	input_unregister_device(m800kbd->input);
	input_free_device(m800kbd->input);
	platform_set_drvdata(pdev, NULL);
	kfree(m800kbd);

	return 0;
}

static struct platform_driver m800kbd_driver = {
	.probe		= m800kbd_probe,
	.remove		= m800kbd_remove,
	.suspend	= m800kbd_suspend,
	.resume		= m800kbd_resume,
	.driver		= {
		.name	= "m800-button",
	},
};

static int __devinit m800kbd_init(void)
{
	return platform_driver_register(&m800kbd_driver);
}

static void __exit m800kbd_exit(void)
{
	platform_driver_unregister(&m800kbd_driver);
}

module_init(m800kbd_init);
module_exit(m800kbd_exit);

MODULE_AUTHOR("Harald Welte <laforge@gnumonks.org>");
MODULE_DESCRIPTION("E-TEN glofiish M800 GPIO buttons input driver");
MODULE_LICENSE("GPL");
