/*
 * SFH7741 Proximity Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Shubhrajyoti D <shubhrajyoti@ti.com>
 * Contributor: Dan Murphy <DMurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/input/sfh7741.h>

#define SFH7741_PROX_ON	1
#define SFH7741_PROX_OFF	0
#define SFH7741_NAME	"sfh7741"

#undef SFH7741_SUSPEND_RESUME

struct sfh7741_drvdata {
	struct input_dev *input;
	struct mutex lock;
	struct platform_device *pdev;
	struct sfh7741_platform_data *pdata;
	struct work_struct irq_work;
	struct workqueue_struct *work_queue;
	int irq;
	int prox_enable;
	int on_before_suspend;
};

static void sfh7741_report_input(struct sfh7741_drvdata *sfh)
{
	int distance;

	if (sfh->pdata->read_prox())
		distance = sfh->pdata->prox_blocked;
	else
		distance = sfh->pdata->prox_unblocked;

	input_report_abs(sfh->input, ABS_DISTANCE, distance);
	input_sync(sfh->input);
}

static irqreturn_t sfh7741_isr(int irq, void *dev)
{
	struct sfh7741_drvdata *sfh = dev;

	disable_irq_nosync(irq);
	queue_work(sfh->work_queue, &sfh->irq_work);

	return IRQ_HANDLED;
}

static void sfh7741_irq_work_func(struct work_struct *work)
{
	struct sfh7741_drvdata *sfh = container_of(work,
				struct sfh7741_drvdata, irq_work);

	sfh7741_report_input(sfh);
	enable_irq(sfh->irq);
}
static ssize_t set_prox_state(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int state = SFH7741_PROX_OFF;
	struct platform_device *pdev = to_platform_device(dev);
	struct sfh7741_drvdata *ddata = platform_get_drvdata(pdev);

	if (sscanf(buf, "%u", &state) != 1)
		return -EINVAL;

	if (state != SFH7741_PROX_OFF)
		state = SFH7741_PROX_ON;

	mutex_lock(&ddata->lock);
	if (state != ddata->prox_enable) {
		if (state) {
			enable_irq(ddata->irq);
			if (ddata->pdata->flags & SFH7741_WAKEABLE_INT)
				enable_irq_wake(ddata->irq);

			sfh7741_report_input(ddata);
		} else {
			disable_irq_nosync(ddata->irq);
			if (ddata->pdata->flags & SFH7741_WAKEABLE_INT)
				disable_irq_wake(ddata->irq);
		}

		ddata->pdata->activate_func(state);
		ddata->prox_enable = state;
	}

	mutex_unlock(&ddata->lock);

	return strnlen(buf, count);
}

static ssize_t show_prox_state(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sfh7741_drvdata *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", ddata->prox_enable);
}
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, show_prox_state, set_prox_state);


static struct attribute *sfh7741_attributes[] = {
	&dev_attr_state.attr,
	NULL
};

static const struct attribute_group sfh7741_attr_group = {
	.attrs = sfh7741_attributes,
};

static int __devinit sfh7741_probe(struct platform_device *pdev)
{
	int error;
	struct sfh7741_drvdata *ddata;
	struct device *dev = &pdev->dev;


	pr_info("%s: SFH7741 Proximity sensor\n", __func__);

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s:platform data is NULL. exiting.\n", __func__);
		error = -ENODEV;
		goto err0;
	}

	ddata = kzalloc(sizeof(*ddata),	GFP_KERNEL);
	if (ddata == NULL) {
		pr_err("%s: platform data is NULL. exiting.\n", __func__);
		error = -ENOMEM;
		goto err1;
	}
	ddata->pdata = pdev->dev.platform_data;
	if (!ddata->pdata->activate_func ||
	    !ddata->pdata->read_prox) {
		pr_err("%s:Read Prox or Activate is NULL\n", __func__);
		error = -EINVAL;
		goto err2;
	}

	ddata->irq = ddata->pdata->irq;
	ddata->prox_enable = SFH7741_PROX_OFF;

	ddata->input = input_allocate_device();
	if (!ddata->input) {
		pr_err("%s:Failed to allocate input device\n", __func__);
		error = -ENOMEM;
		goto err3;
	}

	ddata->input->name = pdev->name;
	ddata->input->phys = "sfh7741/input0";
	ddata->input->dev.parent = &pdev->dev;
	ddata->input->id.bustype = BUS_HOST;

	__set_bit(EV_ABS, ddata->input->evbit);
	input_set_abs_params(ddata->input, ABS_DISTANCE,
				ddata->pdata->prox_blocked,
				ddata->pdata->prox_unblocked, 0, 0);

	error = input_register_device(ddata->input);
	if (error) {
		pr_err("%s:Unable to register input device,error: %d\n",
			__func__, error);
		goto err4;
	}

	sfh7741_report_input(ddata);

	platform_set_drvdata(pdev, ddata);

	INIT_WORK(&ddata->irq_work, sfh7741_irq_work_func);
	ddata->work_queue = create_singlethread_workqueue("sfh7741_wq");
	if (!ddata->work_queue) {
		error = -ENOMEM;
		pr_err("%s: cannot create work queue: %d\n", __func__, error);
		goto err5;
	}
	error = request_irq(ddata->irq, sfh7741_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "sfh7741_irq", ddata);

	if (error < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, error);
		goto err6;
	}

	mutex_init(&ddata->lock);
	error = sysfs_create_group(&dev->kobj, &sfh7741_attr_group);
	if (error) {
		pr_err("%s: Failed to create sysfs entries\n", __func__);
		error = -EINVAL;
		goto err7;
	}

	disable_irq_nosync(ddata->irq);

	return 0;

err7:
	mutex_destroy(&ddata->lock);
err6:
	destroy_workqueue(ddata->work_queue);
	platform_set_drvdata(pdev, NULL);
err5:
	input_unregister_device(ddata->input);
err4:
	input_free_device(ddata->input);
err3:
err2:
	kfree(ddata);
err1:
err0:
	return error;

}

static int __devexit sfh7741_remove(struct platform_device *pdev)
{
	struct sfh7741_drvdata *ddata = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	mutex_destroy(&ddata->lock);
	sysfs_remove_group(&dev->kobj, &sfh7741_attr_group);
	free_irq(ddata->irq, (void *)ddata);
	destroy_workqueue(ddata->work_queue);
	input_unregister_device(ddata->input);
	kfree(ddata);
	return 0;
}

#ifdef SFH7741_SUSPEND_RESUME
static int sfh7741_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sfh7741_drvdata *ddata = platform_get_drvdata(pdev);

	disable_irq_nosync(ddata->irq);
	/* Save the prox state for the resume */
	ddata->on_before_suspend = ddata->prox_enable
	ddata->pdata->activate_func(SFH7741_PROX_OFF);
	return 0;
}

static int sfh7741_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sfh7741_drvdata *ddata = platform_get_drvdata(pdev);

	enable_irq(ddata->irq);
	if (ddata->on_before_suspend)
		ddata->pdata->activate_func(SFH7741_PROX_ON);

	sfh7741_report_input(ddata);

	return 0;
}

static const struct dev_pm_ops sfh7741_pm_ops = {
	.suspend	= sfh7741_suspend,
	.resume		= sfh7741_resume,
};
#endif

static struct platform_driver sfh7741_device_driver = {
	.probe		= sfh7741_probe,
	.remove		= __devexit_p(sfh7741_remove),
	.driver		= {
		.name	= SFH7741_NAME,
		.owner	= THIS_MODULE,
#ifdef SFH7741_SUSPEND_RESUME
		.pm	= &sfh7741_pm_ops,
#endif
	}
};

static int __init sfh7741_init(void)
{
	return platform_driver_register(&sfh7741_device_driver);
}

static void __exit sfh7741_exit(void)
{
	platform_driver_unregister(&sfh7741_device_driver);
}

module_init(sfh7741_init);
module_exit(sfh7741_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Proximity sensor SFH7741 driver");
MODULE_ALIAS("platform:sfh7741");
