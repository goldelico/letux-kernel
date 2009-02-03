/* Philips PCF50606 Power Management Unit (PMU) driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *	    Matt Hsu <matt@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/mfd/pcf50606/core.h>

/* Read a block of upto 32 regs  */
int pcf50606_read_block(struct pcf50606 *pcf , u8 reg,
					int nr_regs, u8 *data)
{
	int ret;

	mutex_lock(&pcf->lock);
	ret = i2c_smbus_read_i2c_block_data(pcf->i2c_client, reg,
							nr_regs, data);
	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_read_block);

/* Write a block of upto 32 regs  */
int pcf50606_write_block(struct pcf50606 *pcf , u8 reg,
					int nr_regs, u8 *data)
{
	int ret;

	mutex_lock(&pcf->lock);
	ret = i2c_smbus_write_i2c_block_data(pcf->i2c_client, reg,
							nr_regs, data);
	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_write_block);

u8 pcf50606_reg_read(struct pcf50606 *pcf, u8 reg)
{
	int ret;

	mutex_lock(&pcf->lock);
	ret = i2c_smbus_read_byte_data(pcf->i2c_client, reg);
	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_reg_read);

int pcf50606_reg_write(struct pcf50606 *pcf, u8 reg, u8 val)
{
	int ret;
	mutex_lock(&pcf->lock);
	ret = i2c_smbus_write_byte_data(pcf->i2c_client, reg, val);
	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_reg_write);

int pcf50606_reg_set_bit_mask(struct pcf50606 *pcf, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	val &= mask;

	mutex_lock(&pcf->lock);

	tmp = i2c_smbus_read_byte_data(pcf->i2c_client, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = i2c_smbus_write_byte_data(pcf->i2c_client, reg, tmp);

	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_reg_set_bit_mask);

int pcf50606_reg_clear_bits(struct pcf50606 *pcf, u8 reg, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&pcf->lock);

	tmp = i2c_smbus_read_byte_data(pcf->i2c_client, reg);
	tmp &= ~val;
	ret = i2c_smbus_write_byte_data(pcf->i2c_client, reg, tmp);

	mutex_unlock(&pcf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pcf50606_reg_clear_bits);

static ssize_t show_resume_reason(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcf50606 *pcf = dev_get_drvdata(dev);
	int n;

	n = sprintf(buf, "%02x%02x%02x%02x%02x\n",
				pcf->resume_reason[0],
				pcf->resume_reason[1],
				pcf->resume_reason[2],
				pcf->resume_reason[3],
				pcf->resume_reason[4]);

	return n;
}
static DEVICE_ATTR(resume_reason, 0400, show_resume_reason, NULL);

static struct attribute *pcf_sysfs_entries[] = {
	&dev_attr_resume_reason.attr,
	NULL,
};

static struct attribute_group pcf_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf_sysfs_entries,
};


static int pcf50606_irq_mask_set(struct pcf50606 *pcf, int irq, int mask)
{
	u8 reg, bits, tmp;
	int ret = 0, idx;

	idx = irq / 8;
	reg =  PCF50606_REG_INT1M + idx;
	bits = 1 << (irq % 8);

	mutex_lock(&pcf->lock);

	if (mask) {
		tmp = i2c_smbus_read_byte_data(pcf->i2c_client, reg);
		tmp |= bits;
		ret = i2c_smbus_write_byte_data(pcf->i2c_client, reg, tmp);

		pcf->mask_regs[idx] &= ~bits;
		pcf->mask_regs[idx] |= bits;
	} else {
		tmp = i2c_smbus_read_byte_data(pcf->i2c_client, reg);
		tmp &= ~bits;
		ret = i2c_smbus_write_byte_data(pcf->i2c_client, reg, tmp);

		pcf->mask_regs[idx] &= ~bits;
	}

	mutex_unlock(&pcf->lock);

	return 0;
}

int pcf50606_irq_mask(struct pcf50606 *pcf, int irq)
{
	dev_info(pcf->dev, "Masking IRQ %d\n", irq);

	return pcf50606_irq_mask_set(pcf, irq, 1);
}
EXPORT_SYMBOL_GPL(pcf50606_irq_mask);

int pcf50606_irq_unmask(struct pcf50606 *pcf, int irq)
{
	dev_info(pcf->dev, "Unmasking IRQ %d\n", irq);

	return pcf50606_irq_mask_set(pcf, irq, 0);
}
EXPORT_SYMBOL_GPL(pcf50606_irq_unmask);

int pcf50606_irq_mask_get(struct pcf50606 *pcf, int irq)
{
	u8 reg, bits;

	reg =  (irq / 8);
	bits = (1 << (irq % 8));

	return pcf->mask_regs[reg] & bits;
}
EXPORT_SYMBOL_GPL(pcf50606_irq_mask_get);

static void pcf50606_irq_call_handler(struct pcf50606 *pcf,
					int irq)
{
	if (pcf->irq_handler[irq].handler)
		pcf->irq_handler[irq].handler(pcf, irq,
					pcf->irq_handler[irq].data);
}

#define PCF50606_ONKEY1S_TIMEOUT 8

static void pcf50606_irq_worker(struct work_struct *work)
{
	struct pcf50606 *pcf;
	int ret, i, j;
	u8 pcf_int[3], chgstat;

	pcf = container_of(work, struct pcf50606, irq_work);

	/* Read the 3 INT regs in one transaction */
	ret = pcf50606_read_block(pcf, PCF50606_REG_INT1,
						sizeof(pcf_int), pcf_int);
	if (ret != sizeof(pcf_int)) {
		dev_info(pcf->dev, "Error reading INT registers\n");

		/* We don't have an option but to retry. Because if
		 * we don't, there won't be another interrupt edge.
		 */
		goto reschedule;
	}

	/* We immediately read the usb and adapter status. We thus make sure
	 * only of CHGINS/CHGRM handlers are called */
	if (pcf_int[1] & (PCF50606_INT2_CHGINS | PCF50606_INT2_CHGRM)) {
		chgstat = pcf50606_reg_read(pcf, PCF50606_REG_MBCS1);
		if (chgstat & (0x1 << 4))
			pcf_int[1] &= ~(1 << PCF50606_INT2_CHGRM);
		else
			pcf_int[1] &= ~(1 << PCF50606_INT2_CHGINS);
	}
	
	dev_info(pcf->dev, "INT1=0x%02x INT2=0x%02x INT3=0x%02x",
				pcf_int[0], pcf_int[1], pcf_int[2]);

	/* Some revisions of the chip don't have a 8s standby mode on
	 * ONKEY1S press. We try to manually do it in such cases. */

	if (pcf_int[0] & PCF50606_INT1_SECOND && pcf->onkey1s_held) {
		dev_info(pcf->dev, "ONKEY1S held for %d secs\n",
							pcf->onkey1s_held);
		if (pcf->onkey1s_held++ == PCF50606_ONKEY1S_TIMEOUT)
			if (pcf->pdata->force_shutdown)
				pcf->pdata->force_shutdown(pcf);
	}

	if (pcf_int[0] & PCF50606_INT1_ONKEY1S) {
		dev_info(pcf->dev, "ONKEY1S held\n");
		pcf->onkey1s_held = 1 ;

		/* Unmask IRQ_SECOND */
		pcf50606_reg_clear_bits(pcf, PCF50606_REG_INT1M,
						PCF50606_INT1_SECOND);

		/* Unmask IRQ_ONKEYF */
		pcf50606_reg_clear_bits(pcf, PCF50606_REG_INT1M,
						PCF50606_INT1_ONKEYF);
	}

	if ((pcf_int[0] & PCF50606_INT1_ONKEYR) && pcf->onkey1s_held) {
		pcf->onkey1s_held = 0;

		/* Mask SECOND and ONKEYF interrupts */
		if (pcf->mask_regs[0] & PCF50606_INT1_SECOND)
			pcf50606_reg_set_bit_mask(pcf,
					PCF50606_REG_INT1M,
					PCF50606_INT1_SECOND,
					PCF50606_INT1_SECOND);

		if (pcf->mask_regs[0] & PCF50606_INT1_ONKEYF)
			pcf50606_reg_set_bit_mask(pcf,
					PCF50606_REG_INT1M,
					PCF50606_INT1_ONKEYF,
					PCF50606_INT1_ONKEYF);
	}

	/* Have we just resumed ? */
	if (pcf->is_suspended) {

		pcf->is_suspended = 0;

		/* Set the resume reason filtering out non resumers */
		for (i = 0; i < ARRAY_SIZE(pcf_int); i++)
			pcf->resume_reason[i] = pcf_int[i] &
						pcf->pdata->resumers[i];

		/* Make sure we don't pass on any input events to
		 * userspace now */
		pcf_int[0] &= ~(PCF50606_INT1_SECOND | PCF50606_INT1_ALARM);
	}

	/* Unset masked interrupts */
	for (i = 0; i < ARRAY_SIZE(pcf_int); i++)
		pcf_int[i] &= ~pcf->mask_regs[i];

	for (i = 0; i < ARRAY_SIZE(pcf_int); i++)
		for (j = 0; j < 8 ; j++)
			if (pcf_int[i] & (1 << j))
				pcf50606_irq_call_handler(pcf, (i * 8) + j);

	put_device(pcf->dev);

	return;
reschedule:
	schedule_work(&pcf->irq_work);

	/* Don't put_device here. Will be used when we are rescheduled */

	return;
}

static irqreturn_t pcf50606_irq(int irq, void *data)
{
	struct pcf50606 *pcf = data;

	get_device(pcf->dev);
	schedule_work(&pcf->irq_work);

	return IRQ_HANDLED;
}

static void
pcf50606_client_dev_register(struct pcf50606 *pcf, const char *name,
						struct platform_device **pdev)
{
	int ret;

	*pdev = platform_device_alloc(name, -1);

	if (!pdev) {
		dev_err(pcf->dev, "Falied to allocate %s\n", name);
		return;
	}

	(*pdev)->dev.parent = pcf->dev;
	platform_set_drvdata(*pdev, pcf);

	ret = platform_device_add(*pdev);
	if (ret != 0) {
		dev_err(pcf->dev, "Failed to register %s: %d\n", name, ret);
		platform_device_put(*pdev);
		*pdev = NULL;
	}
}

#ifdef CONFIG_PM
static int pcf50606_suspend(struct device *dev, pm_message_t state)
{
	struct pcf50606 *pcf;
	int ret, i;
	u8 res[3];

	pcf = dev_get_drvdata(dev);

	/* Make sure our interrupt handlers are not called
	 * henceforth */
	disable_irq(pcf->irq);

	/* Make sure that an IRQ worker has quit */
	cancel_work_sync(&pcf->irq_work);

	/* Save the masks */
	ret = pcf50606_read_block(pcf, PCF50606_REG_INT1M,
				ARRAY_SIZE(pcf->suspend_irq_masks),
						pcf->suspend_irq_masks);
	if (ret < 0)
		dev_err(pcf->dev, "error saving irq masks\n");

	/* Set interrupt masks. So that only those sources we want to wake
	 * us up can
	 */
	for (i = 0; i < ARRAY_SIZE(res); i++)
		res[i] = ~pcf->pdata->resumers[i];

	pcf50606_write_block(pcf, PCF50606_REG_INT1M,
					ARRAY_SIZE(res), &res[0]);

	pcf->is_suspended = 1;

	return 0;
}

static int pcf50606_resume(struct device *dev)
{
	struct pcf50606 *pcf;

	pcf = dev_get_drvdata(dev);

	/* Write the saved mask registers */
	pcf50606_write_block(pcf, PCF50606_REG_INT1M,
			ARRAY_SIZE(pcf->suspend_irq_masks),
					pcf->suspend_irq_masks);

	/* Clear any pending interrupts and set resume reason if any */
	pcf50606_irq_worker(&pcf->irq_work);

	enable_irq(pcf->irq);

	return 0;
}
#else
#define pcf50606_suspend NULL
#define pcf50606_resume NULL
#endif

static int pcf50606_probe(struct i2c_client *client,
				const struct i2c_device_id *ids)
{
	struct pcf50606 *pcf;
	struct pcf50606_platform_data *pdata;
	int i, ret = 0;
	int version, variant;
	u8 mbcs1;

	pdata = client->dev.platform_data;

	pcf = kzalloc(sizeof(*pcf), GFP_KERNEL);
	if (!pcf)
		return -ENOMEM;

	pcf->pdata = pdata;
	pdata->pcf = pcf;

	mutex_init(&pcf->lock);

	i2c_set_clientdata(client, pcf);
	pcf->dev = &client->dev;
	pcf->i2c_client = client;

	INIT_WORK(&pcf->irq_work, pcf50606_irq_worker);

	version = pcf50606_reg_read(pcf, 0);
	if (version < 0) {
		dev_err(pcf->dev, "Unable to probe pcf50606\n");
		kfree(pcf);
		return -ENODEV;
	}

	variant = pcf50606_reg_read(pcf, 1);
	if (version < 0) {
		dev_err(pcf->dev, "Unable to probe pcf50606\n");
		kfree(pcf);
		return -ENODEV;
	}

	dev_info(pcf->dev, "Probed device version %d variant %d\n",
							version, variant);

	/* Enable all inteerupts except RTC SECOND */
	pcf->mask_regs[0] = 0x80;
	pcf50606_reg_write(pcf, PCF50606_REG_INT1M, 0x80);

	pcf50606_reg_write(pcf, PCF50606_REG_INT2M, 0x00);
	pcf50606_reg_write(pcf, PCF50606_REG_INT3M, 0x00);

	pcf50606_client_dev_register(pcf, "pcf50606-input",
						&pcf->input.pdev);
	pcf50606_client_dev_register(pcf, "pcf50606-rtc",
						&pcf->rtc.pdev);
	pcf50606_client_dev_register(pcf, "pcf50606-mbc",
						&pcf->mbc.pdev);
	pcf50606_client_dev_register(pcf, "pcf50606-adc",
						&pcf->adc.pdev);
	pcf50606_client_dev_register(pcf, "pcf50606-wdt",
						&pcf->wdt.pdev);
	for (i = 0; i < PCF50606_NUM_REGULATORS; i++) {
		struct platform_device *pdev;

		pdev = platform_device_alloc("pcf50606-regltr", i);
		if (!pdev) {
			dev_err(pcf->dev, "Cannot create regulator\n");
			continue;
		}

		pdev->dev.parent = pcf->dev;
		pdev->dev.platform_data = &pdata->reg_init_data[i];
		pdev->dev.driver_data = pcf;
		pcf->pmic.pdev[i] = pdev;

		platform_device_add(pdev);
	}

	pcf->irq = client->irq;

	if (client->irq) {
		ret = request_irq(client->irq, pcf50606_irq,
				IRQF_TRIGGER_FALLING, "pcf50606", pcf);

		if (ret) {
			dev_err(pcf->dev, "Failed to request IRQ %d\n", ret);
			goto err;
		}
	} else {
		dev_err(pcf->dev, "No IRQ configured\n");
		goto err;
	}

	if (enable_irq_wake(client->irq) < 0)
		dev_err(pcf->dev, "IRQ %u cannot be enabled as wake-up source"
			"in this hardware revision", client->irq);

	/* Cold Intialization */
	mbcs1 = pcf50606_reg_read(pcf, PCF50606_REG_MBCS1);

	if (mbcs1 & (0x01 << 4)) /* Charger present ? */
		pcf50606_irq_call_handler(pcf, PCF50606_IRQ_CHGINS);

	ret = sysfs_create_group(&client->dev.kobj, &pcf_attr_group);
	if (ret)
		dev_err(pcf->dev, "error creating sysfs entries\n");

	if (pdata->probe_done)
		pdata->probe_done(pcf);

	return 0;

err:
	kfree(pcf);
	return ret;
}

static int pcf50606_remove(struct i2c_client *client)
{
	struct pcf50606 *pcf = i2c_get_clientdata(client);

	free_irq(pcf->irq, pcf);
	kfree(pcf);

	return 0;
}

static struct i2c_device_id pcf50606_id_table[] = {
	{"pcf50606", 0x73},
};

static struct i2c_driver pcf50606_driver = {
	.driver = {
		.name	= "pcf50606",
		.suspend = pcf50606_suspend,
		.resume	= pcf50606_resume,
	},
	.id_table = pcf50606_id_table,
	.probe = pcf50606_probe,
	.remove = pcf50606_remove,
};

static int __init pcf50606_init(void)
{
	return i2c_add_driver(&pcf50606_driver);
}

static void pcf50606_exit(void)
{
	i2c_del_driver(&pcf50606_driver);
}

MODULE_DESCRIPTION("I2C chip driver for NXP PCF50606 PMU");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

module_init(pcf50606_init);
module_exit(pcf50606_exit);
