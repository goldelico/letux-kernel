#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <mach/irqs.h>
#include <linux/module.h>
#include <linux/apm_bios.h>
#include <linux/apm-emulation.h>
#include <linux/mfd/axp-mfd.h>
#include "axp-cfg.h"
#include "axp-rw.h"

int axp_register_notifier(struct device *dev, struct notifier_block *nb,
				uint64_t irqs)
{
	struct axp_dev *chip = dev_get_drvdata(dev);

	chip->ops->enable_irqs(chip, irqs);
	if(NULL != nb) {
	    return blocking_notifier_chain_register(&chip->notifier_list, nb);
	}

    return 0;
}
EXPORT_SYMBOL_GPL(axp_register_notifier);

int axp_unregister_notifier(struct device *dev, struct notifier_block *nb,
				uint64_t irqs)
{
	struct axp_dev *chip = dev_get_drvdata(dev);

	chip->ops->disable_irqs(chip, irqs);
	if(NULL != nb) {
	    return blocking_notifier_chain_unregister(&chip->notifier_list, nb);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(axp_unregister_notifier);

int axp_write(struct device *dev, int reg, uint8_t val)
{


	return __axp_write(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(axp_write);

int axp_writes(struct device *dev, int reg, int len, uint8_t *val)
{


	return  __axp_writes(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(axp_writes);

int axp_read(struct device *dev, int reg, uint8_t *val)
{


	return __axp_read(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(axp_read);

int axp_reads(struct device *dev, int reg, int len, uint8_t *val)
{


	return __axp_reads(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(axp_reads);

int axp_set_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;
	struct axp_dev *chip;

	chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	ret = __axp_read(chip->client, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & bit_mask) != bit_mask) {
		reg_val |= bit_mask;
		ret = __axp_write(chip->client, reg, reg_val);
	}
out:
	mutex_unlock(&chip->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(axp_set_bits);

int axp_clr_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;
	struct axp_dev *chip;

	chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);

	ret = __axp_read(chip->client, reg, &reg_val);
	if (ret)
		goto out;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = __axp_write(chip->client, reg, reg_val);
	}
out:
	mutex_unlock(&chip->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(axp_clr_bits);

int axp_update(struct device *dev, int reg, uint8_t val, uint8_t mask)
{
	struct axp_dev *chip = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&chip->lock);

	ret = __axp_read(chip->client, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | val;
		ret = __axp_write(chip->client, reg, reg_val);
	}
out:
	mutex_unlock(&chip->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(axp_update);

