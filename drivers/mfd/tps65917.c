/*
 * TI TPS65917 Integrated power management chipsets
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps65917.h>
#include <linux/of_device.h>

#define TPS65917_EXT_REQ (TPS65917_EXT_CONTROL_ENABLE1 |	\
			TPS65917_EXT_CONTROL_ENABLE2 |	\
			TPS65917_EXT_CONTROL_NSLEEP)

struct tps65917_sleep_requestor_info {
	int id;
	int reg_offset;
	int bit_pos;
};

#define EXTERNAL_REQUESTOR(_id, _offset, _pos)		\
		[TPS65917_EXTERNAL_REQSTR_ID_##_id] = {		\
		.id = TPS65917_EXTERNAL_REQSTR_ID_##_id,	\
		.reg_offset = _offset,			\
		.bit_pos = _pos,			\
	}

static struct tps65917_sleep_requestor_info sleep_req_info[] = {
	EXTERNAL_REQUESTOR(REGEN1, 0, 0),
	EXTERNAL_REQUESTOR(REGEN2, 0, 1),
	EXTERNAL_REQUESTOR(REGEN3, 0, 6),
	EXTERNAL_REQUESTOR(SMPS1, 1, 0),
	EXTERNAL_REQUESTOR(SMPS2, 1, 1),
	EXTERNAL_REQUESTOR(SMPS3, 1, 2),
	EXTERNAL_REQUESTOR(SMPS4, 1, 3),
	EXTERNAL_REQUESTOR(SMPS5, 1, 4),
	EXTERNAL_REQUESTOR(LDO1, 2, 0),
	EXTERNAL_REQUESTOR(LDO2, 2, 1),
	EXTERNAL_REQUESTOR(LDO3, 2, 2),
	EXTERNAL_REQUESTOR(LDO4, 2, 3),
	EXTERNAL_REQUESTOR(LDO5, 2, 4),
};

static const struct regmap_config tps65917_regmap_config[TPS65917_NUM_CLIENTS] = {
	{
		.reg_bits = 8,
		.val_bits = 8,
		.cache_type = REGCACHE_RBTREE,
		.max_register = TPS65917_BASE_TO_REG(TPS65917_PU_PD_OD_BASE,
					TPS65917_PU_PD_INPUT_CTRL4),
	},
	{
		.reg_bits = 8,
		.val_bits = 8,
		.cache_type = REGCACHE_RBTREE,
		.max_register = TPS65917_BASE_TO_REG(TPS65917_GPADC_BASE,
					TPS65917_GPADC_SMPS_VSEL_MONITORING),
	},
	{
		.reg_bits = 8,
		.val_bits = 8,
		.cache_type = REGCACHE_RBTREE,
		.max_register = TPS65917_BASE_TO_REG(TPS65917_TRIM_GPADC_BASE,
					TPS65917_GPADC_TRIM16),
	},
};

static const struct regmap_irq tps65917_irqs[] = {
	/* INT1 IRQs */
	[TPS65917_RESERVED1] = {
		.mask = TPS65917_RESERVED,
	},
	[TPS65917_PWRON_IRQ] = {
		.mask = TPS65917_INT1_STATUS_PWRON,
	},
	[TPS65917_LONG_PRESS_KEY_IRQ] = {
		.mask = TPS65917_INT1_STATUS_LONG_PRESS_KEY,
	},
	[TPS65917_RESERVED2] = {
		.mask = TPS65917_RESERVED,
	},
	[TPS65917_PWRDOWN_IRQ] = {
		.mask = TPS65917_INT1_STATUS_PWRDOWN,
	},
	[TPS65917_HOTDIE_IRQ] = {
		.mask = TPS65917_INT1_STATUS_HOTDIE,
	},
	[TPS65917_VSYS_MON_IRQ] = {
		.mask = TPS65917_INT1_STATUS_VSYS_MON,
	},
	[TPS65917_RESERVED3] = {
		.mask = TPS65917_RESERVED,
	},
	/* INT2 IRQs*/
	[TPS65917_RESERVED4] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 1,
	},
	[TPS65917_OTP_ERROR_IRQ] = {
		.mask = TPS65917_INT2_STATUS_OTP_ERROR,
		.reg_offset = 1,
	},
	[TPS65917_WDT_IRQ] = {
		.mask = TPS65917_INT2_STATUS_WDT,
		.reg_offset = 1,
	},
	[TPS65917_RESERVED5] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 1,
	},
	[TPS65917_RESET_IN_IRQ] = {
		.mask = TPS65917_INT2_STATUS_RESET_IN,
		.reg_offset = 1,
	},
	[TPS65917_FSD_IRQ] = {
		.mask = TPS65917_INT2_STATUS_FSD,
		.reg_offset = 1,
	},
	[TPS65917_SHORT_IRQ] = {
		.mask = TPS65917_INT2_STATUS_SHORT,
		.reg_offset = 1,
	},
	[TPS65917_RESERVED6] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 1,
	},
	/* INT3 IRQs */
	[TPS65917_GPADC_AUTO_0_IRQ] = {
		.mask = TPS65917_INT3_STATUS_GPADC_AUTO_0,
		.reg_offset = 2,
	},
	[TPS65917_GPADC_AUTO_1_IRQ] = {
		.mask = TPS65917_INT3_STATUS_GPADC_AUTO_1,
		.reg_offset = 2,
	},
	[TPS65917_GPADC_EOC_SW_IRQ] = {
		.mask = TPS65917_INT3_STATUS_GPADC_EOC_SW,
		.reg_offset = 2,
	},
	[TPS65917_RESREVED6] = {
		.mask = TPS65917_RESERVED6,
		.reg_offset = 2,
	},
	[TPS65917_RESERVED7] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 2,
	},
	[TPS65917_RESERVED8] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 2,
	},
	[TPS65917_RESERVED9] = {
		.mask = TPS65917_RESERVED,
		.reg_offset = 2,
	},
	[TPS65917_VBUS_IRQ] = {
		.mask = TPS65917_INT3_STATUS_VBUS,
		.reg_offset = 2,
	},
	/* INT4 IRQs */
	[TPS65917_GPIO_0_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_0,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_1_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_1,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_2_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_2,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_3_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_3,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_4_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_4,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_5_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_5,
		.reg_offset = 3,
	},
	[TPS65917_GPIO_6_IRQ] = {
		.mask = TPS65917_INT4_STATUS_GPIO_6,
		.reg_offset = 3,
	},
	[TPS65917_RESERVED10] = {
		.mask = TPS65917_RESERVED10,
		.reg_offset = 3,
	},
};

static struct regmap_irq_chip tps65917_irq_chip = {
	.name = "tps65917",
	.irqs = tps65917_irqs,
	.num_irqs = ARRAY_SIZE(tps65917_irqs),

	.num_regs = 4,
	.irq_reg_stride = 5,
	.status_base = TPS65917_BASE_TO_REG(TPS65917_INTERRUPT_BASE,
			TPS65917_INT1_STATUS),
	.mask_base = TPS65917_BASE_TO_REG(TPS65917_INTERRUPT_BASE,
			TPS65917_INT1_MASK),
};

int tps65917_ext_control_req_config(struct tps65917 *tps65917,
				    enum tps65917_external_requestor_id id,
				    int ext_ctrl, bool enable)
{
	int preq_mask_bit = 0;
	int reg_add = 0;
	int bit_pos;
	int ret;

	if (!(ext_ctrl & TPS65917_EXT_REQ))
		return 0;

	if (id >= TPS65917_EXTERNAL_REQSTR_ID_MAX)
		return 0;

	if (ext_ctrl & TPS65917_EXT_CONTROL_NSLEEP) {
		reg_add = TPS65917_NSLEEP_RES_ASSIGN;
		preq_mask_bit = 0;
	} else if (ext_ctrl & TPS65917_EXT_CONTROL_ENABLE1) {
		reg_add = TPS65917_ENABLE1_RES_ASSIGN;
		preq_mask_bit = 1;
	} else if (ext_ctrl & TPS65917_EXT_CONTROL_ENABLE2) {
		reg_add = TPS65917_ENABLE2_RES_ASSIGN;
		preq_mask_bit = 2;
	}

	bit_pos = sleep_req_info[id].bit_pos;
	reg_add += sleep_req_info[id].reg_offset;
	if (enable)
		ret = tps65917_update_bits(tps65917, TPS65917_RESOURCE_BASE,
					   reg_add, BIT(bit_pos), BIT(bit_pos));
	else
		ret = tps65917_update_bits(tps65917, TPS65917_RESOURCE_BASE,
					   reg_add, BIT(bit_pos), 0);
	if (ret < 0) {
		dev_err(tps65917->dev, "Resource reg 0x%02x update failed %d\n",
			reg_add, ret);
		return ret;
	}

	/* Unmask the PREQ */
	ret = tps65917_update_bits(tps65917, TPS65917_PMU_CONTROL_BASE,
				   TPS65917_POWER_CTRL, BIT(preq_mask_bit), 0);
	if (ret < 0) {
		dev_err(tps65917->dev, "POWER_CTRL register update failed %d\n",
			ret);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tps65917_ext_control_req_config);

static int tps65917_set_pdata_irq_flag(struct i2c_client *i2c,
				       struct tps65917_platform_data *pdata)
{
	struct irq_data *irq_data = irq_get_irq_data(i2c->irq);
	if (!irq_data) {
		dev_err(&i2c->dev, "Invalid IRQ: %d\n", i2c->irq);
		return -EINVAL;
	}

	pdata->irq_flags = irqd_get_trigger_type(irq_data);
	dev_info(&i2c->dev, "Irq flag is 0x%08x\n", pdata->irq_flags);
	return 0;
}

static void tps65917_dt_to_pdata(struct i2c_client *i2c,
				 struct tps65917_platform_data *pdata)
{
	struct device_node *node = i2c->dev.of_node;
	int ret;
	u32 prop;

	ret = of_property_read_u32(node, "ti,mux-pad1", &prop);
	if (!ret) {
		pdata->mux_from_pdata = 1;
		pdata->pad1 = prop;
	}

	ret = of_property_read_u32(node, "ti,mux-pad2", &prop);
	if (!ret) {
		pdata->mux_from_pdata = 1;
		pdata->pad2 = prop;
	}

	/* The default for this register is all masked */
	ret = of_property_read_u32(node, "ti,power-ctrl", &prop);
	if (!ret)
		pdata->power_ctrl = prop;
	else
		pdata->power_ctrl = TPS65917_POWER_CTRL_NSLEEP_MASK |
					TPS65917_POWER_CTRL_ENABLE1_MASK |
					TPS65917_POWER_CTRL_ENABLE2_MASK;
	if (i2c->irq)
		tps65917_set_pdata_irq_flag(i2c, pdata);

	pdata->pm_off = of_property_read_bool(node,
			"ti,system-power-controller");
}

static struct tps65917 *tps65917_dev;

static const struct of_device_id of_tps65917_match_tbl[] = {
	{
		.compatible = "ti,tps65917",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_tps65917_match_tbl);

static int tps65917_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct tps65917 *tps65917;
	struct tps65917_platform_data *pdata;
	struct device_node *node = i2c->dev.of_node;
	int ret = 0, i;
	unsigned int reg, addr, *features;
	int slave;
	const struct of_device_id *match;

	pdata = dev_get_platdata(&i2c->dev);

	if (node && !pdata) {
		pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);

		if (!pdata)
			return -ENOMEM;

		tps65917_dt_to_pdata(i2c, pdata);
	}

	if (!pdata)
		return -EINVAL;

	tps65917 = devm_kzalloc(&i2c->dev, sizeof(struct tps65917), GFP_KERNEL);
	if (tps65917 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, tps65917);
	tps65917->dev = &i2c->dev;
	tps65917->irq = i2c->irq;

	match = of_match_device(of_tps65917_match_tbl, &i2c->dev);

	if (!match)
		return -ENODATA;

	features = (unsigned int *)match->data;

	for (i = 0; i < TPS65917_NUM_CLIENTS; i++) {
		if (i == 0) {
			tps65917->i2c_clients[i] = i2c;
		} else {
			tps65917->i2c_clients[i] =
					i2c_new_dummy(i2c->adapter,
						      i2c->addr + i);
			if (!tps65917->i2c_clients[i]) {
				dev_err(tps65917->dev,
					"can't attach client %d\n", i);
				ret = -ENOMEM;
				goto err_i2c;
			}
			tps65917->i2c_clients[i]->dev.of_node = of_node_get(node);
		}
		tps65917->regmap[i] = devm_regmap_init_i2c(tps65917->i2c_clients[i],
							   &tps65917_regmap_config[i]);
		if (IS_ERR(tps65917->regmap[i])) {
			ret = PTR_ERR(tps65917->regmap[i]);
			dev_err(tps65917->dev,
				"Failed to allocate regmap %d, err: %d\n",
				i, ret);
			goto err_i2c;
		}
	}

	if (!tps65917->irq) {
		dev_warn(tps65917->dev, "IRQ missing: skipping irq request\n");
		goto no_irq;
	}

	/* Change interrupt line output polarity */
	if (pdata->irq_flags & IRQ_TYPE_LEVEL_HIGH)
		reg = TPS65917_POLARITY_CTRL_INT_POLARITY;
	else
		reg = 0;
	ret = tps65917_update_bits(tps65917, TPS65917_PU_PD_OD_BASE,
				   TPS65917_POLARITY_CTRL,
				   TPS65917_POLARITY_CTRL_INT_POLARITY, reg);
	if (ret < 0) {
		dev_err(tps65917->dev, "POLARITY_CTRL updat failed: %d\n", ret);
		goto err_i2c;
	}

	/* Change IRQ into clear on read mode for efficiency */
	slave = TPS65917_BASE_TO_SLAVE(TPS65917_INTERRUPT_BASE);
	addr = TPS65917_BASE_TO_REG(TPS65917_INTERRUPT_BASE, TPS65917_INT_CTRL);
	reg = TPS65917_INT_CTRL_INT_CLEAR;

	regmap_write(tps65917->regmap[slave], addr, reg);

	ret = regmap_add_irq_chip(tps65917->regmap[slave], tps65917->irq,
				  IRQF_ONESHOT | pdata->irq_flags, 0,
				  &tps65917_irq_chip,
				  &tps65917->irq_data);
	if (ret < 0)
		goto err_i2c;

no_irq:
	slave = TPS65917_BASE_TO_SLAVE(TPS65917_PU_PD_OD_BASE);
	addr = TPS65917_BASE_TO_REG(TPS65917_PU_PD_OD_BASE,
				    TPS65917_PRIMARY_SECONDARY_PAD1);

	if (pdata->mux_from_pdata) {
		reg = pdata->pad1;
		ret = regmap_write(tps65917->regmap[slave], addr, reg);
		if (ret)
			goto err_irq;
	} else {
		ret = regmap_read(tps65917->regmap[slave], addr, &reg);
		if (ret)
			goto err_irq;
	}

	addr = TPS65917_BASE_TO_REG(TPS65917_PU_PD_OD_BASE,
				    TPS65917_PRIMARY_SECONDARY_PAD2);

	if (pdata->mux_from_pdata) {
		reg = pdata->pad2;
		ret = regmap_write(tps65917->regmap[slave], addr, reg);
		if (ret)
			goto err_irq;
	} else {
		ret = regmap_read(tps65917->regmap[slave], addr, &reg);
		if (ret)
			goto err_irq;
	}

	reg = pdata->power_ctrl;

	slave = TPS65917_BASE_TO_SLAVE(TPS65917_PMU_CONTROL_BASE);
	addr = TPS65917_BASE_TO_REG(TPS65917_PMU_CONTROL_BASE,
				    TPS65917_POWER_CTRL);

	ret = regmap_write(tps65917->regmap[slave], addr, reg);
	if (ret)
		goto err_irq;

	/*
	 * If we are probing with DT do this the DT way and return here
	 * otherwise continue and add devices using mfd helpers.
	 */
	if (node) {
		ret = of_platform_populate(node, NULL, NULL, &i2c->dev);
		if (ret < 0)
			goto err_irq;
		else if (pdata->pm_off && !pm_power_off)
			tps65917_dev = tps65917;
	}

	return ret;

err_irq:
	regmap_del_irq_chip(tps65917->irq, tps65917->irq_data);
err_i2c:
	for (i = 1; i < TPS65917_NUM_CLIENTS; i++) {
		if (tps65917->i2c_clients[i])
			i2c_unregister_device(tps65917->i2c_clients[i]);
	}
	return ret;
}

static int tps65917_i2c_remove(struct i2c_client *i2c)
{
	struct tps65917 *tps65917 = i2c_get_clientdata(i2c);
	int i;

	regmap_del_irq_chip(tps65917->irq, tps65917->irq_data);

	for (i = 1; i < TPS65917_NUM_CLIENTS; i++) {
		if (tps65917->i2c_clients[i])
			i2c_unregister_device(tps65917->i2c_clients[i]);
	}

	return 0;
}

static const struct i2c_device_id tps65917_i2c_id[] = {
	{ "tps65917", },
};
MODULE_DEVICE_TABLE(i2c, tps65917_i2c_id);

static struct i2c_driver tps65917_i2c_driver = {
	.driver = {
		   .name = "tps65917",
		   .of_match_table = of_tps65917_match_tbl,
		   .owner = THIS_MODULE,
	},
	.probe = tps65917_i2c_probe,
	.remove = tps65917_i2c_remove,
	.id_table = tps65917_i2c_id,
};

static int __init tps65917_i2c_init(void)
{
	return i2c_add_driver(&tps65917_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(tps65917_i2c_init);

static void __exit tps65917_i2c_exit(void)
{
	i2c_del_driver(&tps65917_i2c_driver);
}
module_exit(tps65917_i2c_exit);

MODULE_AUTHOR("J Keerthy <j-keerthy@ti.com>");
MODULE_DESCRIPTION("TPS65917 chip family multi-function driver");
MODULE_LICENSE("GPL v2");
