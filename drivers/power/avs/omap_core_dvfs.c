/*
 * Copyright (C) 2014 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/opp.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/err.h>

#define DRIVER_NAME	"omap-core-dvfs"


struct omap_core_dvfs_map {
	unsigned long cpu_freq;
	unsigned long core_freq;
};

struct omap_core_dvfs_data {
	struct device *dev;
	struct clk *l3_clock;
	struct clk *dpll_clock;
	struct regulator *reg;
	unsigned int volt_tolerance;
	long curr_freq;
	struct omap_core_dvfs_map *map;
};

static struct omap_core_dvfs_data *core_dvfs_data;

static const struct of_device_id omap_core_dvfs_match_tbl[] __initdata = {
	{.compatible = "ti,omap-core-dvfs"},
	{},
};
MODULE_DEVICE_TABLE(of, omap_core_dvfs_match_tbl);
static int get_match_freq(unsigned long cpu_freq, unsigned long *core_freq)
{
	struct omap_core_dvfs_map *map = core_dvfs_data->map;
	while (map->core_freq && map->cpu_freq) {
		if (map->cpu_freq == cpu_freq) {
			*core_freq = map->core_freq;
			return 0;
		}
		map++;
	}
	return -ENODATA;
}
static int cpufreq_trans(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *)data;
	struct omap_core_dvfs_data *pdata = core_dvfs_data;
	int ret = 0;
	long d, new_freq, dpll_freq, old_freq;
	struct opp *opp;
	unsigned long volt = 0, volt_old = 0, tol = 0;

	if (IS_ERR_OR_NULL(freqs))
		return -ENODATA;

	if (val != CPUFREQ_PRECHANGE || freqs->new == freqs->old)
		return 0;

	ret = get_match_freq(freqs->new * 1000, &new_freq);
	if (ret) {
		pr_err(
		"Could not find cpu freq %u in core map\n", freqs->new * 1000);
		goto f_out;
	}
	if (pdata->curr_freq == new_freq)
		return 0;

	/* calculate target frequency to be set in dpll */
	old_freq = clk_get_rate(pdata->l3_clock);
	d = clk_get_rate(pdata->dpll_clock) / old_freq;
	dpll_freq = clk_round_rate(pdata->dpll_clock, new_freq * d);
	if (dpll_freq < 0)
		dpll_freq = new_freq * d;

	rcu_read_lock();
	opp = opp_find_freq_ceil(pdata->dev, &new_freq);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		pr_err("failed to find core OPP for %ld\n", new_freq);
		ret = PTR_ERR(opp);
		goto f_out;
	}
	volt = opp_get_voltage(opp);
	rcu_read_unlock();

	tol = volt * pdata->volt_tolerance / 100;
	volt_old = regulator_get_voltage(pdata->reg);

	pr_debug("L3 DVFS %ld MHz, %ld mV --> %ld MHz, %ld mV\n",
		 old_freq / 1000000, volt_old ? volt_old / 1000 : -1,
		 dpll_freq / d / 1000000, volt ? volt / 1000 : -1);

	if (freqs->new > freqs->old) {
		ret = regulator_set_voltage_tol(pdata->reg, volt, tol);
		if (ret) {
			pr_err("failed to scale core voltage up: %d\n", ret);
			goto f_out;
		}
	}
	ret = clk_set_rate(pdata->dpll_clock, dpll_freq);
	if (ret) {
		pr_err("failed to set core clock rate: %d\n", ret);
		regulator_set_voltage_tol(pdata->reg, volt_old, tol);
		goto f_out;
	}
	if (freqs->new < freqs->old) {
		ret = regulator_set_voltage_tol(pdata->reg, volt, tol);
		if (ret) {
			pr_err("failed to scale voltage down: %d\n", ret);
			clk_set_rate(pdata->dpll_clock, old_freq * d);
			goto f_out;
		}
	}
	pdata->curr_freq = new_freq;
f_out:
	return ret;
}

static struct notifier_block cpufreq_trans_block = {
	.notifier_call = cpufreq_trans
};

static int of_init_opp_map(struct device *dev, struct omap_core_dvfs_map **map)
{
	const struct property *prop;
	const __be32 *val;
	struct omap_core_dvfs_map *m;
	int nr, i;

	prop = of_find_property(dev->of_node, "map", NULL);
	if (!prop)
		return -ENODEV;
	if (!prop->value)
		return -ENODATA;

	/*
	 * Each entry is a set of tuples consisting of freq-kHz from
	 * OPP CPU list and index of matching OPP in core list.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2) {
		dev_err(dev, "Invalid map\n");
		return -EINVAL;
	}

	m = devm_kzalloc(dev, prop->length, GFP_KERNEL);
	if (!m) {
		dev_err(dev, "Unable to create new map\n");
		return -ENOMEM;
	}
	*map = m;
	val = prop->value;
	for (i = 0; i < nr; i += 2, m++) {
		m->cpu_freq = be32_to_cpup(val++) * 1000;
		m->core_freq = be32_to_cpup(val++) * 1000;
	}
	m->cpu_freq = 0;
	m->core_freq = 0;

	return 0;
}

static int omap_core_dvfs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *nd = dev->of_node;
	int ret;
	struct omap_core_dvfs_data *data;
	const char *pname, *str;
	struct regulator *reg;

	if (!nd) {
		dev_err(dev, "no OF information?\n");
		return -EINVAL;
	}

	reg = devm_regulator_get(dev, "core_dvfs");
	if (IS_ERR(reg)) {
		dev_err(dev, "core_dvfs regulator not ready, retry\n");
		return -EPROBE_DEFER;
	}


	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Unable to allocate data\n");
		return -ENOMEM;
	}

	ret = of_init_opp_table(dev);
	if (ret) {
		dev_err(dev, "Failed to init OPP table: %d\n", ret);
		goto fail;
	}

	ret = of_init_opp_map(dev, &data->map);
	if (ret) {
		dev_err(dev, "Failed to init map: %d\n", ret);
		goto fail;
	}
	pname = "l3_clkname";
	ret = of_property_read_string(nd, pname, &str);
	if (ret)
		goto property_err;

	data->l3_clock = devm_clk_get(dev, str);
	if (IS_ERR(data->l3_clock)) {
		ret = PTR_ERR(data->l3_clock);
		dev_err(dev, "Failed to get %s clock: %d\n", str, ret);
		goto fail;
	}

	pname = "dpll_clkname";
	ret = of_property_read_string(nd, pname, &str);
	if (ret)
		goto property_err;

	data->dpll_clock = devm_clk_get(dev, str);
	if (IS_ERR(data->dpll_clock)) {
		ret = PTR_ERR(data->dpll_clock);
		dev_err(dev, "Failed to get %s clock: %d\n", str, ret);
		goto fail;
	}

	of_property_read_u32(nd, "voltage-tolerance", &data->volt_tolerance);

	ret = cpufreq_register_notifier(&cpufreq_trans_block,
		CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		dev_err(dev, "CPU notifier registration failed with %d\n", ret);
		cpufreq_unregister_notifier(
			&cpufreq_trans_block, CPUFREQ_TRANSITION_NOTIFIER);
		goto fail;
	}
	data->reg = reg;
	data->dev = dev;
	core_dvfs_data = data;
	platform_set_drvdata(pdev, data);

	return 0;

property_err:
	dev_err(dev, " Missing/Invalid '%s' property\n", pname);

fail:
	return ret;
}

static struct platform_driver omap_core_dvfs_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(omap_core_dvfs_match_tbl),
		},
	.probe = omap_core_dvfs_probe,
};
static int __init omap_core_dvfs_init(void)
{
	int ret;
	ret = platform_driver_register(&omap_core_dvfs_driver);
	if (ret)
		pr_err("driver register failed for omap_pmic(%d)\n", ret);
	return ret;
}
device_initcall_sync(omap_core_dvfs_init);

static void __exit omap_core_dvfs_exit(void)
{
	platform_driver_unregister(&omap_core_dvfs_driver);
}
module_exit(omap_core_dvfs_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("OMAP Global PRM driver");
MODULE_LICENSE("GPL v2");

