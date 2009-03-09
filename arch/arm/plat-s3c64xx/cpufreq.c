/* linux/arch/arm/plat-s3c64xx/cpufreq.c
 *
 * Copyright 2009 Wolfson Microelectronics plc
 *
 * S3C64XX CPUfreq Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/cpu.h>

static struct clk *armclk;

static struct cpufreq_frequency_table s3c6410_freq_table[] = {
	{ 0,  66000 },
	{ 1, 133000 },
	{ 2, 222000 },
	{ 3, 266000 },
	{ 4, 333000 },
	{ 5, 400000 },
	{ 6, 532000 },
	{ 7, 533000 },
	{ 8, 667000 },
	{ 0, CPUFREQ_TABLE_END },
};

/* Data tables for current CPU and maximum index into it */
static struct cpufreq_frequency_table *s3c64xx_freq_table;

static int s3c64xx_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, s3c64xx_freq_table);
}

static unsigned int s3c64xx_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;

	return clk_get_rate(armclk) / 1000;
}

static int s3c64xx_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	int ret;
	unsigned int index;
	struct cpufreq_freqs freqs;

	ret = cpufreq_frequency_table_target(policy, s3c64xx_freq_table,
					     target_freq, relation, &index);
	if (ret != 0)
		return ret;

	freqs.cpu = 0;
	freqs.old = clk_get_rate(armclk) / 1000;
	freqs.new = s3c64xx_freq_table[index].frequency;
	freqs.flags = 0;

	if (freqs.old == freqs.new)
		return 0;

	pr_debug("cpufreq: Transition %d-%dkHz\n", freqs.old, freqs.new);

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	ret = clk_set_rate(armclk, freqs.new * 1000);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (ret < 0) {
		pr_err("cpufreq: Failed to set rate %dkHz: %d\n",
		       freqs.new, ret);
		return ret;
	}

	pr_debug("cpufreq: Set actual frequency %lukHz\n",
		 clk_get_rate(armclk) / 1000);

	return 0;
}

static int __init s3c64xx_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq;;

	if (policy->cpu != 0)
		return -EINVAL;

	if (cpu_is_s3c6410())
		s3c64xx_freq_table = s3c6410_freq_table;

	if (s3c64xx_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}

	armclk = clk_get(NULL, "armclk");
	if (IS_ERR(armclk)) {
		pr_err("cpufreq: Unable to obtain ARMCLK: %ld\n",
		       PTR_ERR(armclk));
		return PTR_ERR(armclk);
	}

	/* Check for frequencies we can generate */
	freq = s3c64xx_freq_table;
	while (freq->frequency != CPUFREQ_TABLE_END) {
		unsigned long r;

		r = clk_round_rate(armclk, freq->frequency * 1000);
		r /= 1000;

		if (r != freq->frequency)
			freq->frequency = CPUFREQ_ENTRY_INVALID;

		freq++;
	}

	policy->cur = clk_get_rate(armclk) / 1000;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	return cpufreq_frequency_table_cpuinfo(policy, s3c64xx_freq_table);
}

static struct cpufreq_driver s3c64xx_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= s3c64xx_cpufreq_verify_speed,
	.target		= s3c64xx_cpufreq_set_target,
	.get		= s3c64xx_cpufreq_get_speed,
	.init		= s3c64xx_cpufreq_driver_init,
	.name		= "s3c64xx",
};

static int __init s3c64xx_cpufreq_init(void)
{
	return cpufreq_register_driver(&s3c64xx_cpufreq_driver);
}
module_init(s3c64xx_cpufreq_init);
