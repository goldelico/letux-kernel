// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Ingenic XBurst SoCs IRQ support
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2021, 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/cpuhotplug.h>

#include <asm/io.h>
#include <asm/mach-ingenic/smp.h>

#define JZ_REG_INTC_STATUS	0x00
#define JZ_REG_INTC_MASK	0x04
#define JZ_REG_INTC_SET_MASK	0x08
#define JZ_REG_INTC_CLEAR_MASK	0x0c
#define JZ_REG_INTC_PENDING	0x10
#define CHIP_SIZE		0x20

#define JZ_INTC_MAX_CHIPS	2
#define JZ_INTC_IRQS_PER_CHIP	32

#define IF_ENABLED(cfg, ptr)	PTR_IF(IS_ENABLED(cfg), (ptr))

enum ingenic_intc_version {
	ID_JZ4730,
	ID_JZ4740,
	ID_JZ4725B,
	ID_JZ4760,
	ID_JZ4770,
	ID_JZ4775,
	ID_JZ4780,
	ID_X1600,
	ID_X2000,
};

struct ingenic_intc_irqchip {
	void __iomem *base;
	unsigned num_chips;

	struct irq_chip_generic *gc[];
};

struct ingenic_intc {
	enum ingenic_intc_version version;
	struct irq_domain *domain;
	struct ingenic_intc_irqchip __percpu *irqchips;

	int irq;
};

static struct ingenic_intc *ingenic_intc;

asmlinkage void plat_irq_dispatch(void)
{
	unsigned long pending = read_c0_cause() & read_c0_status() & ST0_IM;

	if (!pending) {
		spurious_interrupt();
		return;
	}

	if (pending & CAUSEF_IP3)
		do_IRQ(MIPS_CPU_IRQ_BASE + 3);
	else if (pending & CAUSEF_IP4)
		do_IRQ(MIPS_CPU_IRQ_BASE + 4);
	else if (pending & CAUSEF_IP2)
		do_IRQ(MIPS_CPU_IRQ_BASE + 2);
	else
		do_IRQ(MIPS_CPU_IRQ_BASE + __ffs(pending));
}

static struct ingenic_intc_irqchip *ingenic_intc_get_irqchip(unsigned cpu)
{
	if (ingenic_intc->version >= ID_X2000)
		return per_cpu_ptr(ingenic_intc->irqchips, cpu);
	else
		return ingenic_intc->irqchips;
}

static irqreturn_t ingenic_intc_cascade(int irq, void *data)
{
	struct ingenic_intc_irqchip *irqchip = data;
	uint32_t pending;
	unsigned i;

	for (i = 0; i < irqchip->num_chips; i++) {
		pending = irq_reg_readl(irqchip->gc[i], JZ_REG_INTC_PENDING);
		if (!pending)
			continue;

		while (pending) {
			int bit = __fls(pending);

			irq = irq_linear_revmap(ingenic_intc->domain,
						bit + (i * JZ_INTC_IRQS_PER_CHIP));
			generic_handle_irq(irq);
			pending &= ~BIT(bit);
		}
	}

	if (ingenic_intc->version == ID_JZ4780)
		IF_ENABLED(CONFIG_SMP, jz4780_smp_switch_irqcpu(smp_processor_id()));

	return IRQ_HANDLED;
}

static void ingenic_intc_cpu_irq_op(unsigned cpu, irq_hw_number_t irq, unsigned reg)
{
	struct ingenic_intc_irqchip *irqchip = ingenic_intc_get_irqchip(cpu);
	unsigned i = irq / JZ_INTC_IRQS_PER_CHIP;
	struct irq_chip_generic *gc = irqchip->gc[i];

	/* Suppress any operations on uninitialised chips associated with CPUs
	   yet to be started. */

	if (!gc)
		return;

	irq_gc_lock(gc);
	irq_reg_writel(gc, 1UL << (irq % JZ_INTC_IRQS_PER_CHIP), reg);
	irq_gc_unlock(gc);
}

static void ingenic_intc_mask_cpu_irq(unsigned cpu, irq_hw_number_t irq)
{
	ingenic_intc_cpu_irq_op(cpu, irq, JZ_REG_INTC_SET_MASK);
}

static void ingenic_intc_global_irq_op(struct irq_data *data, unsigned reg)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	const struct cpumask *affinity = irq_data_get_affinity_mask(data);
	unsigned cpu;

	for (cpu = 0; cpu < num_possible_cpus(); cpu++)
		if (cpumask_test_cpu(cpu, affinity))
			ingenic_intc_cpu_irq_op(cpu, irq, reg);
}

static void ingenic_intc_mask(struct irq_data *data)
{
	ingenic_intc_global_irq_op(data, JZ_REG_INTC_SET_MASK);
}

static void ingenic_intc_unmask(struct irq_data *data)
{
	ingenic_intc_global_irq_op(data, JZ_REG_INTC_CLEAR_MASK);
}

static int ingenic_intc_set_affinity(struct irq_data *data,
                                     const struct cpumask *dest,
                                     bool force)
{
	irq_hw_number_t irq = irqd_to_hwirq(data);
	const struct cpumask *affinity = irq_data_get_affinity_mask(data);
	unsigned cpu;

	/* Mask the IRQ for CPUs that are no longer to be associated with the IRQ. */

	for_each_cpu_andnot(cpu, affinity, dest)
		ingenic_intc_mask_cpu_irq(cpu, irq);

	irq_data_update_affinity(data, dest);
	irq_data_update_effective_affinity(data, dest);

	return IRQ_SET_MASK_OK;
}

static int __init ingenic_intc_setup_irqchip(unsigned int cpu)
{
	struct ingenic_intc_irqchip *irqchip;
	struct irq_chip_type *ct;
	unsigned int i;

	irqchip = ingenic_intc_get_irqchip(cpu);

	for (i = 0; i < irqchip->num_chips; i++) {
		irqchip->gc[i] = irq_get_domain_generic_chip(ingenic_intc->domain,
							     i * JZ_INTC_IRQS_PER_CHIP);

		irqchip->gc[i]->wake_enabled = IRQ_MSK(JZ_INTC_IRQS_PER_CHIP);
		irqchip->gc[i]->reg_base = irqchip->base + (i * CHIP_SIZE);

		ct = irqchip->gc[i]->chip_types;
		ct->regs.enable = JZ_REG_INTC_CLEAR_MASK;
		ct->regs.disable = JZ_REG_INTC_SET_MASK;
		ct->chip.irq_unmask = ingenic_intc_unmask;
		ct->chip.irq_mask = ingenic_intc_mask;
		ct->chip.irq_mask_ack = ingenic_intc_mask;
		ct->chip.irq_set_wake = irq_gc_set_wake;
		ct->chip.irq_set_affinity = ingenic_intc_set_affinity;
		ct->chip.flags = IRQCHIP_MASK_ON_SUSPEND;

		/* Mask all irqs */
		irq_reg_writel(irqchip->gc[i], IRQ_MSK(JZ_INTC_IRQS_PER_CHIP),
			       JZ_REG_INTC_SET_MASK);
	}

	return 0;
}

static const struct of_device_id __maybe_unused ingenic_intc_of_matches[] __initconst = {
	{ .compatible = "ingenic,jz4730-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4740-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4725b-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4760-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4770-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4775-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,jz4780-intc", .data = (void *) ID_JZ4780 },
	{ .compatible = "ingenic,x1000-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,x1600-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,x1830-intc", .data = (void *) ID_JZ4730 },
	{ .compatible = "ingenic,x2000-intc", .data = (void *) ID_X2000 },
	{ .compatible = "ingenic,x2500-intc", .data = (void *) ID_X2000 },
	{ /* sentinel */ }
};

static int __init ingenic_intc_probe(struct device_node *np, unsigned num_chips)
{
	const struct of_device_id *id = of_match_node(ingenic_intc_of_matches, np);
	struct ingenic_intc_irqchip *irqchip;
	struct ingenic_intc *intc;
	void __iomem *base;
	unsigned int cpu;
	int ret;

	intc = kzalloc(sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	intc->version = (enum ingenic_intc_version)id->data;

	if (intc->version >= ID_X2000)
		intc->irqchips = __alloc_percpu(struct_size(irqchip, gc, num_chips), 1);
	else
		intc->irqchips = kzalloc(struct_size(irqchip, gc, num_chips), GFP_KERNEL);

	if (!intc->irqchips) {
		ret = -ENOMEM;
		goto out_free;
	}

	base = of_io_request_and_map(np, 0, of_node_full_name(np));
	if (!base) {
		pr_err("%s: Failed to map INTC registers\n", __func__);
		ret = PTR_ERR(base);
		goto out_free_percpu;
	}

	intc->irq = irq_of_parse_and_map(np, 0);
	if (!intc->irq) {
		pr_crit("%s: Cannot to get INTC IRQ\n", __func__);
		ret = -EINVAL;
		goto out_unmap_base;
	}

	for (cpu = 0; cpu < num_possible_cpus(); cpu++) {
		if (intc->version >= ID_X2000) {
			irqchip = per_cpu_ptr(intc->irqchips, cpu);
			irqchip->base = base + 0x100 * cpu;
		} else {
			irqchip = intc->irqchips;
			irqchip->base = base;
		}

		irqchip->num_chips = num_chips;
	}

	ingenic_intc = intc;

	return 0;

out_unmap_base:
	iounmap(base);
out_free_percpu:
	free_percpu(intc->irqchips);
out_free:
	kfree(intc);
	return ret;
}

static int __init ingenic_intc_of_init(struct device_node *np,
				       unsigned num_chips)
{
	struct ingenic_intc *intc;
	struct irq_domain *domain;
	int ret;

	ret = ingenic_intc_probe(np, num_chips);
	if (ret) {
		pr_crit("%s: Failed to initialize INTC clocks: %d\n", __func__, ret);
		return ret;
	}

	intc = ingenic_intc;
	if (IS_ERR(intc))
		return PTR_ERR(intc);

	domain = irq_domain_add_linear(np, num_chips * JZ_INTC_IRQS_PER_CHIP,
				       &irq_generic_chip_ops, NULL);
	if (!domain) {
		ret = -ENOMEM;
		goto out_unmap_irq;
	}

	intc->domain = domain;

	ret = irq_alloc_domain_generic_chips(domain, JZ_INTC_IRQS_PER_CHIP, 1, "INTC",
					     handle_level_irq, 0, IRQ_NOPROBE | IRQ_LEVEL, 0);
	if (ret)
		goto out_domain_remove;

	if (intc->version >= ID_X2000) {
		ret = request_percpu_irq(intc->irq, ingenic_intc_cascade,
					 "SoC intc cascade interrupt", intc->irqchips);
		if (ret) {
			pr_err("Failed to register SoC intc cascade interrupt\n");
			goto out_domain_remove;
		}

		/* Setup irqchips on each CPU core */
		ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "irq-ingenic:online",
					ingenic_intc_setup_irqchip, NULL);
		if (ret < 0) {
			pr_crit("%s: Unable to init percpu irqchips: %x\n", __func__, ret);
			goto out_domain_remove;
		}
	} else {
		ret = request_irq(intc->irq, ingenic_intc_cascade, IRQF_NO_SUSPEND,
				  "SoC intc cascade interrupt", intc->irqchips);
		if (ret) {
			pr_err("Failed to register SoC intc cascade interrupt\n");
			goto out_domain_remove;
		}

		ret = ingenic_intc_setup_irqchip(0);
		if (ret < 0) {
			pr_crit("%s: Unable to init percpu irqchips: %x\n", __func__, ret);
			goto out_domain_remove;
		}
	}

	return 0;

out_domain_remove:
	irq_domain_remove(domain);
out_unmap_irq:
	irq_dispose_mapping(intc->irq);
	free_percpu(intc->irqchips);
	kfree(intc);
	return ret;
}

static int __init intc_1chip_of_init(struct device_node *np,
				     struct device_node *parent)
{
	return ingenic_intc_of_init(np, 1);
}
IRQCHIP_DECLARE(jz4730_intc, "ingenic,jz4730-intc", intc_1chip_of_init);
IRQCHIP_DECLARE(jz4740_intc, "ingenic,jz4740-intc", intc_1chip_of_init);
IRQCHIP_DECLARE(jz4725b_intc, "ingenic,jz4725b-intc", intc_1chip_of_init);

static int __init intc_2chip_of_init(struct device_node *np,
	struct device_node *parent)
{
	return ingenic_intc_of_init(np, 2);
}
IRQCHIP_DECLARE(jz4760_intc, "ingenic,jz4760-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(jz4770_intc, "ingenic,jz4770-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(jz4775_intc, "ingenic,jz4775-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(jz4780_intc, "ingenic,jz4780-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(x1000_intc, "ingenic,x1000-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(x1600_intc, "ingenic,x1600-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(x1830_intc, "ingenic,x1830-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(x2000_intc, "ingenic,x2000-intc", intc_2chip_of_init);
IRQCHIP_DECLARE(x2500_intc, "ingenic,x2500-intc", intc_2chip_of_init);
