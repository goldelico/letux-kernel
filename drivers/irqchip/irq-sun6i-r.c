// SPDX-License-Identifier: GPL-2.0-only
//
// Allwinner A31 and newer SoCs R_INTC driver
//

#include <linux/atomic.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/syscore_ops.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

#define NMI_HWIRQ		0

#define SUN6I_R_INTC_NR_IRQS	16

#define SUN6I_R_INTC_CTRL	0x0c
#define SUN6I_R_INTC_PENDING	0x10
#define SUN6I_R_INTC_ENABLE	0x40
#define SUN6I_R_INTC_MASK	0x50

enum {
	SUNXI_SRC_TYPE_LEVEL_LOW = 0,
	SUNXI_SRC_TYPE_EDGE_FALLING,
	SUNXI_SRC_TYPE_LEVEL_HIGH,
	SUNXI_SRC_TYPE_EDGE_RISING,
};

static void __iomem *base;
static irq_hw_number_t parent_offset;
static u32 parent_type;
#ifdef CONFIG_PM_SLEEP
static atomic_t wake_mask;
#endif

static void sun6i_r_intc_irq_enable(struct irq_data *data)
{
	if (data->hwirq == NMI_HWIRQ)
		writel(BIT(NMI_HWIRQ), base + SUN6I_R_INTC_ENABLE);

	irq_chip_enable_parent(data);
}

static void sun6i_r_intc_irq_disable(struct irq_data *data)
{
	if (data->hwirq == NMI_HWIRQ)
		writel(0, base + SUN6I_R_INTC_ENABLE);

	irq_chip_disable_parent(data);
}

static void sun6i_r_intc_irq_mask(struct irq_data *data)
{
	if (data->hwirq == NMI_HWIRQ)
		writel(BIT(NMI_HWIRQ), base + SUN6I_R_INTC_MASK);

	irq_chip_mask_parent(data);
}

static void sun6i_r_intc_irq_unmask(struct irq_data *data)
{
	if (data->hwirq == NMI_HWIRQ)
		writel(0, base + SUN6I_R_INTC_MASK);

	irq_chip_unmask_parent(data);
}

static void sun6i_r_intc_irq_eoi(struct irq_data *data)
{
	if (data->hwirq == NMI_HWIRQ)
		writel(BIT(NMI_HWIRQ), base + SUN6I_R_INTC_PENDING);

	irq_chip_eoi_parent(data);
}

static int sun6i_r_intc_irq_set_type(struct irq_data *data, unsigned int type)
{
	if (data->hwirq == NMI_HWIRQ) {
		u32 src_type;

		switch (type) {
		case IRQ_TYPE_EDGE_FALLING:
			src_type = SUNXI_SRC_TYPE_EDGE_FALLING;
			break;
		case IRQ_TYPE_EDGE_RISING:
			src_type = SUNXI_SRC_TYPE_EDGE_RISING;
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			src_type = SUNXI_SRC_TYPE_LEVEL_HIGH;
			break;
		case IRQ_TYPE_NONE:
		case IRQ_TYPE_LEVEL_LOW:
			src_type = SUNXI_SRC_TYPE_LEVEL_LOW;
			break;
		default:
			pr_err("%pOF: invalid trigger type %d for IRQ %d\n",
			       irq_domain_get_of_node(data->domain), type,
			       data->irq);
			return -EBADR;
		}
		writel(src_type, base + SUN6I_R_INTC_CTRL);

		irqd_set_trigger_type(data, type);

		/* Send the R_INTC -> GIC trigger type to the GIC driver. */
		type = parent_type;
	}

	return irq_chip_set_type_parent(data, type);
}

#ifdef CONFIG_PM_SLEEP
static int sun6i_r_intc_irq_set_wake(struct irq_data *data, unsigned int on)
{
	if (on)
		atomic_or(BIT(data->hwirq), &wake_mask);
	else
		atomic_andnot(BIT(data->hwirq), &wake_mask);

	/* GIC cannot wake, so there is no need to call the parent hook. */
	return 0;
}
#else
#define sun6i_r_intc_irq_set_wake NULL
#endif

static struct irq_chip sun6i_r_intc_chip = {
	.name			= "sun6i-r-intc",
	.irq_enable		= sun6i_r_intc_irq_enable,
	.irq_disable		= sun6i_r_intc_irq_disable,
	.irq_mask		= sun6i_r_intc_irq_mask,
	.irq_unmask		= sun6i_r_intc_irq_unmask,
	.irq_eoi		= sun6i_r_intc_irq_eoi,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= sun6i_r_intc_irq_set_type,
	.irq_set_wake		= sun6i_r_intc_irq_set_wake,
	.irq_set_vcpu_affinity	= irq_chip_set_vcpu_affinity_parent,
};

static int sun6i_r_intc_domain_translate(struct irq_domain *domain,
					 struct irq_fwspec *fwspec,
					 unsigned long *hwirq,
					 unsigned int *type)
{
	if (!is_of_node(fwspec->fwnode) || fwspec->param_count != 2)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type  = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static int sun6i_r_intc_domain_alloc(struct irq_domain *domain,
				     unsigned int virq,
				     unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec gic_fwspec;
	irq_hw_number_t hwirq;
	unsigned int type;
	int i, ret;

	ret = sun6i_r_intc_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;
	if (hwirq + nr_irqs > SUN6I_R_INTC_NR_IRQS)
		return -EINVAL;

	/* Construct a GIC-compatible fwspec from this fwspec. */
	gic_fwspec = (struct irq_fwspec) {
		.fwnode      = domain->parent->fwnode,
		.param_count = 3,
		.param       = { GIC_SPI, parent_offset + hwirq, type },
	};

	for (i = 0; i < nr_irqs; ++i)
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &sun6i_r_intc_chip, NULL);

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &gic_fwspec);
}

static const struct irq_domain_ops sun6i_r_intc_domain_ops = {
	.translate	= sun6i_r_intc_domain_translate,
	.alloc		= sun6i_r_intc_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

#ifdef CONFIG_PM_SLEEP
static int sun6i_r_intc_suspend(void)
{
	/* All wake IRQs are enabled during suspend. */
	writel(atomic_read(&wake_mask), base + SUN6I_R_INTC_ENABLE);

	return 0;
}

static void sun6i_r_intc_resume(void)
{
	u32 mask = atomic_read(&wake_mask) & BIT(NMI_HWIRQ);

	/* Only the NMI is relevant during normal operation. */
	writel(mask, base + SUN6I_R_INTC_ENABLE);
}

static struct syscore_ops sun6i_r_intc_syscore_ops = {
	.suspend	= sun6i_r_intc_suspend,
	.resume		= sun6i_r_intc_resume,
};

static void sun6i_r_intc_syscore_init(void)
{
	register_syscore_ops(&sun6i_r_intc_syscore_ops);
}
#else
static inline void sun6i_r_intc_syscore_init(void) {}
#endif

static int __init sun6i_r_intc_init(struct device_node *node,
				    struct device_node *parent)
{
	struct irq_domain *domain, *parent_domain;
	struct of_phandle_args parent_irq;
	int ret;

	/* Extract the R_INTC -> GIC mapping from the OF node. */
	ret = of_irq_parse_one(node, 0, &parent_irq);
	if (ret)
		return ret;
	if (parent_irq.args_count != 3 || parent_irq.args[0] != GIC_SPI)
		return -EINVAL;
	parent_offset = parent_irq.args[1];
	parent_type   = parent_irq.args[2];

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%pOF: Failed to obtain parent domain\n", node);
		return -ENXIO;
	}

	base = of_io_request_and_map(node, 0, NULL);
	if (IS_ERR(base)) {
		pr_err("%pOF: Failed to map MMIO region\n", node);
		return PTR_ERR(base);
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0,
					  SUN6I_R_INTC_NR_IRQS, node,
					  &sun6i_r_intc_domain_ops, NULL);
	if (!domain) {
		pr_err("%pOF: Failed to allocate domain\n", node);
		iounmap(base);
		return -ENOMEM;
	}

	/* Disable and unmask all interrupts. */
	writel(0, base + SUN6I_R_INTC_ENABLE);
	writel(0, base + SUN6I_R_INTC_MASK);

	/* Clear any pending interrupts. */
	writel(~0, base + SUN6I_R_INTC_PENDING);

	sun6i_r_intc_syscore_init();

	return 0;
}
IRQCHIP_DECLARE(sun6i_r_intc, "allwinner,sun6i-a31-r-intc", sun6i_r_intc_init);
