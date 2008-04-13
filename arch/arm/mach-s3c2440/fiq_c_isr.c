/*
 * Copyright 2007  Andy Green <andy@warmcat.com>
 * S3C modfifications
 * Copyright 2008 Andy Green <andy@openmoko.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/hardware.h>
#include <asm/fiq.h>
#include "fiq_c_isr.h"
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/irq.h>

#include <asm/arch/pwm.h>
#include <asm/plat-s3c/regs-timer.h>

/*
 * Major Caveats for using FIQ
 * ---------------------------
 *
 * 1) it CANNOT touch any vmalloc()'d memory, only memory
 *    that was kmalloc()'d.  Static allocations in the monolithic kernel
 *    are kmalloc()'d so they are okay.  You can touch memory-mapped IO, but
 *    the pointer for it has to have been stored in kmalloc'd memory.  The
 *    reason for this is simple: every now and then Linux turns off interrupts
 *    and reorders the paging tables.  If a FIQ happens during this time, the
 *    virtual memory space can be partly or entirely disordered or missing.
 *
 * 2) Because vmalloc() is used when a module is inserted, THIS FIQ
 *    ISR HAS TO BE IN THE MONOLITHIC KERNEL, not a module.  But the way
 *    it is set up, you can all to enable and disable it from your module
 *    and intercommunicate with it through struct fiq_ipc
 *    fiq_ipc which you can define in
 *    asm/archfiq_ipc_type.h.  The reason is the same as above, a
 *    FIQ could happen while even the ISR is not present in virtual memory
 *    space due to pagetables being changed at the time.
 *
 * 3) You can't call any Linux API code except simple macros
 *    - understand that FIQ can come in at any time, no matter what
 *      state of undress the kernel may privately be in, thinking it
 *      locked the door by turning off interrupts... FIQ is an
 *      unstoppable monster force (which is its value)
 *    - they are not vmalloc()'d memory safe
 *    - they might do crazy stuff like sleep: FIQ pisses fire and
 *      is not interested in 'sleep' that the weak seem to need
 *    - calling APIs from FIQ can re-enter un-renterable things
 *    - summary: you cannot interoperate with linux APIs directly in the FIQ ISR
 *
 * If you follow these rules, it is fantastic, an extremely powerful, solid,
 * genuine hard realtime feature.
 *
 */

/* more than enough to cover our jump instruction to the isr */
#define SIZEOF_FIQ_JUMP 8
/* more than enough to cover s3c2440_fiq_isr() in 4K blocks */
#define SIZEOF_FIQ_ISR 0x2000
/* increase the size of the stack that is active during FIQ as needed */
static u8 u8aFiqStack[4096];

/* only one FIQ ISR possible, okay to do these here */
u32 _fiq_ack_mask; /* used by isr exit define */
unsigned long _fiq_count_fiqs; /* used by isr exit define */
static int _fiq_irq; /* private ; irq index we were started with, or 0 */
struct s3c2410_pwm pwm_timer_fiq;
int _fiq_timer_index;
u16 _fiq_timer_divisor;


/* this function must live in the monolithic kernel somewhere!  A module is
 * NOT good enough!
 */
extern void __attribute__ ((naked)) s3c2440_fiq_isr(void);

/* this is copied into the hard FIQ vector during init */

static void __attribute__ ((naked)) s3c2440_FIQ_Branch(void)
{
	asm __volatile__ (
		"mov pc, r8 ; "
	);
}

/* sysfs */

static ssize_t show_count(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%ld\n", _fiq_count_fiqs);
}

static DEVICE_ATTR(count, 0444, show_count, NULL);

static struct attribute *s3c2440_fiq_sysfs_entries[] = {
	&dev_attr_count.attr,
	NULL
};

static struct attribute_group s3c2440_fiq_attr_group = {
	.name	= "fiq",
	.attrs	= s3c2440_fiq_sysfs_entries,
};

/*
 * call this from your kernel module to set up the FIQ ISR to service FIQs,
 * You need to have configured your FIQ input pin before anything will happen
 *
 * call it with, eg, IRQ_TIMER3 from asm-arm/arch-s3c2410/irqs.h
 *
 * you still need to clear the source interrupt in S3C2410_INTMSK to get
 * anything good happening
 */
static int fiq_init_irq_source(int irq_index_fiq)
{
	int rc = 0;

	if (!irq_index_fiq) /* no interrupt */
		goto bail;

	local_fiq_disable();

	_fiq_irq = irq_index_fiq;
	_fiq_ack_mask = 1 << (irq_index_fiq - S3C2410_CPUIRQ_OFFSET);
	timer_index = (irq_index_fiq - IRQ_TIMER0);

	/* set up the timer to operate as a pwm device */

	rc = s3c2410_pwm_init(&pwm_timer_fiq);
	if (rc)
		goto bail;

	pwm_timer_fiq.timerid = PWM0 + timer_index;
	pwm_timer_fiq.prescaler = (6 - 1) / 2;
	pwm_timer_fiq.divider = S3C2410_TCFG1_MUX3_DIV2;
	/* default rate == ~32us */
	pwm_timer_fiq.counter = pwm_timer_fiq.comparer =
					timer_divisor = 64;

	rc = s3c2410_pwm_enable(&pwm_timer_fiq);
	if (rc)
		goto bail;

	s3c2410_pwm_start(&pwm_timer_fiq);

	/* let our selected interrupt be a magic FIQ interrupt */
	__raw_writel(_fiq_ack_mask, S3C2410_INTMOD);

	/* it's ready to go as soon as we unmask the source in S3C2410_INTMSK */
	local_fiq_enable();
bail:
	return rc;
}


/* call this from your kernel module to disable generation of FIQ actions */
static void fiq_disable_irq_source(void)
{
	/* nothing makes FIQ any more */
	__raw_writel(0, S3C2410_INTMOD);
	local_fiq_disable();
	_fiq_irq = 0; /* no active source interrupt now either */
}

/*
 * fiq_kick() forces a FIQ event to happen shortly after leaving the routine
 */
void fiq_kick(void)
{
	unsigned long flags;
	u32 tcon;

	/* we have to take care about FIQ because this modification is
	 * non-atomic, FIQ could come in after the read and before the
	 * writeback and its changes to the register would be lost
	 * (platform INTMSK mod code is taken care of already)
	 */
	local_save_flags(flags);
	local_fiq_disable();
	/* allow FIQs to resume */
	__raw_writel(__raw_readl(S3C2410_INTMSK) &
		     ~(1 << (_fiq_irq - S3C2410_CPUIRQ_OFFSET)),
		     S3C2410_INTMSK);
	tcon = __raw_readl(S3C2410_TCON) & ~S3C2410_TCON_T3START;
	/* fake the timer to a count of 1 */
	__raw_writel(1, S3C2410_TCNTB(timer_index));
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD, S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD | S3C2410_TCON_T3START,
		     S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3START, S3C2410_TCON);
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(fiq_kick);




static int __init sc32440_fiq_probe(struct platform_device *pdev)
{
	struct resource *r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!r)
		return -EIO;
	/* configure for the interrupt we are meant to use */
	printk(KERN_INFO"Enabling FIQ using irq %d\n", r->start);

	fiq_init_irq_source(r->start);

	return sysfs_create_group(&pdev->dev.kobj, &s3c2440_fiq_attr_group);
}

static int sc32440_fiq_remove(struct platform_device *pdev)
{
	fiq_disable_irq_source();
	sysfs_remove_group(&pdev->dev.kobj, &s3c2440_fiq_attr_group);
	return 0;
}

static void fiq_set_vector_and_regs(void)
{
	struct pt_regs regs;

	/* prep the special FIQ mode regs */
	memset(&regs, 0, sizeof(regs));
	regs.ARM_r8 = (unsigned long)s3c2440_fiq_isr;
	regs.ARM_sp = (unsigned long)u8aFiqStack + sizeof(u8aFiqStack) - 4;
	/* set up the special FIQ-mode-only registers from our regs */
	set_fiq_regs(&regs);
	/* copy our jump to the real ISR into the hard vector address */
	set_fiq_handler(s3c2440_FIQ_Branch, SIZEOF_FIQ_JUMP);
}

#ifdef CONFIG_PM
static int sc32440_fiq_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* nothing makes FIQ any more */
	__raw_writel(0, S3C2410_INTMOD);
	local_fiq_disable();

	return 0;
}

static int sc32440_fiq_resume(struct platform_device *pdev)
{
	fiq_set_vector_and_regs();
	fiq_init_irq_source(_fiq_irq);
	return 0;
}
#else
#define sc32440_fiq_suspend	NULL
#define sc32440_fiq_resume	NULL
#endif

static struct platform_driver sc32440_fiq_driver = {
	.driver = {
		.name	= "sc32440_fiq",
		.owner	= THIS_MODULE,
	},

	.probe	 = sc32440_fiq_probe,
	.remove	 = __devexit_p(sc32440_fiq_remove),
	.suspend = sc32440_fiq_suspend,
	.resume	 = sc32440_fiq_resume,
};

static int __init sc32440_fiq_init(void)
{
	fiq_set_vector_and_regs();

	return platform_driver_register(&sc32440_fiq_driver);
}

static void __exit sc32440_fiq_exit(void)
{
	fiq_disable_irq_source();
}

MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_LICENSE("GPL");

module_init(sc32440_fiq_init);
module_exit(sc32440_fiq_exit);
