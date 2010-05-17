#include <linux/kernel.h>

#include <asm/fiq.h>
#include <mach/regs-irq.h>
#include <plat/regs-timer.h>
#include <mach/irqs.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/hdq.h>

/* -------------------------------------------------------------------------------
 * GTA02 FIQ related
 *
 * Calls into vibrator and hdq and based on the return values
 * determines if we the FIQ source be kept alive
 */

#define DIVISOR_FROM_US(x) ((x) << 3)

#ifdef CONFIG_HDQ_GPIO_BITBANG
#define FIQ_DIVISOR_HDQ DIVISOR_FROM_US(HDQ_SAMPLE_PERIOD_US)
extern int hdq_fiq_handler(void);
#endif

/* Global data related to our fiq source */
static uint32_t gta02_fiq_ack_mask;
static const int gta02_gta02_fiq_timer_id = 2;

struct pwm_device* gta02_fiq_timer;

void gta02_fiq_handler(void)
{
	unsigned long intmask;
	int keep_running = 0;
	/* disable further timer interrupts if nobody has any work
	 * or adjust rate according to who still has work
	 *
	 * CAUTION: it means forground code must disable FIQ around
	 * its own non-atomic S3C2410_INTMSK changes... not common
	 * thankfully and taken care of by the fiq-basis patch
	 */

#ifdef CONFIG_HDQ_GPIO_BITBANG
	keep_running = hdq_fiq_handler();
#endif
	if (!keep_running) {
		/* Disable irq */
		intmask = __raw_readl(S3C2410_INTMSK);
		intmask |= (gta02_fiq_ack_mask);
		__raw_writel(intmask, S3C2410_INTMSK);
	}

	__raw_writel(gta02_fiq_ack_mask, S3C2410_SRCPND);
}

void gta02_fiq_kick(void)
{
	unsigned long flags;
	unsigned long intmask;
	/* we have to take care about FIQ because this modification is
	 * non-atomic, FIQ could come in after the read and before the
	 * writeback and its changes to the register would be lost
	 * (platform INTMSK mod code is taken care of already)
	 */
	local_save_flags(flags);
	local_fiq_disable();

	/* allow FIQs to resume */
	intmask = __raw_readl(S3C2410_INTMSK);
	intmask &= ~(gta02_fiq_ack_mask);
	__raw_writel(intmask, S3C2410_INTMSK);

	local_irq_restore(flags);

}

int gta02_fiq_enable(void)
{
	int ret = 0;

	local_fiq_disable();

	gta02_fiq_timer = pwm_request(gta02_gta02_fiq_timer_id, "fiq timer");

	if (IS_ERR(gta02_fiq_timer)) {
		ret = PTR_ERR(gta02_fiq_timer);
		printk("GTA02 FIQ: Could not request fiq timer: %d\n", ret);
		return ret;
	}

	gta02_fiq_ack_mask = 1 << (IRQ_TIMER0 + gta02_gta02_fiq_timer_id
					- S3C2410_CPUIRQ_OFFSET);


	ret = pwm_config(gta02_fiq_timer, HDQ_SAMPLE_PERIOD_US * 1000,
					HDQ_SAMPLE_PERIOD_US * 1000);
	if (ret) {
		printk("GTA02 FIQ: Could not configure fiq timer: %d\n", ret);
		goto err;
	}

	set_fiq_c_handler(gta02_fiq_handler);

	__raw_writel(gta02_fiq_ack_mask, S3C2410_INTMOD);

	pwm_enable(gta02_fiq_timer);

	local_fiq_enable();

	return 0;

err:
	pwm_free(gta02_fiq_timer);

	return ret;
}

void gta02_fiq_disable(void)
{
	local_fiq_disable();

	if (!gta02_fiq_timer)
		return;

	__raw_writel(0, S3C2410_INTMOD);
	set_fiq_c_handler(NULL);

	pwm_disable(gta02_fiq_timer);

	pwm_free(gta02_fiq_timer);

	gta02_fiq_timer = NULL;
}
/* -------------------- /GTA02 FIQ Handler ------------------------------------- */
