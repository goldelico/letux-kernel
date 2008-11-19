/*
 * arch/arm/mach-s3c2410/3c2410-pwm.c
 *
 * Copyright (c) by Javi Roman <javiroman@kernel-labs.org>
 * 		 for the OpenMoko Project.
 *
 *     S3C2410A SoC PWM support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/hardware.h>
#include <asm/plat-s3c/regs-timer.h>
#include <asm/arch/pwm.h>

int s3c2410_pwm_disable(struct s3c2410_pwm *pwm)
{
	unsigned long tcon;

	/* stop timer */
	tcon = __raw_readl(S3C2410_TCON);
	tcon &= 0xffffff00;
	__raw_writel(tcon, S3C2410_TCON);

	clk_disable(pwm->pclk);
	clk_put(pwm->pclk);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_disable);

int s3c2410_pwm_init(struct s3c2410_pwm *pwm)
{
	pwm->pclk = clk_get(NULL, "timers");
	if (IS_ERR(pwm->pclk))
		return PTR_ERR(pwm->pclk);

	clk_enable(pwm->pclk);
	pwm->pclk_rate = clk_get_rate(pwm->pclk);
	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_init);

int s3c2410_pwm_enable(struct s3c2410_pwm *pwm)
{
	unsigned long tcfg0, tcfg1, tcnt, tcmp;

	/* control registers bits */
	tcfg1 = __raw_readl(S3C2410_TCFG1);
	tcfg0 = __raw_readl(S3C2410_TCFG0);

	/* divider & scaler slection */
	switch (pwm->timerid) {
	case PWM0:
		tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		break;
	case PWM1:
		tcfg1 &= ~S3C2410_TCFG1_MUX1_MASK;
		tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		break;
	case PWM2:
		tcfg1 &= ~S3C2410_TCFG1_MUX2_MASK;
		tcfg0 &= ~S3C2410_TCFG_PRESCALER1_MASK;
		break;
	case PWM3:
		tcfg1 &= ~S3C2410_TCFG1_MUX3_MASK;
		tcfg0 &= ~S3C2410_TCFG_PRESCALER1_MASK;
		break;
	case PWM4:
		/* timer four is not capable of doing PWM */
		break;
	default:
		clk_disable(pwm->pclk);
		clk_put(pwm->pclk);
		return -1;
	}

	/* divider & scaler values */
	tcfg1 |= pwm->divider;
	__raw_writel(tcfg1, S3C2410_TCFG1);

	switch (pwm->timerid) {
	case PWM0:
	case PWM1:
		tcfg0 |= pwm->prescaler;
		__raw_writel(tcfg0, S3C2410_TCFG0);
		break;
	default:
		if ((tcfg0 | pwm->prescaler) != tcfg0) {
			printk(KERN_WARNING "not changing prescaler of PWM %u,"
			       " since it's shared with timer4 (clock tick)\n",
			       pwm->timerid);
		}
		break;
	}

	/* timer count and compare buffer initial values */
	tcnt = pwm->counter;
	tcmp = pwm->comparer;

	__raw_writel(tcnt, S3C2410_TCNTB(pwm->timerid));
	__raw_writel(tcmp, S3C2410_TCMPB(pwm->timerid));

	/* ensure timer is stopped */
	s3c2410_pwm_stop(pwm);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_enable);

int s3c2410_pwm_start(struct s3c2410_pwm *pwm)
{
	unsigned long tcon;

	tcon = __raw_readl(S3C2410_TCON);

	switch (pwm->timerid) {
	case PWM0:
		tcon |= S3C2410_TCON_T0START;
		tcon &= ~S3C2410_TCON_T0MANUALUPD;
		break;
	case PWM1:
		tcon |= S3C2410_TCON_T1START;
		tcon &= ~S3C2410_TCON_T1MANUALUPD;
		break;
	case PWM2:
		tcon |= S3C2410_TCON_T2START;
		tcon &= ~S3C2410_TCON_T2MANUALUPD;
		break;
	case PWM3:
		tcon |= S3C2410_TCON_T3START;
		tcon &= ~S3C2410_TCON_T3MANUALUPD;
		break;
	case PWM4:
		/* timer four is not capable of doing PWM */
	default:
		return -ENODEV;
	}

	__raw_writel(tcon, S3C2410_TCON);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_start);

int s3c2410_pwm_stop(struct s3c2410_pwm *pwm)
{
	unsigned long tcon;

	tcon = __raw_readl(S3C2410_TCON);

	switch (pwm->timerid) {
	case PWM0:
		tcon &= ~0x00000000;
		tcon |= S3C2410_TCON_T0RELOAD;
		tcon |= S3C2410_TCON_T0MANUALUPD;
		break;
	case PWM1:
		tcon &= ~0x00000080;
		tcon |= S3C2410_TCON_T1RELOAD;
		tcon |= S3C2410_TCON_T1MANUALUPD;
		break;
	case PWM2:
		tcon &= ~0x00000800;
		tcon |= S3C2410_TCON_T2RELOAD;
		tcon |= S3C2410_TCON_T2MANUALUPD;
		break;
	case PWM3:
		tcon &= ~0x00008000;
		tcon |= S3C2410_TCON_T3RELOAD;
		tcon |= S3C2410_TCON_T3MANUALUPD;
		break;
	case PWM4:
		/* timer four is not capable of doing PWM */
	default:
		return -ENODEV;
	}

	__raw_writel(tcon, S3C2410_TCON);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_stop);

int s3c2410_pwm_duty_cycle(int reg_value, struct s3c2410_pwm *pwm)
{
	__raw_writel(reg_value, S3C2410_TCMPB(pwm->timerid));

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_duty_cycle);

int s3c2410_pwm_dumpregs(void)
{
	printk(KERN_INFO "TCON: %08lx, TCFG0: %08lx, TCFG1: %08lx\n",
			(unsigned long)	__raw_readl(S3C2410_TCON),
			(unsigned long)	__raw_readl(S3C2410_TCFG0),
			(unsigned long)	__raw_readl(S3C2410_TCFG1));

	return 0;
}
EXPORT_SYMBOL_GPL(s3c2410_pwm_dumpregs);
