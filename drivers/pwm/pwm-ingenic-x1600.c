// SPDX-License-Identifier: GPL-2.0
/*
 * PWM controller of the Ingenic X1600
 *
 * Copyright (c) 2026 Olivier Louvignes <olivier@mgcrea.io>
 *
 * Eight channels, each with a power-of-two clock divider and a waveform word
 * holding the counts at the high and at the low level; the channel's initial
 * level picks which comes first (X1600 programming manual, chapter 13). Unlike
 * the JZ47xx and X1000, whose PWMs are TCU timer channels (pwm-jz4740), the
 * X1600 has this separate controller.
 *
 * Limitations:
 * - The duty offset is 0 or period - duty (the high level last).
 * - Neither count may be zero, so a duty cycle of 0 or 100% switches the
 *   channel off and holds its output at the idle level instead.
 * - A disabled channel drives its idle level, low.
 * - Changing the divider or the initial level stops the channel first, which
 *   cuts the current period short.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/time.h>
#include <linux/types.h>

#define X1600_PWM_ENS			0x00	/* write 1: switch a channel on */
#define X1600_PWM_ENC			0x04	/* write 1: switch a channel off */
#define X1600_PWM_EN			0x08	/* the channels switched on */
#define X1600_PWM_UPT			0x10	/* write 1: load the waveform word */
#define X1600_PWM_BUSY			0x14	/* a load is pending */
#define X1600_PWM_MS			0x20	/* 0: waveform word, 1: DMA */
#define X1600_PWM_INL			0x24	/* the level a period starts with */
#define X1600_PWM_IDL			0x28	/* the level while off */
#define X1600_PWM_CCFG(n)		(0x40 + 4 * (n))
#define X1600_PWM_WCFG(n)		(0x80 + 4 * (n))

#define X1600_PWM_CCFG_PRESCALE		GENMASK(3, 0)	/* divide by 2^PRESCALE */
#define X1600_PWM_WCFG_HIGH		GENMASK(31, 16)
#define X1600_PWM_WCFG_LOW		GENMASK(15, 0)

#define X1600_PWM_NUM			8
#define X1600_PWM_MAX_COUNT		0xffff
#define X1600_PWM_MAX_PRESCALE		7

struct x1600_pwm {
	void __iomem *base;
	struct clk *clk;
};

struct x1600_pwm_wfhw {
	u32 wcfg;
	u8 prescale;
	bool enabled;
	bool inl;
	bool idl;
};

static u64 x1600_pwm_counts(u64 ns, unsigned long rate, unsigned int prescale)
{
	return mul_u64_u64_div_u64(ns, rate, (u64)NSEC_PER_SEC << prescale);
}

static u64 x1600_pwm_ns(u64 counts, unsigned long rate, unsigned int prescale)
{
	return DIV64_U64_ROUND_UP((counts * NSEC_PER_SEC) << prescale, rate);
}

static int x1600_pwm_round_waveform_tohw(struct pwm_chip *chip, struct pwm_device *pwm,
					 const struct pwm_waveform *wf, void *_wfhw)
{
	struct x1600_pwm *x = pwmchip_get_drvdata(chip);
	unsigned long rate = clk_get_rate(x->clk);
	struct x1600_pwm_wfhw *wfhw = _wfhw;
	u64 period, high, offset;
	unsigned int prescale;
	int ret = 0;

	*wfhw = (struct x1600_pwm_wfhw){ };
	if (!wf->period_length_ns)
		return 0;

	for (prescale = 0; prescale < X1600_PWM_MAX_PRESCALE; prescale++)
		if (x1600_pwm_counts(wf->period_length_ns, rate, prescale) <=
		    X1600_PWM_MAX_COUNT)
			break;

	period = min(x1600_pwm_counts(wf->period_length_ns, rate, prescale),
		     X1600_PWM_MAX_COUNT);
	if (period < 2) {
		period = 2;
		ret = 1;
	}
	high = min(x1600_pwm_counts(wf->duty_length_ns, rate, prescale), period);
	offset = x1600_pwm_counts(wf->duty_offset_ns, rate, prescale);

	wfhw->prescale = prescale;
	wfhw->wcfg = FIELD_PREP(X1600_PWM_WCFG_HIGH, high) |
		     FIELD_PREP(X1600_PWM_WCFG_LOW, period - high);
	wfhw->enabled = high && high < period;
	/* Low first only if the requested offset reaches period - duty. */
	wfhw->inl = !wfhw->enabled || offset < period - high;
	wfhw->idl = high == period;

	return ret;
}

static int x1600_pwm_round_waveform_fromhw(struct pwm_chip *chip, struct pwm_device *pwm,
					   const void *_wfhw, struct pwm_waveform *wf)
{
	struct x1600_pwm *x = pwmchip_get_drvdata(chip);
	unsigned long rate = clk_get_rate(x->clk);
	const struct x1600_pwm_wfhw *wfhw = _wfhw;
	u32 high = FIELD_GET(X1600_PWM_WCFG_HIGH, wfhw->wcfg);
	u32 low = FIELD_GET(X1600_PWM_WCFG_LOW, wfhw->wcfg);

	if (!rate)
		return -EINVAL;

	wf->period_length_ns = x1600_pwm_ns(high + low, rate, wfhw->prescale);
	wf->duty_length_ns = x1600_pwm_ns(high, rate, wfhw->prescale);
	wf->duty_offset_ns = wfhw->enabled && !wfhw->inl ?
			     x1600_pwm_ns(low, rate, wfhw->prescale) : 0;

	return 0;
}

static int x1600_pwm_read_waveform(struct pwm_chip *chip, struct pwm_device *pwm,
				   void *_wfhw)
{
	struct x1600_pwm *x = pwmchip_get_drvdata(chip);
	struct x1600_pwm_wfhw *wfhw = _wfhw;
	unsigned int ch = pwm->hwpwm;

	wfhw->enabled = readl(x->base + X1600_PWM_EN) & BIT(ch);
	wfhw->inl = readl(x->base + X1600_PWM_INL) & BIT(ch);
	wfhw->idl = readl(x->base + X1600_PWM_IDL) & BIT(ch);
	wfhw->prescale = FIELD_GET(X1600_PWM_CCFG_PRESCALE,
				   readl(x->base + X1600_PWM_CCFG(ch)));
	wfhw->wcfg = readl(x->base + X1600_PWM_WCFG(ch));

	return 0;
}

static void x1600_pwm_set_bit(struct x1600_pwm *x, unsigned int reg,
			      unsigned int ch, bool set)
{
	u32 val = readl(x->base + reg);

	if (set)
		val |= BIT(ch);
	else
		val &= ~BIT(ch);
	writel(val, x->base + reg);
}

static int x1600_pwm_write_waveform(struct pwm_chip *chip, struct pwm_device *pwm,
				    const void *_wfhw)
{
	struct x1600_pwm *x = pwmchip_get_drvdata(chip);
	const struct x1600_pwm_wfhw *wfhw = _wfhw;
	unsigned long rate = clk_get_rate(x->clk);
	struct x1600_pwm_wfhw cur;
	unsigned int ch = pwm->hwpwm;
	u32 busy;

	x1600_pwm_read_waveform(chip, pwm, &cur);

	/*
	 * Running with the same divider and initial level: load the new word,
	 * which the channel takes at the end of its period.
	 */
	if (cur.enabled && wfhw->enabled && cur.prescale == wfhw->prescale &&
	    cur.inl == wfhw->inl) {
		writel(wfhw->wcfg, x->base + X1600_PWM_WCFG(ch));
		writel(BIT(ch), x->base + X1600_PWM_UPT);
		return readl_poll_timeout(x->base + X1600_PWM_BUSY, busy,
					  !(busy & BIT(ch)), 10,
					  DIV_ROUND_UP_ULL(2ULL * USEC_PER_SEC *
							   (X1600_PWM_MAX_COUNT << cur.prescale),
							   rate));
	}

	/* The idle level first, so that stopping holds the right level. */
	x1600_pwm_set_bit(x, X1600_PWM_IDL, ch, wfhw->idl);
	if (cur.enabled)
		writel(BIT(ch), x->base + X1600_PWM_ENC);

	/* The prescaler only changes while the channel is off. */
	writel(FIELD_PREP(X1600_PWM_CCFG_PRESCALE, wfhw->prescale),
	       x->base + X1600_PWM_CCFG(ch));
	x1600_pwm_set_bit(x, X1600_PWM_INL, ch, wfhw->inl);
	writel(wfhw->wcfg, x->base + X1600_PWM_WCFG(ch));
	x1600_pwm_set_bit(x, X1600_PWM_MS, ch, false);

	if (wfhw->enabled)
		writel(BIT(ch), x->base + X1600_PWM_ENS);

	return 0;
}

static const struct pwm_ops x1600_pwm_ops = {
	.sizeof_wfhw = sizeof(struct x1600_pwm_wfhw),
	.round_waveform_tohw = x1600_pwm_round_waveform_tohw,
	.round_waveform_fromhw = x1600_pwm_round_waveform_fromhw,
	.read_waveform = x1600_pwm_read_waveform,
	.write_waveform = x1600_pwm_write_waveform,
};

static int x1600_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct x1600_pwm *x;

	chip = devm_pwmchip_alloc(dev, X1600_PWM_NUM, sizeof(*x));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	x = pwmchip_get_drvdata(chip);

	x->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(x->base))
		return PTR_ERR(x->base);

	x->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(x->clk))
		return dev_err_probe(dev, PTR_ERR(x->clk), "Unable to get clock\n");

	chip->ops = &x1600_pwm_ops;
	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id x1600_pwm_of_match[] = {
	{ .compatible = "ingenic,x1600-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, x1600_pwm_of_match);

static struct platform_driver x1600_pwm_driver = {
	.driver = {
		.name = "ingenic-x1600-pwm",
		.of_match_table = x1600_pwm_of_match,
	},
	.probe = x1600_pwm_probe,
};
module_platform_driver(x1600_pwm_driver);

MODULE_AUTHOR("Olivier Louvignes <olivier@mgcrea.io>");
MODULE_DESCRIPTION("Ingenic X1600 PWM driver");
MODULE_LICENSE("GPL");
