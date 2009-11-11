/*
 * linux/drivers/video/omap2/dss/dss.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DSS"

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/clk.h>

#include <plat/display.h>
#include "dss.h"

#ifndef CONFIG_ARCH_OMAP4
/* DSS */
#define DSS_BASE                        0x48050000
/* DISPLAY CONTROLLER */
#define DISPC_BASE                      0x48050400
#else
/* DSS */
#define DSS_BASE                        0x58000000
/* DISPLAY CONTROLLER */
#define DISPC_BASE                      0x58001000
#endif

#define DSS_SZ_REGS			SZ_512

struct dss_reg {
	u16 idx;
};

#define DSS_REG(idx)			((const struct dss_reg) { idx })

#define DSS_REVISION			DSS_REG(0x0000)
#define DSS_SYSCONFIG			DSS_REG(0x0010)
#define DSS_SYSSTATUS			DSS_REG(0x0014)
#define DSS_CONTROL			DSS_REG(0x0040)
#define DSS_SDI_CONTROL			DSS_REG(0x0044)
#define DSS_PLL_CONTROL			DSS_REG(0x0048)
#define DSS_SDI_STATUS			DSS_REG(0x005C)

#ifdef CONFIG_ARCH_OMAP4
#define DSS_STATUS				DSS_REG(0x005C)
#endif
void test(void);


#define REG_GET(idx, start, end) \
	FLD_GET(dss_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end) \
	dss_write_reg(idx, FLD_MOD(dss_read_reg(idx), val, start, end))

static struct {
	void __iomem    *base;

	struct clk	*dpll4_m4_ck;

	unsigned long	cache_req_pck;
	unsigned long	cache_prate;
	struct dss_clock_info cache_dss_cinfo;
	struct dispc_clock_info cache_dispc_cinfo;

	u32		ctx[DSS_SZ_REGS / sizeof(u32)];
} dss;

void __iomem  *dss_base;
void __iomem  *dispc_base;

static int _omap_dss_wait_reset(void);

static inline void dss_write_reg(const struct dss_reg idx, u32 val)
{
	__raw_writel(val, dss.base + idx.idx);
	__raw_readl(dss.base + 0x00); //sv5
}

static inline u32 dss_read_reg(const struct dss_reg idx)
{
	return __raw_readl(dss.base + idx.idx);
}

#define SR(reg) \
	dss.ctx[(DSS_##reg).idx / sizeof(u32)] = dss_read_reg(DSS_##reg)
#define RR(reg) \
	dss_write_reg(DSS_##reg, dss.ctx[(DSS_##reg).idx / sizeof(u32)])

void dss_save_context(void)
{
	if (cpu_is_omap24xx())
		return;

	SR(SYSCONFIG);
	SR(CONTROL);

#ifdef CONFIG_OMAP2_DSS_SDI
	SR(SDI_CONTROL);
	SR(PLL_CONTROL);
#endif
}

void dss_restore_context(void)
{
	if (_omap_dss_wait_reset())
		DSSERR("DSS not coming out of reset after sleep\n");

	RR(SYSCONFIG);
	RR(CONTROL);

#ifdef CONFIG_OMAP2_DSS_SDI
	RR(SDI_CONTROL);
	RR(PLL_CONTROL);
#endif
}

#undef SR
#undef RR

void dss_sdi_init(u8 datapairs)
{
	u32 l;

	BUG_ON(datapairs > 3 || datapairs < 1);

	l = dss_read_reg(DSS_SDI_CONTROL);
	l = FLD_MOD(l, 0xf, 19, 15);		/* SDI_PDIV */
	l = FLD_MOD(l, datapairs-1, 3, 2);	/* SDI_PRSEL */
	l = FLD_MOD(l, 2, 1, 0);		/* SDI_BWSEL */
	dss_write_reg(DSS_SDI_CONTROL, l);

	l = dss_read_reg(DSS_PLL_CONTROL);
	l = FLD_MOD(l, 0x7, 25, 22);	/* SDI_PLL_FREQSEL */
	l = FLD_MOD(l, 0xb, 16, 11);	/* SDI_PLL_REGN */
	l = FLD_MOD(l, 0xb4, 10, 1);	/* SDI_PLL_REGM */
	dss_write_reg(DSS_PLL_CONTROL, l);
}

int dss_sdi_enable(void)
{
	unsigned long timeout;

	dispc_pck_free_enable(1);

	/* Reset SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 1, 18, 18); /* SDI_PLL_SYSRESET */
	udelay(1);	/* wait 2x PCLK */

	/* Lock SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 1, 28, 28); /* SDI_PLL_GOBIT */

	/* Waiting for PLL lock request to complete */
	timeout = jiffies + msecs_to_jiffies(500);
	while (dss_read_reg(DSS_SDI_STATUS) & (1 << 6)) {
		if (time_after_eq(jiffies, timeout)) {
			DSSERR("PLL lock request timed out\n");
			goto err1;
		}
	}

	/* Clearing PLL_GO bit */
	REG_FLD_MOD(DSS_PLL_CONTROL, 0, 28, 28);

	/* Waiting for PLL to lock */
	timeout = jiffies + msecs_to_jiffies(500);
	while (!(dss_read_reg(DSS_SDI_STATUS) & (1 << 5))) {
		if (time_after_eq(jiffies, timeout)) {
			DSSERR("PLL lock timed out\n");
			goto err1;
		}
	}

	dispc_lcd_enable_signal(1);

	/* Waiting for SDI reset to complete */
	timeout = jiffies + msecs_to_jiffies(500);
	while (!(dss_read_reg(DSS_SDI_STATUS) & (1 << 2))) {
		if (time_after_eq(jiffies, timeout)) {
			DSSERR("SDI reset timed out\n");
			goto err2;
		}
	}

	return 0;

 err2:
	dispc_lcd_enable_signal(0);
 err1:
	/* Reset SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 0, 18, 18); /* SDI_PLL_SYSRESET */

	dispc_pck_free_enable(0);

	return -ETIMEDOUT;
}

void dss_sdi_disable(void)
{
	dispc_lcd_enable_signal(0);

	dispc_pck_free_enable(0);

	/* Reset SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 0, 18, 18); /* SDI_PLL_SYSRESET */
}

void dss_dump_clocks(struct seq_file *s)
{
	unsigned long dpll4_ck_rate;
	unsigned long dpll4_m4_ck_rate;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dpll4_ck_rate = clk_get_rate(clk_get_parent(dss.dpll4_m4_ck));
	dpll4_m4_ck_rate = clk_get_rate(dss.dpll4_m4_ck);

	seq_printf(s, "- DSS -\n");

	seq_printf(s, "dpll4_ck %lu\n", dpll4_ck_rate);

	seq_printf(s, "dss1_alwon_fclk = %lu / %lu * 2 = %lu\n",
			dpll4_ck_rate,
			dpll4_ck_rate / dpll4_m4_ck_rate,
			dss_clk_get_rate(DSS_CLK_FCK1));

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
}

void dss_dump_regs(struct seq_file *s)
{
#define DUMPREG(r) seq_printf(s, "%-35s %08x\n", #r, dss_read_reg(r))

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	DUMPREG(DSS_REVISION);
	DUMPREG(DSS_SYSCONFIG);
	DUMPREG(DSS_SYSSTATUS);
	DUMPREG(DSS_CONTROL);
	DUMPREG(DSS_SDI_CONTROL);
	DUMPREG(DSS_PLL_CONTROL);
	DUMPREG(DSS_SDI_STATUS);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
#undef DUMPREG
}

void dss_select_clk_source(bool dsi, bool dispc)
{
	u32 r;
	r = dss_read_reg(DSS_CONTROL);
	r = FLD_MOD(r, dsi, 1, 1);	/* DSI_CLK_SWITCH */
	r = FLD_MOD(r, dispc, 0, 0);	/* DISPC_CLK_SWITCH */
	/* TODO: extend for LCD2 and HDMI */
	dss_write_reg(DSS_CONTROL, r);
}

int dss_get_dsi_clk_source(void)
{
	return FLD_GET(dss_read_reg(DSS_CONTROL), 1, 1);
}

int dss_get_dispc_clk_source(void)
{
	return FLD_GET(dss_read_reg(DSS_CONTROL), 0, 0);
}

/* calculate clock rates using dividers in cinfo */
int dss_calc_clock_rates(struct dss_clock_info *cinfo)
{
	unsigned long prate;

	if (cinfo->fck_div > 16 || cinfo->fck_div == 0)
		return -EINVAL;

	prate = clk_get_rate(clk_get_parent(dss.dpll4_m4_ck));

	cinfo->fck = prate / cinfo->fck_div;

	return 0;
}

int dss_set_clock_div(struct dss_clock_info *cinfo)
{
	unsigned long prate;
	int r;

	if (cpu_is_omap34xx()) {
		prate = clk_get_rate(clk_get_parent(dss.dpll4_m4_ck));
		DSSDBG("dpll4_m4 = %ld\n", prate);

		r = clk_set_rate(dss.dpll4_m4_ck, prate / cinfo->fck_div);
		if (r)
			return r;
	}

	DSSDBG("fck = %ld (%d)\n", cinfo->fck, cinfo->fck_div);

	return 0;
}

int dss_get_clock_div(struct dss_clock_info *cinfo)
{
	cinfo->fck = dss_clk_get_rate(DSS_CLK_FCK1);

	if (cpu_is_omap34xx()) {
		unsigned long prate;
		prate = clk_get_rate(clk_get_parent(dss.dpll4_m4_ck));
		cinfo->fck_div = prate / (cinfo->fck / 2);
	} else {
		cinfo->fck_div = 0;
	}

	return 0;
}

unsigned long dss_get_dpll4_rate(void)
{
	if (cpu_is_omap34xx())
		return clk_get_rate(clk_get_parent(dss.dpll4_m4_ck));
	else
		return 0;
}

int dss_calc_clock_div(bool is_tft, unsigned long req_pck,
		struct dss_clock_info *dss_cinfo,
		struct dispc_clock_info *dispc_cinfo)
{
	unsigned long prate;
	struct dss_clock_info best_dss;
	struct dispc_clock_info best_dispc;

	unsigned long fck;

	u16 fck_div;

	int match = 0;
	int min_fck_per_pck;

	prate = dss_get_dpll4_rate();

	fck = dss_clk_get_rate(DSS_CLK_FCK1);
	if (req_pck == dss.cache_req_pck &&
			((cpu_is_omap34xx() && prate == dss.cache_prate) ||
			 dss.cache_dss_cinfo.fck == fck)) {
		DSSDBG("dispc clock info found from cache.\n");
		*dss_cinfo = dss.cache_dss_cinfo;
		*dispc_cinfo = dss.cache_dispc_cinfo;
		return 0;
	}

	min_fck_per_pck = CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK;

	if (min_fck_per_pck &&
		req_pck * min_fck_per_pck > DISPC_MAX_FCK) {
		DSSERR("Requested pixel clock not possible with the current "
				"OMAP2_DSS_MIN_FCK_PER_PCK setting. Turning "
				"the constraint off.\n");
		min_fck_per_pck = 0;
	}

retry:
	memset(&best_dss, 0, sizeof(best_dss));
	memset(&best_dispc, 0, sizeof(best_dispc));

	if (cpu_is_omap24xx()) {
		struct dispc_clock_info cur_dispc;
		/* XXX can we change the clock on omap2? */
		fck = dss_clk_get_rate(DSS_CLK_FCK1);
		fck_div = 1;

		dispc_find_clk_divs(is_tft, req_pck, fck, &cur_dispc);
		match = 1;

		best_dss.fck = fck;
		best_dss.fck_div = fck_div;

		best_dispc = cur_dispc;

		goto found;
	} else if (cpu_is_omap34xx()) {
		for (fck_div = 16; fck_div > 0; --fck_div) {
			struct dispc_clock_info cur_dispc;

			fck = prate / fck_div * 2;

			if (fck > DISPC_MAX_FCK)
				continue;

			if (min_fck_per_pck &&
					fck < req_pck * min_fck_per_pck)
				continue;

			match = 1;

			dispc_find_clk_divs(is_tft, req_pck, fck, &cur_dispc);

			if (abs(cur_dispc.pck - req_pck) <
					abs(best_dispc.pck - req_pck)) {

				best_dss.fck = fck;
				best_dss.fck_div = fck_div;

				best_dispc = cur_dispc;

				if (cur_dispc.pck == req_pck)
					goto found;
			}
		}
	} else {
		BUG();
	}

found:
	if (!match) {
		if (min_fck_per_pck) {
			DSSERR("Could not find suitable clock settings.\n"
					"Turning FCK/PCK constraint off and"
					"trying again.\n");
			min_fck_per_pck = 0;
			goto retry;
		}

		DSSERR("Could not find suitable clock settings.\n");

		return -EINVAL;
	}

	if (dss_cinfo)
		*dss_cinfo = best_dss;
	if (dispc_cinfo)
		*dispc_cinfo = best_dispc;

	dss.cache_req_pck = req_pck;
	dss.cache_prate = prate;
	dss.cache_dss_cinfo = best_dss;
	dss.cache_dispc_cinfo = best_dispc;

	return 0;
}



static irqreturn_t dss_irq_handler_omap2(int irq, void *arg)
{
	dispc_irq_handler();

	return IRQ_HANDLED;
}

static irqreturn_t dss_irq_handler_omap3(int irq, void *arg)
{
	/* INT_24XX_DSS_IRQ is dedicated for DISPC interrupt request only */
	/* DSI1, DSI2 and HDMI to be handled in seperate handlers */
	dispc_irq_handler();

	return IRQ_HANDLED;
}

static int _omap_dss_wait_reset(void)
{
	unsigned timeout = 1000;

	while (REG_GET(DSS_SYSSTATUS, 0, 0) == 0) {
		udelay(1);
		if (!--timeout) {
			DSSERR("soft reset failed\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int _omap_dss_reset(void)
{
#if 0 //sv3
	/* Soft reset */
	REG_FLD_MOD(DSS_SYSCONFIG, 1, 1, 1);
	return _omap_dss_wait_reset();
#endif
}

void dss_set_venc_output(enum omap_dss_venc_type type)
{
	int l = 0;

	if (type == OMAP_DSS_VENC_TYPE_COMPOSITE)
		l = 0;
	else if (type == OMAP_DSS_VENC_TYPE_SVIDEO)
		l = 1;
	else
		BUG();

	/* venc out selection. 0 = comp, 1 = svideo */
	REG_FLD_MOD(DSS_CONTROL, l, 6, 6);
}

void dss_set_dac_pwrdn_bgz(bool enable)
{
	REG_FLD_MOD(DSS_CONTROL, enable, 5, 5);	/* DAC Power-Down Control */
}

void dss_switch_tv_hdmi(int hdmi)
{
	REG_FLD_MOD(DSS_CONTROL, hdmi, 15, 15);	/* 0x1 for HDMI, 0x0 TV */
}

int dss_init(bool skip_init)
{
	int r;
	u32 rev;
	u32 val;

	dss_base = dss.base = ioremap(DSS_BASE, DSS_SZ_REGS);
	if (!dss.base) {
		DSSERR("can't ioremap DSS\n");
		r = -ENOMEM;
		goto fail0;
	}
	test();

	if (!skip_init) {
		/* disable LCD and DIGIT output. This seems to fix the synclost
		 * problem that we get, if the bootloader starts the DSS and
		 * the kernel resets it */
		//omap_writel(omap_readl(0x48050440) & ~0x3, 0x48050440);
		omap_writel(omap_readl(0x48041040) & ~0x3, 0x48041040);


		/* We need to wait here a bit, otherwise we sometimes start to
		 * get synclost errors, and after that only power cycle will
		 * restore DSS functionality. I have no idea why this happens.
		 * And we have to wait _before_ resetting the DSS, but after
		 * enabling clocks.
		 */
		msleep(50);

		_omap_dss_reset();
	}

	/* autoidle */
	REG_FLD_MOD(DSS_SYSCONFIG, 1, 0, 0);

	/* Select DPLL */
	REG_FLD_MOD(DSS_CONTROL, 0, 0, 0);

#ifdef CONFIG_OMAP2_DSS_VENC
	REG_FLD_MOD(DSS_CONTROL, 1, 4, 4);	/* venc dac demen */
	REG_FLD_MOD(DSS_CONTROL, 1, 3, 3);	/* venc clock 4x enable */
	REG_FLD_MOD(DSS_CONTROL, 0, 2, 2);	/* venc clock mode = normal */
#endif
#ifndef CONFIG_ARCH_OMAP4
	r = request_irq(INT_24XX_DSS_IRQ,
			cpu_is_omap24xx()
			? dss_irq_handler_omap2
			: dss_irq_handler_omap3,
			0, "OMAP DSS", NULL);
#else
	r = request_irq(INT_44XX_DSS_IRQ,
			dss_irq_handler_omap3,
			0, "OMAP DSS", (void *)1);
#endif
	if (r < 0) {
		DSSERR("omap2 dss: request_irq failed\n");
		goto fail1;
	}

	if (cpu_is_omap34xx()) {
		dss.dpll4_m4_ck = clk_get(NULL, "dpll4_m4_ck");
		if (IS_ERR(dss.dpll4_m4_ck)) {
			DSSERR("Failed to get dpll4_m4_ck\n");
			r = PTR_ERR(dss.dpll4_m4_ck);
			goto fail2;
		}
	}

	dss_save_context();

	rev = dss_read_reg(DSS_REVISION);
	printk(KERN_INFO "OMAP DSS rev %d.%d\n",
			FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

#if 0
	 gpio_request(102,"gpio102");

	 gpio_direction_output(102, 1);
	 mdelay(10);
	 gpio_set_value(102,0 );
	 mdelay(10);
	 gpio_set_value(102,1 );
	 mdelay(10);
#endif
#if 0
omap_set_gpio_dataout(78, 0);

printk(KERN_INFO "\n M-1-M");

// GPIO clear_data_out

temp = __raw_readl(0xd9052090);

temp = temp | (1<<14);

__raw_writel(temp, 0xd9052090);

printk(KERN_INFO "\n M-2-M");

mdelay(10);

printk(KERN_INFO "\n M-3-M");

omap_set_gpio_dataout(78, 1);

printk(KERN_INFO "\n M-4-M");

// GPIO set_data_out

temp = __raw_readl(0xd9052094);

printk(KERN_INFO "\n M-5-M");

temp = temp | (1<<14);

__raw_writel(temp, 0xd9052094);

printk(KERN_INFO "\n M-6-M");

mdelay(1000);

printk(KERN_INFO "\n M-7-M");







temp = __raw_readl(0xd905203C);

printk(KERN_INFO "\n AFTER GPIO_RESET 0x4905203C = 0x%X ", temp);





temp = __raw_readl(0xd9052094);

printk(KERN_INFO "\n AFTER GPIO_CLEAR (14)  0x49052094 = 0x%X ", temp);


#endif

#if 0 //sv3
// MJ Powerup-Powerdown sequence

	printk(KERN_INFO "\n ==================PowerUpPowerDown Seq============================= \n");

	// PowerDown PowerUp sequence from Symbian
 	//Request PRCM clocks

	val = omap_readl(0x4A307100);
	omap_writel(val, 0x4A307100);
	udelay(10);
	val = val |0x3; // ON state
	omap_writel(val, 0x4A307100);


	// CM_DSS_DSS_CLKCTRL

	val = omap_readl(0x4A009120); //CM_FCLKEN_DSS1,DSS2,TV
	val = val | 0xE00;// 0x7; // ON
	omap_writel(val, 0x4A009120);
	udelay(10);
	printk(KERN_INFO "\n CM_FCLKEN_DSS = 0x%X ", omap_readl(0x4A009120));

	printk(KERN_INFO "\n CM_SYS_CLKSEL = 0x4A306110 == 0x%X ", omap_readl(0x4A306110));

	//CM_CLKMODE_DPLL_CORE
	// DPLL4 locking
	val = omap_readl(0x4A004120);
	val = (val >>16) & 0x7; //need  to check the bits.
	if(val != 7) {
	            printk(KERN_INFO "DPLL4 should be locked before DSS release");
	}
 
	val = omap_readl(0x4A004120);
	printk(KERN_INFO "\n CM_CLKMODE_DPLL_CORE = 0x%X ", val);
	printk(KERN_INFO "\n DSS_SYSCONFIG = 0x%X ", dss_read_reg(DSS_SYSCONFIG));
 
#if 0
	//Wait for DSS reset sequence
	dss_write_reg(DSS_SYSCONFIG, 0x2);  //DSS_SYSCONFIG - Softreset
	val = dss_read_reg(DSS_SYSCONFIG);

	while( (dss_read_reg(DSS_SYSCONFIG) & 0x2) != 0x0); //wait till reset happens
	printk(KERN_INFO "\n DSSSYSCONFIG Reset happened ");
#endif

	val = dss_read_reg(DSS_SYSSTATUS);
	while(val != 0x1) { 
            val = dss_read_reg(DSS_SYSSTATUS);
	            // val = omap_readl(0x58000014); // Wait till Reset is DONE
	            udelay(1);
	}

	printk(KERN_INFO "\n DSS SYSSTATUS RESET DONE ");

#if 1

	// CM_DSS_DSS_CLKCTRL
	val = omap_readl(0x4A009120);
	val = val & ~(0xE00); // OFF FCLK DSS1,DSS2 and TVOUT
	omap_writel(val, 0x4A009120);
	udelay(10);
	printk(KERN_INFO "\n CM_FCLKEN_DSS = 0x%X ", omap_readl(0x4A009120));
	

	// DSS_PWRST_CTRL
	val = omap_readl(0x4A307100);
	val = val & ~(0x3); // Retention state
	omap_writel(val, 0x4A307100);
	udelay(10);
	printk(KERN_INFO "\n DSS_PWRST_CTRL to retention = 0x%X ", omap_readl(0x4A307104));
	
	val = omap_readl(0x4A307100);
	val = val | 0x3; // ON state 
	omap_writel(val, 0x4A307100);
	udelay(10);
	printk(KERN_INFO "\n DSS_PWRST_CTRL to ON = 0x%X ", omap_readl(0x4A307104));
	
 	val = omap_readl(0x4A009120);
	val = val | 0xE00; // ON FCLK DSS1,DSS2 and TVOUT
	omap_writel(val, 0x4A009120);
	udelay(10);

	printk(KERN_INFO "\n ON FCLK DSS1,DSS2 and TVOUT = 0x%X ", omap_readl(0x4A009120));

#endif

#if 0	 
	//Soft reset bit to DSS_RESET Wait for DSS reset sequence
	dss_write_reg(DSS_SYSCONFIG, 0x2);  //DSS_SYSCONFIG - Softreset
	val = dss_read_reg(DSS_SYSCONFIG);
#endif
	while( (dss_read_reg(DSS_SYSSTATUS) & 0x1) != 0x0); //wait till reset happens
	printk(KERN_INFO "\n DSSSYSCONFIG Reset happened ");

 	dispc_base = ioremap(0x58001000, 0x1000);
	dss_write_reg(dispc_base+0x10, 0x2);  //DSS_SYSCONFIG - Softreset
	while( (__raw_readl(dispc_base+0x14) & 0x1) != 0x0); //wait till DISPCreset happens
	            printk(KERN_INFO "\n DISPC SYSCONFIG Reset happened ");

	printk(KERN_INFO "\n ----------------------------------------------------------- \n");
	


#endif

	return 0;

fail2:
	free_irq(INT_24XX_DSS_IRQ, NULL);
fail1:
	iounmap(dss.base);
fail0:
	return r;
}

void dss_exit(void)
{
	if (cpu_is_omap34xx())
		clk_put(dss.dpll4_m4_ck);
#ifndef CONFIG_ARCH_OMAP4
	free_irq(INT_24XX_DSS_IRQ, NULL);
#else
	free_irq(INT_44XX_DSS_IRQ, NULL);
#endif

	iounmap(dss.base);
}




void test(void)
{
	u32 a, b, c;
	//	a = ioremap(0x58000000, 0x60);
	b = ioremap(0x4A009100, 0x30);
	c = ioremap(0x4a307100, 0x10);

	//printk(KERN_INFO "dss status 0x%x 0x%x\n", __raw_readl(a+0x5c), (a+0x5c));
	printk(KERN_INFO "CM_DSS_CLKSTCTRL 0x%x 0x%x\n", __raw_readl(b), b);
	printk(KERN_INFO "CM_DSS_DSS_CLKCTRL 0x%x 0x%x\n", __raw_readl(b+0x20), (b+0x20));
	printk(KERN_INFO "PM DSS wrst 0x%x 0x%x\n", __raw_readl(c+0x4), (c+0x4));

}




