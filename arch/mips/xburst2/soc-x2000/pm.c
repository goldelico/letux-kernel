#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include "pm.h"
#if defined CONFIG_KP_AXP
#include <linux/regulator/machine.h>
#endif



//#define X2000_IDLE_PD


#ifdef DEBUG_PM
static void x2000_pm_gate_check(void)
{
	unsigned int gate0 = cpm_inl(CPM_CLKGR);
	unsigned int gate1 = cpm_inl(CPM_CLKGR1);
	int i;
	int x;

	printk("gate0 = 0x%08x\n", gate0);
	printk("gate1 = 0x%08x\n", gate1);
	for (i = 0; i < 32; i++) {
		x = (gate0 >> i) & 1;
		if (x == 0)
			printk("warning : bit[%d] in clk gate0 is enabled\n", i);
	}
	for (i = 0; i < 32; i++) {
		x = (gate1 >> i) & 1;
		if (x == 0)
			printk("warning : bit[%d] in clk gate1 is enabled\n", i);
	}

}
#endif




static inline void ddrp_auto_calibration(void)
{
	unsigned int reg_val = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	unsigned int timeout = 0xffffff;
	unsigned int wait_cal_done = DDRP_CALIB_DONE_HDQCFA | DDRP_CALIB_DONE_LDQCFA;

	reg_val &= ~(DDRP_TRAINING_CTRL_DSCSE_BP);
	reg_val |= DDRP_TRAINING_CTRL_DSACE_START;
	ddr_writel(reg_val, DDRP_INNOPHY_TRAINING_CTRL);

	while(!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & wait_cal_done) == wait_cal_done) && --timeout) {
		TCSM_PCHAR('t');
	}

	if(!timeout) {
		TCSM_PCHAR('f');
	}
	ddr_writel(0, DDRP_INNOPHY_TRAINING_CTRL);
}




static void load_func_to_tcsm(unsigned int *tcsm_addr,unsigned int *f_addr,unsigned int size)
{
	unsigned int instr;
	int offset;
	int i;
#ifdef DEBUG_PM
	printk("tcsm addr = %p %p size = %d\n",tcsm_addr,f_addr,size);
#endif
	for(i = 0;i < size / 4;i++) {
		instr = f_addr[i];
		if((instr >> 26) == 2){
			offset = instr & 0x3ffffff;
			offset = (offset << 2) - ((unsigned int)f_addr & 0xfffffff);
			if(offset > 0) {
				offset = ((unsigned int)tcsm_addr & 0xfffffff) + offset;
				instr = (2 << 26) | (offset >> 2);
			}
		}
		tcsm_addr[i] = instr;
	}
}


#ifdef X2000_IDLE_PD
static int soc_pm_idle_pd(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3; // low power mode: IDLE
	lcr |= 2;
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~ (1<<31);
	opcr |= (1 << 30);
	opcr &= ~(1 << 26);	//l2c power down
	opcr |= 1 << 2; // select RTC clk
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}
#else
static int soc_pm_idle(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 30);
	opcr &= ~(1 << 31);
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}
#endif

static int soc_pm_sleep(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3;
	lcr |= 1 << 0; // low power mode: SLEEP
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 31);
	opcr |= (1 << 30);
	opcr &= ~(1 << 26); //L2C power down
	opcr |= (1 << 21); // cpu 32k ram retention.
	opcr |= (1 << 3); // power down CPU
	opcr &= ~(1 << 4); // exclk disable;
	opcr |= (1 << 2); // select RTC clk
	opcr |= (1 << 22);
	opcr |= (1 << 20);
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}

static int soc_post_wakeup(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	lcr &= ~0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	printk("post wakeup!\n");


	{
		/* after power down cpu by set PD in OPCR, resume cpu's frequency and L2C's freq */
		unsigned int val;

		/* change disable */
		val = cpm_inl(CPM_CPCCR);
		val &= ~(1 << 22);
		cpm_outl(val, CPM_CPCCR);

		/* resume cpu_div in CPCCR */
		val &= ~0xf;
		val |= sleep_param->cpu_div;
		cpm_outl(val, CPM_CPCCR);

		/* change enable */
		val |= (1 << 22);
		cpm_outl(val, CPM_CPCCR);

		while (cpm_inl(CPM_CPCSR) & 1);
	}
	return 0;
}

static noinline void cpu_resume_bootup(void)
{
	TCSM_PCHAR('X');
	/* set reset entry */
	*(volatile unsigned int *)0xb2200f00 = 0xbfc00000;

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"move $29, %0	\n\t"
		"jr.hb   %1	\n\t"
		"nop		\n\t"
		".set pop	\n\t"
		:
		:"r" (SLEEP_CPU_RESUME_SP), "r"(SLEEP_CPU_RESUME_TEXT)
		:
		);

}
static noinline void cpu_resume(void)
{
	unsigned int ddrc_ctrl;

	TCSM_PCHAR('R');

	if (sleep_param->state == PM_SUSPEND_MEM) {
		int tmp;

		/* enable pll */
		tmp = reg_ddr_phy(0x21);
		tmp &= ~(1 << 1);
		reg_ddr_phy(0x21) = tmp;

		while (!(reg_ddr_phy(0x32) & 0x8))
			serial_put_hex(reg_ddr_phy(0x32));

		/* dfi_init_start = 0, wait dfi_init_complete */
		*(volatile unsigned int *)0xb3012000 &= ~(1 << 3);
		while(!(*(volatile unsigned int *)0xb3012004 & 0x1));

		/* bufferen_core = 1 */
		tmp = rtc_read_reg(0xb0003048);
		tmp |= (1 << 21);
		rtc_write_reg(0xb0003048, tmp);

		/* exit sr */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl &= ~(1<<5);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		while(ddr_readl(DDRC_STATUS) & (1<<2));

		TCSM_DELAY(1000);
		TCSM_PCHAR('1');
		ddrp_auto_calibration();
		TCSM_PCHAR('2');

		/* restore ddr auto-sr */
		ddr_writel(sleep_param->autorefresh, DDRC_AUTOSR_EN);
		TCSM_PCHAR('3');

		/* restore ddr LPEN */
		ddr_writel(sleep_param->dlp, DDRC_DLP);

		/* restore ddr deep power down state */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= sleep_param->pdt;
		ddrc_ctrl |= sleep_param->dpd;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		TCSM_PCHAR('4');
	}


	TCSM_PCHAR('5');

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"jr.hb %0	\n\t"
		"nop		\n\t"
		".set pop 	\n\t"
		:
		: "r" (restore_goto)
		:
		);
}


static noinline void cpu_sleep(void)
{


	sleep_param->cpu_div = cpm_inl(CPM_CPCCR) & 0xf;

	blast_dcache32();
	blast_scache64();
	__sync();
	__fast_iob();

	{
		/* before power down cpu by set PD in OPCR, reduce cpu's frequency as the same as L2C's freq */
		unsigned int val, div;

		/* change disable */
		val = cpm_inl(CPM_CPCCR);
		val &= ~(1 << 22);
		cpm_outl(val, CPM_CPCCR);

		/* div cpu = div l2c */
		div = val & (0xf << 4);
		val &= ~0xf;
		val |= (div >> 4);
		cpm_outl(val, CPM_CPCCR);

		/* change enable */
		val |= (1 << 22);
		cpm_outl(val, CPM_CPCCR);

		while (cpm_inl(CPM_CPCSR) & 1);
	}

	TCSM_PCHAR('D');

	if (sleep_param->state == PM_SUSPEND_MEM) {
		unsigned int tmp;
		unsigned int ddrc_ctrl;

		ddrc_ctrl = ddr_readl(DDRC_CTRL);

		/* save ddr low power state */
		sleep_param->pdt = ddrc_ctrl & DDRC_CTRL_PDT_MASK;
		sleep_param->dpd = ddrc_ctrl & DDRC_CTRL_DPD;
		sleep_param->dlp = ddr_readl(DDRC_DLP);
		sleep_param->autorefresh = ddr_readl(DDRC_AUTOSR_EN);

		/* ddr disable deep power down */
		ddrc_ctrl &= ~(DDRC_CTRL_PDT_MASK);
		ddrc_ctrl &= ~(DDRC_CTRL_DPD);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		/* ddr diasble LPEN*/
		ddr_writel(0, DDRC_DLP);

		/* ddr disable auto-sr */
		ddr_writel(0, DDRC_AUTOSR_EN);
		tmp = *(volatile unsigned int *)0xa0000000;

		/* DDR self refresh */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= 1 << 5;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while(!(ddr_readl(DDRC_STATUS) & (1<<2)));


		/* bufferen_core = 0 */
		tmp = rtc_read_reg(0xb0003048);
		tmp &= ~(1 << 21);
		rtc_write_reg(0xb0003048, tmp);


		/* dfi_init_start = 1 */
		*(volatile unsigned int *)0xb3012000 |= (1 << 3);

		{
			int i;
			for (i = 0; i < 4; i++) {

				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
			}
		}

		/* disable pll */
		reg_ddr_phy(0x21) |= (1 << 1);
	}

	TCSM_PCHAR('W');
	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"wait		\n\t"
		"nop		\n\t"
		"nop		\n\t"
		"nop		\n\t"
		".set pop	\n\t"
	);

	TCSM_PCHAR('N');

	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"jr.hb %0	\n\t"
		"nop		\n\t"
		".set pop 	\n\t"
		:
		: "r" (SLEEP_CPU_RESUME_BOOTUP_TEXT)
		:
		);
	TCSM_PCHAR('F');
}


int x2000_pm_enter(suspend_state_t state)
{
	extern volatile u8 *uart_base;

	printk("x2000 pm enter!!\n");

	sleep_param->uart_base = (unsigned int)uart_base;

	sleep_param->state = state;

#ifdef DEBUG_PM
	printk("sleep_param->state addr =  %d\n", sleep_param->state);
#endif

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_BOOTUP_TEXT, (unsigned int *)cpu_resume_bootup, SLEEP_CPU_RESUME_BOOTUP_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_TEXT, (unsigned int *)cpu_resume, SLEEP_CPU_RESUME_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	if (state == PM_SUSPEND_STANDBY) {
#ifdef X2000_IDLE_PD
		soc_pm_idle_pd();
#else
		soc_pm_idle();
#endif
	} else if (state == PM_SUSPEND_MEM) {
		soc_pm_sleep();
	} else {
		printk("WARNING : unsupport pm suspend state\n");
	}


	/* set reset entry */
	*(volatile unsigned int *)0xb2200f00 = SLEEP_CPU_RESUME_BOOTUP_TEXT;


#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
	x2000_pm_gate_check();
#endif

	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT);
	mb();

	soc_post_wakeup();

	return 0;
}

static int x2000_pm_begin(suspend_state_t state)
{
	printk("x2000 suspend begin\n");
	return 0;
}

static void x2000_pm_end(void)
{
	printk("x2000 pm end!\n");
}
#if defined CONFIG_KP_AXP
static int x2000_suspend_prepare(void)
{
	return regulator_suspend_prepare(PM_SUSPEND_MEM);
}
static void x2000_suspend_finish(void)
{
	if (regulator_suspend_finish())
		pr_err("%s: Suspend finish failed\n", __func__);
}
#endif

static int ingenic_pm_valid(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}
static const struct platform_suspend_ops x2000_pm_ops = {
	.valid		= ingenic_pm_valid,
	.begin		= x2000_pm_begin,
	.enter		= x2000_pm_enter,
	.end		= x2000_pm_end,
#if defined CONFIG_KP_AXP
	.prepare = x2000_suspend_prepare,
	.finish = x2000_suspend_finish,
#endif
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&x2000_pm_ops);


	return 0;
}

late_initcall(pm_init);
