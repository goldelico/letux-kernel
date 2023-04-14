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


#define DDR_DEEP_SLEEP

#ifdef DEBUG_PM
static void x2500_pm_gate_check(void)
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
	for (i = 0; i < 16; i++) {
		x = (gate1 >> i) & 1;
		if (x == 0)
			printk("warning : bit[%d] in clk gate1 is enabled\n", i);
	}

}
#endif



#ifdef DDR_DEEP_SLEEP
static void ddrp_auto_calibration(void)
{
	unsigned int timeout = 0xffffff;

	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0x74 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0x75 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0xa4 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0xa5 << 2)));

	timeout = 0xffffff;
	ddr_writel(0xa1, DDRP_INNOPHY_TRAINING_CTRL);

	while(!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & 0xf) == 0xf) && --timeout) {
		TCSM_PCHAR('t');
	}

	if(!timeout) {
		TCSM_PCHAR('f');
	}

	ddr_writel(0xa0, DDRP_INNOPHY_TRAINING_CTRL);

	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0x74 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0x75 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0xa4 << 2)));
	serial_put_hex(*(volatile unsigned int *)(0xb3011000 + (0xa5 << 2)));
#if 1
	{
		unsigned int cycsel, tmp;
		unsigned int read_data0, read_data1;
		unsigned int read_data2, read_data3;
		unsigned int c0, c1, c2, c3;
		unsigned int max;

		read_data0 = *(volatile unsigned int *)(0xb3011000 + (0x74 << 2));
		read_data1 = *(volatile unsigned int *)(0xb3011000 + (0x75 << 2));
		read_data2 = *(volatile unsigned int *)(0xb3011000 + (0xa4 << 2));
		read_data3 = *(volatile unsigned int *)(0xb3011000 + (0xa5 << 2));
		c0 = (read_data0 >> 4) & 0x7;
		c1 = (read_data1 >> 4) & 0x7;
		c2 = (read_data0 >> 4) & 0x7;
		c3 = (read_data1 >> 4) & 0x7;

		max = max(max(c0, c1), max(c2, c3));

		cycsel = max + 3;

		tmp = *(volatile unsigned int *)(0xb3011000 + (0xa << 2));
		tmp &= ~(7 << 1);
		tmp |= cycsel << 1;
		*(volatile unsigned int *)(0xb3011000 + (0xa << 2)) = tmp;


		tmp = *(volatile unsigned int *)(0xb3011000 + 0x4);
		tmp |= 1 << 6;
		*(volatile unsigned int *)(0xb3011000 + (0x1 << 2)) = tmp;
		TCSM_PCHAR('C');
	}
#endif
}

static void ddrp_hs_config(void)
{

	/* odt */
	reg_ddr_phy(0xb0) = 0xf;
	reg_ddr_phy(0xb1) = 0xf;
	reg_ddr_phy(0xb2) = 0x11;
	reg_ddr_phy(0xb3) = 0x11;

	reg_ddr_phy(0xc0) = 0x3;
	reg_ddr_phy(0xc1) = 0x3;
	reg_ddr_phy(0xd0) = 0x3;
	reg_ddr_phy(0xd1) = 0x3;
	reg_ddr_phy(0xe0) = 0x3;
	reg_ddr_phy(0xe1) = 0x3;
	reg_ddr_phy(0xf0) = 0x3;
	reg_ddr_phy(0xf1) = 0x3;

	reg_ddr_phy(0xc2) = 0x11;
	reg_ddr_phy(0xc3) = 0x11;
	reg_ddr_phy(0xd2) = 0x11;
	reg_ddr_phy(0xd3) = 0x11;
	reg_ddr_phy(0xe2) = 0x11;
	reg_ddr_phy(0xe3) = 0x11;
	reg_ddr_phy(0xf2) = 0x11;
	reg_ddr_phy(0xf3) = 0x11;

	/* dll */
	reg_ddr_phy(0x15) = 0xc;
	reg_ddr_phy(0x16) = 0x1;

	reg_ddr_phy(0x54) = 0xc;
	reg_ddr_phy(0x64) = 0xc;
	reg_ddr_phy(0x84) = 0xc;
	reg_ddr_phy(0x94) = 0xc;

	reg_ddr_phy(0x55) = 0x1;
	reg_ddr_phy(0x65) = 0x1;
	reg_ddr_phy(0x85) = 0x1;
	reg_ddr_phy(0x95) = 0x1;

	/* skew */
	reg_ddr_phy(0x100 + 0x0) = 0x8;
	reg_ddr_phy(0x100 + 0x1) = 0x8;
	reg_ddr_phy(0x100 + 0x2) = 0x8;
	reg_ddr_phy(0x100 + 0x3) = 0x8;
	reg_ddr_phy(0x100 + 0x4) = 0x8;
	reg_ddr_phy(0x100 + 0x5) = 0x8;
	reg_ddr_phy(0x100 + 0x6) = 0x8;
	reg_ddr_phy(0x100 + 0x7) = 0x8;
	reg_ddr_phy(0x100 + 0x8) = 0x8;
	reg_ddr_phy(0x100 + 0x9) = 0x8;
	reg_ddr_phy(0x100 + 0xa) = 0x8;
	reg_ddr_phy(0x100 + 0xb) = 0x8;
	reg_ddr_phy(0x100 + 0xc) = 0x8;
	reg_ddr_phy(0x100 + 0xd) = 0x8;
	reg_ddr_phy(0x100 + 0xe) = 0x8;
	reg_ddr_phy(0x100 + 0xf) = 0x8;
	reg_ddr_phy(0x100 + 0x10) = 0x8;
	reg_ddr_phy(0x100 + 0x11) = 0x8;
	reg_ddr_phy(0x100 + 0x12) = 0x8;
	reg_ddr_phy(0x100 + 0x13) = 0x8;
	reg_ddr_phy(0x100 + 0x14) = 0x8;
	reg_ddr_phy(0x100 + 0x15) = 0x8;
	reg_ddr_phy(0x100 + 0x16) = 0x8;
	reg_ddr_phy(0x100 + 0x17) = 0x8;
	reg_ddr_phy(0x100 + 0x18) = 0x8;
	reg_ddr_phy(0x100 + 0x19) = 0x8;
	reg_ddr_phy(0x100 + 0x1a) = 0x8;
	reg_ddr_phy(0x100 + 0x1b) = 0x8;
	reg_ddr_phy(0x100 + 0x1c) = 0x8;
	reg_ddr_phy(0x100 + 0x1d) = 0x8;
	reg_ddr_phy(0x100 + 0x1e) = 0x8;

	reg_ddr_phy(0x100 + 0x16) = 0x8;
	reg_ddr_phy(0x100 + 0x17) = 0x8;

	reg_ddr_phy(0x120 + 0x0) = 0x8;
	reg_ddr_phy(0x120 + 0x1) = 0x8;
	reg_ddr_phy(0x120 + 0x2) = 0x8;
	reg_ddr_phy(0x120 + 0x3) = 0x8;
	reg_ddr_phy(0x120 + 0x4) = 0x8;
	reg_ddr_phy(0x120 + 0x5) = 0x8;
	reg_ddr_phy(0x120 + 0x6) = 0x8;
	reg_ddr_phy(0x120 + 0x7) = 0x8;
	reg_ddr_phy(0x120 + 0x8) = 0x8;
	reg_ddr_phy(0x120 + 0xb) = 0x8;
	reg_ddr_phy(0x120 + 0xc) = 0x8;
	reg_ddr_phy(0x120 + 0xd) = 0x8;
	reg_ddr_phy(0x120 + 0xe) = 0x8;
	reg_ddr_phy(0x120 + 0xf) = 0x8;
	reg_ddr_phy(0x120 + 0x10) = 0x8;
	reg_ddr_phy(0x120 + 0x11) = 0x8;
	reg_ddr_phy(0x120 + 0x12) = 0x8;
	reg_ddr_phy(0x120 + 0x13) = 0x8;

	reg_ddr_phy(0x1a0 + 0x0) = 0x8;
	reg_ddr_phy(0x1a0 + 0x1) = 0x8;
	reg_ddr_phy(0x1a0 + 0x2) = 0x8;
	reg_ddr_phy(0x1a0 + 0x3) = 0x8;
	reg_ddr_phy(0x1a0 + 0x4) = 0x8;
	reg_ddr_phy(0x1a0 + 0x5) = 0x8;
	reg_ddr_phy(0x1a0 + 0x6) = 0x8;
	reg_ddr_phy(0x1a0 + 0x7) = 0x8;
	reg_ddr_phy(0x1a0 + 0x8) = 0x8;
	reg_ddr_phy(0x1a0 + 0xb) = 0x8;
	reg_ddr_phy(0x1a0 + 0xc) = 0x8;
	reg_ddr_phy(0x1a0 + 0xd) = 0x8;
	reg_ddr_phy(0x1a0 + 0xe) = 0x8;
	reg_ddr_phy(0x1a0 + 0xf) = 0x8;
	reg_ddr_phy(0x1a0 + 0x10) = 0x8;
	reg_ddr_phy(0x1a0 + 0x11) = 0x8;
	reg_ddr_phy(0x1a0 + 0x12) = 0x8;
	reg_ddr_phy(0x1a0 + 0x13) = 0x8;


	reg_ddr_phy(2) |= 0x8;
}

#endif



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

static int soc_pm_sleep(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3;
	lcr |= 1 << 0; // low power mode: SLEEP
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 31);
	opcr |= (1 << 30);
	opcr |= (1 << 4); // exclk enable;
	opcr &= ~(1 << 24);
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}

static int soc_post_wakeup(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	lcr &= ~0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	printk("post wakeup!\n");


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
#define reg_ddr_phy(x)   (*(volatile unsigned int *)(0xb3011000 + ((x) << 2)))
static noinline void cpu_resume(void)
{
	unsigned int ddrc_ctrl;

	TCSM_PCHAR('R');

	if (sleep_param->state == PM_SUSPEND_MEM) {
#ifdef DDR_DEEP_SLEEP
		int tmp;

		/* enable pll */
		tmp = reg_ddr_phy(0x21);
		tmp &= ~(1 << 1);
		reg_ddr_phy(0x21) = tmp;

		while (!(reg_ddr_phy(0x42) & 0x8))
			serial_put_hex(reg_ddr_phy(0x42));
#endif

		/* dfi_init_start = 0, wait dfi_init_complete */
		*(volatile unsigned int *)0xb3012000 &= ~(1 << 3);
		while(!(*(volatile unsigned int *)0xb3012004 & 0x1));

#ifdef DDR_DEEP_SLEEP
		/* bufferen_core = 1 */
		*(volatile unsigned int *)0xb3012000 &= ~(1 << 4);

#endif

		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl &= ~(1<<5);
		ddrc_ctrl |= (1<<1);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		while(ddr_readl(DDRC_STATUS) & (1<<2));
		while(REG32(DDRC_DWSTATUS) & (1<<4));

		TCSM_DELAY(1000);
		TCSM_PCHAR('1');
#ifdef DDR_DEEP_SLEEP
		ddrp_hs_config();
		ddrp_auto_calibration();
#endif
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



		/* DDR self refresh, */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= 1 << 5;
		ddrc_ctrl &= ~(1 << 1);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while(!(ddr_readl(DDRC_STATUS) & (1<<2)));
		while(!(REG32(DDRC_DWSTATUS) & (1<<4)));

#ifdef DDR_DEEP_SLEEP
		/* bufferen_core = 0 */
		*(volatile unsigned int *)0xb3012000 |= (1 << 4);
#endif

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

#ifdef DDR_DEEP_SLEEP
		/* disable pll */
		reg_ddr_phy(0x21) |= (1 << 1);
#endif
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


int x2500_pm_enter(suspend_state_t state)
{
	extern volatile u8 *uart_base;

	printk("x2500 pm enter!!\n");

	sleep_param->uart_base = (unsigned int)uart_base;

	sleep_param->state = state;

#ifdef DEBUG_PM
	printk("sleep_param->state addr =  %d\n", sleep_param->state);
#endif

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_BOOTUP_TEXT, (unsigned int *)cpu_resume_bootup, SLEEP_CPU_RESUME_BOOTUP_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_TEXT, (unsigned int *)cpu_resume, SLEEP_CPU_RESUME_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	if (state == PM_SUSPEND_STANDBY) {
		soc_pm_idle();
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
	x2500_pm_gate_check();
#endif

	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT);
	mb();

	soc_post_wakeup();

	return 0;
}

static int x2500_pm_begin(suspend_state_t state)
{
	printk("x2500 suspend begin\n");
	return 0;
}

static void x2500_pm_end(void)
{
	printk("x2500 pm end!\n");
}

static int x2500_suspend_prepare(void)
{
	printk("x2500 suspend prepare\n");
	return 0;
}

static void x2500_suspend_finish(void)
{
	printk("x2500 suspend finish\n");
}

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
static const struct platform_suspend_ops x2500_pm_ops = {
	.valid		= ingenic_pm_valid,
	.begin		= x2500_pm_begin,
	.enter		= x2500_pm_enter,
	.end		= x2500_pm_end,
	.prepare	= x2500_suspend_prepare,
	.finish		= x2500_suspend_finish,
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&x2500_pm_ops);


	return 0;
}

late_initcall(pm_init);
