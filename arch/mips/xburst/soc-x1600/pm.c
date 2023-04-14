#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <asm/r4kcache.h>
#include "pm.h"

#define CORE_PD
#ifdef CORE_PD
#define X1600_IDLE_PD
#endif
#define MEMORY_PD

//#define PM_TEST


/*************************************************************************************/
#ifdef PM_TEST

#define GPIO_PB_BASE (0xb0010100)
#define GPIO_PC_BASE (0xb0010200)


unsigned int intc0_msk;
unsigned int intc1_msk;
unsigned int gpio_pb_int;
unsigned int gpio_pb_msk;
unsigned int gpio_pb_pat1;
unsigned int gpio_pb_pat0;
unsigned int gpio_pc_int;
unsigned int gpio_pc_msk;
unsigned int gpio_pc_pat1;
unsigned int gpio_pc_pat0;

unsigned int clk_gate0;
unsigned int clk_gate1;



static void enable_pc31_int_low(void)
{
	unsigned int ahb2_intc = 0xb0001000;

	printk("intc0 src, ahb2_intc + 0x00: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x00));
	printk("intc0 msk, ahb2_intc + 0x04: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x04));
	printk("intc0 pnd, ahb2_intc + 0x10: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x10));

	printk("intc1 src, ahb2_intc + 0x20: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x20));
	printk("intc1 msk, ahb2_intc + 0x24: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x24));
	printk("intc1 pnd, ahb2_intc + 0x30: %08x\n", *(volatile unsigned int *)(ahb2_intc + 0x30));



	intc0_msk = *(volatile unsigned int *)0xb0001004;
	*(volatile unsigned int *)0xb0001004 = intc0_msk & ~(1<<15); // GPIO2 INT MSK
	gpio_pc_int = *(volatile unsigned int *) (GPIO_PC_BASE + 0x10);
	gpio_pc_msk = *(volatile unsigned int *) (GPIO_PC_BASE + 0x20);
	gpio_pc_pat1 = *(volatile unsigned int *)(GPIO_PC_BASE + 0x30);
	gpio_pc_pat0 = *(volatile unsigned int *)(GPIO_PC_BASE + 0x40);


	*(volatile unsigned int *)(GPIO_PC_BASE + 0x10) = gpio_pc_int | (1<<31);
	*(volatile unsigned int *)(GPIO_PC_BASE + 0x20) = gpio_pc_msk & ~(1<<31);
	*(volatile unsigned int *)(GPIO_PC_BASE + 0x30) = gpio_pc_pat1 & ~(1<<31);
	*(volatile unsigned int *)(GPIO_PC_BASE + 0x40) = gpio_pc_pat0 & ~(1<<31); // low level trigger int
	//*(volatile unsigned int *)(GPIO_PB_BASE + 0x40) = gpio_pb_pat0 | ~(1<<23); // high level trigger int

}




static void enable_rtc_wakeup(void)
{
	unsigned int tmp;

	intc1_msk = *(volatile unsigned int *)0xb0001024;
	*(volatile unsigned int *)0xb0001024 = intc1_msk & ~(1<<0); // RTC INT MSK
	tmp = rtc_read_reg(0xb0003048);
	tmp |= (1 << 18);
	rtc_write_reg(0xb0003048, tmp);

	tmp = rtc_read_reg(0xb000302c);
	tmp |= 1;
	rtc_write_reg(0xb000302c, tmp);
}


static void pm_test_config(void)
{

	//clk gate
	clk_gate0 = cpm_inl(CPM_CLKGR);
	clk_gate1 = cpm_inl(CPM_CLKGR1);

	clk_gate0 = 0xffffffff;
	clk_gate0 &= ~(1 << 31);	//ddr
	clk_gate0 &= ~(1 << 29);	//ahb0
	clk_gate0 &= ~(1 << 28);	//apb0
	clk_gate0 &= ~(1 << 27);	//rtc
	clk_gate0 &= ~(1 << 21);	//pdma
	clk_gate0 &= ~(1 << 20);	//ost
	clk_gate0 &= ~(1 << 16);	//uart2

	clk_gate1 = 0xffffffff;
	clk_gate1 &= ~(1 << 30);	//arb
	clk_gate1 &= ~(1 << 26);	//intc

	cpm_outl(clk_gate0, CPM_CLKGR);
	cpm_outl(clk_gate1, CPM_CLKGR1);

	// wakeup source
	enable_pc31_int_low();

	enable_rtc_wakeup();


}


#endif	//PM_TEST
/*************************************************************************************/

static void memory_pd_enter(void)
{
#ifdef MEMORY_PD
	unsigned int val;
	val = 0xffffffff;
	val &= ~(1 << 22);	//msc1
	val &= ~(1 << 19);	//pdma_slp
	val &= ~(1 << 18);	//pdma_sec
	val &= ~(1 << 17);	//pdma
	val &= ~(1 << 2);	//uart2
	*(volatile unsigned int *)0xb00000f8 = val;
	val = 0xfffffff0;
	*(volatile unsigned int *)0xb00000fc = val;
#endif
}

static void memory_pd_exit(void)
{
#ifdef MEMORY_PD
	*(volatile unsigned int *)0xb00000f8 = 0;
	*(volatile unsigned int *)0xb00000fc = 0;
#endif
}


#ifdef DEBUG_PM
static void x1600_pm_gate_check(void)
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



static void ddrp_auto_calibration(void)
{
	unsigned int reg;
	unsigned int timeout = 0xffffff;
	unsigned int al, ah;

	reg = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	reg &= ~(DDRP_TRAINING_CTRL_DSCSE_BP);
	ddr_writel(reg, DDRP_INNOPHY_TRAINING_CTRL);

	reg = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	reg |= (DDRP_TRAINING_CTRL_DSACE_START);
	ddr_writel(reg, DDRP_INNOPHY_TRAINING_CTRL);

	while(!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & 0x13) == 3) && --timeout) {
		TCSM_PCHAR('t');
		serial_put_hex(ddr_readl(DDRP_INNOPHY_CALIB_DONE));
	}

	if(!timeout) {
		TCSM_PCHAR('f');
	}

	reg = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	reg &= ~(1);
	ddr_writel(reg, DDRP_INNOPHY_TRAINING_CTRL);

	al = ddr_readl(DDRP_INNOPHY_CALIB_DELAY_AL);
	ah = ddr_readl(DDRP_INNOPHY_CALIB_DELAY_AH);

	{
		unsigned int cycsel, tmp;
		unsigned int read_data0, read_data1;
		unsigned int c0, c1;
		unsigned int max;

		read_data0 = *(volatile unsigned int *)(0xb3011000 + (0x74 << 2));
		read_data1 = *(volatile unsigned int *)(0xb3011000 + (0x75 << 2));
		c0 = (read_data0 >> 4) & 0x7;
		c1 = (read_data1 >> 4) & 0x7;

		max = max(c0, c1);

		cycsel = max + 1;

		tmp = *(volatile unsigned int *)(0xb3011000 + (0xa << 2));
		tmp &= ~(7 << 1);
		tmp |= cycsel << 1;
		*(volatile unsigned int *)(0xb3011000 + (0xa << 2)) = tmp;


		tmp = *(volatile unsigned int *)(0xb3011000 + 0x4);
		tmp |= 1 << 6;
		*(volatile unsigned int *)(0xb3011000 + (0x1 << 2)) = tmp;
	}

	TCSM_PCHAR('B');

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


#ifdef X1600_IDLE_PD
static int soc_pm_idle_pd(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	printk("lpm is idle pd\n");

	lcr &=~ 0x3; // low power mode: IDLE
	lcr |= 2;
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~ (1<<31);
	opcr |= (1 << 30);
	opcr |= (1 << 26);	//l2c power down
	opcr |= 1 << 2; // select RTC clk
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}
#else
static int soc_pm_idle(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	printk("lpm is idle \n");

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
	opcr |= (1 << 21); // pdma ram clear sd.
	opcr &= ~(1 << 4); // exclk disable;
	opcr |= (1 << 2); // select RTC clk
	opcr |= (1 << 22);
	opcr |= (1 << 20);

#ifdef CORE_PD
	opcr |= (1 << 3); // power down CPU
	opcr |= (1 << 26); //L2C power down
	printk("pd core and l2c\n");
#else
	opcr &= ~(1 << 3); // no power down CPU
	opcr &= ~(1 << 26); //L2C not power down
	printk("not pd core and l2c\n");
#endif
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

#ifdef PM_TEST
	{
		cpm_outl(clk_gate0, CPM_CLKGR);
		cpm_outl(clk_gate1, CPM_CLKGR1);

		*(volatile unsigned int*)(GPIO_PB_BASE + 0x10) = gpio_pb_int;
		*(volatile unsigned int*)(GPIO_PB_BASE + 0x20) = gpio_pb_msk;
		*(volatile unsigned int *)(GPIO_PB_BASE + 0x30) = gpio_pb_pat1;
		*(volatile unsigned int *)(GPIO_PB_BASE + 0x40) = gpio_pb_pat0;

		*(volatile unsigned int*)(0xb0001004) = intc0_msk;

	}
#endif

	memory_pd_exit();

	return 0;
}

static noinline void cpu_resume_bootup(void)
{
	TCSM_PCHAR('X');

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
	unsigned int r11s6;
	unsigned int r16s6;

	TCSM_PCHAR('R');

	/* set reset entry */
	r11s6 = __read_32bit_c0_register($11, 6);
	__asm__ volatile("ehb\n\t");
	r16s6 = __read_32bit_c0_register($16, 6);
	__asm__ volatile("ehb\n\t");
	r11s6 |= (1 << 8);
	r16s6 &= ~0xfffff000;
	r16s6 |= 0xbfc00000 & 0xfffff000;
	__write_32bit_c0_register($11, 6, r11s6);
	__write_32bit_c0_register($16, 6, r16s6);

	TCSM_PCHAR('D');

	if (sleep_param->state == PM_SUSPEND_STANDBY) {
		/* ddr auto clock gating */
		*(volatile unsigned int *)0xb3012068 &= ~(1 << 31);
	}

	if (sleep_param->state == PM_SUSPEND_MEM) {
		int tmp;

		/* enable pll */
		tmp = reg_ddr_phy(0x21);
		tmp &= ~(1 << 1);
		reg_ddr_phy(0x21) = tmp;

		while (!(reg_ddr_phy(0x42) & 0x8))
			serial_put_hex(reg_ddr_phy(0x42));

		/* dfi_init_start = 0, wait dfi_init_complete */
		*(volatile unsigned int *)0xb301208c &= ~1;
		while(!(*(volatile unsigned int *)0xb301208c & (0x1 << 1)));

		/* bufferen_core = 1 */
		tmp = *(volatile unsigned int *)0xb3012034;
		tmp &= ~(1 << 2);	//high
		*(volatile unsigned int *)0xb3012034 = tmp;

		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl &= ~(1<<5);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		while(ddr_readl(DDRC_STATUS) & (1<<2));

		TCSM_DELAY(1000);
		TCSM_PCHAR('1');
		ddrp_auto_calibration();
		TCSM_PCHAR('2');

		/* restore ddr auto-sr */
		*(volatile unsigned int *)0xb3011304 = sleep_param->autorefresh;
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

	TCSM_PCHAR('C');

	sleep_param->cpu_div = cpm_inl(CPM_CPCCR) & 0xf;

	blast_dcache32();
	blast_scache32();
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


	if (sleep_param->state == PM_SUSPEND_STANDBY) {
		/* ddr auto clock gating */
		*(volatile unsigned int *)0xb000002c |= (1 << 26);
		*(volatile unsigned int *)0xb3012068 |= (1 << 31) | (1 << 28);
	}

	if (sleep_param->state == PM_SUSPEND_MEM) {
		unsigned int tmp;
		unsigned int ddrc_ctrl;

		ddrc_ctrl = ddr_readl(DDRC_CTRL);

		/* save ddr low power state */
		sleep_param->pdt = ddrc_ctrl & DDRC_CTRL_PDT_MASK;
		sleep_param->dpd = ddrc_ctrl & DDRC_CTRL_DPD;
		sleep_param->dlp = ddr_readl(DDRC_DLP);
		sleep_param->autorefresh = *(volatile unsigned int *)0xb3011304;

		/* ddr disable deep power down */
		ddrc_ctrl &= ~(DDRC_CTRL_PDT_MASK);
		ddrc_ctrl &= ~(DDRC_CTRL_DPD);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		/* ddr diasble LPEN*/
		ddr_writel(0, DDRC_DLP);

		/* ddr disable auto-sr */
		*(volatile unsigned int *)0xb3011304 = 0;
		tmp = *(volatile unsigned int *)0xa0000000;


		/* DDR self refresh, */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= 1 << 5;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while(!(ddr_readl(DDRC_STATUS) & (1<<2)));


		/* bufferen_core = 0 */
		tmp = *(volatile unsigned int *)0xb3012034;
		tmp |= (1 << 2);	//low
		*(volatile unsigned int *)0xb3012034 = tmp;



		/* dfi_init_start = 1 */
		*(volatile unsigned int *)0xb301208c |= 1;

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


int x1600_pm_enter(suspend_state_t state)
{
	extern volatile u8 *uart_base;
	unsigned int r11s6;
	unsigned int r16s6;


	printk("x1600 pm enter!!\n");

	*(volatile unsigned int *)0xb0000020 &= ~(1 << 21);
	set_c0_status(0x100 << 2);

	sleep_param->uart_base = (unsigned int)uart_base;

	sleep_param->state = state;

#ifdef DEBUG_PM
	printk("sleep_param->state addr =  %d\n", sleep_param->state);
#endif

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_BOOTUP_TEXT, (unsigned int *)cpu_resume_bootup, SLEEP_CPU_RESUME_BOOTUP_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_TEXT, (unsigned int *)cpu_resume, SLEEP_CPU_RESUME_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	if (state == PM_SUSPEND_STANDBY) {
#ifdef X1600_IDLE_PD
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
	r11s6 = __read_32bit_c0_register($11, 6);
	__asm__ volatile("ehb\n\t");
	r16s6 = __read_32bit_c0_register($16, 6);
	__asm__ volatile("ehb\n\t");
	r11s6 |= (1 << 8);
	r16s6 &= ~0xfffff000;
	r16s6 |= SLEEP_CPU_RESUME_BOOTUP_TEXT & 0xfffff000;
	__write_32bit_c0_register($11, 6, r11s6);
	__write_32bit_c0_register($16, 6, r16s6);


	memory_pd_enter();

#ifdef PM_TEST
	pm_test_config();
#endif

#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
	x1600_pm_gate_check();
	printk("mem_pd0 = 0x%x\n", *(volatile unsigned int *)0xb00000f8);
	printk("mem_pd1 = 0x%x\n", *(volatile unsigned int *)0xb00000fc);
	printk("intc0 mask = 0x%x\n", *(volatile unsigned int *)0xb0001004);
	printk("intc1 mask = 0x%x\n", *(volatile unsigned int *)0xb0001024);
#endif

	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT);
	mb();

	soc_post_wakeup();

	return 0;
}

static int x1600_pm_begin(suspend_state_t state)
{
	printk("x1600 suspend begin\n");
	return 0;
}

static void x1600_pm_end(void)
{
	printk("x1600 pm end!\n");
}
static int x1600_suspend_prepare(void)
{
	printk("x1600 suspend prepare\n");

	return 0;
}
static void x1600_suspend_finish(void)
{
	printk("x1600 suspend finish");
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
static const struct platform_suspend_ops x1600_pm_ops = {
	.valid		= ingenic_pm_valid,
	.begin		= x1600_pm_begin,
	.enter		= x1600_pm_enter,
	.end		= x1600_pm_end,
	.prepare	= x1600_suspend_prepare,
	.finish		= x1600_suspend_finish,
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&x1600_pm_ops);


	return 0;
}

late_initcall(pm_init);
