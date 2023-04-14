#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <ccu.h>

#include "pm_fastboot.h"


volatile unsigned int intc0_msk;
volatile unsigned int intc1_msk;
volatile unsigned int gpio_pb_int;
volatile unsigned int gpio_pb_msk;
volatile unsigned int gpio_pb_pat1;
volatile unsigned int gpio_pb_pat0;
volatile unsigned int gpio_pe_int;
volatile unsigned int gpio_pe_msk;
volatile unsigned int gpio_pe_pat1;
volatile unsigned int gpio_pe_pat0;

volatile unsigned int clk_gate0;
volatile unsigned int clk_gate1;

volatile unsigned int mem_pd0;
volatile unsigned int mem_pd1;

#define  g_state  ((volatile suspend_state_t *)0xb0000bfc)
volatile unsigned int _regs_stack_0[136 / 4];
volatile unsigned int _regs_stack_1[136 / 4];
volatile unsigned int *_regs_stack;

static inline void rtc_write_reg(unsigned int reg, unsigned int val)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );
	*(volatile unsigned int *)0xb000303c = 0xa55a;
	while (!((*(volatile unsigned int *)0xb000303c >> 31) & 0x1) );
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );

	*(volatile unsigned int *)reg = val;

	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );
}

static inline unsigned int  rtc_read_reg(unsigned int reg)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );

	return *(volatile unsigned int *)reg;
}

#define PRINT_DEBUG


#ifdef PRINT_DEBUG

#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)
#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

#define U_IOBASE (UART2_IOBASE + 0xa0000000)
#define TCSM_PCHAR(x)                                                   \
	*((volatile unsigned int*)(U_IOBASE+OFF_TDR)) = x;              \
while ((*((volatile unsigned int*)(U_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))
#else
#define TCSM_PCHAR(x)
#endif

#define TCSM_DELAY(x)                                           \
	do{                                                     \
		register unsigned int i = x;                    \
		while(i--)                                      \
		__asm__ volatile(".set mips32\n\t"      \
				"nop\n\t"              \
				".set mips32");        \
	}while(0)                                               \

static inline void serial_put_hex(unsigned int x) {
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		TCSM_PCHAR(d);
	}
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
}

#if 1
struct uart_regs {
	unsigned int udllr;
	unsigned int udlhr;
	unsigned int uthr;
	unsigned int uier;
	unsigned int ufcr;
	unsigned int ulcr;
	unsigned int umcr;
	unsigned int uspr;
	unsigned int isr;
	unsigned int umr;
	unsigned int uacr;
	unsigned int urcr;
	unsigned int utcr;

};


static void uart_save(struct uart_regs *uart_regs)
{
	printk("uart save\n");
	*(volatile unsigned int *)0xb003200c |= (1 << 7);
	uart_regs->udllr = *(volatile unsigned int *)0xb0032000;
	uart_regs->udlhr = *(volatile unsigned int *)0xb0032004;

	*(volatile unsigned int *)0xb003200c &= ~(1 << 7);
	uart_regs->uthr = *(volatile unsigned int  *)0xb0032000;
	uart_regs->uier = *(volatile unsigned int  *)0xb0032004;

	uart_regs->ufcr = *(volatile unsigned int  *)0xb0032008;
	uart_regs->ulcr = *(volatile unsigned int  *)0xb003200c;
	uart_regs->umcr = *(volatile unsigned int  *)0xb0032010;
	uart_regs->uspr = *(volatile unsigned int  *)0xb003201c;
	uart_regs->isr = *(volatile unsigned int  *)0xb0032020;
	uart_regs->umr = *(volatile unsigned int  *)0xb0032024;
	uart_regs->uacr = *(volatile unsigned int *)0xb0032028;
	uart_regs->urcr = *(volatile unsigned int *)0xb0032040;
	uart_regs->utcr = *(volatile unsigned int *)0xb0032044;

	printk("uthr   = 0x%08x\n", uart_regs->uthr);
	printk("uier   = 0x%08x\n", uart_regs->uier);
	printk("udllr   = 0x%08x\n", uart_regs->udllr);
	printk("udlhr   = 0x%08x\n", uart_regs->udlhr);
	printk("ULCR = 0x%08x\n", *(volatile unsigned int *)0xb003200c);
	printk("UMCR = 0x%08x\n", *(volatile unsigned int *)0xb0032010);
	printk("ULSR = 0x%08x\n", *(volatile unsigned int *)0xb0032014);
	printk("UMSR = 0x%08x\n", *(volatile unsigned int *)0xb0032018);
	printk("USPR = 0x%08x\n", *(volatile unsigned int *)0xb003201c);
	printk("ISR  = 0x%08x\n", *(volatile unsigned int *)0xb0032020);
	printk("UMR  = 0x%08x\n", *(volatile unsigned int *)0xb0032024);
	printk("UACR = 0x%08x\n", *(volatile unsigned int *)0xb0032028);
	printk("URCR = 0x%08x\n", *(volatile unsigned int *)0xb0032040);
	printk("UTCR = 0x%08x\n", *(volatile unsigned int *)0xb0032044);


}

static void uart_set_baud(struct uart_regs *uart_regs)
{
	*(volatile unsigned int *)0xb0032024 = uart_regs->umr;
	*(volatile unsigned int *)0xb0032028 = uart_regs->uacr;

	*(volatile unsigned int *)0xb003200c |= (1 << 7);
	*(volatile unsigned int *)0xb0032000 = uart_regs->udllr;
	*(volatile unsigned int *)0xb0032004 = uart_regs->udlhr;
	*(volatile unsigned int *)0xb003200c &= ~(1 << 7);

}


static void uart_restore(struct uart_regs *uart_regs)
{

	*(volatile unsigned int *)0xb0000020 &= ~(1 << 16);

	/*uart2 PD30 PD31 func0 */
	*(volatile unsigned int *)0xb0010318 |= (3 << 30);
	*(volatile unsigned int *)0xb0010328 |= (3 << 30);
	*(volatile unsigned int *)0xb0010338 |= (3 << 30);
	*(volatile unsigned int *)0xb0010348 |= (3 << 30);


	*(volatile unsigned int *)0xb0032008 = uart_regs->ufcr;
	*(volatile unsigned int *)0xb003200c = uart_regs->ulcr;
	*(volatile unsigned int *)0xb0032010 = uart_regs->umcr;
	*(volatile unsigned int *)0xb003201c = uart_regs->uspr;
	*(volatile unsigned int *)0xb0032020 = uart_regs->isr;
	*(volatile unsigned int *)0xb0032024 = uart_regs->umr;
	*(volatile unsigned int *)0xb0032028 = uart_regs->uacr;
	*(volatile unsigned int *)0xb0032040 = uart_regs->urcr;
	*(volatile unsigned int *)0xb0032044 = uart_regs->utcr;

	uart_set_baud(uart_regs);

	*(volatile unsigned int *)0xb0032008 = (1 << 4) | (1 << 0) | (1 << 1) | (1 << 2);

}
#endif

struct ost_regs {
	unsigned int ostccr;
	unsigned int oster;
	unsigned int ostcr;
	unsigned int ostfr;
	unsigned int ostmr;
	unsigned int ostdfr;
	unsigned int ostcnt;

	unsigned int g_ostccr;
	unsigned int g_oster;
	unsigned int g_ostcr;
	unsigned int g_ostcnth;
	unsigned int g_ostcntl;
	unsigned int g_ostcntb;
};

static void ost_save(struct ost_regs *ost_regs)
{
	ost_regs->ostccr = *(volatile unsigned int *)0xb2100000;
	ost_regs->oster = *(volatile unsigned int *)0xb2100004;
	ost_regs->ostcr = *(volatile unsigned int *)0xb2100008;
	ost_regs->ostfr = *(volatile unsigned int *)0xb210000c;
	ost_regs->ostmr = *(volatile unsigned int *)0xb2100010;
	ost_regs->ostdfr = *(volatile unsigned int *)0xb2100014;
	ost_regs->ostcnt = *(volatile unsigned int *)0xb2100018;

	ost_regs->g_ostccr = *(volatile unsigned int *)0xb2000000;
	ost_regs->g_oster = *(volatile unsigned int *)0xb2000004;
	ost_regs->g_ostcr = *(volatile unsigned int *)0xb2000008;
	ost_regs->g_ostcnth = *(volatile unsigned int *)0xb200000c;
	ost_regs->g_ostcntl = *(volatile unsigned int *)0xb2000010;
	ost_regs->g_ostcntb = *(volatile unsigned int *)0xb2000014;

}

static void ost_restore(struct ost_regs *ost_regs)
{
	*(volatile unsigned int *)0xb2100000 = ost_regs->ostccr;
	*(volatile unsigned int *)0xb2100008 = ost_regs->ostcr;
	*(volatile unsigned int *)0xb210000c = ost_regs->ostfr;
	*(volatile unsigned int *)0xb2100010 = ost_regs->ostmr;
	*(volatile unsigned int *)0xb2100014 = ost_regs->ostdfr;
	*(volatile unsigned int *)0xb2100018 = ost_regs->ostcnt;
	*(volatile unsigned int *)0xb2100004 = ost_regs->oster;

	*(volatile unsigned int *)0xb2000000 = ost_regs->g_ostccr;
	*(volatile unsigned int *)0xb2000008 = ost_regs->g_ostcr;
	*(volatile unsigned int *)0xb2000004 = ost_regs->g_oster;

}

static noinline void cpu_resume(void);


static void rtc_ram_write_enable(void)
{
	unsigned int tmp;

	/* write RTC RAM enable */
	tmp = rtc_read_reg(0xb0003048);
	tmp &= ~(1 << 23);
	tmp &= ~(1 << 22);
	tmp |= (1 << 21);
	tmp |= (1 << 20);
	rtc_write_reg(0xb0003048, tmp);

}

static void rtc_ram_write_disable(void)
{
	unsigned int tmp;

	/*exit write RTC RAM enable */
	tmp = rtc_read_reg(0xb0003048);
	tmp |= (1 << 22);
	rtc_write_reg(0xb0003048, tmp);

}


static void soc_pm_fastboot(void)
{

	unsigned int tmp;

	clk_gate0 = cpm_inl(CPM_CLKGR);
	clk_gate1 = cpm_inl(CPM_CLKGR1);

	clk_gate0 |= 1 << 20; // disable ost
	clk_gate0 &= ~(1 << 28); // enable apb0
	clk_gate0 &= ~(1 << 27); //enable rtc

	cpm_outl(clk_gate0, CPM_CLKGR);

	printk("clk_gate0: 0x%08x, clk_gate1: 0x%08x\n", clk_gate0, clk_gate1);



	intc1_msk = *(volatile unsigned int *)0xb0001024;
	*(volatile unsigned int *)0xb0001024 = intc1_msk & ~(1<<0); // RTC INT MSK

	/* RTC EALM */
	tmp = rtc_read_reg(0xb000302c);
	tmp |= 1;
	rtc_write_reg(0xb000302c, tmp);

	/* RTC enable AE */
	tmp = rtc_read_reg(0xb0003000);
	tmp |= (1 << 2) | (1 << 0);
	rtc_write_reg(0xb0003000, tmp);

	/* 32k rtc clk */
	tmp = rtc_read_reg(0xb0003000);
	tmp &= ~(1 << 1);
	rtc_write_reg(0xb0003000, tmp);
}

static int soc_post_wakeup(void)
{

	clk_gate0 &= ~(1 << 20);	//enable ost
	cpm_outl(clk_gate0, CPM_CLKGR);
	cpm_outl(clk_gate1, CPM_CLKGR1);


	return 0;
}





static struct store_regs *store_regs;

static void save_resume_pc(void)
{

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	store_regs->resume_pc = (unsigned int)cpu_resume;
	printk("xxxxxxxxxxxxxxxxxxx save resume_pc = 0x%08x\n", store_regs->resume_pc);
}
static void pll_store(void)
{
	struct pll_resume_reg *pll_resume_reg;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	pll_resume_reg = &store_regs->pll_resume_reg;

	pll_resume_reg->cpccr = *(volatile unsigned int *)0xb0000000;
	pll_resume_reg->cppcr = *(volatile unsigned int *)0xb000000c;
	pll_resume_reg->cpapcr = *(volatile unsigned int *)0xb0000010;
	pll_resume_reg->cpmpcr = *(volatile unsigned int *)0xb0000014;
	pll_resume_reg->cpepcr = *(volatile unsigned int *)0xb0000018;
	pll_resume_reg->ddrcdr = *(volatile unsigned int *)0xb000002c;

}

static void ddrc_store(void)
{
	struct ddrc_resume_reg *ddrc_resume_reg;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	ddrc_resume_reg = &store_regs->ddrc_resume_reg;

	ddrc_resume_reg->dcfg = *(volatile unsigned int *)0xb34f0008;
	ddrc_resume_reg->dctrl = *(volatile unsigned int *)0xb34f0010;
	ddrc_resume_reg->dlmr = *(volatile unsigned int *)0xb34f0018;
	ddrc_resume_reg->ddlp = *(volatile unsigned int *)0xb34f0020;
	ddrc_resume_reg->dasr_en = *(volatile unsigned int *)0xb34f0030;
	ddrc_resume_reg->dasr_cnt = *(volatile unsigned int *)0xb34f0028;
	ddrc_resume_reg->drefcnt = *(volatile unsigned int *)0xb34f0038;
	ddrc_resume_reg->dtimming1 = *(volatile unsigned int *)0xb34f0040;
	ddrc_resume_reg->dtimming2 = *(volatile unsigned int *)0xb34f0048;
	ddrc_resume_reg->dtimming3 = *(volatile unsigned int *)0xb34f0050;
	ddrc_resume_reg->dtimming4 = *(volatile unsigned int *)0xb34f0058;
	ddrc_resume_reg->dtimming5 = *(volatile unsigned int *)0xb34f0060;
	ddrc_resume_reg->dmmap0 = *(volatile unsigned int *)0xb34f0078;
	ddrc_resume_reg->dmmap1 = *(volatile unsigned int *)0xb34f0080;
	ddrc_resume_reg->dwcfg = *(volatile unsigned int *)0xb3012000;
	ddrc_resume_reg->dremap1 = *(volatile unsigned int *)0xb3012008;
	ddrc_resume_reg->dremap2 = *(volatile unsigned int *)0xb301200c;
	ddrc_resume_reg->dremap3 = *(volatile unsigned int *)0xb3012010;
	ddrc_resume_reg->dremap4 = *(volatile unsigned int *)0xb3012014;
	ddrc_resume_reg->dremap5 = *(volatile unsigned int *)0xb3012018;
	ddrc_resume_reg->ccguc0 = *(volatile unsigned int *)0xb3012064;
	ddrc_resume_reg->ccguc1 = *(volatile unsigned int *)0xb3012068;

}

static void ddr_phy_store(void)
{
	struct ddr_phy_resume_reg *ddr_phy_resume_reg;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	ddr_phy_resume_reg = &store_regs->ddr_phy_resume_reg;

	ddr_phy_resume_reg->mem_cfg = *(volatile unsigned int *)0xb3011004;
	ddr_phy_resume_reg->dq_width = *(volatile unsigned int *)0xb301107c;
	ddr_phy_resume_reg->cl = *(volatile unsigned int *)0xb3011014;
	ddr_phy_resume_reg->al = *(volatile unsigned int *)0xb3011018;
	ddr_phy_resume_reg->cwl = *(volatile unsigned int *)0xb301101c;
	ddr_phy_resume_reg->pll_fbdiv = *(volatile unsigned int *)0xb3011080;
	ddr_phy_resume_reg->pll_ctrl = *(volatile unsigned int *)0xb3011084;
	ddr_phy_resume_reg->pll_pdiv = *(volatile unsigned int *)0xb3011088;
	ddr_phy_resume_reg->training_ctrl = *(volatile unsigned int *)0xb3011008;
	ddr_phy_resume_reg->calib_bypass_al = *(volatile unsigned int *)0xb3011118;
	ddr_phy_resume_reg->calib_bypass_ah = *(volatile unsigned int *)0xb3011158;
	ddr_phy_resume_reg->wl_mode1 = *(volatile unsigned int *)0xb301100c;
	ddr_phy_resume_reg->wl_mode2 = *(volatile unsigned int *)0xb3011010;
}

static void rtc_ram_store(void)
{
	pll_store();
	ddrc_store();
	ddr_phy_store();
	save_resume_pc();
}


static int fastboot_resume_code[] = {

#include "fastboot_resume_code.hex"

};






extern long long save_goto(unsigned int);
extern int restore_goto(void);



static noinline void cpu_resume(void)
{

	unsigned int cpu_id;


	cpu_id = read_c0_ebase() & 0x3ff;

	if (cpu_id == 0) {
		_regs_stack = _regs_stack_0;
	} else if (cpu_id == 1) {
		_regs_stack = _regs_stack_1;
	}

	*(volatile unsigned int *)0xb2200f00 = 0xbfc00000; /* RESET entry = 0xbfc00000 ,reset value */

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




#define jump() do{                                  \
	                void (*func)(void);                                             \
	                func = (void(*)(void)) 0xb0004000;              \
	                func();                                                                 \
	        }while(0)

#if 0
static void ddrp_auto_calibration(void)
{
	unsigned int reg_val = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	unsigned int timeout = 0xffffff;
	unsigned int wait_cal_done = DDRP_CALIB_DONE_HDQCFA | DDRP_CALIB_DONE_LDQCFA;

	reg_val &= ~(DDRP_TRAINING_CTRL_DSCSE_BP);
	reg_val |= DDRP_TRAINING_CTRL_DSACE_START;
	ddr_writel(reg_val, DDRP_INNOPHY_TRAINING_CTRL);

	while(!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & wait_cal_done) == wait_cal_done) && --timeout) {
		TCSM_PCHAR('t');
		serial_put_hex(*(volatile unsigned int *)(0x80015060 + 0 * 4));
		serial_put_hex(*(volatile unsigned int *)(0x80015060 + 1 * 4));
		serial_put_hex(*(volatile unsigned int *)(0x80015060 + 2 * 4));
		serial_put_hex(*(volatile unsigned int *)(0x80015060 + 3 * 4));
	}

	if(!timeout) {
		TCSM_PCHAR('f');
	}
	ddr_writel(0, DDRP_INNOPHY_TRAINING_CTRL);
}
#endif


static noinline void cpu_sleep(void)
{


	unsigned int tmp;
	unsigned int ddrc_ctrl;

	blast_dcache32();
	blast_scache64();
	__sync();
	__fast_iob();

	ddr_writel(0, DDRC_AUTOSR_EN);
	tmp = *(volatile unsigned int *)0xa0000000;


	/* DDR self refresh, */
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
	/* ddr phy pll power down */
	*(volatile unsigned int *)0xb3011084 |= (1 << 1);



	TCSM_PCHAR('o');


	/* rtc disable power detect */
	rtc_write_reg(0xb000302c, (0x1a55a5a5 << 3) | 1);
	/* RTC PD */
	rtc_write_reg(0xb0003020, 1);


	while (1);
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


static int x2000_pm_enter(suspend_state_t state)
{
	unsigned int cpu_id;
	struct uart_regs *uart_regs;
	struct ost_regs *ost_regs;

	uart_regs = kmalloc(sizeof(struct uart_regs), GFP_KERNEL);
	ost_regs = kmalloc(sizeof(struct ost_regs), GFP_KERNEL);
	soc_pm_fastboot();

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	rtc_ram_write_enable();
	rtc_ram_store();
	memcpy((unsigned int *)RTC_RAM_CODE_ADDR1, (unsigned int *)fastboot_resume_code, RTC_RAM_CODE_SIZE);
	rtc_ram_write_disable();

	uart_save(uart_regs);
	ost_save(ost_regs);

	cpu_id = read_c0_ebase() & 0x3ff;

	printk("cpu_id = %d\n", cpu_id);

	if (cpu_id == 0) {
		_regs_stack = _regs_stack_0;
	} else if (cpu_id == 1) {
		_regs_stack = _regs_stack_1;
	}


	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT);
	mb();


	soc_post_wakeup();

	ost_restore(ost_regs);
	uart_restore(uart_regs);
	set_ccu_mimr(1 << 0);


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

static const struct platform_suspend_ops x2000_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.begin		= x2000_pm_begin,
	.enter		= x2000_pm_enter,
	.end		= x2000_pm_end,
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&x2000_pm_ops);

	return 0;
}

arch_initcall(pm_init);
