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

#include "pm.h"
#include "pm_fastboot.h"


static int fastboot_resume_code[] = {
#include "fastboot_resume_code.hex"
};

static noinline void fastboot_cpu_resume(void);
static noinline void fastboot_cpu_sleep(void);



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

static void cpm_save(struct cpm_regs *cpm_regs)
{
	cpm_regs->cpm_sftint = *(volatile unsigned int *)0xb00000bc;
	cpm_regs->cpsppr = *(volatile unsigned int *)0xb0000038;
	cpm_regs->cpspr = *(volatile unsigned int *)0xb0000034;
	cpm_regs->lcr = *(volatile unsigned int *)0xb0000004;
	cpm_regs->pswc0st = *(volatile unsigned int *)0xb0000090;
	cpm_regs->pswc1st = *(volatile unsigned int *)0xb0000094;
	cpm_regs->pswc2st = *(volatile unsigned int *)0xb0000098;
	cpm_regs->pswc3st = *(volatile unsigned int *)0xb000009c;
	cpm_regs->clkgr0 = *(volatile unsigned int*)0xb0000020;
	cpm_regs->clkgr1 = *(volatile unsigned int*)0xb0000028;
	cpm_regs->mestsel = *(volatile unsigned int*)0xb00000ec;
	cpm_regs->srbc = *(volatile unsigned int*)0xb00000c4;
	cpm_regs->exclk_ds = *(volatile unsigned int*)0xb00000e0;
	cpm_regs->memory_pd0 = *(volatile unsigned int*)0xb00000f8;
	cpm_regs->memory_pd1 = *(volatile unsigned int*)0xb00000fc;
	cpm_regs->slbc = *(volatile unsigned int*)0xb00000c8;
	cpm_regs->slpc = *(volatile unsigned int*)0xb00000cc;
	cpm_regs->opcr = *(volatile unsigned int*)0xb0000024;
	cpm_regs->rsr = *(volatile unsigned int*)0xb0000008;

}

static void cpm_restore(struct cpm_regs *cpm_regs)
{
	*(volatile unsigned int *)0xb0000020 = 0;
	*(volatile unsigned int *)0xb0000028 = 0;
	*(volatile unsigned int *)0xb00000bc = cpm_regs->cpm_sftint;
	*(volatile unsigned int *)0xb0000038 = cpm_regs->cpsppr;
	*(volatile unsigned int *)0xb0000034 = cpm_regs->cpspr;
	*(volatile unsigned int *)0xb0000004 = cpm_regs->lcr;
	*(volatile unsigned int *)0xb0000090 = cpm_regs->pswc0st;
	*(volatile unsigned int *)0xb0000094 = cpm_regs->pswc1st;
	*(volatile unsigned int *)0xb0000098 = cpm_regs->pswc2st;
	*(volatile unsigned int *)0xb000009c = cpm_regs->pswc3st;
	*(volatile unsigned int *)0xb00000ec = cpm_regs->mestsel;
	*(volatile unsigned int *)0xb00000c4 = cpm_regs->srbc;
	*(volatile unsigned int *)0xb00000e0 = cpm_regs->exclk_ds;
	*(volatile unsigned int *)0xb00000f8 = cpm_regs->memory_pd0;
	*(volatile unsigned int *)0xb00000fc = cpm_regs->memory_pd1;
	*(volatile unsigned int *)0xb00000c8 = cpm_regs->slbc;
	*(volatile unsigned int *)0xb00000cc = cpm_regs->slpc;
	*(volatile unsigned int *)0xb0000024 = cpm_regs->opcr;
	*(volatile unsigned int *)0xb0000008 = cpm_regs->rsr;
	*(volatile unsigned int *)0xb0000020 = cpm_regs->clkgr0;
	*(volatile unsigned int *)0xb0000028 = cpm_regs->clkgr1;

}

static void ccu_save(struct ccu_regs *ccu_regs)
{
	ccu_regs->mscr = *(volatile unsigned int *)0xb2200060;
	ccu_regs->pimr = *(volatile unsigned int *)0xb2200120;
	ccu_regs->mimr = *(volatile unsigned int *)0xb2200160;
	ccu_regs->oimr = *(volatile unsigned int *)0xb22001a0;
	ccu_regs->dipr = *(volatile unsigned int *)0xb22001c0;
	ccu_regs->gdimr = *(volatile unsigned int *)0xb22001e0;
	ccu_regs->ldimr0 = *(volatile unsigned int *)0xb2200300;
	ccu_regs->ldimr1 = *(volatile unsigned int *)0xb2200332;
	ccu_regs->rer = *(volatile unsigned int *)0xb2200f00;
	ccu_regs->mbr0 = *(volatile unsigned int *)0xb2201000;
	ccu_regs->mbr1 = *(volatile unsigned int *)0xb2201004;
	ccu_regs->cslr0 = *(volatile unsigned int *)0xb2200f10;
	ccu_regs->cslr1 = *(volatile unsigned int *)0xb2200f18;
	ccu_regs->csar0 = *(volatile unsigned int *)0xb2200f14;
	ccu_regs->csar1 = *(volatile unsigned int *)0xb2200f1c;

}

static void ccu_restore(struct ccu_regs *ccu_regs)
{
	*(volatile unsigned int *)0xb2200060 = ccu_regs->mscr;
	*(volatile unsigned int *)0xb2200120 = ccu_regs->pimr;
	*(volatile unsigned int *)0xb2200160 = ccu_regs->mimr;
	*(volatile unsigned int *)0xb22001a0 = ccu_regs->oimr;
	*(volatile unsigned int *)0xb22001c0 = ccu_regs->dipr;
	*(volatile unsigned int *)0xb22001e0 = ccu_regs->gdimr;
	*(volatile unsigned int *)0xb2200300 = ccu_regs->ldimr0;
	*(volatile unsigned int *)0xb2200332 = ccu_regs->ldimr1;
	*(volatile unsigned int *)0xb2200f00 = ccu_regs->rer;
	*(volatile unsigned int *)0xb2201000 = ccu_regs->mbr0;
	*(volatile unsigned int *)0xb2201004 = ccu_regs->mbr1;
	*(volatile unsigned int *)0xb2200f10 = ccu_regs->cslr0;
	*(volatile unsigned int *)0xb2200f18 = ccu_regs->cslr1;
	*(volatile unsigned int *)0xb2200f14 = ccu_regs->csar0;
	*(volatile unsigned int *)0xb2200f1c = ccu_regs->csar1;

}





struct uart_regs *uart_regs;
struct ost_regs *ost_regs;
struct cpm_regs *cpm_regs;
struct ccu_regs *ccu_regs;

void sys_save(void)
{
	uart_regs = kmalloc(sizeof(struct uart_regs), GFP_KERNEL);
	ost_regs = kmalloc(sizeof(struct ost_regs), GFP_KERNEL);
	cpm_regs = kmalloc(sizeof(struct cpm_regs), GFP_KERNEL);
	ccu_regs = kmalloc(sizeof(struct ccu_regs), GFP_KERNEL);

	ost_save(ost_regs);
	cpm_save(cpm_regs);
	ccu_save(ccu_regs);
}

void sys_restore(void)
{
	ost_restore(ost_regs);
	cpm_restore(cpm_regs);
	ccu_restore(ccu_regs);


	kfree(uart_regs);
	kfree(ost_regs);
}






static struct store_regs *store_regs;

static void save_resume_pc(void)
{

	store_regs = (struct store_regs *)FASTBOOT_DATA_ADDR;
	store_regs->resume_pc = (unsigned int)fastboot_cpu_resume;
	printk("xxxxxxxxxxxxxxxxxxx save resume_pc = 0x%08x\n", store_regs->resume_pc);
}

static void save_uart_index(void)
{
	store_regs = (struct store_regs *)FASTBOOT_DATA_ADDR;
	store_regs->uart_index = bc_idx;
	printk("xxxxxxxxxxxxxxxxxxx save uart_index = %d\n", store_regs->uart_index);
}

static void pll_store(void)
{
	struct pll_resume_reg *pll_resume_reg;

	store_regs = (struct store_regs *)FASTBOOT_DATA_ADDR;
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

	store_regs = (struct store_regs *)FASTBOOT_DATA_ADDR;
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

	ddrc_resume_reg->dbwcfg = *(volatile unsigned int *)0xb34f0088;
	ddrc_resume_reg->dbwstp = *(volatile unsigned int *)0xb34f0090;
	ddrc_resume_reg->hregpro = *(volatile unsigned int *)0xb34f00d8;
	ddrc_resume_reg->dbgen = *(volatile unsigned int *)0xb34f00e0;

	ddrc_resume_reg->dwcfg = *(volatile unsigned int *)0xb3012000;
	ddrc_resume_reg->dremap1 = *(volatile unsigned int *)0xb3012008;
	ddrc_resume_reg->dremap2 = *(volatile unsigned int *)0xb301200c;
	ddrc_resume_reg->dremap3 = *(volatile unsigned int *)0xb3012010;
	ddrc_resume_reg->dremap4 = *(volatile unsigned int *)0xb3012014;
	ddrc_resume_reg->dremap5 = *(volatile unsigned int *)0xb3012018;

	ddrc_resume_reg->cpac = *(volatile unsigned int *)0xb301201c;
	ddrc_resume_reg->cchc0 = *(volatile unsigned int *)0xb3012020;
	ddrc_resume_reg->cchc1 = *(volatile unsigned int *)0xb3012024;
	ddrc_resume_reg->cchc2 = *(volatile unsigned int *)0xb3012028;
	ddrc_resume_reg->cchc3 = *(volatile unsigned int *)0xb301202c;
	ddrc_resume_reg->cchc4 = *(volatile unsigned int *)0xb3012030;
	ddrc_resume_reg->cchc5 = *(volatile unsigned int *)0xb3012034;
	ddrc_resume_reg->cchc6 = *(volatile unsigned int *)0xb3012038;
	ddrc_resume_reg->cchc7 = *(volatile unsigned int *)0xb301203c;
	ddrc_resume_reg->cschc0 = *(volatile unsigned int *)0xb3012040;
	ddrc_resume_reg->cschc1 = *(volatile unsigned int *)0xb3012044;
	ddrc_resume_reg->cschc2 = *(volatile unsigned int *)0xb3012048;
	ddrc_resume_reg->cschc3 = *(volatile unsigned int *)0xb301204c;
	ddrc_resume_reg->cmonc0 = *(volatile unsigned int *)0xb3012050;
	ddrc_resume_reg->cmonc1 = *(volatile unsigned int *)0xb3012054;
	ddrc_resume_reg->cmonc2 = *(volatile unsigned int *)0xb3012058;
	ddrc_resume_reg->cmonc3 = *(volatile unsigned int *)0xb301205c;
	ddrc_resume_reg->cmonc4 = *(volatile unsigned int *)0xb3012060;

	ddrc_resume_reg->ccguc0 = *(volatile unsigned int *)0xb3012064;
	ddrc_resume_reg->ccguc1 = *(volatile unsigned int *)0xb3012068;

	ddrc_resume_reg->pregpro = *(volatile unsigned int *)0xb301206c;
	ddrc_resume_reg->bufcfg = *(volatile unsigned int *)0xb3012070;
}

static void ddr_phy_store(void)
{
	struct ddr_phy_resume_reg *ddr_phy_resume_reg;

	store_regs = (struct store_regs *)FASTBOOT_DATA_ADDR;
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
	save_uart_index();
}


void load_func_to_rtc_ram(void)
{

	load_func_to_tcsm((unsigned int *)FASTBOOT_SLEEP_CODE_ADDR, (unsigned int *)fastboot_cpu_sleep, FASTBOOT_SLEEP_CODE_LEN);

	rtc_ram_write_enable();
	memset((unsigned int *)RTC_MEMORY_START, 0, 4096);
	rtc_ram_store();
	memcpy((unsigned int *)FASTBOOT_RESUME_CODE1_ADDR, (unsigned int *)fastboot_resume_code, FASTBOOT_RESUME_CODE_LEN);
	rtc_ram_write_disable();
}





static noinline void fastboot_cpu_resume(void)
{


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


static noinline void fastboot_cpu_sleep(void)
{


	unsigned int tmp;
	unsigned int ddrc_ctrl;

	blast_dcache32();
	blast_scache64();
	__sync();
	__fast_iob();

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
	/* ddr phy pll power down */
	*(volatile unsigned int *)0xb3011084 |= (1 << 1);


	TCSM_PCHAR('W');


	/* rtc disable power detect */
	rtc_write_reg(0xb000302c, (0x1a55a5a5 << 3) | 1);
	/* RTC PD */
	rtc_write_reg(0xb0003020, 1);


	while (1);
}


void soc_pm_fastboot_config(void)
{

	unsigned int tmp;
	unsigned int intc1_msk;
	unsigned int clk_gate0;

	clk_gate0 = cpm_inl(CPM_CLKGR);

	clk_gate0 &= ~(1 << 27); //enable rtc

	cpm_outl(clk_gate0, CPM_CLKGR);


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

	rtc_write_reg(0xb0003028, 0);
	rtc_write_reg(0xb0003024, 0);
}



static int soc_pm_wakeup_fastboot_config(void)
{

	return 0;
}

void soc_pm_wakeup_fastboot(void)
{

	soc_pm_wakeup_fastboot_config();

	sys_restore();

}









