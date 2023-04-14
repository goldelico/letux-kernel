#include <ddr.h>
#include <pm_fastboot.h>



static struct store_regs *store_regs;


void sys_clk_restore(void)
{
	unsigned int tmp;
	struct pll_resume_reg *pll_resume_reg;
	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;

	pll_resume_reg = &store_regs->pll_resume_reg;

	tmp = *(volatile unsigned int *)0xb0000000;

	tmp &= ~(7 << 20);
	tmp |= (pll_resume_reg->cpccr & 0x000fffff);
	*(volatile unsigned int *)0xb0000000 = tmp;
	*(volatile unsigned int *)0xb0000000 |= (7 << 20);
	while ((*(volatile unsigned int *)0xb00000d4) & 0x7);

	tmp = *(volatile unsigned int *)0xb0000000;
	tmp &= ~(0xff << 24);
	tmp |= (pll_resume_reg->cpccr & 0xff000000);
	*(volatile unsigned int *)0xb0000000 = tmp;


	while (!((*(volatile unsigned int *)0xb00000d4 & 0xf0000000) == 0xf0000000));

}


static void pll_restore(void)
{
	struct pll_resume_reg *pll_resume_reg;
	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;

	pll_resume_reg = &store_regs->pll_resume_reg;

	*(volatile unsigned int *)0xb000000c = pll_resume_reg->cppcr;

	*(volatile unsigned int *)0xb0000010 = (pll_resume_reg->cpapcr & 0xfffffff0) | 1;
	while (!((*(volatile unsigned int *)0xb0000010 >> 3) & 1));

	*(volatile unsigned int *)0xb0000014 = (pll_resume_reg->cpmpcr & 0xfffffff0) | 1;
	while (!((*(volatile unsigned int *)0xb0000014 >> 3) & 1));

	*(volatile unsigned int *)0xb0000018 = (pll_resume_reg->cpepcr & 0xfffffff0) | 1;
	while (!((*(volatile unsigned int *)0xb0000018 >> 3) & 1));


	*(volatile unsigned int *)0xb000002c = (pll_resume_reg->ddrcdr & 0xc000000f);
	*(volatile unsigned int *)0xb000002c |= (1 << 29);
	while (((*(volatile unsigned int *)0xb000002c) >> 28) & 1);

	sys_clk_restore();
}



static void phy_pll_init(struct ddr_phy_resume_reg *ddr_phy_resume_reg)
{
	*(volatile unsigned int *)0xb3011084 |= (1 << 1);

	*(volatile unsigned int *)0xb3010080 &= ~0xff;
	*(volatile unsigned int *)0xb3011080 |= ddr_phy_resume_reg->pll_fbdiv;

	*(volatile unsigned int *)0xb3010088 &= ~0xff;
	*(volatile unsigned int *)0xb3011088 |= ddr_phy_resume_reg->pll_pdiv;

	*(volatile unsigned int *)0xb3010084 &= ~3;
	*(volatile unsigned int *)0xb3011084 |= (ddr_phy_resume_reg->pll_ctrl & 0x1);

	*(volatile unsigned int *)0xb3011084 &= ~(1 << 1);	//enable pll

	while(!(*(volatile unsigned int *)0xb30110c8 & (1 << 3)));	//wait pll lock
}

static void phy_cfg(struct ddr_phy_resume_reg *ddr_phy_resume_reg)
{
	*(volatile unsigned int *)0xb3011004 = ddr_phy_resume_reg->mem_cfg;
	*(volatile unsigned int *)0xb301107c = ddr_phy_resume_reg->dq_width;
	*(volatile unsigned int *)0xb3011014 = ddr_phy_resume_reg->cl;
	*(volatile unsigned int *)0xb3011018 = ddr_phy_resume_reg->al;
	*(volatile unsigned int *)0xb301101c = ddr_phy_resume_reg->cwl;
}



static void dfi_init(struct ddrc_resume_reg *ddrc_resume_reg)
{
	*(volatile unsigned int *)0xb3012000 &= ~(1 << 3);	//dfi_init_start = 0
	while(!(*(volatile unsigned int *)0xb3012004 & 0x1));	//wait dfi_init_compllete
	*(volatile unsigned int *)0xb34f0008 = ddrc_resume_reg->dcfg;

}


static void ddrc_init(struct ddrc_resume_reg *ddrc_resume_reg)
{
	*(volatile unsigned int *)0xb34f0040 = ddrc_resume_reg->dtimming1;
	*(volatile unsigned int *)0xb34f0048 = ddrc_resume_reg->dtimming2;
	*(volatile unsigned int *)0xb34f0050 = ddrc_resume_reg->dtimming3;
	*(volatile unsigned int *)0xb34f0058 = ddrc_resume_reg->dtimming4;
	*(volatile unsigned int *)0xb34f0060 = ddrc_resume_reg->dtimming5;
	*(volatile unsigned int *)0xb34f0078 = ddrc_resume_reg->dmmap0;
	*(volatile unsigned int *)0xb34f0080 = ddrc_resume_reg->dmmap1;
	*(volatile unsigned int *)0xb34f0038 = ddrc_resume_reg->drefcnt;
	*(volatile unsigned int *)0xb34f0010 = ddrc_resume_reg->dctrl;

	*(volatile unsigned int *)0xb3012008 = ddrc_resume_reg->dremap1;
	*(volatile unsigned int *)0xb301200c = ddrc_resume_reg->dremap2;
	*(volatile unsigned int *)0xb3012010 = ddrc_resume_reg->dremap3;
	*(volatile unsigned int *)0xb3012014 = ddrc_resume_reg->dremap4;
	*(volatile unsigned int *)0xb3012018 = ddrc_resume_reg->dremap5;

	*(volatile unsigned int *)0xb3012000 &= ~0x3;
	*(volatile unsigned int *)0xb3012000 |= ddrc_resume_reg->dwcfg & 0x3;


	*(volatile unsigned int *)0xb34f0088 = ddrc_resume_reg->dbwcfg;
	*(volatile unsigned int *)0xb34f0090 = ddrc_resume_reg->dbwstp;
	*(volatile unsigned int *)0xb34f00d8 = ddrc_resume_reg->hregpro;
	*(volatile unsigned int *)0xb34f00e0 = ddrc_resume_reg->dbgen;
	*(volatile unsigned int *)0xb301201c = ddrc_resume_reg->cpac;
	*(volatile unsigned int *)0xb3012020 = ddrc_resume_reg->cchc0;
	*(volatile unsigned int *)0xb3012024 = ddrc_resume_reg->cchc1;
	*(volatile unsigned int *)0xb3012028 = ddrc_resume_reg->cchc2;
	*(volatile unsigned int *)0xb301202c = ddrc_resume_reg->cchc3;
	*(volatile unsigned int *)0xb3012030 = ddrc_resume_reg->cchc4;
	*(volatile unsigned int *)0xb3012034 = ddrc_resume_reg->cchc5;
	*(volatile unsigned int *)0xb3012038 = ddrc_resume_reg->cchc6;
	*(volatile unsigned int *)0xb301203c = ddrc_resume_reg->cchc7;
	*(volatile unsigned int *)0xb3012040 = ddrc_resume_reg->cschc0;
	*(volatile unsigned int *)0xb3012044 = ddrc_resume_reg->cschc1;
	*(volatile unsigned int *)0xb3012048 = ddrc_resume_reg->cschc2;
	*(volatile unsigned int *)0xb301204c = ddrc_resume_reg->cschc3;
	*(volatile unsigned int *)0xb3012050 = ddrc_resume_reg->cmonc0;
	*(volatile unsigned int *)0xb3012054 = ddrc_resume_reg->cmonc1;
	*(volatile unsigned int *)0xb3012058 = ddrc_resume_reg->cmonc2;
	*(volatile unsigned int *)0xb301205c = ddrc_resume_reg->cmonc3;
	*(volatile unsigned int *)0xb3012060 = ddrc_resume_reg->cmonc4;
	*(volatile unsigned int *)0xb301206c = ddrc_resume_reg->pregpro;
	*(volatile unsigned int *)0xb3012070 = ddrc_resume_reg->bufcfg ;


}

static void ddr_lp_restore(struct ddrc_resume_reg *ddrc_resume_reg)
{
	*(volatile unsigned int *)0xb34f0020 = ddrc_resume_reg->ddlp;
	*(volatile unsigned int *)0xb34f0030 = ddrc_resume_reg->dasr_en;
	*(volatile unsigned int *)0xb34f0028 = ddrc_resume_reg->dasr_cnt;

}

static void ddrp_auto_calibration(void)
{
	unsigned int reg_val = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	unsigned int timeout = 0xffffff;
	unsigned int wait_cal_done = DDRP_CALIB_DONE_HDQCFA | DDRP_CALIB_DONE_LDQCFA;

	reg_val &= ~(DDRP_TRAINING_CTRL_DSCSE_BP);
	reg_val |= DDRP_TRAINING_CTRL_DSACE_START;
	ddr_writel(reg_val, DDRP_INNOPHY_TRAINING_CTRL);

	while(!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & wait_cal_done) == wait_cal_done) && --timeout) {
		DEBUG("t");
	}

	if(!timeout) {
		DEBUG("f");
	}
	ddr_writel(0, DDRP_INNOPHY_TRAINING_CTRL);

}

static void ddr_init(void)
{
	struct ddrc_resume_reg *ddrc_resume_reg;
	struct ddr_phy_resume_reg *ddr_phy_resume_reg;

	unsigned int ddrc_ctrl;
	volatile unsigned int tmp;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	ddrc_resume_reg = &store_regs->ddrc_resume_reg;
	ddr_phy_resume_reg = &store_regs->ddr_phy_resume_reg;


//	*(volatile unsigned int *)0xb34f0010 = (0x1 << 20);	//phy reset
	phy_cfg(ddr_phy_resume_reg);
//	*(volatile unsigned int *)0xb34f0010 &= ~(0x1 << 20);

	phy_pll_init(ddr_phy_resume_reg);
	dfi_init(ddrc_resume_reg);
	ddrc_init(ddrc_resume_reg);

	*(volatile unsigned int *)0xb34f0030 = 0;
	*(volatile unsigned int *)0xb34f0028 = 0;


	{
		ddrc_ctrl = *(volatile unsigned int *)0xb34f0010;

		/* bufferen_core = 1 */
		tmp = rtc_read_reg(0xb0003048);
		tmp |= (1 << 21);
		rtc_write_reg(0xb0003048, tmp);

		/* exit ddr self refresh */
		ddrc_ctrl &= ~(1<<5);
		ddrc_ctrl |= 1 << 1;
		*(volatile unsigned int*)0xb34f0010 = ddrc_ctrl;

		while((*(volatile unsigned int *)0xb34f0000) & (1<<2));
	}


	DEBUG("ddr restore finish\n");

	ddrp_auto_calibration();


	ddr_lp_restore(ddrc_resume_reg);


}



static unsigned int get_resume_pc(void)
{
	unsigned int resume_pc;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	resume_pc = store_regs->resume_pc;

	return resume_pc;
}

unsigned int get_uart_index(void)
{
	unsigned int uart_index;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	uart_index = store_regs->uart_index;

	return uart_index;
}




void fastboot_restore_code_2(void)
{
	unsigned int resume_pc;
	unsigned int uart_index;

	uart_index = get_uart_index();
#ifdef FASTBOOT_DEBUG
	serial_init(uart_index);
	DEBUG_HEX(uart_index);
#endif

	resume_pc = get_resume_pc();
	DEBUG_HEX(resume_pc);

	DEBUG("fastboot begin restore\n");

	pll_restore();
	DEBUG("pll restore ok\n");

	ddr_init();


	__asm__ volatile(
			".set push      \n\t"
			".set mips32r2  \n\t"
			"sync       \n\t"
			"jr.hb %0       \n\t"
			"nop            \n\t"
			".set pop       \n\t"
			:
			: "r" (resume_pc)
			:
			);

}


__attribute__((section(".resume"))) void fastboot_restore_code_1(void)
{

	__asm__ volatile(
			".set push      \n\t"
			".set noreorder      \n\t"
			".set mips32r2  \n\t"
			"move $29, %0   \n\t"
			"jr.hb   %1     \n\t"
			"nop            \n\t"
			".set reorder      \n\t"
			".set pop       \n\t"
			:
			:"r" (RTC_RAM_SP_ADDR), "r"(fastboot_restore_code_2)
			:
			);

}


