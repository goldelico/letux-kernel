#include <ddr.h>
#include <pm_fastboot.h>



//#define FASTBOOT_DEBUG
#ifdef FASTBOOT_DEBUG

#include <uart.h>

#define DEBUG(s)        serial_puts(2, s)
#define DEBUG_HEX(a)    serial_put_hex(2, a)


#define	GPIO_PULSE()		\
	while (1) {		\
		*(volatile unsigned int *)0xb0010218 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010224 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010238 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010248 = (1 << 20);	\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		*(volatile unsigned int *)0xb0010218 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010224 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010238 = (1 << 20);	\
		*(volatile unsigned int *)0xb0010244 = (1 << 20);	\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
		__asm__ __volatile__("nop\n\t");			\
	}


#else
#define DEBUG(s)
#define DEBUG_HEX(a)
#endif


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
}

static void ddr_init(void)
{
	struct ddrc_resume_reg *ddrc_resume_reg;
	struct ddr_phy_resume_reg *ddr_phy_resume_reg;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	ddrc_resume_reg = &store_regs->ddrc_resume_reg;
	ddr_phy_resume_reg = &store_regs->ddr_phy_resume_reg;


//	*(volatile unsigned int *)0xb34f0010 = (0x1 << 20);	//phy reset
	phy_cfg(ddr_phy_resume_reg);
//	*(volatile unsigned int *)0xb34f0010 &= ~(0x1 << 20);

	phy_pll_init(ddr_phy_resume_reg);
	dfi_init(ddrc_resume_reg);
	ddrc_init(ddrc_resume_reg);
}

#define DDR_PHY_OFFSET  (-0x4e0000 + 0x1000)
#define DDRP_INNOPHY_RXDLL_DELAY_AL             (DDR_PHY_OFFSET + (0x48 << 2))
#define DDRP_INNOPHY_RXDLL_DELAY_AH             (DDR_PHY_OFFSET + (0x58 << 2))

#define REG32(addr) *(volatile unsigned int *)(addr)
#define ddr_writel(value, reg)  (REG32(DDRC_BASE + reg) = (value))
#define ddr_readl(reg)          REG32(DDRC_BASE + reg)

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





static unsigned int get_resume_pc(void)
{
	unsigned int resume_pc;

	store_regs = (struct store_regs *)RTC_RAM_DATA_ADDR;
	resume_pc = store_regs->resume_pc;

	return resume_pc;
}


static void rtc_write_reg(unsigned int reg, unsigned int val)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );
	*(volatile unsigned int *)0xb000303c = 0xa55a;
	while (!((*(volatile unsigned int *)0xb000303c >> 31) & 0x1) );
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );

	*(volatile unsigned int *)reg = val;

	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );
}

static unsigned int  rtc_read_reg(unsigned int reg)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1) );

	return *(volatile unsigned int *)reg;
}

void fastboot_restore_code_2(void)
{
	unsigned int resume_pc;

#ifdef FASTBOOT_DEBUG
	serial_init(2);
#endif

	resume_pc = get_resume_pc();
	DEBUG_HEX(resume_pc);

	DEBUG("fastboot begin restore\n");

	pll_restore();
	DEBUG("pll restore ok\n");

	ddr_init();

	{
		unsigned int ddrc_ctrl = *(volatile unsigned int *)0xb34f0010;
		volatile unsigned int tmp;

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


	ddr_writel(1, DDRC_AUTOSR_EN);
	while(ddr_readl(DDRC_STATUS) & (1<<2));

	DEBUG("ddr restore finish\n");

	ddrp_auto_calibration();

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


