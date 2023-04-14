

#define RTC_SLEEP_MEMORY_START          0xb0004000
#define RTC_SLEEP_MEMORY_END            0xb0005000
#define RTC_RAM_SP_ADDR                 (RTC_RAM_DATA_ADDR - 4)
//#define RTC_RAM_SP_SIZE                 (256 - 4)
#define RTC_RAM_DATA_ADDR               0xb0004c00
#define RTC_RAM_DATA_SIZE               1024
#define RTC_RAM_CODE_ADDR1              0xb0004000
#define RTC_RAM_CODE_ADDR2              (RTC_RAM_CODE_ADDR1 + 64)
#define RTC_RAM_CODE_SIZE               0xb00

#define SLEEP_CPU_SLEEP_TEXT		0xb2400000
#define SLEEP_CPU_SLEEP_LEN		4096



struct pll_resume_reg {
	unsigned int cpccr;
	unsigned int cppcr;
	unsigned int cpapcr;
	unsigned int cpmpcr;
	unsigned int cpepcr;
	unsigned int ddrcdr;
};

struct ddrc_resume_reg {
	unsigned int dcfg;
	unsigned int dctrl;
	unsigned int dlmr;
	unsigned int ddlp;
	unsigned int dasr_en;
	unsigned int dasr_cnt;
	unsigned int drefcnt;
	unsigned int dtimming1;
	unsigned int dtimming2;
	unsigned int dtimming3;
	unsigned int dtimming4;
	unsigned int dtimming5;
	unsigned int dmmap0;
	unsigned int dmmap1;
	unsigned int dbwcfg;
	unsigned int dbwstp;
	unsigned int hregpro;
	unsigned int dbgen;
	unsigned int dwcfg;
	unsigned int dremap1;
	unsigned int dremap2;
	unsigned int dremap3;
	unsigned int dremap4;
	unsigned int dremap5;
	unsigned int cpac;
	unsigned int cchc0;
	unsigned int cchc1;
	unsigned int cchc2;
	unsigned int cchc3;
	unsigned int cchc4;
	unsigned int cchc5;
	unsigned int cchc6;
	unsigned int cchc7;
	unsigned int cschc0;
	unsigned int cschc1;
	unsigned int cschc2;
	unsigned int cschc3;
	unsigned int cmonc0;
	unsigned int cmonc1;
	unsigned int cmonc2;
	unsigned int cmonc3;
	unsigned int cmonc4;
	unsigned int ccguc0;
	unsigned int ccguc1;
	unsigned int pregpro;
	unsigned int bufcfg;
};

struct ddr_phy_resume_reg {
	unsigned int phy_rst;
	unsigned int mem_cfg;
	unsigned int dq_width;
	unsigned int cl;
	unsigned int al;
	unsigned int cwl;
	unsigned int pll_fbdiv;
	unsigned int pll_ctrl;
	unsigned int pll_pdiv;
	unsigned int training_ctrl;
	unsigned int calib_bypass_al;
	unsigned int calib_bypass_ah;
	unsigned int wl_mode1;
	unsigned int wl_mode2;

};

struct store_regs {
	unsigned int resume_pc;
	unsigned int uart_index;
	struct pll_resume_reg pll_resume_reg;
	struct ddrc_resume_reg ddrc_resume_reg;
	struct ddr_phy_resume_reg ddr_phy_resume_reg;
};




#define DDR_PHY_OFFSET  (-0x4e0000 + 0x1000)
#define DDRP_INNOPHY_RXDLL_DELAY_AL             (DDR_PHY_OFFSET + (0x48 << 2))
#define DDRP_INNOPHY_RXDLL_DELAY_AH             (DDR_PHY_OFFSET + (0x58 << 2))

#define REG32(addr) *(volatile unsigned int *)(addr)
#define ddr_writel(value, reg)  (REG32(DDRC_BASE + reg) = (value))
#define ddr_readl(reg)          REG32(DDRC_BASE + reg)



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


#define FASTBOOT_DEBUG

#ifdef FASTBOOT_DEBUG

#include <uart.h>

extern unsigned int get_uart_index(void);

void DEBUG(const char *s)
{
	serial_puts(get_uart_index(), s);
}

void DEBUG_HEX(unsigned int  d)
{
	serial_put_hex(get_uart_index(), d);
}


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

