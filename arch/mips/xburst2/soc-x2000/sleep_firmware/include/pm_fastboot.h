

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
#if 0
	unsigned int dbwcfg;
	unsigned int dbwstp;
	unsigned int hregpro;
	unsigned int dbgen;
#endif
	unsigned int dwcfg;
	unsigned int dremap1;
	unsigned int dremap2;
	unsigned int dremap3;
	unsigned int dremap4;
	unsigned int dremap5;
#if 0
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
#endif
	unsigned int ccguc0;
	unsigned int ccguc1;
#if 0
	unsigned int pregpro;
	unsigned int bufcfg;
#endif
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
	struct pll_resume_reg pll_resume_reg;
	struct ddrc_resume_reg ddrc_resume_reg;
	struct ddr_phy_resume_reg ddr_phy_resume_reg;
};

