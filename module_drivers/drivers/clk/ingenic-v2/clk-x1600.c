#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/clk.h>

#include <dt-bindings/clock/ingenic-x1600.h>

#include "clk.h"



/*******************************************************************************
 *      FIXED CLK
 ********************************************************************************/
static struct ingenic_fixed_rate_clock x1600_fixed_rate_ext_clks[] __initdata = {
	FRATE(CLK_EXT, "ext", NULL, 0, 24000000),
	FRATE(CLK_RTC_EXT, "rtc_ext", NULL, 0, 32768),
	FRATE(CLK_EXT_DIV_512, "ext_div_512", NULL, 0, 46875),
};



/*******************************************************************************
 *      PLL
 ********************************************************************************/

static struct ingenic_pll_rate_table x1600_pll_rate_table[] = {
	PLL_RATE(1000000000, 125, 3, 1, 1),
	PLL_RATE(900000000, 75, 1, 2, 1),
	PLL_RATE(800000000, 100, 1, 3, 1),
	PLL_RATE(700000000, 175, 2, 3, 1),
	PLL_RATE(600000000, 25, 1, 1, 1),
	PLL_RATE(500000000, 125, 2, 3, 1),
	PLL_RATE(400000000, 50, 1, 3, 1),
};



/*PLL HWDESC*/
static struct ingenic_pll_hwdesc apll_hwdesc = \
	PLL_DESC(CPM_CPAPCR, 20,12, 14, 6, 11, 3, 8, 3, 3, 0);

static struct ingenic_pll_hwdesc mpll_hwdesc = \
	PLL_DESC(CPM_CPMPCR, 20,12, 14, 6, 11, 3, 8, 3, 3, 0);

static struct ingenic_pll_hwdesc epll_hwdesc = \
	PLL_DESC(CPM_CPEPCR, 20,12, 14, 6, 11, 3, 8, 3, 3, 0);



static struct ingenic_pll_clock x1600_pll_clks[] __initdata = {
	PLL(CLK_PLL_APLL, "apll", "ext", &apll_hwdesc, x1600_pll_rate_table),
	PLL(CLK_PLL_MPLL, "mpll", "ext", &mpll_hwdesc, x1600_pll_rate_table),
	PLL(CLK_PLL_EPLL, "epll", "ext", &epll_hwdesc, x1600_pll_rate_table),
};


/*******************************************************************************
 *      MUX
 ********************************************************************************/
PNAME(mux_table_0)	= {"stop",	"ext",		"apll"};
PNAME(mux_table_1)	= {"stop",	"sclka",	"mpll"};
PNAME(mux_table_2)	= {"sclka",	"mpll",		"epll"};
PNAME(mux_table_3)	= {"sclka", 	"mpll",		"epll",		"ext"};
PNAME(mux_table_4)	= {"sclka", 	"epll"};
PNAME(mux_table_5)	= {"ext_div_512", 	"rtc_ext"};

static unsigned int ingenic_mux_table[] = {0, 1, 2, 3, 4};


static struct ingenic_mux_clock x1600_mux_clks[] __initdata = {
	MUX(CLK_MUX_SCLKA,	"sclka",		ingenic_mux_table, mux_table_0, CPM_CPCCR,	30, 2, 0),
	MUX(CLK_MUX_CPU_L2C,	"mux_cpu_l2c",		ingenic_mux_table, mux_table_1, CPM_CPCCR,	28, 2, 0),
	MUX(CLK_MUX_AHB0,	"mux_ahb0",		ingenic_mux_table, mux_table_1, CPM_CPCCR,	26, 2, 0),
	MUX(CLK_MUX_AHB2,	"mux_ahb2",		ingenic_mux_table, mux_table_1, CPM_CPCCR,	24, 2, 0),

	MUX(CLK_MUX_DDR,	"mux_ddr",		ingenic_mux_table, mux_table_1, CPM_DDRCDR,	30, 2, 0),
	MUX(CLK_MUX_MACPHY,	"mux_macphy",		ingenic_mux_table, mux_table_2, CPM_MACPHYCDR,	30, 2, 0),
	MUX(CLK_MUX_I2S0T,	"mux_i2s0t",		ingenic_mux_table, mux_table_4, CPM_I2SCDR1,	30, 1, 0),
	MUX(CLK_MUX_I2S0R,	"mux_i2s0r",		ingenic_mux_table, mux_table_4, CPM_I2SCDR,	30, 1, 0),
	MUX(CLK_MUX_LCD,	"mux_lcd",		ingenic_mux_table, mux_table_2, CPM_LPCDR,	30, 2, 0),
	MUX(CLK_MUX_MSC0,	"mux_msc0",		ingenic_mux_table, mux_table_2, CPM_MSC0CDR,	30, 2, 0),
	MUX(CLK_MUX_MSC1,	"mux_msc1",		ingenic_mux_table, mux_table_2, CPM_MSC1CDR,	30, 2, 0),
	MUX(CLK_MUX_SFC,	"mux_sfc",		ingenic_mux_table, mux_table_2, CPM_SFCCDR,	30, 2, 0),
	MUX(CLK_MUX_SSI,	"mux_ssi",		ingenic_mux_table, mux_table_2, CPM_SSICDR,	30, 2, 0),
	MUX(CLK_MUX_CIM,	"mux_cim",		ingenic_mux_table, mux_table_2, CPM_CIMCDR,	30, 2, 0),
	MUX(CLK_MUX_PWM,	"mux_pwm",		ingenic_mux_table, mux_table_2, CPM_PWMCDR,	30, 2, 0),
	MUX(CLK_MUX_CAN0,	"mux_can0",		ingenic_mux_table, mux_table_3, CPM_CAN0CDR,	30, 2, 0),
	MUX(CLK_MUX_CAN1,	"mux_can1",		ingenic_mux_table, mux_table_3, CPM_CAN1CDR,	30, 2, 0),
	MUX(CLK_MUX_CDBUS,	"mux_cdbus",		ingenic_mux_table, mux_table_2, CPM_CDBUSCDR,	30, 2, 0),
	MUX(CLK_MUX_WDT,	"mux_wdt",		ingenic_mux_table, mux_table_5, CPM_OPCR,	2, 1, 0),

};

/*******************************************************************************
 *      DIV BUS
 ********************************************************************************/

static struct ingenic_bus_clock x1600_bus_div_clks[] __initdata = {
	BUS_DIV(CLK_DIV_CPU,		"div_cpu",		"mux_cpu_l2c",		CPM_CPCCR,	0,  4, 0, 0, CPM_CPCSR, 0, 22, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_L2C,		"div_l2c",		"mux_cpu_l2c",		CPM_CPCCR,	4,  4, 0, 0, CPM_CPCSR, 0, 22, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_AHB0,		"div_ahb0",		"mux_ahb0",		CPM_CPCCR,	8,  4, 0, 0, CPM_CPCSR, 1, 21, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_AHB2,		"div_ahb2",		"mux_ahb2",		CPM_CPCCR,	12, 4, 0, 0, CPM_CPCSR, 2, 20, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_APB,		"div_apb",		"mux_ahb2",		CPM_CPCCR,	16, 4, 0, 0, CPM_CPCSR, 2, 20, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_CPU_L2C_X1,	"div_cpu_l2c_x1",	"mux_cpu_l2c",		CPM_CPCCR,	0,  4, 4, 4, CPM_CPCSR, 0, 22, BUS_DIV_ONE),
	BUS_DIV(CLK_DIV_CPU_L2C_X2,	"div_cpu_l2c_x2",	"mux_cpu_l2c",		CPM_CPCCR,	0,  4, 4, 4, CPM_CPCSR, 0, 22, BUS_DIV_TWO),
};


/*******************************************************************************
 *      DIV
 ********************************************************************************/

static struct clk_div_table x1600_clk_div_table[256] = {};

static struct ingenic_div_clock x1600_div_clks[] __initdata = {
	DIV(CLK_DIV_DDR,		"div_ddr",		"mux_ddr",		CPM_DDRCDR,	4,  0, NULL),
	DIV(CLK_DIV_MACPHY,		"div_macphy",		"mux_macphy",		CPM_MACPHYCDR,	8,  0, NULL),
	DIV(CLK_DIV_LCD,		"div_lcd",		"mux_lcd",		CPM_LPCDR,	8,  0, NULL),
	DIV(CLK_DIV_MSC0,		"div_msc0",		"mux_msc0",		CPM_MSC0CDR,	8,  0, x1600_clk_div_table),
	DIV(CLK_DIV_MSC1,		"div_msc1",		"mux_msc1",		CPM_MSC1CDR,	8,  0, x1600_clk_div_table),
	DIV(CLK_DIV_SFC,		"div_sfc",		"mux_sfc",		CPM_SFCCDR,	8,  0, NULL),
	DIV(CLK_DIV_SSI,		"div_ssi",		"mux_ssi",		CPM_SSICDR,	8,  0, NULL),
	DIV(CLK_DIV_CIM,		"div_cim",		"mux_cim",		CPM_CIMCDR,	8,  0, NULL),
	DIV(CLK_DIV_PWM,		"div_pwm",		"mux_pwm",		CPM_PWMCDR,	4,  0, NULL),
	DIV(CLK_DIV_CAN0,		"div_can0",		"mux_can0",		CPM_CAN0CDR,	8,  0, NULL),
	DIV(CLK_DIV_CAN1,		"div_can1",		"mux_can1",		CPM_CAN1CDR,	8,  0, NULL),
	DIV(CLK_DIV_CDBUS,		"div_cdbus",		"mux_cdbus",		CPM_CDBUSCDR,	8,  0, NULL),

};

/*******************************************************************************
 *      FRACTIONAL-DIVIDER
 ********************************************************************************/

static struct ingenic_fra_div_clock x1600_fdiv_clks[] __initdata = {
	FRA_DIV(CLK_DIV_I2S0R,	"div_i2s0r",	"mux_i2s0r",	CPM_I2SCDR,	20, 9, 0, 20),
	FRA_DIV(CLK_DIV_I2S0T,	"div_i2s0t",	"mux_i2s0t",	CPM_I2SCDR1,	20, 9, 0, 20),
};

/*******************************************************************************
 *      GATE
 ********************************************************************************/
static struct ingenic_gate_clock x1600_gate_clks[] __initdata = {

	GATE(CLK_GATE_DDR,	"gate_ddr",		"div_ddr",	CPM_CLKGR,  31, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AHB0,	"gate_ahb0",		"div_ahb0",	CPM_CLKGR,  29, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_APB0,	"gate_apb0",		"div_ahb0",	CPM_CLKGR,  28, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_RTC,	"gate_rtc",		"rtc_ext",	CPM_CLKGR,  27, 0,			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AES,	"gate_aes",		"div_ahb2",	CPM_CLKGR,  24, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_LCD,	"gate_lcd",		"div_lcd",	CPM_CLKGR,  23, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CIM,	"gate_cim",		"div_cim",	CPM_CLKGR,  22, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PDMA,	"gate_pdma",		"div_ahb2",	CPM_CLKGR,  21, CLK_IGNORE_UNUSED, 	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OST,	"gate_ost",		"ext",		CPM_CLKGR,  20, CLK_IGNORE_UNUSED, 	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSI0,	"gate_ssi0",		"div_ssi",	CPM_CLKGR,  19, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TCU,	"gate_tcu",		"div_apb",	CPM_CLKGR,  18, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DTRNG,	"gate_dtrng",		"div_apb",	CPM_CLKGR,  17, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART2,	"gate_uart2",		"div_apb",	CPM_CLKGR,  16, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART1,	"gate_uart1",		"div_apb",	CPM_CLKGR,  15, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART0,	"gate_uart0",		"div_apb",	CPM_CLKGR,  14, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SADC,	"gate_sadc",		"div_apb",	CPM_CLKGR,  13, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AUDIO,	"gate_audio",		"div_ahb2",	CPM_CLKGR,  11, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSISLV,	"gate_ssislv",		"div_apb",	CPM_CLKGR,  10, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB1,	"gate_i2c1",		"div_apb",	CPM_CLKGR,  8,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB0,	"gate_i2c0",		"div_apb",	CPM_CLKGR,  7,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC1,	"gate_msc1",		"div_msc1",	CPM_CLKGR,  5,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC0,	"gate_msc0",		"div_msc0",	CPM_CLKGR,  4,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OTG,	"gate_otg",		"div_ahb2",	CPM_CLKGR,  3,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SFC,	"gate_sfc",		"div_sfc",	CPM_CLKGR,  2,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_EFUSE,	"gate_efuse",		"div_ahb2",	CPM_CLKGR,  1,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_NEMC,	"gate_nemc",		"div_ahb2",	CPM_CLKGR,  0,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ARB,	"gate_arb",		"div_ahb0",	CPM_CLKGR1, 30, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MIPI_CSI, "gate_csi",		"div_ahb0",	CPM_CLKGR1, 28, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_INTC,	"gate_intc",		"div_ahb2",	CPM_CLKGR1, 26, CLK_IGNORE_UNUSED, 	CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_GMAC0,	"gate_gmac0",		"div_ahb2",	CPM_CLKGR1, 23, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART3,	"gate_uart3",		"div_apb",	CPM_CLKGR1, 16, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S0T,	"gate_i2s0t",		"div_i2s0t",	CPM_CLKGR1, 9,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S0R,	"gate_i2s0r",		"div_i2s0r",	CPM_CLKGR1, 8,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_HASH,	"gate_hash",		"div_ahb2",	CPM_CLKGR1, 6,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PWM,	"gate_pwm",		"div_pwm",	CPM_CLKGR1, 5,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CDBUS,	"gate_cdbus",		"div_apb",	CPM_CLKGR1, 2,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CAN1,	"gate_can1",		"div_apb",	CPM_CLKGR1, 1,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CAN0,	"gate_can0",		"div_apb",	CPM_CLKGR1, 0,  0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_USBPHY,	"gate_usbphy",		"div_apb",	CPM_OPCR,   23, 0, 			CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_CE_I2S0T,	"ce_i2s0t",		"div_i2s0t",	CPM_I2SCDR1, 29, 0, 0),
	GATE(CLK_CE_I2S0R,	"ce_i2s0r",		"div_i2s0r",	CPM_I2SCDR, 29, 0, 0),
};

static struct ingenic_gate_power x1600_gate_power[] __initdata = {

	POWER(PD_MEM_CDBUS,	"pd_mem_cdbus",		"gate_cdbus",	CPM_MPDCR0,  29, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PWM,	"pd_mem_pwm",		"gate_cdbus",	CPM_MPDCR0,  26, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_GMAC,	"pd_mem_gmac",		"gate_cdbus",	CPM_MPDCR0,  25, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_USB,	"pd_mem_usb",		"gate_usb",	CPM_MPDCR0,  24, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SFC,	"pd_mem_sfc",		"gate_sfc",	CPM_MPDCR0,  23, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_MSC1,	"pd_mem_msc1",		"gate_msc1",	CPM_MPDCR0,  22, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_MSC0,	"pd_mem_msc0",		"gate_msc0",	CPM_MPDCR0,  21, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_AES,	"pd_mem_aes",		"gate_aes",	CPM_MPDCR0,  20, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PDMA_SLP,	"pd_mem_pdma_slp",	"gate_pdma",	CPM_MPDCR0,  19, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PDMA_SEC,	"pd_mem_pdma_sec",	"gate_pdma",	CPM_MPDCR0,  18, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PDMA,	"pd_mem_pdma",		"gate_pdma",	CPM_MPDCR0,  17, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_NEMC,	"pd_mem_nemc",		"gate_nemc",	CPM_MPDCR0,  16, 0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_AIC,	"pd_mem_aic",		"gate_audio",	CPM_MPDCR0,  6,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SSISLV,	"pd_mem_ssislv",	"gate_ssislv",	CPM_MPDCR0,  5,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SSI0,	"pd_mem_ssi0",		"gate_ssi0",	CPM_MPDCR0,  4,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART3,	"pd_mem_uart3",		"gate_uart3",	CPM_MPDCR0,  3,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART2,	"pd_mem_uart2",		"gate_uart2",	CPM_MPDCR0,  2,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART1,	"pd_mem_uart1",		"gate_uart1",	CPM_MPDCR0,  1,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART0,	"pd_mem_uart0",		"gate_uart0",	CPM_MPDCR0,  0,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_CIM,	"pd_mem_cim",		"gate_cim",	CPM_MPDCR1, 5,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DPU,	"pd_mem_dpu",		"gate_dpu",	CPM_MPDCR1, 4,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH6,	"pd_mem_ddr_ch6",	"gate_ddr",	CPM_MPDCR1, 3,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH5,	"pd_mem_ddr_ch5",	"gate_ddr",	CPM_MPDCR1, 2,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH3,	"pd_mem_ddr_ch3",	"gate_ddr",	CPM_MPDCR1, 1,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_TOP,	"pd_mem_ddr_top",	"gate_ddr",	CPM_MPDCR1, 0,  0, CLK_IGNORE_UNUSED,	CLK_GATE_SET_TO_DISABLE, 0),

};

static void clk_div_table_generate(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(x1600_clk_div_table); i++) {
		x1600_clk_div_table[i].val = i;
		x1600_clk_div_table[i].div = (i + 1) * 2;
	}

}

static const struct of_device_id ext_clk_match[] __initconst = {
	        { .compatible = "ingenic,fixed-clock", .data = (void *)0, },
		{},
};


/* Register x1600 clocks. */
static void __init x1600_clk_init(struct device_node *np, void __iomem *base)
{
	struct ingenic_clk_provider *ctx;
	void __iomem *reg_base;


	printk("x1600 Clock Power Management Unit init!\n");

	reg_base = base;

	if (np) {
		reg_base = of_iomap(np, 0);
		if (!reg_base)
			panic("%s: failed to map registers\n", __func__);
	}


	ctx = ingenic_clk_init(np, reg_base, NR_CLKS);
	if (!ctx)
		panic("%s: unable to allocate context.\n", __func__);

	/* Register Ext Clocks From DT */
	ingenic_clk_of_register_fixed_ext(ctx, x1600_fixed_rate_ext_clks,
			                        ARRAY_SIZE(x1600_fixed_rate_ext_clks), ext_clk_match);

	/* Register PLLs. */
	ingenic_clk_register_pll(ctx, x1600_pll_clks,
				ARRAY_SIZE(x1600_pll_clks), reg_base);


	/* Register Muxs */
	ingenic_clk_register_mux(ctx, x1600_mux_clks, ARRAY_SIZE(x1600_mux_clks));

	/* Register Bus Divs */
	ingenic_clk_register_bus_div(ctx, x1600_bus_div_clks, ARRAY_SIZE(x1600_bus_div_clks));

	/* Register Divs */
	clk_div_table_generate();
	ingenic_clk_register_cgu_div(ctx, x1600_div_clks, ARRAY_SIZE(x1600_div_clks));

	/* Register Fractional Divs */
	ingenic_clk_register_fra_div(ctx, x1600_fdiv_clks, ARRAY_SIZE(x1600_fdiv_clks));

	/* Register Gates */
	ingenic_clk_register_gate(ctx, x1600_gate_clks, ARRAY_SIZE(x1600_gate_clks));

	/* Register Powers */
	ingenic_power_register_gate(ctx, x1600_gate_power, ARRAY_SIZE(x1600_gate_power));

	ingenic_clk_of_add_provider(np, ctx);


	//ingenic_clk_of_dump(ctx);


	pr_info("=========== x1600 clocks: =============\n"
		"\tapll     = %lu , mpll     = %lu, ddr = %lu\n"
		"\tcpu_clk  = %lu , l2c_clk  = %lu\n"
		"\tahb0_clk = %lu , ahb2_clk = %lu\n"
		"\tapb_clk  = %lu , ext_clk  = %lu\n\n",
		_get_rate("apll"),	_get_rate("mpll"), _get_rate("div_ddr"),
		_get_rate("div_cpu"), _get_rate("div_l2c"),
		_get_rate("div_ahb0"), _get_rate("div_ahb2"),
		_get_rate("div_apb"), _get_rate("ext"));

}

CLK_OF_DECLARE(x1600_clk, "ingenic,x1600-clocks", x1600_clk_init);

