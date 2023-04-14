#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/ctype.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include "pm.h"
#include "pm_sleep.h"
#include "pm_fastboot.h"


#define SLEEP_LEVEL		(*(volatile unsigned int *)0xb0000034 & 0xf)
#define SLEEP_LEVEL_MAGIC	((*(volatile unsigned int *)0xb0000034 >> 16) & 0xffff)
#define SLEEP_MAGIC		(0x0915)


#ifdef DEBUG_PM
static void m300_pm_gate_check(void)
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



void load_func_to_tcsm(unsigned int *tcsm_addr,unsigned int *f_addr,unsigned int size)
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
	printk("soc pm idle \n");

	load_func_to_sram();

	soc_pm_idle_config();

	return 0;
}

static int soc_pm_idle_pd(void)
{
	printk("soc pm idle pd \n");

	load_func_to_sram();

	soc_pm_idle_pd_config();

	soc_set_reset_entry();

	return 0;
}

static void soc_pm_sleep(void)
{
	printk("soc pm sleep \n");

	load_func_to_sram();

	soc_pm_sleep_config();

	soc_set_reset_entry();
}


static void soc_pm_fastboot(void)
{
	printk("soc pm fastboot \n");

	soc_pm_fastboot_config();

	load_func_to_rtc_ram();

	sys_save();
}


static void goto_sleep(unsigned int sleep_addr)
{
	mb();
	save_goto(sleep_addr);
	mb();
}

int m300_pm_enter(suspend_state_t state)
{
	unsigned int sleep_addr = 0;
	unsigned int sleep_level = 0;

	printk("m300 pm enter!!\n");

	sleep_param->uart_base = (UART0_IOBASE | 0xa0000000) + UART_OFF * bc_idx;
	sleep_param->state = state;

	if (SLEEP_LEVEL_MAGIC == SLEEP_MAGIC) {
		if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE)) {
			soc_pm_idle();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE_PD)) {
			soc_pm_idle_pd();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == SLEEP)) {
			soc_pm_sleep();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == FASTBOOT)) {
			soc_pm_fastboot();
			sleep_addr = FASTBOOT_SLEEP_CODE_ADDR;
			sleep_level = FASTBOOT;
		} else {
			printk("error suspend state or suspend level ! \n");
			return -1;
		}
	} else {
		if (state == PM_SUSPEND_STANDBY) {
			soc_pm_idle();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if (state == PM_SUSPEND_MEM) {
			soc_pm_sleep();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else {
			printk("error suspend state ! \n");
			return -1;
		}
	}


#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
	m300_pm_gate_check();
#endif


	goto_sleep(sleep_addr);


	if (sleep_level == FASTBOOT) {
		soc_pm_wakeup_fastboot();
	} else {
		soc_pm_wakeup_idle_sleep();
	}

	return 0;
}

static int m300_pm_begin(suspend_state_t state)
{
	printk("m300 suspend begin\n");
	return 0;
}

static void m300_pm_end(void)
{
	printk("m300 pm end!\n");
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
static const struct platform_suspend_ops m300_pm_ops = {
	.valid		= ingenic_pm_valid,
	.begin		= m300_pm_begin,
	.enter		= m300_pm_enter,
	.end		= m300_pm_end,
};


int bc_idx = 0;
static int __init suspend_console_setup(char *str)
{

	char buf[32];
	char *s;

	strncpy(buf, str, sizeof(buf) - 1);

	for (s = buf; *s; s++)
		if (isdigit(*s) || *s == ',')
			break;

	bc_idx = simple_strtoul(s, NULL, 10);

	return 0;
}

__setup("console=", suspend_console_setup);




/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&m300_pm_ops);


	return 0;
}

late_initcall(pm_init);
