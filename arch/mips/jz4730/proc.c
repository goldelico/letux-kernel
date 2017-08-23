/*
 * linux/arch/mips/jz4730/proc.c
 *
 * /proc/jz/ procfs for on-chip peripherals.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#if CONFIG_JZ4730_MINIPC

#define NETWORKLED_IO 9  //wjx network eth0 led 2008.1.29
#define BATTERY_IO 17  //wjx check the battery capacity 2007.12.10
#define CAPSLOCKLED_IO 27   //wjx 2008.3.31
#define NUMLOCKLED_IO  86
#define INTERNAL_WIFI_IO 95 //wey 2008.5.28

#endif

struct proc_dir_entry *proc_jz_root;

/*
 * EMC Module
 */
static int emc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "BCR:       0x%08x\n", REG_EMC_BCR);
	len += sprintf (page+len, "SMCR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SMCR0, REG_EMC_SMCR1, REG_EMC_SMCR2, REG_EMC_SMCR3, REG_EMC_SMCR4, REG_EMC_SMCR5);
	len += sprintf (page+len, "SACR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SACR0, REG_EMC_SACR1, REG_EMC_SACR2, REG_EMC_SACR3, REG_EMC_SACR4, REG_EMC_SACR5);
	len += sprintf (page+len, "DMCR:      0x%08x\n", REG_EMC_DMCR);
	len += sprintf (page+len, "RTCSR:     0x%04x\n", REG_EMC_RTCSR);
	len += sprintf (page+len, "RTCOR:     0x%04x\n", REG_EMC_RTCOR);
	len += sprintf (page+len, "DMAR(0-1): 0x%08x 0x%08x\n", REG_EMC_DMAR1, REG_EMC_DMAR2);
	return len;
}

/* 
 * Power Manager Module
 */
static int pmc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned long lpcr = REG_CPM_LPCR;
	unsigned long mscr = REG_CPM_MSCR;

	len += sprintf (page+len, "LPCR           : 0x%08lx\n", lpcr);
	len += sprintf (page+len, "Low Power Mode : %s\n", 
			((lpcr & CPM_LPCR_LPM_MASK) == (CPM_LPCR_LPM_IDLE)) ?
			"idle" : (((lpcr & CPM_LPCR_LPM_MASK) == (CPM_LPCR_LPM_SLEEP)) ? "sleep" : "hibernate"));
	len += sprintf (page+len, "Doze Mode      : %s\n", 
			(lpcr & CPM_LPCR_DOZE) ? "on" : "off");
	if (lpcr & CPM_LPCR_DOZE)
		len += sprintf (page+len, "     duty      : %d\n", (int)((lpcr & CPM_LPCR_DUTY_MASK) >> CPM_LPCR_DUTY_BIT));
	len += sprintf (page+len, "CKO1           : %s\n",
			(REG_CPM_CFCR & CPM_CFCR_CKOEN1) ? "enable" : "disable");
	len += sprintf (page+len, "UART0          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART0) ? "stopped" : "running");
	len += sprintf (page+len, "UART1          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART1) ? "stopped" : "running");
	len += sprintf (page+len, "UART2          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART2) ? "stopped" : "running");
	len += sprintf (page+len, "UART3          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART3) ? "stopped" : "running");
	len += sprintf (page+len, "OST            : %s\n",
			(mscr & CPM_MSCR_MSTP_OST) ? "stopped" : "running");
	len += sprintf (page+len, "DMAC           : %s\n",
			(mscr & CPM_MSCR_MSTP_DMAC) ? "stopped" : "running");
	len += sprintf (page+len, "ETH            : %s\n",
			(mscr & CPM_MSCR_MSTP_ETH) ? "stopped" : "running");
	len += sprintf (page+len, "UHC/UDC        : %s\n",
			(mscr & CPM_MSCR_MSTP_UHC) ? "stopped" : "running");
	len += sprintf (page+len, "PWM0           : %s\n",
			(mscr & CPM_MSCR_MSTP_PWM0) ? "stopped" : "running");
	len += sprintf (page+len, "PWM1           : %s\n",
			(mscr & CPM_MSCR_MSTP_PWM1) ? "stopped" : "running");
	len += sprintf (page+len, "I2C            : %s\n",
			(mscr & CPM_MSCR_MSTP_I2C) ? "stopped" : "running");
	len += sprintf (page+len, "SSI            : %s\n",
			(mscr & CPM_MSCR_MSTP_SSI) ? "stopped" : "running");
	len += sprintf (page+len, "SCC            : %s\n",
			(mscr & CPM_MSCR_MSTP_SCC) ? "stopped" : "running");
	return len;
}

static int pmc_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	REG_CPM_MSCR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * Clock Generation Module
 */
static int cgm_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int cfcr = REG_CPM_CFCR;
	unsigned int plcr1 = REG_CPM_PLCR1;
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};


	len += sprintf (page+len, "PLCR1          : 0x%08x\n", plcr1);
	len += sprintf (page+len, "CFCR           : 0x%08x\n", cfcr);
	len += sprintf (page+len, "PLL            : %s\n", 
			(plcr1 & CPM_PLCR1_PLL1EN) ? "ON" : "OFF");
	len += sprintf (page+len, "NF:NR:NO       : %d:%d:%d\n",
			__cpm_plcr1_fd() + 2,
			__cpm_plcr1_rd() + 2,
			od[__cpm_plcr1_od()]
		);
	len += sprintf (page+len, "I:S:M:P        : %d:%d:%d:%d\n", 
			div[(cfcr & CPM_CFCR_IFR_MASK) >> CPM_CFCR_IFR_BIT],
			div[(cfcr & CPM_CFCR_SFR_MASK) >> CPM_CFCR_SFR_BIT],
			div[(cfcr & CPM_CFCR_MFR_MASK) >> CPM_CFCR_MFR_BIT],
			div[(cfcr & CPM_CFCR_PFR_MASK) >> CPM_CFCR_PFR_BIT]
		);
	len += sprintf (page+len, "PLL Freq       : %d MHz\n", __cpm_get_pllout()/1000000);
	len += sprintf (page+len, "ICLK           : %d MHz\n", __cpm_get_iclk()/1000000);
	len += sprintf (page+len, "SCLK           : %d MHz\n", __cpm_get_sclk()/1000000);
	len += sprintf (page+len, "MCLK           : %d MHz\n", __cpm_get_mclk()/1000000);
	len += sprintf (page+len, "PCLK           : %d MHz\n", __cpm_get_pclk()/1000000);
	len += sprintf (page+len, "DEVCLK         : %d MHz\n", __cpm_get_devclk()/1000000);
	len += sprintf (page+len, "RTCCLK         : %d KHz\n", __cpm_get_rtcclk()/1000);
	len += sprintf (page+len, "USBCLK         : %d MHz\n", __cpm_get_usbclk()/1000000);
#if defined(CONFIG_FB_JZ)
	len += sprintf (page+len, "LCDCLK         : %d MHz\n", __cpm_get_lcdclk()/1000000);
	len += sprintf (page+len, "PIXCLK         : %d MHz\n", __cpm_get_pixclk()/1000000);
#endif
	return len;
}

static int cgm_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CFCR = simple_strtoul(buffer, 0, 16);
	return count;
}

/* 
 * WDT
 */
static int wdt_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "WDT_WTCSR   : 0x%08x\n", REG_WDT_WTCSR);
	len += sprintf (page+len, "WDT_WTCNT   : 0x%08x\n", REG_WDT_WTCNT);

	return len;
}

static int wdt_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long cnt = simple_strtoul(buffer, 0, 16);

	REG_WDT_WTCNT = cnt;
	REG_WDT_WTCSR = WDT_WTCSR_START;

	return count;
}

/*
 * PWM
 */

static int proc_jz_pwm_read_byte(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	return sprintf (page, "0x%02x\n", REG8(data));
}

static int proc_jz_pwm_read_word(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
     	return sprintf (page, "0x%04x\n", REG16(data));
}

static int proc_jz_pwm_write_byte(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG8(data) = simple_strtoul(buffer, 0, 16);
	return count;
}
                         
static int proc_jz_pwm_write_word(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG16(data) =  simple_strtoul(buffer, 0, 16);
	return count;
}

#define PWM_NUM 2

static int jz_pwm_proc_init(void) 
{
	struct proc_dir_entry *proc_jz_pwm, *res;
	char name[16];
	unsigned char i;

	for (i = 0; i < PWM_NUM; i++) {
		sprintf(name, "pwm%d", i);
		proc_jz_pwm = proc_mkdir(name, proc_jz_root);
		res = create_proc_entry("control", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_byte;
			res->write_proc = proc_jz_pwm_write_byte;
			if (i)
				res->data = (void * )PWM_CTR(1);
			else
				res->data = (void * )PWM_CTR(0);
		}
		res = create_proc_entry("period", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_word;
			res->write_proc = proc_jz_pwm_write_word;
			if (i)
				res->data = (void *)PWM_PER(1);
			else
				res->data = (void *)PWM_PER(0);
		}
		res = create_proc_entry("duty", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_word;
			res->write_proc = proc_jz_pwm_write_word;
			if (i)
				res->data = (void * )PWM_DUT(1);
			else
				res->data = (void * )PWM_DUT(0);
		}
	}
	return 0;
}

#if CONFIG_JZ4730_MINIPC

/* wjx 2007.12.10 battery status */
static int battery_read_proc (char *page, char **start, off_t off,
							  int count, int *eof, void *data)
{
	int len = 0;
	
	__gpio_as_input(BATTERY_IO);
	
	if (__gpio_get_pin(BATTERY_IO) == 1)
		{
			len += sprintf (page+len, "1\n");		
		}
	else
		{
			len += sprintf (page+len, "0\n");						
		}
	return len;
}

static int eth0_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	printk("eth0_write_proc\n");
	unsigned long val = simple_strtoul(buffer, 0, 10);
	
	if (val)
		{
			__gpio_as_output(NETWORKLED_IO);
			__gpio_clear_pin(NETWORKLED_IO);  //wjx 2007.12.13 up eth0 led
		}
	else
		{
			__gpio_as_output(NETWORKLED_IO);
			__gpio_set_pin(NETWORKLED_IO);		//wjx 2007.12.13 down eth0 led
		}
	return count;
}

//wjx 2008.3.31
static int capslock_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long val = simple_strtoul(buffer, 0, 10);
	
	if (val)
		{
			__gpio_as_output(CAPSLOCKLED_IO);
			__gpio_clear_pin(CAPSLOCKLED_IO);  //wjx 2007.12.13 up eth0 led
		}
	else
		{
			__gpio_as_output(CAPSLOCKLED_IO);
			__gpio_set_pin(CAPSLOCKLED_IO);		//wjx 2007.12.13 down eth0 led
		}
	return count;
}

static int numlock_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long val = simple_strtoul(buffer, 0, 10);
	
	if (val)
		{
			__gpio_as_output(NUMLOCKLED_IO);
			__gpio_clear_pin(NUMLOCKLED_IO);  //wjx 2007.12.13 up eth0 led
		}
	else
		{
			__gpio_as_output(NUMLOCKLED_IO);
			__gpio_set_pin(NUMLOCKLED_IO);		//wjx 2007.12.13 down eth0 led
		}
	return count;
}

static int lcd_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	extern  void minipc_bl_set_intensity(int n);
	unsigned long val = simple_strtoul(buffer, 0, 10);
	if (val <= 300)
		{
			minipc_bl_set_intensity(val);  //set the lcd backlight		
		}
	return count;
}

/* wjx test read mcu i2c */
static int mcu_read_proc (char *page, char **start, off_t off,
						  int count, int *eof, void *data)
{
	int len = 0;
	int nr;
	unsigned char battery_status = 0;
	
	i2c_open();
	nr = i2c_read(0x50>>1, &battery_status, 0xdb, 1);	   // battery adc
	i2c_close();
	
	len += sprintf (page+len, "nr: %d\n", nr);
	len += sprintf(page+len, "status: %u\n", battery_status);  
	
	return len;
}

/*control the internal wifi's power. wey 2008.5.28*/
static int iwifi_write_proc (struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long val = simple_strtoul(buffer, 0, 10);
	
	if (val)
		{
			__gpio_as_output(INTERNAL_WIFI_IO);
			__gpio_set_pin(INTERNAL_WIFI_IO);  
		}
	else
		{
			__gpio_as_output(INTERNAL_WIFI_IO);
			__gpio_clear_pin(INTERNAL_WIFI_IO);		
		}
	return count; 
}

#endif

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *entry;

	/* create /proc/jz */
	proc_jz_root = proc_mkdir("jz", 0);

	/* create /proc/jz/emc */
	entry = create_proc_entry("emc", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = emc_read_proc;
		entry->write_proc = NULL;
		entry->data = NULL;
	}

	/* create /proc/jz/pmc */
	entry = create_proc_entry("pmc", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = pmc_read_proc;
		entry->write_proc = pmc_write_proc;
		entry->data = NULL;
	}

	/* create /proc/jz/cgm */
	entry = create_proc_entry("cgm", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = cgm_read_proc;
		entry->write_proc = cgm_write_proc;
		entry->data = NULL;
	}

	/* create /proc/jz/wdt */
	entry = create_proc_entry("wdt", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = wdt_read_proc;
		entry->write_proc = wdt_write_proc;
		entry->data = NULL;
	}

#if CONFIG_JZ4730_MINIPC
	/* wjx 2007.12.10 battery   */	
	entry = create_proc_entry("battery", 0644, proc_jz_root);
	if (entry) {
		__gpio_as_input(BATTERY_IO);
		entry->read_proc = battery_read_proc;
		entry->write_proc = NULL;		
		entry->data = NULL;
	}	
	
	/* wjx 2007.12.10 eth0 network led   */
	entry = create_proc_entry("eth0_led", 0644, proc_jz_root);
	if (entry) {
		__gpio_as_output(NETWORKLED_IO);
		__gpio_set_pin(NETWORKLED_IO);	
		entry->read_proc = NULL;		
		entry->write_proc = eth0_write_proc;
		entry->data = NULL;
	}		

	/* wjx 2008.3.31 capslock led   */
	entry = create_proc_entry("capslock_led", 0644, proc_jz_root);
	if (entry) {
		__gpio_as_output(CAPSLOCKLED_IO);
		__gpio_set_pin(CAPSLOCKLED_IO);	
		/*edwin for test*/
		__gpio_as_output(95);
		__gpio_set_pin(95);
		/**/
		entry->read_proc = NULL;		
		entry->write_proc = capslock_write_proc;
		entry->data = NULL;
	}		

	/* wjx 2008.3.31 numlock led   */
	entry = create_proc_entry("numlock_led", 0644, proc_jz_root);
	if (entry) {
		__gpio_as_output(NUMLOCKLED_IO);
		__gpio_set_pin(NUMLOCKLED_IO);		
		entry->read_proc = NULL;		
		entry->write_proc = numlock_write_proc;
		entry->data = NULL;
	}		

	/* wjx 2007.12.14 lcd backlight   */
	
	entry = create_proc_entry("backlight", 0644, proc_jz_root);
	if (entry) {	
		entry->write_proc = lcd_write_proc;
		entry->read_proc = NULL;		
		entry->data = NULL;
	}			
	/* wjx 2008.1.29 mcu read battery status   */
	
	entry = create_proc_entry("battery-level", 0644, proc_jz_root);
	if (entry) {	
		entry->write_proc = NULL;
		entry->read_proc = mcu_read_proc;		
		entry->data = NULL;
	}			
		
	/*wey 2008.5.28 control the internal wifi*/
	entry = create_proc_entry("wifi-power", 0644, proc_jz_root);
	if (entry) {
		__gpio_as_output(INTERNAL_WIFI_IO);
		__gpio_clear_pin(INTERNAL_WIFI_IO);//by default turn off its power.wey 2008.5.28
		entry->read_proc =  NULL;
		entry->write_proc = iwifi_write_proc;		
		entry->data = NULL;
	}
#endif
	
	/* PWM */
	jz_pwm_proc_init();

	return 0;
}

__initcall(jz_proc_init);
