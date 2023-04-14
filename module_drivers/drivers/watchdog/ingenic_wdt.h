/*
 * ingenic watchdog_device register definition
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *
 */
#ifndef __INGENIC_WDT_H__
#define __INGENIC_WDT_H__
/*
 * watchdog registers offset address definition
 */
#define INGENIC_REG_WDT_TIMER_DATA     (0x0)
#define INGENIC_REG_WDT_COUNTER_ENABLE (0x4)
#define INGENIC_REG_WDT_TIMER_COUNTER  (0x8)
#define INGENIC_REG_WDT_TIMER_CONTROL  (0xC)
#define INGENIC_REG_WDT_TIMER_STOP_RD  (0x1C)
#define INGENIC_REG_WDT_TIMER_STOP_SET (0x2C)
#define INGENIC_REG_WDT_TIMER_STOP_CLR (0x3C)
#define INGENIC_REG_WDT_TIMER_FLAG_RD  (0x20)
#define INGENIC_REG_WDT_TIMER_FLAG_SET (0x24)
#define INGENIC_REG_WDT_TIMER_FLAG_CLR (0x28)
#define INGENIC_REG_WDT_TIMER_MASK_RD  (0x30)
#define INGENIC_REG_WDT_TIMER_MASK_SET (0x34)
#define INGENIC_REG_WDT_TIMER_MASK_CLR (0x38)

#define INGENIC_WDT_OFFSET_16	  (16)
#define INGENIC_WDT_FULL_MAX	  (0xffff)

/*Watchdog timer stop Register(TCU_STOP_RD/SET/CLR,0x1C/0x2C/0x3C) bit*/
#define INGENIC_WDT_TIMER_STOP	BIT(16)

/*Watchdog timer flag Register(TCU_FLAG_RD/SET/CLR,0x20/0x24/0x28) bit*/
#define INGENIC_WDT_TIMER_FLAG	BIT(24)

/*Watchdog timer mask Register(TCU_MASK_RD/SET/CLR,0x30/0x34/0x38) bit*/
#define INGENIC_WDT_TIMER_MASK	BIT(24)

/*Watchdog Control Register(WDT_CONTROL,0x0C)*/
#define INGENIC_WDT_CLOCK_RTC	BIT(1)
#define INGENIC_WDT_CLOCK_CLRZ	BIT(10)

#define INGENIC_WDT_CLOCK_DIV_SHIFT   (3)
#define INGENIC_WDT_CLOCK_DIV_MAX  (0x7)
#define INGENIC_WDT_CLOCK_DIV_1    (0 << INGENIC_WDT_CLOCK_DIV_SHIFT)
#define INGENIC_WDT_CLOCK_DIV_4    (1 << INGENIC_WDT_CLOCK_DIV_SHIFT)
#define INGENIC_WDT_CLOCK_DIV_16   (2 << INGENIC_WDT_CLOCK_DIV_SHIFT)
#define INGENIC_WDT_CLOCK_DIV_64   (3 << INGENIC_WDT_CLOCK_DIV_SHIFT)
#define INGENIC_WDT_CLOCK_DIV_256  (4 << INGENIC_WDT_CLOCK_DIV_SHIFT)
#define INGENIC_WDT_CLOCK_DIV_1024 (5 << INGENIC_WDT_CLOCK_DIV_SHIFT)

#define DEFAULT_HEARTBEAT (15)
#define MAX_HEARTBEAT     (2047 * 1000)

/*ingenic watchdog drvdata struct*/
struct ingenic_wdt_drvdata {
	int irq;
	struct work_struct work;
	struct watchdog_device wdt;
	void __iomem *base;
	struct clk *wdt_clk;
	struct clk *tcu_clk;
	unsigned int timeout;
	struct notifier_block restart_handler;
};

#endif /* __INGENIC_WDT_H__*/
