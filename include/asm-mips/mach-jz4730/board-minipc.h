/*
 *  linux/include/asm-mips/mach-jz4730/board-minipc.h
 *  JZ4730-based XIP MINIPC
 *  "Author": <ard@kwaak.net>
 *
 *  Derived from skytone celinux:board-pmpv1.h and jzl:board-pmp.h
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_MINIPC_H__
#define __ASM_JZ4730_MINIPC_H__

/*====================================================================== 
 * EXTAL frequency
 */

/*
 * These are defaults actually
 */

#define JZ_EXTAL		3686400	
#define JZ_EXTAL2		32768		/* EXTAL2: 32.768 KHz */


/*====================================================================== 
 * GPIO
 */
#define GPIO_PW_I         97
#define GPIO_PW_O         66
#define GPIO_LED_EN       92
#define GPIO_DISP_OFF_N   93
#define GPIO_PWM0         94
#define GPIO_RTC_IRQ      96
#define GPIO_USB_CLK_EN   29
#define GPIO_CHARG_STAT   125
#define GPIO_TS_PENIRQ    98
#define GPIO_UDC_HOTPLUG  86

/*====================================================================== 
 * MMC/SD
 */
/*
 * From celinux:include/asm/jz4730-boards/pmpv1.h
 */
#define MSC_WP_PIN         66
#define MSC_POWEREN_PIN    21
#define MSC_HOTPLUG_PIN    64
#define MSC_HOTPLUG_IRQ    (IRQ_GPIO_0 + MSC_HOTPLUG_PIN)

/* enable slot power */
#define __msc_init_io()				\
do {						\
      	__gpio_as_input(MSC_WP_PIN);		\
      	__gpio_as_output(MSC_POWEREN_PIN);	\
} while (0)

/* enable slot power */
#define __msc_enable_power()			\
do {						\
      	__gpio_clear_pin(MSC_POWEREN_PIN);	\
} while (0)

/* disable slot power */
#define __msc_disable_power()			\
do {						\
      	__gpio_set_pin(MSC_POWEREN_PIN);	\
} while (0)

/* detect card insertion or not */
#define __msc_card_detected(slot)				\
({							\
	int ret;					\
	if (slot == 0) {				\
	      	__gpio_mask_irq(MSC_HOTPLUG_PIN);	\
	      	__gpio_as_input(MSC_HOTPLUG_PIN);	\
	     	ret = __gpio_get_pin(MSC_HOTPLUG_PIN);	\
		__gpio_unmask_irq(MSC_HOTPLUG_PIN);	\
	}						\
	else {						\
     		ret = 1;				\
	}						\
	ret = !ret;					\
	ret;						\
})

#endif /* __ASM_JZ4730_MINIPC_H__ */
