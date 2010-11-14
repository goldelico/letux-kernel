/*
 *    Copyright (c) 2010 Grant Erickson <marathon96@gmail.com>
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    version 2 as published by the Free Software Foundation.
 *
 *    Description:
 *      This file is defines platform-specific configuration data for
 *      the OMAP generic PWM platform driver.
 */

#ifndef _OMAP2_PWM_H
#define _OMAP2_PWM_H

/**
 * struct omap2_pwm_platform_config - OMAP platform-specific data for PWMs
 * @timer_id: the OMAP dual-mode timer ID.
 * @polarity: the polarity (active-high or -low) of the PWM.
 *
 * This identifies the OMAP dual-mode timer (dmtimer) that will be bound
 * to the PWM.
 */
struct omap2_pwm_platform_config {
	int timer_id;
	bool polarity;
};

#endif /* _OMAP2_PWM_H */
