/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Generic GPIO card-detect helper header
 *
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 */

#ifndef MMC_SLOT_GPIO_H
#define MMC_SLOT_GPIO_H

#include <linux/interrupt.h>
#include <linux/types.h>

struct mmc_host;

int mmc_gpio_get_ro(struct mmc_host *host);
int mmc_gpio_get_cd(struct mmc_host *host);
void mmc_gpio_set_cd_irq(struct mmc_host *host, int irq);

void mmc_gpio_set_rs(struct mmc_host *host, int state);
int mmc_gpio_request_rs(struct mmc_host *host, unsigned int gpio, int inv);
void mmc_gpio_free_rs(struct mmc_host *host);

int mmc_gpiod_request_cd(struct mmc_host *host, const char *con_id,
			 unsigned int idx, bool override_active_level,
			 unsigned int debounce);
int mmc_gpiod_request_ro(struct mmc_host *host, const char *con_id,
			 unsigned int idx, unsigned int debounce);
int mmc_gpiod_set_cd_config(struct mmc_host *host, unsigned long config);
void mmc_gpio_set_cd_isr(struct mmc_host *host, irq_handler_t isr);
int mmc_gpio_set_cd_wake(struct mmc_host *host, bool on);
void mmc_gpiod_request_cd_irq(struct mmc_host *host);
bool mmc_can_gpio_cd(struct mmc_host *host);
bool mmc_can_gpio_ro(struct mmc_host *host);

#endif
