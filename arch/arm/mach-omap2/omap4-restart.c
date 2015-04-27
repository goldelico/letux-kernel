/*
 * omap4-restart.c - Common to OMAP4 and OMAP5
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/reboot.h>
#include "prminst44xx.h"
#include "omap4-sar-layout.h"
#include "soc.h"
#include "common.h"

static void __iomem *sar_base;

/**
 * omap44xx_restart - trigger a software restart of the SoC
 * @mode: the "reboot mode", see arch/arm/kernel/{setup,process}.c
 * @cmd: passed from the userspace program rebooting the system (if provided)
 *
 * Resets the SoC.  For @cmd, see the 'reboot' syscall in
 * kernel/sys.c.  No return value.
 */
void omap44xx_restart(enum reboot_mode mode, const char *cmd)
{
	int offset = 0;
	char *reason = "normal";
	if (cpu_is_omap44xx())
		offset = OMAP4_REBOOT_REASON_OFFSET;
	else if (soc_is_omap54xx())
		offset = OMAP5_REBOOT_REASON_OFFSET;
	else if (soc_is_dra7xx())
		offset = DRA7XX_REBOOT_REASON_OFFSET;
	else
		WARN("undefined chip, %s", __func__);

	if (cmd != NULL && *(char *)cmd)
		reason = (char *)cmd;

	if (!sar_base)
		sar_base = omap4_get_sar_ram_base();

	strlcpy(sar_base + offset, reason, min((int)strlen(reason) + 1,
				OMAP_REBOOT_REASON_SIZE - 1));

	omap4_prminst_global_warm_sw_reset(); /* never returns */
	while (1)
		;
}
