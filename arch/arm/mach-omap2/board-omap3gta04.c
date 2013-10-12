/* Minimal board file for those bit which cannot be managed
 * by devicetree yet.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include "common.h"
#include "common-board-devices.h"
#include <video/omapdss.h>
#include <asm/mach/arch.h>
#include "mux.h"

static struct omap_dss_board_info omap3_dss_data = {
	.default_display_name = "lcd",
};

static struct omap_board_mux board_mux[] __initdata = {
	/* Enable GPT11_PWM_EVT instead of GPIO-57 */
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE3),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

void __init omap_generic_init(void);
static void __init omap_gta04_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_generic_init();
	omap_display_init(&omap3_dss_data);
}

static const char *gta04_compat[] __initdata = {
	"ti,omap3-gta04",
	NULL,
};

DT_MACHINE_START(OMAP3_DT, "GTA04 enhanced Generic OMAP3 (Flattened Device Tree)")
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3430_init_early,
	.init_irq	= omap_intc_of_init,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= omap_gta04_init,
	.init_late	= omap3_init_late,
	.init_time	= omap3_sync32k_timer_init,
	.dt_compat	= gta04_compat,
	.restart	= omap3xxx_restart,
MACHINE_END
