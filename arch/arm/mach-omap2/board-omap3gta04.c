/* Minimal board file for those bit which cannot be managed
 * by devicetree yet.
 */

DT_MACHINE_START(OMAP3_DT, "GTA04 enhanced Generic OMAP3 (Flattened Device Tree)")
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3430_init_early,
	.init_irq	= omap_intc_of_init,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= omap_generic_init,
	.init_late	= omap3_init_late,
	.init_time	= omap3_sync32k_timer_init,
	.dt_compat	= omap3_boards_compat,
	.restart	= omap3xxx_restart,
MACHINE_END
