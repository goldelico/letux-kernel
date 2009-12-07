/*
 * omap-abe.h
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Contact: Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __OMAP_MCPDM_H__
#define __OMAP_MCPDM_H__

#define OMAP_ABE_MM_DAI			0
#define OMAP_ABE_TONES_DL_DAI		1
#define OMAP_ABE_VOICE_DAI		2
#define OMAP_ABE_DIG_UPLINK_DAI		3
#define OMAP_ABE_VIB_DAI		4

extern struct snd_soc_dai omap_abe_dai[];

#endif	/* End of __OMAP_MCPDM_H__ */
