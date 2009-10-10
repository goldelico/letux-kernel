/*
 * ALSA SoC ABE-TWL6030 codec driver
 *
 * Author:      Misael Lopez Cruz <x0052729@ti.com>
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

#ifndef __ABE_TWL6030_H__
#define __ABE_TWL6030_H__

extern struct snd_soc_dai abe_dai[];
extern struct snd_soc_codec_device soc_codec_dev_abe_twl6030;

struct twl6030_setup_data {
	void (*codec_enable)(int enable);
};

#endif /* End of __ABE_TWL6030_H__ */
