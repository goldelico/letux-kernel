/*
 * arch/arm/plat-s3c/include/mach/cpu.h
 *
 *  S3C cpu type detection
 *
 *  Copyright (C) 2008 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * Derived from OMAP cpu.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_ARCH_S3C_CPU_H
#define __ASM_ARCH_S3C_CPU_H

extern unsigned int system_rev;

/*
 * cpu_is_s3c24xx():	True for s3c2400, s3c2410, s3c2440 and so on
 * cpu_is_s3c241x():	True fro s3c2410, s3c2412
 * cpu_is_s3c244x():	True fro s3c2440, s3c2442, s3c2443
 * cpu_is_s3c64xx():	True for s3c6400, s3c6410
 */
#define GET_S3C_CLASS	((system_rev >> 24) & 0xff)

#define IS_S3C_CLASS(class, id)						\
static inline int is_s3c ##class (void)					\
{									\
	return (GET_S3C_CLASS == (id)) ? 1 : 0;				\
}

#define GET_S3C_SUBCLASS	((system_rev >> 20) & 0xfff)

#define IS_S3C_SUBCLASS(subclass, id)					\
static inline int is_s3c ##subclass (void)				\
{									\
	return (GET_S3C_SUBCLASS == (id)) ? 1 : 0;			\
}

IS_S3C_CLASS(24xx, 0x24)
IS_S3C_CLASS(64xx, 0x64)

IS_S3C_SUBCLASS(241x, 0x241)
IS_S3C_SUBCLASS(244x, 0x244)

#define cpu_is_s3c24xx()		0
#define cpu_is_s3c241x()		0
#define cpu_is_s3c244x()		0
#define cpu_is_s3c64xx()		0

#if defined(CONFIG_ARCH_S3C2410)
# undef  cpu_is_s3c24xx
# undef  cpu_is_s3c241x
# undef  cpu_is_s3c244x
# define cpu_is_s3c24xx()		is_s3c24xx()
# define cpu_is_s3c241x()		is_s3c241x()
# define cpu_is_s3c244x()		is_s3c244x()
#endif

#if defined(CONFIG_ARCH_S3C64XX)
# undef  cpu_is_s3c64xx
# define cpu_is_s3c64xx()		is_s3c64xx()
#endif

/*
 * Macros to detect individual cpu types.
 * cpu_is_s3c2410():	True for s3c2410
 * cpu_is_s3c2440():	True for s3c2440
 * cpu_is_s3c6400():	True for s3c6400
 * cpu_is_s3c6410():	True for s3c6410
 *
 * Exception:
 * Store Revision A to 1
 * s3c2410a -> s3c2411
 * s3c2440a -> s3c2441
 */

#define GET_S3C_TYPE	((system_rev >> 16) & 0xffff)

#define IS_S3C_TYPE(type, id)						\
static inline int is_s3c ##type (void)					\
{									\
	return (GET_S3C_TYPE == (id)) ? 1 : 0;				\
}

IS_S3C_TYPE(2400, 0x2400)
IS_S3C_TYPE(2410, 0x2410)
IS_S3C_TYPE(2410a, 0x2411)
IS_S3C_TYPE(2412, 0x2412)
IS_S3C_TYPE(2440, 0x2440)
IS_S3C_TYPE(2440a, 0x2441)
IS_S3C_TYPE(2442, 0x2442)
IS_S3C_TYPE(2443, 0x2443)
IS_S3C_TYPE(6400, 0x6400)
IS_S3C_TYPE(6410, 0x6410)

#define cpu_is_s3c2400()		0
#define cpu_is_s3c2410()		0
#define cpu_is_s3c2410a()		0
#define cpu_is_s3c2412()		0
#define cpu_is_s3c2440()		0
#define cpu_is_s3c2440a()		0
#define cpu_is_s3c2442()		0
#define cpu_is_s3c2443()		0
#define cpu_is_s3c6400()		0
#define cpu_is_s3c6410()		0

#if defined(CONFIG_ARCH_S3C2410)
# undef  cpu_is_s3c2400
# define cpu_is_s3c2400()		is_s3c2400()
#endif

#if defined(CONFIG_CPU_S3C2410)
# undef  cpu_is_s3c2410
# undef  cpu_is_s3c2410a
# define cpu_is_s3c2410()		is_s3c2410()
# define cpu_is_s3c2410a()		is_s3c2410a()
#endif

#if defined(CONFIG_CPU_S3C2412)
# undef  cpu_is_s3c2412
# define cpu_is_s3c2412()		is_s3c2412()
#endif

#if defined(CONFIG_CPU_S3C2440)
# undef  cpu_is_s3c2440
# undef  cpu_is_s3c2440a
# define cpu_is_s3c2440()		is_s3c2440()
# define cpu_is_s3c2440a()		is_s3c2440a()
#endif

#if defined(CONFIG_CPU_S3C2442)
# undef  cpu_is_s3c2442
# define cpu_is_s3c2442()		is_s3c2442()
#endif

#if defined(CONFIG_CPU_S3C2443)
# undef  cpu_is_s3c2443
# define cpu_is_s3c2443()		is_s3c2443()
#endif

#if defined(CONFIG_ARCH_S3C64XX)
# undef  cpu_is_s3c6400
# undef  cpu_is_s3c6410
# define cpu_is_s3c6400()		is_s3c6400()
# define cpu_is_s3c6410()		is_s3c6410()
#endif

#endif
