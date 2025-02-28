/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_MXC_TIMEX_H__
#define __ASM_ARCH_MXC_TIMEX_H__

#if defined CONFIG_ARCH_MX1
#define CLOCK_TICK_RATE		16000000
#elif defined CONFIG_ARCH_MX2
#define CLOCK_TICK_RATE		13300000
#elif defined CONFIG_ARCH_MX3
#define CLOCK_TICK_RATE		16625000
#elif defined CONFIG_ARCH_MX25
#define CLOCK_TICK_RATE		16000000
#elif defined CONFIG_ARCH_MX5
#define CLOCK_TICK_RATE		8000000
#elif defined CONFIG_ARCH_MX6
#define CLOCK_TICK_RATE		8000000
#elif defined CONFIG_ARCH_MVF
#define CLOCK_TICK_RATE		8000000	//FIXME
#endif

#endif				/* __ASM_ARCH_MXC_TIMEX_H__ */
