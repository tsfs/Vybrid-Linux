/*
 * drivers/char/pitimer_reg.h
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.  All rights reserved.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#define PIT_MCR_OFFSET			0x00
#define 	PIT_MCR_FRZ		0x01
#define 	PIT_MCR_MDIS	0x02

#define PIT_LTMR64H_OFFSET		0x04
#define PIT_LTMR64L_OFFSET		0x08


#define PIT_LDVAL_OFFSET(x)		( ( x * 0x10) + 0x100)
#define PIT_CVAL_OFFSET(x)		( ( x * 0x10) + 0x104)
#define PIT_TCTRL_OFFSET(x)		( ( x * 0x10) + 0x108)
#define 	PIT_TCTR_TEN	0x01
#define 	PIT_TCTR_TIE	0x02
#define 	PIT_TCTR_CHN	0x04

#define PIT_TFLG_OFFSET(x)		( ( x * 0x10) + 0x10c)
#define 	PIT_TFLG_TIF	0x01
