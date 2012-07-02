/*
 * drivers/char/lptimer_reg.h
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

#define LPTMR_CSR_OFFSET	0x0000
#define 	LPTMR_CSR_TEN	0x01
#define 	LPTMR_CSR_TMS	0x02
#define 	LPTMR_CSR_TFC	0x04
#define 	LPTMR_CSR_TPP	0x08
#define 	LPTMR_CSR_TIE	0x40
#define 	LPTMR_CSR_TCF	0x80


#define LPTMR_PSR_OFFSET	0x0004 
#define LPTMR_CMR_OFFSET	0x0008
#define LPTMR_CNR_OFFSET	0x000C

