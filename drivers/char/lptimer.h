/*
 * drivers/char/lptimer.h
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.  All rights reserved.
 *
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

struct mvf_lpt_request{
	unsigned long	compare_value;
	unsigned short	timer_mode;

	unsigned short	pulse_pin_polarity;
	unsigned short	pulse_pin_select;

	unsigned short	prs_clock_sel;
	unsigned short	prs_bypass;
	unsigned short	prs_value;
};


#define LPT_PARAM_TM_TIMECOUNTER	0
#define LPT_PARAM_TM_PULSECOUNTER	1

#define LPT_PARAM_PPP_ACTIVEHIGH	0
#define LPT_PARAM_PPP_ACTIVELOW		1

#define LPT_PARAM_PPS_INPUT0		0
#define LPT_PARAM_PPS_INPUT1		1
#define LPT_PARAM_PPS_INPUT2		2
#define LPT_PARAM_PPS_INPUT3		3


#define LPT_PARAM_PCS_CLOCK0		0
#define LPT_PARAM_PCS_CLOCK1		1
#define LPT_PARAM_PCS_CLOCK2		2
#define LPT_PARAM_PCS_CLOCK3		3

#define LPT_PARAM_PB_GF_ENABLE		0
#define LPT_PARAM_PB_GF_BYPASS		1

#define LPT_PARAM_PV_DIV2_NA				0
#define LPT_PARAM_PV_DIV4_RISE2				1
#define LPT_PARAM_PV_DIV8_RISE4				2
#define LPT_PARAM_PV_DIV16_RISE8			3
#define LPT_PARAM_PV_DIV32_RISE16			4
#define LPT_PARAM_PV_DIV64_RISE32			5
#define LPT_PARAM_PV_DIV128_RISE64			6
#define LPT_PARAM_PV_DIV256_RISE128			7
#define LPT_PARAM_PV_DIV512_RISE256			8
#define LPT_PARAM_PV_DIV1024_RISE512		9
#define LPT_PARAM_PV_DIV2048_RISE1024		10
#define LPT_PARAM_PV_DIV4096_RISE2048		11
#define LPT_PARAM_PV_DIV8192_RISE4096		12
#define LPT_PARAM_PV_DIV16384_RISE8192		13
#define LPT_PARAM_PV_DIV32768_RISE16384		14
#define LPT_PARAM_PV_DIV65536_RISE32768		15

int lpt_alloc_timer( void);
int lpt_free_timer(int timer_handle);
int lpt_enable_timer( int timer_handle);
int lpt_disable_timer( int timer_handle);
int lpt_read_counter(int timer_handle, unsigned long *counter);
int lpt_param_set( int timer_handle, struct mvf_lpt_request *req, void (*event_handler)(void));
