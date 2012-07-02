/*
 * drivers/char/ftimer.h
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


struct mvf_ftm_request{
	unsigned long	 clocksource;
	unsigned long	 divider;
	unsigned short	 start;
	unsigned short	 end;
};

typedef enum {
	FMT0,
	FMT1,
	FMT_AVAILABLE_CHANNEL
} ftm_channel;


//	clock source
#define FTM_PARAM_CLK_NOCLOCK		0x00
#define FTM_PARAM_CLK_SYSTEMCLOCK	0x01
#define FTM_PARAM_CLK_FIXEDFREQ		0x02
#define FTM_PARAM_CLK_EXTERNAL		0x03

//	divider
#define FTM_PARAM_DIV_BY_1			0x00
#define FTM_PARAM_DIV_BY_2			0x01
#define FTM_PARAM_DIV_BY_4			0x02
#define FTM_PARAM_DIV_BY_8			0x03
#define FTM_PARAM_DIV_BY_16			0x04
#define FTM_PARAM_DIV_BY_32			0x05
#define FTM_PARAM_DIV_BY_64			0x06
#define FTM_PARAM_DIV_BY_128		0x07

int ftm_alloc_timer( ftm_channel ch);
int ftm_free_timer( int timer_handle);
int ftm_enable_timer( int timer_handle);
int ftm_disable_timer( int timer_handle);
int ftm_read_counter( int timer_handle, unsigned long *counter);
int ftm_param_set( int timer_handle, struct mvf_ftm_request *req, void (*event_handler)(int));

