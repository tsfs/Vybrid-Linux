/*
 * drivers/char/pitimer.h
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

typedef enum {
	PIT0,
	PIT1,
	PIT2,
	PIT3,
	PIT4,
	PIT5,
	PIT6,
	PIT7,
	PIT_AVAILABLE_CHANNEL
} pit_channel;


int pit_alloc_timer( pit_channel ch);
int pit_free_timer(int timer_handle);
int pit_enable_timer( int timer_handle);
int pit_disable_timer( int timer_handle);
int pit_read_counter(int timer_handle, unsigned long *counter);
int pit_param_set( int timer_handle, unsigned long load_val, void (*event_handler)(int));
