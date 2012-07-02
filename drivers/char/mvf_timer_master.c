/*
 * drivers/char/mvf_timer_master.c
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

struct timer_master_control{
	int probe_cnt;
	int	is_opened[ TIMER_MASTER_MAX_TIMER];
#define TIMER_AVAILABLE	0
#define TIMER_BUSY		1
	struct platform_device *plat_timerdev[ TIMER_MASTER_MAX_TIMER];
};

static struct timer_master_control master_control;

static int timer_master_is_valid_handle(int index)
{
	int val;

	val = 0;
	//	error check
	if ( index > TIMER_MASTER_ENUM_AVAILABLE) val = -EINVAL;

	return val;
}

static int timer_master_is_opened(int index)
{
	int	ret;

	ret = timer_master_is_valid_handle(index); 
	if ( ret < 0) return ret;

	return master_control.is_opened[index];
}

#if 0
static int timer_master_set_eventhandler( int timer_handle, void (*event_handler)(void))
{
	master_control.event_handler[ timer_handle] = event_handler;
	return 0;
}
#endif


static  struct platform_device *timer_master_get_pdev(int index)
{
	return master_control.plat_timerdev[ index];
}

static void timer_master_register_platform(struct platform_device *plat)
{
	int index = master_control.probe_cnt;

	if (master_control.probe_cnt >= TIMER_MASTER_MAX_TIMER){
		printk(KERN_WARNING"Timer regist count overflow\n");
	}else{
		master_control.plat_timerdev[ index] = plat;
		master_control.probe_cnt++;
	}
}

static int timer_master_alloc_timer( int index)
{
	int	i,ret;

	ret = timer_master_is_valid_handle( index); 
	if ( ret < 0) return ret;

	ret = -EBUSY;
	if ( index == TIMER_MASTER_ENUM_AVAILABLE){
		for ( i = 0; i < TIMER_MASTER_MAX_TIMER; i ++){
			if ( master_control.is_opened[ i] == TIMER_AVAILABLE){
				master_control.is_opened[ i] = TIMER_BUSY;
				ret = i;
			}
		}
	}else
	if ( master_control.is_opened[ index] == TIMER_AVAILABLE){
		master_control.is_opened[ index] = TIMER_BUSY;
		ret = index;
	}

	return ret;
}

static int timer_master_free(int index)
{
	int ret;

	ret = timer_master_is_valid_handle(index); 
	if ( ret < 0) return ret;

	master_control.is_opened[ index] = TIMER_AVAILABLE;

	return 0;
}

