/*
 * based on arch/arm/mach-mx6/cpu_op-mx6.c
 */

/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include "cpu_op-day.h"

extern struct cpu_op *(*get_cpu_op)(int *op);
//extern struct dvfs_op *(*get_dvfs_core_op)(int *wp);
extern void (*set_num_cpu_op)(int num);
extern u32 arm_max_freq;
static int num_cpu_op;

static struct cpu_op mvf_cpu_op[] = { //FIXME
	{
	 .pll_rate = 528000000,
	 .cpu_rate = 452000000,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
};

#if 0 //FIXME: need dvfs support ?
static struct dvfs_op dvfs_core_setpoint[] = {
	{33, 14, 33, 10, 128, 0x08},   /* 800MHz */
	{26, 8, 33, 100, 200, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08} };   /* 200MHz*/

static struct dvfs_op *mvf_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}
#endif

struct cpu_op *mvf_get_cpu_op(int *op)
{
	*op =  num_cpu_op = ARRAY_SIZE(mvf_cpu_op);
	return mvf_cpu_op;
}

void mvf_set_num_cpu_op(int num)
{
	num_cpu_op = num;
	return;
}

void mvf_cpu_op_init(void)
{
	get_cpu_op = mvf_get_cpu_op;
	set_num_cpu_op = mvf_set_num_cpu_op;

	//get_dvfs_core_op = mvf_get_dvfs_core_table;
}

