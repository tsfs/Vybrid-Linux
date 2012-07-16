/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __INCLUDE_DCU_H__
#define __INCLUDE_DCU_H__

#include <linux/types.h>
#include <linux/device.h>
#include <mach/clock.h>
#include <linux/clkdev.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>


#define MVF_DCU_MAX_NUM	 1

/* Globals */
struct dcu_irq_node {
	irqreturn_t(*handler) (int, void *);	/*!< the ISR */
	const char *name;	/*!< device associated with the interrupt */
	void *dev_id;		/*!< some unique information for the ISR */
	__u32 flags;		/*!< not used */
};

struct dcu_soc {
	bool online;
	bool display_configured;

	/*clk*/
	struct clk *dcu_clk;
	struct clk pixel_clk;

	/*irq*/
	int irq_generic;
	struct dcu_irq_node irq_list[DCU_IRQ_COUNT];

	/*reg*/
	u32 *dcu_base_reg;
	u32 *clut_tile_mem_base;
	u32 *gamma_r_mem_base;
	u32 *gamma_g_mem_base;
	u32 *gamma_b_mem_base;
	u32 *cursor_mem_base;

	struct device *platform_dev;
	struct device *dcu_cdev;

	uint32_t dcu_layer_transfer_complete_irq;
	struct completion layer_transfer_complete;

	/*use count*/
	atomic_t dcu_use_count;
	atomic_t layer_use_count;

	struct mutex mutex_lock;
	spinlock_t spin_lock;
};

static inline u32 dcu_read(struct dcu_soc *dcu, unsigned offset)
{
	return readl(dcu->dcu_base_reg + offset);
}

static inline void dcu_write(struct dcu_soc *dcu,
		u32 value, unsigned offset)
{
	writel(value, dcu->dcu_base_reg + offset);
}

#endif				/* __INCLUDE_DCU_H__ */
