/*
 *  arch/armplat-mxc/mvf_time.c
 *
 *  based on linux/arch/arm/kernel/smp_twd.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/seq_file.h>

#include <asm/sched_clock.h>
#include <asm/hardware/gic.h>
#include <mach/hardware.h>

/* global timer registers */
#define GT_COUNT_LOWER		0x0
#define GT_COUNT_UPPER		0x4
#define GT_CNTRL		0x8
#define GT_CNTRL_AOUT_INC	(1 << 3)
#define GT_CNTRL_INT_EN		(1 << 2)
#define GT_CNTRL_CMP_EN		(1 << 1)
#define GT_CNTRL_TIMER_EN	(1 << 0)
#define GT_INT_STAT		0xC
#define GT_INT_STAT_EFLG	(1 << 0)
#define GT_CMP_LOWER		0x10
#define GT_CMP_UPPER		0x14
#define GT_AUTO_INC		0x18

/* set up by the platform code */
void __iomem *timer_base;
static unsigned long timer_rate;
static struct clocksource clocksource_mvf;
static struct clock_event_device clockevent_mvf;

static DEFINE_CLOCK_DATA(cd);

static cycle_t mvf_clock_source_read(struct clocksource *c)
{
	cycle_t upper, val;

	while (1) {
		upper = __raw_readl(timer_base + GT_COUNT_UPPER);
		val = __raw_readl(timer_base + GT_COUNT_LOWER);
		if (upper == __raw_readl(timer_base + GT_COUNT_UPPER)) {
			val |= upper << 32;
			break;
		}
	}

	return val;
}

static void notrace mvf_update_sched_clock(void)
{
	cycle_t cyc = mvf_clock_source_read(&clocksource_mvf);
	update_sched_clock(&cd, cyc, (u32)~0);
}

static struct clocksource clocksource_mvf = {
	.name		= "mvf_timer",
	.rating		= 350,
	.read		= mvf_clock_source_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void mvf_timer_clear_int(void)
{
	if (__raw_readl(timer_base + GT_INT_STAT)) {
		__raw_writel(GT_INT_STAT_EFLG, timer_base + GT_INT_STAT);
	}
}

static void mvf_set_mode(enum clock_event_mode mode,
			struct clock_event_device *clk)
{
	unsigned long ctrl;
	cycle_t cmp;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cmp = mvf_clock_source_read(&clocksource_mvf) + (timer_rate / HZ);
		__raw_writel((u32)(cmp & 0xFFFFFFFF), timer_base + GT_CMP_LOWER);
		__raw_writel((u32)(cmp >> 32), timer_base + GT_CMP_UPPER);
		__raw_writel(timer_rate / HZ, timer_base + GT_AUTO_INC);
		/* timer load already set up */
		ctrl = GT_CNTRL_TIMER_EN | GT_CNTRL_CMP_EN | GT_CNTRL_INT_EN |
			GT_CNTRL_AOUT_INC;
		mvf_timer_clear_int();
		gic_enable_ppi(clk->irq);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl = GT_CNTRL_CMP_EN | GT_CNTRL_INT_EN;
		mvf_timer_clear_int();
		gic_enable_ppi(clk->irq);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
		gic_disable_ppi(clk->irq);
	}

	__raw_writel(ctrl, timer_base + GT_CNTRL);
}

static int mvf_set_next_event(unsigned long evt,
			struct clock_event_device *unused)
{
	unsigned long ctrl = __raw_readl(timer_base + GT_CNTRL);
	cycle_t cmp = mvf_clock_source_read(&clocksource_mvf);

	ctrl |= GT_CNTRL_TIMER_EN;
	cmp += (cycle_t)evt;

	__raw_writel((u32)(cmp & 0xFFFFFFFF), timer_base + GT_CMP_LOWER);
	__raw_writel((u32)(cmp >> 32), timer_base + GT_CMP_UPPER);
	__raw_writel(ctrl, timer_base + GT_CNTRL);

	return 0;
}

asmlinkage void __exception_irq_entry do_global_timer(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	int cpu = smp_processor_id();
	struct clock_event_device *evt = &clockevent_mvf;
	u32 reg;

	reg = __raw_readl(timer_base + GT_INT_STAT);
	if (reg) {
		mvf_timer_clear_int();
		__inc_irq_stat(cpu, global_timer_irqs);
		evt->event_handler(evt);
	}

	set_irq_regs(old_regs);
	
}

void show_global_timer_irqs(struct seq_file *p, int prec)
{
	unsigned int cpu;

	seq_printf(p, "%*s: ", prec, "GLOB");

	for_each_present_cpu(cpu)
		seq_printf(p, "%10u ", __get_irq_stat(cpu, global_timer_irqs));

	seq_printf(p, " Global timer interrupts\n");
}

static struct clock_event_device clockevent_mvf = {
	.name		= "mvf_timer",
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= mvf_set_mode,
	.set_next_event	= mvf_set_next_event,
	.rating		= 350,
};

/*
 * Setup the global clock events for a CPU.
 */
void __init mvf_timer_init(struct clk *timer_clk, void __iomem *base, int irq)
{
	clk_enable(timer_clk);

	timer_base = base;

	/*
	 * init timer
	 */
	/* stop timer */
	__raw_writel(0, timer_base + GT_CNTRL);
	/* clear timer count */
	__raw_writel(0, timer_base + GT_COUNT_LOWER);
	__raw_writel(0, timer_base + GT_COUNT_UPPER);
	/* clear timer int */
	mvf_timer_clear_int();
	/* clear timer cmp val */
	__raw_writel(0, timer_base + GT_CMP_LOWER);
	__raw_writel(0, timer_base + GT_CMP_UPPER);
	/* clear auto increment val */
	__raw_writel(0, timer_base + GT_AUTO_INC);

	timer_rate = clk_get_rate(timer_clk);
	init_sched_clock(&cd, mvf_update_sched_clock, 64, timer_rate);
	clocksource_register_hz(&clocksource_mvf, timer_rate);
	clockevent_mvf.irq = irq;
	clockevent_mvf.cpumask = cpumask_of(0);
	clockevents_config_and_register(&clockevent_mvf, timer_rate,
					0xf, 0xffffffff);

	/* Make sure our local interrupt controller has this enabled */
	gic_enable_ppi(irq);
}
