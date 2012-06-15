/*
 *  based on linux/arch/arm/plat-mxc/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2006-2007 Pavel Pisa (ppisa@pikron.com)
 *  Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <asm/sched_clock.h>
#include <asm/mach/time.h>
#include <mach/common.h>

/*
 * VF Timer: using Periodic Interrupt Timer(PIT)
 */
/* PIT clock has a frequency of 50MHz(20ns/clock) */
#define BASE_CLOCK     50000000
#define TIMER_CH       0

/* defines common for VF All chanels */
#define PIT_MCR        0x0000
#define PIT_MCR_MDIS   (1<<1)

#define PIT_LTMR64H    0x00e0
#define PIT_LTMR64L    0x00e4

/* */
#define PIT_LDVAL(x)   0x100+(x<<4)
#define PIT_CVAL(x)   0x104+(x<<4)
#define PIT_TCTRL(x)   0x108+(x<<4)
#define PIT_TCTRL_CHN  (1<<2) /* Chain Mode */
#define PIT_TCTRL_TIE  (1<<1) /* Timer Interrupt Enable */
#define PIT_TCTRL_TEN  (1<<0) /* Timer Enable */
#define PIT_TFLG(x)    0x10C+(x<<4)
#define PIT_TFLG_TIF   (1<<0)



static struct clock_event_device clockevent_mvf;
static enum clock_event_mode clockevent_mode = CLOCK_EVT_MODE_UNUSED;

static void __iomem *timer_base;

static inline void gpt_irq_disable(void)
{
	unsigned int tmp;

	tmp = __raw_readl(timer_base + PIT_TCTRL(TIMER_CH));
	__raw_writel(tmp & ~PIT_TCTRL_TIE , timer_base + PIT_TCTRL(TIMER_CH));
}

static inline void gpt_irq_enable(void)
{
	unsigned int tmp;

	tmp = __raw_readl(timer_base + PIT_TCTRL(TIMER_CH));
	__raw_writel(tmp | PIT_TCTRL_TIE , timer_base + PIT_TCTRL(TIMER_CH));
}

static void gpt_irq_acknowledge(void)
{
	
	__raw_writel(__raw_readl(timer_base + PIT_TFLG(TIMER_CH)), 
				 timer_base + PIT_TFLG(TIMER_CH));
}

static void __iomem *sched_clock_reg;

static DEFINE_CLOCK_DATA(cd);
unsigned long long notrace sched_clock(void)
{
	cycle_t cyc = sched_clock_reg ? __raw_readl(sched_clock_reg) : 0;

	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

static void notrace mvf_update_sched_clock(void)
{
	cycle_t cyc = sched_clock_reg ? __raw_readl(sched_clock_reg) : 0;
	update_sched_clock(&cd, cyc, (u32)~0);
}

static int __init mvf_clocksource_init(struct clk *timer_clk)
{
	unsigned int c = clk_get_rate(timer_clk);
	void __iomem *reg = timer_base + PIT_CVAL(TIMER_CH);

	sched_clock_reg = reg;

	init_sched_clock(&cd, mvf_update_sched_clock, 32, c);
	return clocksource_mmio_init(reg, "mvf_timer1", c, 200, 32,
			clocksource_mmio_readl_up);
}



static void mvf_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	unsigned long flags;

	/*
	 * The timer interrupt generation is disabled at least
	 * for enough time to call mvf_set_next_event()
	 */
	/* DEBUG */ printk("DBG: %s[%d]: Start Mode = %x, jiffies = %ld\n",__func__,__LINE__, mode,jiffies);

	local_irq_save(flags);

	/* Disable interrupt in GPT module */
	gpt_irq_disable();

	if (mode != clockevent_mode) {
		/* Clear pending interrupt */
		gpt_irq_acknowledge();
	}
	
#ifdef DEBUG
	printk(KERN_INFO "mvf_set_mode: changing mode from %s to %s\n",
		clock_event_mode_label[clockevent_mode],
		clock_event_mode_label[mode]);
#endif /* DEBUG */

	/* Remember timer mode */
	clockevent_mode = mode;
	local_irq_restore(flags);
	
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk(KERN_ERR"mvf_set_mode: Periodic mode is not "
			   "supported for MVF\n");
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/*
	 * Do not put overhead of interrupt enable/disable into
	 * mvf_set_next_event(), the core has about 4 minutes
	 * to call mvf_set_next_event() or shutdown clock after
	 * mode switching
	 */
		local_irq_save(flags);
		gpt_irq_enable();
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		/* Left event sources disabled, no more interrupts appear */
		break;
	}
	/* DEBUG */ printk("DBG: %s[%d]: Start jiffies = %ld\n",__func__,__LINE__, jiffies);
}

/*
 * IRQ handler for the timer
 */
#if 1
#define COUNT_UP_VAL	660000
static unsigned long total_cnt = 0;
static unsigned long long old_jiff = 0;
#endif
static irqreturn_t mvf_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_mvf;
	uint32_t tstat;
	unsigned long reg;

	tstat = __raw_readl(timer_base + PIT_TFLG(TIMER_CH));
	if ( tstat ) {
		__raw_writel(tstat, timer_base + PIT_TFLG(TIMER_CH));
		gpt_irq_acknowledge();
		evt->event_handler(evt);
#if 1
		reg = __raw_readl(timer_base + PIT_LDVAL(TIMER_CH));
		total_cnt += reg;
		if (total_cnt >= COUNT_UP_VAL) {
			total_cnt = 0;
			jiffies++;
			//printk("jiffies = %ld\n",jiffies);
		}
#endif
		return IRQ_HANDLED;
	}
	//	/* DEBUG */ printk("DBG: %s[%d]: NONE Exit\n",__func__,__LINE__);
	return IRQ_NONE;
}

static int mvf_set_next_event(unsigned long evt,
			      struct clock_event_device *unused)
{
#if 1
	unsigned long tcmp;
	tcmp = evt;
	/* STOP Time */
	__raw_writel(__raw_readl(timer_base + PIT_TCTRL(TIMER_CH)) 
				 & ~(PIT_TCTRL_TEN), 
				 timer_base + PIT_TCTRL(TIMER_CH));
	__raw_writel(tcmp, timer_base + PIT_LDVAL(TIMER_CH));
	/* Start Timer */
	__raw_writel(__raw_readl(timer_base + PIT_TCTRL(TIMER_CH)) 
				 | (PIT_TCTRL_TEN), 
				 timer_base + PIT_TCTRL(TIMER_CH));
#else
	unsigned long tcmp,cval;
	unsigned long oval,nval;
	tcmp = evt;
	cval = __raw_readl(timer_base + PIT_CVAL(TIMER_CH));
	oval = __raw_readl(timer_base + PIT_LDVAL(TIMER_CH));
	/* STOP Time */
	__raw_writel(__raw_readl(timer_base + PIT_TCTRL(TIMER_CH)) 
				 & ~(PIT_TCTRL_TEN), 
				 timer_base + PIT_TCTRL(TIMER_CH));
	__raw_writel(tcmp, timer_base + PIT_LDVAL(TIMER_CH));
	/* Start Timer */
	__raw_writel(__raw_readl(timer_base + PIT_TCTRL(TIMER_CH)) 
				 | (PIT_TCTRL_TEN), 
				 timer_base + PIT_TCTRL(TIMER_CH));
	
	__raw_writel(tcmp+cval, timer_base + PIT_LDVAL(TIMER_CH));
	nval = __raw_readl(timer_base + PIT_LDVAL(TIMER_CH));
	tcmp = __raw_readl(timer_base + PIT_CVAL(TIMER_CH));
	//	printk("DEBUG: KATSU: evt=%lud, oldval = %lud, nval = %lud,cval = %lud,tcmp = %lud\n",evt,oval,nval,cval,tcmp);
#endif
	return 0;

}



static struct irqaction mvf_timer_irq = {
	.name		= "MVF Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= mvf_timer_interrupt,
};

static struct clock_event_device clockevent_mvf = {
	.name		= "mvf_timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_mode	= mvf_set_mode,
	.set_next_event	= mvf_set_next_event,
	.rating		= 200,
};

static int __init mvf_clockevent_init(struct clk *timer_clk)
{
	unsigned int c = clk_get_rate(timer_clk);

	clockevent_mvf.mult = div_sc(c, NSEC_PER_SEC,
					clockevent_mvf.shift);
	clockevent_mvf.max_delta_ns =
			clockevent_delta2ns(0xfffffffe, &clockevent_mvf);
	clockevent_mvf.min_delta_ns =
			clockevent_delta2ns(0xff, &clockevent_mvf);

	clockevent_mvf.cpumask = cpumask_of(0);

	clockevents_register_device(&clockevent_mvf);

	return 0;
}


void __init mvf_timer_init(struct clk *timer_clk, void __iomem *base, int irq)
{

#if 1 /* Clock is fix to 50MHz */
	clk_enable(timer_clk);
#endif
	printk("PIT base = 0x%08lx",(unsigned long)base);
	timer_base = base;

	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */

	//	__raw_writel(PIT_MCR_MDIS, timer_base + PIT_MCR); /* Stop PIT */
	__raw_writel(0, timer_base + PIT_MCR); /* Stop PIT */

	/* init and register the timer to the framework */
	mvf_clocksource_init(timer_clk);
	mvf_clockevent_init(timer_clk);

	/* Make irqs happen */
	setup_irq(irq, &mvf_timer_irq);
}
