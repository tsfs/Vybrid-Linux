/*
 * based on arch/arm/mach-mx6/clock.c
 */

/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/regulator/consumer.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include "crm_regs.h"
#include "cpu_op-mvf.h"
#include "regs-anadig.h"

#ifdef CONFIG_CLK_DEBUG
#define __INIT_CLK_DEBUG(n)	.name = #n,
#else
#define __INIT_CLK_DEBUG(n)
#endif

extern u32 arm_max_freq;
extern int mxc_jtag_enabled;
extern struct regulator *cpu_regulator;
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int lp_high_freq;
extern int lp_med_freq;
#if 0 //FIXME
extern int vf6xx_revision(void);
#endif

void __iomem *apll_base;
static struct clk pll1_sys_main_clk;
static struct clk pll1_pfd1;
static struct clk pll1_pfd2;
static struct clk pll1_pfd3;
static struct clk pll1_pfd4;
static struct clk pll2_528_main_clk;
static struct clk pll2_pfd1;
static struct clk pll2_pfd2;
static struct clk pll2_pfd3;
static struct clk pll2_pfd4;
static struct clk pll3_480_usb1_main_clk;
static struct clk pll3_pfd1;
static struct clk pll3_pfd2;
static struct clk pll3_pfd3;
static struct clk pll3_pfd4;
static struct clk pll4_audio_main_clk;
static struct clk pll5_enet_main_clk;
static struct clk pll6_video_main_clk;
static struct clk pll_480_usb2_main_clk;

static struct cpu_op *cpu_op_tbl;
#if 0
static int cpu_op_nr;
#endif

#define SPIN_DELAY	1200000 /* in nanoseconds */ //FIXME

#define AUDIO_VIDEO_MIN_CLK_FREQ	650000000
#define AUDIO_VIDEO_MAX_CLK_FREQ	1300000000

/* We need to check the exp status again after timer expiration,
 * as there might be interrupt coming between the first time exp
 * and the time reading, then the time reading may be several ms
 * after the exp checking due to the irq handle, so we need to
 * check it to make sure the exp return the right value after
 * timer expiration. */
#define WAIT(exp, timeout) \
({ \
	struct timespec nstimeofday; \
	struct timespec curtime; \
	int result = 1; \
	getnstimeofday(&nstimeofday); \
	while (!(exp)) { \
		getnstimeofday(&curtime); \
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > (timeout)) { \
			if (!(exp)) \
				result = 0; \
			break; \
		} \
	} \
	result; \
})

/* External clock values passed-in by the board code */
static unsigned long internal_high_reference, internal_low_reference;
static unsigned long external_high_reference, external_low_reference;
static unsigned long anaclk_1_reference, audio_ext_clk_reference;
static unsigned long enet_ts_clk_reference, usb_clk_reference;

/* For MX 6DL/S, Video PLL may be used by synchronous display devices,
 * such as HDMI or LVDS, and also by the EPDC.  If EPDC is in use,
 * it must use the Video PLL to achieve the clock frequencies it needs.
 * So if EPDC is in use, the "epdc" string should be added to kernel
 * parameters, in order to set the EPDC parent clock to the Video PLL.
 * This will have an impact on the behavior of HDMI and LVDS.
 */
int epdc_enabled;
static int __init epdc_setup(char *__unused)
{
	epdc_enabled = 1;
	return 1;
}
__setup("epdc", epdc_setup);

static void __calc_pre_post_dividers(u32 max_podf, u32 div, u32 *pre, u32 *post)
{
	//FIXME
	u32 min_pre, temp_pre, old_err, err;

	/* Some of the podfs are 3 bits while others are 6 bits.
	  * Handle both cases here.
	  */
	if (div >= 512 && (max_podf == 64)) {
		/* For pre = 3bits and podf = 6 bits, max divider is 512. */
		*pre = 8;
		*post = 64;
	} else if (div >= 64 && (max_podf == 8)) {
		/* For pre = 3bits and podf = 3 bits, max divider is 64. */
		*pre = 8;
		*post = 8;
	} else if (div >= 8) {
		/* Find the minimum pre-divider for a max podf */
		if (max_podf == 64)
			min_pre = (div - 1) / (1 << 6) + 1;
		else
			min_pre = (div - 1) / (1 << 3) + 1;
		old_err = 8;
		/* Now loop through to find the max pre-divider. */
		for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
			err = div % temp_pre;
			if (err == 0) {
				*pre = temp_pre;
				break;
			}
			err = temp_pre - err;
			if (err < old_err) {
				old_err = err;
				*pre = temp_pre;
			}
		}
		*post = (div + *pre - 1) / *pre;
	} else if (div < 8) {
		*pre = div;
		*post = 1;
	}
}

static int _clk_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= MXC_CCM_CCGRx_MOD_ON << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq++;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq++;

	return 0;
}

static int _clk_force_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= MXC_CCM_CCGRx_MOD_FORCE_ON << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq++;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq++;

	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq--;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq--;
}

static void _clk_disable_inwait(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= MXC_CCM_CCGRx_MOD_IDLE << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}

/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else
		BUG();

	return 0;
}

static inline void __iomem *_get_pll_base(struct clk *pll)
{
	if (pll == &pll1_sys_main_clk)
		return PLL1_SYS_BASE_ADDR;
	else if (pll == &pll2_528_main_clk)
		return PLL2_528_BASE_ADDR;
	else if (pll == &pll3_480_usb1_main_clk)
		return PLL3_480_USB1_BASE_ADDR;
	else if (pll == &pll4_audio_main_clk)
		return PLL4_AUDIO_BASE_ADDR;
	else if (pll == &pll5_enet_main_clk)
		return PLL5_ENET_BASE_ADDR;
	else if (pll == &pll6_video_main_clk)
		return PLL6_VIDEO_BASE_ADDR;
	else if (pll == &pll_480_usb2_main_clk)
		return ANADIG_USB2_PLL_CTRL;
	else
		BUG();
	return NULL;
}


/*
 * For the 6-to-1 muxed input clock
 */
static inline u32 _get_mux6(struct clk *parent, struct clk *m0, struct clk *m1,
			    struct clk *m2, struct clk *m3, struct clk *m4,
			    struct clk *m5)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else if (parent == m4)
		return 4;
	else if (parent == m5)
		return 5;
	else
		BUG();

	return 0;
}

static unsigned long get_high_reference_clock_rate(struct clk *clk)
{
	return internal_high_reference;
}

static unsigned long get_low_reference_clock_rate(struct clk *clk)
{
	return internal_low_reference;
}

static unsigned long get_external_high_reference_clock_rate(struct clk *clk)
{
	return external_high_reference;
}

static unsigned long get_external_low_reference_clock_rate(struct clk *clk)
{
	return external_low_reference;
}

static unsigned long _clk_anaclk_1_get_rate(struct clk *clk)
{
	return anaclk_1_reference;
}

static int _clk_anaclk_1_set_rate(struct clk *clk, unsigned long rate)
{
	anaclk_1_reference = rate;
	return 0;
}

static unsigned long _clk_audio_ext_get_rate(struct clk *clk)
{
	return audio_ext_clk_reference;
}

static int _clk_audio_ext_set_rate(struct clk *clk, unsigned long rate)
{
	audio_ext_clk_reference = rate;
	return 0;
}

static unsigned long _clk_enet_ext_get_rate(struct clk *clk)
{
	return 50000000;
}

static unsigned long _clk_enet_ts_get_rate(struct clk *clk)
{
	return enet_ts_clk_reference;
}

static int _clk_enet_ts_set_rate(struct clk *clk, unsigned long rate)
{
	enet_ts_clk_reference = rate;
	return 0;
}

static unsigned long _clk_usb_clk_get_rate(struct clk *clk)
{
	return 60000000;
}

/* Internal high frequency clock */
static struct clk ckih_clk = {
	__INIT_CLK_DEBUG(ckih_clk)
	.get_rate = get_high_reference_clock_rate,
};

/* External high frequency clock */
static struct clk osch_clk = {
	__INIT_CLK_DEBUG(osch_clk)
	.get_rate = get_external_high_reference_clock_rate,
};

/* Internal low frequency (128kHz) clock */
static struct clk ckil_clk = {
	__INIT_CLK_DEBUG(ckil_clk)
	.get_rate = get_low_reference_clock_rate,
};

/* External low frequency (32kHz) clock */
static struct clk oscl_clk = {
	__INIT_CLK_DEBUG(oscl_clk)
	.get_rate = get_external_low_reference_clock_rate,
};

static struct clk anaclk_1 = {
	__INIT_CLK_DEBUG(anaclk_1)
	.get_rate = _clk_anaclk_1_get_rate,
	.set_rate = _clk_anaclk_1_set_rate,
};

static struct clk audio_ext = {
	__INIT_CLK_DEBUG(audio_ext)
	.get_rate = _clk_audio_ext_get_rate,
	.set_rate = _clk_audio_ext_set_rate,
};

static struct clk enet_ext = {
	__INIT_CLK_DEBUG(enet_ext)
	.get_rate = _clk_enet_ext_get_rate,
};

static struct clk enet_ts = {
	__INIT_CLK_DEBUG(enet_ts)
	.get_rate = _clk_enet_ts_get_rate,
	.set_rate = _clk_enet_ts_set_rate,
};

static struct clk usb_clk = {
	__INIT_CLK_DEBUG(usb_clk)
	.get_rate = _clk_usb_clk_get_rate,
};

static unsigned long pfd_round_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;

	tmp = (u64)clk_get_rate(clk->parent) * 18;
	tmp += rate/2;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 12 ? 12 : frac;
	frac = frac > 35 ? 35 : frac;
	tmp = (u64)clk_get_rate(clk->parent) * 18;
	do_div(tmp, frac);
	return tmp;
}

static unsigned long pfd_get_rate(struct clk *clk)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk_get_rate(clk->parent) * 18;

	frac = (__raw_readl(clk->enable_reg) >> clk->enable_shift) &
			ANADIG_PFD_FRAC_MASK;

	do_div(tmp, frac);

	return tmp;
}

static int pfd_set_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk_get_rate(clk->parent) * 18;

	/* Round up the divider so that we don't set a rate
	  * higher than what is requested. */
	tmp += rate/2;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 12 ? 12 : frac;
	frac = frac > 35 ? 35 : frac;
	/* set clk frac bits */
	__raw_writel(frac << clk->enable_shift,
			(int)clk->enable_reg);

	return 0;
}

static int _clk_pfd_enable(struct clk *clk)
{
	u32 reg;

	/* clear clk gate bit */
	reg = __raw_readl((int)clk->enable_reg);
	__raw_writel(reg & ~(1 << (clk->enable_shift + ANADIG_PFD_CLKGATE)),
			(int)clk->enable_reg);

	/* enable PLLm_PFDn */
	reg = __raw_readl(MXC_CCM_CCSR);
	if (clk == &pll1_pfd1) {
		reg |= MXC_CCM_CCSR_PLL1_PFD1_EN;
	} else if (clk == &pll1_pfd2) {
		reg |= MXC_CCM_CCSR_PLL1_PFD2_EN;
	} else if (clk == &pll1_pfd3) {
		reg |= MXC_CCM_CCSR_PLL1_PFD3_EN;
	} else if (clk == &pll1_pfd4) {
		reg |= MXC_CCM_CCSR_PLL1_PFD4_EN;
	} else if (clk == &pll2_pfd1) {
		reg |= MXC_CCM_CCSR_PLL2_PFD1_EN;
	} else if (clk == &pll2_pfd2) {
		reg |= MXC_CCM_CCSR_PLL2_PFD2_EN;
	} else if (clk == &pll2_pfd3) {
		reg |= MXC_CCM_CCSR_PLL2_PFD3_EN;
	} else if (clk == &pll2_pfd4) {
		reg |= MXC_CCM_CCSR_PLL2_PFD4_EN;
	} else if (clk == &pll3_pfd1) {
		reg |= MXC_CCM_CCSR_PLL3_PFD1_EN;
	} else if (clk == &pll3_pfd2) {
		reg |= MXC_CCM_CCSR_PLL3_PFD2_EN;
	} else if (clk == &pll3_pfd3) {
		reg |= MXC_CCM_CCSR_PLL3_PFD3_EN;
	} else if (clk == &pll3_pfd4) {
		reg |= MXC_CCM_CCSR_PLL3_PFD4_EN;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static void _clk_pfd_disable(struct clk *clk)
{
	u32 reg;

	/* disable PLLm_PFDn */
	reg = __raw_readl(MXC_CCM_CCSR);
	if (clk == &pll1_pfd1) {
		reg &= ~MXC_CCM_CCSR_PLL1_PFD1_EN;
	} else if (clk == &pll1_pfd2) {
		reg &= ~MXC_CCM_CCSR_PLL1_PFD2_EN;
	} else if (clk == &pll1_pfd3) {
		reg &= ~MXC_CCM_CCSR_PLL1_PFD3_EN;
	} else if (clk == &pll1_pfd4) {
		reg &= ~MXC_CCM_CCSR_PLL1_PFD4_EN;
	} else if (clk == &pll2_pfd1) {
		reg &= ~MXC_CCM_CCSR_PLL2_PFD1_EN;
	} else if (clk == &pll2_pfd2) {
		reg &= ~MXC_CCM_CCSR_PLL2_PFD2_EN;
	} else if (clk == &pll2_pfd3) {
		reg &= ~MXC_CCM_CCSR_PLL2_PFD3_EN;
	} else if (clk == &pll2_pfd4) {
		reg &= ~MXC_CCM_CCSR_PLL2_PFD4_EN;
	} else if (clk == &pll3_pfd1) {
		reg &= ~MXC_CCM_CCSR_PLL3_PFD1_EN;
	} else if (clk == &pll3_pfd2) {
		reg &= ~MXC_CCM_CCSR_PLL3_PFD2_EN;
	} else if (clk == &pll3_pfd3) {
		reg &= ~MXC_CCM_CCSR_PLL3_PFD3_EN;
	} else if (clk == &pll3_pfd4) {
		reg &= ~MXC_CCM_CCSR_PLL3_PFD4_EN;
	} else {
		return;
	}
	__raw_writel(reg, MXC_CCM_CCSR);

	/* set clk gate bit */
	reg = __raw_readl((int)clk->enable_reg);
	__raw_writel(reg | (1 << (clk->enable_shift + ANADIG_PFD_CLKGATE)),
			(int)clk->enable_reg);
}

static int _clk_pll_enable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg &= ~ANADIG_PLL_BYPASS;
	reg &= ~ANADIG_PLL_POWER_DOWN;

	/* The 480MHz PLLs have the opposite definition for power bit. */
	if (clk == &pll3_480_usb1_main_clk || clk == &pll_480_usb2_main_clk)
		reg |= ANADIG_PLL_POWER_DOWN;

	__raw_writel(reg, pllbase);

#if 0 //FIXME
	/* It will power on pll3 */
	if (clk == &pll3_480_usb1_main_clk)
		__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0, apll_base + HW_ANADIG_ANA_MISC2_CLR);
#endif

	/* Wait for PLL to lock */
	if (!WAIT(__raw_readl(pllbase) & ANADIG_PLL_LOCK,
				SPIN_DELAY))
		panic("pll enable failed\n");

	/* Enable the PLL output now*/
	reg = __raw_readl(pllbase);
	reg |= ANADIG_PLL_ENABLE;
	__raw_writel(reg, pllbase);

	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg |= ANADIG_PLL_BYPASS;
	reg &= ~ANADIG_PLL_ENABLE;

	__raw_writel(reg, pllbase);

#if 0 //FIXME
	/*
	 * It will power off PLL3's power, it is the TO1.1 fix
	 * Please see TKT064178 for detail.
	 */
	if (clk == &pll3_480_usb1_main_clk)
		__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0, apll_base + HW_ANADIG_ANA_MISC2_SET);
#endif
}

static unsigned long  _clk_pll1_main_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL1_SYS_BASE_ADDR) & ANADIG_PLL_SYS_DIV_SELECT;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;

	else
		val = clk_get_rate(clk->parent) * 20;

	return val;
}

static int _clk_pll1_main_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg, div;

	switch (rate) {
	case 528000000:
		div = 1;
		break;
	case 480000000:
		div = 0;
		break;
	default:
		return -EINVAL;
	}

	/* Update div */
	reg = __raw_readl(PLL1_SYS_BASE_ADDR) & ~ANADIG_PLL_SYS_DIV_SELECT;
	reg |= div;
	__raw_writel(reg, PLL1_SYS_BASE_ADDR);

	// FIXME: need wait?
	/* Wait for PLL1 to lock */
	if (!WAIT(__raw_readl(PLL1_SYS_BASE_ADDR) & ANADIG_PLL_LOCK,
				SPIN_DELAY))
		panic("pll1 enable failed\n");

	return 0;
}

static struct clk pll1_sys_main_clk = {
	__INIT_CLK_DEBUG(pll1_sys_main_clk)
	.parent = &osch_clk,
	.get_rate = _clk_pll1_main_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = _clk_pll1_main_set_rate,
#endif
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
};

static struct clk pll1_pfd1 = {
	__INIT_CLK_DEBUG(pll1_pfd1_pll)
	.id = 0,
	.parent = &pll1_sys_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528_SYS,
	.enable_shift = ANADIG_PFD1_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
#endif
};

static struct clk pll1_pfd2 = {
	__INIT_CLK_DEBUG(pll1_pfd2)
	.parent = &pll1_sys_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528_SYS,
	.enable_shift = ANADIG_PFD2_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
#endif
};

static struct clk pll1_pfd3 = {
	__INIT_CLK_DEBUG(pll1_pfd3)
	.parent = &pll1_sys_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528_SYS,
	.enable_shift = ANADIG_PFD3_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
#endif
};

static struct clk pll1_pfd4 = {
	__INIT_CLK_DEBUG(pll1_pfd4)
	.parent = &pll1_sys_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528_SYS,
	.enable_shift = ANADIG_PFD4_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
#endif
};

static int _pll1_pfd_out_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_PLL1_PFD_CLK_SEL_MASK;
	int mux;

	mux = _get_mux6(parent, &pll1_sys_main_clk, &pll1_pfd1, &pll1_pfd2,
			&pll1_pfd3, &pll1_pfd4, NULL);
	reg |= mux;
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk pll1_pfd_out = {
	__INIT_CLK_DEBUG(pll1_pfd_out)
	.parent = &pll1_pfd2,
#if 0	//we do not support to change PLL1
	.set_parent = _pll1_pfd_out_set_parent,
#endif
};

static unsigned long _clk_pll1_pfd_out_div_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = ((cacrr & MXC_CCM_CACRR_PLL1_PFD_CLK_DIV_MASK) >>
		MXC_CCM_CACRR_PLL1_PFD_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

#if 0
static int _clk_pll1_pfd_out_div_set_rate(struct clk *clk, unsigned long rate)
{
	int i;
	u32 cacrr, div;
	u32 parent_rate;


	for (i = 0; i < cpu_op_nr; i++) {
		if (rate == cpu_op_tbl[i].cpu_rate)
			break;
	}
	if (i >= cpu_op_nr)
		return -EINVAL;

	if (cpu_op_tbl[i].pll_rate != clk_get_rate(&pll1_sys_main_clk) ||
	    cpu_op_tbl[i].cpu_rate != clk_get_rate(&pll1_pfd_out)) {
		/* Change the PLL1 rate. */
		pll1_sys_main_clk.set_rate(&pll1_sys_main_clk, cpu_op_tbl[i].pll_rate);
		pll1_pfd_out.set_parent(&pll1_pfd_out, &pll1_pfd2);
		pll1_pfd2.set_rate(&pll1_pfd2, cpu_op_tbl[i].cpu_rate);
	}

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;

	if ((parent_rate / div) > rate)
		div++;

	if (div > 4)
		return -1;

	cacrr = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_PLL1_PFD_CLK_DIV_MASK;
	cacrr |= (div -1) << MXC_CCM_CACRR_PLL1_PFD_CLK_DIV_OFFSET;
	__raw_writel(cacrr, MXC_CCM_CACRR);

	return 0;
}
#endif

static struct clk pll1_pfd_out_div = {
	__INIT_CLK_DEBUG(pll1_pfd_out_div)
	.parent = &pll1_pfd_out,
	.get_rate = _clk_pll1_pfd_out_div_get_rate,
#if 0	//we do not support to change PLL1
	.set_rate = _clk_pll1_pfd_out_div_set_rate,
#endif
};

static unsigned long _clk_pll2_main_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL2_528_BASE_ADDR) & ANADIG_PLL_528_DIV_SELECT;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;

	else
		val = clk_get_rate(clk->parent) * 20;

	return val;
}

static int _clk_pll2_main_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(PLL2_528_BASE_ADDR);
	reg &= ~ANADIG_PLL_528_DIV_SELECT;
	reg |= div;
	__raw_writel(reg, PLL2_528_BASE_ADDR);

	return 0;
}

static struct clk pll2_528_main_clk = {
	__INIT_CLK_DEBUG(pll2_528_main_clk)
	.parent = &osch_clk,
	.get_rate = _clk_pll2_main_get_rate,
	.set_rate = _clk_pll2_main_set_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
};

static struct clk pll2_pfd1 = {
	__INIT_CLK_DEBUG(pll2_pfd1)
	.parent = &pll2_528_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528,
	.enable_shift = ANADIG_PFD1_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll2_pfd2 = {
	__INIT_CLK_DEBUG(pll2_pfd2)
	.parent = &pll2_528_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528,
	.enable_shift = ANADIG_PFD2_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll2_pfd3 = {
	__INIT_CLK_DEBUG(pll2_pfd3)
	.parent = &pll2_528_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528,
	.enable_shift = ANADIG_PFD3_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll2_pfd4 = {
	__INIT_CLK_DEBUG(pll2_pfd4)
	.parent = &pll2_528_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_528,
	.enable_shift = ANADIG_PFD4_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.get_rate = pfd_get_rate,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static int _pll2_pfd_out_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_PLL2_PFD_CLK_SEL_MASK;

	if (parent == &pll2_528_main_clk) {
		reg |= 0x0;
	} else if (parent == &pll2_pfd1) {
		reg |= 0x1;
	} else if (parent == &pll2_pfd2) {
		reg |= 0x2;
	} else if (parent == &pll2_pfd3) {
		reg |= 0x3;
	} else if (parent == &pll2_pfd4) {
		reg |= 0x4;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk pll2_pfd_out = {
	__INIT_CLK_DEBUG(pll2_pfd_out)
	.parent = &pll2_pfd1,
	.set_parent = _pll2_pfd_out_set_parent,
};

static unsigned long _clk_pll3_480_usb1_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL3_480_USB1_BASE_ADDR)
		& ANADIG_PLL_480_DIV_SELECT_MASK;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;
	else
		val = clk_get_rate(clk->parent) * 20;
	return val;
}

static int _clk_pll3_480_usb1_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(PLL3_480_USB1_BASE_ADDR);
	reg &= ~ANADIG_PLL_480_DIV_SELECT_MASK;
	reg |= div;
	__raw_writel(reg, PLL3_480_USB1_BASE_ADDR);

	return 0;
}


/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_480_usb1_main_clk = {
	__INIT_CLK_DEBUG(pll3_480_usb1_main_clk)
	.parent = &osch_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll3_480_usb1_set_rate,
	.get_rate = _clk_pll3_480_usb1_get_rate,
};

static unsigned long _clk_pll3_div_get_rate(struct clk *clk)
{
	unsigned int div;

	div = (__raw_readl(MXC_CCM_CACRR) &~MXC_CCM_CACRR_PLL3_CLK_DIV) >>
		MXC_CCM_CACRR_PLL3_CLK_DIV_OFFSET;
	div += 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_pll3_div_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 2)
		return -1;

	reg = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_PLL3_CLK_DIV;
	reg |= div << MXC_CCM_CACRR_PLL3_CLK_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CACRR);

	return 0;
}

static struct clk pll3_div_clk = {
	__INIT_CLK_DEBUG(pll3_div_clk)
	.parent = &pll3_480_usb1_main_clk,
	.set_rate = _clk_pll3_div_set_rate,
	.get_rate = _clk_pll3_div_get_rate,
};

static struct clk pll3_pfd1 = {
	__INIT_CLK_DEBUG(pll3_pfd1)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_480_USB1,
	.enable_shift = ANADIG_PFD1_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll3_pfd2 = {
	__INIT_CLK_DEBUG(pll3_pfd2)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_480_USB1,
	.enable_shift = ANADIG_PFD2_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll3_pfd3 = {
	__INIT_CLK_DEBUG(pll3_pfd3)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_480_USB1,
	.enable_shift = ANADIG_PFD3_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll3_pfd4 = {
	__INIT_CLK_DEBUG(pll3_pfd4)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = (void *)ANADIG_PLL_PFD_480_USB1,
	.enable_shift = ANADIG_PFD4_FRAC_SHIFT,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static unsigned long  _clk_audio_video_get_rate(struct clk *clk)
{
	//FIXME: need TEST_DIV_SEL support?
	unsigned int div, mfn, mfd;
	unsigned long rate;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	void __iomem *pllbase;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL6_VIDEO_BASE_ADDR;

	div = __raw_readl(pllbase) & ANADIG_PLL_SYS_DIV_SELECT;
	mfn = __raw_readl(pllbase + PLL_NUM_DIV_OFFSET);
	mfd = __raw_readl(pllbase + PLL_DENOM_DIV_OFFSET);

	rate = (parent_rate * div) + ((parent_rate / mfd) * mfn);

	return rate;
}

static int _clk_audio_video_set_rate(struct clk *clk, unsigned long rate)
{
	//FIXME: need TEST_DIV_SEL support?
	unsigned int reg,  div;
	unsigned int mfn, mfd = 1000000;
	s64 temp64;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	void __iomem *pllbase;
	unsigned long pre_div_rate;

	if ((rate < AUDIO_VIDEO_MIN_CLK_FREQ) || (rate > AUDIO_VIDEO_MAX_CLK_FREQ))
		return -EINVAL;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL6_VIDEO_BASE_ADDR;

	pre_div_rate = rate;
	div = pre_div_rate / parent_rate;
	temp64 = (u64) (pre_div_rate - (div * parent_rate));
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	reg = __raw_readl(pllbase)
			& ~ANADIG_PLL_SYS_DIV_SELECT;
	reg |= div;
	__raw_writel(reg, pllbase);
	__raw_writel(mfn, pllbase + PLL_NUM_DIV_OFFSET);
	__raw_writel(mfd, pllbase + PLL_DENOM_DIV_OFFSET);

	return 0;
}

static unsigned long _clk_audio_video_round_rate(struct clk *clk,
						unsigned long rate)
{
	//FIXME: need TEST_DIV_SEL support?
	unsigned int div;
	unsigned int mfn, mfd = 1000000;
	s64 temp64;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	unsigned long pre_div_rate;
	unsigned long final_rate;

	if (rate < AUDIO_VIDEO_MIN_CLK_FREQ)
		return AUDIO_VIDEO_MIN_CLK_FREQ;

	if (rate > AUDIO_VIDEO_MAX_CLK_FREQ)
		return AUDIO_VIDEO_MAX_CLK_FREQ;

	pre_div_rate = rate;
	div = pre_div_rate / parent_rate;
	temp64 = (u64) (pre_div_rate - (div * parent_rate));
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	final_rate = (parent_rate * div) + ((parent_rate / mfd) * mfn);

	return final_rate;
}

static int _clk_audio_video_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;
	void __iomem *pllbase;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL6_VIDEO_BASE_ADDR;

	reg = __raw_readl(pllbase) & ~ANADIG_PLL_BYPASS_CLK_SRC;
	mux = _get_mux(parent, &osch_clk, &anaclk_1, NULL, NULL);
	reg |= mux << ANADIG_PLL_BYPASS_CLK_SRC;
	__raw_writel(reg, pllbase);

	/* Set anaclk_x as input */
	if (parent == &anaclk_1) {
		reg = __raw_readl(ANADIG_ANA_MISC1);
		reg |= (ANADIG_ANA_MISC1_LVDSCLK1_IBEN &
				~ANADIG_ANA_MISC1_LVDSCLK1_OBEN);
		__raw_writel(reg, ANADIG_ANA_MISC1);
	}

	return 0;
}

static struct clk pll4_audio_main_clk = {
	__INIT_CLK_DEBUG(pll4_audio_main_clk)
	.parent = &osch_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_audio_video_set_rate,
	.get_rate = _clk_audio_video_get_rate,
	.round_rate = _clk_audio_video_round_rate,
	.set_parent = _clk_audio_video_set_parent,
};

static struct clk pll6_video_main_clk = {
	__INIT_CLK_DEBUG(pll6_video_main_clk)
	.parent = &osch_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_audio_video_set_rate,
	.get_rate = _clk_audio_video_get_rate,
	.round_rate = _clk_audio_video_round_rate,
	.set_parent = _clk_audio_video_set_parent,
};

static unsigned long _clk_pll6_div_get_rate(struct clk *clk)
{
	unsigned int div;

	div = (__raw_readl(MXC_CCM_CACRR) &~MXC_CCM_CACRR_PLL6_CLK_DIV) >>
		MXC_CCM_CACRR_PLL6_CLK_DIV_OFFSET;
	div += 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_pll6_div_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 2)
		return -1;

	reg = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_PLL6_CLK_DIV;
	reg |= div << MXC_CCM_CACRR_PLL6_CLK_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CACRR);

	return 0;
}

static struct clk pll6_div_clk = {
	__INIT_CLK_DEBUG(pll6_div_clk)
	.parent = &pll6_video_main_clk,
	.set_rate = _clk_pll6_div_set_rate,
	.get_rate = _clk_pll6_div_get_rate,
};

static unsigned long _clk_pll_480_usb2_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(ANADIG_USB2_PLL_CTRL)
		& ANADIG_PLL_480_DIV_SELECT_MASK;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;
	else
		val = clk_get_rate(clk->parent) * 20;
	return val;
}

static int _clk_pll_480_usb2_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(ANADIG_USB2_PLL_CTRL);
	reg &= ~ANADIG_PLL_480_DIV_SELECT_MASK;
	reg |= div;
	__raw_writel(reg, ANADIG_USB2_PLL_CTRL);

	return 0;
}

static struct clk pll_480_usb2_main_clk = {
	__INIT_CLK_DEBUG(pll_480_usb2_main_clk)
	.parent = &osch_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll_480_usb2_set_rate,
	.get_rate = _clk_pll_480_usb2_get_rate,

};

static struct clk pll5_enet_main_clk = {
	__INIT_CLK_DEBUG(pll5_enet_main_clk)
	.parent = &osch_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
};

static int _clk_enet_enable(struct clk *clk)
{
	unsigned int reg;

	/* Enable ENET ref clock */
	reg = __raw_readl(PLL5_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_BYPASS;
	reg |= ANADIG_PLL_ENABLE;
	__raw_writel(reg, PLL5_ENET_BASE_ADDR);

	return 0;
}

static void _clk_enet_disable(struct clk *clk)
{
	unsigned int reg;

	/* Enable ENET ref clock */
	reg = __raw_readl(PLL5_ENET_BASE_ADDR);
	reg |= ANADIG_PLL_BYPASS;
	reg &= ~ANADIG_PLL_ENABLE;
	__raw_writel(reg, PLL5_ENET_BASE_ADDR);
}

static int _clk_enet_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg, div = 1;

	switch (rate) {
	case 25000000:
		div = 0;
		break;
	case 50000000:
		div = 1;
		break;
	case 100000000:
		div = 2;
		break;
	case 125000000:
		div = 3;
		break;
	default:
		return -EINVAL;
	}
	reg = __raw_readl(PLL5_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_ENET_DIV_SELECT_MASK;
	reg |= (div << ANADIG_PLL_ENET_DIV_SELECT_OFFSET);
	__raw_writel(reg, PLL5_ENET_BASE_ADDR);

	return 0;
}

static unsigned long _clk_enet_get_rate(struct clk *clk)
{
	unsigned int div;

	div = (__raw_readl(PLL5_ENET_BASE_ADDR))
		& ANADIG_PLL_ENET_DIV_SELECT_MASK;

	switch (div) {
	case 0:
		div = 20;
		break;
	case 1:
		div = 10;
		break;
	case 2:
		div = 5;
		break;
	case 3:
		div = 4;
		break;
	}

	return 500000000 / div;
}

static struct clk enet_clk = {
	__INIT_CLK_DEBUG(enet_clk)
	.id = 0,
	.parent = &pll5_enet_main_clk,
	.enable = _clk_enet_enable,
	.disable = _clk_enet_disable,
#if 0	//we do not support enet freq
	.set_rate = _clk_enet_set_rate,
#endif
	.get_rate = _clk_enet_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

unsigned long _clk_enet_div2_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 2;
}

static struct clk enet_div2_clk = {
	__INIT_CLK_DEBUG(enet_div2_clk)
	.parent = &enet_clk,
	.get_rate = _clk_enet_div2_get_rate,
};

static unsigned long _clk_sys_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = ((cacrr & MXC_CCM_CACRR_ARM_CLK_DIV_MASK) >>
		MXC_CCM_CACRR_ARM_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

#if 0
static int _clk_sys_set_rate(struct clk *clk, unsigned long rate)
{
	int i;
	u32 cacrr, div;
	u32 parent_rate;


	for (i = 0; i < cpu_op_nr; i++) {
		if (rate == cpu_op_tbl[i].cpu_rate)
			break;
	}
	if (i >= cpu_op_nr)
		return -EINVAL;

	if (cpu_op_tbl[i].pll_rate != clk_get_rate(&pll1_sys_main_clk) ||
	    cpu_op_tbl[i].cpu_rate != clk_get_rate(&pll1_pfd_out)) {
		/* Change the PLL1 rate. */
		pll1_sys_main_clk.set_rate(&pll1_sys_main_clk, cpu_op_tbl[i].pll_rate);
		pll1_pfd_out.set_parent(&pll1_pfd_out, &pll1_pfd2);
		pll1_pfd2.set_rate(&pll1_pfd2, cpu_op_tbl[i].cpu_rate);
	}

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;

	if ((parent_rate / div) > rate)
		div++;

	if (div > 8)
		return -1;

	cacrr = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_ARM_CLK_DIV_MASK;
	cacrr |= (div -1) << MXC_CCM_CACRR_ARM_CLK_DIV_OFFSET;
	__raw_writel(cacrr, MXC_CCM_CACRR);

	return 0;
}

static int _clk_sys_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux6(parent, &osch_clk, &oscl_clk, &pll2_pfd_out,
			&pll2_528_main_clk, &pll1_pfd_out, &pll3_480_usb1_main_clk);
	reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_SYS_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CCSR_SYS_CLK_SE_OFFSET;
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}
#endif

static struct clk sys_clk = {
	__INIT_CLK_DEBUG(sys_clk)
	.parent = &pll1_pfd_out,
#if 0 /* we do not support to change sys_clk */
	.set_rate = _clk_sys_set_rate,
#endif
	.get_rate = _clk_sys_get_rate,
#if 0 /* we do not support to change sys_clk */
	.set_parent = _clk_sys_set_parent,
#endif
};

static int _clk_ca5_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &sys_clk, &pll1_pfd_out_div, NULL, NULL);
	reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_CA5_CLK_SEL;
	reg |= mux << MXC_CCM_CCSR_CA5_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk cpu_clk = {
	__INIT_CLK_DEBUG(cpu_clk)
	.parent = &sys_clk,
#if 0	//we do not support to change ca5_clk
	.set_parent = _clk_ca5_set_parent,
#endif
};

static int _clk_ddrc_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll2_pfd2, &sys_clk, NULL, NULL);
	reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_DDRC_CLK_SEL;
	reg |= mux << MXC_CCM_CCSR_DDRC_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk ddrc_clk = {
	__INIT_CLK_DEBUG(ddrc_clk)
	.parent = &pll2_pfd2,
#if 0	//we do not support to change dram_clk
	.set_parent = _clk_ddrc_set_parent,
#endif
};

static unsigned long _clk_plat_bus_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = ((cacrr & MXC_CCM_CACRR_BUS_CLK_DIV_MASK) >>
		MXC_CCM_CACRR_BUS_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_plat_bus_set_rate(struct clk *clk, unsigned long rate)
{
	u32 cacrr, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 8)
		return -1;

	cacrr = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_BUS_CLK_DIV_MASK;
	cacrr |= (div -1) << MXC_CCM_CACRR_BUS_CLK_DIV_OFFSET;
	__raw_writel(cacrr, MXC_CCM_CACRR);

	return 0;
}

static struct clk plat_bus_clk = {
	__INIT_CLK_DEBUG(plat_bus_clk)
	.parent = &sys_clk,
	.get_rate = _clk_plat_bus_get_rate,
#if 0	//we do not support to change plat_bus_clk
	.set_rate = _clk_plat_bus_set_rate,
#endif
};

static struct clk cm4_clk = {
	__INIT_CLK_DEBUG(cm4_clk)
	.parent = &plat_bus_clk,
};

static unsigned long _clk_flex_bus_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = ((cacrr & MXC_CCM_CACRR_FLEX_CLK_DIV_MASK) >>
		MXC_CCM_CACRR_FLEX_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_flex_bus_set_rate(struct clk *clk, unsigned long rate)
{
	u32 cacrr, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 8)
		return -EINVAL;

	cacrr = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_FLEX_CLK_DIV_MASK;
	cacrr |= (div -1) << MXC_CCM_CACRR_FLEX_CLK_DIV_OFFSET;
	__raw_writel(cacrr, MXC_CCM_CACRR);

	return 0;
}

static struct clk flex_bus_clk = {
	__INIT_CLK_DEBUG(flex_bus_clk)
	.parent = &plat_bus_clk,
	.get_rate = _clk_flex_bus_get_rate,
	.set_rate = _clk_flex_bus_set_rate,
};

static unsigned long _clk_ips_bus_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = ((cacrr & MXC_CCM_CACRR_IPG_CLK_DIV_MASK) >>
		MXC_CCM_CACRR_IPG_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_ips_bus_set_rate(struct clk *clk, unsigned long rate)
{
	u32 cacrr, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 4)
		return -EINVAL;

	cacrr = __raw_readl(MXC_CCM_CACRR) & ~MXC_CCM_CACRR_IPG_CLK_DIV_MASK;
	cacrr |= (div -1) << MXC_CCM_CACRR_IPG_CLK_DIV_OFFSET;
	__raw_writel(cacrr, MXC_CCM_CACRR);

	return 0;
}

static struct clk ips_bus_clk = {
	__INIT_CLK_DEBUG(ips_bus_clk)
	.parent = &plat_bus_clk,
	.get_rate = _clk_ips_bus_get_rate,
#if 0	//we do not support to change ips_bus_clk
	.set_rate = _clk_ips_bus_set_rate,
#endif
};

static unsigned long _clk_snvs_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	div = ((reg & MXC_CCM_CSCDR4_SNVS_CLK_DIV_MASK) >>
		MXC_CCM_CSCDR4_SNVS_CLK_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_snvs_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (div == 0)
		div = 1;
	if (((parent_rate / div) != rate) || div > 4)
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR4) & ~MXC_CCM_CSCDR4_SNVS_CLK_DIV_MASK;
	reg |= (div -1) << MXC_CCM_CSCDR4_SNVS_CLK_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CACRR);

	return 0;
}

static struct clk snvs_clk = {
	__INIT_CLK_DEBUG(snvs_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.get_rate = _clk_snvs_get_rate,
	.set_rate = _clk_snvs_set_rate,
};

static struct clk wdog_snvs_clk = {
	__INIT_CLK_DEBUG(wdog_snvs_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_can0_root_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_CAN0_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static void _clk_can0_root_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_CAN0_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static int _clk_can0_root_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &osch_clk, &ips_bus_clk, NULL, NULL);
	reg = __raw_readl(CAN0_CTRL1) & ~CAN_CTRL_CLKSRC;
	reg |= mux << CAN_CTRL_CLKSRC_OFFSET;
	__raw_writel(reg, CAN0_CTRL1);

	return 0;
}

static struct clk can0_clk_root = {
	__INIT_CLK_DEBUG(can0_clk_root)
	.id = 0,
	.parent = &ips_bus_clk,
	.enable = _clk_can0_root_enable,
	.disable = _clk_can0_root_disable,
	.set_parent = _clk_can0_root_set_parent,
};

static struct clk can0_clk[] = {
	{
	 __INIT_CLK_DEBUG(can0_clk_0)
	.id = 0,
	.parent = &can0_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can0_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE, //FIXME
	},
	{
	 __INIT_CLK_DEBUG(can0_clk_1)
	.id = 1,
	.parent = &can0_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can0_clk[2],
	},
	{
	 __INIT_CLK_DEBUG(can0_clk_2)
	.id = 2,
	.parent = &can0_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can0_clk[3],
	},
	{
	 __INIT_CLK_DEBUG(can0_clk_3)
	.id = 4,
	.parent = &can0_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static int _clk_can1_root_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_CAN1_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static void _clk_can1_root_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_CAN1_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static int _clk_can1_root_set_parent(struct clk *clk, struct clk *parent)
{
	
	u32 reg;
	int mux;

	mux = _get_mux(parent, &osch_clk, &ips_bus_clk, NULL, NULL);
	reg = __raw_readl(CAN1_CTRL1) & ~CAN_CTRL_CLKSRC;
	reg |= mux << CAN_CTRL_CLKSRC_OFFSET;
	__raw_writel(reg, CAN1_CTRL1);

	return 0;
}

static struct clk can1_clk_root = {
	__INIT_CLK_DEBUG(can1_root_clk)
	.parent = &ips_bus_clk,
	.enable = _clk_can1_root_enable,
	.disable = _clk_can1_root_disable,
	.set_parent = _clk_can1_root_set_parent,
};

static struct clk can1_clk[] = {
	{
	 __INIT_CLK_DEBUG(can1_clk_0)
	.id = 0,
	.parent = &can1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can1_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE, //FIXME
	},
	{
	 __INIT_CLK_DEBUG(can1_clk_1)
	.id = 1,
	.parent = &can1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can1_clk[2],
	},
	{
	 __INIT_CLK_DEBUG(can1_clk_2)
	.id = 2,
	.parent = &can1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can1_clk[3],
	},
	{
	 __INIT_CLK_DEBUG(can1_clk_3)
	.id = 3,
	.parent = &can1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static struct clk ftm0_ext_clk;
static struct clk ftm1_ext_clk;
static struct clk ftm2_ext_clk;
static struct clk ftm3_ext_clk;
static struct clk ftm0_fix_clk;
static struct clk ftm1_fix_clk;
static struct clk ftm2_fix_clk;
static struct clk ftm3_fix_clk;

static int _clk_ftm_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) | (1 << clk->enable_shift);
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static void _clk_ftm_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) & ~(1 << clk->enable_shift);
	__raw_writel(reg, MXC_CCM_CSCDR1);
}

static unsigned long _clk_ftm_ext_get_rate(struct clk *clk)
{
	if (clk->parent == &osch_clk)
		return clk_get_rate(clk->parent) / 2;
	else
		return clk_get_rate(clk->parent);
}

static int _clk_ftm_ext_set_parent(struct clk *clk, struct clk *parent)
{
	u8  shift;
	u32 reg;
	int mux;

	if (clk == &ftm0_ext_clk)
		shift = MXC_CCM_CSCMR2_FTM0_EXT_CLK_SEL_OFFSET;
	else if (clk == &ftm1_ext_clk)
		shift = MXC_CCM_CSCMR2_FTM1_EXT_CLK_SEL_OFFSET;
	else if (clk == &ftm2_ext_clk)
		shift = MXC_CCM_CSCMR2_FTM2_EXT_CLK_SEL_OFFSET;
	else if (clk == &ftm3_ext_clk)
		shift = MXC_CCM_CSCMR2_FTM3_EXT_CLK_SEL_OFFSET;
	else
		return -EINVAL;

	mux = _get_mux(parent, &ckil_clk, &oscl_clk, &osch_clk, &audio_ext);
	reg = __raw_readl(MXC_CCM_CSCMR2) & ~(0x3 << shift);
	reg |= mux << shift;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_ftm_fix_set_parent(struct clk *clk, struct clk *parent)
{
	u8  shift;
	u32 reg;
	int mux;

	if (clk == &ftm0_fix_clk)
		shift = MXC_CCM_CSCMR2_FTM0_FIX_CLK_SEL_OFFSET;
	else if (clk == &ftm1_fix_clk)
		shift = MXC_CCM_CSCMR2_FTM1_FIX_CLK_SEL_OFFSET;
	else if (clk == &ftm2_fix_clk)
		shift = MXC_CCM_CSCMR2_FTM2_FIX_CLK_SEL_OFFSET;
	else if (clk == &ftm3_fix_clk)
		shift = MXC_CCM_CSCMR2_FTM3_FIX_CLK_SEL_OFFSET;
	else
		return -EINVAL;

	if (parent == &oscl_clk)
		mux = 0;
	else if (parent == &ckil_clk)
		mux = 1;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR2) & ~(1 << shift);
	reg |= mux << shift;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk ftm0_ext_clk = {
	__INIT_CLK_DEBUG(ftm0_ext_clk)
	.parent = &ckil_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM0_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.get_rate = _clk_ftm_ext_get_rate,
	.set_parent = _clk_ftm_ext_set_parent,
};

static struct clk ftm1_ext_clk = {
	__INIT_CLK_DEBUG(ftm1_ext_clk)
	.parent = &ckil_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM1_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.get_rate = _clk_ftm_ext_get_rate,
	.set_parent = _clk_ftm_ext_set_parent,
};

static struct clk ftm2_ext_clk = {
	__INIT_CLK_DEBUG(ftm2_ext_clk)
	.parent = &ckil_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM2_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.get_rate = _clk_ftm_ext_get_rate,
	.set_parent = _clk_ftm_ext_set_parent,
};

static struct clk ftm3_ext_clk = {
	__INIT_CLK_DEBUG(ftm3_ext_clk)
	.parent = &ckil_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM3_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.get_rate = _clk_ftm_ext_get_rate,
	.set_parent = _clk_ftm_ext_set_parent,
};

static struct clk ftm0_fix_clk = {
	__INIT_CLK_DEBUG(ftm0_fix_clk)
	.parent = &oscl_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM0_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.set_parent = _clk_ftm_fix_set_parent,
};

static struct clk ftm1_fix_clk = {
	__INIT_CLK_DEBUG(ftm1_fix_clk)
	.parent = &oscl_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM1_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.set_parent = _clk_ftm_fix_set_parent,
};

static struct clk ftm2_fix_clk = {
	__INIT_CLK_DEBUG(ftm2_fix_clk)
	.parent = &oscl_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM2_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.set_parent = _clk_ftm_fix_set_parent,
};

static struct clk ftm3_fix_clk = {
	__INIT_CLK_DEBUG(ftm3_fix_clk)
	.parent = &oscl_clk,
	.enable_shift = MXC_CCM_CSCDR1_FTM3_CLK_EN_OFFSET,
	.enable = _clk_ftm_enable,
	.disable = _clk_ftm_disable,
	.set_parent = _clk_ftm_fix_set_parent,
};

static int _clk_ftm0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	if (parent == NULL)
		mux = 0x0;
	else if (parent == &sys_clk)
		mux = 0x1;
	else if (parent == &ftm0_fix_clk)
		mux = 0x2;
	else if (parent == &ftm0_ext_clk)
		mux = 0x3;
	else
		return -EINVAL;

	reg = __raw_readl(FTM0_SC) & ~FTM_SC_CLKS_MASK;
	reg |= mux << FTM_SC_CLKS_OFFSET;
	__raw_writel(reg, FTM0_SC);

	return 0;
}

static struct clk ftm0_clk = {
	__INIT_CLK_DEBUG(ftm0_clk)
	.parent = &ftm0_fix_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ftm0_set_parent,
};

static int _clk_ftm1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	if (parent == NULL)
		mux = 0x0;
	else if (parent == &sys_clk)
		mux = 0x1;
	else if (parent == &ftm1_fix_clk)
		mux = 0x2;
	else if (parent == &ftm1_ext_clk)
		mux = 0x3;
	else
		return -EINVAL;

	reg = __raw_readl(FTM1_SC) & ~FTM_SC_CLKS_MASK;
	reg |= mux << FTM_SC_CLKS_OFFSET;
	__raw_writel(reg, FTM1_SC);

	return 0;
}

static struct clk ftm1_clk = {
	__INIT_CLK_DEBUG(ftm1_clk)
	.parent = &ftm1_fix_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ftm1_set_parent,
};

static int _clk_ftm2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	if (parent == NULL)
		mux = 0x0;
	else if (parent == &sys_clk)
		mux = 0x1;
	else if (parent == &ftm2_fix_clk)
		mux = 0x2;
	else if (parent == &ftm2_ext_clk)
		mux = 0x3;
	else
		return -EINVAL;

	reg = __raw_readl(FTM2_SC) & ~FTM_SC_CLKS_MASK;
	reg |= mux << FTM_SC_CLKS_OFFSET;
	__raw_writel(reg, FTM2_SC);

	return 0;
}

static struct clk ftm2_clk = {
	__INIT_CLK_DEBUG(ftm2_clk)
	.parent = &ftm2_fix_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ftm2_set_parent,
};

static int _clk_ftm3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	if (parent == NULL)
		mux = 0x0;
	else if (parent == &sys_clk)
		mux = 0x1;
	else if (parent == &ftm3_fix_clk)
		mux = 0x2;
	else if (parent == &ftm3_ext_clk)
		mux = 0x3;
	else
		return -EINVAL;

	reg = __raw_readl(FTM3_SC) & ~FTM_SC_CLKS_MASK;
	reg |= mux << FTM_SC_CLKS_OFFSET;
	__raw_writel(reg, FTM3_SC);

	return 0;
}

static struct clk ftm3_clk = {
	__INIT_CLK_DEBUG(ftm3_clk)
	.parent = &ftm3_fix_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ftm3_set_parent,
};

/* FIXME NFC */
static int _clk_nfc_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_NFC_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static void _clk_nfc_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_NFC_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static unsigned long _clk_nfc_get_rate(struct clk *clk)
{
	u32 reg, div, prediv, fracdiv, frac_en;

	prediv = ((__raw_readl(MXC_CCM_CSCDR3) & MXC_CCM_CSCDR3_NFC_PRE_DIV_MASK) >> 
			MXC_CCM_CSCDR3_NFC_PRE_DIV_OFFSET) + 1;
	reg = __raw_readl(MXC_CCM_CSCDR2);
	frac_en = MXC_CCM_CSCDR2_NFC_FRAC_DIV_EN & MXC_CCM_CSCDR2_NFC_FRAC_DIV_EN;
	fracdiv = ((reg & MXC_CCM_CSCDR2_NFC_FRAC_DIV_MASK) >>
			MXC_CCM_CSCDR2_NFC_FRAC_DIV_OFFSET) + 1;

	if (!frac_en)
		div = prediv * fracdiv;
	else
		div = prediv * fracdiv; //FIXME

	return clk_get_rate(clk->parent) / div;

}

static int _clk_nfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 128)
		return -EINVAL;

	//FIXME
	__calc_pre_post_dividers(1 << 4, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CSCDR3) & ~MXC_CCM_CSCDR3_NFC_PRE_DIV_MASK;
	reg |= (pre -1) << MXC_CCM_CSCDR3_NFC_PRE_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_NFC_FRAC_DIV_MASK;
	reg &= ~MXC_CCM_CSCDR2_NFC_FRAC_DIV_EN;
	reg |= (post -1) << MXC_CCM_CSCDR2_NFC_FRAC_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static int _clk_nfc_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_NFC_CLK_SEL_MASK;
	mux = _get_mux(parent, &plat_bus_clk, &pll1_pfd1, &pll3_pfd1, &pll3_pfd3);
	reg |= mux << MXC_CCM_CSCMR1_NFC_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk nfc_clk_root = {
	__INIT_CLK_DEBUG(nfc_clk_root)
	.parent = &plat_bus_clk,
	.enable = _clk_nfc_enable,
	.disable = _clk_nfc_disable,
	.get_rate = _clk_nfc_get_rate,
	.set_rate = _clk_nfc_set_rate,
	.set_parent = _clk_nfc_set_parent,
};

static struct clk nfc_clk[] = {
	{
	__INIT_CLK_DEBUG(nfc_clk_0)
	.id = 0,
	.parent = &nfc_clk_root,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &nfc_clk[1],
	},
	{
	__INIT_CLK_DEBUG(nfc_clk_1)
	.id = 1,
	.parent = &nfc_clk_root,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &nfc_clk[2],
	},
	{
	__INIT_CLK_DEBUG(nfc_clk_2)
	.id = 2,
	.parent = &nfc_clk_root,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &nfc_clk[3],
	},
	{
	__INIT_CLK_DEBUG(nfc_clk_3)
	.id = 3,
	.parent = &nfc_clk_root,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};
/* FIXME QSPI */


static int _clk_enet_rmii_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) | MXC_CCM_CSCDR1_RMII_CLK_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static void _clk_enet_rmii_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) & ~MXC_CCM_CSCDR1_RMII_CLK_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);
}

static int _clk_enet_rmii_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_RMII_CLK_SEL_MASK;
	mux = _get_mux(parent, &enet_ext, &audio_ext, &enet_clk, &enet_div2_clk);
	reg |= mux << MXC_CCM_CSCMR2_RMII_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk enet_rmii_clk = {
	__INIT_CLK_DEBUG(enet_rmii_clk)
	.parent = &enet_clk,
	//.parent = &enet_div2_clk,
	.enable = _clk_enet_rmii_enable,
	.disable = _clk_enet_rmii_disable,
#if 1 //FIXME 0	//we do not support to change enet freq
	.set_parent = _clk_enet_rmii_set_parent,
#endif
};

static int _clk_enet_ts_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) | MXC_CCM_CSCDR1_ENET_TS_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static void _clk_enet_ts_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) & ~MXC_CCM_CSCDR1_ENET_TS_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);
}

static int _clk_enet_ts_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	if (parent == &enet_ext)
		mux = 0x0;
	else if (parent == &osch_clk)
		mux = 0x1;
	else if (parent == &audio_ext)
		mux = 0x2;
	else if (parent == &usb_clk)
		mux = 0x3;
	else if (parent == &enet_ts)
		mux = 0x4;
	else if (parent == &enet_div2_clk)
		mux = 0x5;
	else if (parent == &enet_clk)
		mux = 0x6;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_ENET_TS_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR2_ENET_TS_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk enet_ts_clk = {
	__INIT_CLK_DEBUG(enet_ts_clk)
	.parent = &enet_ext,
	.enable = _clk_enet_ts_enable,
	.disable = _clk_enet_ts_disable,
#if 0	//we do not support to change enet freq
	.set_parent = _clk_enet_ts_set_parent,
#endif
};

static struct clk fec0_clk = {
	__INIT_CLK_DEBUG(fec0_rmii_clk)
	.parent = &enet_rmii_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
#ifdef CONFIG_FEC_1588
	.secondary = &enet_ts_clk,
#endif
};

static struct clk fec1_clk = {
	__INIT_CLK_DEBUG(fec1_rmii_clk)
	.parent = &enet_rmii_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
#ifdef CONFIG_FEC_1588
	.secondary = &enet_ts_clk,
#endif
};

static int _clk_sdhc0_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_ESDHC0_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	_clk_enable(clk);

	return 0;
}

static void _clk_sdhc0_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_ESDHC0_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static unsigned long _clk_sdhc0_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CSCDR2);
	u32 div = ((reg & MXC_CCM_CSCDR2_ESDHC0_DIV_MASK) >>
			MXC_CCM_CSCDR2_ESDHC0_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_sdhc0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 16))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_ESDHC0_DIV_MASK;
	reg |= (div -1) << MXC_CCM_CSCDR2_ESDHC0_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);
	return 0;
}

static int _clk_sdhc0_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_ESDHC0_CLK_SEL_MASK;

	mux = _get_mux(parent, &pll3_480_usb1_main_clk, &pll3_pfd3,
			&pll1_pfd3, &plat_bus_clk);
	reg |= (mux << MXC_CCM_CSCMR1_ESDHC0_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk sdhc0_clk = {
	__INIT_CLK_DEBUG(sdhc0_clk)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_sdhc0_enable,
	.disable = _clk_sdhc0_disable,
	.get_rate = _clk_sdhc0_get_rate,
	.set_rate = _clk_sdhc0_set_rate,
	.set_parent = _clk_sdhc0_set_parent,
};

static int _clk_sdhc1_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_ESDHC1_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	_clk_enable(clk);

	return 0;
}

static void _clk_sdhc1_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_ESDHC1_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static unsigned long _clk_sdhc1_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CSCDR2);
	u32 div = ((reg & MXC_CCM_CSCDR2_ESDHC1_DIV_MASK) >>
			MXC_CCM_CSCDR2_ESDHC1_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_sdhc1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 16))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_ESDHC1_DIV_MASK;
	reg |= (div -1) << MXC_CCM_CSCDR2_ESDHC1_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);
	return 0;
}

static int _clk_sdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_ESDHC1_CLK_SEL_MASK;

	mux = _get_mux(parent, &pll3_480_usb1_main_clk, &pll3_pfd3,
			&pll1_pfd3, &plat_bus_clk);
	reg |= (mux << MXC_CCM_CSCMR1_ESDHC1_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk sdhc1_clk = {
	__INIT_CLK_DEBUG(sdhc1_clk)
	.parent = &pll3_480_usb1_main_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_sdhc1_enable,
	.disable = _clk_sdhc1_disable,
	.get_rate = _clk_sdhc1_get_rate,
	.set_rate = _clk_sdhc1_set_rate,
	.set_parent = _clk_sdhc1_set_parent,
};

static struct clk dcu0_clk_root;
static struct clk dcu1_clk_root;

static int _clk_dcu_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	reg |= 1 <<  clk->enable_shift;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static void _clk_dcu_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	reg &= ~(1 <<  clk->enable_shift);
	__raw_writel(reg, MXC_CCM_CSCDR3);
}

static unsigned long _clk_dcu_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	if (clk == &dcu0_clk_root)
		div = ((reg & ~MXC_CCM_CSCDR3_DCU0_DIV_MASK) >>
			MXC_CCM_CSCDR3_DCU0_DIV_OFFSET) + 1;
	else
		div = ((reg & ~MXC_CCM_CSCDR3_DCU1_DIV_MASK) >>
			MXC_CCM_CSCDR3_DCU1_DIV_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_dcu_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	if (clk == &dcu0_clk_root) {
		reg &= ~MXC_CCM_CSCDR3_DCU0_DIV_MASK;
		reg |= (div -1) << MXC_CCM_CSCDR3_DCU0_DIV_OFFSET;
	} else if (clk == &dcu1_clk_root) {
		reg &= ~MXC_CCM_CSCDR3_DCU1_DIV_MASK;
		reg |= (div -1) << MXC_CCM_CSCDR3_DCU1_DIV_OFFSET;
	} else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static int _clk_dcu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_pfd2, &pll3_480_usb1_main_clk, NULL, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (clk == &dcu0_clk_root) {
		reg &= ~MXC_CCM_CSCMR1_DCU0_CLK_SEL;
		reg |= mux << MXC_CCM_CSCMR1_DCU0_CLK_SEL_OFFSET;
	} else if (clk == &dcu1_clk_root) {
		reg &= ~MXC_CCM_CSCMR1_DCU1_CLK_SEL;
		reg |= mux << MXC_CCM_CSCMR1_DCU1_CLK_SEL_OFFSET;
	} else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk dcu0_clk_root = {
	__INIT_CLK_DEBUG(dcu0_clk_root)
	.parent = &pll1_pfd2, //FIXME
	.enable_shift = MXC_CCM_CSCDR3_DCU0_EN_OFFSET,
	.enable = _clk_dcu_enable,
	.disable = _clk_dcu_disable,
	.set_rate = _clk_dcu_set_rate,
	.get_rate = _clk_dcu_get_rate,
	.set_parent = _clk_dcu_set_parent,
};

static struct clk dcu0_clk[] = {
	{
	__INIT_CLK_DEBUG(dcu0_clk_0)
	.id = 0,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[1],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_1)
	.id = 1,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[2],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_2)
	.id = 2,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[3],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_3)
	.id = 3,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[4],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_4)
	.id = 4,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[5],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_5)
	.id = 5,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[6],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_6)
	.id = 6,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu0_clk[7],
	},
	{
	__INIT_CLK_DEBUG(dcu0_clk_7)
	.id = 7,
	.parent = &dcu0_clk_root,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static struct clk dcu1_clk_root = {
	__INIT_CLK_DEBUG(dcu1_clk_root)
	.parent = &pll1_pfd2, //FIXME
	.enable_shift = MXC_CCM_CSCDR3_DCU1_EN_OFFSET,
	.enable = _clk_dcu_enable,
	.disable = _clk_dcu_disable,
	.set_rate = _clk_dcu_set_rate,
	.get_rate = _clk_dcu_get_rate,
	.set_parent = _clk_dcu_set_parent,
};

static struct clk dcu1_clk[] = {
	{
	__INIT_CLK_DEBUG(dcu1_clk_0)
	.id = 0,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[1],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_1)
	.id = 1,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[2],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_2)
	.id = 2,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[3],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_3)
	.id = 3,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[4],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_4)
	.id = 4,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[5],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_5)
	.id = 5,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[6],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_6)
	.id = 6,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &dcu1_clk[7],
	},
	{
	__INIT_CLK_DEBUG(dcu1_clk_7)
	.id = 7,
	.parent = &dcu1_clk_root,
	.enable_reg = MXC_CCM_CCGR9,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

/* FIXME ESAI, SPDIF, SAI */

static int _clk_video_adc_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR1) | MXC_CCM_CSCDR1_VADC_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	_clk_enable(clk);
	return 0;
}

static void _clk_video_adc_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);

	reg = __raw_readl(MXC_CCM_CSCDR1) & ~MXC_CCM_CSCDR1_VADC_EN;
	__raw_writel(reg, MXC_CCM_CSCDR1);
}

static unsigned long _clk_video_adc_get_rate(struct clk *clk)
{
	unsigned int div;

	div = (__raw_readl(MXC_CCM_CSCDR1) & MXC_CCM_CSCDR1_VADC_DIV_MASK) >>
		MXC_CCM_CSCDR1_VADC_DIV_OFFSET;

	return clk_get_rate(clk->parent) / (div + 1);
}

static int _clk_video_adc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 4))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1) & ~MXC_CCM_CSCDR1_VADC_DIV_MASK;
	reg |= div << MXC_CCM_CSCDR1_VADC_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static int _clk_video_adc_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg;

	mux = _get_mux(parent, &pll6_div_clk, &pll3_div_clk,
			&pll3_480_usb1_main_clk, NULL);

	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_VADC_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_VADC_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk video_adc_clk = {
	__INIT_CLK_DEBUG(video_adc_clk)
	.parent = &pll6_div_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_video_adc_enable,
	.disable = _clk_video_adc_disable,
	.get_rate = _clk_video_adc_get_rate,
	.set_rate = _clk_video_adc_set_rate,
	.set_parent = _clk_video_adc_set_parent,
};

static unsigned long _clk_video_adc_div2_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 2;
}

static struct clk video_adc_div2_clk = {
	__INIT_CLK_DEBUG(video_adc_div2_clk)
	.parent = &video_adc_clk,
	.get_rate = _clk_video_adc_div2_get_rate,
};

static int _clk_gpu_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR2) | MXC_CCM_CSCDR2_GPU_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	_clk_enable(clk);
	return 0;
}

static void _clk_gpu_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);

	reg = __raw_readl(MXC_CCM_CSCDR2) & ~MXC_CCM_CSCDR2_GPU_EN;
	__raw_writel(reg, MXC_CCM_CSCDR2);
}

static int _clk_gpu_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg;

	mux = _get_mux(parent, &pll2_pfd2, &pll3_pfd2, NULL, NULL);

	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_GPU_CLK_SEL;
	reg |= mux << MXC_CCM_CSCMR1_GPU_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk gpu_clk = {
	__INIT_CLK_DEBUG(gpu_clk)
	.parent = &pll2_pfd2, //FIXME
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_gpu_enable,
	.disable = _clk_gpu_disable,
	.set_parent = _clk_gpu_set_parent,
};

static int _clk_swo_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3) | MXC_CCM_CSCDR3_SWO_EN;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static void _clk_swo_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3) & ~MXC_CCM_CSCDR3_SWO_EN;
	__raw_writel(reg, MXC_CCM_CSCDR3);
}

static unsigned long _clk_swo_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CSCDR3);
	u32 div = ((reg & MXC_CCM_CSCDR3_SWO_DIV) >>
		  MXC_CCM_CSCDR3_SWO_DIV_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_swo_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 2))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR3) & ~MXC_CCM_CSCDR3_SWO_DIV;
	reg |= (div -1) << MXC_CCM_CSCDR3_SWO_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static int _clk_swo_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_SWO_CLK_SEL;

	mux = _get_mux(parent, &ckih_clk, &ips_bus_clk, NULL, NULL);
	reg |= mux << MXC_CCM_CSCMR2_SWO_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk swo_clk = {
	__INIT_CLK_DEBUG(swo_clk)
	.parent = &ips_bus_clk, //FIXME
	.enable = _clk_swo_enable,
	.disable = _clk_swo_disable,
	.set_rate = _clk_swo_set_rate,
	.get_rate = _clk_swo_get_rate,
	.set_parent = _clk_swo_set_parent,
};

static int _clk_trace_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3) | MXC_CCM_CSCDR3_TRACE_EN;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static void _clk_trace_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCDR3) & ~MXC_CCM_CSCDR3_TRACE_EN;
	__raw_writel(reg, MXC_CCM_CSCDR3);
}

static unsigned long _clk_trace_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CSCDR3);
	u32 div = ((reg & MXC_CCM_CSCDR3_TRACE_DIV_MASK) >>
		  MXC_CCM_CSCDR3_TRACE_DIV_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_trace_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 4))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR3) & ~MXC_CCM_CSCDR3_TRACE_DIV_MASK;
	reg |= (div -1) << MXC_CCM_CSCDR3_TRACE_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static int _clk_trace_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_TRACE_CLK_SEL;

	mux = _get_mux(parent, &plat_bus_clk, &pll3_480_usb1_main_clk,
			NULL, NULL);
	reg |= mux << MXC_CCM_CSCMR2_TRACE_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk trace_clk = {
	__INIT_CLK_DEBUG(trace_clk)
	.parent = &plat_bus_clk, //FIXME
	.enable = _clk_trace_enable,
	.disable = _clk_trace_disable,
	.set_rate = _clk_trace_set_rate,
	.get_rate = _clk_trace_get_rate,
	.set_parent = _clk_trace_set_parent,
};

static struct clk ca5_scu_clk = {
	__INIT_CLK_DEBUG(ca5_scu_clk)
	.parent = &plat_bus_clk,
};

static struct clk twd_clk = {
	__INIT_CLK_DEBUG(twd_clk)
	.parent = &ca5_scu_clk,
};

static struct clk dma_mux0_clk = {
	__INIT_CLK_DEBUG(dma_mux0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk dma_mux1_clk = {
	__INIT_CLK_DEBUG(dma_mux1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk uart0_clk = {
	__INIT_CLK_DEBUG(uart0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk uart1_clk = {
	__INIT_CLK_DEBUG(uart1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk uart2_clk = {
	__INIT_CLK_DEBUG(uart2_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk uart3_clk = {
	__INIT_CLK_DEBUG(uart3_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk spi0_clk = {
	__INIT_CLK_DEBUG(spi0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk spi1_clk = {
	__INIT_CLK_DEBUG(spi1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk usbc0_clk = {
	__INIT_CLK_DEBUG(usbc0_clk)
	.parent = &ips_bus_clk, //FIXME
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk pdb_clk = {
	__INIT_CLK_DEBUG(pdb_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk pit_clk = {
	__INIT_CLK_DEBUG(pit_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk adc0_clk = {
	__INIT_CLK_DEBUG(adc0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk tcon0_clk = {
	__INIT_CLK_DEBUG(tcon0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk wdog_a5_clk = {
	__INIT_CLK_DEBUG(wdog_a5_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk wdog_m4_clk = {
	__INIT_CLK_DEBUG(wdog_m4_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk lptmr_clk = {
	__INIT_CLK_DEBUG(lptmr_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk rle_clk = {
	__INIT_CLK_DEBUG(rle_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk iomux_clk = {
	__INIT_CLK_DEBUG(iomux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk port_a_mux_clk = {
	__INIT_CLK_DEBUG(port_a_mux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk port_b_mux_clk = {
	__INIT_CLK_DEBUG(port_b_mux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk port_c_mux_clk = {
	__INIT_CLK_DEBUG(port_c_mux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk port_d_mux_clk = {
	__INIT_CLK_DEBUG(port_d_mux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk port_e_mux_clk = {
	__INIT_CLK_DEBUG(port_e_mux_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk anadig_clk = {
	__INIT_CLK_DEBUG(anadig_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk scsc_clk = {
	__INIT_CLK_DEBUG(scsc_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk asrc_clk = {
	__INIT_CLK_DEBUG(asrc_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk ewm_clk = {
	__INIT_CLK_DEBUG(ewm_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk i2c0_clk = {
	__INIT_CLK_DEBUG(i2c0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk i2c1_clk = {
	__INIT_CLK_DEBUG(i2c1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk wkup_clk = {
	__INIT_CLK_DEBUG(wkup_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk ccm_clk = {
	__INIT_CLK_DEBUG(ccm_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk gpc_clk = {
	__INIT_CLK_DEBUG(gpc_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk vreg_clk = {
	__INIT_CLK_DEBUG(vreg_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk src_clk = {
	__INIT_CLK_DEBUG(src_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk cmu_clk = {
	__INIT_CLK_DEBUG(cmu_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk dma_mux2_clk = {
	__INIT_CLK_DEBUG(dma_mux2_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk dma_mux3_clk = {
	__INIT_CLK_DEBUG(dma_mux3_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk ocotp_clk = {
	__INIT_CLK_DEBUG(ocotp_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};


static struct clk uart4_clk = {
	__INIT_CLK_DEBUG(uart4_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk uart5_clk = {
	__INIT_CLK_DEBUG(uart5_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk spi2_clk = {
	__INIT_CLK_DEBUG(spi2_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk spi3_clk = {
	__INIT_CLK_DEBUG(spi3_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk ddrmc_clk = {
	__INIT_CLK_DEBUG(ddrmc_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk usbc1_clk = {
	__INIT_CLK_DEBUG(usbc1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk adc1_clk = {
	__INIT_CLK_DEBUG(adc1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk tcon1_clk = {
	__INIT_CLK_DEBUG(tcon1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk seg_lcd_clk = {
	__INIT_CLK_DEBUG(seg_lcd_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk video_dec_clk = {
	__INIT_CLK_DEBUG(video_dec_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk viu3_clk = {
	__INIT_CLK_DEBUG(viu3_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk dac0_clk = {
	__INIT_CLK_DEBUG(dac0_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk dac1_clk = {
	__INIT_CLK_DEBUG(dac1_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR8,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk i2c2_clk = {
	__INIT_CLK_DEBUG(i2c2_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk i2c3_clk = {
	__INIT_CLK_DEBUG(i2c3_clk)
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk eth_l2_sw_clk[] = {
	{
	__INIT_CLK_DEBUG(eth_l2_sw_0_clk)
	.id = 0,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[1],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_1_clk)
	.id = 1,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[2],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_2_clk)
	.id = 2,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[3],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_3_clk)
	.id = 3,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[4],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_4_clk)
	.id = 4,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[5],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_5_clk)
	.id = 5,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[6],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_6_clk)
	.id = 6,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &eth_l2_sw_clk[7],
	},
	{
	__INIT_CLK_DEBUG(eth_l2_sw_7_clk)
	.id = 7,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR10,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static struct clk caam_clk[] = {
	{
	__INIT_CLK_DEBUG(caam_0_clk)
	.id = 0,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[1],
	},
	{
	__INIT_CLK_DEBUG(caam_1_clk)
	.id = 1,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[2],
	},
	{
	__INIT_CLK_DEBUG(caam_2_clk)
	.id = 2,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[3],
	},
	{
	__INIT_CLK_DEBUG(caam_3_clk)
	.id = 3,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[4],
	},
	{
	__INIT_CLK_DEBUG(caam_4_clk)
	.id = 4,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[5],
	},
	{
	__INIT_CLK_DEBUG(caam_5_clk)
	.id = 5,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[6],
	},
	{
	__INIT_CLK_DEBUG(caam_6_clk)
	.id = 6,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[7],
	},
	{
	__INIT_CLK_DEBUG(caam_7_clk)
	.id = 7,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[8],
	},
	{
	__INIT_CLK_DEBUG(caam_8_clk)
	.id = 8,
	.parent = &ips_bus_clk,
	.enable_reg = MXC_CCM_CCGR11,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

#define _REGISTER_CLOCK(d, n, c) \
	{ \
		.dev_id = d, \
		.con_id = n, \
		.clk = &c, \
	}


static struct clk_lookup lookups[] = {
	_REGISTER_CLOCK(NULL, "osch", osch_clk),
	_REGISTER_CLOCK(NULL, "ckih", ckih_clk),
	_REGISTER_CLOCK(NULL, "oscl", oscl_clk),
	_REGISTER_CLOCK(NULL, "ckil", ckil_clk),
	_REGISTER_CLOCK(NULL, "pll1_main_clk", pll1_sys_main_clk),
	_REGISTER_CLOCK(NULL, "pll1_pfd1", pll1_pfd1),
	_REGISTER_CLOCK(NULL, "pll1_pfd2", pll1_pfd2),
	_REGISTER_CLOCK(NULL, "pll1_pfd3", pll1_pfd3),
	_REGISTER_CLOCK(NULL, "pll1_pfd4", pll1_pfd4),
	_REGISTER_CLOCK(NULL, "pll1_pfd_out", pll1_pfd_out),
	_REGISTER_CLOCK(NULL, "pll1_pfd_out_div", pll1_pfd_out_div),
	_REGISTER_CLOCK(NULL, "pll2", pll2_528_main_clk),
	_REGISTER_CLOCK(NULL, "pll2_pfd1", pll2_pfd1),
	_REGISTER_CLOCK(NULL, "pll2_pfd2", pll2_pfd2),
	_REGISTER_CLOCK(NULL, "pll2_pfd3", pll2_pfd3),
	_REGISTER_CLOCK(NULL, "pll2_pfd4", pll2_pfd4),
	_REGISTER_CLOCK(NULL, "pll1_pfd_out", pll1_pfd_out),
	_REGISTER_CLOCK(NULL, "pll3_main_clk", pll3_480_usb1_main_clk),
	_REGISTER_CLOCK(NULL, "pll3_div_clk", pll3_div_clk),
	_REGISTER_CLOCK(NULL, "pll3_pfd1", pll2_pfd1),
	_REGISTER_CLOCK(NULL, "pll3_pfd2", pll2_pfd2),
	_REGISTER_CLOCK(NULL, "pll3_pfd3", pll2_pfd3),
	_REGISTER_CLOCK(NULL, "pll3_pfd4", pll2_pfd4),
	_REGISTER_CLOCK(NULL, "pll4", pll4_audio_main_clk),
	_REGISTER_CLOCK(NULL, "pll5", pll5_enet_main_clk),
	_REGISTER_CLOCK(NULL, "pll6", pll6_video_main_clk),
	_REGISTER_CLOCK(NULL, "pll6_div_clk", pll6_div_clk),
	_REGISTER_CLOCK(NULL, "pll_usb2", pll_480_usb2_main_clk),
	_REGISTER_CLOCK(NULL, "cpu_clk", cpu_clk),
	_REGISTER_CLOCK(NULL, "ddrc_clk", ddrc_clk),
	_REGISTER_CLOCK(NULL, "plat_bus_clk", plat_bus_clk),
	_REGISTER_CLOCK(NULL, "cm4_clk", cm4_clk),
	_REGISTER_CLOCK(NULL, "flex_bus_clk", flex_bus_clk),
	_REGISTER_CLOCK(NULL, "ips_bus_clk", ips_bus_clk),
	_REGISTER_CLOCK(NULL, "snvs_clk", snvs_clk),
	_REGISTER_CLOCK(NULL, "wdog_snvs_clk", wdog_snvs_clk),
	_REGISTER_CLOCK(NULL, "can0_root_clk", can0_clk_root),
	_REGISTER_CLOCK(NULL, "can1_root_clk", can1_clk_root),
	_REGISTER_CLOCK("mvf-flexcan.0", NULL, can0_clk[0]),
	_REGISTER_CLOCK("mvf-flexcan.1", NULL, can1_clk[0]),
	_REGISTER_CLOCK(NULL, "ftm0_ext_clk", ftm0_ext_clk),
	_REGISTER_CLOCK(NULL, "ftm1_ext_clk", ftm1_ext_clk),
	_REGISTER_CLOCK(NULL, "ftm2_ext_clk", ftm2_ext_clk),
	_REGISTER_CLOCK(NULL, "ftm3_ext_clk", ftm3_ext_clk),
	_REGISTER_CLOCK(NULL, "ftm0_fix_clk", ftm0_fix_clk),
	_REGISTER_CLOCK(NULL, "ftm1_fix_clk", ftm1_fix_clk),
	_REGISTER_CLOCK(NULL, "ftm2_fix_clk", ftm2_fix_clk),
	_REGISTER_CLOCK(NULL, "ftm3_fix_clk", ftm3_fix_clk),
	_REGISTER_CLOCK(NULL, "ftm0_clk", ftm0_clk),
	_REGISTER_CLOCK(NULL, "ftm1_clk", ftm1_clk),
	_REGISTER_CLOCK(NULL, "ftm2_clk", ftm2_clk),
	_REGISTER_CLOCK(NULL, "ftm3_clk", ftm3_clk),
	_REGISTER_CLOCK(NULL, "nfc_clk_root", nfc_clk_root),
	_REGISTER_CLOCK(NULL, "nfc_clk", nfc_clk[0]),
	_REGISTER_CLOCK(NULL, "enet_clk", enet_clk),
	_REGISTER_CLOCK(NULL, "enet_div2_clk", enet_div2_clk),
	_REGISTER_CLOCK(NULL, "enet_rmii_clk", enet_rmii_clk),
	_REGISTER_CLOCK(NULL, "enet_ts_clk", enet_ts_clk),
	_REGISTER_CLOCK(NULL, "fec_clk", fec0_clk),
	_REGISTER_CLOCK(NULL, "fec1_clk", fec1_clk),
	_REGISTER_CLOCK(NULL, "sdhc0_clk", sdhc0_clk),
	_REGISTER_CLOCK(NULL, "sdhc1_clk", sdhc1_clk),
	_REGISTER_CLOCK(NULL, "dcu0_clk_root", dcu0_clk_root),
	_REGISTER_CLOCK(NULL, "dcu1_clk_root", dcu1_clk_root),
	_REGISTER_CLOCK(NULL, "dcu0_clk", dcu0_clk[0]),
	_REGISTER_CLOCK(NULL, "dcu1_clk", dcu1_clk[0]),
	_REGISTER_CLOCK(NULL, "video_adc_clk", video_adc_clk),
	_REGISTER_CLOCK(NULL, "video_adc_div2_clk", video_adc_div2_clk),
	_REGISTER_CLOCK(NULL, "gpu_clk", gpu_clk),
	_REGISTER_CLOCK(NULL, "swo_clk", swo_clk),
	_REGISTER_CLOCK(NULL, "trace_clk", trace_clk),
	_REGISTER_CLOCK(NULL, "ca5_scu_clk", ca5_scu_clk),
	_REGISTER_CLOCK("smp_twd", NULL, twd_clk),
	_REGISTER_CLOCK(NULL, "dma_mix0_clk", dma_mux0_clk),
	_REGISTER_CLOCK(NULL, "dma_mix1_clk", dma_mux1_clk),
	_REGISTER_CLOCK("mvf-uart.0", NULL, uart0_clk),
	_REGISTER_CLOCK("mvf-uart.1", NULL, uart1_clk),
	_REGISTER_CLOCK("mvf-uart.2", NULL, uart2_clk),
	_REGISTER_CLOCK("mvf-uart.3", NULL, uart3_clk),
	_REGISTER_CLOCK(NULL, "spi0_clk", spi0_clk),
	_REGISTER_CLOCK(NULL, "spi1_clk", spi1_clk),
	_REGISTER_CLOCK(NULL, "usbc0_clk", usbc0_clk),
	_REGISTER_CLOCK(NULL, "pdb_clk", pdb_clk),
	_REGISTER_CLOCK(NULL, "pit_clk", pit_clk),
	_REGISTER_CLOCK(NULL, "adc0_clk", adc0_clk),
	_REGISTER_CLOCK(NULL, "tcon0_clk", tcon0_clk),
	_REGISTER_CLOCK(NULL, "wdog_a5_clk", wdog_a5_clk),
	_REGISTER_CLOCK(NULL, "wdog_m4_clk", wdog_m4_clk),
	_REGISTER_CLOCK(NULL, "lptmr_clk", lptmr_clk),
	_REGISTER_CLOCK(NULL, "rle_clk", rle_clk),
	_REGISTER_CLOCK(NULL, "iomux_clk", iomux_clk),
	_REGISTER_CLOCK(NULL, "port_a_mux_clk", port_a_mux_clk),
	_REGISTER_CLOCK(NULL, "port_b_mux_clk", port_b_mux_clk),
	_REGISTER_CLOCK(NULL, "port_c_mux_clk", port_c_mux_clk),
	_REGISTER_CLOCK(NULL, "port_d_mux_clk", port_d_mux_clk),
	_REGISTER_CLOCK(NULL, "port_e_mux_clk", port_e_mux_clk),
	_REGISTER_CLOCK(NULL, "anadig_clk", anadig_clk),
	_REGISTER_CLOCK(NULL, "scsc_clk", scsc_clk),
	_REGISTER_CLOCK(NULL, "asrc_clk", asrc_clk),
	_REGISTER_CLOCK(NULL, "ewm_clk", ewm_clk),
	_REGISTER_CLOCK(NULL, "i2c0_clk", i2c0_clk),
	_REGISTER_CLOCK(NULL, "i2c1_clk", i2c1_clk),
	_REGISTER_CLOCK(NULL, "wkup_clk", wkup_clk),
	_REGISTER_CLOCK(NULL, "ccm_clk", ccm_clk),
	_REGISTER_CLOCK(NULL, "gpc_clk", gpc_clk),
	_REGISTER_CLOCK(NULL, "vreg_clk", vreg_clk),
	_REGISTER_CLOCK(NULL, "src_clk", src_clk),
	_REGISTER_CLOCK(NULL, "cmu_clk", cmu_clk),
	_REGISTER_CLOCK(NULL, "dma_mix2_clk", dma_mux2_clk),
	_REGISTER_CLOCK(NULL, "dma_mix3_clk", dma_mux3_clk),
	_REGISTER_CLOCK(NULL, "ocotp_clk", ocotp_clk),
	_REGISTER_CLOCK("mvf-uart.4", NULL, uart4_clk),
	_REGISTER_CLOCK("mvf-uart.5", NULL, uart5_clk),
	_REGISTER_CLOCK(NULL, "spi2_clk", spi2_clk),
	_REGISTER_CLOCK(NULL, "spi3_clk", spi3_clk),
	_REGISTER_CLOCK(NULL, "ddrmc_clk", ddrmc_clk),
	_REGISTER_CLOCK(NULL, "usbc1_clk", usbc1_clk),
	_REGISTER_CLOCK(NULL, "adc1_clk", adc1_clk),
	_REGISTER_CLOCK(NULL, "tcon1_clk", tcon1_clk),
	_REGISTER_CLOCK(NULL, "seg_lcd_clk", seg_lcd_clk),
	_REGISTER_CLOCK(NULL, "video_dec_clk", video_dec_clk),
	_REGISTER_CLOCK(NULL, "viu3_clk", viu3_clk),
	_REGISTER_CLOCK(NULL, "dac0_clk", dac0_clk),
	_REGISTER_CLOCK(NULL, "dac1_clk", dac1_clk),
	_REGISTER_CLOCK(NULL, "i2c2_clk", i2c2_clk),
	_REGISTER_CLOCK(NULL, "i2c3_clk", i2c3_clk),
	_REGISTER_CLOCK(NULL, "eth_l2_sw_clk", eth_l2_sw_clk[0]),
	_REGISTER_CLOCK(NULL, "caam_clk", caam_clk[0]),
	_REGISTER_CLOCK(NULL, "anaclk_1", anaclk_1),
	_REGISTER_CLOCK(NULL, "audio_ext", audio_ext),
	_REGISTER_CLOCK(NULL, "enet_ext", enet_ext),
	_REGISTER_CLOCK(NULL, "enet_ts", enet_ts),
	_REGISTER_CLOCK(NULL, "usb_clk", usb_clk),
};

static void clk_tree_init(void)

{
	u32 mux;

	mux = (__raw_readl(MXC_CCM_CCSR) & MXC_CCM_CCSR_PLL1_PFD_CLK_SEL_MASK)
		>> MXC_CCM_CCSR_PLL1_PFD_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		pll1_pfd_out.parent = &pll1_sys_main_clk;
		break;
	case 1:
		pll1_pfd_out.parent = &pll1_pfd1;
		break;
	case 2:
		pll1_pfd_out.parent = &pll1_pfd2;
		break;
	case 3:
		pll1_pfd_out.parent = &pll1_pfd3;
		break;
	case 4:
		pll1_pfd_out.parent = &pll1_pfd4;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CCSR) & MXC_CCM_CCSR_PLL2_PFD_CLK_SEL_MASK)
		>> MXC_CCM_CCSR_PLL2_PFD_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		pll2_pfd_out.parent = &pll2_528_main_clk;
		break;
	case 1:
		pll2_pfd_out.parent = &pll2_pfd1;
		break;
	case 2:
		pll2_pfd_out.parent = &pll2_pfd2;
		break;
	case 3:
		pll2_pfd_out.parent = &pll2_pfd3;
		break;
	case 4:
		pll2_pfd_out.parent = &pll2_pfd4;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CCSR) & MXC_CCM_CCSR_SYS_CLK_SEL_MASK)
		>> MXC_CCM_CCSR_SYS_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		sys_clk.parent = &osch_clk;
		break;
	case 1:
		sys_clk.parent = &oscl_clk;
		break;
	case 2:
		sys_clk.parent = &pll2_pfd_out;
		break;
	case 3:
		sys_clk.parent = &pll2_528_main_clk;
		break;
	case 4:
		sys_clk.parent = &pll1_pfd_out;
		break;
	case 5:
		sys_clk.parent = &pll3_480_usb1_main_clk;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CCSR) & MXC_CCM_CCSR_CA5_CLK_SEL)
		>> MXC_CCM_CCSR_CA5_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		cpu_clk.parent = &sys_clk;
		break;
	case 1:
		cpu_clk.parent = &pll1_pfd_out_div;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CCSR) & MXC_CCM_CCSR_DDRC_CLK_SEL)
		>> MXC_CCM_CCSR_DDRC_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		ddrc_clk.parent = &pll2_pfd2;
		break;
	case 1:
		ddrc_clk.parent = &sys_clk;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CSCMR2) & MXC_CCM_CSCMR2_RMII_CLK_SEL_MASK)
		>> MXC_CCM_CSCMR2_RMII_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		enet_rmii_clk.parent = &enet_ext;
		break;
	case 1:
		enet_rmii_clk.parent = &audio_ext;
		break;
	case 2:
		enet_rmii_clk.parent = &enet_clk;
		break;
	case 3:
		enet_rmii_clk.parent = &enet_div2_clk;
		break;
	}

	mux = (__raw_readl(MXC_CCM_CSCMR2) & MXC_CCM_CSCMR2_ENET_TS_CLK_SEL_MASK)
		>> MXC_CCM_CSCMR2_ENET_TS_CLK_SEL_OFFSET;
	switch (mux) {
	case 0:
		enet_ts_clk.parent = &enet_ext;
		break;
	case 1:
		enet_ts_clk.parent = &osch_clk;
		break;
	case 2:
		enet_ts_clk.parent = &audio_ext;
		break;
	case 3:
		enet_ts_clk.parent = &usb_clk;
		break;
	case 4:
		enet_ts_clk.parent = &enet_ts;
		break;
	case 5:
		enet_ts_clk.parent = &enet_div2_clk;
		break;
	case 6:
		enet_ts_clk.parent = &enet_clk;
		break;
	}
}


int __init mvf_clocks_init(unsigned long sirc, unsigned long firc,
	unsigned long sxosc, unsigned long fxosc)
{
	__iomem void *base;
	int i;
	struct clk *old_parent;

	internal_low_reference = sirc;
	internal_high_reference = firc;
	external_low_reference = sxosc;
	external_high_reference = fxosc;

	apll_base = ioremap(MVF_ANADIG_BASE_ADDR, SZ_4K);

	clk_tree_init();

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		clk_debug_register(lookups[i].clk);
	}

	/* keep correct count. */
	clk_enable(&cpu_clk);
	clk_enable(&sys_clk);
	clk_enable(&plat_bus_clk);
	clk_enable(&ips_bus_clk);
	clk_enable(&ddrc_clk);

	/* setup enet rmii clock 50MHz */
	old_parent = clk_get_parent(&enet_rmii_clk);
#if 0 //ENET PLL5 Main clock 50MHz
	if (old_parent != &enet_clk)
		clk_set_parent(&enet_rmii_clk, &enet_clk);
#else //ENET External clock 50MHz
	if (old_parent != &enet_ext)
		clk_set_parent(&enet_rmii_clk, &enet_ext);
#endif

	/* Disable un-necessary PFDs & PLLs */
	if (pll2_pfd1.usecount == 0)
		pll2_pfd1.disable(&pll2_pfd1);
	if (pll2_pfd2.usecount == 0)
		pll2_pfd2.disable(&pll2_pfd2);
	if (pll2_pfd3.usecount == 0)
		pll2_pfd3.disable(&pll2_pfd3);
	if (pll2_pfd4.usecount == 0)
		pll2_pfd4.disable(&pll2_pfd4);

	pll4_audio_main_clk.disable(&pll4_audio_main_clk);
	pll6_video_main_clk.disable(&pll6_video_main_clk);
	pll_480_usb2_main_clk.disable(&pll_480_usb2_main_clk);
	pll5_enet_main_clk.disable(&pll5_enet_main_clk);


	/* Initialize Audio and Video PLLs to valid frequency (650MHz). */
	clk_set_rate(&pll4_audio_main_clk, 650000000);
	clk_set_rate(&pll6_video_main_clk, 650000000);

#if 0
	mvf_cpu_op_init();
	cpu_op_tbl = get_cpu_op(&cpu_op_nr);
#endif

	/* Gate off all possible clocks */
	__raw_writel(3 << MXC_CCM_CCPGRx_PPCG0_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG1_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG2_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG3_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG6_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG9_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG10_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG11_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG12_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG13_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG14_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG15_OFFSET,
		     MXC_CCM_CCPGR0);
	__raw_writel(3 << MXC_CCM_CCPGRx_PPCG0_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG1_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG2_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG3_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG4_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG5_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG7_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG8_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG9_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG13_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG14_OFFSET,
		     MXC_CCM_CCPGR1);
	__raw_writel(3 << MXC_CCM_CCPGRx_PPCG0_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG7_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG8_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG9_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG10_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG12_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG14_OFFSET,
		     MXC_CCM_CCPGR2);
	__raw_writel(3 << MXC_CCM_CCPGRx_PPCG0_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG1_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG2_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG3_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG4_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG5_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG6_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG8_OFFSET |
		     3 << MXC_CCM_CCPGRx_PPCG9_OFFSET,
		     MXC_CCM_CCPGR3);

	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG3_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG5_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR0);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG3_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR1);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET,
		     MXC_CCM_CCGR2);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR3);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG5_OFFSET |
		     3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR4);
	__raw_writel(3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG5_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET,
		     MXC_CCM_CCGR6);
	__raw_writel(3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET,
		     MXC_CCM_CCGR7);
	__raw_writel(3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR8);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG5_OFFSET |
		     3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR9);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG3_OFFSET |
		     3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     3 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG14_OFFSET |
		     3 << MXC_CCM_CCGRx_CG15_OFFSET,
		     MXC_CCM_CCGR10);
	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
		     3 << MXC_CCM_CCGRx_CG1_OFFSET |
		     3 << MXC_CCM_CCGRx_CG2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG3_OFFSET |
		     3 << MXC_CCM_CCGRx_CG4_OFFSET |
		     3 << MXC_CCM_CCGRx_CG5_OFFSET |
		     3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET,
		     MXC_CCM_CCGR11);

	base = MVF_IO_ADDRESS(MVF_CA5_SCU_GIC_BASE_ADDR + 0x200);
	mvf_timer_init(&ca5_scu_clk, base, IRQ_GLOBALTIMER);

	lp_high_freq = 0;
	lp_med_freq = 0;

	return 0;
}
