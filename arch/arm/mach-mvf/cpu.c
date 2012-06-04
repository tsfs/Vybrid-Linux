/*
 * based on arch/arm/mach-mx6/cpu.c
 *
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iram_alloc.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/map.h>

#include "crm_regs.h"
#include "cpu_op-mvf.h"


#if 0 //FIXME
void *mvf_wait_in_iram_base;
void (*mvf_wait_in_iram)(void);
extern void mvf_wait(void);


struct cpu_op *(*get_cpu_op)(int *op);
#endif
bool enable_wait_mode;
u32 arm_max_freq = CPU_AT_450MHz;

void __iomem *gpc_base;
void __iomem *ccm_base;

static int cpu_silicon_rev = -1;
#define SI_REV_OFFSET 	0x48

static int get_vf6xx_srev(void)
{
	void __iomem *romcp = ioremap(IROM_BASE_ADDR, SZ_8K);
	u32 rev;

	if (!romcp) {
		cpu_silicon_rev = -EINVAL;
		return 0;
	}

	rev = __raw_readl(romcp + SI_REV_OFFSET);
	rev &= 0xff;

	iounmap(romcp);
	switch (rev) {
	case 0x02:
		cpu_silicon_rev = IMX_CHIP_REVISION_1_1;
		break;
	case 0x10:
		break;
	case 0x20:
		cpu_silicon_rev = IMX_CHIP_REVISION_3_0;
		break;
	default:
		cpu_silicon_rev = IMX_CHIP_REVISION_1_0;
		break;
	}
	return 0;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not a mx50
 */
int vf6xx_revision(void)
{
	if (!cpu_is_vf6xx())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = get_vf6xx_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(vf6xx_revision);

static int __init post_cpu_init(void)
{
	unsigned int reg;
	void __iomem *base;
	unsigned long iram_paddr, cpaddr;


#if 0 //FIXME
	iram_init(MVF_IRAM0_BASE_ADDR, MVF_IRAM0_SIZE);	//FIXME
#endif

	base = ioremap(AIPS0_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x20);
	__raw_writel(0x0, base + 0x24);
	__raw_writel(0x0, base + 0x28);
	__raw_writel(0x0, base + 0x2C);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	__raw_writel(0x0, base + 0x50);
	__raw_writel(0x0, base + 0x54);
	__raw_writel(0x0, base + 0x58);
	__raw_writel(0x0, base + 0x5C);
	__raw_writel(0x0, base + 0x60);
	__raw_writel(0x0, base + 0x64);
	__raw_writel(0x0, base + 0x68);
	__raw_writel(0x0, base + 0x6C);
	reg = __raw_readl(base + 0x80) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x80);
	iounmap(base);

	base = ioremap(AIPS1_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x20);
	__raw_writel(0x0, base + 0x24);
	__raw_writel(0x0, base + 0x28);
	__raw_writel(0x0, base + 0x2C);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	__raw_writel(0x0, base + 0x50);
	__raw_writel(0x0, base + 0x54);
	__raw_writel(0x0, base + 0x58);
	__raw_writel(0x0, base + 0x5C);
	__raw_writel(0x0, base + 0x60);
	__raw_writel(0x0, base + 0x64);
	__raw_writel(0x0, base + 0x68);
	__raw_writel(0x0, base + 0x6C);
	reg = __raw_readl(base + 0x80) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x80);
	iounmap(base);

#if 0 //FIXME
	if (enable_wait_mode) {
		/* Allow SCU_CLK to be disabled when all cores are in WFI*/
		base = MVF_IO_ADDRESS(MVF_CA5_SCU_GIC_BASE_ADDR);
		reg = __raw_readl(base);
		reg |= 0x20;
		__raw_writel(reg, base);
	}

	/* Disable SRC warm reset to work aound system reboot issue */
	base = MVF_IO_ADDRESS(MVF_ASRC_BASE_ADDR);
	reg = __raw_readl(base);
	reg &= ~0x1;
	__raw_writel(reg, base);

	/* Allocate IRAM for WAIT code. */
	/* Move wait routine into iRAM */
	cpaddr = (unsigned long)iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	mvf_wait_in_iram_base = __arm_ioremap(iram_paddr, SZ_4K,
					  MT_MEMORY_NONCACHED);
	pr_info("cpaddr = %x wait_iram_base=%x\n",
		(unsigned int)cpaddr, (unsigned int)mvf_wait_in_iram_base);

	/*
	 * Need to run the suspend code from IRAM as the DDR needs
	 * to be put into low power mode manually.
	 */
	memcpy((void *)cpaddr, mvf_wait, SZ_4K);
	mvf_wait_in_iram = (void *)mvf_wait_in_iram_base;
#endif

	gpc_base = MVF_IO_ADDRESS(MVF_GPC_BASE_ADDR);
	ccm_base = MVF_IO_ADDRESS(MVF_CCM_BASE_ADDR);

	return 0;
}
postcore_initcall(post_cpu_init);

#if 0 //FIXME
static int __init enable_wait(char *p)
{
	if (memcmp(p, "on", 2) == 0) {
		enable_wait_mode = true;
		p += 2;
	} else if (memcmp(p, "off", 3) == 0) {
		enable_wait_mode = false;
		p += 3;
	}
	return 0;
}
early_param("enable_wait_mode", enable_wait);

static int __init arm_core_max(char *p)
{
	if (memcmp(p, "450", 3) == 0) {
		arm_max_freq = CPU_AT_450MHz;
		p += 3;
	}
	return 0;
}

early_param("arm_freq", arm_core_max);
#endif

