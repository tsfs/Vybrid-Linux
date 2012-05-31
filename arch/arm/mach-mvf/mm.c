/*
 * based on arch/arm/mach-mx6/mm.c
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

/*
 * Create static mapping between physical to virtual memory.
 */

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/mach/map.h>
#include <mach/iomux-vmvf.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/hardware/cache-l2x0.h>

/*!
 * This structure defines the Faraday memory map.
 */
static struct map_desc mvf_io_desc[] __initdata = {
	imx_map_entry(MVF, AIPS0, MT_DEVICE),
	imx_map_entry(MVF, AIPS1, MT_DEVICE),
};

static void mvf_set_cpu_type(void)
{
	//FIXME
#ifdef CONFIG_SOC_VF6XX
	mxc_set_cpu_type(MXC_CPU_VF6XX);
#endif
}

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mvf_map_io(void)
{
	iotable_init(mvf_io_desc, ARRAY_SIZE(mvf_io_desc));
	mxc_iomux_vmvf_init(MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR));
	mxc_arch_reset_init(MVF_IO_ADDRESS(MVF_WDOC_A5_BASE_ADDR));
	mvf_set_cpu_type();
#if 0
	mxc_cpu_lp_set(WAIT_CLOCKED);
#endif
}

#ifdef CONFIG_CACHE_L2X0
int mxc_init_l2x0(void)
{
	unsigned int val;

	writel(0x132, MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_TAG_LATENCY_CTRL));
	writel(0x132, MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_DATA_LATENCY_CTRL));

	val = readl(MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_PREFETCH_CTRL));
	val |= 0x40800000;
	writel(val, MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_PREFETCH_CTRL));
	val = readl(MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_POWER_CTRL));
	val |= L2X0_DYNAMIC_CLK_GATING_EN;
	val |= L2X0_STNDBY_MODE_EN;
	writel(val, MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR + L2X0_POWER_CTRL));

	l2x0_init(MVF_IO_ADDRESS(MVF_CA5_L2C_BASE_ADDR), 0x0, ~0x00000000);
	return 0;
}


arch_initcall(mxc_init_l2x0);
#endif
