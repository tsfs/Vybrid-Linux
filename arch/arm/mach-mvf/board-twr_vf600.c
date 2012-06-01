/*
 * based on arch/arm/mach-mx6/board-mx6q_arm2.c
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
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
//#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <sound/pcm.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mipi_csi2.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

//#include "usb.h"
#include "devices-mvf.h"
#include "crm_regs.h"
//#include "cpu_op-mvf.h"
#include "board-twr_vf600.h"

#if 0 //FIXME
void __init early_console_setup(unsigned long base, struct clk *clk);

static const struct imxuart_platform_data mx6_arm2_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static inline void twr_vf600_init_uart(void)
{
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(1, &mx6_arm2_uart1_data);
}
#endif //FIXME

//FIXME
static int twr_vf600_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	return 0;
}

static int twr_vf600_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set AR8031 debug reg 0xb to hibernate power*/
	phy_write(phydev, 0x1d, 0xb);
	val = phy_read(phydev, 0x1e);

	val |= 0x8000;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init			= twr_vf600_fec_phy_init,
	.power_hibernate	= twr_vf600_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
};

static void twr_vf600_suspend_enter(void)
{
	/* suspend preparation */
}

static void twr_vf600_suspend_exit(void)
{
	/* resmue resore */
}
static const struct pm_platform_data twr_vf600_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= twr_vf600_suspend_enter,
	.suspend_exit	= twr_vf600_suspend_exit,
};

static void __init twr_vf600_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

/*
 * Board specific initialization.
 */
static void __init twr_vf600_init(void)
{
	iomux_vmvf_cfg_t *common_pads = NULL;
	int common_pads_cnt;

	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	common_pads = twr_vf6xx_pads;
	common_pads_cnt = ARRAY_SIZE(twr_vf6xx_pads);

	BUG_ON(!common_pads);
	mxc_iomux_vmvf_setup_multiple_pads(common_pads, common_pads_cnt);

	/*
	 * IEEE-1588 ts_clk, S/PDIF in and i2c3 are mutually exclusive
	 * because all of them use GPIO_16.
	 * S/PDIF out and can1 stby are mutually exclusive because both
	 * use GPIO_17.
	 */
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	//mxc_iomux_set_gpr_register(1, 21, 1, 1);

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */

#if 0
	twr_vf600_init_uart();
	mvf_add_imx_snvs_rtc();
#endif
	mvf_init_fec(fec_data);

#if 0
	vf6xx_add_pm_imx(0, &twr_vf600_pm_data);
	mvf_add_imx2_wdt(0, NULL);
	mvf_add_dma();
#endif
}

//extern void __iomem *twd_base;
static void __init mvf_timer_init(void)
{
	struct clk *uart_clk;
#if 0 //FIXME
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
#endif
	mvf_clocks_init(128000, 24000000, 32000, 24000000);

#if 1 //FIXME
	uart_clk = clk_get_sys("imx-uart.1", NULL);
	early_console_setup(MVF_UART1_BASE_ADDR, uart_clk);
#else
	uart_clk = clk_get_sys("imx-uart.2", NULL);
	early_console_setup(MVF_UART2_BASE_ADDR, uart_clk);
#endif
}

static struct sys_timer mxc_timer = {
	.init   = mvf_timer_init,
};

#if 0 //FIXME
static void __init twr_vf600_reserve(void)
{
	phys_addr_t phys;

	if (imx6_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_free(phys, imx6_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
}
#endif

MACHINE_START(TWR_VF600, "Freescale MVF TWR-VF600 Board")
	.boot_params	= MVF_PHYS_OFFSET + 0x100,
	.fixup		= twr_vf600_fixup,
	.map_io		= mvf_map_io,
	.init_irq	= mvf_init_irq,
	.init_machine	= twr_vf600_init,
	.timer		= &mxc_timer,
	//.reserve	= twr_vf600_reserve,
MACHINE_END
