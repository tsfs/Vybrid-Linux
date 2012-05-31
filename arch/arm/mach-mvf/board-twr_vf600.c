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
/* GPIO PIN, sort by PORT/BIT */
#define MX6_ARM2_LDB_BACKLIGHT		IMX_GPIO_NR(1, 9)
#define MX6_ARM2_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define MX6_ARM2_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define MX6_ARM2_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define MX6_ARM2_DISP0_PWR		IMX_GPIO_NR(3, 24)
#define MX6_ARM2_DISP0_I2C_EN		IMX_GPIO_NR(3, 28)
#define MX6_ARM2_CAP_TCH_INT		IMX_GPIO_NR(3, 31)
#define MX6_ARM2_DISP0_DET_INT		IMX_GPIO_NR(3, 31)
#define MX6_ARM2_CSI0_RST		IMX_GPIO_NR(4, 5)
#define MX6_ARM2_DISP0_RESET		IMX_GPIO_NR(5, 0)
#define MX6_ARM2_CSI0_PWN		IMX_GPIO_NR(5, 23)
#define MX6_ARM2_CAN2_EN		IMX_GPIO_NR(5, 24)
#define MX6_ARM2_CSI0_RST_TVIN		IMX_GPIO_NR(5, 25)
#define MX6_ARM2_SD3_CD			IMX_GPIO_NR(6, 11)
#define MX6_ARM2_SD3_WP			IMX_GPIO_NR(6, 14)
#define MX6_ARM2_CAN1_STBY		IMX_GPIO_NR(7, 12)
#define MX6_ARM2_CAN1_EN		IMX_GPIO_NR(7, 13)
#define MX6_ARM2_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6_ARM2_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)
#define MX6DL_ARM2_EPDC_SDDO_0		IMX_GPIO_NR(2, 22)
#define MX6DL_ARM2_EPDC_SDDO_1		IMX_GPIO_NR(3, 10)
#define MX6DL_ARM2_EPDC_SDDO_2		IMX_GPIO_NR(3, 12)
#define MX6DL_ARM2_EPDC_SDDO_3		IMX_GPIO_NR(3, 11)
#define MX6DL_ARM2_EPDC_SDDO_4		IMX_GPIO_NR(2, 27)
#define MX6DL_ARM2_EPDC_SDDO_5		IMX_GPIO_NR(2, 30)
#define MX6DL_ARM2_EPDC_SDDO_6		IMX_GPIO_NR(2, 23)
#define MX6DL_ARM2_EPDC_SDDO_7		IMX_GPIO_NR(2, 26)
#define MX6DL_ARM2_EPDC_SDDO_8		IMX_GPIO_NR(2, 24)
#define MX6DL_ARM2_EPDC_SDDO_9		IMX_GPIO_NR(3, 15)
#define MX6DL_ARM2_EPDC_SDDO_10		IMX_GPIO_NR(3, 16)
#define MX6DL_ARM2_EPDC_SDDO_11		IMX_GPIO_NR(3, 23)
#define MX6DL_ARM2_EPDC_SDDO_12		IMX_GPIO_NR(3, 19)
#define MX6DL_ARM2_EPDC_SDDO_13		IMX_GPIO_NR(3, 13)
#define MX6DL_ARM2_EPDC_SDDO_14		IMX_GPIO_NR(3, 14)
#define MX6DL_ARM2_EPDC_SDDO_15		IMX_GPIO_NR(5, 2)
#define MX6DL_ARM2_EPDC_GDCLK		IMX_GPIO_NR(2, 17)
#define MX6DL_ARM2_EPDC_GDSP		IMX_GPIO_NR(2, 16)
#define MX6DL_ARM2_EPDC_GDOE		IMX_GPIO_NR(6, 6)
#define MX6DL_ARM2_EPDC_GDRL		IMX_GPIO_NR(5, 4)
#define MX6DL_ARM2_EPDC_SDCLK		IMX_GPIO_NR(3, 31)
#define MX6DL_ARM2_EPDC_SDOEZ		IMX_GPIO_NR(3, 30)
#define MX6DL_ARM2_EPDC_SDOED		IMX_GPIO_NR(3, 26)
#define MX6DL_ARM2_EPDC_SDOE		IMX_GPIO_NR(3, 27)
#define MX6DL_ARM2_EPDC_SDLE		IMX_GPIO_NR(3, 1)
#define MX6DL_ARM2_EPDC_SDCLKN		IMX_GPIO_NR(3, 0)
#define MX6DL_ARM2_EPDC_SDSHR		IMX_GPIO_NR(2, 29)
#define MX6DL_ARM2_EPDC_PWRCOM		IMX_GPIO_NR(2, 28)
#define MX6DL_ARM2_EPDC_PWRSTAT		IMX_GPIO_NR(2, 21)
#define MX6DL_ARM2_EPDC_PWRCTRL0	IMX_GPIO_NR(2, 20)
#define MX6DL_ARM2_EPDC_PWRCTRL1	IMX_GPIO_NR(2, 19)
#define MX6DL_ARM2_EPDC_PWRCTRL2	IMX_GPIO_NR(2, 18)
#define MX6DL_ARM2_EPDC_PWRCTRL3	IMX_GPIO_NR(3, 28)
#define MX6DL_ARM2_EPDC_BDR0		IMX_GPIO_NR(3, 2)
#define MX6DL_ARM2_EPDC_BDR1		IMX_GPIO_NR(3, 3)
#define MX6DL_ARM2_EPDC_SDCE0		IMX_GPIO_NR(3, 4)
#define MX6DL_ARM2_EPDC_SDCE1		IMX_GPIO_NR(3, 5)
#define MX6DL_ARM2_EPDC_SDCE2		IMX_GPIO_NR(3, 6)
#define MX6DL_ARM2_EPDC_SDCE3		IMX_GPIO_NR(3, 7)
#define MX6DL_ARM2_EPDC_SDCE4		IMX_GPIO_NR(3, 8)
#define MX6DL_ARM2_EPDC_SDCE5		IMX_GPIO_NR(3, 9)
#define MX6DL_ARM2_EPDC_PMIC_WAKE	IMX_GPIO_NR(2, 31)
#define MX6DL_ARM2_EPDC_PMIC_INT	IMX_GPIO_NR(2, 25)
#define MX6DL_ARM2_EPDC_VCOM		IMX_GPIO_NR(3, 17)

#define MX6_ARM2_IO_EXP_GPIO1(x)	(MX6_ARM2_MAX7310_1_BASE_ADDR + (x))
#define MX6_ARM2_IO_EXP_GPIO2(x)	(MX6_ARM2_MAX7310_2_BASE_ADDR + (x))

#define MX6_ARM2_CAN2_STBY		MX6_ARM2_IO_EXP_GPIO2(1)


#define BMCR_PDOWN			0x0800 /* PHY Powerdown */

void __init early_console_setup(unsigned long base, struct clk *clk);
static int spdif_en;
static int flexcan_en;
static int disable_mipi_dsi;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern char *gp_reg_id;
extern int epdc_enabled;
extern void mx6_cpu_regulator_init(void);
static int max17135_regulator_init(struct max17135 *max17135);


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

static struct mxc_dvfs_platform_data arm2_dvfscore_data = {
	.reg_id			= "cpu_vddgp",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};
#endif //FIXME

static void __init twr_vf600_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

/*
 * Board specific initialization.
 */
static void __init twr_vf600_init(void)
{
#if 0 //FIXME
	int i;
	int ret;
	iomux_vmf_cfg_t *common_pads = NULL;
	int common_pads_cnt;

	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	common_pads = mx6q_arm2_pads;
	common_pads_cnt = ARRAY_SIZE(mx6q_arm2_pads);

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
	mxc_iomux_set_gpr_register(1, 21, 1, 1);

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */

	twr_vf600_init_uart();
	mvf_add_imx_snvs_rtc();
	mvf_init_fec(fec_data);

	imx6q_add_pm_imx(0, &twr_vf600_pm_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6_arm2_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6_arm2_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	imx6q_add_vpu();
	twr_vf600_init_usb();
	twr_vf600_init_audio();
	platform_device_register(&arm2_vmmc_reg_devices);
	mx6_cpu_regulator_init();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(MX6_ARM2_DISP0_RESET, "disp0-reset");
	gpio_direction_output(MX6_ARM2_DISP0_RESET, 0);

	/* DISP0 I2C enable */
	if (!disable_mipi_dsi) {
		gpio_request(MX6_ARM2_DISP0_I2C_EN, "disp0-i2c");
		gpio_direction_output(MX6_ARM2_DISP0_I2C_EN, 0);
	}
	gpio_request(MX6_ARM2_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(MX6_ARM2_DISP0_PWR, 1);

	gpio_request(MX6_ARM2_LDB_BACKLIGHT, "ldb-backlight");
	gpio_direction_output(MX6_ARM2_LDB_BACKLIGHT, 1);
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&arm2_dvfscore_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_arm2_pwm_backlight_data);

	if (spdif_en) {
		mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
		clk_put(mxc_spdif_data.spdif_core_clk);
		imx6q_add_spdif(&mxc_spdif_data);
		imx6q_add_spdif_dai();
		imx6q_add_spdif_audio_device();
	} else if (flexcan_en) {
		ret = gpio_request_array(mx6_flexcan_gpios,
				ARRAY_SIZE(mx6_flexcan_gpios));
		if (ret) {
			pr_err("failed to request flexcan-gpios: %d\n", ret);
		} else {
			imx6q_add_flexcan0(&mx6_arm2_flexcan_pdata[0]);
			imx6q_add_flexcan1(&mx6_arm2_flexcan_pdata[1]);
		}
	}

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&mx6_arm2_mlb150_data);

	if (cpu_is_mx6dl() && epdc_enabled) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
		mxc_register_device(&max17135_sensor_device, NULL);
		imx6dl_add_imx_epdc(&epdc_data);
	}
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
