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

/* GPIO PIN, sort by PORT/BIT */
#define TWR_VF600_GPIO10		MVF_GPIO_NR(1, 10)	//PTA20
#define TWR_VF600_GPIO20		MVF_GPIO_NR(1, 20)	//PTA30
#define TWR_VF600_GPIO21		MVF_GPIO_NR(1, 21)	//PTA31
#define TWR_VF600_GPIO28		MVF_GPIO_NR(1, 28)	//PTB6
#define TWR_VF600_GPIO29		MVF_GPIO_NR(1, 29)	//PTB7
#define TWR_VF600_GPIO30		MVF_GPIO_NR(1, 30)	//PTB8
#define TWR_VF600_GPIO31		MVF_GPIO_NR(1, 31)	//PTB9
#define TWR_VF600_GPIO32		MVF_GPIO_NR(2, 0)	//PTB10
#define TWR_VF600_GPIO33		MVF_GPIO_NR(2, 1) 	//PTB11
#define TWR_VF600_GPIO34		MVF_GPIO_NR(2, 2)	//PTB12
#define TWR_VF600_GPIO38		MVF_GPIO_NR(2, 6)	//PTB16
#define TWR_VF600_GPIO39		MVF_GPIO_NR(2, 7)	//PTB17
#define TWR_VF600_GPIO85		MVF_GPIO_NR(3, 21)	//PTD6
#define TWR_VF600_GPIO92		MVF_GPIO_NR(3, 28)	//PTD13
#define TWR_VF600_GPIO93		MVF_GPIO_NR(3, 29)	//PTB23
#define TWR_VF600_GPIO96		MVF_GPIO_NR(4, 0)	//PTB26
#define TWR_VF600_GPIO98		MVF_GPIO_NR(4, 2)	//PTB28
#define TWR_VF600_GPIO102		MVF_GPIO_NR(4, 6)	//PTC29
#define TWR_VF600_GPIO103		MVF_GPIO_NR(4, 7)	//PTC30
#define TWR_VF600_GPIO104		MVF_GPIO_NR(4, 8)	//PTC31
#define TWR_VF600_GPIO108		MVF_GPIO_NR(4, 12)	//PTE3
#define TWR_VF600_GPIO134		MVF_GPIO_NR(5, 6)	//PTA7

#define HOME                    TWR_VF600_GPIO38
#define BACK                    TWR_VF600_GPIO39
#define MENU                    TWR_VF600_GPIO29
#define VOLUP                   TWR_VF600_GPIO30
#define VOLDOWN                 TWR_VF600_GPIO31

void __init early_console_setup(unsigned long base, struct clk *clk);

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
	{															\
		.gpio		= gpio_num,									\
		.type		= EV_KEY,								\
		.code		= ev_code,								\
		.active_low	= act_low,								\
		.desc		= "btn " descr,							\
		.wakeup		= wake,									\
	}
static struct gpio_keys_button mvf_buttons[] = {
	GPIO_BUTTON(HOME,   KEY_HOME,      1,"home",       0),
	GPIO_BUTTON(BACK,   KEY_BACK,      1,"back",       0),
	GPIO_BUTTON(MENU,   KEY_MENU,      1,"menu",       0),
	GPIO_BUTTON(VOLUP,  KEY_VOLUMEUP,  1,"volume-up",  0),
	GPIO_BUTTON(VOLDOWN,KEY_VOLUMEDOWN,1,"volume-down",0),
};
/* For Keypad dirver*/
static struct gpio_keys_platform_data mvf_button_data = {
	.buttons = mvf_buttons,
	.nbuttons = ARRAY_SIZE(mvf_buttons),
};
static struct platform_device mvf_android_button_device = {
	.name   = "gpio-keys",
	.id     = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &mvf_button_data,
	},
};
static void __init vf600_add_android_device_buttons(void)
{
	platform_device_register(&mvf_android_button_device);
}
#else
static void __init vf600_add_android_device_buttons(void) {}
#endif

#if 0 //FIXME
static const struct imxuart_platform_data mx6_arm2_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};
#endif //FIXME

static inline void twr_vf600_init_uart(void)
{
	mvf_add_imx_uart(0, NULL);
	mvf_add_imx_uart(1, NULL);
	mvf_add_imx_uart(2, NULL);
	mvf_add_imx_uart(3, NULL);
	mvf_add_imx_uart(4, NULL);
	mvf_add_imx_uart(5, NULL);
}
#if 0
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
#endif
static struct fec_platform_data fec_data __initdata = {
	.init			= NULL,
	.power_hibernate	= NULL,
	.phy			= PHY_INTERFACE_MODE_RMII,
};

static struct resource edma_resources0[] = {
	[0] = {
		.start = MVF_DMA0_BASE_ADDR,
		.end = MVF_DMA0_BASE_ADDR + 0x2000-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_DMA0,
		.end = MXC_INT_DMA0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = MXC_INT_DMA0_ERROR,
		.end = MXC_INT_DMA0_ERROR,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device edma_device0 = {
       .name = "mvf-edma",
       .id = 0,
       .num_resources = 3,
       .resource = edma_resources0,
};

static struct resource edma_resources1[] = {
	[0] = {
		.start = MVF_DMA1_BASE_ADDR,
		.end = MVF_DMA1_BASE_ADDR + 0x2000,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_DMA1,
		.end = MXC_INT_DMA1,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = MXC_INT_DMA1_ERROR,
		.end = MXC_INT_DMA1_ERROR,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device edma_device1 = {
       .name = "mvf-edma",
       .id = 1,
       .num_resources = 3,
       .resource = edma_resources1,
};

//
//	Timer resources
//
//	pit
static struct resource pit_resources[] = {
	[0] = {
		.start = MVF_PIT_BASE_ADDR,
		.end = MVF_PIT_BASE_ADDR + 0x1000-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_PIT,
		.end = MXC_INT_PIT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device pit_device = {
       .name = "mvf-pit",
       .id = 0,
       .num_resources = 2,
       .resource = pit_resources,
};
#if 1
//	ftm 0
static struct resource ftm0_resources[] = {
	[0] = {
		.start = MVF_FTM0_BASE_ADDR,
		.end = MVF_FTM0_BASE_ADDR + 0x1000-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_FTM0,
		.end = MXC_INT_FTM0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ftm0_device = {
       .name = "mvf-ftm",
       .id = 0,
       .num_resources = 2,
       .resource = ftm0_resources,
};
#endif
#if 1
//	ftm 1
static struct resource ftm1_resources[] = {
	[0] = {
		.start = MVF_FTM1_BASE_ADDR,
		.end = MVF_FTM1_BASE_ADDR + 0x1000-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_FTM1,
		.end = MXC_INT_FTM1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ftm1_device = {
       .name = "mvf-ftm",
       .id = 1,
       .num_resources = 2,
       .resource = ftm1_resources,
};
#endif

//	pit
static struct resource lpt_resources[] = {
	[0] = {
		.start = MVF_LPTMR_BASE_ADDR,
		.end = MVF_LPTMR_BASE_ADDR + 0x1000-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_LP_TIMER0,
		.end = MXC_INT_LP_TIMER0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device lpt_device = {
       .name = "mvf-lpt",
       .id = 0,
       .num_resources = 2,
       .resource = lpt_resources,
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

	twr_vf600_init_uart();
	vf6xx_add_imx_snvs_rtc();
	mvf_init_fec(fec_data);

	platform_device_register(&edma_device0);
	platform_device_register(&edma_device1);
	platform_device_register(&pit_device);
	platform_device_register(&ftm0_device);
	platform_device_register(&ftm1_device);
	platform_device_register(&lpt_device);
	vf600_add_android_device_buttons();


#if 0
	vf6xx_add_pm_imx(0, &twr_vf600_pm_data);
	mvf_add_imx2_wdt(0, NULL);
	mvf_add_dma();
#endif
}

//extern void __iomem *twd_base;
static void __init vf600_timer_init(void)
{
	struct clk *uart_clk;
#if 0 //FIXME
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
#endif
	mvf_clocks_init(128000, 24000000, 32000, 24000000);

	uart_clk = clk_get_sys("mvf-uart.1", NULL);
	early_console_setup(MVF_UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = vf600_timer_init,
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
