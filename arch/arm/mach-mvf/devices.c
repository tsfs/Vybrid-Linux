/*
 * based on arch/arm/mach-mx6/devices.c
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/ipu.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/uio_driver.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

static struct mvf_gpio_port mvf_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.gbase = MVF_IO_ADDRESS(GPIO0_BASE_ADDR),
		.pbase = MVF_IO_ADDRESS(MVF_PORT_0_BASE_ADDR),
		.ibase = MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR),
		.pad = 0,
		.irq =MXC_INT_GPIOA,
		.virtual_irq_start = MXC_GPIO_IRQ_START
	},
	{
		.chip.label = "gpio-1",
		.gbase = MVF_IO_ADDRESS(GPIO1_BASE_ADDR),
		.pbase = MVF_IO_ADDRESS(MVF_PORT_1_BASE_ADDR),
		.ibase = MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR),
		.pad = 32*1,
		.irq = MXC_INT_GPIOB,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 1
	},
	{
		.chip.label = "gpio-2",
		.gbase = MVF_IO_ADDRESS(GPIO2_BASE_ADDR),
		.pbase = MVF_IO_ADDRESS(MVF_PORT_2_BASE_ADDR),
		.ibase = MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR),
		.pad = 32*2,
		.irq = MXC_INT_GPIOC,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	},
	{
		.chip.label = "gpio-3",
		.gbase = MVF_IO_ADDRESS(GPIO3_BASE_ADDR),
		.pbase = MVF_IO_ADDRESS(MVF_PORT_3_BASE_ADDR),
		.ibase = MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR),
		.irq = MXC_INT_GPIOD,
		//.irq_high = MXC_INT_GPIO4_INT31_16_NUM,
		.pad = 32*3,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	},
	{
		.chip.label = "gpio-4",
		.gbase = MVF_IO_ADDRESS(GPIO4_BASE_ADDR),
		.pbase = MVF_IO_ADDRESS(MVF_PORT_4_BASE_ADDR),
		.ibase = MVF_IO_ADDRESS(MVF_IOMUXC_BASE_ADDR),
		.irq = MXC_INT_GPIOE,
		.pad = 32*4,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 4
	},
};

int mvf_register_gpios(void)
{
	/* 5 ports for MVF */
	return mvf_gpio_init(mvf_gpio_ports, 5);
}
