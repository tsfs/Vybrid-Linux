/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ASM_ARCH_MVF_GPIO_H__
#define __ASM_ARCH_MVF_GPIO_H__

#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <asm-generic/gpio.h>


/* There's a off-by-one betweem the gpio bank number and the gpiochip */
/* range e.g. GPIO_1_5 is gpio 5 under linux */
#define MVF_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))

/* use gpiolib dispatchers */
#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep

#define gpio_to_irq(gpio)	(MXC_GPIO_IRQ_START + (gpio))
#define irq_to_gpio(irq)	((irq) - MXC_GPIO_IRQ_START)

struct mvf_gpio_port {
	void __iomem *gbase;    /* GPIO Register Base Address */
	                       /*      Use for Value  */ 
	void __iomem *pbase;   /* PORT Register Base Address */
	                       /*      Use for Interrupts */
	void __iomem *ibase;   /* IOMUX Register Base Address */
	                       /* IOMUX USE For Direction */
	int irq;
	int virtual_irq_start;
	struct gpio_chip chip;
	u32 both_edges;
	spinlock_t lock;
};

#define DEFINE_IMX_GPIO_PORT_IRQ_HIGH(soc, _id, _hwid, _irq, _irq_high)	\
	{								\
		.chip.label = "gpio-" #_id,				\
		.irq = _irq,						\
		.irq_high = _irq_high,					\
		.base = soc ## _IO_ADDRESS(				\
				soc ## _GPIO ## _hwid ## _BASE_ADDR),	\
		.virtual_irq_start = MXC_GPIO_IRQ_START + (_id) * 32,	\
	}

#define DEFINE_IMX_GPIO_PORT_IRQ(soc, _id, _hwid, _irq)			\
	DEFINE_IMX_GPIO_PORT_IRQ_HIGH(soc, _id, _hwid, _irq, 0)
#define DEFINE_IMX_GPIO_PORT(soc, _id, _hwid)				\
	DEFINE_IMX_GPIO_PORT_IRQ(soc, _id, _hwid, 0)

int mvf_gpio_init(struct mvf_gpio_port*, int);

#define _PORT_PCR(x,y)   (((x)-1)<<12 | (y)<<2) /* Change to PORT ADDR */
#define GPIO_NUM(x,y)  (((x)-1)<<8 | (y))
#define GPIO(x,y)  _PORT_PCR(x,y)<<16 | GPIO_ADDR(x,y)

#define PORT_A   1
#define PORT_B   2
#define PORT_C   3
#define PORT_D   4
#define PORT_E   5

#define GPIO_PDOR   0x00
#define GPIO_PSOR   0x04
#define GPIO_PCOR   0x08
#define GPIO_PTOR   0x0c
#define GPIO_PDIR   0x10

#define PORT_PCR(n)  ((n)<<2)
#define PORT_PCR_ISF   (1<<24)

#define PCR_IRQC_MASK         ~(0xf<<16)
#define PCR_IRQC_NONE         (0x0<<16)  /* DISABLE */
#define PCR_IRQC_LEVEL_LOW    (0x8<<16)
#define PCR_IRQC_RISE_EDGE    (0x9<<16)
#define PCR_IRQC_FALL_EDGE    (0xA<<16)
#define PCR_IRQC_BOTH_EDGE    (0xB<<16)
#define PCR_IRQC_LEVEL_HIGH   (0xC<<16)

#define PORT_ISFR     0xa0
#define PORT_DFER     0xc0
#define PORT_DFCR     0xc4
#define PORT_DFWR     0xc8


#endif
