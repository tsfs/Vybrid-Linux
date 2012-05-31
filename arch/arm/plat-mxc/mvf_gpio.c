/*
 * arch/arm/plat-mxc/mvf_gpio.c
 *
 * basedon arch/arm/plat-mxc/mvf_gpio.c
 *
 * Based on code from Freescale,
 * Copyright (C) 2004-2012 Freescale Semiconductor, Inc.
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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <asm-generic/bug.h>
#include <asm/mach/irq.h>


/*
 * The controllers related to this processing are GPIO and PORT. 
 * PORT: Chapter 6. Base Addr. 0x40049000
 * GPIO: Chapter 7. Base Addr. 0x400FF000
 *
 */

static int gpio_table_size;

/* GPIO Num TO IOMUX Address
 *  bit : 15-13 (0 to 7)   PORT(a=0)
 *        12 -8 (0 to 31)  BIT
 *         7- 0 (0 to 255) IOMUX address shift 2 (mux 0x3cf)
 */
//#define GTM(x,y,z) ((x<<13)|(y<<8)|((z>>2)&0x00ff))
#define GTM(x,y,z) ((z>>2)&0x00ff)
#define PTA  0
#define PTB  1
#define PTC  3
#define PTD  4
#define PTE  5

#define NOCFG	0

static unsigned char gpio_to_mux[] = {
	/* PT_A */
	GTM(PTA, 0,NOCFG),GTM(PTA, 1,NOCFG),GTM(PTA, 2,NOCFG),GTM(PTA, 3,NOCFG),
	GTM(PTA, 4,NOCFG),GTM(PTA, 5,NOCFG),GTM(PTA, 6,0x000),GTM(PTA, 7,0x218),
	GTM(PTA, 8,0x010),GTM(PTA, 9,0x008),GTM(PTA,10,0x00c),GTM(PTA,11,0x010),
    GTM(PTA,12,0x014),GTM(PTA,13,NOCFG),GTM(PTA,14,NOCFG),GTM(PTA,15,NOCFG),
    GTM(PTA,16,0x018),GTM(PTA,17,0x01c),GTM(PTA,18,0x020),GTM(PTA,19,0x024),
    GTM(PTA,20,0x028),GTM(PTA,21,0x02c),GTM(PTA,22,0x030),GTM(PTA,23,0x034),
    GTM(PTA,24,0x038),GTM(PTA,25,0x03c),GTM(PTA,26,0x040),GTM(PTA,27,0x044),
    GTM(PTA,28,0x048),GTM(PTA,29,0x04c),GTM(PTA,30,0x050),GTM(PTA,31,0x054),
	/* PT_B */
	GTM(PTB, 0,0x058),GTM(PTB, 1,0x05c),GTM(PTB, 2,0x060),GTM(PTB, 3,0x064),
	GTM(PTB, 4,0x068),GTM(PTB, 5,0x06c),GTM(PTB, 6,0x070),GTM(PTB, 7,0x074),
	GTM(PTB, 8,0x078),GTM(PTB, 9,0x07c),GTM(PTB,10,0x080),GTM(PTB,11,0x084),
    GTM(PTB,12,0x088),GTM(PTB,13,0x08c),GTM(PTB,14,0x090),GTM(PTB,15,0x094),
    GTM(PTB,16,0x098),GTM(PTB,17,0x09c),GTM(PTB,18,0x0a0),GTM(PTB,19,0x0a4),
    GTM(PTB,20,0x0a8),GTM(PTB,21,0x0ac),GTM(PTB,22,0x0b0),GTM(PTB,23,NOCFG),
    GTM(PTB,24,NOCFG),GTM(PTB,25,0x17c),GTM(PTB,26,0x180),GTM(PTB,27,0x184),
    GTM(PTB,28,0x188),GTM(PTB,29,NOCFG),GTM(PTB,30,NOCFG),GTM(PTB,31,NOCFG),
	/* PT_C */
	GTM(PTC, 0,0x0b4),GTM(PTC, 1,0x0b8),GTM(PTC, 2,0x0bc),GTM(PTC, 3,0x0c0),
	GTM(PTC, 4,0x0c4),GTM(PTC, 5,0x0c8),GTM(PTC, 6,0x0cc),GTM(PTC, 7,0x0d0),
	GTM(PTC, 8,0x0d4),GTM(PTC, 9,0x0d8),GTM(PTC,10,0x0dc),GTM(PTC,11,0x0e0),
    GTM(PTC,12,0x0e4),GTM(PTC,13,0x0e8),GTM(PTC,14,0x0ec),GTM(PTC,15,0x0f0),
    GTM(PTC,16,0x0f4),GTM(PTC,17,0x0f8),GTM(PTC,18,NOCFG),GTM(PTC,19,NOCFG),
    GTM(PTC,20,NOCFG),GTM(PTC,21,NOCFG),GTM(PTC,22,NOCFG),GTM(PTC,23,NOCFG),
    GTM(PTC,24,NOCFG),GTM(PTC,25,NOCFG),GTM(PTC,26,0x18c),GTM(PTC,27,0x190),
    GTM(PTC,28,0x194),GTM(PTC,29,0x198),GTM(PTC,30,0x19c),GTM(PTC,31,0x1a0),
	/* PT_D */
	GTM(PTD, 0,0x13c),GTM(PTD, 1,0x140),GTM(PTD, 2,0x144),GTM(PTD, 3,0x148),
	GTM(PTD, 4,0x14c),GTM(PTD, 5,0x150),GTM(PTD, 6,0x15c),GTM(PTD, 7,0x158),
	GTM(PTD, 8,0x15c),GTM(PTD, 9,0x160),GTM(PTD,10,0x164),GTM(PTD,11,0x168),
    GTM(PTD,12,0x16c),GTM(PTD,13,0x170),GTM(PTD,14,0x174),GTM(PTD,15,0x17c),
    GTM(PTD,16,0x138),GTM(PTD,17,0x134),GTM(PTD,18,0x130),GTM(PTD,19,0x12c),
    GTM(PTD,20,0x128),GTM(PTD,21,0x124),GTM(PTD,22,0x120),GTM(PTD,23,0x11c),
    GTM(PTD,24,0x118),GTM(PTD,25,0x114),GTM(PTD,26,0x110),GTM(PTD,27,0x10c),
    GTM(PTD,28,0x108),GTM(PTD,29,0x104),GTM(PTD,30,0x100),GTM(PTD,31,0x0fc),
	/* PT_E */
	GTM(PTE, 0,0x1a4),GTM(PTE, 1,0x1a8),GTM(PTE, 2,0x1ac),GTM(PTE, 3,0x1b0),
	GTM(PTE, 4,0x1b4),GTM(PTE, 5,0x1b8),GTM(PTE, 6,0x1bc),GTM(PTE, 7,0x1c0),
	GTM(PTE, 8,0x1c4),GTM(PTE, 9,0x1c8),GTM(PTE,10,0x1cc),GTM(PTE,11,0x1d0),
    GTM(PTE,12,0x1d4),GTM(PTE,13,0x1d8),GTM(PTE,14,0x1dc),GTM(PTE,15,0x1e0),
    GTM(PTE,16,0x1e0),GTM(PTE,17,0x1e8),GTM(PTE,18,0x1ec),GTM(PTE,19,0x1f4),
    GTM(PTE,20,0x1f4),GTM(PTE,21,0x1f8),GTM(PTE,22,0x1fc),GTM(PTE,23,0x200),
    GTM(PTE,24,0x204),GTM(PTE,25,0x208),GTM(PTE,26,0x20c),GTM(PTE,27,0x210),
    GTM(PTE,28,0x214),GTM(PTE,29,NOCFG),GTM(PTE,30,NOCFG),GTM(PTE,31,NOCFG),
};
#define IOMUX_OFF(x) gpio_to_mux[x]

static struct mvf_gpio_port *mvf_gpio_ports;
/*
 * IRQ Setting: Initialize PORTx_PCRn ( see Chapter 6)
 */


/*
 * GPIO Setting: 
 *
 *  GPIOx_PDOR : Port Data Output Register
 *  GPIOx_PSOR :   Port Set Output
 *  GPIOx_PCOR :   Port Clear Output
 *  GPIOx_PTOR :   Port Toggle Output
 *  GPIOx_PDIR : Port Data input Register
 */

/*
 * Change Input/output
 *    => Use PAD Register?
 * Change 
 *
 */


/* PTx PTA to PTE */
#define GPIO_ODR    0x00
#define GPIO_IDR    0x10


/* Note: This driver assumes 32 GPIOs are handled in one register */

static void _clear_gpio_irqstatus(struct mvf_gpio_port *port, u32 index)
{
	void __iomem *adr = (void __iomem *)((u32)(port->pbase) | PORT_PCR(index));
	__raw_writel(__raw_readl(adr) | PORT_PCR_ISF, adr);
}

static void _set_gpio_irqenable(struct mvf_gpio_port *port, u32 index,
				int enable)
{
	u32 l;

	l = __raw_readl((u32)port->pbase + PORT_PCR(index));
	l &= PCR_IRQC_MASK;
	l = enable?port->int_type:PCR_IRQC_NONE;
	__raw_writel(l, (u32)port->pbase + PORT_PCR(index));
}

static void gpio_ack_irq(struct irq_data *d)
{
	u32 gpio = irq_to_gpio(d->irq);
	_clear_gpio_irqstatus(&mvf_gpio_ports[gpio / 32], gpio & 0x1f);
}

static void gpio_mask_irq(struct irq_data *d)
{
	u32 gpio = irq_to_gpio(d->irq);
	_set_gpio_irqenable(&mvf_gpio_ports[gpio / 32], gpio & 0x1f, 0);
}

static void gpio_unmask_irq(struct irq_data *d)
{
	u32 gpio = irq_to_gpio(d->irq);
	_set_gpio_irqenable(&mvf_gpio_ports[gpio / 32], gpio & 0x1f, 1);
}

static int mvf_gpio_get(struct gpio_chip *chip, unsigned offset);

static int gpio_set_irq_type(struct irq_data *d, u32 type)
{
	u32 gpio = irq_to_gpio(d->irq);
	struct mvf_gpio_port *port = &mvf_gpio_ports[gpio / 32];
	u32 val;
	int edge;
	void __iomem *reg = port->pbase;

	port->both_edges &= ~(1 << (gpio & 0x1f));
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = PCR_IRQC_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = PCR_IRQC_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		edge = PCR_IRQC_BOTH_EDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = PCR_IRQC_LEVEL_LOW;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = PCR_IRQC_LEVEL_HIGH;
		break;
	default:
		return -EINVAL;
	}
	
	reg = (void __iomem *)((u32)reg | PORT_PCR(gpio%32));
	val = __raw_readl(reg) & PCR_IRQC_MASK;
	port->int_type = edge;
	__raw_writel(val | (edge <<16), reg);
	_clear_gpio_irqstatus(port, gpio & 0x1f);

	return 0;
}


/* handle 32 interrupts in one status register */
static void mvf_gpio_irq_handler(struct mvf_gpio_port *port, u32 irq_stat)
{
	u32 gpio_irq_no_base = port->virtual_irq_start;

	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;
		generic_handle_irq(gpio_irq_no_base + irqoffset);

		irq_stat &= ~(1 << irqoffset);
	}
}

static void mvf_gpio_irq_chain_handler(u32 irq, struct irq_desc *desc)
{
	u32 irq_stat;
	struct mvf_gpio_port *port = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);

	irq_stat = __raw_readl((void __iomem *)((u32)(port->pbase) + PORT_ISFR));

	mvf_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);	

}

/*
 * Set interrupt number "irq" in the GPIO as a wake-up source.
 * While system is running, all registered GPIO interrupts need to have
 * wake-up enabled. When system is suspended, only selected GPIO interrupts
 * need to have wake-up enabled.
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * @return       This function returns 0 on success.
 */
static int gpio_set_wake_irq(struct irq_data *d, u32 enable)
{
	u32 gpio = irq_to_gpio(d->irq);
	struct mvf_gpio_port *port = &mvf_gpio_ports[gpio / 32];

	if (enable) {
		enable_irq_wake(port->irq);
	} else {
		disable_irq_wake(port->irq);
	}

	return 0;
}

static struct irq_chip gpio_irq_chip = {
	.name = "GPIO",
	.irq_ack = gpio_ack_irq,
	.irq_mask = gpio_mask_irq,
	.irq_unmask = gpio_unmask_irq,
	.irq_set_type = gpio_set_irq_type,
	.irq_set_wake = gpio_set_wake_irq,
};

/*
 * dir = 0: input
 *       1: output
 */
static void _set_gpio_direction(struct gpio_chip *chip, unsigned offset,
				int dir)
{
	struct mvf_gpio_port *port =
		container_of(chip, struct mvf_gpio_port, chip);
	u32 l;
	unsigned long flags;
	unsigned long off = IOMUX_OFF(offset);

	spin_lock_irqsave(&port->lock, flags);
	l = __raw_readl(port->pbase + off) & ~0x3;
	l |= 1<<dir;
	__raw_writel(l, port->pbase + off);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void mvf_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mvf_gpio_port *port =
		container_of(chip, struct mvf_gpio_port, chip);

	void __iomem *reg = (void __iomem *)((u32)(port->gbase) + GPIO_PDOR);
	u32 l;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	l = (__raw_readl(reg) & (~(1 << offset))) | (!!value << offset);
	__raw_writel(l, reg);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int mvf_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct mvf_gpio_port *port =
		container_of(chip, struct mvf_gpio_port, chip);

	return (__raw_readl((u32)(port->gbase) + GPIO_PSOR) >> offset) & 1;
}

static int mvf_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	_set_gpio_direction(chip, offset, 0);
	return 0;
}

static int mvf_gpio_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	mvf_gpio_set(chip, offset, value);
	_set_gpio_direction(chip, offset, 1);
	return 0;
}

/*
 * This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

int mvf_gpio_init(struct mvf_gpio_port *port, int cnt)
{
	int i, j;
	static bool initialed;

	/* save for local usage */
	mvf_gpio_ports = port;
	gpio_table_size = cnt;

	printk(KERN_INFO "MVF GPIO hardware\n");

	for (i = 0; i < cnt; i++) {
		/* disable the interrupt and clear the status */
		__raw_writel(~0, port[i].pbase + PORT_ISFR);
		for (j = port[i].virtual_irq_start;
			j < port[i].virtual_irq_start + 32; j++) {
			irq_set_lockdep_class(j, &gpio_lock_class);
			irq_set_chip_and_handler(j, &gpio_irq_chip,
						 handle_level_irq);
			set_irq_flags(j, IRQF_VALID);
		}

		/* register gpio chip */
		port[i].chip.direction_input = mvf_gpio_direction_input;
		port[i].chip.direction_output = mvf_gpio_direction_output;
		port[i].chip.get = mvf_gpio_get;
		port[i].chip.set = mvf_gpio_set;
		port[i].chip.base = i * 32;
		port[i].chip.ngpio = 32;

		spin_lock_init(&port[i].lock);

		if (!initialed)
			/* its a serious configuration bug when it fails */
			BUG_ON(gpiochip_add(&port[i].chip) < 0);

		/* setup one handler for each entry */
		irq_set_chained_handler(port[i].irq,
								mvf_gpio_irq_chain_handler);
		irq_set_handler_data(port[i].irq, &port[i]);
	}
	initialed = true;

	return 0;
}
