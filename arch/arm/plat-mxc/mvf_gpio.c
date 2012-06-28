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


//#define GPIO_DEBUG
#undef GPIO_DEBUG
#ifdef GPIO_DEBUG
#define GPRT(fmt, args...) printk("DBG:%s[%d]" fmt,__func__,__LINE__,## args)
#else
#define GPRT(fmt...)
#endif

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
#define PT0  0
#define PT1  1
#define PT2  2
#define PT3  3
#define PT4  4

#define NOCFG	0
#if 0
static unsigned char gpio_to_mux[] = {
	/* PT_A */
	GTM(PT0, 0,NOCFG),GTM(PT0, 1,NOCFG),GTM(PT0, 2,NOCFG),GTM(PT0, 3,NOCFG),
	GTM(PT0, 4,NOCFG),GTM(PT0, 5,NOCFG),GTM(PT0, 6,0x000),GTM(PT0, 7,0x218),
	GTM(PT0, 8,0x010),GTM(PT0, 9,0x008),GTM(PT0,10,0x00c),GTM(PT0,11,0x010),
    GTM(PT0,12,0x014),GTM(PT0,13,NOCFG),GTM(PT0,14,NOCFG),GTM(PT0,15,NOCFG),
    GTM(PT0,16,0x018),GTM(PT0,17,0x01c),GTM(PT0,18,0x020),GTM(PT0,19,0x024),
    GTM(PT0,20,0x028),GTM(PT0,21,0x02c),GTM(PT0,22,0x030),GTM(PT0,23,0x034),
    GTM(PT0,24,0x038),GTM(PT0,25,0x03c),GTM(PT0,26,0x040),GTM(PT0,27,0x044),
    GTM(PT0,28,0x048),GTM(PT0,29,0x04c),GTM(PT0,30,0x050),GTM(PT0,31,0x054),
	/* PT_B */
	GTM(PT1, 0,0x058),GTM(PT1, 1,0x05c),GTM(PT1, 2,0x060),GTM(PT1, 3,0x064),
	GTM(PT1, 4,0x068),GTM(PT1, 5,0x06c),GTM(PT1, 6,0x070),GTM(PT1, 7,0x074),
	GTM(PT1, 8,0x078),GTM(PT1, 9,0x07c),GTM(PT1,10,0x080),GTM(PT1,11,0x084),
    GTM(PT1,12,0x088),GTM(PT1,13,0x08c),GTM(PT1,14,0x090),GTM(PT1,15,0x094),
    GTM(PT1,16,0x098),GTM(PT1,17,0x09c),GTM(PT1,18,0x0a0),GTM(PT1,19,0x0a4),
    GTM(PT1,20,0x0a8),GTM(PT1,21,0x0ac),GTM(PT1,22,0x0b0),GTM(PT1,23,NOCFG),
    GTM(PT1,24,NOCFG),GTM(PT1,25,0x17c),GTM(PT1,26,0x180),GTM(PT1,27,0x184),
    GTM(PT1,28,0x188),GTM(PT1,29,NOCFG),GTM(PT1,30,NOCFG),GTM(PT1,31,NOCFG),
	/* PT_C */
	GTM(PT2, 0,0x0b4),GTM(PT2, 1,0x0b8),GTM(PT2, 2,0x0bc),GTM(PT2, 3,0x0c0),
	GTM(PT2, 4,0x0c4),GTM(PT2, 5,0x0c8),GTM(PT2, 6,0x0cc),GTM(PT2, 7,0x0d0),
	GTM(PT2, 8,0x0d4),GTM(PT2, 9,0x0d8),GTM(PT2,10,0x0dc),GTM(PT2,11,0x0e0),
    GTM(PT2,12,0x0e4),GTM(PT2,13,0x0e8),GTM(PT2,14,0x0ec),GTM(PT2,15,0x0f0),
    GTM(PT2,16,0x0f4),GTM(PT2,17,0x0f8),GTM(PT2,18,NOCFG),GTM(PT2,19,NOCFG),
    GTM(PT2,20,NOCFG),GTM(PT2,21,NOCFG),GTM(PT2,22,NOCFG),GTM(PT2,23,NOCFG),
    GTM(PT2,24,NOCFG),GTM(PT2,25,NOCFG),GTM(PT2,26,0x18c),GTM(PT2,27,0x190),
    GTM(PT2,28,0x194),GTM(PT2,29,0x198),GTM(PT2,30,0x19c),GTM(PT2,31,0x1a0),
	/* PT_D */
	GTM(PT3, 0,0x13c),GTM(PT3, 1,0x140),GTM(PT3, 2,0x144),GTM(PT3, 3,0x148),
	GTM(PT3, 4,0x14c),GTM(PT3, 5,0x150),GTM(PT3, 6,0x15c),GTM(PT3, 7,0x158),
	GTM(PT3, 8,0x15c),GTM(PT3, 9,0x160),GTM(PT3,10,0x164),GTM(PT3,11,0x168),
    GTM(PT3,12,0x16c),GTM(PT3,13,0x170),GTM(PT3,14,0x174),GTM(PT3,15,0x17c),
    GTM(PT3,16,0x138),GTM(PT3,17,0x134),GTM(PT3,18,0x130),GTM(PT3,19,0x12c),
    GTM(PT3,20,0x128),GTM(PT3,21,0x124),GTM(PT3,22,0x120),GTM(PT3,23,0x11c),
    GTM(PT3,24,0x118),GTM(PT3,25,0x114),GTM(PT3,26,0x110),GTM(PT3,27,0x10c),
    GTM(PT3,28,0x108),GTM(PT3,29,0x104),GTM(PT3,30,0x100),GTM(PT3,31,0x0fc),
	/* PT_E */
	GTM(PT4, 0,0x1a4),GTM(PT4, 1,0x1a8),GTM(PT4, 2,0x1ac),GTM(PT4, 3,0x1b0),
	GTM(PT4, 4,0x1b4),GTM(PT4, 5,0x1b8),GTM(PT4, 6,0x1bc),GTM(PT4, 7,0x1c0),
	GTM(PT4, 8,0x1c4),GTM(PT4, 9,0x1c8),GTM(PT4,10,0x1cc),GTM(PT4,11,0x1d0),
    GTM(PT4,12,0x1d4),GTM(PT4,13,0x1d8),GTM(PT4,14,0x1dc),GTM(PT4,15,0x1e0),
    GTM(PT4,16,0x1e0),GTM(PT4,17,0x1e8),GTM(PT4,18,0x1ec),GTM(PT4,19,0x1f4),
    GTM(PT4,20,0x1f4),GTM(PT4,21,0x1f8),GTM(PT4,22,0x1fc),GTM(PT4,23,0x200),
    GTM(PT4,24,0x204),GTM(PT4,25,0x208),GTM(PT4,26,0x20c),GTM(PT4,27,0x210),
    GTM(PT4,28,0x214),GTM(PT4,29,NOCFG),GTM(PT4,30,NOCFG),GTM(PT4,31,NOCFG),
};
#define IOMUX_OFF(x) (gpio_to_mux[x]<<2)
#else
#define IOMUX_OFF(x) ((x)<<2)
#endif

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


/* PTx PT0 to PT4 */
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

	GPRT("irq_stat = %d\n",irq_stat);
	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;
		generic_handle_irq(gpio_irq_no_base + irqoffset);
		GPRT("virq = %d\n",gpio_irq_no_base + irqoffset);

		irq_stat &= ~(1 << irqoffset);
	}
}

static void mvf_gpio_irq_chain_handler(u32 irq, struct irq_desc *desc)
{
	u32 irq_stat;
	struct mvf_gpio_port *port = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);

	GPRT("irq = %d\n",irq);
	chained_irq_enter(chip, desc);

	irq_stat = __raw_readl((void __iomem *)((u32)(port->pbase) + PORT_ISFR));

	mvf_gpio_irq_handler(port, irq_stat);

	__raw_writel(irq_stat,(void __iomem *)((u32)(port->pbase) + PORT_ISFR));

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
	unsigned long off = IOMUX_OFF(offset + port->pad);

	spin_lock_irqsave(&port->lock, flags);
	l = __raw_readl(port->ibase + off) & ~0x3;
	GPRT("offset = %d,l = %x,dir = %d,off = %x(%d)\n",offset,l,dir,off,off>>2);
	l |= 1<<dir;
	__raw_writel(l, port->ibase + off);
	spin_unlock_irqrestore(&port->lock, flags);
	GPRT("result : addr = %08x = %08x\n",port->ibase + off,__raw_readl(port->ibase + off)); 
}

static void mvf_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mvf_gpio_port *port =
		container_of(chip, struct mvf_gpio_port, chip);

	void __iomem *reg;
	u32 l;
	unsigned long flags;

	GPRT("offset = %d,value = %d,reg = %08x\n",offset,value,reg);
	spin_lock_irqsave(&port->lock, flags);
#if 0
	reg = (void __iomem *)((u32)(port->gbase) + GPIO_PSOR);
	l = (__raw_readl(reg) & (~(1 << offset))) | (!!value << offset);
#else
	if ( value )
		reg = (void __iomem *)((u32)(port->gbase) + GPIO_PSOR);
	else
		reg = (void __iomem *)((u32)(port->gbase) + GPIO_PCOR);
	l = 1<<offset;
#endif
	GPRT("offset = %d,val = %08x\n",offset,l);
	__raw_writel(l, reg);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int mvf_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	unsigned long get_val;
	struct mvf_gpio_port *port =
		container_of(chip, struct mvf_gpio_port, chip);
	get_val = __raw_readl((u32)(port->gbase) + GPIO_PDIR);
	ret = (get_val >> offset & 1);
	GPRT("offset = %d,addr = %08x, value = %08x,val = %08x\n",offset,(u32)(port->gbase) + GPIO_PDOR,get_val,ret);
	return ret;
}

static int mvf_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	GPRT("offset = %d\n",offset);
	_set_gpio_direction(chip, offset, 0);
	return 0;
}

static int mvf_gpio_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	GPRT("value = %d\n",value);
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

	GPRT("init\n");
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
