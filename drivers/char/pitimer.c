/*
 * drivers/char/pitimer.c
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <mach/clock.h>
#include "pitimer_reg.h"
#include "pitimer.h"

//	define for timer master
#define TIMER_MASTER_MAX_TIMER			8
#define TIMER_MASTER_ENUM_AVAILABLE		PIT_AVAILABLE_CHANNEL
#define TIMER_MASTER_ENUM_MAXCHANNEL	PIT7
#include "mvf_timer_master.c"


//	define for pit
#define PIT_MAXCHANNEL			8

////////////////////////////////////////////////////////////////

#define PIT_DRIVER_NAME	"pit"

struct mvf_pit_dev {
	void __iomem *membase;
	int irq;
	unsigned long configured[ PIT_MAXCHANNEL];
	unsigned long refer_ch[ PIT_MAXCHANNEL];
	void (*event_handler[ PIT_MAXCHANNEL])(int);
	struct clk *clk;
};


static irqreturn_t pit_int_handler(int irq, void *dev_id)
{
	struct mvf_pit_dev *timedevptr = dev_id;
	unsigned long val;
	irqreturn_t	iret;
	int i;

	iret = IRQ_NONE;

	for ( i = 0; i < PIT_MAXCHANNEL; i ++){
		if ( timedevptr->configured[ i]){
			val = readl( timedevptr->membase + PIT_TFLG_OFFSET( i));
			if ( val & PIT_TFLG_TIF){
				(*timedevptr->event_handler[i])( timedevptr->refer_ch[ i]);
			}
			//	ack
			writel( val, timedevptr->membase + PIT_TFLG_OFFSET( i));
			iret = IRQ_HANDLED;
		}
	}

	return iret;
	
}

int pit_enable_timer( int timer_handle)
{
	struct platform_device *pdev;
	struct mvf_pit_dev *timedevptr;
	void __iomem *membase;
	unsigned long val;
	int i;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}
	
	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);
	i = timer_handle % PIT_MAXCHANNEL;

	if ( !timedevptr->configured[ i]){
		return -EAGAIN;
	}

	membase = timedevptr->membase;

	writel( PIT_TFLG_TIF, timedevptr->membase + PIT_TFLG_OFFSET( i));
	val = PIT_TCTR_TEN | PIT_TCTR_TIE;
	writel( val, membase + PIT_TCTRL_OFFSET( i));

 	return 0;
}
EXPORT_SYMBOL(pit_enable_timer);

int pit_disable_timer( int timer_handle)
{
	int i;
	struct platform_device *pdev;
	struct mvf_pit_dev *timedevptr;
	void __iomem *membase;
	

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev( timer_handle);
	timedevptr = platform_get_drvdata( pdev);

	i = timer_handle % PIT_MAXCHANNEL;
	if ( !timedevptr->configured[ i]){
		return -EAGAIN;
	}

	membase = timedevptr->membase;

	writel( 0, membase + PIT_TCTRL_OFFSET( i));

	return  0;
}
EXPORT_SYMBOL(pit_disable_timer);

int pit_alloc_timer( pit_channel ch)
{
	return timer_master_alloc_timer( ch);
}
EXPORT_SYMBOL(pit_alloc_timer);

int pit_free_timer( int timer_handle)
{
	if ( !pit_disable_timer( timer_handle)){
		return timer_master_free( timer_handle);
	}

	return -EAGAIN;
}
EXPORT_SYMBOL( pit_free_timer);

int pit_read_counter( int timer_handle, unsigned long *counter)
{
	struct platform_device *pdev;
	struct mvf_pit_dev *timedevptr;
	int i;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}
	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	i = timer_handle % PIT_MAXCHANNEL;

	//	33bit timer
	*counter = readl( timedevptr->membase + PIT_CVAL_OFFSET(i));

	return 0;
}
EXPORT_SYMBOL( pit_read_counter);

int pit_param_set( int timer_handle, unsigned long load_val, void (*event_handler)(int))
{
	int i;
	void __iomem *membase;
	struct platform_device *pdev;
	struct mvf_pit_dev *timedevptr;
	

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	membase = timedevptr->membase;
	i = timer_handle % PIT_MAXCHANNEL;

	writel( load_val, membase + PIT_LDVAL_OFFSET(i));

	timedevptr->refer_ch[ i] = timer_handle;
	timedevptr->event_handler[ i] = event_handler;

	timedevptr->configured[ i]++;
	//	check 32bit cyclic
	if( timedevptr->configured[ i] == 0){
		timedevptr->configured[ i]++;
	}
	

	return 0;
}
EXPORT_SYMBOL( pit_param_set);

static __devinit
int pit_probe(struct platform_device *pdev)
{
	int size;
	int result;
	int i;
	unsigned long val;
	struct resource *pit_membase, *pit_irq;
	struct mvf_pit_dev *timedevptr;

	pit_membase = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pit_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!pit_irq || !pit_membase){
		 printk(KERN_WARNING"PIT resource allocation failed\n");
		return -ENODEV;
	}

	timedevptr = kzalloc(sizeof(struct mvf_pit_dev), GFP_KERNEL);
	if (!timedevptr) {
		 printk(KERN_WARNING"PIT malloc fail.\n");
		return  -ENOMEM;
	}
	timedevptr->clk = clk_get(&pdev->dev, "pit_clk");
	if ( IS_ERR( timedevptr->clk )) {
		dev_err(&pdev->dev, "Could not pit clk\n");
		return PTR_ERR( timedevptr->clk);
	}
	clk_enable( timedevptr->clk);

	size = pit_membase->end - pit_membase->start + 1;
	timedevptr->membase = ioremap(pit_membase->start, size);
	if (!timedevptr->membase){
		printk(KERN_WARNING"PIT ioremap failed.\n");
		return  -ENOMEM;
	}
	timedevptr->irq = pit_irq->start;
	
	result = request_irq(timedevptr->irq, pit_int_handler, 0, PIT_DRIVER_NAME, timedevptr);
	if (result < 0) {
		printk(KERN_WARNING"PIT Error %d can't get irq.\n", result);
		return result;
	}

	platform_set_drvdata(pdev, timedevptr);

	//	for each channel
	for ( i = 0; i < TIMER_MASTER_MAX_TIMER; i ++){
		timer_master_register_platform( pdev);
	}

	val = readl( timedevptr->membase + PIT_MCR_OFFSET);
	if( (val & PIT_MCR_MDIS)){
		//	timer clock start
		writel( 0, timedevptr->membase + PIT_MCR_OFFSET);
	}else{
		//	assume PIT 0 is kernel system tick
		pit_alloc_timer( PIT0);
		printk(KERN_WARNING"Maybe PIT0 is system tick\n");
	}

	return 0;
}

static int __devexit pit_remove(struct platform_device *pdev)
{
	struct mvf_pit_dev *timedevptr;

	timedevptr = platform_get_drvdata(pdev);

	//	disable all
	pit_disable_timer( 0);
	clk_disable( timedevptr->clk);
	kfree( timedevptr);

	return 0;
}

static struct platform_driver pit_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mvf-pit",
		   },
	.probe = pit_probe,
	.remove = pit_remove,
};

static int __init pit_plat_dev_init(void)
{
	return platform_driver_register(&pit_driver);
}

static void __exit pit_plat_dev_cleanup(void)
{
	platform_driver_unregister(&pit_driver);
}

module_init(pit_plat_dev_init);
module_exit(pit_plat_dev_cleanup);
