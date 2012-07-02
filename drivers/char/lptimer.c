/*
 * drivers/char/lptimer.c
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
#include "lptimer_reg.h"
#include "lptimer.h"

//	define for timer master
#define TIMER_MASTER_MAX_TIMER			1
#define TIMER_MASTER_ENUM_AVAILABLE		1
#define TIMER_MASTER_ENUM_MAXCHANNEL	1
#include "mvf_timer_master.c"

////////////////////////////////////////////////////////////////

#define LPT_DRIVER_NAME	"lpt"

struct mvf_lpt_dev {
	void __iomem *membase;
	int irq;
	unsigned long configured;
	void (*event_handler)(void);
	struct clk *clk;
};


static irqreturn_t lpt_int_handler(int irq, void *dev_id)
{
	struct mvf_lpt_dev *timedevptr = dev_id;
	unsigned long val;
	irqreturn_t	iret;
	
	iret = IRQ_HANDLED;

	val = readl( timedevptr->membase + LPTMR_CSR_OFFSET);
	if (val & LPTMR_CSR_TCF){

		if ( timedevptr->event_handler){
			(*timedevptr->event_handler)();
		}
		//	clear 1 write.
		writel( val, timedevptr->membase + LPTMR_CSR_OFFSET);
	}else{
		iret = IRQ_NONE;
	}

	return iret;
	
}

int lpt_enable_timer( int timer_handle)
{
	struct platform_device *pdev;
	struct mvf_lpt_dev *timedevptr;
	void __iomem *membase;
	unsigned long val;

	if ( !timer_master_is_opened(timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	if ( !timedevptr->configured){
		return -EAGAIN;
	}
	
	membase = timedevptr->membase;

	//	When TEN is set,the LPTMR is enabled. 
	//	While writing 1 to this field, CSR[5:1] 
	//	must not be altered.
	val = readl(membase + LPTMR_CSR_OFFSET);
	val |= ( LPTMR_CSR_TEN|LPTMR_CSR_TIE);
	writel( val, membase + LPTMR_CSR_TEN);

 	return 0;
}
EXPORT_SYMBOL(lpt_enable_timer);

int lpt_disable_timer( int timer_handle)
{
	struct platform_device *pdev;
	struct mvf_lpt_dev *timedevptr;
	void __iomem *membase;
	unsigned long val;

	if ( !timer_master_is_opened(timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	if ( !timedevptr->configured){
		return -EAGAIN;
	}

	membase = timedevptr->membase;

	//	When TEN is clear, it resets the LPTMR internal logic, 
	//	including the CNR and TCF.
	val = readl(membase + LPTMR_CSR_OFFSET);
	val &= ~LPTMR_CSR_TEN;
	writel( val, membase + LPTMR_CSR_TEN);

	return  0;
}
EXPORT_SYMBOL(lpt_disable_timer);

int lpt_alloc_timer( void)
{
	return timer_master_alloc_timer(0);
}
EXPORT_SYMBOL(lpt_alloc_timer);

int lpt_free_timer( int timer_handle)
{
	if ( !lpt_disable_timer( timer_handle)){
		return timer_master_free( timer_handle);
	}

	return -EAGAIN;
}
EXPORT_SYMBOL( lpt_free_timer);

int lpt_read_counter( int timer_handle, unsigned long *counter)
{
	struct platform_device *pdev;
	struct mvf_lpt_dev *timedevptr;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}
	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	//	16bit timer
	*counter = readl( timedevptr->membase + LPTMR_CNR_OFFSET) & 0x0000ffff;

	return 0;
}
EXPORT_SYMBOL( lpt_read_counter);

int lpt_param_set( int timer_handle, struct mvf_lpt_request *req, void (*event_handler)(void))
{
	void __iomem *membase;
	struct platform_device *pdev;
	struct mvf_lpt_dev *timedevptr;
	unsigned long val;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	membase = timedevptr->membase;
	if ( req == NULL){
        printk( KERN_ERR"lptmr param error \n");
        return -EINVAL;
	}
	
	if( req->timer_mode > LPT_PARAM_TM_PULSECOUNTER){
        printk( KERN_ERR"lptmr clock mode setting error\n");
        return -EINVAL;
	}

	if( req->pulse_pin_polarity > LPT_PARAM_PPP_ACTIVELOW){
        printk( KERN_ERR"lptmr clock pin polarity error\n");
        return -EINVAL;
	}

	if( req->pulse_pin_select > LPT_PARAM_PPS_INPUT3){
        printk( KERN_ERR"lptmr clock pin polarity error\n");
        return -EINVAL;
	}
	
	if( req->prs_clock_sel > LPT_PARAM_PCS_CLOCK3){
        printk( KERN_ERR"lptmr clock select error\n");
        return -EINVAL;
	}

	if( req->prs_bypass > LPT_PARAM_PB_GF_BYPASS){
        printk( KERN_ERR"lptmr clock bypass error\n");
        return -EINVAL;
	}
	

	if( req->prs_value > LPT_PARAM_PV_DIV65536_RISE32768){
        printk( KERN_ERR"lptmr clock bypass error\n");
        return -EINVAL;
	}

	if( req->compare_value > 0x0000ffff){
        printk( KERN_ERR"lptmr clock compare value over 16bit\n");
	}
	

	//	clear all param
	writel( 0, membase + LPTMR_CSR_OFFSET);

	//	CSR	:		TMS(1)						TPP(3)								TPS(4-5)
	val = ( req->timer_mode << 1) | ( req->pulse_pin_polarity << 3) |  ( req->pulse_pin_select << 4);
	writel( val, membase + LPTMR_CSR_OFFSET);

	//	PSR	:		PCS(0)						PBYP(2)				PRESCALE(3-6)
	val = ( req->prs_clock_sel) | ( req->prs_bypass << 2) |  ( req->prs_value << 3);
	writel( val, membase + LPTMR_PSR_OFFSET);

	//	compare
	writel( req->compare_value, membase + LPTMR_CMR_OFFSET);

	timedevptr->event_handler = event_handler;
	timedevptr->configured++;
	//	check 32bit cyclic
	if( timedevptr->configured ==0){
		timedevptr->configured++;
	}
	

	return 0;
}
EXPORT_SYMBOL( lpt_param_set);

static __devinit
int lpt_probe(struct platform_device *pdev)
{
	int size;
	int result;
	struct resource *lptmr_membase, *lptmr_irq;
	struct mvf_lpt_dev *timedevptr;

	lptmr_membase = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	lptmr_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!lptmr_irq || !lptmr_membase){
		 printk(KERN_WARNING"lptmr resource allocation failed\n");
		return -ENODEV;
	}

	timedevptr = kzalloc(sizeof(struct mvf_lpt_dev), GFP_KERNEL);
	if (!timedevptr) {
		 printk(KERN_WARNING"lptmr malloc fail.\n");
		return  -ENOMEM;
	}
	timedevptr->clk = clk_get(&pdev->dev, "lptmr_clk");
	if ( IS_ERR( timedevptr->clk )) {
		dev_err(&pdev->dev, "Could not low power timer clk\n");
		return PTR_ERR( timedevptr->clk);
	}
	clk_enable( timedevptr->clk);

	size = lptmr_membase->end - lptmr_membase->start + 1;
	timedevptr->membase = ioremap(lptmr_membase->start, size);
	if (!timedevptr->membase){
		printk(KERN_WARNING"lptmr ioremap failed.\n");
		return  -ENOMEM;
	}
	timedevptr->irq = lptmr_irq->start;
	
	result = request_irq(timedevptr->irq, lpt_int_handler, 0, LPT_DRIVER_NAME, timedevptr);
	if (result < 0) {
		 printk(KERN_WARNING"lptmr Error %d can't get irq.\n", result);
		return result;
	}

	platform_set_drvdata(pdev, timedevptr);

	timer_master_register_platform(pdev);

	return 0;
}

static int __devexit lpt_remove(struct platform_device *pdev)
{
	struct mvf_lpt_dev *timedevptr;

	timedevptr = platform_get_drvdata(pdev);

	//	disable all
	lpt_disable_timer( 0);
	clk_disable( timedevptr->clk);
	kfree( timedevptr);
	return 0;
}

static struct platform_driver lpt_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mvf-lpt",
		   },
	.probe = lpt_probe,
	.remove = lpt_remove,
};

static int __init lpt_plat_dev_init(void)
{
	return platform_driver_register(&lpt_driver);
}

static void __exit lpt_plat_dev_cleanup(void)
{
	platform_driver_unregister(&lpt_driver);
}

module_init(lpt_plat_dev_init);
module_exit(lpt_plat_dev_cleanup);
