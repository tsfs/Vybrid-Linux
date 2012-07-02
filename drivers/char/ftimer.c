/*
 * drivers/char/ftimer.c
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
#include "ftimer_reg.h"
#include "ftimer.h"

//	define for timer master
#define TIMER_MASTER_MAX_TIMER			2
#define TIMER_MASTER_ENUM_AVAILABLE		FMT_AVAILABLE_CHANNEL
#define TIMER_MASTER_ENUM_MAXCHANNEL	FMT1
#include "mvf_timer_master.c"


////////////////////////////////////////////////////////////////

#define FTM_DRIVER_NAME	"ftm"

struct mvf_ftm_dev {
	struct clk *clk;
	void __iomem *membase;
	int irq;
	unsigned long configured;
	unsigned long refer_ch;
	void (*event_handler)(int);
};

void ftm_module_enable( void __iomem *membase)
{
	unsigned long val;

	val = readl(membase + FTM_MODE_OFFSET);
	if (!( val & FTM_MODE_FTMEN)){
		val |= ( FTM_MODE_FTMEN | FTM_MODE_INIT);
		writel(val, membase + FTM_MODE_OFFSET);
	}
}

void ftm_module_disable( void __iomem *membase)
{
	unsigned long val;

	val = readl(membase + FTM_MODE_OFFSET);
	val &= ~FTM_MODE_FTMEN;
	writel(val, membase + FTM_MODE_OFFSET);
}


void ftm_write_protect_on( void __iomem *membase)
{
	unsigned long val;
	
	val = readl(membase + FTM_FMS_OFFSET);
	if ( !(val & FTM_FMS_WPEN)){
		val |= FTM_FMS_WPEN;
		writel(val, membase + FTM_FMS_OFFSET);
	}
}

void ftm_write_protect_off( void __iomem *membase)
{
	unsigned long val,val2;

	val = readl(membase + FTM_MODE_OFFSET);
	val2 = readl(membase + FTM_FMS_OFFSET);
	if ( val2 & FTM_FMS_WPEN){
		val |= FTM_MODE_WPDIS;
		writel(val, membase + FTM_MODE_OFFSET);
	}

}

static int in_ftm_timer_stop( struct mvf_ftm_dev *timedevptr)
{
	void __iomem *membase;
	unsigned long val;

	membase = timedevptr->membase;

	val = readl(membase + FTM_SC_OFFSET);
	val &= ~FTM_SC_TOIE;

	//	interrupt disable
	writel( val, membase + FTM_SC_OFFSET);

	return 0;
}

static int in_ftm_timer_start( struct mvf_ftm_dev *timedevptr)
{
	void __iomem *membase;
	unsigned long val;
	int ret;

	membase = timedevptr->membase;

	ret = 0;

	if ( timedevptr->configured){
		//	read sc reg
		val = readl(membase + FTM_SC_OFFSET);
		val |= FTM_SC_TOIE;

		//	clear counter
		writel(0, membase + FTM_CNT_OFFSET);

		//	interrupt enable
		writel( val, membase + FTM_SC_OFFSET);
	}else{
		ret = -EAGAIN;
	}

	return ret;
}

static irqreturn_t ftm_int_handler(int irq, void *dev_id)
{
	struct mvf_ftm_dev *timedevptr = dev_id;
	unsigned long val;
	irqreturn_t	iret;
	
	iret = IRQ_HANDLED;

	val = readl( timedevptr->membase + FTM_SC_OFFSET);
	if (val & FTM_SC_TOF){

		if ( timedevptr->event_handler){
			(*timedevptr->event_handler)( timedevptr->refer_ch);
		}
		val &= ~FTM_SC_TOF;
		writel( val, timedevptr->membase + FTM_SC_OFFSET);
	}else{
		iret = IRQ_NONE;
	}

	return iret;
	
}

int ftm_enable_timer( int timer_handle)
{
	struct platform_device *pdev;
	struct mvf_ftm_dev *timedevptr;

	if ( !timer_master_is_opened(timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

 	return in_ftm_timer_start( timedevptr);
}
EXPORT_SYMBOL( ftm_enable_timer);

int ftm_disable_timer( int timer_handle)
{
	struct platform_device *pdev;
	struct mvf_ftm_dev *timedevptr;

	if ( !timer_master_is_opened(timer_handle)){
		return -EAGAIN;
	}
	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);


	return  in_ftm_timer_stop( timedevptr);
}
EXPORT_SYMBOL(ftm_disable_timer);


int ftm_alloc_timer( ftm_channel ch)
{
	return timer_master_alloc_timer(ch);
}
EXPORT_SYMBOL(ftm_alloc_timer);

int ftm_free_timer (int timer_handle)
{
	if ( !ftm_disable_timer( timer_handle)){
		return timer_master_free( timer_handle);
	}

	return -EAGAIN;
}
EXPORT_SYMBOL(ftm_free_timer);

int ftm_read_counter( int timer_handle, unsigned long *counter)
{
	struct platform_device *pdev;
	struct mvf_ftm_dev *timedevptr;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}
	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	//	16bit timer
	*counter = readl( timedevptr->membase + FTM_CNT_OFFSET) & 0x0000ffff;

	return 0;
}
EXPORT_SYMBOL(ftm_read_counter);

int ftm_param_set( int timer_handle, struct mvf_ftm_request *req, void (*event_handler)(int))
{
	void __iomem *membase;
	struct platform_device *pdev;
	struct mvf_ftm_dev *timedevptr;
	unsigned long val;

	if ( !timer_master_is_opened( timer_handle)){
		return -EAGAIN;
	}

	pdev = timer_master_get_pdev(timer_handle);
	timedevptr = platform_get_drvdata(pdev);

	membase = timedevptr->membase;
	if ( req == NULL){
        printk( KERN_ERR"FTM param error \n");
        return -EINVAL;
	}
	
	if( req->clocksource > FTM_PARAM_CLK_EXTERNAL){
        printk( KERN_ERR"FTM clock source setting error\n");
        return -EINVAL;
	}

	if( req->divider > FTM_PARAM_DIV_BY_128){
        printk( KERN_ERR"FTM clock divider error\n");
        return -EINVAL;
	}
	
	//	WP off
	ftm_write_protect_off( membase);
	
	//	enable ip
	ftm_module_enable( membase);
	
	//	set clksrc and divider. IE is OFF.
	val = (req->clocksource << 3) | req->divider;
	writel(val, membase + FTM_SC_OFFSET);

	//	counter initial value(=reset value)
	writel(req->start, membase + FTM_CNTIN_OFFSET);
	//	modulo value(=max count)
	writel(req->end, membase + FTM_MOD_OFFSET);

	timedevptr->event_handler = event_handler;
	timedevptr->refer_ch = timer_handle;

	timedevptr->configured++;
	//	check 32bit cyclic
	if( timedevptr->configured == 0){
		timedevptr->configured++;
	}
	
	ftm_write_protect_on( membase);

	return 0;
}
EXPORT_SYMBOL(ftm_param_set);

static __devinit
int ftm_probe(struct platform_device *pdev)
{
	int size;
	int result;
	struct resource *ftm_membase, *ftm_irq;
	struct mvf_ftm_dev *timedevptr;

	ftm_membase = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ftm_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!ftm_irq || !ftm_membase){
		 printk(KERN_WARNING"FTM resource allocation failed\n");
		return -ENODEV;
	}

	timedevptr = kzalloc(sizeof(struct mvf_ftm_dev), GFP_KERNEL);
	if (!timedevptr) {
		 printk(KERN_WARNING"FTM malloc fail.\n");
		return  -ENOMEM;
	}

	if ( pdev->id == 0){
		timedevptr->clk = clk_get(&pdev->dev, "ftm0_clk");
	}else{
		timedevptr->clk = clk_get(&pdev->dev, "ftm1_clk");
	}
	if ( IS_ERR( timedevptr->clk )) {
		dev_err(&pdev->dev, "Could not get FTM %d clock \n",pdev->id );
		return PTR_ERR( timedevptr->clk);
	}
	clk_enable( timedevptr->clk);

	size = ftm_membase->end - ftm_membase->start + 1;
	timedevptr->membase = ioremap(ftm_membase->start, size);
	if (!timedevptr->membase){
		printk(KERN_WARNING"FTM ioremap failed.\n");
		return  -ENOMEM;
	}
	timedevptr->irq = ftm_irq->start;

	result = request_irq(timedevptr->irq, ftm_int_handler, 0, FTM_DRIVER_NAME, timedevptr);
	if (result < 0) {
		 printk(KERN_WARNING"FTM Error %d can't get irq.\n", result);
		return result;
	}

	platform_set_drvdata(pdev, timedevptr);

	timer_master_register_platform(pdev);

	return 0;
}

static int __devexit ftm_remove(struct platform_device *pdev)
{
	struct mvf_ftm_dev *timedevptr;

	timedevptr = platform_get_drvdata(pdev);

	//	disable all
	writel( 0, timedevptr->membase + FTM_SC_OFFSET);
	in_ftm_timer_stop( timedevptr);
	clk_disable( timedevptr->clk);
	kfree( timedevptr);

	return 0;
}

static struct platform_driver ftm_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mvf-ftm",
		   },
	.probe = ftm_probe,
	.remove = ftm_remove,
};

static int __init ftm_plat_dev_init(void)
{
	return platform_driver_register(&ftm_driver);
}

static void __exit ftm_plat_dev_cleanup(void)
{
	platform_driver_unregister(&ftm_driver);
}

module_init(ftm_plat_dev_init);
module_exit(ftm_plat_dev_cleanup);
