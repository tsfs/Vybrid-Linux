/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * eDMA driver
 *
 * based on drivers/dma/imx-sdma.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <mach/common.h>
#include <asm/mvf_edma.h>
#include <asm/mvf_edma_regs.h>

#define MVF_MAX_DMA_ENGINE		1
#define MVF_EDMA_CHANNELS		(MVF_MAX_DMA_ENGINE*MVF_EACH_DMA_CHANNEL)
#define MVF_MAX_XFER_BYTES		2048

struct mvf_dma_chan {
	struct mvf_dma_engine		*mvf_dma;
	struct dma_chan			chan;
	struct dma_async_tx_descriptor	desc;
	struct tasklet_struct		tasklet;
	dma_addr_t			ccw_phys;
	int				desc_count;
	dma_cookie_t			last_completed;
	enum dma_status			status;
	unsigned int			flags;
	void __iomem			*chan_mem_base;
#define MVF_DMA_SG_LOOP			(1 << 0)
};

struct mvf_dma_engine {
	int				dev_id;
	unsigned int			version;
	void __iomem			*base[MVF_MAX_DMA_ENGINE];
	struct clk			*clk;
	struct dma_device		dma_device;
	struct device_dma_parameters	dma_parms;
	struct mvf_dma_chan		mvf_chans[MVF_EDMA_CHANNELS];
	int				dma_irq[MVF_MAX_DMA_ENGINE];
	int				err_irq[MVF_MAX_DMA_ENGINE];
};


static void mvf_dma_reset_chan(struct mvf_dma_chan *mvf_chan)
{
	void __iomem *base = mvf_chan->chan_mem_base;
	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;

	MVF_EDMA_TCD_CSR(base, channel) = 0x0000;

}

static void mvf_dma_enable_chan(struct mvf_dma_chan *mvf_chan)
{
	void __iomem *base = mvf_chan->chan_mem_base;
	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;

	MVF_EDMA_SERQ(base) = channel;
	MVF_EDMA_SSRT(base) = channel;
}

static void mvf_dma_disable_chan(struct mvf_dma_chan *mvf_chan)
{
	void __iomem *base = mvf_chan->chan_mem_base;
	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;

	mvf_chan->status = DMA_SUCCESS;
	MVF_EDMA_CEEI(base) = MVF_EDMA_CEEI_CEEI(channel);

}

static void mvf_dma_pause_chan(struct mvf_dma_chan *mvf_chan)
{
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;
//	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;

	//
	//	pause code
	//

	mvf_chan->status = DMA_PAUSED;
}

static void mvf_dma_resume_chan(struct mvf_dma_chan *mvf_chan)
{
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;
//	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;

	//	resume code
	mvf_chan->status = DMA_IN_PROGRESS;
}

static dma_cookie_t mvf_dma_assign_cookie(struct mvf_dma_chan *mvf_chan)
{
	dma_cookie_t cookie = mvf_chan->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	mvf_chan->chan.cookie = cookie;
	mvf_chan->desc.cookie = cookie;

	return cookie;
}

static struct mvf_dma_chan *to_mvf_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct mvf_dma_chan, chan);
}

static dma_cookie_t mvf_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(tx->chan);
	dma_cookie_t cookie = mvf_dma_assign_cookie(mvf_chan);

	//	tx start
	mvf_dma_enable_chan(mvf_chan);

	return cookie;
}

static void mvf_dma_tasklet(unsigned long data)
{
	struct mvf_dma_chan *mvf_chan = (struct mvf_dma_chan *) data;

	if (mvf_chan->desc.callback)
		mvf_chan->desc.callback(mvf_chan->desc.callback_param);
}

struct mvf_dma_chan *
mvf_find_chan(struct mvf_dma_engine *mvf_dma, int eng_idx, int chan_id)
{
	int i;

	for ( i = 0; i < MVF_EDMA_CHANNELS; i ++){
		if ( mvf_dma->mvf_chans[i].chan.chan_id == (chan_id + (eng_idx*MVF_EACH_DMA_CHANNEL))){
			return &mvf_dma->mvf_chans[i];
		}
	}
	return 0;
}

static irqreturn_t mvf_dma_int_handler(int irq, void *dev_id)
{
	int i,int_src,engine;
	struct mvf_dma_engine *mvf_dma = dev_id;
	struct mvf_dma_chan *mvf_chan;
	void __iomem *base;

	base = 0;

	//	find normal irq index
	for (i = 0; i < MVF_MAX_DMA_ENGINE; i++) {
		if ( mvf_dma->dma_irq[i] == irq){
			base = mvf_dma->base[i];
			engine = i;
		}
	}

	//	fail safe
	if (!base){
		printk("error irq\n");
		return IRQ_HANDLED;
	}

	int_src = MVF_EDMA_INT(base);
	for (i = 0; i < MVF_EACH_DMA_CHANNEL; i++) {
		if ( int_src & (1 << i)){
			// find chan 
			mvf_chan = mvf_find_chan(mvf_dma, engine, i);
			if (mvf_chan){
				mvf_chan->status = DMA_SUCCESS;
				mvf_chan->last_completed = mvf_chan->desc.cookie;
		
				/* schedule tasklet on this channel */
				tasklet_schedule(&mvf_chan->tasklet);
			}
		}
	}
	MVF_EDMA_CINT(base) = MVF_EDMA_CINT_CAIR;

	return IRQ_HANDLED;
}

static irqreturn_t mvf_dma_err_handler(int irq, void *dev_id)
{
	int i,err,engine;
	struct mvf_dma_engine *mvf_dma = dev_id;
	struct mvf_dma_chan *mvf_chan;
	void __iomem *base;

	base = 0;

	for (i = 0; i < MVF_MAX_DMA_ENGINE; i++) {
		if ( mvf_dma->err_irq[i] == irq){
			base = mvf_dma->base[i];
			engine = i;
		}
	}

	//	fail safe
	if (!base){
		printk("error irq\n");
		return IRQ_HANDLED;
	}

	err = MVF_EDMA_ERR(base);
	for (i = 0; i < MVF_EACH_DMA_CHANNEL; i++) {
		if ( err & (1 << i)){
			mvf_chan = mvf_find_chan(mvf_dma, engine, i);
			if (mvf_chan){
				mvf_chan->status = DMA_ERROR;
				tasklet_schedule(&mvf_chan->tasklet);
			}
		}
	}
	MVF_EDMA_CERR(base) = MVF_EDMA_CERR_CAER;

	return IRQ_HANDLED;
}


static int mvf_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;

	mvf_dma_reset_chan(mvf_chan);

	dma_async_tx_descriptor_init(&mvf_chan->desc, chan);
	mvf_chan->desc.tx_submit = mvf_dma_tx_submit;

	/* the descriptor is ready */
	async_tx_ack(&mvf_chan->desc);

	return 0;
}

static void mvf_dma_free_chan_resources(struct dma_chan *chan)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;

	mvf_dma_disable_chan(mvf_chan);

//	free_irq(mvf_chan->chan_irq, mvf_dma);
}


void
mvf_edma_set_tcd_params(struct mvf_dma_chan *mvf_chan, u32 source, u32 dest,
			u32 attr, u32 soff, u32 nbytes, u32 slast,
			u32 citer, u32 biter, u32 doff, u32 dlast_sga,
			int major_int, int disable_req)
{
//	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;

	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;
	void __iomem *base = mvf_chan->chan_mem_base;
	
	MVF_EDMA_TCD_SADDR(base, channel) = source;
	MVF_EDMA_TCD_DADDR(base, channel) = dest;
	MVF_EDMA_TCD_ATTR(base, channel) = attr;
	MVF_EDMA_TCD_SOFF(base, channel) = MVF_EDMA_TCD_SOFF_SOFF(soff);
	MVF_EDMA_TCD_NBYTES(base, channel) = MVF_EDMA_TCD_NBYTES_NBYTES(nbytes);
	MVF_EDMA_TCD_SLAST(base, channel) = MVF_EDMA_TCD_SLAST_SLAST(slast);
	MVF_EDMA_TCD_CITER(base, channel) = MVF_EDMA_TCD_CITER_CITER(citer);
	MVF_EDMA_TCD_BITER(base, channel) = MVF_EDMA_TCD_BITER_BITER(biter);
	MVF_EDMA_TCD_DOFF(base, channel) = MVF_EDMA_TCD_DOFF_DOFF(doff);
	MVF_EDMA_TCD_DLAST_SGA(base, channel) = MVF_EDMA_TCD_DLAST_SGA_DLAST_SGA(dlast_sga);

	/* interrupt at the end of major loop */
	if (major_int)
		MVF_EDMA_TCD_CSR(base,channel) |= MVF_EDMA_TCD_CSR_INT_MAJOR;
	else
		MVF_EDMA_TCD_CSR(base,channel) &= ~MVF_EDMA_TCD_CSR_INT_MAJOR;

	/* disable request at the end of major loop of transfer or not */
	if (disable_req)
		MVF_EDMA_TCD_CSR(base,channel) |= MVF_EDMA_TCD_CSR_D_REQ;
	else
		MVF_EDMA_TCD_CSR(base,channel) &= ~MVF_EDMA_TCD_CSR_D_REQ;

	/* enable error interrupt */
	MVF_EDMA_SEEI(base) = MVF_EDMA_SEEI_SEEI(channel);
}

static struct dma_async_tx_descriptor *mvf_dma_prep_memcpy
(struct dma_chan *chan,
	dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
//	struct mvf_dma_engine *mvf_dma = mvf_chan->mvf_dma;
//	int channel = mvf_chan->chan.chan_id;

	if (mvf_chan->status == DMA_IN_PROGRESS)
		return NULL;

	mvf_chan->status = DMA_IN_PROGRESS;

	//	chan_id
#if 1
	mvf_edma_set_tcd_params(
						mvf_chan,
						src,
						dst,
						(0 | MVF_EDMA_TCD_ATTR_SSIZE_32BIT | MVF_EDMA_TCD_ATTR_DSIZE_32BIT), 
						0x04,
						len, 0x0, 1, 1, 
						0x04, 0x0, 0x1,0x0);

#else
	//	channel control
	if ( channel == 10){
		mvf_edma_set_tcd_params(
						channel,
						src,
						dst,
						(0 | MVF_EDMA_TCD_ATTR_SSIZE_32BIT | MVF_EDMA_TCD_ATTR_DSIZE_32BIT), 
						0x04,
						len, 0x0, 1, 1, 
						0x04, 0x0, 0x1,0x0);
	}else{
	}
#endif


	mvf_chan->desc_count = 0;

	return &mvf_chan->desc;
}


static int mvf_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	int ret = 0;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		mvf_dma_reset_chan(mvf_chan);
		mvf_dma_disable_chan(mvf_chan);
		break;
	case DMA_PAUSE:
		mvf_dma_pause_chan(mvf_chan);
		break;
	case DMA_RESUME:
		mvf_dma_resume_chan(mvf_chan);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static enum dma_status mvf_dma_tx_status(struct dma_chan *chan,
			dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	dma_cookie_t last_used;

	last_used = chan->cookie;
	dma_set_tx_state(txstate, mvf_chan->last_completed, last_used, 0);

	return mvf_chan->status;
}

static void mvf_dma_issue_pending(struct dma_chan *chan)
{
	/*
	 * Nothing to do. We only have a single descriptor.
	 */
}

static int __init mvf_dma_init(struct mvf_dma_engine *mvf_dma)
{
	int i,ret,cnt;
	u32 grp0_pri = MVF_EDMA_CR_GRP0PRI(0x00);
	u32 grp1_pri = MVF_EDMA_CR_GRP1PRI(0x01);

	for (i = 0; i < MVF_MAX_DMA_ENGINE; i++) {
		MVF_EDMA_CR(mvf_dma->base[i]) = (0 | grp0_pri | grp1_pri);
		
		//	clear every tcd
		for (cnt=0; cnt <MVF_EACH_DMA_CHANNEL; cnt ++){
			MVF_EDMA_TCD_CSR(mvf_dma->base[i], cnt) = 0x0000;
		}

		ret = request_irq(mvf_dma->dma_irq[i], mvf_dma_int_handler,0, "mvf_dma", mvf_dma);
		if( ret) return -EBUSY;

		ret = request_irq(mvf_dma->err_irq[i], mvf_dma_err_handler,0, "mvf_dmaerr", mvf_dma);
		if( ret) return -EBUSY;

	}
	return 0;


#if 0
	int ret;

	ret = clk_prepare_enable(mvf_dma->clk);
	if (ret)
		return ret;

	ret = mxs_reset_block(mvf_dma->base);
	if (ret)
		goto err_out;

err_out:
	return ret;
#endif
}

static int __init mvf_dma_probe(struct platform_device *pdev)
{
	int ret, index,i;
	struct mvf_dma_engine *mvf_dma;
	struct resource *iores, *irq_res, *errirq_res;

	mvf_dma = kzalloc(sizeof(*mvf_dma), GFP_KERNEL);
	if (!mvf_dma)
		return -ENOMEM;

	for(i = 0; i< MVF_MAX_DMA_ENGINE; i ++){
		iores = platform_get_resource(pdev, IORESOURCE_MEM, i);

		if (!request_mem_region(iores->start, resource_size(iores),
					pdev->name)) {
			ret = -EBUSY;
			goto err_request_region;
		}

		mvf_dma->base[i] = ioremap(iores->start, resource_size(iores));
		if (!mvf_dma->base) {
			ret = -ENOMEM;
			goto err_ioremap;
		}

		irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, i*2+0);
		if (!irq_res)
			return -ENODEV;

		mvf_dma->dma_irq[i]=irq_res->start;

		errirq_res = platform_get_resource(pdev, IORESOURCE_IRQ, i*2+1);
		if (!errirq_res)
			return -ENODEV;
		mvf_dma->err_irq[i]=errirq_res->start;
	}

#if 0
	mvf_dma->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(mvf_dma->clk)) {
		ret = PTR_ERR(mvf_dma->clk);
		goto err_clk;
	}
#endif

	dma_cap_set(DMA_MEMCPY, mvf_dma->dma_device.cap_mask);

	INIT_LIST_HEAD(&mvf_dma->dma_device.channels);

	/* Initialize channel parameters */
	for (i = 0; i < MVF_EDMA_CHANNELS; i++) {
		struct mvf_dma_chan *mvf_chan = &mvf_dma->mvf_chans[i];

		index =  i / MVF_EACH_DMA_CHANNEL;

		mvf_chan->mvf_dma = mvf_dma;
		mvf_chan->chan.device = &mvf_dma->dma_device;
		mvf_chan->chan_mem_base = mvf_dma->base[index];

		tasklet_init(&mvf_chan->tasklet, mvf_dma_tasklet,
			     (unsigned long) mvf_chan);


		/* Add the channel to mvf_chan list */
		list_add_tail(&mvf_chan->chan.device_node,
			&mvf_dma->dma_device.channels);
	}

	ret = mvf_dma_init(mvf_dma);
	if (ret)
		goto err_init;

	mvf_dma->dma_device.dev = &pdev->dev;

	mvf_dma->dma_device.dev->dma_parms = &mvf_dma->dma_parms;
	dma_set_max_seg_size(mvf_dma->dma_device.dev, MVF_MAX_XFER_BYTES);

	mvf_dma->dma_device.device_alloc_chan_resources = mvf_dma_alloc_chan_resources;
	mvf_dma->dma_device.device_free_chan_resources = mvf_dma_free_chan_resources;
	mvf_dma->dma_device.device_tx_status = mvf_dma_tx_status;
	mvf_dma->dma_device.device_prep_dma_memcpy = mvf_dma_prep_memcpy;
	mvf_dma->dma_device.device_control = mvf_dma_control;
	mvf_dma->dma_device.device_issue_pending = mvf_dma_issue_pending;

	ret = dma_async_device_register(&mvf_dma->dma_device);
	if (ret) {
		dev_err(mvf_dma->dma_device.dev, "unable to register\n");
		goto err_init;
	}

	dev_info(mvf_dma->dma_device.dev, "initialized\n");

	return 0;

err_init:
#if 0
	clk_put(mvf_dma->clk);
err_clk:
	for (i = 0; i < MVF_EDMA_CHANNELS; i++) if (mvf_dma->base[i]) iounmap(mvf_dma->base[i]);
#endif
err_ioremap:
	release_mem_region(iores->start, resource_size(iores));
err_request_region:
	kfree(mvf_dma);
	return ret;
}

static struct platform_driver mvf_dma_driver = {
	.driver		= {
		.name	= "mvf-edma",
	},
};

static int __init mvf_dma_module_init(void)
{
	return platform_driver_probe(&mvf_dma_driver, mvf_dma_probe);
}
subsys_initcall(mvf_dma_module_init);
