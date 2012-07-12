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

//#define SCATTER_TEST

#define MVF_MODE_MEMCPY	0x01
#define MVF_MODE_CYCLIC	0x02
#define MVF_MODE_SC		0x03

#define MVF_MAX_DMA_ENGINE		1
#define MVF_EDMA_CHANNELS		(MVF_MAX_DMA_ENGINE*MVF_EACH_DMA_CHANNEL)
#define MVF_MAX_XFER_BYTES		2048

struct mvf_dma_chan {
	struct mvf_dma_engine		*mvf_dma;
	struct dma_chan			chan;
	struct dma_async_tx_descriptor	desc;
	struct tasklet_struct		tasklet;
	dma_addr_t			per_address;
	unsigned long		watermark_level;
	enum dma_slave_buswidth		word_size;
	unsigned int dma_mode;
	int					desc_count;
	dma_cookie_t			last_completed;
	enum dma_status			status;
	unsigned int			flags;
	unsigned int			resbytes;
	void __iomem			*chan_mem_base;
	struct scatterlist		*sg_list;
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

void mvf_dma_regdump( void __iomem *base, int channel)
{
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_SADDR(base, channel),	(unsigned int)MVF_EDMA_TCD_SADDR(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_DADDR(base, channel),	(unsigned int)MVF_EDMA_TCD_DADDR(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_ATTR(base, channel), 	(unsigned int)MVF_EDMA_TCD_ATTR(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_SOFF(base, channel),		(unsigned int)MVF_EDMA_TCD_SOFF(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_NBYTES(base, channel),	(unsigned int)MVF_EDMA_TCD_NBYTES(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_SLAST(base, channel),	(unsigned int)MVF_EDMA_TCD_SLAST(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_CITER(base, channel),	(unsigned int)MVF_EDMA_TCD_CITER(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_BITER(base, channel),	(unsigned int)MVF_EDMA_TCD_BITER(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_DOFF(base, channel),		(unsigned int)MVF_EDMA_TCD_DOFF(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_DLAST_SGA(base, channel),(unsigned int)MVF_EDMA_TCD_DLAST_SGA(base, channel));
	printk("REG ADDR=%x  REG%x\n", (unsigned int)&MVF_EDMA_TCD_CSR(base, channel),		(unsigned int)MVF_EDMA_TCD_CSR(base, channel));
}

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

#define MVF_DMA_LENGTH_LOOP	((unsigned int)-1)

static int mvf_dma_sg_next(struct mvf_dma_chan *mvf_chan, struct scatterlist *sg)
{
	unsigned long now;
	int channel = mvf_chan->chan.chan_id % MVF_EACH_DMA_CHANNEL;
	void __iomem *base = mvf_chan->chan_mem_base;
	unsigned short srcflag,dstflag,srcdelta,dstdelta;


	now = min(mvf_chan->resbytes, sg->length);
	if (mvf_chan->resbytes != MVF_DMA_LENGTH_LOOP)
		mvf_chan->resbytes -= now;

	srcflag = dstflag = 0;
	srcdelta = dstdelta = 0;
	if ((mvf_chan->dma_mode & DMA_MODE_MASK) == DMA_MODE_READ){
		MVF_EDMA_TCD_DADDR(base, channel) = sg->dma_address;
		MVF_EDMA_TCD_SADDR(base, channel) = mvf_chan->per_address;
		if ( mvf_chan->word_size == DMA_SLAVE_BUSWIDTH_1_BYTE){
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_8BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_8BIT;
			srcdelta = 1;
			dstdelta = 1;
		}else
		if ( mvf_chan->word_size == DMA_SLAVE_BUSWIDTH_2_BYTES){
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_16BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_16BIT;
			srcdelta = 2;
			dstdelta = 2;
		}else{
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_32BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_32BIT;
			srcdelta = 4;
			dstdelta = 4;
		}

	}else{
		MVF_EDMA_TCD_DADDR(base, channel) = mvf_chan->per_address;
		MVF_EDMA_TCD_SADDR(base, channel) = sg->dma_address;

		if ( mvf_chan->word_size == DMA_SLAVE_BUSWIDTH_1_BYTE){
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_8BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_8BIT;
			srcdelta = 1;
			dstdelta = 0;
		}else
		if ( mvf_chan->word_size == DMA_SLAVE_BUSWIDTH_2_BYTES){
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_16BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_16BIT;
			srcdelta = 2;
			dstdelta = 0;
		}else{
			srcflag = MVF_EDMA_TCD_ATTR_SSIZE_32BIT;
			dstflag = MVF_EDMA_TCD_ATTR_DSIZE_32BIT;
			srcdelta = 4;
			dstdelta = 0;
		}
	}

	MVF_EDMA_TCD_ATTR(base, channel) = srcflag | dstflag;
	MVF_EDMA_TCD_SOFF(base, channel) = MVF_EDMA_TCD_SOFF_SOFF(srcdelta);
	MVF_EDMA_TCD_DOFF(base, channel) = MVF_EDMA_TCD_DOFF_DOFF(dstdelta);
	MVF_EDMA_TCD_NBYTES(base, channel) = MVF_EDMA_TCD_NBYTES_NBYTES(now);

	MVF_EDMA_TCD_SLAST(base, channel) = MVF_EDMA_TCD_SLAST_SLAST(0);
	MVF_EDMA_TCD_CITER(base, channel) = MVF_EDMA_TCD_CITER_CITER(1);
	MVF_EDMA_TCD_BITER(base, channel) = MVF_EDMA_TCD_BITER_BITER(1);
	MVF_EDMA_TCD_DLAST_SGA(base, channel) = MVF_EDMA_TCD_DLAST_SGA_DLAST_SGA(0);

//	mvf_dma_regdump(base, channel);

	MVF_EDMA_TCD_CSR(base,channel) |= MVF_EDMA_TCD_CSR_INT_MAJOR;
	MVF_EDMA_TCD_CSR(base,channel) &= ~MVF_EDMA_TCD_CSR_D_REQ;
	MVF_EDMA_SEEI(base) = MVF_EDMA_SEEI_SEEI(channel);

	return now;
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


static int mvf_dma_handle(struct mvf_dma_chan *mvf_chan)
{
	int ret;
	struct scatterlist *current_sg;

	ret = 0;
	if ( mvf_chan->flags != MVF_MODE_MEMCPY){
		if (mvf_chan->sg_list) {
			current_sg = mvf_chan->sg_list;
			mvf_chan->sg_list = sg_next(mvf_chan->sg_list);
			// prepare next transfer
			if (mvf_chan->sg_list) {
#ifdef SCATTER_TEST
			if ( mvf_chan->sg_list->length != 0){
#endif
				//	re-fill tx parameter
				mvf_dma_sg_next(mvf_chan, mvf_chan->sg_list);
				//	start tx
				mvf_dma_enable_chan(mvf_chan);
				ret = 1;
#ifdef SCATTER_TEST
			}
#endif
			}
		}
	}
	return ret;
}

static irqreturn_t mvf_dma_int_handler(int irq, void *dev_id)
{
	int i,int_src,engine;
	int ret;
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
	//printk("DMA irq occured = CR:%x  ES:%x SRC:%x \n", MVF_EDMA_CR(base), MVF_EDMA_ES(base),MVF_EDMA_INT(base));

	//	read int source and clear soon
	int_src = MVF_EDMA_INT(base);

	//	deliver int source and re-enable (if scatter gather is configured)
	for (i = 0; i < MVF_EACH_DMA_CHANNEL; i++) {
		if ( int_src & (1 << i)){
			// find chan 
			mvf_chan = mvf_find_chan(mvf_dma, engine, i);
			if (mvf_chan){
				ret = mvf_dma_handle(mvf_chan);
				if (ret == 0){
					mvf_chan->status = DMA_SUCCESS;
					mvf_chan->last_completed = mvf_chan->desc.cookie;
							
					/* schedule tasklet on this channel */
					tasklet_schedule(&mvf_chan->tasklet);
				}
			}
			MVF_EDMA_CINT(base) = i;
		}
	}

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
	printk(KERN_INFO"DMA error irq occured = CR:%x  ES:%x\n", (unsigned int)MVF_EDMA_CR(base), (unsigned int)MVF_EDMA_ES(base));

	//	fail safe
	if (!base){
		printk(KERN_INFO"error irq\n");
		return IRQ_HANDLED;
	}

	err = MVF_EDMA_ERR(base);
	for (i = 0; i < MVF_EACH_DMA_CHANNEL; i++) {
		if ( err & (1 << i)){
			mvf_chan = mvf_find_chan(mvf_dma, engine, i);
			if (mvf_chan){
				mvf_chan->last_completed = mvf_chan->desc.cookie;
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

	mvf_dma_reset_chan(mvf_chan);

	dma_async_tx_descriptor_init(&mvf_chan->desc, chan);
	mvf_chan->desc.tx_submit = mvf_dma_tx_submit;

	//	the descriptor is ready
	async_tx_ack(&mvf_chan->desc);

	return 0;
}

static void mvf_dma_free_chan_resources(struct dma_chan *chan)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);

	mvf_dma_disable_chan(mvf_chan);
}


void
mvf_edma_set_tcd_params(struct mvf_dma_chan *mvf_chan, u32 source, u32 dest,
			u32 attr, u32 soff, u32 nbytes, u32 slast,
			u32 citer, u32 biter, u32 doff, u32 dlast_sga,
			int major_int, int disable_req)
{
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

#if 0
	printk("Channel:%d  Top reg:%x\n TX descriptor  src : %x / dest : %x / attr : %x\n",channel, &MVF_EDMA_TCD_SADDR(base, channel), source,dest,attr);
	MVF_EDMA_TCD_SADDR(base, channel) = source&0xfffffff0;
	MVF_EDMA_TCD_DADDR(base, channel) = dest&0xffffff00;
	MVF_EDMA_TCD_ATTR(base, channel) = (0 | MVF_EDMA_TCD_ATTR_SSIZE_32BIT | MVF_EDMA_TCD_ATTR_DSIZE_32BIT);
	MVF_EDMA_TCD_SOFF(base, channel) = 0x4;
	MVF_EDMA_TCD_NBYTES(base, channel) = 16;
	MVF_EDMA_TCD_SLAST(base, channel) = 0;
	MVF_EDMA_TCD_CITER(base, channel) = 1;
	MVF_EDMA_TCD_BITER(base, channel) = 1;
	MVF_EDMA_TCD_DOFF(base, channel) = 4;
	MVF_EDMA_TCD_DLAST_SGA(base, channel) = 0;
	MVF_EDMA_TCD_CSR(base, channel) = 0x0000;
#endif

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


int
mvf_dma_setup_sg(
		struct mvf_dma_chan *mvf_chan,
		unsigned int sgcount,
		unsigned int dma_length,
		unsigned int dmamode)
{
	mvf_chan->dma_mode = dmamode;
	mvf_chan->resbytes = dma_length;

	//	param check
	if (!mvf_chan->sg_list || !sgcount) {
		printk(KERN_ERR "empty sg list\n");
		return -EINVAL;
	}
	if (!mvf_chan->sg_list->length) {
		printk(KERN_ERR "dma_setup_sg zero length\n");
		return -EINVAL;
	}

	mvf_dma_sg_next(mvf_chan, mvf_chan->sg_list);

	return 0;
}

static struct dma_async_tx_descriptor *mvf_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	struct scatterlist *sg;
	int i, ret, dma_length = 0;
	unsigned int dmamode;

	if (mvf_chan->status == DMA_IN_PROGRESS)
		return NULL;

	mvf_chan->status = DMA_IN_PROGRESS;
	mvf_chan->flags = MVF_MODE_SC;

	for_each_sg(sgl, sg, sg_len, i) {
		dma_length += sg->length;
	}

	if (direction == DMA_DEV_TO_MEM)
		dmamode = DMA_MODE_READ;
	else
		dmamode = DMA_MODE_WRITE;

	switch (mvf_chan->word_size) {
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		if (sgl->length & 3 || sgl->dma_address & 3)
			return NULL;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		if (sgl->length & 1 || sgl->dma_address & 1)
			return NULL;
		break;
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		break;
	default:
		return NULL;
	}
	mvf_chan->sg_list = sgl;

	ret = mvf_dma_setup_sg(mvf_chan, sg_len, dma_length, dmamode);

	if (ret)
		return NULL;

	return &mvf_chan->desc;
}

static struct dma_async_tx_descriptor *mvf_dma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	int i, ret;
	unsigned int periods = buf_len / period_len;
	unsigned int dmamode;

	if (mvf_chan->status == DMA_IN_PROGRESS)
		return NULL;

	mvf_chan->status = DMA_IN_PROGRESS;
	mvf_chan->flags = MVF_MODE_CYCLIC;

	if (mvf_chan->sg_list)
		kfree(mvf_chan->sg_list);

	mvf_chan->sg_list = kcalloc(periods + 1,
			sizeof(struct scatterlist), GFP_KERNEL);

	if (!mvf_chan->sg_list)
		return NULL;

	sg_init_table(mvf_chan->sg_list, periods);

	for (i = 0; i < periods; i++) {
		mvf_chan->sg_list[i].page_link = 0;
		mvf_chan->sg_list[i].offset = 0;
		mvf_chan->sg_list[i].dma_address = dma_addr;
		mvf_chan->sg_list[i].length = period_len;
		dma_addr += period_len;
	}

	/* close the loop */
#ifndef SCATTER_TEST
	mvf_chan->sg_list[periods].offset = 0;
	mvf_chan->sg_list[periods].length = 0;
	mvf_chan->sg_list[periods].page_link =
		((unsigned long)mvf_chan->sg_list | 0x01) & ~0x02;
#endif

	if (direction == DMA_DEV_TO_MEM)
		dmamode = DMA_MODE_READ;
	else
		dmamode = DMA_MODE_WRITE;

	ret = mvf_dma_setup_sg(mvf_chan, periods, MVF_DMA_LENGTH_LOOP, dmamode);

	if (ret)
		return NULL;

	return &mvf_chan->desc;
}

static struct dma_async_tx_descriptor *mvf_dma_prep_memcpy
(struct dma_chan *chan,
	dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);

	if (mvf_chan->status == DMA_IN_PROGRESS)
		return NULL;

	mvf_chan->flags = MVF_MODE_MEMCPY;
	mvf_chan->status = DMA_IN_PROGRESS;

	mvf_edma_set_tcd_params(
						mvf_chan,
						src,
						dst,
						(0 | MVF_EDMA_TCD_ATTR_SSIZE_8BIT | MVF_EDMA_TCD_ATTR_DSIZE_8BIT), 
						0x01,
						len, 0x0, 1, 1, 
						0x01, 0x0, 0x1,0x0);
	mvf_chan->desc_count = 0;

	return &mvf_chan->desc;
}


static int mvf_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	struct dma_slave_config *dmaengine_cfg = (void *)arg;
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

		case DMA_SLAVE_CONFIG:
			if (dmaengine_cfg->direction == DMA_DEV_TO_MEM) {
				mvf_chan->per_address = dmaengine_cfg->src_addr;
				mvf_chan->watermark_level = dmaengine_cfg->src_maxburst;
				mvf_chan->word_size = dmaengine_cfg->src_addr_width;
			} else {
				mvf_chan->per_address = dmaengine_cfg->dst_addr;
				mvf_chan->watermark_level = dmaengine_cfg->dst_maxburst;
				mvf_chan->word_size = dmaengine_cfg->dst_addr_width;
			}
			return 0;
		default:
			return -ENOSYS;
	}

	return ret;
}

static enum dma_status mvf_dma_tx_status(struct dma_chan *chan,
			dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct mvf_dma_chan *mvf_chan = to_mvf_dma_chan(chan);
	dma_cookie_t last_used;
	enum dma_status ret;

	last_used = chan->cookie;

	ret = dma_async_is_complete(cookie, mvf_chan->last_completed, last_used);
	dma_set_tx_state(txstate, mvf_chan->last_completed, last_used, 0);

	return ret;
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
		if (!mvf_dma->base[i]) {
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

	dma_cap_set(DMA_MEMCPY, mvf_dma->dma_device.cap_mask);
	dma_cap_set(DMA_SLAVE, mvf_dma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, mvf_dma->dma_device.cap_mask);

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
	mvf_dma->dma_device.device_prep_slave_sg = mvf_prep_slave_sg;
	mvf_dma->dma_device.device_prep_dma_memcpy = mvf_dma_prep_memcpy;
	mvf_dma->dma_device.device_prep_dma_cyclic = mvf_dma_prep_dma_cyclic;

	mvf_dma->dma_device.device_control = mvf_dma_control;
	mvf_dma->dma_device.device_issue_pending = mvf_dma_issue_pending;

	ret = dma_async_device_register(&mvf_dma->dma_device);
	if (ret) {
		dev_err(mvf_dma->dma_device.dev, "unable to register\n");
		goto err_init;
	}

	printk(KERN_INFO "eDMA driver Installed.\n");

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
