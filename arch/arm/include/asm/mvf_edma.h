 /*
  * mvf_edma.h - mvf eDMA driver header file.
  *
  * Copyright 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
  *
  *  Add support for mvf platform (Lanttor.Guo@freescale.com)
  *
  * This program is free software; you can redistribute  it and/or modify it
  * under  the terms of  the GNU General  Public License as published by the
  * Free Software Foundation;  either version 2 of the  License, or (at your
  * option) any later version.
  */

#ifndef _MCF_EDMA_H
#define _MCF_EDMA_H

#include <linux/interrupt.h>
#include <asm/mvf_edma_regs.h>
#include <linux/scatterlist.h>

#define DMA_MODE_READ		0
#define DMA_MODE_WRITE		1
#define DMA_MODE_MASK		1


#define MCF_EDMA_INT0_CHANNEL_BASE 		(8)
#define MCF_EDMA_INT0_CONTROLLER_BASE 	(64)
#define MCF_EDMA_INT0_BASE				(MCF_EDMA_INT0_CHANNEL_BASE + MCF_EDMA_INT0_CONTROLLER_BASE)
#define MCF_EDMA_INT0_NUM				(16)
#define MCF_EDMA_INT0_END				(MCF_EDMA_INT0_NUM)

#define MCF_EDMA_INT1_CHANNEL_BASE 		(8)
#define MCF_EDMA_INT1_CONTROLLER_BASE	(128)
#define MCF_EDMA_INT1_BASE				(MCF_EDMA_INT1_CHANNEL_BASE + MCF_EDMA_INT1_CONTROLLER_BASE)
#define MCF_EDMA_INT1_NUM				(40)
#define MCF_EDMA_INT1_END				(MCF_EDMA_INT0_END + MCF_EDMA_INT1_NUM)

#define MCF_EDMA_INT2_CHANNEL_BASE 		(0)
#define MCF_EDMA_INT2_CONTROLLER_BASE	(192)
#define MCF_EDMA_INT2_BASE				(MCF_EDMA_INT2_CHANNEL_BASE + MCF_EDMA_INT2_CONTROLLER_BASE)
#define MCF_EDMA_INT2_NUM				(8)
#define MCF_EDMA_INT2_END				(MCF_EDMA_INT1_END + MCF_EDMA_INT2_NUM)

#define MCF_EDMA_CHANNEL_ANY			(0xFF)
#define MCF_EDMA_INT_ERR				(16)	/* edma error interrupt */

#define MCF_EDMA_TCD_PER_CHAN				256
#define MVF_EACH_DMA_CHANNEL	32

/* Setup transfer control descriptor (TCD)
 *   channel - descriptor number
 *   source  - source address
 *   dest    - destination address
 *   attr    - attributes
 *   soff    - source offset
 *   nbytes  - number of bytes to be transfered in minor loop
 *   slast   - last source address adjustment
 *   citer   - major loop count
 *   biter   - begining minor loop count
 *   doff    - destination offset
 *   dlast_sga - last destination address adjustment
 *   major_int - generate interrupt after each major loop
 *   disable_req - disable DMA request after major loop
 */
#if 0
void mvf_edma_set_tcd_params(int channel, u32 source, u32 dest,
			     u32 attr, u32 soff, u32 nbytes, u32 slast,
			     u32 citer, u32 biter, u32 doff, u32 dlast_sga,
			     int major_int, int disable_req);
#endif

/* Setup transfer control descriptor (TCD) and enable halfway irq
 *   channel - descriptor number
 *   source  - source address
 *   dest    - destination address
 *   attr    - attributes
 *   soff    - source offset
 *   nbytes  - number of bytes to be transfered in minor loop
 *   slast   - last source address adjustment
 *   biter   - major loop count
 *   doff    - destination offset
 *   dlast_sga - last destination address adjustment
 *   disable_req - disable DMA request after major loop
 */
void mvf_edma_set_tcd_params_halfirq(int channel, u32 source, u32 dest,
				     u32 attr, u32 soff, u32 nbytes, u32 slast,
				     u32 biter, u32 doff, u32 dlast_sga,
				     int disable_req);

/* check if dma is done
 *   channel - descriptor number
 *   return 1 if done
 */
int mvf_edma_check_done(int channel);

#if 0

/* Starts eDMA transfer on specified channel
 *   channel - eDMA TCD number
 */
static inline void
mvf_edma_start_transfer(int channel)
{
	MCF_EDMA_SERQ = channel;
	MCF_EDMA_SSRT = channel;
}

/* Restart eDMA transfer from halfirq
 *   channel - eDMA TCD number
 */
static inline void
mvf_edma_confirm_halfirq(int channel)
{
	/*MCF_EDMA_TCD_CSR(channel) = 7;*/
	MCF_EDMA_SSRT = channel;
}

/* Starts eDMA transfer on specified channel based on peripheral request
 *   channel - eDMA TCD number
 */
static inline void  mvf_edma_enable_transfer(int channel)
{
	MCF_EDMA_SERQ = channel;
}


/* Stops eDMA transfer
 *   channel - eDMA TCD number
 */
static inline void
mvf_edma_stop_transfer(int channel)
{
	MCF_EDMA_CINT = channel;
	MCF_EDMA_CERQ = channel;
}

/* Confirm that interrupt has been handled
 *   channel - eDMA TCD number
 */
static inline void
mvf_edma_confirm_interrupt_handled(int channel)
{
	MCF_EDMA_CINT = channel;
}
#endif


/**
 * mvf_edma_request_channel - Request an eDMA channel
 * @channel: channel number. In case it is equal to EDMA_CHANNEL_ANY
 *		it will be allocated a first free eDMA channel.
 * @handler: dma handler
 * @error_handler: dma error handler
 * @irq_level: irq level for the dma handler
 * @arg: argument to pass back
 * @lock: optional spinlock to hold over interrupt
 * @device_id: device id
 *
 * Returns allocatedd channel number if success or
 * a negative value if failure.
 */
int mvf_edma_request_channel(int channel,
			     irqreturn_t(*handler) (int, void *),
			     void (*error_handler) (int, void *),
			     u8 irq_level,
			     void *arg,
			     spinlock_t *lock, const char *device_id);

/**
 * Update the channel callback/arg
 * @channel: channel number
 * @handler: dma handler
 * @error_handler: dma error handler
 * @arg: argument to pass back
 *
 * Returns 0 if success or a negative value if failure
 */
int mvf_edma_set_callback(int channel,
			  irqreturn_t(*handler) (int, void *),
			  void (*error_handler) (int, void *), void *arg);

/**
 * Free the edma channel
 * @channel: channel number
 * @arg: argument created with
 *
 * Returns 0 if success or a negative value if failure
 */
int mvf_edma_free_channel(int channel, void *arg);

void mvf_edma_dump_channel(int channel);

#endif				/* _MCF_EDMA_H */
