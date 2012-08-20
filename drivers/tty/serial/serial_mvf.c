/*
 * Driver for MVF serial ports
 *
 * Copyright 2012 Freescale
 *
 * Based on drivers/tty/serial/imx.c
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

/*!
 * @file drivers/serial/serial_mvf.c
 *
 */


#if defined(CONFIG_SERIAL_MVF_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/mvf_uart.h>
#include <asm/mvf_edma.h>

#define SERIAL_MVF_MAJOR	207
#define MINOR_START		16
#define DRIVER_NAME		"MVF-uart"
#define DEV_NAME		"ttymvf"

#define UART_NR			6

#define MCTRL_TIMEOUT		(250*HZ/1000)

struct mvf_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
	int			txirq, rxirq, rtsirq;
	unsigned int		have_rtscts;
	unsigned int		msb_first;
	struct clk		*clk;
	unsigned short		txfifo;
	unsigned short		rxfifo;
	unsigned char		c2;
	unsigned char		c5;
	struct mutex		txmutex;

	/* DMA fields */
	int			enable_dma;
	struct dma_chan		*dma_chan_rx, *dma_chan_tx;
	struct scatterlist	rx_sgl, tx_sgl[2];
	void			*rx_buf;
	unsigned int		rx_bytes, tx_bytes;
	struct work_struct	tsk_dma_rx, tsk_dma_tx;
	unsigned int		dma_tx_nents;
	bool			dma_is_rxing;
	wait_queue_head_t	dma_wait;
};

#ifdef CONFIG_IRDA
# Error IRDA not implemented yet.
#endif


#ifdef DEBUG_MVF
static void mvf_serial_dump(struct mvf_port *sport)
{
	printk("dmp\n");
	printk("   BH: %02x, BL: %02x\n",
		readb(sport->port.membase + MVF_UART_BDH),
		readb(sport->port.membase + MVF_UART_BDL));
	printk("   C1: %02x, C2: %02x C3:%02x\n",
		readb(sport->port.membase + MVF_UART_C1),
		readb(sport->port.membase + MVF_UART_C2),
		readb(sport->port.membase + MVF_UART_C3));
	printk("   S1: %02x, S2: %02x\n",
		readb(sport->port.membase + MVF_UART_S1),
		readb(sport->port.membase + MVF_UART_S2));
	printk("ret\n");
}
#else
#define mvf_serial_dump(x)
#endif

/*
 * Handle any change of modem status signal since we were last called.
 */
#if 0
static void mvf_mctrl_check(struct mvf_port *sport)
{
	/*
	 * TBD
	 */
	bdh = readb(sport->port.membase + MVF_UART_BDH) & 0xc0;
}
#endif

static inline int mvf_set_bps(struct mvf_port *sport, unsigned long base,
			unsigned long bps)
{
	unsigned char bdh,bdl,brfa, c4;
	unsigned long sbr;

	sbr = (base/(16*bps))&0x1fff;
	bdh = (sbr>>8);
	bdl = (sbr&0xff);
	writeb(bdh, sport->port.membase + MVF_UART_BDH);
	writeb(bdl, sport->port.membase + MVF_UART_BDL);

	brfa = 0;
	if (base%(16*bps)) {
		unsigned long tmp1, tmp2;

		for (brfa = 1; brfa < 32; brfa++) {
			tmp1 = (base*2)/(sbr*32+brfa);
			if (tmp1 == bps)
				break;
			else if (tmp1 < bps) {
				tmp2 = (base*2)/(sbr*32+(brfa-1));
				if ((tmp2-bps) < (bps-tmp1))
					brfa--;
				break;
			}
		}
		if (brfa >= 32)
			brfa = 31;
	}
	c4 = readb(sport->port.membase + MVF_UART_C4) & ~0x1f;
	c4 |= brfa;
	writeb(c4, sport->port.membase + MVF_UART_C4);

	return 0;
}

static inline int mvf_uart_enable(struct mvf_port *sport)
{
	unsigned char c2;

	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 |= (UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);
	return 0;
}

static inline int mvf_uart_disable(struct mvf_port *sport)
{
	unsigned char c2;

	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 &= ~(UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);
	return 0;
}


/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void mvf_timeout(unsigned long data)
{
	struct mvf_port *sport = (struct mvf_port *)data;
	unsigned long flags;

	if (sport->port.state) {
		spin_lock_irqsave(&sport->port.lock, flags);
		//		mvf_mctrl_check(sport);
		spin_unlock_irqrestore(&sport->port.lock, flags);

		mod_timer(&sport->timer, jiffies + MCTRL_TIMEOUT);
	}
}

/*
 * interrupts disabled on entry
 */
static void mvf_stop_tx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned char c2;

	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	writeb(c2 & ~(UART_C2_TIE), sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);
}

/*
 * interrupts disabled on entry
 */
static void mvf_stop_rx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned char c2;

	/*
	 * We are in SMP now, so if the DMA RX thread is running,
	 * we have to wait for it to finish.
	 */
	if (sport->enable_dma && sport->dma_is_rxing)
		return;

	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	writeb(c2 & ~(UART_C2_RE), sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void mvf_enable_ms(struct uart_port *port)
{
	/*
	 * TBD
	 */
	return ;
}

static inline void mvf_transmit_buffer(struct mvf_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;

	while (!uart_circ_empty(xmit) &&
	       readb(sport->port.membase + MVF_UART_TCFIFO) < sport->txfifo) {
		/* send xmit->buf[xmit->tail] out the port here */
		writeb(xmit->buf[xmit->tail], sport->port.membase + MVF_UART_D);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		mvf_stop_tx(&sport->port);
}

static void dma_tx_callback(void *data)
{
	struct mvf_port *sport = data;
	struct scatterlist *sgl = &sport->tx_sgl[0];
	struct circ_buf *xmit = &sport->port.state->xmit;

	dma_unmap_sg(sport->port.dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);

	/* update the stat */
	spin_lock(&sport->port.lock);
	xmit->tail = (xmit->tail + sport->tx_bytes) & (UART_XMIT_SIZE - 1);
	sport->port.icount.tx += sport->tx_bytes;
	spin_unlock(&sport->port.lock);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	schedule_work(&sport->tsk_dma_tx);
}

static void dma_tx_work(struct work_struct *w)
{
	struct mvf_port *sport = container_of(w, struct mvf_port, tsk_dma_tx);
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct scatterlist *sgl = &sport->tx_sgl[0];
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *chan = sport->dma_chan_tx;
	enum dma_status status;
	unsigned long flags;
	int ret;

	status = chan->device->device_tx_status(chan, (dma_cookie_t)NULL, NULL);
	if (DMA_IN_PROGRESS == status)
		return;

	spin_lock_irqsave(&sport->port.lock, flags);
	sport->tx_bytes = uart_circ_chars_pending(xmit);
	if (sport->tx_bytes > 0) {
		if (xmit->tail > xmit->head) {
			sport->dma_tx_nents = 2;
			sg_init_table(sgl, 2);
			sg_set_buf(sgl, xmit->buf + xmit->tail,
					UART_XMIT_SIZE - xmit->tail);
			sg_set_buf(&sgl[1], xmit->buf, xmit->head);
		} else {
			sport->dma_tx_nents = 1;
			sg_init_one(sgl, xmit->buf + xmit->tail,
					sport->tx_bytes);
		}
		spin_unlock_irqrestore(&sport->port.lock, flags);

		ret = dma_map_sg(sport->port.dev, sgl,
				sport->dma_tx_nents, DMA_TO_DEVICE);
		if (ret == 0) {
			pr_err("DMA mapping error for TX.\n");
			return;
		}
		desc = chan->device->device_prep_slave_sg(chan, sgl,
				sport->dma_tx_nents, DMA_MEM_TO_DEV, 0);
		if (!desc) {
			pr_err("We cannot prepare for the TX slave dma!\n");
			return;
		}
		desc->callback = dma_tx_callback;
		desc->callback_param = sport;

		/* fire it */
		dmaengine_submit(desc);
		return;
	}
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return;
}

/*
 * interrupts disabled on entry
 */
static void mvf_start_tx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned char c2;

	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	writeb(c2 | UART_C2_TIE , sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);

	if (sport->enable_dma)
		schedule_work(&sport->tsk_dma_tx);
}

static irqreturn_t mvf_txint(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock,flags);
	if (sport->port.x_char){
		/* Send next char */
		writeb(sport->port.x_char, sport->port.membase + MVF_UART_D);
		sport->port.x_char = 0;
		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		mvf_stop_tx(&sport->port);
		goto out;
	}

	mvf_transmit_buffer(sport);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

out:
	spin_unlock_irqrestore(&sport->port.lock,flags);
	return IRQ_HANDLED;
}

static irqreturn_t mvf_rxint(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	unsigned int rx,flg,ignored = 0;
	struct tty_struct *tty = sport->port.state->port.tty;
	unsigned long flags;
	unsigned char sfifo,s1,c1;

	spin_lock_irqsave(&sport->port.lock,flags);

	c1 = readb(sport->port.membase + MVF_UART_C1);
	while (readb(sport->port.membase + MVF_UART_RCFIFO)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		sfifo = readb(sport->port.membase + MVF_UART_SFIFO);
		writeb(sfifo&0x05, sport->port.membase + MVF_UART_SFIFO);
		s1 = readb(sport->port.membase + MVF_UART_S1);
		rx = (unsigned int)readb(sport->port.membase + MVF_UART_D);
		if (c1 & UART_C1_PE)
			rx &= 0x7F;

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if ((sfifo & (UART_SFIFO_RXOF | UART_SFIFO_RXUF))
			|| (s1 & (UART_S1_PF | UART_S1_FE))) {
			if (s1 & UART_S1_PF)
				sport->port.icount.parity++;
			else if (s1 & UART_S1_FE)
				sport->port.icount.frame++;
			if (s1 & UART_S1_OR)
				sport->port.icount.overrun++;

			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			if (s1 & UART_S1_PF )
				flg = TTY_PARITY;
			else if (s1 & UART_S1_FE)
				flg = TTY_FRAME;
			if (s1 & UART_S1_OR)
				flg = TTY_OVERRUN;
			
#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		tty_insert_flip_char(tty, rx, flg);
	}

out:
	spin_unlock_irqrestore(&sport->port.lock,flags);
	tty_flip_buffer_push(tty);
	return IRQ_HANDLED;
}

static void mvf_dma_rxint(struct mvf_port *sport)
{
	unsigned char c5;

	if (!sport->dma_is_rxing) {
		sport->dma_is_rxing = true;
		/* disable rxint and enable rx dma */
		c5 = readb(sport->port.membase + MVF_UART_C5) | UART_C5_RDMAS;
		writeb(c5, sport->port.membase + MVF_UART_C5);
		sport->c5 = c5;

		/* tell the DMA to receive the data. */
		schedule_work(&sport->tsk_dma_rx);
	}
}

static irqreturn_t mvf_int(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	unsigned char s1,s2;

	s1 = readb(sport->port.membase + MVF_UART_S1);
	s2 = readb(sport->port.membase + MVF_UART_S2);

	/* DEBUG */ mvf_serial_dump(sport);

	/* Check RXINT */
	if (s1 & UART_S1_RDRF) {
		if (sport->enable_dma)
			mvf_dma_rxint(sport);
		else
			mvf_rxint(irq, dev_id);
	}

	if (s1 & UART_S1_TDRE){
		mvf_txint(irq, dev_id);
	}

	if ( s2 )
		readb(sport->port.membase + MVF_UART_S2);

	//	writeb(s1,sport->port.membase + MVF_UART_S1);

	return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int mvf_tx_empty(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	return (readb(sport->port.membase + MVF_UART_S1) 
			& UART_S1_TC)?TIOCSER_TEMT : 0;
}

static unsigned int mvf_get_mctrl(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned int ret = TIOCM_DSR | TIOCM_CAR;
	unsigned char modem = readb(sport->port.membase + MVF_UART_MODEM);

	if (modem & UART_MODEM_RXRTSE)
		ret |= TIOCM_RTS;
	if (modem & UART_MODEM_TXCTSE)
		ret |= TIOCM_CTS;

	return ret;
}

static void mvf_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/*
	 * We use MVF UART's hardware for CTS/RTS, so don't need any for that.
	 */
}

/*
 * Interrupts always disabled.
 */
static void mvf_break_ctl(struct uart_port *port, int break_state)
{
	/*
	 * TBD
	 */
}

#define TXWATER		2 /* FIXME */
#define RXWATER		1 /* FIXME */
#define GET_FSIZE(x)	((x)==0?1:1<<((x)+1))
#define RX_BUF_SIZE	(PAGE_SIZE)

static bool mvf_uart_filter(struct dma_chan *chan, void *param)
{
	struct mvf_port *sport = param;

	if (strcmp(dev_name(chan->device->dev), "mvf-edma.0") &&
	    strcmp(dev_name(chan->device->dev), "mvf-edma.1")) {
		printk("dev_name:%s\n", dev_name(chan->device->dev));
		return false;
	}

	if (!strcmp(dev_name(chan->device->dev), "mvf-edma.0")) {
		if (sport->port.line >= 4) {
			if (chan->chan_id < 16)
				return false;
		} else {
			if (chan->chan_id >= 16)
				return false;
		}
	} else {
		if (sport->port.line >= 4) {
			if (chan->chan_id >= 16)
				return false;
		} else {
			if (chan->chan_id < 16)
				return false;
		}
	}
	return true;
}

static int start_rx_dma(struct mvf_port *sport);

static void dma_rx_work(struct work_struct *w)
{
	struct mvf_port *sport = container_of(w, struct mvf_port, tsk_dma_rx);
	struct tty_struct *tty = sport->port.state->port.tty;

	if (sport->rx_bytes) {
		tty_insert_flip_string(tty, sport->rx_buf, sport->rx_bytes);
		tty_flip_buffer_push(tty);
		sport->rx_bytes = 0;
	}

	if (sport->dma_is_rxing)
		start_rx_dma(sport);
}

static void mvf_finish_dma(struct mvf_port *sport)
{
	unsigned long flags;
	unsigned char c5;

	spin_lock_irqsave(&sport->port.lock, flags);
	mutex_lock(&sport->txmutex);

	/* enable rx int and disable rx dma */
	c5 = readb(sport->port.membase + MVF_UART_C5) & ~(UART_C5_RDMAS);
	writeb(c5, sport->port.membase + MVF_UART_C5);
	sport->c5 = c5;

	sport->dma_is_rxing = false;
	if (waitqueue_active(&sport->dma_wait))
		wake_up(&sport->dma_wait);

	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock,flags);
}

static void dma_rx_callback(void *data)
{
	struct mvf_port *sport = data;
	struct dma_chan *chan = sport->dma_chan_rx;
	unsigned int count;
	struct tty_struct *tty;
	struct scatterlist *sgl;
	struct dma_tx_state state;
	enum dma_status status;

	tty = sport->port.state->port.tty;
	sgl = &sport->rx_sgl;

	/* unmap it first */
	dma_unmap_sg(sport->port.dev, sgl, 1, DMA_FROM_DEVICE);

	/* If we have finish the reading. we will not accept any more data. */
	if (tty->closing) {
		mvf_finish_dma(sport);
		return;
	}

	status = chan->device->device_tx_status(chan,
					(dma_cookie_t)NULL, &state);
	count = RX_BUF_SIZE - state.residue;
	if (count) {
		sport->rx_bytes = count;
		schedule_work(&sport->tsk_dma_rx);
	} else
		mvf_finish_dma(sport);
}

static int start_rx_dma(struct mvf_port *sport)
{
	struct scatterlist *sgl = &sport->rx_sgl;
	struct dma_chan *chan = sport->dma_chan_rx;
	struct dma_async_tx_descriptor *desc;
	int ret;

	sg_init_one(sgl, sport->rx_buf, RX_BUF_SIZE);
	ret = dma_map_sg(sport->port.dev, sgl, 1, DMA_FROM_DEVICE);
	if (ret == 0) {
		pr_err("DMA mapping error for RX.\n");
		return -EINVAL;
	}
	desc = chan->device->device_prep_slave_sg(chan,
				sgl, 1, DMA_DEV_TO_MEM, 0);
	if (!desc) {
		pr_err("We cannot prepare for the RX slave dma!\n");
		return -EINVAL;
	}
	desc->callback = dma_rx_callback;
	desc->callback_param = sport;

	dmaengine_submit(desc);
	return 0;
}

static void mvf_uart_dma_exit(struct mvf_port *sport)
{
	if (sport->dma_chan_rx) {
		dma_release_channel(sport->dma_chan_rx);
		sport->dma_chan_rx = NULL;
		if (sport->rx_buf) {
			kfree(sport->rx_buf);
			sport->rx_buf = NULL;
		}
	}

	if (sport->dma_chan_tx) {
		dma_release_channel(sport->dma_chan_tx);
		sport->dma_chan_tx = NULL;
	}
}

static void mvf_uart_set_dma_mux(struct mvf_port *sport)
{
	struct mvfuart_platform_data *pdata = sport->port.dev->platform_data;
	void __iomem *base;
	unsigned char val;
	int dma_num, ch;

	//FIXME
	/* setup RX DMAMUX */
	if (!strcmp(dev_name(sport->dma_chan_rx->device->dev), "mvf-edma.0"))
		dma_num = 0;
	else
		dma_num = 1;
	ch = sport->dma_chan_rx->chan_id;

	if (!dma_num) {
		if (ch < 32)
			base = MVF_IO_ADDRESS(MVF_DMAMUX0_BASE_ADDR);
		else
			base = MVF_IO_ADDRESS(MVF_DMAMUX1_BASE_ADDR);
	} else {
		if (ch < 32)
			base = MVF_IO_ADDRESS(MVF_DMAMUX2_BASE_ADDR);
		else
			base = MVF_IO_ADDRESS(MVF_DMAMUX3_BASE_ADDR);
	}
	ch %= 32;

	writeb(0x0, base + ch);
	val = 0x80 + pdata->dma_req_rx;
	writeb(val, base + ch);

	/* setup TX DMAMUX */
	if (!strcmp(dev_name(sport->dma_chan_tx->device->dev), "mvf-edma.0"))
		dma_num = 0;
	else
		dma_num = 1;
	ch = sport->dma_chan_tx->chan_id;

	if (!dma_num) {
		if (ch < 32)
			base = MVF_IO_ADDRESS(MVF_DMAMUX0_BASE_ADDR);
		else
			base = MVF_IO_ADDRESS(MVF_DMAMUX1_BASE_ADDR);
	} else {
		if (ch < 32)
			base = MVF_IO_ADDRESS(MVF_DMAMUX2_BASE_ADDR);
		else
			base = MVF_IO_ADDRESS(MVF_DMAMUX3_BASE_ADDR);
	}
	ch %= 32;

	writeb(0x0, base + ch);
	val = 0x80 + pdata->dma_req_tx;
	writeb(val, base + ch);
}

static int mvf_uart_dma_init(struct mvf_port *sport)
{
	struct dma_slave_config slave_config;
	dma_cap_mask_t mask;
	int ret;

	/* prepare for RX : */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	sport->dma_chan_rx = dma_request_channel(mask, mvf_uart_filter, sport);
	if (!sport->dma_chan_rx) {
		pr_err("cannot get the RX DMA channel.\n");
		ret = -EINVAL;
		goto error;
	}

	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.src_addr = (dma_addr_t)(sport->port.membase + MVF_UART_D);
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.src_maxburst = readb(sport->port.membase + MVF_UART_RWFIFO); //FIXME
	ret = dmaengine_slave_config(sport->dma_chan_rx, &slave_config);
	if (ret) {
		pr_err("error in RX dma configuration.\n");
		goto error;
	}

	sport->rx_buf = kzalloc(RX_BUF_SIZE, GFP_DMA);
	if (!sport->rx_buf) {
		pr_err("cannot alloc RX DMA buffer.\n");
		ret = -ENOMEM;
		goto error;
	}
	sport->rx_bytes = 0;

	/* prepare for TX : */
	sport->dma_chan_tx = dma_request_channel(mask, mvf_uart_filter, sport);
	if (!sport->dma_chan_tx) {
		pr_err("cannot get the TX DMA channel!\n");
		ret = -EINVAL;
		goto error;
	}

	slave_config.direction = DMA_MEM_TO_DEV;
	slave_config.dst_addr = (dma_addr_t)(sport->port.membase + MVF_UART_D);
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.dst_maxburst = readb(sport->port.membase + MVF_UART_TWFIFO); //FIXME
	ret = dmaengine_slave_config(sport->dma_chan_tx, &slave_config);
	if (ret) {
		pr_err("error in TX dma configuration.");
		goto error;
	}
	mvf_uart_set_dma_mux(sport);

	return 0;
error:
	mvf_uart_dma_exit(sport);
	return ret;
}

static int mvf_startup(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	int retval = 0;
	unsigned long flags;
	struct tty_struct *tty;
	unsigned char c2,c5,s2;

	/*
	 * Disable Interrupts.
	 */
	s2 = readb(sport->port.membase + MVF_UART_S2);
	s2 &= 0xc0; /* Clear Status Interrupts */
	if (sport->msb_first)
		s2 |= 0x20; /* MSB first */
	else
		s2 &= ~0x20; /* LSB first */
	writeb(s2, sport->port.membase + MVF_UART_S2);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(sport->port.irq, mvf_int, IRQ_TYPE_LEVEL_HIGH,
						 DRIVER_NAME, sport);
	if (retval) {
		free_irq(sport->port.irq, sport);
		goto error_out1;
	}

	spin_lock_irqsave(&sport->port.lock, flags);
	mutex_lock(&sport->txmutex);
	/* Set FIFO Watermark */
	if (TXWATER > sport->txfifo)
		writeb(sport->txfifo - 1, sport->port.membase + MVF_UART_TWFIFO);
	else
		writeb(TXWATER, sport->port.membase + MVF_UART_TWFIFO);
	if (RXWATER > sport->rxfifo)
		writeb(sport->rxfifo - 1, sport->port.membase + MVF_UART_RWFIFO);
	else
		writeb(RXWATER, sport->port.membase + MVF_UART_RWFIFO);
	/* FIFO Enable */
	writeb(readb(sport->port.membase + MVF_UART_PFIFO) | 0x88, 
		   sport->port.membase + MVF_UART_PFIFO);

	/*
	 * Finally, clear and enable interrupts
	 */
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 |= (UART_C2_RIE | UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;

	c5 = readb(sport->port.membase + MVF_UART_C5);
	if (sport->enable_dma) {
		mutex_unlock(&sport->txmutex);
		retval = mvf_uart_dma_init(sport);
		mutex_lock(&sport->txmutex);
		if (retval) {
			free_irq(sport->port.irq, sport);
			goto error_out2;
		}
		sport->port.flags |= UPF_LOW_LATENCY; //FIXME
		INIT_WORK(&sport->tsk_dma_tx, dma_tx_work);
		INIT_WORK(&sport->tsk_dma_rx, dma_rx_work);
		init_waitqueue_head(&sport->dma_wait);
		c5 |= UART_C5_TDMAS;
	} else
		c5 = 0;
	writeb(c5, sport->port.membase + MVF_UART_C5);
	sport->c5 = c5;

	mvf_enable_ms(&sport->port);
	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock,flags);

	tty = sport->port.state->port.tty;

	return 0;

error_out2:
	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock,flags);
error_out1:
	return retval;
}

static void mvf_shutdown(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long flags;
	unsigned char c2;

	if (sport->enable_dma) {
		/* We have to wait for the DMA to finish. */
		wait_event(sport->dma_wait, !sport->dma_is_rxing);
		mvf_stop_rx(port);
		mvf_uart_dma_exit(sport);
	}

	spin_lock_irqsave(&sport->port.lock, flags);
	mutex_lock(&sport->txmutex);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 &= ~(UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	spin_unlock_irqrestore(&sport->port.lock, flags);

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

	/*
	 * Free the interrupts
	 */
	free_irq(sport->port.irq, sport);

	/*
	 * Disable all interrupts, port and break condition.
	 */
	spin_lock_irqsave(&sport->port.lock, flags);
	writeb(0, sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;
	writeb(0, sport->port.membase + MVF_UART_C5);
	sport->c5 = 0;
	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock, flags);
}


static void
mvf_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long flags;
	unsigned int baud;
	unsigned char c1,c2,c3;

	/*
	 * Release 1: No support Modem control lines.
	 */
	if (0) {
		termios->c_cflag &= ~(HUPCL | CRTSCTS | CMSPAR);
		termios->c_cflag |= CLOCAL;
	}

	/*
	 * We only support CS8.
	 */
	termios->c_cflag &=~CSIZE;
	termios->c_cflag |= CS8;

	spin_lock_irqsave(&sport->port.lock, flags);
	mutex_lock(&sport->txmutex);
	c1 = readb(sport->port.membase + MVF_UART_C1) & ~UART_C1_M;
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c3 = readb(sport->port.membase + MVF_UART_C3);

	if (termios->c_cflag & CRTSCTS) {
		if (sport->have_rtscts) {
			//printk("uart(%d): enable RTSCTS\n", port->line);
			writeb((UART_MODEM_RXRTSE | UART_MODEM_TXCTSE),
				sport->port.membase + MVF_UART_MODEM);
		} else {
			//printk("uart(%d): disable RTSCTS #1\n", port->line);
			writeb(0, sport->port.membase + MVF_UART_MODEM);
			termios->c_cflag &= ~CRTSCTS;
		}
	} else {
		//printk("uart(%d): disable RTSCTS #2\n", port->line);
		writeb(0, sport->port.membase + MVF_UART_MODEM);
	}

	if (termios->c_cflag & PARENB){
		c1 |= UART_C1_PE;
		if (termios->c_cflag & PARODD) {
			//printk("uart(%d): odd Parity\n", port->line);
			c1 |= UART_C1_PT;
		} else {
			//printk("uart(%d): even Parity\n", port->line);
			c1 &= ~UART_C1_PT;
		}
	} else {
		c1 &= ~(UART_C1_PE | UART_C1_PT);
		//printk("uart(%d): disable Parity\n", port->line);
	}
	
	/*
	 * We only supprt STOPB
	 */
	
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);
	mvf_set_bps(sport,port->uartclk,baud);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	writeb(c1,sport->port.membase + MVF_UART_C1);
	writeb(c2,sport->port.membase + MVF_UART_C2);
	writeb(c3,sport->port.membase + MVF_UART_C3);
	sport->c2 = c2;
	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static const char *mvf_type(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	return sport->port.type == PORT_IMX ? "MVF" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void mvf_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *mmres;

	mmres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mmres->start, mmres->end - mmres->start + 1);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int mvf_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *mmres;
	void *ret;

	mmres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mmres)
		return -ENODEV;

	ret = request_mem_region(mmres->start, mmres->end - mmres->start + 1,
			"mvf-uart");

	return  ret ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void mvf_config_port(struct uart_port *port, int flags)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	if (flags & UART_CONFIG_TYPE &&
	    mvf_request_port(&sport->port) == 0)
		sport->port.type = PORT_IMX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_IMX and PORT_UNKNOWN
 */
static int
mvf_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_IMX)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if ((void *)sport->port.mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}


static struct uart_ops mvf_pops = {
	.tx_empty	= mvf_tx_empty,
	.set_mctrl	= mvf_set_mctrl,
	.get_mctrl	= mvf_get_mctrl,
	.stop_tx	= mvf_stop_tx,
	.start_tx	= mvf_start_tx,
	.stop_rx	= mvf_stop_rx,
	.enable_ms	= mvf_enable_ms,
	.break_ctl	= mvf_break_ctl,
	.startup	= mvf_startup,
	.shutdown	= mvf_shutdown,
	.set_termios	= mvf_set_termios,
	.type		= mvf_type,
	.release_port	= mvf_release_port,
	.request_port	= mvf_request_port,
	.config_port	= mvf_config_port,
	.verify_port	= mvf_verify_port,
};


static struct mvf_port *mvf_ports[UART_NR];

#ifdef CONFIG_SERIAL_MVF_CONSOLE
static void mvf_console_putchar(struct uart_port *port, int ch)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	while (!(readb(sport->port.membase + MVF_UART_S1) & UART_S1_TDRE)) {
		barrier();
	}

	writeb(ch, sport->port.membase + MVF_UART_D);
}

static void
mvf_console_write(struct console *co, const char *s, unsigned int count)
{
	struct mvf_port *sport = mvf_ports[co->index];
	unsigned int status, c2;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	mutex_lock(&sport->txmutex);
	/*
	 * First save the control registers and then disable the interrupts
	 */
	c2 = readb(sport->port.membase + MVF_UART_C2);
	sport->c2 = c2;

	c2 |= (UART_C2_TE );
	writeb(c2, sport->port.membase + MVF_UART_C2);

	uart_console_write(&sport->port, s, count, mvf_console_putchar);

	/*
	 * Finally, wait for the transmitter to become empty
	 */
	do {
		status = readb(sport->port.membase + MVF_UART_S1);
	} while (!(status & UART_S1_TC));

	/*
	 * Restore the control registers
	 */
	writeb(sport->c2, sport->port.membase + MVF_UART_C2);
	mutex_unlock(&sport->txmutex);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	return;
}
/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
mvf_console_get_options(struct mvf_port *sport, int *baud,
                           int *parity, int *bits)
{
	
	/* UART Enable */
	if ( readb(sport->port.membase + MVF_UART_C2) & (UART_C2_TE | UART_C2_RE)) {
		unsigned char c1, brfa;
		unsigned short br;

		c1 = readb(sport->port.membase + MVF_UART_C1);
		*parity = 'n';
		if ( c1 & UART_C1_PE)
			*parity = (c1 & UART_C1_PT)?'o':'e';

		*bits = 8;

		br = readb(sport->port.membase + MVF_UART_BDH)<<8
			| readb(sport->port.membase + MVF_UART_BDL);
		br &= 0x1fff;
		brfa = readb(sport->port.membase + MVF_UART_C4) & 0x1f;
		*baud = (sport->port.uartclk*2)/(br*32+brfa);
	}
}

static int __init
mvf_console_setup(struct console *co,char *options)
{
	struct mvf_port *sport;
	int baud = 9600;
	int bits = 8;
	int parity  = 'n';
	int flow = 'n';


	if (co->index == -1 || co->index >= ARRAY_SIZE(mvf_ports))
		co->index = 0;
	sport = mvf_ports[co->index];
	if(sport == NULL)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		mvf_console_get_options(sport, &baud, &parity, &bits);


	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}


static struct uart_driver mvf_reg;
static struct console mvf_console = {
	.name		= DEV_NAME,
	.write		= mvf_console_write,
	.device		= uart_console_device,
	.setup		= mvf_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &mvf_reg,
};

#define MVF_CONSOLE	&mvf_console
#else
#define MVF_CONSOLE	NULL
#endif //CONFIG_SERIAL_MVF_CONSOLE

static struct uart_driver mvf_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_MVF_MAJOR,
	.minor          = MINOR_START,
	.nr             = ARRAY_SIZE(mvf_ports),
	.cons           = MVF_CONSOLE,
};
	

#ifdef CONFIG_PM
static int serial_mvf_suspend(struct platform_device *dev,pm_message_t state)
{
	/* TBD */
	return 0;
}

static int serial_mvf_resume(struct platform_device *dev)
{
	/* TBD */
	return 0;
}

static const struct dev_pm_ops serial_mvf_pm_ops = {
	.suspend  = serial_mvf_suspend,
	.resume   = serial_mvf_resume,
};
#endif
	

static int serial_mvf_probe(struct platform_device *pdev)
{
	struct mvf_port *sport;
	struct mvfuart_platform_data *pdata;
	void __iomem *base;
	int ret = 0;
	struct resource *res;
	unsigned char fifo;

	/* Allocate mvf port struct */
	sport = kzalloc(sizeof(*sport),GFP_KERNEL);

	if ( !sport )
		return -ENOMEM;

	/* Get memory resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if ( !res ){
		ret = ENODEV;
		goto free;
	}

	base = ioremap(res->start, PAGE_SIZE);

	sport->port.dev = &pdev->dev;
	sport->port.mapbase = res->start;
	sport->port.membase = base;
	sport->port.type = PORT_IMX;
	sport->port.iotype = UPIO_MEM;
	/* Get IRQ 1 */
	sport->port.irq = platform_get_irq(pdev, 0);

	sport->port.ops = &mvf_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;
	sport->port.line = pdev->id;
	init_timer(&sport->timer);
	sport->timer.function = mvf_timeout;
	sport->timer.data     = (unsigned long)sport;

	sport->clk = clk_get(&pdev->dev, "uart");
;

	if (IS_ERR(sport->clk)) {
		ret = PTR_ERR(sport->clk);
		goto unmap;
	}
	clk_enable(sport->clk);

	sport->port.uartclk = clk_get_rate(sport->clk);
	mvf_ports[pdev->id] = sport;

	mutex_init(&sport->txmutex);

	pdata = pdev->dev.platform_data;
	if (pdata && (pdata->flags & MVFUART_HAVE_RTSCTS))
		sport->have_rtscts = 1;
#if 0 //FIXME
	if (pdata && (pdata->flags & MVFUART_USE_DCEDTE))
		sport->use_dcedte = 1;
	if (pdata && (pdata->flags & MVFUART_SDMA))
		sport->enable_dma = 1;
#endif
	if (pdata && (pdata->flags & MVFUART_MSB_FIRST))
		sport->msb_first = 1;

	/* Get FIFO Size */
	fifo = readb(sport->port.membase + MVF_UART_PFIFO);
	sport->txfifo = GET_FSIZE((fifo>>4)&0x07);
	sport->rxfifo = GET_FSIZE((fifo)&0x07);
	sport->port.fifosize = sport->txfifo;

	if (pdata && pdata->init) {
		ret = pdata->init(pdev);
		if (ret)
			goto clkput;
	}

	ret = uart_add_one_port(&mvf_reg, &sport->port);
	if (ret)
		goto deinit;
	platform_set_drvdata(pdev, &sport->port);

	return 0;
deinit:
	if (pdata && pdata->exit)
		pdata->exit(pdev);
clkput:
	clk_put(sport->clk);
	clk_disable(sport->clk);
unmap:
	iounmap(sport->port.membase);
free:
	kfree(sport);

	return ret;
}


static int serial_mvf_remove(struct platform_device *pdev)
{
	struct mvfuart_platform_data *pdata;
	struct mvf_port *sport = platform_get_drvdata(pdev);
	
	pdata = pdev->dev.platform_data;
	
	platform_set_drvdata(pdev, NULL);
	
	if (sport){
		uart_remove_one_port(&mvf_reg, &sport->port);
	}

	clk_disable(sport->clk);

	if (pdata && pdata->exit)
		pdata->exit(pdev);
	
	iounmap(sport->port.membase);
	kfree(sport);
	
	return 0;
}

static struct platform_driver serial_mvf_driver = {
	.probe    = serial_mvf_probe,
	.remove   = serial_mvf_remove,

	.driver   = {
		.name    = "mvf-uart",
		.owner   = THIS_MODULE,
#ifdef CONFIG_PM
		.pm      = &serial_mvf_pm_ops,
#endif
	},
};


static int __init mvf_serial_init(void)
{
	int ret;
	printk(KERN_INFO "Serial: MVF driver\n");
	ret = uart_register_driver(&mvf_reg);
	if ( ret )
		return ret;

	ret = platform_driver_register(&serial_mvf_driver);
	if ( ret != 0 )
		uart_unregister_driver(&mvf_reg);

	return 0;
}


static void __exit mvf_serial_exit(void)
{
	platform_driver_unregister(&serial_mvf_driver);
	uart_unregister_driver(&mvf_reg);
}


module_init(mvf_serial_init);
module_exit(mvf_serial_exit);

MODULE_AUTHOR("Freescale");
MODULE_DESCRIPTION("MVF serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mvf-uart");

