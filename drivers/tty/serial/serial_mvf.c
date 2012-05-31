/*
 * Driver for MVF serial ports
 *
 * Copyright 2012
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


#include <mach/mvf_uart.h>

#define DRIVER_NAME  "MVF-uart"

#define UART_NR   6

struct mvf_port {
	struct uart_port   port;
	struct time_list   timer;
	unsigned int       old_status;
	int          txirq,rxirqmrtsirq;
	unsigned int
};

#ifdef CONFIG_IRDA
# Error IRDA not implemented yet.
#endif


static inline int mvf_set_bps(struct mvf_port *sport,
								  unsigned long base,unsigned long bps)
{
	unsigned char bdh,bdl;
	unsigned long sbr;
	sbr = base/((bps*10)+5)/160;
	bdh = readb(sport->port.membase + MVF_UART_BDH) & 0xc0;
	bdh = (sbr>>8) & 0x1f;
	bdl = (sbr&0xff);
	writeb(bdh, sport->port.membase + MVF_UART_BDH);
	writeb(bdl, sport->port.membase + MVF_UART_BDL);
	
}

static inline mvf_set_paerity(struct mvf_port *sport,
								  unsigned long base,unsigned short )

static inline int mvf_uart_enable(struct mvf_port *sport)
{
	unsinged char c2;
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 |= (UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	
}
static inline int mvf_uart_disable(struct mvf_port *sport)
{
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 &= ~(UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);

}


/*
 * Handle any change of modem status signal since we were last called.
 */
static void mvf_mctrl_check(struct mvf_port *sport)
{
	/*
	 * TDB
	 */
	return ;

}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void mvf_timeout(unsigned long data)
{
	/*
	 * TDB
	 */
	return;
}

/*
 * interrupts disabled on entry
 */
static void mvf_stop_tx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long temp;

	if (USE_IRDA(sport)) {
		/* half duplex - wait for end of transmission */
		int n = 256;
		while ((--n > 0) &&
		      !(readb(sport->port.membase + USR2) & USR2_TXDC)) {
			udelay(5);
			barrier();
		}
		/*
		 * irda transceiver - wait a bit more to avoid
		 * cutoff, hardware dependent
		 */
		udelay(sport->trcv_delay);

		/*
		 * half duplex - reactivate receive mode,
		 * flush receive pipe echo crap
		 */
		if (readb(sport->port.membase + USR2) & USR2_TXDC) {
			temp = readb(sport->port.membase + UCR1);
			temp &= ~(UCR1_TXMPTYEN | UCR1_TRDYEN);
			writeb(temp, sport->port.membase + UCR1);

			temp = readb(sport->port.membase + UCR4);
			temp &= ~(UCR4_TCEN);
			writeb(temp, sport->port.membase + UCR4);

			while (readb(sport->port.membase + URXD0) &
			       URXD_CHARRDY)
				barrier();

			temp = readb(sport->port.membase + UCR1);
			temp |= UCR1_RRDYEN;
			writeb(temp, sport->port.membase + UCR1);

			temp = readb(sport->port.membase + UCR4);
			temp |= UCR4_DREN;
			writeb(temp, sport->port.membase + UCR4);
		}
		return;
	}

	temp = readb(sport->port.membase + UCR1);
	writeb(temp & ~UCR1_TXMPTYEN, sport->port.membase + UCR1);
}

/*
 * interrupts disabled on entry
 */
static void mvf_stop_rx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long temp;

	temp = readb(sport->port.membase + UCR2);
	writeb(temp &~ UCR2_RXEN, sport->port.membase + UCR2);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void mvf_enable_ms(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	mod_timer(&sport->timer, jiffies);
}

static inline void mvf_transmit_buffer(struct mvf_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;

	while (!uart_circ_empty(xmit) &&
			!(readb(sport->port.membase + UTS) & UTS_TXFULL)) {
		/* send xmit->buf[xmit->tail]
		 * out the port here */
		writeb(xmit->buf[xmit->tail], sport->port.membase + URTX0);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		mvf_stop_tx(&sport->port);
}


/*
 * interrupts disabled on entry
 */
static void mvf_start_tx(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long temp;

	if (USE_IRDA(sport)) {
		/* half duplex in IrDA mode; have to disable receive mode */
		temp = readb(sport->port.membase + UCR4);
		temp &= ~(UCR4_DREN);
		writeb(temp, sport->port.membase + UCR4);

		temp = readb(sport->port.membase + UCR1);
		temp &= ~(UCR1_RRDYEN);
		writeb(temp, sport->port.membase + UCR1);
	}


	if (USE_IRDA(sport)) {
		temp = readb(sport->port.membase + UCR1);
		temp |= UCR1_TRDYEN;
		writeb(temp, sport->port.membase + UCR1);

		temp = readb(sport->port.membase + UCR4);
		temp |= UCR4_TCEN;
		writeb(temp, sport->port.membase + UCR4);
	}


	if (readb(sport->port.membase + UTS) & UTS_TXEMPTY)
		mvf_transmit_buffer(sport);
}

static irqreturn_t mvf_rtsint(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);

	writeb(USR1_RTSD, sport->port.membase + USR1);
	val = readb(sport->port.membase + USR1) & USR1_RTSS;
	uart_handle_cts_change(&sport->port, !!val);
	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t mvf_txint(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock,flags);
	if (sport->port.x_char)
	{
		/* Send next char */
		writeb(sport->port.x_char, sport->port.membase + URTX0);
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
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock,flags);

	while (readb(sport->port.membase + USR2) & USR2_RDR) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		rx = readb(sport->port.membase + URXD0);

		temp = readb(sport->port.membase + USR2);
		if (temp & USR2_BRCD) {
			writeb(USR2_BRCD, sport->port.membase + USR2);
			if (uart_handle_break(&sport->port))
				continue;
		}

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (rx & (URXD_PRERR | URXD_OVRRUN | URXD_FRMERR) ) {
			if (rx & URXD_PRERR)
				sport->port.icount.parity++;
			else if (rx & URXD_FRMERR)
				sport->port.icount.frame++;
			if (rx & URXD_OVRRUN)
				sport->port.icount.overrun++;

			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			rx &= sport->port.read_status_mask;

			if (rx & URXD_PRERR)
				flg = TTY_PARITY;
			else if (rx & URXD_FRMERR)
				flg = TTY_FRAME;
			if (rx & URXD_OVRRUN)
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


static irqreturn_t mvf_int(int irq, void *dev_id)
{
	struct mvf_port *sport = dev_id;
	unsigned int sts;

	sts = readb(sport->port.membase + USR1);

	if (sts & USR1_RRDY) {
		mvf_rxint(irq, dev_id);
	}

	if (sts & USR1_TRDY &&
			readb(sport->port.membase + UCR1) & UCR1_TXMPTYEN)
		mvf_txint(irq, dev_id);

	if (sts & USR1_RTSD)
		mvf_rtsint(irq, dev_id);

	if (sts & USR1_AWAKE)
		writeb(USR1_AWAKE, sport->port.membase + USR1);

	return IRQ_HANDLED;
}


/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int mvf_tx_empty(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	return (readb(sport->port.membase + USR2) & USR2_TXDC) ?  TIOCSER_TEMT : 0;
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int mvf_get_mctrl(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned int tmp = TIOCM_DSR | TIOCM_CAR;

	if (readb(sport->port.membase + USR1) & USR1_RTSS)
		tmp |= TIOCM_CTS;

	if (readb(sport->port.membase + UCR2) & UCR2_CTS)
		tmp |= TIOCM_RTS;

	return tmp;
}

static void mvf_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long temp;

	temp = readb(sport->port.membase + UCR2) & ~UCR2_CTS;

	if (mctrl & TIOCM_RTS)
		temp |= UCR2_CTS;

	writeb(temp, sport->port.membase + UCR2);
}

/*
 * Interrupts always disabled.
 */
static void mvf_break_ctl(struct uart_port *port, int break_state)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock, flags);

	temp = readb(sport->port.membase + UCR1) & ~UCR1_SNDBRK;

	if ( break_state != 0 )
		temp |= UCR1_SNDBRK;

	writeb(temp, sport->port.membase + UCR1);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

#define TXTL 2 /* reset default */
#define RXTL 1 /* reset default */


static int mvf_startup(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	int retval;
	unsigned long flags, temp;
	struct tty_struct *tty;


	/* disable the DREN bit (Data Ready interrupt enable) before
	 * requesting IRQs
	 */
	temp = readb(sport->port.membase + UCR4);

	if (USE_IRDA(sport))
		temp |= UCR4_IRSC;

	/* set the trigger level for CTS */
	temp &= ~(UCR4_CTSTL_MASK<<  UCR4_CTSTL_SHF);
	temp |= CTSTL<<  UCR4_CTSTL_SHF;

	writeb(temp & ~UCR4_DREN, sport->port.membase + UCR4);

	/*
	 * Allocate the IRQ(s) i.MX1 has three interrupts whereas later
	 * chips only have one interrupt.
	 */
	if (sport->txirq > 0) {
		retval = request_irq(sport->rxirq, mvf_rxint, 0,
				DRIVER_NAME, sport);
		if (retval)
			goto error_out1;

		retval = request_irq(sport->txirq, mvf_txint, 0,
				DRIVER_NAME, sport);
		if (retval)
			goto error_out2;

		/* do not use RTS IRQ on IrDA */
		if (!USE_IRDA(sport)) {
			retval = request_irq(sport->rtsirq, mvf_rtsint,
				     (sport->rtsirq < MAX_INTERNAL_IRQ) ? 0 :
				       IRQF_TRIGGER_FALLING |
				       IRQF_TRIGGER_RISING,
					DRIVER_NAME, sport);
			if (retval)
				goto error_out3;
		}
	} else {
		retval = request_irq(sport->port.irq, mvf_int, 0,
				DRIVER_NAME, sport);
		if (retval) {
			free_irq(sport->port.irq, sport);
			goto error_out1;
		}
	}

	spin_lock_irqsave(&sport->port.lock, flags);
	/*
	 * Finally, clear and enable interrupts
	 */
	writeb(USR1_RTSD, sport->port.membase + USR1);

	temp = readb(sport->port.membase + UCR1);
	temp |= UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN;
	if (sport->enable_dma) {
		temp |= UCR1_RDMAEN | UCR1_TDMAEN;
		/* ICD, wait for more than 32 frames, but it still to short. */
		temp |= UCR1_ICD_REG(3);
	}

	if (USE_IRDA(sport)) {
		temp |= UCR1_IREN;
		temp &= ~(UCR1_RTSDEN);
	}

	writeb(temp, sport->port.membase + UCR1);

	temp = readb(sport->port.membase + UCR2);
	temp |= (UCR2_RXEN | UCR2_TXEN);
	writeb(temp, sport->port.membase + UCR2);

	if (USE_IRDA(sport)) {
		/* clear RX-FIFO */
		int i = 64;
		while ((--i > 0) &&
			(readb(sport->port.membase + URXD0) & URXD_CHARRDY)) {
			barrier();
		}
	}

	if (!cpu_is_mx1()) {
		temp = readb(sport->port.membase + UCR3);
		temp |= MX2_UCR3_RXDMUXSEL;
		writeb(temp, sport->port.membase + UCR3);
	}

	if (USE_IRDA(sport)) {
		temp = readb(sport->port.membase + UCR4);
		if (sport->irda_inv_rx)
			temp |= UCR4_INVR;
		else
			temp &= ~(UCR4_INVR);
		writeb(temp | UCR4_DREN, sport->port.membase + UCR4);

		temp = readb(sport->port.membase + UCR3);
		if (sport->irda_inv_tx)
			temp |= UCR3_INVT;
		else
			temp &= ~(UCR3_INVT);
		writeb(temp, sport->port.membase + UCR3);
	}

	if (sport->enable_dma) {
		temp = readb(sport->port.membase + UCR4);
		temp |= UCR4_IDDMAEN;
		writeb(temp, sport->port.membase + UCR4);
	}

	/*
	 * Enable modem status interrupts
	 */
	mvf_enable_ms(&sport->port);
	spin_unlock_irqrestore(&sport->port.lock,flags);

	if (USE_IRDA(sport)) {
		struct mvfuart_platform_data *pdata;
		pdata = sport->port.dev->platform_data;
		sport->irda_inv_rx = pdata->irda_inv_rx;
		sport->irda_inv_tx = pdata->irda_inv_tx;
		sport->trcv_delay = pdata->transceiver_delay;
		if (pdata->irda_enable)
			pdata->irda_enable(1);
	}

	tty = sport->port.state->port.tty;

	return 0;

error_out3:
	if (sport->txirq)
		free_irq(sport->txirq, sport);
error_out2:
	if (sport->rxirq)
		free_irq(sport->rxirq, sport);
error_out1:
	return retval;
}

static void mvf_shutdown(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long temp;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readb(sport->port.membase + UCR2);
	temp &= ~(UCR2_TXEN);
	writeb(temp, sport->port.membase + UCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (USE_IRDA(sport)) {
		struct mvfuart_platform_data *pdata;
		pdata = sport->port.dev->platform_data;
		if (pdata->irda_enable)
			pdata->irda_enable(0);
	}

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

	/*
	 * Free the interrupts
	 */
	if (sport->txirq > 0) {
		if (!USE_IRDA(sport))
			free_irq(sport->rtsirq, sport);
		free_irq(sport->txirq, sport);
		free_irq(sport->rxirq, sport);
	} else
		free_irq(sport->port.irq, sport);

	/*
	 * Disable all interrupts, port and break condition.
	 */

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readb(sport->port.membase + UCR1);
	temp &= ~(UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN);
	if (USE_IRDA(sport))
		temp &= ~(UCR1_IREN);
	writeb(temp, sport->port.membase + UCR1);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void
mvf_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long flags;
	unsigned char c1,c2,c3,c4;
	
	/*
	 * We only support CS8.
	 */
	termios->c_cflag = CS8;
	
	c1 = UART_C1_M;
	
	if (termios->c_cflag & PARENB){
		c1 |= UART_C1_PE;
		if (termios->c_cflag & PARODD)
			c1 |= UART_C1_PT;
	}
		
	/*
	 * We only supprt STOPB
	 */
	

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);
	
		
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&sport->port.lock, flags);
	
	
	writeb(c1,sport->port.membase + UART_C1);
	writeb(c2,sport->port.membase + UART_C2);
	writeb(c3,sport->port.membase + UART_C3);




	spin_unlock_irqrestore(&sport->port.lock, flags);
		
}

static const char *mvf_type(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	return sport->port.type == PORT_MVF ? "MVF" : NULL;
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
		sport->port.type = PORT_MVF;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_MVF and PORT_UNKNOWN
 */
static int
mvf_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct faradady_port *sport = (struct mvf_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_MVF)
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


static struct mvf_ports[UART_NR];

static void mvf_console_putchar(struct uart_port *port, int ch)
{
	struct imx_port *sport = (struct imx_port *)port;
	
	while (readb(sport->port.membase + S1) & UART_S1_TDRE)
		barrier();
	
	writeb(ch, sport->port.membase + UART_D);
}

static void
mvf_console_write(struct console *co, const char *s, unsigned int count)
{
	uart_console_write(&sport->port, s, count, mvf_console_putchar);
	return;
}
/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
imx_console_get_options(struct imx_port *sport, int *baud,
                           int *parity, int *bits)
{
	/* UART Enable */
	if ( readb(soprt->port.membase + UART_C1) & (UART_C1_TE | UART_C1_RE)) {
		unsigned char c1,c2,c3,c4;
		unsigend short br;
		c1 = readb(soprt->port.membase + UART_C1);
		*parity = 'n';
		if ( c1 & UART_C1_PE)
			*parity = (c1 & UART_C1_PT)?'o':'e';
		*bits = 7;
		br = readb(soprt->port.membase + UART_BDH)<<8
			| readb(soprt->port.membase + UART_BDH);
		br &= 0x1fff;
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
	sport = imx_ports[co->index];
	if(sport == NULL)
		return -ENODEV;
	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}


static struct console mvf_console = {
	.name		= DEV_NAME,
	.write		= mvf_console_write,
	.device		= uart_console_device,
	.setup		= mvf_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &mvf_reg,
};

static struct uart_drive mvf_reg = {
	.onwer          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_MVF_MAJOR,
	.minor          = MINOR_START,
	.nr             = ARRAY_SIZE(mvf_ports),
	.cons           = MVF_CONSOLE,
};
	


static int serial_farada_suspend(struct platform_device *dev,pm_message_t state)
{
	/* TBD */
	return 0;
}

static int serial_mvf_resume(struct platform_device *dev)
{
	/* TBD */
	return 0;
}

#idef CONFIG_PM
static consit struct dev_pm_ops serial_mvf_pm_ops = {
	.suspend  = serial_mvf_suspend,
	.resume   = serial_mvf_resume,
};
#endif
	

static int seryal_mvf_probe(struct platform_device *dev)
{
	struct mvf_port *sport;
	struct mvf_platform_data *pdata;
	void __iomem *base;
	int ret = 0;
	struct resource *res;

	sport = kzalloc(sizeof(*sport),GFP_KERNEL);

	if ( !sport )
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if ( !res ){
		ret = ENODEV;
		goto free;
	}

	base = ioremap(res->start, PAGE_SIZE);
	

	sport->port.dev = &pdev->dev;
	sport->port.mapbase = res->start;
	sport->port.membase = base;
	sport->port.type = PORT_MVF,
	sport->port.iotype = UPIO_MEM;
	sport->port.irq = platform_get_irq(pdev, 0);
	sport->rxirq = platform_get_irq(pdev, 0);
	sport->txirq = platform_get_irq(pdev, 1);
	sport->rtsirq = platform_get_irq(pdev, 2);
	sport->port.fifosize = 32;
	sport->port.ops = &mvf_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;
	sport->port.line = pdev->id;
	init_timer(&sport->timer);
	sport->timer.function = mvf_timeout;
	sport->timer.data     = (unsigned long)sport;

	sport->clk = clk_get(&pdev->dev, "uart");
	if (IS_ERR(sport->clk)) {
		ret = PTR_ERR(sport->clk);
		goto unmap;
	}
	clk_enable(sport->clk);

	sport->port.uartclk = clk_get_rate(sport->clk);

	mvf_ports[pdev->id] = sport;

	pdata = pdev->dev.platform_data;
	if (pdata && (pdata->flags & MVFUART_HAVE_RTSCTS))
		sport->have_rtscts = 1;
	if (pdata && (pdata->flags & MVFUART_USE_DCEDTE))
		sport->use_dcedte = 1;
	if (pdata && (pdata->flags & MVFUART_SDMA))
		sport->enable_dma = 1;

#ifdef CONFIG_IRDA
	if (pdata && (pdata->flags & MVFUART_IRDA))
		sport->use_irda = 1;
#endif

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


static void serial_mvf_remove(struct platform_device *dev)
{

	return;


static struct platform_driver serial_mvf_driver = {
	.probe    = serial_mvf_probe,
	.remove   = serial_mvf_remove,

	.driver   = {
		.name    = "mvf-uart",
		.owner   = THIS_MODULE,
#ifdef CONFIG_PM
		.pm      = &serial_mvf_pm_ops,
	},
};


static int __init mvf_serial_init(void)
{
	int ret;
	printk(KERN_INFO "Serial: Mvf driver\n");
	ret = uart_regiseter_driver(&mvf_reg);
	if ( ret )
		return ret;

	ret = platform_driver_register(&serial_mvf_driver);
	if ( ret != 0 )
		uart_unregister_driver(&mvf_reg);

	return 0;
}


static void __exit mvf_serial_exit(void)
{
	platform_driver_unregiseter(&serial_mvf_driver);
	uart_unregister_driver(&mvf_reg);
}


module_init(mvf_serial_init);
module_exit(mvf_serial_exit);

MODULES_AUTHER("");
MODULES_DESCRIPTION("Faradeay serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:");
