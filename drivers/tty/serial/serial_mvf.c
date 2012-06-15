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

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/mvf_uart.h>

#define SERIAL_MVF_MAJOR        207
#define MINOR_START	        16
#define DRIVER_NAME  "MVF-uart"
#define DEV_NAME	 "ttyvmvf"

#define UART_NR   6

#define MCTRL_TIMEOUT	(250*HZ/1000)

struct mvf_port {
	struct uart_port   port;
	struct timer_list   timer;
	unsigned int       old_status;
	int          txirq,rxirq,rtsirq;
	struct clk		*clk;
	unsigned short txfifo;
	unsigned short rxfifo;

};

#ifdef CONFIG_IRDA
# Error IRDA not implemented yet.
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
}
#endif

static inline int mvf_set_bps(struct mvf_port *sport,
								  unsigned long base,unsigned long bps)
{
	unsigned char bdh,bdl;
	unsigned long sbr;
	sbr = base/(16*bps);
	bdh = readb(sport->port.membase + MVF_UART_BDH) & 0xc0;
	bdh = (sbr>>8) & 0x1f;
	bdl = (sbr&0xff);
	writeb(bdh, sport->port.membase + MVF_UART_BDH);
	writeb(bdl, sport->port.membase + MVF_UART_BDL);
	return 0;
}

static inline int mvf_uart_enable(struct mvf_port *sport)
{
	unsigned char c2;
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 |= (UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
	return 0;
}
static inline int mvf_uart_disable(struct mvf_port *sport)
{
	unsigned char c2;
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 &= ~(UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
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

	c2 = readb(sport->port.membase + MVF_UART_C2);
	writeb(c2 & ~UART_C2_TE, sport->port.membase + MVF_UART_C2);
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
	c2 = readb(sport->port.membase + MVF_UART_C2);
	writeb(c2 &~UART_C2_RE, sport->port.membase + MVF_UART_C2);
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
		   !(readb(sport->port.membase + MVF_UART_TCFIFO) < sport->txfifo)){
		/* out the port here */
		writeb(xmit->buf[xmit->tail], sport->port.membase + MVF_UART_D);
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

	if (readb(sport->port.membase + MVF_UART_SFIFO) & UART_SFIFO_TXEMPT)
		mvf_transmit_buffer(sport);
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
	unsigned char sfifo,s1;

	spin_lock_irqsave(&sport->port.lock,flags);

	while (!(readb(sport->port.membase + MVF_UART_SFIFO) 
			 & UART_SFIFO_RXEMPT)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		rx = (unsigned int)readb(sport->port.membase + MVF_UART_D);
		
		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		sfifo = readb(sport->port.membase + MVF_UART_SFIFO);
		s1 = readb(sport->port.membase + MVF_UART_S1);
		if ((sfifo & (UART_SFIFO_RXOF | UART_SFIFO_RXUF))
			|| (s1 & (UART_S1_PF | UART_S1_FE))) {
			if (s1 & UART_S1_PF)
				sport->port.icount.parity++;
			else if (s1 & UART_S1_FE)
				sport->port.icount.frame++;
			if (sfifo& UART_SFIFO_RXOF)
				sport->port.icount.overrun++;

			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}
			
			if ( s1 & UART_S1_PF )
				flg = TTY_PARITY;
			else if (s1 & UART_S1_FE)
				flg = TTY_FRAME;
			if ( sfifo & UART_SFIFO_RXOF)
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
	unsigned char s1,s2;

	s1 = readb(sport->port.membase + MVF_UART_S1);
	s2 = readb(sport->port.membase + MVF_UART_S2);

	/* Check RXINT */
	if (s1 & UART_S1_RDRF) {
		mvf_rxint(irq, dev_id);
	}

	if (s1 & UART_S1_TDRE)
		mvf_txint(irq, dev_id);


	return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int mvf_tx_empty(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;

	return (readb(sport->port.membase + MVF_UART_S1) 
			& UART_S1_TDRE)?TIOCSER_TEMT : 0;
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int mvf_get_mctrl(struct uart_port *port)
{
	unsigned int tmp = TIOCM_DSR | TIOCM_CAR;
	/*
	 * TBD
	 */
	return tmp;
}

static void mvf_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/*
	 * TBD
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

#define TXTL 2 /* reset default */
#define RXTL 1 /* reset default */

#define GET_FSIZE(x)  ((x)==0?1:1<<((x)+1))

static int mvf_startup(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	int retval;
	unsigned long flags;
	struct tty_struct *tty;
	unsigned char c2,fifo;

	/*
	 * Disable Interrupts.
	 */
	writeb(0xff,sport->port.membase + MVF_UART_S2); /* Clear All Interrupts */

	/*
	 * Allocate the IRQ(s) i.MX1 has three interrupts whereas later
	 * chips only have one interrupt.
	 */
	retval = request_irq(sport->port.irq, mvf_int, 0,
						 DRIVER_NAME, sport);
	if (retval) {
		free_irq(sport->port.irq, sport);
		goto error_out1;
	}



	spin_lock_irqsave(&sport->port.lock, flags);
	/*
	 * Finally, clear and enable interrupts
	 */

	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 |= (UART_C2_RIE | UART_C2_TIE | UART_C2_TE | UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);

	

	mvf_enable_ms(&sport->port);
	spin_unlock_irqrestore(&sport->port.lock,flags);

	/* SET FIFO Size */
	fifo = readb(sport->port.membase + MVF_UART_PFIFO);
	sport->txfifo = GET_FSIZE((fifo>>4)&0x07);
	sport->rxfifo = GET_FSIZE((fifo)&0x07);


	tty = sport->port.state->port.tty;

	return 0;

error_out1:
	return retval;
}

static void mvf_shutdown(struct uart_port *port)
{
	struct mvf_port *sport = (struct mvf_port *)port;
	unsigned long flags;
	unsigned char c2;

	spin_lock_irqsave(&sport->port.lock, flags);
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c2 &= ~(UART_C2_RE);
	writeb(c2, sport->port.membase + MVF_UART_C2);
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
	c2 = readb(sport->port.membase + MVF_UART_C2);
#if 0
	c2 &= ~(UART_C2_TIE | UART_C2_TCIE | UART_C2_RIE 
			| UART_C2_ILIE | UART_C2_TE | UART_C2_RE
			| UART_C2_RWU | 1);
#endif
	c2 = 0;
	writeb(c2, sport->port.membase + MVF_UART_C2);

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
	 * Release 1: No support Mode control lines.
	 */
	if ( 1 ) {
		termios->c_cflag &= ~(HUPCL | CRTSCTS | CMSPAR);
		termios->c_cflag |= CLOCAL;
	}

	/*
	 * We only support CS8.
	 */
	termios->c_cflag &=~CSIZE;
	termios->c_cflag |= CS8;
	
	c1 &=~UART_C1_M;
	c2 = readb(sport->port.membase + MVF_UART_C2);
	c3 = readb(sport->port.membase + MVF_UART_C3);

	
	if (termios->c_cflag & PARENB){
		c1 |= UART_C1_PE;
		if (termios->c_cflag & PARODD)
			c1 |= UART_C1_PT;
		else
			c1 &= ~UART_C1_PT;
	}else
		c1 &= ~(UART_C1_PE | UART_C1_PT);
	
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

	spin_lock_irqsave(&sport->port.lock, flags);
	
	
	writeb(c1,sport->port.membase + MVF_UART_C1);
	writeb(c2,sport->port.membase + MVF_UART_C2);
	writeb(c3,sport->port.membase + MVF_UART_C3);

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
	
	while (!(readb(sport->port.membase + MVF_UART_S1) & UART_S1_TDRE))
		barrier();
	
	writeb(ch, sport->port.membase + MVF_UART_D);
}

static void
mvf_console_write(struct console *co, const char *s, unsigned int count)
{
	struct mvf_port *sport = mvf_ports[co->index];
	uart_console_write(&sport->port, s, count, mvf_console_putchar);
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
		unsigned char c1;
		unsigned short br;
		c1 = readb(sport->port.membase + MVF_UART_C1);
		*parity = 'n';
		if ( c1 & UART_C1_PE)
			*parity = (c1 & UART_C1_PT)?'o':'e';
		*bits = 7;
		br = readb(sport->port.membase + MVF_UART_BDH)<<8
			| readb(sport->port.membase + MVF_UART_BDH);
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
#endif

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

static consit struct dev_pm_ops serial_mvf_pm_ops = {
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
	
	sport->port.fifosize = 32;
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


	pdata = pdev->dev.platform_data;
#if 0
	if (pdata && (pdata->flags & MVFUART_HAVE_RTSCTS))
		sport->have_rtscts = 1;
	if (pdata && (pdata->flags & MVFUART_USE_DCEDTE))
		sport->use_dcedte = 1;
	if (pdata && (pdata->flags & MVFUART_SDMA))
		sport->enable_dma = 1;
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
	printk(KERN_INFO "Serial: Mvf driver\n");
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
MODULE_DESCRIPTION("Faradeay serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:");

