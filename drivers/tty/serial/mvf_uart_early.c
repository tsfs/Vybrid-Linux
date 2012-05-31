/*
 * based on drivers/tty/serial/mxc_uart_early.c
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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
 * @file drivers/serial/mvf_uart_early.c
 *
 * @brief Driver for the Freescale Semiconductor MVF serial ports based on
 * drivers/char/8250_early.c,
 * Copyright 2004 Hewlett-Packard Development Company,
 * L.P.by Bjorn Helgaasby.
 *
 * Early serial console for MVF UARTS.
 *
 * This is for use before the serial driver has initialized, in
 * particular, before the UARTs have been discovered and named.
 * Instead of specifying the console device as, e.g., "ttymvf0",
 * we locate the device directly by its MMIO or I/O port address.
 *
 * The user can specify the device directly, e.g.,
 *	console=mvfuart,0x43f90000,115200n8
 * or platform code can call early_uart_console_init() to set
 * the early UART device.
 *
 * After the normal serial driver starts, we try to locate the
 * matching ttymvf device and start a console there.
 */

/*
 * Include Files
 */

#include <linux/tty.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#include <mach/mvf_uart.h>

struct mvf_early_uart_device {
	struct uart_port port;
	char options[16];	/* e.g., 115200n8 */
	unsigned int baud;
	struct clk *clk;
};
static struct mvf_early_uart_device mvf_early_device __initdata;

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
 * Write out a character once the UART is ready
 */
static void __init mvfuart_console_write_char(struct uart_port *port, int ch)
{
	unsigned int status;

	while (readb(sport->port.membase + S1) & UART_S1_TDRE)
		barrier();
	
	writeb(ch, sport->port.membase + UART_D);

}

/*!
 * This function is called to write the console messages through the UART port.
 *
 * @param   co    the console structure
 * @param   s     the log message to be written to the UART
 * @param   count length of the message
 */
void __init early_mvfuart_console_write(struct console *co, const char *s,
					u_int count)
{
	struct uart_port *port = &mvf_early_device.port;
	unsigned int status, oldc1, oldc2, oldc3, c1, c2, c3;

	/*
	 * First save the control registers and then disable the interrupts
	 */
	oldc1 = readb(port->membase + UART_C1);
	oldc2 = readb(port->membase + UART_C2);
	oldc3 = readb(port->membase + UART_C3);

	c2 = oldc2 & ~(UART_C2_TE | UART_C2_RE 
				   | UART_C2_RIE | UART_C2_RIE);

	writeb(c2, port->membase + MVF_UARTUCR2);

	/* Transmit string */
	uart_console_write(port, s, count, mvfuart_console_write_char);

	/*
	 * Finally, wait for the transmitter to become empty
	 */
	do {
		status = readb(port->membase + UART_S1);
	} while (!(status & UART_S1_TDRE));

	/*
	 * Restore the control registers
	 */
	writeb(oldc2, port->membase + UART_C2);

}

static unsigned int __init probe_baud(struct uart_port *port)
{
	/* FIXME Return Default Baud Rate */
	return 115200;
}

static int __init mvf_early_uart_setup(struct console *console, char *options)
{
	struct mvf_early_uart_device *device = &mvf_early_device;
	struct uart_port *port = &device->port;
	int length;

	if (device->port.membase || device->port.iobase)
		return -ENODEV;

	/* Enable Early MVF UART Clock */
	clk_enable(device->clk);

	port->uartclk = 50000000;
	port->iotype = UPIO_MEM;
	port->membase = ioremap(port->mapbase, SZ_4K);

	if (options) {
		device->baud = simple_strtoul(options, NULL, 0);
		length = min(strlen(options), sizeof(device->options));
		strncpy(device->options, options, length);
	} else {
		device->baud = probe_baud(port);
		snprintf(device->options, sizeof(device->options), "%u",
			 device->baud);
	}
	printk(KERN_INFO
	       "MVF_Early serial console at MMIO 0x%x (options '%s')\n",
	       port->mapbase, device->options);
	return 0;
}

static struct console mvf_early_uart_console __initdata = {
	.name = "ttymvf",
	.write = early_mvfuart_console_write,
	.setup = mvf_early_uart_setup,
	.flags = CON_PRINTBUFFER | CON_BOOT,
	.index = -1,
};

int __init mvf_early_serial_console_init(unsigned long base, struct clk *clk)
{
	mvf_early_device.clk = clk;
	mvf_early_device.port.mapbase = base;

	register_console(&mvf_early_uart_console);
	return 0;
}

int __init mvf_early_uart_console_disable(void)
{
	struct mvf_early_uart_device *device = &mvf_early_device;
	struct uart_port *port = &device->port;

	if (mvf_early_uart_console.index >= 0) {
		iounmap(port->membase);
		clk_disable(device->clk);
		clk_put(device->clk);
	}
	return 0;
}
late_initcall(mvf_early_uart_console_disable);
