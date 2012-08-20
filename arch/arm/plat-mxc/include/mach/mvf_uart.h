/*
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
 * @defgroup UART Universal Asynchronous Receiver Transmitter (UART) Driver
 */

/*!
 * @file arch-mxc/mxc_uart.h
 *
 * @brief This file contains the UART configuration structure definition.
 *
 *
 * @ingroup UART
 */

#ifndef __ASM_ARCH_MVF_UART_H__
#define __ASM_ARCH_MVF_UART_H__

#ifdef __KERNEL__

#include <linux/serial_core.h>

#if 0
typedef struct {
	struct uart_port port;
	int 
} uart_mvf_port;
#endif
struct mvfuart_platform_data {
	int (*init)(struct platform_device *pdev);
	void (*exit)(struct platform_device *pdev);
	unsigned int flags;
	void (*irda_enable)(int enable);
	unsigned int irda_inv_rx:1;
	unsigned int irda_inv_tx:1;
	unsigned short transceiver_delay;
	unsigned int dma_req_rx;
	unsigned int dma_req_tx;
};

#define MVFUART_HAVE_RTSCTS	(1<<0)
#define MVFUART_IRDA		(1<<1)
#define MVFUART_USE_DCEDTE	(1<<2)
#define MVFUART_SDMA		(1<<3)
#define MVFUART_MSB_FIRST	(1<<4)

#define MVF_UART_BDH     0x00   /* UART Bard Rate Registers:High 	   */
#define MVF_UART_BDL     0x01   /* UART Baud Rate Registers:Low  	   */
#define MVF_UART_C1      0x02   /* UART Control Register 1       	   */
#define MVF_UART_C2      0x03   /* UART Control Register 2       	   */
#define MVF_UART_S1      0x04   /* UART Status Register 1        	   */
#define MVF_UART_S2      0x05   /* UART Status Register 2        	   */
#define MVF_UART_C3      0x06   /* UART Control Register 3       	   */
#define MVF_UART_D       0x07   /* UART Data Register            	   */
#define MVF_UART_MA1     0x08   /* UART Match Address Register 1 	   */
#define MVF_UART_MA2     0x09   /* UART Match Address Register 2 	   */
#define MVF_UART_C4      0x0A   /* UART Control Register 4       	   */
#define MVF_UART_C5      0x0B   /* UART Control Register 5       	   */
#define MVF_UART_ED      0x0C   /* UART Extend Data Register     	   */
#define MVF_UART_MODEM   0x0D   /* UART Modem Register           	   */
#define MVF_UART_IR      0x0E   /* UART Infrared Register        	   */
#define MVF_UART_PFIFO   0x10   /* UART FIFO Parameters          	   */
#define MVF_UART_CFIFO   0x11   /* UART FIFO Control Register    	   */
#define MVF_UART_SFIFO   0x12   /* UART FIFO Status Register     	   */
#define MVF_UART_TWFIFO  0x13   /* UART FIFO Transmit Watermark  	   */
#define MVF_UART_TCFIFO  0x14   /* UART FIFO Transmit Count      	   */
#define MVF_UART_RWFIFO  0x15   /* UART FIFO Recive Watermark    	   */
#define MVF_UART_RCFIFO  0x16   /* UART FIFO Recive Count              */
#define MVF_UART_C7816   0x18   /* UART 7816 Control Register          */
#define MVF_UART_IE7616  0x19   /* UART 7816 Interrupt Enable Register */
#define MVF_UART_IS7816  0x1A   /* UART 7816 Interrupt Status Register */
#define MVF_UART_WP7816T0  0x1B /* UART 7816 Wait Paramater Register   */
#define MVF_UART_WP7816T1  0x1B /* UART 7816 Wait Paramater Register   */
#define MVF_UART_WN7816  0x1C   /* UART 7816 Wait N Register           */
#define MVF_UART_WF7816  0x1D   /* UART 7816 Wait FD Register          */
#define MVF_UART_ET7816  0x1E   /* UART 7816 Error Threshold Register  */
#define MVF_UART_TL7816  0x1F   /* UART 7816 Transmit Length Register  */
#define MVF_UART_C6      0x21   /* CEA709,1-B Control Register         */
#define MVF_UART_PCTH    0x22   /* CEA709,1-B Pachet Cycle Time Cnt H  */
#define MVF_UART_PCLT    0x23   /* CEA709,1-B Packet Cycle Time Cnt L  */
#define MVF_UART_IE0     0x24   /* CEA709,1-B Interrupt Enable Register*/
#define MVF_UART_SDTH    0x25   /* CEA709,1-B Secondary Delay Time Hi  */
#define MVF_UART_SDTL    0x26   /* CEA709,1-B Secondary Delay Time Lo  */
#define MVF_UART_PRE     0x27   /* CEA709,1-B Preamble                 */
#define MVF_UART_TPL     0x28   /* CEA709,1-B Transmit Packet Length   */
#define MVF_UART_IE      0x29   /* CEA709,1-B Interrupt Enable Register*/
#define MVF_UART_S3      0x2B   /* CEA709,1-B Status Register          */
#define MVF_UART_S4      0x2C   /* CEA709,1-B Status Register          */
#define MVF_UART_RPL     0x2D   /* CEA709,1-B Received Packet length   */
#define MVF_UART_RPREL   0x2E   /* CEA709,1-B Received Preamble Length */
#define MVF_UART_CPW     0x2F   /* CEA709,1-B Collision Pulse Width    */
#define MVF_UART_RIDTH   0x30   /* CEA709,1-B Receive Indet. Time Hi   */
#define MVF_UART_RIDTL   0x31   /* CEA709,1-B Receive Indet. Time Lo   */
#define MVF_UART_TIDTH   0x32   /* CEA709,1-B Transmit Indet. Time Hi  */
#define MVF_UART_TIDTL   0x33   /* CEA709,1-B Transmit Indet. TIme Lo  */
#define MVF_UART_RB1TH   0x34   /* CEA709,1-B Recive Beta1 Time High   */
#define MVF_UART_RB1TL   0x35   /* CEA709,1-B Recive Bata1 Time Low    */
#define MVF_UART_TB1TH   0x36   /* CEA709,1-B Transmit Bata1 Time High */
#define MVF_UART_TB1TL   0x37   /* CEA709,1-B Transmit Bata1 Time Low  */
#define MVF_UART_PROG_REG  0x38 /* CEA709,1-B Programmable register    */
#define MVF_UART_STATE_REG 0x39 /* CEA709,1-B Status register          */

#define UART_C1_LOOPS  (1<<7)
#define UART_C1_RSRC   (1<<5)
#define UART_C1_M      (1<<4)
#define UART_C1_WAKE   (1<<3)
#define UART_C1_ILT    (1<<2)
#define UART_C1_PE     (1<<1)
#define UART_C1_PT     (1<<0)

#define UART_C2_TIE    (1<<7)
#define UART_C2_TCIE   (1<<6)
#define UART_C2_RIE    (1<<5)
#define UART_C2_ILIE   (1<<4)
#define UART_C2_TE     (1<<3)
#define UART_C2_RE     (1<<2)
#define UART_C2_RWU    (1<<1)
#define UART_C2_SBK    (1<<0)

#define UART_C3_T8     (1<<6)
#define UART_C3_TXDIR  (1<<5)
#define UART_C3_TXINV  (1<<4)
#define UART_C3_ORIE   (1<<3)
#define UART_C3_NEIE   (1<<2)
#define UART_C3_FEIE   (1<<1)
#define UART_C3_PEIE   (1<<0)

#define UART_S1_TDRE   (1<<7)
#define UART_S1_TC     (1<<6)
#define UART_S1_RDRF   (1<<5)
#define UART_S1_IDLE   (1<<4)
#define UART_S1_OR     (1<<3)
#define UART_S1_NF     (1<<2)
#define UART_S1_FE     (1<<1)
#define UART_S1_PF     (1<<0)

#define UART_S2_LBKDIF  (1<<7)
#define UART_S2_RXEDGIF (1<<6)
#define UART_S2_MSBF    (1<<5)
#define UART_S2_RXINV   (1<<4)
#define UART_S2_RWUID   (1<<3)
#define UART_S2_BRK13   (1<<2)
#define UART_S2_LBKDE   (1<<1)
#define UART_S2_RAF     (1<<0)

#define UART_C5_TDMAS   (1<<7)
#define UART_C5_RDMAS   (1<<5)

#define UART_MODEM_RXRTSE	(1<<3)
#define UART_MODEM_TXRTSPOL	(1<<2)
#define UART_MODEM_TXRTSE	(1<<1)
#define UART_MODEM_TXCTSE	(1<<0)

#define UART_PFIFO_TXFE   (1<<7)
#define UART_PFOFO_RXFE   (1<<3)

#define UART_SFIFO_TXEMPT (1<<7)
#define UART_SFIFO_RXEMPT (1<<6)
#define UART_SFIFO_RXOF   (1<<2)
#define UART_SFIFO_TXOF   (1<<1)
#define UART_SFIFO_RXUF   (1<<0)
#endif
#endif
