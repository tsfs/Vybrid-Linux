/*
 *		mvfswitch -- L2 Switch Controller for mvf SoC
 *		   processors.
 *
 *      Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 2 of the License, or (at
 *     your option) any later version.
 *
 */
#ifndef SWITCH_H
#define	SWITCH_H

/*
 * The Switch stores dest/src/type, data, and checksum for receive packets.
 */
#define PKT_MAXBUF_SIZE         		1518
#define PKT_MINBUF_SIZE         		64
#define PKT_MAXBLR_SIZE         		1520

/*
 * The 5441x RX control register also contains maximum frame
 * size bits.
 */
#define OPT_FRAME_SIZE  				(PKT_MAXBUF_SIZE << 16)

/*
 * Some hardware gets it MAC address out of local flash memory.
 * if this is non-zero then assume it is the address to get MAC from.
 */
#define FEC_FLASHMAC    				0

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it it best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#ifdef CONFIG_SWITCH_DMA_USE_SRAM
#define SWITCH_ENET_RX_PAGES       		6
#else
#define SWITCH_ENET_RX_PAGES       		8
#endif

#define SWITCH_ENET_RX_FRSIZE      		2048
#define SWITCH_ENET_RX_FRPPG       		( PAGE_SIZE / SWITCH_ENET_RX_FRSIZE )
#define RX_RING_SIZE            		(SWITCH_ENET_RX_FRPPG * SWITCH_ENET_RX_PAGES)
#define SWITCH_ENET_TX_FRSIZE      		2048
#define SWITCH_ENET_TX_FRPPG       		(PAGE_SIZE / SWITCH_ENET_TX_FRSIZE)

#ifdef CONFIG_SWITCH_DMA_USE_SRAM
#define TX_RING_SIZE            		8      /* Must be power of two */
#define TX_RING_MOD_MASK        		7      /*   for this to work */
#else
#define TX_RING_SIZE            		16      /* Must be power of two */
#define TX_RING_MOD_MASK        		15      /*   for this to work */
#endif

#if (((RX_RING_SIZE + TX_RING_SIZE) * 8) > PAGE_SIZE)
#error "L2SWITCH: descriptor ring size constants too large"
#endif

/*unsigned long MCF_ESW_LOOKUP_MEM;*/
#if 0
#define MCF_ESW_REVISION   (*(volatile unsigned long *)(0xFC0DC000))
#define MCF_ESW_PER        (*(volatile unsigned long *)(0xFC0DC008))
#define MCF_ESW_VLANV      (*(volatile unsigned long *)(0xFC0DC010))
#define MCF_ESW_DBCR       (*(volatile unsigned long *)(0xFC0DC014))
#define MCF_ESW_DMCR       (*(volatile unsigned long *)(0xFC0DC018))
#define MCF_ESW_BKLR       (*(volatile unsigned long *)(0xFC0DC01C))
#define MCF_ESW_BMPC       (*(volatile unsigned long *)(0xFC0DC020))
#define MCF_ESW_MODE       (*(volatile unsigned long *)(0xFC0DC024))

#define MCF_ESW_ISR        (*(volatile unsigned long *)(0xFC0DC400))
#define MCF_ESW_IMR        (*(volatile unsigned long *)(0xFC0DC404))
#define MCF_ESW_TDAR       (*(volatile unsigned long *)(0xFC0DC418))
#define MCF_ESW_LOOKUP_MEM (*(volatile unsigned long *)(0xFC0E0000))

#define MCF_PPMCR0   (*(volatile unsigned short *)(0xFC04002D))
#define MCF_PPMHR0   (*(volatile unsigned long *)(0xFC040030))
#endif

#if 0
//	for compile
#define MCF_FEC_EIR0       (*(volatile unsigned long *)(0xFC0D4004))
#define MCF_FEC_EIR1       (*(volatile unsigned long *)(0xFC0D8004))
#define MCF_FEC_EIMR0      (*(volatile unsigned long *)(0xFC0D4008))
#define MCF_FEC_EIMR1      (*(volatile unsigned long *)(0xFC0D8008))
#define MCF_FEC_MMFR0      (*(volatile unsigned long *)(0xFC0D4040))
#define MCF_FEC_MMFR1      (*(volatile unsigned long *)(0xFC0D8040))
#define MCF_FEC_MSCR0      (*(volatile unsigned long *)(0xFC0D4044))
#define MCF_FEC_MSCR1      (*(volatile unsigned long *)(0xFC0D8044))
#define MCF_FEC_RCR0       (*(volatile unsigned long *)(0xFC0D4084))
#define MCF_FEC_RCR1       (*(volatile unsigned long *)(0xFC0D8084))
#define MCF_FEC_TCR0       (*(volatile unsigned long *)(0xFC0D40C4))
#define MCF_FEC_TCR1       (*(volatile unsigned long *)(0xFC0D80C4))
#define MCF_FEC_ECR0       (*(volatile unsigned long *)(0xFC0D4024))
#define MCF_FEC_ECR1       (*(volatile unsigned long *)(0xFC0D8024))
#else
//	from fec.h
//	#define FEC_R_CNTRL		0x084 /* Receive control reg */
//	#define FEC_X_CNTRL		0x0c4 /* Transmit Control reg */
//	#define FEC_IEVENT		0x004 /* Interrupt event reg */
//	#define FEC_IMASK		0x008 
//	#define FEC_MII_DATA		0x040 /* MII manage frame reg */
//	#define FEC_MII_SPEED		0x044 /* MII speed control reg */
//	#define FEC_ECNTRL		0x024 /* Ethernet control reg */

#endif

#define MCF_FEC_RCR_PROM                (0x00000008)
#define MCF_FEC_RCR_RMII_MODE           (0x00000100)
#define MCF_FEC_RCR_MAX_FL(x)           (((x)&0x00003FFF)<<16)
#define MCF_FEC_RCR_CRC_FWD             (0x00004000)
#define MCF_FEC_TCR_FDEN                (0x00000004)
#define MCF_FEC_ECR_ETHER_EN            (0x00000002)
#define MCF_FEC_ECR_ENA_1588            (0x00000010)


/*=============================================================*/
#define LEARNING_AGING_TIMER (10 * HZ)

/******************************************************************************/
/* Recieve is empty		*/
#define BD_SC_EMPTY     				((unsigned short)0x8000)

/* Transmit is ready	*/
#define BD_SC_READY     				((unsigned short)0x8000)

/* Last buffer descriptor */
#define BD_SC_WRAP      				((unsigned short)0x2000)

/* Interrupt on change */
#define BD_SC_INTRPT    				((unsigned short)0x1000)

/* Continous mode */
#define BD_SC_CM						((unsigned short)0x0200)

/* Rec'd too many idles */
#define BD_SC_ID						((unsigned short)0x0100)

/* xmt preamble */
#define BD_SC_P         				((unsigned short)0x0100)

/* Break received */
#define BD_SC_BR						((unsigned short)0x0020)

/* Framing error */
#define BD_SC_FR						((unsigned short)0x0010)

/* Parity error */
#define BD_SC_PR						((unsigned short)0x0008)

/* Overrun */
#define BD_SC_OV						((unsigned short)0x0002)
#define BD_SC_CD						((unsigned short)0x0001)


/*
 *	Buffer descriptor control/status used by Ethernet receive.
 */
#define BD_ENET_RX_EMPTY				((unsigned short)0x8000)
#define BD_ENET_RX_WRAP					((unsigned short)0x2000)
#define BD_ENET_RX_INTR					((unsigned short)0x1000)
#define BD_ENET_RX_LAST					((unsigned short)0x0800)
#define BD_ENET_RX_FIRST				((unsigned short)0x0400)
#define BD_ENET_RX_MISS					((unsigned short)0x0100)
#define BD_ENET_RX_LG					((unsigned short)0x0020)
#define BD_ENET_RX_NO					((unsigned short)0x0010)
#define BD_ENET_RX_SH					((unsigned short)0x0008)
#define BD_ENET_RX_CR					((unsigned short)0x0004)
#define BD_ENET_RX_OV					((unsigned short)0x0002)
#define BD_ENET_RX_CL					((unsigned short)0x0001)
/* All status bits */
#define BD_ENET_RX_STATS				((unsigned short)0x013f)


/* 
 *Buffer descriptor control/status used by Ethernet transmit.
 */
#define BD_ENET_TX_READY        ((unsigned short)0x8000)
#define BD_ENET_TX_PAD          ((unsigned short)0x4000)
#define BD_ENET_TX_WRAP         ((unsigned short)0x2000)
#define BD_ENET_TX_INTR         ((unsigned short)0x1000)
#define BD_ENET_TX_LAST         ((unsigned short)0x0800)
#define BD_ENET_TX_TC           ((unsigned short)0x0400)
#define BD_ENET_TX_DEF          ((unsigned short)0x0200)
#define BD_ENET_TX_HB           ((unsigned short)0x0100)
#define BD_ENET_TX_LC           ((unsigned short)0x0080)
#define BD_ENET_TX_RL           ((unsigned short)0x0040)
#define BD_ENET_TX_RCMASK       ((unsigned short)0x003c)
#define BD_ENET_TX_UN           ((unsigned short)0x0002)
#define BD_ENET_TX_CSL          ((unsigned short)0x0001)
/* All status bits */
#define BD_ENET_TX_STATS        ((unsigned short)0x03ff)

/*Copy from validation code */
#define RX_BUFFER_SIZE 1520
#define TX_BUFFER_SIZE 1520
#define NUM_RXBDS 20
#define NUM_TXBDS 20

#define TX_BD_R                 0x8000
#define TX_BD_TO1               0x4000
#define TX_BD_W                 0x2000
#define TX_BD_TO2               0x1000
#define TX_BD_L                 0x0800
#define TX_BD_TC                0x0400

#define TX_BD_INT       0x40000000
#define TX_BD_TS        0x20000000
#define TX_BD_PINS      0x10000000
#define TX_BD_IINS      0x08000000
#define TX_BD_TXE       0x00008000
#define TX_BD_UE        0x00002000
#define TX_BD_EE        0x00001000
#define TX_BD_FE        0x00000800
#define TX_BD_LCE       0x00000400
#define TX_BD_OE        0x00000200
#define TX_BD_TSE       0x00000100
#define TX_BD_BDU       0x80000000

#define RX_BD_E                 0x8000
#define RX_BD_R01               0x4000
#define RX_BD_W                 0x2000
#define RX_BD_R02               0x1000
#define RX_BD_L                 0x0800
#define RX_BD_M                 0x0100
#define RX_BD_BC                0x0080
#define RX_BD_MC                0x0040
#define RX_BD_LG                0x0020
#define RX_BD_NO                0x0010
#define RX_BD_CR                0x0004
#define RX_BD_OV                0x0002
#define RX_BD_TR                0x0001

#define RX_BD_ME               0x80000000
#define RX_BD_PE               0x04000000
#define RX_BD_CE               0x02000000
#define RX_BD_UC               0x01000000
#define RX_BD_INT              0x00800000
#define RX_BD_ICE              0x00000020
#define RX_BD_PCR              0x00000010
#define RX_BD_VLAN             0x00000004
#define RX_BD_IPV6             0x00000002
#define RX_BD_FRAG             0x00000001
#define RX_BD_BDU              0x80000000
/****************************************************************************/

/* Address Table size in bytes(2048 64bit entry ) */
#define ESW_ATABLE_MEM_SIZE         (2048*8)
/* How many 64-bit elements fit in the address table */
#define ESW_ATABLE_MEM_NUM_ENTRIES  (2048)
/* Address Table Maximum number of entries in each Slot */
#define ATABLE_ENTRY_PER_SLOT 8
/* log2(ATABLE_ENTRY_PER_SLOT)*/
#define ATABLE_ENTRY_PER_SLOT_bits 3
/* entry size in byte */
#define ATABLE_ENTRY_SIZE     8
/*  slot size in byte */
#define ATABLE_SLOT_SIZE    (ATABLE_ENTRY_PER_SLOT * ATABLE_ENTRY_SIZE)
/* width of timestamp variable (bits) within address table entry */
#define AT_DENTRY_TIMESTAMP_WIDTH    10
/* number of bits for port number storage */
#define AT_DENTRY_PORT_WIDTH     4
/* number of bits for port bitmask number storage */
#define AT_SENTRY_PORT_WIDTH     7
/* address table static entry port bitmask start address bit */
#define AT_SENTRY_PORTMASK_shift     21
/* number of bits for port priority storage */
#define AT_SENTRY_PRIO_WIDTH	7
/* address table static entry priority start address bit */
#define AT_SENTRY_PRIO_shift     18
/* address table dynamic entry port start address bit */
#define AT_DENTRY_PORT_shift     28
/* address table dynamic entry timestamp start address bit */
#define AT_DENTRY_TIME_shift     18
/* address table entry record type start address bit */
#define AT_ENTRY_TYPE_shift     17
/* address table entry record type bit: 1 static, 0 dynamic */
#define AT_ENTRY_TYPE_STATIC      1
#define AT_ENTRY_TYPE_DYNAMIC     0
/* address table entry record valid start address bit */
#define AT_ENTRY_VALID_shift     16
#define AT_ENTRY_RECORD_VALID     1

#define AT_EXTRACT_VALID(x)   \
	((x >> AT_ENTRY_VALID_shift) & AT_ENTRY_RECORD_VALID)

#define AT_EXTRACT_PORTMASK(x)  \
	((x >> AT_SENTRY_PORTMASK_shift) & AT_SENTRY_PORT_WIDTH)

#define AT_EXTRACT_PRIO(x)  \
	((x >> AT_SENTRY_PRIO_shift) & AT_SENTRY_PRIO_WIDTH)

/* return block corresponding to the 8 bit hash value calculated */
#define GET_BLOCK_PTR(hash)  (hash << 3)
#define AT_EXTRACT_TIMESTAMP(x) \
	((x >> AT_DENTRY_TIME_shift) & ((1 << AT_DENTRY_TIMESTAMP_WIDTH)-1))
#define AT_EXTRACT_PORT(x)   \
	((x >> AT_DENTRY_PORT_shift) & ((1 << AT_DENTRY_PORT_WIDTH)-1))
#define AT_SEXTRACT_PORT(x)  \
	((~((x >> AT_SENTRY_PORTMASK_shift) &  \
	   ((1 << AT_DENTRY_PORT_WIDTH)-1))) >> 1)
#define TIMEDELTA(newtime, oldtime) \
	 ((newtime - oldtime) & \
	  ((1 << AT_DENTRY_TIMESTAMP_WIDTH)-1))

#define AT_EXTRACT_IP_PROTOCOL(x) ((x >> 8) & 0xff)
#define AT_EXTRACT_TCP_UDP_PORT(x) ((x >> 16) & 0xffff)

/* increment time value respecting modulo. */
#define TIMEINCREMENT(time) \
	((time) = ((time)+1) & ((1 << AT_DENTRY_TIMESTAMP_WIDTH)-1))
/* ------------------------------------------------------------------------- */
/* Bit definitions and macros for MCF_ESW_REVISION */
#define MCF_ESW_REVISION_CORE_REVISION(x)      (((x)&0x0000FFFF)<<0)
#define MCF_ESW_REVISION_CUSTOMER_REVISION(x)  (((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for MCF_ESW_PER */
#define MCF_ESW_PER_TE0                        (0x00000001)
#define MCF_ESW_PER_TE1                        (0x00000002)
#define MCF_ESW_PER_TE2                        (0x00000004)
#define MCF_ESW_PER_RE0                        (0x00010000)
#define MCF_ESW_PER_RE1                        (0x00020000)
#define MCF_ESW_PER_RE2                        (0x00040000)

/* Bit definitions and macros for MCF_ESW_VLANV */
#define MCF_ESW_VLANV_VV0                      (0x00000001)
#define MCF_ESW_VLANV_VV1                      (0x00000002)
#define MCF_ESW_VLANV_VV2                      (0x00000004)
#define MCF_ESW_VLANV_DU0                      (0x00010000)
#define MCF_ESW_VLANV_DU1                      (0x00020000)
#define MCF_ESW_VLANV_DU2                      (0x00040000)

/* Bit definitions and macros for MCF_ESW_DBCR */
#define MCF_ESW_DBCR_P0                        (0x00000001)
#define MCF_ESW_DBCR_P1                        (0x00000002)
#define MCF_ESW_DBCR_P2                        (0x00000004)

/* Bit definitions and macros for MCF_ESW_DMCR */
#define MCF_ESW_DMCR_P0                        (0x00000001)
#define MCF_ESW_DMCR_P1                        (0x00000002)
#define MCF_ESW_DMCR_P2                        (0x00000004)

/* Bit definitions and macros for MCF_ESW_BKLR */
#define MCF_ESW_BKLR_BE0                       (0x00000001)
#define MCF_ESW_BKLR_BE1                       (0x00000002)
#define MCF_ESW_BKLR_BE2                       (0x00000004)
#define MCF_ESW_BKLR_LD0                       (0x00010000)
#define MCF_ESW_BKLR_LD1                       (0x00020000)
#define MCF_ESW_BKLR_LD2                       (0x00040000)

/* Bit definitions and macros for MCF_ESW_BMPC */
#define MCF_ESW_BMPC_PORT(x)                   (((x)&0x0000000F)<<0)
#define MCF_ESW_BMPC_MSG_TX                    (0x00000020)
#define MCF_ESW_BMPC_EN                        (0x00000040)
#define MCF_ESW_BMPC_DIS                       (0x00000080)
#define MCF_ESW_BMPC_PRIORITY(x)               (((x)&0x00000007)<<13)
#define MCF_ESW_BMPC_PORTMASK(x)               (((x)&0x00000007)<<16)

/* Bit definitions and macros for MCF_ESW_MODE */
#define MCF_ESW_MODE_SW_RST                    (0x00000001)
#define MCF_ESW_MODE_SW_EN                     (0x00000002)
#define MCF_ESW_MODE_STOP                      (0x00000080)
#define MCF_ESW_MODE_CRC_TRAN                  (0x00000100)
#define MCF_ESW_MODE_P0CT                      (0x00000200)
#define MCF_ESW_MODE_STATRST                   (0x80000000)

/* Bit definitions and macros for MCF_ESW_VIMSEL */
#define MCF_ESW_VIMSEL_IM0(x)                  (((x)&0x00000003)<<0)
#define MCF_ESW_VIMSEL_IM1(x)                  (((x)&0x00000003)<<2)
#define MCF_ESW_VIMSEL_IM2(x)                  (((x)&0x00000003)<<4)

/* Bit definitions and macros for MCF_ESW_VOMSEL */
#define MCF_ESW_VOMSEL_OM0(x)                  (((x)&0x00000003)<<0)
#define MCF_ESW_VOMSEL_OM1(x)                  (((x)&0x00000003)<<2)
#define MCF_ESW_VOMSEL_OM2(x)                  (((x)&0x00000003)<<4)

/* Bit definitions and macros for MCF_ESW_VIMEN */
#define MCF_ESW_VIMEN_EN0                      (0x00000001)
#define MCF_ESW_VIMEN_EN1                      (0x00000002)
#define MCF_ESW_VIMEN_EN2                      (0x00000004)

/* Bit definitions and macros for MCF_ESW_VID */
#define MCF_ESW_VID_TAG(x)                     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_MCR */
#define MCF_ESW_MCR_PORT(x)                    (((x)&0x0000000F)<<0)
#define MCF_ESW_MCR_MEN                        (0x00000010)
#define MCF_ESW_MCR_INGMAP                     (0x00000020)
#define MCF_ESW_MCR_EGMAP                      (0x00000040)
#define MCF_ESW_MCR_INGSA                      (0x00000080)
#define MCF_ESW_MCR_INGDA                      (0x00000100)
#define MCF_ESW_MCR_EGSA                       (0x00000200)
#define MCF_ESW_MCR_EGDA                       (0x00000400)

/* Bit definitions and macros for MCF_ESW_EGMAP */
#define MCF_ESW_EGMAP_EG0                      (0x00000001)
#define MCF_ESW_EGMAP_EG1                      (0x00000002)
#define MCF_ESW_EGMAP_EG2                      (0x00000004)

/* Bit definitions and macros for MCF_ESW_INGMAP */
#define MCF_ESW_INGMAP_ING0                    (0x00000001)
#define MCF_ESW_INGMAP_ING1                    (0x00000002)
#define MCF_ESW_INGMAP_ING2                    (0x00000004)

/* Bit definitions and macros for MCF_ESW_INGSAL */
#define MCF_ESW_INGSAL_ADDLOW(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_INGSAH */
#define MCF_ESW_INGSAH_ADDHIGH(x)              (((x)&0x0000FFFF)<<0)

/* Bit definitions and macros for MCF_ESW_INGDAL */
#define MCF_ESW_INGDAL_ADDLOW(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_INGDAH */
#define MCF_ESW_INGDAH_ADDHIGH(x)              (((x)&0x0000FFFF)<<0)

/* Bit definitions and macros for MCF_ESW_ENGSAL */
#define MCF_ESW_ENGSAL_ADDLOW(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_ENGSAH */
#define MCF_ESW_ENGSAH_ADDHIGH(x)              (((x)&0x0000FFFF)<<0)

/* Bit definitions and macros for MCF_ESW_ENGDAL */
#define MCF_ESW_ENGDAL_ADDLOW(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_ENGDAH */
#define MCF_ESW_ENGDAH_ADDHIGH(x)              (((x)&0x0000FFFF)<<0)

/* Bit definitions and macros for MCF_ESW_MCVAL */
#define MCF_ESW_MCVAL_COUNT(x)                 (((x)&0x000000FF)<<0)

/* Bit definitions and macros for MCF_ESW_MMSR */
#define MCF_ESW_MMSR_BUSY                      (0x00000001)
#define MCF_ESW_MMSR_NOCELL                    (0x00000002)
#define MCF_ESW_MMSR_MEMFULL                   (0x00000004)
#define MCF_ESW_MMSR_MFLATCH                   (0x00000008)
#define MCF_ESW_MMSR_DQ_GRNT                   (0x00000040)
#define MCF_ESW_MMSR_CELLS_AVAIL(x)            (((x)&0x000000FF)<<16)

/* Bit definitions and macros for MCF_ESW_LMT */
#define MCF_ESW_LMT_THRESH(x)                  (((x)&0x000000FF)<<0)

/* Bit definitions and macros for MCF_ESW_LFC */
#define MCF_ESW_LFC_COUNT(x)                   (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_PCSR */
#define MCF_ESW_PCSR_PC0                       (0x00000001)
#define MCF_ESW_PCSR_PC1                       (0x00000002)
#define MCF_ESW_PCSR_PC2                       (0x00000004)

/* Bit definitions and macros for MCF_ESW_IOSR */
#define MCF_ESW_IOSR_OR0                       (0x00000001)
#define MCF_ESW_IOSR_OR1                       (0x00000002)
#define MCF_ESW_IOSR_OR2                       (0x00000004)

/* Bit definitions and macros for MCF_ESW_QWT */
#define MCF_ESW_QWT_Q0WT(x)                    (((x)&0x0000001F)<<0)
#define MCF_ESW_QWT_Q1WT(x)                    (((x)&0x0000001F)<<8)
#define MCF_ESW_QWT_Q2WT(x)                    (((x)&0x0000001F)<<16)
#define MCF_ESW_QWT_Q3WT(x)                    (((x)&0x0000001F)<<24)

/* Bit definitions and macros for MCF_ESW_P0BCT */
#define MCF_ESW_P0BCT_THRESH(x)                (((x)&0x000000FF)<<0)

/* Bit definitions and macros for MCF_ESW_P0FFEN */
#define MCF_ESW_P0FFEN_FEN                     (0x00000001)
#define MCF_ESW_P0FFEN_FD(x)                   (((x)&0x00000003)<<2)

/* Bit definitions and macros for MCF_ESW_PSNP */
#define MCF_ESW_PSNP_EN                        (0x00000001)
#define MCF_ESW_PSNP_MODE(x)                   (((x)&0x00000003)<<1)
#define MCF_ESW_PSNP_CD                        (0x00000008)
#define MCF_ESW_PSNP_CS                        (0x00000010)
#define MCF_ESW_PSNP_PORT_COMPARE(x)           (((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for MCF_ESW_IPSNP */
#define MCF_ESW_IPSNP_EN                       (0x00000001)
#define MCF_ESW_IPSNP_MODE(x)                  (((x)&0x00000003)<<1)
#define MCF_ESW_IPSNP_PROTOCOL(x)              (((x)&0x000000FF)<<8)

/* Bit definitions and macros for MCF_ESW_PVRES */
#define MCF_ESW_PVRES_PRI0(x)                  (((x)&0x00000007)<<0)
#define MCF_ESW_PVRES_PRI1(x)                  (((x)&0x00000007)<<3)
#define MCF_ESW_PVRES_PRI2(x)                  (((x)&0x00000007)<<6)
#define MCF_ESW_PVRES_PRI3(x)                  (((x)&0x00000007)<<9)
#define MCF_ESW_PVRES_PRI4(x)                  (((x)&0x00000007)<<12)
#define MCF_ESW_PVRES_PRI5(x)                  (((x)&0x00000007)<<15)
#define MCF_ESW_PVRES_PRI6(x)                  (((x)&0x00000007)<<18)
#define MCF_ESW_PVRES_PRI7(x)                  (((x)&0x00000007)<<21)

/* Bit definitions and macros for MCF_ESW_IPRES */
#define MCF_ESW_IPRES_ADDRESS(x)               (((x)&0x000000FF)<<0)
#define MCF_ESW_IPRES_IPV4SEL                  (0x00000100)
#define MCF_ESW_IPRES_PRI0(x)                  (((x)&0x00000003)<<9)
#define MCF_ESW_IPRES_PRI1(x)                  (((x)&0x00000003)<<11)
#define MCF_ESW_IPRES_PRI2(x)                   (((x)&0x00000003)<<13)
#define MCF_ESW_IPRES_READ                     (0x80000000)

/* Bit definitions and macros for MCF_ESW_PRES */
#define MCF_ESW_PRES_VLAN                      (0x00000001)
#define MCF_ESW_PRES_IP                        (0x00000002)
#define MCF_ESW_PRES_MAC                       (0x00000004)
#define MCF_ESW_PRES_DFLT_PRI(x)               (((x)&0x00000007)<<4)

/* Bit definitions and macros for MCF_ESW_PID */
#define MCF_ESW_PID_VLANID(x)                  (((x)&0x0000FFFF)<<0)

/* Bit definitions and macros for MCF_ESW_VRES */
#define MCF_ESW_VRES_P0                        (0x00000001)
#define MCF_ESW_VRES_P1                        (0x00000002)
#define MCF_ESW_VRES_P2                        (0x00000004)
#define MCF_ESW_VRES_VLANID(x)                 (((x)&0x00000FFF)<<3)

/* Bit definitions and macros for MCF_ESW_DISCN */
#define MCF_ESW_DISCN_COUNT(x)                 (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_DISCB */
#define MCF_ESW_DISCB_COUNT(x)                 (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_NDISCN */
#define MCF_ESW_NDISCN_COUNT(x)                (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_NDISCB */
#define MCF_ESW_NDISCB_COUNT(x)                (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_POQC */
#define MCF_ESW_POQC_COUNT(x)                  (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_PMVID */
#define MCF_ESW_PMVID_COUNT(x)                 (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_PMVTAG */
#define MCF_ESW_PMVTAG_COUNT(x)                (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_PBL */
#define MCF_ESW_PBL_COUNT(x)                   (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_ISR */
#define MCF_ESW_ISR_EBERR                      (0x00000001)
#define MCF_ESW_ISR_RXB                        (0x00000002)
#define MCF_ESW_ISR_RXF                        (0x00000004)
#define MCF_ESW_ISR_TXB                        (0x00000008)
#define MCF_ESW_ISR_TXF                        (0x00000010)
#define MCF_ESW_ISR_QM                         (0x00000020)
#define MCF_ESW_ISR_OD0                        (0x00000040)
#define MCF_ESW_ISR_OD1                        (0x00000080)
#define MCF_ESW_ISR_OD2                        (0x00000100)
#define MCF_ESW_ISR_LRN                        (0x00000200)

/* Bit definitions and macros for MCF_ESW_IMR */
#define MCF_ESW_IMR_EBERR                      (0x00000001)
#define MCF_ESW_IMR_RXB                        (0x00000002)
#define MCF_ESW_IMR_RXF                        (0x00000004)
#define MCF_ESW_IMR_TXB                        (0x00000008)
#define MCF_ESW_IMR_TXF                        (0x00000010)
#define MCF_ESW_IMR_QM                         (0x00000020)
#define MCF_ESW_IMR_OD0                        (0x00000040)
#define MCF_ESW_IMR_OD1                        (0x00000080)
#define MCF_ESW_IMR_OD2                        (0x00000100)
#define MCF_ESW_IMR_LRN                        (0x00000200)

/* Bit definitions and macros for MCF_ESW_RDSR */
#define MCF_ESW_RDSR_ADDRESS(x)                (((x)&0x3FFFFFFF)<<2)

/* Bit definitions and macros for MCF_ESW_TDSR */
#define MCF_ESW_TDSR_ADDRESS(x)                (((x)&0x3FFFFFFF)<<2)

/* Bit definitions and macros for MCF_ESW_MRBR */
#define MCF_ESW_MRBR_SIZE(x)                   (((x)&0x000003FF)<<4)

/* Bit definitions and macros for MCF_ESW_RDAR */
#define MCF_ESW_RDAR_R_DES_ACTIVE              (0x01000000)

/* Bit definitions and macros for MCF_ESW_TDAR */
#define MCF_ESW_TDAR_X_DES_ACTIVE              (0x01000000)

/* Bit definitions and macros for MCF_ESW_LREC0 */
#define MCF_ESW_LREC0_MACADDR0(x)              (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_ESW_LREC1 */
#define MCF_ESW_LREC1_MACADDR1(x)              (((x)&0x0000FFFF)<<0)
#define MCF_ESW_LREC1_HASH(x)                  (((x)&0x000000FF)<<16)
#define MCF_ESW_LREC1_SWPORT(x)                (((x)&0x00000003)<<24)

/* Bit definitions and macros for MCF_ESW_LSR */
#define MCF_ESW_LSR_DA                         (0x00000001)

/* port mirroring port number match */
#define MIRROR_EGRESS_PORT_MATCH		1
#define MIRROR_INGRESS_PORT_MATCH		2

/* port mirroring mac address match */
#define MIRROR_EGRESS_SOURCE_MATCH		1
#define MIRROR_INGRESS_SOURCE_MATCH		2
#define MIRROR_EGRESS_DESTINATION_MATCH		3
#define MIRROR_INGRESS_DESTINATION_MATCH	4

/*-------------ioctl command ---------------------------------------*/
#define ESW_SET_LEARNING_CONF               0x9101
#define ESW_GET_LEARNING_CONF               0x9201
#define ESW_SET_BLOCKING_CONF               0x9102
#define ESW_GET_BLOCKING_CONF               0x9202
#define ESW_SET_MULTICAST_CONF              0x9103
#define ESW_GET_MULTICAST_CONF              0x9203
#define ESW_SET_BROADCAST_CONF              0x9104
#define ESW_GET_BROADCAST_CONF              0x9204
#define ESW_SET_PORTENABLE_CONF             0x9105
#define ESW_GET_PORTENABLE_CONF             0x9205
#define ESW_SET_IP_SNOOP_CONF               0x9106
#define ESW_GET_IP_SNOOP_CONF               0x9206
#define ESW_SET_PORT_SNOOP_CONF             0x9107
#define ESW_GET_PORT_SNOOP_CONF             0x9207
#define ESW_SET_PORT_MIRROR_CONF	    0x9108
#define ESW_GET_PORT_MIRROR_CONF            0x9208
#define ESW_SET_PIRORITY_VLAN               0x9109
#define ESW_GET_PIRORITY_VLAN               0x9209
#define ESW_SET_PIRORITY_IP                 0x910A
#define ESW_GET_PIRORITY_IP                 0x920A
#define ESW_SET_PIRORITY_MAC                0x910B
#define ESW_GET_PIRORITY_MAC                0x920B
#define ESW_SET_PIRORITY_DEFAULT            0x910C
#define ESW_GET_PIRORITY_DEFAULT            0x920C
#define ESW_SET_P0_FORCED_FORWARD           0x910D
#define ESW_GET_P0_FORCED_FORWARD           0x920D
#define ESW_SET_SWITCH_MODE                 0x910E
#define ESW_GET_SWITCH_MODE                 0x920E
#define ESW_SET_BRIDGE_CONFIG               0x910F
#define ESW_GET_BRIDGE_CONFIG               0x920F
#define ESW_SET_VLAN_OUTPUT_PROCESS         0x9110
#define ESW_GET_VLAN_OUTPUT_PROCESS         0x9210
#define ESW_SET_VLAN_INPUT_PROCESS          0x9111
#define ESW_GET_VLAN_INPUT_PROCESS          0x9211
#define ESW_SET_VLAN_DOMAIN_VERIFICATION    0x9112
#define ESW_GET_VLAN_DOMAIN_VERIFICATION    0x9212
#define ESW_SET_VLAN_RESOLUTION_TABLE       0x9113
#define ESW_GET_VLAN_RESOLUTION_TABLE       0x9213
#define ESW_GET_ENTRY_PORT_NUMBER	    0x9214
#define ESW_GET_LOOKUP_TABLE		    0x9215
#define ESW_GET_PORT_STATUS	    	    0x9216
#define ESW_SET_VLAN_ID			    0x9114
#define ESW_SET_VLAN_ID_CLEARED		    0x9115
#define ESW_SET_PORT_IN_VLAN_ID	    	    0x9116
#define ESW_SET_PORT_ENTRY_EMPTY            0x9117
#define ESW_SET_OTHER_PORT_ENTRY_EMPTY      0x9118
#define ESW_GET_PORT_ALL_STATUS		    0x9217
#define ESW_SET_PORT_MIRROR_CONF_PORT_MATCH 0x9119
#define ESW_SET_PORT_MIRROR_CONF_ADDR_MATCH 0x911A

#define ESW_GET_STATISTICS_STATUS           0x9221
#define ESW_SET_OUTPUT_QUEUE_MEMORY         0x9125
#define ESW_GET_OUTPUT_QUEUE_STATUS         0x9225
#define ESW_UPDATE_STATIC_MACTABLE          0x9226
#define ESW_CLEAR_ALL_MACTABLE              0x9227
#define ESW_GET_USER_PID                    0x9228

#endif /* SWITCH_H */
