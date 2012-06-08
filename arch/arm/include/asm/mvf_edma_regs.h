/* mvf_edma_regs.h
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All rights reserved.
 * Lanttor.Guo@freescale.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _MVF_EDMA_REG_H__
#define _MVF_EDMA_REG_H__

#define MVF_REG32(x)	*((unsigned long *)x)
#define MVF_REG16(x)	*((unsigned short *)x)
#define MVF_REG08(x)	*((unsigned char *)x)


/*
 * Enhanced DMA (EDMA)
 */

/* Channels */
#define MVF_EDMA_CHAN_DREQ0	0	/* External DMA request 0 */
#define MVF_EDMA_CHAN_DREQ1	1	/* External DMA request 1 */
#define MVF_EDMA_CHAN_UART0_RX	2	/* UART0 Receive */
#define MVF_EDMA_CHAN_UART0_TX	3	/* UART0 Transmit */
#define MVF_EDMA_CHAN_UART1_RX	4	/* UART1 Receive */
#define MVF_EDMA_CHAN_UART1_TX	5	/* UART1 Transmit */
#define MVF_EDMA_CHAN_UART2_RX	6	/* UART2 Receive */
#define MVF_EDMA_CHAN_UART2_TX	7	/* UART2 Transmit */
#define MVF_EDMA_CHAN_TIMER0	8	/* Timer 0 / SSI0 Rx */
#define MVF_EDMA_CHAN_TIMER1	9	/* Timer 1 / SSI1 Rx */
#define MVF_EDMA_CHAN_TIMER2	10	/* Timer 2 / SSI0 Tx */
#define MVF_EDMA_CHAN_TIMER3	11	/* Timer 3 / SSI1 Tx */
#define MVF_EDMA_CHAN_DSPI0_RX	12	/* DSPI0 Receive */
#define MVF_EDMA_CHAN_DSPI0_TX	13	/* DSPI0 Transmit */
#define MVF_EDMA_CHAN_DSPI1_RX	14	/* DSPI1 Receive */
#define MVF_EDMA_CHAN_DSPI1_TX	15	/* DSPI1 Transmit */
#define MVF_EDMA_CHAN_UART3_RX	16	/* UART3 Receive */
#define MVF_EDMA_CHAN_UART3_TX	17	/* UART3 Transmit */
#define MVF_EDMA_CHAN_UART4_RX	18	/* UART4 Receive */
#define MVF_EDMA_CHAN_UART4_TX	19	/* UART4 Transmit */
#define MVF_EDMA_CHAN_UART5_RX	20	/* UART5 Receive */
#define MVF_EDMA_CHAN_UART5_TX	21	/* UART5 Transmit */
#define MVF_EDMA_CHAN_UART6_RX	22	/* UART6 Receive */
#define MVF_EDMA_CHAN_UART6_TX	23 	/* UART6 Transmit */
#define MVF_EDMA_CHAN_I2C0	24	/* I2C0 */
#define MVF_EDMA_CHAN_I2C1	25	/* I2C1 */
#define MVF_EDMA_CHAN_I2C2	26	/* I2C2 */
#define MVF_EDMA_CHAN_I2C3	27	/* I2C3 */
#define MVF_EDMA_CHAN_DSPI2_RX	28	/* DSPI2 Receive */
#define MVF_EDMA_CHAN_DSPI2_TX	29	/* DSPI2 Transmit */
#define MVF_EDMA_CHAN_N0	30	/* Available for software */
#define MVF_EDMA_CHAN_N1	31	/* Available for software */
#define MVF_EDMA_CHAN_UART7_RX	32	/* UART7 Receive */
#define MVF_EDMA_CHAN_UART7_TX	33	/* UART7 Transmit */
#define MVF_EDMA_CHAN_UART8_RX	34	/* UART8 Receive */
#define MVF_EDMA_CHAN_UART8_TX	35	/* UART8 Transmit */
#define MVF_EDMA_CHAN_UART9_RX	36	/* UART9 Receive */
#define MVF_EDMA_CHAN_UART9_TX	37	/* UART9 Transmit */
#define MVF_EDMA_CHAN_OW	38	/* 1-Wire */
#define MVF_EDMA_CHAN_RESERVED	39	/* Reserved */
#define MVF_EDMA_CHAN_I2C4	40	/* I2C4 */
#define MVF_EDMA_CHAN_I2C5	41	/* I2C5 */
#define MVF_EDMA_CHAN_N2	42	/* Available for software */
#define MVF_EDMA_CHAN_N3	43	/* Available for software */
#define MVF_EDMA_CHAN_DSPI3_RX	44	/* DSPI3 Receive */
#define MVF_EDMA_CHAN_DSPI3_TX	45	/* DSPI3 Transmit */
#define MVF_EDMA_CHAN_SSI0_RX0	48	/* SSI0 Receive 0 */
#define MVF_EDMA_CHAN_SSI0_RX1	49	/* SSI0 Receive 1 */
#define MVF_EDMA_CHAN_SSI0_TX0	50	/* SSI0 Transmit 0 */
#define MVF_EDMA_CHAN_SSI0_TX1	51	/* SSI0 Transmit 1 */
#define MVF_EDMA_CHAN_SSI1_RX0	52	/* SSI1 Receive 0 */
#define MVF_EDMA_CHAN_SSI1_RX1	53	/* SSI1 Receive 1 */
#define MVF_EDMA_CHAN_SSI1_TX0	54	/* SSI1 Transmit 0 */
#define MVF_EDMA_CHAN_SSI1_TX1	55	/* SSI1 Transmit 1 */
#define MVF_EDMA_CHAN_PWM_CAP	56	/* PWM Capture */
#define MVF_EDMA_CHAN_PWM_VAL	57	/* PWM Value */
#define MVF_EDMA_CHAN_RESERVED2 58	/* Reserved */
#define MVF_EDMA_CHAN_ESDHC	59	/* eSDHC */
#define MVF_EDMA_CHAN_ADC0	60	/* ADC 0 */
#define MVF_EDMA_CHAN_ADC1	61	/* ADC 1 */
#define MVF_EDMA_CHAN_DAC0	62	/* DAC 0 */
#define MVF_EDMA_CHAN_DAC1	63	/* DAC 1 */

/* Register read/write macros */
/*	offset 0x0000_0000 - 0x0000_00ff  main dma control area	*/
#define MVF_EDMA_CR(base)							MVF_REG32((long)(base) + 0x00000000)
#define MVF_EDMA_ES(base)							MVF_REG32((long)(base) + 0x00000004)
//#define MVF_EDMA_ERQH(base)							MVF_REG32((long)(base) + 0x00000008)
#define MVF_EDMA_ERQ(base)							MVF_REG32((long)(base) + 0x0000000C)
//#define MVF_EDMA_EEIH(base)							MVF_REG32((long)(base) + 0x00000010)
#define MVF_EDMA_EEI(base)							MVF_REG32((long)(base) + 0x00000014)
#define MVF_EDMA_SERQ(base)							MVF_REG08((long)(base) + 0x00000008)
#define MVF_EDMA_CERQ(base)							MVF_REG08((long)(base) + 0x00000019)
#define MVF_EDMA_SEEI(base)							MVF_REG08((long)(base) + 0x0000001A)
#define MVF_EDMA_CEEI(base)							MVF_REG08((long)(base) + 0x0000001B)
#define MVF_EDMA_CINT(base)							MVF_REG08((long)(base) + 0x0000001C)
#define MVF_EDMA_CERR(base)							MVF_REG08((long)(base) + 0x0000001D)
#define MVF_EDMA_SSRT(base)							MVF_REG08((long)(base) + 0x0000001E)
#define MVF_EDMA_CDNE(base)							MVF_REG08((long)(base) + 0x0000001F)
//#define MVF_EDMA_INTH(base)							MVF_REG32((long)(base) + 0x00000020)
#define MVF_EDMA_INT(base)							MVF_REG32((long)(base) + 0x00000024)
//#define MVF_EDMA_ERRH(base)							MVF_REG32((long)(base) + 0x00000028)
#define MVF_EDMA_ERR(base)							MVF_REG32((long)(base) + 0x0000002C)
//#define MVF_EDMA_RSH(base)							MVF_REG32((long)(base) + 0x00000030)
#define MVF_EDMA_RS(base)							MVF_REG32((long)(base) + 0x00000034)

/* Parameterized register read/write macros for multiple registers */
/*	offset 0x0000_0100 - 0x0000_011f  dma channel priority area	*/
#define MVF_EDMA_DCHPRI(base,x)						MVF_REG08((long)(base) + 0x00000100 +((x)*0x001))


/*	offset 0x0000_1000 - 0x0000_13ff  tcd area	*/
#define MVF_EDMA_TCD_SADDR(base,x)					MVF_REG32((long)(base) + 0x00001000 +((x)*0x020))
#define MVF_EDMA_TCD_ATTR(base,x)					MVF_REG16((long)(base) + 0x00001004 +((x)*0x020))
#define MVF_EDMA_TCD_SOFF(base,x)					MVF_REG16((long)(base) + 0x00001006 +((x)*0x020))
#define MVF_EDMA_TCD_NBYTES(base,x)					MVF_REG32((long)(base) + 0x00001008 +((x)*0x020))
#define MVF_EDMA_TCD_SLAST(base,x)					MVF_REG32((long)(base) + 0x0000100C +((x)*0x020))
#define MVF_EDMA_TCD_DADDR(base,x)					MVF_REG32((long)(base) + 0x00001010 +((x)*0x020))
#define MVF_EDMA_TCD_CITER_ELINK(base,x)			MVF_REG16((long)(base) + 0x00001014 +((x)*0x020))
#define MVF_EDMA_TCD_CITER(base, x)					MVF_REG16((long)(base) + 0x00001014 +((x)*0x020))
#define MVF_EDMA_TCD_DOFF(base,x)					MVF_REG16((long)(base) + 0x00001016 +((x)*0x020))
#define MVF_EDMA_TCD_DLAST_SGA(base, x)				MVF_REG32((long)(base) + 0x00001018 +((x)*0x020))
#define MVF_EDMA_TCD_BITER_ELINK(base,x)			MVF_REG16((long)(base) + 0x0000101C +((x)*0x020))
#define MVF_EDMA_TCD_BITER(base, x)					MVF_REG16((long)(base) + 0x0000101C +((x)*0x020))
#define MVF_EDMA_TCD_CSR(base,x)					MVF_REG16((long)(base) + 0x0000101e +((x)*0x020))

/* Bit definitions and macros for CR */
#define MVF_EDMA_CR_EDBG							(0x00000002)
#define MVF_EDMA_CR_ERCA							(0x00000004)
#define MVF_EDMA_CR_ERGA							(0x00000008)
#define MVF_EDMA_CR_HOE								(0x00000010)
#define MVF_EDMA_CR_HALT							(0x00000020)
#define MVF_EDMA_CR_CLM		(0x00000040)
#define MVF_EDMA_CR_EMLM	(0x00000080)
#define MVF_EDMA_CR_GRP0PRI(x)	(((x)&0x03)<<8)
#define MVF_EDMA_CR_GRP1PRI(x)	(((x)&0x03)<<10)
#define MVF_EDMA_CR_GRP2PRI(x)	(((x)&0x03)<<12)
#define MVF_EDMA_CR_GRP3PRI(x)	(((x)&0x03)<<14)
#define MVF_EDMA_CR_ECX		(0x00010000)
#define MVF_EDMA_CR_CX		(0x00020000)

/* Bit definitions and macros for ES */
#define MVF_EDMA_ES_DBE         (0x00000001)
#define MVF_EDMA_ES_SBE         (0x00000002)
#define MVF_EDMA_ES_SGE         (0x00000004)
#define MVF_EDMA_ES_NCE         (0x00000008)
#define MVF_EDMA_ES_DOE         (0x00000010)
#define MVF_EDMA_ES_DAE         (0x00000020)
#define MVF_EDMA_ES_SOE         (0x00000040)
#define MVF_EDMA_ES_SAE         (0x00000080)
#define MVF_EDMA_ES_ERRCHN(x)   (((x)&0x0000003F)<<8)
#define MVF_EDMA_ES_CPE         (0x00004000)
#define MVF_EDMA_ES_GPE         (0x00008000)
#define MVF_EDMA_ES_ECX         (0x00010000)
#define MVF_EDMA_ES_VLD         (0x80000000)

/* Bit definitions and macros for ERQ: 0~63 bits */
#define MVF_EDMA_ERQ_ERQH(x)	(0x01<<x)	/*32~63*/
#define MVF_EDMA_ERQ_ERQL(x)	(0x01<<x)	/*0~31*/

/* Bit definitions and macros for EEI: 0~63 bits */
#define MVF_EDMA_EEI_EEIH(x)	(0x01<<x)	/*32~63*/
#define MVF_EDMA_EEI_EEIL(x)	(0x01<<x)	/*0~31*/

/* Bit definitions and macros for SERQ */
#define MVF_EDMA_SERQ_SERQ(x)	(((x)&0x3F))
#define MVF_EDMA_SERQ_SAER	(0x40)
#define MVF_EDMA_SERQ_NOP	(0x80)

/* Bit definitions and macros for CERQ */
#define MVF_EDMA_CERQ_CERQ(x)	(((x)&0x3F))
#define MVF_EDMA_CERQ_CAER	(0x40)
#define MVF_EDMA_CERQ_NOP	(0x80)

/* Bit definitions and macros for SEEI */
#define MVF_EDMA_SEEI_SEEI(x)	(((x)&0x3F))
#define MVF_EDMA_SEEI_SAEE	(0x40)
#define MVF_EDMA_SEEI_NOP	(0x80)

/* Bit definitions and macros for CEEI */
#define MVF_EDMA_CEEI_CEEI(x)	(((x)&0x3F))
#define MVF_EDMA_CEEI_CAEE	(0x40)
#define MVF_EDMA_CEEI_NOP	(0x80)

/* Bit definitions and macros for CINT */
#define MVF_EDMA_CINT_CINT(x)	(((x)&0x3F))
#define MVF_EDMA_CINT_CAIR	(0x40)
#define MVF_EDMA_CINT_NOP	(0x80)

/* Bit definitions and macros for CERR */
#define MVF_EDMA_CERR_CERR(x)   (((x)&0x3F))
#define MVF_EDMA_CERR_CAER      (0x40)
#define MVF_EDMA_CERR_NOP	(0x80)

/* Bit definitions and macros for SSRT */
#define MVF_EDMA_SSRT_SSRT(x)   (((x)&0x3F))
#define MVF_EDMA_SSRT_SAST      (0x40)
#define MVF_EDMA_SSRT_NOP	(0x80)

/* Bit definitions and macros for CDNE */
#define MVF_EDMA_CDNE_CDNE(x)	(((x)&0x3F))
#define MVF_EDMA_CDNE_CADN	(0x40)
#define MVF_EDMA_CDNE_NOP	(0x80)

/* Bit definitions and macros for INTR: 0~63 bits */
#define MVF_EDMA_INTR_INTH(x)	(0x01<<x)	/*32~63*/
#define MVF_EDMA_INTR_INTL(x)	(0x01<<x)	/*0~31*/

/* Bit definitions and macros for ERR: 0~63 bits */
#define MVF_EDMA_ERR_ERRH(x)	(0x01<<x)	/*32~63*/
#define MVF_EDMA_ERR_ERRL(x)	(0x01<<x)	/*0~31*/

/* Bit defineitions and macros for HRSH/HRSL */
#define MVF_EDMA_HRS_HRSH(x)	(0x01<<x)	/*32~63*/
#define MVF_EDMA_HRS_HRSL(x)	(0x01<<x)	/*0~31*/

/* Bit definitions and macros for DCHPRI group */
#define MVF_EDMA_DCHPRI_CHPRI(x)	(((x)&0x0F))
#define MVF_EDMA_DCHPRI_GRPPRI(x)	(((x)&0x03) << 4)
#define MVF_EDMA_DCHPRI_DPA		(0x40)
#define MVF_EDMA_DCHPRI_ECP		(0x80)

/* Bit definitions and macros for TCD_SADDR group */
#define MVF_EDMA_TCD_SADDR_SADDR(x)     (x)

/* Bit definitions and macros for TCD_ATTR group */
#define MVF_EDMA_TCD_ATTR_DSIZE(x)          (((x)&0x0007))
#define MVF_EDMA_TCD_ATTR_DMOD(x)           (((x)&0x001F)<<3)
#define MVF_EDMA_TCD_ATTR_SSIZE(x)          (((x)&0x0007)<<8)
#define MVF_EDMA_TCD_ATTR_SMOD(x)           (((x)&0x001F)<<11)
#define MVF_EDMA_TCD_ATTR_SSIZE_8BIT        (0x0000)
#define MVF_EDMA_TCD_ATTR_SSIZE_16BIT       (0x0100)
#define MVF_EDMA_TCD_ATTR_SSIZE_32BIT       (0x0200)
#define MVF_EDMA_TCD_ATTR_SSIZE_16BYTE      (0x0400)
#define MVF_EDMA_TCD_ATTR_DSIZE_8BIT        (0x0000)
#define MVF_EDMA_TCD_ATTR_DSIZE_16BIT       (0x0001)
#define MVF_EDMA_TCD_ATTR_DSIZE_32BIT       (0x0002)
#define MVF_EDMA_TCD_ATTR_DSIZE_16BYTE      (0x0004)

/* Bit definitions and macros for TCD_SOFF group */
#define MVF_EDMA_TCD_SOFF_SOFF(x)   (x)

/* Bit definitions and macros for TCD_NBYTES group */
#define MVF_EDMA_TCD_NBYTES_NBYTES(x)   (x)
#define MVF_EDMA_TCD_NBYTES_SMLOE	(0x80000000)
#define MVF_EDMA_TCD_NBYTES_DMLOE	(0x40000000)
#define MVF_EDMA_TCD_NBYTES_MLOFF(x)	(((x)&0xFFFFF)<<20)
#define MVF_EDMA_TCD_NBYTES_9BITS	((x)&0x1FF)

/* Bit definitions and macros for TCD_SLAST group */
#define MVF_EDMA_TCD_SLAST_SLAST(x)     (x)

/* Bit definitions and macros for TCD_DADDR group */
#define MVF_EDMA_TCD_DADDR_DADDR(x)     (x)

/* Bit definitions and macros for TCD_CITER_ELINK group */
#define MVF_EDMA_TCD_CITER_ELINK_CITER(x)       (((x)&0x01FF))
#define MVF_EDMA_TCD_CITER_ELINK_LINKCH(x)      (((x)&0x003F)<<9)
#define MVF_EDMA_TCD_CITER_ELINK_E_LINK         (0x8000)

/* Bit definitions and macros for TCD_CITER group */
#define MVF_EDMA_TCD_CITER_CITER(x)     (((x)&0x7FFF))
#define MVF_EDMA_TCD_CITER_E_LINK       (0x8000)

/* Bit definitions and macros for TCD_DOFF group */
#define MVF_EDMA_TCD_DOFF_DOFF(x)   (x)

/* Bit definitions and macros for TCD_DLAST_SGA group */
#define MVF_EDMA_TCD_DLAST_SGA_DLAST_SGA(x)     (x)

/* Bit definitions and macros for TCD_BITER_ELINK group */
#define MVF_EDMA_TCD_BITER_ELINK_BITER(x)       (((x)&0x01FF))
#define MVF_EDMA_TCD_BITER_ELINK_LINKCH(x)      (((x)&0x003F)<<9)
#define MVF_EDMA_TCD_BITER_ELINK_E_LINK         (0x8000)

/* Bit definitions and macros for TCD_BITER group */
#define MVF_EDMA_TCD_BITER_BITER(x)     (((x)&0x7FFF))
#define MVF_EDMA_TCD_BITER_E_LINK       (0x8000)

/* Bit definitions and macros for TCD_CSR group */
#define MVF_EDMA_TCD_CSR_START              (0x0001)
#define MVF_EDMA_TCD_CSR_INT_MAJOR          (0x0002)
#define MVF_EDMA_TCD_CSR_INT_HALF           (0x0004)
#define MVF_EDMA_TCD_CSR_D_REQ              (0x0008)
#define MVF_EDMA_TCD_CSR_E_SG               (0x0010)
#define MVF_EDMA_TCD_CSR_E_LINK             (0x0020)
#define MVF_EDMA_TCD_CSR_ACTIVE             (0x0040)
#define MVF_EDMA_TCD_CSR_DONE               (0x0080)
#define MVF_EDMA_TCD_CSR_LINKCH(x)          (((x)&0x003F)<<8)
#define MVF_EDMA_TCD_CSR_BWC(x)             (((x)&0x0003)<<14)
#define MVF_EDMA_TCD_CSR_BWC_NO_STALL       (0x0000)
#define MVF_EDMA_TCD_CSR_BWC_4CYC_STALL     (0x8000)
#define MVF_EDMA_TCD_CSR_BWC_8CYC_STALL     (0xC000)

/* Bit definitions and macros for TCD0_CSR */
#define MVF_EDMA_TCD0_CSR_START             (0x0001)
#define MVF_EDMA_TCD0_CSR_INT_MAJOR         (0x0002)
#define MVF_EDMA_TCD0_CSR_INT_HALF          (0x0004)
#define MVF_EDMA_TCD0_CSR_D_REQ             (0x0008)
#define MVF_EDMA_TCD0_CSR_E_SG              (0x0010)
#define MVF_EDMA_TCD0_CSR_E_LINK            (0x0020)
#define MVF_EDMA_TCD0_CSR_ACTIVE            (0x0040)
#define MVF_EDMA_TCD0_CSR_DONE              (0x0080)
#define MVF_EDMA_TCD0_CSR_LINKCH(x)         (((x)&0x003F)<<8)
#define MVF_EDMA_TCD0_CSR_BWC(x)            (((x)&0x0003)<<14)
#define MVF_EDMA_TCD0_CSR_BWC_NO_STALL      (0x0000)
#define MVF_EDMA_TCD0_CSR_BWC_4CYC_STALL    (0x8000)
#define MVF_EDMA_TCD0_CSR_BWC_8CYC_STALL    (0xC000)

#endif
