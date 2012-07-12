/****************************************************************************/
/*
 *  L2 switch Controller (Etheren switch) driver for VF600.
 *  based on L2 switch Controller for MCF5441x.
 *		   processors.
 *
 *  Copyright (C) 2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/****************************************************************************/
#ifndef MVF_SWITCH_H
#define	MVF_SWITCH_H
/****************************************************************************/
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <asm/pgtable.h>

/*
 * Some hardware gets it MAC address out of local flash memory.
 * if this is non-zero then assume it is the address to get MAC from.
 */
#define FEC_FLASHMAC    0

#ifdef CONFIG_SWITCH_DMA_USE_SRAM
#define TX_RING_SIZE            8      /* Must be power of two */
#define TX_RING_MOD_MASK        7      /*   for this to work */
#else
#define TX_RING_SIZE            16      /* Must be power of two */
#define TX_RING_MOD_MASK        15      /*   for this to work */
#endif

#define SWITCH_EPORT_NUMBER	2

//	2.5MHz + HOLD time
#define MVF_MII_SWITCH_SPEED	((0x09<<1)|((uint)0x100))


//	register offset for fec
#define FEC_R_CNTRL		0x084 /* Receive control reg */
#define FEC_X_CNTRL		0x0c4 /* Transmit Control reg */
#define FEC_IEVENT		0x004 /* Interrupt event reg */
#define FEC_IMASK		0x008 
#define FEC_MII_DATA		0x040 /* MII manage frame reg */
#define FEC_MII_SPEED		0x044 /* MII speed control reg */
#define FEC_ECNTRL		0x024 /* Ethernet control reg */

/*-----------------------------------------------------------------------*/
typedef struct l2switch_output_queue_status {
	unsigned long ESW_MMSR;
	unsigned long ESW_LMT;
	unsigned long ESW_LFC;
	unsigned long ESW_PCSR;
	unsigned long ESW_IOSR;
	unsigned long ESW_QWT;
	unsigned long esw_reserved;
	unsigned long ESW_P0BCT;
} esw_output_queue_status;

typedef struct l2switch_statistics_status {
	/*
	 * Total number of incoming frames processed
	 * but discarded in switch
	 */
	unsigned long ESW_DISCN;
	/*Sum of bytes of frames counted in ESW_DISCN*/
	unsigned long ESW_DISCB;
	/*
	 * Total number of incoming frames processed
	 * but not discarded in switch
	 */
	unsigned long ESW_NDISCN;
	/*Sum of bytes of frames counted in ESW_NDISCN*/
	unsigned long ESW_NDISCB;
} esw_statistics_status;

typedef struct l2switch_port_statistics_status {
	/*outgoing frames discarded due to transmit queue congestion*/
	unsigned long MCF_ESW_POQC;
	/*incoming frames discarded due to VLAN domain mismatch*/
	unsigned long MCF_ESW_PMVID;
	/*incoming frames discarded due to untagged discard*/
	unsigned long MCF_ESW_PMVTAG;
	/*incoming frames discarded due port is in blocking state*/
	unsigned long MCF_ESW_PBL;
} esw_port_statistics_status;

typedef struct l2switch {
	//	0x00-0x34
	unsigned long ESW_REVISION;
	unsigned long ESW_SCRATCH;
	unsigned long ESW_PER;
	unsigned long reserved0[1];
	//	0x10
	unsigned long ESW_VLANV;
	unsigned long ESW_DBCR;
	unsigned long ESW_DMCR;
	unsigned long ESW_BKLR;
	//	0x20
	unsigned long ESW_BMPC;
	unsigned long ESW_MODE;
	unsigned long ESW_VIMSEL;
	unsigned long ESW_VOMSEL;
	//	0x30
	unsigned long ESW_VIMEN;
	unsigned long ESW_VID;
	unsigned long esw_reserved0[2];

	//	0x40
	unsigned long ESW_MCR;
	unsigned long ESW_EGMAP;
	unsigned long ESW_INGMAP;
	unsigned long ESW_INGSAL;

	//	0x50
	unsigned long ESW_INGSAH;
	unsigned long ESW_INGDAL;
	unsigned long ESW_INGDAH;
	unsigned long ESW_ENGSAL;

	//	0x60
	unsigned long ESW_ENGSAH;
	unsigned long ESW_ENGDAL;
	unsigned long ESW_ENGDAH;
	unsigned long ESW_MCVAL;
	/*from 0x70--0x7C*/
	unsigned long esw_reserved1[4];

	//	0x80
	unsigned long ESW_MMSR;
	unsigned long ESW_LMT;
	unsigned long ESW_LFC;
	unsigned long ESW_PCSR;

	//	0x90
	unsigned long ESW_IOSR;
	unsigned long ESW_QWT;
	unsigned long esw_reserved2[1];
	unsigned long ESW_P0BCT;

	//	0xa0
	/*from 0xA0-0xB8*/
	unsigned long esw_reserved3[7];
	unsigned long ESW_P0FFEN;

	//	0xc0-0xdf
	unsigned long ESW_PSNP[8];

	//	0xe0-0xff
	unsigned long ESW_IPSNP[8];

	//	0x100-0x13f
	unsigned long ESW_PVRES[3];
	unsigned long esw_reserved4[13];

	//	0x140
	unsigned long ESW_IPRES;
	unsigned long esw_reserved5[15];

	unsigned long ESW_PRES[3];
	unsigned long esw_reserved6[29];

	unsigned long ESW_PID[3];
	unsigned long esw_reserved7[29];

	unsigned long ESW_VRES[32];

	unsigned long ESW_DISCN;
	unsigned long ESW_DISCB;
	unsigned long ESW_NDISCN;
	unsigned long ESW_NDISCB;
	esw_port_statistics_status port_statistics_status[3];
	unsigned long esw_reserved8[48];

	/*unsigned long MCF_ESW_ISR;*/
	unsigned long   switch_ievent;             /* Interrupt event reg */
	/*unsigned long MCF_ESW_IMR;*/
	unsigned long   switch_imask;              /* Interrupt mask reg */
	/*unsigned long MCF_ESW_RDSR;*/
	unsigned long   fec_r_des_start;        /* Receive descriptor ring */
	/*unsigned long MCF_ESW_TDSR;*/
	unsigned long   fec_x_des_start;        /* Transmit descriptor ring */
	/*unsigned long MCF_ESW_MRBR;*/
	unsigned long   fec_r_buff_size;        /* Maximum receive buff size */
	/*unsigned long MCF_ESW_RDAR;*/
	unsigned long   fec_r_des_active;       /* Receive descriptor reg */
	/*unsigned long MCF_ESW_TDAR;*/
	unsigned long   fec_x_des_active;       /* Transmit descriptor reg */
	unsigned long esw_reserved9[57];

	unsigned long ESW_LREC0;
	unsigned long ESW_LREC1;
	unsigned long ESW_LSR;
} switch_t;

typedef struct _64bTableEntry {
	unsigned int lo;  /* lower 32 bits */
	unsigned int hi;  /* upper 32 bits */
} AddrTable64bEntry;

typedef struct l2switchaddrtable {
	AddrTable64bEntry  eswTable64bEntry[2048];
} eswAddrTable_t;


#define MCF_FEC_RCR_PROM                     (0x00000008)
#define MCF_FEC_RCR_RMII_MODE                (0x00000104)
#define MCF_FEC_RCR_MAX_FL(x)                (((x)&0x00003FFF)<<16)
#define MCF_FEC_RCR_CRC_FWD                  (0x00004000)
#define MCF_FEC_TCR_FDEN                     (0x00000004)
#define MCF_FEC_ECR_ETHER_EN                 (0x00000002)
#define MCF_FEC_ECR_ENA_1588                 (0x00000010)
#define MCF_FEC_ECR_SWAP                 	 (0x00000100)

typedef struct _eswIOCTL_PORT_CONF {
	int port;
	int enable;
} eswIoctlPortConfig;

typedef struct _eswIOCTL_PORT_EN_CONF {
	int port;
	int tx_enable;
	int rx_enable;
} eswIoctlPortEnableConfig;

typedef struct _eswIOCTL_IP_SNOOP_CONF {
	int mode;
	unsigned long ip_header_protocol;
} eswIoctlIpsnoopConfig;

typedef struct _eswIOCTL_P0_FORCED_FORWARD_CONF {
	int port1;
	int port2;
	int enable;
} eswIoctlP0ForcedForwardConfig;

typedef struct _eswIOCTL_PORT_SNOOP_CONF {
     int mode;
     unsigned short compare_port;
     int compare_num;
} eswIoctlPortsnoopConfig;

typedef struct _eswIOCTL_PORT_Mirror_CONF {
	int mirror_port;
	int port;
	int egress_en;
	int ingress_en;
	int egress_mac_src_en;
	int egress_mac_des_en;
	int ingress_mac_src_en;
	int ingress_mac_des_en;
	unsigned char *src_mac;
	unsigned char *des_mac;
	int mirror_enable;
} eswIoctlPortMirrorConfig;

struct eswIoctlMirrorCfgPortMatch {
	int mirror_port;
	int port_match_en;
	int port;
};

struct eswIoctlMirrorCfgAddrMatch {
	int mirror_port;
	int addr_match_en;
	unsigned char *mac_addr;
};

typedef struct _eswIOCTL_PRIORITY_VLAN_CONF {
	int port;
	int func_enable;
	int vlan_pri_table_num;
	int vlan_pri_table_value;
} eswIoctlPriorityVlanConfig;

typedef struct _eswIOCTL_PRIORITY_IP_CONF {
	int port;
	int func_enable;
	int ipv4_en;
	int ip_priority_num;
	int ip_priority_value;
} eswIoctlPriorityIPConfig;

typedef struct _eswIOCTL_PRIORITY_MAC_CONF {
	int port;
} eswIoctlPriorityMacConfig;

typedef struct _eswIOCTL_PRIORITY_DEFAULT_CONF {
	int port;
	unsigned char priority_value;
} eswIoctlPriorityDefaultConfig;

typedef struct _eswIOCTL_IRQ_STATUS {
	unsigned long isr;
	unsigned long imr;
	unsigned long rx_buf_pointer;
	unsigned long tx_buf_pointer;
	unsigned long rx_max_size;
	unsigned long rx_buf_active;
	unsigned long tx_buf_active;
} eswIoctlIrqStatus;

typedef struct _eswIOCTL_PORT_Mirror_STATUS {
	unsigned long ESW_MCR;
	unsigned long ESW_EGMAP;
	unsigned long ESW_INGMAP;
	unsigned long ESW_INGSAL;
	unsigned long ESW_INGSAH;
	unsigned long ESW_INGDAL;
	unsigned long ESW_INGDAH;
	unsigned long ESW_ENGSAL;
	unsigned long ESW_ENGSAH;
	unsigned long ESW_ENGDAL;
	unsigned long ESW_ENGDAH;
	unsigned long ESW_MCVAL;
} eswIoctlPortMirrorStatus;

typedef struct _eswIOCTL_VLAN_OUTPUT_CONF {
	int port;
	int mode;
} eswIoctlVlanOutputConfig;

typedef struct _eswIOCTL_VLAN_INPUT_CONF {
	int port;
	int mode;
	unsigned short port_vlanid;
} eswIoctlVlanInputConfig;

typedef struct _eswIOCTL_VLAN_DOMAIN_VERIFY_CONF {
	int port;
	int vlan_domain_verify_en;
	int vlan_discard_unknown_en;
} eswIoctlVlanVerificationConfig;

typedef struct _eswIOCTL_VLAN_RESOULATION_TABLE {
	unsigned short port_vlanid;
	unsigned char vlan_domain_port;
	unsigned char vlan_domain_num;
} eswIoctlVlanResoultionTable;

struct eswVlanTableItem {
	eswIoctlVlanResoultionTable table[32];
	unsigned char valid_num;
};

typedef struct _eswIOCTL_VLAN_INPUT_STATUS {
	unsigned long ESW_VLANV;
	unsigned long ESW_PID[3];
	unsigned long ESW_VIMSEL;
	unsigned long ESW_VIMEN;
	unsigned long ESW_VRES[32];
} eswIoctlVlanInputStatus;

typedef struct _eswIOCTL_Static_MACTable {
	unsigned char *mac_addr;
	int port;
	int priority;
} eswIoctlUpdateStaticMACtable;

typedef struct _eswIOCTL_OUTPUT_QUEUE{
	int fun_num;
	esw_output_queue_status  sOutputQueue;
} eswIoctlOutputQueue;
/*
 * Info received from Hardware Learning FIFO,
 * holding MAC address and corresponding Hash Value and
 * port number where the frame was received (disassembled).
 */
typedef struct _eswPortInfo {
	/* MAC lower 32 bits (first byte is 7:0). */
	unsigned int   maclo;
	/* MAC upper 16 bits (47:32). */
	unsigned int   machi;
	/* the hash value for this MAC address. */
	unsigned int   hash;
	/* the port number this MAC address is associated with. */
	unsigned int   port;
} eswPortInfo;

/*
 * Hardware Look up Address Table 64-bit element.
 */
typedef volatile struct _64bitTableEntry {
	unsigned int lo;  /* lower 32 bits */
	unsigned int hi;  /* upper 32 bits */
} eswTable64bitEntry;

struct eswAddrTableEntryExample {
	/* the entry number */
	unsigned short entrynum;
	/* mac address array */
	unsigned char mac_addr[6];
	unsigned char item1;
	unsigned short item2;
};

/*
 *	Define the buffer descriptor structure.
 */
#if defined(CONFIG_ARCH_MXC) 
typedef struct bufdesc {
	unsigned short	cbd_datlen;		/* Data length */
	unsigned short	cbd_sc;			/* Control and status info */
	unsigned long	cbd_bufaddr;		/* Buffer address */
#if defined(CONFIG_FEC_1588)
	unsigned long   ebd_status;
	unsigned long 	cbd_prot;
	unsigned long   bdu;
	unsigned long   timestamp;
	unsigned long   reserverd_word1;
	unsigned long   reserverd_word2;
#endif
} cbd_t;
#else
typedef struct bufdesc {
	unsigned short	cbd_sc;			/* Control and status info */
	unsigned short	cbd_datlen;		/* Data length */
	unsigned long	cbd_bufaddr;		/* Buffer address */
#if defined(CONFIG_FEC_1588)
	unsigned long   ebd_status;
	unsigned short  length_proto_type;
	unsigned short  payload_checksum;
	unsigned long   bdu;
	unsigned long   timestamp;
	unsigned long   reserverd_word1;
	unsigned long   reserverd_word2;
#endif
} cbd_t;
#endif

/* Forward declarations of some structures to support different PHYs
 */
typedef struct {
	uint mii_data;
	void (*funct)(uint mii_reg, struct net_device *dev);
} phy_cmd_t;

typedef struct {
	uint id;
	char *name;

	const phy_cmd_t *config;
	const phy_cmd_t *startup;
	const phy_cmd_t *ack_int;
	const phy_cmd_t *shutdown;
} phy_info_t;

struct port_status {
	/* 1: link is up, 0: link is down */
	int port1_link_status;
	int port2_link_status;
	/* 1: blocking, 0: unblocking */
	int port0_block_status;
	int port1_block_status;
	int port2_block_status;
};

struct port_all_status {
	/* 1: link is up, 0: link is down */
	int link_status;
	/* 1: blocking, 0: unblocking */
	int block_status;
	/* 1: unlearning, 0: learning */
	int learn_status;
	/* vlan domain verify 1: enable 0: disable */
	int vlan_verify;
	/* discard unknow 1: enable 0: disable */
	int discard_unknown;
	/* multicast resolution 1: enable 0: disable */
	int multi_reso;
	/* broadcast resolution 1: enable 0: disalbe */
	int broad_reso;
	/* transmit 1: enable 0: disable */
	int ftransmit;
	/* receive 1: enable 0: disable */
	int freceive;
};

/* The switch buffer descriptors track the ring buffers.  The rx_bd_base and
 * tx_bd_base always point to the base of the buffer descriptors.  The
 * cur_rx and cur_tx point to the currently available buffer.
 * The dirty_tx tracks the current buffer that is being sent by the
 * controller.  The cur_tx and dirty_tx are equal under both completely
 * empty and completely full conditions.  The empty/ready indicator in
 * the buffer descriptor determines the actual condition.
 */
struct switch_enet_private {
	/* Hardware registers of the switch device */
	volatile switch_t  *hwp;
	volatile eswAddrTable_t  *hwentry;

	void __iomem *fec[2];

	struct net_device *netdev;
	struct platform_device *pdev;
	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	unsigned char *tx_bounce[TX_RING_SIZE];
	struct  sk_buff *tx_skbuff[TX_RING_SIZE];
	ushort  skb_cur;
	ushort  skb_dirty;

	/* CPM dual port RAM relative addresses.
	 */
	cbd_t   *rx_bd_base;            /* Address of Rx and Tx buffers. */
	cbd_t   *tx_bd_base;
	cbd_t   *cur_rx, *cur_tx;               /* The next free ring entry */
	cbd_t   *dirty_tx;      /* The ring entries to be free()ed. */
	uint    tx_full;
	/* hold while accessing the HW like ringbuffer for tx/rx but not MAC */
	spinlock_t hw_lock;

	/* hold while accessing the mii_list_t() elements */
	spinlock_t mii_lock;
	struct mii_bus *mdio_bus;
	struct phy_device *phydev[SWITCH_EPORT_NUMBER];

	uint    phy_id;
	uint    phy_id_done;
	uint    phy_status;
	uint    phy_speed;
	phy_info_t const        *phy;
	struct work_struct phy_task;
	volatile switch_t  *phy_hwp;

	uint    sequence_done;
	uint    mii_phy_task_queued;

	uint    phy_addr;

	int     index;
	int     opened;
	int     full_duplex;
	int     msg_enable;
	int     phy1_link;
	int     phy1_old_link;
	int     phy1_duplex;
	int     phy1_speed;

	int     phy2_link;
	int     phy2_old_link;
	int     phy2_duplex;
	int     phy2_speed;
	/* --------------Statistics--------------------------- */
	/* when a new element deleted a element with in
	 * a block due to lack of space */
	int atBlockOverflows;
	/* Peak number of valid entries in the address table */
	int atMaxEntries;
	/* current number of valid entries in the address table */
	int atCurrEntries;
	/* maximum entries within a block found
	 * (updated within ageing)*/
	int atMaxEntriesPerBlock;

	/* -------------------ageing function------------------ */
	/* maximum age allowed for an entry */
	int ageMax;
	/* last LUT entry to block that was
	 * inspected by the Ageing task*/
	int ageLutIdx;
	/* last element within block inspected by the Ageing task */
	int ageBlockElemIdx;
	/* complete table has been processed by ageing process */
	int ageCompleted;
	/* delay setting */
	int ageDelay;
	/* current delay Counter */
	int  ageDelayCnt;

	/* ----------------timer related---------------------------- */
	/* current time (for timestamping) */
	int currTime;
	/* flag set by timer when currTime changed
	 * and cleared by serving function*/
	int timeChanged;

	/**/
	/* Timer for Aging */
	struct timer_list       timer_aging;
	int learning_irqhandle_enable;

	dma_addr_t	bd_dma;

};

struct switch_platform_private {
	struct platform_device  *pdev;
	struct clk *fec0,*fec1,*l2sw;

	unsigned long           quirks;
	int                     num_slots;      /* Slots on controller */
	struct switch_enet_private *fep_host[0];      /* Pointers to hosts */
};
#endif
