/*
 * l2switch.c
 *
 * Sub-architcture dependant initialization code for the Freescale
 * MVF family L2 Switch module.
 *
 * based on 5441X.
 *
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * ShrekWu B16972@freescale.com
 *
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

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <asm/traps.h>
#include <asm/mvf_switch.h>
#include <mach/hardware.h>
#include <asm/mach/arch.h>

#if (defined(CONFIG_SOC_IMX28) || defined(CONFIG_ARCH_MX6)) || defined(CONFIG_ARCH_MVF) \
				&& defined(CONFIG_FEC_1588)
#define CONFIG_ENHANCED_BD
#endif


//	base address
//#define FEC_ETH0		0x400D0000
//#define FEC_ETH1		0x400D1000
//#define L2SWITCH_1		0x400E8000
#define L2SWITCH_ATBL	0x400EC000

static unsigned char    switch_mac_default[] = {
//	0x00, 0x04, 0x9F, 0x00, 0xB3, 0x49,
	0xae, 0xc6, 0x09, 0x97, 0x21, 0x01,
};

static unsigned char switch_mac_addr[6];

static void switch_request_intrs(struct net_device *dev,
	irqreturn_t switch_net_irq_handler(int irq, void *private),
	void *irq_privatedata)
{
	struct switch_enet_private *fep;
	int b;
	static const struct idesc {
		char *name;
		unsigned short irq;
	} *idp, id[] = {
		{ "esw_isr", MXC_INT_ENET_SWITCH},
		{ NULL },
	};

	fep = netdev_priv(dev);
	/*intrruption L2 ethernet SWITCH */

	/* Setup interrupt handlers. */
	for (idp = id; idp->name; idp++) {
		if (request_irq(idp->irq,
			switch_net_irq_handler, 0,idp->name, irq_privatedata) != 0)
			printk(KERN_ERR "FEC: Could not alloc %s IRQ(%d)!\n",
				idp->name, idp->irq);
		printk(KERN_INFO "L2 Switch: %s IRQ(%d)  installed.!\n", idp->name, idp->irq);
	}
}

static void switch_set_mii(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	volatile switch_t *fecp;

	writel((MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE | MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD), fep->fec[0] + FEC_R_CNTRL);
	writel((MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE | MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD), fep->fec[1] + FEC_R_CNTRL);

	writel(MCF_FEC_TCR_FDEN, fep->fec[0] + FEC_X_CNTRL);
	writel(MCF_FEC_TCR_FDEN, fep->fec[1] + FEC_X_CNTRL);

#ifdef CONFIG_ENHANCED_BD
	writel(MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588 |MCF_FEC_ECR_SWAP, fep->fec[0] + FEC_ECNTRL);
	writel(MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588 |MCF_FEC_ECR_SWAP, fep->fec[1] + FEC_ECNTRL);
#else
	writel(MCF_FEC_ECR_ETHER_EN |MCF_FEC_ECR_SWAP, fep->fec[0] + FEC_ECNTRL);
	writel(MCF_FEC_ECR_ETHER_EN |MCF_FEC_ECR_SWAP, fep->fec[1] + FEC_ECNTRL);
#endif
	writel( MVF_MII_SWITCH_SPEED, fep->fec[0] + FEC_MII_SPEED);
	writel( MVF_MII_SWITCH_SPEED, fep->fec[1] + FEC_MII_SPEED);

}

static void switch_get_mac(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	volatile switch_t *fecp;
	unsigned char *iap;

	fecp = fep->hwp;

	if (FEC_FLASHMAC) {
		/*
		* Get MAC address from FLASH.
		* If it is all 1's or 0's, use the default.
		*/
		iap = FEC_FLASHMAC;
		if ((iap[0] == 0) && (iap[1] == 0) && (iap[2] == 0) &&
			(iap[3] == 0) && (iap[4] == 0) && (iap[5] == 0))
			iap = switch_mac_default;
		if ((iap[0] == 0xff) && (iap[1] == 0xff) &&
			(iap[2] == 0xff) && (iap[3] == 0xff) &&
			(iap[4] == 0xff) && (iap[5] == 0xff))
			iap = switch_mac_default;

	} else {
		iap = &switch_mac_addr[0];

		if ((iap[0] == 0) && (iap[1] == 0) && (iap[2] == 0) &&
			(iap[3] == 0) && (iap[4] == 0) && (iap[5] == 0))
			iap = switch_mac_default;
		if ((iap[0] == 0xff) && (iap[1] == 0xff) &&
			(iap[2] == 0xff) && (iap[3] == 0xff) &&
			(iap[4] == 0xff) && (iap[5] == 0xff))
			iap = switch_mac_default;
	}

	memcpy(dev->dev_addr, iap, ETH_ALEN);
	/* Adjust MAC if using default MAC address */
	if (iap == switch_mac_default)
		dev->dev_addr[ETH_ALEN-1] = switch_mac_default[ETH_ALEN-1] +
						fep->index;
}

static void switch_enable_phy_intr(void)
{
}

static void switch_disable_phy_intr(void)
{
}

static void switch_phy_ack_intr(void)
{
}

static void switch_localhw_setup(void)
{
}

static void switch_uncache(unsigned long addr)
{
}

static void switch_platform_flush_cache(void)
{
}

/*
 * Define the fixed address of the FEC hardware.
 */
static unsigned int switch_platform_hw[] = {
	MVF_ETH_L2_SW_BASE_ADDR,
	L2SWITCH_ATBL,
};

static unsigned int fec_platform_hw[] = {
	MVF_ENET0_IEEE1588_BASE_ADDR,
	MVF_ENET1_IEEE1588_BASE_ADDR,
};

static struct mvf_switch_platform_data mvf_switch_data = {
	.hash_table = 0,
	.fec_hw = fec_platform_hw,
	.switch_hw = switch_platform_hw,
	.request_intrs = switch_request_intrs,
	.set_mii = switch_set_mii,
	.get_mac = switch_get_mac,
	.enable_phy_intr = switch_enable_phy_intr,
	.disable_phy_intr = switch_disable_phy_intr,
	.phy_ack_intr = switch_phy_ack_intr,
	.localhw_setup = switch_localhw_setup,
	.uncache = switch_uncache,
	.platform_flush_cache = switch_platform_flush_cache,
};

//	non-used-structure
static struct resource l2switch_resources[] = {
	[0] = {
		.start  = MVF_ETH_L2_SW_BASE_ADDR,
		.end    = MVF_ETH_L2_SW_BASE_ADDR+0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = MXC_INT_ENET_SWITCH,
		.end    = MXC_INT_ENET_SWITCH,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = AIPS1_OFF_BASE_ADDR + 0x4F000,
		.end    = AIPS1_OFF_BASE_ADDR + 0x4F000 + 0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device l2switch_mvf_device = {
	.name = "mvf-switch",
	.id = 0,
	.resource = l2switch_resources,
	.num_resources = ARRAY_SIZE(l2switch_resources),
	.dev = {
		.platform_data = &mvf_switch_data,
		.coherent_dma_mask = ~0,        /* $$$ REVISIT */
	}
};


static int __init mvf_switch_dev_init(void)
{
	int retval = 0;

	retval = platform_device_register(&l2switch_mvf_device);

	if (retval < 0) {
		printk(KERN_ERR "MVF L2Switch: platform_device_register"
				" failed with code=%d\n", retval);
	}

	return retval;
}

static int __init param_switch_addr_setup(char *str)
{
	char *end;
	int i;
#if 0
	for (i = 0; i < 6; i++) {
		switch_mac_addr[i] = str ? simple_strtoul(str, &end, 16) : 0;
		if (str)
			str = (*end ) ? end + 1 : end;
	}
#endif
	return 0;
}
__setup("switchaddr=", param_switch_addr_setup);

arch_initcall(mvf_switch_dev_init);
