/*
 * based on arch/arm/mach-mx6/mx6_fec.c
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/fec.h>
#include <linux/etherdevice.h>

#include <mach/common.h>
#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include "devices-mvf.h"

#define HW_OCOTP_MACn(n)        (0x00000620 + (n) * 0x10)

static int fec_get_mac_addr(unsigned char *mac)
{
	unsigned int value;
#if 0
#if 1
	value = 0x01;
#endif
	//value = readl(MVF_IO_ADDRESS(MVF_OTP_CTRL_BASE_ADDR) + HW_OCOTP_MACn(0));
	mac[5] = value & 0xff;
	mac[4] = (value >> 8) & 0xff;
	mac[3] = (value >> 16) & 0xff;
	mac[2] = (value >> 24) & 0xff;
	//value = readl(MVF_IO_ADDRESS(MVF_OTP_CTRL_BASE_ADDR) + HW_OCOTP_MACn(1));
	mac[1] = value & 0xff;
	mac[0] = (value >> 8) & 0xff;
#endif
	mac[5]=0x00;
	mac[4]=0x21;
	mac[3]=0x97;
	mac[2]=0x09;
	mac[1]=0xc6;
	mac[0]=0xae;

	return 0;
}

void __init mvf_init_fec(struct fec_platform_data fec_data)
{
	fec_get_mac_addr(fec_data.mac);
	if (!is_valid_ether_addr(fec_data.mac))
		random_ether_addr(fec_data.mac);

	vf6xx_add_fec(&fec_data);
}
